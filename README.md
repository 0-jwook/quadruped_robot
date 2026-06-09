# Quadruped Robot — ROS 2 시스템 가이드

12-DOF 사족보행 로봇의 ROS 2 (Humble) 측 구성 + 운용 가이드.
다른 세션 / 다른 AI 에이전트가 빠르게 컨텍스트를 잡을 수 있도록 작성됨.

---

## 1. 시스템 구조 한눈에 보기

```
[Raspberry Pi: ROS 2 Humble]                  [STM32F103RB MCU]
─────────────────────────────                 ──────────────────
  teleop_twist_keyboard
        ↓ /cmd_vel (Twist)
  gait_node (50 Hz)
   ├ BezierGait 알고리즘
   ├ IK (각 다리)
   └ stand/walk FSM
        ↓ /joint_trajectory_controller/joint_trajectory
  hardware_bridge
   ├ rad → deg + SERVO_TRIMS 보정
   ├ CRC8 패킷 생성
   └ /imu publisher (MCU에서 받은 IMU)
        ↓ UART 115200 (USB ST-Link 경유 /dev/ttyACM0)
  STM32 (ros_com)
   ├ CRC8 패킷 파싱
   ├ heartbeat 송신
   └ PCA9685.SetAngle(channel, deg)
        ↓ I2C1 (PB6 SCL, PB7 SDA)
  PCA9685 (16-channel PWM driver)
        ↓ 16 PWM 채널 (50 Hz)
  12개 서보 (SG90/MG90s)
```

## 2. 디렉토리 구조

```
/home/jaewook/Quardruped/
├── src/
│   ├── quadruped_gait/            ← 핵심: 보행 알고리즘 + 하드웨어 브릿지
│   │   └── quadruped_gait/
│   │       ├── gait_node.py        ROS 노드 (50Hz timer, cmd_vel 수신)
│   │       ├── gait_planner.py     BezierGait 알고리즘 + 자동 게이트 전환
│   │       ├── kinematics.py       각 다리 3-DOF IK (도메인 위반 clamp)
│   │       └── hardware_bridge.py  rad→deg + SERVO_TRIMS + CRC8 packet → UART
│   ├── quadruped_bringup/
│   │   └── launch/
│   │       ├── hardware.launch.py   실제 로봇용 (gait_node + hardware_bridge)
│   │       └── sim.launch.py        Gazebo 시뮬용
│   ├── quadruped_description/      URDF + xacro
│   └── quadruped_control/          ros2_control 설정 + 시뮬용 mcu_bridge
├── tools/
│   └── calibrate.py                인터랙티브 캘리브레이션 (standalone)
└── README.md                       (이 문서)
```

## 3. 핵심 알고리즘: BezierGait

### 출처
- **moribots/spot_mini_mini** (SpotMicroAI 추천 코드) 의 BezierGait 를 Python으로 포팅
- 원본 논문: MIT Cheetah (12-point Bezier swing + sinusoidal stance)

### 동작 원리

1. **Phase tracking**: time 기반. `time_since_last_TD` + 다리별 `dSref` (phase lag) 로 각 다리의 swing/stance 결정
2. **Swing 궤적**: 12-point Bernstein polynomial (Bezier 곡선) — 발이 부드럽게 올라갔다 내려옴
3. **Stance 궤적**: `step = L * (1 - 2*phase)` 선형 + cosine penetration

### Phase 분배 (dSref)

| 게이트 | FL | FR | RL | RR | 비고 |
|--------|-----|-----|-----|-----|------|
| **trot** | 0.0 | 0.5 | 0.5 | 0.0 | 대각선 쌍 동기 |
| **wave (8phase)** | 0.0 | 0.5 | 0.75 | 0.25 | 한 다리씩 swing |

### 자동 게이트 전환 (gait_planner.py)

`gait_type='trot'` 모드에서:
- 일반 명령 (전진/후진/회전/호 회전) → **trot** 게이트
- **측방 이동** (`|vy| > 0.01`, `|vx| < 0.005`, 게다리 모션) → **wave** 자동 전환

이유: trot 은 측방에 본질적으로 약함 (좌/우 같은 쪽 두 다리가 동시에 한 방향으로 → roll 진동).
wave 는 한 다리씩 옮겨 항상 3-leg 지지 → 측방에 robust.
제자리 회전은 yaw_step 으로 trot 게이트 내에서 처리.

### Duty Factor

| 게이트 | tstance_min_ratio | duty | 비행 구간 |
|--------|------------------|------|---------|
| trot (현재) | 3.0 | 0.75 | 없음 ✓ |
| wave | 2.8 | 0.74 | 없음 ✓ |

mike4192/spotMicro 의 4-phase trot 과 동일한 duty 0.75 확보 (overlap phase 대신 ratio 키워서).

### 회전 처리 (`_yaw_step`)

- 수평 변위만 담당 (clearance/penetration = 0)
- 발 들기는 메인 `_bezier_swing` 이 일관되게 담당
- 좌우 비대칭 보정: 회전 방향에 따라 "body 앞으로 미는" 쪽 다리 stride 15% 강화 (FORWARD_BOOST)

## 4. 파라미터 (launch 파일)

`src/quadruped_bringup/launch/hardware.launch.py` 의 gait_node 파라미터:

```python
'L1': 0.030,             # 어깨 hip offset (m)
'L2': 0.115,             # 허벅지 (m)
'L3': 0.135,             # 정강이 (m)
'body_height': 0.17,     # stand 시 어깨~지면 거리 (m)
'step_height': 0.03,     # Bezier swing 최대 발 들기 높이 (m)
'max_stride':  0.035,    # half-stride L 의 상한 (m, 보폭 = 2L)
'period':      0.55,     # trot Tswing = period/2 = 0.275s, wave Tswing = period/4 = 0.1375s
'height_min':  0.07,     # body_height_cmd 의 최소값 (앉기 가능)
'height_max':  0.21,
'gait_type':   'trot',   # 'trot' (측방 시 자동 wave 전환) 또는 '8phase' (항상 wave)
'cmd_vel_hold_time': 30.0,  # cmd_vel 받은 뒤 stand 자세 복귀까지 시간 (s)
'pitch_offset': 0.015,   # 가상 pitch 보정 (rad). + = 앞 들기, - = 앞 내림
'roll_offset':  0.015,   # 가상 roll 보정  (rad). + = 우측 들기, - = 좌측 들기
```

### 안전 clamp (gait_planner 내부)

- `MAX_LIN = 0.15 m/s`  (cmd_vel.linear)
- `MAX_ANG = 0.6 rad/s` (cmd_vel.angular)

teleop_twist_keyboard 기본 0.5 m/s 가 너무 빠르기 때문.

## 5. ROS 토픽

### 구독 (gait_node)

| 토픽 | 메시지 | 용도 |
|------|--------|------|
| `/cmd_vel` | geometry_msgs/Twist | 속도 명령 |
| `/imu` | sensor_msgs/Imu | (IMU 설치 시) 자세 피드백 |
| `/body_height_cmd` | std_msgs/Float32 | body 높이 명령 (앉기/일어서기) |

### 발행 (gait_node)

| 토픽 | 메시지 | 용도 |
|------|--------|------|
| `/joint_trajectory_controller/joint_trajectory` | trajectory_msgs/JointTrajectory | 12 관절 각도 (radian) |

### hardware_bridge

- 수신: `/joint_trajectory_controller/joint_trajectory` → SERVO_TRIMS + CRC8 → UART 송신
- 발행: `/imu` (MCU에서 받은 IMU 데이터를 Quaternion으로 변환)

## 6. 캘리브레이션 (SERVO_TRIMS)

`hardware_bridge.py` 상단에 다리별 trim 값 (조립 비대칭 보정):

```python
SERVO_TRIMS = {
    'FL': ( 1.0,  19.0, -13.0),   # (hip, thigh, calf) 도 단위 보정
    'FR': ( 7.0, -28.0,   4.0),
    'RL': ( 0.0,  22.0,  -8.0),
    'RR': ( 6.0,  -8.0,  15.0),
}
```

### 캘리브 도구

```bash
# 1. ROS 노드 종료
# 2. 인터랙티브 캘리브:
python3 tools/calibrate.py [/dev/ttyACM0] [--pose home|stand]
```

키: `d`/`a` 다리 이동, `s`/`w` 관절, `+`/`-` ±1°, `]`/`[` ±5°, `p` 출력, `q` 종료.

기본 모드는 **STAND 자세** (보행 자세에서 캘리브). `--pose home` 으로 다리 쫙 편 자세 캘리브도 가능.

스크립트는 hardware_bridge.py 의 현재 SERVO_TRIMS 와 launch 의 pitch_offset 을 정규식으로 자동 로드 → 추가 미세조정만 하면 됨.

## 7. 실행 방법

### 빌드
```bash
cd ~/Quardruped
colcon build --packages-select quadruped_gait quadruped_bringup
source install/setup.bash
```

### 하드웨어 모드
```bash
ros2 launch quadruped_bringup hardware.launch.py
# 다른 포트:
ros2 launch quadruped_bringup hardware.launch.py port:=/dev/ttyUSB0
```

### 시뮬레이션 (Gazebo)
```bash
ros2 launch quadruped_bringup sim.launch.py
```

### 텔레옵 (별도 터미널)
```bash
source ~/Quardruped/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| 키 | 동작 |
|----|------|
| `i` / `,` | 전진 / 후진 |
| `j` (소문자) / `l` | 좌회전 / 우회전 (제자리 → wave 자동 전환) |
| `J` (Shift+j) / `L` | 좌측 / 우측 평행이동 |
| `u` / `o` | 좌사선 / 우사선 전진 (호 회전) |
| `k` 또는 스페이스 | 정지 |
| `q` / `z` | 전체 속도 ±10% |

### 앉기 / 일어서기
```bash
ros2 topic pub --once /body_height_cmd std_msgs/Float32 "data: 0.07"   # 앉기
ros2 topic pub --once /body_height_cmd std_msgs/Float32 "data: 0.17"   # 일어서기
```

## 8. 통신 프로토콜 (ROS ↔ MCU)

### ROS → MCU (joint command)

```
[0xAA 0x55] [0x03] [0x30=48] [12×float32 (degrees, 0~180)] [CRC8]
  헤더       ID    LEN      payload (48 bytes)              체크섬
```

- CRC8: polynomial 0x07, init 0x00 — `hardware_bridge.py:_crc8()` 와 MCU `CRC8Update()` 동일
- 각도 순서: `[FL.hip, FL.thigh, FL.calf, FR.hip, ..., RR.calf]`

### MCU → ROS (텍스트, line-based)

| 메시지 | 형식 | 의미 |
|--------|------|------|
| IMU | `IMU:<roll>,<pitch>,<yaw>` (도) | 자세 데이터 (IMU 설치 시) |
| Heartbeat | `HB:<tick>,CRC:<n>,ERR:<n>,PKT:<n>,WDG:<n>,TO:<n>` | 1Hz 진단 |
| Error | `[ERROR] <msg>` | MCU 에러 |

## 9. 현재 캘리브 상태

```python
SERVO_TRIMS = {  # 도 단위 — hardware_bridge.py 상단
    'FL': ( 1.0,  19.0, -13.0),
    'FR': ( 7.0, -28.0,   4.0),
    'RL': ( 0.0,  22.0,  -8.0),
    'RR': ( 6.0,  -8.0,  15.0),
}
```

큰 thigh 비대칭(FR -28, RL +22) 은 실제 조립 오차를 SW 로 보정한 결과.
잔여 우측-앞 기울임은 `roll_offset`/`pitch_offset` 로 추가 보정 중 (launch 파일).

## 10. 알려진 한계 / 향후 개선

| 항목 | 상태 | 비고 |
|------|------|------|
| 우측-앞쪽 기울임 | ⚠️ 잔여 | `roll_offset`, `pitch_offset` 으로 일부 보정 (각 0.015 rad) |
| 전진 약함 / 후진 강함 | ⚠️ 잔여 | 하드웨어 비대칭 (무게중심, 발 마찰) 의심 |
| 측방 보행 | ✅ 해결 | wave 자동 전환으로 안정 |
| 회전 시 backward drift | ⚠️ 잔여 | `yaw_step` 의 FORWARD_BOOST=1.15 로 보정 |
| 패킷 손실 (PKT:16~32) | ⚠️ 가끔 | 서보 노이즈 → I2C BUSY → 메인 루프 블로킹 |
| IMU 피드백 | ❌ 미설치 | 코드는 있음, 센서 안 달면 dz=offset 만 작용 |
| Body shift (8phase) | ❌ 미구현 | mike4192 의 안정성 비결. 추가 가능 |
| MPC / PID 자세 제어 | ❌ | SG90/MG90s 위치 제어 모터 한계 |

## 11. 참조 코드 (학습용)

| 위치 | 출처 | 알고리즘 |
|------|------|---------|
| `/home/jaewook/spotMicro` | mike4192/spot_micro | tick-phase + 적분 stance + 삼각형 swing + body shift |
| `/home/jaewook/spot_mini_mini` | moribots/spot_mini_mini | time-phase + sin stance + **12-point Bezier swing** (← 우리 알고리즘 출처) |

## 12. PCA9685 채널 매핑 (MCU 측)

`Core/Src/Quadruped.cpp` 의 `JOINT_CHANNELS[4][3]` 배열. 현재:

| 다리 | hip | thigh | calf |
|------|-----|-------|------|
| FL (앞 좌) | 8 | 9 | 10 |
| FR (앞 우) | 12 | 13 | 14 |
| RL (뒤 좌) | 4 | 5 | 6 |
| RR (뒤 우) | 0 | 1 | 2 |

→ 채널 3, 7, 11, 15 는 미사용.

## 13. Git 저장소

- **ROS**: https://github.com/0-jwook/quadruped_robot (브랜치: main)
- **MCU**: https://github.com/0-jwook/quadruped_robot_MCU (브랜치: master)

## 14. 문제 해결 빠른 진단

| 증상 | 의심 항목 | 확인 / 처방 |
|------|----------|------------|
| 로봇 안 움직임 | 시리얼 포트 / MCU 전원 | `ls -l /dev/ttyACM*`, MCU LED |
| heartbeat PKT 낮음 | I2C 노이즈로 메인 루프 블로킹 | 서보 전원 caps, 풀업 점검 |
| 좌측/우측 휨 | 좌우 비대칭 (캘리브 잔여) | `tools/calibrate.py` 재실행 또는 yaw_step `FORWARD_BOOST` 조정 |
| 우측 또는 앞 기울임 | 자세 보정 부족 | launch 의 `roll_offset` / `pitch_offset` ± 키움 |
| 회전 시 뒤로 | 좌우 추진력 비대칭 | `FORWARD_BOOST` 키움 |
| 발 끌림 | step_height 너무 낮음 | `step_height` 0.03 → 0.045 |
| 너무 빠름 | period 짧음 | `period` 키움 |
| 너무 느림 | period 김 | `period` 줄임 |
| 비행 구간 / 흔들림 | duty < 0.5 | `tstance_min_ratio` 키움 (현재 3.0, duty 0.75) |
| 측방 보행 불안 | 게이트 자동 전환 안 됨 | `gait_type='trot'` 확인 |
