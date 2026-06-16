# Quadruped Robot — ROS 2 시스템 가이드

12-DOF 사족보행 로봇의 ROS 2 (Humble) 측 구성 + 운용 가이드.
다른 세션 / 다른 AI 에이전트가 빠르게 컨텍스트를 잡을 수 있도록 작성됨.

---

## 1. 시스템 구조 한눈에 보기

```
[Raspberry Pi: ROS 2 Humble]                  [STM32F103RB MCU]
─────────────────────────────                 ──────────────────
  teleop_key (또는 웹 게임패드)
        ↓ /cmd_vel /body_pose /gesture /body_height_cmd
  gait_node (50 Hz, 모드 관리자)
   ├ 모드: GESTURE > BODY_POSE > WALK > STAND
   ├ BezierGait (통합 회전 운동학 + 고정 duty + CoM shift)
   ├ 시작 ramp(SIT→STAND), 넘어짐 감지/자동기립, leveling
   └ IK (각 다리)
        ↓ /joint_trajectory_controller/joint_trajectory (rad)
  hardware_bridge
   ├ rad → deg + SERVO_TRIMS(config/servo_trims.yaml)
   ├ CRC8 패킷 생성
   └ /imu publisher (MCU IMU 라인 수신 시)
        ↓ UART 115200 (/dev/ttyACM0)
  STM32 → CRC8 파싱 / heartbeat / PCA9685.SetAngle
        ↓ I2C → PCA9685 → 12개 서보 (SG90/MG90s)
```

> ⚠️ **`/cmd_vel` 소스는 한 번에 하나만** (teleop ↔ 웹 게임패드 동시 금지). 둘이 같이 발행하면 한쪽의 0이 다른 쪽 명령을 덮어써 전진이 막힘. 다중 입력이 필요하면 `twist_mux` 권장.

## 2. 디렉토리 구조

```
~/robot/quadruped_robot/
├── src/
│   ├── quadruped_gait/                ← 핵심: 보행 + 모드 + 하드웨어 브릿지
│   │   ├── quadruped_gait/
│   │   │   ├── gait_node.py            모드 관리 ROS 노드 (50Hz, ramp/넘어짐감지)
│   │   │   ├── gait_planner.py         BezierGait (통합 회전 + 고정 duty + CoM/leveling)
│   │   │   ├── kinematics.py           3-DOF IK/FK (도달범위 clamp)
│   │   │   ├── body_pose.py            발 고정 몸통 6축 Body IK
│   │   │   ├── gestures.py             제스처 11종 키프레임 + 재생기
│   │   │   ├── hardware_bridge.py      rad→deg + SERVO_TRIMS + CRC8 → UART, /imu 발행
│   │   │   └── teleop_key.py           통합 키보드 텔레옵 (walk/pose/gesture)
│   │   └── config/servo_trims.yaml     서보 트림 (calibrate.py 가 기록, bridge 가 로드)
│   ├── quadruped_bringup/launch/
│   │   ├── hardware.launch.py          실제 로봇 (gait_node + hardware_bridge)
│   │   └── sim.launch.py               Gazebo 시뮬용
│   ├── quadruped_description/          URDF + xacro
│   └── quadruped_control/              ros2_control 설정 + 시뮬용 mcu_bridge
├── tools/calibrate.py                  인터랙티브 서보 캘리브 (standalone, yaml 자동 저장)
├── prd/
│   ├── gemini.md                       초기 시뮬레이션 PRD
│   └── web_gamepad.md                  웹 게임패드 원격조종 PRD (별도 개발)
└── README.md                           (이 문서)
```

## 3. 핵심 알고리즘: BezierGait

### 출처
- **moribots/spot_mini_mini** 의 BezierGait 를 Python 포팅. 원본: MIT Cheetah (12-point Bezier swing).

### 핵심 설계
1. **순수 modular 위상**: `pos = (time/Tstride − dSref[i]) % 1.0`, `pos < duty` 면 stance. 기준다리 앵커 없이 시간만으로 결정 → 네 다리 완전 대칭 (직진 휨 알고리즘 원인 제거).
2. **Swing 궤적**: 12-point Bernstein(Bezier) — 발이 부드럽게 올라갔다 내림.
3. **Stance 궤적**: 선형 후퇴 + 사인 penetration (발끌림 0).
4. **통합 회전 운동학**: 각 발 지면속도 `v_foot = v_body + ω × r_foot` → stride 벡터. 병진+회전이 자연 합성(호 회전), 별도 yaw 보정상수 불필요. `r_foot` 는 `hip_x/hip_y`(몸통중심~발) 사용.
5. **고정 duty**: `Tstance = duty·period`, `Tswing = (1−duty)·period`. duty≥0.5 면 비행구간 없음. stride 가 `max_stride` 초과 시 명령을 scale-down → 고속에서도 비행 없음.
6. **CoM shift**: wave 시 지지발 centroid 로 몸통 LPF 이동(정적 안정). trot 대각 지지는 centroid≈0 → 자연 감쇠.
7. **기하 leveling**: IMU roll/pitch 로 각 발 z 보정해 몸통 수평 유지 (`level_gain`, 최대 30° 경사). **IMU 데이터 필요**.

### Phase 분배 (dSref) / Duty
| 게이트 | FL | FR | RL | RR | duty | 비행 |
|--------|----|----|----|----|------|------|
| **trot** | 0.0 | 0.5 | 0.5 | 0.0 | `duty_trot`=0.6 | 없음 |
| **wave** | 0.0 | 0.5 | 0.75 | 0.25 | `duty_wave`=0.75 | 없음 |

### 게이트 자동 전환
- 기본 **trot**. 측방 우세(`|vy|>|vx|`) 또는 제자리 회전 시 **wave** 로 cycle 경계에서 전환.
- 단, 현재 운용은 아래 "동작 제약" 참고 — **제자리 회전·측방은 비활성**이라 실질적으로 trot 위주.

### 동작 제약 (운용 정책)
- **회전(ω)은 병진(전진/후진) 중에만 적용** (gait_node) → 제자리 회전 비활성, **전진+회전 = 호 회전**(trot).
- **측방 이동(q/e)** 은 teleop 에서 제거 (실기 미동작).

## 4. 파라미터 (`hardware.launch.py` → gait_node)

```python
'L1':0.030, 'L2':0.115, 'L3':0.135,   # 다리 링크 (m)
'body_height': 0.14,     # stand 높이 (m)
'step_height': 0.035,    # swing 발 들기 높이 (m)
'max_stride':  0.05,     # 발 stride 벡터 상한 (속도 상한 결정)
'period':      0.9,      # 전체 cycle Tstride (s). 최고속도 = max_stride/(duty·period) ≈ 0.093 m/s
'duty_trot':   0.6,      # trot 접지율 (≥0.5 → 비행 없음)
'duty_wave':   0.75,     # wave 접지율 (3-leg 지지)
'hip_x':       0.1225,   # 몸통중심~발 종방향 = BODY_L/2 (회전 운동학, URDF 실측)
'hip_y':       0.10,     # 몸통중심~발 횡방향 = BODY_W/2 + L1
'level_gain':  1.0,      # 중심잡기(수평 유지) 강도 0~1 (IMU 필요)
'level_max':   0.09,     # leveling 발 z 보정 상한 (m)
'gait_type':   'trot',
'cmd_vel_hold_time': 30.0,
'pitch_offset': 0.015,   # 고정 pitch 보정 (rad). + = 앞 들기
'roll_offset':  0.015,   # 고정 roll 보정 (rad). + = 우측 들기
'yaw_trim':     0.09,    # 직진 휨 보정 (rad/s). 우측 휨 → 양수(좌향). 병진 중에만 적용
'fall_detect':  True,    # 넘어짐 감지 (IMU 필요)
'fall_tilt_thresh': 1.0, # 넘어짐 판정 기울기 (rad ~57°)
'auto_recover': False,   # 자동 기립 (구현됨, 현재 OFF). True 면 웅크렸다 밀어올려 STAND
'recover_time': 3.0,     # 기립 시퀀스 길이 (s)
'startup_ramp_time': 3.0,# 기동 시 SIT→STAND ramp (s)
```

**안전 clamp (gait_planner 내부)**: `MAX_LIN = 0.30 m/s`, `MAX_ANG = 0.8 rad/s` (실제 상한은 max_stride scale-down 이 담당).

## 5. ROS 토픽

**gait_node 구독**: `/cmd_vel`(Twist, 보행) · `/body_pose`(Twist, 발고정 몸통 6축) · `/gesture`(String, 제스처) · `/body_height_cmd`(Float32) · `/imu`(Imu)
**gait_node 발행**: `/joint_trajectory_controller/joint_trajectory` (JointTrajectory, 12관절 rad)
**hardware_bridge**: 위 trajectory 수신 → 서보 deg+CRC8 → UART / `/imu` 발행 / `/imu_zero`(Empty) 구독(MCU IMU 영점 재캘리브)

## 6. 캘리브레이션 (SERVO_TRIMS)

트림은 **`src/quadruped_gait/config/servo_trims.yaml`** 에 저장 → `hardware_bridge` 가 부팅 시 로드 (파일 없으면 내장 기본값).

```bash
# 1) ROS 노드 종료 (포트 점유 해제) — calibrate 는 시리얼 직접 사용
# 2) 캘리브 실행 (자세는 실제 걷기 STAND 와 일치):
python3 tools/calibrate.py [/dev/ttyACM0] [--pose home|stand]
```
키: `d`/`a` 다리, `s`/`w` 관절, `+`/`-` ±1°, `]`/`[` ±5°, `r` 리셋, **`p` 출력 + yaml 자동 저장**, `q` 종료.

→ `p` 누르면 `config/servo_trims.yaml` 에 **자동 기록** (수동 복사 불필요). 재기동 시 bridge 가 로드.
캘리브 자세는 launch 의 `body_height`/`pitch_offset` 을 읽어 **실제 걷기 자세와 동일하게** 생성.

## 7. 실행 방법

```bash
# 빌드
cd ~/robot/quadruped_robot
colcon build --symlink-install
source install/setup.bash

# 하드웨어 모드
ros2 launch quadruped_bringup hardware.launch.py
ros2 launch quadruped_bringup hardware.launch.py port:=/dev/ttyUSB0   # 포트 지정

# 텔레옵 (별도 터미널) — 이 레포 내장 통합 teleop
ros2 run quadruped_gait teleop_key
```

### teleop_key 키맵 (모드 기반)
| 모드 전환 | `1`=WALK  `2`=POSE |
|------|------|
| **WALK** | `w/s` 전진/후진 · `a/d` 좌/우 회전(**전진 중에만 = 호 회전**, 제자리 회전 X) · `Space/x` 정지 |
| **POSE** (발 고정) | `w/s` pitch · `a/d` yaw · `z/c` roll · `Space` 중립 |
| **높이(공통)** | `[` 낮추기 · `]` 올리기 |
| **제스처(아무때나)** | `h`인사 `j`기지개 `k`끄덕 `l`갸웃 `n`둘러보기 `m`몸털기 `.`까치발 `o`앉기 `p`엎드리기 `i`준비 |
| 종료 | `Ctrl+C` |

> 웹 게임패드로 조종하려면 `prd/web_gamepad.md` 참고 (rosbridge + roslib). teleop 과 동시 사용 금지.

### 앉기 / 일어서기 (토픽 직접)
```bash
ros2 topic pub --once /body_height_cmd std_msgs/Float32 "data: 0.07"   # 앉기
ros2 topic pub --once /body_height_cmd std_msgs/Float32 "data: 0.14"   # 일어서기
```

## 8. 통신 프로토콜 (ROS ↔ MCU)

**ROS → MCU (joint command)**
```
[0xAA 0x55] [0x03] [0x30=48] [12×float32 (degrees 0~180)] [CRC8]
```
- CRC8: polynomial 0x07, init 0x00 (`hardware_bridge._crc8()` = MCU `CRC8Update()`)
- 각도 순서: `[FL.hip, FL.thigh, FL.calf, FR..., RR.calf]`

**MCU → ROS (텍스트, line-based)**
| 메시지 | 형식 | 의미 |
|--------|------|------|
| IMU | `IMU:<roll>,<pitch>,<yaw>` (도) | 자세 (3개 콤마값) |
| Heartbeat | `HB:...PKT:<n>,CRC:<n>,ERR:<n>,WDG:<n>,TO:<n>` | 진단 |
| Error | `[ERROR] <msg>` | MCU 에러 |

## 9. 현재 캘리브 / 튜닝 상태

`config/servo_trims.yaml`:
```yaml
servo_trims:   # [shoulder, thigh, calf] 도 단위
  FL: [1.0, 19.0, -13.0]
  FR: [7.0, -30.0, 0.0]
  RL: [0.0, 22.0, -15.0]
  RR: [6.0, -5.0, 15.0]
```
- 직진 휨: `yaw_trim=0.09` 로 보정 (실기 튜닝값).
- 자세 기울임: `pitch_offset`/`roll_offset` 각 0.015 rad.

## 10. 알려진 한계 / 상태

| 항목 | 상태 | 비고 |
|------|------|------|
| 직진 휨 | ✅ 보정 | 알고리즘 대칭(modular 위상) + `yaw_trim` |
| 호 회전 (전진+회전) | ✅ | trot 통합 회전 운동학 |
| 제자리 회전 / 측방 | ⏸ 비활성 | 의도적 (실기 미동작 → teleop/게이팅으로 차단) |
| 중심잡기 (leveling) | ✅ 작동 (IMU 시) | `level_gain=1.0`. IMU 미수신이면 무동작 |
| 넘어짐 감지/자동기립 | 🟡 구현·OFF | `auto_recover=False`. IMU 필요. 켜려면 True |
| IMU | ⚠️ 환경의존 | MCU 가 `IMU:` 라인 송출해야 작동 (부팅 시 init) |
| MCU 리셋 시 기동 자세 | ⚠️ 펌웨어 | 쫙폄→앉음→쫙폄 — STM32 펌웨어 부팅 시퀀스 이슈 |
| 보행 중 미세 휘청 | ⚠️ 잔여 | 추후 튜닝 |
| config/ 빌드 (description) | ⚠️ 워크어라운드 | `quadruped_description/CMakeLists` 가 없는 config/ 설치 시도 → 로컬 .gitkeep |

## 11. PCA9685 채널 매핑 (MCU 측, 참고)

| 다리 | hip | thigh | calf |
|------|-----|-------|------|
| FL (앞 좌) | 8 | 9 | 10 |
| FR (앞 우) | 12 | 13 | 14 |
| RL (뒤 좌) | 4 | 5 | 6 |
| RR (뒤 우) | 0 | 1 | 2 |

## 12. Git 저장소

- **ROS**: https://github.com/0-jwook/quadruped_robot (main)
- **MCU**: https://github.com/0-jwook/quadruped_robot_MCU (master)

## 13. 문제 해결 빠른 진단

| 증상 | 의심 / 처방 |
|------|------------|
| 전진 명령 줘도 안 감 | **`/cmd_vel` 퍼블리셔 2개?** (`ros2 topic info /cmd_vel`) → teleop/웹 중 하나만. `ros2 topic echo /cmd_vel` 로 0 도배 확인 |
| 로봇 안 움직임 | 시리얼 포트/MCU 전원: `ls -l /dev/ttyACM*`, heartbeat `ERR/TO` |
| 좌/우 휨 | `yaw_trim` 조정 (우측 휨 → 양수↑, 좌측 → ↓). 또는 재캘리브 |
| 중심잡기 안 됨 | `/imu` 발행 확인 (`ros2 topic hz /imu`). 데이터 없으면 MCU 펌웨어/IMU 점검 |
| 너무 빠름/느림 | `period` 키움/줄임 (속도 = max_stride/(duty·period)) |
| 발 끌림 | `step_height` ↑ |
| 캘리브 안 됨 | ROS 노드 종료 후 실행 (포트 점유). `p` 로 yaml 저장 |
