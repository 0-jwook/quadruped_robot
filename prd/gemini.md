# PRD: 사족보행 로봇 ROS2 시뮬레이션 (MCU 없는 환경)

## 1. 프로젝트 개요

| 항목 | 내용 |
|------|------|
| 프로젝트명 | Quadruped Robot ROS2 Simulation |
| 버전 | v0.1.0 |
| 작성일 | 2026-03-04 |
| 목적 | MCU 없이 ROS2 + Gazebo 환경에서 사족보행 로봇의 보행 알고리즘 검증 |
| 개발 방식 | AI CLI 도구 기반 코드 생성 및 반복 개발 |

---

## 2. 배경 및 목적

실제 하드웨어(MCU + 12개 서보모터) 연결 전에 소프트웨어 스택을 검증하기 위해 순수 소프트웨어 시뮬레이션 환경을 먼저 구축한다.

**핵심 목표:**
- ROS2 기반 제어 아키텍처 조기 검증
- 보행 알고리즘(Gait) 시뮬레이션 반복 테스트
- 실제 MCU 연결 시 최소한의 수정으로 전환 가능한 구조 설계
- AI CLI 도구로 빠른 프로토타이핑

---

## 3. 범위 (Scope)

### ✅ In Scope
- URDF/Xacro 기반 사족보행 로봇 모델링 (다리 4개, 각 3-DOF)
- Gazebo 시뮬레이터 연동
- ROS2 기반 Joint State Publisher / Controller 구성
- 기본 보행 패턴 구현 (Trot, Crawl)
- RViz2 시각화
- ros2_control 인터페이스 구성 (추후 MCU 연결 대비)

### ❌ Out of Scope
- 실제 MCU 펌웨어 개발
- 실제 서보모터 PWM 제어
- 강화학습 기반 보행 (v2 이후)
- 카메라/LiDAR 센서 통합

---

## 4. 시스템 아키텍처

```
┌─────────────────────────────────────────────┐
│              ROS2 Ecosystem                 │
│                                             │
│  [Gait Planner Node]                        │
│       │ /joint_trajectory                   │
│       ▼                                     │
│  [ros2_control Manager]                     │
│       │ /joint_states                       │
│       ▼                                     │
│  [Gazebo Plugin / sim_hardware_interface]   │
│       │                                     │
│       ▼                                     │
│  [Gazebo Simulation]  ◄──► [RViz2]          │
└─────────────────────────────────────────────┘
```

**추후 MCU 연결 시 변경 포인트:**
```
[sim_hardware_interface] → [real_hardware_interface (serial/CAN)]
```
*나머지 노드는 변경 없음*

---

## 5. 로봇 모델 사양

```
다리 구성: 4개 (FL, FR, RL, RR)
각 다리 DOF: 3
  - Joint 1: Hip Abduction/Adduction
  - Joint 2: Hip Flexion/Extension  
  - Joint 3: Knee Flexion/Extension

총 Joint 수: 12개
서보모터 토크: 시뮬레이션 파라미터로 설정
링크 길이: upper_leg 0.1m / lower_leg 0.1m (조정 가능)
```

---

## 6. 기능 요구사항

### F-01. 로봇 모델링 (URDF/Xacro)
- [ ] base_link 정의 (몸통)
- [ ] 4개 다리 × 3 Joint URDF 작성
- [ ] 충돌 모델(collision) 및 관성(inertia) 정의
- [ ] Gazebo 플러그인 태그 삽입
- [ ] 시각화용 mesh 또는 primitive geometry 적용

### F-02. ROS2 패키지 구성
- [ ] `quadruped_description` — URDF, mesh, launch 파일
- [ ] `quadruped_control` — ros2_control config, controller 설정
- [ ] `quadruped_gait` — 보행 알고리즘 노드
- [ ] `quadruped_bringup` — 전체 시스템 launch

### F-03. 시뮬레이션 환경
- [ ] Gazebo world 파일 구성 (평지)
- [ ] `gazebo_ros2_control` 플러그인 연동
- [ ] Joint Position Controller 12개 설정
- [ ] 로봇 spawn launch 파일 작성

### F-04. 보행 알고리즘 (Gait)
- [ ] Forward Kinematics 구현
- [ ] Inverse Kinematics 구현 (각 다리 단독)
- [ ] Trot Gait 구현 (대각선 다리 쌍 동시 이동)
- [ ] Crawl Gait 구현 (한 번에 한 다리 이동)
- [ ] `/cmd_vel` 토픽 수신하여 속도 명령 처리

### F-05. 시각화 및 디버깅
- [ ] RViz2 설정 파일 작성 (joint_states, TF 시각화)
- [ ] `/joint_states` 퍼블리시 확인
- [ ] rqt_graph로 노드 연결 확인 가능

---

## 7. 비기능 요구사항

| 항목 | 요구사항 |
|------|----------|
| ROS2 버전 | Humble 또는 Iron |
| 시뮬레이터 | Gazebo Classic 11 또는 Ignition Fortress |
| 언어 | Python 3.10+ / C++ (선택) |
| 제어 주기 | 100Hz 이상 |
| 재사용성 | MCU 연결 시 hardware_interface만 교체 |
| 코드 구조 | AI CLI로 생성 가능한 단일 패키지 단위 분리 |

---

## 8. 파일 / 디렉토리 구조

```
quadruped_ws/
├── src/
│   ├── quadruped_description/
│   │   ├── urdf/
│   │   │   └── quadruped.urdf.xacro
│   │   ├── meshes/
│   │   ├── launch/
│   │   │   └── display.launch.py
│   │   └── config/
│   │       └── rviz2.rviz
│   │
│   ├── quadruped_control/
│   │   ├── config/
│   │   │   └── controllers.yaml
│   │   └── launch/
│   │       └── control.launch.py
│   │
│   ├── quadruped_gait/
│   │   ├── quadruped_gait/
│   │   │   ├── __init__.py
│   │   │   ├── kinematics.py       # IK/FK
│   │   │   ├── gait_planner.py     # Trot / Crawl
│   │   │   └── gait_node.py        # ROS2 Node
│   │   └── setup.py
│   │
│   └── quadruped_bringup/
│       └── launch/
│           └── sim.launch.py       # 전체 시스템 기동
└── README.md
```

---

## 9. 토픽 / 인터페이스 정의

| 토픽 | 타입 | 방향 | 설명 |
|------|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | Subscribe | 속도 명령 입력 |
| `/joint_states` | `sensor_msgs/JointState` | Publish | 12개 조인트 상태 |
| `/joint_trajectory_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Publish | 궤적 명령 |
| `/tf` | `tf2_msgs/TFMessage` | Publish | 좌표계 변환 |
| `/robot_description` | `std_msgs/String` | Publish | URDF 문자열 |

---

## 10. 개발 단계 (Milestone)

```
Phase 1 — 모델링 및 환경 구축        (1~2일)
  └─ URDF 작성 → Gazebo spawn → RViz2 확인

Phase 2 — ros2_control 연동          (1~2일)
  └─ controller.yaml 설정 → joint position 수동 명령 테스트

Phase 3 — Kinematics 구현            (2~3일)
  └─ FK 구현 → IK 구현 → 단일 다리 말단 위치 제어 테스트

Phase 4 — Gait 알고리즘              (3~4일)
  └─ Trot Gait → Crawl Gait → cmd_vel 연동

Phase 5 — 통합 테스트 및 정리        (1~2일)
  └─ 전체 launch 통합 → 문서화 → MCU 연결 인터페이스 정의
```

---

## 11. AI CLI 개발 가이드라인

AI CLI 도구(Claude Code 등)를 활용할 때 아래 프롬프트 패턴을 사용한다.

```bash
# URDF 생성 예시
"사족보행 로봇 URDF를 작성해줘.
 다리 4개(FL, FR, RL, RR), 각 다리 Hip/Thigh/Knee 3개 joint,
 링크 길이 upper 0.1m lower 0.1m, Gazebo 플러그인 포함"

# Controller 설정 예시  
"ros2_control joint_position_controller를 12개 joint에 대해
 controllers.yaml로 작성해줘. 로봇명: quadruped"

# IK 구현 예시
"3-DOF 다리의 Inverse Kinematics Python 코드 작성.
 입력: 말단 위치 (x, y, z), 출력: joint angle 3개
 링크 파라미터는 config로 분리"
```

**AI CLI 작업 원칙:**
- 파일 단위로 생성 요청 (한 번에 하나의 파일)
- 생성 후 즉시 `colcon build` 로 컴파일 확인
- 각 Phase 완료 후 git commit

---

## 12. 완료 기준 (Definition of Done)

- [ ] `ros2 launch quadruped_bringup sim.launch.py` 단일 명령으로 전체 시스템 기동
- [ ] Gazebo에서 로봇이 스폰되고 넘어지지 않는 정적 균형 확인
- [ ] `/cmd_vel`에 전진 명령 시 Trot Gait으로 전진 동작 확인
- [ ] RViz2에서 TF 및 joint_states 실시간 시각화 확인
- [ ] 코드 수정 없이 hardware_interface 교체만으로 실제 MCU 연결 가능한 구조 문서화

---

## 13. 리스크 및 대응

| 리스크 | 가능성 | 대응 |
|--------|--------|------|
| URDF 관성 설정 오류로 Gazebo 불안정 | 높음 | 단순 box geometry로 시작 후 점진적 정밀화 |
| IK 특이점(singularity) 처리 | 중간 | joint limit 설정 및 예외 처리 필수 |
| ros2_control 버전 호환성 | 중간 | Humble 기준 패키지 버전 고정 |
| Gait 주기 타이밍 불일치 | 낮음 | Timer 기반 고정 주기 제어 사용 |