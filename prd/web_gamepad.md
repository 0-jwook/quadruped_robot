# PRD: 웹 기반 게임패드/조이스틱 원격 조종 (Web Gamepad Teleop)

## 1. 개요

| 항목 | 내용 |
|------|------|
| 기능명 | Web Gamepad Teleop |
| 버전 | v0.1.0 |
| 작성일 | 2026-06-16 |
| 목적 | 브라우저(웹)에서 게임패드/가상 조이스틱으로 사족보행 로봇을 원격 조종 |
| 개발 위치 | **라즈베리파이가 아닌 별도 환경**(PC 등)에서 개발하는 웹 앱 |
| 대상 로봇 | github.com/0-jwook/quadruped_robot (ROS2 Humble, STM32 하드웨어) |

---

## 2. 배경 & 동기

현재 조작은 `teleop_key`(키보드)뿐인데, **키 하나당 한 축만** 입력돼서 "전진+회전 동시(호 이동)" 같은 조합 동작을 못 낸다. 게임패드의 아날로그 스틱은 **연속·다축 입력**이 가능해 `/cmd_vel`(vx, vy, ω)을 자연스럽게 조합할 수 있다. 웹으로 만들면 **휴대폰/노트북 브라우저만으로** 무선 조종이 가능하고 별도 앱 설치가 필요 없다.

---

## 3. 범위 (Scope)

### ✅ In Scope
- 브라우저에서 동작하는 조종 UI (PC/모바일)
- 물리 게임패드(Gamepad API) + 화면 가상 조이스틱(터치) 둘 다 지원
- 보행 조종: 좌스틱=병진(vx, vy), 우스틱=회전(ω) → `/cmd_vel`
- 바디포즈 조종: roll/pitch/yaw → `/body_pose`
- 제스처 버튼 (bow, sit, lie 등) → `/gesture`
- 몸체 높이 슬라이더 → `/body_height_cmd`
- **E-Stop 버튼** (즉시 정지/안전 자세)
- 텔레메트리 표시: MCU heartbeat, 연결상태, (가능 시) IMU 자세, 배터리

### ❌ Out of Scope
- 자율주행/경로계획 (Nav2 등)
- 영상 스트리밍 (별도 기능, v2 이후)
- 로봇 펌웨어(STM32) 수정
- 라즈베리파이 측 보행 알고리즘 변경 (이미 구현됨)

---

## 4. 시스템 아키텍처

```
[브라우저 (PC/모바일, 외부)]
   │  · Gamepad API / 가상 조이스틱
   │  · roslib.js (WebSocket)
   ▼  ws://<robot-ip>:9090
[rosbridge_websocket  (라즈베리파이에서 실행)]   ← ros-humble-rosbridge-suite
   │  publish/subscribe
   ▼
[ROS2 그래프 (gait_node 등)]
   /cmd_vel /body_pose /gesture /body_height_cmd /imu /estop ...
```

- **웹 ↔ ROS 연결**: `rosbridge_suite`(WebSocket, 포트 9090) + 브라우저의 `roslib.js`. 웹 앱은 라파의 IP로 WebSocket 접속해 토픽을 직접 pub/sub.
- 웹 앱 자체는 정적 파일(HTML/JS) — 어디서 호스팅하든 무방(외부 PC, GitHub Pages 등). 단 브라우저가 로봇과 **같은 네트워크**여야 함.
- 라즈베리파이는 `rosbridge_websocket`만 추가로 띄우면 됨(보행 스택은 그대로).

---

## 5. 인터페이스 (ROS 토픽) — 기존 노드와 호환

| 토픽 | 타입 | 방향(웹 기준) | 설명 |
|------|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | publish | linear.x=전후, linear.y=좌우, angular.z=회전 |
| `/body_pose` | `geometry_msgs/Twist` | publish | linear=(dx,dy,dz), angular=(roll,pitch,yaw) |
| `/gesture` | `std_msgs/String` | publish | 제스처 이름 (bow/stretch/nod/.../ready) |
| `/body_height_cmd` | `std_msgs/Float32` | publish | 몸체 높이 (m, 0.07~0.21) |
| `/estop` | `std_msgs/Bool` (신규) | publish | true=비상정지 (※ 라파 측 노드 지원 필요) |
| `/imu` | `sensor_msgs/Imu` | subscribe | 자세 표시용 (현재 MCU 미송출 — 추후) |
| `/mcu_status` | (heartbeat) | subscribe | 연결·통신 상태 표시 (브릿지 토픽화 필요) |

> 제스처 가능 목록(현재): `bow, stretch, nod, tilt, look, shake, tall, sit, lie, ready`

---

## 6. 기능 요구사항

### F-01. 연결 관리
- [ ] 로봇 IP/포트 입력 → WebSocket 연결, 상태 표시(연결/끊김/재연결)
- [ ] 연결 끊기면 자동 재연결 + 화면 경고

### F-02. 보행 조이스틱
- [ ] 좌스틱 → `linear.x`(상하), `linear.y`(좌우). 우스틱 좌우 → `angular.z`
- [ ] 데드존(중앙 무입력 영역) + 최댓값 클램프 (vx,vy ≤ 0.15, ω ≤ 0.6 권장 — 로봇 한계와 일치)
- [ ] 입력 없으면 0 발행. **주기적 발행(예: 20Hz)** 로 deadman 효과

### F-03. 바디포즈 모드
- [ ] 모드 토글(WALK ↔ POSE)
- [ ] POSE에서 스틱 → roll/pitch/yaw (`/body_pose`)

### F-04. 제스처 / 높이
- [ ] 제스처 버튼 패널 (목록 동적 표시)
- [ ] 높이 슬라이더 → `/body_height_cmd`

### F-05. 안전 (E-Stop & Deadman)
- [ ] 크고 명확한 **E-Stop 버튼** → `/estop true` (또는 cmd_vel 0 연속 발행)
- [ ] **Deadman**: 입력/연결 끊기면 일정 시간 후 자동 정지 명령
- [ ] 연결 끊김 시 시각·청각 경고

### F-06. 텔레메트리
- [ ] MCU 연결/통신 상태 (heartbeat)
- [ ] (가능 시) IMU roll/pitch 표시, 배터리 전압

---

## 7. 비기능 요구사항

| 항목 | 요구사항 |
|------|----------|
| 입력 지연 | 조작→발행 < 50ms (로컬망) |
| 발행 주기 | 보행 명령 ≥ 20Hz |
| 호환 브라우저 | Chrome/Edge (Gamepad API), 모바일 터치 |
| 프레임워크 | 자유 (Vanilla JS / React / Vue 등) + **roslib.js** |
| 라파 의존 | `ros-humble-rosbridge-suite` 설치 + `rosbridge_websocket` 실행만 |
| 보안 | 로컬망 전제. 외부 노출 시 인증/토큰 필요(권장) |

---

## 8. 라즈베리파이(로봇) 측 준비사항

1. 패키지 설치: `sudo apt install ros-humble-rosbridge-suite`
2. 브릿지 실행: `ros2 launch rosbridge_server rosbridge_websocket_launch.xml` (포트 9090)
3. (선택) `/estop`, `/mcu_status` 지원을 위해 라파 노드 보강 — **별도 작업**:
   - gait_node에 `/estop`(Bool) 구독 → true면 안전 정지/sit
   - hardware_bridge heartbeat를 토픽(`/mcu_status`)으로 발행

> 위 3번(라파 노드 보강)은 이 PRD의 웹 개발과 별개로 quadruped_robot 레포에서 진행 가능.

---

## 9. 개발 단계 (Milestone)

```
Phase 1 — 연결 & 기본 보행 (rosbridge + roslib + 가상 조이스틱 → /cmd_vel)
Phase 2 — 물리 게임패드(Gamepad API) 매핑 + 데드존/클램프
Phase 3 — POSE 모드 + 제스처 버튼 + 높이 슬라이더
Phase 4 — E-Stop/Deadman + 텔레메트리(heartbeat/IMU)
Phase 5 — 모바일 UI 다듬기 + 통합 테스트
```

---

## 10. 게임패드 매핑(권장 기본값)

| 입력 | 동작 |
|------|------|
| 좌스틱 ↕ | 전진/후진 (vx) |
| 좌스틱 ↔ | 좌/우 횡이동 (vy) |
| 우스틱 ↔ | 좌/우 회전 (ω) |
| A 버튼 | WALK/POSE 모드 토글 |
| B 버튼 | **E-Stop** |
| 십자키 | 제스처 단축 (예: ↑=ready, ↓=sit, ←=bow, →=nod) |
| 트리거(L/R) | 몸체 높이 −/+ |

> 주의: 라파 측 보행 속도 한계(vx≤0.15, ω≤0.6 등)와 일치시켜 클램프할 것. 자세한 보행 파라미터는 `hardware.launch.py` 참고.

---

## 11. 완료 기준 (Definition of Done)
- [ ] 브라우저에서 로봇 IP 접속 → 조이스틱으로 전진+회전(호) 동시 조종 확인
- [ ] 물리 게임패드 연결 시 자동 인식·매핑
- [ ] E-Stop 1회로 즉시 정지
- [ ] 연결 끊기면 deadman으로 자동 정지
- [ ] 모바일 브라우저에서 가상 조이스틱으로 조종 가능
