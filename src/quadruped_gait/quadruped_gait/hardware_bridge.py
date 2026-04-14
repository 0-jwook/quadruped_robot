"""
hardware_bridge.py
==================
실제 STM32F103RB 하드웨어와 ROS2를 연결하는 브릿지 노드.

동작 흐름:
  gait_node  →  /joint_trajectory_controller/joint_trajectory
         → (이 노드) 라디안→서보 도 변환 → UART ASCII 전송 → STM32
  STM32  →  "IMU:<roll>,<pitch>,<yaw>\\n"  →  (이 노드)  →  /imu 토픽

시뮬레이션 노드(stm32_bridge, mcu_bridge, sim.launch.py)는 수정하지 않음.
하드웨어 모드는 hardware.launch.py 로 기동.

서보 각도 변환 기준 (README.md):
  Home Position (기립, 다리 아래로 쭉 뻗은 상태):
    FL/RL (왼쪽): shoulder=90°, thigh=0°,   calf=180°
    FR/RR (오른쪽): shoulder=90°, thigh=180°, calf=0°

  IK 중립값 (body_height=0.27m, L1=0.08m, L2=L3=0.2m):
    Q2_NEUTRAL = -0.8327 rad
    Q3_NEUTRAL =  1.6597 rad

  왼쪽 다리 변환:
    shoulder_deg = 90 + degrees(q1)
    thigh_deg    = 0  + degrees(q2 - Q2_NEUTRAL)
    calf_deg     = 180 - degrees(q3 - Q3_NEUTRAL)

  오른쪽 다리 변환 (delta 부호 반전):
    shoulder_deg = 90  - degrees(q1)
    thigh_deg    = 180 - degrees(q2 - Q2_NEUTRAL)
    calf_deg     = 0   + degrees(q3 - Q3_NEUTRAL)
"""

import math
import threading

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ---------------------------------------------------------------------------
# IK 중립 각도 (gait_planner.py 의 Q2_NEUTRAL, Q3_NEUTRAL 과 동일 값)
# ---------------------------------------------------------------------------
Q2_NEUTRAL = -0.8327  # rad  (허벅지, 기립 자세)
Q3_NEUTRAL =  1.6597  # rad  (종아리, 기립 자세)


def _clamp(val: float, lo: float = 0.0, hi: float = 180.0) -> float:
    return max(lo, min(hi, val))


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    """Roll/Pitch/Yaw (rad) → 쿼터니언 (x, y, z, w)"""
    cr, cp, cy = math.cos(roll / 2), math.cos(pitch / 2), math.cos(yaw / 2)
    sr, sp, sy = math.sin(roll / 2), math.sin(pitch / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def ik_to_servo_deg(q1: float, q2: float, q3: float, is_right: bool):
    """
    IK 관절 각도(rad)를 하드웨어 서보 각도(deg, 0~180)로 변환.

    Parameters
    ----------
    q1 : shoulder(abduction) 각도 (rad)
    q2 : thigh 각도 (rad)
    q3 : calf 각도 (rad)
    is_right : True → 오른쪽 다리(FR/RR), False → 왼쪽 다리(FL/RL)

    Returns
    -------
    (shoulder_deg, thigh_deg, calf_deg) : 각 0.0 ~ 180.0
    """
    dq2 = math.degrees(q2 - Q2_NEUTRAL)
    dq3 = math.degrees(q3 - Q3_NEUTRAL)

    if not is_right:
        shoulder = _clamp(90.0 + math.degrees(q1))
        thigh    = _clamp(0.0  + dq2)
        calf     = _clamp(180.0 - dq3)
    else:
        shoulder = _clamp(90.0  - math.degrees(q1))
        thigh    = _clamp(180.0 - dq2)
        calf     = _clamp(0.0   + dq3)

    return shoulder, thigh, calf


class HardwareBridge(Node):
    """
    ROS2 ↔ STM32 UART 브릿지 (ASCII 텍스트 프로토콜).

    JointTrajectory 구독 → A: 명령 전송
    IMU ASCII 수신      → /imu 토픽 발행
    """

    def __init__(self):
        super().__init__('hardware_bridge')

        # ------------------------------------------------------------------
        # 파라미터
        # ------------------------------------------------------------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        # ------------------------------------------------------------------
        # 시리얼 포트 초기화
        # ------------------------------------------------------------------
        self.ser = None
        if SERIAL_AVAILABLE:
            try:
                self.ser = serial.Serial(port, baud, timeout=1.0)
                self.get_logger().info(f'STM32 연결 완료: {port} @ {baud} bps')
            except Exception as e:
                self.get_logger().error(f'시리얼 포트 열기 실패: {e}')
                self.get_logger().warn('시리얼 없이 기동합니다 (명령 전송 비활성화)')
        else:
            self.get_logger().warn('pyserial 미설치 — 시리얼 통신 비활성화')

        # ------------------------------------------------------------------
        # ROS2 인터페이스
        # ------------------------------------------------------------------
        # gait_node 가 발행하는 동일 토픽 구독 (시뮬레이션과 토픽 이름 공유)
        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self._traj_callback,
            10,
        )

        # IMU → /imu 발행 (gait_node 의 imu_callback 과 동일 토픽)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        # ------------------------------------------------------------------
        # IMU 수신 스레드 (별도 데몬 스레드)
        # ------------------------------------------------------------------
        self._stop_event = threading.Event()
        if self.ser:
            self._read_thread = threading.Thread(
                target=self._serial_read_loop, daemon=True
            )
            self._read_thread.start()

        self.get_logger().info('Hardware Bridge 노드 시작.')

    # ----------------------------------------------------------------------
    # JointTrajectory 콜백
    # ----------------------------------------------------------------------
    def _traj_callback(self, msg: JointTrajectory):
        """
        12개 관절 각도(rad)를 받아 STM32 A 명령으로 전송.

        joint_names 순서 (gait_node 기준):
          [0] FL_shoulder  [1] FL_thigh  [2] FL_calf
          [3] FR_shoulder  [4] FR_thigh  [5] FR_calf
          [6] RL_shoulder  [7] RL_thigh  [8] RL_calf
          [9] RR_shoulder [10] RR_thigh [11] RR_calf

        MCU A 명령 순서:
          A:<h0>,<t0>,<c0>,<h1>,<t1>,<c1>,<h2>,<t2>,<c2>,<h3>,<t3>,<c3>\\n
          leg0=FL, leg1=FR, leg2=BL(RL), leg3=BR(RR)
        """
        if not self.ser or not self.ser.is_open:
            return
        if not msg.points:
            return

        pos = msg.points[0].positions
        if len(pos) < 12:
            self.get_logger().warn(f'관절 수 부족: {len(pos)}/12')
            return

        # 각 다리별 변환 (is_right: 1=FR, 3=RR → True)
        fl_s, fl_t, fl_c = ik_to_servo_deg(pos[0],  pos[1],  pos[2],  is_right=False)
        fr_s, fr_t, fr_c = ik_to_servo_deg(pos[3],  pos[4],  pos[5],  is_right=True)
        rl_s, rl_t, rl_c = ik_to_servo_deg(pos[6],  pos[7],  pos[8],  is_right=False)
        rr_s, rr_t, rr_c = ik_to_servo_deg(pos[9],  pos[10], pos[11], is_right=True)

        cmd = (
            f'A:{fl_s:.1f},{fl_t:.1f},{fl_c:.1f},'
            f'{fr_s:.1f},{fr_t:.1f},{fr_c:.1f},'
            f'{rl_s:.1f},{rl_t:.1f},{rl_c:.1f},'
            f'{rr_s:.1f},{rr_t:.1f},{rr_c:.1f}\n'
        )

        try:
            self.ser.write(cmd.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f'시리얼 쓰기 오류: {e}')

    # ----------------------------------------------------------------------
    # 시리얼 읽기 루프 (별도 스레드)
    # ----------------------------------------------------------------------
    def _serial_read_loop(self):
        """STM32 에서 오는 데이터 파싱 루프."""
        while not self._stop_event.is_set():
            if not self.ser or not self.ser.is_open:
                break
            try:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('IMU:'):
                    self._handle_imu(line)
                elif line.startswith('[SYSTEM]'):
                    self.get_logger().info(f'STM32: {line}')
                elif line.startswith('[ERROR]'):
                    self.get_logger().error(f'STM32: {line}')

            except Exception as e:
                if not self._stop_event.is_set():
                    self.get_logger().warn(f'시리얼 읽기 오류: {e}')

    def _handle_imu(self, line: str):
        """
        "IMU:<roll>,<pitch>,<yaw>" 파싱 후 sensor_msgs/Imu 발행.
        STM32 전송 단위: 도(°). yaw는 항상 0.0 (자력계 없음).
        """
        try:
            parts = line[4:].split(',')   # "IMU:" 이후
            if len(parts) != 3:
                return
            roll_deg  = float(parts[0])
            pitch_deg = float(parts[1])
            yaw_deg   = float(parts[2])
        except ValueError:
            self.get_logger().warn(f'IMU 파싱 실패: {line}')
            return

        roll  = math.radians(roll_deg)
        pitch = math.radians(pitch_deg)
        yaw   = math.radians(yaw_deg)

        qx, qy, qz, qw = _rpy_to_quaternion(roll, pitch, yaw)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'

        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        # STM32 는 각속도/가속도를 별도 전송하지 않으므로 공분산 -1 (미지정)
        imu_msg.orientation_covariance[0] = -1.0
        imu_msg.angular_velocity_covariance[0] = -1.0
        imu_msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(imu_msg)

    # ----------------------------------------------------------------------
    # 소멸자
    # ----------------------------------------------------------------------
    def destroy_node(self):
        self._stop_event.set()
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
