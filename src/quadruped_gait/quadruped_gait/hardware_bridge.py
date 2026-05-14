import math
import threading
import struct

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
# 서보 트림값 (기계적 조립 오차 보정, 단위: degree)
# 완전 펴진 상태(q1=q2=q3=0)에서 실측값 - 이론값
# ---------------------------------------------------------------------------
SERVO_TRIMS = {
    #        shoulder  thigh   calf
    'FL': (   0.0,    0.0,    0.0),   # FL: 90, 0, 180
    'FR': (   5.0,  -12.0,    0.0),   # FR: 95, 168, 0
    'RL': (   0.0,   10.0,    0.0),   # RL: 90, 10, 180
    'RR': (   5.0,    0.0,   10.0),   # RR: 95, 180, 10
}


def _clamp(val: float, lo: float = 0.0, hi: float = 180.0) -> float:
    return max(lo, min(hi, val))


def _crc8(data: bytes) -> int:
    """CRC-8 (polynomial 0x07, init 0x00) — MCU CRC8Update()와 동일 알고리즘"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    """Roll/Pitch/Yaw (rad) → 쿼터니언 (x, y, z, w)"""
    cr, cp, cy = math.cos(roll / 2), math.cos(pitch / 2), math.cos(yaw / 2)
    sr, sp, sy = math.sin(roll / 2), math.sin(pitch / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def ik_to_servo_deg(q1: float, q2: float, q3: float, leg: str):
    """
    IK 관절 각도(rad)를 하드웨어 서보 각도(deg, 0~180)로 변환.
    """
    ts, tt, tc = SERVO_TRIMS[leg]
    is_right = leg in ('FR', 'RR')

    if not is_right:  # 왼쪽 (FL, RL): 허벅지 수직하=0°
        shoulder = _clamp( 90.0 + math.degrees(q1) + ts)
        thigh    = _clamp(  0.0 - math.degrees(q2) + tt)
        calf     = _clamp(180.0 - math.degrees(q3) + tc)
    else:             # 오른쪽 (FR, RR): 허벅지 수직하=180° (서보 반대 장착)
        shoulder = _clamp( 90.0 + math.degrees(q1) + ts)
        thigh    = _clamp(180.0 + math.degrees(q2) + tt)
        calf     = _clamp(  0.0 + math.degrees(q3) + tc)

    return shoulder, thigh, calf


class HardwareBridge(Node):
    """
    ROS2 ↔ STM32 UART 브릿지 (바이너리 프로토콜 버전).
    """

    def __init__(self):
        super().__init__('hardware_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        self.ser = None
        if SERIAL_AVAILABLE:
            try:
                self.ser = serial.Serial(port, baud, timeout=1.0)
                self.get_logger().info(f'STM32 연결 완료: {port} @ {baud} bps (바이너리 모드)')
            except Exception as e:
                self.get_logger().error(f'시리얼 포트 열기 실패: {e}')
        else:
            self.get_logger().warn('pyserial 미설치')

        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self._traj_callback,
            10,
        )

        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        self._stop_event = threading.Event()
        if self.ser:
            self._read_thread = threading.Thread(
                target=self._serial_read_loop, daemon=True
            )
            self._read_thread.start()

        self.get_logger().info('Hardware Bridge 노드 시작.')

    def _traj_callback(self, msg: JointTrajectory):
        """12개 관절 각도(rad)를 바이너리 패킷으로 전송."""
        if not self.ser or not self.ser.is_open:
            return
        if not msg.points:
            return

        pos = msg.points[0].positions
        if len(pos) < 12:
            return

        # 각 다리별 변환 (트림값 포함)
        fl = ik_to_servo_deg(pos[0],  pos[1],  pos[2],  'FL')
        fr = ik_to_servo_deg(pos[3],  pos[4],  pos[5],  'FR')
        rl = ik_to_servo_deg(pos[6],  pos[7],  pos[8],  'RL')
        rr = ik_to_servo_deg(pos[9],  pos[10], pos[11], 'RR')

        # 12개 각도 리스트 생성
        angles = list(fl) + list(fr) + list(rl) + list(rr)

        # 바이너리 패킷 생성: [0xAA, 0x55] [ID=0x03] [LEN=48] [Payload(48)] [Checksum]
        header = b'\xaa\x55'
        packet_id = 0x03
        length = 48
        
        payload = struct.pack('<12f', *angles)
        checksum = _crc8(bytes([packet_id, length]) + payload)
        
        packet = header + bytes([packet_id, length]) + payload + bytes([checksum])

        try:
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f'시리얼 쓰기 오류: {e}')

        # 5초마다 로그 출력
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, '_last_log_t') or now_sec - self._last_log_t > 5.0:
            self._last_log_t = now_sec
            angle_str = ", ".join([f"{a:.1f}" for a in angles])
            self.get_logger().info(f'전송 중 (Binary ID 0x03): {angle_str}')

    def _serial_read_loop(self):
        while not self._stop_event.is_set():
            if not self.ser or not self.ser.is_open:
                break
            try:
                raw = self.ser.readline()
                if not raw: continue
                line = raw.decode('ascii', errors='ignore').strip()
                if not line: continue

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
        try:
            parts = line[4:].split(',')
            if len(parts) != 3: return
            roll, pitch, yaw = map(lambda x: math.radians(float(x)), parts)
        except ValueError: return

        qx, qy, qz, qw = _rpy_to_quaternion(roll, pitch, yaw)
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w = qx, qy, qz, qw
        self.imu_pub.publish(imu_msg)

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
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
