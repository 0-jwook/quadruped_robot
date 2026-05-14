import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import serial
import struct
import math


def crc8(data: bytes) -> int:
    """CRC-8 (poly=0x07, init=0x00) — MCU ros_com.cpp와 동일"""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) if (crc & 0x80) else (crc << 1)
            crc &= 0xFF
    return crc


# L1=0.03, L2=0.115, L3=0.135, body_height=0.17m 기준 IK 중립 각도
_Q2_N = -37.4   # 허벅지 중립 (도)
_Q3_N =  91.75  # 무릎 중립 (도)

# SERVO_MAP[leg][joint] = (home_deg, direction, q_neutral_deg)
# 변환: servo = clamp(home + direction * (deg(q) - q_neutral), 0, 180)
# leg: FL=0, FR=1, BL=2, BR=3  /  joint: hip=0, thigh=1, calf=2
SERVO_MAP = [
    # FL: home={90, 0, 180}
    [(90.0, +1, 0.0), (0.0,   +1, _Q2_N), (180.0, -1, _Q3_N)],
    # FR: home={95, 168, 0}  (좌우 대칭 → 방향 반전)
    [(95.0, -1, 0.0), (168.0, -1, _Q2_N), (0.0,   +1, _Q3_N)],
    # BL: home={90, 10, 180}
    [(90.0, +1, 0.0), (10.0,  +1, _Q2_N), (180.0, -1, _Q3_N)],
    # BR: home={95, 180, 10}  (좌우 대칭 → 방향 반전)
    [(95.0, -1, 0.0), (180.0, -1, _Q2_N), (10.0,  +1, _Q3_N)],
]


def _q_to_servo(q_rad: float, home: float, direction: int, q_n_deg: float) -> float:
    servo = home + direction * (math.degrees(q_rad) - q_n_deg)
    return max(0.0, min(180.0, servo))


class MCUBridge(Node):
    def __init__(self):
        super().__init__('mcu_bridge')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value

        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f'Connected to MCU on {port} at {baud}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MCU: {e}')
            self.ser = None

        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.joint_callback,
            10)

        self.get_logger().info('MCU Bridge started')

    def _make_packet(self, pkt_id: int, payload: bytes) -> bytes:
        """0xAA 0x55 | ID | LEN | payload | CRC8(ID+LEN+payload)"""
        meta = bytes([pkt_id, len(payload)])
        return bytes([0xAA, 0x55]) + meta + payload + bytes([crc8(meta + payload)])

    def joint_callback(self, msg):
        if not self.ser or not msg.points:
            return

        positions = list(msg.points[-1].positions)
        if len(positions) < 12:
            return

        # gait_node 순서: FL[q1,q2,q3], FR[q1,q2,q3], BL[q1,q2,q3], BR[q1,q2,q3]
        # MCU JointAngleCmd 순서: 동일 (FL,FR,BL,BR 각 hip,thigh,calf)
        angles = []
        for leg in range(4):
            for joint in range(3):
                home, direction, q_n = SERVO_MAP[leg][joint]
                angles.append(_q_to_servo(positions[leg * 3 + joint], home, direction, q_n))

        try:
            self.ser.write(self._make_packet(0x03, struct.pack('<12f', *angles)))
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MCUBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
