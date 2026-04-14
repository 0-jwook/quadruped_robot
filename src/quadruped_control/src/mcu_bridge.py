import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import serial
import time

class MCUBridge(Node):
    def __init__(self):
        super().__init__('mcu_bridge')
        
        # 파라미터 선언 (기본 포트와 보드레이트)
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

        # 조인트 트래젝토리 구독
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.joint_callback,
            10)
            
        self.get_logger().info('MCU Bridge Node Started')

    def joint_callback(self, msg):
        if not self.ser or not msg.points:
            return
            
        # 최신 포인트의 각도 데이터 추출 (12개)
        positions = msg.points[-1].positions
        
        # 라디안 데이터를 도(Degree) 단위로 변환 (MCU 제어 편의성)
        angles_deg = [round(math.degrees(p), 2) for p in positions]
        
        # 데이터 패킷 생성 (예: <ang1,ang2,...,ang12>\n)
        payload = "<" + ",".join(map(str, angles_deg)) + ">\n"
        
        try:
            self.ser.write(payload.encode())
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')

import math
def main(args=None):
    rclpy.init(args=args)
    node = MCUBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
