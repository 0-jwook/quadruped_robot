import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import Imu
import serial
import struct
import math

class STM32Bridge(Node):
    def __init__(self):
        super().__init__('stm32_bridge')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        
        self.get_logger().info(f"Connecting to STM32 on {port} at {baud}...")
        
        try:
            self.ser = serial.Serial(port, baud, timeout=0.01)
            self.get_logger().info("Connected successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            self.ser = None

        # 1. Subscribe to Joint Trajectory (ROS -> STM32)
        # gait_node publishes to this topic
        self.joint_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self.joint_callback,
            10)

        # 2. Publish IMU data (STM32 -> ROS)
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)
        
        # Timer for reading from Serial (100Hz)
        self.read_timer = self.create_timer(0.01, self.read_from_serial)

    def joint_callback(self, msg):
        """Send joint angles to STM32 using Binary Protocol with Checksum"""
        if self.ser and self.ser.is_open and len(msg.points) > 0:
            # angles: List of 12 floats (radiants to degrees conversion if needed)
            # ROS typically uses radians, but our STM32 expects degrees (0-180)
            angles_rad = msg.points[0].positions
            angles_deg = [math.degrees(a) + 90.0 for a in angles_rad]
            
            # Protocol: [0xAA, 0x55] [ID=0x03] [LEN=48] [Payload(48)] [Checksum]
            header = b'\xaa\x55'
            packet_id = 0x03
            length = 48
            
            # Pack 12 floats (48 bytes)
            payload = struct.pack('<12f', *angles_deg)
            
            # Calculate Checksum: sum of (ID + LEN + Payload bytes)
            checksum = (packet_id + length + sum(payload)) & 0xFF
            
            packet = header + bytes([packet_id, length]) + payload + bytes([checksum])
            
            try:
                self.ser.write(packet)
            except Exception as e:
                self.get_logger().error(f"Write error: {e}")

    def read_from_serial(self):
        """Read ASCII telemetry from STM32 (IMU:roll,pitch,yaw)"""
        if not self.ser or not self.ser.is_open:
            return

        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('IMU:'):
                    # IMU:roll,pitch,yaw
                    parts = line[4:].split(',')
                    if len(parts) == 3:
                        r, p, y = map(float, parts)
                        msg = Imu()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = "imu_link"
                        # Simple RPY to Quat conversion if needed, 
                        # for now just publishing raw orientation placeholder or skip
                        self.get_logger().debug(f"IMU: R={r}, P={p}, Y={y}")
                elif line.startswith('HB:'):
                    self.get_logger().debug(f"Heartbeat: {line}")
        except Exception as e:
            self.get_logger().error(f"Read error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
