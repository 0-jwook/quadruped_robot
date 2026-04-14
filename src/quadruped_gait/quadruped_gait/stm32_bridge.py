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
        """Send joint angles to STM32"""
        if self.ser and self.ser.is_open and len(msg.points) > 0:
            angles = msg.points[0].positions # List of 12 floats
            
            # Packet: [Start(0xAA, 0x55)] + [12 floats (48 bytes)] + [End(0x0D, 0x0A)]
            header = struct.pack('BB', 0xAA, 0x55)
            data = struct.pack('12f', *angles)
            footer = struct.pack('BB', 0x0D, 0x0A)
            
            try:
                self.ser.write(header + data + footer)
            except Exception as e:
                self.get_logger().error(f"Write error: {e}")

    def read_from_serial(self):
        """Read IMU data from STM32"""
        if not self.ser or not self.ser.is_open:
            return

        # Expected packet size: Header(2) + IMU(40) + Footer(2) = 44 bytes
        if self.ser.in_waiting >= 44:
            # Look for header 0x55 0xAA
            if self.ser.read(1) == b'\x55':
                if self.ser.read(1) == b'\xAA':
                    payload = self.ser.read(40)
                    footer = self.ser.read(2)
                    
                    if footer == b'\x0D\x0A':
                        try:
                            # 10 floats: Quat(x,y,z,w), AngVel(x,y,z), Accel(x,y,z)
                            imu_data = struct.unpack('10f', payload)
                            
                            msg = Imu()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = "imu_link"
                            
                            # Orientation
                            msg.orientation.x = imu_data[0]
                            msg.orientation.y = imu_data[1]
                            msg.orientation.z = imu_data[2]
                            msg.orientation.w = imu_data[3]
                            
                            # Angular Velocity
                            msg.angular_velocity.x = imu_data[4]
                            msg.angular_velocity.y = imu_data[5]
                            msg.angular_velocity.z = imu_data[6]
                            
                            # Linear Acceleration
                            msg.linear_acceleration.x = imu_data[7]
                            msg.linear_acceleration.y = imu_data[8]
                            msg.linear_acceleration.z = imu_data[9]
                            
                            self.imu_pub.publish(msg)
                        except Exception as e:
                            self.get_logger().error(f"Unpack error: {e}")
                    else:
                        self.get_logger().warn("Invalid footer received")

def main(args=None):
    rclpy.init(args=args)
    node = STM32Bridge()
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
