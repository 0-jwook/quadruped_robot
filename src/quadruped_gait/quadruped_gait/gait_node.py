import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from .kinematics import LegKinematics
from .gait_planner import GaitPlanner
import math

class GaitNode(Node):
    def __init__(self):
        super().__init__('gait_node')
        # L1=0.04, L2=0.1, L3=0.1 for simple model
        self.kin = LegKinematics(L1=0.04, L2=0.1, L3=0.1)
        self.planner = GaitPlanner(self.kin)
        
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.02, self.timer_callback)
        
        self.cmd_vx, self.cmd_vy, self.cmd_omega = 0.0, 0.0, 0.0
        self.init_t = None
        
        self.joint_names = [
            'front_left_shoulder_joint', 'front_left_leg_joint', 'front_left_foot_joint',
            'front_right_shoulder_joint', 'front_right_leg_joint', 'front_right_foot_joint',
            'rear_left_shoulder_joint', 'rear_left_leg_joint', 'rear_left_foot_joint',
            'rear_right_shoulder_joint', 'rear_right_leg_joint', 'rear_right_foot_joint'
        ]
        self.get_logger().info('Simple Unified Gait Node Started')

    def cmd_vel_callback(self, msg):
        self.cmd_vx, self.cmd_vy, self.cmd_omega = msg.linear.x, msg.linear.y, msg.angular.z

    def timer_callback(self):
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        if self.init_t is None: self.init_t = t
        elapsed = t - self.init_t
        
        joint_angles = []
        # Stand in '>' shape (target z = -0.14)
        if elapsed < 10.0 or (abs(self.cmd_vx) < 0.001 and abs(self.cmd_vy) < 0.001):
            target_z = -0.14
            for i in range(4):
                res = self.kin.ik(0.0, 0.0, target_z, leg_id=i)
                if res: joint_angles.extend(res)
                else: joint_angles.extend([0.0, -0.7, 1.4])
        else:
            joint_angles = self.planner.get_walk_posture(self.cmd_vx, self.cmd_vy, self.cmd_omega, elapsed)
            
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.nanosec = 20000000 
        msg.points.append(point)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
