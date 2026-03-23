import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# 분리된 파일에서 클래스 임포트
from .kinematics import LegKinematics
from .gait_planner import GaitPlanner

class GaitNode(Node):
    def __init__(self):
        super().__init__('gait_node')
        
        # 하드웨어 파라미터 초기화
        self.kin = LegKinematics(L1=0.08, L2=0.2, L3=0.2)
        self.planner = GaitPlanner(self.kin)
        
        # ROS2 통신 설정
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # 타이머 주기 설정 (50Hz = 0.02s)
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)
        
        # 상태 변수
        self.cmd_vx, self.cmd_vy, self.cmd_omega = 0.0, 0.0, 0.0
        self.init_t = None
        
        self.joint_names = [
            'front_left_shoulder_joint', 'front_left_leg_joint', 'front_left_foot_joint',
            'front_right_shoulder_joint', 'front_right_leg_joint', 'front_right_foot_joint',
            'rear_left_shoulder_joint', 'rear_left_leg_joint', 'rear_left_foot_joint',
            'rear_right_shoulder_joint', 'rear_right_leg_joint', 'rear_right_foot_joint'
        ]
        
        self.get_logger().info('Quadruped Gait Node with Bezier & Raibert logic started.')

    def cmd_vel_callback(self, msg):
        """속도 명령 수신"""
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_omega = msg.angular.z

    def timer_callback(self):
        """메인 제어 루프"""
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        
        if self.init_t is None:
            self.init_t = t
            return
            
        elapsed = t - self.init_t
        
        # 1. 속도 명령 유무에 따른 자세 결정 (데드존 적용)
        is_moving = abs(self.cmd_vx) > 0.01 or abs(self.cmd_vy) > 0.01 or abs(self.cmd_omega) > 0.01
        
        if not is_moving:
            joint_angles = self.planner.get_stand_posture()
        else:
            joint_angles = self.planner.get_walk_posture(self.cmd_vx, self.cmd_vy, self.cmd_omega, elapsed)

        # 2. 메시지 생성 및 발행
        msg = JointTrajectory()
        msg.header.stamp = now.to_msg()
        msg.joint_names = self.joint_names
        
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.velocities = [0.0] * 12 # Effort 제어기 호환성
        
        # 보간을 위해 주기보다 약간 길게 도달 시간 설정
        duration = self.dt * 1.5
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(duration * 1e9)
        
        msg.points.append(point)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = GaitNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()