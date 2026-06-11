import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import math
import time

# 분리된 파일에서 클래스 임포트
from .kinematics import LegKinematics
from .gait_planner import GaitPlanner

def euler_from_quaternion(q):
    """
    Quaternion(x, y, z, w)를 Euler(roll, pitch, yaw)로 변환
    """
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw

class GaitNode(Node):
    def __init__(self):
        super().__init__('gait_node')

        # ── 다리 치수 파라미터 (하드웨어 기본값; 시뮬레이션은 launch 파일에서 재정의)
        # 실제 로봇 치수: L1=3cm, L2=11.5cm, L3=13.5cm (최대 도달 25cm)
        self.declare_parameter('L1', 0.030)
        self.declare_parameter('L2', 0.115)
        self.declare_parameter('L3', 0.135)
        self.declare_parameter('body_height', 0.17)
        self.declare_parameter('step_height', 0.04)
        self.declare_parameter('max_stride',  0.05)
        self.declare_parameter('period',      0.5)    # 전체 cycle 시간 (Tstride)
        self.declare_parameter('duty_trot',   0.6)    # trot stance 비율 (≥0.5 면 비행 없음)
        self.declare_parameter('duty_wave',   0.75)   # wave stance 비율 (3-leg 지지)
        self.declare_parameter('hip_x',       0.1225) # 몸통중심~발 종방향 = BODY_L/2 (URDF 실측)
        self.declare_parameter('hip_y',       0.10)   # 몸통중심~발 횡방향 = BODY_W/2 + L1 (URDF 실측)
        self.declare_parameter('height_min',  0.11)
        self.declare_parameter('height_max',  0.21)
        self.declare_parameter('gait_type',   'trot')
        self.declare_parameter('cmd_vel_hold_time', 30.0)
        # 고정 자세 보정.
        # pitch_offset: + → 앞쪽 올라감, - → 앞쪽 내려감
        # roll_offset:  + → 우측 올라감(좌측 내려감), - → 좌측 올라감(우측 내려감)
        # 1° ≈ 0.0175 rad.
        self.declare_parameter('pitch_offset', 0.02)
        self.declare_parameter('roll_offset',  0.0)

        L1 = self.get_parameter('L1').value
        L2 = self.get_parameter('L2').value
        L3 = self.get_parameter('L3').value
        bh = self.get_parameter('body_height').value
        sh = self.get_parameter('step_height').value
        ms = self.get_parameter('max_stride').value
        p  = self.get_parameter('period').value
        self._height_min = self.get_parameter('height_min').value
        self._height_max = self.get_parameter('height_max').value
        gt = self.get_parameter('gait_type').value
        dt_trot = self.get_parameter('duty_trot').value
        dt_wave = self.get_parameter('duty_wave').value
        hip_x = self.get_parameter('hip_x').value
        hip_y = self.get_parameter('hip_y').value
        self._cmd_vel_hold_time = self.get_parameter('cmd_vel_hold_time').value
        self._pitch_offset = self.get_parameter('pitch_offset').value
        self._roll_offset  = self.get_parameter('roll_offset').value

        self.kin     = LegKinematics(L1=L1, L2=L2, L3=L3)
        self.planner = GaitPlanner(self.kin,
                                   body_height=bh, step_height=sh,
                                   max_stride=ms, period=p, gait_type=gt,
                                   duty_trot=dt_trot, duty_wave=dt_wave,
                                   hip_x=hip_x, hip_y=hip_y)
        self.get_logger().info(
            f'Gait: {gt}, period={p}s, duty_trot={dt_trot}, '
            f'max_speed≈{self.planner.max_speed():.3f} m/s')

        # ROS2 통신 설정
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.height_sub = self.create_subscription(Float32, '/body_height_cmd', self.height_callback, 10)
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # SpotMicro 방식: cmd_vel을 한 번 받으면 Walk 모드 진입 → phase cycle 계속 돔
        # cmd_vel=0이어도 hold_time 안에는 walk_posture 호출 → 제자리 걷기
        self.cmd_vx, self.cmd_vy, self.cmd_omega = 0.0, 0.0, 0.0
        self._last_cmd_time = 0.0
        self._walk_active = False    # Walk 상태 플래그 (SpotMicro FSM 의 Walk state 진입 여부)
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0
        self.init_t = None

        self.target_body_height  = bh
        self.current_body_height = bh
        self.height_rate = 0.005
        
        self.joint_names = [
            'front_left_shoulder_joint', 'front_left_leg_joint', 'front_left_foot_joint',
            'front_right_shoulder_joint', 'front_right_leg_joint', 'front_right_foot_joint',
            'rear_left_shoulder_joint', 'rear_left_leg_joint', 'rear_left_foot_joint',
            'rear_right_shoulder_joint', 'rear_right_leg_joint', 'rear_right_foot_joint'
        ]
        
        self.get_logger().info('Quadruped Gait Node with IMU feedback started.')

    def cmd_vel_callback(self, msg):
        """속도 명령 수신 — SpotMicro Walk 모드 진입 트리거"""
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_omega = msg.angular.z
        self._last_cmd_time = time.monotonic()
        # 어떤 cmd_vel 메시지든 한 번 받으면 Walk 모드 진입.
        # (값이 0이어도 walk_posture를 계속 호출해서 SpotMicro 영상처럼 제자리 걷기)
        self._walk_active = True

    def imu_callback(self, msg):
        """IMU 데이터 수신 (자세 제어를 위한 Roll, Pitch 추출)"""
        self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.orientation)

    def height_callback(self, msg):
        """몸체 높이 명령 수신 (teleop_key의 t/b 키)"""
        self.target_body_height = max(self._height_min, min(self._height_max, float(msg.data)))

    def timer_callback(self):
        """메인 제어 루프"""
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        
        if self.init_t is None:
            self.init_t = t
            return
            
        elapsed = t - self.init_t
        
        # 높이 부드럽게 전환 (0.25 m/s)
        diff = self.target_body_height - self.current_body_height
        if abs(diff) > 0.001:
            self.current_body_height += math.copysign(min(self.height_rate, abs(diff)), diff)

        # SpotMicro FSM 모사:
        #   Walk 상태: cmd_vel 수신 → hold_time 내 → phase cycle 계속 (값 0이면 제자리 걷기)
        #   Stand 상태: hold_time 초과 시 자동 복귀
        since_last = time.monotonic() - self._last_cmd_time
        walking = self._walk_active and (since_last < self._cmd_vel_hold_time)

        # IMU roll/pitch 에 고정 offset 더하기 (실제 로봇 기울임 보정).
        # IMU 없을 때 self.roll=pitch=0 이라 offset 만 적용됨.
        pitch_eff = self.pitch + self._pitch_offset
        roll_eff  = self.roll  + self._roll_offset

        if walking:
            joint_angles = self.planner.get_walk_posture(
                self.cmd_vx, self.cmd_vy, self.cmd_omega, elapsed,
                roll_eff, pitch_eff, self.current_body_height)
        else:
            self._walk_active = False
            joint_angles = self.planner.get_stand_posture(
                roll_eff, pitch_eff, self.current_body_height)

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