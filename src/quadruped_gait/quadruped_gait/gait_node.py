import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32, String
import math
import time

# 분리된 모듈
from .kinematics import LegKinematics
from .gait_planner import GaitPlanner
from .body_pose import BodyPoseController
from .gestures import GesturePlayer, gesture_names


def euler_from_quaternion(q):
    """Quaternion(x,y,z,w) → Euler(roll, pitch, yaw)"""
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = max(-1.0, min(1.0, t2))
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


class GaitNode(Node):
    """
    모드 관리 노드. 우선순위:  GESTURE > BODY_POSE > WALK > STAND
      /cmd_vel    (Twist)  → 보행
      /body_pose  (Twist)  → 발 고정 몸통 6축 (linear=이동, angular=rpy)
      /gesture    (String) → 제스처 재생
      /body_height_cmd (Float32) → 높이
    시작 시 SIT → STAND 로 점진 ramp.
    """

    def __init__(self):
        super().__init__('gait_node')

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter('L1', 0.030)
        self.declare_parameter('L2', 0.115)
        self.declare_parameter('L3', 0.135)
        self.declare_parameter('body_height', 0.14)
        self.declare_parameter('step_height', 0.05)
        self.declare_parameter('max_stride',  0.05)
        self.declare_parameter('period',      0.8)
        self.declare_parameter('duty_trot',   0.6)
        self.declare_parameter('duty_wave',   0.75)
        self.declare_parameter('hip_x',       0.1225)
        self.declare_parameter('hip_y',       0.10)
        self.declare_parameter('level_gain',  1.0)
        self.declare_parameter('level_max',   0.09)
        self.declare_parameter('level_lpf_tau', 0.6)   # 보행 중 leveling LPF (경사만 반응)
        self.declare_parameter('height_min',  0.07)
        self.declare_parameter('height_max',  0.21)
        self.declare_parameter('gait_type',   'trot')
        self.declare_parameter('cmd_vel_hold_time', 30.0)
        self.declare_parameter('pitch_offset', 0.0)
        self.declare_parameter('roll_offset',  0.0)
        self.declare_parameter('startup_ramp_time', 3.0)  # SIT→STAND ramp 시간
        self.declare_parameter('yaw_trim', 0.0)  # 직진 휨 보정 (rad/s). 좌측 휨이면 음수(우향 보정)
        # 넘어짐 감지 / 자동 기립 (IMU roll/pitch 필요)
        self.declare_parameter('fall_detect', True)        # 넘어짐 감지 on/off
        self.declare_parameter('fall_tilt_thresh', 1.0)    # rad. roll/pitch 이 이상 기울면 넘어짐 (~57°)
        self.declare_parameter('fall_hold_time', 0.5)      # s. 이 시간 이상 유지돼야 넘어짐 확정
        self.declare_parameter('auto_recover', True)       # 넘어지면 자동 기립 시도
        self.declare_parameter('recover_time', 3.0)        # s. 기립 시퀀스 길이

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
        lvl_gain = self.get_parameter('level_gain').value
        lvl_max  = self.get_parameter('level_max').value
        self._level_lpf_tau = self.get_parameter('level_lpf_tau').value
        self._cmd_vel_hold_time = self.get_parameter('cmd_vel_hold_time').value
        self._pitch_offset = self.get_parameter('pitch_offset').value
        self._roll_offset  = self.get_parameter('roll_offset').value
        self._ramp_time = self.get_parameter('startup_ramp_time').value
        self._yaw_trim = self.get_parameter('yaw_trim').value
        self._fall_detect = self.get_parameter('fall_detect').value
        self._fall_tilt_thresh = self.get_parameter('fall_tilt_thresh').value
        self._fall_hold = self.get_parameter('fall_hold_time').value
        self._auto_recover = self.get_parameter('auto_recover').value
        self._recover_time = self.get_parameter('recover_time').value

        self.kin     = LegKinematics(L1=L1, L2=L2, L3=L3)
        self.planner = GaitPlanner(self.kin,
                                   body_height=bh, step_height=sh,
                                   max_stride=ms, period=p, gait_type=gt,
                                   duty_trot=dt_trot, duty_wave=dt_wave,
                                   hip_x=hip_x, hip_y=hip_y,
                                   level_gain=lvl_gain, level_max=lvl_max)
        self.body_pose = BodyPoseController(self.kin, hip_x=hip_x, hip_y=hip_y,
                                            body_height=bh,
                                            level_gain=lvl_gain, level_max=lvl_max)
        self.gesture = GesturePlayer(self.body_pose, dt=0.02)

        self.get_logger().info(
            f'Gait: {gt}, period={p}s, max_speed≈{self.planner.max_speed():.3f} m/s | '
            f'gestures: {gesture_names()}')

        # ── 통신 ──────────────────────────────────────────────────
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(Twist, '/body_pose', self.body_pose_callback, 10)
        self.create_subscription(String, '/gesture', self.gesture_callback, 10)
        self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.create_subscription(Float32, '/body_height_cmd', self.height_callback, 10)
        self.publisher = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.timer_callback)

        # ── 상태 ──────────────────────────────────────────────────
        self.cmd_vx = self.cmd_vy = self.cmd_omega = 0.0
        self._last_cmd_time = 0.0
        self._walk_active = False

        # body pose 명령 (발 고정 몸통)
        self.pose_cmd = None            # dict 또는 None
        self._last_pose_time = 0.0
        self._pose_hold = 1.0           # pose 명령 유효시간 (s)

        self.roll = self.pitch = self.yaw = 0.0
        self._roll_lpf = self._pitch_lpf = 0.0   # leveling 용 LPF 상태

        self.init_t = None
        self.target_body_height  = bh
        self.current_body_height = bh
        self.height_rate = 0.005
        self._stand_bh = bh

        # 시작 ramp: SIT → STAND
        self._ramp_done = (self._ramp_time <= 0.0)
        self._sit_height = 0.085        # SIT 자세 높이 (MCU SIT 와 맞춤)
        # 넘어짐/자동기립 상태
        self._fall_acc = 0.0            # 기울임 누적 시간
        self._recovering = False        # 기립 시퀀스 진행 중
        self._recover_t0 = 0.0
        self._recover_low = 0.07        # 기립 중 웅크리는 최저 높이

        self.joint_names = [
            'front_left_shoulder_joint', 'front_left_leg_joint', 'front_left_foot_joint',
            'front_right_shoulder_joint', 'front_right_leg_joint', 'front_right_foot_joint',
            'rear_left_shoulder_joint', 'rear_left_leg_joint', 'rear_left_foot_joint',
            'rear_right_shoulder_joint', 'rear_right_leg_joint', 'rear_right_foot_joint'
        ]
        self.get_logger().info('Quadruped Gait Node (modes: walk/pose/gesture) started.')

    # ── 콜백 ──────────────────────────────────────────────────────
    def cmd_vel_callback(self, msg):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_omega = msg.angular.z
        self._last_cmd_time = time.monotonic()
        self._walk_active = True

    def body_pose_callback(self, msg):
        """발 고정 몸통 6축. linear=(dx,dy,dz), angular=(roll,pitch,yaw)."""
        self.pose_cmd = dict(dx=msg.linear.x, dy=msg.linear.y, dz=msg.linear.z,
                             roll=msg.angular.x, pitch=msg.angular.y, yaw=msg.angular.z)
        self._last_pose_time = time.monotonic()

    def gesture_callback(self, msg):
        name = msg.data.strip()
        if self.gesture.start(name, current_height=self.current_body_height):
            self.get_logger().info(f'제스처 시작: {name}')
        else:
            self.get_logger().warn(f'알 수 없는 제스처: "{name}" (가능: {gesture_names()})')

    def imu_callback(self, msg):
        self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.orientation)

    def height_callback(self, msg):
        self.target_body_height = max(self._height_min, min(self._height_max, float(msg.data)))

    # ── 메인 루프 ─────────────────────────────────────────────────
    def timer_callback(self):
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        if self.init_t is None:
            self.init_t = t
            return
        elapsed = t - self.init_t

        # 몸통 높이 부드러운 전환
        diff = self.target_body_height - self.current_body_height
        if abs(diff) > 0.001:
            self.current_body_height += math.copysign(min(self.height_rate, abs(diff)), diff)

        # leveling 입력: roll/pitch 에 LPF (경사=저주파 반응, 보행 흔들림=고주파 무시)
        a = self.dt / (self._level_lpf_tau + self.dt)
        self._roll_lpf  += (self.roll  - self._roll_lpf)  * a
        self._pitch_lpf += (self.pitch - self._pitch_lpf) * a
        roll_eff  = self._roll_lpf  + self._roll_offset
        pitch_eff = self._pitch_lpf + self._pitch_offset

        # ── 시작 ramp: SIT → STAND (다른 모드보다 우선) ──
        if not self._ramp_done:
            r = min(1.0, elapsed / self._ramp_time)
            ease = 0.5 * (1.0 - math.cos(math.pi * r))
            bh = self._sit_height + (self._stand_bh - self._sit_height) * ease
            joint_angles = self.planner.get_stand_posture(0.0, 0.0, bh)
            if r >= 1.0:
                self._ramp_done = True
                self.current_body_height = self._stand_bh
                # ramp 중 버퍼된 명령 무효화 (기립 직후 의도치 않은 보행 방지)
                self._walk_active = False
                self.pose_cmd = None
                self.get_logger().info('기립 ramp 완료 → 정상 동작')
            self._publish(joint_angles, now)
            return

        # ── 넘어짐 감지 + 자동 기립 (IMU tilt 기반, 최우선) ──
        if self._fall_detect and not self._recovering:
            tilt = max(abs(self.roll), abs(self.pitch))
            self._fall_acc = self._fall_acc + self.dt if tilt > self._fall_tilt_thresh else 0.0
            if self._fall_acc >= self._fall_hold:
                self.get_logger().warn(
                    f'넘어짐 감지 (tilt={math.degrees(tilt):.0f}°)'
                    + (' → 자동 기립' if self._auto_recover else ''))
                self._fall_acc = 0.0
                if self._auto_recover:
                    self._recovering = True
                    self._recover_t0 = elapsed
        if self._recovering:
            rt = elapsed - self._recover_t0
            if rt >= self._recover_time:
                self._recovering = False        # 기립 완료 → 일반 동작 복귀
            else:
                frac = rt / self._recover_time
                if frac < 0.4:                   # ① 웅크리기 (다리 모음)
                    h = self._stand_bh + (self._recover_low - self._stand_bh) * (frac / 0.4)
                else:                            # ② 밀어 올려 STAND
                    h = self._recover_low + (self._stand_bh - self._recover_low) * ((frac - 0.4) / 0.6)
                self.current_body_height = h
                self._walk_active = False
                self._publish(self.planner.get_stand_posture(0.0, 0.0, h), now)
                return

        # ── 모드 우선순위: GESTURE > BODY_POSE > WALK > STAND ──
        gesture_res = self.gesture.step(self.current_body_height) if self.gesture.is_active() else None
        pose_active = (self.pose_cmd is not None and
                       (time.monotonic() - self._last_pose_time) < self._pose_hold)
        since_last = time.monotonic() - self._last_cmd_time
        walking = self._walk_active and (since_last < self._cmd_vel_hold_time)

        if gesture_res is not None:
            joint_angles, gh, height_active = gesture_res
            self.current_body_height = gh        # 제스처 높이 추종
            if height_active:
                # height 키프레임(sit/lie/ready)일 때만 목표 높이 갱신.
                # pose 제스처는 높이 안 건드림 → 진행 중 /body_height_cmd 보존.
                self.target_body_height = gh
        elif pose_active:
            pc = self.pose_cmd
            # leveling(roll_eff/pitch_eff)을 z보정으로 적용 → POSE 중립이 WALK stand 와 일치
            joint_angles = self.body_pose.get_pose_posture(
                dx=pc['dx'], dy=pc['dy'], dz=pc['dz'],
                roll=pc['roll'], pitch=pc['pitch'], yaw=pc['yaw'],
                body_height=self.current_body_height,
                lvl_roll=roll_eff, lvl_pitch=pitch_eff)
        elif walking:
            # 회전(omega)은 병진(전후/측방) 중에만 적용 → 제자리 회전 금지, 전진+회전 = 호 회전.
            # 병진 중엔 yaw_trim(직진 휨 보정)도 함께 더함.
            if abs(self.cmd_vx) > 0.005 or abs(self.cmd_vy) > 0.005:
                omega_eff = self.cmd_omega + self._yaw_trim
            else:
                omega_eff = 0.0   # 병진 없으면 회전 무시 (제자리 회전 안 됨)
            joint_angles = self.planner.get_walk_posture(
                self.cmd_vx, self.cmd_vy, omega_eff, elapsed,
                roll_eff, pitch_eff, self.current_body_height)
        else:
            self._walk_active = False
            joint_angles = self.planner.get_stand_posture(
                roll_eff, pitch_eff, self.current_body_height)

        self._publish(joint_angles, now)

    def _publish(self, joint_angles, now):
        msg = JointTrajectory()
        msg.header.stamp = now.to_msg()
        msg.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = list(joint_angles)
        point.velocities = [0.0] * 12
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
