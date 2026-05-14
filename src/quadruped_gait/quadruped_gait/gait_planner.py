import math

class GaitPlanner:
    """
    대각선 두 다리가 동시에 움직이는 Trot Gait 플래너입니다.
    FL+RR 쌍과 FR+RL 쌍이 교대로 스윙하며, 항상 대각 지지선 위에
    무게중심이 위치하여 후방 편향 로봇에서도 안정적으로 전진합니다.
    """
    def __init__(self, kinematics):
        self.kin = kinematics

        # --- [물리 파라미터] ---
        # 실제 하드웨어: L2=0.075m, L3=0.095m → 최대 도달 0.17m
        self.body_height = 0.13   # 기본 자세: 최대 도달의 76% (무릎 자연스럽게 굽힘)
        self.step_height = 0.030  # Trot: 두 다리 동시 스윙이라 지면 여유 필요
        self.period = 1.0         # Trot 주기 (Wave 1.75s → Trot 1.0s)

        # --- [Trot Gait 핵심 설정] ---
        # Duty Factor 0.5: 대각선 두 다리 동시 스윙 (FL+RR / FR+RL)
        # 항상 대각 지지선 위에 CoM이 위치 → 후방 편향 로봇에 유리
        self.duty_factor = 0.5
        self.leg_phases  = [0.0, 0.5, 0.5, 0.0]  # FL+RR 동기 / FR+RL 동기

        # Trot은 대각 지지라 Wave만큼 전방 편향이 필요 없음
        self.front_x_offset = 0.04   # 앞발: 어깨보다 4cm 전방
        self.rear_x_offset  = -0.01  # 뒷발: 어깨보다 1cm 후방

        # L2+L3=0.17m, h=0.13m → 수평 최대 도달 = sqrt(0.17²-0.13²) = 0.109m
        self.max_stride = 0.03      # 보폭 상한 (m)

        # 중립 자세(기립) 기준값: body_height=0.13m, L1=0.042, L2=0.075, L3=0.095 실측 계산값
        self.Q2_NEUTRAL = -0.5408  # rad
        self.Q3_NEUTRAL =  1.3479  # rad
        self.last_angles = [[0.0, self.Q2_NEUTRAL, self.Q3_NEUTRAL] for _ in range(4)]

    def get_stand_posture(self, roll=0.0, pitch=0.0, body_height=None):
        """정지 상태 자세 (IMU 피드백 반영).
        body_height: 외부에서 전달된 목표 높이(m). None이면 기본값 사용.
        """
        bh = body_height if body_height is not None else self.body_height
        joint_angles = []
        # 피드백 게인 (pitch 게인을 높여 뒤로 쏠림 즉시 복원)
        kp_roll = 0.8
        kp_pitch = 1.5

        for i in range(4):
            leg_x = 0.2 if i < 2 else -0.2
            leg_y = 0.1 if (i == 0 or i == 2) else -0.1

            # IMU 기반 수평 유지 보정 (자세 제어)
            z_balance = -(leg_x * math.sin(pitch) * kp_pitch - leg_y * math.sin(roll) * kp_roll)

            target_x = self.front_x_offset if i < 2 else self.rear_x_offset
            target_y = self.kin.L1 if (i == 0 or i == 2) else -self.kin.L1
            target_z = -bh + z_balance

            res = self.kin.ik(target_x, target_y, target_z, leg_id=i)
            if res:
                self.last_angles[i] = list(res)
            joint_angles.extend(self.last_angles[i])
        return joint_angles

    def get_walk_posture(self, vx, vy, omega, t, roll=0.0, pitch=0.0, body_height=None):
        """회전(omega)과 횡이동(vy) 로직을 포함하며, IMU 피드백을 통해 동적 안정을 꾀함.
        body_height: 외부에서 전달된 목표 높이(m). None이면 기본값 사용.
        """
        bh = body_height if body_height is not None else self.body_height
        phi = (t % self.period) / self.period
        joint_angles = []

        # 피드백 게인 (보행 중 pitch 복원력 강화)
        kp_roll = 0.5
        kp_pitch = 1.0

        for i in range(4):
            leg_phi = (phi + self.leg_phases[i]) % 1.0
            side_sign = 1.0 if (i == 0 or i == 2) else -1.0
            base_y = self.kin.L1 * side_sign
            anchor_x = self.front_x_offset if i < 2 else self.rear_x_offset

            leg_x_pos = 0.2 if i < 2 else -0.2
            leg_y_pos = 0.1 * side_sign

            # IMU 기반 수평 유지 보정
            z_balance = -(leg_x_pos * math.sin(pitch) * kp_pitch - leg_y_pos * math.sin(roll) * kp_roll)

            # --- [보폭 및 회전 계산] ---
            stride_x = vx * (self.period * (1.0 - self.duty_factor))
            stride_y = vy * (self.period * (1.0 - self.duty_factor))
            # front_x_offset=0.17 기준 IK 최대 도달 범위 초과 방지
            stride_x = max(-self.max_stride, min(self.max_stride, stride_x))
            stride_y = max(-self.max_stride, min(self.max_stride, stride_y))

            turn_radius = 0.15
            stride_yaw_x = -omega * turn_radius * side_sign * (self.period * (1.0 - self.duty_factor))
            stride_yaw_y = omega * turn_radius * (1.0 if i < 2 else -1.0) * (self.period * (1.0 - self.duty_factor))

            total_stride_x = stride_x + stride_yaw_x
            total_stride_y = stride_y + stride_yaw_y

            if leg_phi < self.duty_factor:  # STANCE (지지기)
                s = leg_phi / self.duty_factor
                step_x = anchor_x + (0.5 - s) * total_stride_x
                step_y = base_y + (0.5 - s) * total_stride_y
                step_z = -bh + z_balance
            else:  # SWING (공중 이동기)
                s = (leg_phi - self.duty_factor) / (1.0 - self.duty_factor)
                step_x = anchor_x + (s - 0.5) * total_stride_x
                step_y = base_y + (s - 0.5) * total_stride_y
                step_z = -bh + self.step_height * math.sin(s * math.pi) + z_balance

            res = self.kin.ik(step_x, step_y, step_z, leg_id=i)
            if res:
                self.last_angles[i] = list(res)

            joint_angles.extend(self.last_angles[i])

        return joint_angles