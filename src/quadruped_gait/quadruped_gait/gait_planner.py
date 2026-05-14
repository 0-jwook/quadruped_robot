import math

class GaitPlanner:
    """
    Wave Gait + Body Shift 플래너 (SpotMicro 방식 참고).

    한 번에 한 다리씩만 스윙하며(duty_factor=0.75), 스윙 직전에
    몸통을 지지 삼각형 쪽으로 미리 이동(Body Shift)시켜
    무게중심이 항상 지지 삼각형 안에 유지되도록 합니다.

    스윙 순서: FR → RR → FL → RL
    """
    def __init__(self, kinematics):
        self.kin = kinematics

        # --- [물리 파라미터] ---
        # 실제 하드웨어: L2=0.075m, L3=0.095m → 최대 도달 0.17m
        self.body_height = 0.13
        self.step_height = 0.035  # SpotMicro 50mm를 다리 길이 비율로 스케일링
        self.period      = 1.5    # 한 사이클 (swing_time ≈ 0.375s/다리)

        # --- [Wave Gait 설정] ---
        # Duty Factor 0.75: 항상 3개 다리가 지면 지지 → 안정적 지지 삼각형
        self.duty_factor = 0.75
        # 스윙 순서: FR(0~25%) → RR(25~50%) → FL(50~75%) → RL(75~100%)
        self.leg_phases  = [0.25, 0.75, 0.0, 0.5]   # [FL, FR, RL, RR]

        self.front_x_offset = 0.04
        self.rear_x_offset  = -0.01
        self.max_stride     = 0.03

        # --- [Body Shift 설정] ---
        # 각 다리 스윙 직전 몸통을 앞으로 이동시켜 무게중심을 지지 삼각형 안으로 유도.
        # y축(좌우) 이동은 다리 좌우 흔들림을 유발하므로 0으로 설정.
        # y축 이동은 좌우 흔들림을 유발하므로 제거, x축(전방)만 유지
        self._leg_shifts = [
            (+0.012, 0.0),   # FL
            (+0.012, 0.0),   # FR
            (+0.018, 0.0),   # RL
            (+0.018, 0.0),   # RR
        ]
        self._pre_swing_blend  = 0.10  # 스윙 10% 전부터 미리 이동 시작
        self._post_swing_blend = 0.05  # 스윙 끝 5% 구간에서 선형 감소 (순간 급락 방지)

        # 중립 자세 기준값: body_height=0.13m, L1=0.042, L2=0.075, L3=0.095
        self.Q2_NEUTRAL = -0.5408
        self.Q3_NEUTRAL =  1.3479
        self.last_angles = [[0.0, self.Q2_NEUTRAL, self.Q3_NEUTRAL] for _ in range(4)]

    # ──────────────────────────────────────────────────────────────────────────
    def _body_shift(self, phi):
        """
        현재 위상 phi에서 몸통 이동량 (bs_x, bs_y) 반환.

        pre_swing 구간: 선형 증가 (스윙 10% 전부터)
        swing 구간:    최대값 유지
        post_swing 구간: 선형 감소 (스윙 끝 5% 구간)
          → 스탠스 전환 시 순간 급락을 방지해 덜컹거림 제거.
        """
        bs_x, bs_y = 0.0, 0.0
        pre  = self._pre_swing_blend
        post = self._post_swing_blend
        swing_dur = 1.0 - self.duty_factor          # = 0.25
        post_start = self.duty_factor + swing_dur - post  # = 0.95

        for i in range(4):
            leg_phi = (phi + self.leg_phases[i]) % 1.0

            if leg_phi >= post_start:
                # 스윙 후반부: 1.0 → 0.0 선형 감소
                weight = (1.0 - leg_phi) / post
            elif leg_phi >= self.duty_factor:
                weight = 1.0                                         # 스윙 중반: 완전 적용
            elif leg_phi >= (self.duty_factor - pre):
                weight = (leg_phi - (self.duty_factor - pre)) / pre  # 준비 구간: 선형 증가
            else:
                weight = 0.0

            bs_x += self._leg_shifts[i][0] * weight
            bs_y += self._leg_shifts[i][1] * weight

        return bs_x, bs_y

    # ──────────────────────────────────────────────────────────────────────────
    def get_stand_posture(self, roll=0.0, pitch=0.0, body_height=None):
        """정지 자세 (IMU 피드백 반영)."""
        bh = body_height if body_height is not None else self.body_height
        joint_angles = []
        kp_roll  = 0.8
        kp_pitch = 1.5

        for i in range(4):
            leg_x = 0.2 if i < 2 else -0.2
            leg_y = 0.1 if (i == 0 or i == 2) else -0.1

            z_balance = -(leg_x * math.sin(pitch) * kp_pitch
                          - leg_y * math.sin(roll)  * kp_roll)

            target_x = self.front_x_offset if i < 2 else self.rear_x_offset
            target_y = self.kin.L1 if (i == 0 or i == 2) else -self.kin.L1
            target_z = -bh + z_balance

            res = self.kin.ik(target_x, target_y, target_z, leg_id=i)
            if res:
                self.last_angles[i] = list(res)
            joint_angles.extend(self.last_angles[i])

        return joint_angles

    # ──────────────────────────────────────────────────────────────────────────
    def get_walk_posture(self, vx, vy, omega, t, roll=0.0, pitch=0.0, body_height=None):
        """보행 자세 계산 (Wave Gait + Body Shift + IMU 피드백)."""
        bh  = body_height if body_height is not None else self.body_height
        phi = (t % self.period) / self.period
        joint_angles = []

        kp_roll  = 0.5
        kp_pitch = 1.0

        # ── Body Shift 계산 ─────────────────────────────────────────────────
        bs_x, bs_y = self._body_shift(phi)

        for i in range(4):
            leg_phi  = (phi + self.leg_phases[i]) % 1.0
            side_sign = 1.0 if (i == 0 or i == 2) else -1.0
            base_y    = self.kin.L1 * side_sign
            anchor_x  = self.front_x_offset if i < 2 else self.rear_x_offset

            leg_x_pos = 0.2 if i < 2 else -0.2
            leg_y_pos = 0.1 * side_sign

            z_balance = -(leg_x_pos * math.sin(pitch) * kp_pitch
                          - leg_y_pos * math.sin(roll)  * kp_roll)

            # ── 보폭 계산 ────────────────────────────────────────────────────
            stride_x = max(-self.max_stride, min(self.max_stride,
                           vx * self.period * (1.0 - self.duty_factor)))
            stride_y = max(-self.max_stride, min(self.max_stride,
                           vy * self.period * (1.0 - self.duty_factor)))

            turn_r = 0.15
            stride_yaw_x = -omega * turn_r * side_sign        * (self.period * (1.0 - self.duty_factor))
            stride_yaw_y =  omega * turn_r * (1.0 if i < 2 else -1.0) * (self.period * (1.0 - self.duty_factor))

            total_stride_x = stride_x + stride_yaw_x
            total_stride_y = stride_y + stride_yaw_y

            # ── Stance / Swing 궤적 ──────────────────────────────────────────
            if leg_phi < self.duty_factor:          # STANCE
                s      = leg_phi / self.duty_factor
                step_x = anchor_x + (0.5 - s) * total_stride_x
                step_y = base_y   + (0.5 - s) * total_stride_y
                step_z = -bh + z_balance
            else:                                   # SWING
                s      = (leg_phi - self.duty_factor) / (1.0 - self.duty_factor)
                step_x = anchor_x + (s - 0.5) * total_stride_x
                step_y = base_y   + (s - 0.5) * total_stride_y
                step_z = -bh + self.step_height * math.sin(s * math.pi) + z_balance

            # ── Body Shift 적용 ───────────────────────────────────────────────
            # 몸통이 +bs_x 앞으로 이동 ↔ 발 타겟이 body frame에서 -bs_x 뒤로
            step_x -= bs_x
            step_y -= bs_y

            res = self.kin.ik(step_x, step_y, step_z, leg_id=i)
            if res:
                self.last_angles[i] = list(res)

            joint_angles.extend(self.last_angles[i])

        return joint_angles
