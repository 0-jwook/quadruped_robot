import math

class GaitPlanner:
    """
    한 번에 한 다리만 움직여 극강의 안정성을 제공하면서도,
    어깨 관절(Shoulder)을 활용해 회전과 횡이동이 가능한 Wave Gait 플래너입니다.
    보폭 제한을 제거하여 입력 속도에 따른 역동적인 움직임이 가능합니다.
    """
    def __init__(self, kinematics):
        self.kin = kinematics
        
        # --- [물리 파라미터] ---
        self.body_height = 0.27
        self.step_height = 0.1    
        self.period = 1.75          
        
        # --- [Wave Gait 핵심 설정] ---
        # Duty Factor 0.75: 세 다리가 땅을 짚고 한 다리만 이동 (안정성 극대화)
        self.duty_factor = 0.75    
        
        # 다리별 페이즈 오프셋 (LH -> LF -> RH -> RF 순차 이동)
        self.leg_phases = [0.25, 0.75, 0.0, 0.5] 
        
        # --- [무게중심 안정성 설정] ---
        # front_x_offset: 앞발 기준점을 어깨 관절보다 전방으로 배치.
        # Wave gait에서 앞발 1개가 Swing 중일 때 지지 삼각형(나머지 3발)의
        # 무게중심이 삼각형 안에 들어오려면 기하학적으로 ≥0.16m 이 필요.
        # (front_x_offset=0일 때 CoM이 지지 삼각형 경계선 위 → 윌리 발생)
        self.front_x_offset = 0.17   # 앞발: 어깨보다 17cm 전방 (윌리 방지 핵심값)
        self.rear_x_offset  = 0.0    # 뒷발: 어깨 바로 아래 (기본값 유지)

        # 고속 이동 시 IK 범위 초과 방지 (front_x_offset=0.17 기준 최대 도달 한계)
        self.max_stride = 0.22       # 보폭 상한 (m)
        
        # 중립 자세(기립) 기준값: body_height=0.27, L2=L3=0.2, L1=0.08 기준으로 계산된 값
        # 하드웨어 서보 각도 변환의 기준점으로도 사용됨
        self.Q2_NEUTRAL = -0.8327  # rad
        self.Q3_NEUTRAL =  1.6597  # rad
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