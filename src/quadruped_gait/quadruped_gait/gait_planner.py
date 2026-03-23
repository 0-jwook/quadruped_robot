import math

class GaitPlanner:
    """
    한 번에 한 다리만 움직여 극강의 안정성을 제공하는 Wave Gait 플래너입니다.
    """
    def __init__(self, kinematics):
        self.kin = kinematics
        
        # --- [물리 파라미터] ---
        self.body_height = 0.27
        self.step_height = 0.06    # 한 다리씩 움직이므로 높게 들지 않아도 안정적입니다.
        self.period = 2.0          # Wave Gait는 천천히 움직여야 효과적입니다 (2초 권장).
        
        # --- [Wave Gait 핵심 설정] ---
        # Duty Factor를 0.75로 설정하면 한 다리가 공중에 있을 때 세 다리는 지면을 지지합니다.
        self.duty_factor = 0.75    
        
        # 다리별 페이즈 오프셋 (순차적 이동: LH -> LF -> RH -> RF)
        # [LF, RF, LH, RH] 순서라면:
        self.leg_phases = [0.25, 0.75, 0.0, 0.5] 
        
        # 앞뒤 간격 유지
        self.front_x_offset = 0.03
        self.rear_x_offset = -0.03
        
        self.last_angles = [[0.0, 0.6, -1.2] for _ in range(4)]

    def get_stand_posture(self):
        """정지 상태 자세"""
        joint_angles = []
        for i in range(4):
            target_x = self.front_x_offset if i < 2 else self.rear_x_offset
            target_z = -self.body_height
            res = self.kin.ik(target_x, self.kin.L1 if i%2==0 else -self.kin.L1, target_z, leg_id=i)
            if res: self.last_angles[i] = list(res)
            joint_angles.extend(self.last_angles[i])
        return joint_angles

    def get_walk_posture(self, vx, vy, omega, t):
        phi = (t % self.period) / self.period
        joint_angles = []
        
        for i in range(4):
            leg_phi = (phi + self.leg_phases[i]) % 1.0
            base_y = self.kin.L1 if (i == 0 or i == 2) else -self.kin.L1
            anchor_x = self.front_x_offset if i < 2 else self.rear_x_offset
            
            # 보폭 계산
            stride_x = vx * (self.period * (1.0 - self.duty_factor))

            if leg_phi < self.duty_factor:  # STANCE (지면 지지 - 75% 시간 동안)
                # 지면을 아주 천천히 뒤로 밀어냄
                s = leg_phi / self.duty_factor
                step_x = anchor_x + (0.5 - s) * stride_x
                step_z = -self.body_height
            else:  # SWING (다리 이동 - 25% 시간 동안)
                # 한 다리씩 빠르게 앞으로 던짐
                s = (leg_phi - self.duty_factor) / (1.0 - self.duty_factor)
                step_x = anchor_x + (s - 0.5) * stride_x
                step_z = -self.body_height + self.step_height * math.sin(s * math.pi)

            res = self.kin.ik(step_x, base_y, step_z, leg_id=i)
            if res: self.last_angles[i] = list(res)
            joint_angles.extend(self.last_angles[i])
            
        return joint_angles