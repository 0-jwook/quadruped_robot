import math

class GaitPlanner:
    """
    센서 피드백이 없는 환경에서 가장 안정적으로 움직이는 
    단순 정현파 기반의 투박한 보행 플래너입니다.
    """
    def __init__(self, kinematics):
        self.kin = kinematics
        
        # --- [물리 파라미터 - 직관적으로 수정] ---
        self.body_height = 0.28    # 다리가 적당히 펴지는 높이
        self.step_height = 0.08    # 발을 확실히 들어올림 (8cm)
        self.period = 1.2          # 아주 천천히 걷게 설정 (안정성 우선)
        self.duty_factor = 0.5     # Trot (대각선 두 다리씩 이동)
        
        # 앞뒤 다리 차이 없이 동일하게 설정 (단순화)
        self.last_angles = [[0.0, 0.6, -1.2] for _ in range(4)]

    def get_stand_posture(self):
        """단순히 네 다리를 수직으로 내리고 서 있는 자세"""
        joint_angles = []
        for i in range(4):
            target_x = 0.0
            target_y = self.kin.L1 if (i == 0 or i == 2) else -self.kin.L1
            target_z = -self.body_height
            
            res = self.kin.ik(target_x, target_y, target_z, leg_id=i)
            if res:
                self.last_angles[i] = list(res)
            joint_angles.extend(self.last_angles[i])
        return joint_angles

    def get_walk_posture(self, vx, vy, omega, t):
        """단순 정현파(Sine)를 이용한 궤적 생성"""
        # 보행 주기 계산 (0.0 ~ 1.0)
        phi = (t % self.period) / self.period
        
        # Trot 보행: 대각선 다리끼리 같은 페이즈
        # 0: 앞왼쪽, 3: 뒤오른쪽 / 1: 앞오른쪽, 2: 뒤왼쪽
        leg_phases = [0.0, 0.5, 0.5, 0.0] 
        
        joint_angles = []
        
        for i in range(4):
            leg_phi = (phi + leg_phases[i]) % 1.0
            base_y = self.kin.L1 if (i == 0 or i == 2) else -self.kin.L1
            
            # 단순 보폭 계산 (vx가 0.1이면 stride_x는 0.06m)
            stride_x = vx * (self.period * 0.5)
            
            # --- 단순 보행 로직 ---
            if leg_phi < self.duty_factor: # 지면 지지기 (Stance)
                # 발을 뒤로 밀어냄 (선형 이동)
                s = leg_phi / self.duty_factor
                step_x = (0.5 - s) * stride_x
                step_z = -self.body_height
            else: # 공중 이동기 (Swing)
                # 발을 앞으로 던짐 (정현파 높이 추가)
                s = (leg_phi - self.duty_factor) / (1.0 - self.duty_factor)
                step_x = (s - 0.5) * stride_x
                
                # 단순히 Sine 곡선으로 발을 들어올림
                # sin(0) = 0, sin(pi/2) = 1, sin(pi) = 0
                step_z = -self.body_height + self.step_height * math.sin(s * math.pi)

            # 옆으로 걷기(vy)는 일단 0으로 고정하여 변수 제거 (필요시 나중에 추가)
            step_y = base_y 
            
            # IK 계산
            res = self.kin.ik(step_x, step_y, step_z, leg_id=i)
            if res:
                self.last_angles[i] = list(res)
            else:
                # IK 실패 시 다리를 최대한 펴거나 이전 값을 유지하여 꼬임 방지
                pass
            
            joint_angles.extend(self.last_angles[i])
            
        return joint_angles