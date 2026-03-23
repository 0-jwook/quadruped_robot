import math

class LegKinematics:
    """
    3자유도(Abduction, Thigh, Calf) 다리의 순기능학 및 역기능학을 담당합니다.
    """
    def __init__(self, L1=0.08, L2=0.2, L3=0.2):
        self.L1 = L1  # 어깨 오프셋 (Shoulder offset)
        self.L2 = L2  # 허벅지 길이 (Thigh length)
        self.L3 = L3  # 종아리 길이 (Calf length)

    def ik(self, px, py, pz, leg_id=0):
        """
        Cartesian 좌표 (px, py, pz)를 관절 각도 (q1, q2, q3)로 변환합니다.
        leg_id: 0, 2는 왼쪽 다리 / 1, 3은 오른쪽 다리
        """
        # 다리 위치에 따른 Y축 오프셋 방향 설정
        side_sign = 1.0 if (leg_id == 0 or leg_id == 2) else -1.0
        l1 = self.L1 * side_sign
        
        # 1. Abduction Angle (q1) - 측면 회전 각도
        r2 = py**2 + pz**2
        r = math.sqrt(r2)
        
        if r < abs(l1):
            return None
            
        # 오프셋 l1을 고려한 q1 계산
        q1 = math.atan2(py, -pz) - math.atan2(l1, math.sqrt(max(0, r2 - l1**2)))

        # 2. X-Z' 평면 투영 (유효 다리 길이 계산)
        z_eff = -math.sqrt(max(0, r2 - l1**2))
        x_eff = px
        
        dist2 = x_eff**2 + z_eff**2
        dist = math.sqrt(dist2)

        # 물리적 한계 도달 여부 확인
        if dist > (self.L2 + self.L3) or dist < abs(self.L2 - self.L3):
            return None

        # 3. Knee Angle (q3) - 제2코사인 법칙 사용
        cos_q3 = (dist2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        q3 = math.acos(cos_q3) 

        # 4. Thigh Angle (q2)
        alpha = math.atan2(x_eff, -z_eff)
        beta = math.atan2(self.L3 * math.sin(q3), self.L2 + self.L3 * math.cos(q3))
        q2 = alpha - beta

        return q1, q2, q3

    def fk(self, q1, q2, q3, leg_id=0):
        """
        관절 각도를 바탕으로 발끝의 Cartesian 좌표를 계산합니다.
        """
        side_sign = 1.0 if (leg_id == 0 or leg_id == 2) else -1.0
        l1 = self.L1 * side_sign
        l2, l3 = self.L2, self.L3
        
        xt = l2 * math.sin(q2) + l3 * math.sin(q2 + q3)
        zt = -(l2 * math.cos(q2) + l3 * math.cos(q2 + q3))
        
        px = xt
        py = l1 * math.cos(q1) - zt * math.sin(q1)
        pz = l1 * math.sin(q1) + zt * math.cos(q1)
        return px, py, pz