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
        도메인 위반 시 SpotMicroAI 방식으로 clamp (None 대신 가장 가까운 가능 해 반환).
        """
        # 다리 위치에 따른 Y축 오프셋 방향 설정
        side_sign = 1.0 if (leg_id == 0 or leg_id == 2) else -1.0
        l1 = self.L1 * side_sign

        # 1. Abduction Angle (q1) - 측면 회전 각도
        r2 = py**2 + pz**2
        # py²+pz² < l1² 일 때는 l1 만큼 떨어진 위치가 도달 불가 — 평면 투영을 0 으로 클램프
        sqrt_inner = max(0.0, r2 - l1**2)
        q1 = math.atan2(py, -pz) - math.atan2(l1, math.sqrt(sqrt_inner))

        # 2. X-Z' 평면 투영 (유효 다리 길이)
        z_eff = -math.sqrt(sqrt_inner)
        x_eff = px

        dist2 = x_eff**2 + z_eff**2
        dist = math.sqrt(dist2) if dist2 > 0.0 else 0.0

        # 도달 범위 클램프 (None 반환 대신 가장 가까운 가능 해 사용)
        max_reach = self.L2 + self.L3
        min_reach = abs(self.L2 - self.L3)
        if dist > max_reach:
            scale = max_reach * 0.999 / dist
            x_eff *= scale
            z_eff *= scale
            dist  = max_reach * 0.999
            dist2 = dist * dist
        elif dist < min_reach and dist > 0.0:
            scale = min_reach * 1.001 / dist
            x_eff *= scale
            z_eff *= scale
            dist  = min_reach * 1.001
            dist2 = dist * dist

        # 3. Knee Angle (q3) - 제2코사인 법칙
        cos_q3 = (dist2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        q3 = math.acos(cos_q3)

        # 4. Thigh Angle (q2)
        alpha = math.atan2(x_eff, -z_eff)
        beta  = math.atan2(self.L3 * math.sin(q3), self.L2 + self.L3 * math.cos(q3))
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