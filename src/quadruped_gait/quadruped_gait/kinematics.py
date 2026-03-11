import math

class LegKinematics:
    def __init__(self, L1=0.04, L2=0.1075, L3=0.130):
        self.L1 = L1 
        self.L2 = L2 
        self.L3 = L3 

    def ik(self, px, py, pz, leg_id=0):
        q1 = math.atan2(py, -pz)
        z_eff = -math.sqrt(max(0, py**2 + pz**2))
        x_eff = px
        dist2 = x_eff**2 + z_eff**2
        dist = math.sqrt(dist2)
        if dist > (self.L2 + self.L3) or dist < abs(self.L2 - self.L3):
            return None
        cos_q3 = (dist2 - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)
        cos_q3 = max(-1.0, min(1.0, cos_q3))
        q3 = math.acos(cos_q3) 
        alpha = math.atan2(x_eff, -z_eff)
        beta = math.atan2(self.L3 * math.sin(q3), self.L2 + self.L3 * math.cos(q3))
        q2 = alpha - beta
        return q1, q2, q3

    def fk(self, q1, q2, q3, leg_id=0):
        l1 = self.L1 * (1.0 if (leg_id == 0 or leg_id == 2) else -1.0)
        l2, l3 = self.L2, self.L3
        xt = l2 * math.sin(q2) + l3 * math.sin(q2 + q3)
        zt = -(l2 * math.cos(q2) + l3 * math.cos(q2 + q3))
        px, py = xt, l1 * math.cos(q1) - zt * math.sin(q1)
        pz = l1 * math.sin(q1) + zt * math.cos(q1)
        return px, py, pz
