import math

class GaitPlanner:
    def __init__(self, kinematics):
        self.kin = kinematics
        self.body_height = 0.16 # Adjusted for 0.1075 + 0.13 legs
        self.step_height = 0.04
        self.period = 0.5
        
    def update_parameters(self, height, step_h, period):
        self.body_height = height
        self.step_height = step_h
        self.period = period

    def get_stand_posture(self):
        joint_angles = []
        for i in range(4):
            res = self.kin.ik(0.0, 0.0, -self.body_height, leg_id=i)
            if res: joint_angles.extend(res)
            else: joint_angles.extend([0.0, -0.7, 1.4])
        return joint_angles

    def get_walk_posture(self, vx, vy, omega, t):
        phi = (t % self.period) / self.period
        leg_phases = [0, 0.5, 0.5, 0] 
        joint_angles = []
        for i in range(4):
            leg_phi = (phi + leg_phases[i]) % 1.0
            if leg_phi < 0.5:
                s = leg_phi * 2.0
                step_x = (0.5 - s) * vx * self.period
                step_y = (0.5 - s) * vy * self.period
                step_z = -self.body_height
            else:
                s = (leg_phi - 0.5) * 2.0
                step_x = (s - 0.5) * vx * self.period
                step_y = (s - 0.5) * vy * self.period
                step_z = -self.body_height + self.step_height * math.sin(s * math.pi)
            res = self.kin.ik(step_x, step_y, step_z, leg_id=i)
            if res: joint_angles.extend(res)
            else: joint_angles.extend([0.0, -0.7, 1.4])
        return joint_angles
