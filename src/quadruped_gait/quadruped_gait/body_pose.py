import math


class BodyPoseController:
    """
    Body Pose 제어 (Body IK).

    발 4개를 지면에 고정한 채 몸통(body)만 6자유도로 움직인다:
      이동: dx(전후), dy(좌우), dz(상하)
      회전: roll(좌우 기울임), pitch(앞뒤 끄덕), yaw(수평 회전)

    원리:
      발의 월드 위치는 고정. 몸통이 (R, t) 만큼 움직이면, 몸통 기준에서 본
      발 위치는 역변환됨:  P_body = R⁻¹ · (P_foot_default − t)
      이를 각 다리 어깨 기준으로 변환해 IK 로 관절각 계산.

    좌표계 (body frame): x=전방(+), y=좌(+), z=상(+, 발은 z<0)
    다리 인덱스: 0=FL, 1=FR, 2=RL, 3=RR
    """

    def __init__(self, kinematics, hip_x=0.1225, hip_y=0.10, body_height=0.14,
                 max_xy=0.03, max_z=0.04, max_ang=0.26,
                 level_gain=1.0, level_max=0.09):
        self.kin = kinematics
        self.hip_x = hip_x          # 몸통중심~발 종방향
        self.hip_y = hip_y          # 몸통중심~발 횡방향 (= body_width/2 + L1)
        self.body_height = body_height
        # 안전 범위 (워크스페이스 보호)
        self.max_xy  = max_xy       # 이동 상한 (m)
        self.max_z   = max_z        # 상하 상한 (m)
        self.max_ang = max_ang      # 회전 상한 (rad, ~15°)
        # leveling (planner 와 동일 — POSE 중립을 stand 와 일치시킴)
        self.level_gain = level_gain
        self.level_max  = level_max
        # IK 실패 fallback
        self._last_angles = [(0.0, -0.54, 1.35) for _ in range(4)]

    def _foot_default(self, idx, bh):
        """발 i 의 기본 위치 (몸통 중심 기준, 지면 고정점)."""
        fx = self.hip_x if idx in (0, 1) else -self.hip_x   # 앞(+)/뒤(-)
        fy = self.hip_y if idx in (0, 2) else -self.hip_y   # 좌(+)/우(-)
        return (fx, fy, -bh)

    def _level_dz(self, idx, lvl_roll, lvl_pitch):
        """planner 와 동일한 기하학적 수평유지 z 보정 (POSE 중립 ↔ stand 일치)."""
        if self.level_gain == 0.0:
            return 0.0
        rx = self.hip_x if idx in (0, 1) else -self.hip_x
        ry = self.hip_y if idx in (0, 2) else -self.hip_y
        dz = (-rx * math.tan(lvl_pitch) + ry * math.tan(lvl_roll)) * self.level_gain
        return max(-self.level_max, min(self.level_max, dz))

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def get_pose_posture(self, dx=0.0, dy=0.0, dz=0.0,
                         roll=0.0, pitch=0.0, yaw=0.0, body_height=None,
                         lvl_roll=0.0, lvl_pitch=0.0):
        """
        몸통 6축 변환에 대한 12 관절각 반환 (발 고정).
        dx,dy,dz: 몸통 이동 (m), roll,pitch,yaw: 몸통 회전 (rad)
        lvl_roll,lvl_pitch: 수평유지용 IMU+offset (stand 와 중립 일치시킴, z 보정)
        """
        bh = body_height if body_height is not None else self.body_height

        # 안전 clamp
        dx = self._clamp(dx, -self.max_xy, self.max_xy)
        dy = self._clamp(dy, -self.max_xy, self.max_xy)
        dz = self._clamp(dz, -self.max_z,  self.max_z)
        roll  = self._clamp(roll,  -self.max_ang, self.max_ang)
        pitch = self._clamp(pitch, -self.max_ang, self.max_ang)
        yaw   = self._clamp(yaw,   -self.max_ang, self.max_ang)

        # 몸통 회전행렬 R 의 역변환 R⁻¹ = Rᵀ (Z-Y-X intrinsic: yaw·pitch·roll)
        cr, sr = math.cos(roll),  math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw),   math.sin(yaw)

        # R = Rz(yaw) · Ry(pitch) · Rx(roll)  (월드←몸통)
        # 발은 월드 고정 → 몸통 기준 발 위치 = Rᵀ · (P_default − t)
        # Rᵀ 성분 (R 의 전치)
        r00 =  cy*cp
        r01 =  sy*cp
        r02 = -sp
        r10 =  cy*sp*sr - sy*cr
        r11 =  sy*sp*sr + cy*cr
        r12 =  cp*sr
        r20 =  cy*sp*cr + sy*sr
        r21 =  sy*sp*cr - cy*sr
        r22 =  cp*cr

        t = (dx, dy, dz)
        angles = []
        for i in range(4):
            fx, fy, fz = self._foot_default(i, bh)
            # P_default − t
            vx = fx - t[0]
            vy = fy - t[1]
            vz = fz - t[2]
            # Rᵀ · v  (몸통 기준 발 위치)
            bx = r00*vx + r01*vy + r02*vz
            by = r10*vx + r11*vy + r12*vz
            bz = r20*vx + r21*vy + r22*vz

            # 몸통 기준 → 어깨 기준 (IK 입력).
            # IK 중립은 발이 어깨 기준 (0, ±L1, -bh) 이므로, 어깨 횡위치 = ±(hip_y − L1)
            # = ±body_width/2. 어깨 종위치 = ±hip_x.
            L1 = self.kin.L1
            sxh = self.hip_x if i in (0, 1) else -self.hip_x
            syh = (self.hip_y - L1) if i in (0, 2) else -(self.hip_y - L1)
            px = bx - sxh
            py = by - syh
            pz = bz + self._level_dz(i, lvl_roll, lvl_pitch)   # 수평유지 (stand 와 일치)

            res = self.kin.ik(px, py, pz, leg_id=i)
            if res is None:
                res = self._last_angles[i]
            else:
                self._last_angles[i] = res
            angles.extend(res)
        return angles
