import math


class GaitPlanner:
    """
    SpotMicroAI (spot_mini_mini) BezierGait 기반 걸음걸이 플래너 (개선판).

    참조: moribots/spot_mini_mini + MIT Cheetah (12-point Bezier swing)

    핵심 설계 (이번 개선):
      · 회전 운동학 통합 (Fix 3): 각 발 지면속도 v_foot = v_body + ω × r_foot
        → 병진+회전이 자연 합성, 발끌림(scrubbing) 0, FORWARD_BOOST 같은 보정상수 불필요
      · 고정 duty (Fix 9/10): period(=전체 cycle)·duty 고정.
        발 stride 가 max_stride 초과 시 명령 속도를 scale-down → 고속에서도 비행 구간 없음
      · CoM shift (Fix 1): wave 게이트에서 몸통을 지지발 centroid 로 LPF 이동 (정적 안정)
      · trot↔wave 안전 전환 (Fix 4): cycle 경계에서만 전환 (phase jump 방지)

    좌표계 (body frame):
      px = 전방(+앞), py = 측방(+좌), pz = 높이(-아래)
      다리 인덱스: 0=FL, 1=FR, 2=RL, 3=RR
    """

    # 12-point Bezier 제어점 (order-11 Bernstein). 정규화: 수평 -1→+1, 수직 0→peak→0
    SWING_H = [-1.0, -1.4, -1.5, -1.5, -1.5, 0.0, 0.0, 0.0, 1.5, 1.5, 1.4, 1.0]
    SWING_V = [ 0.0,  0.0,  0.9,  0.9,  0.9, 0.9, 0.9, 1.1, 1.1, 1.1, 0.0, 0.0]

    def __init__(self, kinematics,
                 body_height=0.17, step_height=0.04,
                 max_stride=0.05, period=0.5, gait_type='trot',
                 duty_trot=0.6, duty_wave=0.75,
                 hip_x=0.10, hip_y=0.05, penetration=0.004,
                 level_gain=1.0, level_max=0.09):
        self.kin         = kinematics
        self.body_height = body_height
        self.clearance   = step_height       # swing 최대 발 들기 높이
        self.max_stride  = max_stride        # 한 발 stride 벡터 크기 상한
        self.penetration = penetration       # stance 시 살짝 눌러 밟기 (백래시 보정)
        self.dt          = 0.02              # 50 Hz
        self.period      = period            # 전체 cycle 시간 Tstride
        self.duty_trot   = duty_trot
        self.duty_wave   = duty_wave
        self.hip_x       = hip_x             # 몸통 중심~발 종방향 거리 (전/후)
        self.hip_y       = hip_y             # 몸통 중심~발 횡방향 거리 (좌/우)
        # 자세 수평 유지 (geometric leveling): 0=끔, 1=완전 수평 유지
        self.level_gain  = level_gain
        self.level_max   = level_max         # leveling dz 절대값 상한 (워크스페이스 보호)
        self.gait_type   = gait_type.lower()

        # 활성 게이트 초기화
        if self.gait_type in ('8phase', 'wave', '4wave'):
            self._set_gait('wave')
        else:
            self._set_gait('trot')
        self._pending_gait = None

        self.ref_idx = 0

        # 위상 추적 상태
        self.time = 0.0
        self.TD_time = 0.0
        self.time_since_last_TD = 0.0
        self.SwRef = 0.0
        self.TD = False

        # CoM shift 상태 (LPF)
        self.com_x = 0.0
        self.com_y = 0.0
        self.com_gain = 0.6        # 지지 centroid 의 몇 배만큼 이동
        self.com_tau  = 0.12       # LPF 시정수 (s)

        # IK 실패 시 fallback
        self._last_angles = [(0.0, -0.54, 1.35) for _ in range(4)]

        # Bernstein 이항계수
        self._n = 11
        self._binomial = [self._binom(self._n, k) for k in range(12)]

    # ── 게이트 설정 ──────────────────────────────────────────────────────────
    def _set_gait(self, name):
        """게이트별 duty/dSref 설정 + Tstance/Tswing 재계산."""
        if name == 'wave':
            self.duty  = self.duty_wave
            self.dSref = [0.0, 0.5, 0.75, 0.25]   # FL, FR, RL, RR (한 다리씩 순차)
        else:  # 'trot'
            self.duty  = self.duty_trot
            self.dSref = [0.0, 0.5, 0.5, 0.0]     # 대각선 쌍 동기
        self.Tstance = self.duty * self.period
        self.Tswing  = (1.0 - self.duty) * self.period
        self._active_gait = name

    def _request_gait(self, target):
        """게이트 전환 요청 — 즉시 바꾸지 않고 cycle 경계에서 적용 (Fix 4)."""
        if target == self._active_gait:
            self._pending_gait = None
        else:
            self._pending_gait = target

    def _apply_pending_gait_if_safe(self):
        """cycle 경계(모든 발이 stance 에 가까운 순간)에서만 게이트 전환."""
        if self._pending_gait is None:
            return
        if self.time_since_last_TD < self.dt:   # 막 touchdown 으로 wrap 된 순간
            self._set_gait(self._pending_gait)
            self._pending_gait = None
            self.time = 0.0
            self.TD_time = 0.0
            self.time_since_last_TD = 0.0
            self.SwRef = 0.0
            self.TD = False

    # ── 유틸 ─────────────────────────────────────────────────────────────────
    @staticmethod
    def _binom(n, k):
        from math import factorial
        return factorial(n) // (factorial(k) * factorial(n - k))

    def _bezier_scalar(self, t, pts):
        """정규화된 12점 Bernstein 다항식 값."""
        out = 0.0
        for k in range(12):
            out += pts[k] * self._binomial[k] * (t ** k) * ((1.0 - t) ** (self._n - k))
        return out

    def _foot_center_xy(self, idx):
        """발 i 의 중립 위치 (몸통 중심 기준 수평 xy). ω×r 계산용."""
        rx = self.hip_x if idx in (0, 1) else -self.hip_x   # FL,FR 앞(+) / RL,RR 뒤(-)
        ry = self.hip_y if idx in (0, 2) else -self.hip_y   # FL,RL 좌(+) / FR,RR 우(-)
        return rx, ry

    def _level_dz(self, idx, roll, pitch):
        """
        기하학적 수평 유지 (geometric body leveling).

        몸통이 (roll, pitch) 만큼 기운 것을 IMU 가 측정했을 때, 각 발의 z 를
        조정해 발 접지면을 world-수평으로 만들어 몸통을 수평 유지.
          Δz = -rx·tan(pitch) + ry·tan(roll)     (REP-103: x전방, y좌, z상)
        부호 근거 (REP-103, pitch>0=nose-down, roll>0=우측down):
          · pitch>0 (앞 내려감) → 앞다리 더 펴서(z↓) 앞 올림 → -rx·tan>0... rx>0 이면 음수 → 더 폄 ✓
          · roll>0  (우측 내려감) → 우측다리 더 펴서 우측 올림 → ry<0(우) 이면 음수 → 더 폄 ✓
        실기에서 leveling 이 거꾸로 작동하면(기울임 가중) level_gain 부호를 뒤집을 것.

        return: 발 z 에 더할 보정량 (m), level_max 로 clamp.
        """
        if self.level_gain == 0.0:
            return 0.0
        rx, ry = self._foot_center_xy(idx)
        dz = (-rx * math.tan(pitch) + ry * math.tan(roll)) * self.level_gain
        return max(-self.level_max, min(self.level_max, dz))

    def _foot_stride(self, idx, vx, vy, omega):
        """
        Fix 3: 발 i 의 지면속도 v_foot = v_body + ω × r_foot 를 적분한 stride 벡터.
          ω × r = (-ω·ry, ω·rx)  (ω 는 +z 축, CCW)
          stride = v_foot · Tstance
        """
        rx, ry = self._foot_center_xy(idx)
        vfx = vx - omega * ry
        vfy = vy + omega * rx
        return vfx * self.Tstance, vfy * self.Tstance

    def _neutral_feet(self, bh):
        """각 다리의 어깨 기준 기본 발 위치 (IK 입력 frame)."""
        L1 = self.kin.L1
        return [
            (0.0,  L1, -bh),   # FL
            (0.0, -L1, -bh),   # FR
            (0.0,  L1, -bh),   # RL
            (0.0, -L1, -bh),   # RR
        ]

    # ── 위상 관리 ────────────────────────────────────────────────────────────
    def _check_touchdown(self):
        if self.SwRef >= 0.9 and self.TD:
            self.TD_time = self.time
            self.TD = False
            self.SwRef = 0.0

    def _increment(self):
        Tstride = self.Tstance + self.Tswing
        self._check_touchdown()
        self.time_since_last_TD = self.time - self.TD_time
        if self.time_since_last_TD > Tstride:
            self.time_since_last_TD = Tstride
        elif self.time_since_last_TD < 0.0:
            self.time_since_last_TD = 0.0
        self.time += self.dt
        if Tstride < self.Tswing + self.dt:
            self.time = 0.0
            self.time_since_last_TD = 0.0
            self.TD_time = 0.0
            self.SwRef = 0.0

    def _get_phase(self, idx):
        """해당 다리의 phase[0~1] 와 is_swing 반환."""
        Tstance = self.Tstance
        Tstride = Tstance + self.Tswing
        if idx == self.ref_idx:
            self.dSref[idx] = 0.0
        ti = self.time_since_last_TD - self.dSref[idx] * Tstride

        if ti < -self.Tswing:
            ti += Tstride

        if 0.0 <= ti <= Tstance:
            phase = ti / Tstance if Tstance > 0.0 else 0.0
            return phase, False

        sw_phase = 0.0
        is_swing = False
        if -self.Tswing <= ti < 0.0:
            sw_phase = (ti + self.Tswing) / self.Tswing
            is_swing = True
        elif Tstance < ti <= Tstride:
            sw_phase = (ti - Tstance) / self.Tswing
            is_swing = True

        if sw_phase >= 1.0:
            sw_phase = 1.0

        if idx == self.ref_idx:
            self.SwRef = sw_phase
            if self.SwRef >= 0.999:
                self.TD = True

        return sw_phase, is_swing

    # ── 궤적 (벡터 stride 기반) ────────────────────────────────────────────────
    def _swing_traj(self, s, sx, sy, clearance):
        """12-point Bezier swing: 수평 -S/2 → +S/2, 수직 0 → peak → 0."""
        h = self._bezier_scalar(s, self.SWING_H)   # -1 → +1
        v = self._bezier_scalar(s, self.SWING_V)   # 0 → peak → 0
        return 0.5 * sx * h, 0.5 * sy * h, clearance * v

    def _stance_traj(self, s, sx, sy, penetration):
        """선형 후퇴 +S/2 → -S/2 (속도 = -v_foot, 발끌림 0) + 작은 penetration."""
        ox = sx * (0.5 - s)
        oy = sy * (0.5 - s)
        oz = -penetration * math.sin(math.pi * s)
        return ox, oy, oz

    # ── CoM shift (Fix 1) ─────────────────────────────────────────────────────
    def _update_com(self, swing_flags):
        """지지(stance) 발들의 중립 centroid 로 몸통을 LPF 이동.
        trot 대각 지지 → centroid≈0 → 자연 감쇠. wave 3-leg → swing 다리 반대로 이동."""
        sx = sy = 0.0
        n = 0
        for i in range(4):
            if not swing_flags[i]:
                rx, ry = self._foot_center_xy(i)
                sx += rx
                sy += ry
                n += 1
        if n > 0:
            cx = self.com_gain * (sx / n)
            cy = self.com_gain * (sy / n)
        else:
            cx = cy = 0.0
        a = self.dt / (self.com_tau + self.dt)
        self.com_x += (cx - self.com_x) * a
        self.com_y += (cy - self.com_y) * a
        return self.com_x, self.com_y

    # ── 공개 API ────────────────────────────────────────────────────────────
    def reset(self):
        self.time = 0.0
        self.TD_time = 0.0
        self.time_since_last_TD = 0.0
        self.SwRef = 0.0
        self.TD = False
        self.com_x = 0.0
        self.com_y = 0.0

    def max_speed(self):
        """현재 게이트에서 stride cap 으로 결정되는 최고 전진 속도 (m/s)."""
        return self.max_stride / self.Tstance if self.Tstance > 0 else 0.0

    def get_stand_posture(self, roll=0.0, pitch=0.0, body_height=None):
        """정지 자세: 모든 발 중립 + 기하학적 수평 유지 (최대 30° 경사)."""
        bh = body_height if body_height is not None else self.body_height
        self.reset()

        neutrals = self._neutral_feet(bh)
        angles = []
        for i in range(4):
            dz = self._level_dz(i, roll, pitch)
            px, py, pz = neutrals[i]
            res = self.kin.ik(px, py, pz + dz, leg_id=i)
            if res is None:
                res = self._last_angles[i]
            else:
                self._last_angles[i] = res
            angles.extend(res)
        return angles

    def get_walk_posture(self, vx, vy, omega, t,
                         roll=0.0, pitch=0.0, body_height=None):
        """
        보행 자세 — 통합 회전 운동학 + 고정 duty + CoM shift.

        지원: 전진/후진(vx), 측방(vy), 회전(omega), 호 회전(전부 조합).
        """
        bh = body_height if body_height is not None else self.body_height

        # ① 안전 clamp (느슨; 실제 상한은 ③ scale-down 이 담당)
        MAX_LIN = 0.30
        MAX_ANG = 0.8
        vx    = max(-MAX_LIN, min(MAX_LIN, vx))
        vy    = max(-MAX_LIN, min(MAX_LIN, vy))
        omega = max(-MAX_ANG, min(MAX_ANG, omega))

        v_mag = math.sqrt(vx * vx + vy * vy)

        # ② 완전 정지 → stand 자세
        if v_mag < 0.005 and abs(omega) < 0.05:
            return self.get_stand_posture(roll, pitch, bh)

        # ②-b 게이트 전환 요청 (Fix 4): 측방 우세 시 wave, 그 외 trot. cycle 경계에서만 적용
        if self.gait_type == 'trot':
            target = 'wave' if (abs(vy) > abs(vx) and v_mag > 0.01) else 'trot'
            self._request_gait(target)
        self._apply_pending_gait_if_safe()

        # ③ 발별 stride (Fix 3: v_foot = v + ω×r) + max_stride scale-down (Fix 9/10)
        strides = [self._foot_stride(i, vx, vy, omega) for i in range(4)]
        max_mag = max(math.hypot(sx, sy) for sx, sy in strides)
        if max_mag > self.max_stride and max_mag > 1e-9:
            sc = self.max_stride / max_mag
            strides = [(sx * sc, sy * sc) for sx, sy in strides]

        # ④ 위상 증가
        if self.Tstance > self.dt:
            self.TD = True
        self._increment()

        # ⑤ 각 다리 phase 계산 → CoM shift → 궤적 → IK
        phases = [self._get_phase(i) for i in range(4)]
        swing_flags = [is_sw for (_, is_sw) in phases]
        com_x, com_y = self._update_com(swing_flags)

        neutrals = self._neutral_feet(bh)

        angles = []
        for i in range(4):
            s, is_swing = phases[i]
            sx, sy = strides[i]

            if is_swing:
                ox, oy, oz = self._swing_traj(s, sx, sy, self.clearance)
            else:
                ox, oy, oz = self._stance_traj(s, sx, sy, self.penetration)

            # 보행 중에도 기하학적 수평 유지 (경사면 보행 적응)
            dz = self._level_dz(i, roll, pitch)

            nx, ny, nz = neutrals[i]
            px = nx + ox - com_x
            py = ny + oy - com_y
            pz = nz + oz + dz

            res = self.kin.ik(px, py, pz, leg_id=i)
            if res is None:
                res = self._last_angles[i]
            else:
                self._last_angles[i] = res
            angles.extend(res)

        return angles
