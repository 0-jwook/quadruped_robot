import math


class GaitPlanner:
    """
    SpotMicroAI (spot_mini_mini) BezierGait 기반 걸음걸이 플래너.

    참조: https://github.com/moribots/spot_mini_mini/blob/master/spotmicro/GaitGenerator/Bezier.py
    원본: MIT Cheetah paper (12-point Bezier swing + sinusoidal stance)

    핵심 아이디어:
      · Trot 게이트 (대각선 쌍 FL+RR, FR+RL 교대)
      · Swing: 12개 제어점 Bezier 곡선 (부드러운 곡선)
      · Stance: 사인 함수 기반 직선 + 살짝 눌러 밟기 (penetration)
      · 위상은 time 기반, Tstance = 2|L|/|vel| 로 속도 연동

    좌표계 (body frame, 각 다리의 어깨 기준):
      px = 전방(+앞), py = 측방(+좌), pz = 높이(-아래)
    """

    def __init__(self, kinematics,
                 body_height=0.17, step_height=0.04,
                 max_stride=0.04, period=0.4, gait_type='trot'):
        self.kin         = kinematics
        self.body_height = body_height
        self.clearance   = step_height       # Bezier swing 최대 높이
        self.max_stride  = max_stride        # L (half stride) 상한
        self.penetration = 0.002             # 스탠스 시 살짝 눌러 밟기
        self.dt          = 0.02              # 50 Hz
        self.Tswing      = period / 2.0      # 한쪽 스윙 시간 (period=0.4 → 0.2s)

        # 다리 순서: 0=FL, 1=FR, 2=RL, 3=RR
        # BezierGait dSref (phase lag): FL=0, FR=0.5, RL=0.5, RR=0
        # → Trot: FL+RR 동시 / FR+RL 동시 (반대각선)
        self.dSref = [0.0, 0.5, 0.5, 0.0]
        self.ref_idx = 0    # FL 기준 다리

        # Bezier 위상 추적 상태
        self.time = 0.0
        self.TD_time = 0.0
        self.time_since_last_TD = 0.0
        self.SwRef = 0.0
        self.TD = False

        # IK 실패 시 마지막 유효 각도 fallback (스냅 방지)
        self._last_angles = [(0.0, -0.54, 1.35) for _ in range(4)]

        # 12-point Bezier 계수 (11 = n)
        self._n = 11
        self._binomial = [self._binom(self._n, k) for k in range(12)]

    # ── 유틸 ─────────────────────────────────────────────────────────────────
    @staticmethod
    def _binom(n, k):
        from math import factorial
        return factorial(n) // (factorial(k) * factorial(n - k))

    def _bernstein(self, t, k, point):
        return point * self._binomial[k] * (t ** k) * ((1.0 - t) ** (self._n - k))

    def _neutral_feet(self, bh):
        """각 다리의 어깨 기준 기본 위치."""
        L1 = self.kin.L1
        return [
            (0.0,  L1, -bh),   # FL
            (0.0, -L1, -bh),   # FR
            (0.0,  L1, -bh),   # RL
            (0.0, -L1, -bh),   # RR
        ]

    # ── 위상 관리 (BezierGait Increment / GetPhase) ─────────────────────────
    def _check_touchdown(self):
        if self.SwRef >= 0.9 and self.TD:
            self.TD_time = self.time
            self.TD = False
            self.SwRef = 0.0

    def _increment(self, Tstride):
        self._check_touchdown()
        self.time_since_last_TD = self.time - self.TD_time
        if self.time_since_last_TD > Tstride:
            self.time_since_last_TD = Tstride
        elif self.time_since_last_TD < 0.0:
            self.time_since_last_TD = 0.0
        self.time += self.dt
        # 스트라이드가 비정상적으로 작으면 리셋
        if Tstride < self.Tswing + self.dt:
            self.time = 0.0
            self.time_since_last_TD = 0.0
            self.TD_time = 0.0
            self.SwRef = 0.0

    def _get_phase(self, idx, Tstance):
        """
        해당 다리의 phase 와 stance/swing 여부 반환.
        :return: (phase[0~1], is_swing[True/False])
        """
        Tstride = Tstance + self.Tswing
        if idx == self.ref_idx:
            self.dSref[idx] = 0.0
        ti = self.time_since_last_TD - self.dSref[idx] * Tstride

        # phase discontinuity 방지 (Tstance > Tswing 시)
        if ti < -self.Tswing:
            ti += Tstride

        is_swing = False
        sw_phase = 0.0

        if 0.0 <= ti <= Tstance:
            # STANCE
            phase = ti / Tstance if Tstance > 0.0 else 0.0
            if idx == self.ref_idx:
                pass
            return phase, False

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

    # ── Bezier Swing / Sine Stance ──────────────────────────────────────────
    def _bezier_swing(self, phase, L, lat_frac, clearance):
        """12-point Bernstein polynomial swing trajectory."""
        cx = math.cos(lat_frac)
        sy = math.sin(lat_frac)

        # Forward component control points (MIT Cheetah 공식)
        STEP = [
            -L,        -L * 1.4,  -L * 1.5,  -L * 1.5,  -L * 1.5,
            0.0,        0.0,       0.0,
             L * 1.5,   L * 1.5,   L * 1.4,   L,
        ]
        # Vertical (위로 들리는 양) control points
        c = clearance
        Z = [
            0.0,    0.0,
            c*0.9,  c*0.9,  c*0.9,  c*0.9,  c*0.9,
            c*1.1,  c*1.1,  c*1.1,
            0.0,    0.0,
        ]

        stepX = stepY = stepZ = 0.0
        for k in range(12):
            stepX += self._bernstein(phase, k, STEP[k]) * cx
            stepY += self._bernstein(phase, k, STEP[k]) * sy
            stepZ += self._bernstein(phase, k, Z[k])

        return stepX, stepY, stepZ

    def _sine_stance(self, phase, L, lat_frac, penetration):
        """선형 후퇴 + 코사인 페너트레이션."""
        cx = math.cos(lat_frac)
        sy = math.sin(lat_frac)
        step = L * (1.0 - 2.0 * phase)
        stepX = step * cx
        stepY = step * sy
        if L != 0.0:
            stepZ = -penetration * math.cos(math.pi * (stepX + stepY) / (2.0 * L))
        else:
            stepZ = 0.0
        return stepX, stepY, stepZ

    # ── 회전 처리 (간단화: yaw rate → leg-별 추가 측방 변위) ────────────────
    def _yaw_step(self, idx, neutral, phase, is_swing, yaw_rate, clearance, penetration):
        """yaw_rate 가 있을 때 각 다리에 회전 접선 방향 변위 추가."""
        if abs(yaw_rate) < 1e-6:
            return 0.0, 0.0, 0.0

        # 다리 위치 기준 접선 방향 각 (body 중심 기준 추정)
        # 우리 시스템은 leg별 어깨 좌표가 IK 내부에 있으므로,
        # 회전 효과를 단순화: 좌측 다리는 +x, 우측 다리는 -x 방향으로 step
        # (몸체 yaw CCW → 좌측 발은 뒤로, 우측 발은 앞으로)
        side = +1.0 if idx in (0, 2) else -1.0   # FL,RL: 좌측 / FR,RR: 우측
        front = +1.0 if idx in (0, 1) else -1.0  # FL,FR: 앞 / RL,RR: 뒤

        # 회전 스텝 크기 (yaw_rate * Tstance/2 와 유사)
        Lr = yaw_rate * self.Tswing * 0.5
        # 좌측은 후퇴, 우측은 전진 방향
        lat = 0.0 if front > 0 else math.pi  # 단순 매핑
        # 더 단순하게: x 방향만 사용
        if is_swing:
            sx, sy, sz = self._bezier_swing(phase, Lr * side, 0.0, clearance * 0.3)
        else:
            sx, sy, sz = self._sine_stance(phase, Lr * side, 0.0, penetration * 0.3)
        return sx, sy, sz

    # ── 공개 API ────────────────────────────────────────────────────────────
    def reset(self):
        """제자리 자세로 돌아갈 때 Bezier 상태 초기화."""
        self.time = 0.0
        self.TD_time = 0.0
        self.time_since_last_TD = 0.0
        self.SwRef = 0.0
        self.TD = False

    def get_stand_posture(self, roll=0.0, pitch=0.0, body_height=None):
        """정지 자세: 모든 발을 중립 위치로 + IMU Roll/Pitch 보정."""
        bh = body_height if body_height is not None else self.body_height
        self.reset()

        kp_r, kp_p = 0.8, 1.5
        refs = [(0.2, 0.1), (0.2, -0.1), (-0.2, 0.1), (-0.2, -0.1)]
        neutrals = self._neutral_feet(bh)
        angles = []
        for i, (lx, ly) in enumerate(refs):
            dz = -(lx * math.sin(pitch) * kp_p - ly * math.sin(roll) * kp_r)
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
        """보행 자세: BezierGait (vx, vy → StepLength + LateralFraction)."""
        bh = body_height if body_height is not None else self.body_height

        # cmd_vel → Bezier 입력 변환
        v_mag = math.sqrt(vx * vx + vy * vy)

        # StepVelocity: 스트라이드 주기를 결정. 정지(vx≈0) 상태에서도 yaw 만 있으면 cycle 유지
        if v_mag < 0.005 and abs(omega) < 0.05:
            # 진짜 정지 — stand 자세로
            return self.get_stand_posture(roll, pitch, bh)

        # StepLength: 속도에 비례, max_stride 로 클램프
        # Tstance = 2L/v 이므로 L = v * Tstance / 2. Tstance ≈ Tswing 가정.
        L_raw = v_mag * self.Tswing  # 가정: Tstance = Tswing → L = v * T
        L = min(self.max_stride, L_raw)

        # LateralFraction: 진행 방향 각도
        if v_mag > 1e-6:
            lat_frac = math.atan2(vy, vx)
        else:
            lat_frac = 0.0

        StepVelocity = max(v_mag, 0.05)
        YawRate = omega

        # Tstance 계산 (Bezier 원본 로직)
        if L > 1e-6:
            Tstance = 2.0 * L / StepVelocity
        else:
            Tstance = 0.0
        if Tstance < self.dt:
            Tstance = 0.0
            L = 0.0
        elif Tstance > 1.3 * self.Tswing:
            Tstance = 1.3 * self.Tswing

        Tstride = Tstance + self.Tswing

        # touchdown 신호 (간단화: 항상 가능)
        if Tstance > self.dt:
            self.TD = True

        # 위상 증가
        self._increment(Tstride)

        neutrals = self._neutral_feet(bh)
        kp_r, kp_p = 0.5, 1.0
        refs = [(0.2, 0.1), (0.2, -0.1), (-0.2, 0.1), (-0.2, -0.1)]

        angles = []
        for i, (lx, ly) in enumerate(refs):
            phase, is_swing = self._get_phase(i, Tstance)

            if Tstance > 0.0:
                if is_swing:
                    rx, ry, rz = self._bezier_swing(phase, L, lat_frac, self.clearance)
                else:
                    rx, ry, rz = self._sine_stance(phase, L, lat_frac, self.penetration)
            else:
                rx, ry, rz = 0.0, 0.0, 0.0

            # yaw 회전 효과 추가
            yx, yy, yz = self._yaw_step(i, neutrals[i], phase, is_swing,
                                        YawRate, self.clearance, self.penetration)

            # IMU 보정
            dz = -(lx * math.sin(pitch) * kp_p - ly * math.sin(roll) * kp_r)

            nx, ny, nz = neutrals[i]
            px = nx + rx + yx
            py = ny + ry + yy
            pz = nz + rz + yz + dz

            res = self.kin.ik(px, py, pz, leg_id=i)
            if res is None:
                res = self._last_angles[i]
            else:
                self._last_angles[i] = res
            angles.extend(res)

        return angles
