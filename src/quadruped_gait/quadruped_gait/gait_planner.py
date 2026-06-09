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
        self.gait_type   = gait_type.lower()

        # 다리 순서: 0=FL, 1=FR, 2=RL, 3=RR
        if self.gait_type in ('8phase', 'wave', '4wave'):
            # 4-leg wave gait — 한 번에 한 다리만 swing, 항상 3-leg 지지
            # swing 순서: FL → RR → FR → RL (대각선 교차)
            # Tswing 은 cycle 의 1/4 (한 다리 swing 비율)
            self.Tswing = period / 4.0
            self.dSref  = [0.0, 0.5, 0.75, 0.25]  # FL, FR, RL, RR
            # Tstance ≈ 3*Tswing (다른 3 다리가 stance 중일 동안 한 다리만 swing)
            self.tstance_min_ratio = 2.8
            self.tstance_max_ratio = 3.2
        else:  # 'trot' 기본
            self.Tswing = period / 2.0
            self.dSref  = [0.0, 0.5, 0.5, 0.0]    # 대각선 쌍 동기
            self.tstance_min_ratio = 0.7
            self.tstance_max_ratio = 1.3

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

    # ── 회전 처리 (수평 변위만 담당, 발 들기는 메인 swing 이 담당) ──────────
    def _yaw_step(self, idx, phase, is_swing, yaw_rate):
        """
        yaw_rate 가 있을 때 좌/우 다리에 반대 방향 stride 추가 (수평 변위만).
        CCW 회전(yaw>0) → 좌측 다리 전진 / 우측 다리 후진.
        clearance/penetration = 0  → 발 들기는 메인 swing 이 일관되게 담당하여
        두 컴포넌트 위상 어긋남으로 인한 발 끌림 / 이중 합산 방지.
        """
        if abs(yaw_rate) < 1e-6:
            return 0.0, 0.0, 0.0

        side = +1.0 if idx in (0, 2) else -1.0   # FL,RL: +좌측 / FR,RR: -우측
        Lr = yaw_rate * self.Tswing * 0.5
        Lr = max(-self.max_stride, min(self.max_stride, Lr))

        # clearance=0, penetration=0 → 수평 변위만 (sz 항상 0)
        if is_swing:
            sx, sy, sz = self._bezier_swing(phase, Lr * side, 0.0, 0.0)
        else:
            sx, sy, sz = self._sine_stance(phase, Lr * side, 0.0, 0.0)
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
        """
        보행 자세: BezierGait (cmd_vel → StepLength + LateralFraction + YawRate).

        지원 동작:
          · 전진:   vx > 0     → 발이 전방으로 step
          · 후진:   vx < 0     → LateralFraction=π 로 매핑 → 발이 후방으로 step
          · 측방:   vy ≠ 0     → LateralFraction 으로 매핑 → 발이 좌/우로 step
          · 회전:   omega ≠ 0  → 좌측 발 전진 / 우측 발 후진 → 제자리 회전
          · 정지:   모두 ≈ 0   → stand 자세
        """
        bh = body_height if body_height is not None else self.body_height

        # ① 안전 clamp: 우리 작은 로봇 크기 (L2+L3=0.25m) 에 맞춰
        #    teleop_twist_keyboard 기본 0.5 m/s 가 너무 빠르기 때문.
        MAX_LIN = 0.15   # m/s
        MAX_ANG = 0.6    # rad/s
        vx    = max(-MAX_LIN, min(MAX_LIN, vx))
        vy    = max(-MAX_LIN, min(MAX_LIN, vy))
        omega = max(-MAX_ANG, min(MAX_ANG, omega))

        v_mag = math.sqrt(vx * vx + vy * vy)

        # ② 완전 정지 → stand 자세
        if v_mag < 0.005 and abs(omega) < 0.05:
            return self.get_stand_posture(roll, pitch, bh)

        # ③ cmd_vel → BezierGait 입력 변환
        pure_rotation = (v_mag < 0.005)
        if not pure_rotation:
            # 전진/후진/측방 모드 (회전이 같이 있을 수도)
            lat_frac = math.atan2(vy, vx)
            L_raw = v_mag * self.Tswing
            L = min(self.max_stride, L_raw)
            StepVelocity = max(v_mag, 0.05)
        else:
            # 제자리 회전 전용 모드:
            #   - 메인 L = 0  → forward 드리프트 없음 (진짜 제자리)
            #   - 메인 swing 의 clearance 는 유지 → 발 들기는 메인 단독
            #   - yaw_step 은 horizontal 변위만 (clearance=0)
            lat_frac = 0.0
            L = 0.0
            StepVelocity = max(abs(omega) * self.kin.L1 * 2.0, 0.05)

        # ④ Tstance 계산 (게이트별 ratio 적용)
        if L > 1e-6:
            Tstance = 2.0 * L / StepVelocity
            Tstance = max(self.Tswing * self.tstance_min_ratio,
                          min(self.Tswing * self.tstance_max_ratio, Tstance))
        elif pure_rotation:
            # L=0 이지만 phase 가 돌아가야 yaw_step 이 작동함
            Tstance = self.Tswing * self.tstance_min_ratio
        else:
            Tstance = 0.0
        Tstride = Tstance + self.Tswing

        # ④ 위상 증가
        if Tstance > self.dt:
            self.TD = True
        self._increment(Tstride)

        # ⑤ 각 다리에 대해 swing/stance 궤적 + yaw 회전 효과 합성 + IK
        neutrals = self._neutral_feet(bh)
        kp_r, kp_p = 0.5, 1.0
        refs = [(0.2, 0.1), (0.2, -0.1), (-0.2, 0.1), (-0.2, -0.1)]

        angles = []
        for i, (lx, ly) in enumerate(refs):
            phase, is_swing = self._get_phase(i, Tstance)

            # 메인 swing/stance — 발 들기/누름은 항상 여기서 담당 (yaw_step 은 horizontal 만)
            # L=0 (회전 모드) 일 때도 호출: stepX/Y 는 0 이지만 stepZ (clearance) 는 살아있음
            if Tstance > 0.0:
                if is_swing:
                    rx, ry, rz = self._bezier_swing(phase, L, lat_frac, self.clearance)
                else:
                    rx, ry, rz = self._sine_stance(phase, L, lat_frac, self.penetration)
            else:
                rx, ry, rz = 0.0, 0.0, 0.0

            # 회전 stride (좌/우 다리에 반대 방향 추가)
            yx, yy, yz = self._yaw_step(i, phase, is_swing, omega)

            # IMU 자세 보정 (roll/pitch)
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
