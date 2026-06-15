import math


# ─────────────────────────────────────────────────────────────────────────────
# 제스처 키프레임 정의
#
# 각 제스처 = 키프레임 리스트. 각 키프레임 = (duration_s, kind, params)
#   kind='pose'   : body pose (발 고정 몸통 6축). params=dict(dx,dy,dz,roll,pitch,yaw)
#   kind='height' : 몸통 높이만 변경. params=dict(height=...)
#   kind='hold'   : 현재 자세 유지 (params 무시)
#
# 키프레임 사이는 cosine 보간으로 부드럽게 전환.
# GesturePlayer 가 body_pose / 높이 컨트롤러로 실제 관절각 생성.
# ─────────────────────────────────────────────────────────────────────────────

D = math.radians   # degree → rad 헬퍼

GESTURES = {
    # 인사 — 앞쪽으로 숙였다 복귀
    'bow': [
        (0.8, 'pose', dict(pitch=D(18), dz=-0.01)),
        (0.6, 'hold', {}),
        (0.8, 'pose', dict()),
    ],
    # 기지개 — 앞으로 더 깊이 숙이고 뒤 올림 (개 스트레칭) 후 복귀
    'stretch': [
        (1.2, 'pose', dict(pitch=D(28), dx=-0.03)),
        (1.0, 'hold', {}),
        (1.0, 'pose', dict()),
    ],
    # 끄덕이기 — 앞뒤로 2회
    'nod': [
        (0.4, 'pose', dict(pitch=D(12))),
        (0.4, 'pose', dict(pitch=D(-6))),
        (0.4, 'pose', dict(pitch=D(12))),
        (0.4, 'pose', dict(pitch=D(-6))),
        (0.4, 'pose', dict()),
    ],
    # 갸웃 — 좌우로 기울이기
    'tilt': [
        (0.6, 'pose', dict(roll=D(14))),
        (0.5, 'hold', {}),
        (0.8, 'pose', dict(roll=D(-14))),
        (0.5, 'hold', {}),
        (0.6, 'pose', dict()),
    ],
    # 둘러보기 — yaw 좌우
    'look': [
        (0.8, 'pose', dict(yaw=D(15))),
        (0.4, 'hold', {}),
        (1.0, 'pose', dict(yaw=D(-15))),
        (0.4, 'hold', {}),
        (0.8, 'pose', dict()),
    ],
    # 몸 털기 — yaw 빠르게 좌우 진동
    'shake': [
        (0.18, 'pose', dict(yaw=D(12))),
        (0.18, 'pose', dict(yaw=D(-12))),
        (0.18, 'pose', dict(yaw=D(12))),
        (0.18, 'pose', dict(yaw=D(-12))),
        (0.18, 'pose', dict(yaw=D(10))),
        (0.18, 'pose', dict(yaw=D(-10))),
        (0.3,  'pose', dict()),
    ],
    # 까치발 — 몸통 최대로 올림 (더 높게)
    'tall': [
        (1.2, 'pose', dict(dz=0.05)),
        (1.0, 'hold', {}),
        (1.0, 'pose', dict()),
    ],
    # 앉기 — 뒤로 앉은 느낌 (높이 중간 + 뒤쪽으로 무게)
    'sit': [
        (1.2, 'height', dict(height=0.10)),
        (0.6, 'pose', dict(pitch=D(-10))),   # 앞 살짝 들고 (앉은 자세)
        (1.2, 'hold', {}),
    ],
    # 엎드리기 — 바닥에 납작 (훨씬 낮게, 수평) — sit(0.10,앞들림)과 명확히 구분
    'lie': [
        (1.8, 'height', dict(height=0.07)),
        (1.2, 'hold', {}),
    ],
    # 준비자세 — 기본 stand 복귀
    'ready': [
        (1.0, 'height', dict(height=0.14)),
        (0.5, 'pose', dict()),
    ],
}


def gesture_names():
    return list(GESTURES.keys())


class GesturePlayer:
    """
    제스처 키프레임 시퀀스를 재생하며 매 tick 관절각 생성.

    body_pose: BodyPoseController (pose 키프레임용)
    set_height_cb: 높이 키프레임에서 호출할 콜백 (height 값 전달)
    get_height_cb: 현재 몸통 높이 반환 콜백
    """

    def __init__(self, body_pose, dt=0.02):
        self.bp = body_pose
        self.dt = dt
        self._active = None        # 현재 제스처 이름
        self._frames = []
        self._idx = 0              # 현재 키프레임 인덱스
        self._t = 0.0              # 현재 키프레임 내 경과 시간
        self._from = self._zero_pose()   # 보간 시작 pose
        self._cur = self._zero_pose()     # 현재 pose
        self._height_from = 0.14
        self._height_cur = 0.14

    @staticmethod
    def _zero_pose():
        return dict(dx=0.0, dy=0.0, dz=0.0, roll=0.0, pitch=0.0, yaw=0.0)

    def is_active(self):
        return self._active is not None

    def start(self, name, current_height=0.14):
        """제스처 시작. 알 수 없는 이름이면 False."""
        if name not in GESTURES:
            return False
        self._active = name
        self._frames = GESTURES[name]
        self._idx = 0
        self._t = 0.0
        self._from = dict(self._cur)
        self._height_from = current_height
        self._height_cur = current_height
        return True

    def stop(self):
        self._active = None
        self._frames = []

    def step(self, body_height):
        """
        한 tick 진행. 재생 중이면 (joint_angles, height) 반환, 끝났으면 None.
        height 는 height 키프레임일 때만 변하고, 그 외엔 입력 body_height 유지.
        """
        if self._active is None:
            return None

        dur, kind, params = self._frames[self._idx]
        self._t += self.dt
        # 키프레임 내 진행률 (cosine ease)
        r = min(1.0, self._t / dur) if dur > 1e-6 else 1.0
        ease = 0.5 * (1.0 - math.cos(math.pi * r))

        height_active = False
        if kind == 'pose':
            target = self._zero_pose()
            target.update(params)
            self._cur = {k: self._from[k] + (target[k] - self._from[k]) * ease
                         for k in target}
            self._height_cur = body_height
        elif kind == 'height':
            target_h = params.get('height', body_height)
            self._height_cur = self._height_from + (target_h - self._height_from) * ease
            height_active = True
            # pose 는 중립(0)으로 수렴 (다른 제스처 중단 시 기울기 잔재 제거)
            self._cur = {k: self._from[k] * (1.0 - ease) for k in self._from}
        else:  # 'hold' — 직전 pose/높이 그대로 유지 (_cur, _height_cur 변경 안 함)
            pass

        # 키프레임 종료 → 다음으로
        if self._t >= dur:
            self._from = dict(self._cur)
            self._height_from = self._height_cur
            self._idx += 1
            self._t = 0.0
            if self._idx >= len(self._frames):
                self.stop()

        # 현재 pose + 높이로 관절각 생성
        angles = self.bp.get_pose_posture(
            dx=self._cur['dx'], dy=self._cur['dy'], dz=self._cur['dz'],
            roll=self._cur['roll'], pitch=self._cur['pitch'], yaw=self._cur['yaw'],
            body_height=self._height_cur)
        return angles, self._height_cur, height_active
