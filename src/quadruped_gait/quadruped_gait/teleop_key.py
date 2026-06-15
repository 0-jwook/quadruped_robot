"""
teleop_key.py
=============
키보드 텔레옵 — 보행 / 바디포즈 / 제스처 통합.

모드 전환:
  1 : 보행(WALK) 모드
  2 : 바디포즈(POSE) 모드 — 발 고정, 몸통만 움직임
  (제스처는 모드 무관, 숫자키 외 단축키로 즉시 재생)

[WALK 모드]
  w/s : 전진 / 후진      a/d : 좌회전 / 우회전
  q/e : 좌/우 횡이동      Space/x : 정지

[POSE 모드] (발 고정, 몸통 6축)
  w/s : 몸통 앞/뒤        a/d : 몸통 좌/우
  r/f : 몸통 올림/내림    q/e : yaw 좌/우 회전
  z/c : roll 좌/우        t/g : pitch 앞/뒤
  Space : 중립 복귀

[높이] (공통)
  [ / ] : 몸체 낮추기 / 올리기

[제스처] (g 접두 없이 바로)
  키 → 제스처:
    h=bow(인사)  j=stretch(기지개)  k=nod(끄덕)  l=tilt(갸웃)
    n=look(둘러보기)  m=shake(몸털기)  ,=wave(손흔들기)
    .=tall(까치발)  o=sit(앉기)  p=lie(엎드리기)  i=ready(준비)

Ctrl+C : 종료

발행 토픽:
  /cmd_vel (Twist)  /body_pose (Twist)  /gesture (String)  /body_height_cmd (Float32)
"""

import sys
import tty
import termios
import select
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, String


# WALK: 키 → (vx, vy, omega) 부호
WALK_BINDINGS = {
    'w': ( 1,  0,  0), 's': (-1,  0,  0),
    'a': ( 0,  0,  1), 'd': ( 0,  0, -1),
    'q': ( 0,  1,  0), 'e': ( 0, -1,  0),
}

# POSE: 키 → (dx, dy, dz, roll, pitch, yaw) 부호
POSE_BINDINGS = {
    'w': ( 1, 0, 0, 0, 0, 0), 's': (-1, 0, 0, 0, 0, 0),   # 앞/뒤
    'a': ( 0, 1, 0, 0, 0, 0), 'd': ( 0,-1, 0, 0, 0, 0),   # 좌/우
    'r': ( 0, 0, 1, 0, 0, 0), 'f': ( 0, 0,-1, 0, 0, 0),   # 올림/내림
    'z': ( 0, 0, 0, 1, 0, 0), 'c': ( 0, 0, 0,-1, 0, 0),   # roll
    't': ( 0, 0, 0, 0, 1, 0), 'g': ( 0, 0, 0, 0,-1, 0),   # pitch
    'q': ( 0, 0, 0, 0, 0, 1), 'e': ( 0, 0, 0, 0, 0,-1),   # yaw
}

# 제스처 단축키
GESTURE_KEYS = {
    'h': 'bow', 'j': 'stretch', 'k': 'nod', 'l': 'tilt',
    'n': 'look', 'm': 'shake', ',': 'wave', '.': 'tall',
    'o': 'sit', 'p': 'lie', 'i': 'ready',
}

HELP = """
================= Quadruped Teleop =================
 모드:  1=보행(WALK)   2=바디포즈(POSE)

 [WALK]  w/s 전후  a/d 회전  q/e 횡이동  Space 정지
 [POSE]  w/s 앞뒤  a/d 좌우  r/f 상하  q/e yaw  z/c roll  t/g pitch  Space 중립
 [높이]  [ 낮추기   ] 올리기
 [제스처] h인사 j기지개 k끄덕 l갸웃 n둘러보기 m몸털기
          ,손흔들기 .까치발 o앉기 p엎드리기 i준비
 Ctrl+C 종료
====================================================
"""


def _get_key(timeout=0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if rlist else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class TeleopKey(Node):
    def __init__(self):
        super().__init__('teleop_key')

        self.declare_parameter('linear_speed',  0.12)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('pose_xy',       0.025)   # POSE 이동량 (m)
        self.declare_parameter('pose_z',        0.03)
        self.declare_parameter('pose_ang',      0.20)    # POSE 회전량 (rad ~11°)
        self.declare_parameter('height_step',   0.01)
        self.declare_parameter('height_min',    0.07)
        self.declare_parameter('height_max',    0.21)
        self.declare_parameter('default_height', 0.14)

        self._lin  = self.get_parameter('linear_speed').value
        self._ang  = self.get_parameter('angular_speed').value
        self._pxy  = self.get_parameter('pose_xy').value
        self._pz   = self.get_parameter('pose_z').value
        self._pang = self.get_parameter('pose_ang').value
        self._step = self.get_parameter('height_step').value
        self._hmin = self.get_parameter('height_min').value
        self._hmax = self.get_parameter('height_max').value
        self._height = self.get_parameter('default_height').value

        self._cmd_pub    = self.create_publisher(Twist,   '/cmd_vel',         10)
        self._pose_pub   = self.create_publisher(Twist,   '/body_pose',       10)
        self._gest_pub   = self.create_publisher(String,  '/gesture',         10)
        self._height_pub = self.create_publisher(Float32, '/body_height_cmd', 10)

        self._mode = 'WALK'
        self._vx = self._vy = self._omega = 0.0
        self._pose = [0.0]*6   # dx,dy,dz,roll,pitch,yaw

        print(HELP)
        self._print_status()

    def _print_status(self):
        if self._mode == 'WALK':
            print(f'\r[WALK] vx={self._vx:+.2f} vy={self._vy:+.2f} ω={self._omega:+.2f}'
                  f' | h={self._height:.2f}m   ', end='', flush=True)
        else:
            p = self._pose
            print(f'\r[POSE] xyz=({p[0]:+.2f},{p[1]:+.2f},{p[2]:+.2f})'
                  f' rpy=({p[3]:+.2f},{p[4]:+.2f},{p[5]:+.2f}) | h={self._height:.2f}m   ',
                  end='', flush=True)

    def _publish_cmd(self):
        m = Twist()
        m.linear.x, m.linear.y, m.angular.z = self._vx, self._vy, self._omega
        self._cmd_pub.publish(m)

    def _publish_pose(self):
        m = Twist()
        m.linear.x, m.linear.y, m.linear.z = self._pose[0], self._pose[1], self._pose[2]
        m.angular.x, m.angular.y, m.angular.z = self._pose[3], self._pose[4], self._pose[5]
        self._pose_pub.publish(m)

    def _publish_height(self):
        m = Float32(); m.data = self._height
        self._height_pub.publish(m)

    def run(self):
        try:
            while rclpy.ok():
                key = _get_key()
                if key == '\x03':
                    break
                if not key:
                    # POSE 모드는 hold 시간 있으니 주기적 재발행
                    if self._mode == 'POSE':
                        self._publish_pose()
                    elif self._mode == 'WALK':
                        self._publish_cmd()
                    continue

                # 모드 전환
                if key == '1':
                    self._mode = 'WALK'; self._vx=self._vy=self._omega=0.0
                    self._publish_cmd()
                elif key == '2':
                    self._mode = 'POSE'; self._pose=[0.0]*6
                    self._publish_pose()
                # 제스처 (모드 무관)
                elif key in GESTURE_KEYS:
                    g = String(); g.data = GESTURE_KEYS[key]
                    self._gest_pub.publish(g)
                    print(f'\n제스처: {GESTURE_KEYS[key]}')
                # 높이 (공통)
                elif key == '[':
                    self._height = max(self._hmin, self._height - self._step)
                    self._publish_height()
                elif key == ']':
                    self._height = min(self._hmax, self._height + self._step)
                    self._publish_height()
                # 모드별 조작
                elif self._mode == 'WALK':
                    if key in WALK_BINDINGS:
                        lx, ly, az = WALK_BINDINGS[key]
                        self._vx, self._vy, self._omega = lx*self._lin, ly*self._lin, az*self._ang
                    elif key in (' ', 'x'):
                        self._vx=self._vy=self._omega=0.0
                    self._publish_cmd()
                elif self._mode == 'POSE':
                    if key in POSE_BINDINGS:
                        b = POSE_BINDINGS[key]
                        scl = [self._pxy,self._pxy,self._pz,self._pang,self._pang,self._pang]
                        self._pose = [b[i]*scl[i] for i in range(6)]
                    elif key == ' ':
                        self._pose=[0.0]*6
                    self._publish_pose()

                self._print_status()
        except Exception as e:
            self.get_logger().error(f'teleop 오류: {e}')
        finally:
            self._vx=self._vy=self._omega=0.0
            self._publish_cmd()
            print('\n종료.')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKey()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
