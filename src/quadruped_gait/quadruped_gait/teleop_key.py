"""
teleop_key.py
=============
키보드 텔레옵 노드.

이동 조작:
  w / s     : 전진 / 후진
  a / d     : 좌 / 우 회전
  q / e     : 좌 / 우 횡이동 (stafe)
  Space / x : 정지

높이 조작:
  t         : 몸체 올리기 (+0.02m, 최대 0.35m) → 다리 펴짐
  b         : 몸체 낮추기 (-0.02m, 최소 0.15m) → 다리 굽힘

  ※ 높이를 바꾼 뒤 이동 명령을 내리면 변경된 높이 그대로 걸어갑니다.

Ctrl+C : 종료

발행 토픽:
  /cmd_vel           (geometry_msgs/Twist)
  /body_height_cmd   (std_msgs/Float32)
"""

import sys
import tty
import termios
import select
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


# 키 → (linear_x_sign, linear_y_sign, angular_z_sign)
MOVE_BINDINGS = {
    'w': ( 1,  0,  0),
    's': (-1,  0,  0),
    'a': ( 0,  0,  1),
    'd': ( 0,  0, -1),
    'q': ( 0,  1,  0),
    'e': ( 0, -1,  0),
}

HELP = """
--- Quadruped Keyboard Teleop ---
  w/s : 전진 / 후진
  a/d : 좌회전 / 우회전
  q/e : 좌 횡이동 / 우 횡이동
  t   : 몸체 올리기 (다리 펴짐)
  b   : 몸체 낮추기 (다리 굽힘)
  Space/x : 정지
  Ctrl+C  : 종료
---------------------------------
"""


def _get_key(timeout: float = 0.1) -> str:
    """Non-blocking 키 읽기."""
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

        self.declare_parameter('linear_speed',  0.3)
        self.declare_parameter('angular_speed', 0.8)
        self.declare_parameter('height_step',   0.02)
        self.declare_parameter('height_min',    0.15)
        self.declare_parameter('height_max',    0.35)
        self.declare_parameter('default_height', 0.27)

        self._lin  = self.get_parameter('linear_speed').value
        self._ang  = self.get_parameter('angular_speed').value
        self._step = self.get_parameter('height_step').value
        self._hmin = self.get_parameter('height_min').value
        self._hmax = self.get_parameter('height_max').value
        self._height = self.get_parameter('default_height').value

        self._cmd_pub    = self.create_publisher(Twist,   '/cmd_vel',          10)
        self._height_pub = self.create_publisher(Float32, '/body_height_cmd',  10)

        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0

        print(HELP)
        self._print_status()

    # ------------------------------------------------------------------

    def _print_status(self):
        print(
            f'\r속도: vx={self._vx:+.2f}  vy={self._vy:+.2f}  ω={self._omega:+.2f}'
            f'  |  높이: {self._height:.2f}m   ',
            end='', flush=True,
        )

    def _publish_cmd(self):
        msg = Twist()
        msg.linear.x  = self._vx
        msg.linear.y  = self._vy
        msg.angular.z = self._omega
        self._cmd_pub.publish(msg)

    def _publish_height(self):
        msg = Float32()
        msg.data = self._height
        self._height_pub.publish(msg)

    # ------------------------------------------------------------------

    def run(self):
        """메인 루프 (스피닝 대신 직접 실행)."""
        try:
            while rclpy.ok():
                key = _get_key()

                if key == '\x03':       # Ctrl+C
                    break

                if key in MOVE_BINDINGS:
                    lx, ly, az = MOVE_BINDINGS[key]
                    self._vx    = lx * self._lin
                    self._vy    = ly * self._lin
                    self._omega = az * self._ang

                elif key in (' ', 'x'):
                    self._vx = self._vy = self._omega = 0.0

                elif key == 't':
                    new_h = self._height + self._step
                    self._height = min(self._hmax, new_h)
                    self._publish_height()

                elif key == 'b':
                    new_h = self._height - self._step
                    self._height = max(self._hmin, new_h)
                    self._publish_height()

                self._publish_cmd()
                self._print_status()

        except Exception as e:
            self.get_logger().error(f'teleop 오류: {e}')
        finally:
            # 종료 시 정지 명령 발행
            self._vx = self._vy = self._omega = 0.0
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
