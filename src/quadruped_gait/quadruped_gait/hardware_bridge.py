import math
import threading
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from trajectory_msgs.msg import JointTrajectory

try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False


# ---------------------------------------------------------------------------
# 서보 트림값 (기계적 조립 오차 보정, 단위: degree)
# ---------------------------------------------------------------------------
SERVO_TRIMS = {
    #        shoulder  thigh   calf
    'FL': (   1.0,   17.0,  -13.0),
    'FR': (   7.0,  -24.0,    4.0),
    'RL': (   0.0,   14.0,   -8.0),
    'RR': (   6.0,   -1.0,   15.0),
}


def _clamp(val: float, lo: float = 0.0, hi: float = 180.0) -> float:
    return max(lo, min(hi, val))


def _crc8(data: bytes) -> int:
    """CRC-8 (polynomial 0x07, init 0x00) — MCU CRC8Update()와 동일"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def _rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cr, cp, cy = math.cos(roll / 2), math.cos(pitch / 2), math.cos(yaw / 2)
    sr, sp, sy = math.sin(roll / 2), math.sin(pitch / 2), math.sin(yaw / 2)
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def ik_to_servo_deg(q1: float, q2: float, q3: float, leg: str):
    ts, tt, tc = SERVO_TRIMS[leg]
    is_right = leg in ('FR', 'RR')

    if not is_right:
        shoulder = _clamp( 90.0 + math.degrees(q1) + ts)
        thigh    = _clamp(  0.0 - math.degrees(q2) + tt)
        calf     = _clamp(180.0 - math.degrees(q3) + tc)
    else:
        shoulder = _clamp( 90.0 + math.degrees(q1) + ts)
        thigh    = _clamp(180.0 + math.degrees(q2) + tt)
        calf     = _clamp(  0.0 + math.degrees(q3) + tc)

    return shoulder, thigh, calf


class HardwareBridge(Node):

    RECONNECT_INTERVAL = 3.0   # 재연결 시도 간격 (초)

    def __init__(self):
        super().__init__('hardware_bridge')

        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)

        self._port = self.get_parameter('port').value
        self._baud = self.get_parameter('baudrate').value
        self._ser_lock = threading.Lock()
        self.ser = None
        self._last_reconnect = 0.0
        self._connect_time = 0.0

        self._connect()

        self.traj_sub = self.create_subscription(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            self._traj_callback,
            10,
        )
        self.imu_pub = self.create_publisher(Imu, '/imu', 10)

        self._stop_event = threading.Event()
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        self.get_logger().info('Hardware Bridge 시작.')

    # ------------------------------------------------------------------
    def _connect(self) -> bool:
        if not SERIAL_AVAILABLE:
            self.get_logger().warn('pyserial 미설치')
            return False
        with self._ser_lock:
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            try:
                self.ser = serial.Serial(self._port, self._baud, timeout=1.0)
                self.ser.reset_input_buffer()  # 연결 전 OS에 쌓인 구 메시지 버림
                self._connect_time = time.monotonic()
                self.get_logger().info(f'STM32 연결: {self._port} @ {self._baud} bps')
                return True
            except Exception as e:
                self.ser = None
                self.get_logger().error(f'연결 실패: {e}')
                return False

    def _reconnect_if_needed(self):
        now = time.monotonic()
        if now - self._last_reconnect < self.RECONNECT_INTERVAL:
            return
        self._last_reconnect = now
        self.get_logger().warn('시리얼 재연결 시도...')
        self._connect()

    # ------------------------------------------------------------------
    def _traj_callback(self, msg: JointTrajectory):
        with self._ser_lock:
            if not self.ser or not self.ser.is_open:
                return
        if not msg.points:
            return

        pos = msg.points[0].positions
        if len(pos) < 12:
            return

        fl = ik_to_servo_deg(pos[0],  pos[1],  pos[2],  'FL')
        fr = ik_to_servo_deg(pos[3],  pos[4],  pos[5],  'FR')
        rl = ik_to_servo_deg(pos[6],  pos[7],  pos[8],  'RL')
        rr = ik_to_servo_deg(pos[9],  pos[10], pos[11], 'RR')

        angles  = list(fl) + list(fr) + list(rl) + list(rr)
        meta    = bytes([0x03, 48])
        payload = struct.pack('<12f', *angles)
        packet  = b'\xaa\x55' + meta + payload + bytes([_crc8(meta + payload)])

        with self._ser_lock:
            try:
                self.ser.write(packet)
            except Exception as e:
                self.get_logger().error(f'쓰기 오류: {e}')
                self.ser = None   # 다음 _reconnect_if_needed 에서 재연결

        # 5초마다 진단 로그
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, '_last_log_t') or now_sec - self._last_log_t > 5.0:
            self._last_log_t = now_sec
            self.get_logger().info(
                f'TX: FL({fl[0]:.0f},{fl[1]:.0f},{fl[2]:.0f}) '
                f'FR({fr[0]:.0f},{fr[1]:.0f},{fr[2]:.0f}) '
                f'RL({rl[0]:.0f},{rl[1]:.0f},{rl[2]:.0f}) '
                f'RR({rr[0]:.0f},{rr[1]:.0f},{rr[2]:.0f})'
            )

    # ------------------------------------------------------------------
    def _serial_read_loop(self):
        while not self._stop_event.is_set():
            with self._ser_lock:
                ser = self.ser
            if not ser or not ser.is_open:
                self._reconnect_if_needed()
                time.sleep(0.5)
                continue

            try:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore').strip()
                if not line:
                    continue

                if line.startswith('IMU:'):
                    self._handle_imu(line)
                elif line.startswith('HB:'):
                    self._handle_heartbeat(line)
                elif line.startswith('[ERROR]'):
                    self.get_logger().error(f'MCU: {line}')

            except Exception as e:
                if not self._stop_event.is_set():
                    self.get_logger().warn(f'읽기 오류: {e}')
                    with self._ser_lock:
                        self.ser = None
                    self._reconnect_if_needed()
                    time.sleep(0.5)

    # ------------------------------------------------------------------
    def _handle_imu(self, line: str):
        try:
            parts = line[4:].split(',')
            if len(parts) != 3:
                return
            roll, pitch, yaw = map(lambda x: math.radians(float(x)), parts)
        except ValueError:
            return

        qx, qy, qz, qw = _rpy_to_quaternion(roll, pitch, yaw)
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'
        msg.orientation.x, msg.orientation.y = qx, qy
        msg.orientation.z, msg.orientation.w = qz, qw
        self.imu_pub.publish(msg)

    def _handle_heartbeat(self, line: str):
        # 형식: HB:<tick>,CRC:<n>,ERR:<n>,PKT:<n>,WDG:<n>,TO:<n>
        try:
            def _field(key):
                if key + ':' not in line:
                    return 0
                return int(line.split(key + ':')[1].split(',')[0])

            crc_n = _field('CRC')
            err_n = _field('ERR')
            pkt_n = _field('PKT')
            wdg_n = _field('WDG')
            to_n  = _field('TO')

            # 항상 출력 (MCU 상태 실시간 확인)
            self.get_logger().info(
                f'MCU HB — PKT:{pkt_n} CRC:{crc_n} ERR:{err_n} WDG:{wdg_n} TO:{to_n}'
            )

            # 연결 직후 3초는 초기화 기간 — PKT:0 이 정상
            since_connect = time.monotonic() - self._connect_time
            if pkt_n == 0 and since_connect > 3.0:
                self.get_logger().warn('MCU가 유효 패킷을 0개 받음 — UART 수신 불량')
            if wdg_n > 0:
                self.get_logger().warn(f'UART 워치독 발동 {wdg_n}회 — UART가 죽었다 복구됨')
            if to_n == 1:
                self.get_logger().warn('MCU 명령 타임아웃 — 서보 홀드 상태')
        except Exception:
            pass

    # ------------------------------------------------------------------
    def destroy_node(self):
        self._stop_event.set()
        with self._ser_lock:
            if self.ser and self.ser.is_open:
                self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HardwareBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
