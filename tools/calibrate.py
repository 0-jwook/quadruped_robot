#!/usr/bin/env python3
"""
4족 로봇 서보 캘리브레이션 도구
=================================

한 관절씩 ±1° / ±5° 단위로 조정하면서 좌우/전후 대칭을 맞춥니다.
조정이 끝나면 hardware_bridge.py 의 SERVO_TRIMS 형식으로 결과를 출력합니다.

ROS 노드(hardware.launch.py)를 띄우지 않은 상태에서 사용하세요.
이 스크립트가 시리얼 포트를 직접 잡고 MCU에 명령을 보냅니다.

사용법:
    python3 calibrate.py                # 기본 포트 /dev/ttyACM0
    python3 calibrate.py /dev/ttyUSB0

키 매핑:
    d / a       : 다음 / 이전 다리 (FL → FR → RL → RR → ...)
    s / w       : 다음 / 이전 관절 (hip → thigh → calf → ...)
    + / -       : 현재 관절 +1° / -1°
    ] / [       : 현재 관절 +5° / -5°
    r           : HOME 자세로 리셋 (모든 관절 초기화)
    p           : 현재 trim 값을 SERVO_TRIMS 형식으로 출력
    q           : 종료 (종료 시도 자동 출력)

권장 캘리브 절차:
    1. 로봇을 옆으로 눕히거나 들어올린 상태 (다리 쫙 펴도 안전한 자세)
    2. ROS 노드(hardware.launch.py) 가 켜져 있으면 종료
    3. 스크립트 실행 → 자동으로 HOME 자세 (다리 쫙 편 일자 자세) 로 이동
    4. 다리 4개가 모두 곧게 수직 아래로 펴진 자세여야 함
    5. 좌/우 또는 전/후 어긋남이 보이면 어긋난 관절을 +/- 키로 조정
    6. 4개 다리가 모두 곧게 + 좌우 평행 + 전후 평행이 되면 OK
    7. 'p' 키로 trim 출력 → hardware_bridge.py 의 SERVO_TRIMS 에 반영
"""

import sys
import struct
import time
import termios
import tty
import select

try:
    import serial
except ImportError:
    print("pyserial 이 필요합니다: pip install pyserial")
    sys.exit(1)


# ── 상수 ────────────────────────────────────────────────────────────────────
LEG_NAMES = ['FL', 'FR', 'RL', 'RR']
JOINT_NAMES = ['hip', 'thigh', 'calf']

# raw HOME (다리 쫙 편 좌우 대칭 자세, SERVO_TRIMS=0 기준)
# FL/RL: thigh=0 (수직 아래), calf=180 (직선)
# FR/RR: thigh=180 (좌측 거울), calf=0 (좌측 거울)
RAW_HOME = [
    90.0,   0.0, 180.0,   # FL
    90.0, 180.0,   0.0,   # FR
    90.0,   0.0, 180.0,   # RL
    90.0, 180.0,   0.0,   # RR
]


def _load_servo_trims_from_file(path):
    """hardware_bridge.py 의 SERVO_TRIMS 딕셔너리를 파싱해서 가져옴.
    이미 캘리브된 trim 위에서 추가 미세조정 하기 위함."""
    import re
    try:
        text = open(path).read()
    except Exception:
        return None
    m = re.search(r"SERVO_TRIMS\s*=\s*\{(.*?)\}", text, re.DOTALL)
    if not m:
        return None
    body = m.group(1)
    result = {}
    pat = r"'(\w+)'\s*:\s*\(\s*([-\d.]+)\s*,\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\)"
    for leg_match in re.finditer(pat, body):
        leg = leg_match.group(1)
        result[leg] = tuple(float(leg_match.group(i)) for i in range(2, 5))
    return result if result else None


HARDWARE_BRIDGE_PATH = '/home/jaewook/Quardruped/src/quadruped_gait/quadruped_gait/hardware_bridge.py'
INITIAL_TRIMS = _load_servo_trims_from_file(HARDWARE_BRIDGE_PATH) or {
    'FL': (0.0, 0.0, 0.0),
    'FR': (0.0, 0.0, 0.0),
    'RL': (0.0, 0.0, 0.0),
    'RR': (0.0, 0.0, 0.0),
}

# 캘리브 시작 자세 = raw HOME + 현재 SERVO_TRIMS (이미 보정된 자세에서 시작)
HOME = list(RAW_HOME)
for _i, _leg in enumerate(LEG_NAMES):
    for _j in range(3):
        HOME[_i * 3 + _j] += INITIAL_TRIMS[_leg][_j]


# ── 시리얼 프로토콜 (hardware_bridge.py 와 동일, MCU CRC8 검증) ──────────
def _crc8(data: bytes) -> int:
    """CRC-8 polynomial 0x07, init 0x00 — MCU CRC8Update() 와 동일"""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def make_packet(angles_deg):
    """[0xAA, 0x55, ID=0x03, LEN=48, 12×float32, CRC8(ID+LEN+payload)]"""
    meta    = bytes([0x03, 48])
    payload = struct.pack('<12f', *angles_deg)
    return b'\xaa\x55' + meta + payload + bytes([_crc8(meta + payload)])


# ── 터미널 단일 키 입력 ────────────────────────────────────────────────────
def getch_non_blocking(timeout=0.05):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return None


# ── 출력 ────────────────────────────────────────────────────────────────────
def print_state(angles, leg_idx, joint_idx):
    cur = angles[leg_idx * 3 + joint_idx]
    raw = RAW_HOME[leg_idx * 3 + joint_idx]
    initial = HOME[leg_idx * 3 + joint_idx]      # raw + initial_trim
    total_trim = cur - raw                       # 누적 trim (현재 + 사용자 추가)
    delta = cur - initial                        # 이번 세션에서 추가된 양
    msg = (f"\r[ {LEG_NAMES[leg_idx]}.{JOINT_NAMES[joint_idx]} ] "
           f"명령={cur:6.1f}°   trim={total_trim:+6.1f}°  "
           f"(Δ {delta:+5.1f}°)   ")
    sys.stdout.write(msg)
    sys.stdout.flush()


def print_trims(angles):
    sys.stdout.write("\n\n")
    sys.stdout.write("=" * 60 + "\n")
    sys.stdout.write("hardware_bridge.py 의 SERVO_TRIMS 에 복사하세요:\n")
    sys.stdout.write("(누적값 — 기존 trim + 이번 추가 조정)\n")
    sys.stdout.write("=" * 60 + "\n")
    sys.stdout.write("SERVO_TRIMS = {\n")
    sys.stdout.write("    #        shoulder   thigh    calf\n")
    for i, leg in enumerate(LEG_NAMES):
        trims = [angles[i * 3 + j] - RAW_HOME[i * 3 + j] for j in range(3)]
        sys.stdout.write(f"    '{leg}': ({trims[0]:8.1f}, {trims[1]:7.1f}, {trims[2]:7.1f}),\n")
    sys.stdout.write("}\n")
    sys.stdout.write("=" * 60 + "\n\n")
    sys.stdout.flush()


# ── 메인 ────────────────────────────────────────────────────────────────────
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    print(__doc__)
    print(f"포트 열기: {port}")
    try:
        ser = serial.Serial(port, 115200, timeout=1)
    except Exception as e:
        print(f"실패: {e}")
        print("ROS 노드가 같은 포트를 잡고 있으면 종료 후 다시 시도하세요.")
        sys.exit(1)

    angles = list(HOME)
    leg_idx, joint_idx = 0, 0

    # 초기 자세 적용 (3회 전송 — 노이즈 대비)
    pkt = make_packet(angles)
    for _ in range(3):
        ser.write(pkt)
        time.sleep(0.05)

    print("\n현재 SERVO_TRIMS 적용 상태에서 시작:")
    for leg in LEG_NAMES:
        h, t, c = INITIAL_TRIMS[leg]
        print(f"  {leg}: hip={h:+5.1f}  thigh={t:+5.1f}  calf={c:+5.1f}")
    print("\n초기 자세(HOME + trim 적용) 도달. 추가 미세 조정 시작.\n")
    print_state(angles, leg_idx, joint_idx)

    last_send = time.time()
    SEND_INTERVAL = 0.1  # 10Hz

    try:
        while True:
            now = time.time()

            # 주기적으로 명령 재전송 (서보가 명령 유지하도록)
            if now - last_send > SEND_INTERVAL:
                ser.write(make_packet(angles))
                last_send = now

            ch = getch_non_blocking(timeout=0.05)
            if ch is None:
                continue

            changed = False
            if ch == 'q':
                break
            elif ch == 'd':
                leg_idx = (leg_idx + 1) % 4
                changed = True
            elif ch == 'a':
                leg_idx = (leg_idx - 1) % 4
                changed = True
            elif ch == 's':
                joint_idx = (joint_idx + 1) % 3
                changed = True
            elif ch == 'w':
                joint_idx = (joint_idx - 1) % 3
                changed = True
            elif ch in ('+', '='):
                angles[leg_idx * 3 + joint_idx] += 1.0
                changed = True
            elif ch == '-':
                angles[leg_idx * 3 + joint_idx] -= 1.0
                changed = True
            elif ch == ']':
                angles[leg_idx * 3 + joint_idx] += 5.0
                changed = True
            elif ch == '[':
                angles[leg_idx * 3 + joint_idx] -= 5.0
                changed = True
            elif ch == 'r':
                angles = list(HOME)
                changed = True
            elif ch == 'p':
                print_trims(angles)
                print_state(angles, leg_idx, joint_idx)
                continue
            elif ch == '\x03':  # Ctrl+C
                break

            if changed:
                # 0~180° 클램프
                angles[leg_idx * 3 + joint_idx] = max(0.0, min(180.0,
                    angles[leg_idx * 3 + joint_idx]))
                ser.write(make_packet(angles))
                last_send = now
                print_state(angles, leg_idx, joint_idx)
    finally:
        ser.close()
        print_trims(angles)


if __name__ == '__main__':
    main()
