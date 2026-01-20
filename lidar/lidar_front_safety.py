import serial
import struct
import math
from collections import deque

HEADER = 0x54
VERLEN = 0x2C
POINTS_PER_PACK = 12

FRONT_ANGLE_MIN = -15
FRONT_ANGLE_MAX =  15

DETECT_DISTANCE = 0.91   # 3 ft

# HYSTERESIS BUFFER (3 packets consistency required)
HYSTERESIS_COUNT = 3


class LD20Packet:
    def __init__(self):
        self.distance = []
        self.angle = []
        self.start_angle = 0
        self.end_angle = 0


def read_packet(ser):
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == HEADER:
            break

    ver = ser.read(1)
    if not ver or ver[0] != VERLEN:
        return None

    raw = ser.read(45)
    if len(raw) != 45:
        return None

    pkt = LD20Packet()
    idx = 0

    idx += 2
    pkt.start_angle = struct.unpack('<H', raw[idx:idx+2])[0] / 100.0
    idx += 2

    for _ in range(POINTS_PER_PACK):
        dist_mm = struct.unpack('<H', raw[idx:idx+2])[0]
        idx += 3
        pkt.distance.append(dist_mm / 1000.0)

    pkt.end_angle = struct.unpack('<H', raw[idx:idx+2])[0] / 100.0
    idx += 2
    idx += 2

    if pkt.end_angle >= pkt.start_angle:
        step = (pkt.end_angle - pkt.start_angle) / POINTS_PER_PACK
    else:
        step = (pkt.end_angle + 360 - pkt.start_angle) / POINTS_PER_PACK

    for i in range(POINTS_PER_PACK):
        pkt.angle.append(math.radians(pkt.start_angle + i * step))

    return pkt


def run_lidar_loop(on_event):

    ser = serial.Serial("/dev/ttyUSB0", 230400, timeout=1)
    print("LIDAR safety loop started (3 ft stop zone).")

    last_cmd = None
    buffer = deque(maxlen=HYSTERESIS_COUNT)

    while True:
        pkt = read_packet(ser)
        if not pkt:
            continue

        start = (pkt.start_angle + 180) % 360 - 180
        end   = (pkt.end_angle   + 180) % 360 - 180

        if end < FRONT_ANGLE_MIN or start > FRONT_ANGLE_MAX:
            continue

        angles_deg = [
            ((math.degrees(a) + 180) % 360 - 180)
            for a in pkt.angle
        ]

        stop_detected = any(
            FRONT_ANGLE_MIN <= ang <= FRONT_ANGLE_MAX and 0 < dist < DETECT_DISTANCE
            for dist, ang in zip(pkt.distance, angles_deg)
        )

        buffer.append("stop" if stop_detected else "forward")

        # Only change when last 3 readings match
        if len(buffer) == HYSTERESIS_COUNT and len(set(buffer)) == 1:
            cmd = buffer[-1]

            if cmd != last_cmd:
                print(f"[LIDAR EVENT] {cmd}")
                on_event(cmd)
                last_cmd = cmd
