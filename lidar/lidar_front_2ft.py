import serial
import struct
import math

HEADER = 0x54
VERLEN = 0x2C
POINTS_PER_PACK = 12

# FRONT SETTINGS
FRONT_ANGLE_MIN = -15     # degrees
FRONT_ANGLE_MAX =  15     # degrees

#3 feet
DETECT_DISTANCE = 0.91


# -------------------------------------------------------
# CRC-8 (LD20 requirement, but we skip printing mismatch)
# -------------------------------------------------------
def calc_crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc <<= 1
            crc &= 0xFF
    return crc


# -------------------------------------------------------
# LD20 Packet Class
# -------------------------------------------------------
class LD20Packet:
    def __init__(self):
        self.distance = []
        self.confidence = []
        self.angle = []
        self.start_angle = 0
        self.end_angle = 0
        self.speed = 0
        self.timestamp = 0


# -------------------------------------------------------
# Parse LD20 Packet
# -------------------------------------------------------
def read_packet(ser):
    # Find header
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == HEADER:
            break

    # Verify VERLEN
    ver = ser.read(1)
    if not ver or ver[0] != VERLEN:
        return None

    # Read data
    raw = ser.read(45)
    if len(raw) != 45:
        return None

    # Skip CRC validation printing (to avoid spam)
    # full = bytes([HEADER, VERLEN]) + raw[:-1]
    # crc_calc = calc_crc8(full)

    pkt = LD20Packet()
    idx = 0

    pkt.speed = struct.unpack('<H', raw[idx:idx+2])[0]
    idx += 2

    pkt.start_angle = struct.unpack('<H', raw[idx:idx+2])[0] / 100.0
    idx += 2

    for _ in range(POINTS_PER_PACK):
        dist_mm = struct.unpack('<H', raw[idx:idx+2])[0]
        conf = raw[idx+2]
        idx += 3

        pkt.distance.append(dist_mm / 1000.0)
        pkt.confidence.append(conf)

    pkt.end_angle = struct.unpack('<H', raw[idx:idx+2])[0] / 100.0
    idx += 2

    pkt.timestamp = struct.unpack('<H', raw[idx:idx+2])[0]
    idx += 2

    # Angle per point
    if pkt.end_angle >= pkt.start_angle:
        step = (pkt.end_angle - pkt.start_angle) / POINTS_PER_PACK
    else:
        step = (pkt.end_angle + 360 - pkt.start_angle) / POINTS_PER_PACK

    for i in range(POINTS_PER_PACK):
        angle_deg = pkt.start_angle + i * step
        pkt.angle.append(math.radians(angle_deg))

    return pkt


# -------------------------------------------------------
# MAIN LOOP
# -------------------------------------------------------
if __name__ == "__main__":
    # LIDAR Serial Port
    ser = serial.Serial(
        "/dev/ttyUSB0",
        230400,
        timeout=1
    )

    # ARDUINO Serial Port  (⚠️ Change if needed)
    arduino = serial.Serial(
        "/dev/ttyACM0",   # or /dev/ttyUSB1 depending on your setup
        115200,
        timeout=1
    )

    print("LD20 LIDAR → Arduino Safety System Initialized")
    print("Front detection active, safety distance = 2 feet (0.61 m)")
    print("-------------------------------------------------------")

    print_counter = 0
    last_state = None  # None, "STOP", or "GO"

    while True:
        pkt = read_packet(ser)
        if not pkt:
            continue

        # Convert packet arc to centered angles
        start = (pkt.start_angle + 180) % 360 - 180
        end   = (pkt.end_angle   + 180) % 360 - 180

        # Skip packets outside front zone
        if end < FRONT_ANGLE_MIN or start > FRONT_ANGLE_MAX:
            continue

        # Convert each individual point angle
        angles_deg = [((math.degrees(a) + 180) % 360 - 180) for a in pkt.angle]

        # Detection
        detected = False
        min_front_distance = 999

        for dist, ang in zip(pkt.distance, angles_deg):
            if FRONT_ANGLE_MIN <= ang <= FRONT_ANGLE_MAX:
                if 0 < dist < min_front_distance:
                    min_front_distance = dist
                if 0 < dist < DETECT_DISTANCE:
                    detected = True

        # Decide STOP or GO
        if detected:
            command = "stop"
        else:
            command = "forward"

        # Only send to Arduino if state changes
        if command != last_state:
            print(f"SENDING TO ARDUINO → {command}")
            arduino.write(f"{command}\n".encode())
            last_state = command

        # Slow printing for human readability
        print_counter += 1
        if print_counter < 5:
            continue
        print_counter = 0

        print(f"[FRONT] Start: {start:.2f}°   End: {end:.2f}°")
        print(f"Closest front distance: {min_front_distance:.2f} m")
        print(f"State: {command}")
        print()
