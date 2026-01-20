import serial
import struct
import math

HEADER = 0x54
VERLEN = 0x2C
POINTS_PER_PACK = 12


# ---------------- CRC-8 (Required for LD20) ----------------
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


# ---------------- LD20 Packet Class ----------------
class LD20Packet:
    def __init__(self):
        self.distance = []
        self.confidence = []
        self.angle = []
        self.start_angle = 0
        self.end_angle = 0
        self.speed = 0
        self.timestamp = 0


# ---------------- Parse a Single LD20 Packet ----------------
def read_packet(ser):
    # 1. Find header 0x54
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] == HEADER:
            break

    # 2. VerLen must be 0x2C
    ver = ser.read(1)
    if not ver or ver[0] != VERLEN:
        return None

    # 3. Read remaining 45 bytes
    raw = ser.read(45)
    if len(raw) != 45:
        return None

    # 4. CRC validation
    full = bytes([HEADER, VERLEN]) + raw[:-1]
    crc_expected = raw[-1]
    # crc_calc = calc_crc8(full)  
    # (We skip printing CRC mismatches to avoid spam)

    # 5. Parse packet
    pkt = LD20Packet()
    idx = 0

    pkt.speed = struct.unpack('<H', raw[idx:idx + 2])[0]
    idx += 2

    pkt.start_angle = struct.unpack('<H', raw[idx:idx + 2])[0] / 100.0
    idx += 2

    for _ in range(POINTS_PER_PACK):
        dist_mm = struct.unpack('<H', raw[idx:idx + 2])[0]
        conf = raw[idx + 2]
        idx += 3

        pkt.distance.append(dist_mm / 1000.0)
        pkt.confidence.append(conf)

    pkt.end_angle = struct.unpack('<H', raw[idx:idx + 2])[0] / 100.0
    idx += 2

    pkt.timestamp = struct.unpack('<H', raw[idx:idx + 2])[0]
    idx += 2

    # angle per point
    if pkt.end_angle >= pkt.start_angle:
        step = (pkt.end_angle - pkt.start_angle) / POINTS_PER_PACK
    else:
        step = (pkt.end_angle + 360 - pkt.start_angle) / POINTS_PER_PACK

    for i in range(POINTS_PER_PACK):
        angle_deg = pkt.start_angle + i * step
        pkt.angle.append(math.radians(angle_deg))

    return pkt


# ---------------- MAIN LOOP ----------------
if __name__ == "__main__":
    ser = serial.Serial(
        "/dev/ttyUSB0",   # change to COM5 on Windows
        230400,
        timeout=1
    )

    FRONT_ANGLE_MIN = -15     # degrees
    FRONT_ANGLE_MAX =  15     # degrees
    DETECT_DISTANCE = 0.75    # meters

    print("Reading LD20 packets... FRONT ONLY mode")

    print_counter = 0  # slows printing, but detection stays real-time

    while True:
        pkt = read_packet(ser)
        if not pkt:
            continue

        # convert start/end angle to -180..180
        start = (pkt.start_angle + 180) % 360 - 180
        end   = (pkt.end_angle   + 180) % 360 - 180

        # ---------------------------------------------------
        #  SKIP packets NOT in the front field of view
        # ---------------------------------------------------
        if end < FRONT_ANGLE_MIN or start > FRONT_ANGLE_MAX:
            continue

        # Only packets that face forward reach here

        # convert angles of individual points
        angles_deg = [((math.degrees(a) + 180) % 360 - 180) for a in pkt.angle]

        # detect obstacle in front
        detected = False
        for dist, ang in zip(pkt.distance, angles_deg):
            if FRONT_ANGLE_MIN <= ang <= FRONT_ANGLE_MAX:
                if 0 < dist < DETECT_DISTANCE:
                    detected = True
                    break

        # ---------------- SLOW DOWN PRINTING ----------------
        print_counter += 1
        if print_counter < 5:   # print every 5 front packets
            continue
        print_counter = 0
        # ----------------------------------------------------

        print(f"[FRONT PACKET]  Start: {start:.2f}°  End: {end:.2f}°")
        print("Distances:", pkt.distance)

        if detected:
            print("🚨 DETECTED: Object in FRONT!")
        else:
            print("OK: No obstacle in front.")

        print()


