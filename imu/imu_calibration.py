import serial
import time

PORT = '/dev/ttyUSB0'
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(1)


# ---------------- UART helpers ---------------- #

def write_reg(reg, val):
    ser.reset_input_buffer()
    cmd = bytes([0xAA, 0x00, reg, 0x01, val])
    ser.write(cmd)
    time.sleep(0.05)
    return ser.read(2)


def read_reg(reg, length):
    ser.reset_input_buffer()
    cmd = bytes([0xAA, 0x01, reg, length])
    ser.write(cmd)
    time.sleep(0.05)
    return ser.read(length + 2)


# ---------------- Init sensor ---------------- #

print("Setting CONFIG mode...")
print(write_reg(0x3D, 0x00))
time.sleep(0.1)

print("Setting NDOF mode...")
print(write_reg(0x3D, 0x0C))
time.sleep(0.3)


# ---------------- Calibration loop ---------------- #

print("\nStarting calibration monitor...\n")
print("Move the IMU as instructed:\n")
print("- Keep still → gyro")
print("- Tilt in different orientations → accel")
print("- Figure-8 motion → magnetometer\n")

while True:
    resp = read_reg(0x35, 1)

    if len(resp) >= 3 and resp[0] == 0xBB:
        calib = resp[2]

        sys_cal   = (calib >> 6) & 0x03
        gyro_cal  = (calib >> 4) & 0x03
        accel_cal = (calib >> 2) & 0x03
        mag_cal   = calib & 0x03

        print(f"SYS:{sys_cal}  GYRO:{gyro_cal}  ACCEL:{accel_cal}  MAG:{mag_cal}")

    else:
        print("Bad response:", resp)

    time.sleep(0.5)
