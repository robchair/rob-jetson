
import serial
import time

port = '/dev/ttyUSB1'
ser = serial.Serial(port, 115200, timeout=1)

def write_reg(reg, val):
    cmd = bytes([0xAA, 0x00, reg, 0x01, val])
    ser.write(cmd)
    time.sleep(0.05)
    return ser.read(2)

def read_reg(reg, length):
    cmd = bytes([0xAA, 0x01, reg, length])
    ser.write(cmd)
    time.sleep(0.05)
    return ser.read(length + 2)

time.sleep(1)

# Config mode
print("Set CONFIG:", write_reg(0x3D, 0x00))
time.sleep(0.05)

# NDOF mode
print("Set NDOF:", write_reg(0x3D, 0x0C))
time.sleep(0.2)

while True:
    resp = read_reg(0x1A, 2)

    if len(resp) >= 4 and resp[0] == 0xBB and resp[1] == 0x02:
        lsb = resp[2]
        msb = resp[3]
        heading_raw = (msb << 8) | lsb
        heading_deg = heading_raw / 16.0
        print(f"Heading: {heading_deg:.2f} deg")
    else:
        print("Bad response:", resp)

    time.sleep(0.2)
