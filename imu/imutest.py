import serial
import time

port = '/dev/ttyUSB1'

print(f"Trying to open {port}...")
ser = serial.Serial(port, 115200, timeout=1)
print("Serial port opened successfully")

time.sleep(2)

cmd = bytes([0xAA, 0x01, 0x00, 0x01])
print("Sending chip ID read command...")
ser.write(cmd)

time.sleep(0.2)

response = ser.read(10)
print("Raw response:", response)

ser.close()
print("Port closed")
