from lidar_module import LD20
import time

lidar = LD20("/dev/ttyUSB0")

print("Reading LD20 packets... (Ctrl+C to stop)")

while True:
    scan = lidar.get_scan()

    for angle, dist in scan:
        print(f"Angle: {angle:6.1f}°, Distance: {dist:.3f} m")

    print("------")
    time.sleep(0.1)
