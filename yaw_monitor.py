#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class YawMonitor(Node):
    def __init__(self):
        super().__init__('yaw_monitor')
        self.create_subscription(Imu, '/imu/data', self.cb, 10)
        self.initial_yaw = None

    def cb(self, msg):
        o = msg.orientation
        # Extract yaw from quaternion
        siny = 2.0 * (o.w * o.z + o.x * o.y)
        cosy = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
        yaw_rad = math.atan2(siny, cosy)
        yaw_deg = math.degrees(yaw_rad)

        if self.initial_yaw is None:
            self.initial_yaw = yaw_deg

        delta = yaw_deg - self.initial_yaw
        # Wrap to [-180, 180]
        delta = (delta + 180) % 360 - 180

        gz = math.degrees(msg.angular_velocity.z)
        print(f"Yaw: {yaw_deg:7.1f}°  ΔYaw: {delta:7.1f}°  GyroZ: {gz:6.2f}°/s")

def main():
    rclpy.init()
    node = YawMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()