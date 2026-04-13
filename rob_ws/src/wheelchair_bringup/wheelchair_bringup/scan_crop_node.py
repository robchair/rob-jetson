#!/usr/bin/env python3
"""
Scan crop node for ROB wheelchair.
Subscribes to /scan, sets ranges within [crop_min_deg, crop_max_deg] to inf,
republishes to /scan_filtered.

"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from math import inf, pi


class ScanCropNode(Node):
    def __init__(self):
        super().__init__('scan_crop_node')

        # Declare parameters with defaults (degrees)
        self.declare_parameter('crop_min_deg', 0.0)
        self.declare_parameter('crop_max_deg', 95.0)

        crop_min = self.get_parameter('crop_min_deg').value
        crop_max = self.get_parameter('crop_max_deg').value

        # Convert to radians
        self.crop_min_rad = crop_min * pi / 180.0
        self.crop_max_rad = crop_max * pi / 180.0

        self.pub = self.create_publisher(LaserScan, '/scan_filtered', 10)
        self.sub = self.create_subscription(LaserScan, '/scan', self.callback, 10)

        self.get_logger().info(
            f'Cropping scan ranges between {crop_min}° and {crop_max}° '
            f'({self.crop_min_rad:.4f} to {self.crop_max_rad:.4f} rad). '
            f'Publishing to /scan_filtered.'
        )

    def callback(self, msg: LaserScan):
        new_ranges = list(msg.ranges)
        angle = msg.angle_min

        for i in range(len(new_ranges)):
            # Normalize angle to [0, 2*pi)
            a = angle % (2 * pi)
            if self.crop_min_rad <= a <= self.crop_max_rad:
                new_ranges[i] = inf
            angle += msg.angle_increment

        msg.ranges = new_ranges
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanCropNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
