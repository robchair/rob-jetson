#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SafetyGate(Node):
    """
    MVP placeholder:
      /cmd_vel_raw  -> /cmd_vel_safe passthrough for now

    Later (Option B):
      Subscribe to /scan and /ultrasonic_range (or custom)
      and publish a safe/zeroed Twist when blocked.
    """

    def __init__(self):
        super().__init__("safety_gate")
        self.pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)
        self.sub = self.create_subscription(Twist, "/cmd_vel_raw", self.cb, 10)
        self.get_logger().info("SafetyGate passthrough running (/cmd_vel_raw -> /cmd_vel_safe)")

    def cb(self, msg: Twist):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = SafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

