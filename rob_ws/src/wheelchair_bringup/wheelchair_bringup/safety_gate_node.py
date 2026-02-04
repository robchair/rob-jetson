#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


class SafetyGate(Node):
    """
    Gating layer:
      /cmd_vel_out  -> /cmd_vel_safe
    Stops motion when ultrasonic reports an obstacle within threshold.
    """

    def __init__(self):
        super().__init__("safety_gate")

        # Params
        self.declare_parameter("stop_distance_m", 0.20)     # 20 cm
        self.declare_parameter("clear_distance_m", 0.25)    # hysteresis: clear at 25 cm
        self.declare_parameter("hold_stop_sec", 0.30)       # once blocked, hold stop for 300 ms
        self.declare_parameter("ultrasonic_timeout_sec", 0.5)

        self.stop_d = float(self.get_parameter("stop_distance_m").value)
        self.clear_d = float(self.get_parameter("clear_distance_m").value)
        self.hold_stop = float(self.get_parameter("hold_stop_sec").value)
        self.ultra_timeout = float(self.get_parameter("ultrasonic_timeout_sec").value)

        # State
        self._last_ultra_time = 0.0
        self._last_range_m = None
        self._blocked_until = 0.0

        self.pub = self.create_publisher(Twist, "/cmd_vel_safe", 10)

        # mux output topic is /cmd_vel_out
        self.cmd_sub = self.create_subscription(Twist, "/cmd_vel_raw", self.on_cmd, 10)
        self.ultra_sub = self.create_subscription(Range, "/ultrasonic/range", self.on_ultra, 10)

        self.get_logger().info("SafetyGate running (/cmd_vel_out -> /cmd_vel_safe), gating on /ultrasonic/range")

    def on_ultra(self, msg: Range):
        now = time.time()
        self._last_ultra_time = now

        r = float(msg.range)

        # Treat invalid readings as "no obstacle" (Range msg uses min/max to bound validity)
        if r < msg.min_range or r > msg.max_range:
            self._last_range_m = None
            return

        self._last_range_m = r

        # Update blocked latch based on hysteresis
        if r <= self.stop_d:
            self._blocked_until = max(self._blocked_until, now + self.hold_stop)
        elif r >= self.clear_d:
            # allow unblock naturally (by time) - don't force extend
            pass

    def should_block(self) -> bool:
        now = time.time()

        # If we haven't heard ultrasonic recently, fail OPEN (do not block),
        if (now - self._last_ultra_time) > self.ultra_timeout:
            return False

        # If blocked latch is active, block
        if now < self._blocked_until:
            return True

        # If we have a valid range and it's within stop threshold, block
        if self._last_range_m is not None and self._last_range_m <= self.stop_d:
            return True

        return False

    def on_cmd(self, msg: Twist):
        if self.should_block():
            stop = Twist()  # zeros
            self.pub.publish(stop)
        else:
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

