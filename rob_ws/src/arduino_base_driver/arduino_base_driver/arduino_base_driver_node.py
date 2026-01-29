#!/usr/bin/env python3
import time
import serial
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class ArduinoBaseDriver(Node):
    """
    Subscribes to /cmd_vel_safe and sends discrete serial commands to Arduino.

    This is intentionally MVP-compatible with our current Arduino sketch
    Later we can upgrade this node and the Arduino firmware to accept continuous velocities.
    """

    def __init__(self):
        super().__init__("arduino_base_driver")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 9600)
        self.declare_parameter("linear_deadband", 0.05)
        self.declare_parameter("angular_deadband", 0.10)
        self.declare_parameter("cmd_timeout_sec", 0.5)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.linear_deadband = float(self.get_parameter("linear_deadband").value)
        self.angular_deadband = float(self.get_parameter("angular_deadband").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)

        self._lock = threading.Lock()
        self._last_cmd = None
        self._last_msg_time = time.time()

        self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=1)
        time.sleep(2.0)  # Arduino reset grace
        self.get_logger().info(f"Connected serial {self.port} @ {self.baud}")

        self.sub = self.create_subscription(Twist, "/cmd_vel_safe", self.on_cmd_vel, 10)

        # Timer to stop chair if commands stop coming OR refresh commands
        # 10Hz check
        self.timer = self.create_timer(0.1, self.watchdog)
        self._last_sent_time = 0.0

    def send(self, cmd: str):
        line = (cmd.strip() + "\n").encode("utf-8")
        with self._lock:
            self.ser.write(line)
            self.ser.flush()
            self._last_sent_time = time.time()

    def send_if_changed(self, cmd: str):
        if cmd == self._last_cmd:
            return
        self.send(cmd)
        self._last_cmd = cmd
        self.get_logger().info(f"Serial -> {cmd}")

    def on_cmd_vel(self, msg: Twist):
        self._last_msg_time = time.time()

        lin = msg.linear.x
        ang = msg.angular.z

        # Decide command with deadbands
        if abs(lin) < self.linear_deadband and abs(ang) < self.angular_deadband:
            cmd = "stop"
        elif abs(lin) >= abs(ang):  # prioritize translation if stronger
            cmd = "forward" if lin > 0 else "backward"
        else:
            cmd = "turnLeft" if ang > 0 else "turnRight"

        self.send_if_changed(cmd)

    def watchdog(self):
        now = time.time()
        # 1. Safety Timeout: If no ROS command recently, force stop
        if (now - self._last_msg_time) > self.cmd_timeout:
            self.send_if_changed("stop")
            return

        # 2. Command Refresh: Re-send current command every ~200ms
        # This fixes the issue where Arduino stops for obstalce but Python doesn't know,
        # so when obstacle clears, we need to re-assert "forward"
        if self._last_cmd and (now - self._last_sent_time) > 0.2:
            self.send(self._last_cmd)


def main():
    rclpy.init()
    node = ArduinoBaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.send("stop")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

