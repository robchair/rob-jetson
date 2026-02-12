#!/usr/bin/env python3
import time
import serial
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range


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
        self.declare_parameter("cmd_timeout_sec", 0.1)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.linear_deadband = float(self.get_parameter("linear_deadband").value)
        self.angular_deadband = float(self.get_parameter("angular_deadband").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self._last_sent_time = time.time()

        self._lock = threading.Lock()
        self._last_cmd = None
        self._last_msg_time = time.time()

        self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=1)
        time.sleep(2.0)  # Arduino reset grace
        self.get_logger().info(f"Connected serial {self.port} @ {self.baud}")

        self.sub = self.create_subscription(Twist, "/cmd_vel_safe", self.on_cmd_vel, 10)

        # Timer to stop chair if commands stop coming
        self.timer = self.create_timer(0.1, self.watchdog)
        self.declare_parameter("ultrasonic_frame_id", "ultrasonic_link")
        self.declare_parameter("ultrasonic_field_of_view", 0.5)  # radians-ish
        self.declare_parameter("ultrasonic_min_range", 0.02)      # meters
        self.declare_parameter("ultrasonic_max_range", 4.0)       # meters

        self.ultra_frame = self.get_parameter("ultrasonic_frame_id").value
        self.ultra_fov = float(self.get_parameter("ultrasonic_field_of_view").value)
        self.ultra_min = float(self.get_parameter("ultrasonic_min_range").value)
        self.ultra_max = float(self.get_parameter("ultrasonic_max_range").value)

        self.range_pub = self.create_publisher(Range, "/ultrasonic/range", 10)

        # Reader thread for telemetry
        self._reader_alive = True
        self.reader_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.reader_thread.start()

    def send(self, cmd: str):
        line = (cmd.strip() + "\n").encode("utf-8")
        with self._lock:
            self.ser.write(line)
            self.ser.flush()

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

        # Command refresh: re-send current command every 200ms
        # when obstacle clears, we need to re-assert "cmd"
        if self._last_cmd and (now - self._last_sent_time) > 0.2:
            self.send(self._last_cmd)

    def read_serial_loop(self):
        """
        Reads telemetry lines from Arduino like:
          RANGE_CM:12.34
        and publishes sensor_msgs/Range on /ultrasonic/range (meters).
        """
        while self._reader_alive and rclpy.ok():
            try:
                # Readline can block; serial timeout=1 in your Serial() constructor
                with self._lock:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()

                if not line:
                    continue

                # Parse: RANGE_CM:xx.xx
                if line.startswith("RANGE_CM:"):
                    val_str = line.split(":", 1)[1].strip()
                    dist_cm = float(val_str)
                    dist_m = dist_cm / 100.0

                    msg = Range()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.ultra_frame
                    msg.radiation_type = Range.ULTRASOUND
                    msg.field_of_view = self.ultra_fov
                    msg.min_range = self.ultra_min
                    msg.max_range = self.ultra_max
                    msg.range = dist_m

                    self.range_pub.publish(msg)

                # Optional: log other Arduino lines if helpful
                # else:
                #     self.get_logger().info(f"Arduino: {line}")

            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                time.sleep(0.1)


def main():
    rclpy.init()
    node = ArduinoBaseDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._reader_alive = False
        try:
            node.send("stop")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

