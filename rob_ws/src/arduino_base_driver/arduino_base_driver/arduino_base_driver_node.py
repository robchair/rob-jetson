#!/usr/bin/env python3
import time
import serial
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Int64MultiArray


class ArduinoBaseDriver(Node):
    """
    Single serial owner for the wheelchair Arduino.
    - Sends motor commands to /cmd_vel_safe
    - Reads RANGE telemetry  -> /ultrasonic/range
    - Reads ENC telemetry    -> /wheel_encoder_ticks
    """

    def __init__(self):
        super().__init__("arduino_base_driver")

        # --- Parameters ---
        self.declare_parameter("port",                  "/dev/ttyACM0")
        self.declare_parameter("baud",                  115200)
        self.declare_parameter("linear_deadband",       0.05)
        self.declare_parameter("angular_deadband",      0.10)
        self.declare_parameter("cmd_timeout_sec",       0.1)
        self.declare_parameter("ultrasonic_frame_id",   "ultrasonic_link")
        self.declare_parameter("ultrasonic_field_of_view", 0.5)
        self.declare_parameter("ultrasonic_min_range",  0.02)
        self.declare_parameter("ultrasonic_max_range",  4.0)

        self.port           = self.get_parameter("port").value
        self.baud           = int(self.get_parameter("baud").value)
        self.linear_db      = float(self.get_parameter("linear_deadband").value)
        self.angular_db     = float(self.get_parameter("angular_deadband").value)
        self.cmd_timeout    = float(self.get_parameter("cmd_timeout_sec").value)
        self.ultra_frame    = self.get_parameter("ultrasonic_frame_id").value
        self.ultra_fov      = float(self.get_parameter("ultrasonic_field_of_view").value)
        self.ultra_min      = float(self.get_parameter("ultrasonic_min_range").value)
        self.ultra_max      = float(self.get_parameter("ultrasonic_max_range").value)

        # --- State ---
        self._lock           = threading.Lock()
        self._last_cmd       = None
        self._last_msg_time  = time.monotonic()
        self._last_sent_time = time.monotonic()

        # --- Serial ---
        self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=1)
        time.sleep(2.0)  # Arduino reset grace
        self.get_logger().info(f"Connected serial {self.port} @ {self.baud}")

        # --- Publishers ---
        self.range_pub   = self.create_publisher(Range,            "/ultrasonic/range",      10)
        self.encoder_pub = self.create_publisher(Int64MultiArray,  "/wheel_encoder_ticks",   50)

        # --- Subscriber ---
        self.sub = self.create_subscription(Twist, "/cmd_vel_safe", self.on_cmd_vel, 10)

        # --- Watchdog timer ---
        self.timer = self.create_timer(0.1, self.watchdog)

        # --- Serial reader thread ---
        self._reader_alive = True
        self.reader_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.reader_thread.start()

    # ------------------------------------------------------------------ #
    #  Serial write                                                        #
    # ------------------------------------------------------------------ #
    def send(self, cmd: str):
        line = (cmd.strip() + "\n").encode("utf-8")
        with self._lock:
            self.ser.write(line)
            self.ser.flush()
        self._last_sent_time = time.monotonic()

    def send_if_changed(self, cmd: str):
        if cmd == self._last_cmd:
            return
        self.send(cmd)
        self._last_cmd = cmd
        self.get_logger().info(f"Serial -> {cmd}")

    # ------------------------------------------------------------------ #
    #  cmd_vel callback                                                    #
    # ------------------------------------------------------------------ #
    def on_cmd_vel(self, msg: Twist):
        self._last_msg_time = time.monotonic()

        lin = msg.linear.x
        ang = msg.angular.z

        if abs(lin) < self.linear_db and abs(ang) < self.angular_db:
            cmd = "stop"
        elif abs(lin) >= abs(ang):
            cmd = "forward" if lin > 0 else "backward"
        else:
            cmd = "turnLeft" if ang > 0 else "turnRight"

        # Only send if the command actually changed — avoids serial spam
        if cmd != self._last_cmd:
            self.send(cmd)
            self._last_cmd = cmd
            self.get_logger().info(f"Command changed -> {cmd}")
    # ------------------------------------------------------------------ #
    #  Watchdog                                                            #
    # ------------------------------------------------------------------ #
    def watchdog(self):
        now = time.monotonic()

        # 1. If navigation stack goes silent, stop the wheelchair
        if (now - self._last_msg_time) > self.cmd_timeout:
            if self._last_cmd != "stop":
                self.send("stop")
                self._last_cmd = "stop"
                self.get_logger().info("Navigation timeout -> stop")
            return

        # 2. keepalive: always resend current command every 100ms
        # Uses self.send() directly, not send_if_changed
        if self._last_cmd is not None and (now - self._last_sent_time) > 0.1:
            self.send(self._last_cmd)

    # ------------------------------------------------------------------ #
    #  Serial reader thread                                                #
    # ------------------------------------------------------------------ #
    def read_serial_loop(self):
        while self._reader_alive and rclpy.ok():
            try:
                with self._lock:
                    line = self.ser.readline().decode("utf-8", errors="ignore").strip()

                if not line:
                    continue

                # --- Ultrasonic: "RANGE,12.34" ---
                if line.startswith("RANGE,"):
                    try:
                        dist_cm = float(line.split(",", 1)[1])
                        dist_m  = dist_cm / 100.0

                        msg = Range()
                        msg.header.stamp    = self.get_clock().now().to_msg()
                        msg.header.frame_id = self.ultra_frame
                        msg.radiation_type  = Range.ULTRASOUND
                        msg.field_of_view   = self.ultra_fov
                        msg.min_range       = self.ultra_min
                        msg.max_range       = self.ultra_max
                        msg.range           = dist_m
                        self.range_pub.publish(msg)
                    except ValueError:
                        self.get_logger().warn(f"Bad RANGE line: {line}")

                # --- Encoders: "ENC,L=212,R=301,T=36016" ---
                elif line.startswith("ENC,"):
                    parsed = self._parse_enc(line)
                    if parsed:
                        left, right, ts = parsed
                        msg = Int64MultiArray()
                        msg.data = [left, right, ts]
                        self.encoder_pub.publish(msg)
                    else:
                        self.get_logger().warn(f"Bad ENC line: {line}")

            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                time.sleep(0.1)

    def _parse_enc(self, line: str):
        """Parse 'ENC,L=212,R=301,T=36016' -> (left, right, time_ms)"""
        try:
            parts = line.split(",")
            if len(parts) != 4 or parts[0] != "ENC":
                return None
            left  = int(parts[1].split("=", 1)[1])
            right = int(parts[2].split("=", 1)[1])
            ts    = int(parts[3].split("=", 1)[1])
            return left, right, ts
        except Exception:
            return None


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
