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
    - Reads motor commands from /cmd_vel_safe
    - Reads RANGE telemetry  -> /ultrasonic/range
    - Reads ENC telemetry    -> /wheel_encoder_ticks
    """

    def __init__(self):
        super().__init__("arduino_base_driver")

        # Parameters
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("linear_deadband", 0.05)
        self.declare_parameter("angular_deadband", 0.10)
        self.declare_parameter("cmd_timeout_sec", 0.1)

        self.declare_parameter("ultrasonic_frame_id", "ultrasonic_link")
        self.declare_parameter("ultrasonic_field_of_view", 0.5)
        self.declare_parameter("ultrasonic_min_range", 0.02)
        self.declare_parameter("ultrasonic_max_range", 4.0)

        self.declare_parameter("cmd_topic", "/cmd_vel_safe")
        self.declare_parameter("range_topic", "/ultrasonic/range")
        self.declare_parameter("encoder_topic", "/wheel_encoder_ticks")

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.linear_db = float(self.get_parameter("linear_deadband").value)
        self.angular_db = float(self.get_parameter("angular_deadband").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)

        self.ultra_frame = self.get_parameter("ultrasonic_frame_id").value
        self.ultra_fov = float(self.get_parameter("ultrasonic_field_of_view").value)
        self.ultra_min = float(self.get_parameter("ultrasonic_min_range").value)
        self.ultra_max = float(self.get_parameter("ultrasonic_max_range").value)

        self.cmd_topic = self.get_parameter("cmd_topic").value
        self.range_topic = self.get_parameter("range_topic").value
        self.encoder_topic = self.get_parameter("encoder_topic").value

        # State
        self._write_lock = threading.Lock()
        self._last_cmd = None
        self._last_msg_time = time.monotonic()
        self._last_sent_time = time.monotonic()
        self._reader_alive = True

        # Serial
        try:
            self.ser = serial.Serial(
                self.port,
                baudrate=self.baud,
                timeout=0.02,           # short timeout for responsive loop
                write_timeout=0.2
            )
            time.sleep(2.0)  # Arduino reset grace
            self.get_logger().info(f"Connected serial {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            raise

        # Publishers
        self.range_pub = self.create_publisher(Range, self.range_topic, 10)
        self.encoder_pub = self.create_publisher(Int64MultiArray, self.encoder_topic, 50)

        # Subscriber
        self.sub = self.create_subscription(Twist, self.cmd_topic, self.on_cmd_vel, 10)

        # Watchdog timer
        self.timer = self.create_timer(0.05, self.watchdog)

        # Serial reader thread
        self.reader_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.reader_thread.start()

    def send(self, cmd: str):
        line = (cmd.strip() + "\n").encode("utf-8")
        with self._write_lock:
            self.ser.write(line)
            self.ser.flush()
        self._last_sent_time = time.monotonic()

    def send_if_changed(self, cmd: str):
        if cmd == self._last_cmd:
            return
        self.send(cmd)
        self._last_cmd = cmd
        self.get_logger().info(f"Serial -> {cmd}")

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

        self.send_if_changed(cmd)

    def watchdog(self):
        now = time.monotonic()

        if (now - self._last_msg_time) > self.cmd_timeout:
            self.send_if_changed("stop")
            return

        if self._last_cmd is not None and (now - self._last_sent_time) > 0.2:
            self.send(self._last_cmd)

    def read_serial_loop(self):
        while self._reader_alive and rclpy.ok():
            try:
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # Support both old and new range formats
                if line.startswith("RANGE_CM:"):
                    self._handle_range_colon(line)
                elif line.startswith("RANGE,"):
                    self._handle_range_csv(line)
                elif line.startswith("ENC,"):
                    self._handle_enc(line)
                else:
                    # Uncomment if you want to see unparsed lines
                    # self.get_logger().info(f"Arduino: {line}")
                    pass

            except Exception as e:
                self.get_logger().warn(f"Serial read error: {e}")
                time.sleep(0.05)

    def _handle_range_colon(self, line: str):
        try:
            dist_cm = float(line.split(":", 1)[1].strip())
            self._publish_range_cm(dist_cm)
        except Exception:
            self.get_logger().warn(f"Bad RANGE_CM line: {line}")

    def _handle_range_csv(self, line: str):
        try:
            dist_cm = float(line.split(",", 1)[1].strip())
            self._publish_range_cm(dist_cm)
        except Exception:
            self.get_logger().warn(f"Bad RANGE line: {line}")

    def _publish_range_cm(self, dist_cm: float):
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

    def _handle_enc(self, line: str):
        parsed = self._parse_enc(line)
        if parsed is None:
            self.get_logger().warn(f"Bad ENC line: {line}")
            return

        left, right, ts = parsed
        msg = Int64MultiArray()
        msg.data = [left, right, ts]
        self.encoder_pub.publish(msg)

    def _parse_enc(self, line: str):
        """
        Parse:
          ENC,L=212,R=301,T=36016
        """
        try:
            parts = line.split(",")
            if len(parts) != 4:
                return None
            if parts[0] != "ENC":
                return None

            left = int(parts[1].split("=", 1)[1])
            right = int(parts[2].split("=", 1)[1])
            ts = int(parts[3].split("=", 1)[1])

            return left, right, ts
        except Exception:
            return None

    def close(self):
        self._reader_alive = False
        if self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)

        try:
            self.ser.close()
        except Exception:
            pass


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

        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()