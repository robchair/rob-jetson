#!/usr/bin/env python3
import time
import serial
import rclpy

from rclpy.node import Node
from std_msgs.msg import Int64MultiArray


class EncoderSerialNode(Node):
    """
    Reads encoder telemetry from a dedicated Arduino over serial.

    Expected serial lines:
      ENC,L=212,R=301,T=36016

    Publishes:
      /wheel_encoder_ticks  (std_msgs/Int64MultiArray)
        data[0] = left_count
        data[1] = right_count
        data[2] = arduino_time_ms
    """

    def __init__(self):
        super().__init__("encoder_serial_node")

        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("topic", "/wheel_encoder_ticks")

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.topic = self.get_parameter("topic").value

        self.pub = self.create_publisher(Int64MultiArray, self.topic, 50)

        try:
            self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=1)
            time.sleep(2.0)  # Arduino reset grace
            self.get_logger().info(f"Connected to encoder Arduino on {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            raise

        self.timer = self.create_timer(0.001, self.read_serial_once)

    def read_serial_once(self):
        try:
            line = self.ser.readline().decode("utf-8", errors="ignore").strip()
            if not line:
                return

            if not line.startswith("ENC,"):
                return

            parsed = self.parse_encoder_line(line)
            if parsed is None:
                self.get_logger().warn(f"Could not parse line: {line}")
                return

            left_count, right_count, arduino_time_ms = parsed

            msg = Int64MultiArray()
            msg.data = [left_count, right_count, arduino_time_ms]
            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def parse_encoder_line(self, line: str):
        """
        Parse lines like:
          ENC,L=212,R=301,T=36016
        """
        try:
            parts = line.split(",")

            if len(parts) != 4:
                return None

            if parts[0] != "ENC":
                return None

            left_str = parts[1].split("=", 1)[1]
            right_str = parts[2].split("=", 1)[1]
            time_str = parts[3].split("=", 1)[1]

            left_count = int(left_str)
            right_count = int(right_str)
            arduino_time_ms = int(time_str)

            return left_count, right_count, arduino_time_ms

        except Exception:
            return None


def main():
    rclpy.init()
    node = EncoderSerialNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.ser.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

