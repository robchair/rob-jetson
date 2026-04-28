#!/usr/bin/env python3
import time
import math
import serial
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from std_msgs.msg import Int64MultiArray


class ArduinoBaseDriver(Node):
    def __init__(self):
        super().__init__("arduino_base_driver")

        # --- Parameters ---
        self.declare_parameter("port", "/dev/ttyARDUINO")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("cmd_timeout_sec", 0.5)
        self.declare_parameter("ultrasonic_frame_id", "ultrasonic_link")
        self.declare_parameter("ultrasonic_field_of_view", 0.5)
        self.declare_parameter("ultrasonic_min_range", 0.02)
        self.declare_parameter("ultrasonic_max_range", 4.0)

        # Differential drive parameters — must match encoder_odom
        self.declare_parameter("wheel_radius_m", 0.305)
        self.declare_parameter("wheel_base_m", 0.515)

        # Velocity limits (m/s and rad/s) — what the robot can physically do
        self.declare_parameter("max_linear_speed", 0.3)
        self.declare_parameter("max_angular_speed", 1.0)

        # PWM limits — max PWM sent to Arduino
        self.declare_parameter("max_pwm", 65)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.cmd_timeout = float(self.get_parameter("cmd_timeout_sec").value)
        self.ultra_frame = self.get_parameter("ultrasonic_frame_id").value
        self.ultra_fov = float(self.get_parameter("ultrasonic_field_of_view").value)
        self.ultra_min = float(self.get_parameter("ultrasonic_min_range").value)
        self.ultra_max = float(self.get_parameter("ultrasonic_max_range").value)

        self.wheel_radius = float(self.get_parameter("wheel_radius_m").value)
        self.wheel_base = float(self.get_parameter("wheel_base_m").value)
        self.max_linear = float(self.get_parameter("max_linear_speed").value)
        self.max_angular = float(self.get_parameter("max_angular_speed").value)
        self.max_pwm = int(self.get_parameter("max_pwm").value)

        # Compute max wheel speed for PWM scaling
        # At max_linear with no rotation, both wheels go at max_linear
        # At max rotation with no linear, wheels go at (wheel_base/2)*max_angular
        # The true max wheel speed is whichever is larger
        self.max_wheel_speed = max(
            self.max_linear,
            (self.wheel_base / 2.0) * self.max_angular
        )

        # --- State ---
        self._lock = threading.Lock()
        self._last_cmd_time = time.monotonic()
        self._last_sent_time = time.monotonic()
        self._last_pwm_str = None
        self._current_left_pwm = 0
        self._current_right_pwm = 0

        # --- Serial ---
        self.ser = serial.Serial(self.port, baudrate=self.baud, timeout=0.1)
        time.sleep(2.0)
        self.get_logger().info(f"Connected serial {self.port} @ {self.baud}")

        # --- Publishers ---
        self.range_pub = self.create_publisher(Range, "/ultrasonic/range", 10)
        self.encoder_pub = self.create_publisher(Int64MultiArray, "/wheel_encoder_ticks", 50)

        # --- Subscriber ---
        self.sub = self.create_subscription(Twist, "/cmd_vel_safe", self.on_cmd_vel, 10)

        # --- Serial reader thread ---
        self._reader_alive = True
        self.reader_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.reader_thread.start()

        self.get_logger().info(
            f"Diff drive: radius={self.wheel_radius}m, "
            f"base={self.wheel_base}m, max_pwm={self.max_pwm}"
        )

        # Ramping state
        self._target_left_pwm = 0
        self._target_right_pwm = 0
        self._current_left_pwm = 0
        self._current_right_pwm = 0
        self._last_ramp_time = time.monotonic() 

        # Ramp rate: PWM units per second
        # 150 PWM/s means reaching 60 PWM in 0.4s — smooth for keyboard,
        # transparent for Nav2 since Nav2's own ramp produces small
        # deltas between 200ms command cycles
        self._ramp_rate = 150.0

        # Replace the watchdog timer with a combined ramp+send timer
        # Running at 20Hz gives smooth ramping
        self.timer = self.create_timer(0.05, self._ramp_and_send)

    # ------------------------------------------------------------------ #
    #  Twist → PWM conversion                                             #
    # ------------------------------------------------------------------ #

    def twist_to_pwm(self, linear_x: float, angular_z: float):
        v_left = linear_x - (angular_z * self.wheel_base / 2.0)
        v_right = linear_x + (angular_z * self.wheel_base / 2.0)

        if self.max_wheel_speed > 0:
            left_pwm = int(round((v_left / self.max_wheel_speed) * self.max_pwm))
            right_pwm = int(round((v_right / self.max_wheel_speed) * self.max_pwm))
        else:
            left_pwm = 0
            right_pwm = 0

        left_pwm = max(-self.max_pwm, min(self.max_pwm, left_pwm))
        right_pwm = max(-self.max_pwm, min(self.max_pwm, right_pwm))

        PWM_DEADZONE = 15
        if abs(left_pwm) < PWM_DEADZONE:
            left_pwm = 0
        if abs(right_pwm) < PWM_DEADZONE:
            right_pwm = 0

        # Invert RIGHT to match motor wiring
        right_pwm = -right_pwm

        return left_pwm, right_pwm

    # ------------------------------------------------------------------ #
    #  Serial write                                                        #
    # ------------------------------------------------------------------ #
    def send_pwm(self, left: int, right: int):
        cmd = f"PWM,L={left},R={right}\n"
        with self._lock:
            try:
                self.ser.write(cmd.encode("utf-8"))
                self.ser.flush()
            except Exception as e:
                self.get_logger().warn(f"Serial write error: {e}")
        self._last_sent_time = time.monotonic()
        self._current_left_pwm = left
        self._current_right_pwm = right

    def _ramp_toward(self, current: float, target: int, max_step: float) -> float:
        if current < target:
            current += max_step
            if current > target:
                current = target
        elif current > target:
            current -= max_step
            if current < target:
                current = target
        return current

    def _ramp_and_send(self):
        now = time.monotonic()

        # Watchdog: if no cmd_vel received recently, target zero
        if (now - self._last_cmd_time) > self.cmd_timeout:
            self._target_left_pwm = 0
            self._target_right_pwm = 0
            if self._current_left_pwm == 0 and self._current_right_pwm == 0:
                # Already stopped — just send keepalive periodically
                if (now - self._last_sent_time) > 0.2:
                    self.send_pwm(0, 0)
                return

        # Compute ramp step for this cycle
        dt = now - self._last_ramp_time
        self._last_ramp_time = now
        max_step = self._ramp_rate * dt

        # Ramp current toward target
        new_left = self._ramp_toward(
            float(self._current_left_pwm), self._target_left_pwm, max_step
        )
        new_right = self._ramp_toward(
            float(self._current_right_pwm), self._target_right_pwm, max_step
        )

        left_out = int(round(new_left))
        right_out = int(round(new_right))

        # Only send if values changed or keepalive needed
        if (left_out != self._current_left_pwm or
            right_out != self._current_right_pwm or
            (now - self._last_sent_time) > 0.2):
            self.send_pwm(left_out, right_out)

        self._current_left_pwm = left_out
        self._current_right_pwm = right_out

    # ------------------------------------------------------------------ #
    #  cmd_vel callback                                                    #
    # ------------------------------------------------------------------ #

    def on_cmd_vel(self, msg: Twist):
        self._last_cmd_time = time.monotonic()
        left_pwm, right_pwm = self.twist_to_pwm(msg.linear.x, msg.angular.z)
        # Only set targets — the ramp timer handles actual sending
        self._target_left_pwm = left_pwm
        self._target_right_pwm = right_pwm


    # # ------------------------------------------------------------------ #
    # #  Watchdog                                                            #
    # # ------------------------------------------------------------------ #
    # def watchdog(self):
    #     now = time.monotonic()

    #     if (now - self._last_cmd_time) > self.cmd_timeout:
    #         if self._current_left_pwm != 0 or self._current_right_pwm != 0:
    #             self.send_pwm(0, 0)
    #             self.get_logger().info("Navigation timeout -> stop")
    #         return

    #     # Keepalive: resend current PWM every 200ms so Arduino watchdog
    #     # doesn't trigger
    #     if (now - self._last_sent_time) > 0.2:
    #         self.send_pwm(self._current_left_pwm, self._current_right_pwm)

    # ------------------------------------------------------------------ #
    #  Serial reader thread                                                #
    # ------------------------------------------------------------------ #
    def read_serial_loop(self):
        while self._reader_alive and rclpy.ok():
            try:
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("RANGE,"):
                    try:
                        dist_cm = float(line.split(",", 1)[1])
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
                    except ValueError:
                        self.get_logger().warn(f"Bad RANGE line: {line}")

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
        try:
            parts = line.split(",")
            if len(parts) != 4 or parts[0] != "ENC":
                return None
            left = int(parts[1].split("=", 1)[1])
            right = int(parts[2].split("=", 1)[1])
            ts = int(parts[3].split("=", 1)[1])
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
            node.send_pwm(0, 0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()