#!/usr/bin/env python3
import math
import time
from typing import Optional, Tuple

import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import UInt8MultiArray


def to_signed_16(lsb: int, msb: int) -> int:
    value = (msb << 8) | lsb
    if value >= 32768:
        value -= 65536
    return value


class BNO055UartNode(Node):
    """
    BNO055 UART ROS 2 node with calibration gating.

    Publishes:
      /imu/data                sensor_msgs/Imu
      /imu/calibration_status  std_msgs/UInt8MultiArray
         data = [sys, gyro, accel, mag]

    Notes:
    - Blocks publishing until required calibration thresholds are met.
    """
    def __init__(self):
        super().__init__("bno055_uart_node")

        # ---------------- Parameters ---------------- #
        self.declare_parameter("port", "/dev/ttyIMU")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("imu_topic", "/imu/data")
        self.declare_parameter("calib_topic", "/imu/calibration_status")
        self.declare_parameter("frame_id", "imu_link")
        self.declare_parameter("poll_hz", 15.0)         # Originally 20
        
        self.last_calib_pub_time =0.0

        self.declare_parameter("gate_on_startup", True)
        self.declare_parameter("required_sys_cal", 0)
        self.declare_parameter("required_gyro_cal", 3)
        self.declare_parameter("required_accel_cal", 3)
        self.declare_parameter("required_mag_cal", 0)

        self.declare_parameter("use_quaternion", True)
        self.declare_parameter("publish_linear_accel", True)
        self.declare_parameter("warn_bad_reads", True)

        self.declare_parameter("orientation_cov_x", 0.03)
        self.declare_parameter("orientation_cov_y", 0.03)
        self.declare_parameter("orientation_cov_z", 0.05)

        self.declare_parameter("angular_vel_cov_x", 0.02)
        self.declare_parameter("angular_vel_cov_y", 0.02)
        self.declare_parameter("angular_vel_cov_z", 0.02)

        self.declare_parameter("linear_accel_cov_x", 0.10)
        self.declare_parameter("linear_accel_cov_y", 0.10)
        self.declare_parameter("linear_accel_cov_z", 0.10)

        self.port = self.get_parameter("port").value
        self.baud = int(self.get_parameter("baud").value)
        self.imu_topic = self.get_parameter("imu_topic").value
        self.calib_topic = self.get_parameter("calib_topic").value
        self.frame_id = self.get_parameter("frame_id").value
        self.poll_hz = float(self.get_parameter("poll_hz").value)

        self.gate_on_startup = bool(self.get_parameter("gate_on_startup").value)
        self.required_sys_cal = int(self.get_parameter("required_sys_cal").value)
        self.required_gyro_cal = int(self.get_parameter("required_gyro_cal").value)
        self.required_accel_cal = int(self.get_parameter("required_accel_cal").value)
        self.required_mag_cal = int(self.get_parameter("required_mag_cal").value)

        self.use_quaternion = bool(self.get_parameter("use_quaternion").value)
        self.publish_linear_accel = bool(self.get_parameter("publish_linear_accel").value)
        self.warn_bad_reads = bool(self.get_parameter("warn_bad_reads").value)

        self.prev_quat = None
        self.max_yaw_jump_rad = math.radians(30.0)

        self.orientation_cov = [
            float(self.get_parameter("orientation_cov_x").value),
            float(self.get_parameter("orientation_cov_y").value),
            float(self.get_parameter("orientation_cov_z").value),
        ]
        self.angular_vel_cov = [
            float(self.get_parameter("angular_vel_cov_x").value),
            float(self.get_parameter("angular_vel_cov_y").value),
            float(self.get_parameter("angular_vel_cov_z").value),
        ]
        self.linear_accel_cov = [
            float(self.get_parameter("linear_accel_cov_x").value),
            float(self.get_parameter("linear_accel_cov_y").value),
            float(self.get_parameter("linear_accel_cov_z").value),
        ]

        # ---------------- Publishers ---------------- #
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 20)
        self.calib_pub = self.create_publisher(UInt8MultiArray, self.calib_topic, 10)

        # ---------------- Serial setup ---------------- #
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(1.0)
        self.get_logger().info(f"Connected to BNO055 on {self.port} @ {self.baud}")

        if not self.check_chip_id():
            raise RuntimeError("BNO055 chip ID check failed")

        self.configure_sensor()

        # Calibration gate
        self.declare_parameter("calibration_file", "/home/rob/rob/imu_calibration.json")
        self.cal_file = self.get_parameter("calibration_file").value

        self.calibrated = False
        cal_loaded = self.load_calibration(self.cal_file)
        if cal_loaded:
            self.get_logger().info("Calibration offsets restored — skipping calibration gate")
            self.calibrated = True
        elif self.gate_on_startup:
            self.wait_for_calibration()
            self.save_calibration(self.cal_file)
        else:
            self.calibrated = True

        # Timer
        self.timer = self.create_timer(1.0 / self.poll_hz, self.poll)

    # ------------------------------------------------ #
    # UART helpers
    # ------------------------------------------------ #
    def write_reg(self, reg: int, value: int) -> bytes:
        self.ser.reset_input_buffer()
        cmd = bytes([0xAA, 0x00, reg, 0x01, value])
        self.ser.write(cmd)
        time.sleep(0.05)
        return self.ser.read(2)
    
    def read_reg(self, reg: int, length: int) -> bytes:
        self.ser.reset_input_buffer()
        cmd = bytes([0xAA, 0x01, reg, length])
        self.ser.write(cmd)

        # Read response header: wait for 0xBB then read length byte
        deadline = time.monotonic() + 0.1  # 100ms timeout
        while time.monotonic() < deadline:
            header = self.ser.read(1)
            if len(header) == 0:
                continue
            if header[0] == 0xBB:
                # Got success header, next byte is length
                len_byte = self.ser.read(1)
                if len(len_byte) == 0:
                    return b''
                payload_len = len_byte[0]
                if payload_len != length:
                    # Length mismatch — corrupt response
                    return b''
                payload = self.ser.read(payload_len)
                if len(payload) != payload_len:
                    return b''
                return bytes([0xBB, payload_len]) + payload
            elif header[0] == 0xEE:
                # Error response from BNO055
                err = self.ser.read(1)
                return b''
        return b''  # timeout

    def check_chip_id(self) -> bool:
        resp = self.read_reg(0x00, 1)
        ok = len(resp) >= 3 and resp[0] == 0xBB and resp[1] == 0x01 and resp[2] == 0xA0
        self.get_logger().info(f"Chip ID response: {resp!r}")
        return ok

    def configure_sensor(self):
        # CONFIG mode
        resp = self.write_reg(0x3D, 0x00)
        self.get_logger().info(f"Set CONFIG mode: {resp!r}")
        time.sleep(0.05)

        # UNIT_SEL = 0x00
        # Orientation: Android
        # Euler: degrees
        # Gyro: degrees/sec
        # Accel: m/s^2
        resp = self.write_reg(0x3B, 0x00)
        self.get_logger().info(f"Set UNIT_SEL: {resp!r}")
        time.sleep(0.05)

        # NDOF mode
        resp = self.write_reg(0x3D, 0x0C)
        #resp = self.write_reg(0x3D, 0x08) # Try IMU mode instead
        self.get_logger().info(f"Set NDOF mode: {resp!r}")
        time.sleep(0.1)

    def save_calibration(self, path: str):
        # Must be in CONFIG mode to read offsets
        self.write_reg(0x3D, 0x00)
        time.sleep(0.05)
        resp = self.read_reg(0x55, 22)
        self.write_reg(0x3D, 0x0C)
        time.sleep(0.1)
        if len(resp) < 24 or resp[0] != 0xBB or resp[1] != 0x16:
            self.get_logger().error("Failed to read calibration offsets")
            return False
        offsets = list(resp[2:24])
        import json
        with open(path, "w") as f:
            json.dump(offsets, f)
        self.get_logger().info(f"Calibration saved to {path}")
        return True

    def load_calibration(self, path: str) -> bool:
        import json
        import os
        if not os.path.exists(path):
            self.get_logger().warn(f"No calibration file found at {path}")
            return False
        with open(path, "r") as f:
            offsets = json.load(f)
        if len(offsets) != 22:
            self.get_logger().error("Calibration file has wrong length")
            return False
        # Must be in CONFIG mode to write offsets
        self.write_reg(0x3D, 0x00)
        time.sleep(0.05)
        for i, val in enumerate(offsets):
            self.write_reg(0x55 + i, val)
            time.sleep(0.01)
        self.write_reg(0x3D, 0x0C)
        time.sleep(0.1)
        self.get_logger().info(f"Calibration loaded from {path}")
        return True

 
    # ------------------------------------------------ #
    # Calibration
    # ------------------------------------------------ #
    def read_calibration_status(self) -> Optional[Tuple[int, int, int, int]]:
        resp = self.read_reg(0x35, 1)
        if len(resp) < 3 or resp[0] != 0xBB or resp[1] != 0x01:
            return None

        calib = resp[2]
        sys_cal = (calib >> 6) & 0x03
        gyro_cal = (calib >> 4) & 0x03
        accel_cal = (calib >> 2) & 0x03
        mag_cal = calib & 0x03
        return (sys_cal, gyro_cal, accel_cal, mag_cal)
    
    def read_all_imu_data(self):
        """Single burst read: gyro(0x14-0x19) + quat(0x20-0x27) + linacc(0x28-0x2D)
        
        Note: there's a gap from 0x1A-0x1F (Euler angles) which we read but ignore.
        Total: 0x14 to 0x2D = 26 bytes.
        """
        resp = self.read_reg(0x14, 26)
        if len(resp) < 28:  # 2 header + 26 data
            return None, None, None

        data = resp[2:]  # strip 0xBB + length byte

        # Gyro: bytes 0-5 (registers 0x14-0x19)
        gx_dps = to_signed_16(data[0], data[1]) / 16.0
        gy_dps = to_signed_16(data[2], data[3]) / 16.0
        gz_dps = to_signed_16(data[4], data[5]) / 16.0
        gyro = (math.radians(gx_dps), math.radians(gy_dps), math.radians(gz_dps))

        # bytes 6-11 are Euler angles (0x1A-0x1F) — skip

        # Quaternion: bytes 12-19 (registers 0x20-0x27)
        w = to_signed_16(data[12], data[13]) / 16384.0
        x = to_signed_16(data[14], data[15]) / 16384.0
        y = to_signed_16(data[16], data[17]) / 16384.0
        z = to_signed_16(data[18], data[19]) / 16384.0
        quat = (x, y, z, w)

        # Linear accel: bytes 20-25 (registers 0x28-0x2D)
        ax = to_signed_16(data[20], data[21]) / 100.0
        ay = to_signed_16(data[22], data[23]) / 100.0
        az = to_signed_16(data[24], data[25]) / 100.0
        linacc = (ax, ay, az)

        return quat, gyro, linacc

    def publish_calibration_status(self, status: Tuple[int, int, int, int]):
        msg = UInt8MultiArray()
        msg.data = [status[0], status[1], status[2], status[3]]
        self.calib_pub.publish(msg)

    def calibration_meets_threshold(self, status: Tuple[int, int, int, int]) -> bool:
        sys_cal, gyro_cal, accel_cal, mag_cal = status
        return (
            sys_cal >= self.required_sys_cal
            and gyro_cal >= self.required_gyro_cal
            and accel_cal >= self.required_accel_cal
            and mag_cal >= self.required_mag_cal
        )

    def wait_for_calibration(self):
        self.get_logger().info(
            "Waiting for IMU calibration: "
            f"SYS>={self.required_sys_cal}, "
            f"GYRO>={self.required_gyro_cal}, "
            f"ACCEL>={self.required_accel_cal}, "
            f"MAG>={self.required_mag_cal}"
        )

        last_log_time = 0.0
        while rclpy.ok():
            status = self.read_calibration_status()
            if status is None:
                now = time.monotonic()
                if now - last_log_time > 1.0:
                    self.get_logger().warn("Failed to read calibration status")
                    last_log_time = now
                time.sleep(0.2)
                continue

            self.publish_calibration_status(status)

            if self.calibration_meets_threshold(status):
                self.calibrated = True
                self.get_logger().info(
                    f"Calibration gate passed: SYS={status[0]} "
                    f"GYRO={status[1]} ACCEL={status[2]} MAG={status[3]}"
                )
                return

            now = time.monotonic()
            if now - last_log_time > 1.0:
                self.get_logger().info(
                    f"Calibration status: SYS={status[0]} "
                    f"GYRO={status[1]} ACCEL={status[2]} MAG={status[3]}"
                )
                last_log_time = now

            time.sleep(0.2)

    # ------------------------------------------------ #
    # Sensor reads
    # ------------------------------------------------ #
    def read_quaternion(self) -> Optional[Tuple[float, float, float, float]]:
        # 0x20..0x27, scale = 1 / 16384
        resp = self.read_reg(0x20, 8)
        if len(resp) < 10 or resp[0] != 0xBB or resp[1] != 0x08:
            return None

        w = to_signed_16(resp[2], resp[3]) / 16384.0
        x = to_signed_16(resp[4], resp[5]) / 16384.0
        y = to_signed_16(resp[6], resp[7]) / 16384.0
        z = to_signed_16(resp[8], resp[9]) / 16384.0
        return (x, y, z, w)

    def read_gyro(self) -> Optional[Tuple[float, float, float]]:
        # 0x14..0x19, deg/s with UNIT_SEL=0x00, convert to rad/s
        resp = self.read_reg(0x14, 6)
        if len(resp) < 8 or resp[0] != 0xBB or resp[1] != 0x06:
            return None

        gx_dps = to_signed_16(resp[2], resp[3]) / 16.0
        gy_dps = to_signed_16(resp[4], resp[5]) / 16.0
        gz_dps = to_signed_16(resp[6], resp[7]) / 16.0
        #print("Raw gyro response:",resp)
        return (
            math.radians(gx_dps),
            math.radians(gy_dps),
            math.radians(gz_dps),
        )

    def read_linear_accel(self) -> Optional[Tuple[float, float, float]]:
        # 0x28..0x2D, m/s^2, scale = 1 / 100
        resp = self.read_reg(0x28, 6)
        if len(resp) < 8 or resp[0] != 0xBB or resp[1] != 0x06:
            return None

        ax = to_signed_16(resp[2], resp[3]) / 100.0
        ay = to_signed_16(resp[4], resp[5]) / 100.0
        az = to_signed_16(resp[6], resp[7]) / 100.0
        return (ax, ay, az)

    # ------------------------------------------------ #
    # Poll loop
    # ------------------------------------------------ #
    def poll(self):
        # Keep publishing calibration status even after startup
        now = time.monotonic()
        if now - self.last_calib_pub_time > 1.0:
            status = self.read_calibration_status()
            if status is not None:
                self.publish_calibration_status(status)
            self.last_calib_pub_time = now
        
        if not self.calibrated:
                    return

        #quat = self.read_quaternion() if self.use_quaternion else None
        #gyro = self.read_gyro()
        #linacc = self.read_linear_accel() if self.publish_linear_accel else None

        quat, gyro, linacc = self.read_all_imu_data()

        if gyro is None:
            return

        if quat is not None:
            mag_sq = quat[0]**2 + quat[1]**2 + quat[2]**2 + quat[3]**2
            if abs(mag_sq - 1.0) > 0.1 or mag_sq < 0.01:
                self.get_logger().warn(
                    f"Bad quaternion (mag²={mag_sq:.3f}), skipping frame"
                )
                return

            # Temporal consistency: reject impossible yaw jumps
            if self.prev_quat is not None:
                # Extract yaw from current and previous quaternion
                prev_yaw = math.atan2(
                    2.0 * (self.prev_quat[3] * self.prev_quat[2] + self.prev_quat[0] * self.prev_quat[1]),
                    1.0 - 2.0 * (self.prev_quat[1]**2 + self.prev_quat[2]**2)
                )
                curr_yaw = math.atan2(
                    2.0 * (quat[3] * quat[2] + quat[0] * quat[1]),
                    1.0 - 2.0 * (quat[1]**2 + quat[2]**2)
                )
                dyaw = abs(math.atan2(math.sin(curr_yaw - prev_yaw), math.cos(curr_yaw - prev_yaw)))
                if dyaw > self.max_yaw_jump_rad:
                    self.get_logger().warn(
                        f"Yaw jump rejected: {math.degrees(dyaw):.1f}° in one frame"
                    )
                    return  # don't update prev_quat — wait for next good frame

            self.prev_quat = quat

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Orientation
        if quat is not None:
            msg.orientation.x = quat[0]
            msg.orientation.y = quat[1]
            msg.orientation.z = quat[2]
            msg.orientation.w = quat[3]
            msg.orientation_covariance[0] = self.orientation_cov[0]
            msg.orientation_covariance[4] = self.orientation_cov[1]
            msg.orientation_covariance[8] = self.orientation_cov[2]
        else:
            msg.orientation_covariance[0] = -1.0

        # Angular velocity
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.angular_velocity_covariance[0] = self.angular_vel_cov[0]
        msg.angular_velocity_covariance[4] = self.angular_vel_cov[1]
        msg.angular_velocity_covariance[8] = self.angular_vel_cov[2]

        # Linear acceleration
        if linacc is not None:
            msg.linear_acceleration.x = linacc[0]
            msg.linear_acceleration.y = linacc[1]
            msg.linear_acceleration.z = linacc[2]
            msg.linear_acceleration_covariance[0] = self.linear_accel_cov[0]
            msg.linear_acceleration_covariance[4] = self.linear_accel_cov[1]
            msg.linear_acceleration_covariance[8] = self.linear_accel_cov[2]
        else:
            msg.linear_acceleration_covariance[0] = -1.0

        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = BNO055UartNode()
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
