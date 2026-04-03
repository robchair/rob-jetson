#!/usr/bin/env python3
"""
RoboBCI - EEG/ACC Control Node for Smart Wheelchair
Uses MUSE 2 accelerometer (via muselsl/pylsl) for head-tilt steering
and EEG alpha/beta ratio for attention-based go/stop gating.

Prerequisites:
  Terminal 1: muselsl stream --address 00:55:DA:B8:34:01 --acc
  Terminal 2: ros2 run <your_package> eeg_cmd_node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from pylsl import StreamInlet, resolve_streams
import numpy as np
from collections import deque
import time


class EegCmdNode(Node):
    def __init__(self):
        super().__init__('eeg_cmd_node')

        # --- Parameters (tune these) ---
        self.declare_parameter('max_linear_speed', 0.3)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('x_threshold', 0.10)       # left/right tilt deadzone
        self.declare_parameter('y_forward_threshold', 0.25) # Y above this = forward
        self.declare_parameter('y_neutral', 0.18)           # Y at rest (from your data)
        self.declare_parameter('attention_threshold', 0.5)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('use_attention_gate', True)  # disable until EEG is tuned

        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.x_thresh = self.get_parameter('x_threshold').value
        self.y_fwd_thresh = self.get_parameter('y_forward_threshold').value
        self.y_neutral = self.get_parameter('y_neutral').value
        self.attn_thresh = self.get_parameter('attention_threshold').value
        self.use_attention = self.get_parameter('use_attention_gate').value
        rate = self.get_parameter('publish_rate').value

        # --- Publisher ---
        self.publisher = self.create_publisher(Twist, '/cmd_vel_eeg', 10)

        # --- Connect to LSL streams ---
        self.acc_inlet = None
        self.eeg_inlet = None
        self.connect_streams()

        # --- EEG buffer for attention calculation ---
        self.eeg_buffer = deque(maxlen=256)

        # --- Calibration ---
        self.x_baseline = 0.0
        self.y_baseline = self.y_neutral
        self.calibrate()

        # --- Timer loop ---
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info('EEG CMD Node started')

    def connect_streams(self):
        """Find and connect to muselsl ACC and EEG streams."""
        self.get_logger().info('Looking for MUSE LSL streams...')
        streams = resolve_streams()

        for s in streams:
            if s.type() == 'ACC' and self.acc_inlet is None:
                self.acc_inlet = StreamInlet(s)
                self.get_logger().info('Connected to ACC stream')
            elif s.type() == 'EEG' and self.eeg_inlet is None:
                self.eeg_inlet = StreamInlet(s)
                self.get_logger().info('Connected to EEG stream')

        if self.acc_inlet is None:
            self.get_logger().error('No ACC stream found! Is muselsl stream --acc running?')
            raise RuntimeError('No ACC stream')

    def calibrate(self):
        """Read 2 seconds of ACC data to find neutral head position."""
        self.get_logger().info('Calibrating... hold your head in neutral position')
        samples = []
        start = time.time()

        while time.time() - start < 2.0:
            sample, ts = self.acc_inlet.pull_sample(timeout=0.1)
            if sample:
                samples.append(sample)

        if samples:
            data = np.array(samples)
            self.x_baseline = np.mean(data[:, 0])
            self.y_baseline = np.mean(data[:, 1])
            self.get_logger().info(
                f'Calibrated - X baseline: {self.x_baseline:.3f}, '
                f'Y baseline: {self.y_baseline:.3f}'
            )
        else:
            self.get_logger().warn('No calibration data, using defaults')

    def compute_attention(self):
        """Alpha/beta ratio from EEG. Returns 0-1."""
        if len(self.eeg_buffer) < 256:
            return 1.0  # default to GO if not enough data

        data = np.array(list(self.eeg_buffer))
        # Use AF7 channel (index 1 in EEG stream: TP9, AF7, AF8, TP10, AUX)
        channel = data[:, 1]

        fft_vals = np.abs(np.fft.rfft(channel))
        freqs = np.fft.rfftfreq(len(channel), d=1.0 / 256.0)

        alpha_mask = (freqs >= 8) & (freqs <= 12)
        beta_mask = (freqs >= 13) & (freqs <= 30)

        alpha_power = np.mean(fft_vals[alpha_mask]) if np.any(alpha_mask) else 1.0
        beta_power = np.mean(fft_vals[beta_mask]) if np.any(beta_mask) else 0.0

        if alpha_power == 0:
            return 1.0

        ratio = beta_power / alpha_power
        attention = min(ratio / 3.0, 1.0)
        return attention

    def timer_callback(self):
        """Main loop: read ACC + EEG, compute Twist, publish."""
        msg = Twist()

        # --- Read all available ACC samples, use latest ---
        acc_sample = None
        while True:
            sample, ts = self.acc_inlet.pull_sample(timeout=0.0)
            if sample is None:
                break
            acc_sample = sample

        # --- Read all available EEG samples into buffer ---
        if self.eeg_inlet:
            while True:
                sample, ts = self.eeg_inlet.pull_sample(timeout=0.0)
                if sample is None:
                    break
                self.eeg_buffer.append(sample)

        if acc_sample is None:
            self.publisher.publish(msg)  # publish zero twist
            return

        x = acc_sample[0] - self.x_baseline
        y = acc_sample[1] - self.y_baseline

        # --- Left/Right steering ---
        if abs(x) > self.x_thresh:
            # Negative X = tilt left = turn left (positive angular.z)
            # Positive X = tilt right = turn right (negative angular.z)
            angular = -x / 0.7 * self.max_angular  # 0.7 is approx max tilt
            msg.angular.z = np.clip(angular, -self.max_angular, self.max_angular)

        # --- Forward/Back ---
        if abs(y) > 0.05:
            linear = y / 0.5 * self.max_linear  # 0.5 is approx max forward tilt
            msg.linear.x = np.clip(linear, -self.max_linear, self.max_linear)

        # --- Attention gate (optional) ---
        if self.use_attention:
            attention = self.compute_attention()
            if attention < self.attn_thresh:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                self.get_logger().debug(f'Attention {attention:.2f} - STOPPED')
                self.publisher.publish(msg)
                return

        self.get_logger().info(
            f'X:{x:+.2f} Y:{y:+.2f} | '
            f'lin:{msg.linear.x:+.2f} ang:{msg.angular.z:+.2f}'
        )

        self.publisher.publish(msg)

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EegCmdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
