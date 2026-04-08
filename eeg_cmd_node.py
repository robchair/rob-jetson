#!/usr/bin/env python3
"""
RoboBCI - EEG/ACC Control Node for Smart Wheelchair
Uses MUSE 2 accelerometer for head-tilt steering and eyebrow raise as a toggle gate.
Prerequisites:
  Terminal 1: muselsl stream --address 00:55:DA:B8:34:01 --acc
  Terminal 2: python3 eeg_cmd_node.py
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pylsl import StreamInlet, resolve_streams
import numpy as np
from collections import deque
import time

class EegCmdNode(Node):
    def __init__(self):
        super().__init__('eeg_cmd_node')

        # --- Parameters ---
        self.declare_parameter('max_linear_speed',   0.3)
        self.declare_parameter('max_angular_speed',  0.5)
        self.declare_parameter('x_threshold',        0.15)   # from diagnostic
        self.declare_parameter('publish_rate',       20.0)
        # Eyebrow raise detection — delta-based, not absolute
        self.declare_parameter('brow_delta_thresh',  150.0)  # µV above rolling baseline
        self.declare_parameter('brow_min_samples',   4)      # ~16ms sustained spike
        self.declare_parameter('brow_cooldown',      2.0)    # seconds between toggles
        self.declare_parameter('brow_baseline_len',  128)    # samples for rolling baseline

        self.max_linear   = self.get_parameter('max_linear_speed').value
        self.max_angular  = self.get_parameter('max_angular_speed').value
        self.x_thresh     = self.get_parameter('x_threshold').value
        self.brow_delta   = self.get_parameter('brow_delta_thresh').value
        self.brow_min     = self.get_parameter('brow_min_samples').value
        self.brow_cool    = self.get_parameter('brow_cooldown').value
        brow_base_len     = self.get_parameter('brow_baseline_len').value
        rate              = self.get_parameter('publish_rate').value

        # --- State ---
        self.enabled          = False
        self.movement_allowed = False
        self.last_brow_time   = 0.0
        self.missed_samples   = 0
        self.MISS_LIMIT       = 20
        self.is_connected     = True

        # --- ACC calibration state ---
        self.x_baseline = 0.0
        self.y_baseline = 0.0

        # --- Publishers ---
        self.publisher  = self.create_publisher(Twist, '/cmd_vel_eeg', 10)
        self.status_pub = self.create_publisher(Bool,  '/eeg_status',  10)

        # --- Subscriber ---
        self.create_subscription(Bool, '/eeg_enabled', self._on_enabled, 10)

        # --- LSL streams ---
        self.acc_inlet = None
        self.eeg_inlet = None
        self.connect_streams()

        # --- EEG buffers ---
        # recent_eeg: last few samples to check for spike
        # baseline_eeg: longer rolling window to compute ambient level
        self.recent_eeg   = deque(maxlen=self.brow_min)
        self.baseline_eeg = deque(maxlen=brow_base_len)

        # --- ACC calibration ---
        self.calibrate()

        # --- Stream health check ---
        self._last_stream_check     = time.time()
        self._stream_check_interval = 2.0

        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        self.get_logger().info('EEG CMD Node ready — raise eyebrows to toggle movement')

    # ------------------------------------------------------------------

    def _on_enabled(self, msg: Bool):
        self.enabled = msg.data
        self.get_logger().info(f'EEG {"ENABLED" if self.enabled else "DISABLED"} via /eeg_enabled')

    def connect_streams(self):
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
        status = Bool(); status.data = True
        self.status_pub.publish(status)

    def calibrate(self):
        self.get_logger().info('Calibrating ACC... hold head still in neutral position')
        samples = []
        start = time.time()
        while time.time() - start < 2.0:
            sample, _ = self.acc_inlet.pull_sample(timeout=0.1)
            if sample:
                samples.append(sample)
        if samples:
            data = np.array(samples)
            self.x_baseline = np.mean(data[:, 0])
            self.y_baseline = np.mean(data[:, 1])
            self.get_logger().info(
                f'Calibrated — X baseline: {self.x_baseline:.3f}, Y baseline: {self.y_baseline:.3f}'
            )
        else:
            self.get_logger().warn('No calibration data — using defaults (0, 0)')

    def detect_eyebrow(self) -> bool:
        """
        Delta-based eyebrow raise detection.
        Returns True when the last brow_min EEG samples all exceed the rolling
        baseline by brow_delta_thresh µV on at least one frontal channel (0 or 3,
        which correspond to AF7 and AF8 on the MUSE 2).
        Using delta instead of absolute µV handles the saturating baseline we saw
        in diagnostics — it fires on change, not raw amplitude.
        """
        if len(self.recent_eeg) < self.brow_min or len(self.baseline_eeg) < 10:
            return False
        baseline = np.mean([np.max(np.abs(np.array(s)[[0, 3]])) for s in self.baseline_eeg])
        recent   = list(self.recent_eeg)[-self.brow_min:]
        return all(
            np.max(np.abs(np.array(s)[[0, 3]])) > baseline + self.brow_delta
            for s in recent
        )

    # ------------------------------------------------------------------

    def timer_callback(self):
        now = time.time()

        # --- Periodic stream health check ---
        if now - self._last_stream_check >= self._stream_check_interval:
            self._last_stream_check = now
            active = [s.type() for s in resolve_streams(wait_time=0.5)]
            if 'ACC' not in active:
                if self.is_connected:
                    self.is_connected = False
                    self.get_logger().warn('ACC stream lost — headset disconnected?')
                    status = Bool(); status.data = False
                    self.status_pub.publish(status)
                if self.enabled:
                    self.publisher.publish(Twist())
                return
            elif not self.is_connected:
                self.is_connected = True
                self.get_logger().info('ACC stream restored')

        # --- Drain ACC (keep latest sample only) ---
        acc_sample = None
        sample, _ = self.acc_inlet.pull_sample(timeout=0.05)
        if sample is not None:
            acc_sample = sample
            while True:
                s, _ = self.acc_inlet.pull_sample(timeout=0.0)
                if s is None:
                    break
                acc_sample = s

        # --- Drain EEG into both buffers ---
        if self.eeg_inlet:
            while True:
                sample, _ = self.eeg_inlet.pull_sample(timeout=0.0)
                if sample is None:
                    break
                self.baseline_eeg.append(sample)
                self.recent_eeg.append(sample)

        # --- Eyebrow raise toggle ---
        if self.detect_eyebrow():
            if now - self.last_brow_time > self.brow_cool:
                self.last_brow_time   = now
                self.movement_allowed = not self.movement_allowed
                state = 'UNLOCKED — tilt head to drive' if self.movement_allowed else 'LOCKED'
                self.get_logger().info(f'Eyebrow raise detected — {state}')

        # --- If UI-disabled, stay silent ---
        if not self.enabled:
            return

        # --- Handle lost ACC sample ---
        if acc_sample is None:
            self.missed_samples += 1
            if self.missed_samples >= self.MISS_LIMIT and self.is_connected:
                self.is_connected = False
                self.get_logger().warn('ACC stream lost — headset disconnected?')
                status = Bool(); status.data = False
                self.status_pub.publish(status)
            self.publisher.publish(Twist())
            return

        # Stream alive
        if not self.is_connected:
            self.is_connected = True
            self.get_logger().info('ACC stream restored')
        self.missed_samples = 0
        status = Bool(); status.data = True
        self.status_pub.publish(status)

        # --- Build Twist from head tilt ---
        msg = Twist()
        x = acc_sample[0] - self.x_baseline
        y = acc_sample[1] - self.y_baseline

        # Left/right turn from X tilt
        if abs(x) > self.x_thresh:
            angular = -x / 0.7 * self.max_angular
            msg.angular.z = float(np.clip(angular, -self.max_angular, self.max_angular))

        # Forward only from Y tilt (no reverse)
        if y > 0.05:
            linear = y / 0.5 * self.max_linear
            msg.linear.x = float(np.clip(linear, 0.0, self.max_linear))

        # --- Gate: zero out commands if movement is locked ---
        if not self.movement_allowed:
            msg.linear.x  = 0.0
            msg.angular.z = 0.0

        self.get_logger().info(
            f'X:{x:+.2f} Y:{y:+.2f} | lin:{msg.linear.x:+.2f} ang:{msg.angular.z:+.2f} '
            f'| gate:{"ON" if self.movement_allowed else "OFF"}'
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