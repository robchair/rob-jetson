#!/usr/bin/env python3
"""
wheelchair_ekf.launch.py

Starts the full EKF sensor-fusion stack:
  1. ArduinoBaseDriver        -> /wheel_encoder_ticks, /ultrasonic/range
  2. EncoderOdom              -> /wheel/odom  (TF disabled — EKF owns odom->base_link)
  3. BNO055UartNode           -> /imu/data
  4. Static TF: base_link -> imu_link  (required for robot_localization to accept IMU)
  5. ekf_filter_node          -> /odometry/filtered, odom->base_link TF

Usage:
  ros2 launch wheelchair_bringup wheelchair_ekf.launch.py

Optional overrides:
  ros2 launch wheelchair_bringup wheelchair_ekf.launch.py \
      arduino_port:=/dev/ttyACM1 \
      imu_port:=/dev/ttyUSB0 \
      debug_calibration:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------ #
    #  Launch arguments                                                    #
    # ------------------------------------------------------------------ #
    imu_port_arg = DeclareLaunchArgument(
        "imu_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for BNO055 IMU",
    )
    # Set true to lower calibration thresholds for initial bringup/debugging
    debug_calibration_arg = DeclareLaunchArgument(
        "debug_calibration",
        default_value="false",
        description="If true, sets IMU cal thresholds to 1 so EKF starts before full cal",
    )

    ekf_config = os.path.join(
        get_package_share_directory("wheelchair_bringup"),
        "config",
        "ekf.yaml",
    )

    # ------------------------------------------------------------------ #
    #  2. Encoder odometry                                                 #
    # ------------------------------------------------------------------ #
    # IMPORTANT: publish_tf is False here.
    # The EKF node (ekf_filter_node) owns the odom -> base_link TF via
    # publish_tf: true in ekf.yaml. If encoder_odom also broadcasts this
    # transform, you get a TF conflict that causes jitter or dropped frames.
    encoder_odom = Node(
        package="wheelchair_localization",
        executable="encoder_odom",
        name="encoder_odom",
        output="screen",
        parameters=[{
            "encoder_topic":  "/wheel_encoder_ticks",
            "odom_topic":     "/wheel/odom",
            "base_frame":     "base_link",
            "odom_frame":     "odom",
            "publish_tf":     False,          # EKF owns this TF
            "wheel_radius_m": 0.305,
            "wheel_base_m":   0.515,
            "ticks_per_rev":  1199.67,
            "left_sign":      -1.0,
            "right_sign":     1.0,
        }],
    )

    # ------------------------------------------------------------------ #
    #  3. BNO055 IMU node                                                  #
    # ------------------------------------------------------------------ #
    # gate_on_startup: True means the node blocks until calibration passes.
    #
    # NOTE: The IMU publishes with frame_id="imu_link". The static TF below
    # tells robot_localization how imu_link relates to base_link so it can
    # transform IMU data into the robot's reference frame before fusion.
    imu_node = Node(
        package="wheelchair_localization",
        executable="imu_node",
        name="bno055_uart_node",
        output="screen",
        parameters=[{
            "port":                 LaunchConfiguration("imu_port"),
            "baud":                 115200,
            "frame_id":             "imu_link",
            "poll_hz":              15.0,        # was 20.0 — reduced for serial stability
            "gate_on_startup":      True,
            "required_gyro_cal":    3,
            "required_accel_cal":   3,
            "required_sys_cal":     1,           # was 0
            "required_mag_cal":     3,           # was 0 — must match test calibration
            "warn_bad_reads":       True,
            "orientation_cov_z":    0.05,
            "angular_vel_cov_z":    0.02,
        }],
    )

    # ------------------------------------------------------------------ #
    #  4. Static TF: base_link -> imu_link                                 #
    # ------------------------------------------------------------------ #
    # 
    # Arguments: x y z roll pitch yaw parent_frame child_frame
    # Adjust x/y/z to match where the BNO055 is physically mounted
    # on the wheelchair frame. For a flat mount with no rotation,
    # roll=pitch=yaw=0. If the IMU is rotated (e.g. mounted sideways),
    # adjust roll/pitch/yaw accordingly.
    #
    # Example: IMU mounted 0.1m forward, 0.0m left, 0.05m up from base_link,
    # no rotation:
    static_imu_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_imu",
        arguments=[#Update values based on the mounting of the IMU onto the chair 
            "-0.065",  "0.0",  "0.05",   # x  y  z  (meters from base_link origin)
            "0.0",  "0.0",  "0.0",    # roll pitch yaw (radians)
            "base_link",
            "imu_link",
        ],
    )

    # ------------------------------------------------------------------ #
    #  5. EKF filter node                                                  #
    # ------------------------------------------------------------------ #
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
        remappings=[
            # ekf_node publishes /odometry/filtered by default.
            # If you want it on a different topic, uncomment and change:
            # ("/odometry/filtered", "/odom/ekf"),
        ],
    )

    return LaunchDescription([
        
        imu_port_arg,
        debug_calibration_arg,
        LogInfo(msg="Starting wheelchair EKF fusion stack..."),
        encoder_odom,
        imu_node,
        static_imu_tf,
        ekf_node,
    ])