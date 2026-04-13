"""
SLAM launch file for ROB wheelchair.
Launch ALONGSIDE mvp.launch.py (which handles twist_mux, safety_gate, arduino_base_driver).

Usage:
  Terminal 1:  ros2 launch wheelchair_bringup mvp.launch.py
  Terminal 2:  ros2 launch wheelchair_bringup slam.launch.py
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    slam_config = PathJoinSubstitution([
        FindPackageShare("wheelchair_bringup"),
        "config",
        "slam_toolbox.yaml"
    ])

    return LaunchDescription([

        # ── RPLiDAR A1 driver ──────────────────────────────────────
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': '/dev/rplidar',   # udev symlink (fall back to /dev/ttyUSB0)
                'serial_baudrate': 115200,
                'frame_id': 'base_laser',
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': '',
            }]
        ),

        # ── Encoder odom (publishing to /odom) ──────────────────────────────────────
        Node(
            package='wheelchair_localization',
            executable='encoder_odom',
            name='encoder_odom',
            output='screen',
            parameters=[{
                'encoder_topic': '/wheel_encoder_ticks',
                'odom_topic': '/odom',
                'wheel_radius_m': 0.305,
                'wheel_base_m': 0.515,
                'ticks_per_rev': 1199.67,
                'left_sign': 1.0,
                'right_sign': -1.0,
            }]
        ),

        # ── Static TF: base_link -> base_laser ─────────────────────
        # Measure actual RPLiDAR A1 mounting position on wheelchair
        #       and update x, y, z, yaw accordingly.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            arguments=[
                '0.32', '-0.2325', '0.82',    # x, y, z  (meters)
                '0', '0', '3.14159',           # roll, pitch, yaw (radians)
                'base_link', 'base_laser'      # parent, child
            ]
        ),

        # ── Static TF: base_link -> ultrasonic_link ────────────────
        # Dummy transform until ultrasonic is mounted
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_ultrasonic_tf',
            arguments=[
                '0.25', '0.00', '0.10',
                '0', '0', '0',
                'base_link', 'ultrasonic_link'
            ]
        ),

        # ── SLAM Toolbox (online async) ───────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('slam_toolbox'),
                    'launch',
                    'online_async_launch.py'
                ])
            ]),
            launch_arguments={
                'slam_params_file': slam_config,
                'use_sim_time': 'false',
            }.items()
        ),
    ])