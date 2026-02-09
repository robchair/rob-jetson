#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

'''
Simplified SLAM launch that builds directly off the working ld19.launch.py logic.
'''

def generate_launch_description():
    # LD19 LiDAR configuration (copied from ld19.launch.py)
    # -----------------------------------------------------
    # Lidar params
    product_name = 'LDLiDAR_LD19'
    topic_name = 'scan'
    frame_id = 'base_laser'
    port_name = '/dev/ttyUSB0'
    port_baudrate = 230400
    laser_scan_dir = True
    enable_angle_crop_func = False
    angle_crop_min = 135.0
    angle_crop_max = 225.0

    # LD19 Node
    ldlidar_node = Node(
        package='ldlidar',
        executable='ldlidar',
        name='ldlidar',
        output='screen',
        parameters=[
            {'product_name': product_name},
            {'topic_name': topic_name},
            {'frame_id': frame_id},
            {'port_name': port_name},
            {'port_baudrate': port_baudrate},
            {'laser_scan_dir': laser_scan_dir},
            {'enable_angle_crop_func': enable_angle_crop_func},
            {'angle_crop_min': angle_crop_min},
            {'angle_crop_max': angle_crop_max}
        ]
    )

    # Static TF: base_link -> base_laser (defined in ld19.launch.py)
    base_link_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_laser_ld19',
        arguments=['0','0','0.18','0','0','0','base_link', frame_id]
    )

    # -----------------------------------------------------
    # SLAM Additions
    # -----------------------------------------------------

    # Fake Odometry: odom -> base_link
    # We need this because we removed RF2O, and SLAM requires an 'odom' frame.
    fake_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        output='screen'
    )

    # SLAM Toolbox
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan'
        }],
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        ldlidar_node,
        base_link_to_laser_tf,
        fake_odom_tf,
        slam_node,
        rviz_node
    ])
