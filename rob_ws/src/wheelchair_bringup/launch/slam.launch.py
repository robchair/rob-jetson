#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = FindPackageShare('wheelchair_bringup')
    xacro_file = PathJoinSubstitution([
        pkg_share,
        'description',
        'wheelchair.urdf.xacro'
    ])

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', xacro_file]),
                    value_type=str
                )
            }],
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            output='screen',
        ),

        # LD19 LiDAR - Include the working launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ldlidar'),
                    'launch',
                    'ld19.launch.py'
                ])
            )
        ),

        # RF2O Laser Odometry
        # RF2O Laser Odometry
        # Node(
        #     package='rf2o_laser_odometry',
        #     executable='rf2o_laser_odometry_node',
        #     name='rf2o_laser_odometry',
        #     output='screen',
        #     parameters=[{
        #         'laser_scan_topic': '/scan',
        #         'odom_topic': '/odom',
        #         'publish_tf': True,
        #         'base_frame_id': 'base_link',
        #         'odom_frame_id': 'odom',
        #         'init_pose_from_topic': '',
        #         'freq': 20.0
        #     }],
        # ),

        # Fake Odometry (Static Transform) - TEMPORARY FIX
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
            output='screen'
        ),

        # SLAM Toolbox
        Node(
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
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'
        )
    ])
