import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_arg = DeclareLaunchArgument(
        "map",
        description="Full path to the map yaml file",
    )

    nav2_config = PathJoinSubstitution([
        FindPackageShare("wheelchair_bringup"),
        "config",
        "nav2_params.yaml",
    ])

    return LaunchDescription([
        map_arg,

            # ── RPLiDAR A1 driver ──────────────────────────────────────
    Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/ttyLIDAR',
            'serial_baudrate': 115200,
            'frame_id': 'base_laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': '',
        }]
    ),

    # ── Laser angle crop filter ────────────────────────────────
    Node(
        package='wheelchair_bringup',
        executable='scan_crop_node',
        name='scan_crop_node',
        output='screen',
        parameters=[{
            'crop_min_deg': 265.0,
            'crop_max_deg': 360.0,
        }]
    ),

    # ── Static TFs ─────────────────────────────────────────────
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_laser',
            '--x', '0.267',
            '--y', '-0.3175',
            '--z', '0.75',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '3.14159',
        ]
    ),

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

    # Map server
    Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            nav2_config,
            {"yaml_filename": LaunchConfiguration("map")},
        ],
    ),

    # AMCL
    Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[{
            "robot_model_type": "differential",
            "base_frame_id": "base_link",
            "odom_frame_id": "odom",
            "global_frame_id": "map",
            "scan_topic": "/scan_filtered",
            "min_particles": 200,
            "max_particles": 500,
            "tf_broadcast": True,
        }],
    ),

    # Nav2 bringup
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                "navigation_launch.py",
            ])
        ]),
        launch_arguments={
            "params_file": nav2_config,
            "use_sim_time": "false",
        }.items(),
    ),
])