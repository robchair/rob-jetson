from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') # A1/A2 default
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value=serial_port),
        DeclareLaunchArgument('serial_baudrate', default_value=serial_baudrate),
        DeclareLaunchArgument('frame_id', default_value=frame_id),
        
        # 1. LiDAR Driver (LD06/LD19)
        # Using official ldrobotSensorTeam/ldlidar_ros2 driver
        Node(
            package='ldlidar_ros2',
            executable='ldlidar_ros2_node',
            name='ldlidar_node',
            output='screen',
            parameters=[
                {'product_name': 'LDLiDAR_LD06'},
                {'topic_name': 'scan'},
                {'frame_id': frame_id},
                {'port_name': serial_port},
                {'port_baudrate': 230400}, # LD06 default
                {'laser_scan_dir': True},
                {'enable_angle_crop_func': False}
            ]
        ),

        # 2. Static Transform (base_link -> laser)
        # Adjust XYZ/RPY as needed. Assuming laser is ~20cm forward, 10cm up.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_base_link_to_laser',
            arguments=['0.2', '0', '0.1', '0', '0', '0', 'base_link', 'laser']
        ),

        # 3. SLAM Toolbox (Async Mode)
        Node(
             package='slam_toolbox',
             executable='async_slam_toolbox_node',
             name='slam_toolbox',
             output='screen',
             parameters=[
               {'use_sim_time': False},
               {'odom_frame': 'odom'},
               {'map_frame': 'map'},
               {'base_frame': 'base_link'},
               {'scan_topic': '/scan'},
             ]
        ),

        # 4. Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            # arguments=['-d', rviz_config_file], # Optional: add later
            output='screen'
        )
    ])
