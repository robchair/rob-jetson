from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyUSB0')
    frame_id = LaunchConfiguration('frame_id', default='base_laser')

    return LaunchDescription([
        # 1. LiDAR Driver (LD19)
        # Using the SDK package we installed
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ldlidar'), 'launch', 'ld19.launch.py'
                ])
            ])
        ),

        # 2. LiDAR Odometry (RF2O)
        # Generates odom -> base_link TF
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom',
                'publish_tf': True,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'init_pose_from_topic': '',
                'freq': 20.0
            }],
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
