"""
vslam_ekf.launch.py

Full integrated stack:
  1. RealSense D435 camera
  2. Encoder odometry          -> /wheel/odom  (ticks from Arduino via demo.launch.py)
  3. BNO055 IMU                -> /imu/data
  4. Static TF: base_link -> imu_link
  5. EKF filter                -> /odometry/filtered  (odom -> base_link TF)
  6. Robot state publisher     -> TF from URDF
  7. RTAB-Map VSLAM            -> consumes /odometry/filtered + camera

NOTE: arduino_base_driver_node is owned by demo.launch.py
      It publishes /wheel_encoder_ticks (consumed here by encoder_odom)
      and consumes /cmd_vel_safe (from the control stack).

Usage:
  ros2 launch wheelchair_bringup vslam_ekf.launch.py

Localization mode (after map is built):
  ros2 launch wheelchair_bringup vslam_ekf.launch.py localization:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(context, *args, **kwargs):
    localization_val = LaunchConfiguration('localization').perform(context)
    imu_port_val = LaunchConfiguration('imu_port').perform(context)
    is_localization = localization_val.lower() == 'true'
    mapping_mode = "false" if is_localization else "true"

    pkg_share = get_package_share_directory('wheelchair_bringup')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    urdf_path = os.path.join(pkg_share, 'urdf', 'wheelchair.urdf')

    robot_description = ParameterValue(
        Command(['cat ', urdf_path]),
        value_type=str
    )

    # ------------------------------------------------------------------ #
    # 1. RealSense D435 camera
    # ------------------------------------------------------------------ #
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_color':             True,
            'enable_depth':             True,
            'enable_infra1':            False,
            'enable_infra2':            False,
            'align_depth.enable':       True,
            'rgb_camera.profile':       '640,480,30',
            'depth_module.profile':     '640,480,30',
            'publish_tf':               True,
            'tf_publish_rate':          0.0,
            'serial_no':                '',
        }],
        env={
            **os.environ,
            'LD_LIBRARY_PATH': '/opt/ros/humble/lib/aarch64-linux-gnu:'
                + os.environ.get('LD_LIBRARY_PATH', '')
        }
    )

    # ------------------------------------------------------------------ #
    # 2. Encoder odometry
    # publish_tf: False — EKF owns odom -> base_link TF
    # Consumes /wheel_encoder_ticks published by arduino_base_driver (demo.launch.py)
    # ------------------------------------------------------------------ #
    encoder_odom = Node(
        package='wheelchair_localization',
        executable='encoder_odom',
        name='encoder_odom',
        output='screen',
        parameters=[{
            'encoder_topic':  '/wheel_encoder_ticks',
            'odom_topic':     '/wheel/odom',
            'base_frame':     'base_link',
            'odom_frame':     'odom',
            'publish_tf':     False,
            'wheel_radius_m': 0.305,
            'wheel_base_m':   0.515,
            'ticks_per_rev':  1199.67,
            'left_sign':      -1.0,
            'right_sign':     1.0,
        }],
    )

    # ------------------------------------------------------------------ #
    # 3. BNO055 IMU
    # ------------------------------------------------------------------ #
    imu_node = Node(
        package='wheelchair_localization',
        executable='imu_node',
        name='bno055_uart_node',
        output='screen',
        parameters=[{
            'port':               imu_port_val,
            'baud':               115200,
            'frame_id':           'imu_link',
            'poll_hz':            20.0,
            'gate_on_startup':    True,
            'required_gyro_cal':  3,
            'required_accel_cal': 3,
            'required_sys_cal':   0,
            'required_mag_cal':   0,
            'warn_bad_reads':     True,
            'calibration_file':   '/home/rob/rob/imu_calibration.json',
        }],
    )

    # ------------------------------------------------------------------ #
    # 4. Static TF: base_link -> imu_link
    # Update x/y/z once IMU is physically mounted
    # ------------------------------------------------------------------ #
    static_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=[
            '-0.04', '0.0', '0.475',
            '0.0', '0.0', '0.0',
            'base_link', 'imu_link',
        ],
    )

    # ------------------------------------------------------------------ #
    # 5. EKF filter
    # Publishes /odometry/filtered and odom -> base_link TF
    # ------------------------------------------------------------------ #
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config],
    )

    # ------------------------------------------------------------------ #
    # 6. Robot state publisher
    # Publishes TF from URDF: base_link -> camera_link, ultrasonic_link
    # ------------------------------------------------------------------ #
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # ------------------------------------------------------------------ #
    # 7. RTAB-Map
    # Consumes /odometry/filtered instead of raw /odom
    # ------------------------------------------------------------------ #
    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id':                     'base_link',
            'odom_frame_id':                'odom',
            'map_frame_id':                 'map',
            'subscribe_depth':              True,
            'subscribe_scan':               False,
            'subscribe_odom':               True,
            'database_path':                '/home/rob/rob/rob_ws/rtabmap.db',
            'approx_sync':                  True,
            'approx_sync_max_interval':     0.1,
            'wait_for_transform':           0.8,
            'Mem/IncrementalMemory':        mapping_mode,
            'Mem/InitWMWithAllNodes':       localization_val,
            'Grid/Sensor':                  '1',
            'Grid/3D':                      'false',
            'Grid/FromDepth':               'true',
            'Grid/RangeMax':                '4.0',
            'Grid/CellSize':                '0.05',
            'RGBD/NeighborLinkRefining':    'true',
            'RGBD/OptimizeFromGraphEnd':    'false',
        }],
        remappings=[
            ('rgb/image',       '/camera/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/camera/color/camera_info'),
            ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
            ('odom',            '/odometry/filtered'),
        ],
    )

    return [
        camera_node,
        encoder_odom,
        imu_node,
        static_imu_tf,
        ekf_node,
        robot_state_publisher,
        rtabmap_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Set to true to run in localization-only mode'
        ),
        DeclareLaunchArgument(
            'imu_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for BNO055 IMU'
        ),
        LogInfo(msg="Starting integrated VSLAM + EKF stack..."),
        OpaqueFunction(function=launch_setup),
    ])