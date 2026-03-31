from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    localization_val = LaunchConfiguration('localization').perform(context)
    is_localization = (localization_val.lower() == 'true')
    mapping_mode = "false" if is_localization else "true"

    base_frame = "base_link"

    remappings = [
        ('rgb/image',       '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
    ]

    pkg_share = get_package_share_directory('wheelchair_bringup')
    urdf_path = os.path.join(pkg_share, 'urdf', 'wheelchair.urdf')

    robot_description = ParameterValue(
        Command(['cat ', urdf_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    rtabmap_node = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': base_frame,
            'odom_frame_id': 'odom',
            'map_frame_id': 'map',
            'subscribe_depth': True,
            'subscribe_scan': False,
            'database_path': '/home/rob/rob/rob_ws/rtabmap.db',
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'wait_for_transform': 0.8,
            'Mem/IncrementalMemory': mapping_mode,
            'Mem/InitWMWithAllNodes': localization_val,
            'Grid/Sensor': '1',
            'Grid/3D': 'false',
            'Grid/FromDepth': 'true',
            'Grid/RangeMax': '4.0',
            'Grid/CellSize': '0.05',
            'RGBD/NeighborLinkRefining': 'true',
        }],
        remappings=remappings,
    )

    return [
        robot_state_publisher_node,
        rtabmap_node
    ]


def generate_launch_description():
    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Set to true to run in localization-only mode'
    )

    return LaunchDescription([
        localization_arg,
        OpaqueFunction(function=launch_setup)
    ])