"""
vslam.launch.py
Launches RTAB-Map with the Intel RealSense D435 for Visual SLAM.

Topics consumed from camera.launch.py:
  /camera/camera/color/image_raw
  /camera/camera/color/camera_info
  /camera/camera/aligned_depth_to_color/image_raw

Topics published:
  /rtabmap/map          — 3D point cloud map
  /rtabmap/odom         — visual odometry (pose estimate)
  /map                  — 2D occupancy grid

Run camera.launch.py FIRST before this.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs):
    # Resolve the localization argument
    localization_val = LaunchConfiguration('localization').perform(context)
    
    is_localization = (localization_val.lower() == 'true')
    mapping_mode = "false" if is_localization else "true"
    
    # We use camera_link as the base frame since base_link isn't published yet
    base_frame = "camera_link"
    
    remappings = [
        ('rgb/image',       '/camera/camera/color/image_raw'),
        ('rgb/camera_info', '/camera/camera/color/camera_info'),
        ('depth/image',     '/camera/camera/aligned_depth_to_color/image_raw'),
    ]

    # 1. Visual Odometry Node
    odom_node = Node(
        package='rtabmap_odom',
        executable='rgbd_odometry',
        name='rgbd_odometry',
        output='screen',
        parameters=[{
            'frame_id': base_frame,
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'approx_sync': True,
            'approx_sync_max_interval': 0.1,
            'wait_for_transform': 0.8,       # Even more time to handle hand-held jitter
            # Robustness tuning for fast motion
            'Vis/MinInliers': '8',           # More lenient (default is 20, we had 12)
            'Vis/MaxFeatures': '1000',       # Detect more points to help matching
            'Odom/Strategy': '0',            # Frame-to-Map
        }],
        remappings=remappings,
    )

    # 2. RTAB-Map Main SLAM Node
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

            # Mapping / localization mode
            'Mem/IncrementalMemory': mapping_mode,
            'Mem/InitWMWithAllNodes': localization_val,

            # 2D occupancy grid & Loop Closure
            'Grid/Sensor': '1',                # Depth camera
            'Grid/3D': 'false',
            'Grid/FromDepth': 'true',          # Ensure grid is built from depth
            'Grid/RangeMax': '4.0',
            'Grid/CellSize': '0.05',
            'RGBD/NeighborLinkRefining': 'true', # Improves loop closure 
        }],
        remappings=remappings,
    )

    return [odom_node, rtabmap_node]


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
