"""
camera.launch.py
Launches the Intel RealSense D435 camera node.
Publishes:
  /camera/color/image_raw
  /camera/depth/image_rect_raw
  /camera/aligned_depth_to_color/image_raw
  /camera/color/camera_info
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            output='screen',
            parameters=[{
                # Enabled streams
                'enable_color': True,
                'enable_depth': True,
                'enable_infra1': False,
                'enable_infra2': False,
                # Align depth to color frame
                'align_depth.enable': True,
                # Resolution and FPS — 10fps is a good "sweet spot" for VSLAM on Jetson
                'rgb_camera.profile': '640,480,30',
                'depth_module.profile': '640,480,30',
                # Publish TF for the camera optical frames
                'publish_tf': True,
                'tf_publish_rate': 0.0,  # static TF only
                # Camera serial number — leave blank to use first detected
                'serial_no': '',
            }],
            env={
                **os.environ,
                'LD_LIBRARY_PATH': '/opt/ros/humble/lib/aarch64-linux-gnu:' + os.environ.get('LD_LIBRARY_PATH', '')
            }
        ),
    ])
