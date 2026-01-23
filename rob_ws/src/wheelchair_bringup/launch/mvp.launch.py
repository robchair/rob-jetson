import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    mux_yaml = PathJoinSubstitution([
        FindPackageShare("wheelchair_bringup"),
        "config",
        "twist_mux.yaml"
    ])

    venv_python = "/home/rob/rob/roboVoice/venv/bin/python3"
    voice_module = "robovoice_ros2.voice_cmd_node"  # must exist as module

    return LaunchDescription([
        # 1) Voice -> /cmd_vel_voice (run with venv python explicitly)
        ExecuteProcess(
            cmd=[
                venv_python, "-m", voice_module,
                "--ros-args",
                "-r", "__node:=voice_cmd_node",
                "-p", "linear_speed:=0.30",
                "-p", "angular_speed:=0.80",
            ],
            output="screen",
        ),

        # 2) Twist mux -> /cmd_vel_raw
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[mux_yaml],
            remappings=[
                ("/cmd_vel_out", "/cmd_vel_raw"),
            ],
        ),
        # 3) Safety gate -> /cmd_vel_safe
        Node(
            package="wheelchair_bringup",
            executable="safety_gate_node",
            name="safety_gate",
            output="screen",
        ),

        # 4) Base driver -> Arduino consumes /cmd_vel_safe
        Node(
            package="arduino_base_driver",
            executable="arduino_base_driver_node",
            name="arduino_base_driver",
            output="screen",
            parameters=[
                {"port": "/dev/ttyACM0"},
                {"baud": 9600},
                {"cmd_timeout_sec": 0.5},
            ],
        ),
    ])

