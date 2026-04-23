import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import TimerAction

def generate_launch_description():
    mux_yaml = PathJoinSubstitution([
        FindPackageShare("wheelchair_bringup"),
        "config",
        "twist_mux.yaml",
    
    ])


    venv_python = "/home/rob/rob/roboVoice/venv/bin/python3"
    voice_module = "robovoice_ros2.voice_cmd_node"
    eeg_script = "/home/rob/rob/eeg_cmd_node.py"

    return LaunchDescription([
        # 1) Voice -> /cmd_vel_voice
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
#        # 2) Keyboard teleop -> /cmd_vel_keyboard
#        ExecuteProcess(
#            cmd=[
#                "ros2", "run", "teleop_twist_keyboard", "teleop_twist_keyboard",
#                "--ros-args", "-r", "cmd_vel:=/cmd_vel_keyboard",
#            ],
#            output="screen",
#        ),
        ExecuteProcess(
            cmd=["muselsl", "stream", "--acc"],
            output="screen",
        ),
        # 3) EEG -> /cmd_vel_eeg
        TimerAction(
            period=10.0,
            actions=[
                ExecuteProcess(
                    cmd=[venv_python, eeg_script],
                    output="screen",
                ),
            ]
        ),
        # 4) Twist mux -> /cmd_vel_raw
        Node(
            package="twist_mux",
            executable="twist_mux",
            name="twist_mux",
            output="screen",
            parameters=[mux_yaml],
            remappings=[("/cmd_vel_out", "/cmd_vel_raw")],
        ),
        # 5) Safety gate -> /cmd_vel_safe
        Node(
            package="wheelchair_bringup",
            executable="safety_gate_node",
            name="safety_gate",
            output="screen",
        )


    ])