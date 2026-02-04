#!/bin/bash
python -m sounddevice
echo "Ensure the device index of the microphone printed above matches the value set in roboVoice/config.yaml"
source ~/rob/roboVoice/venv/bin/activate
cd ~/rob/rob_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
#source install/setup.bash
source ~/rob/rob_ws/install/setup.bash
ros2 launch wheelchair_bringup mvp.launch.py
