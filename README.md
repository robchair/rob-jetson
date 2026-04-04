# rob-jetson software package on the Jetson Orin Nano

To get all the contents of submodules, if this 'rob-jetson' repo was just cloned, run:
```bash
cd ~/
git clone git@github.com:robchair/rob-jetson.git
cd rob-jetson
git submodule update --init --recursive
```
* No git submodules as of current

# To run voice control but with ROS2, do the following:

```bash
source ~/rob/roboVoice/venv/bin/activate
cd ~/rob/rob_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch wheelchair_bringup mvp.launch.py # Only need to run this one if no changes were made since last build

```

# To activate keyboard control:

```bash
# In another terminal while the mvp.launch.py is running with ROS2 run:
source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash
source ~/rob/roboVoice/venv/bin/activate

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cmd_vel_keyboard
```
* Next step: maybe make keyboard launch automatically by editing `mvp.launch.py`?
* TODO: Make the keyboard controls have persistence until another control is given (right now motors stop after 0.5s).

# Troubleshooting when having build issues:

If ROS2 is mentioning it cant find certain launch files, despite the setup.py files in each package looking correct, run the following commands to clear the cached build data.
```bash
cd ~/rob/rob_ws
rm -rf build install log
```

If the ROS2 daemon is crashed or bugging, restart it by running:
```bash
deactivate 2>/dev/null || true

source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash

#Restart the ros2 CLI daemon
ros2 daemon stop
pkill -f ros2daemon 2>/dev/null || true
rm -rf ~/.ros/ros2cli 2>/dev/null || true
ros2 daemon start

#Try any  again
ros2 node list
```

# Terminal 1: Start camera
```bash
cd ~/rob/rob_ws
rm -rf build install log   # clean old branch build
colcon build --packages-select wheelchair_bringup
source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash

ros2 launch wheelchair_bringup camera.launch.py
```

# Terminal 2: Start encoder serial node
```bash
cd ~/rob/rob_ws
colcon build --packages-select wheelchair_localization
source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash

ros2 run wheelchair_localization encoder_serial_node \
  --ros-args \
  -p port:=/dev/ttyACM0 \
  -p baud:=115200
```

# Terminal 3: Start encoder odometry
```bash
source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash

ros2 run wheelchair_localization encoder_odom \
  --ros-args \
  -p encoder_topic:=/wheel_encoder_ticks \
  -p odom_topic:=/odom \
  -p wheel_radius_m:=0.305 \
  -p wheel_base_m:=0.515 \
  -p ticks_per_rev:=1199.67 \
  -p left_sign:=1.0 \
  -p right_sign:=-1.0
```

# Terminal 4: Start Visual SLAM (RTAB-Map)
```bash
source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash

ros2 launch wheelchair_bringup vslam.launch.py
```

# Optional: Open RViz (if not auto-launched)
```bash
rviz2
```

