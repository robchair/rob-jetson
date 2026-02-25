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


# To see raw LiDAR LaserScan output in Rviz2:

Note: Using this remository driver git clone `https://github.com/ldrobotSensorTeam/ldlidar_ros2.git`

```bash
cd ~/rob/rob_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch ldlidar_ros2 ld19.launch.py
```

In another terminal run:
```bash
source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash
ros2 run tf2_ros tf2_echo base_link base_laser
```

Then run in a new terminal to launch Rviz2:
```bash
source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash
rviz2
```

In the Rviz2 GUI:

Set Fixed Frame = base_link (or base_laser if base_link isn’t available yet)

Add -> LaserScan

Set Topic = /scan


## To try and run SLAM:

```bash
source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash
#ros2 launch slam_toolbox online_async_launch.py
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/rob/rob_ws/src/wheelchair_bringup/config/slam_toolbox.yaml
```

Then in the Rviz2 GUI, add:

- Map (/map)

- TF

- LaserScan (/scan)

