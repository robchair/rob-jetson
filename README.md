# rob-jetson software package on the Jetson Orin Nano

To get all the contents of submodules, if this 'rob-jetson' repo was just cloned, run:
```bash
cd ~/
git clone git@github.com:robchair/rob-jetson.git
cd rob-jetson
git submodule update --init --recursive
```
* No git submodules as of current

# To run base ROS2 control model, do the following:

```bash
source ~/rob/roboVoice/venv/bin/activate
cd ~/rob/rob_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch wheelchair_bringup mvp.launch.py # Only need to run this one if no changes were made since last build

```

# To activate voice control 

```bash
source /home/rob/rob/roboVoice/venv/bin/activate

export PYTHONPATH=/home/rob/rob:$PYTHONPATH

python3 -m robovoice_ros2.voice_cmd_node \
  --ros-args \
  -r __node:=voice_cmd_node \
  -p linear_speed:=0.30 \
  -p angular_speed:=0.80
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

# To launch EEG-headset control node

```bash
cd ~/rob
source ~/rob/rob_ws/install/setup.bash
/home/rob/rob/roboVoice/venv/bin/python3 eeg_cmd_node.py
```

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


# To run SLAM:

Note: Using this remository driver git clone `https://github.com/ldrobotSensorTeam/ldlidar_ros2.git`

```bash
cd ~/rob/rob_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ldlidar_ros2 ld19.launch.py

# Then run this to test, should see ~6Hz:
ros2 topic hz /scan
```

In a new terminal, start Arduino Mega driver node:
```bash
source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash
ros2 run arduino_base_driver arduino_base_driver_node   --ros-args   -p port:=/dev/ttyACM0   -p baud:=115200
```

Then bring up odom node:

```bash
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

x y z: the sensor’s position relative to base_link, in meters
roll pitch yaw: the sensor’s orientation relative to base_link, in radian

Launch static TF for ultrasonic:

```bash
ros2 run tf2_ros static_transform_publisher \
  --x 0.25 --y 0.00 --z 0.10 \
  --roll 0.0 --pitch 0.0 --yaw 0.0 \
  --frame-id base_link \
  --child-frame-id ultrasonic_link
  ```

Launch SLAM toolbox:


```bash
source /opt/ros/humble/setup.bash
source ~/rob/rob_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=/home/rob/rob/rob_ws/src/wheelchair_bringup/config/slam_toolbox.yaml
```

Launch the following to verify live TF:

```bash
ros2 run tf2_tools view_frames
```


Ensure the definition in `/home/rob/rob/rob_ws/src/ldlidar_ros2/launch/ld19.launch.py` contains these parameters:

```python
def generate_launch_description():
  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar_ros2',
      executable='ldlidar_ros2_node',
      name='ldlidar_publisher_ld19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'laser_scan_topic_name': 'scan'},
        {'point_cloud_2d_topic_name': 'pointcloud2d'},
        {'frame_id': 'base_laser'},
        {'port_name': '/dev/ttyUSB0'},
        {'serial_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': True},
        {'angle_crop_min': 0.0},  # unit is degress
        {'angle_crop_max': 95.0},  # unit is degress
        {'range_min': 0.02}, # unit is meter
        {'range_max': 12.0}   # unit is meter
      ]
  )

  # base_link to base_laser tf node
  base_link_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_link_to_base_laser_ld19',
    #arguments=['0','0','0.18','0','0','0','base_link','base_laser']
    arguments=['0.32','-0.2325','0.82','0','0','3.14159','base_link','base_laser'] # LiDAR is currently pointed backwards, this is the base_link -> base_laser TF
  )
```


# How to run the EEG headest

First, check to see if the Muse headset is connected and what the MAC address is:
```bash
muselst list
```
# Terminal 1: Connect headset
```bash
muselsl stream --address 00:55:DA:B834:01
```

# Terminal 2: Connect headset
```bash
python3 ~/rob/eeg_cmd_node.py
```

