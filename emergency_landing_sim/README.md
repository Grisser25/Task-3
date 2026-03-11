# emergency_landing_sim

Gazebo + RViz simulation package for a Pixhawk 450-style drone carrying **6 depth sensors**:
- 5 side depth cameras at 72° spacing
- 1 bottom depth camera for landing zone scanning

## What this package gives you
- A URDF/Xacro drone model
- Six Gazebo depth sensors publishing `sensor_msgs/PointCloud2`
- A sample Gazebo world with nearby obstacles
- An RViz config already set up for the 6 point clouds

## Published topics
```text
/sensor_side_1/depth_cloud
/sensor_side_2/depth_cloud
/sensor_side_3/depth_cloud
/sensor_side_4/depth_cloud
/sensor_side_5/depth_cloud
/sensor_bottom/depth_cloud
```

## Dependencies
This package assumes ROS 2 with Gazebo Classic and `gazebo_ros` installed.

Typical packages:
```bash
sudo apt install ros-$ROS_DISTRO-gazebo-ros-pkgs \
                 ros-$ROS_DISTRO-xacro \
                 ros-$ROS_DISTRO-robot-state-publisher \
                 ros-$ROS_DISTRO-joint-state-publisher \
                 ros-$ROS_DISTRO-rviz2
```

## Build
Copy the folder into your workspace `src` directory and build:

```bash
cd ~/your_ws/src
cp -r /path/to/emergency_landing_sim .
cd ~/your_ws
colcon build --packages-select emergency_landing_sim
source install/setup.bash
```

## Run
```bash
ros2 launch emergency_landing_sim sim_6depth_gazebo.launch.py
```

## Verify topics
```bash
ros2 topic list | grep depth_cloud
ros2 topic echo /sensor_bottom/depth_cloud --once
```

## Notes
- The drone is spawned as a simple sensor platform, not a full flight dynamics model.
- This is intended to generate the six depth point clouds needed for your landing pipeline.
- If a sensor looks in the wrong direction, flip its `rpy` in `urdf/pixhawk450_6depth.urdf.xacro`.
- If you want PX4/Ignition/Harmonic integration later, this package should be treated as the perception-layer prototype.
