# Task-3
Emergency autonomous landing in unknown environment: Scan the environment beneath the drone to look for safe landing spot and land. This will be executed autonomously in case the battery of the drone low, drone sensors malfunction or loses signals.
To rebuild

cd ~/drone_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select emergency_landing_sim --symlink-install
source install/setup.bash

Terminal 1 – Launch Rviz and Gazebo

cd ~/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch emergency_landing_sim sim_6depth_gazebo.launch.py

Terminal 2 – Start the Bridge

cd ~/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run emergency_landing_sim gazebo_cmdvel_bridge

Terminal 3 – Run the Landing Node

cd ~/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run emergency_landing_sim emergency_landing_node

Terminal 4 – Trigger the Landing Node (choose 1 scenario)

/battery_percent

cd ~/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub /battery_percent std_msgs/msg/Float32 "{data: 10.0}" -r 2

/signal_ok

cd ~/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub /signal_ok std_msgs/msg/Bool "{data: false}"

/sensor_ok

cd ~/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub /sensor_ok std_msgs/msg/Bool "{data: false}"

Terminal 5 – Monitor the system

cd ~/drone_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /emergency_landing_status
