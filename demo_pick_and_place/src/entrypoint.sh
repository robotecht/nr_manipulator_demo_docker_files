#!/bin/bash
set -e

# Source ROS 2 environment
source /opt/ros/$ROS_DISTRO/setup.bash

# Source the built workspace environment
source /root/ros2_ws/install/setup.bash

# Execute the ROS 2 launch commands
# First, launch MoveIt with a fake robot
ros2 launch xarm_moveit_config xarm5_moveit_fake.launch.py add_gripper:=true &

# Then, launch the pick-and-place demo
ros2 launch xarm_planner xarm5_pick_and_place.launch.py

# # Keep the container running (if necessary)
# exec "$@"
