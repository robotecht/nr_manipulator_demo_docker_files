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


if [ "$RUN_MODE" = "real" ]; then
    echo "Running with real hardware..."
    # Call your real hardware launch command
    # exec ./launch_real_hardware.sh
elif [ "$RUN_MODE" = "sim" ]; then
    echo "Running simulation..."
    # Call your simulation launch command
    # exec ./launch_simulation.sh