#!/bin/bash
# entrypoint.sh
echo "RUN_MODE is set to: $RUN_MODE"

# Check if the RUN_MODE environment variable is set to "hardware"
if [ "$RUN_MODE" = "hardware" ]; then
    echo "Running with real hardware..."
    # Call your real hardware launch command
    source /opt/ros/$ROS_DISTRO/setup.bash
    # Source the built workspace environment
    source /root/ros2_ws/install/setup.bash
    # Execute the ROS 2 launch commands
    ros2 launch xarm_moveit_config xarm5_moveit_realmove.launch.py robot_ip:=192.168.1.198 add_gripper:=true &
    # Then, launch the pick-and-place demo
    ros2 launch xarm_planner xarm5_pick_and_place.launch.py
    # # Keep the container running (if necessary)
    # exec "$@"
else
    echo "Running simulation..."
    # Call your simulation launch command
    source /opt/ros/$ROS_DISTRO/setup.bash
    # Source the built workspace environment
    source /root/ros2_ws/install/setup.bash
    # Execute the ROS 2 launch commands
    # First, launch MoveIt with a fake robot
    ros2 launch xarm_moveit_config xarm5_moveit_fake.launch.py add_gripper:=true &
    # Then, launch the pick-and-place demo
    ros2 launch xarm_planner xarm5_pick_and_place.launch.py
    # # # Keep the container running (if necessary)
    # exec "$@"
fi
