# Use the official ROS Humble desktop image as the base image
FROM osrf/ros:humble-desktop-full

ENV ROS_DISTRO=humble

COPY /src/NRManipultaorSetup /root/ros2_ws/src/

# Set the working directory inside the container
WORKDIR /root/ros2_ws/src/

# Update apt-get and install dependencies for rosdep

# RUN apt-get update && \
#     apt-get install -y \
#     git \
#     && rm -rf /etc/ros/rosdep/sources.list.d/20-default.list

# # RUN git submodule update --init --recursive
# # Initialize rosdep and update it
# RUN rosdep init
# RUN rosdep update 
# RUN rosdep install --from-paths /root/ros2_ws/src/NRManipultaorSetup/src/xarm_ros2_NR --ignore-src --rosdistro $ROS_DISTRO -y


