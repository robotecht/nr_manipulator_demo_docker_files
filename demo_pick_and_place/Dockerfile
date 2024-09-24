# Use the official ROS Humble desktop image as the base image
FROM osrf/ros:humble-desktop-full

ENTRYPOINT [ "/src/entrypoint.sh"]

ENV ROS_DISTRO=humble

COPY /src /root/ros2_ws/src/

# Set the working directory inside the container
WORKDIR /root/ros2_ws/src/nr_manipulator_software_files/src/xarm_ros2_NR/

# Update apt-get and install dependencies for rosdep

RUN apt-get update && \
    apt-get install -y \
    git \
    && rm -rf /etc/ros/rosdep/sources.list.d/20-default.list

# RUN git submodule update --init --recursive
# Initialize rosdep and update it
RUN rosdep init && rosdep update && rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

WORKDIR /root/ros2_ws/

RUN cd /root/ros2_ws \
    && . /opt/ros/humble/setup.sh \
    && colcon build


