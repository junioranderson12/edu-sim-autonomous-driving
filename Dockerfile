# Use an official ROS2 image as a base image
FROM ros:humble-ros-base

# Set environment variables to avoid warnings
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Remove existing rosdep configuration if exists, then initialize rosdep
RUN sudo rm -f /etc/ros/rosdep/sources.list.d/20-default.list \
    && sudo rosdep init \
    && sudo rosdep update

# Install additional tools for development
RUN pip3 install --upgrade setuptools

# Create a workspace directory
RUN mkdir -p /root/ros2_ws/src

# Set the working directory
WORKDIR /root/ros2_ws

# Optionally clone your project here (uncomment if needed)
# RUN git clone https://github.com/your/repo.git src/

# Build the workspace using colcon
RUN colcon build

# Set the entrypoint for the container
ENTRYPOINT ["/bin/bash"]