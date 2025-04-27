# Start from the official Ubuntu 24.04 (Noble) base image
FROM ubuntu:24.04

# Set non-interactive mode for apt to avoid prompts during install
ENV DEBIAN_FRONTEND=noninteractive

# Update package list and install basic dependencies
RUN apt-get update && apt-get install -y \
    locales \
    lsb-release \
    wget \
    curl \
    gnupg2 \
    build-essential \        # For C++ development
    python3-colcon-common-extensions \ # For building ROS2 workspaces
    python3-pip \             # Python package manager
    python3-rosdep \          # Dependency manager for ROS
    python3-argcomplete \     # Auto-completion for ROS2 CLI
    git \                     # Version control tool
    && rm -rf /var/lib/apt/lists/*

# Generate and configure locale to support UTF-8
RUN locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LC_ALL en_US.UTF-8

# Add the ROS2 repository to the apt sources list
RUN echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
    | tee /etc/apt/sources.list.d/ros2.list

# Import the ROS2 GPG key
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
    gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

# Install ROS2 Jazzy Desktop version
RUN apt-get update && apt-get install -y \
    ros-jazzy-desktop \
    && rm -rf /var/lib/apt/lists/*

# Set up ROS2 environment variables automatically on container startup
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
ENV ROS_DISTRO jazzy

# Initialize rosdep (safe to ignore the error if already initialized)
RUN rosdep init || true
RUN rosdep update

# Set the working directory inside the container
WORKDIR /root/ros2_ws

# Expose a ROS2 default communication port (optional)
EXPOSE 11311

# Default command when container starts
CMD ["/bin/bash"]