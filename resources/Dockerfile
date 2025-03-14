# Base image: Ubuntu 22.04
FROM ubuntu:22.04

# Metadata
LABEL author="Boris Benedikter"
LABEL version="1.0"
LABEL description="This is a clean Ubuntu 22.04 image with ROS 2 Iron installed."

# Locale
RUN apt-get update && \
    apt-get install -y locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Enable Universe APT Repository
RUN apt-get update && \
    apt-get install -y software-properties-common && \
    add-apt-repository universe

# Install CURL
RUN apt-get update && \
    apt-get install -y curl

# Download ROS key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS repository
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Desktop (without prompts)
RUN export DEBIAN_FRONTEND=noninteractive && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y ros-iron-desktop

# Create user
RUN useradd -m -s /bin/bash rosuser && \
    echo "rosuser:rosuser" | chpasswd && \
    adduser rosuser sudo

# Set the default user
USER rosuser
WORKDIR /home/rosuser
