FROM ubuntu:22.04

RUN locale  # check for UTF-8 && \
    \
    apt update && apt install locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

ENV LANG=en_US.UTF-8

RUN locale  # verify settings

RUN apt update && \
    apt install -y software-properties-common && \
    add-apt-repository universe

RUN apt update && apt install curl -y && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update

RUN apt upgrade

# Set environment variable to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
RUN apt install -y ros-humble-ros-base
# Reset DEBIAN_FRONTEND to avoid issues with interactive shells later
ENV DEBIAN_FRONTEND=dialog

RUN apt install -y ros-dev-tools

RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc
