FROM ubuntu:22.04

SHELL ["/bin/bash", "-c"]

ENV ROS_DISTRO=humble

RUN locale  # check for UTF-8
RUN apt-get update && apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
RUN locale  # verify settings

RUN apt-get install software-properties-common -y
RUN add-apt-repository universe

RUN apt-get update && apt-get install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Set environment variable to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
        python3-flake8-docstrings \
        python3-pip \
        python3-pytest-cov \
        ros-dev-tools
# Reset DEBIAN_FRONTEND to avoid issues with interactive shells later
ENV DEBIAN_FRONTEND=dialog

RUN apt-get install -y \
        python3-flake8-blind-except \
        python3-flake8-builtins \
        python3-flake8-class-newline \
        python3-flake8-comprehensions \
        python3-flake8-deprecated \
        python3-flake8-import-order \
        python3-flake8-quotes \
        python3-pytest-repeat \
        python3-pytest-rerunfailures

RUN mkdir -p ~/ros2_humble/src && \
    cd ~/ros2_humble && \
    vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src

RUN sudo apt-get upgrade

RUN rosdep init
RUN rosdep update
RUN rosdep install --from-paths ~/ros2_humble/src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN cd ~/ros2_humble/ && \
    colcon build --symlink-install

RUN echo "source ~/ros2_humble/install/local_setup.bash" >> /etc/bash.bashrc

RUN apt-get update && apt-get install -y git

RUN source ~/ros2_humble/install/local_setup.bash && \
    cd ~/ && \
    mkdir uros_ws && cd uros_ws && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh

RUN echo "source ~/uros_ws/install/local_setup.bash" >> /etc/bash.bashrc


RUN apt-get install gcc build-essential cmake rpm libusb-1.0 libusb-1.0-0-dev libgtk-3-dev pandoc -y

RUN mkdir ~/git
WORKDIR /root/git
RUN git clone https://github.com/stlink-org/stlink.git
WORKDIR /root/git/stlink
RUN make clean
RUN make release
RUN make install
RUN ldconfig
RUN make debug
RUN make package


RUN apt install python3-rosinstall-generator

RUN mkdir -p /root/mavros_ws/src
WORKDIR /root/mavros_ws

RUN rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos
RUN rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos

RUN vcs import src < /tmp/mavlink.repos
RUN vcs import src < /tmp/mavros.repos

WORKDIR /root/mavros_ws/src
RUN git clone https://github.com/ros/diagnostics.git -b 4.0.3
RUN git clone https://github.com/ros-geographic-info/geographic_info.git -b 1.0.6
RUN git clone https://github.com/ros/eigen_stl_containers.git -b 1.1.0
RUN git clone https://github.com/ros/angles.git -b 1.15.0

WORKDIR /root/mavros_ws/
RUN apt-get update
RUN source ~/ros2_humble/install/setup.bash && rosdep install --from-paths src --ignore-src -y

RUN ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh

RUN source ~/ros2_humble/install/setup.bash && colcon build

RUN echo "source ~/mavros_ws/install/setup.bash" >> /etc/bash.bashrc


WORKDIR /root


CMD ["/bin/bash"]
