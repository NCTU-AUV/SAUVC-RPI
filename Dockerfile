FROM ros:humble

SHELL ["/bin/bash", "-c"]

# Use --no-install-recommends to save space and avoid pulling massive UI dependencies (like qt) unless needed
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    curl \
    libasio-dev \
    stlink-tools \
    sudo \
    build-essential \
    flex \
    bison \
    python3-aiohttp \
    python3-opencv \
    python3-numpy \
    ros-humble-cv-bridge \
    ros-humble-mavros \
    && curl -fsSL https://deb.nodesource.com/setup_18.x | bash - \
    && apt-get install -y --no-install-recommends nodejs \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Build micro-ROS agent
RUN source /opt/ros/humble/setup.bash && \
    cd ~/ && \
    mkdir -p uros_ws/src && cd uros_ws && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    apt-get update && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --skip-keys "clang-tidy" && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh && \
    rm -rf build log

RUN echo "source ~/uros_ws/install/local_setup.bash" >> /etc/bash.bashrc
RUN source /opt/ros/humble/setup.bash && ros2 run mavros install_geographiclib_datasets.sh

WORKDIR /root

# Default command
CMD ["/bin/bash"]
