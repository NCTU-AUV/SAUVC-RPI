#!/bin/bash

git submodule update --init --recursive

# Following instructions from: https://github.com/micro-ROS/micro_ros_setup

CONTAINER_NAME="orca-auv-ros2-container"

echo "Creating and starting a new container: $CONTAINER_NAME"
docker run -it \
    --net=host \
    -v /dev:/dev \
    -v /$PWD/microros_ws:/microros_ws \
    --privileged \
    --name $CONTAINER_NAME \
    ros:humble \
    bash -c "
        source /opt/ros/\$ROS_DISTRO/setup.bash && \
        cd microros_ws && \
        echo 'Updating dependencies...';
        sudo apt update && rosdep update && \
        rosdep install --from-paths src --ignore-src -y && \
        echo 'Installing pip...' && \
        sudo apt-get install -y python3-pip && \
        echo 'Building micro-ROS tools...' && \
        colcon build && \
        echo 'Sourcing setup...' && \
        source install/local_setup.bash && \
        ros2 run micro_ros_setup create_agent_ws.sh && \
        ros2 run micro_ros_setup build_agent.sh && \
        source install/local_setup.sh && \
        echo 'Setup complete. Starting container shell.';
        exec bash
    "
