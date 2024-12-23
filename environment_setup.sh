#!/bin/bash

# Following instructions from: https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/

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
        echo 'Setup complete. Starting container shell.';
        exec bash
    "
