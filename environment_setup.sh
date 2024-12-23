#!/bin/bash

CONTAINER_NAME="orca-auv-ros2-container"

if [ "$(docker ps -a --format '{{.Names}}' | grep -w $CONTAINER_NAME)" ]; then
  echo "Starting existing container: $CONTAINER_NAME"
  docker start -ai $CONTAINER_NAME
else
  echo "Creating and starting a new container: $CONTAINER_NAME"
  docker run -it \
  --net=host \
  -v /dev:/dev \
  -v /$PWD/microros_ws:/microros_ws \
  --privileged \
  --name $CONTAINER_NAME \
  ros:humble \
  bash -c "
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
fi
