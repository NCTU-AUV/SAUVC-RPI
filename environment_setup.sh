#!/bin/bash

CONTAINER_NAME="orca-auv-ros2-container"

if [ "$(docker ps -a --format '{{.Names}}' | grep -w $CONTAINER_NAME)" ]; then
  echo "Starting existing container: $CONTAINER_NAME"
  docker start -ai $CONTAINER_NAME
else
  echo "Creating and starting a new container: $CONTAINER_NAME"
  docker run -it --net=host -v /dev:/dev --privileged --name $CONTAINER_NAME ros:humble
fi
