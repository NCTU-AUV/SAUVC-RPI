# SAUVC2024-RPI

This repository should contain all the codes that need to be run on the Raspberry PI of the Orca-AUV.

## use-ros2

This branch is intended to be developed alongside [SAUVC2024_STM32/use-micro-ros](https://github.com/NCTU-AUV/SAUVC2024_STM32/tree/use-micro-ros).

## Test ROS2 in docker

1. Follow: [Getting started with ROS and Docker](https://wiki.ros.org/docker/Tutorials/Docker)
2. Install ssh in the container: 
```
apt update
apt install openssh-client
```
3. Set up your access to this repository. See: [Connecting to GitHub with SSH](https://docs.github.com/en/authentication/connecting-to-github-with-ssh)
4. Clone this repository into the container: `git clone git@github.com:NCTU-AUV/SAUVC2024-RPI.git`

## Test Pressure Sensor

See [How to Test Pressure Sensor](./documentation/how_to_test_pressure_sensor.md).
