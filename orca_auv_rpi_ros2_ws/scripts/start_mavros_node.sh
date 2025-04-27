#!/bin/bash

ros2 run mavros mavros_node --ros-args -p fcu_url:=serial:///dev/ttyACM0:2000000
