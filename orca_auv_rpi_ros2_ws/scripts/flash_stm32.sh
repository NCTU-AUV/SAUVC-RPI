#!/bin/bash

st-flash --reset write /root/orca_auv_rpi_ros2_ws/stm32_binary/ORCA-STM32.bin 0x08000000
