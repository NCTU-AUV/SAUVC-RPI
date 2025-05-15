# How to Test Pressure Sensor

## Use Arduino

Use the arduino code provided by BlueRobotics: [https://github.com/bluerobotics/BlueRobotics_MS5837_Library/tree/master](https://github.com/bluerobotics/BlueRobotics_MS5837_Library/tree/master).

## Test on OrcaAUV

1. `ssh gorden@192.168.0.118` connect to the Raspberry PI.
2. Change directory to SAUVC2024-RPI `cd ~/SAUVC2024-RPI`.
3. Start the docker container `make start_container`.

To enter the docker container, after starting it, run `make enter_container` in `~/SAUVC2024-RPI` directory.

Each of the following steps are all run in the docker container in new terminal windows.

4. Start MicroROS agent so that the ROS2 on RPI can communicate
with the STM32 board: `./orca_auv_rpi_ros2_ws/scripts/start_micro_ros_agent.sh`.
5. Flash the binary to the STM32 so that it restarts: `./orca_auv_rpi_ros2_ws/scripts/flash_stm32.sh`.
6. Inspect the pressure sensor topic: `ros2 topic echo /orca_auv/pressure_sensor_depth_m`.
