# SAUVC-RPI

This repository should contain all the codes that need to be run on the Raspberry PI of the Orca-AUV.

## use-ros2

This branch is intended to be developed alongside [SAUVC-STM32/use-micro-ros](https://github.com/NCTU-AUV/SAUVC-STM32/tree/use-micro-ros).

## Quick Start (docker compose)

1. `ssh orca@raspberrypi.local`
2. `cd SAUVC-RPI`
3. `make compose_up`
4. `make compose_init` (installs deps and builds the workspace; first time or after source changes)
5. `make compose_launch` to run `ros2 launch src/launch/test_launch.py`
6. Open GUI in your browser: [http://raspberrypi.local](http://raspberrypi.local)

## Test Pressure Sensor

See [How to Test Pressure Sensor](./documentation/how_to_test_pressure_sensor.md).
