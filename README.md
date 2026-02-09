# SAUVC-RPI

This repository should contain all the codes that need to be run on the Raspberry PI of the Orca-AUV.

## Quick Start (docker compose)

1. `ssh orca@raspberrypi.local`
2. `cd SAUVC-RPI`
3. `make compose_up`
4. `make compose_init` (installs deps and builds the workspace; first time or after source changes)
5. `make compose_launch` to run `ros2 launch src/launch/test_launch.py`
6. Open GUI in your browser: [http://raspberrypi.local](http://raspberrypi.local)
