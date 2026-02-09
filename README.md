# SAUVC-RPI

This repository should contain all the codes that need to be run on the Raspberry PI of the Orca-AUV.

## Connect to Raspberry Pi

```shell
ssh pi@raspberrypi.local
```

## Download

You start from here if you want to run on your local machine.

```shell
git clone https://github.com/NCTU-AUV/SAUVC-RPI.git
cd SAUVC-RPI
git submodule update --init --recursive
```

## Quick Start

1. `make compose_init` (installs deps and builds the workspace; first time or after source changes)
1. `make compose_launch` to run `ros2 launch src/launch/test_launch.py`
1. Open GUI in your browser: [http://raspberrypi.local](http://raspberrypi.local)
