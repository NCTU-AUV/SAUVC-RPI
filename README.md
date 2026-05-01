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

1. `make init` (installs deps and builds the workspace; first time or after source changes)
1. `make launch` to run `ros2 launch src/launch/orca_bringup.launch.py`
1. Or run `make` to do both in sequence.
1. Open GUI in your browser: [http://raspberrypi.local](http://raspberrypi.local) (or [http://localhost](http://localhost) if you are running on your local machine)
