# Define variables
ROS_DISTRO := humble
IMAGE_NAME := orca-auv-ros2-image
CONTAINER_NAME := orca-auv-ros2-container
WORKSPACE := orca_auv_ros2_ws
DOCKER_IMAGE := ros:$(ROS_DISTRO)-ros-core

.PHONY: all build_container run_container exec_container clean flash_stm32 reset_stm32

all: build_container

build_container:
	git submodule sync orca_auv_ros2_ws/src/micro_ros_setup
	git submodule update --init orca_auv_ros2_ws/src/micro_ros_setup
	docker build -t $(IMAGE_NAME) .
	@echo "Creating and starting a new container: $(CONTAINER_NAME)"
	docker run -it $(DOCKER_IMAGE)

run_container:
	@echo "Starting existing container: $(CONTAINER_NAME)"
	docker start -ai $(CONTAINER_NAME)

exec_container:
	@echo "Executing a shell inside container: $(CONTAINER_NAME)"
	docker exec -it $(CONTAINER_NAME) bash

clean:
	docker rm -f $(CONTAINER_NAME) || true
	rm -rf $(WORKSPACE)/build $(WORKSPACE)/install $(WORKSPACE)/log

flash_stm32:
	git submodule sync SAUVC2024_STM32
	git submodule update --init SAUVC2024_STM32
	st-flash --reset write SAUVC2024_STM32/build/SAUVC2022.bin 0x08000000

reset_stm32:
	st-flash reset
