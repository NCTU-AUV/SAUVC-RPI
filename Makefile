# Define variables
IMAGE_NAME := orca-auv-rpi-ros2-image
CONTAINER_NAME := orca-auv-rpi-ros2-container
WORKSPACE := orca_auv_rpi_ros2_ws
IMAGE_OWNER_NAME := dianyueguo
PWD := $(shell pwd)

.PHONY: all build_container start_container stop_container enter_container update_image clean flash_stm32 reset_stm32

all: build_container enter_container

build_container:
	@echo "Creating and starting a new container: $(CONTAINER_NAME)"
	docker run -dit \
	    --net=host \
	    -v /dev:/dev \
	    -v $(PWD)/$(WORKSPACE):/root/$(WORKSPACE) \
	    -v $(PWD)/SAUVC-STM32/build:/root/$(WORKSPACE)/stm32_binary \
		-v /home/orca/.Xauthority:/tmp/.Xauthority:ro \
		-e XAUTHORITY=/tmp/.Xauthority \
		-e DISPLAY=$(DISPLAY) \
	    --privileged \
	    --name $(CONTAINER_NAME) \
	    --pull always \
	    $(IMAGE_OWNER_NAME)/$(IMAGE_NAME):latest

	docker exec $(CONTAINER_NAME) /bin/bash -i -c "cd $(WORKSPACE) \
							&& rosdep install --from-paths src --ignore-src -y \
							&& colcon build --symlink-install \
							&& echo \"source $(WORKSPACE)/install/setup.bash\" >> /etc/bash.bashrc"

start_container:
	@echo "Starting existing container: $(CONTAINER_NAME)"
	docker start $(CONTAINER_NAME)

stop_container:
	@echo "Stopping container: $(CONTAINER_NAME)"
	docker stop $(CONTAINER_NAME)

enter_container:
	@echo "Executing a shell inside container: $(CONTAINER_NAME)"
	docker exec -it $(CONTAINER_NAME) /bin/bash

quick_launch: clean build_container
	docker start $(CONTAINER_NAME)
	docker exec -it $(CONTAINER_NAME) /bin/bash -ic "cd $(WORKSPACE) \
							&& ros2 launch src/launch/test_launch.py"

update_image:
	docker buildx build --pull --platform=linux/arm64,linux/amd64 -t $(IMAGE_OWNER_NAME)/$(IMAGE_NAME):latest . --push

clean:
	docker rm -f $(CONTAINER_NAME) || true
	sudo rm -rf $(WORKSPACE)/build $(WORKSPACE)/install $(WORKSPACE)/log

flash_stm32:
	git submodule sync SAUVC-STM32
	git submodule update --init SAUVC-STM32
	st-flash --reset write SAUVC-STM32/build/SAUVC2024.bin 0x08000000

reset_stm32:
	st-flash reset
