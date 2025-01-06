# Define variables
IMAGE_NAME := orca-auv-ros2-image
CONTAINER_NAME := orca-auv-ros2-container
WORKSPACE := orca_auv_ros2_ws
IMAGE_OWNER_NAME := dianyueguo

.PHONY: all build_container run_container start_container update_image clean flash_stm32 reset_stm32

all: build_container

build_container:
	@echo "Creating and starting a new container: $(CONTAINER_NAME)"
	docker run -dit \
	    --net=host \
	    -v /dev:/dev \
	    -v $(PWD)/$(WORKSPACE):/$(WORKSPACE) \
	    --privileged \
	    --name $(CONTAINER_NAME) \
	    $(IMAGE_OWNER_NAME)/$(IMAGE_NAME):latest

start_container:
	@echo "Starting existing container: $(CONTAINER_NAME)"
	docker start $(CONTAINER_NAME)

exec_container:
	@echo "Executing a shell inside container: $(CONTAINER_NAME)"
	docker exec -it $(CONTAINER_NAME) bash

update_image:
	docker buildx build --platform=linux/arm64,linux/arm/v7 -t $(IMAGE_OWNER_NAME)/$(IMAGE_NAME):latest . --push

clean:
	docker rm -f $(CONTAINER_NAME) || true
	rm -rf $(WORKSPACE)/build $(WORKSPACE)/install $(WORKSPACE)/log

flash_stm32:
	git submodule sync SAUVC2024_STM32
	git submodule update --init SAUVC2024_STM32
	st-flash --reset write SAUVC2024_STM32/build/SAUVC2022.bin 0x08000000

reset_stm32:
	st-flash reset
