# Define variables
ROS_DISTRO := humble
CONTAINER_NAME := orca-auv-ros2-container
WORKSPACE := orca_auv_ros2_ws
DOCKER_IMAGE := ros:$(ROS_DISTRO)

.PHONY: all build_container run_container exec_container clean flash_stm32 reset_stm32

all: build_container

build_container:
	git submodule sync orca_auv_ros2_ws/src/micro_ros_setup
	git submodule update --init orca_auv_ros2_ws/src/micro_ros_setup
	@echo "Creating and starting a new container: $(CONTAINER_NAME)"
	docker run -dit \
	    --net=host \
	    -v /dev:/dev \
	    -v $(PWD)/$(WORKSPACE):/$(WORKSPACE) \
	    --privileged \
	    --name $(CONTAINER_NAME) \
	    $(DOCKER_IMAGE) \
	    bash
	@echo "Initializing container: $(CONTAINER_NAME)"
	docker exec -it $(CONTAINER_NAME) bash -c "\
	    source /opt/ros/$(ROS_DISTRO)/setup.bash && \
	    cd /$(WORKSPACE) && \
	    echo 'Updating dependencies...' && \
	    sudo apt update && rosdep update && \
	    rosdep install --from-paths src --ignore-src -y && \
	    echo 'Installing pip...' && \
	    sudo apt-get install -y python3-pip && \
	    echo 'Building micro-ROS tools...' && \
	    colcon build && \
	    echo 'Sourcing setup...' && \
	    source install/local_setup.bash && \
	    ros2 run micro_ros_setup create_agent_ws.sh && \
	    ros2 run micro_ros_setup build_agent.sh && \
	    source install/local_setup.sh && \
	    echo 'Initialization complete.'"

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
