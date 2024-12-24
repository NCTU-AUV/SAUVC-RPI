# Define variables
ROS_DISTRO := humble
CONTAINER_NAME := orca-auv-ros2-container
WORKSPACE := orca_auv_ros2_ws
DOCKER_IMAGE := ros:$(ROS_DISTRO)

.PHONY: all setup build_container run_container exec_container clean

all: setup build_container

setup:
	git submodule sync
	git submodule update --init --recursive
	cd $(WORKSPACE)/src/micro_ros_setup && git checkout $(ROS_DISTRO)

build_container:
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
