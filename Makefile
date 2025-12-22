# Define variables
IMAGE_NAME := orca-auv-rpi-ros2-image
CONTAINER_NAME := orca-auv-rpi-ros2-container
WORKSPACE := orca_auv_rpi_ros2_ws
IMAGE_OWNER_NAME := dianyueguo
PWD := $(shell pwd)
COMPOSE := docker compose
# Host display/X11 wiring varies by platform; set defaults and skip mounts when missing
ifeq ($(OS),Windows_NT)
	HOST_DISPLAY ?= host.docker.internal:0.0
	XAUTH_FILE :=
else
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Darwin)
		HOST_DISPLAY ?= host.docker.internal:0
	else
		HOST_DISPLAY ?= $(DISPLAY)
	endif
	XAUTH_FILE ?= $(HOME)/.Xauthority
endif
XAUTHORITY ?= /tmp/.Xauthority
XAUTH_FLAGS := $(if $(and $(XAUTH_FILE),$(wildcard $(XAUTH_FILE))),-v $(XAUTH_FILE):/tmp/.Xauthority:ro -e XAUTHORITY=/tmp/.Xauthority,)

.PHONY: compose_up compose_down compose_build compose_shell compose_init compose_launch compose_clean update_image flash_stm32 reset_stm32

update_image:
	docker buildx build --pull --platform=linux/arm64,linux/amd64 -t $(IMAGE_OWNER_NAME)/$(IMAGE_NAME):latest . --push

compose_up:
	@echo "Starting compose stack"
	@mkdir -p $(dir $(XAUTH_FILE))
	@touch $(XAUTH_FILE)
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(COMPOSE) up -d

compose_down:
	$(COMPOSE) down

compose_build:
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(COMPOSE) build --pull

compose_shell:
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(COMPOSE) exec orca /bin/bash -lc "\
		source /opt/ros/humble/setup.bash; \
		if [ -f /root/uros_ws/install/local_setup.bash ]; then \
			source /root/uros_ws/install/local_setup.bash; \
		fi; \
		if [ -f $(WORKSPACE)/install/setup.bash ]; then \
			source $(WORKSPACE)/install/setup.bash; \
		fi; \
		exec bash"

compose_init: compose_up
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(COMPOSE) exec orca /bin/bash -lc "\
		source /opt/ros/humble/setup.bash && \
		cd $(WORKSPACE) && \
		rosdep install --from-paths src --ignore-src -y && \
		colcon build --symlink-install && \
		echo \"source $(WORKSPACE)/install/setup.bash\" >> /etc/bash.bashrc"

compose_launch: compose_up
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(COMPOSE) exec orca /bin/bash -lc "\
		cd $(WORKSPACE) && \
		source /opt/ros/humble/setup.bash && \
		source /root/uros_ws/install/local_setup.bash && \
		source install/setup.bash && \
		ros2 launch src/launch/test_launch.py"

compose_clean:
	$(COMPOSE) down -v

flash_stm32:
	git submodule sync SAUVC-STM32
	git submodule update --init SAUVC-STM32
	st-flash --reset write SAUVC-STM32/build/SAUVC2024.bin 0x08000000

reset_stm32:
	st-flash reset
