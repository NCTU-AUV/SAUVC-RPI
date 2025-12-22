FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y git

RUN source /opt/ros/humble/setup.bash && \
    cd ~/ && \
    mkdir uros_ws && cd uros_ws && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh

RUN echo "source ~/uros_ws/install/local_setup.bash" >> /etc/bash.bashrc


RUN apt-get install -y stlink-tools


RUN sudo apt -y install ros-humble-mavros
RUN source /opt/ros/humble/setup.bash && ros2 run mavros install_geographiclib_datasets.sh


RUN apt-get install -y ros-humble-rqt*


WORKDIR /root


CMD ["/bin/bash"]
