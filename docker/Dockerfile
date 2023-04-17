# Build arguments
ARG ROS_DISTRO=humble

FROM microros/base:${ROS_DISTRO} AS micro-ros-agent-builder

# Set up the PX-4 communication
RUN mkdir -p ~/px4_ros_com_ros2/src 
RUN git clone https://github.com/PX4/px4_ros_com ~/px4_ros_com_ros2/src/px4_ros_com
RUN git clone https://github.com/PX4/px4_msgs.git ~/px4_ros_com_ros2/src/px4_msgs
RUN cd ~/px4_ros_com_ros2  && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

WORKDIR /uros_ws
RUN . /opt/ros/$ROS_DISTRO/setup.sh \
&&  . install/local_setup.sh \
&&  apt update \
&&  ros2 run micro_ros_setup create_agent_ws.sh \
&&  ros2 run micro_ros_setup build_agent.sh \
&&  rm -rf log/ build/ src/

# Disable shared memory
COPY disable_fastdds_shm.xml disable_fastdds_shm_localhost_only.xml /tmp/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV MICROROS_DISABLE_SHM=1

RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo ". /uros_ws/install/setup.bash" >> ~/.bashrc

# Add the entrypoint script
ADD ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/bin/sh", "/entrypoint.sh" ]
CMD ["--help"]


