# Build arguments
ARG ROS_DISTRO=humble

# Base image
FROM ros:${ROS_DISTRO}-ros-base

# Set up micro-dds-agent
RUN mkdir -p /px4_uros_uxrce_dds_ws/src
RUN git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /px4_uros_uxrce_dds_ws/src
RUN cd /px4_uros_uxrce_dds_ws && . /opt/ros/${ROS_DISTRO}/setup.sh && colcon build

# Source ROS2 and the local workspace
RUN echo ". /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo ". ~/px4_uros_uxrce_dds_ws/install/setup.bash" >> ~/.bashrc

# Add the entrypoint script
ADD ./entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/bin/sh", "/entrypoint.sh" ]
CMD ["--help"]


