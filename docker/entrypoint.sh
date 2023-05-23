#!/bin/bash

. /opt/ros/${ROS_DISTRO}/setup.sh
. /px4_uros_uxrce_dds_ws/install/setup.sh

exec MicroXRCEAgent "$@"