#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
source /turtlebot3_ws/install/setup.bash

# Diagnostic output
echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
echo "LDS_MODEL=${LDS_MODEL}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"

# Device check
echo "--- Device check ---"
[ -e /dev/ttyACM0 ] && echo "OpenCR:  /dev/ttyACM0 found" || echo "WARNING: /dev/ttyACM0 not found (OpenCR)"
[ -e /dev/ttyUSB0 ] && echo "LiDAR:   /dev/ttyUSB0 found" || echo "WARNING: /dev/ttyUSB0 not found (LiDAR)"
[ -e /dev/video0 ]  && echo "Camera:  /dev/video0 found"  || echo "WARNING: /dev/video0 not found (Camera)"
echo "--------------------"

exec "$@"
