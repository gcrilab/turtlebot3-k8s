#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Diagnostic output
echo "=== Controller Environment ==="
echo "TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "ROS_DISTRO=${ROS_DISTRO}"
echo "==============================="

exec "$@"
