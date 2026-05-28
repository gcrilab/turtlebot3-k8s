#!/bin/bash
# Run the TurtleBot3 Figure-8 controller container
#
# Usage:
#   ./run.sh                              # defaults
#   ./run.sh --ros-args -p v_max:=0.15    # pass ROS 2 params
#
# Set ROS_DOMAIN_ID before running if needed:
#   export ROS_DOMAIN_ID=0

IMAGE_NAME="gcrilab/turtlebot3-figure8:latest"
LOG_DIR="$(pwd)/logs"

mkdir -p "$LOG_DIR"

docker run --rm \
    --network=host \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e TURTLEBOT3_MODEL=burger \
    -v "$LOG_DIR":/app/logs \
    "$IMAGE_NAME" \
    python3 /app/enhanced_fdb_eight.py "$@"
