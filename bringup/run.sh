#!/usr/bin/env bash
# =============================================================
# Run TurtleBot3 Burger bringup container on the Raspberry Pi 4
# =============================================================
#
# Prerequisites (on the RPi4 host, one-time setup):
#   1. Install Docker:          curl -fsSL https://get.docker.com | sh
#   2. Add user to docker group: sudo usermod -aG docker $USER
#   3. Install udev rules for OpenCR:
#        echo 'ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", ENV{ID_MM_DEVICE_IGNORE}="1", MODE:="0666", GROUP:="dialout", SYMLINK+="opencr"' \
#          | sudo tee /etc/udev/rules.d/99-opencr-cdc.rules
#        sudo udevadm control --reload-rules && sudo udevadm trigger
#   4. Build the image:          docker build -t turtlebot3-burger-bringup .
#
# Usage:
#   bash run.sh          # start the container
#   bash run.sh stop     # stop and remove the container

IMAGE_NAME="gcrilab/turtlebot3-burger-bringup-3"
CONTAINER_NAME="tb3-burger"

# ---------- stop / cleanup ----------
if [ "$1" = "stop" ]; then
    echo "Stopping container ${CONTAINER_NAME}..."
    docker stop "${CONTAINER_NAME}" 2>/dev/null
    docker rm "${CONTAINER_NAME}" 2>/dev/null
    exit 0
fi

# Remove stale container if it exists
docker rm -f "${CONTAINER_NAME}" 2>/dev/null

# ---------- run ----------
docker run -it \
    --name "${CONTAINER_NAME}" \
    --privileged \
    --network=host \
    -e TURTLEBOT3_MODEL=burger \
    -e LDS_MODEL=LDS-02 \
    -e ROS_DOMAIN_ID=0 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    --device=/dev/ttyACM0 \
    --device=/dev/ttyUSB0 \
    --device=/dev/video0 \
    "${IMAGE_NAME}"
