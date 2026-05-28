FROM ros:humble-ros-base-jammy

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install Python dependencies and TurtleBot3 messages
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-humble-turtlebot3-msgs \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir numpy pandas psutil

# Set TurtleBot3 model
ENV TURTLEBOT3_MODEL=burger
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Copy the controller script
WORKDIR /app
COPY enhanced_fdb_eight.py .

# Create output directory for CSV logs
RUN mkdir -p /app/logs

WORKDIR /app/logs

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["python3", "/app/enhanced_fdb_eight.py"]
