FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-c"]

# ── Apt dependencies (mirrors the ROBOTIS SBC setup guide) ──
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-argcomplete \
    python3-colcon-common-extensions \
    python3-rosdep \
    libboost-system-dev \
    libudev-dev \
    ros-humble-hls-lfcd-lds-driver \
    ros-humble-turtlebot3-msgs \
    ros-humble-dynamixel-sdk \
    ros-humble-xacro \
    ros-humble-v4l2-camera \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init 2>/dev/null || true && rosdep update

# ── Build TurtleBot3 packages from source ──
ENV WS=/turtlebot3_ws
RUN mkdir -p ${WS}/src

WORKDIR ${WS}/src
RUN git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git && \
    git clone -b humble https://github.com/ROBOTIS-GIT/coin_d4_driver.git

# Remove packages not needed on the SBC (cartographer & navigation run on Remote PC)
RUN rm -rf ${WS}/src/turtlebot3/turtlebot3_cartographer \
           ${WS}/src/turtlebot3/turtlebot3_navigation2

# Install any missing rosdep keys, then build
WORKDIR ${WS}
RUN source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true && \
    rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --parallel-workers 1

# ── Udev rules for OpenCR ──
RUN cp $(source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash && \
    ros2 pkg prefix turtlebot3_bringup)/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules \
    /etc/udev/rules.d/ 2>/dev/null || true

# ── Environment ──
ENV TURTLEBOT3_MODEL=burger
ENV LDS_MODEL=LDS-02
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ── Custom launch file (bringup + camera) ──
COPY launch/bringup_with_camera.launch.py /launch/bringup_with_camera.launch.py

# ── Entrypoint ──
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "/launch/bringup_with_camera.launch.py"]
