# syntax=docker/dockerfile:1.6

# =======================================================
# 1. 定義基底映像檔
# =======================================================
ARG BASE_PC=osrf/ros:noetic-desktop
ARG BASE_AGX=nycusystemlab/l4t-noetic:latest

# =======================================================
# 2. 自動選擇基底
# =======================================================
FROM ${BASE_PC} AS base-amd64
ENV ROS_DISTRO=noetic

FROM ${BASE_AGX} AS base-arm64
ENV ROS_DISTRO=noetic
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra

FROM base-${TARGETARCH} AS final

ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=/root

# =======================================================
# 3. 統一安裝依賴
# =======================================================
RUN apt-get update && apt-get install -y \
    build-essential cmake git python3-catkin-tools python3-pip wget \
    ros-${ROS_DISTRO}-pcl-ros libpcl-dev \
    ros-${ROS_DISTRO}-tf2-eigen ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-diagnostic-updater ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-ddynamic-reconfigure ros-${ROS_DISTRO}-rgbd-launch \
    ros-${ROS_DISTRO}-rosserial-python \
    ros-${ROS_DISTRO}-imu-tools \
    libssl-dev libusb-1.0-0-dev libudev-dev pkg-config libpcap-dev \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# =======================================================
# 4. 安裝 GTSAM
# =======================================================
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt-get update && \
    apt-get install -y libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

# =======================================================
# 5. 編譯 Librealsense
# =======================================================
RUN git clone -b v2.48.0 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && mkdir build && cd build && \
    cmake ../ -DCMAKE_BUILD_TYPE=Release -DBUILD_NETWORK_DEVICE=OFF -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DFORCE_RSUSB_BACKEND=ON && \
    make -j$(nproc) && make install && rm -rf /tmp/librealsense

# =======================================================
# 6. 複製程式碼並「預先編譯」 (Production Ready)
# =======================================================
WORKDIR ${WORKSPACE}

# (A) HDL Workspace
COPY ./src/hdl_ws/src ${WORKSPACE}/hdl_ws/src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                  cd ${WORKSPACE}/hdl_ws && catkin_make"

# (B) LiDAR Workspace (isolated)
COPY ./src/lidar_ws/src ${WORKSPACE}/lidar_ws/src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                  cd ${WORKSPACE}/lidar_ws && catkin_make_isolated"

# (C) Realsense Workspace
COPY ./src/realsense_ws/src ${WORKSPACE}/realsense_ws/src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                  cd ${WORKSPACE}/realsense_ws && catkin_make"

# (D) Keyboard Control Workspace
COPY ./src/keyboard_control_ws/src ${WORKSPACE}/keyboard_control_ws/src
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                  cd ${WORKSPACE}/keyboard_control_ws && catkin_make"

# =======================================================
# 7. 設定 Entrypoint
# =======================================================
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

WORKDIR /root
ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]