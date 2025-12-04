# syntax=docker/dockerfile:1.6

# 繼承自您提供的 Base Image
FROM nycusystemlab/l4t-noetic:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV WORKSPACE=/root
# *** 修正：確保 ROS_DISTRO 變數被定義 ***
ENV ROS_DISTRO=noetic

# -------------------------------------------------
# 1. 安裝系統與 ROS 依賴 (包含 ddynamic_reconfigure, rgbd_launch)
# -------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-pcl-ros \
    libpcl-dev \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-message-filters \
    ros-${ROS_DISTRO}-diagnostic-updater \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-ddynamic-reconfigure \
    ros-${ROS_DISTRO}-rgbd-launch \
    ros-${ROS_DISTRO}-rosserial-python \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config \
    libpcap-dev \
    && rm -rf /var/lib/apt/lists/*

# -------------------------------------------------
# 2. 安裝 GTSAM
# -------------------------------------------------
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt-get update && \
    apt-get install -y libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

# -------------------------------------------------
# 3. 手動編譯 Librealsense v2.48.0 (解決 fastrtps/USB 相容性)
# -------------------------------------------------
# 這是為了確保 Librealsense SDK 的版本和設定與舊 ROS 專案 (realsense-ros 2.3.1) 匹配
RUN git clone -b v2.48.0 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && \
    mkdir build && cd build && \
    cmake ../ -DCMAKE_BUILD_TYPE=Release \
              -DBUILD_NETWORK_DEVICE=OFF \
              -DBUILD_EXAMPLES=OFF \
              -DBUILD_GRAPHICAL_EXAMPLES=OFF \
              -DFORCE_RSUSB_BACKEND=ON && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/librealsense

# -------------------------------------------------
# 4. 建立工作目錄 (包含 realsense_ws)
# -------------------------------------------------
RUN mkdir -p ${WORKSPACE}/hdl_ws/src \
             ${WORKSPACE}/lidar_ws/src \
             ${WORKSPACE}/realsense_ws/src \
             ${WORKSPACE}/keyboard_control_ws/src

# -------------------------------------------------
# 5. 設定 Entrypoint
# -------------------------------------------------
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

WORKDIR /root
ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]
