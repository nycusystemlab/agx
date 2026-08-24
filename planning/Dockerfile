# syntax=docker/dockerfile:1.6

# ===========================================================================
#  Planning Container — Nav2 / SLAM / EKF / RealSense / 車控
# ===========================================================================

# ── 1. 基底映像檔 ──────────────────────────────────────────────────────────
ARG BASE_PC=osrf/ros:humble-desktop
ARG BASE_AGX=nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c

# ── 2. 架構分流 ────────────────────────────────────────────────────────────

# [PC 端]
FROM ${BASE_PC} AS base-amd64
RUN apt-get update && apt-get install -y python3-pip git && \
    pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu

# [AGX 端]
FROM ${BASE_AGX} AS base-arm64
USER root
RUN apt-get update && apt-get install -y python3-pip git vim && rm -rf /var/lib/apt/lists/*
RUN pip3 install -U jetson-stats

# ── 3. 應用層依賴 ─────────────────────────────────────────────────────────
ARG TARGETARCH
FROM base-${TARGETARCH} AS final

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/root/ros2_ws

# 加入第三方 APT Repository
RUN curl -fsSL https://dl.yarnpkg.com/debian/pubkey.gpg \
    | gpg --dearmor -o /usr/share/keyrings/yarn.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/yarn.gpg] https://dl.yarnpkg.com/debian stable main" \
    > /etc/apt/sources.list.d/yarn.list

# ROS 2 套件 + 系統工具 (單次 apt-get，減少 layer)
RUN apt-get update && \
    apt-get install -y -o Dpkg::Options::="--force-overwrite" \
    # ── Nav2 / SLAM / Localization ──
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-laser-filters \
    # ── RealSense D455 (ROS wrapper only, library will be built from source) ──
    ros-${ROS_DISTRO}-realsense2-camera \
    ros-${ROS_DISTRO}-diagnostic-updater \
    # ── Robot Model / TF ──
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    # ── Middleware / Comm ──
    # 兩套 rmw 並存，切換靠 RMW_IMPLEMENTATION，不需重建映像
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rmw-zenoh-cpp \
    # ── Utilities ──
    ros-${ROS_DISTRO}-tf-transformations \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-twist-mux \
    nlohmann-json3-dev \
    python3-vcstool \
    usbutils \
    git-lfs \
    tmux \
    udev \
    # ── librealsense2 source build 依賴 ──
    libusb-1.0-0-dev \
    libssl-dev \
    cmake \
    pkg-config \
    python3-dev \
    pybind11-dev \
    && apt-get remove -y ros-humble-compressed-depth-image-transport \
    && rm -rf /var/lib/apt/lists/*

# ── 3b. 從原始碼編譯 librealsense2 (RSUSB 後端，解決 Jetson 無 HID 核心模組) ──
# Jetson Tegra 核心缺少 hid_sensor_custom 模組，apt 版 librealsense 用 V4L2/IIO 後端無法存取 IMU。
# 從原始碼編譯並啟用 FORCE_RSUSB_BACKEND，讓 librealsense 直接透過 libusb 讀取 D455 IMU。
# ⚠️ 編譯後必須覆蓋 /opt/ros 下的 .so，因為 ROS 節點的 RPATH 優先載入該路徑。
RUN git clone --depth 1 --branch v2.56.4 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    mkdir -p /tmp/librealsense/build && cd /tmp/librealsense/build && \
    cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_EXAMPLES=false \
    -DBUILD_GRAPHICAL_EXAMPLES=false \
    -DBUILD_PYTHON_BINDINGS=true \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j$(nproc) && \
    make install && \
    cp /tmp/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/ && \
    # 覆蓋 ROS apt 安裝的 V4L2 版
    cp /usr/local/lib/librealsense2.so.2.56.4 /opt/ros/humble/lib/$(dpkg-architecture -qDEB_HOST_MULTIARCH)/librealsense2.so.2.56.4 && \
    ldconfig && \
    # 將校正腳本複製到 /root 目錄下保存，再刪除原始碼
    cp /tmp/librealsense/tools/rs-imu-calibration/rs-imu-calibration.py /root/rs-imu-calibration.py && \
    rm -rf /tmp/librealsense

# ── 4. Python 庫 ──────────────────────────────────────────────────────────
RUN pip3 install --no-cache-dir \
    setuptools \
    pyserial \
    "numpy<1.24.0" \
    PyYAML

# ── 5. 外部依賴源碼 (vcstool) ─────────────────────────────────────────────
WORKDIR ${WORKSPACE}
COPY dependencies.repos .
RUN mkdir -p src && \
    vcs import src < dependencies.repos && \
    vcs custom src --args submodule update --init --recursive

RUN apt-get update && \
    rosdep update && \
    rosdep install -i --from-paths src -y --ignore-src && \
    rm -rf /var/lib/apt/lists/*

# ── 6. 複製原始碼 & 編譯 ──────────────────────────────────────────────────
COPY ./src ${WORKSPACE}/src

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)"

# ── 7. Entrypoint ─────────────────────────────────────────────────────────
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
WORKDIR /root
ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]
