# syntax=docker/dockerfile:1.6

# =======================================================
# 1. 定義基底映像檔
# =======================================================
ARG BASE_PC=osrf/ros:humble-desktop
# [AGX] 
ARG BASE_AGX=nvcr.io/nvidia/isaac/ros:aarch64-ros2_humble_4c0c55dddd2bbcc3e8d5f9753bee634c

# =======================================================
# 2. 架構分流
# =======================================================

# [PC 端] 
FROM ${BASE_PC} AS base-amd64
RUN echo "PC Environment: Installing CPU PyTorch and Git..."
# UPDATE: 補上 git，否則無法 clone urg_node2
RUN apt-get update && apt-get install -y python3-pip git && \
    pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu

# [AGX 端] 
FROM ${BASE_AGX} AS base-arm64
RUN echo "AGX Environment: Using pre-installed PyTorch 2.5..."
USER root
RUN apt-get update && apt-get install -y python3-pip git vim && rm -rf /var/lib/apt/lists/*
RUN pip3 install -U jetson-stats

# =======================================================
# 3. 安裝應用層依賴 (系統套件)
# =======================================================
ARG TARGETARCH
FROM base-${TARGETARCH} AS final

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/root/ros2_ws

# 加入 Yarn GPG key 和 repository (解決 repository 未簽名問題)
RUN curl -fsSL https://dl.yarnpkg.com/debian/pubkey.gpg | gpg --dearmor -o /usr/share/keyrings/yarn.gpg && \
    echo "deb [signed-by=/usr/share/keyrings/yarn.gpg] https://dl.yarnpkg.com/debian stable main" > /etc/apt/sources.list.d/yarn.list

# 安裝 Nav2 與 ROS 工具
RUN apt-get update && \
    apt-get install -y -o Dpkg::Options::="--force-overwrite" \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# =======================================================
# 4. 安裝 Python 庫 (Stable Baselines 3 等)
# =======================================================
RUN pip3 install --no-cache-dir \
    "stable-baselines3" \
    "gymnasium" \
    "tensorboard" \
    "filterpy" \
    "squaternion" \
    "setuptools" \
    "onnxruntime" \
    "pyyaml"

# =======================================================
# 5. [新增] 外部依賴源碼安裝 (urg_node2)
# =======================================================
# 設定工作目錄為 src
WORKDIR ${WORKSPACE}/src

# 下載 urg_node2
RUN git clone --recursive https://github.com/Hokuyo-aut/urg_node2.git

# 切回 workspace 根目錄進行依賴安裝
WORKDIR ${WORKSPACE}

# 初始化 rosdep 並安裝 urg_node2 所需的依賴
# 注意：這裡加上 -y 與 --ignore-src
RUN apt-get update && \
    rosdep update && \
    rosdep install -i --from-paths src -y --ignore-src

# =======================================================
# 6. 複製本地程式碼與最終編譯
# =======================================================
# 將您的本地 src 複製進來 (此時 src 資料夾內會有 urg_node2 和您的 code)
COPY ./src ${WORKSPACE}/src

# 執行編譯
# UPDATE: 加入了 --symlink-install (依照您的需求)
# 並保留了原本的優化參數 (--parallel-workers)
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build \
    --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    --parallel-workers $(nproc)"

COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

WORKDIR /root
ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]