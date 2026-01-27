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
RUN echo "PC Environment: Installing CPU PyTorch..."
RUN apt-get update && apt-get install -y python3-pip && \
    pip3 install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu

# [AGX 端] 
FROM ${BASE_AGX} AS base-arm64
RUN echo "AGX Environment: Using pre-installed PyTorch 2.5..."
USER root

RUN apt-get update && apt-get install -y python3-pip git vim && rm -rf /var/lib/apt/lists/*
RUN pip3 install -U jetson-stats

# =======================================================
# 3. 安裝應用層依賴
# =======================================================
ARG TARGETARCH
FROM base-${TARGETARCH} AS final

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/root/ros2_ws

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
# 4. 安裝 RL 庫 (Stable Baselines 3)
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
# 5. 編譯程式碼
# =======================================================
WORKDIR ${WORKSPACE}
COPY ./src ${WORKSPACE}/src

RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)"

COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

WORKDIR /root
ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]