# syntax=docker/dockerfile:1.6

# -----------------------------
# Base image: NVIDIA CUDA
# -----------------------------
ARG BASE_IMAGE=nvcr.io/nvidia/cuda:12.6.2-devel-ubuntu20.04
FROM ${BASE_IMAGE}

# -----------------------------
# 基本工具
# -----------------------------
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    sudo \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    locales \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

# -----------------------------
# ROS Noetic 安裝
# -----------------------------
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-noetic.list' \
    && apt-get update \
    && apt-get install -y ros-noetic-ros-base  \
       ros-noetic-desktop-full  \
    && rm -rf /var/lib/apt/lists/*

# ROS 環境
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc

# -----------------------------
# 編譯與開發工具
# -----------------------------
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-catkin-tools \
    python3-rosdep \
    python3-pip \
    python3-dev \
    python3-numpy \
    x11-apps \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/*

# # PyTorch CUDA 測試
# RUN pip3 install --no-cache-dir torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu126

# # -----------------------------
# # 工作目錄
# # -----------------------------
# WORKDIR /workspace
# RUN mkdir -p /workspace/src

# # 將你的專案程式碼複製進容器
# COPY ./src /workspace/src

# # 安裝 Python 專案依賴（如果有 requirements.txt）
# RUN if [ -f /workspace/src/requirements.txt ]; then pip3 install -r /workspace/src/requirements.txt; fi

# -----------------------------
# 預設啟動 bash
# -----------------------------
CMD ["bash"]
