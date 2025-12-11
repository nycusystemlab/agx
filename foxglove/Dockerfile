# syntax=docker/dockerfile:1.6

ARG BASE_PC=osrf/ros:humble-desktop
ARG BASE_AGX=dustynv/ros:humble-desktop-l4t-r36.2.0

FROM ${BASE_PC} AS base-amd64
FROM ${BASE_AGX} AS base-arm64

ARG TARGETARCH
FROM base-${TARGETARCH} AS final

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble

# [修復 GPG Key]
RUN apt-get update || true && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# [安裝 Foxglove & 基礎工具]
# 加入 ros-base 以確保有 CLI 工具
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-foxglove-bridge \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# 設定環境
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

EXPOSE 8765

# 啟動指令
CMD ["bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml send_buffer_limit:=50000000"]