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

# [安裝 Foxglove & 基礎工具 & Cyclone DDS]
# 關鍵修改：加入了 ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-ros-base \
    ros-${ROS_DISTRO}-foxglove-bridge \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# 設定環境
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

EXPOSE 8765

# 啟動指令
CMD ["bash", "-c", "source /opt/ros/${ROS_DISTRO}/setup.bash && ros2 launch foxglove_bridge foxglove_bridge_launch.xml send_buffer_limit:=20000000"]