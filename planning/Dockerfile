# syntax=docker/dockerfile:1.6

# =======================================================
# 1. 定義基底映像檔
# =======================================================
ARG BASE_PC=osrf/ros:humble-desktop
ARG BASE_AGX=dustynv/ros:humble-desktop-l4t-r36.2.0

# =======================================================
# 2. 自動選擇基底
# =======================================================
FROM ${BASE_PC} AS base-amd64
RUN echo "Base AMD64 Set"

FROM ${BASE_AGX} AS base-arm64
RUN echo "Base ARM64 Set"

ARG TARGETARCH
FROM base-${TARGETARCH} AS final

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=humble
ENV WORKSPACE=/root/ros2_ws

# =======================================================
# 更新過期的 ROS GPG Key
# =======================================================
RUN apt-get update || true && \
    apt-get install -y curl gnupg2 lsb-release && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# =======================================================
# 3. 安裝 Nav2 與相關依賴
# =======================================================
RUN apt-get update && \
    apt-get install -y -o Dpkg::Options::="--force-overwrite" \
    # [關鍵新增] 顯式安裝 ros-base，確保 ros2 指令存在
    ros-${ROS_DISTRO}-ros-base \
    # [之前新增] Cyclone DDS
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    # Nav2 相關
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher \
    python3-colcon-common-extensions \
    python3-pip \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# =======================================================
# 4. 複製程式碼並「預先編譯」
# =======================================================
WORKDIR ${WORKSPACE}

# 複製 src
COPY ./src ${WORKSPACE}/src

# colcon 編譯
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
                  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers $(nproc)"

# =======================================================
# 5. 設定 Entrypoint
# =======================================================
COPY entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

WORKDIR /root
ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]