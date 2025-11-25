# syntax=docker/dockerfile:1.6

FROM nycusystemlab/l4t-noetic:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV WORKSPACE=/root

# -------------------------------------------------
# 安裝 ROS 依賴與 GTSAM
# -------------------------------------------------
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-pcl-ros \
    libpcl-dev \
    ros-${ROS_DISTRO}-tf2-eigen \
    ros-${ROS_DISTRO}-message-filters \
    python3-catkin-tools \
    libpcap-dev \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt-get update && \
    apt-get install -y libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

# -------------------------------------------------
# 建立工作目錄（空）
# -------------------------------------------------
RUN mkdir -p ${WORKSPACE}/hdl_ws/src \
             ${WORKSPACE}/lidar_ws/src

# -------------------------------------------------
# 複製 entrypoint
# -------------------------------------------------
COPY navigation/entrypoint.sh /root/entrypoint.sh
RUN chmod +x /root/entrypoint.sh

ENTRYPOINT ["/root/entrypoint.sh"]
CMD ["bash"]
