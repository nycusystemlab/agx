# syntax=docker/dockerfile:1.6

FROM nycusystemlab/l4t-noetic:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV WORKSPACE=/root

# -------------------------------------------------
# 安裝 ROS 點雲依賴與其他套件
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

# -------------------------------------------------
# 添加 GTSAM PPA 並安裝 GTSAM
# -------------------------------------------------
RUN add-apt-repository -y ppa:borglab/gtsam-release-4.1 && \
    apt-get update && \
    apt-get install -y libgtsam-dev libgtsam-unstable-dev && \
    rm -rf /var/lib/apt/lists/*

# -------------------------------------------------
# 建立工作目錄
# -------------------------------------------------
RUN mkdir -p ${WORKSPACE}/hdl_ws/src \
             ${WORKSPACE}/lidar_ws/src

# -------------------------------------------------
# 複製原始碼
# -------------------------------------------------
COPY ./src/hdl_ws/src ${WORKSPACE}/hdl_ws/src/
COPY ./src/lidar_ws/src ${WORKSPACE}/lidar_ws/src/

# -------------------------------------------------
# 編譯 HDL WS
# -------------------------------------------------
WORKDIR ${WORKSPACE}/hdl_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make -DCMAKE_BUILD_TYPE=Release -DBUILD_VGICP_CUDA=ON"

# -------------------------------------------------
# 編譯 LiDAR WS (isolated)
# -------------------------------------------------
WORKDIR ${WORKSPACE}/lidar_ws
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    catkin_make_isolated -j\$(nproc)"

# -------------------------------------------------
# 啟動設定
# -------------------------------------------------
RUN echo "source ${WORKSPACE}/hdl_ws/devel/setup.bash" >> ~/.bashrc
RUN echo "source ${WORKSPACE}/lidar_ws/devel_isolated/setup.bash" >> ~/.bashrc

CMD ["bash"]
