# ./nanollm/Dockerfile

# 1. 基底映像檔 - Jetson 專用 NanoLLM (含 ROS Humble)
FROM dustynv/nano_llm:humble-r36.3.0

ENV DEBIAN_FRONTEND=noninteractive

# 2. 更新 ROS 2 GPG key (解決過期問題)
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# 2.1 修正 pip 設定 - 保留 NGC，加入 cu126，移除 cu122
# 保留原有的 global 設定，只修改 :env: 環境變數設定
ENV PIP_INDEX_URL=https://pypi.org/simple
ENV PIP_EXTRA_INDEX_URL="https://pypi.jetson-ai-lab.io/jp6/cu126 https://pypi.ngc.nvidia.com"
ENV PIP_TRUSTED_HOST="pypi.jetson-ai-lab.io jetson.webredirect.org pypi.ngc.nvidia.com"

# 3. 安裝基礎工具
RUN apt-get update && apt-get install -y \
    curl \
    wget \
    git \
    htop \
    nano \
    && rm -rf /var/lib/apt/lists/*

# 3. 安裝 Jetson 監控工具
RUN pip3 install -U jetson-stats

# 4. 安裝額外 ROS 2 Humble 套件（基底已包含 ROS Humble）
# RUN apt-get update && apt-get install -y \
#     ros-humble-rmw-cyclonedds-cpp \
#     ros-humble-cv-bridge \
#     ros-humble-image-transport \
#     && rm -rf /var/lib/apt/lists/*

# 5. 安裝額外的 Python 套件 (不要覆蓋基底映像檔的 numpy 和 opencv)
RUN pip3 install --no-cache-dir \
    pillow \
    transformers \
    timm \ 
    accelerate

# 安裝 CLIP (ftfy 和 regex 是相依套件)
RUN pip3 install --no-cache-dir ftfy regex
RUN pip3 install --no-cache-dir git+https://github.com/openai/CLIP.git 
# 6. 設定環境變數
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV ROS_DOMAIN_ID=0

# 7. 設定工作目錄
WORKDIR /data

# 8. 把 source setup.bash 寫入
RUN echo "source /opt/ros/install/setup.bash" >> /root/.bashrc

# 9. 預設指令
CMD ["bash", "-l"]
