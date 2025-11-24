# syntax=docker/dockerfile:1.6

# ============================================
# PointCloud Layer for Jetson AGX Orin
# Base: nycusystemlab/agx:latest (from nvcr.io/nvidia/l4t-base:35.4.1)
# ============================================

FROM nycusystemlab/agx:latest

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=noetic
ENV WORKSPACE=/root

# RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/arm64/cuda-ubuntu2004.pin
#     mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
#     https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-tegra-repo-ubuntu2004-11-8-local_11.8.0-1_arm64.deb
#     dpkg -i cuda-tegra-repo-ubuntu2004-11-8-local_11.8.0-1_arm64.deb
#     cp /var/cuda-tegra-repo-ubuntu2004-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
#     apt-get update
#     sudo apt-get install -y cuda-toolkit-11-8

