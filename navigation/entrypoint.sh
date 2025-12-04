#!/bin/bash

# 如果任何命令出錯，立即退出
set -eo pipefail

ROS_DISTRO=noetic
WORKSPACE=/root

echo "========================================="
echo "   ROS ${ROS_DISTRO} Development Container"
echo "========================================="

# -------------------------------------------------
# 1. Source System ROS
# -------------------------------------------------
if [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
    source /opt/ros/${ROS_DISTRO}/setup.bash
fi

# -------------------------------------------------
# 2. Build HDL Workspace (Core SLAM)
# -------------------------------------------------
HDL_WS=${WORKSPACE}/hdl_ws

if [ -d "${HDL_WS}/src" ]; then
    echo "=== Checking hdl_ws ==="
    if [ ! -f "${HDL_WS}/devel/setup.bash" ]; then
        echo ">>> hdl_ws not built. Building now..."
        cd ${HDL_WS}
        catkin_make -j$(nproc) || echo "!!! HDL build failed, continuing..."
    else
        echo ">>> hdl_ws already built. Skipping build."
    fi

    if [ -f "${HDL_WS}/devel/setup.bash" ]; then
        source ${HDL_WS}/devel/setup.bash
    fi
else
    echo "!!! WARNING: hdl_ws/src not found. Skipping."
fi

# -------------------------------------------------
# 3. Build LiDAR Workspace
# -------------------------------------------------
LIDAR_WS=${WORKSPACE}/lidar_ws

if [ -d "${LIDAR_WS}/src" ]; then
    echo "=== Checking lidar_ws ==="
    if [ ! -f "${LIDAR_WS}/devel_isolated/setup.bash" ]; then
        echo ">>> lidar_ws not built. Building now..."
        cd ${LIDAR_WS}
        catkin_make_isolated -j$(nproc) || echo "!!! LiDAR build failed, continuing..."
    else
        echo ">>> lidar_ws already built. Skipping build."
    fi

    if [ -f "${LIDAR_WS}/devel_isolated/setup.bash" ]; then
        source ${LIDAR_WS}/devel_isolated/setup.bash
    fi
else
    echo "!!! WARNING: lidar_ws/src not found. Skipping."
fi

# -------------------------------------------------
# 4. Build RealSense Workspace
# -------------------------------------------------
REALSENSE_WS=${WORKSPACE}/realsense_ws

if [ -d "${REALSENSE_WS}/src" ]; then
    echo "=== Checking realsense_ws ==="
    if [ ! -f "${REALSENSE_WS}/devel/setup.bash" ]; then
        echo ">>> realsense_ws not built. Building now..."
        cd ${REALSENSE_WS}
        catkin_make -j$(nproc) || echo "!!! RealSense build failed, continuing..."
    else
        echo ">>> realsense_ws already built. Skipping build."
    fi

    if [ -f "${REALSENSE_WS}/devel/setup.bash" ]; then
        source ${REALSENSE_WS}/devel/setup.bash
    fi
else
    echo "!!! WARNING: realsense_ws/src not found. Skipping."
fi

# -------------------------------------------------
# 5. Build Keyboard Control Workspace [新增]
# -------------------------------------------------
# 假設我們將其掛載到 /root/keyboard_control_ws
KEYBOARD_WS=${WORKSPACE}/keyboard_control_ws

if [ -d "${KEYBOARD_WS}/src" ]; then
    echo "=== Checking keyboard_control_ws ==="
    if [ ! -f "${KEYBOARD_WS}/devel/setup.bash" ]; then
        echo ">>> keyboard_control_ws not built. Building now..."
        cd ${KEYBOARD_WS}
        catkin_make -j$(nproc) || echo "!!! Keyboard Control build failed, continuing..."
    else
        echo ">>> keyboard_control_ws already built. Skipping build."
    fi

    if [ -f "${KEYBOARD_WS}/devel/setup.bash" ]; then
        source ${KEYBOARD_WS}/devel/setup.bash
    fi
else
    echo "!!! WARNING: keyboard_control_ws/src not found. Skipping."
fi

# -------------------------------------------------
# 6. Append sourcing to .bashrc
# -------------------------------------------------
BASHRC_FILE="${WORKSPACE}/.bashrc"

# ROS System
if ! grep -Fxq "source /opt/ros/${ROS_DISTRO}/setup.bash" $BASHRC_FILE; then
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $BASHRC_FILE
fi

# Function to safely append to bashrc
append_to_bashrc() {
    local WS_SETUP=$1
    if [ -f "$WS_SETUP" ]; then
        if ! grep -Fxq "source $WS_SETUP" $BASHRC_FILE; then
            echo "source $WS_SETUP" >> $BASHRC_FILE
        fi
    fi
}

append_to_bashrc "${HDL_WS}/devel/setup.bash"
append_to_bashrc "${LIDAR_WS}/devel_isolated/setup.bash"
append_to_bashrc "${REALSENSE_WS}/devel/setup.bash"
append_to_bashrc "${KEYBOARD_WS}/devel/setup.bash" # [新增]

# -------------------------------------------------
# 7. Execute passed command
# -------------------------------------------------
echo "=== Environment ready ==="

if [ $# -gt 0 ]; then
    exec "$@"
else
    exec bash
fi