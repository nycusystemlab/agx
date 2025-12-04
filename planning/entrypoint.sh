#!/bin/bash
set -e

# 載入 ROS 2 環境
source /opt/ros/humble/setup.bash

# 如果有編譯過 workspace，則載入
if [ -f "/root/ros2_ws/install/setup.bash" ]; then
    source /root/ros2_ws/install/setup.bash
fi

exec "$@"