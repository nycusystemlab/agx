#!/bin/bash
# 如果任何命令出錯，立即退出 (保護機制)
set -eo pipefail

ROS_DISTRO=${ROS_DISTRO:-noetic}
WORKSPACE=/root
BASHRC_FILE="${WORKSPACE}/.bashrc"
AUTO_ROSCORE=${AUTO_ROSCORE:-true}

echo "========================================="
echo "   ROS ${ROS_DISTRO} Hybrid Container"
echo "========================================="

# -------------------------------------------------
# Helper 函數：檢查並決定是否編譯
# -------------------------------------------------
build_workspace() {
    local ws_path=$1
    local build_type=$2 # "catkin_make" or "catkin_make_isolated"

    if [ -d "${ws_path}/src" ]; then
        echo "=== Checking $(basename $ws_path) ==="
        
        local setup_file="${ws_path}/devel/setup.bash"
        if [ "$build_type" == "catkin_make_isolated" ]; then
            setup_file="${ws_path}/devel_isolated/setup.bash"
        fi

        # [修改點] 建立一個 flag 來決定是否編譯
        local need_compile="false"

        if [ ! -f "$setup_file" ]; then
            echo "   -> No build artifacts found (Clean build)."
            need_compile="true"
        else
            # 偵測變更核心邏輯：
            # 尋找 src 內比 setup_file 還要新的檔案，只要找到一個 (-print -quit) 就停止
            # 注意：這裡假設 setup.bash 的時間戳代表了上次編譯完成的時間
            local changed_files=$(find "${ws_path}/src" -type f -newer "$setup_file" -print -quit)
            
            if [ -n "$changed_files" ]; then
                echo "   -> Source code change detected ($changed_files)."
                need_compile="true"
            else
                echo "   -> Up-to-date. Skipping compilation."
                need_compile="false"
            fi
        fi

        # 執行編譯
        if [ "$need_compile" == "true" ]; then
            echo ">>> Building $(basename $ws_path)..."

            # 處理 catkin_make 的 CMakeLists 連結問題
            if [ "$build_type" == "catkin_make" ] && [ -f "${ws_path}/src/CMakeLists.txt" ]; then
                # 只有當檔案是連結且斷掉時才刪除，或者直接防禦性刪除
                # 這裡保留原本邏輯，但在 catkin_make 前執行很安全
                if [ -L "${ws_path}/src/CMakeLists.txt" ]; then
                     rm "${ws_path}/src/CMakeLists.txt"
                fi
            fi

            cd ${ws_path}
            # 執行編譯指令
            $build_type -j$(nproc)
        fi

        # 設定環境變數 (無論是否這次有編譯，只要檔案存在就要 source)
        if [ -f "$setup_file" ]; then
            # 1. 載入到當前腳本環境
            source $setup_file
            
            # 2. 寫入 .bashrc
            local source_cmd="source $setup_file --extend"
            if ! grep -Fxq "$source_cmd" $BASHRC_FILE; then
                echo "$source_cmd" >> $BASHRC_FILE
                echo "   -> Added to .bashrc with --extend"
            fi
        fi
    else
        echo "!!! WARNING: $(basename $ws_path)/src not found. Skipping."
    fi
}

# -------------------------------------------------
# 1. Source System ROS & Setup .bashrc
# -------------------------------------------------
source /opt/ros/${ROS_DISTRO}/setup.bash

if ! grep -Fxq "source /opt/ros/${ROS_DISTRO}/setup.bash" $BASHRC_FILE; then
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $BASHRC_FILE
fi

# -------------------------------------------------
# 2. 依序處理所有工作空間
# -------------------------------------------------
build_workspace "${WORKSPACE}/hdl_ws" "catkin_make"
build_workspace "${WORKSPACE}/lidar_ws" "catkin_make_isolated"
build_workspace "${WORKSPACE}/realsense_ws" "catkin_make"
build_workspace "${WORKSPACE}/keyboard_control_ws" "catkin_make"

# -------------------------------------------------
# 2.5 [新增] 自動啟動 Roscore
# -------------------------------------------------
if [ "$AUTO_ROSCORE" = "true" ]; then
    echo "-----------------------------------------"
    echo "   Starting Local Roscore"
    echo "-----------------------------------------"
    
    roscore &
    ROSCORE_PID=$!
    
    echo "Waiting for roscore to initialize..."
    TIMEOUT=10
    COUNT=0
    until rostopic list > /dev/null 2>&1; do
        if [ "$COUNT" -ge "$TIMEOUT" ]; then
            echo "!!! Timeout waiting for roscore !!!"
            break 
        fi
        sleep 1
        COUNT=$((COUNT+1))
        echo -n "."
    done
    echo ""
    echo ">>> Roscore started (PID: $ROSCORE_PID)"
fi

# -------------------------------------------------
# 3. 執行指令
# -------------------------------------------------
source $BASHRC_FILE
echo "=== Environment ready ==="

if [ $# -gt 0 ]; then
    exec "$@"
else
    exec bash
fi