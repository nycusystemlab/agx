# AGX Hybrid Navigation System (ROS 1 Noetic + ROS 2 Humble)

這是一個基於 **Docker** 的混合導航系統專案，專為 **NVIDIA Jetson AGX Orin (JetPack 6)** 平台設計。

本專案採用 **雙軌並行架構**：

1.  **ROS 1 (Legacy):** 負責底層硬體驅動 (Arduino, RealSense, Lidar) 與 3D SLAM (HDL-Graph-SLAM)。
2.  **ROS 2 (Modern):** 負責高階路徑規劃 (Nav2) 與未來的 AI/RL 擴充。
3.  **Bridge:** 透過 `ros1_bridge` 實現跨世代通訊。

-----

## 📂 目錄結構 (Directory Structure)

```text
agx_ros/
├── README.md                   # 本文件
├── docker-compose.yaml         # [AGX] 部署用設定檔 (ARM64/L4T)
├── docker-compose.pc.yaml      # [PC]  開發用設定檔 (x86_64)
├── navigation/                 # [ROS 1] 底層控制與驅動
│   ├── Dockerfile              # AGX 用的 ROS 1 映像檔 (基於 L4T)
│   ├── Dockerfile.pc           # PC  用的 ROS 1 映像檔 (基於 osrf/ros)
│   ├── entrypoint.sh           # 啟動腳本
│   └── src/                    # ROS 1 原始碼 (Host 與 Container 共用)
│       ├── hdl_ws/             # SLAM 演算法
│       ├── lidar_ws/           # 雷達驅動
│       └── realsense_ws/       # 深度相機驅動
├── planning/                   # [ROS 2] 高階導航規劃
│   ├── Dockerfile              # AGX 用的 ROS 2 映像檔 (基於 dustynv)
│   ├── Dockerfile.pc           # PC  用的 ROS 2 映像檔 (基於 osrf/ros)
│   ├── entrypoint.sh           # 啟動腳本
│   └── src/                    # ROS 2 原始碼 (Host 與 Container 共用)
│       └── agx_nav2_config/    # Nav2 參數與地圖
└── vlm/                        # [Future] 視覺語言模型/RL 擴充
```
## 🚀 系統需求 (Prerequisites)

### 硬體
- **Robot:** NVIDIA Jetson AGX Orin (JetPack 6.0+)
- **Workstation:** PC / Laptop (Ubuntu 20.04 或 22.04)
- **Sensors:** RealSense D455, 3D LiDAR, Arduino Microcontroller

### 軟體
- Docker Engine
- Docker Compose (V2)
- NVIDIA Container Toolkit (AGX 必備，PC 若需 GPU 加速也需安裝)

---

## 💻 PC 開發流程 (Development on PC)

在 PC 上進行程式碼撰寫、編譯檢查與邏輯驗證（不含真實硬體）。

### 1. 啟動環境
使用 PC 專用的 Compose 檔啟動：

```bash
docker compose -f docker-compose.pc.yaml up -d --build
```
## 2. 編譯 ROS 1 專案 (Control)

```bash
docker exec -it pc_control_ros1 bash
```
3. 編譯 ROS 2 專案 (Planning)
```bash
docker exec -it pc_planning_ros2 bash

# --- 在容器內 ---
cd /root/ros2_ws
colcon build --symlink-install
```
4. 驗證通訊 (Bridge Test)
```bash
docker logs -f pc_bridge
```
## 🤖 AGX 部署流程 (Deployment on AGX)
1. 啟動環境
```bash
docker compose up -d --build
```
2. 硬體權限確認
容器已開啟 privileged: true 模式，理論上可直接存取 /dev/ttyUSB* 與 /dev/video*。

3. 實機編譯
```bash
docker exec -it agx_control_ros1 bash
```

🗓️ 專案規劃 (Roadmap)

[x] Phase 1: 建立 AGX JetPack 6 混合容器架構 (ROS 1 + ROS 2)

[x] Phase 2: 完成硬體驅動 (Arduino, RealSense) 與 Docker 整合

[ ] Phase 3: 部署 Nav2 導航堆疊並完成與 ros1_bridge 對接

[ ] Phase 4: 引入 VLM/RL 模型於 ROS 2 節點中進行 AI 導航

Maintainer: NYCUSystemLab