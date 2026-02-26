推薦方案：混合使用
在 AGX 上安裝 jetson-containers，用它來 pull/build image，然後用 Docker Compose 管理容器運行。

# 步驟 1：在 AGX 上安裝 jetson-containers
```bash
# 在 AGX 上執行
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh
```

# 步驟 2：用 jetson-containers 拉取 image
```bash
# 拉取 nano_llm 相容的 image
jetson-containers run $(autotag nano_llm)
# 或指定版本
jetson-containers run dustynv/nano_llm:humble-r36.3.0
```
# 步驟 3：修改 docker-compose.yaml 直接使用 dustynv image

# 在 AGX 上啟動
```bash
cd ~/agx
git pull

# 直接 pull 並啟動（不需要 build）
docker compose up -d nanollm

# 進入容器
make join service=nanollm
```

優點
✅ 不需要 build - 直接 pull 預建 image
✅ 與 Docker Compose 整合 - 統一管理所有服務
✅ 可以用 jetson-containers 工具 - 查看其他可用 image
查看其他可用 image
你的 Dockerfile 可以保留作為參考，或之後需要客製化時再使用。