### 2d lidar
主機須先設定ethernet ip:
```bash
if [ "$ARCH" = "aarch64" ]; then
    # [AGX 端]
    echo "Configuring Network for AGX (eth0)..."
    # 檢查 eth0 是否存在，避免報錯
    if ip link show eth0 > /dev/null 2>&1; then
        # 嘗試設定 IP，如果已經存在則忽略錯誤 (|| true)
        ip addr add 192.168.1.100/24 dev eth0 || echo "IP 192.168.1.100 may already be assigned to eth0."
        ip link set eth0 up
    else
        echo "WARNING: Interface eth0 not found!"
    fi

elif [ "$ARCH" = "x86_64" ]; then
    # [PC 端]
    echo "Configuring Network for PC (eno1)..."
    # 檢查 eno1 是否存在
    if ip link show eno1 > /dev/null 2>&1; then
        sudo ip addr add 192.168.1.100/24 dev eno1 || echo "IP 192.168.1.100 may already be assigned to eno1."
        sudo ip link set eno1 up
    else
        echo "WARNING: Interface eno1 not found! Trying to find generic ethernet..."
        # (選用) 如果 PC 不叫 eno1 (例如叫 eth0 或 enp3s0)，可以在這裡加入備用邏輯
    fi
fi
```
```bash
ros2 launch urg_node2 urg_node2.launch.py
```
