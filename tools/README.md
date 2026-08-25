# tools —— 通訊行為量測探針

本目錄原本存放 `zenoh-bridge-dds` 的設定與 compose，遷移到 `rmw_zenoh_cpp` 之後
橋接容器不存在了（router 是機器層級的 systemd unit，設定在 infra repo 的
`robot-net/zenoh/`），這裡只剩量測工具。

| 檔案 | 用途 |
|---|---|
| `zping.py` | rclpy 版探針：`ping` / `pong` / `flood` / `sink` / `expect-none` |
| `zping.cpp`＋`CMakeLists.txt` | rclcpp 版：只有 `ping` / `pong` / `expect-none`，這三個角色的邏輯、topic、QoS 與 py 版完全相同（吞吐用的 `flood` / `sink` 只有 py 版有） |
| `zenoh-latency.md` | 實測結果。§1–§9 是舊橋接架構（刻意保留），§10 是 rmw_zenoh |

新舊架構共用同一支探針，所以兩邊的數字可以直接對照——改動它時要維持這個性質。
建置與重跑的完整指令只寫在一處：`zenoh-latency.md` §10.8（C++ 版要 x86 與 aarch64 各編一次）。
