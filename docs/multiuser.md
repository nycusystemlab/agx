# 多人開發環境

> 具體的機器位址、帳號與 domain 對應表在 `docs/multiuser.local.md`（不進版控）。
> 沒有那份檔案的話，跟管理者索取。

## 機器分工

| 代號 | 用途 |
|---|---|
| **DEV** | 開發主力。程式碼、模擬、訓練都在這 |
| **SIM** | 實車在環站。唯一連得到實車的機器 |
| **ROBOT** | AGX 實機（部署目標）。L4T R36.4.7 / aarch64 |
| **NAS** | SMB 共用區、Forgejo |

## 你的資源分配

**不要用別人的 domain**——同網段共用會互相收到對方的 `/cmd_vel`。

分配原則（實際對應見 local 檔）：

```
ROS_DOMAIN_ID  = uid - 1000 + 1      # 原有帳號保留 0
Isaac 端       = ROS_DOMAIN_ID + 30
FOXGLOVE_PORT  = 8765 + ROS_DOMAIN_ID
```

這些已寫進各自的 `~/.bashrc`。家目錄在 `/srv/home/<帳號>`，無 quota，但每 6 小時檢查水位，超過 85% 告警。

`domain_bridge` 的 **per-user 轉發已經做好了**（2026-08-18）：設定檔改成 template，
sidecar 啟動時依環境變數展開，所以 31↔1、32↔2、33↔3 都成立，不必再自己複製一份 yaml。

實作在 `amr_simulate` repo 的 `docker/isaacsim/`：

```bash
# 你自己的 checkout，docker/isaacsim/.env
ROS_DOMAIN_ID=1
ISAAC_ROS_DOMAIN_ID=31
CONTAINER_PREFIX=<你的帳號>_

docker compose up -d domain-bridge     # 只起 sidecar
docker logs <你的帳號>_amr_domain_bridge | head -2   # 會印出實際橋的 domain 配對
```

⚠️ **Isaac Sim 本身仍然只能有一份**（WebRTC server 綁 host 的 49100 埠）。
所以實務上是「誰在跑 Isaac，誰決定 Isaac 端的 domain」，其他人把自己的 sidecar
橋到那個 domain。要獨佔的話還是照舊：用之前先講一聲。

## 連線

在實驗室內直接 `ssh <帳號>@<DEV>`。遠端要先連 **wg-lab** VPN。

⚠️ **機器人專用的 WireGuard 網段連不到 DEV**——那是刻意封閉的，開發請用 wg-lab 的 peer。

### 連 ROBOT：一律經 SIM 跳板

存取模型是同心圓，**SIM 是唯一入口**，而且這是被路由器防火牆強制的，不是慣例：

```
遠端開發者 ──wg-lab──▶ SIM ──(跳板)──▶ ROBOT
工作站網段 / wg-lab peer ────✗────▶ ROBOT   （不可直達）
```

```bash
ssh <SIM> -t 'ssh <ROBOT 的 wg IP>'
```

**ROBOT 的位址就是它的 wg IP**（見 local 檔），實驗室內、外場都是同一個 ——
車進出實驗室時你不用改任何腳本或 SSH 設定。底層走有線還是手機熱點只影響速度，不影響位址。

網路規則本身收在 infra repo 的 `robot-net/`（VLAN、防火牆白名單、zenoh 設定範本、驗證腳本）。

## 跨機通訊：兩端 router，不再有橋接容器

ROS 2 的跨機資料以前走 `zenoh-bridge-dds` 容器（本 repo 的 `zenoh/docker-compose.yaml`，已移除；
該目錄改名為 `tools/`，只留量測探針）。
2026-08-24／25 之後改成**兩端各一個 `zenohd` router**，`planning` 與 `foxglove` 直接以
`rmw_zenoh_cpp` 講 zenoh，中間不再有 DDS↔zenoh 的翻譯：

```
SIM  zenohd（listen 127.0.0.1:7447，connect 車的 wg 位址）
       │  ← 白名單只寫在 SIM 端：一條連線同時管住兩個方向
       ▼
車    zenohd（listen wg 位址 + loopback，不載 DDS plugin）
```

- **router 不在本 repo，也不在 compose。** 它是機器層級的 systemd unit `rmw-zenohd`
  （`Restart=always`，開機自動啟動），設定檔、unit 與安裝腳本都在 infra repo 的
  `robot-net/zenoh/`。理由是它的生命週期屬於機器，不屬於任何一個服務堆疊。
- **連線由 SIM 主動發起**，與既有防火牆的最小授權一致，路由器規則一條都沒改。
- **白名單是 keyexpr 層級**（`access_control`，`default_permission: "deny"` + 10 條
  獨立 keyexpr）。keyexpr 格式是 `<domain>/<topic>/<type>/<type_hash>`，所以白名單直接
  照 topic 路徑寫。`0/{scan,goal_pose}/**` 這種列舉語法在 zenoh 1.10 **不支援、不報錯、
  靜默零命中**——改白名單時務必逐條寫。
- **代價：跨機 ROS graph 不通**（`deny` 這個預設一生效，liveliness token 就過不去，
  無解）。所以另一台機器的 topic 不會出現在 `ros2 topic list` 或 Foxglove 裡。
  **資料是通的，只有 graph 不通**，workaround 是指定型別：`ros2 topic echo <topic> <型別>`。
  這是已決定的取捨（留白名單），不是設定壞了。

⚠️ **這不是「全面遷移到 zenoh」。** 現況是：**跨機與主要節點（`planning`、`foxglove`）用
zenoh，Isaac Sim 留在 DDS。** 車端 router 刻意不載 DDS plugin，SIM 端的 DDS plugin 則被
實測否決（`zenoh-bridge-dds` 的 keyexpr 命名空間與 rmw_zenoh 不相容，上游 1.10 沒有相容
模式），所以 **Isaac Sim 的資料進不了 zenoh graph**——要跑 Isaac 的驗證流程就得把
`AGX_RMW` 設回 `rmw_cyclonedds_cpp`（見 `CLAUDE.md`）。`cosmos`（VLA）、`vlm`、`nanollm`
也維持 CycloneDDS，其中 VLA 因此在車上失聯，是刻意取捨、另有 issue 追蹤。

### 啟動與排查

**前置：本機要有一個 `zenohd` router 在跑**，否則節點起得來、`ros2 topic list` 也列得出
名稱，就是收不到資料：

```bash
systemctl status rmw-zenohd        # active 才會通；Restart=always，掛了會自己起來
sudo systemctl restart rmw-zenohd  # 重啟 router 不需要重啟任何 ROS 節點
# unit 名稱以該機器實際安裝的為準（車與 SIM 的 unit 檔在 infra repo 是分開兩份）
docker compose -f docker-compose.yaml up -d planning foxglove
```

| 症狀 | 原因 | 處置 |
|---|---|---|
| `ros2 topic list` **列得出 topic，就是沒資料**；啟動時有一則 `WARN [rmw_zenoh_cpp]: Unable to connect to a Zenoh router` | router 沒起／掛了。**只印 WARN 就照常初始化**，這是唯一線索 | `systemctl status rmw-zenohd`。既有節點圖不受影響，只有**新起的行程**收不到 |
| router 起不來，journal 是 `Address already in use` | 7447 被別人佔著（舊的 `zenoh-bridge` 容器就是這樣） | `docker ps` 找 `zenoh-bridge` 停掉它；橋接容器已從本 repo 移除，車上若還有殘留就是舊部署 |
| 對面機器的 topic **完全不出現**在 `ros2 topic list` / Foxglove | 跨機白名單的 `default_permission: "deny"` 讓 graph 資訊不跨機，**無解，是已決定的取捨** | 資料是通的：`ros2 topic echo <topic> <型別>` 指定型別就收得到 |
| 白名單內的某條 topic 跨機收不到 | keyexpr 寫成 `0/{a,b}/**` 這種列舉語法 —— zenoh 1.10 **不支援、不報錯、靜默零命中** | 逐條拆成獨立 keyexpr（設定在 infra repo） |
| 跨機 `ros2 action send_goal` 停在 `Waiting for an action server` | router↔router 拓撲本身的問題，**與白名單無關**（拿掉 `access_control` 重測一樣） | 未解，已知限制 |
| `Timed out waiting for transform` / `Publisher count = 0` | 兩端 rmw 不一致（最常見：Isaac Sim 的驗證流程忘了整組切回 CycloneDDS） | 把 `AGX_RMW` 與 host 的 `RMW_IMPLEMENTATION` 對齊 |

驗通訊本身用 `tools/zping.py`（`ping`／`pong`／`expect-none`…），
用法見 [`tools/zenoh-latency.md`](../tools/zenoh-latency.md) §10.8。

### 多人隔離：擱置，不是取消

本次遷移的範圍刻意收斂為單人，用既有的預設 domain。以下**都還在桌上，只是沒做**，
之後接手的人可以直接續，不必重談：

- zenoh 的 access control 目前**只用來做 keyexpr 過濾**，`subjects` 僅以網卡分
  「本機（`lo`）／跨機（`eno1`）」——**沒有憑證、沒有 per-user subject**。
- per-user 的 router 與連接埠沒有做。
- 跨機白名單的 keyexpr 寫死 `0/…`，所以**跨機目前只有 domain 0 會通**；本機的
  domain 隔離不受影響（domain 是 keyexpr 的第一段，天生互不可見）。

## compose 變數

```bash
cp .env.example .env    # 填自己的值
```

| 變數 | 不設定時 |
|---|---|
| `ROS_DOMAIN_ID` | `0` |
| `CONTAINER_PREFIX` | 空（容器叫 `planning`、`foxglove`…） |
| `FOXGLOVE_PORT` | `8765` |
| `AGX_PROJECT_ROOT` / `AGX_WORKSPACES` | 實機的預設路徑 |
| `AGX_RMW` | `planning` 與 `foxglove` 都是 `rmw_zenoh_cpp`。兩者的映像都裝了兩套 rmw，切換不必重建映像。用 zenoh 時本機要有 `zenohd` router，且跟 Isaac Sim、`cosmos`、`vlm`、`nanollm` 是斷的（它們仍是 CycloneDDS）——**在 SIM 上跑 Nav2 驗證流程要設成 `rmw_cyclonedds_cpp`**。Foxglove 只列得出**同一台機器**的 topic（跨機 graph 被 keyexpr 白名單擋住，是已決定的取捨）。細節見 `.env.example` |

除了 `AGX_RMW`（`planning`／`foxglove` 預設已改為 zenoh），不建 `.env` 時行為與單人時期相同。`.env` 已 gitignore——每人的值不同，不要 commit。

## 兩個會踩的坑

> 🚧 **這一節正在被 infra repo 的 `robot-net/` 取代，但還沒完全交接。**
>
> | | 狀態（2026-08-25） |
> |---|---|
> | ROS 2 topic | ✅ **已改用兩端 `zenohd` router + `rmw_zenoh_cpp`**（見上面「跨機通訊」一節），keyexpr 白名單明列；新舊架構的延遲實測見 [`tools/zenoh-latency.md`](../tools/zenoh-latency.md) |
> | 連 ROBOT | ✅ 已改用車的 wg IP，見上面「連 ROBOT」一節 |
> | 取 image | ✅ ROBOT 已能直接 `docker pull`；下面第 2 點的三步中轉**作廢** |
> | rosbag | ✅ 防火牆已放行 ROBOT → NAS |
> | ROBOT ↔ SIM 傳大檔 | ⬜ **SIM 的 USB WiFi 還在**，下面第 1 點仍然適用 |
>
> 所以：**第 2 點已經是歷史，第 1 點還是現況。** 等 dock 插線 + USB WiFi 退役後，
> 整節可以刪掉。判準是在 SIM 上跑 infra repo 的 `bash robot-net/verify.sh`。

### 1. ROBOT ↔ SIM 傳大檔要走有線，差五倍

SIM 有兩個介面，走 USB WiFi 只有 2.3 MB/s，走有線有 17–19 MB/s。而且**必須由 ROBOT 主動發起**，否則 SIM 的路由表會把流量導回 WiFi：

```bash
# SIM 端先就位
nc -l -p 19999 | zstd -d | docker load
# ROBOT 端推送（位址見 local 檔）
ssh <ROBOT> 'docker save <image> | zstd -1 -T0 | nc -q0 <SIM 有線位址> 19999'
```

`zstd -1` 有明顯效果——`docker save` 輸出的 layer 是未壓縮 tar。

### 2. ROBOT 連不到 Harbor，還原 image 要經 SIM 中轉

arm64 image 備份在 DEV 的 Harbor（project `agx-arm64`，tag 對應 L4T 版本），但防火牆上 Harbor 只開放給 SIM。所以還原是三步：

```bash
# 在 SIM 上
docker pull --platform linux/arm64 <harbor>/agx-arm64/planning:<tag>
docker tag  <harbor>/agx-arm64/planning:<tag> agx_ros/planning:latest
docker save agx_ros/planning:latest | zstd -1 | ssh <ROBOT> 'zstd -d | docker load'
```

**刪 ROBOT 上的 image 前要想清楚**——還原一個 40 GB 的 image 約 20 分鐘。

另外 SIM 上有 x86 版的同名 image，中轉前先 `docker tag agx_ros/planning:latest agx_ros/planning:x86-local` 保護，否則 arm64 版會搶走 `:latest`，模擬環境跑不起來。

## 規矩

- **`colcon build` 在實機做**（產物僅約 136 MB）；**`docker build` 不要在實機做**（cache 會長到 134 GB）
- **實車、Isaac Sim、SIM 的桌面** 都只有一份，用之前先講一聲
