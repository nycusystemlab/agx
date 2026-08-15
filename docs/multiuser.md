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

> ⚠️ **`domain_bridge` 目前只橋 domain 30 ↔ 0**（見 `amr_simulate/docker/isaacsim/domain_bridge.yaml`）。
> 上面的「Isaac 端 = ROS_DOMAIN_ID + 30」是**位址分配的約定**，不是已經做好的轉發：
> 31↔1、32↔2、33↔3 的 per-user 轉發**還沒有人實作**。
> 所以除了 `ubuntu`（domain 0）以外，你的 Isaac 端不會自動橋到自己的 domain ——
> 要嘛用 domain 0，要嘛自己複製一份 `domain_bridge.yaml` 改掉 domain 再起一個 sidecar。
> 寫在這裡是為了讓你**不要照著超前的文件，去除錯一個不存在的功能**。

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
| `ZENOH_BRIDGE_CONFIG` | `robot-bridge.json5`（車端）。**在 SIM 上要設成 `sim-bridge.json5`** |

不建 `.env` 時行為與單人時期完全相同。`.env` 已 gitignore——每人的值不同，不要 commit。

## 兩個會踩的坑

> 🚧 **這兩個坑正在被 infra repo 的 `robot-net/` 消滅，但施工還沒做。**
> 下面寫的是**現在**（USB WiFi 還在、車連不到 Harbor）真的要照著做的流程。
> `robot-net` 套用後改成這樣，屆時請把這一節換掉、只留新流程：
>
> | | 現在 | robot-net 套用後 |
> |---|---|---|
> | ROBOT ↔ SIM | SIM 的 USB WiFi 加入車的網段；傳大檔必須由 ROBOT 主動發起 | SIM 有線 → wg；USB WiFi 退役，沒有路由不對稱的問題 |
> | 取 image | 三步中轉（下面第 2 點），40 GB 約 20 分鐘 | ROBOT 直接 `docker pull <harbor>/agx-arm64/…` |
> | ROS 2 topic | CycloneDDS 多播，靠「剛好同一個 L2」 | `zenoh-bridge-dds` 明列 allowlist（`zenoh/docker-compose.yaml`） |
> | rosbag | 手動 scp 中轉 | ROBOT 直接寫 NAS 共用目錄 |
>
> 施工完成的判準：在 SIM 上跑 infra repo 的 `bash robot-net/verify.sh` 全綠。

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
