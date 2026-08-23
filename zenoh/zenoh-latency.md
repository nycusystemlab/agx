# zenoh 橋接實測：延遲、流量上限與瓶頸歸因（SIM ↔ 實車）

量測日期 **2026-08-18／19**，車在 dock（wg 底層走有線）。工具是本目錄的
`zping.py`（rclpy）與 `zping.cpp`（rclcpp），兩者邏輯、topic、QoS 完全相同。
下面每一格都是實際跑出來的，沒有推估值。

## 一句話

橋本身很便宜（四段 hop 合計 <1 ms、線上額外位元組 ~5%）；
**延遲的大宗是端點語言與網路，吞吐的上限是 zenoh 送端對分片的壅塞丟棄** ——
兩者都不是頻寬、不是 CPU、也不是 wg。

| 想知道 | 答案 |
|---|---|
| 控制訊息延遲 | rclpy 3.6–3.9 ms / **rclcpp 2.9–3.3 ms**（其中網路就佔 1.6–2.1 ms） |
| 雷達（4.4 KB）延遲 | 3.3–3.9 ms，30 Hz 連續 30 s 不掉包 |
| 640×480 未壓縮影像延遲 | 126–173 ms（**不建議過橋**；同資料直連 DDS 只要 2.9 ms） |
| 實用吞吐（大訊息） | ~20 MB/s；改壅塞策略後 ~25 MB/s |
| 實用吞吐（≤256 KB 訊息） | **45 MB/s 不掉包、68 MB/s 掉 2.9%** |
| 訊息率 | 2000 msg/s 仍未觸及橋的上限 |

---

## 1. 量測方法

**延遲**：兩端各一個節點，pong 把收到的 `header.stamp` 原樣抄回 `/goal_pose`，
所以 RTT 全程用 SIM 的時鐘算，**兩台機器不需要對時**。

```
SIM  ping ──/initialpose（或 /scan、/d455/color/image_raw）──▶ 車 pong
SIM  ping ◀───────────── /goal_pose（原樣抄回 header.stamp）── 車 pong
```

**吞吐**：單向灌，序號放在 `header.frame_id`，收端算實際到達的訊息數、位元組數與序號缺口。

探針 topic 只能從 zenoh allowlist 裡挑（`initialpose`、`scan`、`d455/color/image_raw`、
`goal_pose`）。量測當下兩端 domain 0 只有 `foxglove_bridge`，這些 topic 沒有別的訂閱者。
**不要用 `/cmd_vel` 當探針**——車上 `control` 會吃它。

```bash
# 車上（planning image，host network，ROS_DOMAIN_ID 對上 bridge 設定的 plugins.dds.domain）
python3 zping.py --role pong  --mode small
python3 zping.py --role sink  --mode image --rate 20 --seconds 45

# SIM 上
python3 zping.py --role ping  --mode small --count 900 --rate 30 --label 30Hz
python3 zping.py --role flood --mode image --rate 20 --seconds 15

# C++ 版：兩個架構各編一次（planning image 內就有 g++/cmake/rclcpp）
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
./build/zping --role ping --mode small --count 200 --rate 10
```

**否定測試**（`expect-none`）：斷言某個 topic 的訊息**不該**送達。收端在 `--seconds`
內收到任何一則即失敗（exit 1），一則都沒收到才通過（exit 0）。白名單設錯時橋不報錯，
只會靜默放行或靜默阻擋，沒有這個角色就分不出「擋住了」與「根本沒在跑」。

`--topic` 可覆寫 `--mode` 的預設 topic（訊息型別仍由 `--mode` 決定），否定測試就是靠它
指向一個**不在白名單內**的 topic。訂閱固定用 BEST_EFFORT，避免 QoS 不相容造成假性通過。

**`expect-none` 單獨跑沒有證據力。** 橋掛掉、網路斷、`ROS_DOMAIN_ID` 設錯時它一樣通過。
必須跟同一時窗的正向 ping（白名單內 topic、**必須**收到）配成一對，兩邊都符合預期才算
閘門有效。下面的範例只是否定那一半。

```bash
# 發送端：對一個不該過橋的 topic 灌流量
python3 zping.py --role flood --mode scan --topic /not_allowlisted --rate 20 --seconds 20

# 接收端：預期收不到
python3 zping.py --role expect-none --mode scan --topic /not_allowlisted --seconds 10 --label gate
./build/zping    --role expect-none --mode scan --topic /not_allowlisted --seconds 10 --label gate
```

---

## 2. 跨橋延遲：rclpy vs rclcpp

RTT，單位 ms，所有情境掉包皆為 0%。

| payload | 速率 | rclpy p50 | rclpy p90 | rclpy p99 | **rclcpp p50** | rclcpp p90 | rclcpp p99 |
|---|---|---|---|---|---|---|---|
| 小訊息 0.7 KB | 10 Hz ×200 | 3.86 | 4.70 | 7.04 | **3.32** | 3.83 | 4.26 |
| 小訊息 0.7 KB | 30 Hz ×900 | 3.56 | 4.07 | 5.04 | **2.94** | 3.48 | 3.96 |
| LaserScan 360 點 1.5 KB | 10 Hz ×150 | 3.65 | 4.26 | 4.58 | **3.30** | 3.86 | 4.25 |
| LaserScan 1080 點 4.4 KB | 10 Hz ×150 | 3.88 | 4.48 | 6.07 | **3.30** | 3.80 | 4.16 |
| 影像 320×240 230 KB | 5 Hz ×60 | 20.5 | 21.8 | 23.0 | **9.76** | 10.40 | 10.82 |
| 影像 640×480 921 KB | 5 Hz ×60 | 166.7 | 197.8 | 364.6 | **126.0** | 227.5 | 338.0 |
| 影像 640×480 921 KB | 1 Hz ×25 | 172.9 | 267.9 | 372.2 | **132.1** | 135.2 | 324.1 |
| 影像 640×480（送端改 block） | 5 Hz ×60 | — | — | — | 127.7 | 329.3 | 549.1 |

換 C++ 的收益：小訊息 −0.4～0.6 ms、230 KB 影像 **−52%**、921 KB 影像 −24%。
**中等大小訊息收益最大**——那個區間序列化／複製成本高，但還沒撞上分片問題。

## 3. 把成本拆開：端點 vs 網路 vs 橋

同一份程式、同一個 image，不過橋（domain 66 loopback）：

| 本機來回 0.7 KB | rclpy | **rclcpp** | 純 C DDS（`ddsperf`） |
|---|---|---|---|
| SIM（x86） | 0.72 | **0.234** | 0.050 |
| 車上（ARM） | 2.24 | **0.457** | 0.092 |

| 網路與其他基準 | |
|---|---|
| ICMP SIM → 路由器（wg 終端） | **0.28 ms** |
| ICMP SIM → 車（穿過 wg） | **1.60–2.08 ms**（min 1.28 / max 2.55） |
| ssh 吞吐 SIM → 車（50 MB） | 55 MB/s |
| 本機來回 921 KB（rclpy） | SIM 42.3 ms、車上 123.3 ms |
| 本機來回 921 KB（**rclcpp**） | SIM **0.89 ms**、車上 **2.92 ms** ← 端點根本不是大訊息的成本 |

**拆帳**（跨橋小訊息 rclcpp 2.94 ms）：網路 1.6–2.1 ms ＋ 兩端 rclcpp 合計 ~0.7 ms
＋ **四段 zenoh bridge hop 0.34 ms**（§6.2 在同一台機器上直接量到，不是推估）。
rclpy 版多出來的 ~0.6 ms 就是 Python。

## 4. 吞吐：單串流與聚合

**上行（SIM → 車），921 KB 影像單一串流**

| offered | delivered | 掉包 |
|---|---|---|
| 5 Hz | 4.7 MB/s | 0% |
| 10 Hz | 9.3 MB/s | 0% |
| 20 Hz | 17.5 MB/s | 3.5–5.4% |
| 40 Hz（發端只發得出 27 Hz） | 13.0 MB/s | 47% |

**上行聚合（921 KB 影像 + 1 MB LaserScan 兩條串流）**——單一 Python 行程發不動更多：

| offered 合計 | delivered 合計 | 掉包 | eno1 實際上行 |
|---|---|---|---|
| ~19 MB/s | **19.0 MB/s** | 0 / 1.5% | ~20 MB/s |
| ~29 MB/s | 27.2 MB/s | 4.3 / 6.4% | ~26 MB/s |
| ~48 MB/s | 35–40 MB/s | 20 / 30% | ~30–38 MB/s |

線上位元組只比 payload 多約 **5%**——zenoh 的封裝開銷可以忽略。

**下行（車 → SIM），921 KB 影像**：**8.4 Hz / 7.8 MB/s、掉包 0%，offered 開到 20 Hz 也一樣**。
瓶頸在車上的 Python publisher（同支腳本在車上自發自收也是 8.3 Hz；在 SIM 上是 26 Hz / 24 MB/s）。
真實相機 driver 是 C++，不受這條限制。

**各端本機天花板（rclpy、921 KB）**：SIM 24.1 MB/s（26 Hz）、車上 7.8 MB/s（8.3 Hz）。

## 5. 瓶頸歸因：是訊息大小，不是頻寬

**把頻寬固定在 ~18 MB/s，只改單筆訊息大小**：

| 單筆大小 | 速率 | delivered | 掉包 |
|---|---|---|---|
| 64 KB | 270 Hz | 17.3 MB/s | **0%** |
| 128 KB | 140 Hz | 17.9 MB/s | **0%** |
| 256 KB | 70 Hz | 18.0 MB/s | **0%** |
| 512 KB | 35 Hz | 17.9 MB/s | 0.4% |
| 921 KB | 20 Hz | 17.5 MB/s | 5.4% |

**同樣用 64 KB 訊息往上推**，證明頻寬遠不止 20 MB/s：

| 速率 | delivered | 掉包 |
|---|---|---|
| 400 Hz | 25.6 MB/s | **0%** |
| 700 Hz | **44.8 MB/s** | **0%** |
| 1100 Hz | **68.2 MB/s** | 2.9% |

**小訊息的訊息率**（0.7 KB）：

| 情境 | offered | delivered | 掉包 |
|---|---|---|---|
| 跨橋 | 500 Hz | 499 Hz | 0% |
| 跨橋 | 2000 Hz | 1978 Hz | 0.2% |
| SIM 本機（對照） | 2000 Hz | 1982 Hz | 0% |

跨橋與本機同一個數字（2000 Hz 是 Python 發端上限），**橋在 2 kHz 下還沒開始限流**。

**機制**：zenoh 的 `batch_size` 是 65535，超過就分片；1.10 預設 tx queue 每個優先權只有
**2 個 batch**，壅塞時預設**丟**（`congestion_control.drop`）。一筆 921 KB 要 15 片，
**掉一片整筆報銷**，所以片數越多命中率越差。

**驗證**（另起一支 bridge，domain 77 → 車 domain 0，只改壅塞策略）：

```json5
qos: { publication: [ { key_exprs: ["**"], config: { congestion_control: "block", ... } } ] },
transport: { link: { tx: { queue: { size: { data: 8 } } } } },
```

| 921 KB 影像 | 預設（drop） | **block + queue 8** |
|---|---|---|
| 20 Hz | 17.5 MB/s，掉 5.4% | **18.1 MB/s，掉 0.7%** |
| 發滿（27 Hz） | 13.0 MB/s，掉 47% | **25.0 MB/s，掉 0.2%** |
| RTT p50 | 126.0 ms | 127.7 ms（**沒有改善**） |

**飽和時的 CPU**（12 核的 AGX）：`zenoh-bridge` 37.5% 的**單核**，兩個 Python 收端各 12–19%，
SIM 端 bridge 10%。**CPU 不是瓶頸**。

> ⚠️ 壅塞策略的改動**只在測試 bridge 上驗證過**，沒有進 infra repo 的設定範本。
> 要套用的話兩端都要改，而且 `key_exprs` 只該指向影像那幾條——
> `/cmd_vel` 這種控制流量不該 block（寧可丟舊的，也不要讓發端卡住）。

## 6. 跟原本的 DDS 雙機通訊比較

### 6.1 先講結論：在現在的網路下，純 DDS 直連**不可能**

實測：兩端各給 CycloneDDS 一份 unicast peer 設定（關多播、互指對方位址）、同一個 domain，
**discovery 完全不成立**，一個 node 都看不到、一則訊息都沒過。

原因在 infra repo 的 `robot-net/wg-robot-allowlist.rsc`：SIM → 車只放行

```
protocol=tcp  dst-port=22,<zenoh port>      # ssh + zenoh
protocol=icmp                                # 診斷用
connection-state=established,related         # 回程
```

**UDP 一條都沒有**，而 DDS 的 RTPS 是 UDP。反向（車 → SIM）連 ICMP 都是 DROP，
實測 `ping 192.168.88.254` 100% 掉包。所以：

- 原本那套之所以能動，是因為 **SIM 插一張 USB WiFi 直接加入車的網段**，兩台在同一個 L2 上靠多播互相看到。
- 這條路現在是刻意封掉的（USB WiFi 也已停用），不是「還沒設定」。
- zenoh 能過，是因為它把所有 topic 多工在 **SIM 主動發起的單一 TCP 連線**上——
  這正好是白名單唯一允許的形狀。

**所以這個比較不是「哪個比較快」，而是「一個能用、一個在現行安全模型下不能用」。**

### 6.2 純協定成本：同一台機器、C++、無網路

為了把「橋的成本」跟「網路的成本」分開，在 SIM 上起一對 zenoh bridge
（domain 88 ⇄ 99，走 `tcp/127.0.0.1:7500`），跟同機同 domain 的直連 DDS 對比。
RTT p50，單位 ms：

| payload | 直連 DDS | 經兩支橋 | 橋的成本 |
|---|---|---|---|
| 0.7 KB | 0.234 | 0.576 | +0.34 |
| 4.4 KB（scan 1080） | 0.223 | 0.672 | +0.45 |
| 64 KB | 0.308 | 0.946 | +0.64 |
| 128 KB | 0.308 | 1.196 | +0.89 |
| 256 KB | 0.589 | 1.711 | +1.12 |
| 512 KB | 1.007（p90 **102**） | 104.1 | 見下 |
| 921 KB | 3.701（p90 **303**） | 108.9 | 見下 |

**≤256 KB：橋只要 0.3–1.1 ms**，四段 hop 平均每段不到 0.3 ms。這跟跨機量到的
「小訊息 2.94 ms、其中網路 1.6–2.1 ms」完全對得起來。

### 6.3 ≥512 KB 的 ~100 ms 量子：**直連 DDS 也有**

注意上表 512 KB / 921 KB 那兩列的直連欄：p50 只有 1.0 / 3.7 ms，
**但 p90 是 102 / 303 ms**。把速率降到 2 Hz 重測，尾巴照樣在（p90 102.7、p99 300.8），
所以不是佇列堆積，是**每則大訊息各自有機率**撞到——典型的
「分片遺失 → 等重傳逾時」行為（~100 ms 的整數倍非常明顯）。

差別在於：**直連的 p50 還在 1–3.7 ms，經橋的 p50 已經落在 105–109 ms**，
也就是橋讓這件事從偶發變成常態。

這也修正了前面幾節一個推論：跨機 921 KB 的 126 ms **不是** AGX 端點的成本——
車上用 C++ 直連量同樣的 921 KB 只要 **2.92 ms**（rclpy 的 123 ms 是 Python 複製大陣列造成的）。
**那 120 ms 幾乎全部是橋在處理大訊息。**

### 6.4 一句話總結這個比較

| | 原本：DDS 直連（同 L2 + 多播） | 現在：zenoh 橋（單一 TCP） |
|---|---|---|
| 在現行防火牆下 | **不通**（UDP 全擋、反向全 DROP） | 通 |
| 哪些 topic 會過 | 全部（含 `/rosout`、參數事件） | 只有 allowlist 明列的 |
| 車進出實驗室要不要改設定 | 要（換網段／換 IP） | 不用（車的 wg IP 不變） |
| 小訊息延遲成本 | 基準 | **+0.34 ms** |
| ≤256 KB 延遲成本 | 基準 | **+0.3–1.1 ms** |
| ≥512 KB | p50 快，但 p90 已有 ~100 ms 尾巴 | p50 就是 ~105 ms |
| 大訊息吞吐 | 未測（此路徑不通） | ~20 MB/s（改壅塞策略 ~25） |
| ≤256 KB 吞吐 | 未測 | 44.8 MB/s 零掉包 |

## 7. 外場情境模擬（netem）：這裡 zenoh 輸得很慘

前面的數字全部來自 dock 有線（實質零丟包）。行動網路不是這樣，所以用 netem 在
SIM 上開兩個容器（各自 netns、`--cap-add=NET_ADMIN`），中間套
**單向 25 ms ±5 ms 抖動 + 1% 丟包**（RTT ≈ 50 ms，接近 4G），
同一組條件下比 **DDS 直連（unicast peers）** 與 **zenoh 橋**（兩支 bridge 分別與端點共用 netns）。

C++ 端點，RTT p50 / p90（ms），`recv` 是 100 送出中實際收到的數量：

| payload | DDS 直連 p50 | p90 | recv | zenoh 橋 p50 | p90 | recv |
|---|---|---|---|---|---|---|
| 0.7 KB | 50.8 | 61.1 | 300/300 | **51.7** | 62.7 | 300/300 |
| 4.4 KB | 56.0 | 71.7 | 149/150 | **53.5** | 61.1 | 150/150 |
| 128 KB | **108.9** | 141.7 | 100/100 | **3404** | 3890 | **74/100** |
| 921 KB | **601** | 707 | 50/50 | **7034** | 8760 | **17/50** |

**控制流量等級（≤4.4 KB）兩者沒有差別**；但 **128 KB 以上 zenoh 慢 30 倍以上，而且大量訊息根本沒到**。

### 為什麼

拆開來看 zenoh 的 128 KB：

| 條件 | p50 |
|---|---|
| 只有延遲（25 ms 單向、0 丟包） | 101 ms |
| 只有丟包（1%、0 延遲） | 1.2 ms |
| **兩者同時** | **3404 ms** |

單獨都沒事，**相乘才爆炸**——這是 TCP 的教科書行為（吞吐 ≈ MSS/(RTT·√p)：
50 ms RTT + 1% 丟包 ≈ 290 KB/s，低於 128 KB × 5 Hz = 640 KB/s，於是佇列無限堆積）。
zenoh 走單一 TCP 連線，就繼承了 TCP 的壅塞控制；DDS 的 RTPS 走 UDP、自己用 NACK
只重傳掉的分片，不會因為丟包就把整體速率砍半。

把 zenoh 端點改成 `udp/` 測過：延遲確實回到 65 ms，但 **1% 丟包下連 session 都建不穩**
（`Received an invalid message in response to an OpenSyn`），100 則只收到 19 則。
以現在這版（1.10）來說不是可用的替代方案。

> 這一節是**模擬**，不是外場實測。真實行動網路的丟包型態（突發、非獨立）與 netem 的
> 均勻丟包不同，數量級可以參考，絕對值不要當成外場保證。

## 8. 結論與建議

| 結論 | 依據 |
|---|---|
| **控制流量（≤4.4 KB）過橋是安全的** | 3.3–3.9 ms、30 Hz 連續 30 s 零掉包、2 kHz 訊息率仍未限流；**在模擬的行動網路下也與 DDS 直連平手** |
| **橋不是延遲來源** | 四段 hop <1 ms；網路 1.6–2.1 ms 才是大頭，且已接近 ICMP 地板 |
| **想更低延遲，先換 C++ 再想別的** | 車上端點 2.24 → 0.457 ms；換 rmw_zenoh 只能省那 <1 ms 的 hop |
| **同機零複製（iceoryx / SHM）對這條路沒用** | 瓶頸在跨機路徑，不在本機 IPC |
| **不要過橋傳未壓縮影像** | 640×480 跨橋 126 ms，其中約 120 ms 是橋處理大訊息；同樣資料直連 DDS 只要 2.9 ms |
| **要提高大訊息吞吐，先動壅塞策略，不是換硬體** | CPU 37% 單核、路徑實測可跑 68 MB/s |
| **外場（有丟包）時大訊息不要走橋** | 模擬 50 ms RTT + 1% 丟包：128 KB 過橋 3.4 s、26% 沒到；DDS 直連 109 ms、零遺失 |

延遲的地板是 ICMP 的 ~1.6 ms —— **現在離地板的距離主要是 Python，不是 DDS，也不是 zenoh。**

## 9. 重跑注意事項

- 兩端都用 `agx_ros/planning:latest`、`--network host`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`，
  `ROS_DOMAIN_ID` 對上各自 bridge 設定裡的 `plugins.dds.domain`。
- 探針 topic 必須在**兩端 allowlist 都有**，否則橋不會過，症狀是 ping 端安靜地收不到回覆。
- C++ 版要在 x86 與 aarch64 各編一次；`build/` 由容器內的 root 產生，清理時記得用 root 容器刪。
- 測完把車上的 pong / sink 容器與暫存檔刪掉，別留在車上。
- 車若在外場走行動網路，數字會完全不同——這份表只代表 **dock 有線**。
