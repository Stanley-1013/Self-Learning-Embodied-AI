---
title: "ROS 2 進階：QoS 與執行器"
prerequisites: ["05-ros2-tf-and-tools"]
estimated_time: 45
difficulty: 3
tags: ["ros2", "qos", "executor", "callback-group", "dds"]
sidebar_position: 6
---

# ROS 2 進階：QoS 與執行器

## 你將學到

- 能精確說出 QoS 七大策略各自管什麼、常用 preset 差在哪，面試被問「LiDAR topic 該用什麼 QoS？」能秒答
- 遇到「兩個 node 連上了卻收不到資料」的情境，立刻想到 QoS 相容性問題並用 `ros2 topic info -v` 診斷
- 能區分 SingleThreadedExecutor、MultiThreadedExecutor、EventsExecutor 的適用場景，知道何時搭配 MutuallyExclusive / Reentrant Callback Group 來避免 data race 或 deadlock

## 核心概念

### QoS（Quality of Service）

**精確定義**：QoS 是 ROS 2 透過底層 DDS 中介軟體暴露的**網路層資料傳輸策略集合**，用來控制 topic 通訊的可靠性、時效性與資源消耗。每個 Publisher / Subscriber 各自宣告自己的 QoS profile，DDS 在配對時檢查相容性 — 不相容就**靜默不通訊**，不會報錯。

**七大策略一覽**：

| 策略 | 選項 | 控制什麼 |
|------|------|----------|
| **Reliability** | `RELIABLE` / `BEST_EFFORT` | 掉包要不要重傳 |
| **Durability** | `VOLATILE` / `TRANSIENT_LOCAL` | 新 Subscriber 能不能收到歷史訊息 |
| **History** | `KEEP_LAST(N)` / `KEEP_ALL` | 佇列保留多少筆 |
| **Depth** | 整數 | 搭配 `KEEP_LAST` 的佇列深度 |
| **Deadline** | 時間 | 多久內必須收到下一筆，超時觸發事件 |
| **Liveliness** | `AUTOMATIC` / `MANUAL_BY_TOPIC` | Publisher 是否還活著的偵測方式 |
| **Lifespan** | 時間 | 訊息過期時間，過了就丟棄 |

**相容性鐵律**：Publisher 的品質 $\geq$ Subscriber 的要求才能配對成功。

$$
\text{Pub: BEST\_EFFORT} + \text{Sub: RELIABLE} = \text{靜默失敗（不會有任何錯誤訊息）}
$$

**物理意義**：Subscriber 要求「保證送達」但 Publisher 只提供「盡力就好」— DDS 認為品質不夠，直接拒絕配對。

**常用 Preset**：

| Preset | Reliability | Durability | History | 典型用途 |
|--------|------------|------------|---------|----------|
| `SensorDataQoS` | Best-effort | Volatile | Keep last 5 | LiDAR、Camera — 掉幀沒關係，要最新的 |
| `SystemDefaultsQoS` | Reliable | Volatile | Keep last 10 | 一般 topic |
| `ServicesQoS` | Reliable | Volatile | Keep last 10 | Service call |
| `ParameterEventsQoS` | Reliable | Transient local | Keep last 1000 | 參數變更通知 |

### Executor（執行器）

**精確定義**：Executor 是 ROS 2 中負責**從 DDS 取出就緒的 callback 並分配到 thread 執行**的排程器。它決定了 timer callback、subscription callback、service callback 等「誰先跑、能不能同時跑」。

| Executor 類型 | 行為 | 適用場景 |
|---------------|------|----------|
| `SingleThreadedExecutor` | 一個 thread 依序執行所有 callback | 最簡單、零並發風險 |
| `StaticSingleThreadedExecutor` | 啟動時掃一次拓撲、之後不再查詢 → CPU 更低 | 拓撲固定的即時系統 |
| `MultiThreadedExecutor` | Thread pool 平行執行 callback | 需要並行處理（感知 pipeline） |
| `EventsExecutor` | Event-driven 取代 WaitSet polling → 延遲更低 | 低延遲即時系統（ROS 2 Humble+） |

### Callback Group（回呼群組）

**精確定義**：Callback Group 是 Executor 用來決定「哪些 callback 之間能不能同時執行」的分組機制。

| 類型 | 行為 | 何時用 |
|------|------|--------|
| `MutuallyExclusiveCallbackGroup` | 同組的 callback **互斥**，不會同時跑 → 免鎖 | 共享資料的 timer + subscription |
| `ReentrantCallbackGroup` | 同組的 callback **可平行** → 需自行加鎖 | 獨立且需要最大吞吐量的 callback |

**在感知 → 規劃 → 控制閉環的位置**：

- **QoS = 網路資料包的底層濾網**：決定感知資料（LiDAR、Camera）能不能可靠且即時地送到規劃 / 控制 node
- **Executor = CPU 計算資源的調度大腦**：決定控制迴圈能不能在嚴格時限內完成 callback
- **一句話**：「QoS 掌控網路品質、Executor 掌控算力排程」
- 輸入：DDS 層的配置參數
- 輸出：穩定的資料流 + 確定性的 callback 排程
- 下游：所有依賴 topic 通訊與即時性的 node（SLAM、Navigation、Controller）

<details>
<summary>深入：QoS 相容性矩陣與 DDS 配對機制</summary>

### 相容性矩陣（Reliability × Durability）

| | Sub: VOLATILE | Sub: TRANSIENT_LOCAL |
|---|---|---|
| **Pub: VOLATILE** | 相容 | 不相容 |
| **Pub: TRANSIENT_LOCAL** | 相容 | 相容 |

| | Sub: BEST_EFFORT | Sub: RELIABLE |
|---|---|---|
| **Pub: BEST_EFFORT** | 相容 | **不相容** |
| **Pub: RELIABLE** | 相容 | 相容 |

規則：**Pub 提供的品質必須 ≥ Sub 要求的品質**。就像合約 — 甲方要求保證送達，乙方只承諾盡力就好，合約就簽不成。

### DDS 配對流程

1. Publisher 和 Subscriber 各自向 DDS 註冊自己的 QoS profile
2. DDS Discovery Protocol 發現對方後，比對 QoS 相容性
3. **全部策略都相容** → 建立通訊管道
4. **任一策略不相容** → 靜默拒絕，觸發 `IncompatibleQosEvent`（但預設沒人監聽）
5. 診斷指令：`ros2 topic info /scan -v` 可看到 Publisher 與 Subscriber 各自的 QoS profile

### Deadline 與 Liveliness 的即時應用

```
Deadline = 100ms：如果 100ms 內沒收到新訊息 → 觸發 deadline_missed callback
→ 用途：偵測感測器掉線、通訊中斷

Liveliness = MANUAL_BY_TOPIC, lease_duration = 500ms：
Publisher 必須每 500ms 主動 assert_liveliness()
→ 用途：確認控制器還活著，不只是 DDS 層面的心跳
```

</details>

<details>
<summary>深入：Executor 內部排程機制與 WaitSet vs Event-driven 比較</summary>

### WaitSet 模型（傳統 Executor）

```
while running:
    waitset.wait(timeout)          // 阻塞等待任何 callback 就緒
    ready_list = get_ready_callbacks()  // 掃描所有 subscription / timer / service
    for cb in ready_list:
        execute(cb)                // 依序或分派到 thread pool
```

**問題**：每次 `wait()` 返回後要遍歷所有 entity 檢查誰就緒 — entity 數量多時 CPU 浪費大。

### EventsExecutor 模型（ROS 2 Humble+）

```
while running:
    event = event_queue.pop()      // 只處理有事件的 callback
    execute(event.callback)
```

**改進**：DDS 中介層直接推送事件到 queue，不需要輪詢掃描。延遲降低、CPU 使用率下降。

### StaticSingleThreadedExecutor 的優化

一般 `SingleThreadedExecutor` 每次 spin 都會重新查詢 node 上有哪些 subscription / timer。`StaticSingleThreadedExecutor` 在 `spin()` 開始時只掃一次，之後用快取的拓撲 — 適合拓撲不會動態變化的系統。

### Callback Group 與 Executor 的交互

```
MultiThreadedExecutor + MutuallyExclusiveCallbackGroup:
  Thread 1 正在跑 group_A 的 callback_1
  → Thread 2 想跑 group_A 的 callback_2
  → Executor 阻擋：同組互斥，等 callback_1 結束

MultiThreadedExecutor + ReentrantCallbackGroup:
  Thread 1 正在跑 group_B 的 callback_1
  → Thread 2 想跑 group_B 的 callback_2
  → Executor 允許：同組可重入，兩者平行跑
  → 但如果 callback_1 和 callback_2 共享變數 → DATA RACE!
```

**黃金法則**：共享資料 → MutuallyExclusive（免鎖）；獨立工作 → Reentrant（最大吞吐）。

</details>

## 直覺理解

| 概念 | 類比 |
|------|------|
| QoS | **快遞 SLA 合約** — 你可以選「普通件（Best-effort）」或「保證送達（Reliable）」；收件方要求保證送達、寄件方只提供普通件 → 快遞公司拒接這筆單 |
| Durability | **公佈欄 vs 即時廣播** — Transient local 像公佈欄（新來的人看得到公告）；Volatile 像廣播（沒在現場就錯過了） |
| Executor | **餐廳外場經理** — 決定哪個服務生先上哪桌的菜、能不能同時上兩桌 |
| Callback Group | **餐廳防撞規則** — Mutually Exclusive = 同一條走道一次只能過一個人；Reentrant = 各走各的但碗盤可能撞到 |
| Deadline | **披薩 30 分鐘送達保證** — 超時就觸發告警、自動處理 |

**模擬器 / 工具觀察**：

- **Gazebo + Nav2**：把 LiDAR topic 的 QoS 從 `SensorDataQoS` 改成 `RELIABLE` → 觀察 costmap 更新延遲暴增（因為掉包重傳造成隊頭阻塞）
- **`ros2 topic info /scan -v`**：即時看 Publisher 和 Subscriber 的 QoS profile，一眼抓出不相容
- **`ros2 doctor`**：檢查整個系統的 QoS 不相容警告
- **`rqt_graph` + `htop`**：用 MultiThreadedExecutor 後觀察 CPU 核心使用率是否均勻分散

## 實作連結

**三個典型工程場景**：

1. **感知 pipeline QoS 配置**：LiDAR / Camera 用 `SensorDataQoS`（Best-effort + Volatile），掉幀不重傳、只保留最新 N 筆；控制指令用 `RELIABLE` + 小 depth，確保指令不丟失。

2. **MultiThreadedExecutor + Callback Group 隔離**：100 Hz timer callback 和 30 Hz subscription callback 共享一個 `shared_state` → 放進同一個 `MutuallyExclusiveCallbackGroup`，Executor 自動互斥、不用加鎖。

3. **即時控制 node 的 Executor 選型**：用 `StaticSingleThreadedExecutor` + CPU 綁定（`taskset`），拓撲啟動時掃一次就不再變、polling 開銷最低，配合 `SCHED_FIFO` 達到硬即時。

```cpp
// 場景 2：MultiThreadedExecutor + Callback Group 骨架
#include <rclcpp/rclcpp.hpp>

class SensorFusionNode : public rclcpp::Node {
public:
  SensorFusionNode() : Node("sensor_fusion") {
    // 共享資料的 callback 放同一個 MutuallyExclusive group → 免鎖
    auto group = create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions opts;
    opts.callback_group = group;

    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", rclcpp::SensorDataQoS(), 
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          shared_state_.latest_scan = msg;  // 安全：同組互斥
        }, opts);

    timer_ = create_wall_timer(10ms, [this]() {
      if (shared_state_.latest_scan) {
        process(shared_state_.latest_scan);  // 安全：同組互斥
      }
    });
    // 注意：timer 也要綁到同一個 group
  }
private:
  struct { sensor_msgs::msg::LaserScan::SharedPtr latest_scan; } shared_state_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorFusionNode>();
  // MultiThreadedExecutor 才會平行跑不同 group 的 callback
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
}
```

<details>
<summary>深入：完整 QoS 配置 + Deadline 監控實作（C++）</summary>

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class QoSDemoNode : public rclcpp::Node {
public:
  QoSDemoNode() : Node("qos_demo") {
    // 自訂 QoS profile
    rclcpp::QoS custom_qos(10);  // depth = 10
    custom_qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    custom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
    custom_qos.deadline(std::chrono::milliseconds(100));  // 100ms deadline
    custom_qos.liveliness(rclcpp::LivelinessPolicy::Automatic);
    custom_qos.lifespan(std::chrono::seconds(1));

    // Subscription 選項：設定 deadline / liveliness 事件 callback
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.event_callbacks.deadline_callback =
      [this](rclcpp::QOSDeadlineRequestedInfo& info) {
        RCLCPP_WARN(get_logger(),
          "Deadline missed! total: %d, delta: %d",
          info.total_count, info.total_count_change);
        // 觸發感測器掉線處理：切換到備用數據源或進入安全模式
      };

    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", custom_qos,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          process_scan(msg);
        }, sub_opts);
  }

private:
  void process_scan(sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // 處理邏輯
  }
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<QoSDemoNode>());
  rclcpp::shutdown();
}
```

### Python 版 QoS 配置

```python
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import LaserScan

class QoSDemoNode(Node):
    def __init__(self):
        super().__init__('qos_demo')

        # 方法 1：使用 preset
        self.sub_sensor = self.create_subscription(
            LaserScan, '/scan',
            self.scan_callback,
            QoSPresetProfiles.SENSOR_DATA.value  # Best-effort, Volatile, depth=5
        )

        # 方法 2：自訂 profile
        custom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub_custom = self.create_subscription(
            LaserScan, '/scan2',
            self.scan_callback,
            custom_qos
        )

    def scan_callback(self, msg):
        self.get_logger().info(f'Got scan with {len(msg.ranges)} rays')
```

### Executor 配置範例

```python
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

class MultiGroupNode(Node):
    def __init__(self):
        super().__init__('multi_group')
        
        # 共享資料的 callback → MutuallyExclusive
        self.critical_group = MutuallyExclusiveCallbackGroup()
        # 獨立工作的 callback → Reentrant
        self.parallel_group = ReentrantCallbackGroup()

        self.sub = self.create_subscription(
            LaserScan, '/scan', self.scan_cb,
            10, callback_group=self.critical_group)
        self.timer = self.create_timer(
            0.01, self.control_cb,
            callback_group=self.critical_group)
        self.sub_camera = self.create_subscription(
            Image, '/camera', self.camera_cb,
            10, callback_group=self.parallel_group)

def main():
    rclpy.init()
    node = MultiGroupNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor.spin()
```

</details>

## 常見誤解

1. **以為 QoS 不配也沒事** — ROS 2 預設 QoS 和 ROS 1 不同。ROS 1 的 topic 全部 Reliable + TCP；ROS 2 的 `ros2 topic pub` 預設 Reliable，但很多 driver 用 `SensorDataQoS`（Best-effort）。**結果**：你的 Subscriber 用預設（Reliable），driver Publisher 用 Best-effort → **靜默收不到資料**，沒有任何錯誤訊息。**避開**：`ros2 topic info /scan -v` 先看 QoS，再配對。

2. **在高頻感測器上用 Reliable QoS** — LiDAR 30 Hz、Camera 60 Hz，Reliable 模式每次掉包都重傳 → 隊頭阻塞（head-of-line blocking）。後續正常封包卡在重傳佇列後面 → **costmap 延遲從 33ms 暴增到 100ms+**。**避開**：感測器 topic 一律用 `SensorDataQoS`（Best-effort），寧可掉一幀也不要全部延遲。

3. **以為 MultiThreadedExecutor 自動安全** — 開了 MultiThreadedExecutor 但所有 callback 都在 default group（Mutually Exclusive）→ 等同 SingleThreaded，完全沒有平行效果。或者用了 Reentrant 但共享了全域變數 → **data race**。**避開**：明確規劃 callback group — 共享資料 → MutuallyExclusive（免鎖）、獨立工作 → Reentrant（最大吞吐）。

4. **Callback Group 設錯導致 deadlock** — 一個 MutuallyExclusive group 裡的 timer callback 呼叫了 service client 的 `call()`（同步），而 service server 的 callback 也在同一個 group → timer 等 service 回應、service 等 timer 結束 → **自己等自己**。**避開**：service client 和 service server 放不同 group，或改用 `async_send_request()`。

## 練習題

<details>
<summary>Q1（中）：Nav2 costmap 更新延遲從 33ms 暴增到 100ms，怎麼排查？</summary>

**分析推理**：
1. **定位現象**：costmap 更新延遲 = LiDAR → costmap node 的資料通路出問題
2. **第一步查 QoS**：`ros2 topic info /scan -v`，看 Publisher（LiDAR driver）和 Subscriber（costmap node）的 QoS profile
3. **最常見原因**：LiDAR driver 發布 `Best-effort`，costmap Subscriber 用 `Reliable` → 不相容 → 靜默失敗。或者都用 `Reliable` → 掉包重傳造成隊頭阻塞
4. **解法**：costmap 的 Subscriber 改用 `SensorDataQoS`（Best-effort + Keep last 5 + Volatile）。掉一幀 LiDAR 對 costmap 完全可接受，但 100ms 延遲會讓避障反應不及
5. **驗證**：改完後 `ros2 topic delay /scan` 確認延遲降回正常值

**面試官想聽到**：「高頻感測器 topic 用 Best-effort 是鐵律 — Reliable 的重傳機制在 30 Hz 以上會造成隊頭阻塞，犧牲少量掉幀換取穩定低延遲。」

</details>

<details>
<summary>Q2（中）：100 Hz timer + 30 Hz subscription 偶發錯誤值，懷疑 data race，怎麼修？</summary>

**分析推理**：
1. **確診**：timer callback 和 subscription callback 同時存取 `shared_state_` → MultiThreadedExecutor 下兩個 thread 可能同時跑 → data race
2. **檢查 callback group**：如果用了 default group（MutuallyExclusive），理論上不該 race — 但如果 timer 和 subscription 在不同的 default group 或沒有指定 group → 可能被分到不同 group
3. **修法**：顯式建立一個 `MutuallyExclusiveCallbackGroup`，把 timer 和 subscription **都**綁上去
4. **為什麼不用 mutex**：MutuallyExclusive group 讓 Executor 在排程層就保證互斥，零鎖開銷、零 priority inversion 風險
5. **驗證**：用 ThreadSanitizer（`-fsanitize=thread`）編譯跑一遍，確認無 race warning

**面試官想聽到**：「ROS 2 的 Callback Group 是 Executor 層的互斥機制，比 mutex 更輕量且不會有 priority inversion — 共享狀態的 callback 放同一個 MutuallyExclusive group 就免鎖了。」

</details>

<details>
<summary>Q3（難）：多台機器人透過 WiFi 通訊不穩定，topic 經常收不到，怎麼架構？</summary>

**分析推理**：
1. **問題根源**：ROS 2 預設用 DDS multicast discovery — WiFi 網段上多台機器人 → multicast 廣播風暴 + WiFi multicast 本身不可靠
2. **Domain ID 隔離**：每台機器人用不同 `ROS_DOMAIN_ID`（0–232），避免不相關的 topic 互相干擾
3. **Discovery Server 模式**：改用 Fast DDS Discovery Server — 用一台機器當中央 discovery server，其他機器人只跟 server unicast 通訊，消除 multicast
4. **QoS 策略調整**：跨機器人的控制指令用 `RELIABLE` + 適當 depth；感測器資料用 `BEST_EFFORT`（WiFi 掉包很正常，重傳只會更慢）
5. **進階**：考慮 Zenoh 作為 DDS 替代（ROS 2 Iron+），原生支援 WiFi / WAN 場景，自動處理重連

**面試官想聽到**：「多機器人 WiFi 場景的三板斧 — Domain ID 隔離不相關 topic、Discovery Server 消除 multicast、QoS 分級（控制 Reliable、感測器 Best-effort）。進階可以看 Zenoh。」

</details>

<details>
<summary>Q4（難）：設計一個需要硬即時保證的 1 kHz 控制 node，Executor 怎麼選？</summary>

**分析推理**：
1. **需求**：1 kHz = 1ms 週期，callback 必須在 deadline 內完成且 jitter 極低
2. **Executor 選擇**：`StaticSingleThreadedExecutor` — 啟動時掃一次拓撲，之後零查詢開銷；不像 `SingleThreadedExecutor` 每次 spin 都重新掃描
3. **CPU 綁定**：`taskset -c 3 ros2 run ...` 把 node 綁到專用核心，避免 OS 遷移 thread 導致 cache miss
4. **即時調度**：設定 `SCHED_FIFO` + 高優先級（需要 `sudo` 或 `CAP_SYS_NICE`），確保 OS 不會搶佔控制 callback
5. **Callback Group**：只有一個 MutuallyExclusive group（SingleThreaded 天然互斥），重點在確保 callback 內部不做任何阻塞操作（禁止 `new`、禁止 mutex、禁止同步 service call）
6. **QoS Deadline**：設 deadline = 2ms，超時觸發告警 → 監控系統即時性退化
7. **驗證**：用 `cyclictest` 測量系統 jitter、`ros2 topic delay` 監控通訊延遲

**面試官想聯到**：「硬即時控制 = StaticSingleThreadedExecutor + CPU affinity + SCHED_FIFO + 零 heap 分配 + Deadline QoS 監控 — 從 Executor 到 OS 到記憶體全面封鎖不確定性。」

</details>

## 面試角度

1. **QoS 相容性是 ROS 2 最常見的「靜默 bug」** — 面試官測你有沒有踩過坑。**帶出**：「ROS 2 兩個 node 連不上時，我第一步不是查網路、而是 `ros2 topic info -v` 比對 QoS profile — 因為 ROS 2 最常見的通訊失敗就是 Reliable/Best-effort 不匹配，而且不會報錯。」

2. **感測器一律 Best-effort 是工程鐵律** — 測你對即時系統的理解。**帶出**：「高頻感測器（LiDAR 30 Hz、Camera 60 Hz）我一定用 SensorDataQoS — Reliable 的重傳機制會造成隊頭阻塞，寧可掉一幀也不要全部延遲。控制指令則反過來、一定 Reliable。」

3. **Executor 隔離即時性** — 測你的系統架構能力。**帶出**：「即時控制 node 我會用 StaticSingleThreadedExecutor + CPU 綁定 + SCHED_FIFO，拓撲固定後不再查詢、polling 開銷最低；感知 pipeline 另開 MultiThreadedExecutor 跑在不同核心。」

4. **Callback Group 免鎖設計** — 測你對並發的理解深度。**帶出**：「需要共享狀態的 callback 我不加 mutex — 直接放進同一個 MutuallyExclusiveCallbackGroup，Executor 在排程層就保證互斥，零鎖開銷也不怕 priority inversion。」

5. **Component + 零拷貝是進階加分項** — 測你知不知道 ROS 2 的效能上限在哪。**帶出**：「多個 node 載入同一個 process（Component 容器），搭配 intra-process communication + unique_ptr → 同 process 內零拷貝通訊，感知 pipeline 吞吐量可以翻倍。」

## 延伸閱讀

- **ROS 2 官方文件：About Quality of Service settings** — QoS 七大策略的權威定義與相容性矩陣，第一手資料
- **ROS 2 Design Documents: Executors** — Executor 和 WaitSet 的設計理念與演進，理解為什麼會有 EventsExecutor
- **Fast DDS Discovery Server 文件** — 多機器人 / WiFi 場景的 discovery 架構，消除 multicast 依賴
- **《ROS 2 入門》Chapter 10：QoS 與 DDS 設定** — 系統性教學 QoS 的工程應用
- **ROS 2 Zenoh RMW** — DDS 的替代方案，原生支援 WiFi / WAN / 多機器人，ROS 2 Iron+ 可用
- **ros2_tracing** — 用 LTTng 追蹤 callback 執行時間與調度延遲，量化 Executor 效能的工具
