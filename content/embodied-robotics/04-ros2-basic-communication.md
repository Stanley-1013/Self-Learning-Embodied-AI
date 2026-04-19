---
title: "ROS 2 節點與核心通信機制"
prerequisites: []
estimated_time: 45
difficulty: 3
tags: ["ros2", "communication", "topic", "service", "action", "dds"]
sidebar_position: 4
---

# ROS 2 節點與核心通信機制

## 你將學到

- 能精確定義 Node、Topic、Service、Action 四大通信原語，面試時兩句講清楚各自適用場景
- 遇到「Subscriber 收不到資料」「callback 延遲飆高」等問題，知道從 QoS 不相容、Executor 阻塞逐步排查
- 判斷何時用 Topic（高頻感測/控制）、何時用 Service（一次性查詢）、何時用 Action（長時間任務 + feedback）

## 核心概念

### Node（節點）

ROS 2 的**基本計算單元**，封裝一組功能：接收感測器資料、發布控制指令、提供服務等。一個 process 可以載入多個 Node（Component 容器化），共用同一 process 的 Node 能啟用 **intra-process communication** 實現 zero-copy。

### Topic（主題）— Pub/Sub 非同步通信

**發布者把訊息丟進主題，所有訂閱者都收到** — 1-to-N 非同步解耦。發布者和訂閱者互相不認識，只靠主題名稱匹配。典型用途：感測器資料流（`/scan`、`/camera/image_raw`）、控制指令（`/cmd_vel`）。

### Service（服務）— Request/Response 同步通信

**Client 發請求，Server 回應一次** — 一對一、阻塞式。適合**一次性查詢**：「地圖存好了嗎？」「切換控制模式」。不適合高頻資料流（每次都要等回應，延遲不穩定）。

### Action（動作）— 長時間任務 + Feedback + Cancel

建在 Topic + Service 之上的複合協定：Client 發 goal → Server 持續回傳 feedback → 最終回傳 result，途中可 cancel。適合**導航到目標點、機械臂軌跡執行、長時間掃描**。

### DDS（Data Distribution Service）

ROS 2 的底層中介軟體，使用 RTPS（Real-Time Publish-Subscribe）協定。DDS 提供 **QoS（Quality of Service）策略**讓開發者控制通信行為：可靠性、歷史深度、延遲預算等。常見實作：Fast DDS、Cyclone DDS。

### Executor（執行器）

負責排程和呼叫 callback 的引擎：

| Executor 類型 | 行為 | 適用場景 |
|--------------|------|---------|
| `SingleThreadedExecutor` | 所有 callback 排隊在同一 thread | 簡單 node、避免資料競爭 |
| `MultiThreadedExecutor` | Thread pool 並行呼叫不同 callback | 需要並行處理多個 Topic |
| `StaticSingleThreadedExecutor` | 靜態分析 callback，省去動態查找開銷 | 即時性敏感、callback 數量固定 |

**在感知 → 規劃 → 控制閉環的位置**：
- Topic、Service、Action **不是某個節點，是串接所有節點的通信基礎設施**
- **感知 → 規劃**：LiDAR / Camera 資料透過 Topic 高頻發布（10–100 Hz），規劃 Node 訂閱
- **規劃 → 控制**：Navigation 透過 Action 發導航 goal，控制 Node 執行軌跡
- **一次性查詢**：Service 處理地圖儲存、模式切換等低頻請求
- **QoS 策略**決定每條通道的可靠性保證：感測器用 Best-effort（丟幀可容忍、要低延遲），指令用 Reliable（不能丟）

### 最少夠用的 API

**QoS Profile 預設**：

$$
\text{SensorDataQoS} = \{\text{Reliability: BestEffort, History: KeepLast(5), Durability: Volatile}\}
$$

物理意義：感測器資料流量大、允許丟幀，只保留最新 5 筆，不為遲到的 Subscriber 保留歷史。

$$
\text{ServicesQoS} = \{\text{Reliability: Reliable, Durability: Volatile}\}
$$

物理意義：Service 呼叫不能丟封包（一問一答必須可靠），但不需要持久化。

**Publisher / Subscriber 骨架**：

```cpp
// Publisher（rclcpp）
auto pub = node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS());
pub->publish(scan_msg);

// Subscriber
auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", rclcpp::SensorDataQoS(),
    [](sensor_msgs::msg::LaserScan::SharedPtr msg) { /* callback */ });
```

**Service Client 非同步呼叫**：

```cpp
auto client = node->create_client<std_srvs::srv::SetBool>("/set_mode");
auto future = client->async_send_request(request);
// 非阻塞：用 callback 或 spin_until_future_complete 處理回應
```

<details>
<summary>深入：DDS QoS 策略完整對照與相容性規則</summary>

### QoS 相容性矩陣

DDS 的 Pub/Sub QoS 必須**相容**才能通信。核心規則：

| 策略 | Publisher | Subscriber | 相容條件 |
|------|-----------|------------|---------|
| Reliability | Best-effort | Best-effort | 相容 |
| Reliability | Reliable | Reliable | 相容 |
| Reliability | Best-effort | Reliable | **不相容** — Sub 要求 Reliable 但 Pub 只給 Best-effort |
| Reliability | Reliable | Best-effort | 相容 — Sub 降級接受 |
| Durability | Volatile | Transient-local | **不相容** — Sub 要歷史但 Pub 不保留 |

### 常見 QoS 組合

```cpp
// 感測器高頻資料（允許丟幀，低延遲）
rclcpp::SensorDataQoS()
// = {best_effort, volatile, keep_last(5)}

// 控制指令（不能丟）
rclcpp::QoS(10).reliable()

// 地圖資料（晚加入的 Subscriber 也要收到）
rclcpp::QoS(1).reliable().transient_local()

// TF static transform（永久保留）
rclcpp::QoS(100).reliable().transient_local().keep_all()
```

### 診斷不相容

```bash
# 查看 QoS 不相容事件
ros2 topic info /scan --verbose
# 看 "Incompatible QoS" 計數器
```

當 Publisher 是 Best-effort 而 Subscriber 用 Reliable 時，DDS 層**靜默拒絕連接**，`ros2 topic echo` 看不到任何資料，也不會報錯 — 這是最常見的「收不到資料」陷阱。

</details>

<details>
<summary>深入：Executor 與 Callback Group 的並行控制</summary>

### MultiThreadedExecutor 不是自動 thread-safe

`MultiThreadedExecutor` 用 thread pool 並行呼叫 callback，但**不幫你保護共享資料**。如果兩個 callback 都讀寫同一個 buffer，你會遇到 data race。

### Callback Group 類型

```cpp
// 1. MutuallyExclusiveCallbackGroup — 同 group 的 callback 不並行
auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

// 2. ReentrantCallbackGroup — 同 group 的 callback 可並行（你自己保護資料）
auto group = node->create_callback_group(
    rclcpp::CallbackGroupType::Reentrant);
```

### 設計模式

```
感測器 callback（高頻，可並行）→ ReentrantCallbackGroup
    ↓ 寫入共享 buffer
控制 callback（低頻，需獨佔）→ MutuallyExclusiveCallbackGroup
    ↓ 讀取共享 buffer

共享 buffer 用 lock-free queue 或 atomic pointer swap 保護
```

### StaticSingleThreadedExecutor 的即時優勢

在初始化時一次性掃描所有 callback，之後不做動態記憶體配置 — 適合硬即時控制 Node（配合 `SCHED_FIFO`）。

</details>

## 直覺理解

| 通信原語 | 類比 |
|---------|------|
| Topic | **廣播電台** — DJ 不管有沒有人在聽，持續放送；聽眾隨時調進來就收到 |
| Service | **打電話** — 撥號、等接通、對方回答、掛斷；一問一答 |
| Action | **叫外送** — 下單（goal）→ 外送員持續回報位置（feedback）→ 送達（result）→ 途中可取消 |
| QoS | **平信 vs 掛號** — Best-effort 是平信（快但可能丟），Reliable 是掛號（保證送達但慢一點） |
| Executor | **餐廳服務生** — SingleThreaded 一個服務生逐桌服務；MultiThreaded 多個服務生並行送餐，但要小心別搶同一盤菜 |
| DDS Discovery | **自助餐會場名牌** — 每個 Node 開機時自動廣播「我有什麼 Topic/Service」，其他 Node 看到名牌就自動配對 |

**模擬器 / 工具觀察**：
- **Gazebo + ROS 2**：啟動一個 LiDAR sensor plugin → `ros2 topic list` 看到 `/scan` → `ros2 topic hz /scan` 確認 10 Hz → `ros2 topic echo /scan` 即時看 LaserScan 資料
- **rqt_graph**：視覺化所有 Node 與 Topic 的連接關係，一眼看出哪些 Node 形成感知→規劃→控制閉環
- **ros2 doctor**：檢查系統健康，包括 QoS 不相容、丟幀率、通信延遲

## 實作連結

**三個典型工程場景**：

1. **LiDAR 感知 pipeline**：Scanner Node publish `/scan`（`SensorDataQoS`）→ SLAM Node subscribe 做地圖更新 → Obstacle Node subscribe 做障礙物偵測。兩個 Subscriber 獨立並行，Topic 天然支持 1-to-N。

2. **Nav2 導航系統**：規劃 Node 透過 Action Client 送 `NavigateToPose` goal → 控制 Node 持續回傳 feedback（目前位置、剩餘距離）→ 到達後回傳 result。途中遇障礙可 cancel 重新規劃。

3. **機械臂控制系統**：100 Hz 關節狀態用 Topic（`/joint_states`）→ 抓取指令用 Action（`/grasp`，有 feedback 回報抓取力道）→ 切換控制模式用 Service（`/set_control_mode`）。

```cpp
// Action Client 骨架：送導航 goal + 處理 feedback
auto action_client = rclcpp_action::create_client<NavigateToPose>(node, "navigate_to_pose");

auto goal_msg = NavigateToPose::Goal();
goal_msg.pose.pose.position.x = 3.0;  // 目標位置

auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
send_goal_options.feedback_callback = [](auto, auto feedback) {
    RCLCPP_INFO(/*...*/, "剩餘距離: %f", feedback->distance_remaining);
};
send_goal_options.result_callback = [](auto result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(/*...*/, "導航完成");
};

action_client->async_send_goal(goal_msg, send_goal_options);
```

<details>
<summary>深入：完整 Component Node 實作（C++ rclcpp，支援 intra-process zero-copy）</summary>

```cpp
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace perception {

class PointCloudFilter : public rclcpp::Node {
public:
  explicit PointCloudFilter(const rclcpp::NodeOptions& options)
    : Node("pointcloud_filter", options)
  {
    // 啟用 intra-process comm（同 process 的 Node 間 zero-copy）
    // NodeOptions 需設定 use_intra_process_comms(true)

    auto qos = rclcpp::SensorDataQoS();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/points", qos,
        [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
          // UniquePtr → 收到的是所有權轉移，不是 deep copy
          filter_and_publish(std::move(msg));
        });

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/filtered_points", qos);
  }

private:
  void filter_and_publish(sensor_msgs::msg::PointCloud2::UniquePtr msg) {
    // 就地修改（已拿到所有權，不影響其他 Subscriber）
    // ... 濾波邏輯 ...

    pub_->publish(std::move(msg));  // 零拷貝轉發
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

}  // namespace perception

RCLCPP_COMPONENTS_REGISTER_NODE(perception::PointCloudFilter)
```

**Launch 載入多個 Component 到同一 process**：

```python
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNode
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='perception',
                plugin='perception::PointCloudFilter',
                name='filter',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='slam',
                plugin='slam::SlamNode',
                name='slam',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )
    return LaunchDescription([container])
```

</details>

## 常見誤解

1. **以為 Topic 和 Service 可以隨便互換** — Topic 是非同步 1-to-N 廣播，丟出去就不管了；Service 是同步一問一答，Client 要等 Server 回應。用 Service 做 100 Hz 感測器串流 → 每幀都要等 response，延遲不穩定且 Server 掛掉就整條 pipeline 卡住。**原則**：高頻資料流 = Topic，一次性查詢 = Service，長任務 = Action。

2. **以為 `MultiThreadedExecutor` 自動 thread-safe** — 它只是讓不同 callback 在不同 thread 上跑，**不幫你保護共享資料**。兩個 callback 同時讀寫同一個 `std::vector` → data race → 偶發 crash 或產生垃圾資料。**避開**：用 `MutuallyExclusiveCallbackGroup` 把互斥的 callback 放同一組，或用 lock-free 資料結構。

3. **QoS 不相容時沒有任何錯誤訊息** — Publisher 用 Best-effort、Subscriber 用 Reliable → DDS 層**靜默拒絕配對**，`ros2 topic echo` 一片空白，開發者以為是 Node 沒啟動。**診斷**：`ros2 topic info /topic_name --verbose` 查看 QoS profile 是否相容。

4. **在 callback 裡做長時間計算** — callback 是被 Executor 呼叫的，SingleThreadedExecutor 下一個 callback 阻塞 = 所有 callback 都卡住。**避開**：長計算丟到另一個 thread，callback 只做「收資料 → 塞進 queue → 立即返回」。

## 練習題

<details>
<summary>Q1（中）：LiDAR Subscriber 啟動後 `ros2 topic echo /scan` 收不到任何資料，但 Publisher Node 確認在正常發布，怎麼排查？</summary>

**分析推理**：
1. **先確認 Topic 名稱是否一致**：`ros2 topic list` 看有沒有 `/scan`；typo 是最常見原因
2. **檢查 QoS 相容性**：`ros2 topic info /scan --verbose` → 看 Publisher 和 Subscriber 的 QoS profile
3. **最常見陷阱**：Publisher 用 `SensorDataQoS`（Best-effort），但 Subscriber 預設用 `Reliable` → DDS **靜默拒絕配對**
4. **修復**：Subscriber 改用 `rclcpp::SensorDataQoS()` 或 `rclcpp::QoS(10).best_effort()`
5. **其他可能**：DDS Domain ID 不同（`ROS_DOMAIN_ID` 環境變數不一致）→ 兩個 Node 在不同 DDS domain，互相看不見

**面試官想聽到**：一上來就問 QoS 相容性，不是亂猜網路問題。能說出 Best-effort vs Reliable 的靜默不相容行為。

</details>

<details>
<summary>Q2（中）：設計機械臂系統的通信架構 — 100 Hz 關節狀態回報、TCP 位置查詢、軌跡執行，分別選什麼通信機制？</summary>

**分析推理**：
1. **100 Hz 關節狀態**：高頻、持續性、1-to-N（控制器要看、UI 要看、logger 要看）→ **Topic**（`/joint_states`），QoS 用 `SensorDataQoS`
2. **TCP 位置查詢**：偶爾一次、需要確切回應 → **Service**（`/get_tcp_pose`），QoS 用 `ServicesQoS`（Reliable）
3. **軌跡執行**：長時間（數秒）、需要 feedback（目前在軌跡的百分比）、可能要 cancel（遇碰撞）→ **Action**（`/follow_joint_trajectory`）
4. **陷阱**：不要用 Topic 做軌跡執行 — 沒有 feedback 和 cancel 機制，規劃端不知道執行進度
5. **陷阱**：不要用 Service 做高頻狀態回報 — 每次都要等回應，100 Hz 根本來不及

**面試官想聽到**：不是死背答案，是能從「頻率 × 方向性 × 是否需要 feedback」三個維度推導出選擇。

</details>

<details>
<summary>Q3（難）：用 `MultiThreadedExecutor` 跑兩個 Subscriber callback，偶爾 crash，ThreadSanitizer 報 data race on shared buffer，怎麼修？</summary>

**分析推理**：
1. **確診**：兩個 callback 在不同 thread 同時讀寫同一個 `shared_buffer` → classic data race
2. **方案 A — Callback Group 隔離**：把兩個 Subscriber 放進同一個 `MutuallyExclusiveCallbackGroup` → Executor 保證同一時間只跑一個 → 問題解決但犧牲並行性
3. **方案 B — Lock-free 設計**：用 `std::atomic<BufferPtr>` 做 triple buffer / double buffer swap → 讀寫完全無鎖 → 保留並行性
4. **方案 C — 讀寫鎖**：`std::shared_mutex` — 多讀單寫 → 比 A 彈性但有鎖開銷
5. **選擇**：即時控制場景 → B（zero lock）；一般感知場景 → A（簡單安全）；讀多寫少 → C
6. **陷阱**：不要只加 `std::mutex` 就覺得沒事 — 1 kHz callback 被 mutex 阻塞 = jitter spike

**面試官想聽到**：能診斷 data race 來源（ThreadSanitizer），能分析多種修復方案的 trade-off，而不是只說「加鎖」。

</details>

<details>
<summary>Q4（難）：感知 pipeline 有 Camera → Filter → SLAM → Planner 四個 Node，每幀 1.2 MB RGBD @ 30 fps，怎麼最小化通信開銷？</summary>

**分析推理**：
1. **問題量化**：1.2 MB × 30 fps = 36 MB/s，若每個 Node 間 deep copy = 4 次 × 36 = 144 MB/s 記憶體頻寬浪費
2. **Component 容器化**：四個 Node 用 `rclcpp_components` 載入同一 process → 啟用 `use_intra_process_comms(true)`
3. **Zero-copy 所有權轉移**：Camera Node 用 `unique_ptr<Image>` publish → Filter 收到所有權（不複製）→ 處理後 `std::move` 發給 SLAM
4. **1-to-N 分岔點**：如果 SLAM 和 Obstacle 同時需要 Filter 的輸出 → 改用 `const shared_ptr`（一塊記憶體多個讀者）
5. **Loaned Message（進階）**：`pub->borrow_loaned_message()` 讓 DDS 中介軟體直接提供 buffer → 連 `make_unique` 的記憶體分配都省
6. **陷阱**：忘記在 Launch 設定 `use_intra_process_comms: True` → 即使在同一 process 也走 DDS 序列化/反序列化 → 效能倒退到跨 process 等級

**面試官想聽到**：Component 容器化 + intra-process zero-copy 的系統性解法，不是只說「用 shared memory」。

</details>

## 面試角度

1. **Topic / Service / Action 選型三維度** — 這是面試最基本也最常問的。**帶出**：「我用三個維度判斷：頻率高低、是否需要 feedback、方向性（1-to-N vs 1-to-1）。高頻感測器 = Topic，一次性查詢 = Service，長時間任務需要 feedback 和 cancel = Action。」

2. **QoS 相容性是 ROS 2 最隱蔽的坑** — 測的是實戰經驗。**帶出**：「我遇過 Subscriber 收不到資料的問題，第一步就是 `ros2 topic info --verbose` 檢查 QoS 相容性。最常見的是 Publisher Best-effort 對 Subscriber Reliable，DDS 靜默拒絕配對。」

3. **Component + Intra-process = 感知 pipeline 的標配** — 測的是系統設計能力。**帶出**：「對於影像/點雲這類大資料 pipeline，我會把多個 Node 用 Component 容器載入同一 process，啟用 intra-process comm，用 `unique_ptr` + `move` 做 zero-copy，把 36 MB/s 的拷貝開銷壓到零。」

4. **Executor 與 Callback Group 的即時性保證** — 測的是並行/即時思維。**帶出**：「MultiThreadedExecutor 不是自動 thread-safe，我會用 MutuallyExclusiveCallbackGroup 隔離互斥的 callback；在即時控制 Node 用 StaticSingleThreadedExecutor 避免動態配置開銷。」

5. **DDS Discovery 與跨機器通信** — 測的是部署實戰。**帶出**：「預設的多播 Discovery 在複雜網路拓撲（多網卡、Wi-Fi）會出問題，我會用 DDS Discovery Server 集中管理，減少網路負載並加速啟動。」

## 延伸閱讀

- **ROS 2 Design Documents — DDS & QoS** — 官方設計文件，解釋為什麼從 ROS 1 的自研通信層轉向 DDS，以及 QoS 策略的完整設計理據
- **ROS 2 官方文件：Intra-process Communication** — 學 Component 容器化 + `unique_ptr` 如何在 Node 間實現零拷貝，附 benchmark 數據
- **Nav2 Documentation** — ROS 2 導航框架的完整架構圖，是 Topic / Service / Action 綜合應用的最佳教科書
- **Fast DDS Documentation — Discovery Server** — 多機器人系統部署必學，解決預設多播 Discovery 在複雜網路的不穩定問題
- **Loaned Message API（ROS 2 Humble+）** — 跨 process 的 zero-copy 機制，讓 DDS 中介軟體直接提供 shared memory buffer
- **`ros2 topic`、`ros2 service`、`ros2 action` CLI 工具** — 日常開發最常用的 debug 工具，每個子命令都值得跑一遍
