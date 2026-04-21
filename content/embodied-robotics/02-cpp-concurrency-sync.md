---
title: "C++ 多執行緒與同步機制"
prerequisites: ["01-cpp-memory-optimization"]
estimated_time: 45
difficulty: 3
tags: ["cpp", "concurrency", "threading", "synchronization"]
sidebar_position: 2
---

# C++ 多執行緒與同步機制

## 你將學到

- 能精確區分 thread/process、mutex 家族、RAII 鎖策略，面試時兩分鐘講清楚 deadlock 四條件與防禦方式
- 遇到「ROS 2 node 跑幾小時後所有 topic 頻率歸零」情境，知道先懷疑 deadlock 並用 `gdb` + TSan 定位
- 判斷何時用 `lock_guard`（最輕）、何時用 `scoped_lock`（多鎖避 deadlock）、何時改用 lock-free queue（硬即時場景）

## 核心概念

### Thread vs Process

**Thread（執行緒）**：輕量級、**共享同一 process 的記憶體空間**。建立與切換成本低，但共享帶來 data race 風險。

**Process（行程）**：獨立位址空間，天生隔離。跨 process 通訊（IPC）靠 shared memory / pipe / socket，延遲高、開銷大。

**機器人系統的選擇**：ROS 2 預設每個 node 是一個 process（安全隔離），但效能敏感時可用 Component 容器化把多個 node 載入同一 process，啟用 intra-process zero-copy。

### std::jthread（C++20）

傳統 `std::thread` 忘記 `join()` 就 `std::terminate`。`jthread` 解決兩個痛點：
1. **Auto-joining**：解構時自動 `join()`，RAII 語意完整
2. **Cooperative cancellation**：內建 `stop_token`，讓執行緒優雅退出而非暴力 kill

### Mutex 家族

| 型別 | 用途 | 注意 |
|------|------|------|
| `std::mutex` | 最基本互斥鎖 | 同一 thread 重複 lock → 未定義行為 |
| `std::recursive_mutex` | 允許同一 thread 多次 lock | **通常是設計臭味**，掩蓋過度耦合 |
| `std::timed_mutex` | 支援 `try_lock_for` 超時 | 避免永久阻塞 |
| `std::shared_mutex` | 讀寫鎖：多讀單寫 | 適合讀多寫少場景（如共享地圖） |

### RAII 鎖策略

| 鎖 | 語意 | 場景 |
|----|------|------|
| `lock_guard` | 最輕量，scope 內上鎖/解鎖 | 單一 mutex、無需手動控制 |
| `unique_lock` | 可延遲鎖定、手動 unlock、搭配 condition_variable | 需要進階控制 |
| `scoped_lock` | **同時鎖多個 mutex，內部用 deadlock-free 演算法** | 多鎖場景首選 |

### Deadlock 四條件

同時滿足才會發生 deadlock：

1. **互斥（Mutual Exclusion）**：資源不可共享
2. **持有等待（Hold and Wait）**：持有一個鎖的同時等另一個
3. **不可剝奪（No Preemption）**：已持有的鎖不能被強制釋放
4. **循環等待（Circular Wait）**：A 等 B、B 等 A

**防禦**：打破任一條件。實務上最常用 `std::lock()` / `scoped_lock` 的 try-and-back-off 演算法，**打破「持有等待」**。

### Condition Variable

解決 busy-waiting（`while (!ready) {}` 吃爆 CPU）的標準工具。**必須搭配 `unique_lock`**。

```cpp
std::unique_lock<std::mutex> lk(mtx);
cv.wait(lk, [&]{ return data_ready; });  // while 語意，防 spurious wakeup
```

關鍵：`wait` 的第二個參數是 predicate lambda，內部展開為 `while (!pred()) { cv.wait(lk); }`。**用 `if` 會被 spurious wakeup 咬到**。

### Future / Promise / Async

非同步三件套：

| 元件 | 角色 |
|------|------|
| `std::promise` | 生產者：寫入結果 |
| `std::future` | 消費者：讀取結果（阻塞或輪詢） |
| `std::async` | 語法糖：自動建 promise + 回傳 future |

**陷阱**：`std::async` 的預設 launch policy 是 `launch::async | launch::deferred`，**不一定開新 thread** — 可能在 `get()` 時才同步執行。要確保並行，顯式傳 `std::launch::async`。

### Thread Pool

預先建立一組常駐 thread，任務透過 queue 分發 — 消除每次 `std::thread` 建立/銷毀的開銷（通常數十 $\mu s$）。

**Oversubscription 問題**：thread 數 > 核心數 → 頻繁 context switch → 效能劣化。經驗法則：thread 數 = CPU 核心數（計算密集）或 $2\times$ 核心數（IO 密集）。

### 閉環定位

**在感知 → 規劃 → 控制閉環的位置**：

- **感知 thread**：Camera / LiDAR callback，容忍一定延遲（~33 ms @ 30 Hz）
- **規劃 thread**：A* / RRT 等搜尋演算法，可能耗時 50-200 ms
- **控制 thread**：硬即時（1 kHz），**絕對不能被 mutex 阻塞**
- **通訊 thread**：ROS 2 executor 排程 callback

ROS 2 `MultiThreadedExecutor` + `CallbackGroup`：
- `MutuallyExclusiveCallbackGroup`：同 group 內的 callback **不會平行**執行 → 免手寫 mutex
- `ReentrantCallbackGroup`：callback 可平行，自行負責同步

$$
\text{控制延遲預算} = \frac{1}{f_{\text{control}}} = 1\text{ ms (@ 1 kHz)}
$$

mutex 一次 contention 可能 10-100 $\mu s$，嚴重時直接吃掉 10% 預算 → 硬即時路徑必須 lock-free。

<details>
<summary>深入：Lock-free Queue 原理與 Thread-safe Queue 實作</summary>

### Lock-free Queue

硬即時路徑不能用 mutex（priority inversion + 不可預測延遲）。Lock-free queue 用 `std::atomic` CAS（Compare-And-Swap）操作實現無鎖推入/彈出：

```cpp
// 概念骨架（單生產者單消費者 SPSC）
template<typename T, size_t N>
class SPSCQueue {
  std::array<T, N> buffer_;
  std::atomic<size_t> head_{0};  // 消費者讀
  std::atomic<size_t> tail_{0};  // 生產者寫

public:
  bool push(const T& item) {
    size_t t = tail_.load(std::memory_order_relaxed);
    size_t next = (t + 1) % N;
    if (next == head_.load(std::memory_order_acquire))
      return false;  // 滿了
    buffer_[t] = item;
    tail_.store(next, std::memory_order_release);
    return true;
  }

  bool pop(T& item) {
    size_t h = head_.load(std::memory_order_relaxed);
    if (h == tail_.load(std::memory_order_acquire))
      return false;  // 空的
    item = buffer_[h];
    head_.store((h + 1) % N, std::memory_order_release);
    return true;
  }
};
```

**為什麼 SPSC 夠用**：機器人系統常見模式是「感知 thread 寫、控制 thread 讀」— 天然的單生產者單消費者。

### Thread-safe Queue（帶 mutex 版本）

非即時路徑可以用 mutex + condition_variable 的安全版本：

```cpp
template<typename T>
class ThreadSafeQueue {
  std::queue<T> queue_;
  std::mutex mtx_;
  std::condition_variable cv_;

public:
  void push(T item) {
    {
      std::lock_guard<std::mutex> lk(mtx_);
      queue_.push(std::move(item));
    }
    cv_.notify_one();
  }

  std::shared_ptr<T> wait_and_pop() {
    std::unique_lock<std::mutex> lk(mtx_);
    cv_.wait(lk, [&]{ return !queue_.empty(); });
    auto result = std::make_shared<T>(std::move(queue_.front()));
    queue_.pop();
    return result;  // 回傳 shared_ptr：合併 front+pop 為單步，並提供強異常安全
  }
};
```

**設計要點**：真正要避免的 race 是「呼叫者先 `front()` 再 `pop()`」這兩步之間被其他 thread 搶先。把兩步放在同一個鎖下、並把結果包成 `shared_ptr` 回傳 — 也順便提供強異常安全保證（複製/移動出 queue 之後才改動 queue 狀態）。用 dummy node 技巧可進一步分離 head/tail 鎖，提升並發度。

</details>

<details>
<summary>深入：std::atomic 與 Memory Order 層級</summary>

### Memory Order 為什麼重要

現代 CPU 會亂序執行（out-of-order execution）、編譯器會重排指令。`std::atomic` 的 memory order 告訴編譯器和 CPU：「這些操作的順序**不能**被打亂」。

| Order | 保證 | 成本 | 用途 |
|-------|------|------|------|
| `relaxed` | 只保證原子性，不保證順序 | 最低 | 計數器、統計 |
| `acquire` | 讀取後的操作不會被重排到此之前 | 中 | 消費者端 |
| `release` | 寫入前的操作不會被重排到此之後 | 中 | 生產者端 |
| `acq_rel` | 同時 acquire + release | 中高 | CAS 操作 |
| `seq_cst` | 全序一致（預設） | 最高 | 簡單場景、不確定時用 |

### Acquire-Release 配對

```
Thread A (producer):          Thread B (consumer):
  data = 42;                    while (!flag.load(acquire));
  flag.store(true, release);    assert(data == 42);  // 保證看到 42
```

`release` 確保 `data = 42` 在 `flag.store` 之前完成；`acquire` 確保 `flag.load` 之後才讀 `data`。

### 為什麼不全用 seq_cst

`seq_cst` 在 ARMv8 上，**純 load/store 與 acquire/release 使用相同的 `LDAR`/`STLR` 指令、沒有額外 barrier 成本**；差異主要在 seq_cst 的 **RMW**（`fetch_add`、`exchange`）會多一道 `DMB ISH` 全屏障（~10-40 ns），以及 seq_cst 對編譯器排序的更嚴格限制會間接影響 pipeline。硬即時 1 kHz 迴圈中若有高頻 atomic RMW，降為 acquire-release 可省掉 DMB；一般 lock-free 結構 acquire-release 配對已足夠。（詳見 Ch03 memory order 對應表）

</details>

## 直覺理解

| 概念 | 類比 |
|------|------|
| Mutex | 洗手間門鎖 — 進去鎖門（`lock`），出來開門（`unlock`），同時只有一人能用 |
| Deadlock | 兩個小孩搶鼓：A 抓左鼓棒等右、B 抓右鼓棒等左 — 永遠打不了鼓 |
| Condition Variable | 火車站月台廣播 — 乘客不用每秒看時刻表（busy-wait），等廣播響了（`notify`）才起身 |
| Thread Pool | 計程車行 — 車隊固定大小常駐待命，乘客（任務）排隊上車，不用每次臨時買新車 |
| Lock-free Queue | 迴轉壽司 — 師傅放盤子（produce）、客人拿盤子（consume），中間靠輸送帶（atomic），不需要互相等 |
| `scoped_lock` | 銀行保險箱雙鑰匙 — 行員和客戶**同時**到場才開箱，避免一方卡住另一方 |

**模擬器 / 工具觀察**：

- **Deadlock 症狀**：`ros2 topic hz` 所有 topic 頻率歸零、CPU 使用率驟降（thread 全卡在等鎖）。用 `gdb -p <pid>` → `thread apply all bt` 看每個 thread 卡在哪個 `lock()`
- **Race condition 症狀**：Gazebo / RViz 中看到障礙物瞬間「跳」到另一位置（感知與規劃 thread 同時讀寫 occupancy grid）、規劃軌跡突然抖動
- **Priority inversion 症狀**：控制 jitter 從 < 50 $\mu s$ 暴增到數 ms → watchdog 觸發急停。低優先級 thread 持有 mutex 被高優先級控制 thread 等待

## 實作連結

**三個典型工程場景**：

1. **ROS 2 Timer + Subscriber 共寫 vector**：Timer callback 每 100 ms 讀 `std::vector<Obstacle>`，Subscriber callback 收到感測資料時寫入。**方案 A**：放同一個 `MutuallyExclusiveCallbackGroup`（ROS 2 保證不平行，免 mutex）。**方案 B**：用 `std::lock_guard<std::mutex>` 保護存取。

2. **感知 33 ms 更新 grid vs 規劃 50 ms A\***：感知寫 grid、規劃讀 grid，頻率不同。**Double buffer + atomic pointer swap**：感知寫到 back buffer → 寫完用 `std::atomic<Grid*>` swap → 規劃永遠讀 front buffer，零等待。搭配 `condition_variable` 通知規劃「有新資料」。

3. **4 核 ARM、6 nodes 即時部署**：控制 node 獨佔 1 核（`SCHED_FIFO` + `cpu_affinity`），感知/規劃/通訊共享剩餘 3 核。控制與規劃之間用 lock-free SPSC queue 傳軌跡指令。分離 executor，每個 executor 綁定不同 `CallbackGroup`。

```cpp
// ROS 2 MutuallyExclusiveCallbackGroup 免 mutex 骨架
auto group = create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

auto sub_options = rclcpp::SubscriptionOptions();
sub_options.callback_group = group;

auto sub = create_subscription<Obstacle>(
    "obstacles", 10,
    [this](const Obstacle::SharedPtr msg) {
      obstacles_.push_back(*msg);  // 安全：同 group 不平行
    },
    sub_options);

auto timer = create_wall_timer(
    100ms,
    [this]() {
      process(obstacles_);  // 安全：同 group 不平行
    },
    group);
```

<details>
<summary>深入：Double Buffer + Atomic Pointer Swap 完整實作</summary>

```cpp
#include <atomic>
#include <memory>
#include <mutex>
#include <condition_variable>

template<typename T>
class DoubleBuffer {
  // 兩塊 buffer，感知寫 back、規劃讀 front
  std::unique_ptr<T> buffers_[2];
  std::atomic<int> front_idx_{0};
  std::mutex notify_mtx_;
  std::condition_variable cv_;
  bool updated_ = false;

public:
  DoubleBuffer() {
    buffers_[0] = std::make_unique<T>();
    buffers_[1] = std::make_unique<T>();
  }

  // 感知 thread 呼叫：取得可寫的 back buffer
  T& back_buffer() {
    return *buffers_[1 - front_idx_.load(std::memory_order_acquire)];
  }

  // 感知 thread 寫完後呼叫：swap + 通知
  void swap_and_notify() {
    int old_front = front_idx_.load(std::memory_order_relaxed);
    front_idx_.store(1 - old_front, std::memory_order_release);
    {
      std::lock_guard<std::mutex> lk(notify_mtx_);
      updated_ = true;
    }
    cv_.notify_one();
  }

  // 規劃 thread 呼叫：取得最新的 front buffer（唯讀）
  const T& front_buffer() const {
    return *buffers_[front_idx_.load(std::memory_order_acquire)];
  }

  // 規劃 thread 呼叫：等待新資料
  void wait_for_update() {
    std::unique_lock<std::mutex> lk(notify_mtx_);
    cv_.wait(lk, [&]{ return updated_; });
    updated_ = false;
  }
};

// 使用範例
// 感知 thread（33 ms）
void perception_callback(DoubleBuffer<OccupancyGrid>& db,
                          const SensorData& data) {
  auto& grid = db.back_buffer();
  update_grid(grid, data);
  db.swap_and_notify();
}

// 規劃 thread（50 ms）
void planning_loop(DoubleBuffer<OccupancyGrid>& db) {
  while (running) {
    db.wait_for_update();
    const auto& grid = db.front_buffer();  // 零拷貝讀取
    auto path = a_star(grid, start, goal);
    publish_path(path);
  }
}
```

**為什麼不用 `shared_mutex`**：讀寫鎖在寫入時仍會阻塞所有讀者。Double buffer 讓讀寫完全不互斥 — 感知寫 back、規劃讀 front，atomic swap 只是一個指標交換（< 1 ns）。

</details>

## 常見誤解

1. **以為 `empty()` + `top()` 是原子操作** — `std::stack` 的 `empty()` 和 `top()` 是兩個獨立呼叫。thread A 檢查 `!empty()` 後、呼叫 `top()` 前，thread B 可能已經 `pop()` 了最後一個元素 → **未定義行為**。**避開**：用 thread-safe wrapper，把檢查和存取合併成單一 atomic 操作。

2. **用 `recursive_mutex` 解決 double-lock** — 表面上修好了 deadlock，實際上掩蓋了設計問題：函式呼叫鏈中重複鎖同一個 mutex，通常代表職責不清、耦合過深。**避開**：重構呼叫鏈，把需要鎖的邏輯集中到一層。

3. **`condition_variable::wait` 用 `if` 而不是 `while`** — OS 可能發出 **spurious wakeup**（無 `notify` 也醒來）。用 `if` 檢查會在條件未真正滿足時繼續執行。**避開**：永遠用帶 predicate 的 `wait(lk, pred)` 或 `while (!pred) { cv.wait(lk); }`。

4. **以為 `std::async` 一定開新 thread** — 預設 launch policy 包含 `deferred`，可能在 `future.get()` 時同步執行。**避開**：需要並行時顯式寫 `std::async(std::launch::async, ...)`。

5. **即時 callback 裡用 mutex** — 高優先級控制 thread 等低優先級 thread 釋放 mutex → **priority inversion**。Linux pthread mutex 預設**不啟用** priority inheritance（但 `pthread_mutexattr_setprotocol(..., PTHREAD_PRIO_INHERIT)` 可開啟，PREEMPT_RT 常用此招）。**避開**：硬即時路徑用 lock-free queue 或 atomic pointer swap；若必須用 mutex 則啟用 `PTHREAD_PRIO_INHERIT`。

## 練習題

<details>
<summary>Q1（簡單）：ROS 2 的 Timer callback 和 Subscriber callback 同時讀寫一個 std::vector，怎麼保護？</summary>

**分析推理**：

1. **問題識別**：兩個 callback 在 `MultiThreadedExecutor` 下可能平行執行 → 同時讀寫 `vector` = data race
2. **方案 A — CallbackGroup**：把 Timer 和 Subscriber 放進同一個 `MutuallyExclusiveCallbackGroup`。ROS 2 保證同 group 的 callback 不會同時執行 → 免寫任何 mutex
3. **方案 B — lock_guard**：若兩個 callback 必須在不同 group（例如為了和其他 callback 平行），用 `std::lock_guard<std::mutex>` 保護 vector 存取
4. **不該做的**：用 `recursive_mutex`（掩蓋問題）、用 `SingleThreadedExecutor`（犧牲整體並行度）
5. **選擇依據**：方案 A 更簡單、更安全、ROS 2 原生支持，優先選擇

**面試官想聽到**：理解 ROS 2 `CallbackGroup` 機制可以取代手動 mutex，展示對框架工具的活用。

</details>

<details>
<summary>Q2（中）：機器人跑了幾小時後突然所有 topic 頻率歸零，CPU 使用率下降，怎麼診斷？</summary>

**分析推理**：

1. **症狀解讀**：所有 topic 歸零 + CPU 下降 = thread 全部阻塞 → 高度懷疑 **deadlock**（非 crash，因為 process 還在）
2. **即時診斷**：
   - `gdb -p <pid>` → `thread apply all bt`：看每個 thread 的 call stack，找到哪些 thread 卡在 `pthread_mutex_lock`
   - 如果兩個 thread 各持有對方需要的鎖 → 確認循環等待
3. **事後防護**：
   - 用 TSan（`-fsanitize=thread`）重新編譯跑壓力測試，提前抓 data race 和 lock-order 問題（2-10x 慢）
   - 或用 Helgrind（Valgrind 工具），不需重編但 20-50x 慢
4. **修復**：
   - 統一鎖順序，或改用 `std::scoped_lock` 同時鎖多個 mutex（內部 try-and-back-off）
   - 評估是否能把互斥的 callback 放進**同一個** `MutuallyExclusiveCallbackGroup` 來消除手動 mutex（注意：不同 ME group 之間的 callback 仍可能平行）
5. **陷阱**：deadlock 可能需要幾小時特定的 callback 交錯順序才觸發 → 單元測試抓不到，需要長時間壓力測試

**面試官想聽到**：診斷三步驟（gdb bt → TSan → 修復鎖順序），並知道 deadlock 可能需要長時間運行才會觸發。

</details>

<details>
<summary>Q3（中-難）：感知以 33 ms 更新 occupancy grid，規劃每 50 ms 跑一次 A*，怎麼設計資料傳遞？</summary>

**分析推理**：

1. **頻率分析**：感知 ~30 Hz 寫、規劃 ~20 Hz 讀，頻率不同 → 不能簡單鎖同一塊記憶體（規劃可能被感知的寫入阻塞）
2. **方案：Double Buffer + Atomic Pointer Swap**：
   - 準備兩塊 `OccupancyGrid` buffer
   - 感知 thread 寫 back buffer（不影響規劃讀 front buffer）
   - 寫完後用 `std::atomic<int>` swap front/back index — **O(1)、wait-free**
   - 用 `condition_variable` 通知規劃 thread「有新 grid 了」
3. **為什麼不用 shared_mutex**：寫入時所有讀者阻塞，50 ms 的 A* 可能和感知衝突
4. **為什麼不用 shared_ptr swap**：可行但 `shared_ptr` 的 atomic 操作比裸 atomic index swap 貴；且如果感知頻率 > 規劃頻率，中間的 grid 會被跳過（這其實是可接受的 — 規劃只需要最新資料）
5. **陷阱**：忘記 `memory_order_release/acquire` 配對 → 規劃可能讀到半寫的 grid；buffer 只準備一塊 → 讀寫衝突

**面試官想聽到**：double buffer 的 wait-free 特性、acquire/release 語意、以及為什麼接受跳過中間幀。

</details>

<details>
<summary>Q4（難）：4 核 ARM 板子上要跑 6 個 ROS 2 nodes，包含 1 kHz 控制迴圈，怎麼部署？</summary>

**分析推理**：

1. **核心分配策略**：
   - 控制 node **獨佔 1 核**：用 `pthread_setaffinity_np` 綁核 + `SCHED_FIFO` 即時排程，確保不被搶佔
   - 其他 5 nodes 共享剩餘 3 核：用 `MultiThreadedExecutor`
2. **跨 node 通訊**：
   - 控制 ↔ 規劃：用 **lock-free SPSC queue** 傳軌跡指令（控制端不能被 mutex 阻塞）
   - 感知 ↔ 規劃：double buffer + atomic swap
   - 其他 non-RT 通訊：正常 ROS 2 topic
3. **Executor 設計**：
   - 控制 node 用獨立 `SingleThreadedExecutor`（確保控制 callback 獨佔執行）
   - 其他 nodes 用 `MultiThreadedExecutor` + 適當的 `CallbackGroup` 區分互斥/可重入
4. **Oversubscription 防護**：6 nodes 不代表需要 6 threads — 合理分配 executor thread 數量 $\leq 4$（= 核心數）
5. **陷阱**：忘記 `isolcpus` kernel 參數導致 OS 排程器把其他 process 也塞到控制核心；控制 node 裡用了 ROS 2 logging（內部有 mutex）

**面試官想聯到**：RT 核心隔離 + lock-free 跨頻率通訊 + executor 分離 = 完整的嵌入式即時部署策略。

</details>

## 面試角度

1. **鎖粒度 + 死結防禦** — 測的是並發設計能力。**帶出**：「我會先選最小粒度的鎖（`lock_guard` 只保護臨界區），多鎖場景用 `scoped_lock` 避 deadlock。更進一步會問：能不能用 `CallbackGroup` 完全消除手動 mutex？」

2. **Spurious wakeup 與 condition_variable** — 測的是細節功力。**帶出**：「`cv.wait` 一定搭配 predicate lambda — 因為 OS 可能 spurious wakeup。這不是理論問題，我在 Linux ARM 板上跑長時間測試時真的遇過。」

3. **Thread pool 與 oversubscription** — 測的是系統層級思考。**帶出**：「thread 數超過核心數就是 oversubscription — 頻繁 context switch 讓即時性不可預測。機器人系統我會固定 thread 數 = 核心數，任務透過 queue 調度。」

4. **Atomic + memory order 與 lock-free** — 測的是硬即時路徑設計。**帶出**：「1 kHz 控制迴圈不能用 mutex（priority inversion），我會用 lock-free SPSC queue 搭配 acquire/release memory order 傳遞軌跡指令。」

## 延伸閱讀

- **《C++ Concurrency in Action》Ch3-4（鎖與 condition variable）** — 掌握 RAII 鎖策略和同步原語的標準教材
- **《C++ Concurrency in Action》Ch5 + Ch7（atomic + 無鎖資料結構）** — 理解 memory order 層級和 lock-free queue 的設計
- **《C++ Concurrency in Action》Ch12（效能與可擴展性）** — 避免 false sharing、cache contention 等進階效能問題
- **ThreadSanitizer (TSan) 官方文件** — 學會用 `-fsanitize=thread` 在 CI 中自動抓 data race
- **ROS 2 文件：Using Callback Groups** — 理解 `MutuallyExclusive` vs `Reentrant` 如何取代手動 mutex
- **PREEMPT_RT + SCHED_FIFO** — Linux 硬即時排程設計，控制 node 獨佔核心的具體操作方式
- **C++20 Coroutines + Asio** — 下一代非同步模型，適合 IO 密集的感知 / 通訊 pipeline
