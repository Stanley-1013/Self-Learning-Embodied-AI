---
title: "C++ 記憶體管理與效能優化"
prerequisites: []
estimated_time: 45
difficulty: 3
tags: ["cpp", "memory", "performance", "real-time"]
sidebar_position: 1
---

# C++ 記憶體管理與效能優化

## 你將學到

- 能清楚講出 stack / heap / RAII / smart pointer 的精確定義，面試不含糊
- 遇到「控制迴路偶爾延遲 spike」情境，知道先懷疑隱藏的 heap 分配並用 `perf` 診斷
- 判斷何時用 `unique_ptr`（零拷貝轉移）、何時用 `shared_ptr`（1 對 N 共享）、何時自建 memory pool

## 核心概念

### Stack vs Heap

**Stack（堆疊）**：編譯器自動配置與釋放，生命週期隨作用域結束自動銷毀。**配置沒有任何執行期開銷**。每個 thread 預設約 1 MB，4096 個 thread 就耗盡 32-bit 的 4 GB 位址空間。

**Heap（堆積）**：透過 `new`/`malloc` 動態請求。呼叫記憶體管理器的開銷可能耗費**數千次記憶體存取指令**，涉及系統呼叫、鎖競爭、碎片合併。

### RAII（Resource Acquisition Is Initialization）

將資源管理綁定到物件的詞法作用域：**建構取得、解構釋放**。無論正常退出或拋出異常，C++ 都保證自動呼叫解構函式。典型例子：`std::lock_guard` 在建構時上鎖、離開 scope 自動解鎖。

### Smart Pointers

| 指標 | 所有權語意 | 成本 |
|------|-----------|------|
| `unique_ptr` | 獨佔，只能 `move` 轉移 | **零成本**（`-O2` 後等同 raw pointer） |
| `shared_ptr` | 共享，引用計數管理 | 昂貴（atomic increment/decrement + full memory barrier） |
| `weak_ptr` | 觀察但不擁有，打破循環引用 | `lock()` 做 atomic 嘗試加強引用 |

`make_shared` 比 `new` 好：一次配置同時放物件與控制塊（省一次 heap 分配、記憶體連續）。

### Memory Pool / Arena Allocator

預先配置一大塊記憶體、分割成固定大小的區塊，配置/釋放只需從 free list pop/push — **O(1)、無碎片、無鎖**。測試：100 萬個實例，pool **4 ms** vs `malloc` **64 ms**（**15× 快**）。

### Cache Line 與 False Sharing

CPU 快取以 **32 或 64 bytes** 的 cache line 為單位讀寫。多個 thread 修改同一 line 內的不同變數 → cache line 所有權在核心間乒乓轉移 → **效能斷崖**。用 `alignas(64)` 或 C++17 `std::hardware_destructive_interference_size` 隔開。

**在感知 → 規劃 → 控制閉環的位置**：
- **不是某個節點，是所有節點的底層基礎設施**
- **控制**（1 kHz+）：禁止 heap 分配 → 不可預測延遲會破壞硬即時保證
- **感知**（點雲/影像，數十 MB/s）：需 zero-copy 設計，否則 CPU 頻寬被深拷貝吃光
- **ROS 2 intra-process**：`unique_ptr` + `std::move` 轉移所有權 = 零複製通訊

$$
\text{延遲預算} = \frac{1}{f_{\text{control}}} \quad \text{（1 kHz → 1 ms，heap 分配可能耗費 2–5 ms，直接爆掉）}
$$

**一句話版本**：「精確的 C++ 記憶體管理是實現零拷貝感知與毫秒級無延遲控制的底層基石。」

<details>
<summary>深入：Smart pointer 底層機制與 Memory pool free list 實作</summary>

### unique_ptr 為什麼零成本
`unique_ptr` 內部只有一個 raw pointer，生命週期管理由編譯器在編譯期靜態安插解構程式碼。開 `-O2` 後，assembly 和操作 raw pointer 完全一致。

### shared_ptr 控制塊佈局
```
new:        [物件] ← ptr1    [控制塊(strong=1, weak=0)] ← ptr2  （兩次配置）
make_shared: [控制塊 | 物件] ← single ptr                         （一次配置、連續記憶體）
```
控制塊包含 strong count + weak count + deleter + allocator。每次 copy 或 destroy 都觸發 atomic 操作。

### Fixed-block memory pool 核心
```cpp
struct FreeBlock { FreeBlock* next; };

class FixedPool {
  FreeBlock* free_list_;
public:
  void* allocate() {
    auto* block = free_list_;
    free_list_ = block->next;  // O(1) pop
    return block;
  }
  void deallocate(void* p) {
    auto* block = static_cast<FreeBlock*>(p);
    block->next = free_list_;  // O(1) push
    free_list_ = block;
  }
};
```
直接利用未使用區塊的前幾 bytes 存 `next` 指標 — 零額外空間浪費。

### weak_ptr::lock() 機制
```
1. 原子讀 strong_count
2. 若 == 0 → 回傳空 shared_ptr（物件已死）
3. 若 > 0 → 原子 CAS 加 1 → 回傳有效 shared_ptr
```

</details>

## 直覺理解

| 概念 | 類比 |
|------|------|
| Stack | 辦公桌上的便條紙疊 — 隨手拿來記，用完順手丟，速度極快 |
| Heap | 遠端倉儲系統 — 容量大但要填申請單、等管理員找空位、多人還要排隊 |
| RAII | 旅館插卡取電 — check-in 拿房卡插入（建構取得），離開拔走自動斷電（解構釋放）；火警逃跑也一樣自動清理 |
| `unique_ptr` | 車鑰匙只有一把 — 要轉交只能實體遞給對方（`std::move`） |
| `shared_ptr` | 圖書館借閱卡 — 多人同時借同一本書，管理員要在小黑板畫正字（atomic 計數） |
| Memory pool | 專屬停車場（劃好線的固定車位） — 進場直接停、離場直接走（O(1)）；vs `malloc` = 在市區到處繞找車位 |

**模擬器 / 工具觀察**：
- 控制迴路裡藏了一個 `new` → 示波器/log 上看到**偶發的 2–5 ms spike**（jitter）
- 感知 pipeline 沒做 zero-copy → `htop` 顯示 CPU 飆高、`perf` 報告 cache miss 暴增
- `ros2 topic hz` 頻率不穩定 → 首先懷疑 callback 裡的 heap 分配或 `shared_ptr` atomic 競爭

## 實作連結

**三個典型工程場景**：

1. **ROS 2 零拷貝傳點雲**：Publisher 用 `unique_ptr<PointCloud2>` 寫入資料，`publish(std::move(msg))` 把所有權轉給 Subscriber — 全程只傳指標不複製。1 對 N 時改用 `const shared_ptr`。

2. **1 kHz 控制迴圈記憶體策略**：初始化階段 `reserve` 所有容器、用 `std::array` 替代 `vector`、自建 fixed-block pool。迴圈內部**零 `new`**。

3. **GPU RL policy → 控制迴圈的跨頻率傳遞**：Triple Buffer（atomic pointer swap）避免 mutex；`alignas(64)` 把 policy output 和 control state 隔到不同 cache line；`cudaHostAlloc` pinned memory 讓 DMA 直達 CPU RAM。

```cpp
// ROS 2 zero-copy publish 骨架
auto msg = std::make_unique<sensor_msgs::msg::PointCloud2>();
fill_pointcloud(*msg, lidar_data);
publisher_->publish(std::move(msg));  // 所有權轉移，零拷貝
```

<details>
<summary>深入：完整 Fixed-block Memory Pool 實作（C++17，可 copy-paste）</summary>

```cpp
#include <cstddef>
#include <cstdint>
#include <new>
#include <cassert>

class FixedPool {
  struct Block { Block* next; };
  Block* free_list_ = nullptr;
  std::byte* arena_ = nullptr;
  std::size_t block_size_;
  std::size_t count_;

public:
  FixedPool(std::size_t block_size, std::size_t count)
    : block_size_(std::max(block_size, sizeof(Block))), count_(count) {
    arena_ = new std::byte[block_size_ * count_];
    // 初始化 free list：把每個區塊串起來
    for (std::size_t i = 0; i < count_; ++i) {
      auto* block = reinterpret_cast<Block*>(arena_ + i * block_size_);
      block->next = free_list_;
      free_list_ = block;
    }
  }

  ~FixedPool() { delete[] arena_; }

  void* allocate() {
    assert(free_list_ && "Pool exhausted");
    auto* block = free_list_;
    free_list_ = block->next;
    return block;
  }

  void deallocate(void* p) {
    auto* block = static_cast<Block*>(p);
    block->next = free_list_;
    free_list_ = block;
  }
};

// 使用範例：ROS 2 控制迴圈中預配置 1000 個 JointCommand
struct JointCommand { double positions[7]; double velocities[7]; };
FixedPool cmd_pool(sizeof(JointCommand), 1000);  // 初始化階段

void control_callback() {
  auto* cmd = static_cast<JointCommand*>(cmd_pool.allocate());  // O(1)，零鎖
  // ... 填入控制指令 ...
  cmd_pool.deallocate(cmd);  // O(1)
}
```

</details>

## 常見誤解

1. **以為 `shared_ptr` 是免費的** — 每次 copy 都觸發 atomic increment（完整記憶體屏障），多 thread 下開銷極大。**避開**：只讀場景傳 `const T&` 或 `.get()`，不要無腦 pass-by-value。

2. **在即時 callback 用 `vector::push_back`** — `size == capacity` 時觸發 realloc：申請雙倍記憶體 + 全量拷貝 + 舊迭代器全部失效。**避開**：初始化 `reserve(N)` 或改用 `std::array`。

3. **以為 false sharing 只在 HPC 才重要** — 多核機器人系統一樣中招。把不同 thread 更新的計數器/狀態塞同一個 struct → cache ping-pong。**避開**：`alignas(64)` 隔開。

4. **以為 `std::string` 很輕量** — SSO（Small String Optimization）只管短字串（通常 < 22 bytes），超過就 heap 分配。迴圈內反覆拼接可慢到 **170×**。**避開**：迴圈外宣告 + `clear()` 復用，或 `reserve()`。

## 練習題

<details>
<summary>Q1（簡單）：ROS 2 callback 要傳大 PointCloud2 給下游 node，用 raw pointer、unique_ptr 還是 shared_ptr？</summary>

**分析推理**：
1. 點雲體積大（數 MB），深拷貝吃爆 CPU — 目標是 zero-copy
2. 排除 raw pointer：無法自動管生命週期，容易 leak 或 dangling
3. **1 對 1**（單一 Subscriber）→ `unique_ptr` + `std::move` 轉移所有權，零額外開銷
4. **1 對 N**（多個 Subscriber，如同時給 SLAM + obstacle detection）→ `const shared_ptr`，多個下游共享唯讀存取同一塊記憶體
5. 陷阱：1 對 1 場景下濫用 `shared_ptr` 白白承受 atomic 開銷

**面試官想聽到**：清楚 ROS 2 intra-process zero-copy 的底層是 `unique_ptr` + `move`，並能區分 1 對 1 vs 1 對 N 的設計差異。

</details>

<details>
<summary>Q2（中）：1 kHz 控制迴路偶發 2–5 ms 延遲 spike，懷疑記憶體問題，怎麼診斷？</summary>

**分析推理**：
1. **確診**：1 ms 週期裡出現 2–5 ms spike → 極大機率是 callback 內觸發 heap 分配（系統呼叫 + 鎖競爭）
2. **工具診斷**：用 `perf` + eBPF 抓 spike 時刻的 call stack，看是否陷入 `malloc`/`free`/syscall
3. **盤查隱藏 new**：
   - `vector::push_back` 超過 capacity
   - 迴圈內宣告 `std::string`（超 SSO 就 heap）
   - 呼叫 `make_shared`
4. **修復**：所有容器 `reserve()` 前置到初始化；改用 `std::array` / fixed-block pool；迴圈內**零 `new`**

**面試官想聽到**：「硬即時迴圈禁止 heap 分配」的鐵律，搭配 `perf` 或 memory pool 的實戰手段。

</details>

<details>
<summary>Q3（中-難）：4 個 ROS 2 node 的感知 pipeline（Camera → SLAM → Obstacle → Planning），每幀 1.2 MB RGBD @ 30fps，怎麼設計記憶體策略？</summary>

**分析推理**：
1. **Component 容器化**：4 個 node 用 `rclcpp_components` 載入同一 process，啟用 `use_intra_process_comms(true)`
2. **Object Pool**：Camera driver 不每幀 new buffer — 預配置 pool，取一塊寫入後封裝成 `const shared_ptr<Image>` 發布
3. **1 對 N 共享**：SLAM 和 Obstacle 平行處理 → `const shared_ptr` 讓兩個 Node 讀同一塊 1.2 MB，最後一個讀完才歸還 pool
4. **SoA + 對齊**：影像矩陣每行起始 32/64 bytes 對齊，配合 SIMD (AVX/SSE) 加速特徵提取
5. 陷阱：忘開 intra-process comm；1 對 N 誤用 `unique_ptr` → 編譯失敗或降級為 deep copy

**面試官想聽到**：Component 容器化 + intra-process zero-copy + Object Pool + SoA 對齊 = 完整的系統級記憶體策略。

</details>

<details>
<summary>Q4（難）：RL policy 在 GPU 跑 30 Hz，關節控制需要 1 kHz 指令，怎麼跨頻率傳資料？</summary>

**分析推理**：
1. **頻率落差問題**：30 Hz 寫、1 kHz 讀，不能用 `std::mutex`（1 kHz 端會被 GPU context switch 阻塞）
2. **無鎖 Triple Buffer**：GPU 寫完寫到 back buffer → `std::atomic<TargetCommand*>` pointer swap（`memory_order_release`）→ 1 kHz 端用 `memory_order_acquire` 讀最新指標，全程 O(1) wait-free
3. **防 false sharing**：RL output 和 control state 綁不同核心 → 用 `alignas(64)` 把兩者的記憶體隔到不同 cache line
4. **GPU → CPU 零拷貝**：`cudaHostAlloc` 分配 pinned memory，GPU DMA 直接寫入 CPU RAM，省去 paged memory 換頁開銷
5. 陷阱：1 kHz 迴圈用 mutex（jitter 爆掉）；忽略 cache line 隔離（底層匯流排塞車）

**面試官想聯到**：atomic pointer swap 解頻率落差 + `alignas(64)` 消 false sharing + pinned memory 壓 GPU-DMA 延遲 = 頂級軟硬體協同。

</details>

## 面試角度

1. **Heap 分配的致命傷** — 測的是硬即時思維。**帶出**：「1 kHz 控制迴圈中，我絕對會避免隱藏的動態記憶體分配，因為系統 allocator 帶來的延遲與鎖競爭是不可預測的。」

2. **Cache Locality 與 False Sharing** — 測的是硬體架構理解力。**帶出**：「多 thread 共享資料結構時，我會用 `alignas(64)` 把不同 thread 存取的變數隔開，徹底消除 cache ping-pong。」

3. **Memory Pool = O(1) 分期攤銷** — 測的是能不能把效能壓到極限。**帶出**：「對於高頻場景，我會實作 fixed-block pool，把分配成本全部前置到初始化階段，執行期只做指標抽換。」

4. **Zero-copy 與 Smart Pointer 的精準打擊** — 測的是所有權掌控。**帶出**：「傳大資料時我用 `unique_ptr` + `move` 實現零拷貝；非得共享時用 `make_shared` 確保控制塊和資料記憶體連續。」

## 延伸閱讀

- **《C++ 性能優化指南》Ch6 動態變量、Ch13 記憶體優化** — 白板面試能寫出 fixed-block pool 結構
- **《C++ 併發編程》Ch5 記憶體模型與原子操作** — 能解釋 `memory_order_acquire/release` 為什麼重要
- **ROS 2 官方文件：Intra-process Communication** — 學 `unique_ptr` 如何在 node 間實現零拷貝
- **jemalloc / tcmalloc** — 無法重構老 code 時，直接抽換 allocator 的「免費午餐」
- **`std::pmr` (C++17 Polymorphic Memory Resources)** — 現代 C++ 把標準容器塞進自訂記憶體池的標準做法
- **perf + FlameGraph** — Linux 下精準定位 cache miss 和 syscall 瓶頸的終極武器
