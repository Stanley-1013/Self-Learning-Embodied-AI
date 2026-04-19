# Extracted: C++ 多執行緒與同步機制
<!-- 提取時間: 2026-04-19 -->
<!-- 來源 Notebook: 0502daf4-1648-4456-bfd4-f1dee3875753 -->
<!-- NotebookLM session: fe23a257 (Q6-Q10) -->

## Q1 — 精確定義 + 閉環定位
- Thread vs Process：thread 輕量、共享記憶體；process 獨立但 IPC 慢
- std::jthread (C++20)：auto-joining + cooperative stop token
- Mutex 家族：mutex/recursive_mutex/timed_mutex/shared_mutex
- RAII 鎖：lock_guard (最輕) / unique_lock (進階) / scoped_lock (多鎖避 deadlock)
- Deadlock 四條件：互斥、持有等待、不可剝奪、循環等待
- Condition variable：搭配 unique_lock、解決 busy-waiting
- future/promise/async：非同步三件套
- Thread pool：預建常駐 thread 消除啟動/銷毀開銷
- 閉環定位：感知/規劃/控制/通訊分 thread；控制 = 硬即時；ROS 2 MultiThreadedExecutor + CallbackGroup

## Q2 — 核心機制 / API
- std::lock() 用 try-and-back-off 避 deadlock
- scoped_lock 同時鎖多個
- condition_variable 搭 while（防 spurious wakeup）
- 粗 vs 細粒度鎖：取捨
- Lock-free queue：硬即時必備
- Thread-safe queue：返回 shared_ptr + dummy node 分離 head/tail 鎖
- TSan：-fsanitize=thread 抓 data race（2-10× 慢）
- Helgrind：不需重編但 20-50× 慢
- ROS 2 MutuallyExclusiveCallbackGroup 免手寫 mutex

## Q3 — 直覺 / 誤解
- 類比：mutex=洗手間鎖、deadlock=搶鼓的兩小孩、cv=火車到站廣播、thread pool=計程車行
- Deadlock 症狀：CPU 下降、topic hz 歸零、gdb thread apply all bt 抓
- Race condition 症狀：幽靈障礙物瞬移、規劃軌跡跳變
- Priority inversion 症狀：控制 jitter + watchdog 急停
- 5 誤解：介面級 race（empty+top 不原子）、recursive_mutex 掩蓋設計問題、cv 用 if 不用 while、async 不一定開新 thread、real-time callback 用 mutex = priority inversion

## Q4 — 4 情境題
1. Timer+Sub 共寫 vector → MutuallyExclusiveCallbackGroup 或 lock_guard
2. 跑幾小時卡住所有 topic 歸零 → gdb bt + TSan + scoped_lock
3. 感知 33ms 更新 grid vs 規劃 50ms A* → shared_ptr pointer swap + double buffer + notify
4. 4 核 ARM 6 nodes → 分離 executor、控制獨佔 1 核 SCHED_FIFO、lock-free queue 傳軌跡

## Q5 — Talking points / 延伸 / 閱讀
- 4 talking points：鎖粒度+死結防禦、spurious wakeup、thread pool+oversubscription、atomic+memory order
- 延伸：std::atomic+memory_order、lock-free DS、PREEMPT_RT+SCHED_FIFO、C++20 coroutines
- 閱讀：《併發》Ch3-4 → Ch12 效能 → Ch5+7 原子+無鎖
