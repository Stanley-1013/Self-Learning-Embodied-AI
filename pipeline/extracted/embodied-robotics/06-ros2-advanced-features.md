# Extracted: ROS 2 進階：QoS 與執行器
<!-- session: fe23a257, Q17 (all-in-one) -->

## 定義
- QoS 7 策略：Reliability(Best-effort/Reliable) / Durability(Volatile/Transient local) / History(Keep last/all) / Depth / Deadline / Liveliness(Auto/Manual) / Lifespan
- 相容性：Pub品質≥Sub要求才通；Best-effort pub + Reliable sub = 靜默失敗
- Presets：SensorDataQoS(Keep last 5, Best-effort, Volatile) / ServicesQoS(Keep last 10, Reliable, Volatile)
- Executor：StaticSingleThreaded(掃一次拓撲、CPU低) / MultiThreaded(thread pool 平行) / EventsExecutor(event-driven 取代 WaitSet polling)
- Callback Group：MutuallyExclusive(互斥、免鎖) / Reentrant(平行、需自行加鎖)

## 閉環定位
- QoS = 網路資料包的底層濾網（可靠性+時效性）
- Executor = CPU 計算資源分配的調度大腦（排程確定性）
- 一句話：「QoS 掌控網路、Executor 掌控算力」

## 直覺 + 誤解
- 類比：QoS=快遞SLA合約、Executor=餐廳外場經理、CallbackGroup=防撞規則
- 4 誤解：不設 Deadline→掉包不知；thread數>核心=oversubscription；Reliable on sensor=延遲暴增(隊頭阻塞)；callback group 設錯→自己等自己 deadlock

## 3 情境題
1. Nav2 costmap 延遲 100ms → 查 QoS Reliable on LiDAR → 改 SensorDataQoS
2. 100Hz timer + 30Hz sub 偶發錯誤 → Data Race → MutuallyExclusiveCallbackGroup
3. WiFi 機器人群不穩 → Multicast 廣播風暴 → Domain ID 隔離 + Discovery Server

## Talking points / 延伸 / 閱讀
- 3 points：QoS 相容性排查(topic info -v)、Executor 隔離即時性(StaticSingleThreaded+CPU綁定)、Component+零拷貝
- 延伸：DDS Discovery Server、Loaned Message、Lifecycle Node
- 閱讀：《ROS2》Ch10、ROS 2 Design Documents (Executors & WaitSet)
