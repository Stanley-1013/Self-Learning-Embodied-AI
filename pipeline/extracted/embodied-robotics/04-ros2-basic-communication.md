# Extracted: ROS 2 節點與核心通信機制
<!-- session: fe23a257, Q14-Q15 -->

## Q1+Q2 — 定義 + 閉環 + API
- Node：基本單元，ROS 2 支援 Component 化（多 node 同 process）
- Topic：Pub/Sub 非同步 1-to-N（感測器資料、cmd_vel）
- Service：Client/Server request-response（一次性查詢）
- Action：長時間任務 + feedback + cancel（導航、抓取）
- DDS：底層中介軟體，RTPS 協定，QoS 策略
- Executor：Single/Multi/StaticSingle；MultiThreaded 用 thread pool 並行回呼
- Launch：Python 取代 XML（條件、動態參數）
- 閉環：Topic=高頻感測/控制、Service=一次性查詢、Action=導航抓取
- QoS：感測器=Best-effort+Volatile、指令=Reliable
- Intra-process+Component：感知→規劃最重要（大資料 zero-copy）
- API 骨架：Publisher/Subscriber rclcpp、Service async_send_request、SensorDataQoS/ServicesQoS preset

## Q3 — 直覺 + 誤解
- 類比：Topic=廣播、Service=打電話、Action=叫外送、QoS=平信vs掛號
- 5 誤解：Topic/Service 互換、MultiThreadedExecutor 非自動 safe、QoS 不相容收不到、callback 長計算阻塞、忘 source setup.bash

## Q4 — 3 情境題
1. LiDAR subscriber 收不到：QoS Incompatible（Reliable sub vs Best-effort pub）→ 改 SensorDataQoS
2. 機械臂通訊選型：100Hz 狀態=Topic、TCP 查詢=Service、軌跡執行=Action
3. MultiThreadedExecutor 偶發 crash：Data Race on shared buffer → TSan 診斷 → MutuallyExclusiveCallbackGroup

## Q5 — Talking points / 延伸 / 閱讀
- 3 points：DDS QoS 調校、Component+Intra-process zero-copy、Executor 隔離即時性
- 延伸：DDS Discovery Server、Loaned Message（跨 process zero-copy）、Lifecycle Node
- 閱讀：《ROS2 實踐》Ch10、ROS 2 Design Documents
