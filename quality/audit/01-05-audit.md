# Audit Report: Ch01-05

**Total flags: 14** — HIGH: 2, MED: 6, LOW: 6

---

## Ch01 C++ Memory Optimization

### Findings

- **[MED]** `01-cpp-memory-optimization.md:46` — Claims "CPU 快取以 **32 或 64 bytes** 的 cache line 為單位". In practice all modern CPUs relevant to robotics (x86-64, ARMv8 Cortex-A7x/Neoverse, Apple M-series, RISC-V server cores) use 64 B. 32 B cache lines exist only on very old / embedded micro-architectures that are irrelevant here. Mentioning "32 or 64" is misleading when the chapter then prescribes `alignas(64)`. Suggested fix: say "通常 64 bytes（x86-64 / ARMv8 皆然；POWER 是 128 bytes）". Confidence: HIGH.

- **[LOW]** `01-cpp-memory-optimization.md:201` — "SSO 通常 < 22 bytes". This is libc++'s threshold (23 including null); libstdc++ since GCC 5 is 15 bytes; MSVC is 15 bytes. Stating a single number as "通常" conflates stdlibs. Suggested fix: "通常 15–22 bytes，視實作而定（libstdc++/MSVC 15、libc++ 22）". Confidence: HIGH.

- **[LOW]** `01-cpp-memory-optimization.md:42` — "pool 4 ms vs malloc 64 ms (**15× 快**)". 64/4 = 16×, not 15×. Minor arithmetic slip. Suggested fix: change to "16× 快" or round the raw numbers. Confidence: HIGH.

- **[LOW]** `01-cpp-memory-optimization.md:22` — "4096 個 thread 就耗盡 32-bit 的 4 GB 位址空間". The math is right (4096 × 1 MB = 4 GiB) but the framing is odd: on a 32-bit process you run out of usable VA long before that (kernel reserves 1–2 GB, heap/code/data eat more, and the default glibc stack is 8 MB not 1 MB unless overridden via `pthread_attr_setstacksize`). The "預設約 1 MB" claim is Windows-biased; Linux default is 8 MB. Suggested fix: specify platform ("Windows 預設 1 MB / Linux 預設 8 MB") and drop the 4096 computation or reframe it. Confidence: HIGH.

- **[LOW]** `01-cpp-memory-optimization.md:36` — `shared_ptr` cost described as "atomic increment/decrement + full memory barrier". The increment is `memory_order_relaxed` (no barrier); only the decrement that reaches zero uses `acq_rel`/`acquire` to synchronize destructor. Saying "full memory barrier" on every copy overstates it. Suggested fix: "atomic increment（relaxed）/ decrement（acq_rel，觸發 destructor 時才需屏障）". Confidence: HIGH.

---

## Ch02 C++ Concurrency & Synchronization

### Findings

- **[MED]** `02-cpp-concurrency-sync.md:351` — "Linux 預設 mutex 不支援 priority inheritance". The statement is true as default behavior, but pthread mutex *does* support PI via `pthread_mutexattr_setprotocol(..., PTHREAD_PRIO_INHERIT)` (used extensively in PREEMPT_RT). The text implies you must abandon mutex for lock-free, which glosses over the standard RT solution. Suggested fix: "Linux 預設 mutex 不啟用 priority inheritance；硬即時可改用 `PTHREAD_PRIO_INHERIT` 或 lock-free". Confidence: HIGH.

- **[MED]** `02-cpp-concurrency-sync.md:384` — "評估是否能用 `MutuallyExclusiveCallbackGroup` 完全消除手動 mutex". Mutual exclusion only applies *within* one group. Two callbacks in *different* ME groups can still run concurrently. If the intent is "protect a shared `vector`", putting both callbacks in the *same* ME group is what is required — the phrasing is ambiguous and Q1 on line 361 gets it right, so this is inconsistent with the correct statement. Suggested fix: clarify "把互斥的 callback 放同一個 ME group". Confidence: MED.

- **[LOW]** `02-cpp-concurrency-sync.md:87` — "每次 `std::thread` 建立/銷毀的開銷（通常數十 $\mu s$）". Real-world thread creation cost on Linux is more like 10–50 µs creation + few µs join, but with significant variance; "數十 µs" is defensible but a bit loose. Confidence: LOW (minor).

- **[LOW]** `02-cpp-concurrency-sync.md:169` — Thread-safe queue returns `std::shared_ptr<T>` "避免 pop 後物件消失的 race". The commentary is right but slightly misleading — the actual race being avoided is the two-step `front()` + `pop()` on an unprotected queue. Once you hold the lock the shared_ptr vs T by-value is just about exception safety during the move/copy out. Suggested fix: say "避免 pop 與 front 之間的 race，並提供強異常安全保證". Confidence: MED.

---

## Ch03 C++ Memory Model & Atomics

### Findings

- **[HIGH]** `03-cpp-memory-model-atomics.md:49, 111, 426` — Repeatedly states `seq_cst store` on ARMv8 = `STLR + DMB`. This is **incorrect for the standard ARMv8 C++ mapping**. Per the accepted C/C++ → ARMv8 mapping (Cambridge "cpp0xmappings"), `atomic::store(seq_cst)` compiles to plain `STLR` (no DMB); `atomic::load(seq_cst)` is `LDAR` (no DMB). The extra `DMB ISH` is only needed for seq_cst RMW sequences or the old "leading fence" mapping. The claim "代價比 x86 大很多" and "STLR + DMB" overstates the cost of seq_cst on ARMv8 for pure load/store. The whole "30 萬 cycles/s 浪費" calculation on line 428 rests on this false premise. Suggested fix: clarify that on ARMv8 `LDAR/STLR` alone already provide seq_cst semantics for load/store; the DMB appears only for RMW (`fetch_add` etc.) or on ARMv7. Godbolt confirms: `atomic<int>::store(..., seq_cst)` on `-march=armv8-a` emits a single `stlr` on GCC/Clang. Confidence: HIGH.

- **[MED]** `03-cpp-memory-model-atomics.md:517` — "`strong` 版本內部其實也是迴圈". This is implementation-defined: on x86 `compare_exchange_strong` uses a single `lock cmpxchg` with no loop; on ARM LL/SC the library does wrap `strong` in a tiny retry to filter spurious STXR failures, but not a semantic retry over user state. Stating flatly "strong 內部也是迴圈" is an over-generalization. Suggested fix: "在 LL/SC 架構（ARM）上 `strong` 內部有防 spurious 的小迴圈；在 x86 上就是單條 `lock cmpxchg`". Confidence: HIGH.

- **[MED]** `03-cpp-memory-model-atomics.md:81` — "唯一被 C++ 標準保證 lock-free 的型別. 只有 `test_and_set()` 和 `clear()` 兩個操作". Since C++20, `std::atomic_flag` also has `test()`, `wait()`, `notify_one()`, `notify_all()`. "只有兩個操作" is outdated for C++17+. Suggested fix: note the C++20 additions. Confidence: HIGH.

- **[LOW]** `03-cpp-memory-model-atomics.md:46` — Consume is described as "deprecated". The C++ standard itself does not mark `memory_order_consume` as deprecated; it is *discouraged* and most implementations promote it to `acquire`. "Deprecated" is too strong. Suggested fix: "實務上棄用（compilers 目前都當 acquire 處理），但標準未正式 deprecate". Confidence: MED.

- **[LOW]** `03-cpp-memory-model-atomics.md:272` — "`TaggedPtr` 需要 128-bit CAS (`cmpxchg16b` on x86-64)". On ARMv8 the equivalent is `CASP` (ARMv8.1) or `LDXP/STXP` pair. The chapter gives only the x86 name; a reader on Jetson might think it's unavailable. Confidence: LOW.

---

## Ch04 ROS 2 Basic Communication

### Findings

- **[MED]** `04-ros2-basic-communication.md:61-65` — Formula styling: `SensorDataQoS = {BestEffort, KeepLast(5), Volatile}`. Correct for the standard `rmw_qos_profile_sensor_data`, and `KeepLast(5)` is right. No error — but the "Durability: Volatile" for Services (line 67-69) is presented as if it were specified. Service QoS in rclcpp is typically `ReliabilityPolicy::Reliable, DurabilityPolicy::Volatile, HistoryPolicy::KeepLast(10)` — the text's summary is OK. No flag. (Removed.)

- **[MED]** `04-ros2-basic-communication.md:388` — "預設的多播 Discovery 在複雜網路拓撲（多網卡、Wi-Fi）會出問題". True in practice, but the framing hides that it is also a common issue on single-machine WSL2 setups. Minor. Confidence: LOW — kept only as context.

- **[LOW]** `04-ros2-basic-communication.md:242` — In the Component sample, the subscription callback takes `sensor_msgs::msg::PointCloud2::UniquePtr`. This only yields true zero-copy intra-process when both the publisher AND subscriber have `use_intra_process_comms(true)` AND the publisher publishes with `unique_ptr` (not `shared_ptr`). The sample only sets it on the subscriber side in prose (line 234 comment says "NodeOptions 需設定"), which is a reader-trap. Suggested fix: explicitly state both publisher and subscriber must enable it, and publisher must `publish(unique_ptr)`. Confidence: HIGH.

*(Otherwise Ch04 reads accurate; QoS compatibility table, Executor descriptions, Action semantics, and Component/intra-process advice are all substantively correct.)*

---

## Ch05 ROS 2 TF & Tools

### Findings

- **[HIGH]** `05-ros2-tf-and-tools.md:36, 378` — "資料永遠從 parent 流向 child" / "資料從大流向小". This is backward / confusing. In TF, a transform published with `frame_id=parent, child_frame_id=child` encodes the pose of child expressed in parent (i.e. $T^{parent}_{child}$). When you *transform a point*, points go from **child frame → parent frame** by multiplying by $T^{parent}_{child}$ (that is how `do_transform_point` works). The chapter's own line 73 gets this right: "把 source frame 裡的一個點乘上這個變換，就得到它在 target frame 裡的座標". So the "parent → child" data-flow slogan contradicts the formal definition two paragraphs later, and is likely to mislead beginners into publishing transforms in the wrong direction. Suggested fix: drop the slogan, or replace with "TF 廣播的方向是 parent 發布 child 的相對姿態；實際座標轉換則是把 child frame 的點映射到 parent frame". Confidence: HIGH.

- **[MED]** `05-ros2-tf-and-tools.md:47` — "Buffer 預設保留 10 秒". The default `tf2_ros::Buffer` cache is 10 s, correct. No flag. (Verified.)

- **[MED]** `05-ros2-tf-and-tools.md:113` — SLERP formula: `q = slerp(q1, q2, t)` with linear interpolation on translation. Correct. Formula OK. No flag.

- **[MED]** `05-ros2-tf-and-tools.md:469` — "歐拉角有 Gimbal Lock、插值不均勻；四元數 4 個參數、SLERP 插值平滑、連乘高效". Technically fine, but "連乘高效" is debatable — matrix multiplication on 3×3 can be faster on vector hardware than quaternion multiplication. The real wins for quaternions are (a) no gimbal lock, (b) half the storage vs rotation matrix, (c) re-normalization is cheap. Suggested fix: drop "連乘高效" or qualify it. Confidence: MED.

- **[LOW]** `05-ros2-tf-and-tools.md:33` — Table says `odom` is "由 wheel odometry / VIO 維護". Strictly VIO (visual-inertial odometry) output is often published as a separate `odom`-like frame, but the classic REP-105 convention has `odom` produced by *any* continuous odometry source (wheel, VIO, lidar odometry). The text's list is fine, just non-exhaustive. Confidence: LOW.

- **[LOW]** `05-ros2-tf-and-tools.md:27` — `earth` frame described as "多機器人系統使用，考慮地球曲率". Per REP-105, `earth` is the ECEF (Earth-Centered, Earth-Fixed) frame used to relate multiple `map` frames on Earth; "考慮地球曲率" is a fair simplification but strictly `earth` is just ECEF, not a curvature model. Confidence: MED.

---

## Summary

**14 flags total** across Ch01-05 — 2 HIGH, 6 MED, 6 LOW.

Highest-priority items to address:
1. **Ch03 ARMv8 seq_cst = STLR + DMB** (recurring claim, line 49/111/426) — incorrect hardware mapping that undermines the chapter's central performance argument.
2. **Ch05 "資料永遠從 parent 流向 child"** (line 36, 378) — contradicts the chapter's own formal definition and is likely to produce wrongly-directed transforms in student code.
3. **Ch03 "strong 內部也是迴圈"** (line 517) — over-generalized claim not true on x86.
4. **Ch02 priority inheritance** (line 351) — omits the standard PTHREAD_PRIO_INHERIT escape hatch, biasing students toward lock-free when PI mutex often suffices.
5. **Ch01 cache line "32 或 64 bytes"** (line 46) — misleading for modern HW; the prescribed `alignas(64)` makes sense only because it's always 64.
