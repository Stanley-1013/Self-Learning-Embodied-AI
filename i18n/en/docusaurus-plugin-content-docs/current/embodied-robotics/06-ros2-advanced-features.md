---
title: "ROS 2 Advanced: QoS and Executors"
prerequisites: ["05-ros2-tf-and-tools"]
estimated_time: 45
difficulty: 3
tags: ["ros2", "qos", "executor", "callback-group", "dds"]
sidebar_position: 6
---

# ROS 2 Advanced: QoS and Executors

## You Will Learn

- Precisely define the 7 QoS policies and know which preset to reach for in each sensor/actuator scenario — no guessing in interviews
- When a Nav2 costmap shows mysterious 100 ms latency, know to suspect a QoS reliability mismatch on the LiDAR topic and fix it in under a minute
- Choose the right Executor + Callback Group combination so that a multi-rate system (e.g. 100 Hz timer + 30 Hz subscriber) runs without data races or deadlocks

## Core Concepts

### Quality of Service (QoS) — The 7 Policies

QoS is the contract between publisher and subscriber that governs **how data travels over DDS**. Think of it as a configurable filter sitting between your node logic and the network wire.

| Policy | Options | Physical meaning |
|--------|---------|-----------------|
| **Reliability** | Best-effort / Reliable | "Is retransmission worth the latency?" Best-effort drops lost packets; Reliable retransmits them |
| **Durability** | Volatile / Transient local | "Should late joiners get old data?" Transient local caches the last N samples for new subscribers |
| **History** | Keep last / Keep all | "How many samples to buffer?" Keep last stores the most recent N; Keep all stores everything (memory danger) |
| **Depth** | Integer | The N in Keep last — how deep the buffer goes |
| **Deadline** | Duration | "Maximum allowed gap between samples" — the middleware raises an event if no message arrives in time |
| **Liveliness** | Automatic / Manual by topic | "How do we detect a dead publisher?" Automatic lets DDS heartbeat handle it; Manual requires the node to explicitly assert liveness |
| **Lifespan** | Duration | "How long is a sample valid?" Expired samples are discarded even if still in the queue |

**Compatibility rule**: A connection is only established when the publisher's QoS **meets or exceeds** what the subscriber demands. Best-effort publisher + Reliable subscriber = **silent failure** — no error message, just no data. This is the single most common QoS debugging headache.

**Built-in presets** (the ones you actually use 90% of the time):

| Preset | Key settings | Use case |
|--------|-------------|----------|
| `SensorDataQoS` | Keep last 5, Best-effort, Volatile | LiDAR, camera, IMU — high-frequency, drop-tolerant |
| `SystemDefaultsQoS` | Keep last 10, Reliable, Volatile | General inter-node communication |
| `ServicesQoS` | Keep last 10, Reliable, Volatile | Service calls |
| `ParametersQoS` | Keep last 1000, Reliable, Transient local | Parameter events |

$$
\text{QoS compatibility} \iff Q_{\text{pub}} \geq Q_{\text{sub}} \quad \text{(publisher quality must meet or exceed subscriber demand)}
$$

*Physical meaning*: this inequality is why a best-effort publisher silently refuses to connect to a reliable subscriber — the publisher cannot guarantee what the subscriber requires.

### Executors — CPU Scheduling for Callbacks

An Executor is the scheduling engine that decides **which callback runs on which thread, and when**. It is to your node's callbacks what an OS scheduler is to processes.

| Executor | Mechanism | Best for |
|----------|-----------|----------|
| `SingleThreadedExecutor` | One thread, round-robin | Simple nodes, guaranteed no concurrency |
| `StaticSingleThreadedExecutor` | Scans callback topology once at startup, then never again | Low CPU overhead, deterministic for fixed-topology systems |
| `MultiThreadedExecutor` | Thread pool, callbacks dispatched in parallel | Nodes with independent heavy callbacks (e.g. perception + planning) |
| `EventsExecutor` | Event-driven (replaces WaitSet polling) | Lowest latency, avoids busy-wait CPU burn |

**Key insight**: `StaticSingleThreadedExecutor` is the go-to for real-time control nodes. It avoids the overhead of re-scanning the callback graph every spin cycle, which can matter at 1 kHz+.

### Callback Groups — Concurrency Control

Callback Groups tell the `MultiThreadedExecutor` which callbacks are allowed to run simultaneously.

| Type | Behavior | When to use |
|------|----------|-------------|
| `MutuallyExclusiveCallbackGroup` | Only one callback from this group runs at a time — **no locking needed** | Default safe choice; use when callbacks share state |
| `ReentrantCallbackGroup` | Multiple callbacks from this group can run in parallel — **you must handle locking** | Use when callbacks are truly independent and you need throughput |

**The golden rule**: if two callbacks touch the same data, put them in the same `MutuallyExclusiveCallbackGroup`. The executor enforces serialization for you — no mutex, no deadlock, no race.

**Position in the Sense → Plan → Control Loop**:

- **QoS** sits at the **network transport layer** — it governs how sensor data (Sense) and commands (Control) flow between nodes. Wrong QoS on a LiDAR topic can starve your entire planning pipeline.
- **Executor** sits at the **compute scheduling layer** — it determines whether your control callback gets CPU time promptly or waits behind a heavy perception callback.
- **One-liner**: "QoS controls the network pipe; Executor controls the CPU pipe."

<details>
<summary>Deep dive: DDS discovery and QoS negotiation internals</summary>

### How DDS discovery works under the hood

ROS 2 uses DDS (Data Distribution Service) as its middleware. When a node starts:

1. **Participant Discovery Phase (PDP)**: The node multicasts a "I exist" announcement on the DDS domain. Other participants respond. This uses SPDP (Simple Participant Discovery Protocol).

2. **Endpoint Discovery Phase (EDP)**: Once participants know about each other, they exchange information about their publishers and subscribers — including topic names, types, and **QoS profiles**. This uses SEDP (Simple Endpoint Discovery Protocol).

3. **QoS Matching**: The DDS middleware compares publisher QoS against subscriber QoS. If compatible, a connection is established. If not, the middleware silently drops the match — no error log by default.

### Why "silent failure" happens

The DDS spec treats QoS incompatibility as a **policy decision, not an error**. The middleware assumes you intentionally set incompatible QoS to prevent a connection. ROS 2 mitigates this with `IncompatibleQosEvent` callbacks (since Foxy), but most tutorials do not set them up.

### Discovery in multi-robot systems

Default DDS multicast discovery sends packets to **all participants on the same Domain ID**. With 10+ robots on the same WiFi:

- **Multicast storm**: each robot announces to all others → quadratic traffic growth
- **Fix 1**: Separate `ROS_DOMAIN_ID` per robot (simple but limits cross-robot communication)
- **Fix 2**: Use **Discovery Server** (Fast DDS feature) — a centralized broker that replaces multicast with unicast. Reduces discovery traffic from $O(n^2)$ to $O(n)$

```bash
# Start a discovery server
fastdds discovery -i 0 -l 192.168.1.100 -p 11811

# Point clients to the server
export ROS_DISCOVERY_SERVER=192.168.1.100:11811
```

</details>

<details>
<summary>Deep dive: EventsExecutor architecture and why it matters for real-time</summary>

### The WaitSet problem

Traditional ROS 2 executors use a **WaitSet** — a DDS primitive that blocks until any registered entity (subscription, timer, service, etc.) has data ready. The executor loop looks like:

```
while running:
    collect all entities into WaitSet    ← re-scans topology every iteration
    wait on WaitSet                       ← blocks until something is ready
    for each ready entity:
        execute callback
```

The problem: `collect all entities` is $O(n)$ where $n$ is the number of subscriptions, timers, services, etc. At 1 kHz with many entities, this overhead is measurable.

### EventsExecutor: event-driven replacement

The `EventsExecutor` (introduced in Iron) replaces the polling model:

```
at entity creation:
    register event callback with DDS layer

when event fires:
    push callback onto lock-free queue

executor loop:
    pop from queue → execute callback
```

**Benefits**:
- No per-cycle topology scan — $O(1)$ dispatch
- No busy-wait — wakes only when data arrives
- Lower CPU usage, lower jitter, better real-time behavior

**Caveat**: As of Jazzy, EventsExecutor is still considered experimental in some DDS implementations. Production real-time systems often stick with `StaticSingleThreadedExecutor` + CPU pinning.

</details>

## Intuition

| Concept | Analogy |
|---------|---------|
| QoS | A **shipping SLA** — you choose between express (Reliable, guaranteed delivery, higher latency) and economy (Best-effort, faster but packages may get lost). Mismatched expectations between sender and receiver = the package never ships |
| Executor | The **restaurant floor manager** — decides which waiter (thread) handles which table (callback). A single-threaded executor is one waiter serving all tables sequentially; a multi-threaded executor is a team |
| Callback Group | **Collision avoidance rules** for the waiters — MutuallyExclusive means only one waiter enters the kitchen at a time (no collisions, no locks needed); Reentrant means multiple waiters can enter simultaneously (faster but they must coordinate) |
| Deadline policy | A **dead man's switch** — if the sensor does not report within the deadline, the system raises an alarm. Without it, a dead sensor looks identical to "no obstacles ahead" |
| Durability: Transient local | A **whiteboard in a meeting room** — late arrivals can read what was written before they joined |

**Observable in a robot system**:
- Set Reliable QoS on a 30 Hz LiDAR topic over WiFi → `ros2 topic hz` shows **erratic drops to 5–10 Hz** with occasional burst catch-ups (head-of-line blocking as DDS retransmits lost packets)
- Switch to `SensorDataQoS` (Best-effort) → steady 30 Hz with occasional dropped scans, but the latest data is always fresh
- Run two heavy callbacks (SLAM + planning) on `SingleThreadedExecutor` → one starves; switch to `MultiThreadedExecutor` with separate `MutuallyExclusiveCallbackGroup` per callback → both run at full rate without data races

**Simulator scenario (Gazebo / Isaac Sim)**:
- Spawn a mobile robot with a noisy WiFi link (add packet loss in Gazebo network plugin). Watch Nav2 costmap freeze when using Reliable QoS. Switch to Best-effort — the costmap updates again, occasionally with a missing scan (acceptable for a probabilistic costmap).

## Implementation Link

**Three typical engineering scenarios**:

1. **Configuring QoS for a multi-sensor robot**: LiDAR and camera use `SensorDataQoS` (Best-effort, Keep last 5) for freshness. The emergency stop topic uses `ReliableQoS` with Deadline set to 100 ms — if no heartbeat arrives within 100 ms, the safety node triggers a stop. Map server uses Transient local so that nodes starting late still receive the last published map.

2. **Isolating real-time control from heavy perception**: The control node runs on a dedicated `StaticSingleThreadedExecutor` pinned to an isolated CPU core (`isolcpus` + `taskset`). Perception nodes run on a separate `MultiThreadedExecutor`. This prevents a 200 ms SLAM update from blocking the 1 kHz joint controller.

3. **Multi-rate timer + subscriber without data races**: A node has a 100 Hz timer callback and a 30 Hz subscriber callback that both read/write a shared state variable. Place both in the same `MutuallyExclusiveCallbackGroup` on a `MultiThreadedExecutor`. The executor guarantees they never run simultaneously — zero locks, zero races.

```cpp
// QoS configuration skeleton
#include <rclcpp/rclcpp.hpp>

class SensorNode : public rclcpp::Node {
public:
  SensorNode() : Node("sensor_node") {
    // Best-effort for high-frequency sensor data
    auto sensor_qos = rclcpp::SensorDataQoS();

    // Reliable + Deadline for safety-critical heartbeat
    auto safety_qos = rclcpp::QoS(10)
      .reliable()
      .deadline(std::chrono::milliseconds(100));

    lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", sensor_qos,
      [this](sensor_msgs::msg::LaserScan::SharedPtr msg) { /* ... */ });

    heartbeat_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/heartbeat", safety_qos,
      [this](std_msgs::msg::Bool::SharedPtr msg) { /* ... */ });
  }
};
```

```cpp
// Executor + Callback Group skeleton
#include <rclcpp/rclcpp.hpp>

class MultiRateNode : public rclcpp::Node {
public:
  MultiRateNode() : Node("multi_rate_node") {
    // Both callbacks share state → MutuallyExclusive = safe
    cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    auto sub_opts = rclcpp::SubscriptionOptions();
    sub_opts.callback_group = cb_group_;

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),  // 100 Hz
      [this]() { update_control(shared_state_); },
      cb_group_);

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {
        shared_state_.linear_x = msg->linear.x;  // safe: mutually exclusive
      },
      sub_opts);
  }

private:
  rclcpp::CallbackGroup::SharedPtr cb_group_;
  // ...
};

// Main: use MultiThreadedExecutor to enable callback group scheduling
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MultiRateNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
}
```

<details>
<summary>Deep dive: Complete real-time executor isolation with CPU pinning (C++)</summary>

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sched.h>
#include <pthread.h>

class RealtimeControlNode : public rclcpp::Node {
public:
  RealtimeControlNode() : Node("rt_control") {
    // Use MutuallyExclusive group for all control callbacks
    rt_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    timer_ = create_wall_timer(
      std::chrono::microseconds(1000),  // 1 kHz
      std::bind(&RealtimeControlNode::control_callback, this),
      rt_group_);
  }

private:
  void control_callback() {
    // Pre-allocated buffers only — zero heap allocation here
    // Read sensors → compute PID → publish joint commands
  }

  rclcpp::CallbackGroup::SharedPtr rt_group_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto rt_node = std::make_shared<RealtimeControlNode>();

  // StaticSingleThreadedExecutor: scans topology once, lowest overhead
  rclcpp::executors::StaticSingleThreadedExecutor rt_executor;
  rt_executor.add_node(rt_node);

  // Pin executor thread to isolated CPU core 3
  std::thread rt_thread([&rt_executor]() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);  // Core 3 reserved via isolcpus=3 in GRUB
    pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset);

    // Set SCHED_FIFO for real-time priority
    struct sched_param param;
    param.sched_priority = 80;
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    rt_executor.spin();
  });

  // Perception runs on a separate MultiThreaded executor, other cores
  // auto perception_node = ...
  // rclcpp::executors::MultiThreadedExecutor perception_executor;
  // perception_executor.add_node(perception_node);
  // perception_executor.spin();  // on default cores

  rt_thread.join();
  rclcpp::shutdown();
}
```

**Key points**:
- `isolcpus=3` in GRUB config prevents the Linux scheduler from placing other tasks on core 3
- `SCHED_FIFO` with priority 80 ensures the control callback preempts normal processes
- `StaticSingleThreadedExecutor` avoids per-cycle topology scanning overhead
- All memory must be pre-allocated before `spin()` — see Chapter 01 for memory pool patterns

</details>

## Common Misconceptions

1. **Not setting Deadline → silently accepting a dead sensor** — without a Deadline policy, the subscriber has no way to know the publisher stopped. A dead LiDAR looks exactly like "empty scan" to downstream nodes. **Avoid**: always set Deadline on safety-critical topics; register a `DeadlineMissedEvent` callback to trigger emergency behavior.

2. **Using Reliable QoS on high-frequency sensors** — Reliable transport retransmits lost packets, which causes **head-of-line blocking**: one lost packet stalls the entire queue until retransmitted. On a 30 Hz LiDAR over WiFi, this can spike latency from 33 ms to 200+ ms. **Avoid**: use `SensorDataQoS` (Best-effort) for sensors where freshness matters more than completeness.

3. **Assuming more executor threads = more performance** — setting thread count higher than the number of CPU cores causes **oversubscription**: context-switch overhead eats the parallelism gains. On a 4-core Jetson, 8 executor threads can be slower than 4. **Avoid**: match thread count to available cores; use `htop` to verify no oversubscription.

4. **Wrong Callback Group causing self-deadlock** — placing a service call and a timer callback in the same `MutuallyExclusiveCallbackGroup` on a `SingleThreadedExecutor`: the timer calls the service, but the service response callback cannot execute because the timer callback still holds the group lock. **Avoid**: put the service client callback in a separate `ReentrantCallbackGroup`, or use async service calls.

## Situational Questions

<details>
<summary>Q1 (easy): Nav2 costmap updates are delayed by ~100 ms even though the LiDAR publishes at 30 Hz. What do you check first?</summary>

**Reasoning chain**:
1. **Symptom**: 100 ms delay on a 33 ms period topic → data is arriving late, not just slow processing
2. **First suspect**: QoS mismatch. Run `ros2 topic info /scan -v` — check if the publisher uses Best-effort while the subscriber demands Reliable, or vice versa
3. **If both are Reliable**: head-of-line blocking on a lossy link (WiFi). One lost packet stalls subsequent deliveries. Switch the LiDAR subscriber to `SensorDataQoS` (Best-effort, Keep last 5)
4. **Verify fix**: `ros2 topic delay /scan` should drop from ~100 ms to ~1-2 ms (network latency only)
5. **Edge case**: if delay persists, check History depth — a depth of 1 with a slow subscriber means samples are overwritten before processing

**What the interviewer wants to hear**: immediate instinct to check QoS compatibility with `ros2 topic info -v`, understanding of head-of-line blocking, and the correct fix (SensorDataQoS for sensors).

</details>

<details>
<summary>Q2 (medium): A node has a 100 Hz timer and a 30 Hz subscriber that both modify a shared trajectory buffer. Intermittent segfaults appear under load. What is the root cause and fix?</summary>

**Reasoning chain**:
1. **Symptom**: intermittent segfault + two callbacks modifying shared state = classic **data race**
2. **Root cause**: if using `MultiThreadedExecutor` with both callbacks in the default callback group (which is `MutuallyExclusive` by default — but only if they are in the **same** group). If no explicit group is assigned, each callback may end up in separate implicit groups, allowing concurrent execution
3. **Diagnosis**: run with ThreadSanitizer (`colcon build --cmake-args -DCMAKE_CXX_FLAGS="-fsanitize=thread"`) to confirm the race location
4. **Fix**: explicitly create a `MutuallyExclusiveCallbackGroup` and assign both the timer and the subscriber to it. The executor will guarantee sequential execution — **no mutex needed**
5. **Alternative (worse)**: add a `std::mutex` around the shared buffer — works but adds lock contention and risk of priority inversion in real-time contexts

**What the interviewer wants to hear**: data race diagnosis → callback group as the ROS 2-idiomatic solution (not raw mutexes) → knows how to verify with ThreadSanitizer.

</details>

<details>
<summary>Q3 (medium-hard): You are deploying 8 robots on the same WiFi network. Discovery takes 30+ seconds and topic throughput degrades. Diagnose and fix.</summary>

**Reasoning chain**:
1. **Symptom**: slow discovery + throughput degradation with many robots → **multicast broadcast storm**
2. **Root cause**: default DDS discovery uses SPDP multicast — each robot announces itself to all others. With 8 robots, each with ~20 topics, the discovery traffic grows quadratically: $O(n^2)$ participant pairs × $O(m)$ endpoints each
3. **Quick fix**: assign different `ROS_DOMAIN_ID` to each robot to isolate discovery scopes. But this prevents inter-robot communication
4. **Better fix**: use **Fast DDS Discovery Server** — a centralized discovery broker that replaces multicast with unicast:
   ```bash
   # On one machine (the server):
   fastdds discovery -i 0 -l 192.168.1.1 -p 11811
   # On each robot:
   export ROS_DISCOVERY_SERVER=192.168.1.1:11811
   ```
   Discovery traffic drops from $O(n^2)$ to $O(n)$. Robots can still communicate across topics.
5. **Additional**: reduce DDS participant count by using `rclcpp_components` to load multiple nodes into a single process (one DDS participant per process, not per node)

**What the interviewer wants to hear**: understands the multicast scaling problem, knows Discovery Server as the production solution, and can articulate the $O(n^2) \to O(n)$ improvement.

</details>

<details>
<summary>Q4 (hard): Design the QoS and executor architecture for a mobile manipulator with: 3D LiDAR (10 Hz), stereo camera (30 Hz), 6-DoF arm controller (500 Hz), and Nav2 (10 Hz planning cycle). The robot operates over WiFi with 5% packet loss.</summary>

**Reasoning chain**:
1. **QoS design by topic criticality**:
   - LiDAR + camera: `SensorDataQoS` (Best-effort, Keep last 5) — 5% packet loss is acceptable; freshness > completeness
   - Arm joint commands `/joint_commands`: Reliable, Depth 1 — every command must arrive; depth 1 ensures only the latest is queued
   - Nav2 goal / plan: Reliable, Transient local — plans change infrequently; late-joining nodes need the current plan
   - Emergency stop `/e_stop`: Reliable, Deadline 50 ms — if no heartbeat in 50 ms, arm freezes
2. **Executor isolation**:
   - **Arm controller**: `StaticSingleThreadedExecutor` pinned to isolated core (core 3, `isolcpus=3`), `SCHED_FIFO` priority 80. Must hit 500 Hz with < 100 us jitter
   - **Perception pipeline** (LiDAR + camera + SLAM): `MultiThreadedExecutor` with 3 threads on cores 0-2. Each heavy callback in its own `MutuallyExclusiveCallbackGroup`
   - **Nav2**: separate `SingleThreadedExecutor` — planning is sequential by nature
3. **Callback Group design**:
   - Arm controller: single `MutuallyExclusiveCallbackGroup` containing the 500 Hz timer + joint state subscriber — guaranteed sequential, no locking
   - Perception: separate groups per pipeline stage so SLAM does not block obstacle detection
4. **WiFi mitigation**: Fast DDS Discovery Server to eliminate multicast. Consider DDS-over-TCP for reliable topics to avoid UDP retransmission storms

**What the interviewer wants to hear**: systematic QoS assignment by criticality, executor-per-domain isolation with CPU pinning, callback group design that eliminates races without mutexes, and WiFi-specific DDS tuning.

</details>

## Interview Angles

1. **QoS compatibility debugging** — tests whether you can diagnose "no data" problems. **Bring out with**: "My first move when a topic shows no data is `ros2 topic info -v` to check QoS compatibility between publisher and subscriber. The most common trap is Best-effort pub with Reliable sub — it silently fails with zero error messages."

2. **Executor isolation for real-time** — tests system-level architecture thinking. **Bring out with**: "I separate real-time control onto a `StaticSingleThreadedExecutor` pinned to an isolated CPU core with `SCHED_FIFO`. This guarantees the control loop cannot be starved by perception callbacks, regardless of SLAM computational load."

3. **Callback Groups as the idiomatic concurrency primitive** — tests ROS 2 depth. **Bring out with**: "Instead of manually managing mutexes, I use `MutuallyExclusiveCallbackGroup` to serialize callbacks that share state. The executor enforces this at the scheduling level — no locks, no deadlocks, no priority inversion."

4. **Component containers + zero-copy + QoS = production architecture** — tests full-stack integration. **Bring out with**: "In production I load nodes into a single component container with intra-process communication enabled. Combined with `SensorDataQoS` for sensors and executor isolation for control, this gives me zero-copy data flow with real-time guarantees."

5. **Multi-robot discovery scaling** — tests deployment experience. **Bring out with**: "Default DDS multicast discovery scales quadratically with robot count. Beyond 3-4 robots on the same network, I switch to Fast DDS Discovery Server to reduce discovery traffic to linear scaling."

## Further Reading

- **ROS 2 Design Documents: Executors** — the authoritative explanation of WaitSet-based scheduling, callback group semantics, and EventsExecutor motivation; essential for understanding *why* executors work the way they do
- **ROS 2 Documentation: About Quality of Service settings** — official reference for all 7 QoS policies with compatibility tables; bookmark this page
- **Fast DDS Discovery Server documentation** — setup guide for centralized discovery; critical reading before any multi-robot WiFi deployment
- ***Robot Operating System 2: Design, Architecture, and Uses in the Wild* (S. Macenski et al., 2022)** — the Nav2 maintainer's perspective on QoS choices in production navigation stacks
- **ROS 2 Real-Time Working Group resources** — community patterns for `SCHED_FIFO`, `isolcpus`, and executor tuning on PREEMPT_RT kernels
- **Loaned Messages API (ROS 2 docs)** — the next level beyond `unique_ptr` zero-copy: the middleware pre-allocates shared memory buffers, eliminating even the `make_unique` call
- **Lifecycle Nodes (ROS 2 docs)** — managed node state machine (Unconfigured → Inactive → Active → Finalized) that pairs naturally with QoS Deadline for health monitoring
