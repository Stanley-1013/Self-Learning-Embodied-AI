---
title: "ROS 2 Nodes and Core Communication"
prerequisites: []
estimated_time: 45
difficulty: 3
tags: ["ros2", "communication", "topic", "service", "action", "dds"]
sidebar_position: 4
---

# ROS 2 Nodes and Core Communication

## You Will Learn

- Precisely define Node, Topic, Service, and Action — what each one is, when to pick which, and why mixing them up is an interview red flag
- When a LiDAR subscriber silently receives nothing, know to suspect QoS incompatibility and fix it with the correct preset profile
- Decide between `MultiThreadedExecutor` vs `SingleThreadedExecutor`, and explain why the former does not magically make callbacks thread-safe

## Core Concepts

### Node

The fundamental execution unit in ROS 2. A single process can host multiple Nodes via **Component containers** (composition), eliminating IPC overhead. Each Node owns its own set of publishers, subscribers, services, and timers.

### Topic (Pub/Sub)

**Asynchronous, 1-to-N data streaming.** A Publisher pushes messages to a named channel; any number of Subscribers can listen. No handshake, no reply — fire and forget. Ideal for high-frequency, continuous data: sensor streams (`/scan`, `/camera/image_raw`), velocity commands (`/cmd_vel`), joint states.

### Service (Client/Server)

**Synchronous request-response.** A Client sends a request and blocks (or awaits) until the Server replies. Strictly **1-to-1 per call**. Use for one-shot queries or commands: "get the current robot pose," "switch controller mode," "trigger a calibration routine."

### Action (Goal + Feedback + Result)

**Long-running task with progress feedback and cancellation.** Built on top of Topics and Services internally. The client sends a Goal, receives periodic Feedback, and eventually gets a Result. Use for navigation (`NavigateToPose`), arm trajectory execution, or any task that takes seconds to minutes and might need aborting.

### DDS and QoS

ROS 2 sits on top of DDS (Data Distribution Service), using the RTPS wire protocol. **Quality of Service (QoS)** profiles control reliability, durability, history depth, and deadline. Mismatched QoS between publisher and subscriber is the single most common cause of "I published but nothing arrives."

| Data Type | Recommended QoS | Why |
|-----------|-----------------|-----|
| Sensor streams (LiDAR, camera) | `SensorDataQoS` — Best-effort + Volatile | Dropping an old frame is better than blocking the pipeline |
| Control commands | `SystemDefaultsQoS` or custom Reliable | Every command must arrive; lost messages cause jerky motion |
| Parameter / config | Reliable + Transient Local | Late joiners need the last-known value |

### Executor

The Executor is the event loop that dispatches callbacks. Three flavors:

| Executor | Behavior | Use Case |
|----------|----------|----------|
| `SingleThreadedExecutor` | One callback at a time, in order | Simple nodes, guaranteed no concurrency |
| `MultiThreadedExecutor` | Thread pool runs callbacks in parallel | High-throughput nodes with multiple subscriptions |
| `StaticSingleThreadedExecutor` | Pre-allocates callback list once | Reduced overhead for fixed-topology nodes |

**Critical**: `MultiThreadedExecutor` does **not** make your callbacks thread-safe. You must still protect shared state with `MutuallyExclusiveCallbackGroup` or explicit locking.

### Intra-Process Communication

When multiple Nodes live in the same Component container with `use_intra_process_comms(true)`, messages pass via `unique_ptr` move — **zero-copy, zero serialization**. This is the key optimization for perception pipelines moving multi-MB images or point clouds between nodes at 30+ Hz.

**Position in the Sense - Plan - Control Loop**:
- **Topic** = the arteries of the loop: sensor data flows forward (Sense), velocity/torque commands flow backward (Control)
- **Service** = side-channel queries: "what is the current controller mode?" (does not sit on the main data path)
- **Action** = task-level orchestration: "navigate to waypoint B" (Plan/Control boundary)
- **Intra-process + Component** = the performance multiplier for Sense - Plan hand-off (large data, zero copy)

$$
\text{bandwidth saved} = N_{\text{subscribers}} \times \text{msg\_size} \times f_{\text{Hz}} \quad \text{(with zero-copy, this cost drops to } \approx 0\text{)}
$$

**One-liner**: "When a message can weigh megabytes and frequency is 30 Hz+, the difference between serialized IPC and intra-process zero-copy is the difference between a usable perception pipeline and a CPU-starved one."

<details>
<summary>Deep dive: DDS Discovery, QoS negotiation, and the RTPS wire protocol</summary>

### DDS Discovery

By default, ROS 2 uses **Simple Discovery Protocol (SDP)** — every participant multicasts its existence. In large fleets (10+ robots on the same network), discovery traffic can saturate bandwidth. Solutions:

1. **Discovery Server** (Fast DDS): a centralized broker that participants register with. Eliminates multicast; recommended for multi-robot deployments.
2. **ROS_DOMAIN_ID**: partitions the DDS domain. Robots on different IDs cannot see each other. Simple but coarse.
3. **ROS_LOCALHOST_ONLY=1**: restricts to loopback for single-machine development.

### QoS Compatibility Matrix

When a Publisher and Subscriber have mismatched QoS, DDS silently refuses the connection. The key rules:

| Publisher | Subscriber | Result |
|-----------|------------|--------|
| Reliable | Reliable | OK |
| Reliable | Best-effort | OK (subscriber downgrades) |
| Best-effort | Best-effort | OK |
| Best-effort | Reliable | **INCOMPATIBLE** — subscriber demands guarantees the publisher cannot provide |

This last case is the classic "LiDAR subscriber receives nothing" bug. The default subscriber QoS is Reliable, but sensor publishers typically use Best-effort. Fix: use `rclcpp::SensorDataQoS()` on the subscriber side.

### RTPS Wire Format

RTPS (Real-Time Publish-Subscribe) is the interoperability protocol beneath DDS:
- **Writer** and **Reader** endpoints exchange data via **HistoryCache**
- Reliable mode uses a negative-acknowledgment (NACK) mechanism — the reader requests retransmission of missed samples
- Best-effort mode simply drops missed samples

### Loaned Messages (Zero-Copy Across Processes)

With DDS implementations that support shared-memory transport (e.g., Eclipse Cyclone DDS with iceoryx), you can use `LoanedMessage` to achieve zero-copy even across separate processes:

```cpp
auto loaned_msg = publisher_->borrow_loaned_message();
loaned_msg.get().data = fill_sensor_data();
publisher_->publish(std::move(loaned_msg));
```

The message is allocated in shared memory; the subscriber reads it directly without deserialization.

</details>

<details>
<summary>Deep dive: Executor internals and callback group isolation</summary>

### How the Executor Works

1. The Executor holds a list of "waitable" entities (subscriptions, timers, services, action servers)
2. On each spin iteration, it calls `rcl_wait()` — a blocking call that returns when any entity has work ready
3. Ready callbacks are dispatched to the thread pool (or executed inline for single-threaded)

### Callback Groups

Callback groups control **concurrency policy**:

- **`MutuallyExclusiveCallbackGroup`**: only one callback in this group can execute at a time. Use when callbacks share mutable state.
- **`ReentrantCallbackGroup`**: callbacks can run in parallel, even with themselves. Use only when callbacks are stateless or internally synchronized.

```cpp
auto group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
auto sub_options = rclcpp::SubscriptionOptions();
sub_options.callback_group = group;

subscription_ = create_subscription<Twist>("/cmd_vel", 10, callback, sub_options);
timer_ = create_wall_timer(10ms, timer_callback, group);
// subscription_ callback and timer_callback will NEVER run concurrently
```

### StaticSingleThreadedExecutor Performance

The default `SingleThreadedExecutor` rebuilds the waitable list every spin cycle. The `StaticSingleThreadedExecutor` builds it once at construction. For nodes with a fixed set of subscriptions, this eliminates per-cycle overhead — measured at **~30% lower CPU** in benchmarks with 50+ subscriptions.

</details>

## Intuition

| Concept | Analogy |
|---------|---------|
| Topic | A radio broadcast — the station transmits whether anyone is tuned in or not; multiple listeners hear the same stream |
| Service | A phone call — you dial, wait for the other side to pick up, ask a question, get an answer, hang up |
| Action | Ordering food delivery — you place the order (Goal), get GPS tracking updates (Feedback), can cancel anytime, and eventually receive the food (Result) |
| QoS | Registered mail vs regular mail — registered (Reliable) guarantees delivery with tracking; regular (Best-effort) is faster but might get lost |
| Executor | A receptionist dispatching visitors — single-threaded means one visitor at a time; multi-threaded means multiple meeting rooms running in parallel |
| Intra-process | Handing a folder to the person sitting next to you vs mailing it across town (serialize, transmit, deserialize) |

**Observable in a robot system**:
- Run `ros2 topic hz /scan` and see stable 10 Hz; switch the subscriber QoS to Reliable (while the LiDAR publishes Best-effort) and watch the frequency drop to 0 — QoS incompatibility in action
- Launch two Nodes as separate processes, then reload them in a Component container with intra-process enabled — `htop` CPU usage drops visibly for image-heavy pipelines
- Use `ros2 doctor` or `ros2 topic info -v` to inspect QoS profiles and spot mismatches before they bite

## Implementation Link

**Three typical engineering scenarios**:

1. **High-frequency sensor pipeline**: Camera Node publishes `Image` at 30 Hz via Topic. SLAM and obstacle detection Nodes subscribe. All Nodes loaded into one Component container with `use_intra_process_comms(true)` for zero-copy.

2. **Arm trajectory execution**: A planning Node sends a `FollowJointTrajectory` Action Goal. The arm controller provides 100 Hz Feedback (current joint positions) and a final Result (success/failure + actual final pose). The planning Node can cancel mid-trajectory if an obstacle is detected.

3. **Parameter/mode switching**: A supervisor Node calls a Service to switch the controller from position mode to impedance mode. This is a one-shot command with a response confirming the switch.

```cpp
// Scenario 1: Sensor subscriber with correct QoS
auto sub = create_subscription<sensor_msgs::msg::PointCloud2>(
    "/lidar/points", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        process_pointcloud(msg);  // zero-copy if intra-process
    });
```

```cpp
// Scenario 2: Action client for arm trajectory
auto action_client = rclcpp_action::create_client<FollowJointTrajectory>(
    this, "/arm_controller/follow_joint_trajectory");

auto goal = FollowJointTrajectory::Goal();
goal.trajectory = build_trajectory(target_joints);

auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
send_goal_options.feedback_callback = [](auto, auto feedback) {
    RCLCPP_INFO(get_logger(), "Progress: %.1f%%", feedback->percent_complete);
};
send_goal_options.result_callback = [](auto result) {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        RCLCPP_INFO(get_logger(), "Trajectory complete");
};
action_client->async_send_goal(goal, send_goal_options);
```

```python
# Scenario 3: Service client (Python, async)
cli = node.create_client(SetParameters, '/controller/set_parameters')
req = SetParameters.Request()
req.parameters = [Parameter(name='mode', value='impedance').to_parameter_msg()]
future = cli.call_async(req)
rclpy.spin_until_future_complete(node, future)
```

<details>
<summary>Deep dive: Complete Component container launch with intra-process communication (Python launch file)</summary>

```python
"""Launch file: load Camera, SLAM, and ObstacleDetection as components with intra-process comms."""
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNode
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='perception_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # MultiThreaded for parallel callbacks
        composable_node_descriptions=[
            ComposableNode(
                package='camera_driver',
                plugin='camera_driver::CameraNode',
                name='camera',
                extra_arguments=[{'use_intra_process_comms': True}],
                parameters=[{
                    'fps': 30,
                    'image_width': 1280,
                    'image_height': 720,
                }],
            ),
            ComposableNode(
                package='slam_toolbox',
                plugin='slam_toolbox::SyncSlamNode',
                name='slam',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='obstacle_detector',
                plugin='obstacle_detector::DetectorNode',
                name='obstacle_detector',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    return LaunchDescription([container])
```

**Key points**:
- `component_container_mt` uses `MultiThreadedExecutor` — necessary when SLAM and obstacle detection should run in parallel
- `use_intra_process_comms: True` on every node enables zero-copy `unique_ptr` passing between them
- If you need 1-to-N sharing (Camera to both SLAM and obstacle detection), ROS 2 automatically upgrades to `const shared_ptr` — still zero-copy, just shared read-only access

</details>

## Common Misconceptions

1. **Thinking Topic and Service are interchangeable** — "I will just use a Service to stream LiDAR data." Services are request-response: the client must actively poll, introducing latency and wasting CPU. Topics are push-based and asynchronous. **Rule**: continuous data = Topic; one-shot query = Service; long task = Action. Violating this pattern creates architectures that fight ROS 2's design.

2. **Assuming `MultiThreadedExecutor` makes callbacks thread-safe** — it does not. It merely allows concurrent dispatch. If two callbacks access the same buffer, you get a data race. **Fix**: assign callbacks that share mutable state to a `MutuallyExclusiveCallbackGroup`, or use explicit locks. Run ThreadSanitizer (`-fsanitize=thread`) to catch races before they become intermittent production crashes.

3. **Ignoring QoS and wondering why messages vanish** — the default subscriber QoS is Reliable, but many sensor drivers publish with Best-effort. DDS silently drops the connection on incompatibility — no error message, no warning, just silence. **Fix**: always check `ros2 topic info -v` to see publisher/subscriber QoS; use `rclcpp::SensorDataQoS()` for sensor subscriptions. This is the most common ROS 2 debugging headache.

4. **Doing heavy computation inside a callback** — a callback that takes 50 ms blocks the Executor for 50 ms, starving every other subscription and timer in the same callback group. **Fix**: offload heavy work to a separate thread or use `MultiThreadedExecutor` with properly isolated callback groups. The callback should only receive data, copy/move it to a work queue, and return immediately.

## Situational Questions

<details>
<summary>Q1 (easy): Your LiDAR subscriber receives zero messages despite the publisher running normally. ros2 topic list shows the topic. What do you check?</summary>

**Reasoning chain**:
1. **First suspect: QoS mismatch.** Run `ros2 topic info /scan -v` to inspect the publisher's and subscriber's QoS profiles
2. Most LiDAR drivers publish with **Best-effort** reliability. If your subscriber uses the default **Reliable** QoS, DDS declares them incompatible and silently drops the connection
3. **Fix**: change the subscriber to `rclcpp::SensorDataQoS()` (Best-effort + Volatile + small history depth)
4. **Second suspect**: ROS_DOMAIN_ID mismatch — publisher and subscriber on different DDS domains
5. **Third suspect**: network firewall blocking DDS multicast (common in corporate WiFi or Docker without `--net=host`)

**What the interviewer wants to hear**: "QoS incompatibility is the #1 cause of silent message loss in ROS 2. I would check `ros2 topic info -v` first to compare QoS profiles, then verify DDS domain ID, then check network multicast."

</details>

<details>
<summary>Q2 (medium): You are designing the communication architecture for a robotic arm system. The arm needs: 100 Hz joint state streaming, TCP pose queries on demand, and multi-second trajectory execution with progress feedback. What communication patterns do you use?</summary>

**Reasoning chain**:
1. **100 Hz joint state streaming** — continuous, high-frequency, 1-to-N (controller, visualization, logging all need it) → **Topic** on `/joint_states`. Use `SensorDataQoS` or a custom profile with small history depth to prevent queue buildup
2. **TCP pose query on demand** — one-shot, request-response, infrequent → **Service** on `/get_tcp_pose`. Client asks, server computes FK and replies. No need for a continuous stream
3. **Multi-second trajectory execution** — long-running, needs feedback (current joint error) and cancellation (obstacle detected mid-motion) → **Action** on `/follow_joint_trajectory`. Feedback at ~100 Hz shows progress; result reports final accuracy

**Pitfall to avoid**: using a Topic for the pose query (wasteful — publishes even when nobody needs it) or a Service for trajectory execution (blocks the caller and provides no progress feedback or cancellation).

**What the interviewer wants to hear**: "I match communication pattern to temporal semantics — continuous data gets Topics, one-shot queries get Services, long tasks with feedback get Actions. This is not just convention; it determines whether the system can cancel, monitor, and scale correctly."

</details>

<details>
<summary>Q3 (medium-hard): A MultiThreadedExecutor occasionally crashes with a segfault. The node has a LiDAR callback writing to a shared obstacle map and a timer callback reading from the same map. How do you diagnose and fix?</summary>

**Reasoning chain**:
1. **Symptom**: intermittent segfault under `MultiThreadedExecutor` → classic data race. Two callbacks access shared mutable state concurrently
2. **Diagnosis**: compile with `-fsanitize=thread` (ThreadSanitizer) and reproduce. TSan will report the exact lines with concurrent read/write on the obstacle map
3. **Root cause**: both callbacks are in the default `ReentrantCallbackGroup`, so the executor dispatches them to separate threads simultaneously. The LiDAR callback writes to the map while the timer callback reads — undefined behavior
4. **Fix options** (pick one):
   - **Callback group isolation**: create a `MutuallyExclusiveCallbackGroup` and assign both callbacks to it. The executor will never run them concurrently. Simplest fix, slight throughput reduction
   - **Double buffering**: LiDAR callback writes to a back buffer; atomically swap pointers when complete; timer reads the front buffer. No blocking, maximum throughput
   - **Mutex**: `std::shared_mutex` with shared lock for reads, exclusive lock for writes. Fine-grained but adds complexity

**What the interviewer wants to hear**: "MultiThreadedExecutor enables concurrency but does not provide synchronization. I would use MutuallyExclusiveCallbackGroup as the first fix for simplicity, then consider double buffering if profiling shows the mutual exclusion is a throughput bottleneck."

</details>

<details>
<summary>Q4 (hard): A 4-node perception pipeline (Camera → Preprocessing → Detection → Planning) processes 1.2 MB RGBD images at 30 fps. CPU usage is 95%. How do you optimize?</summary>

**Reasoning chain**:
1. **Diagnose the bottleneck**: at 1.2 MB × 30 Hz × 3 inter-node hops = ~108 MB/s of serialization + deserialization if nodes are in separate processes. Check `perf` or `ros2 topic bw` to confirm
2. **Component container**: load all 4 nodes into a single process using `rclcpp_components`. Enable `use_intra_process_comms(true)` on every node. Messages now pass as `unique_ptr` moves — zero serialization, zero copy
3. **1-to-N optimization**: if Detection and Planning both subscribe to the same preprocessed image, ROS 2 automatically uses `const shared_ptr` for intra-process — one copy, shared read-only
4. **Pre-allocated buffers**: use an object pool in the Camera node so it never calls `new` for image buffers in the hot loop. Recycle buffers after the last downstream consumer finishes
5. **Executor choice**: use `component_container_mt` (MultiThreaded) so Preprocessing and Detection can run in parallel on different cores. Assign them to separate `ReentrantCallbackGroup`s
6. **Verify**: `htop` should show CPU drop from 95% to ~30-40%; `ros2 topic delay` should show reduced latency

**What the interviewer wants to hear**: "The first thing I check is whether the nodes are in separate processes — if so, serialization is the bottleneck. Component composition with intra-process communication eliminates it. Combined with buffer pooling and a MultiThreadedExecutor, this is the standard high-performance perception architecture in ROS 2."

</details>

## Interview Angles

1. **DDS QoS is where ROS 2 debugging starts** — 80% of "it does not work" issues in ROS 2 are QoS mismatches. Bring out with: "When a subscriber receives nothing, my first step is always `ros2 topic info -v` to compare QoS profiles. I know the compatibility matrix — a Best-effort publisher and a Reliable subscriber will silently disconnect."

2. **Component composition + intra-process = zero-copy architecture** — this is the defining performance optimization in ROS 2 perception pipelines. Bring out with: "For any pipeline moving megabyte-scale data at 30+ Hz, I load nodes into a single Component container with intra-process communication. This eliminates serialization entirely and reduces CPU usage by 50-70% in practice."

3. **Executor and callback group design determines real-time behavior** — shows you understand concurrency beyond "just use MultiThreadedExecutor." Bring out with: "I use MutuallyExclusiveCallbackGroup to isolate callbacks that share mutable state, and I know that StaticSingleThreadedExecutor reduces scheduling overhead for fixed-topology nodes. Executor selection is a concurrency design decision, not a checkbox."

4. **Topic / Service / Action — matching temporal semantics** — demonstrates architectural thinking, not just API knowledge. Bring out with: "I choose the communication pattern based on data flow semantics: continuous data is a Topic, one-shot queries are Services, and long-running tasks with feedback are Actions. This is not interchangeable — using the wrong pattern breaks cancellation, monitoring, and scalability."

5. **Launch system as infrastructure-as-code** — shows awareness of deployment, not just node development. Bring out with: "ROS 2 launch files in Python give me conditional logic, parameter namespacing, and component loading — I treat them as the deployment layer that defines the system topology, not just a startup script."

## Further Reading

- **ROS 2 Design Documents (design.ros2.org)** — the authoritative source for why ROS 2 made specific architecture choices (DDS, QoS, executors); essential for understanding the "why" behind the API
- **ROS 2 Demos: Intra-process Communication** — official tutorial with benchmarks showing zero-copy latency improvement; run it locally to see the CPU difference
- **Fast DDS Discovery Server documentation** — explains how to scale multi-robot deployments without multicast storms; critical for real-world fleet scenarios
- **Eclipse Cyclone DDS + iceoryx (Loaned Messages)** — cross-process zero-copy via shared memory; the next step beyond intra-process communication for truly large-scale systems
- **Lifecycle Nodes (ROS 2 Managed Nodes)** — state machine for node startup/shutdown (`Unconfigured → Inactive → Active`); essential for production systems that need graceful error recovery
- ***A Concise Introduction to Robot Programming with ROS 2* (F. Rico)** — practical, hands-on coverage of Nodes, Topics, Services, Actions with working code examples
