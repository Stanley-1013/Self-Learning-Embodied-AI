---
title: "C++ Multithreading and Synchronization"
prerequisites: ["01-cpp-memory-optimization"]
estimated_time: 45
difficulty: 3
tags: ["cpp", "concurrency", "threading", "synchronization"]
sidebar_position: 2
---

# C++ Multithreading and Synchronization

## You Will Learn

- Precisely define thread vs process, mutex families, and RAII lock guards — no fumbling in interviews
- When a robot system freezes and all topic frequencies drop to zero, know to suspect deadlock and reach for `gdb` + ThreadSanitizer
- Decide when to use `lock_guard` (simple critical section) vs `scoped_lock` (multi-lock deadlock avoidance) vs a lock-free queue (hard-real-time path)

## Core Concepts

### Thread vs Process

**Thread**: lightweight execution unit that **shares memory** with other threads in the same process. Context-switching is cheap (no address-space swap). The flip side: shared memory = data races if you aren't careful.

**Process**: independent address space, isolated by the OS. Communication goes through IPC (pipes, shared memory, sockets) — safe but slow. ROS 2 nodes default to separate processes; composable nodes in a single process trade isolation for speed.

### std::jthread (C++20)

Drop-in upgrade over `std::thread`. Two killer features: **auto-join on destruction** (no more forgetting `.join()`) and a **cooperative stop token** — request graceful shutdown via `stop_token.stop_requested()` instead of volatile flags.

### Mutex Family

| Mutex | When to Use |
|-------|-------------|
| `std::mutex` | Default choice. Non-recursive, non-timed. |
| `std::recursive_mutex` | Same thread may lock multiple times (usually a design smell — prefer refactoring). |
| `std::timed_mutex` | Need `try_lock_for()` / `try_lock_until()` to bail out on timeout. |
| `std::shared_mutex` | Many-reader / one-writer pattern (e.g. costmap read by planner + controller, written by perception). |

### RAII Lock Guards

| Guard | Cost | Capability |
|-------|------|------------|
| `lock_guard` | Lightest | Lock one mutex on construction, unlock on destruction. That's it. |
| `unique_lock` | Slightly heavier | Deferred locking, timed locking, condition-variable interop. |
| `scoped_lock` | Same as `lock_guard` | Lock **multiple** mutexes atomically — deadlock-free by design (`std::lock` algorithm under the hood). |

Rule of thumb: `lock_guard` for single mutex, `scoped_lock` when locking two or more.

### Deadlock

Four conditions must all hold simultaneously: **mutual exclusion**, **hold and wait**, **no preemption**, **circular wait**. Break any one and deadlock is impossible. `scoped_lock` breaks circular wait by acquiring all locks in a consistent global order using a try-and-back-off algorithm internally.

### Condition Variable

Lets a thread **sleep until a condition is met**, waking only when notified — eliminates busy-waiting. Must pair with `unique_lock` and a **`while` loop** (not `if`) to guard against spurious wakeups.

```cpp
std::unique_lock lock(mtx);
cv.wait(lock, [&]{ return !queue.empty(); });  // while-loop predicate form
```

### future / promise / async

The async trio. `promise` writes a value; `future` blocks-then-reads it. `std::async` wraps both: launch a task, get its result later. Caveat: `std::async` with `launch::async` guarantees a new thread; `launch::deferred` does not — it runs lazily in the calling thread.

### Thread Pool

Pre-create a fixed number of threads; feed work items via a shared queue. Eliminates per-task thread creation/destruction overhead. Prevents **oversubscription** (more threads than cores → context-switch storm). C++23 will standardize `std::execution`; until then, roll your own or use libraries.

**Position in the Sense → Plan → Control Loop**:
- **Control** (1 kHz+): hard-real-time thread, typically pinned to a dedicated core with `SCHED_FIFO`; uses lock-free queues — never blocks on a mutex
- **Perception / Planning**: separate threads or callback groups; can tolerate mutex-based synchronization
- **ROS 2**: `MultiThreadedExecutor` + `MutuallyExclusiveCallbackGroup` gives you thread safety without hand-writing mutexes
- **Cross-thread data flow**: perception writes sensor data, planner reads it, controller reads trajectories — each boundary is a synchronization design decision

$$
\text{deadlock risk} \propto \text{number of shared resources} \times \text{number of lock-ordering violations}
$$

**One-liner**: "Correct synchronization is what separates a robot that works from a robot that freezes mid-task — or worse, moves unpredictably."

<details>
<summary>Deep dive: std::lock() try-and-back-off algorithm and lock-free queue fundamentals</summary>

### How std::lock() avoids deadlock

`std::lock(mtx_a, mtx_b)` does not simply lock A then B (which would deadlock if another thread locks B then A). Instead it uses a **try-and-back-off** strategy:

```
1. Lock mtx_a
2. try_lock(mtx_b)
   - Success → done, both locked
   - Failure → unlock mtx_a, yield, retry from step 1 (possibly swapping order)
```

`scoped_lock` wraps this internally — you get deadlock freedom for free.

### Lock-free queue skeleton (single-producer, single-consumer)

```cpp
template<typename T, size_t N>
class SPSCQueue {
  std::array<T, N> buffer_;
  std::atomic<size_t> head_{0};  // written by consumer
  std::atomic<size_t> tail_{0};  // written by producer

public:
  bool push(const T& val) {
    auto t = tail_.load(std::memory_order_relaxed);
    auto next = (t + 1) % N;
    if (next == head_.load(std::memory_order_acquire)) return false;  // full
    buffer_[t] = val;
    tail_.store(next, std::memory_order_release);  // publish
    return true;
  }

  bool pop(T& val) {
    auto h = head_.load(std::memory_order_relaxed);
    if (h == tail_.load(std::memory_order_acquire)) return false;  // empty
    val = buffer_[h];
    head_.store((h + 1) % N, std::memory_order_release);
    return true;
  }
};
```

Key insight: head and tail are on **different cache lines** (they are modified by different threads). `memory_order_acquire/release` provides the minimum necessary ordering without full barriers.

### Thread-safe queue with fine-grained locking

A more general MPMC queue separates head and tail locks using a **dummy node**:

```cpp
struct Node { T data; std::unique_ptr<Node> next; };

class TSQueue {
  std::mutex head_mtx_, tail_mtx_;
  std::unique_ptr<Node> head_;  // guarded by head_mtx_
  Node* tail_;                  // guarded by tail_mtx_

public:
  TSQueue() : head_(std::make_unique<Node>()), tail_(head_.get()) {}

  void push(T val) {
    auto new_node = std::make_unique<Node>();
    Node* new_tail = new_node.get();
    std::lock_guard lk(tail_mtx_);
    tail_->data = std::move(val);
    tail_->next = std::move(new_node);
    tail_ = new_tail;
  }

  std::shared_ptr<T> pop() {
    std::lock_guard lk(head_mtx_);
    if (head_.get() == get_tail()) return {};  // empty
    auto old_head = std::move(head_);
    head_ = std::move(old_head->next);
    return std::make_shared<T>(std::move(old_head->data));
  }
};
```

Head lock and tail lock are independent — push and pop can proceed concurrently.

</details>

## Intuition

| Concept | Analogy |
|---------|---------|
| Mutex | Bathroom lock — one person at a time; everyone else waits in line |
| Deadlock | Two kids each holding one drumstick, refusing to let go, so nobody can play the drum |
| Condition variable | Train station PA announcement — passengers sleep on benches instead of repeatedly checking the platform |
| Thread pool | Taxi company with a fixed fleet — dispatch available cars to riders instead of buying a new car for every trip |
| `scoped_lock` | A bouncer who collects all your tickets at once — no chance of the "I have ticket A, you have ticket B" standoff |

**Observable in a robot system**:
- **Deadlock symptoms**: CPU utilization drops to near zero; `ros2 topic hz` on all topics shows 0 Hz; the robot freezes mid-motion. Diagnosis: `gdb -p <pid>` → `thread apply all bt` reveals two threads each waiting on the other's mutex.
- **Race condition symptoms**: ghost obstacles that teleport in the costmap; planned trajectories that jump erratically between cycles. The bug is intermittent — classic sign of unsynchronized shared memory access.
- **Priority inversion symptoms**: control loop jitter spikes to 10–50 ms while a low-priority logging thread holds a mutex; worst case triggers the safety watchdog and an emergency stop. Fix: use lock-free communication on the real-time path, or apply priority inheritance (`PTHREAD_PRIO_INHERIT`).

## Implementation Link

**Three typical engineering scenarios**:

1. **ROS 2 callback synchronization**: A `TimerCallback` and a `SubscriptionCallback` both write to a shared `std::vector`. Option A: put both callbacks in a `MutuallyExclusiveCallbackGroup` — ROS 2 guarantees they never run concurrently (zero user-side mutex code). Option B: protect the vector with `std::lock_guard<std::mutex>` if callbacks must stay in separate groups.

2. **Perception → Planning double buffer**: Perception updates an occupancy grid at 33 ms; the planner reads it for A* at 50 ms. A mutex would block the planner during grid writes. Instead: perception writes into a back buffer, then does an **atomic `shared_ptr` pointer swap** (`std::atomic_store`). The planner always reads a consistent, complete grid — zero blocking.

3. **4-core ARM, 6 ROS 2 nodes**: Pin the control node to core 3 with `SCHED_FIFO` (hard-real-time). Run perception and planning nodes on cores 0–2 via separate executors. Pass trajectory commands from planner to controller through a **lock-free SPSC queue** — the controller never blocks.

```cpp
// ROS 2: MutuallyExclusiveCallbackGroup — thread safety without hand-written mutex
auto cb_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
auto sub = create_subscription<Twist>("cmd_vel", 10, callback, sub_options);
auto timer = create_wall_timer(10ms, timer_callback, cb_group);
// Both callbacks in the same group → never concurrent → shared data is safe
```

<details>
<summary>Deep dive: Perception → Planning lock-free double buffer (complete C++17 example)</summary>

```cpp
#include <atomic>
#include <memory>
#include <thread>
#include <chrono>

struct OccupancyGrid {
  std::array<uint8_t, 1000 * 1000> cells;
  uint64_t timestamp_ns;
};

class DoubleBuffer {
  std::shared_ptr<const OccupancyGrid> current_;
  std::mutex write_mtx_;  // only serializes writers (if multiple exist)

public:
  DoubleBuffer()
    : current_(std::make_shared<const OccupancyGrid>()) {}

  // Writer (perception thread): build a new grid, then swap atomically
  void publish(std::shared_ptr<const OccupancyGrid> new_grid) {
    std::atomic_store(&current_, std::move(new_grid));
  }

  // Reader (planning thread): grab latest snapshot — wait-free
  std::shared_ptr<const OccupancyGrid> snapshot() const {
    return std::atomic_load(&current_);
  }
};

// Usage sketch
DoubleBuffer grid_buf;

void perception_loop() {
  while (running) {
    auto grid = std::make_shared<OccupancyGrid>();
    fill_from_lidar(*grid);
    grid_buf.publish(std::move(grid));
    std::this_thread::sleep_for(std::chrono::milliseconds(33));
  }
}

void planning_loop() {
  while (running) {
    auto grid = grid_buf.snapshot();  // never blocks
    run_astar(*grid);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}
```

Key design decisions:
- `std::atomic_store` / `std::atomic_load` on `shared_ptr` is lock-free on most x86 implementations (uses a spinlock internally on some platforms — profile on your target ARM).
- The writer allocates a **new** grid each cycle; readers hold `shared_ptr` to the old one until they are done. No data is ever mutated while being read.
- For true lock-free guarantees on ARM, consider C++20 `std::atomic<std::shared_ptr<T>>` or a raw atomic pointer swap with manual lifetime management.

</details>

## Common Misconceptions

1. **Checking `empty()` then calling `top()` on a shared container is safe** — these are two separate operations. Another thread can pop the last element between your `empty()` check and your `top()` call. This is an **interface-level race condition**. **Fix**: use a combined `try_pop()` that returns `std::optional` or a `shared_ptr` under a single lock.

2. **`recursive_mutex` solves reentrant locking cleanly** — it lets the same thread lock multiple times without deadlocking, but this usually masks a design problem (tangled call graph where locking responsibilities are unclear). **Fix**: refactor so each function documents whether it expects the lock to be held by the caller, and use a plain `mutex`.

3. **Using `if` instead of `while` with condition variables** — `condition_variable::wait()` can wake up **spuriously** (no notify happened). An `if` guard checks the predicate once and proceeds on a false wakeup. **Fix**: always use the predicate overload `cv.wait(lock, predicate)`, which loops internally.

4. **`std::async` always launches a new thread** — only `launch::async` does. The default policy is `launch::async | launch::deferred`, meaning the runtime may choose deferred execution (runs in the calling thread when you call `.get()`). **Fix**: specify `std::async(std::launch::async, func)` explicitly if you need true concurrency.

5. **Using `std::mutex` inside a real-time control callback** — a low-priority thread holding the mutex causes **priority inversion**: the high-priority control thread blocks until the low-priority thread releases. Jitter explodes, watchdog triggers. Linux pthread mutex does **not** enable priority inheritance by default, but you can turn it on with `pthread_mutexattr_setprotocol(..., PTHREAD_PRIO_INHERIT)` (the standard PREEMPT_RT escape hatch). **Fix**: use lock-free data structures (atomic pointer swap, SPSC queue) on the real-time path; if you must hold a mutex, enable `PTHREAD_PRIO_INHERIT`.

## Situational Questions

<details>
<summary>Q1 (easy): A ROS 2 node has a TimerCallback and a SubscriptionCallback that both write to the same std::vector. You see occasional segfaults. What do you do?</summary>

**Reasoning chain**:
1. **Symptom**: intermittent segfault on a shared `vector` — textbook data race. Two callbacks run concurrently and one triggers reallocation while the other is iterating.
2. **Option A — ROS 2 native**: move both callbacks into the same `MutuallyExclusiveCallbackGroup`. The executor guarantees they never overlap. Zero mutex code, zero risk of forgetting to lock.
3. **Option B — manual lock**: wrap every access to the vector in `std::lock_guard<std::mutex>`. Works, but you must audit every code path that touches the vector.
4. **Prefer Option A** when both callbacks are in the same node — simpler, less error-prone.
5. **Pitfall**: putting them in a `ReentrantCallbackGroup` does **not** prevent concurrency — it explicitly allows it.

**What the interviewer wants to hear**: knows ROS 2's callback-group threading model and can choose the lightest correct synchronization mechanism.

</details>

<details>
<summary>Q2 (medium): Your robot runs fine for an hour, then all topics drop to 0 Hz and the robot freezes. CPU is near 0%. How do you diagnose?</summary>

**Reasoning chain**:
1. **Symptom**: all progress stops, CPU idle — classic deadlock (threads are sleeping, not spinning).
2. **Immediate diagnosis**: `gdb -p <pid>` → `thread apply all bt`. Look for two or more threads each blocked on `pthread_mutex_lock`, each holding a mutex the other needs.
3. **Identify the lock ordering violation**: thread A locks `mtx_sensor` then waits on `mtx_map`; thread B locks `mtx_map` then waits on `mtx_sensor`.
4. **Fix**: replace the two `lock_guard` calls with a single `std::scoped_lock(mtx_sensor, mtx_map)` — acquires both atomically, breaking the circular-wait condition.
5. **Prevention**: compile with `-fsanitize=thread` (ThreadSanitizer) during development — it detects lock-order inversions and data races at runtime (2–10x slowdown, not for production).
6. **Alternative tool**: Valgrind's Helgrind — no recompilation needed, but 20–50x slowdown.

**What the interviewer wants to hear**: systematic deadlock diagnosis (gdb → backtrace → lock graph) + prevention strategy (scoped_lock + TSan in CI).

</details>

<details>
<summary>Q3 (medium-hard): Perception updates an occupancy grid every 33 ms. The planner runs A* every 50 ms. How do you share the grid without blocking either?</summary>

**Reasoning chain**:
1. **Constraint**: perception must not be delayed by a slow A* run; planner must read a consistent (not half-written) grid.
2. **Rejected approach**: `std::mutex` around the grid — A* can hold the lock for 10+ ms, blocking perception and causing frame drops.
3. **Solution: atomic pointer swap (double/triple buffer)**:
   - Perception writes into a fresh `shared_ptr<const OccupancyGrid>`.
   - On completion, `std::atomic_store(&current_, new_grid)` — O(1) pointer swap.
   - Planner calls `std::atomic_load(&current_)` to grab the latest snapshot — wait-free.
4. **Why `shared_ptr`**: the planner might still be reading the old grid when perception publishes a new one. `shared_ptr` ensures the old grid stays alive until the last reader finishes.
5. **Notification**: after the swap, perception calls `cv.notify_one()` so the planner wakes immediately instead of polling on a fixed timer.
6. **Pitfall**: using a raw pointer swap without lifetime management → dangling pointer when perception overwrites the buffer the planner is still reading.

**What the interviewer wants to hear**: lock-free pointer swap for cross-rate data sharing + `shared_ptr` for safe lifetime management + notification to minimize latency.

</details>

<details>
<summary>Q4 (hard): You have a 4-core ARM board running 6 ROS 2 nodes. The control loop must hit 1 kHz with < 100 μs jitter. Design the threading architecture.</summary>

**Reasoning chain**:
1. **Core allocation**: 6 nodes on 4 cores → oversubscription is inevitable if all share one executor. Dedicate **core 3** exclusively to the control node.
2. **Control node**: single-threaded executor, `SCHED_FIFO` real-time priority, `mlockall()` to prevent page faults. **Zero mutex, zero heap allocation** in the callback.
3. **Remaining nodes**: run on cores 0–2 via one or two `MultiThreadedExecutor` instances. Perception + SLAM can share a `ReentrantCallbackGroup` for parallelism; planning gets its own `MutuallyExclusiveCallbackGroup` since it modifies the global plan.
4. **Planner → Controller data path**: lock-free SPSC queue. Planner pushes trajectory segments; controller pops the latest. No mutex on the real-time path.
5. **Perception → Planner data path**: `shared_ptr` atomic pointer swap (as in Q3). Planner tolerates the occasional stale frame.
6. **Pitfall**: using `std::mutex` anywhere on the control path — a low-priority perception thread holding the lock causes priority inversion → jitter explosion → watchdog e-stop.

**What the interviewer wants to hear**: core pinning + `SCHED_FIFO` for hard-RT + lock-free IPC between rate domains + explicit separation of real-time and non-real-time executors.

</details>

## Interview Angles

1. **Lock granularity and deadlock prevention** — tests whether you think about synchronization design, not just correctness. **Bring out with**: "I choose the coarsest lock that meets latency requirements, then refine. For multi-lock scenarios I always use `scoped_lock` to enforce a consistent acquisition order automatically."

2. **Spurious wakeups and condition variables** — tests attention to subtle concurrency bugs. **Bring out with**: "I never use `if` with `cv.wait` — always the predicate overload, which loops internally. A spurious wakeup with an `if` guard silently corrupts the pipeline."

3. **Thread pool sizing and oversubscription** — tests systems thinking. **Bring out with**: "More threads than cores means context-switch overhead dominates. I size the pool to match available cores, and use callback groups in ROS 2 to control concurrency without spawning extra threads."

4. **Atomics and memory ordering on the real-time path** — tests hardware-software co-design awareness. **Bring out with**: "On the hard-real-time path I replace mutexes with `std::atomic` pointer swaps using `memory_order_acquire/release` — minimum ordering, maximum throughput, zero priority inversion risk."

## Further Reading

- ***C++ Concurrency in Action*, Ch3–4 (Sharing Data, Synchronization)** — the definitive reference for mutex, lock guard, and condition variable patterns
- ***C++ Concurrency in Action*, Ch5 + Ch7 (Memory Model, Atomics, Lock-Free)** — understand `memory_order_acquire/release` and how to build lock-free data structures
- ***C++ Concurrency in Action*, Ch12 (Performance)** — thread pool sizing, oversubscription, and cache-line effects
- **ROS 2 docs: Executors and Callback Groups** — how `MultiThreadedExecutor` and `MutuallyExclusiveCallbackGroup` map to OS threads
- **ThreadSanitizer (TSan)** — `-fsanitize=thread` catches data races and lock-order violations at development time; integrate into CI
- **PREEMPT_RT Linux patch + `SCHED_FIFO`** — turn Linux into a real-time OS for hard-deadline control loops
- **C++20 `std::jthread` + stop tokens** — modern cooperative cancellation that replaces fragile `volatile bool` flags
