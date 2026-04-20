---
title: "C++ Memory Model and Atomic Operations"
prerequisites: ["02-cpp-concurrency-sync"]
estimated_time: 45
difficulty: 4
tags: ["cpp", "memory-model", "atomic", "lock-free"]
sidebar_position: 3
---

# C++ Memory Model and Atomic Operations

## You Will Learn

- Precisely define happens-before, modification order, and the six memory orderings — no hand-waving in interviews
- When a lock-free data structure silently corrupts data on ARM but passes every test on x86, know to suspect relaxed ordering and diagnose with `memory_order_acquire/release`
- Choose the right ordering level for any inter-thread communication pattern: relaxed for independent counters, acquire-release for producer-consumer, and seq_cst only when you can justify the cost

## Core Concepts

### The C++ Memory Model

**Precise definition**: The C++ memory model is a contract between the programmer and the compiler/hardware that specifies when writes by one thread become visible to reads by another. Its two foundational ideas are:

1. **Modification order** — for every atomic object, all threads agree on a single total order of modifications (even if they disagree on the interleaving of modifications to *different* objects).
2. **Happens-before** — if operation A *happens-before* operation B, then A's side effects are guaranteed visible to B. Within a single thread this is trivial (program order). Across threads, it is established through *synchronizes-with* relationships (e.g., a release-store synchronizes-with an acquire-load of the same atomic).

### The Six Memory Orderings

| Ordering | Guarantee | Cost on ARM | Cost on x86 |
|----------|-----------|-------------|-------------|
| `relaxed` | Atomicity only; no ordering constraints | Cheapest | Cheapest |
| `consume` | Data-dependency ordering (deprecated in practice) | — | — |
| `acquire` | Nothing scheduled *after* this load can move *before* it | `ldar` instruction | Free (loads are already acquire) |
| `release` | Nothing scheduled *before* this store can move *after* it | `stlr` instruction | Free (stores are already release) |
| `acq_rel` | Both acquire and release on a single RMW operation | Both barriers | Nearly free |
| `seq_cst` | Global total order across all seq_cst operations | Pure load/store = `LDAR`/`STLR` (same as acquire/release); seq_cst RMW (`fetch_add` etc.) adds `DMB ISH` | `mfence` on stores; loads free |

**Physical meaning**: `release` is "seal and ship the envelope" — everything I wrote before this store is finalized. `acquire` is "open the envelope" — everything I read after this load sees the sender's finalized state. `seq_cst` is "everyone submits their exam at the same desk, in a single queue" — expensive, but unambiguous.

### `std::atomic<T>`

Hardware-level atomic read-modify-write on scalar types and small user-defined types (up to the platform's lock-free width, typically 8 or 16 bytes). Larger types fall back to an internal mutex — check `is_lock_free()` at compile time.

### Compare-And-Swap (CAS)

$$
\text{CAS}(\text{expected}, \text{desired}) = \begin{cases} \text{store desired, return true} & \text{if current} == \text{expected} \\ \text{update expected, return false} & \text{otherwise} \end{cases}
$$

**Physical meaning**: "If nobody else has touched this value since I last looked, swap it to my new value; otherwise tell me what it is now so I can retry."

- `compare_exchange_weak` — may spuriously fail (must be used in a loop). Compiles to a tighter instruction sequence on LL/SC architectures (ARM, RISC-V).
- `compare_exchange_strong` — only fails when the value genuinely differs. Slightly more expensive on LL/SC but does not require a retry loop for single-attempt logic.

### `std::atomic_flag`

The **only** type the standard guarantees is always lock-free. Pre-C++20 it exposed only `test_and_set()` and `clear()`; **C++20 added `test()`, `wait()`, `notify_one()`, and `notify_all()`**, enabling non-busy-wait spinlocks. Still no load/copy/assign — deliberately minimal. Use it to build spinlocks or one-shot signal flags.

**Position in the Sense → Plan → Control Loop**:
- **Control ↔ Planning boundary**: Acquire-release on atomic pointers enables lock-free, wait-free data handoff between a high-frequency control loop (1 kHz+) and a lower-frequency planner (10–100 Hz) without mutex jitter
- **Perception → Planning**: Sensor fusion pipelines use atomic pointer swaps (double/triple buffer) so the reader always gets a consistent snapshot without blocking the writer
- **Critical on ARM**: Most robot CPUs (Cortex-A, Jetson) have weak memory ordering — on ARMv8 the cost of `seq_cst` shows up primarily on RMW operations (`fetch_add`, CAS) and in the compiler's reordering freedom, not on plain load/store (which is `LDAR`/`STLR`, same as acquire/release). On ARMv7 and for RMW-heavy paths, `seq_cst` can burn cycles a 1 kHz loop cannot afford

$$
\text{If } A \xrightarrow{\text{happens-before}} B, \text{ then all side effects of } A \text{ are visible to } B
$$

**One-liner**: "The memory model tells you exactly when a write in one thread becomes visible in another — get it wrong and your lock-free code works on x86, crashes on ARM."

<details>
<summary>Deep dive: How acquire-release builds happens-before across threads</summary>

### The synchronizes-with chain

Consider a producer-consumer pair:

```cpp
// Shared state
std::atomic<bool> flag{false};
int data = 0;  // non-atomic!

// Thread A (producer)
data = 42;                                    // (1) ordinary write
flag.store(true, std::memory_order_release);  // (2) release store

// Thread B (consumer)
while (!flag.load(std::memory_order_acquire)) {}  // (3) acquire load
assert(data == 42);  // (4) guaranteed to see 42
```

**Why this works, step by step**:

1. Within Thread A: (1) *sequenced-before* (2) — program order within a thread.
2. (2) *synchronizes-with* (3) — a release-store on `flag` synchronizes-with an acquire-load that reads the stored value.
3. By transitivity: (1) *happens-before* (4).
4. Therefore the non-atomic write `data = 42` is guaranteed visible to Thread B's read.

### What relaxed would break

If both operations used `memory_order_relaxed`, there is no synchronizes-with relationship. The compiler and CPU are free to reorder (1) after (2), or make (1) invisible to Thread B when it reads `data`. The assert could fire — and on ARM, it *will* fire under load.

### The cost of seq_cst on ARM

The accepted C/C++ → ARMv8 mapping (used by GCC and Clang) is:

```
// ARM64 assembly for seq_cst store:
stlr x0, [x1]  // same instruction as release store

// ARM64 assembly for seq_cst load:
ldar x0, [x1]  // same instruction as acquire load

// ARM64 assembly for seq_cst RMW (e.g. fetch_add):
ldaxr w2, [x1]    // load-acquire-exclusive
add   w2, w2, w3
stlxr w4, w2, [x1]  // store-release-exclusive (retries on failure)
// plus a DMB ISH if compiler picks the stronger mapping

// ARM64 assembly for release store:
stlr x0, [x1]  // single instruction, barrier baked in
```

Concretely: on ARMv8, **pure seq_cst load/store costs nothing beyond acquire/release** — both map to `LDAR`/`STLR`. The `DMB ISH` cost (~20–40 ns on Cortex-A72) shows up only for seq_cst RMW (`fetch_add`, `exchange`, CAS) and for ARMv7 (pre-`LDAR`/`STLR`). seq_cst can still hurt on ARMv8 in a different way: it restricts the compiler from reordering instructions across other atomic operations, which can reduce scheduling freedom. Verify what you care about: paste `std::atomic<int>::store(x, memory_order_seq_cst)` with `-march=armv8-a -O2` into Godbolt — you will see a single `stlr`.

### Fences vs per-variable ordering

`std::atomic_thread_fence(memory_order_release)` is a *global* barrier: it orders all preceding writes (atomic and non-atomic) before all subsequent atomic stores. Useful when you have multiple relaxed writes that should all be "sealed" together before publishing a flag.

```cpp
data1 = x;
data2 = y;
data3 = z;
std::atomic_thread_fence(std::memory_order_release);  // fence: everything above is visible
flag.store(1, std::memory_order_relaxed);              // cheap store, fence did the work
```

</details>

<details>
<summary>Deep dive: The ABA problem in lock-free data structures</summary>

### What ABA is

CAS compares bit patterns, not identity. If a pointer goes through the sequence A → B → A (the node at address A is freed and a new node is allocated at the same address), CAS sees the original value and succeeds — but the data structure's invariants may be broken.

### Concrete example: lock-free stack

```
Initial stack: top → Node_A → Node_B → null

Thread 1: reads top = Node_A, next = Node_B
          (gets preempted before CAS)

Thread 2: pops Node_A (top → Node_B)
          pops Node_B (top → null)
          pushes Node_C at address of old Node_A (reuse!)
          top → Node_C(at addr A) → null

Thread 1: resumes, CAS(expected=A, desired=B) succeeds!
          top → Node_B → ??? (Node_B was already freed)
          → SEGFAULT or silent corruption
```

### Solutions

| Technique | Mechanism | Trade-off |
|-----------|-----------|-----------|
| **Tagged pointer** | Pack a monotonic counter into unused pointer bits; CAS on pointer+tag | Simple; limited counter width (16 bits on x86-64 with 48-bit addresses) |
| **Hazard pointers** | Each thread publishes which nodes it is currently reading; reclamation skips those | Bounded memory overhead; non-trivial implementation |
| **Epoch-based reclamation** | Threads advance through epochs; memory freed only when all threads have passed | Simpler than hazard pointers; one stalled thread blocks all reclamation |
| **Split reference counting** | External + internal ref count; free when both reach zero | Used in `std::shared_ptr` control blocks |

For robotics: hazard pointers or epoch-based reclamation are the pragmatic choices for lock-free sensor buffers where nodes get recycled frequently.

</details>

## Intuition

| Concept | Analogy |
|---------|---------|
| Memory model | Traffic laws — cars (threads) can drive in any order, but the law (model) defines who has right-of-way at intersections (shared data) |
| `release` store | Sealing an envelope and dropping it in the mailbox — everything you wrote is finalized the moment you let go |
| `acquire` load | Opening the envelope — from this point on you see everything the sender finalized before sealing |
| `seq_cst` | A single exam submission desk — everyone queues, global order is unambiguous, but the queue is slow |
| `relaxed` | Each professor grades independently — fast, but you cannot infer submission order across subjects |
| CAS | "I think the shelf has item X; if it still does, swap it for Y; if someone changed it, tell me what's there now" |
| ABA problem | You parked in spot #7, left, came back to find a *different* car in spot #7 — the spot number matches, but the car (data) is not what you expect |

**Observable in a robot system**:
- Lock-free double buffer with `relaxed` ordering → works perfectly on x86 dev machine → **sporadic garbage readings on Jetson** (ARM weak ordering)
- Switching to `acquire`/`release` on the pointer swap → readings become consistent (the ordering semantics were what was missing, not barrier count)
- Using `seq_cst` on RMW-heavy paths on ARM → control loop jitter increases under load; switching to targeted `acquire`/`release` on CAS loops eliminates the extra `DMB ISH` and restores headroom

## Implementation Link

**Three typical engineering scenarios**:

1. **SPSC Ring Buffer for sensor data**: The producer writes payload with ordinary (non-atomic) stores, then advances the write index with `memory_order_release`. The consumer loads the write index with `memory_order_acquire`, then reads the payload. The acquire-release pair creates a happens-before edge — the consumer is guaranteed to see the complete payload. Internal payload writes can be relaxed or non-atomic.

2. **Lock-free triple buffer for cross-rate transfer**: A 30 Hz planner writes a `TargetCommand` to the back buffer, then swaps the atomic pointer with `release`. A 1 kHz controller loads the pointer with `acquire` and reads the latest command. Three buffers ensure the writer never blocks the reader and the reader always gets a complete, consistent snapshot.

3. **Wait-free multi-sensor fusion**: Each sensor thread writes its latest reading into a per-sensor double buffer, then does an atomic pointer swap (`release`). The fusion thread loads all sensor pointers (`acquire`) and computes the fused estimate. Each sensor is independent — no global lock, no priority inversion.

```cpp
// Lock-free triple buffer skeleton (control ↔ planner)
struct TargetCommand { double positions[7]; double velocities[7]; };

class TripleBuffer {
  TargetCommand buffers_[3];
  std::atomic<int> latest_{0};  // index of most recently written buffer
  int write_idx_ = 1;           // only accessed by writer thread

public:
  // Called by planner thread (30 Hz)
  TargetCommand& start_write() { return buffers_[write_idx_]; }
  void finish_write() {
    int prev = latest_.exchange(write_idx_, std::memory_order_release);
    write_idx_ = prev;  // reclaim the old "latest" as next write target
  }

  // Called by control thread (1 kHz) — wait-free, never blocks
  const TargetCommand& read() const {
    int idx = latest_.load(std::memory_order_acquire);
    return buffers_[idx];
  }
};
```

<details>
<summary>Deep dive: Complete SPSC ring buffer with acquire-release (C++17)</summary>

```cpp
#include <atomic>
#include <array>
#include <cstddef>
#include <optional>

template <typename T, std::size_t N>
class SPSCRingBuffer {
  static_assert((N & (N - 1)) == 0, "N must be a power of 2 for fast modulo");

  alignas(64) std::array<T, N> buffer_{};
  alignas(64) std::atomic<std::size_t> write_pos_{0};  // cache-line isolated
  alignas(64) std::atomic<std::size_t> read_pos_{0};   // from each other

public:
  // Producer only — returns false if full
  bool try_push(const T& item) {
    const auto w = write_pos_.load(std::memory_order_relaxed);  // only producer reads this
    const auto r = read_pos_.load(std::memory_order_acquire);   // need to see consumer's progress
    if (w - r >= N) return false;  // full

    buffer_[w & (N - 1)] = item;  // ordinary write — ordered by the release below
    write_pos_.store(w + 1, std::memory_order_release);  // publish: "item is ready"
    return true;
  }

  // Consumer only — returns nullopt if empty
  std::optional<T> try_pop() {
    const auto r = read_pos_.load(std::memory_order_relaxed);   // only consumer reads this
    const auto w = write_pos_.load(std::memory_order_acquire);  // see producer's progress
    if (r >= w) return std::nullopt;  // empty

    T item = buffer_[r & (N - 1)];  // ordinary read — ordered by the acquire above
    read_pos_.store(r + 1, std::memory_order_release);  // publish: "slot is free"
    return item;
  }
};

// Usage: IMU data from sensor thread → control thread
// SPSCRingBuffer<ImuReading, 64> imu_queue;
// Sensor callback: imu_queue.try_push(reading);
// Control loop:    if (auto r = imu_queue.try_pop()) process(*r);
```

Key design choices:
- `alignas(64)` on both indices prevents false sharing between producer and consumer cores
- Power-of-2 size enables bitwise AND instead of modulo for index wrapping
- Only the cross-thread index loads use `acquire`; same-thread index loads use `relaxed`

</details>

## Common Misconceptions

1. **Thinking `seq_cst` is free on x86 so you can use it everywhere** — x86 stores are naturally `release` and loads are naturally `acquire`, so `acquire`/`release` is indeed free. But `seq_cst` *stores* still require an `mfence` instruction (~20 ns). On ARMv8, seq_cst load/store maps to the same `LDAR`/`STLR` as acquire/release — but seq_cst RMW (`fetch_add`, `compare_exchange_*`) picks up a `DMB ISH` full barrier, and seq_cst globally restricts reordering across atomics. **Rule**: default to `acquire`/`release` for producer-consumer; reach for `seq_cst` only when you need a global total order across multiple atomics and can articulate *why*.

2. **Confusing `volatile` with `atomic`** — `volatile` tells the compiler "don't optimize away this read/write" (for memory-mapped I/O). It provides **zero** atomicity, **zero** ordering guarantees, and **zero** cross-thread visibility semantics. On MSVC, `volatile` historically implied acquire-release, which tricked a generation of Windows developers into thinking it was safe. It is not. Use `std::atomic`.

3. **Using `compare_exchange_weak` without a loop** — Weak CAS can spuriously fail on LL/SC architectures (ARM, RISC-V) even when the expected value matches, because the LL/SC reservation was lost (context switch, cache eviction, nearby store). If you use `weak` outside a retry loop, your code silently drops updates under contention. **Rule**: `weak` = always in a `while` loop; `strong` = acceptable for single-attempt logic.

4. **Believing `relaxed` is unsafe** — `relaxed` is perfectly safe for operations that only need atomicity, not ordering: per-thread statistics counters, progress indicators, approximate metrics. The danger is using `relaxed` when you need to establish ordering between an atomic flag and non-atomic payload data — that requires `acquire`/`release`.

## Situational Questions

<details>
<summary>Q1 (medium): You are building a single-producer, single-consumer ring buffer for LiDAR scan data (sensor thread → SLAM thread). What memory orderings do you use and where?</summary>

**Reasoning chain**:

1. **Identify the pattern**: SPSC queue — one writer, one reader. No contention on the same index.
2. **Payload writes are non-atomic**: The sensor thread fills the buffer slot with ordinary writes (point cloud data — too large for atomics).
3. **Write index uses `release`**: After filling the slot, `write_pos_.store(w + 1, memory_order_release)` seals the data — ensures all payload writes are visible before the index advances.
4. **Read of write index uses `acquire`**: The SLAM thread does `write_pos_.load(memory_order_acquire)` — after this load returns, it is guaranteed to see the complete payload.
5. **Same-thread index reads use `relaxed`**: The producer reading its own `write_pos_` does not need cross-thread ordering — it wrote the value itself.
6. **Pitfall to avoid**: Using `relaxed` on the cross-thread index read — on ARM, the SLAM thread could see the advanced index but stale payload data, leading to corrupted point clouds.
7. **False sharing**: Place `write_pos_` and `read_pos_` on separate cache lines (`alignas(64)`) since they are written by different cores.

**What the interviewer wants to hear**: Producer-consumer = acquire-release; payload ordering piggybacks on the index's release-store; same-thread reads are relaxed; cache-line isolation for the two indices.

</details>

<details>
<summary>Q2 (hard): Your lock-free stack (pop operation) segfaults under stress on a 4-core Jetson. The CAS on the top pointer succeeds but the next pointer is garbage. What happened?</summary>

**Reasoning chain**:

1. **Symptom**: CAS succeeds (the top pointer matches expected), but the node's `next` field is invalid. This means the node was recycled or freed between the initial read and the CAS.
2. **Diagnosis**: This is the **ABA problem**. Thread A reads `top = Node_X` and `Node_X->next = Node_Y`. Thread B pops `Node_X`, frees it. Thread C allocates a *new* node at the same address as `Node_X` and pushes it. Thread A's CAS sees `top == Node_X` (same address) and succeeds — but `Node_X->next` is now whatever Thread C wrote, not `Node_Y`.
3. **Why it manifests on Jetson**: ARM's weak ordering makes the window wider for interleaving; also, the allocator aggressively reuses freed addresses.
4. **Fix options**:
   - **Tagged pointer**: Pack a monotonic counter into the upper 16 bits of the pointer (x86-64 uses only 48 bits for addresses). CAS on the combined pointer+tag. Counter increments on every pop, so even if the address recycles, the tag differs.
   - **Hazard pointers**: Each thread publishes which node it is currently examining. The reclaimer skips those nodes. Bounded overhead, but more complex to implement.
   - **Epoch-based reclamation**: Threads enter/exit epochs; freed nodes are only actually reclaimed when all threads have advanced past the epoch in which the node was freed.
5. **Pitfall**: Trying to fix ABA by adding a mutex defeats the purpose of lock-free; trying to fix it with `seq_cst` does not help — ABA is a *logical* problem, not an ordering problem.

**What the interviewer wants to hear**: Immediately names ABA; explains the address-reuse mechanism; proposes tagged pointers as the simplest fix; knows that ordering upgrades do not solve ABA.

</details>

<details>
<summary>Q3 (hard): A 4-core ARM board fuses 3 sensor streams (IMU 1 kHz, LiDAR 10 Hz, camera 30 Hz) into a single state estimate for a 200 Hz planner. Design the lock-free data flow.</summary>

**Reasoning chain**:

1. **Requirements**: Three writers at different rates, one reader at 200 Hz. Reader must never block. Each sensor update must be independently publishable.
2. **Per-sensor double buffer**: Each sensor thread writes its latest reading into the back buffer, then does `atomic<SensorData*>.store(back_ptr, memory_order_release)` to swap. This is wait-free — no CAS needed because only one thread writes to each sensor's atomic pointer.
3. **Fusion thread reads**: At 200 Hz, loads all three sensor pointers with `memory_order_acquire`. Each acquire-load establishes a happens-before with the corresponding sensor's release-store, guaranteeing a consistent snapshot of that sensor's data.
4. **Why not a single shared buffer**: Different update rates mean sensors would contend on a shared lock or atomic — unnecessary. Per-sensor isolation eliminates all cross-sensor contention.
5. **Cache alignment**: Each sensor's atomic pointer + buffer pair should sit on its own cache line (`alignas(64)`) to prevent false sharing between sensor threads.
6. **Why not `seq_cst`**: For the pointer-swap `store`/`load` themselves, ARMv8 emits the same `STLR`/`LDAR` regardless of whether you pick seq_cst or release/acquire — the barrier count is identical. But seq_cst also forces a global total order across *all* seq_cst atomics, which constrains compiler scheduling and adds `DMB ISH` to any RMW path (if you later reach for `exchange`/`fetch_add`). Acquire-release communicates *exactly* the ordering the readers need — producer-to-consumer synchronizes-with — without the cross-variable total-order tax.

**What the interviewer wants to hear**: Per-sensor double buffer with atomic pointer swap; acquire-release is sufficient (no seq_cst); wait-free because each sensor has its own atomic — no CAS contention; cache-line isolation for the per-sensor atomics.

</details>

## Interview Angles

1. **"I never default to `seq_cst`"** — This signals hardware awareness. On x86 the cost is an `mfence` on stores. On ARMv8 pure seq_cst load/store maps to the same `LDAR`/`STLR` as acquire/release — but seq_cst RMW (`fetch_add`, CAS) picks up a `DMB ISH`, and seq_cst globally restricts compiler reordering across atomics. Bring out with: "For producer-consumer patterns I use acquire-release because it expresses exactly the synchronizes-with edge I need, without the global-total-order constraint that restricts the compiler and taxes any RMW paths I add later."

2. **ABA awareness and memory reclamation** — This is the dividing line between someone who read about lock-free and someone who has built it. Bring out with: "CAS compares bit patterns, not identity — so I always consider whether the pointed-to node could be recycled. My go-to fix is a tagged pointer for simple stacks; for queues with high churn, I reach for hazard pointers or epoch-based reclamation."

3. **Knowing when `relaxed` is correct** — Many engineers fear `relaxed` because it sounds dangerous. But for independent counters (statistics, telemetry, progress), `relaxed` gives atomicity with zero ordering cost. Bring out with: "Relaxed is the right choice whenever I only need atomicity — for example, a dropped-frame counter that is read by a monitoring thread but never used to synchronize data flow."

4. **`volatile` is not `atomic`** — A classic gotcha. Bring out with: "volatile suppresses compiler optimization for memory-mapped I/O registers; it provides zero atomicity and zero ordering. For inter-thread communication I always use std::atomic, which gives me both the hardware instruction and the memory-model guarantees."

5. **Weak CAS requires a retry loop** — On LL/SC architectures, `compare_exchange_weak` can spuriously fail due to reservation loss. Bring out with: "I use weak CAS inside retry loops for better codegen on ARM; for single-attempt operations like a one-shot flag, I use strong CAS to avoid silent update drops."

## Further Reading

- ***C++ Concurrency in Action* (A. Williams), Ch5: The C++ Memory Model and Operations on Atomic Types** — the definitive walkthrough of happens-before, synchronizes-with, and all six orderings with diagrams. Start here.
- ***C++ Concurrency in Action*, Ch7: Designing Lock-Free Concurrent Data Structures** — builds a lock-free stack and queue from scratch, covers ABA, hazard pointers, and split reference counting. Read after Ch5.
- **Herb Sutter, "atomic<> Weapons" (CppCon 2012, two-part talk)** — two hours that permanently fix your mental model of acquire-release and seq_cst. The best video introduction to the topic.
- **Preshing on Programming: "An Introduction to Lock-Free Programming"** — blog series with clear diagrams showing how acquire-release fences interact with store buffers on x86 and ARM.
- **Hazard Pointers (Maged Michael, 2004)** and **Epoch-Based Reclamation** — the two main safe memory reclamation schemes for lock-free data structures. Essential reading before building any production lock-free container.
- **MESI Cache Coherence Protocol** — understanding how cache lines bounce between cores makes the cost of `seq_cst` and false sharing viscerally obvious. Any computer architecture textbook (Patterson & Hennessy Ch5) covers this well.
- **Sequence Locks (`seqlock`)** — a lightweight read-optimized synchronization primitive used in the Linux kernel; useful for "one writer, many readers" scenarios where readers can afford to retry on conflict.
