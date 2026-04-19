---
title: "Dynamic Obstacle Avoidance and Real-time Replanning"
prerequisites: ["10-basic-path-planning", "11-trajectory-optimization"]
estimated_time: 45
difficulty: 4
tags: ["obstacle-avoidance", "dwa", "teb", "mppi", "reactive"]
sidebar_position: 12
---

# Dynamic Obstacle Avoidance and Real-time Replanning

## You Will Learn

- Describe the "layered planning architecture" precisely in two sentences: how a Global planner (low-frequency, globally optimal) and a Local planner (high-frequency, real-time evasion) cooperate — no vagueness in interviews
- Diagnose "the robot keeps emergency-stopping and rerouting in a crowd but still collides" — you will know the root cause is missing dynamic prediction, and reach for Kalman Filters / LSTMs to predict pedestrian trajectories and write them into a spatiotemporal costmap
- Decide when to use DWA (differential-drive rapid prototyping), TEB (Ackermann non-holonomic constraints), or MPPI (nonlinear high-dimensional systems with GPU parallelism)

## Core Concepts

### Layered Planning Architecture

**Precise Definition**: dynamic obstacle avoidance systems are typically split into two layers — a **Global planner** runs at low frequency (~1 Hz) to find the shortest or safest path on the full map (A\*, Dijkstra), producing coarse waypoints; a **Local planner** runs at high frequency (20–50 Hz) within a local window, considering dynamic obstacles, kinematic constraints, and comfort to generate executable velocity commands or short-horizon trajectory segments.

**Location in the Sense → Plan → Control Loop**:
- **Input**: real-time obstacle positions and velocity estimates from the perception layer (LiDAR scans, depth cameras, tracking module output), the global map (static map + costmap layers), and the current robot state $(x, y, \theta, v, \omega)$
- **Output**: velocity commands $(v, \omega)$ or short-term trajectory waypoints ready for the low-level controller
- **Downstream consumers**: base motion controllers (PID / pure pursuit), safety monitoring (emergency stop), multi-robot coordination modules
- **Loop node**: straddles **planning** and **control** — the local planner reads the latest perception every tick, replans, and forms the high-frequency loop "sense → local plan → velocity command → execute → sense"

**One-line version**: "Dynamic obstacle avoidance is the robot's cerebellar reflex — the global path is GPS navigation, and local planning is the real-time steering wheel."

### Algorithm Comparison

| Algorithm | Core Idea | Best For | Frequency | Kinematic Constraints |
|-----------|-----------|----------|-----------|----------------------|
| **DWA** | sample $(v, \omega)$ in velocity space + cost function evaluation | differential / omnidirectional bases | 20–50 Hz | velocity and acceleration bounds |
| **TEB** | Timed Elastic Band, C² continuous trajectory optimization | Ackermann (car-like), non-holonomic | 10–30 Hz | minimum turning radius, acceleration limits |
| **MPPI** | sampling-based MPC, GPU-parallel thousands of trajectories | nonlinear, high-dimensional systems | 30–100 Hz | arbitrary nonlinear dynamics |

### Minimum Sufficient Math

1. **DWA Cost Function** (multi-objective weighting in velocity space):

$$
G(v, \omega) = \sigma \big[\alpha \cdot \text{heading}(v,\omega) + \beta \cdot \text{dist}(v,\omega) + \gamma \cdot \text{velocity}(v,\omega)\big]
$$

**Physical meaning**: $\text{heading}$ measures alignment toward the goal, $\text{dist}$ measures clearance from the nearest obstacle, and $\text{velocity}$ encourages forward speed. $\alpha, \beta, \gamma$ are tunable weights. Within the admissible velocity window $(v, \omega) \in V_s \cap V_d \cap V_a$ (static limits $\cap$ dynamic limits $\cap$ feasible deceleration), the pair with the highest cost is sent to the base.

2. **Velocity Obstacle (VO)** (collision cone excluding dangerous velocities):

$$
VO_{A|B} = \left\{ \mathbf{v}_A \mid \exists\, t > 0 : \mathbf{v}_A \cdot t \in D(B \oplus (-A)) \right\}
$$

**Physical meaning**: inflate obstacle $B$ by the Minkowski sum with the robot's shape; the velocity cone emanating from the origin represents all velocities that lead to collision. Safe velocities lie outside the cone. RVO (Reciprocal VO) splits the avoidance responsibility equally between both agents, and ORCA linearizes it for millisecond-level solving.

3. **MPPI Trajectory Weighting** (information-theoretic MPC):

$$
w^{(k)} = \exp\!\left(-\frac{1}{\lambda} S(\tau^{(k)})\right), \quad u^* = \frac{\sum_k w^{(k)} u^{(k)}}{\sum_k w^{(k)}}
$$

**Physical meaning**: compute the total cost $S$ for each of $K$ randomly perturbed trajectories $\tau^{(k)}$, then aggregate via Boltzmann-weighted softmax. $\lambda$ is the temperature — smaller means greedier (trusting only the best few), larger means more conservative (averaging all). On a GPU, $K = 2048\text{–}8192$ trajectories roll out in parallel with zero data dependencies.

<details>
<summary>Deep dive: full DWA algorithm flow and velocity window derivation</summary>

**Complete steps per DWA tick**:

1. **Build the velocity search space**:
   - Static window $V_s = \{(v, \omega) \mid v \in [v_{\min}, v_{\max}],\; \omega \in [\omega_{\min}, \omega_{\max}]\}$
   - Dynamic window $V_d = \{(v, \omega) \mid v \in [v_c - \dot{v}_b \Delta t,\; v_c + \dot{v}_a \Delta t],\; \omega \in [\omega_c - \dot{\omega}_b \Delta t,\; \omega_c + \dot{\omega}_a \Delta t]\}$
   - Admissible window $V_a = \{(v, \omega) \mid v \leq \sqrt{2 \cdot \text{dist}(v,\omega) \cdot \dot{v}_b}\}$ (guarantees the robot can stop before collision)
   - Final search space $V = V_s \cap V_d \cap V_a$

2. **Forward simulation**: for each discrete pair $(v_i, \omega_j)$ in $V$, simulate the resulting arc over $T_{\text{sim}}$ (typically 1–3 seconds)

3. **Cost evaluation**:
   - $\text{heading}$: angle between the trajectory endpoint heading and the goal direction (smaller is better)
   - $\text{dist}$: shortest distance along the trajectory to the nearest obstacle (larger is better)
   - $\text{velocity}$: forward speed magnitude (encourages fast movement)

4. **Select optimum**: pick $(v^*, \omega^*)$ that maximizes $G = \alpha \cdot \text{heading} + \beta \cdot \text{dist} + \gamma \cdot \text{velocity}$

**Velocity space discretization**: typical DWA uses a $30 \times 30$ to $50 \times 50$ grid. Finer grids improve solution quality but increase compute. Nav2's DWA uses iterative forward simulation with 20–30 time steps per trajectory.

**Tuning in practice**:
- Narrow corridors: increase $\beta$ (safety clearance weight), decrease $\gamma$ (allow slowdowns)
- Open spaces: increase $\gamma$ (pursue speed), decrease $\beta$
- Crowded environments: add a prediction layer that writes future pedestrian positions into the costmap with decaying cost

</details>

<details>
<summary>Deep dive: MPPI GPU implementation architecture and temperature tuning</summary>

**MPPI GPU parallel architecture**:

```
┌─────────────────────────────────────────┐
│  Host (CPU)                             │
│  ├── Read current state x₀              │
│  ├── Generate control perturbations     │
│  │   δu ~ N(0, Σ)                       │
│  └── Upload x₀ + δu to GPU             │
├─────────────────────────────────────────┤
│  Device (GPU) — K thread blocks          │
│  ├── Each thread: rollout one trajectory │
│  │   ├── x_{t+1} = f(x_t, u_nom + δu_t) │
│  │   └── Accumulate stage cost S_k      │
│  ├── Parallel reduction: find min(S)     │
│  └── Weighted avg: u* = Σ w_k·u_k / Σ w_k│
├─────────────────────────────────────────┤
│  Host: send u* to controller             │
└─────────────────────────────────────────┘
```

**Temperature $\lambda$ tuning**:
- $\lambda \to 0$: approaches arg min, selecting only the lowest-cost trajectory — aggressive but noisy
- $\lambda \to \infty$: all trajectories weighted equally — overly conservative
- Practice: $\lambda \in [0.01, 10]$, typically start at $\lambda = 1$ and binary-search in simulation for the sweet spot between "no collisions" and "no excessive detours"

**Cost function design (common in autonomous driving)**:

$$
S(\tau) = \sum_{t=0}^{T} \left[ q_{\text{goal}} \|x_t - x_{\text{goal}}\|^2 + q_{\text{obs}} \cdot c_{\text{obs}}(x_t) + q_u \|u_t\|^2 + q_{\text{jerk}} \|\Delta u_t\|^2 \right]
$$

- $c_{\text{obs}}$: looked up from a Signed Distance Field (SDF); cost grows exponentially as distance to obstacles shrinks
- $q_{\text{jerk}}$: penalizes abrupt control changes, improving ride comfort

**Common implementations**: NVIDIA Isaac Lab ships a built-in MPPI controller; [MPPI-Generic](https://github.com/ACDSLab/MPPI-Generic) is an open-source C++/CUDA library.

</details>

### Costmap Architecture

The costmap is the critical data structure bridging perception and planning:

| Layer | Purpose | Update Rate |
|-------|---------|-------------|
| **Static Layer** | Static occupancy loaded from the SLAM map | Once |
| **Obstacle Layer** | Real-time obstacles from LiDAR / depth cameras | 10–30 Hz |
| **Inflation Layer** | Exponentially decaying safety gradient around obstacles | Follows Obstacle Layer |
| **Voxel Layer** | 3D voxel grid (handles overhangs, under-table clearance) | 10–30 Hz |

**Physical meaning**: the Inflation Layer's cost decays exponentially from obstacle edges — when the robot center enters a high-cost zone, the physical shell is nearly touching. `inflation_radius` sets the expansion distance, `cost_scaling_factor` controls how steeply cost drops off.

## Intuition

**Analogy: three layers of driving reaction**
- **GPS navigation = Global planner**: tells you "take the highway, then exit onto Route 3" — very low frequency (recomputes every few seconds)
- **Steering = Local planner (DWA/TEB)**: real-time lane-level adjustments to dodge the car that just cut in, 20–50 Hz
- **ABS braking = Reactive safety layer (VO/ORCA/CBF)**: bypasses "planning" entirely and eliminates dangerous velocity commands in velocity space, <5 ms response

**DWA intuition**: imagine standing in the middle of a room and fanning out 900 possible walking paths (different speeds and turn rates). Walk each one a few steps, checking: will I hit something? Am I heading toward the door? Am I moving fast enough? Pick the best path, take one step, then re-fan 900 paths from the new position.

**TEB intuition**: imagine threading a rubber band through pins (obstacles) toward the goal. The rubber band naturally snaps taut into the shortest, smoothest path. TEB places this rubber band in *spacetime* — it avoids not just where obstacles are, but where they *will be* at each moment.

**Simulator observation**: in Gazebo + Nav2, switch between DWA and TEB controllers on the same map and observe:
- DWA is fluid in wide corridors but tends to oscillate in narrow passages
- TEB produces more natural curved turns on Ackermann platforms
- Open `local_costmap` visualization in rviz2 to see how inflation gradients steer path selection

## Implementation Link

**Three representative engineering scenarios**:

1. **ROS 2 Nav2 layered planning**: `bt_navigator` uses a Behavior Tree to coordinate a global planner (NavFn / Smac) with a local controller (DWA / TEB / MPPI). The global path is published as `nav_msgs/Path`; the local controller reads `local_costmap` + global path every tick and outputs `geometry_msgs/Twist` to the base.

2. **Autonomous driving local planning (Autoware / Apollo)**: MPPI or Lattice planner runs at 30–100 Hz in Frenet frame, with cost terms for lane-center deviation, obstacle SDF, comfort (jerk), and traffic rules. The output trajectory feeds a PID / Stanley / MPC tracking controller.

3. **Multi-robot warehouse logistics (ORCA)**: each AGV runs ORCA decentralized reciprocal collision avoidance independently — no central coordinator needed. Per-agent complexity is $O(n)$ ($n$ = nearby neighbors), scaling to 100+ robots. Priority arbitration: loaded AGVs > empty AGVs; intersections use an auction protocol to allocate right-of-way.

**Code skeleton** (Python, Nav2 DWA style):

```python
class DWAPlanner:
    def __init__(self, config: DWAConfig):
        self.config = config  # v_max, w_max, acc_lim, dt, sim_time, weights

    def compute_velocity(
        self,
        state: RobotState,        # (x, y, theta, v, omega)
        goal: Pose2D,             # target pose
        costmap: Costmap2D,       # local costmap
        global_path: list[Pose2D] # global path reference points
    ) -> tuple[float, float]:     # returns (v, omega)
        """DWA core: sample → simulate → evaluate → select best"""
        best_score = -float('inf')
        best_v, best_w = 0.0, 0.0

        for v, w in self._sample_velocities(state):
            traj = self._simulate(state, v, w)
            if self._check_collision(traj, costmap):
                continue
            score = self._evaluate(traj, goal, costmap, global_path)
            if score > best_score:
                best_score, best_v, best_w = score, v, w

        return best_v, best_w
```

<details>
<summary>Deep dive: complete Nav2 MPPI Controller ROS 2 configuration and tuning guide</summary>

**Nav2 MPPI controller configuration** (`nav2_params.yaml`):

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 30.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56              # forward simulation steps
      model_dt: 0.05              # time per step → 2.8s prediction horizon
      batch_size: 2000            # parallel trajectory count on GPU
      vx_std: 0.2                 # linear velocity perturbation std dev
      vy_std: 0.0                 # differential drive: vy = 0
      wz_std: 0.4                 # angular velocity perturbation std dev
      vx_max: 0.5
      vx_min: -0.35
      wz_max: 1.9
      temperature: 0.3            # lambda — lower = more aggressive
      gamma: 0.015                # control smoothness weight
      iteration_count: 1          # MPPI iterations per planning cycle
      critics:
        - "GoalCritic"
        - "GoalAngleCritic"
        - "ObstaclesCritic"
        - "PathFollowCritic"
        - "PathAngleCritic"
        - "PreferForwardCritic"
      ObstaclesCritic:
        cost_power: 1
        repulsion_weight: 1.5     # obstacle repulsion weight
        critical_weight: 20.0     # near-collision cost (very high)
        collision_cost: 10000.0
        near_goal_distance: 0.5
```

**Tuning strategy**:
1. Start with `batch_size`: ensure the GPU does not OOM (an RTX 3060 handles roughly 4096 trajectories)
2. Lower `temperature` from 1.0: watch for collisions (too low) or excessive detours (too high)
3. `ObstaclesCritic.repulsion_weight` and `critical_weight` control avoidance aggressiveness
4. `vx_std / wz_std` set exploration range: too small and good paths are missed; too large and compute is wasted

**CPU vs GPU backend**: Nav2 MPPI has a pure-CPU mode (workable up to batch_size ~300) and a CUDA mode. Warehouse AGVs on Jetson Orin run CUDA mode with batch_size=2048 at a steady 50 Hz.

</details>

## Common Misconceptions

1. **"The local planner can replace the global planner"** — the local planner sees only a small window (typically 3–5 m) and will get trapped at local minima inside U-shaped corridors or mazes. The global planner provides directional guidance that pulls the local planner out of dead ends. **Avoid**: always maintain the layered architecture; if the local planner spins in place beyond a timeout threshold, trigger a global replan.

2. **"DWA works for every chassis"** — DWA assumes constant-curvature arcs (constant $v, \omega$) and poorly models Ackermann constraints (minimum turning radius). Forcing it onto a car-like platform produces infeasible commands. **Avoid**: use TEB or MPPI for Ackermann bases — they natively support non-holonomic constraints.

3. **"Inflate obstacles as much as possible for maximum safety"** — over-inflation erases traversable space, causing the robot to freeze in front of doorways. **Avoid**: set `inflation_radius` to the robot's physical radius + a modest safety margin (typically 0.1–0.3 m), and use `cost_scaling_factor` to create a gradient — approaching obstacles without contact is acceptable.

4. **"A 2D costmap is always sufficient"** — a pure 2D costmap cannot represent an overhanging shelf that is safe to drive under, or a low-hanging sign that must be dodged. **Avoid**: add a Voxel Layer for 3D → 2D projection, or use a full 3D SDF (ESDF / Voxblox) for collision checking.

5. **"Reactive avoidance (VO/RVO) does not need prediction"** — VO assumes obstacles move at constant velocity in a straight line; it fails when a pedestrian suddenly turns. **Avoid**: feed a pedestrian trajectory prediction module (Kalman Filter, Social Force Model, or LSTM) upstream of the VO stage and convert predicted paths into spatiotemporal collision cones.

## Situational Questions

<details>
<summary>Q1 (medium): Your robot uses DWA in a crowded exhibition hall and keeps emergency-stopping → rerouting → nearly stalling. How do you analyze this? What tools do you use? What pitfalls must you avoid?</summary>

**Complete reasoning chain**:

1. **Diagnose root cause**: DWA's obstacle layer reflects only the *current* positions — it does not predict where pedestrians will be next. When a person moves, DWA instantly discovers a previously safe velocity is now dangerous, triggers an emergency stop, re-samples, hits another sudden change, and oscillates.
2. **Add dynamic prediction**:
   - Integrate a pedestrian tracking module (e.g., `spencer_people_tracking`, or custom YOLO + DeepSORT)
   - Predict each pedestrian's trajectory 2–3 seconds ahead using a Kalman Filter or Constant Velocity Model
   - Write predicted positions into a **spatiotemporal costmap** with decaying cost (farther in the future = lower cost, reflecting increasing uncertainty)
3. **Upgrade the planner**:
   - Option A: keep DWA but feed the spatiotemporal costmap as an obstacle layer — lowest-cost change
   - Option B: switch to TEB (natively accepts dynamic obstacle inputs) or MPPI (add a predicted-trajectory collision term to the cost function)
4. **Pitfalls to avoid**:
   - Do not stretch the prediction window beyond ~3 s — accumulated prediction error floods the costmap with high-cost cells and paralyzes the robot
   - Do not forget that pedestrians may stop suddenly — the prediction model needs a "velocity decay" fallback

**What the interviewer wants to hear**: clear distinction between reactive vs predictive avoidance; a concrete costmap-integration plan for predictions; awareness of the side effects of overlong prediction horizons.

</details>

<details>
<summary>Q2 (medium-hard): MoveIt-planned manipulator motions are too slow for dynamic pick-and-place on a conveyor belt — planning-to-execution latency exceeds 500 ms and the object has already moved. How do you fix this?</summary>

**Complete reasoning chain**:

1. **Understand the bottleneck**: MoveIt's RRT/PRM planner searches the full C-space, requiring 100–500 ms per plan, and does not account for obstacle motion.
2. **Layered solution**:
   - **Low-frequency layer (~2 Hz)**: MoveIt pre-plans a reference trajectory to a *predicted* grasp point (object velocity $\times$ estimated arrival time as lead)
   - **High-frequency layer (100–1000 Hz)**: correct the reference in joint space using QP (Quadratic Programming) or MPC
     - Read the object's real-time pose from visual tracking
     - Use an SDF (Signed Distance Field) for environment collision constraints
     - Solve $\min \|q - q_{\text{ref}}\|^2$ s.t. collision constraints + joint limits + velocity limits
3. **SDF acceleration**: build an ESDF offline with `voxblox` or GPU voxelization; query online in $O(1)$, completing collision checks within 10 ms
4. **Toolchain**:
   - `moveit_servo`: reads Twist commands for real-time Cartesian-space tracking
   - `curobo` (NVIDIA): GPU-accelerated motion planning, planning time < 10 ms

**What the interviewer wants to hear**: layered planning mindset (offline coarse plan + online fine correction); the performance advantage of SDF-based collision checking; familiarity with real-time avoidance toolchains like `moveit_servo` and `curobo`.

</details>

<details>
<summary>Q3 (hard): 100 AGVs in a warehouse deadlock frequently at intersections. The current central dispatcher has high latency and is a single point of failure. How do you design a decentralized avoidance scheme?</summary>

**Complete reasoning chain**:

1. **Choose the algorithm**: **ORCA** (Optimal Reciprocal Collision Avoidance)
   - Each AGV independently computes its safe velocity set (intersection of ORCA half-planes), with no central coordinator
   - Per-agent complexity is $O(n)$ ($n$ = nearby neighbors); 100 AGVs each only consider neighbors within a 5 m radius
   - Mathematical guarantee: as long as all agents obey ORCA, collisions are avoided (reciprocal assumption)
2. **Priority arbitration**:
   - Static priority: loaded AGV > empty AGV > charging AGV
   - Dynamic priority: closer to destination → higher priority (prevents near-goal starvation)
   - Intersections: higher-priority AGVs' ORCA half-planes impose stronger constraints on lower-priority agents
3. **Deadlock detection and recovery**:
   - Monitor: if an AGV's speed drops below a threshold for $T$ seconds, flag it as "suspected deadlock"
   - Recover: randomly select one AGV to reverse 0.5 m and replan, breaking symmetry
   - Fallback: if regional deadlock (>3 AGVs mutually locked), escalate to a regional central dispatcher (hybrid architecture)
4. **Pitfalls to avoid**:
   - ORCA assumes all agents follow the protocol — mixing in human-driven forklifts requires an additional safety layer
   - ORCA ignores non-holonomic constraints — Ackermann AGVs need the NH-ORCA variant

**What the interviewer wants to hear**: the decentralized vs centralized trade-off; ORCA's reciprocal guarantee; the practical hybrid architecture (decentralized by default + regional central dispatch fallback).

</details>

<details>
<summary>Q4 (hard): Your autonomous vehicle uses MPPI for local planning. It performs well on highways but scrapes walls in narrow city alleys. Without switching algorithms, how do you fix it?</summary>

**Complete reasoning chain**:

1. **Diagnose root cause**: highway settings use large `vx_std` and `wz_std` (broad exploration), but in narrow alleys most random trajectories collide and are discarded, leaving too few valid samples; the weighted-average $u^*$ is poor quality
2. **Dynamic parameter switching**:
   - Detect entry into narrow passages (traversable width in the global costmap drops below a threshold) and auto-switch to a different parameter set:
     - Lower `vx_std / wz_std` (narrow the sampling range, concentrating on feasible regions)
     - Increase `batch_size` (sample more densely within the narrower range to maintain coverage)
     - Lower `temperature` (trust the few good trajectories more, avoid pollution by bad ones)
3. **Cost function adjustments**:
   - Add a `near_wall_penalty` in `CostmapCritic`: cost grows exponentially when distance to wall < 0.5 m
   - Increase `PathFollowCritic` weight: strictly follow the global path centerline in narrow passages
4. **Alternative optimizations**:
   - Importance sampling (non-uniform): densely sample near the previous tick's optimal trajectory
   - Warm-start: use the previous tick's optimal control sequence as the sampling mean, reducing waste

**What the interviewer wants to hear**: understanding of MPPI's sampling efficiency problem (low fraction of valid trajectories in tight spaces); ability to dynamically adapt hyperparameters instead of using one fixed set everywhere; knowledge of importance sampling and warm-start acceleration techniques.

</details>

## Interview Angles

1. **Layered planning architecture is the core design principle** — proves you understand the "compute budget vs reaction speed" trade-off in real systems. **Bring out with**: "I split the planning stack into Global (1 Hz A\*, guarantees global reachability) and Local (30+ Hz DWA/MPPI, guarantees real-time avoidance). Low frequency for the big picture, high frequency for fine evasion — this is not a textbook taxonomy, it is the only viable solution under real compute budgets."

2. **Integrating dynamic prediction into the costmap is the differentiating skill** — separates "can run a Nav2 demo" from "can handle dynamic environments." **Bring out with**: "Pure reactive avoidance is fine for static maps but collapses in crowds. My approach writes pedestrian tracking + trajectory prediction into a spatiotemporal costmap layer with a 2–3 second horizon, decaying cost with prediction uncertainty."

3. **Algorithm selection depends on chassis kinematics and compute** — shows you make engineering decisions based on constraints, not memorized textbook answers. **Bring out with**: "Differential drive defaults to DWA — simple, efficient. Ackermann uses TEB for the minimum turning radius. High-dimensional nonlinear systems get MPPI — but only if a GPU is available. Choosing the wrong algorithm is more fatal than tuning the wrong parameter."

4. **The reactive safety layer is the last line of defense** — demonstrates understanding of defense-in-depth system safety. **Bring out with**: "Even with a fast local planner, I still place a VO/CBF safety layer at the bottom with <5 ms latency, purely eliminating velocity commands that would collide. When the planning layer has a bug, this layer keeps people safe."

5. **Multi-robot scenarios require a shift from centralized to decentralized thinking** — shows system-level vision. **Bring out with**: "100 AGVs cannot rely on one central server replanning for everyone every 50 ms. ORCA lets each unit compute its own safe velocity using only neighbor information, $O(n)$ complexity. But you still need priority arbitration and deadlock detection on top — pure decentralization still deadlocks at intersections."

## Further Reading

- **Fox et al., *The Dynamic Window Approach to Collision Avoidance* (1997)** — the original DWA paper and foundational work on velocity-space sampling; essential for understanding DWA's design motivation and math
- **Rösmann et al., *Timed-Elastic-Band Local Planner* (2013–2017 series)** — the full evolution of TEB from basic concept to multi-topology path selection; key reference for spatiotemporal elastic band optimization
- **Williams et al., *Information Theoretic MPC (MPPI)* (2017)** — the information-theoretic foundation of MPPI; explains why Boltzmann weighting is more robust than simple arg min
- **van den Berg et al., *Reciprocal n-Body Collision Avoidance (ORCA)* (2011)** — the mathematical guarantee behind decentralized multi-robot avoidance; clarifies the geometry of reciprocal half-planes
- **ROS 2 Nav2 documentation — Controller Plugins** — complete configuration reference and tuning guide for DWA, TEB, and MPPI controllers
- **NVIDIA Isaac Lab / curobo** — GPU-accelerated motion planning and obstacle avoidance; the go-to toolchain for real-time planning in high-dimensional manipulator scenarios
- ***Embodied AI Algorithm Engineer Interview Questions*, Ch8.3 Dynamic Obstacle Detection, Ch8.4 Dynamic Replanning** — high-frequency interview topics: layered architecture, prediction integration, algorithm selection talking points
