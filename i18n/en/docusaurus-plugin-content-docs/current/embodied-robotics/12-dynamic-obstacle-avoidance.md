---
title: "Dynamic Obstacle Avoidance and Real-time Replanning"
prerequisites: ["10-basic-path-planning", "11-trajectory-optimization"]
estimated_time: 60
difficulty: 4
tags: ["obstacle-avoidance", "dwa", "teb", "mppi", "cbf", "orca", "social-nav", "reactive"]
sidebar_position: 12
---

# Dynamic Obstacle Avoidance and Real-time Replanning

## You Will Learn

- Describe the "layered planning architecture" precisely in two sentences: how a Global planner (low-frequency, globally optimal) + Local planner (high-frequency, real-time evasion) + Reactive safety layer (VO / CBF, millisecond-level gatekeeper) cooperate — no vagueness in interviews
- Diagnose "the robot keeps emergency-stopping and rerouting in a crowd but still collides" — you know the root cause is missing dynamic prediction and will reach for Kalman / IMM / Social-LSTM to predict pedestrian trajectories, write them into a spatiotemporal costmap, and use **Chance-Constrained MPC with 3σ inflation** as a hard constraint covering uncertainty
- Decide when to use DWA (differential-drive prototype), TEB (Ackermann non-holonomic), MPPI (nonlinear high-dimensional GPU-parallel), ORCA (multi-agent decentralized), CBF-QP (RL spinal-reflex safety filter), FASTER / Flight Corridor (3D UAV), or J^T·F_rep + null-space (collaborative manipulator HRI certification)
- See "service robot frozen in a crowd (Freezing Robot Problem)" → immediately reach for **Social-Aware methods**: SFM / CADRL / SARL Self-Attention / Group-Aware; see "end-to-end RL avoidance" → immediately reach for **Privileged Teacher-Student distillation + Domain Randomization + CBF safety net**
- After finishing, you can do situational reasoning in real time during interviews or AI collaboration: **"See X situation → reach for tool Y → because principle Z → avoid pitfall W"**

## Core Concepts

### Layered Planning Architecture (Three Lines of Defense)

**Precise Definition**: dynamic obstacle avoidance systems are typically split into three layers — a **Global planner** (low-frequency ~1 Hz, A\* / Dijkstra guarantees reachability) + a **Local planner** (high-frequency 20–50 Hz, real-time evasion with kinematic constraints) + a **Reactive safety layer** (millisecond VO / CBF, safety net when the planning layer has a bug). Each layer has a distinct role and this is the only industrial design that fits real compute budgets.

**Location in the Sense → Plan → Control loop**:
- **Input**: real-time obstacle positions + velocity estimates from perception (LiDAR / depth camera / pedestrian tracker), global map (static + costmap layers), dynamic obstacle trajectory predictions (spatiotemporal tubes from Kalman / IMM / Social-LSTM), and current robot state $(x, y, \theta, v, \omega)$
- **Output**: velocity commands $(v, \omega)$ or short-horizon trajectory waypoints ready for the low-level controller
- **Downstream consumers**: base motion controllers (PID / pure pursuit / MPC tracking), safety monitors (emergency stop / SSM ISO 15066), multi-robot coordination (ORCA / priority arbitration)
- **Loop node**: straddles **planning** and **control** — the local planner reads the latest perception every tick, replans, and forms the high-frequency loop "sense → local plan → velocity command → execute → sense"; the reactive safety layer is the tail-end hard "project-onto-safe-space" net

**One-line version**: "Dynamic obstacle avoidance is the robot's cerebellar reflex — the global path is GPS navigation, local planning is real-time steering, and the reactive safety layer is ABS emergency braking."

### Nine-Family Map (A-level core)

The A-level avoidance knowledge map consists of 9 sub-families covering traditional / multi-agent / predictive / safety-critical / social-aware / 3D aerial / pure-vision / end-to-end RL / manipulator HRI:

| Family | Core Weapon | Scenario Criterion |
|--------|-------------|-------------------|
| F1. Traditional local planning | DWA / APF / VFH | Indoor AGV flat 2D + low-speed static |
| F2. Multi-agent decentralized | VO / RVO / ORCA | 100+ warehouse AGVs / drone swarms |
| F3. Dynamic prediction + MPC | IMM / Social-LSTM / Chance-Constrained MPC | L4 autonomy / high-speed through crowds |
| F4. Safety-critical control | CBF / HOCBF / CBF-QP | RL spinal reflex / AEB |
| F5. Social-aware | SFM / CADRL / SARL / Group-Aware | Hospital/mall service robots |
| F6. 3D aerial avoidance | FASTER / Flight Corridor / Event Camera | High-speed UAV / disaster SAR |
| F7. Pure-vision avoidance | MiDaS / Optical Flow / TTC / SNN | Tesla FSD / Skydio pure vision |
| F8. End-to-end RL | Privileged Teacher-Student / Diffusion Policy | Quadruped outdoor exploration |
| F9. Manipulator HRI | SSM / J^T·F_rep / null-space | Cobot ISO 10218 / 15066 PLd certification |

### Minimum Sufficient Math (keep formulas, annotate physical meaning)

**① DWA Cost Function** (velocity-space multi-objective weighting, cornerstone of F1):

$$
G(v, \omega) = \sigma \big[\alpha \cdot \text{heading}(v,\omega) + \beta \cdot \text{dist}(v,\omega) + \gamma \cdot \text{velocity}(v,\omega)\big]
$$

**Physical meaning**: $\text{heading}$ measures alignment to goal, $\text{dist}$ measures clearance from nearest obstacle, $\text{velocity}$ encourages forward speed. $\alpha, \beta, \gamma$ tunable. The key is the **dynamic window** $V_d = \{(v,\omega) \mid v \in [v_c - a_{\max}\Delta t,\; v_c + a_{\max}\Delta t]\}$ naturally **samples only within physically reachable velocities** → automatically handles acceleration limits, never produces infeasible commands.

**② APF Attractive-Repulsive Potentials** (another F1 classic, with fatal local-minima flaw):

$$
U_{\text{att}} = \tfrac{1}{2} k \|q - q_{\text{goal}}\|^2, \qquad
U_{\text{rep}} = \tfrac{1}{2} \eta \left(\tfrac{1}{\rho} - \tfrac{1}{\rho_0}\right)^2 \;\; (\rho < \rho_0)
$$

**Physical meaning**: attractive potential pulls toward goal (parabolic, grows with distance); repulsive potential only activates when distance $\rho < \rho_0$, exploding as you approach. **Disaster case**: any point where $-\nabla U_{\text{att}} + \sum -\nabla U_{\text{rep},i} = 0$ (or where the potential has a local minimum) deadlocks the robot — two symmetric obstacles is just one instance; **U-shaped corridors, narrow passages, and goals hidden behind concave walls** all trigger it. Classic interview trap.

**③ Velocity Obstacle / ORCA Half-plane** (F2 gold standard):

$$
VO_{A|B} = \{\mathbf{v}_A \mid \exists\, t > 0 : (\mathbf{v}_A - \mathbf{v}_B) \cdot t \in B \oplus (-A)\}
$$

**Physical meaning**: inflate obstacle $B$ via Minkowski sum with the robot shape; the cone of relative velocities from the origin is the set of velocities that collide. ORCA linearizes the collision cone into a **half-plane constraint** $(\mathbf{v}_A - (\mathbf{v}_A^{\text{opt}} + \mathbf{u}/2)) \cdot \mathbf{n} \geq 0$, where $\mathbf{u}$ is the minimum vector pushing the relative velocity out of the VO boundary and $\mathbf{n}$ is the **unit vector of $\mathbf{u}$ (pointing outward from VO)** — the half-plane therefore points to the "collision-free" side. All constraints linear → **LP solvable in $O(n)$**, milliseconds for thousands of AGVs.

**④ Chance-Constrained MPC 3σ Spatiotemporal Tube** (F3 modern answer):

$$
\|\mathbf{p}_{\text{ego}}(k) - \boldsymbol{\mu}_{\text{obs}}(k)\|^2 \geq (d_{\text{safe}} + 3\sigma_{\text{obs}}(k))^2, \quad \forall k \in [1, H]
$$

**Physical meaning**: Social-LSTM predicts each future step's pedestrian as a 2D Gaussian $\mathcal{N}(\boldsymbol{\mu}_k, \Sigma_k)$; MPC **inflates the avoidance constraint outward by 3σ to cover 99.7% probability**. The ego vehicle automatically opens a wider lateral margin around erratic pedestrians. **"Prediction uncertainty seamlessly embedded as a control hard constraint"** is the key sentence for L4 industrial interviews.

**⑤ CBF Forward Invariant Set** (F4 unifying safety-control skeleton post-2020):

$$
\dot{h}(x, u) + \alpha(h(x)) \geq 0, \qquad h(x) \geq 0 \iff x \in \mathcal{C}
$$

**Physical meaning**: $h(x) \geq 0$ defines the safe set $\mathcal{C}$ (distance to obstacle). $-\dot{h}$ is the **actual** approach rate toward the danger boundary; the inequality $\dot h + \alpha(h) \geq 0$ rewrites as $-\dot h \leq \alpha(h)$, capping the actual approach rate at $\alpha(h(x))$ (the **allowed max approach rate**, decreasing linearly with $h$: the closer to the boundary, the smaller this cap). **Guarantee**: as $h \to 0$, the allowed max approach rate $\alpha(h) \to 0$, so motion toward danger **decays geometrically to zero** → never cross. **CBF is a safety filter, not an active force**, so no APF-style local minima.

**⑥ CBF-QP Minimum Projection** (F4 implementation form — standard architecture for adding hard safety to RL):

$$
\min_{u} \; \tfrac{1}{2} \|u - u_{\text{nom}}\|^2 \quad \text{s.t.}\quad L_f h(x) + L_g h(x) \cdot u + \alpha h(x) \geq 0, \quad u_{\min} \leq u \leq u_{\max}
$$

**Physical meaning**: RL brain outputs $u_{\text{nom}}$; CBF-QP is the **spinal reflex layer**, intervening only when $u_{\text{nom}}$ would violate safety, projecting onto the safe space with minimum cost. Convex optimization in microseconds → RL generalization + mathematical safety floor.

**⑦ MPPI Trajectory Weighting** (F1 / F3 hybrid, GPU-parallel information-theoretic MPC):

$$
w^{(k)} = \exp\!\left(-\tfrac{1}{\lambda} S(\tau^{(k)})\right), \qquad u^* = \frac{\sum_k w^{(k)} u^{(k)}}{\sum_k w^{(k)}}
$$

**Physical meaning**: compute total cost $S$ for $K$ randomly perturbed trajectories, aggregate via Boltzmann softmax. $\lambda$ is temperature — smaller is greedier (trust only the best few), larger is more conservative. GPU runs $K = 2048\text{–}8192$ in parallel with zero data dependencies.

**⑧ Manipulator Cartesian Potential → Jacobian-transpose mapping** (F9 Cobot certification core):

$$
\boldsymbol{\tau}_{\text{avoid}} = \sum_i \mathbf{J}_i(q)^\top \cdot \mathbf{F}_{\text{rep},i}, \qquad
\boldsymbol{\tau}_{\text{null}} = (\mathbf{I} - \mathbf{J}^+ \mathbf{J}) \cdot \boldsymbol{\tau}_{\text{avoid}}
$$

**Physical meaning**: each link's Capsule approaching an obstacle generates a 3D virtual repulsive force $\mathbf{F}_{\text{rep}}$; the Jacobian transpose maps Cartesian forces to joint torques. **7-DoF redundancy core**: projected onto the Jacobian null space → elbow yields compliantly, the cup at the end effector stays perfectly still → main task decoupled from avoidance. This is the mathematical skeleton behind ISO 10218 / 15066 PLd certification.

**⑨ Optical Flow TTC — Insect-style Avoidance** (F7 pure-vision elegance):

$$
\tau = \frac{Z}{V_z} = \frac{d}{\dot{d}}
$$

**Physical meaning**: no need for absolute distance $Z$ or true velocity $V_z$ — **pixel area divided by its expansion rate gives the time-to-collision**. Pixel area grows geometrically on approach; insects and drones use this reflex for avoidance, clock-cycle down to event-camera sub-millisecond level.

### Costmap Architecture (shared input layer for F1–F3)

The costmap is the critical data structure bridging perception and planning:

| Layer | Purpose | Update Rate |
|-------|---------|-------------|
| **Static Layer** | Static occupancy loaded from the SLAM map | Once |
| **Obstacle Layer** | Real-time obstacles from LiDAR / depth cameras | 10–30 Hz |
| **Inflation Layer** | Exponentially decaying safety gradient around obstacles | Follows Obstacle Layer |
| **Voxel Layer** | 3D voxel grid (handles overhangs, under-table clearance) | 10–30 Hz |
| **Spatiotemporal Layer** (A-level) | Pedestrian trajectory predictions ($k$ future steps, 3σ inflation) | Follows prediction module |

**Physical meaning**: the Inflation Layer's cost decays exponentially from obstacle edges — when the robot center enters a high-cost zone, the physical shell is nearly touching. `inflation_radius` sets the expansion distance, `cost_scaling_factor` controls steepness. The **Spatiotemporal Layer** is the A-level upgrade: pedestrian predictions for future step $k$ + 3σ uncertainty are written into different time slices; MPC reads multiple slices for spatiotemporal planning.


<details>
<summary>Deep dive: full DWA algorithm flow and velocity window derivation (F1 skeleton)</summary>

**Complete steps per DWA tick**:

1. **Build the velocity search space**:
   - Static window $V_s = \{(v, \omega) \mid v \in [v_{\min}, v_{\max}],\; \omega \in [\omega_{\min}, \omega_{\max}]\}$
   - Dynamic window $V_d = \{(v, \omega) \mid v \in [v_c - \dot{v}_b \Delta t,\; v_c + \dot{v}_a \Delta t],\; \omega \in [\omega_c - \dot{\omega}_b \Delta t,\; \omega_c + \dot{\omega}_a \Delta t]\}$
   - Admissible window $V_a = \{(v, \omega) \mid v \leq \sqrt{2 \cdot \text{dist}(v,\omega) \cdot \dot{v}_b}\}$ (guarantees braking before collision)
   - Final search space $V = V_s \cap V_d \cap V_a$

2. **Forward simulation**: for each discrete $(v_i, \omega_j)$ in $V$, simulate the arc trajectory over $T_{\text{sim}}$ (typically 1–3 s).

3. **Cost evaluation**:
   - $\text{heading}$: angle between trajectory endpoint heading and goal direction (smaller is better)
   - $\text{dist}$: shortest distance along the trajectory to the nearest obstacle (larger is better)
   - $\text{velocity}$: forward speed magnitude (encourages fast motion)

4. **Select optimum**: pick $(v^*, \omega^*)$ maximizing $G = \alpha \cdot \text{heading} + \beta \cdot \text{dist} + \gamma \cdot \text{velocity}$.

**Velocity-space discretization**: typical DWA uses a $30 \times 30$ to $50 \times 50$ grid. Nav2's DWA does iterative forward simulation with 20–30 steps per trajectory.

**DWA's fatal flaw — U-shaped dead end**:
- DWA is very myopic (looks ahead only 1–2 s)
- U-shaped corridor / dead-end: **every direction moves the robot farther from the goal** → stuck in place
- Must rely on the Global planner for detour guidance → the origin of the layered architecture

**Tuning in practice**:
- Narrow corridors: increase $\beta$ (safety clearance), decrease $\gamma$ (allow slowdowns)
- Open spaces: increase $\gamma$ (pursue speed), decrease $\beta$
- Crowded environments: add a prediction layer writing future pedestrian positions into the costmap (upgrade to F3 family)

**Nav2 DWB C++ core scoring**:

```cpp
double score = alpha * headingDiff(traj.back(), goal)
             + beta * (1.0 / costmap->minDist(traj))
             + gamma * (max_v - traj.velocity);
```

</details>

<details>
<summary>Deep dive: APF local-minima physical reasoning and VFH improvement (F1 pitfall map)</summary>

**APF local-minima disaster**:

Scenario: robot at corridor center, pillars on left and right, goal directly ahead.
- **Attractive force $\mathbf{F}_{\text{att}} = -k(\mathbf{q} - \mathbf{q}_{\text{goal}})$**: points forward toward goal
- **Left pillar repulsion $\mathbf{F}_{\text{rep},L}$**: points right
- **Right pillar repulsion $\mathbf{F}_{\text{rep},R}$**: points left
- By symmetry $\mathbf{F}_{\text{rep},L} + \mathbf{F}_{\text{rep},R} = 0$, **but the attractive force aligns exactly opposite to the total repulsive force and they cancel** → net force zero → robot deadlocks

**Classic variants**:
- Goal behind a U-shaped wall: attraction points into the wall, repulsion points outward — they cancel
- Robot centered between two identical obstacles: symmetric cancellation

**VFH (Vector Field Histogram) improvement**:
- Sensor point cloud → **polar histogram**: one density value per angular sector
- Threshold → find "Free Valleys"
- Head directly toward **the free valley closest to the goal direction**
- **Key**: VFH never computes attractive / repulsive forces — it finds geometric free space → completely sidesteps the local-minima problem

**Interview answer — when to use APF / DWA / VFH**:
- APF: teaching only; rarely deployed industrially due to local minima
- DWA: indoor AGV flat 2D + low-speed (< 2 m/s) + static shelves → fast + cost-effective → mainstream
- VFH: staple of early mobile robots (Pioneer, MobileRobots)
- **High dynamics (60 km/h cars / erratic pedestrians)**: DWA / APF both fail (myopic / non-holonomic / high-speed sideslip not modeled) → **must upgrade to F3 family "prediction-based MPC"**

</details>


<details>
<summary>Deep dive: ORCA half-plane LP and the Freezing Robot Problem (F2 gold standard)</summary>

**VO → RVO → ORCA evolution**:

1. **VO geometric essence**: A and B assumed to maintain current velocity; A's "collision cone" in relative velocity space is a triangular cone. As long as the relative velocity $\mathbf{v}_A - \mathbf{v}_B$ stays outside the cone, no future collision.
2. **RVO oscillation elimination**: pure VO causes corridor encounters to "dodge left together, then right together" → **Reciprocal Dance oscillation**. RVO requires **each agent to take 50% avoidance responsibility**: A only adjusts half, expecting B to do the other half.
3. **ORCA linearization**: geometric cone → **strict half-plane constraint**
   $$
   (\mathbf{v}_A - (\mathbf{v}_A^{\text{opt}} + \mathbf{u}/2)) \cdot \mathbf{n} \geq 0
   $$
   where $\mathbf{u}$ is the minimum perturbation vector pushing $\mathbf{v}_A - \mathbf{v}_B$ out of the collision cone, and $\mathbf{n}$ is the **unit vector of $\mathbf{u}$ (pointing outward from VO)** so that the half-plane consistently points to the "collision-free" side — reversing $\mathbf{n}$ flips the inequality entirely. **All linear → LP in $O(n)$**, milliseconds for 1000+ AGVs / drones.

**Freezing Robot Problem disaster (interview must-answer)**:
- Sandwiched between two opposing pedestrians: left gives a "go right" half-plane, right gives a "go left" half-plane.
- In dense crowds, **all half-planes intersect to the empty set** → LP infeasible → instant emergency stop → frozen.
- **Fixes**:
  1. **Hard → soft constraints**: introduce slack $\epsilon_i \geq 0$ allowing tiny half-plane violation with a huge penalty $\min \sum \lambda_i \epsilon_i$
  2. **Combine with topological graph search**: LP infeasible on this path → global replan around the crowd
  3. **Social-Aware upgrade** (F5 family): stop modeling pedestrians as moving cylinders; model "people yield to people" negotiation

**RVO2 Library C++**:

```cpp
Line orca;
orca.point = velocity_ + 0.5f * u;              // each takes half responsibility
orca.direction = Vector2(-n.y(), n.x());        // perpendicular normal
orcaLines_.push_back(orca);
linearProgram2(orcaLines_, prefVelocity_, newVelocity_);  // solve LP
```

**NH-ORCA (Non-Holonomic ORCA)**: Ackermann AGVs / car-like robots cannot sideslip → convert non-holonomic constraints into additional velocity-space restrictions, then apply ORCA. Freiburg's Siegwart group 2011 classic.

**ORCA industrial scenarios**: Amazon Kiva warehouse (now Amazon Robotics), Cainiao logistics, DJI Swarm formations.

</details>

<details>
<summary>Deep dive: Chance-Constrained MPC + Social-LSTM spatiotemporal tube (F3 L4 industrial standard)</summary>

**Why MPC is the strongest weapon for moving-obstacle avoidance**:
- APF / DWA only see the instantaneous snapshot → myopic
- **MPC lookahead**: at time $t$, optimizes the next $H$ steps; writes dynamic-obstacle **future $k$-step predicted positions** into the next $k$-step constraints
- Automatically learns **slow-and-yield, accelerate-and-pass, pre-detour** dynamic game-play

**Three generations of dynamic-obstacle motion models**:

| Generation | Method | Characteristic |
|------------|--------|----------------|
| 1st | CV (Constant Velocity) | Straight-line, simplest, fails on turning pedestrians |
| 2nd | CA + IMM (Interacting Multiple Models) | Runs multiple models in parallel (straight / left / right), Bayes-weighted switching |
| 3rd | **Social-LSTM / Trajectron++** | RNN / Transformer combines pedestrian history + Social Force → multimodal probability distribution |

**Why Social-LSTM is powerful**:
- Pedestrian trajectories are highly coupled (people avoid people)
- Traditional Kalman treats each pedestrian in isolation → large prediction error
- Social-LSTM's Social Pooling layer lets neighboring pedestrians' hidden states interact
- Predicts **multimodal probabilistic trajectories** for 3–5 s into the future (probability of going left / right)

**Chance-Constrained MPC — turning uncertainty into a hard constraint**:
- NN predicts → 2D Gaussian $\mathcal{N}(\boldsymbol{\mu}_k, \Sigma_k)$ at each future step
- **3σ inflation**: MPC avoidance constraint expanded outward by $3\sigma$, covering 99.7% probability
  $$
  \|\mathbf{p}_{\text{ego}}(k) - \boldsymbol{\mu}_{\text{obs}}(k)\|^2 \geq (d_{\text{safe}} + 3\sigma_{\text{obs}}(k))^2
  $$
- Ego automatically opens wider lateral clearance around erratic pedestrians
- **Prediction uncertainty seamlessly embedded as a control hard constraint** ← memorize for industrial interviews

**CasADi Chance-Constrained MPC Python**:

```python
for k in range(horizon):
    opti.subject_to(X[:, k+1] == rk4_step(model, X[:, k], U[:, k], dt))
    dist_sq = (X[0, k+1] - obs_mu_x[k])**2 + (X[1, k+1] - obs_mu_y[k])**2
    safe_margin = robot_radius + obs_radius + 3.0 * obs_sigma[k]
    opti.subject_to(dist_sq >= safe_margin**2)  # 3σ hard constraint
```

**Interview "industrial avoidance standard" answer**:
- L4 autonomy / high-end embodied avoidance = **spatiotemporal optimization problem**, not geometry
- Traditional: treat a moving car as "a moving wall" → extremely conservative emergency brake
- **Modern "Prediction + MPC"**:
  - Front-end Social-LSTM predicts pedestrian **spatiotemporal tube**
  - Back-end MPC guarantees ego trajectory does not intersect the tube
- **Platforms**: Waymo Driver, Tesla FSD, Apollo, Tier IV Autoware

</details>

<details>
<summary>Deep dive: MPPI GPU architecture and temperature tuning</summary>

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
- $\lambda \to 0$: approaches arg min, selects only the lowest-cost trajectory — aggressive but noisy
- $\lambda \to \infty$: all trajectories weighted equally — overly conservative
- Practice: $\lambda \in [0.01, 10]$, start at $\lambda = 1$ and binary-search to the sweet spot between "no collisions" and "no excessive detours"

**Cost function design (common in autonomous driving)**:

$$
S(\tau) = \sum_{t=0}^{T} \left[ q_{\text{goal}} \|x_t - x_{\text{goal}}\|^2 + q_{\text{obs}} \cdot c_{\text{obs}}(x_t) + q_u \|u_t\|^2 + q_{\text{jerk}} \|\Delta u_t\|^2 \right]
$$

- $c_{\text{obs}}$: looked up from a Signed Distance Field (SDF); cost grows exponentially as distance to obstacles shrinks
- $q_{\text{jerk}}$: penalizes abrupt control changes, improving ride comfort

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
      wz_std: 0.4                 # angular velocity perturbation std dev
      temperature: 0.3            # lambda — lower = more aggressive
      gamma: 0.015                # control smoothness weight
      critics:
        - "GoalCritic"
        - "ObstaclesCritic"
        - "PathFollowCritic"
      ObstaclesCritic:
        repulsion_weight: 1.5
        critical_weight: 20.0
        collision_cost: 10000.0
```

**Common implementations**: NVIDIA Isaac Lab ships a built-in MPPI controller; [MPPI-Generic](https://github.com/ACDSLab/MPPI-Generic) is an open-source C++/CUDA library; Nav2 MPPI Plugin supports both CPU and CUDA backends.

</details>


<details>
<summary>Deep dive: CBF / HOCBF Forward Invariant Set theory and CBF-QP as RL spinal reflex (F4 unified safety control)</summary>

**Why CBF became the unifying safety theory post-2020**:
- Traditional: goal tracking used **CLF (Control Lyapunov)** + avoidance used APF / geometric hard constraints — two separate frameworks
- **Ames et al.'s CBF rigorously reformulates "safety" as a Forward Invariant Set problem**
- Safety constraints and control commands unified under the same energy / gradient framework → convex optimization solvable

**Forward Invariant Set definition**:
- Safe set $\mathcal{C} = \{x \in \mathbb{R}^n \mid h(x) \geq 0\}$
- If control $u$ satisfies $\dot{h}(x, u) + \alpha(h(x)) \geq 0$, then $h$ is a CBF
- **Physical meaning**: $\dot{h}(x, u) = \nabla h \cdot f(x) + \nabla h \cdot g(x) u$ is your approach speed to the danger boundary; $\alpha(h(x))$ is the allowed max approach speed (proportional to distance from boundary, usually $\alpha(h) = \gamma h$)
- **Guarantee**: "the closer to the death-line $h(x) = 0$, the approach speed toward danger decays geometrically to zero" → **never crosses**

**HOCBF (Higher-Order CBF) solves Relative Degree > 1**:
- Robots are second-order systems (input is torque / acceleration, constraint is on position $x$)
- Direct derivative of position only yields velocity — control input does not appear in $\dot{h}$ → Relative Degree > 1
- HOCBF uses multi-step Taylor expansion + exponential decay:
  $$
  \psi_1(x) = \dot{h}(x) + \alpha_1(h(x)) \geq 0
  $$
  $$
  \psi_2(x, u) = \dot{\psi}_1(x, u) + \alpha_2(\psi_1(x)) \geq 0
  $$
- Solves the **braking-distance problem**: you need to start decelerating while braking distance still exists, not after hitting the wall

**CBF vs APF** (the elegance of no local minima):
- APF actively generates forces → attraction + repulsion cancel → **local minima**
- **CBF is a safety filter, not an active force**:
  - Only monitors the nominal controller (PID or RL output $u_{\text{nom}}$)
  - Does nothing when $u_{\text{nom}}$ is safe
  - Only when $u_{\text{nom}}$ would violate safety does CBF project with minimum cost to the safe space
- **Does not alter the original energy-field topology** → naturally free of local minima

**CBF-QP math**:

$$
\min_u \; \tfrac{1}{2} \|u - u_{\text{nom}}\|^2
$$

s.t.
$$
L_f h(x) + L_g h(x) \cdot u + \alpha h(x) \geq 0 \quad \text{(safety)}
$$
$$
u_{\min} \leq u \leq u_{\max} \quad \text{(actuator limits)}
$$

where $L_f h = \nabla h \cdot f$, $L_g h = \nabla h \cdot g$ are Lie derivatives.

**Industrial answer for "adding hard safety to RL"**:
- RL is a black box + exploratory → easily dangerous actions in physical world
- Cannot rely on RL learning 100% safety on its own (exponential sample complexity)
- Standard solution: **"RL brain outputs $u_{\text{nom}}$, CBF-QP spinal-reflex layer intercepts"**
- CBF-QP convex optimization solves in microseconds → least-squares interception → RL generalization + mathematical safety floor
- This is the engineering compromise answering "end-to-end RL demos look cool but rarely ship in industry"

**CBF-QP Python implementation**:

```python
import cvxpy as cp
u = cp.Variable(u_nom.shape)
objective = cp.Minimize(0.5 * cp.sum_squares(u - u_nom))
Lf_h = grad_h @ f_x
Lg_h = grad_h @ g_x
cbf_constraint = [Lf_h + Lg_h @ u + alpha * h_val >= 0]
torque_limit = [u >= -MAX_TORQUE, u <= MAX_TORQUE]
cp.Problem(objective, cbf_constraint + torque_limit).solve(solver=cp.OSQP)
```

**Platforms**: automotive AEB (Mobileye EyeQ, Bosch), collaborative manipulators' Speed and Separation Monitoring, Boston Dynamics Atlas safety layer.

</details>

<details>
<summary>Deep dive: Social-Aware navigation — SFM / CADRL / SARL Self-Attention / Group-Aware (F5 hospital/mall service)</summary>

**Freezing Robot Problem disaster**:
- Treating pedestrians as moving cylinders → in dense crowds, all future Feasible Velocity Sets are occupied by pedestrian spatiotemporal trajectories → planner infeasible → stuck frozen.
- **But in reality, people yield to people** → the robot proactively nudges a bit and the crowd naturally parts.
- Must introduce "Social-Aware" behavior.

**Social Force Model (SFM, Helbing 1995)**:
- Models the crowd as interacting particles
- Pedestrian attracted to goal + repelled by other pedestrians / walls
  $$
  \mathbf{F}_i = \mathbf{F}_{\text{goal}} + \sum_j \mathbf{F}_{\text{rep},ij} + \mathbf{F}_{\text{wall}}
  $$
- **Drawback**: passive reactive, cannot model "active negotiation" (e.g., polite yielding, proactive acceleration)

**CADRL (Collision Avoidance with Deep RL)**:
- End-to-end RL + **Self-play** (robot plays against itself in simulation)
- Learns implicit social rules like "go behind others, proactively yield"
- Solves 2–4 agent navigation

**SARL (Socially Attentive RL) — dense-crowd core architecture**:
- Crowd size $N$ varies → traditional RL struggles with variable-dimension state
- **Self-Attention mechanism**:
  - Compute attention score $\alpha_i = \text{softmax}(e_i)$ for each pedestrian, where $e_i = \text{MLP}(s_{\text{robot}}, s_{\text{human},i})$
  - Focus only on "likely collision" key pedestrians (high attention weight)
  - Variable-length crowd state compressed to fixed-length context vector
- Input variable $N$ pedestrians, output fixed-length policy → perfect for dynamic crowds

**Group-Aware Navigation**:
- Two or three people walking side-by-side chatting → cutting through the middle is rude (and may scare them)
- Cluster pedestrians with similar trajectories and proximity into a **Group** → treat as indivisible → detour around
- Methods: DBSCAN on pedestrian trajectories + F-formation geometric detection

**Interview answer for "service robot commercialization last mile"**:
- Hospitals and malls' challenge is not "finding the path" — it is "getting stuck" or "provoking human annoyance"
- **SARL + SFM hybrid = nonverbal social negotiation capability**
- Determines whether the robot appears clumsy or integrates gracefully into human society
- **Platforms**: UBTECH Walker, Figure 01 hospital navigation, Amazon Astro, Toyota HSR

**SARL Attention PyTorch snippet**:

```python
class CrowdAttention(nn.Module):
    def forward(self, robot_state, human_states):
        joint = torch.cat([robot_state.expand(-1, N, -1), human_states], dim=2)
        features = F.relu(self.mlp(joint))          # MLP encoding
        scores = self.attention_net(features)       # attention scoring
        weights = F.softmax(scores, dim=1)          # variable → fixed
        context = torch.sum(weights * features, dim=1)
        return context  # fixed-dim environment context
```

**Further reading**: GP-SARL (combines Gaussian Process uncertainty), DS-RNN (Decentralized Structural RNN), intention-aware navigation (predict intent before acting).

</details>

<details>
<summary>Deep dive: Aerial 3D avoidance — FASTER / Flight Corridor / Event Camera (F6 high-speed UAV)</summary>

**Three challenges for 3D UAV avoidance**:
1. **SE(3) flight + severe underactuation**: no brakes; deceleration requires pitching the body (4 rotors only produce vertical thrust + 3 moments — 6-DoF state with only 4 control inputs)
2. **Limited depth-camera FOV**: high-speed turns leave blind spots with no map → crash
3. **High-speed 10 m/s flight dynamics**: sense → map → plan serial latency cannot exceed 50 ms

**FASTER Planner (MIT) — dual-trajectory parallel**:
- Core philosophy: **safety fallback**
- Each cycle solves two trajectories simultaneously:
  - **Primary trajectory**: aggressive, exploring free-space boundary
  - **Backup trajectory**: guaranteed able to stop / hover within the safe zone
- If primary fails → seamlessly switch to backup and hover → absolute safety
- **Difference from single-trajectory MPC**: no need to wait until the next tick to discover failure

**Bubble Planner / Flight Corridors**:
- 3D point-cloud avoidance is **non-convex** → direct NLP is slow
- **Corridor planner** inflates free space into **overlapping convex polyhedra or spheres** (Polyhedral Safe Corridor)
- Non-convex avoidance → staying inside the polyhedron is a set of linear inequalities $\mathbf{A}_i \mathbf{x} \leq \mathbf{b}_i$
- QP solves B-spline trajectories extremely fast
- **B-spline convex-hull property guarantees: all control points inside the polyhedron → the continuous curve never escapes** ← this is the key math

**Reactive vision-based avoidance (event cameras)**:
- Traditional cameras suffer **motion blur** at high speed + 30 Hz framerate creates fatal blind spots at 20 m/s
- **Event cameras** respond only to brightness changes with **< 1 ms asynchronous event stream + ultra-high dynamic range (140 dB)**
- Sub-millisecond computation of **Time-to-Contact (TTC)** for onrushing obstacles → pure reactive evasion

**Perception-aware planning**:
- Plan under a **camera FOV constraint**
- During lateral flight, **keep the nose (camera) pointed at the unknown region being entered** → avoid blind flight
- Math: add FOV term $\|\mathbf{v}_{\text{desired}} - \mathbf{d}_{\text{camera}}\|^2$ to MPC objective

**Interview answer for "event camera is the next UAV paradigm"**:
- Traditional (FASTER / Bubble) depend on "map → plan → execute" serial pipelines
- At extreme dynamics, map latency is enough for a crash
- **Event cameras break the framerate limit**, asynchronous pulses directly reflect dynamic environment edges
- Combined with SNN (Spiking Neural Networks) or low-latency reactive → **insect-like optical-flow-expansion reflex**
- **Leaps from "geometric navigation" → "biomimetic reflex"**

**Platforms**: Skydio X10 (pure vision AI avoidance), DJI Mavic 3 (ActiveTrack 5.0), Zipline medical delivery UAVs, Shield AI Nova.

**Flight Corridor QP C++ snippet**:

```cpp
for (int i = 0; i < num_control_points; ++i) {
    A = getPolyhedronA(i); b = getPolyhedronB(i);
    for (int j = 0; j < A.rows(); ++j) {
        // A_j · P_i ≤ b_j: B-spline convex hull → all control points inside → curve inside
        solver.addConstraint(A.row(j), control_points[i], b(j));
    }
}
solver.addObjective(smoothness_cost + tracking_cost);
solver.solve();  // QP solved in a few ms
```

**Recommended papers**: FASTER (Tordesillas 2019), EGO-Planner (Zhou 2020), Fast-Planner / Ego-Swarm (Gao 2018–2022).

</details>


<details>
<summary>Deep dive: Pure-vision avoidance — MiDaS / Optical Flow TTC / SNN biomimetic (F7)</summary>

**Monocular depth estimation explosion in 2024**:
- Previously lacked scale information → monocular could not give absolute distances
- **MiDaS / ZoeDepth / Depth Anything** use Transformer + massive mixed-dataset training → robust relative / absolute depth
- Avoidance often does not need precise 3.14 m — just "closer than background and rapidly expanding" is enough to trigger

**Optical flow expansion (insect-style avoidance)**:
- Approaching an obstacle, pixel area **expands geometrically**
- **TTC (Time-to-Collision)**: $\tau = Z / V_z = d / \dot{d}$
- **Physical meaning**: no absolute distance $Z$ or true velocity $V_z$ needed — pixel distance divided by expansion rate gives time-to-collision
- Bees, flies, dragonflies all rely on this: visual-field expansion rate exceeds threshold → reflex turn

**Learned Depth + MPC (modern Vision MPC)**:
- NN real-time depth prediction → 3D occupancy voxels / SDF → feeds MPC inequality constraints
- Preserves visual semantic richness + MPC dynamic smoothness
- Tesla FSD's Occupancy Network is exactly this family

**Event Camera + SNN biomimetic avoidance**:
- Traditional 30 Hz cameras suffer motion blur at high speed
- Event cameras record only brightness-change asynchronous events with microsecond latency
- **SNN (Spiking Neural Networks) event-driven** → sub-millisecond onrushing-object detection + avoidance
- Low power + low latency → the only option for tiny micro-UAVs

**Interview answer for "monocular + AI replaces LiDAR"**:
- LiDAR is geometrically precise but **semantically sparse** (cannot distinguish a real rock from water spray / tall grass) → phantom braking
- Monocular + AI is low-cost (Tesla FSD, Skydio pure vision)
- **VLM understands "a poster of a car is not a real car"**
- **Visual information entropy ceiling is far higher than LiDAR** (8-bit × 3 channel × 1M pixel RGB vs 16-bit × 32-line × 1800-point LiDAR)
- **Counterargument**: LiDAR still wins at night / rain / fog / glass; L4 still uses multi-sensor fusion

**Vision MPC snippet (ZoeDepth + CasADi)**:

```python
depth_map = zoedepth_model(rgb_image)              # infer depth
sdf_tensor = depth_to_sdf(depth_map)                # convert to SDF
d_safe_k = get_obstacle_clearance(sdf_tensor)       # lookup
opti.subject_to(ca.norm_2(p_k - obs_pred_k) >= d_safe_k + margin)  # MPC hard constraint
```

**Platforms**: Tesla FSD Pure Vision, Skydio X10, comma.ai openpilot, DJI Avata.

</details>

<details>
<summary>Deep dive: End-to-end RL avoidance — Privileged Teacher-Student / Domain Randomization / Diffusion Policy (F8)</summary>

**Pixel → Action direct learning**:
- Traditional "sense → map → plan → control" is serial with errors compounding at each stage
- **End-to-end RL**: RGB-D pixels → CNN / Transformer → policy → joint torques
- Network learns implicitly "see a pit → lift leg", "see a human → step aside"

**Sim-to-Real core: Domain Randomization**:
- Pure-vision RL easily overfits simulator rendering
- In Isaac Sim training, **aggressively randomize lighting, textures, camera noise, FOV, friction, mass, action latency**
- Forces the network to abandon specific visual textures → **learn truly invariant geometric avoidance depth features**
- Anymal / Spot quadruped outdoor robust avoidance all rely on this

**Visual Policy Distillation (Asymmetric Actor-Critic)**:
- **Teacher (Privileged)**: simulator god-view + precise 3D positions, velocities, frictions → quickly learns perfect avoidance
- **Student (Vision)**: real-world only has cameras → Behavior Cloning to match Teacher actions
- **Distillation loss**:
  $$
  \mathcal{L} = \mathbb{E}\left[\|\pi_{\text{student}}(a \mid o_{\text{vision}}) - \pi_{\text{teacher}}(a \mid s_{\text{priv}})\|^2\right]
  $$
- ETH Zurich ANYmal team classic paradigm, Hwangbo 2019 foundational work

**Legged Gym / Isaac Lab massive parallel training**:
- 10,000+ environments simultaneously roll out on GPU
- Single-GPU 4096 robots training in parallel, originally weeks-long training done in hours

**Diffusion Policy 2024 frontier**:
- PPO / SAC Gaussian averaging → "left detour vs right detour" averaged **hits the obstacle center** (disaster case)
- Diffusion Policy treats avoidance actions as denoising generation → **perfectly preserves multimodal solutions**
- Columbia's Cheng Chi et al. 2023 paper

**"Demo looks cool but rarely ships in industry" pitfall (interview must-answer)**:
- **Interpretability + functional safety certification**: end-to-end NN black box → cannot debug which layer failed after a collision
- **ISO 13849 / 15066 SIL / PLd certification** requires deterministic limits + traceable decisions
- **Engineering compromise**: **RL gives Proposal → underneath, a deterministic CBF-QP or MPC safety filter catches** (F4 CBF family application)
- This is the modern industrial answer to the "end-to-end vs modular" debate

**Platforms**: ANYbotics ANYmal (Privileged Teacher-Student), Boston Dynamics Atlas (partial RL modules), Figure 01, Tesla Optimus, Columbia Diffusion Policy.

</details>

<details>
<summary>Deep dive: Manipulator HRI dynamic avoidance — SSM / J^T·F_rep / null-space redundancy (F9 Cobot certification core)</summary>

**Manipulator vs mobile-base avoidance — fundamental difference**:
- Mobile base: "point mass / bounding box" on a 2D plane
- **Manipulator is a high-dimensional articulated chain**:
  - **Self-collision**: elbow cannot hit the base
  - **Whole-body links** cannot hit the environment
  - **7-DoF C-space** is computationally complex

**SSM (Speed and Separation Monitoring) — ISO/TS 15066 core**:
- 3D cameras detect approaching humans (Intel RealSense, Microsoft Kinect Azure, Pilz SafetyEYE)
- **Real-time max manipulator speed limit based on human-robot separation distance**
- Crossing red line (e.g., < 0.5 m) → **safety-rated emergency stop** or **downgrade to zero-torque teaching mode**
- This is the mandatory requirement for **Category 3 PLd** safety rating

**SSM speed formula (simplified ISO/TS 15066)**:

$$
v_{\max} = \max\left(0, \frac{S_p - (S_B + S_R + S_C)}{T}\right)
$$

where $S_p$ is separation distance, $S_B, S_R, S_C$ are robot braking distance, reaction distance, and human intrusion distance, and $T$ is the robot's total **reaction + braking time**. **Note**: this is a simplified inverse — in practice, ISO/TS 15066 compares the protective separation $S_p(t_0) = S_H + S_R + S_S + C + Z_d + Z_r$ against a threshold rather than directly solving for $v_{\max}$.

**Cartesian potential-field avoidance**:
- Wrap manipulator links in **Capsules** (cylinder + two hemispheres — tighter than spheres, simpler than convex hulls)
- Point cloud detects dynamic obstacle approaching link $i$
- Compute 3D virtual repulsive force $\mathbf{F}_{\text{repulsive}}$
- **Jacobian transpose $\mathbf{J}_i^\top \cdot \mathbf{F}_{\text{rep}}$ maps to joint avoidance torques**
- Superposition formula:
  $$
  \boldsymbol{\tau}_{\text{avoid}} = \sum_i \mathbf{J}_i(q)^\top \cdot \mathbf{F}_{\text{rep},i}
  $$

**Null-space optimization (7-DoF redundancy core)**:
- **Torque-space projection**: $(\mathbf{I} - \mathbf{J}^\top \mathbf{J}^{+\top}) \cdot \boldsymbol{\tau}_{\text{avoid}}$ projects torques onto the null space of the end-effector wrench mapping (note: the **velocity null-space projector** $(\mathbf{I} - \mathbf{J}^+\mathbf{J})$ is for $\dot q$; the **torque null-space projector** $(\mathbf{I} - \mathbf{J}^\top \mathbf{J}^{+\top})$ is for $\tau$ — they are transposes of each other)
- Worker pushes manipulator elbow → **elbow yields compliantly, but the cup held at the end effector does not move a millimeter**
- Main task (end-effector pose) and avoidance (joint-level) fully decoupled
- Math: $\mathbf{J}^{+\top} \mathbf{J}^\top \tau_{\text{null}} = 0$ means null-space torques produce no end-effector wrench → cup stays still

**Interview answer for "HRI certification core"**:
- Cobot (UR / KUKA iiwa / Franka Panda) customers pay for **safety**
- $\mathbf{J}^\top \cdot \mathbf{F}_{\text{rep}}$ potential-field avoidance + SSM speed monitoring = **mathematical skeleton of ISO 10218 / ISO/TS 15066 PLd certification**
- Converts unpredictable human behavior into rigorous physical mechanical constraints
- Guarantees **no pinching or fatal collisions in HRI scenarios**

**Franka C++ snippet**:

```cpp
for (int i = 0; i < critical_links.size(); ++i) {
    double dist = compute_min_distance(critical_links[i], obstacles);
    Eigen::Vector3d normal = compute_repulsive_direction(...);
    if (dist < safe_margin) {
        double mag = eta * (1.0/dist - 1.0/safe_margin) * (1.0/(dist*dist));
        Eigen::Vector3d F_rep = mag * normal;
        Eigen::MatrixXd J_i = compute_jacobian_for_link(state.q, i);
        tau_avoid += J_i.transpose().topRows(3) * F_rep;
    }
}
// null-space projection
Eigen::MatrixXd N = I - J_pinv * J;  // null-space projector
tau_null = N * tau_avoid;
robot.setJointTorques(tau_task + tau_null);  // main task + null-space avoidance superposed
```

**Platforms**: Franka Panda HRI mode, KUKA iiwa Cobot, Universal Robots UR series, TM Robot AI Cobot.

**Advanced**: Cartesian Impedance Control + CBF (combined with F4), MoveIt 2 Servo + real-time SDF, NVIDIA cuRobo GPU-accelerated planning.

</details>

## Intuition

**Analogy: three layers of driving reaction**
- **GPS navigation = Global planner**: tells you "take the highway, then exit onto Route 3" — very low frequency (recomputes every few seconds)
- **Steering = Local planner (DWA / TEB / MPPI)**: real-time lane-level adjustments, 20–50 Hz
- **ABS braking = Reactive safety layer (VO / ORCA / CBF)**: bypasses "planning" entirely, eliminates dangerous velocity commands in velocity space, < 5 ms response

**DWA intuition**: imagine standing in the middle of a room and fanning out 900 possible walking paths (different speeds and turn rates). Walk each one a few steps, checking: will I hit something? Am I heading toward the door? Am I moving fast enough? Pick the best one, take one step, then re-fan 900 paths.

**APF local-minima intuition**: you are an iron ball attracted by a magnet (goal), but two reverse-polarity magnetic pillars are on your path. When the pillars' push forces are exactly symmetric, the magnet's pull is exactly opposite to their resultant force → **you float in midair, unable to move**.

**ORCA intuition**: you meet someone in a corridor. Pure VO means you both step left simultaneously → then both step right → oscillation. ORCA means you say "I step half-right, you step half-left, we split it" → smooth passing. But if three people surround you each imposing a half-plane constraint, **the intersection is empty** → you can only emergency-stop. That is Freezing Robot.

**CBF intuition**: CBF does not "force you to brake" — it "mandates that the closer to a cliff you are, the harder you must brake". $\alpha(h(x)) = \gamma h$ means: 10 m from cliff you can approach at 5 m/s; at 1 m you can only approach at 0.5 m/s; at 0 you cannot approach at all. **Geometric decay → never falls off**.

**MPC + Prediction spatiotemporal tube intuition**: a pedestrian is like a wriggling spacetime worm (probability distribution at each $k$ seconds in the future); your car draws its own spacetime tube. **Two non-intersecting spacetime tubes = no collision**. The 3σ inflation gives the wriggling worm a buffer.

**Flight Corridor intuition**: UAV flying in a 3D point cloud. Directly "avoid every point" is a painfully slow non-convex problem. **Slice free space into a string of connected transparent bubbles** (convex polyhedra), and the UAV only needs to "find a smooth curve inside bubble $i$, exiting into bubble $i+1$'s entrance" → the whole trajectory is solved by QP in seconds.

**7-DoF null-space intuition**: you hold a full cup of water (end effector 6-DoF pose locked); your elbow has 1 remaining DoF free to rotate. A worker pushes your elbow → your elbow gives way, but **the water cup position and orientation stay perfectly still, not a drop spills**. That 1-DoF redundancy is the entire magic of "null-space avoidance".

**Simulator observations**:
- **Gazebo + Nav2**: switch DWA / TEB / MPPI on the same map, observe differences in wide corridors vs narrow passages vs crowds
- **Isaac Sim + ORCA**: 100 AGVs at a warehouse intersection, observe the instant Freezing Robot happens (all freeze simultaneously)
- **MuJoCo + CBF-QP**: 7-DoF Franka tracking a circular end-effector trajectory while obstacles are randomly generated → elbow naturally yields but end trajectory unchanged
- **AirSim / Flightmare + FASTER**: UAV flying through a 10 m/s window, observe the moment primary fails and it switches to backup hover

## Implementation Link

**Six representative engineering scenarios** (mapping to 9-family selection):

1. **ROS 2 Nav2 layered planning (F1)**: `bt_navigator` uses a Behavior Tree to coordinate global planner (NavFn / Smac) with local controller (DWA / TEB / MPPI). Global path published as `nav_msgs/Path`; local controller reads `local_costmap` + global path every tick and outputs `geometry_msgs/Twist`.

2. **Autonomous driving local planning (F3, Autoware / Apollo)**: MPPI or Lattice planner at 30–100 Hz in Frenet frame; cost terms include lane-center deviation, obstacle SDF, Social-LSTM-predicted trajectory 3σ spatiotemporal tube, comfort (jerk), and traffic rules. Output feeds a PID / Stanley / MPC tracking controller.

3. **Multi-robot warehouse logistics (F2, 100+ AGVs)**: each AGV runs ORCA decentralized reciprocal collision avoidance — no central coordinator. Complexity $O(n)$ ($n$ = nearby neighbors), scales to 100+ robots. Priority arbitration: loaded > empty; intersections use auction protocol; Freezing cases escalate to regional central dispatch.

4. **RL + CBF spinal reflex (F4 + F8, quadruped)**: policy network outputs $u_{\text{nom}}$ → CBF-QP projects to safe space → joint torques. Typical frequencies: policy at 50 Hz, CBF-QP at 500–1000 Hz. Boston Dynamics Atlas, ANYbotics ANYmal use similar architectures.

5. **High-speed UAV flight (F6, FASTER)**: at 50 Hz, solve primary + backup B-splines simultaneously. Primary succeeds → execute; failure → switch to backup. Flight Corridor QP produces convex polyhedral trajectory in 5–10 ms.

6. **Collaborative manipulator HRI (F9, Cobot)**: `moveit_servo` reads Twist commands for real-time Cartesian tracking; `moveit_core` checks self-collisions + external obstacles; SSM speed limits enforced. Main task controlled via Jacobian end-effector; avoidance via $\mathbf{J}^\top \mathbf{F}_{\text{rep}}$ in null space.

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

**Layered avoidance skeleton** (A-level industrial template):

```python
class HierarchicalAvoidance:
    """Three lines of defense: Global + Local + Reactive Safety"""

    def __init__(self):
        self.global_planner = AStarPlanner()          # F1 guarantees reachability
        self.local_planner = MPPIController()         # F1/F3 high-frequency evasion
        self.pedestrian_predictor = SocialLSTM()      # F3 dynamic prediction
        self.safety_filter = CBFQP()                  # F4 spinal-reflex net

    def step(self, obs):
        # L1: Global (1 Hz)
        if self.global_replan_needed(obs):
            self.global_path = self.global_planner.plan(obs.map, obs.goal)

        # L2: Local (30 Hz) + spatiotemporal costmap
        pred_trajectories = self.pedestrian_predictor.predict(obs.pedestrians, horizon=3.0)
        spatiotemporal_costmap = build_stcmap(obs.costmap, pred_trajectories, sigma=3.0)
        u_nom = self.local_planner.solve(obs.state, self.global_path, spatiotemporal_costmap)

        # L3: Reactive Safety (1 kHz)
        u_safe = self.safety_filter.project(obs.state, u_nom)
        return u_safe
```


## Common Misconceptions

1. **"The local planner can replace the global planner"** — the local planner sees only a small window (typically 3–5 m) and gets trapped at local minima inside U-shaped corridors or mazes. The global planner provides directional guidance. DWA / APF both get stuck in U-shaped obstacles. **Avoid**: always maintain the layered architecture; if the local planner spins in place beyond a timeout threshold, trigger a global replan; a real A-level system has three layers (Global + Local + Reactive Safety).

2. **"DWA works for every chassis"** — DWA assumes constant-curvature arcs (constant $v, \omega$) and poorly models Ackermann constraints (minimum turning radius). Forcing it onto a car-like platform produces infeasible commands. **Avoid**: use TEB or MPPI for Ackermann; use NH-ORCA for multi-robot non-holonomic scenarios; upgrade to F3 family MPC + Prediction for high-dynamics (autonomy / humanoid).

3. **"Inflate obstacles as much as possible for maximum safety"** — over-inflation erases traversable space, freezing the robot in front of doorways (CBF triggers the same problem: overly conservative $\alpha$ freezes everything). **Avoid**: set `inflation_radius` to robot radius + modest safety margin (0.1–0.3 m); use `cost_scaling_factor` for a cost gradient; balance $\alpha$ aggressiveness in CBF.

4. **"Reactive avoidance (VO / RVO) does not need prediction"** — VO assumes obstacles move at constant velocity in a straight line; it fails when a pedestrian suddenly turns. **Avoid**: feed a pedestrian trajectory prediction module (Kalman / IMM / Social-LSTM) upstream and convert predicted paths into spatiotemporal collision cones; pure ORCA in dense crowds triggers Freezing Robot Problem.

5. **"APF is simpler than CBF, just use APF"** — APF has fatal **local minima** (two symmetric obstacles centered / goal behind a U-shaped wall → attraction and repulsion cancel). CBF does not actively generate forces — it only projects safely → naturally free of local minima. **Avoid**: industrial scenarios (AEB, Cobot safety layer, RL spinal reflex) prioritize CBF-QP; APF only for teaching or simple prototypes.

6. **"Treating pedestrians as moving cylinders is enough"** — this triggers Freezing Robot Problem: in dense crowds, all half-planes intersect to the empty set → frozen emergency stop. **Avoid**: introduce **Social-Aware** (SFM / SARL Self-Attention / Group-Aware) to model the "people yield to people" negotiation; a physical robot must proactively nudge to move through crowds.

7. **"A flashy end-to-end RL avoidance demo can ship"** — RL black boxes cannot pass ISO 13849 / 15066 SIL / PLd functional safety certification; there is no way to debug which layer failed after a collision. **Avoid**: the engineering compromise is **"RL gives the proposal + CBF-QP / MPC hard-safety filter catches"**, offering deterministic mathematical safety guarantees + RL generalization.

## Situational Questions

<details>
<summary>Q1 (medium): Your robot uses DWA in a crowded exhibition hall and keeps emergency-stopping → rerouting → nearly stalling. How do you analyze this? What tools do you use? What pitfalls must you avoid?</summary>

**Complete reasoning chain**:

1. **Diagnose root cause**: DWA's obstacle layer reflects only *current* positions — no pedestrian prediction. The moment a person moves, DWA discovers a previously safe velocity is now dangerous, emergency-stops, resamples, hits another sudden change, and oscillates. This is classic F1 family (reactive) failure in a dynamic environment.
2. **Add dynamic prediction (upgrade to F3 family)**:
   - Integrate a pedestrian tracking module (e.g., `spencer_people_tracking`, or custom YOLO + DeepSORT)
   - Predict each pedestrian's trajectory 2–3 s ahead with Kalman / IMM / Social-LSTM
   - Write predicted positions into a **spatiotemporal costmap** with decaying cost (farther future = lower cost, reflecting increasing uncertainty)
3. **Upgrade the planner**:
   - Option A: keep DWA but feed spatiotemporal costmap as an obstacle layer — lowest-cost change
   - Option B: switch to TEB (native dynamic obstacle support) or MPPI (add predicted-trajectory collision term to cost)
   - Option C (A-level): Chance-Constrained MPC puts Social-LSTM's Gaussian $\mathcal{N}(\boldsymbol{\mu}_k, \Sigma_k)$ directly into hard constraints with 3σ inflation
4. **Pitfalls to avoid**:
   - Do not stretch prediction window beyond ~3 s — accumulated error floods the costmap → upgrades to Freezing Robot Problem
   - Do not forget pedestrians may stop suddenly — prediction model needs a "velocity decay" fallback
   - Exhibition crowds: prediction alone insufficient; add **Social-Aware** modeling "robot yields to people, people yield to robot"

**What the interviewer wants to hear**: clear distinction reactive vs predictive; concrete costmap integration plan; awareness of overlong prediction horizon side effects; mention Social-Aware as a more advanced option.

</details>

<details>
<summary>Q2 (medium-hard): MoveIt-planned manipulator motions are too slow for dynamic conveyor pick-and-place — planning-to-execution > 500 ms, object has moved. How do you fix it?</summary>

**Complete reasoning chain**:

1. **Understand the bottleneck**: MoveIt's RRT / PRM searches the full C-space, 100–500 ms per plan, and ignores obstacle motion.
2. **Layered solution**:
   - **Low-frequency layer (~2 Hz)**: MoveIt pre-plans a reference trajectory to a *predicted* grasp point (object velocity × estimated arrival time as lead)
   - **High-frequency layer (100–1000 Hz)**: correct the reference in joint space with QP or MPC
     - Read object real-time pose from visual tracking
     - Use SDF for environment collision constraints
     - Solve $\min \|q - q_{\text{ref}}\|^2$ s.t. collision + joint limits + velocity limits
3. **SDF acceleration**: build ESDF offline with `voxblox` or GPU voxelization; query online in $O(1)$, collision checks in < 10 ms
4. **Add HRI safety layer (F9)**:
   - If workers near the conveyor → mandatory SSM (Speed and Separation Monitoring)
   - $\mathbf{J}^\top \mathbf{F}_{\text{rep}}$ potential-field avoidance + null-space projection: elbow yields compliantly while end-effector tracking unaffected
5. **Toolchain**:
   - `moveit_servo`: Twist command for real-time Cartesian-space tracking
   - `curobo` (NVIDIA): GPU-accelerated motion planning, planning time < 10 ms
   - `Franka Control Interface`: 1 kHz joint-torque control

**What the interviewer wants to hear**: layered planning mindset (offline coarse + online fine); SDF performance advantage; familiarity with `moveit_servo` / `curobo`; mention SSM / null-space as HRI upgrade.

</details>

<details>
<summary>Q3 (hard): 100 AGVs in a warehouse deadlock frequently at intersections. The current central dispatcher has high latency and is a single point of failure. How do you design a decentralized avoidance scheme?</summary>

**Complete reasoning chain**:

1. **Choose the algorithm (F2 family)**: **ORCA** (Optimal Reciprocal Collision Avoidance)
   - Each AGV computes its safe velocity set independently — no central coordinator
   - Complexity $O(n)$ per agent ($n$ = nearby neighbors); 100 AGVs each see only a 5 m radius
   - Mathematical guarantee: if all agents follow ORCA, no collisions (reciprocal assumption)
2. **Priority arbitration**:
   - Static priority: loaded AGV > empty AGV > charging AGV
   - Dynamic priority: closer to destination → higher priority (avoids near-goal starvation)
   - Intersections: higher-priority half-planes impose stronger constraints on lower-priority agents
3. **Freezing Robot Problem handling**:
   - **Hard → soft constraints**: introduce slack $\epsilon \geq 0$ to allow small half-plane violation with huge penalty
   - **Hybrid architecture**: if regional deadlock (> 3 AGVs), escalate to regional central dispatcher (fallback)
   - **Topological graph search**: LP infeasible on this path → global replan around the crowd
4. **Deadlock detection & recovery**:
   - Monitor: if an AGV's speed drops below threshold for $T$ seconds → flag "suspected deadlock"
   - Recover: randomly select one AGV to reverse 0.5 m and replan, breaking symmetry
5. **Pitfalls to avoid**:
   - ORCA assumes all agents obey the protocol — human-driven forklifts require F4 safety layer (CBF hard floor)
   - ORCA ignores non-holonomic constraints — Ackermann AGVs need NH-ORCA
   - Extreme density (> 10 neighbors) where plain ORCA fails → must upgrade Social-Aware or MPC + Prediction

**What the interviewer wants to hear**: decentralized vs centralized trade-off; ORCA reciprocal guarantee; hybrid architecture (decentralized by default + regional central dispatch fallback); knowledge of Freezing Robot Problem and fixes.

</details>

<details>
<summary>Q4 (hard): Your autonomous vehicle uses MPPI for local planning. It runs well on highways but scrapes walls in narrow alleys, and emergency-brakes on pedestrians instead of detouring. Without switching algorithms, how do you fix it?</summary>

**Complete reasoning chain**:

1. **Diagnose root causes (two stacked problems)**:
   - **Narrow-alley wall scraping**: highway settings have large `vx_std` and `wz_std` (broad exploration); in narrow alleys most trajectories collide and are discarded, leaving too few valid samples
   - **Pedestrian emergency braking**: current cost function treats pedestrians as static obstacles — no Social-LSTM prediction → any pedestrian movement triggers high cost
2. **Dynamic parameter switching**:
   - Detect entry into narrow passages (traversable width in global costmap drops below threshold), auto-switch parameter set:
     - Lower `vx_std / wz_std` (narrow sampling range, concentrate in feasible regions)
     - Increase `batch_size` (sample more densely in the narrower range)
     - Lower `temperature` (trust few good trajectories more, less polluted by bad ones)
3. **Cost function upgrade to F3**:
   - Add Chance-Constrained term: $\|\mathbf{p}_{\text{ego}}(k) - \boldsymbol{\mu}_{\text{ped}}(k)\|^2 \geq (d_{\text{safe}} + 3\sigma_{\text{ped}}(k))^2$
   - Feed Social-LSTM or Trajectron++ pedestrian predictions
   - For each MPPI trajectory's $S(\tau)$, pedestrian cost decays with future time (increasing uncertainty)
4. **Alternative optimizations**:
   - Importance sampling: densely sample near previous tick's optimal trajectory
   - Warm-start: use previous tick's optimal control sequence as the mean
   - Add `PathFollowCritic` to strictly follow global path centerline in narrow alleys
5. **Defense in depth**: add an F4 CBF-QP beneath MPPI — project MPPI output onto the safe space

**What the interviewer wants to hear**: understanding MPPI sampling efficiency in tight spaces; dynamic hyperparameter adaptation; Chance-Constrained upgrade awareness; CBF as last safety net.

</details>

<details>
<summary>Q5 (hard): A service robot navigating a hospital lobby is frozen for 10 minutes by a group of visitors chatting in a circle. Plain ORCA and plain DWA both fail. How do you design the system?</summary>

**Complete reasoning chain**:

1. **Diagnose root causes (F5 family core scenario)**:
   - ORCA failure: Freezing Robot Problem (all half-planes intersect to empty set)
   - DWA failure: myopic + no pedestrian intent modeling → never finds safe velocity
   - Fundamental issue: **treating pedestrians as moving cylinders**, ignoring "people yield to people and to polite robots" social dynamics
2. **Three-layer solution**:
   - **L1 Group-Aware detection**: DBSCAN + F-formation detection of "three people in a chatting circle" → cluster as impassable group → detour outside
   - **L2 SARL Self-Attention Policy**: end-to-end RL learns "detour from crowd edge, yield to leader" social rules; Self-Attention handles variable crowd size
   - **L3 SFM safety net**: if SARL output is too aggressive, SFM's reactive repulsion field catches basic safety
3. **Communication value-add**:
   - Robot emits "excuse me" sound cue → pedestrians usually yield
   - VLM voice module: "excuse me, may I pass" natural-language request
4. **Freezing recovery**:
   - If truly infeasible (> 30 s no progress): robot says "please let me through" and starts **slowly approaching** (< 0.2 m/s) — crowds usually open up
   - **Proactively nudging** is the essence of social-aware navigation: without nudging, you never get through
5. **Pitfalls to avoid**:
   - Do not stretch prediction window (chatting people barely move — next 30 s all same position → costmap saturates)
   - Do not deploy RL exploration in crowds — functional safety forbids it; always place CBF beneath before deployment

**What the interviewer wants to hear**: clear articulation of Freezing Robot Problem; Group-Aware / SARL / SFM three-layer division; "proactive nudge" social-dynamics insight; awareness of service-robot commercialization value (hospitals/malls).

</details>

<details>
<summary>Q6 (hard): You train a quadruped end-to-end RL avoidance policy in Isaac Lab that beautifully traverses a valley in simulation, but crashes into trees frequently during real outdoor deployment. How do you diagnose and fix it?</summary>

**Complete reasoning chain**:

1. **Diagnose root cause (F8 family classic failure)**:
   - **Sim-to-Real Gap**: simulator textures / lighting / depth noise differ significantly from reality → "obstacle visual features" learned in sim fail in reality
   - Possible concrete phenomena: leaf occlusion on depth camera, sunlight saturation, low tree-vs-background contrast
2. **Fix 1: Domain Randomization**:
   - During training, **aggressively randomize**: lighting (8-stop HDR range), textures (random UVs / procedural noise), camera intrinsics / noise, FOV, action latency, friction coefficients, mass distribution
   - Forces the network to abandon specific visual features → learn truly invariant geometric avoidance depth
3. **Fix 2: Privileged Teacher-Student distillation**:
   - **Teacher** (simulator god view): precise 3D + velocity + friction → quickly learns perfect avoidance
   - **Student** (real vision): RGB-D pixels → Behavior Cloning to match Teacher actions
   - Distillation loss: $\mathcal{L} = \mathbb{E}[\|\pi_{\text{student}}(a|o_{\text{vision}}) - \pi_{\text{teacher}}(a|s_{\text{priv}})\|^2]$
4. **Fix 3: add F4 CBF safety net**:
   - RL policy outputs $u_{\text{nom}}$ → CBF-QP projects to safe space → joint torques
   - Even if RL decides wrongly in front of a tree, CBF hard constraint prevents collision
5. **Fix 4: Diffusion Policy to avoid multimodal collapse**:
   - PPO / SAC may learn "left detour / right detour" average → crash tree center
   - Diffusion Policy preserves multimodality → real deployment randomly samples left or right
6. **Validation order**: first add Domain Randomization in Isaac Sim and retrain → then Teacher-Student → then stack CBF → deploy in small-area real tests → expand

**What the interviewer wants to hear**: Sim-to-Real Gap is the #1 killer; Domain Randomization + Teacher-Student is standard answer; CBF safety net is engineering compromise; mention Diffusion Policy to solve multimodal collapse.

</details>

## Interview Angles

1. **Layered planning architecture is the core design principle** — proves you understand the "compute budget vs reaction speed" trade-off. **Why this is the key**: interviewers want to see systems thinking, not just algorithm memorization. **Bring out (two-minute version)**: "I split the planning stack into Global (1 Hz A\*, guarantees global reachability) + Local (30+ Hz DWA/MPPI, guarantees real-time avoidance) + Reactive Safety (1 kHz CBF / VO, safety net). Low-frequency for the big picture, high-frequency for fine evasion, reactive layer for the last line — this is not a textbook taxonomy, it is the only viable solution under real compute budgets."

2. **Integrating dynamic prediction into the costmap is the differentiating skill** — separates "Nav2 demo runner" from "dynamic-environment handler." **Why this is the key**: pure reactive avoidance was retired from industry 5+ years ago, but many candidates still stop at DWA. **Bring out**: "Pure reactive avoidance works for static maps but collapses in crowds. I write pedestrian tracking + Social-LSTM predictions into a spatiotemporal costmap layer with a 2–3 s horizon, and use Chance-Constrained MPC with 3σ inflation as a hard constraint covering uncertainty. This is the L4 industrial standard."

3. **Algorithm selection depends on chassis kinematics and compute** — shows you make engineering decisions based on constraints, not memorized textbook answers. **Why this is the key**: selecting the wrong algorithm is more fatal than tuning the wrong parameter. **Bring out**: "Differential drive defaults to DWA — simple, efficient. Ackermann uses TEB for minimum turning radius. High-dimensional nonlinear systems get MPPI — but only if a GPU is available. Wrong algorithm choice beats wrong tuning."

4. **ORCA reciprocal half-plane + Freezing Robot Problem** — deep test for multi-robot avoidance. **Why this is the key**: interviewers use this to filter candidates who know the RVO name but don't understand the LP failure mode. **Bring out**: "ORCA linearizes the geometric collision cone into half-plane constraints and solves LP in $O(n)$ milliseconds. But in dense crowds, all half-planes intersect to the empty set → Freezing → LP infeasible → frozen. The standard fix is hard-to-soft constraint with slack variables, or a hybrid architecture escalating to regional central dispatch."

5. **Chance-Constrained MPC embeds prediction uncertainty as a 3σ hard control constraint** — the soul sentence of L4 industrial standard. **Why this is the key**: saying this proves you've read Waymo / Apollo papers; not saying it means you only know textbook MPC. **Bring out**: "Social-LSTM predicts each future step's pedestrian as a 2D Gaussian; MPC inflates the avoidance constraint outward by 3σ to cover 99.7% probability. Erratic pedestrians automatically get a wider lateral margin. Prediction uncertainty seamlessly embedded as a control hard constraint — the modern L4 answer."

6. **CBF Forward Invariant Set is the unified language of safety control post-2020** — shows you track academic frontiers. **Why this is the key**: CBF moved from paper to industry (AEB / Cobot / Atlas) in just five years; answering well proves you read recent papers. **Bring out**: "$\dot{h} + \alpha(h) \geq 0$ means the closer to the danger boundary, the approach speed must decay geometrically to zero. CBF is a safety filter that doesn't actively generate forces, so no APF local minima. HOCBF solves Relative Degree > 1 position constraints, handling second-order system braking distance."

7. **CBF-QP is the spinal reflex layer for RL** (standard industrial answer for RL + hard safety) — key compromise for end-to-end RL deployment. **Why this is the key**: this is the standard solution to "RL demos look cool but can't pass safety cert"; nearly every embodied / quadruped / manipulator company uses it. **Bring out**: "RL brain outputs $u_{\text{nom}}$; CBF-QP least-squares projects onto the safe space. Microsecond convex optimization → RL generalization + mathematical safety floor. This is the standard architecture for RL deployment under ISO 13849 PLd certification."

8. **SARL Self-Attention for variable crowd sizes + Group-Aware polite bypass** — core modern Social-Aware architecture. **Why this is the key**: last mile of service-robot commercialization (hospital, mall, airport), freezing = commercial failure. **Bring out**: "In dense crowds with varying $N$, traditional RL struggles with variable state. SARL uses Self-Attention to focus on likely-collision key pedestrians, compressing variable-length crowd state to fixed-length context. Group-Aware uses F-formation to detect chatting circles → detour outside. This determines whether the robot integrates clumsily or gracefully into human society."

9. **FASTER dual-trajectory parallel + Flight Corridor B-spline convex-hull property** — elegant math for high-speed UAV avoidance. **Why this is the key**: 10 m/s UAV map latency is enough for a crash — safety fallback is mandatory. **Bring out**: "FASTER solves primary (aggressive) and backup (guaranteed-to-stop) trajectories each cycle. On failure, seamless switch to backup → absolute safety. Flight Corridor inflates 3D free space into overlapping convex polyhedra; B-spline convex-hull property guarantees the entire curve never escapes → non-convex avoidance becomes a convex QP solved in milliseconds."

10. **Manipulator $\mathbf{J}^\top \mathbf{F}_{\text{rep}}$ + null-space "elbow yields, cup stays still" is the mathematical skeleton of ISO 10218 / 15066 PLd certification** — the signature selling point of Cobot commercialization. **Why this is the key**: Cobot customers (UR / KUKA / Franka users) pay for **safety**; citing certification math proves you understand industrial logic, not just toy examples. **Bring out**: "Each link wrapped in a Capsule; Jacobian transpose maps Cartesian repulsion to joint torques. 7-DoF redundancy projected onto null space → elbow yields compliantly when pushed, but the cup at the end stays still. This is the underlying math of ISO 15066 PLd certification."

11. **Privileged Teacher-Student distillation is the standard Sim-to-Real paradigm** — industrial answer for end-to-end RL avoidance. **Why this is the key**: ETH ANYmal, Boston Dynamics Atlas all use this; not knowing it is behind-the-times. **Bring out**: "Teacher in simulator with god-view (precise 3D + friction) learns perfect avoidance → Student in reality only has cameras → Behavior Cloning matches Teacher actions. Plus Domain Randomization (aggressive randomization of lighting, textures, noise) forces Student to learn geometric invariants, avoiding overfitting the simulator renderer."

12. **Diffusion Policy avoids "PPO averages and crashes" multimodal collapse** — 2024 frontier must-answer. **Why this is the key**: interviewers use this to separate "read only up to 2023 papers" from "tracks 2024 frontier." **Bring out**: "PPO / SAC Gaussian averaging makes 'left detour + right detour' average into crashing the obstacle center. Diffusion Policy treats avoidance actions as denoising generation, perfectly preserving multimodal solutions. This is why Columbia's 2023 paper went viral in robotics."

## Further Reading

- **Fox et al., *The Dynamic Window Approach to Collision Avoidance* (1997)** — F1 family DWA original paper; foundational for velocity-space sampling and understanding dynamic-window math
- **Rösmann et al., *Timed-Elastic-Band Local Planner* (2013–2017 series)** — TEB's full evolution from basic concept to multi-topology path selection; key reference for spatiotemporal elastic band optimization
- **Williams et al., *Information Theoretic MPC (MPPI)* (2017)** — MPPI's information-theoretic foundation; why Boltzmann weighting is more robust than arg min
- **van den Berg et al., *Reciprocal n-Body Collision Avoidance (ORCA)* (2011)** — F2 family mathematical guarantee for decentralized multi-robot avoidance; geometry of reciprocal half-planes
- **Alonso-Mora et al., *Collision Avoidance for Quadrotor Swarms under Uncertainty* (2019)** — ORCA + Chance Constraints; industrial F2 / F3 fusion
- **Alahi et al., *Social LSTM: Human Trajectory Prediction in Crowded Spaces* (2016)** — F3 family classic; Social Pooling layer explained
- **Salzmann et al., *Trajectron++* (2020)** — stronger multimodal trajectory prediction than Social-LSTM; industry L4 default
- **Ames et al., *Control Barrier Functions: Theory and Applications* (2019)** — F4 family CBF / HOCBF authoritative survey; from theory to RL safety control
- **Chen et al., *Socially Aware Motion Planning with Deep Reinforcement Learning (CADRL/SARL)* (2017–2019)** — F5 family Social-Aware RL foundational papers
- **Helbing & Molnár, *Social Force Model for Pedestrian Dynamics* (1995)** — F5 family SFM original paper; reactive social mechanics basics
- **Tordesillas et al., *FASTER: Fast and Safe Trajectory Planner* (IROS 2019)** — F6 family dual-trajectory classic; MIT CSAIL landmark
- **Zhou et al., *EGO-Planner* (RA-L 2021)** — F6 family Flight Corridor / B-spline industrial implementation
- **Ranftl et al., *MiDaS / ZoeDepth* (2020–2023)** — F7 family monocular depth; key tech for Tesla FSD / Skydio pure vision
- **Chi et al., *Diffusion Policy* (RSS 2023)** — F8 family Diffusion preserving multimodal action distributions; 2024 robotics hit
- **Hwangbo et al., *Learning Agile and Dynamic Motor Skills for Legged Robots* (Science Robotics 2019)** — F8 family Privileged Teacher-Student distillation classic; ANYmal outdoor quadruped avoidance
- **ISO/TS 15066 — Collaborative Robots Speed and Separation Monitoring** — F9 family Cobot safety certification standard; SSM / Power and Force Limiting spec
- **Haddadin et al., *Robot Collisions: A Survey* (RAL 2017)** — F9 family manipulator HRI avoidance panoramic survey
- **ROS 2 Nav2 documentation — Controller Plugins** — complete configuration reference and tuning guide for DWA, TEB, and MPPI controllers
- **NVIDIA Isaac Lab / cuRobo** — GPU-accelerated motion planning and obstacle avoidance; go-to toolchain for real-time high-dimensional manipulator planning
- ***Embodied AI Algorithm Engineer Interview Questions*, Ch8.3–8.5 Dynamic Avoidance / Social-Aware / RL Avoidance** — high-frequency interview topics: layered architecture, prediction integration, CBF safety-layer talking points
