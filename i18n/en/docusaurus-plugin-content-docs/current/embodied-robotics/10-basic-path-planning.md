---
title: "Fundamental Path Planning (A*, RRT, APF)"
prerequisites: ["07-forward-kinematics-dh"]
estimated_time: 90
difficulty: 4
tags: ["path-planning", "a-star", "rrt", "apf", "c-space", "theta-star", "d-star-lite", "informed-rrt-star", "bit-star", "hybrid-a-star", "mppi", "teb", "cbs", "orca", "esdf", "diffuser"]
sidebar_position: 10
---

# Fundamental Path Planning (A*, RRT, APF)

## You Will Learn

- Precisely distinguish path planning, motion planning, and trajectory planning — and never mix them in interviews
- Given a "get from A to B" problem, immediately decide based on **dimensionality**, **kinodynamic constraints**, **dynamism**, and **multi-robot** whether to use A*, Theta*, D* Lite, RRT, RRT\*, Informed RRT\*, BIT\*, Hybrid A\*, Lattice, DWA/TEB/MPPI, or Diffuser — and articulate the **full selection reasoning chain**
- Understand why Configuration Space (C-space) is the core abstraction of path planning, and how it reduces collision checking to "is this point inside a region"
- When a Local Planner deadlocks, diagnose whether it is a **costmap inflation mismatch**, an **ignored non-holonomic constraint**, a **myopic (short-sight) trap**, or a **high-dimensional narrow passage** — and give a concrete fix
- Articulate why Learning-based planning (Diffuser / NeRF / CVAE / VLM) in industry is a "proposal / heuristic" layer rather than an end-to-end replacement

## Core Concepts

**Precise definition**: **Path planning** finds a **collision-free geometric route** from a start to a goal in a known or partially known environment (pure space, no time). It is the strategic navigator of robot autonomy — it answers "which road to take", not "how to drive it" or "how fast".

**Three commonly confused terms**:
- **Path planning**: pure geometric route (waypoint sequence), no speed/acceleration
- **Motion planning**: umbrella term — any start-to-goal motion problem (geometry + kinodynamics)
- **Trajectory planning**: path + time parameterization (when each waypoint is reached, at what speed)

**Configuration Space (C-space)**: collapse all robot DOFs (joint angles, position, orientation) into a single point, and inflate obstacles into C-space obstacles. Collision checking then becomes "is this point inside the C-obstacle region" — geometry reduced to set membership. A 2D mobile robot has C-space $\mathbb{R}^2$; a 7-DoF manipulator has C-space $\mathbb{R}^7$ (or $T^7$ accounting for joint angle wraparound).

**Position in the perception → planning → control loop**:
- **Node**: **planning front-end** (global planner) + **planning back-end** (local planner / trajectory optimizer)
- **Input**: start configuration $q_{\text{start}}$, goal configuration $q_{\text{goal}}$, environment map (occupancy grid / octomap / ESDF / point cloud), kinodynamic constraints (max accel, min turning radius)
- **Output**: waypoint sequence $\{q_0, q_1, \ldots, q_N\}$ (collision-free C-space path) + reference velocity / control commands
- **Downstream**: trajectory optimizer (adds time/velocity) → controller (tracks trajectory) → actuators

**One-line summary**: "Path planning is the geometric engine for finding safe passage in a multi-dimensional maze — dimensionality, kinodynamics, and dynamism together dictate algorithm choice."

---

### A* Algorithm

**Core formula**:

$$
f(n) = g(n) + h(n)
$$

**Physical meaning**: $g(n)$ is the known cost from start to node $n$ (how far we've traveled), $h(n)$ is the heuristic estimate from $n$ to goal (how far remains). $f(n)$ is the "total estimated cost through this node" — A* always expands the lowest-$f$ node next.

**Two critical properties** (the dual guarantee for interviews):
- **Admissibility**: $h(n) \le h^*(n)$ — the heuristic never overestimates true shortest cost. **Guarantees optimality**. Overestimation causes A\* to wrongly skip the true optimal path
- **Consistency (monotonicity)**: $h(n) \le c(n, n') + h(n')$ (triangle inequality) — the estimate drop between neighbors never exceeds the actual edge cost. **Guarantees each node is popped at most once**, overall $O(N \log N)$ with no re-expansion

With $h(n) = 0$, A\* degenerates to Dijkstra (uniform wavefront expansion — slow but robust, globally shortest guaranteed).

**Applies to**: low-dimensional discrete spaces (2D/3D grid maps) — warehouse AGVs, indoor navigation.

**Fatal weakness**: as C-space dimension grows, grid nodes explode exponentially (curse of dimensionality). A 7-DoF arm discretized at 100 bins per axis gives $100^7 = 10^{14}$ nodes — completely infeasible.

---

### Theta\* / D\* Lite (Any-angle and Dynamic)

**Theta\* core idea**: classical A\* on an 8-connected grid can only extend in multiples of 0°/45°/90°. To follow a 15° shortcut it must zig-zag via horizontal + diagonal segments → **jerky zig-zag path** → robot wobbles, wears actuators, wastes energy. Theta\* allows parent-child nodes to be **non-adjacent**. During expansion it runs a **Line-of-Sight (LoS) check**: if the grandparent directly sees $s'$ (nothing in between), set $s'$'s parent to the grandparent, skipping intermediate nodes. **Path smoothing happens during search**, yielding near-true-shortest Euclidean paths.

**D\* Lite core idea** (the standard answer for dynamic environments):
- **Reverse search** from Goal
- Each node maintains $g(n)$ (current known cost) + $\text{rhs}(n)$ (one-step lookahead cost from neighbors)
- Static: $g = \text{rhs}$; sensor finds new obstacle → only local edge cost updates → $g \neq \text{rhs}$ creates **inconsistency**
- Priority queue only re-propagates the "inconsistent" frontier → reuses prior heuristic and cost map
- Orders of magnitude faster than re-running A\*

**Interview key point**: never re-run A\* in dynamic environments (deadlocks + oscillates). Standard answer routes:
1. **D\* Lite incremental repair** — update only local nodes affected by new obstacles
2. **Layered planning** — Global topology planner + Local (TEB/DWA/MPPI) in velocity space for dynamic obstacles

---

### RRT (Rapidly-exploring Random Tree)

**Core loop** (per iteration):

1. **Random sample**: draw $q_{\text{rand}}$ uniformly from C-space
2. **Nearest**: find tree node $q_{\text{near}}$ closest to $q_{\text{rand}}$
3. **Steer**: extend a fixed step from $q_{\text{near}}$ toward $q_{\text{rand}}$, yielding $q_{\text{new}}$
4. **Collision check**: is segment $q_{\text{near}} \to q_{\text{new}}$ obstacle-free?
5. If yes → add to tree; if no → discard, resample

**RRT's Voronoi Bias (the physical intuition for interviews)**: isolated tree nodes have the largest Voronoi cells → random samples land in large cells with high probability → tree branches **grow rapidly into unexplored open space**. This is why the algorithm is called "**Rapidly-exploring**" — geometrically biased toward unexplored coverage.

**Probabilistic Completeness**: $\lim_{N \to \infty} P(\text{find path}) = 1$ — if a solution exists and sample count $\to \infty$, the probability of finding it → 1. But **no optimality guarantee** (paths are typically wiggly).

**Goal Biasing (5-10%)**: pure random sampling is blind; with 5-10% probability sample the goal directly — preserves exploration while adding attraction, significantly speeds convergence.

**RRT\***: adds **rewiring** on top of RRT — after inserting $x_{\text{new}}$:
1. Find all neighbors $Q_{\text{near}}$ within radius $r$
2. **Choose best parent**: for each $q \in Q_{\text{near}}$, if reaching $x_{\text{new}}$ via $q$ is cheaper and collision-free, reparent
3. **Reverse rewire**: for each $q \in Q_{\text{near}}$, if routing $q$ through $x_{\text{new}}$ lowers $q$'s cost, cut and reconnect

This achieves **almost-surely asymptotically optimal** paths, at the cost of added computation (neighbor set maintenance + repeated cost evaluation).

$$
r = \gamma \left( \frac{\log n}{n} \right)^{1/d}
$$

**Physical meaning**: search radius shrinks as sample count $n$ grows; $d$ is C-space dimension. As $n \to \infty$, the path converges to the global optimum.

**Applies to**: high-dimensional C-space (6-DoF / 7-DoF arms), cluttered obstacle environments.

---

### Informed RRT\* / BIT\* / AIT\* (Modern Sampling-based)

**Informed RRT\***: after finding the first solution, construct a **prolate hyperspheroid with start/goal as foci** as the sampling region:

$$
\lVert x_{\text{rand}} - x_{\text{start}} \rVert_2 + \lVert x_{\text{rand}} - x_{\text{goal}} \rVert_2 \le c_{\text{best}}
$$

**Physical meaning**: any point outside the ellipsoid has start+goal distance exceeding the current best cost $c_{\text{best}}$ — so it **cannot improve the solution**. Sample only inside the ellipsoid, collapsing the space instantly. As $c_{\text{best}}$ shrinks, the ellipsoid contracts and sample density grows hundreds-fold → **10-100× speedup**.

**BIT\* (Batch Informed Trees)**:
- **Unifies Informed Sampling + A\* Graph Search**
- Batch sampling scatters points → A\* searches over the random-point graph
- **Lazy Edge Collision Check**: assume edges are safe; only check collision when A\* selects an edge as part of the optimal candidate → massive compute savings (collision checking consumes 80-90% of high-D planning time)

**AIT\* (Adaptively Informed Trees)**: simultaneously runs a forward random tree + a reverse heuristic estimation tree, dynamically updating the heuristic — adapts sampling focus in extremely complex mazes.

**OMPL is the industry de facto standard**: MoveIt, Nav2 are OMPL wrappers.

---

### APF (Artificial Potential Field)

**Core formula**:

$$
F_{\text{total}} = F_{\text{att}} + F_{\text{rep}} = -\nabla U_{\text{att}}(q) - \nabla U_{\text{rep}}(q)
$$

**Physical meaning**: goal generates an "attractive field" pulling the robot in, obstacles generate "repulsive fields" pushing it away. The robot descends the combined gradient.

- $U_{\text{att}}(q) = \frac{1}{2} k_{\text{att}} \lVert q - q_{\text{goal}} \rVert^2$ — attractive potential, grows with distance (spring pull)
- $U_{\text{rep}}(q)$: non-zero only within a safety threshold of obstacles, grows as distance shrinks (push)

**Fatal weakness**: **local minima** — in U-shaped obstacles or symmetric configurations, attractive and repulsive forces cancel, the robot stalls.

**Applies to**: typically a **local planner** or reactive obstacle layer, paired with a global planner like A\*.

---

### PRM (Probabilistic Roadmap)

**Idea**: offline, scatter many points in C-space, run collision checks, connect into a roadmap; online, attach start and goal to the roadmap and query with Dijkstra / A*.

**Applies to**: **static environment + multi-query**. Complementary to RRT — RRT is single-query (rebuild each time), PRM is multi-query (build once, query many). Standard choice for **factory pick-and-place repeated tasks**.

---

### Kinodynamic / Lattice / Hybrid A\* (Planning with Dynamics)

**Why pure geometric planning fails for self-driving / drones / quadrupeds**:
- Polyline paths violate max acceleration, min turning radius
- **Non-holonomic constraints**: Ackermann cars can't strafe or spin in place; constraint equation $\dot{x} \sin\theta - \dot{y} \cos\theta = 0$ (rear-wheel velocity only along body heading)
- Dynamic inertia limits
- Forcing polyline tracking → controller saturates → trajectory deviates or vehicle rolls over

**Dubins Car** (forward-only, min turning radius): optimal paths are **CCC (arc-arc-arc) or CSC (arc-straight-arc)** — 6 combinations total.

**Reeds-Shepp Car** (allows reverse): **46 word codes** (e.g. `C|C|C`, where `|` denotes gear change).

**Kinodynamic RRT** (core physical intuition): don't connect straight lines! At $x_{\text{near}}$ randomly sample **control input** $u$ (steering angle, throttle), feed into dynamics ODE and **Forward Integrate (RK4)** for $dt$. The integration endpoint is $x_{\text{new}}$. **Every branch is physically 100% executable**.

**Hybrid A\*** (Waymo / Tesla parking core):
- A* searches 3D state $(x, y, \theta)$
- Node expansion uses kinematic equations (steer left/center/right) integrated for a small step
- Reeds-Shepp model as the heuristic during node expansion
- In a narrow parking slot: "forward → hits obstacle, huge cost" vs "reverse `C|C` curve → passes narrow entrance, lower total cost" → **three-point reverse parking emerges automatically**

**Frenet Conformal Lattice** (highway self-driving gold standard):
- Classical Lattice is rigid — fails on curves / variable-width lanes
- Generate motion primitives in Frenet frame $(s, d)$ (longitudinal/lateral along lane)
- Map back to Cartesian → vehicle naturally follows curves
- Quintic polynomial primitives $d(s) = a_0 + a_1 s + \ldots + a_5 s^5$ ensure position, heading, and curvature align smoothly at endpoints

**Interview pitfall**:
- **Indoor AGV = differential drive**, allows in-place rotation → geometric A\* polyline + rotate-and-go works
- **Self-driving car = Ackermann**, strict min turning radius + $C^2$ curvature continuity → **must use Frenet Lattice / Hybrid A\***

---

### Local Planner Trinity (DWA / TEB / MPPI)

**DWA (Dynamic Window Approach)**: sample feasible control inputs in velocity space $(v, \omega)$, forward-simulate each for 1-2 seconds, pick best by cost:

$$
\text{Cost} = \alpha \cdot \text{heading} + \beta \cdot \text{distance} + \gamma \cdot \text{velocity}
$$

**Fatal weakness**: **myopic** — only looks 1-2 seconds ahead, spins in place in U-shaped obstacles or dead ends.

**TEB (Timed Elastic Band)**: treat the global path as a **time-parameterized elastic band**, with $\Delta T$ as a decision variable, solved by nonlinear optimization (G2O) integrating obstacles + time optimality + dynamics. **More robust to dynamic pedestrians**: can predict pedestrian positions at time $t$, bend in the spatiotemporal graph to go around + slow down and wait (stretch $\Delta T$).

**MPPI (Model Predictive Path Integral)**: the extreme of sampling-based MPC. GPU-parallel thousands of random control trajectories, weighted average by final cost.

**MPPI's key advantage: friendly to non-smooth cost**. TEB / G2O rely on gradient descent — discrete grid / step-function collision penalties blow up gradients. MPPI only **evaluates** cost (forward sim + integration), **no derivative required** → killer choice for off-road, rough terrain, step-function penalty landscapes.

**Scenario selection** (must-know for interviews):
- **Indoor narrow corridor** → **TEB** (DWA has very few valid samples in narrow spaces, gets stuck)
- **Dense crowds** → **TEB** or **MPPI** (spatiotemporal prediction)
- **Unstructured off-road** → **MPPI** (chassis slip + cliff non-smooth cost)

---

### Multi-robot Path Planning (MAPF / CBS / ORCA)

**MAPF definition**: $n$ agents share a graph; at every time $t$ no **Vertex Conflict** (same node) or **Edge Conflict** (swap edge); minimize total cost (Makespan or Sum-of-Costs). **NP-hard**.

**CBS (Conflict-Based Search)** two-level architecture:
- **Low-level**: each agent runs A\* independently, oblivious to others
- **High-level**: build a **Conflict Tree**
  - A, B both at $v$ at time $t$ → bifurcate into two child nodes
  - Left: "A forbidden from $v$ at $t$"; Right: "B forbidden from $v$ at $t$"
  - Low-level re-runs A\* under the new constraint
- **Globally optimal and complete**, but the conflict tree can explode exponentially

**Dense Warehouse sub-optimal solution** (100-AGV production answer):
1. **Impose traffic rules** — one-way lanes, intersection "traffic lights" → cuts head-on conflicts by 80%+
2. **Priority Planning** — assign AGV priorities; earlier-planned trajectories become dynamic obstacles for later ones → **decoupled computation**, fast but sub-optimal
3. **Low-level ORCA / MPC** — handle instantaneous micro-conflicts

**ORCA (Optimal Reciprocal Collision Avoidance)**: $O(1)$ ultra-fast decentralized collision avoidance for drone swarms. Compute half-plane constraints in velocity space; each pair **shares 50/50** of the avoidance burden:

$$
(v_A - (v_A^{\text{opt}} + u/2)) \cdot \mathbf{n} \ge 0
$$

**Physical meaning**: A's new velocity must lie in the half-plane "pushing away from the other" — each side yields half, achieving reciprocal avoidance.

**Freezing Robot Problem**: in dense crowds, treating every person as a dynamic obstacle collapses feasible space → robot freezes. Fix: Social Layer (pedestrian intent prediction + social force model) + MPPI spatiotemporal prediction.

---

### 3D Aerial / Rough-Terrain Planning

**3D curse of dimensionality**: Occupancy Grid memory blows up. Solutions:
- **OctoMap (octree)**: merge free/occupied voxels, $O(\log n)$ queries
- **TSDF (Truncated Signed Distance Field)**: 3D vision reconstruction (KinectFusion), keeps only near-surface distances
- **ESDF (Euclidean Signed Distance Field)** (most important for planning): Euclidean distance to nearest obstacle. **$\nabla \text{ESDF}$ directly points toward fastest obstacle escape** → trajectory optimizers use gradients to push trajectories away from obstacles

**Fast-Planner / EGO-Planner (HKUST Gao Fei)**:
1. A\* or JPS for initial geometric path
2. **B-spline parameterization**
3. NLopt optimizes cost:

$$
J = \lambda_s J_s + \lambda_c J_c + \lambda_d J_d
$$

**Physical meaning**: $J_s$ smoothness (second difference of control points), $J_c$ collision (ESDF pushes away), $J_d$ dynamics (velocity/accel limits).

**EGO-Planner breakthrough**: abandon the global ESDF; only for collision trajectory segments, generate **local penalty gradients** using obstacle surface normals → 10× speedup.

**FASTER dual-trajectory**: during high-speed drone flight, maintain both an "aggressive trajectory" (full free space, fast but risky) and a "conservative backup" (stop-in-place within known safe region). Switch to backup on perception shocks → **Recursive Feasibility guaranteed**.

**Quadruped / Humanoid Rough Terrain**:
- 2.5D **Elevation Map** — analyze normals + roughness
- **CoM (center-of-mass) planner** — avoid large rocks / unstable surfaces
- **Foothold Planner** — find flat, uninterfered footstep locations on either side of the CoM trajectory
- Platforms: Unitree H1, Boston Dynamics Spot, ANYmal

---

### Learning-based Planning (2024 SOTA)

**Core principle** (the mature interview answer): **Learning is not a replacement — it is a proposal / heuristic layer**. NNs hallucinate + lack 100% hard physical constraints → end-to-end motor commands cannot pass industrial safety certification. **SOTA architecture** is Diffuser / VLM generates reference path → MPC / local planner validates collision + dynamically refines.

**Diffuser / Diffusion Planning**: treat planning as **image denoising**. The diffusion model directly outputs a full trajectory $[x_0, x_1, \ldots, x_T]$. Key advantage: **precisely captures multi-modal distribution** (go-left vs go-right). Traditional RL averages the two and **crashes into the wall**.

**NeRF-guided Planning**: NeRF provides high-fidelity continuous 3D implicit reconstruction. The planner computes collision gradients directly in the density field — finer than OctoMap.

**CVAE + RRT\* Seeding**: train a CVAE (Conditional VAE): given start/goal → guess the region where expert trajectories are likely. RRT\* samples from the CVAE distribution → **blind full-space search → precise tunnel along predicted corridor**, 100× speedup.

**Foundation Model Planning (VLM high-level decomposition)**: GPT-4V as "brain": vague semantics ("clear the table") → high-level motion proposals ("navigate to table → grasp cup"); "cerebellum" MPC / RRT\* enforces dynamics + hard collision constraints. Platforms: Tesla Optimus, Figure humanoid.

---

<details>
<summary>Deep dive: Complete A\* algorithm steps and complexity analysis</summary>

**Full A\* pseudocode**:

```
OPEN ← priority queue with start (f = h(start))
CLOSED ← empty set
g[start] = 0

while OPEN is not empty:
    n ← node with lowest f in OPEN
    if n == goal: return reconstruct_path(n)

    OPEN.remove(n)
    CLOSED.add(n)

    for each neighbor m of n:
        if m in CLOSED: continue
        tentative_g = g[n] + cost(n, m)
        if tentative_g < g[m]:     // found a better path
            g[m] = tentative_g
            f[m] = g[m] + h(m)
            parent[m] = n
            if m not in OPEN: OPEN.add(m)
```

**Complexity**:
- Time: $O(b^d)$ where $b$ is branching factor, $d$ is optimal solution depth. A tight $h$ drastically reduces expanded nodes
- Space: $O(b^d)$ (OPEN + CLOSED both stored) — the main bottleneck for A\* on large maps

**Common heuristics** (2D grid):
- **Manhattan distance**: admissible for 4-connected grids, $h = |x_1 - x_2| + |y_1 - y_2|$
- **Chebyshev distance**: 8-connected grids with equal diagonal/cardinal cost, $h = \max(|dx|, |dy|)$
- **Euclidean distance**: always admissible but possibly loose, $h = \sqrt{dx^2 + dy^2}$
- **Diagonal distance**: 8-connected grid with $\sqrt{2}$ diagonal cost: $h = \max(|dx|, |dy|) + (\sqrt{2} - 1) \min(|dx|, |dy|)$

**Weighted A\***: $f = g + w \cdot h$ with $w > 1$ trades optimality for speed; solution quality within $w \times$ optimal ($w$-admissible). **Production speedup 10-100×**. Nav2's NavFn planner is weighted A\*.

**JPS (Jump Point Search)**: speeds up A\* on uniform-cost grids. Core insight: on large open grids, classical A\* stuffs symmetric-equivalent paths into the OPEN list, causing massive redundancy. JPS's **jump-point rule** skips uniform grids lacking forced neighbors, expanding only at "necessary turns" → compresses OPEN list, 10×+ speedup. Limitation: uniform-cost grids only.

**Grid resolution trade-off** (interview pitfall):
- Too fine → curse of dimensionality + jagged paths
- Too coarse → narrow passages misclassified as blocked due to discretization (narrow passage trap)
- **Production fix**: multi-resolution maps (Octree / Quadtree) or hierarchical planning

</details>

<details>
<summary>Deep dive: Theta\* Line-of-Sight update and D\* Lite incremental mechanism</summary>

**Theta\* LoS update**:

```python
def update_vertex(u, s_prime):
    """Theta* node expansion — if grandparent sees s_prime, skip u and connect grandparent"""
    if line_of_sight(u.parent, s_prime):
        # Path 1: set s_prime's parent directly to u's parent (smooth the bend)
        new_cost = u.parent.g + euclidean(u.parent, s_prime)
        if new_cost < s_prime.g:
            s_prime.g = new_cost
            s_prime.parent = u.parent
    else:
        # Path 2: degenerate to standard A* update
        new_cost = u.g + euclidean(u, s_prime)
        if new_cost < s_prime.g:
            s_prime.g = new_cost
            s_prime.parent = u
```

**LoS check implementation**: Bresenham line algorithm sweeps the grid; if any cell is an obstacle, LoS = False.

**D\* Lite core state**:

Each node maintains:
- $g(n)$: current best known cost from $n$ to goal
- $\text{rhs}(n) = \min_{n' \in \text{succ}(n)} (c(n, n') + g(n'))$: one-step lookahead cost via neighbors

**Consistency judgment**:
- $g(n) = \text{rhs}(n)$ → **locally consistent**
- $g(n) > \text{rhs}(n)$ → **overconsistent** (a better path exists — update)
- $g(n) < \text{rhs}(n)$ → **underconsistent** (cost rose — new obstacle detected)

**Incremental update flow**:
1. Sensor detects new obstacle → edge $(u, v)$ cost changes from $c_{\text{old}}$ to $c_{\text{new}}$
2. Update $\text{rhs}(u)$
3. If $\text{rhs}(u) \neq g(u)$, push $u$ back into priority queue
4. Only propagate through affected region; unaffected nodes keep their $g$
5. Continue until all nodes adjacent to start are consistent

**Why orders of magnitude faster than re-running A\***: re-running A\* expands all nodes from scratch $O(V \log V)$; D\* Lite only updates locally affected nodes, **reusing 99% of the search tree**.

</details>

<details>
<summary>Deep dive: RRT\* complete rewiring implementation and asymptotic optimality</summary>

**RRT\* rewiring steps**:

```python
def rrt_star_extend(tree, q_rand, r):
    q_near = nearest(tree, q_rand)
    q_new = steer(q_near, q_rand)
    if not collision_free(q_near, q_new):
        return

    Q_near = near_nodes(tree, q_new, radius=r)

    # Step 1: Pick best parent
    best_parent = q_near
    best_cost = cost(q_near) + dist(q_near, q_new)
    for q in Q_near:
        c = cost(q) + dist(q, q_new)
        if c < best_cost and collision_free(q, q_new):
            best_parent = q
            best_cost = c

    tree.add(q_new, parent=best_parent, cost=best_cost)

    # Step 2: Reverse rewiring — new node may shorten neighbors
    for q in Q_near:
        new_cost = best_cost + dist(q_new, q)
        if new_cost < cost(q) and collision_free(q_new, q):
            q.parent = q_new
            q.cost = new_cost
            propagate_cost_to_children(q)  # recursively update subtree
```

**Search radius theoretical formula**:

$$
r = \gamma \left( \frac{\log n}{n} \right)^{1/d}, \quad \gamma > \left(2 \cdot \frac{1 + 1/d}{\xi_d} \right)^{1/d} \cdot \mu(X_{\text{free}})^{1/d}
$$

$\xi_d$ is the unit ball volume, $\mu(X_{\text{free}})$ is free space volume. Constant $\gamma$ ensures asymptotic optimality.

**Practical variants**:
- **RRT-Connect**: bidirectional growth (trees from both start and goal), **finds paths much faster** but without optimality. MoveIt defaults to RRTConnect because manipulation tasks usually just need "good-enough"
- **Informed RRT\***: after initial solution, shrink sampling to an ellipsoid (see above)
- **BIT\***: combines RRT* and graph-based search
- **RRT\*-Smart**: biased sampling near initial solution + path shortcutting

</details>

<details>
<summary>Deep dive: High-dim collision detection internals (GJK / EPA / FCL / BVH)</summary>

**Why collision checking is the bottleneck in high-dim planning**: it consumes **80-90%** of high-dim planning time. Every steer checks a new edge; a single RRT run on a 7-DoF arm in clutter may require $10^5$+ checks.

**Broad Phase + Narrow Phase pipeline**:

1. **Broad Phase**: rapidly cull impossible pairs
   - **AABB (Axis-Aligned Bounding Box)**: $O(1)$ overlap test
   - **BVH (Bounding Volume Hierarchy)**: tree structure, $O(\log n)$ queries
   - For arms — one bounding box per link, adjacent links can't self-collide (excluded)

2. **Narrow Phase**: exact convex pair checks
   - **GJK (Gilbert-Johnson-Keerthi)**: exploits Minkowski difference $A \ominus B = \{a - b \mid a \in A, b \in B\}$
   - **Core theorem**: $A \cap B \neq \emptyset \iff \mathbf{0} \in A \ominus B$ (intersection iff origin lies in the Minkowski difference)
   - GJK iteratively builds a simplex containing the origin to test this
   - **EPA (Expanding Polytope Algorithm)**: when intersecting, computes penetration depth and normal

**FCL (Flexible Collision Library)**: MoveIt's default library, supports:
- AABB / OBB / RSS bounding volumes
- BVH / Octree spatial partition
- GJK / EPA narrow phase
- Mesh-to-mesh / mesh-to-primitive

**Lazy Collision Checking** (BIT\* core speedup): classical RRT\* checks collision per edge; lazy assumes edges safe, runs A\* to find candidate optimal path, **only checks collision on candidate-path edges**. If clear, confirm; else delete edge and re-search. **Saves 90%+ of collision checks**.

**Custom OMPL StateValidityChecker**:

```cpp
class CustomCollisionChecker : public ompl::base::StateValidityChecker {
public:
    bool isValid(const State* state) const override {
        const auto* q = state->as<RealVectorStateSpace::StateType>();
        // 1) Broad phase first (fast AABB cull)
        if (fast_aabb_check(q) == COLLISION) return false;
        // 2) Narrow phase (exact GJK)
        return exact_gjk_check(q);
    }
};
```

**Learned Sampling breaks narrow passages**: a 7-DoF arm reaching into a cluttered bookshelf faces C-space narrow passages whose volume fraction shrinks exponentially in dimension. Uniform random sampling has near-zero probability of landing there → planning fails. Fix: train a GMM / CVAE to weight samples toward narrow-passage entrances learned from historical successes.

</details>

<details>
<summary>Deep dive: Hybrid A\* full expansion flow + three-point reverse parking</summary>

**Hybrid A\* state**: $(x, y, \theta)$ continuous, internally discretized on a 3D grid (typical 0.5m × 0.5m × 5°) for visited checks. **Key**: nodes are not snapped to grid centers; the same cell can hold multiple $(x, y, \theta)$ states.

**Node expansion**: for each node, simulate steering left/center/right, each integrated for $dt$:

```python
def expand_node(node, max_steer=0.6, dt=0.3, v=1.0):
    children = []
    for steer in [-max_steer, 0.0, +max_steer]:
        # Ackermann kinematics
        dx = v * cos(node.theta) * dt
        dy = v * sin(node.theta) * dt
        dtheta = v / L * tan(steer) * dt  # L = wheelbase
        new_state = (node.x + dx, node.y + dy, node.theta + dtheta)

        # Also generate reverse option
        new_state_reverse = (node.x - dx, node.y - dy, node.theta - dtheta)

        children.extend([new_state, new_state_reverse])
    return children
```

**Dual heuristic — take the max**:
1. **Non-holonomic without obstacles**: Reeds-Shepp distance (respects turning radius, ignores obstacles) — guarantees a kinematically feasible shortest-path estimate
2. **Holonomic with obstacles**: 2D Euclidean A\* distance (respects obstacles, ignores kinematics)

**Use $h = \max(h_1, h_2)$** — both admissible, taking max tightens the estimate.

**Three-point reverse parking emergence**: in a narrow slot:
- Pure forward → hits front wall → enormous cost
- Reverse gear `C|C` curve (gear change + turn) → incurs reverse penalty but passes the narrow entrance → lower total cost
- A* automatically selects the reverse plan → **three-point reverse parking emerges naturally**, no hardcoded rule required

**Analytic Expansion**: periodically try connecting the current node to goal via an analytic Reeds-Shepp solution (if that segment is collision-free, terminate search immediately). Speedup 10-20×.

</details>

<details>
<summary>Deep dive: MPPI full control law and non-smooth cost handling</summary>

**MPPI core idea**: recast optimal control as probabilistic inference — importance-weight random trajectory perturbations.

**Full algorithm per control cycle** (10-50 Hz):

```python
def mppi_control(x_curr, nominal_u, dynamics, cost_fn,
                 K=1000, H=20, sigma=0.5, lambda_=1.0):
    """
    K: number of sampled trajectories (GPU parallel)
    H: horizon
    sigma: control perturbation std
    lambda_: temperature (smaller = greedier)
    """
    # 1) Sample K perturbation sequences δu ~ N(0, Σ)
    delta_u = np.random.randn(K, H, u_dim) * sigma

    # 2) Rollout K trajectories
    costs = np.zeros(K)
    for k in range(K):  # GPU parallel
        x = x_curr.copy()
        for t in range(H):
            u = nominal_u[t] + delta_u[k, t]
            x = dynamics(x, u)
            costs[k] += cost_fn(x, u)  # non-smooth OK!

    # 3) Importance weighting
    beta = costs.min()
    weights = np.exp(-(costs - beta) / lambda_)
    weights /= weights.sum()

    # 4) Weighted update of nominal control
    for t in range(H):
        nominal_u[t] += np.sum(weights[:, None] * delta_u[:, t], axis=0)

    return nominal_u[0]  # execute first step only, receding horizon
```

**Why MPPI handles non-smooth cost gracefully**:
- TEB / G2O rely on Gauss-Newton / Levenberg-Marquardt → **cost must be differentiable**
- Step-function collision cost (1 collide, 0 free) → zero (or infinite) gradient → optimizer blows up
- MPPI only **evaluates** cost (forward simulation + integration), **no derivative needed** → handles any discrete / step / discontinuous cost

**Off-road quadruped / AV application**: slip cost on rough terrain, cliff-edge step penalty, discrete traffic rules (red-light forbidden) — all drop directly into the cost function.

**Nvidia Isaac MPPI**: Robot4Hz, Neural MPPI research — NN as dynamics, learns off-road rollouts directly from point clouds.

</details>

<details>
<summary>Deep dive: Complete Python implementation (A* + RRT* + APF, runnable)</summary>

```python
import numpy as np
import heapq
from typing import Optional

# ============ A* on 2D Grid ============

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_grid(grid: np.ndarray, start: tuple, goal: tuple) -> Optional[list]:
    """A* on 2D occupancy grid. grid[r][c]=1 means obstacle."""
    rows, cols = grid.shape
    open_set = [(manhattan(start, goal), 0, start)]
    came_from = {}
    g = {start: 0}
    closed = set()
    directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while open_set:
        f_val, g_val, cur = heapq.heappop(open_set)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            return path[::-1]
        for dr, dc in directions:
            nr, nc = cur[0]+dr, cur[1]+dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                nb = (nr, nc)
                if nb in closed:
                    continue
                step_cost = 1.414 if abs(dr)+abs(dc)==2 else 1.0
                ng = g_val + step_cost
                if ng < g.get(nb, float('inf')):
                    g[nb] = ng
                    came_from[nb] = cur
                    heapq.heappush(open_set, (ng + manhattan(nb, goal), ng, nb))
    return None

# ============ RRT* with rewiring ============

class Node:
    def __init__(self, pos, parent=None, cost=0.0):
        self.pos = np.array(pos, dtype=float)
        self.parent = parent
        self.cost = cost

def rrt_star(start, goal, obstacles, bounds, step=0.3, max_iter=3000,
             radius=1.0, goal_bias=0.05):
    def collision_free(p1, p2, n=10):
        for t in np.linspace(0, 1, n):
            pt = p1 + t * (p2 - p1)
            for ox, oy, r in obstacles:
                if np.hypot(pt[0]-ox, pt[1]-oy) < r: return False
        return True

    root = Node(start, cost=0.0)
    nodes = [root]

    for _ in range(max_iter):
        sample = np.array(goal) if np.random.rand() < goal_bias else \
                 np.array([np.random.uniform(bounds[0], bounds[1]),
                           np.random.uniform(bounds[2], bounds[3])])
        nearest = min(nodes, key=lambda n: np.linalg.norm(n.pos - sample))
        direction = sample - nearest.pos
        dist = np.linalg.norm(direction)
        if dist < 1e-6: continue
        new_pos = nearest.pos + direction / dist * min(step, dist)
        if not collision_free(nearest.pos, new_pos): continue

        # Step 1: Find best parent in radius
        near_nodes = [n for n in nodes if np.linalg.norm(n.pos - new_pos) < radius]
        best_parent = nearest
        best_cost = nearest.cost + np.linalg.norm(new_pos - nearest.pos)
        for n in near_nodes:
            c = n.cost + np.linalg.norm(new_pos - n.pos)
            if c < best_cost and collision_free(n.pos, new_pos):
                best_parent, best_cost = n, c

        new_node = Node(new_pos, parent=best_parent, cost=best_cost)
        nodes.append(new_node)

        # Step 2: Rewire neighbors through new_node
        for n in near_nodes:
            c = new_node.cost + np.linalg.norm(n.pos - new_pos)
            if c < n.cost and collision_free(new_pos, n.pos):
                n.parent = new_node
                n.cost = c

        if np.linalg.norm(new_pos - np.array(goal)) < step:
            path = []
            n = new_node
            while n is not None:
                path.append(n.pos.tolist())
                n = n.parent
            return path[::-1]
    return None

# ============ APF in 2D ============

def apf_2d(start, goal, obstacles, k_att=1.0, k_rep=100.0, d0=1.0,
           step=0.05, max_iter=1000, tol=0.1):
    pos = np.array(start, dtype=float)
    path = [pos.copy()]

    for _ in range(max_iter):
        if np.linalg.norm(pos - np.array(goal)) < tol:
            return [p.tolist() for p in path]

        f_att = -k_att * (pos - np.array(goal))
        f_rep = np.zeros(2)
        for ox, oy, r in obstacles:
            obs = np.array([ox, oy])
            diff = pos - obs
            dist = np.linalg.norm(diff) - r
            dist = max(dist, 0.01)
            if dist < d0:
                f_rep += k_rep * (1.0/dist - 1.0/d0) * (1.0/dist**2) * (diff / np.linalg.norm(diff))

        f_total = f_att + f_rep
        pos = pos + step * f_total / (np.linalg.norm(f_total) + 1e-6)
        path.append(pos.copy())

    return [p.tolist() for p in path]  # may not have reached goal (local min!)

if __name__ == "__main__":
    grid = np.zeros((20, 20), dtype=int)
    grid[5:15, 10] = 1
    path = a_star_grid(grid, (2, 2), (18, 18))
    print(f"A* path length: {len(path) if path else 'No solution'}")

    obs = [(5, 5, 1.0), (3, 8, 0.8)]
    p_rrt = rrt_star([0, 0], [10, 10], obs, bounds=(0, 12, 0, 12))
    print(f"RRT* path length: {len(p_rrt) if p_rrt else 'No solution'}")

    p_apf = apf_2d([0, 0], [10, 10], obs)
    print(f"APF path length: {len(p_apf)}")
```

</details>

<details>
<summary>Deep dive: Diffuser trajectory generation PyTorch skeleton</summary>

```python
import torch
import torch.nn as nn

class Diffuser(nn.Module):
    """Treat planning as image denoising — directly output a smooth trajectory"""
    def __init__(self, state_dim, horizon, n_diffusion_steps=100):
        super().__init__()
        self.horizon = horizon
        self.state_dim = state_dim
        self.n_steps = n_diffusion_steps
        # 1D U-Net backbone (real impl uses temporal convolutions)
        self.denoiser = TemporalUNet(state_dim)

    def sample(self, cond_start, cond_goal):
        """Start from pure noise, progressively denoise into a trajectory"""
        tau = torch.randn((1, self.horizon, self.state_dim))

        for k in reversed(range(self.n_steps)):
            # Hard-inject conditions (start / goal) — reset endpoints at every step
            tau[:, 0, :] = cond_start
            tau[:, -1, :] = cond_goal

            # Predict noise
            noise_pred = self.denoiser(tau, k)

            # Denoise one step
            tau = remove_noise_step(tau, noise_pred, k)

        return tau  # smooth obstacle-avoiding reference trajectory

def plan_with_diffuser(diffuser, start, goal, mpc_refiner):
    """SOTA hybrid architecture: Diffuser proposes → MPC hard-constraint refines"""
    ref_traj = diffuser.sample(start, goal)                # NN proposal
    safe_traj = mpc_refiner.optimize(ref_traj, obstacles)  # physical constraints
    return safe_traj
```

**Why multi-modal distribution matters**: traditional RL with MSE loss averages "go-left" and "go-right" policies → **crashes into the center wall**. Diffuser's denoising process naturally preserves multi-modal distributions → each sample gives a sensible path.

</details>

## Intuitive Understanding

**Three foundational analogies**:

1. **A\* = GPS navigation**: when you use Google Maps, it computes "expected total time" ($f$) for every possible route and recommends the fastest. $h$ is Maps' estimate of "remaining time" — the better the estimate, the faster the search; but it must never overestimate (or you'd miss the true fastest route)

2. **RRT = explorer throwing stones in the dark**: you're lost in an unknown forest searching for the exit. At each step you toss a stone in a random direction and walk that way to see if there's a path. Walkable → mark; blocked → toss another stone. Eventually you cover the forest — but the path you walked is not the shortest

3. **APF = attraction magnet + repulsion magnet**: the goal is a big magnet pulling you, obstacles are same-pole magnets pushing you. You follow the net force. But if you're wedged between two repulsive magnets exactly balancing the goal's pull — you get stuck (local minimum)

**Advanced analogies**:

4. **Informed RRT\* = time-limited exam**: you spent 60 minutes on the first solve; now you only resample questions that could fit in <60 minutes — anything longer is provably worse
5. **D\* Lite = routing table patch**: an internet router discovers a broken link — it doesn't recompute the entire global routing table, just patches the affected subset. Incremental search lives on this idea
6. **Hybrid A\* = a driver who can't rotate in place**: vanilla A\* can teleport, but a car can't — each expansion can only "steer left / straight / right" and drive a short distance
7. **MPPI = 100,000 monkeys throwing dice**: GPU-parallel 100k random control trajectories, weighted-average by cost → intelligence emerges from statistical aggregation
8. **Diffuser = sculptor carving a Buddha from rough stone**: the initial trajectory is pure noise; each denoising step makes it more "reasonable path" — avoidance and smoothness emerge

**Simulator observations**:

- **Gazebo + Nav2**: launch `nav2_bringup`, set goal in Rviz2 via `2D Goal Pose`. Enable `publish_potential: true` on the NavFn planner to visualize the potential field. Compare DWA vs TEB vs MPPI Controller behaviors in narrow corridors
- **MoveIt + OMPL**: run RRTConnect / RRT\* / BIT\* on a 7-DoF arm with varying clutter density — observe how planning time grows
- **CARLA self-driving sim**: implement a Frenet Lattice Planner — generate multiple lateral-offset quintic polynomials in Frenet coordinates, select by collision/offset/velocity cost. Watch the lattice hug lane curvature automatically
- **Isaac Sim + drone**: run Fast-Planner (B-spline + ESDF); see the optimizer "push" a rough A\* polyline into a smooth obstacle-avoiding trajectory
- **2D Python animation**: matplotlib visualizing A\* node expansion + RRT\* tree growth + rewiring — the fastest way to build intuition

## Implementation Connection

**Six typical production scenarios**:

1. **ROS 2 Nav2 indoor navigation**: mobile robot on a 2D occupancy grid uses A\* (NavFn / Smac Planner) for the global path, then hands off to DWB / TEB / MPPI Controller for dynamic obstacle avoidance. Classic layered architecture — global optimality + local reactivity

2. **MoveIt manipulator pick-and-place**: 7-DoF arm grasps objects on a cluttered table. MoveIt uses OMPL under the hood, defaulting to RRTConnect. For tighter paths switch to BIT\* + Lazy collision

3. **Warehouse multi-robot fleet (Amazon Kiva)**: hundreds of AGVs share a map; Priority Planning + CBS resolves conflicts; ORCA handles instantaneous micro-conflicts

4. **Self-driving Hybrid A\* parking + Frenet Lattice highway**: Waymo / Tesla use Hybrid A\* for narrow-slot three-point reverse parking; highways use Frenet Lattice quintic polynomial primitives

5. **Quadruped / Humanoid rough terrain (Unitree H1)**: Elevation Map + CoM planner + Foothold Planner — avoid rocks and plan feasible footsteps

6. **Drone Fast-Planner high-speed flight (DJI APAS)**: A\* → B-spline trajectory optimization → NLopt pushes trajectories away using ESDF gradients + dynamic constraints

**Code skeleton** (ROS 2 Nav2 Layered Costmap configuration):

```yaml
# nav2_params.yaml — Global Costmap layered config
global_costmap:
  ros__parameters:
    resolution: 0.05
    robot_radius: 0.22
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: "scan"
      scan:
        topic: /scan
        raytrace_max_range: 3.0
        obstacle_max_range: 2.5
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0      # α — inflation decay speed
      inflation_radius: 0.55        # robot radius + safety margin
```

**Code skeleton** (OMPL BIT\* for 7-DoF manipulation):

```cpp
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

auto space = std::make_shared<ob::RealVectorStateSpace>(7);  // 7-DoF
space->setBounds(-M_PI, M_PI);

auto ss = std::make_shared<og::SimpleSetup>(space);
ss->setStateValidityChecker([&](const ob::State* s) {
    // Custom collision check — broad + narrow phase
    return custom_collision_check(s);
});

auto planner = std::make_shared<og::BITstar>(ss->getSpaceInformation());
planner->setSamplesPerBatch(100);
planner->setUseJustInTimeSampling(true);
planner->setPruneThresholdFraction(0.05);

ss->setPlanner(planner);
ss->solve(5.0);  // 5-second planning budget
```

**Code skeleton** (Hybrid A\* node expansion):

```python
def hybrid_a_star_expand(node, dt=0.3, v=1.0, L=2.5):
    """Ackermann kinematics integration — self-driving parking core"""
    children = []
    for steer in [-0.6, 0.0, 0.6]:
        for direction in [+1, -1]:  # forward + reverse
            dx = direction * v * math.cos(node.theta) * dt
            dy = direction * v * math.sin(node.theta) * dt
            dtheta = direction * v / L * math.tan(steer) * dt
            children.append(State(
                node.x + dx, node.y + dy, node.theta + dtheta,
                parent=node,
                cost=node.cost + 1.0 + (0.5 if direction < 0 else 0)
            ))
    return children
```

## Common Misconceptions

1. **"RRT finds the shortest path"** — wrong. RRT is only **probabilistically complete** (it finds *a* path); paths are typically wiggly and far from optimal. For asymptotic optimality use **RRT\*** (with rewiring). When asked "what's the difference between RRT and RRT\*", the answer is rewiring → asymptotic optimality.

2. **"APF can be a global planner"** — dangerous. APF has **local minima**: U-shaped obstacles, dead ends, symmetric configurations trap the robot. Industry uses APF only as a **local reactive layer**; the global path comes from A\* or RRT. Nav2's global/local layered architecture embodies this.

3. **"A\* is universal"** — infeasible in high-dim C-space. A 7-DoF arm has 7D C-space; grid discretization blows up exponentially. That's why MoveIt uses sampling-based methods (RRT / PRM). **Dimensionality dictates algorithm choice** — the most important rule in path planning.

4. **"Path planning and trajectory planning are the same thing"** — no. Path is purely geometric (where to go); trajectory adds time (when to arrive, at what speed). The pipeline is path → trajectory → control, three distinct layers.

5. **"The Global planner's path is always tractable by the Local planner"** — wrong. **Costmap inflation mismatch** is the #1 cause of Local deadlock: Global costmap with small inflation → plans a path hugging the wall; Local considers real robot footprint + non-holonomic constraints → can't squeeze through → oscillation/stuck. Fix: Global costmap introduces a **smooth cost decay gradient**, forcing Global to route through corridor centers.

6. **"Just re-run A\* on every sensor update in dynamic environments"** — disaster. Re-running A\* every second deadlocks and oscillates. Correct answer: **D\* Lite incremental repair** only updates locally affected nodes, reusing 99% of the search tree; or **layered planning** — Global slow (1-5 Hz) + Local fast (20-100 Hz).

7. **"Learning-based planning will fully replace traditional methods"** — industry disagrees. NNs hallucinate + lack hard physics, end-to-end motor commands can't pass safety certification. **SOTA architecture: Diffuser / VLM generates proposal → MPC / local planner validates with hard constraints**. AI provides generalization speed, traditional control provides absolute safety.

## Practice Problems

<details>
<summary>Q1: A warehouse AGV must travel from rack A to rack B in a static 2D environment. What algorithm would you choose, and why?</summary>

**Full reasoning chain**:

1. **Dimensionality check**: AGV moves in 2D, C-space is $(x, y)$ or $(x, y, \theta)$ — at most 3D → grid-based methods are feasible
2. **Scenario check**: static environment, single query, need shortest path → **A\*** is first choice
3. **Heuristic choice**: 4-connected grid → Manhattan; 8-connected → diagonal distance; both admissible
4. **Inflation radius**: inflate obstacles by AGV physical size so the AGV can be treated as a point. Set `inflation_radius = robot_radius + safety_margin`
5. **Pitfall to avoid**: on very large maps (1000×1000), vanilla A\* expands too many nodes. Use **JPS** (Jump Point Search) for 10×+ speedup, or **Weighted A\*** ($w \in [1.2, 2.0]$) trading some optimality for speed
6. **If smoothness required**: use **Theta\*** instead — does Line-of-Sight smoothing during search, eliminating 8-connected zigzag

**Conclusion**: A\* (low-dim + optimality + static), with inflation layer and appropriate heuristic. Large maps → JPS; smooth paths → Theta\*.

</details>

<details>
<summary>Q2: A 7-DoF arm must grasp a cup on a cluttered desk. What planner do you use?</summary>

**Full reasoning chain**:

1. **Dimensionality check**: 7-DoF → 7D C-space → grid explodes ($100^7 = 10^{14}$), **must use sampling-based**
2. **Single vs multi-query**: grasping is usually single-query (goal changes every time) → **RRT family** beats PRM
3. **Optimality need**: arms usually don't need "globally shortest" — just "good-enough collision-free path" → **RRTConnect** (bidirectional RRT, fast) is the MoveIt default
4. **Narrow passages**: in tight scenes, C-space passage volume fraction shrinks exponentially → uniform sampling fails. Switch to **BIT\* + Learned Sampling** (CVAE seeding) — weight samples toward known narrow entrances
5. **Collision detection bottleneck**: 80-90% of high-dim planning time is collision checking. MoveIt uses **FCL**, broad phase (AABB) + narrow phase (GJK/EPA). **Lazy Collision Checking** (BIT\* core) assumes edges safe, only checks candidate optimal path → saves 90%+ of checks
6. **Post-processing**: planned paths usually need **shortcutting + smoothing** — otherwise unnecessary detours

**Conclusion**: RRTConnect (MoveIt/OMPL default) for fast paths; in cluttered narrow scenes switch to BIT\* + Lazy collision + CVAE seeding. Pair with FCL collision detection and path shortcutting. For static scenes with many queries, consider PRM.

</details>

<details>
<summary>Q3: In Nav2, Global A\* finds a path but the Local Planner (TEB/DWA) oscillates in a narrow corridor. How do you diagnose?</summary>

**Full reasoning chain (must-know interview question)**:

1. **Root cause**: 90% of the time it's **Costmap Inflation Mismatch**
   - Global costmap uses small inflation radius → plans a wall-hugging path
   - Local Planner considers real robot footprint + non-holonomic constraints → can't fit through → oscillates

2. **Debug steps**:
   - In Rviz visualize both global and local costmap inflation layers
   - Compare `inflation_radius` — if Global's is smaller than Local's, that's the culprit
   - Check `cost_scaling_factor` — too small means even far-from-obstacle areas have high cost

3. **Fix**: **Introduce a smooth cost-decay gradient in the Global Costmap**
   - Formula: $\text{Cost}(d) = 252 \cdot \exp(-\alpha \cdot (d - r_{\text{inscribed}}))$
   - Near obstacle → cost skyrockets → A\* routed through corridor center
   - Typical values: `cost_scaling_factor: 3.0`, `inflation_radius: 0.55` (for robot radius 0.22m)

4. **If Local Planner is DWA**: DWA is myopic, few valid samples in narrow corridors → **switch to TEB or MPPI**. TEB uses nonlinear optimization integrating obstacles + time, far better in narrow passages; MPPI's parallel sampling is more robust on non-smooth cost surfaces

5. **Check recovery behaviors**: Nav2's `recovery_server` provides `ClearCostmap / Spin / BackUp`. Disabling recovery causes small disturbances to deadlock

**Conclusion**: Global + Cost Decay to force center routing + correct Local Planner (TEB for narrows, MPPI for non-smooth) + ensure recovery behaviors enabled. The dividing question between "read the textbook" and "actually debugged production".

</details>

<details>
<summary>Q4: Design a 100-AGV warehouse scheduling system for multi-robot path planning.</summary>

**Full reasoning chain**:

1. **Problem nature**: MAPF (Multi-Agent Path Finding) is **NP-hard**. $n$ agents share a graph, no vertex/edge conflicts, minimize Makespan or Sum-of-Costs

2. **Theoretically optimal**: **CBS (Conflict-Based Search)**
   - Low-level: each agent runs A\* independently
   - High-level: build conflict tree, branch on conflicts with added constraints
   - Guarantees global optimality + completeness, but the conflict tree **explodes exponentially at 100 agents** → takes 10+ minutes

3. **Production: reduce + compromise**
   - **Traffic rules**: one-way lanes, intersection "lights" → 80%+ fewer head-on conflicts
   - **Priority Planning**: assign AGV priorities; earlier-planned trajectories become dynamic obstacles for later ones → **decoupled compute**, fast but sub-optimal
   - **Low-level ORCA / MPC**: handle instantaneous micro-conflicts, $O(1)$ decentralized

4. **ORCA's core physical intuition**: in velocity space, each pair shares 50/50 of the avoidance burden — $(v_A - (v_A^{\text{opt}} + u/2)) \cdot \mathbf{n} \ge 0$, naturally symmetric and decentralized

5. **Freezing Robot Problem**: dense crowds treated as obstacles collapse feasible space → robot freezes. Fix: Social Layer (social force model + pedestrian intent prediction) + MPPI spatiotemporal prediction

6. **Reference platforms**: Amazon Kiva (Priority Planning), Alibaba Cainiao, self-driving Platooning

**Conclusion**: don't chase theoretical optimum — aim for collision-free sub-optimal within 100ms. **Layered architecture**: Traffic rules (prevention) → Priority Planning (offline decoupling) → ORCA (online micro-adjust).

</details>

<details>
<summary>Q5: Plan a Waymo-level self-driving three-point reverse parking in a narrow slot.</summary>

**Full reasoning chain**:

1. **Why not vanilla A\***: it lets nodes be arbitrarily adjacent, but vehicles have **non-holonomic constraints** — Ackermann kinematics $\dot{x}\sin\theta - \dot{y}\cos\theta = 0$, can't strafe or spin

2. **Core method: Hybrid A\***
   - State: $(x, y, \theta)$ continuous, internally 3D grid discretized for visited check
   - Expansion: simulate steering left/center/right integrated for $dt$ → every branch physically executable
   - **Key**: include reverse option ($v < 0$) with cost penalty (prefer forward)

3. **Dual heuristic, take max**:
   - $h_1$ = Reeds-Shepp distance (respects turning radius, ignores obstacles)
   - $h_2$ = 2D Euclidean A\* distance (respects obstacles, ignores kinematics)
   - $h = \max(h_1, h_2)$ remains admissible

4. **Reverse-parking emergence**:
   - In a narrow slot: pure forward → hits front obstacle → huge cost
   - Reverse `C|C` curve (gear shift + turn) → small reverse penalty but passes narrow entrance → lower total cost
   - A\* auto-selects reverse → **three-point reverse parking naturally emerges**, no hardcoded rule

5. **Analytic Expansion for speedup**: periodically try connecting current node to goal via analytic Reeds-Shepp (if collision-free, terminate). 10-20× speedup

6. **Reeds-Shepp 46 word codes**: e.g. `C|C|C`, `CSC`, where `|` denotes gear change. This is Waymo/Tesla's industrial standard for reverse parking planning

**Conclusion**: Hybrid A\* + Reeds-Shepp heuristic + Analytic Expansion. Reverse parking **emerges from the cost function**, not from hardcoded rules. This question separates "can use A\*" from "understands vehicle kinematics".

</details>

<details>
<summary>Q6: A drone flies high-speed through unknown 3D environments and must instantly avoid obstacles on sensor shocks. Design the architecture.</summary>

**Full reasoning chain**:

1. **3D curse of dimensionality**: Occupancy Grid explodes → use **OctoMap** (merge free/occupied, $O(\log n)$) or **ESDF** (Euclidean distance to nearest obstacle, $\nabla \text{ESDF}$ directly gives obstacle-escape gradient)

2. **Two-layer planning architecture**:
   - **Front-end**: A\* or JPS on OctoMap for a rough geometric path
   - **Back-end**: **B-spline trajectory optimization** (Fast-Planner / EGO-Planner)
     - Cost = $\lambda_s J_s + \lambda_c J_c + \lambda_d J_d$
     - $J_s$ smoothness, $J_c$ collision (ESDF pushes away), $J_d$ dynamics
     - NLopt "pushes" the rough trajectory into a smooth obstacle-avoiding optimum

3. **EGO-Planner breakthrough**: abandon global ESDF; on collision segments only, use obstacle surface normals as local penalty gradients → 10× speedup

4. **High-speed shock handling: FASTER dual-trajectory**
   - Aggressive trajectory: full free space, fast but risky
   - Conservative backup: stop-in-place within known safe region
   - On perception shock, switch to backup → **Recursive Feasibility guaranteed**

5. **Underactuated dynamics**: no brakes — must pitch/roll to generate thrust reversal → 3D planning must **deeply couple differential flatness + dynamic constraints into trajectory optimization**. B-spline control points naturally satisfy differential flatness

6. **Learning-based speedup**: train **CVAE** on expert flight trajectories; RRT\* seeds from the CVAE distribution for 100× speedup. Or **Diffuser** directly generates reference trajectories

**Conclusion**: OctoMap/ESDF environment + A\* front-end + B-spline back-end + FASTER dual-trajectory + optional Learning-based proposal for speedup. Platforms: DJI Mavic APAS, HKUST Fast-Planner.

</details>

## Interview Angles

1. **Dimensionality dictates algorithm** — the most important rule of path planning. Bring up: "My first question picking a planner is always: how many dimensions is C-space? 2D/3D → A\* / Theta\*; 6-DoF+ → sampling-based (RRT / BIT\*), because grid discretization explodes exponentially." **Why this is the key**: shows you understand the **complexity root cause** of algorithm choice, not cookbook selection.

2. **Admissibility + Consistency dual guarantee** — A\* correctness core. Bring up: "Admissibility $h \le h^*$ guarantees optimality (never misses the true best path); Consistency (triangle inequality) guarantees each node pops at most once (no re-expansion)." **Why this is the key**: separates "calls the API" from "understands the algorithm"; also the theoretical basis for Weighted A\* trading optimality for 10-100× speed.

3. **Weighted A\* sub-optimal is production-useful** — bring up: "$f = g + w \cdot h$ with $w > 1$ trades optimality for speed; solution within $w \times$ optimal. Nav2's NavFn is weighted A\*. Large-map planning goes from 500ms to 50ms — industry always buys it." **Why this is the key**: shows **engineering trade-off thinking**, not dogmatic pursuit of theoretical optimum.

4. **Theta\* smooths during search** — bring up: "Classical A\* on an 8-connected grid can't follow 15° shortcuts without zig-zag. Theta\*'s Line-of-Sight check lets parent-child nodes be non-adjacent, doing path smoothing during search." **Why this is the key**: elegant design that eliminates 8-connected zig-zag; interviewers will ask how LoS works (Bresenham).

5. **D\* Lite is the standard answer for dynamic environments** — bring up: "Never re-run A\* in dynamic environments (deadlocks + oscillates). D\* Lite reverse-searches + tracks rhs/g consistency, updating only affected edges and reusing 99% of the search tree — orders of magnitude faster." **Why this is the key**: must-have answer for "dynamic environments" in interviews; shows incremental-search depth.

6. **PRM multi-query vs RRT single-query** — bring up: "PRM builds a roadmap offline for static factory repetition; RRT grows a tree from start into free space for high-dim dynamic settings. RRT's Voronoi bias naturally extends toward open regions — an elegant probabilistic property." **Why this is the key**: shows understanding of sampling-based **design philosophy**, not formula memorization.

7. **Informed RRT\* ellipsoid convergence + BIT\* Lazy Collision** — bring up: "After the first solution, sample only within the ellipsoid $\lVert x - \text{start}\rVert + \lVert x - \text{goal}\rVert \le c_{\text{best}}$ — instantly collapsing the space. BIT\* combines A\* graph search + lazy collision, cutting 60%+ time vs RRT\* for 7-DoF narrow-space manipulation." **Why this is the key**: must-know for manipulation planning interviews; speaking to `samples_per_batch` tuning beats memorizing RRT\*.

8. **Costmap Inflation Mismatch is the main cause of Local deadlock** — bring up: "Global costmap with small inflation → plans a wall-hugging path; Local considers real footprint + non-holonomic → can't fit → oscillation. Fix: Global adds Cost Decay gradient forcing center routing." **Why this is the key**: decisive question separating "read the textbook" from "actually debugged production".

9. **Non-holonomic Ackermann + Hybrid A\*** — bring up: "Ackermann cars $\dot{x}\sin\theta - \dot{y}\cos\theta = 0$ can't strafe. Hybrid A\* uses kinematic integration for expansion + Reeds-Shepp heuristic; **three-point reverse parking emerges from cost**, not hardcoded rules." **Why this is the key**: core differentiator of self-driving planning; separates "can use A\*" from "understands vehicle kinematics".

10. **DWA myopic trap + TEB spatiotemporal prediction + MPPI non-smooth friendly** — bring up: "DWA looks 1-2 seconds ahead and deadlocks in U-shapes; TEB uses G2O nonlinear optimization with $\Delta T$ as a decision variable, robust to pedestrians; MPPI GPU-parallel thousands of trajectories, derivative-free — a killer choice for off-road step-function cost." **Why this is the key**: Local Planner differentiation is the most common Nav2 interview question; scenario selection must be justified.

11. **CBS conflict tree + Priority Planning reduction + ORCA decentralized** — bring up: "CBS is theoretically optimal but explodes at 100 agents; production uses Priority Planning for decoupled compute + ORCA 50/50 avoidance + traffic rules to cut conflicts. Amazon Kiva follows this stack." **Why this is the key**: standard answer moving NP-hard problems from theory to production, shows **pragmatic industry thinking**.

12. **ESDF gradient-guided 3D trajectory optimization + Learning as proposal** — bring up: "Fast-Planner uses A\* for rough path → B-spline parameterization → ESDF gradients push away + dynamic penalties. EGO-Planner abandons global ESDF using only local normals for 10× speedup. **Learning is proposal, not replacement** — Diffuser generates → MPC hard-constraint validates is the SOTA architecture." **Why this is the key**: drone planning hardcore vocabulary + mature answer on AI in production, showing both depth and industrial awareness.

## Extended Reading

- **LaValle, *Planning Algorithms* Ch5-Ch14** (sampling-based + kinodynamic + decision-theoretic) — bible by the RRT/PRM author, free online: <http://lavalle.pl/planning/>
- **Dolgov et al. "Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments" (2010)** — original Hybrid A\* paper, foundation for DARPA Grand Challenge reverse parking
- **Karaman & Frazzoli "Sampling-based algorithms for optimal motion planning" (2011)** — original RRT\* paper, proves asymptotic optimality, theoretical foundation for rewiring
- **Gammell et al. "Informed RRT\*" (2014) + "BIT\*" (2015)** — informed sampling ellipsoid + batch graph search, mandatory for modern sampling-based
- **Koenig & Likhachev "D\* Lite" (2002)** — standard answer for incremental search in dynamic environments
- **Zhou et al. "EGO-Planner" (2021) + "Fast-Planner" (2019)** — HKUST Gao Fei group, SOTA for drone B-spline + ESDF trajectory optimization
- **Janson et al. "Diffuser" (Stanford, 2022)** — landmark work on diffusion-based planning, PyTorch code open-sourced
- **Sharon et al. "CBS" (2015)** — theoretically optimal algorithm for multi-agent path finding
- **Van den Berg et al. "ORCA" (2011)** — decentralized multi-robot collision avoidance, $O(1)$ reactive
- **OMPL official tutorials**: <https://ompl.kavrakilab.org/> — MoveIt's underlying planner library, supports RRT / RRT\* / PRM / BIT\* / AIT\*
- **Nav2 official docs**: <https://navigation.ros.org/> — layered costmap + global/local planner + recovery behavior configuration
- **Apollo Lattice Planner source code**: Baidu open-source self-driving, industrial implementation of Frenet Lattice quintic polynomials
- **MoveIt + OMPL production**: <https://moveit.ros.org/documentation/> — 7-DoF arm sampling-based planning in practice
