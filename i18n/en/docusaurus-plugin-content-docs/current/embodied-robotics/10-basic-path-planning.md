---
title: "Fundamental Path Planning (A*, RRT, APF)"
prerequisites: ["07-forward-kinematics-dh"]
estimated_time: 45
difficulty: 3
tags: ["path-planning", "a-star", "rrt", "apf", "c-space"]
sidebar_position: 10
---

# Fundamental Path Planning (A*, RRT, APF)

## You Will Learn

- Precisely define the difference between path planning, motion planning, and trajectory planning, and explain when each term applies
- Choose the right planner for a given problem: A\* for low-dimensional grid worlds, RRT/PRM for high-dimensional C-space, APF for reactive local control
- Diagnose common planning failures (A\* blowing up in high dimensions, APF stuck in local minima, RRT returning jerky paths) and know the standard fixes

## Core Concepts

### Terminology: Path vs Motion vs Trajectory

| Term | What it produces | Time information? |
|------|-----------------|-------------------|
| **Path planning** | A geometric sequence of waypoints from start to goal | No — pure geometry |
| **Motion planning** | Umbrella term covering path + trajectory + constraints | Varies |
| **Trajectory planning** | A path augmented with velocity, acceleration, and timing | Yes — time-parameterized |

A path planner answers *where* to go; a trajectory planner answers *when* to be there and *how fast*.

### Configuration Space (C-space)

**Precise definition**: C-space is the space of all possible joint configurations $q = [q_1, q_2, \dots, q_n]$ of a robot. Each point in C-space fully specifies the pose of every link. Obstacles in the physical workspace map to forbidden regions ($\mathcal{C}_{\text{obs}}$) in C-space; the robot itself shrinks to a dimensionless point.

**Why it matters**: collision detection reduces to checking whether a point lies inside $\mathcal{C}_{\text{obs}}$, regardless of the robot's actual shape. The price is that C-space dimension equals the robot's DOF — a 7-DOF arm lives in a 7D space, which is why grid-based methods choke on manipulators.

**Location in the Sense --> Plan --> Control Loop**:
- **Input**: start configuration $q_{\text{start}}$, goal configuration $q_{\text{goal}}$, environment map (occupancy grid, point cloud, or mesh)
- **Output**: ordered sequence of waypoints in C-space (or task space for mobile robots)
- **Downstream consumers**: trajectory optimizer (smooths the path and adds timing), local controller (tracks waypoints), collision checker (validates feasibility)
- **Loop node**: **planning front-end** — the "strategic navigator" that decides the global route before the trajectory optimizer and controller take over execution

**One-line version**: "Path planning is the geometric engine that finds a safe corridor through a multi-dimensional maze."

---

### A\* Search

**Precise definition**: A\* is a best-first graph search that expands the node minimizing $f(n) = g(n) + h(n)$, where $g(n)$ is the cost-so-far from start to node $n$, and $h(n)$ is a heuristic estimate of the remaining cost to the goal. If $h$ is **admissible** (never overestimates), A\* is guaranteed to find the optimal path.

$$
f(n) = g(n) + h(n)
$$

**Physical meaning**: $g$ is how far you have actually traveled; $h$ is your best guess of what remains. Together they let A\* focus expansion toward the goal instead of flooding outward uniformly.

**Dijkstra as a special case**: set $h(n) = 0$ and A\* degenerates into Dijkstra's algorithm — guaranteed optimal, but expands in all directions without directional guidance.

**Common heuristics** (2D grid):

| Heuristic | Formula | Movement model |
|-----------|---------|---------------|
| Manhattan | $\|x_1 - x_2\| + \|y_1 - y_2\|$ | 4-connected grid |
| Euclidean | $\sqrt{(x_1-x_2)^2 + (y_1-y_2)^2}$ | Any-angle movement |
| Octile | $\max(\Delta x, \Delta y) + (\sqrt{2}-1)\min(\Delta x, \Delta y)$ | 8-connected grid |

**Limitation**: the number of nodes grows exponentially with dimension. A 2D 1000x1000 grid has $10^6$ cells — manageable. A 7-DOF arm discretized at 100 bins per joint has $10^{14}$ cells — impossible.

<details>
<summary>Deep dive: A* optimality proof sketch and weighted A*</summary>

**Optimality guarantee**: when A\* terminates by popping the goal from the open list, the path cost equals $g(\text{goal})$. Because $h$ is admissible, every unexpanded node $n$ satisfies $f(n) \geq f^*$ (the true optimal cost). Therefore any path through an unexpanded node cannot be cheaper than the one already found.

**Weighted A\***: replace $f(n) = g(n) + \varepsilon \cdot h(n)$ with $\varepsilon > 1$. This inflates the heuristic, making the search greedier. The resulting path cost is at most $\varepsilon$ times optimal, but expansion count drops dramatically. Nav2's `NavfnPlanner` exposes a `tolerance` parameter that effectively controls this tradeoff.

**Jump Point Search (JPS)**: exploits grid symmetry to skip nodes that would be expanded identically. For uniform-cost grids, JPS achieves order-of-magnitude speedups over vanilla A\* without sacrificing optimality. Only applies to uniform grids — not to weighted or continuous spaces.

</details>

---

### Rapidly-exploring Random Tree (RRT)

**Precise definition**: RRT incrementally builds a tree rooted at $q_{\text{start}}$ by repeating: (1) sample a random configuration $q_{\text{rand}}$, (2) find the nearest node $q_{\text{near}}$ in the tree, (3) steer from $q_{\text{near}}$ toward $q_{\text{rand}}$ by step size $\delta$ to get $q_{\text{new}}$, (4) if the edge is collision-free, add $q_{\text{new}}$ to the tree. Terminate when a node enters the goal region.

$$
q_{\text{new}} = q_{\text{near}} + \delta \cdot \frac{q_{\text{rand}} - q_{\text{near}}}{\|q_{\text{rand}} - q_{\text{near}}\|}
$$

**Physical meaning**: steer one step-length from the nearest tree node toward a random sample, then check for collisions along that edge.

**Key properties**:
- **Probabilistically complete**: given infinite time, the probability of finding a path (if one exists) approaches 1
- **Not optimal**: the first path found is typically jagged and suboptimal
- **Scales to high dimensions**: no grid discretization required; the tree explores C-space by sampling

**RRT\***: adds two operations after finding $q_{\text{new}}$: (1) **rewiring** — check nearby nodes to see if routing through $q_{\text{new}}$ reduces their cost, and (2) **parent reselection** — choose the lowest-cost parent for $q_{\text{new}}$ from its neighborhood. This makes RRT\* **asymptotically optimal** — the path cost converges to the true optimum as the sample count grows.

**RRT-Connect**: grows two trees simultaneously (one from start, one from goal) and attempts to connect them at each iteration. Much faster in practice for narrow-passage problems.

<details>
<summary>Deep dive: RRT* rewiring mechanics and Informed RRT*</summary>

**Rewiring radius**: the neighborhood radius $r$ must shrink as the number of nodes $n$ grows. The standard formula is:

$$
r = \min\left(\gamma \left(\frac{\log n}{n}\right)^{1/d}, \eta\right)
$$

where $d$ is the C-space dimension, $\gamma$ is a constant depending on the free-space volume, and $\eta$ is the step size. This ensures asymptotic optimality while keeping the number of neighbors manageable.

**Informed RRT\***: once a first solution of cost $c_{\text{best}}$ is found, restrict future sampling to the prolate hyperspheroid (ellipse in 2D) defined by foci at $q_{\text{start}}$ and $q_{\text{goal}}$ with major axis $c_{\text{best}}$. This dramatically accelerates convergence because samples outside the ellipse cannot improve the current solution.

**Practical tuning**:
- Step size $\delta$: too small = slow exploration; too large = misses narrow passages. Start at 5% of workspace diagonal.
- Goal bias: sample $q_{\text{goal}}$ directly with probability 5--10% to speed convergence without losing exploration.
- Collision checker dominates runtime: use bounding-box pre-checks and spatial hashing (e.g., FCL, HPP-FCL) to accelerate.

</details>

---

### Artificial Potential Field (APF)

**Precise definition**: APF constructs a scalar potential function $U(q) = U_{\text{att}}(q) + U_{\text{rep}}(q)$. The attractive potential pulls the robot toward the goal; the repulsive potential pushes it away from obstacles. The robot moves by gradient descent: $\dot{q} = -\nabla U(q)$.

$$
U_{\text{att}}(q) = \frac{1}{2} k_a \|q - q_{\text{goal}}\|^2
$$

**Physical meaning**: a quadratic well centered at the goal — the farther away, the stronger the pull, like a spring.

$$
U_{\text{rep}}(q) = \begin{cases} \frac{1}{2} k_r \left(\frac{1}{\rho(q)} - \frac{1}{\rho_0}\right)^2 & \text{if } \rho(q) \leq \rho_0 \\ 0 & \text{if } \rho(q) > \rho_0 \end{cases}
$$

**Physical meaning**: $\rho(q)$ is the distance to the nearest obstacle; $\rho_0$ is the influence radius. Inside $\rho_0$, a repulsive "force field" intensifies as the robot gets closer, like same-pole magnets repelling.

**Critical flaw — local minima**: when repulsive and attractive gradients cancel exactly, the robot gets stuck. Classic example: a U-shaped obstacle between the robot and the goal. The robot enters the cavity, the repulsive walls push inward, and the attractive force pulls forward, creating equilibrium inside the trap.

**Standard escapes**: random walk perturbation, virtual waypoints along the obstacle boundary, combining APF with a global planner (A\* or RRT provides waypoints, APF handles local reactive control).

<details>
<summary>Deep dive: Navigation Functions and guaranteed convergence</summary>

**Navigation Functions** (Rimon & Koditschek, 1992) are a class of potential functions with a single global minimum at the goal and no other local minima. They guarantee convergence from almost all starting configurations in sphere-world domains.

The key idea: construct $\varphi(q) = \frac{\|q - q_{\text{goal}}\|^{2k}}{\left(\|q - q_{\text{goal}}\|^{2k} + \beta(q)\right)^{1/k}}$, where $\beta(q)$ encodes obstacle boundaries and $k$ is a tuning parameter. As $k \to \infty$, $\varphi$ becomes a Morse function with a single minimum.

In practice, navigation functions are hard to construct for complex environments, which is why the industry typically pairs APF with a global planner rather than relying on potential fields alone.

</details>

---

### Probabilistic Roadmap (PRM)

**Precise definition**: PRM is a multi-query sampling-based planner. In the **offline phase**, it scatters $N$ random configurations in $\mathcal{C}_{\text{free}}$, connects nearby collision-free pairs with edges, and stores the resulting graph. In the **online phase**, it connects the query start and goal to the roadmap and runs a graph search (typically A\* or Dijkstra).

**When to use**: static environments with many planning queries (e.g., the same robot arm picks objects from different bins throughout a shift). Building the roadmap is expensive, but each query is fast.

**Comparison with RRT**: RRT builds a new tree per query (single-query); PRM amortizes construction across queries (multi-query). For one-shot problems, RRT is typically more efficient.

---

### Algorithm Selection Guide

| Scenario | Recommended | Why |
|----------|-------------|-----|
| 2D mobile robot, static grid map | **A\*** | Low-dimensional, guarantees optimality |
| 7-DOF arm in cluttered workspace | **RRT\* / PRM** | Sampling-based scales to high dimensions |
| Real-time reactive obstacle avoidance | **APF + DWA** | Fast gradient computation, no global search |
| Repeated queries, same environment | **PRM** | Amortized roadmap construction |
| Narrow passages, high DOF | **RRT-Connect** | Bidirectional growth excels in tight spaces |

## Intuition

**A\* = GPS navigation**: you have a complete street map, and the GPS systematically explores the most promising routes toward your destination. It will always find the shortest path, but it needs the entire map loaded in memory — and if the map were 7-dimensional, your GPS would melt.

**RRT = explorer with a compass**: no map, just a compass pointing toward the goal. The explorer randomly picks directions, walks a fixed distance, checks for walls, and plants a flag. Over time, the flags form a tree that reaches the goal. The first route found is rarely the shortest, but the explorer works even in a pitch-dark labyrinth with no prior map.

**APF = magnets**: the goal is a strong magnet attracting you; each obstacle is a same-pole magnet repelling you. You slide along the net force. Simple and fast, but you can get trapped in a magnetic "dead zone" between obstacles.

**Simulator observation**: in MuJoCo or Gazebo, set up a 2D costmap with a U-shaped obstacle. Run A\* — it finds the shortest path around the opening. Run APF — watch the robot slide into the cavity and freeze. Add a random perturbation escape — it eventually wiggles out but takes erratic detours. This viscerally demonstrates why global planners and local planners serve different roles.

## Implementation Link

**Three representative engineering scenarios**:

1. **Nav2 global planner (2D mobile robot)**: the `NavfnPlanner` plugin in ROS 2 Navigation2 runs a variant of Dijkstra/A\* on the 2D costmap. Configuration parameters include `tolerance` (goal proximity threshold) and `use_astar` (toggle A\* heuristic on/off). The costmap's `inflation_radius` effectively enlarges obstacles in C-space.

2. **MoveIt motion planning for a 7-DOF arm**: MoveIt wraps OMPL (Open Motion Planning Library), which provides RRT, RRT\*, RRT-Connect, PRM, and dozens of other sampling-based planners. The user specifies the goal in task space; MoveIt's planning pipeline converts it to a C-space query, runs the sampler, then post-processes with trajectory optimization (TOTG or time-optimal parameterization).

3. **Layered planning architecture**: production robots typically run a **global planner** (A\* on the costmap, refreshed at 1--5 Hz) to set waypoints, and a **local planner** (DWA, TEB, or APF-inspired) at 10--20 Hz for reactive obstacle avoidance. The local planner never replaces the global one — it only handles transient perturbations within its planning horizon.

**Code skeleton** (Python, A\* on a 2D grid):

```python
import heapq
from typing import List, Tuple

def a_star(grid: List[List[int]], start: Tuple[int,int], goal: Tuple[int,int]) -> List[Tuple[int,int]]:
    """A* on a 2D occupancy grid. grid[r][c]==1 means obstacle."""
    rows, cols = len(grid), len(grid[0])
    open_set = [(0, start)]          # (f, node)
    g_score = {start: 0}
    came_from = {}

    def heuristic(a, b):             # Euclidean (admissible)
        return ((a[0]-b[0])**2 + (a[1]-b[1])**2) ** 0.5

    while open_set:
        f, current = heapq.heappop(open_set)
        if current == goal:
            # Reconstruct path
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        for dr, dc in [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]:
            nr, nc = current[0]+dr, current[1]+dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                move_cost = 1.414 if dr != 0 and dc != 0 else 1.0
                tentative_g = g_score[current] + move_cost
                if tentative_g < g_score.get((nr,nc), float('inf')):
                    g_score[(nr,nc)] = tentative_g
                    came_from[(nr,nc)] = current
                    heapq.heappush(open_set, (tentative_g + heuristic((nr,nc), goal), (nr,nc)))

    return []  # No path found
```

**Code skeleton** (Python, basic RRT):

```python
import numpy as np
from typing import Optional

class RRTNode:
    def __init__(self, q: np.ndarray, parent: Optional['RRTNode'] = None):
        self.q = q
        self.parent = parent

def rrt(q_start: np.ndarray, q_goal: np.ndarray,
        is_collision_free: callable, bounds: np.ndarray,
        step_size: float = 0.5, max_iter: int = 5000,
        goal_threshold: float = 0.3, goal_bias: float = 0.05) -> list:
    """Basic RRT in arbitrary-dimensional C-space."""
    tree = [RRTNode(q_start)]

    for _ in range(max_iter):
        # Sample (with goal bias)
        if np.random.rand() < goal_bias:
            q_rand = q_goal
        else:
            q_rand = np.random.uniform(bounds[:, 0], bounds[:, 1])

        # Nearest neighbor
        dists = [np.linalg.norm(node.q - q_rand) for node in tree]
        nearest = tree[np.argmin(dists)]

        # Steer
        direction = q_rand - nearest.q
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            continue
        q_new = nearest.q + step_size * direction / dist

        # Collision check
        if is_collision_free(nearest.q, q_new):
            new_node = RRTNode(q_new, parent=nearest)
            tree.append(new_node)
            if np.linalg.norm(q_new - q_goal) < goal_threshold:
                # Trace back
                path = []
                node = new_node
                while node is not None:
                    path.append(node.q)
                    node = node.parent
                return path[::-1]

    return []  # Failed to find path
```

<details>
<summary>Deep dive: complete OMPL + MoveIt integration walkthrough</summary>

In a typical MoveIt-based pipeline, you never call RRT directly. Instead:

```python
# Python — MoveIt 2 via moveit_py
from moveit.planning import MoveItPy

moveit = MoveItPy(node_name="planner_node")
arm = moveit.get_planning_component("manipulator")

# Set start and goal
arm.set_start_state_to_current_state()
arm.set_goal_state(configuration_name="pick_ready")

# Plan — OMPL runs RRTConnect under the hood
plan_result = arm.plan()
if plan_result:
    trajectory = plan_result.trajectory
    # trajectory is a RobotTrajectory; post-process with TOTG
    arm.execute(trajectory)
```

**Configuration** lives in `ompl_planning.yaml`:

```yaml
manipulator:
  planner_configs:
    - RRTConnectkConfigDefault
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.05              # Step size in C-space (radians)
```

**Key parameters to tune**:
- `range` (step size): smaller values improve narrow-passage success but slow down planning
- `planning_time`: default 5 s; increase for complex scenes
- `goal_tolerance`: position (m) and orientation (rad) tolerance at the goal
- `num_planning_attempts`: run multiple seeds and keep the best

**Collision checking** is the bottleneck. MoveIt uses FCL (Flexible Collision Library) by default. For performance-critical applications, switch to `hpp-fcl` or use GPU-accelerated collision checking (e.g., cuRobo).

</details>

## Common Misconceptions

1. **"RRT finds the shortest path"** — vanilla RRT is probabilistically complete but **not** optimal. The first path it returns is typically 2--5x longer than the true shortest path, full of jagged turns. You need **RRT\*** (with rewiring) for asymptotic optimality, or post-process with a shortcutting/smoothing step. In practice, MoveIt runs a path simplifier after every OMPL plan.

2. **"APF is a complete planner"** — APF has no completeness guarantee. It will get stuck in local minima of the potential function (classic case: U-shaped obstacle). Never use APF as your sole planner for global path planning. The standard architecture is: global planner (A\* or RRT) provides waypoints, local planner (APF or DWA) handles reactive avoidance between waypoints.

3. **"A\* works for any robot"** — A\* requires discretizing C-space into a graph. For a 2D mobile robot on a grid, this is natural and efficient. For a 6--7 DOF manipulator, the grid has $O(k^n)$ cells (where $k$ is bins per joint and $n$ is DOF), making A\* computationally infeasible. The curse of dimensionality is **the** reason sampling-based methods (RRT, PRM) were invented.

4. **"Path = trajectory"** — a path is a pure geometric sequence with no timing. Sending raw path waypoints directly to joint controllers causes jerky, unpredictable motion because there are no velocity or acceleration constraints. The path must go through a **trajectory optimizer** (time-optimal parameterization, TOPP-RA, or spline fitting) before execution.

## Situational Questions

<details>
<summary>Q1 (easy): A warehouse AGV needs to navigate from point A to point B in a static environment with fixed shelving. Which planner do you choose and why?</summary>

**Complete reasoning chain**:

1. **Characterize the problem**: 2D, static known environment (shelving layout is pre-mapped), single-query (or repeated queries on the same map), and you want the shortest path to minimize travel time and energy.
2. **Dimension check**: the AGV moves in $(x, y, \theta)$ — effectively 2D on an occupancy grid if you treat orientation separately or use a lattice planner. This is well within A\*'s comfort zone.
3. **Choose A\***: it guarantees optimality (shortest path) given an admissible heuristic, and a 2D grid with ~$10^6$ cells is trivially solvable on modern hardware in milliseconds.
4. **Heuristic selection**: Euclidean distance is admissible and consistent for 8-connected grids. If the AGV can only move in 4 directions, use Manhattan distance.
5. **Engineering details**: use Nav2's `NavfnPlanner` or `SmacPlanner2D`. Tune `inflation_radius` to keep the AGV a safe distance from shelves. Add a local planner (DWA) to handle dynamic obstacles (humans walking through the warehouse).

**What the interviewer wants to hear**: dimension awareness ("it's 2D, so grid search is fine"), optimality reasoning ("A\* guarantees shortest path with admissible h"), and practical deployment knowledge (Nav2 stack, inflation radius, layered planning).

</details>

<details>
<summary>Q2 (medium): A 7-DOF manipulator needs to reach into a cluttered shelf to grasp an object. A* is infeasible. What do you use and how do you configure it?</summary>

**Complete reasoning chain**:

1. **Dimension check**: 7-DOF means C-space is 7-dimensional. At 100 bins per joint, A\* would need $10^{14}$ nodes — completely infeasible.
2. **Choose sampling-based planner**: RRT-Connect is the default in MoveIt/OMPL for single-query manipulation tasks. Bidirectional growth is especially effective for narrow passages (reaching between objects on a shelf).
3. **Configuration**:
   - Step size (`range`): start at 0.05 rad; reduce to 0.02 if the shelf is tight
   - Planning time: 5 s default, increase to 10 s for very cluttered scenes
   - Goal bias: 5% (OMPL default) — too high makes the planner greedy and misses narrow passages
4. **Post-processing**: the raw RRT path is jagged. MoveIt's `PathSimplifier` shortcutting + B-spline smoothing + TOTG time parameterization produces a smooth, executable trajectory.
5. **If the environment is static and queries repeat**: consider PRM. Build the roadmap once (offline), then each pick query is a fast graph search.

**What the interviewer wants to hear**: immediate recognition that high-DOF invalidates grid-based methods, familiarity with OMPL/MoveIt configuration, awareness that raw RRT paths need post-processing, and the PRM option for multi-query scenarios.

</details>

<details>
<summary>Q3 (medium-hard): Your Nav2 global planner consistently produces paths that hug walls or take wide detours. How do you diagnose and fix this?</summary>

**Complete reasoning chain**:

1. **Visualize the costmap**: in Rviz2, display the global costmap with the `costmap` plugin. Look at the inflation layer — if `inflation_radius` is too large, the inflated obstacles merge, blocking shortcuts and forcing detours.
2. **Check the heuristic weight**: `NavfnPlanner` uses Dijkstra by default (`use_astar: false`). Switching to A\* with `use_astar: true` focuses expansion toward the goal, which can reduce unnecessary exploration that leads to wall-hugging.
3. **Tune inflation parameters**:
   - `inflation_radius`: reduce from default (e.g., 0.55 m to 0.35 m) if the robot's footprint allows
   - `cost_scaling_factor`: higher values make the cost decay steeper away from obstacles, making the planner less afraid of passing near walls
4. **Consider `SmacPlanner2D`**: it uses A\* with a more configurable cost function and supports SE2 (including heading), which avoids awkward turns near walls.
5. **Verify resolution**: if the costmap resolution is too coarse (e.g., 0.1 m), paths snap to grid cells and appear jagged. Reduce to 0.05 m if compute allows.

**What the interviewer wants to hear**: systematic diagnosis starting from costmap visualization, understanding the relationship between inflation parameters and path quality, and knowledge of alternative Nav2 planners.

</details>

<details>
<summary>Q4 (hard): You are designing a planning pipeline for a mobile manipulator that must navigate through a warehouse, approach a shelf, and pick an object. How do you architect the full system?</summary>

**Complete reasoning chain**:

1. **Decompose the problem**: the task has two phases — (a) mobile base navigation to the shelf vicinity, and (b) arm motion planning to reach the object. These operate in different C-spaces (2D for the base, 7D for the arm) and should use different planners.
2. **Base navigation**:
   - Global planner: A\* on the 2D costmap (Nav2 `SmacPlanner2D` or `NavfnPlanner`)
   - Local planner: DWA or TEB at 20 Hz for reactive avoidance of dynamic obstacles (humans)
   - Recovery behaviors: rotate-in-place, back-up, clear costmap
3. **Pre-grasp positioning**: before planning the arm, the base must be positioned so the object is within the arm's reachable workspace. Use inverse reachability maps (precomputed offline) to choose the optimal base pose relative to the shelf.
4. **Arm planning**:
   - Planner: RRT-Connect via MoveIt/OMPL
   - Scene representation: add the shelf and neighboring objects as collision objects in the MoveIt planning scene (from depth camera point cloud)
   - Post-processing: path simplification + TOTG time parameterization
5. **Coordination**: a behavior tree (BT) orchestrates the sequence: navigate → position base → perceive shelf → plan arm motion → execute grasp → retract → navigate to drop-off. Each node has preconditions and failure recovery.
6. **Whole-body planning** (advanced): instead of sequencing base and arm, plan them jointly in an 10D C-space (3 base DOF + 7 arm DOF). This produces more efficient motions but requires specialized planners like `SmacPlannerHybrid` or OMPL's constrained planning.

**What the interviewer wants to hear**: clear decomposition of base vs arm planning, awareness that different C-space dimensions call for different algorithms, knowledge of inverse reachability maps for base placement, and the coordination layer (behavior trees).

</details>

## Interview Angles

1. **"Dimensionality dictates the algorithm"** — this is the single most important insight in path planning. In an interview, lead with: "The first thing I check is the C-space dimension. 2D or 3D: grid search with A\*. 6+ DOF: sampling-based methods like RRT\* or PRM. This is not a preference — it is a hard computational constraint. A\* in 7D is physically impossible with current hardware."

2. **"Heuristic engineering is where theory meets practice"** — shows you have tuned real planners, not just studied pseudocode. Bring out with: "An admissible heuristic guarantees optimality, but a *tight* heuristic is what makes A\* fast. In Nav2 I tune the cost scaling factor and inflation radius because they directly shape the effective heuristic landscape — a bad configuration makes A\* degenerate into Dijkstra."

3. **"Layered planning architecture"** — demonstrates systems-level thinking. Bring out with: "Production robots never use a single planner. The global planner (A\* at 1--5 Hz) sets strategic waypoints; the local planner (DWA or TEB at 10--20 Hz) handles reactive avoidance. This separation of concerns mirrors the strategic-vs-tactical split in military navigation and is the backbone of Nav2, move_base, and every serious mobile robot stack."

4. **"Sampling-based planners need post-processing"** — separates engineers who have shipped code from those who have only read papers. Bring out with: "Raw RRT output is not executable — it is a jagged sequence of random samples. Before sending it to the controller, I always run shortcutting, B-spline smoothing, and time-optimal parameterization. MoveIt does this automatically, but understanding the pipeline matters when debugging jerky motions."

5. **"APF is a local method, not a planner"** — a surprisingly common misconception. Bring out with: "I never use APF as a standalone global planner because it has no completeness guarantee — local minima will trap the robot. I use it as a reactive layer beneath a global planner, or for simple scenarios like drone obstacle avoidance where the geometry is convex."

## Further Reading

- **LaValle, *Planning Algorithms* (2006), Chapters 5--6** — the definitive reference on sampling-based planning (RRT, PRM), available free online at planning.cs.uiuc.edu. Worth reading for the C-space formalism alone.
- **Hart, Nilsson & Raphael, "A Formal Basis for the Heuristic Determination of Minimum Cost Paths" (1968)** — the original A\* paper. Short, elegant, and surprisingly readable. Grounds your understanding of admissibility and optimality.
- **Karaman & Frazzoli, "Sampling-based algorithms for optimal motion planning" (IJRR, 2011)** — introduces RRT\* and PRM\* with formal asymptotic optimality proofs. The paper that made "rewiring" standard practice.
- **Khatib, "Real-time obstacle avoidance for manipulators and mobile robots" (IJRR, 1986)** — the seminal APF paper. Read it to understand the original formulation and its limitations.
- **Nav2 documentation (navigation.ros.org)** — practical configuration guides for `NavfnPlanner`, `SmacPlanner2D`, `SmacPlannerHybrid`, `ThetaStarPlanner`, and the DWB/TEB local planners. The fastest path from theory to a working robot.
- **OMPL (ompl.kavrakilab.org)** — the Open Motion Planning Library used by MoveIt. Includes benchmarking tools, planner zoo, and tutorials for custom state spaces. Essential for anyone working with manipulators.
- **Gammell et al., "Informed RRT\*: Optimal Sampling-based Path Planning via Direct Sampling of an Admissible Ellipsoidal Heuristic" (IROS, 2014)** — elegant improvement that dramatically speeds up RRT\* convergence by restricting sampling to an informed region.
