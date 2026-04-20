---
title: "Trajectory Optimization"
prerequisites: ["10-basic-path-planning"]
estimated_time: 75
difficulty: 4
tags: ["trajectory", "optimization", "smoothness", "b-spline", "topp", "min-snap", "trajopt", "ilqr", "contact-implicit"]
sidebar_position: 11
---

# Trajectory Optimization (Time, Energy, Smoothness)

## You Will Learn

- State in two sentences the difference between a **path** and a **trajectory**, and never confuse them in an interview
- When facing "the arm carries parts too slowly" or "the welding tip vibrates", you immediately know which objective function to pick (minimum time / minimum energy / minimum jerk), which parameterization to use (polynomial / B-spline / S-curve), and why you can never push all three to their extremes simultaneously
- Decide when you need TOPP-RA for time-optimal reparameterization vs minimum-snap for smooth trajectories
- **(A-level extension)** In a 2-minute interview window, explain why Differential Flatness is the post-2011 soul of quadrotor planning, the essential differences between the CHOMP/STOMP/TrajOpt trio, the iLQR/DDP/Shooting/Collocation decision tree, and why Atlas backflips can only be solved with Direct Collocation + Contact-Implicit

## Core Concepts

**Precise definition**: **Path** is pure geometry (an ordered list of waypoints or a continuous curve), with no time information. **Trajectory** is a path plus a time parameterization $q(t)$, producing position $q(t)$, velocity $\dot{q}(t)$, and acceleration $\ddot{q}(t)$ at every instant, while satisfying physical constraints (velocity limits, torque limits, jerk limits). **Trajectory optimization** is the problem of finding, given a path or waypoints, the best time allocation and curve shape that minimizes some objective.

**One-line version**: "A path is the route drawn on the map; a trajectory is the complete driving plan that decides when to hit the gas, when to brake, and never makes passengers carsick."

**Location in the sense → plan → control loop**:
- **Node**: the bridge from **planning → control**
- **Input**: waypoints from a path planner + robot dynamics limits ($\dot{q}_{\max}$, $\ddot{q}_{\max}$, $\tau_{\max}$)
- **Output**: time-parameterized $q(t), \dot{q}(t), \ddot{q}(t)$ (joint space) or $x(t), \dot{x}(t), \ddot{x}(t)$ (Cartesian space)
- **Downstream**: the controller's feedforward term (PD + feedforward needs $\ddot{q}_{\text{desired}}$), MPC's reference trajectory, inverse-dynamics torque feedforward $\tau_{\text{ff}} = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q)$

**The three canonical objectives**:

| Objective | Math form | Physical meaning | Typical use |
|------|---------|---------|---------|
| Minimum time | $\min \int_0^T dt = \min\, T$ | Finish the motion in the shortest wall-clock time | High-throughput pick-and-place |
| Minimum energy | $\min \int_0^T \|\tau\|^2 dt$ | Minimize integral of squared torque, reduce motor heating | Long continuous operation |
| Minimum smoothness | $\min \int_0^T \|\dddot{q}\|^2 dt$ (min jerk) | Minimize jerk, suppress vibration and high-frequency excitation | Precision machining, welding |

**Polynomial order decision table** (A-level core):

| Order | Continuity | When to use | Fatal flaw |
|------|-------|---------|---------|
| Linear (1st) | $C^0$ | Never use | Acceleration is a Dirac delta → infinite instantaneous torque blows the current loop |
| Cubic (3rd) | $C^1$ (velocity continuous) | Teaching demo | A single segment cannot specify acceleration at both endpoints → when multiple cubic segments are joined, acceleration is discontinuous at junctions → jerk diverges → micro-jitter |
| **Quintic (5th)** | **$C^2$ (acceleration continuous)** | **Industrial PTP gold standard** | No major flaw |
| Septic (7th) | $C^3$ (jerk continuous) | CNC laser cutting, high-precision machining | More parameters, heavier compute |
| LSPB (trapezoidal + parabolic blend) | $C^1$ | Low-compute PLCs, simple CNC moves | Guarantees only velocity continuity |
| High-order global (9+) | $C^{k}$ | **Never use** | **Runge phenomenon**: fit 10 via-points with a 9th-order poly → violent edge oscillation → the robot thrashes |

**The Runge phenomenon trap** (interview must-answer): never force 10 via-points through a single 9th-order polynomial. High-order polynomials oscillate violently near the endpoints, and the robot will thrash. **Correct answer**: use a B-spline of piecewise low-order (typically 5th) polynomials and enforce $C^2$ continuity at the junctions.

**Minimum sufficient math**:

1. **Cubic polynomial interpolation** (satisfies endpoint position + velocity boundary conditions):

$$
q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3
$$

**Physical meaning**: 4 coefficients exactly solve 4 boundary conditions (start position/velocity, end position/velocity); the simplest smooth trajectory. Within a single segment, acceleration $\ddot{q} = 2a_2 + 6a_3 t$ is a continuous linear function, but its start/end values cannot be specified — so **when multiple cubic segments are joined, acceleration is discontinuous at junctions → jerk has a step → micro-jitter**.

2. **Quintic polynomial** (adds acceleration boundary conditions):

$$
q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5
$$

**Physical meaning**: 6 coefficients solve 6 boundary conditions (position, velocity, acceleration at both endpoints). Acceleration continuous → torque continuous → mechanical shock is minimized. **Key result**: the analytic solution of jerk-minimization is equivalent to a quintic, which is why the industrial PTP gold standard is 5th-order, not 3rd.

3. **Minimum Snap for quadrotors** (Mellinger-Kumar 2011, the cornerstone of drone planning):

$$
\min_{c} \int_0^T \left\| p^{(4)}(t) \right\|^2 dt = \min_{c} \; c^\top Q c \quad \text{s.t.} \quad A_{\text{eq}} c = b_{\text{eq}}
$$

**Physical meaning**: $p^{(4)}$ (snap, the fourth derivative) is directly proportional to **motor torque rate of change**. Quadrotor position $p$ → acceleration $\ddot{p}$ (propeller thrust + body tilt) → jerk $\dddot{p}$ (angular velocity) → snap $p^{(4)}$ (motor torque rate). **Min-snap = minimizing motor thrust oscillation** → silky attitude when racing through a window.

4. **TOPP-RA time-optimal reparameterization** (the core idea):

$$
s(t): [0, T] \to [0, 1], \quad \dot{s} \geq 0
$$

$$
\text{subject to: } \tau_{\min} \leq M(q(s))\ddot{q}(s) + C(q,\dot{q})\dot{q}(s) + g(q(s)) \leq \tau_{\max}
$$

**Physical meaning**: reparameterize the geometric path by arclength $s$, then search in the $(s, \dot{s})$ phase plane for the fastest velocity profile such that every instantaneous torque stays within limits. TOPP-RA's key change of variables $u = \ddot{s}$, $v = \dot{s}^2$ **linearizes** the nonlinear dynamics into $\tau(s) = A(s) u + B(s) v + C(s)$, turning it into a sequence of LPs solvable in $O(N)$ within 1 ms.

<details>
<summary>Deep dive: TrajOpt frameworks compared (CHOMP / STOMP / TrajOpt SQP)</summary>

### CHOMP (Covariant Hamiltonian Optimization)

Discretize the trajectory into a waypoint vector $\xi \in \mathbb{R}^{N \times n}$, gradient descent:

$$
\xi_{i+1} = \xi_i - \eta \cdot A^{-1} \nabla U(\xi_i)
$$

- **Objective** $U = U_{\text{smooth}} + U_{\text{obstacle}}$
- **The "covariant" key**: $A^{-1}$ is a Riemannian metric based on trajectory differences, spreading energy smoothly along the whole trajectory **independently of waypoint density**
- **Obstacle avoidance**: SDF (signed distance field) gradient pushes waypoints away
- **Weakness**: obstacle avoidance is a **soft constraint** → graze possible; requires differentiable cost → cannot handle non-smooth cost

### STOMP (Stochastic Trajectory Optimization)

**No gradient needed**:
1. Add smooth Gaussian noise around the reference trajectory → generate $K$ candidate trajectories $\{\xi^{(k)}\}$
2. Evaluate cost $S^{(k)}$ for each
3. Exponentially weighted average (cousin of Softmax / MPPI): $\xi_{\text{new}} = \sum_k \frac{e^{-S^{(k)}/\lambda}}{\sum_j e^{-S^{(j)}/\lambda}} \xi^{(k)}$

**Killer app**: CHOMP needs SDF gradients; for **self-collision, discrete collision checks, or avoiding camera occlusion**-style non-smooth cost, the gradient is 0 or infinite → CHOMP dies on the spot. STOMP handles non-differentiable cost perfectly through random sampling.

### TrajOpt (UC Berkeley, John Schulman)

Strict **SQP** formulation:

$$
\min_{\xi} \; f(\xi) \quad \text{s.t.} \quad \text{dist}(x_i) \geq d_{\text{safe}}, \quad \forall i
$$

**Convexification**: first-order Taylor expansion of obstacle SDF

$$
\text{dist}(x_i) \approx \text{dist}(x_{i,0}) + J_{\text{dist}}(x_{i,0}) \cdot \Delta x_i \geq d_{\text{safe}}
$$

→ linear inequality constraint, solved iteratively inside a Trust Region.

**Hard-constraint king**: as long as SQP finds a solution, the trajectory is **provably collision-free**; continuous-time collision detection (swept volume) prevents "time-sampling skipping through a narrow gap".

### Decision tree by scenario

| Scenario | First choice | Why |
|------|------|-------|
| Grabbing a book from a crowded shelf (narrow space, hard constraint) | **TrajOpt** | SQP hard constraint, no model penetration |
| Avoiding camera occlusion, avoiding liquid splash (non-differentiable cost) | **STOMP** | Sampling handles non-smooth cost |
| Smoothing OMPL's coarse output | **CHOMP** | Covariant gradient straightens polylines in milliseconds |

### MoveIt! 2 YAML configuration

```yaml
planning_pipelines:
  pipeline_names: [ompl, chomp, stomp]
ompl:
  request_adapters: |
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/CHOMPOptimizingAdapter
```

</details>

<details>
<summary>Deep dive: full polynomial interpolation derivation and matrix solution</summary>

### Cubic polynomial

Given boundary conditions: $q(0) = q_0$, $\dot{q}(0) = v_0$, $q(T) = q_f$, $\dot{q}(T) = v_f$

From $q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3$ we get $\dot{q}(t) = a_1 + 2a_2 t + 3a_3 t^2$.

Plugging in the boundary conditions gives the linear system:

$$
\begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 1 & T & T^2 & T^3 \\ 0 & 1 & 2T & 3T^2 \end{bmatrix}
\begin{bmatrix} a_0 \\ a_1 \\ a_2 \\ a_3 \end{bmatrix}
=
\begin{bmatrix} q_0 \\ v_0 \\ q_f \\ v_f \end{bmatrix}
$$

Direct solution: $a_0 = q_0$, $a_1 = v_0$,

$$
a_2 = \frac{3(q_f - q_0) - (2v_0 + v_f)T}{T^2}
$$

$$
a_3 = \frac{2(q_0 - q_f) + (v_0 + v_f)T}{T^3}
$$

### Quintic polynomial

Add two more conditions $\ddot{q}(0) = a_0^{\text{acc}}$, $\ddot{q}(T) = a_f^{\text{acc}}$; the matrix becomes 6×6. In practice one often sets $a_0^{\text{acc}} = a_f^{\text{acc}} = 0$ (rest-to-rest), giving the most common "rest-to-rest" quintic trajectory.

```python
import numpy as np

def solve_quintic(q0, v0, a0, qf, vf, af, T):
    A = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, T, T**2, T**3, T**4, T**5],
        [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
        [0, 0, 2, 6*T, 12*T**2, 20*T**3],
    ])
    b = np.array([q0, v0, a0, qf, vf, af])
    return np.linalg.solve(A, b)
```

### Minimum jerk trajectory (closed form)

When $v_0 = v_f = a_0 = a_f = 0$ (rest-to-rest), minimum jerk has an analytic solution:

$$
q(t) = q_0 + (q_f - q_0)\left[10\left(\frac{t}{T}\right)^3 - 15\left(\frac{t}{T}\right)^4 + 6\left(\frac{t}{T}\right)^5\right]
$$

This is a special case of the quintic; integral of squared jerk is minimized. Biologically, human arm reaching motions are strikingly well-fit by this curve (Flash & Hogan, 1985).

**Neuroscience origin**: Flash & Hogan (1985) discovered that human arm reaching optimizes **neither minimum time nor minimum energy**, but the **integral of squared jerk**. Biologically this represents the smoothest, most joint-friendly natural motion. Industrial robots borrowed this finding, adopting min-jerk as the gold target for PTP motion.

### Multi-waypoint interpolation

Given $n$ waypoints, use $n-1$ cubic polynomial segments, enforcing position, velocity, and acceleration continuity at every junction. This forms a tridiagonal linear system, solvable in $O(n)$ (Thomas algorithm).

**Warning**: do not use a single $(n-1)$-degree global polynomial through all waypoints — the Runge phenomenon will blow up.

</details>

<details>
<summary>Deep dive: Differential Flatness and the full min-snap QP (quadrotors)</summary>

### What Differential Flatness is

A quadrotor is an **underactuated system** (12 states `[x,y,z,ẋ,ẏ,ż,φ,θ,ψ,p,q,r]` with only 4 motor inputs; here $\psi$ is the yaw of the Euler RPY parameterization). Mellinger-Kumar (2011) proved that a quadrotor is a **differentially flat system**: there exist 4 flat outputs $\sigma = [x, y, z, \psi]$ (3D position + **yaw — the same $\psi$ as in the state vector**; roll $\phi$ and pitch $\theta$ do not appear in the flat outputs because they can be recovered algebraically from the second derivatives of the position trajectory). All 12 states and 4 control inputs can then be **purely algebraically derived from $\sigma$ and its finitely-many derivatives**.

### Why this is a dimensional-reduction hammer

Traditional approach: do nonlinear MPC in 12-D state space → compute explodes, real-time performance tanks.

Diff-Flat approach:
1. The planner works **in pure geometry** in 3-D, drawing a smooth $(x(t), y(t), z(t), \psi(t))$ curve
2. This geometric curve **automatically carries dynamic feasibility** (flatness guarantees thrust + attitude + motor commands are algebraically recoverable)
3. The drone is guaranteed to be able to fly it

This is the underlying soul of Fast-Planner, Drone Racing, and MIT ACL-style quadrotor trajectory systems.

### Min-snap QP problem setup

Each segment is parameterized by a 7th-order polynomial: $p_i(t) = \sum_{k=0}^{7} c_{i,k} t^k$.

Objective (integral of squared snap):

$$
J = \int_0^T \left\| p^{(4)}(t) \right\|^2 dt = c^\top Q c
$$

with Q-matrix entries:

$$
Q_{ij} = \int_0^T \frac{i!}{(i-4)!} \frac{j!}{(j-4)!} t^{i+j-8} \, dt = \frac{i!\, j!}{(i-4)!(j-4)!} \cdot \frac{T^{i+j-7}}{i+j-7} \quad (i, j \geq 4)
$$

### Constraints: waypoints + $C^3$ continuity

**Waypoint positions**: $p_i(0) = w_i$, $p_i(T_i) = w_{i+1}$

**$C^3$ continuity between adjacent segments** (position, velocity, acceleration, jerk continuous):

$$
p_i^{(k)}(T_i) = p_{i+1}^{(k)}(0), \quad k = 0, 1, 2, 3
$$

Combined into $A_{\text{eq}} c = b_{\text{eq}}$; the QP `min cᵀQc s.t. A_eq c = b_eq` is solved instantly by OSQP or OOQP.

### C++ skeleton (building the Q matrix)

```cpp
Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_coeffs, num_coeffs);
for (int i = 4; i < num_coeffs; ++i) {     // snap starts at order 4
    for (int j = 4; j < num_coeffs; ++j) {
        double cost_coeff = (factorial(i)/factorial(i-4)) *
                            (factorial(j)/factorial(j-4));
        Q(i, j) = cost_coeff * std::pow(T, i+j-7) / (i+j-7);
    }
}
// Call OSQP to solve min cᵀQc s.t. A_eq c = b_eq
```

### Time allocation optimization

The segment time $T_i$ is also a decision variable. Practical approach:
- **Initial guess**: coarse trapezoidal velocity profile
- **Outer iteration**: fix $c$ and update $T_i$ (NLP), then fix $T_i$ and solve QP (Richter et al. 2016)

</details>

<details>
<summary>Deep dive: TrajOpt frameworks compared, quick reference (CHOMP / STOMP / TrajOpt SQP)</summary>

### CHOMP (Covariant Hamiltonian Optimization)

Discretize the trajectory into a waypoint vector $\xi \in \mathbb{R}^{N \times n}$, gradient descent:

$$
\xi_{i+1} = \xi_i - \eta \cdot A^{-1} \nabla U(\xi_i)
$$

- **Objective** $U = U_{\text{smooth}} + U_{\text{obstacle}}$
- **The "covariant" key**: $A^{-1}$ is a Riemannian metric matrix based on trajectory differences, spreading energy smoothly along the trajectory **independent of waypoint density**
- **Obstacle avoidance**: SDF gradient pushes waypoints away
- **Weakness**: avoidance is a **soft constraint** → grazing possible; requires differentiable cost → can't handle non-smooth cost

### STOMP (Stochastic Trajectory Optimization)

**No gradient computation required**:
1. Add smooth Gaussian noise around the reference trajectory → generate $K$ candidate trajectories $\{\xi^{(k)}\}$
2. Evaluate cost $S^{(k)}$ for each
3. Exponentially weighted average (same family as Softmax / MPPI): $\xi_{\text{new}} = \sum_k \frac{e^{-S^{(k)}/\lambda}}{\sum_j e^{-S^{(j)}/\lambda}} \xi^{(k)}$

**Killer app**: CHOMP needs SDF gradients; for **self-collision, discrete collision detection, avoiding camera occlusion** and other non-smooth cost, gradients are 0 or infinite → CHOMP dies instantly. STOMP handles non-differentiable cost perfectly via random sampling.

### TrajOpt (UC Berkeley, John Schulman)

Strict **SQP** formulation with **convexification**: first-order Taylor expansion of the obstacle SDF

$$
\text{dist}(x_i) \approx \text{dist}(x_{i,0}) + J_{\text{dist}}(x_{i,0}) \cdot \Delta x_i \geq d_{\text{safe}}
$$

→ linear inequality constraints, solved iteratively inside a Trust Region.

**Hard-constraint king**: as long as SQP finds a solution, the trajectory is **guaranteed collision-free**; continuous-time collision detection (swept volume) prevents "sampling skipping through a narrow slot".

### Scenario decision tree

| Scenario | First choice | Why |
|------|------|-------|
| Grabbing a book from a crowded shelf (narrow space, hard constraint) | **TrajOpt** | SQP hard constraint, no penetration |
| Avoiding camera occlusion, avoiding liquid splash (non-differentiable cost) | **STOMP** | Sampling handles non-smooth costs |
| Post-processing a coarse OMPL trajectory | **CHOMP** | Covariant gradient straightens polylines in milliseconds |

### MoveIt! 2 YAML configuration

```yaml
planning_pipelines:
  pipeline_names: [ompl, chomp, stomp]
ompl:
  request_adapters: |
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/CHOMPOptimizingAdapter
```

</details>

## Intermediate Concepts: Time-Optimal, Contact-Aware, and Learning-based Generation

### Why TOPP-RA is the industrial standard

Traditional single-shot formulations (treating $q$ and $t$ as decision variables jointly) are **non-convex** → slow solve, prone to divergence. The industrial standard is to **decouple**: first find a geometric path $q(s)$ with RRT\*/CHOMP, then compute optimal $\dot{s}, \ddot{s}$ with TOPP-RA.

**Bang-Bang principle** (Pontryagin's minimum principle, interview must-answer): the time-optimal solution is **necessarily bang-bang** — at every instant at least one joint torque is pinned to its physical limit. Intuitively: "full-throttle acceleration → full-throttle cruise → full-throttle deceleration", squeezing every drop of hardware potential.

**Min-snap + TOPP-RA = a combo dimensional hammer**:
- Min-snap guarantees **geometric silkiness** (minimum motor thrust rate of change)
- TOPP-RA then **precisely squeezes motor torque potential** along that smooth path to time-optimal
- Convex + fast + engineering-controllable → the industrial gold pipeline

### Contact-Aware Trajectory Optimization (unifying Manipulation and Locomotion)

This is the single most important unifying perspective in modern embodied AI — **pushing a box, twisting a knob, pulling a drawer, and Atlas backflipping are mathematically the same problem**.

**CITO (Contact-Implicit TrajOpt) core math**:

$$
\begin{aligned}
\min_{q, \dot{q}, u, \lambda} \;& J(q, u) \\
\text{s.t.} \;& M(q)\ddot{q} + h(q, \dot{q}) = B u + J_c^\top \lambda \\
& \phi(q) \geq 0, \quad \lambda \geq 0, \quad \phi(q) \cdot \lambda = 0 \quad \text{(complementarity)}
\end{aligned}
$$

**Physical meaning**:
- $\phi(q) \geq 0$: non-penetration constraint (fingers / feet cannot pass through objects)
- $\lambda \geq 0$: contact forces can only push, not pull
- $\phi \cdot \lambda = 0$: **complementarity** — either separated ($\phi > 0, \lambda = 0$) or in contact ($\phi = 0, \lambda > 0$)
- **The KKT Lagrange multiplier $\lambda$ physically emerges as the contact force** — the solver autonomously decides when to switch contact modes

**Why this is a unifying view**: both manipulator-pushes-box and Atlas-backflip are **underactuated dynamics** problems — you cannot directly control the box or the airborne centre-of-mass, only **search for the optimal contact point + contact force $J^\top f_c$ to indirectly change state**. Master Contact-Implicit and both legged and dexterous-hand domains open up at once.

<details>
<summary>Deep dive: TOPP-RA change of variables and forward/backward sweep</summary>

### The core mathematical magic (interview memorize)

The original dynamics $\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q)$ are **nonlinear** in $q$ and $\ddot{q}$.

Reparameterize along the path $q(s)$:
- $\dot{q} = q'(s) \cdot \dot{s}$
- $\ddot{q} = q''(s) \cdot \dot{s}^2 + q'(s) \cdot \ddot{s}$

**Key substitution**: let $u = \ddot{s}$, $v = \dot{s}^2$ (note $\dot{v} = 2\dot{s}\ddot{s} = 2\dot{s} u$, so $u = \frac{1}{2}\frac{dv}{ds}$).

Plugging into the dynamics:

$$
\tau(s) = \underbrace{M(q(s)) q'(s)}_{A(s)} u + \underbrace{\left[M(q(s))q''(s) + C q'(s)\right]}_{B(s)} v + \underbrace{G(q(s))}_{C(s)}
$$

**The nonlinear dynamics have become linear in $u, v$!** Torque constraints $\tau_{\min} \leq \tau \leq \tau_{\max}$ become linear inequalities in $u, v$.

### TOPP-RA algorithm flow (Pham 2018)

1. Slice the trajectory into $N$ segments $\{s_0, s_1, \ldots, s_N\}$
2. **Backward pass**: from $s_N$, compute each point's reachable set $[v_{\min}^{\text{reach}}(s_i), v_{\max}^{\text{reach}}(s_i)]$
3. **Forward pass**: from $s_0$, at each point take the **maximum $v$** in the reachable set
4. Each step solves a tiny LP (typically 2 variables), $O(N)$ total, **under 1 ms**

Compared to the original TOPP (ODE integration + switch-point search):
- Original TOPP is numerically unstable, and switch-point search is lossy
- TOPP-RA is based on reachability analysis, guaranteeing global optimality + numerical robustness

### Drake TOPP-RA Python API

```python
from pydrake.all import Toppra

toppra = Toppra(geometric_path, plant)
toppra.AddJointVelocityLimit(v_min, v_max)
toppra.AddJointAccelerationLimit(a_min, a_max)
toppra.AddJointTorqueLimit(tau_min, tau_max)
time_optimal_traj = toppra.SolvePathParameterization()  # under 1 ms
```

### Why Min-snap + TOPP-RA is the perfect combo

- Min-snap takes care of **pure geometric smoothness** (min snap → min motor torque rate of change → silky attitude)
- TOPP-RA takes care of **time squeezing** (Bang-Bang maxes out torque limits)
- The two decouple → two convex problems, independently verifiable

**vs Single-shot, the crushing advantage**: single-shot is non-convex with no real-time guarantee, easily trapped in local minima; Min-snap + TOPP-RA is two stages of convex problems, with enormous engineering determinism.

</details>

<details>
<summary>Deep dive: Contact-Implicit TrajOpt and MPCC relaxation (Atlas-backflip level)</summary>

### Why complementarity constraints are hard

$\phi(q) \cdot \lambda = 0$ is non-differentiable at the origin (gradient does not exist). Standard NLP solvers (IPOPT, SNOPT) get stuck immediately.

### MPCC (Mathematical Program with Complementarity Constraints) relaxation

Introduce a small relaxation constant $\epsilon$:

$$
\phi(q) \cdot \lambda \leq \epsilon
$$

Shrink $\epsilon$ gradually (outer iteration $\epsilon_k = 0.5 \epsilon_{k-1}$); IPOPT can then solve it. Drake and Casadi ship this framework out of the box.

```python
prog.AddConstraint(phi >= 0)
prog.AddConstraint(lam >= 0)
prog.AddConstraint(phi * lam <= 1e-4)  # MPCC relaxation helps IPOPT
```

### Quadruped footstep + swing trajectory

**Footstep location**: Linear Inverted Pendulum (LIPM) or Capture Point theory keeps the CoM stable.

**Why swing trajectory uses Bézier curves**:
- **Convex Hull Property**: the whole Bézier curve is enclosed by the convex polytope of its control points
- Constrain intermediate control points to lie **above the obstacle height** $h_{\text{clearance}}$ → the entire swing-leg trajectory **cannot trip**
- Avoidance constraint reduces to **a few linear inequalities on control points** → QP solved instantly
- vs free polynomials: you would have to check the whole curve at every instant — compute explodes

### Centroidal Momentum-aware Optimization (CMM, Atlas-backflip level)

Atlas has 30+ joints; planning directly in the full state space blows up. **CMM reduces to the 6-D centroidal momentum space**:

$$
\dot{h}_G = \begin{bmatrix} m \ddot{c} \\ \dot{k}_G \end{bmatrix} = \begin{bmatrix} \sum f_i - mg \\ \sum (p_i - c) \times f_i + \tau_i \end{bmatrix}
$$

- $c$: CoM position
- $k_G$: angular momentum
- $f_i, p_i$: force + location at each contact point

**Plan only the CoM trajectory $p_c$ and angular momentum $k$ → constraint $\dot{k} = \sum (p_i - p_c) \times f_i$ → derive the ground reaction force (GRF) that produces the backflip rotational torque**. This is the core math behind Boston Dynamics Atlas' backflip.

</details>

<details>
<summary>Deep dive: Learning-based Trajectory Generation (Diffusion + Transformer + ALOHA ACT)</summary>

### Motion Planning Diffusion (MPD)

Treat the trajectory as an "image denoising" problem:
1. Initialize a Gaussian noise trajectory $\xi_T \sim \mathcal{N}(0, I)$
2. **Conditional diffusion**: start/end points / LiDAR point cloud / natural language ("go around the cup") injected as condition into a UNet
3. Iteratively denoise $\xi_{t-1} = \xi_t - \epsilon_\theta(\xi_t, t, c) + \sigma_t z$
4. A smooth trajectory finally collapses out

### Guided Diffusion with Constraints (physical feasibility patch)

Pure diffusion may still penetrate objects. Each denoising step also adds the **gradient of a physics cost**:

$$
\hat{\epsilon} = \epsilon_\theta(x_t, \varnothing) + w \cdot \left[\epsilon_\theta(x_t, c) - \epsilon_\theta(x_t, \varnothing)\right] + \lambda \nabla_x C(x_t)
$$

- $C(x) = $ SDF collision penalty + dynamics constraints
- $\nabla_x C$ pushes away from obstacles, ensuring physical feasibility

### Trajectory / Decision Transformer

Quantize (s, a, R) into tokens: `[R_1, s_1, a_1, R_2, s_2, a_2, ...]`

GPT-style causal attention predicts the next token. **At inference, feed in a high-reward target** $R_{\text{target}} = 1$ → the model autoregressively produces the optimal action trajectory. Essentially: "reframe RL as offline sequence modeling".

### ALOHA ACT (Action Chunking + Temporal Ensemble)

Workhorse for dexterous manipulation. Teleoperation data is full of hand jitter and hesitations (high-frequency noise); traditional single-step BC geometrically amplifies micro-errors.

**ACT recipe**:
1. Generate a future $k$-step chunk $\{a_t, a_{t+1}, \ldots, a_{t+k-1}\}$ in one shot
2. At each step, do exponentially-weighted moving average over **all overlapping chunks** (Temporal Ensemble)
3. Temporal low-pass filter → extremely smooth

**Why this is the only way to thread needles or crack eggs**: precise small-scale actions demand high temporal continuity; Temporal Ensemble smooths jitter in the time domain, outperforming naive action smoothing (it preserves dynamic patterns inside each chunk).

```python
all_predictions = np.zeros((T, T, action_dim))
for t in range(T):
    chunk = act_policy(obs)  # k steps at once
    all_predictions[t, t:t+k] = chunk
    valid = all_predictions[:, t, :]  # all past predictions for step t
    weights = np.exp(-np.arange(len(valid)) / decay)
    smooth_action_t = np.sum(valid * weights / weights.sum(), axis=0)
```

### VLA (RT-2 / OpenVLA / π0) Trajectory Head

VLAs do not output single-step actions directly; they emit **1-2 second continuous spatial waypoints**, piped into an Action Chunking Head (MDN or Diffusion Head).

### "Symbol → Neural" modern architectural philosophy

No more blind collision-check search → **a Diffusion Model "imagines" expert trajectories → MPC / impedance controller digests the residual**. The problem solved is not raw precision but **"open-world human-intuition generalization"**.

</details>

## Advanced Concepts: iLQR/DDP, Multi-phase, and Rollout Bridging

### The Hessian secret behind iLQR vs DDP

**iLQR (Iterative LQR)** backward Bellman recursion:

$$
V(x_k) = \min_u \left[\ell(x, u) + V_{k+1}(f(x, u))\right]
$$

Along the nominal trajectory $(\bar{x}_k, \bar{u}_k)$, **linearize dynamics + quadraticize cost**, compute the quadratic expansion of the Q-function $(Q_x, Q_u, Q_{xx}, Q_{uu}, Q_{ux})$, and solve for the optimal control law:

$$
\delta u_k = k_k + K_k \cdot \delta x_k \quad \text{(feedforward + feedback)}
$$

**DDP (Differential Dynamic Programming)** vs iLQR additionally retains the **second-order Hessian tensor of the dynamics** $\partial^2 f / \partial x^2$:
- iLQR: Gauss-Newton approximation, only uses the Jacobian $(f_x, f_u)$
- DDP: quadratic convergence, perceives world "curvature" (rotational centripetal forces etc.)
- **Cost**: second-order tensor computation is very expensive
- **Practical takeaway**: iLQR has **the best performance-per-cost**, and is what you want in almost every scenario

### Shooting vs Multiple Shooting vs Direct Collocation

| Method | Decision variables | Dynamics handling | Suitable for |
|------|---------|-----------|------|
| **Single Shooting** | $u_{0:T}$ | Hard integration through physics engine | Short horizons, weakly nonlinear |
| **Multiple Shooting** | $x_{0:T}, u_{0:T-1}$ + gap-closing constraints | Piecewise integration + continuity equalities | Medium horizons, numerical stability required |
| **Direct Collocation** | $x_{0:T}, u_{0:T}$ at every timestep | Algebraic equalities between adjacent nodes (Hermite-Simpson) | **Atlas backflip**, long horizons, strongly nonlinear |

**Why Atlas backflip must use Collocation** (interview must-answer):
- Single Shooting: the solver makes a small torque error early on → $x(t)$ integration blows up mid-air → gradients diverge, solver is lost
- **Collocation allows the solver to violate physics in early iterations** (Defect $\neq 0$): let the robot "float" through the flip geometry first, then gradually tighten dynamics constraints
- "Simultaneously optimize in geometric and physical space" is far more stable than Shooting

**Scenario decision tree**:
- **Microsecond-scale online MPC + simple non-convex** → iLQR (no NLP solver dependency, hand-written C++ matrix-multiplies at 1 kHz)
- **Offline large-scale multi-phase extreme motions like Atlas backflip** → Direct Collocation + IPOPT (slow but strong nonlinear constraint exploration + numerical stability)
- **Manipulator grasping, quadruped gaits** → Multiple Shooting + Casadi + IPOPT (the balanced choice)

### Multi-phase / Hybrid Trajectory Optimization

Phase-switching trajectories: **P₁ double-leg support → P₂ flight → P₃ single-leg touchdown**

Each phase has different dynamics:
- P₁: driven by GRF ($\ddot{x} = f(x, u, \text{GRF})$)
- P₂: GRF = 0, degenerating to a ballistic rigid body + free rotation
- P₃: the other foot's GRF

**Adjacent phases must have a state continuity constraint** $x(P_1, \text{end}) = x(P_2, \text{start})$.

**Angular momentum conservation during flight** (ice skater principle):
Flight phase has no external moment → Newton-Euler $\dot{k}_{\text{com}} = \sum r \times F_{\text{grf}} = 0$

$$
k_{\text{com}}(t) = k_{\text{com}}(t_0), \quad t \in [\text{takeoff}, \text{landing}]
$$

**Atlas airborne front flip**: at the moment of takeoff, in the last instant of P₁, GRF must create **enough initial angular momentum**; mid-air, the robot contracts its limbs to reduce its inertia tensor and spin faster.

<details>
<summary>Deep dive: full iLQR backward/forward Riccati derivation</summary>

### Quadratic expansion of the Q-function

$$
Q(\delta x, \delta u) = \ell(x + \delta x, u + \delta u) + V'(f(x + \delta x, u + \delta u))
$$

Taylor-expand around $(\bar{x}, \bar{u})$ to second order:

$$
\begin{aligned}
Q_x &= \ell_x + f_x^\top V_x' \\
Q_u &= \ell_u + f_u^\top V_x' \\
Q_{xx} &= \ell_{xx} + f_x^\top V_{xx}' f_x \\
Q_{uu} &= \ell_{uu} + f_u^\top V_{xx}' f_u \\
Q_{ux} &= \ell_{ux} + f_u^\top V_{xx}' f_x
\end{aligned}
$$

(DDP adds extra second-order terms $V_x' \cdot f_{xx}, f_{uu}, f_{ux}$; iLQR drops them.)

### Optimal control law

$\delta u^* = -Q_{uu}^{-1}(Q_u + Q_{ux} \delta x) = k + K \delta x$

- Feedforward $k = -Q_{uu}^{-1} Q_u$
- Feedback $K = -Q_{uu}^{-1} Q_{ux}$

### Value function backward update

$$
V_x = Q_x + K^\top Q_{uu} k + K^\top Q_u + Q_{ux}^\top k
$$

$$
V_{xx} = Q_{xx} + K^\top Q_{uu} K + K^\top Q_{ux} + Q_{ux}^\top K
$$

**After substituting $k = -Q_{uu}^{-1} Q_u$ and $K = -Q_{uu}^{-1} Q_{ux}$**, these simplify to the standard form most implementations use directly:

$$
V_x = Q_x - Q_{ux}^\top Q_{uu}^{-1} Q_u = Q_x + K^\top Q_u, \qquad V_{xx} = Q_{xx} - Q_{ux}^\top Q_{uu}^{-1} Q_{ux} = Q_{xx} + K^\top Q_{ux}
$$

The four-term expanded form above is algebraically correct but its inter-term sign relationships rely on the negative signs in the definitions of $k, K$; reading the expansion without substituting in $k, K$ makes it easy to get wrong.

### Regularization to keep $Q_{uu}$ positive definite

Add $\mu I$ to $Q_{uu}$, the same way as Levenberg-Marquardt. Large $\mu$ degenerates to gradient descent (stable but slow); small $\mu$ approaches Newton (fast but may diverge).

</details>

<details>
<summary>Deep dive: Direct Collocation and Casadi implementation (vs Multiple Shooting)</summary>

### Hermite-Simpson collocation

Each segment $[t_k, t_{k+1}]$ uses cubic Hermite interpolation at the midpoint $t_{k+1/2}$:

$$
x(t_{k+1/2}) = \frac{1}{2}(x_k + x_{k+1}) + \frac{h}{8}(\dot{x}_k - \dot{x}_{k+1})
$$

Dynamics constraint: the dynamics value at the midpoint must match the derivative of the interpolant

$$
\dot{x}(t_{k+1/2}) = -\frac{3}{2h}(x_k - x_{k+1}) - \frac{1}{4}(\dot{x}_k + \dot{x}_{k+1})
$$

The equation $\dot{x}(t_{k+1/2}) - f(x(t_{k+1/2}), u(t_{k+1/2})) = 0$ is the collocation constraint.

### Casadi Multiple Shooting skeleton

```python
import casadi as ca

opti = ca.Opti()
X = opti.variable(nx, N+1)  # states
U = opti.variable(nu, N)    # controls

# Dynamics constraint (Runge-Kutta 4 integration)
def rk4(x, u, dt):
    k1 = f(x, u)
    k2 = f(x + dt/2 * k1, u)
    k3 = f(x + dt/2 * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

for k in range(N):
    x_next = rk4(X[:, k], U[:, k], dt)
    opti.subject_to(X[:, k+1] == x_next)  # Gap closing
    opti.subject_to(opti.bounded(u_min, U[:, k], u_max))

# Boundary conditions
opti.subject_to(X[:, 0] == x0)
opti.subject_to(X[:, N] == xf)

# Objective
cost = sum(ca.sumsqr(U[:, k]) for k in range(N))
opti.minimize(cost)

opti.solver('ipopt')
sol = opti.solve()
```

### Engineering cure for multi-phase divergence

NLP is essentially local optimization; multi-phase problems have strong nonlinearity (SO(3) rotations + bilinear GRF coupling). A bad initial guess (GRF too weak against gravity, footholds too far) traps the solver instantly.

**Engineering iron rule**:
1. First solve a coarse but physically reasonable CoM trajectory + GRF using **simplified SRBD (Single Rigid Body Dynamics) or LIPM** (convex problem, solved in seconds)
2. Feed this coarse solution as an **Initial Guess** to full-size TrajOpt
3. Converges 100% of the time, and is typically 10-100× faster than naive zero-initial-guess

This is the signature move of industry veterans.

</details>

<details>
<summary>Deep dive: bridging TrajOpt to closed-loop control (TVLQR / Funnel / DAgger)</summary>

### Why an open-loop trajectory cannot be executed directly

TrajOpt squeezes every drop of potential from the model — the solution is usually **pinned to the edge of the friction cone**. Real-world errors (mass / inertia / friction) + disturbances (wind, uneven ground) + state drift → butterfly effect divergence → the robot falls.

### TVLQR (Time-Varying LQR) bridging solution

- **Nominal trajectory** $\bar{x}(t), \bar{u}(t)$ is the reference
- Linearize along the trajectory: $\delta x_{k+1} \approx A_k \delta x_k + B_k \delta u_k$
- Solve Riccati backward → time-varying feedback gain $K(t)$
- **On-hardware control law**: $u(t) = \bar{u}(t) + K(t) \cdot [x(t) - \bar{x}(t)]$

$\bar{u}$ provides the **big feedforward drive**; $K$ is like a spring pulling deviated states back.

### iLQR's K is "free"

The iLQR Backward Pass has already computed $K_k$! Just push $(u^*_k, x^*_k, K_k)$ to the hardware and you have **plug-and-play closed-loop tracking**. This is the key difference between "someone who uses iLQR" and "someone who understands iLQR".

### Funnel Control / ROA (MIT Tedrake)

**Sum-of-Squares (SOS) optimization** computes a Lyapunov "invariant tube" surrounding the trajectory:

$$
V(x, t) \leq \rho(t) \Rightarrow x(t+dt) \text{ still in funnel}
$$

As long as the initial disturbance falls inside the funnel entrance ($V(x_0) \leq \rho(0)$), **theoretical guarantee**: no fall + convergence to the goal. Graduate-level TrajOpt robustness guarantee.

### Modern TrajOpt + NN Residual (DAgger + Distillation)

1. Offline, use TrajOpt to generate **tens of thousands of perfect trajectories under different disturbances** (Teacher)
2. Train a NN Student policy to fit them
3. Deploy for O(1) blazing-fast closed loop + preserved TrajOpt optimality

### The "open-loop perfect, hardware divergent" interview answer pattern

"TrajOpt gives you only **feedforward**. On hardware I would:
1. Compute **TVLQR $K_t$** along the nominal trajectory to provide closed-loop feedback
2. If there is a low-level WBC (whole-body controller), feed TrajOpt's $\bar{x}, \dot{\bar{x}}, \ddot{\bar{x}}$ as reference commands into the 1 kHz WBC
3. **The WBC's PD impedance + dynamics feedforward absorb unmodeled disturbances** — this is the industry iron rule for Atlas / ANYmal extreme motion"

</details>

## Comparison of Common Parameterizations

| Method | Continuity | Local control | Pros | Cons |
|------|--------|---------|------|------|
| Cubic polynomial | $C^1$ | No | Simple, analytic | Acceleration not continuous |
| Quintic polynomial | $C^2$ | No | Acceleration continuous | High-order oscillation (multi-segment) |
| Trapezoidal velocity | $C^0$ (velocity) | Yes | Industrial standard, cheap | Jerk is infinite (step) |
| S-curve (7-segment) | $C^1$ (acceleration) | Yes | Jerk bounded, no shock | More parameters, heavier compute |
| B-spline | $C^{k-1}$ | **Yes** | Local edits don't affect the whole | Need to choose knot vector |
| Minimum snap | $C^3$ | No | Quadrotor standard | Compute grows with waypoints |
| Bézier (convex hull) | $C^k$ | Yes | Control point geometry = avoidance | Must be same-degree |

<details>
<summary>Deep dive: B-spline basics and the Cox-de Boor recursion</summary>

### B-spline trajectory

An order-$k$ B-spline curve is defined by control points $P_i$ and a knot vector $\{t_0, t_1, \ldots, t_{n+k}\}$:

$$
q(t) = \sum_{i=0}^{n} N_{i,k}(t) P_i
$$

where $N_{i,k}(t)$ are the B-spline basis functions (Cox-de Boor recursion):

$$
N_{i, 0}(t) = \begin{cases} 1 & t_i \leq t < t_{i+1} \\ 0 & \text{otherwise} \end{cases}
$$

$$
N_{i, k}(t) = \frac{t - t_i}{t_{i+k} - t_i} N_{i, k-1}(t) + \frac{t_{i+k+1} - t}{t_{i+k+1} - t_{i+1}} N_{i+1, k-1}(t)
$$

**Key properties**:
- **Local control**: moving a control point only affects the curve within $k$ knot spans (no global Runge oscillation)
- **Convex hull property**: each curve segment lies within the convex hull of its control points (collision-check friendly)
- **Differentiation is simple**: the derivative of an order-$k$ B-spline is an order-$(k-1)$ B-spline, computed via control-point differences

**Usage in trajectory optimization**: treat control points as decision variables, solve a QP or NLP minimizing $\int \|\ddot{q}\|^2 dt$ or $\int \|\dddot{q}\|^2 dt$, with velocity / acceleration / collision constraints.

</details>

## Intuition

**Analogy: road trip**

- **Path** = the route drawn by Google Maps (geometry only)
- **Trajectory** = the complete driving plan: where to hit the gas, where to brake, slow down before turns, and do not let the passengers get carsick
- **Minimum time** = race driver: full acceleration to the limit, hard brakes, apex cut — fast but uncomfortable
- **Minimum energy** = economy driver: smooth accelerations, coast early — slow but fuel-efficient
- **Minimum smoothness** = driving Grandma around: gentle start, progressive braking, jerk so small it's imperceptible — most comfortable but slowest
- **Pareto front** = you cannot be fastest, most fuel-efficient, and most comfortable simultaneously; you can only trade off

**Visual difference between trapezoidal velocity and S-curve**:

- **Trapezoidal velocity**: the velocity curve looks like a trapezoid (accelerate-cruise-decelerate), but acceleration is a square wave → jerk is a delta function → a "clack" as the arm starts/stops
- **S-curve**: rounds off the square-wave acceleration (7 segments); jerk is bounded → silky starts/stops, no mechanical shock

**Physical intuition for Differential Flatness**:
Think of a quadrotor as a "writing ghost hand" — you only have to draw a beautiful smooth curve in 3-D space (flat outputs $[x, y, z, \psi]$), and all 12 states and 4 motor thrusts are **algebraically recoverable** from that curve. The planner does not care about dynamics; it just draws pretty curves, and dynamics are automatically satisfied.

**Intuition for Contact-Implicit**:
Imagine you want "to nudge a pool ball to a particular spot with a cue stick" — you do not plan "the ball's trajectory", you plan "when the cue should touch the ball and with what force". Contact-Implicit TrajOpt generalizes this: the robot does not directly control the manipulated object, it **plans when and with what force to contact the environment**, using contact forces to indirectly push the system.

**Simulator observations**:
- **MuJoCo**: run the same PTP motion with trapezoidal velocity vs S-curve and observe (1) how smooth the end-effector trajectory is (zoom in for tiny vibrations), (2) joint torque curves (trapezoidal shows obvious jumps), (3) settling time near the goal (S-curve converges faster because it excites no high-frequency resonance)
- **Isaac Sim + Drake**: run Contact-Implicit TrajOpt to push a box with the manipulator, and watch how the solver **automatically decides when to touch the box and when to release it** — that is the physical emergence of the KKT multiplier $\lambda$
- **Gazebo + Pinocchio**: Atlas airborne backflip, visualize the angular momentum vector $k$ staying constant during flight — direct verification of Newton-Euler conservation

## Implementation Links

**Typical engineering scenarios**:

1. **Industrial PTP pick-and-place**: need minimum cycle time but torque cannot exceed limits. Use TOPP-RA to do time-optimal reparameterization on the waypoints from the path planner. Use RRT/PRM for the geometric path, then TOPP-RA to inject time.

2. **Welding / glue-dispensing linear motion**: the end-effector must follow a Cartesian line with uniform velocity, no jitter. Use minimum jerk or B-spline parameterization with a jerk upper-bound constraint. A Cartesian line may cross singularities in joint space → check Jacobian condition number.

3. **Quadrotor waypoint flight**: between waypoints use **minimum snap + Differential Flatness**. Open-source systems like Fast-Planner use this pipeline: take the 3D geometric curve directly as reference, a low-level geometric controller back-computes thrust and attitude from the flat outputs.

4. **Grabbing a book from a crowded shelf (hard-constraint narrow space)**: use **TrajOpt SQP**. Convexify SDF + Trust Region iteration, guaranteeing no grazing. One of the default pipelines in MoveIt 2.

5. **Quadruped leg swing over obstacle**: Bézier swing trajectory, constrain control points above the obstacle → convex-hull property guarantees clearance along the entire curve.

6. **Atlas backflip / extreme parkour**: multi-phase Direct Collocation + CITO, with SRBD coarse solution as the initial guess.

7. **ALOHA dexterous manipulation**: Action Chunking Transformer + Temporal Ensemble, VLA directly outputs continuous waypoints, downstream impedance controller digests residuals.

**Code skeleton** (Python, TOPP-RA):

```python
import toppra as ta
import numpy as np

# 1. Define geometric path (joint-space waypoints)
waypoints = np.array([[0, 0, 0, 0, 0, 0],
                       [1.0, -0.5, 0.8, 0, 0, 0],
                       [1.5, -1.0, 1.2, 0, 0, 0]])
path = ta.SplineInterpolator(
    np.linspace(0, 1, len(waypoints)), waypoints)

# 2. Define constraints (velocity, acceleration, torque)
vel_limit = np.array([2.0] * 6)   # rad/s
acc_limit = np.array([5.0] * 6)   # rad/s^2
pc_vel = ta.constraint.JointVelocityConstraint(vel_limit)
pc_acc = ta.constraint.JointAccelerationConstraint(acc_limit)

# 3. Build TOPP-RA instance and solve
instance = ta.algorithm.TOPPRA(
    [pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
trajectory = instance.compute_trajectory()  # returns time-optimal trajectory

# 4. Sample → send to controller
ts = np.linspace(0, trajectory.duration, 100)
qs = trajectory(ts)          # position (100, 6)
qds = trajectory(ts, 1)     # velocity
qdds = trajectory(ts, 2)    # acceleration
```

<details>
<summary>Deep dive: full implementation — quintic polynomial + minimum jerk (Python)</summary>

```python
import numpy as np
import matplotlib.pyplot as plt


def quintic_trajectory(q0, qf, v0, vf, a0, af, T, dt=0.001):
    """Quintic polynomial trajectory: satisfies 6 boundary conditions."""
    A = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, T, T**2, T**3, T**4, T**5],
        [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
        [0, 0, 2, 6*T, 12*T**2, 20*T**3],
    ])
    b = np.array([q0, v0, a0, qf, vf, af])
    coeffs = np.linalg.solve(A, b)

    ts = np.arange(0, T + dt, dt)
    q = sum(coeffs[i] * ts**i for i in range(6))
    qd = sum(i * coeffs[i] * ts**(i-1) for i in range(1, 6))
    qdd = sum(i * (i-1) * coeffs[i] * ts**(i-2) for i in range(2, 6))
    qddd = sum(i * (i-1) * (i-2) * coeffs[i] * ts**(i-3) for i in range(3, 6))
    return ts, q, qd, qdd, qddd


def minimum_jerk(q0, qf, T, dt=0.001):
    """Minimum-jerk trajectory (rest-to-rest closed form, Flash & Hogan 1985)."""
    ts = np.arange(0, T + dt, dt)
    tau = ts / T  # normalized time [0, 1]
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    q = q0 + (qf - q0) * s
    sd = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
    qd = (qf - q0) * sd
    sdd = (60 * tau - 180 * tau**2 + 120 * tau**3) / T**2
    qdd = (qf - q0) * sdd
    sddd = (60 - 360 * tau + 360 * tau**2) / T**3
    qddd = (qf - q0) * sddd
    return ts, q, qd, qdd, qddd
```

**Note**: under rest-to-rest conditions, the quintic polynomial ($v_0=v_f=a_0=a_f=0$) and the minimum-jerk solution are **identical** — both are the $10\tau^3 - 15\tau^4 + 6\tau^5$ curve. They diverge when boundary conditions are non-zero.

</details>

<details>
<summary>Deep dive: full implementation — Min-snap QP (Python + OSQP)</summary>

```python
import numpy as np
import osqp
from scipy import sparse
from math import factorial

def build_Q_snap(T, poly_order=7):
    """Build the snap quadratic matrix Q."""
    n = poly_order + 1
    Q = np.zeros((n, n))
    for i in range(4, n):
        for j in range(4, n):
            coeff = (factorial(i) / factorial(i-4)) * (factorial(j) / factorial(j-4))
            Q[i, j] = coeff * T**(i+j-7) / (i+j-7)
    return Q

def min_snap_qp(waypoints, segment_times, poly_order=7):
    """
    waypoints: (M+1, dim), M segments
    segment_times: (M,) per-segment time
    Each segment uses a poly_order polynomial.
    """
    M = len(segment_times)
    dim = waypoints.shape[1]
    n = poly_order + 1

    # Block diagonal Q
    Q_blocks = [build_Q_snap(T) for T in segment_times]
    Q = sparse.block_diag(Q_blocks * dim, format='csc')

    # Equality constraints: waypoints + C^3 continuity
    A_list, b_list = [], []
    # ... (omitted: waypoint position constraints + C^3 continuity between segments)

    A = sparse.vstack(A_list, format='csc')
    b = np.concatenate(b_list)

    prob = osqp.OSQP()
    prob.setup(Q, np.zeros(Q.shape[0]), A, b, b)  # eq constraint
    result = prob.solve()
    return result.x  # coefficients
```

</details>

<details>
<summary>Deep dive: full implementation — iLQR swing-up pendulum (Python)</summary>

```python
import numpy as np

def ilqr_swingup(dynamics, cost, x0, u_init, max_iter=50):
    """
    iLQR solving a pendulum swing-up.
    dynamics: f(x, u) -> x_next
    cost: (l, l_x, l_u, l_xx, l_uu, l_ux) at (x, u)
    """
    N = len(u_init)
    u = u_init.copy()
    x = np.zeros((N+1, len(x0)))
    x[0] = x0

    for iteration in range(max_iter):
        # Forward pass: roll out the trajectory
        for k in range(N):
            x[k+1] = dynamics(x[k], u[k])

        # Backward pass: Riccati recursion
        V_x = cost_terminal_x(x[N])
        V_xx = cost_terminal_xx(x[N])
        K = np.zeros((N, nu, nx))
        k_ff = np.zeros((N, nu))

        for k in range(N-1, -1, -1):
            f_x, f_u = linearize(dynamics, x[k], u[k])
            l_x, l_u, l_xx, l_uu, l_ux = quadratize(cost, x[k], u[k])

            Q_x = l_x + f_x.T @ V_x
            Q_u = l_u + f_u.T @ V_x
            Q_xx = l_xx + f_x.T @ V_xx @ f_x
            Q_uu = l_uu + f_u.T @ V_xx @ f_u + mu * np.eye(nu)  # regularization
            Q_ux = l_ux + f_u.T @ V_xx @ f_x

            K[k] = -np.linalg.solve(Q_uu, Q_ux)
            k_ff[k] = -np.linalg.solve(Q_uu, Q_u)

            V_x = Q_x + K[k].T @ Q_uu @ k_ff[k] + K[k].T @ Q_u + Q_ux.T @ k_ff[k]
            V_xx = Q_xx + K[k].T @ Q_uu @ K[k] + K[k].T @ Q_ux + Q_ux.T @ K[k]

        # Line-search forward pass
        alpha = 1.0
        for _ in range(10):
            x_new = np.zeros_like(x); x_new[0] = x0
            u_new = np.zeros_like(u)
            for k in range(N):
                u_new[k] = u[k] + alpha * k_ff[k] + K[k] @ (x_new[k] - x[k])
                x_new[k+1] = dynamics(x_new[k], u_new[k])
            if total_cost(x_new, u_new) < total_cost(x, u):
                x, u = x_new, u_new
                break
            alpha *= 0.5

    return x, u, K  # K is the closed-loop feedback gain — deploy with u_t = u_t* + K_t (x_t - x_t*)
```

**Key point**: `K` appears for free at the end of iteration → plug it straight into hardware as TVLQR closed loop, no extra computation.

</details>

## Common Misconceptions

1. **"Time-optimal and energy-optimal can be achieved simultaneously"** — Wrong. Time-optimal requires acceleration pinned to limits (bang-bang), which drives peak torque high and burns energy; energy-optimal prefers slow motion with small torques. The two are trade-offs on the Pareto frontier. In practice, use a weighted objective $J = w_1 T + w_2 \int \tau^2 dt$ to compromise.

2. **"Higher polynomial order always makes the trajectory better"** — Wrong. High-order polynomials suffer from the **Runge phenomenon**: violent oscillation in the middle segments. B-splines solve this with piecewise low-order polynomials + local control. Rule of thumb: global polynomials beyond 7th order are almost never used.

3. **"A straight line in Cartesian space is also smooth in joint space"** — Wrong. Once the Cartesian line is mapped through IK into joint space, if it passes close to a **singular configuration**, the Jacobian becomes rank-deficient → joint velocities blow up. **Workaround**: before executing a Cartesian line, scan the Jacobian condition number $\kappa(J)$; if $\kappa > 100$, detour or fall back to joint-space interpolation.

4. **"Kinematic clipping is enough"** — Wrong. Limiting only $\dot{q}_{\max}$ and $\ddot{q}_{\max}$ without considering dynamics (mass, Coriolis, gravity) produces trajectories that may demand torque beyond the motor's rated limit. The value of TOPP-RA is precisely that it folds **dynamics constraints** ($\tau_{\min} \leq \tau \leq \tau_{\max}$) into the optimization.

5. **"iLQR and DDP are basically the same, either is fine"** — Wrong. DDP retains the second-order Hessian tensor of the dynamics, gaining quadratic convergence and curvature awareness, but at a huge computational cost ($O(n^3)$ tensor multiplies). **Most production systems use iLQR** for best performance-per-cost; DDP is only worth the extra cost for extremely dynamic motions (high-speed spins, strong nonlinearity).

6. **"Atlas backflip can be solved with Single Shooting"** — Wrong. Single Shooting makes a slight torque error in early iterations → the physics engine integrates it into a mid-air explosion → gradients diverge and the solver is lost. Atlas backflip must use **Direct Collocation**, which allows the iterate to violate physics (Defect $\neq 0$) early, letting the geometry float first and tightening dynamics constraints gradually.

7. **"Open-loop TrajOpt trajectories can be executed directly on hardware"** — Wrong. TrajOpt squeezes every drop of the model (right at the edge of the friction cone), and real-world disturbances diverge within a second. **You must add a closed loop**: TVLQR $K_t$, 1 kHz WBC to absorb disturbances, or Funnel SOS to stay in the tube. This is the root cause of "open-loop perfect, hardware diverges".

8. **"Contact-Implicit and pre-defined mode sequence are about the same"** — Wrong. Mode sequence hard-codes "takeoff at 0.5 s, land at 1.2 s" → fast to compute but sacrifices autonomy; Contact-Implicit **discovers switching times autonomously**, but MPCC relaxation is harder to converge. Early MIT Cheetah 3 used mode sequences; modern Drake / Pinocchio leans increasingly on CITO.

## Practice Problems

<details>
<summary>Q1: Your factory pick-and-place line has a cycle time 20% too long, and the boss wants it cut — but motor torques are already near their limits. What do you do?</summary>

**Full reasoning chain**:

1. **Identify the bottleneck**: first look at what trajectory planning is currently used — if it is trapezoidal velocity, acceleration limits may be set too conservatively. Record joint torque curves and check how much margin remains versus the real limit
2. **TOPP-RA reparameterization**: feed the existing path into TOPP-RA with the real **torque constraints** (not the kinematic limits) and let it find the time-optimal profile automatically. **The Bang-Bang principle guarantees** that the time-optimal solution pins at least one joint torque at the limit at every instant; this typically outperforms hand-tuned trapezoidal parameters by 15-30%
3. **Path itself may need to change**: if TOPP-RA already maxes out torque and is still too slow, the geometric path is too long. Replan a shorter path using RRT-Connect + shortcutting or CHOMP
4. **Traps to avoid**: do not just raise the acceleration cap — you will exceed torque and damage reducers. Do not ignore jerk — even when torque is within limits, excessive jerk excites harmonic vibration and degrades positioning accuracy
5. **Final validation**: in the simulator run the full cycle, ensure torque headroom ≥ 10%, jerk is in an acceptable range, and settling time has not regressed

**Conclusion**: TOPP-RA + path optimization in tandem — squeeze time first, then verify torque and jerk safety.

</details>

<details>
<summary>Q2: A quadrotor has to pass through 5 waypoints for high-speed cinematography, requiring smooth flight and trackable reference. What trajectory representation do you pick and why?</summary>

**Full reasoning chain**:

1. **Choose Minimum Snap + Differential Flatness** (Mellinger-Kumar 2011): quadrotor thrust $f$ is proportional to acceleration ($f = m\ddot{x} + mg$), rate-of-change of thrust is proportional to jerk, and **rate-of-change of thrust rate is proportional to snap**. Minimizing snap → minimizing high-order motor torque variation → silky attitude, can race through windows without losing control
2. **Diff-Flat dimensional hammer**: the planner only has to draw $[x, y, z, \psi]$ curves in 3-D geometric space; all 12 states and 4 motor inputs can be algebraically back-computed. No need to solve MPC in 12-D nonlinear dynamics
3. **Math form**: each segment is a 7th-order polynomial (8 coefficients); 5 waypoints → 4 segments. At each waypoint demand position, velocity, acceleration, and jerk continuity ($C^3$). QP form `min cᵀQc s.t. A_eq c = b_eq`
4. **Time allocation**: initialize segment times $T_i$ with a trapezoidal velocity profile, then jointly optimize via NLP (Richter et al. 2016)
5. **Combine with TOPP-RA**: after Min-snap, pipe through TOPP-RA to squeeze motor torque — convex + fast + high engineering determinism
6. **Trap to avoid**: forgetting thrust limits → snap is minimized but some segment's acceleration exceeds max thrust → add inequality constraints or post-process with TOPP-RA

**Conclusion**: Min-snap QP + Diff-Flat + TOPP-RA post-processing is the standard pipeline for open-source quadrotor trajectory systems like Fast-Planner.

</details>

<details>
<summary>Q3: A manipulator must grab a book from a crowded shelf with many obstacles. CHOMP, STOMP, or TrajOpt — which and why?</summary>

**Full reasoning chain**:

1. **Key constraint: hard + narrow space** → **TrajOpt SQP**. CHOMP treats avoidance as a soft constraint (gradient push); in narrow corridors it may graze. TrajOpt enforces $\text{SDF}(x) \geq d_{\text{safe}}$, so if SQP finds a solution the trajectory is **provably collision-free**
2. **Convexification**: TrajOpt Taylor-expands the SDF to first order `dist(x) ≈ dist(x₀) + J_dist·Δx ≥ d_safe` → linear inequalities → solved by Trust Region SQP
3. **Continuous-time collision detection**: TrajOpt uses swept volume, preventing "time-sampled trajectories slipping through a narrow gap" — critical in a tight shelf
4. **Why not CHOMP**: CHOMP's covariant gradient is fast in smooth open spaces, but oscillates at obstacle boundaries in truly narrow hard-constraint settings
5. **Why not STOMP**: STOMP shines on non-smooth cost (e.g. "avoid blocking the camera"). Shelf avoidance cost is smooth (SDF), so STOMP wastes samples
6. **MoveIt 2 implementation**: `planning_pipelines.pipeline_names: [trajopt]`, with `d_safe = 0.02` m
7. **Trap to avoid**: TrajOpt is SQP — initial guess matters. Use RRT-Connect for a coarse path as the starting point

**Conclusion**: TrajOpt SQP + SDF convexification with hard constraints is the standard answer for "retrieval from a narrow space".

</details>

<details>
<summary>Q4: A welding manipulator vibrates at the tip while moving along a straight line, and weld quality fails. What is your analysis flow?</summary>

**Full reasoning chain**:

1. **Look at frequency first**: high-frequency sample end-effector vibration with an accelerometer or encoder. Low frequency (< 10 Hz) → trajectory issue; high frequency (> 50 Hz) → structural resonance or controller issue
2. **Trajectory angle**:
   - Check whether trapezoidal velocity is being used → infinite jerk excites resonance → switch to S-curve or minimum jerk (or septic polynomial)
   - Check whether the Cartesian interpolation passes close to a singularity → Jacobian condition number spikes → joint velocities oscillate → monitor $\kappa(J)$, stay away from singularities or use damped least squares
   - Check whether waypoint spacing causes interpolation error → densify waypoints
3. **Control angle**: high PD gains amplify high-frequency noise; add a low-pass filter on velocity commands
4. **Mechanical angle**: check reducer backlash, linkage stiffness
5. **Trap to avoid**: do not reduce speed blindly to hide vibration — that only masks the problem. Identify the root cause (jerk step / singularity / controller gain) and treat it

**Conclusion**: frequency analysis localizes the root cause; use min-jerk + B-spline trajectory to reduce high-frequency excitation and keep a safe margin from singularities.

</details>

<details>
<summary>Q5: Atlas has to do a backflip. How do you formulate this trajectory optimization problem?</summary>

**Full reasoning chain**:

1. **Framework choice: Direct Collocation + Contact-Implicit + IPOPT**
   - Why not Shooting: early-iteration torque errors → integration blows up mid-air → gradients diverge
   - Collocation treats $x_k, u_k$ as decision variables; dynamics are equality constraints between adjacent time points, and **early iterations are allowed to violate physics** → let the geometry float first and tighten dynamics later
2. **Multi-phase partition**:
   - P₁: takeoff (both feet driven by GRF)
   - P₂: flight (GRF = 0, Newton-Euler rigid body)
   - P₃: landing
3. **Flight-phase angular momentum conservation**: enforce $\dot{k}_{\text{com}}(t) = 0$ or $k_{\text{com}}(t) = k_{\text{com}}(t_0)$. At Atlas' takeoff instant, the last moment of P₁ must use GRF to create **sufficient initial angular momentum**; in the air the robot contracts its limbs to reduce the inertia tensor and spin faster (figure-skater principle)
4. **Dimensionality reduction: CMM centroidal momentum space**: Atlas has 30+ joints; full-state planning explodes. Plan instead a CoM trajectory $p_c$ and angular momentum $k$, with constraint $\dot{k} = \sum (p_i - p_c) \times f_i$
5. **Initial guess strategy**: first solve with **SRBD (Single Rigid Body Dynamics)** for a coarse but physically reasonable CoM trajectory + GRF → feed as the Initial Guess to full-size TrajOpt. **100% convergence, and typically 10-100× faster than naive zero-initial-guess**
6. **Closed-loop bridging**: TrajOpt gives open-loop $\bar{x}(t), \bar{u}(t)$; during deployment compute TVLQR $K_t$ along the trajectory or feed a 1 kHz WBC as reference to absorb real hardware disturbances
7. **Trap to avoid**: do not optimize phase durations and trajectory jointly — NLPs are highly local-optimum sensitive. Fix phase durations first, outer bisection later

**Conclusion**: Multi-phase Direct Collocation + CMM reduction + angular momentum conservation + SRBD initial guess + TVLQR/WBC closed loop.

</details>

<details>
<summary>Q6: An ALOHA teleop manipulator does a "thread the needle" task, but the BC-learned policy jitters and often misses. How do you fix it?</summary>

**Full reasoning chain**:

1. **Root cause**: teleoperation data intrinsically contains high-frequency noise (human hand jitter, hesitation pauses). Traditional single-step BC predicts only the next action; micro-errors are geometrically amplified → precise tasks like threading a needle collapse
2. **Solution: Action Chunking Transformer (ACT)** (ALOHA paper core):
   - **Predict a future $k$-step action chunk** in one shot (typically $k = 100$ timesteps ≈ 1 s)
   - Input: current observation (multi-camera RGB + proprioception)
   - Output: future k-step action sequence
3. **Temporal Ensemble**: at deployment, exponentially weighted moving average over **all overlapping chunks**
   - Action at time $t$ is the weighted average of predictions for $t$ made by all past chunks
   - This is a temporal low-pass filter; jitter is thoroughly smoothed
4. **Why it works**: threading a needle demands high temporal continuity; Temporal Ensemble smooths jitter in time and outperforms naive action smoothing (it preserves dynamic patterns inside chunks)
5. **Advanced recipe**: VLA (OpenVLA / π0) emits continuous spatial waypoints directly; an impedance controller downstream digests residuals — the "Diffusion Model imagines expert trajectories → MPC digests" Symbol → Neural modern architecture
6. **Trap to avoid**: do not use L2 loss to force the policy to learn the "mean action" — you will get a muddy intermediate solution. Use an MDN (Mixture Density Network) or Diffusion Head so the policy emits a multi-mode distribution

**Conclusion**: ACT + Temporal Ensemble is the dexterous manipulation workhorse, and the only working answer for threading needles, cracking eggs, and similar fine tasks.

</details>

<details>
<summary>Q7: The offline trajectory is perfect, but the real robot falls within a second. Why, and how do you save it?</summary>

**Full reasoning chain**:

1. **Root cause**: TrajOpt gives **open-loop feedforward**. The solver squeezes every drop of the model, and the solution sits **right on the friction-cone edge**. Real-world errors (mass / inertia / friction) + disturbances (wind, uneven ground) + state drift → butterfly effect → fall
2. **TVLQR bridging solution**:
   - Nominal trajectory $\bar{x}(t), \bar{u}(t)$ as reference
   - Linearize along the trajectory: $\delta x_{k+1} \approx A_k \delta x_k + B_k \delta u_k$
   - Backward Riccati → time-varying feedback gain $K(t)$
   - Hardware control law: $u(t) = \bar{u}(t) + K(t) \cdot [x(t) - \bar{x}(t)]$
3. **iLQR gives K for free**: if the solver was iLQR, the backward pass has already produced $K_k$; push it straight to hardware
4. **Bottom-layer WBC architecture**: industry iron rule of Atlas / ANYmal — feed TrajOpt's $\bar{x}, \dot{\bar{x}}, \ddot{\bar{x}}$ as reference to a 1 kHz WBC; the WBC's PD impedance + dynamics feedforward absorbs unmodeled disturbances
5. **Funnel Control (graduate-level)**: use SOS optimization to compute a Lyapunov invariant tube, with theoretical guarantee that disturbances inside the tube boundary do not produce a fall
6. **Modern DAgger + Distillation**: offline generate **tens of thousands of perfect trajectories under different disturbances** to train a NN Student policy; deploy at O(1) speed with preserved TrajOpt optimality
7. **Trap to avoid**: do not just crank up PD gains and "muscle through" disturbances — you will excite structural resonance and overload motors

**Conclusion**: TrajOpt (feedforward) + TVLQR/WBC (closed-loop feedback) is the standard industrial two-layer architecture.

</details>

## Interview Angles

1. **Precise distinction between Path and Trajectory** — the most basic concept, and the most frequently conflated. **Why it matters**: the interviewer judges immediately whether you "read a textbook" or "really get it". Two-minute version: "A path is pure geometry without time; a trajectory is a path plus time parameterization, producing $q(t), \dot{q}(t), \ddot{q}(t)$. Trajectory optimization is finding the best time allocation under physical constraints."

2. **Pareto trade-off: time vs energy vs smoothness** — show you grasp multi-objective optimization. **Why it matters**: proves you are not naively chasing a single metric. Two-minute version: "Time-optimal wants bang-bang, energy-optimal wants slow, smoothness-optimal crushes jerk — the three are mutually exclusive on a Pareto front. In practice, compromise with weighted or constrained objectives."

3. **Why quintic polynomials are the industrial PTP gold standard** — **why it matters**: being able to explain "why not cubic" separates "formula memorizer" from "physics understander". Two-minute version: "Cubic has discontinuous acceleration → jerk step → mechanical shock. Quintic adds acceleration boundary conditions, giving $C^2$ continuity + bounded jerk → no torque step → PTP gold standard. The analytic solution of jerk-minimization is exactly quintic."

4. **Runge phenomenon → B-spline piecewise low-order** — **why it matters**: interviewers love to ask "how do you interpolate 10 via-points"; anyone who answers "use a 9th-order polynomial" is out. Two-minute version: "High-order global polynomials oscillate violently at the edges (Runge), so you must use a B-spline with piecewise low-order segments + $C^2$ continuity."

5. **Differential Flatness dimensional hammer** — **why it matters**: must-have for quadrotor interviews, showing depth of understanding of underactuated systems. Two-minute version: "The quadrotor has 12 states and 4 inputs; Mellinger 2011 proved that $[x, y, z, \psi]$ are flat outputs from which all states can be algebraically recovered. The planner only draws a 3D smooth curve and dynamics are automatically satisfied — the soul of Fast-Planner."

6. **Min-snap corresponds to motor thrust rate of change** — **why it matters**: reciting the chain $p \to \ddot{p} \to \dddot{p} \to p^{(4)}$ distinguishes "understands quadrotor dynamics" from "just solves QPs".

7. **TOPP-RA $u = \ddot{s}, v = \dot{s}^2$ linearization** — **why it matters**: explaining why nonlinear dynamics become linear proves "solid math fundamentals". Two-minute version: "Reparameterize along the path, set $u = \ddot{s}, v = \dot{s}^2$; dynamics become $\tau = A(s)u + B(s)v + C(s)$, linear. Slice trajectory into N segments, forward/backward sweep to solve a sequence of LPs, under 1 ms."

8. **Bang-Bang principle** — **why it matters**: the physical essence of time-optimality; the standard answer to "how do you squeeze hardware limits". Two-minute version: "Pontryagin's minimum principle proves time-optimal must be bang-bang; at every instant at least one joint torque is pinned at the limit — full acceleration, full cruise, full deceleration."

9. **KKT Lagrange multiplier = physical emergence of contact force** — **why it matters**: the mathematical magic of Contact-Implicit; distinguishes "solves TrajOpt" from "understands contact dynamics". Two-minute version: "In CITO, the KKT multiplier of the complementarity constraint $\phi(q) \geq 0, \lambda \geq 0, \phi \cdot \lambda = 0$ physically emerges as the contact force $\lambda$. The solver autonomously decides when to switch contact modes."

10. **Atlas backflip and manipulator-pushes-box share the same equation** — **why it matters**: the killer question showing systems-level perspective. Two-minute version: "Both are underactuated dynamics: you cannot directly control the manipulated object or the airborne CoM, only search for the optimal contact point and force $J^\top f_c$ to indirectly change state. Master Contact-Implicit and both legged + dexterous-hand domains open up."

11. **Shooting vs Collocation: why Atlas backflip needs Collocation** — **why it matters**: the watershed between "uses" and "understands". Two-minute version: "Shooting blows up on a tiny torque error, gradients diverge; Collocation lets the solver violate physics early, floating the geometry first and tightening dynamics later."

12. **iLQR gives K for free for closed loop** — **why it matters**: separates "uses iLQR" from "understands iLQR". The $K_k$ already computed in the Backward Pass plugs straight into hardware as a TVLQR closed loop.

13. **TrajOpt feedforward + WBC closed loop = Atlas industry iron rule** — **why it matters**: the standard answer for "open loop diverges". Two-minute version: "TrajOpt gives feedforward $\bar{x}, \bar{u}$; compute TVLQR $K_t$ along the trajectory for feedback; if a WBC exists below, pipe reference to a 1 kHz impedance controller that absorbs disturbances."

14. **ALOHA Temporal Ensemble as temporal low-pass** — **why it matters**: the smoothing magic of the dexterous-manipulation workhorse. Two-minute version: "ACT emits a k-step chunk at once, and at deployment does exponentially weighted averaging over overlapping chunks — a time-domain low-pass filter, the only solution for threading a needle or cracking an egg."

## Further Reading

- **Pham & Pham, "TOPP-RA: Time-Optimal Path Parameterization for Redundantly Actuated Robots" (2018)** — the original TOPP-RA paper; clean algorithm with open-source Python `toppra`, the industrial reference for time-optimal parameterization
- **Mellinger & Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors" (2011)** — the classic paper on quadrotor min-snap + Differential Flatness, defining the industry standard for post-2011 quadrotor trajectories
- **Schulman et al., "Finding Locally Optimal, Collision-Free Trajectories with Sequential Convex Optimization" (2013)** — the TrajOpt original paper, SQP + SDF convexification, one of the default MoveIt 2 pipelines
- **Posa, Kuindersma & Tedrake, "Optimization and stabilization of trajectories for constrained dynamical systems" (2016)** — the foundational paper on Contact-Implicit TrajOpt and MPCC relaxation
- **Todorov & Li, "A generalized iterative LQG method for locally-optimal feedback control of constrained nonlinear stochastic systems"** — one of the foundational iLQR papers in MPC / closed-loop scenarios
- **Kuindersma et al., "Optimization-based locomotion planning, estimation, and control design for Atlas" (2016)** — the full public description of Boston Dynamics Atlas' trajectory optimization system
- **Flash & Hogan, "The Coordination of Arm Movements" (1985)** — the source paper for the minimum-jerk hypothesis, explaining why human arm reaching naturally follows a quintic curve
- **Zhao et al., "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware" (ALOHA, 2023)** — the ACT + Temporal Ensemble original paper, SOTA dexterous manipulation
- **Janner et al., "Planning with Diffusion for Flexible Behavior Synthesis" (2022)** — the seminal work on using Diffusion Models for trajectory generation
- **Lynch & Park, *Modern Robotics* Ch9 Trajectory Generation** — a systematic textbook on polynomials, trapezoidal velocity, S-curves; full MIT OCW video coverage
- **Tedrake, *Underactuated Robotics* (MIT 6.832) Ch10-12** — the academic authority on iLQR, Collocation, and SOS Funnel; free online
- **`toppra` Python package (GitHub: hungpham2511/toppra)** — the official open-source TOPP-RA implementation, usable directly in ROS 2 + MoveIt
- **`drake` (Robotics Toolbox, Russ Tedrake)** — one-stop toolbox for MathematicalProgram + Contact-Implicit + iLQR + Collocation
- **`crocoddyl` (INRIA / Pinocchio)** — high-performance C++ DDP / FDDP implementation used by ANYmal quadrupeds
