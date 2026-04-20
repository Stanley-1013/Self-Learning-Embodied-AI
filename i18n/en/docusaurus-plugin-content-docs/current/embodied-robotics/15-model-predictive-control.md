---
title: "Model Predictive Control"
prerequisites: ["13-pid-control-tuning", "14-force-impedance-control", "11-trajectory-optimization"]
estimated_time: 75
difficulty: 5
tags: ["mpc", "nmpc", "control", "optimization", "real-time", "acados", "sqp", "rti", "srbd", "cbf", "tube-mpc", "robust-mpc", "stochastic-mpc", "distributed-mpc", "ci-mpc", "mppi", "diffusion-mpc", "lie-group", "offset-free", "event-triggered", "learning-mpc"]
sidebar_position: 15
---

# Model Predictive Control

## You Will Learn

- State in two sentences why MPC beats PID for trajectory tracking under hard constraints, and within 10 seconds decide the right variant (linear QP / NMPC / Tube / Stochastic / Distributed / Sampling / Contact-Implicit) by probing the system (fully vs under-actuated, rigid vs contact-rich, single vs multi-agent, deterministic vs disturbed)
- When you see field symptoms like "quadruped oscillates when turning", "sim-perfect but hardware chatters", "NMPC solver timeouts at 1 kHz", "infeasible solution on impact", "Franka slams into a moving human", "AGV fleet deadlocks", or "quadruped feet choose wrong contact mode", you can immediately name the disease (linearization breakdown, missing delay compensation, unused RTI + warm start, no slack fallback, no CBF safety filter, centralized vs ADMM-distributed, hand-coded mode sequence) and the right remedy
- In an interview covering SRBD + WBC layering, SQP_RTI vs IPOPT, Multiple Shooting vs Condensing, Feasibility & Slack Fallback, GP-MPC / Meta-MPC / Amortized MPC + CBF Safety Filter, Tube vs Stochastic vs Min-Max, Economic MPC, Distributed MPC with ADMM, Contact-Implicit MPC with MPCC relaxation, Diffusion-MPC, Foundation Model + MPC, Lie Group SE(3) MPC, Offset-free integral augmentation, Event / Self-triggered MPC -- you can deliver the key reasoning for each in under two minutes
- Master the real industrial MPC pipeline: problem framing --> model reduction (SRBD / centroidal / bicycle) --> solver selection (acados / CasADi / OCS2 / mjpc) --> real-time techniques (RTI + warm start + HPIPM) --> robustness layer (Tube / CBF / Slack) --> online adaptation (RLS / GP / Meta) --> fallback state machine --> ISO functional-safety certification

## Core Concepts

**Precise definition**: **Model Predictive Control (MPC)** is an online optimization-based control strategy. At each control cycle it uses a **prediction model** to project the system $N$ steps forward, minimizes a **cost function** (tracking error + control effort + economic cost) subject to **constraints** (torque limits, joint bounds, friction cones, collision, chance / CBF / terminal), and executes **only the first control action**. The next cycle re-solves from the latest measurement. This **receding-horizon** mechanism gives MPC inherent look-ahead and by-construction constraint satisfaction.

**The three pillars of MPC**:
1. **Prediction model** -- discretized dynamics $x_{k+1} = f(x_k, u_k)$, linear (QP) or nonlinear (NLP), with explicit Euler / RK4 / Multiple Shooting integration
2. **Cost function** -- weighted sum of state tracking error, control effort, and (for Economic MPC) direct economic cost (energy / cycle time / wear)
3. **Constraints** -- hard physical limits (torque, velocity, friction cone), hard safety (CBF, collision), soft safety (Slack + L1 / L2 penalty), terminal (for stability)

**MPC vs PID**: PID is **reactive** -- corrects only after seeing error, has no concept of the future, cannot enforce constraints (anti-windup is a band-aid). MPC is **predictive** -- selects the optimal $N$-step strategy under constraints, handling hard limits by construction. PID is **error-driven** (model-free recovery works even with bad model); MPC is **model-driven** (bad model $\Rightarrow$ bad prediction $\Rightarrow$ catastrophically wrong open-loop command). This is why MPC needs online model adaptation far more than PID.

**MPC vs LQR**: LQR is a **special case of MPC** (infinite horizon + no constraints + linear model). LQR's feedback $u = -Kx$ is a single offline matrix; online cost is one matrix-vector multiply. The moment you have hard actuator limits, LQR can only clamp externally (no longer optimal) -- MPC handles constraints **inside** the optimization.

**MPC vs Ch11 offline trajectory optimization**: Ch11 plans the full trajectory offline with no feedback. MPC re-plans online every tick -- a small trajectory optimization each cycle -- so it naturally compensates for model error and disturbances via continuous measurement feedback.

**Location in the sense --> plan --> control loop**:
- **Input**: current state estimate $x_k$ (from Ch07 Kalman / EKF / VIO state estimator), reference trajectory $x^{\text{ref}}_{k:k+N}$ (from Ch08 / Ch11 planner or from a high-level VLA policy), contact schedule (for legged locomotion), disturbance estimate $\hat{d}_k$ (from DOB / Luenberger)
- **Output**: optimal control action $u_k^*$ (torque, velocity, acceleration, or ground reaction force) to the low-level driver / PID / WBC
- **Downstream**: actuator executes $\Rightarrow$ state updates $\Rightarrow$ sensors feed back $\Rightarrow$ next MPC solve
- **Loop node**: MPC sits in the **control layer** but doubles as online re-planning, bridging plan and control. In the embodied-AI stack it is the **cerebellum (100 Hz -- 1 kHz)** between a slow "brain" (VLA / LLM at 1-10 Hz) and a hard "spinal reflex" (WBC / impedance at 1 kHz)

**One-line version**: "MPC is the robot's anticipatory brain for the next $N$ steps -- it simulates several scenarios, picks the one that tracks best without hitting any physical wall, executes one step, then re-thinks. It is the only control paradigm that turns hard constraints (torque, friction cone, collision) into mathematical guarantees instead of after-the-fact clipping."

### Minimum Sufficient Math

**1. Linear MPC as a QP** (the industrial workhorse):

$$
\min_{U} \sum_{k=0}^{N-1} \left[ (x_k - x_k^{\text{ref}})^T Q (x_k - x_k^{\text{ref}}) + u_k^T R\, u_k \right] + (x_N - x_N^{\text{ref}})^T Q_f (x_N - x_N^{\text{ref}})
$$

$$
\text{s.t.} \quad x_{k+1} = A x_k + B u_k, \quad u_{\min} \le u_k \le u_{\max}, \quad x_{\min} \le x_k \le x_{\max}
$$

**Physical meaning**: $Q$ penalizes tracking deviation (larger $\Rightarrow$ tighter tracking), $R$ penalizes control effort (larger $\Rightarrow$ gentler but looser), $Q_f$ is the terminal cost (guarantees finite-horizon stability when set to the Riccati solution -- equivalent to the infinite-horizon LQR tail). The whole problem is a **Quadratic Program (QP)** solvable in milliseconds by OSQP, qpOASES, HPIPM.

**2. Receding horizon principle** (what makes MPC MPC):

$$
u^*_{0:N-1} = \arg\min J(U) \quad \Rightarrow \quad \text{execute only } u^*_0,\; \text{discard } u^*_{1:N-1}
$$

**Physical meaning**: each cycle re-solves with the latest measurement, correcting for model mismatch. This is why MPC has inherent closed-loop robustness -- even with an imperfect model, continuous feedback correction suppresses drift. The trick is that we waste the tail of the solution, but gain feedback.

**3. NMPC (Nonlinear MPC)** -- the full nonlinear program:

$$
\min_{U, X} \sum_{k=0}^{N-1} \ell(x_k, u_k) + \ell_N(x_N) \quad \text{s.t.} \quad x_{k+1} = f(x_k, u_k),\; g(x_k, u_k) \le 0
$$

**Physical meaning**: $f$ is the full nonlinear dynamics (contains $\sin, \cos$, quadratic Coriolis, rigid-body inertia), $\ell$ can be any smooth cost, $g$ can be nonlinear safety (collision, friction cone). Solve as an NLP via SQP (Sequential Quadratic Programming) or IPOPT (Interior Point). More accurate but computationally heavier.

**4. Soft constraint via slack** (the industrial safety net):

$$
g(x_k, u_k) \le \varepsilon_k, \quad \varepsilon_k \ge 0, \quad J \leftarrow J + \lambda_1 \|\varepsilon\|_1 + \lambda_2 \|\varepsilon\|_2^2
$$

**Physical meaning**: hard constraint $g \le 0$ can become **infeasible** under disturbance (robot kicked, obstacle pops up). Slack $\varepsilon \ge 0$ allows tiny violation when no solution exists, penalized by very large $\lambda$ so normally $\varepsilon = 0$. Industrial MPC spends **70% of engineering effort on slack + fallback state machines**, because "solver returns infeasible" on the factory floor is the difference between a red-screen warning and motors exploding.

**5. Control Barrier Function (CBF) safety filter**:

$$
\dot{h}(x) + \alpha \cdot h(x) \ge 0, \quad h(x) \ge 0 \text{ defines the safe set}
$$

**Physical meaning**: $h(x) = $ signed distance to danger (distance to wall, to human, to joint limit). The CBF inequality says "as you approach the boundary, you **must** decelerate with rate $\alpha$" -- it defines a **forward-invariant set**: once inside the safe set, never leave. Plugged into MPC as a constraint, it gives the solver a non-negotiable "do not cross the boundary" tether that terminal constraints cannot give (terminal only guards the last step, the middle can crash).

**Relative-degree caveat**: the first-order form above only works when $h$ has relative degree 1 (i.e. $\dot{h}$ contains $u$ directly). For position-level $h$ on second-order systems (acceleration-controlled manipulators, quadrotors), the first-order CBF constraint does **not** contain $u$ and is vacuous -- use **HOCBF** (Xiao & Belta 2019), which recursively applies class-$\mathcal{K}$ functions $\alpha_i$ to higher-order derivatives of $h$.

**6. Chance constraint** (probabilistic safety for Stochastic MPC / GP-MPC):

$$
\Pr\!\big(g(x_k) \le 0\big) \ge 1 - \varepsilon \quad \Rightarrow \quad g(\mu_k) + 3\sigma_k \le 0
$$

**Physical meaning**: when a GP predicts dynamics residual, it outputs mean $\mu$ and variance $\sigma^2$. Rather than demanding absolute safety, demand "safe with 99.7% probability" -- which, under Gaussian assumption, tightens the constraint by $3\sigma$. The beauty: **when the model is uncertain ($\sigma$ large), the robot automatically drives more conservatively; when the model is sharp, it drives aggressively**. The bridge between control and learning.

**7. Contact-Implicit MPC complementarity** (the 2024 manipulation game-changer):

$$
\phi(q) \ge 0, \quad \lambda \ge 0, \quad \phi(q) \cdot \lambda = 0
$$

**Physical meaning**: $\phi(q)$ = signed distance to contact surface, $\lambda$ = contact normal force. Both are **decision variables**, not pre-scheduled constants. The complementarity says: either no contact ($\phi > 0, \lambda = 0$) or in contact ($\phi = 0, \lambda > 0$) -- never both. The optimizer **autonomously discovers when to make and break contact** without a hand-coded mode sequence. This is how manipulation sequences (push, grasp, pivot) can emerge from optimization alone.

**8. Offset-free disturbance augmentation** (industrial must-have):

$$
\begin{bmatrix} x_{k+1} \\ d_{k+1} \end{bmatrix} = \begin{bmatrix} A & B_d \\ 0 & I \end{bmatrix} \begin{bmatrix} x_k \\ d_k \end{bmatrix} + \begin{bmatrix} B \\ 0 \end{bmatrix} u_k, \quad y_k = C x_k
$$

**Physical meaning**: introduce an "integrating disturbance" state $d_k$ that absorbs persistent model bias (gravity drift, wind, friction offset). A Kalman / Luenberger observer estimates $\hat{d}_k$ from residuals; MPC's target calculator uses $\hat{d}_k$ to **pre-compute feedforward** $u$ that exactly cancels the disturbance at steady state $\Rightarrow$ zero offset. Without this, nominal MPC has steady-state error just like a PD (no I) -- it's the "I-term" of MPC. This is the **state-disturbance** canonical form; the equivalent **output-disturbance** form is $x_{k+1} = Ax_k + Bu_k, \; y_k = Cx_k + d_k$. Pick one — **do not mix $B_d d_k$ into $x$ and $d_k$ into $y$ simultaneously** (over-parameterized and typically unobservable).

**9. Lie Group error** (for SO(3) / SE(3) states):

$$
e_R = \log(R^T_\text{ref} R)^\vee \in \mathbb{R}^3, \quad R_{k+1} = R_k \cdot \exp(\hat{\omega} \cdot dt)
$$

**Physical meaning**: orientation lives on the Lie group $SO(3)$, not in $\mathbb{R}^3$. Using Euler angles suffers gimbal lock at $\pm 90^\circ$; using quaternions forces a nonconvex unit-norm constraint $\|q\|^2 = 1$ that zig-zags SQP. The **logarithmic map** produces a clean 3-vector "how much to rotate to align" that SQP linearizes cleanly on the tangent space (Lie algebra $\mathfrak{so}(3) \cong \mathbb{R}^3$). Mandatory for quadrupeds, drones, and any floating-base robot.

**10. MPPI (sampling-based, gradient-free)**:

$$
w_i \propto \exp\!\left(-\frac{S(\tau_i)}{\lambda}\right), \quad u^* = \sum_i w_i \cdot u_i
$$

**Physical meaning**: sample tens of thousands of random control trajectories in parallel (GPU), roll each out, compute its cost $S$, then softmax-weighted average. $\lambda$ is the temperature (exploration vs exploitation). Crucial advantage: **no gradient needed** -- non-smooth contacts, discrete actions, and non-convex costs all work. The price is the curse of dimensionality: fails above ~20 control dimensions unless paired with a reduced model.

<details>
<summary>Deep dive: from QP to KKT -- matrix expansion, solver landscape, and stability guarantees</summary>

### Linear MPC QP matrix expansion

Unrolling $x_{k+1} = Ax_k + Bu_k$ over $N$ steps expresses all future states as an affine function of the initial state $x_0$ and the control sequence $U = [u_0, \dots, u_{N-1}]^T$:

$$
\mathbf{X} = \mathcal{A} x_0 + \mathcal{B} U
$$

where $\mathcal{A} = [A; A^2; \dots; A^N]$ stacks powers of $A$, and $\mathcal{B}$ is a lower-triangular Toeplitz matrix of $A$ and $B$. Substituting into the cost yields the standard QP:

$$
\min_U \tfrac{1}{2} U^T H U + g(x_0)^T U \quad \text{s.t.} \quad C U \le d(x_0)
$$

$H = \mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R} \succ 0$ (given $Q \succeq 0, R \succ 0$), guaranteeing a unique global solution. This is called the **condensed** form.

### Solver landscape

| Solver | Method | Typical speed | Best for |
|--------|--------|---------------|----------|
| OSQP | ADMM | < 1 ms (medium) | Embedded, quadrupeds (warm-start friendly) |
| qpOASES | Active-set | < 0.5 ms (small) | Hard real-time aerospace (explicit active set) |
| HPIPM | Structure-exploiting IP | < 1 ms (NMPC) | acados backend, banded sparsity |
| ECOS / Gurobi | Interior-point | 1-10 ms | SOCP cone constraints (friction cone) |
| IPOPT | Interior-point NLP | 10-100 ms | Offline NMPC, no warm-start |
| acados SQP_RTI | Single SQP + HPIPM | 0.5-5 ms | High-speed NMPC (Cheetah, ANYmal) |
| mjpc / MPPI | Sampling (GPU) | 5-20 ms | Contact-rich, non-smooth |

### Condensing vs Sparse formulation

- **Sparse (Multiple Shooting)**: keep both $x$ and $u$ as decision variables, add equality constraint $x_{k+1} = f(x_k, u_k)$ as a block. KKT matrix is banded (tridiagonal-block). HPIPM exploits this. Favored when $N$ is large or the dynamics are unstable (error accumulation limited to one shoot interval).
- **Condensed (Single Shooting)**: eliminate $x$ using the dynamics, keep only $U$. Leads to a dense QP. Favored when state dimension $n_x$ is much larger than $N \cdot n_u$. Unstable for open-loop-unstable systems: rollout error compounds exponentially.
- **Partial condensing** (acados default): group every few steps, getting sparsity + dimension balance.

### Stability guarantees

Closed-loop stability of linear MPC typically requires one of:
- **Terminal cost** $Q_f = P_\infty$ (solution to discrete Riccati): makes the cost function a Lyapunov function, guaranteeing decrease.
- **Terminal constraint** $x_N \in \mathcal{X}_f$ where $\mathcal{X}_f$ is a **positively invariant set** under some local controller.
- In practice, a sufficiently long $N$ (covering the settling time) + generous $Q_f$ usually suffices without a strict terminal constraint; this is the **"long horizon + end cost"** industrial pragma.

For NMPC, stability analysis uses contractive constraints or quasi-infinite horizon techniques (Chen-Allgower 1998). Formal proof is hard; most practitioners verify stability empirically with closed-loop sim.

</details>

<details>
<summary>Deep dive: RTI (Real-Time Iteration) -- the "one SQP step" trick that makes NMPC fit 1 kHz</summary>

### The problem

Standard NMPC via IPOPT solves an NLP in 10-100 ms, far too slow for a 1 kHz (1 ms) control loop. SQP converges in 3-10 iterations, each iteration requires a QP solve + dynamics linearization, total 10 ms easily.

### The insight

Between consecutive control cycles the problem barely changes (dt = 1 ms, state change is small). **Running SQP to convergence each cycle is wasteful**: the first SQP iteration already reduces the error by 90%, and the previous solution is an excellent initial guess.

### RTI algorithm (Diehl 2005)

Per control cycle:
1. **Preparation phase** (can run in parallel with sensor wait):
   - Linearize the dynamics $f$ and the cost $\ell$ around the previous solution $(X^*_{\text{prev}}, U^*_{\text{prev}})$
   - Assemble the QP matrices $H, g, C, d$
2. **Feedback phase** (when the new state $x_0$ arrives):
   - Plug $x_0$ in, solve the QP **once** (no inner iteration)
   - Extract $u_0^*$, send to actuator
3. **Shift**: take the rest of the solution as the next warm start, shift one step forward, append $u_{\text{init}}$ at the tail.

Total online: one QP solve ($\sim 100\,\mu\text{s}$ with HPIPM on a modern CPU) + feedback delay of microseconds. **This is what makes NMPC fit a 1 kHz control loop on an embedded computer**.

### When RTI fails

- Problem is highly nonlinear and drifts fast between cycles (e.g. abrupt contact events): single iteration cannot catch up; closed-loop may destabilize.
- **Mitigation**: fall back to 2-3 SQP iterations per cycle (still cheap with HPIPM), or trigger a full re-solve on detected contact events.

### Multiple Shooting + RTI + HPIPM = industrial hard-real-time stack

- **Multiple Shooting**: stabilizes rollout error for open-loop-unstable systems (quadruped swing phase, drone hover).
- **RTI**: cuts compute by an order of magnitude.
- **HPIPM**: exploits banded sparsity, inner solver of acados.
- Combined: **ANYmal C** runs an 18-state, 12-input, N=10 NMPC at 400 Hz on an embedded Intel NUC.

</details>

<details>
<summary>Deep dive: SRBD reduction + WBC layering -- why quadrupeds use two MPCs instead of one</summary>

### The problem with full-body NMPC on quadrupeds

Full-body floating-base dynamics $M(q)\ddot{q} + C(q, \dot{q})\dot{q} + G(q) = S^T\tau + J_c^T\lambda$ has $\sim 18$ DoF for a quadruped. Exploiting sparsity still gives $O(n^3) \sim O(5000)$ FLOPs per NMPC iteration. Running 3-5 iterations $\times$ $N=15$ horizon $\times$ 100 Hz = hopeless on an embedded CPU.

### SRBD -- Single Rigid Body Dynamics

Lump all mass at the CoM:
$$
m \ddot{p}_c = \sum_i f_i - m g, \qquad I_c \dot{\omega} = \sum_i r_i \times f_i
$$
- State: CoM position + orientation + linear/angular velocity = 12 dims
- Control: 4 foot GRF vectors = 12 dims
- Constraint: each $f_i$ inside friction cone $\|f_{i,\text{tan}}\| \le \mu f_{i,\text{norm}}$
- **Linearized around nominal orientation**, this becomes a **convex QP** solvable in $\sim 1$ ms

This is the **MIT Cheetah "Convex MPC" trick** (Di Carlo et al. 2018). Assumptions: low angular momentum, small roll/pitch. Breaks down for acrobatic maneuvers (barrel rolls) -- then you need centroidal dynamics or full NMPC.

### WBC -- Whole-Body Control at 1 kHz

The SRBD MPC produces a **reference CoM trajectory + desired GRFs** at $\sim 100$ Hz. The **Whole-Body Controller** runs at 1 kHz with a single-step QP:
$$
\min_{\ddot{q}, \tau, \lambda} \sum_i w_i \|\ddot{x}_i - \ddot{x}_i^{\text{des}}\|^2 \quad \text{s.t.}\quad M\ddot{q} + h = S^T\tau + J_c^T\lambda, \text{ torque limits, friction cone}
$$
WBC enforces the full non-linear dynamics at every instant, handles joint limits, prevents singularities, and resolves redundancy via task priority.

### The "sees far / walks steady" division of labor

- **MPC (100 Hz, reduced model)**: sees ahead 1-2 s, decides **where to step, when to swing, how to shift CoM** to avoid obstacles and prepare for terrain. Fast because the model is coarse.
- **WBC (1 kHz, full dynamics)**: executes the current-instant MPC command with maximal fidelity to the physical robot. Slow per-step horizon but high fidelity.

If you collapse both into one controller you either lose look-ahead (fast but myopic) or lose real-time (accurate but 50 ms late). The two-layer stack is the standard for ANYmal, MIT Cheetah, Unitree Go1, Boston Dynamics Atlas. In interviews: **if you can draw this stack on a whiteboard in 30 seconds you pass the "understands legged control" bar.**

</details>

**Common APIs / toolchain**:

| Layer | Tool | Typical interface |
|-------|------|-------------------|
| QP solver (embedded) | OSQP | `osqp.solve()` -> $U^*$, warm-start friendly |
| QP solver (structure-exploiting) | HPIPM | acados backend, banded KKT |
| NLP solver (prototyping) | CasADi + IPOPT | `nlpsol('solver', 'ipopt', {...})` |
| NMPC framework (production) | **acados** | `AcadosOcpSolver.solve_for_x0(x0)` -- RTI per call |
| Legged MPC framework | **OCS2 (ETH)** | SLQ / DDP, switched contact dynamics |
| Sampling MPC | **MuJoCo MPC (mjpc)** | Predictive Sampling, iLQG, GPU rollouts |
| Convex quadruped MPC | MIT Cheetah Convex MPC | Linearized SRBD + friction QP at 1 kHz |
| Distributed MPC | ADMM + local solvers | Each agent solves local QP, broadcasts trajectory |
| ROS 2 integration | `ros2_control` + MPC plugin | `update()` returns `JointTrajectory` |
| Lie Group math | `manif`, `Sophus`, Drake | Clean exp / log on SO(3), SE(3) |

## Intuition

**Analogy: driving while looking ahead 200 m**. PID is like correcting the wheel only after noticing you have drifted -- reactive. MPC is like an experienced driver scanning ahead, mentally simulating "if I press the throttle this much, where will I be in 3 s, will I exceed the speed limit?" and picking the best action. Every second the driver re-simulates because the road (state feedback) changes. LQR is like a driver with one fixed steering-response pattern baked in at the factory -- works on a straight freeway but never adapts to cones or cops.

**Visual metaphor: Go AI's look-ahead search**. MPC's horizon $N$ is like AlphaGo searching $N$ moves ahead; deeper search means smarter play (better tracking) but more compute (longer solve time). The cost function is the board evaluator, constraints are the rules. Receding horizon is re-searching every move -- because the opponent (disturbance) surprises you. Sampling MPC (MPPI) is like Monte Carlo tree search vs gradient-based minimax: MPPI rolls out thousands of random scenarios in parallel and picks the best weighted average; NMPC gradient-descends along one nominal trajectory.

**Metaphor for robust MPC**: imagine driving in heavy fog.
- **Nominal MPC**: assumes perfect visibility -- crashes.
- **Tube MPC**: drives the center of a wide lane, reserving half-lane margin; guaranteed to stay in the safe lane no matter how the fog pushes you.
- **Min-Max MPC**: assumes the worst possible gust and plans for it -- drives at 5 km/h everywhere, absurdly conservative.
- **Stochastic MPC**: knows the fog follows a distribution; drives to keep collision probability below 0.3%; $3\sigma$-safe, not absolute-safe.

**Metaphor for Contact-Implicit MPC**: imagine picking up a slippery jar.
- **Hand-coded FSM**: "reach, align, close fingers, lift" -- fails if the jar is at an odd angle.
- **CI-MPC**: "minimize the cost of delivering the jar to my mouth, subject to physics" -- optimizer discovers on its own when to push, when to regrasp, when to pivot against the table edge to reorient. The mode sequence **emerges** from optimization.

**Simulator observations** (MuJoCo / Isaac Sim / Gazebo):
- Sweep MPC horizon $N = 5 \to 30$ on a quadruped: $N$ too short $\Rightarrow$ walks unsteadily, cannot prepare turns. $N$ too long $\Rightarrow$ solve overruns the control period, frequency drops, oscillates.
- Drop $u_{\max}$ from 33 Nm to 10 Nm: MPC automatically slows down; PID with the same command saturates and crashes.
- Kick a quadruped mid-stride: nominal MPC slips. Tube MPC with pre-tightened constraints stays upright.
- Place a quadruped on a 0.3-friction patch and set the MPC $\mu$ to 0.6: feet slip. Drop $\mu$ to 0.2: MPC reduces tangential force $\Rightarrow$ stable gait. This is constraint-driven behavior change without any controller re-tuning.

## Implementation Link

**Five representative engineering scenarios** (pick the one closest to your use case, bring the rest as interview talking points):

1. **Quadruped locomotion (SRBD + WBC layered)**: MIT Cheetah 3 / ANYmal / Unitree Go1. Simplify 18-DoF dynamics to a 6-DoF centroidal model via SRBD, solve a QP at 100 Hz under friction cone $\|f_t\| \le \mu f_n$ to produce CoM trajectory + GRFs. WBC at 1 kHz maps to joint torques and resolves redundancy. ANYmal uses OCS2 (SLQ-based); Cheetah uses custom convex MPC.

2. **Manipulator real-time trajectory tracking**: MoveIt generates a nominal trajectory; MPC replaces PID as a tracker that handles torque limits, joint bounds, and online collision updates. acados + operational-space linearization hits $< 1$ ms per solve. Franka uses this for adaptive pick-and-place; Tesla Optimus supposedly uses NMPC-WBC layering.

3. **Autonomous vehicle path following**: bicycle model $\dot{x} = v\cos\theta, \dot{y} = v\sin\theta, \dot{\theta} = v\tan\delta/L$ (nonlinear). NMPC minimizes lateral + heading error under lane boundaries + speed limits + passenger comfort (jerk). L4 production uses **Tube MPC** for hard pedestrian avoidance. CasADi + IPOPT is academic; acados is production.

4. **Drone acrobatic flight / racing**: PX4 / Agilicious. NMPC on a 13-state quaternion model runs at 100 Hz on an NVIDIA Jetson. The key is **SO(3) Lie group** treatment of orientation and **thrust-axis decoupling**. Can achieve 60 km/h through a slalom course.

5. **Multi-agent warehouse AGV fleet**: Amazon / Geek+. 100+ AGVs cannot be centrally optimized (matrix dimension explodes). **Distributed MPC with ADMM** -- each AGV solves a small local MPC and broadcasts its planned trajectory; neighbors penalize overlap, iterate 2-3 times per cycle to converge to a collision-free consensus.

**Code skeleton** (Python, acados + CasADi NMPC, quadruped-style):

```python
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np

# ---------- 1. Build symbolic SRBD model ----------
p = ca.SX.sym('p', 3)          # CoM position
R_vec = ca.SX.sym('R', 9)      # flattened rotation (SO(3))
v = ca.SX.sym('v', 3)          # linear velocity
w = ca.SX.sym('w', 3)          # angular velocity
x = ca.vertcat(p, R_vec, v, w) # state: 18

f_grf = ca.SX.sym('f', 12)     # 4 foot GRFs (3D each)
u = f_grf                      # control: 12

m, g = 12.0, 9.81
I_body = np.diag([0.1, 0.3, 0.3])
# dynamics (simplified, linearized around nominal R_ref)
# ... skeleton only; real implementation unrolls SE(3) kinematics
x_dot = ca.vertcat(v, ca.SX.zeros(9), ...)  # to be filled in

model = AcadosModel()
model.x = x; model.u = u; model.f_expl_expr = x_dot
model.name = 'quadruped_srbd'

# ---------- 2. Build OCP with soft constraints ----------
ocp = AcadosOcp(); ocp.model = model
N, dt = 15, 0.02
ocp.dims.N = N; ocp.solver_options.tf = N * dt

# Cost: tracking + control regularization
Q_diag = np.array([100, 100, 200,  10, 10, 10,  10, 10, 10,  1, 1, 1,  1, 1, 1,  1, 1, 1])
R_diag = np.ones(12) * 0.001
ocp.cost.cost_type = 'LINEAR_LS'
ocp.cost.W = np.diag(np.concatenate([Q_diag, R_diag]))

# Friction cone + unilateral contact (soft for feasibility recovery)
ocp.constraints.lbu = np.tile([-1e3, -1e3, 0], 4)    # f_z >= 0
ocp.constraints.ubu = np.tile([ 1e3,  1e3, 500], 4)  # f_z <= 500 N
# Slack + penalty for feasibility
ocp.cost.zl = 1e5 * np.ones(12)   # L1 lower slack
ocp.cost.Zl = 1e3 * np.ones(12)   # L2 slack
ocp.constraints.idxsh = np.arange(12)

# ---------- 3. Real-time mode ----------
ocp.solver_options.nlp_solver_type = 'SQP_RTI'    # <- key for 100+ Hz
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
solver = AcadosOcpSolver(ocp)

# ---------- 4. Online control loop ----------
def mpc_step(x0, x_ref_seq):
    solver.set(0, 'lbx', x0); solver.set(0, 'ubx', x0)   # fix initial state
    for k in range(N):
        solver.set(k, 'yref', np.concatenate([x_ref_seq[k], np.zeros(12)]))
    status = solver.solve()
    if status != 0:
        return fallback_controller(x0)                    # safe mode
    return solver.get(0, 'u')                             # first action only
```

<details>
<summary>Deep dive: complete runnable linear MPC (Python + OSQP) with warm-start timing</summary>

```python
import numpy as np
import osqp
from scipy import sparse
import time

def build_linear_mpc(A, B, Q, R, Qf, N, x_min, x_max, u_min, u_max):
    """Build OSQP problem once; online only q + initial-state bounds update."""
    nx, nu = B.shape
    nz = (nx + nu) * N + nx

    # Cost: H = blkdiag(Q, R, Q, R, ..., Qf)
    P_blocks = []
    for k in range(N):
        P_blocks.append(sparse.block_diag([Q, R], format='csc'))
    P_blocks.append(sparse.csc_matrix(Qf))
    P_cost = sparse.block_diag(P_blocks, format='csc')

    # Dynamics equality: x_{k+1} - A x_k - B u_k = 0
    Ax_list = []
    for k in range(N):
        row = np.zeros((nx, nz))
        idx_xk = k * (nx + nu); idx_uk = idx_xk + nx; idx_xk1 = (k + 1) * (nx + nu)
        row[:, idx_xk:idx_xk + nx] = A
        row[:, idx_uk:idx_uk + nu] = B
        row[:, idx_xk1:idx_xk1 + nx] = -np.eye(nx)
        Ax_list.append(row)
    Aeq = sparse.csc_matrix(np.vstack(Ax_list))

    # Initial state equality (updated online): x_0 = x_current
    A_init = np.zeros((nx, nz)); A_init[:, :nx] = np.eye(nx)

    # Box bounds
    lb = np.tile(np.concatenate([x_min, u_min]), N)
    lb = np.concatenate([lb, x_min])
    ub = np.tile(np.concatenate([x_max, u_max]), N)
    ub = np.concatenate([ub, x_max])

    A_total = sparse.vstack([Aeq, sparse.csc_matrix(A_init), sparse.eye(nz)], format='csc')
    l_total = np.concatenate([np.zeros(N * nx), np.zeros(nx), lb])
    u_total = np.concatenate([np.zeros(N * nx), np.zeros(nx), ub])

    solver = osqp.OSQP()
    solver.setup(P_cost, np.zeros(nz), A_total, l_total, u_total,
                 warm_start=True, verbose=False,
                 eps_abs=1e-6, eps_rel=1e-6, max_iter=200)
    return solver, nz

def mpc_step(solver, x0, x_ref_seq, nx, nu, N, Q, Qf):
    nz = (nx + nu) * N + nx
    q = np.zeros(nz)
    for k in range(N):
        q[k * (nx + nu): k * (nx + nu) + nx] = -Q @ x_ref_seq[k]
    q[N * (nx + nu):] = -Qf @ x_ref_seq[N]

    # Update initial-state constraint l = u = x0
    l_init_idx_start = N * nx
    l_new = solver.constraints.l.copy() if hasattr(solver.constraints, 'l') else None
    # (OSQP API: use solver.update for bounds + q)
    solver.update(q=q)
    solver.update_bounds(l_new, l_new)  # pseudo-code
    t0 = time.perf_counter()
    result = solver.solve()
    solve_ms = (time.perf_counter() - t0) * 1e3
    if result.info.status != 'solved':
        return np.zeros(nu), solve_ms
    u0 = result.x[nx : nx + nu]
    return u0, solve_ms

# ---------- Example: double integrator at 200 Hz ----------
dt = 0.005
A = np.array([[1, dt], [0, 1]])
B = np.array([[0.5 * dt**2], [dt]])
Q = np.diag([50.0, 5.0])
R = np.array([[0.05]])
Qf = 10 * Q
N = 40                                   # 200 ms look-ahead

solver, _ = build_linear_mpc(
    A, B, Q, R, Qf, N,
    x_min=np.array([-10, -5]), x_max=np.array([10, 5]),
    u_min=np.array([-2.0]),   u_max=np.array([2.0]))

# Warm-started steady loop: measured 0.15-0.3 ms on a modern x86 laptop
# Cold start: 1-2 ms; second iteration onwards uses previous solution as init
```

**Key takeaways**:
- Build offline, update `q + bounds` online. Do not re-construct matrices each cycle.
- OSQP `warm_start=True` typically gives 2-5x speedup after the first solve.
- At 200 Hz on medium-size problems ($N=40$, $n_x=2$, $n_u=1$) OSQP solves in $< 0.3$ ms.
- For embedded MCU (STM32H7) use `osqp-embedded` or `qpOASES` with code generation.

</details>

<details>
<summary>Deep dive: Slack-Variable feasibility recovery and fallback state machine (production pattern)</summary>

Industrial MPC isn't "solve one QP, ship the result." It's a **state machine that gracefully degrades when the optimizer hiccups**. Here is a typical three-layer pattern used in factory arm MPCs.

```cpp
enum class MpcState {
    NORMAL,       // optimizer happy, slack = 0
    SOFT_VIOL,    // slack > 0 but bounded, constraint temporarily violated
    DEGRADED,     // solver infeasible, use shifted previous solution
    SAFE_MODE,    // solver failed 3+ cycles, switch to pure impedance hold
};

struct MpcOutput {
    Eigen::VectorXd u;
    MpcState state;
    double solve_ms;
};

MpcOutput mpc_step_robust(State x, Trajectory ref) {
    static Eigen::VectorXd u_prev_seq;   // rolling previous solution
    static int fail_streak = 0;

    // ---------- Layer 1: Normal solve with slack ----------
    // acados/OSQP: constraint g(x, u) <= eps, cost += 1e5 * |eps|_1 + 1e3 * |eps|^2
    auto result = solver.solve_with_slack(x, ref);

    if (result.status == SOLVED && result.slack.maxCoeff() < 1e-3) {
        fail_streak = 0;
        u_prev_seq = result.u_sequence;
        return {result.u_sequence[0], NORMAL, result.solve_ms};
    }

    // ---------- Layer 2: Solved but slack > threshold ----------
    if (result.status == SOLVED) {
        fail_streak = 0;
        u_prev_seq = result.u_sequence;
        log_warning("Slack violation: ", result.slack.maxCoeff());
        return {result.u_sequence[0], SOFT_VIOL, result.solve_ms};
    }

    // ---------- Layer 3: Infeasible, shift previous ----------
    fail_streak++;
    if (fail_streak < 3 && u_prev_seq.size() > 0) {
        Eigen::VectorXd u_shifted = shift(u_prev_seq, 1);
        return {u_shifted, DEGRADED, 0.0};
    }

    // ---------- Layer 4: SAFE MODE -- impedance hold ----------
    Eigen::VectorXd u_safe = safe_impedance_hold(x);
    return {u_safe, SAFE_MODE, 0.0};
}
```

**Principles**:
- **Slack penalty coefficients**: `zl = 1e5` (L1) and `Zl = 1e3` (L2) so slack is near-zero at normal operation but activates on disturbance.
- **Never return `nullptr` or `NaN`** from the control callback. Always hand the low-level driver **some** command, even if it's a safe impedance hold.
- **Log every state transition** -- in post-mortem analysis of industrial failures, the state transition trace tells you exactly when and why the MPC lost feasibility.
- **70/30 rule**: production MPC engineers spend 70% of development effort on **slack + fallback + logging**, 30% on the nominal QP tuning. In interviews, if you don't mention this, you haven't shipped MPC to a factory.

</details>

<details>
<summary>Deep dive: CBF Safety Filter as a post-MPC QP -- the industrial pattern for learning-based MPC</summary>

When a neural network approximates MPC (Amortized MPC), or when a VLA policy emits actions, you lose hard constraint guarantees. The **CBF Safety Filter** is a tiny QP that minimally projects the proposed action onto the safe set -- microseconds per solve.

```python
import cvxpy as cp
import numpy as np

def cbf_filter(u_nn, x_current, h_fn, grad_h_fn, f_fn, g_fn, u_min, u_max,
               alpha=5.0):
    """
    Minimally alter u_nn so the CBF constraint holds.
    h(x) >= 0 is the safe set (e.g. distance to obstacle).
    dh/dt = grad_h(x) @ (f(x) + g(x) @ u) >= -alpha * h(x)
    """
    h_val = h_fn(x_current)
    grad_h = grad_h_fn(x_current)
    f_val = f_fn(x_current)
    g_val = g_fn(x_current)

    u_var = cp.Variable(u_nn.shape)
    cost = cp.sum_squares(u_var - u_nn)      # stay close to NN suggestion
    cbf_lhs = grad_h @ (f_val + g_val @ u_var)
    constraints = [
        cbf_lhs >= -alpha * h_val,           # CBF invariance
        u_var >= u_min, u_var <= u_max,       # torque limits
    ]
    prob = cp.Problem(cp.Minimize(cost), constraints)
    prob.solve(solver=cp.OSQP, warm_start=True)
    if prob.status != cp.OPTIMAL:
        return safe_fallback(x_current)
    return u_var.value

# Usage in a VLA + MPC loop:
# u_proposed = vla_policy(image, language).action                # unsafe
# u_safe = cbf_filter(u_proposed, x_state, ...)                  # always safe
# send_to_actuator(u_safe)
```

**Why this matters**:
- **Foundation-model-driven robots cannot pass ISO 13849 / ISO 10218 functional-safety certification** if the neural policy can emit arbitrary torque. The CBF filter turns the stack into "probably-smart policy + provably-safe filter" -- the filter is the certifiable layer.
- The filter is just a tiny QP ($n_u$-dim variable, 2-10 inequalities). Solves in 50 $\mu$s on a modern CPU.
- If the filter modifies the action too often, it's a signal that the upstream policy is misbehaving -- log the "filter activation rate" as a monitoring metric.

This pattern is the heart of the **"Probably-Approximately-Correct (PAC) guarantee stack"** emerging in 2024 embodied AI: VLA for semantics, MPC/RL for tracking, CBF filter for physics-layer safety.

</details>

<details>
<summary>Deep dive: Tube MPC -- constraint tightening by a control-invariant set (the L4 self-driving standard)</summary>

### The idea

Nominal MPC plans an ideal trajectory $\bar{x}(t)$. In reality, bounded disturbance $w \in W$ (wind gust, road unevenness, unmodeled force) pushes the real state $x(t)$ off the nominal. Tube MPC guarantees:

$$
x(t) - \bar{x}(t) \in Z \quad \forall\, t, \forall\, w \in W
$$

where $Z$ is a **Robust Control Invariant (RCI) set** -- once you're inside $Z$, an ancillary controller $u = \bar{u} + K(x - \bar{x})$ keeps you inside $Z$ no matter the disturbance.

### The construction

1. **Compute $Z$** offline: $Z$ is the minimal RCI set under a chosen ancillary gain $K$. Computed via polytope iteration (solvers: MPT3 toolbox in Matlab).
2. **Tighten constraints**: original hard constraint $X$ becomes $X \ominus Z$ (Pontryagin / Minkowski difference) for the nominal planner.
3. **Nominal MPC** plans on the tightened constraints: $\bar{x} \in X \ominus Z$.
4. **Online**: ancillary controller applies $u = \bar{u} + K(x - \bar{x})$, fighting the disturbance.

**Guarantee**: the real trajectory $x$ is always inside $\bar{x} \oplus Z \subseteq X$, i.e. inside the original safe set.

### Why this is the L4 self-driving production standard

Waymo / Cruise face pedestrian-avoidance with hard safety (distance $> 1$ m). They cannot use stochastic MPC (99.7% safe is not enough when ISO 26262 demands absolute safety within spec). Tube MPC shrinks the lane by the tube radius, nominal plan stays in the shrunken lane, ancillary controller absorbs the gust -- **provably safe** for all bounded disturbance.

### Adaptive Tube MPC (2020+)

Classic Tube MPC uses a fixed $Z$, leading to conservatism (large $Z$ = tiny free lane). Adaptive Tube MPC combines:
- **Set-Membership identification**: narrow the disturbance bound $W$ online as data accumulates.
- **RLS parameter adaptation**: update model $\hat{A}, \hat{B}$ with forgetting factor.
- Tube shrinks as the robot "gets to know" its own dynamics $\Rightarrow$ initially conservative (wide tube), becomes aggressive (narrow tube) after warm-up.

**Behavioral signature**: ANYmal picks up an unknown box -- starts walking cautiously, after 30 s adapts and walks at full speed. Can be demoed in Isaac Sim.

</details>

<details>
<summary>Deep dive: MPPI -- when gradients fail, sample. JAX + GPU parallelism</summary>

### When to reach for MPPI (vs gradient NMPC)

- Non-smooth dynamics (discrete contact events, stick-slip friction, on-off actuators)
- Highly multi-modal cost (two valid paths around a tree -- gradient sticks at the saddle)
- Cost function is a black box (simulator, neural world model)
- You can afford a GPU and don't need < 1 ms solve

### Algorithm

At each control cycle:
1. Sample $K$ control trajectories: $u_i = u_\text{nominal} + \epsilon_i$, $\epsilon_i \sim \mathcal{N}(0, \Sigma)$.
2. **Parallel rollout** on GPU: compute $S(\tau_i)$ = total cost along each trajectory (requires a fast simulator, e.g. MuJoCo or differentiable dynamics).
3. Weighted softmax: $w_i = \exp(-S(\tau_i) / \lambda) / Z$.
4. New nominal: $u_\text{new} = \sum_i w_i u_i$.
5. Execute $u_\text{new}[0]$, shift the nominal for next cycle.

### JAX reference implementation

```python
import jax, jax.numpy as jnp
from jax import random, vmap

def mppi_step(state, u_nominal, key, rollout_fn, cost_fn,
              K=4096, horizon=30, std=0.3, lam=1.0):
    noise = random.normal(key, (K, horizon, u_nominal.shape[-1])) * std
    u_samples = u_nominal[None] + noise                      # (K, H, nu)
    trajectories = vmap(rollout_fn, in_axes=(None, 0))(state, u_samples)
    costs = vmap(cost_fn)(trajectories, u_samples)           # (K,)
    costs_shifted = costs - jnp.min(costs)                   # numerical stability
    weights = jnp.exp(-costs_shifted / lam)
    weights = weights / jnp.sum(weights)                     # (K,)
    delta = jnp.sum(weights[:, None, None] * noise, axis=0)
    u_new = u_nominal + delta
    return u_new

# With 4096 samples, horizon 30, nu = 12 (quadruped) -- runs at 100+ Hz
# on an RTX 3080 using jax.jit + vmap parallelism.
```

### Crucial tuning knobs

- **$\lambda$ (temperature)**: low $\lambda$ is nearly winner-take-all (exploitation, brittle); high $\lambda$ is averaging (exploration, smooth). Typical $0.1 \le \lambda \le 10$.
- **$\Sigma$ (sample covariance)**: too small $\Rightarrow$ miss good actions; too large $\Rightarrow$ most samples are useless. CEM (Cross Entropy Method) dynamically tightens $\Sigma$ around top-k elites.
- **$K$ (sample count)**: 1000 for low-dim ($n_u < 6$), 10k for quadruped ($n_u = 12$), 100k+ for humanoids. Curse of dimensionality kicks in hard above 20.

### Production examples

- **MuJoCo MPC (mjpc)** by DeepMind: open-source, Predictive Sampling + iLQG, ships with 10+ task demos. Best starting point for learning.
- **Tesla Optimus** (rumored): MPPI for whole-body manipulation.
- **Boston Dynamics Atlas**: sampling-based trajectory search for parkour.

### The killer demo

Put a tree in front of a quadruped. Gradient NMPC's Jacobian has a sharp saddle -- "left or right" choice kills convergence. MPPI samples 4000 random trajectories, half go left half go right, softmax picks the lower-cost side, robot flows smoothly around. **This is the non-differentiable multi-modal scenario where sampling is fundamentally superior.**

</details>

<details>
<summary>Deep dive: Contact-Implicit MPC -- MPCC relaxation, the 2024 manipulation breakthrough</summary>

### Why traditional MPC fails at contact-rich manipulation

A pre-defined **mode sequence** ("left foot lifts at 0.1 s, lands at 0.3 s; right foot lifts at 0.3 s...") forces the MPC to commit to a contact schedule. If the environment changes (terrain shifts, object slips), the schedule becomes wrong -- re-planning is expensive, and during the re-plan the robot executes stale commands.

For manipulation, the combinatorial explosion is worse: "grasp here or there, push now or later, pivot against the left edge or the right." Hand-coding the state machine covers 80% of cases and fails spectacularly on the long tail.

### The CI-MPC unification

Introduce contact distance $\phi(q) \ge 0$ and contact force $\lambda \ge 0$ as **decision variables**, coupled by the complementarity:

$$
\phi(q) \cdot \lambda = 0 \quad\text{(either no contact or on the surface, never both)}
$$

The optimizer now decides **when and where to contact** as part of the optimization. Grasping, pushing, pivoting, walking -- all emerge as trajectories through the combined continuous-discrete search space.

### The MPCC relaxation (Mathematical Program with Complementarity Constraints)

$\phi \cdot \lambda = 0$ is non-smooth and non-convex; the gradient is undefined at $\phi = \lambda = 0$, breaking SQP. The standard fix:

$$
\phi(q) \cdot \lambda \le \varepsilon
$$

Start with $\varepsilon = 10^{-1}$, solve, then anneal $\varepsilon \to 10^{-6}$ over iterations. This is the interior-point barrier analog for complementarity.

### Drake MPCC sketch

```python
from pydrake.solvers import MathematicalProgram, SnoptSolver

prog = MathematicalProgram()
q = prog.NewContinuousVariables(horizon, n_q, "q")
phi = prog.NewContinuousVariables(horizon, n_contacts, "phi")
lam = prog.NewContinuousVariables(horizon, n_contacts, "lambda")

for k in range(horizon):
    prog.AddConstraint(phi[k] >= 0)
    prog.AddConstraint(lam[k] >= 0)
    for c in range(n_contacts):
        prog.AddConstraint(phi[k, c] * lam[k, c] <= epsilon)  # MPCC relaxation

# Dynamics with contact: M(q) q_ddot = tau + J_c^T lambda + ...
# ...

prog.AddCost(sum_task_errors(q) + sum_control_energy(tau))
result = SnoptSolver().Solve(prog)
```

### Computation status (2025)

- Offline CI-TrajOpt: minutes per solve, produces rich through-contact trajectories.
- Online CI-MPC: still slow ($> 100$ ms). Two mainstream accelerations:
  1. **Warm-start from offline**: precompute trajectory libraries, at runtime pick the closest and warm-start MPCC.
  2. **Learned contact-mode proposal**: NN predicts the optimal mode sequence; MPC then solves a smooth problem given the proposed schedule. Best of both worlds: emergent behavior + real-time speed.

### What interviewers check

"How does CI-MPC **unify** free-space motion and in-contact manipulation?"
- Before CI-MPC: separate controllers -- position PD during free motion, impedance during contact, FSM to switch. Brittle.
- With CI-MPC: both are a single continuous trajectory through augmented $(q, \phi, \lambda)$ space. The optimizer decides when to transition. **Expert heuristics replaced by rigorous optimization.**

</details>

<details>
<summary>Deep dive: Lie Group MPC -- why quaternion + unit-norm constraint zig-zags SQP</summary>

### The problem with Euclidean state representations

Orientation in 3D lives on **SO(3)**, a 3-DoF manifold embedded in $\mathbb{R}^9$ (or $\mathbb{R}^4$ via unit quaternion). Naive approaches:
- **Euler angles** (roll, pitch, yaw): 3 variables, no redundancy. But **gimbal lock at pitch $= \pm 90^\circ$**: Jacobian loses rank, MPC explodes. Avoid for any robot that pitches near $\pm 90^\circ$ (drones, humanoids during acrobatics).
- **Quaternion with $\|q\| = 1$ hard constraint**: 4 variables, 3 DoF. Adding the nonconvex constraint $\|q\|^2 = 1$ breaks QP -- it's a nonlinear equality. Worse: the constraint gradient $\nabla \|q\|^2 = 2q$ is **normal to the sphere**, and any SQP search direction perpendicular to $q$ violates it. SQP **zig-zags**: step along the tangent, project back to sphere, step again -- each step is tiny because the projection reorients the search. Compute time **10-100x slower** than Lie group.
- **Rotation matrix with 9 variables + 6 orthonormality constraints**: even worse, 3x redundancy, 6 coupled nonlinear equalities.

### The Lie group fix

Treat state as a pair $(R \in SO(3), \text{decision variable } \omega \in \mathbb{R}^3)$. The decision variable lives in the **Lie algebra** $\mathfrak{so}(3) \cong \mathbb{R}^3$ -- a clean unconstrained 3-vector. Update the manifold element via the **exponential map**:

$$
R_{k+1} = R_k \cdot \exp(\hat{\omega}_k \cdot dt)
$$

where $\hat{\omega}$ is the skew-symmetric matrix of $\omega$.

Error between two orientations via the **logarithmic map**:

$$
e_R = \log(R_\text{ref}^T R)^\vee \in \mathbb{R}^3
$$

This is "how much to rotate (axis * angle) to align $R$ with $R_\text{ref}$" -- a physically meaningful clean 3-vector that linearizes beautifully in SQP.

### C++ with `manif` library

```cpp
#include <manif/manif.h>
using namespace manif;

SE3d X_target = ...;                          // target pose
SE3d X_current = ...;                         // current pose
// Error in se(3) -- clean 6-vector [translation(3), rotation(3)]
SE3Tangentd error = (X_target.inverse() * X_current).log();
Eigen::VectorXd e_vec = error.coeffs();       // for QP cost

// Integrate control in the tangent space
SE3Tangentd delta_u = ...;                    // MPC's control in se(3)
SE3d X_next = X_current + delta_u * dt;       // operator+ does exp map
```

### Who uses this

- **MIT Cheetah / ANYmal**: quadruped body orientation in SO(3); CoM dynamics use Lie algebra for angular momentum.
- **PX4 flight stack**: drone attitude in SO(3), no gimbal lock even in acrobatic flight.
- **Drake optimization framework**: SE(3) MPC built in.
- **GTSAM / iSAM2** (factor graph SLAM): Lie group factors for consistent linearization.

### Interview trap

"Why can't I just use quaternion with a unit-norm constraint?"
- Correct answer: the unit-norm constraint is a **sphere manifold**; SQP's linearization approximates the sphere by a tangent plane. Any step along the tangent violates the sphere, so the solver must reproject and retry. Projection reorients the search direction, leading to **zig-zagging**. The fix is to parameterize the decision variable **in the tangent space from the start** (Lie algebra) and use the exponential map to land back on the manifold -- no projection needed.

</details>

<details>
<summary>Deep dive: Distributed MPC via ADMM -- the shadow-price mechanism for multi-agent consensus</summary>

### Why centralized MPC fails on a fleet

100 AGVs in a warehouse. Centralized MPC has state dimension $\sim 1000$, control dimension $\sim 300$, horizon 20 -- the QP has $\sim 30000$ variables. Real-time impossible. Communication is also single-point-of-failure.

### ADMM formulation

Split the global optimization into per-agent local problems + consensus on overlap:

$$
L_i = J_i(x_i, u_i) + y_i^T (x_i - z) + \frac{\rho}{2} \|x_i - z\|^2
$$

- $J_i$ = local tracking + energy cost for AGV $i$
- $z$ = shared consensus trajectory (the "safe corridor" everyone agrees on)
- $y_i$ = dual variable (**shadow price**) enforcing $x_i = z$
- $\rho$ = penalty parameter

### Iteration per control cycle

1. **Local solve (parallel)**: each AGV solves its local QP with the current $z, y_i$. Output: candidate trajectory $x_i^\text{new}$.
2. **Consensus update**: exchange $x_i$ with neighbors; $z$ = weighted average of overlapping trajectories.
3. **Dual update**: $y_i \leftarrow y_i + \rho (x_i - z)$.
4. Repeat 2-3 times until convergence (good enough for MPC's receding-horizon).

### Shadow price intuition

$y_i$ is the **economic cost** of disagreement. If AGV 1 wants trajectory A (cheap for itself, overlaps with AGV 2's path), $y_1$ grows -- it "costs" AGV 1 more to insist on A. After a few iterations, AGV 1 yields to AGV 2 (or vice versa), naturally achieving collision avoidance via **economic pressure** rather than hard constraint.

### Why ADMM and not just a central server

- **Scalability**: local solves are $O(n_\text{local}^3)$, constant in fleet size.
- **Robustness**: one AGV's crash doesn't stop the rest; it just drops from the consensus.
- **Privacy / modularity**: in multi-company warehouses each vendor can run its own local controller, sharing only trajectory messages.

### Applications

- **Amazon / Geek+** warehouse AGV fleets
- **Drone swarms** (Kimera, DJI Agras formation)
- **Platooning trucks** on highways (each truck runs local MPC, negotiates spacing via ADMM)

</details>

<details>
<summary>Deep dive: Event-triggered and Self-triggered MPC -- SWaP-constrained embedded MPC</summary>

### Why solve every cycle is wasteful

In steady-state cruise, the state evolves inside the previous solution's predicted trajectory. Re-solving just returns essentially the same answer. Energy-constrained platforms (micro-drones, solar-powered agriculture bots) cannot afford continuous NMPC on a raspberry-pi-class CPU.

### Event-triggered MPC

Define a trigger condition:

$$
\|x(t) - \hat{x}(t | t_k)\|_Q \ge \sigma \cdot \|x(t_k)\|
$$

- $\hat{x}(t | t_k)$ = state predicted at previous solve $t_k$
- $\sigma$ = sensitivity threshold ($\sim 0.05 - 0.2$)

When condition holds, **wake up** the optimizer and re-solve. Otherwise, shift the previous solution and use it as-is. Typical savings: **50-80% of compute** in steady-state cruise.

### Self-triggered MPC (more aggressive)

When solving, also **predict the next trigger time** $t_\text{next}$. In between, the CPU can sleep or handle other tasks. Mathematically: compute the time horizon over which the predicted trajectory stays within the trigger tolerance, schedule the next wake at $t_\text{next}$.

### Stability guarantee

Proven via **Input-to-State Stability (ISS) + Lyapunov analysis**: design the trigger condition so the Lyapunov function $V(x)$ decreases monotonically between triggers, $\dot{V} \le -\alpha \|x\|^2$. When the bound is violated, trigger and re-solve.

### Code sketch

```cpp
bool should_trigger(const State& x, const State& x_pred, double sigma) {
    Eigen::VectorXd err = x - x_pred;
    double err_cost = err.transpose() * Q * err;
    double thresh = sigma * x.squaredNorm();
    return err_cost > thresh;
}

void control_loop() {
    static ControlSeq u_prev;
    static State x_pred;
    State x_now = measure();

    if (u_prev.empty() || should_trigger(x_now, x_pred, 0.1)) {
        auto result = nmpc.solve(x_now);
        u_prev = result.sequence;
        x_pred = result.predicted_next_state;
    } else {
        u_prev = shift(u_prev);               // cheap O(1) shift
        x_pred = nominal_propagate(x_pred, u_prev[0]);
    }
    actuate(u_prev[0]);
}
```

### The SWaP lens

**SWaP = Size, Weight, Power**. Future embodied AI -- micro-drones, insect-scale bots, skin-worn assistive devices -- cannot afford desktop GPUs. Event-triggered MPC + model reduction + learned warm-start is the path to MPC on a STM32H7 class MCU. "Compute-on-demand" is the motto, and the Lyapunov trigger is the gatekeeper that lets you cut compute by 80% without losing stability.

</details>

## Common Misconceptions

1. **"MPC can handle arbitrary nonlinear models out of the box"** -- Being able to write down a model is not the same as being able to solve it in real time. NMPC's NLP is nonconvex (no global optimality), and compute grows exponentially with model complexity. **Correct view**: industrial MPC overwhelmingly uses linearization + QP, or RTI-approximated NMPC with a reduced model (SRBD / centroidal / bicycle). Full nonlinear MPC is practical only **offline** or at slow rates (< 10 Hz).

2. **"MPC only works on linear systems"** -- Quite the opposite. NMPC handles nonlinear models; it just requires specialized solve techniques (SQP_RTI, warm start, sparse structure exploitation, condensing). Quadruped locomotion, drone acrobatics, and autonomous driving all use NMPC. **The real question** is not "can I model nonlinear?" but "can I solve this nonlinear problem within my control period?"

3. **"Using MPC means no more tuning"** -- MPC has **more** knobs than PID: $Q$ (per-state weights), $R$ (control penalty), $Q_f$ (terminal cost), $N$ (horizon), $dt$ (sampling), slack penalties $\lambda_1, \lambda_2$, plus model parameters. The $Q/R$ ratio directly sets tracking aggressiveness; $N$ too short destabilizes, $N$ too long overruns compute; wrong $Q_f$ can make nominal stability analytically vacuous. **Practical starting point**: Bryson's rule ($Q_{ii} = 1/x_{i,\max}^2$, $R_{jj} = 1/u_{j,\max}^2$), closed-loop sim to find shortest stable $N$, then verify solver timing on target hardware.

4. **"Receding horizon wastes compute -- why not solve the full horizon once and execute"** -- Full-horizon optimization (Ch11) is offline and has no feedback. MPC re-plans every step with the latest state, naturally compensating for model error and disturbance. The cost is solving a small QP/NLP per step, but each instance is small (only $N$ steps). **The essence of MPC: online closed-loop trajectory optimization.**

5. **"MPC with feasibility = solver returns OPTIMAL every cycle"** -- In production, solvers routinely return INFEASIBLE under disturbance (robot kicked, obstacle appears, actuator faults). **Industrial-grade MPC spends 70% of engineering effort on slack + fallback state machines**, not on the nominal QP tuning. The difference between "optimizer red-screens" and "factory line explodes" is the quality of the fallback. If you ship MPC without Slack + Fallback + SafeMode + logging, you have a lab demo, not a product.

6. **"NMPC is always safer than linear MPC -- why not just use NMPC?"** -- NMPC is nonconvex; SQP converges to a local minimum, and the "more accurate model" doesn't save you if the solver finds a bad local minimum under warm-start drift. Linear MPC with a well-linearized model is often **more robust in practice** because QP solvers give **global optima**. **Use NMPC only when linearization error dominates**, e.g. large-angle maneuvers, contact switching. For manipulator real-time tracking with small operating envelopes, linear MPC is usually better.

7. **"Quaternion is fine if I add $\|q\|^2 = 1$ as a constraint"** -- No. The unit-norm constraint is a sphere; SQP's linearization approximates the sphere by a tangent plane; any SQP step along the tangent violates the sphere; the solver zig-zags between sphere reprojections, making iteration 10-100x slower. **Industrial fix**: parameterize rotation in the Lie algebra $\mathfrak{so}(3)$ from the start (a clean 3-vector), lift to SO(3) via $\exp$ map. All serious quadruped / drone MPCs do this.

8. **"Tube MPC is the best robust MPC -- use it always"** -- Tube MPC gives **hard** robust safety but pays with conservatism (tube radius shrinks the free space). For scenarios where mild violations are acceptable in exchange for aggressive performance (quadruped parkour, drone racing), **Stochastic MPC / Chance-Constrained MPC** is better -- accept 0.3% collision probability, go 3x faster. L4 autonomous driving: Tube MPC (absolute safety). Quadruped parkour: Stochastic. The right tool depends on whether the application tolerates probabilistic vs deterministic safety.

## Practice Questions

<details>
<summary>Q1 (medium): A quadruped walking on wet tile starts slipping. The MPC was tuned on dry ground. Systematically diagnose and fix.</summary>

**Complete reasoning chain**:

1. **Root cause**: slipping means ground reaction forces exceed the friction cone $\|f_t\| \le \mu f_n$. Wet tile has $\mu \approx 0.2$; the MPC still uses the dry-ground $\mu \approx 0.6$. The MPC commands tangential forces that are physically impossible, the real feet slip.
2. **Immediate fix**: reduce MPC's $\mu$ parameter to a conservative value (0.15-0.2). The tighter friction cone forces MPC to reduce tangential demand $\Rightarrow$ gentler gait (shorter strides, lower speed), but no slip.
3. **Advanced -- online friction estimation**: use foot-contact force sensors + slip detection (compare commanded vs. actual foot velocity) to estimate $\hat{\mu}$ in real time, update the MPC constraint each cycle. This is what **ANYmal's terrain adaptation** does via latent-space inference.
4. **Architectural move**: add **Stochastic MPC / Chance-Constrained** wrapping around $\mu$ -- treat $\mu$ as uncertain with $3\sigma = 0.3$, then MPC inherits natural conservatism under friction uncertainty.
5. **Trap to avoid**: do NOT try to fix slipping by raising $Q$ ("track harder"). That makes MPC push harder tangentially, precisely the wrong direction on a low-friction surface. This is a classic PID-reflex that fails on MPC.
6. **Fallback state machine**: if slip is detected more than 3 times in 1 s, trigger SAFE_MODE (stand in place, call upstream for replanning).

**What the interviewer wants to hear**: MPC's core power is constraint handling. Friction estimation $\Rightarrow$ constraint update $\Rightarrow$ auto strategy change. Articulating this chain is the key, plus the chance-constrained upgrade for robustness.

</details>

<details>
<summary>Q2 (medium-hard): Your quadruped NMPC runs at 50 Hz (20 ms/step); the control loop needs 200 Hz (5 ms). No hardware upgrade. How do you compress to under 5 ms?</summary>

**Complete reasoning chain**:

1. **Profile first**: confirm the bottleneck is NLP solving (usually is, 80%+). Check dynamics-assembly cost and state-estimator cost separately. Optimizing the wrong thing wastes days.
2. **RTI (Real-Time Iteration)** -- single biggest win: switch from "SQP to convergence" to **one QP iteration per step**, warm-started from the previous solution. acados `SQP_RTI` mode. Typical 5-10x speedup, nearly zero quality loss for smooth dynamics.
3. **Warm start**: shift previous solution one step, append reasonable $u_\text{init}$ at tail. Cuts QP iterations by 2-3x.
4. **Sparsity exploitation**: drop IPOPT, use HPIPM or qpOASES with partial condensing. The KKT matrix of MPC is banded -- HPIPM is 10-100x faster than general-purpose NLP solvers on this structure.
5. **Model reduction**: full 18-DoF quadruped dynamics $\to$ SRBD (6-DoF centroidal) $\to$ MPC runs as a convex QP. Pair with WBC at 1 kHz for joint-level fidelity.
6. **Condensing vs Sparse**: high $n_x$ $\to$ sparse (keep $x$, KKT banded); low $n_x$ but long $N$ $\to$ partial condensing; single shooting + dense only for very short $N$ and stable dynamics.
7. **Last resort**: shorten $N$ (verify closed-loop stability with shorter horizon), or cascade a slow NMPC outer loop (10 Hz) + fast linear MPC inner loop (200 Hz).
8. **Validation**: closed-loop sim under disturbance, measure worst-case solve time over 10 min, confirm it fits the budget at p99.

**What the interviewer wants to hear**: not "use a faster solver" but the **specific combination** RTI + warm start + structure exploitation + reduced model, each justified with reasoning. Bonus: mention that acados bundles RTI + HPIPM + Multiple Shooting out of the box -- the industry default.

</details>

<details>
<summary>Q3 (hard): MPC tracks perfectly in sim; the real robot has 3 ms comms delay plus unmodeled harmonic-drive flexibility, and violently chatters. How do you debug and fix?</summary>

**Complete reasoning chain**:

1. **Delay compensation first** (most common miss): the "current state" $x_k$ that MPC sees is actually 3 ms old. Fix: **predict forward** before solving, $\hat{x}_{k+d} = f(x_k, u_{k-d:k})$, so MPC plans from the state that will exist when the command is actually applied.
2. **Frequency sweep the motor**: inject a chirp 0-500 Hz, log current-to-position. Find actual motor bandwidth. If MPC's control rate (100 Hz) sits above the motor Nyquist, you're commanding frequencies the motor cannot realize -- the motor just chatters. Fix: low-pass filter MPC output, or add $\Delta u$ (rate) penalty in the cost.
3. **Rate penalty in cost**: add $\sum \|u_k - u_{k-1}\|^2_R$ to the MPC cost with a larger $R$. This penalizes high-frequency control jumps, forcing smoother commands within motor bandwidth.
4. **Disturbance Observer (DOB)**: add an integrating disturbance state to capture unmodeled dynamics (harmonic drive flex, friction drift). MPC's prediction includes $\hat{d}_k$, so feedforward compensates on the fly.
5. **Offset-free augmentation**: formally add $d_{k+1} = d_k$ state + Kalman estimation of $d$ from innovation. This is the rigorous version of DOB + feedforward; gives zero steady-state error under persistent disturbance.
6. **Model refinement via SysID**: if the harmonic drive has meaningful flexibility, add a two-mass model in the prediction ($q_\text{motor} \ne q_\text{joint}$ via a spring-damper). Often worth the state-dimension increase.
7. **Tube tightening**: add margin to torque limits $u_\text{max}^\text{MPC} = u_\text{max}^\text{real} - \Delta u$. The real system sees $u_\text{max}^\text{real}$ even when MPC respects the tightened limit -- leaves room for the ancillary feedback.
8. **Verify**: repeat the failure scenario in sim with the delay + unmodeled flex injected; confirm the fix holds.

**What the interviewer wants to hear**: state prediction for delay is the most commonly missed yet critical fix; DOB + offset-free is the go-to industrial model correction; rate penalty is the simplest motor-bandwidth-aware fix; chirp sweep is the standard diagnostic. Knowing this is the watershed between "MPC in sim" and "MPC on hardware."

</details>

<details>
<summary>Q4 (medium): A junior engineer asks "why not just use LQR?" How do you explain the difference and when each wins?</summary>

**Complete reasoning chain**:

1. **LQR is a special case of MPC**: LQR = infinite horizon + **no constraints** + linear model. LQR's gain $K = -R^{-1}B^T P$ is a constant matrix computed offline; online cost is one matrix-vector multiply. MPC is LQR + constraints + finite horizon + re-solving per step.
2. **Constraint handling**: LQR gives $u = Kx$ with no guarantee that $u_\text{min} \le u \le u_\text{max}$. You can clamp externally, but post-clamp the control is no longer optimal (and windup can occur). MPC enforces constraints **inside** the optimization $\Rightarrow$ optimal under the constraints.
3. **Nonlinear systems**: LQR linearizes once at an operating point and degrades farther from it. NMPC re-linearizes (or uses the full nonlinear model) each cycle, so it handles large excursions and varying operating points naturally.
4. **Time-varying reference**: LQR tracks a setpoint easily; tracking a trajectory requires time-varying LQR (computed via Riccati at each time), whose implementation complexity approaches MPC anyway -- at which point you might as well use MPC.
5. **Predictive look-ahead**: LQR reacts to current state only. MPC anticipates -- can slow down before a known curve, shift CoM in advance of a swing phase. Critical for quadrupeds.
6. **Bottom line**:
   - Simple linear system + no constraints + setpoint tracking $\to$ **LQR** (faster, simpler).
   - Hard constraints / nonlinear / time-varying trajectory / predictive maneuvering $\to$ **MPC**.
7. **Bonus**: mention that MIT Cheetah's "convex MPC" is essentially "constrained QP LQR with friction cone" -- showing that understanding LQR as the special case makes MPC intuition easy.

**What the interviewer wants to hear**: clear statement that LQR is MPC's special case, constraint handling is MPC's defining advantage, and the decision rule is constraint-driven and look-ahead-driven.

</details>

<details>
<summary>Q5 (hard): You build an Amortized MPC (NN that imitates NMPC output). Deployment shows rare catastrophic actions on OOD states. How do you keep the $O(1)$ inference speed but guarantee safety?</summary>

**Complete reasoning chain**:

1. **The failure mode**: NN is a **soft** approximation of MPC. On OOD states (states not in training distribution), the NN can output arbitrary garbage -- including actions that violate torque limits, drive into obstacles, or pick up impossible velocities.
2. **Pattern: CBF Safety Filter post-NN**:
   - NN emits proposed action $u_\text{nn}$.
   - A tiny QP minimally projects $u_\text{nn}$ onto the safe set defined by CBF: $\dot{h}(x) + \alpha h(x) \ge 0$.
   - QP cost: $\|u - u_\text{nn}\|^2$ (stay close to NN). Constraints: CBF + torque limits + friction cone.
   - QP is $n_u$-dim with a handful of inequalities $\Rightarrow$ 50 $\mu$s on embedded CPU $\Rightarrow$ $O(1)$-friendly.
3. **Define $h$ carefully**: $h(x) = $ signed distance to danger. Examples: $h_\text{torque}(u) = u_\text{max} - |u|$; $h_\text{collision}(x) = \text{dist}(x, \text{obstacle})$; $h_\text{joint}(q) = q_\text{max} - q$. Multiple $h$ functions = multiple constraints.
4. **Monitor filter activation**: log "how often the filter modified the action." Low activation $\Rightarrow$ NN is learning well. Rising activation $\Rightarrow$ OOD state, maybe add to training set or retrain.
5. **Certification**: the CBF filter is **mathematically provable** safe (forward-invariant set). A black-box NN is not. This combo -- NN for speed, CBF for safety -- is how you get past ISO 13849 review for an AI-heavy controller. Regulators scrutinize the filter, not the NN.
6. **Boundary case**: if the filter makes $u$ very different from $u_\text{nn}$, that's a signal the policy is in a regime it wasn't trained on; trigger SAFE_MODE and log for retraining.
7. **Trap**: don't skip the QP thinking "NN already learned the constraints." Learned constraints are **soft** (99% satisfied); safety-critical deployment demands **hard** (100%). The filter is non-negotiable.

**What the interviewer wants to hear**: distinguish between "soft learned" and "hard provable" constraint satisfaction; name the CBF QP pattern precisely (filter, not replacement); mention certification path. Bonus: mention this is the 2024 standard for Foundation-Model + MPC stacks.

</details>

<details>
<summary>Q6 (hard): Your manipulator MPC has been perfect for 6 months. Monday morning, after the warehouse upgraded to a new conveyor belt, the arm starts showing 5 mm steady-state error on pick tasks. How do you systematically debug?</summary>

**Complete reasoning chain**:

1. **Localize the error source**: 5 mm is ridiculously large for a Franka-class arm (expected $< 0.1$ mm). Not a tuning issue -- something structural changed. First move: compare what's different about Monday.
2. **Hypothesis: persistent external force from the new conveyor** -- it exerts a constant force on the end-effector during pickup (magnetic, belt friction drag, air flow). Nominal MPC has **no integral action** $\Rightarrow$ persistent disturbance $\Rightarrow$ steady-state error, same as PD (no I) against gravity.
3. **Confirm**: take a static sample -- arm holds position at the pickup location; measure F/T sensor. If $F \ne 0$ in steady state, confirmed external force.
4. **Fix -- Offset-free MPC (the "I-term" of MPC)**:
   - Augment state with integrating disturbance: $d_{k+1} = d_k$, $y_k = C x_k + d_k$.
   - Kalman / Luenberger observer estimates $\hat{d}_k$ from innovation.
   - Target calculator re-solves steady-state targets given $\hat{d}_k$: MPC computes $x_s, u_s$ so that in steady state the augmented dynamics hold.
   - MPC regulates around the updated target $\Rightarrow$ zero steady-state error.
5. **Alternative -- Velocity-form MPC** (incremental): optimize $\Delta u_k = u_k - u_{k-1}$ instead of $u_k$. Naturally integrates past $\Delta u$, so any persistent error accumulates into $u$ until error vanishes. Simpler but observer is implicit.
6. **Don't** just crank $Q$: raising weights makes the error proportionally smaller ($\propto 1/K_p$) but never zero. Adding $Q \to \infty$ degrades stability margin.
7. **Long-term**: if conveyor force is actually a disturbance **by design**, add it as a **known feedforward** (measure with F/T on install, feed into MPC). Offset-free observer handles the remaining unmodeled residual.
8. **Verify**: track steady-state error over 1000 picks. Should drop from 5 mm to $< 0.1$ mm.

**What the interviewer wants to hear**: precise identification of the "no-integral-action" symptom; name-drop Offset-free / Velocity-form MPC; distinguish "known disturbance -> feedforward" from "unknown disturbance -> observer." Bonus: treating this as a MPC engineer is how you handle any plant-change incident in production.

</details>

<details>
<summary>Q7 (very hard): You have a humanoid doing contact-rich manipulation (pivoting a heavy door). Traditional MPC needs a hand-coded mode FSM which breaks for odd door angles. Walk through how Contact-Implicit MPC changes this, including the MPCC relaxation and why it's hard to run in real time.</summary>

**Complete reasoning chain**:

1. **Traditional pain**: FSM encodes "reach door handle, grip, pull, pivot, release." Each transition hand-crafted. Odd door tilt (15 degrees off vertical) breaks the geometry; FSM lacks coverage.
2. **CI-MPC core idea**: treat contact distance $\phi(q)$ and contact force $\lambda$ as **decision variables**, not scheduled constants. Complementarity $\phi \cdot \lambda = 0$ says "either not touching ($\phi > 0, \lambda = 0$) or on surface ($\phi = 0, \lambda > 0$)." The optimizer **autonomously chooses when to touch**, emerging behavior.
3. **MPCC (Mathematical Program with Complementarity Constraints) relaxation**: $\phi \cdot \lambda = 0$ is non-smooth, non-convex, gradient undefined at origin. Relax to $\phi \cdot \lambda \le \varepsilon$, anneal $\varepsilon \to 0$. Same philosophy as interior-point barriers.
4. **Dynamics**: $M(q) \ddot{q} + C(q, \dot{q})\dot{q} + G = \tau + J_c^T \lambda$, with $\lambda$ coupled via complementarity. The resulting NLP is **rich** but nonconvex.
5. **Why real-time is hard**: the combined NLP has $O(\text{horizon} \times \text{contacts})$ complementarity pairs; each non-smooth. Current solvers (SNOPT, IPOPT with MPCC) take 100s of ms to seconds.
6. **Production accelerations**:
   - **Offline trajectory libraries**: precompute CI-MPC trajectories for a grid of door configurations. At runtime, match the closest and warm-start an online solve.
   - **Learned mode proposals**: NN predicts the likely contact mode sequence given sensor input; online MPC solves a **smooth** optimization conditional on the predicted sequence (no complementarity). Best of both.
   - **Receding-horizon schedule refinement**: reoptimize only a short window, freeze modes outside.
7. **Platform examples**: MIT Cheetah uses CI-TrajOpt offline + online tracking. Drake supports CI-MPC experimentally. No production humanoid runs pure online CI-MPC yet (2025).
8. **The interview hook**: "CI-MPC unifies **free-space motion** and **in-contact manipulation** into a single optimization -- what used to require separate controllers + FSM switch is now one continuous trajectory through $(q, \phi, \lambda)$ space. Expert-heuristics replaced by rigorous optimization."

**What the interviewer wants to hear**: precise statement of the complementarity formulation, the MPCC relaxation step with $\varepsilon$ annealing, honest acknowledgment of compute cost and the pragmatic hybrid (offline library + online warm start, or learned proposals). Frame it as the **2024 breakthrough for contact-rich manipulation**.

</details>

<details>
<summary>Q8 (medium-hard): A warehouse owner deploys 100 AGVs running centralized MPC; the control server becomes a bottleneck and occasionally freezes. Explain Distributed MPC via ADMM and how it solves this.</summary>

**Complete reasoning chain**:

1. **Why centralized fails**: state dim $= 100 \times 4 \approx 400$, control dim $\approx 200$, horizon 20 $\Rightarrow$ 12000-variable QP per cycle. Solve is $O(n^3) = 10^{12}$ FLOPs $\Rightarrow$ seconds, not milliseconds. Single-server also = single point of failure.
2. **Distributed MPC with ADMM**:
   - Each AGV solves a **local** MPC of its own state + control.
   - AGVs whose trajectories overlap share a **consensus variable** $z$.
   - Augmented Lagrangian: $L_i = J_i(x_i, u_i) + y_i^T(x_i - z) + \frac{\rho}{2}\|x_i - z\|^2$.
   - $y_i$ = dual variable (shadow price of disagreement), $\rho$ = penalty parameter.
3. **Iteration** (2-3 times per control cycle):
   - **Local solve (parallel)**: each AGV computes $x_i^\text{new}$ given current $z, y_i$.
   - **Consensus**: exchange trajectories with neighbors; update $z$ = weighted average over overlapping regions.
   - **Dual update**: $y_i \leftarrow y_i + \rho(x_i - z)$.
4. **Shadow-price intuition**: $y_i$ grows when AGV insists on a trajectory that overlaps neighbors; AGV "pays" more to keep its plan; eventually yields. Economic pressure $\Rightarrow$ consensus $\Rightarrow$ collision-free fleet behavior. **Compare to market pricing.**
5. **Why this works**:
   - Scalability: per-agent solve is constant in fleet size.
   - Robustness: one AGV's crash doesn't stop the rest.
   - Privacy: multi-vendor warehouses can run their own local MPCs, sharing only trajectories (not models or costs).
6. **Trade-offs**:
   - ADMM converges in ~10-20 iterations for strictly convex problems; per-MPC-cycle budget allows 2-3. Consensus isn't fully resolved, but receding-horizon re-consensus fixes it over time.
   - $\rho$ tuning matters: too small $\Rightarrow$ slow convergence; too large $\Rightarrow$ oscillation in the dual.
7. **Production**: Amazon, Geek+ use variants; DJI Agras formation flying uses similar distributed optimization. Standard graduate-level course topic.

**What the interviewer wants to hear**: ADMM mechanics + the shadow-price interpretation (economic pressure for consensus). Name ADMM explicitly, mention strict convexity + $\rho$ tuning. Bonus: frame as the decentralized analog of A* path coordination used in multi-robot planning.

</details>

<details>
<summary>Q9 (hard): A VLA (Vision-Language-Action) foundation model outputs desired end-effector waypoints at 5 Hz; you must control a 7-DoF arm at 1 kHz. Design the full stack including safety.</summary>

**Complete reasoning chain**:

1. **Problem decomposition**: 5 Hz semantic planner (VLA) + 1 kHz physical execution. Classic "brain slow / cerebellum fast / spine hard-reflex" layering.
2. **Layer 1 (brain, 5-10 Hz)**: VLA (RT-2 / OpenVLA / $\pi_0$) outputs waypoint sequence $p_\text{ref}(t)$, 1-2 seconds ahead. Semantic-rich (knows "avoid the cup"), slow.
3. **Layer 2 (cerebellum, 100 Hz)**: NMPC on the arm's operational-space dynamics. Tracks $p_\text{ref}$ from VLA, respects joint torque limits, avoids singularities via null-space cost, handles short-term dynamics. acados SQP_RTI + Multiple Shooting fits the budget.
4. **Layer 3 (spinal reflex, 1 kHz)**: impedance control (Ch14) at the joint level -- implements the MPC's torque command via high-fidelity force tracking, softens collisions by design.
5. **Safety filter (on every VLA output + on every MPC output)**:
   - CBF QP: minimally project commands to satisfy joint-limit barriers + workspace barriers + human-proximity barriers.
   - If CBF frequently activates, log for retraining VLA / MPC.
6. **State estimation (Ch07)**: Kalman filter fusing joint encoders + IMU + optional VIO to feed $x_k$ to MPC.
7. **Failure modes**:
   - VLA outputs waypoint in collision $\Rightarrow$ CBF rejects, fallback: hold last-valid waypoint.
   - MPC solver infeasible $\Rightarrow$ fallback: shift previous solution, degrade to SAFE_MODE after 3 consecutive failures.
   - Comms loss between VLA and MPC $\Rightarrow$ MPC runs on last waypoint (stale but safe for $\sim 1$ s).
8. **Why this structure**: pure end-to-end VLA cannot pass ISO 13849 -- no formal safety. Pure MPC cannot handle "avoid the cup" semantics. Layered stack provides **semantic richness + hard safety**. "Large model slow thinking, MPC fast reflex, CBF absolute physical floor."

**What the interviewer wants to hear**: layered architecture with explicit frequency separation (5 / 100 / 1000 Hz); CBF filter as the bridge between learned policy and certifiable safety; mention this is the **2025 standard architecture** for VLA-driven robots. Bonus: call it "brain / cerebellum / spinal cord" to signal systems thinking.

</details>

## Interview Angles

1. **MPC vs PID vs LQR decision rule** -- the most basic test. **Why this is the key**: most interviewees confuse MPC with "smarter PID" without grasping the constraint-handling distinction. Bring it out with: "PID is reactive and model-free; LQR is predictive-but-no-constraints and linear-only; MPC is the full Pareto front -- predictive, constraint-aware, nonlinear-capable. I pick MPC when I see hard actuator limits, torque saturation, friction cones, or look-ahead maneuvering; LQR for simple linear systems; PID for the innermost loop where latency beats optimality."

2. **Fully-actuated vs under-actuated decision rule (manipulator vs quadruped)** -- often asked in embodied-AI-focused interviews. **Why this is the key**: many candidates don't understand why the **same** MPC architecture can't apply to both. Bring it out with: "Manipulators are fully-actuated fixed-base -- CTC + PID at 1 kHz perfectly tracks, no need for MPC. Quadrupeds are floating-base under-actuated -- the only way to avoid falling is to **anticipate GRFs over a horizon**. That's why MPC is optional on the Franka but essential on ANYmal."

3. **Convex relaxation is the key to deployment** -- separates textbook MPC from MPC that runs on hardware. **Why this is the key**: real constraints are rarely convex out of the box. Bring it out with: "Friction cones need SOCP relaxation, collision constraints need convex-hull approximation, nonlinear dynamics need sequential linearization (SQP_RTI). Turning nonconvex problems into efficiently solvable convex ones is the engineering core of MPC."

4. **Feasibility is the production deathline; 70% engineering effort on slack + fallback** -- the watershed between "demo MPC" and "factory MPC". **Why this is the key**: solvers routinely return infeasible under real disturbances, and the difference between a red-screen warning and an exploding motor is the fallback. Bring it out with: "Every MPC I ship has three layers: normal solve with slack, fallback to shifted previous solution on infeasibility, and SAFE_MODE (pure impedance hold) after 3 consecutive failures. Plus full logging -- the post-mortem of industrial failures is a trace of state-machine transitions."

5. **RTI + Warm Start + HPIPM + Multiple Shooting = the hard-real-time stack** -- shows industry maturity. **Why this is the key**: the gap between IPOPT (100 ms) and production NMPC (< 1 ms) is exactly this combination. Bring it out with: "For 1 kHz NMPC I use acados, which bundles SQP_RTI (single iteration per cycle) + warm start from shifted previous + HPIPM exploiting the banded KKT + Multiple Shooting for stable rollout of open-loop-unstable systems. Plain IPOPT is 100x too slow."

6. **Offset-free is the MPC analog of PID's I-term -- industrial must-have** -- surfaces real production experience. **Why this is the key**: nominal MPC has no integral action, leaving persistent steady-state error against any constant disturbance (gravity drift, wind, friction). Bring it out with: "I always augment my plant model with an integrating disturbance state and estimate it with a Kalman / Luenberger observer. That + target calculator gives zero steady-state error offset, exactly the role I did on PID."

7. **Tube vs Stochastic vs Min-Max -- pick by safety regime** -- the robust MPC selector. **Why this is the key**: beginners say "use robust MPC" without knowing which flavor. Bring it out with: "Tube MPC for absolute hard safety (L4 self-driving pedestrian avoidance); Stochastic / Chance-Constrained for probabilistic safety with 3x better aggressiveness (quadruped parkour, drone racing); Min-Max almost never in production (too conservative + exponential compute). Adaptive Tube is state-of-the-art 2024 -- tube shrinks as SysID narrows uncertainty, so the robot starts cautious and learns to be bold."

8. **CBF Safety Filter after any learned policy** -- the 2024 embodied AI architecture must-know. **Why this is the key**: Foundation Models can't pass functional safety certification alone; pairing with a provably-safe filter is the only way. Bring it out with: "When I deploy VLA or any NN policy, I always pipe the action through a CBF QP safety filter. It's a tiny QP (50 microseconds) that minimally projects the action onto the safe set defined by $\dot{h} + \alpha h \ge 0$. This is what makes a 'learned cerebellum' certifiable."

9. **Contact-Implicit MPC -- the 2024 manipulation game-changer** -- frontier topic. **Why this is the key**: for years manipulation needed hand-coded FSMs; CI-MPC unifies free-space and in-contact motion via complementarity constraints. Bring it out with: "CI-MPC turns contact distance and force into decision variables; the optimizer autonomously discovers when to touch, push, pivot -- mode sequences emerge from optimization. The catch is the complementarity $\phi \cdot \lambda = 0$ is non-smooth; MPCC relaxation ($\le \varepsilon$, anneal) is the standard trick. Still slow to run online -- hybrid with offline libraries is the pragmatic compromise."

10. **Lie Group MPC for floating-base rotation** -- separates "knows quaternions" from "has shipped aerial/legged MPC". **Why this is the key**: naive quaternion + unit-norm constraint zig-zags SQP; $SO(3)$ on the tangent space is the only sane way. Bring it out with: "For any floating-base robot I parameterize rotation on the Lie algebra $\mathfrak{so}(3)$ from the start -- unconstrained 3-vector, lifted to SO(3) via $\exp$ map. The error via $\log$ map is a clean 'how much to rotate to align' vector. This single decision cuts my NMPC solve time 10-100x vs quaternion + norm constraint."

11. **Foundation Model + MPC = 'brain slow / cerebellum fast / spine hard-reflex'** -- the systems view for embodied AI. **Why this is the key**: pure end-to-end RL is not ISO-certifiable; pure MPC can't do "avoid the cup" semantics. Bring it out with: "My architecture is VLA at 5-10 Hz for open-world semantic planning + MPC at 100 Hz for physical tracking + impedance at 1 kHz for safe contact + CBF filter as the physics-layer safety net. Large models explain **what**; MPC executes **how**; CBF guarantees **never**."

12. **Event / Self-triggered MPC for SWaP embedded AI** -- the edge-compute pragma. **Why this is the key**: micro-drones, agriculture bots, wearables can't afford continuous NMPC. Bring it out with: "I use event-triggered MPC for SWaP-constrained platforms: solve only when the state drifts past a Lyapunov-certified bound, shift previous otherwise. Cuts 50-80% of compute while preserving stability. This is the bridge that lets MPC run on a STM32H7 MCU next to a Foundation Model on a Jetson."

13. **MPC failures almost always trace to sim-to-real delay + model mismatch** -- sim-to-real debugging experience. **Why this is the key**: "sim perfect, hardware chatters" is the canonical field symptom. Bring it out with: "Three-step checklist: (1) state prediction before MPC to compensate comms delay, (2) chirp-sweep the motor to find its true bandwidth and add rate penalty in the cost to keep commands below Nyquist, (3) add Offset-free or DOB to capture unmodeled dynamics. Applied in order, these fix 90% of sim-to-real chatter."

14. **Distributed MPC via ADMM for fleets -- shadow-price economics** -- shows breadth beyond single-robot. **Why this is the key**: 100-AGV fleets need distributed optimization, not centralized MPC. Bring it out with: "ADMM splits the global problem: each AGV solves a small local MPC, broadcasts its trajectory, neighbors penalize overlap via a dual variable -- essentially a shadow price. After 2-3 iterations the fleet converges to a consensus free of collisions. Scales linearly in fleet size, survives individual crashes."

## Further Reading

- **Rawlings, Mayne & Diehl, *Model Predictive Control: Theory, Computation, and Design* (2nd ed)** -- the MPC bible. Ch5 (stability), Ch7 (robust), Ch10 (NMPC), Ch8 (distributed). High-frequency interview topics all live here.
- **Borrelli, Bemporad & Morari, *Predictive Control for Linear and Hybrid Systems*** -- the hybrid-system MPC reference; crucial for Contact-Implicit thinking.
- **acados documentation + examples (github.com/acados/acados)** -- the fastest open-source NMPC framework; industrial reference for SQP_RTI + HPIPM + Multiple Shooting. Read the quadruped example repo.
- **CasADi tutorials (web.casadi.org)** -- Swiss-army knife for symbolic modeling + NLP solving; essential learning scaffold.
- **MIT Cheetah 3 MPC paper (Di Carlo et al., 2018)** -- canonical quadruped convex-MPC architecture (SRBD + friction QP at 1 kHz).
- **ANYmal C paper and OCS2 framework (github.com/leggedrobotics/ocs2)** -- ETH's SLQ / DDP-based MPC for legged robots with switched contacts.
- **MuJoCo MPC (github.com/google-deepmind/mujoco_mpc)** -- DeepMind's open-source sampling MPC (Predictive Sampling + iLQG). Plays with mjpc locally for 10+ control demos.
- **Drake contact-implicit optimization examples** -- the open-source reference for CI-TrajOpt and CI-MPC prototyping.
- **Tube MPC tutorial by Mayne et al.** -- "Robust model predictive control of constrained linear systems with bounded disturbances"; canonical paper for the L4 self-driving standard.
- **Paper: Diehl, Ferreau & Haverbeke, "Efficient Numerical Methods for Nonlinear MPC and Moving Horizon Estimation"** -- introduces RTI and explains why one SQP iteration is enough.
- **Paper: Ames et al., "Control Barrier Functions: Theory and Applications"** -- the CBF safety filter reference; pair with any learned-policy deployment.
- **Paper: Posa, Cantu & Tedrake, "A direct method for trajectory optimization of rigid bodies through contact"** -- foundation for Contact-Implicit MPC.
- **Paper: Williams et al., "Model Predictive Path Integral Control"** -- the MPPI classic; sampling-based MPC for non-smooth dynamics.
- **Paper: Hafner et al., "DreamerV3"** -- learned world model + online MPC in latent space; sample-efficient embodied AI.
- **Paper: Brunke et al., "Safe Learning in Robotics" (Annual Review)** -- survey of learning-MPC combinations with safety guarantees.
- **`manif` library (github.com/artivis/manif)** -- Lie group / Lie algebra C++ header-only library; essential for any SO(3) / SE(3) MPC.
- **ISO 13849 + ISO 10218 + ISO/TS 15066** -- functional-safety standards that MPC-driven robots must satisfy; reading them is how you learn why CBF filters + slack fallbacks are mandatory.
- **Foundation-model-driven robotics papers (RT-2, OpenVLA, $\pi_0$)** -- the upper layer that MPC now sits beneath; read these to understand where the industry is heading in 2025.

