---
title: "Model Predictive Control (MPC) and Trajectory Tracking"
prerequisites: ["13-pid-control-tuning", "11-trajectory-optimization"]
estimated_time: 45
difficulty: 4
tags: ["mpc", "control", "optimization", "real-time"]
sidebar_position: 15
---

# Model Predictive Control (MPC) and Trajectory Tracking

## You Will Learn

- Articulate the fundamental difference between MPC and PID: MPC looks N steps ahead and handles constraints; PID reacts to present and past error
- When faced with "the robot must track a fast trajectory under joint torque limits," know that MPC is the right tool rather than cranking up PID gains
- Decide when to use linear MPC (QP, millisecond-scale), when NMPC (SQP/IPOPT, more accurate but slower) is necessary, and how to compress solve time when it won't fit a real-time loop

## Core Concepts

**Precise Definition**: **Model Predictive Control (MPC)** is an online optimization-based control strategy. At each control cycle it uses a **prediction model** to project the system N steps forward, minimizes a **cost function** (tracking error + control effort) subject to **constraints** (torque limits, joint bounds, friction cones, etc.), and executes only the first control action. The next cycle re-solves from the latest measurement. This **receding horizon** mechanism gives MPC inherent look-ahead and constraint-handling ability.

**The three pillars of MPC**:
1. **Prediction model** — discretized system dynamics $x_{k+1} = f(x_k, u_k)$ (linear or nonlinear)
2. **Cost function** — weighted sum of state tracking error and control energy
3. **Constraints** — hard physical limits (torque saturation, velocity bounds, collision avoidance, friction cones)

**MPC vs PID**: PID is **reactive** — it corrects only after observing error, has no concept of the future, and cannot enforce constraints (anti-windup is after-the-fact). MPC is **predictive** — it selects the optimal N-step strategy under constraints, handling hard limits by construction.

**MPC vs Ch11 offline trajectory optimization**: Ch11 generates an entire trajectory offline. MPC re-plans online every tick — essentially a small trajectory optimization each cycle. MPC can wrap around or replace PID as a high-level tracking controller.

**Location in the Sense → Plan → Control Loop**:
- **Input**: current state estimate $x_k$ (from sensors / state estimator), reference trajectory $x^{\text{ref}}_{k:k+N}$ (from planner)
- **Output**: optimal control action $u_k^*$ for the current instant (torque, velocity, or acceleration command sent to the low-level driver / PID)
- **Downstream**: actuator executes → state changes → sensors feed back → next MPC solve
- **Loop node**: sits in the **control layer** but doubles as online re-planning, bridging planning and control

**Minimum Sufficient Math**:

1. **Linear MPC as a QP** (the most common industrial configuration):

$$
\min_{U} \sum_{k=0}^{N-1} \left[ (x_k - x_k^{\text{ref}})^T Q (x_k - x_k^{\text{ref}}) + u_k^T R\, u_k \right] + (x_N - x_N^{\text{ref}})^T Q_f (x_N - x_N^{\text{ref}})
$$

$$
\text{s.t.} \quad x_{k+1} = A x_k + B u_k, \quad u_{\min} \le u_k \le u_{\max}, \quad x_{\min} \le x_k \le x_{\max}
$$

**Physical meaning**: $Q$ penalizes tracking deviation (larger → tighter tracking), $R$ penalizes control effort (larger → gentler but looser), $Q_f$ is the terminal cost (guarantees finite-horizon stability). The entire problem is a **Quadratic Program (QP)** with mature millisecond-scale solvers (OSQP, qpOASES).

2. **Receding horizon principle**:

$$
u^*_{0:N-1} = \arg\min J(U) \quad \Rightarrow \quad \text{execute only } u^*_0,\; \text{discard } u^*_{1:N-1}
$$

**Physical meaning**: every cycle re-solves using the latest measurement, correcting for model mismatch. This gives MPC natural closed-loop robustness — even with an imperfect model, continuous feedback correction suppresses error.

3. **NMPC (Nonlinear MPC)**: the prediction model becomes nonlinear $x_{k+1} = f(x_k, u_k)$, and cost/constraints may also be nonlinear. Solving becomes a **Nonlinear Program (NLP)**, addressed with SQP (Sequential Quadratic Programming) or IPOPT (Interior Point Optimizer). More accurate but computationally heavier.

<details>
<summary>Deep dive: from QP to solver — MPC computation pipeline and convex relaxation techniques</summary>

### Linear MPC QP matrix expansion

Unrolling the prediction model $x_{k+1} = Ax_k + Bu_k$ over $N$ steps expresses all future states as an affine function of the initial state $x_0$ and the control sequence $U = [u_0, u_1, \dots, u_{N-1}]^T$:

$$
\mathbf{X} = \mathcal{A} x_0 + \mathcal{B} U
$$

where $\mathcal{A}$ stacks powers of $A$ and $\mathcal{B}$ is a lower-triangular Toeplitz matrix of $A$ and $B$. Substituting into the cost yields the standard QP:

$$
\min_U \frac{1}{2} U^T H U + g^T U \quad \text{s.t.} \quad C U \le d
$$

$H = \mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R}$ is positive definite (given $Q \succeq 0, R \succ 0$), guaranteeing a unique global solution.

### Solver landscape

| Solver | Method | Typical speed | Best for |
|--------|--------|---------------|----------|
| OSQP | ADMM | < 1 ms (medium) | Embedded, quadrupeds |
| qpOASES | Active-set | < 0.5 ms (small) | Hard real-time |
| ECOS | Interior-point | 1–10 ms | SOCP cone constraints |
| IPOPT | Interior-point (NLP) | 10–100 ms | NMPC |
| acados | Structure-exploiting SQP_RTI | < 1 ms (NMPC) | High-speed NMPC |

### Convex relaxation techniques

Nonlinear constraints cannot be solved directly as a QP. Common strategies:

1. **SQP_RTI (Real-Time Iteration)**: perform only one QP iteration per control cycle instead of solving to convergence, warm-starting from the previous solution
2. **Friction cone relaxation**: Coulomb friction $\|f_t\| \le \mu f_n$ is an SOCP (Second-Order Cone Program), solvable with ECOS directly
3. **Convex approximation**: wrap nonconvex obstacle constraints in half-planes or convex polyhedra, converting to linear inequalities

### Stability guarantees

Closed-loop stability of linear MPC typically requires:
- Terminal cost $Q_f$ set to the Riccati solution (equivalent to the infinite-horizon LQR cost)
- Terminal constraint set $\mathcal{X}_f$ as an invariant set
- In practice, a sufficiently long $N$ plus a generous $Q_f$ usually suffices without a strict terminal constraint

</details>

**Common APIs / Toolchain**:

| Layer | Tool | Example interface |
|-------|------|-------------------|
| QP solver | OSQP | `osqp.solve()` → returns $u^*$ |
| NMPC framework | acados | `ocp_solver.solve_for_x0(x0)` → one RTI step |
| NLP solver | CasADi + IPOPT | `solver(x0=..., lbg=..., ubg=...)` → general NLP |
| Quadruped MPC | MIT Cheetah / OCS2 | Linearized SRBD + QP at 1 kHz |
| ROS 2 integration | ros2_control + MPC plugin | `update()` returns `JointTrajectory` |

## Intuition

**Analogy: driving while looking ahead N steps**. PID is like correcting the steering wheel only after noticing you have drifted — reactive. MPC is like an experienced driver scanning 200 meters ahead, mentally simulating "if I press the throttle this much, where will I be in 3 seconds, will I exceed the speed limit (constraint)?" and choosing the best action. Every second the driver re-simulates because road conditions (state feedback) change.

**Visual metaphor: Go AI's look-ahead search**. MPC's horizon $N$ is like AlphaGo searching $N$ moves ahead; deeper search means smarter play (better tracking) but higher compute (longer solve time). The cost function is the board evaluator, constraints are the rules. The receding horizon is re-searching every move — because the opponent (disturbances) surprises you.

**Simulator observation**: in MuJoCo, run a quadruped robot and sweep the MPC horizon from $N = 5$ to $N = 30$:
- $N$ too short: the robot walks unsteadily, cannot prepare for turns
- $N$ just right: smooth gait, torques within bounds
- $N$ too long: per-step solve time grows, control frequency drops, oscillations appear
- Drop `u_max` from 33 Nm to 10 Nm: MPC automatically slows down to satisfy constraints; PID saturates and crashes

## Implementation Link

**Three representative engineering scenarios**:

1. **Quadruped locomotion MPC**: simplify an 18-DoF quadruped to a 6-DoF centroidal model via Single Rigid Body Dynamics (SRBD), solve a QP under friction cone constraints $\|f_t\| \le \mu f_n$ at 1 kHz to produce contact forces → inverse dynamics maps to joint torques. MIT Cheetah 3 and Unitree Go1 both use this architecture.

2. **Manipulator real-time trajectory tracking**: MoveIt generates an offline trajectory, but disturbances occur during execution. Replace PID with MPC as the tracking controller: prediction model is linearized operational-space dynamics $M\ddot{x} + C\dot{x} = F$, constraints are joint torque/velocity limits, cost is tracking error. The acados framework can solve within 1 ms.

3. **Autonomous vehicle path following**: vehicle dynamics via bicycle model (nonlinear), NMPC minimizes lateral deviation and heading error under lane boundary + speed constraints. CasADi + IPOPT is the academic standard; production uses acados or custom QP.

**Code skeleton** (Python, CasADi-style linear MPC):

```python
import casadi as ca
import numpy as np

# Build MPC problem
N = 20  # prediction horizon
nx, nu = 4, 2  # state / control dimensions

# Decision variables
U = ca.MX.sym('U', nu, N)
X = ca.MX.sym('X', nx, N + 1)
P = ca.MX.sym('P', nx + nx * N)  # initial state + reference trajectory

# Cost + constraints (skeleton)
cost = 0
constraints = [X[:, 0] - P[:nx]]  # initial state constraint
for k in range(N):
    x_ref = P[nx + k * nx : nx + (k+1) * nx]
    cost += ca.mtimes([(X[:, k] - x_ref).T, Q, (X[:, k] - x_ref)])
    cost += ca.mtimes([U[:, k].T, R, U[:, k]])
    x_next = A @ X[:, k] + B @ U[:, k]  # linear model
    constraints.append(X[:, k+1] - x_next)

# nlp_solver = ca.nlpsol('solver', 'ipopt', {...})
# sol = nlp_solver(x0=..., lbx=u_min, ubx=u_max, ...)
# u_optimal = sol['x'][:nu]  # take only the first step
```

<details>
<summary>Deep dive: complete runnable linear MPC example (Python + OSQP)</summary>

```python
import numpy as np
import osqp
from scipy import sparse

def build_linear_mpc(A, B, Q, R, Qf, N, x_min, x_max, u_min, u_max):
    """
    Build QP matrices for linear MPC (offline, done once).
    A: (nx, nx), B: (nx, nu), Q/R/Qf: weights, N: horizon
    Returns an OSQP solver object.
    """
    nx, nu = B.shape

    # Cost matrix H = blkdiag(Q, R, Q, R, ..., Qf)
    P_blocks = []
    for k in range(N):
        P_blocks.append(sparse.block_diag([Q, R], format='csc'))
    P_blocks.append(sparse.csc_matrix(Qf))
    P_cost = sparse.block_diag(P_blocks, format='csc')

    # Equality constraints: x_{k+1} = A x_k + B u_k
    # Arranged as Aeq @ z = beq (z = [x0, u0, x1, u1, ..., xN])
    # Inequality constraints: lb <= z <= ub
    nz = (nx + nu) * N + nx  # total decision variables

    # Dynamics constraint matrix
    Ax_list = []
    for k in range(N):
        row = np.zeros((nx, nz))
        idx_xk = k * (nx + nu)
        idx_uk = idx_xk + nx
        idx_xk1 = (k + 1) * (nx + nu)
        row[:, idx_xk:idx_xk + nx] = A
        row[:, idx_uk:idx_uk + nu] = B
        row[:, idx_xk1:idx_xk1 + nx] = -np.eye(nx)
        Ax_list.append(row)
    Aeq = sparse.csc_matrix(np.vstack(Ax_list))

    # Bound constraints
    lb = np.tile(np.concatenate([x_min, u_min]), N)
    lb = np.concatenate([lb, x_min])
    ub = np.tile(np.concatenate([x_max, u_max]), N)
    ub = np.concatenate([ub, x_max])

    # OSQP format: min 0.5 z'Pz + q'z, s.t. l <= Az <= u
    A_total = sparse.vstack([
        Aeq,
        sparse.eye(nz)
    ], format='csc')
    l_total = np.concatenate([np.zeros(N * nx), lb])
    u_total = np.concatenate([np.zeros(N * nx), ub])

    solver = osqp.OSQP()
    solver.setup(P_cost, np.zeros(nz), A_total, l_total, u_total,
                 warm_start=True, verbose=False,
                 eps_abs=1e-6, eps_rel=1e-6, max_iter=200)
    return solver


def mpc_step(solver, x0, x_ref_trajectory, nx, nu, N):
    """
    Online MPC single-step solve.
    x0: current state
    x_ref_trajectory: (N+1, nx) reference trajectory
    Returns u0: first optimal control action
    """
    # Update cost linear term q (for reference tracking)
    q = np.zeros((nx + nu) * N + nx)
    # ... (update q based on x_ref)

    # Update initial state constraint
    # solver.update(l=..., u=..., q=q)
    result = solver.solve()

    if result.info.status != 'solved':
        print(f"MPC solve failed: {result.info.status}")
        return np.zeros(nu)  # fallback

    u0 = result.x[nx:nx + nu]  # extract first control action
    return u0


# Example: double integrator (position + velocity)
dt = 0.01  # 100 Hz
A = np.array([[1, dt], [0, 1]])
B = np.array([[0.5 * dt**2], [dt]])
Q = np.diag([10.0, 1.0])   # heavier weight on position tracking
R = np.array([[0.1]])       # light control penalty → aggressive tracking
Qf = 10 * Q                # terminal cost
N = 30                      # 0.3 s look-ahead

solver = build_linear_mpc(
    A, B, Q, R, Qf, N,
    x_min=np.array([-10, -5]),
    x_max=np.array([10, 5]),
    u_min=np.array([-1.0]),  # torque limit
    u_max=np.array([1.0]),
)
# Control loop: u = mpc_step(solver, x_now, x_ref, nx=2, nu=1, N=30)
```

**Key points**:
- OSQP supports warm start — the previous solution initializes the next solve, typically 2-5x speedup
- Matrices are built once (offline); online updates only touch $q$ (linear term) and constraint bounds
- At 100 Hz, OSQP solves medium-scale problems ($N=30$, $nx=12$) in < 1 ms

</details>

<details>
<summary>Deep dive: NMPC acceleration techniques — from too-slow to 1 kHz</summary>

### Problem

Standard NMPC via IPOPT solves an NLP in 10-100 ms, far too slow for a 1 kHz (1 ms) control loop.

### Acceleration strategies (ranked by impact)

1. **RTI (Real-Time Iteration)**: instead of solving the NLP to convergence, perform only one SQP iteration per control cycle. This is the core idea behind acados. When the model does not change drastically between cycles, a single step is close enough to optimal.

2. **Warm Start**: shift the previous solution one step forward as the initial guess. $u^*_{1:N-1}$ shifts left, append a reasonable $u_{\text{init}}$ at the end.

3. **Sparsity exploitation**: the MPC KKT matrix is naturally banded sparse. acados uses HPIPM (High-Performance Interior Point Method) to exploit this structure directly, 10-100x faster than general-purpose IPOPT.

4. **Model simplification**:
   - Quadrupeds: SRBD (6-DoF centroid) instead of full 18-DoF
   - Manipulators: operational-space linearization instead of full nonlinear dynamics
   - Vehicles: bicycle model instead of full tire model

5. **Explicit MPC**: for small-scale linear MPC, precompute the optimal control law offline as a piecewise-affine function of the initial state. Online cost is just a table lookup, O(log n). Impractical when state dimension > 5 (partition count explodes).

6. **Learning-based MPC**: train a neural network to approximate the MPC policy; online cost is a single forward pass (< 0.1 ms). Requires safety guarantees — typically paired with a CBF (Control Barrier Function) safety filter.

### acados RTI typical flow

```python
from acados_template import AcadosOcp, AcadosOcpSolver

ocp = AcadosOcp()
ocp.model = my_nonlinear_model        # CasADi symbolic model
ocp.cost.cost_type = 'LINEAR_LS'
ocp.constraints.lbu = u_min
ocp.constraints.ubu = u_max
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # key: RTI mode
ocp.solver_options.tf = N * dt

solver = AcadosOcpSolver(ocp)

# Control loop
for step in range(sim_steps):
    solver.set(0, 'lbx', x_current)
    solver.set(0, 'ubx', x_current)  # fix initial state
    solver.solve()  # RTI: single QP iteration
    u = solver.get(0, 'u')
    x_current = simulate(x_current, u)
```

</details>

## Common Misconceptions

1. **"MPC can handle arbitrary nonlinear models"** — being able to model something does not mean it can be solved in real time. NMPC's NLP has no global optimality guarantee (nonconvex), and compute grows exponentially with model complexity. **Correct understanding**: industrial MPC overwhelmingly uses linearization + QP, or RTI-approximated NMPC. Full nonlinear MPC is practical only offline or at slow rates (< 10 Hz).

2. **"MPC only works for linear systems"** — quite the opposite. NMPC handles nonlinear models; it simply requires specialized solve techniques (SQP_RTI, warm start, structure exploitation). Quadruped locomotion, drone acrobatics, and autonomous driving all use NMPC. **The real question** is not whether nonlinear is possible, but whether you can solve it within your control period.

3. **"Once you use MPC you do not need to tune"** — MPC has more parameters to tune than PID: $Q$ (per-state weights), $R$ (control penalty), $Q_f$ (terminal cost), $N$ (horizon), sampling time $dt$, and model parameters. The $Q/R$ ratio directly sets tracking aggressiveness; $N$ too short causes instability, $N$ too long overruns compute. **Practical starting point**: Bryson's rule ($Q_{ii} = 1 / x_{i,\text{max}}^2$, $R_{jj} = 1 / u_{j,\text{max}}^2$), then fine-tune.

4. **"Receding horizon wastes compute — why not solve the full horizon once?"** — full-horizon optimization (Ch11 trajectory optimization) runs offline and cannot correct for feedback. MPC re-plans at every step with the latest state, naturally compensating for model error and disturbances. The cost is solving a QP/NLP each step, but each instance is small (only N steps). **Essence**: MPC = online closed-loop trajectory optimization.

## Situational Questions

<details>
<summary>Q1 (medium): A quadruped walking on a wet floor starts slipping. The MPC was tuned on dry ground. How do you diagnose and fix?</summary>

**Complete reasoning chain**:

1. **Root cause**: slipping means ground reaction forces exceed the friction cone $\|f_t\| \le \mu f_n$. The wet surface has $\mu \approx 0.2$ instead of the tuned $\mu \approx 0.6$, but MPC still uses 0.6.
2. **Immediate fix**: reduce MPC's $\mu$ parameter to a conservative value (0.15-0.2). The tighter cone forces MPC to reduce tangential forces → slower gait, no slip.
3. **Advanced: online friction estimation**: use contact force sensors + slip detection at the feet to estimate $\hat{\mu}$ in real time, feeding it back into the MPC constraints.
4. **Trap to avoid**: do not try to fix slipping by increasing $Q$ to "track harder" — this makes MPC push harder tangentially, which is exactly wrong on a low-friction surface.

**What the interviewer wants to hear**: MPC's core advantage is constraint handling. Friction estimation → constraint update → MPC auto-adjusts strategy — articulating this chain is the key.

</details>

<details>
<summary>Q2 (medium-hard): Your NMPC runs at 50 Hz (20 ms/step), but the control loop needs 200 Hz (5 ms). No hardware upgrade available. How do you compress to under 5 ms?</summary>

**Complete reasoning chain**:

1. **Profile first**: confirm the bottleneck is NLP solving (usually is), not model assembly or matrix construction.
2. **RTI (Real-Time Iteration)**: switch from solving SQP to convergence to a single QP iteration per step. acados SQP_RTI mode. Typical 5-10x speedup.
3. **Warm start**: shift the previous solution one step forward as the initial guess, reducing QP iterations.
4. **Sparsity exploitation**: switch from general-purpose IPOPT to HPIPM, which directly exploits MPC's banded-sparse structure.
5. **Model simplification**: if using full-body dynamics, switch to a reduced model (SRBD, centroidal dynamics) to shrink NLP dimension.
6. **Condensing vs Sparsity**: high state dimension → sparse formulation (keep $x$); high control dimension → condensing (eliminate $x$, keep only $U$).
7. **Last resort**: shorten $N$ (verify stability still holds), or use a cascaded architecture with slow NMPC outer loop + fast linear MPC inner loop.

**What the interviewer wants to hear**: not just "use a faster solver," but the specific combination of RTI + warm start + structure exploitation and why each helps.

</details>

<details>
<summary>Q3 (hard): MPC tracks perfectly in sim, but the real robot has 3 ms communication delay plus model uncertainty, causing severe trajectory deviation. How do you compensate?</summary>

**Complete reasoning chain**:

1. **Delay compensation**: the "current state" $x_k$ used by MPC is actually 3 ms old. Fix: forward-predict the state before solving — $\hat{x}_{k+d} = f(x_k, u_{k-d:k})$ — so MPC plans from the state that will exist when the solution is applied.
2. **Model correction**: add a Disturbance Observer (DOB) that estimates the gap between model and reality as an external force $\hat{d}$, injected into the prediction model $x_{k+1} = f(x_k, u_k) + \hat{d}_k$.
3. **Robust MPC**: use tube-based MPC — the nominal trajectory is planned by MPC, and a linear ancillary controller keeps the real state within a tube around the nominal.
4. **Constraint tightening**: leave margin in constraints, $u_{\max}^{\text{MPC}} = u_{\max}^{\text{real}} - \Delta u_{\text{margin}}$, absorbing uncertainty within the margin.
5. **System ID**: run sweep signals on hardware to identify true dynamics parameters and update MPC's $A, B$ matrices.

**What the interviewer wants to hear**: delay compensation (state prediction) is the most commonly overlooked yet critical fix; DOB is the go-to industrial model correction; tube MPC is the academic standard for robust MPC.

</details>

<details>
<summary>Q4 (medium): A junior engineer asks "why not just use LQR — why the overhead of MPC?" How do you explain?</summary>

**Complete reasoning chain**:

1. **LQR is a special case of MPC**: LQR = infinite horizon + no constraints + linear model. LQR's gain $K = -R^{-1}B^TP$ is a constant matrix computed offline; online cost is a single matrix multiply.
2. **Constraint handling**: LQR computes $u = Kx$ with no guarantee that $u_{\min} \le u \le u_{\max}$. You can clamp externally, but post-clamp the control is no longer optimal. MPC handles constraints inside the optimization.
3. **Nonlinear systems**: LQR linearizes at one operating point and degrades far from it. NMPC can use the full nonlinear model.
4. **Time-varying reference**: LQR tracks a fixed setpoint easily, but tracking time-varying trajectories requires time-varying LQR, whose implementation complexity approaches MPC anyway.
5. **Bottom line**: simple system + no constraints + setpoint tracking → LQR is sufficient and faster. Constraints / nonlinear / time-varying trajectory → MPC is the correct choice.

**What the interviewer wants to hear**: clear statement that LQR is a special case of MPC, and constraint handling is MPC's defining advantage.

</details>

## Interview Angles

1. **Horizon vs compute trade-off** — the most fundamental MPC design knob. Bring out with: "Choosing $N$ is a stability-vs-compute trade-off — too short and the controller is myopic and unstable, too long and the QP grows beyond the control period. In practice I binary-search for the shortest stable horizon with closed-loop simulation, then verify the solver fits within the timing budget."

2. **Convex relaxation is the key to deployment** — separates textbook MPC from MPC that runs on hardware. Bring out with: "Real constraints are rarely convex out of the box — friction cones need SOCP relaxation, collision constraints need convex-hull approximation, nonlinear dynamics need sequential linearization (SQP_RTI). Turning nonconvex problems into efficiently solvable convex ones is the core competency of an MPC engineer."

3. **Hard constraint handling is MPC's fundamental advantage over PID** — the most frequently probed point. Bring out with: "PID can only do anti-windup after saturation — it is a post-hoc band-aid. MPC incorporates torque limits, velocity bounds, and friction cones into the optimization itself, guaranteeing that outputs always stay within the safe operating envelope. This is safety-critical for quadruped walking and autonomous driving."

4. **Delay compensation and robustness** — elevates "I know the theory" to "I have fought sim-to-real." Bring out with: "MPC that works perfectly in sim usually breaks on hardware because of communication delay and model mismatch. My standard recipe is state prediction before MPC to compensate delay, DOB to estimate unmodeled dynamics, and constraint tightening to absorb the rest."

5. **MPC and RL are complementary** — shows frontier awareness. Bring out with: "MPC has a model but is limited by model fidelity; RL does not need an explicit model but lacks safety guarantees. The industry trend is using RL to learn residual dynamics or warm-start MPC, or using MPC as an RL safety filter (CBF-MPC)."

## Further Reading

- **Rawlings, Mayne & Diehl, *Model Predictive Control: Theory, Computation, and Design*** — the MPC bible; Ch5 (stability) and Ch10 (NMPC) are high-frequency interview topics
- **acados documentation and examples** — currently the fastest open-source NMPC framework, the industrial reference for RTI + HPIPM
- **MIT Cheetah 3 MPC paper (Di Carlo et al., 2018)** — the canonical quadruped MPC architecture: SRBD + friction-cone QP at 1 kHz
- **CasADi tutorials** — the Swiss-army knife for symbolic modeling + NLP solving, essential tooling for learning MPC
- **OCS2 (Optimal Control for Switched Systems)** — ETH open-source framework supporting switched-system MPC (quadruped gait transitions), well integrated with ROS 2
- **Paper: *Neural Network-based Model Predictive Control*** — introduction to learning-based MPC, NN-approximated policy + CBF safety filter
- **OSQP documentation** — embedded QP solver with warm start, used by MIT Cheetah and Boston Dynamics
