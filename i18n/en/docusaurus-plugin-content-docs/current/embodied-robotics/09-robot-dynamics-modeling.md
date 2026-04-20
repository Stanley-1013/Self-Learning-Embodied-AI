---
title: "Dynamics Modeling (Newton-Euler and Lagrangian)"
prerequisites: ["08-inverse-kinematics"]
estimated_time: 60
difficulty: 4
tags: ["dynamics", "newton-euler", "lagrangian", "computed-torque", "passivity", "adaptive-control", "contact", "featherstone"]
sidebar_position: 9
---

# Dynamics Modeling (Newton-Euler and Lagrangian)

## You Will Learn

- Distinguish kinematics from dynamics in two sentences, and precisely state the roles of forward/inverse dynamics in simulators and controllers
- Recognize where a problem lives in the dynamics stack when you see "can't keep up at high speed", "sim-to-real torque mismatch", "the robot bounces on contact", or "performance differs 3x between cold and warm operation" -- is it parameters, friction, contact, flexibility, or actuator bandwidth?
- In an interview, explain in under two minutes: computed-torque control, Passivity-Based Control, Featherstone ABA, Contact Wrench Cone QP, and Teacher-Student Privileged Learning
- Master the industrial 8-step System Identification pipeline (including the LMI physical-consistency check), and understand when Slotine-Li adaptive control drifts (Persistent Excitation condition)
- Understand the Underactuated Dynamics, angular momentum conservation, and Contact Wrench Cone that tie a floating-base humanoid/quadruped together through its Whole-Body Controller

## Core Concepts

**Precise definition**: **Robot dynamics** studies the relationship between forces/torques and motion. Where kinematics asks "how do the geometries move?", dynamics asks "what forces are required to move like this?" and "given these forces, how will it move?" The central artifact of the discipline is an **equation of motion** that ties together inertia, Coriolis, gravity, and (for free-flying or legged systems) contact forces.

**Position in the sense --> plan --> control loop**:
- **Input**: joint position $q$, velocity $\dot{q}$, acceleration $\ddot{q}$ (for inverse dynamics) or joint torques $\tau$ (for forward dynamics)
- **Output**: required torques $\tau$ (inverse dynamics) or resulting accelerations $\ddot{q}$ (forward dynamics)
- **Downstream consumers**: Computed Torque Control (CTC) feedforward, Passivity-Based Control, impedance control torque compensation, TrajOpt/NMPC Hessian (inverse dynamics); physics-simulator integrators (MuJoCo / PyBullet / Isaac Sim), MPC rollouts, differentiable-simulator gradient paths (forward dynamics)
- **Loop node**: sits at the heart of **control**, but reaches up into planning (TrajOpt uses dynamics as constraints) and down into actuation (FOC current loop sources the torque). For floating-base robots it also reaches into perception (IMU + contact model infer terrain)

**One-line version**: "The dynamics model is the robot's muscle-and-bone owner's manual -- it lets the simulator predict the future, the controller deliver torque precisely, and the learning algorithm know which quadrant of the physical world it is operating in."

### The Standard Equation of Motion (Fixed Base)

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau
$$

- $M(q) \in \mathbb{R}^{n \times n}$: **inertia matrix**, symmetric positive definite. Physical meaning: how much torque is required to accelerate each joint at 1 rad/s$^2$, and it varies with pose (an extended arm has more inertia than a folded one).
- $C(q, \dot{q})\dot{q} \in \mathbb{R}^n$: **Coriolis + centrifugal** forces. Physical meaning: "phantom forces" arising from velocity cross-terms when multiple joints move simultaneously -- think of the sideways push when a car takes a sharp turn.
- $g(q) \in \mathbb{R}^n$: **gravity term**. Physical meaning: even at rest the motors must fight gravity to hold the arm up.
- $\tau \in \mathbb{R}^n$: joint torques (or forces), the controller's output.

**Inverse dynamics** (motion to torque) -- the basis of computed-torque feedforward:

$$
\tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q)
$$

**Forward dynamics** (torque to acceleration) -- what a physics simulator does every timestep:

$$
\ddot{q} = M(q)^{-1}\left[\tau - C(q, \dot{q})\dot{q} - g(q)\right]
$$

**Skew-symmetry of $\dot{M} - 2C$** (the keystone of passivity and Lyapunov stability):

$$
x^T\left[\dot{M}(q) - 2C(q, \dot{q})\right]x = 0, \quad \forall x
$$

Physical meaning: this property guarantees the system is passive -- Coriolis/centrifugal forces **do no work**, they only redistribute energy. Combined with the Lyapunov function $V = \tfrac{1}{2}\dot{q}^T M \dot{q}$, the time derivative cancels the Coriolis contribution and gives strict stability guarantees. This is the mathematical lifeline for every energy-based controller (PBC, IDA-PBC, Slotine-Li adaptive).

### The Two Main Formulations

| Property | Newton-Euler (RNEA) | Lagrangian |
|----------|---------------------|------------|
| Essence | Recursive force/moment balance | Energy differentiation |
| Complexity | $O(n)$ | $O(n^3)$ to $O(n^4)$ |
| Primary use | Real-time inverse dynamics | Offline symbolic derivation / analysis |
| Output | Direct torque vector $\tau$ | Explicit $M, C, g$ matrices |
| Strength | Very fast, embedded-friendly | Structured, amenable to stability proofs |
| Industry default | 1 kHz+ real-time control | Control-theoretic analysis, System ID |

<details>
<summary>Deep dive: Featherstone spatial algebra, and why RNEA/CRBA/ABA are all linear-time</summary>

**Why separate 3D vectors are wasteful**: using $\omega, v$ separately forces every coordinate transform to apply rotation+translation on each vector independently, duplicating matrix multiplies and cross products. FLOPs explode.

**Plucker coordinates (spatial vectors)**: fuse motion and force into 6D vectors:

$$
\nu = \begin{bmatrix}\omega \\ v\end{bmatrix} \in \mathbb{R}^6, \quad f = \begin{bmatrix}n \\ f_{\text{linear}}\end{bmatrix} \in \mathbb{R}^6
$$

Spatial inertia becomes a $6 \times 6$ matrix, and each kinematic/dynamic propagation is a single 6x6 times 6x1 multiply -- FLOPs minimized.

**RNEA (Recursive Newton-Euler Algorithm) -- O(n) inverse dynamics**:
- **Forward pass** (base -> tip): `v_i = v_{i-1} + S_i q_i_dot`, propagate spatial velocity/acceleration ("how is each link moving?")
- **Backward pass** (tip -> base): `f_i = I_i a_i + v_i cross_f (I_i v_i)`, project onto joint axis `tau_i = S_i^T f_i` ("how much torque must each motor supply?")

**CRBA (Composite Rigid Body Algorithm) -- O(n^2) to build M(q)**: recursively accumulate subtree inertia from tip to base, yielding the full $M$ matrix.

**ABA (Articulated Body Algorithm) -- O(n) forward dynamics**:
1. Define **articulated inertia** -- the effective inertia looking down each subchain
2. Three O(n) passes: forward (velocity bias) -> backward (articulated inertia) -> forward (acceleration)
3. **Completely bypasses $M^{-1}$** -- this is ABA's essence

**Interview trap: is ABA always faster than CRBA + Cholesky?**
- Theoretically O(n) vs O(n^3), but constants matter
- At low DoF (n=6,7), Cholesky heavily leverages SIMD + cache locality -- the constant is tiny
- **In practice ABA only wins decisively for n > 9** -- a 6-axis arm does not necessarily require ABA, but a 30+ DoF humanoid must

**Spatial cross-product duality (mix them up and your math is wrong)**:
- Motion x Motion: `nu_1 cross_m nu_2`
- Motion x Force: `nu cross_f f = -(nu cross_m)^T f`
- Pinocchio maintains two separate APIs; using the wrong one silently corrupts the result

**Real-world failure**: a 30-DoF humanoid using `M^-1 (tau - h)` for forward dynamics consumed 2 ms per tick and missed the 1 kHz deadline. Switching to ABA dropped it to 50 us.

</details>

<details>
<summary>Deep dive: full RNEA recursion + engineering tricks to extract M, C, g from RNEA</summary>

**Forward pass** (base -> end-effector, $i = 1 \ldots n$):

$$
\omega_i = R_i^T (\omega_{i-1} + \dot{q}_i \hat{z})
$$
$$
\dot{\omega}_i = R_i^T (\dot{\omega}_{i-1} + \ddot{q}_i \hat{z} + \omega_{i-1} \times \dot{q}_i \hat{z})
$$
$$
\ddot{p}_i = R_i^T \ddot{p}_{i-1} + \dot{\omega}_i \times r_i + \omega_i \times (\omega_i \times r_i)
$$

**Backward pass** (end-effector -> base, $i = n \ldots 1$):

$$
f_i = m_i \ddot{p}_{c_i} + R_{i+1} f_{i+1}
$$
$$
n_i = I_i \dot{\omega}_i + \omega_i \times (I_i \omega_i) + R_{i+1} n_{i+1} + s_i \times m_i \ddot{p}_{c_i} + r_{i+1} \times R_{i+1} f_{i+1}
$$
$$
\tau_i = n_i^T \hat{z}
$$

**Engineering tricks to extract $M, C, g$ from RNEA** (when the control law needs explicit $M$):

- $g(q) = \text{RNEA}(q, 0, 0)$ -- set base acceleration to $-g$ and everything else to zero
- $C(q, \dot{q})\dot{q} = \text{RNEA}(q, \dot{q}, 0) - g(q)$ -- zero acceleration + real velocity
- Row $i$ of $M(q)$ = $\text{RNEA}(q, 0, e_i) - g(q)$ -- unit-vector method, $n$ RNEA calls assemble full $M$

**Why analytic derivatives matter for NMPC/RL**:
- NMPC needs $\partial \tau / \partial q, \partial \tau / \partial \dot{q}$ as the optimizer's Jacobian
- Finite difference: 7-DoF requires 21 RNEA calls + truncation error; 15 ms per step blows the 1 kHz budget
- **Pinocchio `computeRNEADerivatives`**: single extended recursion, >10x faster, down to 1.2 ms
- API: `pinocchio::computeRNEADerivatives(model, data, q, v, a, dtau_dq, dtau_dv, dtau_da)`

**Real-world failure**: a Franka 7-DoF NMPC using finite-difference Jacobians took 15 ms per step, missing 1 kHz. Switching to analytical gradients dropped it to 1.2 ms.

</details>

<details>
<summary>Deep dive: Lagrangian energy derivation, Christoffel symbols, closed-form 2-link M/C/g</summary>

**Lagrangian**:

$$
\mathcal{L}(q, \dot{q}) = T(q, \dot{q}) - V(q), \quad T = \tfrac{1}{2}\dot{q}^T M(q)\dot{q}
$$

**Euler-Lagrange equation**:

$$
\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i
$$

Expanded, this yields exactly $M\ddot{q} + C\dot{q} + g = \tau$.

**Christoffel symbols of the Coriolis matrix** (this is where the $\dot{M} - 2C$ skew symmetry comes from):

$$
C_{ij} = \sum_{k=1}^n c_{ijk} \dot{q}_k, \quad c_{ijk} = \tfrac{1}{2}\left(\frac{\partial M_{ij}}{\partial q_k} + \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{jk}}{\partial q_i}\right)
$$

**Closed-form 2-link planar arm** (useful for interview whiteboard + sanity-checking code):

$$
M(q) = \begin{bmatrix}
m_1 l_{c1}^2 + m_2(l_1^2 + l_{c2}^2 + 2l_1 l_{c2}\cos q_2) + I_1 + I_2 & m_2(l_{c2}^2 + l_1 l_{c2}\cos q_2) + I_2 \\
m_2(l_{c2}^2 + l_1 l_{c2}\cos q_2) + I_2 & m_2 l_{c2}^2 + I_2
\end{bmatrix}
$$

Notice $M$ depends on $q_2$ (elbow angle) but not $q_1$ (shoulder angle) -- the shoulder motor feels different inertia as the elbow bends.

**When to use Lagrangian**:
- Offline symbolic derivation (SymPy / MATLAB) followed by code generation
- Educational analysis of low-DoF systems (2-3 DoF hand derivation)
- Control-theoretic stability proofs (substitute into Lyapunov functions)
- Building the regressor matrix for System Identification

**When not to use**: real-time control of $n \geq 6$ systems. Symbolic expansion grows explosively, and even evaluating $M(q)$ online is $O(n^3)$.

</details>

### Passivity-Based Control (PBC) -- Control as Energy Shaping

**Core philosophy**: view the robot as an **energy converter**. Stored energy $\leq$ input energy $+$ dissipated energy, so the system **cannot diverge**.

**PBC vs PD+G(q)**:
- PD+G(q): guarantees only set-point stability
- PBC: actively **shapes energy** -- high-speed tracking and contact interactions are strictly passive

**IDA-PBC (Interconnection & Damping Assignment)**: modifies the closed-loop effective mass and potential, endowing the robot with a virtual "passive spring-damper shell" -- it cannot go wild on contact.

**Trap**: digital discretization delay + sensor-filter phase lag can **inject spurious energy**, breaking passivity and causing high-frequency contact oscillations. In practice a Passivity Observer + Passivity Controller (POPC) monitors the energy budget continuously.

**Platforms**: Franka Panda and KUKA iiwa joint-torque modes are Passivity-based controllers at their core -- this is why they can safely do human-robot interaction.

### Floating-Base Dynamics -- the Bread and Butter of Humanoid / Quadruped

**The brutal fact of 6 unactuated DoF**: the base has 3 translational + 3 rotational DoF and zero motors; the robot can only change the base via **ground reaction forces (GRF)** or thrust -- this is **Underactuated Dynamics**.

$$
\begin{bmatrix}M_{\text{base}} & M_{bj} \\ M_{bj}^T & M_{jj}\end{bmatrix}\begin{bmatrix}\ddot{x}_{\text{base}} \\ \ddot{q}_j\end{bmatrix} + h = \begin{bmatrix}0 \\ \tau\end{bmatrix} + \begin{bmatrix}J_c^T F_c \\ J_{c,j}^T F_c\end{bmatrix}
$$

Physical meaning: the top block has RHS equal to $0 + J_c^T F_c$ -- base acceleration is **entirely determined by contact force**. Motors only indirectly change the base by driving the joints that produce $F_c$.

**The golden rule -- angular momentum conservation**: during a flight phase there are no contact forces and gravity produces no torque about the center of mass -- so **total angular momentum $k$ is strictly conserved**. Cats flipping mid-air, MIT Cheetah's backflip, gymnasts tucking to spin faster -- all follow this.

**Centroidal Momentum Matrix (CMM)**:

$$
A(q)\dot{q} = \begin{bmatrix}l \\ k\end{bmatrix}
$$

$l$ = linear momentum, $k$ = angular momentum. $A(q)$ maps joint velocities to centroidal momentum and is the high-level planning surface for WBC. Pinocchio API: `pinocchio::ccrba(model, data, q, v)` -> `data.Ag, data.hg`.

<details>
<summary>Deep dive: Contact Wrench Cone (CWC) and multi-contact QP -- why ZMP is obsolete</summary>

**Why ZMP falls apart**:
- Assumes all contacts are on one horizontal plane
- Assumes infinite friction
- A humanoid "one hand on wall + two feet on different stairs" in 3D is physically impossible to express in ZMP

**CWC unifies four contact constraints**:
1. **Normal pressure** $f_z > 0$ (ground cannot pull the robot)
2. **Coulomb friction cone** $\sqrt{f_x^2 + f_y^2} \leq \mu f_z$ (no slip)
3. **CoP constraint** (center of pressure within the foot's support polygon, no flipping)
4. **Yaw torque constraint** (feet do not spin in place)

Linearized, these become a polyhedral cone $W \cdot F_c \geq 0$.

**CWC-QP + WBC layered architecture**:

$$
\min_{F_c} \|\sum F_c - m(\ddot{p}_{\text{des}} - g)\|^2 + \|\sum (p_i \times F_c) - \dot{k}_{\text{des}}\|^2
$$
$$
\text{s.t. } W \cdot F_c \geq 0
$$

**Why this is the hidden rule of embodied AI**: Tesla Optimus, Figure, Unitree H1 -- every multi-contact maneuver (arms hugging a box + legs standing) runs CWC-QP at the bottom of the stack.

**Incline trap**: if CWC is not rotated from world frame into the local contact frame, normal force is mis-estimated and the robot splits into a split-leg slip on startup.

**Real-world failure**: a quadruped landing from a jump without constraining $\dot{k}$ in its WBC -- the front legs touched first and generated a huge pitching moment, causing a forward flip into the ground. Fix: add a strong penalty on $\dot{k}$ in the MPC cost.

</details>

**Common APIs (industry toolchain)**:

| Layer | Package | Example |
|-------|---------|---------|
| Fast C++ | Pinocchio | `rnea(model, data, q, v, a)` -> $\tau$ |
| Fast C++ | Pinocchio | `aba(model, data, q, v, tau)` -> $\ddot{q}$ |
| Analytic derivatives | Pinocchio | `computeRNEADerivatives(...)` -> $\partial\tau/\partial q, \partial\tau/\partial \dot{q}$ |
| Centroidal momentum | Pinocchio | `ccrba(...)` -> CMM + $l, k$ |
| Symbolic / AD | CasADi + Pinocchio | Auto-differentiation + C codegen, NMPC friendly |
| Simulator | MuJoCo | `mj_forward` / `mj_inverse` (soft contact) |
| Simulator | Drake | `CalcInverseDynamics` (strict LCP available) |
| Differentiable sim | MuJoCo MJX / Brax / Dojo | JIT + `jax.grad` through physics |
| ROS 2 | KDL | `ChainIdSolver_RNE::CartToJnt(...)` |

## Intuition

**Analogy: driving on a mountain road**:
- **$M(q)$ = vehicle mass** (and it changes with pose): extended arm = loaded vehicle, folded arm = empty vehicle
- **$C(q, \dot{q})\dot{q}$ = the sideways push through a turn**: a phantom force from simultaneous high-speed motion
- **$g(q)$ = throttle on an uphill**: even at rest you must push to resist gravity
- **$\tau$ = throttle/brake output**: the controller's command to motors

**Contact dynamics analogy: emergency brake vs gentle steering**:
- Rigid-body collision is like slamming the brakes -- instant impulse, velocity jump
- Soft contact is like ABS kicking in -- the system allows tiny "penetration" in exchange for continuity
- RL trained on soft contact has backpropagatable gradients; Drake's strict LCP is like a manual transmission -- accurate but slow

**Simulator validation experiments**:

1. **MuJoCo gravity compensation**: set all control torques to $\tau = g(q)$ (pure gravity compensation) and the arm should float in any pose. Under-compensate and it sags; over-compensate and it drifts up.
2. **High-speed swing observation**: in Isaac Sim run a fast joint swing with PD only; you will see large end-effector tracking error ($C$ and $M\ddot{q}$ are uncompensated). Add computed-torque feedforward and error shrinks dramatically.
3. **Pose-dependent inertia**: in PyBullet command the same torque to a joint with the arm extended vs folded -- acceleration is clearly smaller when extended (since $M(q)$ is larger).
4. **Angular momentum conservation**: in MuJoCo flail a humanoid's arms mid-air and watch the torso counter-rotate (total angular momentum conserved to zero -- any local disturbance is absorbed by the counter-rotation).
5. **Contact stiffness tuning**: play with MuJoCo's `solimp / solref`. Too soft and RL learns to cheat by clipping through objects; too hard and the timestep cannot track the stiff response, causing explosive bouncing. Tuning this by hand teaches you more about sim-to-real than any paper.

## Implementation Link

**Four representative engineering scenarios**:

1. **Computed Torque Control (CTC)**: inverse dynamics feedforward + PD feedback. The feedforward cancels ~90% of the nonlinearity, leaving PD to mop up the residual. The error dynamics become linear second-order: $\ddot{e} = K_p e + K_d \dot{e}$.

2. **Physics simulator inner loop**: every MuJoCo / PyBullet timestep computes forward dynamics -- take the policy's $\tau$, compute $\ddot{q}$, integrate. Simulation quality is capped by parameter accuracy.

3. **Impedance control with dynamics compensation**: impedance control needs inverse dynamics for gravity compensation + inertia decoupling so the end-effector actually feels like the desired mass-spring-damper. Without dynamics compensation, impedance control loses fidelity at high speed or under load.

4. **TrajOpt and NMPC**: trajectory optimization treats dynamics as equality constraints. iLQR needs first-order Jacobians, DDP needs second-order Hessians, CITO (Contact-Implicit TrajOpt) uses complementarity constraints to let the optimizer **discover footsteps on its own** -- the workhorse behind Atlas parkour and MIT Cheetah running gaits.

**Code skeleton** (C++, Pinocchio):

```cpp
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/parsers/urdf.hpp>

pinocchio::Model model;
pinocchio::urdf::buildModel("robot.urdf", model);
pinocchio::Data data(model);

// 1. Inverse dynamics (RNEA, O(n))
Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

// 2. Forward dynamics (ABA, O(n))
Eigen::VectorXd ddq = pinocchio::aba(model, data, q, v, tau);

// 3. Full M(q) (CRBA, O(n^2))
pinocchio::crba(model, data, q);   // fills data.M (upper triangle)

// 4. Analytical RNEA derivatives (for NMPC)
pinocchio::computeRNEADerivatives(model, data, q, v, a);
// data.dtau_dq, data.dtau_dv, data.M (dtau_da)

// 5. Centroidal momentum matrix (floating-base robots)
pinocchio::ccrba(model, data, q, v);
// data.Ag is CMM, data.hg is [l; k]
```

<details>
<summary>Deep dive: full Python implementation -- Pinocchio + CTC + gravity compensation + passivity monitor</summary>

```python
import numpy as np
import pinocchio as pin

# === Load model ===
model = pin.buildModelFromUrdf("panda.urdf")
data = model.createData()
nq = model.nq

# === Basic dynamics ===
q = np.zeros(nq)
v = np.zeros(nq)
a = np.zeros(nq)

# RNEA: O(n) inverse dynamics
tau = pin.rnea(model, data, q, v, a)

# ABA: O(n) forward dynamics (bypasses M^-1 entirely)
tau_in = np.ones(nq) * 0.5
ddq = pin.aba(model, data, q, v, tau_in)

# Explicit M, C, g when needed
M = pin.crba(model, data, q)                                  # O(n^2)
pin.computeCoriolisMatrix(model, data, q, v)
C = data.C
g = pin.computeGeneralizedGravity(model, data, q)

# === Computed Torque Control ===
def computed_torque_control(q, v, q_des, v_des, a_des, Kp, Kd):
    """Feedforward RNEA + PD feedback => linear second-order error dynamics."""
    e = q_des - q
    de = v_des - v
    a_cmd = a_des + Kp @ e + Kd @ de
    return pin.rnea(model, data, q, v, a_cmd)

Kp = np.diag([100.0] * nq)
Kd = np.diag([20.0] * nq)  # near-critical damping: Kd = 2*sqrt(Kp)

# === Pure gravity compensation (teaching demo: arm floats) ===
def gravity_compensation(q):
    return pin.computeGeneralizedGravity(model, data, q)

# === Passivity energy audit (POPC skeleton) ===
def energy_audit(q, v, tau_applied, dt, energy_state):
    """
    E_total = E_kinetic + E_potential - integral of tau^T v dt
    At any moment E_total must be non-increasing (definition of a passive system).
    """
    pin.crba(model, data, q)
    M = data.M
    T = 0.5 * v @ M @ v                              # kinetic energy
    V = pin.computePotentialEnergy(model, data, q)   # potential energy
    work_in = tau_applied @ v * dt                   # work input this step
    energy_state["input"] += work_in
    E_total = T + V
    passivity_residual = E_total - energy_state["input"]
    # If passivity_residual grows monotonically, the system is injecting spurious
    # energy (discretization error) -- lower the control gain.
    return passivity_residual

# === Verify Mdot - 2C skew-symmetry ===
def verify_skew_symmetry(q, v):
    pin.crba(model, data, q)
    pin.computeCoriolisMatrix(model, data, q, v)
    # Finite-difference approximation of Mdot
    eps = 1e-6
    q_next = q + v * eps
    pin.crba(model, data, q_next)
    M_next = data.M.copy()
    pin.crba(model, data, q)
    M_now = data.M.copy()
    M_dot = (M_next - M_now) / eps
    S = M_dot - 2 * data.C
    asym_error = np.linalg.norm(S + S.T)  # should be close to zero
    return asym_error
```

**Key points**:
- `rnea` is the recursive Newton-Euler algorithm, O(n)
- `crba` builds the full inertia matrix, O(n^2); `aba` uses the Articulated Body Algorithm, O(n)
- CTC gains should be tuned with the inertia matrix in mind; $K_d = 2\sqrt{K_p}$ is a critical-damping starting point
- Passivity monitoring is essential for contact interactions -- sensor filtering and discretization both leak energy

</details>

<details>
<summary>Deep dive: the industrial 8-step System Identification pipeline (with LMI physical-consistency check)</summary>

Model accuracy depends on parameter quality. CAD-supplied mass and inertia are only starting estimates; real robots have cables, reducers, end-effectors, and gearbox grease that CAD never captures. The industry standard is 8 steps:

**Step 1: analytical model derivation**
- SymPy / Pinocchio URDF -> regressor matrix $Y \cdot \pi$
- The dynamics equation becomes linear in parameters: $\tau = Y(q, \dot{q}, \ddot{q}) \cdot \pi$, where $\pi$ is the vector of identifiable parameters (masses, centers of mass, 6 inertia tensor entries per link, friction coefficients)

**Step 2: friction model extension**
- Basic: $\tau_f = F_c \text{sign}(\dot{q}) + F_v \dot{q}$
- Advanced: add Stribeck term $F_s e^{-|\dot{q}|/v_s} \text{sign}(\dot{q})$
- Top tier: LuGre dynamic model (see Q3 below)

**Step 3: excitation trajectory design**
- **Objective**: minimize $\text{cond}(Y)$ so every parameter is fully excited
- **Method**: 5th-order Fourier series; optimize frequencies/amplitudes with MATLAB `fmincon`
- **Constraints**: joint limits, velocity limits, self-collision

**Step 4: 1 kHz data collection on hardware**
- Franka FCI / UR RTDE: record $(q, \dot{q}, \tau_{\text{measured}})$
- $\ddot{q}$ from numerical differentiation of $\dot{q}$

**Step 5: zero-phase low-pass filtering with filtfilt**
- 4th-order Butterworth at 20 Hz cutoff
- **filtfilt** (forward + backward) is critical -- zero phase delay, does not amplify high-frequency noise in $\ddot{q}$

**Step 6: weighted least squares**

$$
\hat{\pi} = (Y^T W Y)^{-1} Y^T W \tau_{\text{measured}}
$$

Weight matrix $W$ reflects the inverse variance of measurement noise per joint.

**Step 7: physical consistency via LMI/SDP**
- This step separates academia from industry
- Plain WLS can produce **negative masses, non-positive-definite inertia tensors** (mathematically optimal, physically illegal)
- Constrain with Linear Matrix Inequalities: $m_i > 0$, $I_i \succ 0$ (positive definite), $\text{tr}(I_i) >$ max eigenvalue (triangle inequality)
- Tooling: SciPy `cvxpy` + SDP solver (Mosek / SCS)

**Step 8: cross-validation**
- Design a new trajectory (different from the excitation trajectory)
- Compare predicted $\hat{\tau} = Y \hat{\pi}$ against measured $\tau$
- RMSE < 5% of rated torque is the pass threshold

**Pinocchio API**: `pinocchio::computeJointTorqueRegressor()` returns $Y$.

**Real-world failure**: UR5 skipped Step 7, identified a negative inertia entry, controller diverged, and the arm hit the table. Adding the LMI constraint stabilized it.

</details>

<details>
<summary>Deep dive: Slotine-Li adaptive control and the Persistent Excitation trap</summary>

**Slotine-Li adaptive control** structure:

1. **Sliding surface**: $s = \dot{e} + \Lambda e$, $\Lambda$ positive definite
2. **Control law**:

$$
\tau = Y(q, \dot{q}, \dot{q}_r, \ddot{q}_r) \hat{\pi} - K s
$$

where $\dot{q}_r = \dot{q}_d + \Lambda e$ is the "reference velocity"

3. **Parameter update law**:

$$
\dot{\hat{\pi}} = -\Gamma Y^T s
$$

$\Gamma$ is the positive-definite adaptive gain.

**Lyapunov proof**: $V = \tfrac{1}{2}s^T M s + \tfrac{1}{2}\tilde{\pi}^T \Gamma^{-1} \tilde{\pi}$. Differentiating and using $\dot{M} - 2C$ skew-symmetry cancels the Coriolis term -> $\dot{V} = -s^T K s \leq 0$ -> $e \to 0$ (but not necessarily $\hat{\pi} \to \pi$).

**Persistent Excitation (PE) trap**:
- $e \to 0$ does not imply parameter convergence
- Convergence requires $\int_t^{t+T} Y^T Y \, d\tau \succ \alpha I$ (the PE condition)
- If $Y$ is rank-deficient (the robot only moves along one curve), the system cannot distinguish "heavier payload" from "higher friction"; estimates drift along the unobservable subspace and can eventually land on a negative mass

**Engineering fixes**:
- Inject a tiny sinusoidal exploration signal so $Y$ stays full-rank (analogous to RL exploration noise)
- Project estimates back into the physically feasible set (Projection Operator)
- If you only need error convergence (not parameter convergence), live with the imperfection

**RLS vs Slotine-Li tradeoff**:

| Method | Convergence | Noise robustness | Use case |
|--------|-------------|------------------|----------|
| RLS + forgetting factor | Very fast (instantaneous payload pickup) | Noise-sensitive, covariance windup | UR5 gripping unknown tool |
| Slotine-Li | Slow | Strictly stable $e \to 0$ | High-DoF precision tracking |

**Platform example**: UR5 picking up an unknown tool runs RLS online to update equivalent inertia so the CTC feedforward stays accurate.

</details>

<details>
<summary>Deep dive: contact dynamics -- LCP / soft contact / MuJoCo tuning pitfalls</summary>

**Rigid-body collision is a hybrid system**:
- Velocity jumps (impulse) in microseconds
- ODE is discontinuous
- Generic Runge-Kutta integrators explode

**LCP (Linear Complementarity Problem) formulation** (used by Drake, ODE):
- Contact distance $\phi \geq 0$
- Contact force $\lambda \geq 0$
- Complementarity $\phi \cdot \lambda = 0$
- Coulomb friction cone: $\lambda_t \leq \mu \lambda_n$ -> NCP (nonlinear complementarity), expensive to solve

**MuJoCo soft contact**:
- Abandons strict LCP, permits tiny penetration -> continuous convex optimization
- `solimp` (stiffness/viscosity) + `solref` (damping/restitution time) -> contact force becomes a **continuous function** of penetration depth
- Gradients backpropagate through it (the foundation of differentiable simulators)

**XML example**:
```xml
<geom solref="0.02 1" solimp="0.9 0.95 0.001" friction="1 0.005 0.0001"/>
```

**Sim-to-real traps**:
- Contact stiffness too soft -> RL learns to "cheat by clipping through objects"
- Too hard -> timestep too large -> objects bounce explosively on contact (numerical instability)
- Slopes / stairs -> if you forget to rotate the friction cone into the local contact frame, you get a split-leg slip at startup

**Industry convention**: train RL in MuJoCo (fast, differentiable, soft contact); validate on Drake (strict LCP, slow but accurate).

**Precision assembly floor**: peg-in-hole micron-level tolerances **cannot trust simulated contact forces blindly**. You must combine Domain Randomization (contact stiffness +/- 50%, friction +/- 30%) with a real impedance controller as a backstop.

</details>

<details>
<summary>Deep dive: Neural Dynamics, Differentiable Simulators, and Teacher-Student architecture</summary>

**Grey-box residual learning**:

$$
\tau_{\text{total}} = \tau_{\text{RNEA}}(q, \dot{q}, \ddot{q}; \pi_{\text{base}}) + \tau_{\text{NN}}(q, \dot{q}, \ddot{q}, T_{\text{temp}})
$$

A physics model does the heavy lifting; the NN learns the residual (cables, temperature, unmodeled friction). Data efficiency is 10-100x better than a pure NN, and adaptability is far better than pure RNEA.

**GPR (Gaussian Process Regression) -- its killer feature**: it outputs not only a prediction but also uncertainty $\sigma^2$. When $\sigma$ is high, fall back to robust control (conservative actions); when $\sigma$ is low, trust the NN.

**Teacher-Student Privileged Learning** (the sim-to-real recipe of ETH ANYmal / Boston Dynamics):
1. **Teacher phase**: in simulation, read privileged information (ground $\mu$, mass $m$, terrain height map) -> train a perfect policy
2. **Student phase**: only proprioception is available (joint position/velocity history + noisy IMU)
3. **Implicit inference via RNN/LSTM**: from the "slip history in foot velocity" the Student implicitly infers $\mu$, distilling Teacher-level recovery skill

**Differentiable simulators (MuJoCo MJX, Brax, Dojo)**:
- Traditional REINFORCE has high-variance gradient estimates
- Differentiable simulators let $\partial s_{t+1}/\partial a_t$ flow through the physics engine
- "Monte Carlo blind search" becomes precise gradient descent -- sample efficiency 100x

**MPPI (Model Predictive Path Integral)**: GPU-parallel sampling of thousands of SRBD (Single Rigid Body Dynamics) trajectories -> softmax-weighted mixture -> real-time optimal control. A shared foundation under Tesla Optimus / Unitree H1.

**Neural ABA (rumored at Tesla/Figure)**: even at O(n), ABA is a CPU bottleneck when parallelizing tens of thousands of environments -> fit an NN to ABA -> GPU O(1) matmul -> million-scale parallel training becomes feasible.

**Interview angle**:
- Naive RL engineers hitting a sim2real gap blindly crank up Domain Randomization -> policies become conservative and rigid
- Dynamics-literate engineers set the action space to "desired centroidal acceleration" or "impedance $\Delta K/D$" and hand off to a WBC underneath -> **big brain RL + little brain WBC hybrid architecture**
- **Failure case**: pure model-free RL learned high-frequency chatter that exploited a physics bug to clip through walls; fix = reward $\tau^2$ penalty + action low-pass filter

</details>

<details>
<summary>Deep dive: TrajOpt, iLQR/DDP, Direct Collocation, CITO -- dynamics in trajectory optimization</summary>

**Why kinematic planning + late-stage time parameterization fails**: pure geometric obstacle avoidance ignores dynamic coupling; at high speed Coriolis and inertial terms explode geometrically -> **torque saturation** -> trajectory is infeasible.

**iLQR vs DDP derivative differences**:
- **iLQR**: first-order Jacobians $A = \partial f/\partial x, B = \partial f/\partial u$
- **DDP**: full second-order Hessian tensors $\partial^2 f/\partial x^2$
- Extremely dynamic maneuvers like backflips have enormous state curvature; only accurate second-order derivatives capture "the curvature of the inertia matrix with respect to pose" -> few iterations achieve quadratic convergence

**Shooting vs Collocation**:

| Method | Decision variables | Pros | Cons |
|--------|-------------------|------|------|
| **Shooting** | only $u$ | Simple to implement | Integration error compounds, gradient explodes |
| **Direct Collocation** | $x$ + $u$ | Can boldly explore physically infeasible intermediate states before converging | Larger optimization problem |

Collocation encodes dynamics as equality constraints: $x_{k+1} - x_k - f(x_{k+1}, u_k) \cdot dt = 0$. IPOPT explores freely -- **very robust**.

**CITO (Contact-Implicit TrajOpt)**:
- Traditionally you must predefine the mode sequence ("which foot lands when")
- CITO treats contact distance $\phi$ and contact force $\lambda$ as complementarity constraints $\phi \geq 0, \lambda \geq 0, \phi \cdot \lambda = 0$
- The optimizer **discovers footsteps automatically** (Automatic Gait Discovery)
- The workhorse behind Atlas parkour and MIT Cheetah training

**Interview trap "a perfect trajectory runs in sim but diverges on the real robot" answer**:
- TrajOpt's open-loop solution is **extremely specialized and fragile** (right at the torque limit)
- 5% friction/backlash error is enough to make it infeasible
- **Engineering fix**: TrajOpt only supplies reference $(x_{\text{ref}}, u_{\text{ref}})$ + feedforward; you **must stack an LQR or WBC closed-loop underneath** to absorb residuals

</details>

## Common Misconceptions

1. **"The inertia matrix $M(q)$ is constant"** -- Wrong. $M(q)$ varies with pose; some entries can differ 3-5x between extended and folded. A PID tuned at one pose may oscillate or lag at another. **Correction**: recompute $M(q)$ each control tick, or use RNEA feedforward which bypasses it.

2. **"PID is enough, no dynamics model needed"** -- OK at low speed and light load. At high speed the Coriolis/centrifugal terms dominate; PID with no feedforward can only react, and tracking error grows rapidly with speed. **Correct practice**: CTC = inverse dynamics feedforward + PD feedback. Feedforward cancels ~90%; PD only mops up the residual.

3. **"Lagrangian is too slow, never use it"** -- Lagrangian is unsuitable for real-time ($O(n^3)$+), but its offline value is enormous: SymPy-derived $M, C, g$ -> code generation -> stability proofs -> System ID regressor construction. **Newton-Euler online, Lagrangian offline -- they are complementary**.

4. **"Friction is just linear"** -- Naive Coulomb + viscous ignores the Stribeck effect, static-to-dynamic discontinuity, LuGre dynamic bristles, harmonic-drive nonlinearity. In precision manipulation or force control this causes stick-slip oscillation directly. **Correct**: at minimum Stribeck + viscous; for stricter environments add LuGre + online Disturbance Observer.

5. **"Contact can be simulated by tuning stiffness"** -- `solimp / solref` are a pair of antagonistic knobs: too soft and RL cheats by clipping through; too hard and timestep can't track the stiff response, causing explosive bouncing. **Correct**: train in MuJoCo + validate in Drake + Domain Randomization + real-hardware closed-loop for critical assemblies (peg-in-hole, etc).

6. **"RL policies do not need a dynamics model"** -- Pure model-free is possible but sample-inefficient and has a stubborn sim-to-real gap. Modern practice is **Privileged Learning + Differentiable Sim + big-brain RL + little-brain WBC**. Dynamics is not replaced by RL -- it becomes structural prior knowledge baked into the action space.

## Practice Questions

<details>
<summary>Q1: Your CTC controller runs a 7-DoF arm doing high-speed pick-and-place, but the tracking error is 3x larger than expected and gets worse as you go faster. How do you diagnose systematically?</summary>

**Complete reasoning chain**:

1. **Velocity sensitivity test**: temporarily reduce speed to 10% and check whether the error returns to an acceptable range. Slow OK + fast not OK => the culprit is a velocity/acceleration-dependent term ($C\dot{q}$ or $M\ddot{q}$).
2. **Parameter comparison**: compute predicted torques with `pin.rnea` and compare to measured motor currents. RMSE > 20% => parameters are wrong.
3. **Run the 8-step SysID pipeline** (see "SysID deep dive"):
   - Analytical regressor -> friction extension -> Fourier excitation trajectory ($\min \text{cond}(Y)$)
   - 1 kHz collection -> zero-phase filtfilt
   - WLS solve -> **LMI physical consistency check** (avoid negative mass)
   - Cross-validate on a new trajectory, RMSE < 5%
4. **Friction check**: if static torque matches after SysID but high-speed is still off, the harmonic-drive wave-generator friction is unmodeled -> add Stribeck or LuGre
5. **Safety net**: add a Disturbance Observer to compensate unmodeled disturbances online: `d_est = LPF(tau_real - tau_model)`
6. **Avoid the trap**: do not just crank up PD gains -- with bad parameters high gains excite high-frequency oscillations

**Conclusion**: the root cause is almost always parameter inaccuracy + nonlinear friction. Standard fix: 8-step SysID (including LMI) + DOB safety net + Slotine-Li online adaptation if needed.

</details>

<details>
<summary>Q2: Your RL policy trained perfectly in MuJoCo; on the real robot the torque commands are completely wrong and the arm shakes. How do you investigate?</summary>

**Complete reasoning chain**:

1. **Static check: gravity compensation**. Hold the real robot in a pose and read actual motor output. Compare against MuJoCo `mj_inverse` in the same pose. Delta > 15% => mass/inertia parameters are wrong.
2. **Dynamic check: friction and contact stiffness**. Slowly oscillate the real robot and measure a torque-velocity curve -- look for the Stribeck hump; compare with MuJoCo's `solimp / solref` (often two orders of magnitude different from real ground).
3. **Actuator delay check**. MuJoCo applies torque instantly; real motor drivers have 1-2 ms of communication + current-loop delay, which is enough to destabilize a 1 kHz loop.
4. **Bandwidth check**. The RL policy may have learned high-frequency chatter (training had no action-smoothness penalty); the real current loop cannot track and resonates.
5. **Three-pronged fix**:
   - **Domain Randomization**: mass +/-20%, friction +/-30%, delay 0-3 ms, contact stiffness +/-50%
   - **System Identification**: update MuJoCo XML `<body mass>` / `<joint damping>` / `solref`
   - **Action smoothing**: low-pass filter or reward with $\dot{\tau}^2$ penalty
6. **Advanced: Teacher-Student Privileged Learning** (see deep-dive fold). Teacher reads privileged $\mu, m$, terrain; Student uses RNN to infer from history.
7. **Avoid the trap**: skipping SysID and only doing DR leaves DR ranges unable to cover the real bias (if gravity is 50% off, DR +/-20% won't cover it).

**Conclusion**: investigate in order "gravity -> friction -> contact -> delay -> bandwidth". SysID calibrates the nominal model; DR adds robustness; Teacher-Student gives implicit adaptation.

</details>

<details>
<summary>Q3: You need inverse dynamics at 1 kHz in an impedance controller on an ARM Cortex embedded system with a 200 us budget. How do you architect it?</summary>

**Complete reasoning chain**:

1. **Algorithm choice: RNEA, not Lagrangian**. $O(n)$ vs $O(n^3)$; 7-DoF RNEA is ~200 floating-point multiply-adds. At small $n$, ABA is not necessarily faster than CRBA+Cholesky (the constant from SIMD / cache locality is tiny).
2. **Toolchain: Pinocchio or CasADi code generation**
   - Offline: Pinocchio's `computeMinverse` + CasADi AD -> pure C code (flat assignments, no dynamic allocation, no virtual calls)
   - Generated code looks like `double c1 = cos(q[0]); double s1 = sin(q[0]); ...`
3. **Trigonometric caching**: 7 joints need only 14 `sin/cos` calls -- use CORDIC or tables instead of `libm`
4. **Memory strategy**: all intermediates in stack-allocated fixed-size arrays, never `malloc`. RNEA intermediates ($\omega, \dot{\omega}, f, n$ per link) use `std::array<Vector3d, N>`.
5. **OS configuration**:
   - PREEMPT_RT Linux kernel
   - `isolcpus` + `taskset` to pin the control thread to an isolated core
   - `mlockall` to lock memory pages against page faults
   - CPU governor set to `performance`
6. **Latency verification**: `clock_gettime(CLOCK_MONOTONIC)` to measure 99.99th percentile worst-case and confirm < 200 us (leave headroom for comms).
7. **If you need to squeeze further**: use `pinocchio::computeAllTerms` to batch RNEA + CRBA + Jacobian in a single traversal.

**Conclusion**: RNEA O(n) + offline codegen + zero-alloc + RT kernel + core isolation is the canonical embedded real-time dynamics stack. Measure worst-case, not average.

</details>

<details>
<summary>Q4: "If the dynamics model has a poor condition number, what are the consequences and how do you fix it?"</summary>

**Complete reasoning chain**:

1. **Definition**: $\kappa(M) = \sigma_{\max}/\sigma_{\min}$. Large $\kappa$ => $M$ is nearly singular.
2. **When it happens**: arm fully extended, some joint inertias vastly different -> $\kappa$ can reach 100:1 or more.
3. **Consequences**:
   - Forward dynamics via $\ddot{q} = M^{-1}(...)$ amplifies numerical error by $\kappa$ -- finite floating-point precision turns into orders-of-magnitude acceleration error
   - Controller feedforward torques drift -- some joints over-compensate, others under-compensate -> distorted trajectories
4. **Fixes**:
   - **Avoid explicit $M^{-1}$**: use Cholesky factorization to solve $M \ddot{q} = b$ ($M$ is positive definite so it always succeeds)
   - **Use ABA**: Pinocchio's `aba()` computes forward dynamics without $M^{-1}$ -- this is the numerical essence of Featherstone's algorithm
   - **Plan around ill-conditioned poses**: add $\kappa(M(q))$ as a cost term in trajectory optimization
   - **Scaling/regularization**: add $M + \lambda I$ (Tikhonov) to guarantee invertibility
5. **For floating-base robots**: it gets worse -- $M_{\text{base}}$ has large directional inertia disparities, so you must use spatial-algebra Plucker representations to keep conditioning in check.

**Conclusion**: use Cholesky or ABA, plan around ill-conditioned poses, regularize if needed. In an interview, being able to explain why ABA numerically avoids $M^{-1}$ gives you a bonus.

</details>

<details>
<summary>Q5: A humanoid does a backflip and on landing the torso pitches forward and nearly falls. How do you analyze this?</summary>

**Complete reasoning chain**:

1. **Theoretical root cause: floating-base + angular momentum conservation**
   - Flight phase has no contact forces and gravity produces no torque about the CoM -> **total angular momentum $k$ is strictly conserved**
   - If $k_{\text{takeoff}}$ is too large, landing must dissipate it quickly via ground reaction torque
   - If the landing WBC does not add $\dot{k}_{\text{des}}$ as a constraint, contact forces only regulate translational CoM -> front legs hit first, producing a large pitching moment -> forward flip into the ground
2. **Diagnostics**:
   - Check whether the takeoff-phase CMM-computed $k$ is within a controllable range (Pinocchio `ccrba`)
   - Verify that the landing WBC cost includes $\|\dot{k} - \dot{k}_{\text{des}}\|^2$
   - Verify the CWC is correctly rotated from world frame to the local contact frame (crucial on slopes / stairs)
3. **Fixes**:
   - Takeoff: MPC schedules the angular-momentum profile so takeoff $k$ is not excessive
   - Landing: add a strong angular-momentum tracking term to WBC; if needed, counter-swing arms/legs (Zero-Momentum Turn)
   - Deep fix: use CITO + DDP to solve a fully dynamically feasible backflip trajectory (takeoff-flight-landing contact sequence in one shot)
4. **Validation**: reproduce in MuJoCo and check the $k$ profile; on real hardware, add IMU and close the loop on measured $k$.

**Conclusion**: the essence of floating-base control is angular-momentum conservation + underactuated dynamics. Failing to explicitly constrain $\dot{k}$ in the WBC is equivalent to letting the airborne attitude spin freely. This is a signature humanoid interview question.

</details>

<details>
<summary>Q6: A UR5 passes morning tests perfectly but after 30 minutes of warm-up triggers safety stops with excessive contact velocities. Why?</summary>

**Complete reasoning chain**:

1. **Symptom localization**: "morning vs afternoon" difference -> time-dependent -> thermal effect
2. **Physical cause**: harmonic-drive grease has very high viscosity when cold; as temperature rises, viscous friction $F_v$ drops by 20-30%
3. **Error chain**:
   - Cold-calibrated friction parameter $F_v^{\text{cold}}$ was baked into CTC feedforward as the nominal value
   - After warm-up, real $F_v^{\text{hot}} < F_v^{\text{cold}}$
   - Feedforward `tau_ff = F_v^cold * q_dot` **over-compensates** -> actual acceleration exceeds desired
   - Contact velocity spikes beyond the safety threshold -> E-stop
4. **Corrective strategies**:
   - **Short term**: warm up for 30 minutes on site then re-run SysID (standard practice in industrial robot manuals)
   - **Medium term**: RLS + forgetting factor tracks thermal drift online and adjusts $F_v$ continuously
   - **Long term**: DOB directly estimates `disturbance_est = LPF(tau_actual - tau_ideal)` as feedforward, independent of any friction model
   - **Contact fallback**: switch to impedance control 50 mm before contact -- goal is low-stiffness safe engagement, not precise tracking
5. **Interview bonus**: this is the canonical "why industrial robots need warm-up" question; candidates with no real shop-floor experience cannot answer.

**Conclusion**: thermal effects are the friction model's Achilles heel. Static SysID is not enough -- you need online adaptation (RLS / DOB) + a mode switch to impedance control near contact.

</details>

## Interview Angles

1. **Feedforward + feedback decoupling is the core of dynamics control** -- This is the watershed between "can tune PID" and "understands control". **Bring it out**: "CTC cancels ~90% of nonlinear dynamics via inverse dynamics feedforward; PD only corrects the residual; closed-loop error becomes linear second-order. This is the industrial standard -- an order of magnitude more robust than pure PID under high speed and varying load."

2. **RNEA O(n) vs Lagrangian O(n^3), and ABA does not always beat Cholesky at low DoF** -- Shows you understand big-O plus constants plus hardware realities. **Bring it out**: "RNEA is the O(n) recursive Newton-Euler -- 7-DoF runs in 50 us on ARM. ABA is theoretically O(n) but the constant is big; at n < 9 CRBA+Cholesky with SIMD is often faster. This is practical knowledge you cannot get from textbooks alone."

3. **Passivity-Based Control and the $\dot{M} - 2C$ skew symmetry** -- Control-theoretic depth. **Bring it out**: "$\dot{M} - 2C$ skew symmetry is not a math curiosity; it enforces that Coriolis forces do no work, treating the robot as an energy converter. PBC actively shapes energy and is more robust than PD+G(q) in contact tasks -- this is why Franka and KUKA iiwa joint-torque modes are Passivity-based."

4. **Industrial 8-step SysID + LMI physical consistency** -- The first step of sim-to-real. **Bring it out**: "CAD parameters are never accurate enough. My pipeline: analytical regressor -> friction extension (Stribeck/LuGre) -> Fourier excitation trajectory minimizing cond(Y) -> 1 kHz collection with filtfilt -> WLS -> **LMI constraints for positive-definite inertia** -> new-trajectory RMSE < 5%. Skipping the LMI step has literally put robots through tables."

5. **Slotine-Li adaptive control and the PE trap** -- Adaptive-control depth. **Bring it out**: "Slotine-Li uses $s = \dot{e} + \Lambda e$ sliding surfaces to guarantee $e \to 0$, but $\hat{\pi} \to \pi$ only holds under Persistent Excitation. When Y is rank-deficient the estimator cannot separate 'payload' from 'friction' and drifts into the unobservable subspace. Engineering fixes: inject exploration sinusoids, project estimates back into the physical feasible set."

6. **Floating-base Underactuated Dynamics + angular-momentum conservation** -- The core of humanoid / quadruped control. **Bring it out**: "The base has 6 unactuated DoF -- motors can only change base motion through GRFs. This is Underactuated Dynamics. In flight phases gravity has zero moment about the CoM, so total angular momentum is strictly conserved -- this is how cats land upright. WBC must constrain $\dot{k}_{\text{des}}$ on landing or the pitching moment is uncontrolled."

7. **CWC beats ZMP and unifies multi-contact constraints in a QP** -- Foundation of modern WBC. **Bring it out**: "ZMP assumes a common horizontal plane and infinite friction, which breaks down in 3D multi-contact (hand on wall + foot on stair). CWC unifies four constraint classes -- normal pressure, friction cone, CoP, yaw torque -- linearizes into a polyhedral cone, and the QP finds optimal contact forces. Tesla Optimus and Figure run CWC-QP under the hood."

8. **Teacher-Student Privileged Learning** -- The sim-to-real recipe for embodied AI. **Bring it out**: "ANYmal / Boston Dynamics route: Teacher reads privileged information (friction, mass, terrain height) in simulation; Student uses proprioception + RNN to infer from slip history. This beats pure Domain Randomization because the Student actually learns to feel the ground rather than being boxed into conservative behavior by randomization bounds."

9. **Differentiable simulators + big-brain RL + little-brain WBC hybrid** -- The next generation of embodied AI. **Bring it out**: "MuJoCo MJX / Brax / Dojo let $\partial s/\partial a$ flow through the physics engine, replacing high-variance REINFORCE gradients with exact gradients -- 100x sample efficiency. Furthermore, instead of outputting torques directly, the RL action space outputs desired centroidal acceleration or impedance $\Delta K/D$, and WBC/CWC-QP handles low-level dynamic feasibility. Big-brain RL + little-brain WBC is the Tesla / Figure playbook."

10. **Non-collocated control and right-half-plane zeros** -- Signature interview question for large flexible arms. **Bring it out**: "A Canadarm 1.5-ton / 17-meter arm has the motor at the joint but needs precision at the distant end-effector. The flexible link between causes the transfer function to have right-half-plane zeros (non-minimum phase); ramping up PID gain diverges. The fix is **Input Shaping (ZV shaper so the second pulse is 180 degrees out of phase with the first) + singular perturbation separating slow/fast subsystems + LQR state observer**."

## Further Reading

- **Roy Featherstone, *Rigid Body Dynamics Algorithms*** -- the bible of spatial algebra, RNEA, and ABA. Required reading for understanding how O(n) forward dynamics avoids matrix inversion
- **Lynch & Park, *Modern Robotics*, Ch8 (Dynamics of Open Chains)** -- free textbook, unifies Newton-Euler and Lagrangian via screw theory
- **Siciliano et al., *Robotics: Modelling, Planning and Control*, Ch7** -- the standard graduate-level dynamics chapter; thorough CTC + impedance coverage
- **Khalil & Dombre, *Modeling, Identification and Control of Robots*** -- industrial-grade System Identification textbook: Fourier excitation + WLS + LMI
- **Pinocchio documentation** ([github.com/stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio)) -- the fastest open-source rigid-body dynamics library; study the source to learn Featherstone's algorithms in production
- **Drake tutorials** ([drake.mit.edu](https://drake.mit.edu)) -- MIT's robotics toolkit with LCP + TrajOpt + CITO examples
- **Russ Tedrake, *Underactuated Robotics* (MIT 6.832)** -- the authoritative free course on underactuated systems, trajectory optimization, and CITO
- **MuJoCo documentation: Computation / Contact** -- required reading for `solimp/solref` tuning
- **Paper: Swevers et al., *Optimal Robot Excitation and Identification*** -- the classic excitation-trajectory-design paper
- **Paper: *LuGre Friction Model* (Canudas-de-Wit)** -- the bible of dynamic friction modeling
- **Paper: *Learning Quadrupedal Locomotion over Challenging Terrain* (ETH, Lee et al. 2020)** -- the flagship Teacher-Student Privileged Learning paper
- **Paper series on differentiable simulators (MIT, Dojo)** -- industrial applications of differentiable physics
- **Public technical reports from MIT Cheetah / ANYmal / Atlas** -- see how real systems stack CWC-QP + WBC + MPC
