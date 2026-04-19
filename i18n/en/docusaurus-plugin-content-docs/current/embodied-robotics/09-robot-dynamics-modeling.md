---
title: "Dynamics Modeling (Newton-Euler and Lagrangian)"
prerequisites: ["08-inverse-kinematics"]
estimated_time: 45
difficulty: 4
tags: ["dynamics", "newton-euler", "lagrangian", "computed-torque"]
sidebar_position: 9
---

# Dynamics Modeling (Newton-Euler and Lagrangian)

## You Will Learn

- Define the difference between kinematics and dynamics precisely enough to never confuse the two in an interview: kinematics is geometry (where does the end-effector go?), dynamics is physics (how much force/torque does it take to get there?)
- Decide which formulation to reach for in practice: Newton-Euler for real-time control, Lagrangian for offline symbolic analysis
- Explain why a pure PID controller falls apart at high speed and what "computed-torque feedforward" buys you

## Core Concepts

**Precise Definition**: **Robot dynamics** is the study of how forces and torques relate to motion. Given a manipulator's joint positions $q$, velocities $\dot{q}$, and accelerations $\ddot{q}$, dynamics tells you the joint torques $\tau$ required to produce that motion (inverse dynamics), or conversely, given applied torques, predicts the resulting accelerations (forward dynamics).

**Location in the Sense --> Plan --> Control Loop**:
- **Input**: joint trajectory $(q, \dot{q}, \ddot{q})$ from the trajectory planner, or applied torques $\tau$ from the actuators
- **Output**: required torques $\tau$ (inverse dynamics) or resulting accelerations $\ddot{q}$ (forward dynamics)
- **Downstream consumers**: computed-torque controllers (feedforward term), impedance/admittance controllers (model-based compliance), physics simulators (MuJoCo, PyBullet, Isaac Sim), system identification pipelines, model predictive controllers
- **Loop node**: sits squarely in the **control** layer. Inverse dynamics provides the feedforward signal that cancels nonlinear coupling, leaving only a simple linear error for the PD feedback loop to handle.

**One-line version**: "Dynamics is the muscle-and-skeleton owner's manual for the robot -- it lets the simulator predict the future and the controller deliver precisely the right torque."

### The Standard Equation of Motion

Every rigid-body serial manipulator obeys:

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau
$$

| Symbol | Name | Physical meaning |
|--------|------|------------------|
| $M(q)$ | Inertia matrix | How "heavy" each joint feels -- a symmetric positive-definite matrix that changes with pose. High condition number means some directions are much harder to accelerate than others. |
| $C(q, \dot{q})$ | Coriolis and centrifugal matrix | Velocity-dependent coupling forces. When joint 1 spins fast, joint 2 feels a phantom push. Built from Christoffel symbols of $M$. The skew-symmetry property $\dot{M} - 2C$ is skew-symmetric is critical for Lyapunov stability proofs. |
| $g(q)$ | Gravity vector | The torque gravity exerts on each joint in the current configuration. A 6-DoF arm at full extension might need 80% of motor torque just to hold still against gravity. |
| $\tau$ | Joint torques | What the motors actually deliver (or what the controller commands). |

### Two Formulations

| Property | Newton-Euler (Recursive) | Lagrangian (Energy) |
|----------|--------------------------|---------------------|
| Core idea | Force/moment balance per link | $L = T - V$, Euler-Lagrange equations |
| Complexity | $O(n)$ | $O(n^3)$ to $O(n^4)$ |
| Best for | Real-time control (1 kHz+) | Symbolic derivation, offline analysis |
| Output | Numerical torques directly | Closed-form symbolic equations |
| Industry tool | RNEA in Pinocchio / KDL / Drake | SymPy / MATLAB Symbolic Toolbox |

### Newton-Euler: The Recursive Algorithm (RNEA)

Two passes along the kinematic chain:

1. **Forward pass** (base --> tip): propagate velocities and accelerations link by link
2. **Backward pass** (tip --> base): accumulate forces and torques from the end-effector back to the base

$$
\text{Forward: } \omega_i, \dot{\omega}_i, \ddot{p}_i = f(q_i, \dot{q}_i, \ddot{q}_i, \omega_{i-1}, \dot{\omega}_{i-1}, \ddot{p}_{i-1})
$$

**Physical meaning**: each link inherits the previous link's motion and adds its own joint contribution. The forward pass is "how fast and how hard is each link accelerating?" The backward pass is "given all that acceleration, what net wrench does each joint need to supply?"

$$
\text{Backward: } f_i, n_i = f(f_{i+1}, n_{i+1}, \text{link inertia}_i, \ddot{p}_i, \omega_i, \dot{\omega}_i)
$$

**Physical meaning**: starting from the tip (where external forces act), each joint must support all the forces from the links beyond it plus its own inertial loads.

<details>
<summary>Deep dive: full RNEA pseudocode and spatial algebra formulation</summary>

**Forward pass** (for $i = 1$ to $n$):

$$
\omega_i = R_i^T \omega_{i-1} + \dot{q}_i \hat{z}_0
$$
$$
\dot{\omega}_i = R_i^T \dot{\omega}_{i-1} + R_i^T \omega_{i-1} \times \dot{q}_i \hat{z}_0 + \ddot{q}_i \hat{z}_0
$$
$$
\ddot{p}_i = R_i^T \left( \ddot{p}_{i-1} + \dot{\omega}_{i-1} \times r_{i-1,i} + \omega_{i-1} \times (\omega_{i-1} \times r_{i-1,i}) \right)
$$
$$
\ddot{p}_{c_i} = \ddot{p}_i + \dot{\omega}_i \times s_i + \omega_i \times (\omega_i \times s_i)
$$

where $s_i$ is the vector from the link frame origin to the center of mass, and $r_{i-1,i}$ is the vector between adjacent frame origins.

**Backward pass** (for $i = n$ down to $1$):

$$
F_i = m_i \ddot{p}_{c_i}
$$
$$
N_i = I_i \dot{\omega}_i + \omega_i \times I_i \omega_i
$$
$$
f_i = R_{i+1} f_{i+1} + F_i
$$
$$
n_i = N_i + R_{i+1} n_{i+1} + s_i \times F_i + r_{i,i+1} \times R_{i+1} f_{i+1}
$$
$$
\tau_i = n_i^T \hat{z}_0
$$

**Featherstone's Spatial Algebra** reformulates this using 6D spatial vectors (twist and wrench) and $6 \times 6$ spatial inertia matrices, making the recursion cleaner and enabling $O(n)$ forward dynamics via the Articulated Body Algorithm (ABA). Libraries like Pinocchio and Drake implement spatial algebra internally.

**Complexity**: each pass visits $n$ links once with constant-time operations per link, giving $O(n)$ total -- the key advantage for real-time control with many joints.

</details>

### Lagrangian: The Energy Method

Define the Lagrangian $L = T - V$ where $T$ is total kinetic energy and $V$ is total potential energy:

$$
T = \frac{1}{2} \dot{q}^T M(q) \dot{q}
$$

**Physical meaning**: the total kinetic energy is a quadratic form in joint velocities, weighted by the configuration-dependent inertia matrix.

$$
V = \sum_{i=1}^n m_i g^T p_{c_i}(q)
$$

**Physical meaning**: the total potential energy is the sum of each link's mass times gravitational acceleration times its center-of-mass height.

The Euler-Lagrange equation:

$$
\frac{d}{dt}\frac{\partial L}{\partial \dot{q}_i} - \frac{\partial L}{\partial q_i} = \tau_i
$$

**Physical meaning**: this is conservation of energy dressed up as a differential equation. Cranking through the partial derivatives for all joints produces the standard $M\ddot{q} + C\dot{q} + g = \tau$ form.

<details>
<summary>Deep dive: deriving $M$, $C$, $g$ from the Lagrangian for a 2-link planar arm</summary>

Consider a 2-link planar manipulator with link masses $m_1, m_2$, link lengths $l_1, l_2$, and center-of-mass distances $l_{c1}, l_{c2}$.

**Positions of centers of mass**:

$$
x_{c1} = l_{c1} \cos q_1, \quad y_{c1} = l_{c1} \sin q_1
$$
$$
x_{c2} = l_1 \cos q_1 + l_{c2} \cos(q_1 + q_2), \quad y_{c2} = l_1 \sin q_1 + l_{c2} \sin(q_1 + q_2)
$$

**Kinetic energy** (including rotational terms with inertias $I_1, I_2$):

$$
T = \frac{1}{2}(m_1 l_{c1}^2 + m_2 l_1^2 + m_2 l_{c2}^2 + 2m_2 l_1 l_{c2} \cos q_2 + I_1 + I_2)\dot{q}_1^2
$$
$$
+ \frac{1}{2}(m_2 l_{c2}^2 + I_2)\dot{q}_2^2 + (m_2 l_{c2}^2 + m_2 l_1 l_{c2} \cos q_2 + I_2)\dot{q}_1 \dot{q}_2
$$

**Inertia matrix** (read off the quadratic form):

$$
M(q) = \begin{bmatrix}
m_1 l_{c1}^2 + m_2(l_1^2 + l_{c2}^2 + 2l_1 l_{c2} \cos q_2) + I_1 + I_2 & m_2(l_{c2}^2 + l_1 l_{c2} \cos q_2) + I_2 \\
m_2(l_{c2}^2 + l_1 l_{c2} \cos q_2) + I_2 & m_2 l_{c2}^2 + I_2
\end{bmatrix}
$$

Notice that $M$ depends on $q_2$ (the elbow angle) but not $q_1$ -- the inertia the shoulder motor feels changes as the elbow bends.

**Coriolis/centrifugal matrix** (via Christoffel symbols $c_{ijk} = \frac{1}{2}\left(\frac{\partial M_{kj}}{\partial q_i} + \frac{\partial M_{ki}}{\partial q_j} - \frac{\partial M_{ij}}{\partial q_k}\right)$):

$$
C(q, \dot{q}) = \begin{bmatrix}
-m_2 l_1 l_{c2} \sin q_2 \cdot \dot{q}_2 & -m_2 l_1 l_{c2} \sin q_2 \cdot (\dot{q}_1 + \dot{q}_2) \\
m_2 l_1 l_{c2} \sin q_2 \cdot \dot{q}_1 & 0
\end{bmatrix}
$$

**Gravity vector**:

$$
g(q) = \begin{bmatrix}
(m_1 l_{c1} + m_2 l_1) g \cos q_1 + m_2 l_{c2} g \cos(q_1 + q_2) \\
m_2 l_{c2} g \cos(q_1 + q_2)
\end{bmatrix}
$$

This 2-link example is the simplest non-trivial case. For 6+ DoF arms, the symbolic expressions explode in size, which is why numerical RNEA is preferred for real-time use.

</details>

### Forward vs Inverse Dynamics

| Direction | Input | Output | Primary use |
|-----------|-------|--------|-------------|
| **Inverse dynamics** | $(q, \dot{q}, \ddot{q})$ | $\tau$ | Computed-torque feedforward, impedance control |
| **Forward dynamics** | $(q, \dot{q}, \tau)$ | $\ddot{q}$ | Physics simulation (MuJoCo, Gazebo, Isaac Sim) |

**Physical meaning of inverse dynamics**: "given the trajectory I want, tell me exactly which torques to apply." This is the engine behind computed-torque control -- it cancels 90% of nonlinear dynamics (inertia coupling, Coriolis, gravity), leaving only a simple linear system for PD feedback.

**Physical meaning of forward dynamics**: "given the torques the motors are applying right now, tell me how the robot will accelerate." This is what every physics simulator computes at each timestep to integrate the equations of motion.

**Common APIs** (industry toolchain):

| Layer | Package | Inverse dynamics | Forward dynamics |
|-------|---------|------------------|------------------|
| C++ high-perf | Pinocchio | `rnea(model, data, q, v, a)` | `aba(model, data, q, v, tau)` |
| C++ planning | Drake | `CalcInverseDynamics()` | `CalcTimeDerivatives()` |
| Python sim | MuJoCo | `mj_inverse(model, data)` | `mj_step(model, data)` |
| Python sim | PyBullet | `calculateInverseDynamics()` | `stepSimulation()` |
| ROS 2 | KDL | `ChainIdSolver_RNE` | `ChainFdSolver_RNE` |

## Intuition

**Analogy: driving a car on a mountain road**.

| Dynamics term | Car analogy |
|---------------|-------------|
| $M(q)$ -- inertia matrix | The car's mass. Heavier car = more gas to accelerate. But here the "weight" changes depending on which gear (configuration) you are in. |
| $C(q, \dot{q})$ -- Coriolis/centrifugal | Taking a sharp turn at speed. You feel a sideways push that has nothing to do with the engine -- it comes purely from the combination of speed and curvature. |
| $g(q)$ -- gravity | Driving uphill. You need to press the gas just to hold your speed, even though you are not accelerating. The steeper the hill (the more extended the arm), the more throttle you burn. |
| $\tau$ -- torques | The engine output. A naive driver (PID only) presses the gas based on how far behind they are. A skilled driver (computed-torque) pre-compensates: "this hill needs 3000 RPM just to maintain speed, so I start there and only add a little more to catch up." |

**Simulator observation**: in MuJoCo, load a Franka Panda arm. Disable gravity compensation (`mjcb_control = None`) and watch the arm collapse under its own weight -- that is $g(q)$. Now enable gravity compensation only and command a fast joint swing: the arm still wobbles because Coriolis coupling $C$ is uncompensated. Finally, enable full inverse dynamics feedforward and see the trajectory tracking become crisp.

**Visual metaphor**: imagine holding a broom upside down on your palm. The "inverse dynamics" problem is: given the trajectory your hand must follow to keep the broom balanced, how hard and in which direction must your muscles push? The "forward dynamics" problem is: if you suddenly jerk your hand left, which way will the broom tip?

## Implementation Link

**Three representative engineering scenarios**:

1. **Computed-Torque Control (CTC)**: the canonical application of inverse dynamics. The controller computes:

$$
\tau = M(q)(\ddot{q}_d + K_d \dot{e} + K_p e) + C(q, \dot{q})\dot{q} + g(q)
$$

where $e = q_d - q$ is the tracking error. The $M \ddot{q}_d + C\dot{q} + g$ part cancels the robot's nonlinear dynamics (feedforward), and $M(K_d \dot{e} + K_p e)$ shapes the error dynamics into a simple second-order linear system. Result: the nonlinear 6-DoF tracking problem reduces to six independent linear PD controllers.

2. **Physics simulation inner loop**: every simulator timestep calls forward dynamics. MuJoCo uses a semi-implicit Euler integrator:
   - Read current state $(q, \dot{q})$ and applied torques $\tau$
   - Compute $\ddot{q} = M^{-1}(\tau - C\dot{q} - g)$ via ABA or Cholesky
   - Integrate: $\dot{q} \leftarrow \dot{q} + \ddot{q} \cdot dt$, $q \leftarrow q + \dot{q} \cdot dt$

3. **Impedance control with dynamics model**: to make the end-effector behave like a mass-spring-damper system (essential for safe human-robot interaction), you need the dynamics model to decouple the natural dynamics from the desired compliance behavior:

$$
\tau = M(q) \ddot{q}_d + C(q, \dot{q})\dot{q} + g(q) + J^T(q)(M_d \ddot{x}_e + B_d \dot{x}_e + K_d x_e)
$$

**Code skeleton** (Python, Pinocchio):

```python
import pinocchio as pin
import numpy as np

# Load robot model from URDF
model = pin.buildModelFromUrdf("panda.urdf")
data = model.createData()

def computed_torque_control(q, dq, q_des, dq_des, ddq_des, Kp, Kd):
    """Computed-torque controller: feedforward + PD feedback."""
    # Inverse dynamics: compute M*ddq_des + C*dq + g
    tau_ff = pin.rnea(model, data, q, dq, ddq_des)

    # PD feedback on tracking error
    e = q_des - q
    de = dq_des - dq

    # Compute M for the feedback term: tau_fb = M * (Kp*e + Kd*de)
    pin.crba(model, data, q)  # fills data.M
    tau_fb = data.M @ (Kp * e + Kd * de)

    return tau_ff + tau_fb
```

<details>
<summary>Deep dive: complete C++ real-time computed-torque controller with Pinocchio</summary>

```cpp
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/parsers/urdf.hpp>

class ComputedTorqueController {
public:
    pinocchio::Model model_;
    pinocchio::Data data_;
    Eigen::VectorXd Kp_, Kd_;

    ComputedTorqueController(const std::string& urdf_path,
                              const Eigen::VectorXd& Kp,
                              const Eigen::VectorXd& Kd)
        : Kp_(Kp), Kd_(Kd) {
        pinocchio::urdf::buildModel(urdf_path, model_);
        data_ = pinocchio::Data(model_);
    }

    // Call this at 1 kHz from the real-time control loop.
    // All memory is pre-allocated -- zero heap allocation.
    Eigen::VectorXd compute(const Eigen::VectorXd& q,
                            const Eigen::VectorXd& dq,
                            const Eigen::VectorXd& q_des,
                            const Eigen::VectorXd& dq_des,
                            const Eigen::VectorXd& ddq_des) {
        // Feedforward: RNEA computes M*ddq_des + C*dq + g in O(n)
        Eigen::VectorXd tau_ff = pinocchio::rnea(model_, data_, q, dq, ddq_des);

        // Feedback: M * (Kp * e + Kd * de)
        pinocchio::crba(model_, data_, q);  // fills data_.M (upper triangle)
        data_.M.triangularView<Eigen::StrictlyLower>() =
            data_.M.transpose().triangularView<Eigen::StrictlyLower>();

        Eigen::VectorXd e = q_des - q;
        Eigen::VectorXd de = dq_des - dq;
        Eigen::VectorXd tau_fb = data_.M * (Kp_.asDiagonal() * e + Kd_.asDiagonal() * de);

        return tau_ff + tau_fb;
    }
};
```

**Key real-time constraints**:
- `pinocchio::rnea` is $O(n)$ and allocation-free -- safe for 1 kHz hard real-time
- `pinocchio::crba` is $O(n^2)$ for the full inertia matrix -- still fast for $n \leq 7$
- All `Eigen::VectorXd` members are pre-sized in the constructor; the `compute` method never calls `new` or `malloc`
- For even tighter timing, use `pinocchio::computeAllTerms` which batches RNEA + CRBA + Jacobian in one traversal

</details>

## Common Misconceptions

1. **"The inertia matrix $M(q)$ is constant"** -- Wrong. $M$ changes with every joint configuration. When a 6-DoF arm is fully extended, the shoulder motor "feels" the entire mass of the arm at maximum lever. When the arm is folded, the effective inertia drops dramatically. Controllers that treat $M$ as constant degrade at poses far from the identification configuration. **Avoid**: always recompute $M(q)$ at the current $q$ every control tick, or use RNEA which implicitly handles configuration-dependent inertia.

2. **"PID is enough for trajectory tracking"** -- Only at low speeds and low payloads. At high speed, Coriolis and centrifugal terms $C(q, \dot{q})\dot{q}$ become significant coupling disturbances that a PID loop must fight reactively. Adding feedforward from inverse dynamics cancels these terms proactively, reducing tracking error by an order of magnitude. **Rule of thumb**: if the robot moves faster than ~30% of its maximum joint velocity, you need model-based feedforward.

3. **"Lagrangian is too slow, always use Newton-Euler"** -- The two are mathematically equivalent but computationally different. Lagrangian is $O(n^3)$+ for numerical evaluation, which is indeed too slow for 1 kHz control of a 7-DoF arm. But Lagrangian shines for **symbolic** analysis: deriving stability proofs, controller design, parameter identification equations. Use Lagrangian offline to derive the structure, then deploy RNEA online for speed. They are complementary, not competing.

4. **"Friction is just $\mu \cdot N$"** -- Coulomb friction is only one piece. Real joints exhibit Stribeck friction (velocity-dependent static-to-dynamic transition), viscous damping, and hysteresis from harmonic drives. A dynamics model that only includes Coulomb friction will have systematic torque prediction errors at low velocities. **Fix**: use a Stribeck + viscous model $\tau_f = f_c \cdot \text{sign}(\dot{q}) + f_v \dot{q} + f_s e^{-|\dot{q}|/v_s} \text{sign}(\dot{q})$, or employ a Disturbance Observer (DOB) to lump all unmodeled friction into a real-time disturbance estimate.

## Situational Questions

<details>
<summary>Q1 (medium): Your computed-torque controller tracks well in simulation but poorly on the real robot. The tracking error is especially bad during fast motions and when carrying a payload. How do you systematically diagnose and fix this?</summary>

**Complete reasoning chain**:

1. **Identify the symptom pattern**: errors worsen with speed (Coriolis terms) and payload (inertia mismatch). This points to **inaccurate dynamics parameters** -- the $M$, $C$, $g$ your controller uses do not match the real robot.

2. **Root cause**: the URDF/CAD inertial parameters (mass, center of mass, inertia tensor) are nominal values. Real robots have cable routing, grease, manufacturing tolerances, and payload geometry that the CAD model does not capture.

3. **Fix -- offline System Identification**:
   - Design an **excitation trajectory**: a trajectory that exercises all joints through a wide range of configurations and velocities. Use Fourier-based optimal excitation (e.g., 5th-order Fourier series) to maximize parameter identifiability.
   - Record $(q, \dot{q}, \ddot{q}, \tau_{\text{measured}})$ at high rate (1 kHz+).
   - The dynamics equation $\tau = Y(q, \dot{q}, \ddot{q}) \cdot \pi$ is **linear in the parameters** $\pi$ (masses, center-of-mass coordinates, inertia entries, friction coefficients). Build the regressor matrix $Y$ and solve via Weighted Least Squares (WLS) for the identified parameter vector $\hat{\pi}$.

4. **Fix -- online robustness**: even after System ID, unmodeled dynamics remain. Add a **Disturbance Observer (DOB)** that estimates the residual $d = \tau_{\text{actual}} - \tau_{\text{model}}$ in real-time and compensates it. Alternatively, use adaptive control to update $\hat{\pi}$ online.

5. **Validation**: replay the excitation trajectory with the updated model. Tracking error should drop by 5-10x. If not, suspect actuator dynamics (motor inertia, gear backlash, current-loop bandwidth) as the next bottleneck.

**What the interviewer wants to hear**: the dynamics equation is linear in its inertial parameters, enabling clean least-squares identification; DOB handles the remaining unmodeled disturbances; and you have a systematic pipeline (excitation design --> data collection --> WLS --> validation --> online compensation).

</details>

<details>
<summary>Q2 (medium-hard): Your RL policy trained in Isaac Sim produces smooth trajectories, but on the real Franka Panda the joint torques are clipped and the arm oscillates. The sim uses a rigid-body dynamics model. What is happening and how do you fix it?</summary>

**Complete reasoning chain**:

1. **Diagnose the sim-to-real gap**: rigid-body simulators model ideal joints. Real harmonic drives have:
   - **Nonlinear friction**: Stribeck effect at low velocities causes stick-slip that the sim never sees
   - **Joint flexibility**: harmonic drives are not perfectly rigid; the spring-like compliance introduces resonance modes
   - **Actuator dynamics**: motor current loops have finite bandwidth (~1 kHz); the sim assumes instantaneous torque application
   - **Torque limits**: the sim may allow torques beyond the real motor's saturation

2. **Why oscillation occurs**: the RL policy learned high-gain corrective torques that work in a frictionless, perfectly rigid sim. On hardware, these high-frequency torque commands excite the harmonic drive's resonance and the current loop's bandwidth limit, causing oscillation.

3. **Fix 1 -- Domain Randomization (DR)**: during training, randomize:
   - Link masses and inertias by +/-20%
   - Joint friction coefficients (add Stribeck + viscous)
   - Torque limits (clip to 80-100% of nominal)
   - Actuator delay (add 1-3 ms random latency)
   - Joint damping coefficients

4. **Fix 2 -- System Identification**: run frequency-sweep signals on the real robot to identify the harmonic drive stiffness, damping, and friction parameters. Feed these into the simulator to close the reality gap from the model side.

5. **Fix 3 -- Action smoothing**: add a low-pass filter or rate limiter on the policy's torque output to prevent exciting hardware resonances. Penalize jerk $\dddot{q}$ in the RL reward to encourage smoother policies during training.

6. **Validation**: compare torque histograms and trajectory smoothness metrics between sim and real after each fix. The oscillation should disappear once the policy no longer relies on instantaneous high-bandwidth torque corrections.

**What the interviewer wants to hear**: the reality gap for dynamics is fundamentally about unmodeled friction, joint flexibility, and actuator bandwidth; Domain Randomization makes the policy robust to these unknowns; System ID closes the gap from the model side; and action smoothing prevents exciting hardware resonances.

</details>

<details>
<summary>Q3 (hard): You need to run inverse dynamics at 1 kHz for a 7-DoF arm inside an impedance controller on an embedded real-time Linux system. The CPU budget is 200 microseconds per tick. How do you architect this?</summary>

**Complete reasoning chain**:

1. **Algorithm choice**: RNEA is $O(n)$ with $n = 7$, requiring roughly 7 forward-pass iterations and 7 backward-pass iterations. Each iteration involves $3 \times 3$ matrix-vector products and cross products. For $n = 7$, this is approximately 500-800 floating-point operations -- well within 200 us even on modest ARM hardware.

2. **Library choice**: Pinocchio's RNEA is heavily optimized with Eigen vectorization and cache-friendly memory layout. Benchmark: on a modern x86 CPU, Pinocchio's `rnea` for a 7-DoF arm takes ~2-5 us. On ARM (e.g., Raspberry Pi-class), expect ~20-50 us. Either way, comfortably within budget.

3. **Real-time guarantees**:
   - Use **PREEMPT_RT** patched Linux kernel for deterministic scheduling
   - Pin the control thread to an isolated CPU core (`isolcpus` + `taskset`)
   - Lock all memory pages (`mlockall`) to prevent page faults
   - Pre-allocate all Pinocchio `Data` structures at startup -- the `rnea` call itself is allocation-free
   - Disable CPU frequency scaling (set `performance` governor)

4. **If even tighter timing is needed**: use **CasADi code generation**. Offline, formulate the RNEA symbolically with CasADi + Pinocchio's CasADi interface, then generate pure C code with no library dependencies. The generated code is a flat sequence of arithmetic operations -- no function calls, no branching, no cache misses. This can shave another 2-5x off execution time.

5. **Impedance controller integration**: the full impedance loop computes:
   - Jacobian $J(q)$: Pinocchio's `computeJointJacobians` is $O(n)$
   - RNEA for feedforward: $O(n)$
   - Cartesian impedance law: matrix-vector multiply, $O(1)$ for 6D
   - Total: still well under 200 us for $n = 7$

6. **Testing**: run the controller in a tight loop for 10 million iterations, log execution times, and verify the 99.99th percentile stays under 200 us. Any spike above budget means you have a latency source (page fault, interrupt, thermal throttling) to hunt down.

**What the interviewer wants to hear**: RNEA is already $O(n)$ and fast enough for most embedded platforms; real-time Linux setup (PREEMPT_RT, core isolation, memory locking) is non-negotiable; Pinocchio + CasADi code generation is the nuclear option for extreme timing constraints; and you measure worst-case latency, not average.

</details>

<details>
<summary>Q4 (hard): A colleague claims that for their legged robot with 12 actuated joints, computing the full inertia matrix $M(q)$ at every control tick is unnecessary. Are they right? When do you need $M$ explicitly vs when can you avoid it?</summary>

**Complete reasoning chain**:

1. **When you do NOT need the full $M(q)$**:
   - **Pure feedforward (inverse dynamics)**: RNEA computes $\tau = M\ddot{q} + C\dot{q} + g$ in $O(n)$ **without ever forming $M$ explicitly**. It directly gives you the torque vector. If all you need is computed-torque feedforward, skip $M$ entirely.
   - **Forward dynamics via ABA**: Featherstone's Articulated Body Algorithm computes $\ddot{q} = f(\tau, q, \dot{q})$ in $O(n)$ without forming $M$ or inverting it. MuJoCo and Drake use this internally.

2. **When you DO need the full $M(q)$**:
   - **Operational space control**: the operational space inertia $\Lambda = (J M^{-1} J^T)^{-1}$ requires $M^{-1}$. Computing $M$ via CRBA is $O(n^2)$ and its Cholesky factorization for the inverse is $O(n^3)$. For $n = 12$, this is still fast (~50 us with Pinocchio).
   - **Impedance control feedback term**: the $M(K_p e + K_d \dot{e})$ term in computed-torque needs $M$ explicitly to shape the closed-loop mass.
   - **Stability analysis**: Lyapunov-based controllers need $M$ and the skew-symmetry property of $\dot{M} - 2C$.
   - **System identification**: the regressor matrix $Y$ is constructed from entries of $M$, $C$, $g$.

3. **Verdict for 12-DoF legged robot**: your colleague is likely right for the control loop. Legged robot controllers (e.g., MIT Cheetah, ANYmal) typically use:
   - Whole-body inverse dynamics (RNEA-based, $O(n)$) for feedforward
   - Operational-space QP for contact force optimization (needs $M$ but only at ~200 Hz, not 1 kHz)
   - Joint-level PD for the fast inner loop (no $M$ needed)

   So the 1 kHz inner loop avoids computing $M$, while the slower outer loop (task-space planner at 200 Hz) computes it when needed.

**What the interviewer wants to hear**: RNEA and ABA bypass $M$ entirely for the most common real-time operations; explicit $M$ is only needed for specific controller architectures (operational space, shaped impedance, stability proofs); and a well-designed hierarchical controller uses different levels of model fidelity at different rates.

</details>

## Interview Angles

1. **Feedforward + feedback decoupling** -- This is the single most important insight in robot control. **Bring out with**: "Computed-torque control splits the problem in two: inverse dynamics cancels 90% of the nonlinear dynamics as feedforward, and a simple PD loop handles the remaining 10% as feedback. Without the feedforward, PID must fight gravity, Coriolis, and inertia coupling reactively -- which fails at high speed. This decoupling is why model-based control dominates industrial robotics."

2. **RNEA's $O(n)$ complexity enables hard real-time** -- Separates you from candidates who only know the Lagrangian textbook derivation. **Bring out with**: "The recursive Newton-Euler algorithm is $O(n)$ because it exploits the serial chain topology -- two passes, constant work per link. Lagrangian gives you $O(n^3)$ or worse, which is fine for symbolic analysis but unusable at 1 kHz. Every industrial robot controller I know of uses RNEA or its spatial-algebra variant for the real-time loop."

3. **Disturbance Observer for unmodeled dynamics** -- Shows you understand that models are always wrong. **Bring out with**: "No dynamics model is perfect -- friction, cable forces, payload uncertainty, thermal drift. A DOB estimates the lumped disturbance $d = \tau_{\text{measured}} - \tau_{\text{model}}$ in real-time and feeds it back as compensation. This is cheaper than perfect System ID and more robust than adaptive control for slowly varying disturbances."

4. **$\dot{M} - 2C$ skew-symmetry and passivity** -- A theory point that proves you can reason about stability, not just code controllers. **Bring out with**: "The skew-symmetry of $\dot{M} - 2C$ is not a mathematical curiosity -- it encodes the fact that a rigid-body manipulator is a passive system (it cannot generate energy). This property is the foundation of every Lyapunov stability proof for robot controllers, from computed-torque to impedance control to adaptive control."

5. **Sim-to-real dynamics gap** -- Directly relevant to embodied AI. **Bring out with**: "The biggest source of sim-to-real failure is dynamics mismatch, not kinematic error. Simulators model rigid bodies perfectly but miss joint friction, actuator bandwidth, cable routing, and thermal effects. Domain Randomization makes RL policies robust to these unknowns; System ID closes the gap from the model side. You almost always need both."

## Further Reading

- **Featherstone, *Robot Dynamics Algorithms*** -- the authoritative treatment of spatial algebra, RNEA, and the Articulated Body Algorithm. Read chapters 2-4 if you want to understand what Pinocchio does internally.
- **Lynch & Park, *Modern Robotics*, Ch8 (Dynamics of Open Chains)** -- clean derivation of both Newton-Euler and Lagrangian formulations with a focus on geometric intuition. Free textbook with video lectures.
- **Siciliano et al., *Robotics: Modelling, Planning and Control*, Ch7** -- the standard graduate-level dynamics chapter. Thorough coverage of computed-torque control and impedance control.
- **Pinocchio documentation and examples** ([github.com/stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio)) -- the fastest open-source rigid-body dynamics library. Study the C++ examples for RNEA, ABA, CRBA, and CasADi code generation to see production-grade dynamics code.
- **Drake tutorials** ([drake.mit.edu](https://drake.mit.edu)) -- MIT's robotics toolkit with excellent tutorials on dynamics, simulation, and control. The "Underactuated Robotics" course by Russ Tedrake builds heavily on dynamics concepts.
- **Paper: *Revisiting the LuGre and Stribeck Friction Models* (Johanastrom & Canudas-de-Wit)** -- if you want to understand why simple Coulomb friction fails and what modern friction models look like.
- **Paper: *Disturbance Observer Based Control* (Chen, 2004)** -- the foundational DOB paper for robotic applications. Essential reading if you work on systems with significant unmodeled dynamics.
