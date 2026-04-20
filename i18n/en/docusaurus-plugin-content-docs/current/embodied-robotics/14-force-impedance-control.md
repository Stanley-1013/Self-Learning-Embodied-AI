---
title: "Force & Impedance/Admittance Control"
prerequisites: ["13-pid-control-tuning"]
estimated_time: 60
difficulty: 4
tags: ["force-control", "impedance", "admittance", "hybrid", "compliance", "operational-space", "variable-impedance", "tactile-sensing", "contact-rich", "hri"]
sidebar_position: 14
---

# Force & Impedance/Admittance Control

## You Will Learn

- State in two sentences the causal difference between impedance and admittance, and within 10 seconds work back from the single hardware question "can your actuator directly control torque?" to which architecture to use
- When you see field disasters like "UR5 wipes glass, slides off the edge, smashes into the floor", "Franka grinds and chatters", "long stick pokes a wall and oscillates", "peg-in-hole jams", or "lead-through teaching sags under gravity", you can immediately name the disease (wrong dimension of the selection matrix, diagonal K is not SPD, gravity-compensation drift, admittance limit cycle against a rigid environment) and pick the right fix
- In an interview covering the selection-matrix orthogonality theorem, the singularity trap in Cartesian vs joint impedance, the adjoint transform in operational-space control, SEA/VSA/QDD actuator selection, variable-impedance RL, the layered ACT + diffusion-policy stack, momentum-observer collision detection, and the HRI + BCI intersection, you can deliver the key reasoning for each in under two minutes
- Master the real industrial force-control pipeline: hardware selection --> F/T dynamic compensation --> contact state machine --> task-frame formalism --> layered architecture (slow VLM "brain" + fast impedance "cerebellum") --> ISO/TS 15066 functional-safety certification

## Core Concepts

**Precise definition**: **force control** is the control strategy that actively regulates the contact force between the robot and its environment, complementing position control -- position control tracks a geometric trajectory (rigid tasks), force control regulates contact force (compliant tasks). Assembly, grinding, kinesthetic teaching, human-robot collaboration, embodied manipulation -- any scenario where "you cannot just slam into something" -- is at its core a force-control problem.

**Impedance control**: make the end-effector behave like a virtual **mass-damper-spring** system. The causal direction is **position deviation --> output force** (sense position, command force). Fits hardware that can **directly output torque** (torque-controlled joints, QDD, SEA).

**Admittance control**: the causal direction is reversed -- **external force input --> position correction output** (sense force, command position). Fits **high-reduction, high-inertia rigid industrial arms**, because that hardware has excellent position-tracking accuracy but struggles with direct force control.

**Hybrid position/force control (Raibert & Craig, 1981)**: use a selection matrix $S$ to decompose task space by degree of freedom -- force-control along the normal, position-control along the tangent -- so one control loop can run mutually exclusive strategies on different axes.

**Location in the sense --> plan --> control loop**:
- **Input**: end-effector wrench (from an F/T sensor, joint-torque sensors, or current-based estimation + momentum observer), end-effector pose (from FK + encoders), desired force $F_d$ and desired pose $x_d$ (from planner or high-level VLM commands), tactile-array signals (from GelSight / DIGIT / Xela)
- **Output**: joint-torque command $\tau$ (impedance) or joint-position correction $\Delta q$ (admittance), fed to the low-level joint controller (Ch13 PID)
- **Upstream**: Ch09 dynamics model provides gravity/inertia compensation; trajectory planner or VLM policy provides $x_d$
- **Downstream**: relies on Ch13 PID for low-level joint tracking; the resulting compliant behavior directly drives assembly success rate, grinding quality, and ISO/TS 15066 human-safety certification
- **Loop node**: the **outermost loop of control**, replacing or stacked on top of position control. Inside the layered architecture of embodied AI, force control is the **spinal-reflex layer (1 kHz hard real-time)**; above it is the cerebellum (MPC / ACT / diffusion policy at ~100 Hz) and the brain (VLM / π0 at 1-10 Hz)

**One-line version**: "Force control is the robot's tactile reflex hub -- when it hits something it yields softly like a human hand instead of punching through like a drill, and it is also the physical guarantee that lets low-level execution safely realize a high-level instruction like 'insert into that hole' handed down by a big AI model."

### Minimum Sufficient Math

**1. Impedance control equation** (end-effector behaves as a virtual mass-damper-spring):

$$
F = M_d \ddot{e} + D_d \dot{e} + K_d e, \quad e = x - x_d
$$

**Physical meaning**: $M_d$ sets how quickly it accelerates under force, $D_d$ sets energy dissipation, $K_d$ sets the restoring force when displaced from the desired pose. The three are tuned independently so the end-effector behaves like a spring system you designed; $K_d \to 0$ gives "zero-force lead-through teaching", $K_d \to \infty$ approaches pure position control.

**2. Admittance control equation** (causality reversed: force --> position correction):

$$
M_d \ddot{x}_{\text{cmd}} + D_d \dot{x}_{\text{cmd}} + K_d x_{\text{cmd}} = F_{\text{ext}}
$$

**Physical meaning**: once the external force $F_{\text{ext}}$ is measured, solve this ODE for the position correction $x_{\text{cmd}}$, then forward it to the low-level position controller. Larger external force --> larger correction; $K_d = 0$ reduces to a pure-damping mode (lead-through teaching).

**3. Hybrid position/force -- selection matrix**:

$$
\tau = J^T(q)\left[S \cdot F_{\text{force}} + (I - S) \cdot F_{\text{pos}}\right]
$$

**Physical meaning**: $S$ is a 6x6 diagonal matrix with 0/1 diagonal entries. $S$ selects the force-controlled DOFs, $(I - S)$ selects the position-controlled DOFs; $S$ and $(I - S)$ are **mathematically strictly orthogonal** --> a clean partition of the 6-D task space, ruling out the over-determined conflict of "demand position = 10 and force = 5 N on the same axis".

**4. Raibert's original architecture (1981)**:

$$
\tau = J^T(q)\cdot\left[S\cdot(F_d - F_{\text{ext}}) + (I-S)\cdot(K_p\cdot e_x + K_d\cdot \dot{e}_x)\right]
$$

**Physical meaning**: force-controlled DOFs track the desired contact force $F_d$; position-controlled DOFs use standard PD tracking on trajectory error. $J^T$ maps both task-space wrenches back to joint torques.

**5. Joint torque <--> end-effector force mapping**:

$$
\tau = J^T(q)\, F
$$

**Physical meaning**: the Jacobian transpose maps a 6-D end-effector wrench back to the torque each joint must exert, the core bridge that turns task-space force into joint-space torque. When the Jacobian loses rank (a singularity), the mapping amplifies error -- high K near a singularity = exploding joint torques.

**6. Operational-space formulation (Khatib, 1987)**:

$$
\Lambda(q)\ddot{x} + \mu(q, \dot{q}) + p(q) = F_{\text{task}}, \quad \Lambda = (J M^{-1} J^T)^{-1}
$$

**Physical meaning**: $\Lambda$ is the **operational-space inertia matrix**, representing the "effective mass" of the end-effector as it appears to 3-D space in the current pose. On the same arm, the same end-effector "feels" very different to a hand push depending on joint configuration -- $\Lambda$ captures exactly that. $\mu$ and $p$ are the task-space Coriolis and gravity terms.

<details>
<summary>Deep dive: full dynamics derivation of impedance control + passivity-based stability analysis</summary>

**Start from task-space dynamics**:

The task-space dynamic equation of the manipulator (ignoring friction):

$$
\Lambda(x)\ddot{x} + \mu(x, \dot{x})\dot{x} + p(x) = F + F_{\text{ext}}
$$

**Derivation of the impedance control law**:

Target: on contact, make the end-effector behave as $M_d \ddot{e} + D_d \dot{e} + K_d e = -F_{\text{ext}}$.

Design the control force:

$$
F = \Lambda(x)\ddot{x}_d + \mu(x, \dot{x})\dot{x} + p(x) - \Lambda(x) M_d^{-1}(D_d\dot{e} + K_d e)
$$

The first three terms are **dynamics feedforward** (compensating inertia, Coriolis, gravity); the last term injects the desired impedance behavior.

Mapped back to joint space:

$$
\tau = J^T F + \left(I - J^T J^{+T}\right)\tau_0
$$

The term $(I - J^T J^{+T})\tau_0$ is the **null-space term** of a redundant manipulator -- it realises the layered control goal of "end-effector task precise + elbow compliant".

**Passivity-based stability analysis**:

Storage function (Lyapunov candidate):

$$
V = \frac{1}{2}\dot{e}^T M_d \dot{e} + \frac{1}{2}e^T K_d e
$$

Time derivative: $\dot{V} = -\dot{e}^T D_d \dot{e} \leq 0$ (assuming $D_d$ is positive definite)

Hence $D_d > 0$ is a **necessary condition** for stability; $K_d > 0$ ensures the equilibrium is unique; $K_d$ must be a **symmetric positive-definite (SPD) matrix** to preserve passivity -- otherwise "non-conservative forces" created by rotational error violate energy conservation, and the system becomes unstable and chatters violently.

**Practical takeaways**:
- $M_d$ too small (response too fast) --> control force exceeds actuator limit --> saturation --> instability
- $K_d / D_d$ ratio too large (under-damped) --> oscillation during contact
- Perfect dynamics feedforward requires an accurate dynamic model; model error must be absorbed by a robust term or an adaptive law

</details>

<details>
<summary>Deep dive: Cartesian vs joint impedance -- singularity torque explosion and 7-DoF null-space compensation</summary>

### Cartesian impedance -- intuitive but dangerous

Control law: $\tau = J^T(q)\cdot F_{\text{cartesian}}$

**Upside**: task-space intuitive (Z soft, XY stiff), aligned perfectly with how engineers think.

**The singularity trap (an interview must-know)**:
- At a singularity, $\det J(q) \to 0$ --> $J$ loses rank, condition number explodes
- $F_{\text{cartesian}}$ multiplied by an ill-conditioned $J^T$ --> **joint torques explode instantly**
- Hardware overload protection trips, gearboxes are damaged, ISO 13849 e-stop is raised

### Joint impedance -- stable but loses task intuition

Control law: $\tau = K_q(q_d - q) - D_q\dot{q}$

**Upside**:
- Extremely fast to compute (diagonal multiplication)
- Never explodes at a singularity ($J$ is not in the calculation)

**Downside**: loses task-space intuition -- you cannot individually set "Z soft, XY stiff", because joint angles do not map one-to-one with task directions.

### Why Franka Panda has a dual-mode design

Near a singularity or at the instant of an uncontrolled collision, the low-level layer **automatically switches to joint impedance as a "spring fallback"**, preventing Cartesian-matrix computation from blowing up. This is a key design choice that lets Franka score highly in ISO/TS 15066 certification.

### 7-DoF redundancy null-space compensation

A 7-axis arm has 1 redundant DoF --> infinitely many joint angles can achieve the same end-effector pose --> you can run an end-effector task and a null-space secondary task at the same time:

$$
\tau_{\text{cmd}} = J^T F_{\text{cartesian}} + \left(I - J^T J^{+T}\right)\tau_{\text{null}}
$$

The **projection matrix** $(I - J^T J^{+T})$ projects $\tau_{\text{null}}$ into the null space that "does not affect the end-effector pose".

**Engineering use case**:
- Main task: high end-effector $K_{\text{task}}$ (precise tracking of the target pose)
- Null space: low $K_{\text{null}} \approx 0$ (elbow compliant, can be pushed aside by a human or an obstacle)
- Achieves the two-layer compliance of "tip precise, elbow gently pushable"

**Platforms**: Franka Emika Panda and KUKA iiwa both ship this architecture by default.

### The symmetric positive-definite (SPD) trap

Cartesian K cannot be a simple diagonal! Translation and rotation in SE(3) are coupled:

- Rotational error must use **unit-quaternion error**, never raw Euler differences
- The K matrix must be **symmetric positive-definite (SPD)**
- Without SPD design + correct rotation error, a large angular deflection produces **non-conservative forces** --> passivity is destroyed --> wild oscillation

Franka's libfranka API forces Cartesian K to be SPD; that requirement is the line between "has read Hogan's papers" and "only knows the formula".

</details>

### Impedance vs Admittance -- causal difference and selection decision

**Causal-direction comparison**:

| Aspect | Impedance | Admittance |
|--------|-----------|------------|
| Causality | Position error --> force (the virtual MDK computes a restoring torque $\tau$) | Force --> position command (F/T reading --> MDK model computes a new target pose --> low-level position loop tracks it) |
| Hardware requirement | **Low-friction actuation** (torque-controlled joints); high-friction gearboxes eat the tiny compliant torques | **High rigidity + high-quality F/T sensor**; the motor does not need precise torque control |
| Representative platforms | Franka Panda, KUKA iiwa, MIT Cheetah legs | ABB IRB, KUKA KR large series (F/T add-on) |
| Typical applications | Collaborative arms, human-robot interaction, legged robots | Heavy-payload industrial arms, precision assembly |

<details>
<summary>Deep dive: admittance limit-cycle trap against a rigid environment + hybrid architecture</summary>

### Limit-cycle oscillation when admittance hits a rigid wall

**Disaster chain**:
1. The F/T sensor measures a rigid impact --> a large impulsive force instantaneously
2. The admittance model computes a large position correction --> the arm lifts off contact in a blink
3. External force goes to zero --> the position controller pulls back to the original pose --> it hits again
4. Communication + position-loop delay at millisecond scale --> the causal chain amplifies in reverse --> **limit-cycle oscillation**

This is admittance's fatal weakness: naturally unstable in rigid environments (metal on metal).

### Hybrid impedance/admittance architecture

The industrial answer -- **dynamic switching**:

| Stage | Strategy | Why |
|-------|----------|-----|
| Free space + kinesthetic teaching | Admittance | Stability + absolute safety |
| Contact tasks (peg-in-hole, grinding) | Impedance | High-bandwidth force response, no limit-cycle risk |
| Near a singularity | Joint impedance | Avoid Cartesian matrix explosion |

**Key to switching**: use a ramp function to smoothly blend $K_d, D_d$, avoiding torque spikes at the transition.

### One-line decision rule

"Can your arm's actuator **directly and accurately control torque**?"
- Yes --> Impedance
- No --> Admittance
- Both required --> Hybrid

This is the 10-second line to work back from hardware to architecture.

</details>

### Force-sensing options (directly set the control bandwidth and accuracy)

| Sensing option | Bandwidth | Strength | Weakness | Platforms |
|----------------|-----------|----------|----------|-----------|
| Current-based estimation | High (kHz) | No extra hardware, high bandwidth | Friction / backlash interference, low accuracy | Generic servo drivers |
| Joint-torque sensors | Mid-high | Whole-arm collision detection, whole-body compliance | Costly, one per joint | KUKA iiwa, Franka |
| End-effector F/T sensor | Mid (>1 kHz) | Full 6-D end-effector force, no transmission error | Needs gravity compensation, adds tip inertia | ATI, Robotiq FT 300 |
| Tactile array | High (>100 Hz) | Local micro-slip detection, friction-cone boundary | Small coverage, heavy processing | GelSight, DIGIT, Xela uSkin |

<details>
<summary>Deep dive: operational space + adjoint transform -- why a diagonal K goes unstable when holding a long stick</summary>

### Screw-theory dual spaces

- **Twist** $V = [\omega, v]^T \in \mathbb{R}^6$ (motion, angular + linear velocity)
- **Wrench** $W = [m, f]^T \in \mathbb{R}^6$ (force, torque + force)
- Dual inner product $P = V^T\cdot W$ = **power** (rate of work done)

This duality tells us: wrench and twist live on SE(3) and you cannot freely lump them in one matrix -- the units are not even consistent (N vs Nm).

### Adjoint transform -- cross-frame wrench conversion

Translate a wrench from the gripper centre A to the tool tip B:

$$
W_A = \mathrm{Ad}_{T_{AB}}^T \cdot W_B, \quad \mathrm{Ad}_T^T = \begin{bmatrix}R & \hat{p}R \\ 0 & R\end{bmatrix}
$$

where $\hat{p}$ is the skew-symmetric matrix of the translation vector.

**Trap**: a pure force $f$, once translated, **couples out an extra moment** $m$ via $\hat{p}\cdot R\cdot f$. Developers who ignore this term issue commands that are simply wrong.

### Why a diagonal K destabilises a 30 cm stick poking a wall (an interview classic)

**Scenario**: an arm holds a 30 cm stick poking a wall, with Cartesian K = diag($K_x, K_y, K_z, K_{wx}, K_{wy}, K_{wz}$) defined at the flange origin.

**Instability mechanism**:
1. A tiny tip displacement at the stick end (a pure translation error)
2. When projected back to the flange frame --> the **lever arm amplifies it into a huge rotational error**
3. The diagonal K cannot supply the translation <--> rotation off-diagonal coupling needed for a correct restoring force
4. **Passivity is violated** --> non-conservative forces accumulate --> violent oscillation

### Fix -- adjoint-transform consistency

Franka / KUKA iiwa FRI APIs mandate: K must be a strict SPD matrix defined **relative to the current TCP** (tool centre point).

```cpp
Eigen::Matrix6d Ad_T;
Ad_T.topLeftCorner(3,3)     = R;
Ad_T.bottomRightCorner(3,3) = R;
Ad_T.bottomLeftCorner(3,3)  = skew_symmetric(p) * R;  // lever-arm coupling term
Ad_T.topRightCorner(3,3)    = Eigen::Matrix3d::Zero();
Eigen::Matrix6d K_tcp = Ad_T.transpose() * K_flange * Ad_T;  // consistent transfer
```

### Physical intuition for the operational-space inertia $\Lambda$

$\Lambda(q) = (J M^{-1} J^T)^{-1}$

When you push the end-effector by hand, "how heavy it feels" in different directions is fully determined by $\Lambda$:
- Pushing along X feels heavy --> $\Lambda_{xx}$ is large
- Pushing along Y feels light --> $\Lambda_{yy}$ is small
- Same end-effector pose, different joint configuration --> $\Lambda$ changes completely

That is why advanced impedance control does $\Lambda$-weighted dynamics decoupling -- only then does "set $K_d = 500$" produce the same feel in any pose.

</details>

## Intuition

**Impedance = holding a steering wheel on a bungee cord**: you hold the wheel (desired pose $x_d$), and between your arm and the wheel there is a bungee ($K_d$) plus a damper ($D_d$). Bumps (external force) push the wheel off-centre; your hand is not locked (pure position control), but allows some displacement and gently pulls it back. Higher $K_d$ = tighter cord = smaller deviation but more ringy; higher $D_d$ = heavier damping = slower return but steadier.

**Admittance = pushing a supermarket cart**: you push the cart (apply $F_{\text{ext}}$); the cart decides how fast and how far it moves based on your push. Push harder --> it travels further. The admittance controller is the mass-plus-friction of the cart, setting "how much force turns into how much displacement". Industrial arms are rigid (like a heavy cart), poorly suited for direct force control, but an excellent fit for "tell me the force, I'll convert it into displacement" admittance.

**Hybrid = erasing a blackboard with an eraser**: the hand's normal direction (pressing into the board) runs force control -- maintain a constant pressure so the eraser stays flush; the tangent direction (left-right motion) runs position control -- follow the wipe trajectory precisely. The selection matrix is the physical spec saying "this axis listens to force, that axis listens to position".

**Variable impedance = muscles tensing and relaxing**: when carrying heavy things your arm tenses (high K holds the pose); when threading a needle your arm relaxes (low K allows micro-adjustment). A VSA (Variable Stiffness Actuator) is the mechanical copy of that; RL-based variable impedance is the software copy.

**Tactile sensing = a blind person feeling**: an F/T sensor only measures "the total force on the whole hand" and cannot distinguish "micro-slip at the fingertip"; a tactile array is like the pad of a blind finger -- each taxel (tactile pixel) senses independently. When gripping a slippery glass, net force may be zero yet tangential micro-slip at the fingertips is already happening -- only the tactile array sees it.

**What to watch in a simulator**:
- **MuJoCo**: set a UR5 + F/T sensor pressing on a spring wall. First run pure position control -- on contact, the force spikes and can even bounce away. Switch to impedance control ($K_d = 500, D_d = 50$) -- compliant press-in, force converges smoothly. Push $K_d \to 5000$ --> approaches position control (hard slam); $K_d = 0$ + small $D_d$ --> lead-through teaching mode. **Caveat: MuJoCo's soft contact allows slight penetration** -- a classic sim-to-real trap for RL training (see practice question Q4).
- **Isaac Sim**: run a peg-in-hole with Franka and watch a variable-impedance policy reduce K and raise D at the jam moment to wiggle free.
- **Gazebo**: at the instant ANYmal's foot lands, the SEA spring shows a clearly damped 50-100 Hz oscillation in the torque signal -- visual evidence that "the spring absorbed the impact".

## Implementation Link

**Three typical engineering scenarios**:

1. **Collaborative-robot lead-through (kinesthetic) teaching**: the operator pushes the arm by hand, the robot follows, and releasing stops the motion. Core: admittance (or transparent-mode impedance) + $K_d = 0$ (no position restoring force), leaving only a small $D_d$ for smooth feel. Hard prerequisite: accurate gravity + friction compensation -- otherwise the arm's own weight makes the operator feel "stuck" or "falling down".

2. **Surface finishing / polishing**: hybrid architecture -- normal axis runs PI force control to hold 10 N constant pressure; tangential axes run position control to follow the finishing trajectory. Needs F/T dynamic compensation (subtract tool gravity and inertia force) + adaptive stiffness (adjust $K_d$ on curvature change) + velocity saturation (prevent a slam into the floor when the tool leaves the workpiece).

3. **Precision assembly (peg-in-hole)**: position-control approach --> contact detection --> spiral search --> force-control insertion. A state machine handles switching; the biggest impulses happen at the switching instant. Modern practice combines residual RL for un-modeled friction (see practice question Q2).

**Code skeleton** (C++, ROS 2 + ros2_control):

```cpp
// Impedance controller -- inside the ros2_control update() loop
// Inputs: current_pose, desired_pose, measured_wrench
// Output: joint_torques
// Rate: 1 kHz (hard real-time)

Eigen::Vector6d pose_error = compute_pose_error_quat(current_pose, desired_pose);
Eigen::Vector6d vel_error  = current_velocity - desired_velocity;

// 1. Use the adjoint to move K from the flange to the TCP (see deep-dive)
Eigen::Matrix6d K_tcp = compute_adjoint_K(K_flange, T_flange_to_tcp);
Eigen::Matrix6d D_tcp = compute_adjoint_K(D_flange, T_flange_to_tcp);

// 2. Impedance force = K * e + D * e_dot (Cartesian)
Eigen::Vector6d F_impedance = K_tcp * pose_error + D_tcp * vel_error;

// 3. Add force-tracking error (if a desired Fd is given)
Eigen::Vector6d F_total = F_impedance + F_feedforward;

// 4. Map back to joint space
Eigen::VectorXd tau = jacobian.transpose() * F_total;

// 5. Add dynamics compensation (gravity + Coriolis)
tau += gravity_compensation(q) + coriolis_compensation(q, qd);

// 6. 7-DoF null-space compensation (elbow compliance)
Eigen::MatrixXd null_projector = Eigen::MatrixXd::Identity(7,7)
                                 - jacobian.transpose() * jacobian_pinv.transpose();
tau += null_projector * tau_null;

// 7. Singularity protection: if the condition number of J is too large, fall back to joint impedance
if (jacobian.jacobiSvd().singularValues().tail(1)(0) < SINGULAR_THRESHOLD) {
    tau = K_joint * (q_desired - q) - D_joint * qd;
}

hardware_interface->set_command(tau);
```

<details>
<summary>Deep dive: full Python admittance-control implementation (MuJoCo environment) + gravity-compensation calibration</summary>

```python
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

class AdmittanceController:
    """
    Admittance controller: F_ext -> position correction -> position controller
    Use case: compliant control on high-reduction industrial arms
    """
    def __init__(self, Md, Dd, Kd, dt=0.001):
        self.Md = np.diag(Md)  # desired inertia [6,]
        self.Dd = np.diag(Dd)  # desired damping [6,]
        self.Kd = np.diag(Kd)  # desired stiffness [6,]
        self.dt = dt
        self.x_cmd = np.zeros(6)
        self.x_cmd_dot = np.zeros(6)

    def update(self, F_ext: np.ndarray) -> np.ndarray:
        """
        Input: 6-D external wrench F_ext (after tool-gravity compensation)
        Output: 6-D position correction x_cmd
        """
        Md_inv = np.linalg.inv(self.Md)
        x_cmd_ddot = Md_inv @ (F_ext - self.Dd @ self.x_cmd_dot
                                      - self.Kd @ self.x_cmd)

        # Semi-implicit Euler integration (more stable than explicit)
        self.x_cmd_dot += x_cmd_ddot * self.dt
        self.x_cmd += self.x_cmd_dot * self.dt

        return self.x_cmd.copy()

    def reset(self):
        self.x_cmd = np.zeros(6)
        self.x_cmd_dot = np.zeros(6)


def calibrate_ft_sensor(raw_wrenches, poses):
    """
    Multi-pose calibration of the F/T sensor: identify tool mass and centre of mass
    raw_wrenches: [N, 6] raw readings across several poses
    poses: [N, 3, 3] corresponding rotation matrices (sensor frame wrt world)
    Returns: tool_mass (scalar), tool_com (3,)
    """
    g_world = np.array([0, 0, -9.81])

    # Linear least squares: F_sensor = m * R^T @ g_world + noise
    # Fit m on the force part
    A = np.vstack([p.T @ g_world for p in poses])  # [N*3, 1] (flattened)
    b = raw_wrenches[:, :3].flatten()
    tool_mass = np.linalg.lstsq(A.reshape(-1, 1), b, rcond=None)[0][0]

    # Identify com on the torque part
    # T_sensor = com x (m * R^T @ g_world)
    # Build a skew-sym matrix per pose
    A_com, b_com = [], []
    for F_raw, T_raw, R_p in zip(raw_wrenches[:,:3], raw_wrenches[:,3:], poses):
        F_grav = tool_mass * R_p.T @ g_world
        # T = skew(com) @ F_grav -> T = -skew(F_grav) @ com
        A_com.append(-np.array([[0, -F_grav[2], F_grav[1]],
                                [F_grav[2], 0, -F_grav[0]],
                                [-F_grav[1], F_grav[0], 0]]))
        b_com.append(T_raw)
    A_com = np.vstack(A_com)
    b_com = np.concatenate(b_com)
    tool_com = np.linalg.lstsq(A_com, b_com, rcond=None)[0]

    return tool_mass, tool_com


def gravity_compensate_ft(raw_wrench, tool_mass, tool_com, R_sensor):
    """
    F/T gravity compensation: remove the projection of tool weight on the sensor frame
    """
    g_world = np.array([0, 0, -9.81])
    g_sensor = R_sensor.T @ g_world
    F_gravity = tool_mass * g_sensor
    T_gravity = np.cross(tool_com, F_gravity)

    compensated = raw_wrench.copy()
    compensated[:3] -= F_gravity
    compensated[3:] -= T_gravity
    return compensated


# === Usage examples ===
# Lead-through teaching mode: Kd=0 (no restoring force), small damping
controller_teach = AdmittanceController(
    Md=[1.0]*6,       # small inertia -> fast response
    Dd=[10.0]*6,      # small damping -> light push moves it
    Kd=[0.0]*6,       # zero stiffness -> lead-through mode
    dt=0.001
)

# Grinding mode: stiffness only on the normal (hold pressure), zero on tangents
controller_grind = AdmittanceController(
    Md=[1.0]*6,
    Dd=[50.0, 50.0, 80.0, 10.0, 10.0, 10.0],
    Kd=[0.0, 0.0, 500.0, 0.0, 0.0, 0.0],  # stiffness only on z
    dt=0.001
)
```

</details>

<details>
<summary>Deep dive: full impedance vs admittance hardware-selection decision tree</summary>

```
Hardware entry point
|
+- Motor type is direct drive / low reduction (< 10:1)?
|   +- Yes -> good back-drivability
|   |        -> prefer **Impedance Control**
|   |        -> Reason: direct torque output; basic compliance works without an F/T sensor
|   |        -> Examples: Franka Panda, MIT Cheetah legs, Tesla Optimus
|   |
|   +- No  -> High reduction (> 50:1) harmonic drive / planetary gear
|            -> poor back-drivability; torque output heavily affected by friction/backlash
|            -> prefer **Admittance Control**
|            -> Reason: exploit high-accuracy position control; F/T measures external force and computes displacement correction
|            -> Examples: UR series, KUKA iiwa (iiwa ships joint torque sensors, so both modes work)
|
+- Is there an end-effector F/T sensor?
|   +- Yes -> admittance is immediately usable
|   |        impedance also works (as an outer loop for force tracking)
|   |
|   +- No  -> Are there joint-torque sensors?
|            +- Yes -> use a generalized momentum observer to estimate external force
|            +- No  -> current-based estimation only; trustable on QDD (low reduction)
|
+- Does the task need hybrid position/force (force in some directions, position in others)?
|   +- Yes -> Hybrid Position/Force (selection matrix splits directions)
|   |        underlying layer can be impedance or admittance
|   +- No  -> pure compliant task -> single mode is fine
|
+- How rigid is the environment?
    +- Extremely rigid metal on metal -> avoid admittance (limit-cycle trap)
    +- Soft surface (sponge, cloth)   -> either works
    +- Unknown environment            -> variable impedance + online RL adaptation
```

**Rules of thumb**:
- Collaborative arms (Franka, KUKA iiwa): impedance-first, joint-torque sensors enable whole-body compliance
- Traditional industrial arms (UR, ABB, Fanuc): admittance-first, paired with an end-effector F/T sensor
- Legged robots (ANYmal, Spot): SEA + impedance (fast torque response for ground reaction control)
- Humanoids (Optimus, Figure): QDD + impedance (low reduction ratio trades for ultimate transparency)
- Dexterous hands (Shadow Hand, Allegro): impedance + tactile array

</details>

<details>
<summary>Deep dive: SEA / VSA / QDD / harmonic drive -- actuator philosophies and the selection triangle</summary>

### Four actuator philosophies

**SEA (Series Elastic Actuator) -- "use a sensor to fix an imperfect actuator"**
- **Physical spring in series** between the motor and the link
- Gearboxes have friction/backlash -> the spring = natural low-pass filter -> absorbs landing impacts to protect the harmonic drive
- A high-resolution encoder measures spring deflection $\Delta x$ -> Hooke's law $F = K \Delta x$ -> **pure, friction-free reading of true contact force**
- Representative platforms: ANYmal quadruped, Baxter, Valkyrie

**VSA (Variable Stiffness Actuator) -- "a mechanical copy of biological muscle"**
- Two antagonistic motors + non-linear spring/cam -> physically variable stiffness at the hardware level
- Humans lift heavy things with tense muscles (high K), thread needles with relaxed muscles (low K)
- Can go soft instantly, store and release explosive energy (throwing, jumping)
- Representative platforms: DLR Hand-Arm System, IIT Walk-Man, MIT Nadia

**QDD (Quasi-Direct Drive) -- "drop the spring, kill the friction directly"**
- Large-torque outrunner motor (pancake motor) + extremely low reduction 6:1 ~ 9:1 planetary gear
- Excellent back-drivability -- external force easily back-drives the rotor
- Current loop $I_q$ -> electromagnetic torque $\tau_m = K_t \cdot I \cdot N$ -> nearly equals true external contact torque
- **Transparent force control without an F/T sensor**
- Representative platforms: MIT Cheetah, Tesla Optimus, Unitree G1

**Harmonic drive -- "ultimate precision + end-effector F/T sensing"**
- Reduction 50:1 ~ 160:1, zero backlash, high stiffness
- Poor back-drivability -> must pair with an end-effector F/T sensor and run admittance
- Representative platforms: UR series, Fanuc, precision surgical robots

### Selection triangle -- bandwidth x impact x precision

| Scenario | Choice | Why |
|----------|--------|-----|
| 0.05 mm orthopaedic surgery | Harmonic drive + end-effector 6-D F/T | Ultimate precision |
| Quadruped crossing rubble (high-frequency ground impact) | SEA | Physical spring protects the gearbox |
| Humanoid arm (catching a ball / handshake) | QDD | Low reduction for ultimate transparency + current-level force control |
| Research platform with variable stiffness | VSA | Throwing, jumping, safety |
| Industrial collaborative arm | Harmonic drive + joint-torque sensors | Franka / iiwa design line |

### QDD without F/T external-force estimation, Python sketch

```python
def estimate_external_torque_qdd(motor_current, q, q_dot, q_ddot, K_t, N):
    """
    External-force estimation on a QDD actuator without an F/T sensor
    Principle: tau_m - tau_dynamics ~= tau_ext (QDD friction is tiny; ignore or compensate simply)
    """
    tau_m = K_t * motor_current * N  # electromagnetic torque
    tau_dyn = compute_RNEA(q, q_dot, q_ddot)  # dynamics feedforward
    tau_fric = coulomb_friction_model(q_dot)  # QDD friction is minimal
    tau_ext = tau_m - tau_dyn - tau_fric
    return tau_ext
```

**Interview talking point**: answering "why does your humanoid pick QDD over a harmonic drive?" in 30 seconds = proof of deep understanding of force-control hardware.

</details>

## Common Misconceptions

1. **"Position control alone is enough for precision assembly."** Parts have tolerances (typically +/-0.05 mm); the hole and peg positions always have a small offset. Pure position control slams --> jam or part damage. **Correct view**: assembly needs compliance -- at minimum switch to force or impedance control on contact so the robot "follows the push" into the hole. The industrial standard is position approach + spiral exploration + impedance-controlled insertion + adaptive stiffness anti-jam (residual RL handles un-modeled friction; see practice Q2).

2. **"Impedance and admittance are the same thing, just different names."** The causal direction is exactly opposite. Impedance: displacement error --> force (like a spring, you push, it pushes back). Admittance: external force --> position correction (like a cart, you push, it moves). Choosing wrong destabilises the system -- impedance on a high-reduction arm has friction eating torque-control accuracy; admittance on a direct-drive arm wastes its direct torque-control advantage; admittance hitting a rigid wall produces a limit-cycle oscillation.

3. **"The F/T reading is the contact force."** Raw readings include tool gravity (varies with pose) and inertia forces from acceleration. Without gravity compensation the robot "feels" fake contact force at rest --> controller thrashes. **Correct practice**: on each boot run an F/T calibration (multi-pose sampling + least-squares identification of tool mass and CoM); at runtime subtract $F_{\text{gravity}} = m_{\text{tool}}\cdot R^T g$ and the inertia term $m\ddot{x}$.

4. **"Turning $K_d$ way up in impedance control makes it as precise as position control."** In theory $K_d \to \infty$ approaches position control. In practice, too-high $K_d$ --> under-damped system --> high-frequency oscillation on contact --> in the worst case damages parts or hurts a person. Worse, **high $K_d$ + a singularity = an instant joint-torque explosion** (the condition number of $J^T$ amplifies error). **Correct practice**: tune $K_d$ and $D_d$ together, holding a critical damping ratio $\zeta = D_d / (2\sqrt{K_d M_d}) \approx 0.7 \sim 1.0$; add a singularity-protection switch to joint impedance.

5. **"Cartesian impedance K only needs diagonal entries."** Translation and rotation in SE(3) are coupled; a diagonal K is not SPD, and far from the origin (e.g. holding a 30 cm stick) it produces non-conservative forces that violate passivity --> wild chatter. **Correct practice**: K must be SPD, rotational error must use a unit quaternion, and cross-frame transforms must use the adjoint (see deep-dive). Franka's libfranka API enforces exactly this.

## Practice Questions

<details>
<summary>Q1 (medium): A Franka Panda 7-axis arm does precision assembly on a factory floor where operators may brush any part of it at any time. Design the full whole-body impedance + collision detection + graded-response architecture.</summary>

**Full reasoning chain**:

1. **Collision-detection layer -- momentum observer**:
   - Every joint has a torque sensor; compute the residual $\tau_{\text{external}} = \tau_{\text{measured}} - \tau_{\text{model}}$ ($\tau_{\text{model}}$ includes dynamics + friction feedforward)
   - Set threshold at 15 N*m (consistent with ISO/TS 15066 PFL)
   - Residual above threshold --> collision detected

2. **Graded-response state machine**:
   - **Light (< 30 N)**: switch to zero-force drag mode ($K_d \to 0$), allow operator to push aside
   - **Medium (30-60 N)**: compliant-rebound mode ($K_d$ reduced 50% + $D_d$ doubled) to absorb energy
   - **Heavy (> 60 N)**: Category-0 e-stop (direct power-off)

3. **Null-space layered impedance**:
   - Main task (end-effector 6D): high $K_{\text{task}}$ to preserve assembly precision (elbow pushes must not disturb the tip)
   - Null space (1 redundant DoF): $K_{\text{null}} \approx 0$, elbow can be pushed gently out of the way
   - Projection: $\tau_{\text{cmd}} = J^T F_{\text{task}} + (I - J^T J^{+T})\tau_{\text{null}}$

4. **Singularity protection**:
   - Monitor the smallest singular value of $J$ continuously
   - If it falls below a threshold --> switch to joint impedance as a spring fallback, preventing the condition number of $J^T$ from exploding

**What the interviewer wants to hear**: momentum-observer collision detection (no F/T needed) + graded response aligned with ISO/TS 15066 + null-space layering for "tip precise, elbow compliant" + singularity protection.

</details>

<details>
<summary>Q2 (hard): Peg-in-hole with +/-0.05 mm tolerance will always jam under pure position control. Design the full staged compliant strategy, including a jam-escape behaviour.</summary>

**Full reasoning chain**:

1. **Stage 1 -- position-control approach**:
   - Pure position control to 5 mm above the hole, fast motion
   - Drop speed to 10 mm/s
   - Pre-reduce $K_d$ from 2000 to 500, preparing for contact

2. **Stage 2 -- contact detection**:
   - Press down while monitoring $F_z$
   - $F_z > 5\text{N}$ --> stop the downward press, enter the next stage
   - Use a ramp function (50-200 ms) to smooth the impedance transition

3. **Stage 3 -- Archimedean spiral search**:
   - XY trace a spiral $x = r(t)\cos\theta, y = r(t)\sin\theta$ with $r$ growing linearly
   - Z holds a constant 5 N press
   - Monitor $F_z$: if it drops off a cliff --> fell into the hole, next stage

4. **Stage 4 -- hybrid force/position insertion**:
   - Z axis: force control (maintain 8 N insertion force)
   - XY axes: position control (hold the centre)
   - Yaw axis: low-K impedance (compliance tolerates tolerance mismatch)

5. **Stage 5 -- jam self-rescue (adaptive impedance)**:
   - Monitor lateral force $\|F_{xy}\|$
   - Abnormal increase (e.g. > 3 N) --> declare jam
   - Auto-reduce $K$, raise $D$ (e.g. K down 50%, D doubled)
   - Small wiggle (+/- 2 deg) so parts re-align --> continue insertion

**Advanced -- residual RL for un-modeled friction**:
- The base controller handles 90% of known dynamics
- An RL network outputs only a small residual $\Delta\tau$ to fit the 10% of un-modeled friction
- Sample efficiency x100 + safety fallback from the base controller

**What the interviewer wants to hear**: staged state machine + spiral-search math + hybrid architecture at the insertion stage + adaptive impedance for jams + awareness of modern residual RL.

</details>

<details>
<summary>Q3 (hard): a UR5 wipes a glass tabletop with pure Z-axis force control at 10 N, but when the cloth slides off the edge the arm accelerates into the floor. Explain the disaster and how to fix it.</summary>

**Full reasoning chain**:

1. **Disaster mechanism**:
   - $S_{zz} = 1$ (Z axis pure force control), desired $F_z = 10\text{N}$
   - Cloth slides off the edge --> $F_{\text{ext}} = 0$ (nothing to push against)
   - Force control law: $F_{\text{cmd}} = K_p(F_d - F_{\text{ext}}) = 10\text{N}$ (constant)
   - The arm keeps accelerating down to reach 10 N
   - **There is no position feedback on Z** (pure force control $S_{zz}=1$, $(I-S)_{zz}=0$) --> nothing stops the downward charge
   - Ultimately it slams into the floor at high speed

2. **Root cause**:
   - The purely force-controlled axis **has no position/velocity bound**
   - The fatal weakness of hybrid control: an $S=1$ axis has zero positional safety margin

3. **Fix -- velocity saturation**:
   ```cpp
   Eigen::Vector6d wrench_cmd = Kp_f * (F_d - F_measured);
   wrench_cmd(2) = std::clamp(wrench_cmd(2), -max_vel_z, max_vel_z);
   ```

4. **Fix -- damping injection**:
   - Inject a virtual damping term $-D_v \dot{x}$ on the force-controlled axis
   - Even with no external force, the faster it moves the larger the virtual drag --> natural deceleration

5. **Fix -- transition to soft hybrid**:
   - Drop the crisp switch $S_{zz}=1$
   - Switch to a low Cartesian-K impedance on Z (soft Z) + force feedforward
   - Impedance has inherent position feedback and will not charge off on loss of contact

6. **Advanced -- contact-loss detection exit**:
   - Monitor $F_z$: if zero for N consecutive ms --> declare loss of contact
   - Immediately switch back to position control and lift Z to a safe height

**What the interviewer wants to hear**: the fatal weakness of a hybrid crisp switch (force-controlled axis has no positional bound) + velocity saturation / damping injection as two engineering fixes + why impedance is safer (inherent position feedback) + the contact-detection state machine.

</details>

<details>
<summary>Q4 (hard): Design a variable-impedance-learning system so an arm wiping a surface of unknown stiffness (metal / sponge / cardboard) auto-tunes K and D.</summary>

**Full reasoning chain**:

1. **Why fixed K/D is not enough**:
   - Rigid metal: small K to avoid rebound oscillation
   - Soft sponge: large K to preserve tracking accuracy
   - Unknown environments cannot be pre-specified

2. **Layered architecture**:
   - **Brain "slow thinking" layer (RL, 10 Hz)**: a SAC network outputs $\Delta K, \Delta D$
   - **Cerebellum "fast reflex" layer (impedance, 1 kHz)**: modulate parameters via $K_t = K_{\text{base}} + \text{tanh}(\text{NN})\cdot a_{\text{scale}}$
   - Guarantee: the 1 kHz impedance controller preserves Lyapunov stability

3. **RL design**:
   - **State**: $s = \{e_x, \dot{x}, F_{\text{measured}}, \tau_{\text{ext}}\}$
   - **Action**: $a = \{\Delta K, \Delta D\}$ (output parameter deltas only, never joint torques)
   - **SAC vs PPO**: SAC wins; max-entropy encourages exploration of diverse contact strategies + robustness to non-linear friction beats PPO

4. **The "lazy policy" trap (must-know)**:
   - Reward only on error --> RL learns "crank K to max, be as stiff as possible --> error minimal" --> loses the point of compliance
   - **Correct reward**: $R = -\|e_x\|^2 - w\cdot\|\tau_{\text{cmd}}\|^2$ (add an energy penalty)
   - Forces "push when needed, relax the rest of the time"

5. **Sim-to-real penetration trap**:
   - MuJoCo's soft contact allows small penetration
   - RL may learn "K -> infinity, squeeze into the object to score points" --> cheating via penetration
   - Fix: (1) domain-randomise contact stiffness (2) hard saturation on action, $K \leq K_{\max}$ via a sigmoid

6. **Why layered, not end-to-end RL**:
   - End-to-end RL outputs torque --> inference latency > 10 ms --> force control collapses
   - RL at 10 Hz outputs K/D, underlying C++ impedance runs at 1 kHz --> deep-learning non-linear adaptation + classical control's 100% high-frequency stability guarantee

**What the interviewer wants to hear**: why RL is needed (environment unknown) + layered design (RL slow + impedance fast) + SAC max-entropy + lazy-policy trap + sim-to-real penetration trap + matching fixes. This distinguishes candidates who have actually read 2023 papers.

</details>

<details>
<summary>Q5 (hard): Integrate modern imitation-learning methods like ACT (Action Chunking Transformer) or Diffusion Policy into contact-rich tasks. Design the layered architecture with low-level impedance.</summary>

**Full reasoning chain**:

1. **Why imitation learning alone is not enough**:
   - VLM / Transformer inference is slow (1-10 Hz); outputting torque directly --> force control collapses
   - Contact-rich tasks need > 100 Hz force feedback to stay compliant

2. **ACT Action Chunking strengths**:
   - Traditional BC predicts a single step $a_t = \pi(s_t)$ --> tiny errors amplify geometrically (compounding error)
   - ACT predicts an absolute-pose sequence $a_{t:t+k}$ for the next k steps at once
   - 1000 Hz low-level tracking + absolute poses --> filters high-frequency teleoperation jitter
   - Success stories: needle threading, cracking eggs (delicate bimanual tasks)

3. **Diffusion Policy strengths**:
   - Contact-rich tasks often have multi-modal action distributions ("go around left" vs "go around right")
   - MSE-based Transformers **average the two** --> the arm charges straight into the obstacle
   - Diffusion: denoise from Gaussian noise step by step --> precisely fits multimodal distributions
   - Objective: $L = E[\|\epsilon - \epsilon_\theta(x_k, k, O)\|^2]$

4. **Layered integration (visuomotor x impedance)**:
   - **Brain (VLM like π0 / OpenVLA) at 1-10 Hz**: outputs task-level commands (go to a target pose)
   - **Midbrain (ACT / Diffusion Policy) at 10-100 Hz**: outputs waypoints / low-level target poses $x_{\text{target}}$
   - **Cerebellum (Cartesian impedance) at 1 kHz**: $\tau = J^T[K(x_{\text{target}} - x) - D\dot{x}]$
   - Contact-force compliance is entirely handed to the low-level impedance

5. **Why Diffusion Policy beats MSE**:
   - Wiping glass "left-to-right vs right-to-left" are both valid --> MSE averages = arm stuck in the middle hitting the workpiece
   - Diffusion denoises to a single mode, picks one stochastically, executes it --> naturally smooth + multimodal

6. **SOTA platforms**:
   - ALOHA / ACT: fine-grained bimanual manipulation
   - Diffusion Policy (Cheng Chi et al.): general manipulation
   - π0 / OpenVLA: VLM backbones, the endgame architecture of embodied AI

**What the interviewer wants to hear**: ACT Action Chunking solves compounding error + Diffusion Policy solves multimodality + slow-brain / fast-cerebellum layered architecture + why a VLM does not directly output torque. These are the two pillars of contact-rich breakthroughs from 2022-2024 and the next three years' battleground.

</details>

<details>
<summary>Q6 (hard): Implement kinesthetic teaching (transparent mode) that also distinguishes "active guidance" from "unexpected collision" and triggers an e-stop.</summary>

**Full reasoning chain**:

1. **Transparent-mode math**:
   - Impedance law $M\ddot{x} + D\dot{x} + Kx = F_{\text{ext}}$
   - $K \to 0, D \to$ very small --> transparent (robot weight almost unfelt)
   - A human applies only a few N to overcome residual inertia --> the arm glides wherever the hand pulls

2. **Avoid sagging under gravity -- precise dynamics feedforward**:
   - $\tau_{\text{ff}} = G(q) + \tau_{\text{fric}}(q, \dot{q})$ applied in real time
   - Release and stop -- gravity is compensated, no drooping

3. **Franka libfranka zero-force mode (example code)**:
   ```cpp
   auto zero_force = [&](const RobotState& s, Duration dt) -> Torques {
       // Franka's internal layer already compensates gravity / Coriolis
       return {0,0,0,0,0,0,0};  // output zero torque = follows wherever the human pulls
   };
   robot.control(zero_force);  // 1 kHz torque control loop
   ```

4. **Separating guidance from collision -- spectral analysis**:
   - **Unexpected collision**: rigid contact, $dF/dt$ very large --> **high-frequency impulse**
   - **Active drag**: human muscles exert force --> **low-frequency < 2 Hz**
   - FFT or a high-pass filter peels off the collision signal instantly --> raise the e-stop

5. **Auxiliary discrimination from tactile distribution**:
   - **Guidance**: multi-finger, wide-area steady wrap (grasp/push)
   - **Collision**: single sharp point of contact
   - If a tactile array (e.g. ANYskin) is available, analyse the contact pattern further

6. **Momentum-observer collision detection (no F/T needed)**:
   $$
   r(t) = K_o\cdot\left[M(q)\dot{q} - \int(\tau_{\text{cmd}} + C^T\dot{q} - G(q) + r(s))ds\right]
   $$
   - Generalized momentum principle estimates external contact torque
   - Above threshold and with a large rate of change --> raise an ISO 15066 PFL brake

7. **Shared control / virtual fixtures (advanced)**:
   - da Vinci surgical robot: inside the safe region $K=0$ (surgeon fully in control)
   - As the tip approaches a vessel boundary --> $K$ dynamically ramps up, large repulsive force haptic feedback blocks dangerous cuts

**What the interviewer wants to hear**: transparent-mode K->0 math + dynamics feedforward compensating gravity (release without sag) + spectral analysis distinguishing guidance/collision + momentum observer detecting collisions without F/T + awareness of advanced HRI topics like virtual fixtures.

</details>

## Interview Angles

1. **Selection-matrix orthogonality theorem (Mason's 1981 Task Frame Formalism)** -- the single most central and most asked force-control question. **What to say**: "The core of the task-frame formalism is the orthogonality theorem: in a local frame built at the contact point, on any axis the **natural constraint (set by environmental geometry, e.g. $v_z = 0$ when wiping a table)** and the **artificial constraint (the engineer's target, e.g. $f_z = 10\text{N}$) must be orthogonal complements**. That is the strict physical spec for hybrid control's selection matrix -- which axis is position-controlled, which is force-controlled, is not a guess but is read off the environmental constraint. The environment sets position (natural positional constraint) --> you can only apply force on that axis (artificial force constraint), and vice versa. On a curved surface the task frame must also be rotated in real time along the contact normal." **Why this is the key**: being able to talk about S from "diagonal 0/1" up to the "orthogonality theorem" signals you have read Mason 1981, not just a textbook.

2. **Impedance vs admittance selection = hardware logic** -- the industrial-landing litmus test. **What to say**: "Selection boils down to one sentence: 'can your arm's actuator directly and accurately control torque?' Yes --> impedance (Franka, iiwa, QDD); no --> admittance (UR, ABB + F/T add-on). Impedance needs low-friction actuation (torque-controlled joints); high-friction harmonic drives eat the tiny compliant torques. Admittance needs high rigidity + a good F/T sensor; the motor does not have to control torque. **Admittance against a rigid wall limit-cycles** because the causal chain amplifies in reverse; the modern approach is hybrid -- admittance in free space for stability and safety, switch to impedance after contact for high-bandwidth response." **Why this is the key**: this question separates "memorises formulas" from "actually understands hardware"; the 10-second hardware-to-architecture reasoning is the engineering intuition the interviewer is listening for.

3. **Cartesian vs joint impedance + singularity torque explosion** -- the hardcore question separating "can use" from "truly understands". **What to say**: "Cartesian impedance is intuitive (Z soft, XY stiff), but near a singularity where $J$ loses rank the condition number explodes --> $\tau = J^T F$ amplifies F astronomically --> hardware overload e-stop. Franka automatically falls back to joint impedance as a spring near singularities. The other trap is that K must be SPD and rotational error must use a quaternion -- a diagonal K holding a 30 cm stick poking a wall produces lever-arm-coupled non-conservative forces --> violates passivity --> wild chatter. Franka's libfranka API forces K to be SPD for exactly this reason." **Why this is the key**: being asked "how do you do force control near a singularity?" and answering "fall back to joint impedance" + "SPD + quaternion error" proves you have read the papers.

4. **Adjoint transform torque-coupling trap + operational-space $\Lambda$** -- the heart of operational-space control, the line that separates people who have read Khatib 1987. **What to say**: "When a wrench is transformed across frames, a force $f$ translated by $\hat{p}R f$ couples out a torque $m$. Engineers who ignore this term issue commands that are simply wrong. Franka / KUKA iiwa FRI force K to be defined relative to the current TCP. The operational-space inertia $\Lambda = (JM^{-1}J^T)^{-1}$ physically is 'the effective mass of the end-effector in 3-D space in the current pose'; how heavy a hand push feels in different directions is fully set by $\Lambda$. Advanced impedance does $\Lambda$-weighted dynamics decoupling so that setting $K_d=500$ feels consistent in any pose." **Why this is the key**: talking about screw-theory duality (the twist-wrench inner product = power) shows you understand SE(3) geometry, not just patchwork formulas.

5. **SEA / VSA / QDD / harmonic drive -- the selection triangle** -- a must-answer in the humanoid / quadruped era. **What to say**: "The triangle is bandwidth x impact x precision. Orthopaedic surgery at 0.05 mm --> harmonic drive + end-effector F/T; quadrupeds crossing rubble with high-frequency ground impact --> SEA (physical spring absorbs impact, protects the gearbox); humanoid arms catching balls or shaking hands --> QDD (low reduction 6:1 to 9:1 + large-torque outrunner, current-level force control with no F/T); research platforms needing variable stiffness --> VSA (antagonistic motors + non-linear cams, a mechanical copy of muscle). MIT Cheetah and Tesla Optimus choose QDD over harmonic drive precisely for 'transparent force control without F/T + running and jumping'." **Why this is the key**: being able to answer in 30 seconds why Tesla Optimus picks QDD shows you have really read actuator-architecture papers and understand the full chain from hardware to control.

6. **Variable-impedance learning -- slow brain, fast cerebellum** -- the 2023-2024 RL x control frontier. **What to say**: "Fixed M/D/K oscillate in unknown-stiffness environments. RL design: state = $\{e_x, \dot{x}, F, \tau_{\text{ext}}\}$, action = $\{\Delta K, \Delta D\}$ (parameter deltas only, never torques). SAC beats PPO because max entropy encourages diverse contact exploration. **Trap 1: lazy policy** -- reward only error and RL learns K-max stiff --> loses compliance; the correct reward adds an energy penalty $R = -\|e_x\|^2 - w\|\tau\|^2$. **Trap 2: sim-to-real penetration** -- MuJoCo soft contact allows penetration, RL may learn K -> infinity to squeeze inside objects to cheat. Layered design: RL at 10 Hz outputs K/D, C++ impedance at 1 kHz -- fusing deep-learning non-linear adaptation with classical control's 100% high-frequency stability guarantee." **Why this is the key**: this one separates "has chased recent papers" from "only read textbooks"; calling out the lazy-policy and penetration traps proves you have actually tuned RL and done sim-to-real.

7. **Touch matters more than vision (the last 5 cm)** -- the core insight of dexterous manipulation. **What to say**: "An F/T sensor only measures net force on the whole hand; it cannot distinguish fingertip micro-slip (a slippery glass may have zero net force while sliding). GelSight / DIGIT (vision-based) use a camera behind an elastomer + photometric stereo to get an ultra-high-resolution 3-D depth map; Xela uSkin (magnetic) responds > 100 Hz and suits closed-loop control. Tactile-based impedance modulation: detect a shear-force micro-slip edge --> instantly raise K on the normal (tighten grip) and raise D on the tangent (dissipate slip energy). In dexterous manipulation vision runs into a hopeless occlusion disaster -- the hand wraps the object and you cannot see; plus vision cannot measure the friction coefficient, mass distribution, or contact stiffness. **'In the last 5 cm, touch matters more than vision'** is the dividing line for whether embodied AI can enter homes." **Why this is the key**: this is a strategic industry-landing view for embodied AI; citing ANYskin / DextAH and 2024 SOTA shows you follow the frontier.


