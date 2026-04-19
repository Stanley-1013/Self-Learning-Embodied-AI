---
title: "Rigid-Body Forward Kinematics and DH Parameter Modeling"
prerequisites: ["05-ros2-tf-and-tools"]
estimated_time: 45
difficulty: 3
tags: ["forward-kinematics", "dh-parameters", "kinematics"]
sidebar_position: 7
---

# Rigid-Body Forward Kinematics and DH Parameter Modeling

## You Will Learn

- Describe forward kinematics precisely enough to answer any interview question about it without vagueness
- Diagnose "the robot installed a new tool and the end-effector position is off" — you will know to check the Tool Frame and DH zero offsets first
- Decide when to use Standard DH vs Modified DH, and when to drop DH entirely in favor of Product of Exponentials

## Core Concepts

**Precise Definition**: **Forward Kinematics (FK)** computes the end-effector pose (position + orientation) in task space given the joint variables (rotary joint angles, prismatic joint distances). It is the mapping **joint space → task space**.

**DH Parameters (Denavit-Hartenberg)**: a standardized 4-parameter tuple $(\alpha, a, d, \theta)$ that describes the geometric relation between two adjacent links, allowing systematic derivation of multi-DOF robot transforms.

**Location in the Sense → Plan → Control Loop**:
- **Input**: joint vector $q = [\theta_1, \theta_2, \dots, \theta_n]$ (rad or m)
- **Output**: $4 \times 4$ homogeneous transform $^0T_n$ of the end-effector relative to the base frame
- **Downstream consumers**: trajectory planners (collision checking, interpolation), IK solvers (as numerical seed), closed-loop controllers (error signal), visual servoing (hand-eye transform), Rviz TF tree visualization
- **Loop node**: straddles **perception (state estimation)** and **control (feedback)**. FK reads encoder positions to compute the actual end-effector pose, which is compared against the planner's target to form the error signal.

**One-line version**: "Tell me what angle each motor is at right now, and I can tell you exactly where the gripper is in space and which way it is pointing."

**Minimum Sufficient Math**:

1. **Homogeneous Transform Matrix** (unifies rotation + translation into a single matrix product):

$$
T = \begin{bmatrix} R & p \\ \mathbf{0} & 1 \end{bmatrix} \in SE(3)
$$

$R \in SO(3)$ is the $3 \times 3$ rotation matrix (orientation of the child frame relative to the parent); $p \in \mathbb{R}^3$ is the translation vector (position of the child origin in the parent frame). **Why $4 \times 4$**: consecutive frame transforms chain via pure matrix multiplication, avoiding the awkward `R·x + p` mix of multiply and add.

2. **Single-Link DH Transform** (Standard DH, composition of four basic transforms):

$$
^{i-1}T_i = \text{Rot}_Z(\theta_i) \cdot \text{Trans}_Z(d_i) \cdot \text{Trans}_X(a_i) \cdot \text{Rot}_X(\alpha_i)
$$

**Physical meaning**: rotate about $Z_{i-1}$ by the joint angle $\theta_i$, translate along $Z_{i-1}$ by $d_i$, translate along the new $X_i$ by the link length $a_i$, then rotate about $X_i$ by the link twist $\alpha_i$.

3. **Full Forward Kinematics** (right-multiply from base to tip):

$$
^0T_n = {^0T_1} \cdot {^1T_2} \cdots {^{n-1}T_n}
$$

**Physical meaning**: accumulate each joint's local transform like relay runners handing off a baton. The product gives the gripper pose in the base frame. Order **cannot be reversed** — each $T$ is relative to the current moving frame, so it must be right-multiplied.

<details>
<summary>Deep dive: Full expansion and geometric derivation of the Standard DH matrix</summary>

Full $4 \times 4$ form of the Standard DH transform between adjacent links:

$$
^{i-1}T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

Per-entry meaning:
- $\cos\theta_i / \sin\theta_i$: horizontal projection of the rotation about Z
- $\cos\alpha_i / \sin\alpha_i$: Y/Z tilt after rotating about X (the link twist)
- $a_i \cos\theta_i / a_i \sin\theta_i$: link length $a_i$ projected onto the parent X/Y axes after rotation by $\theta_i$
- $d_i$: pure linear translation along the parent Z axis

**Key differences between Standard and Modified DH**:
- **Standard DH** (original Denavit): frame $i$ attaches to the **distal end** of link $i$; uses $a_i, \alpha_i$ between axis $i$ and axis $i+1$; multiplication order is `Rot_Z → Trans_Z → Trans_X → Rot_X`
- **Modified DH** (Craig's version): frame $i$ attaches to the **proximal end** of link $i$; uses $a_{i-1}, \alpha_{i-1}$ between axis $i-1$ and axis $i$; multiplication order is `Rot_X → Trans_X → Rot_Z → Trans_Z`

**Industry choice**: textbooks commonly use Standard DH; Modified DH is more robust for tree topologies, closed chains, and parallel adjacent axes. Modern ROS kinematics libraries (Orocos KDL, Pinocchio internals) tend toward Modified DH, or skip DH entirely in favor of Product of Exponentials (screw theory).

</details>

**Common APIs** (industry toolchain):

| Layer | Package | Example signature |
|-------|---------|-------------------|
| ROS 2 base | tf2 | `buffer.lookup_transform(target, source, time) → TransformStamped` |
| Planning hub | MoveIt | `RobotState::getGlobalLinkTransform(link_name) → Eigen::Isometry3d` |
| High-speed solver | Pinocchio | `pinocchio::forwardKinematics(model, data, q)` (MPC workhorse) |
| Simulator-built-in | MuJoCo / PyBullet | `mj_forward(model, data)` / `p.getLinkState(robot, link_idx)` |

## Intuition

**Analogy: relay race / stacking blocks**. Base is the start, each joint is a runner, and $^{i-1}T_i$ records "where and which way the next runner stands relative to the previous one." Multiply the batons' offsets together and you get the final runner's (gripper's) absolute pose relative to the start.

**Visual metaphor: engineer measuring DH with a tape**:
- **$a$** (link length): tape-measure the common perpendicular between two adjacent Z axes
- **$\alpha$** (link twist): sighting along that perpendicular, the angle between the two Z axes
- **$d$** (link offset): slide along the previous Z axis until you hit the common perpendicular's endpoint
- **$\theta$** (joint angle): the only moving quantity — how far this axis has rotated right now

**Simulator observation**: in Isaac Sim / Gazebo / MuJoCo, feed in a specific $q$ → read the simulator's ground-truth end-effector pose → compare against your own $^0T_n$. Position error < 0.1 mm and orientation error < 0.01 rad counts as passing. In rviz2, set Fixed Frame to `base_link`, enable the TF plugin with Show Axes, and eyeball whether your computed axes line up with the rendered ones.

## Implementation Link

**Three representative engineering scenarios**:

1. **ROS 2 real-time TF broadcast**: every control tick, `robot_state_publisher` reads joint angles from `/joint_states`, runs FK internally, and broadcasts each link's transform on the TF tree. Other nodes query arbitrary frame-to-frame relations with `tf2_buffer.lookup_transform`.

2. **Model Predictive Control**: at 1 kHz each iteration must compute $N$ future end-effector poses from the current $q$. FK runs $N$ times and cannot allocate on the heap. Standard practice: pre-derive $^0T_n$ as a closed-form analytical expression with SymPy offline, then emit unrolled C++ assignments + SIMD for the hot path.

3. **RL reward shaping**: the policy outputs joint torques; each env step needs end-effector pose against the target to compute reward. Use the simulator's built-in FK (MuJoCo's `mj_forward`) — faster and more reliable than rolling your own.

**Code skeleton** (C++, ROS 2 + KDL):

```cpp
// Parse URDF → build KDL::Tree → pick chain → run FK
KDL::Tree tree;
kdl_parser::treeFromUrdfModel(urdf_model, tree);
KDL::Chain chain;
tree.getChain("base_link", "tool0", chain);

KDL::ChainFkSolverPos_recursive fk_solver(chain);
KDL::JntArray q(chain.getNrOfJoints());
q(0) = 0.1; q(1) = -0.5; /* ... */

KDL::Frame end_effector_pose;  // Output: end-effector 4x4
fk_solver.JntToCart(q, end_effector_pose);
```

<details>
<summary>Deep dive: complete runnable Python FK implementation (copy-paste ready)</summary>

```python
import numpy as np

def dh_transform_standard(a, alpha, d, theta):
    """Standard DH single-link transform: Rot_Z(theta) * Trans_Z(d) * Trans_X(a) * Rot_X(alpha)"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d    ],
        [0,   0,        0,       1    ],
    ])


def forward_kinematics(dh_table, joint_values, tool_transform=None):
    """
    dh_table: list of (a, alpha, d, theta_offset) per link
    joint_values: theta for rotary joints; swap d for prismatic joints (adjust accordingly)
    tool_transform: optional, TCP transform relative to the last link
    """
    T = np.eye(4)
    for (a, alpha, d, theta_offset), q in zip(dh_table, joint_values):
        theta = theta_offset + q  # Add joint zero offset
        T = T @ dh_transform_standard(a, alpha, d, theta)
    if tool_transform is not None:
        T = T @ tool_transform  # Right-multiply the tool transform
    return T


# Example: approximate UR5 Standard DH table
ur5_dh = [
    (0,      np.pi/2,  0.089,  0),
    (-0.425, 0,        0,      0),
    (-0.392, 0,        0,      0),
    (0,      np.pi/2,  0.109,  0),
    (0,      -np.pi/2, 0.095,  0),
    (0,      0,        0.082,  0),
]
q = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
T_end = forward_kinematics(ur5_dh, q)
print("End-effector position:", T_end[:3, 3])
print("Rotation matrix:\n", T_end[:3, :3])
```

If Pinocchio is installed, you would normally use it directly:

```python
import pinocchio as pin
model = pin.buildModelFromUrdf("ur5.urdf")
data = model.createData()
pin.forwardKinematics(model, data, q)
print(data.oMi[model.getFrameId("tool0")])  # End-effector SE3
```

Pinocchio uses spatial algebra + recursive Newton-Euler internally, an order of magnitude faster than hand-rolled matrix products.

</details>

## Common Misconceptions

1. **Mixing Standard and Modified DH** — the two conventions differ in frame attachment and the ordering of the four basic transforms. Mixing causes total divergence. **Avoid**: verify the manufacturer spec before implementing and comment the DH version in the source.
2. **Ignoring joint zero offsets** — the URDF zero position is not the same as the DH table zero, and the physical robot's mechanical zero may add yet another offset. **Avoid**: always apply `theta_math = theta_motor + zero_offset_i`; during debugging, set all joints to 0 and confirm your FK matches the hardware's initial pose.
3. **Forgetting the Tool Frame (TCP)** — stopping at the flange and ignoring the gripper/welder offset means the software thinks the target is reached while the real tool tip is tens of centimeters off. **Avoid**: always write $^0T_{\text{tool}} = {^0T_6} \cdot {^6T_{\text{tool}}}$; the tool transform must be right-multiplied.
4. **Accumulated floating-point error in rotation matrices** — chained multiplications let $R \cdot R^T \ne I$, distorting orientation. **Avoid**: periodically renormalize (SVD or Gram-Schmidt), or keep orientation in quaternions internally and convert back to a matrix only at the output.

## Situational Questions

<details>
<summary>Q1 (easy): You receive a new 6-DoF robot URDF. How do you quickly and rigorously validate your hand-written C++ FK?</summary>

**Complete reasoning chain**:

1. **Load URDF, launch ROS 2**: start `robot_state_publisher` + `rviz2`, confirm the TF tree is broadcasting.
2. **Design test cases**: do not only test zero-config. At minimum:
   - (A) Zero pose
   - (B) Joint limits (±90° or each joint's limit)
   - (C) 1000 Monte Carlo random $q$
3. **Fetch ground truth**: `tf2_buffer.lookup_transform("base_link", "tool0", rclcpp::Time(0))`.
4. **Matrix comparison**: subtract your C++ FK's 4×4 from the matrix converted from the returned `TransformStamped`.
   - Position error < 1e-6 m and orientation error < 1e-6 rad → pass
   - If not, suspect first that URDF `<origin>` offsets disagree with your DH zero convention

**What the interviewer wants to hear**: a software-engineering mindset that automates random sampling against ground truth, and awareness that URDF's freely placed frames and DH's strict common-perpendicular rules are the chief source of inconsistency.

</details>

<details>
<summary>Q2 (medium): After installing a non-symmetric gripper, the real-world end-effector pose lags FK's prediction by 5 cm plus a strange rotation. How do you debug logically?</summary>

**Complete reasoning chain**:

1. **Isolate variables**: the factory FK was already validated, so the base DH is fine. Command all joints to zero and rotate only the last axis; observe whether the error follows the rotation or stays fixed in space.
2. **Diagnose: wrong TCP**: error follows rotation → the tool frame offset is wrong.
3. **Right-multiply the tool transform**: build $^6T_{\text{tool}}$ (with xyz translation and the non-symmetric RPY); change the code to $^0T_{\text{tool}} = {^0T_6} \cdot {^6T_{\text{tool}}}$. It **must** be right-multiplied because the tool rides on the moving frame of joint 6.
4. **Laser-tracker calibration**: drive the robot through several poses and least-squares-solve for the precise $^6T_{\text{tool}}$ to absorb assembly tolerances.

**What the interviewer wants to hear**: disciplined variable isolation, understanding that the tool frame right-multiplies at the end of the FK chain, and hands-on experience moving from "theoretical CAD" to "calibrated reality" via laser-tracker fits.

</details>

<details>
<summary>Q3 (medium-hard): Your RL policy outputs joint torques. In sim everything is smooth, but on hardware the end-effector trajectory oscillates and misses the goal. How do you diagnose and fix?</summary>

**Complete reasoning chain**:

1. **Set up in-sim visualization**: in each env step callback, read $q$ from the simulator and run FK to get the end-effector XYZ.
2. **Publish Rviz markers**: gather the trajectory points as `visualization_msgs/Marker` type `LINE_STRIP`, so you can eyeball whether the sim-side trajectory is actually smooth.
3. **Diagnose reality gap**: the RL policy has overfit to unmodeled dynamics — the real robot has harmonic-drive backlash, joint elasticity, and nonlinear friction that a rigid-body simulator hides.
4. **Fix on two fronts**:
   - **Domain Randomization**: during training, perturb friction, inertia, and damping by ±20%.
   - **System Identification**: run sweep signals on hardware to identify dynamics parameters and feed them back into the simulator to close the gap.

**What the interviewer wants to hear**: the full torque → sim integration → $q$ → FK → end-effector XYZ chain; recognition that the core sim-to-real gap lies in unmodeled dynamics; and standard responses (Domain Randomization + System ID).

</details>

<details>
<summary>Q4 (hard): Upgrading from 6-DoF to a redundant 7-DoF arm. FK + Jacobian must finish inside a 1 ms control loop at 1 kHz. Naive matrix products blow the CPU budget. How do you compress them?</summary>

**Complete reasoning chain**:

1. **Extend DH to 7 joints**: add $(\alpha_7, a_7, d_7, \theta_7)$; note that 7-DoF redundancy produces infinitely many IK solutions, but FK is still seven 4×4 matrices multiplied.
2. **Symbolic pre-computation**: on-line `for` loops waste cycles multiplying and adding zeros. Offline, derive $^0T_7$ and $J$ as closed-form analytical expressions with SymPy or MATLAB.
3. **Aggressive C++ optimization**:
   - Emit the analytical expression as flattened assignments
   - Cache $\cos\theta_i$ / $\sin\theta_i$ as `c1, s1, ...` and reuse them
   - Use SIMD (AVX/SSE) to compute Jacobian entries in parallel
   - **Never** call `new` / `malloc` / grow a `std::vector`
4. **Null-space avoidance**: $\Delta\Theta = J^+ v + (I - J^+ J) \nabla H$ — the primary task tracks the end-effector trajectory, the secondary task uses $\nabla H$ to push the elbow away from obstacles, all within 1 ms.

**What the interviewer wants to hear**: an unflinching pursuit of low-level performance (allocations, symbolic simplification, trig caching, SIMD) and the insight that the point of 7-DoF redundancy is null-space optimization — exploiting the extra freedom to do useful work without disturbing the end-effector path.

</details>

## Interview Angles

1. **DH version trap (Standard vs Modified)** — shows you have shipped modern multi-joint arms rather than recited textbook chapters. **Bring out with**: "Before any FK implementation I read the vendor's manual to confirm Standard vs Modified DH, because the frame attachment dictates the entire downstream TF tree — pick wrong and everything diverges."

2. **Aligning theory with the simulator (URDF vs DH)** — marries pure math with ROS 2 engineering reality, proving you have walked this trap. **Bring out with**: "After writing FK I don't trust the math alone — I validate the 4×4 against `tf2_echo` or the simulator ground truth across a thousand random configurations to be sure URDF's `<origin>` offsets match my DH zero convention."

3. **Low-level real-time performance** — the divide between "pure ML people" and engineers who ship to hardware. **Bring out with**: "For a 1 ms control window I refuse to run matrix products in a C++ loop; I pre-derive closed-form analytical expressions, unroll them into assignments, avoid all heap allocation, and vectorize with SIMD."

4. **Singularity anticipation (Jacobian rank deficiency)** — extends from FK into the sensitivity of the system near its boundaries. **Bring out with**: "While computing FK I monitor the Jacobian's condition number in lock-step; once it spikes near a singular configuration, I switch to damped least-squares or null-space optimization before the joint-velocity commands explode."

## Further Reading

- ***Embodied AI Algorithm Engineer Interview Questions*, Ch1.2 Kinematic Modeling, Ch3 Singularity Analysis** — drill the DH physical meanings, multi-solution / no-solution / singular configurations you'll be asked about
- ***ROS 2 Robot Development*, Ch6 URDF + Xacro + TF tree** — turn pen-and-paper matrices into a maintainable ROS 2 model
- **Lynch & Park, *Modern Robotics*, Ch4 (Product of Exponentials)** — the modern alternative to DH that is singularity-free and not tied to arbitrary axis conventions; MIT open courseware with videos and notebooks
- **Pinocchio examples** — read the C++ spatial-algebra source to see how to drive FK, Jacobian, and dynamics to the limit
- **Paper: *Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: a Survey*** — panoramic view of the reality gap and domain randomization
- **URDF / SDF / MJCF specifications** — sim-to-real consistency issues almost always trace back to subtle differences across these formats
