---
title: "Trajectory Optimization (Time, Energy, Smoothness)"
prerequisites: ["10-basic-path-planning"]
estimated_time: 45
difficulty: 3
tags: ["trajectory", "optimization", "smoothness", "b-spline", "topp"]
sidebar_position: 11
---

# Trajectory Optimization (Time, Energy, Smoothness)

## You Will Learn

- Distinguish **path** (pure geometry) from **trajectory** (geometry + time law + physical constraints), and explain why a path planner's output is only half the job
- Choose the right objective — minimum time, minimum energy, or minimum jerk — for a given industrial task, and articulate the Pareto trade-off between them
- Decide when to use cubic/quintic polynomials, trapezoidal velocity profiles, S-curves, or B-spline parameterizations, and diagnose vibration or torque-limit violations caused by picking the wrong one

## Core Concepts

**Precise Definition**: **Trajectory optimization** takes a geometric path (a sequence of waypoints or a continuous curve in joint or Cartesian space) and assigns a time law $s(t)$ so that the resulting motion $q(t)$ satisfies all kinematic and dynamic constraints (velocity, acceleration, jerk, torque limits) while minimizing a chosen cost — execution time, energy consumption, or motion smoothness. It is the bridge that turns a collision-free path into a physically executable motion profile.

**Location in the Sense → Plan → Control Loop**:
- **Input**: waypoints or a continuous path from the path planner + dynamic limits (joint velocity/acceleration/torque bounds from the robot spec sheet)
- **Output**: fully parameterized time-series $q(t), \dot{q}(t), \ddot{q}(t)$ (and optionally $\tau(t)$) that the controller can track
- **Downstream consumers**: feedforward controller (uses $\ddot{q}(t)$ for inverse-dynamics feedforward), PID/computed-torque controller (uses $q_d(t), \dot{q}_d(t)$ as the reference signal), MPC (uses the trajectory as a warm start or reference horizon)

**One-line version**: "Trajectory optimization injects time and physical law into a geometric path — it decides where to floor the accelerator, where to brake, and how to keep the ride smooth."

**Minimum Sufficient Math**:

1. **Cubic polynomial interpolation** (satisfies position + velocity boundary conditions at two knots):

$$
q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3
$$

**Physical meaning**: four coefficients absorb four boundary conditions ($q_0, \dot{q}_0, q_f, \dot{q}_f$). Guarantees continuous position and velocity but acceleration jumps at segment boundaries — fine for slow pick-and-place, dangerous for high-speed or force-sensitive tasks.

2. **Quintic polynomial** (adds acceleration boundary conditions):

$$
q(t) = \sum_{k=0}^{5} a_k t^k
$$

**Physical meaning**: six coefficients handle six boundary conditions ($q, \dot{q}, \ddot{q}$ at both ends). Acceleration is now continuous across segments, which means no instantaneous force spikes at the joints — critical for high-payload or high-speed arms.

3. **Minimum-jerk cost** (smoothness objective):

$$
J = \int_0^T \left\| \dddot{q}(t) \right\|^2 dt
$$

**Physical meaning**: jerk is the rate of change of acceleration, i.e., how abruptly force changes. Minimizing jerk suppresses mechanical vibration and resonance. Human arm movements naturally approximate minimum-jerk profiles — this is why minimum-jerk trajectories feel "natural" and are widely used in human-robot interaction.

4. **Time-optimal path parameterization** (TOPP — phase-plane method):

Given a fixed geometric path $q(s)$ where $s \in [0, 1]$, the trajectory becomes $q(s(t))$ and the optimization reduces to finding $\dot{s}(t)$ that minimizes total time $T$ subject to:

$$
\tau_{\min} \leq M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) \leq \tau_{\max}
$$

**Physical meaning**: the torque inequality, expressed in the phase plane $(s, \dot{s})$, carves out a feasible velocity region. The time-optimal solution rides the upper boundary of this region — accelerate as hard as the weakest joint allows, then decelerate just in time. This is the generalized "bang-bang" control for multi-joint robots.

<details>
<summary>Deep dive: polynomial coefficient derivation and TOPP-RA formulation</summary>

### Cubic polynomial coefficient solution

Given boundary conditions $q(0) = q_0,\ \dot{q}(0) = v_0,\ q(T) = q_f,\ \dot{q}(T) = v_f$:

$$
\begin{bmatrix} a_0 \\ a_1 \\ a_2 \\ a_3 \end{bmatrix}
= \begin{bmatrix}
1 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 \\
-3/T^2 & -2/T & 3/T^2 & -1/T \\
2/T^3 & 1/T^2 & -2/T^3 & 1/T^2
\end{bmatrix}
\begin{bmatrix} q_0 \\ v_0 \\ q_f \\ v_f \end{bmatrix}
$$

Each row is obtained by substituting the boundary conditions into $q(t)$ and $\dot{q}(t)$ and solving the resulting $4 \times 4$ linear system. The matrix is always invertible for $T > 0$.

### Quintic polynomial

The same procedure with 6 boundary conditions ($q, \dot{q}, \ddot{q}$ at $t = 0$ and $t = T$) yields a $6 \times 6$ linear system. The extra two degrees of freedom buy continuous acceleration, eliminating force discontinuities at segment joints.

### TOPP-RA (Time-Optimal Path Parameterization via Reachability Analysis)

Given a fixed path $q(s)$, substituting $\dot{q} = q'(s)\dot{s}$ and $\ddot{q} = q'(s)\ddot{s} + q''(s)\dot{s}^2$ into the dynamics equation yields:

$$
\tau = M(q(s))\bigl[q'(s)\ddot{s} + q''(s)\dot{s}^2\bigr] + C\bigl(q(s), q'(s)\dot{s}\bigr)q'(s)\dot{s} + g(q(s))
$$

Rearranging:

$$
\tau = \mathbf{a}(s)\ddot{s} + \mathbf{b}(s)\dot{s}^2 + \mathbf{c}(s)
$$

where $\mathbf{a}, \mathbf{b}, \mathbf{c}$ are path-dependent vectors. For each $s$, the torque limits $\tau_{\min} \leq \tau \leq \tau_{\max}$ define a linear constraint on $(\ddot{s}, \dot{s}^2)$. TOPP-RA discretizes the path into $N$ grid points and solves a sequence of linear programs to find the maximum feasible $\dot{s}$ at each grid point.

The algorithm runs in $O(N \cdot n_{\text{dof}})$ — linear in both path resolution and number of joints — making it real-time capable at 1 kHz for 7-DoF arms.

**Key insight**: TOPP-RA decouples the problem. The path planner handles geometry and collision avoidance; TOPP-RA handles time allocation and dynamic feasibility. This separation makes both problems simpler.

</details>

<details>
<summary>Deep dive: B-spline vs polynomial — why local control matters</summary>

### The Runge phenomenon trap

High-degree single-polynomial interpolation through $n$ waypoints (degree $n-1$) produces wild oscillations between the knots — this is Runge's phenomenon. The oscillations grow exponentially near the endpoints.

### B-spline solution

A B-spline of order $k$ with $n$ control points uses $n + k$ knots and achieves $C^{k-2}$ continuity while each control point only influences $k$ consecutive segments. This **local control** property means:

- Modifying one waypoint does not ripple through the entire trajectory
- Numerical conditioning stays bounded regardless of trajectory length
- Adding a waypoint (knot insertion) does not change the existing curve

### Practical choice guide

| Criterion | Cubic/Quintic poly | B-spline | Trapezoidal | S-curve (7-segment) |
|-----------|-------------------|----------|-------------|---------------------|
| Continuity | $C^2$ / $C^4$ | $C^{k-2}$ (tunable) | $C^0$ accel | $C^1$ accel |
| Local control | No | Yes | N/A (PTP) | N/A (PTP) |
| Jerk continuity | Quintic: no; higher: yes | Order ≥ 5: yes | No (infinite jerk) | Yes |
| Computational cost | $O(1)$ per eval | $O(k)$ per eval | $O(1)$ | $O(1)$ |
| Best for | Short 2-point segments | Long multi-waypoint paths | Simple PTP with trapezoidal speed | PTP needing smooth acceleration |

</details>

## Intuition

**Analogy: driving a car on a mapped route**. The path planner draws the route on the map (turn left here, go straight there). Trajectory optimization is the driving style — where to floor the gas, where to brake, and how to ease into turns so the passengers do not get carsick. Minimum time = race driver; minimum energy = hypermiler; minimum jerk = chauffeur.

**Trapezoidal velocity profile — the elevator ride**. An elevator accelerates at constant rate, cruises at max speed, then decelerates at constant rate. Simple and effective, but the jerk (acceleration jump) at each phase boundary is infinite — this is why cheap elevators give you that stomach-drop feeling. The S-curve (7-segment) profile rounds off those corners with linear jerk ramps, eliminating the jolt.

**Simulator observation**: in MuJoCo or Isaac Sim, command a 6-DoF arm to move between two poses using (1) a trapezoidal velocity profile and (2) a minimum-jerk profile. Plot the joint torques. The trapezoidal profile shows sharp torque spikes at phase transitions; the minimum-jerk profile shows smooth, bell-shaped torque curves. Now crank up the speed until the trapezoidal profile exceeds the motor's torque limit while the minimum-jerk profile still fits — that is the practical payoff of smoothness.

## Implementation Link

**Three representative engineering scenarios**:

1. **Industrial pick-and-place (time-optimal)**: the robot must move a part from bin A to bin B as fast as possible without exceeding joint torques. Use TOPP-RA on the collision-free path from RRT to find the fastest feasible time parameterization.

2. **Welding seam tracking (smoothness-critical)**: the end-effector must follow a Cartesian straight line at constant tool speed with minimal vibration. Use a quintic spline in Cartesian space with minimum-jerk weighting. Monitor the Jacobian condition number along the path — if it spikes near a singularity, the joint velocities will explode even if the Cartesian trajectory is smooth.

3. **Mobile robot navigation (Nav2 / ROS 2)**: the DWA local planner generates velocity commands. If the robot turns too aggressively, reduce `max_vel_theta` and `max_accel_theta` in the DWA parameter file, or increase the smoothness cost weight in the TEB local planner.

**Code skeleton** (Python, TOPP-RA):

```python
import toppra as ta
import numpy as np

def time_optimal_parameterize(waypoints, joint_vel_limits, joint_acc_limits, dt=0.01):
    """
    Takes waypoints from a path planner and returns a time-optimal trajectory.
    waypoints: (N, n_dof) array of joint configurations
    Returns: (T, n_dof) arrays of q(t), qd(t), qdd(t)
    """
    # 1. Fit a cubic spline through the waypoints (geometric path)
    ss = np.linspace(0, 1, len(waypoints))
    path = ta.SplineInterpolator(ss, waypoints)

    # 2. Define kinematic constraints
    vel_constraint = ta.constraint.JointVelocityConstraint(joint_vel_limits)
    acc_constraint = ta.constraint.JointAccelerationConstraint(joint_acc_limits)

    # 3. Solve TOPP-RA
    instance = ta.algorithm.TOPPRA(
        [vel_constraint, acc_constraint], path
    )
    trajectory = instance.compute_trajectory(0, 0)  # zero start/end velocity

    # 4. Sample the result
    ts = np.arange(0, trajectory.duration, dt)
    qs = trajectory(ts)           # joint positions
    qds = trajectory(ts, 1)      # joint velocities
    qdds = trajectory(ts, 2)     # joint accelerations
    return ts, qs, qds, qdds
```

```cpp
// C++ skeleton: minimum-jerk trajectory for a single joint segment
struct QuinticSegment {
    double coeffs[6];  // a0..a5

    // Solve for coeffs given boundary conditions at t=0 and t=T
    void solve(double q0, double v0, double a0,
               double qf, double vf, double af, double T);

    double position(double t) const;
    double velocity(double t) const;
    double acceleration(double t) const;
};

// Multi-joint trajectory: one QuinticSegment per joint per segment
class MinJerkTrajectory {
    std::vector<std::vector<QuinticSegment>> segments_;  // [segment][joint]
public:
    void plan(const std::vector<Eigen::VectorXd>& waypoints,
              const Eigen::VectorXd& durations);
    Eigen::VectorXd eval(double t, int derivative_order) const;
};
```

<details>
<summary>Deep dive: complete Python implementation — quintic polynomial + minimum-jerk multi-segment</summary>

```python
import numpy as np
from dataclasses import dataclass

@dataclass
class QuinticCoeffs:
    a: np.ndarray  # shape (6,) — coefficients a0..a5

def solve_quintic(q0, v0, a0, qf, vf, af, T):
    """Solve quintic polynomial coefficients for one joint, one segment."""
    T2, T3, T4, T5 = T**2, T**3, T**4, T**5
    # Boundary conditions → 6x6 linear system
    A = np.array([
        [1, 0,   0,     0,      0,      0     ],
        [0, 1,   0,     0,      0,      0     ],
        [0, 0,   2,     0,      0,      0     ],
        [1, T,   T2,    T3,     T4,     T5    ],
        [0, 1,   2*T,   3*T2,   4*T3,   5*T4  ],
        [0, 0,   2,     6*T,    12*T2,  20*T3 ],
    ])
    b = np.array([q0, v0, a0, qf, vf, af])
    return QuinticCoeffs(a=np.linalg.solve(A, b))

def eval_quintic(c: QuinticCoeffs, t, deriv=0):
    """Evaluate quintic or its derivative at time t."""
    a = c.a
    if deriv == 0:
        return a[0] + a[1]*t + a[2]*t**2 + a[3]*t**3 + a[4]*t**4 + a[5]*t**5
    elif deriv == 1:
        return a[1] + 2*a[2]*t + 3*a[3]*t**2 + 4*a[4]*t**3 + 5*a[5]*t**4
    elif deriv == 2:
        return 2*a[2] + 6*a[3]*t + 12*a[4]*t**2 + 20*a[5]*t**3
    elif deriv == 3:
        return 6*a[3] + 24*a[4]*t + 60*a[5]*t**2
    raise ValueError(f"deriv={deriv} not supported")


def plan_min_jerk_trajectory(waypoints, segment_durations):
    """
    Plan a minimum-jerk (quintic) trajectory through waypoints.

    waypoints: list of np.ndarray, each shape (n_dof,)
    segment_durations: list of float, len = len(waypoints) - 1

    Returns: list of list of QuinticCoeffs — [segment_idx][joint_idx]
    """
    n_segments = len(waypoints) - 1
    n_dof = len(waypoints[0])
    all_coeffs = []

    for seg in range(n_segments):
        T = segment_durations[seg]
        seg_coeffs = []
        for j in range(n_dof):
            q0 = waypoints[seg][j]
            qf = waypoints[seg + 1][j]
            # Zero velocity and acceleration at endpoints (rest-to-rest)
            c = solve_quintic(q0, 0.0, 0.0, qf, 0.0, 0.0, T)
            seg_coeffs.append(c)
        all_coeffs.append(seg_coeffs)

    return all_coeffs


def sample_trajectory(all_coeffs, segment_durations, dt=0.001):
    """Sample the full trajectory at fixed dt."""
    positions, velocities, accelerations, jerks = [], [], [], []
    times = []
    t_offset = 0.0

    for seg_idx, (seg_coeffs, T) in enumerate(zip(all_coeffs, segment_durations)):
        ts_local = np.arange(0, T, dt)
        for t in ts_local:
            q = np.array([eval_quintic(c, t, 0) for c in seg_coeffs])
            qd = np.array([eval_quintic(c, t, 1) for c in seg_coeffs])
            qdd = np.array([eval_quintic(c, t, 2) for c in seg_coeffs])
            qddd = np.array([eval_quintic(c, t, 3) for c in seg_coeffs])
            positions.append(q)
            velocities.append(qd)
            accelerations.append(qdd)
            jerks.append(qddd)
            times.append(t_offset + t)
        t_offset += T

    return (np.array(times), np.array(positions),
            np.array(velocities), np.array(accelerations), np.array(jerks))


# --- Example usage ---
if __name__ == "__main__":
    waypoints = [
        np.array([0.0, 0.0, 0.0]),       # start (3-DoF)
        np.array([1.0, -0.5, 0.3]),       # via point
        np.array([1.5, 0.2, -0.1]),       # goal
    ]
    durations = [2.0, 1.5]  # seconds per segment
    coeffs = plan_min_jerk_trajectory(waypoints, durations)
    ts, qs, qds, qdds, qddds = sample_trajectory(coeffs, durations)

    # Verify: zero velocity at start/end
    print(f"Start vel: {qds[0]}")   # should be ~[0, 0, 0]
    print(f"End vel:   {qds[-1]}")  # should be ~[0, 0, 0]
    print(f"Max jerk:  {np.max(np.abs(qddds), axis=0)}")
```

</details>

## Common Misconceptions

1. **"Minimizing time and minimizing energy are the same thing"** — they are fundamentally opposed. Time-optimal control pushes actuators to their limits (bang-bang), which means maximum current draw and heat. Energy-optimal control uses gentle, continuous accelerations, which takes longer. In practice, you are always on a Pareto frontier trading one for the other — the right balance depends on the task (cycle time vs. motor lifespan).

2. **"Higher-degree polynomials are always smoother"** — beyond degree 7–9, single-polynomial interpolation through many waypoints produces Runge oscillations: the curve overshoots wildly between knots. The fix is not a higher degree but **piecewise** methods (cubic/quintic splines) or **B-splines** with local control. Degree should match the number of boundary conditions you actually have.

3. **"If the Cartesian path is smooth, the joint trajectory is smooth"** — near kinematic singularities, even a perfectly smooth Cartesian straight line demands infinite joint velocities (the Jacobian loses rank, so $\dot{q} = J^{-1}v$ blows up). Always check the Jacobian condition number along the path, or plan in joint space and verify the Cartesian result.

4. **"Velocity and acceleration limits are enough — torque limits do not matter"** — kinematic limits ignore the robot's dynamics. A trajectory that respects velocity and acceleration bounds can still violate torque limits when the arm is fully extended (high inertia configuration) or carrying a heavy payload. TOPP-RA's strength is that it incorporates full inverse-dynamics constraints, not just kinematic ones.

## Situational Questions

<details>
<summary>Q1: A pick-and-place station needs to move parts as fast as possible. You have the collision-free path from RRT*. The robot occasionally faults on "torque exceeded." How do you fix it?</summary>

**Complete reasoning chain**:

1. **Diagnose**: the path planner output is geometrically valid, but the time parameterization (probably trapezoidal or naive constant-speed) does not respect the robot's dynamic limits. The torque fault means the commanded acceleration exceeds what the motors can deliver in the current configuration (especially near full arm extension where inertia is highest).

2. **Solution — TOPP-RA**:
   - Feed the RRT* path into TOPP-RA with **torque** constraints (not just velocity/acceleration).
   - TOPP-RA will solve for the maximum feasible speed profile in the phase plane $(s, \dot{s})$, automatically slowing down through high-inertia configurations.
   - The result is provably time-optimal given the torque limits.

3. **Implementation**:
   - Use the `toppra` Python package: define `JointTorqueConstraint` with the robot's URDF dynamics model.
   - Verify by plotting the resulting $\tau(t)$ against the motor spec sheet.

4. **Pitfall to avoid**: do not just reduce the global speed scaling factor. That makes the entire trajectory slower, including the easy straight-line segments. TOPP-RA slows down only where needed and pushes the speed everywhere else.

**Conclusion**: TOPP-RA with full dynamics constraints is the standard tool for time-optimal industrial trajectories.

</details>

<details>
<summary>Q2: A welding robot tracks a Cartesian straight-line seam, but the weld bead quality is poor — the torch vibrates at the start and end of the seam. What is happening and how do you fix it?</summary>

**Complete reasoning chain**:

1. **Diagnose vibration source**: the velocity profile likely has discontinuous acceleration (e.g., trapezoidal profile), causing infinite jerk at phase transitions. The jerk excites structural resonances in the robot arm, which shows up as torch vibration.

2. **Check for singularity**: if the seam passes near a kinematic singularity, even a smooth Cartesian trajectory produces wild joint velocities. Compute the Jacobian condition number along the path; if it exceeds ~100, reroute or add a via-point to dodge the singular configuration.

3. **Fix the time parameterization**:
   - Switch from trapezoidal to **S-curve (7-segment)** or **minimum-jerk quintic** for the velocity profile.
   - Alternatively, parameterize the path with a B-spline of order ≥ 5 (guarantees $C^3$ / jerk continuity).

4. **Tune feedforward**: even with a smooth reference, the controller must have adequate feedforward. Provide $\ddot{q}_d(t)$ to the computed-torque or inverse-dynamics feedforward term — this prevents the controller from lagging behind the reference and generating tracking-error oscillations.

5. **Pitfall to avoid**: do not blindly increase controller gains (PID) to suppress vibration. High gains amplify sensor noise and can make the oscillation worse. Fix the trajectory first, then tune gains.

**Conclusion**: smooth trajectory (minimum jerk) + singularity check + feedforward control = vibration-free welding.

</details>

<details>
<summary>Q3: Your Nav2-based mobile robot overshoots sharp 90-degree corridor turns, bumping into walls. The DWA local planner is configured with default parameters. What do you adjust?</summary>

**Complete reasoning chain**:

1. **Diagnose**: DWA (Dynamic Window Approach) samples velocity commands $(v, \omega)$ and scores them. Default parameters often set `goal_heading` cost weight too high relative to `path_distance` and `obstacle_distance`, causing the robot to turn aggressively toward the goal heading.

2. **Parameter tuning**:
   - Reduce `max_vel_theta` and `max_accel_theta` to limit how fast the robot can rotate.
   - Lower the `goal_heading` weight so the planner does not sacrifice path adherence for aggressive turning.
   - Increase `obstacle_distance` weight to maintain clearance.

3. **Consider switching to TEB**: the Timed-Elastic-Band planner optimizes a full trajectory (not just single-step velocity commands) and inherently handles smoothness. It has explicit parameters for `weight_acc_lim_theta` (angular acceleration penalty) and `weight_kinematics_nh` (non-holonomic constraint weight). TEB typically produces smoother corridor navigation than DWA.

4. **Trajectory smoothness at the global level**: if the global planner (e.g., NavFn or Theta*) produces a path with sharp right-angle kinks, smooth it with a B-spline or cubic-spline post-processing step before handing it to the local planner. The local planner should not have to compensate for a jagged global path.

5. **Pitfall to avoid**: do not just slow the robot down globally. That hurts throughput. Instead, apply curvature-dependent speed limiting: reduce speed proportionally to path curvature so the robot slows only around tight corners.

**Conclusion**: tune angular velocity/acceleration limits + adjust cost weights + consider TEB planner for inherently smoother trajectories.

</details>

<details>
<summary>Q4: You are deploying a drone that must fly through a set of waypoints as fast as possible while keeping the trajectory smooth enough for stable flight. What formulation do you use?</summary>

**Complete reasoning chain**:

1. **Formulation choice — minimum snap**: for quadrotors, thrust is proportional to the second derivative of position (acceleration), and the angular rates depend on the third and fourth derivatives (jerk and snap). Minimizing snap ($\int \|\ddddot{q}\|^2 dt$) produces trajectories that keep angular rates smooth, which is essential for stable flight.

2. **Mathematical setup**:
   - Represent each axis $(x, y, z)$ as a piecewise 7th-degree polynomial (8 coefficients → enough to match $q, \dot{q}, \ddot{q}, \dddot{q}$ at both ends of each segment).
   - Minimize $\int \|\ddddot{q}\|^2 dt$ subject to waypoint constraints.
   - This is a QP (quadratic program) — convex, globally solvable, and fast.

3. **Time allocation**: the segment durations $T_i$ are not given by the problem. Use an initial estimate based on distance/max-velocity, then iteratively refine: solve the QP for fixed $T_i$, check if any dynamic constraint is violated, adjust $T_i$ accordingly. Alternatively, use the TOPP approach to find the optimal time allocation post-hoc.

4. **Corridor constraints**: if the drone must stay within a safe flight corridor (polytope constraints), the problem becomes a QP with inequality constraints. The `fast_planner` and `ego_planner` ROS packages implement this efficiently.

5. **Pitfall to avoid**: do not use minimum-jerk for quadrotors. Jerk-optimal trajectories can still have discontinuous snap, which means the commanded angular velocity has jumps — the flight controller cannot track these, causing altitude drops or flips.

**Conclusion**: minimum-snap QP is the industry standard for aggressive quadrotor trajectory generation.

</details>

## Interview Angles

1. **Path vs Trajectory — the bridge to control** — this is the first thing to establish. Interviewers test whether you understand that a path planner's output is incomplete. Lead with: "A path is pure geometry — it tells you the route but not the speed. Trajectory optimization adds the time law and physical constraints, producing the $q(t)$ signal the controller actually tracks. Without it, the path is just a map with no driving instructions."

2. **Pareto trade-off between time, energy, and smoothness** — demonstrates systems-level thinking. Lead with: "Time-optimal trajectories push actuators to their limits, which burns energy and stresses hardware. Energy-optimal trajectories are slow. In practice, I formulate a weighted multi-objective cost and tune the weights based on the task — cycle time for manufacturing, energy for battery-powered mobile robots, smoothness for human-facing applications or delicate manipulation."

3. **B-spline local control vs polynomial global coupling** — shows you have dealt with real multi-waypoint paths. Lead with: "I prefer B-splines for long paths because they have local control — modifying one waypoint only affects nearby segments. A single high-degree polynomial through $n$ waypoints couples everything and risks Runge oscillation. For two-point motions, quintic polynomials are perfectly fine."

4. **Cartesian smoothness does not guarantee joint smoothness — the singularity trap** — catches many candidates off guard. Lead with: "A perfectly smooth Cartesian straight line can demand infinite joint velocities near a singularity because the Jacobian loses rank. I always monitor the condition number of $J$ along the planned path and either reroute, add via-points, or add a Jacobian condition number constraint to the optimizer."

5. **TOPP-RA for time-optimal industrial trajectories** — signals that you know the state-of-the-art tool, not just textbook bang-bang control. Lead with: "For time-optimal trajectories on real robots, I use TOPP-RA, which formulates the problem in the $(s, \dot{s})$ phase plane and incorporates full inverse-dynamics constraints — not just kinematic limits. It runs in linear time and is available as an open-source Python library."

## Further Reading

- **Pham & Pham, "TOPP-RA: A Fast Algorithm for Time-Optimal Path Parameterization" (IEEE T-RO 2018)** — the definitive reference for time-optimal trajectory generation with dynamics constraints; includes open-source Python implementation
- **Mellinger & Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors" (ICRA 2011)** — established the minimum-snap QP formulation now used in virtually every aggressive drone trajectory planner
- **Lynch & Park, *Modern Robotics*, Ch9 (Trajectory Generation)** — clear treatment of polynomial interpolation, trapezoidal profiles, and time scaling with physical constraints
- **`toppra` Python package (github.com/hungpham2511/toppra)** — production-quality TOPP-RA implementation; integrates with ROS 2 and supports torque, velocity, and acceleration constraints
- ***Embodied AI Interview Questions*, Ch5.1 + Ch5.3** — trajectory optimization problem sets with worked solutions; Ch5.4 covers MPC-based trajectory tracking
- **`fast_planner` / `ego_planner` (github.com/HKUST-Aerial-Robotics)** — real-time B-spline trajectory optimization for drones in cluttered environments; good reference for corridor constraints and gradient-based refinement
