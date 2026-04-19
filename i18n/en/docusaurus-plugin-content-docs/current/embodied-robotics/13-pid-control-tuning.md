---
title: "PID Control Principles and Anti-windup"
prerequisites: ["09-robot-dynamics-modeling"]
estimated_time: 45
difficulty: 3
tags: ["pid", "control", "anti-windup", "cascade"]
sidebar_position: 13
---

# PID Control Principles and Anti-windup

## You Will Learn

- Describe precisely what each PID term does and why the combination makes a system accurate, fast, and stable
- Diagnose "joint saturates, then releases with wild overshoot" as integral windup and pick the right anti-windup strategy on the spot
- Draw the three-loop cascade PID architecture in an interview and explain why tuning proceeds from the innermost loop outward

## Core Concepts

**Precise Definition**: A **PID controller** computes the control signal as a weighted sum of the present error (P), the accumulated past error (I), and the rate of change of the error (D), then sends it to the actuator. It is the "last mile" of closed-loop control — turning the desired trajectory from upstream planners into physical torques or voltages that the motor can actually track.

**Location in the Sense → Plan → Control Loop**:
- **Input**: error $e(t) = x_{\text{desired}}(t) - x_{\text{actual}}(t)$ (position, velocity, or torque-level difference)
- **Output**: control signal $u(t)$ (torque command / voltage / PWM duty)
- **Upstream**: IK solver and trajectory planner supply $x_{\text{desired}}$
- **Downstream**: actuator (motor driver) receives $u$; sensors feed back $x_{\text{actual}}$ to close the loop
- **Loop node**: sits at the **terminal end of control** — the translator from "math plan" to "physical reality"

**One-line version**: "PID is the robot's spinal reflex — it sees a deviation and immediately pushes back, no re-planning required."

**Minimum Sufficient Math**:

1. **Continuous-time PID control law**:

$$
u(t) = K_p \, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \, \frac{de(t)}{dt}
$$

**Physical meaning**: $K_p e$ reacts to "how far off right now" with immediate corrective force; $K_i \int e$ watches "how much total error has piled up" and gradually compensates steady-state offsets (gravity, friction); $K_d \dot{e}$ watches "which way the error is heading" and brakes early to prevent overshoot.

2. **Discrete position form vs incremental form**:

$$
u[k] = K_p\,e[k] + K_i \sum_{j=0}^{k} e[j]\,\Delta t + K_d \frac{e[k]-e[k-1]}{\Delta t} \quad \text{(position form)}
$$

$$
\Delta u[k] = K_p(e[k]-e[k-1]) + K_i\,e[k]\,\Delta t + K_d\frac{e[k]-2e[k-1]+e[k-2]}{\Delta t} \quad \text{(incremental form)}
$$

**Physical meaning**: the position form outputs the **absolute** control value each step — straightforward but the integral term can blow up; the incremental form outputs only the **change** $\Delta u$, inherently preventing historical accumulation, making it ideal for stepper motors or communication-delayed actuators.

3. **Integral windup and anti-windup (the critical safeguard)**:

$$
\text{Clamping: } \quad \text{if } |u| \geq u_{\text{max}}, \quad \text{stop integrating} \; (\dot{I} = 0)
$$

**Physical meaning**: when the actuator is already saturated (the motor cannot exert more torque), continuing to accumulate the I term only inflates the internal state. The moment the error reverses, the bloated I term causes violent overshoot or loss of control. Clamping simply says "stop integrating while saturated" — the most intuitive and most common approach.

<details>
<summary>Deep dive: full comparison of three anti-windup strategies with math</summary>

### 1. Clamping (output limiting + integral freeze)

The simplest approach: when the total control output $u$ exceeds the saturation limit $u_{\text{max}}$, freeze the integral term.

```
if |u_total| >= u_max:
    integral_term = integral_term  # stop accumulating
else:
    integral_term += Ki * e * dt
```

**Pros**: one-line `if`, virtually zero computational overhead.
**Cons**: the freeze/unfreeze threshold is coarse; recovery from saturation may lag slightly.

### 2. Integral separation

When the error is large, I is mostly irrelevant (P and D dominate), so just switch it off:

$$
u = K_p e + \begin{cases} K_i \int e\,dt & \text{if } |e| < e_{\text{threshold}} \\ 0 & \text{if } |e| \geq e_{\text{threshold}} \end{cases} + K_d \dot{e}
$$

**Use case**: large transient errors at startup; I only matters near steady state.
**Drawback**: requires manually choosing $e_{\text{threshold}}$; poor choices cause a control-signal discontinuity at the switching point.

### 3. Back-calculation (tracking)

The most refined method: feed back the saturation error into the integrator so that I decays automatically.

$$
\dot{I}(t) = K_i \, e(t) + \frac{1}{T_t}\bigl(u_{\text{sat}} - u_{\text{raw}}\bigr)
$$

where $u_{\text{raw}}$ is the unsaturated output, $u_{\text{sat}}$ is the clamped output, and $T_t$ is the tracking time constant (typically $T_t = \sqrt{T_i \cdot T_d}$).

**Physical meaning**: when $u_{\text{raw}} = u_{\text{sat}}$ (no saturation), the feedback term is zero and I accumulates normally; during saturation, the feedback term "pulls back" the excess I at a rate controlled by $T_t$.

**Pros**: smoothest transitions, no switching discontinuities.
**Cons**: one extra parameter $T_t$; slightly more complex to implement.

### Industrial rule of thumb

| Scenario | Recommendation | Rationale |
|----------|---------------|-----------|
| Embedded MCU, tight resources | Clamping | Zero overhead, easy to debug |
| Huge initial error at startup | Integral separation | Prevents I from exploding during transient |
| High-performance servo drive | Back-calculation | Smoothest saturation recovery |
| Not sure which to pick | Start with clamping | Covers 80% of cases; upgrade later |

</details>

**Ziegler-Nichols tuning (engineering starting point)**:

| Controller | $K_p$ | $T_i$ | $T_d$ |
|------------|-------|-------|-------|
| P | $0.5\,K_u$ | — | — |
| PI | $0.45\,K_u$ | $T_u / 1.2$ | — |
| PID | $0.6\,K_u$ | $T_u / 2$ | $T_u / 8$ |

**Physical meaning**: first find the ultimate gain $K_u$ (gain at which the system sustains constant-amplitude oscillation) and the oscillation period $T_u$, then use the table above for initial values. This is only a **starting point** — Ziegler-Nichols assumes a linear model without friction, backlash, or gravity compensation, so real hardware always needs manual fine-tuning.

**Cascade PID architecture**:

$$
\text{Position loop (outer)} \xrightarrow{e_{\text{pos}}} \text{Velocity loop (mid)} \xrightarrow{e_{\text{vel}}} \text{Current loop (inner)}
$$

**Physical meaning**: the outer loop (position, ~100 Hz, P or PD) outputs a velocity setpoint; the middle loop (velocity, ~1 kHz, PI) outputs a current setpoint; the inner loop (current, ~10 kHz, PI) outputs PWM. **Inner loops must be faster than outer loops**, and tuning proceeds from the innermost loop outward — there is no point tuning the outer loop if the inner loop is not yet stable.

<details>
<summary>Deep dive: cascade PID bandwidth design rules and tuning procedure</summary>

### Bandwidth separation principle

The key to cascade control is that **the inner loop bandwidth must be at least 5–10x the outer loop bandwidth**. This makes the inner loop appear as an "ideal tracker" to the outer loop, effectively decoupling the dynamics.

Typical industrial servo bandwidth allocation:

| Loop | Update rate | Bandwidth | Controller | Physical role |
|------|------------|-----------|------------|---------------|
| Current | 10–20 kHz | ~1 kHz | PI | Controls winding current = controls torque |
| Velocity | 1–5 kHz | ~100 Hz | PI | Controls speed, rejects load disturbances |
| Position | 100–500 Hz | ~10 Hz | P or PD | Controls angular accuracy |

### Tuning order (inside out)

1. **Current loop first**: disconnect the velocity loop, apply a current step command, tune PI until the current response settles in ~1 ms with no overshoot.
2. **Velocity loop next**: with the current loop stable, apply a velocity step, tune PI for smooth settling in 5–10 ms.
3. **Position loop last**: with the velocity loop stable, apply a position step, tune P (or PD) for settling in 50–100 ms with the required accuracy.

Why not the reverse? If the current loop is unstable, the velocity loop's output (a current command) cannot be tracked — tuning the outer loop becomes meaningless.

### What happens without bandwidth separation

If the position loop and velocity loop bandwidths are too close (e.g. both ~100 Hz), the dynamics of both loops couple and interfere:
- Low-frequency oscillation (both loops "fighting" for control)
- Gain margin drops precipitously
- Apparently "random" instability

</details>

## Intuition

**Driving analogy** (each term has a job):
- **P = watching the gap and pressing the gas**: the farther you fall behind the car ahead, the harder you press. But P alone always leaves a small gap at steady state (steady-state error).
- **I = gradually adding throttle in a headwind**: you notice the gap never closes, so you keep adding a little more gas over time. But accumulate too long and you will overshoot (windup).
- **D = seeing brake lights and easing off early**: you anticipate the gap closing and release the gas before you actually close it, preventing a crash (overshoot). But on a bumpy road (sensor noise), D makes you jerk the gas erratically.

**Simulator observation**: in Gazebo or Isaac Sim, set up a single-joint position controller:
1. **Pure P**: set $K_p = 10$, observe the joint tracks the target but always falls short (gravity causes steady-state error).
2. **Add I**: $K_i = 5$, steady-state error vanishes, but command a target beyond the torque limit → joint saturates → upon release, violent overshoot (windup, clearly visible).
3. **Add clamping anti-windup**: same scenario, overshoot disappears.
4. **Add D**: $K_d = 1$, faster response, less overshoot; but crank $K_d$ to 10 → joint starts buzzing (D amplifies encoder quantization noise).

**One-line mnemonic**: "P handles the present, I handles the past, D handles the future; anti-windup keeps I from losing its mind."

## Implementation Link

**Three representative engineering scenarios**:

1. **ROS 2 joint position control**: the `JointTrajectoryController` in `ros2_controllers` is PID under the hood. At startup it loads YAML parameters ($K_p$, $K_i$, $K_d$, $i_{\text{clamp}}$); each control tick it reads the actual joint angle from `/joint_states`, computes the error against the interpolated trajectory target, runs PID, and outputs a torque command.

2. **Embedded motor drive**: on an STM32 + FOC driver, the current-loop PID runs in a 10 kHz interrupt using the incremental form to avoid windup natively. The velocity loop runs at 1 kHz, the position loop at 100 Hz — nested at different interrupt priorities.

3. **Auto-tuning**: run a relay feedback test (the controller toggles on/off to induce a limit cycle), software reads $K_u$ and $T_u$ automatically, computes Ziegler-Nichols initial values, then runs a gradient-free optimizer for fine-tuning.

**Code skeleton** (C++, ROS 2 style):

```cpp
// PID controller core (with clamping anti-windup)
struct PIDController {
    double kp, ki, kd;
    double integral = 0.0;
    double prev_error = 0.0;
    double i_clamp;       // integral term clamp
    double output_max;    // output saturation limit

    double compute(double error, double dt) {
        // P
        double p_term = kp * error;
        // I (with anti-windup clamping)
        integral += error * dt;
        integral = std::clamp(integral, -i_clamp, i_clamp);
        double i_term = ki * integral;
        // D (add a low-pass filter in production to suppress noise)
        double d_term = kd * (error - prev_error) / dt;
        prev_error = error;
        // Output saturation
        double output = p_term + i_term + d_term;
        return std::clamp(output, -output_max, output_max);
    }
};
```

<details>
<summary>Deep dive: complete Python implementation (with back-calculation anti-windup + simulation test)</summary>

```python
import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    """PID controller with clamping and back-calculation anti-windup."""

    def __init__(self, kp, ki, kd, output_min, output_max,
                 anti_windup='clamping', tracking_tc=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.anti_windup = anti_windup

        # Back-calculation tracking time constant
        if tracking_tc is None:
            ti = kp / ki if ki > 0 else 1.0
            td = kd / kp if kp > 0 else 0.0
            self.tracking_tc = np.sqrt(ti * td) if td > 0 else ti
        else:
            self.tracking_tc = tracking_tc

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        # P term
        p_term = self.kp * error

        # D term (backward difference)
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # I term
        i_term = self.ki * self.integral

        # Raw (unsaturated) output
        u_raw = p_term + i_term + d_term

        # Saturate
        u_sat = np.clip(u_raw, self.output_min, self.output_max)

        # Anti-windup: update integral
        if self.anti_windup == 'clamping':
            if self.output_min < u_raw < self.output_max:
                self.integral += error * dt
        elif self.anti_windup == 'back_calculation':
            self.integral += (error + (u_sat - u_raw) / self.tracking_tc) * dt
        else:
            # No anti-windup (dangerous — for comparison only)
            self.integral += error * dt

        return u_sat


def simulate_pid(setpoint, controller, plant_mass=1.0, gravity=9.81,
                 torque_limit=15.0, duration=5.0, dt=0.001):
    """Simulate single-joint position control (with gravity + torque saturation)."""
    steps = int(duration / dt)
    t = np.zeros(steps)
    pos = np.zeros(steps)
    vel = np.zeros(steps)
    cmd = np.zeros(steps)

    for k in range(1, steps):
        t[k] = k * dt
        error = setpoint - pos[k - 1]
        u = controller.compute(error, dt)
        cmd[k] = u

        # Simple point mass + gravity: a = (u - m*g) / m
        accel = (np.clip(u, -torque_limit, torque_limit) - plant_mass * gravity) / plant_mass
        vel[k] = vel[k - 1] + accel * dt
        pos[k] = pos[k - 1] + vel[k] * dt

    return t, pos, cmd


# Compare three anti-windup strategies
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
setpoint = 2.0

for aw, label, color in [
    ('none', 'No anti-windup', 'red'),
    ('clamping', 'Clamping', 'blue'),
    ('back_calculation', 'Back-calculation', 'green'),
]:
    ctrl = PIDController(kp=20, ki=10, kd=5,
                         output_min=-15, output_max=15,
                         anti_windup=aw)
    t, pos, cmd = simulate_pid(setpoint, ctrl)
    axes[0].plot(t, pos, label=label, color=color)
    axes[1].plot(t, cmd, label=label, color=color, alpha=0.7)

axes[0].axhline(setpoint, ls='--', color='gray', label='Setpoint')
axes[0].set_ylabel('Position')
axes[0].legend()
axes[0].set_title('Anti-windup comparison (torque limit = 15 N·m)')
axes[1].set_ylabel('Command')
axes[1].set_xlabel('Time (s)')
axes[1].legend()
plt.tight_layout()
plt.savefig('pid_antiwindup_comparison.png', dpi=150)
plt.show()
```

The resulting plot clearly shows: red (no anti-windup) overshoots violently after saturation release; blue (clamping) greatly reduces overshoot; green (back-calculation) provides the smoothest transition.

</details>

## Common Misconceptions

1. **"Higher D gain = more stability"** — Wrong. The D term differentiates the error signal, amplifying high-frequency noise (encoder quantization, ADC noise). Excessive $K_d$ → high-frequency chattering in the control output → audible motor buzzing → accelerated mechanical wear. **Correct approach**: add a first-order low-pass filter before the D term (cutoff at ~1/10 of the control frequency), or use "derivative on measurement" (differentiate $-x_{\text{actual}}$ instead of $e$) to avoid the impulse from setpoint steps.

2. **"Steady-state error? Just crank up I"** — Steady-state error does not always need I to fix. If the offset comes from a known disturbance like gravity or Coulomb friction, a **feedforward** term ($u_{\text{ff}} = mg$ or $\hat{f}_{\text{friction}}$) plus PD control is faster, more stable, and avoids the phase lag and windup risk that I introduces. **Principle**: use feedforward for known disturbances; reserve I for unknown, slowly varying residuals only.

3. **"PID can handle everything"** — PID is a linear controller. Nonlinear systems (inertia coupling at high speed, Coriolis forces, sudden contact transitions) overwhelm it. The industry standard is **PID + dynamics feedforward** ($\tau_{\text{ff}} = M(q)\ddot{q}_d + C(q,\dot{q})\dot{q}_d + g(q)$), where PID only corrects the residual the feedforward misses. In interviews, saying "PID is feedback, dynamics is feedforward, they complement each other" is a strong statement.

4. **"Skipping anti-windup is fine"** — In simulation, forces may never truly saturate, so windup stays hidden. On real hardware there are physical torque limits; a collision or stall lets I accumulate unchecked → the instant the obstruction clears, the joint flings the workpiece or smashes the gripper. **Every deployed PID must have anti-windup** — this is the minimum bar for defensive programming.

## Situational Questions

<details>
<summary>Q1: A robot joint tracks its target but always has a 0.5° steady-state error. How do you analyze this? What do you check first?</summary>

**Complete reasoning chain**:

1. **Identify the source of steady-state error**: 0.5° in the gravity direction → strong suspicion of uncompensated gravity. Rotate the joint horizontal (gravity no longer acts on the joint axis) and check if the error disappears.
2. **If error disappears when horizontal**: confirmed gravity torque → do not add I. Apply **gravity feedforward** $\tau_{\text{ff}} = m g L \cos\theta$; PD control alone will then eliminate the steady-state error.
3. **If error persists when horizontal**: likely friction (Coulomb / Stribeck). Try friction feedforward $\hat{f} = f_c \cdot \text{sign}(\dot\theta) + f_v \cdot \dot\theta$.
4. **If feedforward is added and residual remains**: now add $K_i$, but **always with clamping anti-windup**. Set $i_{\text{clamp}}$ just above the expected steady-state correction magnitude.
5. **Trap to avoid**: jumping straight to a large $K_i$ → phase margin drops → the system may start oscillating, especially near velocity reversals (Stribeck region).

**What the interviewer wants to hear**: root-cause analysis first, feedforward for known disturbances, I as a last resort always paired with anti-windup.

</details>

<details>
<summary>Q2: The arm hits an obstacle and stalls for three seconds; when released, the joint swings wildly with massive overshoot. How do you diagnose and fix?</summary>

**Complete reasoning chain**:

1. **Symptom analysis**: stall → error stays positive → I term accumulates for three seconds → upon release, P+I+D all fire at once → enormous control signal → joint swings violently → classic **integral windup**.
2. **Quick verification**: check the I term in the log; during the stall it will rise monotonically to a very large value.
3. **Fix (simple to refined)**:
   - **Step 1**: add clamping anti-windup; set $i_{\text{clamp}}$ so that $K_i \cdot i_{\text{clamp}} < u_{\text{max}}$.
   - **Step 2**: if mild overshoot remains, switch to back-calculation ($T_t = \sqrt{T_i \cdot T_d}$) for smoother recovery.
   - **Step 3**: add **collision detection** — when the torque command is persistently saturated and velocity is zero, declare a collision, zero out I, and switch to a safe mode.
4. **Trap to avoid**: only reducing $K_i$ without adding anti-windup — steady-state performance degrades but the root cause (unbounded accumulation) is not solved.

**What the interviewer wants to hear**: a precise physical description of the windup process (stall → I explodes → release overshoot); knowledge that clamping is the standard first line; the ability to escalate to collision detection + safe mode.

</details>

<details>
<summary>Q3: Design a servo drive for a robot joint requiring ±0.01° position accuracy and 1 kHz velocity response. How do you architect the three-loop cascade PID?</summary>

**Complete reasoning chain**:

1. **Architecture (inside out)**:
   - **Current loop** (innermost): 10–20 kHz PI, controls winding current = controls torque. Target bandwidth ~1 kHz.
   - **Velocity loop** (middle): 1–5 kHz PI, controls speed. Target bandwidth ~100–200 Hz (at least 5x the position loop).
   - **Position loop** (outermost): 100–500 Hz P or PD, controls angular position. Target bandwidth ~10–30 Hz.
2. **Tuning order**:
   - Current loop first: disconnect the velocity loop, apply a current step, tune PI for settling in ~1 ms with no overshoot.
   - Velocity loop next: current loop is now stable; apply a velocity step, tune PI for smooth settling in 5–10 ms.
   - Position loop last: velocity loop is now stable; apply a position step, tune P (or PD) for settling in 50–100 ms at the required accuracy.
3. **Accuracy guarantee**: ±0.01° demands a high-resolution encoder (≥ 17-bit, 131072 counts/rev) plus a fast enough current loop to suppress torque ripple.
4. **Traps to avoid**:
   - Never tune the outer loop before the inner loop is stable (outer loop outputs become meaningless).
   - Position and velocity loop bandwidths too close → coupled oscillation.
   - Forgetting anti-windup on the current and velocity loops.

**What the interviewer wants to hear**: clear three-loop frequency allocation, the physical reason for inside-out tuning, the 5–10x bandwidth separation rule, and the link between accuracy and encoder resolution.

</details>

<details>
<summary>Q4: Your PID works perfectly in simulation, but on hardware the joint buzzes continuously. How do you troubleshoot?</summary>

**Complete reasoning chain**:

1. **Isolate D vs. quantization noise**: temporarily set $K_d = 0$. If buzzing disappears → D is amplifying sensor noise.
2. **Fix**:
   - Add a first-order low-pass filter to the D term: $D_{\text{filtered}}[k] = \alpha \cdot D_{\text{raw}}[k] + (1-\alpha) \cdot D_{\text{filtered}}[k-1]$, where $\alpha = \frac{dt}{dt + 1/(2\pi f_c)}$ and $f_c$ is set to ~1/10 of the control frequency.
   - Switch to "derivative on measurement": differentiate $-x_{\text{actual}}$ instead of $e$, which eliminates the impulse from setpoint changes.
3. **If buzzing persists with $K_d = 0$**: check discretization effects — the control rate may be too low (< 10x the mechanical resonance frequency). Increase the rate or lower $K_p$.
4. **If neither helps**: inspect driver dead zones, PWM resolution, and gear backlash — these nonlinearities cause the linear PID to chatter around zero.
5. **Trap to avoid**: blindly reducing all gains — steady-state performance and dynamic response both degrade, treating the symptom instead of the cause.

**What the interviewer wants to hear**: that D-term noise amplification is the number one sim-to-real discrepancy; and the two standard solutions — low-pass filtering and derivative on measurement.

</details>

## Interview Angles

1. **Anti-windup is the minimum bar for deployment** — This separates "wrote PID in a simulator" from "deployed PID on real hardware." Bring out with: "Every PID I ship includes clamping anti-windup at minimum, because real actuators have torque limits and a controller without windup protection is a ticking bomb."

2. **Feedforward + feedback separation** — Demonstrates systems thinking rather than "tune gains until it works." Bring out with: "When I see a steady-state error, I don't immediately reach for I. I first analyze whether gravity or friction is the cause and apply feedforward to cancel the known component — PID only handles the residual. This makes the system faster, more stable, and easier to tune."

3. **Cascade control with bandwidth separation** — Proves you understand real industrial servo architecture, not just the textbook single-loop PID. Bring out with: "A real servo drive runs three nested loops — current, velocity, position — with the inner loop bandwidth at least 5x the outer. Tuning goes inside out. Answering a PID question with only a single loop is an incomplete answer."

4. **Discretization and latency awareness** — Shows you understand the theory-to-implementation gap. Bring out with: "Continuous PID looks clean on paper, but on an MCU it is discrete. I choose incremental form to avoid windup, add a D-term low-pass to suppress noise, and verify the control rate is at least 10x the plant's mechanical bandwidth."

5. **Knowing PID's limits: when to move beyond PID** — Shows you are not a one-trick engineer. Bring out with: "For fast, nonlinear scenarios I use computed torque + PD, or MPC. PID is right for linearized-residual scenarios; forcing PID into whole-body control is the wrong tool for the job."

## Further Reading

- ***Embodied AI Algorithm Engineer Interview Questions*, Ch1.1 PID Fundamentals, Ch6.2 Classical Control** — curated high-frequency interview questions on anti-windup and cascade that you will almost certainly be asked
- **Astrom & Murray, *Feedback Systems*, Ch10–11** — theoretical foundations of PID tuning and anti-windup; MIT open courseware, free PDF
- **ROS 2 `ros2_controllers` source code, `PidROS` class** — see how production PID handles clamping, D-term filtering, and dynamic parameter reconfiguration
- **Paper: Bohn & Atherton, "An analysis of the anti-windup problem" (1995)** — the classic comparison of back-calculation vs tracking methods
- **Simulink / MATLAB PID Tuner App** — visual Bode-plot tuning to build intuition for gain margin and phase margin
- **Fuzzy PID for variable-load robots** — an advanced direction for when fixed gains are insufficient and the load varies with configuration
- **Smith Predictor** — the standard PID augmentation for systems with significant communication delay (e.g. teleoperation)
