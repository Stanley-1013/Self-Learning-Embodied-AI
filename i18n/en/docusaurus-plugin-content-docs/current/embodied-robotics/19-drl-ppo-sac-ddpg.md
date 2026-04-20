---
title: "Deep RL (PPO, SAC, DDPG)"
prerequisites: ["18-rl-mdp-basics"]
estimated_time: 50
difficulty: 4
tags: ["deep-rl", "ppo", "sac", "ddpg", "actor-critic", "reward-shaping"]
sidebar_position: 19
---

# Deep RL (PPO, SAC, DDPG)

## You Will Learn

- Precisely distinguish the design motivations and use cases of DQN / DDPG / TD3 / SAC / PPO — when an interviewer asks "why did your project choose SAC over PPO?", you can deliver a complete reasoning chain on the spot
- Diagnose "trained for 5000 epochs in the simulator and it still won't converge" by systematically checking reward shaping, hyperparameters, and value loss in that order
- Articulate the Actor-Critic architecture, PPO's clipping mechanism, and SAC's maximum entropy framework with physical meaning in under two minutes

## Core Concepts

### Seven Precise Definitions

**DQN (Deep Q-Network)**: approximates the Q-table with a neural network, combined with Experience Replay (breaks temporal correlation) and a Target Network (stabilizes the training target). Solves Q-value estimation in high-dimensional discrete action spaces. **Limitation**: discrete actions only — unusable for continuous joint control in robotics.

**DDPG (Deep Deterministic Policy Gradient)**: an Actor-Critic algorithm for continuous action spaces. The Actor outputs a **deterministic action** $\mu_\theta(s)$ (not a distribution); the Critic scores it with $Q_\phi(s,a)$. Exploration comes from additive OU noise. Off-policy with a replay buffer for sample efficiency.

**TD3 (Twin Delayed DDPG)**: three targeted fixes for DDPG's Q-value overestimation — twin Critics (take the minimum), delayed Actor updates (update Actor every $d$ steps), and target policy smoothing (add noise to the target action). A "stability patch" for DDPG.

**SAC (Soft Actor-Critic)**: off-policy Actor-Critic under a maximum-entropy framework. The objective includes an entropy bonus $\alpha \mathcal{H}(\pi)$ that encourages diverse exploration. Outputs a **stochastic policy** (a distribution, not a point value); temperature $\alpha$ is auto-tuned. Far more stable than DDPG in robotics.

**PPO (Proximal Policy Optimization)**: on-policy algorithm that replaces TRPO's KL constraint with a Clipped Surrogate Objective to bound policy update magnitude. Paired with GAE (Generalized Advantage Estimation) for bias-variance balance. Simple to implement and fast to converge with massively parallel environments.

**On-policy vs Off-policy**: on-policy (PPO) can only train on data from the current policy — stable but sample-inefficient. Off-policy (SAC, DDPG) reuses historical transitions from a replay buffer — sample-efficient but potentially unstable.

**Actor-Critic architecture**: Actor = policy network outputting actions; Critic = value network scoring those actions. Lower variance than pure Policy Gradient (Critic provides a baseline); handles continuous actions unlike pure value-based methods.

### Location in the Sense-Plan-Control Loop

- **Node**: DRL sits at the **highest level of the planning layer**, producing an end-to-end policy $\pi(a|s)$ that replaces hand-written state machines
- **Input**: state vector $s$ from the perception system (joint angles / end-effector pose / point cloud features / camera images)
- **Output**: action $a$ (joint torques / target joint angles / end-effector velocity commands)
- **Downstream**: low-level MPC / PID for hard real-time tracking + safety constraints; or DRL outputs torques directly via Sim-to-Real
- **One-liner**: "DRL lets a robot distill millions of simulated trial-and-error episodes into sensor-to-motor muscle memory."

### Algorithm Selection Comparison

| Property | DDPG | TD3 | PPO | SAC |
|----------|------|-----|-----|-----|
| Policy type | Deterministic | Deterministic | Stochastic | Stochastic |
| On/Off-policy | Off | Off | On | Off |
| Exploration | OU noise | Gaussian behavior noise (target smoothing is a critic-side regularizer) | Policy stochasticity | Maximum entropy |
| Sample efficiency | High | High | Low | High |
| Training stability | Poor | Medium | Good | Good |
| Hyperparameter sensitivity | Very high | High | Medium | Low ($\alpha$ auto-tuned) |
| Robotics sweet spot | Deprecated | Simple continuous | Massively parallel sim | General default |

### Five Core Math Formulas

1. **Policy Gradient Theorem**:

$$
\nabla_\theta J(\theta) = \mathbb{E}_{\pi_\theta} \left[ \nabla_\theta \log \pi_\theta(a|s) \cdot Q^{\pi}(s,a) \right]
$$

**Physical meaning**: adjust action probabilities proportionally to the Critic's score — high-scoring actions get their log-probability increased, low-scoring ones get suppressed. "Good actions are encouraged, bad ones are inhibited."

2. **PPO Clipped Surrogate Objective**:

$$
L^{\text{CLIP}}(\theta) = \mathbb{E}_t \left[ \min \left( r_t(\theta) \hat{A}_t, \; \text{clip}(r_t(\theta), 1-\varepsilon, 1+\varepsilon) \hat{A}_t \right) \right]
$$

where $r_t(\theta) = \frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{\text{old}}}(a_t|s_t)}$ is the probability ratio between new and old policies.

**Physical meaning**: $r_t$ measures "how much the policy changed"; $\varepsilon$ is the safety rope (typically 0.2). If the update is too large, clip it back — prevent one step from leaping too far and destabilizing training. An engineering realization of the trust region.

3. **SAC Maximum Entropy Objective**:

$$
J(\pi) = \mathbb{E}_\pi \left[ \sum_{t=0}^{\infty} \gamma^t \left( r_t + \alpha \mathcal{H}(\pi(\cdot|s_t)) \right) \right]
$$

**Physical meaning**: maximize not just cumulative reward, but also policy uncertainty — the entropy term $\alpha \mathcal{H}$ penalizes premature convergence to a single behavior pattern, preserving exploration. Temperature $\alpha$ auto-tunes: high early (explore broadly), low later (converge to optimal).

4. **TD Error (Temporal Difference)**:

$$
\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)
$$

**Physical meaning**: "actual reward from this step + estimated future value" minus "what we originally thought this state was worth." Positive $\delta$ means a pleasant surprise, negative means disappointment. The Critic learns by minimizing $\delta^2$.

5. **GAE (Generalized Advantage Estimation)**:

$$
\hat{A}_t = \sum_{l=0}^{\infty} (\gamma \lambda)^l \delta_{t+l}
$$

**Physical meaning**: $\lambda$ controls the bias-variance trade-off between 0 and 1. $\lambda=0$ uses only one-step TD (low variance, high bias); $\lambda=1$ is equivalent to Monte Carlo (high variance, low bias). PPO defaults to $\lambda=0.95$, leaning slightly toward lower variance.

<details>
<summary>Deep dive: DQN to SAC evolution chain with architecture rationale</summary>

### Evolution Timeline

```
Q-Learning (tabular)
    │ High-dim state space → approximate with NN
    ▼
DQN (2013, DeepMind)
  ├─ Experience Replay: break temporal correlation
  ├─ Target Network: stabilize target Q
  └─ Problem: discrete actions only
    │ Continuous action space → Actor outputs directly
    ▼
DDPG (Lillicrap et al., arXiv 2015 / ICLR 2016, DeepMind)
  ├─ Deterministic Actor μ(s) + Critic Q(s,a)
  ├─ OU noise for exploration
  ├─ Soft target update: θ' ← τθ + (1-τ)θ'
  └─ Problem: Q-value overestimation → instability
    │ Three targeted fixes
    ▼
TD3 (2018, Fujimoto)
  ├─ Twin Critic: take min(Q₁, Q₂) to suppress overestimation
  ├─ Delayed Actor Update: update Actor every d steps
  ├─ Target Policy Smoothing: add noise to target action
  └─ Problem: deterministic policy limits exploration
    │ Add maximum entropy framework
    ▼
SAC (2018, Haarnoja)
  ├─ Stochastic policy π(a|s) = tanh(μ + σ·ε)
  ├─ Auto-tuned temperature α → no manual exploration knobs
  ├─ Twin Critic (inherited from TD3)
  └─ Current default for continuous robotic control
```

### Why SAC Dominates Robotics

1. **Stable exploration**: the maximum entropy framework naturally encourages diverse actions without hand-tuning noise parameters
2. **Hyperparameter robustness**: auto-tuned $\alpha$ is easier to set than DDPG's OU noise or TD3's target smoothing standard deviation
3. **Multimodal policies**: stochastic policies can learn multiple solutions (e.g., reaching left or right around an obstacle); deterministic policies commit to one
4. **Off-policy sample efficiency**: real-robot data is expensive — a replay buffer lets you reuse every transition many times

### Where PPO Wins

PPO shines in **massively parallel GPU simulators** (Isaac Gym with thousands of environments):
- On-policy requires no replay buffer — constant memory footprint
- GPU-parallel data collection offsets the sample efficiency gap
- Simpler implementation, easier debugging
- OpenAI's RLHF (ChatGPT) also chose PPO, because LLM fine-tuning batches are large enough on their own

### Typical Hyperparameter Reference

| Parameter | PPO | SAC |
|-----------|-----|-----|
| Learning rate (Actor) | 3e-4 | 3e-4 |
| Learning rate (Critic) | 3e-4 | 3e-4 |
| $\gamma$ (discount) | 0.99 | 0.99 |
| $\varepsilon$ (clip) | 0.2 | — |
| $\lambda$ (GAE) | 0.95 | — |
| $\alpha$ (entropy) | — | Auto-tuned |
| Batch size | 2048-8192 | 256 |
| Replay buffer | — | 1e6 |
| Target update $\tau$ | — | 0.005 |

</details>

### Tool Chain

| Layer | Package | Purpose |
|-------|---------|---------|
| Algorithm library | Stable-Baselines3 (SB3) | Most mature PyTorch DRL library; `PPO("MlpPolicy", env)` trains in three lines |
| Minimal reference | CleanRL | Single-file implementations, ideal for reading source to understand algorithm details |
| GPU-parallel sim | Isaac Gym / Isaac Lab | NVIDIA GPU physics engine, thousands of envs running PPO simultaneously |
| Physics engine | MuJoCo | DeepMind's precise contact dynamics simulator, standard SAC/TD3 benchmark |
| Unified interface | Gymnasium (Farama) | Maintained fork of OpenAI Gym, standardized `env.step()` API |
| Lightweight sim | PyBullet | Free and open source, good for rapid prototyping |
| Training monitoring | Weights & Biases / TensorBoard | Real-time tracking of reward curves, value loss, entropy |

## Intuition

**Four Core Analogies**:

1. **Actor-Critic = player + coach**: the Actor is the player making decisions on the field; the Critic is the coach watching the big picture and scoring from the sideline. "That pass was great" (positive advantage) — player passes more next time. "Too risky" (negative advantage) — player avoids it.

2. **PPO clip = climbing safety rope**: policy updates are like rock climbing — you want to go higher (more reward), but the safety rope clips the **probability ratio** $r_t = \pi_\theta / \pi_{\text{old}}$ into the interval $[1-\varepsilon, 1+\varepsilon]$ (typically $[0.8, 1.2]$); once $r_t$ leaves the band, that sample stops contributing gradient. Note: the clip is on the probability ratio, not on action deviation or KL directly. TRPO is a precision safety harness (strict KL constraint); PPO approximates it with a clip (industrial safety rope) — easier to build, nearly as effective.

3. **SAC maximum entropy = explorer's mindset**: standard RL is "find one path and commit"; SAC is "find all viable paths and pick the best." The entropy term keeps the policy from being too certain, like an experienced explorer who never commits to a single route.

4. **Experience Replay = reviewing old notebooks**: off-policy algorithms store past $(s,a,r,s')$ tuples in a replay buffer and learn from them repeatedly. Like reviewing past exam problems before a test — but not in the original order (random sampling breaks correlation).

**Observable Phenomena in Simulators** (TensorBoard / W&B monitoring):

- **Clip ratio clustered near 1** — learning rate too small or insufficient exploration; the policy is barely moving
- **SAC entropy starts high then decays** — normal behavior: broad exploration early, maintaining trace entropy at convergence
- **Reward oscillating wildly** — two main suspects: reward function bug (some states give massive positive/negative rewards) or learning rate too high
- **Value loss >> policy loss** — Critic struggling to learn; usually POMDP (incomplete observations) or sparse rewards

## Implementation Link

**Three representative engineering scenarios**:

1. **Isaac Gym massively parallel PPO training**: run 4096 quadruped environments simultaneously on GPU, collect on-policy data each env step, PPO updates the policy each epoch. Completes millions of steps in 40 minutes (traditional CPU sim would take days).

2. **SAC + MuJoCo tabletop manipulation**: single MuJoCo environment running SAC with a 1M-transition replay buffer. After 500K steps the arm pushes a block to the target. Off-policy advantage: a small number of real-robot demos can be injected directly into the buffer for fine-tuning.

3. **Hierarchical control deployment (Sim-to-Real)**: high-level RL policy outputs target joint angles at 20 Hz; low-level PID/MPC tracks them at 1 kHz with torque limits. RL decides "where to go"; the low-level layer ensures "getting there safely."

**Code skeleton** (SB3 + Gymnasium):

```python
# Standard SAC training pipeline
import gymnasium as gym
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import EvalCallback

env = gym.make("FetchReach-v3")  # Continuous-action arm task
eval_env = gym.make("FetchReach-v3")

model = SAC(
    "MultiInputPolicy",
    env,
    learning_rate=3e-4,
    buffer_size=1_000_000,
    batch_size=256,
    tau=0.005,          # Soft target update coefficient
    gamma=0.99,
    ent_coef="auto",    # Auto-tuned α ← SAC's core feature
    verbose=1,
    tensorboard_log="./logs/",
)

eval_cb = EvalCallback(eval_env, eval_freq=5000, best_model_save_path="./best/")
model.learn(total_timesteps=500_000, callback=eval_cb)
```

<details>
<summary>Deep dive: complete Isaac Gym PPO training example (from environment definition to deployment)</summary>

```python
# Isaac Gym + rl_games PPO training pipeline
# Reference: NVIDIA IsaacGymEnvs framework

import isaacgym  # Must import first
import torch
from isaacgymenvs.tasks import isaacgym_task_map
from rl_games.algos_torch import torch_ext
from rl_games.common import env_configurations, vecenv
from rl_games.common.algo_observer import AlgoObserver
from rl_games.torch_runner import Runner

# ──────────────────────────────────────────────
# 1. Environment Configuration (Ant quadruped example)
# ──────────────────────────────────────────────
task_cfg = {
    "name": "Ant",
    "physics_engine": "physx",
    "env": {
        "numEnvs": 4096,              # 4096 envs on GPU simultaneously
        "envSpacing": 5.0,
        "episodeLength": 1000,
    },
    "sim": {
        "dt": 1.0 / 60.0,            # 60 Hz physics timestep
        "substeps": 2,
        "physx": {
            "num_threads": 4,
            "solver_type": 1,         # TGS solver
            "use_gpu": True,
        },
    },
}

# ──────────────────────────────────────────────
# 2. PPO Hyperparameters
# ──────────────────────────────────────────────
ppo_cfg = {
    "algo": {
        "name": "a2c_continuous",     # rl_games internal PPO implementation
    },
    "network": {
        "name": "actor_critic",
        "mlp": {
            "units": [256, 128, 64],  # 3-layer MLP
            "activation": "elu",
        },
    },
    "config": {
        "gamma": 0.99,
        "tau": 0.95,                  # GAE λ
        "e_clip": 0.2,               # PPO clip ε
        "learning_rate": 3e-4,
        "lr_schedule": "adaptive",    # KL-based LR decay
        "kl_threshold": 0.008,
        "mini_epochs": 5,            # Reuse each batch 5 times
        "minibatch_size": 16384,
        "horizon_length": 16,        # n-step rollout
        "normalize_advantage": True,
        "normalize_input": True,
        "reward_shaper": {
            "scale_value": 1.0,
        },
        "max_epochs": 5000,
    },
}

# ──────────────────────────────────────────────
# 3. Reward Shaping (quadruped locomotion example)
# ──────────────────────────────────────────────
def compute_reward(obs_buf, actions, reset_buf):
    """
    Reward design principles:
    - Dense rewards are primary (feedback at every step)
    - Multiple weighted terms, no single term dominates
    """
    # Forward velocity reward (primary objective)
    forward_reward = obs_buf[:, 0] * 2.0  # x-direction velocity

    # Alive reward (prevent giving up after falling)
    alive_reward = torch.ones_like(forward_reward) * 0.5

    # Action smoothness penalty (prevent jittering)
    action_penalty = -0.005 * torch.sum(actions ** 2, dim=-1)

    # Joint velocity penalty (prevent high-speed flailing)
    joint_vel_penalty = -0.001 * torch.sum(obs_buf[:, 13:21] ** 2, dim=-1)

    total_reward = forward_reward + alive_reward + action_penalty + joint_vel_penalty
    return total_reward

# ──────────────────────────────────────────────
# 4. Training Loop (simplified)
# ──────────────────────────────────────────────
runner = Runner()
runner.load(ppo_cfg)
runner.run({
    "train": True,
    "play": False,
    "checkpoint": None,
})
# Model saved in runs/ directory after training
# Test with runner.run({"play": True, "checkpoint": "path/to/model.pth"})

# ──────────────────────────────────────────────
# 5. Sim-to-Real Deployment Considerations
# ──────────────────────────────────────────────
# - Domain Randomization: randomize friction [0.5, 1.5], mass ±15%,
#   observation noise ±5%, actuator delay 1-3 steps
# - Hierarchical deployment: RL policy @ 20Hz → PD controller @ 1kHz
# - Action clamping: torch.clamp(action, -1, 1) * max_torque
# - Safety monitoring: if joint velocity exceeds threshold → switch to PD zero-position
```

**Key Design Decisions Explained**:

- **4096 environments**: massively parallel GPU simulation is what allows PPO to compete with off-policy methods on wall-clock time. num_envs x horizon_length determines the effective batch size per update
- **mini_epochs = 5**: reuse the same batch 5 times (PPO allows modest reuse, but not the tens-of-thousands reuse ratio of SAC)
- **adaptive LR**: monitors KL divergence and auto-adjusts learning rate. KL too large → lower LR (updates too aggressive); KL too small → raise LR (learning too slow)
- **reward shaping**: four weighted terms, each with clear physical meaning. Forward motion is the primary objective; others are constraints. Weights require iterative tuning

</details>

## Common Misconceptions

1. **"DRL doesn't need hyperparameter tuning — just throw data at it"** — Reality: DRL is **extremely sensitive** to hyperparameters. Changing the learning rate by one order of magnitude, adjusting clip from 0.2 to 0.3, or missing one penalty term can flip convergence to divergence. SB3 defaults are starting points; real tasks almost always require tuning.

2. **"You can train DRL directly on the real robot"** — Reality: millions of trial-and-error episodes are infeasible on hardware (wear, safety, time). The standard pipeline is **Sim → Domain Randomization → Sim-to-Real Transfer**. Real hardware is only used for fine-tuning or System ID to update simulator parameters.

3. **"Sparse rewards (only +1 when goal is reached) are sufficient"** — Reality: sparse rewards in high-dimensional continuous spaces almost always fail. The probability of random exploration stumbling onto the goal is near zero. You must design **dense intermediate rewards** (distance, alignment, etc.) or use HER (Hindsight Experience Replay) to retroactively relabel goals.

4. **"SAC is always better than PPO"** — Reality: in Isaac Gym scenarios with thousands of GPU-parallel environments, PPO's on-policy approach needs no replay buffer, has constant memory, and is easier to debug. Massive parallelism eliminates the sample efficiency disadvantage. SAC is better for single-environment or limited real-robot data scenarios.

5. **"Off-policy always saves more time than on-policy"** — Reality: off-policy is more sample-efficient, but each gradient step requires buffer sampling and dual-Critic computation. With enough GPU-parallel environments, on-policy wall-clock time can actually be faster (no need to maintain a massive buffer).

6. **"DRL is always better than PID"** — Reality: for straightforward trajectory tracking and setpoint control, a well-tuned PID/MPC is more reliable, more interpretable, and lower-latency. DRL's value lies in **high-dimensional decision-making + unstructured environments** (terrain adaptation, dexterous manipulation, multimodal perception). The common engineering pattern is a **hierarchical architecture**: DRL for high-level decisions, PID/MPC for low-level tracking.

## Situational Questions

<details>
<summary>Q1 (medium): You need to train a quadruped to walk over uneven terrain. DDPG, PPO, or SAC — how do you analyze the choice?</summary>

**Complete reasoning chain**:

1. **Task characteristics**:
   - Action space: 12 joint torques, high-dimensional continuous → DQN eliminated
   - Environment: random terrain requiring diverse gait adaptation → strong exploration needed
   - Simulator: Isaac Gym available, can run thousands of parallel envs

2. **Systematic elimination**:
   - **DDPG**: eliminated. Deterministic policy has weak exploration; likely gets stuck in a local optimum (e.g., learns to walk on flat ground only, falls on slopes)
   - **PPO**: viable. Massive parallel environments provide abundant on-policy data, training is stable, debugging is friendly. NVIDIA's official IsaacGymEnvs examples use PPO for quadruped locomotion
   - **SAC**: best fit. Maximum entropy framework naturally encourages diverse gaits (exploring both left-foot-first and right-foot-first patterns); off-policy is more sample-efficient with fewer environments

3. **Final choice**:
   - With Isaac Gym large-scale GPU parallelism → **PPO**, simple and stable, results in 40 minutes
   - With single MuJoCo environment → **SAC**, higher sample efficiency, stronger exploration
   - **Never DDPG**: superseded by TD3/SAC in all scenarios

**What the interviewer wants to hear**: not a memorized answer, but **systematic selection based on task characteristics (action space, exploration needs, simulator conditions)**, plus awareness that DDPG is no longer a first choice in 2024+.

</details>

<details>
<summary>Q2 (medium-hard): PPO training for 5000 epochs fails to converge — reward oscillates around -200. How do you troubleshoot?</summary>

**Complete reasoning chain**:

1. **Layer 1: Reward function bug** (most common):
   - Inspect every term of the reward function, print value distributions
   - Common bug: one penalty term's magnitude dwarfs the primary reward (e.g., action penalty weight too high — policy learns "doing nothing is optimal")
   - Verify: plot each reward component separately, check which dominates

2. **Layer 2: Hyperparameter issues**:
   - **Learning rate too high** → reward oscillates violently. Try reducing to 1e-4
   - **Clip $\varepsilon$ too large** → policy changes too much per step. Try reducing from 0.2 to 0.1
   - **GAE $\lambda$ too high** → variance too large. Try reducing to 0.9
   - **Batch size too small** → advantage estimates are noisy. PPO recommends 2048+

3. **Layer 3: Value loss far exceeds policy loss**:
   - Critic cannot learn effectively → observations are insufficient (POMDP problem)
   - Fix: add observation dimensions (include history frames or velocity estimates), use LSTM/Transformer for sequential observations
   - Or reward is too sparse for the Critic to extract meaningful signal

4. **Systematic debugging order**: reward bug → hyperparameters → observation design → network architecture

**What the interviewer wants to hear**: a clear debugging hierarchy (not random trial-and-error), starting with the lowest-cost checks (reward bugs only require reading code), then hyperparameters, and architecture changes only as a last resort.

</details>

<details>
<summary>Q3 (hard): Design a reward function for a peg-in-hole assembly task, and defend it against reward hacking.</summary>

**Complete reasoning chain**:

1. **Dense reward design** (coarse-to-fine guidance):
   - **Distance reward**: $r_{\text{dist}} = -\|p_{\text{peg}} - p_{\text{hole}}\|_2$ — closer to the hole opening means higher reward
   - **Alignment reward**: $r_{\text{align}} = \hat{z}_{\text{peg}} \cdot \hat{z}_{\text{hole}}$ — cosine similarity between peg axis and hole axis
   - **Contact force penalty**: $r_{\text{force}} = -\lambda \|F_{\text{contact}}\|_2$ — prevent violent impacts
   - **Sparse success bonus**: $r_{\text{success}} = +100$ — one-time large reward upon full insertion

2. **Weight design principles**:
   - Distance reward magnitude > alignment > force penalty (learn coarse positioning first, then fine alignment)
   - Success bonus must be large enough to matter, but not so large that the policy gambles for a lucky insertion while ignoring other terms

3. **Reward hacking defenses**:
   - **Jitter exploitation**: the policy may learn to oscillate rapidly near the hole to repeatedly collect "approaching target" reward → add **jerk penalty** $r_{\text{jerk}} = -\mu \|\ddot{a}\|_2$
   - **Edge-sticking**: the policy lodges the peg against the hole edge — distance score is maxed but it won't risk insertion → add **insertion depth reward** $r_{\text{depth}} = k \cdot d_{\text{insertion}}$
   - **Brute force**: slamming in with excessive force → contact force penalty + torque clamping

4. **Complete reward function**:

$$
r = w_1 r_{\text{dist}} + w_2 r_{\text{align}} + w_3 r_{\text{force}} + w_4 r_{\text{jerk}} + w_5 r_{\text{depth}} + r_{\text{success}}
$$

**What the interviewer wants to hear**: reward shaping is one of the hardest engineering problems in deploying DRL — being able to articulate specific anti-hacking strategies (not just designing rewards, but also plugging the loopholes that policies will exploit).

</details>

<details>
<summary>Q4 (hard): A policy trained in simulation flails the arm at dangerous speeds on the real robot. How do you handle it?</summary>

**Complete reasoning chain**:

1. **Root cause**: the simulator lacks realistic torque limits, joint compliance, and collision consequences. The policy learned "swing faster = more reward," but on real hardware this can damage equipment or injure people.

2. **Soft constraints (reward level)**:
   - **Action L2 penalty**: $r_{\text{smooth}} = -\alpha \|a\|_2^2$ — suppress large torque outputs
   - **Jerk penalty**: $r_{\text{jerk}} = -\beta \|a_t - a_{t-1}\|_2^2$ — suppress sudden acceleration/deceleration
   - **Joint velocity penalty**: $r_{\text{vel}} = -\gamma \|\dot{q}\|_2^2$ — limit joint angular velocity
   - **Problem**: soft constraints are only "suggestions" — if the primary reward is large enough, the policy may ignore penalties

3. **Hard constraints (safety guarantee layer)**:
   - **Control Barrier Function (CBF)**: add a safety projection layer after RL action output; if an action would leave the safe set, project it back to the safety boundary
   - **Action clamping**: `action = torch.clamp(action, -max_torque, max_torque)`
   - **Velocity monitoring**: if joint angular velocity exceeds threshold → emergency switch to PD zero-position hold

4. **Hierarchical control architecture (best practice)**:
   - **High-level RL policy** (20 Hz): outputs target joint angles or end-effector poses
   - **Low-level MPC / PD** (1 kHz): tracks targets + enforces torque limits + collision avoidance
   - RL decides "where to go"; the low-level layer guarantees "getting there safely"

5. **Summary**: soft constraints (reward penalties) + hard constraints (CBF / clamping) + hierarchical control (RL planning + MPC tracking) = three layers of defense

**What the interviewer wants to hear**: not relying solely on reward penalties (soft constraints can be ignored), having the concept of a **hard constraint safety layer** (CBF, hierarchical control), and knowing that hierarchical architecture is the industry standard for Sim-to-Real deployment.

</details>

## Interview Angles

1. **PPO vs SAC selection logic** — proves you make engineering decisions based on context, not memorized algorithm names. **Bring out with**: "For large-scale GPU parallelism (Isaac Gym, 4096 envs) I choose PPO — on-policy needs no replay buffer, memory is constant, debugging is straightforward. For single-environment or real-robot data scenarios I choose SAC — off-policy sample efficiency and maximum entropy exploration are more valuable. DDPG has been superseded by TD3/SAC; I would not use it."

2. **Reward shaping + hacking defense** — the biggest engineering pain point in deploying DRL. **Bring out with**: "My reward design follows a 'dense guidance + sparse bonus' principle, but I simultaneously defend against reward hacking — jerk penalties prevent oscillation exploits, depth rewards prevent edge-sticking. I always plot each reward component separately to verify magnitude balance."

3. **Actor-Critic variance reduction mechanism** — from mathematical understanding to engineering intuition. **Bring out with**: "Pure Policy Gradient has excessive variance (REINFORCE relies on Monte Carlo returns where trajectory returns vary wildly). Adding a Critic as a baseline compresses variance; GAE's $\lambda$ further balances bias-variance. PPO's clip then bounds the update step on top of this."

4. **Sim-to-Real hierarchical deployment** — the divide between "can only run demos in sim" and "has actually deployed." **Bring out with**: "I deploy with a three-layer architecture: RL policy at 20 Hz outputting targets, PD/MPC at 1 kHz for tracking + safety constraints, motor drivers at the bottom. Training includes Domain Randomization (friction +/-30%, mass +/-15%, observation noise +/-5%). When I find a sim-real gap on hardware, I do System ID to update the simulator."

5. **CBF hard constraints vs reward soft constraints** — critical awareness for safe deployment. **Bring out with**: "Reward penalties are merely 'suggestions' — the policy can ignore them if the primary reward is large enough. Real safety requires Control Barrier Functions projecting actions back into the safe set, or a hierarchical controller with low-level torque limiting. Three lines of defense: reward soft constraints + CBF hard constraints + hierarchical control."

## Further Reading

- ***Embodied AI Algorithm Engineer Interview Questions*, Ch4.3 DRL Algorithm Comparison, Ch12.3 Reward Shaping** — high-frequency interview topics: PPO/SAC selection, reward design pitfalls, on/off-policy trade-offs
- ***Embodied AI Algorithm Engineer Interview Questions*, Ch9 Sim-to-Real Transfer** — complete methodology for Domain Randomization, System ID, and hierarchical control deployment
- **PPO original paper (Schulman et al., 2017) + SAC original paper (Haarnoja et al., 2018)** — for PPO focus on the clip derivation motivation (why clip approximates TRPO); for SAC focus on the maximum entropy objective and auto-tuned $\alpha$
- ***Multimodal Foundation Models*, Ch6 RLHF** — PPO applied to LLM fine-tuning; understanding that DRL is not just for robots but a core tool for AI alignment
- **CleanRL GitHub repository** — single-file PPO / SAC / TD3 implementations with line-by-line comments, ideal for understanding algorithm internals
- **Hindsight Experience Replay (HER, Andrychowicz et al., 2017)** — the classic solution for sparse rewards, most effective when paired with SAC
- **DreamerV3 (Hafner et al., 2023)** — World Model direction, training policies in imagination, another order-of-magnitude improvement in sample efficiency
- **Isaac Lab official docs and examples** — NVIDIA's latest robotics RL training framework, successor to Isaac Gym
