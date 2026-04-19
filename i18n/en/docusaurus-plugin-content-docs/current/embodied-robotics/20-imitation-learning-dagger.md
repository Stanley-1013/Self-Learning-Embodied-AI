---
title: "Imitation Learning and Behavior Cloning"
prerequisites: ["19-drl-ppo-sac-ddpg"]
estimated_time: 45
difficulty: 3
tags: ["imitation-learning", "behavior-cloning", "dagger", "irl"]
sidebar_position: 20
---

# Imitation Learning and Behavior Cloning

## You Will Learn

- How to precisely articulate the core differences between Behavior Cloning (BC), DAgger, and Inverse RL (IRL): BC is pure supervised learning on demonstrations, DAgger uses online expert correction to solve distribution shift, IRL recovers the reward function
- When faced with "we have expert demonstration data but don't want to design a reward function," how to choose based on data quantity, interactive expert availability, and task complexity
- How to explain distribution shift, why BC collapses on long-horizon tasks, and how DAgger fixes it in a two-minute interview answer

## Core Concepts

**Precise Definition**: **Imitation Learning (IL)** trains an agent to learn a policy $\pi_\theta$ from expert demonstrations, without requiring a self-defined reward function. Core assumption: "I don't know what's good (reward), but I have someone showing me (demonstration)." The essential difference from RL — RL learns through trial-and-error + reward signals; IL learns through imitation + demonstration data.

**Three Main Approaches**:

- **Behavior Cloning (BC)**: The simplest — treat $(s, a)$ demonstration pairs as supervised learning $(x, y)$ and directly fit $\pi_\theta(a|s)$. It's just a regression/classification problem. Fast but fragile.
- **DAgger (Dataset Aggregation)**: Solves BC's distribution shift — run the student policy in the environment, when it deviates ask the expert to label the correct action, add new data to the training set, and iterate. Requires an interactive online expert.
- **Inverse RL (IRL)**: Doesn't learn a policy directly but recovers the reward function $R^*$ from demonstrations, then trains a policy using RL under the learned reward. Most flexible but slowest.

**Distribution Shift — The Core Challenge of IL**:

BC only sees states visited by the expert $s \sim d^{\pi_E}$ during training. At deployment, if the policy makes a small mistake and reaches a state the expert never visited $s' \notin d^{\pi_E}$ → the policy's output for $s'$ is garbage (never seen during training) → error compounds → cumulative drift → complete collapse.

$$
\text{BC cumulative error} \propto T^2 \cdot \epsilon
$$

where $T$ is the horizon length and $\epsilon$ is the single-step imitation error. **Physical meaning**: Errors accumulate quadratically — doubling the horizon makes cumulative error 4x worse. This is the mathematical reason BC inevitably collapses on long-horizon tasks (autonomous driving, long manipulation sequences).

**Position in the Perception → Planning → Control Loop**:
- **Input**: State/observation $s$ (same as RL policies) + expert demonstration dataset $\mathcal{D} = \{(s_i, a_i)\}$ (from human teleoperation or motion planners)
- **Output**: Policy $\pi_\theta(a|s)$ — interface identical to Deep RL policies
- **Downstream**: Actions go to low-level controllers. Complements Deep RL — RL needs reward but not demonstrations; IL needs demonstrations but not reward
- **Loop position**: Like Deep RL, spans **planning + control**, but the learning signal changes from reward to demonstration

**Essential Math**:

1. **Behavior Cloning Loss** (standard supervised learning):

$$
L_{BC}(\theta) = \mathbb{E}_{(s,a) \sim \mathcal{D}} \left[ \| \pi_\theta(s) - a \|^2 \right]
$$

**Physical meaning**: Minimize MSE between policy output and expert action. L2 loss for continuous actions, cross-entropy for discrete. Simple and direct, but only guarantees performance on the **expert data distribution**.

2. **DAgger Iterative Process**:

$$
\mathcal{D}_{i+1} = \mathcal{D}_i \cup \{ (s, \pi_E(s)) \mid s \sim d^{\pi_i} \}
$$

**Physical meaning**: Round $i$ runs the student policy $\pi_i$ in the environment to collect states $s$, then asks the expert "what would you do in this state?" to get $\pi_E(s)$. Add the new $(s, \pi_E(s))$ pairs to the dataset and retrain. Intuition: let the student make mistakes, the teacher corrects, next time they'll know.

3. **IRL Objective** (MaxEntropy IRL):

$$
\max_{R} \mathbb{E}_{\pi_E}[R(s,a)] - \log Z(R) - \lambda \|R\|
$$

where $Z(R) = \int \exp(\sum_t R(s_t, a_t)) d\tau$ is the partition function.

**Physical meaning**: Find a reward function $R$ that makes the expert's behavior the most probable among all possible behaviors (MaxEnt principle). $\lambda \|R\|$ regularizes to prevent reward overfitting. Ambiguity problem: multiple $R$ functions can explain the same set of demonstrations (reward ambiguity).

<details>
<summary>Deep Dive: Rigorous Distribution Shift Analysis and DAgger Convergence Proof</summary>

### Quantifying Distribution Shift

Let the expert policy's state distribution be $d^{\pi_E}$ and the student policy $\pi_\theta$'s state distribution be $d^{\pi_\theta}$. BC training data comes from $d^{\pi_E}$, but at deployment the policy faces $d^{\pi_\theta}$.

Performance gap upper bound (Ross et al., 2011):

$$
J(\pi_E) - J(\pi_\theta) \le T^2 \epsilon_{train} + O(T)
$$

where $\epsilon_{train}$ is the average single-step error on the **expert distribution**. Key: $T^2$ means even tiny single-step errors explode over long horizons.

### DAgger's Theoretical Guarantee

After $N$ DAgger iterations, the performance gap bound becomes:

$$
J(\pi_E) - J(\pi_{best}) \le \epsilon_{train} \cdot T + O(\sqrt{T \log N / N})
$$

Note: $T^2$ drops to $T$ (linear instead of quadratic). This is because DAgger collects data on the student's own distribution $d^{\pi_i}$ and has the expert label it, eliminating train-test distribution mismatch.

### Why IRL Has Ambiguity

Given a set of expert trajectories $\{\tau_1, \tau_2, ...\}$, infinitely many reward functions could make these trajectories optimal. Extreme example: $R(s,a) = 0$ for all $s, a$ — all behaviors are equally good, so expert behavior is trivially "optimal."

MaxEntropy IRL selects the reward that "makes the fewest additional assumptions" via the maximum entropy principle — but ambiguity can never be fully eliminated. In practice, IRL-learned rewards need manual sanity checks.

</details>

<details>
<summary>Deep Dive: GAIL and Diffusion Policy — Evolution of Modern Imitation Learning</summary>

### GAIL (Generative Adversarial Imitation Learning, 2016)

GAIL merges IRL + RL into one step — using a GAN framework to directly match the trajectory distribution of the policy with the expert's:

$$
\min_\pi \max_D \mathbb{E}_{\pi_E}[\log D(s,a)] + \mathbb{E}_{\pi}[\log(1 - D(s,a))]
$$

- Discriminator $D$: distinguish "is this $(s,a)$ from the expert or the student?"
- Generator (policy $\pi$): fool the Discriminator — make its own behavior increasingly resemble the expert's

Pros: No need to explicitly learn reward → bypasses IRL's ambiguity problem.
Cons: GAN training is unstable; requires extensive online interaction (inner loop needs RL).

### Diffusion Policy (2023)

Replaces simple MLP/Gaussian policies with a Diffusion Model:

$$
\pi_\theta(a|s) = \text{DenoisingProcess}(a^T \to a^0 | s, \theta)
$$

Pros:
- **Multi-modal actions**: Wiping a table can go clockwise or counter-clockwise; a Gaussian policy can only output the average (stuck in the middle doing nothing); Diffusion can represent multiple modes
- **High-dimensional continuous actions**: Outputs entire action sequences (action chunks) rather than single-step actions. Reduces compounding error
- **Combines with BC**: Diffusion Policy is essentially a stronger BC — using diffusion models for better conditional distribution modeling

Cons: Slow inference (requires multiple denoising steps); still challenging for 10-100 Hz control scenarios.

### Evolution Roadmap

```
BC (supervised) → DAgger (online correction) → GAIL (GAN+RL) → Diffusion Policy (stronger distribution modeling)
                                              → IRL (learn reward)
```

Current trend (2024): Diffusion Policy + action chunking is the hottest IL approach, particularly impressive on bimanual manipulation tasks.

</details>

## Intuition

**Analogy: Master teaching an apprentice.** BC is the master performing once for the apprentice on video; the apprentice practices by watching — great when steps are correct, but once they slip, the video never taught "what to do when you slip," so errors compound until total failure. DAgger is the master standing beside the apprentice watching — every time the apprentice slips, the master immediately says "do this instead," recording the correction for next time. IRL is the highest level: not learning the master's actions, but understanding *why* the master acts that way (the reward function), then independently discovering potentially better approaches.

**Distribution shift visual metaphor**: Like a self-driving car trained only on highway straight segments. On the road, a slight deviation → enters a "drifted from lane" state never seen in training → doesn't know how to correct → drifts more → complete loss of control. This is the essence of compounding error. DAgger's fix: let the car drive, but with an experienced driver in the passenger seat who says "turn left" or "turn right" whenever it starts drifting, recording these corrections for retraining.

**Simulator observation**: In MuJoCo's `Hopper-v4`:
- Collect 100 expert trajectories → BC training: first few steps match the expert, but around step 50 it starts drifting, falls by step 100
- Same data amount but with 5 rounds of DAgger: stable for 200+ steps
- BC with only 10 expert trajectories: collapses almost immediately — small dataset + distribution shift double hit
- Comparison: BC with data augmentation (add noise to states) survives 100+ steps — noise partially simulates deviation states

## Implementation Link

**Three typical engineering scenarios**:

1. **Teleoperation data → BC rapid prototyping (arm grasping)**: A human uses a SpaceMouse to teleoperate a robotic arm for 50 grasping demonstrations, collecting $(s, a)$ pairs, training an MLP policy. 5 minutes of training and it's deployable — but only works for object positions seen during training.

2. **DAgger in autonomous driving**: Start with a BC-trained basic driving policy, then put the car on the road (safety driver beside). Each time the policy is about to leave the lane, the safety driver intervenes → record $(s_{deviation}, a_{correction})$ → add to dataset and retrain. After 3-5 rounds, policy robustness improves significantly.

3. **IRL for learning human preferences (navigation)**: Observe human walking trajectories in an office, use IRL to recover "what makes a good path" — potentially including implicit preferences like "stay far from walls" or "avoid crowded corridors." Then the robot uses this reward for path planning.

**Code skeleton** (Python, BC and DAgger):

```python
import torch
import torch.nn as nn
import numpy as np

class BCPolicy(nn.Module):
    """Behavior Cloning: supervised learning to fit expert actions"""
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, act_dim),  # Continuous actions: direct output
        )

    def forward(self, obs):
        return self.net(obs)

# BC Training
# loss = F.mse_loss(policy(obs_batch), action_batch)
# optimizer.step()

# DAgger Iteration
# for i in range(num_dagger_rounds):
#     rollout_states = collect_rollout(policy)      # Student runs environment
#     expert_actions = query_expert(rollout_states)  # Ask expert
#     dataset.add(rollout_states, expert_actions)    # Aggregate data
#     policy = train_bc(dataset)                     # Retrain
```

<details>
<summary>Deep Dive: Complete DAgger Training Pipeline (Python)</summary>

```python
"""
DAgger: Complete flow from scratch to deployment
Dependencies: torch, gymnasium, numpy
"""
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import gymnasium as gym
from collections import deque

class MLPPolicy(nn.Module):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, act_dim),
        )

    def forward(self, obs):
        return self.net(obs)

    def get_action(self, obs):
        with torch.no_grad():
            obs_t = torch.FloatTensor(obs).unsqueeze(0)
            return self.net(obs_t).squeeze(0).numpy()

class DAggerTrainer:
    def __init__(self, env_name, expert_policy, obs_dim, act_dim):
        self.env = gym.make(env_name)
        self.expert = expert_policy  # Expert policy (or human interface)
        self.policy = MLPPolicy(obs_dim, act_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=1e-3)
        self.dataset = {"obs": [], "act": []}

    def collect_expert_demos(self, num_episodes=50):
        """Phase 0: Collect initial expert demonstrations"""
        for _ in range(num_episodes):
            obs, _ = self.env.reset()
            done = False
            while not done:
                action = self.expert.get_action(obs)
                self.dataset["obs"].append(obs)
                self.dataset["act"].append(action)
                obs, _, terminated, truncated, _ = self.env.step(action)
                done = terminated or truncated

    def dagger_iteration(self, num_rollouts=20, horizon=500):
        """Phase 1+: Collect states with student policy, label with expert"""
        new_obs, new_act = [], []
        for _ in range(num_rollouts):
            obs, _ = self.env.reset()
            for _ in range(horizon):
                # Student policy decides the action (but we collect the state)
                student_action = self.policy.get_action(obs)

                # Key: ask expert "what would you do in this state?"
                expert_action = self.expert.get_action(obs)

                new_obs.append(obs)
                new_act.append(expert_action)  # Record expert's action

                # Use student's action to step the environment (not expert's!)
                obs, _, terminated, truncated, _ = self.env.step(student_action)
                if terminated or truncated:
                    break

        # Aggregate into dataset
        self.dataset["obs"].extend(new_obs)
        self.dataset["act"].extend(new_act)

    def train_policy(self, epochs=50, batch_size=256):
        """Supervised learning to train policy"""
        obs_arr = torch.FloatTensor(np.array(self.dataset["obs"]))
        act_arr = torch.FloatTensor(np.array(self.dataset["act"]))
        dataset_size = len(obs_arr)

        for epoch in range(epochs):
            indices = np.random.permutation(dataset_size)
            total_loss = 0
            for i in range(0, dataset_size, batch_size):
                batch_idx = indices[i:i+batch_size]
                obs_batch = obs_arr[batch_idx]
                act_batch = act_arr[batch_idx]

                pred = self.policy(obs_batch)
                loss = nn.functional.mse_loss(pred, act_batch)

                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
                total_loss += loss.item()

        return total_loss / (dataset_size // batch_size)

    def run_dagger(self, num_rounds=5):
        """Complete DAgger pipeline"""
        # Phase 0: Initial expert demonstrations
        print("Phase 0: Collecting expert demonstrations...")
        self.collect_expert_demos(num_episodes=50)
        loss = self.train_policy()
        print(f"  Initial BC loss: {loss:.4f}")

        # Phase 1-N: DAgger iterations
        for round_i in range(1, num_rounds + 1):
            print(f"DAgger Round {round_i}/{num_rounds}")

            # Collect with student policy + expert labeling
            self.dagger_iteration(num_rollouts=20)

            # Retrain on aggregated data
            loss = self.train_policy()
            print(f"  Loss: {loss:.4f}, Dataset size: {len(self.dataset['obs'])}")

            # Evaluate
            rewards = self.evaluate(num_episodes=10)
            print(f"  Avg reward: {np.mean(rewards):.1f}")

    def evaluate(self, num_episodes=10):
        rewards = []
        for _ in range(num_episodes):
            obs, _ = self.env.reset()
            total_reward = 0
            done = False
            while not done:
                action = self.policy.get_action(obs)
                obs, r, terminated, truncated, _ = self.env.step(action)
                total_reward += r
                done = terminated or truncated
            rewards.append(total_reward)
        return rewards
```

**Key implementation details**:

- In `dagger_iteration`, the **student's action** is used to step the environment, but the **expert's action** is recorded — this is DAgger's core: collecting expert labels on the student's distribution
- The dataset grows continuously (aggregate), not replaced — retaining early data prevents forgetting
- In practice, "asking the expert" could be: a human teleoperation interface, an expensive motion planner, or a pre-trained RL policy
- Data augmentation can further improve results: adding Gaussian noise to observations simulates deviation states

</details>

## Common Misconceptions

1. **"BC just needs enough data to avoid distribution shift"** — Mathematically, BC's cumulative error is $O(T^2 \epsilon)$; even if $\epsilon$ is very small (lots of data gives good fit), sufficiently long $T$ will still cause collapse. And in practice, covering all possible deviation states is impossible — that's exponential. **Correct understanding**: More data reduces $\epsilon$ but cannot eliminate the $T^2$ amplification. Solutions are DAgger (fix the distribution) or action chunking (shorten effective horizon).

2. **"IRL can learn the one true reward function"** — IRL is inherently ambiguous. Most extreme: $R=0$ makes all behaviors equally good, so expert behavior is trivially "optimal." Even MaxEntropy IRL only selects the reward that "makes the fewest assumptions," not the uniquely correct one. **Correct understanding**: IRL-learned rewards need manual sanity checks, and different regularization/priors yield different rewards.

3. **"Imitation learning requires perfect expert demonstrations"** — BC/DAgger care more about demonstration **consistency** than perfection. If 10 demonstrations show 5 going left and 5 going right, the policy learns to "go middle" (averaging) — often the worst choice. Consistently going one way (even if suboptimal) works better. **Correct understanding**: Demonstration **consistency** matters more than **optimality**. If expert behavior has multiple modes, use mixture models or Diffusion Policy to represent multi-modal distributions.

## Situational Questions

<details>
<summary>Q1: You have 100 teleoperation demonstrations of robotic arm grasping. BC achieves 90% success on seen object positions but drops to 30% when objects move 5 cm. How do you analyze and improve?</summary>

**Analysis**:

1. **Root cause: distribution shift**. Training data only covers the object position distribution during expert operation. A 5 cm shift → enters states the policy hasn't seen → unreliable outputs
2. **Option 1: Data augmentation**: Add random offsets (plus/minus 10 cm) to object positions in observations during training, with corresponding action adjustments. Lets the policy see deviation states during training
3. **Option 2: DAgger**: Let the BC policy attempt grasps (objects randomly placed), when it fails have the human label the correct action, add to dataset and retrain. After 5 rounds, coverage of deviation states improves significantly
4. **Option 3: Action chunking**: Output the next 10 steps as an action sequence (action chunk) instead of single-step actions. This reduces effective horizon (from 100 steps to 10 chunks), reducing compounding error
5. **Pitfall to avoid**: Don't just pile on more demonstrations — 100 demos might all be within the same narrow distribution. What's needed is **distribution coverage** (diverse object positions), not repeated demos at the same position

**Conclusion**: Short-term use data augmentation + action chunking; if an online expert is available, use DAgger. The core is expanding training distribution coverage.

</details>

<details>
<summary>Q2: Your task requires a long manipulation sequence (open cabinet → take bowl → place on table → close cabinet). BC starts failing at step two. What do you do?</summary>

**Analysis**:

1. **Root cause**: Long horizon + compounding error. Four sub-tasks chained, each ~50 steps → effective $T = 200$. BC's $O(T^2)$ cumulative error at $T=200$ is uncontrollable
2. **Option 1: Hierarchical imitation**: Split the task into 4 sub-policies each with separate BC, use a high-level policy (state machine or learned meta-policy) to decide when to switch. Each sub-policy's effective horizon drops to ~50 steps
3. **Option 2: DAgger + segmentation**: Train BC on the full flow first, then DAgger on the real robot — but focus data collection on sub-task **transition points**, where distribution shift is most severe
4. **Option 3: Action chunking + Diffusion Policy**: Use Diffusion Policy to output long action chunks (e.g., 16 steps) with temporal ensemble for overlapping predictions. The ACT paper demonstrates effectiveness on bimanual manipulation
5. **Option 4: Goal-conditioned BC**: Each sub-task has a clear goal state (e.g., "door opened to 90 degrees"); the policy takes $(s, g)$ as input. Switch to the next sub-task upon reaching the goal
6. **Pitfall to avoid**: Don't try to learn the full 200-step sequence with a single end-to-end BC policy — even with 1000 demonstrations, compounding error's $T^2$ effect at $T=200$ is uncontrollable

**Conclusion**: Hierarchical imitation (reduce effective horizon) + action chunking (reduce compounding error). Long manipulation sequences are nearly impossible to solve with pure end-to-end BC.

</details>

<details>
<summary>Q3: Your team wants to use IRL instead of BC for learning robot table wiping. Is it worth it? How do you evaluate?</summary>

**Analysis**:

1. **IRL's advantages**: Learns a reward function rather than a policy → (a) can retrain with RL in new environments without new demonstrations; (b) more generalizable — reward describes "what is good" not "how to do it"; (c) can capture implicit preferences (e.g., "corners matter more than center")
2. **IRL's costs**: (a) Inner loop requires running RL → 10-100x BC training time; (b) reward ambiguity — multiple rewards can explain the same demonstrations, requires manual verification; (c) more hyperparameters and harder to debug
3. **Evaluation criteria**:
   - Will the table surface shape change? If it's always the same table → BC suffices
   - Need to transfer to new tables/objects? → IRL's learned reward is more transferable
   - Sufficient compute for the inner RL loop? → IRL training requires extensive RL in simulation
4. **Alternatives**: GAIL — skips explicit reward learning, directly matches trajectory distributions. More stable than IRL but still needs online interaction. Or Diffusion Policy — stronger distribution modeling than BC, avoids mode averaging
5. **Conclusion**: Unless there's a clear "transfer to new environments" requirement with sufficient compute, BC (or Diffusion Policy) + data augmentation + DAgger has higher ROI. IRL is better suited for research settings

**Key insight**: Don't be dazzled by "more advanced" methods; make engineering judgments based on actual needs and costs.

</details>

## Interview Angles

1. **Distribution shift is the core challenge of imitation learning** — A must-know concept. Lead with: "BC's cumulative error scales with the square of the horizon — even tiny per-step deviations compound over 100 steps into complete failure. This is distribution shift: the policy makes an error → reaches a state never seen during training → doesn't know how to correct → drifts further. DAgger's solution is collecting data on the student's own distribution and having the expert label correct actions."

2. **BC vs DAgger vs IRL selection logic** — Demonstrates methodological judgment. Lead with: "Lots of data + short horizon + no online expert → BC suffices. Long horizon + online expert available → DAgger. Need environment transfer + sufficient compute → IRL. In practice, 80% of scenarios can be solved with BC + data augmentation; DAgger is the upgrade path when BC hits its ceiling."

3. **Action chunking as an engineering weapon against compounding error** — Shows awareness of cutting-edge practice. Lead with: "Instead of outputting single-step actions, output a sequence of future actions; this effectively shortens the horizon. ACT and Diffusion Policy both use this technique, with remarkable results on bimanual manipulation. This is one of the most important engineering breakthroughs in IL from 2023-2024."

4. **Imitation learning and RL are complementary** — Shows big-picture vision. Lead with: "RL needs reward but not demonstrations; IL needs demonstrations but not reward. In practice they're often combined: use BC/DAgger for a good initial policy (warm start), then RL fine-tune to surpass the expert. Or use IRL to learn a reward from demonstrations, then train with PPO/SAC under the learned reward."

5. **Demonstration consistency matters more than perfection** — Separates "knows IL" from "has been burned by it." Lead with: "If expert demonstrations have multiple modes (half go left, half go right), standard BC learns to go middle — this mode averaging is usually the worst behavior. The fix is using policies that can represent multi-modal distributions (GMM, Diffusion Policy), or ensuring consistency during data collection."

## Further Reading

- **Ross et al., "A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning" (2011)** — DAgger original paper, rigorous distribution shift analysis and solution, clearly written
- **Ho & Ermon, "Generative Adversarial Imitation Learning" (2016)** — GAIL paper, unifying IRL + RL with a GAN framework, an important milestone in modern IL
- **Zhao et al., "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware" (2023)** — ACT (Action Chunking with Transformers) paper, breakthrough results on bimanual manipulation with action chunking + Transformer
- **Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion" (2023)** — Diffusion Policy paper, using diffusion models for policy representation, solving multi-modal action problems
- **Ziebart et al., "Maximum Entropy Inverse Reinforcement Learning" (2008)** — MaxEnt IRL original paper, an elegant solution to IRL's ambiguity problem
- **robomimic framework** — NVIDIA's imitation learning benchmark + implementation framework, supporting BC, BC-RNN, HBC, with complete evaluation pipeline
- **LeRobot (Hugging Face)** — Open-source robot learning platform supporting ACT and Diffusion Policy training and deployment with low-cost hardware
