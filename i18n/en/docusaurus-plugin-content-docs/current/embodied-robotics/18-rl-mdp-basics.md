---
title: "Reinforcement Learning Basics: MDPs and Q-Learning"
prerequisites: []
estimated_time: 50
difficulty: 3
tags: ["reinforcement-learning", "mdp", "q-learning", "reward"]
sidebar_position: 18
---

# Reinforcement Learning Basics: MDPs and Q-Learning

## You Will Learn

- Describe the MDP five-tuple and the physical meaning of the Bellman equation precisely enough for any interview
- Diagnose "RL training won't converge" or "reward hacking" scenarios by checking reward shaping, exploration strategy, and experience replay
- Decide when tabular Q-Learning suffices, when to jump to DQN, and when to switch to policy gradient methods

## Core Concepts

**Precise Definition**: **Reinforcement Learning (RL)** is a framework where an agent learns optimal behavior through interaction with an environment, guided by reward signals. **MDP (Markov Decision Process)** is the mathematical foundation of RL, formalizing sequential decision-making with the five-tuple $(S, A, P, R, \gamma)$. **Q-Learning** is the most classical model-free, off-policy RL algorithm that iteratively updates a Q-table to approximate the optimal action-value function.

**The MDP Five-Tuple**:
- $S$: state space — all possible environment states
- $A$: action space — all actions available to the agent
- $P(s' \mid s, a)$: state transition probability — the probability of reaching $s'$ after taking action $a$ in state $s$
- $R(s, a, s')$: reward function — the immediate feedback signal at each step
- $\gamma \in [0, 1)$: discount factor — balances the importance of immediate vs future rewards

**Location in the Sense → Plan → Control Loop**:
- **Input**: sensor observations → state representation $s$ (can be joint angles, image features, object poses)
- **Output**: policy $\pi(a \mid s)$ — what action to take in each state
- **Downstream consumers**: action $a$ is sent to the controller (torque command / velocity setpoint / high-level decision)
- **Loop node**: belongs to the **decision-control layer**, replacing hand-written if-else state machines. RL's core value is **letting the robot discover optimal behavior from experience automatically**, rather than manually designing rules.

**One-line version**: "Give the robot a goal and reward rules, let it figure out the best approach through trial and error — like training a dog with treats and commands."

**Minimum Sufficient Math**:

1. **Value Function (state value)**:

$$
V^\pi(s) = \mathbb{E}_\pi \left[ \sum_{t=0}^{\infty} \gamma^t R_t \mid S_0 = s \right]
$$

**Physical meaning**: starting from state $s$ and following policy $\pi$, the expected sum of all discounted future rewards. "How valuable is this state" — higher values mean more reward can be collected from here.

2. **Action-Value Function (Q function)**:

$$
Q^\pi(s, a) = \mathbb{E}_\pi \left[ \sum_{t=0}^{\infty} \gamma^t R_t \mid S_0 = s, A_0 = a \right]
$$

**Physical meaning**: starting from state $s$, taking action $a$, then following $\pi$ thereafter — the expected cumulative reward. "How good is this action in this state" — the action with the highest Q value is the current best choice.

3. **Bellman Optimality Equation** (the core recursive relation of RL):

$$
Q^*(s, a) = \mathbb{E} \left[ R + \gamma \max_{a'} Q^*(s', a') \right]
$$

**Physical meaning**: optimal Q value = immediate reward + discounted optimal Q of the next step. This recursion decomposes "long-term optimal" into "one step now + future optimal" and is the mathematical foundation of all Q-Learning family algorithms.

4. **Q-Learning Update Rule** (tabular, simplest form):

$$
Q(s, a) \leftarrow Q(s, a) + \alpha \left[ r + \gamma \max_{a'} Q(s', a') - Q(s, a) \right]
$$

**Physical meaning**: after each step ($s, a \rightarrow r, s'$), use the actual reward $r$ plus the best future estimate, compare against the current Q value, and correct by the learning rate $\alpha$ times the difference (TD error). Like "debriefing after each trade and slightly adjusting your assessment of that move."

<details>
<summary>Deep dive: full Bellman equation derivation and V/Q relationships</summary>

**Relationship from V to Q**:

$$
V^\pi(s) = \sum_{a} \pi(a \mid s) Q^\pi(s, a)
$$

$V$ is the policy-weighted average of all actions' $Q$ values.

**Bellman Expectation Equation** (non-optimal version, used for policy evaluation):

$$
V^\pi(s) = \sum_{a} \pi(a \mid s) \left[ R(s, a) + \gamma \sum_{s'} P(s' \mid s, a) V^\pi(s') \right]
$$

Unpacked: current state's value = choose action by policy → collect immediate reward → transition to new state → recursively compute new state's value.

**Bellman Optimality Equation** (finding the optimal policy):

$$
V^*(s) = \max_{a} \left[ R(s, a) + \gamma \sum_{s'} P(s' \mid s, a) V^*(s') \right]
$$

$$
Q^*(s, a) = R(s, a) + \gamma \sum_{s'} P(s' \mid s, a) \max_{a'} Q^*(s', a')
$$

**Why Q-Learning is off-policy**:
- The update uses $\max_{a'} Q(s', a')$ (greedy policy)
- The behavior uses $\epsilon$-greedy (exploration policy)
- The two differ → off-policy
- Benefit: can learn the optimal policy from experience collected by any behavior policy

**Convergence conditions** (Watkins & Dayan, 1992):
- Every $(s, a)$ pair is visited infinitely often
- Learning rate $\alpha_t$ satisfies $\sum \alpha_t = \infty$, $\sum \alpha_t^2 < \infty$ (e.g., $\alpha_t = 1/t$)
- Finite state and action spaces

In practice, a fixed learning rate with sufficiently many episodes converges to a near-optimal solution.

</details>

**Common APIs** (industry toolchain):

| Layer | Package | Purpose |
|-------|---------|---------|
| RL framework | Stable-Baselines3 / CleanRL | Standard algorithm implementations (PPO, SAC, DQN, etc.) |
| Environment interface | Gymnasium (OpenAI Gym) | Unified `reset() → step(action) → obs, reward, done` interface |
| Robot simulation | Isaac Gym / MuJoCo / PyBullet | Physics simulators, GPU-accelerated parallel training |
| Distributed training | Ray RLlib / Sample Factory | Massive parallel environments + multi-GPU training |
| Reward utilities | gymnasium.wrappers | Reward shaping, normalization, clipping |

## Intuition

**Analogy: training a dog to fetch**. You want to teach a dog to retrieve a ball. The dog (agent) is in a park (environment). Each time it performs an action (run, bite, carry back), you give a treat or verbal praise (reward). At first the dog runs randomly (exploration), but gradually learns "run toward ball → grab it → bring it back → maximum treats." $\gamma$ is the dog's patience — $\gamma$ near 1 means it will delay gratification for a bigger future reward; $\gamma$ near 0 means it only cares about what is right in front of it.

**Visual metaphor: Sudoku strategy**:
- Each cell's state = all numbers currently on the board
- Action = place a specific digit in a specific cell
- Reward = finishing the whole board without contradiction scores high; mid-puzzle contradiction penalizes
- Q-Learning records "given this board state, how much reward placing this digit here earns on average" — with enough experience the optimal strategy emerges

**Simulator observation**: run `CartPole-v1` in Gymnasium with Q-Learning. Observe: (1) in the first 100 episodes the pole falls immediately (random policy); (2) mid-training, the pole occasionally balances for 50 steps (Q-table estimates start to be meaningful); (3) later, it consistently holds for 500 steps (convergence). Visualize the Q-table as a heatmap and watch the pattern "pole tilting right + angular velocity leftward → push left" emerge.

## Implementation Link

**Three representative engineering scenarios**:

1. **Reward shaping for robotic grasping**: a binary "grasped/not grasped" 0/1 reward is too sparse — the agent learns nothing (random flailing always yields 0). Practice uses layered dense rewards: approaching the object +0.1, contacting it +0.3, gripping (contact force > threshold) +0.5, lifting to target height +1.0. Every step has a gradient signal guiding the agent in the right direction.

2. **Navigation and obstacle avoidance (discrete actions)**: a mobile robot uses Q-Learning to navigate a grid world maze. State = grid coordinate, actions = up/down/left/right, wall collision -1, goal reached +10, each step -0.01 (encourages taking shortcuts). This is the classic tabular Q-Learning scenario — the state space is small enough (< 10000) for a Q-table.

3. **Stepping stone to continuous state spaces**: when the state space grows large (e.g., six continuous joint angles → infinite state space), the Q-table cannot fit — this is the **curse of dimensionality**. Use a neural network to approximate the Q function (DQN). Key additions: experience replay buffer (breaks temporal correlation) + target network (stabilizes training).

**Code skeleton** (Python, tabular Q-Learning):

```python
import numpy as np
import gymnasium as gym

env = gym.make("FrozenLake-v1", is_slippery=False)
n_states = env.observation_space.n   # Number of states
n_actions = env.action_space.n       # Number of actions

Q = np.zeros((n_states, n_actions))  # Initialize Q-table
alpha = 0.1    # Learning rate
gamma = 0.99   # Discount factor
epsilon = 1.0  # Exploration rate (epsilon-greedy)

for episode in range(10000):
    state, _ = env.reset()
    done = False
    while not done:
        # Epsilon-greedy action selection
        if np.random.random() < epsilon:
            action = env.action_space.sample()     # Explore
        else:
            action = np.argmax(Q[state])            # Exploit

        next_state, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated

        # Q-Learning update
        td_target = reward + gamma * np.max(Q[next_state]) * (not terminated)
        Q[state, action] += alpha * (td_target - Q[state, action])

        state = next_state
    epsilon = max(0.01, epsilon * 0.995)  # Epsilon decay
```

<details>
<summary>Deep dive: complete DQN implementation skeleton (PyTorch) — the critical jump from Q-table to neural network</summary>

```python
import torch
import torch.nn as nn
import numpy as np
from collections import deque
import random

class QNetwork(nn.Module):
    """Neural network replaces Q-table: input state → output Q value for each action"""
    def __init__(self, state_dim, action_dim, hidden=128):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(state_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, action_dim),
        )

    def forward(self, x):
        return self.net(x)


class ReplayBuffer:
    """Experience replay: breaks temporal correlation for more stable training"""
    def __init__(self, capacity=100000):
        self.buffer = deque(maxlen=capacity)

    def push(self, state, action, reward, next_state, done):
        self.buffer.append((state, action, reward, next_state, done))

    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        states, actions, rewards, next_states, dones = zip(*batch)
        return (
            torch.FloatTensor(np.array(states)),
            torch.LongTensor(actions),
            torch.FloatTensor(rewards),
            torch.FloatTensor(np.array(next_states)),
            torch.FloatTensor(dones),
        )


def train_dqn(env, episodes=1000, batch_size=64, gamma=0.99, lr=1e-3):
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.n

    q_net = QNetwork(state_dim, action_dim)        # Online network
    target_net = QNetwork(state_dim, action_dim)    # Target network (stabilizes training)
    target_net.load_state_dict(q_net.state_dict())

    optimizer = torch.optim.Adam(q_net.parameters(), lr=lr)
    buffer = ReplayBuffer()
    epsilon = 1.0

    for ep in range(episodes):
        state, _ = env.reset()
        total_reward = 0
        done = False

        while not done:
            # Epsilon-greedy
            if random.random() < epsilon:
                action = env.action_space.sample()
            else:
                with torch.no_grad():
                    q_values = q_net(torch.FloatTensor(state))
                    action = q_values.argmax().item()

            next_state, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
            buffer.push(state, action, reward, next_state, float(done))
            state = next_state
            total_reward += reward

            # Start training only when buffer has enough samples
            if len(buffer.buffer) >= batch_size:
                s, a, r, ns, d = buffer.sample(batch_size)
                # Current Q values
                current_q = q_net(s).gather(1, a.unsqueeze(1)).squeeze()
                # Target Q values (computed with target network for stability)
                with torch.no_grad():
                    max_next_q = target_net(ns).max(1)[0]
                    target_q = r + gamma * max_next_q * (1 - d)
                # TD loss
                loss = nn.MSELoss()(current_q, target_q)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

        # Update target network every 10 episodes
        if ep % 10 == 0:
            target_net.load_state_dict(q_net.state_dict())
        epsilon = max(0.01, epsilon * 0.995)
```

**Three critical changes from Q-table to DQN**:
1. **Neural Network**: function approximation replaces the table, handling high-dimensional/continuous state spaces
2. **Experience Replay**: store past experiences $(s, a, r, s')$ and sample randomly for training, breaking temporal correlation
3. **Target Network**: use a delayed-update network for target Q computation, preventing the "chasing its own tail" instability

</details>

<details>
<summary>Deep dive: On-policy vs Off-policy — fundamental differences and selection</summary>

| Aspect | On-policy (e.g., SARSA, PPO) | Off-policy (e.g., Q-Learning, SAC) |
|--------|------------------------------|-----------------------------------|
| **Behavior policy** | Data collection policy = policy being learned | Data collection policy ≠ policy being learned |
| **Update target** | $Q(s', a')$, where $a'$ follows current policy | $\max_{a'} Q(s', a')$, independent of behavior policy |
| **Sample efficiency** | Low (use once then discard) | High (can repeatedly train on old experiences in replay buffer) |
| **Stability** | More stable | Requires target network and other tricks for stability |
| **Use cases** | Continuous control (PPO/TRPO) | Discrete actions (DQN) / continuous control (SAC) |

**Practical selection guide**:
- Discrete actions + small state space → tabular Q-Learning
- Discrete actions + large state space → DQN / Double DQN / Dueling DQN
- Continuous actions + need stable training → PPO (on-policy but simple to implement)
- Continuous actions + need sample efficiency → SAC (off-policy + entropy regularization)
- Real-robot training (samples are expensive) → prefer off-policy (SAC) + sim-to-real

</details>

## Common Misconceptions

1. **"Just train RL on the real robot"** — RL training requires millions to billions of trial-and-error steps. Real hardware gets damaged, motors burn out, and people get hurt. **Correct approach**: train to convergence in a simulator (MuJoCo / Isaac Gym) first, then use Domain Randomization + System ID for sim-to-real transfer. Only the final fine-tuning touches the real robot.

2. **"Q-table can solve anything"** — the Q-table size is $|S| \times |A|$. Once the state space is continuous or high-dimensional (e.g., six floating-point joint angles → infinite state space), the table literally cannot fit — this is the **curse of dimensionality**. **Stepping stone**: use a neural network to approximate the Q function (DQN), turning table lookup into function evaluation.

3. **"Reward design doesn't matter — the agent will figure out correct behavior on its own"** — reward design is the hardest part of RL. Sparse rewards (only score at the final success/failure) leave the agent with no gradient signal at all — it never converges. Poorly designed rewards cause **reward hacking**: the agent exploits loopholes (e.g., spinning in place to accumulate "distance" reward instead of actually navigating). **Avoid**: use layered dense rewards + energy penalties + periodically review agent behavior videos.

## Situational Questions

<details>
<summary>Q1 (medium): You are training a robot to grasp objects. The reward is "grasped +1, not grasped 0." After 1 million steps there is zero progress. How do you diagnose?</summary>

**Complete reasoning chain**:

1. **Diagnosis: reward is too sparse**: in a million-dimensional joint state-action space, random exploration is virtually guaranteed never to accidentally grasp the object. The agent receives only 0 reward from start to finish — the Q-table/Q-network gets no meaningful gradient signal.
2. **Fix 1 — layered dense reward**:
   - Arm approaches the object → +0.1 (distance reward, use FK to compute end-effector-to-object distance)
   - Contact with object → +0.3
   - Grip (contact force > threshold) → +0.5
   - Lift to target height → +1.0
   - Add -0.001 per-step time penalty (encourage fast completion)
3. **Fix 2 — Hindsight Experience Replay (HER)**: even if the original goal was not reached, treat the actually reached position as a "pretend goal" and store it in the replay buffer. This way, every step yields positive-reward experience.
4. **Fix 3 — Curriculum Learning**: start training with the object placed right next to the hand (easy); once success rate > 80%, gradually increase the object distance.
5. **Verify**: plot the reward curve — confirm average reward starts rising within 100k steps; watch agent behavior videos to confirm actions are meaningful (not jittering in place).

**What the interviewer wants to hear**: not just "change the reward," but knowledge of specific layered dense reward design + HER principle + curriculum learning approach.

</details>

<details>
<summary>Q2 (medium-hard): You train a navigation agent with DQN. The training curve rises then suddenly collapses (catastrophic forgetting). How do you handle it?</summary>

**Complete reasoning chain**:

1. **Root cause**: DQN uses its own predictions as targets (bootstrapping). If targets are unstable, updates amplify errors → positive feedback loop → Q values explode → policy collapses.
2. **Checklist**:
   - Is the replay buffer large enough? Too small → recent experiences overwrite old ones → forgets early lessons.
   - Is the target network update frequency correct? Too frequent → targets unstable; too slow → learning stalls.
   - Is the learning rate too high? → step size overshoots the optimum.
3. **Fixes**:
   - **Double DQN**: use online network to select the action, target network to evaluate Q — prevents Q-value overestimation.
   - **Increase replay buffer** (100k → 1M), ensuring old experiences are not overwritten too quickly.
   - **Soft update**: `target_params = tau * online_params + (1 - tau) * target_params`, tau = 0.005, smoother than hard copy.
   - **Gradient clipping**: `torch.nn.utils.clip_grad_norm_(params, max_norm=1.0)`, prevents gradient explosion.
4. **Monitoring**: plot Q-value distributions (not just reward curves). If Q values rise monotonically while reward stagnates or drops → confirmed Q-value overestimation problem.

**What the interviewer wants to hear**: awareness that DQN instability stems from the "deadly triad" of bootstrapping + function approximation + off-policy; concrete remedies — Double DQN, soft update, gradient clipping.

</details>

<details>
<summary>Q3 (hard): Your RL agent has learned a bizarre behavior — spinning in place to farm reward (reward hacking) instead of completing the actual task. How do you fix it?</summary>

**Complete reasoning chain**:

1. **Root cause analysis**: the reward function has a loophole. For example, the reward includes a "distance traveled" term; the agent discovers that spinning in place also counts as "travel" (angular velocity produces spurious displacement), which is easier than navigating to the goal.
2. **Verify**: record agent behavior videos and manually inspect; decompose the reward into individual terms (distance reward, time penalty, pose reward) and plot each separately to identify which term is being hacked.
3. **Solution levels**:
   - **Level 1 — fix the reward**: remove the spurious rotational displacement; compute only the Euclidean distance delta to the goal $\Delta d = d_{t-1} - d_t$.
   - **Level 2 — add constraint penalties**: add an energy consumption penalty $-\lambda \| \tau \|^2$ ($\tau$ = joint torques); spinning is energy-expensive and naturally becomes unprofitable.
   - **Level 3 — use IRL (Inverse RL)**: infer the reward function from human demonstrations, avoiding hand-designed reward loopholes.
   - **Level 4 — RLHF (RL from Human Feedback)**: have humans rank agent behaviors by preference and train a reward model from those preferences.
4. **Preventive measures**: during training, automatically record agent behavior videos at regular intervals (every 1000 episodes); set up behavior sanity checks (e.g., "average forward distance < 0.1 m but reward is still rising" → auto-alert).

**What the interviewer wants to hear**: reward hacking is one of the most common pitfalls in RL engineering; knowing the progression from "fix the reward" to "learn the reward"; emphasis on "always watch agent behavior videos — never just trust the reward curve."

</details>

## Interview Angles

1. **Exploration vs exploitation decay schedule** — the dividing line between "ran a tutorial" and "actually tuned RL." **Bring out with**: "The epsilon-greedy decay schedule is critical. I typically start at epsilon = 1.0 and decay linearly to 0.01, but I don't decay based on episode count — I decay based on the amount of experience in the replay buffer. If there isn't enough data yet, I don't rush to reduce epsilon, avoiding premature convergence to a local optimum."

2. **Hindsight Experience Replay (HER) for sparse rewards** — demonstrates depth in RL engineering techniques. **Bring out with**: "When sparse rewards cause non-convergence, my first instinct is not to rewrite the reward but to apply HER — store the actually reached position as a virtual goal in the buffer so the agent learns useful things from failures without modifying the reward at all."

3. **Value-based vs policy gradient selection** — a high-frequency interview question. **Bring out with**: "Discrete actions → DQN family (value-based), continuous control → SAC or PPO (policy gradient). But the real selection criterion is sample efficiency vs stability: off-policy (SAC) is sample-efficient but tuning-sensitive; on-policy (PPO) is stable but needs massive environment interaction. For real-robot training I prioritize SAC; for massive parallel simulation I use PPO."

4. **Reward shaping engineering principles** — in practice, 80% of RL time is spent on reward design. **Bring out with**: "Good reward design follows three principles: (1) dense — every step has signal; (2) monotonic — reward increases as you get closer to the goal; (3) exploit-proof — no shortcut that scores higher than completing the task. After designing a reward I always run a random policy first to confirm the random-action reward distribution is sensible."

## Further Reading

- ***Embodied AI Algorithm Engineer Interview Questions*, Ch4.1 MDP Fundamentals, Ch4.2 Q-Learning and DQN** — core RL interview questions, covering Bellman equation derivation + tabular vs function approximation comparison
- **Sutton & Barto, *Reinforcement Learning: An Introduction* (2nd ed.)** — the RL bible; Ch3-6 cover MDP through TD Learning fundamentals; free online version available
- **Mnih et al., *Human-level control through deep reinforcement learning* (Nature 2015)** — the seminal DQN paper; understand why experience replay + target network matter
- **Andrychowicz et al., *Hindsight Experience Replay* (NeurIPS 2017)** — the classic solution for sparse rewards, with deep impact on robotic grasping
- **Stable-Baselines3 official documentation** — the most widely used RL implementation library with comprehensive examples; ideal for quickly getting up to speed with different algorithms
- **CleanRL** — single-file RL algorithm implementations, one Python file per algorithm; ideal for reading source code to understand algorithm details
- **Isaac Gym / Isaac Lab official tutorials** — GPU-accelerated robot RL training environments; learn the engineering practices of large-scale parallel training
