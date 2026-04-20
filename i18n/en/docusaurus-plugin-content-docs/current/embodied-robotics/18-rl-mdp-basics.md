---
title: "Reinforcement Learning Basics: MDPs and Q-Learning"
prerequisites: []
estimated_time: 50
difficulty: 3
tags: ["reinforcement-learning", "mdp", "q-learning", "bellman", "exploration"]
sidebar_position: 18
---

# Reinforcement Learning Basics: MDPs and Q-Learning

## You Will Learn

- Precisely define the MDP five-tuple, Bellman equation, and V/Q functions in two sentences each — no hand-waving in interviews
- Diagnose "RL training won't converge" or "reward hacking" scenarios by checking reward shaping, exploration strategy, and on/off-policy selection
- Decide when tabular Q-Learning suffices, when to jump to DQN, and when SARSA is safer than Q-Learning

## Core Concepts

### Eight Precise Definitions

1. **MDP (Markov Decision Process)**: a mathematical framework that formalizes sequential decision-making with the five-tuple $(S, A, P, R, \gamma)$. $S$ is the state space, $A$ is the action space, $P(s'|s,a)$ is the transition probability, $R(s,a,s')$ is the reward function, $\gamma \in [0,1)$ is the discount factor. **Robotic grasping example**: $S$ = joint angles + object pose, $A$ = per-joint torque commands, $P$ = physics-engine transitions, $R$ = grasp success +1 / drop -0.5 / per-step -0.001, $\gamma = 0.99$.

2. **Bellman Equation**: the recursive expression of the principle of optimality — "a sub-policy of an optimal policy must itself be optimal." Decomposes long-term optimality into "this step + future optimal" and underpins all value-based RL.

3. **V(s) — State Value Function**: the expected sum of discounted future rewards starting from state $s$ and following policy $\pi$. "How valuable is this state."

4. **Q(s,a) — Action Value Function**: the expected cumulative reward of taking action $a$ in state $s$, then following $\pi$ thereafter. "How good is this action in this state." **Relationship**: $V(s) = \max_a Q(s,a)$ under the optimal policy.

5. **Policy $\pi(a|s)$**: the mapping from states to actions. **Deterministic** $\pi(s) = a$ (outputs an action directly) vs **stochastic** $\pi(a|s)$ (outputs a probability distribution over actions). Robotics often uses stochastic policies for exploration and robustness to perturbations.

6. **On-policy vs Off-policy**: On-policy (e.g., SARSA) updates the policy using data collected by that same policy — conservative and safe. Off-policy (e.g., Q-Learning) learns the optimal policy from data collected by any behavior policy — more sample-efficient but more aggressive.

7. **Q-Learning**: a model-free, off-policy RL algorithm. The update uses $\max_{a'} Q(s', a')$ (greedy) while behavior uses $\epsilon$-greedy (exploratory) — the mismatch makes it off-policy.

8. **$\epsilon$-greedy and TD vs MC**:
   - **$\epsilon$-greedy**: with probability $\epsilon$ explore randomly, with $(1-\epsilon)$ exploit the highest-Q action. Pure greed gets stuck in local optima; anneal $\epsilon$ from 1.0 down to 0.01
   - **TD (Temporal Difference)**: update after every step using bootstrapping (estimate from estimate) — low variance but biased
   - **MC (Monte Carlo)**: wait for the full episode, update with the actual return — unbiased but high variance. TD powers Q-Learning; MC powers REINFORCE

**Location in the Sense → Plan → Control Loop**:
- **Input**: sensor observations → state representation $s$ (joint angles, image features, object poses)
- **Output**: policy $\pi(a|s)$ — what action to take in each state
- **Downstream consumers**: action $a$ feeds into the controller (torque command / velocity setpoint / high-level decision)
- **Loop node**: belongs to the **decision-control layer**, replacing hand-written if-else state machines. RL complements classical control — RL handles high-level decisions ("push the obstacle aside or go around it"), PID/MPC handles low-level trajectory tracking

**One-line version**: "RL lets the robot forge sensor-to-motor decision instincts through trial and error in a sandbox."

### Six Core Formulas

1. **Return**:

$$
G_t = \sum_{k=0}^{\infty} \gamma^k R_{t+k+1}
$$

**Physical meaning**: from time $t$, sum all future rewards discounted by $\gamma^k$. $\gamma$ is the "temporal myopia" dial — near 1 looks far ahead (patient for big delayed rewards), near 0 only cares about what is immediate.

2. **Bellman Optimality Equation — V form**:

$$
V^*(s) = \max_{a} \left[ R(s,a) + \gamma \sum_{s'} P(s'|s,a) \, V^*(s') \right]
$$

**Physical meaning**: optimal state value = pick the best action → collect immediate reward → add the discounted optimal value of the next state. The $\max$ embodies the principle of optimality — make the best choice at every step.

3. **Bellman Optimality Equation — Q form**:

$$
Q^*(s,a) = R(s,a) + \gamma \sum_{s'} P(s'|s,a) \max_{a'} Q^*(s',a')
$$

**Physical meaning**: optimal Q value = immediate reward + discounted Q of the best action in the next state. This recursion decomposes "long-term optimal" into "one step + future optimal."

4. **Q-Learning Update Rule**:

$$
Q(s,a) \leftarrow Q(s,a) + \alpha \big[ r + \gamma \max_{a'} Q(s',a') - Q(s,a) \big]
$$

**Physical meaning**: after each step $(s,a \to r,s')$, compute TD error $= r + \gamma \max_{a'} Q(s',a') - Q(s,a)$ and correct by learning rate $\alpha$. $\alpha$ is the step size (too large oscillates, too small converges slowly), $\max$ makes it learn the optimal policy (off-policy), the bracket is "actual minus expected."

5. **SARSA Update Rule** (contrast):

$$
Q(s,a) \leftarrow Q(s,a) + \alpha \big[ r + \gamma Q(s',a') - Q(s,a) \big]
$$

**Physical meaning**: the only difference from Q-Learning — uses $Q(s', a')$ (action $a'$ chosen by the current policy) instead of $\max_{a'} Q(s', a')$. The result is more conservative: SARSA takes the long way around a cliff; Q-Learning walks right along the edge (it assumes future actions are always optimal).

6. **Policy Improvement Theorem**:

$$
Q^\pi(s, \pi'(s)) \geq V^\pi(s), \; \forall s \implies V^{\pi'}(s) \geq V^\pi(s), \; \forall s
$$

**Physical meaning**: if a new policy $\pi'$ picks actions that are no worse than the old policy $\pi$ in every state (Q value ≥ V value), then the new policy is at least as good overall. This is the theoretical guarantee that policy iteration converges.

<details>
<summary>Deep dive: full Bellman equation derivation and dynamic programming</summary>

**Relationship from V to Q**:

$$
V^\pi(s) = \sum_{a} \pi(a|s) \, Q^\pi(s,a)
$$

$V$ is the policy-weighted average of all actions' $Q$ values.

**Bellman Expectation Equation** (non-optimal version, used for policy evaluation):

$$
V^\pi(s) = \sum_{a} \pi(a|s) \left[ R(s,a) + \gamma \sum_{s'} P(s'|s,a) \, V^\pi(s') \right]
$$

Unpacked: current state's value = choose action by policy → collect immediate reward → transition to new state → recursively compute new state's value.

**Dynamic Programming Trilogy** (when the full MDP model is known):

1. **Policy Evaluation**: fix policy $\pi$, iterate the Bellman expectation equation until $V^\pi$ converges
2. **Policy Improvement**: greedily improve with $\pi'(s) = \arg\max_a Q^\pi(s,a)$
3. **Policy Iteration**: alternate evaluation → improvement until the policy stabilizes

**Value Iteration** fuses evaluation and improvement into one step:

$$
V_{k+1}(s) = \max_a \left[ R(s,a) + \gamma \sum_{s'} P(s'|s,a) \, V_k(s') \right]
$$

**Why Q-Learning is off-policy**:
- The update uses $\max_{a'} Q(s', a')$ (greedy policy)
- The behavior uses $\epsilon$-greedy (exploration policy)
- The two differ → off-policy
- Benefit: can learn the optimal policy from experience collected by any behavior policy

**Convergence conditions** (Watkins & Dayan, 1992):
- Every $(s,a)$ pair is visited infinitely often
- Learning rate $\alpha_t$ satisfies $\sum \alpha_t = \infty$, $\sum \alpha_t^2 < \infty$
- Finite state and action spaces

**TD vs MC mathematical comparison**:

| | TD(0) | Monte Carlo |
|---|---|---|
| Update target | $r + \gamma V(s')$ (bootstrapping) | $G_t$ (full return) |
| Bias | Biased (relies on estimates) | Unbiased (uses actual returns) |
| Variance | Low (one-step update) | High (accumulates full-episode randomness) |
| Applicability | Works for continuing tasks, no need to wait for episode end | Requires episodic tasks |

TD(n) interpolates between the two — uses $n$ steps of actual returns + bootstrapping, trading off bias and variance.

</details>

**Common APIs / Tools** (industry toolchain):

| Layer | Package | Purpose |
|-------|---------|---------|
| RL framework | Stable-Baselines3 / CleanRL | Standard algorithm implementations (PPO, SAC, DQN, etc.) |
| Environment interface | Gymnasium (OpenAI Gym) | `env.reset() → obs, info`; `env.step(action) → obs, reward, terminated, truncated, info` |
| Robot simulation | Isaac Gym / MuJoCo / PyBullet | Physics simulators, GPU-accelerated parallel training |
| Distributed training | Ray RLlib / Sample Factory | Massive parallel environments + multi-GPU training |
| Reward utilities | gymnasium.wrappers | Reward shaping, normalization, clipping |

**Hyperparameter Recommendations**:

| Hyperparameter | Tabular Q-Learning | DQN |
|----------------|-------------------|-----|
| Learning rate $\alpha$ | 0.01 – 0.1 | 1e-4 – 1e-3 |
| Discount factor $\gamma$ | 0.95 – 0.99 | 0.95 – 0.99 |
| $\epsilon$ initial | 1.0 | 1.0 |
| $\epsilon$ final | 0.01 | 0.01 |
| $\epsilon$ decay | Exponential ×0.995/ep | Linear over 100k steps |
| Replay buffer | — | 100k – 1M |
| Target net update | — | Hard copy every 1k–10k steps or soft $\tau=0.005$ |

## Intuition

**Four Analogies**:

1. **MDP = board game**: the board is the state space, legal moves are the action space, the board change after a move is the transition probability, winning +1 / losing -1 is the reward, $\gamma$ is how many uncomfortable intermediate moves you are willing to tolerate for a win
2. **Q-table = exam answer key**: each cell records "given this exam state, how many points does this answer earn on average." After enough practice exams you know which answer scores highest
3. **$\epsilon$-greedy = restaurant choice**: you have a go-to restaurant (exploit), but occasionally try a new place (explore). $\epsilon$ is the "try somewhere new" probability — high at first (still uncertain), gradually lowered (you know your favorites, but keep sampling occasionally)
4. **TD vs MC = scoring style**: TD is like rating each dish as it arrives (one-step update); MC is like giving a single overall score after the entire meal (update at episode end). Per-dish scoring is faster but potentially unfair (biased); whole-meal scoring is fairer but slower and subject to randomness across courses (high variance)

**Simulator observation**: run `CartPole-v1` in Gymnasium with Q-Learning. Observe: (1) first 100 episodes the pole falls immediately (random policy); (2) mid-training, it occasionally balances for 50 steps (Q-table estimates start to be meaningful); (3) later, it consistently holds for 500 steps (convergence). Visualize the Q-table as a heatmap and watch the pattern "pole tilting right + angular velocity leftward → push left" emerge.

In MuJoCo, run a robotic grasping scenario with layered dense rewards. At 10k steps the agent flails randomly; at 50k it begins approaching the object consistently; at 100k it learns to touch; at 200k it grasps. The reward curve shows distinct "staircase" rises at each phase transition.

## Implementation Link

**Three representative engineering scenarios**:

1. **Reward shaping for robotic grasping**: a binary "grasped/not grasped" 0/1 reward is too sparse — the agent learns nothing. Practice uses layered dense rewards: approaching +0.1, contact +0.3, grip +0.5, lift +1.0, per-step -0.001 (encourages speed) + collision -0.5 (safety).

2. **Grid-world navigation (discrete)**: a mobile robot uses Q-Learning to navigate a maze. State = grid coordinate, actions = up/down/left/right, wall collision -1, goal +10. Classic tabular scenario — state space is small enough (< 10000).

3. **Stepping stone to continuous spaces**: 6-DoF joint angles → infinite state space → Q-table cannot fit (curse of dimensionality). Use a neural network to approximate Q (DQN). Key additions: Experience Replay (breaks temporal correlation) + Target Network (stabilizes training) → the subject of Ch19.

**Code skeleton** (Python, tabular Q-Learning + Gymnasium):

```python
import numpy as np
import gymnasium as gym

env = gym.make("FrozenLake-v1", is_slippery=False)
Q = np.zeros((env.observation_space.n, env.action_space.n))
alpha, gamma, epsilon = 0.1, 0.99, 1.0

for episode in range(10000):
    state, _ = env.reset()
    done = False
    while not done:
        # Epsilon-greedy action selection
        if np.random.random() < epsilon:
            action = env.action_space.sample()       # Explore
        else:
            action = np.argmax(Q[state])              # Exploit
        next_state, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated
        # Q-Learning core update
        # Note (Gymnasium API): the bootstrap mask uses `terminated` only (a true absorbing
        # state with V(s')=0) — NOT `truncated`. A time-limit cutoff (e.g. CartPole 500-step
        # cap) means the underlying MDP continues, so you must still bootstrap with V(s');
        # otherwise states near the cutoff get systematically undervalued.
        td_target = reward + gamma * np.max(Q[next_state]) * (not terminated)
        Q[state, action] += alpha * (td_target - Q[state, action])
        state = next_state
    epsilon = max(0.01, epsilon * 0.995)  # Exponential decay
```

<details>
<summary>Deep dive: complete Q-Learning + DQN Python implementation</summary>

**Full tabular Q-Learning (with training monitoring)**:

```python
import numpy as np
import gymnasium as gym
from collections import defaultdict

def train_q_learning(env_name="FrozenLake-v1", episodes=10000,
                     alpha=0.1, gamma=0.99, eps_start=1.0,
                     eps_end=0.01, eps_decay=0.995):
    """Complete tabular Q-Learning with logging"""
    env = gym.make(env_name, is_slippery=False)
    Q = np.zeros((env.observation_space.n, env.action_space.n))
    epsilon = eps_start
    reward_history = []

    for ep in range(episodes):
        state, _ = env.reset()
        total_reward = 0
        done = False
        steps = 0

        while not done:
            # Epsilon-greedy
            if np.random.random() < epsilon:
                action = env.action_space.sample()
            else:
                action = np.argmax(Q[state])

            next_state, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            # Q-Learning update
            # Bootstrap mask uses `terminated` only, NOT `truncated` (time-limit cutoff means the MDP continues)
            td_error = reward + gamma * np.max(Q[next_state]) * (not terminated) - Q[state, action]
            Q[state, action] += alpha * td_error

            state = next_state
            total_reward += reward
            steps += 1

        epsilon = max(eps_end, epsilon * eps_decay)
        reward_history.append(total_reward)

        if (ep + 1) % 1000 == 0:
            avg = np.mean(reward_history[-100:])
            print(f"Episode {ep+1}: avg_reward={avg:.3f}, epsilon={epsilon:.3f}")

    return Q, reward_history


# SARSA comparison (on-policy version)
def train_sarsa(env_name="CliffWalking-v0", episodes=5000,
                alpha=0.1, gamma=0.99, eps_start=1.0,
                eps_end=0.01, eps_decay=0.995):
    """SARSA — on-policy, safer than Q-Learning in cliff environments"""
    env = gym.make(env_name)
    Q = np.zeros((env.observation_space.n, env.action_space.n))
    epsilon = eps_start

    def eps_greedy(state):
        if np.random.random() < epsilon:
            return env.action_space.sample()
        return np.argmax(Q[state])

    for ep in range(episodes):
        state, _ = env.reset()
        action = eps_greedy(state)
        done = False

        while not done:
            next_state, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated
            next_action = eps_greedy(next_state)

            # SARSA update: uses Q(s', a') instead of max Q(s', ·)
            # Bootstrap mask uses `terminated` only, NOT `truncated` (time-limit cutoff means the MDP continues)
            td_error = reward + gamma * Q[next_state, next_action] * (not terminated) - Q[state, action]
            Q[state, action] += alpha * td_error

            state, action = next_state, next_action

        epsilon = max(eps_end, epsilon * eps_decay)

    return Q
```

**Complete DQN implementation (PyTorch)**:

```python
import torch
import torch.nn as nn
import numpy as np
from collections import deque
import random
import gymnasium as gym


class QNetwork(nn.Module):
    """Neural network replaces Q-table: input state → output Q value per action"""
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
    """Experience replay: breaks temporal correlation for stable training"""
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

    def __len__(self):
        return len(self.buffer)


def train_dqn(env_name="CartPole-v1", episodes=1000, batch_size=64,
              gamma=0.99, lr=1e-3, buffer_size=100000,
              target_update=10, eps_start=1.0, eps_end=0.01, eps_decay=0.995):
    env = gym.make(env_name)
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.n

    q_net = QNetwork(state_dim, action_dim)        # Online network
    target_net = QNetwork(state_dim, action_dim)    # Target network
    target_net.load_state_dict(q_net.state_dict())

    optimizer = torch.optim.Adam(q_net.parameters(), lr=lr)
    buffer = ReplayBuffer(buffer_size)
    epsilon = eps_start

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
            if len(buffer) >= batch_size:
                s, a, r, ns, d = buffer.sample(batch_size)
                # Current Q values
                current_q = q_net(s).gather(1, a.unsqueeze(1)).squeeze()
                # Target Q values (computed with target network for stability)
                with torch.no_grad():
                    max_next_q = target_net(ns).max(1)[0]
                    target_q = r + gamma * max_next_q * (1 - d)
                # TD loss + gradient clip
                loss = nn.MSELoss()(current_q, target_q)
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(q_net.parameters(), max_norm=1.0)
                optimizer.step()

        # Periodically update target network
        if ep % target_update == 0:
            target_net.load_state_dict(q_net.state_dict())
        epsilon = max(eps_end, epsilon * eps_decay)

        if (ep + 1) % 50 == 0:
            print(f"Episode {ep+1}: reward={total_reward:.1f}, eps={epsilon:.3f}")

    return q_net
```

**Three critical changes from Q-table to DQN**:
1. **Neural Network**: function approximation replaces the table, handling high-dimensional / continuous state spaces
2. **Experience Replay**: store $(s,a,r,s')$ tuples and sample randomly, breaking temporal correlation
3. **Target Network**: use a delayed-update network for target Q computation, preventing the "chasing its own tail" instability

</details>

## Common Misconceptions

1. **"Just train RL on the real robot"** — RL training requires millions to billions of trial-and-error steps. Real hardware gets damaged, motors burn out, and people get hurt. **Correct approach**: train to convergence in a simulator (MuJoCo / Isaac Gym), then use Domain Randomization + System ID for sim-to-real transfer. Only the final fine-tuning touches the real robot.

2. **"Q-table handles any state space"** — Q-table size is $|S| \times |A|$. Six continuous joint angles → infinite state space → the table literally cannot fit. This is the **curse of dimensionality**. **Fix**: approximate Q with a neural network (DQN), turning table lookup into function evaluation.

3. **"Sparse reward plus lots of training will converge"** — if the reward only fires at final success/failure, random exploration almost never triggers success. The agent receives nothing but zeros — no gradient signal at all. **Fix**: layered dense rewards (signal at every step) + HER (Hindsight Experience Replay).

4. **"Q-Learning is always better than SARSA"** — Q-Learning learns the optimal policy ($\max$); SARSA learns the Q values of the current policy. In safety-critical settings (cliffs, collisions), SARSA factors in the risk of exploratory mistakes and learns more conservative, safer paths. **Classic example**: in CliffWalking, SARSA steers clear of the cliff while Q-Learning walks along the edge.

5. **"$\gamma = 1$ gives the longest planning horizon"** — with $\gamma = 1$ the return becomes an infinite series that may not converge, and the value function can diverge. **Safe range**: $\gamma \in [0.95, 0.99]$. The standard definition of **effective horizon** is $1/(1-\gamma) = 100$ steps for $\gamma = 0.99$; a separate heuristic is "the step count at which the discount factor drops to $1/e \approx 37\%$", which also lands near 100 for $\gamma = 0.99$ ($0.99^{100} \approx 0.37$). Both definitions are defensible — just don't conflate them.

## Situational Questions

<details>
<summary>Q1 (medium): You are training a 6-DoF robot arm to grasp objects. How should you design the reward?</summary>

**Complete reasoning chain**:

1. **Core problem**: grasping is the ultimate goal, but a 0/1 sparse reward cannot converge — the continuous state-action space of a six-axis arm is too vast for random exploration to ever reach a grasp
2. **Layered dense reward design**:
   - End-effector approaches object: $r_{\text{dist}} = -\| p_{\text{ee}} - p_{\text{obj}} \|$ (FK computes end-effector position; smaller distance → higher reward)
   - Contact with object: $+0.3$ (contact sensor triggers)
   - Grip held: $+0.5$ (gripper force > threshold and object has not slipped)
   - Lift to target height: $+1.0$ (successful grasp)
   - Safety term: table collision $-0.5$, joint near limit $-0.2$
   - Efficiency term: per-step $-0.001$ (encourages fast completion)
3. **Trap**: using only distance reward without safety penalties can teach the agent to slam into the object at high speed (reward hacking)
4. **Advanced options**: add CBF (Control Barrier Function) constraints for safety guarantees, or use HER to extract learning value from failures

**What the interviewer wants to hear**: concrete numeric design for layered dense rewards + safety constraints + alternative approaches for sparse rewards (HER / Curriculum Learning).

</details>

<details>
<summary>Q2 (medium-hard): Q-Learning works on your grid world, but completely fails when you move to 6-DoF robot control. What is the cause? How do you bridge the gap?</summary>

**Complete reasoning chain**:

1. **Diagnosis: curse of dimensionality**: grid world has 100 cells × 4 actions = 400-cell Q-table. A 6-DoF arm with continuous joint angles discretized to 100 bins each → $100^6 = 10^{12}$ cells — completely infeasible
2. **Fundamental mismatch**: tabular Q-Learning assumes every $(s,a)$ pair can be visited enough times; continuous spaces violate this assumption
3. **Solution path**:
   - **DQN**: approximate $Q(s,a)$ with a neural network — input state vector → output Q values per action. But the action space is also continuous → DQN only handles discrete actions
   - **DDPG / SAC**: actor-critic architecture where the actor outputs continuous actions and the critic evaluates Q values; two networks trained alternately. SAC adds entropy regularization for stability
   - **Discretization compromise**: coarsely discretize continuous actions (e.g., per-axis $\{-1, 0, +1\}$ torque) → $3^6 = 729$ discrete actions, run DQN as a proof of concept
4. **Engineering validation**: first verify on a simplified MuJoCo scenario (e.g., Reacher-v4, only 2 DoF), then scale up to 6 DoF

**What the interviewer wants to hear**: a clear progression from curse of dimensionality → function approximation → actor-critic architecture.

</details>

<details>
<summary>Q3 (hard): Your navigation agent has learned to clip through walls for shortcuts (reward hacking) instead of properly avoiding obstacles. How do you fix it?</summary>

**Complete reasoning chain**:

1. **Root cause analysis**: the reward has a loophole — likely it only measures "distance to goal decreased," and clipping through a wall drastically reduces distance, yielding high reward
2. **Verification**: record agent behavior videos + decompose reward into individual terms and plot each separately to identify which is being hacked
3. **Solution levels**:
   - **Level 1 — fix the reward**: add collision penalty $r_{\text{collision}} = -1.0$, ensuring any wall-clipping behavior has negative net reward
   - **Level 2 — add physical constraints**: use CBF (Control Barrier Function) to hard-constrain the agent from entering obstacle regions; let the reward handle only the task objective
   - **Level 3 — fix the environment**: ensure the simulator's collision detection is correct (some simple environments allow wall clipping); increase collision rebound forces
   - **Level 4 — learn the reward**: use IRL to infer the reward from human demonstrations, or RLHF to learn a reward model from human preferences
4. **Prevention**: auto-record videos every 1000 episodes + behavior sanity checks ("agent traversed an impassable region" → auto-alert)

**What the interviewer wants to hear**: reward hacking stems from reward–objective misalignment (a microcosm of the alignment problem); knowing the escalation from fix reward → add constraints → learn reward.

</details>

<details>
<summary>Q4 (medium): In the CliffWalking environment, what paths do SARSA and Q-Learning learn, and why do they differ?</summary>

**Complete reasoning chain**:

1. **Environment**: 4×12 grid, bottom row is a cliff (falling in gives reward = -100, resets to start), each step costs -1, goal is in the bottom-right corner
2. **Q-Learning (off-policy) path**: walks along the cliff edge — the shortest route. Because it updates with $\max_{a'} Q(s',a')$, it assumes future actions are always optimal → ignores the risk of $\epsilon$-greedy exploration accidentally stepping off the cliff
3. **SARSA (on-policy) path**: takes a higher, safer route away from the cliff. Because it updates with $Q(s',a')$ (the action actually chosen by $\epsilon$-greedy), the occasional cliff-fall penalty during exploration drags down Q values near the edge
4. **Core difference**: Q-Learning learns the "if I always play perfectly" optimal path; SARSA learns the "accounting for my occasional mistakes" safe path
5. **Selection insight**: real-robot environments (high cost of failure) → SARSA/on-policy is safer; pure simulation (trial and error is free) → Q-Learning/off-policy is more efficient

**What the interviewer wants to hear**: the on/off-policy difference is not just an algorithmic technicality but a fundamental question about whether exploration risk is factored into the learned values.

</details>

## Interview Angles

1. **Bellman equation recursive intuition** — an interview staple. **Bring out with**: "The Bellman equation's core is the principle of optimality: if your overall policy is optimal, then the sub-policy from any intermediate state must also be optimal. So optimal V decomposes into 'best reward this step + discounted optimal V next step' — this recursion is the bedrock of all value-based RL."

2. **On-policy vs off-policy safety** — separates "ran a tutorial" from "understands the design trade-off." **Bring out with**: "Q-Learning's $\max$ assumes future actions are always optimal, so it walks along the cliff's edge. SARSA folds exploration mistakes into its Q values, so it takes a wider path. For real-robot settings I lean on-policy first, because the cost of a single mistake is too high."

3. **Exploration-exploitation annealing design** — demonstrates real tuning experience. **Bring out with**: "$\epsilon$-greedy annealing is not just linear decay. I typically use exponential decay $\epsilon \times 0.995$, but the key is decaying based on replay-buffer experience volume, not episode count — if there is not enough data yet, I do not rush to lower $\epsilon$, avoiding premature convergence. Boltzmann exploration (softmax) is also more nuanced when action Q-value gaps are small."

4. **Q-table → DQN motivation and key changes** — bridges fundamentals to deep RL. **Bring out with**: "Q-table works for small discrete spaces, but continuous states trigger the curse of dimensionality. DQN approximates Q with a neural network, but naive training is unstable — so we add Experience Replay (breaks temporal correlation) and a Target Network (stabilizes the bootstrap target). Those two tricks are the soul of DQN."

5. **Reward shaping is RL's art** — signals engineering depth. **Bring out with**: "80% of RL engineering time goes into reward design. Good reward has three properties: dense (signal at every step), monotonic (increases as you approach the goal), and exploit-proof (no shortcut that scores higher than completing the task). After designing a reward I always run a random policy first to confirm the random-action reward distribution is sensible — if random actions score high, the reward has a hole."

## Further Reading

- **Sutton & Barto, *Reinforcement Learning: An Introduction* (2nd ed.)** — the RL bible; Ch3-6 cover MDP, Dynamic Programming, and TD Learning; free online; the single best resource for interview prep
- ***Embodied AI Algorithm Engineer Interview Questions*, Ch4.1 MDP Fundamentals, Ch4.2 Q-Learning and DQN** — core interview questions covering Bellman derivation + tabular vs function approximation
- ***Multimodal Foundation Models*, Ch2.4.2 RL and RLHF** — how RL integrates with foundation models; modern reward model training methods
- **Andrychowicz et al., *Hindsight Experience Replay* (NeurIPS 2017)** — the classic solution for sparse rewards, with deep impact on robotic grasping
- **Curriculum Learning for RL** — progressively increasing task difficulty accelerates convergence and improves final performance
- **Multi-step TD / TD($\lambda$)** — unified framework between TD and MC; $\lambda = 0$ is pure TD, $\lambda = 1$ is pure MC; practitioners often use $\lambda = 0.95$
- **Double Q-Learning (Hasselt 2010) + Prioritized Experience Replay (Schaul 2015)** — standard DQN upgrades addressing Q-value overestimation and prioritized learning
- **CleanRL** — single-file RL algorithm implementations, one Python file per algorithm; the fastest way to understand algorithm internals by reading source code
