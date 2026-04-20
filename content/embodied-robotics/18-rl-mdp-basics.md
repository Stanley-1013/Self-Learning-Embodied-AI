---
title: "強化學習基礎：MDP 與 Q-Learning"
prerequisites: []
estimated_time: 50
difficulty: 3
tags: ["reinforcement-learning", "mdp", "q-learning", "bellman", "exploration"]
sidebar_position: 18
---

# 強化學習基礎：MDP 與 Q-Learning

## 你將學到

- 能用兩句話講清楚 MDP 五元組、Bellman 方程、V/Q 函數的物理意義，面試被問不含糊
- 遇到「RL 訓練不收斂」或「reward hacking」的情境，知道從 reward shaping、探索策略、on/off-policy 選型三個方向排查
- 判斷何時 tabular Q-Learning 夠用、何時該跳到 DQN、何時 SARSA 比 Q-Learning 更安全

## 核心概念

### 八個精確定義

1. **MDP（Markov Decision Process）**：用五元組 $(S, A, P, R, \gamma)$ 形式化描述序列決策問題的數學框架。$S$ 是狀態空間、$A$ 是動作空間、$P(s'|s,a)$ 是狀態轉移機率、$R(s,a,s')$ 是獎勵函數、$\gamma \in [0,1)$ 是折扣因子。**機器人抓取範例**：$S$ = 關節角度 + 物體位姿、$A$ = 各關節力矩指令、$P$ = 物理引擎模擬的轉移、$R$ = 抓取成功 +1 / 掉落 -0.5 / 每步 -0.001、$\gamma = 0.99$。

2. **Bellman 方程**：最優性原理的遞迴表達 — 「一個最優策略的子策略也必然最優」。把長期最優分解成「這一步 + 之後最優」，是所有 value-based RL 的數學基礎。

3. **V(s) — 狀態價值函數**：從狀態 $s$ 出發、按策略 $\pi$ 行動，未來所有折扣獎勵的期望總和。「這個狀態有多值錢」。

4. **Q(s,a) — 動作價值函數**：在狀態 $s$ 做動作 $a$，之後按 $\pi$ 行動的累積獎勵期望。「在這個狀態做這個動作有多好」。**V 和 Q 的關係**：$V(s) = \max_a Q(s,a)$（最優策略下）。

5. **Policy $\pi(a|s)$**：狀態到動作的映射。**確定性策略** $\pi(s) = a$（直接給動作）vs **隨機策略** $\pi(a|s)$（給動作的機率分布）。機器人場景常用隨機策略，因為需要探索和對環境擾動的魯棒性。

6. **On-policy vs Off-policy**：On-policy（如 SARSA）用當前策略收集的資料更新當前策略 — 保守安全；Off-policy（如 Q-Learning）用任意策略收集的資料學習最優策略 — 樣本效率高但更激進。

7. **Q-Learning**：model-free、off-policy 的 RL 演算法。更新規則中用 $\max_{a'} Q(s', a')$（greedy），但行為用 $\epsilon$-greedy（探索），兩者不同所以是 off-policy。

8. **$\epsilon$-greedy 與 TD vs MC**：
   - **$\epsilon$-greedy**：以 $\epsilon$ 機率隨機探索、$(1-\epsilon)$ 機率選 Q 值最高的動作。不能純貪心（陷入局部最優），需退火衰減（$\epsilon: 1.0 \to 0.01$）
   - **TD（Temporal Difference）**：每走一步就用 bootstrapping 更新（自舉 — 用估計值更新估計值），方差低但有偏差
   - **MC（Monte Carlo）**：跑完整個 episode 再用實際 return 更新，無偏差但方差高。TD 是 Q-Learning 的核心、MC 是 REINFORCE 的核心

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：感測器觀測 → 狀態表示 $s$（joint angles、image features、object poses）
- **輸出**：策略 $\pi(a|s)$ — 在每個狀態該做什麼動作
- **下游**：動作 $a$ 送進控制器（torque command / velocity setpoint / 高層決策指令）
- **閉環節點**：屬於**決策控制層**，替代手寫 if-else 狀態機。RL 和傳統控制互補 — RL 負責高階決策（「先推開障礙物還是繞過去」），PID/MPC 負責底層追軌跡

**一句話版本**：「RL 是讓機器人在沙盒中試錯淬鍊出感測器→馬達的決策直覺。」

### 六個核心公式

1. **累積回報（Return）**：

$$
G_t = \sum_{k=0}^{\infty} \gamma^k R_{t+k+1}
$$

**物理意義**：從時刻 $t$ 開始，把未來每步獎勵乘以 $\gamma^k$ 折扣後加總。$\gamma$ 是「時間近視度」 — 接近 1 看得遠（耐心等大獎勵），接近 0 只看眼前。

2. **Bellman 最優方程 — V 版**：

$$
V^*(s) = \max_{a} \left[ R(s,a) + \gamma \sum_{s'} P(s'|s,a) \, V^*(s') \right]
$$

**物理意義**：最優狀態價值 = 選最好的動作 → 拿即時獎勵 → 加上折扣後的下一狀態最優價值。$\max$ 代表最優性原理 — 每步都做最好的選擇。

3. **Bellman 最優方程 — Q 版**：

$$
Q^*(s,a) = R(s,a) + \gamma \sum_{s'} P(s'|s,a) \max_{a'} Q^*(s',a')
$$

**物理意義**：最優 Q 值 = 即時獎勵 + 折扣後在下一狀態選最好動作的 Q 值。這個遞迴把「長期最優」拆成「一步 + 未來最優」。

4. **Q-Learning 更新規則**：

$$
Q(s,a) \leftarrow Q(s,a) + \alpha \big[ r + \gamma \max_{a'} Q(s',a') - Q(s,a) \big]
$$

**物理意義**：每做完一步 $(s,a \to r,s')$，算 TD error $= r + \gamma \max_{a'} Q(s',a') - Q(s,a)$，用學習率 $\alpha$ 修正。$\alpha$ 是步長（太大振盪、太小收斂慢），$\max$ 讓它學最優策略（off-policy），方括號是「實際值和預期值的差」。

5. **SARSA 更新規則**（對比用）：

$$
Q(s,a) \leftarrow Q(s,a) + \alpha \big[ r + \gamma Q(s',a') - Q(s,a) \big]
$$

**物理意義**：和 Q-Learning 唯一差別 — 用 $Q(s', a')$（按當前策略選的 $a'$）取代 $\max_{a'} Q(s', a')$。結果更保守安全：SARSA 在懸崖邊會繞遠路，Q-Learning 會貼著懸崖走最短路（因為它假設未來都做最優選擇）。

6. **策略改進定理（Policy Improvement Theorem）**：

$$
Q^\pi(s, \pi'(s)) \geq V^\pi(s), \; \forall s \implies V^{\pi'}(s) \geq V^\pi(s), \; \forall s
$$

**物理意義**：如果新策略 $\pi'$ 在每個狀態選的動作都不比舊策略 $\pi$ 差（Q 值 ≥ V 值），那新策略整體一定不比舊的差。這是 policy iteration 收斂的理論保證。

<details>
<summary>深入：Bellman 方程完整推導與動態規劃</summary>

**從 V 到 Q 的關係**：

$$
V^\pi(s) = \sum_{a} \pi(a|s) \, Q^\pi(s,a)
$$

$V$ 是把所有動作的 $Q$ 按策略機率加權平均。

**Bellman 期望方程**（非最優版本，用於 policy evaluation）：

$$
V^\pi(s) = \sum_{a} \pi(a|s) \left[ R(s,a) + \gamma \sum_{s'} P(s'|s,a) \, V^\pi(s') \right]
$$

展開含義：當前狀態的價值 = 按策略選動作 → 拿即時獎勵 → 轉移到新狀態 → 遞迴算新狀態的價值。

**動態規劃三部曲**（已知完整 MDP 模型時）：

1. **Policy Evaluation**：固定策略 $\pi$，用 Bellman 期望方程迭代到 $V^\pi$ 收斂
2. **Policy Improvement**：用 $\pi'(s) = \arg\max_a Q^\pi(s,a)$ 貪心改進
3. **Policy Iteration**：反覆 evaluation → improvement 直到策略不再變化

**Value Iteration** 是把 evaluation 和 improvement 合成一步：

$$
V_{k+1}(s) = \max_a \left[ R(s,a) + \gamma \sum_{s'} P(s'|s,a) \, V_k(s') \right]
$$

**Q-Learning 為什麼是 off-policy**：
- 更新用 $\max_{a'} Q(s', a')$（greedy policy）
- 行為用 $\epsilon$-greedy（探索策略）
- 兩者不同 → off-policy
- 好處：可以用任意行為收集的經驗來學習最優策略

**收斂條件**（Watkins & Dayan, 1992）：
- 每個 $(s,a)$ pair 被訪問無窮多次
- 學習率 $\alpha_t$ 滿足 $\sum \alpha_t = \infty$, $\sum \alpha_t^2 < \infty$
- 狀態和動作空間有限

**TD 與 MC 的數學對比**：

| | TD(0) | Monte Carlo |
|---|---|---|
| 更新目標 | $r + \gamma V(s')$（bootstrapping） | $G_t$（完整回報） |
| 偏差 | 有偏（依賴估計值） | 無偏（用真實回報） |
| 方差 | 低（只用一步） | 高（累積整個 episode 的隨機性） |
| 適用 | 可連續任務、不需等 episode 結束 | 需 episodic 任務 |

TD(n) 是兩者的插值 — 用 $n$ 步實際回報 + bootstrapping，trade-off 偏差和方差。

</details>

**常用 API / 工具**（業界工具鏈）：

| 層級 | 套件 | 功能 |
|------|------|------|
| RL 框架 | Stable-Baselines3 / CleanRL | 標準演算法實作（PPO, SAC, DQN 等） |
| 環境介面 | Gymnasium (OpenAI Gym) | `env.reset() → obs, info`；`env.step(action) → obs, reward, terminated, truncated, info` |
| 機器人模擬 | Isaac Gym / MuJoCo / PyBullet | 物理模擬環境，GPU 加速並行訓練 |
| 分散式訓練 | Ray RLlib / Sample Factory | 大規模並行環境 + 多 GPU 訓練 |
| Reward 工具 | gymnasium.wrappers | Reward shaping、normalization、clipping |

**超參數建議**：

| 超參數 | 表格 Q-Learning | DQN |
|--------|----------------|-----|
| 學習率 $\alpha$ | 0.01 ~ 0.1 | 1e-4 ~ 1e-3 |
| 折扣因子 $\gamma$ | 0.95 ~ 0.99 | 0.95 ~ 0.99 |
| $\epsilon$ 初始值 | 1.0 | 1.0 |
| $\epsilon$ 最終值 | 0.01 | 0.01 |
| $\epsilon$ 衰減 | 指數衰減 ×0.995/ep | 線性衰減 over 100k steps |
| Replay buffer | — | 100k ~ 1M |
| Target net 更新 | — | 每 1k~10k steps hard copy 或 $\tau=0.005$ soft |

## 直覺理解

**四個類比**：

1. **MDP = 棋盤遊戲**：棋盤就是狀態空間，每步可以下的位置是動作空間，下完一步棋盤變化是轉移機率，贏棋 +1 輸棋 -1 是 reward，$\gamma$ 是你願意為了贏棋忍受多少步的「不舒服」
2. **Q-table = 考試答案對照表**：每個格子記「在這個題目狀態下、選這個答案能拿幾分」。做多了模考就知道哪個答案得分最高
3. **$\epsilon$-greedy = 吃飯選擇**：你有常去的老餐廳（exploit），但偶爾去新店探索（explore）。$\epsilon$ 就是「試新店的機率」，開始要高（還不確定哪家好），慢慢降（已經知道哪家最好了，但偶爾還是試試新的）
4. **TD vs MC = 評分方式**：TD 像每吃完一道菜就打分（一步更新），MC 像吃完整頓飯才給總分（episode 結束才更新）。一道菜打分比較快但可能不公平（偏差），整頓打分比較公正但要等很久而且受菜色隨機性影響（方差）

**模擬器觀察**：在 Gymnasium 跑 `CartPole-v1`，用 Q-Learning 訓練。觀察：(1) 前 100 episode 杆子立刻倒（random policy）；(2) 中間開始偶爾撐住 50 步（Q-table 開始有意義的估計）；(3) 後期穩定撐滿 500 步（convergence）。把 Q-table 用 heatmap 視覺化，看到「杆子偏右 + 角速度向左 → 往左推」的模式逐漸浮現。

在 MuJoCo 裡跑機器人抓取場景，用分層稠密 reward 訓練。前 10k 步 agent 亂揮手臂，50k 步開始穩定接近物體，100k 步學會碰觸，200k 步學會夾住。每個階段的 reward 曲線會有明顯的「階梯式」上升。

## 實作連結

**三個典型工程場景**：

1. **機器人抓取的 reward shaping**：直接給「抓到/沒抓到」的 0/1 reward 太稀疏，agent 學不到東西。實務用分層稠密 reward：接近物體 +0.1、碰到 +0.3、夾住 +0.5、抬起 +1.0、每步 -0.001（鼓勵快速完成）+ 安全獎勵（碰撞 -0.5）。

2. **導航避障（離散）**：移動機器人在格子世界用 Q-Learning 走迷宮。狀態 = 格子座標，動作 = 上下左右，撞牆 -1、到終點 +10。這是 tabular Q-Learning 的經典場景，狀態空間小（< 10000）。

3. **連續空間的跳板**：6 DoF joint angles → 狀態空間無窮大 → Q-table 放不下（維度災難）。用 neural network 逼近 Q 函數（DQN）。關鍵改動：Experience Replay（打破時序相關性）+ Target Network（穩定訓練）→ 這就是 Ch19 的主題。

**Code 骨架**（Python，tabular Q-Learning + Gymnasium）：

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
        # ε-greedy 選動作
        if np.random.random() < epsilon:
            action = env.action_space.sample()       # 探索
        else:
            action = np.argmax(Q[state])              # 利用
        next_state, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated
        # Q-Learning 核心更新
        # 注意（Gymnasium API）：bootstrap mask 只用 `terminated`（真正吸收態，V(s')=0），
        # 不可用 `truncated`。time-limit 截斷（如 CartPole 500 步上限）時 MDP 仍在繼續，
        # 要用 V(s') 繼續 bootstrap；否則會系統性低估靠近截斷處的狀態值。
        td_target = reward + gamma * np.max(Q[next_state]) * (not terminated)
        Q[state, action] += alpha * (td_target - Q[state, action])
        state = next_state
    epsilon = max(0.01, epsilon * 0.995)  # 指數退火
```

<details>
<summary>深入：完整 Q-Learning + DQN Python 實作</summary>

**完整 tabular Q-Learning（含訓練監控與視覺化）**：

```python
import numpy as np
import gymnasium as gym
from collections import defaultdict

def train_q_learning(env_name="FrozenLake-v1", episodes=10000,
                     alpha=0.1, gamma=0.99, eps_start=1.0,
                     eps_end=0.01, eps_decay=0.995):
    """完整 tabular Q-Learning with logging"""
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
            # ε-greedy
            if np.random.random() < epsilon:
                action = env.action_space.sample()
            else:
                action = np.argmax(Q[state])

            next_state, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            # Q-Learning update
            # bootstrap mask 只用 `terminated`，不用 `truncated`（time-limit 截斷時 MDP 仍在繼續）
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


# SARSA 對比實作（on-policy 版本）
def train_sarsa(env_name="CliffWalking-v0", episodes=5000,
                alpha=0.1, gamma=0.99, eps_start=1.0,
                eps_end=0.01, eps_decay=0.995):
    """SARSA — on-policy, 在懸崖環境比 Q-Learning 更安全"""
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

            # SARSA update: 用 Q(s', a') 而非 max Q(s', ·)
            # bootstrap mask 只用 `terminated`，不用 `truncated`（time-limit 截斷時 MDP 仍在繼續）
            td_error = reward + gamma * Q[next_state, next_action] * (not terminated) - Q[state, action]
            Q[state, action] += alpha * td_error

            state, action = next_state, next_action

        epsilon = max(eps_end, epsilon * eps_decay)

    return Q
```

**DQN 完整實作（PyTorch）**：

```python
import torch
import torch.nn as nn
import numpy as np
from collections import deque
import random
import gymnasium as gym


class QNetwork(nn.Module):
    """用神經網路取代 Q-table，輸入 state → 輸出每個 action 的 Q 值"""
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
    """經驗回放：打破時序相關性，讓訓練更穩定"""
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

    q_net = QNetwork(state_dim, action_dim)        # 線上網路
    target_net = QNetwork(state_dim, action_dim)    # 目標網路
    target_net.load_state_dict(q_net.state_dict())

    optimizer = torch.optim.Adam(q_net.parameters(), lr=lr)
    buffer = ReplayBuffer(buffer_size)
    epsilon = eps_start

    for ep in range(episodes):
        state, _ = env.reset()
        total_reward = 0
        done = False

        while not done:
            # ε-greedy
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

            # 經驗夠多才開始訓練
            if len(buffer) >= batch_size:
                s, a, r, ns, d = buffer.sample(batch_size)
                # 當前 Q 值
                current_q = q_net(s).gather(1, a.unsqueeze(1)).squeeze()
                # 目標 Q 值（用 target network 算，更穩定）
                with torch.no_grad():
                    max_next_q = target_net(ns).max(1)[0]
                    target_q = r + gamma * max_next_q * (1 - d)
                # TD loss + gradient clip
                loss = nn.MSELoss()(current_q, target_q)
                optimizer.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(q_net.parameters(), max_norm=1.0)
                optimizer.step()

        # 定期更新目標網路
        if ep % target_update == 0:
            target_net.load_state_dict(q_net.state_dict())
        epsilon = max(eps_end, epsilon * eps_decay)

        if (ep + 1) % 50 == 0:
            print(f"Episode {ep+1}: reward={total_reward:.1f}, eps={epsilon:.3f}")

    return q_net
```

**Q-table → DQN 的三個關鍵改動**：
1. **Neural Network**：用函數逼近取代表格，處理高維 / 連續狀態空間
2. **Experience Replay**：把 $(s,a,r,s')$ 存起來隨機抽樣，打破時序相關性
3. **Target Network**：用延遲更新的網路算 target Q，避免「自己追自己的尾巴」的不穩定

</details>

## 常見誤解

1. **「RL 直接在真機上訓練就好」** — RL 訓練需要百萬到十億步試錯，真機會被摔壞、電機燒毀、人員受傷。**正確做法**：先在模擬器（MuJoCo / Isaac Gym）訓練到收斂，再用 Domain Randomization + System ID 做 sim-to-real transfer。只有最後微調才碰真機。

2. **「Q-table 能處理任何狀態空間」** — Q-table 大小 $|S| \times |A|$。六個連續 joint angles → 狀態空間無窮大 → 表格根本存不下，這就是**維度災難（curse of dimensionality）**。**解法**：用 neural network 逼近 Q 函數（DQN），把表格查詢變函數求值。

3. **「稀疏 reward 配大量訓練就能收斂」** — 如果 reward 只在最後成功/失敗給分，隨機探索幾乎不可能偶然觸發成功。agent 收到的全是 0，Q-table/Q-network 完全沒有梯度訊號。**解法**：分層稠密 reward（每一步都有訊號） + HER（Hindsight Experience Replay）。

4. **「Q-Learning 永遠比 SARSA 好」** — Q-Learning 學最優策略（$\max$），SARSA 學當前策略的 Q 值。在有安全約束的場景（懸崖、碰撞），SARSA 因為考慮了探索行為的風險，會學出更保守更安全的路徑。**經典例子**：CliffWalking 環境中 SARSA 遠離懸崖走安全路線、Q-Learning 貼著懸崖走最短路。

5. **「$\gamma = 1$ 就能看到最遠的未來」** — $\gamma = 1$ 會讓 return 變成無窮級數、不一定收斂，value function 可能發散到無窮大。**安全範圍**：$\gamma \in [0.95, 0.99]$。$\gamma = 0.99$ 的**有效視野**標準定義是 $1/(1-\gamma) = 100$ 步；另一常用啟發式是「折扣衰減到 $1/e$（$\approx 37\%$）的步數」，$\gamma = 0.99$ 恰好也約 100 步（$0.99^{100} \approx 0.37$）。兩者都定義合理，但別混用。

## 練習題

<details>
<summary>Q1（中）：你在訓練 6-DoF 機械臂抓取物體，reward 該怎麼設計？</summary>

**完整推理鏈**：

1. **核心問題**：抓取成功是最終目標，但直接用 0/1 sparse reward 不可能收斂 — 六軸手臂的連續狀態-動作空間太大，隨機探索幾乎碰不到物體
2. **分層稠密 reward 設計**：
   - 末端接近物體：$r_{\text{dist}} = -\| p_{\text{ee}} - p_{\text{obj}} \|$（FK 算末端位置，距離越小 reward 越高）
   - 碰到物體：$+0.3$（接觸感測器觸發）
   - 夾住物體：$+0.5$（夾爪力 > 閾值且物體未滑落）
   - 抬起到目標高度：$+1.0$（成功抓取）
   - 安全項：碰撞桌面 $-0.5$，關節接近極限 $-0.2$
   - 效率項：每步 $-0.001$（鼓勵快速完成）
3. **陷阱**：如果只用距離 reward 而不加安全懲罰，agent 可能學到高速衝撞物體（reward hacking）
4. **進階方案**：加入 CBF（Control Barrier Function）約束確保安全、或用 HER 讓失敗經驗也有學習價值

**面試官想聽到**：分層稠密 reward 的具體數值設計 + 安全約束 + 稀疏 reward 的替代方案（HER / Curriculum Learning）。

</details>

<details>
<summary>Q2（中-難）：你在 grid world 上訓練 Q-Learning 成功了，但換到 6-DoF 機器人控制就完全崩潰，原因是什麼？怎麼過渡？</summary>

**完整推理鏈**：

1. **診斷：維度災難**：grid world 狀態空間 100 個格子 × 4 動作 = 400 格 Q-table。6-DoF 機械臂每個關節角是連續值（如 -π ~ π），即使離散化為 100 份 → $100^6 = 10^{12}$ 格，完全不可行
2. **根本矛盾**：tabular Q-Learning 假設每個 $(s,a)$ 都能被訪問足夠多次，連續空間違反這個假設
3. **解法路徑**：
   - **DQN**：用 neural network 逼近 $Q(s,a)$，輸入 state vector → 輸出各 action 的 Q 值。但動作空間也是連續的 → DQN 只能處理離散動作
   - **DDPG / SAC**：actor-critic 架構，actor 輸出連續動作、critic 評估 Q 值，兩個網路交替訓練。SAC 加 entropy regularization 更穩定
   - **離散化折衷**：把連續動作粗離散化（如每軸 $\{-1, 0, +1\}$ 力矩）→ $3^6 = 729$ 個離散動作，先用 DQN 跑概念驗證
4. **工程驗證**：先在 MuJoCo 簡化場景（如 Reacher-v4，只有 2 DOF）確認演算法 work，再逐步提升到 6 DoF

**面試官想聯到**：從維度災難出發、過渡到函數逼近、再到 actor-critic 架構的思路清晰。

</details>

<details>
<summary>Q3（難）：你的導航 agent 學會了撞牆走捷徑（reward hacking），而不是好好避障到達目標，怎麼修？</summary>

**完整推理鏈**：

1. **根因分析**：reward 有漏洞 — 可能 reward 只看「到目標的距離減少」，撞穿牆反而距離大幅減少，拿到高 reward
2. **驗證**：錄 agent 行為影片 + 分解 reward 各項分別畫圖，找出哪一項被 hack
3. **解法層級**：
   - **Level 1 — 修 reward**：加碰撞懲罰 $r_{\text{collision}} = -1.0$，確保任何穿牆行為的淨 reward 都是負的
   - **Level 2 — 加物理約束**：用 CBF（Control Barrier Function）硬性約束 agent 不能進入障礙物區域，reward 只管任務
   - **Level 3 — 環境修正**：確保模擬器的碰撞檢測正確（有些簡易環境允許穿牆），加大碰撞反彈力
   - **Level 4 — 學 reward**：用 IRL 從人類示範反推 reward，或 RLHF 從人類偏好學 reward model
4. **預防**：每 1000 episode 自動錄影 + behavior sanity check（「agent 穿越了不可達區域」→ 自動告警）

**面試官想聽到**：reward hacking 的根因是 reward 和真實目標不一致（alignment problem 的縮影）；知道從修 reward → 加約束 → 學 reward 的逐步升級路線。

</details>

<details>
<summary>Q4（中）：在 CliffWalking 環境中，SARSA 和 Q-Learning 學出的路徑有什麼不同？為什麼？</summary>

**完整推理鏈**：

1. **環境描述**：4×12 grid，底部是懸崖（掉下去 reward = -100、回到起點），每步 reward = -1，目標在右下角
2. **Q-Learning（off-policy）學出的路徑**：沿著懸崖邊走最短路。因為它用 $\max_{a'} Q(s',a')$ 更新，假設未來永遠做最優選擇 → 不考慮 $\epsilon$-greedy 探索時不小心掉下懸崖的風險
3. **SARSA（on-policy）學出的路徑**：遠離懸崖邊、走上面的安全路線。因為它用 $Q(s',a')$（實際按 $\epsilon$-greedy 選的動作）更新，把探索時掉下懸崖的「偶然」負 reward 也算進去了 → 靠近懸崖的狀態 Q 值被拉低
4. **核心差異**：Q-Learning 學的是「如果我永遠都做對了」的最優路徑；SARSA 學的是「考慮到我有時候會犯錯」的安全路徑
5. **選型啟示**：真機環境（摔壞代價高）→ SARSA/on-policy 更安全；純模擬環境（試錯免費）→ Q-Learning/off-policy 更高效

**面試官想聽到**：清楚 on/off-policy 的本質差異不只是演算法技巧，而是「是否考慮探索風險」的安全性問題。

</details>

## 面試角度

1. **Bellman 方程的遞迴直覺** — 面試必問的基礎。**帶出**：「Bellman 方程的核心是最優性原理：如果你的整體策略是最優的，那從任何中間狀態開始的子策略也必須是最優的。所以最優 V 值可以拆成『這一步最好的 reward + 折扣後下一步的最優 V 值』，這個遞迴是所有 value-based RL 的基石。」

2. **On-policy vs Off-policy 的安全性差異** — 區分「跑過 tutorial」和「真正理解選型」。**帶出**：「Q-Learning 用 $\max$ 假設未來永遠做最優選擇，所以會走懸崖邊的最短路。SARSA 把探索時的犯錯風險算進 Q 值，所以繞遠路但更安全。真機場景我會先考慮 on-policy 方法，因為試錯代價太高。」

3. **探索利用的退火設計** — 展示實務調參經驗。**帶出**：「$\epsilon$-greedy 退火不是線性就好。我通常用指數衰減 $\epsilon \times 0.995$，但關鍵是根據 replay buffer 的經驗量而非 episode 數衰減 — 經驗不夠多就不急著降 $\epsilon$，避免過早收斂到局部最優。另外 Boltzmann exploration（softmax）在動作 Q 值差異小時比 $\epsilon$-greedy 更精細。」

4. **Q-table → DQN 的動機與關鍵改動** — 串接基礎和深度 RL。**帶出**：「Q-table 在離散小空間 work，但狀態空間一連續就爆炸（維度災難）。DQN 用 neural network 逼近 Q 函數，但直接訓練不穩定 — 所以需要 Experience Replay（打破時序相關）和 Target Network（穩定 bootstrap target）。這兩個 trick 是 DQN 的靈魂。」

5. **Reward shaping 是 RL 的藝術** — 展示工程深度。**帶出**：「RL 工程 80% 時間在設計 reward。好的 reward 三原則：稠密（每步有訊號）、單調（越接近目標越高）、無漏洞（不存在比完成任務更容易刷分的捷徑）。設計完 reward 我一定用 random policy 跑一次，確認隨機動作的 reward 分布合理 — 如果 random 也能拿高分，那 reward 一定有漏洞。」

## 延伸閱讀

- **Sutton & Barto,《Reinforcement Learning: An Introduction》(2nd ed.)** — RL 聖經，Ch3-6 覆蓋 MDP、Dynamic Programming、TD Learning；免費線上版，是面試準備的最佳單一資源
- **《具身智能算法工程師 面試題》Ch4.1 MDP 基礎、Ch4.2 Q-Learning 與 DQN** — 面試核心考點，Bellman 方程推導 + tabular vs function approximation 的完整比較
- **《多模態大模型》Ch2.4.2 RL 與 RLHF** — 了解 RL 如何和大模型結合，reward model 訓練的現代方法
- **Andrychowicz et al.,《Hindsight Experience Replay》(NeurIPS 2017)** — 稀疏獎勵的經典解法，對機器人抓取場景影響深遠
- **Curriculum Learning for RL** — 從簡單任務逐步增加難度，加速收斂且提升最終性能
- **Multi-step TD / TD($\lambda$)** — TD 和 MC 的統一框架，$\lambda = 0$ 是純 TD、$\lambda = 1$ 是純 MC，實務常用 $\lambda = 0.95$
- **Double Q-Learning（Hasselt 2010）+ Prioritized Experience Replay（Schaul 2015）** — 解決 Q 值高估和加速學習的兩個標準 trick，DQN 的必備升級
- **CleanRL** — 單檔案 RL 演算法實作，每個演算法一個 Python 檔，讀源碼理解細節最快
