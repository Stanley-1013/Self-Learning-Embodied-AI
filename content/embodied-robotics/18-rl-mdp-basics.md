---
title: "強化學習基礎：MDP 與 Q-Learning"
prerequisites: []
estimated_time: 50
difficulty: 3
tags: ["reinforcement-learning", "mdp", "q-learning", "reward"]
sidebar_position: 18
---

# 強化學習基礎：MDP 與 Q-Learning

## 你將學到

- 能用兩句話講清楚 MDP 五元組和 Bellman 方程的物理意義，面試被問不含糊
- 遇到「RL 訓練不收斂」或「reward hacking」的情境，知道從 reward shaping、探索策略、經驗回放三個方向排查
- 判斷何時 tabular Q-Learning 夠用、何時該跳到 DQN、何時該改用 policy gradient

## 核心概念

**精確定義**：**強化學習（Reinforcement Learning, RL）** 是 agent 透過與環境互動，根據獎勵訊號學習最優行為策略的框架。**MDP（Markov Decision Process）** 是 RL 的數學基礎，用五元組 $(S, A, P, R, \gamma)$ 形式化描述一個序列決策問題。**Q-Learning** 是最經典的 model-free、off-policy RL 演算法，透過迭代更新 Q-table 來逼近最優 action-value function。

**MDP 五元組**：
- $S$：狀態空間（state space）— 環境所有可能的狀態
- $A$：動作空間（action space）— agent 可以執行的動作
- $P(s' \mid s, a)$：狀態轉移機率 — 在狀態 $s$ 做動作 $a$ 後到達 $s'$ 的機率
- $R(s, a, s')$：獎勵函數 — 每一步的即時回饋訊號
- $\gamma \in [0, 1)$：折扣因子 — 權衡當下獎勵和未來獎勵的重要性

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：感測器觀測 → 狀態表示 $s$（可以是 joint angles、image features、object poses）
- **輸出**：策略 $\pi(a \mid s)$ — 在每個狀態該做什麼動作
- **下游**：動作 $a$ 送進控制器（torque command / velocity setpoint / 高層決策指令）
- **閉環節點**：屬於**決策控制層**，替代手寫的 if-else 狀態機。RL 的核心價值是**讓機器人從經驗中自動發現最優行為**，而非人工設計規則

**一句話版本**：「給機器人一個目標和獎懲規則，讓它自己試錯摸索出最好的做法 — 像訓練小狗用零食和口令。」

**最少夠用的數學**：

1. **Value Function（狀態價值函數）**：

$$
V^\pi(s) = \mathbb{E}_\pi \left[ \sum_{t=0}^{\infty} \gamma^t R_t \mid S_0 = s \right]
$$

**物理意義**：從狀態 $s$ 開始，按策略 $\pi$ 行動，未來所有折扣獎勵的期望總和。「這個狀態有多值錢」— 值越大代表從這裡出發能拿到越多獎勵。

2. **Action-Value Function（Q 函數）**：

$$
Q^\pi(s, a) = \mathbb{E}_\pi \left[ \sum_{t=0}^{\infty} \gamma^t R_t \mid S_0 = s, A_0 = a \right]
$$

**物理意義**：從狀態 $s$ 做動作 $a$，之後按 $\pi$ 行動的累積獎勵期望。「在這個狀態做這個動作有多好」— Q 值最高的動作就是當下最佳選擇。

3. **Bellman 最優方程**（RL 的核心遞迴關係）：

$$
Q^*(s, a) = \mathbb{E} \left[ R + \gamma \max_{a'} Q^*(s', a') \right]
$$

**物理意義**：最優 Q 值 = 即時獎勵 + 折扣後的下一步最優 Q 值。這個遞迴把「長期最優」分解成「當下一步 + 未來最優」，是所有 Q-Learning 系列演算法的數學基礎。

4. **Q-Learning 更新規則**（表格版，最簡形式）：

$$
Q(s, a) \leftarrow Q(s, a) + \alpha \left[ r + \gamma \max_{a'} Q(s', a') - Q(s, a) \right]
$$

**物理意義**：每次 agent 做完一步（$s, a \rightarrow r, s'$），用實際拿到的獎勵 $r$ 加上對未來的最優估計，和當前 Q 值的差（TD error）乘以學習率 $\alpha$ 來修正。像是「每次交易完覆盤，稍微更新自己對這個動作的評價」。

<details>
<summary>深入：Bellman 方程的完整推導與 V/Q 的關係</summary>

**從 V 到 Q 的關係**：

$$
V^\pi(s) = \sum_{a} \pi(a \mid s) Q^\pi(s, a)
$$

$V$ 是把所有動作的 $Q$ 按策略機率加權平均。

**Bellman 期望方程**（非最優版本，用於 policy evaluation）：

$$
V^\pi(s) = \sum_{a} \pi(a \mid s) \left[ R(s, a) + \gamma \sum_{s'} P(s' \mid s, a) V^\pi(s') \right]
$$

展開含義：當前狀態的價值 = 按策略選動作 → 拿即時獎勵 → 轉移到新狀態 → 遞迴算新狀態的價值。

**Bellman 最優方程**（目標是找最優策略）：

$$
V^*(s) = \max_{a} \left[ R(s, a) + \gamma \sum_{s'} P(s' \mid s, a) V^*(s') \right]
$$

$$
Q^*(s, a) = R(s, a) + \gamma \sum_{s'} P(s' \mid s, a) \max_{a'} Q^*(s', a')
$$

**Q-Learning 為什麼是 off-policy**：
- 更新用 $\max_{a'} Q(s', a')$（greedy policy）
- 行為用 $\epsilon$-greedy（探索策略）
- 兩者不同 → off-policy
- 好處：可以用任意行為收集的經驗來學習最優策略

**收斂條件**（Watkins & Dayan, 1992）：
- 每個 $(s, a)$ pair 被訪問無窮多次
- 學習率 $\alpha_t$ 滿足 $\sum \alpha_t = \infty$, $\sum \alpha_t^2 < \infty$（如 $\alpha_t = 1/t$）
- 狀態和動作空間有限

實務上用固定學習率 + 足夠多的 episode 就能收斂到近似最優。

</details>

**常用 API**（業界工具鏈）：

| 層級 | 套件 | 功能 |
|------|------|------|
| RL 框架 | Stable-Baselines3 / CleanRL | 標準演算法實作（PPO, SAC, DQN 等） |
| 環境介面 | Gymnasium (OpenAI Gym) | 統一的 `reset() → step(action) → obs, reward, done` 介面 |
| 機器人模擬 | Isaac Gym / MuJoCo / PyBullet | 物理模擬環境，GPU 加速並行訓練 |
| 分散式訓練 | Ray RLlib / Sample Factory | 大規模並行環境 + 多 GPU 訓練 |
| Reward 工具 | gymnasium.wrappers | Reward shaping、normalization、clipping |

## 直覺理解

**類比：訓練小狗撿球**。你想教小狗把球撿回來。小狗（agent）在公園（environment）裡。每次牠做了動作（跑、咬、叼回來），你給零食或口頭獎勵（reward）。一開始小狗亂跑（exploration），但慢慢學到「跑向球 → 咬住 → 叼回來 → 拿到最多零食」。$\gamma$ 是小狗的耐心 — $\gamma$ 接近 1 表示牠願意為了更大的遠期獎勵忍住不立刻吃零食；$\gamma$ 接近 0 表示牠只看眼前。

**視覺比喻：填數獨的策略**：
- 每個空格的狀態 = 目前棋盤上所有數字
- 動作 = 在某空格填某數字
- 獎勵 = 填完整張不矛盾就得高分，中途矛盾就罰
- Q-Learning 就是記錄「在這個棋盤狀態下，填這個數字平均能拿幾分」，經驗夠多就知道最佳策略

**模擬器觀察**：在 Gymnasium 跑 `CartPole-v1`，用 Q-Learning 訓練。觀察：(1) 前 100 episode 杆子立刻倒（random policy）；(2) 中間開始偶爾撐住 50 步（Q-table 開始有意義的估計）；(3) 後期穩定撐滿 500 步（convergence）。把 Q-table 用 heatmap 視覺化，看到「杆子偏右 + 角速度向左 → 往左推」的模式逐漸浮現。

## 實作連結

**三個典型工程場景**：

1. **機器人抓取的 reward shaping**：直接給「抓到/沒抓到」的 0/1 reward 太稀疏，agent 學不到東西（到處亂動都是 0）。實務上用分層稠密 reward：接近物體 +0.1、碰到物體 +0.3、夾住 +0.5、抬起 +1.0。每一步都有梯度訊號引導 agent 往對的方向走。

2. **導航避障（離散動作空間）**：移動機器人在格子世界裡用 Q-Learning 學走迷宮。狀態 = 格子座標，動作 = 上下左右，撞牆 -1、到終點 +10、每步 -0.01（鼓勵走捷徑）。這是 tabular Q-Learning 的經典場景，狀態空間夠小（< 10000）Q-table 放得下。

3. **連續狀態空間的跳板**：當狀態空間變大（如 joint angles 的連續值），Q-table 放不下 → 用 neural network 逼近 Q 函數（DQN）。關鍵改動：加 experience replay buffer（打破時序相關性）+ target network（穩定訓練）。

**Code 骨架**（Python，tabular Q-Learning）：

```python
import numpy as np
import gymnasium as gym

env = gym.make("FrozenLake-v1", is_slippery=False)
n_states = env.observation_space.n   # 狀態數
n_actions = env.action_space.n       # 動作數

Q = np.zeros((n_states, n_actions))  # Q-table 初始化
alpha = 0.1    # 學習率
gamma = 0.99   # 折扣因子
epsilon = 1.0  # 探索率（ε-greedy）

for episode in range(10000):
    state, _ = env.reset()
    done = False
    while not done:
        # ε-greedy 選動作
        if np.random.random() < epsilon:
            action = env.action_space.sample()     # 探索
        else:
            action = np.argmax(Q[state])            # 利用

        next_state, reward, terminated, truncated, _ = env.step(action)
        done = terminated or truncated

        # Q-Learning 更新
        td_target = reward + gamma * np.max(Q[next_state]) * (not terminated)
        Q[state, action] += alpha * (td_target - Q[state, action])

        state = next_state
    epsilon = max(0.01, epsilon * 0.995)  # 探索率退火
```

<details>
<summary>深入：完整 DQN 實作骨架（PyTorch）— 從 Q-table 到神經網路的關鍵跳躍</summary>

```python
import torch
import torch.nn as nn
import numpy as np
from collections import deque
import random

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


def train_dqn(env, episodes=1000, batch_size=64, gamma=0.99, lr=1e-3):
    state_dim = env.observation_space.shape[0]
    action_dim = env.action_space.n

    q_net = QNetwork(state_dim, action_dim)        # 線上網路
    target_net = QNetwork(state_dim, action_dim)    # 目標網路（穩定訓練）
    target_net.load_state_dict(q_net.state_dict())

    optimizer = torch.optim.Adam(q_net.parameters(), lr=lr)
    buffer = ReplayBuffer()
    epsilon = 1.0

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
            if len(buffer.buffer) >= batch_size:
                s, a, r, ns, d = buffer.sample(batch_size)
                # 當前 Q 值
                current_q = q_net(s).gather(1, a.unsqueeze(1)).squeeze()
                # 目標 Q 值（用 target network 算，更穩定）
                with torch.no_grad():
                    max_next_q = target_net(ns).max(1)[0]
                    target_q = r + gamma * max_next_q * (1 - d)
                # TD loss
                loss = nn.MSELoss()(current_q, target_q)
                optimizer.zero_grad()
                loss.backward()
                optimizer.step()

        # 每 10 episode 更新目標網路
        if ep % 10 == 0:
            target_net.load_state_dict(q_net.state_dict())
        epsilon = max(0.01, epsilon * 0.995)
```

**Q-table → DQN 的三個關鍵改動**：
1. **Neural Network**：用函數逼近取代表格，處理高維/連續狀態空間
2. **Experience Replay**：把過去的經驗 $(s, a, r, s')$ 存起來隨機抽樣訓練，打破時序相關性
3. **Target Network**：用一個延遲更新的網路算 target Q，避免「自己追自己的尾巴」的不穩定

</details>

<details>
<summary>深入：On-policy vs Off-policy 的本質差異與選型</summary>

| 面向 | On-policy（如 SARSA, PPO） | Off-policy（如 Q-Learning, SAC） |
|------|---------------------------|--------------------------------|
| **行為策略** | 用來收集資料的策略 = 正在學的策略 | 收集資料的策略 ≠ 正在學的策略 |
| **更新目標** | $Q(s', a')$，$a'$ 按當前策略選 | $\max_{a'} Q(s', a')$，和行為策略無關 |
| **樣本效率** | 低（用完就丟） | 高（可以反覆用 replay buffer 裡的舊經驗） |
| **穩定性** | 較穩定 | 需要 target network 等技巧穩定 |
| **適用場景** | 連續控制（PPO/TRPO） | 離散動作（DQN）/ 連續控制（SAC） |

**實務選型指南**：
- 離散動作 + 狀態空間小 → tabular Q-Learning
- 離散動作 + 狀態空間大 → DQN / Double DQN / Dueling DQN
- 連續動作 + 需要穩定訓練 → PPO（on-policy 但實作簡單）
- 連續動作 + 需要樣本效率 → SAC（off-policy + entropy regularization）
- 真機訓練（樣本極貴）→ 優先 off-policy（SAC）+ sim-to-real

</details>

## 常見誤解

1. **「RL 直接在真機上訓練就好」** — RL 訓練需要百萬到十億步的試錯，真機會被摔壞、電機燒毀、人員受傷。**正確做法**：先在模擬器（MuJoCo / Isaac Gym）訓練到收斂，再用 Domain Randomization + System ID 做 sim-to-real transfer。只有微調最後幾步才用真機。

2. **「Q-table 能解所有問題」** — Q-table 的大小是 $|S| \times |A|$。狀態空間一旦連續或高維（如 joint angles 6 個浮點數 → 狀態空間無窮大），表格根本存不下，這就是**維度災難（curse of dimensionality）**。**跳板**：用 neural network 逼近 Q 函數（DQN），把表格查詢變成函數求值。

3. **「Reward 隨便設就好、agent 自己會學到正確行為」** — Reward 設計是 RL 最難的部分。稀疏 reward（只在最後成功/失敗給分）導致 agent 根本收到不了梯度訊號，不收斂。設計不當的 reward 會導致 **reward hacking**：agent 找到鑽規則漏洞的捷徑（例如用旋轉代替前進來刷 reward）。**避開**：用分層稠密 reward + 能量懲罰項 + 定期檢查 agent 行為的影片。

## 練習題

<details>
<summary>Q1（中）：你在訓練機器人抓取物體，reward 設成「抓到 +1、沒抓到 0」，訓練了 100 萬步完全沒進展，怎麼排查？</summary>

**完整推理鏈**：

1. **診斷：reward 太稀疏**：在百萬維的聯合狀態-動作空間裡，隨機探索幾乎不可能偶然抓到物體。agent 從頭到尾只收到 0 的 reward，Q-table/Q-network 沒有任何有意義的梯度訊號
2. **解法 1 — 分層稠密 reward**：
   - 手臂接近物體 → +0.1（距離 reward，用 FK 算末端到物體距離）
   - 碰到物體 → +0.3
   - 夾住（接觸力 > 閾值）→ +0.5
   - 抬起到目標高度 → +1.0
   - 每步加 -0.001 時間懲罰（鼓勵快速完成）
3. **解法 2 — Hindsight Experience Replay (HER)**：即使沒達到原始目標，把實際到達的位置當作「假裝的目標」存進 replay buffer，這樣每一步都有正 reward 的經驗可以學
4. **解法 3 — 課程學習（Curriculum Learning）**：先從物體放在手旁邊開始訓練（簡單），成功率 > 80% 後逐漸拉遠物體距離
5. **驗證**：畫 reward curve，確認平均 reward 在 10 萬步內開始上升；看 agent 行為影片確認動作有意義（不是在原地抖動）

**面試官想聽到**：不是只說「改 reward」，而是知道分層稠密 reward 的具體設計方法 + HER 的原理 + 課程學習的思路。

</details>

<details>
<summary>Q2（中-難）：你用 DQN 訓練導航 agent，訓練曲線先上升後突然崩潰（catastrophic forgetting），怎麼處理？</summary>

**完整推理鏈**：

1. **診斷根因**：DQN 的 Q-network 用自己的預測當 target 來更新自己（bootstrapping），如果 target 不穩定，更新會放大誤差 → 正回饋迴路 → Q 值爆炸 → 策略崩潰
2. **檢查清單**：
   - Replay buffer 夠大嗎？太小 → 最近的經驗覆蓋舊經驗 → 忘記早期學到的東西
   - Target network 更新頻率對嗎？太頻繁 → target 不穩定；太慢 → 學習太慢
   - 學習率太大？→ 步長過大跳過最優解
3. **解法**：
   - **Double DQN**：用 online network 選動作、target network 算 Q 值，避免 Q 值高估
   - **加大 replay buffer**（100k → 1M），確保舊經驗不被覆蓋太快
   - **Soft update**：`target_params = τ * online_params + (1-τ) * target_params`，τ = 0.005，比硬複製更平滑
   - **Gradient clipping**：`torch.nn.utils.clip_grad_norm_(params, max_norm=1.0)`，防止梯度爆炸
4. **監控**：畫 Q 值分布圖（不只是 reward curve），Q 值如果持續單調上升而 reward 停滯/下降 → 確認是 Q 值高估問題

**面試官想聽到**：知道 DQN 不穩定的根因是 bootstrapping + function approximation 的「deadly triad」；能講出 Double DQN、soft update、gradient clipping 的具體措施。

</details>

<details>
<summary>Q3（難）：你發現 RL agent 學會了一種奇怪的行為 — 在原地旋轉刷 reward（reward hacking），不去完成真正的任務，怎麼修？</summary>

**完整推理鏈**：

1. **根因分析**：reward 函數有漏洞。例如 reward 包含「移動距離」項，agent 發現原地旋轉也算「移動」（角速度產生的虛假位移），比費力走到目標更容易拿分
2. **驗證**：錄 agent 行為影片，人工檢查；把 reward 各項分解（距離 reward、時間懲罰、姿態 reward）分別畫圖，找到哪一項被 hack
3. **解法層級**：
   - **Level 1：修 reward**：把旋轉的虛假位移去掉，只算末端到目標的 Euclidean 距離差 $\Delta d = d_{t-1} - d_t$
   - **Level 2：加約束懲罰**：加能量消耗懲罰 $-\lambda \| \tau \|^2$（$\tau$ = joint torques），旋轉耗能高自然就不划算
   - **Level 3：用 IRL（Inverse RL）**：從人類示範中反推 reward 函數，避免手寫 reward 的漏洞
   - **Level 4：RLHF（RL from Human Feedback）**：讓人類對 agent 行為做偏好排序，用偏好學習 reward model
4. **預防性措施**：訓練過程中定期（每 1000 episode）自動錄 agent 行為影片，設 behavior sanity check（如「平均前進距離 < 0.1 m 但 reward 還在漲」→ 自動告警）

**面試官想聽到**：reward hacking 是 RL 工程中最常見的坑之一；知道從「修 reward」到「學 reward」的三個層次；強調「一定要看 agent 行為影片，不能只看 reward curve」。

</details>

## 面試角度

1. **探索 vs 利用（Exploration vs Exploitation）的退火策略** — 區分「跑了一下 tutorial」和「真正調過 RL」的分水嶺。**帶出**：「$\epsilon$-greedy 的退火排程很關鍵。我通常從 $\epsilon = 1.0$ 線性退到 0.01，但不是靠 episode 數退火，而是靠 replay buffer 裡的經驗量 — 經驗不夠多就不急著降 $\epsilon$，避免過早收斂到局部最優。」

2. **Hindsight Experience Replay (HER) 解稀疏獎勵** — 展示對 RL 工程技巧的深度理解。**帶出**：「稀疏 reward 不收斂時，我的第一反應不是改 reward，而是用 HER — 把失敗經驗裡實際到達的位置當作虛擬目標存進 buffer，這樣不用修 reward 就能讓 agent 從失敗中學到有用的東西。」

3. **Value-based vs Policy Gradient 的選型** — 面試高頻題。**帶出**：「離散動作用 DQN 系列（value-based），連續控制用 SAC 或 PPO（policy gradient）。但真正的選型考量是樣本效率 vs 穩定性：off-policy（SAC）樣本效率高但調參敏感，on-policy（PPO）穩定但需要大量環境互動。真機訓練優先 SAC，模擬器大規模並行用 PPO。」

4. **Reward shaping 的工程原則** — RL 實務中 80% 的時間花在 reward 設計。**帶出**：「好的 reward 有三個原則：(1) 稠密 — 每一步都有訊號；(2) 單調 — 越接近目標 reward 越高；(3) 無漏洞 — 不存在比完成任務更容易刷分的捷徑。我在設計完 reward 後一定用 random policy 跑一次，確認隨機動作的 reward 分布合理。」

## 延伸閱讀

- **《具身智能算法工程師 面試題》Ch4.1 MDP 基礎、Ch4.2 Q-Learning 與 DQN** — RL 面試核心考點，Bellman 方程推導 + 表格法 vs 函數逼近的完整比較
- **Sutton & Barto,《Reinforcement Learning: An Introduction》(2nd ed.)** — RL 聖經，Ch3-6 覆蓋 MDP 到 TD Learning 的所有基礎；免費線上版
- **Mnih et al.,《Human-level control through deep reinforcement learning》(Nature 2015)** — DQN 開山論文，理解 experience replay + target network 為什麼重要
- **Andrychowicz et al.,《Hindsight Experience Replay》(NeurIPS 2017)** — 稀疏獎勵的經典解法，對機器人抓取場景影響深遠
- **Stable-Baselines3 官方文件** — 最常用的 RL 實作庫，範例完整，適合快速上手各種演算法
- **CleanRL** — 單檔案 RL 演算法實作，每個演算法一個 Python 檔，適合讀源碼理解演算法細節
- **Isaac Gym / Isaac Lab 官方教程** — GPU 加速的機器人 RL 訓練環境，了解大規模並行訓練的工程實踐
