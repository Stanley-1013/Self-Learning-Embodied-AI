---
title: "模仿學習與行為克隆"
prerequisites: ["19-drl-ppo-sac-ddpg"]
estimated_time: 45
difficulty: 3
tags: ["imitation-learning", "behavior-cloning", "dagger", "irl"]
sidebar_position: 20
---

# 模仿學習與行為克隆

## 你將學到

- 能精確講出 Behavior Cloning (BC)、DAgger、Inverse RL (IRL) 三種模仿學習方法的核心差異：BC 是純監督學習擬合 demo、DAgger 用在線人類糾偏解決 distribution shift、IRL 反推 reward function
- 遇到「有專家示範數據但不想設計 reward function」時，知道先判斷數據量、是否有在線專家、任務複雜度，然後選對方法
- 面試時能在兩分鐘內講清楚 distribution shift 是什麼、為什麼 BC 在長 horizon 任務會崩潰、DAgger 怎麼修

## 核心概念

**精確定義**：**模仿學習 (Imitation Learning, IL)** 是讓 agent 從專家示範中學習策略 $\pi_\theta$，不需要自己定義 reward function。核心假設：「我不知道什麼是好的（reward），但我有人做給我看（demonstration）」。和 RL 的本質差異 — RL 通過試錯 + reward 信號學習；IL 通過模仿 + 示範數據學習。

**三大方法**：

- **Behavior Cloning (BC)**：最簡單 — 把 $(s, a)$ 示範對當作監督學習的 $(x, y)$，直接擬合 $\pi_\theta(a|s)$。就是一個回歸/分類問題。快但脆弱。
- **DAgger (Dataset Aggregation)**：解決 BC 的 distribution shift 問題 — 用學生策略跑環境，遇到偏離時請專家標註正確動作，把新數據加入訓練集，反覆迭代。需要在線專家（interactive expert）。
- **Inverse RL (IRL)**：不直接學策略，而是從示範反推 reward function $R^*$，然後用 RL 在學到的 reward 下訓練策略。最靈活但最慢。

**Distribution Shift — IL 的核心困難**：

BC 在訓練時只見過專家走過的狀態 $s \sim d^{\pi_E}$。部署時如果策略犯了一個小錯，到達專家沒去過的狀態 $s' \notin d^{\pi_E}$ → 策略對 $s'$ 的輸出是垃圾（訓練時沒見過）→ 錯上加錯 → 累積偏移 → 完全崩潰。

$$
\text{BC 的累積誤差} \propto T^2 \cdot \epsilon
$$

其中 $T$ 是 horizon 長度，$\epsilon$ 是單步模仿誤差。**物理意義**：錯誤是二次方累積的 — horizon 長 2 倍，累積誤差惡化 4 倍。這就是 BC 在長 horizon 任務（如自駕、長序列操作）必然崩潰的數學原因。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：狀態/觀察 $s$（和 RL 策略一樣）+ 專家示範數據集 $\mathcal{D} = \{(s_i, a_i)\}$（來自人類遙操作或運動規劃器）
- **輸出**：策略 $\pi_\theta(a|s)$ — 和 Deep RL 策略的介面完全相同
- **下游**：動作送給底層控制器。和 Deep RL 互補 — RL 需要 reward 但不需要示範，IL 需要示範但不需要 reward
- **閉環節點**：和 Deep RL 一樣跨越**規劃 + 控制**，但學習信號從 reward 換成了 demonstration

**最少夠用的數學**：

1. **Behavior Cloning 的損失函數**（就是標準監督學習）：

$$
L_{BC}(\theta) = \mathbb{E}_{(s,a) \sim \mathcal{D}} \left[ \| \pi_\theta(s) - a \|^2 \right]
$$

**物理意義**：最小化策略輸出和專家動作的 MSE。連續動作空間用 L2 loss，離散用 cross-entropy。簡單直接，但只能保證在**專家數據分布**上表現好。

2. **DAgger 的迭代流程**：

$$
\mathcal{D}_{i+1} = \mathcal{D}_i \cup \{ (s, \pi_E(s)) \mid s \sim d^{\pi_i} \}
$$

**物理意義**：第 $i$ 輪用學生策略 $\pi_i$ 跑環境收集狀態 $s$，然後問專家「在這個狀態你會做什麼？」得到 $\pi_E(s)$。把新的 $(s, \pi_E(s))$ 加入數據集重新訓練。直覺：讓學生犯錯、老師糾正、下次就會了。

3. **IRL 的目標**（MaxEntropy IRL）：

$$
\max_{R} \mathbb{E}_{\pi_E}[R(s,a)] - \log Z(R) - \lambda \|R\|
$$

其中 $Z(R) = \int \exp(\sum_t R(s_t, a_t)) d\tau$ 是配分函數。

**物理意義**：找一個 reward function $R$，使得專家的行為在所有可能行為中概率最高（MaxEnt 原則）。$\lambda \|R\|$ 是正則化防止 reward 過擬合。歧義問題：多個 $R$ 可能解釋同一組示範（reward ambiguity）。

<details>
<summary>深入：Distribution Shift 的嚴格分析與 DAgger 的收斂證明</summary>

### Distribution Shift 的量化

設專家策略的狀態分布為 $d^{\pi_E}$，學生策略 $\pi_\theta$ 的狀態分布為 $d^{\pi_\theta}$。BC 的訓練數據來自 $d^{\pi_E}$，但部署時策略面對的是 $d^{\pi_\theta}$。

性能差距的上界（Ross et al., 2011）：

$$
J(\pi_E) - J(\pi_\theta) \le T^2 \epsilon_{train} + O(T)
$$

其中 $\epsilon_{train}$ 是在**專家分布**上的平均單步誤差。關鍵：$T^2$ 意味著即使單步誤差很小，長 horizon 下也會爆炸。

### DAgger 的理論保證

DAgger 在 $N$ 輪迭代後，性能差距的上界變為：

$$
J(\pi_E) - J(\pi_{best}) \le \epsilon_{train} \cdot T + O(\sqrt{T \log N / N})
$$

注意：$T^2$ 降到了 $T$（線性而非二次）。這是因為 DAgger 讓學生在自己的分布 $d^{\pi_i}$ 上收集數據，然後由專家標註，消除了 train-test distribution mismatch。

### 為什麼 IRL 有歧義

給定一組專家軌跡 $\{\tau_1, \tau_2, ...\}$，存在無限多個 reward function 能讓這些軌跡最優。極端例子：$R(s,a) = 0$ 對所有 $s, a$ — 所有行為都一樣好，專家行為當然也是最優的。

MaxEntropy IRL 通過最大熵原則選擇「最不做額外假設」的 reward — 但本質上歧義無法完全消除。實務上 IRL 學到的 reward 需要人工 sanity check。

</details>

<details>
<summary>深入：GAIL 與 Diffusion Policy — 現代模仿學習的演化</summary>

### GAIL (Generative Adversarial Imitation Learning, 2016)

GAIL 把 IRL + RL 合成一步 — 用 GAN 的框架直接匹配策略的軌跡分布和專家的軌跡分布：

$$
\min_\pi \max_D \mathbb{E}_{\pi_E}[\log D(s,a)] + \mathbb{E}_{\pi}[\log(1 - D(s,a))]
$$

- Discriminator $D$：區分「這個 $(s,a)$ 是專家的還是學生的？」
- Generator（策略 $\pi$）：騙過 Discriminator — 讓自己的行為越來越像專家

優點：不需要顯式學 reward → 跳過 IRL 的歧義問題。
缺點：GAN 訓練不穩定，需要大量在線交互（因為內層需要 RL）。

### Diffusion Policy (2023)

用 Diffusion Model 取代簡單的 MLP/Gaussian 作為策略表示：

$$
\pi_\theta(a|s) = \text{DenoisingProcess}(a^T \to a^0 | s, \theta)
$$

優點：
- **多模態動作**：擦桌子可以順時針也可以逆時針，Gaussian 策略只能輸出平均（卡在中間不動），Diffusion 能表示多個模式
- **高維連續動作**：輸出整段動作序列（action chunk），而不是單步動作。減少 compounding error
- **和 BC 結合**：Diffusion Policy 本質是一個更強的 BC — 用 diffusion model 做更好的條件分布建模

缺點：推理速度慢（需要多步去噪），目前在 10-100 Hz 控制場景仍有挑戰。

### 演化路線

```
BC (監督學習) → DAgger (在線糾偏) → GAIL (GAN+RL) → Diffusion Policy (強分布建模)
                                   → IRL (學 reward)
```

當前趨勢（2024）：Diffusion Policy + action chunking 是最 hot 的 IL 方法，特別在雙臂操作任務表現亮眼。

</details>

## 直覺理解

**類比：師傅帶徒弟**。BC 是師傅做一遍給徒弟看影片，徒弟照著影片練 — 步驟正確時很好，但一旦手滑偏了，影片裡沒教過「偏了怎麼修」，於是越偏越遠。DAgger 是師傅站在旁邊看徒弟做 — 每次徒弟偏了師傅立刻說「這時候應該這樣」，把糾正也記下來，下次就知道偏了怎麼救。IRL 是最高境界：不是學師傅的動作，而是理解師傅「為什麼這樣做」（reward function），然後自己摸索出可能比師傅更好的做法。

**Distribution Shift 的視覺比喻**：像一輛自駕車只在高速公路直線段訓練過。上路後稍微偏了一點 → 進入從沒訓練過的「偏離車道」狀態 → 不知道怎麼修正 → 偏更多 → 完全失控。這就是 compounding error 的本質。DAgger 的解法：讓車在路上開，但方向盤後面坐一個老司機，每次快偏時老司機說「打左」或「打右」，把這些修正動作記錄下來重新訓練。

**模擬器觀察**：在 MuJoCo 的 `Hopper-v4` 環境：
- 收集 100 條專家軌跡 → BC 訓練：初始幾步和專家一樣，但大約 50 步後開始偏離，100 步後摔倒
- 同樣的數據量但加 DAgger 迭代 5 輪：200+ 步仍然穩定
- 只用 10 條專家軌跡做 BC：幾乎立刻崩潰 — 數據少 + distribution shift 雙重打擊
- 對比：加數據增強（狀態加噪聲）的 BC，可以撐到 100+ 步 — 噪聲部分模擬了偏離狀態

## 實作連結

**三個典型工程場景**：

1. **遙操作數據 → BC 快速原型（機械臂抓取）**：人類用 SpaceMouse 遙控機械臂做 50 次抓取示範，收集 $(s, a)$ 對，訓練一個 MLP 策略。5 分鐘訓練完就能部署 — 但只對訓練時見過的物體位置有效。

2. **DAgger 在自駕場景**：先用 BC 訓練一個基礎駕駛策略，然後讓車上路（安全員坐旁邊）。每次策略即將偏離車道，安全員介入 → 記錄 $(s_{偏離}, a_{修正})$ → 加入數據集重新訓練。3-5 輪後策略的魯棒性顯著提升。

3. **IRL 學習人類偏好（導航）**：觀察人類在辦公室走動的軌跡，用 IRL 反推「人類覺得什麼路線好」的 reward — 可能包含「離牆遠一點」、「避開擁擠走廊」等隱含偏好。然後機器人用這個 reward 做路徑規劃。

**Code 骨架**（Python，BC 和 DAgger）：

```python
import torch
import torch.nn as nn
import numpy as np

class BCPolicy(nn.Module):
    """Behavior Cloning: 監督學習擬合專家動作"""
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, act_dim),  # 連續動作: 直接輸出
        )

    def forward(self, obs):
        return self.net(obs)

# BC 訓練
# loss = F.mse_loss(policy(obs_batch), action_batch)
# optimizer.step()

# DAgger 迭代
# for i in range(num_dagger_rounds):
#     rollout_states = collect_rollout(policy)    # 學生跑環境
#     expert_actions = query_expert(rollout_states) # 問專家
#     dataset.add(rollout_states, expert_actions)   # 聚合數據
#     policy = train_bc(dataset)                    # 重新訓練
```

<details>
<summary>深入：完整 DAgger 訓練流程（Python）</summary>

```python
"""
DAgger: 從零到部署的完整流程
依賴: torch, gymnasium, numpy
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
        self.expert = expert_policy  # 專家策略（或人類介面）
        self.policy = MLPPolicy(obs_dim, act_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=1e-3)
        self.dataset = {"obs": [], "act": []}

    def collect_expert_demos(self, num_episodes=50):
        """Phase 0: 收集初始專家示範"""
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
        """Phase 1+: 用學生策略收集狀態，問專家標註"""
        new_obs, new_act = [], []
        for _ in range(num_rollouts):
            obs, _ = self.env.reset()
            for _ in range(horizon):
                # 學生策略決定動作（但我們收集的是狀態）
                student_action = self.policy.get_action(obs)

                # 關鍵：問專家「在這個狀態你會做什麼？」
                expert_action = self.expert.get_action(obs)

                new_obs.append(obs)
                new_act.append(expert_action)  # 記錄的是專家動作

                # 用學生動作推進環境（不是專家動作！）
                obs, _, terminated, truncated, _ = self.env.step(student_action)
                if terminated or truncated:
                    break

        # 聚合到數據集
        self.dataset["obs"].extend(new_obs)
        self.dataset["act"].extend(new_act)

    def train_policy(self, epochs=50, batch_size=256):
        """監督學習訓練策略"""
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
        """完整 DAgger 流程"""
        # Phase 0: 初始專家示範
        print("Phase 0: Collecting expert demonstrations...")
        self.collect_expert_demos(num_episodes=50)
        loss = self.train_policy()
        print(f"  Initial BC loss: {loss:.4f}")

        # Phase 1-N: DAgger 迭代
        for round_i in range(1, num_rounds + 1):
            print(f"DAgger Round {round_i}/{num_rounds}")

            # 用學生策略收集 + 專家標註
            self.dagger_iteration(num_rollouts=20)

            # 在聚合數據上重新訓練
            loss = self.train_policy()
            print(f"  Loss: {loss:.4f}, Dataset size: {len(self.dataset['obs'])}")

            # 評估
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

**關鍵實作細節**：

- `dagger_iteration` 中用**學生動作**推進環境，但記錄的是**專家動作** — 這是 DAgger 的核心：在學生的分布上收集專家標註
- 數據集不斷增長（aggregate），不是替換 — 保留早期數據防止遺忘
- 實務中「問專家」可能是：人類遙操作介面、一個更昂貴的運動規劃器、或預訓練好的 RL 策略
- 數據增強可以進一步提升效果：給 obs 加高斯噪聲模擬偏離狀態

</details>

## 常見誤解

1. **「BC 只要數據夠多就不會有 distribution shift」** — 數學上 BC 的累積誤差是 $O(T^2 \epsilon)$，即使 $\epsilon$ 很小（大量數據讓擬合很好），$T$ 夠長仍然會崩。而且現實中不可能覆蓋所有可能的偏離狀態 — 那是指數級的。**正確理解**：數據多可以降低 $\epsilon$，但無法消除 $T^2$ 的放大效應。解法是 DAgger（改分布）或 action chunking（縮短有效 horizon）。

2. **「IRL 能學到唯一正確的 reward function」** — IRL 本質上有歧義。最極端：$R=0$ 讓所有行為等價，專家行為當然「最優」。即使用 MaxEntropy IRL 也只是選了一個「最不做假設」的 reward，不代表唯一正確。**正確理解**：IRL 學到的 reward 需要人工 sanity check，而且不同的正則化/先驗會給出不同的 reward。

3. **「模仿學習需要完美的專家示範」** — BC/DAgger 對示範的一致性比完美性更重要。如果 10 條示範中 5 條走左邊、5 條走右邊，策略會學到「走中間」（取平均）— 這可能是最差的選擇。反而統一走一邊（即使不是最優）效果更好。**正確理解**：示範的**一致性**比**最優性**重要。如果專家行為有多種模式，需要用 mixture model 或 Diffusion Policy 來表示多模態分布。

## 練習題

<details>
<summary>Q1：你有 100 條人類遙操作的機械臂抓取示範。BC 訓練後在已見過的物體位置成功率 90%，但物體稍微移動 5 cm 就掉到 30%。怎麼分析和改善？</summary>

**分析推理**：

1. **根因：distribution shift**。訓練數據只覆蓋專家操作時的物體位置分布。物體偏移 5 cm → 進入策略沒見過的狀態 → 輸出不可靠
2. **方案一：數據增強**：在訓練時給 obs 中的物體位置加隨機偏移（±10 cm），同時對 action 做對應調整。讓策略在訓練時就見過偏離的狀態
3. **方案二：DAgger**：讓 BC 策略自己抓（物體隨機放），每次失敗時人類介入標註正確動作，加入數據集重新訓練。5 輪後覆蓋到更多偏離狀態
4. **方案三：Action chunking**：不輸出單步動作，而是輸出未來 10 步的動作序列（action chunk）。這降低了有效 horizon（從 100 步降到 10 個 chunk），減少 compounding error
5. **避開陷阱**：不要只堆更多示範 — 100 條示範可能都在同一個窄分布裡。需要的是**分布覆蓋**（diverse 的物體位置），不是同一位置的重複示範

**結論**：短期用數據增強 + action chunking；有在線專家資源就上 DAgger。核心是擴大訓練分布的覆蓋範圍。

</details>

<details>
<summary>Q2：你的任務需要機器人做一個長序列操作（打開櫃門 → 拿出碗 → 放到桌上 → 關門），BC 在第二步就開始崩潰。怎麼辦？</summary>

**分析推理**：

1. **根因**：長 horizon + compounding error。4 個子任務串聯，每個子任務 50 步 → 有效 $T = 200$。BC 的 $O(T^2)$ 累積誤差在 $T=200$ 時已經失控
2. **方案一：分層模仿**（Hierarchical IL）：把任務拆成 4 個子策略分別做 BC，用一個高層策略（state machine 或學習的 meta-policy）決定什麼時候切換。每個子策略的有效 horizon 降到 ~50 步
3. **方案二：DAgger + 分段**：先用 BC 訓練全流程，然後在真機上 DAgger — 但只在子任務**銜接點**（transition）重點收集專家糾偏。因為銜接點是 distribution shift 最嚴重的地方
4. **方案三：Action chunking + Diffusion Policy**：用 Diffusion Policy 輸出長 action chunk（如 16 步），配合 temporal ensemble 做重疊預測。ACT (Action Chunking with Transformers) 論文證明這在雙臂操作很有效
5. **方案四：Goal-conditioned BC**：每個子任務有明確的目標狀態（如「門打開到 90°」），策略以 $(s, g)$ 為輸入。達到 goal 就切換到下一個子任務
6. **避開陷阱**：不要試圖用一個端到端 BC 策略學完整個 200 步序列 — 即使有 1000 條示範，compounding error 的 $T^2$ 效應在 $T=200$ 時是不可控的

**結論**：分層模仿（降低有效 horizon）+ action chunking（減少 compounding error）。長序列操作幾乎不可能用純 BC 端到端解決。

</details>

<details>
<summary>Q3：團隊想用 IRL 而非 BC 來學習機器人擦桌子。你覺得值不值？怎麼評估？</summary>

**分析推理**：

1. **IRL 的優勢**：學到 reward function 而非策略 → (a) 可以在新環境用 RL 重新訓練而不需要新示範；(b) 更 generalizable — reward 描述「什麼是好的」而非「怎麼做」；(c) 能學到隱含偏好（如「擦角落比中間重要」）
2. **IRL 的代價**：(a) 內層需要跑 RL → 訓練時間 10-100x BC；(b) reward ambiguity — 多個 reward 都能解釋同一組示範，需要人工驗證；(c) IRL 的超參數更多、debug 更難
3. **評估標準**：
   - 擦桌子的桌面形狀會不會變？如果每次都是同一張桌子 → BC 就夠
   - 需不需要遷移到新桌子/新物體？→ IRL 學到的 reward 更 transferable
   - 有沒有足夠算力做內層 RL？→ IRL 訓練需要在模擬器裡跑大量 RL
4. **替代方案**：GAIL — 省去顯式 reward 學習，直接匹配軌跡分布。比 IRL 更穩定但仍需要在線交互。或者用 Diffusion Policy — 比 BC 更強的分布建模，避免 mode averaging
5. **結論**：除非有明確的「遷移到新環境」需求且算力充足，否則 BC（或 Diffusion Policy）+ 數據增強 + DAgger 的 ROI 更高。IRL 更適合學術研究場景

**面試官想聽到**：不被「更高級」的方法迷惑，能根據實際需求和成本做工程判斷。

</details>

## 面試角度

1. **Distribution Shift 是模仿學習的核心困難** — 這是必考概念。帶出：「BC 的累積誤差和 horizon 的平方成正比 — 單步只偏一點點，100 步後就完全跑偏了。這就是 distribution shift：策略犯錯 → 到了訓練時沒見過的狀態 → 不知道怎麼修 → 越偏越遠。DAgger 的解法是讓學生在自己的分布上收集數據、由專家標註正確動作。」

2. **BC vs DAgger vs IRL 的選擇邏輯** — 展現方法論判斷力。帶出：「數據多 + 短 horizon + 無在線專家 → BC 就夠。長 horizon + 有在線專家 → DAgger。需要遷移到新環境 + 算力充足 → IRL。實務中 80% 的場景 BC + 數據增強就能解決，DAgger 是 BC 撞瓶頸時的升級路線。」

3. **Action Chunking 是對抗 Compounding Error 的工程利器** — 展現前沿實作意識。帶出：「不輸出單步動作而是輸出未來一段動作序列，等效於把有效 horizon 縮短。ACT 和 Diffusion Policy 都用了這個技巧，在雙臂操作任務上效果顯著。這是 2023-2024 年 IL 領域最重要的工程突破之一。」

4. **模仿學習和 RL 是互補的** — 展現全局視野。帶出：「RL 需要 reward 但不需要示範，IL 需要示範但不需要 reward。實務中常常組合：先用 BC/DAgger 得到一個不錯的初始策略（warm start），再用 RL fine-tune 超越專家。或者用 IRL 從示範學 reward，再用 PPO/SAC 在學到的 reward 下訓練。」

5. **示範的一致性比完美性重要** — 區分「知道 IL」和「踩過坑」。帶出：「如果專家的示範有多種模式（一半走左一半走右），標準 BC 會學到走中間 — 這是 mode averaging，通常是最差的行為。解法是用能表示多模態分布的策略（GMM、Diffusion Policy），或者在收集數據時就保持一致性。」

## 延伸閱讀

- **Ross et al.,《A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning》(2011)** — DAgger 原論文，distribution shift 的嚴格分析和解法，寫得很清晰
- **Ho & Ermon,《Generative Adversarial Imitation Learning》(2016)** — GAIL 論文，用 GAN 框架統一 IRL + RL，是現代 IL 的重要里程碑
- **Zhao et al.,《Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware》(2023)** — ACT (Action Chunking with Transformers) 論文，action chunking + Transformer 在雙臂操作的突破性結果
- **Chi et al.,《Diffusion Policy: Visuomotor Policy Learning via Action Diffusion》(2023)** — Diffusion Policy 論文，用擴散模型做策略表示，解決多模態動作問題
- **Ziebart et al.,《Maximum Entropy Inverse Reinforcement Learning》(2008)** — MaxEnt IRL 原論文，IRL 歧義問題的優雅解法
- **robomimic 框架** — NVIDIA 的模仿學習 benchmark + 實作框架，支持 BC、BC-RNN、HBC 等方法，有完整的 evaluation pipeline
- **LeRobot (Hugging Face)** — 開源機器人學習平台，支持 ACT、Diffusion Policy 的訓練和部署，配合低成本硬體
