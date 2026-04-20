---
title: "模仿學習與行為克隆"
prerequisites: ["19-drl-ppo-sac-ddpg"]
estimated_time: 50
difficulty: 3
tags: ["imitation-learning", "behavior-cloning", "dagger", "irl", "gail", "diffusion-policy"]
sidebar_position: 20
---

# 模仿學習與行為克隆

## 你將學到

- 能精確講出 Behavior Cloning (BC)、DAgger、IRL、GAIL、Diffusion Policy 五種模仿學習方法的核心機制與適用場景，面試時不含糊
- 遇到「有專家示範但不想設計 reward」的場景，能根據數據量、是否有在線專家、動作分布是否多模態，判斷該用哪個方法
- 理解 distribution shift 是 BC 必然崩潰的數學原因，並知道 DAgger 如何從 $O(T^2)$ 降到 $O(T)$

## 核心概念

### 六個精確定義

1. **Behavior Cloning (BC)**：把專家示範 $(s, a)$ 當監督學習的 $(x, y)$，直接最小化 $\|\pi_\theta(s) - a\|^2$。本質是回歸問題。快速出原型但脆弱 — 依賴 i.i.d. 假設，而部署時策略自身的分布和訓練分布不同。

2. **Distribution Shift（分布偏移）**：BC 訓練時見的是專家走過的狀態 $s \sim d^{\pi_E}$。部署時策略犯一個小錯 → 進入專家沒走過的狀態 → 策略輸出不可靠 → 錯上加錯。累積誤差和 horizon 的**平方**成正比：$\text{error} \propto T^2 \cdot \epsilon$。

3. **DAgger (Dataset Aggregation)**：學生策略執行 → 收集學生實際到達的狀態 → 專家標註正確動作 → 新數據聚合到訓練集 → 重新訓練 → 迭代。核心：在**學生的分布**上收集**專家的標註**，直接解決 train-test distribution mismatch。

4. **Inverse RL (IRL)**：不學策略學 reward — 從示範反推 $R^*$，再用 RL 在學到的 reward 下訓練策略。歧義性是本質問題：$R=0$ 也能讓所有行為「最優」。MaxEntropy IRL 用最大熵原則約束，選最不做額外假設的 reward。

5. **GAIL (Generative Adversarial Imitation Learning)**：GAN 思想 — Generator 是策略 $\pi$，Discriminator 區分「這個 $(s,a)$ 是專家還是學生的？」。跳過顯式 reward 學習，直接匹配軌跡分布。省去 IRL 的歧義問題，但 GAN 訓練不穩定且需要大量在線交互。

6. **Diffusion Policy**：用擴散模型（denoising diffusion）生成動作序列。從高斯噪聲開始，逐步去噪到乾淨動作。核心優勢：能表示**多模態動作分布** — 擦桌子可以順時針也可以逆時針，MSE loss 的 Gaussian 策略只能輸出平均（卡在中間不動），Diffusion 能保留多個 mode。輸出 action chunk（多步動作序列）進一步減少 compounding error。

### 閉環定位

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：狀態/觀察 $s$（和 RL 策略一樣）+ 專家示範數據集 $\mathcal{D} = \{(s_i, a_i)\}$（來自人類遙操作或運動規劃器）
- **輸出**：策略 $\pi_\theta(a|s)$ — 和 Deep RL 策略的介面完全相同
- **下游**：動作送給底層控制器（MPC / PID）。和 Deep RL 互補 — RL 需要 reward 但不需要示範，IL 需要示範但不需要 reward
- **閉環節點**：跨越**規劃 + 控制**，學習信號從 reward 換成 demonstration。實務中常見組合：IL warm start → RL fine-tune 超越專家

### 最少夠用的數學

1. **BC Loss**（標準監督學習損失）：

$$
L_{BC}(\theta) = \mathbb{E}_{(s,a) \sim \mathcal{D}} \left[ \| \pi_\theta(s) - a \|^2 \right]
$$

**物理意義**：最小化策略輸出和專家動作的 MSE。連續動作空間用 L2，離散用 cross-entropy。只能保證在**專家數據分布上**表現好，對偏離狀態沒有任何保證。

2. **BC 累積誤差的上界**（Ross & Bagnell, AISTATS 2010）：

$$
J(\pi_E) - J(\pi_\theta) \le T^2 \epsilon + O(T)
$$

**物理意義**：$\epsilon$ 是單步模仿誤差，$T$ 是 horizon。即使 $\epsilon$ 很小，$T$ 夠長就會爆炸。這是 BC 在長 horizon 任務（自駕、長序列操作）**必然崩潰**的數學根源。

3. **DAgger 的數據聚合**：

$$
\mathcal{D}_{i+1} = \mathcal{D}_i \cup \{ (s, \pi_E(s)) \mid s \sim d^{\pi_i} \}
$$

**物理意義**：第 $i$ 輪用學生策略 $\pi_i$ 跑環境收集狀態 $s$，問專家「你在這裡會怎麼做？」得到 $\pi_E(s)$。把新 $(s, \pi_E(s))$ 加入數據集重新訓練。DAgger 把誤差上界從 $O(T^2)$ 降到 $O(T)$ — 線性取代二次。

**原論文的 $\beta$-mixing schedule**：Ross et al. 2011 定義的 rollout 策略其實是 $\pi_i = \beta_i \pi_E + (1 - \beta_i)\hat{\pi}_i$，$\beta_i$ 隨迭代遞減到 0 — 初期多讓專家帶路避免學生過度瞎走。實務上大多數實作（含本章 code 骨架）直接取 $\beta = 0$ 純用學生動作，稱為 "pure DAgger"；論文的 regret 保證對此變體仍然成立，但若 $\beta > 0$ 初期會更穩。

4. **IRL 的 MaxEntropy 目標**：

$$
\max_{R} \mathbb{E}_{\pi_E}[R(s,a)] - \log Z(R) - \lambda \|R\|
$$

**物理意義**：找 reward function 使專家行為在所有可能行為中概率最高。$Z(R)$ 是配分函數（所有軌跡的指數加權和），$\lambda\|R\|$ 防止 reward 過擬合。歧義性無法完全消除 — IRL 學到的 reward 需要人工 sanity check。

5. **GAIL 的 min-max 目標**：

$$
\min_\pi \max_D \mathbb{E}_{\pi_E}[\log D(s,a)] + \mathbb{E}_{\pi}[\log(1 - D(s,a))]
$$

**物理意義**：Discriminator 學會區分專家和學生的 $(s,a)$；策略學會騙過 Discriminator。和 GAN 完全對偶 — Discriminator 隱式地提供了 reward signal。

6. **Diffusion Policy 的去噪過程**：

$$
a^{k-1} = \frac{1}{\sqrt{\alpha_k}}\left(a^k - \frac{1-\alpha_k}{\sqrt{1-\bar{\alpha}_k}}\epsilon_\theta(a^k, k, s)\right) + \sigma_k z
$$

**物理意義**：從純噪聲 $a^K$ 開始，每步用網路 $\epsilon_\theta$ 預測噪聲並去除，逐步恢復乾淨動作。conditioning on $s$ 讓去噪過程依賴當前觀測。多步去噪天然支持多模態 — 不同噪聲初始化收斂到不同 mode。

<details>
<summary>深入：Distribution Shift 的嚴格分析與 DAgger 收斂證明</summary>

### Distribution Shift 的量化

設專家策略的狀態分布為 $d^{\pi_E}$，學生策略 $\pi_\theta$ 的狀態分布為 $d^{\pi_\theta}$。BC 在 $d^{\pi_E}$ 上訓練，但部署時面對 $d^{\pi_\theta}$。

Ross & Bagnell (AISTATS 2010, "Efficient Reductions for Imitation Learning") 證明的 BC 性能差距上界：

$$
J(\pi_E) - J(\pi_\theta) \le T^2 \epsilon_{train} + O(T)
$$

其中 $\epsilon_{train}$ 是在**專家分布**上的平均單步誤差。$T^2$ 項的來源：

1. 第 1 步：誤差 $\epsilon$（和專家行為的偏差）
2. 第 2 步：因為第 1 步偏了，狀態偏離 → 額外誤差 $\epsilon + \delta$
3. 第 $t$ 步：前面所有步的偏差累積 → 誤差 $\approx t \cdot \epsilon$
4. 總和 $\sum_{t=1}^T t \cdot \epsilon \approx T^2 \epsilon / 2$

### DAgger 的收斂保證

Ross, Gordon & Bagnell (AISTATS 2011) 對 DAgger 的線性上界：

$$
J(\pi_E) - J(\pi_{best}) \le \epsilon_{train} \cdot T + O\left(\sqrt{\frac{T \log N}{N}}\right)
$$

關鍵改進：$T^2$ 降到 $T$。原因：DAgger 讓學生在**自己的分布** $d^{\pi_i}$ 上收集數據，由專家標註，直接消除 train-test distribution mismatch。

### 為什麼 IRL 有歧義

給定專家軌跡 $\{\tau_1, \tau_2, ...\}$，存在無限多個 reward function 讓這些軌跡最優。極端例子：$R(s,a) = 0$ → 所有行為等價 → 專家行為「最優」。

MaxEntropy IRL 通過最大熵原則選「最不做假設」的 reward，但歧義無法完全消除。GAIL 繞過了這個問題 — 不顯式學 reward，而是直接匹配軌跡分布的佔據度量 (occupancy measure)。

### GAIL 與 IRL 的對偶關係

Ho & Ermon (2016) 證明：GAIL 的最優 Discriminator 恰好是 IRL reward 的函數。具體來說，GAIL 等價於在 occupancy measure 空間做 Jensen-Shannon divergence 最小化：

$$
\min_\pi D_{JS}(\rho_\pi \| \rho_{\pi_E})
$$

其中 $\rho_\pi(s,a) = \sum_t P(s_t=s, a_t=a \mid \pi)$ 是策略的 occupancy measure。

</details>

<details>
<summary>深入：Diffusion Policy 的完整機制與 Action Chunking 工程細節</summary>

### Diffusion Policy 架構

Diffusion Policy (Chi et al., 2023) 把動作生成建模為條件去噪擴散過程：

**Forward process（加噪）**：

$$
q(a^k \mid a^{k-1}) = \mathcal{N}(a^k; \sqrt{\alpha_k} a^{k-1}, (1-\alpha_k)I)
$$

從乾淨動作 $a^0$ 逐步加噪到純高斯噪聲 $a^K$。

**Reverse process（去噪/生成）**：

$$
p_\theta(a^{k-1} \mid a^k, s) = \mathcal{N}(a^{k-1}; \mu_\theta(a^k, k, s), \sigma_k^2 I)
$$

學一個網路 $\epsilon_\theta$ 預測噪聲，conditioning on 觀測 $s$。推理時從 $a^K \sim \mathcal{N}(0, I)$ 開始去噪 $K$ 步得到動作。

**訓練損失**（噪聲預測）：

$$
L = \mathbb{E}_{k, a^0, \epsilon} \left[\| \epsilon - \epsilon_\theta(\sqrt{\bar{\alpha}_k}a^0 + \sqrt{1-\bar{\alpha}_k}\epsilon, k, s) \|^2\right]
$$

### 為什麼 Diffusion Policy 能解決多模態問題

考慮擦桌子任務：專家有時順時針擦、有時逆時針擦。

- **MSE loss（Gaussian BC）**：最小化 $\|a_{pred} - a_{expert}\|^2$ → 預測的動作是所有專家動作的**均值** → 既不順時針也不逆時針，卡在中間不動 → 最差行為
- **Diffusion Policy**：不同的噪聲初始化 $a^K$ 在去噪過程中收斂到不同的 mode → 有時輸出順時針、有時逆時針 → 每次都是有效動作

### Action Chunking 工程細節

不輸出單步動作 $a_t$，而是輸出未來 $H$ 步的動作序列 $[a_t, a_{t+1}, ..., a_{t+H-1}]$：

1. **降低有效 horizon**：原來 $T=200$ 步變成 $T/H = 200/16 \approx 13$ 個 chunk → compounding error 從 $T^2$ 降到 $(T/H)^2$
2. **Temporal ensemble（ACT 的推理時 trick，非 chunking 本身的性質）**：ACT 在推理時把多個重疊 chunk 對同一時刻的預測做指數加權平均 → 動作更平滑。這是 ACT 論文特有的 inference-time 設計；單純 action chunking 不需要 temporal ensemble 也能用。
3. **推理頻率**：不需要每步推理，每 $H$ 步推理一次 → 允許更大的模型（推理延遲被 amortize）

**ACT (Action Chunking with Transformers)**：用 CVAE + Transformer 做 action chunking，在雙臂操作任務表現突出。Diffusion Policy + action chunking 是 2023-2024 年 IL 最 hot 的組合。

### 推理速度問題與解法

標準 DDPM 需要 $K=100-1000$ 步去噪 → 推理延遲太高（>100ms）。解法：

- **DDIM**：確定性去噪，$K$ 可降到 10-20 步
- **Consistency Distillation**：蒸餾成 1-4 步生成
- **Flow Matching**：用 ODE 取代 SDE，更高效的軌跡

目前 Diffusion Policy 在 10 Hz 控制場景可行，100 Hz 仍有挑戰。

</details>

### 常用 API / 框架

| 框架 | 支援方法 | 典型用途 |
|------|---------|---------|
| robomimic (NVIDIA) | BC, BC-RNN, HBC | 標準化 IL benchmark + 評估 |
| LeRobot (Hugging Face) | ACT, Diffusion Policy | 開源訓練 + 低成本硬體部署 |
| imitation (Stable Baselines) | BC, DAgger, GAIL | 快速原型 + 教學 |
| d3rlpy | BC, IQL（離線 RL/IL） | 離線學習一站式工具 |

## 直覺理解

**類比：師傅帶徒弟**

- **BC**：師傅做一遍拍影片，徒弟照影片練。步驟對的時候很好，但手一滑偏了，影片沒教過「偏了怎麼修」→ 越偏越遠。食譜式學習，偏離食譜就崩。
- **DAgger**：師傅站旁邊看徒弟做，每次偏了師傅立刻說「這時候應該這樣」，把糾正記下來。下次就知道偏了怎麼救。駕訓班模式。
- **IRL**：不學師傅的動作，而是理解師傅「為什麼這樣做」（reward function）。最高境界但最慢 — 且師傅的意圖可能有多種解讀（歧義）。
- **GAIL**：找一個裁判（Discriminator）看著學生和師傅做，不斷告訴學生「你這個動作不像師傅」。不需要理解師傅為什麼這樣做，只要模仿到分不出來就行。
- **Diffusion Policy**：師傅示範了多種做法（有時左擦有時右擦），學生能記住所有做法並隨機選一種執行 — 而不是把所有做法平均成「不動」。

**Distribution Shift 的視覺比喻**：自駕車只在高速公路直線段訓練過。上路後稍微偏了一點 → 進入沒訓練過的「偏離車道」狀態 → 不知道怎麼修正 → 偏更多 → 完全失控。這就是 compounding error。DAgger 的解法：方向盤後面坐一個老司機，每次快偏時老司機說「打左」或「打右」，把修正動作記錄下來重新訓練。

**模擬器觀察**：在 MuJoCo 的 `Hopper-v4`：
- 100 條專家軌跡 → BC：初始 50 步 OK，100 步後摔倒
- 同數據量 + DAgger 5 輪：200+ 步穩定
- 只用 10 條做 BC：幾乎立刻崩（數據少 + distribution shift 雙重打擊）
- BC + 狀態噪聲增強：撐到 100+ 步（噪聲部分模擬偏離狀態）
- Diffusion Policy：多模態任務（T 型迷宮）中不會卡在 mode 之間，BC 會

## 實作連結

**四個典型工程場景**：

1. **遙操作數據 → BC 快速原型（機械臂抓取）**：SpaceMouse 遙控機械臂做 50 次抓取示範 → 收集 $(s, a)$ → MLP 訓練 5 分鐘 → 部署。只對訓練時見過的物體位置有效，但作為 baseline 極快。

2. **DAgger 在自駕場景**：BC 訓練基礎駕駛策略 → 上路（安全員坐旁邊）→ 每次即將偏離車道，安全員介入 → 記錄 $(s_{偏離}, a_{修正})$ → 聚合重新訓練。3-5 輪後魯棒性顯著提升。

3. **Diffusion Policy 做雙臂操作**：收集 200+ 條雙臂折疊衣服的 demo → Diffusion Policy + action chunking → 能處理「左手先折」和「右手先折」兩種 mode，不會卡死。ACT/Diffusion Policy 是 2024 年雙臂操作的首選。

4. **IL warm start + RL fine-tune**：先用 BC 從 50 條 demo 學一個能完成 60% 任務的初始策略 → 再用 PPO/SAC 在真實 reward 下 fine-tune → 最終超越專家。IL 提供良好初始化，RL 突破專家天花板。

**Code 骨架**（Python，BC + DAgger）：

```python
import torch
import torch.nn as nn
import numpy as np

class BCPolicy(nn.Module):
    """Behavior Cloning: 監督學習擬合專家動作"""
    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, act_dim),
        )

    def forward(self, obs: torch.Tensor) -> torch.Tensor:
        return self.net(obs)

# --- BC 訓練 ---
# loss = F.mse_loss(policy(obs_batch), action_batch)
# optimizer.step()

# --- DAgger 迭代 ---
# for i in range(num_dagger_rounds):
#     states = collect_rollout(policy)        # 學生跑環境
#     expert_acts = query_expert(states)       # 問專家
#     dataset.add(states, expert_acts)          # 聚合
#     policy = train_bc(dataset)                # 重新訓練
```

<details>
<summary>深入：完整 DAgger 訓練流程（Python，可直接運行）</summary>

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

class MLPPolicy(nn.Module):
    def __init__(self, obs_dim: int, act_dim: int):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, act_dim),
        )

    def forward(self, obs):
        return self.net(obs)

    def get_action(self, obs: np.ndarray) -> np.ndarray:
        with torch.no_grad():
            return self.net(torch.FloatTensor(obs).unsqueeze(0)).squeeze(0).numpy()

class DAggerTrainer:
    def __init__(self, env_name: str, expert_policy, obs_dim: int, act_dim: int):
        self.env = gym.make(env_name)
        self.expert = expert_policy
        self.policy = MLPPolicy(obs_dim, act_dim)
        self.optimizer = optim.Adam(self.policy.parameters(), lr=1e-3)
        self.dataset = {"obs": [], "act": []}

    def collect_expert_demos(self, num_episodes: int = 50):
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

    def dagger_iteration(self, num_rollouts: int = 20, horizon: int = 500):
        """DAgger 核心：用學生跑、問專家標、聚合數據"""
        for _ in range(num_rollouts):
            obs, _ = self.env.reset()
            for _ in range(horizon):
                student_action = self.policy.get_action(obs)
                expert_action = self.expert.get_action(obs)  # 問專家
                self.dataset["obs"].append(obs)
                self.dataset["act"].append(expert_action)     # 記專家動作
                obs, _, terminated, truncated, _ = self.env.step(student_action)  # 用學生動作推進
                if terminated or truncated:
                    break

    def train_policy(self, epochs: int = 50, batch_size: int = 256) -> float:
        obs_arr = torch.FloatTensor(np.array(self.dataset["obs"]))
        act_arr = torch.FloatTensor(np.array(self.dataset["act"]))
        n = len(obs_arr)
        total_loss = 0.0
        for _ in range(epochs):
            for i in range(0, n, batch_size):
                batch_idx = np.random.randint(0, n, batch_size)
                pred = self.policy(obs_arr[batch_idx])
                loss = nn.functional.mse_loss(pred, act_arr[batch_idx])
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
                total_loss += loss.item()
        return total_loss / max(1, (n // batch_size) * epochs)

    def run(self, num_rounds: int = 5):
        self.collect_expert_demos(50)
        loss = self.train_policy()
        print(f"BC baseline loss: {loss:.4f}")
        for r in range(1, num_rounds + 1):
            self.dagger_iteration(20)
            loss = self.train_policy()
            print(f"DAgger round {r}: loss={loss:.4f}, data={len(self.dataset['obs'])}")
```

**關鍵實作細節**：
- `dagger_iteration` 中用**學生動作**推進環境，但記錄**專家動作** — 這是 DAgger 的核心
- 數據集不斷增長（aggregate），不是替換 — 保留早期數據防止遺忘
- 「問專家」可以是：人類遙操作、運動規劃器、預訓練 RL 策略
- 可加數據增強：obs 加高斯噪聲模擬偏離狀態，進一步提升魯棒性

</details>

## 常見誤解

1. **「BC 只要數據夠多就能泛化」** — 數學上 BC 累積誤差 $\propto T^2 \epsilon$。即使 $\epsilon$ 小（大量數據讓擬合好），$T$ 夠長仍崩。更關鍵的是，不可能覆蓋所有偏離狀態 — 那是指數級的。**正確理解**：數據多降低 $\epsilon$，但無法消除 $T^2$ 放大效應。要靠 DAgger（改分布）或 action chunking（縮短有效 horizon）。

2. **「IRL 能學到唯一正確的 reward function」** — IRL 本質有歧義。$R=0$ 讓所有行為等價，專家行為當然「最優」。即使 MaxEntropy IRL 也只是選了最不做假設的 reward。不同先驗/正則化給出不同 reward。**正確理解**：IRL 學到的 reward 需要人工 sanity check，是「合理的」而非「唯一的」。

3. **「示範越多越好，品質不重要」** — 如果 10 條示範中 5 條走左、5 條走右，MSE loss 會學到「走中間」— 通常是最差行為（mode averaging）。反而統一走一邊效果更好。**正確理解**：示範的**一致性**比**數量**重要。多模態行為需用 GMM 或 Diffusion Policy 表示，不能用 MSE 平均化。

4. **「離線模仿學習就夠了，不需要在線交互」** — 純離線 BC 在簡單短 horizon 任務 OK。但稍微複雜一點就需要某種形式的在線數據收集（DAgger、GAIL 都需要）。即使 Diffusion Policy 也受益於迭代式在線數據收集。**正確理解**：離線是起點，不是終點。在線交互是提升魯棒性的關鍵路徑。

5. **「Diffusion Policy 太慢，不能用在實時控制」** — 標準 DDPM 確實慢（100+ 步去噪）。但 DDIM 可以 10-20 步，consistency distillation 可以 1-4 步。配合 action chunking（每 $H$ 步推理一次），有效推理頻率可達 10+ Hz。**正確理解**：Diffusion Policy 在 10 Hz 控制場景已可行，搭配 action chunking 和加速去噪技術，速度不再是瓶頸。100 Hz 力控場景仍需分層架構。

## 練習題

<details>
<summary>Q1：你有 100 條遙操作的機械臂抓取示範。BC 訓練後成功率 90%，但物體偏移 5 cm 就掉到 30%。怎麼分析和改善？</summary>

**完整推理鏈**：

1. **根因：distribution shift**。訓練數據只覆蓋專家操作時的物體位置分布。物體偏移 5 cm → 進入策略沒見過的狀態 → 輸出不可靠
2. **方案一：數據增強**：給 obs 中的物體位置加隨機偏移（±10 cm）+ 對應 action 調整。讓策略在訓練時就見過偏離的狀態。成本最低但效果有上限
3. **方案二：DAgger**：讓 BC 策略自己抓（物體隨機放），每次失敗時人類介入標註正確動作，加入數據集重新訓練。5 輪後覆蓋更多偏離狀態
4. **方案三：Action chunking**：輸出未來 10 步動作序列（action chunk），降低有效 horizon（100 步 → 10 個 chunk），減少 compounding error
5. **避開陷阱**：不要只堆更多示範 — 100 條示範可能都在同一個窄分布。需要的是**分布覆蓋**（diverse 的物體位置），不是同一位置的重複

**結論**：短期用數據增強 + action chunking；有在線專家資源就上 DAgger。核心是擴大訓練分布的覆蓋。

</details>

<details>
<summary>Q2：長序列操作（開門 → 拿碗 → 放桌 → 關門），BC 在第二步就崩潰。怎麼辦？</summary>

**完整推理鏈**：

1. **根因**：長 horizon + compounding error。4 個子任務串聯，每個 50 步 → $T = 200$。BC 的 $O(T^2)$ 在 $T=200$ 時失控
2. **方案一：分層模仿（Hierarchical IL）**：拆成 4 個子策略分別 BC，用高層 meta-policy（state machine 或學習的切換器）決定何時切換。每個子策略有效 horizon ~50 步，compounding error 大幅降低
3. **方案二：DAgger + 重點銜接**：全流程 BC → 真機 DAgger，但重點在子任務**銜接點**收集專家糾偏（銜接點 distribution shift 最嚴重）
4. **方案三：Diffusion Policy + Action Chunking**：輸出 16 步 action chunk + temporal ensemble 重疊預測。ACT 論文證明在雙臂操作有效
5. **方案四：Goal-conditioned BC**：每個子任務有明確 goal（門開 90°），以 $(s, g)$ 為輸入。達到 goal 切換下一個子任務
6. **避開陷阱**：不要用一個端到端 BC 學 200 步序列 — $T^2$ 效應在 $T=200$ 時不可控

**結論**：分層模仿（降有效 horizon）+ action chunking（減 compounding error）。長序列純 BC 端到端幾乎不可能。

</details>

<details>
<summary>Q3：少量 demo（只有 10 條），但需要讓機器人學會一個中等複雜度的操作。怎麼最大化利用？</summary>

**完整推理鏈**：

1. **10 條 demo 的困境**：數據太少 → 擬合不好（$\epsilon$ 大）+ 分布覆蓋窄（distribution shift 嚴重）
2. **數據增強優先**：對 obs 加隨機裁剪/色彩抖動/高斯噪聲，對 action 加小幅隨機擾動 → 等效擴充 10x-50x。但增強的數據不能偏離太遠
3. **Sim-to-Real 擴充**：在 sim 裡做 domain randomization + 大量自動 demo → 預訓練 → 用 10 條真機 demo fine-tune。Sim 數據解決量的問題，真機 demo 做 domain adaptation
4. **Diffusion Policy / CVAE**：比 MSE loss BC 更能從少量數據學到有效分布。Diffusion Policy 的正則化效果（去噪過程本身是一種 regularization）在小數據量時特別有優勢
5. **Few-shot IL**：如果有預訓練的 VLA（如 Octo），用 10 條 demo 做 fine-tune 就可能夠用 — 預訓練已經學到了通用的操作知識
6. **避開陷阱**：不要用 GAIL 或 IRL — 10 條 demo + 內層 RL 的樣本效率太低，不現實

**結論**：數據增強 + sim 預訓練 + Diffusion Policy。如果有預訓練 VLA，10 條 demo fine-tune 是最高效路線。

</details>

<details>
<summary>Q4：任務有多模態動作（擦桌子順時針或逆時針都對），BC 訓練後機器人卡在中間不動。怎麼解決？</summary>

**完整推理鏈**：

1. **根因：mode averaging**。MSE loss 最小化 $\|a_{pred} - a_{expert}\|^2$ → 順時針 $(+1)$ 和逆時針 $(-1)$ 的平均是 $0$（不動）→ 最差行為
2. **確認是 mode averaging**：visualize 策略輸出的動作分布 — 如果集中在兩個 mode 的中間（而非任何一個 mode），就是 mode averaging
3. **方案一：Diffusion Policy**：天然支持多模態 — 不同噪聲初始化收斂到不同 mode。推理時隨機選一個 mode 執行，不會卡中間
4. **方案二：GMM (Gaussian Mixture Model) head**：策略輸出 $K$ 個 Gaussian 的參數 $(\mu_k, \sigma_k, \pi_k)$，loss 是 negative log-likelihood。訓練時能保留多個 mode
5. **方案三：CVAE (Conditional VAE)**：用 latent variable $z$ 編碼 mode 選擇，不同 $z$ 解碼到不同動作 mode
6. **方案四：過濾數據**：如果多模態不是必要的（例如順逆時針效果一樣好），在數據收集時統一一種模式。犧牲靈活性換穩定性
7. **避開陷阱**：不要試圖用更多數據解決 — 更多「一半左一半右」的數據只會讓平均更穩定地收斂到中間

**結論**：Diffusion Policy 是多模態場景的首選。GMM head 和 CVAE 也可以，但 Diffusion 在高維連續動作空間的表現最好。

</details>

## 面試角度

1. **Distribution Shift 是 BC 的致命傷** — 必考概念。帶出：「BC 的累積誤差和 horizon 平方成正比。單步偏一點點，100 步後完全跑偏。因為策略犯錯 → 到了沒訓練過的狀態 → 不知道怎麼修 → 越偏越遠。DAgger 的核心是在**學生的分布**上收集**專家的標註**，把 $O(T^2)$ 降到 $O(T)$。」

2. **DAgger 的核心機制** — 區分「知道 DAgger」和「理解為什麼有效」。帶出：「DAgger 不是簡單的加數據重訓。關鍵是學生用自己的策略跑環境（暴露錯誤），然後由專家在**學生到達的狀態**標註正確動作。這消除了 train-test distribution mismatch — 訓練數據的分布和部署時的分布一致了。」

3. **IRL 歧義性與 GAIL 的對偶關係** — 展現理論深度。帶出：「IRL 本質有歧義 — 無限多個 reward 能解釋同一組示範。GAIL 繞過了這問題：不顯式學 reward，用 GAN 直接匹配軌跡分布。Ho & Ermon 證明 GAIL 等價於在 occupancy measure 空間做 JS divergence 最小化。」

4. **Diffusion Policy 解決多模態** — 展現前沿實作意識。帶出：「MSE loss 的 BC 對多模態動作會 mode averaging — 擦桌子順時針逆時針平均成不動。Diffusion Policy 用去噪過程生成動作，不同噪聲初始化收斂到不同 mode。加上 action chunking 輸出多步動作序列，是 2023-2024 年 IL 最重要的突破。」

5. **IL + RL 混合是實務首選** — 展現全局視野。帶出：「IL 和 RL 是互補的：IL 需要示範不需要 reward，RL 需要 reward 不需要示範。實務中先用 BC/DAgger warm start 一個不錯的初始策略，再用 PPO/SAC fine-tune 超越專家。或者用 IRL 從 demo 學 reward，再用 RL 在學到的 reward 下訓練。這比純 RL 從零開始快一個數量級。」

## 延伸閱讀

- **Ross & Bagnell,《Efficient Reductions for Imitation Learning》(AISTATS 2010)** — BC $O(T^2)$ 下界的原論文，distribution shift 數學根源的嚴格分析
- **Ross, Gordon & Bagnell,《A Reduction of Imitation Learning and Structured Prediction to No-Regret Online Learning》(AISTATS 2011)** — DAgger 原論文，給出 $O(T)$ 線性上界與 $\beta$-mixing schedule，IL 領域的基石
- **Ho & Ermon,《Generative Adversarial Imitation Learning》(2016)** — GAIL 論文，用 GAN 統一 IRL + RL，證明了和 occupancy measure matching 的對偶關係
- **Chi et al.,《Diffusion Policy: Visuomotor Policy Learning via Action Diffusion》(2023)** — Diffusion Policy 論文，用擴散模型做策略表示，解決多模態動作問題
- **Zhao et al.,《Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware》(2023)** — ACT 論文，action chunking + Transformer，雙臂操作突破性結果
- **Ziebart et al.,《Maximum Entropy Inverse Reinforcement Learning》(2008)** — MaxEnt IRL 原論文，IRL 歧義問題的經典解法
- **robomimic 框架（NVIDIA）** — IL benchmark + 實作框架，支持 BC/BC-RNN/HBC，有完整 evaluation pipeline
- **LeRobot (Hugging Face)** — 開源機器人學習平台，支持 ACT、Diffusion Policy 訓練和部署，配合低成本硬體
- **Mandlekar et al.,《What Matters in Learning from Offline Human Demonstrations for Robot Manipulation》(2022)** — 系統性比較 BC 變體的 ablation study，工程選型的重要參考
