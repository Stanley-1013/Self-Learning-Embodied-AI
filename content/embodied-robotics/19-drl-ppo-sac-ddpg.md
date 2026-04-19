---
title: "深度強化學習（PPO, SAC, DDPG）"
prerequisites: ["18-rl-mdp-basics"]
estimated_time: 50
difficulty: 4
tags: ["deep-rl", "ppo", "sac", "ddpg", "actor-critic", "reward-shaping"]
sidebar_position: 19
---

# 深度強化學習（PPO, SAC, DDPG）

## 你將學到

- 能精確區分 DQN / DDPG / TD3 / SAC / PPO 五種演算法的設計動機與適用場景，面試被問到「你們專案為什麼選 SAC 不選 PPO？」能當場答出完整推理鏈
- 遇到「模擬器裡訓了 5000 epoch 還不收斂」這類情境，知道從 reward shaping → 超參 → value loss 三層排查
- 掌握 Actor-Critic 架構、PPO clip 機制、SAC 最大熵框架的物理意義，兩分鐘內講清楚

## 核心概念

### 七個精確定義

**DQN (Deep Q-Network)**：用神經網路近似 Q-table，搭配 Experience Replay（打破時序相關性）和 Target Network（穩定目標值），解決高維離散動作空間的 Q 值估計。**局限**：只能處理離散動作，機器人的連續關節控制用不了。

**DDPG (Deep Deterministic Policy Gradient)**：連續動作空間的 Actor-Critic 演算法。Actor 輸出**確定性動作** $\mu_\theta(s)$（不是分布），Critic 用 $Q_\phi(s,a)$ 打分。探索靠往動作加 OU 噪聲。Off-policy，用 replay buffer 提高樣本效率。

**TD3 (Twin Delayed DDPG)**：針對 DDPG 的 Q 值過估計問題，三個改進：雙 Critic 取最小值、延遲更新 Actor（每 $d$ 步才更新一次）、目標策略平滑化（target action 加噪聲）。是 DDPG 的「穩定補丁」。

**SAC (Soft Actor-Critic)**：最大熵框架下的 off-policy Actor-Critic。目標函數多了熵項 $\alpha \mathcal{H}(\pi)$，鼓勵策略保持多樣化探索。**隨機策略**（輸出分布，不是確定值），溫度 $\alpha$ 自動調節。在機器人領域穩定性遠勝 DDPG。

**PPO (Proximal Policy Optimization)**：On-policy 演算法，用 Clipped Surrogate Objective 取代 TRPO 的 KL 約束來限制策略更新步長。搭配 GAE（Generalized Advantage Estimation）平衡 bias-variance。實作簡單、大量並行環境下收斂快。

**On-policy vs Off-policy**：On-policy（PPO）只能用當前策略生成的資料更新，穩定但樣本效率低；Off-policy（SAC、DDPG）能重用 replay buffer 中的歷史資料，樣本效率高但更新不穩。

**Actor-Critic 架構**：Actor = 策略網路，輸出動作；Critic = 價值網路，替動作打分。比純 Policy Gradient 方差低（Critic 提供 baseline），比純 value-based 方法能處理連續動作。

### 在感知 → 規劃 → 控制閉環的位置

- **節點**：DRL 位於**規劃層**的最高階，產生端到端策略 $\pi(a|s)$ 取代手寫狀態機
- **輸入**：感知系統的狀態向量 $s$（關節角 / 末端位姿 / 點雲特徵 / 相機影像）
- **輸出**：動作 $a$（關節力矩 / 目標關節角 / 末端速度指令）
- **下游**：底層 MPC / PID 做硬即時追蹤 + 安全約束；或 DRL 直接輸出力矩走 Sim-to-Real
- **一句話**：「DRL 讓機器人在 sim 裡百萬次試錯淬鍊出感測器 → 馬達的肌肉記憶」

### 演算法選型對比表

| 特性 | DDPG | TD3 | PPO | SAC |
|------|------|-----|-----|-----|
| 策略類型 | 確定性 | 確定性 | 隨機 | 隨機 |
| On/Off-policy | Off | Off | On | Off |
| 探索機制 | OU 噪聲 | Target smoothing | 策略本身的隨機性 | 最大熵 |
| 樣本效率 | 高 | 高 | 低 | 高 |
| 訓練穩定性 | 差 | 中 | 好 | 好 |
| 超參敏感度 | 非常高 | 高 | 中 | 低（$\alpha$ 自動調） |
| 機器人首選場景 | 已棄用 | 簡單連續控制 | 大規模並行 sim | 通用首選 |

### 五個核心數學公式

1. **Policy Gradient（策略梯度定理）**：

$$
\nabla_\theta J(\theta) = \mathbb{E}_{\pi_\theta} \left[ \nabla_\theta \log \pi_\theta(a|s) \cdot Q^{\pi}(s,a) \right]
$$

**物理意義**：按 Critic 的打分來調整動作出現的機率 — 得分高的動作，增大其 log 機率；得分低的，壓低。「表現好的動作被鼓勵、差的被抑制」。

2. **PPO Clipped Surrogate Objective**：

$$
L^{\text{CLIP}}(\theta) = \mathbb{E}_t \left[ \min \left( r_t(\theta) \hat{A}_t, \; \text{clip}(r_t(\theta), 1-\varepsilon, 1+\varepsilon) \hat{A}_t \right) \right]
$$

其中 $r_t(\theta) = \frac{\pi_\theta(a_t|s_t)}{\pi_{\theta_{\text{old}}}(a_t|s_t)}$ 是新舊策略的機率比。

**物理意義**：$r_t$ 衡量「策略改了多少」，$\varepsilon$ 是安全繩（通常 0.2）。更新太大就裁剪回來，防止一步跳太遠爆掉。信任域的工程實現。

3. **SAC 最大熵目標**：

$$
J(\pi) = \mathbb{E}_\pi \left[ \sum_{t=0}^{\infty} \gamma^t \left( r_t + \alpha \mathcal{H}(\pi(\cdot|s_t)) \right) \right]
$$

**物理意義**：不只追求最大累積獎勵，還要策略盡量「不確定」— 熵項 $\alpha \mathcal{H}$ 懲罰過早收斂到單一行為，保持探索。溫度 $\alpha$ 自動調節：訓練初期 $\alpha$ 大（多探索），後期自動降低（收斂到最優）。

4. **TD Error（時序差分誤差）**：

$$
\delta_t = r_t + \gamma V(s_{t+1}) - V(s_t)
$$

**物理意義**：「這一步的實際回報 + 未來估計」vs.「原本對這個狀態的估計」之差。正的 $\delta$ 代表比預期好（驚喜），負的代表失望。Critic 靠最小化 $\delta^2$ 來學。

5. **GAE（Generalized Advantage Estimation）**：

$$
\hat{A}_t = \sum_{l=0}^{\infty} (\gamma \lambda)^l \delta_{t+l}
$$

**物理意義**：$\lambda$ 在 0 和 1 之間控制 bias-variance trade-off。$\lambda=0$ 只看一步 TD（低方差高偏差），$\lambda=1$ 等同 Monte Carlo（高方差低偏差）。PPO 預設 $\lambda=0.95$，往低方差略偏。

<details>
<summary>深入：DQN → DDPG → TD3 → SAC 演化鏈與架構圖</summary>

### 演化脈絡

```
Q-Learning (表格)
    │ 高維狀態空間 → 用 NN 近似
    ▼
DQN (2013, DeepMind)
  ├─ Experience Replay: 打破時序相關性
  ├─ Target Network: 穩定目標 Q
  └─ 問題：只能離散動作
    │ 連續動作空間 → 用 Actor 直接輸出
    ▼
DDPG (2015, DeepMind)
  ├─ 確定性 Actor μ(s) + Critic Q(s,a)
  ├─ OU 噪聲探索
  ├─ Soft target update: θ' ← τθ + (1-τ)θ'
  └─ 問題：Q 值過估計 → 不穩定
    │ 修補三大問題
    ▼
TD3 (2018, Fujimoto)
  ├─ Twin Critic: 取 min(Q₁, Q₂) 壓過估計
  ├─ Delayed Actor Update: 每 d 步才更新 Actor
  ├─ Target Policy Smoothing: target action 加噪聲
  └─ 問題：確定性策略探索能力有限
    │ 加入最大熵框架
    ▼
SAC (2018, Haarnoja)
  ├─ 隨機策略 π(a|s) = tanh(μ + σ·ε)
  ├─ 自動溫度 α 調節 → 不用手調探索量
  ├─ Twin Critic (繼承 TD3)
  └─ 當前機器人連續控制的預設選擇
```

### 為什麼 SAC 在機器人領域勝出

1. **探索穩定**：最大熵框架天然鼓勵多樣動作，不需要手調噪聲參數
2. **超參不敏感**：溫度 $\alpha$ 自動調節，比 DDPG 的 OU 噪聲、TD3 的 target smoothing 標準差都好設
3. **多模態策略**：隨機策略能學到多種解法（例如左繞右繞都行的路徑），確定性策略只會選一條
4. **Off-policy 樣本效率**：真機資料珍貴，replay buffer 可反覆利用

### PPO 的定位

PPO 在**大規模並行模擬器**（Isaac Gym 幾千個環境同時跑）場景下反而比 SAC 更實用：
- On-policy 不需要 replay buffer，記憶體開銷恆定
- GPU 並行蒐集大量 on-policy 資料，抵消樣本效率劣勢
- 實作簡單，debug 容易
- OpenAI 的 RLHF（ChatGPT）也選 PPO，因為 LLM fine-tuning 的 batch 本身就夠大

### 典型超參數參考

| 參數 | PPO | SAC |
|------|-----|-----|
| Learning rate (Actor) | 3e-4 | 3e-4 |
| Learning rate (Critic) | 3e-4 | 3e-4 |
| $\gamma$ (discount) | 0.99 | 0.99 |
| $\varepsilon$ (clip) | 0.2 | — |
| $\lambda$ (GAE) | 0.95 | — |
| $\alpha$ (entropy) | — | 自動調節 |
| Batch size | 2048-8192 | 256 |
| Replay buffer | — | 1e6 |
| Target update $\tau$ | — | 0.005 |

</details>

### 常用工具鏈

| 層級 | 套件 | 用途 |
|------|------|------|
| 演算法庫 | Stable-Baselines3 (SB3) | 最成熟的 PyTorch DRL 庫；`PPO("MlpPolicy", env)` 三行開訓 |
| 極簡參考 | CleanRL | 單檔實作，適合讀 source code 學演算法細節 |
| GPU 並行 Sim | Isaac Gym / Isaac Lab | NVIDIA 的 GPU 物理引擎，數千環境同時跑 PPO |
| 物理引擎 | MuJoCo | DeepMind 的精確接觸動力學模擬器，SAC/TD3 標準 benchmark |
| 通用介面 | Gymnasium (Farama) | OpenAI Gym 的維護版，統一 `env.step()` API |
| 輕量模擬 | PyBullet | 免費開源，適合快速 prototype |
| 訓練監控 | Weights & Biases / TensorBoard | reward curve、value loss、entropy 即時追蹤 |

## 直覺理解

**四個核心類比**：

1. **Actor-Critic = 球員 + 教練**：Actor 是場上做決定的球員，Critic 是場邊看全局打分的教練。教練說「剛才那球傳得好」（正 advantage），球員下次多傳；教練說「那球太冒險」（負 advantage），球員下次避開。

2. **PPO clip = 攀岩安全繩**：策略更新像攀岩，你想爬高（提高 reward），但安全繩（$\varepsilon=0.2$）限制你每步最多偏離 20%。TRPO 是嚴格的 KL 約束（精密安全繩），PPO 用 clip 近似（工業安全繩），更好實作但效果幾乎一樣。

3. **SAC 最大熵 = 探索家精神**：普通 RL 是「找到一條路就走到死」，SAC 是「我要找到所有能走的路，挑最好的」。熵項讓策略不敢太篤定，就像有經驗的探險家不會只走一條路線。

4. **Experience Replay = 翻舊筆記**：Off-policy 演算法把過去的 $(s,a,r,s')$ 存在 replay buffer 裡反覆學。就像考前翻以前做過的題目，但不是照原本順序翻（隨機取樣打破相關性）。

**模擬器可觀察的現象**（TensorBoard / W&B 監控）：

- **clip ratio 集中在 1 附近** → learning rate 太小或探索不夠，策略幾乎沒在動
- **SAC entropy 先高後降** → 正常行為：初期大量探索，收斂時維持微小熵
- **reward 劇烈震盪** → 兩大嫌疑：reward function 有 bug（某些狀態給了巨大正/負獎勵）或 learning rate 太大
- **value loss >> policy loss** → Critic 評估困難，通常是 POMDP（觀測不完整）或稀疏獎勵導致

## 實作連結

**三個典型工程場景**：

1. **Isaac Gym 大規模並行 PPO 訓練**：在 GPU 上同時跑 4096 個四足機器人環境，每個 env step 收集一次 on-policy 資料，PPO 每 epoch 更新一次策略。40 分鐘完成百萬步訓練（傳統 CPU sim 需要數天）。

2. **SAC + MuJoCo 桌面操作任務**：單個 MuJoCo 環境跑 SAC，replay buffer 存 1M 筆 transitions。訓練 50 萬步後，機械臂能把方塊推到目標位置。Off-policy 的好處：真機收集的少量 demo 可以直接丟進 buffer 做 fine-tuning。

3. **分層控制部署（Sim-to-Real）**：高層 RL policy 以 20 Hz 輸出目標關節角，底層 PID/MPC 以 1 kHz 追蹤關節角 + 力矩限制。RL 只管「想去哪」，底層保證「安全到達」。

**Code 骨架**（SB3 + Gymnasium）：

```python
# 標準 SAC 訓練流程
import gymnasium as gym
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import EvalCallback

env = gym.make("FetchReach-v3")  # 連續動作的機械臂任務
eval_env = gym.make("FetchReach-v3")

model = SAC(
    "MultiInputPolicy",
    env,
    learning_rate=3e-4,
    buffer_size=1_000_000,
    batch_size=256,
    tau=0.005,          # soft target update 係數
    gamma=0.99,
    ent_coef="auto",    # α 自動調節 ← SAC 核心特色
    verbose=1,
    tensorboard_log="./logs/",
)

eval_cb = EvalCallback(eval_env, eval_freq=5000, best_model_save_path="./best/")
model.learn(total_timesteps=500_000, callback=eval_cb)
```

<details>
<summary>深入：完整 Isaac Gym PPO 訓練範例（從環境定義到部署）</summary>

```python
# Isaac Gym + rl_games 的 PPO 訓練 pipeline
# 參考 NVIDIA IsaacGymEnvs 框架

import isaacgym  # 必須先 import
import torch
from isaacgymenvs.tasks import isaacgym_task_map
from rl_games.algos_torch import torch_ext
from rl_games.common import env_configurations, vecenv
from rl_games.common.algo_observer import AlgoObserver
from rl_games.torch_runner import Runner

# ──────────────────────────────────────────────
# 1. 環境設定（以 Ant 四足為例）
# ──────────────────────────────────────────────
task_cfg = {
    "name": "Ant",
    "physics_engine": "physx",
    "env": {
        "numEnvs": 4096,              # GPU 上同時跑 4096 個環境
        "envSpacing": 5.0,
        "episodeLength": 1000,
    },
    "sim": {
        "dt": 1.0 / 60.0,            # 60 Hz 物理步長
        "substeps": 2,
        "physx": {
            "num_threads": 4,
            "solver_type": 1,         # TGS solver
            "use_gpu": True,
        },
    },
}

# ──────────────────────────────────────────────
# 2. PPO 超參數
# ──────────────────────────────────────────────
ppo_cfg = {
    "algo": {
        "name": "a2c_continuous",     # rl_games 內部 PPO 實作
    },
    "network": {
        "name": "actor_critic",
        "mlp": {
            "units": [256, 128, 64],  # 3 層 MLP
            "activation": "elu",
        },
    },
    "config": {
        "gamma": 0.99,
        "tau": 0.95,                  # GAE λ
        "e_clip": 0.2,               # PPO clip ε
        "learning_rate": 3e-4,
        "lr_schedule": "adaptive",    # KL-based LR 衰減
        "kl_threshold": 0.008,
        "mini_epochs": 5,            # 每批資料重用 5 次
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
# 3. Reward Shaping（四足行走範例）
# ──────────────────────────────────────────────
def compute_reward(obs_buf, actions, reset_buf):
    """
    獎勵設計原則：
    - 稠密獎勵為主（每步都有反饋）
    - 多項加權，避免單一主導
    """
    # 前進速度獎勵（主目標）
    forward_reward = obs_buf[:, 0] * 2.0  # x 方向速度

    # 存活獎勵（防止摔倒就放棄）
    alive_reward = torch.ones_like(forward_reward) * 0.5

    # 動作平滑懲罰（防止抖動）
    action_penalty = -0.005 * torch.sum(actions ** 2, dim=-1)

    # 關節速度懲罰（防止高速甩動）
    joint_vel_penalty = -0.001 * torch.sum(obs_buf[:, 13:21] ** 2, dim=-1)

    total_reward = forward_reward + alive_reward + action_penalty + joint_vel_penalty
    return total_reward

# ──────────────────────────────────────────────
# 4. 訓練迴圈（簡化版）
# ──────────────────────────────────────────────
runner = Runner()
runner.load(ppo_cfg)
runner.run({
    "train": True,
    "play": False,
    "checkpoint": None,
})
# 訓練完成後模型存在 runs/ 目錄
# 用 runner.run({"play": True, "checkpoint": "path/to/model.pth"}) 測試

# ──────────────────────────────────────────────
# 5. Sim-to-Real 部署考量
# ──────────────────────────────────────────────
# - Domain Randomization: 訓練時隨機化摩擦 [0.5, 1.5]、質量 ±15%、
#   觀測噪聲 ±5%、actuator delay 1-3 步
# - 分層部署: RL policy @ 20Hz → PD controller @ 1kHz
# - 動作限幅: torch.clamp(action, -1, 1) * max_torque
# - 安全監控: 若關節速度超限 → 切回 PD 零位
```

**關鍵設計決策解說**：

- **4096 個環境**：GPU 上的大規模並行是 PPO 能跟 off-policy 方法拼速度的關鍵。環境數 × horizon_length 決定每次 update 的 batch size
- **mini_epochs = 5**：同一批資料重用 5 次（PPO 允許少量重用，但不能像 SAC 重用上萬次）
- **adaptive LR**：監控 KL divergence，自動調整 learning rate。KL 太大 → 降 LR（更新太激進），KL 太小 → 升 LR（學太慢）
- **reward shaping**：四項加權獎勵，每項都有物理意義。前進是主目標，其他是約束。權重需要反覆 tune

</details>

## 常見誤解

1. **「DRL 不需要調參，丟進去就能學」** — 現實：DRL 對超參數**極度敏感**。learning rate 差一個量級、clip 值從 0.2 改 0.3、reward 少一項懲罰，都可能從收斂變發散。SB3 的預設值只是起點，實際任務幾乎一定要 tune。

2. **「可以直接在真機上訓 DRL」** — 現實：百萬級 episode 的試錯在真機上不可行（壽命、安全、時間）。標準流程是 **Sim → Domain Randomization → Sim-to-Real Transfer**。真機只用來 fine-tune 或做 System ID 更新 sim 參數。

3. **「稀疏獎勵（到達目標才給 +1）就能學會」** — 現實：稀疏獎勵在高維連續空間幾乎必定失敗。策略隨機探索碰到目標的機率趨近零。必須設計**稠密的中間獎勵**（距離、姿態對齊等），或用 HER（Hindsight Experience Replay）事後修改 goal。

4. **「SAC 永遠比 PPO 好」** — 現實：在 Isaac Gym 這種數千環境 GPU 並行場景，PPO 的 on-policy 不需要 replay buffer、記憶體恆定、debug 容易。大規模並行消除了樣本效率劣勢。SAC 更適合單環境或少量真機資料的場景。

5. **「Off-policy 一定比 on-policy 省時間」** — 現實：off-policy 樣本效率高，但每次 gradient step 需要從 buffer 取樣、計算雙 Critic。當 GPU 並行環境數夠多時，on-policy 的 wall-clock time 反而可能更快（不需要維護巨大 buffer）。

6. **「DRL 一定比 PID 好」** — 現實：對於單純的軌跡追蹤、穩態控制，調好的 PID/MPC 更可靠、更可解釋、延遲更低。DRL 的價值在**高維決策 + 非結構化環境**（地形適應、靈巧操作、多模態感知）。工程上常見的是**分層架構**：DRL 做高層決策，PID/MPC 做底層追蹤。

## 練習題

<details>
<summary>Q1（中）：你要訓練一隻四足機器人在不平地形上行走，DDPG、PPO、SAC 三選一，怎麼分析？</summary>

**完整推理鏈**：

1. **任務特徵分析**：
   - 動作空間：12 個關節力矩，連續高維 → 排除 DQN
   - 環境特性：地形隨機、需要多樣化步態適應 → 需要強探索能力
   - 模擬器設定：有 Isaac Gym，能跑數千並行環境

2. **逐一篩選**：
   - **DDPG**：排除。確定性策略探索能力差，面對隨機地形容易卡在局部最優（例如只學會在平地走，遇到坡就倒）
   - **PPO**：可行。大量並行環境下 on-policy 資料充足，訓練穩定，debug 友善。NVIDIA 的 IsaacGymEnvs 官方範例就是用 PPO 訓四足
   - **SAC**：最適合。最大熵框架天然鼓勵多樣化步態（左腳先、右腳先都探索），off-policy 在環境數較少時樣本效率更高

3. **最終選擇**：
   - 若有 Isaac Gym 大規模 GPU 並行 → **PPO**，簡單穩定，40 分鐘出結果
   - 若只有 MuJoCo 單環境 → **SAC**，樣本效率高，探索能力強
   - **不選 DDPG**：已被 TD3/SAC 全面取代

**面試官想聽到**：不是背答案，而是**根據任務特徵（動作空間、探索需求、sim 條件）做系統化選型**，且知道 DDPG 在 2024 年已經不是首選。

</details>

<details>
<summary>Q2（中-難）：PPO 訓練 5000 epoch 不收斂，reward 在 -200 附近震盪，怎麼排查？</summary>

**完整推理鏈**：

1. **第一層：Reward 本身有 bug**（最常見）：
   - 檢查 reward function 的每一項，print 出各項的數值分布
   - 常見 bug：某個 penalty 項量級遠大於主獎勵（例如 action penalty 權重設太大，策略學到「不動最好」）
   - 驗證：把 reward 各項拆開畫圖，看哪一項主導

2. **第二層：超參數問題**：
   - **learning rate 太大** → reward 劇烈震盪。試降到 1e-4
   - **clip $\varepsilon$ 太大** → 策略每步改太多。試從 0.2 降到 0.1
   - **GAE $\lambda$ 太高** → 方差太大。試降到 0.9
   - **batch size 太小** → advantage 估計不準。PPO 推薦 2048+

3. **第三層：Value loss 遠大於 policy loss**：
   - Critic 無法學好 → 觀測資訊不夠（POMDP 問題）
   - 解法：增加觀測維度（加入歷史 frame 或速度估計）、用 LSTM/Transformer 處理序列觀測
   - 或者 reward 太稀疏，Critic 看不到有意義的信號

4. **系統化排查順序**：reward bug → 超參 → 觀測設計 → 網路架構

**面試官想聽到**：有清晰的除錯層次（不是亂試），先排除最低成本的問題（reward bug 只需看 code），再動超參，最後才動架構。

</details>

<details>
<summary>Q3（難）：設計一個 peg-in-hole（軸孔裝配）任務的 reward function，要避免 reward hacking。</summary>

**完整推理鏈**：

1. **稠密獎勵設計**（從粗到精的引導）：
   - **距離獎勵**：$r_{\text{dist}} = -\|p_{\text{peg}} - p_{\text{hole}}\|_2$ — 越靠近洞口獎勵越高
   - **姿態對齊獎勵**：$r_{\text{align}} = \hat{z}_{\text{peg}} \cdot \hat{z}_{\text{hole}}$ — 軸方向和洞方向的 cosine 相似度
   - **接觸力懲罰**：$r_{\text{force}} = -\lambda \|F_{\text{contact}}\|_2$ — 避免暴力撞擊
   - **成功稀疏大獎**：$r_{\text{success}} = +100$ — 完全插入後給一次大獎

2. **權重設計原則**：
   - 距離獎勵的量級 > 姿態獎勵 > 力懲罰（粗對齊先學會，再學精對齊）
   - 成功大獎要夠大，但不能大到策略為了碰運氣一次而忽略其他項

3. **Reward Hacking 防禦**：
   - **抖動騙分**：策略可能學到在洞口附近快速抖動來反覆獲得「接近目標」的獎勵 → 加 **jerk 懲罰** $r_{\text{jerk}} = -\mu \|\ddot{a}\|_2$
   - **卡在邊緣**：策略把 peg 卡在洞邊，距離分已經拿滿但不願冒風險插入 → 加 **插入深度獎勵** $r_{\text{depth}} = k \cdot d_{\text{insertion}}$
   - **暴力通過**：用很大力直接撞進去 → 接觸力懲罰 + 力矩限幅

4. **完整 reward function**：

$$
r = w_1 r_{\text{dist}} + w_2 r_{\text{align}} + w_3 r_{\text{force}} + w_4 r_{\text{jerk}} + w_5 r_{\text{depth}} + r_{\text{success}}
$$

**面試官想聯到**：reward shaping 是 DRL 落地最難的工程問題之一，能講出「防 hacking」的具體策略（不只是設計獎勵，還要防堵策略鑽漏洞）。

</details>

<details>
<summary>Q4（難）：Sim 裡訓好的策略在真機上高速甩手臂，有安全風險。怎麼處理？</summary>

**完整推理鏈**：

1. **問題根源**：Sim 裡沒有真實的力矩限制、關節柔性、碰撞後果。策略學到「甩越快 reward 越高」，但真機上可能撞壞東西或傷人。

2. **軟約束（reward 層面）**：
   - **動作 L2 懲罰**：$r_{\text{smooth}} = -\alpha \|a\|_2^2$ — 抑制大力矩輸出
   - **Jerk 懲罰**：$r_{\text{jerk}} = -\beta \|a_t - a_{t-1}\|_2^2$ — 抑制急加速/急減速
   - **關節速度懲罰**：$r_{\text{vel}} = -\gamma \|\dot{q}\|_2^2$ — 限制關節角速度
   - **問題**：軟約束只是「建議」，策略如果主獎勵夠大，可能忽略懲罰項

3. **硬約束（安全保障層）**：
   - **Control Barrier Function (CBF)**：在 RL 動作輸出後加一層安全投影，若動作會導致離開安全集合，投影回安全邊界
   - **動作限幅**：`action = torch.clamp(action, -max_torque, max_torque)`
   - **速度監控**：若關節角速度超過閾值 → 緊急切回 PD 歸零位

4. **分層控制架構（最佳實踐）**：
   - **高層 RL policy**（20 Hz）：輸出目標關節角或末端位姿
   - **底層 MPC / PD**（1 kHz）：追蹤目標 + 力矩限制 + 碰撞避免
   - RL 只管「想去哪」，底層保證「安全到達」

5. **總結策略**：軟約束（reward 裡加懲罰）+ 硬約束（CBF / 限幅）+ 分層控制（RL 規劃 + MPC 追蹤）= 三層防線

**面試官想聽到**：不是只靠 reward penalty（軟約束可能被忽略），要有**硬約束保障層**的概念（CBF、分層控制），並知道分層架構是業界 Sim-to-Real 的標準做法。

</details>

## 面試角度

1. **PPO vs SAC 選型邏輯** — 證明你不是背演算法名，而是根據場景做工程決策。**帶出**：「大規模 GPU 並行（Isaac Gym 4096 env）我選 PPO，on-policy 不需要 replay buffer、記憶體恆定、debug 容易。單環境或真機資料場景我選 SAC，off-policy 樣本效率高、最大熵探索穩定。DDPG 已經被 TD3/SAC 取代，我不會用。」

2. **Reward Shaping + Hacking 防禦** — DRL 落地最大的工程痛點。**帶出**：「reward 設計我遵循『稠密引導 + 稀疏大獎』原則，但同時防堵 reward hacking — 加 jerk 懲罰防抖動騙分、加深度獎勵防卡在邊緣。每一項 reward 我都會單獨畫圖確認量級合理。」

3. **Actor-Critic 方差降低機制** — 從數學理解到工程直覺。**帶出**：「純 Policy Gradient 的方差太大（REINFORCE 靠 Monte Carlo 回報，每條軌跡的 return 差異極大）。加 Critic 作為 baseline 把方差壓下來，GAE 的 $\lambda$ 再進一步平衡 bias-variance。PPO 的 clip 是在這基礎上限制更新步長。」

4. **Sim-to-Real 分層部署** — 區分「只會在 sim 裡跑 demo」和「真正部署過」的分水嶺。**帶出**：「我用三層架構部署：RL policy 20 Hz 輸出目標、PD/MPC 1 kHz 追蹤 + 安全約束、底層電機驅動。訓練時加 Domain Randomization（摩擦 ±30%、質量 ±15%、觀測噪聲 ±5%）。真機發現 gap 就做 System ID 更新 sim。」

5. **CBF 硬約束 vs Reward 軟約束** — 安全部署的關鍵認知。**帶出**：「reward penalty 只是『建議』，策略如果主獎勵夠大可能忽略。真正安全靠 Control Barrier Function 投影回安全集合，或分層控制的底層限幅。三層防線：reward 軟約束 + CBF 硬約束 + 分層控制。」

## 延伸閱讀

- **《具身智能算法工程師 面試題》Ch4.3 DRL 演算法對比、Ch12.3 Reward Shaping** — 面試高頻考點：PPO/SAC 選型、reward 設計陷阱、on/off-policy trade-off
- **《具身智能算法工程師 面試題》Ch9 Sim-to-Real Transfer** — Domain Randomization、System ID、分層控制部署的完整方法論
- **PPO 原論文 (Schulman et al., 2017) + SAC 原論文 (Haarnoja et al., 2018)** — 讀 PPO 重點看 clip 的推導動機（為什麼 clip 能近似 TRPO），SAC 重點看最大熵目標和 $\alpha$ 自動調節
- **《多模態大模型》Ch6 RLHF** — PPO 在 LLM fine-tuning 的應用，理解 DRL 不只是機器人，也是 AI alignment 的核心工具
- **CleanRL GitHub repo** — 單檔 PPO / SAC / TD3 實作，每行都有註解，適合讀 source code 理解演算法細節
- **Hindsight Experience Replay (HER, Andrychowicz et al., 2017)** — 解決稀疏獎勵的經典方法，搭配 SAC 使用效果最好
- **DreamerV3 (Hafner et al., 2023)** — World Model 方向，在想像中訓練策略，樣本效率再提升一個量級
- **Isaac Lab 官方文檔與範例** — NVIDIA 最新的機器人 RL 訓練框架，Isaac Gym 的繼任者
