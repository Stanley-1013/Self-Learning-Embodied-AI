---
title: "Sim-to-Real 遷移與物理誤差處理"
prerequisites: ["19-drl-ppo-sac-ddpg", "20-imitation-learning-dagger"]
estimated_time: 50
difficulty: 4
tags: ["sim-to-real", "domain-randomization", "system-id", "transfer-learning", "teacher-student"]
sidebar_position: 21
---

# Sim-to-Real 遷移與物理誤差處理

## 你將學到

- 能精確區分 Reality Gap 的三個層面（物理動力學 / 視覺渲染 / 感測器噪聲），面試時不會籠統地說「模擬不夠真」
- 遇到「sim 裡 95% 成功率、真機只剩 30%」這類情境，知道按「動力學 → 延遲 → 摩擦 → 噪聲」的優先級排查，並能在 Domain Randomization、System ID、Domain Adaptation 三策略間做取捨
- 判斷何時用 Teacher-Student distillation 把 sim 特權資訊壓縮成真機可用的 policy

## 核心概念

### 七個精確定義

**Reality Gap（真實差距）**：模擬器與真實世界之間**不可完全消除**的差異，分三個層面 — (1) **物理動力學**：摩擦、接觸剛度、致動器延遲等 (2) **視覺渲染**：光照、材質、相機噪聲 (3) **感測器噪聲**：IMU 漂移、力矩感測偏差。在閉環裡，Reality Gap 是**訓練（Sim）→ 部署（Real）**之間的濾波器：gap 越大，policy 遷移後的性能衰退越嚴重。

**Domain Randomization（DR，域隨機化）**：在訓練時對模擬器參數（質量、摩擦、延遲、光照等）加入隨機擾動，讓 policy 學到對參數變異的魯棒性。核心思想：**讓真實世界成為訓練分佈的子集**。分為**物理 DR**（動力學參數）和**視覺 DR**（紋理、光照、背景）。

**Adaptive Domain Randomization（ADR，自適應域隨機化）**：根據 agent 的即時表現**自動擴展或收縮**隨機化邊界。表現好 → 擴大範圍加難度；表現差 → 縮小範圍讓 agent 先穩住。本質是**天然的課程學習**（curriculum learning）。

**System Identification（SysID，系統辨識）**：給真機施加**激勵軌跡**（掃頻 / 隨機信號），收集輸入-輸出數據，用最小平方法反推物理參數（質量、慣量、摩擦係數）。和 DR 互補：SysID 鎖基準參數，DR 在基準附近擾動。

**Domain Adaptation（DA，域適應）**：在**部署時**（非訓練時）用 GAN 或對抗特徵對齊，把真機觀測映射到 sim 分佈，讓 sim-trained policy 直接接受。和 DR 的區別：DR 在訓練時泛化，DA 在部署時適配。

**Residual Learning（殘差學習）**：$u = u_{\text{model}} + u_{\text{residual\_NN}}$，用解析模型（剛體動力學、PID 控制器）搞定 90% 的可建模部分，NN 只負責學那 10% 的不可建模殘差（非線性摩擦、柔性）。**工程優勢**：解析部分提供安全基線 + 可解釋性，NN 部分可以小、可以快。

**Teacher-Student Distillation（師生蒸餾）**：在 sim 裡訓練一個能存取**特權資訊**（精確接觸力、物體質量、摩擦係數）的 Teacher policy，再訓練一個只用**真機可觀測量**（相機、關節編碼器）的 Student policy 模仿 Teacher 的行為。透過**資訊瓶頸**迫使 Student 從歷史觀測中**隱式推斷**物理參數。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：sim-trained policy + 真機感測器數據
- **輸出**：能在真機上執行的 robust policy 或 adapted 觀測
- **下游**：真機部署的成功率、安全性、泛化能力
- **閉環節點**：橫跨**訓練（Sim）→ 遷移技術 → 真機部署（Real）**，是整個 pipeline 的最後一道品質關卡

### DR / SysID / DA 三策略對比

| 維度 | Domain Randomization | System Identification | Domain Adaptation |
|------|---------------------|-----------------------|-------------------|
| **作用時機** | 訓練時 | 訓練前（參數校準） | 部署時 |
| **核心機制** | 加噪讓 policy 泛化 | 測量讓 sim 更準 | 映射讓觀測對齊 |
| **優點** | 無需真機數據、可大規模並行 | 精確、可解釋 | 不需重新訓練 policy |
| **缺點** | 範圍太大 → 保守崩潰 | 需要真機實驗、會漂移 | GAN 訓練不穩定、需真機數據 |
| **典型組合** | DR + SysID 最常見 | SysID 鎖基準 + DR 擾動 | 視覺任務 + CycleGAN |

### 最少夠用的數學

1. **DR 期望值公式**（policy 在隨機化分佈上的最佳化目標）：

$$
\pi^* = \arg\max_\pi \; \mathbb{E}_{\xi \sim p(\xi)} \left[ \sum_t \gamma^t r(s_t, a_t; \xi) \right]
$$

**物理意義**：$\xi$ 是模擬器參數向量（質量、摩擦、延遲等），$p(\xi)$ 是人為設定的隨機化分佈。這條公式說的是：最佳 policy 不是在某「一組」參數下最優，而是在**整個分佈上的期望 return 最大**。只要真實參數 $\xi_{\text{real}} \in \text{support}(p(\xi))$，理論上 policy 就能 handle 真機。

2. **SysID 最小平方法**（從真機數據反推參數）：

$$
\hat{\xi} = \arg\min_\xi \sum_{t=1}^{N} \left\| y_t - f(u_t; \xi) \right\|^2
$$

**物理意義**：$y_t$ 是真機量測的輸出（關節角速度、末端力），$u_t$ 是施加的激勵輸入，$f$ 是模擬器的前向模型。找一組 $\xi$ 讓 sim 預測盡量逼近真機量測。**持續激勵**（persistent excitation）是前提 — 如果激勵不夠豐富，某些參數不可辨識。

3. **Residual 架構**（解析 + 殘差 NN）：

$$
u(t) = u_{\text{model}}(t) + u_{\text{NN}}(s_t; \theta)
$$

**物理意義**：$u_{\text{model}}$ 是基於剛體動力學或 PID 的解析控制量（佔 90%），$u_{\text{NN}}$ 是小型 NN 輸出的殘差修正（佔 10%，負責非線性摩擦等未建模效應）。工程上等於「讓物理模型打底，NN 只做微調」，大幅降低 NN 需要學的東西量和安全風險。

<details>
<summary>深入：ADR 演算法與課程機制完整流程</summary>

**Adaptive Domain Randomization (ADR)** 的核心是讓隨機化範圍**自動跟著 agent 能力成長**，而非人工調參。OpenAI 在 Rubik's Cube 任務中的 ADR 流程：

**演算法步驟**：

1. **初始化**：每個可隨機化參數 $\xi_i$ 設定初始範圍 $[\xi_i^{\text{lo}}, \xi_i^{\text{hi}}]$，通常很窄（接近 SysID 基準值）

2. **訓練 + 評估循環**：
   - 每 $K$ 個 episode，在當前範圍的**邊界**（boundary）上評估 agent 表現
   - 設定閾值 $\tau$（例如成功率 80%）

3. **範圍調整**：
   ```
   if performance_at_boundary(ξ_i_hi) > τ:
       ξ_i_hi += Δ          # 擴大上界
   if performance_at_boundary(ξ_i_lo) > τ:
       ξ_i_lo -= Δ          # 擴大下界
   if performance_at_boundary(ξ_i_hi) < τ_low:
       ξ_i_hi -= Δ          # 收縮上界
   if performance_at_boundary(ξ_i_lo) < τ_low:
       ξ_i_lo += Δ          # 收縮下界
   ```

4. **天然課程**：新手 agent 的範圍窄（簡單任務），隨能力提升範圍自動擴大（難度增加），**無需人工設計課程進度表**

**為什麼優於手動 DR**：
- 手動設太寬 → policy 過度保守，所有任務都只用最安全的策略，精度崩潰
- 手動設太窄 → policy 過擬合 sim 參數，真機上 fail
- ADR 自動找到「剛好夠難」的 sweet spot，類似 ZPD（最近發展區）

**實作注意**：
- 參數之間可能有交互效應（高質量 + 高摩擦 vs 低質量 + 低摩擦），ADR 通常獨立調每個參數，但 OpenAI 的 Dactyl 論文指出有些參數需要聯合擴展
- 邊界評估的 episode 數量影響穩定性，太少會因為隨機性誤判
- ADR 天然和 PPO 搭配（on-policy 每次 rollout 重新採 $\xi$）；off-policy 方法需要注意 replay buffer 裡的 $\xi$ 分佈已過時

**典型參數清單**（物理 DR + ADR 可控）：

| 類別 | 參數 | 典型範圍 |
|------|------|----------|
| 動力學 | 連桿質量 | ±30% |
| 動力學 | 摩擦係數 | ±50% |
| 動力學 | 致動器延遲 | 0-80 ms |
| 動力學 | 接觸剛度 | ±40% |
| 視覺 | 光照方向/強度 | 隨機 |
| 視覺 | 紋理/背景 | 隨機替換 |
| 感測器 | 相機噪聲 | 高斯 σ ∈ [0, 0.05] |
| 感測器 | 編碼器偏差 | ±0.5° |

</details>

<details>
<summary>深入：完整 Teacher-Student Pipeline 實作（Python / PyTorch）</summary>

```python
import torch
import torch.nn as nn
import torch.optim as optim

# ── Teacher：能存取 sim 特權資訊 ──
class TeacherPolicy(nn.Module):
    """
    輸入：關節狀態 (q, dq) + 特權資訊 (物體質量, 摩擦, 接觸力)
    輸出：動作 (關節力矩 or 目標位置)
    """
    def __init__(self, obs_dim, privileged_dim, act_dim, hidden=256):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(obs_dim + privileged_dim, hidden),
            nn.ReLU(),
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, act_dim),
        )

    def forward(self, obs, privileged):
        return self.net(torch.cat([obs, privileged], dim=-1))


# ── Student：只用真機可觀測量 ──
class StudentPolicy(nn.Module):
    """
    輸入：關節狀態歷史 (最近 H 步) — 用 LSTM 隱式推斷物理參數
    輸出：動作（模仿 Teacher）
    """
    def __init__(self, obs_dim, act_dim, hidden=256, history_len=50):
        super().__init__()
        self.history_len = history_len
        # 資訊瓶頸：LSTM 將歷史壓縮成低維 latent
        self.encoder = nn.LSTM(obs_dim, hidden, batch_first=True)
        self.head = nn.Sequential(
            nn.Linear(hidden, hidden),
            nn.ReLU(),
            nn.Linear(hidden, act_dim),
        )

    def forward(self, obs_history):
        # obs_history: (batch, history_len, obs_dim)
        _, (h_n, _) = self.encoder(obs_history)
        latent = h_n.squeeze(0)  # (batch, hidden) — 隱式物理參數估計
        return self.head(latent)


# ── 訓練流程 ──
def train_teacher_student(
    env,              # IsaacGym / MuJoCo 環境
    teacher,          # 已訓練好的 Teacher（用 PPO 在 sim 特權模式下訓練）
    student,          # 待訓練的 Student
    epochs=100,
    batch_size=256,
    lr=3e-4,
):
    optimizer = optim.Adam(student.parameters(), lr=lr)
    loss_fn = nn.MSELoss()

    for epoch in range(epochs):
        # Step 1: 用 Teacher 在 sim 中收集 rollout
        obs_history_batch = []
        teacher_action_batch = []

        obs = env.reset()
        history_buffer = []

        for step in range(env.max_steps):
            privileged = env.get_privileged_info()  # sim 才有

            with torch.no_grad():
                teacher_action = teacher(obs, privileged)

            # 記錄歷史 + Teacher 動作
            history_buffer.append(obs.clone())
            if len(history_buffer) >= student.history_len:
                obs_hist = torch.stack(
                    history_buffer[-student.history_len:]
                )
                obs_history_batch.append(obs_hist)
                teacher_action_batch.append(teacher_action)

            obs, _, done, _ = env.step(teacher_action)
            if done:
                obs = env.reset()
                history_buffer = []

        # Step 2: Student 模仿 Teacher
        obs_history_tensor = torch.stack(obs_history_batch)
        teacher_actions = torch.stack(teacher_action_batch)

        student_actions = student(obs_history_tensor)
        loss = loss_fn(student_actions, teacher_actions)

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        if epoch % 10 == 0:
            print(f"Epoch {epoch}, Distillation Loss: {loss.item():.4f}")

    return student


# ── 部署到真機 ──
def deploy_student(student, real_robot, history_len=50):
    """真機只需要 Student + 歷史 buffer"""
    history = []
    obs = real_robot.get_observation()

    while True:
        history.append(torch.tensor(obs))
        if len(history) > history_len:
            history = history[-history_len:]

        if len(history) == history_len:
            obs_hist = torch.stack(history).unsqueeze(0)
            with torch.no_grad():
                action = student(obs_hist)
            real_robot.send_command(action.numpy())

        obs = real_robot.get_observation()
```

**關鍵設計決策**：
1. **為什麼用 LSTM 而非 MLP？** — Student 需要從時序觀測中推斷物理參數（質量、摩擦），這些資訊散佈在歷史軌跡裡，MLP 看不到時間相關性
2. **history_len 怎麼選？** — 太短（<10）推斷不準，太長（>100）LSTM 梯度衰減。實務上 50 步通常夠用
3. **損失函數的選擇** — MSE 最簡單；也可以用 Teacher 的 value function 做加權（高 reward 區域的動作更重要）
4. **DAgger 變體** — 進階做法：讓 Student 自己 rollout，Teacher 標註 Student 的觀測（不是 Teacher 自己的軌跡），解決 distribution shift

</details>

## 直覺理解

**三個類比**：

1. **DR = 極端天氣駕訓班**：在大太陽、暴雨、大霧、冰路上全練過的駕駛員，上路後什麼天氣都能應對。但如果訓練範圍太極端（颱風 + 地震 + 海嘯同時來），學生會崩潰成「什麼都不敢做」的保守策略。

2. **SysID = 調校飛行模擬器**：買了一台波音模擬器，先拿真飛機的飛行紀錄去校準模擬器的空氣動力學參數（升力係數、阻力曲線），讓模擬器的反應盡量逼近真機。校準完再在模擬器上練飛行員。

3. **DA = 美圖秀秀濾鏡**：不改模型、不改訓練，而是在輸入端「美化」真機影像 → 讓它看起來像 sim 的風格 → sim-trained policy 就能直接處理。CycleGAN 就是這個「風格轉換濾鏡」。

**模擬器觀察**：在 Isaac Gym 裡同時開 1000 個環境，每個環境的物理參數（質量 ±30%、摩擦 ±50%）都不同 — 看 policy 在不同「世界設定」下的表現方差。方差越小 → 越 robust → 遷移到真機的機會越高。

## 實作連結

**三個典型工程場景**：

1. **IsaacGym 大規模 DR 訓練**：同時運行數千個平行環境，每個環境在 reset 時從 $p(\xi)$ 採樣新參數。PPO 的 rollout buffer 天然混合了不同物理設定的經驗，無需額外 code。

2. **真機 SysID 迴圈**：用 UR5 跑掃頻激勵 → 記錄 `/joint_states`（角度、角速度、力矩）→ 用 `scipy.optimize.least_squares` 反推慣量 + 摩擦 → 更新 MuJoCo XML 的 `<body mass>` 和 `<joint damping>` → 重新訓練 policy。

3. **Teacher-Student for 靈巧手操作**：Teacher 在 sim 裡存取精確的物體接觸力和質量 → 訓練出 expert policy → Student 只用指尖觸覺感測器和關節編碼器的歷史序列 → LSTM 隱式推斷物理參數 → 部署到 LEAP Hand 真機。

**Code 骨架**（Python，DR 訓練配置）：

```python
# IsaacGym DR 參數配置骨架
dr_config = {
    "mass_range": [0.7, 1.3],          # 質量 ±30%
    "friction_range": [0.3, 1.5],       # 摩擦係數
    "actuator_delay": [0, 0.08],        # 延遲 0-80ms
    "observation_noise": {
        "joint_pos": 0.005,             # rad
        "joint_vel": 0.05,              # rad/s
    },
}

def randomize_env_params(env, dr_config):
    """每次 env.reset() 時呼叫，採樣新參數"""
    import numpy as np
    mass_scale = np.random.uniform(*dr_config["mass_range"])
    friction = np.random.uniform(*dr_config["friction_range"])
    delay = np.random.uniform(*dr_config["actuator_delay"])
    env.set_body_mass_scale(mass_scale)
    env.set_friction(friction)
    env.set_actuator_delay(delay)
```

<details>
<summary>深入：完整 SysID Pipeline（Python + scipy）</summary>

```python
import numpy as np
from scipy.optimize import least_squares

def generate_excitation_trajectory(duration=10.0, dt=0.001, n_joints=6):
    """
    產生掃頻 + 隨機信號的激勵軌跡
    要求：持續激勵 (persistent excitation) — 覆蓋所有頻率分量
    """
    t = np.arange(0, duration, dt)
    trajectories = []
    for j in range(n_joints):
        # 掃頻：0.1 Hz → 10 Hz
        freq = np.linspace(0.1, 10, len(t))
        chirp = 0.5 * np.sin(2 * np.pi * np.cumsum(freq) * dt)
        # 疊加小幅隨機
        noise = 0.1 * np.random.randn(len(t))
        trajectories.append(chirp + noise)
    return t, np.array(trajectories).T  # (N, n_joints)


def forward_dynamics_model(params, u_sequence, dt):
    """
    簡化的前向動力學：τ = M(q) * ddq + C(q, dq) * dq + friction
    params: [inertia_1, ..., friction_1, ..., damping_1, ...]
    """
    n_joints = u_sequence.shape[1]
    n = len(params) // 3
    inertias = params[:n]
    frictions = params[n:2*n]
    dampings = params[2*n:3*n]

    q = np.zeros(n_joints)
    dq = np.zeros(n_joints)
    predicted = []

    for u in u_sequence:
        ddq = (u - frictions * np.sign(dq) - dampings * dq) / inertias
        dq += ddq * dt
        q += dq * dt
        predicted.append(dq.copy())

    return np.array(predicted)


def system_identification(u_data, y_data, dt, n_joints=6):
    """
    u_data: 施加的力矩 (N, n_joints)
    y_data: 量測的角速度 (N, n_joints)
    """
    # 初始猜測
    x0 = np.ones(n_joints * 3)  # [inertias, frictions, dampings]

    def residual(params):
        y_pred = forward_dynamics_model(params, u_data, dt)
        return (y_pred - y_data).flatten()

    result = least_squares(
        residual, x0,
        bounds=(0.01, 100),  # 物理參數必須正
        verbose=1,
    )

    identified = {
        "inertias": result.x[:n_joints],
        "frictions": result.x[n_joints:2*n_joints],
        "dampings": result.x[2*n_joints:],
        "residual_norm": result.cost,
    }
    return identified


def update_mujoco_xml(xml_path, identified_params):
    """把辨識結果寫回 MuJoCo XML"""
    import xml.etree.ElementTree as ET
    tree = ET.parse(xml_path)
    root = tree.getroot()
    for i, body in enumerate(root.iter("body")):
        if i < len(identified_params["inertias"]):
            inertial = body.find("inertial")
            if inertial is not None:
                inertial.set("mass", str(identified_params["inertias"][i]))
    tree.write(xml_path)
    print(f"Updated {xml_path} with identified parameters")
```

**SysID 實務陷阱**：
- **持續激勵不足**：如果只給正弦波，只能辨識該頻率附近的參數。掃頻 + 白噪聲是最基本的要求
- **參數耦合**：質量和慣量常高度耦合，需要多姿態下的數據才能分離
- **漂移**：真機參數會隨溫度、磨損變化，定期 re-calibrate 是必要的（不是「做一次就夠」）

</details>

## 常見誤解

1. **「模擬器越高保真就不需要 DR」** — 錯。即使用最精細的 FEM 模擬器，接觸力學（表面微觀幾何）和致動器非線性永遠無法完美建模。高保真 sim 縮小 gap 但不消除 gap；DR 負責處理殘餘不確定性。**正確理解**：SysID 提升 sim 的基準精度，DR 處理基準周圍的不確定性，兩者互補。

2. **「DR 隨機化範圍越大越好」** — 錯。範圍太大會讓 policy 過度保守 — 它要同時應對「摩擦 0.1」和「摩擦 2.0」，最終選擇最安全（也最慢、最不精確）的策略。OpenAI Rubik's Cube 論文明確指出過度隨機化導致性能崩潰。**正確做法**：先用 SysID 鎖基準，再用 ADR 從窄範圍自動擴展。

3. **「Sim-to-Real 只是視覺問題」** — 錯。視覺差異（渲染 vs 真實相機）容易察覺但通常不是致命的 — CycleGAN / 簡單 DA 就能處理。**動力學差異**（摩擦、接觸、延遲）才是大多數操作任務失敗的主因，且更難診斷。排查優先級：動力學 > 延遲 > 感測器噪聲 > 視覺。

4. **「SysID 做一次就夠了」** — 錯。真機參數會隨溫度（減速器潤滑油黏度）、磨損（齒輪背隙增大）、負載（末端工具更換）而漂移。工業場景通常每季度 re-calibrate，高精度任務甚至每班次做一次快速校準。

5. **「Student 的表現一定比 Teacher 差」** — 不一定。Teacher 依賴精確的 sim 特權資訊，這些資訊在真機上本就不可用且可能有雜訊。Student 被迫從歷史序列中學習 robust 的隱式表徵，有時反而更 robust。資訊瓶頸不是劣勢，而是**正則化**。

## 練習題

<details>
<summary>Q1：你的 RL policy 在 MuJoCo 裡抓取成功率 95%，但部署到 UR5 真機後只剩 30%。你怎麼有系統地排查？</summary>

**完整推理鏈**：

1. **先查延遲**：MuJoCo 預設 0ms 致動器延遲，真機 UR5 有 ~8ms 控制延遲 + 通訊延遲。在 sim 裡加入 8ms delay 重新測成功率 → 如果掉到 50%，延遲是主因
2. **再查摩擦**：MuJoCo 預設摩擦係數 vs 真機夾爪實際摩擦可能差 2-3 倍。真機上觀察物體是否在夾爪中滑動 → 如果是，在 sim 裡降低摩擦係數重測
3. **再查噪聲**：真機 RGB 相機有色彩偏差、運動模糊；關節編碼器有量化噪聲。在 sim 裡加入對應噪聲模型
4. **逐項加入後比對**：每加一個 gap 源，記錄成功率變化，找出**最大貢獻者**
5. **解法**：對最大貢獻者做 SysID（量測真機延遲和摩擦），更新 sim 參數；同時對所有參數加 DR（±20%），重新訓練

**面試官想聽到**：不是一上來就「加 DR」，而是先系統性排查哪個 gap 源貢獻最大，再針對性處理。延遲和動力學通常比視覺差異更致命。

</details>

<details>
<summary>Q2：你的 DR 範圍設為 friction ∈ [0.1, 2.0]，policy 訓練收斂但在真機上動作極慢且保守，抓取精度很差。怎麼診斷和修？</summary>

**完整推理鏈**：

1. **診斷：過度保守**：policy 要同時 handle friction=0.1（超滑）和 friction=2.0（超黏），唯一安全策略是慢慢來 + 用最大夾力 — 這在真機上表現為「動作遲緩、力量過大、缺乏精度」
2. **Step 1 — SysID 鎖基準**：用真機跑滑動實驗（推物體、測力），辨識真實摩擦約 0.6-0.8
3. **Step 2 — 縮窄 DR 到合理範圍**：friction ∈ [0.4, 1.2]（基準 ± 合理不確定性）
4. **Step 3 — 改用 ADR**：初始範圍 [0.55, 0.85]，讓 agent 表現好時自動擴展，表現差時收縮，**自動找到最優難度**
5. **驗證**：真機成功率 + 動作速度 + 夾力大小三指標同時看，不能只看成功率

**面試官想聽到**：理解 DR 範圍不是「越大越好」的原因（保守崩潰 / policy collapse）；知道 SysID 和 ADR 是解決過度隨機化的標準工具組合。

</details>

<details>
<summary>Q3：你的 sim 用 Unity 渲染訓練視覺 policy，但部署到工業相機後視覺輸入差異很大（光照、色溫、鏡頭畸變都不同）。你怎麼處理？</summary>

**完整推理鏈**：

1. **分析差異來源**：(a) 全局光照 — Unity 用 HDR 環境光，工廠是日光燈 (b) 色溫 — sim 偏暖、真實偏冷 (c) 鏡頭畸變 — sim 是完美 pinhole，真機有 barrel distortion (d) 背景 — sim 是乾淨桌面，真機有雜物
2. **三路並行處理**：
   - **視覺 DR（訓練時）**：隨機化光照方向/強度/色溫、隨機貼紋理到桌面和物體、加 Gaussian noise / motion blur / 隨機 crop
   - **CycleGAN DA（部署時）**：收集少量真機影像（~500 張），訓練 real→sim 風格轉換，讓真機畫面「看起來像 sim」
   - **語意 Mask 降維**：不直接用 RGB，先過 segmentation model 提取語意 mask（物體 / 桌面 / 背景），policy 基於 mask 決策 — 完全繞過 appearance gap
3. **鏡頭畸變**：用 OpenCV `undistort` 在前處理階段修正，sim 訓練時也加入隨機畸變參數
4. **選擇順序**：語意 Mask > 視覺 DR > CycleGAN DA — 越前面越穩定、越不需要真機數據

**面試官想聽到**：視覺 gap 不是一個問題而是多個子問題的組合；語意 Mask 降維是最 robust 的方案因為它直接消除 appearance 變異。

</details>

<details>
<summary>Q4：你要訓練一個能泛化抓取 5 種不同物體（形狀、質量、摩擦都不同）的 policy，並部署到真機。你的完整 pipeline 是什麼？</summary>

**完整推理鏈**：

1. **資產準備**：5 種物體的 3D mesh（掃描或 CAD）→ 建 URDF/MJCF → 在 sim 裡載入並驗證碰撞幾何
2. **物理 DR 配置**：
   - 質量：每種物體 ±30%
   - 摩擦：[0.3, 1.5]（涵蓋塑膠到橡膠）
   - 接觸剛度：±40%
   - 致動器延遲：[0, 50ms]
3. **Teacher 訓練**：
   - 特權觀測 = 關節狀態 + **精確物體姿態** + **物體質量** + **摩擦係數** + **接觸力**
   - 用 PPO 在 IsaacGym 訓練（4096 並行環境 x DR），直到所有 5 種物體成功率 > 90%
4. **Student 蒸餾**：
   - 觀測 = 關節狀態歷史（50 步）+ 指尖觸覺（如有）
   - **LSTM encoder** 從歷史中隱式推斷物體的物理屬性（不需要明確的質量/摩擦輸入）
   - 用 Teacher 的動作做監督學習（DAgger 變體更穩）
5. **真機部署前驗證**：
   - 在 sim 裡把 Student 部署到 SysID-calibrated 環境（不是 DR 環境），確認成功率 > 80%
   - 凍結 Student 權重，真機先用已知物體測試，再逐步引入新物體
6. **Residual 微調（可選）**：如果 Student 在某些物體上精度不夠，在真機上用 residual policy 做少量 fine-tuning（基於 Student 的基線 + 小 NN 殘差）

**面試官想聽到**：完整的 pipeline 思維（資產 → DR → Teacher → Student → 部署）；理解 LSTM 的隱式物理推斷是 Teacher-Student 的核心價值；知道 DAgger 解決 distribution shift。

</details>

## 面試角度

1. **DR + SysID 混合策略是業界標準** — 不是二選一，而是 SysID 鎖基準、DR 處理殘餘不確定性。面試時用這句話帶出：「我不會只靠 DR 或只靠 SysID — SysID 給 sim 一個精確的起點，DR 在起點周圍加擾動讓 policy robust。」

2. **Residual Learning 的工程優勢** — 最實際的 sim-to-real 策略之一，因為保留了解析模型的安全性和可解釋性。面試時用這句話帶出：「在工業部署中我偏好 residual 架構 — 90% 的控制量由物理模型出，NN 只補殘差，這樣即使 NN 輸出異常，系統不會完全失控。」

3. **Teacher-Student 的資訊瓶頸設計** — 區分「會用」和「理解原理」的關鍵。面試時用這句話帶出：「Teacher-Student 的核心不是降維，而是迫使 Student 從歷史觀測中隱式推斷物理參數 — LSTM 的 hidden state 其實就是一個 learned system identification 模塊。」

4. **動力學 gap > 視覺 gap 的優先級** — 很多人直覺認為「看起來不像」是主因，但操作任務的失敗通常來自力/摩擦/延遲的不匹配。面試時用這句話帶出：「排查 sim-to-real gap 時我一定先查動力學（延遲、摩擦、接觸），因為視覺差異可以用簡單的 DA 處理，但動力學錯誤會讓 policy 的力控完全失效。」

5. **ADR 是 curriculum learning 的自然形式** — 展示你對訓練策略的深度理解。面試時用這句話帶出：「ADR 本質上是 curriculum learning — agent 表現好就自動加大隨機化難度，表現差就收縮範圍，無需人工設計課程進度表。OpenAI 用它解決了 Rubik's Cube 這種極端 sim-to-real 任務。」

## 延伸閱讀

- **OpenAI,《Solving Rubik's Cube with a Robot Hand》(2019)** — ADR 的經典應用，展示了從窄範圍自動擴展到極端物理變異的完整流程；是理解「為什麼 DR 範圍不能手動設」的最佳案例
- **Tobin et al.,《Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World》(2017)** — DR 的奠基論文，定義了視覺 DR 和物理 DR 的基本框架
- **Rusu et al.,《Sim-to-Real Robot Learning from Pixels with Progressive Nets》(2017)** — progressive neural network 做 sim-to-real 遷移，了解非 DR 路線的替代方案
- **ETH Zurich,《Learning to Walk in Minutes Using Massively Parallel Deep RL》(2022)** — Teacher-Student + IsaacGym 大規模並行訓練的範例，從 sim 到 ANYmal 四足機器人的完整 pipeline
- **《具身智能算法工程師 面試題》Ch9 Sim-to-Real** — 涵蓋 DR / SysID / DA 的面試高頻考點，情境題格式
- **Zhao et al.,《Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: a Survey》(2020)** — 最全面的 survey，分類整理了所有 sim-to-real 策略及其適用場景
- **Isaac Gym / IsaacLab 官方文件** — 大規模並行 DR 訓練的工程實作指南，含 DR 參數配置範例
- **MuJoCo 官方文件 — System Identification section** — MuJoCo 內建的 SysID 工具和參數調校流程
- **Digital Twin 概念與 Sim2Sim 策略** — 當一個 sim 不夠時，先在高保真 sim 驗證再遷移到真機的中間路線
