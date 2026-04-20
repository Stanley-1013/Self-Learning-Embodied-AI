---
title: "模型預測控制 (MPC) 與軌跡追蹤"
prerequisites: ["13-pid-control-tuning", "11-trajectory-optimization"]
estimated_time: 75
difficulty: 4
tags: ["mpc", "nmpc", "srbd", "wbc", "cbf", "tube-mpc", "ci-mpc", "mppi", "lie-group", "offset-free", "event-triggered", "control", "optimization", "real-time"]
sidebar_position: 15
---

# 模型預測控制 (MPC) 與軌跡追蹤

## 你將學到

- 兩句話精確講清楚 MPC 是什麼、它跟 PID / LQR / 離線軌跡最佳化的本質差異，並點出它在「感知 → 規劃 → 控制」閉環的位置
- 能分辨什麼時候用線性 MPC（QP，毫秒級）、什麼時候必須上 NMPC（SQP/RTI/IPOPT），以及為什麼**機械臂不用 MPC 但四足必須用 MPC**
- 面試被問 SRBD + WBC 分層、NMPC 求解器選型、Feasibility/Slack/Fallback、CBF safety filter、Tube vs Stochastic robust MPC、Contact-Implicit MPC、MPPI sampling、Lie Group MPC、Offset-free、Event-triggered 時，每題能在兩分鐘內講清楚關鍵邏輯
- 建立 2024–2025 前沿圖像：Learning-based MPC（GP-MPC / DeepMPC / Meta-MPC / Amortized MPC）、Foundation Model + MPC（VLA 規劃 + MPC 追蹤）、Dreamer World Model + MPC、Diffusion-MPC 的定位與應用場景
- 掌握產業生死線：**70% 工程精力花在 Slack / Fallback / Safety Filter** — 學術玩具 vs 產線產品的本質分水嶺

## 核心概念

**精確定義**：**Model Predictive Control (MPC)** 是一種在線最佳化控制策略 — 每個控制週期，用系統的**預測模型**往前推演 $N$ 步，在**約束**（力矩上限、關節極限、摩擦錐、避障等）下最小化一個**代價函數**（追蹤誤差 + 控制能量），只執行解出的第一步控制量 $u_0^*$，然後下一個週期拿新的狀態量重新求解。這種「**滾動時域 (receding horizon)**」機制讓 MPC 天然具備前瞻性與約束處理能力。

**一句話版本**：「MPC 是會看未來 N 步、會算物理極限的規劃型控制器；PID 只會看當下誤差，LQR 會看未來但不會處理約束。」

**MPC 三要素**：
1. **預測模型** — 離散化動力學 $x_{k+1} = f(x_k, u_k)$（線性或非線性；SRBD 降階或完整浮基）
2. **代價函數** — 狀態追蹤誤差 + 控制能量 + 平滑度（+ 能耗 / 接觸力等）
3. **約束** — 硬性物理限制（力矩飽和、速度極限、避碰、摩擦錐、關節極限、CBF）

**vs 其他控制器（面試常考）**：

| 比較 | 核心差異 |
|------|---------|
| **vs PID** | PID 反應式、看現在和過去、無法處理硬約束；MPC 預測式、看未來 N 步、天生能處理約束 |
| **vs LQR** | LQR 也看未來但**無約束**（代價函數無約束的二次型有閉解）；MPC = **帶約束的 LQR 在線版** |
| **vs Ch11 離線軌跡最佳化** | Ch11 離線算整條最優軌跡；MPC 每個 tick 做一次小型 trajectory optimization 線上重規劃；MPC 相當於「滾動時域的 trajectory opt」 |
| **vs RL** | RL 靠大量離線訓練學 policy；MPC 每步解在線優化。RL 缺硬約束保證，MPC 缺語義泛化 → 2024+ 潮流是「RL/VLA 高層規劃 + MPC 底層追蹤」 |

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：當前狀態估計 $x_k$（感測器 / 狀態估計器）、參考軌跡 $x^{\text{ref}}_{k:k+N}$（規劃器 / 高層 policy）
- **輸出**：當前時刻的最佳控制量 $u_k^*$（力矩 / 速度 / 加速度 / GRF 指令）
- **下游**：驅動器執行 / WBC 分配馬達扭矩 / PID 追蹤
- **閉環節點**：坐落在 **控制層**，但兼具「在線重規劃」的特性，是規劃與控制的橋樑；在具身智能分層架構裡扮演**小腦（10–100 Hz 快預測）**，上接大腦（VLA / LLM 1–10 Hz），下接脊髓（PID / WBC 1 kHz）

### 最少夠用的數學

**1. 線性 MPC 的 QP 標準形式**（工業最常見配置）：

$$
\min_{U} \sum_{k=0}^{N-1} \left[ (x_k - x_k^{\text{ref}})^T Q (x_k - x_k^{\text{ref}}) + u_k^T R\, u_k \right] + (x_N - x_N^{\text{ref}})^T Q_f (x_N - x_N^{\text{ref}})
$$

$$
\text{s.t.} \quad x_{k+1} = A x_k + B u_k, \quad u_{\min} \le u_k \le u_{\max}, \quad x_{\min} \le x_k \le x_{\max}
$$

**物理意義**：$Q$ 懲罰追蹤偏差（越大越緊追），$R$ 懲罰控制能量（越大越省力但追不緊），$Q_f$ 是終端代價（保證有限時域穩定性）。整個問題是 **Quadratic Program (QP)**，OSQP / qpOASES / HPIPM 都能 ms 級解完。

**2. 非線性 MPC (NMPC) 的 NLP 形式**：

$$
\min_{U} \sum_{k=0}^{N-1} \ell(x_k, u_k) + V_f(x_N), \quad \text{s.t. } x_{k+1} = f(x_k, u_k),\; g(x_k, u_k) \le 0
$$

**物理意義**：動力學 $f$ 不再是 $Ax+Bu$，可以是剛體浮基 + 接觸 + 科氏離心項；$g$ 可以是摩擦錐、SDF 避障等非線性不等式。解不出閉解，只能用 SQP / Interior Point 迭代。

**3. 滾動時域實施（receding horizon）**：

$$
u^*_{0:N-1} = \arg\min_{U} J(x_k, U), \quad u_{\text{applied}} = u_0^*, \quad k \leftarrow k+1
$$

**物理意義**：每週期都重解整個 N 步優化，但只執行第一步。下個週期拿新感測狀態當初始條件、**Warm Start** 上週期解做熱啟動 → 這就是 MPC「在線重規劃」的本質。

<details>
<summary>深入：SRBD 降階 + WBC 分層 — 四足機器人為何不能用完整動力學跑 MPC</summary>

### 問題核心：完整浮基動力學 $O(n^4)$，100 Hz 即時解不動

四足機器人狀態維度：
- 浮基底座姿態 + 位置：6 DoF
- 12 關節角度 + 速度：24 DoF
- 合計 **30 維狀態 + 12 維控制**，預測時域 20 步 → 每步 NLP 有 840 變數

NMPC 每個 SQP 迭代需算 Jacobian（$O(n^4)$）、Hessian、KKT 系統。在嵌入式 CPU 上拿完整動力學去解 → 單迭代就超 100 ms → 即時控制迴路崩潰。

### 解法：SRBD 降階（Single Rigid Body Dynamics）

**核心假設**：忽略腿質量（相對機身小），把整台四足**視為單剛體質心 + 四個可切換接觸點**。

動力學簡化為：

$$
m\ddot{p}_c = \sum_{i=1}^{4} f_i + m g, \quad I\dot{\omega} + \omega \times I\omega = \sum_{i=1}^{4} (p_i - p_c) \times f_i
$$

**物理意義**：質心加速度 = Σ 足端 GRF + 重力；角動量變化率 = Σ 足端力對質心力矩。$p_i$ 是足端位置、$f_i$ 是地面反作用力（GRF）。

- 維度降到 **12 維質心狀態 + 12 維 GRF**（原 30+）
- 凸 QP 可在 1–3 ms 解完 → MIT Cheetah、Unitree、ANYmal 標配

### WBC 底層 1 kHz 補回被忽略的腿動力學

MPC 輸出 GRF 指令後，不能直接送馬達（還需分配到每個關節扭矩）。**Whole-Body Control (WBC)** 用**完整浮基動力學** + QP 處理：
- 關節扭矩極限
- 奇異姿態
- 任務優先級（足端力追蹤 > 姿態 > 零空間）

分工哲學：
- **MPC (50–100 Hz)**：**看得遠** — 預測 1–2 秒的質心軌跡 + 足端排程（避障、跑酷落腳點）
- **WBC (1 kHz)**：**走得穩** — 精確力矩分配，處理高頻碰撞與奇異

**面試金句**：「MPC 管質心路線圖，WBC 管每條腿怎麼踩。**兩層一起才是足式機器人能跑能跳的底層骨架**。」

### 相關平台

- **MIT Cheetah**：Convex MPC (SRBD) + WBC，嵌入式 CPU 100 Hz+ 奔跑
- **ANYmal (ETH)**：OCS2 Hierarchical MPC + WBC + 地形估計
- **Unitree Go1/H1**：類似架構，SDK 開放 MPC + WBC 調參介面

</details>

<details>
<summary>深入：NMPC 求解器、RTI、Multiple Shooting、業界框架全景</summary>

### 為什麼 LMPC 在足式 / 操作任務會崩

- **科氏力 / 離心力** ∝ 速度² → 強非線性，高速機動完全失真
- **三角函數幾何映射**（body orientation → world frame）
- **接觸切換 (Hybrid Dynamics)**：腳離地 / 著地動力學不連續
- 線性化工作點一偏離就發散

→ 必上 **NMPC**。

### 兩大 NMPC 求解器流派

**1. SQP (Sequential Quadratic Programming)**
- 把 NLP 分解成一串 QP 子問題
- **Warm Start 極佳** — 上週期解直接餵下週期
- 非線性約束友善
- **硬即時控制首選**（acados / OCS2）

**2. Interior Point (IPOPT)**
- 把不等式轉成對數障礙項，走中心路徑
- 大規模稀疏矩陣友善
- **熱啟動差**：每次要重新走中心路徑
- 常用於概念驗證 / 離線軌跡最佳化（Drake / CasADi）

### 即時化三大技術

**1. Real-Time Iteration (RTI) — Diehl 的殺手鐧**

- **每週期只跑 1 次 SQP 迭代**（不等收斂）
- 仰賴 Warm Start：連續兩週期狀態相近 → 單迭代已夠好
- $O(n^3)$ NLP 壓到**固定 μs 級預算** → 硬即時保證

**2. Multiple Shooting vs Single Shooting**

- **Single Shooting**：只把控制 $u$ 當決策變數，狀態 $x$ 靠積分推出
  - 非線性誤差累積 → 不穩定系統（倒立擺、四足）數值爆炸
- **Multiple Shooting**：$x$ 和 $u$ 都當決策變數，加連續性約束 $x_{k+1} = F_{\text{RK4}}(x_k, u_k)$
  - 誤差限於小區間、並行友善
  - **四足 / 人形 NMPC 標配**

**3. Condensing vs Sparse**

- Multiple Shooting 產生稀疏大矩陣
- **Condensing**：消去中間 $x$ 得稠密 QP（qpOASES 擅長）
- **Sparse**：保留稀疏結構（HPIPM 擅長）
- 維度高時 sparse 贏、低時 condensing 贏

### 業界框架對照

| 框架 | 特色 | 適用 |
|------|------|------|
| **acados** | C++ + RTI + HPIPM，工業硬即時首選 | ANYmal / 無人機 / 自駕 |
| **CasADi + IPOPT** | 符號微分，概念驗證快 | 研究 / 離線 |
| **OCS2 (ETH)** | SLQ / DDP，接觸切換特化 | 足式機器人 |
| **Drake** | SNOPT/IPOPT，manipulation 強 | 機械臂接觸優化 |
| **MuJoCo MPC (mjpc)** | Predictive Sampling + iLQG，DeepMind 開源 | sampling-based 研究 |

### 機械臂 vs 四足 NMPC 差異（面試必考）

- **機械臂 = 全驅動固定基座**：CTC (Computed Torque Control) 用 $O(n)$ 逆動力學 + PID 1 kHz 就能完美追蹤 → **NMPC 算力浪費**（除非要處理避碰或接觸）
- **四足 = 浮動基座欠驅動**：必須前瞻規劃 GRF 才能防摔 → **必須 MPC**

**面試金句**：「如果你能直接算逆動力學餵 PID，那不需要 MPC。MPC 是給欠驅動 + 受約束系統的武器。」

</details>

<details>
<summary>深入：Feasibility 危機處理與 CBF-MPC — 工業級 70% 精力在這</summary>

### 什麼時候會 Infeasible

1. **初始狀態違反約束**：機器人被踹一腳 → 當前狀態已在可行域外
2. **約束互相矛盾**：時間硬約束 + 扭矩硬約束 + 路徑被封死 → 解集為空
3. **求解器數值問題**：NaN / 無法收斂 → 返回 infeasible flag

優化器一旦回報 infeasible，若沒準備 fallback → **馬達就被鎖在上週期的指令暴衝**。

### 產線救援三層防護

**Layer 1：Slack Variable 軟化硬約束**

引入鬆弛變數 $\varepsilon \ge 0$，把硬約束 $h(x) \le 0$ 改為 $h(x) \le \varepsilon$，代價函數加上重罰 $\lambda \varepsilon^2$（$\lambda$ 極大）：

$$
\min_{U, \varepsilon} J(x, u) + \lambda \varepsilon^2, \quad \text{s.t. } h(x) \le \varepsilon, \; \varepsilon \ge 0
$$

**物理意義**：平時 $\varepsilon = 0$ 等同硬約束；無解時允許微小違反換取可行解，不讓系統崩潰。$\lambda$ 越大，系統越接近硬約束行為。

**acados 軟約束 pseudo-code**：

```c
ocp_nlp_cost_model->zl = 1e6;   // L1 lower slack penalty
ocp_nlp_cost_model->Zl = 1e4;   // L2 quadratic slack penalty
ocp_nlp_constraints_model->idxsh = {...};  // 哪些約束可軟化
```

**Layer 2：降級用上一步可行解**

拿上週期預測序列 $u^*_{1:N-1}$ 的第二步 $u^*_1$ 頂替當前輸出。即使舊解不再最優，至少物理上可行。

**Layer 3：Fallback Controller（兜底）**

當優化器返回 NaN / 異常 → 切換到純阻尼控制（$u = -K_d \dot{x}$）「軟趴下」，保證安全停機。

### CBF (Control Barrier Function) — 前向不變集的數學保證

**傳統 Terminal Constraint 的致命缺陷**：只保證最後一步 $x_N$ 在安全集裡，**中間時域可能撞到**。

**CBF 的定義**：找一個光滑函數 $h(x)$，使 $\{x : h(x) \ge 0\}$ 是安全集；若滿足

$$
\dot{h}(x) \ge -\alpha \cdot h(x), \quad \alpha > 0
$$

則安全集是**前向不變集 (Forward Invariant Set)** — 一旦進入就絕不出去。

**物理意義**：$h$ 是「距離危險的餘裕」；$\dot{h} \ge -\alpha h$ 等於說「越靠近邊界 ($h$ 小) 就越強制遠離 ($\dot{h}$ 必為正）」。求解器戴上「絕不越界」的緊箍咒。

**CBF 嵌入 MPC**：把 $\dot{h} + \alpha h \ge 0$ 作為每個預測時間步的約束 → Safety-Critical MPC。

**協作機械臂應用**：人突然闖入工作區 → 即使 MPC 想抄捷徑（cost 最低但要穿越人的位置），CBF 在邊界強制排斥速度 → 自動煞車。

### 工業級 MPC 的真相

> **70% 工程精力設計 Slack + Fallback + CBF 狀態機**，只有 30% 在調 Q/R 權重。保證極端干擾（通訊延遲、感測器失真、突然碰撞）下仍輸出平滑安全指令，這才是產品級 MPC。

</details>

<details>
<summary>深入：Learning-based MPC — GP / Deep / Meta / Amortized 四大流派</summary>

### 為什麼模型永遠不準

1. **執行器電流環延遲**：馬達驅動器不是理想力源
2. **諧波減速器非線性柔性變形**：高精密但非剛體
3. **軟地面非剛性接觸**：草地、沙地摩擦不穩
4. **齒輪背隙 + 溫漂摩擦**：運行數小時後參數漂移

→ 模型不準 → MPC 預測與實際脫節 → 性能退化甚至違反安全約束。

### 四大流派對照

**1. GP-MPC (Gaussian Process MPC)**

- GP 擬合模型殘差 $\Delta f = f_{\text{true}} - f_{\text{nominal}}$
- 輸出 $\mu$ (均值) + $\sigma^2$ (變異數) → **天然給不確定性量化**
- **Chance Constraint 魔法**：
  - 傳統硬約束：$h(x) \le 0$
  - GP 版：$P(h(x) \le 0) \ge 99\% \Leftrightarrow h(\mu) + 3\sigma \le 0$
  - **物理意義**：GP 越沒把握（$\sigma$ 大）→ 邊界越緊 → 機器人越保守越慢 → **絕對安全**

**2. DeepMPC (Neural Network 殘差學習)**

- NN 擬合殘差，推論極快（< 1 ms）
- **代價**：黑箱、OOD 狀態下易發散、無不確定性量化
- 適合線上部署但需搭配 safety filter

**3. Meta-MPC (MAML / Meta-learning)**

- 在多任務分佈上預訓練
- 部署時**在線 2–3 週期觀測就能梯度更新** → 適應負載突變
- ANYmal 抓未知重量物體、機械臂換不同夾具的殺手鐧

**4. Amortized MPC (Imitation from MPC)**

- 雲端離線跑完美 NMPC → 收集 `(State, Optimal_u)` → 訓 NN 克隆
- 部署：**$O(n^3)$ → $O(1)$ 微秒推論**
- **代價**：失去物理硬約束保證 → 必須**墊 QP Safety Filter**（基於 CBF）做最後安全裁切

### 真實平台對應

- **ANYmal (ETH)**：OCS2 + latent space 地形摩擦估計 → MPC + WBC
- **MIT Cheetah**：Convex MPC (SRBD) + WBC，嵌入式 100 Hz+ 奔跑
- **Tesla Optimus**：End-to-End RL (高層決策) + 傳統 MPC/WBC (底層平衡)
- **Boston Dynamics Atlas**：離線軌跡庫 + 線上 MPC 在軌跡管內跟蹤

### 面試 Debug 思維（必考）

**Q：「模擬器 MPC 完美但真機劇烈抖動，怎麼 debug？」**

**三步走**：
1. **通訊 / 執行延遲**：模擬假設零延遲無限頻寬；預測模型必須顯式補償（用上週期 $u$ 作當前狀態歷史輸入）
2. **頻率掃頻辨識真實馬達頻寬**：MPC 輸出超馬達 Nyquist 頻率的高頻跳變 → 硬體只能抖動
3. **代價函數加 $\Delta u$ 懲罰**：調高控制量變化率權重 $R_\Delta$，強制平滑且在馬達頻寬內的指令

</details>

<details>
<summary>深入：Robust MPC 三流派 — Tube / Min-Max / Stochastic 選型</summary>

### 為什麼 Nominal MPC 在實機會失敗

- 未建模摩擦 + 外部擾動（風、接觸）+ 感測雜訊
- 預測軌跡與實際軌跡逐步偏離 → 超調或違反安全約束

### 三大流派

**1. Tube MPC（管狀 MPC）— L4 自動駕駛標準解**

- **核心**：Nominal MPC 規劃理想名義軌跡 $\bar{x}$，輔助 LQR 把真實 $x$ 「束縛」在 Tube $Z$ 內
- **Control Invariant Set**：若 $x_0 - \bar{x}_0 \in Z$，輔助控制律下未來誤差永不逃出 $Z$
- **物理意義**：「最壞擾動下，真實軌跡只在名義軌跡周圍微小管道內震盪」
- **工程實作**：**把原硬約束向內收縮一個 tube 半徑**，Nominal MPC 在收縮後邊界內規劃 → 真實系統絕對安全

$$
\bar{x} \in X \ominus Z, \quad \bar{u} \in U \ominus K Z
$$

**物理意義**：$\ominus$ 是 Minkowski 差（集合收縮），$K$ 是輔助控制律增益。

**2. Min-Max MPC（極小極大）**

$$
\min_u \max_{w \in W} J(x, u, w)
$$

**物理意義**：對抗性優化，對**最壞擾動** $w$ 做規劃。計算極度昂貴（時域內擾動分支指數爆炸），工業幾乎不用。實務常用 **Scenario-based approximation** 隨機抽幾條極端擾動軌跡近似。

**3. Stochastic MPC / Chance-Constrained — 四足跑酷標配**

- 擾動建模為機率分佈
- 約束 $P(h(x) \le 0) \ge 1 - \varepsilon$（如 99.7%）
- **與 GP-MPC 結合**：GP 預測殘差自帶 $\sigma^2$；3σ 涵蓋 99.7%；約束向內收縮 3σ → 轉化為確定性 NLP
- 平台：四足機器人跑酷 / 碎石路（接受 5% 滑移概率換取敏捷）

### 三流派 Trade-off 量化（面試必答）

| 類別 | 計算成本 | 保守度 | 安全保證 | 典型應用 |
|------|---------|--------|----------|----------|
| **Min-Max** | 最慢 | 最保守 | 絕對最壞情況 | 幾乎不用 |
| **Tube** | 極快（只解 Nominal） | 中等 | 絕對安全 (Hard Safety) | **L4 自動駕駛** |
| **Stochastic** | 中等 | 適中 | 機率安全（如 99.7%） | **四足碎石 / 跑酷** |

### CasADi Chance Constraint 3σ 範例

```python
mu_x, sigma_x = gp_predict(x_k, u_k)
distance = ca.norm_2(mu_x[0:2] - obs_pos)
tightened_safe_dist = d_safe + 3.0 * ca.max(sigma_x)
g.append(distance - tightened_safe_dist)
lbg.append(0.0)
```

</details>

<details>
<summary>深入：Contact-Implicit MPC (CI-MPC) — 2024 Manipulation Game-Changer</summary>

### 傳統 MPC 的接觸限制

- 必須**預定 Mode Sequence**（0.1s 左腳落、0.2s 右腳落；或「先碰箱左邊再推右邊」）
- 環境變化導致提早接觸 → MPC 崩潰
- 撬門栓、翻頁、推箱子等需要**主動接觸**的 manipulation 任務必須寫複雜 FSM 切位置控 / 阻抗控

### CI-MPC 的本質突破

**打破模式序列限制**：接觸力 $\lambda$ 與接觸距離 $\phi(q)$ 同時作為**優化變數**（不是預設常數）。

**互補約束 (Complementarity Constraint)**：

$$
\phi(q) \ge 0, \quad \lambda \ge 0, \quad \phi(q) \cdot \lambda = 0
$$

**物理意義**：
- $\phi(q) \ge 0$：不穿透（距離非負）
- $\lambda \ge 0$：只能推不能拉
- $\phi(q) \cdot \lambda = 0$：**要麼沒碰（$\phi > 0, \lambda = 0$），要麼碰到（$\phi = 0, \lambda > 0$）**

### 為什麼能「自動發現」接觸策略

- NLP Solver 追最小 Cost 時有權調 $\phi$ 和 $\lambda$
- 發現「讓 $\phi = 0$ 主動去碰 → 獲得 $\lambda > 0$ 的力 → 利用力推箱子到目標點 → Cost 下降」
- 抓取、推動、跑酷步伐都是 **Emergent Behavior 自發湧現**

### 計算挑戰 + MPCC 鬆弛

- $\phi \cdot \lambda = 0$ 極度非光滑非凸，梯度 undefined，求解器卡死
- **MPCC (Mathematical Program with Complementarity Constraints) 鬆弛**：
  - 改為 $\phi(q) \cdot \lambda \le \varepsilon$
  - $\varepsilon$ 隨迭代從大慢慢 → 0（類似 Interior Point Barrier 思維）

### Drake MPCC 鬆弛片段

```python
phi = prog.NewContinuousVariables(1, "phi")
lam = prog.NewContinuousVariables(1, "lambda")
prog.AddConstraint(phi >= 0)
prog.AddConstraint(lam >= 0)
epsilon = 1e-3  # 外層迴圈讓 ε → 0
prog.AddConstraint(phi * lam <= epsilon)
prog.AddCost(cost_function(phi, lam, state, target))
```

### 最新前沿：Neural Dual MPC / Learned Contact Sequences

- CI-MPC 線上求解仍慢
- **離線跑萬次 CI-MPC** 生成 through-contact 軌跡 → 訓 NN 預測最優接觸模式
- 線上 MPC 套預測模式 → 非光滑 → 平滑問題

### 面試關鍵 talking point

> **「CI-MPC 統一了 Free-space Motion 與 In-contact Manipulation 的數學表達」**
> 過去撬門栓需寫複雜 FSM 切位置控與阻抗控；CI-MPC 眼中這就是一段連續數學軌跡 → 優化器自己決定何時輕碰、何時發力推。**把依賴專家經驗的 Heuristics 變成嚴謹數值優化問題**。

**平台**：MIT Cheetah CI-TrajOpt + online MPC、Drake manipulation。

</details>

<details>
<summary>深入：Adaptive / Lie Group / Offset-free / Event-triggered MPC — 產品級必備四件套</summary>

### A. Adaptive MPC — 對付未知參數（抓 10 kg 鉛球）

**為什麼 MPC 比 PID 更需要 online adaptation**：
- PID Error-driven，模型錯了積分仍可硬拉回
- **MPC Model-driven**，模型錯 → 預測未來 10 步全偏離 → 開環指令災難

**Set-Membership MPC**：未知參數 $m \in [1, 5]$ kg → 多面體約束；優化器必須對**集合內每個可能值**都不違反安全約束。

**Adaptive Tube MPC**：結合 RLS 線上參數估計，**Tube 半徑動態收縮**。
- 物理行為：機器人剛開始謹慎（管徑大），「摸熟」物體重量後管徑收縮 → 動作變激進高效

**Neural Adaptive MPC (Meta-learning)**：Meta-learning 上層網路根據 Context（風速 / 摩擦）**直接輸出 MPC 內部 Cost Weights**。
- 冰面時自動調高「軌跡偏差懲罰」、調低「速度獎勵」

**RLS 參數更新律**：

$$
\hat{\theta}_k = \hat{\theta}_{k-1} + K_k (y_k - \varphi_k^T \hat{\theta}_{k-1})
$$

**物理意義**：新估計 = 舊估計 + 預測誤差驅動的修正；$K_k$ 增益隨資料累積收斂。

**Adaptive vs Robust 陷阱**：Robust MPC 假設最壞情況 → 邊界極大 → 機器人過度保守；**Adaptive MPC 小邊界 + SysID 線上學真實參數 → 安全 + 最優高動態性能兼得**。

### B. Lie Group MPC — 旋轉的正確處理

**四元數 / 旋轉矩陣直接作 State 的災難**：
- **歐拉角奇異性 (Gimbal Lock)**：俯仰角 ±90° 時自由度丟失 → Jacobian 秩虧
- **四元數 Over-parameterization**：旋轉只 3 DoF，四元數 4 變數 → 強制加非凸歸一化硬約束 $\|q\|^2 = 1$ → 求解器 Zig-zagging

**Lie Group MPC 流形優化哲學**：
- State 存在於李群 $M$（SO(3) 或 SE(3)）
- **優化的增量 $\Delta$ 與誤差存在切空間的李代數 $\mathfrak{m}$（$\mathfrak{so}(3) \cong \mathbb{R}^3$）**
- 指數映射拉回流形：$R_{k+1} = R_k \cdot \exp(\Delta\omega^\wedge)$

**誤差的純淨數學（對數映射）**：

$$
e_R = \log(R_{\text{target}}^T \cdot R_{\text{current}})^\vee \in \mathbb{R}^3
$$

**物理意義**：完美代表「要繞哪個軸轉多少角度對齊」；SQP 收斂極快，無奇異性、無冗餘約束。

**manif 庫 C++ 片段**：

```cpp
SE3Tangentd error_tangent = (X_target.inverse() * X_current).log();
return error_tangent.coeffs();  // 乾淨的 6D 向量 [平移 3D, 旋轉 3D]
SE3d X_next = X_current + SE3Tangentd(delta_u * dt);  // 內部 exp map
```

**平台**：MIT Cheetah / ANYmal body orientation 定義在 SO(3)、PX4 無人機飛控採用 SO(3) 誤差流形。

### C. Offset-free MPC — 對付恆定擾動（= MPC 的 I 項）

**為什麼 Nominal MPC 有穩態誤差**：純數學模型 $x_{k+1} = Ax_k + Bu_k$ 沒有外力；現實被風吹走 → MPC 永遠差一點（**本質上是 P-controller**）。

**Disturbance Model Augmentation**：

$$
x_{k+1} = A x_k + B u_k + B_d d_k, \quad d_{k+1} = d_k, \quad y_k = C x_k + d_k
$$

**物理意義**：把「未建模的恆定擾動」當新狀態 $d$，用 Kalman / Luenberger Observer 從殘差估計 $d_k$，MPC 在預測中前饋抵消 → **Offset-free**。

**Velocity Form MPC（增量式）**：不優化絕對 $u_k$，優化 $\Delta u_k$；內部自然形成積分器 $u_k = \sum \Delta u$。

**面試金句**：「沒 Offset-free 機制的 MPC，就像沒 I 項的 PD 控制器 — 穩態永遠帶靜差。**這是學術玩具 vs 產線產品的分水嶺**。」

### D. Event-triggered / Self-triggered MPC — 邊緣 AI 必備

**為什麼每週期重解是浪費**：機器人穩定巡航 + 狀態變化在上週期 trajectory tube 內 → 上一解仍**近乎最優**。

**Event-triggered MPC**：只有當觸發條件滿足才喚醒優化器：

$$
\|x(t) - \hat{x}(t|t_k)\|_Q \ge \sigma \cdot \|x(t_k)\| \Rightarrow \text{re-solve MPC}
$$

- 穩態下觸發頻率從 100 Hz 降到 10 Hz → **省 50–80% 計算**
- 其他時刻：直接 Shift 沿用上週期序列

**Self-triggered MPC**（更激進）：解當前 MPC 時**主動預測下次必須重解時刻** $t_{\text{next}}$；期間處理器進入 Sleep。

**理論保證：ISS + Lyapunov**：觸發條件設計保證價值函數 $V(x)$ 衰減率 $\dot{V} \le -\alpha \|x\|^2$；舊序列讓 $V$ 停止下降時強制觸發。

**SWaP (Size, Weight, Power) 邊緣計算必要性**：
- 未來具身智能微型無人機 / 太陽能農業機器人無法搭桌面級 GPU
- **「週期性強計算」→「按需計算 Compute-on-demand」**
- Lyapunov 當「觸發守門員」保證不失控前提下砍 80% NMPC 計算
- **大模型 + 複雜控制在嵌入式 MCU (STM32) 共存的唯一出路**

**C++ Event-triggered 邏輯**：

```cpp
bool should_trigger_mpc(x_curr, x_pred, sigma) {
    double error = (x_curr - x_pred).T * Q * (x_curr - x_pred);
    double threshold = sigma * x_curr.squaredNorm();
    return error > threshold;
}
if (should_trigger_mpc(...)) { solve_mpc_nlp(); u_cmd = get_first(); }
else { u_cmd = get_shifted(); }  // O(1) 查表，極低功耗
```

</details>

## 直覺理解

### 類比：西洋棋 vs 反射神經

- **PID** = 見招拆招的反射神經 — 看到對方出棋才反應
- **LQR** = 會算最佳走法的西洋棋選手，但**只在無時間壓力的理想環境**
- **MPC** = 一邊下棋一邊算未來 N 步、還要考慮「不能動某些被封鎖的格子」的選手 — **有時間壓力、有硬規則、要不斷重算**
- **RL Policy** = 棋譜背熟的直覺派 — 又快又泛化，但遇到沒背過的棋局會亂下

**四足機器人的分工**：RL/VLA 在上層決定「要跳過這個溝」；MPC 在中層算出「質心軌跡怎麼走、四個腳什麼時候在哪裡落」；WBC 在底層把 MPC 的 GRF 變成每顆馬達的扭矩。

### 模擬器可觀察現象

- **MuJoCo / Isaac Sim 跑 MIT Cheetah Convex MPC**：
  - 質心軌跡平滑、GRF 方向箭頭（debug view）永遠在摩擦錐內
  - 被踹一腳：第一週期軌跡偏離 → 但 tube / slack 機制讓它 0.3 秒內拉回
  - 突然關掉 CBF → 馬上看到機器人撞障礙
- **Drake 機械臂 CI-MPC 推箱子**：
  - 最初「空揮」幾次 → 優化器發現接觸可以降 cost → 自動「發現」要去推
  - 這是 emergent behavior，不是寫死 FSM
- **MPPI 並行撒 5000 條軌跡**（MuJoCo MPC / mjpc）：
  - Debug view 能看到五顏六色的候選軌跡 + Softmax 權重可視化
  - 面前有一棵樹時：左右兩片軌跡束，最後 softmax 坍縮到代價較低那側

### 「大腦 / 小腦 / 脊髓」三層骨架

| 層級 | 頻率 | 角色 | 代表 |
|------|------|------|------|
| **大腦**（慢思考） | 1–10 Hz | 語義理解 + 長程規劃 | VLA / LLM / RL policy |
| **小腦**（快預測） | 10–100 Hz | 動力學預測 + 軌跡重規劃 | MPC / NMPC |
| **脊髓**（硬反射） | 1 kHz+ | 扭矩分配 + 物理硬約束 | WBC / PID / CBF QP Filter |

**面試金句**：「未來具身智能 = 大模型 + 多層 MPC + WBC 的混合體。端到端 RL 缺硬約束保證通不過工業安全認證；純 MPC 缺語義泛化處理不了 open-world 指令。」

## 實作連結

### ROS 2 MPC 控制節點骨架

真實工程情境：四足機器人在 ROS 2 上跑 SRBD MPC，50 Hz 迴圈，輸出 GRF 給 WBC。

```cpp
// mpc_controller_node.cpp — SRBD MPC + acados RTI
class MpcControllerNode : public rclcpp::Node {
public:
    MpcControllerNode() {
        // 訂閱狀態估計 + 高層軌跡
        state_sub_ = create_subscription<StateMsg>("/state_estimate", 10, ...);
        ref_sub_   = create_subscription<TrajMsg>("/ref_trajectory", 10, ...);
        // 發佈 GRF + 足端排程給 WBC
        grf_pub_   = create_publisher<GrfMsg>("/desired_grf", 10);

        // 50 Hz 控制迴路
        timer_ = create_wall_timer(20ms, std::bind(&MpcControllerNode::step, this));

        // 初始化 acados NLP solver（RTI + HPIPM）
        ocp_solver_ = acados_create_solver(...);
    }

    void step() {
        // 1) 拿當前狀態（需做通訊延遲補償）
        auto x_curr = get_latest_state_with_delay_compensation();

        // 2) 事件觸發檢查（節能）
        if (!should_trigger_mpc(x_curr, x_pred_last_, sigma_)) {
            u_cmd_ = shift_last_solution();  // O(1) 沿用上週期序列
        } else {
            // 3) 設定 reference + warm start
            acados_set_state(ocp_solver_, 0, x_curr);
            acados_set_reference(ocp_solver_, ref_trajectory_);
            warm_start_from_last(ocp_solver_);

            // 4) RTI 單次 SQP 迭代
            int status = acados_solve_rti(ocp_solver_);

            // 5) Feasibility 救援三層
            if (status == ACADOS_SUCCESS) {
                u_cmd_ = acados_get_u(ocp_solver_, 0);
            } else if (last_solution_valid_) {
                u_cmd_ = shift_last_solution();              // Layer 2
            } else {
                u_cmd_ = damping_fallback(x_curr);           // Layer 3
            }
        }

        // 6) CBF Safety Filter — 最後一道安全鎖
        u_cmd_ = cbf_safety_filter(u_cmd_, x_curr);

        // 7) 發佈給 WBC
        publish_grf(u_cmd_);
    }
private:
    acados_solver* ocp_solver_;
    // ...
};
```

**關鍵工程細節**：
- **時間戳同步**：`x_curr` 必須加上從感測到計算的延遲補償，否則預測與實際脫節
- **Warm Start**：上週期解 shift 一格當初始猜測，RTI 單迭代就夠準
- **三層救援**：solver 失敗不可讓馬達空指令，必須 fallback
- **CBF Filter**：即使 MPC 解出來，最後過一道微秒級 QP 把指令拉回安全集

<details>
<summary>深入：完整 Python 實作 — CasADi Multiple Shooting NMPC + Chance Constraint</summary>

```python
import casadi as ca
import numpy as np

N = 20          # prediction horizon
dt = 0.05       # 50 Hz
nx, nu = 12, 12 # SRBD: 12 state, 12 GRF

# ---- 1. 決策變數（Multiple Shooting）----
opti = ca.Opti()
X = opti.variable(nx, N+1)
U = opti.variable(nu, N)
EPS = opti.variable(N)   # slack variable

# ---- 2. 初始條件 + 參考 ----
x0 = opti.parameter(nx)
X_ref = opti.parameter(nx, N+1)
opti.subject_to(X[:, 0] == x0)

# ---- 3. 動力學約束（RK4 + SRBD）----
def srbd_rk4(x, u, dt):
    k1 = srbd_dynamics(x, u)
    k2 = srbd_dynamics(x + dt/2*k1, u)
    k3 = srbd_dynamics(x + dt/2*k2, u)
    k4 = srbd_dynamics(x + dt*k3, u)
    return x + dt/6*(k1 + 2*k2 + 2*k3 + k4)

for k in range(N):
    opti.subject_to(X[:, k+1] == srbd_rk4(X[:, k], U[:, k], dt))

# ---- 4. 軟約束：摩擦錐 + 關節極限 + Slack ----
for k in range(N):
    fx, fy, fz = U[0, k], U[1, k], U[2, k]
    # 摩擦錐（向內收縮 3σ for Stochastic）
    mu = 0.6
    sigma = predict_friction_std(k)
    tightened_mu = mu - 3.0 * sigma
    opti.subject_to(fx**2 + fy**2 <= (tightened_mu * fz + EPS[k])**2)
    opti.subject_to(fz >= 0)
    opti.subject_to(EPS[k] >= 0)

# ---- 5. CBF 約束（人侵入工作區時強制避讓）----
for k in range(N):
    h_k = cbf_distance_to_obstacle(X[:, k])
    h_dot = cbf_derivative(X[:, k], U[:, k])
    alpha = 5.0
    opti.subject_to(h_dot + alpha * h_k >= 0)

# ---- 6. 代價函數 ----
Q = np.diag([...])
R = np.diag([...])
R_du = np.diag([...])
lam_slack = 1e6

cost = 0
for k in range(N):
    e = X[:, k] - X_ref[:, k]
    cost += ca.mtimes([e.T, Q, e]) + ca.mtimes([U[:, k].T, R, U[:, k]])
    if k > 0:
        du = U[:, k] - U[:, k-1]
        cost += ca.mtimes([du.T, R_du, du])
    cost += lam_slack * EPS[k]**2   # 軟約束重罰

# terminal
e_N = X[:, N] - X_ref[:, N]
cost += ca.mtimes([e_N.T, Q, e_N])

opti.minimize(cost)

# ---- 7. 求解（IPOPT + Warm Start）----
opts = {"ipopt.print_level": 0, "print_time": 0,
        "ipopt.warm_start_init_point": "yes"}
opti.solver("ipopt", opts)

# ---- 8. 滾動時域迴圈 ----
x_curr = np.array([...])
for step in range(1000):
    opti.set_value(x0, x_curr)
    opti.set_value(X_ref, generate_ref(step))
    opti.set_initial(X, X_warm)
    opti.set_initial(U, U_warm)

    try:
        sol = opti.solve()
        u_apply = sol.value(U[:, 0])
        X_warm = np.column_stack([sol.value(X)[:, 1:], sol.value(X)[:, -1:]])
        U_warm = np.column_stack([sol.value(U)[:, 1:], sol.value(U)[:, -1:]])
    except RuntimeError:
        u_apply = damping_fallback(x_curr)    # Layer 3

    x_curr = simulate_one_step(x_curr, u_apply, dt)
```

**要點**：
1. **Multiple Shooting** — $X$ 與 $U$ 都是決策變數，`X[:, k+1] == RK4(X[:, k], U[:, k])` 作連續性約束
2. **Slack Variable** — 每個摩擦錐配一個 $\varepsilon_k$，成本加 $10^6 \varepsilon_k^2$
3. **CBF 約束** — $\dot{h} + \alpha h \ge 0$ 強制不越過障礙安全邊界
4. **Chance Constraint** — 摩擦係數向內收縮 $3\sigma$，轉化為確定性約束
5. **Warm Start** — 每週期把上週期解 shift 一格當初始猜測
6. **Fallback** — 求解器 raise 例外 → 切阻尼控制

### C++ 生產級對照（acados + HPIPM）

```cpp
// 初始化 acados RTI solver
acados_ocp_solver* solver = model_acados_create();

// 每週期
for (;;) {
    // 時戳對齊的狀態
    auto x0 = get_state_with_delay_comp();
    model_acados_set(solver, 0, "lbx", x0.data());
    model_acados_set(solver, 0, "ubx", x0.data());
    model_acados_set(solver, 0, "p", params.data());  // 動態參數

    // 單次 RTI
    int status = model_acados_solve(solver);

    if (status == 0) {
        model_acados_get(solver, 0, "u", u_opt.data());
        apply_cbf_safety_filter(u_opt, x0);
        publish(u_opt);
    } else {
        publish(damping_fallback(x0));
    }
    sleep_until_next_tick(20ms);
}
```

**HPIPM 做 sparse KKT**，整個 RTI 迴圈在 ARM Cortex-A72 上可穩定 `< 5 ms`。

</details>

## 常見誤解

### 誤解 1：「MPC 就是 LQR 加個 for 迴圈」

**真相**：LQR 是**無約束**的二次型最優控制，有閉解；MPC 的真正威力來自**處理硬約束**（力矩限、摩擦錐、CBF、避障）。沒有約束的話直接用 LQR 就好 — 大部分工程師誤以為 MPC 只是「在線算 LQR」，會錯失 MPC 的核心賣點。**LQR = 簡單系統的 MPC 在無約束時的退化；MPC = 帶約束的 LQR 在線版**。

### 誤解 2：「機械臂也應該用 MPC 取代 PID」

**真相**：**機械臂是全驅動固定基座**。只要 $O(n)$ 逆動力學 (CTC, Computed Torque Control) + PID 1 kHz 就能完美追蹤，NMPC 算力純浪費。**MPC 真正的戰場是欠驅動 / 浮基 / 接觸切換系統**：四足、人形、無人機、有避障需求的 manipulation。面試被問「為什麼 ABB/KUKA 機械臂不用 MPC」要能立刻答出「全驅動 + 固定基座 + 不需前瞻 GRF」。

### 誤解 3：「模擬器跑通就 OK，上真機只是調參數」

**真相**：模擬器假設**零延遲、零雜訊、完美模型**。真機會遇到：
- 通訊延遲（ROS DDS、CAN bus）
- 執行器電流環延遲
- 模型失配（諧波減速器、齒輪背隙、軟地面）
- 感測器雜訊

→ 不處理這些的 MPC 上真機會劇烈抖動或違反約束。**工業界 70% 精力花在 Slack / Fallback / Safety Filter / Offset-free / Delay Compensation**，只有 30% 在調 Q/R。

### 誤解 4：「把四元數歸一化約束 $\|q\|^2 = 1$ 塞進 NLP 就能做旋轉優化」

**真相**：$\|q\|^2 = 1$ 是球面流形；NLP 線性化時約束法向量與狀態更新方向強烈耦合；切線方向的更新瞬間偏離球面 → 求解器為了把解拉回球面產生極小迭代步長（**Zig-zagging**）→ 計算耗時暴增數十倍。**正解**：用 **Lie Group MPC** — 狀態存在 SO(3)，優化變數存在切空間 $\mathfrak{so}(3) \cong \mathbb{R}^3$；誤差用對數映射 $\log(R_{\text{target}}^T R_{\text{current}})^\vee$。PX4 飛控、MIT Cheetah、ANYmal 全都這樣做。

### 誤解 5：「Neural MPC / Amortized MPC 訓好就能直接取代傳統 MPC」

**真相**：NN 克隆 MPC 後**失去物理硬約束保證**，OOD 時可能輸出致命越界指令。**必墊 CBF QP Safety Filter**：微秒級 QP 把 NN 指令「拉回」安全集。Tesla Optimus、很多前沿具身智能方案都是「NN 輸出 + CBF 兜底」這種結構。面試能講出這個細節 = 分辨「聽過」vs「實際部署過」。

### 誤解 6：「Terminal Constraint 就能保證 MPC 穩定又安全」

**真相**：Terminal Constraint 只保證**最後一步** $x_N$ 落在穩定區；**中間時域 $x_1, \ldots, x_{N-1}$ 可能穿越障礙**。Safety-Critical 場合必用 **CBF (Control Barrier Function)** — 對每一步強制 $\dot{h}(x) + \alpha h(x) \ge 0$，保證前向不變集 → **一旦進入安全集絕不出去**。Terminal Constraint + CBF 結合才是協作機器人 / 自駕的標配。

## 練習題

### 情境題 1：四足機器人 MPC 在轉彎時劇烈振盪，怎麼系統性除錯？

**分析鏈**：

<details>
<summary>完整推理</summary>

**三個病因 + 修法，依可能性排序**：

1. **線性化誤差**（機率最高）
   - Yaw 角變化大，LMPC 的 $\sin(\theta) \approx \theta$ 假設失效
   - **修法 A**：提高線性化更新頻率（每步重線性化）
   - **修法 B**：切 NMPC（acados RTI + SRBD 完整非線性模型）
   - 驗證方式：MuJoCo 畫出預測軌跡 vs 實際軌跡殘差，殘差隨 yaw rate 增大就是這個問題

2. **預測時域 N 太短**（常被忽略）
   - $N \cdot \Delta t$ 短於轉彎動作時長 → 看不到彎道盡頭 → 方向盤瞬間左右打
   - **修法**：動態拉長 $N$（入彎前觸發），或加入前瞻參考軌跡引導 (lookahead reference)

3. **約束衝突**
   - 摩擦錐 + 關節速度限幅太緊 → Feasible Set 幾乎空
   - 求解器被迫逼近邊界 → 每週期解都在邊界附近跳動
   - **修法**：引入 Slack Variable 軟化摩擦錐與關節速度約束（代價加 $\lambda \varepsilon^2$）；同時記錄 $\varepsilon$ 值監控是否頻繁違反

**綜合建議**：先用 MuJoCo + RViz debug view 看「預測軌跡 vs 實際軌跡」差異位置，快速鎖定是 (1) 模型失配、(2) N 太短、還是 (3) 約束衝突。實戰中 (1)+(3) 最常同時出現。

</details>

### 情境題 2：協作機械臂在人突然闖入時要絕對煞車，但 MPC 已經解了最優軌跡準備發指令，怎麼辦？

**分析鏈**：

<details>
<summary>完整推理</summary>

**單靠 MPC 的代價函數 + terminal constraint 不夠**：
- Cost 雖然懲罰接近人，但若距離目標點代價很大，優化器會「計算」取巧從人旁邊切過
- Terminal constraint 只管最後一步 → 中間時域仍可能撞人

**正解：CBF (Control Barrier Function) 雙層保護**：

1. **MPC 層**：每個預測時間步加入 CBF 約束 $\dot{h}(x_k) + \alpha h(x_k) \ge 0$，其中 $h(x)$ 是到人的距離 - 安全距離
   - 物理意義：越靠近人 ($h$ 小) 就越強制遠離 ($\dot{h}$ 必為正）

2. **執行層 Safety Filter**：即使 MPC 解出來，最後過一道微秒級 QP

$$
u_{\text{safe}} = \arg\min_u \|u - u_{\text{mpc}}\|^2 \quad \text{s.t. CBF constraint}
$$

**物理意義**：盡可能接近 MPC 想要的指令，但被 CBF 約束「拉回」安全集。

**加分點（面試提升）**：
- CBF $\alpha$ 的選擇：太大機器人過保守不動；太小危急時反應慢
- 整體架構：感知層（人 pose 估計 + tracking）→ CBF 定義距離場 → MPC + Safety Filter
- **相關標準**：ISO/TS 15066（協作機器人功能安全）規定功能性限速，CBF 是達成這標準的數學工具

</details>

### 情境題 3：無人機在強風條件下懸停，MPC 永遠在目標點下風側 30 cm 穩態，怎麼修？

**分析鏈**：

<details>
<summary>完整推理</summary>

**病因**：這是**穩態誤差 (steady-state offset)**，Nominal MPC 的本質問題。
- 預測模型 $x_{k+1} = Ax_k + Bu_k$ 沒有風力項
- 現實有恆定風擾動 $d_{\text{wind}}$
- MPC 本質是個高級 P-controller，有偏差但不累積 → 永遠差一點

**正解：Offset-free MPC（= MPC 的 I 項）**

1. **Disturbance Augmentation**：狀態擴增擾動變數

$$
x_{k+1} = A x_k + B u_k + B_d d_k, \quad d_{k+1} = d_k
$$

2. **Luenberger / Kalman Observer**：從「模型預測輸出 vs 感測實測」殘差估計 $d_k$
3. **MPC 前饋抵消**：在代價函數與目標重計算中帶入 $d_k$

**替代方案 Velocity Form MPC**：優化 $\Delta u_k$ 而非 $u_k$，內部自然形成積分器，誤差不為零 → $\Delta u$ 不為零 → 累積直到消除。

**驗證**：風力突變時觀察無人機是否能在幾秒內消除 offset；觀測器估計的 $\hat{d}$ 應該逼近真實風力方向。

**面試加分**：提到「Offset-free 是工業鐵律」— 化工廠 Process Control、機器人抗風、產線機械臂抓取重物不確定質量，都靠這個機制。**沒 Offset-free = 沒 I 項 PD = 學術玩具**。

</details>

### 情境題 4：機械臂要推箱子移動到桌面指定位置，但箱子質量未知、桌面摩擦未知，用什麼 MPC 變種？

**分析鏈**：

<details>
<summary>完整推理</summary>

這題涉及三個核心挑戰：
- **（A）主動接觸**：MPC 要自己決定何時推、從哪推
- **（B）未知參數**：質量、摩擦
- **（C）安全邊界**：不能把箱子推出桌面

**綜合方案**：

**1. CI-MPC (Contact-Implicit MPC) 處理 A**
- 接觸力 $\lambda$ 和接觸距離 $\phi$ 同為優化變數
- 互補約束 $\phi \cdot \lambda = 0$（MPCC 鬆弛為 $\phi \lambda \le \varepsilon$）
- 優化器自己發現「空手 → 碰箱 → 推」的 emergent behavior
- 框架：Drake MPCC

**2. Adaptive MPC / Set-Membership 處理 B**
- 質量 $m \in [1, 10]$ kg、摩擦 $\mu \in [0.3, 0.8]$ → 多面體集合
- RLS 在線估計，tube 半徑隨 $\Sigma$ 收縮（Adaptive Tube）
- 剛開始謹慎（慢、推小力），摸熟後激進

**3. CBF 處理 C**
- $h(x) = d_{\text{edge}} - d_{\text{box to edge}}$
- CBF 約束保證箱子不出桌面

**4. 兜底**
- Slack Variable + Fallback（位置控制兜底）

**Bonus（前沿）**：
- **Diffusion-MPC 當 proposal generator**：diffusion 生成候選接觸序列，CI-MPC 微調確保硬約束 — 解 NMPC 卡局部最優
- **Meta-MPC**：meta-learning 上層根據「物體類別 token」直接輸出 MPC cost weights

</details>

### 情境題 5：四足機器人跑酷面前有一棵樹，從左繞和從右繞在 cost 上幾乎一樣，NMPC 的 gradient solver 卡在鞍點跳不動，怎麼辦？

**分析鏈**：

<details>
<summary>完整推理</summary>

**病因**：**多模態最優解 (multi-modal solution)**。
- 從左繞 = 局部最優 A
- 從右繞 = 局部最優 B
- 中間正對樹 = 鞍點
- Gradient-based solver (SQP / IPOPT) 線性化後卡在鞍點不動 → 撞樹

**正解：Sampling-based MPC (MPPI) 或混合策略**

**方案 1：MPPI (Model Predictive Path Integral)**
- GPU 並行撒上萬條軌跡候選 $\{u^{(i)}\}$
- 每條 rollout 算 cost $S(\tau_i)$
- **Softmax 權重加權平均**：

$$
w_i = \frac{\exp(-S(\tau_i)/\lambda)}{\sum_j \exp(-S(\tau_j)/\lambda)}, \quad u_{\text{opt}} = u_{\text{nominal}} + \sum_i w_i \cdot \text{noise}_i
$$

- 左右兩片軌跡束並行探索 → softmax 自然坍縮到代價較低那側
- 不需梯度 → 天然免疫鞍點與非光滑接觸
- **框架**：MuJoCo MPC (mjpc) 內建 Predictive Sampling；JAX 實作大量 GPU 並行

**方案 2：Diffusion-MPC**
- Diffusion 作為 proposal generator 給多模態候選軌跡
- SQP 下游微調

**方案 3：混合架構**
- 高層 RL / VLA 決策「從左還是從右」（離散決策）
- MPC 追蹤給定 waypoints

**為什麼 MPPI 在足式機器人重要**：踩不平地面接觸力是硬拐角（非光滑），NMPC Jacobian 爆炸。MPPI 靠暴力採樣繞過求導 → 天然穩定。**劣勢**：控制維度 > 20 時採樣指數爆炸 → 必結合 SRBD 降階。

</details>

### 情境題 6：嵌入式 STM32 MCU 上要跑四足 NMPC，算力完全不夠，有什麼選項？

**分析鏈**：

<details>
<summary>完整推理</summary>

**算力受限 (SWaP constraint) 的具身智能必備策略**：

**層次 1：降階模型**
- **SRBD**（Single Rigid Body Dynamics）取代完整浮基動力學：狀態 30 維 → 12 維
- 凸 QP 用 OSQP 幾 ms 解完

**層次 2：Real-Time Iteration (RTI)**
- 每週期只跑 1 次 SQP 迭代，固定 μs 級預算
- Warm Start 連續性確保準度

**層次 3：Event-triggered MPC**
- 穩態時直接 shift 上週期解，省 50–80% 計算
- 狀態偏差超閾值才喚醒 NLP
- Lyapunov 觸發條件保證不失控

**層次 4：Amortized MPC + CBF Safety Filter**
- 離線跑千萬次 MPC 收集 (state, $u^*$) → NN 克隆 → O(1) 推論
- 墊 CBF QP Filter 補硬約束保證
- 微秒級即可輸出

**層次 5：MPPI on 微型 GPU**
- 若有 Jetson Orin Nano 這類小 GPU → MPPI 平行撒 1000 條
- 無 GPU → forget it

**組合策略（ANYmal 家用機實際做法）**：
- SRBD + RTI + acados (sparse HPIPM)
- Event-triggered 節能模式（走路時 10 Hz 解，跑酷時 100 Hz 解）
- WBC 依舊 1 kHz 硬即時

**面試金句**：「**從週期性強計算變按需計算**是邊緣具身智能的唯一出路。大模型 + MPC 共存在 MCU 上 = 大腦用 Diffusion / VLA 算粗軌跡，MPC Event-triggered 只在必要時精算。」

</details>

### 情境題 7：模擬器裡 MPC 完美追蹤，上真機馬達劇烈抖動，怎麼系統性 debug？

**分析鏈**：

<details>
<summary>完整推理</summary>

**三步走排查**：

**Step 1：通訊 / 執行延遲 (最常見)**
- 模擬器零延遲；真機 DDS/CAN 可能 5–20 ms 延遲 + 電流環 1–2 ms
- **症狀**：MPC 用當前狀態預測，但實際執行時狀態已變 → 每週期「過度修正」→ 高頻振盪
- **修法**：
  - 延遲補償：預測當前狀態「加上延遲時間的模擬推進」
  - 歷史輸入：用上週期 $u$ 作當前狀態的歷史輸入

**Step 2：馬達頻寬不足**
- MPC 輸出 $u$ 中含超過馬達 Nyquist 的高頻成分 → 硬體追不上 → 振盪
- **修法**：
  - 頻率掃頻辨識真實馬達頻寬（Bode 圖）
  - MPC 代價函數調高 $\Delta u$ 懲罰項 $R_\Delta$
  - 後濾波：MPC 輸出 → 低通濾波器 → 馬達

**Step 3：模型失配**
- 諧波減速器柔性、齒輪背隙、摩擦 — 模擬器常簡化
- **修法**：
  - SysID 實測真實參數重校準模型
  - 加入 GP 殘差學習（GP-MPC）或 Meta-MPC 在線適應
  - Offset-free MPC 處理恆定偏差

**順序建議**：先檢 Step 1（延遲），這最容易且影響最大；再 Step 2（頻寬）；最後才是 Step 3（模型）。**面試加分**：提到這個順序本身就是工程思維。

</details>

### 情境題 8：要開發一款家用教育機器人，成本有限、需跑 open-world 語音指令、同時保證安全，怎麼組合 MPC 與其他模組？

**分析鏈**：

<details>
<summary>完整推理</summary>

**「大腦 / 小腦 / 脊髓」三層架構**（具身智能下一範式的標準答案）：

**大腦層（1–10 Hz）：Foundation Model**
- **VLA (Vision-Language-Action)**：RT-2 / OpenVLA / π0
- 接收語音 + 視覺 → 輸出 1–2 秒 3D 目標路徑點 (waypoints / trajectory)
- **擅長**：語義理解、long-horizon planning、多模態泛化
- **缺**：沒有硬約束保證、推理延遲高

**小腦層（10–100 Hz）：Hierarchical MPC**
- **High-level MPC (10–50 Hz)**：接 VLA waypoints + SRBD 動力學 → 質心軌跡 + 接觸排程
- **Adaptive / Robust MPC**：
  - Adaptive Tube 處理未知質量
  - Stochastic / GP-MPC 處理感測雜訊
- **Event-triggered** 節能（家用不能吵、不能發熱）

**脊髓層（1 kHz）：WBC / PID + CBF Safety Filter**
- WBC 分配馬達扭矩
- CBF QP Safety Filter 守物理硬約束（關節限、摩擦錐、避障）
- ISO 13849 / ISO/TS 15066 功能安全

**為什麼這樣分層**：
- VLA 端到端 RL 無法通過工業認證（無硬約束）
- 純 MPC 無法處理「把紅色杯子拿過來」這種 open-world 語義指令
- **大腦慢思考 + 小腦快預測 + 脊髓硬反射** → 大算力與嚴謹數學的完美結合

**現實對照**：
- **Tesla Optimus**：RL 高層 + 傳統 MPC/WBC 底層
- **Figure 02**：OpenAI VLM + 自研 MPC
- **1X Neo**：類似混合架構

**面試金句**：「純 end-to-end RL 過不了工業安全認證；純 MPC 處理不了 open-world 語義指令。**VLA + MPC + CBF QP 是工業級具身智能的唯一範式**。」

</details>

## 面試角度

### 1. MPC vs LQR vs PID 的本質差異

**為什麼這是重點**：入門題、面試 5 分鐘內必考。答得「MPC 比 PID 好」是含糊；答得「MPC 看未來 N 步 + 處理約束，LQR 看未來但無約束，PID 只看現在和過去」是合格；能答「LQR = 無約束 MPC 的閉解版」是優秀。

**兩分鐘講法**：PID 反應式、只用當前誤差，無法處理硬約束。LQR 會算未來最優，但只有無約束二次型才有閉解。**MPC = 帶硬約束的 LQR 在線版** — 每週期解帶約束 QP，執行第一步，滾動重規劃。MPC 天然處理力矩飽和、摩擦錐、避障 — PID 辦不到，LQR 不考慮。

### 2. 為什麼機械臂不用 MPC 但四足必須用 MPC

**為什麼這是重點**：分辨懂原理 vs 背術語的標準題。

**兩分鐘講法**：**機械臂 = 全驅動固定基座**。只要算 $O(n)$ 逆動力學 (Computed Torque Control) + PID 1 kHz 就能完美追蹤 — 沒有「預測 GRF 防摔」的需求，NMPC 算力純浪費（除非要處理 manipulation 接觸）。**四足 = 浮動基座欠驅動** — 沒有 GRF 前瞻規劃就會摔。MPC 的真正戰場是欠驅動 + 浮基 + 接觸切換系統。

### 3. SRBD + WBC 分層（MIT Cheetah / ANYmal 標配）

**為什麼這是重點**：足式機器人面試一定問；展現「系統設計」視角而非單一算法。

**兩分鐘講法**：完整浮基動力學 $O(n^4)$ 100 Hz 解不動 → SRBD 降階（單剛體 + 足端 GRF）把 QP 壓到 1–3 ms。但 SRBD 忽略腿動力學 → 底層用 1 kHz WBC（完整動力學 + 關節極限 + 任務優先級）補回來。**「MPC 管質心路線圖，WBC 管每條腿怎麼踩」** — 兩層一起才是足式機器人骨架。

### 4. NMPC 求解器選型：acados / RTI / Multiple Shooting

**為什麼這是重點**：分辨「聽過 NMPC」vs「跑過 NMPC」的細節題。

**兩分鐘講法**：硬即時首選 **acados + RTI + HPIPM**。RTI 每週期只跑 1 次 SQP 迭代，固定 μs 預算；Multiple Shooting ($x$ 和 $u$ 都是決策變數) 誤差限於小區間、並行友善、四足標配；HPIPM 處理 sparse KKT。概念驗證用 CasADi + IPOPT 符號運算方便但部署慢。OCS2 (ETH) 對足式接觸切換有特化。

### 5. Feasibility 三層救援 + 70% 工程精力

**為什麼這是重點**：產業 vs 學術的分水嶺；面試官最想聽「你真的上過產線」。

**兩分鐘講法**：工業 MPC **70% 精力在 Slack / Fallback / Safety Filter**。三層救援：Layer 1 Slack Variable 軟化硬約束（代價加 $\lambda \varepsilon^2$），允許微小違反換可行解；Layer 2 沿用上週期解第二步；Layer 3 solver NaN → 純阻尼控制軟趴下。**沒這三層，一個擾動產線馬達就暴衝**。

### 6. CBF-MPC 保證 Safety-Critical

**為什麼這是重點**：協作機器人、自駕 L4、醫療機器人 — 所有安全嚴苛場景必考。

**兩分鐘講法**：Terminal Constraint 只保證最後一步 $x_N$ 在安全集，中間時域可能撞。**CBF**：找 $h(x) \ge 0$ 定義安全集，要求 $\dot{h} + \alpha h \ge 0$ → **前向不變集**（進入就絕不出去）。CBF 嵌 MPC 每個預測步 + 微秒級 QP Safety Filter 過濾 NN 輸出 — 這才是工業級標準。相關標準 ISO/TS 15066。

### 7. Tube vs Stochastic MPC 選型

**為什麼這是重點**：考察對「絕對安全」vs「機率安全」的理解深度。

**兩分鐘講法**：**Tube MPC** 把硬約束向內收縮一個 tube 半徑，Nominal MPC 在收縮邊界內規劃 → 最壞擾動下真實軌跡只在管道內震盪 → **絕對安全**（L4 自駕標準）。**Stochastic MPC** 擾動建為機率分佈，約束 $P(h \le 0) \ge 1-\varepsilon$；與 GP-MPC 結合，$3\sigma$ 收縮 → **機率安全**（四足跑酷，接受 5% 滑移換敏捷）。Min-Max 計算爆炸，幾乎不用。

### 8. Contact-Implicit MPC — 2024 manipulation game-changer

**為什麼這是重點**：2024–2025 最熱的 manipulation 研究方向；能提這個 = 跟上前沿。

**兩分鐘講法**：傳統 MPC 必須預定 mode sequence；CI-MPC 把 **接觸力 $\lambda$ 和接觸距離 $\phi$ 同時當優化變數**，用互補約束 $\phi \lambda = 0$（MPCC 鬆弛為 $\phi \lambda \le \varepsilon$）。**優化器自己發現「空手 → 碰箱 → 推」的 emergent behavior**。意義：**把依賴專家 FSM 的 heuristics 變成嚴謹數值優化問題**。Drake + MIT Cheetah CI-TrajOpt 是代表。

### 9. MPPI Sampling-based MPC — 多模態 + 非光滑接觸

**為什麼這是重點**：和 gradient-based 的分水嶺；足式跑酷、非光滑接觸的唯一解。

**兩分鐘講法**：GPU 並行撒上萬條軌跡，Softmax 權重加權平均得最優控制。**不需梯度** → 天然免疫鞍點、非光滑接觸、離散 action。經典場景：**四足面前一棵樹，左右繞成本相近，gradient solver 卡鞍點 → MPPI 並行探索左右兩片自然坍縮到較低那側**。劣勢：控制維度 > 20 採樣爆炸 → 必結合 SRBD 降階。框架 MuJoCo MPC (mjpc)、JAX 實作。

### 10. Lie Group MPC — 旋轉的正確處理

**為什麼這是重點**：分辨「懂旋轉」vs「亂用四元數」；航太、飛控、足式機器人必考。

**兩分鐘講法**：歐拉角有 Gimbal Lock，四元數加 $\|q\|^2=1$ 硬約束讓 NLP Zig-zagging 慢幾十倍。**正解**：狀態在 SO(3) / SE(3) 李群，優化變數在切空間 $\mathfrak{so}(3) \cong \mathbb{R}^3$；誤差用對數映射 $e_R = \log(R_{\text{target}}^T R)^\vee$。指數映射拉回流形 $R_{k+1} = R_k \cdot \exp(\Delta\omega^\wedge)$。PX4 飛控、MIT Cheetah、ANYmal 全採用。

### 11. Offset-free MPC — 產品級必備

**為什麼這是重點**：沒這個 MPC 就是學術玩具，面試官一問立刻看出工程素養。

**兩分鐘講法**：Nominal MPC 本質是 P-controller，對恆定擾動有穩態誤差（風、未知質量、未建模摩擦）。**Disturbance Augmentation** + Kalman/Luenberger Observer 在線估計 $d_k$，MPC 前饋抵消 → Offset-free。Velocity Form MPC 優化 $\Delta u$ 自然形成積分器。**類比：MPC 的 Offset-free = PID 的 I 項**。

### 12. VLA + MPC 下一範式

**為什麼這是重點**：2025+ 具身智能業界方向；展示「業界嗅覺」的 talking point。

**兩分鐘講法**：純端到端 RL 缺硬約束保證 → 過不了工業安全認證。純 MPC 處理不了 open-world 語義指令。**必然的混合體**：VLA (RT-2 / OpenVLA / π0) 高層 1–10 Hz 語義規劃 → MPC 100 Hz–1 kHz 底層追蹤 → WBC / CBF QP 絕對安全兜底。「大腦慢思考、小腦快預測、脊髓硬反射」。Tesla Optimus、Figure、1X Neo 都走這路線。

### 13. Event-triggered MPC + SWaP 邊緣 AI

**為什麼這是重點**：家用 / 農業 / 微型無人機必考；分辨「跑過實機」vs「紙上談兵」。

**兩分鐘講法**：穩態時每週期重解是浪費。**Event-triggered**：$\|x - \hat{x}\|_Q \ge \sigma\|x\|$ 才喚醒 NLP，否則 shift 上週期解 → 省 50–80% 計算。Self-triggered 更激進 — 預測下次觸發時刻，期間 sleep。**ISS + Lyapunov 保證衰減** → 不失控前提下砍 80% 計算。「**週期性強計算 → 按需計算 (Compute-on-demand)**」是大模型 + MPC 在 STM32 共存的唯一出路。

## 延伸閱讀

### 教科書 / 經典

- **Rawlings, Mayne, Diehl — *Model Predictive Control: Theory, Computation, Design***：MPC 聖經，理論 + 算法 + 工程實作全面。
- **Borrelli, Bemporad, Morari — *Predictive Control for Linear and Hybrid Systems***：線性 + 混合系統 MPC 權威教材。
- **Grüne, Pannek — *Nonlinear Model Predictive Control***：NMPC 穩定性與計算，學術視角。

### 核心 Paper

- **Di Carlo et al. (2018)** — Convex MPC for Quadrupedal Locomotion（MIT Cheetah SRBD MPC 起源）
- **Diehl et al. (2005)** — Real-Time Iteration Scheme for NMPC（RTI 奠基作）
- **Ames et al. (2019)** — Control Barrier Functions: Theory and Applications（CBF 綜述）
- **Hogan & Rodriguez (2020)** — Contact-Implicit Trajectory Optimization（CI-MPC 核心）
- **Williams et al. (2017)** — Model Predictive Path Integral Control（MPPI 起源）
- **Kabzan et al. (2019)** — Learning-based MPC for autonomous racing（GP-MPC 工程落地）
- **Howell et al. (2022)** — Predictive Sampling + MuJoCo MPC（mjpc 論文）
- **Black, Sreenath (2023)** — Neural network-based MPC with safety guarantees via CBF（Amortized MPC + Safety Filter）

### 框架 / 工具

- **acados**（[https://docs.acados.org](https://docs.acados.org)）— 工業硬即時 NMPC 首選，C/C++/Python 綁定，RTI + HPIPM
- **CasADi**（[https://web.casadi.org](https://web.casadi.org)）— 符號微分 + IPOPT 介面，研究驗證必備
- **OCS2**（[https://leggedrobotics.github.io/ocs2](https://leggedrobotics.github.io/ocs2)）— ETH 足式 MPC，SLQ/DDP
- **Drake**（[https://drake.mit.edu](https://drake.mit.edu)）— MIT manipulation，CI-TrajOpt / MPCC
- **MuJoCo MPC (mjpc)**（[https://github.com/google-deepmind/mujoco_mpc](https://github.com/google-deepmind/mujoco_mpc)）— DeepMind sampling-based MPC
- **manif**（[https://github.com/artivis/manif](https://github.com/artivis/manif)）— Lie Group C++ / Python 庫
- **OSQP / HPIPM / qpOASES** — QP 求解器三劍客

### 模擬器 Demo

- **MuJoCo 3 + mjpc**：內建 cheetah / humanoid / manipulation demo，GPU 並行 MPPI
- **Isaac Sim + Isaac Lab**：ANYmal + 四足 MPC 官方教學
- **Gazebo + ROS 2 + acados**：四足 MPC ROS 整合範本
- **Drake + examples/manipulation_station**：CI-MPC 推箱子範例

### 業界與前沿

- **Boston Dynamics Atlas 技術報告**：離線軌跡庫 + 線上 MPC 跟蹤
- **Tesla Optimus AI Day 影片**：端到端 RL + 傳統 MPC/WBC
- **ANYmal C 技術白皮書**：OCS2 + latent-space 地形估計 MPC
- **π0 (Physical Intelligence)** / **OpenVLA / RT-2**：VLA + MPC 前沿對照
- **Diffusion-MPC 系列 Paper**：2024 arXiv 最新 manipulation 前沿