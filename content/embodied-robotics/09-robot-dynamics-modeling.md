---
title: "動力學建模（牛頓-歐拉與拉格朗日）"
prerequisites: ["08-inverse-kinematics"]
estimated_time: 60
difficulty: 4
tags: ["dynamics", "newton-euler", "lagrangian", "computed-torque", "passivity", "adaptive-control", "contact", "featherstone"]
sidebar_position: 9
---

# 動力學建模（牛頓-歐拉與拉格朗日）

## 你將學到

- 兩句話區分 kinematics 與 dynamics，並精確說明 forward dynamics / inverse dynamics 在模擬器和控制器裡扮演的角色
- 遇到「高速追不上軌跡」「sim-to-real 力矩偏差」「接觸後機器人彈飛」「熱機半小時前後表現差 3 倍」時，知道問題在動力學建模的哪一層（參數、摩擦、接觸、柔性、執行器頻寬）
- 面試被問到 computed torque control、Passivity-Based Control、Featherstone ABA、CWC-QP、Teacher-Student Privileged Learning 時，能在兩分鐘內講清楚關鍵邏輯
- 掌握工業級 System Identification 8 步流程（含物理一致性校驗），知道 Slotine-Li 自適應控制何時會漂移（PE 條件）
- 理解浮動基座機器人（humanoid / quadruped）的 Underactuated Dynamics、角動量守恆、Contact Wrench Cone 如何串起整個 WBC 架構

## 核心概念

**精確定義**：**Robot dynamics** 研究的是力/力矩 (forces/torques) 與運動 (motion) 之間的關係。相比 kinematics 只管「幾何上怎麼動」，dynamics 回答的是「要出多大力才能這樣動」以及「給了這個力會怎麼動」。整個學科的核心產出是一條**運動方程式 (equation of motion)**，把慣性、科氏力、重力、接觸力四項串起來。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：關節位置 $q$、關節速度 $\dot{q}$、關節加速度 $\ddot{q}$（inverse dynamics）或關節力矩 $\tau$（forward dynamics）
- **輸出**：inverse dynamics 輸出所需力矩 $\tau$；forward dynamics 輸出加速度 $\ddot{q}$
- **下游**：inverse dynamics → computed torque control (CTC) 前饋、Passivity-Based Control、阻抗控制的力矩補償、TrajOpt / NMPC 的 Hessian；forward dynamics → 物理模擬器積分引擎（MuJoCo / PyBullet / Isaac Sim）、MPC rollout、可微模擬器的梯度路徑
- **閉環節點**：核心在**控制層**，但向上延伸到規劃（TrajOpt 用動力學當約束）、向下延伸到執行器（FOC 電流環的力矩來源）；對浮動基座機器人，還支配著感知層（IMU + 接觸模型推估地形）

**一句話版本**：「動力學模型是機器人的肌肉骨骼說明書 — 讓模擬器預言未來、控制器精準下力矩、學習算法知道自己在物理世界的哪個象限。」

### 標準動力學方程（固定基座）

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau
$$

- $M(q) \in \mathbb{R}^{n \times n}$：**慣性矩陣**（inertia matrix），正定對稱。物理意義：每個關節加速 1 rad/s$^2$ 需要多少力矩，且會受當前姿態 $q$ 影響（手臂伸直比彎曲慣性大）
- $C(q, \dot{q})\dot{q} \in \mathbb{R}^n$：**科氏力 + 離心力**（Coriolis + centrifugal）。物理意義：關節在動時，速度交叉項產生的「假力」— 轉彎時的離心甩動
- $g(q) \in \mathbb{R}^n$：**重力項**。物理意義：即使靜止不動，重力也在拉手臂往下，伺服馬達得持續出力對抗
- $\tau \in \mathbb{R}^n$：關節力矩（或力），控制器的輸出

**Inverse dynamics**（已知運動求力矩）— computed torque control 前饋基礎：

$$
\tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q)
$$

**Forward dynamics**（已知力矩求加速度）— 物理模擬器每個 timestep 在做的事：

$$
\ddot{q} = M(q)^{-1}\left[\tau - C(q, \dot{q})\dot{q} - g(q)\right]
$$

**Ṁ − 2C 斜對稱性質**（passivity 與 Lyapunov 穩定性基石）：

$$
x^T\left[\dot{M}(q) - 2C(q, \dot{q})\right]x = 0, \quad \forall x
$$

物理意義：這個性質保證系統被動（passive）— 科氏力/離心力**不做功**，只做能量重分配。搭配李雅普諾夫函數 $V = \tfrac{1}{2}\dot{q}^T M \dot{q}$，求導後科氏項被抵消，控制器穩定性有嚴格數學保障。所有能量型控制器（PBC、IDA-PBC、Slotine-Li 自適應）的證明命脈。

### 兩種主力建模方法對比

| 特性 | Newton-Euler (RNEA) | Lagrangian |
|------|-------------------|------------|
| 本質 | 力的平衡（遞迴） | 能量微分 |
| 複雜度 | O(n) | O(n$^3$ ~ n$^4$) |
| 主要用途 | 即時 inverse dynamics | 離線符號推導 / 分析 |
| 輸出 | 直接給 $\tau$ | 給完整 $M, C, g$ 矩陣 |
| 優勢 | 極速、適合嵌入式 | 結構清晰、便於穩定性證明 |
| 業界首選 | 1 kHz 以上即時控制 | 控制理論推導、System ID |

<details>
<summary>深入：Featherstone 空間代數、RNEA/CRBA/ABA 為何 O(n)</summary>

**為什麼 3D 向量分離算很差勁**：用純 $\omega, v$ 分離表示時，每個座標變換要分別對 $\omega$ 和 $v$ 做旋轉+平移，重複的矩陣乘法與叉乘讓 FLOPs 爆炸。

**Plücker 座標（Spatial Vectors）**：把運動與力各自合併成 6D 向量：

$$
\nu = \begin{bmatrix}\omega \\ v\end{bmatrix} \in \mathbb{R}^6, \quad f = \begin{bmatrix}n \\ f_{\text{linear}}\end{bmatrix} \in \mathbb{R}^6
$$

空間慣性變 $6 \times 6$ 矩陣，每層運動學/力學傳遞只需一次 6×6 × 6×1 乘法 — FLOPs 壓縮到最小。

**RNEA (Recursive Newton-Euler Algorithm) — O(n) inverse dynamics**：
- **Forward pass**（基座 → 末端）：`ν_i = ν_{i-1} + S_i · q̇_i`，傳遞空間速度/加速度（「連桿怎麼動」）
- **Backward pass**（末端 → 基座）：`f_i = I_i · a_i + ν_i ×_f (I_i · ν_i)`，投影到關節軸 `τ_i = S_i^T · f_i`（「馬達要出多少力」）

**CRBA (Composite Rigid Body Algorithm) — O(n²) 建構 M(q)**：從末端向基座遞迴累加子樹慣量，整個 $M$ 矩陣直接列出。

**ABA (Articulated Body Algorithm) — O(n) forward dynamics**：
1. 定義**鉸接體慣量**（Articulated Inertia）— 考慮從某節點向下所有子鏈的等效慣量
2. 三次 O(n) 遞迴：forward 算速度偏置 → backward 傳鉸接慣量 → forward 算加速度
3. **完全繞過 $M^{-1}$** — 這是 ABA 的精髓

**面試陷阱：ABA 真的永遠比 CRBA+Cholesky 快嗎？**
- 理論 O(n) vs O(n³)，但有常數項
- 低 DoF（n=6, 7）時 Cholesky 分解高度利用 SIMD + Cache Locality，常數項極小
- **實務通常 n > 9 才顯出優勢** → 6 軸機械臂不一定強制 ABA，humanoid（30+ DoF）則必須

**Spatial Cross Product 的對偶（誤用直接算錯）**：
- Motion × Motion: `ν₁ ×_m ν₂`
- Motion × Force: `ν ×_f f = -(ν ×_m)^T f`
- Pinocchio 嚴格區分兩套 API，寫 custom algorithm 時混用會徹底錯

**真實失敗案例**：30-DoF 人形機器人用 `M⁻¹(τ-h)` 算前向動力學 → 2 ms/週期超時 → 改 ABA 降至 50 μs。

</details>

<details>
<summary>深入：RNEA 完整遞迴公式 + 從 RNEA 萃取 M/C/g 的實務技巧</summary>

**Forward pass**（從 base → end-effector，$i = 1 \ldots n$）：

> 向量 convention（Featherstone / Pinocchio 標準）：**每個下標變數 $v_i$ 表達在對應的 frame $i$**（例如 $\omega_{i-1}$ 在 frame $i-1$、$\omega_i$ 在 frame $i$）；$R_i$ 是 frame $i-1 \to i$ 的旋轉矩陣（因此 $R_i^T$ 把 frame $i-1$ 的向量變換回 frame $i$）；$\hat{z} = [0,0,1]^T$ **在 frame $i-1$ 或 frame $i$ 都是 $[0,0,1]$**（joint 軸與 DH 轉軸對齊），因此 $\omega_{i-1} + \dot q_i \hat z$ 的加法在 frame $i-1$ 下進行、再由 $R_i^T$ 整個轉回 frame $i$；$r_i$ 是 frame $i-1$ 原點指向 frame $i$ 原點的向量（表達在 frame $i$）。重力透過 base pseudo-acceleration $\ddot p_0 = -g$ 注入，後續公式不再顯式列重力項。

$$
\omega_i = R_i^T (\omega_{i-1} + \dot{q}_i \hat{z})
$$
$$
\dot{\omega}_i = R_i^T (\dot{\omega}_{i-1} + \ddot{q}_i \hat{z} + \omega_{i-1} \times \dot{q}_i \hat{z})
$$
$$
\ddot{p}_i = R_i^T \ddot{p}_{i-1} + \dot{\omega}_i \times r_i + \omega_i \times (\omega_i \times r_i)
$$

**Backward pass**（從 end-effector → base，$i = n \ldots 1$）：

$$
f_i = m_i \ddot{p}_{c_i} + R_{i+1} f_{i+1}
$$
$$
n_i = I_i \dot{\omega}_i + \omega_i \times (I_i \omega_i) + R_{i+1} n_{i+1} + s_i \times m_i \ddot{p}_{c_i} + r_{i+1} \times R_{i+1} f_{i+1}
$$
$$
\tau_i = n_i^T \hat{z}
$$

**從 RNEA 萃取 $M, C, g$ 的工程技巧**（有時控制律需要顯式 $M$）：

- $g(q) = \text{RNEA}(q, 0, 0)$ — 把基座加速度設為 $-g$（重力），其他全零
- $C(q, \dot{q})\dot{q} = \text{RNEA}(q, \dot{q}, 0) - g(q)$ — 零加速度+真實速度
- $M(q)$ 第 $i$ 行 $= \text{RNEA}(q, 0, e_i) - g(q)$ — 單位向量法，$n$ 次 RNEA 組出完整 $M$

**Analytic Derivatives 對 NMPC/RL 的重要性**：
- NMPC 需要 $\partial \tau / \partial q, \partial \tau / \partial \dot{q}$ 作 optimizer 的 Jacobian
- Finite Difference：7-DoF 要 21 次 RNEA + 截斷誤差，單步 15 ms 直接爆 1 kHz 預算
- **Pinocchio `computeRNEADerivatives`**：一次擴展遞迴直接算，速度 >10×，降至 1.2 ms
- API：`pinocchio::computeRNEADerivatives(model, data, q, v, a, dtau_dq, dtau_dv, dtau_da)`

**真實失敗案例**：Franka 7-DoF NMPC 用 finite diff 算 Jacobian → 單步 15 ms 超過 1 kHz → 換 analytical gradients 降至 1.2 ms。

</details>

<details>
<summary>深入：Lagrangian 能量推導、Christoffel 符號、2-link 封閉式 M/C/g</summary>

**Lagrangian**：

$$
\mathcal{L}(q, \dot{q}) = T(q, \dot{q}) - V(q), \quad T = \tfrac{1}{2}\dot{q}^T M(q)\dot{q}
$$

**Euler-Lagrange 方程**：

$$
\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i
$$

展開恰好給出 $M\ddot{q} + C\dot{q} + g = \tau$。

**科氏矩陣的 Christoffel 符號（這是 $\dot{M} - 2C$ 斜對稱性的來源）**：

$$
C_{ij} = \sum_{k=1}^n c_{ijk} \dot{q}_k, \quad c_{ijk} = \tfrac{1}{2}\left(\frac{\partial M_{ij}}{\partial q_k} + \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{jk}}{\partial q_i}\right)
$$

**2-link planar arm 封閉式結果**（用於面試手推 + 驗證程式）：

$$
M(q) = \begin{bmatrix}
m_1 l_{c1}^2 + m_2(l_1^2 + l_{c2}^2 + 2l_1 l_{c2}\cos q_2) + I_1 + I_2 & m_2(l_{c2}^2 + l_1 l_{c2}\cos q_2) + I_2 \\
m_2(l_{c2}^2 + l_1 l_{c2}\cos q_2) + I_2 & m_2 l_{c2}^2 + I_2
\end{bmatrix}
$$

注意 $M$ 與 $q_2$（手肘角）相關而與 $q_1$（肩膀角）無關 — 肩膀馬達感受到的慣性隨手肘彎曲而變。

**何時用 Lagrangian**：
- 離線符號推導（SymPy / MATLAB Symbolic Toolbox），得到封閉式方程後做代碼生成
- 低自由度系統教學分析（2-3 DOF 手算）
- 控制理論穩定性分析（代入 Lyapunov 函數）
- System Identification 的 regressor 建構

**何時不用**：$n \geq 6$ 的即時控制。符號展開後矩陣元素爆炸增長，線上計算 $M(q)$ 本身就是 O(n$^3$)。

</details>

### Passivity-Based Control（PBC）— 能量觀點的控制哲學

**核心哲學**：把機器人看成**能量轉換器**，儲存能量 $\leq$ 輸入能量 $+$ 耗散能量 → 系統**永不發散**。

**PBC vs PD+G(q)**：
- PD+G(q)：只保證點到點（set-point）穩定
- PBC：主動做**能量塑形 (Energy Shaping)** → 高速追蹤、接觸互動都嚴格無源

**IDA-PBC (Interconnection & Damping Assignment)**：修改閉迴路的等效質量與勢能，賦予機器人「虛擬被動彈簧-阻尼外殼」，接觸時絕不暴走。

**陷阱**：數位離散化延遲 + 感測器濾波相位滯後會**注入虛假能量**，破壞 Passivity → 高頻接觸震盪。實務上要用 Passivity Observer + Passivity Controller (POPC) 持續監測能量收支。

**平台**：Franka Panda、KUKA iiwa 的關節力矩模式內建 Passivity-based controller，這是它們能安全做人機互動的根本原因。

### 浮動基座動力學（Floating-Base Dynamics）— humanoid / quadruped 必備

**未致動 6 DoF 的殘酷事實**：基座 3 平移 + 3 旋轉沒有馬達驅動，機器人只能靠**地面反作用力 (GRF)** 或推力改變基座運動 → **Underactuated Dynamics**。

$$
\begin{bmatrix}M_{\text{base}} & M_{bj} \\ M_{bj}^T & M_{jj}\end{bmatrix}\begin{bmatrix}\ddot{x}_{\text{base}} \\ \ddot{q}_j\end{bmatrix} + h = \begin{bmatrix}0 \\ \tau\end{bmatrix} + \begin{bmatrix}J_c^T F_c \\ J_{c,j}^T F_c\end{bmatrix}
$$

物理意義：上半區塊等式右邊是 $0 + J_c^T F_c$ — 基座加速度**完全由接觸力決定**。馬達只能透過驅動關節蹬地產生 $F_c$，再間接改變基座。

**角動量守恆黃金原則**：空中飛行相無接觸力，重力不產生力矩 → **總角動量 $k$ 嚴格守恆**。貓空中翻正、MIT Cheetah 後空翻、體操選手抱膝加速旋轉，全部遵守這條。

**Centroidal Momentum Matrix (CMM)**：

$$
A(q)\dot{q} = \begin{bmatrix}l \\ k\end{bmatrix}
$$

$l$ = 線動量、$k$ = 角動量。$A(q)$ 把關節速度映射到質心動量，是 WBC 高層規劃的核心。Pinocchio API：`pinocchio::ccrba(model, data, q, v)` → `data.Ag, data.hg`。**Convention 提示**：本文採 Pinocchio 排列（上 3 維 linear、下 3 維 angular，對應 `hg = [l; k]`）；Orin & Goswami 2008 原文採相反排序 $[k; l]$，跨 paper 抄公式時務必先檢查 convention，否則 index 容易錯位。

<details>
<summary>深入：Contact Wrench Cone (CWC) 與 Multi-contact QP — 為什麼淘汰了 ZMP</summary>

**ZMP 的局限**：
- 假設所有接觸點共水平面
- 假設摩擦力無限大
- 人形機器人「一手扶牆 + 雙腳踩不同階梯」在 3D 空間直接失效

**CWC 統一四類接觸約束**：
1. **法向壓力** $f_z > 0$（地面不能拉機器人）
2. **庫倫摩擦錐** $\sqrt{f_x^2 + f_y^2} \leq \mu f_z$（不打滑）
3. **CoP 約束**（壓力中心在腳底支撐多邊形內，不翻腳）
4. **Yaw 扭矩約束**（腳底不原地打滑旋轉）

線性化後 → 多面體錐 $W \cdot F_c \geq 0$。

**CWC-QP + WBC 分層架構**：

$$
\min_{F_c} \|\sum F_c - m(\ddot{p}_{\text{des}} - g)\|^2 + \|\sum (p_i \times F_c) - \dot{k}_{\text{des}}\|^2
$$
$$
\text{s.t. } W \cdot F_c \geq 0
$$

**為何是具身智能的主流骨架**：多接觸操作（雙手抱箱 + 雙腳站立）在 ETH ANYmal / MIT Cheetah 公開論文中都以 CWC-QP 為底層；Tesla Optimus / Figure / Unitree H1 未公開完整細節，但社群廣泛推測走類似路線。

**斜坡陷阱**：若未把 CWC 從世界系旋轉到局部接觸面系 → 法向力估計錯誤 → 啟動瞬間打滑劈腿。

**真實失敗案例**：四足跳躍落地未在 WBC 約束 $\dot{k}$ → 前腿觸地產生大俯仰力矩 → 前空翻砸地。修正：MPC cost 加對 $\dot{k}$ 的強懲罰。

</details>

**常用 API**（業界工具鏈）：

| 層級 | 套件 | 介面示例 |
|------|------|----------|
| 快速 C++ | Pinocchio | `rnea(model, data, q, v, a)` → $\tau$ |
| 快速 C++ | Pinocchio | `aba(model, data, q, v, tau)` → $\ddot{q}$ |
| 解析導數 | Pinocchio | `computeRNEADerivatives(...)` → $\partial\tau/\partial q, \partial\tau/\partial \dot{q}$ |
| 質心動量 | Pinocchio | `ccrba(...)` → CMM + $l, k$ |
| 符號/微分 | CasADi + Pinocchio | 自動微分 + C 代碼生成，NMPC 用 |
| 模擬器 | MuJoCo | `mj_forward` / `mj_inverse`（Soft contact） |
| 模擬器 | Drake | `CalcInverseDynamics`（嚴格 LCP 可用） |
| 可微模擬 | MuJoCo MJX / Brax / Dojo | JIT + `jax.grad` 穿透物理 |
| ROS 2 | KDL | `ChainIdSolver_RNE::CartToJnt(...)` |

## 直覺理解

**類比：開車上山彎道**：
- **$M(q)$ = 車重**（且隨姿態變）：手臂伸直像滿載，收回像空車
- **$C(q, \dot{q})\dot{q}$ = 轉彎的甩動**：多關節同時高速動產生的假力
- **$g(q)$ = 上坡踩油門**：靜止也要持續出力對抗重力
- **$\tau$ = 油門/煞車輸出**：控制器給馬達的指令

**接觸動力學類比：剎車點 vs 輕觸方向盤**：
- 剛體碰撞像急踩煞車 — 瞬間衝量、速度跳變
- Soft contact 像按到 ABS — 系統容許微小「穿透」換取連續性
- RL 訓練在 soft contact 上可反傳梯度；Drake 的嚴格 LCP 像手排車，準但慢

**模擬器驗證直覺**：

1. **MuJoCo 重力補償實驗**：把所有關節控制力矩設為 $\tau = g(q)$（純重力補償），機械臂應懸浮在任意姿態不動。少補一點往下掉，多補一點往上飄。
2. **高速甩動觀察**：Isaac Sim 裡讓機械臂只用 PD 控制快速揮動，觀察末端嚴重偏離軌跡（$C$ 和 $M\ddot{q}$ 項沒補償）。加上 computed torque 前饋後，追蹤誤差銳減。
3. **慣性姿態依賴性**：PyBullet 讓同一關節以相同力矩加速，手臂分別在伸直 vs 彎曲 — 伸直時加速度明顯小（$M(q)$ 大）。
4. **角動量守恆**：MuJoCo 讓人形機器人空中甩動手臂，觀察軀幹反向旋轉（總角動量嚴守為零，局部擾動必被反作用抵消）。
5. **Contact Stiffness 調參**：MuJoCo 調 `solimp / solref`，太軟 RL 學會穿透抓取作弊，太硬時間步太大直接爆炸彈飛 — 親手調過才懂 sim-to-real 的接觸陷阱。

## 實作連結

**四個典型工程場景**：

1. **Computed Torque Control (CTC)**：inverse dynamics 前饋 + PD 回饋。前饋抵消 ~90% 非線性，PD 只修殘差。把非線性系統線性化成 $\ddot{e} = K_p e + K_d \dot{e}$。

2. **物理模擬器內核**：MuJoCo / PyBullet 每個 timestep 做 forward dynamics — 拿 policy 輸出的 $\tau$，算 $\ddot{q}$，數值積分得下一刻 $q, \dot{q}$。精度取決於動力學參數準確度。

3. **阻抗控制的力矩補償**：阻抗控制需要 inverse dynamics 做重力補償 + 慣性解耦，才能讓末端表現 mass-spring-damper 行為。沒動力學補償的阻抗控制在高速/大負載下嚴重失準。

4. **TrajOpt 與 NMPC**：軌跡優化把動力學當等式約束，iLQR 需要一階 Jacobian、DDP 需要二階 Hessian；CITO (Contact-Implicit TrajOpt) 用互補條件讓優化器**自動發現步伐**（Atlas 跑酷、MIT Cheetah 的訓練神器）。

**Code 骨架**（C++，Pinocchio 版）：

```cpp
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/parsers/urdf.hpp>

pinocchio::Model model;
pinocchio::urdf::buildModel("robot.urdf", model);
pinocchio::Data data(model);

// 1. Inverse dynamics (RNEA, O(n))
Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

// 2. Forward dynamics (ABA, O(n))
Eigen::VectorXd ddq = pinocchio::aba(model, data, q, v, tau);

// 3. 完整 M(q) (CRBA, O(n²))
pinocchio::crba(model, data, q);   // fills data.M (upper triangle)

// 4. 解析 RNEA 導數（NMPC 用）
pinocchio::computeRNEADerivatives(model, data, q, v, a);
// data.dtau_dq, data.dtau_dv, data.M (dtau_da)

// 5. 質心動量矩陣（浮動基座）
pinocchio::ccrba(model, data, q, v);
// data.Ag 是 CMM, data.hg 是 [l; k]
```

<details>
<summary>深入：完整 Python 實作 — Pinocchio + CTC + 重力補償 + Passivity 觀察</summary>

```python
import numpy as np
import pinocchio as pin

# === 載入模型 ===
model = pin.buildModelFromUrdf("panda.urdf")
data = model.createData()
nq = model.nq

# === 基礎動力學計算 ===
q = np.zeros(nq)
v = np.zeros(nq)
a = np.zeros(nq)

# RNEA: O(n) inverse dynamics
tau = pin.rnea(model, data, q, v, a)

# ABA: O(n) forward dynamics（完全繞過 M⁻¹）
tau_in = np.ones(nq) * 0.5
ddq = pin.aba(model, data, q, v, tau_in)

# 取得顯式 M, C, g（需要時）
M = pin.crba(model, data, q)                                  # O(n²)
pin.computeCoriolisMatrix(model, data, q, v)
C = data.C
g = pin.computeGeneralizedGravity(model, data, q)

# === Computed Torque Control ===
def computed_torque_control(q, v, q_des, v_des, a_des, Kp, Kd):
    """前饋 RNEA + PD 回饋，讓誤差動態呈二階線性。"""
    e = q_des - q
    de = v_des - v
    a_cmd = a_des + Kp @ e + Kd @ de
    return pin.rnea(model, data, q, v, a_cmd)

Kp = np.diag([100.0] * nq)
Kd = np.diag([20.0] * nq)  # 近似臨界阻尼 Kd = 2*sqrt(Kp)

# === 純重力補償模式（教學用：讓機器人懸浮） ===
def gravity_compensation(q):
    return pin.computeGeneralizedGravity(model, data, q)

# === Passivity 能量收支監測（POPC 雛形） ===
def energy_audit(q, v, tau_applied, dt, energy_state):
    """
    E_total = E_kinetic + E_potential - ∫τᵀv dt
    任何時刻 E_total 應非增長（被動系統的定義）
    """
    pin.crba(model, data, q)
    M = data.M
    T = 0.5 * v @ M @ v                              # 動能
    V = pin.computePotentialEnergy(model, data, q)   # 位能
    work_in = tau_applied @ v * dt                   # 本 step 輸入功
    energy_state["input"] += work_in
    E_total = T + V
    passivity_residual = E_total - energy_state["input"]
    # 若 passivity_residual 單調增 → 系統注入虛假能量（離散化誤差），要降控制增益
    return passivity_residual

# === 驗證 Ṁ - 2C 斜對稱性 ===
def verify_skew_symmetry(q, v):
    pin.crba(model, data, q)
    pin.computeCoriolisMatrix(model, data, q, v)
    # 用有限差分逼近 Ṁ
    eps = 1e-6
    q_next = q + v * eps
    pin.crba(model, data, q_next)
    M_next = data.M
    pin.crba(model, data, q)
    M_now = data.M
    M_dot = (M_next - M_now) / eps
    S = M_dot - 2 * data.C
    asym_error = np.linalg.norm(S + S.T)  # 應接近 0
    return asym_error
```

**重點**：
- `rnea` 是 Newton-Euler 遞迴，O(n)
- `crba` 算完整慣性矩陣，O(n²)；`aba` 用 Articulated Body Algorithm，O(n)
- CTC 的 PD 增益配合慣性矩陣調，$K_d = 2\sqrt{K_p}$ 是臨界阻尼起點
- Passivity 監測是接觸互動的救命稻草，感測器濾波/離散化都會破壞被動性

</details>

<details>
<summary>深入：工業級 System Identification 8 步完整流程（含 LMI 物理一致性校驗）</summary>

動力學模型的準確度取決於參數品質。CAD 的質量、慣量只是初始值，真機的摩擦、線纜、減速器、末端工具都沒算進去。業界標準 8 步：

**Step 1：解析模型推導**
- SymPy / Pinocchio URDF → 回歸矩陣 $Y$·$\pi$
- 動力學方程線性化為 $\tau = Y(q, \dot{q}, \ddot{q}) \cdot \pi$，$\pi$ 是要辨識的參數向量（質量、質心、慣性張量 6 項、摩擦係數）

**Step 2：摩擦模型擴充**
- 基本：$\tau_f = F_c \text{sign}(\dot{q}) + F_v \dot{q}$
- 進階：加 Stribeck 項 $F_s e^{-|\dot{q}|/v_s} \text{sign}(\dot{q})$
- 最終：LuGre 動態模型（見下方 Q3）

**Step 3：激勵軌跡設計**
- **目標**：最小化 $\text{cond}(Y)$，讓所有參數充分激勵
- **方法**：5th-order Fourier series，MATLAB `fmincon` 優化頻率/振幅
- **限制**：關節極限、速度極限、自碰撞

**Step 4：1 kHz 實機採集**
- Franka FCI / UR RTDE：記錄 $(q, \dot{q}, \tau_{\text{measured}})$
- $\ddot{q}$ 從 $\dot{q}$ 數值微分得

**Step 5：filtfilt 零相位低通**
- Butterworth 4 階 20 Hz cutoff
- **filtfilt**（前向+反向濾波）關鍵：零相位延遲，不放大 $\ddot{q}$ 高頻雜訊

**Step 6：WLS 求解**

$$
\hat{\pi} = (Y^T W Y)^{-1} Y^T W \tau_{\text{measured}}
$$

權重矩陣 $W$ 反映各關節量測雜訊的方差倒數。

**Step 7：物理一致性校驗（LMI/SDP）**
- 這一步是學術和工業的分水嶺
- 純 WLS 可能解出**負質量、非正定慣性張量**（數學上最優，物理上不合法）
- 用 Linear Matrix Inequality 約束：$m_i > 0$，$I_i \succ 0$（正定），並於 principal axes 下滿足**三角不等式** $I_{jj} + I_{kk} \geq I_{ii}$ 對所有 $\{i,j,k\}$ 的排列都成立（等價於 **pseudo-inertia matrix $\succeq 0$** 的 LMI 形式，見 Traversaro, Prete & Nori 2016、Wensing, Kim & Slotine 2017；註：$\text{tr}(I_i) > \lambda_{\max}$ 只是此條件的必要弱化，單獨使用會漏掉物理上不可實現的 inertia）
- 套件：SciPy `cvxpy` + SDP solver (Mosek / SCS)

**Step 8：交叉驗證**
- 設計新軌跡（不同於激勵軌跡）
- 比較預測 $\hat{\tau} = Y \hat{\pi}$ 與實測 $\tau$
- RMSE < 額定扭矩 5% 才算過關

**Pinocchio API**：`pinocchio::computeJointTorqueRegressor()` 直接給 $Y$。

**真實失敗案例**：UR5 跳過 Step 7，辨識出負慣量 → 控制器數值發散 → 真機撞桌。加 LMI 約束後才穩定。

</details>

<details>
<summary>深入：Slotine-Li 自適應控制 + Persistent Excitation 死穴</summary>

**Slotine-Li Adaptive Control** 架構：

1. **滑動曲面**：$s = \dot{e} + \Lambda e$，$\Lambda$ 正定
2. **控制律**：

$$
\tau = Y(q, \dot{q}, \dot{q}_r, \ddot{q}_r) \hat{\pi} - K s
$$

其中 $\dot{q}_r = \dot{q}_d + \Lambda e$ 是「參考速度」

3. **參數更新律**：

$$
\dot{\hat{\pi}} = -\Gamma Y^T s
$$

$\Gamma$ 是正定自適應增益矩陣。

**李雅普諾夫證明**：$V = \tfrac{1}{2}s^T M s + \tfrac{1}{2}\tilde{\pi}^T \Gamma^{-1} \tilde{\pi}$，求導用 $\dot{M} - 2C$ 斜對稱性抵消科氏項 → $\dot{V} = -s^T K s \leq 0$。因 $V$ 有下界且 $\dot V \leq 0$，故 $s \in L^\infty \cap L^2$；再由 **Barbalat's Lemma**（$\dot V$ 均勻連續）推得 $s \to 0$，進而 $e \to 0$（但不保證 $\hat{\pi} \to \pi$）。

**Persistent Excitation (PE) 死穴**：
- $e \to 0$ 不等於參數估計收斂
- 需要 $\int_t^{t+T} Y^T Y \, d\tau \succ \alpha I$（PE 條件）才保證 $\hat{\pi} \to \pi$
- 若 $Y$ 秩虧損（機器人只走某條曲線），系統無法區分「負載變重」vs「摩擦變大」→ 估計參數沿不可觀測子空間漂移 → 最終算出負質量

**工程解法**：
- 加入微小正弦探索軌跡確保 $Y$ 滿秩（類似 RL 的 exploration noise）
- 強制**把估計參數投影回物理可行域**（Projection Operator）
- 若只需誤差收斂而非參數收斂，接受不完美但穩定

**RLS vs Slotine-Li 權衡**：

| 方法 | 收斂速度 | 雜訊魯棒 | 適用場景 |
|------|----------|----------|----------|
| RLS + 遺忘因子 | 極快（瞬間抓重物） | 敏感，有 covariance windup | UR5 抓未知工具 |
| Slotine-Li | 慢 | 絕對穩定 $e \to 0$ | 高自由度、精密追蹤 |

**平台案例**：UR5 抓取未知重量工具時用 RLS 線上更新等效慣量，確保 CTC 前饋持續精準。

</details>

<details>
<summary>深入：接觸動力學 — LCP / Soft Contact / MuJoCo 調參陷阱</summary>

**剛體碰撞 = Hybrid System**：
- 微秒內速度瞬時跳變（衝量 impulse）
- 微分方程不連續
- Runge-Kutta 等通用積分器直接爆炸

**LCP (Linear Complementarity Problem) 表述**（Drake、ODE 用）：
- 接觸距離 $\phi \geq 0$
- 接觸力 $\lambda \geq 0$
- 互補 $\phi \cdot \lambda = 0$
- 庫倫摩擦錐：$\lambda_t \leq \mu \lambda_n$ → NCP（非線性互補），求解極耗時

**MuJoCo Soft Contact**：
- 放棄嚴格 LCP，允許微小穿透 → 連續凸最佳化
- `solimp`（彈性/黏性）+ `solref`（阻尼/回復時間） → 接觸力變穿透深度的**連續函數**
- RL 梯度可反傳（Differentiable Simulators 的根基）

**XML 範例**：
```xml
<geom solref="0.02 1" solimp="0.9 0.95 0.001" friction="1 0.005 0.0001"/>
```

**Sim-to-Real 陷阱**：
- Contact stiffness 太軟 → RL 學會「穿透物體作弊抓取」（穿模 exploit）
- 太硬 → 時間步長不夠小 → 物體一碰就爆炸彈飛（數值不穩定）
- 斜坡/階梯 → 若未把摩擦錐從世界系旋轉到局部接觸面系 → 打滑劈腿

**業界慣例**：RL 訓練用 MuJoCo（快、可微、接觸柔軟），最終驗證用 Drake（嚴格 LCP、慢但準）。

**精密裝配的底線**：peg-in-hole 微米級公差**絕不能盲信模擬接觸力**。必須 Domain Randomization（contact stiffness ±50%、摩擦 ±30%）+ 真實阻抗控制器兜底。

</details>

<details>
<summary>深入：Neural Dynamics、Differentiable Simulators、Teacher-Student 架構</summary>

**灰箱 Residual Learning**：

$$
\tau_{\text{total}} = \tau_{\text{RNEA}}(q, \dot{q}, \ddot{q}; \pi_{\text{base}}) + \tau_{\text{NN}}(q, \dot{q}, \ddot{q}, T_{\text{temp}})
$$

物理模型打底，NN 學殘差（線纜、溫度、未建模摩擦）。比純 NN 資料效率高 10-100×，比純 RNEA 適應力強。

**GPR (Gaussian Process Regression) 的獨特價值**：不只輸出預測，還給**不確定性 $\sigma^2$**。高 $\sigma$ 時退回魯棒控制（保守動作），低 $\sigma$ 時信 NN 模型。

**Teacher-Student Privileged Learning**（ETH ANYmal / Boston Dynamics 的 sim-to-real 公式）：
1. **Teacher 階段**：模擬器裡讀 privileged info（地面 $\mu$、質量 $m$、地形高度圖） → 訓練完美策略
2. **Student 階段**：只用 proprioception（關節位置/速度歷史 + 雜訊 IMU）
3. **RNN/LSTM 隱式推斷**：從「腳底打滑的速度波動歷史」學會推估 $\mu$ → 蒸餾出 Teacher 級救車能力

**Differentiable Simulators (MuJoCo MJX, Brax, Dojo)**：
- 傳統 REINFORCE 高方差梯度估計
- 可微模擬器直接讓 $\partial s_{t+1}/\partial a_t$ 穿透物理引擎
- 「蒙特卡洛盲搜」變精確梯度下降 → sample efficiency × 100

**MPPI (Model Predictive Path Integral)**：GPU 並行採樣上萬條 SRBD（Single Rigid Body Dynamics）軌跡 → Softmax 權重混合 → 即時最優控制。Tesla Optimus / Unitree H1 底層共識。

**Neural ABA（Tesla/Figure 傳言）**：ABA 雖 O(n) 但在萬環境並行仍是 CPU 瓶頸 → NN 擬合 ABA → GPU O(1) 矩陣乘 → 百萬級並行訓練成為可能。

**面試關鍵**：
- 純 RL 工程師遇 Sim2Real Gap → 盲加 Domain Randomization → 策略保守僵化
- 懂動力學的：Action 空間設為「期望質心加速度」或「阻抗 $\Delta K/D$」，底層交 WBC → **大腦 RL + 小腦 WBC 混合架構**
- **失敗案例**：純 Model-Free RL 學會高頻抖動卡物理 bug 穿模移動；修正：reward 加 $\tau^2$ 能量懲罰 + action 低通濾波

</details>

<details>
<summary>深入：TrajOpt、iLQR/DDP、Direct Collocation、CITO — 動力學在軌跡優化的角色</summary>

**運動學規劃 + 後期時間參數化為什麼失敗**：只考慮幾何避障 → 忽略動力學非線性耦合 → 高速時科氏/慣性呈幾何級數放大 → **Torque Saturation** → 軌跡不可執行。

**iLQR vs DDP 的導數差異**：
- **iLQR**：只用一階 Jacobian $A = \partial f/\partial x, B = \partial f/\partial u$
- **DDP**：完整二階 Hessian 張量 $\partial^2 f/\partial x^2$
- 後空翻等極動態動作狀態曲率極大，只有精確二階導才能捕捉「慣性矩陣隨姿態的曲率」→ 幾步迭代二次收斂

**Shooting vs Collocation**：

| 方法 | 決策變量 | 優點 | 缺點 |
|------|----------|------|------|
| **Shooting** | 僅 $u$ | 實作簡單 | 積分誤差指數放大，梯度爆炸 |
| **Direct Collocation** | $x$ + $u$ | 大膽探索違反物理的中間狀態，最後收斂 | 問題規模大 |

Collocation 把動力學轉等式約束：$x_{k+1} - x_k - f(x_{k+1}, u_k) \cdot dt = 0$。IPOPT 大膽探索，**極穩**。

**CITO (Contact-Implicit TrajOpt)**：
- 傳統必須預定 mode sequence（「何時哪隻腳落地」）
- CITO 把接觸距離 $\phi$ 與接觸力 $\lambda$ 當互補約束 $\phi \geq 0, \lambda \geq 0, \phi \cdot \lambda = 0$
- 優化器**自動發現步伐** (Automatic Gait Discovery)
- Atlas 跑酷、MIT Cheetah 訓練神器

**面試陷阱「完美軌跡模擬能跑真機發散」答題**：
- TrajOpt 開環解是**極度特化脆弱**的（恰卡在力矩極限）
- 真實摩擦/背隙 5% 誤差就不可執行
- **工程解法**：TrajOpt 只給 $(x_{\text{ref}}, u_{\text{ref}})$ 參考 + 前饋；底層**必須墊 LQR 或 WBC 閉環反饋**吸收殘差

</details>

## 常見誤解

1. **「慣性矩陣 $M(q)$ 是常數」** — 錯。$M(q)$ 隨姿態變化，伸直 vs 彎曲某些元素可能差 3-5 倍。用 PID 調好一個姿態，換個姿態可能就振盪或反應不足。**正確理解**：每個控制週期重算，或用 RNEA 前饋直接繞過。

2. **「PID 夠用，不需要動力學模型」** — 低速、輕負載時 PID 能湊合。高速時科氏/離心項變顯著，純 PID 只靠誤差驅動，追蹤誤差隨速度急劇惡化。**正確做法**：CTC = inverse dynamics 前饋 + PD 回饋，前饋抵消 ~90%，PD 只修殘差。

3. **「Lagrangian 方法太慢所以沒用」** — Lagrangian 不適合即時控制（O(n$^3$)+），但離線分析價值極高：SymPy 推完整 $M, C, g$ → 代碼生成 → 穩定性證明 → System ID regressor 建構。**Newton-Euler 線上跑、Lagrangian 離線推，兩者互補**。

4. **「摩擦用線性模型就夠了」** — 簡單庫倫 + 黏滯忽略了 Stribeck 效應、靜摩擦跳變、LuGre 動態刷毛、harmonic drive 非線性。精密操作或力控場景會直接導致 stick-slip 震盪。**正確做法**：至少 Stribeck + 黏滯，嚴謹場景上 LuGre + 線上 Disturbance Observer。

5. **「接觸靠調剛性係數就能模擬好」** — Soft contact 的 `solimp / solref` 像一對相互牽制的旋鈕：太軟 RL 穿模作弊，太硬時間步不夠小導致爆炸彈飛。**正確做法**：MuJoCo 訓練 + Drake 驗證 + Domain Randomization，關鍵部件（如精密裝配）必須真機閉環補強。

6. **「RL policy 不需要動力學模型」** — 純 Model-Free 可以，但 sample efficiency 極低且 sim-to-real gap 難補。現代做法是 **Privileged Learning + Differentiable Sim + 大腦 RL + 小腦 WBC**。動力學模型不是被 RL 取代，而是變成了 Action Space 的結構先驗。

## 練習題

<details>
<summary>Q1：你用 CTC 控制一支 7-DoF 機械臂做高速 pick-and-place，發現追蹤誤差比預期大 3 倍，而且越快越差。你會怎麼系統性診斷？</summary>

**完整推理鏈**：

1. **速度敏感性測試**：暫時降速到 10%，看誤差是否回到可接受範圍。慢速 OK 快速不行 → 問題在與速度/加速度相關的項（$C\dot{q}$ 或 $M\ddot{q}$）
2. **參數比對**：用 `pin.rnea` 算出預測力矩，和馬達實測電流比較。RMSE > 20% → 參數不準
3. **啟動 SysID 8 步流程**（見「深入：SysID」摺疊塊）：
   - 解析模型 → 摩擦擴充 → 激勵軌跡設計（Fourier + $\min \text{cond}(Y)$）
   - 1 kHz 採集 → filtfilt 零相位濾波
   - WLS 求解 → **LMI 物理一致性校驗**（避免負質量）
   - 新軌跡交叉驗證 RMSE < 5%
4. **摩擦模型檢查**：辨識後靜態力矩改善但高速仍偏差 → harmonic drive wave generator 摩擦未建模 → 加 Stribeck 或 LuGre
5. **保底手段**：上 Disturbance Observer 線上補未建模干擾 `d_est = LPF(τ_real - τ_model)`
6. **避開陷阱**：別急著加大 PD 增益 — 參數不準時高增益會激發高頻振盪

**結論**：問題根源幾乎都是動力學參數不準 + 非線性摩擦。標準解法 SysID 8 步（含 LMI）+ DOB 保底 + 必要時 Slotine-Li 線上自適應。

</details>

<details>
<summary>Q2：你在做 sim-to-real，RL policy 在 MuJoCo 訓練完美，部署到真機就力矩指令完全不對、機器人亂抖。排查？</summary>

**完整推理鏈**：

1. **靜態查：重力補償**。真機靜止某姿態，讀馬達實際出力。和 MuJoCo `mj_inverse` 同姿態算的重力力矩比較。差 > 15% → 質量/慣量參數有問題
2. **動態查：摩擦與接觸剛性**。真機慢速來回運動量測力矩-速度曲線，看是否有 Stribeck 效應；MuJoCo 的 `solimp / solref` 跟真實地面硬度可能差兩個數量級
3. **查執行器延遲**。MuJoCo 力矩即時生效，真實馬達驅動器有 1-2 ms communication + 電流環響應延遲。1 kHz 下足以破壞穩定性
4. **查頻寬**。RL 可能學了高頻抖動（訓練時 action 無平滑懲罰），真機電流環跟不上就共振
5. **三管齊下**：
   - **Domain Randomization**：質量 ±20%、摩擦 ±30%、延遲 0-3 ms、contact stiffness ±50%
   - **System Identification**：更新 MuJoCo XML 的 `<body mass>` / `<joint damping>` / `solref`
   - **Action 平滑**：低通濾波 or reward 加 $\dot{\tau}^2$ 懲罰
6. **進階：Teacher-Student Privileged Learning**（見深入摺疊塊）。Teacher 讀 privileged $\mu, m, 地形$，Student 用 RNN 從歷史推估
7. **避開陷阱**：只做 DR 跳過 SysID 會讓 DR 範圍蓋不住真實偏差（重力差 50% 時 DR 的 ±20% 不夠）

**結論**：按「重力 → 摩擦 → 接觸 → 延遲 → 頻寬」五層排查，SysID 校準 nominal model + DR 增強魯棒性 + Teacher-Student 做隱式適應。

</details>

<details>
<summary>Q3：你要在 1 kHz 控制迴圈跑阻抗控制的 inverse dynamics，目標硬體是 ARM Cortex 嵌入式，200 μs 預算。怎麼架構？</summary>

**完整推理鏈**：

1. **演算法選型：RNEA 而非 Lagrangian**。$O(n)$ vs $O(n^3)$，7-DoF RNEA ~200 次浮點乘加。小 $n$ 時不一定要 ABA（CRBA+Cholesky 的 SIMD/Cache Locality 常數項極小）
2. **工具鏈：Pinocchio 或 CasADi 代碼生成**
   - 離線：Pinocchio 的 `computeMinverse` + CasADi 自動微分 → 純 C 代碼（扁平賦值、無動態分配、無虛函數）
   - 生成的代碼就是一堆 `double c1 = cos(q[0]); double s1 = sin(q[0]); ...`
3. **三角函數快取**：7 關節只需 14 次 `sin/cos`，用 CORDIC 或查表替代 `libm`
4. **記憶體策略**：所有中間變數 stack-allocated fixed-size array，絕不 `malloc`。RNEA 中間結果（每連桿 $\omega, \dot{\omega}, f, n$）用 `std::array<Vector3d, N>`
5. **作業系統設定**：
   - PREEMPT_RT Linux 內核
   - `isolcpus` + `taskset` 把控制執行緒釘在獨立核
   - `mlockall` 鎖記憶體分頁避免 page fault
   - CPU 頻率 governor 設 `performance`
6. **驗證即時性**：`clock_gettime(CLOCK_MONOTONIC)` 量 99.99-th percentile worst-case，穩定 < 200 μs（留一半給通訊/其他任務）
7. **極限榨取**：如果還不夠，用 `pinocchio::computeAllTerms` 批次 RNEA + CRBA + Jacobian 一次遍歷

**結論**：RNEA O(n) + 離線代碼生成 + zero-alloc + RT 內核 + CPU 核隔離，是嵌入式即時動力學的標準組合。量 worst-case 不量 average。

</details>

<details>
<summary>Q4：面試官問你「動力學模型 condition number 很差會有什麼問題？怎麼處理？」</summary>

**完整推理鏈**：

1. **定義**：$\kappa(M) = \sigma_{\max}/\sigma_{\min}$（最大奇異值除以最小奇異值）。$\kappa$ 大 → $M$ 接近奇異
2. **發生時機**：手臂完全伸直、某些關節慣量差距懸殊 → $\kappa$ 可達 100:1 或更高
3. **造成的問題**：
   - Forward dynamics 算 $\ddot{q} = M^{-1}(...)$ 時數值誤差被放大 $\kappa$ 倍 → 浮點精度有限時加速度誤差可達幾個量級
   - 控制器前饋力矩不準 → 某些關節過補償、某些欠補償 → 軌跡扭曲
4. **處理方法**：
   - **避免顯式 $M^{-1}$**：改 Cholesky 分解求解 $M\ddot{q} = b$（$M$ 正定保證成功）
   - **改 ABA**：Pinocchio 的 `aba()` 直接算 forward dynamics 不經 $M^{-1}$，數值穩定性更好（Featherstone 的精髓）
   - **規劃避開病態姿態**：軌跡優化代價函數加 $\kappa(M(q))$ 懲罰項
   - **Scaling/Regularization**：某些實作會加 $M + \lambda I$（Tikhonov）保證可逆
5. **對浮動基座**：情況更嚴重 — $M_{\text{base}}$ 不同方向慣量差異大，必須用空間代數的 Plücker 表示降低 condition 問題

**結論**：用 Cholesky 或 ABA，規劃避開病態姿態，必要時加正則化。面試時能具體講 ABA 繞過 $M^{-1}$ 的機制 +1。

</details>

<details>
<summary>Q5：一台 humanoid 在空中做後空翻，落地時軀幹嚴重前傾幾乎摔倒。你會怎麼分析？</summary>

**完整推理鏈**：

1. **理論根源：浮動基座 + 角動量守恆**
   - 空中飛行相無接觸力，重力不產生力矩 → **總角動量 $k$ 嚴格守恆**
   - 若起跳時 $k_{\text{takeoff}}$ 太大 → 落地時需要快速消除 → 靠腳踩地產生反向力矩
   - 但落地瞬間接觸模型若未把 $\dot{k}_{\text{des}}$ 加入 WBC 約束 → 接觸力分配只考慮平移 COM → 前腳觸地產生大俯仰力矩 → 前空翻砸地
2. **診斷**：
   - 檢查起跳階段 CMM 計算的 $k$ 是否在可控範圍（Pinocchio `ccrba`）
   - 檢查落地 WBC cost 是否包含 $\|\dot{k} - \dot{k}_{\text{des}}\|^2$
   - 檢查 CWC 約束是否正確從世界系旋轉到腳底局部系（斜坡/樓梯時尤其重要）
3. **修正**：
   - 起跳階段：MPC 規劃角動量剖面，起跳時刻 $k$ 不過大
   - 落地階段：WBC 加強角動量追蹤項，必要時用雙腳/手臂反向擺動（Zero-Momentum Turn）
   - 深層：用 CITO + DDP 做動力學可行的完整跳躍軌跡，一次解出蹬地-飛行-落地的接觸序列
4. **驗證**：MuJoCo 重現，觀察 $k$ 剖面是否被正確追蹤；真機加 IMU 讀取實際 $k$ 閉環反饋

**結論**：浮動基座的精髓是角動量守恆 + Underactuated Dynamics。沒有在 WBC 中顯式約束 $\dot{k}$，就等於放任空中姿態亂轉。這題是 humanoid 面試的 signature question。

</details>

<details>
<summary>Q6：UR5 早上測試完美，下午熱機 30 分鐘後頻繁觸發安全停機，接觸速度偏大。為什麼？</summary>

**完整推理鏈**：

1. **現象定位**：「早上 vs 下午」差異 → 時間相關 → 熱機效應
2. **物理原因**：諧波減速器（harmonic drive）內潤滑脂冷機時黏度極大，溫升後黏滯摩擦 $F_v$ 下降 20-30%
3. **錯誤連鎖**：
   - 冷機辨識的摩擦參數 $F_v^{\text{cold}}$ 被當 nominal 寫入 CTC 前饋
   - 熱機後真實 $F_v^{\text{hot}} < F_v^{\text{cold}}$
   - 前饋 `τ_ff = F_v^{\text{cold}} · q̇` **過補償** → 實際加速比期望大
   - 接觸速度暴衝 → 撞擊力超過安全閾值 → E-stop
4. **修正策略**：
   - **短期**：現場 warm-up 30 分鐘後再重新跑 SysID（工業機器人手冊常見要求）
   - **中期**：RLS + 遺忘因子線上追蹤溫度漂移，即時微調 $F_v$
   - **長期**：DOB（Disturbance Observer）直接估 `disturbance_est = LPF(τ_actual - τ_ideal)` 作前饋，不依賴摩擦模型
   - **接觸前兜底**：進入接觸 50 mm 前切換為阻抗控制，目標是低剛度安全接觸而非精準追蹤
5. **面試 bonus**：這是「為什麼工業機器人需要 warm-up」的經典題，沒做過現場的人答不出來

**結論**：熱機效應是摩擦模型的軟肋。靜態 SysID 不夠，需要 online adaptation（RLS / DOB）+ 接觸情境切換阻抗控制。

</details>

## 面試角度

1. **前饋 + 回饋解耦是動力學控制的核心架構** — 區分「會調 PID」與「懂控制」的分水嶺。**帶出**：「CTC 把非線性動力學用 inverse dynamics 前饋抵消 ~90%，PD 只修殘差，閉環誤差動態變線性二階。這是工業機器人標準做法，比純 PID 在高速、變負載下魯棒一個數量級。」

2. **RNEA O(n) vs Lagrangian O(n³)，且 ABA 在低 DoF 未必贏 Cholesky** — 展示你不只懂大 O，還懂常數項與硬體現實。**帶出**：「RNEA 是 O(n) 的 Newton-Euler 遞迴，7-DoF 在 ARM 上 50 μs 搞定。ABA 理論 O(n) 但常數大，n < 9 時 CRBA+Cholesky 靠 SIMD 往往更快 — 這是實務知識，只看 textbook 答不出來。」

3. **Passivity-Based Control 與 $\dot{M} - 2C$ 斜對稱性** — 控制理論深度。**帶出**：「$\dot{M} - 2C$ 斜對稱不是數學玩具，它保證科氏力不做功，把機器人當能量轉換器處理。PBC 主動做能量塑形，比 PD+G(q) 在接觸互動時更魯棒；Franka、KUKA iiwa 的 joint torque mode 都是 Passivity 底。」

4. **工業級 SysID 8 步 + LMI 物理一致性校驗** — sim-to-real 的第一步。**帶出**：「CAD 參數永遠不準，我的 SysID 流程是：解析 regressor → 摩擦擴充（含 Stribeck/LuGre）→ Fourier 激勵軌跡 `min cond(Y)` → 1 kHz 採集 filtfilt → WLS → **LMI 約束保證正定慣性張量** → 新軌跡 RMSE < 5%。跳過 LMI 可能解出負質量直接撞桌。」

5. **Slotine-Li Adaptive Control 的 PE 死穴** — 自適應控制深度。**帶出**：「Slotine-Li 用 $s = \dot{e} + \Lambda e$ 滑動面保證 $e \to 0$，但不保證 $\hat{\pi} \to \pi$ — 除非 Persistent Excitation 條件滿足。Y 秩虧損時系統分不清『負載』vs『摩擦』，估計會沿不可觀測子空間漂移。工程解法是加探索正弦 + 參數投影回物理可行域。」

6. **浮動基座 Underactuated + 角動量守恆** — humanoid / quadruped 的核心。**帶出**：「基座 6 DoF 沒馬達，機器人只能靠 GRF 改基座加速度 — 這是 Underactuated Dynamics。空中飛行相重力不產生力矩，**總角動量嚴格守恆** — 貓翻正、後空翻都是這條。WBC 必須在落地時約束 $\dot{k}_{\text{des}}$，否則俯仰力矩失控就摔。」

7. **CWC 超越 ZMP，多接觸的 QP 統一約束** — 現代 WBC 基石。**帶出**：「ZMP 假設共水平面 + 無限摩擦，3D 多接觸（扶牆+踩階）直接失效。CWC 統一四類約束：法向壓力、摩擦錐、CoP、yaw torque，線性化成多面體錐給 QP 求最優接觸力。ETH ANYmal / MIT Cheetah 公開論文採用此架構，人形機器人公司（Tesla Optimus / Figure）未公開細節但推測路線類似。」

8. **Teacher-Student Privileged Learning** — 具身 AI 的 sim-to-real 公式。**帶出**：「ANYmal / Boston Dynamics 路線：Teacher 在模擬器讀 privileged（μ、質量、地形），Student 只用 proprioception + RNN 從『打滑歷史』隱式推估摩擦。這樣做比單純 Domain Randomization 強，因為 Student 真的在線上學會『感覺』地面，而不是對著隨機化邊界調保守策略。」

9. **Differentiable Simulators + 大腦 RL + 小腦 WBC 混合架構** — 下一代具身 AI。**帶出**：「MuJoCo MJX / Brax / Dojo 讓 `∂s/∂a` 穿透物理引擎，REINFORCE 的高方差變精確梯度，sample efficiency ×100。進一步：RL action space 不直接輸出力矩，而是期望質心加速度或阻抗 $\Delta K/D$，底層交 WBC/CWC-QP。大腦 RL 做決策、小腦 WBC 做動力學可行性，這是多家人形機器人公司（Tesla / Figure 等）推測採取的路線。」

10. **Non-collocated Control 的右半平面零點** — 大型柔性臂面試 signature。**帶出**：「Canadarm 1.5 噸/17 m 臂馬達在關節、精度要求在末端 — 中間巨大柔性連桿讓傳遞函數出現右半平面零點（non-minimum phase），PID 拉高增益直接發散。必須 **Input Shaping（ZV shaper 讓兩脈衝相位差 180° 互抵）+ 奇異攝動分離慢/快子系統 + LQR 狀態觀測器**。」

## 延伸閱讀

- **Roy Featherstone,《Rigid Body Dynamics Algorithms》** — Spatial algebra + RNEA/ABA 聖經。想真正懂 O(n) forward dynamics 怎麼不用求逆必讀
- **Lynch & Park,《Modern Robotics》Ch8（Dynamics of Open Chains）** — 免費教材，screw theory 統一推導 Newton-Euler 和 Lagrangian
- **Siciliano et al.,《Robotics: Modelling, Planning and Control》Ch7** — 標準研究所級動力學章節，CTC + 阻抗控制覆蓋全
- **Khalil & Dombre,《Modeling, Identification and Control of Robots》** — System Identification 的工業級教材，Fourier 激勵軌跡 + WLS + LMI
- **《具身智能算法工程師 面試題》Ch1.4 動力學建模** — 面試考點精練題目集
- **Pinocchio 官方文檔** ([github.com/stack-of-tasks/pinocchio](https://github.com/stack-of-tasks/pinocchio)) — 業界最快 rigid body dynamics 庫，讀 source 學 Featherstone 實作
- **Drake tutorials** ([drake.mit.edu](https://drake.mit.edu)) — MIT 的 LCP + TrajOpt + CITO 實作範例
- **Russ Tedrake,《Underactuated Robotics》MIT 6.832 公開課** — Underactuated / 軌跡優化 / CITO 的權威教材
- **MuJoCo 官方文檔：Computation / Contact** — 調 soft contact 的 `solimp/solref` 必讀
- **論文：Swevers et al.,《Optimal Robot Excitation and Identification》** — SysID 激勵軌跡設計經典
- **論文：*LuGre Friction Model*（Canudas-de-Wit）** — 動態摩擦建模聖經
- **論文：*Learning Quadrupedal Locomotion over Challenging Terrain*（ETH, Lee et al. 2020）** — Teacher-Student Privileged Learning 的代表作
- **論文：*Differentiable Simulation for Manipulation*（MIT, Dojo 系列）** — 可微模擬器工業應用
- **MIT Cheetah / ANYmal / Atlas 公開技術報告** — 看真實系統怎麼堆 CWC-QP + WBC + MPC
