---
title: "動力學建模（牛頓-歐拉與拉格朗日）"
prerequisites: ["08-inverse-kinematics"]
estimated_time: 45
difficulty: 4
tags: ["dynamics", "newton-euler", "lagrangian", "computed-torque"]
sidebar_position: 9
---

# 動力學建模（牛頓-歐拉與拉格朗日）

## 你將學到

- 能用兩句話區分 kinematics 與 dynamics，並精確說明 forward dynamics 與 inverse dynamics 各自在模擬器和控制器裡扮演什麼角色
- 遇到「高速運動下 PID 追不上軌跡」或「sim-to-real 力矩偏差很大」時，知道問題出在動力學模型，能判斷該用 Newton-Euler 還是 Lagrangian 來解
- 面試被問到 computed torque control 時，能在兩分鐘內講清楚前饋＋回饋解耦的完整邏輯

## 核心概念

**精確定義**：**Robot dynamics** 研究的是力/力矩 (forces/torques) 與運動 (motion) 之間的關係。相比 kinematics 只管「幾何上怎麼動」，dynamics 回答的是「要出多大力才能這樣動」以及「給了這個力會怎麼動」。整個學科的核心產出是一條**運動方程式 (equation of motion)**，把慣性、科氏力、重力三項串起來。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：關節位置 $q$、關節速度 $\dot{q}$、關節加速度 $\ddot{q}$（inverse dynamics）或關節力矩 $\tau$（forward dynamics）
- **輸出**：inverse dynamics 輸出所需力矩 $\tau$；forward dynamics 輸出加速度 $\ddot{q}$
- **下游**：inverse dynamics → computed torque control (CTC) 前饋、阻抗控制的力矩補償；forward dynamics → 物理模擬器積分引擎（MuJoCo / PyBullet / Isaac Sim）、MPC 預測模型
- **閉環節點**：核心在**控制層**。規劃器給出期望軌跡 $(q_d, \dot{q}_d, \ddot{q}_d)$，inverse dynamics 算出前饋力矩抵消非線性項，PD 回饋只需處理殘餘誤差；forward dynamics 則是模擬器的心臟，讓 RL policy 能在虛擬世界「試跑」

**一句話版本**：「動力學模型是機器人的肌肉骨骼說明書 — 讓模擬器預言未來、控制器精準下力矩。」

**最少夠用的數學**：

1. **標準動力學方程（manipulator equation of motion）**：

$$
M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau
$$

- $M(q) \in \mathbb{R}^{n \times n}$：**慣性矩陣**（inertia matrix），正定對稱。物理意義：每個關節加速 1 rad/s$^2$ 需要多少力矩，且會受當前姿態 $q$ 影響（手臂伸直比彎曲慣性大）
- $C(q, \dot{q})\dot{q} \in \mathbb{R}^n$：**科氏力 + 離心力**（Coriolis + centrifugal）。物理意義：關節在動的時候，速度交叉項產生的「假力」— 轉彎時的離心甩動
- $g(q) \in \mathbb{R}^n$：**重力項**。物理意義：即使靜止不動，重力也在拉手臂往下，伺服馬達得持續出力對抗
- $\tau \in \mathbb{R}^n$：關節力矩（或力），控制器的輸出

2. **Inverse dynamics**（已知運動求力矩）：

$$
\tau = M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q)
$$

物理意義：「告訴我軌跡上每一點的位置、速度、加速度，我就算出每個馬達要出多少力。」這是 computed torque control 和軌跡前饋的基礎。

3. **Forward dynamics**（已知力矩求加速度）：

$$
\ddot{q} = M(q)^{-1}\left[\tau - C(q, \dot{q})\dot{q} - g(q)\right]
$$

物理意義：「告訴我現在每個馬達出多少力，我就預測手臂會怎麼加速。」這是物理模擬器每個 timestep 在做的事 — 解出 $\ddot{q}$ 後數值積分得到下一刻的 $q, \dot{q}$。

4. **Ṁ − 2C 斜對稱性質**（控制理論關鍵）：

$$
\dot{M}(q) - 2C(q, \dot{q}) \text{ 是斜對稱矩陣}
$$

物理意義：這個性質保證了系統的被動性（passivity），是 Lyapunov 穩定性證明的核心 — 能量不會憑空增加，控制器的穩定性有數學保障。

<details>
<summary>深入：Newton-Euler 遞迴演算法（RNEA）完整推導</summary>

**Recursive Newton-Euler Algorithm (RNEA)** 是 inverse dynamics 的 O(n) 實現，分兩個 pass：

**Forward pass（從 base → end-effector）**：遞迴傳播速度與加速度

對第 $i$ 個連桿：

$$
\omega_i = R_i^T \left(\omega_{i-1} + \dot{q}_i \hat{z}\right)
$$

$$
\dot{\omega}_i = R_i^T \left(\dot{\omega}_{i-1} + \ddot{q}_i \hat{z} + \omega_{i-1} \times \dot{q}_i \hat{z}\right)
$$

$$
\ddot{p}_i = R_i^T \left(\ddot{p}_{i-1}\right) + \dot{\omega}_i \times r_i + \omega_i \times (\omega_i \times r_i)
$$

- $\omega_i$：第 $i$ 連桿的角速度（在連桿 $i$ 的局部 frame）
- $\ddot{p}_i$：第 $i$ 連桿原點的線加速度
- $r_i$：從第 $i$ 關節到第 $i$ 連桿質心的向量
- $\hat{z} = [0, 0, 1]^T$：旋轉軸方向

質心加速度：

$$
\ddot{p}_{c_i} = \ddot{p}_i + \dot{\omega}_i \times s_i + \omega_i \times (\omega_i \times s_i)
$$

**Backward pass（從 end-effector → base）**：遞迴傳播力與力矩

$$
f_i = m_i \ddot{p}_{c_i} + R_{i+1} f_{i+1}
$$

$$
n_i = I_i \dot{\omega}_i + \omega_i \times (I_i \omega_i) + R_{i+1} n_{i+1} + s_i \times m_i \ddot{p}_{c_i} + r_{i+1} \times R_{i+1} f_{i+1}
$$

最終，每個關節的力矩：

$$
\tau_i = n_i^T \hat{z}
$$

**複雜度**：每個 pass 是 O(n)，總共 O(n)。對比 Lagrangian 方法需要先算完整個 $M(q)$（O(n$^3$) 或更差），RNEA 在即時控制場景有壓倒性優勢。

**為什麼是 O(n)**：每個連桿只處理一次，沒有矩陣求逆或矩陣乘法，只有局部的 3×3 旋轉和向量叉乘。

</details>

<details>
<summary>深入：Lagrangian 方法的能量推導與符號建模</summary>

**Lagrangian 方法**從能量出發：

$$
\mathcal{L}(q, \dot{q}) = T(q, \dot{q}) - V(q)
$$

- $T$：系統總動能（kinetic energy）
- $V$：系統總位能（potential energy）

**動能**的一般形式：

$$
T = \frac{1}{2} \dot{q}^T M(q) \dot{q}
$$

其中 $M(q)$ 就是慣性矩陣，它的每個元素：

$$
M_{ij}(q) = \sum_{k=\max(i,j)}^{n} \left[ m_k J_{v_k}^{(i)T} J_{v_k}^{(j)} + J_{\omega_k}^{(i)T} I_k J_{\omega_k}^{(j)} \right]
$$

$J_{v_k}^{(i)}$ 是第 $k$ 個連桿質心線速度對第 $i$ 個關節角的偏微分。

**Euler-Lagrange 方程**：

$$
\frac{d}{dt}\frac{\partial \mathcal{L}}{\partial \dot{q}_i} - \frac{\partial \mathcal{L}}{\partial q_i} = \tau_i
$$

展開後恰好得到 $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$。

**科氏矩陣的 Christoffel 符號**：

$$
C_{ij} = \sum_{k=1}^{n} c_{ijk} \dot{q}_k, \quad c_{ijk} = \frac{1}{2}\left(\frac{\partial M_{ij}}{\partial q_k} + \frac{\partial M_{ik}}{\partial q_j} - \frac{\partial M_{jk}}{\partial q_i}\right)
$$

**何時用 Lagrangian**：
- 離線符號推導（用 SymPy / MATLAB Symbolic Toolbox），得到封閉式方程後再做代碼生成
- 低自由度系統的教學或分析（2-3 DOF 手算還行）
- 需要做控制理論穩定性分析時，完整的 $M, C, g$ 矩陣形式方便代入 Lyapunov 函數

**何時不用**：$n \geq 6$ 的即時控制。符號展開後矩陣元素會爆炸性增長，且線上計算 $M(q)$ 本身就是 O(n$^3$)。

</details>

**兩種方法的比較**：

| 特性 | Newton-Euler (RNEA) | Lagrangian |
|------|-------------------|------------|
| 本質 | 力的平衡（遞迴） | 能量微分 |
| 複雜度 | O(n) | O(n$^3$ ~ n$^4$) |
| 主要用途 | 即時 inverse dynamics | 離線符號推導 / 分析 |
| 輸出 | 直接給 $\tau$ | 給完整 $M, C, g$ 矩陣 |
| 優勢 | 極速、適合嵌入式 | 結構清晰、便於穩定性證明 |
| 業界首選 | 1 kHz 以上即時控制 | 控制理論推導、System ID |

**常用 API**（業界工具鏈）：

| 層級 | 套件 | 介面示例 |
|------|------|----------|
| 快速 C++ | Pinocchio | `pinocchio::rnea(model, data, q, v, a)` → $\tau$ |
| 快速 C++ | Pinocchio | `pinocchio::aba(model, data, q, v, tau)` → $\ddot{q}$ (forward dynamics) |
| 符號推導 | CasADi + Pinocchio | 自動微分 + 代碼生成，MPC 用 |
| 模擬器 | MuJoCo | `mj_forward(model, data)` / `mj_inverse(model, data)` |
| 模擬器 | PyBullet | `p.calculateInverseDynamics(robot, q, dq, ddq)` |
| ROS 2 | KDL | `ChainIdSolver_RNE::CartToJnt(q, dq, ddq, wrenches, tau)` |

## 直覺理解

**類比：開車上山彎道**：
- **$M(q)$（慣性矩陣）= 車重**：車越重，加速踩油門要更深。但機器人的「車重」會隨姿態變 — 手臂伸直時慣性大（像車上滿載），手臂收回時慣性小（像空車）
- **$C(q, \dot{q})\dot{q}$（科氏＋離心力）= 轉彎的甩動**：高速過彎會被甩向外側。機器人多個關節同時高速動時，速度交叉項產生的「假力」就是科氏力和離心力
- **$g(q)$（重力）= 上坡踩油門**：即使車停在斜坡上不動，也要踩住煞車。機械臂即使靜止，伺服馬達也得出力對抗重力把手臂撐住
- **$\tau$（力矩）= 油門/煞車輸出**：控制器最終要給馬達的指令

**模擬器驗證直覺**：
1. **MuJoCo 重力補償實驗**：把所有關節的控制力矩設為 $\tau = g(q)$（純重力補償），機械臂應該懸浮在任意姿態不動。少補一點就往下掉，多補一點就往上飄
2. **高速甩動觀察**：在 Isaac Sim 裡讓機械臂只用 PD 控制快速揮動，觀察末端嚴重偏離軌跡（因為沒補償 $C$ 和 $M\ddot{q}$ 項）。加上 computed torque 前饋後，軌跡追蹤誤差銳減
3. **慣性姿態依賴性**：在 PyBullet 裡讓同一個關節以相同力矩加速，但手臂分別在伸直和彎曲姿態 — 伸直時加速度明顯更小，因為 $M(q)$ 變大了

## 實作連結

**三個典型工程場景**：

1. **Computed Torque Control (CTC)**：最經典的動力學應用。規劃器給 $(q_d, \dot{q}_d, \ddot{q}_d)$，inverse dynamics 算出前饋力矩 $\tau_{ff}$ 抵消 ~90% 的慣性 + 科氏 + 重力干擾，剩下 ~10% 誤差交給 PD 回饋修正。結果：把非線性機器人系統變成線性二階系統，PD 調參變超簡單。

2. **物理模擬器內核**：MuJoCo / PyBullet 每個 timestep 做的事就是 forward dynamics — 拿 RL policy 輸出的 $\tau$，算 $\ddot{q} = M^{-1}(\tau - C\dot{q} - g)$，再數值積分得下一刻狀態。模擬器精度直接取決於動力學參數（質量、慣量、摩擦係數）的準確度。

3. **阻抗控制的力矩補償**：阻抗控制裡，需要 inverse dynamics 做重力補償和慣性解耦，才能讓末端表現出期望的 mass-spring-damper 行為。沒有動力學補償的阻抗控制在高速或大負載時會嚴重失準。

**Code 骨架**（C++，Pinocchio 版）：

```cpp
#include <pinocchio/algorithm/rnea.hpp>      // inverse dynamics
#include <pinocchio/algorithm/aba.hpp>       // forward dynamics
#include <pinocchio/parsers/urdf.hpp>

// 載入模型
pinocchio::Model model;
pinocchio::urdf::buildModel("robot.urdf", model);
pinocchio::Data data(model);

// Inverse dynamics: (q, dq, ddq) → τ
Eigen::VectorXd q(model.nq), v(model.nv), a(model.nv);
// ... 填入當前狀態 ...
Eigen::VectorXd tau = pinocchio::rnea(model, data, q, v, a);

// Forward dynamics: (q, dq, τ) → ddq
Eigen::VectorXd ddq = pinocchio::aba(model, data, q, v, tau);

// 純重力補償（ddq=0, dq=0 時的 inverse dynamics）
Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, data, q);
```

<details>
<summary>深入：完整 Python 實作（Pinocchio + CTC 控制迴路）</summary>

```python
import numpy as np
import pinocchio as pin

# === 載入模型 ===
model = pin.buildModelFromUrdf("panda.urdf")
data = model.createData()
nq = model.nq  # 關節數

# === Inverse dynamics: RNEA ===
q = np.zeros(nq)        # 關節位置
v = np.zeros(nq)        # 關節速度
a = np.zeros(nq)        # 關節加速度（期望的）

tau = pin.rnea(model, data, q, v, a)
print(f"Inverse dynamics 力矩: {tau}")

# === Forward dynamics: ABA ===
tau_input = np.ones(nq) * 0.5  # 假設輸入力矩
ddq = pin.aba(model, data, q, v, tau_input)
print(f"Forward dynamics 加速度: {ddq}")

# === 取得慣性矩陣、科氏矩陣、重力向量 ===
M = pin.crba(model, data, q)          # 慣性矩陣 M(q)
pin.computeCoriolisMatrix(model, data, q, v)
C = data.C                             # 科氏矩陣 C(q, dq)
g = pin.computeGeneralizedGravity(model, data, q)  # 重力向量

# === Computed Torque Control (CTC) 迴路 ===
def computed_torque_control(
    model, data, q, v,
    q_des, v_des, a_des,
    Kp, Kd
):
    """
    CTC = 前饋 (inverse dynamics) + PD 回饋
    把非線性系統線性化成 ë = 0（理想情況）
    """
    # 位置誤差與速度誤差
    e = q_des - q
    de = v_des - v

    # 修正後的期望加速度（加入 PD 回饋）
    a_cmd = a_des + Kp @ e + Kd @ de

    # Inverse dynamics: 算出完整力矩
    tau = pin.rnea(model, data, q, v, a_cmd)
    return tau

# 控制增益
Kp = np.diag([100.0] * nq)   # 位置增益
Kd = np.diag([20.0] * nq)    # 速度增益（臨界阻尼: Kd = 2*sqrt(Kp)）

# 模擬一步
dt = 0.001  # 1 kHz
q_des = np.array([0.5, -0.3, 0.0, -1.5, 0.0, 1.2, 0.0])
v_des = np.zeros(nq)
a_des = np.zeros(nq)

tau_cmd = computed_torque_control(
    model, data, q, v, q_des, v_des, a_des, Kp, Kd
)
print(f"CTC 輸出力矩: {tau_cmd}")

# 用 forward dynamics 積分
ddq = pin.aba(model, data, q, v, tau_cmd)
v_new = v + ddq * dt
q_new = q + v_new * dt  # 簡單 Euler 積分（實務用 semi-implicit）
```

**重點注意**：
- Pinocchio 的 `rnea` 就是 Newton-Euler 遞迴，O(n)
- `crba` 算完整慣性矩陣，O(n$^2$)；`aba` 用 Articulated Body Algorithm 做 forward dynamics，O(n)
- CTC 的 PD 增益要配合慣性矩陣調，$K_d = 2\sqrt{K_p}$ 是臨界阻尼的起點

</details>

<details>
<summary>深入：System Identification — 如何辨識動力學參數</summary>

動力學模型的準確度取決於參數品質。CAD 給的質量、慣量只是初始值，真機的摩擦、線纜、減速器背隙都沒算進去。

**標準辨識流程**：

1. **參數化**：動力學方程可以改寫成**線性參數形式**：

$$
\tau = Y(q, \dot{q}, \ddot{q}) \cdot \pi
$$

$Y$ 是 **regressor matrix**（由運動狀態組成），$\pi$ 是要辨識的參數向量（質量、慣量、摩擦係數等）。

2. **激勵軌跡設計**：讓機器人跑特定軌跡（通常是 Fourier series 疊加），讓 regressor matrix 的 condition number 最小化，確保所有參數都被充分激勵。

3. **數據收集**：記錄 $(q, \dot{q}, \ddot{q}, \tau)$ — 其中 $\ddot{q}$ 通常由 $\dot{q}$ 數值微分 + 低通濾波得到。

4. **最小二乘求解**：

$$
\hat{\pi} = (Y^T Y)^{-1} Y^T \tau_{\text{measured}}
$$

實務用 Weighted Least Squares (WLS) 或 Total Least Squares (TLS)，因為 $Y$ 和 $\tau$ 都有量測雜訊。

5. **驗證**：用新的軌跡跑一遍，比較預測力矩和實測力矩的 RMSE。

**工具**：Pinocchio 有 `computeJointTorqueRegressor()` 直接給 $Y$；也可用 `robot_identification` ROS 2 package。

</details>

## 常見誤解

1. **「慣性矩陣 $M(q)$ 是常數」** — 錯。$M(q)$ 隨姿態變化。手臂伸直時某些元素可能是彎曲時的好幾倍。這意味著同樣的力矩在不同姿態下產生的加速度完全不同。用 PID 調好一個姿態，換個姿態可能就振盪或反應不足。**正確理解**：$M(q)$ 每個控制週期都要重算（或用前饋補償掉）。

2. **「PID 夠用，不需要動力學模型」** — 在低速、輕負載時 PID 確實能湊合。但高速運動時科氏力和離心力項變得顯著，純 PID 沒有前饋，全靠誤差驅動，追蹤誤差會隨速度增大而急劇惡化。**正確理解**：工業機器人標準做法是 inverse dynamics 前饋 + PD 回饋（CTC），前饋抵消 ~90% 非線性項，PD 只修殘差。

3. **「Lagrangian 方法太慢所以沒用」** — Lagrangian 確實不適合即時控制（O(n$^3$) 以上），但它的價值在離線分析：用 SymPy 推出完整的 $M, C, g$ 矩陣後做代碼生成、控制理論穩定性證明、System Identification 的 regressor 建構。**正確理解**：Newton-Euler 線上跑、Lagrangian 離線推，兩者互補。

4. **「摩擦用線性模型就夠了」** — 簡單的庫侖摩擦 + 黏滯摩擦模型忽略了 Stribeck 效應（低速時摩擦力反而隨速度增加）、靜摩擦-動摩擦不連續跳變。在精密操作或力控場景，摩擦建模不準會直接導致 stick-slip 震盪。**正確做法**：至少用 Stribeck 摩擦模型，或加 Disturbance Observer (DOB) 線上補償未建模的摩擦。

## 練習題

<details>
<summary>Q1：你用 computed torque control 控制一支 7-DoF 機械臂做高速 pick-and-place，發現追蹤誤差比預期大 3 倍，而且越快越差。你會怎麼診斷？</summary>

**完整推理鏈**：

1. **確認是前饋不準而非 PD 調太軟**：暫時降低速度到 10%，看誤差是否回到可接受範圍。如果慢速 OK 快速不行 → 問題出在與速度/加速度相關的項（$C\dot{q}$ 或 $M\ddot{q}$），不是靜態增益
2. **檢查動力學參數**：CAD 給的質量、慣量可能跟實機差很多（線纜、減速器、末端工具都有質量）。用 `pinocchio::rnea` 算出的預測力矩和實測電流比較 — 如果 RMSE > 20% → 參數不準
3. **做 System Identification**：設計激勵軌跡（Fourier series），收集 $(q, \dot{q}, \ddot{q}, \tau)$，用 WLS 辨識動力學參數，更新 URDF 的慣性屬性
4. **檢查摩擦模型**：如果辨識後靜態力矩匹配改善但高速仍偏差 → 非線性摩擦（特別是 harmonic drive 的 wave generator 摩擦）沒建模。加入 Stribeck 摩擦項或上 DOB
5. **避開陷阱**：不要急著加大 PD 增益 — 增益太高在參數不準時會激發高頻振盪。正確做法是先把前饋做準，再微調 PD

**結論**：問題根源幾乎都是動力學參數不準。標準解法是 System ID + 摩擦補償，讓前饋盡可能準確。

</details>

<details>
<summary>Q2：你在做 sim-to-real transfer，RL policy 在 MuJoCo 訓練完美，部署到真機力矩指令完全不對，機器人亂抖。怎麼排查？</summary>

**完整推理鏈**：

1. **先查最簡單的：重力補償**：讓真機靜止在某姿態，讀取馬達實際出力。和 MuJoCo 裡 `mj_inverse` 在同姿態算的重力力矩比較。如果差異 > 15% → 質量/慣量參數有問題
2. **查摩擦**：MuJoCo 預設的摩擦模型很簡化，真機 harmonic drive 減速器的摩擦是高度非線性的。讓真機慢速來回運動，量測力矩-速度曲線，看有沒有 Stribeck 效應
3. **查執行器延遲**：MuJoCo 裡力矩是即時生效的，但真實馬達驅動器有 communication latency + 電流環響應延遲。1-2 ms 延遲在 1 kHz 控制下會嚴重影響穩定性
4. **解法三管齊下**：
   - **Domain Randomization**：訓練時對質量 ±10%、摩擦 ±30%、延遲 0-3 ms 隨機擾動
   - **System ID**：用真機數據辨識動力學參數，更新 MuJoCo XML 的 `<body mass>` 和 `<joint damping>`
   - **加觀測延遲**：在 sim 裡模擬 1-2 步的觀測延遲
5. **避開陷阱**：不要只做 Domain Randomization 而跳過 System ID。DR 讓 policy robust，但如果 nominal model 差太遠（比如重力差 50%），DR 的隨機範圍蓋不住

**結論**：按「重力 → 摩擦 → 延遲」的順序排查，System ID 校準 nominal model + Domain Randomization 增強魯棒性。

</details>

<details>
<summary>Q3：你需要在 1 kHz 控制迴圈裡跑阻抗控制的 inverse dynamics，目標硬體是 ARM Cortex 嵌入式處理器，計算資源有限。怎麼保證即時性？</summary>

**完整推理鏈**：

1. **選 RNEA 不選 Lagrangian**：Newton-Euler 遞迴是 O(n)，7-DoF 機器人大約 ~200 次浮點乘加。Lagrangian 要算完整 $M(q)$ 是 O(n$^2$) ~ O(n$^3$)，嵌入式跑不動
2. **用 Pinocchio 或 CasADi 做代碼生成**：離線用 Pinocchio 的 `computeMinverse` 配 CasADi 的自動微分，生成純 C 代碼（無動態記憶體分配、無虛函數呼叫）。生成的代碼就是一堆 `double c1 = cos(q[0]); double s1 = sin(q[0]); ...` 的扁平賦值
3. **三角函數快取**：$\cos(q_i)$ 和 $\sin(q_i)$ 只算一次存起來重用。7 個關節只需 14 次三角函數呼叫（用 CORDIC 或查表替代 `libm`）
4. **記憶體策略**：所有中間變數用 stack-allocated fixed-size array，絕不 `malloc`。RNEA 的中間結果（每個連桿的 $\omega, \dot{\omega}, f, n$）用 `std::array<Vector3d, N_JOINTS>` 固定大小
5. **驗證即時性**：用 `clock_gettime(CLOCK_MONOTONIC)` 量測 worst-case execution time，確保 < 500 μs（留一半給通訊和其他任務）

**結論**：RNEA O(n) + 離線代碼生成 + zero-alloc 記憶體策略，是嵌入式即時動力學的標準做法。

</details>

<details>
<summary>Q4：面試官問你「如果動力學模型的 condition number 很差，會有什麼問題？怎麼處理？」</summary>

**完整推理鏈**：

1. **什麼是 condition number**：慣性矩陣 $M(q)$ 的 condition number $\kappa = \sigma_{\max} / \sigma_{\min}$（最大奇異值除以最小奇異值）。$\kappa$ 越大，矩陣越接近奇異
2. **什麼情況 $\kappa$ 會大**：機器人接近某些姿態時（例如手臂完全伸直，某些關節慣性差距懸殊），$M(q)$ 的特徵值比值可能達 100:1 或更高
3. **造成的問題**：
   - Forward dynamics 算 $\ddot{q} = M^{-1}(...)$ 時，數值誤差被放大 $\kappa$ 倍。浮點精度有限時，計算出的加速度可能有幾個量級的誤差
   - 控制器的前饋力矩也會不準，導致某些關節過補償、某些欠補償
4. **處理方法**：
   - **避免用顯式矩陣求逆** $M^{-1}$：改用 Cholesky 分解（$M$ 正定，保證成功）求解線性系統 $M \ddot{q} = b$
   - **用 ABA 算法**：Pinocchio 的 `aba()` 直接算 forward dynamics 不經過 $M^{-1}$，數值穩定性更好
   - **軌跡規劃避開病態姿態**：在路徑規劃時加入 $\kappa(M(q))$ 作為代價項，讓機器人不要經過 condition number 極大的姿態

**結論**：condition number 影響數值穩定性，用 Cholesky 或 ABA 而非顯式求逆，且規劃時避開病態姿態。

</details>

## 面試角度

1. **前饋 + 回饋解耦是動力學控制的核心架構** — 這是區分「會調 PID」和「懂控制」的分水嶺。面試時帶出：「我做控制不會只靠 PID 硬扛，標準做法是 inverse dynamics 前饋抵消 90% 非線性項（慣性、科氏、重力），PD 只修殘餘誤差。這把非線性系統變成線性二階系統，調參和穩定性分析都簡單得多。」

2. **RNEA O(n) 是硬即時的關鍵** — 展現你不只懂理論還能落地到嵌入式。面試時帶出：「即時控制我用 Newton-Euler 遞迴而不是 Lagrangian，因為 RNEA 是 O(n) 而 Lagrangian 是 O(n$^3$)。配上離線代碼生成和 zero-alloc 策略，7-DoF 的 inverse dynamics 在 ARM Cortex 上跑不到 100 μs。」

3. **DOB / 自適應控制抗未建模干擾** — 展現你知道理想模型和真機之間的 gap 怎麼補。面試時帶出：「動力學模型再準也有未建模項（非線性摩擦、線纜彈性、負載變化），我會加 Disturbance Observer 線上估計並補償干擾力矩，或用自適應控制線上修正參數。」

4. **$\dot{M} - 2C$ 斜對稱性是穩定性證明的基石** — 展現控制理論深度。面試時帶出：「Lyapunov 穩定性分析時，$\dot{M} - 2C$ 的斜對稱性保證了被動性，讓我可以設計一類能量型控制器（如 PD + 重力補償）並嚴格證明全域漸近穩定。」

5. **System ID 是 sim-to-real 的基本功** — 展現實戰經驗。面試時帶出：「CAD 參數永遠不夠準，我會設計激勵軌跡做 System Identification，用 WLS 辨識動力學參數，再用新軌跡驗證預測力矩 RMSE。這是縮小 sim-to-real gap 的第一步，比直接上 Domain Randomization 更有底氣。」

## 延伸閱讀

- **Roy Featherstone,《Rigid Body Dynamics Algorithms》** — Spatial algebra + RNEA/ABA 的聖經級參考。想真正理解為什麼 O(n) 做得到、forward dynamics 怎麼不用求逆，讀這本
- **Lynch & Park,《Modern Robotics》Ch8（Dynamics of Open Chains）** — 免費教材，用 screw theory 統一推導 Newton-Euler 和 Lagrangian，配有 Python 範例和影片
- **《具身智能算法工程師 面試題》Ch1.4 動力學建模** — 對標面試考點的精練題目集，特別是 CTC、System ID、摩擦建模
- **Pinocchio 官方文檔與 Tutorial** — 業界最快的開源 rigid body dynamics 庫，讀 source code 學 spatial algebra 實作，MPC 和 RL 項目的標準依賴
- **MuJoCo 官方文檔：Computation 章節** — 理解模擬器內部如何做 forward dynamics（constraint-based + ABA 混合），對調 sim 參數和 debug sim-to-real 問題必讀
- **論文：Swevers et al.,《Optimal Robot Excitation and Identification》** — System ID 激勵軌跡設計的經典方法論，想做精密力控必讀
