---
title: "軌跡最佳化（時間、能量、平滑度）"
prerequisites: ["10-basic-path-planning"]
estimated_time: 75
difficulty: 4
tags: ["trajectory", "optimization", "smoothness", "b-spline", "topp", "min-snap", "trajopt", "ilqr", "contact-implicit"]
sidebar_position: 11
---

# 軌跡最佳化（時間、能量、平滑度）

## 你將學到

- 能用兩句話區分 path 與 trajectory，面試時不混用
- 遇到「機械臂搬運太慢」或「焊接末端抖動」時，知道該用什麼目標函數（時間最優 / 能量最優 / 平滑度最優）、什麼參數化方法（多項式 / B-spline / S 曲線），以及為什麼三者不能同時極致
- 判斷何時需要 TOPP-RA 做時間最優重參數化，何時該用 minimum snap 做平滑軌跡
- **(A-level 擴展)** 能在面試 2 分鐘內講清 Differential Flatness 為什麼是無人機 2011 年後的底層靈魂、CHOMP/STOMP/TrajOpt 三框架的本質差異、iLQR/DDP/Shooting/Collocation 的 trade-off 決策樹、以及 Atlas 後空翻為什麼只能用 Direct Collocation + Contact-Implicit

## 核心概念

**精確定義**：**Path** 是純幾何路線（一組有序 waypoints 或連續曲線），不含時間資訊；**Trajectory** 是 path 加上時間參數化 $q(t)$，產出每個時刻的位置 $q(t)$、速度 $\dot{q}(t)$、加速度 $\ddot{q}(t)$，且滿足物理約束（速度極限、力矩極限、jerk 極限）。**軌跡最佳化**就是在給定 path 或 waypoints 上，找出最佳的時間分配與曲線形狀，使某個目標函數最小化。

**一句話版本**：「Path 是地圖上畫的路線；trajectory 是決定哪裡踩油門、哪裡煞車、而且不讓乘客暈車的完整行車計畫。」

**在感知 → 規劃 → 控制閉環的位置**：
- **節點**：**規劃 → 控制**的橋梁
- **輸入**：path planner 給的 waypoints + 機器人動力學限制（$\dot{q}_{\max}$, $\ddot{q}_{\max}$, $\tau_{\max}$）
- **輸出**：時間參數化的 $q(t), \dot{q}(t), \ddot{q}(t)$（關節空間）或 $x(t), \dot{x}(t), \ddot{x}(t)$（笛卡爾空間）
- **下游**：controller 的 feedforward 項（PD + feedforward 需要 $\ddot{q}_{\text{desired}}$）、MPC 的 reference trajectory、力矩前饋需要 $\tau_{\text{ff}} = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q)$

**三大目標函數**：

| 目標 | 數學形式 | 物理意義 | 典型應用 |
|------|---------|---------|---------|
| 時間最優 | $\min \int_0^T dt = \min\, T$ | 最短時間完成動作 | 高產量 pick-and-place |
| 能量最優 | $\min \int_0^T \|\tau\|^2 dt$ | 最小化力矩平方積分，降馬達發熱 | 長時間連續作業 |
| 平滑度最優 | $\min \int_0^T \|\dddot{q}\|^2 dt$ (min jerk) | 最小化 jerk，抑制振動與高頻激發 | 精密加工、焊接 |

**多項式階數決策表**（A-level 核心）：

| 階數 | 連續性 | 適用場景 | 致命缺陷 |
|------|-------|---------|---------|
| 一次 Linear | $C^0$ | 永遠不要用 | 加速度是 Dirac δ → 瞬間無窮扭矩擊穿電流 |
| 三次 Cubic | $C^1$（速度連續） | 教學示範 | 加速度端點跳變 → jerk 無窮 → 微小震顫 |
| **五次 Quintic** | **$C^2$（加速度連續）** | **工業 PTP 黃金標準** | 無重大缺陷 |
| 七次 Septic | $C^3$（jerk 連續） | CNC 雷射切割、高精度加工 | 參數多、計算量大 |
| LSPB（梯形+拋物線） | $C^1$ | 低算力 PLC、機床簡單動作 | 本質只保證速度連續 |
| 高階（9+次）全域 | $C^{k}$ | **永遠不要用** | **Runge 現象**：10 個 via-point 用 9 次多項式 → 邊緣劇烈震盪 → 機器人瘋狂扭動 |

**Runge 現象陷阱**（面試必答）：不要把 10 個 via-points 強塞給一個 9 次多項式。高次多項式在邊緣會劇烈震盪，機器人會瘋狂扭動。**正解**：用 B-Spline 分段低次（通常 5 次）多項式插值，在拼接點強加 $C^2$ 連續性約束。

**最少夠用的數學**：

1. **三次多項式插值**（滿足端點 position + velocity 邊界條件）：

$$
q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3
$$

**物理意義**：4 個係數剛好解 4 個邊界條件（起點位置/速度、終點位置/速度），是最簡單的平滑軌跡。但加速度 $\ddot{q} = 2a_2 + 6a_3 t$ 只是線性變化，在軌跡段切換點會產生 jerk 階躍 → 微小震顫。

2. **五次多項式**（再加 acceleration 邊界條件）：

$$
q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5
$$

**物理意義**：6 個係數解 6 個邊界條件（起點和終點各有 position、velocity、acceleration）。加速度連續 → 力矩連續 → 機械衝擊更小。**重要結論**：jerk-minimization 的解析解等價於 quintic，這就是為什麼工業 PTP 黃金標準是 5 次而不是 3 次。

3. **Minimum Snap 四旋翼標準**（Mellinger-Kumar 2011，無人機基石）：

$$
\min_{c} \int_0^T \left\| p^{(4)}(t) \right\|^2 dt = \min_{c} \; c^\top Q c \quad \text{s.t.} \quad A_{\text{eq}} c = b_{\text{eq}}
$$

**物理意義**：$p^{(4)}$（snap，四階微分）直接對應**馬達控制扭矩變化率**。四旋翼位置 $p$ → 加速度 $\ddot{p}$（螺旋槳推力 + 機身傾角）→ jerk $\dddot{p}$（角速度）→ snap $p^{(4)}$（馬達扭矩變化率）。**Min-snap = Min 馬達推力劇烈波動** → 高速穿窗框時姿態絲滑不失控。

4. **TOPP-RA 時間最優重參數化**（核心思想）：

$$
s(t): [0, T] \to [0, 1], \quad \dot{s} \geq 0
$$

$$
\text{subject to: } \tau_{\min} \leq M(q(s))\ddot{q}(s) + C(q,\dot{q})\dot{q}(s) + g(q(s)) \leq \tau_{\max}
$$

**物理意義**：把幾何路徑用弧長 $s$ 重新參數化，然後在 $(s, \dot{s})$ 相平面上找最快的速度曲線，使得每個瞬間的力矩都不超限。TOPP-RA 的關鍵變數代換 $u = \ddot{s}$, $v = \dot{s}^2$ 把非線性動力學**線性化**為 $\tau(s) = A(s) u + B(s) v + C(s)$，轉成 LP 序列 $O(N)$ 複雜度 1ms 內解完。

<details>
<summary>深入：TrajOpt 三框架對比（CHOMP / STOMP / TrajOpt SQP）</summary>

### CHOMP (Covariant Hamiltonian Optimization)

把軌跡離散化為 waypoints 向量 $\xi \in \mathbb{R}^{N \times n}$，梯度下降：

$$
\xi_{i+1} = \xi_i - \eta \cdot A^{-1} \nabla U(\xi_i)
$$

- **目標函數** $U = U_{\text{smooth}} + U_{\text{obstacle}}$
- **"Covariant" 關鍵**：$A^{-1}$ 是基於軌跡差分的黎曼度量矩陣，把能量平滑散佈整條軌跡，**不受參數化點密度影響**
- **避障**：SDF（signed distance field）梯度推開 waypoints
- **弱點**：避障是**軟約束** → 可能擦撞；梯度需可微 → 碰不了 non-smooth cost

### STOMP (Stochastic Trajectory Optimization)

**不需計算梯度**：
1. 在參考軌跡周圍加平滑高斯雜訊 → 生成 $K$ 條候選軌跡 $\{\xi^{(k)}\}$
2. 評估每條軌跡的 cost $S^{(k)}$
3. 指數加權（Softmax / MPPI 同源）：$\xi_{\text{new}} = \sum_k \frac{e^{-S^{(k)}/\lambda}}{\sum_j e^{-S^{(j)}/\lambda}} \xi^{(k)}$

**殺手應用**：CHOMP 需 SDF 梯度；對**自碰撞、離散碰撞檢測、避開相機遮擋**這類 non-smooth cost，梯度為 0 或無窮大 → CHOMP 當場廢掉。STOMP 靠隨機採樣完美處理 non-differentiable cost。

### TrajOpt (UC Berkeley, John Schulman)

嚴格 **SQP** 公式化：

$$
\min_{\xi} \; f(\xi) \quad \text{s.t.} \quad \text{dist}(x_i) \geq d_{\text{safe}}, \quad \forall i
$$

**Convexification 凸化**：障礙 SDF 一階泰勒展開

$$
\text{dist}(x_i) \approx \text{dist}(x_{i,0}) + J_{\text{dist}}(x_{i,0}) \cdot \Delta x_i \geq d_{\text{safe}}
$$

→ 線性不等式約束，可信賴域 (Trust Region) 內迭代求解。

**硬約束王者**：只要 SQP 有解，軌跡**絕對無碰撞**；連續時間碰撞檢測（swept volume）避免「時間取樣漏掉細縫穿牆」。

### 場景選型決策樹

| 場景 | 首選 | 為什麼 |
|------|------|-------|
| 擁擠書架取書（狹窄空間硬約束） | **TrajOpt** | SQP 硬約束不穿模 |
| 避相機遮擋視野、避液體潑灑（不可微 cost） | **STOMP** | 採樣暴力處理 non-smooth |
| OMPL 粗軌跡平滑後處理 | **CHOMP** | 共變梯度毫秒內拉平折線 |

### MoveIt! 2 YAML 配置

```yaml
planning_pipelines:
  pipeline_names: [ompl, chomp, stomp]
ompl:
  request_adapters: |
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/CHOMPOptimizingAdapter
```

</details>

<details>
<summary>深入：多項式插值完整推導與矩陣求解（經典推導）</summary>

### 三次多項式

給定邊界條件：$q(0) = q_0$, $\dot{q}(0) = v_0$, $q(T) = q_f$, $\dot{q}(T) = v_f$

從 $q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3$ 得 $\dot{q}(t) = a_1 + 2a_2 t + 3a_3 t^2$

代入邊界條件得線性系統：

$$
\begin{bmatrix} 1 & 0 & 0 & 0 \\ 0 & 1 & 0 & 0 \\ 1 & T & T^2 & T^3 \\ 0 & 1 & 2T & 3T^2 \end{bmatrix}
\begin{bmatrix} a_0 \\ a_1 \\ a_2 \\ a_3 \end{bmatrix}
=
\begin{bmatrix} q_0 \\ v_0 \\ q_f \\ v_f \end{bmatrix}
$$

直接解：$a_0 = q_0$, $a_1 = v_0$

$$
a_2 = \frac{3(q_f - q_0) - (2v_0 + v_f)T}{T^2}
$$

$$
a_3 = \frac{2(q_0 - q_f) + (v_0 + v_f)T}{T^3}
$$

### 五次多項式

多加 $\ddot{q}(0) = a_0^{\text{acc}}$, $\ddot{q}(T) = a_f^{\text{acc}}$ 兩個條件，矩陣擴為 6×6。實務上常設 $a_0^{\text{acc}} = a_f^{\text{acc}} = 0$（起止靜止），得到最常用的「靜止到靜止」五次軌跡。

```python
import numpy as np

def solve_quintic(q0, v0, a0, qf, vf, af, T):
    A = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, T, T**2, T**3, T**4, T**5],
        [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
        [0, 0, 2, 6*T, 12*T**2, 20*T**3],
    ])
    b = np.array([q0, v0, a0, qf, vf, af])
    return np.linalg.solve(A, b)
```

### Minimum jerk 軌跡（閉式解）

當 $v_0 = v_f = a_0 = a_f = 0$（靜止到靜止），minimum jerk 有解析解：

$$
q(t) = q_0 + (q_f - q_0)\left[10\left(\frac{t}{T}\right)^3 - 15\left(\frac{t}{T}\right)^4 + 6\left(\frac{t}{T}\right)^5\right]
$$

這是五次多項式的特例，jerk 積分最小。生物學上人類手臂的自然運動高度吻合這條曲線（Flash & Hogan, 1985）。

**神經科學起源**：Flash-Hogan 1985 年發現，人類手臂伸手抓取優化的**既不是時間最短、也不是能量最小**，而是 **jerk 平方積分最小**。這在生物學上代表最平滑 + 關節磨損最小的自然軌跡。工業機器人借鑒這個發現，把 min-jerk 作為 PTP 運動的黃金目標。

### 多段 waypoint 插值

$n$ 個 waypoint 之間用 $n-1$ 段三次多項式，相鄰段在交接處要求位置、速度、加速度連續，組成 tridiagonal 線性系統，$O(n)$ 可解（Thomas algorithm）。

**警告**：不要用一個 $(n-1)$ 次全域多項式穿過所有 waypoints，會爆發 Runge 現象。

</details>

<details>
<summary>深入：Differential Flatness 與 Min-snap QP 完整推導（無人機）</summary>

### 什麼是 Differential Flatness

四旋翼是**欠驅動系統**（12 狀態 `[x,y,z,ẋ,ẏ,ż,φ,θ,ψ,p,q,r]` 只有 4 馬達輸入）。Mellinger-Kumar 2011 證明：四旋翼是 **Differentially Flat System**，即存在 4 個 flat outputs $\sigma = [x, y, z, \psi]$（3D 位置 + 偏航角），所有 12 狀態和 4 控制輸入都能**以純代數方程完全從 $\sigma$ 及其有限階導數導出**。

### 為什麼這是降維打擊

傳統做法：在 12 維狀態空間做非線性 MPC → 算力爆炸、實時性差。

Diff-Flat 做法：
1. 規劃器**純幾何**在 3D 空間畫一條平滑 $(x(t), y(t), z(t), \psi(t))$ 曲線
2. 這條幾何曲線**自帶動力學可行性**（因為 flatness 保證可從曲線代數算出 thrust + 姿態 + 馬達指令）
3. 無人機絕對飛得出來

這就是 Fast-Planner、Drone Racing、MIT ACL 等無人機軌跡系統的底層靈魂。

### Min-snap QP 問題設定

每段軌跡用 7 次多項式參數化：$p_i(t) = \sum_{k=0}^{7} c_{i,k} t^k$

目標函數（snap 平方積分）：

$$
J = \int_0^T \left\| p^{(4)}(t) \right\|^2 dt = c^\top Q c
$$

其中 Q 矩陣元素：

$$
Q_{ij} = \int_0^T \frac{i!}{(i-4)!} \frac{j!}{(j-4)!} t^{i+j-8} \, dt = \frac{i!\, j!}{(i-4)!(j-4)!} \cdot \frac{T^{i+j-7}}{i+j-7} \quad (i, j \geq 4)
$$

### 約束：waypoint + $C^3$ 連續性

**Waypoint 位置**：$p_i(0) = w_i$, $p_i(T_i) = w_{i+1}$

**相鄰段 $C^3$ 連續**（位置、速度、加速度、jerk 連續）：

$$
p_i^{(k)}(T_i) = p_{i+1}^{(k)}(0), \quad k = 0, 1, 2, 3
$$

組合為 $A_{\text{eq}} c = b_{\text{eq}}$，QP 形式 `min cᵀQc s.t. A_eq c = b_eq` 用 OSQP 或 OOQP 秒解。

### C++ 骨架（構建 Q 矩陣）

```cpp
Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_coeffs, num_coeffs);
for (int i = 4; i < num_coeffs; ++i) {     // snap 從 4 階開始
    for (int j = 4; j < num_coeffs; ++j) {
        double cost_coeff = (factorial(i)/factorial(i-4)) *
                            (factorial(j)/factorial(j-4));
        Q(i, j) = cost_coeff * std::pow(T, i+j-7) / (i+j-7);
    }
}
// 呼叫 OSQP 求解 min cᵀQc s.t. A_eq c = b_eq
```

### 時間分配最佳化

$T_i$（每段時間）也是決策變數。實務做法：
- **初始猜測**：trapezoidal velocity profile 粗估
- **外層迭代**：固定 $c$ 更新 $T_i$（NLP），固定 $T_i$ 解 QP（Richter et al. 2016）

</details>

<details>
<summary>深入：TrajOpt 三框架對比（CHOMP / STOMP / TrajOpt SQP）</summary>

### CHOMP (Covariant Hamiltonian Optimization)

把軌跡離散化為 waypoints 向量 $\xi \in \mathbb{R}^{N \times n}$，梯度下降：

$$
\xi_{i+1} = \xi_i - \eta \cdot A^{-1} \nabla U(\xi_i)
$$

- **目標函數** $U = U_{\text{smooth}} + U_{\text{obstacle}}$
- **"Covariant" 關鍵**：$A^{-1}$ 是基於軌跡差分的黎曼度量矩陣，把能量平滑散佈整條軌跡，**不受參數化點密度影響**
- **避障**：SDF（signed distance field）梯度推開 waypoints
- **弱點**：避障是**軟約束** → 可能擦撞；梯度需可微 → 碰不了 non-smooth cost

### STOMP (Stochastic Trajectory Optimization)

**不需計算梯度**：
1. 在參考軌跡周圍加平滑高斯雜訊 → 生成 $K$ 條候選軌跡 $\{\xi^{(k)}\}$
2. 評估每條軌跡的 cost $S^{(k)}$
3. 指數加權（與 Softmax / MPPI 同源）：$\xi_{\text{new}} = \sum_k \frac{e^{-S^{(k)}/\lambda}}{\sum_j e^{-S^{(j)}/\lambda}} \xi^{(k)}$

**殺手應用**：CHOMP 需 SDF 梯度；對**自碰撞、離散碰撞檢測、避開相機遮擋**這類 non-smooth cost，梯度為 0 或無窮大 → CHOMP 當場廢掉。STOMP 靠隨機採樣完美處理 non-differentiable cost。

### TrajOpt (UC Berkeley, John Schulman)

嚴格 **SQP** 公式化，含 **Convexification 凸化**：障礙 SDF 一階泰勒展開

$$
\text{dist}(x_i) \approx \text{dist}(x_{i,0}) + J_{\text{dist}}(x_{i,0}) \cdot \Delta x_i \geq d_{\text{safe}}
$$

→ 線性不等式約束，可信賴域 (Trust Region) 內迭代求解。

**硬約束王者**：只要 SQP 有解，軌跡**絕對無碰撞**；連續時間碰撞檢測（swept volume）避免「時間取樣漏掉細縫穿牆」。

### 場景選型決策樹

| 場景 | 首選 | 為什麼 |
|------|------|-------|
| 擁擠書架取書（狹窄空間硬約束） | **TrajOpt** | SQP 硬約束不穿模 |
| 避相機遮擋視野、避液體潑灑（不可微 cost） | **STOMP** | 採樣暴力處理 non-smooth |
| OMPL 粗軌跡平滑後處理 | **CHOMP** | 共變梯度毫秒內拉平折線 |

### MoveIt! 2 YAML 配置

```yaml
planning_pipelines:
  pipeline_names: [ompl, chomp, stomp]
ompl:
  request_adapters: |
    default_planner_request_adapters/AddTimeOptimalParameterization
    default_planner_request_adapters/CHOMPOptimizingAdapter
```

</details>

## 中階概念：Time-Optimal、Contact-Aware 與學習式生成

### TOPP-RA 為什麼是工業標準

傳統 Single-shot（把 $q$ 和 $t$ 同時當決策變數）是**非凸問題** → 求解慢 + 易發散。工業標準是**解耦**：先用 RRT\*/CHOMP 找幾何路徑 $q(s)$，再用 TOPP-RA 算最優 $\dot{s}, \ddot{s}$。

**Bang-Bang 原則**（龐特里亞金極小值原理，面試必答）：時間最優解**必然是 Bang-Bang 的**——每一瞬間至少有一個關節力矩達到物理極限。直觀理解：「滿載加速 → 滿載勻速 → 滿載減速」，完美榨乾硬體潛力。

**Min-snap + TOPP-RA = 降維打擊組合**：
- Min-snap 保證幾何**極致平滑**（馬達推力變化率最小）
- TOPP-RA 在這條平滑路徑上**精確壓榨馬達扭矩潛力**到時間最優
- 凸 + 快 + 工程可控 → 工業界黃金 pipeline

### Contact-Aware Trajectory Optimization（Manipulation + Locomotion 統一）

這是現代具身智能最重要的統一觀念——**推箱子、轉手、拉抽屜、Atlas 後空翻，在數學上是同一個問題**。

**CITO（Contact-Implicit TrajOpt）核心數學**：

$$
\begin{aligned}
\min_{q, \dot{q}, u, \lambda} \;& J(q, u) \\
\text{s.t.} \;& M(q)\ddot{q} + h(q, \dot{q}) = B u + J_c^\top \lambda \\
& \phi(q) \geq 0, \quad \lambda \geq 0, \quad \phi(q) \cdot \lambda = 0 \quad \text{(complementarity)}
\end{aligned}
$$

**物理意義**：
- $\phi(q) \geq 0$：不穿透約束（手指 / 足不穿入物體）
- $\lambda \geq 0$：接觸力只能推不能拉
- $\phi \cdot \lambda = 0$：**互補約束** — 要麼離開（$\phi > 0, \lambda = 0$），要麼接觸（$\phi = 0, \lambda > 0$）
- **KKT 拉格朗日乘子 $\lambda$ 物理湧現為接觸力** — 求解器自己決定何時換接觸模式

**為什麼這是統一視角**：機械臂推箱子和 Atlas 後空翻都是 **underactuated dynamics** 問題——無法直接控制箱子或空中質心，只能**尋找最優接觸點 + 接觸力 $J^\top f_c$ 間接改變狀態**。掌握 Contact-Implicit → 足式 + 靈巧手兩大領域同時打通。

<details>
<summary>深入：TOPP-RA 變數代換與向前/向後掃描完整推導</summary>

### 核心數學魔法（面試必背）

原始動力學 $\tau = M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q)$ 是 $q$ 和 $\ddot{q}$ 的**非線性**方程。

沿路徑 $q(s)$ 重參數化：
- $\dot{q} = q'(s) \cdot \dot{s}$
- $\ddot{q} = q''(s) \cdot \dot{s}^2 + q'(s) \cdot \ddot{s}$

**關鍵代換**：令 $u = \ddot{s}$，$v = \dot{s}^2$（注意 $\dot{v} = 2\dot{s}\ddot{s} = 2\dot{s} u$，所以 $u = \frac{1}{2}\frac{dv}{ds}$）

代入動力學：

$$
\tau(s) = \underbrace{M(q(s)) q'(s)}_{A(s)} u + \underbrace{\left[M(q(s))q''(s) + C q'(s)\right]}_{B(s)} v + \underbrace{G(q(s))}_{C(s)}
$$

**原非線性動力學變成 $u, v$ 的線性方程！** 力矩約束 $\tau_{\min} \leq \tau \leq \tau_{\max}$ 變成 $u, v$ 的線性不等式。

### TOPP-RA 演算法流程（Pham 2018）

1. 軌跡切 $N$ 段 $\{s_0, s_1, \ldots, s_N\}$
2. **向後掃描**（backward pass）：從 $s_N$ 反推每點的 reachable 集合 $[v_{\min}^{\text{reach}}(s_i), v_{\max}^{\text{reach}}(s_i)]$
3. **向前掃描**（forward pass）：從 $s_0$ 正推，每點取 reachable 集合中**最大的 $v$**
4. 每步解極小規模 LP（typically 2 個變數），$O(N)$ 極速求解，**1ms 內**

對比原版 TOPP（ODE 積分 + switch point 搜尋）：
- 原 TOPP 數值不穩定，switch point 搜尋容易漏
- TOPP-RA 基於 reachability analysis，保證全域最優 + 數值穩健

### Drake TOPP-RA Python API

```python
from pydrake.all import Toppra

toppra = Toppra(geometric_path, plant)
toppra.AddJointVelocityLimit(v_min, v_max)
toppra.AddJointAccelerationLimit(a_min, a_max)
toppra.AddJointTorqueLimit(tau_min, tau_max)
time_optimal_traj = toppra.SolvePathParameterization()  # 1ms 內
```

### 為什麼 Min-snap + TOPP-RA 是完美組合

- Min-snap 負責**純幾何平滑**（snap 最小 → 馬達扭矩變化率最小 → 姿態絲滑）
- TOPP-RA 負責**時間壓榨**（Bang-Bang 打滿扭矩極限）
- 兩者解耦 → 雙凸問題，可分別獨立驗證

**vs Single-shot 的碾壓優勢**：Single-shot 非凸無即時保障，容易卡在局部解；Min-snap + TOPP-RA 兩階段都是凸問題，工程確定性極高。

</details>

<details>
<summary>深入：Contact-Implicit TrajOpt 與 MPCC 鬆弛（Atlas 後空翻級別）</summary>

### 互補約束為什麼難

$\phi(q) \cdot \lambda = 0$ 在原點不可微（梯度不存在），標準 NLP（IPOPT, SNOPT）直接卡死。

### MPCC (Mathematical Program with Complementarity Constraints) 鬆弛

引入小鬆弛常數 $\epsilon$：

$$
\phi(q) \cdot \lambda \leq \epsilon
$$

逐步減小 $\epsilon$（外層迭代 $\epsilon_k = 0.5 \epsilon_{k-1}$），IPOPT 可解。Drake 與 Casadi 已內建此框架。

```python
prog.AddConstraint(phi >= 0)
prog.AddConstraint(lam >= 0)
prog.AddConstraint(phi * lam <= 1e-4)  # MPCC 鬆弛利於 IPOPT
```

### 四足 Footstep + Swing Trajectory

**Footstep 位置**：倒立擺 (LIPM) 或 Capture Point 理論保質心穩定。

**Swing 軌跡為什麼用 Bezier 曲線**：
- **凸包性 (Convex Hull Property)**：整條 Bezier 曲線必被控制點的凸多邊形包裹
- 只要約束中間控制點**高於地面障礙高度** $h_{\text{clearance}}$ → 整條擺動腿軌跡**絕不絆倒**
- 避障約束簡化為**幾個控制點的線性不等式** → QP 秒解
- 對比自由多項式：需要檢查整條曲線的每個時刻，計算爆炸

### Centroidal Momentum-aware Optimization (CMM, Atlas 後空翻級別)

Atlas 有 30+ 關節，直接在全狀態空間規劃會爆炸。**CMM 降維到質心 6D 動量空間**：

$$
\dot{h}_G = \begin{bmatrix} m \ddot{c} \\ \dot{k}_G \end{bmatrix} = \begin{bmatrix} \sum f_i - mg \\ \sum (p_i - c) \times f_i + \tau_i \end{bmatrix}
$$

- $c$：質心位置
- $k_G$：角動量
- $f_i, p_i$：每個接觸點的力 + 位置

**只規劃 CoM 軌跡 $p_c$ 和角動量 $k$ → 約束 $\dot{k} = \sum (p_i - p_c) \times f_i$ → 精確算出足底 GRF 產生後空翻旋轉力矩**。這就是 Boston Dynamics Atlas 後空翻的核心數學。

</details>

<details>
<summary>深入：Learning-based Trajectory Generation（Diffusion + Transformer + ALOHA ACT）</summary>

### Motion Planning Diffusion (MPD)

把軌跡當「圖片去噪」問題：
1. 初始化高斯雜訊軌跡 $\xi_T \sim \mathcal{N}(0, I)$
2. **Conditional Diffusion**：起終點 / LiDAR 點雲 / 自然語言（"繞過杯子"）作 condition 注 UNet
3. 迭代去噪 $\xi_{t-1} = \xi_t - \epsilon_\theta(\xi_t, t, c) + \sigma_t z$
4. 最終坍縮出平滑軌跡

### Guided Diffusion with Constraints（物理可行性補丁）

單純擴散可能穿模。每步去噪額外加**物理 cost 梯度**：

$$
\hat{\epsilon} = \epsilon_\theta(x_t, \varnothing) + w \cdot \left[\epsilon_\theta(x_t, c) - \epsilon_\theta(x_t, \varnothing)\right] + \lambda \nabla_x C(x_t)
$$

- $C(x) = $ SDF 碰撞懲罰 + 動力學約束
- $\nabla_x C$ 推離障礙，確保物理可行

### Trajectory / Decision Transformer

(s, a, R) 量化成 Token：`[R_1, s_1, a_1, R_2, s_2, a_2, ...]`

GPT 式 causal attention 預測下一 token。**推理時輸入高回報目標** $R_{\text{target}} = 1$ → 自迴歸吐出最優動作軌跡。本質是「把 RL 問題 offline 轉成序列建模問題」。

### ALOHA ACT (Action Chunking + Temporal Ensemble)

靈巧操作 workhorse。遙操作數據充滿手部抖動停頓（高頻雜訊），傳統單步 BC 會把微誤差幾何放大。

**ACT 做法**：
1. 一次生成未來 $k$ 步 chunk $\{a_t, a_{t+1}, \ldots, a_{t+k-1}\}$
2. 每步對**所有重疊片段**指數加權移動平均 (Temporal Ensemble)
3. 時間維度低通濾波 → 極平滑

**為什麼穿針、打雞蛋等精細操作唯一解**：小尺度精密動作要求高時間連續性，Temporal Ensemble 在時域平滑抖動，比直接 smooth action 更優（因為保留了 chunk 內部的動態模式）。

```python
all_predictions = np.zeros((T, T, action_dim))
for t in range(T):
    chunk = act_policy(obs)  # 一次 k 步
    all_predictions[t, t:t+k] = chunk
    valid = all_predictions[:, t, :]  # 過去所有對 t 的預測
    weights = np.exp(-np.arange(len(valid)) / decay)
    smooth_action_t = np.sum(valid * weights / weights.sum(), axis=0)
```

### VLA（RT-2 / OpenVLA / π0）Trajectory Head

VLA 不輸出單步 action，直接輸出**未來 1-2 秒連續空間 Waypoints**，接 Action Chunking Head（MDN 或 Diffusion Head）。

### 「Symbol → Neural」現代架構哲學

不再碰撞檢測盲搜 → **Diffusion Model「想像」專家軌跡 → MPC / 阻抗控制器消化殘差**。解決的不是精度問題，是**「開放世界人類直覺泛化」**。

</details>

## 進階概念：iLQR/DDP、Multi-phase 與 Rollout 橋接

### iLQR vs DDP 的 Hessian 秘密

**iLQR (Iterative LQR)** Bellman 反向遞推：

$$
V(x_k) = \min_u \left[\ell(x, u) + V_{k+1}(f(x, u))\right]
$$

在標稱軌跡 $(\bar{x}_k, \bar{u}_k)$ 上**線性化 + 代價二次化**，計算 Q-function 二次展開 $(Q_x, Q_u, Q_{xx}, Q_{uu}, Q_{ux})$，解出最優控制律：

$$
\delta u_k = k_k + K_k \cdot \delta x_k \quad \text{(前饋 + 反饋)}
$$

**DDP (Differential Dynamic Programming)** 比 iLQR **多保留動力學二階 Hessian 張量** $\partial^2 f / \partial x^2$：
- iLQR：高斯-牛頓近似，只用 Jacobian $(f_x, f_u)$
- DDP：二次收斂，感知世界「曲率」（旋轉向心力等非線性）
- **代價**：二階張量計算耗時巨大
- **實務結論**：iLQR **性價比最高**，絕大多數場景用 iLQR

### Shooting vs Multiple Shooting vs Direct Collocation

| 方法 | 決策變數 | 動力學處理 | 適用 |
|------|---------|-----------|------|
| **Single Shooting** | $u_{0:T}$ | 靠物理引擎硬積分 | 短時程、弱非線性 |
| **Multiple Shooting** | $x_{0:T}, u_{0:T-1}$ + gap closing 約束 | 分段積分 + 連續性等式 | 中等時程、數值穩定需求 |
| **Direct Collocation** | $x_{0:T}, u_{0:T}$ 全時點 | 相鄰節點代數等式（Hermite-Simpson） | **Atlas 後空翻**、長時程、強非線性 |

**為什麼 Atlas 後空翻必用 Collocation**（面試必答）：
- Single Shooting：求解器迭代初期給錯一點力矩 → $x(t)$ 積分到半空炸飛 → 梯度發散找不到
- **Collocation 允許求解器迭代初期違反物理定律**（Defect $\neq 0$）：先讓機器人「飄」在空中完成翻轉幾何，慢慢收緊動力學約束
- 「同時在幾何與物理空間尋優」的穩定性遠超 Shooting

**場景選型決策樹**：
- **微秒級在線 MPC + 簡單非凸** → iLQR（不依賴 NLP solver，自寫 C++ 矩陣乘 1kHz）
- **Atlas 後空翻等離線大尺度跨相態極限動作** → Direct Collocation + IPOPT（慢但非線性約束探索強 + 數值穩定）
- **機械臂抓取、四足步態** → Multiple Shooting + Casadi + IPOPT（平衡點）

### Multi-phase / Hybrid Trajectory Optimization

Phase 切換軌跡：**P₁ 雙腳支撐 → P₂ 騰空 → P₃ 單腳落地**

每個 Phase 動力學不同：
- P₁：受 GRF 驅動（$\ddot{x} = f(x, u, \text{GRF})$）
- P₂：GRF = 0，退化為重力拋體 + 旋轉剛體
- P₃：另一隻腳 GRF

**相鄰 Phase 必須加狀態連續性約束** $x(P_1, \text{end}) = x(P_2, \text{start})$。

**騰空相角動量守恆**（花式滑冰選手原理）：
Flight phase 無外力矩 → 牛頓-歐拉 $\dot{k}_{\text{com}} = \sum r \times F_{\text{grf}} = 0$

$$
k_{\text{com}}(t) = k_{\text{com}}(t_0), \quad t \in [\text{起跳}, \text{落地}]
$$

**Atlas 空中前空翻**：起跳瞬間 P₁ 最後一刻必須用 GRF 創造**足夠初始角動量**，空中靠收縮四肢改慣量張量加快翻轉。

<details>
<summary>深入：iLQR Backward/Forward Pass 完整 Riccati 推導</summary>

### Q-function 二次展開

$$
Q(\delta x, \delta u) = \ell(x + \delta x, u + \delta u) + V'(f(x + \delta x, u + \delta u))
$$

在 $(\bar{x}, \bar{u})$ 周圍泰勒展開到二階：

$$
\begin{aligned}
Q_x &= \ell_x + f_x^\top V_x' \\
Q_u &= \ell_u + f_u^\top V_x' \\
Q_{xx} &= \ell_{xx} + f_x^\top V_{xx}' f_x \\
Q_{uu} &= \ell_{uu} + f_u^\top V_{xx}' f_u \\
Q_{ux} &= \ell_{ux} + f_u^\top V_{xx}' f_x
\end{aligned}
$$

（DDP 會多加 $V_x' \cdot f_{xx}, f_{uu}, f_{ux}$ 二階項，iLQR 省略）

### 最優控制律

$\delta u^* = -Q_{uu}^{-1}(Q_u + Q_{ux} \delta x) = k + K \delta x$

- 前饋 $k = -Q_{uu}^{-1} Q_u$
- 反饋 $K = -Q_{uu}^{-1} Q_{ux}$

### Value function 反向更新

$$
V_x = Q_x + K^\top Q_{uu} k + K^\top Q_u + Q_{ux}^\top k
$$

$$
V_{xx} = Q_{xx} + K^\top Q_{uu} K + K^\top Q_{ux} + Q_{ux}^\top K
$$

### Regularization 防止 $Q_{uu}$ 不正定

加 $\mu I$ 到 $Q_{uu}$，如同 Levenberg-Marquardt。$\mu$ 大時退化為梯度下降（穩但慢），$\mu$ 小時接近 Newton（快但可能發散）。

</details>

<details>
<summary>深入：Direct Collocation 與 Casadi 實作（Multiple Shooting 對照）</summary>

### Hermite-Simpson Collocation

每段 $[t_k, t_{k+1}]$ 中點 $t_{k+1/2}$ 用三次 Hermite 插值：

$$
x(t_{k+1/2}) = \frac{1}{2}(x_k + x_{k+1}) + \frac{h}{8}(\dot{x}_k - \dot{x}_{k+1})
$$

動力學約束：中點的動力學值必須符合插值的導數

$$
\dot{x}(t_{k+1/2}) = -\frac{3}{2h}(x_k - x_{k+1}) - \frac{1}{4}(\dot{x}_k + \dot{x}_{k+1})
$$

等式 $\dot{x}(t_{k+1/2}) - f(x(t_{k+1/2}), u(t_{k+1/2})) = 0$ 就是 collocation constraint。

### Casadi Multiple Shooting 骨架

```python
import casadi as ca

opti = ca.Opti()
X = opti.variable(nx, N+1)  # 狀態
U = opti.variable(nu, N)    # 控制

# 動力學約束（Runge-Kutta 4 integration）
def rk4(x, u, dt):
    k1 = f(x, u)
    k2 = f(x + dt/2 * k1, u)
    k3 = f(x + dt/2 * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

for k in range(N):
    x_next = rk4(X[:, k], U[:, k], dt)
    opti.subject_to(X[:, k+1] == x_next)  # Gap closing
    opti.subject_to(opti.bounded(u_min, U[:, k], u_max))

# 邊界條件
opti.subject_to(X[:, 0] == x0)
opti.subject_to(X[:, N] == xf)

# 目標函數
cost = sum(ca.sumsqr(U[:, k]) for k in range(N))
opti.minimize(cost)

opti.solver('ipopt')
sol = opti.solve()
```

### Multi-phase 發散陷阱的工程正解

NLP 本質是**局部優化**；Multi-phase 存強非線性（SO(3) 旋轉 + GRF 雙線性耦合）。初值猜錯（GRF 不足克服重力、落腳點太遠）→ 立刻卡局部死胡同。

**工程鐵律**：
1. 先用**簡化 SRBD (Single Rigid Body Dynamics) 或 LIPM** 解粗糙但物理合理的質心軌跡 + GRF（凸問題，秒解）
2. 把這個粗解作 **Initial Guess** 餵給全尺寸 TrajOpt
3. 100% 收斂，而且通常比純暴力猜零初值快 10-100 倍

這是業界經驗的簽名。

</details>

<details>
<summary>深入：TrajOpt → 閉環控制橋接（TVLQR / Funnel / DAgger）</summary>

### 為什麼開環軌跡不能直接執行

TrajOpt 榨乾模型每分潛力——解通常**剛好踩在摩擦錐極限邊緣**。真實誤差（質量 / 慣量 / 摩擦）+ 擾動（風、地不平）+ 狀態漂移 → 蝴蝶效應發散 → 機器人摔倒。

### TVLQR (Time-Varying LQR) 橋接方案

- **名義軌跡** $\bar{x}(t), \bar{u}(t)$ 作基準
- 沿軌跡展開 $\delta x_{k+1} \approx A_k \delta x_k + B_k \delta u_k$
- 反向解 Riccati → 時變反饋增益 $K(t)$
- **實機控制律**：$u(t) = \bar{u}(t) + K(t) \cdot [x(t) - \bar{x}(t)]$

$\bar{u}$ 提供**前饋大動力**；$K$ 像彈簧拉回偏離的狀態。

### iLQR 的 K 天然免費

iLQR Backward Pass 已經算出 $K_k$！直接把 $(u^*_k, x^*_k, K_k)$ 下發硬體**即插即用閉環跟蹤**。這是分辨「只會用 iLQR」vs「懂 iLQR」的關鍵差別。

### Funnel Control / ROA (MIT Tedrake)

**Sum-of-Squares (SOS) 最佳化** 計算包覆軌跡的李雅普諾夫「不變管」(Invariant Tube)：

$$
V(x, t) \leq \rho(t) \Rightarrow x(t+dt) \text{ still in funnel}
$$

初始擾動只要落在 Funnel 入口（$V(x_0) \leq \rho(0)$），**理論保證**不摔 + 收束到終點。研究所等級的 TrajOpt 魯棒性保證。

### TrajOpt + NN Residual (DAgger + Distillation) 現代做法

1. 離線用 TrajOpt 生成**萬條不同擾動下的完美軌跡**（Teacher）
2. 訓 NN Student policy 擬合
3. 部署 O(1) 極速閉環 + 保留 TrajOpt 最優性

### 「開環完美、真機發散」面試答題範式

「TrajOpt 給的只是**前饋 (Feedforward)**。上真機我會：
1. 沿名義軌跡**算 TVLQR $K_t$** 提供閉環反饋
2. 若底層有 WBC（whole-body controller），TrajOpt 的 $\bar{x}, \dot{\bar{x}}, \ddot{\bar{x}}$ 作參考指令餵 1kHz WBC
3. **WBC 的 PD 阻抗 + 動力學前饋吸收未建模干擾** — 這是 Atlas / ANYmal 極限運動的業界鐵律」

</details>

## 常見參數化方法比較
