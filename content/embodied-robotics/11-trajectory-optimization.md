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
| 三次 Cubic | $C^1$（速度連續） | 教學示範 | 單段加速度不能同時指定起／終點值 → 多段拼接時交接點加速度不連續 → jerk 發散 → 微小震顫 |
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

**物理意義**：4 個係數剛好解 4 個邊界條件（起點位置/速度、終點位置/速度），是最簡單的平滑軌跡。但單段內加速度 $\ddot{q} = 2a_2 + 6a_3 t$ 雖是連續線性函數，起／終點加速度值無法指定，**多段 cubic 拼接時交接點加速度不連續 → jerk 出現階躍 → 微小震顫**。

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

四旋翼是**欠驅動系統**（12 狀態 `[x,y,z,ẋ,ẏ,ż,φ,θ,ψ,p,q,r]` 只有 4 馬達輸入；這裡的 $\psi$ 是歐拉 RPY 的 yaw）。Mellinger-Kumar 2011 證明：四旋翼是 **Differentially Flat System**，即存在 4 個 flat outputs $\sigma = [x, y, z, \psi]$（3D 位置 + **yaw，與狀態變數中的 $\psi$ 同一個量**；roll $\phi$ 與 pitch $\theta$ 不進入 flat outputs，因為它們可由位置軌跡的二階導數代數反推），所有 12 狀態和 4 控制輸入都能**以純代數方程完全從 $\sigma$ 及其有限階導數導出**。

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

**代入 $k = -Q_{uu}^{-1} Q_u$、$K = -Q_{uu}^{-1} Q_{ux}$ 化簡後**（標準形式，實作時常直接用這個）：

$$
V_x = Q_x - Q_{ux}^\top Q_{uu}^{-1} Q_u = Q_x + K^\top Q_u, \qquad V_{xx} = Q_{xx} - Q_{ux}^\top Q_{uu}^{-1} Q_{ux} = Q_{xx} + K^\top Q_{ux}
$$

上面展開式保留四項代數上無誤，但項與項的符號關係依賴 $k, K$ 定義中的負號；讀者若只看展開式、沒一起代入 $k, K$，很容易算錯。

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

| 方法 | 連續性 | 局部控制 | 優點 | 缺點 |
|------|--------|---------|------|------|
| 三次多項式 | $C^1$ | 否 | 簡單、解析解 | 加速度不連續 |
| 五次多項式 | $C^2$ | 否 | 加速度連續 | 高階震盪（多段時） |
| 梯形速度 | $C^0$（速度） | 是 | 工業標準、好算 | jerk 突變（無限大） |
| S 曲線（7 段） | $C^1$（加速度） | 是 | jerk 有限、消除衝擊 | 參數多、計算較複雜 |
| B-spline | $C^{k-1}$ | **是** | 局部修改不影響整體 | 需要選 knot vector |
| Minimum snap | $C^3$ | 否 | 四旋翼標準 | 計算量隨 waypoint 增長 |
| Bezier（凸包性） | $C^k$ | 是 | 控制點幾何約束 = 避障 | 必須 same-degree |

<details>
<summary>深入：B-spline 基礎與 Cox-de Boor 遞推</summary>

### B-spline 軌跡

$k$ 階 B-spline 曲線由控制點 $P_i$ 和 knot vector $\{t_0, t_1, \ldots, t_{n+k}\}$ 定義：

$$
q(t) = \sum_{i=0}^{n} N_{i,k}(t) P_i
$$

其中 $N_{i,k}(t)$ 是 B-spline basis function（Cox-de Boor 遞推）：

$$
N_{i, 0}(t) = \begin{cases} 1 & t_i \leq t < t_{i+1} \\ 0 & \text{otherwise} \end{cases}
$$

$$
N_{i, k}(t) = \frac{t - t_i}{t_{i+k} - t_i} N_{i, k-1}(t) + \frac{t_{i+k+1} - t}{t_{i+k+1} - t_{i+1}} N_{i+1, k-1}(t)
$$

**核心性質**：
- **局部控制**：移動一個控制點只影響 $k$ 個 knot span 內的曲線（不像全域多項式會 Runge 震盪）
- **凸包性**：曲線段落在控制點的凸包內（collision checking 友善）
- **微分簡單**：$k$ 階 B-spline 的導數是 $k-1$ 階 B-spline，直接操作控制點差分

**軌跡最佳化中的用法**：把控制點當決策變數，用 QP 或 NLP 最小化 $\int \|\ddot{q}\|^2 dt$ 或 $\int \|\dddot{q}\|^2 dt$，同時加入速度/加速度/避障約束。

</details>

## 直覺理解

**類比：開車旅行**

- **Path** = Google Maps 畫出的路線（只有幾何）
- **Trajectory** = 完整的駕駛計畫：哪裡踩油門、哪裡踩煞車、彎道前要減速、而且不讓乘客暈車
- **時間最優** = 賽車手：全力加速到極限、急煞、彎道切 apex — 快但不舒服
- **能量最優** = 省油駕駛：平順加速、提早鬆油門滑行 — 慢但省油
- **平滑度最優** = 載老人家：起步溫和、煞車漸進、jerk 小到感覺不到 — 最舒適但最慢
- **Pareto 前緣** = 你不可能同時最快又最省油又最舒適，只能在三者間 trade-off

**梯形速度 vs S 曲線的視覺差異**：

- **梯形速度**：速度曲線像梯形（加速-等速-減速），但加速度是方波 → jerk 是 delta function → 啟停瞬間機械臂「咔」一聲
- **S 曲線**：把方波加速度的邊角圓滑化（7 段），jerk 有限 → 啟停絲滑無衝擊

**Differential Flatness 的物理直覺**：
想像四旋翼是個「會寫字的幽靈手」——你只要在 3D 空間畫出一條好看的平滑曲線（flat outputs $[x, y, z, \psi]$），所有 12 維狀態和 4 個馬達推力都能從這條曲線**代數反推**。規劃器不用管動力學，只管畫漂亮曲線，動力學會自動滿足。

**Contact-Implicit 的直覺**：
想像你要「用撞球桿推動一顆球到特定位置」——你不是規劃「球的軌跡」，而是規劃「球桿該在哪個時刻以什麼力道碰球」。Contact-Implicit TrajOpt 把這個思維泛化：機器人不是直接控制被操作物，而是**規劃自己在何時以何種力接觸環境**，靠接觸力間接推動系統。

**模擬器觀察**：
- **MuJoCo**：給同一段 PTP 運動分別用梯形速度和 S 曲線，觀察：(1) 末端軌跡的平滑程度（zoom in 看微小振動），(2) 關節力矩曲線（梯形的力矩有明顯跳變），(3) 末端在目標附近的 settling time（S 曲線因為沒激發高頻共振，收斂更快）
- **Isaac Sim + Drake**：跑 Contact-Implicit TrajOpt 讓機械臂推箱子，觀察求解器如何**自動決定何時觸碰箱子、何時離開** — 這就是 KKT 乘子 $\lambda$ 的物理湧現
- **Gazebo + Pinocchio**：Atlas 騰空後空翻，視覺化角動量向量 $k$ 在飛行相保持常數 — 直接驗證牛頓-歐拉守恆

## 實作連結

**典型工程場景**：

1. **工業 PTP 搬運**：需要最短 cycle time，但力矩不能超限。用 TOPP-RA 對 path planner 給的 waypoints 做時間最優重參數化。先用 RRT/PRM 算幾何路徑，再用 TOPP-RA 注入時間。

2. **焊接/塗膠直線運動**：末端必須沿笛卡爾直線走，且速度均勻、不能抖。用 minimum jerk 或 B-spline 參數化，加 jerk 上限約束。注意笛卡爾直線在關節空間可能經過奇異點 → 要檢查 Jacobian condition number。

3. **四旋翼航點飛行**：waypoints 間用 **minimum snap + Differential Flatness**。Fast-Planner 等開源系統用這個 pipeline：把 3D 幾何曲線直接當 reference，底層 geometric controller 用 flat outputs 反推推力與姿態。

4. **擁擠書架取書（硬約束狹窄空間）**：用 **TrajOpt SQP**。凸化 SDF + Trust Region 迭代，保證不擦撞。MoveIt 2 預設 pipeline 之一。

5. **四足抬腿過障**：Bezier 曲線 swing trajectory，控制點約束高於障礙 → 凸包性保證整條曲線 clearance。

6. **Atlas 後空翻 / 極限跑酷**：Multi-phase Direct Collocation + CITO，用 SRBD 粗解做初值。

7. **ALOHA 靈巧操作**：Action Chunking Transformer + Temporal Ensemble，VLA 直接輸出連續 waypoints，下游阻抗控制器消化殘差。

**Code 骨架**（Python，TOPP-RA）：

```python
import toppra as ta
import numpy as np

# 1. 定義幾何路徑（關節空間 waypoints）
waypoints = np.array([[0, 0, 0, 0, 0, 0],
                       [1.0, -0.5, 0.8, 0, 0, 0],
                       [1.5, -1.0, 1.2, 0, 0, 0]])
path = ta.SplineInterpolator(
    np.linspace(0, 1, len(waypoints)), waypoints)

# 2. 定義約束（速度、加速度、力矩）
vel_limit = np.array([2.0] * 6)   # rad/s
acc_limit = np.array([5.0] * 6)   # rad/s²
pc_vel = ta.constraint.JointVelocityConstraint(vel_limit)
pc_acc = ta.constraint.JointAccelerationConstraint(acc_limit)

# 3. 建立 TOPP-RA 實例並求解
instance = ta.algorithm.TOPPRA(
    [pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
trajectory = instance.compute_trajectory()  # 回傳時間最優軌跡

# 4. 取樣 → 送給 controller
ts = np.linspace(0, trajectory.duration, 100)
qs = trajectory(ts)          # 位置 (100, 6)
qds = trajectory(ts, 1)     # 速度
qdds = trajectory(ts, 2)    # 加速度
```

<details>
<summary>深入：完整實作 — 五次多項式 + minimum jerk（Python）</summary>

```python
import numpy as np
import matplotlib.pyplot as plt


def quintic_trajectory(q0, qf, v0, vf, a0, af, T, dt=0.001):
    """五次多項式軌跡：滿足 6 個邊界條件"""
    A = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, T, T**2, T**3, T**4, T**5],
        [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
        [0, 0, 2, 6*T, 12*T**2, 20*T**3],
    ])
    b = np.array([q0, v0, a0, qf, vf, af])
    coeffs = np.linalg.solve(A, b)

    ts = np.arange(0, T + dt, dt)
    q = sum(coeffs[i] * ts**i for i in range(6))
    qd = sum(i * coeffs[i] * ts**(i-1) for i in range(1, 6))
    qdd = sum(i * (i-1) * coeffs[i] * ts**(i-2) for i in range(2, 6))
    qddd = sum(i * (i-1) * (i-2) * coeffs[i] * ts**(i-3) for i in range(3, 6))
    return ts, q, qd, qdd, qddd


def minimum_jerk(q0, qf, T, dt=0.001):
    """Minimum jerk 軌跡（靜止到靜止的閉式解，Flash & Hogan 1985）"""
    ts = np.arange(0, T + dt, dt)
    tau = ts / T  # 正規化時間 [0, 1]
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    q = q0 + (qf - q0) * s
    sd = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
    qd = (qf - q0) * sd
    sdd = (60 * tau - 180 * tau**2 + 120 * tau**3) / T**2
    qdd = (qf - q0) * sdd
    sddd = (60 - 360 * tau + 360 * tau**2) / T**3
    qddd = (qf - q0) * sddd
    return ts, q, qd, qdd, qddd
```

**注意**：靜止到靜止條件下，五次多項式（$v_0=v_f=a_0=a_f=0$）和 minimum jerk 的解**完全一致** — 都是那條 $10\tau^3 - 15\tau^4 + 6\tau^5$ 曲線。差異出現在有非零邊界條件時。

</details>

<details>
<summary>深入：完整實作 — Min-snap QP（Python + OSQP）</summary>

```python
import numpy as np
import osqp
from scipy import sparse
from math import factorial

def build_Q_snap(T, poly_order=7):
    """構建 snap 二次項矩陣 Q"""
    n = poly_order + 1
    Q = np.zeros((n, n))
    for i in range(4, n):
        for j in range(4, n):
            coeff = (factorial(i) / factorial(i-4)) * (factorial(j) / factorial(j-4))
            Q[i, j] = coeff * T**(i+j-7) / (i+j-7)
    return Q

def min_snap_qp(waypoints, segment_times, poly_order=7):
    """
    waypoints: (M+1, dim), M segments
    segment_times: (M,) 每段時間
    每段 poly_order 階多項式
    """
    M = len(segment_times)
    dim = waypoints.shape[1]
    n = poly_order + 1

    # Block diagonal Q
    Q_blocks = [build_Q_snap(T) for T in segment_times]
    Q = sparse.block_diag(Q_blocks * dim, format='csc')

    # 等式約束：waypoints + C^3 連續
    A_list, b_list = [], []
    # ... (略：waypoint 位置約束 + 相鄰段 C^3 連續性)

    A = sparse.vstack(A_list, format='csc')
    b = np.concatenate(b_list)

    prob = osqp.OSQP()
    prob.setup(Q, np.zeros(Q.shape[0]), A, b, b)  # eq constraint
    result = prob.solve()
    return result.x  # coefficients
```

</details>

<details>
<summary>深入：完整實作 — iLQR Swing-up Pendulum（Python）</summary>

```python
import numpy as np

def ilqr_swingup(dynamics, cost, x0, u_init, max_iter=50):
    """
    iLQR 求解 pendulum swing-up
    dynamics: f(x, u) -> x_next
    cost: (l, l_x, l_u, l_xx, l_uu, l_ux) at (x, u)
    """
    N = len(u_init)
    u = u_init.copy()
    x = np.zeros((N+1, len(x0)))
    x[0] = x0

    for iteration in range(max_iter):
        # Forward pass: roll out 軌跡
        for k in range(N):
            x[k+1] = dynamics(x[k], u[k])

        # Backward pass: 反向 Riccati
        V_x = cost_terminal_x(x[N])
        V_xx = cost_terminal_xx(x[N])
        K = np.zeros((N, nu, nx))
        k_ff = np.zeros((N, nu))

        for k in range(N-1, -1, -1):
            f_x, f_u = linearize(dynamics, x[k], u[k])
            l_x, l_u, l_xx, l_uu, l_ux = quadratize(cost, x[k], u[k])

            Q_x = l_x + f_x.T @ V_x
            Q_u = l_u + f_u.T @ V_x
            Q_xx = l_xx + f_x.T @ V_xx @ f_x
            Q_uu = l_uu + f_u.T @ V_xx @ f_u + mu * np.eye(nu)  # regularization
            Q_ux = l_ux + f_u.T @ V_xx @ f_x

            K[k] = -np.linalg.solve(Q_uu, Q_ux)
            k_ff[k] = -np.linalg.solve(Q_uu, Q_u)

            V_x = Q_x + K[k].T @ Q_uu @ k_ff[k] + K[k].T @ Q_u + Q_ux.T @ k_ff[k]
            V_xx = Q_xx + K[k].T @ Q_uu @ K[k] + K[k].T @ Q_ux + Q_ux.T @ K[k]

        # Line search forward pass
        alpha = 1.0
        for _ in range(10):
            x_new = np.zeros_like(x); x_new[0] = x0
            u_new = np.zeros_like(u)
            for k in range(N):
                u_new[k] = u[k] + alpha * k_ff[k] + K[k] @ (x_new[k] - x[k])
                x_new[k+1] = dynamics(x_new[k], u_new[k])
            if total_cost(x_new, u_new) < total_cost(x, u):
                x, u = x_new, u_new
                break
            alpha *= 0.5

    return x, u, K  # K 是閉環反饋增益 — 部署時用 u_t = u_t* + K_t (x_t - x_t*)
```

**關鍵點**：`K` 在迭代結束時自然產生 → 直接拿去實機做 TVLQR 閉環，不需額外算。

</details>

## 常見誤解

1. **「時間最優 + 能量最優可以同時達到」** — 錯。時間最優要求加速度打滿（bang-bang 控制），力矩峰值大、能耗高；能量最優傾向慢慢來、力矩小。兩者在 Pareto 前緣上是 trade-off 關係。實務上用加權目標函數 $J = w_1 T + w_2 \int \tau^2 dt$ 取折衷。

2. **「多項式階數越高軌跡越好」** — 錯。高階多項式容易出現 **Runge 現象**：中間段產生大幅振盪。B-spline 用分段低階多項式 + 局部控制解決這問題。實務經驗：超過 7 階的全域多項式幾乎不用。

3. **「笛卡爾空間的直線路徑在關節空間也是平滑的」** — 錯。笛卡爾直線經過 IK 轉到關節空間後，如果路徑靠近**奇異構型**（singular configuration），Jacobian 接近秩虧 → 關節速度會爆掉。**避開**：走笛卡爾直線前先掃描 Jacobian condition number $\kappa(J)$，若 $\kappa > 100$ 就繞路或改用關節空間插值。

4. **「只做運動學限幅就夠了」** — 錯。只限 $\dot{q}_{\max}$ 和 $\ddot{q}_{\max}$ 不考慮動力學（質量、Coriolis、重力），算出來的軌跡可能需要超出馬達額定的力矩。TOPP-RA 的價值就是把**動力學約束**（$\tau_{\min} \leq \tau \leq \tau_{\max}$）納入考量。

5. **「iLQR 和 DDP 差不多，隨便用哪個」** — 錯。DDP 保留動力學二階 Hessian 張量，二次收斂、感知世界曲率，但計算成本極高（$O(n^3)$ 張量乘法）。**絕大多數 production 系統用 iLQR** 因為性價比最高；DDP 只在極動態動作（高速旋轉、強非線性）才值得付這個計算代價。

6. **「Atlas 後空翻可以用 Single Shooting 求解」** — 錯。Single Shooting 求解器早期迭代給錯一點力矩 → 物理引擎積分到半空炸飛 → 梯度發散找不到。Atlas 後空翻必須用 **Direct Collocation**，允許迭代初期違反物理定律（Defect $\neq 0$），先把幾何飄起來再慢慢收緊動力學約束。

7. **「開環 TrajOpt 軌跡可以直接丟給真機執行」** — 錯。TrajOpt 榨乾模型每分潛力（剛好在摩擦錐邊緣），真實擾動一秒內發散。**必須加閉環**：TVLQR $K_t$、WBC 1kHz 吸收擾動、或 Funnel SOS 保證管內不摔。這是「開環完美、真機發散」的 root cause。

8. **「Contact-Implicit 和 Mode Sequence 差不多」** — 錯。Mode Sequence 人為寫死「0.5s 起跳、1.2s 落地」→ 計算快，但犧牲自發性；Contact-Implicit 優化器**自發現**切換時刻，但 MPCC 鬆弛難收斂。MIT Cheetah 3 早期用 Mode Sequence，現代 Drake / Pinocchio 更多用 CITO。

## 練習題

<details>
<summary>Q1：工廠 pick-and-place 線 cycle time 太長，老闆要求縮短 20%，但馬達力矩已經接近極限，你會怎麼做？</summary>

**完整推理鏈**：

1. **確認瓶頸**：先看目前用什麼軌跡規劃 — 如果是梯形速度，加速度限值可能設得太保守。錄下關節力矩曲線，看峰值離真實極限還有多少 margin
2. **TOPP-RA 重參數化**：把現有路徑丟進 TOPP-RA，加入真實的**力矩約束**（不是運動學限值），自動找時間最優。**Bang-Bang 原則保證**時間最優解會讓至少一個關節時刻打滿扭矩，通常比手調梯形參數快 15-30%
3. **路徑也要改**：如果 TOPP-RA 已經打滿力矩還不夠快，問題出在幾何路徑太長。用 RRT-Connect + shortcutting 或 CHOMP 重新規劃更短的路徑
4. **避開的陷阱**：不要只加大加速度限值 — 會超力矩、損壞減速機。也不要忽略 jerk — 即使力矩沒超，jerk 太大會激發諧波振動，positioning 精度下降
5. **最終驗證**：在模擬器裡跑完整 cycle，確認力矩 headroom ≥ 10%，jerk 在可接受範圍，settling time 沒惡化

**結論**：TOPP-RA + 路徑優化雙管齊下，先壓時間、再確認力矩和 jerk 安全。

</details>

<details>
<summary>Q2：四旋翼要穿越 5 個 waypoints 做高速航拍，要求飛行平順且能追蹤 reference，你會選什麼軌跡表示法？為什麼？</summary>

**完整推理鏈**：

1. **選 Minimum Snap + Differential Flatness**（Mellinger-Kumar 2011）：四旋翼的推力 $f$ 正比於加速度（$f = m\ddot{x} + mg$），推力變化率正比於 jerk，**推力變化率的變化率正比於 snap**。最小化 snap → 最小化馬達扭矩的高階變動 → 姿態絲滑、高速穿窗不失控
2. **Diff-Flat 降維打擊**：規劃器只需在 3D 幾何空間畫 $[x, y, z, \psi]$ 曲線，所有 12 狀態和 4 馬達輸入都能代數反推。不用在 12 維非線性動力學空間解 MPC
3. **數學形式**：每段 7 次多項式（8 個係數），5 個 waypoints → 4 段。在 waypoint 處要求 position、velocity、acceleration、jerk 連續（$C^3$）。QP 形式 `min cᵀQc s.t. A_eq c = b_eq`
4. **時間分配**：segment 時間 $T_i$ 用 trapezoidal velocity profile 估初值，再用 NLP 聯合最佳化（Richter et al., 2016）
5. **組合 TOPP-RA**：Min-snap 後再過 TOPP-RA 壓榨馬達扭矩極限——凸 + 快 + 工程確定性極高
6. **避開的陷阱**：忘記考慮 thrust limit → snap 最小化了但某段加速度超出推力上限 → 需要加不等式約束或用 TOPP-RA 做後處理

**結論**：Min-snap QP + Diff-Flat + TOPP-RA 後處理，是 Fast-Planner 等開源無人機軌跡系統的標準 pipeline。

</details>

<details>
<summary>Q3：機械臂要從擁擠書架中取出一本書，中間有多個障礙物，用 CHOMP / STOMP / TrajOpt 哪個？為什麼？</summary>

**完整推理鏈**：

1. **關鍵約束：硬約束 + 狹窄空間** → **TrajOpt SQP**。CHOMP 把避障當軟約束（梯度推離），在狹窄通道可能擦撞；TrajOpt 強制 $\text{SDF}(x) \geq d_{\text{safe}}$，只要 SQP 有解軌跡**絕對無碰撞**
2. **Convexification 凸化**：TrajOpt 把 SDF 一階泰勒展開 `dist(x) ≈ dist(x₀) + J_dist·Δx ≥ d_safe` → 線性不等式約束 → 可信賴域 SQP 解
3. **連續時間碰撞檢測**：TrajOpt 用 swept volume 檢查，避免「時間取樣漏掉細縫穿牆」，這在狹窄書架尤其關鍵
4. **為什麼不選 CHOMP**：CHOMP 的共變梯度對平滑的開放空間很快，但遇到真狹窄硬約束會在障礙邊緣震盪不收斂
5. **為什麼不選 STOMP**：STOMP 強項是處理 non-smooth cost（例如「避開相機遮擋視野」）。書架避障 cost 是光滑的 SDF，STOMP 反而浪費採樣
6. **MoveIt 2 實作**：`planning_pipelines.pipeline_names: [trajopt]`，設定 `d_safe = 0.02` m
7. **避開的陷阱**：TrajOpt 是 SQP，初值很重要 → 先用 RRT-Connect 找粗路徑作初值

**結論**：TrajOpt SQP + SDF 凸化硬約束，是「狹窄空間取物」的標準答案。

</details>

<details>
<summary>Q4：焊接機械臂沿直線焊接時末端抖動嚴重，焊道品質不合格，你的分析流程？</summary>

**完整推理鏈**：

1. **先看頻率**：用 accelerometer 或 encoder 高頻取樣分析末端振動頻譜。低頻（< 10 Hz）→ 軌跡問題；高頻（> 50 Hz）→ 結構共振或 controller 問題
2. **軌跡面向**：
   - 檢查是否用了梯形速度 → jerk 無限大會激發共振 → 換 S 曲線或 minimum jerk（或 septic 7 次多項式）
   - 檢查是否在笛卡爾空間插值且靠近奇異構型 → Jacobian condition number 飆高 → 關節速度震盪 → 加 $\kappa(J)$ 監控，遠離奇異或加 damped least squares
   - 檢查 waypoint 間距是否太大導致插值誤差 → 加密 waypoint
3. **控制面向**：PD gain 太高會放大高頻噪音；加 low-pass filter 在速度命令上
4. **結構面向**：檢查減速機背隙、連桿剛性不足
5. **避開的陷阱**：不要一味降速度來壓抖動 — 這只是遮蓋問題。要找到根因（jerk 突變 / 奇異 / 控制器增益）對症下藥

**結論**：從頻譜分析定位根因，軌跡上用 min jerk + B-spline 降低高頻激發，同時確認奇異點安全距離。

</details>

<details>
<summary>Q5：Atlas 人形要做後空翻，你怎麼公式化這個軌跡最佳化問題？</summary>

**完整推理鏈**：

1. **框架選擇：Direct Collocation + Contact-Implicit + IPOPT**
   - 為什麼不用 Shooting：Shooting 迭代初期給錯一點力矩 → 積分到半空炸飛 → 梯度發散
   - Collocation 允許 $x_k, u_k$ 都當決策變數，動力學是相鄰時點的等式約束，**迭代初期可違反物理定律** → 先讓幾何飄起來再收緊
2. **Multi-phase 分段**：
   - P₁：起跳（雙腳 GRF 驅動）
   - P₂：騰空（GRF = 0，牛頓-歐拉剛體動力學）
   - P₃：落地
3. **騰空相角動量守恆**：強加 $\dot{k}_{\text{com}}(t) = 0$ 或 $k_{\text{com}}(t) = k_{\text{com}}(t_0)$。Atlas 起跳瞬間 P₁ 最後一刻必須用 GRF 創造**足夠初始角動量**，空中靠收縮四肢改慣量張量加快翻轉（花式滑冰原理）
4. **降維：CMM 質心動量空間**：Atlas 有 30+ 關節，直接全狀態規劃爆炸。改規劃 CoM 軌跡 $p_c$ 和角動量 $k$，約束 $\dot{k} = \sum (p_i - p_c) \times f_i$
5. **初值策略**：先用 **SRBD (Single Rigid Body Dynamics)** 解粗糙但物理合理的質心軌跡 + GRF → 作 Initial Guess 餵給全尺寸 TrajOpt。**100% 收斂，且比純暴力猜零初值快 10-100 倍**
6. **閉環橋接**：TrajOpt 給的是開環 $\bar{x}(t), \bar{u}(t)$，部署時沿軌跡算 TVLQR $K_t$ 或餵 WBC 1kHz 作參考，才能吸收真機擾動
7. **避開的陷阱**：不要試圖同時優化 phase 時長 + 軌跡 — NLP 局部最優性極敏感。先固定 phase 時長求軌跡，外層 bisection 調 phase 時長

**結論**：Multi-phase Direct Collocation + CMM 降維 + 角動量守恆 + SRBD 初值 + TVLQR/WBC 閉環。

</details>

<details>
<summary>Q6：ALOHA 遙操作機械臂做「穿針」任務，BC 學出來的 policy 末端抖動嚴重、常常刺偏，怎麼解？</summary>

**完整推理鏈**：

1. **根因分析**：遙操作數據本質充滿高頻雜訊（人類手部抖動、停頓猶豫）。傳統單步 BC (Behavior Cloning) 每次只預測下一步 action，微誤差在幾何上被放大 → 穿針這種精密任務直接崩
2. **解法：Action Chunking Transformer (ACT)**（ALOHA 論文核心）：
   - **一次預測未來 $k$ 步 action chunk**（典型 $k = 100$ 個時步，1 秒左右）
   - 輸入：當前觀測（多相機 RGB + 本體感知）
   - 輸出：未來 k 步 action 序列
3. **Temporal Ensemble**：部署時每步對**所有重疊片段**指數加權移動平均
   - 時刻 $t$ 的 action 從「過去所有 chunk 中對時刻 $t$ 的預測」加權平均
   - 這是在時間維度做低通濾波，抖動被徹底平滑
4. **為什麼 work**：穿針需要高時間連續性，Temporal Ensemble 在時域平滑抖動，比直接 smooth action 更優（保留 chunk 內部動態模式）
5. **進階做法**：VLA (OpenVLA / π0) 直接輸出連續空間 waypoints，下游用阻抗控制器消化殘差 → 「Diffusion Model 想像專家軌跡 → MPC 消化」的 Symbol → Neural 現代架構
6. **避開的陷阱**：不要用 L2 loss 讓 policy 學「平均 action」— 會學出含糊的中間解。用 MDN (Mixture Density Network) 或 Diffusion Head 讓 policy 輸出多 mode 分佈

**結論**：ACT + Temporal Ensemble 是靈巧操作 workhorse，穿針、打雞蛋等精細任務的唯一解。

</details>

<details>
<summary>Q7：離線跑出完美的軌跡，真機一秒內摔倒，為什麼？怎麼救？</summary>

**完整推理鏈**：

1. **根因**：TrajOpt 給的是**開環前饋 (Feedforward)**。求解器榨乾模型每分潛力，解通常**剛好踩在摩擦錐極限邊緣**。真實誤差（質量 / 慣量 / 摩擦）+ 擾動（風、地不平）+ 狀態漂移 → 蝴蝶效應發散 → 摔
2. **TVLQR 橋接方案**：
   - 名義軌跡 $\bar{x}(t), \bar{u}(t)$ 作基準
   - 沿軌跡展開 $\delta x_{k+1} \approx A_k \delta x_k + B_k \delta u_k$
   - 反向解 Riccati → 時變反饋增益 $K(t)$
   - 實機控制律：$u(t) = \bar{u}(t) + K(t) \cdot [x(t) - \bar{x}(t)]$
3. **iLQR 的 K 免費**：如果原本用 iLQR 求解，Backward Pass 已經算出 $K_k$，直接下發硬體即插即用
4. **底層 WBC 架構**：Atlas / ANYmal 業界鐵律——TrajOpt 的 $\bar{x}, \dot{\bar{x}}, \ddot{\bar{x}}$ 作參考指令餵 1kHz WBC，WBC 的 PD 阻抗 + 動力學前饋吸收未建模干擾
5. **Funnel Control (研究所等級)**：用 SOS 最佳化計算李雅普諾夫不變管 (Invariant Tube)，理論保證擾動不超過管邊界時不摔
6. **DAgger + Distillation 現代做法**：離線生成**萬條不同擾動下的完美軌跡**訓 NN Student policy，部署 O(1) 極速閉環 + 保留 TrajOpt 最優性
7. **避開的陷阱**：不要單純加大 PD gain 想「硬扛」擾動 — 會激發結構共振、電機過載

**結論**：TrajOpt（前饋）+ TVLQR/WBC（閉環反饋）是業界標準二層架構。

</details>

## 面試角度

1. **Path vs Trajectory 的精確區分** — 這是最基本但最常被混用的概念。**為什麼這是重點**：面試官立刻判斷你是「看過教材」還是「真的懂」。兩分鐘版本：「Path 只有幾何沒時間；trajectory 是 path 加時間參數化，產出 $q(t), \dot{q}(t), \ddot{q}(t)$。軌跡最佳化就是在物理約束下找最佳時間分配。」

2. **Pareto trade-off：時間 vs 能量 vs 平滑度** — 展現多目標最佳化的本質理解。**為什麼這是重點**：證明你不是天真追求單一指標。兩分鐘版本：「時間最優要 bang-bang、能量最優慢慢來、平滑度最優壓 jerk — 三者在 Pareto 前緣互斥，實務用加權目標或約束式取折衷。」

3. **Quintic 多項式為什麼是工業 PTP 黃金標準** — **為什麼這是重點**：能講清「為什麼不用 cubic」是分辨「背公式」vs「懂物理」。兩分鐘版本：「Cubic 加速度不連續 → jerk 階躍 → 機械衝擊。Quintic 把 acceleration 邊界條件也納入，$C^2$ 連續 + jerk 有限 → 力矩無跳變 → PTP 黃金標準。Jerk-minimization 解析解等價於 quintic。」

4. **Runge 現象 → B-Spline 分段低次** — **為什麼這是重點**：面試官常問「10 個 via-points 怎麼插值」，答「用 9 次多項式」的人立刻被篩掉。兩分鐘版本：「高階全域多項式邊緣劇烈震盪 (Runge)，必用 B-spline 分段低次 + $C^2$ 連續性約束。」

5. **Differential Flatness 降維打擊** — **為什麼這是重點**：無人機面試必備，展現對欠驅動系統的深度理解。兩分鐘版本：「四旋翼 12 狀態 4 輸入，Mellinger 2011 證明 $[x, y, z, \psi]$ 4 個 flat outputs 能代數反推所有狀態。規劃器只需畫 3D 平滑曲線，動力學自動滿足 — 這是 Fast-Planner 的底層靈魂。」

6. **Min-snap 對應馬達推力變化率** — **為什麼這是重點**：能背出 $p \to \ddot{p} \to \dddot{p} \to p^{(4)}$ 的物理鏈分辨「懂四旋翼動力學」vs「只會算 QP」。

7. **TOPP-RA $u = \ddot{s}, v = \dot{s}^2$ 線性化** — **為什麼這是重點**：能講清為什麼非線性動力學變線性，證明「數學基本功紮實」。兩分鐘版本：「沿路徑重參數化後，令 $u = \ddot{s}, v = \dot{s}^2$，動力學變 $\tau = A(s)u + B(s)v + C(s)$ 線性方程。軌跡切 N 段，向前/向後掃描解 LP 序列，1ms 內完成。」

8. **Bang-Bang 原則** — **為什麼這是重點**：時間最優的物理本質，面試「怎麼壓榨硬體極限」的標準答案。兩分鐘版本：「龐特里亞金極小值原理證明時間最優必 bang-bang，每瞬間至少一個關節扭矩打滿 — 滿載加速、滿載勻速、滿載減速。」

9. **KKT 拉格朗日乘子 = 接觸力物理湧現** — **為什麼這是重點**：Contact-Implicit 的數學魔法，分辨「會解 TrajOpt」vs「懂 Contact Dynamics」。兩分鐘版本：「CITO 中 $\phi(q) \geq 0, \lambda \geq 0, \phi \cdot \lambda = 0$ 互補約束的 KKT 乘子 $\lambda$ 物理湧現為接觸力。求解器自己決定何時換接觸模式。」

10. **Atlas 後空翻 + 機械臂推箱子同一方程** — **為什麼這是重點**：展示系統觀的必殺題。兩分鐘版本：「兩者都是 underactuated dynamics：無法直接控被操作物或空中質心，只能找最優接觸點 + 接觸力 $J^\top f_c$ 間接改狀態。掌握 Contact-Implicit → 足式 + 靈巧手同時打通。」

11. **Shooting vs Collocation：Atlas 後空翻為什麼只能用 Collocation** — **為什麼這是重點**：分辨「會用」vs「懂」的標誌。兩分鐘版本：「Shooting 給錯力矩就炸飛梯度發散；Collocation 允許迭代初期違反物理定律，先讓幾何飄起來再收緊動力學。」

12. **iLQR 的 K 免費給閉環** — **為什麼這是重點**：分辨「用 iLQR」vs「懂 iLQR」。Backward Pass 已算出的 $K_k$ 直接下發硬體就是 TVLQR 閉環。

13. **TrajOpt 前饋 + WBC 閉環 = Atlas 業界鐵律** — **為什麼這是重點**：面試「開環發散」的標準答案。兩分鐘版本：「TrajOpt 給前饋 $\bar{x}, \bar{u}$，沿軌跡算 TVLQR $K_t$ 提供反饋；底層若有 WBC 就餵 1kHz 阻抗控制器吸收擾動。」

14. **ALOHA Temporal Ensemble 時間低通** — **為什麼這是重點**：靈巧操作 workhorse 的平滑魔法。兩分鐘版本：「ACT 一次輸出 k 步 chunk，部署時對重疊片段指數加權平均 — 時間維度低通濾波，穿針打雞蛋等精細操作唯一解。」

## 延伸閱讀

- **Pham & Pham, "TOPP-RA: Time-Optimal Path Parameterization for Redundantly Actuated Robots" (2018)** — TOPP-RA 原始論文，演算法清晰、有開源 Python 實作 `toppra`，是工業界時間最優的標準參考
- **Mellinger & Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors" (2011)** — 四旋翼 min-snap + Differential Flatness 的經典論文，定義了 2011 後無人機軌跡的業界標準
- **Schulman et al., "Finding Locally Optimal, Collision-Free Trajectories with Sequential Convex Optimization" (2013)** — TrajOpt 原始論文，SQP + SDF 凸化，MoveIt 2 預設 pipeline 之一
- **Posa, Kuindersma & Tedrake, "Optimization and stabilization of trajectories for constrained dynamical systems" (2016)** — Contact-Implicit TrajOpt 與 MPCC 鬆弛的奠基論文
- **Todorov & Li, "A generalized iterative LQG method for locally-optimal feedback control of constrained nonlinear stochastic systems" (iLQR 最重要的原論文之一)** — iLQR 在 MPC / 閉環場景的奠基
- **Kuindersma et al., "Optimization-based locomotion planning, estimation, and control design for Atlas" (2016)** — Boston Dynamics Atlas 軌跡最佳化系統的完整公開描述
- **Flash & Hogan, "The Coordination of Arm Movements" (1985)** — Minimum jerk 假說源頭論文，解釋為什麼人類手臂自然運動遵循五次多項式
- **Zhao et al., "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware" (ALOHA, 2023)** — Action Chunking Transformer + Temporal Ensemble 的原論文，靈巧操作 SOTA
- **Janner et al., "Planning with Diffusion for Flexible Behavior Synthesis" (2022)** — Diffusion Model 用於軌跡生成的開山之作
- **Lynch & Park,《Modern Robotics》Ch9 Trajectory Generation** — 多項式、梯形速度、S 曲線的系統性教材，MIT OCW 有完整影片
- **Tedrake,《Underactuated Robotics》（MIT 6.832）Ch10-12** — iLQR、Collocation、SOS Funnel 的學術權威教材，免費線上
- **`toppra` Python 套件（GitHub: hungpham2511/toppra）** — TOPP-RA 的官方開源實作，直接用在 ROS 2 + MoveIt pipeline
- **`drake` (Robotics Toolbox, Russ Tedrake)** — MathematicalProgram + Contact-Implicit + iLQR + Collocation 一站式工具箱
- **`crocoddyl`（INRIA / Pinocchio）** — DDP / FDDP 高效 C++ 實作，ANYmal 四足使用

