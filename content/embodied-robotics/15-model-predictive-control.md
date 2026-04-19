---
title: "模型預測控制 (MPC) 與軌跡追蹤"
prerequisites: ["13-pid-control-tuning", "11-trajectory-optimization"]
estimated_time: 45
difficulty: 4
tags: ["mpc", "control", "optimization", "real-time"]
sidebar_position: 15
---

# 模型預測控制 (MPC) 與軌跡追蹤

## 你將學到

- 能精確講出 MPC 和 PID 的本質差異：MPC 看未來 N 步 + 處理約束，PID 看現在和過去
- 遇到「機器人要在關節力矩限制下追蹤高速軌跡」時，知道該用 MPC 而非加更多 PID 增益
- 判斷什麼時候用線性 MPC（QP，毫秒級）、什麼時候必須上 NMPC（SQP/IPOPT，更精確但慢），以及跑不進即時迴圈時怎麼壓

## 核心概念

**精確定義**：**Model Predictive Control (MPC)** 是一種在線最佳化控制策略 — 每個控制週期，用系統的**預測模型**往前推演 $N$ 步，在**約束**（力矩上限、關節極限、摩擦錐等）下最小化一個**代價函數**（追蹤誤差 + 控制力），只執行解出的第一步控制量，然後下一個週期重新求解。這種「**滾動時域 (receding horizon)**」機制讓 MPC 天然具備前瞻性與約束處理能力。

**MPC 三要素**：
1. **預測模型** — 離散化的系統動力學 $x_{k+1} = f(x_k, u_k)$（線性或非線性）
2. **代價函數** — 狀態追蹤誤差 + 控制能量的加權和
3. **約束** — 硬性物理限制（力矩飽和、速度極限、避碰、摩擦錐）

**vs PID**：PID 是 **反應式** — 看到誤差才修正，沒有未來的概念，也無法處理約束（頂多 anti-windup 做飽和後補償）。MPC 是 **預測式** — 在約束下選出未來 N 步最佳策略，天生能處理硬約束。

**vs Ch11 離線軌跡最佳化**：Ch11 的 trajectory optimization 是離線生成整條軌跡；MPC 是在線滾動重規劃，相當於每個 tick 做一次小型 trajectory optimization。MPC 可以包裹或替代 PID 作為高階追蹤控制器。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：當前狀態估計 $x_k$（來自感測器 / 狀態估計器）、參考軌跡 $x^{\text{ref}}_{k:k+N}$（來自規劃器）
- **輸出**：當前時刻的最佳控制量 $u_k^*$（力矩、速度或加速度指令，送給底層驅動器 / PID）
- **下游**：驅動器執行 → 系統狀態改變 → 感測器回饋 → 下一次 MPC 求解
- **閉環節點**：坐落在 **控制層**，但兼具「在線重規劃」的特性，是規劃與控制的橋樑

**最少夠用的數學**：

1. **線性 MPC 的 QP 標準形式**（最常見的工業級配置）：

$$
\min_{U} \sum_{k=0}^{N-1} \left[ (x_k - x_k^{\text{ref}})^T Q (x_k - x_k^{\text{ref}}) + u_k^T R\, u_k \right] + (x_N - x_N^{\text{ref}})^T Q_f (x_N - x_N^{\text{ref}})
$$

$$
\text{s.t.} \quad x_{k+1} = A x_k + B u_k, \quad u_{\min} \le u_k \le u_{\max}, \quad x_{\min} \le x_k \le x_{\max}
$$

**物理意義**：$Q$ 懲罰追蹤偏差（越大越緊追），$R$ 懲罰控制能量（越大越省力但追不緊），$Q_f$ 是終端代價（保證有限時域穩定性）。整個問題是 **Quadratic Program (QP)**，有成熟的毫秒級求解器（OSQP、qpOASES）。

2. **滾動時域原理**（receding horizon）：

$$
u^*_{0:N-1} = \arg\min J(U) \quad \Rightarrow \quad \text{只執行 } u^*_0 \text{，丟棄 } u^*_{1:N-1}
$$

**物理意義**：每個控制週期都重新求解，用最新的狀態量測修正模型偏差。這讓 MPC 具備天然的閉環魯棒性 — 即使模型不完美，持續回饋修正也能壓住誤差。

3. **NMPC（非線性 MPC）**：預測模型改為非線性 $x_{k+1} = f(x_k, u_k)$，代價函數和約束也可以是非線性的。求解變成 **Nonlinear Program (NLP)**，用 SQP（Sequential Quadratic Programming）或 IPOPT（Interior Point Optimizer）求解。更精確但計算量大。

<details>
<summary>深入：從 QP 到求解器 — MPC 的計算流程與凸鬆弛技巧</summary>

### 線性 MPC 的 QP 矩陣展開

將預測模型 $x_{k+1} = Ax_k + Bu_k$ 沿 $N$ 步展開，可以把所有未來狀態表達為初始狀態 $x_0$ 和控制序列 $U = [u_0, u_1, \dots, u_{N-1}]^T$ 的仿射函數：

$$
\mathbf{X} = \mathcal{A} x_0 + \mathcal{B} U
$$

其中 $\mathcal{A}$ 是 $A$ 的冪次堆疊，$\mathcal{B}$ 是 $B$ 和 $A$ 組成的下三角 Toeplitz 矩陣。代入代價函數後，得到標準 QP：

$$
\min_U \frac{1}{2} U^T H U + g^T U \quad \text{s.t.} \quad C U \le d
$$

$H = \mathcal{B}^T \bar{Q} \mathcal{B} + \bar{R}$ 是正定的（只要 $Q \succeq 0, R \succ 0$），保證全局唯一解。

### 求解器選擇

| 求解器 | 方法 | 典型速度 | 適用場景 |
|--------|------|----------|----------|
| OSQP | ADMM | < 1 ms (中等規模) | 嵌入式、四足 |
| qpOASES | Active-set | < 0.5 ms (小規模) | 即時硬約束 |
| ECOS | Interior-point | 1–10 ms | SOCP 錐約束 |
| IPOPT | Interior-point (NLP) | 10–100 ms | NMPC |
| acados | 結構利用 SQP_RTI | < 1 ms (NMPC) | 高速 NMPC |

### 凸鬆弛技巧

非線性約束無法直接用 QP 解。常見做法：

1. **逐步線性化 (SQP_RTI)**：每一步只做一次 QP 迭代而非解到收斂，搭配 warm start 在下一個控制週期接著解
2. **摩擦錐鬆弛**：庫侖摩擦錐 $\|f_t\| \le \mu f_n$ 是 SOCP（Second-Order Cone Program），可以用 ECOS 直接解
3. **凸化近似**：把非凸障礙物約束用半平面或凸多面體包圍，轉成線性不等式

### 穩定性保證

線性 MPC 的閉環穩定性通常需要：
- 終端代價 $Q_f$ 是 Riccati 方程的解（等價於無限時域 LQR 的代價）
- 終端約束集 $\mathcal{X}_f$ 是不變集
- 在工業實務中，通常選夠長的 $N$ + 足夠大的 $Q_f$ 就夠穩定，不嚴格要求終端約束

</details>

**常用 API / 工具鏈**：

| 層級 | 工具 | 介面示例 |
|------|------|----------|
| QP 求解器 | OSQP | `osqp.solve()` → 回傳 $u^*$ |
| NMPC 框架 | acados | `ocp_solver.solve_for_x0(x0)` → RTI 一步解 |
| NLP 求解器 | CasADi + IPOPT | `solver(x0=..., lbg=..., ubg=...)` → 通用 NLP |
| 四足 MPC | MIT Cheetah / OCS2 | 線性化 SRBD + QP，1 kHz |
| ROS 2 整合 | ros2_control + MPC plugin | `update()` 回傳 `JointTrajectory` |

## 直覺理解

**類比：開車看前方 N 步**。PID 像只看方向盤偏了多少就修 — 反應式。MPC 像老司機目光看前方 200 公尺，腦中預演「如果我現在踩多少油門、過 3 秒會到哪、會不會超速（約束）」，然後選出最佳操作。每一秒都重新預演，因為路況（狀態回饋）會變。

**視覺比喻：圍棋 AI 的前瞻搜索**。MPC 的 horizon $N$ 就像 AlphaGo 往前搜 $N$ 步棋；搜越深越聰明（追蹤越好），但算力越貴（求解時間越長）。代價函數像棋局評估函數，約束像規則。滾動時域像每走一步就重新搜索 — 因為對手（擾動）會出乎意料。

**模擬器觀察**：在 MuJoCo 裡跑一個四足機器人，把 MPC horizon 從 $N = 5$ 拉到 $N = 30$，觀察：
- $N$ 太短：機器人走路不穩、無法提前準備轉彎
- $N$ 適中：平穩行走，力矩在約束內
- $N$ 太長：每步求解時間拉長，控制頻率下降，反而開始抖
- 把 `u_max` 從 33 Nm 降到 10 Nm：MPC 會自動放慢速度去滿足約束；PID 直接飽和後亂飛

## 實作連結

**三個典型工程場景**：

1. **四足機器人行走 MPC**：用 Single Rigid Body Dynamics (SRBD) 簡化 18-DoF 四足為 6-DoF 質心模型，在摩擦錐約束 $\|f_t\| \le \mu f_n$ 下求解 QP，1 kHz 輸出接觸力 → 逆動力學分配到關節力矩。MIT Cheetah 3 / Unitree Go1 都是這個架構。

2. **機械臂即時軌跡追蹤**：MoveIt 生成離線軌跡，但執行時有外力擾動。用 MPC 替代 PID 做追蹤控制器：預測模型是線性化的操作空間動力學 $M\ddot{x} + C\dot{x} = F$，約束是關節力矩/速度上限，代價是追蹤誤差。acados 框架可以在 1 ms 內解完。

3. **自駕車路徑跟隨**：車輛用 bicycle model（非線性），NMPC 在車道邊界 + 速度限制約束下最小化橫向偏差和航向誤差。CasADi + IPOPT 是學術界標配；量產用 acados 或客製 QP。

**Code 骨架**（Python，CasADi 版線性 MPC）：

```python
import casadi as ca
import numpy as np

# 建立 MPC 問題
N = 20  # prediction horizon
nx, nu = 4, 2  # 狀態/控制維度

# 決策變數
U = ca.MX.sym('U', nu, N)
X = ca.MX.sym('X', nx, N + 1)
P = ca.MX.sym('P', nx + nx * N)  # 初始狀態 + 參考軌跡

# 代價函數 + 約束 (骨架)
cost = 0
constraints = [X[:, 0] - P[:nx]]  # 初始狀態約束
for k in range(N):
    x_ref = P[nx + k * nx : nx + (k+1) * nx]
    cost += ca.mtimes([(X[:, k] - x_ref).T, Q, (X[:, k] - x_ref)])
    cost += ca.mtimes([U[:, k].T, R, U[:, k]])
    x_next = A @ X[:, k] + B @ U[:, k]  # 線性模型
    constraints.append(X[:, k+1] - x_next)

# nlp_solver = ca.nlpsol('solver', 'ipopt', {...})
# sol = nlp_solver(x0=..., lbx=u_min, ubx=u_max, ...)
# u_optimal = sol['x'][:nu]  # 只取第一步
```

<details>
<summary>深入：完整可執行的線性 MPC 範例（Python + OSQP）</summary>

```python
import numpy as np
import osqp
from scipy import sparse

def build_linear_mpc(A, B, Q, R, Qf, N, x_min, x_max, u_min, u_max):
    """
    建立線性 MPC 的 QP 矩陣（離線做一次）
    A: (nx, nx), B: (nx, nu), Q/R/Qf: 權重, N: horizon
    回傳 OSQP solver 物件
    """
    nx, nu = B.shape

    # 代價矩陣 H = blkdiag(Q, R, Q, R, ..., Qf)
    P_blocks = []
    for k in range(N):
        P_blocks.append(sparse.block_diag([Q, R], format='csc'))
    P_blocks.append(sparse.csc_matrix(Qf))
    P_cost = sparse.block_diag(P_blocks, format='csc')

    # 等式約束: x_{k+1} = A x_k + B u_k
    # 整理成 Aeq @ z = beq（z = [x0, u0, x1, u1, ..., xN]）
    # 不等式約束: lb <= z <= ub
    nz = (nx + nu) * N + nx  # 決策變數總數

    # 動力學約束矩陣
    Ax_list = []
    for k in range(N):
        row = np.zeros((nx, nz))
        idx_xk = k * (nx + nu)
        idx_uk = idx_xk + nx
        idx_xk1 = (k + 1) * (nx + nu)
        row[:, idx_xk:idx_xk + nx] = A
        row[:, idx_uk:idx_uk + nu] = B
        row[:, idx_xk1:idx_xk1 + nx] = -np.eye(nx)
        Ax_list.append(row)
    Aeq = sparse.csc_matrix(np.vstack(Ax_list))

    # 邊界約束
    lb = np.tile(np.concatenate([x_min, u_min]), N)
    lb = np.concatenate([lb, x_min])
    ub = np.tile(np.concatenate([x_max, u_max]), N)
    ub = np.concatenate([ub, x_max])

    # OSQP 要求: min 0.5 z'Pz + q'z, s.t. l <= Az <= u
    A_total = sparse.vstack([
        Aeq,
        sparse.eye(nz)
    ], format='csc')
    l_total = np.concatenate([np.zeros(N * nx), lb])
    u_total = np.concatenate([np.zeros(N * nx), ub])

    solver = osqp.OSQP()
    solver.setup(P_cost, np.zeros(nz), A_total, l_total, u_total,
                 warm_start=True, verbose=False,
                 eps_abs=1e-6, eps_rel=1e-6, max_iter=200)
    return solver


def mpc_step(solver, x0, x_ref_trajectory, nx, nu, N):
    """
    在線 MPC 一步求解
    x0: 當前狀態
    x_ref_trajectory: (N+1, nx) 參考軌跡
    回傳 u0: 第一步最佳控制量
    """
    # 更新代價函數的線性項 q（追蹤參考）
    q = np.zeros((nx + nu) * N + nx)
    # ... (根據 x_ref 更新 q)

    # 更新初始狀態約束
    # solver.update(l=..., u=..., q=q)
    result = solver.solve()

    if result.info.status != 'solved':
        print(f"MPC 求解失敗: {result.info.status}")
        return np.zeros(nu)  # fallback

    u0 = result.x[nx:nx + nu]  # 取第一步控制量
    return u0


# 使用範例：雙積分器 (位置+速度)
dt = 0.01  # 100 Hz
A = np.array([[1, dt], [0, 1]])
B = np.array([[0.5 * dt**2], [dt]])
Q = np.diag([10.0, 1.0])   # 位置追蹤權重大
R = np.array([[0.1]])       # 控制量懲罰小 → 積極追蹤
Qf = 10 * Q                # 終端代價
N = 30                      # 0.3 秒前瞻

solver = build_linear_mpc(
    A, B, Q, R, Qf, N,
    x_min=np.array([-10, -5]),
    x_max=np.array([10, 5]),
    u_min=np.array([-1.0]),  # 力矩限制
    u_max=np.array([1.0]),
)
# 控制迴圈: u = mpc_step(solver, x_now, x_ref, nx=2, nu=1, N=30)
```

**重點**：
- OSQP 支援 warm start — 上一步的解作為下一步的初始猜測，典型加速 2-5x
- 矩陣只建一次（離線），在線只更新 $q$（線性項）和約束邊界
- 100 Hz 控制迴圈下，OSQP 對中等規模問題 ($N=30$, $nx=12$) 通常 < 1 ms

</details>

<details>
<summary>深入：NMPC 加速技巧 — 從跑不進即時到 1 kHz</summary>

### 問題

標準 NMPC 用 IPOPT 解 NLP，典型需要 10-100 ms，跑不進 1 kHz (1 ms) 的控制迴圈。

### 加速策略（按效果排序）

1. **RTI (Real-Time Iteration)**：不把 NLP 解到收斂，每個控制週期只做一次 SQP 迭代。acados 框架的核心思路。在模型變化不劇烈時，每次一步就夠接近最優。

2. **Warm Start**：用上一個週期的解平移一格作為初始猜測。$u^*_{1:N-1}$ 左移，末端補上一個合理的 $u_{\text{init}}$。

3. **稀疏結構利用**：MPC 的 KKT 矩陣天然是帶狀稀疏的。acados 用 HPIPM（High-Performance Interior Point Method）直接利用這個結構，比通用 IPOPT 快 10-100x。

4. **模型簡化**：
   - 四足用 SRBD（6-DoF 質心）替代全身 18-DoF 模型
   - 機械臂用 operational space 線性化替代完整非線性動力學
   - 車輛用 bicycle model 替代完整輪胎模型

5. **顯式 MPC (Explicit MPC)**：對小規模線性 MPC，離線把所有可能的初始狀態分區，每個分區預計算最佳控制法則（分段仿射）。在線只需查表，O(log n) 複雜度。但狀態維度 > 5 時分區數爆炸。

6. **Learning-based MPC**：用神經網路近似 MPC 策略，在線只需做一次前向推理（< 0.1 ms）。但需要安全約束保證 — 通常搭配 CBF (Control Barrier Function) 做安全過濾。

### acados RTI 典型流程

```python
from acados_template import AcadosOcp, AcadosOcpSolver

ocp = AcadosOcp()
ocp.model = my_nonlinear_model        # CasADi 符號模型
ocp.cost.cost_type = 'LINEAR_LS'
ocp.constraints.lbu = u_min
ocp.constraints.ubu = u_max
ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
ocp.solver_options.nlp_solver_type = 'SQP_RTI'  # 關鍵：RTI
ocp.solver_options.tf = N * dt

solver = AcadosOcpSolver(ocp)

# 控制迴圈
for step in range(sim_steps):
    solver.set(0, 'lbx', x_current)
    solver.set(0, 'ubx', x_current)  # 固定初始狀態
    solver.solve()  # RTI: 一次 QP 迭代
    u = solver.get(0, 'u')
    x_current = simulate(x_current, u)
```

</details>

## 常見誤解

1. **「MPC 能處理任意非線性模型」** — 能建模不代表能即時解。NMPC 的 NLP 沒有全局最優保證（非凸），而且計算量隨模型複雜度指數增長。**正確理解**：工業 MPC 絕大多數是線性化 + QP，或用 RTI 近似解 NMPC。真正的全非線性 MPC 只在離線或慢迴圈（< 10 Hz）場景才實用。

2. **「MPC 只能做線性系統」** — 恰恰相反，NMPC 可以處理非線性模型，只是需要特殊求解技巧（SQP_RTI、warm start、結構利用）。四足行走、無人機翻滾、自駕車都在用 NMPC。**關鍵**：不是能不能做非線性，而是能不能在你的控制頻率內解完。

3. **「用了 MPC 就不需要調參」** — MPC 需要調的參數比 PID 還多：$Q$（每個狀態的權重）、$R$（控制懲罰）、$Q_f$（終端代價）、$N$（horizon 長度）、取樣時間 $dt$、模型參數。$Q/R$ 的比值直接決定追蹤 aggressiveness，$N$ 太短會不穩定，$N$ 太長會算不完。**避開**：先用 Bryson's rule（$Q_{ii} = 1 / x_{i,\text{max}}^2$, $R_{jj} = 1 / u_{j,\text{max}}^2$）給合理初值，再微調。

4. **「滾動時域 = 浪費算力，為什麼不直接解全時域？」** — 全時域最佳化（Ch11 trajectory optimization）是離線的，不能回饋修正。MPC 每一步都用最新狀態重新規劃，天然補償模型誤差和外力擾動。代價是每步都要解一次 QP/NLP，但單次規模小（只看 N 步）。**本質**：MPC = 在線閉環版的軌跡最佳化。

## 練習題

<details>
<summary>Q1（中）：四足機器人在濕滑地面走路，原本在乾地面調好的 MPC 開始打滑，你怎麼診斷和修？</summary>

**完整推理鏈**：

1. **判斷根因**：打滑 = 地面反力超出摩擦錐約束 $\|f_t\| \le \mu f_n$。濕滑地面的 $\mu$ 從 ~0.6 降到 ~0.2，但 MPC 的摩擦錐參數還是 0.6
2. **即時修正**：把 MPC 的 $\mu$ 參數降到保守值（0.15-0.2）。摩擦錐收窄 → MPC 會自動減小切向力 → 步態變慢但不打滑
3. **進階：在線摩擦估計**：用接觸力感測器 + 足底滑移偵測，即時估計 $\hat{\mu}$ 並回饋給 MPC 約束
4. **避開陷阱**：不要只靠增大 $Q$ 來「追更緊」— 這會讓 MPC 輸出更大的切向力，在低摩擦下反而更滑

**面試官想聽到**：MPC 的核心優勢就是約束處理。摩擦估計 → 約束更新 → MPC 自動調整策略，這條鏈講清楚就夠。

</details>

<details>
<summary>Q2（中-難）：你的 NMPC 跑在 50 Hz（20 ms / 步），但控制迴圈需要 200 Hz（5 ms）。不能換硬體，怎麼壓到 5 ms 以內？</summary>

**完整推理鏈**：

1. **先 profile**：確認瓶頸是 NLP 求解（通常是），不是模型建立或矩陣組裝
2. **RTI (Real-Time Iteration)**：把 SQP 從解到收斂改為每步只做一次 QP 迭代。acados 的 SQP_RTI 模式。典型加速 5-10x
3. **Warm start**：上一步的解平移一格當初始猜測，減少 QP 迭代次數
4. **稀疏結構利用**：從通用 IPOPT 換到 HPIPM，直接利用 MPC 帶狀稀疏結構
5. **模型簡化**：如果是全身動力學，改用簡化模型（SRBD、centroidal dynamics），降低 NLP 維度
6. **Condensing vs Sparsity**：狀態維度高時用 sparse formulation（不消去 $x$），控制維度高時用 condensing（消去 $x$ 只留 $U$）
7. **最後手段**：縮短 $N$（但要驗證穩定性），或在外層跑慢速 NMPC + 內層跑快速線性 MPC 做 cascaded 架構

**面試官想聽到**：不是只會說「用更快的求解器」，而是能講出 RTI + warm start + 結構利用這三板斧的原理和組合策略。

</details>

<details>
<summary>Q3（難）：你在 sim 裡用 MPC 追蹤效果很好，但真機有 3 ms 通訊延遲 + 模型不確定性，軌跡偏離嚴重。怎麼補償？</summary>

**完整推理鏈**：

1. **延遲補償**：MPC 求解用的「當前狀態」$x_k$ 其實是 3 ms 前的。解法：用模型往前推 3 ms — 在 MPC 求解前，先用 $\hat{x}_{k+d} = f(x_k, u_{k-d:k})$ 把狀態預測到「MPC 解出結果時的真實時刻」
2. **模型修正**：加 Disturbance Observer (DOB)，把模型和實際的差異估計為一個外力 $\hat{d}$，加進 MPC 的預測模型 $x_{k+1} = f(x_k, u_k) + \hat{d}_k$
3. **魯棒 MPC**：用 tube-based MPC — 名義軌跡由 MPC 規劃，加一個線性 ancillary controller 把真實狀態拉回名義軌跡的管 (tube) 內
4. **約束收緊**：在約束上留 margin，$u_{\max}^{\text{MPC}} = u_{\max}^{\text{real}} - \Delta u_{\text{margin}}$，把不確定性的影響吸收在 margin 裡
5. **System ID**：跑掃頻訊號識別真實動力學參數，更新 MPC 的 $A, B$ 矩陣

**面試官想聯到**：延遲補償（state prediction）是最容易忽略也最關鍵的；DOB 是工業界最常用的模型修正手段；tube MPC 是學術界魯棒 MPC 的標準框架。

</details>

<details>
<summary>Q4（中）：團隊新人問「為什麼不直接用 LQR，MPC 多此一舉？」你怎麼解釋？</summary>

**完整推理鏈**：

1. **LQR 是 MPC 的特例**：LQR = 無限時域 + 無約束 + 線性模型的 MPC。LQR 的增益 $K = -R^{-1}B^TP$ 是離線算好的常數矩陣，在線只需矩陣乘法
2. **約束處理**：LQR 算出的 $u = Kx$ 不保證滿足 $u_{\min} \le u \le u_{\max}$。你可以在外面 clamp，但 clamp 後就不再是最佳的了。MPC 直接在最佳化中處理約束
3. **非線性系統**：LQR 需要在工作點線性化，遠離工作點就失效。MPC 可以用非線性模型（NMPC）
4. **時變參考**：LQR 追蹤固定 setpoint 沒問題，但追蹤時變軌跡需要做 time-varying LQR，實作複雜度接近 MPC
5. **結論**：簡單系統 + 無約束 + 定點追蹤 → LQR 夠用且更快。有約束 / 非線性 / 時變軌跡 → MPC 是正確選擇

**面試官想聽到**：能清楚講出 LQR 是 MPC 的特例，以及約束處理是 MPC 最核心的優勢。

</details>

## 面試角度

1. **時域 vs 算力權衡** — MPC 的 horizon $N$ 是最核心的設計旋鈕。面試時帶出：「$N$ 的選擇是穩定性和算力的 trade-off — $N$ 太短看不夠遠會不穩定，$N$ 太長 QP 規模變大算不完。實務上我用二分搜配合 closed-loop simulation 找到最短穩定 horizon，然後驗證求解時間是否進得去控制頻率。」

2. **凸鬆弛是工業落地的關鍵** — 區分「教科書 MPC」和「能上機的 MPC」。面試時帶出：「真實系統的約束很少天然是凸的 — 摩擦錐需要 SOCP 鬆弛、避碰約束需要凸包近似、非線性動力學需要逐步線性化 (SQP_RTI)。能把非凸問題轉成高效可解的凸問題，是 MPC 工程師的核心能力。」

3. **硬約束處理是 MPC 相比 PID 的根本優勢** — 這是面試最常被追問的點。帶出：「PID 只能在飽和後做 anti-windup 補償，本質是事後修。MPC 在規劃階段就把力矩上限、速度限制、摩擦錐全部納入最佳化，保證輸出永遠在安全範圍內 — 這在四足行走和自駕車裡是安全關鍵的。」

4. **延遲補償與魯棒性** — 把「我懂理論」升級為「我踩過 sim-to-real 的坑」。帶出：「Sim 裡完美的 MPC 到真機通常會崩，最常見的兩個原因是通訊延遲和模型不準。我的標準做法是在 MPC 前加 state prediction 補延遲，加 DOB 估計未建模動力學，約束上留 safety margin。」

5. **MPC 與 RL 的互補** — 展現前沿視野。帶出：「MPC 有模型但受限於模型精度，RL 不需要精確模型但缺乏安全保證。業界趨勢是用 RL 學殘差模型或直接學 MPC 的 warm start，或用 MPC 做 RL 的 safety filter (CBF-MPC)。」

## 延伸閱讀

- **Rawlings, Mayne & Diehl,《Model Predictive Control: Theory, Computation, and Design》** — MPC 理論聖經，Ch5（穩定性）和 Ch10（NMPC）是面試高頻考點
- **acados 官方文檔與範例** — 目前最快的開源 NMPC 框架，RTI + HPIPM 的工業標準實作
- **MIT Cheetah 3 MPC 論文 (Di Carlo et al., 2018)** — 四足 MPC 的經典架構：SRBD + 摩擦錐 QP，1 kHz 求解
- **CasADi 教學** — 符號建模 + NLP 求解的瑞士刀，學 MPC 必會的工具
- **OCS2 (Optimal Control for Switched Systems)** — ETH 開源框架，支援 switched-system MPC（四足步態切換），ROS 2 整合完善
- **論文《Neural Network-based Model Predictive Control》** — Learning MPC 的入門，用 NN 近似 MPC 策略 + CBF 安全過濾
- **OSQP 文檔** — 嵌入式 QP 求解器，支援 warm start，MIT Cheetah / Boston Dynamics 都在用
