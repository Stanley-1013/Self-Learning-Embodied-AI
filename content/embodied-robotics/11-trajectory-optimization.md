---
title: "軌跡最佳化（時間、能量、平滑度）"
prerequisites: ["10-basic-path-planning"]
estimated_time: 45
difficulty: 3
tags: ["trajectory", "optimization", "smoothness", "b-spline", "topp"]
sidebar_position: 11
---

# 軌跡最佳化（時間、能量、平滑度）

## 你將學到

- 能用兩句話區分 path 與 trajectory，面試時不混用
- 遇到「機械臂搬運太慢」或「焊接末端抖動」時，知道該用什麼目標函數（時間最優 / 能量最優 / 平滑度最優）、什麼參數化方法（多項式 / B-spline / S 曲線），以及為什麼三者不能同時極致
- 判斷何時需要 TOPP-RA 做時間最優重參數化，何時該用 minimum snap 做平滑軌跡

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

**最少夠用的數學**：

1. **三次多項式插值**（滿足端點 position + velocity 邊界條件）：

$$
q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3
$$

**物理意義**：4 個係數剛好解 4 個邊界條件（起點位置/速度、終點位置/速度），是最簡單的平滑軌跡。但加速度在端點不連續，可能產生力矩跳變。

2. **五次多項式**（再加 acceleration 邊界條件）：

$$
q(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5
$$

**物理意義**：6 個係數解 6 個邊界條件（起點和終點各有 position、velocity、acceleration）。加速度連續 → 力矩連續 → 機械衝擊更小。

3. **TOPP-RA 時間最優重參數化**（核心思想）：

$$
s(t): [0, T] \to [0, 1], \quad \dot{s} \geq 0
$$

$$
\text{subject to: } \tau_{\min} \leq M(q(s))\ddot{q}(s) + C(q,\dot{q})\dot{q}(s) + g(q(s)) \leq \tau_{\max}
$$

**物理意義**：把幾何路徑用弧長 $s$ 重新參數化，然後在 $(s, \dot{s})$ 相平面上找最快的速度曲線，使得每個瞬間的力矩都不超限。TOPP-RA 把這個問題轉成一連串小的 LP，$O(n)$ 複雜度，可以即時跑。

<details>
<summary>深入：多項式插值完整推導與矩陣求解</summary>

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

### Minimum jerk 軌跡（閉式解）

當 $v_0 = v_f = a_0 = a_f = 0$（靜止到靜止），minimum jerk 有解析解：

$$
q(t) = q_0 + (q_f - q_0)\left[10\left(\frac{t}{T}\right)^3 - 15\left(\frac{t}{T}\right)^4 + 6\left(\frac{t}{T}\right)^5\right]
$$

這是五次多項式的特例，jerk 積分最小。生物學上人類手臂的自然運動高度吻合這條曲線（Flash & Hogan, 1985）。

### 多段 waypoint 插值

$n$ 個 waypoint 之間用 $n-1$ 段三次多項式，相鄰段在交接處要求位置、速度、加速度連續，組成 tridiagonal 線性系統，$O(n)$ 可解（Thomas algorithm）。

</details>

**常見參數化方法比較**：

| 方法 | 連續性 | 局部控制 | 優點 | 缺點 |
|------|--------|---------|------|------|
| 三次多項式 | $C^1$ | 否 | 簡單、解析解 | 加速度不連續 |
| 五次多項式 | $C^2$ | 否 | 加速度連續 | 高階震盪（多段時） |
| 梯形速度 | $C^0$（速度） | 是 | 工業標準、好算 | jerk 突變（無限大） |
| S 曲線（7 段） | $C^1$（加速度） | 是 | jerk 有限、消除衝擊 | 參數多、計算較複雜 |
| B-spline | $C^{k-1}$ | **是** | 局部修改不影響整體 | 需要選 knot vector |
| Minimum snap | $C^3$ | 否 | 四旋翼標準 | 計算量隨 waypoint 增長 |

<details>
<summary>深入：B-spline 與 TOPP-RA 演算法細節</summary>

### B-spline 軌跡

$k$ 階 B-spline 曲線由控制點 $P_i$ 和 knot vector $\{t_0, t_1, \ldots, t_{n+k}\}$ 定義：

$$
q(t) = \sum_{i=0}^{n} N_{i,k}(t) P_i
$$

其中 $N_{i,k}(t)$ 是 B-spline basis function（Cox-de Boor 遞推）。

**核心性質**：
- **局部控制**：移動一個控制點只影響 $k$ 個 knot span 內的曲線（不像全域多項式會龍格震盪）
- **凸包性**：曲線段落在控制點的凸包內（collision checking 友善）
- **微分簡單**：$k$ 階 B-spline 的導數是 $k-1$ 階 B-spline，直接操作控制點差分

**軌跡最佳化中的用法**：把控制點當決策變數，用 QP 或 NLP 最小化 $\int \|\ddot{q}\|^2 dt$ 或 $\int \|\dddot{q}\|^2 dt$，同時加入速度/加速度/避障約束。

### TOPP-RA（Time-Optimal Path Parameterization via Reachability Analysis）

**問題設定**：給定幾何路徑 $q(s), s \in [0,1]$，找時間最優的 $s(t)$ 使得：

$$
\min \int_0^T dt \quad \text{s.t.} \quad \tau_{\min} \leq M(q)\left[q''\dot{s}^2 + q'\ddot{s}\right] + C\, q'\dot{s}^2 + g \leq \tau_{\max}
$$

其中 $q' = dq/ds$。

**關鍵變數代換**：令 $x = \dot{s}^2$，則 $\ddot{s} = \frac{1}{2}\frac{dx}{ds}$，約束變成 $x$ 和 $dx/ds$ 的**線性**不等式。

**演算法步驟**：
1. 離散化 $s$ 為 $s_0, s_1, \ldots, s_K$
2. 在每個 $s_i$，由動力學約束算出 $x$ 的上下界
3. 從 $s_K$ backward 傳播 reachability set
4. 從 $s_0$ forward 積分，取最大可行 $x$

複雜度 $O(K \cdot n_{\text{joints}})$，實務上 $K=100$ 即可，亞毫秒可算完。

**對比 TOPP（原版）**：TOPP 用 ODE 積分 + switch point 搜尋，數值不穩定；TOPP-RA 用 LP reachability 分析，更穩健且保證全域最優。

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

**模擬器觀察**：在 MuJoCo 或 Isaac Sim 裡，給同一段 PTP 運動分別用梯形速度和 S 曲線，觀察：(1) 末端軌跡的平滑程度（zoom in 看微小振動），(2) 關節力矩曲線（梯形的力矩有明顯跳變），(3) 末端在目標附近的 settling time（S 曲線因為沒激發高頻共振，收斂更快）。

## 實作連結

**三個典型工程場景**：

1. **工業 PTP 搬運**：需要最短 cycle time，但力矩不能超限。用 TOPP-RA 對 path planner 給的 waypoints 做時間最優重參數化。先用 RRT/PRM 算幾何路徑，再用 TOPP-RA 注入時間。

2. **焊接/塗膠直線運動**：末端必須沿笛卡爾直線走，且速度均勻、不能抖。用 minimum jerk 或 B-spline 參數化，加 jerk 上限約束。注意笛卡爾直線在關節空間可能經過奇異點 → 要檢查 Jacobian condition number。

3. **四旋翼航點飛行**：waypoints 間用 minimum snap（四階微分最小）軌跡，因為 snap 直接對應推力的變化率。經典做法是把問題轉成 QP，決策變數是每段多項式係數。

**Code 骨架**（Python，TOPP-RA）：

```python
import toppra as ta
import numpy as np

# 1. 定義幾何路徑（關節空間 waypoints）
waypoints = np.array([[0, 0, 0, 0, 0, 0],    # 起點
                       [1.0, -0.5, 0.8, 0, 0, 0],  # 中間
                       [1.5, -1.0, 1.2, 0, 0, 0]])  # 終點
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
    """
    五次多項式軌跡：滿足 6 個邊界條件
    q0, qf: 起終點位置
    v0, vf: 起終點速度
    a0, af: 起終點加速度
    T: 總時間
    """
    # 解 6x6 線性系統
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
    q = np.polyval(coeffs[::-1], ts)    # 注意 polyval 要反轉係數順序
    # 更精確的做法：手動展開避免 Horner 精度問題
    q = sum(coeffs[i] * ts**i for i in range(6))
    qd = sum(i * coeffs[i] * ts**(i-1) for i in range(1, 6))
    qdd = sum(i * (i-1) * coeffs[i] * ts**(i-2) for i in range(2, 6))
    qddd = sum(i * (i-1) * (i-2) * coeffs[i] * ts**(i-3) for i in range(3, 6))

    return ts, q, qd, qdd, qddd


def minimum_jerk(q0, qf, T, dt=0.001):
    """
    Minimum jerk 軌跡（靜止到靜止的閉式解）
    Flash & Hogan, 1985
    """
    ts = np.arange(0, T + dt, dt)
    tau = ts / T  # 正規化時間 [0, 1]

    # 位置
    s = 10 * tau**3 - 15 * tau**4 + 6 * tau**5
    q = q0 + (qf - q0) * s

    # 速度
    sd = (30 * tau**2 - 60 * tau**3 + 30 * tau**4) / T
    qd = (qf - q0) * sd

    # 加速度
    sdd = (60 * tau - 180 * tau**2 + 120 * tau**3) / T**2
    qdd = (qf - q0) * sdd

    # Jerk
    sddd = (60 - 360 * tau + 360 * tau**2) / T**3
    qddd = (qf - q0) * sddd

    return ts, q, qd, qdd, qddd


# 範例：比較五次多項式與 minimum jerk
if __name__ == "__main__":
    T = 2.0
    ts1, q1, _, _, jerk1 = quintic_trajectory(0, 1, 0, 0, 0, 0, T)
    ts2, q2, _, _, jerk2 = minimum_jerk(0, 1, T)

    fig, axes = plt.subplots(2, 1, figsize=(10, 6))
    axes[0].plot(ts1, q1, label="Quintic (zero BC)")
    axes[0].plot(ts2, q2, "--", label="Min Jerk")
    axes[0].set_ylabel("Position")
    axes[0].legend()

    axes[1].plot(ts1, jerk1, label="Quintic jerk")
    axes[1].plot(ts2, jerk2, "--", label="Min Jerk jerk")
    axes[1].set_ylabel("Jerk")
    axes[1].set_xlabel("Time (s)")
    axes[1].legend()

    plt.tight_layout()
    plt.savefig("trajectory_comparison.png", dpi=150)
    plt.show()
```

**注意**：靜止到靜止條件下，五次多項式（$v_0=v_f=a_0=a_f=0$）和 minimum jerk 的解**完全一致** — 都是那條 $10\tau^3 - 15\tau^4 + 6\tau^5$ 曲線。差異出現在有非零邊界條件時。

</details>

## 常見誤解

1. **「時間最優 + 能量最優可以同時達到」** — 錯。時間最優要求加速度打滿（bang-bang 控制），力矩峰值大、能耗高；能量最優傾向慢慢來、力矩小。兩者在 Pareto 前緣上是 trade-off 關係。實務上用加權目標函數 $J = w_1 T + w_2 \int \tau^2 dt$ 取折衷。

2. **「多項式階數越高軌跡越好」** — 錯。高階多項式容易出現**龍格震盪**（Runge's phenomenon）：中間段產生大幅振盪。B-spline 用分段低階多項式 + 局部控制解決這問題。實務經驗：超過 7 階的全域多項式幾乎不用。

3. **「笛卡爾空間的直線路徑在關節空間也是平滑的」** — 錯。笛卡爾直線經過 IK 轉到關節空間後，如果路徑靠近**奇異構型**（singular configuration），Jacobian 接近秩虧 → 關節速度會爆掉。**避開**：走笛卡爾直線前先掃描 Jacobian condition number $\kappa(J)$，若 $\kappa > 100$ 就繞路或改用關節空間插值。

4. **「只做運動學限幅就夠了」** — 錯。只限 $\dot{q}_{\max}$ 和 $\ddot{q}_{\max}$ 不考慮動力學（質量、Coriolis、重力），算出來的軌跡可能需要超出馬達額定的力矩。TOPP-RA 的價值就是把**動力學約束**（$\tau_{\min} \leq \tau \leq \tau_{\max}$）納入考量。

## 練習題

<details>
<summary>Q1：工廠 pick-and-place 線 cycle time 太長，老闆要求縮短 20%，但馬達力矩已經接近極限，你會怎麼做？</summary>

**完整推理鏈**：

1. **確認瓶頸**：先看目前用什麼軌跡規劃 — 如果是梯形速度，加速度限值可能設得太保守。錄下關節力矩曲線，看峰值離真實極限還有多少 margin
2. **TOPP-RA 重參數化**：把現有路徑丟進 TOPP-RA，加入真實的力矩約束（不是運動學限值），自動找時間最優。通常比手調梯形參數快 15-30%
3. **路徑也要改**：如果 TOPP-RA 已經打滿力矩了還不夠快，問題出在幾何路徑太長。用 RRT-Connect + shortcutting 或 CHOMP 重新規劃更短的路徑
4. **避開的陷阱**：不要只加大加速度限值 — 會超力矩、損壞減速機。也不要忽略 jerk — 即使力矩沒超，jerk 太大會激發諧波振動，positioning 精度下降
5. **最終驗證**：在模擬器裡跑完整 cycle，確認力矩 headroom ≥ 10%，jerk 在可接受範圍，settling time 沒惡化

**結論**：TOPP-RA + 路徑優化雙管齊下，先壓時間、再確認力矩和 jerk 安全。

</details>

<details>
<summary>Q2：焊接機械臂沿直線焊接時末端抖動嚴重，焊道品質不合格，你的分析流程？</summary>

**完整推理鏈**：

1. **先看頻率**：用 accelerometer 或 encoder 高頻取樣分析末端振動頻譜。低頻（< 10 Hz）→ 軌跡問題；高頻（> 50 Hz）→ 結構共振或 controller 問題
2. **軌跡面向**：
   - 檢查是否用了梯形速度 → jerk 無限大會激發共振 → 換 S 曲線或 minimum jerk
   - 檢查是否在笛卡爾空間插值且靠近奇異構型 → Jacobian condition number 飆高 → 關節速度震盪 → 加 $\kappa(J)$ 監控，遠離奇異或加 damped least squares
   - 檢查 waypoint 間距是否太大導致插值誤差 → 加密 waypoint
3. **控制面向**：PD gain 太高會放大高頻噪音；加 low-pass filter 在速度命令上
4. **結構面向**：檢查減速機背隙、連桿剛性不足
5. **避開的陷阱**：不要一味降速度來壓抖動 — 這只是遮蓋問題。要找到根因（jerk 突變 / 奇異 / 控制器增益）對症下藥

**結論**：從頻譜分析定位根因，軌跡上用 min jerk + B-spline 降低高頻激發，同時確認奇異點安全距離。

</details>

<details>
<summary>Q3：Nav2 導航機器人轉彎時甩尾、輪子打滑，DWA local planner 的軌跡看起來轉得太急，怎麼調？</summary>

**完整推理鏈**：

1. **參數層面**：
   - 降低 `max_vel_theta`（最大角速度）和 `max_accel_theta`（最大角加速度）→ 直接限制轉彎激烈程度
   - 降低 `goal_heading` cost weight → DWA 不會為了快速朝向目標而急轉
   - 提高 `path_distance` cost weight → 優先貼著全域路徑走，而不是走捷徑急轉
2. **軌跡層面**：DWA 本身的軌跡是短時 rollout（~1-2 秒），天生就比較 myopic。如果問題根源是全域路徑的轉角太銳，需要在 global planner 層用 B-spline smoothing 把 waypoint 轉角圓滑化
3. **物理層面**：確認輪子摩擦係數 — 如果地面光滑，即使命令的角速度在名義上可行，實際也會打滑。在 URDF 的 `<friction>` 或導航參數裡加 safety margin
4. **避開的陷阱**：不要只調 DWA 的 `sim_time`（rollout 長度）想解決轉彎問題 — 延長 rollout 會增加計算量但不一定改善轉彎平滑度

**結論**：降低角速度/角加速度上限 + 全域路徑 B-spline 平滑 + 確認物理摩擦 margin。

</details>

<details>
<summary>Q4：四旋翼要穿越 5 個 waypoints 做航拍，要求飛行平順且能追蹤 reference，你會選什麼軌跡表示法？為什麼？</summary>

**完整推理鏈**：

1. **選 minimum snap（四階微分最小化）**：四旋翼的推力 $f$ 正比於加速度（$f = m\ddot{x} + mg$），推力的變化率正比於 jerk，推力變化率的變化率正比於 **snap**。最小化 snap → 最小化推力的高階變動 → 電機轉速變化最平滑 → 飛行最穩
2. **數學形式**：每段用 7 次多項式（8 個係數），5 個 waypoints → 4 段 → 決策變數 = 4×8×3（xyz 三軸）。在 waypoint 處要求 position、velocity、acceleration、jerk 連續。整體轉成 QP 求解
3. **時間分配**：segment 時間 $T_i$ 通常先用 trapezoidal velocity profile 估初值，再用 NLP 聯合最佳化（Richter et al., 2016）
4. **和 B-spline 的對比**：minimum snap 是全域最優但計算量隨 waypoint 增加；B-spline 局部控制、適合大量 waypoint 和 online replanning。航拍 5 個 waypoint → minimum snap 足夠
5. **避開的陷阱**：忘記考慮 thrust limit → snap 最小化了但某段加速度超出推力上限 → 需要加不等式約束或用 TOPP-RA 做後處理

**結論**：minimum snap + QP 求解，配合時間分配最佳化和推力上限約束。

</details>

## 面試角度

1. **Path vs Trajectory 的精確區分** — 這是最基本但最常被混用的概念。面試時用這句話帶出：「Path 只有幾何，沒有時間；trajectory 是 path 加上時間參數化，產出 $q(t), \dot{q}(t), \ddot{q}(t)$ 三條曲線。軌跡最佳化就是在物理約束下找最佳的時間分配。」

2. **Pareto trade-off：時間 vs 能量 vs 平滑度** — 展現你理解多目標最佳化的本質，不是天真地追求單一指標。面試時帶出：「時間最優要 bang-bang、能量最優要慢慢來、平滑度最優要壓 jerk — 三者在 Pareto 前緣上互斥，實務上用加權目標或約束式來取折衷。」

3. **B-spline 局部控制性** — 說明你理解高階全域多項式的龍格震盪問題。面試時帶出：「用 B-spline 而不是高階多項式，因為修改一個控制點只影響局部幾個 knot span，不會像全域多項式那樣產生龍格震盪。」

4. **笛卡爾空間的奇異陷阱** — 從軌跡最佳化延伸到 Jacobian 的邊界敏感度。面試時帶出：「笛卡爾直線看起來最短，但經過奇異構型時 Jacobian 秩虧，關節速度會爆掉。我會在軌跡規劃時加 $\kappa(J)$ 上限約束，或在接近奇異時自動切換到關節空間插值。」

5. **TOPP-RA 的工程價值** — 展現你知道現代時間最優重參數化的標準工具。面試時帶出：「TOPP-RA 把動力學約束轉成 $(s, \dot{s})$ 相平面上的 LP 序列，$O(n)$ 複雜度、亞毫秒可算，比傳統 TOPP 更穩健且保證全域最優。」

## 延伸閱讀

- **Pham & Pham, "TOPP-RA: Time-Optimal Path Parameterization for Redundantly Actuated Robots" (2018)** — TOPP-RA 的原始論文，演算法清晰、有開源 Python 實作 `toppra`，是工業界時間最優的標準參考
- **Mellinger & Kumar, "Minimum Snap Trajectory Generation and Control for Quadrotors" (2011)** — 四旋翼 minimum snap 的經典論文，把軌跡規劃轉成 QP，至今仍是無人機圈標準
- **Flash & Hogan, "The Coordination of Arm Movements" (1985)** — Minimum jerk 假說的源頭論文，解釋為什麼人類手臂自然運動遵循五次多項式
- **Lynch & Park,《Modern Robotics》Ch9 Trajectory Generation** — 多項式、梯形速度、S 曲線的系統性教材，MIT OCW 有完整影片
- **`toppra` Python 套件（GitHub: hungpham2511/toppra）** — TOPP-RA 的官方開源實作，直接可用在 ROS 2 + MoveIt 的 pipeline 裡
- **《具身智能算法工程師 面試題》Ch5.1 軌跡插值 + Ch5.3 時間最優** — 中文面試考點整理，搭配本章閱讀效果最佳
