# Extracted v2: Ch11 軌跡最佳化 — 5 dedicated queries (from scratch)

## Q1 補強（Polynomial + Min-snap/Diff-flat + TrajOpt 框架）

### 情境題 A：多項式軌跡基礎（Cubic / Quintic / Septic + LSPB）
- **Linear Trajectory 災難**：
  - 速度突變 → 加速度呈 Dirac δ（無窮大尖峰）
  - `τ = M(q)·q̈` 要求馬達瞬間無窮扭矩 → 擊穿電流 + 減速機毀滅性衝擊
- **Cubic Polynomial**：`q(t) = a₀ + a₁t + a₂t² + a₃t³`
  - 4 個未知 = 滿足 q(0), q(t_f), v(0), v(t_f)
  - **缺陷**：加速度 `q̈ = 2a₂ + 6a₃t` 線性變化 → 軌跡切換點 jerk 階躍不連續 → 微小震顫
- **Quintic Polynomial 工業經典**（6 參數，保 q, v, a 連續）：
  - **jerk 有限且連續** → 徹底消機械衝擊 → PTP 點到點運動黃金標準
  - Jerk-minimization 的解析解等價於 quintic
- **Septic Polynomial (7 次)**（8 參數）：
  - 保 jerk 連續、snap 有限
  - CNC 數控、超高精度雷射切割必用（微小 jerk 突變會激發機床共振產生水波紋）
- **LSPB (Linear Segments with Parabolic Blends)**：
  - PLC 算力有限，5 次多項式太耗時
  - 中間勻速（Linear 最快）+ 起終點拋物線（恆定加速度）
  - **梯形速度曲線 (Trapezoidal Velocity Profile)**，計算極快
- **Runge 現象陷阱**（面試必答）：
  - 10 via-points **不要**用一個 9 次多項式一次穿過
  - 高次多項式邊緣劇烈震盪 → 機器人瘋狂扭動
  - **正解**：分段低次多項式插值（如 B-Spline）在拼接點強加 C² 連續約束
- **Quintic 求解 Python**：
  ```python
  A = np.array([
      [1,0,0,0,0,0], [0,1,0,0,0,0], [0,0,2,0,0,0],
      [1,T,T**2,T**3,T**4,T**5],
      [0,1,2*T,3*T**2,4*T**3,5*T**4],
      [0,0,2,6*T,12*T**2,20*T**3]
  ])
  b = np.array([q0, v0, a0, qf, vf, af])
  coeffs = np.linalg.solve(A, b)
  ```

### 情境題 B：Min-jerk / Min-snap + Differential Flatness（無人機基石）
- **Min-jerk 與 Flash-Hogan 理論 (1985)**：
  - 神經科學發現人類手臂伸手抓取優化的不是時間最短也不是能量最小
  - **而是「加加速度 (Jerk) 的平方積分最小」**
  - 生物學上最平滑 + 關節磨損最小的自然軌跡
- **Min-snap Trajectory (Mellinger-Kumar 2011)**（無人機最重要論文之一）：
  - 四旋翼是欠驅動系統
  - **物理鏈條**：位置 p → `p̈` 加速度（螺旋槳推力 + 機身傾角）→ `p⃛` Jerk（角速度）→ `p⁽⁴⁾` Snap（**馬達控制扭矩變化率**）
  - **Min-snap = Min 馬達推力劇烈波動** → 高速穿窗框時姿態「絲滑」不失控
- **Differential Flatness 數學魔法**：
  - 四旋翼 12 狀態 `[x,y,z,ẋ,ẏ,ż,φ,θ,ψ,p,q,r]` + 4 馬達輸入
  - **證明四旋翼是 Differentially Flat System**
  - 4 個 Flat Outputs `[x, y, z, ψ]`（3D 位置 + 偏航角）**以純代數方程完全推出所有 12 狀態 + 4 控制**
  - **突破**：規劃器不需痛苦非線性動力學積分；**純幾何 3D 空間規劃平滑 x,y,z 曲線 = 馬達推力指令**
- **BVP + Min-snap 解析解**：
  - 軌跡多段 7-8 次多項式參數化
  - Min-snap 目標 `∫(p⁽⁴⁾)² dt` → QP `min c^T Q c`
  - 加 Waypoint 位置 + C³ 連續約束 `A_eq·c = b_eq`
- **面試 talking point「Min-snap + Diff-flat 是無人機軌跡 2011 後基石」**：
  - 傳統 MPC 12 維非線性空間算力爆炸
  - **Differential Flatness 實現完美降維打擊**，動力學約束完全解耦
  - 3D QP 算出 Min-snap 多項式曲線 → **這條幾何曲線自帶動力學可行性**，無人機絕對飛得出來
  - Fast-Planner 和 Drone Racing 的底層靈魂
- **QP 矩陣構建 C++**：
  ```cpp
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(num_coeffs, num_coeffs);
  for (int i = 4; i < num_coeffs; ++i) {     // Snap 從 4 階開始
      for (int j = 4; j < num_coeffs; ++j) {
          double cost_coeff = (factorial(i)/factorial(i-4)) *
                              (factorial(j)/factorial(j-4));
          Q(i, j) = cost_coeff * pow(T, i+j-7) / (i+j-7);
      }
  }
  // 呼叫 OSQP/OOQP 求解 min c^T Q c s.t. Ac = b
  ```

### 情境題 C：TrajOpt 三框架（CHOMP / STOMP / TrajOpt）
- **CHOMP (Covariant Hamiltonian Optimization)**：
  - 軌跡離散化為 waypoints 向量 ξ
  - 梯度下降 `ξ_{i+1} = ξ_i - η·A⁻¹·∇U(ξ)` 優化平滑度 + 避障
  - **"Covariant" 關鍵**：A⁻¹ 是基於軌跡差分的黎曼度量矩陣
  - 把能量平滑散佈整條軌跡 + 不受參數化點密度影響
- **STOMP (Stochastic Trajectory Optimization)**：
  - **不需計算梯度**
  - 在參考軌跡周圍加平滑高斯雜訊 → 生成候選軌跡 batch (Rollouts)
  - 指數加權（類 Softmax / MPPI）平均出更優軌跡
  - **優勢**：CHOMP 需 SDF 梯度；自碰撞 / 離散不可微成本梯度為 0 或無窮大
  - STOMP 靠隨機採樣完美處理 **Non-smooth / Non-differentiable Cost**
- **TrajOpt (UC Berkeley, John Schulman)**：
  - **嚴格 SQP** 問題
  - 連續時間碰撞檢測
  - **Convexification 凸化**：障礙 SDF 一階泰勒展開 → 線性不等式約束
  - 可信賴域 (Trust Region) 內迭代求解
  - **硬約束王者**：強制 `SDF(x) > d_safe` → 只要 SQP 有解軌跡絕對無碰撞
  - vs CHOMP 把避障當軟約束可能擦撞
- **場景選型**（面試）：
  - **擁擠書架取書** → **TrajOpt**（SQP + 凸化 SDF 處理硬約束，狹窄不穿模）
  - **無法微分的成本**（避開相機遮擋視野、避液體潑灑）→ **STOMP**（暴力採樣）
  - **CHOMP**：多作為 OMPL 粗軌跡（RRT*）的快速平滑後處理器，共變梯度毫秒內把折線拉平
- **TrajOpt SQP 泰勒展開**：`dist(x_i) ≈ dist(x_{i,0}) + J_dist(x_{i,0})·Δx_i ≥ d_safe`
- **MoveIt! 2 YAML**：
  ```yaml
  planning_pipelines:
    pipeline_names: [ompl, chomp, stomp]
  ompl:
    request_adapters: |
      default_planner_request_adapters/AddTimeOptimalParameterization
      default_planner_request_adapters/CHOMPOptimizingAdapter
  ```

### 面試 talking points（Q1）
1. **Quintic jerk 連續 = 工業點到點黃金標準**：能講清楚為什麼不用 cubic
2. **Runge 現象 → B-Spline 分段低次**：面試多 via-points 陷阱的正解
3. **Differential Flatness 降維打擊**：無人機軌跡規劃的靈魂，面試無人機必備
4. **Min-snap 對應馬達推力變化率**：物理鏈 `p → p̈ → p⃛ → p⁽⁴⁾` 要背熟
5. **TrajOpt SDF 凸化 = 硬約束王者**：面試機械臂狹窄空間操作的標準答案
6. **STOMP 非光滑成本友善**：分辨「只背 CHOMP」vs「懂三框架差異」

## Q2 補強（Time-Optimal + Contact-Aware + Learning-based）

### 情境題 D：TOPP-RA (Time-Optimal Path Parameterization)
- **「先幾何 + 後時間」工業標準**：
  - Single-shot (q + t 同為決策變數) = 非凸問題 → 求解慢 + 易發散
  - 解耦：RRT*/CHOMP 找幾何路徑 `q(s)` → TOTP 算最優 `ṡ, s̈`
- **TOPP 核心數學魔法**：
  - `q̇ = q'(s)·ṡ`, `q̈ = q''(s)·ṡ² + q'(s)·s̈`
  - 定義 `u = s̈`, `v = ṡ²`
  - **原非線性動力學 `τ = M·q̈ + C·q̇ + G` 變 u/v 的線性方程** `τ(s) = A(s)·u + B(s)·v + C(s)`
- **TOPP-RA (Pham 2018)**：
  - 軌跡切 N 段；向後 / 向前兩遍掃描
  - 轉化為一系列極小規模 LP 或 QP
  - **O(N) 極速求解，1ms 內**
- **Bang-Bang 原則**（龐特里亞金極小值原理）：
  - 時間最優必然 Bang-Bang
  - 每瞬間**至少一個關節力矩達物理極限**
  - 「滿載加速 → 滿載勻速 → 滿載減速」
- **Min-snap + TOTP vs Single-shot**：
  - Single-shot 非凸無即時保障
  - Min-snap + TOPP-RA 是**降維打擊完美組合**：Min-snap 純幾何保極致平滑；TOPP-RA 精確壓榨馬達扭矩潛力
  - 凸 + 快 + 工程安全可控
- **Drake TOPP-RA Python**：
  ```python
  from pydrake.all import Toppra
  toppra = Toppra(geometric_path, plant)
  toppra.AddJointVelocityLimit(v_min, v_max)
  toppra.AddJointAccelerationLimit(a_min, a_max)
  toppra.AddJointTorqueLimit(tau_min, tau_max)
  time_optimal_traj = toppra.SolvePathParameterization()  # 1ms 內
  ```

### 情境題 E：Contact-Aware TrajOpt（Manipulation + Locomotion 統一）
- **CITO Through-contact manipulation**（推箱子、轉手、拉抽屜）：
  - **KKT 拉格朗日乘子 λ 物理湧現為接觸力**
  - 硬約束 `ϕ(q) ≥ 0`（不穿透）→ 求解器生成 λ
  - 互補約束 `ϕ(q)·λ = 0` → 規劃器自己決定何時 ϕ=0 換 λ>0 推物體
- **四足 Footstep + Swing Trajectory**：
  - Footstep 位置：倒立擺 (LIPM) 或 Capture Point 理論保質心穩定
  - **Swing 軌跡用 Bezier 曲線的原因**：
    - **凸包性 (Convex Hull Property)**：整條曲線必被控制點的凸多邊形包裹
    - 只要約束中間控制點**高於地面障礙** → 整條擺動腿軌跡絕不絆倒
    - 避障約束簡化為**幾個控制點的線性不等式**
- **Centroidal Momentum-aware Optimization (CMM)**（Atlas 後空翻等）：
  - 30+ 關節直接規劃易發散
  - **CMM 降維到質心 6D 動量空間**
  - 只規劃 CoM 軌跡 `p_c` 和角動量 `k`
  - 約束 `k̇ = Σ(p_i - p_c) × f_i` → 精確算出足底 GRF 產生後空翻旋轉力矩
- **質心角動量動態**：
  - `ḣ_G = [m·c̈; k̇_G] = [Σf_i - mg; Σ(p_i - c) × f_i + τ_i]`
- **Contact-aware 統一 Manipulation/Locomotion**（面試經典）：
  - Atlas 後空翻 + 機械臂推箱子 **是同一個方程**
  - 都是 **Underactuated Dynamics** 問題
  - 無法直接控箱子或空中質心，只能**尋找最優接觸點 + 接觸力 `J^T·f_c` 間接改狀態**
  - 掌握 Contact-Implicit → 足式 + 靈巧手兩大領域同時打通
- **Drake MathematicalProgram MPCC**：
  ```python
  prog.AddConstraint(phi >= 0)
  prog.AddConstraint(lam >= 0)
  prog.AddConstraint(phi * lam <= 1e-4)  # MPCC 鬆弛利於 IPOPT
  ```

### 情境題 F：Learning-based Trajectory Generation 2024
- **Motion Planning Diffusion (MPD)**：
  - 軌跡當「圖片去噪」問題
  - 初始化高斯雜訊軌跡
  - **Conditional Diffusion**：起終點 / LiDAR 點雲 / 自然語言 ("繞過杯子") 作 condition 注 UNet
  - 迭代去噪坍縮出平滑軌跡
- **Guided Diffusion with Constraints**：
  - 單純擴散可能穿模
  - 每步去噪加物理 Cost 梯度：`x_{t-1} ← x_{t-1} - α·∇_x C(x_{t-1})`
  - C(x) = SDF 碰撞懲罰
  - **推離障礙確保物理可行**
  - 公式：`ε̂ = ε_θ(x_t, ∅) + w·(ε_θ(x_t, c) - ε_θ(x_t, ∅)) + λ·∇_x C(x_t)`
- **Trajectory Transformer / Decision Transformer**：
  - (s, a, R) 量化成 Token：`[R_1, s_1, a_1, R_2, s_2, a_2, ...]`
  - GPT 式 causal attention 預測下一 token
  - 推理輸入高回報目標 `R_target = 1` → 自迴歸吐最優動作軌跡
- **VLA (RT-2/OpenVLA/π0) Trajectory Head**：
  - VLA 輸出接 **Action Chunking Head**（MDN 或 Diffusion Head）
  - 不輸出單步 action，**直接輸出未來 1-2 秒連續空間 Waypoints**
- **ALOHA ACT (Action Chunking + Temporal Ensemble) 靈巧操作 workhorse**：
  - 遙操作數據充滿手部抖動停頓（高頻雜訊）
  - 傳統單步 BC 微誤差幾何放大
  - **一次生成未來 k 步 chunk + 每步對重疊片段指數加權移動平均 (Temporal Ensemble)**
  - 時間維度低通濾波 → 極平滑
  - **穿針、打雞蛋等精細操作唯一解**
- **Temporal Ensemble Python**：
  ```python
  all_predictions = np.zeros((T, T, action_dim))
  for t in range(T):
      chunk = act_policy(obs)  # 一次 k 步
      all_predictions[t, t:t+k] = chunk
      valid = all_predictions[:, t, :]  # 過去所有對 t 的預測
      weights = np.exp(-np.arange(len(valid)) / decay)
      smooth_action_t = np.sum(valid * weights / weights.sum(), axis=0)
  ```
- **面試 talking point「Symbol → Neural」**：
  - 不再碰撞檢測盲搜
  - **Diffusion Model「想像」專家軌跡 → MPC / 阻抗控制器消化殘差**
  - 解決的不是精度問題，是**「開放世界人類直覺泛化」**

### 面試 talking points（Q2）
7. **TOPP-RA u = s̈, v = ṡ² 線性化**：能背出為什麼非線性動力學變線性，分辨「懂數學」
8. **Bang-Bang 原則**：時間最優必有關節滿載，工業壓榨硬體極限的標準答案
9. **Bezier 凸包性保腳不絆**：四足 Swing 軌跡的優雅設計
10. **KKT 拉格朗日乘子 = 接觸力物理湧現**：Contact-Implicit 的數學魔法
11. **Atlas 後空翻與機械臂推箱子同一方程**：展示系統觀的必殺題
12. **Diffusion Guided + SDF 梯度**：神經生成 + 物理約束混合的現代架構
13. **ALOHA Temporal Ensemble 時間低通**：靈巧操作 workhorse 的平滑魔法

## Q3 補強（iLQR/DDP + Multi-phase + Trajectory Rollout 橋接）

### 情境題 G：iLQR / DDP + Shooting vs Collocation
- **iLQR Bellman 反向遞推**：標稱軌跡上線性化 + 代價二次化
  - Backward Pass: `V(x_k) = min_u [l(x,u) + V_{k+1}(f(x,u))]`
  - 計算 Q-function 二次展開 `(Q_x, Q_u, Q_xx, Q_uu, Q_ux)`
  - 求最優控制律：`δu_k = k_k + K_k·δx_k`（前饋 + 反饋）
- **DDP 比 iLQR 多的二階項**：
  - iLQR 假設只用一階 Jacobian `(f_x, f_u)`（高斯-牛頓近似）
  - DDP 嚴格保留動力學二階 Hessian 張量 `∂²f/∂x²`
  - 物理意義：DDP 感知世界「曲率」（旋轉向心力非線性），極動態動作收斂更快（二次收斂）但二階張量耗時巨大
  - 實務：iLQR 性價比最高
- **三種構法 Trade-off**：
  - **Single Shooting**：只把 u_{0:T} 當決策；x_t 靠物理引擎硬積分
    - 尾端狀態對初始控制極敏感，梯度爆炸
  - **Multiple Shooting**：時間切 N 段，x_k + u_k 都當決策 + 連續性等式約束 `x_{k+1} - integrate(x_k, u_k) = 0`
    - 兼具積分準確 + 數值穩定
  - **Direct Collocation**：所有時間點 (x_k, u_k) 全當決策 → 動力學變相鄰代數等式
    - Hermite-Simpson 插值
- **為什麼 Atlas 後空翻用 Collocation 而不是 Shooting**：
  - Shooting：優化器給錯一點力矩 → x(t) 積分到半空炸飛 → 梯度發散找不到
  - **Collocation 允許求解器迭代初期違反物理定律 (Defect ≠ 0)**：先讓機器人「飄」在空中完成翻轉幾何，慢慢收緊動力學約束
  - 「同時在幾何與物理空間尋優」的穩定性遠超 Shooting
- **場景選型**：
  - **微秒級在線 MPC + 簡單非凸** → iLQR（不依賴 NLP solver，自寫 C++ 矩陣乘 1kHz）
  - **Atlas 後空翻等離線大尺度跨相態極限動作** → Direct Collocation + IPOPT（慢但非線性約束探索強 + 數值穩定）
- **Casadi Multiple Shooting C++**：
  ```python
  opti = ca.Opti()
  X = opti.variable(nx, N+1); U = opti.variable(nu, N)
  for k in range(N):
      x_next = runge_kutta_4(X[:,k], U[:,k])
      opti.subject_to(X[:,k+1] == x_next)  # Gap closing
      opti.subject_to(opti.bounded(u_min, U[:,k], u_max))
  opti.minimize(cost); opti.solver('ipopt'); opti.solve()
  ```

### 情境題 H：Multi-phase / Hybrid Trajectory Optimization
- **Phase 切換 TrajOpt**：P₁ 雙腳支撐 → P₂ 騰空 → P₃ 單腳落地
  - 動力學不同：P₁ 受 GRF 驅動；P₂ GRF=0 退化為重力拋體+旋轉剛體
  - **相鄰 Phase 必須加狀態連續性約束** `x(P₁, end) = x(P₂, start)`
- **Mode Sequence vs Contact-Implicit**：
  - **Mode Sequence**：人為寫死「0.5s 起跳、1.2s 落地」→ 計算快，MIT Cheetah 3 早期跳躍
  - **Contact-Implicit**：優化器自發現切換時刻，但 MPCC 鬆弛難收斂
- **騰空相角動量守恆約束**：
  - Flight phase 無外力矩 → 牛頓-歐拉 `k̇_com = Σr × F_grf = 0`
  - TrajOpt 騰空節點強加等式 `k̇_com(t) = 0` 或 `k_com(t) = k_com(t₀)`
  - Atlas 空中前空翻：**起跳瞬間 P₁ 最後一刻必須用 GRF 創造足夠初始角動量**
  - 空中靠收縮四肢改慣量張量加快翻轉（花式滑冰選手原理）
- **Swing 腳 Clearance 約束**：中間時刻 `p_foot,z(t_mid) ≥ h_clearance` → 抛物線足端軌跡
- **Multi-phase 發散陷阱**（面試必答）：
  - NLP 本質局部優化；Multi-phase 存極強非線性（SO(3) 旋轉 + GRF 雙線性耦合）
  - 初值猜錯（GRF 不足克服重力、落腳點太遠）→ 立刻局部死胡同
  - **工程正解**：先用**簡化 SRBD 或 LIPM** 解粗糙但物理合理的質心軌跡 + GRF → 作 Initial Guess 餵給全尺寸 TrajOpt → 100% 收斂

### 情境題 I：Trajectory Rollout → Closed-loop 橋接
- **為什麼開環軌跡不能直接執行**：
  - TrajOpt 榨乾模型每分潛力（剛好踩在摩擦錐極限邊緣）
  - 真實誤差（質量/慣量/摩擦）+ 擾動（風、地不平）+ 狀態漂移
  - 蝴蝶效應發散 → 摔倒
- **TVLQR (Time-Varying LQR) 橋接方案**：
  - 名義軌跡 `x̄(t), ū(t)` 當基準
  - 沿軌跡展開 `δx_{k+1} ≈ A_k·δx_k + B_k·δu_k`
  - 反向解 Riccati → 時變反饋增益 `K(t)`
  - **實機控制律**：`u(t) = ū(t) + K(t)·[x(t) - x̄(t)]`
  - ū 提供前饋大動力；K 像彈簧拉回偏離的狀態
- **iLQR 的 K 天然免費**：
  - iLQR Backward Pass 已算出 K_k
  - 直接把 `(u*_k, x*_k, K_k)` 下發硬體即插即用閉環跟蹤
- **Funnel Control / ROA (MIT Tedrake)**：
  - **Sum-of-Squares (SOS) 最佳化** 計算包覆軌跡的李雅普諾夫「不變管 (Invariant Tube)」
  - 初始擾動落 Funnel 入口 → 理論保證不摔 + 收束到終點
- **TrajOpt + NN Residual (DAgger + Distillation)** 現代做法：
  - 離線用 TrajOpt 生成萬條不同擾動的完美軌跡（Teacher）
  - 訓 NN Student policy 擬合
  - 部署 O(1) 極速閉環 + 保留 TrajOpt 最優性
- **「開環完美真機發散」面試答題**：
  - TrajOpt 給的只是**前饋 (Feedforward)**
  - **前饋 + 閉環反饋**：沿名義軌跡算 TVLQR K_t
  - 若底層有 WBC → TrajOpt 的 `x̄, x̄̇, x̄̈` 作參考指令餵 1kHz WBC
  - **WBC 的 PD 阻抗 + 動力學前饋吸收未建模干擾** = Atlas / ANYmal 極限運動業界鐵律

### 面試 talking points（Q3）
14. **iLQR vs DDP Hessian**：能講清楚「DDP 感知曲率快，iLQR 性價比高」
15. **Shooting vs Collocation**：Atlas 後空翻為什麼只能用 Collocation
16. **Multi-phase 初值用 SRBD/LIPM 粗解**：實戰經驗的簽名
17. **角動量守恆在空中 = 花式滑冰**：物理直覺 + 數學約束的融合答案
18. **iLQR 的 K 免費給閉環**：分辨「用 iLQR」vs「懂 iLQR」
19. **Funnel SOS 不變管**：研究所等級的 TrajOpt 魯棒性保證
20. **TrajOpt 前饋 + WBC 閉環 = Atlas 業界鐵律**：面試「開環發散」的標準答案
