# Extracted v2: 動力學建模 — 補強素材 (session f239e001)

## Q1 補強（SysID + Neural Dynamics）

### 情境題 A：工業級 SysID 8 步
1. 解析模型推導（SymPy/Pinocchio URDF → 回歸矩陣 Y·π）
2. 摩擦模型擴充（庫倫+粘滯+Stribeck）
3. 激勵軌跡設計（傅立葉級數、最小化 Y 條件數、MATLAB fmincon）
4. 1kHz 實機採集 (q, q̇, τ via FCI)
5. filtfilt 零相位低通（Butterworth，不放大 q̈ 高頻雜訊）
6. WLS 求解 π=(Y^T W Y)⁻¹ Y^T W τ
7. **物理一致性校驗（LMI/SDP）** 避免負質量/非正定慣性張量
8. 交叉驗證新軌跡 RMSE < 額定扭矩 5%

### 情境題 B：Neural Dynamics 灰箱架構
- 純 NN vs RNEA 對比：物理可解釋性 / 未建模動態 / 資料需求 / 外推能力 / 計算效率
- **灰箱 Residual**：τ_total = τ_RNEA(q,q̇,q̈;π_base) + τ_NN(q,q̇,q̈, T_temp)
- **GPR 的獨特價值**：輸出不確定性 σ²，高 σ 時退回魯棒控制

### 面試 talking points
1. **RNEA O(n) vs Lagrangian O(n⁴)**：1kHz+ 控制迴圈無法用 Lagrangian
2. **動力學模型保質期**：溫漂 30% 粘滯、負載變化；RLS + 遺忘因子 online adaptation
3. **Passivity（Ṁ-2C 斜對稱）**：x^T(Ṁ-2C)x=0；李雅普諾夫 V=½q̇^T M q̇ 求導抵消科氏力做功，證明漸近穩定

## Q2 補強（PBC + Adaptive + Contact Dynamics）

### 情境題 C：Passivity-Based Control 深度
- **核心**：系統視為能量轉換器，儲存能量 ≤ 輸入能量 + 耗散能量 → 永不發散
- **Ṁ - 2C 斜對稱**：½q̇^T(Ṁ-2C)q̇ = 0（科氏力/離心力不做功）→ 李雅普諾夫證明命脈
- **PBC vs PD+G(q)**：PD 只保證點到點穩定，PBC 主動能量塑形 → 高速追蹤也無源
- **IDA-PBC**（Interconnection & Damping Assignment）：修改閉迴路等效質量與勢能，賦予機器人虛擬被動彈簧-阻尼外殼
- **接觸不暴走原理**：「把機器人閹割成被動彈簧阻尼系統，不能無中生有產生能量」
- **陷阱**：數位離散化延遲 + 感測器濾波相位滯後會注入虛假能量，破壞 Passivity → 高頻接觸震盪
- **平台**：Franka Panda、KUKA iiwa 內建 Passivity-based joint torque controller

### 情境題 D：Adaptive Control 線上參數估計
- **線性參數化**：τ = Y(q,q̇,q̈)π，π = 基礎參數向量（質量、質心、慣性張量）
- **Slotine-Li 控制律**：滑動曲面 s = ė + Λe；π̂̇ = -ΓY^T s → 李雅普諾夫全域穩定
- **Persistent Excitation 死穴**：Y 秩虧損 → 無法區分「負載變重」vs「摩擦變大」→ ∫Y^T Y dt 不可逆 → π̂ 發散漂移（甚至估計出負質量）
- **RLS + 遺忘因子 vs Slotine-Li**：
  - RLS 收斂極快（瞬間抓重物），但雜訊敏感 + covariance windup
  - Slotine-Li 收斂慢但誤差 e→0 絕對穩定，雜訊魯棒
- **工程 tip**：設計微小正弦探索軌跡確保 Y 滿秩；強制把估計參數投影回物理可行域
- **平台**：UR5 抓取未知工具時用 RLS 線上更新等效慣量

### 情境題 E：Contact Dynamics 與 Sim-to-Real
- **剛體碰撞 = Hybrid System**：微秒內速度瞬時跳變（衝量）→ 微分方程不連續 → Runge-Kutta 直接爆炸
- **LCP 表述**：接觸距離 ϕ≥0，接觸力 λ≥0，ϕ·λ=0（互補條件）
- **庫倫摩擦錐**：λ_t ≤ μλ_n → NCP 非線性互補 → 求解極耗時
- **MuJoCo Soft Contact**：放棄嚴格 LCP，允許微小穿透 → 連續凸最佳化；`solimp`(彈性/黏性) + `solref`(阻尼/回復時間) → 接觸力變穿透深度的連續函數 → RL 梯度可反傳
- **Sim-to-Real 陷阱**：
  - contact stiffness 太軟 → RL 學會「穿透物體作弊抓取」
  - 太硬 → 時間步長不夠小 → 物體一碰就爆炸彈飛
- **平台**：ANYmal 四足（足端連續打滑）、Franka 插孔
- **XML 範例**：`<geom solref="0.02 1" solimp="0.9 0.95 0.001" friction="1 0.005 0.0001"/>`
- **面試關鍵**：精密裝配（peg-in-hole 微米公差）絕不盲信模擬接觸力；Domain Randomization + 真實阻抗控制器兜底

### 面試 talking points（補強）
4. **PBC vs PD+G 的本質差異**：能量視角 vs 誤差視角；講清楚為什麼 PBC 對環境互動更魯棒
5. **Adaptive Control 的 PE 條件**：講得出什麼時候參數估計會漂移，以及實務上怎麼設計激勵
6. **LCP / Soft Contact 的工程權衡**：MuJoCo solimp/solref 調得動，Drake 嚴格 LCP 慢但準；RL 訓練用 MuJoCo、驗證用 Drake

## Q3 補強（Featherstone 空間代數 + RNEA 細節 + 摩擦/柔性）

### 情境題 F：Spatial Algebra 與 CRBA/ABA
- **為什麼分開算很差勁**：3D 向量分離時每次座標變換要分別對 ω/v 做旋轉平移，重複矩陣乘法 + 叉積
- **Plücker 座標**：ν = [ω^T, v^T]^T ∈ ℝ⁶，力旋量 f = [n^T, f^T]^T；空間慣性張量變 6×6 矩陣
  - 運動學/力學傳遞只需一次 6×6 × 6×1 乘法 → 壓縮 FLOPs
- **CRBA O(n²)**：從末端向基座遞迴累加子樹慣性，計算 M(q)
- **ABA O(n)**：
  - 定義「鉸接體慣量 Articulated Inertia」
  - 三次 O(n) 遞迴：forward 算速度偏置 → backward 傳鉸接慣量 → forward 算加速度
  - **完全繞過 M 矩陣建構與求逆**
- **ABA 真的永遠比 CRBA+Cholesky 快嗎？面試陷阱**：
  - 理論 O(n) vs O(n³) 但有常數項
  - 低 DoF (n=6, 7) 時 Cholesky 分解高度利用 SIMD + Cache Locality，常數項極小
  - **實務上通常 n > 9 才顯優勢** → 6 軸機械臂不一定強制 ABA
- **Spatial Cross Product 對偶**：
  - Motion (速度對速度): `ν₁ ×_m ν₂`
  - Force (速度對力): `ν ×_f f` = `-(ν ×_m)^T f`
  - Pinocchio 區分兩種 API，混用直接算錯
- **Pinocchio API**：`pinocchio::crba(model, data, q)`、`pinocchio::aba(model, data, q, v, tau)`
- **真實失敗**：30-DoF 人形機器人用 M⁻¹(τ-h) 算前向動力學 → 2ms/週期超時 → 改 ABA 降至 50μs

### 情境題 G：RNEA 工程實作細節
- **兩步遞迴物理意義**：
  - Forward Pass（基座→末端）：`v_i = v_{i-1} + s_i·q̇_i` 傳遞空間速度/加速度 → 「連桿怎麼動」
  - Backward Pass（末端→基座）：`f_i = I_i·a_i + v_i ×_f (I_i·v_i)` → 投影關節軸 `τ_i = s_i^T·f_i` → 「馬達要出多少力」
- **從 RNEA 萃取 M/C/G（實務技巧）**：
  - `G(q) = RNEA(q, 0, 0)`（基座加速度設為 -g）
  - `C(q,q̇)q̇ = RNEA(q, q̇, 0) - G(q)`
  - `M(q) 第 i 行 = RNEA(q, 0, e_i) - G(q)`（單位向量法）
- **Analytic Derivatives 對 NMPC/RL 的重要性**：
  - NMPC 需要 `∂τ/∂q`, `∂τ/∂q̇` 作 optimizer 的 Jacobian
  - Finite Difference：7-DoF 要 21 次 RNEA + ε 數值截斷誤差
  - **Pinocchio `computeRNEADerivatives`**：一次擴展遞迴直接算，速度 >10×
- **RNEA vs Lagrangian**：數學等價，但 Lagrangian O(n⁴)，RNEA O(n)；1kHz 控制迴圈只能用 RNEA
- **API**：`pinocchio::computeRNEADerivatives(model, data, q, v, a, dtau_dq, dtau_dv, dtau_da)`
- **真實失敗**：Franka 7-DoF NMPC 用 finite diff 算 Jacobian → 單步 15ms 超過 1kHz → 換 analytical gradients 降至 1.2ms

### 情境題 H：摩擦與柔性真實陷阱
- **庫倫摩擦靜止發散**：
  - `τ_f = F_c·sign(q̇) + F_v·q̇`
  - q̇≈0 時 sign 無限高頻跳變 → RK4 積分爆炸 + PID 微分項 Chattering
- **Stribeck 效應 / Stick-Slip**：
  - 低速時摩擦力隨速度指數下降（最大靜摩擦 → 動摩擦）
  - 產生**負阻尼** → 低速爬行黏滑震盪
- **LuGre 動態摩擦模型**：
  - 內部狀態 z 想像成接觸面無數「微小刷毛 Bristle」
  - 彎曲未滑動=彈性力；滑斷=動摩擦
  - `ż = q̇ - (|q̇|/g(q̇))·z`
  - `τ_fric = σ₀·z + σ₁·ż + σ₂·q̇`
  - 辨識 (σ₀, σ₁, σ₂, F_c) 要專門低速正弦激勵實驗
- **諧波減速器真相**：非線性扭轉彈簧 + 背隙 (Backlash) → 軌跡微抖
- **SEA (Series Elastic Actuator) 設計哲學**：
  - Boston Dynamics / ANYmal 乾脆在馬達與連桿之間加**實體彈簧**
  - 吸收地面高頻衝擊力（保護馬達）
  - 量測彈簧變形 Δx → F = K·Δx → **無摩擦干擾的高頻力控**
- **熱機 30 分鐘真相**：
  - 諧波減速器潤滑脂冷機時黏度極大
  - 溫升後 F_v 下降 20-30%
  - 冷機辨識的前饋參數熱機後**過補償** → 軌跡超調
- **補償策略**：
  - **DOB (Disturbance Observer)**：不依賴摩擦模型；`disturbance_est = LPF(τ_actual - τ_ideal)` → 前饋抵消
  - **RLS + Forgetting Factor**：線上追蹤溫度漂移即時微調 F_v
- **真實失敗**：UR5 插孔早上測試完美，下午熱機後頻繁觸發安全停機；排查發現摩擦補償變過大 → 接觸速度暴衝；修正：online ID + 接觸前 50mm 切阻抗控制

### 面試 talking points（Q3 補強）
7. **ABA O(n) vs CRBA+Cholesky O(n³) 的常數項陷阱**：低 DoF 未必 ABA 快，要懂 SIMD/Cache locality
8. **Spatial cross product 的 motion/force 對偶**：Pinocchio 誤用直接算錯
9. **analytical RNEA derivatives 對 NMPC 的必要性**：能量化「 15ms → 1.2ms」才算真懂
10. **熱機 30 分鐘**：面試經典「為什麼工業機器人需要 warm-up」，沒做過現場的答不出來

## Q4 補強（浮動基座 + CWC + Dynamics for RL）

### 情境題 I：Floating-Base Dynamics 全解
- **未致動 6 DoF**：基座 3 平移 + 3 旋轉沒有馬達驅動；運動完全靠 GRF / 推力
- **CMM (Centroidal Momentum Matrix)**：
  - `A(q)·q̇ = [l; k]^T`（l = 線動量, k = 角動量）
  - 物理意義：q + q̇ → 機器人往哪飛、在空中怎麼轉
- **全身動力學標準形式**：
  ```
  ┌M_base  M_bj ┐┌ẍ_base┐   ┌ 0 ┐   ┌J_c^T·F_c┐
  │             ││       │+C=│    │ + │          │
  └M_jb^T  M_jj ┘└q̈_j   ┘   └ τ ┘   └          ┘
  ```
  - Upper-left block τ=0（基座沒馬達）→ **Underactuated Dynamics**
  - 只能用關節馬達蹬地產生 F_c，再透過 J_c^T·F_c 間接改變基座加速度
- **Newton-Euler on CoM（GRF → 浮動基加速度）**：
  - `l̇ = ∑F_c - mg`（線動量 = 總接觸力 - 重力）
  - `k̇ = ∑(p_i - p_com) × F_c + τ_contact`（角動量 = 接觸力矩總和）
- **Angular Momentum 守恆（黃金原則）**：
  - 空中飛行相：無接觸力 + 重力不產生力矩 → **總角動量 k 嚴格守恆**
  - 貓空中翻正、MIT Cheetah 後空翻核心
  - 甩四肢改局部角動量 → 角動量守恆迫使軀幹反向旋轉（Zero-Momentum Turn）
- **真實失敗**：四足跳躍落地未在 WBC 約束 `k̇` → 前腿觸地產生大俯仰力矩 → 前空翻砸地；修正：MPC cost 加對 k̇ 的強懲罰
- **Pinocchio API**：`pinocchio::ccrba(model, data, q, v); Ag = data.Ag; hg = data.hg;`

### 情境題 J：Contact Wrench Cone (CWC) 與 Multi-contact QP
- **為什麼 ZMP 被淘汰**：
  - 假設所有接觸點共水平面 + 摩擦力無限大
  - 人形機器人「一手扶牆+雙腳踩不同階梯」→ ZMP 在 3D 失去物理意義
- **CWC 統一 4 類約束**：
  1. 法向壓力 `f_z > 0`（不能拉地面）
  2. 庫倫摩擦錐 `√(f_x² + f_y²) ≤ μ·f_z`（不打滑）
  3. CoP 約束（腳邊不翻轉）
  4. Yaw torque 約束（腳底不原地打滑轉）
  - 線性化 → 多面體錐 `W·F_c ≥ 0`
- **CWC-QP 與 WBC 分層**：
  ```
  min_{F_c} ‖∑F_c - m(p̈_des - g)‖² + ‖∑(p_i × F_c) - k̇_des‖²
  s.t. W·F_c ≥ 0
  ```
- **為何是具身智能「潛規則」**：Tesla Optimus / Figure / Unitree H1 多接觸（雙手抱箱+雙腳站）底層 100% 依賴 CWC-QP
- **斜坡陷阱**：若未把 CWC 從世界系旋轉到局部接觸面系 → 法向力估錯 → 啟動瞬間打滑劈腿

### 情境題 K：Dynamics for Learned Policies
- **Model-Free vs Model-Based**：
  - Free：只看 (S,A,R,S') 黑箱，海量隨機探索
  - Based：利用 M/C/G 先驗，RL 只學殘差或阻抗參數
- **Privileged Learning (Teacher-Student)**（ETH ANYmal / Boston Dynamics 路線）：
  - Teacher：模擬器裡讀 privileged info（地面 μ、質量 m、地形高度圖）→ 學出完美策略
  - Student：只用 proprioception（關節位置/速度歷史 + 雜訊 IMU）
  - RNN/LSTM 從「腳底打滑的速度波動歷史」隱式推斷 μ → 蒸餾出 Teacher 級救車能力
- **Differentiable Simulators (MuJoCo MJX, Brax, Dojo)**：
  - 傳統 REINFORCE 高方差估計 → 可微模擬器直接讓 `∂s_{t+1}/∂a_t` 穿透物理引擎
  - 「蒙特卡洛盲搜」變精確梯度下降 → sample efficiency × 100
- **MPPI (Model Predictive Path Integral)**：GPU 並行採樣上萬條 SRBD 軌跡 → Softmax 權重混合 → 即時最優控制
- **Neural ABA (Tesla/Figure 傳言)**：O(n) 的 ABA 在萬環境並行仍是 CPU 瓶頸 → NN 擬合 ABA → GPU O(1) 矩陣乘 → 百萬級並行
- **面試關鍵**：
  - 純 RL 工程師遇 Sim2Real Gap → 盲加 Domain Randomization → 策略保守僵化
  - 懂動力學的：Action 空間設為「期望質心加速度」或「阻抗 ΔK/D」，底層交 WBC → 大腦 RL + 小腦 WBC 混合架構
- **失敗案例**：純 Model-Free RL 學會高頻抖動卡物理 bug 穿模移動；修正：reward 加 `τ²` 能量懲罰 + action 低通濾波

## Q5 補強（TrajOpt + 馬達/執行器 + 柔性連桿）

### 情境題 L：動力學在 TrajOpt（DDP/iLQR/Collocation/CITO）的角色
- **運動學規劃 + 後期時間參數化為什麼失敗**：
  - 只考慮幾何避障 → 忽略動力學非線性耦合
  - 高速時科氏+慣性呈幾何級數放大 → **Torque Saturation** → 軌跡不可執行
- **iLQR vs DDP 的導數差異**：
  - iLQR：只用一階 Jacobian `A=∂f/∂x, B=∂f/∂u`
  - DDP：完整二階 Hessian 張量 `∂²f/∂x²`
- **為什麼需要 `∂²RNEA/∂q²`**：後空翻等極動態動作狀態曲率極大，只有精確二階導才能捕捉「慣性矩陣隨姿態的曲率」→ 幾步迭代二次收斂
- **Shooting vs Collocation**：
  - **Shooting（打靶法）**：只把 u 當決策，x 靠動力學硬積分 → 積分誤差指數放大 → 梯度爆炸，四足少單用
  - **Direct Collocation（直接配點）**：x + u 都當決策，動力學轉等式約束 `x_{k+1} - x_k - f(x_{k+1},u_k)·dt = 0` → IPOPT 可大膽探索違反物理的狀態，最後收斂至物理解 → 極穩
- **CITO (Contact-Implicit)**：
  - 傳統必須預定「何時哪隻腳落地」（Mode Sequence）
  - CITO 把接觸距離 ϕ 與接觸力 λ 當互補約束 `ϕ≥0, λ≥0, ϕ·λ=0`
  - 優化器自己發現「何時 λ>0 = 接觸發生」→ **自動湧現步伐 (Automatic Gait Discovery)**
- **平台應用**：Atlas 跑酷（離線 Collocation + 在線 MPC 跟蹤）、MIT Cheetah（iLQR + SRBD 高速奔跑）
- **面試陷阱「完美軌跡模擬能跑真機發散」答題**：
  - TrajOpt 開環解是極度特化脆弱的（恰卡在力矩極限）
  - 真實世界摩擦/背隙 5% 誤差就不可執行
  - **工程解法**：TrajOpt 只給 (x_ref, u_ref) 參考+前饋，底層必須墊 LQR 或 WBC 閉環反饋吸收殘差

### 情境題 M：馬達電氣動力學 + 執行器
- **時間常數差兩個數量級**：
  - 電氣 `τ_e = L/R`：0.1-1 ms（電流/力矩爬升）
  - 機械 `τ_m = J/B`：10-100 ms（轉速爬升）
  - **根本原因**：電流環可遠快於速度/位置環
- **FOC 三環架構**（Park 變換解耦 I_d/I_q）：
  - **電流環** 1-4 kHz：控 I_q 跟蹤目標力矩（最高頻寬）
  - **速度環**：位置誤差 → 目標速度
  - **位置環**：最外層
- **Cogging Torque / Torque Ripple**：
  - PMSM 轉子磁鐵 + 定子齒槽磁吸力 → 不通電轉動都有頓挫感
  - 對阻抗控制注入週期性干擾 → 必須預標定 + 前饋電流補償
- **Backlash 為什麼無法在 RNEA 裡建模**：
  - RNEA 假設連桿絕對剛性連接
  - 背隙 = 反轉瞬間馬達轉但連桿沒動（死區）
  - 打破 `q̈_motor ∝ q̈_link` 微分連續性
  - **Spongy Joint Model**：馬達 + 連桿拆兩獨立質量塊，中間非線性彈簧耦合
  - 方程組：
    ```
    M_link·q̈_L + C_L·q̇_L + G(q_L) = τ_J
    J_motor·q̈_M + B_M·q̇_M + τ_J = τ_motor
    τ_J = K(θ)·(q_M/N - q_L) + D·(q̇_M/N - q̇_L)
    ```
  - 背隙區間 K≈0 → 馬達與連桿瞬間脫鉤
- **SEA vs QDD 差異**：
  - **SEA** (ANYmal)：馬達+連桿間加實體彈簧 → 吸收衝擊 + 純粹力回饋（量 Δx）；缺點：降低頻寬（變軟）
  - **QDD** (MIT Mini Cheetah)：大扭矩 BLDC + 超低減速比 6:1～9:1 → 幾乎無背隙/無摩擦 → **極高反向驅動性** → 電流環估力矩極準 → 高頻寬透明力控
- **頻寬鐵律**（面試必考）：
  - **電流環 > 速度環 > 位置環**，內環至少比外環快 5-10 倍
  - 位置環規劃太快 + 內環電流環跟不上 → 相位滯後 → 振盪/發散

### 情境題 N：柔性連桿 + 大型機械臂
- **為什麼大型臂（>6m）不能用剛體假設**：自然共振頻率 < 10 Hz，馬達加減速極易激發結構共振 → 末端數十公分難衰減抖動
- **建模兩大流派**：
  - **FEM**（有限元素）：精度高但狀態成千上萬，不可即時控制
  - **Assumed Mode Method (AMM)**：
    - `y(x,t) = ∑ φ_i(x)·δ_i(t)`（振型 × 時間廣義座標）
    - 截取前 2-3 階低頻模態 → 狀態空間友善
- **Singular Perturbation Theory**（奇異攝動）：
  - 慢子系統：大範圍宏觀剛體運動（CTC）
  - 快子系統：局部彈性高頻振動（LQR 抑制）
  - **解耦後分別設計控制器**
- **Input Shaping**（土法煉鋼最有效）：
  - ZV Shaper 兩脈衝：
    - `t_1=0, A_1 = 1/(1+K)`
    - `t_2 = π/ω_d, A_2 = K/(1+K), K = exp(-ζπ/√(1-ζ²))`
  - 物理意義：第二脈衝引起的振動相位剛好與第一脈衝差 180° → 物理上完全抵消
  - 實作：原指令與 Shaper 脈衝做卷積
- **平台案例**：
  - **Canadarm2**（ISS 1.5 噸 / 17m）：抓太空艙必依 Input Shaping + 模態抑制
  - **ABB YuMi**：前饋 Input Shaping 抑制關節柔性抖動
- **Soft Robotics 挑戰**：真正無窮自由度；D-H + Jacobian 全失效；需 **Cosserat Rod Theory** 或 **PCC (Piecewise Constant Curvature)** → ODE 升級 PDE
- **大型臂面試關鍵「非同位控制 Non-collocated Control」**：
  - 馬達在關節（致動點）、精度要求在 6m 外末端（量測點）
  - 中間巨大柔性連桿 → 傳遞函數出現**右半平面零點 (Non-minimum Phase Zeroes)**
  - 傳統 PID 拉高增益直接發散 → 必須**狀態觀測器 + LQR** 或奇異攝動

### 面試 talking points（Q4 + Q5 補強）
11. **浮動基座 Underactuated Dynamics**：講得出為什麼「馬達 τ 不能直接讓機器人起飛」、要透過 GRF 間接改基座加速度
12. **Angular Momentum 守恆**：空中飛行相的黃金原則，貓翻正/後空翻的物理核心
13. **CWC 超越 ZMP**：3D 多接觸統一約束，現代 WBC 基石
14. **Teacher-Student Privileged Learning**：ANYmal/BD 的 Sim2Real 公式，RNN 隱式推斷摩擦
15. **TrajOpt 脆弱性 + LQR 閉環兜底**：面試必答「為什麼完美軌跡真機發散」
16. **電流環頻寬 > 速度環 > 位置環**：控制理論鐵律，串級設計基礎
17. **Backlash 的 Spongy Joint Model**：RNEA 建不了的原因 + 解法
18. **Non-collocated Control 的右半平面零點**：大型柔性臂面試的 signature question
