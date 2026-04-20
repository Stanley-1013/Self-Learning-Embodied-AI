# Extracted v2: MPC — 補強素材 (session 531c9d80)

## Q1 補強（SRBD 降階 + 轉彎振盪除錯）

### 情境題 A：四足 SRBD + WBC 分層
- 問題：完整浮基動力學 O(n⁴) 無法 100Hz+ 即時
- **SRBD 降階**（Single Rigid Body Dynamics）— 所有質量集中質心，凸 QP 幾 ms 解出質心軌跡+足端 GRF
- **WBC 底層 1kHz**（Whole-Body Control）— 接收 MPC 的 GRF+質心加速度，用完整動力學處理關節極限/奇異/力矩分配
- MPC「看得遠」（避障預測）+ WBC「走得穩」（精確力矩）

### 情境題 B：轉彎振盪系統性除錯
- Step1: **線性化誤差** — Yaw 角變化大，sin(θ)≈θ 假設失效→提高線性化更新頻率或切 NMPC
- Step2: **預測時域盲區** — N 太短看不到彎道→動態拉長 N 或加入前瞻軌跡引導
- Step3: **約束衝突** — 摩擦錐/關節速度限幅太緊→引入 Slack Variables 軟化硬約束

### 面試 talking points
1. **MPC vs LQR**：LQR 無約束時足夠；LQR 無法處理物理硬約束→必須 MPC
2. **Learning MPC / Neural MPC**：Warm Start NN 砍迭代；Behavioral Cloning O(n³)→O(1)

## Q2 補強（NMPC 求解器 + Feasibility + Learning MPC）

### 情境題 C：NMPC vs LMPC + 實時求解技術
- **LMPC 失敗原因**：
  - 科氏力/離心力 ∝ 速度² 強非線性
  - 三角函數幾何映射
  - 接觸切換（Hybrid Dynamics）
  - 偏離線性化工作點就崩
- **NMPC 求解器**：
  - **SQP**：序列 QP，非線性約束友善，Warm Start 佳 → 硬即時首選
  - **Interior Point (IPOPT)**：大規模稀疏友善，但熱啟動差
- **實時三技術**：
  - **RTI (Real-Time Iteration)**：每週期只跑 1 次 SQP 迭代 + Warm Start → O(n³) 壓到固定 μs 級
  - **Multiple Shooting vs Single Shooting**：
    - Single：只把 u 當決策，x 靠積分 → 非線性誤差累積 → 不穩定系統數值爆炸
    - Multiple：x+u 都當決策，加連續性約束 x_{k+1} = F_RK4(x_k, u_k) → 誤差限於小區間 + 並行友善 → **四足標配**
  - **Condensing vs Sparse**：Multiple Shooting 稀疏大矩陣 → Condensing 消去 x 得稠密 QP（qpOASES）/ Sparse 保留交 HPIPM
- **業界框架**：
  - **acados**：C++ + RTI + HPIPM，工業界硬即時首選
  - **CasADi + IPOPT**：符號運算，概念驗證快，部署慢
  - **OCS2 (ETH)**：SLQ/DDP，足式機器人切換接觸動力學特化
- **機械臂 vs 四足 NMPC 差異**（面試必考）：
  - 機械臂 = **全驅動固定基座**：CTC O(n) 逆動力學 + PID 1kHz 完美追蹤 → NMPC 算力浪費
  - 四足 = **浮動基座欠驅動**：必須前瞻規劃 GRF 才能防摔 → 必須 MPC

### 情境題 D：Feasibility 與 CBF-MPC
- **Infeasible 發生時機**：
  - 初始狀態違反約束（如被踹一腳踢出可行域）
  - 約束互相矛盾（時間硬約束 + 扭矩硬約束 + 路徑封死）
- **產線救援三層**：
  - Layer 1: **Slack Variable 軟約束** — 引入 ε≥0，代價加 `λ·ε²`（λ 極大）；平時 ε=0，無解時允許微違反
  - Layer 2: **降級上一步可行解** — 拿上週期預測序列的第二步頂替
  - Layer 3: **Fallback Controller** — 優化器 NaN → 切純阻尼控制「軟趴下」
- **CBF 優於 Terminal Constraint**：
  - Terminal 只保證最後一步安全，中間時域可能撞
  - CBF: `ḣ(x) ≥ -α·h(x)` 定義**前向不變集 (Forward Invariant Set)**
  - CBF 嵌 MPC → 求解器戴「絕對不駛出邊界」緊箍咒
- **協作機械臂應用**：人突然闖入 → 即使 MPC 想抄捷徑，CBF 在邊界強制排斥速度 → 煞車
- **工業級 MPC 真相**：**70% 精力設計 Slack + Fallback 狀態機**，保證極端干擾下仍輸出平滑安全指令
- **acados 軟約束 pseudo-code**：
  ```c
  ocp_nlp_cost_model->zl = 1e6;  // L1 lower slack penalty
  ocp_nlp_cost_model->Zl = 1e4;  // L2 quadratic slack penalty
  ocp_nlp_constraints_model->idxsh = {...};  // which constraints to soften
  ```

### 情境題 E：Learning-based MPC + Model Mismatch
- **為什麼模型永遠不準**：
  - 執行器電流環延遲
  - 諧波減速器非線性柔性變形
  - 軟地面非剛性接觸
  - 齒輪背隙 + 溫漂摩擦
- **三大流派**：
  - **GP-MPC**：Gaussian Process 擬合殘差，輸出 μ (均值) + σ² (變異數) → 可設計 Chance Constraint
  - **DeepMPC**：NN 擬合殘差，推論快但黑箱，OOD 狀態下易發散
  - **Meta-MPC**（MAML）：在線 2-3 週期觀測就能梯度更新，適應負載突變
- **Chance Constraint 魔法**：
  - 傳統：h(x) ≤ 0
  - GP 版：P(h(x) ≤ 0) ≥ 99% → 轉化為 `h(μ) + 3σ ≤ 0`
  - **物理意義**：GP 越沒把握 (σ 大) → 邊界越緊 → 機器人越保守越慢 → 絕對安全
- **Amortized MPC (Imitation from MPC)**：
  - 雲端跑完美 NMPC → 收集 (State, Optimal_u) → NN 克隆
  - 部署：O(n³) → O(1) 微秒推論
  - **代價**：失去物理硬約束保證 → 必須**墊 QP Safety Filter**（基於 CBF）做最後安全裁切
- **真實平台對應**：
  - **ANYmal (ETH)**：OCS2 + latent space 地形摩擦估計 → MPC + WBC
  - **MIT Cheetah**：Convex MPC (SRBD) + WBC，嵌入式 100Hz+ 奔跑
  - **Tesla Optimus**：End-to-End RL (高層決策) + 傳統 MPC/WBC (底層平衡)
  - **Boston Dynamics Atlas**：離線軌跡庫 + 線上 MPC 在軌跡管內跟蹤

### 面試 Debug 思維（必考）
**問：「模擬器 MPC 完美但真機劇烈抖動，怎麼 debug？」**
- **答題策略（三步走）**：
  1. **通訊/執行延遲** — 模擬假設零延遲無限頻寬；預測模型必須顯式補償（用上週期 u 作當前狀態歷史輸入）
  2. **頻率掃頻辨識真實馬達頻寬** — MPC 輸出超馬達 Nyquist 頻率的高頻跳變 → 硬體只能抖動
  3. **代價函數加 Δu 懲罰** — 大幅調高控制量變化率權重 R，強制平滑且在馬達頻寬內的指令

### 面試 talking points（補強）
3. **全驅 vs 欠驅分辨**：面試「為什麼機械臂不用 MPC 但四足要用」的標準答案
4. **Feasibility 是量產生死線**：螢幕紅字 vs 產線馬達暴衝的本質差異；70% 精力在 Slack/Fallback
5. **GP 不確定性 → 自動收緊邊界**：跨控制與學習的橋樑，面試常問「怎麼處理模型不確定性」
6. **Amortized MPC + Safety Filter**：解釋端到端 NN 學 MPC 為什麼還需要 QP 兜底

## Q3 補強（Robust MPC + Economic/Distributed/Hierarchical + Contact-Implicit）

### 情境題 F：Robust MPC 三流派（Tube / Min-Max / Stochastic）
- **為什麼 Nominal MPC 在實機失敗**：
  - 未建模摩擦 + 外部擾動（風、接觸）+ 感測雜訊
  - 預測軌跡與實際軌跡偏離 → 超調或違反安全約束
- **Tube MPC（管狀 MPC）**：
  - **核心**：Nominal MPC 規劃理想名義軌跡 x̄，輔助 LQR 把真實 x 「束縛」在 Tube Z 內
  - 定義 Tube：**Control Invariant Set** — 若 `x_0 - x̄_0 ∈ Z`，輔助控制律下未來誤差永不逃出 Z
  - **物理意義**：「最壞擾動下，真實軌跡只在名義軌跡周圍微小管道內震盪」
  - 工程實作：**把原硬約束向內收縮一個 tube 半徑**，Nominal MPC 在收縮後邊界內規劃 → 真實系統絕對安全
  - 平台：L4 自動駕駛避讓行人的標準解法
- **Min-Max MPC（極小極大）**：
  - `min_u max_{w∈W} J(x, u, w)` 對抗性優化
  - **計算極度昂貴**：時域內擾動分支指數爆炸
  - 實務用 **Scenario-based approximation** 隨機抽幾條極端擾動軌跡近似
  - 工業幾乎不用
- **Stochastic MPC / Chance-Constrained**：
  - 擾動建模為機率分佈；約束 `P(h(x) ≤ 0) ≥ 1 - ε`
  - **與 GP-MPC 結合**：GP 預測殘差自帶 σ²；3σ 涵蓋 99.7%；約束向內收縮 3σ → 轉化為確定性 NLP
  - 平台：四足機器人跑酷 / 碎石路（接受 5% 滑移概率換取敏捷）
- **三流派 Trade-off 量化**（面試必答）：

| 類別 | 計算成本 | 保守度 | 安全保證 | 典型應用 |
|------|---------|--------|----------|----------|
| Min-Max | 最慢 | 最保守 | 絕對最壞情況 | 幾乎不用 |
| **Tube** | 極快（只解 Nominal） | 中等 | 絕對安全 (Hard Safety) | **L4 自動駕駛** |
| **Stochastic** | 中等 | 適中 | 機率安全（如 99.7%） | **四足碎石/跑酷** |

- **CasADi Chance Constraint 3σ 範例**：
  ```python
  mu_x, sigma_x = gp_predict(x_k, u_k)
  distance = ca.norm_2(mu_x[0:2] - obs_pos)
  tightened_safe_dist = d_safe + 3.0 * ca.max(sigma_x)
  g.append(distance - tightened_safe_dist); lbg.append(0.0)
  ```

### 情境題 G：Economic MPC + Distributed / Hierarchical MPC
- **Economic MPC**：
  - 成本函數 = **直接經濟指標**（能耗、生產節拍、刀具磨損）
  - 不是「跟蹤誤差平方」那種正定二次型
  - 甚至沒有全局最小平衡點
  - **物理意義**：「不要求又快又準走到目標，問在規定時間內怎樣走最省電」
  - 優化器可能讓機械臂半空滑行利用慣性而非全程出力
  - 平台：工廠機械臂（能耗 vs 週期時間）、無人機航程電量最佳化
- **Distributed MPC (DMPC) — 多機器人協作**：
  - 集中式 MPC 矩陣維度爆炸（100 台 AGV），毫秒級不可能
  - **ADMM (交替方向乘子法)**：全局問題拆局部；每台 AGV 解小 MPC + 廣播預測軌跡
  - **Consensus Constraint 產生影子價格（懲罰）**：軌跡重疊時迭代退讓達成無碰撞共識
  - 拉格朗日式：`L_i = J_i(x_i, u_i) + y_i^T(x_i - z) + (ρ/2)||x_i - z||²`
  - z = 鄰居協商公共安全軌跡；`y_i` = 拉格朗日乘子；偏離 z 受 ρ 二次懲罰
  - 平台：倉儲物流 AGV 隊、無人機編隊
- **Hierarchical MPC 分層**（ANYmal / 自動駕駛路線）：
  - **High-level MPC (10/50 Hz)**：簡化 SRBD 動力學，預測 1-2 秒，算質心軌跡 + 落腳點
  - **Low-level WBC/MPC (1 kHz)**：全尺寸動力學，預測極短（甚至單時刻 QP），精確分配關節馬達力矩 + 處理高頻碰撞
- **面試 talking point「未來具身智能混合架構」**：
  - **「大模型/RL + 多層 MPC + WBC」混合體**
  - RL 擅長模糊語義 + 離散決策（「跳過這個溝」），但缺硬約束保證 → 高層
  - Economic MPC 做中層能量/軌跡預測
  - 1kHz WBC QP 守關節扭矩 + 摩擦錐絕對物理底線
  - **「大腦慢思考、小腦快預測、脊髓硬反射」**

### 情境題 H：Contact-Implicit MPC (CI-MPC) — 2024 Game-Changer
- **傳統 MPC 的限制**：必須預定 Mode Sequence（0.1s 左腳落、0.2s 右腳落；先碰箱左邊再推右邊）
  - 環境變化導致提早接觸 → MPC 崩潰
- **CI-MPC 本質突破**：
  - 打破模式序列限制
  - 接觸力 λ + 接觸距離 ϕ(q) 同時當**優化變數**（不是預設常數）
  - **互補約束 (Complementarity Constraint)**：
    - `ϕ(q) ≥ 0`（不穿透）
    - `λ ≥ 0`（只能推不能拉）
    - `ϕ(q)·λ = 0`（要麼沒碰力為零，要麼碰到距離為零）
- **為什麼能「自動發現」策略**：
  - NLP Solver 追最小 Cost 時有權調 ϕ 和 λ
  - 發現「讓 ϕ=0 主動去碰 → 獲得 λ > 0 的力 → 利用力推箱子到目標點 → Cost 下降」
  - 抓取、推動、跑酷步伐都是 **Emergent Behavior 自發湧現**
- **計算挑戰 + MPCC 鬆弛**：
  - `ϕ·λ = 0` 極度非光滑非凸，梯度 undefined，求解器卡死
  - **MPCC（Mathematical Program with Complementarity Constraints）鬆弛**：
    - 改為 `ϕ(q)·λ ≤ ε`
    - ε 隨迭代從大慢慢 →  0（類似 Interior Point Barrier 思維）
- **最新前沿：Neural Dual MPC / Learned Contact Sequences**：
  - CI-MPC 線上求解仍慢
  - 離線跑萬次 CI-MPC 生成 through-contact 軌跡 → 訓 NN 預測最優接觸模式
  - 線上 MPC 套預測模式 → 非光滑 → 平滑問題
- **Manipulation 場景**：推箱子、翻頁、撬門栓的 through-contact 優化
- **平台**：MIT Cheetah CI-TrajOpt + online MPC
- **面試關鍵 talking point**：
  - **「統一 Free-space Motion 與 In-contact Manipulation 的數學表達」**
  - 過去撬門栓需寫複雜 FSM 切位置控與阻抗控
  - CI-MPC 眼中這就是一段連續數學軌跡 → 優化器自己決定何時輕碰、何時發力推
  - **把依賴專家經驗的 Heuristics 變成嚴謹數值優化問題**
- **Drake MPCC 鬆弛片段**：
  ```python
  phi = prog.NewContinuousVariables(1, "phi")
  lam = prog.NewContinuousVariables(1, "lambda")
  prog.AddConstraint(phi >= 0); prog.AddConstraint(lam >= 0)
  epsilon = 1e-3  # 外層迴圈讓 ε → 0
  prog.AddConstraint(phi * lam <= epsilon)
  prog.AddCost(cost_function(phi, lam, state, target))
  ```

### 面試 talking points（Q3 補強）
7. **Tube vs Stochastic MPC 選型**：L4 自駕用 Tube、四足跑酷用 Stochastic，講得出為什麼
8. **ADMM 影子價格**：Distributed MPC 的數學核心，AGV 協作落地的解釋骨架
9. **Hierarchical 「大腦/小腦/脊髓」三層**：展現系統觀的標準 talking point
10. **CI-MPC 的 emergent behavior**：2024 manipulation game-changer 必提；能解釋「優化器自己發現該怎麼碰」
11. **MPCC 鬆弛 `ϕ·λ ≤ ε`**：能分辨「聽過 CI-MPC」vs「實際跑過 CI-MPC」的細節

## Q4 補強（Learning-based 2024 + Adaptive MPC + Sampling-based）

### 情境題 I：Learning-based MPC 2024 前沿架構
- **Diffusion-MPC (DMD-MPC)**：
  - Diffusion 去噪過程 → MPC 的「初始軌跡生成器（Proposal Generator）」或先驗分佈
  - Diffusion 給候選軌跡 → SQP 求解器加動力學 + 防碰撞硬約束微調
  - **完美解 NMPC 容易陷入局部最優的痛點**
- **Foundation Model MPC (VLA + 底層控制)**：
  - RT-2 / OpenVLA / π0 等 VLA 作為 **High-level Planner (1-10 Hz)**
    - 輸出未來 1-2 秒 3D 目標路徑點 (waypoints/trajectory)
  - MPC 作為 **Low-level Tracker (100Hz-1kHz)**
    - 本體動力學矩陣精確追蹤 + 確保關節扭矩不超限
- **Dreamer / World Model + MPC**：
  - RNN/Transformer 從 pixel 學**緊湊潛在狀態空間 (Latent Space)**
  - 潛在空間內做線上 MCTS 前瞻搜索
  - 目標突然改變時 MPC 腦海推演新軌跡 → 完全不需重新訓練
  - **DreamerV3 + MPC 碾壓純 RL on DM Control**：樣本效率 + 零樣本泛化
- **Neural MPC Distillation + CBF Safety Filter**：
  - NN 模仿離線 MPC 輸出（Behavior Cloning）→ O(1) 推理
  - **代價**：失去硬約束保證 → 可能輸出致命越界指令
  - **必墊 CBF QP Safety Filter**：
    - CBF 安全約束數學式：`ḣ(x) + α·h(x) ≥ 0`
    - 物理意義：只要距離變化率 + 自身衰減項 > 0，永不越過 h(x)=0 死亡邊界
    - 微秒級 QP 求解，將 NN 指令「拉回」安全集
- **PyTorch 片段（NN + CBF Filter）**：
  ```python
  u_nn = neural_policy_net(state).detach().numpy()
  u_var = cp.Variable(u_nn.shape)
  cost = cp.sum_squares(u_var - u_nn)  # 盡可能跟 NN
  cbf_constraint = grad_h @ (f + g @ u_var) >= -alpha * h
  prob = cp.Problem(cp.Minimize(cost), [cbf_constraint, torque_limit])
  prob.solve(solver=cp.OSQP)
  ```
- **面試 talking point「Foundation Model + MPC 下一範式」**：
  - 純 end-to-end RL 缺物理可解釋性 + 硬約束保證 → 無法通過工業安全認證
  - 「大腦慢思考、小腦快反射」：
    - 大模型解 Open-world 語義泛化 + 長程規劃
    - MPC 解底層非線性動力學 + 高頻物理接觸
  - **大算力與嚴謹數學最完美的結合**

### 情境題 J：Adaptive MPC / Set-Membership / Neural Adaptive
- **為什麼 MPC 比 PID 更需要 online adaptation**：
  - PID Error-driven，模型錯了積分仍可硬拉回
  - **MPC Model-driven**，模型錯 → 預測未來 10 步全偏離物理 → 開環指令災難性錯誤
  - 例：抓起未知 10kg 鉛球 → MPC 預測完全失真
- **Set-Membership MPC**：
  - 未知參數 m 限於集合 `m ∈ [1, 5] kg` → 多面體 (Polytope)
  - 優化器必須保證**集合內每個可能參數值**都不違反安全約束
- **Adaptive Tube MPC**：
  - 傳統 Tube 的 tube size 固定
  - **結合 RLS 線上參數估計**：隨運行參數變異數矩陣越來越小 → Tube 半徑動態收縮
  - 物理行為：機器人剛開始謹慎（管徑大），「摸熟」物體重量後管徑收縮 → 動作變激進高效
- **Neural Adaptive MPC (Meta-learning)**：
  - 不只更新動力學參數
  - Meta-learning 上層網路根據 Context（風速 / 摩擦）**直接輸出 MPC 內部 Cost Weights**
  - 冰面時自動調高「軌跡偏差懲罰」、調低「速度獎勵」
- **RLS 參數更新律**：`θ̂_k = θ̂_{k-1} + K_k·(y_k - ϕ_k^T·θ̂_{k-1})`
  - 新估計 = 舊估計 + 預測誤差驅動的修正項
  - K_k 增益隨資料增多收斂
- **Adaptive vs Robust 面試陷阱**：
  - Robust MPC 假設最壞情況 → 邊界極大 → 機器人**過度保守僵硬**
  - **Adaptive MPC 先小邊界 + SysID 線上學真實參數** → 主動消除不確定性 → **安全 + 最優高動態性能兼得**
- **場景**：ANYmal 抓未知重量物體、無人機氣流變化、機械臂換不同夾具

### 情境題 K：Sampling-based MPC（MPPI / CEM）
- **MPPI (Model Predictive Path Integral)**：
  - 源自**隨機最優控制（Stochastic Optimal Control）**
  - 不依賴梯度，GPU 並行撒上萬條軌跡
  - Rollout 每條算 cost `S(τ_i)` → **Softmax 權重加權平均** → 最優控制
  - 權重公式：`w_i = exp(-S(τ_i)/λ) / Σ_j exp(-S(τ_j)/λ)`，`u_opt = Σ w_i·u_i`
  - λ = 溫度參數，控制探索/利用
- **為什麼比 NMPC 穩定**：
  - 不需計算 `∂f/∂x`
  - 足式機器人踩不平地面接觸力是硬拐角（非光滑）→ NMPC Jacobian 爆炸
  - **MPPI 靠暴力採樣繞過求導 → 天然免疫梯度爆炸**
- **CEM (Cross-Entropy Method)**：
  - 抽樣 → 保留 top-k 精英軌跡 → 重新擬合高斯 N(μ, σ²) → 下輪再抽
  - 迭代收斂至最優分佈
- **Sampling vs Gradient (acados IPOPT)**：
  - **優勢**：非凸、非光滑、離散 action 全部處理
  - **劣勢**：維度災難 — 控制維度 > 20（人形全關節扭矩）採樣指數爆炸 → 必結合降階模型（SRBD）
- **MuJoCo MPC (mjpc)**：
  - DeepMind 開源 MuJoCo 3 內建
  - 核心 Predictive Sampling + iLQG
  - 直接用 MuJoCo 極快物理正向積分 + GPU 並行上萬物理實例
- **Tesla Optimus / Boston Dynamics 傳言採用 MPPI 落地**
- **面試 talking point「sampling 不可替代場景」**：
  - **多模態決策 + 非光滑接觸**
  - 四足跑酷面前一棵樹：從左繞（局部最佳 A）vs 從右繞（局部最佳 B）
  - Gradient solver 卡鞍點或直接撞樹
  - MPPI 並行探索左右 → Softmax 自然坍縮到代價最低路徑
- **JAX MPPI 核心片段**：
  ```python
  noise = random.normal(key, (num_samples, horizon, action_dim)) * std
  u_samples = u_nominal + noise
  trajectory_costs = vmap(rollout)(u_samples)  # 並行萬條軌跡
  weights = exp(-(trajectory_costs - min_cost) / lambda_); weights /= weights.sum()
  u_optimal = u_nominal + sum(weights[:,None,None] * noise, axis=0)
  ```

## Q5 補強（Lie Group MPC + Offset-free + Event-triggered）

### 情境題 L：SE(3) / Lie Group MPC — 旋轉正確處理
- **四元數 / 旋轉矩陣直接作 State 的災難**：
  - **歐拉角奇異性 (Gimbal Lock)**：俯仰角 ±90° 時自由度丟失 → Jacobian 秩虧
  - **四元數 Over-parameterization**：旋轉只 3 DoF，四元數 4 變數 → 強制加非凸歸一化硬約束 `||q||² = 1`
- **Lie Group MPC 流形優化哲學**：
  - State 存在於李群 M (SO(3))，**優化的增量 Δ 與誤差存在切空間的李代數 m (so(3) ≅ ℝ³)**
  - 物理意義：用 ℝ³ 向量 ω 表示決策變數 → 完全避開冗餘 + 奇異性
  - 指數映射拉回流形：`R_{k+1} = R_k · exp(Δω^)`
- **誤差的純淨數學**：
  - 傳統 `q_target - q_current` 毫無物理意義
  - **對數映射 Logarithmic Map**：`e_R = log(R_target^T · R_current)^∨ ∈ ℝ³`
  - 完美代表「要轉多少角度對齊」，SQP 收斂極快
- **平台處理 body orientation**：
  - **MIT Cheetah / ANYmal**：WBC/MPC 機身姿態定義在 SO(3)，李代數 ℝ³ 算質心動量與 GRF 映射
  - **PX4 無人機飛控**：完全採用 SO(3) 誤差流形，避免翻滾姿態計算崩潰
- **面試陷阱「四元數 + 歸一化約束」致命傷**：
  - 面試常問：「為什麼 IPOPT 加 `||q||²=1` 不好？」
  - **正解**：`||q||²=1` 是球面流形；NLP 線性化時約束法向量與狀態更新方向強烈耦合；切線方向更新瞬間偏離球面 → 求解器為拉回球面產生極小迭代步長（**Zig-zagging**）→ 計算耗時暴增數十倍
- **manif 庫 C++ 片段**：
  ```cpp
  SE3Tangentd error_tangent = (X_target.inverse() * X_current).log();
  return error_tangent.coeffs();  // 乾淨的 6D 向量 [平移 3D, 旋轉 3D]
  SE3d X_next = X_current + SE3Tangentd(delta_u * dt);  // + 重載內部 exp map
  ```
- **框架**：Drake SE(3) MPC、OCS2 Manifold Optimization、GTSAM Lie Theory

### 情境題 M：Offset-free MPC / Integral Action
- **為什麼 Nominal MPC 對恆定擾動有穩態誤差**：
  - 預測模型 `x_{k+1} = A·x_k + B·u_k` 純數學
  - 現實有恆定外力 d_true → 模型預測「u=0 會停在原點」但實際被風吹走
  - **MPC 本質是個高級比例控制器 (P-controller)**，永遠差一點
- **Disturbance Model Augmentation 擴增**：
  - State 加入**積分擾動狀態 d_k**：
    - `x_{k+1} = A·x_k + B·u_k + B_d·d_k`
    - `d_{k+1} = d_k`（假設低頻/恆定）
    - `y_k = C·x_k + d_k`
  - **Kalman Filter / Luenberger Observer 介入**：
    - 對比「模型預測輸出」vs「感測器實際測量」→ 殘差積分實時估計 d_k
  - **前饋抵消**：MPC 帶 d_k 求解 → 優化器自然多輸出 Δu 頂住干擾 → **Offset-free**
- **Velocity Form MPC（增量式）**：
  - 不優化絕對 u_k，優化**增量 Δu_k = u_k - u_{k-1}**
  - 內部自然形成積分器 `u_k = Σ Δu`
  - 誤差不為零 → Δu 不為零 → 累積到誤差消除
- **穩態目標重計算 (Target Calculator)**：
  - `[A-I, B; C, 0] · [x_s; u_s] = [-B_d·d_k; r_k - d_k]`
- **面試 talking point「Offset-free 是工業鐵律」**：
  - 真實化工廠（Process Control）或無人機抗風懸停無完美模型
  - 沒 Offset-free 機制 → 如無 I 值 PD 控制器永遠帶靜差
  - 擴增 Integrating Disturbance + Luenberger 觀測器
  - **「閉環自我修正」能力 = 學術玩具 vs 產線產品的本質分水嶺**

### 情境題 N：Event-triggered / Self-triggered MPC — 計算資源優化
- **為什麼每週期重解是浪費**：
  - 機器人穩定巡航 + 狀態變化在上一週期 trajectory tube 內 → 上一解 `u_{t:t+N}^*` 仍**近乎最優 (Near-optimal)**
  - 無腦重解浪費算力
- **Event-triggered MPC**：
  - 平時**直接 Shift 沿用上週期控制序列**
  - 只有當 `||x(t) - x̂(t|t_k)||_Q ≥ σ·||x(t_k)||` 觸發條件滿足才喚醒優化器
  - 穩態下觸發頻率 100Hz → 10Hz → **省 50-80% 計算**
- **Self-triggered MPC**（更極致）：
  - 解當前 MPC 時**主動預測下次必須重解時刻 t_next**
  - 期間處理器進入 Sleep 模式
- **理論保證：閉環穩定性**：
  - 基於 **ISS (Input-to-State Stability) + Lyapunov Function**
  - 觸發條件設計：保證價值函數 V(x) 衰減率 `V̇ ≤ -α·||x||²`
  - 舊序列導致 V(x) 停止下降並開始回升時強制觸發
- **邊緣計算必要性**（面試 talking point）：
  - 未來具身智能面臨 **SWaP (Size, Weight, Power)** 嚴苛限制
  - 微型無人機 / 太陽能農業機器人無法搭桌面級 GPU
  - **「週期性強計算」→「按需計算 Compute-on-demand」**
  - Lyapunov 當「觸發守門員」保證不失控前提下砍 80% NMPC 計算
  - **大模型 + 複雜控制在嵌入式 MCU (STM32) 共存的唯一出路**
- **C++ Event-triggered 邏輯**：
  ```cpp
  bool should_trigger_mpc(x_curr, x_pred, sigma) {
      double error = (x_curr - x_pred).T * Q * (x_curr - x_pred);
      double threshold = sigma * x_curr.squaredNorm();
      return error > threshold;
  }
  if (should_trigger_mpc(...)) { solve_mpc_nlp(); u_cmd = get_first(); }
  else { u_cmd = get_shifted(); }  // O(1) 查表，極低功耗
  ```
- **平台**：低功耗無人機、太陽能農業機器人（太陽能 + 有限算力）

### 面試 talking points（Q4+Q5 補強）
12. **VLA + MPC 下一範式**：端到端 RL 無硬約束不能通過工業認證；講得出為什麼「Foundation Model + MPC」是必然
13. **Dreamer World Model + MPC 碾壓純 RL**：樣本效率 + 零樣本泛化的機制
14. **Adaptive Tube MPC 動態收縮**：講得出「機器人從謹慎變激進」的物理行為
15. **MPPI 多模態接觸繞樹**：sampling vs gradient 的分水嶺場景
16. **Lie Group / so(3) 切空間優化**：分辨「懂旋轉」vs「亂用四元數」
17. **Offset-free = MPC 的 I**：工業/產品級 MPC 必備，類比 PID 的 I 項 easy to reason about
18. **Event-triggered SWaP 邊緣 AI**：嵌入式具身智能落地的關鍵詞
