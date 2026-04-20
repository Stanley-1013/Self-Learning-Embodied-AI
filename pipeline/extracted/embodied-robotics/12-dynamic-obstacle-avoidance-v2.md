# Extracted v2: Ch12 動態避障 — 5 dedicated queries (from scratch)

## Q1 補強（DWA/APF + VO/RVO/ORCA + MPC+Prediction）

### 情境題 A：DWA + APF 基礎
- **DWA (Dynamic Window Approach)**：
  - 速度空間 (v, ω) 前向模擬
  - **動態窗口**：`V_d = {(v,ω) | v ∈ [v_c - a_max·Δt, v_c + a_max·Δt]}` — 只在物理絕對可達範圍採樣 → **天然處理加速度極限**
  - 軌跡評分：`cost = α·heading + β·obstacle_dist + γ·velocity`
- **DWA 致命缺陷**：
  - 極短視（Myopic），只前瞻 1-2 秒
  - **U 型障礙/死胡同**：無論怎麼走離目標都變遠 → 原地打轉卡死
- **APF 數學**：
  - 吸引 `U_att = ½k||q - q_goal||²`（引導向目標）
  - 排斥 `U_rep = ½η·(1/ρ - 1/ρ₀)²`（ρ < ρ₀ 生效；靠近指數飆升）
- **APF Local Minima 災難**：
  - 機器人夾在兩障礙正中央 / 目標在牆後 → 引力梯度 = 斥力梯度方向相反 → **合力為零死鎖**
- **VFH (Vector Field Histogram)** 改良：
  - 感測器點雲 → 極座標直方圖
  - 計算「自由山谷 (Free Valleys)」
  - 直接朝最接近目標的自由山谷前進 → 避免力抵消
- **自駕/人形 vs AGV 選型答題**：
  - **室內 AGV 平坦 2D + 低速 (<2 m/s) + 靜態貨架** → DWA 極快 + 成本效益 → 主流
  - 自駕 / 人形**高動態環境（60 km/h 汽車、亂跑行人）**：
    - DWA 短視採樣無法預測 3 秒後行人
    - APF 無法處理非完整約束高速側滑
  - **必轉向「基於行為預測的 MPC」**
- **Nav2 DWB C++**：
  ```cpp
  double score = alpha * headingDiff(traj.back(), goal)
               + beta * (1.0 / costmap->minDist(traj))
               + gamma * (max_v - traj.velocity);
  ```

### 情境題 B：VO / RVO / ORCA 多智能體黃金標準
- **VO 幾何本質**：
  - A 與 B 保持當前速度的「碰撞錐 (Collision Cone)」
  - 相對速度空間分析 → 只要相對速度**不在錐內**就絕對安全
- **RVO 震盪消除**：
  - 純 VO 走廊相遇會「同時往左閃、再同時往右閃」→ **Reciprocal Dance 震盪**
  - RVO: **雙方各承擔 50% 避讓責任** → A 只調一半、期望 B 也做一半
- **ORCA 半平面線性規劃**：
  - 幾何錐 → 嚴格**半平面約束**
  - 數學：`(v_A - (v_A^opt + u/2)) · n ≥ 0`
  - **所有約束都線性 → LP 求解 O(n)**
  - 毫秒級算出上千台 AGV / 無人機無碰速度
- **Freezing Robot Problem** 卡死陷阱（面試必答）：
  - 被夾兩個相反方向行人中間 → 左行人給向右半平面、右行人給向左
  - 人群密集時**所有半平面交集為空** → LP 無解 → 瞬間煞停
  - **修正方案**：
    1. 硬約束轉軟約束：引入鬆弛變數 ε，允許微小侵入加巨大懲罰
    2. 結合拓撲圖搜索：發現這條路不通 → 全局路徑繞到人群後方
- **RVO2 Library C++**：
  ```cpp
  Line orca;
  orca.point = velocity_ + 0.5 * u;  // 各承擔一半責任
  orca.direction = Vector2(-n.y(), n.x());  // 垂直法向量
  orcaLines_.push_back(orca);
  linearProgram2(orcaLines_, prefVelocity_, newVelocity_);  // LP 求解
  ```

### 情境題 C：MPC + Prediction 自駕工業標配
- **為什麼 MPC 是移動避障最強武器**：
  - APF/DWA 只看當下瞬間快照
  - **MPC 前瞻**：在 t 時刻優化未來 H 步軌跡
  - 把動態障礙「未來 k 步預測位置」寫進 MPC 未來 k 步約束
  - 自動學會**「減速讓行、加速搶道、提前繞行」** 動態博弈
- **動態障礙運動模型**：
  - **傳統物理**：CV (勻速) / CA (勻加速) / **IMM (Interacting Multiple Models)** 交互多模型切換（直行 vs 轉彎機動）
  - **神經網路**：**Social-LSTM / Trajectron++**
    - 行人軌跡高度耦合（人避人）
    - RNN/Transformer 把周圍行人歷史 + Social Force Model 結合
    - 預測未來 3-5 秒**多模態機率分佈軌跡**
- **Chance-Constrained MPC**：
  - 預測不確定（直走 or 右轉）→ 當固定點會撞
  - **NN 預測軌跡 → 時間變化的 2D 高斯分佈 `N(μ_k, Σ_k)`**
  - MPC 避障約束向外擴張 **3σ 涵蓋 99.7% 機率**
  - 面對行為莫測行人自動拉開更大橫向安全距離
- **MPC 動態避障硬約束**：
  - `||p_ego(k) - μ_obs(k)||² ≥ (d_safe + 3σ_obs(k))²`  ∀k ∈ [1,H]
  - 物理意義：每未來步自車到障礙預測質心距離必嚴格 > 基礎安全半徑 + 3σ 不確定擴張
- **面試答題「避障工業標配」**：
  - L4 自駕 / 高端具身：**避障 = 時空優化問題**，不是幾何問題
  - 傳統：把移動車當「正在移動的牆」→ 極保守煞停
  - **現代「Prediction + MPC」**：
    - 前端 Social-LSTM 預測行人**時空管 (Spatio-temporal Tube)**
    - 後端 MPC 保證自車時空軌跡不與行人時空管干涉
  - **預測不確定性完美融入控制硬約束**
- **平台**：Waymo Driver、Tesla FSD、Apollo
- **CasADi Chance-Constrained MPC**：
  ```python
  for k in range(horizon):
      opti.subject_to(X[:,k+1] == rk4_step(model, X[:,k], U[:,k], dt))
      dist_sq = (X[0,k+1] - obs_mu_x[k])**2 + (X[1,k+1] - obs_mu_y[k])**2
      safe_margin = robot_radius + obs_radius + 3.0 * obs_sigma[k]
      opti.subject_to(dist_sq >= safe_margin**2)  # 3σ 硬約束
  ```

### 面試 talking points（Q1）
1. **DWA 動態窗口天然處理加速度極限**：數學上絕對可達範圍採樣的優雅
2. **APF Local Minima 合力為零死鎖**：兩障礙中央的物理推理
3. **RVO 50% 互相禮讓消震盪**：Reciprocal Dance 的標準解
4. **ORCA LP 毫秒千台**：線性規劃轉幾何為優化的關鍵
5. **Freezing Robot + 鬆弛變數**：密集人群的工業補救
6. **3σ 時空管避障**：Chance-Constrained MPC 的現代答案
7. **自駕/人形退役 DWA 但 AGV 主流**：分辨場景選型成熟度

## Q2 補強（CBF + Social-Aware + Aerial 3D）

### 情境題 D：Control Barrier Functions (CBF) 安全關鍵控制
- **2020 後 CBF 成統一理論的理由**：
  - 傳統：目標追蹤用 CLF（控制李雅普諾夫）+ 避障用 APF/幾何硬約束，兩套框架
  - **Ames 等人的 CBF 把「安全」嚴格轉化為 Forward Invariant Set 數學問題**
  - 安全約束與控制指令在同一能量/梯度語境下統一
- **CBF 核心定義**：
  - 安全集合 `C = {x ∈ ℝⁿ | h(x) ≥ 0}`
  - 若控制 u 能滿足 `ḣ(x,u) + α(h(x)) ≥ 0` → h 為 CBF
  - **物理意義**：ḣ(x) 是你靠近危險邊界的速度；α(h(x)) 是允許的最大靠近速度（正比於離邊界距離）
  - 保證：「越靠近死線 h(x)=0，向危險方向移動的速度必須幾何級數衰減至零」→ 永不越界
- **HOCBF (Higher-Order CBF)**：
  - 機器人是二階系統（輸入是力矩/加速度，約束在位置 x）
  - 直接對位置求一階導只得速度（Relative Degree > 1）
  - HOCBF 多階泰勒展開 + 指數衰減 → 映射高階狀態到位置約束
  - **解決剎車距離不足問題**
- **CBF vs APF**（沒有 local minima 的優雅性）：
  - APF 主動施加斥力 → 引力與斥力抵消時 local minima
  - **CBF 是安全濾網，不主動產力**：
    - 只監聽 Nominal Controller（PID 或 RL 輸出 u_nom）
    - u_nom 安全時完全不干預
    - 只有 u_nom 想越界時 CBF 以最小代價投影到安全空間
  - **不改變原能量場拓撲** → 天然無 local minima
- **CBF-QP 數學**：
  - `min_u ½||u - u_nom||²`
  - s.t. `L_f·h(x) + L_g·h(x)·u + α·h(x) ≥ 0`（安全約束 = 李導數形式）
  - s.t. `u_min ≤ u ≤ u_max`（物理極限）
- **「給 RL 加硬安全保證」工業答題**：
  - RL 黑箱 + 探索性 → 物理世界極易危險動作
  - 不能指望 RL 自己學會 100% 安全
  - 標準解：**「RL 大腦給 u_nom，CBF-QP 脊髓反射層攔截」**
  - CBF-QP 凸優化微秒求解 → 最小二乘攔截修正 → RL 泛化 + 數學物理安全底線
- **平台**：自駕 AEB（自動緊急煞車）、協作機械臂 Speed and Separation Monitoring
- **CBF-QP Python**：
  ```python
  u = cp.Variable(u_nom.shape)
  objective = cp.Minimize(0.5 * cp.sum_squares(u - u_nom))
  Lf_h = grad_h @ f_x
  Lg_h = grad_h @ g_x
  cbf_constraint = [Lf_h + Lg_h @ u + alpha * h_val >= 0]
  torque_limit = [u >= -MAX_TORQUE, u <= MAX_TORQUE]
  cp.Problem(objective, cbf_constraint + torque_limit).solve(solver=cp.OSQP)
  ```

### 情境題 E：Social-Aware / Crowd Navigation
- **Freezing Robot Problem 災難**：
  - 把行人當移動圓柱障礙 → 密集人群所有未來 Feasible Velocity Set 被行人時空軌跡佔滿 → 規劃器無解 → 煞停卡死
  - **但實際上人會讓人** → 機器人主動擠一下人群自然分開
  - 必須引入「社會感知 (Social-Aware)」
- **Social Force Model (SFM, Helbing 1995)**：
  - 人群為相互作用的粒子系統
  - 行人受目標吸引 + 其他行人/牆壁排斥
  - `F_i = F_goal + Σ F_repulsive + F_wall`
  - **缺點**：被動反應式，無法建模「主動博弈協商」
- **CADRL (Collision Avoidance with Deep RL)**：
  - 端到端 RL + 自我博弈 (Self-play)
  - 學到「從別人身後繞、主動讓路」等隱式社會規則
  - 解決 2-4 個 Agent 導航
- **SARL (Socially Attentive RL)**：
  - 密集人群 N 變動 → 傳統 RL 難處理可變維度狀態
  - **Self-Attention 機制**：
    - 計算自身與每個行人的注意力權重 Attention Score
    - 只聚焦「可能碰撞」的關鍵行人
    - 變長人群狀態壓縮為定長 Context Vector
  - 公式：`e_i = MLP(s_robot, s_human_i)`, `α_i = exp(e_i) / Σ exp(e_j)`
- **Group-Aware Navigation**：
  - 兩三人並排走聊天 → 從中間穿過不禮貌
  - 軌跡相似 + 距離近的行人聚類為 Group → 視為不可分割整體 → 從外側繞行
- **「服務型機器人商業化最後一哩」答題**：
  - 醫院商場挑戰不是「找路」，而是「被卡死」或「引發人類反感」
  - **SARL + SFM 混合 = 非語言社會協商能力**
  - 決定機器人是笨拙還是優雅融入人類社會
- **平台**：UBTECH Walker、Figure 01 醫院導航、Amazon Astro
- **SARL Attention PyTorch**：
  ```python
  class CrowdAttention(nn.Module):
      def forward(self, robot_state, human_states):
          joint = torch.cat([robot.expand(...), humans], dim=2)
          features = F.relu(self.mlp(joint))
          scores = self.attention_net(features)
          weights = F.softmax(scores, dim=1)
          context = torch.sum(weights * features, dim=1)  # 固定維度環境 context
          return context
  ```

### 情境題 F：Aerial / 3D 避障 + Emergency Evasion
- **無人機 3D 避障三大挑戰**：
  1. **SE(3) 飛行 + 嚴重欠驅動**（無剎車，減速必須機身 Pitch 仰轉）
  2. **深度相機 FOV 有限**：高速轉彎盲區無地圖 → 撞機
  3. **高速 10 m/s 穿梭動力學**
- **FASTER Planner (MIT) — 雙軌跡並行**：
  - 核心哲學：**安全備援 (Safety Fallback)**
  - 每週期同時解兩條：
    - **Primary Trajectory**：激進高速，探索 Free space
    - **Backup Trajectory**：保證能在安全區域完全煞停/懸停
  - 主軌跡規劃失敗 → 無縫切到 Backup 煞停 → 絕對安全
- **Bubble Planner / Flight Corridors**：
  - 3D 點雲避障是**非凸**
  - **Corridor Planner** 把 Free Space 膨脹成**重疊的凸多面體或球體**
  - 非凸避障 → 待在凸多面體內的線性不等式 `A_i·x ≤ b_i`
  - QP 極速求解 B 樣條軌跡
  - **B-Spline 凸包性保證所有控制點在多面體內 → 整條連續曲線絕不穿出**
- **Reactive Vision-based Avoidance (事件相機)**：
  - 傳統相機高速運動**動態模糊** + 30Hz 幀率在 20 m/s 致命盲區
  - **Event Camera** 只對光強變化反應，**異步事件流 <1ms 延遲 + 超高動態範圍**
  - 亞毫秒內算撲面障礙的 **Time-to-Contact (TTC)** → 純反應式躲避 (Reactive Evasion)
- **Perception-aware Planning**：
  - 規劃時把**相機 FOV 作約束**
  - 橫移/側飛時**機頭(相機)始終對準即將前往的未知區域** → 避免盲飛
- **「Event Camera 下一代 UAV 方向」答題**：
  - 傳統 (FASTER/Bubble) 依賴「建圖-規劃-執行」串行管線
  - 極高動態下建圖延遲足以墜毀
  - **Event Camera 打破幀率限制**，異步脈衝直接反映環境動態邊緣
  - 結合 SNN（脈衝神經網路）或低延遲 Reactive → **像昆蟲一樣光流膨脹本能反射**
  - **從「幾何導航」→「仿生本能反應」跨代升級**
- **平台**：Skydio、DJI Mavic 3、Zipline 醫療無人機
- **Flight Corridor QP C++**：
  ```cpp
  for (int i = 0; i < num_control_points; ++i) {
      A = getPolyhedronA(i); b = getPolyhedronB(i);
      for (int j = 0; j < A.rows(); ++j) {
          // A_j · P_i ≤ b_j：B 樣條凸包性 → 所有控制點在內 → 整條曲線在內
          solver.addConstraint(A.row(j), control_points[i], b(j));
      }
  }
  ```

### 面試 talking points（Q2）
8. **CBF 前向不變集 + ḣ + αh ≥ 0**：安全控制 2020 後的統一公式
9. **CBF-QP 是 RL 的脊髓反射層**：RL + 硬安全保證的標準工業答題
10. **HOCBF 解 Relative Degree > 1**：位置約束二階系統的數學處理
11. **SARL Self-Attention 變長人群**：Social-Aware 現代架構核心
12. **Group-Aware 禮貌繞行**：服務型機器人商業化判準
13. **FASTER 雙軌跡並行**：高速 UAV 的安全備援設計
14. **B-Spline 凸包 + Flight Corridor**：非凸 3D 避障轉凸 QP 的關鍵
15. **Event Camera → 仿生本能反應**：下一代 UAV 避障範式的答題
