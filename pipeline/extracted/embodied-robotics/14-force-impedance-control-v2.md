# Extracted v2: 力控制與阻抗控制 — 補強素材 (session 531c9d80)

## Q1 補強（全身阻抗 + 螺旋探索）

### 情境題 A：Franka Panda 7軸全身阻抗+碰撞偵測
- Step1: 殘差觀測 τ_external = τ_measured - τ_model；>15N 閾值判定碰撞（ISO/TS 15066）
- Step2: 分級反應狀態機 — 輕度→零力拖動 / 中度→柔順回彈(彈簧耗能) / 重度→Cat.0 急停
- Step3: **零空間分層阻抗** — 主任務高 K_task / 零空間 K_null≈0（手肘輕柔避開、手端精度不變）

### 情境題 B：精密 Peg-in-Hole ±0.05mm
- Step1: 位置控接觸（Fz > 5N 停止下壓）
- Step2: **阿基米德螺旋探索**（XY 軌跡 + Z 恆定下壓力）；Fz 斷崖式下降=掉入
- Step3: 力位混合 — Z 力控 / XY 位控維持中心 / Yaw 柔順配合
- Step4: **自適應阻抗防卡阻** — 側向力異常→降 K 提 B，搖晃適應

### 面試 talking points
1. **關節力矩感測器 vs F/T sensor**：關節=全身碰撞偵測 vs F/T=末端直測（>1kHz 頻寬、無傳動誤差）
2. **Variable Impedance Learning**：固定 M/D/K 遇未知剛度環境震盪；RL（SAC/PPO）State=接觸力, Action=K,D

## Q2 補強（Impedance vs Admittance + Cartesian vs Joint + RL Variable Impedance）

### 情境題 C：Impedance vs Admittance 本質差異
- **因果關係**：
  - Impedance：位置誤差 → 力（虛擬彈簧阻尼器計算恢復力矩 τ）
  - Admittance：力 → 位置指令（F/T 讀值 → MDK 模型算新目標位置 → 底層位置環追蹤）
- **硬體條件決定選型**：
  - Impedance 需要**低摩擦驅動**（torque-controlled joints）；高摩擦減速器會吃掉微小柔順力矩
  - Admittance 需要**高剛性 + 高品質 F/T sensor**；不需要馬達精確控力
- **選型表**：
  - Impedance：Franka Panda、KUKA iiwa（協作機械臂、人機互動）
  - Admittance：ABB IRB、KUKA KR 大型系列（大負載工業機械臂 + 外掛 F/T）
- **Admittance 在剛性環境振盪陷阱**：
  1. F/T 測到剛性撞擊 → 大衝擊力
  2. 導納模型算出大位置修正量 → 瞬間抬起脫離接觸
  3. 力歸零 → 位置控制器拉回原位 → 再撞
  4. 通訊+位置環毫秒級延遲 → 因果鏈反向放大 → **極限環振盪 (Limit Cycle Oscillation)**
- **Hybrid Impedance/Admittance 架構**：
  - 自由空間 + 拖動示教 → Admittance（穩定性 + 絕對安全）
  - 接觸作業（插孔）→ 平滑切換到 Impedance（高頻力響應優勢）
- **數學**：τ = J^T(q)[M_d ë + B_d ė + K_d e] + τ_dynamics

### 情境題 D：Cartesian vs Joint Impedance 工程取捨
- **Cartesian Impedance**：τ = J^T(q) F_cartesian
  - 任務空間直覺（Z 軸軟、XY 硬），符合工程設計
  - **奇異點陷阱**：J(q) 降秩/條件數極大 → F_cartesian 乘病態 J^T → 關節力矩瞬間爆炸 → 硬體過載保護/減速機損壞
- **Joint Impedance**：τ = K_q(q_d - q) - D_q q̇
  - 計算極快、絕對不因奇異點爆炸
  - 失去任務空間直覺（無法單獨設 Z 軸軟）
- **Franka 雙模式設計理由**：奇異點附近或不可控碰撞時，底層切 Joint Impedance 當「彈簧兜底」防 Cartesian 矩陣崩潰
- **7-DoF 零空間補償**：
  - 投影矩陣 `(I - J^T J^{+T})` 把柔順控制投影到不影響末端位姿的 null space
  - 實現「手端精確、手肘可柔順避開」
  - τ_cmd = J^T F_cartesian + (I - J^T J^{+T}) τ_null
- **對稱正定矩陣陷阱**（面試進階）：
  - SE(3) 平移與旋轉耦合，K 不能只填對角線
  - 旋轉偏差必須用**單位四元數誤差**（Unit Quaternion Error）
  - 未設計 SPD + 正確旋轉誤差 → 大角度扭轉產生**非保守力** → 破壞 Passivity → 系統失穩狂抖

### 情境題 E：Variable Impedance Learning（RL 結合）
- **為什麼固定 K/D 不夠**：
  - 剛性金屬：K 小避免回彈振盪
  - 海綿柔性：K 大確保追蹤精度
  - 未知環境提前無法給定
- **RL 設計**：
  - State: `s = {e_x, ẋ, F_measured, τ_ext}`
  - Action: `a = {ΔK, ΔD}`（只輸出參數變化量，不輸出底層力矩）
  - 保證：底層 1kHz 阻抗控制器保持李雅普諾夫穩定性
- **SAC vs PPO 選型**：SAC 強勝出
  - 最大熵機制天然鼓勵多樣接觸策略探索
  - 對非線性摩擦 + 不確定接觸的魯棒性遠優於 PPO
- **「懶惰 Policy」陷阱**（必考）：
  - 只獎勵誤差 → RL 學到「K 開最大最僵硬 → 誤差最小」→ 失去柔順意義
  - 正確 reward: `R = -||e_x||² - w·||τ_cmd||²`（加能量懲罰）→ 強迫「需要時發力、平時放鬆」
- **Sim-to-Real 穿模陷阱**：
  - MuJoCo soft contact 允許微小穿透
  - RL 若發現 K→∞ 可「擠進」物體拿高分 → 學出作弊穿模抓取
  - 解法：(1) 接觸剛度 Domain Randomization (2) Action 層 Sigmoid 硬飽和
- **論文平台**：
  - ETH Zurich ANYmal 腿部阻抗適應
  - Franka 擦拭未知曲面（SAC + 可變阻抗）
- **神經網路動作映射**：`K_t = K_base + a_scale · tanh(NN_θ(s_t))`

### 面試 talking points（補強）
3. **Impedance/Admittance 選型 = 硬體邏輯**：能一句話講清楚「你的機械臂能控力矩還是只能控位置？」
4. **奇異點 + 高 K 的力矩爆炸**：面試必考，能說出 J^T 條件數如何放大誤差
5. **SPD + 四元數誤差**：Cartesian K 的正確設計，分辨「讀過論文」vs「只會套公式」
6. **Variable Impedance = 大腦慢思考、小腦快反射**：
   - 端到端 RL 輸出力矩（推理延遲 >10ms）會讓力控崩潰
   - RL 10Hz 輸出 K/D，底層 C++ 1kHz 阻抗控制
   - 融合「深度學習非線性適應」+「傳統控制 100% 高頻穩定保證」

## Q3 補強（Raibert Hybrid + Task Frame Formalism）

### 情境題 F：Raibert Hybrid Force/Position Control 與 Selection Matrix
- **互斥性數學**：同正交維度上「位置 X=10」+「力 Fx=5N」= 超定問題 (Overdetermined)
  - 環境剛性決定位置 → 力指令打架 → 積分飽和發散
- **Selection Matrix S（6×6 對角陣 0/1）**：
  - S 選出需力控維度，(I-S) 選出需位控維度
  - **數學上絕對正交** → 完美切分 6D 任務空間
- **Raibert 原始架構 (1981)**：
  `τ = J^T(q)·[S·(F_d - F_ext) + (I-S)·(K_p·e_x + K_d·ė_x)]`
- **時間複用 vs 空間複用**：
  - 空間：X 位控、Z 力控（穩定）
  - **時間**：依接觸狀態動態切 S（例：碰前 S=0，碰後 S=1）→ **Crisp switch 離散切換** → 指令跳變 → **切換震盪 Chattering**
  - 現代控制器偏好阻抗控制「軟過渡」
- **Hybrid vs Impedance 本質差異**（面試陷阱）：
  - Hybrid S = **Crisp Switch 非黑即白硬開關**，物理上強制切斷某維度位置反饋
  - Impedance M-B-K = **Continuous Blend 連續融合**，K 小則「軟化」該維度
  - Impedance 應對未知環境更穩，Hybrid 在已知幾何約束下力控精度更高
- **UR5 擦玻璃失敗案例**：
  - Z 軸純力控 S_zz=1，抹布滑出玻璃邊緣 → F_ext=0
  - 力控為達 10N 目標 → 不斷加速 Z → **機械臂高速砸地面**
  - 修正：純力控維度必加**速度飽和 (Velocity Saturation)** 或 **Damping Injection**
  - `wrench_cmd(2) = clamp(Kp_f * F_err, -max_vel, max_vel)`

### 情境題 G：Task Frame Formalism (Mason 1981) + 自然/人工約束
- **Task Frame**：接觸點建立局部座標系 {T}
- **Natural Constraints（自然約束）**：環境幾何決定，不能改變的事實
  - 位置自然約束：擦桌面時桌面剛性 → `v_z = 0` 必然
  - 力自然約束：空中揮舞無空氣阻力 → `f_z = 0` 必然
- **Artificial Constraints（人工約束）**：工程師目標
  - 力人工約束：擦桌面希望施加 `f_z = 10N` 按壓力
  - 位置人工約束：`v_x = 0.1 m/s` 擦拭速度
- **正交性定理**：
  - 任一維度上自然與人工約束**必定正交互補**
  - 環境約束位置（自然位置約束）→ 你只能在該維度施加力（人工力約束）
  - 反之亦然
  - **這就是 Hybrid Control 的 S 矩陣嚴格物理配置指南**
- **Task Frame 動態更新**：擦弧面 vs 擦平面 frame 旋轉；必須實時依接觸法向量更新
- **為什麼還是工業力控骨架**：Mason 的正交性給「哪一軸位控、哪一軸力控」提供唯一正確答案，無需憑經驗猜

## Q4 補強（Operational Space + SEA/QDD 硬體 + Tactile 觸覺）

### 情境題 H：Operational Space Formulation (OSC) + 6-DoF Wrench
- **Khatib 1987 OSC 核心**：直接把任務空間當獨立牛頓-歐拉系統
  - `Λ(q)·ẍ + μ(q,q̇) + p(q) = F_task`
- **操作空間慣量矩陣** `Λ = (J·M⁻¹·J^T)⁻¹`
  - **物理意義**：末端在當前姿態下，3D 空間表現的「有效質量/慣量」
  - 用手推末端時不同方向「感覺多重」完全由 Λ 決定
- **力/力矩不能混同一個 K**：
  - 單位不一致（N vs Nm）
  - 同一 K_{6×6} 沒有 scale invariance → 座標系變換時能量度量失效
- **Screw Theory 對偶空間**：
  - Twist V = [ω, v]^T ∈ ℝ⁶（運動）
  - Wrench W = [m, f]^T ∈ ℝ⁶（受力）
  - 對偶內積 `P = V^T·W` = 功率（做功速率）
- **Adjoint Transform 陷阱**：
  - Wrench 從夾爪中心 A 平移到工具尖端 B：`W_A = Ad_{T_{AB}}^T · W_B`
  - `Ad_T^T = [R, p̂·R; 0, R]`（p̂ = 平移向量反對稱矩陣）
  - **陷阱**：單純力 f 平移後會透過 `p̂·R·f` 被耦合出力矩 m
  - 開發者忽略 → 指令完全錯誤
- **對角陣 K 抓 30cm 長棍戳牆失穩之謎**（面試經典）：
  - Cartesian K = diag(K_x, K_y, K_z, K_wx, K_wy, K_wz) 預設剛度中心在法蘭盤原點
  - 30cm 外長棍尖端微小位移 → 純平移誤差投射回法蘭盤 → **力臂效應產生巨大旋轉誤差**
  - 對角陣無法提供平移↔旋轉非對角耦合恢復力
  - **破壞系統 Passivity → 非保守力 → 劇烈振盪**
- **Franka/KUKA iiwa FRI 強制要求**：K 必須相對當前 TCP 的嚴格對稱正定矩陣（SPD）
- **修正 C++ 實作核心**：
  ```cpp
  Eigen::Matrix6d Ad_T = ...;  // 法蘭盤 → TCP 的 Adjoint
  Ad_T.topLeftCorner(3,3) = R;
  Ad_T.bottomRightCorner(3,3) = R;
  Ad_T.bottomLeftCorner(3,3) = skew_symmetric(p) * R;
  Eigen::Matrix6d K_tcp = Ad_T.transpose() * K_flange * Ad_T;  // 一致性轉移
  ```

### 情境題 I：SEA + VSA + QDD 硬體哲學
- **SEA（Series Elastic Actuator）「用感測器解決致動器不完美」**：
  - 哲學：減速器有摩擦/背隙/測不準力 → 馬達與連桿間串聯物理彈簧
  - **彈簧 = 天然低通濾波器** → 瞬間吸收落地衝擊 → 保護諧波減速器
  - 高精度編碼器測彈簧變形 Δx → 虎克定律 `F = K·Δx` → **極純粹無摩擦干擾的真實接觸力**
  - ANYmal 四足實現極高頻力閉環
- **VSA（Variable Stiffness Actuator）「生物肌肉機械復刻」**：
  - 哲學：人搬重物肌肉繃緊、穿針引線肌肉放鬆
  - 兩拮抗馬達 + 非線性彈簧/凸輪 → **硬體層剛度物理可變**
  - 平台：DLR Hand-Arm System、IIT Walk-Man、MIT Nadia
  - 能瞬間變軟、儲存並釋放爆炸能量（投擲、跳躍）
- **QDD（Quasi-Direct Drive）「放棄彈簧，直接消摩擦」**：
  - 大扭矩外轉子馬達（盤式電機）+ 極低減速比 6:1～9:1 行星齒輪
  - 極佳**反向驅動性 Back-drivability**：外力輕易反推回馬達轉子
  - 電流環 I_q → 馬達電磁扭矩 `τ_m = K_t·I·N` → 幾乎等於外界真實接觸力矩
  - MIT Cheetah、Tesla Optimus 用 QDD → **無力感測器下的透明力控 + 跑跳**
- **面試選型三角「頻寬 × 衝擊 × 精度」**：
  - 0.05mm 骨科手術 = **Harmonic Drive + 末端 6 維 F/T**
  - 四足走廢墟（高頻地面衝擊）= **SEA**（實體彈簧保護減速器）
  - 人形機器人手臂（接球 / 握手）= **QDD**（低減速比換極致透明度 + 電流級力控）
- **QDD 無 F/T 估外力 Python 片段**：
  ```python
  tau_m = K_t * motor_current * N
  tau_dyn = compute_RNEA(q, q_dot, q_ddot)
  tau_ext = tau_m - tau_dyn - minor_friction(q_dot)  # QDD 摩擦極小
  ```

### 情境題 J：Tactile Sensing 整合到阻抗控制
- **F/T sensor 的致命盲區**：
  - 只測法蘭盤處總合 Wrench（淨力 / 淨力矩）
  - 抓滑溜杯子時指尖**局部微觀滑移 (Incipient Slip)**，淨力可能為零
  - F/T 根本不知物體正在掉落
- **Tactile Array 感測器對比**：
  - **GelSight / DIGIT**（Vision-based）：
    - 微型相機放彈性體背面
    - Photometric Stereo → 彈性體微觀形變 → 超高分辨率 3D 深度圖 + 剪切力分佈
    - 擅長**紋理識別、微滑移檢測**
  - **Xela Robotics uSkin**（磁場 / 霍爾效應）：
    - 響應速度極快 >100Hz
    - 每 Taxel（觸覺像素）直接輸出 3D 力向量
    - 適合**閉環即時控制**
- **Tactile-based Impedance Modulation**：
  - 傳統 K/D 固定或任務預設
  - 有觸覺陣列 → **空間阻抗調變**
  - 檢測到「接觸面積變小」或「剪切力邊緣微滑移」→ 瞬間在滑移法向增 K（捏緊）、切向增 D（耗散滑動能量）
- **Proprioception + Tactile Fusion**（Franka / Tesla Optimus 最佳策略）：
  - **宏觀 Proprioception**：關節扭矩殘差檢測手臂與外界大規模碰撞 / 承重
  - **微觀 Tactile**：指尖 DIGIT 處理盲區內接觸法向量、摩擦錐邊界、局部物體位姿
- **面試 talking point「未來觸覺比視覺更重要」**：
  - Dexterous manipulation 階段視覺遭無解**遮擋災難 (Occlusion)** — 手包覆物體時看不見
  - 視覺無法測「摩擦係數、質量分佈、接觸剛度」
  - **「在最後的 5 公分，觸覺比視覺更重要」**
  - 未來：Tactile Array 形變特徵 → RL State → 輸出 ΔK/ΔD → 底層阻抗控制
  - **像人類盲解魔術方塊般的極致手感**
- **Tactile 動態阻抗調節片段**：
  ```python
  slip, contact_area = analyze_tactile_flow(tactile_image)
  if slip > THRESHOLD:
      K[z] += 200 * slip  # 法向捏緊
      D[x] += 10 * slip; D[y] += 10 * slip  # 切向耗散
  if contact_area < SMALL:
      K *= 0.5  # 尖銳接觸保護物體
  ```

## Q5 補強（Contact-Rich 前沿 + Residual RL Assembly + HRI）

### 情境題 K：Contact-Rich Manipulation 近期突破（2022-2024）
- **ACT (Action Chunking Transformer) + ALOHA 雙手成功密碼**：
  - 傳統 BC 單步預測 `a_t = π(s_t)` → 微小誤差幾何級數放大（Compounding Error）
  - ACT 引入 **Action Chunking** → 一次預測未來 k 步絕對位姿序列 `a_{t:t+k}`
  - 1000Hz 底層控制 + 絕對位姿追蹤 → 過濾高頻遙操作抖動
  - 成功任務：穿針引線、打雞蛋（精細雙手協作）
- **Diffusion Policy 為何贏過 Transformer/MSE**：
  - Contact-rich 任務常有多模態動作分佈（「從左繞」vs「從右繞」）
  - MSE-based Transformer 會平均兩解 → 機械臂直撞障礙物
  - Diffusion Policy：從高斯雜訊逐步去噪 → 精準擬合多峰分佈 + 天然平滑
  - 目標函數：`L = E[||ε - ε_θ(x_k, k, O)||²]`
- **Visuomotor Policy (RT-2 / OpenVLA / π0) × 阻抗控制**：
  - VLM 推理慢（1-10 Hz），直接輸出力矩會崩
  - **現代分層架構**：
    - VLM「大腦」：低頻輸出低階目標位姿 `x_target` (waypoints)
    - 「小腦」1kHz 笛卡爾阻抗 `τ = J^T(K(x_target - x) - D·ẋ)`
    - 大模型指方向，接觸力柔順處理全交給底層阻抗控制
- **Dexterous Manipulation SOTA**：
  - **ANYskin**：高密度磁性觸覺皮膚 + RL，觸覺特徵嵌 state，解視覺遮擋災難
  - **DextAH**：非對稱 Teacher-Student，Teacher 讀 privileged（摩擦/質量），Student 只看五指 proprioception 隱式推斷
- **面試 talking point「contact-rich 是未來 3 年決勝戰場」**：
  - 純視覺 AI 已被大模型基本解決
  - 物理落地無法純靠視覺 — 插拔線束/精密組裝時視覺遭嚴重遮擋
  - 摩擦/材質形變等非線性接觸動力學 VLM 看不到
  - **誰能融合觸覺陣列 + Diffusion Policy + 底層阻抗控制，誰就能讓具身智能走進工廠與家庭**

### 情境題 L：Peg-in-Hole / Assembly 現代 Learning 解法
- **Residual RL Policy（100× 樣本效率）**：
  - 基礎控制器 π_base（CTC + Impedance）處理 90% 已知動力學
  - RL 網路 π_θ 只輸出微小殘差 Δx 或 Δτ，擬合 10% 未建模非線性接觸 + 摩擦
  - 探索空間極大壓縮 + 底層兜底安全 + 樣本效率 ×100
  - 控制律：`τ_cmd = M(q)·q̈_d + C·q̇ + G(q) + K_p·e + K_d·ė + π_θ(s_t, F_ext)`
- **Force-Conditional Diffusion**：
  - 六維 F/T 時序訊號經 1D-CNN 抽特徵 → 擴散模型的 Condition
  - 卡阻時 policy 依力覺生成「搖晃試探 (Wiggle)」解鎖軌跡
- **Sim-to-Real for Assembly 要隨機化什麼**（不只 Domain Randomization 視覺）：
  - **接觸模型是命脈**：
    - 接觸剛度與阻尼（MuJoCo `solimp`, `solref`）
    - 摩擦錐係數（靜/滑動）
    - 零件尺寸公差（peg/hole 半徑 ±0.1mm 隨機縮放）
- **「大孔徑成功、微米精密還是傳統」陷阱**：
  - AI 在 >1mm 公差插入很成功
  - 微米級（航太插頭、手錶排線）仍是 Force/Position Hybrid 天下
  - **原因**：MuJoCo Soft Contact 允許微小穿透保梯度連續 → 純 RL 會學「利用物理 Bug 穿透零件」作弊 → 真機金屬卡死崩潰
- **ROS 2 MoveIt Servo 殘差疊加 C++**：
  ```cpp
  Twist base_twist = impedance_controller.compute(...);
  Twist residual_twist = rl_agent.predict({error, wrench});
  final_cmd.linear.z = clamp(base + residual, -MAX_VZ, MAX_VZ);  // 安全邊界
  servo_node->publish_twist_command(final_cmd);
  ```

### 情境題 M：HRI 中的阻抗控制
- **Kinesthetic Teaching / Transparent Mode**：
  - 阻抗公式 `M·ẍ + D·ẋ + K·x = F_ext`，K→0, D→極小 → Transparent Mode
  - **避免重力塌下**：極精準動力學前饋 `τ_ff = G(q) + τ_fric(q,q̇)` 實時抵消
  - 人類只施加幾 N 力克服殘餘慣量 → 機械臂如絲滑隨手移動，鬆手即停
- **分辨「主動引導」vs「意外碰撞」**：
  - **頻譜分析**：
    - 意外碰撞：剛性接觸、dF/dt 極大 → 高頻衝擊脈衝
    - 主動拖拽：人類肌肉發力 → 低頻 < 2 Hz
    - FFT 或高通濾波器瞬間剝離碰撞信號 → 觸發急停
  - **觸覺位置分佈**：
    - 引導：多指大面積穩態包覆（Grasp/Push）
    - 碰撞：單點尖銳接觸
- **Momentum Observer 碰撞檢測**（無需 F/T sensor）：
  - `r(t) = K_o·[M(q)·q̇ - ∫(τ_cmd + C^T·q̇ - G(q) + r(s))ds]`
  - 廣義動量定理估外部接觸力矩 → 超閾值且變化率大 → ISO 15066 PFL 煞車
- **Shared Control / Virtual Fixtures**（da Vinci 外科機器人）：
  - 安全區域 K=0，醫生完全主導
  - 刀尖逼近血管/神經邊界 → K 動態急劇拉高 → 大排斥力 haptic feedback 強制阻止危險切入
- **HRI + BCI 的未來交會點**（面試 talking point）：
  - 腦機介面信號頻寬低延遲高，直接控位置極度危險
  - 未來 Shared Autonomy：
    - 大腦 BCI 輸出高層方向意圖
    - 阻抗控制底層接管所有物理接觸、柔順避障、力學約束
  - 阻抗控制補足 BCI 缺失的「脊髓反射」
  - **殘障人士安全使用具身智能的唯一路徑**
- **Franka libfranka 零力模式 C++**：
  ```cpp
  auto zero_force = [&](const RobotState& s, Duration dt) -> Torques {
      auto coriolis = model.coriolis(s);
      auto gravity = model.gravity(s);
      return {0,0,0,0,0,0,0};  // Franka 內部已補重力/科氏力；僅輸出 0 力矩 = 人拉哪走哪
  };
  robot.control(zero_force);  // 1kHz torque control 迴圈
  ```

### 面試 talking points（Q3+Q4+Q5 補強）
7. **Selection Matrix 正交性定理**：S 把 6D 任務空間切兩子空間；Natural ⊥ Artificial Constraint 的物理配置指南
8. **Adjoint Transform 力矩耦合陷阱**：wrench 跨系轉換 `p̂·R·f` 衍生力矩；對角陣 K 在遠離原點抓長棍失穩
9. **SEA / VSA / QDD / Harmonic Drive 選型三角**：頻寬 × 衝擊 × 精度；面試「你選什麼執行器」必備
10. **觸覺比視覺更重要（最後 5 公分）**：Occlusion 災難 + 摩擦/質量/剛度無法用視覺測
11. **ACT Action Chunking + Diffusion Policy 多峰分佈**：contact-rich 2022-2024 突破的兩大支柱
12. **Residual RL + 傳統控制器**：樣本效率 ×100 + 安全性底層兜底；vs 純 RL 的產業落地答案
13. **Transparent Mode 重力補償 + Momentum Observer 碰撞檢測**：HRI 零力拖動示教的核心雙技能
14. **阻抗控制 = BCI 脊髓反射**：未來具身智能 × 腦機介面的交會點
