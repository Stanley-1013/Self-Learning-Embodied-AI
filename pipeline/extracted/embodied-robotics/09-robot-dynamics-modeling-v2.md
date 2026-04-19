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
