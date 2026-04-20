---
title: "力控制與阻抗／導納控制"
prerequisites: ["13-pid-control-tuning"]
estimated_time: 60
difficulty: 4
tags: ["force-control", "impedance", "admittance", "hybrid", "compliance", "operational-space", "variable-impedance", "tactile-sensing", "contact-rich", "hri"]
sidebar_position: 14
---

# 力控制與阻抗／導納控制

## 你將學到

- 兩句話精確講清楚 impedance 與 admittance 的因果差異，並能在 10 秒內從「你的驅動能不能控力矩」這一硬體問題反推出該用哪種架構
- 面對「UR5 擦玻璃滑出邊緣砸地面」「Franka 研磨抖動」「長棍戳牆振盪」「Peg-in-Hole 卡阻」「手引示教往下塌」這類現場災難，能立刻指名病因（Selection Matrix 維度誤、對角 K 非 SPD、重力補償漂移、Admittance 在剛性環境極限環）並選對修法
- 面試被問 Selection Matrix 正交性定理、Cartesian vs Joint Impedance 的奇異點陷阱、Operational Space 的 Adjoint Transform、SEA/VSA/QDD 執行器選型、Variable Impedance RL、ACT + Diffusion Policy 的分層架構、Momentum Observer 碰撞偵測、HRI + BCI 交會點時，每題都能在兩分鐘內講清楚關鍵邏輯
- 掌握工業級力控落地真實流程：硬體選型 → F/T 動態補償 → 接觸狀態機 → Task Frame Formalism → 分層架構 (大腦 VLM 慢 + 小腦阻抗快) → ISO/TS 15066 功能安全通過

## 核心概念

**精確定義**：**力控制（Force Control）** 是讓機器人主動調節與環境接觸力的控制策略，與位置控制互補 — 位置控制追蹤幾何軌跡（剛性任務），力控制調節接觸力（柔順任務）。裝配、研磨、手引示教、人機協作、具身操作等「不能硬撞」的場景，核心都是力控。

**阻抗控制（Impedance Control）**：讓末端表現得像虛擬的 **質量-阻尼-彈簧** 系統。因果方向是 **位移偏差 → 輸出力**（感測位移，控制力）。適合**能直接輸出力矩**的硬體（torque-controlled joints、QDD、SEA）。

**導納控制（Admittance Control）**：因果方向相反 — **外力輸入 → 輸出位移修正**（感測力，控制位移）。適合**剛性高減速比的大慣量工業臂**，因為這類硬體位置控制精度高、但直接做力控困難。

**力位混合控制（Hybrid Position/Force，Raibert & Craig 1981）**：用 Selection Matrix $S$ 把任務空間按自由度分解 — 法向做力控、切向做位控，讓同一個控制迴圈在不同維度上執行互斥策略。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：末端 wrench（來自 F/T sensor、關節力矩感測器、或電流推算 + Momentum Observer）、末端位姿（來自 FK + encoder）、期望力 $F_d$ 與期望位姿 $x_d$（來自規劃器或 VLM 高階指令）、觸覺陣列訊號（來自 GelSight / DIGIT / Xela）
- **輸出**：關節力矩指令 $\tau$（impedance）或關節位置修正 $\Delta q$（admittance），送入底層關節控制器（Ch13 PID）
- **上游**：Ch09 動力學模型做重力/慣性補償、trajectory planner 或 VLM policy 給 $x_d$
- **下游**：依賴 Ch13 PID 做底層關節跟蹤；輸出的柔順行為直接影響裝配成功率、研磨品質、ISO/TS 15066 人機安全通過
- **閉環節點**：位於 **控制** 階段的最外層迴圈，替代或疊加在位置控制之上；在具身智能分層架構裡，力控是**脊髓反射層（1 kHz 硬即時）**，上方是小腦（MPC / ACT / Diffusion Policy 100 Hz）、大腦（VLM / π0 1-10 Hz）

**一句話版本**：「力控是賦予機器人觸覺反射的神經樞紐 — 碰到東西時像人手一樣柔順收力，而不是像鑽頭一樣硬撞；也是 AI 大模型給出『插入這個孔』這種高階指令時，底層能安全落地的物理保證。」

### 最少夠用的數學

**1. 阻抗控制方程**（末端表現為虛擬質量-阻尼-彈簧）：

$$
F = M_d \ddot{e} + D_d \dot{e} + K_d e, \quad e = x - x_d
$$

**物理意義**：$M_d$ 決定加速反應快慢、$D_d$ 決定能量耗散速度、$K_d$ 決定偏離期望位置時的回復力大小。三者獨立調整，讓末端的接觸行為像你設計的彈簧系統；$K_d \to 0$ 變成「手引示教零力感」、$K_d \to \infty$ 趨近位控。

**2. 導納控制方程**（因果反轉：力 → 位移修正）：

$$
M_d \ddot{x}_{\text{cmd}} + D_d \dot{x}_{\text{cmd}} + K_d x_{\text{cmd}} = F_{\text{ext}}
$$

**物理意義**：感測到外力 $F_{\text{ext}}$ 後，解這個 ODE 得到位置修正量 $x_{\text{cmd}}$，再送給底層位置控制器。外力越大 → 修正越遠；$K_d = 0$ 時變成純阻尼模式（手引示教）。

**3. 力位混合 — Selection Matrix**：

$$
\tau = J^T(q)\left[S \cdot F_{\text{force}} + (I - S) \cdot F_{\text{pos}}\right]
$$

**物理意義**：$S$ 是 6×6 對角矩陣，每個對角元素 0 或 1。$S$ 選出需力控維度，$(I - S)$ 選出需位控維度；$S$ 與 $(I - S)$ 在數學上**絕對正交** → 完美切分 6D 任務空間，避免「同一維度又要位置 = 10 又要力 = 5N」的超定問題。

**4. Raibert 原始架構（1981）**：

$$
\tau = J^T(q)\cdot\left[S\cdot(F_d - F_{\text{ext}}) + (I-S)\cdot(K_p\cdot e_x + K_d\cdot \dot{e}_x)\right]
$$

**物理意義**：力控維度追蹤期望接觸力 $F_d$，位控維度用標準 PD 追蹤軌跡誤差；$J^T$ 把兩種任務空間力映射回關節力矩。

**5. 關節力矩 ↔ 末端力的映射**：

$$
\tau = J^T(q)\, F
$$

**物理意義**：Jacobian 轉置把末端六維 wrench 映射回每個關節需出的力矩，是力控從任務空間轉回關節空間的核心橋梁。Jacobian 降秩（奇異點）時這個映射會放大誤差 — 高 K 配奇異點 = 關節力矩爆炸。

**6. Operational Space Formulation（Khatib 1987）**：

$$
\Lambda(q)\ddot{x} + \mu(q, \dot{q}) + p(q) = F_{\text{task}}, \quad \Lambda = (J M^{-1} J^T)^{-1}
$$

**物理意義**：$\Lambda$ 是**操作空間慣量矩陣**，代表末端在當前姿態下 3D 空間表現的「有效質量」— 同一台機械臂，姿態不同、用手推末端感覺的「重量」完全不同，由 $\Lambda$ 描述。$\mu, p$ 分別是任務空間的 Coriolis 與重力項。

<details>
<summary>深入：阻抗控制的完整動力學推導與 Passivity 穩定性分析</summary>

**從任務空間動力學出發**：

機械臂在任務空間的動力學方程（忽略摩擦）：

$$
\Lambda(x)\ddot{x} + \mu(x, \dot{x})\dot{x} + p(x) = F + F_{\text{ext}}
$$

**阻抗控制律推導**：

目標：讓末端在接觸時表現為 $M_d \ddot{e} + D_d \dot{e} + K_d e = -F_{\text{ext}}$。

設計控制力：

$$
F = \Lambda(x)\ddot{x}_d + \mu(x, \dot{x})\dot{x} + p(x) - \Lambda(x) M_d^{-1}(D_d\dot{e} + K_d e)
$$

前三項做**動力學前饋**（補償慣性、Coriolis、重力），最後一項注入期望阻抗行為。

轉回關節空間：

$$
\tau = J^T F + \left(I - J^T J^{+T}\right)\tau_0
$$

其中 $(I - J^T J^{+T})\tau_0$ 是冗餘機械臂的 null-space 項，實現「末端任務精確 + 手肘柔順避開」的雙層控制。

**Passivity 穩定性分析**：

系統儲存函數（Lyapunov candidate）：

$$
V = \frac{1}{2}\dot{e}^T M_d \dot{e} + \frac{1}{2}e^T K_d e
$$

對時間微分：$\dot{V} = -\dot{e}^T D_d \dot{e} \leq 0$（前提：$D_d$ 正定）

因此 $D_d > 0$ 是穩定的**必要條件**；$K_d > 0$ 保證平衡點唯一；$K_d$ 必須為**對稱正定矩陣（SPD）**才能保證 Passivity，否則旋轉偏差產生的「非保守力」會破壞能量守恆 → 系統失穩狂抖。

**實務要點**：
- $M_d$ 設太小（反應太快）→ 控制力超過執行器上限 → 飽和 → 失穩
- $K_d / D_d$ 比值過大（欠阻尼）→ 接觸時振盪
- 完美動力學前饋依賴精確動力學模型；模型誤差需靠魯棒項或自適應律補償

</details>

<details>
<summary>深入：Cartesian vs Joint Impedance — 奇異點力矩爆炸與 7-DoF 零空間補償</summary>

### Cartesian Impedance — 直覺但危險

控制律：$\tau = J^T(q)\cdot F_{\text{cartesian}}$

**優點**：任務空間直覺（Z 軸軟、XY 硬），完全符合工程設計思維。

**奇異點陷阱（面試必考）**：
- 奇異點 $\det J(q) \to 0$ → $J$ 降秩、條件數極大
- $F_{\text{cartesian}}$ 乘病態的 $J^T$ → **關節力矩瞬間爆炸**
- 硬體過載保護觸發、減速機損壞、ISO 13849 急停

### Joint Impedance — 穩定但失去直覺

控制律：$\tau = K_q(q_d - q) - D_q\dot{q}$

**優點**：
- 計算極快（對角乘法）
- 絕對不因奇異點爆炸（$J$ 未進入計算）

**缺點**：失去任務空間直覺 — 無法單獨設「Z 軸軟、XY 硬」，因為關節角度與任務方向非一對一對應。

### Franka Panda 雙模式設計理由

在奇異點附近或不可控碰撞瞬間，底層**自動切換到 Joint Impedance 當「彈簧兜底」**，防止 Cartesian 矩陣計算崩潰。這是 Franka 能在 ISO/TS 15066 認證中拿到高分的關鍵設計。

### 7-DoF 冗餘零空間補償

7 軸機械臂有 1 個冗餘自由度 → 同一末端位姿對應無窮多組關節角 → 可同時執行末端任務 + 零空間次任務：

$$
\tau_{\text{cmd}} = J^T F_{\text{cartesian}} + \left(I - J^T J^{+T}\right)\tau_{\text{null}}
$$

**投影矩陣** $(I - J^T J^{+T})$ 把 $\tau_{\text{null}}$ 投影到「不影響末端位姿」的零空間。

**工程應用**：
- 主任務：末端高 $K_{\text{task}}$（精準追蹤目標位姿）
- 零空間：低 $K_{\text{null}} \approx 0$（手肘柔順、可避開人或障礙）
- 實現「手端精確、手肘輕柔可被推開」的雙層柔順

**平台**：Franka Emika Panda 與 KUKA iiwa 標配此架構。

### 對稱正定矩陣（SPD）陷阱

Cartesian K 不能只填對角線！SE(3) 平移與旋轉是耦合的：

- 旋轉偏差必須用**單位四元數誤差（Unit Quaternion Error）**，不能直接用歐拉角差
- K 矩陣必須是**對稱正定矩陣（SPD）**
- 未設計 SPD + 錯誤旋轉誤差 → 大角度扭轉產生**非保守力** → 破壞 Passivity → 狂抖失穩

Franka 的 libfranka API 強制要求 Cartesian K 為 SPD，這是「讀過 Hogan 論文」vs「只會套公式」的分水嶺。

</details>

### Impedance vs Admittance — 因果差異與選型決策

**因果方向對比**：

| 面向 | Impedance | Admittance |
|------|-----------|------------|
| 因果 | 位置誤差 → 力（虛擬 MDK 算恢復力矩 $\tau$） | 力 → 位置指令（F/T 讀值 → MDK 模型算新目標位置 → 底層位置環追蹤）|
| 硬體需求 | **低摩擦驅動**（torque-controlled joints），高摩擦減速器會吃掉微小柔順力矩 | **高剛性 + 高品質 F/T sensor**，不需馬達精確控力 |
| 代表平台 | Franka Panda、KUKA iiwa、MIT Cheetah 腿 | ABB IRB、KUKA KR 大型系列（外掛 F/T） |
| 典型應用 | 協作機械臂、人機互動、腿式機器人 | 大負載工業機械臂、精密裝配 |

<details>
<summary>深入：Admittance 在剛性環境的極限環振盪陷阱 + Hybrid 架構</summary>

### Admittance 撞剛性牆的 Limit Cycle Oscillation

**災難鏈**：
1. F/T sensor 測到剛性撞擊 → 瞬間大衝擊力
2. 導納模型算出大位置修正量 → 機械臂瞬間抬起脫離接觸
3. 外力歸零 → 位置控制器拉回原位置 → 再撞
4. 通訊 + 位置環毫秒級延遲 → 因果鏈反向放大 → **極限環振盪（Limit Cycle Oscillation）**

這是 Admittance 的致命弱點：在剛性環境（金屬對金屬）下天然不穩定。

### Hybrid Impedance/Admittance 架構

工業解法 — **動態切換**：

| 階段 | 策略 | 理由 |
|------|------|------|
| 自由空間 + 拖動示教 | Admittance | 穩定性 + 絕對安全 |
| 接觸作業（插孔、研磨） | Impedance | 高頻力響應、不怕極限環 |
| 奇異點附近 | Joint Impedance | 避免 Cartesian 矩陣爆炸 |

**切換關鍵**：使用 ramp 函數平滑過渡 $K_d, D_d$，避免切換瞬間的力矩尖峰。

### 一句話判別

「你的機械臂的驅動器**能不能直接精確控力矩**？」
- 能 → Impedance
- 不能 → Admittance
- 兩者都要 → Hybrid

這是 10 秒內從硬體反推架構的必殺句。

</details>

### 力感測方案比較（直接影響控制頻寬和精度）

| 感測方式 | 頻寬 | 優勢 | 劣勢 | 代表平台 |
|----------|------|------|------|----------|
| 電流推算 | 高（kHz 級） | 無額外硬體、頻寬高 | 摩擦/背隙干擾大、精度差 | 通用伺服驅動 |
| 關節力矩感測器 | 中-高 | 全身碰撞偵測、整臂柔順 | 成本高、需每關節裝 | KUKA iiwa、Franka |
| 末端 F/T sensor | 中（>1kHz） | 六維末端力完整量測、無傳動誤差 | 需重力補償、安裝增加末端慣量 | ATI、Robotiq FT 300 |
| 觸覺陣列 | 高（>100Hz） | 局部微滑移檢測、摩擦錐邊界 | 覆蓋面積小、處理複雜 | GelSight、DIGIT、Xela uSkin |

<details>
<summary>深入：Operational Space + Adjoint Transform — 為什麼對角 K 抓長棍會失穩</summary>

### Screw Theory 對偶空間

- **Twist** $V = [\omega, v]^T \in \mathbb{R}^6$（運動，角速度+線速度）
- **Wrench** $W = [m, f]^T \in \mathbb{R}^6$（受力，力矩+力）
- 對偶內積 $P = V^T\cdot W$ = **功率**（做功速率）

這個對偶結構告訴我們：wrench 與 twist 在 SE(3) 上不能隨便混在同一個矩陣裡，單位就不一致（N vs Nm）。

### Adjoint Transform — 跨座標系 wrench 轉換

把 wrench 從夾爪中心 A 平移到工具尖端 B：

$$
W_A = \mathrm{Ad}_{T_{AB}}^T \cdot W_B, \quad \mathrm{Ad}_T^T = \begin{bmatrix}R & \hat{p}R \\ 0 & R\end{bmatrix}
$$

其中 $\hat{p}$ 是平移向量的反對稱矩陣。

**陷阱**：單純的力 $f$ 平移後會透過 $\hat{p}\cdot R\cdot f$ 被**耦合出額外力矩** $m$ ！開發者忽略這項 → 指令完全錯誤。

### 對角陣 K 抓 30cm 長棍戳牆失穩之謎（面試經典）

**情境**：機械臂抓一根 30cm 長棍子戳牆，用 Cartesian K = diag($K_x, K_y, K_z, K_{wx}, K_{wy}, K_{wz}$) 預設剛度中心在法蘭盤原點。

**失穩機制**：
1. 長棍尖端微小位移（純平移誤差）
2. 投射回法蘭盤座標系時 → **力臂效應產生巨大旋轉誤差**
3. 對角陣 K 無法提供平移↔旋轉的非對角耦合恢復力
4. **破壞系統 Passivity** → 非保守力累積 → 劇烈振盪

### 修正方案 — Adjoint 轉換一致性

Franka / KUKA iiwa FRI API 強制要求：K 必須**相對當前 TCP（工具中心點）** 的嚴格 SPD 矩陣。

```cpp
Eigen::Matrix6d Ad_T;
Ad_T.topLeftCorner(3,3)     = R;
Ad_T.bottomRightCorner(3,3) = R;
Ad_T.bottomLeftCorner(3,3)  = skew_symmetric(p) * R;  // 力臂耦合項
Ad_T.topRightCorner(3,3)    = Eigen::Matrix3d::Zero();
Eigen::Matrix6d K_tcp = Ad_T.transpose() * K_flange * Ad_T;  // 一致性轉移
```

### 操作空間慣量矩陣 $\Lambda$ 的物理直覺

$\Lambda(q) = (J M^{-1} J^T)^{-1}$

用手推末端時不同方向「感覺多重」完全由 $\Lambda$ 決定：
- 推 X 方向感覺沉 → $\Lambda_{xx}$ 大
- 推 Y 方向感覺輕 → $\Lambda_{yy}$ 小
- 同一末端位姿、不同關節配置 → $\Lambda$ 完全不同

這就是為什麼高級阻抗控制要做 Λ-weighted dynamics decoupling，才能讓「設定 $K_d = 500$」在任何姿態下都表現一致。

</details>

## 直覺理解

**阻抗 = 拉著彈力繩握方向盤**：你握著方向盤（期望位置 $x_d$），手臂和方向盤之間有一條彈力繩（$K_d$）加一個阻尼器（$D_d$）。路面顛簸（外力）讓方向盤偏離，你的手不是鎖死（純位控），而是允許一定偏移再柔柔拉回。$K_d$ 越大 = 繩越緊 = 偏離越小但越容易震；$D_d$ 越大 = 阻尼越重 = 回來越慢但越穩。

**導納 = 推超市推車**：你推推車（施加外力 $F_{\text{ext}}$），推車根據你推的力決定移動多快、移動多遠。推越大力 → 走越遠。導納控制器就像推車的質量和摩擦力，決定「這股力要換算成多少位移」。工業臂硬體剛性高（像重型推車），不適合直接做力控，但很適合「你告訴我力有多大，我算出該移多少」的導納模式。

**力位混合 = 用橡皮擦擦黑板**：手的法向（壓向黑板）做力控 — 維持恆定壓力讓橡皮擦貼合；切向（左右滑動）做位控 — 精確走擦拭軌跡。Selection Matrix 就是「這個方向聽力的、那個方向聽位置的」物理配置指南。

**Variable Impedance = 人體肌肉繃鬆切換**：搬重物時手臂繃緊（高 K 維持姿勢），穿針引線時手臂放鬆（低 K 容許微調）；VSA（Variable Stiffness Actuator）就是機械層復刻這個能力，RL Variable Impedance 則是軟體層復刻。

**觸覺感測 = 盲人摸索**：F/T sensor 只測「整隻手受到的總合力」，無法分辨「指尖的微滑移」；觸覺陣列像盲人手指的指腹，每個 taxel（觸覺像素）都能獨立感受。抓滑溜杯子時淨力可能為零，但指尖的切向微滑移已經在發生 — 只有觸覺陣列看得到。

**模擬器觀察**：
- **MuJoCo**：設定 UR5 + F/T sensor 推彈簧牆面。先跑純位控 — 碰到牆面力會瞬間飆高甚至彈飛。切成阻抗控制（$K_d = 500, D_d = 50$）— 柔順壓入、力平滑收斂。$K_d \to 5000$ → 接近位控（硬撞）；$K_d = 0$ + 小 $D_d$ → 手引示教模式。**注意：MuJoCo 的 soft contact 允許微小穿透**，是 RL 訓練的 Sim-to-Real 陷阱（見情境題 L）。
- **Isaac Sim**：用 Franka 執行 Peg-in-Hole，觀察 Variable Impedance 策略如何在卡阻瞬間自動降 K 提 D 搖晃解鎖。
- **Gazebo**：ANYmal 四足落地瞬間，SEA 的彈簧在力矩訊號上會呈現明顯的 50-100Hz 衰減震盪 — 這是「彈簧吸收衝擊」的肉眼可見證據。

## 實作連結

**三個典型工程場景**：

1. **協作機器人手引示教（Lead-through Teaching / Kinesthetic Teaching）**：操作員直接用手推機械臂，機器人跟著走，鬆手即停。核心是 Admittance（或 Transparent Mode Impedance）+ $K_d = 0$（無位置回復力），只保留小 $D_d$ 讓手感平滑。關鍵前提：精確重力補償 + 摩擦補償 — 否則機器人自重讓操作員覺得「推不動」或「往下掉」。

2. **表面研磨/拋光（Surface Finishing）**：Hybrid 架構 — 法向用 PI 力控維持 10 N 恆定壓力、切向用位控走研磨軌跡。需要 F/T sensor 動態補償（扣除工具重力和慣性力）+ 自適應剛度（遇曲率變化時調整 $K_d$）+ 速度飽和保護（防止離開工件時機械臂狂撞地面）。

3. **精密裝配（Peg-in-Hole）**：位控接近 → 接觸偵測 → 螺旋搜尋 → 力控插入。需要狀態機管理切換，切換瞬間最容易出衝擊力。現代做法結合 Residual RL 處理未建模摩擦（見情境題 L）。

**Code 骨架**（C++，ROS 2 + ros2_control 架構）：

```cpp
// Impedance controller — 在 ros2_control 的 update() 迴圈中
// 輸入：current_pose, desired_pose, measured_wrench
// 輸出：joint_torques
// 頻率：1 kHz（硬即時）

Eigen::Vector6d pose_error = compute_pose_error_quat(current_pose, desired_pose);
Eigen::Vector6d vel_error  = current_velocity - desired_velocity;

// 1. 計算 Adjoint 把 K 從法蘭盤轉到 TCP（見深入塊）
Eigen::Matrix6d K_tcp = compute_adjoint_K(K_flange, T_flange_to_tcp);
Eigen::Matrix6d D_tcp = compute_adjoint_K(D_flange, T_flange_to_tcp);

// 2. 阻抗力 = K * e + D * ė（Cartesian）
Eigen::Vector6d F_impedance = K_tcp * pose_error + D_tcp * vel_error;

// 3. 加上力跟蹤誤差（若有期望力 Fd）
Eigen::Vector6d F_total = F_impedance + F_feedforward;

// 4. 轉回關節空間
Eigen::VectorXd tau = jacobian.transpose() * F_total;

// 5. 加動力學補償（重力 + Coriolis）
tau += gravity_compensation(q) + coriolis_compensation(q, qd);

// 6. 7-DoF 零空間補償（手肘柔順）
Eigen::MatrixXd null_projector = Eigen::MatrixXd::Identity(7,7)
                                 - jacobian.transpose() * jacobian_pinv.transpose();
tau += null_projector * tau_null;

// 7. 奇異點保護：若 J 條件數過大，切 Joint Impedance 兜底
if (jacobian.jacobiSvd().singularValues().tail(1)(0) < SINGULAR_THRESHOLD) {
    tau = K_joint * (q_desired - q) - D_joint * qd;
}

hardware_interface->set_command(tau);
```

<details>
<summary>深入：完整 Python 導納控制實作（MuJoCo 環境）+ 重力補償校準</summary>

```python
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R

class AdmittanceController:
    """
    導納控制器：F_ext → 位移修正 → 送入位置控制器
    適用場景：高減速比工業臂的柔順控制
    """
    def __init__(self, Md, Dd, Kd, dt=0.001):
        self.Md = np.diag(Md)  # 期望慣量 [6,]
        self.Dd = np.diag(Dd)  # 期望阻尼 [6,]
        self.Kd = np.diag(Kd)  # 期望剛度 [6,]
        self.dt = dt
        self.x_cmd = np.zeros(6)
        self.x_cmd_dot = np.zeros(6)

    def update(self, F_ext: np.ndarray) -> np.ndarray:
        """
        輸入：六維外力 F_ext（需已扣除工具重力）
        輸出：六維位移修正 x_cmd
        """
        Md_inv = np.linalg.inv(self.Md)
        x_cmd_ddot = Md_inv @ (F_ext - self.Dd @ self.x_cmd_dot
                                      - self.Kd @ self.x_cmd)

        # 半隱式歐拉積分（比顯式穩定）
        self.x_cmd_dot += x_cmd_ddot * self.dt
        self.x_cmd += self.x_cmd_dot * self.dt

        return self.x_cmd.copy()

    def reset(self):
        self.x_cmd = np.zeros(6)
        self.x_cmd_dot = np.zeros(6)


def calibrate_ft_sensor(raw_wrenches, poses):
    """
    F/T sensor 多姿態校準：辨識工具質量與質心
    raw_wrenches: [N, 6] 多個姿態下的原始讀值
    poses: [N, 3, 3] 對應的旋轉矩陣 (sensor frame 相對 world)
    回傳：tool_mass (scalar), tool_com (3,)
    """
    g_world = np.array([0, 0, -9.81])

    # 線性最小二乘：F_sensor = m * R^T @ g_world + noise
    # 對力的部分做 LS 拟合 m
    A = np.vstack([p.T @ g_world for p in poses])  # [N*3, 1]（展平）
    b = raw_wrenches[:, :3].flatten()
    tool_mass = np.linalg.lstsq(A.reshape(-1, 1), b, rcond=None)[0][0]

    # 對力矩的部分辨識 com
    # T_sensor = com × (m * R^T @ g_world)
    # 對每筆姿態建立 skew-sym 矩陣
    A_com, b_com = [], []
    for F_raw, T_raw, R_p in zip(raw_wrenches[:,:3], raw_wrenches[:,3:], poses):
        F_grav = tool_mass * R_p.T @ g_world
        # T = skew(com) @ F_grav → T = -skew(F_grav) @ com
        A_com.append(-np.array([[0, -F_grav[2], F_grav[1]],
                                [F_grav[2], 0, -F_grav[0]],
                                [-F_grav[1], F_grav[0], 0]]))
        b_com.append(T_raw)
    A_com = np.vstack(A_com)
    b_com = np.concatenate(b_com)
    tool_com = np.linalg.lstsq(A_com, b_com, rcond=None)[0]

    return tool_mass, tool_com


def gravity_compensate_ft(raw_wrench, tool_mass, tool_com, R_sensor):
    """
    F/T sensor 重力補償：扣除工具自重在 sensor frame 的投影
    """
    g_world = np.array([0, 0, -9.81])
    g_sensor = R_sensor.T @ g_world
    F_gravity = tool_mass * g_sensor
    T_gravity = np.cross(tool_com, F_gravity)

    compensated = raw_wrench.copy()
    compensated[:3] -= F_gravity
    compensated[3:] -= T_gravity
    return compensated


# === 使用範例 ===
# 手引示教模式：Kd=0（無回復力），小阻尼
controller_teach = AdmittanceController(
    Md=[1.0]*6,       # 小慣量 → 反應快
    Dd=[10.0]*6,      # 小阻尼 → 輕推就動
    Kd=[0.0]*6,       # 零剛度 → 手引模式
    dt=0.001
)

# 研磨模式：法向有剛度（維持壓力），切向零剛度
controller_grind = AdmittanceController(
    Md=[1.0]*6,
    Dd=[50.0, 50.0, 80.0, 10.0, 10.0, 10.0],
    Kd=[0.0, 0.0, 500.0, 0.0, 0.0, 0.0],  # 只有 z 向有剛度
    dt=0.001
)
```

</details>

<details>
<summary>深入：阻抗 vs 導納的硬體選型決策樹（完整版）</summary>

```
硬體特性判斷入口
│
├─ 馬達類型是直驅 / 低減速比（< 10:1）？
│   ├─ 是 → 有良好的反向驅動性（back-drivability）
│   │        → 首選 **Impedance Control**
│   │        → 理由：能直接輸出精確力矩；不需 F/T sensor 也能做基本柔順
│   │        → 代表：Franka Panda、MIT Cheetah 腿、Tesla Optimus
│   │
│   └─ 否 → 高減速比（> 50:1）諧波減速器 / 行星齒輪
│            → 反向驅動性差，力矩輸出受摩擦/背隙嚴重影響
│            → 首選 **Admittance Control**
│            → 理由：利用高精度位置控制，由 F/T 量測外力算位移修正
│            → 代表：UR 系列、KUKA iiwa（iiwa 有關節力矩感測器，兩種都能用）
│
├─ 有末端 F/T sensor？
│   ├─ 是 → 導納控制可直接用
│   │        阻抗控制也能用（做力跟蹤外迴圈）
│   │
│   └─ 否 → 有關節力矩感測器？
│            ├─ 是 → 可用 Generalized Momentum Observer 估測外部力
│            └─ 否 → 只能用電流推算；QDD 驅動器（低減速比）可信度高
│
├─ 任務是否需要力位混合（同時控制某些方向力 + 另些方向位置）？
│   ├─ 是 → Hybrid Position/Force（Selection Matrix 分解方向）
│   │        底層可搭配 impedance 或 admittance
│   └─ 否 → 純柔順任務 → 單一模式即可
│
└─ 環境剛性如何？
    ├─ 極剛性金屬對金屬 → 避免 Admittance（Limit Cycle 陷阱）
    ├─ 柔軟表面（海綿、布料）→ Impedance 或 Admittance 都可
    └─ 未知環境 → Variable Impedance + RL 線上適應
```

**經驗法則**：
- 協作機器人（Franka, KUKA iiwa）：阻抗控制為主，關節力矩感測器提供全身柔順
- 傳統工業臂（UR, ABB, Fanuc）：導納控制為主，搭配末端 F/T sensor
- 腿式機器人（ANYmal, Spot）：SEA + 阻抗控制（快速力矩響應做地面反力控制）
- 人形機器人（Optimus, Figure）：QDD + 阻抗控制（低減速比換極致透明度）
- 靈巧手（Shadow Hand, Allegro）：阻抗控制 + 觸覺感測器陣列

</details>

<details>
<summary>深入：SEA / VSA / QDD / Harmonic Drive — 執行器哲學與選型三角</summary>

### 四大執行器哲學對比

**SEA（Series Elastic Actuator）— 「用感測器解決致動器不完美」**
- 馬達與連桿間**串聯物理彈簧**
- 減速器有摩擦/背隙 → 彈簧 = 天然低通濾波器 → 瞬間吸收落地衝擊保護諧波減速器
- 高精度編碼器測彈簧變形 $\Delta x$ → 虎克定律 $F = K \Delta x$ → **極純粹無摩擦干擾的真實接觸力**
- 代表平台：ANYmal 四足、Baxter、Valkyrie

**VSA（Variable Stiffness Actuator）— 「生物肌肉機械復刻」**
- 兩拮抗馬達 + 非線性彈簧/凸輪 → 硬體層剛度物理可變
- 人搬重物肌肉繃緊（高 K）、穿針引線肌肉放鬆（低 K）
- 能瞬間變軟、儲存並釋放爆炸能量（投擲、跳躍）
- 代表平台：DLR Hand-Arm System、IIT Walk-Man、MIT Nadia

**QDD（Quasi-Direct Drive）— 「放棄彈簧，直接消摩擦」**
- 大扭矩外轉子馬達（盤式電機）+ 極低減速比 6:1 ~ 9:1 行星齒輪
- 極佳反向驅動性（Back-drivability）— 外力輕易反推回馬達轉子
- 電流環 $I_q$ → 電磁扭矩 $\tau_m = K_t \cdot I \cdot N$ → 幾乎等於外界真實接觸力矩
- **無 F/T sensor 實現透明力控**
- 代表平台：MIT Cheetah、Tesla Optimus、Unitree G1

**Harmonic Drive — 「極致精度 + 末端 F/T 感測」**
- 減速比 50:1 ~ 160:1，零背隙、高剛性
- 反向驅動性差 → 必須配末端 F/T sensor 做 Admittance
- 代表平台：UR 系列、Fanuc、精密手術機器人

### 選型三角 — 頻寬 × 衝擊 × 精度

| 場景 | 選型 | 理由 |
|------|------|------|
| 0.05mm 骨科手術 | Harmonic Drive + 末端 6 維 F/T | 極致精度 |
| 四足走廢墟（高頻地面衝擊） | SEA | 實體彈簧保護減速器 |
| 人形機器人手臂（接球 / 握手） | QDD | 低減速比換極致透明度 + 電流級力控 |
| 研究平台需變剛度 | VSA | 投擲、跳躍、安全 |
| 工業協作機械臂 | Harmonic Drive + 關節力矩感測器 | Franka / iiwa 路線 |

### QDD 無 F/T 估外力 Python 片段

```python
def estimate_external_torque_qdd(motor_current, q, q_dot, q_ddot, K_t, N):
    """
    QDD 驅動器無 F/T sensor 外力估測
    原理：τ_m - τ_dynamics ≈ τ_ext（QDD 摩擦極小，可忽略或簡單補償）
    """
    tau_m = K_t * motor_current * N  # 電磁扭矩
    tau_dyn = compute_RNEA(q, q_dot, q_ddot)  # 動力學前饋
    tau_fric = coulomb_friction_model(q_dot)  # QDD 摩擦極小
    tau_ext = tau_m - tau_dyn - tau_fric
    return tau_ext
```

**面試 talking point**：能在 30 秒內講出「你的人形機器人為什麼選 QDD 不選 Harmonic Drive」= 深度理解力控硬體底層的標誌。

</details>

## 常見誤解

1. **「精密裝配光靠位置控制就夠」** — 零件有公差（通常 ±0.05 mm），孔位和銷的相對位置永遠有微小偏差。純位控會硬撞 → 卡死或損壞零件。**正確理解**：裝配需要 compliance — 至少在接觸後切入力控或阻抗控制，讓機器人「順著推」找到孔位。業界標準做法是位控接近 + 螺旋探索 + 阻抗控制插入 + 自適應剛度防卡阻（Residual RL 處理未建模摩擦，見情境題 L）。

2. **「阻抗控制和導納控制一樣，只是名字不同」** — 因果方向完全相反。Impedance：位移偏差 → 力（像彈簧，你推它、它反推你）。Admittance：外力 → 位移修正（像推車，你推它、它移動）。選錯會導致系統不穩定 — 阻抗控制用在高減速比臂上，摩擦會吃掉力矩控制精度；導納控制用在直驅臂上，浪費了直接力矩控制的優勢；Admittance 撞剛性牆會產生 Limit Cycle Oscillation。

3. **「F/T sensor 讀出來就是接觸力」** — 原始讀數包含工具重力（隨姿態變化）和加速度引起的慣性力。不做重力補償就用 → 機器人靜止時就「感覺」到假的接觸力 → 控制器亂動。**正確做法**：每次開機先做 F/T sensor 校準（多姿態採樣 + 最小二乘辨識工具質量和質心），運行時即時扣除 $F_{\text{gravity}} = m_{\text{tool}}\cdot R^T g$ 和加速度慣性項 $m\ddot{x}$。

4. **「把阻抗控制的剛度 $K_d$ 調很高就能像位控一樣精準」** — 理論上 $K_d \to \infty$ 確實趨近位控。但實際上 $K_d$ 太高 → 系統變成欠阻尼 → 接觸時高頻震盪 → 嚴重時損壞零件或傷人。更糟的是，**高 $K_d$ + 奇異點 = 關節力矩瞬間爆炸**（$J^T$ 條件數放大誤差）。**正確做法**：$K_d$ 和 $D_d$ 一起調，保持臨界阻尼比 $\zeta = D_d / (2\sqrt{K_d M_d}) \approx 0.7 \sim 1.0$；同時設計奇異點保護切換到 Joint Impedance。

5. **「Cartesian Impedance 的 K 填對角線就好」** — SE(3) 平移與旋轉是耦合的，對角 K 非 SPD，在遠離原點（抓 30cm 長棍）時會產生非保守力破壞 Passivity → 狂抖。**正確做法**：K 必須是 SPD 矩陣，旋轉偏差用單位四元數誤差，跨座標系轉換用 Adjoint Transform（見深入塊）。Franka libfranka API 強制要求這一點。

## 練習題

<details>
<summary>Q1（中）：Franka Panda 7 軸在廠房執行精密組裝，操作員可能隨時靠近觸碰任意位置。請設計全身阻抗 + 碰撞偵測 + 分級反應的完整架構。</summary>

**完整推理鏈**：

1. **碰撞偵測層 — Momentum Observer**：
   - 每個關節有扭矩感測器，計算殘差 $\tau_{\text{external}} = \tau_{\text{measured}} - \tau_{\text{model}}$（$\tau_{\text{model}}$ 包含動力學 + 摩擦前饋）
   - 設定閾值 15 N·m（符合 ISO/TS 15066 PFL 標準）
   - 殘差超閾值 → 判定發生碰撞

2. **分級反應狀態機**：
   - **輕度（< 30 N）**：切換到零力拖動模式（$K_d \to 0$），允許操作員推開
   - **中度（30-60 N）**：柔順回彈模式（$K_d$ 降低 50% + $D_d$ 加倍）吸收能量
   - **重度（> 60 N）**：Category 0 急停（直接斷電）

3. **零空間分層阻抗**：
   - 主任務（末端 6D）：高 $K_{\text{task}}$ 維持精密組裝精度（不能因手肘被推而影響末端）
   - 零空間（1 冗餘 DoF）：$K_{\text{null}} \approx 0$，手肘可被操作員輕柔推開
   - 投影矩陣：$\tau_{\text{cmd}} = J^T F_{\text{task}} + (I - J^T J^{+T})\tau_{\text{null}}$

4. **奇異點保護**：
   - 持續監控 $J$ 的最小奇異值
   - 若 < 閾值 → 切換到 Joint Impedance 當彈簧兜底，防止 $J^T$ 條件數爆炸

**面試官想聽到**：Momentum Observer 碰撞偵測（無需 F/T）+ 分級反應符合 ISO/TS 15066 + 零空間分層實現「手端精度、手肘柔順」雙層目標 + 奇異點保護。

</details>

<details>
<summary>Q2（難）：Peg-in-Hole 公差 ±0.05mm，純位控必定卡阻。請設計完整的階段式柔順策略，包括卡阻自救。</summary>

**完整推理鏈**：

1. **階段 1 — 位置控接近（Approach）**：
   - 純位控快速移動到孔位上方 5mm
   - 降低速度到 10 mm/s
   - 預降 $K_d$ 從 2000 到 500，為接觸做準備

2. **階段 2 — 接觸偵測（Contact Detection）**：
   - 下壓同時監控 $F_z$
   - $F_z > 5\text{N}$ → 停止下壓，進入下階段
   - 用 ramp 函數（50-200 ms）平滑切換阻抗參數

3. **階段 3 — 阿基米德螺旋探索（Spiral Search）**：
   - XY 走螺旋軌跡 $x = r(t)\cos\theta, y = r(t)\sin\theta$，$r$ 線性增長
   - Z 維持恆定下壓力 5N
   - 監控 $F_z$：若斷崖式下降 → 掉入孔中，進入下階段

4. **階段 4 — Hybrid 力位混合插入**：
   - Z 軸：力控（維持 8N 插入力）
   - XY 軸：位控（維持中心位置）
   - Yaw 軸：低 K 阻抗（允許柔順配合公差）

5. **階段 5 — 卡阻自救（自適應阻抗）**：
   - 監控側向力 $\|F_{xy}\|$
   - 異常增大（如 > 3N）→ 判定卡阻
   - 自動降 $K$ 提 $D$（如 $K$ 降 50%、$D$ 加倍）
   - 輕微搖晃（夾角 $\pm$ 2°）讓零件重新配合 → 繼續插入

**進階 — Residual RL 處理未建模摩擦**：
- 基礎控制器處理 90% 已知動力學
- RL 網路只輸出微小殘差 $\Delta\tau$ 擬合 10% 未建模摩擦
- 樣本效率 ×100 + 底層兜底安全

**面試官想聽到**：階段式狀態機 + 螺旋探索數學 + Hybrid 架構在插入階段的設計 + 自適應阻抗防卡阻 + 對現代 Residual RL 的認識。

</details>

<details>
<summary>Q3（難）：UR5 做玻璃桌面擦拭，Z 軸純力控 10N，但當抹布滑出桌面邊緣時機械臂高速砸地面。怎麼解釋這個災難、怎麼修？</summary>

**完整推理鏈**：

1. **災難機制**：
   - $S_{zz} = 1$（Z 軸純力控），期望 $F_z = 10\text{N}$
   - 抹布滑出邊緣 → $F_{\text{ext}} = 0$（沒東西可推）
   - 力控律：$F_{\text{cmd}} = K_p(F_d - F_{\text{ext}}) = 10N$（恆定）
   - 機械臂為了達到 10N → 不斷加速 Z 軸下降
   - **Z 軸沒有位置回饋**（純力控 $S_{zz}=1$，$(I-S)_{zz}=0$）→ 沒有任何東西阻止它往下衝
   - 最終高速砸地面

2. **根因**：
   - 純力控維度**缺乏位置/速度限制**
   - Hybrid Control 的致命弱點：S=1 的維度完全沒有位置安全邊界

3. **修法 — 速度飽和（Velocity Saturation）**：
   ```cpp
   Eigen::Vector6d wrench_cmd = Kp_f * (F_d - F_measured);
   wrench_cmd(2) = std::clamp(wrench_cmd(2), -max_vel_z, max_vel_z);
   ```

4. **修法 — Damping Injection**：
   - 在純力控維度注入虛擬阻尼項 $-D_v \dot{x}$
   - 即使沒外力，速度越快虛擬阻力越大 → 自然減速

5. **修法 — 過渡到軟 Hybrid**：
   - 放棄 $S_{zz}=1$ 的 Crisp Switch
   - 改用 Impedance 控制 $K_{zz}$ 小（軟 Z 軸）+ 力跟蹤前饋
   - Impedance 天然有位置反饋，不會因為失接觸而狂衝

6. **進階 — 接觸偵測退出**：
   - 監控 $F_z$：連續 N 毫秒為零 → 判定失接觸
   - 立即切換回位控，把 Z 抬到安全高度

**面試官想聽到**：Hybrid Crisp Switch 的致命弱點（力控維度無位置邊界）+ 速度飽和 / Damping Injection 兩種工程修法 + Impedance 為何更安全（天然有位置反饋）+ 接觸偵測狀態機。

</details>

<details>
<summary>Q4（難）：設計一個 Variable Impedance Learning 系統，讓機械臂在擦拭未知剛度的曲面（金屬 / 海綿 / 紙板）時自動調整 K, D。</summary>

**完整推理鏈**：

1. **為什麼固定 K/D 不夠**：
   - 剛性金屬：K 小避免回彈振盪
   - 海綿柔性：K 大確保追蹤精度
   - 未知環境無法提前給定

2. **分層架構**：
   - **大腦慢思考層（RL, 10 Hz）**：SAC 網路輸出 $\Delta K, \Delta D$
   - **小腦快反射層（阻抗, 1 kHz）**：用 $K_t = K_{\text{base}} + \text{tanh}(\text{NN})\cdot a_{\text{scale}}$ 調變參數
   - 保證：底層 1 kHz 阻抗控制器保持 Lyapunov 穩定性

3. **RL 設計**：
   - **State**：$s = \{e_x, \dot{x}, F_{\text{measured}}, \tau_{\text{ext}}\}$
   - **Action**：$a = \{\Delta K, \Delta D\}$（只輸出參數變化量，不輸出底層力矩）
   - **SAC vs PPO**：SAC 勝出，因最大熵機制鼓勵多樣接觸策略探索 + 對非線性摩擦的魯棒性優於 PPO

4. **「懶惰 Policy」陷阱（必考）**：
   - 只獎勵誤差 → RL 學到「K 開最大最僵硬 → 誤差最小」→ 失去柔順意義
   - **正確 reward**：$R = -\|e_x\|^2 - w\cdot\|\tau_{\text{cmd}}\|^2$（加能量懲罰）
   - 強迫「需要時發力、平時放鬆」

5. **Sim-to-Real 穿模陷阱**：
   - MuJoCo soft contact 允許微小穿透
   - RL 可能學到「K → ∞ 擠進物體拿高分」→ 作弊穿模
   - 解法：(1) 接觸剛度 Domain Randomization (2) Action 層 Sigmoid 硬飽和 $K \leq K_{\max}$

6. **為什麼分層而不是端到端 RL**：
   - 端到端 RL 輸出力矩 → 推理延遲 > 10 ms → 力控崩潰
   - RL 10 Hz 輸出 K/D，底層 C++ 1 kHz 阻抗 → 融合「深度學習非線性適應」+「傳統控制 100% 高頻穩定保證」

**面試官想聽到**：為什麼用 RL（環境未知）+ 分層設計（RL 慢 + 阻抗快）+ SAC 最大熵機制 + 懶惰 Policy 陷阱 + Sim-to-Real 穿模陷阱 + 對應解法。這是能區分「追過 2023 paper」的考點。

</details>

<details>
<summary>Q5（難）：你要把 ACT（Action Chunking Transformer）/ Diffusion Policy 這類現代 imitation learning 方法整合到接觸密集任務。請設計與底層阻抗控制的整合架構。</summary>

**完整推理鏈**：

1. **為什麼 imitation learning 單打獨鬥不夠**：
   - VLM / Transformer 推理慢（1-10 Hz），直接輸出力矩 → 力控崩潰
   - Contact-rich 任務要求 > 100 Hz 的力反饋才能柔順

2. **ACT Action Chunking 優勢**：
   - 傳統 BC 單步預測 $a_t = \pi(s_t)$ → 微小誤差幾何級數放大（Compounding Error）
   - ACT 一次預測未來 k 步絕對位姿序列 $a_{t:t+k}$
   - 1000Hz 底層追蹤 + 絕對位姿 → 過濾高頻遙操作抖動
   - 成功任務：穿針引線、打雞蛋（精細雙手協作）

3. **Diffusion Policy 優勢**：
   - Contact-rich 任務常有多模態動作分佈（「從左繞」vs「從右繞」）
   - MSE-based Transformer 會**平均兩解** → 機械臂直撞障礙物
   - Diffusion：從高斯雜訊逐步去噪 → 精準擬合多峰分佈
   - 目標函數：$L = E[\|\epsilon - \epsilon_\theta(x_k, k, O)\|^2]$

4. **分層整合架構（Visuomotor × Impedance）**：
   - **大腦（VLM 如 π0 / OpenVLA）1-10 Hz**：輸出任務等級指令（去目標位姿）
   - **中腦（ACT / Diffusion Policy）10-100 Hz**：輸出 waypoints 低階目標位姿 $x_{\text{target}}$
   - **小腦（Cartesian Impedance）1 kHz**：$\tau = J^T[K(x_{\text{target}} - x) - D\dot{x}]$
   - 接觸力柔順處理全交給底層阻抗控制

5. **為什麼 Diffusion Policy 贏 MSE**：
   - 擦玻璃「從左擦 vs 從右擦」兩解都對 → MSE 平均 = 機械臂卡在中間撞工件
   - Diffusion 從雜訊去噪到單一模態，隨機選一個執行 → 天然平滑 + 多模態

6. **SOTA 平台**：
   - ALOHA / ACT：雙手精細操作
   - Diffusion Policy（Cheng Chi et al.）：通用 manipulation
   - π0 / OpenVLA：VLM 骨幹，Embodied AI 終局架構

**面試官想聽到**：ACT Action Chunking 解決 Compounding Error + Diffusion Policy 解決多模態 + 大腦慢小腦快的分層架構 + 為什麼 VLM 不直接輸出力矩。這是 2022-2024 Contact-rich 突破的兩大支柱，也是 contact-rich 未來 3 年決勝戰場。

</details>

<details>
<summary>Q6（難）：你要實作 Kinesthetic Teaching（手引示教）Transparent Mode，同時要能區分「主動引導」vs「意外碰撞」並觸發急停。</summary>

**完整推理鏈**：

1. **Transparent Mode 數學**：
   - 阻抗公式 $M\ddot{x} + D\dot{x} + Kx = F_{\text{ext}}$
   - $K \to 0, D \to$ 極小 → Transparent（幾乎感覺不到機器人重量）
   - 人類只施加幾 N 力克服殘餘慣量 → 機械臂如絲滑隨手移動

2. **避免重力塌下 — 精準動力學前饋**：
   - $\tau_{\text{ff}} = G(q) + \tau_{\text{fric}}(q, \dot{q})$ 實時抵消
   - 鬆手即停 — 重力被補償，不會往下掉

3. **Franka libfranka 零力模式（範例程式碼）**：
   ```cpp
   auto zero_force = [&](const RobotState& s, Duration dt) -> Torques {
       // Franka 內部已補重力/科氏力
       return {0,0,0,0,0,0,0};  // 輸出 0 力矩 = 人拉哪走哪
   };
   robot.control(zero_force);  // 1kHz torque control 迴圈
   ```

4. **區分主動引導 vs 意外碰撞 — 頻譜分析**：
   - **意外碰撞**：剛性接觸、$dF/dt$ 極大 → **高頻衝擊脈衝**
   - **主動拖拽**：人類肌肉發力 → **低頻 < 2 Hz**
   - FFT 或高通濾波器瞬間剝離碰撞信號 → 觸發急停

5. **觸覺位置分佈輔助判別**：
   - **引導**：多指大面積穩態包覆（Grasp/Push）
   - **碰撞**：單點尖銳接觸
   - 若有觸覺陣列（如 ANYskin）可進一步分析接觸 pattern

6. **Momentum Observer 碰撞檢測（無需 F/T sensor）**：
   $$
   r(t) = K_o\cdot\left[M(q)\dot{q} - \int(\tau_{\text{cmd}} + C^T\dot{q} - G(q) + r(s))ds\right]
   $$
   - 廣義動量定理估外部接觸力矩
   - 超閾值且變化率大 → ISO 15066 PFL 煞車

7. **Shared Control / Virtual Fixtures（進階）**：
   - da Vinci 外科機器人：安全區 $K=0$（醫生完全主導）
   - 刀尖逼近血管邊界 → $K$ 動態急劇拉高 → 大排斥力 haptic feedback 阻止危險切入

**面試官想聽到**：Transparent Mode 的 K→0 數學 + 動力學前饋補償重力（鬆手不塌）+ 頻譜分析分辨引導/碰撞 + Momentum Observer 無 F/T 也能測碰撞 + 對 Virtual Fixtures 等 HRI 進階主題的認識。

</details>

## 面試角度

1. **Selection Matrix 正交性定理（Mason 1981 Task Frame Formalism）** — 這是力控最核心、最常被問的考點。**帶出**：「Task Frame Formalism 的核心是正交性定理：在接觸點建立局部座標系，任一維度上**自然約束（環境幾何決定，如擦桌面時 $v_z = 0$）**與**人工約束（工程師目標，如 $f_z = 10\text{N}$）必定正交互補**。這就是 Hybrid Control 的 Selection Matrix 嚴格物理配置指南 — 哪一軸位控、哪一軸力控，不是憑經驗猜，而是看環境約束。環境決定位置（自然位置約束）→ 你只能在該維度施加力（人工力約束），反之亦然。擦弧面時 Task Frame 還要實時依接觸法向量旋轉更新。」**為什麼是重點**：能把 S 矩陣從「對角 0/1」講到「正交性定理」= 你讀過 Mason 1981 原始論文，不是只看過教科書。

2. **Impedance vs Admittance 選型 = 硬體邏輯** — 這題是產業落地能力的試金石。**帶出**：「選型其實就一句話：『你的機械臂的驅動器能不能直接精確控力矩？』能 → Impedance（Franka、iiwa、QDD），不能 → Admittance（UR、ABB + 外掛 F/T）。Impedance 需要低摩擦驅動（torque-controlled joints），高摩擦諧波減速器會吃掉微小柔順力矩；Admittance 需要高剛性 + 高品質 F/T sensor，不需馬達控力。**Admittance 撞剛性牆會 Limit Cycle 振盪**因為因果鏈反向放大；現代做法是 Hybrid — 自由空間 Admittance 穩定安全、接觸後切 Impedance 高頻響應。」**為什麼是重點**：這題能篩掉「死背公式」和「真懂硬體」的分水嶺，10 秒內從硬體反推架構是面試官想聽到的真實工程直覺。

3. **Cartesian vs Joint Impedance + 奇異點力矩爆炸** — 這是區分「會用」和「真懂」的硬核考點。**帶出**：「Cartesian Impedance 直覺（Z 軸軟、XY 硬），但在奇異點 $J$ 降秩時條件數爆炸 → $\tau = J^T F$ 會放大 F 到天文數字 → 硬體過載急停。Franka 在奇異點附近自動切 Joint Impedance 當彈簧兜底。另一個陷阱是 K 必須是 SPD 矩陣 + 旋轉用四元數誤差，對角 K 抓 30cm 長棍戳牆會因為力臂耦合產生非保守力 → 破壞 Passivity → 狂抖。Franka libfranka API 強制要求 K 為 SPD 就是這個原因。」**為什麼是重點**：面試官問「你怎麼在奇異點附近做力控」時，能講出「切 Joint Impedance 兜底」+「對稱正定矩陣 + 四元數誤差」= 論文讀到位的證明。

4. **Adjoint Transform 力矩耦合陷阱 + 操作空間 Λ** — 這是 Operational Space 的精髓，分辨「讀過 Khatib 1987」的人。**帶出**：「Wrench 跨座標系轉換時，力 $f$ 平移會透過 $\hat{p}R f$ 耦合出力矩 $m$。工程師忽略這項 → 指令完全錯誤。Franka / KUKA iiwa FRI 強制 K 相對當前 TCP 定義。操作空間慣量矩陣 $\Lambda = (JM^{-1}J^T)^{-1}$ 的物理意義是『末端在當前姿態下 3D 空間的有效質量』，用手推不同方向感覺多重由 $\Lambda$ 決定。高級阻抗控制做 Λ-weighted dynamics decoupling 才能讓 $K_d=500$ 在任何姿態下表現一致。」**為什麼是重點**：能講出 Screw Theory 對偶性（Twist & Wrench 內積 = 功率）= 你懂 SE(3) 幾何基礎，不是只會拼湊公式。

5. **SEA / VSA / QDD / Harmonic Drive 選型三角** — 人形機器人 / 四足時代必考。**帶出**：「選型三角是頻寬 × 衝擊 × 精度。骨科手術 0.05mm 級 → Harmonic Drive + 末端 F/T；四足走廢墟高頻地面衝擊 → SEA（實體彈簧吸收衝擊保護減速器）；人形手臂接球握手 → QDD（低減速比 6:1~9:1 + 大扭矩外轉子馬達，電流級力控無需 F/T）；研究平台需變剛度 → VSA（拮抗馬達 + 非線性凸輪，生物肌肉機械復刻）。MIT Cheetah、Tesla Optimus 選 QDD 不選 Harmonic Drive 就是為了『無 F/T 的透明力控 + 跑跳能力』。」**為什麼是重點**：能 30 秒講出 Tesla Optimus 為什麼選 QDD = 你真的讀過執行器架構論文、理解力控從硬體層到控制層的完整鏈條。

6. **Variable Impedance Learning — 大腦慢思考、小腦快反射** — 這是 2023-2024 RL × 控制的前沿。**帶出**：「固定 M/D/K 遇未知剛度環境會震盪。RL 設計：State = $\{e_x, \dot{x}, F, \tau_{\text{ext}}\}$、Action = $\{\Delta K, \Delta D\}$（不輸出力矩，只輸出參數變化量）。SAC 勝 PPO 因為最大熵機制鼓勵多樣接觸探索。**必考陷阱 1：懶惰 Policy**：只獎勵誤差 → RL 學會 K 開最大僵硬 → 失去柔順，正確 reward 要加能量懲罰 $R = -\|e_x\|^2 - w\|\tau\|^2$。**必考陷阱 2：Sim-to-Real 穿模**：MuJoCo soft contact 允許穿透，RL 可能學會 K→∞ 擠進物體作弊。分層設計：RL 10 Hz 輸出 K/D，底層 C++ 1 kHz 阻抗控制 — 融合深度學習非線性適應 + 傳統控制 100% 高頻穩定保證。」**為什麼是重點**：這題是「追過前沿 paper」vs「只看教科書」的分水嶺，能講出懶惰 Policy + 穿模兩個陷阱 = 真正調過 RL 做過 Sim-to-Real。

7. **觸覺比視覺更重要（最後 5 公分）** — 這是 Dexterous Manipulation 的核心洞察。**帶出**：「F/T sensor 只測整隻手的淨力，無法分辨指尖微滑移（抓滑溜杯子可能淨力為零但正在掉）。GelSight / DIGIT（Vision-based）用相機拍彈性體背面 + Photometric Stereo 得超高分辨率 3D 深度圖；Xela uSkin（磁場）響應 > 100Hz 適合閉環即時控制。Tactile-based Impedance Modulation：檢測到剪切力邊緣微滑移 → 瞬間在法向增 K（捏緊）、切向增 D（耗散滑動能量）。Dexterous manipulation 階段視覺遭無解遮擋災難（Occlusion）— 手包覆物體時看不見；且視覺無法測摩擦係數、質量分佈、接觸剛度。**『在最後的 5 公分，觸覺比視覺更重要』**是未來具身智能能否走進家庭的分水嶺。」**為什麼是重點**：這是 Embodied AI 產業接軌的戰略級觀點，能講出 ANYskin / DextAH 等 2024 SOTA = 你關注具身智能前沿。

<details>
<summary>深入：面試 talking point 8-14（進階考點延伸）</summary>

8. **ACT Action Chunking + Diffusion Policy 分層架構** — Contact-rich 2022-2024 突破兩大支柱。**帶出**：「ACT 解決 Compounding Error（BC 單步誤差幾何級數放大），一次預測 k 步絕對位姿序列 + 1000Hz 底層追蹤。Diffusion Policy 解決多模態動作分佈（『從左繞』vs『從右繞』）— MSE Transformer 會平均兩解撞障礙物，Diffusion 從雜訊去噪到單一模態天然平滑。整合架構：VLM 1-10 Hz 出 waypoints → ACT/Diffusion 10-100 Hz 出低階位姿 → Cartesian Impedance 1 kHz $\tau = J^T[K(x_t-x)-D\dot{x}]$。大模型指方向，接觸力柔順處理全交給底層阻抗。」**為什麼是重點**：能講出 Diffusion Policy 為何贏 MSE = 你讀過 Cheng Chi 的 paper，不是紙上談兵。

9. **Residual RL Assembly（樣本效率 ×100）** — 產業落地的答案。**帶出**：「純 RL 做 Peg-in-Hole 要百萬步、還會學穿模；Residual RL 用經典控制器 $\tau_{\text{base}} = M\ddot{q}_d + C\dot{q} + G + K_p e + K_d \dot{e}$ 處理 90% 已知動力學，RL 只輸出小殘差 $\pi_\theta(s, F_{\text{ext}})$ 擬合未建模摩擦 → 探索空間極小 + 安全兜底 + 樣本效率 ×100。**大孔徑 AI 成功、微米精密仍是傳統**因為 MuJoCo soft contact 允許穿透 → 純 RL 學作弊 → 真機金屬卡死崩潰。」**為什麼是重點**：這是「做過真機部署」vs「只跑 sim」的分水嶺。

10. **Momentum Observer 無 F/T 做碰撞偵測** — HRI 核心技能。**帶出**：「$r(t) = K_o[M\dot{q} - \int(\tau_{\text{cmd}} + C^T\dot{q} - G + r)ds]$，廣義動量定理估外部接觸力矩，無需 F/T sensor。ISO/TS 15066 PFL 認證核心技術。Franka / KUKA iiwa 全身柔順的感測基礎。」**為什麼是重點**：人機協作認證必考。

11. **Transparent Mode 重力補償 + 頻譜分辨引導/碰撞** — Kinesthetic Teaching 工程精髓。**帶出**：「K→0、動力學前饋 $G(q) + \tau_{\text{fric}}$ 精準抵消 → 鬆手即停絲滑拖動。意外碰撞 = 剛性接觸高頻脈衝 dF/dt 極大；主動引導 = 人類肌肉發力 < 2 Hz 低頻。FFT 或高通濾波即分辨，觸發急停或跟隨。」**為什麼是重點**：協作機器人廠商面試必考。

12. **HRI + BCI 交會點（阻抗控制 = 脊髓反射）** — 未來具身智能戰略級觀點。**帶出**：「腦機介面信號頻寬低延遲高，直接控位置極度危險。未來 Shared Autonomy：BCI 輸出高層方向意圖，阻抗控制底層接管所有物理接觸、柔順避障、力學約束。**阻抗控制補足 BCI 缺失的『脊髓反射』** — 這是殘障人士安全使用具身智能的唯一路徑。Virtual Fixtures（da Vinci 外科）在安全區 K=0 醫生主導，逼近血管 K 急劇拉高 haptic feedback 阻止危險切入 — 同樣思路可擴展到 BCI。」**為什麼是重點**：NUS 做 Embodied AI 會看你對未來 10 年架構的 vision。

13. **Raibert Crisp Switch vs Impedance Continuous Blend** — 架構本質差異。**帶出**：「Hybrid S 矩陣是 Crisp Switch 非黑即白硬開關，物理上強制切斷某維度位置反饋，在已知幾何約束下力控精度高；Impedance M-B-K 是 Continuous Blend 連續融合，K 小則『軟化』該維度，應對未知環境更穩。時間複用 S（碰前 S=0、碰後 S=1）容易 Crisp Switch 指令跳變導致 Chattering，現代控制器偏好 Impedance 軟過渡。」**為什麼是重點**：分辨「只會說 Hybrid」vs「理解架構本質」的考點。

14. **接觸過渡狀態機設計** — 工業落地最常出事的地方。**帶出**：「自由空間到接觸的過渡是力控最容易出事的時刻。三階段：(1) 接近時預降 $K_d$ 從 2000 到 200、加阻尼 $D_d$ (2) 力閾值觸發切換（如 3N），用 ramp 函數 50-200 ms 平滑過渡 $K_d, D_d$ (3) 穩態力控，力矩飽和保護防尖峰。接觸偵測不能只看力閾值，要加加速度補償避免運動慣性力誤觸發。」**為什麼是重點**：這是工業部署 99% 會遇到的問題。

</details>

## 延伸閱讀

- **Hogan, "Impedance Control: An Approach to Manipulation, Parts I-III" (1985)** — 阻抗控制的開山三部曲，定義 impedance/admittance 因果框架、Passivity 穩定性、多自由度擴展。理解力控必讀的經典聖經
- **Raibert & Craig, "Hybrid Position/Force Control of Manipulators" (1981)** — Selection Matrix 力位混合控制的原始論文，研磨/裝配場景的理論基礎
- **Mason, "Compliance and Force Control for Computer Controlled Manipulators" (1981)** — Task Frame Formalism + 自然/人工約束正交性定理的原始論文，S 矩陣物理配置的聖經
- **Khatib, "A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation" (1987)** — 操作空間公式、$\Lambda$ 慣量矩陣、末端任務與零空間分層的開山論文
- **De Luca, Albu-Schäffer, Haddadin et al., "Collision Detection and Safe Reaction with the DLR-III Lightweight Robot Arm" (2006)** — Generalized Momentum Observer 的經典實作論文，全身柔順控制的感測基礎
- **Albu-Schäffer, Ott, Hirzinger, "A unified passivity-based control framework for position, torque and impedance control of flexible joint robots" (2007)** — Franka / iiwa 阻抗控制底層架構的理論基礎
- **Buchli, Stulp, Theodorou, Schaal, "Variable Impedance Control — A Reinforcement Learning Approach" (2011)** — RL + 阻抗控制結合的代表工作，Variable Impedance 的起點
- **Cheng Chi et al., "Diffusion Policy: Visuomotor Policy Learning via Action Diffusion" (RSS 2023)** — Diffusion Policy 解多模態動作分佈，Contact-rich 2022-2024 突破的里程碑
- **Zhao et al., "Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware" (ACT / ALOHA, RSS 2023)** — ACT Action Chunking 解 Compounding Error，雙手精細操作新典範
- **Lynch & Park, "Modern Robotics: Mechanics, Planning, and Control" 第 11 章** — 力控章節的標準教科書，含 Screw Theory、Adjoint Transform、Hybrid Control 完整推導
- **Siciliano & Villani, "Robot Force Control" (1999)** — 力控獨立專書，包含阻抗、導納、混合、間接/直接力控所有經典主題
- **ros2_control + force_torque_sensor_broadcaster + cartesian_controllers** — ROS 2 下力控的標準實作套件，快速搭建阻抗/導納控制器
- **libfranka (Franka Emika)** — 實際產品級 Cartesian/Joint Impedance 控制 API，範例包含完整 Adjoint Transform 處理
- **MuJoCo 官方教程 — Contact-rich manipulation** — 模擬器中觀察力控行為的最佳起手式，含 Sim-to-Real 穿模陷阱的示範
- **Xiao, Tomizuka et al., "DextAH: Dexterous Anthropomorphic Hand" 系列論文 + ANYskin 觸覺皮膚** — 2024 Dexterous Manipulation SOTA，觸覺陣列 + RL + 阻抗分層
- **《具身智能算法工程師 面試題》Ch6 力控合集** — 面試最常問的力控考點整理，包含阻抗/導納/力位混合/Variable Impedance 完整對比
