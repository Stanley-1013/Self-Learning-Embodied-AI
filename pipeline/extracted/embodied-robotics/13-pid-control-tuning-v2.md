# Extracted v2: PID 控制 — 補強素材 (session 7df0af65)

## Q1 補強（串級 PID + RL 混合）

### 情境題 A：6-DoF 串級調參完整流程
- 「由內而外、末端至基座」原則
- Step1: 電流環 1-4kHz（驅動器預設）
- Step2: 速度環 PI — 斷位置環，Kp 逼臨界振盪→回調 0.6 倍→加 Ki 消穩態誤差
- Step3: 位置環純 P（速度環 Ki 已消靜差）
- Step4: 階躍響應測試 — Overshoot < 5%（精密任務要求 0%）、Settling time < 50-100ms
- Step5: **動力學前饋補償**（重力+科氏力疊加至電流環）
- Step6: 全軸聯動 + 共振頻率陷波濾波 + Bode phase margin > 45°

### 情境題 B：RL vs PID 混合架構
- PID 不可替代場景：單關節電流/速度跟蹤（μs 延遲、Lyapunov 穩定證明、可解釋）
- 混合架構1：**分層** — RL 10-50Hz 出參考軌跡 → PID/MPC 1kHz 跟蹤
- 混合架構2：**Gain Scheduling** — RL 依狀態動態調 Kp/Ki/Kd

### 面試 talking points
1. **增量式 PID 工業價值**：Δu vs 絕對 u；步進馬達/通訊延遲天然無 windup；斷通訊=安全停
2. **微分先行（D on Measurement）**：避免 setpoint 跳變產生 Derivative Kick；改微分 -Kd·dy/dt

## Q2 補強（Anti-Windup + 經典調參邊界 + Feedforward）

### 情境題 C：Anti-Windup 三種策略選型
- **Integral Windup 物理意義**：馬達已飽和但積分器仍累積「想像誤差」→ 目標反向時需漫長 unwind → 巨大 overshoot
- **Conditional Integration**：saturation 時凍結積分 → 適用**關節角度限位 / 遇障礙卡死**（防止馬達過熱燒毀）
- **Back-Calculation**：`u_I += Ki·e·dt + Kb·(u_sat - u)·dt` → 最平滑動態，適用**關節扭矩飽和**（無縫退飽和）
- **Clamping**：積分值硬截斷上下限 → 適用**電流限制 / 足式跌倒恢復**（極端姿態兜底）
- **Cascade PID 耦合陷阱**：內環（電流/速度）先飽和，但外環（位置）不知道仍瘋狂積分 → Coupled Saturation
- **正解 Tracking Anti-Windup**：內環 saturation → 布林回傳外環 → 外環同步凍結積分或啟動回算
- **平台**：ANYmal 接觸地面瞬間力矩飽和 → Clamping + Back-calculation

### 情境題 D：Ziegler-Nichols 為何調不動機械臂
- **Z-N / Cohen-Coon 底層假設**：FOPDT（一階慣性 + 純延遲），設計給熱力學/化工過程
- **機械臂違反 FOPDT 的三處**：
  1. 重力耦合：不同姿態等效重力矩完全不同
  2. 非線性摩擦：諧波減速器 Stribeck 效應，啟動死區
  3. 多 DOF 耦合：軸 1 運動產生離心力影響軸 2 慣性矩陣 → 破壞 SISO 假設
- **致命後果**：Z-N 要求「等幅臨界振盪」→ 直接摧毀諧波減速器柔輪
- **正解工業三步走**：
  1. SysID（傅立葉激勵軌跡辨識動力學參數）
  2. 模型前饋抵消非線性（CTC: τ = M(q)q̈_d + C(q,q̇)q̇_d + G(q)）
  3. PID 只處理線性化殘差 + Bode phase margin > 45°
- **平台**：UR 協作機械臂內建精確動力學模型做重力/慣量補償，PID 只處理殘差

### 情境題 E：Feedforward + PID 工程結構
- **PID 反應式死穴**：Error-driven → 必須先有誤差才出力 → 高速畫圓必相位滯後
- **Kd 不能解決**：預測未來靠 FF，Kd 只放大雜訊
- **Feedforward 三層**：
  - Kinematic FF：q̇_d, q̈_d 直接前饋
  - Model-based FF（CTC）：τ_ff = M(q)q̈_d + C(q,q̇)q̇_d + G(q)
  - ILC：重複任務從歷史誤差學前饋波形
- **FF 注入點必須在電流環**（1-4kHz）：
  - 注入位置環會被外環低頻寬過濾掉
  - 馬達瞬間獲得克服慣性+重力所需扭矩
- **FF 設計錯誤後果**：負載估錯 → 過度補償 → FB 反向拉扯 → FF/FB 互相打架 → 低頻共振或高頻抖振
- **架構**：τ_total = τ_ff_model + τ_fb_pid → Safety Monitor 力矩峰值審查
- **平台**：KUKA KRC4 / ABB IRC5 軌跡規劃層提前數 ms 算 τ_ff，驅動器端與編碼器 FB 結合

### 面試 talking points（補強）
3. **Back-Calculation 為什麼比 Clamping 優雅**：動態泄壓 vs 硬截斷；退飽和無縫銜接
4. **Cascade PID Windup 的層級耦合**：講得出內外環如何同步抗飽和
5. **FF/FB 分工邏輯**：「FF 處理已知動力學、FB 處理未建模殘差」；面試常問「為什麼不用純 FF？」（模型不精確 + 雜訊）

## Q4 補強（頻域設計 + Robust PID + 現代架構整合）

### 情境題 F：Loop Shaping + Bode/Nyquist 頻域設計
- **時域指標不夠用**：階躍響應看似完美可能隱藏高共振峰或低相位裕度；感測器雜訊或通訊延遲下瞬間崩潰
- **頻域三大指標**：**頻寬**（反應速度）、**相位裕度 PM**（阻尼/穩定性）、**增益裕度 GM**（抗參數漂移）
- **Loop Shaping 三步法**：
  1. 定義期望 L(jω)：低頻高增益（消穩態誤差）→ 中頻 -20dB/dec 穿越 0dB → 高頻快衰減（抑雜訊）
  2. 設計 C(s)：讓 C(s)·P(s) = L(s)
  3. 驗證 PM > 45° + GM > 6 dB
- **PID 的頻域本質**：
  - P：整體抬升開環幅頻（頻寬↑，PM↓）
  - I：低頻無限增益消靜差，但帶 -90° 致命相位滯後
  - **D：高頻 +90° 超前相位 → 補償 I 與慣性的相位滯後 → 穩定裕度的關鍵**
- **PM 與 ζ 黃金公式**：`ζ ≈ PM/100`（PM 60° → ζ≈0.6，最佳超調 < 10% 且整定極快）
- **Nyquist 判據**：開環穩定時 `Z=N=0` → Nyquist 曲線不環繞 (-1, j0)；離該點多遠 = 魯棒性多高
- **頻寬三道物理牆**：
  1. Sensor Noise → D 增益放大高頻雜訊 → 馬達尖叫發熱
  2. Actuator Saturation → 大瞬間扭矩 → 電流飽和（Windup）→ 非線性失控
  3. **共振模態** → 頻寬蓋到結構共振頻率 → 激發毀滅性結構震盪
- **Notch Filter 實務**：掃頻（Chirp signal）畫 Bode → 發現共振峰 → 串聯陷波濾波器壓平 + 頻寬退讓
- **真實失敗**：肩部 PID 空載完美；抓 2kg → 20Hz 狂抖。掃頻發現負載 2kg 時共振峰 25Hz，而 PID 頻寬 30Hz 包覆共振→ 加 25Hz Notch + 頻寬降 15Hz

### 情境題 G：Robust PID（IMC + H∞ + μ-synthesis）
- **IMC 設計哲學**：控制器 = 過程逆模型 × 低通濾波器 f(s)；模型一致時無差跟蹤，失配時 f(s) 濾高頻不確定性保魯棒
- **IMC-PID 對 FOPDT**（一階+純延遲 `G(s) = K·e^{-Ls}/(τs+1)`）：
  - Pade 近似 + IMC 直接推導出 PID 三參數：
    - `K_p = (τ + L/2) / (K(λ + L/2))`
    - `T_i = τ + L/2`
    - `T_d = τL / (2(τ + L/2))`
  - **關鍵價值**：PID 三參數降維到**只有一個調諧 λ**（閉環時間常數）
    - λ 小 = 快速；λ 大 = 魯棒
- **H∞ 混合靈敏度**：
  - 最小化 `||[W₁S; W₂KS; W₃T]||_∞ < 1`
  - S = 1/(1+PC) 靈敏度函數；`W₁·S` 低頻跟蹤性能
  - T = PC/(1+PC) 補靈敏度；`W₃·T` 高頻魯棒穩定
- **μ-synthesis**：機器人連桿質量/摩擦不確定性定義為 `m = m₀(1 + 0.2δ)`；保證所有 Δ 最壞情況下仍穩定
- **IMC-PID 是工業軍規的原因**：
  - Z-N 把系統逼臨界振盪邊緣 = 自殺
  - IMC-PID 從數學根源內建低通 + 模型逆 → 天然對未建模延遲/摩擦免疫
- **真實失敗**：ABB IRB 換非標定端拾器 → 固定 PID 過沖觸發過載。修正：H∞ 混合靈敏度，質量建模為 m∈[1,15]kg 乘性不確定性 → 犧牲 10% 空載頻寬換全負載無條件穩定

### 情境題 H：PID + Observer + Model-based FF 現代完整架構
- **光 PID 不夠用**：
  - 純 Reactive → 必須先有誤差才出力
  - D 對編碼器位置直接求導 → 高頻量化雜訊放大 → 燒馬達
  - 無法預見重力/科氏力非線性
- **Observer 整合**：
  - EKF / Luenberger 結合「動力學預測 + 感測器修正」 → 平滑無延遲 `q̇̂`
  - 餵給 PID 微分環 → D 增益可放心開大，不怕放大雜訊
- **DOB (Disturbance Observer)**：
  - `d̂ = LPF(τ_ideal - τ_measured)`
  - 前饋抵消未建模摩擦/外力，不依賴精確摩擦模型
- **CTC + PID 聖杯**：
  - `τ = M(q)·(q̈_d + K_v·ė + K_p·e) + C(q,q̇)·q̇ + G(q)`
  - **反饋線性化**：前半 `C·q̇ + G` 抵消非線性項；乘 M(q) 歸一慣量
  - 結果：N 軸耦合非線性 → **N 個獨立線性雙積分器** → 外環 PD 只處理極小線性殘差
- **具身智能分層控制哲學**：
  - **大腦 1Hz**：VLM/LLM 語義理解 + 高層任務拆解
  - **小腦 100Hz**：MPC 或 RL 策略，SRBD 上滾動優化
  - **脊髓 1-4kHz**：CTC + 增量式 PID，毫秒級硬即時電流跟蹤 + 硬體安全保護
- **面試展現「系統觀」答題**：「我設計的不是一個控制器，是分層解耦防禦體系：1kHz Luenberger 清雜訊 → CTC 剝 90% 非線性 → PID 處理 10% 線性殘差 → 100Hz MPC/RL 負責長視角避障與決策。讓 RL 做擅長的泛化，讓 PID 守物理穩定的絕對底線。」
- **真實失敗**：四足腿部直接端到端 RL 輸出力矩（20ms 推理延遲）→ 馬達電流振盪過熱燒毀。修正：「RL 50Hz 出參考腳端軌跡 + 關節 CTC+PID 2kHz 跟蹤」混合架構 → 馬達溫 45°C 連跑 2 小時

## Q5 補強（Digital 離散化 + Adaptive/Gain Scheduling + 特殊應用）

### 情境題 I：Digital PID 嵌入式實作細節
- **三種離散化方法**：
  - **Backward Euler** `s = (1-z⁻¹)/T`：最簡單，穩定極點映射正確，高頻失真嚴重
  - **Tustin / Bilinear** `s = (2/T)·(1-z⁻¹)/(1+z⁻¹)`：**工程界推薦**，左半 s 平面完整映射到 z 單位圓內，Bode 最準
  - **ZOH**：輸入階梯狀 → DAC 場景最符物理
- **採樣頻率 = 閉環頻寬 × 10-15**：工業伺服電流環頻寬 100-300 Hz → **採樣 1-4 kHz**；ADC 前必須類比 Anti-aliasing LPF（防奈奎斯特折疊）
- **Position vs Incremental Form**：
  - Position: u(k) 絕對值 → 通訊斷連 MCU 當機 = 馬達保持**最後錯誤指令**
  - **Incremental**: u(k) = u(k-1) + Δu(k) → 斷連時 Δu=0 → 馬達**平滑保位**（救命）+ 天然免於積分飽和
- **定點數實作** (ARM Cortex-M0 / 8051 無 FPU)：Q-format，增益 ×1024 左移 10 位，算完右移 → 單週期
- **D 項放大雜訊陷阱**：
  - 純離散微分 `D = K_d·(e(k)-e(k-1))/T`，T=1ms 下 quantization noise × 1000
  - **正解 Low-pass Filtered Derivative**（不完全微分）：
    - `D(k) = (T_d/(T_d+N·T_s))·D(k-1) + (K_d·N/(T_d+N·T_s))·(e(k)-e(k-1))`
    - N = 10-20 濾波係數
    - 截止頻率設為系統頻寬 1/5-1/10
- **真實失敗**：輪式機器人用 Position Form → Wi-Fi 斷連 2 秒 → 舊指令持續 → 全速撞牆。修正：改 Incremental + Watchdog，逾時 Δu=0 平滑煞車

### 情境題 J：Adaptive PID / Gain Scheduling / 工業現狀
- **Gain Scheduling（90% 工業現場方案）**：
  - 機械臂伸展時 M(q) 大 → 需高 K_p；摺疊時低 K_p；用錯會遲鈍或震盪
  - 公式：`K_p(q) = K_{p0} · det(M(q))/det(M₀)`、`K_d 保持阻尼比恆定`
  - 無人機：依電池電壓動態放大 PID（Voltage Scaling）
- **MRAC**：定義理想參考模型，實際誤差經李雅普諾夫法則推導自適應律，強迫物理表現如理想模型
- **STR (Self-Tuning Regulator)**：RLS 線上辨識 + 代數極點配置重算 PID
- **RL-based Gain Tuning**：State=風速/地形，Action=(ΔK_p, ΔK_i, ΔK_d)；底層保留傳統 PID
- **為什麼工業界 90% 不用 Neural/Fuzzy/RL PID**：
  - 工業追求**絕對穩定性與可預測性 (Deterministic)**
  - Gain Scheduling 每工作點可用 Bode 驗證 PM > 45° + GM
  - 黑箱 RL 在 OOD 狀態可能輸出極端增益損毀機台
  - **無法通過 ISO 13849 功能安全認證**
- **面試答題：「我調過 PID」vs「我設計自適應 PID 系統」**：
  - 不要只說「我用 Z-N 法調過 Kp/Ki/Kd」
  - 要說：「我設計過自適應 PID — 發現固定參數無法應對負載突變；引入基於 M(q) 的 Gain Scheduling + 變增益 `K_p = K_{P_high}·(1-sech(a·e(t)))`；確保所有姿態下 PM > 60°，這才是工業級解法」
- **真實失敗**：四足草地行走正常，冰面 PID 過剛 → 滑倒。修正：Fuzzy-PID 基於足底滑移檢測（高頻速度誤差）自動降 K_p 軟化關節

### 情境題 K：特殊應用場景 PID
- **無人機姿態 Cascade**：
  - **外環 Angle PID**（目標角度 → 目標角速度）
  - **內環 Rate PID**（目標角速度 → 馬達推力）
  - **Acro mode 只調 Rate PID**：搖桿輸入直接當「目標角速度」繞過外環 → 無限翻滾極致跟手感
- **AGV / 差動輪**：
  - 不直接控左右輪，先解耦到「線速度 v + 角速度 ω」
  - **Pure Pursuit + PID**：Pure Pursuit 給期望曲率 κ（→ω）+ PID 跟蹤 v/ω，最後逆解算給左右輪
- **液壓系統**：
  - 油液壓縮遲滯 + 閥門死區 + 高度非線性
  - **I 增益必須極小**：系統響應慢，大 K_i 在誤差消除前瘋狂累積 → 閥門打開時撞缸
  - 策略：Feedforward 預估閥開度 + 強 P 為主
- **力控 PID 剛性接觸陷阱**：
  - 剛性桌面微小位移 → 巨大接觸力突變
  - 純力 PID 的 I 項累積 → **高頻極限環振盪 (Limit Cycle Oscillation)**
  - **必須搭配 Impedance Inner Loop 或 Damping Injection**
  - 人為加虛擬速度反饋阻尼 `B·ẋ` 消耗碰撞能量
- **平台對應**：
  - **PX4**：attitude + position cascade
  - **ROS 2 Nav2 DWB controller**
  - **Franka panda Cartesian stiffness**
- **面試答題「你在機器人上調過什麼 PID」**：
  - 要展現「懂應用脈絡」：
    - 「無人機 Acro 我只調 Rate PID，因為搖桿給的是目標角速度」
    - 「AGV 我先解耦到 v/ω 再調，不直接對輪子下 PID」
    - 「液壓我 I 設極小，主靠 Feedforward 補閥開度」
    - 「力控絕不用純力 PID，必墊阻抗內環」
- **真實失敗**：Franka peg-in-hole 直接用末端力 PID → 接觸金屬瞬間 50Hz 狂抖觸發急停。修正：改導納控制柔順架構，`F = M·ẍ + B·ẋ + K·x` 加虛擬阻尼 → 「像浸在糖漿中」的阻尼特性徹底消除接觸震盪

### 面試 talking points（Q4+Q5 補強）
6. **PM ≈ 100·ζ 黃金公式 + Notch Filter 抑共振**：面試必考，能量化描述頻寬極限
7. **IMC-PID λ 單參數調諧**：比 Z-N 高一個維度，講得出「為什麼是軍規」
8. **CTC + PID 反饋線性化本質**：剝非線性讓 PID 處理線性殘差，分辨「泛泛懂 PID」vs「懂控制理論」
9. **分層架構 1Hz/100Hz/1kHz**：面試展現系統觀必備
10. **Incremental Form 救命**：通訊斷連場景，嵌入式開發經驗的試金石
11. **D on Measurement + LPF Filtered Derivative**：高頻雜訊陷阱的標準解
12. **工業為什麼用 Gain Scheduling 不用 RL**：穩定性 + ISO 13849 認證；分辨「學界論文」vs「工業經驗」
13. **力控不能純用力 PID**：必須墊阻抗內環；懂接觸動力學面試主打
