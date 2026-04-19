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
