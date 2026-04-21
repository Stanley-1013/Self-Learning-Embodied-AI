---
title: "PID 控制原理與抗飽和實務"
prerequisites: ["09-robot-dynamics-modeling"]
estimated_time: 60
difficulty: 3
tags: ["pid", "control", "anti-windup", "cascade", "feedforward", "gain-scheduling", "loop-shaping", "imc", "ctc"]
sidebar_position: 13
---

# PID 控制原理與抗飽和實務

## 你將學到

- 兩句話精確講清楚 PID 三個項各自做什麼、為什麼合在一起能讓系統穩準快，並點出它在「感知 → 規劃 → 控制」閉環的位置
- 遇到「機械臂卡死後釋放 → 瘋狂超調」「熱機後撞機」「抓重物後 20 Hz 抖動」「Wi-Fi 斷線後全速撞牆」這類現場災難，能立刻指名病因（windup、摩擦漂移、共振包覆、Position Form 失效）並選對修法
- 面試被問串級 PID、Anti-Windup 三策略、Ziegler-Nichols 為什麼調不動機械臂、Loop Shaping、IMC-PID、CTC + PID、分層控制（1 Hz / 100 Hz / 1 kHz）時，每題都能在兩分鐘內講清楚關鍵邏輯
- 掌握工業級調參真實流程：SysID → Model-based Feedforward → PID 處理線性化殘差 → Gain Scheduling 應對姿態變化 → ISO 13849 功能安全通過

## 核心概念

**精確定義**：**PID 控制器**根據當前誤差（P）、過去累積誤差（I）、誤差變化趨勢（D）三個訊號的加權和，計算出控制量送給致動器。它是閉環控制中最普遍的「最後一哩路」 — 把上游規劃出的期望軌跡，變成馬達實際能追上的力矩或電壓。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：誤差 $e(t) = x_{\text{desired}}(t) - x_{\text{actual}}(t)$（位置、速度或力矩層級的差值）
- **輸出**：控制量 $u(t)$（力矩指令 / 電壓 / PWM duty）
- **上游**：IK 求解器、trajectory planner、MPC 或 RL policy 給出 $x_{\text{desired}}$
- **下游**：致動器（馬達驅動器）、感測器回傳的 $x_{\text{actual}}$ 形成回饋迴路
- **閉環節點**：位於 **控制** 最末端，是「數學規劃 → 物理現實」的翻譯層；在具身智能的分層控制裡，PID 是**脊髓層（1 kHz 硬即時）**，上方是小腦（MPC/RL 100 Hz）、大腦（VLM/LLM 1 Hz）

**一句話版本**：「PID 是機器人的脊髓反射 — 看到偏差就出力修正，不需要大腦重新規劃；物理穩定的絕對底線由它守住。」

### 最少夠用的數學

**1. 連續時間 PID 控制律**：

$$
u(t) = K_p \, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \, \frac{de(t)}{dt}
$$

**物理意義**：$K_p e$ 看「現在偏多少」立刻出力修正；$K_i \int e$ 看「過去累積偏了多少」慢慢補償穩態誤差（如重力、摩擦）；$K_d \dot{e}$ 看「偏差正在往哪個方向變」提前減速避免超調。

**2. 離散位置式 vs 增量式**：

$$
u[k] = K_p\,e[k] + K_i \sum_{j=0}^{k} e[j]\,\Delta t + K_d \frac{e[k]-e[k-1]}{\Delta t} \quad \text{(位置式)}
$$

$$
\Delta u[k] = K_p(e[k]-e[k-1]) + K_i\,e[k]\,\Delta t + K_d\frac{e[k]-2e[k-1]+e[k-2]}{\Delta t} \quad \text{(增量式)}
$$

**物理意義**：位置式每次算出控制量的**絕對值**，直接可用但積分項容易爆；增量式只算**變化量** $\Delta u$，天然不累積歷史；**通訊斷線時位置式會讓馬達鎖在最後錯誤指令衝出去，增量式 $\Delta u = 0$ 會平滑保位 — 這是嵌入式開發的試金石**。

**3. Integral Windup 與 Anti-Windup（核心防線）**：

$$
\text{Clamping: } \quad \text{if } |u| \geq u_{\text{max}}, \quad \text{stop integrating} \; (\dot{I} = 0)
$$

**物理意義**：當致動器已經飽和（馬達出不了更大力），繼續累積 I 項只會讓內部狀態膨脹。一旦誤差反轉，龐大的 I 會讓系統瘋狂超調甚至失控。

<details>
<summary>深入：Anti-Windup 三策略完整比較 + Cascade PID 層級耦合陷阱</summary>

### 三種工程主流策略

**1. Conditional Integration（條件積分 / 凍結積分）**

最簡單：飽和時凍結積分器。

```
if |u_total| >= u_max and sign(u) == sign(e):
    integral_term unchanged  # 不再累加
else:
    integral_term += Ki * e * dt
```

- **適用**：關節角度限位、遇障礙卡死、防止馬達過熱燒毀
- **缺點**：退飽和時可能有微小延遲

**2. Back-Calculation（反算衰減）**

最精緻：用飽和前後的差值反饋回積分器，I 自動衰減。

$$
\dot{I}(t) = K_i \, e(t) + \frac{1}{T_t}\bigl(u_{\text{sat}} - u_{\text{raw}}\bigr)
$$

$T_t$ 是追蹤時間常數（通常取 $T_t = \sqrt{T_i \cdot T_d}$）。未飽和時反饋項為零，I 正常累積；飽和時反饋項把多餘 I「吸回來」。

**實作變體注意**：下方 Python 範例把積分器存成 `integral = ∫e dt`（不含 $K_i$），然後 `i_term = K_i · integral`。該版本的更新律實際為 `integral += (e + (u_sat − u_raw)/T_t)·dt`，等效的有效時間常數是 $T_t / K_i$。兩種寫法功能等價，只差 $T_t$ 的縮放，工程上按你習慣的參數化調 $T_t$ 即可。

- **適用**：關節扭矩飽和、高性能伺服驅動 — 無縫退飽和
- **缺點**：多一個參數 $T_t$

**3. Clamping（硬截斷）**

直接把積分值限幅在上下限內，最粗暴但最安全。

- **適用**：電流限制、足式跌倒恢復等極端姿態兜底
- **缺點**：離邊界較遠的動態略受影響

### Cascade PID 層級耦合陷阱（真工業殺手）

**情境**：三環串級中，**內環（電流/速度）先飽和**，但外環（位置）**渾然不知仍瘋狂積分** → 最終災難性超調，即使每個環單獨都有 anti-windup 也會爆。

**正解 — Tracking Anti-Windup**：
- 內環 saturation 狀態**回傳給外環**（布林訊號或實際 $u_{\text{sat}}$）
- 外環同步凍結積分 or 啟動 back-calculation
- Siemens / Rockwell 等工業 PLC 的 PID 函數庫內建 `saturated` 輸出端子專為此而設

### 工業實務選擇（表）

| 場景 | 推薦 | 理由 |
|------|------|------|
| 嵌入式 MCU、資源極有限 | Conditional | 零開銷、好 debug |
| 啟動瞬間誤差極大 | Conditional + 積分分離 | 避免啟動時 I 亂飆 |
| 高性能伺服驅動 | Back-calculation | 退飽和最平滑 |
| 足式跌倒恢復 / 極端姿態 | Clamping + Back-calculation | 硬邊界 + 平滑退飽和 |
| 串級控制 | Tracking Anti-Windup | 必須層級間傳遞飽和狀態 |

**平台**：ANYmal 接觸地面瞬間力矩飽和採用 Clamping + Back-calculation 組合，在極端姿態下保持穩定。

</details>

### Ziegler-Nichols 在機械臂為什麼失敗

**傳統 Z-N / Cohen-Coon 的底層假設**：FOPDT（一階慣性 + 純延遲 $G(s) = K \, e^{-Ls} / (\tau s + 1)$），設計給熱力學、化工過程。

$$
\text{Z-N (PID)}: K_p = 0.6\,K_u, \quad T_i = T_u/2, \quad T_d = T_u/8
$$

**物理意義**：找臨界增益 $K_u$ 和振盪週期 $T_u$ 算初值 — **但這招在機械臂上經常翻車**。

<details>
<summary>深入：機械臂違反 FOPDT 的三處 + 工業真正用的調參流程</summary>

### 機械臂違反 FOPDT 的三處

1. **重力耦合**：不同姿態等效重力矩完全不同，伸直姿態調好的參數換個姿態就振盪
2. **非線性摩擦**：諧波減速器 Stribeck 效應、啟動死區 — 低速反轉時摩擦方向跳變，線性模型直接崩
3. **多 DoF 耦合**：軸 1 運動產生的離心力影響軸 2 慣性矩陣 → 徹底破壞 SISO 假設

### 致命後果

Z-N 要求「等幅臨界振盪」作為實驗步驟 → **直接摧毀諧波減速器柔輪**（Harmonic Drive 的 flex spline 在共振下應力集中）→ 減速器報廢成本遠超調參收益。

### 工業真正用的三步走

1. **SysID**（用 Fourier 激勵軌跡辨識 $M(q), C(q,\dot{q}), g(q)$ 動力學參數）
2. **模型前饋抵消非線性**（Computed Torque Control）：

$$
\tau = M(q)\ddot{q}_d + C(q,\dot{q})\dot{q}_d + G(q)
$$

3. **PID 只處理線性化殘差**：配合 Bode 分析確保 Phase Margin > 45°

**平台對應**：UR 協作機械臂、Franka Panda、KUKA iiwa 全部內建精確動力學模型做重力/慣量補償，PID 只處理殘差 — 這是「跑得動」和「用得起」的分水嶺。

### 6-DoF 串級 PID 工業調參完整流程

「由內而外、末端至基座」六步：

1. **Step 1 — 電流環**：1-4 kHz，驅動器廠商預設值多可直接用
2. **Step 2 — 速度環 PI**：斷開位置環，$K_p$ 逼臨界振盪邊緣 → 回調至 0.6 倍 → 加 $K_i$ 消穩態誤差
3. **Step 3 — 位置環純 P**：速度環 $K_i$ 已消靜差，位置環一般不需要 I
4. **Step 4 — 階躍響應驗收**：Overshoot < 5%（精密任務要求 0%）、Settling time < 50-100 ms
5. **Step 5 — 動力學前饋補償**：把 $\tau_{ff} = M\ddot{q}_d + C\dot{q}_d + G$ **疊加至電流環**
6. **Step 6 — 全軸聯動 + 共振抑制**：掃頻畫 Bode 找共振峰 → 加陷波濾波器 → 驗證 Phase Margin > 45°

</details>

### 串級 PID（Cascade）架構

$$
\text{位置環 (外)} \xrightarrow{e_{\text{pos}}} \text{速度環 (中)} \xrightarrow{e_{\text{vel}}} \text{電流環 (內)}
$$

**物理意義**：外環（位置，~100 Hz P 或 PD）輸出速度指令；中環（速度，~1 kHz PI）輸出電流指令；內環（電流，~10 kHz PI）輸出 PWM。**內快外慢**，內環先穩定才有意義往外調。

<details>
<summary>深入：串級 PID 頻寬設計準則 + 調參順序 + 沒分離的後果</summary>

### 頻寬分離原則

串級控制成功的關鍵是**內環頻寬至少是外環的 5–10 倍**。這樣外環看內環就像一個「理想的追蹤器」。

典型工業伺服驅動頻寬配置：

| 環路 | 頻率 | 頻寬 | 控制器 | 物理意義 |
|------|------|------|--------|----------|
| 電流環 | 10–20 kHz | ~1 kHz | PI | 控制繞組電流 = 控制力矩 |
| 速度環 | 1–5 kHz | ~100 Hz | PI | 控制轉速、抑制負載擾動 |
| 位置環 | 100–500 Hz | ~10 Hz | P 或 PD | 控制角度位置精度 |

### 調參順序（由內而外）

1. **先調電流環**：斷開速度迴路，只給電流環階躍指令，調 PI 讓電流響應 < 1 ms 且無超調
2. **再調速度環**：電流環已穩定，給速度階躍指令，調 PI 讓速度響應在 5-10 ms 穩定
3. **最後調位置環**：速度環已穩定，給位置階躍指令，通常只用 P 或 PD

為什麼不能反過來？如果電流環不穩，速度環的輸出（電流指令）根本追不上，外環怎麼調都沒用。

### 沒有頻寬分離的後果

位置環和速度環頻寬太接近（例如都是 100 Hz）→ 兩環動態耦合互相干擾：
- 低頻振盪（兩環搶控制權）
- 增益裕度急劇下降
- 看似「隨機」的不穩定

</details>

### Feedforward（前饋）— PID 反應式的解藥

**PID 的死穴**：Error-driven，**必須先有誤差才出力** → 高速畫圓必相位滯後。$K_d$ 不能解決（它放大雜訊，不是預測未來）。預測未來靠 **Feedforward**。

**三層 Feedforward**：

1. **Kinematic FF**：$\dot{q}_d, \ddot{q}_d$ 直接前饋給速度/加速度環
2. **Model-based FF（CTC）**：

$$
\tau_{ff} = M(q)\ddot{q}_d + C(q, \dot{q})\dot{q}_d + G(q)
$$

   **物理意義**：用動力學模型算出「做這個軌跡需要多少力矩」，馬達瞬間拿到該力矩不需等誤差累積
3. **ILC (Iterative Learning Control)**：重複任務從歷史誤差學前饋波形，適合晶圓搬運、印刷這類高度重複場景

<details>
<summary>深入：FF 必須注入電流環、FF + FB 架構與失敗模式</summary>

### FF 注入點的血淚經驗

**必須注入電流環（1-4 kHz）**，不能注入位置環：
- 注入位置環會被外環低頻寬（~10 Hz）**過濾掉** → 前饋失效
- 注入電流環 → 馬達瞬間獲得克服慣性+重力所需扭矩 → 軌跡跟蹤誤差銳減

### FF 設計錯誤的連鎖後果

負載估錯 → 過度補償 → FB（PID）反向拉扯 → FF/FB **互相打架** → 低頻共振（兩者週期性拉扯）或高頻抖振（增益邊界震盪）。

### 標準架構

$$
\tau_{\text{total}} = \tau_{ff}^{\text{model}} + \tau_{fb}^{\text{PID}}
$$

加上 **Safety Monitor** 審查力矩峰值 — 若 FF + FB 總和超過硬體安全上限 → 啟動保護模式或降級控制。

### 平台案例

KUKA KRC4 / ABB IRC5：軌跡規劃層**提前數 ms** 算出 $\tau_{ff}$，驅動器端與編碼器 FB 結合。離線規劃 + 線上補償是工業標準。

### 面試角度

常被問「為什麼不用純 FF？」
- 答：**模型永遠不精確**（摩擦、溫度漂移、未建模動態）+ 雜訊擾動
- **FF 處理已知動力學、FB 處理未建模殘差** — 分工明確

</details>

### Loop Shaping 與頻域設計

**時域指標（Overshoot、Settling time）不夠用**：階躍響應看似完美，可能隱藏高共振峰或低相位裕度，感測器雜訊或通訊延遲下瞬間崩潰。**工業驗收必須頻域驗證**。

**頻域三大指標**：
- **頻寬（Bandwidth, BW）**：反應速度
- **相位裕度（Phase Margin, PM）**：阻尼 / 穩定性
- **增益裕度（Gain Margin, GM）**：抗參數漂移

**PM ≈ 100ζ 黃金公式**：

$$
\zeta \approx \frac{\text{PM (degrees)}}{100}
$$

**物理意義**：PM 60° → $\zeta \approx 0.6$（近乎最佳阻尼，超調 < 10% 且整定極快）；PM 45° 是工業最低底線；PM < 30° 接近臨界振盪不可上線。

<details>
<summary>深入：Loop Shaping 三步法 + PID 的頻域本質 + 頻寬三道物理牆</summary>

### Loop Shaping 三步法

1. **定義期望開環 $L(j\omega) = C(j\omega) \cdot P(j\omega)$**：
   - 低頻：高增益（消穩態誤差）
   - 中頻：-20 dB/dec 穿越 0 dB（良好阻尼）
   - 高頻：快速衰減（抑制雜訊與未建模動態）
2. **設計 $C(s)$**：讓 $C(s) \cdot P(s) = L(s)$
3. **驗證**：PM > 45°，GM > 6 dB，Nyquist 曲線遠離 $(-1, j0)$

### PID 的頻域本質

- **P**：整體抬升開環幅頻 → 頻寬↑，PM↓
- **I**：低頻無限增益消靜差，但帶 **-90°** 致命相位滯後
- **D**：高頻 **+90°** 超前相位 → **補償 I 與慣性的相位滯後** → D 的真正價值不是「預測未來」而是「穩定裕度」

### Nyquist 判據

開環穩定時 $Z = N = 0$ → Nyquist 曲線不環繞 $(-1, j0)$；離該點多遠 = 魯棒性多高（這是 GM/PM 的幾何意義）。

### 頻寬三道物理牆

頻寬不能無限大，撞到下列任一面牆就失敗：

1. **Sensor Noise 牆**：D 增益放大高頻雜訊 → 馬達尖叫、發熱、磨損
2. **Actuator Saturation 牆**：大瞬間扭矩 → 電流飽和（Windup）→ 非線性失控
3. **結構共振牆**（最致命）：頻寬蓋到結構共振頻率 → 激發毀滅性結構震盪

### Notch Filter（陷波濾波器）實務

**流程**：掃頻（Chirp signal 0-500 Hz）畫 Bode → 找共振峰 → 串聯陷波濾波器 $G_n(s) = \frac{s^2 + 2\zeta_1\omega_n s + \omega_n^2}{s^2 + 2\zeta_2\omega_n s + \omega_n^2}$（$\zeta_2 \gg \zeta_1$）壓平該峰 + 頻寬退讓。

**真實失敗**：肩部 PID 空載完美；抓 2 kg 工件 → 20 Hz 狂抖。掃頻發現**負載 2 kg 時共振峰落在 25 Hz**，而 PID 頻寬 30 Hz **包覆共振**。修正：加 25 Hz Notch + 頻寬退到 15 Hz → 穩定抓持。

</details>

### Robust PID — IMC、H∞、μ-synthesis

**IMC (Internal Model Control) 設計哲學**：控制器 = 過程逆模型 × 低通濾波器 $f(s)$。模型一致時無差跟蹤，失配時 $f(s)$ 濾掉高頻不確定性保持魯棒。

<details>
<summary>深入：IMC-PID 單參數 λ 調諧 + H∞ 混合靈敏度 + μ-synthesis</summary>

### IMC-PID 對 FOPDT 的推導結果

對一階 + 純延遲 $G(s) = K e^{-Ls} / (\tau s + 1)$，經 Pade 近似 + IMC 推導可直接給出 PID 三參數：

$$
K_p = \frac{\tau + L/2}{K(\lambda + L/2)}, \quad T_i = \tau + L/2, \quad T_d = \frac{\tau L}{2(\tau + L/2)}
$$

**關鍵價值**：PID 三參數**降維到只有一個調諧 $\lambda$**（閉環時間常數）。
- $\lambda$ 小 → 快速響應
- $\lambda$ 大 → 高魯棒性（對模型誤差容忍）

### 為什麼 IMC-PID 是工業軍規

- Z-N 把系統逼到**臨界振盪邊緣** = 等於自殺實驗
- IMC-PID 從數學根源**內建低通 + 模型逆** → 天然對未建模延遲/摩擦免疫
- 石化、半導體產業的 PLC PID 幾乎都是 IMC 派生

### H∞ 混合靈敏度（Mixed Sensitivity）

最小化：

$$
\left\| \begin{bmatrix} W_1 S \\ W_2 KS \\ W_3 T \end{bmatrix} \right\|_\infty < 1
$$

- $S = 1/(1 + PC)$：**靈敏度函數**，$W_1 \cdot S$ 塑造**低頻跟蹤性能**
- $T = PC/(1+PC)$：**補靈敏度函數**，$W_3 \cdot T$ 塑造**高頻魯棒穩定**
- $W_2$ 限制控制量能量

### μ-synthesis（結構化不確定性）

機器人連桿質量/摩擦不確定性定義為 $m = m_0(1 + 0.2\delta), \, \|\delta\| \leq 1$；μ-synthesis **保證所有可能的 $\Delta$ 最壞情況下仍穩定**。

### 真實失敗與修正

**失敗**：ABB IRB 換非標定端拾器（質量未知）→ 固定 PID 過沖 → 觸發過載。
**修正**：H∞ 混合靈敏度，質量建模為 $m \in [1, 15]$ kg **乘性不確定性** → 犧牲 10% 空載頻寬換全負載無條件穩定。

</details>

### PID + Observer + Model-based FF 現代架構

**純 PID 不夠**的三大原因：
1. Reactive：必須先有誤差才出力
2. D 對編碼器位置直接求導 → 高頻量化雜訊放大 → 燒馬達
3. 無法預見重力、科氏力等非線性

**現代完整架構**：

$$
\tau = \underbrace{M(q)(\ddot{q}_d + K_v \dot{e} + K_p e)}_{\text{CTC + PD}} + \underbrace{C(q, \dot{q})\dot{q} + G(q)}_{\text{抵消非線性}}
$$

<details>
<summary>深入：CTC 反饋線性化 + Observer (EKF/DOB) + 分層控制哲學</summary>

### CTC + PID 聖杯

**反饋線性化原理**：
- 前半 $C \dot{q} + G$ **抵消非線性項**
- 乘 $M(q)$ **歸一慣量**
- 結果：N 軸耦合非線性 → **N 個獨立線性雙積分器** → 外環 PD 只處理極小線性殘差

這是「PID 能在複雜機械臂上 work 的根本原因」。

### Observer 整合

**EKF / Luenberger Observer**：結合「動力學預測 + 感測器修正」 → 輸出平滑無延遲 $\hat{\dot{q}}$

$$
\hat{\dot{q}}_{k+1} = \hat{\dot{q}}_k + \ddot{q}_{\text{model}} \cdot dt + L(q_{\text{measured}} - \hat{q}_k)
$$

**關鍵價值**：餵給 PID 微分環 → **D 增益可放心開大**，不怕放大雜訊。

### DOB (Disturbance Observer)

$$
\hat{d} = \text{LPF}(\tau_{\text{ideal}} - \tau_{\text{measured}})
$$

**物理意義**：把「理想模型應該出的力矩」減去「實際測量力矩」，低通濾波後就是未建模擾動的估計。前饋抵消未建模摩擦 / 外力，**不依賴精確摩擦模型** — 這是熱機漂移的終極兜底。

### 具身智能分層控制哲學（面試展現系統觀必備）

| 層級 | 頻率 | 角色 | 技術 |
|------|------|------|------|
| **大腦** | 1 Hz | 語義理解、高層任務拆解 | VLM / LLM |
| **小腦** | 100 Hz | 局部軌跡優化、策略推理 | MPC / RL（SRBD 上滾動優化）|
| **脊髓** | 1-4 kHz | 毫秒級硬即時電流跟蹤、硬體安全保護 | **CTC + 增量式 PID** |

**面試答題模版**：
> 「我設計的不是一個控制器，是**分層解耦防禦體系**：1 kHz Luenberger 清雜訊 → CTC 剝 90% 非線性 → PID 處理 10% 線性殘差 → 100 Hz MPC/RL 負責長視角避障與決策。讓 RL 做擅長的泛化，讓 PID 守物理穩定的絕對底線。」

### 真實失敗與修正

**失敗**：四足腿部直接端到端 RL 輸出力矩（20 ms 推理延遲）→ 馬達電流振盪過熱燒毀
**修正**：「RL 50 Hz 出參考腳端軌跡 + 關節 CTC + PID 2 kHz 跟蹤」混合架構 → 馬達溫控在 45°C 連跑 2 小時

</details>

### Digital PID 與離散化（嵌入式實務）

**三種離散化方法**：

| 方法 | 公式 | 特性 |
|------|------|------|
| Backward Euler | $s = (1 - z^{-1})/T$ | 最簡單，高頻失真嚴重 |
| **Tustin / Bilinear（推薦）** | $s = \frac{2}{T} \cdot \frac{1 - z^{-1}}{1 + z^{-1}}$ | 左半 s 平面映射到 z 單位圓**內部**（穩定極點保持穩定），Bode 最準；但虛軸 $j\omega$ 會發生 **frequency warping**，關鍵頻點實作時常用 pre-warping 校正 |
| ZOH | — | 輸入階梯狀 → DAC 場景最符物理 |

**採樣頻率經驗法則**：$f_s = f_{\text{BW}} \times 10 \sim 15$
- 工業伺服電流環頻寬 100-300 Hz → **採樣 1-4 kHz**
- ADC 前必須類比 Anti-aliasing LPF（防止奈奎斯特折疊）

<details>
<summary>深入：Position vs Incremental Form 救命差異 + D on Measurement + LPF Filtered Derivative</summary>

### Position Form vs Incremental Form 實戰對比

| 特性 | Position Form | Incremental Form |
|------|---------------|------------------|
| 輸出 | 絕對值 $u(k)$ | 增量 $\Delta u(k)$，累積為 $u(k) = u(k-1) + \Delta u(k)$ |
| **通訊斷連** | MCU 當機 → 馬達**鎖最後錯誤指令** | $\Delta u = 0$ → 馬達**平滑保位** |
| Windup | 必須顯式 Anti-Windup | 天然免疫（不累積歷史）|
| 適用 | 實驗室、連線穩定 | 野外、無線通訊、救命場景 |

**真實失敗**：輪式機器人用 Position Form → Wi-Fi 斷連 2 秒 → 舊指令持續 → **全速撞牆**。
**修正**：改 Incremental + Watchdog，逾時 $\Delta u = 0$ 平滑煞車。

### Derivative on Measurement（微分先行）

**問題**：對誤差微分 $K_d \cdot \dot{e}$，當 setpoint 階躍變化時會產生 **Derivative Kick**（瞬間極大脈衝）。

**修正**：改對**量測值**微分

$$
D_{\text{term}} = -K_d \cdot \dot{x}_{\text{actual}}
$$

setpoint 變化不再產生脈衝，同時仍能抑制快速擾動。

### Low-pass Filtered Derivative（不完全微分）

純離散微分 $D = K_d (e(k) - e(k-1))/T$，$T = 1$ ms 下 quantization noise 被 **放大 1000 倍**。

**正解 — 濾波微分**：

$$
D(k) = \frac{T_d}{T_d + N \cdot T_s} D(k-1) + \frac{K_d \cdot N}{T_d + N \cdot T_s}(e(k) - e(k-1))
$$

- $N = 10 \sim 20$：濾波係數
- 截止頻率設為系統頻寬 1/5 ~ 1/10

### 定點數實作（ARM Cortex-M0 / 8051 無 FPU）

Q-format：增益 × 1024 左移 10 位，算完右移 10 位 → 單週期完成，無浮點硬體也能跑。

</details>

### Gain Scheduling 與 Adaptive PID

**Gain Scheduling — 工業機械臂 / 機器人關節控制的主流解**（高階平台如 KUKA iiwa / Franka Panda 基於 $M(q)$ 動態調增益；一般工業臂常用依姿態/負載切換參數表）：

$$
K_p^{(i)}(q) = K_{p_0}^{(i)} \cdot \frac{M_{ii}(q)}{M_{ii,0}}, \quad K_d^{(i)} \text{ 保持阻尼比恆定}
$$

**物理意義**：對每個關節 $i$，SISO PID 實際感受到的是 mass matrix 的**對角元素 $M_{ii}(q)$**（或操作空間慣量 $\Lambda_{ii}(q)$）— 即「該軸當下的有效慣量」。機械臂伸展時某些 $M_{ii}$ 放大 → 需高 $K_p$；摺疊時 $M_{ii}$ 小 → 低 $K_p$；用錯會遲鈍或震盪。**不要用 $\det(M)$**：determinant 是整個耦合系統的性質，在奇異附近會趨近 0 或暴增，與單軸感受到的慣量不單調對應。無人機依電池電壓動態放大 PID（Voltage Scaling）是類似概念的單通道版本。

<details>
<summary>深入：MRAC / STR / RL-based Tuning + 為什麼工業界不用 Neural/RL PID</summary>

### 進階自適應控制技術

**MRAC (Model Reference Adaptive Control)**：
- 定義**理想參考模型**（想要的響應）
- 實際誤差經**李雅普諾夫法則**推導自適應律
- 強迫物理系統表現如理想模型

**STR (Self-Tuning Regulator)**：
- **RLS 線上辨識**系統參數
- 代數**極點配置**重算 PID 增益
- 慣性突變場景（抓取未知重物）

**RL-based Gain Tuning**：
- State = 風速 / 地形 / 負載
- Action = $(\Delta K_p, \Delta K_i, \Delta K_d)$
- **底層保留傳統 PID**，RL 只調增益

### 為什麼工業界絕大多數場景不用 Neural / Fuzzy / RL PID

1. **追求絕對穩定性與可預測性（Deterministic）**：工廠不容許「偶爾失控」
2. Gain Scheduling 每個工作點可用 Bode 驗證 PM > 45° + GM > 6 dB
3. 黑箱 RL 在 **OOD (Out-of-Distribution) 狀態**可能輸出極端增益 → 損毀機台
4. **無法通過 ISO 13849 / ISO 10218 功能安全認證** — 這是工業機器人上市的法律底線

### 面試答題分水嶺：「我調過 PID」vs「我設計自適應 PID 系統」

**普通答法**：「我用 Z-N 法調過 $K_p / K_i / K_d$」— 面試官 yawn
**A 級答法**：
> 「我設計過自適應 PID — 發現固定參數無法應對負載突變；引入基於 $M(q)$ 的 Gain Scheduling + 變增益 $K_p = K_{P_{\text{high}}} \cdot (1 - \text{sech}(a \cdot e(t)))$；**確保所有姿態下 PM > 60°**，這才是工業級解法。」

### 真實失敗與修正

**失敗**：四足草地行走正常，冰面 PID 過剛 → 滑倒
**修正**：**Fuzzy-PID** 基於足底**滑移檢測**（高頻速度誤差）自動降 $K_p$ 軟化關節

</details>

## 直覺理解

**開車類比**（三項各司其職）：
- **P = 盯車距踩油門**：前車離你越遠，踩油門越猛。但純靠 P，追到接近時油門太小，永遠差一點（穩態誤差）
- **I = 逆風中慢慢加油**：發現持續被風吹、車距一直沒縮到零，I 慢慢加油補償。但加太久會「過度加速」（windup）
- **D = 看前車煞車燈提前減速**：還沒碰到就先收油門，避免追撞（超調）。但路面顛簸（雜訊）時 D 會讓你瘋狂踩放油門

**分層控制類比**：大腦（GPS 規劃路線 1 Hz）→ 小腦（MPC 決定油門剎車序列 100 Hz）→ 脊髓（PID 把油門踏板從 30% 追到 35% 並在打滑時立即反射 1 kHz）。少一層都不完整。

**模擬器觀察**：在 Gazebo 或 Isaac Sim 中開一個單關節位置控制：
1. **純 P**：設 $K_p = 10$，觀察關節追目標但永遠差一點（重力造成穩態誤差）
2. **加 I**：$K_i = 5$，穩態誤差消失，但如果給一個超出力矩限制的目標 → 關節飽和 → 釋放後瘋狂超調（windup 現象肉眼可見）
3. **加 anti-windup**：同樣場景，超調消失
4. **加 D**：$K_d = 1$，響應更快、超調更小；但把 $K_d$ 調到 10 → 關節開始抖（D 放大編碼器量化雜訊）
5. **加重力前饋**：$\tau_{ff} = mgL\cos\theta$，穩態誤差瞬間消失且不需 I → 直觀理解 FF 比 I 快、比 I 穩
6. **掃頻測共振**：慢慢增大 $K_p$ 觀察哪個頻率開始高幅振動 → Notch Filter 壓那一峰

**一句話記憶**：「P 管現在、I 管過去、D 管未來；Anti-Windup 管 I 不要失控；FF 管已知動力學、讓 PID 只修殘差。」

## 實作連結

**四個典型工程場景**：

1. **ROS 2 關節位置控制**：`ros2_controllers` 中的 `JointTrajectoryController` 內部就是 PID。啟動時載入 YAML 參數（$K_p, K_i, K_d, i_{\text{clamp}}$），每個 control tick 從 `/joint_states` 讀回實際角度、和 trajectory 插值後的目標角度算誤差、跑 PID、輸出力矩指令。

2. **嵌入式馬達驅動**：STM32 + FOC 驅動器中，電流環 PID 跑在 10 kHz 中斷裡，用增量式避免位置式的 windup 問題。速度環 1 kHz、位置環 100 Hz，層層嵌套在不同頻率的中斷中。

3. **無人機飛控**（PX4 / Betaflight）：Attitude + Rate 雙環串級，Acro mode 只調內環 Rate PID 達成極致跟手感。

4. **力控機械臂**（Franka Panda）：絕不用純力 PID — 必墊 **Impedance Inner Loop**，虛擬阻尼 $F = M\ddot{x} + B\dot{x} + K x$ 消耗碰撞能量，避免剛性接觸極限環振盪。

**Code 骨架**（C++，ROS 2 風格）：

```cpp
// PID 控制器核心（含 conditional anti-windup + D on measurement + LPF）
struct PIDController {
    double kp, ki, kd;
    double integral = 0.0;
    double prev_measurement = 0.0;
    double d_filtered = 0.0;
    double d_filter_tc;       // D 低通時間常數
    double i_clamp;           // 積分項限幅
    double output_max;

    double compute(double setpoint, double measurement, double dt) {
        double error = setpoint - measurement;
        // P
        double p_term = kp * error;
        // I（含 conditional anti-windup）
        double output_unsat = p_term + ki * integral;
        if (std::abs(output_unsat) < output_max) {
            integral += error * dt;
        }
        integral = std::clamp(integral, -i_clamp, i_clamp);
        double i_term = ki * integral;
        // D on Measurement + LPF（避免 kick + 抑雜訊）
        double d_raw = -kd * (measurement - prev_measurement) / dt;
        double alpha = dt / (dt + d_filter_tc);
        d_filtered = alpha * d_raw + (1 - alpha) * d_filtered;
        prev_measurement = measurement;
        // 輸出限幅
        double output = p_term + i_term + d_filtered;
        return std::clamp(output, -output_max, output_max);
    }
};
```

<details>
<summary>深入：完整 Python 實作（Incremental Form + Back-calculation + 模擬測試）</summary>

```python
import numpy as np
import matplotlib.pyplot as plt

class IncrementalPID:
    """工業級 PID 控制器：Incremental Form + Back-calculation anti-windup
    + D on Measurement + LPF Filtered Derivative。"""

    def __init__(self, kp, ki, kd, output_min, output_max,
                 tracking_tc=None, d_filter_tc=0.01):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.d_filter_tc = d_filter_tc

        # Back-calculation tracking time constant
        if tracking_tc is None:
            ti = kp / ki if ki > 0 else 1.0
            td = kd / kp if kp > 0 else 0.0
            self.tracking_tc = np.sqrt(ti * td) if td > 0 else ti
        else:
            self.tracking_tc = tracking_tc

        self.integral = 0.0
        self.prev_measurement = 0.0
        self.d_filtered = 0.0
        self.u_prev = 0.0

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        # P term
        p_term = self.kp * error

        # I term（Back-calculation anti-windup 之後再更新）
        i_term = self.ki * self.integral

        # D on Measurement + LPF（避免 setpoint kick + 抑雜訊）
        d_raw = -self.kd * (measurement - self.prev_measurement) / dt
        alpha = dt / (dt + self.d_filter_tc)
        self.d_filtered = alpha * d_raw + (1 - alpha) * self.d_filtered
        self.prev_measurement = measurement

        # 未限幅輸出
        u_raw = p_term + i_term + self.d_filtered
        # 限幅
        u_sat = np.clip(u_raw, self.output_min, self.output_max)

        # Back-calculation: 飽和差值衰減積分
        self.integral += (error + (u_sat - u_raw) / self.tracking_tc) * dt
        self.u_prev = u_sat
        return u_sat


def simulate_joint(setpoint_fn, controller, plant_mass=1.0, gravity=9.81,
                   link_len=0.5, torque_limit=15.0, duration=5.0, dt=0.001,
                   inject_ff=False):
    """模擬單關節位置控制：重力 + 力矩飽和 + 可選重力前饋。"""
    steps = int(duration / dt)
    t = np.zeros(steps)
    pos = np.zeros(steps)
    vel = np.zeros(steps)
    cmd = np.zeros(steps)

    for k in range(1, steps):
        t[k] = k * dt
        sp = setpoint_fn(t[k])
        u_fb = controller.compute(sp, pos[k - 1], dt)

        # 重力前饋（FF + FB 架構示範）
        u_ff = plant_mass * gravity * link_len * np.cos(pos[k - 1]) if inject_ff else 0.0
        u_total = u_fb + u_ff
        u_total = np.clip(u_total, -torque_limit, torque_limit)
        cmd[k] = u_total

        # 動力學：ma = τ - mgL·cos(θ)
        accel = (u_total - plant_mass * gravity * link_len * np.cos(pos[k - 1])) / plant_mass
        vel[k] = vel[k - 1] + accel * dt
        pos[k] = pos[k - 1] + vel[k] * dt

    return t, pos, cmd


# 實驗：比較 PID vs PID + FF
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
sp_fn = lambda t: 1.5 if t > 0.1 else 0.0

for label, ff, color in [
    ('PID only', False, 'red'),
    ('PID + Gravity FF', True, 'green'),
]:
    ctrl = IncrementalPID(kp=30, ki=10, kd=5,
                          output_min=-15, output_max=15)
    t, pos, cmd = simulate_joint(sp_fn, ctrl, inject_ff=ff)
    axes[0].plot(t, pos, label=label, color=color)
    axes[1].plot(t, cmd, label=label, color=color, alpha=0.7)

axes[0].axhline(1.5, ls='--', color='gray', label='Setpoint')
axes[0].set_ylabel('Joint angle (rad)')
axes[0].set_title('PID vs PID + Gravity FF (torque limit 15 N·m)')
axes[0].legend()
axes[1].set_ylabel('Torque command (N·m)')
axes[1].set_xlabel('Time (s)')
axes[1].legend()
plt.tight_layout()
plt.savefig('pid_vs_ff.png', dpi=150)
plt.show()
```

**觀察重點**：
- PID only：穩態依賴 $K_i$ 慢慢爬，且峰值力矩接近飽和
- PID + FF：瞬間到位、峰值力矩遠低於飽和、無穩態誤差依賴 I

</details>

<details>
<summary>深入：特殊應用場景的 PID 差異化設計</summary>

### 無人機姿態控制（PX4 / Betaflight）

- **外環 Angle PID**：目標角度 → 目標角速度
- **內環 Rate PID**：目標角速度 → 馬達推力差
- **Acro mode**：搖桿輸入**直接當目標角速度**繞過外環 → 無限翻滾極致跟手感，飛手專屬
- 調參重點：Rate PID 是核心，Angle PID 只是外殼

### AGV / 差動輪機器人

- **不直接控左右輪**，先解耦到「線速度 $v$ + 角速度 $\omega$」
- **Pure Pursuit + PID**：
  - Pure Pursuit 算前瞻點 → 給期望曲率 $\kappa$（$\omega = v \kappa$）
  - PID 跟蹤 $v, \omega$
  - 最後逆解算 $v_L = v - \omega L/2, v_R = v + \omega L/2$
- **ROS 2 Nav2 DWB / Regulated Pure Pursuit** 實際採用此架構

### 液壓系統

- 油液壓縮遲滯 + 閥門死區 + 高度非線性
- **$K_i$ 必須極小**：系統響應慢，大 $K_i$ 在誤差消除前瘋狂累積 → 閥門打開時撞缸
- **策略**：Feedforward 預估閥開度 + 強 P 為主 + 極小 I
- 工程機械（挖掘機、舉升平台）必備

### 力控 PID（剛性接觸陷阱）

- 剛性桌面**微小位移 → 巨大接觸力突變**
- 純力 PID 的 I 項累積 → **高頻極限環振盪（Limit Cycle Oscillation）**
- **必須搭配 Impedance Inner Loop 或 Damping Injection**：

$$
F = M \ddot{x} + B \dot{x} + K x
$$

  人為加虛擬速度反饋阻尼 $B \dot{x}$ 消耗碰撞能量

- **真實失敗**：Franka peg-in-hole 直接用末端力 PID → 接觸金屬瞬間 50 Hz 狂抖觸發急停
- **修正**：改**導納控制（Admittance Control）**柔順架構 → 「像浸在糖漿中」的阻尼特性徹底消除接觸震盪

### 面試答題「你在機器人上調過什麼 PID」

展現「懂應用脈絡」：
- 「無人機 Acro 我只調 Rate PID，因為搖桿給的是目標角速度」
- 「AGV 我先解耦到 $v/\omega$ 再調，不直接對輪子下 PID」
- 「液壓我 $K_i$ 設極小，主靠 Feedforward 補閥開度」
- 「力控絕不用純力 PID，必墊阻抗內環」

</details>

## 常見誤解

1. **「D 增益越大越穩」** — 錯。D 項本質是對誤差做微分，高頻雜訊（編碼器量化、ADC 雜訊）會被放大。$K_d$ 過大 → 控制量高頻抖動 → 馬達嗡嗡聲 → 機構磨損。**正確做法**：D 項前面加一階低通（截止頻率約控制頻率的 1/10），或改用 **Derivative on Measurement**（對 $-x_{\text{actual}}$ 微分而非 $e$）避免 setpoint 階躍脈衝。

2. **「有穩態誤差就無腦加 I」** — 穩態誤差不一定要靠 I。如果誤差來自已知的重力或摩擦，用**前饋補償**（$u_{ff} = mg$ 或 $\hat{f}_{\text{friction}}$）+ PD 效果更好、響應更快、不會引入 I 的相位滯後和 windup 風險。**原則**：**已知擾動用前饋消、未知慢變擾動才靠 I**。

3. **「PID 能搞定一切」** — PID 是線性控制器，面對非線性系統（高速慣量耦合、Coriolis、接觸力突變）會力不從心。工業伺服標準做法是 **CTC 前饋 + PD 回饋**（$\tau_{ff} = M\ddot{q}_d + C\dot{q}_d + G$），PID 只修前饋沒算準的殘差。面試時講「PID 是回饋、dynamics 是前饋、兩者互補」是加分句。

4. **「沒寫 anti-windup 也沒差」** — 模擬器裡力矩沒真正飽和可能看不出問題。但實機有物理力矩上限，卡住（碰撞、堵轉）時 I 項爆掉 → 釋放瞬間砸飛工件或撞壞夾爪。**任何上線的 PID 都必須有 anti-windup + 串級時必須傳遞飽和訊號給外環**（Tracking Anti-Windup）。

5. **「Ziegler-Nichols 是標準答案」** — Z-N 假設 FOPDT 線性模型，機械臂違反三處（重力耦合、非線性摩擦、多 DoF 耦合），且臨界振盪實驗可能**直接摧毀諧波減速器**。**正確流程**：SysID → Model-based FF → PID 只處理線性化殘差 → Bode 驗證 PM > 45°。IMC-PID 的 $\lambda$ 單參數調諧更適合工業場景。

6. **「RL 取代 PID 是未來」** — 學界熱、工業冷。工業追求 Deterministic 可預測性，**無法通過 ISO 13849 功能安全認證**。主流架構是 **RL 10-50 Hz 出參考軌跡 + PID/CTC 1 kHz 跟蹤**（分層），或 **RL 做 Gain Scheduling**（底層仍是 PID）。面試時被問「RL vs PID」，答「分層混合」而非「取代」。

7. **「Position Form 和 Incremental Form 沒差」** — 天差地遠。通訊斷連 / MCU 當機時，Position Form 讓馬達鎖最後錯誤指令衝出去；Incremental Form 的 $\Delta u = 0$ 會平滑保位。**野外機器人、無線通訊場景必用 Incremental Form**。

## 練習題

<details>
<summary>Q1：機械臂關節追蹤目標時穩態總差 0.5°，你會怎麼分析？先做什麼、再做什麼？</summary>

**完整推理鏈**：

1. **先判斷穩態誤差來源**：0.5° 在重力方向 → 高度懷疑重力未補償。讓關節水平放（重力不作用在關節軸上），看誤差是否消失
2. **如果水平放誤差消失**：確認是重力矩 → 不要加 I，改用**重力前饋** $\tau_{ff} = mgL\cos\theta$，PD 即可消除穩態誤差
3. **如果水平放仍有誤差**：可能是摩擦（Coulomb / Stribeck），嘗試摩擦前饋 $\hat{f} = f_c \text{sign}(\dot\theta) + f_v \dot\theta$
4. **前饋都加了仍有殘差**：此時才加 $K_i$，但**必須搭配 Conditional Anti-Windup**，$i_{\text{clamp}}$ 設為比穩態修正量稍大
5. **進階**：熱機後 $F_v$ 漂移 → 加 DOB（Disturbance Observer）線上補未建模擾動，不依賴精確摩擦模型
6. **要避開的陷阱**：直接加大 $K_i$ → 相位裕度下降 → 系統可能開始振盪，尤其在低速反轉（Stribeck 區域）

**面試官想聽到**：先分析根因、優先用 FF 解已知擾動、I 是最後手段且必須帶 anti-windup、熱機漂移用 DOB 兜底。

</details>

<details>
<summary>Q2：機械臂碰到障礙物卡住三秒後鬆開，關節瘋狂甩動超調，怎麼診斷和修？（含串級耦合）</summary>

**完整推理鏈**：

1. **現象分析**：卡住 → 誤差持續為正 → I 項三秒內狂累積 → 鬆開瞬間 P+I+D 全部釋放 → 巨大控制量 → 關節高速甩動 → 典型 **Integral Windup**
2. **立即驗證**：在 log 裡看 I 項數值，卡住期間 I 會單調上升到極大值
3. **修法（由簡到精）**：
   - **Step 1 — Conditional**：飽和時凍結積分，$i_{\text{clamp}}$ 使得 $K_i \cdot i_{\text{clamp}} < u_{\text{max}}$
   - **Step 2 — Back-Calculation**：仍有輕微超調 → 用 $T_t = \sqrt{T_i \cdot T_d}$ 讓退飽和更平滑
   - **Step 3 — 串級 Tracking Anti-Windup**：**關鍵！** 位置環、速度環、電流環任一飽和 → 布林訊號傳遞給所有外層，同步凍結積分。否則內環飽和時外環還在瘋狂積分 → 即使每層單獨有 anti-windup 仍爆
   - **Step 4 — 碰撞偵測 + Safe Mode**：當力矩指令持續飽和且速度為零，判定碰撞，主動清零 I 並切換安全模式（低增益 + 重力補償 only）
4. **要避開的陷阱**：只調低 $K_i$ 不加 anti-windup — 穩態性能變差但根因沒解決

**面試官想聽到**：能精確描述 windup 的物理過程（卡住 → I 爆 → 釋放超調）、**懂串級 Tracking Anti-Windup 的層級耦合**、能進階到碰撞偵測 + 安全模式。

</details>

<details>
<summary>Q3：你的 PID 空載完美，抓 2 kg 工件後肩部 20 Hz 狂抖，怎麼診斷？</summary>

**完整推理鏈**：

1. **第一直覺：共振包覆**。負載改變 → 結構共振頻率下移 → 可能撞到 PID 頻寬
2. **驗證**：做**掃頻測試（Chirp signal）**— 輸入 0-500 Hz 頻率掃描訊號，量測響應，畫 Bode plot
3. **觀察**：負載 0 kg 時共振峰在 50 Hz（PID 頻寬 30 Hz 安全）；負載 2 kg 時共振峰降到 25 Hz → **PID 頻寬 30 Hz 已包覆共振** → 激發結構震盪
4. **修法**：
   - **加 Notch Filter**：在 25 Hz 處串聯陷波濾波器壓平該峰
   - **頻寬退讓**：降 $K_p$ 讓 PID 頻寬退到 15 Hz
   - **Gain Scheduling**：依負載質量切換兩套 PID 參數（空載 vs 2 kg）
   - **進階（H∞）**：把質量建模為 $m \in [0, 5]$ kg 乘性不確定性，求 H∞ 解 → 犧牲 10% 空載頻寬換全負載無條件穩定
5. **要避開的陷阱**：直接加大 D 項「阻尼」— D 放大 20 Hz 附近雜訊，抖得更厲害

**面試官想聽到**：懂**頻寬三道物理牆**（共振是最致命那道）、知道 Chirp 掃頻是標準診斷、能升級到 H∞ 魯棒設計。

</details>

<details>
<summary>Q4：你在 UR5 上設計串級 PID 做高速 pick-and-place，速度環調完後位置環加不上去（一加就振盪），為什麼？</summary>

**完整推理鏈**：

1. **第一個懷疑：頻寬分離不夠**。檢查速度環實際頻寬，若已經 50 Hz，位置環想做 30 Hz → 倍數不夠 5-10 倍 → 兩環耦合搶控制權
2. **驗證**：用系統辨識測速度閉環的 Bode，看 -3 dB 點在多少 Hz
3. **修法**：
   - **降速度環頻寬**：通常速度環「能穩就好」，不必逼極限
   - **降位置環頻寬**：從 30 Hz 退到 10 Hz，確保內外頻寬 1:10
   - **電流環先驗證**：如果電流環頻寬 < 500 Hz，速度環想做 100 Hz 本身就不合理 → 問題上溯到電流環
4. **檢查調參順序錯誤**：是否先調位置環才調速度環？若是 → 重頭來。**由內而外調是鐵律**
5. **動力學前饋**：速度環前饋 $\dot{q}_d$、電流環前饋 $\tau_{ff} = M\ddot{q}_d + C\dot{q}_d + G$ → 位置環只處理殘差，可大幅降低所需增益
6. **要避開的陷阱**：在外環加 filter 壓振盪 — 相位滯後更慘、問題更難診斷

**面試官想聽到**：懂**頻寬分離 5-10 倍鐵律**、知道由內而外調、FF 注入電流環的物理意義、不要在外環 hack filter。

</details>

<details>
<summary>Q5：輪式機器人現場部署，Wi-Fi 斷線 2 秒後全速撞牆。為什麼？怎麼修？</summary>

**完整推理鏈**：

1. **現象分析**：Wi-Fi 斷線 → 遠端指令無更新 → MCU 接收緩衝區保留**最後一筆指令** → PID 照舊執行 → 馬達全速衝出去
2. **根因**：用了 **Position Form PID**，$u(k)$ 是絕對值。MCU 沒收到新誤差但仍輸出舊 $u$
3. **修法 — Incremental Form 救命**：
   - 改用 $\Delta u(k) = K_p(e(k)-e(k-1)) + K_i e(k) \Delta t + K_d \cdot \ldots$
   - 通訊斷連 → $e$ 無更新 → $\Delta u = 0$ → 馬達**平滑保位**，不再累積歷史
   - 天然免疫 Windup（不儲存 $\int e$）
4. **加 Watchdog**：通訊逾時 > 100 ms → 強制 $\Delta u = 0$ + 緩慢減速 + 亮警示燈
5. **分層防禦**：
   - MCU 本地保留 Safe Trajectory（回家路線）
   - 長時間斷線（> 5 秒）→ 主動停機 + 切到低功耗等待重連
6. **面試 bonus**：這是「為什麼嵌入式用 Incremental Form」的經典題，沒做過野外部署的人答不出來

**面試官想聽到**：懂 Position vs Incremental Form 的實戰差異、Watchdog + Safe Trajectory 的分層防禦思維。

</details>

<details>
<summary>Q6：Franka peg-in-hole 用末端力 PID 接觸金屬瞬間 50 Hz 狂抖觸發急停，怎麼解？</summary>

**完整推理鏈**：

1. **現象分析**：剛性接觸 → 微米級位移產生巨大力突變 → 純力 PID 的 P 項劇烈反應 → 反作用 → 再次接觸 → 形成**高頻極限環振盪（Limit Cycle Oscillation）**
2. **根因**：剛性-剛性接觸的傳遞函數有**極高增益極點**，純力 PID 的帶寬遠超該極點 → 激發震盪
3. **修法 — Impedance / Admittance 阻抗控制**：
   - 虛擬質量-彈簧-阻尼 $F = M\ddot{x} + B\dot{x} + K x$
   - 人為加 $B\dot{x}$ 虛擬阻尼消耗碰撞能量
   - 「像浸在糖漿中」的手感 → 穩定接觸
4. **進階 — Passivity-Based Control**：保證整個系統被動（passive），能量只流向耗散 → 無條件穩定
5. **架構選擇**：
   - **Impedance**：力模式下精確，需要高精度力感測器
   - **Admittance**：位置模式下精確，力感測器經過雙積分較易處理
6. **要避開的陷阱**：降低力 PID 增益 — 變慢但震盪仍在（只是週期變長）

**面試官想聽到**：懂**力控不能純用力 PID 的物理原因**（剛性接觸的系統動力學）、知道 Impedance / Admittance 差異、能帶到 Passivity。

</details>

<details>
<summary>Q7：UR5 早上測試完美，下午熱機 30 分鐘後開始撞機。為什麼？</summary>

**完整推理鏈**：

1. **現象定位**：「早上 vs 下午」差異 → 時間相關 → 熱機效應
2. **物理原因**：諧波減速器潤滑脂冷機時黏度極大，溫升後**黏滯摩擦 $F_v$ 下降 20-30%**
3. **錯誤連鎖**：
   - 冷機辨識的摩擦參數 $F_v^{\text{cold}}$ 被當 nominal 寫入 CTC 前饋
   - 熱機後真實 $F_v^{\text{hot}} < F_v^{\text{cold}}$
   - 前饋 $\tau_{ff} = F_v^{\text{cold}} \cdot \dot{q}$ **過度補償** → 實際加速比期望大
   - 接觸速度暴衝 → 撞擊力超過安全閾值 → E-stop
4. **修法**：
   - **短期**：現場 warm-up 30 分鐘後重跑 SysID（工業機器人手冊常見要求）
   - **中期 — RLS 線上自適應**：用 Recursive Least Squares + 遺忘因子追蹤溫度漂移，即時微調 $F_v$
   - **長期 — DOB 兜底**：$\hat{d} = \text{LPF}(\tau_{\text{actual}} - \tau_{\text{ideal}})$ 作前饋，不依賴摩擦模型
   - **接觸前降級**：進入接觸 50 mm 前切換為阻抗控制，目標是低剛度安全接觸
5. **進階**：部署 Temperature-Aware Gain Scheduling，依溫感測器動態調 $K_p$
6. **面試 bonus**：這是「為什麼工業機器人需要 warm-up」的經典題

**面試官想聽到**：懂摩擦模型受溫度影響、知道 RLS / DOB 是 online adaptation 標準解、接觸前降級是工程保險。

</details>

## 面試角度

1. **Anti-Windup 是防禦性編程的最低要求，串級必須 Tracking Anti-Windup** — 區分「寫過模擬器 PID」和「部署過實機 PID」的分水嶺。**為什麼這是重點**：實機有物理飽和，任何卡住/堵轉場景都會觸發；串級時即使每層獨立有 anti-windup 也不夠，**內環飽和必須傳訊號給外環同步凍結**。帶出：「我部署的 PID 預設三道防禦 — Conditional Anti-Windup + Back-Calculation + Tracking across cascade。沒做第三道就是定時炸彈。」

2. **Ziegler-Nichols 調不動機械臂，工業真正用 SysID + CTC + IMC-PID** — 展現你不是只會教科書。**為什麼這是重點**：機械臂違反 FOPDT 三處（重力耦合、非線性摩擦、多 DoF 耦合），Z-N 臨界振盪實驗**可能摧毀諧波減速器**。帶出：「我不用 Z-N — 先跑 SysID 辨識 $M(q), C, G$，前饋抵消非線性，PID 只處理線性化殘差；調諧用 IMC-PID 的 $\lambda$ 單參數，比 $K_p/K_i/K_d$ 三維好調一個數量級。」

3. **PM ≈ 100ζ 黃金公式 + 頻寬三道物理牆 + Notch Filter 抑共振** — 頻域設計的產業接軌點。**為什麼這是重點**：時域指標（Overshoot、Settling time）看似完美可能隱藏共振；抓負載改變共振頻率是真實現場的標誌性故障。帶出：「PID 頻寬不能無限大，會撞三道牆：感測器雜訊、致動器飽和、結構共振。我會用 Chirp 掃頻找共振峰，加 Notch Filter 壓平 + 頻寬退讓，確保 PM > 60°（$\zeta \approx 0.6$）。」

4. **FF + FB 分工：FF 處理已知動力學、FB 處理未建模殘差，FF 必須注入電流環** — 系統性思考而非「調參數到天荒地老」。**為什麼這是重點**：純 FB（PID）是 reactive，必須先有誤差才出力，高速跟蹤永遠相位滯後；純 FF 模型不精確必失效。帶出：「穩態誤差我不會第一時間加 I — 分析是重力還是摩擦，用前饋消掉已知項，PID 只負責修殘差。FF **必須注入電流環**，注入位置環會被低頻寬過濾掉。」

5. **Cascade PID 頻寬分離 5-10 倍，由內而外調** — 工業伺服真實架構。**為什麼這是重點**：教科書只講單環 PID，實際伺服是三環串級；頻寬不分離會耦合振盪。帶出：「實際伺服驅動是三環串級 — 電流 10 kHz、速度 1 kHz、位置 100 Hz，頻寬比 5-10 倍。**先調電流環**（內環不穩外環無意義），FF 注入電流環。面試答 PID 只答單環是不完整的。」

6. **Incremental Form 救命**：通訊斷連場景的試金石。**為什麼這是重點**：野外機器人 / AGV / 無人機遇 Wi-Fi 斷線是常態；Position Form 會讓馬達鎖最後錯誤指令衝出去撞牆。帶出：「嵌入式我預設用 Incremental Form — $\Delta u = 0$ 在通訊斷連時馬達平滑保位，天然免疫 Windup。Position Form 只用在實驗室連線穩定的場景。」

7. **D on Measurement + LPF Filtered Derivative + Derivative Kick** — 從模擬跨到實機的第一座橋。**為什麼這是重點**：模擬完美實機抖動幾乎都是 D 放大量化雜訊；setpoint 階躍時普通 D 會產生 Derivative Kick。帶出：「我的 D 項絕不對誤差微分 — 改對量測值微分避免 setpoint kick；再加一階低通（截止 1/10 控制頻率）抑量化雜訊。這兩招是 sim-to-real 的必做。」

8. **IMC-PID 單參數 λ 調諧是軍規** — 比 Z-N 高一維。**為什麼這是重點**：IMC-PID 從數學根源**內建低通 + 模型逆**，天然對未建模延遲/摩擦免疫，調諧只剩一個 $\lambda$（快慢 vs 魯棒）。帶出：「Z-N 把系統逼臨界振盪邊緣是自殺實驗；IMC-PID 用過程模型逆 + 低通濾波，$\lambda$ 小快、$\lambda$ 大穩 — 工業軍規就這樣。」

9. **CTC + PID 反饋線性化本質** — 分辨「泛泛懂 PID」vs「懂控制理論」。**為什麼這是重點**：CTC 前饋抵消 $C\dot{q} + G$、乘 $M(q)$ 歸一慣量 → N 軸耦合非線性變 N 個獨立線性雙積分器 → PID 只處理線性化殘差。帶出：「PID 在複雜機械臂 work 的根本原因是 CTC 做反饋線性化 — 不是 PID 本身強大，是把問題變成 PID 能處理的形狀。」

10. **分層控制哲學：1 Hz 大腦 / 100 Hz 小腦 / 1 kHz 脊髓，PID 守物理穩定底線** — 具身智能系統觀。**為什麼這是重點**：純端到端 RL 輸出力矩會因推理延遲（20 ms）導致電流振盪燒馬達；工業可接受的混合架構是 RL 出參考軌跡 + PID/CTC 跟蹤。帶出：「我設計的不是一個控制器，是分層解耦防禦體系：1 kHz Luenberger 清雜訊 → CTC 剝 90% 非線性 → PID 處理 10% 線性殘差 → 100 Hz MPC/RL 長視角決策。讓 RL 做擅長的泛化，讓 PID 守物理穩定的絕對底線。」

11. **工業為什麼用 Gain Scheduling 不用 RL** — 分辨學界論文 vs 工業經驗。**為什麼這是重點**：工業追求 Deterministic 穩定，Gain Scheduling 每工作點可 Bode 驗證 PM > 45°；黑箱 RL 在 OOD 狀態可能輸出極端增益，**無法通過 ISO 13849 / ISO 10218 功能安全認證**。帶出：「工業絕大多數關節控制採 Gain Scheduling 依 $M(q)$ 或姿態表動態調增益；不用 RL 不是技術差，是認證過不了 — 黑箱控制器在 OOD 狀態的行為無法被 Bode 或 Lyapunov 證明。」

12. **力控絕不用純力 PID，必墊 Impedance 內環** — 接觸動力學的產業常識。**為什麼這是重點**：剛性接觸的傳遞函數有極高增益極點，純力 PID 會激發 50 Hz 極限環震盪；必須 $F = M\ddot{x} + B\dot{x} + Kx$ 加虛擬阻尼消耗碰撞能量。帶出：「力控我絕不用純力 PID — 必墊 Impedance / Admittance 內環，Franka peg-in-hole 就是這樣做。純力 PID 剛接觸金屬必抖到急停。」

## 延伸閱讀

- **《具身智能算法工程師 面試題》Ch1.1 PID 基礎、Ch6.2 經典控制、Ch7 進階控制** — 高頻面試考點的標準答法整理，anti-windup、串級、IMC-PID 全有
- **Astrom & Murray,《Feedback Systems》Ch10–11** — PID 調參和 anti-windup 的理論基礎，MIT 開放教材，PDF 免費
- **Astrom & Hagglund,《Advanced PID Control》** — IMC-PID、Gain Scheduling、自適應 PID 的工業軍規教材
- **Skogestad,《Multivariable Feedback Control》Ch2, Ch9** — Loop Shaping、H∞、$S/T$ 靈敏度塑形的標準參考
- **ROS 2 `ros2_controllers` 原始碼中的 `PidROS` 類** — 看工業級 PID 怎麼處理 clamping、D 項濾波、參數動態重載
- **論文：Bohn & Atherton,《An analysis of the anti-windup problem》(1995)** — Back-calculation 和 Tracking 方法的經典比較
- **Simulink / MATLAB PID Tuner App + Frequency Response Estimator** — 可視化 Bode 調參，理解 PM、GM、共振峰的物理意義
- **PX4 / Betaflight 飛控源碼** — 看 Attitude + Rate cascade、Gyro LPF、Derivative on Measurement 的工業實作
- **Franka Panda Cartesian Impedance Controller 源碼** — 力控+阻抗控制的標桿實作
- **Khalil,《Nonlinear Systems》Ch14** — Passivity-Based Control 的理論根基
- **Isermann,《Digital Control Systems》** — 嵌入式 PID 的離散化、定點數、Watchdog 全覆蓋
- **論文：Fuzzy-PID 在足式機器人地面適應的應用** — 冰面 / 草地場景 Gain Scheduling 的進階方向
- **ISO 13849 + ISO 10218 文件** — 工業機器人功能安全認證標準，讀過才知道為什麼 RL PID 在工業上市難
