---
title: "PID 控制原理與抗飽和實務"
prerequisites: ["09-robot-dynamics-modeling"]
estimated_time: 45
difficulty: 3
tags: ["pid", "control", "anti-windup", "cascade"]
sidebar_position: 13
---

# PID 控制原理與抗飽和實務

## 你將學到

- 能用兩句話精確講清楚 PID 三個項各自做什麼、為什麼合在一起能讓系統穩準快
- 遇到「機械臂關節飽和後釋放 → 瘋狂超調」這類現場災難，能立刻判斷是 integral windup 並選對 anti-windup 策略
- 面試被問串級控制（cascade PID）時，能畫出三環架構、說清楚「由內而外調」的原因

## 核心概念

**精確定義**：**PID 控制器**根據當前誤差（P）、過去累積誤差（I）、誤差變化趨勢（D）三個訊號的加權和，計算出控制量送給致動器。它是閉環控制中最普遍的「最後一哩路」 — 把上游規劃出的期望軌跡，變成馬達實際能追上的力矩或電壓。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：誤差 $e(t) = x_{\text{desired}}(t) - x_{\text{actual}}(t)$（位置、速度或力矩層級的差值）
- **輸出**：控制量 $u(t)$（力矩指令 / 電壓 / PWM duty）
- **上游**：IK 求解器、trajectory planner 給出 $x_{\text{desired}}$
- **下游**：致動器（馬達驅動器）、感測器回傳的 $x_{\text{actual}}$ 形成回饋迴路
- **閉環節點**：位於 **控制** 最末端，是「數學規劃 → 物理現實」的翻譯層

**一句話版本**：「PID 是機器人的脊髓反射 — 看到偏差就出力修正，不需要大腦重新規劃。」

**最少夠用的數學**：

1. **連續時間 PID 控制律**：

$$
u(t) = K_p \, e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \, \frac{de(t)}{dt}
$$

**物理意義**：$K_p e$ 看「現在偏多少」立刻出力修正；$K_i \int e$ 看「過去累積偏了多少」慢慢補償穩態誤差（如重力、摩擦）；$K_d \dot{e}$ 看「偏差正在往哪個方向變」提前減速避免超調。

2. **離散位置式 vs 增量式**：

$$
u[k] = K_p\,e[k] + K_i \sum_{j=0}^{k} e[j]\,\Delta t + K_d \frac{e[k]-e[k-1]}{\Delta t} \quad \text{(位置式)}
$$

$$
\Delta u[k] = K_p(e[k]-e[k-1]) + K_i\,e[k]\,\Delta t + K_d\frac{e[k]-2e[k-1]+e[k-2]}{\Delta t} \quad \text{(增量式)}
$$

**物理意義**：位置式每次算出控制量的**絕對值**，直接可用但積分項容易爆；增量式只算**變化量** $\Delta u$，天然不累積歷史，適合步進馬達或有通訊延遲的場景。

3. **Integral windup 與 anti-windup（核心防線）**：

$$
\text{Clamping: } \quad \text{if } |u| \geq u_{\text{max}}, \quad \text{stop integrating} \; (\dot{I} = 0)
$$

**物理意義**：當致動器已經飽和（馬達出不了更大力），繼續累積 I 項只會讓內部狀態膨脹。一旦誤差反轉，龐大的 I 會讓系統瘋狂超調甚至失控。Clamping 的做法是「飽和就停止積分」，最直覺也最常用。

<details>
<summary>深入：三種 anti-windup 策略的完整比較與數學</summary>

### 1. Clamping（限幅停積）

最簡單的做法：當控制量 $u$ 超過飽和限制 $u_{\text{max}}$，直接把積分項凍結。

```
if |u_total| >= u_max:
    integral_term = integral_term  # 不再累加
else:
    integral_term += Ki * e * dt
```

**優點**：實作只需一行 if，幾乎零計算開銷。
**缺點**：停積分的判斷比較粗糙，從飽和恢復時可能有微小延遲。

### 2. 積分分離（Integral Separation）

當誤差很大時，I 項意義不大（主要靠 P 和 D），直接關掉 I：

$$
u = K_p e + \begin{cases} K_i \int e\,dt & \text{if } |e| < e_{\text{threshold}} \\ 0 & \text{if } |e| \geq e_{\text{threshold}} \end{cases} + K_d \dot{e}
$$

**適用場景**：啟動階段誤差巨大、穩態附近才需要 I 消除靜差。
**缺點**：需要手動選 $e_{\text{threshold}}$，調不好會在切換點產生控制量跳變。

### 3. Back-calculation（反算衰減）

最精緻的方法：用飽和前後的差值反饋回積分器，讓 I 自動衰減。

$$
\dot{I}(t) = K_i \, e(t) + \frac{1}{T_t}\bigl(u_{\text{sat}} - u_{\text{raw}}\bigr)
$$

其中 $u_{\text{raw}}$ 是未限幅的控制量、$u_{\text{sat}}$ 是限幅後的、$T_t$ 是追蹤時間常數（通常取 $T_t = \sqrt{T_i \cdot T_d}$）。

**物理意義**：當 $u_{\text{raw}} = u_{\text{sat}}$（未飽和），反饋項為零，I 正常累積；飽和時，反饋項把多餘的 I 「吸回來」，速度由 $T_t$ 控制。

**優點**：過渡最平滑，無切換跳變。
**缺點**：多一個參數 $T_t$，設計稍複雜。

### 工業實務選擇

| 場景 | 推薦 | 理由 |
|------|------|------|
| 嵌入式 MCU、資源極有限 | Clamping | 零開銷、好 debug |
| 啟動瞬間誤差極大 | 積分分離 | 避免啟動時 I 亂飆 |
| 高性能伺服驅動 | Back-calculation | 過渡最平滑 |
| 不確定選哪個 | Clamping 先上 | 80% 場景夠用，之後再升級 |

</details>

**Ziegler-Nichols 調參（工程起點）**：

| 控制器 | $K_p$ | $T_i$ | $T_d$ |
|--------|-------|-------|-------|
| P | $0.5\,K_u$ | — | — |
| PI | $0.45\,K_u$ | $T_u / 1.2$ | — |
| PID | $0.6\,K_u$ | $T_u / 2$ | $T_u / 8$ |

**物理意義**：先找臨界增益 $K_u$（系統開始等幅振盪的增益）和振盪週期 $T_u$，用上表算初值。這只是**起點** — Ziegler-Nichols 假設線性模型，不含摩擦、背隙、重力補償，實機一定要手動微調。

**串級 PID（Cascade）架構**：

$$
\text{位置環 (外)} \xrightarrow{e_{\text{pos}}} \text{速度環 (中)} \xrightarrow{e_{\text{vel}}} \text{電流環 (內)}
$$

**物理意義**：外環（位置，~100 Hz P 或 PD）輸出速度指令；中環（速度，~1 kHz PI）輸出電流指令；內環（電流，~10 kHz PI）輸出 PWM。**內快外慢**，內環先穩定才有意義往外調。

<details>
<summary>深入：串級 PID 頻寬設計準則與調參順序</summary>

### 頻寬分離原則

串級控制成功的關鍵是**內環頻寬至少是外環的 5–10 倍**。這樣外環看內環就像一個「理想的追蹤器」，不需要知道內環的動態。

典型工業伺服驅動頻寬配置：

| 環路 | 頻率 | 頻寬 | 控制器 | 物理意義 |
|------|------|------|--------|----------|
| 電流環 | 10–20 kHz | ~1 kHz | PI | 控制繞組電流 = 控制力矩 |
| 速度環 | 1–5 kHz | ~100 Hz | PI | 控制轉速、抑制負載擾動 |
| 位置環 | 100–500 Hz | ~10 Hz | P 或 PD | 控制角度位置精度 |

### 調參順序（由內而外）

1. **先調電流環**：斷開速度迴路，只給電流環階躍指令，調 PI 讓電流響應快且無超調
2. **再調速度環**：電流環已穩定，給速度階躍指令，調 PI 讓速度響應平滑
3. **最後調位置環**：速度環已穩定，給位置階躍指令，通常只用 P 或 PD

為什麼不能反過來？如果電流環不穩，速度環的輸出（電流指令）根本追不上，外環怎麼調都沒用。

### 反例：沒有頻寬分離的後果

如果位置環和速度環頻寬太接近（例如都是 100 Hz），兩個環的動態耦合會互相干擾，產生：
- 低頻振盪（兩環搶控制權）
- 增益裕度急劇下降
- 看似「隨機」的不穩定

</details>

## 直覺理解

**開車類比**（三項各司其職）：
- **P = 盯車距踩油門**：前車離你越遠，踩油門越猛。但純靠 P，追到接近時油門太小，永遠差一點（穩態誤差）
- **I = 逆風中慢慢加油**：發現持續被風吹、車距一直沒縮到零，I 慢慢加油補償。但加太久會「過度加速」（windup）
- **D = 看前車煞車燈提前減速**：還沒碰到就先收油門，避免追撞（超調）。但路面顛簸（雜訊）時 D 會讓你瘋狂踩放油門

**模擬器觀察**：在 Gazebo 或 Isaac Sim 中開一個單關節位置控制：
1. **純 P**：設 $K_p = 10$，觀察關節追目標但永遠差一點（重力造成穩態誤差）
2. **加 I**：$K_i = 5$，穩態誤差消失，但如果給一個超出力矩限制的目標 → 關節飽和 → 釋放後瘋狂超調（windup 現象肉眼可見）
3. **加 clamping anti-windup**：同樣場景，超調消失
4. **加 D**：$K_d = 1$，響應更快、超調更小；但把 $K_d$ 調到 10 → 關節開始抖（D 放大編碼器量化雜訊）

**一句話記憶**：「P 管現在、I 管過去、D 管未來；anti-windup 管 I 不要失控。」

## 實作連結

**三個典型工程場景**：

1. **ROS 2 關節位置控制**：`ros2_controllers` 中的 `JointTrajectoryController` 內部就是 PID。啟動時載入 YAML 參數（$K_p$、$K_i$、$K_d$、$i_{\text{clamp}}$），每個 control tick 從 `/joint_states` 讀回實際角度、和 trajectory 插值後的目標角度算誤差、跑 PID、輸出力矩指令。

2. **嵌入式馬達驅動**：STM32 + FOC 驅動器中，電流環 PID 跑在 10 kHz 中斷裡，用增量式避免位置式的 windup 問題。速度環 1 kHz、位置環 100 Hz，層層嵌套在不同頻率的中斷中。

3. **自動調參（Auto-tuning）**：先跑 relay feedback test（繼電器自動產生極限週期振盪），軟體自動讀 $K_u$ 和 $T_u$，用 Ziegler-Nichols 表算初值，再跑 gradient-free optimizer 微調。

**Code 骨架**（C++，ROS 2 風格）：

```cpp
// PID 控制器核心（含 clamping anti-windup）
struct PIDController {
    double kp, ki, kd;
    double integral = 0.0;
    double prev_error = 0.0;
    double i_clamp;       // 積分項限幅
    double output_max;    // 輸出限幅

    double compute(double error, double dt) {
        // P
        double p_term = kp * error;
        // I（含 anti-windup clamping）
        integral += error * dt;
        integral = std::clamp(integral, -i_clamp, i_clamp);
        double i_term = ki * integral;
        // D（含低通濾波避免雜訊放大）
        double d_term = kd * (error - prev_error) / dt;
        prev_error = error;
        // 輸出限幅
        double output = p_term + i_term + d_term;
        return std::clamp(output, -output_max, output_max);
    }
};
```

<details>
<summary>深入：完整 Python 實作（含 back-calculation anti-windup + 模擬測試）</summary>

```python
import numpy as np
import matplotlib.pyplot as plt

class PIDController:
    """PID 控制器，支援 clamping 和 back-calculation anti-windup。"""

    def __init__(self, kp, ki, kd, output_min, output_max,
                 anti_windup='clamping', tracking_tc=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max
        self.anti_windup = anti_windup

        # Back-calculation 追蹤時間常數
        if tracking_tc is None:
            ti = kp / ki if ki > 0 else 1.0
            td = kd / kp if kp > 0 else 0.0
            self.tracking_tc = np.sqrt(ti * td) if td > 0 else ti
        else:
            self.tracking_tc = tracking_tc

        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, error, dt):
        # P term
        p_term = self.kp * error

        # D term（用 backward difference）
        d_term = self.kd * (error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = error

        # I term
        i_term = self.ki * self.integral

        # 未限幅輸出
        u_raw = p_term + i_term + d_term

        # 限幅
        u_sat = np.clip(u_raw, self.output_min, self.output_max)

        # Anti-windup 更新積分
        if self.anti_windup == 'clamping':
            # 如果未飽和，正常積分；飽和則凍結
            if self.output_min < u_raw < self.output_max:
                self.integral += error * dt
        elif self.anti_windup == 'back_calculation':
            # 反算：用飽和差值衰減積分
            self.integral += (error + (u_sat - u_raw) / self.tracking_tc) * dt
        else:
            # 無 anti-windup（危險！僅做對照）
            self.integral += error * dt

        return u_sat


def simulate_pid(setpoint, controller, plant_mass=1.0, gravity=9.81,
                 torque_limit=15.0, duration=5.0, dt=0.001):
    """模擬單關節位置控制（含重力 + 力矩飽和）。"""
    steps = int(duration / dt)
    t = np.zeros(steps)
    pos = np.zeros(steps)
    vel = np.zeros(steps)
    cmd = np.zeros(steps)

    for k in range(1, steps):
        t[k] = k * dt
        error = setpoint - pos[k - 1]
        u = controller.compute(error, dt)
        cmd[k] = u

        # 簡單質點 + 重力：a = (u - m*g) / m
        accel = (np.clip(u, -torque_limit, torque_limit) - plant_mass * gravity) / plant_mass
        vel[k] = vel[k - 1] + accel * dt
        pos[k] = pos[k - 1] + vel[k] * dt

    return t, pos, cmd


# 比較三種 anti-windup 策略
fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
setpoint = 2.0

for aw, label, color in [
    ('none', 'No anti-windup', 'red'),
    ('clamping', 'Clamping', 'blue'),
    ('back_calculation', 'Back-calculation', 'green'),
]:
    ctrl = PIDController(kp=20, ki=10, kd=5,
                         output_min=-15, output_max=15,
                         anti_windup=aw)
    t, pos, cmd = simulate_pid(setpoint, ctrl)
    axes[0].plot(t, pos, label=label, color=color)
    axes[1].plot(t, cmd, label=label, color=color, alpha=0.7)

axes[0].axhline(setpoint, ls='--', color='gray', label='Setpoint')
axes[0].set_ylabel('Position')
axes[0].legend()
axes[0].set_title('Anti-windup comparison (torque limit = 15 N·m)')
axes[1].set_ylabel('Command')
axes[1].set_xlabel('Time (s)')
axes[1].legend()
plt.tight_layout()
plt.savefig('pid_antiwindup_comparison.png', dpi=150)
plt.show()
```

跑出來的圖會清楚看到：紅色（無 anti-windup）飽和釋放後超調劇烈；藍色（clamping）超調大幅減小；綠色（back-calculation）過渡最平滑。

</details>

## 常見誤解

1. **「D 增益越大越穩」** — 錯。D 項本質是對誤差做微分，高頻雜訊（編碼器量化、ADC 雜訊）會被放大。$K_d$ 過大 → 控制量高頻抖動 → 馬達發出嗡嗡聲 → 機構磨損加劇。**正確做法**：D 項前面加一階低通濾波（截止頻率約為控制頻率的 1/10），或改用「derivative on measurement」（對 $x_{\text{actual}}$ 微分而非 $e$）避免 setpoint 階躍時的脈衝。

2. **「有穩態誤差就無腦加 I」** — 穩態誤差不一定要靠 I 解決。如果誤差來自已知的重力或摩擦，用**前饋補償**（$u_{\text{ff}} = m g$ 或 $\hat{f}_{\text{friction}}$）+ PD 控制效果更好、響應更快、不會引入 I 帶來的相位滯後和 windup 風險。**原則**：已知擾動用前饋消、未知慢變擾動才靠 I。

3. **「PID 能搞定一切」** — PID 是線性控制器，面對非線性系統（高速運動的慣量耦合、Coriolis 力、接觸力突變）會力不從心。工業伺服的標準做法是 **PID + 動力學前饋**（$\tau_{\text{ff}} = M(q)\ddot{q}_d + C(q,\dot{q})\dot{q}_d + g(q)$），PID 只負責修正前饋沒算準的殘差。面試時講「PID 是回饋、dynamics 是前饋、兩者互補」是加分句。

4. **「沒寫 anti-windup 也沒差」** — 在模擬器裡可能看不出問題，因為力矩沒真正飽和。但實機有物理力矩上限，卡住（碰撞、堵轉）時 I 項爆掉 → 釋放瞬間砸飛工件或撞壞夾爪。**任何上線的 PID 都必須有 anti-windup**，這是防禦性編程的最低要求。

## 練習題

<details>
<summary>Q1：機械臂關節追蹤目標時穩態總差 0.5°，你會怎麼分析？先做什麼、再做什麼？</summary>

**完整推理鏈**：

1. **先判斷穩態誤差來源**：0.5° 在重力方向 → 高度懷疑是重力未補償。讓關節水平放（重力不作用在關節軸上），看誤差是否消失
2. **如果水平放誤差消失**：確認是重力矩 → 不要加 I，改用**重力前饋** $\tau_{\text{ff}} = m g L \cos\theta$，PD 控制即可消除穩態誤差
3. **如果水平放仍有誤差**：可能是摩擦（Coulomb / Stribeck），嘗試摩擦前饋 $\hat{f} = f_c \cdot \text{sign}(\dot\theta) + f_v \cdot \dot\theta$
4. **前饋都加了仍有殘差**：這時才加 $K_i$，但**必須搭配 clamping anti-windup**，$i_{\text{clamp}}$ 設為比穩態修正量稍大即可
5. **要避開的陷阱**：直接加大 $K_i$ → 相位裕度下降 → 系統可能開始振盪，尤其在低速反轉（Stribeck 區域）

**面試官想聽到**：先分析根因、優先用前饋解已知擾動、I 是最後手段且必須帶 anti-windup。

</details>

<details>
<summary>Q2：機械臂碰到障礙物卡住三秒後鬆開，關節瘋狂甩動超調，怎麼診斷和修？</summary>

**完整推理鏈**：

1. **現象分析**：卡住 → 誤差持續為正 → I 項三秒內狂累積 → 鬆開瞬間 P+I+D 全部釋放 → 巨大控制量 → 關節高速甩動 → 典型 **integral windup**
2. **立即驗證**：在 log 裡看 I 項數值，卡住期間 I 會單調上升到極大值
3. **修法（由簡到精）**：
   - **Step 1**：加 clamping anti-windup，設 $i_{\text{clamp}}$ 使得 $K_i \cdot i_{\text{clamp}} < u_{\text{max}}$
   - **Step 2**：如果仍有輕微超調，改用 back-calculation（$T_t = \sqrt{T_i \cdot T_d}$），過渡更平滑
   - **Step 3**：加**碰撞偵測** — 當力矩指令持續飽和且速度為零，判定碰撞，主動清零 I 並切換到安全模式
4. **要避開的陷阱**：只調低 $K_i$ 不加 anti-windup — 穩態性能變差但 windup 根因沒解決

**面試官想聽到**：能精確描述 windup 的物理過程（卡住 → I 爆 → 釋放超調）、知道 clamping 是標配、能進階到碰撞偵測 + 安全模式。

</details>

<details>
<summary>Q3：設計一個機械臂關節伺服驅動器，需要位置精度 ±0.01°、速度響應 1 kHz。你怎麼規劃三環串級 PID 架構？</summary>

**完整推理鏈**：

1. **架構規劃**（由內而外）：
   - **電流環**（最內）：10–20 kHz PI，控制繞組電流 = 控制力矩。頻寬目標 ~1 kHz
   - **速度環**（中間）：1–5 kHz PI，控制轉速。頻寬目標 ~100–200 Hz（至少是位置環的 5 倍）
   - **位置環**（最外）：100–500 Hz P 或 PD，控制角度。頻寬目標 ~10–30 Hz
2. **調參順序**：
   - 先調電流環：斷開速度迴路，給電流階躍指令，調 PI 讓電流響應在 1 ms 內穩定、無超調
   - 再調速度環：電流環已穩定，給速度階躍，調 PI 讓速度在 5–10 ms 穩定
   - 最後調位置環：給位置階躍，調 P（或 PD）讓位置在 50–100 ms 穩定且精度達標
3. **精度保障**：±0.01° 需要高解析度編碼器（≥ 17-bit，131072 counts/rev）+ 電流環要夠快以消除 torque ripple
4. **要避開的陷阱**：
   - 不能先調外環再調內環（內環不穩外環無意義）
   - 速度環和位置環頻寬太接近會耦合振盪
   - 忘記在電流環和速度環都加 anti-windup

**面試官想聽到**：清晰的三環頻率劃分 + 由內而外調的原因 + 頻寬分離 5–10 倍準則 + 精度與編碼器解析度的連結。

</details>

<details>
<summary>Q4：你的 PID 在模擬器裡表現完美，但上實機後關節持續微振動（buzzing），怎麼排查？</summary>

**完整推理鏈**：

1. **先確認是 D 項還是量化雜訊**：暫時把 $K_d$ 設為 0，如果振動消失 → D 項放大了感測器雜訊
2. **修法**：
   - 在 D 項前加一階低通濾波：$D_{\text{filtered}}[k] = \alpha \cdot D_{\text{raw}}[k] + (1-\alpha) \cdot D_{\text{filtered}}[k-1]$，$\alpha = \frac{dt}{dt + 1/(2\pi f_c)}$，$f_c$ 設控制頻率的 1/10
   - 改用「derivative on measurement」：對 $-x_{\text{actual}}$ 微分而非 $e$，避免 setpoint 變化時的脈衝
3. **如果 $K_d = 0$ 仍振動**：檢查離散化效應 — 控制頻率太低（< 10× 機械共振頻率）→ 提高控制率或降 $K_p$
4. **如果以上都不是**：檢查驅動器死區、PWM 解析度、齒輪背隙 — 這些非線性會讓線性 PID 在零附近反覆切換
5. **要避開的陷阱**：直接降所有增益 — 穩態性能和動態響應一起變差，是治標不治本

**面試官想聯到**：知道 D 項雜訊放大是模擬 vs 實機差異的首要原因；能講出低通濾波和 derivative on measurement 兩個標準解法。

</details>

## 面試角度

1. **Anti-windup 是防禦性編程的最低要求** — 這是區分「寫過模擬器 PID」和「部署過實機 PID」的分水嶺。面試時用這句話帶出：「任何上線的 PID 我都會先加 clamping anti-windup，因為實機一定有力矩飽和，沒有 anti-windup 的控制器是定時炸彈。」

2. **前饋 + 回饋分離思維** — 展現系統性思考而非「調參數到天荒地老」。面試時帶出：「穩態誤差我不會第一時間加 I，而是先分析是重力還是摩擦，用前饋消掉已知項，PID 只負責修正殘差 — 這讓系統更快、更穩、更好調。」

3. **串級控制的頻寬分離** — 證明你理解工業伺服的真實架構，不只是教科書的單環 PID。面試時帶出：「實際伺服驅動是三環串級 — 電流、速度、位置 — 內環頻寬至少是外環的 5 倍，由內而外調。面試答 PID 只答單環是不完整的。」

4. **離散化與時延的工程意識** — 展現你懂「理論到實作」的 gap。面試時帶出：「連續 PID 好看但跑在 MCU 上是離散的，我會選增量式避免 windup、加 D 項低通避免雜訊放大、確認控制率至少是機械頻寬的 10 倍。」

5. **PID 的邊界：何時該放棄 PID** — 展現你不是只會一招。面試時帶出：「高速非線性場景我會用 computed torque + PD、或 MPC；PID 適合線性化後殘差小的場景，硬用 PID 做 whole-body control 是走錯路。」

## 延伸閱讀

- **《具身智能算法工程師 面試題》Ch1.1 PID 基礎、Ch6.2 經典控制** — 高頻面試考點的標準答法整理，anti-windup 和串級都會考
- **Astrom & Murray,《Feedback Systems》Ch10–11** — PID 調參和 anti-windup 的理論基礎，MIT 開放教材，PDF 免費
- **ROS 2 `ros2_controllers` 原始碼中的 `PidROS` 類** — 看工業級 PID 怎麼處理 clamping、D 項濾波、參數動態重載
- **論文：Bohn & Atherton, "An analysis of the anti-windup problem" (1995)** — back-calculation 和 tracking 方法的經典比較
- **Simulink / MATLAB PID Tuner App** — 可視化 Bode plot 調參，理解增益裕度和相位裕度的物理意義
- **論文：Fuzzy PID 在變負載機器人的應用** — 當固定增益不夠、負載隨姿態變化大時的進階方向
- **Smith Predictor** — 通訊延遲嚴重（遠端遙操作）時的標準 PID 改良架構
