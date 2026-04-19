---
title: "力控制與阻抗／導納控制"
prerequisites: ["13-pid-control-tuning"]
estimated_time: 45
difficulty: 4
tags: ["force-control", "impedance", "admittance", "hybrid", "compliance"]
sidebar_position: 14
---

# 力控制與阻抗／導納控制

## 你將學到

- 能判斷什麼場景需要力控而非位置控制，並用兩三句話在面試中講出 impedance 與 admittance 的因果差異
- 遇到「機械臂研磨抖動」「協作手引示教有殘留力」這類工程情境，知道先檢查力位混合的 Selection Matrix 與動態補償
- 掌握 impedance / admittance / hybrid position-force 三種架構的適用硬體條件與選型邏輯

## 核心概念

**精確定義**：**力控制（Force Control）** 是讓機器人主動調節與環境接觸力的控制策略，與位置控制互補 — 位置控制追蹤幾何軌跡（剛性任務），力控制調節接觸力（柔順任務）。裝配、研磨、手引示教等需要「碰到東西不硬撞」的場景，核心都是力控。

**阻抗控制（Impedance Control）**：讓末端表現得像一個虛擬的 **彈簧-阻尼-質量** 系統。因果方向是 **位移偏差 → 輸出力**（感測位移，控制力）。適合直驅馬達或柔性關節等能直接輸出力矩的硬體。

**導納控制（Admittance Control）**：因果方向相反 — **外力輸入 → 輸出位移修正**（感測力，控制位移）。適合剛性高減速比的大慣量工業臂，因為這類硬體位置控制精度高、但直接做力控困難。

**力位混合控制（Hybrid Position/Force）**：用 Selection Matrix $S$ 把任務空間分解成不同方向 — 法向做力控、切向做位控，讓同一個控制迴圈在不同自由度上執行不同策略。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：末端力/力矩（來自 F/T sensor 或關節力矩感測器或電流推算）、末端位姿（來自 FK + encoder）、期望力 $F_d$ 與期望位姿 $x_d$（來自規劃器）
- **輸出**：關節力矩指令 $\tau$（impedance）或關節位置修正 $\Delta q$（admittance），送入底層關節控制器
- **下游**：依賴 Ch09 動力學模型做重力/慣性補償、Ch13 PID 做底層關節跟蹤；輸出的柔順行為直接影響裝配成功率、研磨品質、人機安全
- **閉環節點**：位於 **控制** 階段的最外層迴圈，替代或疊加在位置控制之上，賦予機器人「觸覺反射」能力

**一句話版本**：「力控是賦予機器人觸覺反射的神經樞紐 — 讓它碰到東西時像人手一樣柔順，而不是像鑽頭一樣硬撞。」

**最少夠用的數學**：

1. **阻抗控制方程**（末端表現為虛擬質量-阻尼-彈簧）：

$$
F = M_d \ddot{e} + D_d \dot{e} + K_d e, \quad e = x - x_d
$$

**物理意義**：$M_d$（期望慣量）決定加速反應快慢、$D_d$（期望阻尼）決定能量耗散速度、$K_d$（期望剛度）決定偏離期望位置時的回復力大小。三者可獨立調整，讓機器人末端的接觸行為像你設計的彈簧系統。

2. **導納控制方程**（因果反轉：力 → 位移修正）：

$$
M_d \ddot{x}_{\text{cmd}} + D_d \dot{x}_{\text{cmd}} + K_d x_{\text{cmd}} = F_{\text{ext}}
$$

**物理意義**：感測到外力 $F_{\text{ext}}$ 後，解這個 ODE 得到位置修正量 $x_{\text{cmd}}$，再送給底層位置控制器。外力越大 → 修正越遠；$K_d$ 設為 0 時變成純阻尼模式（手引示教零力感）。

3. **力位混合 — Selection Matrix**：

$$
\tau = J^T \left[ S \cdot F_{\text{force\_ctrl}} + (I - S) \cdot F_{\text{pos\_ctrl}} \right]
$$

**物理意義**：$S$ 是對角矩陣，每個對角元素 0 或 1，決定該自由度做力控還是位控。例如研磨：法向 $S_{zz}=1$（力控維持恆定壓力）、切向 $S_{xx}=S_{yy}=0$（位控走軌跡）。

4. **關節力矩與末端力的映射**：

$$
\tau = J^T F
$$

**物理意義**：Jacobian 轉置把末端六維力/力矩映射回每個關節需出的力矩，是力控從任務空間轉回關節空間的核心橋梁。

<details>
<summary>深入：阻抗控制的完整動力學推導與穩定性分析</summary>

**從任務空間動力學出發**：

機械臂在任務空間的動力學方程（忽略摩擦）：

$$
\Lambda(x) \ddot{x} + \mu(x, \dot{x}) \dot{x} + p(x) = F + F_{\text{ext}}
$$

其中 $\Lambda = (J M^{-1} J^T)^{-1}$ 是任務空間慣量矩陣，$\mu$ 是 Coriolis/centrifugal 項，$p$ 是重力項，$F$ 是控制力，$F_{\text{ext}}$ 是環境接觸力。

**阻抗控制律推導**：

目標：讓末端在接觸時表現為 $M_d \ddot{e} + D_d \dot{e} + K_d e = -F_{\text{ext}}$。

設計控制力：

$$
F = \Lambda(x) \ddot{x}_d + \mu(x, \dot{x}) \dot{x} + p(x) - \Lambda(x) M_d^{-1} (D_d \dot{e} + K_d e)
$$

前三項做**動力學前饋**（補償慣性、Coriolis、重力），最後一項注入期望的阻抗行為。

轉回關節空間：$\tau = J^T F + (I - J^T J^{-T}) \tau_0$

其中 $(I - J^T J^{-T}) \tau_0$ 是冗餘機械臂的 null-space 項。

**穩定性條件（Passivity 觀點）**：

系統存儲函數：

$$
V = \frac{1}{2} \dot{e}^T M_d \dot{e} + \frac{1}{2} e^T K_d e
$$

$\dot{V} = -\dot{e}^T D_d \dot{e} \leq 0$（前提：$D_d$ 正定）

因此 $D_d > 0$ 是穩定的**必要條件**。$K_d > 0$ 保證平衡點唯一。

**實務要點**：
- $M_d$ 設太小（反應太快）→ 控制力超過執行器上限 → 飽和 → 失穩
- $K_d / D_d$ 比值過大（欠阻尼）→ 接觸時振盪
- 完美的動力學前饋依賴精確的動力學模型；模型誤差需靠魯棒項或自適應律補償

</details>

**力感測方案比較**（選型直接影響控制頻寬和精度）：

| 感測方式 | 頻寬 | 優勢 | 劣勢 |
|----------|------|------|------|
| 電流推算 | 高（kHz 級） | 無額外硬體、頻寬高 | 摩擦/背隙干擾大，精度差 |
| 關節力矩感測器 | 中-高 | 全身柔順、碰撞偵測 | 成本高、需每關節裝 |
| 末端 F/T sensor | 中 | 六維末端力完整量測 | 需重力補償、安裝增加末端慣量 |

## 直覺理解

**阻抗 = 拉著彈力繩握方向盤**：你握著方向盤（期望位置 $x_d$），手臂和方向盤之間有一條彈力繩（$K_d$）加一個阻尼器（$D_d$）。路面顛簸（外力）讓方向盤偏離，你的手不是鎖死（純位控），而是允許一定偏移再柔柔拉回。$K_d$ 越大 = 繩越緊 = 偏離越小但越容易震；$D_d$ 越大 = 阻尼越重 = 回來越慢但越穩。

**導納 = 推超市推車**：你推推車（施加外力 $F_{\text{ext}}$），推車根據你推的力決定移動多快、移動多遠。推越大力 → 走越遠。導納控制器就像推車的質量和摩擦力，決定「這股力要換算成多少位移」。工業臂硬體剛性高（像重型推車），不適合直接做力控，但很適合「你告訴我力有多大，我算出該移多少」的導納模式。

**力位混合 = 用橡皮擦擦黑板**：手的法向（壓向黑板的方向）做力控 — 維持恆定壓力讓橡皮擦貼合；切向（左右滑動的方向）做位控 — 精確走擦拭軌跡。Selection Matrix 就是在說「這個方向聽力的、那個方向聽位置的」。

**模擬器觀察**：在 MuJoCo 或 Isaac Sim 中，設定一個 UR5 + F/T sensor 推一個彈簧牆面。先跑純位置控制 — 碰到牆面力會瞬間飆高甚至彈飛。再切成阻抗控制（$K_d = 500, D_d = 50$）— 碰到牆面會柔順壓入一段距離，力平滑收斂到期望值。把 $K_d$ 調到 5000 → 行為接近位控（硬撞）；$K_d = 0, D_d$ 很小 → 手引示教模式（輕推就動）。

## 實作連結

**三個典型工程場景**：

1. **協作機器人手引示教（Lead-through Teaching）**：操作員直接用手推機械臂，機器人跟著走。核心是導納控制 + $K_d = 0$（無位置回復力），只保留小阻尼 $D_d$ 讓手感平滑。關鍵前提：精確的重力補償 — 否則機器人自重會讓操作員覺得「推不動」或「往下掉」。

2. **表面研磨/拋光（Surface Finishing）**：力位混合控制 — 法向用 PI 力控維持 10 N 恆定壓力、切向用位控走研磨軌跡。需要 F/T sensor 動態補償（扣除工具重力和慣性力），以及自適應剛度（遇到曲率變化時調整 $K_d$）。

3. **精密裝配（Peg-in-Hole）**：位控接近 → 力控插入。需要狀態機管理切換：接近階段（純位控）→ 初始接觸偵測（力閾值觸發）→ 搜尋孔位（螺旋搜尋 + compliance）→ 插入（力控 + 阻尼）。切換瞬間最容易出衝擊力。

**Code 骨架**（C++，ROS 2 + ros2_control 架構）：

```cpp
// Impedance controller — 在 ros2_control 的 update() 迴圈中
// 輸入：current_pose, desired_pose, measured_wrench
// 輸出：joint_torques

Eigen::Vector6d pose_error = compute_pose_error(current_pose, desired_pose);
Eigen::Vector6d vel_error  = current_velocity - desired_velocity;

// 阻抗力 = K * e + D * ė（忽略 Md 項做一階近似）
Eigen::Vector6d F_impedance = Kd * pose_error + Dd * vel_error;

// 加上力跟蹤誤差（若有期望力 Fd）
Eigen::Vector6d F_total = F_impedance + F_feedforward;

// 轉回關節空間
Eigen::VectorXd tau = jacobian.transpose() * F_total;

// 加動力學補償（重力 + Coriolis）
tau += gravity_compensation + coriolis_compensation;

hardware_interface->set_command(tau);
```

<details>
<summary>深入：完整 Python 導納控制實作（MuJoCo 環境）</summary>

```python
import mujoco
import numpy as np

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
        # 狀態：位移修正量和速度
        self.x_cmd = np.zeros(6)
        self.x_cmd_dot = np.zeros(6)

    def update(self, F_ext: np.ndarray) -> np.ndarray:
        """
        輸入：六維外力 F_ext（需已扣除工具重力）
        輸出：六維位移修正 x_cmd
        """
        # 解 ODE: Md * x_cmd_ddot + Dd * x_cmd_dot + Kd * x_cmd = F_ext
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


def gravity_compensate_ft(raw_wrench, tool_mass, tool_com, R_sensor):
    """
    F/T sensor 重力補償：扣除工具自重在 sensor frame 的投影
    raw_wrench: [fx, fy, fz, tx, ty, tz] sensor frame
    tool_mass: 工具質量 (kg)
    tool_com: 工具質心在 sensor frame 的位置 [3,]
    R_sensor: sensor frame 相對於 world frame 的旋轉矩陣
    """
    g_world = np.array([0, 0, -9.81])
    g_sensor = R_sensor.T @ g_world
    F_gravity = tool_mass * g_sensor

    # 力矩 = r × F
    T_gravity = np.cross(tool_com, F_gravity)

    compensated = raw_wrench.copy()
    compensated[:3] -= F_gravity
    compensated[3:] -= T_gravity
    return compensated


# === 使用範例 ===
# 手引示教模式：Kd=0（無回復力），小阻尼
controller = AdmittanceController(
    Md=[1.0]*6,       # 小慣量 → 反應快
    Dd=[10.0]*6,      # 小阻尼 → 輕推就動
    Kd=[0.0]*6,       # 零剛度 → 手引模式
    dt=0.001
)

# 研磨模式：法向有剛度（維持壓力），切向零剛度
controller_grinding = AdmittanceController(
    Md=[1.0]*6,
    Dd=[50.0, 50.0, 80.0, 10.0, 10.0, 10.0],
    Kd=[0.0, 0.0, 500.0, 0.0, 0.0, 0.0],  # 只有 z 向有剛度
    dt=0.001
)
```

</details>

<details>
<summary>深入：阻抗 vs 導納的硬體選型決策樹</summary>

```
硬體特性判斷入口
│
├─ 馬達類型是直驅 / 低減速比（< 10:1）？
│   ├─ 是 → 有良好的反向驅動性（back-drivability）
│   │        → 首選 **Impedance Control**
│   │        → 理由：能直接輸出精確力矩，不需力感測器也能做基本柔順
│   │        → 代表：Franka Emika Panda、MIT Cheetah 腿
│   │
│   └─ 否 → 高減速比（> 50:1）諧波減速器 / 行星齒輪
│            → 反向驅動性差，力矩輸出受摩擦/背隙嚴重影響
│            → 首選 **Admittance Control**
│            → 理由：利用高精度位置控制能力，由力感測器量測外力再算位移修正
│            → 代表：UR 系列、KUKA iiwa（iiwa 有關節力矩感測器，兩種都能用）
│
├─ 有末端 F/T sensor？
│   ├─ 是 → 導納控制可直接用
│   │        阻抗控制也能用（做力跟蹤外迴圈）
│   │
│   └─ 否 → 有關節力矩感測器？
│            ├─ 是 → 可用外部力估測器（Generalized Momentum Observer）推算末端力
│            └─ 否 → 只能用電流推算，精度受限，建議加裝感測器
│
└─ 任務是否需要力位混合（同時控制某些方向的力和另一些方向的位置）？
    ├─ 是 → Hybrid Position/Force（Selection Matrix 分解方向）
    │        底層可搭配 impedance 或 admittance
    └─ 否 → 純柔順任務 → 單一模式即可
```

**經驗法則**：
- 協作機器人（Franka, KUKA iiwa）：阻抗控制為主，關節力矩感測器提供全身柔順
- 傳統工業臂（UR, ABB, Fanuc）：導納控制為主，搭配末端 F/T sensor
- 腿式機器人：阻抗控制（需要快速力矩響應來做地面反力控制）
- 靈巧手：阻抗控制（手指直驅馬達 + 觸覺感測器）

</details>

## 常見誤解

1. **「精密裝配光靠位置控制就夠」** — 零件有公差（通常 ±0.05 mm），孔位和銷的相對位置永遠有微小偏差。純位控會硬撞 → 卡死或損壞零件。**正確理解**：裝配需要 compliance — 至少在接觸後切入力控或阻抗控制，讓機器人「順著推」找到孔位。業界標準做法是位控接近 + 阻抗控制插入。

2. **「阻抗控制和導納控制一樣，只是名字不同」** — 因果方向完全相反。Impedance：位移偏差 → 力（像彈簧，你推它、它反推你）。Admittance：外力 → 位移修正（像推車，你推它、它移動）。選錯會導致系統不穩定 — 阻抗控制用在高減速比臂上，摩擦會吃掉力矩控制精度；導納控制用在直驅臂上，浪費了直接力矩控制的優勢。

3. **「F/T sensor 讀出來就是接觸力」** — 原始讀數包含工具重力（隨姿態變化）和加速度引起的慣性力。不做重力補償就用 → 機器人靜止時就「感覺」到假的接觸力 → 控制器亂動。**正確做法**：每次開機先做 F/T sensor 校準（多姿態採樣 + 最小二乘辨識工具質量和質心），運行時即時扣除 $F_{\text{gravity}} = m_{\text{tool}} \cdot R^T g$。

4. **「把阻抗控制的剛度 $K_d$ 調很高就能像位控一樣精準」** — 理論上 $K_d \to \infty$ 確實趨近位控。但實際上 $K_d$ 太高 → 系統變成欠阻尼 → 接觸時高頻震盪 → 嚴重時損壞零件或傷人。**正確做法**：$K_d$ 和 $D_d$ 要一起調，保持臨界阻尼比 $\zeta = D_d / (2\sqrt{K_d M_d}) \approx 0.7 \sim 1.0$。

## 練習題

<details>
<summary>Q1（中）：機械臂做表面研磨，要求法向維持恆定 10 N 壓力、切向走預定軌跡。但實際力波動 ±5 N 且表面有刮痕。你會怎麼分析？</summary>

**完整推理鏈**：

1. **架構確認**：這是典型力位混合控制場景 — Selection Matrix 法向 $S_{zz}=1$（力控）、切向 $S_{xx}=S_{yy}=0$（位控）
2. **力波動 ±5 N 診斷**：
   - 先看 F/T sensor 原始訊號 → 是否做了重力補償？工具換姿態時重力分量會變
   - 力控迴圈的 PI 參數是否合理？$K_p$ 太大 → 震盪、$K_i$ 太小 → 穩態誤差
   - 工件表面曲率是否均勻？曲率變化 → 接觸幾何變 → 需要自適應剛度
3. **刮痕診斷**：
   - 力波動高頻分量 → 切向軌跡跟蹤出現頓挫 → 研磨頭不均勻接觸
   - 檢查力控和位控的頻寬是否匹配 — 力控迴圈太慢跟不上位控的軌跡速度
4. **解法**：
   - F/T sensor 加低通濾波（截止頻率 30–50 Hz，高於力控頻寬但濾掉機構共振）
   - 力控用 PI 控制：$F_{\text{cmd}} = K_p (F_d - F) + K_i \int (F_d - F) dt$
   - 加前饋：已知工件 CAD 模型的曲率 → 前饋法向位移修正
   - 自適應剛度：曲率大的地方降低 $K_d$，讓機器人更柔順地貼合表面

**面試官想聽到**：力位混合架構 + F/T 動態補償 + PI 力控調參邏輯 + 自適應剛度的概念。

</details>

<details>
<summary>Q2（中）：協作機器人做手引示教（lead-through teaching），操作員反映「推不動」或「放手後機器人自己飄」。怎麼分析？</summary>

**完整推理鏈**：

1. **「推不動」診斷**：
   - 導納控制器的 $D_d$ 太大或 $M_d$ 太大 → 虛擬阻力大 → 操作員需要大力才推得動
   - 重力補償不精確 → 機器人自重殘留力 → 某些姿態特別重
   - 摩擦補償不足 → 靜摩擦力閾值沒被補償 → 需要先克服靜摩擦才開始動
2. **「放手後飄」診斷**：
   - $D_d$ 太小 → 虛擬阻尼不夠 → 放手後機器人靠慣性繼續滑
   - $K_d$ 應為 0（手引模式不需要回到原位），但 $D_d$ 要足夠大到讓機器人放手後 0.3–0.5 秒內停下
   - 重力補償有偏差 → 某些姿態自重沒補乾淨 → 放手後往下掉
3. **系統性解法**：
   - 精確辨識每個關節的摩擦模型（Coulomb + viscous），並在控制器中補償
   - 精確辨識連桿質量/質心/慣量（用 payload identification 程序）
   - $K_d = 0$，$D_d$ 調到操作員手感舒適（通常 5–20 Ns/m），$M_d$ 調到 0.5–2.0 kg
   - 加死區（dead band）：力量 < 2 N 時不動，防止噪聲導致飄移

**面試官想聯到**：導納控制參數調整 + 重力/摩擦補償是手引示教品質的關鍵；不是控制架構的問題，是補償精度的問題。

</details>

<details>
<summary>Q3（難）：位控移動中，機器人從自由空間突然碰到工件，產生巨大衝擊力。怎麼設計控制策略平滑過渡？</summary>

**完整推理鏈**：

1. **問題根因**：純位控在接觸瞬間，位置誤差突然無法消除（被工件擋住），高增益位控器會輸出巨大力矩試圖「撞過去」
2. **狀態機設計**（三階段平滑過渡）：
   - **Phase 1 — 接近（Approach）**：純位控，但逐步降低阻抗剛度 $K_d$（例如從 2000 降到 200 N/m）和增加阻尼 $D_d$，為接觸做準備
   - **Phase 2 — 初始接觸偵測**：監控力/力矩訊號，力超過閾值（如 3 N）→ 切換到阻抗/力控模式。切換瞬間**不能跳變** — 需要用 ramp 函數平滑過渡 $K_d$ 和 $D_d$
   - **Phase 3 — 穩態接觸**：阻抗或力控穩定工作，力追蹤期望值
3. **工程細節**：
   - 接觸偵測不能只看力閾值 — 需要加速度補償和濾波，避免運動慣性力誤觸發
   - $K_d$ 的 ramp 過渡時間通常 50–200 ms
   - 過渡期間限制最大力（力矩飽和保護），防止切換瞬間力尖峰
4. **進階做法**：
   - Variable Impedance Control：用 RL 或 Model Reference Adaptive Control 線上調整 $K_d, D_d$
   - 基於能量的切換：監控碰撞能量 $E = \frac{1}{2} m v^2$，接近工件時降低速度確保碰撞能量在安全範圍

**面試官想聽到**：狀態機三階段設計 + 接觸前預降剛度 + 切換平滑過渡 + 力矩飽和保護。這是工業力控落地最常遇到的工程問題。

</details>

<details>
<summary>Q4（難）：你要在一臺新的 7-DoF 協作臂上實作全身阻抗控制（不只末端，每個關節都能柔順）。設計你的控制架構。</summary>

**完整推理鏈**：

1. **全身阻抗 vs 末端阻抗**：
   - 末端阻抗：只控制末端 6D 的接觸行為，$\tau = J^T F_{\text{imp}}$
   - 全身阻抗：每個關節都有自己的虛擬彈簧阻尼，人碰到任何位置都能柔順
2. **架構設計**：
   - 需要**關節力矩感測器**（如 KUKA iiwa 每個關節都有）或至少能從電流精確估算力矩
   - 每個關節獨立的阻抗：$\tau_i = K_{d,i} (q_{d,i} - q_i) + D_{d,i} (\dot{q}_{d,i} - \dot{q}_i) + g_i(q)$
   - 末端任務疊加：用 null-space projection 確保末端任務不受全身柔順影響
3. **碰撞偵測與反應**：
   - Generalized Momentum Observer：$r = K_O \left( p(t) - \int_0^t (\tau + C^T \dot{q} - g + r) dt \right)$
   - $r$ 不為零 → 有外部碰撞力 → 切換到高阻尼低剛度模式
4. **7-DoF 冗餘利用**：
   - 主任務：末端力控/阻抗（6D）
   - 冗餘自由度：$(I - J^+ J)$ null-space 做關節阻抗，提供全身柔順而不影響末端行為

**面試官想聽到**：關節空間阻抗 + null-space 分層 + 碰撞偵測（Momentum Observer）+ 感測器需求。

</details>

## 面試角度

1. **力位混合的 Selection Matrix 解耦** — 這是力控最常被問的核心概念。**帶出**：「力位混合控制的關鍵是用 Selection Matrix 把任務空間按自由度分解 — 例如研磨的法向做 PI 力控維持壓力、切向做位控走軌跡。這個解耦不是固定的，遇到曲面工件還需要即時旋轉 Selection Matrix 的座標系。」

2. **F/T sensor 動態補償** — 區分「會用 F/T sensor」和「真正用好 F/T sensor」的分水嶺。**帶出**：「F/T sensor 原始讀數包含工具重力和加速度慣性力，必須做動態補償。我的做法是開機時多姿態採樣辨識工具質量和質心，運行時即時扣除 $R^T m g$ 和 $m \ddot{x}$ 項。」

3. **阻抗 vs 導納的硬體選型** — 證明你不是死背公式，而是根據硬體特性選控制架構。**帶出**：「阻抗控制適合直驅或低減速比的硬體（如 Franka），因為能直接精確輸出力矩。高減速比工業臂（如 UR）反向驅動性差，適合導納控制 — 用 F/T sensor 量力、位控器執行修正。選錯架構會從根本上影響控制品質。」

4. **接觸過渡的狀態機設計** — 展現工業落地經驗。**帶出**：「自由空間到接觸的過渡是力控最容易出事的時刻。我的做法是三階段狀態機：接近時預降剛度、接觸偵測觸發切換、ramp 平滑過渡阻抗參數，加上力矩飽和保護防止衝擊。」

5. **Variable Impedance 與 RL 的結合** — 展示對前沿趨勢的理解。**帶出**：「固定阻抗參數無法適應動態變化的任務。現在的趨勢是用 RL 線上調整 $K_d, D_d$ — 把阻抗參數作為 policy 的 action space，reward 設計結合力追蹤精度和能量效率。這是 Variable Impedance Control 的核心思路。」

## 延伸閱讀

- **Hogan, "Impedance Control: An Approach to Manipulation" (1985)** — 阻抗控制的開山論文，定義了 impedance/admittance 的因果框架，理解力控必讀的經典
- **Raibert & Craig, "Hybrid Position/Force Control of Manipulators" (1981)** — Selection Matrix 力位混合控制的原始論文，研磨/裝配場景的理論基礎
- **《具身智能算法工程師 面試題》Ch6.1 力控基礎、Ch6.3 柔順控制** — 面試最常問的力控考點整理，包含阻抗/導納/力位混合的完整對比
- **ros2_control + force_torque_sensor_broadcaster** — ROS 2 框架下力控的標準實作入口，搭配 `cartesian_controllers` 套件可快速搭建阻抗/導納控制器
- **De Luca et al., "Collision Detection and Safe Reaction with the DLR-III Lightweight Robot Arm"** — 基於 Generalized Momentum Observer 的碰撞偵測經典論文，全身柔順控制的感測基礎
- **Buchli et al., "Variable Impedance Control — A Reinforcement Learning Approach"** — RL + 阻抗控制結合的代表性工作，展示如何用 policy gradient 線上調整 $K_d, D_d$
- **MuJoCo 官方教程 — Contact-rich manipulation** — 在模擬器中觀察力控行為的最佳起手式，含阻抗控制和力位混合的範例場景
