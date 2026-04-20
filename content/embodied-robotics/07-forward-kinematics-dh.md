---
title: "剛體正向運動學與 DH 參數建模"
prerequisites: ["05-ros2-tf-and-tools"]
estimated_time: 45
difficulty: 3
tags: ["forward-kinematics", "dh-parameters", "kinematics"]
sidebar_position: 7
---

# 剛體正向運動學與 DH 參數建模

## 你將學到

- 能用兩句話講清楚 forward kinematics 在做什麼，面試被問不含糊
- 遇到「機械臂裝了新夾爪、末端位置算錯」這類情境，知道先檢查 Tool Frame 與 DH 零位
- 判斷何時用 Standard DH、何時用 Modified DH，何時該放棄 DH 改用 Product of Exponentials

## 核心概念

**精確定義**：**Forward kinematics (FK)** 是從關節變量（rotary joint 的角度、prismatic joint 的距離）計算末端執行器在任務空間（Cartesian space）的位姿（position + orientation）。本質是 **關節空間 → 任務空間的映射**。

**DH 參數（Denavit-Hartenberg）**：用四組量 $(\alpha, a, d, \theta)$ 標準化描述相鄰連桿的幾何關係，讓多自由度機械臂的變換可以被系統化地推導。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：關節向量 $q = [\theta_1, \theta_2, \dots, \theta_n]$（rad 或 m）
- **輸出**：末端執行器相對於 base frame 的 4×4 齊次變換矩陣 $^0T_n$（含 position 與 orientation）
- **下游**：軌跡規劃（碰撞檢測、軌跡插值）、IK 求解器的數值初值、閉環控制器的誤差計算、視覺伺服的手眼座標轉換、Rviz 的 TF tree 視覺化
- **閉環節點**：橫跨 **感知（狀態估計）** 與 **控制（誤差反饋）**；FK 從編碼器讀當下關節角算出實際末端位姿，和 planner 給的目標位姿相減形成誤差訊號

**一句話版本**：「告訴我每個馬達現在轉了幾度，我就能算出夾爪在空間的絕對座標和朝向。」

**最少夠用的數學**：

1. **齊次變換矩陣**（統一 rotation + translation 成單一矩陣乘法）：

$$
T = \begin{bmatrix} R & p \\ \mathbf{0} & 1 \end{bmatrix} \in SE(3)
$$

$R \in SO(3)$ 是 3×3 旋轉矩陣（子 frame 相對父 frame 的朝向）；$p \in \mathbb{R}^3$ 是平移向量（子 frame 原點在父 frame 下的位置）。**組成 4×4 的物理意義**：連續座標變換可用純矩陣乘法串接，避免 `R·x + p` 這種混合運算。

2. **單一連桿的 DH 變換**（Standard DH，四步基本變換的合成）：

$$
^{i-1}T_i = \text{Rot}_Z(\theta_i) \cdot \text{Trans}_Z(d_i) \cdot \text{Trans}_X(a_i) \cdot \text{Rot}_X(\alpha_i)
$$

**物理意義**：先繞 $Z_{i-1}$ 轉關節角 $\theta_i$、沿 $Z_{i-1}$ 滑 $d_i$、沿新的 $X_i$ 推連桿長 $a_i$、最後繞 $X_i$ 扭連桿扭角 $\alpha_i$。

3. **整條機械臂 FK**（從 base 右乘到末端）：

$$
^0T_n = {^0T_1} \cdot {^1T_2} \cdots {^{n-1}T_n}
$$

**物理意義**：把每一節關節的局部變換「接力累加」，最終得到末端夾爪在 base frame 下的絕對位姿。順序**不能顛倒** — DH 約定每個 $T$ 是相對於當前移動 frame 的變換，必須右乘。

<details>
<summary>深入：Standard DH 變換矩陣的完整展開式與幾何推導</summary>

Standard DH 相鄰連桿變換的完整 4×4 矩陣：

$$
^{i-1}T_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i \cos\alpha_i & \sin\theta_i \sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i \cos\alpha_i & -\cos\theta_i \sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

每一項的幾何意義：
- $\cos\theta_i / \sin\theta_i$：繞 Z 軸旋轉的水平投影分量
- $\cos\alpha_i / \sin\alpha_i$：繞 X 軸扭角後，Y/Z 軸的傾斜投影
- $a_i \cos\theta_i / a_i \sin\theta_i$：連桿長 $a_i$ 經關節角 $\theta_i$ 旋轉後在父 frame X/Y 軸上的分量
- $d_i$：沿父 frame Z 軸的線性平移

**Standard 與 Modified DH 的關鍵差異**（以 frame $i$ 的 $Z_i$ 軸對齊哪個關節軸作分界，最不會搞混）：
- **Standard DH**（Denavit 原版）：frame $i$ 的 $Z_i$ 對齊**第 $i+1$ 軸**；$a_i, \alpha_i$ 是第 $i$ 軸到第 $i+1$ 軸的幾何關係；乘法順序為 `Rot_Z → Trans_Z → Trans_X → Rot_X`
- **Modified DH**（Craig 版）：frame $i$ 的 $Z_i$ 對齊**第 $i$ 軸**；$a_{i-1}, \alpha_{i-1}$ 是第 $i-1$ 軸到第 $i$ 軸的幾何關係；乘法順序為 `Rot_X → Trans_X → Rot_Z → Trans_Z`

**業界選擇**：教科書常見 Standard DH；但 Modified DH 對樹狀結構、閉鏈、相鄰平行關節處理更穩定，現代 ROS 底層運動學庫（如 Orocos KDL、Pinocchio 內部）傾向用 MDH 或直接跳過 DH 改用 screw theory（Product of Exponentials）。

</details>

**常用 API**（業界工具鏈）：

| 層級 | 套件 | 介面示例 |
|------|------|----------|
| ROS 2 底層 | tf2 | `buffer.lookup_transform(target, source, time) → TransformStamped` |
| 規劃中樞 | MoveIt | `RobotState::getGlobalLinkTransform(link_name) → Eigen::Isometry3d` |
| 極速求解 | Pinocchio | `pinocchio::forwardKinematics(model, data, q)`（MPC 首選） |
| 模擬器內建 | MuJoCo / PyBullet | `mj_forward(model, data)` / `p.getLinkState(robot, link_idx)` |

## 直覺理解

**類比：接力傳球 / 搭積木**。Base 是起點，每個關節是一位跑者，$^{i-1}T_i$ 記錄「下一棒相對於上一棒的位置與面朝方向」。把每棒的相對偏移連乘，就得到最後一棒（夾爪）相對於起點的絕對位姿。

**視覺比喻：工程師拿捲尺量 DH**：
- **$a$**（連桿長）：拿捲尺量兩根相鄰 Z 軸之間最短的公垂線長度
- **$\alpha$**（連桿扭角）：沿公垂線望過去，兩根 Z 軸扭開了幾度
- **$d$**（連桿偏距）：沿前一根 Z 軸滑多遠才碰到公垂線起點
- **$\theta$**（關節角）：唯一會動的量 — 這根軸現在轉了幾度

**模擬器觀察**：在 Isaac Sim / Gazebo / MuJoCo 裡，給一組 $q$ 傳進去 → 讀模擬器 Ground Truth 的末端位姿 → 和自己算的 $^0T_n$ 比對。位置誤差 < 0.1 mm、姿態 < 0.01 rad 就算通過。在 rviz2 把 Fixed Frame 設為 `base_link`、加 TF plugin 打開 Show Axes，肉眼看三軸方向和自己算的是否一致。

## 實作連結

**三個典型工程場景**：

1. **ROS 2 即時 TF 廣播**：每個 control loop tick，`robot_state_publisher` 從 `/joint_states` 讀關節角，內部跑 FK，把每個 link 的變換發到 TF tree。其他 node 用 `tf2_buffer.lookup_transform` 就能查任意兩個 frame 的關係。

2. **MPC 模型預測控制**：1 kHz 控制迴圈每 1 ms 需要從當前 $q$ 算未來 $N$ 步末端軌跡，FK 得跑 $N$ 次且不能分配 heap。實務做法是離線用 SymPy 把 $^0T_n$ 推導成封閉解析式，線上跑展開過的 C++ 賦值語句 + SIMD。

3. **強化學習 reward shaping**：RL policy 輸出 joint torque，每個 step 需要算末端位姿跟 target 比對算 reward。直接用模擬器內建的 FK API（MuJoCo 的 `mj_forward`）比自己寫快又穩。

**Code 骨架**（C++，ROS 2 + KDL 版）：

```cpp
// 讀 URDF → 建 KDL::Tree → 選 chain → 算 FK
KDL::Tree tree;
kdl_parser::treeFromUrdfModel(urdf_model, tree);
KDL::Chain chain;
tree.getChain("base_link", "tool0", chain);

KDL::ChainFkSolverPos_recursive fk_solver(chain);
KDL::JntArray q(chain.getNrOfJoints());
q(0) = 0.1; q(1) = -0.5; /* ... */

KDL::Frame end_effector_pose;  // 輸出：末端 4x4
fk_solver.JntToCart(q, end_effector_pose);
```

<details>
<summary>深入：完整 Python 實作（從 DH 表算 FK，可 copy-paste 跑）</summary>

```python
import numpy as np

def dh_transform_standard(a, alpha, d, theta):
    """Standard DH 單連桿變換：Rot_Z(theta) * Trans_Z(d) * Trans_X(a) * Rot_X(alpha)"""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d    ],
        [0,   0,        0,       1    ],
    ])


def forward_kinematics(dh_table, joint_values, tool_transform=None):
    """
    dh_table: list of (a, alpha, d, theta_offset) 每連桿
    joint_values: rotary 關節給 theta，prismatic 關節給 d（需對應修改）
    tool_transform: 可選，TCP 相對於最後一個 link 的 4x4 變換
    """
    T = np.eye(4)
    for (a, alpha, d, theta_offset), q in zip(dh_table, joint_values):
        theta = theta_offset + q  # 加上關節零位 offset
        T = T @ dh_transform_standard(a, alpha, d, theta)
    if tool_transform is not None:
        T = T @ tool_transform  # 右乘工具變換
    return T


# 範例：UR5 的 Standard DH 表（以下為四捨五入值，精確值請查 Universal Robots 官方手冊：
# d_1=0.089159, d_4=0.10915, d_5=0.09465, d_6=0.0823, a_2=-0.425, a_3=-0.39225）
ur5_dh = [
    (0,      np.pi/2,  0.089,  0),
    (-0.425, 0,        0,      0),
    (-0.392, 0,        0,      0),
    (0,      np.pi/2,  0.109,  0),
    (0,      -np.pi/2, 0.095,  0),
    (0,      0,        0.082,  0),
]
q = [0.0, -np.pi/2, 0.0, -np.pi/2, 0.0, 0.0]
T_end = forward_kinematics(ur5_dh, q)
print("末端位置:", T_end[:3, 3])
print("旋轉矩陣:\n", T_end[:3, :3])
```

若裝了 Pinocchio，實務上會直接用：

```python
import pinocchio as pin
model = pin.buildModelFromUrdf("ur5.urdf")
data = model.createData()
pin.forwardKinematics(model, data, q)
print(data.oMi[model.getFrameId("tool0")])  # 末端 SE3
```

Pinocchio 內部用 spatial algebra + recursive Newton-Euler，比手寫的矩陣連乘快一個數量級。

</details>

## 常見誤解

1. **混用 Standard 與 Modified DH** — 兩者 frame 附著位置和 4 步變換的乘法順序完全不同。混用會讓結果發散。**避開**：實作前先查原廠手冊確認，把 DH 版本寫在 code comment 裡。
2. **忽略關節零位 offset** — URDF 的零位不等於 DH 表的零位，物理機械臂的機械零點也可能再加上一層 offset。**避開**：FK 代入公式前，`theta_math = theta_motor + zero_offset_i`；除錯時把所有關節設 0 度，確認 FK 算出的末端 XYZ 和實機初始位一致。
3. **忘記 Tool Frame (TCP)** — 只算到第 6 軸 flange，忽略末端夾爪/焊槍的偏移，導致目標以為到了但實際差幾十 cm。**避開**：FK 永遠寫成 $^0T_{\text{tool}} = {^0T_6} \cdot {^6T_{\text{tool}}}$，工具變換必須右乘。
4. **旋轉矩陣連乘後數值累積誤差** — 浮點誤差累積會讓 $R \cdot R^T \ne I$，姿態扭曲。**避開**：連乘後定期 renormalize（SVD 或 Gram-Schmidt），或底層改用 quaternion 計算，最後轉回矩陣。

## 練習題

<details>
<summary>Q1（簡單）：給你一支全新 6-DoF 機械臂 URDF，如何快速嚴謹驗證你 C++ 手寫的 FK 正確？</summary>

**完整推理鏈**：

1. **讀 URDF、啟 ROS 2**：跑 `robot_state_publisher` + `rviz2`，確認 TF tree 正常廣播
2. **設計測試用例**：不能只測 zero-config，至少給：
   - (A) 零位
   - (B) 各關節極限角度（±90° 或各自 limit）
   - (C) 1000 組 Monte Carlo 隨機 $q$
3. **取 Ground Truth**：`tf2_buffer.lookup_transform("base_link", "tool0", rclcpp::Time(0))`
4. **矩陣比對**：C++ FK 的 4×4 和 TF 傳回的 `TransformStamped` 轉成矩陣後相減
   - 位置誤差 < 1e-6 m、姿態誤差 < 1e-6 rad → 通過
   - 若不符，先懷疑 URDF `<origin>` 偏移跟你的 DH 零位定義不一致

**面試官想聽到**：自動化隨機採樣比對 GT 的軟體工程思維，並知道 URDF 自由座標系與 DH 嚴格公垂線規範之間極易不一致是主要陷阱。

</details>

<details>
<summary>Q2（中）：機械臂加裝非對稱夾爪後，真實世界位置跟 FK 算的固定差 5 cm 加一個詭異角度，怎麼有邏輯地除錯？</summary>

**完整推理鏈**：

1. **隔離變數**：出廠 FK 已驗證，先排除底層 DH 錯。指令所有關節 0 度、只轉最後一軸，觀察誤差是跟著旋轉還是固定在絕對空間
2. **確診 TCP 錯**：誤差跟著旋轉 → 是 tool frame 偏移沒設對
3. **右乘工具變換**：建立 $^6T_{\text{tool}}$（含 xyz 平移 + 非對稱 RPY），程式改成 $^0T_{\text{tool}} = {^0T_6} \cdot {^6T_{\text{tool}}}$；**必須右乘**，因為工具附著在第 6 軸的局部移動 frame
4. **雷射追蹤儀校準**：讓機械臂多姿態移動，用最小二乘反解精確的 $^6T_{\text{tool}}$，消除組裝公差

**面試官想聽到**：控制變因法的清晰除錯邏輯；懂 tool frame 是右乘在 FK chain 末端；具備「理論 CAD 尺寸 → 實機校準」消除組裝公差的實戰經驗。

</details>

<details>
<summary>Q3（中-難）：RL policy 輸出 joint torque，sim 完美但真機軌跡抖到無法到達目標，怎麼診斷與修？</summary>

**完整推理鏈**：

1. **Sim 內建視覺化 pipeline**：每個 env step callback，從模擬器讀 $q$ 丟進 FK 算末端 XYZ
2. **發 Rviz marker**：`visualization_msgs/Marker` 類型 `LINE_STRIP` 收集軌跡點，肉眼看 sim 裡軌跡是否真的平滑
3. **診斷 Reality Gap**：RL 對 unmodeled dynamics 過擬合 — 真機有 harmonic drive 背隙、關節柔性、非線性摩擦，sim 剛體模型看不到
4. **解法兩路併行**：
   - **Domain Randomization**：訓練時對摩擦、質量、阻尼 ±20% 隨機擾動
   - **System Identification**：真機跑掃頻訊號，反辨識動力學參數，更新 sim 參數縮小 gap

**面試官想聽到**：RL torque 控制 → sim 積分 → $q$ → FK → 末端 XYZ 這條鏈要講清楚；知道 Sim-to-Real 核心落差是 unmodeled dynamics；能給 Domain Randomization + System ID 的標準解法。

</details>

<details>
<summary>Q4（難）：6-DoF 升 7-DoF 冗餘機械臂，1000 Hz 控制迴圈 FK + Jacobian 要 1 ms 內算完，傳統矩陣乘法 CPU 負載超標，怎麼壓？</summary>

**完整推理鏈**：

1. **DH 擴 7 軸**：加第 7 軸的 $(\alpha_7, a_7, d_7, \theta_7)$；理解 7-DoF 的冗餘會在 IK 產生無窮解，但 FK 仍是 7 個 4×4 連乘
2. **符號預計算**：線上 for-loop 矩陣乘有大量 × 0 加 0 浪費。離線用 SymPy / MATLAB 推 $^0T_7$ 和 $J$ 的封閉解析式
3. **C++ 極致壓榨**：
   - 解析式展開成扁平賦值語句
   - 把 $\cos\theta_i$ / $\sin\theta_i$ 預計算成 `c1, s1, ...` 重用
   - 用 SIMD (AVX/SSE) 平行算 Jacobian 元素
   - **絕不** `new` / `malloc` / `std::vector` 擴容
4. **Null-space 避障**：$\Delta\Theta = J^+ v + (I - J^+ J) \nabla H$，主任務追蹤末端軌跡，次任務用 $\nabla H$ 把手肘推離障礙物；全部在 1 ms 內完成

**面試官想聽到**：對軟體底層效能（記憶體、符號化簡、三角函數快取、avoid heap alloc）的極致追求；懂 7-DoF 冗餘核心價值就是 null-space — 不改變末端軌跡的前提下拿關節自由度做事（避障、遠離 limit、避奇異）。

</details>

## 面試角度

1. **DH 版本陷阱（Standard vs Modified）** — 證明做過現代複雜機械臂底層開發，不只是死背教科書。**帶出**：「在實作 FK 前，我一定會先查原廠手冊確認是 Standard 還是 Modified DH，因為這直接影響座標系原點附著位置，弄錯整個 TF tree 會完全發散。」

2. **理論與模擬器的對齊（URDF vs DH）** — 把純數學公式和 ROS 2 軟體工程實踐結合，展現你踩過「理論 vs 實機落差」的坑。**帶出**：「寫完 FK 我不會只看公式，會把輸出的 4×4 和 `tf2_echo` 或模擬器 GT 做千組隨機點位比對，確保 URDF 的 `<origin>` 偏移跟我的 DH 零位完全一致。」

3. **底層實時性與效能壓榨** — 區分「純做 AI」與「真能把演算法落地到硬體」的關鍵分水嶺。**帶出**：「1 ms 控制週期下我不會在 C++ 迴圈做矩陣連乘，而是離線推封閉解析解、展開為賦值語句，徹底避開 heap 分配並用 SIMD 加速。」

4. **奇異點預判（Jacobian 秩虧）** — 從 FK 延伸到 Jacobian 的邊界敏感度。**帶出**：「算 FK 時我會同步監控 Jacobian condition number，接近奇異就觸發 damped least squares 或 null-space 優化，防止後續關節速度指令爆掉。」

## 延伸閱讀

- **《具身智能算法工程師 面試題》Ch1.2 運動學建模、Ch3 奇異點分析** — 對答如流 DH 物理意義、多解/無解/奇異經典考點
- **《ROS 2 機器人開發》Ch6 URDF + Xacro + TF tree** — 把紙上矩陣落到 ROS 2 系統裡可維護的機器人模型
- **Lynch & Park,《Modern Robotics》Ch4（PoE）** — 現代替代 DH 的方式，無奇異、不依賴座標軸死板設定；MIT 開放教材，影片與 Notebook 齊全
- **Pinocchio 官方範例** — 讀 C++ spatial algebra source code 學如何把 FK / Jacobian / dynamics 寫到極速
- **論文《Sim-to-Real Transfer in Deep Reinforcement Learning for Robotics: a Survey》** — Reality Gap 與 Domain Randomization 全貌
- **URDF / SDF / MJCF 三種格式的官方文件** — Sim-to-Real 時不同模擬器的解讀差異是主要地雷
