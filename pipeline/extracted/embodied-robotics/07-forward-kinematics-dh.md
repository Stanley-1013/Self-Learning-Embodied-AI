# Extracted: 剛體正向運動學與 DH 參數建模
<!-- 提取時間: 2026-04-19 -->
<!-- 來源 Notebook: 0502daf4-1648-4456-bfd4-f1dee3875753 -->
<!-- NotebookLM session: e6cb9a47 -->
<!-- 5 次查詢完整原始輸出（供合成章節時參考，亦供日後追溯） -->

## Q1 — 精確定義 / 閉環定位 / 一句話版本
(見合成章節 content/embodied-robotics/07-forward-kinematics-dh.md `核心概念` 段落)

- Forward kinematics：由關節變量（旋轉角、平移距離）計算末端執行器在任務空間的位姿；本質是「關節空間 → 任務空間」映射
- DH 參數：用 4 組量 `(a, α, d, θ)` 標準化描述相鄰連桿的幾何關係
- Standard DH vs Modified DH（Craig 版）：座標系附著位置與變換順序不同；樹狀結構 / 平行關節場景 MDH 較穩定；現代 ROS 底層運動學庫較常用 MDH（通識補充）
- 閉環位置：跨「感知（狀態估計）」與「控制（誤差反饋）」兩節點；下游：軌跡規劃、IK 初值、閉環控制、視覺伺服、Rviz / TF tree
- 白話一句：「告訴我每個馬達轉幾度、每個滑軌多長，我就能算出夾爪在空間的絕對位置跟朝向」

## Q2 — 核心數學 / API / 物理意義
### 齊次變換矩陣
$T = \begin{bmatrix} R & p \\ 0 & 1 \end{bmatrix}$；R 是 3×3 旋轉、p 是 3×1 平移；組成 4×4 讓座標變換化為純矩陣乘法
### DH 變換矩陣（Standard DH，來源用的版本）
$^{i-1}T_i = \text{Rot}_Z(\theta_i) \cdot \text{Trans}_Z(d_i) \cdot \text{Trans}_X(a_i) \cdot \text{Rot}_X(\alpha_i)$
### 整條機械臂 FK
$^0T_n = ^0T_1 \cdot ^1T_2 \cdots ^{n-1}T_n$；必須從基座向末端右乘，不可顛倒
### 姿態表示優缺點
- Rotation matrix：無奇異、9 參數 3 自由度冗餘、運算重
- Euler angles (RPY)：直觀、Gimbal lock、插值難
- Quaternion：4 參數、無死鎖、插值平滑、不直觀
- Axis-angle：直觀但 0 度無定義
### 業界 API
- ROS 2 tf2：`buffer.lookup_transform(target, source, time)` → `TransformStamped`
- MoveIt: `RobotState::getGlobalLinkTransform()`
- KDL: 底層 C++ 樹狀求解器
- Pinocchio: 極速 C++/Python 庫，MPC 首選
- MuJoCo / PyBullet: 模擬器內建 FK

## Q3 — 直覺 / 視覺 / 常見誤解
### 類比
- **接力傳球 / 搭積木**：每個 T 記錄「下一棒相對於上一棒」的位姿；連乘 = 累加每棒的相對偏移
- **工程師拿捲尺量 DH**：a = 兩軸最短公垂線；α = 兩軸扭角；d = 沿軸滑動距離；θ = 關節轉角（唯一會動的）
### 模擬器驗證
- Gazebo/MuJoCo：給一組 q → 讀 GT 末端位姿 → 比對自己算的 $^0T_n$
- 誤差閾值：位置 < 0.1 mm / 姿態 < 0.01 rad
- ROS 2 + rviz2：fixed frame = base_link + TF plugin + Show Axes，比對自己算的末端軸方向
### 4 個真實陷阱
1. Standard / Modified DH 混用 — 座標系附著位置與乘法順序不同
2. 關節零位（zero offset）差異 — URDF 零位 ≠ DH 零位，`θ_math = θ_motor + offset`
3. 漏掉 Tool Frame — 完整 FK 要乘到 `^0T_tool`，不能停在 flange
4. 旋轉矩陣數值累積誤差 — 連乘後失去正交性，要定期 renormalize 或改用四元數
### 除錯工具關鍵字
`tf2_echo` / `rqt_tf_tree` / `view_frames` / `rviz2 TF plugin` / `robot_state_publisher`

## Q4 — 4 題情境題 + 完整推理鏈
1. **6-DoF URDF 快速驗證 FK 函式正確性** → robot_state_publisher + rviz2 → 設計邊界 + 隨機 1000 組測試 → tf2_echo 取 GT → 誤差 <1e-6 判定
2. **加裝非對稱夾爪後真實誤差 5 cm** → 隔離變數（只轉最後一軸）→ 確診 TCP 錯 → 右乘 `^6T_tool` → 雷射追蹤儀校準
3. **RL torque policy 在 sim 完美但真機抖** → step callback 算 FK(q) 發 Marker LINE_STRIP 視覺化 → 診斷 unmodeled dynamics（backlash / 柔性 / 摩擦）→ Domain Randomization + System ID
4. **7-DoF 1000 Hz 極致效能** → DH 擴充到 7 軸 → 符號預計算封閉解析解（MATLAB / SymPy）→ C++ 展開賦值 + 三角函數快取 + SIMD → Null-space 避障 `ΔΘ = J⁺v + (I - J⁺J)∇H`

## Q5 — 面試 talking points / 延伸關鍵字 / 閱讀順序
### 面試 key points
1. DH 版本陷阱（Standard vs Modified） — 證明做過現代複雜機械臂底層開發
2. 理論與模擬器的對齊（URDF vs DH） — 軟體工程踩坑經驗
3. 底層實時性與效能壓榨 — 區分「純 AI」與「演算法落地」
4. 奇異點的預判與處理（Jacobian 秩虧）— 系統穩定性邊界敏感度
### 延伸關鍵字
- URDF / SDF / MJCF 格式差異 — Sim-to-Real 地雷
- Product of Exponentials (PoE) — DH 的現代替代，避奇異
- Pinocchio / KDL / Drake — 讀原始碼學 spatial algebra
- Sim-to-Real Domain Randomization — 摩擦/紋理隨機化
- VLA (Vision-Language-Action) 中的 FK 對齊 — 端到端機器人控制前沿
### 閱讀順序
1. 《面試題》Ch1.2 & Ch3 奇異點 — 對答如流經典考點
2. 《ROS 2 實踐》Ch6 URDF + Xacro + TF — 紙上矩陣 → 系統 TF tree
3. 《Sim-to-Real Survey》 — Reality Gap 與 Domain Randomization
4. 《C++ 性能優化指南》Ch6 & Ch13 + 併發編程 — 1000 Hz 控制迴圈不卡頓
