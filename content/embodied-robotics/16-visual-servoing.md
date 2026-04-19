---
title: "視覺伺服控制與特徵點提取"
prerequisites: ["07-forward-kinematics-dh", "15-model-predictive-control"]
estimated_time: 45
difficulty: 4
tags: ["visual-servoing", "ibvs", "pbvs", "feature-extraction"]
sidebar_position: 16
---

# 視覺伺服控制與特徵點提取

## 你將學到

- 能精確講出 IBVS 和 PBVS 的本質差異：IBVS 在像素空間直接控制、PBVS 先重建 3D 再控制，各自的適用場景和致命弱點
- 遇到「相機看著目標，機械臂要精準對準」時，知道先判斷 eye-in-hand 還是 eye-to-hand、2D 特徵夠不夠用、是否會經過 interaction matrix 奇異
- 判斷何時用純 IBVS、何時用 PBVS、何時該上 2.5D 混合伺服

## 核心概念

**精確定義**：**Visual servoing (VS)** 是用視覺回饋直接驅動機械臂控制迴路的技術 — 把相機看到的特徵（像素點、線段、矩等）和目標特徵的差異作為誤差訊號，通過 interaction matrix 映射到相機（或末端）的速度指令。本質是**感知到控制的最短反射弧**，跳過完整的建圖和軌跡規劃。

**兩大範式**：
- **IBVS (Image-Based Visual Servoing)**：直接在 2D 像素空間定義誤差 $e = s - s^*$（$s$ 是當前特徵、$s^*$ 是目標特徵），通過 interaction matrix 算出相機速度。不需要 3D 重建，但軌跡在 Cartesian space 可能不直觀。
- **PBVS (Position-Based Visual Servoing)**：先從影像估計目標的 3D 位姿 $^cT_o$，在 Cartesian space 定義誤差 $e = (t, \theta u)$，再算速度指令。笛卡爾空間軌跡直觀，但完全依賴 3D 重建精度（標定誤差會直接傳遞）。

**Interaction Matrix (Image Jacobian)** — 核心數學工具：

建立 2D 影像特徵速度 $\dot{s}$ 和 6-DoF 相機速度 $v_c = (v, \omega)$ 之間的映射：

$$
\dot{s} = L_s \cdot v_c
$$

**物理意義**：$L_s$ 告訴你「相機往哪個方向移 → 像素點會往哪個方向跑」。有了這個映射，就能反過來算：「像素點要從現在的位置移到目標位置，相機應該怎麼動」。

**Eye-in-Hand vs Eye-to-Hand**：
- **Eye-in-Hand**：相機裝在末端 — 末端動相機也動，interaction matrix 直接給末端速度。精密對位首選（PCB 插件、焊接）。
- **Eye-to-Hand**：相機固定在外部 — 看整個工作空間，需要額外的手眼轉換。全局視野，適合抓取規劃。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：相機影像 → 特徵提取 $s$（來自感知）、目標特徵 $s^*$（來自任務定義）
- **輸出**：相機/末端速度指令 $v_c$（送給運動控制器 / Jacobian 轉成關節速度）
- **下游**：$v_c$ 經 robot Jacobian $J$ 轉成 $\dot{q}$，送給關節伺服
- **閉環節點**：直接把**感知**和**控制**短路 — 不經過完整建圖/規劃，是最緊湊的視覺閉環

**最少夠用的數學**：

1. **單點的 Interaction Matrix**（一個像素點 $(u, v)$ 對應 2 行 × 6 列）：

$$
L_s = \begin{bmatrix}
-\frac{f}{Z} & 0 & \frac{u}{Z} & \frac{uv}{f} & -(f + \frac{u^2}{f}) & v \\
0 & -\frac{f}{Z} & \frac{v}{Z} & (f + \frac{v^2}{f}) & -\frac{uv}{f} & -u
\end{bmatrix}
$$

**物理意義**：$f$ 是焦距、$Z$ 是特徵點深度。前 3 列對應平移速度（越近 $Z$ 越小、像素移動越快），後 3 列對應旋轉速度（像素位置越偏、旋轉效應越大）。**$Z$ 未知是 IBVS 的核心困難** — 通常用深度相機或固定估計。

2. **IBVS 控制律**（最經典的比例控制）：

$$
v_c = -\lambda \hat{L}_s^+ (s - s^*)
$$

**物理意義**：$\hat{L}_s^+$ 是 interaction matrix 的偽逆，$(s - s^*)$ 是特徵誤差向量。$\lambda > 0$ 是增益。把像素誤差「反投影」回相機速度空間 — 直覺就是「特徵往目標差多少，相機就以對應速度去補」。

3. **PBVS 控制律**：

$$
v_c = -\lambda \begin{pmatrix} ^ct - ^{c^*}t \\ \theta u \end{pmatrix}
$$

**物理意義**：直接在 3D 空間做比例控制 — 平移誤差 + 旋轉誤差（axis-angle 表示）。軌跡在 Cartesian space 是直線，但精度完全依賴 $^cT_o$ 的估計品質。

<details>
<summary>深入：Interaction Matrix 的完整推導與奇異性分析</summary>

### 從針孔模型到 Interaction Matrix

假設針孔相機模型（無畸變）：

$$
u = f \frac{X}{Z}, \quad v = f \frac{Y}{Z}
$$

對兩邊取時間微分，用鏈式法則：

$$
\dot{u} = f \frac{\dot{X}Z - X\dot{Z}}{Z^2}
$$

將 3D 點在相機座標系中的速度分解為相機的平移速度 $v = (v_x, v_y, v_z)$ 和旋轉速度 $\omega = (\omega_x, \omega_y, \omega_z)$：

$$
\dot{P} = -v - \omega \times P
$$

展開整理，就得到上面的 $L_s$ 矩陣。

### 奇異性分析

$L_s$ 的秩決定了系統的可控性。常見的奇異情況：

1. **所有特徵點共面且平行於像平面**：$Z$ 全部相同，$L_s$ 失去深度方向的資訊，$v_z$ 不可控
2. **特徵點共線**：$L_s$ 的秩 < 6，某些自由度不可控
3. **特徵點太少**：$n$ 個點給 $2n$ 行的 $L_s$，至少需要 4 個非共面點才能讓 $L_s$ 滿秩（6）

### $n$ 個特徵點的堆疊

$$
L_s = \begin{bmatrix} L_{s_1} \\ L_{s_2} \\ \vdots \\ L_{s_n} \end{bmatrix} \in \mathbb{R}^{2n \times 6}
$$

用偽逆 $L_s^+ = (L_s^T L_s)^{-1} L_s^T$ 求解超定方程。特徵點越多越穩定（冗餘），但計算量也越大 — 實務上 4-8 個點是甜蜜區。

### IBVS 的軌跡問題

IBVS 在像素空間做線性插值，但對應的 Cartesian 軌跡可能嚴重非直線。經典例子：目標需要 180° 旋轉時，IBVS 會走出 retreat-then-advance 的弧線（先後退再前進），甚至可能因為 $Z$ 估計錯誤而失敗。

解法：**2.5D Visual Servoing** — 部分特徵在像素空間控制（平移），部分在 Cartesian 空間控制（旋轉），結合兩者優勢。

</details>

**常用 API / 工具鏈**：

| 層級 | 工具 | 介面示例 |
|------|------|----------|
| 特徵提取 | OpenCV | `cv2.goodFeaturesToTrack()`, `cv2.ORB_create()` |
| Visual servoing 框架 | ViSP | `vpServo.setServo(vpServo::EYEINHAND_L_cVe_eJe)` |
| 深度估計 | RealSense / ZED | `rs2::depth_frame.get_distance(u, v)` |
| ROS 2 整合 | visp_ros | `vpROSGrabber` + `vpServo` |
| 端到端 VS | 研究中 | CNN 直接從 image → $v_c$（跳過特徵提取） |

## 直覺理解

**類比：穿針引線**。IBVS 就像穿針 — 你盯著針眼和線頭在視野中的相對位置，直接微調手指讓兩者重合。你不需要知道針距離你幾公分（3D 座標），只需要在「視野畫面」中讓兩個東西對齊。PBVS 則像先用尺量出針的 3D 座標，規劃一條空間直線軌跡過去 — 更「理性」但需要精確量測。

**視覺比喻：兩種停車方式**。IBVS = 看後視鏡影像中車身和停車格線的相對位置，直接調方向盤。PBVS = 用超音波測距算出車和車位的精確 3D 位置，規劃最短路徑。前者不需要精確測距但軌跡可能彎，後者軌跡直但測距一偏就歪。

**模擬器觀察**：在 Gazebo + ViSP 裡設一個 eye-in-hand 場景（相機裝在 UR5 末端，看一個 AprilTag）：
- IBVS 模式：把 AprilTag 四角的像素座標設為目標，觀察末端軌跡 — 像素空間收斂很快但 Cartesian 軌跡是弧線
- PBVS 模式：估計 AprilTag 的 3D 位姿再做 Cartesian 控制 — 空間軌跡直線但若相機標定偏 2 度，最終位置會差好幾毫米
- 把相機幀率從 60 fps 降到 15 fps：觀察延遲造成的震盪（特別是 IBVS 增益 $\lambda$ 過大時）

## 實作連結

**三個典型工程場景**：

1. **PCB 插孔對位（IBVS）**：相機裝在末端（eye-in-hand），看 PCB 上的定位孔。孔的圓心像素座標作為特徵，interaction matrix 映射到末端速度。精度 < 0.1 mm。2D 平面場景 + 特徵豐富 → IBVS 最佳選擇。

2. **bin picking 抓取（PBVS）**：深度相機（eye-to-hand）看散亂零件，用點雲配準估計目標的 6-DoF 位姿 $^cT_o$，在 Cartesian space 做 PBVS 控制。需要精確的手眼標定 $^bT_c$。

3. **高速移動目標追蹤（2.5D 混合）**：追蹤傳送帶上的物件，需要既快速（像素級回饋）又不走怪軌跡（Cartesian 約束）。用 2.5D visual servoing：平移用 IBVS、旋轉用 PBVS 的混合控制律。

**Code 骨架**（Python，ViSP 風格 IBVS）：

```python
import visp  # visp-python bindings
import numpy as np

# 初始化 servo
servo = visp.vpServo()
servo.setServo(visp.vpServo.EYEINHAND_L_cVe_eJe)
servo.setLambda(0.5)  # 比例增益

# 定義 4 個點特徵
for i in range(4):
    s = visp.vpFeaturePoint()      # 當前特徵
    sd = visp.vpFeaturePoint()     # 目標特徵
    s.buildFrom(u_current[i], v_current[i], Z_est[i])  # Z 需要估計
    sd.buildFrom(u_target[i], v_target[i], Z_target[i])
    servo.addFeature(s, sd)

# 控制迴圈
# v_c = servo.computeControlLaw()  # 回傳 (vx,vy,vz,wx,wy,wz)
# dq = J_inv @ v_c                 # 轉成關節速度
# robot.setVelocity(dq)
```

<details>
<summary>深入：完整 IBVS + ROS 2 實作範例（Python）</summary>

```python
#!/usr/bin/env python3
"""
IBVS eye-in-hand: 用 AprilTag 四角做特徵，控制 UR5 末端對準目標
依賴: visp, opencv, ros2, apriltag
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import cv2

class IBVSNode(Node):
    def __init__(self):
        super().__init__('ibvs_controller')
        self.bridge = CvBridge()
        self.lambda_gain = 0.5
        self.Z_est = 0.3  # 估計深度 (m)，若有深度相機則即時更新

        # 相機內參 (從 CameraInfo 讀取)
        self.fx = 600.0
        self.fy = 600.0
        self.cx = 320.0
        self.cy = 240.0

        # 目標特徵 (AprilTag 四角在目標位姿的像素座標)
        self.s_star = np.array([
            [280, 200], [360, 200],
            [360, 280], [280, 280]
        ], dtype=np.float64)

        # ROS 2 介面
        self.vel_pub = self.create_publisher(
            TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.img_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

    def compute_interaction_matrix(self, points, Z):
        """計算 n 個點的堆疊 interaction matrix L_s (2n x 6)"""
        L = []
        for u, v in points:
            # 轉換到歸一化座標
            x = (u - self.cx) / self.fx
            y = (v - self.cy) / self.fy
            L.append([
                -1/Z,  0,    x/Z,  x*y,    -(1+x*x), y,
                0,     -1/Z, y/Z,  (1+y*y), -x*y,    -x,
            ])
        return np.array(L).reshape(-1, 6)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # 偵測 AprilTag (簡化，實務用 apriltag 庫)
        corners = self.detect_apriltag(frame)
        if corners is None or len(corners) != 4:
            return

        s = corners.astype(np.float64)  # 當前特徵 (4x2)

        # 特徵誤差
        error = (s - self.s_star).flatten()  # (8,)

        # Interaction matrix (8x6)
        Ls = self.compute_interaction_matrix(s, self.Z_est)

        # 偽逆
        Ls_pinv = np.linalg.pinv(Ls)  # (6x8)

        # IBVS 控制律: v_c = -lambda * Ls+ * error
        v_c = -self.lambda_gain * Ls_pinv @ error

        # 發布速度指令
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = float(v_c[0])
        twist.twist.linear.y = float(v_c[1])
        twist.twist.linear.z = float(v_c[2])
        twist.twist.angular.x = float(v_c[3])
        twist.twist.angular.y = float(v_c[4])
        twist.twist.angular.z = float(v_c[5])
        self.vel_pub.publish(twist)

    def detect_apriltag(self, frame):
        # 簡化版: 實務用 pupil_apriltags 或 apriltag_ros
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # detector = apriltag.Detector()
        # results = detector.detect(gray)
        # return results[0].corners if results else None
        return None  # placeholder

def main():
    rclpy.init()
    node = IBVSNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**關鍵實作細節**：
- `Z_est` 若用固定值，IBVS 仍可收斂但速度不精確；有深度相機就即時更新
- 增益 $\lambda$ 太大會因相機延遲震盪；典型從 0.3 開始調，配合 30+ fps
- 實際部署時，用 `moveit_servo` 的 `delta_twist_cmds` 介面最方便 — 自帶碰撞檢查和關節極限保護

</details>

<details>
<summary>深入：手眼標定誤差如何傳遞到控制精度</summary>

### Eye-in-Hand 手眼標定

相機裝在末端，需要知道相機相對法蘭的變換 $^eT_c$（eye-to-hand 則需要 $^bT_c$）。

標定公式（Tsai-Lenz 1989）：

$$
A_i X = X B_i, \quad i = 1, \dots, n
$$

其中 $A_i = {^{e_1}T_{e_i}}$（末端移動）、$B_i = {^{c_1}T_{c_i}}$（相機觀測）、$X = {^eT_c}$（待求解）。

至少需要 2 組不同的機器人運動（$n \ge 2$），實務跑 10-20 組取最小二乘解。

### 誤差傳遞分析

手眼標定的旋轉誤差 $\Delta R$ 和平移誤差 $\Delta t$ 會直接傳到控制結果：

- **PBVS 受影響最大**：目標位姿 $^bT_o = {^bT_c} \cdot {^cT_o}$，$^bT_c$ 有誤差 → $^bT_o$ 直接偏移 → 控制精度上界就是標定精度
- **IBVS 受影響較小**：控制律不直接用 $^eT_c$，但 interaction matrix 的 $Z$ 估計和 Jacobian 轉換仍需要。旋轉標定偏 1° → IBVS 約偏 1-2 mm（在工作距離 30 cm 時）
- **2.5D 混合**：平移部分走 IBVS（少受標定影響），旋轉部分走 PBVS（軌跡直）→ 綜合最佳

### 降低標定誤差的實務做法

1. **多姿態標定**：至少 15-20 組不同方向的標定運動
2. **非線性最佳化精修**：Tsai-Lenz 給初值，再用 Levenberg-Marquardt 最小化重投影誤差
3. **在線自標定**：機器人運動時持續更新 $^eT_c$，補償機械磨損
4. **直接用 IBVS 迴避**：如果任務允許，純 IBVS 最不依賴標定精度

</details>

## 常見誤解

1. **「PBVS 永遠比 IBVS 好，因為 3D 控制更精確」** — PBVS 的精度完全受限於 3D 位姿估計。相機標定偏 1°、深度估計偏 5%，PBVS 的末端誤差可以到公分級。反而 IBVS 不需要精確的 3D 重建，在 2D 特徵清晰的場景（PCB、平面對位）精度更高。**正確理解**：IBVS 適合平面 / 近距離精密場景，PBVS 適合大範圍 / 需要直線軌跡的場景。

2. **「IBVS 完全不需要標定」** — IBVS 不需要精確的 3D 重建，但仍需要：(a) 相機內參（焦距 $f$、主點），否則 interaction matrix 算不對；(b) 深度估計 $Z$，否則速度映射的 scale 全錯；(c) robot Jacobian $J$，否則 $v_c$ 轉不成 $\dot{q}$。**正確理解**：IBVS 對標定的**敏感度低**，不是**不需要**標定。

3. **「特徵越多越穩定」** — 特徵多確實增加冗餘（抗遮擋），但也：(a) 增加 interaction matrix 的計算量（$2n \times 6$）；(b) 品質差的特徵（模糊、誤匹配）反而拉低精度；(c) 特徵分布不好（共線、密集）會讓 $L_s$ 接近奇異。**正確理解**：4-8 個高品質、分散分布的特徵 >> 50 個密集低品質特徵。

4. **「Visual servoing 只能做慢速精密對位」** — 高速 visual servoing 是活躍研究領域。用事件相機（event camera，μs 級延遲）+ 高幀率處理，已有 500+ Hz 的 visual servoing 實現。傳統相機在 60 fps 下配合 EKF 預測也能做到 100+ Hz 控制。**關鍵**：瓶頸在影像處理延遲和相機幀率，不在控制律本身。

## 練習題

<details>
<summary>Q1（中）：PCB 自動插件任務，相機裝在末端看 PCB 上的定位孔，要求插件精度 < 0.05 mm。你選 IBVS 還是 PBVS？怎麼設計？</summary>

**完整推理鏈**：

1. **選 IBVS**：PCB 是平面場景、定位孔是高對比度圓形特徵、工作距離短（~5-10 cm）→ IBVS 的強項。PBVS 在這距離的深度估計精度不夠 0.05 mm
2. **特徵選擇**：用 2-4 個定位孔的圓心像素座標。OpenCV `HoughCircles` 或亞像素角點偵測
3. **深度估計**：工作距離幾乎固定（PCB 放在已知高度的治具上），用固定 $Z$ 就夠。若有深度相機，即時更新更好
4. **增益調整**：$\lambda$ 從 0.3 開始，配合 60 fps 相機。接近目標時切換為更小的 $\lambda$（0.1）避免震盪
5. **避開陷阱**：確認相機鏡頭畸變已校正 — 在邊緣區域畸變大會讓 interaction matrix 算錯，0.05 mm 精度下這是致命的

**面試官想聽到**：IBVS 在 2D 平面場景的精度優勢、特徵選擇策略、以及「畸變校正」這個容易忽略但對極高精度至關重要的細節。

</details>

<details>
<summary>Q2（難）：目標物需要 180° 旋轉才能對準，你發現 IBVS 走出先後退再前進的奇怪軌跡。怎麼分析和修？</summary>

**完整推理鏈**：

1. **根因分析**：IBVS 在像素空間做線性插值。180° 旋轉時，目標像素位置和當前像素位置在像素空間的「最短路徑」不等於 Cartesian 空間的最短路徑。像素座標的中間態可能對應到相機後退的位姿
2. **經典現象**：稱為 IBVS 的 retreat 問題。特徵點在像素空間交叉 → interaction matrix 的 $Z$ 估計變負 → 速度方向反轉
3. **解法一：2.5D Visual Servoing**：平移方向用 IBVS（像素誤差），旋轉方向用 PBVS（$\theta u$ 表示的旋轉誤差）。避免旋轉在像素空間走歪路
4. **解法二：Path Planning + VS**：先用 PBVS 做粗定位（旋轉到 ~10° 內），再切 IBVS 做精定位
5. **解法三：虛擬特徵**：選不會因旋轉交叉的特徵（如影像矩 $\mu$ 而非角點），讓 interaction matrix 在大旋轉下仍穩定

**面試官想聽到**：理解 IBVS retreat 問題的根因（像素空間線性化 ≠ Cartesian 空間線性化），以及 2.5D VS 的混合策略。

</details>

<details>
<summary>Q3（中-難）：相機 15 fps，但控制迴圈 100 Hz。影像和控制頻率不匹配，怎麼處理？</summary>

**完整推理鏈**：

1. **問題本質**：100 Hz 控制需要每 10 ms 一個速度指令，但影像每 67 ms 才更新一次。中間 6 個控制週期沒有新的影像
2. **解法：異步插值 + 預測**：
   - 每次收到新影像 → 更新特徵 $s_k$ 和 interaction matrix $L_s$
   - 在影像之間的控制週期 → 用 EKF 預測特徵位置：$\hat{s}_{k+\Delta t} = s_k + L_s \cdot v_c \cdot \Delta t$
   - 或者用前兩幀的特徵做線性外推
3. **增益降低**：因為有效回饋頻率只有 15 Hz，$\lambda$ 要比 60 fps 時更小（~0.1-0.2），否則基於過時資訊的高增益控制會震盪
4. **硬體升級路徑**：如果精度要求高，換高幀率工業相機（120+ fps）或事件相機。事件相機的微秒級延遲 + 異步觸發特別適合 visual servoing
5. **避開陷阱**：不要在沒有新影像的週期直接 hold 上一次的 $v_c$ — 物體移動了但速度沒更新，會累積位置誤差

**面試官想聽到**：EKF 預測插值是標準做法、增益必須配合有效回饋頻率、事件相機是硬體解決方案。

</details>

## 面試角度

1. **Interaction Matrix 的物理意義與奇異規避** — 這是 visual servoing 面試的核心考點。帶出：「Interaction matrix 就是影像空間的 Jacobian — 它把 6-DoF 相機速度映射到 2D 特徵速度。奇異發生在特徵共面共線或深度趨近零時，我的做法是選分散的特徵點 + 監控 $L_s$ 的 condition number，接近奇異就降增益或切換特徵組。」

2. **IBVS vs PBVS 的場景選擇邏輯** — 展現工程判斷力而非死背教條。帶出：「不是哪個更好的問題，而是看場景：平面 + 短距離 + 高精度 → IBVS；大範圍 + 需要直線軌跡 + 有可靠深度 → PBVS；大旋轉或混合需求 → 2.5D。我選之前會先評估相機標定精度和深度估計品質，這兩個決定了 PBVS 的天花板。」

3. **手眼標定誤差如何傳遞到控制精度** — 區分「用過 visual servoing」和「真正理解精度限制」。帶出：「PBVS 的精度上界就是手眼標定精度 — 旋轉偏 1° 在 30 cm 工作距離會導致 ~5 mm 末端誤差。所以高精度場景我偏好 IBVS，或者投資更多時間做多姿態標定 + LM 精修。」

4. **異步延遲補償** — 展現 real-world deployment 經驗。帶出：「相機 15 fps 但控制 100 Hz 是常見場景。我的做法是用 EKF 在影像間預測特徵位置，同時把增益降到配合有效回饋頻率。另外必須在 MPC/控制律裡加入相機延遲模型，否則高增益 + 延遲 = 震盪。」

5. **從特徵工程到端到端 VS 的趨勢** — 展現前沿視野。帶出：「傳統 visual servoing 依賴手工特徵（ORB、角點），在遮擋和光照變化下容易丟失。業界趨勢是用 CNN 直接從 image 回歸 $v_c$（端到端 VS），或用 NeRF/foundation model 做更魯棒的特徵表示。但傳統方法的可解釋性和安全保證仍是工業場景的硬性需求。」

## 延伸閱讀

- **Chaumette & Hutchinson,《Visual Servo Control Part I & II》(IEEE RAM, 2006/2007)** — Visual servoing 的經典教科書級綜述，IBVS/PBVS/2.5D 的理論基礎全在這裡
- **ViSP (Visual Servoing Platform) 教學與範例** — 最完整的開源 VS 框架，C++ 核心 + Python bindings，教學覆蓋所有場景
- **Tsai & Lenz,《A New Technique for Fully Autonomous and Efficient 3D Robotics Hand-Eye Calibration》(1989)** — 手眼標定的奠基論文，至今仍是標準方法
- **論文《Event-based Visual Servoing》** — 事件相機在 VS 的應用，μs 級延遲突破幀率瓶頸
- **論文《End-to-End Training of Deep Visuomotor Policies》(Levine et al., 2016)** — 端到端 VS 的開山作，CNN 直接從 image → motor command
- **OpenCV 相機標定教學** — 相機標定是 VS 的前置條件，畸變校正是精度的基礎
- **《具身智能算法工程師 面試題》Ch7.1 視覺伺服基礎、Ch7.4 手眼標定** — 面試高頻考點：interaction matrix 推導、IBVS vs PBVS 選擇邏輯
