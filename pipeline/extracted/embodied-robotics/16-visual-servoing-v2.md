# Extracted v2: Ch16 視覺伺服 — 5 dedicated queries (from scratch)

## Q1 補強（IBVS/PBVS + Image Jacobian + Eye-in/Eye-to-Hand）

### 情境題 A：IBVS vs PBVS 本質對比
- **PBVS (Position-Based VS)**：
  - 相機 → 特徵匹配 + PnP → 重建 3D pose → 笛卡爾空間控制
  - **誤差 SE(3) 空間**：`e = log(T_target⁻¹ · T_current)^∨ ∈ ℝ⁶`
  - **優勢**：3D 軌跡直線直觀
  - **缺點**：極度依賴相機內參 + 手眼標定；3D 重建對雜訊敏感（2D 微抖 → 3D 巨跳變）
- **IBVS (Image-Based VS)**：
  - 跳過 3D 重建
  - **直接算 2D 像素差異**：`e = s_current - s_target ∈ ℝ^{2k}`（k 個特徵點）
  - Image Jacobian 直接映射像素速度 → 關節速度
  - **優勢**：對標定誤差 + 3D 模型誤差極魯棒
  - **缺點**：3D 軌跡不可預測（可能退後繞遠）；特徵出 FOV 系統崩潰
- **Hybrid 2.5D VS (Malis)**：
  - 旋轉誤差 3D 空間控（單應性 Homography 提取）
  - 平移誤差 2D 影像空間控
  - 特徵不易丟失 + 3D 軌跡平滑
- **場景選型**（面試必答）：
  - **高精度工業軸孔裝配**（CAD 模型 + 精準標定）→ **PBVS** 保證 3D 直線插入符 ISO
  - **非結構化抓取未知物 / 標定易漂移** → **IBVS** 標定誤差魯棒性強

### 情境題 B：Image Jacobian (Interaction Matrix)
- **定義**：相機 6-DoF 速度 `v_c = [v_x, v_y, v_z, ω_x, ω_y, ω_z]^T` 如何引發像素速度 `ṡ = [u̇, v̇]^T` 的線性關係
  - `ṡ = L(s, Z) · v_c`
- **單點交互矩陣**：
  ```
  L = [ -1/Z    0   u/Z    uv      -(1+u²)   v  ]
      [   0  -1/Z  v/Z  1+v²        -uv    -u  ]
  ```
  - **平移列 ∝ 1/Z**（越遠像素移動越慢）
  - **旋轉列與 Z 無關**
- **Depth Ambiguity Z 未知的三策略**：
  1. 假設常數 Z*（距離變化不大時）
  2. EKF 或自適應控制**在線估計 Z**
  3. 改用「面積/矩 (Moments)」特徵繞過單點深度依賴
- **4 個非共線點最小配置**：
  - 一點 2 方程 → 控制 6-DoF 看似 3 點夠
  - **但 3 點 PnP 有 4 個模糊解 (Cylinder Ambiguity)**
  - 加第 4 點提供 8 方程保 `L^T L` 滿秩 → 唯一確定解
- **Chaumette Conundrum 相機後退陷阱**：
  - 目標繞相機光軸純旋轉 180° 時
  - IBVS 迫使 2D 點在影像上走直線 → 4 點畫面中心必互相靠近（收縮）
  - 根據 L 矩陣讓像素收縮 = 相機**向後退 (增加 Z)**
  - 導致極大 3D 空間偏移，機械臂可能撞底座
  - **這就是 2.5D VS 發明的根本原因**
- **ViSP C++ 範例**：
  ```cpp
  vpFeaturePoint s;
  s.set_x(0.5); s.set_y(0.2); s.set_Z(1.0);
  vpMatrix L = s.interaction();  // 2×6 Interaction Matrix
  // v_c = -lambda * L^+ * (s - s*)
  ```

### 情境題 C：Eye-in-Hand vs Eye-to-Hand 配置
- **Eye-in-Hand**（相機在末端執行器）：
  - 相機跟機械臂移動
  - 最後 5cm 高解析度局部視野，避遮擋
  - **缺點**：視野受限，大幅運動目標易丟失
- **Eye-to-Hand**（相機固定）：
  - 全局視野穩定監控工作台
  - **缺點**：機械臂去抓時自身連桿遮擋相機
- **Hand-Eye Calibration `AX = XB`**：
  - A = 機械臂末端兩時刻相對運動（FK 讀取）
  - B = 相機兩時刻觀測標定板的相對運動（PnP 解算）
  - X = 未知固定剛體變換
    - Eye-in-Hand: `X = T_ee_cam`
    - Eye-to-Hand: `X = T_base_cam`
- **經典算法**：
  - **Tsai-Lenz**：軸角分離先求 R_x，再最小二乘求 t_x（快但誤差累積）
  - **Park-Martin**：基於李群 SE(3) 聯合優化
- **Hybrid 架構（工業標配）**：
  - **全局相機 Eye-to-Hand 粗定位**（6D Pose）
  - 機械臂移到目標上方 → **腕部相機 Eye-in-Hand 高頻 IBVS 精操作插入**
  - 平台：UR/Franka + Intel RealSense / Photoneo
- **「標定誤差 = VS 精度天花板」答題**：
  - 視覺伺服算的永遠是「相機座標系」速度 v_c
  - 必須透過手眼矩陣 `^e T_c` → 末端座標系 → 雅可比 J⁻¹ → 關節空間
  - 若 `^e T_c` 錯 5mm，相機以為對齊 → 實際夾爪始終偏 5mm
  - **系統性偏差無法透過視覺閉環本身消除**
  - 部署前必做 MoveIt! 20+ 點嚴格 Hand-Eye Calibration
- **OpenCV 求解**：
  ```python
  R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
      R_gripper2base, t_gripper2base,
      R_target2cam, t_target2cam,
      method=cv2.CALIB_HAND_EYE_TSAI  # or CALIB_HAND_EYE_PARK
  )
  ```

### 面試 talking points（Q1）
1. **PBVS 3D 直線 vs IBVS 標定魯棒**：場景選型的核心權衡
2. **Chaumette Conundrum 後退陷阱**：分辨「背公式」vs「懂陷阱」
3. **Image Jacobian 平移 ∝ 1/Z 旋轉無關 Z**：物理直覺必備
4. **4 點非共線解 Cylinder Ambiguity**：IBVS 最小配置的幾何推理
5. **2.5D VS 旋轉 3D + 平移 2D 解耦**：Hybrid 融合的優雅設計
6. **Eye-in-Hand + Eye-to-Hand Hybrid**：工業真實標配
7. **手眼標定 = 精度天花板**：系統性偏差的本質答案

## Q2 補強（Dynamic VS + Uncertainty/Occlusion + Modern VLA）

### 情境題 D：Dynamic Visual Servoing 動態目標追蹤
- **靜態 VS 失敗的誤差動態**：
  - 目標靜止時 `ṡ = L_s·v_c`
  - 目標移動時 `ṡ = L_s·v_c + ∂s/∂t` 多了第二項
  - 靜態控制律下機器人**永遠落後一步（steady-state tracking error）= 跑步機跟不上**
- **Feedforward Compensation 前饋補償**：
  - `v_c = -λ·L_s⁺·(s - s*) + L_s⁺·ṡ̂_target`
  - 物理意義：P 控制拉誤差 + **前饋讓相機主動以目標像素速度移動 → 相對速度為零**
- **Predictive VS (MPC)**：
  - 前饋需「現在」的速度，視覺延遲讓前饋也滯後
  - **轉化為 MPC**：Kalman filter 估目標狀態 + 預測未來 N 步軌跡 ŝ_{k+i|k}
  - 求解一段最優相機軌跡讓誤差總和最小
- **Learning-based Moving VS**：RL / Diffusion 直接學「追移動目標」，輸入連續圖像幀輸出帶預測性質的關節速度
- **Time Synchronization 致命性**（面試必答）：
  - 靜態 VS 晚 50ms 反應慢一點
  - 動態追蹤必須 **PTP (IEEE 1588) 微秒級硬體時間對齊**（相機曝光時間戳 + IMU + 編碼器）
  - 沒對齊的前饋 ṡ̂_target 帶相位偏差 → **前饋變正回饋擾動 → 劇烈震盪發散**
- **平台**：SpaceX Starship Catch Tower 筷子捕捉助推器、Tesla FSD 跟車、Amazon Kiva picking
- **Python 前饋實作**：
  ```python
  s_dot_target_est = target_kf.predict_velocity(s_current)
  L_s_pinv = np.linalg.pinv(L_s)
  v_feedback = -lambda_gain * L_s_pinv @ (s_current - s_target)
  v_feedforward = L_s_pinv @ s_dot_target_est
  v_camera_cmd = v_feedback + v_feedforward
  ```

### 情境題 E：VS under Uncertainty（雜訊/遮擋/失焦）
- **真實場景視覺災難**：
  - **運動模糊 (Motion blur)** 高頻運動造成
  - 曝光過度/不足
  - Eye-in-Hand 最後 5cm 失焦 (Defocus) + 夾爪自遮擋
  - 這些讓 L_s 秩虧（Rank Deficient）→ 系統崩潰
- **Robust Feature Tracking**：
  - 不依賴單幀檢測
  - **KLT 光流跟蹤 + DeepSORT / ByteTrack 多目標跟蹤**
  - 一幀被反光掩蓋 → 追蹤器依上幀動量維持特徵生命週期
- **Kalman Filter + VS + IMU 融合**（保命核心）：
  - 視覺特徵誤差放入 EKF 觀測更新
  - 機械臂 IMU/FK 作狀態預測
  - 相機瞬間完全遮擋 → **EKF 依運動學模型盲推 (Blind dead-reckoning)** 維持穩定控制輸出
- **Occlusion 恢復**：
  - Particle Filter 或 Re-identification 模型
  - 遮擋結束迅速找回目標
- **安全 Fallback**（特徵丟失 > 50%）：
  - **絕對不能讓機械臂繼續盲動**
  - 工業防護機制：**軟煞停 → 沿 Z 軸光軸緩慢後退拉開距離 → 重新擴大 FOV 全域特徵搜尋**
- **「穩定大於精度」答題**：
  - 傳統極限解算 → 強光反光/器械遮擋時雅可比奇異 → 扭矩突變撞毀設備
  - **EKF 濾波 + 動態特徵剔除**：犧牲微小靜態精度保 50% 遮擋/高頻震盪下不發散
  - 符 ISO 安全認證的產線級 Robustness
- **平台**：水下機器人 ROV（光線衰減折射）、達文西手術機器人（組織變形血液遮擋）
- **C++ Fallback 邏輯**：
  ```cpp
  float loss_ratio = 1.0 - (current_features.size() / expected_num_features);
  if (loss_ratio > 0.5) {
      robot.stop();
      retreat_cmd.linear.z = -0.05;  // 沿光軸後退 5cm
      robot.send_velocity(retreat_cmd);
      reinit_tracking();
  } else {
      vpColVector v_c = compute_ibvs_velocity(current_features);
      robot.send_velocity(v_c);
  }
  ```

### 情境題 F：Modern Learning-based VS 2024
- **End-to-End Visuomotor (ACT / Diffusion Policy / RT-2)**：
  - **徹底跳過特徵提取、標定、PnP、Image Jacobian**
  - 多視角 RGB pixel → CNN/Transformer 潛在表徵 → 直接回歸未來 N 步關節絕對位置
- **VLA Trajectory Output (RT-2 / OpenVLA / π0)**：
  - VLA 大模型控制頻率低（1-5 Hz）
  - **不直出 1 kHz 馬達扭矩，輸出低頻高階 waypoints**
  - 底層仍依賴傳統阻抗控制或改進版 VS 高頻閉環跟蹤
- **Diff-VS (Differentiable VS)**：
  - 把特徵提取 + Jacobian 估計寫成 PyTorch 可微張量
  - 整個 VS 變成 NN 中一層 Layer → 端到端訓練
  - 網路自學「提什麼特徵 L_s 最好」
- **NeRF / 3D Gaussian Splatting + VS**：
  - NeRF/3DGS 隱式重建場景
  - VS 目標：**找一組相機位姿變化，使當前渲染隱式視圖與目標圖像 Photometric Error 最小**
  - 在隱式場直接梯度下降控制機械臂
- **Foundation Model 特徵 (DINOv2 / CLIP) + VS**：
  - 傳統 SIFT/ORB 受限幾何紋理
  - **DINOv2/CLIP 深層特徵圖**：語義不變性（光照劇變/物體變形仍穩）
  - 從**幾何對齊 → 語義對齊**
- **「VLA + 傳統 VS 混合必贏純 RL」答題**：
  - VLA「常識推理強，物理微操弱」
  - 純端到端 VLA/RL 缺硬物理邊界保證 → 穿模/碰撞
  - 3 Hz 輸出無法應付動態高頻擾動
  - **「大腦慢思考，小腦快反射」**：
    - VLA 理解語言指令 + 空間語義級 Target Waypoints
    - 底層 500 Hz 傳統 VS / 阻抗「小腦」用數學剛性吸收局部誤差 + 絕對安全
  - 具身智能落地的唯一安全路徑
- **平台**：Google RT-2 / RT-X、Stanford ALOHA 雙手疊衣炒菜、Physical Intelligence π₀ 通用折紙盒
- **VLA Hybrid Python**：
  ```python
  with torch.no_grad():
      target_waypoints = vla_model.predict_action(rgb_image, instruction)
  next_target_pose = target_waypoints[0]
  for i in range(high_freq_steps):  # 底層 500 Hz 追蹤
      current_pose = robot_controller.get_pose()
      action_cmd = robot_controller.compute_impedance_control(current_pose, next_target_pose)
      robot.step(action_cmd)
  ```

### 面試 talking points（Q2）
8. **動態 VS 誤差動態方程多了 ∂s/∂t**：分辨「只背靜態 IBVS」vs「懂動態追蹤」
9. **Time Synchronization PTP 微秒對齊**：動態 VS 工程坑必備
10. **Kalman Filter 盲推保命**：遮擋恢復的工業標準
11. **特徵丟失 → Z 軸光軸後退重搜**：工業防護機制
12. **穩定 > 精度 的 ISO 認證導向**：產業落地思維
13. **DINOv2/CLIP 語義對齊 vs SIFT 幾何對齊**：2024 年前沿
14. **VLA 大腦慢思考 + 傳統 VS 小腦快反射**：端到端 RL 無法取代的核心理由
