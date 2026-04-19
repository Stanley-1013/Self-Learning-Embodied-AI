# Extracted v2: 力控制與阻抗控制 — 補強素材 (session 531c9d80, Q1/2)
## 補強情境題 A（Franka Panda 7軸全身阻抗+碰撞偵測）
- Step1: 殘差觀測 τ_external = τ_measured - τ_model；>15N 閾值判定碰撞（ISO/TS 15066）
- Step2: 分級反應狀態機 — 輕度→零力拖動 / 中度→柔順回彈(彈簧耗能) / 重度→Cat.0 急停
- Step3: **零空間分層阻抗** — 主任務高 K_task / 零空間 K_null≈0（手肘輕柔避開、手端精度不變）

## 補強情境題 B（精密 Peg-in-Hole ±0.05mm）
- Step1: 位置控接觸（Fz > 5N 停止下壓）
- Step2: **阿基米德螺旋探索**（XY 軌跡 + Z 恆定下壓力）；Fz 斷崖式下降=掉入
- Step3: 力位混合 — Z 力控 / XY 位控維持中心 / Yaw 柔順配合
- Step4: **自適應阻抗防卡阻** — 側向力異常→降 K 提 B，搖晃適應

## 補強面試 talking points
1. **關節力矩感測器 vs F/T sensor**：關節=全身碰撞偵測（受摩擦干擾、末端精度差）vs F/T=末端直測（>1kHz 頻寬、無傳動誤差）；應用場景選型
2. **Variable Impedance Learning**：固定 M/D/K 遇未知剛度環境震盪；RL（SAC/PPO）State=接觸力, Action=K,D → 剛柔並濟
