# Extracted: 動力學建模（牛頓-歐拉與拉格朗日）
<!-- session: fe23a257, Q19 -->

## 定義
- Dynamics vs Kinematics：力/力矩 vs 幾何
- Newton-Euler：遞迴 O(n)，正推速度加速度/反推力矩；即時控制首選
- Lagrangian：能量法 T-V，符號推導佳；O(n³~n⁴)，離線分析用
- 標準形式：M(q)q̈ + C(q,q̇)q̇ + g(q) = τ
  - M：慣性矩陣（正定對稱，condition number 影響數值穩定）
  - C：科氏+離心力（Christoffel 符號，Ṁ-2C 斜對稱 → Lyapunov 穩定性）
  - g：重力補償
- Forward dynamics：τ→q̈（模擬器核心）
- Inverse dynamics：(q,q̇,q̈)→τ（CTC 前饋 / 阻抗控制）

## 閉環
- Inverse dynamics = CTC 前饋（抵消 90% 慣性+科氏+重力干擾，剩 10% 給 PD）
- Forward dynamics = 模擬器引擎（MuJoCo/PyBullet 積分）
- 一句話：「動力學模型是機器人的肌肉骨骼說明書，讓模擬器預言未來、控制器精準下力矩」

## 直覺 + 誤解
- 類比：運動學=地圖導航、動力學=踩油門（上坡踩深=g、轉彎快=C、車重=M）
- 4 誤解：M(q)非常數(隨姿態變)、PID 不夠(高速需前饋)、Lagrangian 太慢(用 RNEA)、線性摩擦模型不夠(Stribeck/DOB)

## 3 情境題
1. CTC 參數不準 → 離線 System ID(激勵軌跡+WLS) + 線上 DOB/自適應
2. RL sim→real 力矩錯 → 查重力補償/非線性摩擦/執行器延遲 → Domain Randomization
3. 1kHz 阻抗控制 inverse dynamics → RNEA O(n) + Pinocchio/CasADi 符號代碼生成

## Talking points / 延伸 / 閱讀
- 3 points：前饋+回饋解耦、RNEA O(n) 硬即時、DOB 抗未建模干擾
- 延伸：Featherstone Spatial Algebra、System Identification、DOB
- 閱讀：《面試題》Ch1.4、Lynch Modern Robotics Ch8
