# Extracted v2: MPC — 補強素材 (session 531c9d80, Q1/2)
## 補強情境題 A（四足 SRBD 降階 + WBC 分層）
- 問題：完整浮基動力學 O(n⁴) 無法 100Hz+ 即時
- **SRBD 降階**（Single Rigid Body Dynamics）— 所有質量集中質心，凸 QP 幾 ms 解出質心軌跡+足端 GRF
- **WBC 底層 1kHz**（Whole-Body Control）— 接收 MPC 的 GRF+質心加速度，用完整動力學處理關節極限/奇異/力矩分配
- MPC「看得遠」（避障預測）+ WBC「走得穩」（精確力矩）

## 補強情境題 B（轉彎振盪系統性除錯）
- Step1: **線性化誤差** — Yaw 角變化大，sin(θ)≈θ 假設失效→提高線性化更新頻率或切 NMPC
- Step2: **預測時域盲區** — N 太短看不到彎道→動態拉長 N 或加入前瞻軌跡引導
- Step3: **約束衝突** — 摩擦錐/關節速度限幅太緊→引入 Slack Variables 軟化硬約束

## 補強面試 talking points
1. **MPC vs LQR**：LQR 無約束時足夠（Riccati 離線解、成本低）；LQR 無法處理物理硬約束→必須 MPC 線上 QP
2. **Learning MPC / Neural MPC**：
   - Warm Start：NN 預測初值砍迭代次數
   - Behavioral Cloning：NN 直接模仿 MPC 輸出，O(n³)→O(1) 微秒部署
