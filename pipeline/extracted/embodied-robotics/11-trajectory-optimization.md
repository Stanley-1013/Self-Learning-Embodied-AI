# Extracted: 軌跡最佳化（時間、能量、平滑度）
<!-- session: fe23a257, Q21 -->

## 定義
- Trajectory vs Path：path=幾何路線、trajectory=加時間參數化+物理約束
- 時間最優：min ∫dt, 約束 v/a/jerk ≤ limit → Bang-Bang 控制
- 能量最優：min ∫τ²dt → 連續加速度、降馬達發熱
- 平滑度：min jerk(三階)/snap(四階) → 抑制振動、高頻共振
- 三次多項式：滿足 pos+vel 邊界；五次：再加 acc 邊界
- B-spline/Bézier：局部控制性、避免龍格震盪
- 梯形速度：acc-cruise-dec 三段，但 jerk 突變
- S 曲線(7段)：jerk 連續、消除剛性衝擊

## 閉環
- 規劃→控制的橋梁
- 輸入：waypoints + 動力學限制；輸出：q(t),q̇(t),q̈(t)；下游：controller feedforward
- 一句話：「軌跡最佳化給幾何路徑注入時間與物理法則的靈魂」

## 直覺 + 誤解
- 類比：path=地圖畫路線、trajectory=決定哪踩油門哪踩煞車且不讓乘客暈車
- 4 誤解：時間+能量不能同時極致(Pareto)、多項式階數非越高越好(龍格震盪)、僅運動學限幅不夠(需動力學約束)、笛卡爾直線≠關節空間平滑(奇異點爆速度)

## 3 情境題
1. PTP 搬運最快不超馬達力矩 → TOPP-RA(相平面速度邊界搜尋)
2. 直線焊接末端抖 → min jerk 積分權重 + S 曲線/B-spline + 檢查奇異點
3. Nav2 DWA 轉彎太急 → 降 goal heading 權重 + 降 max_vel_theta/max_accel_theta

## Talking points / 延伸 / 閱讀
- 3 points：Pareto 多目標權衡、B-spline 局部控制、笛卡爾空間奇異陷阱(加 Jacobian condition number 約束)
- 延伸：TOPP-RA、Minimum Snap、Convexification(非凸→QP)
- 閱讀：《面試題》Ch5.1+5.3、Ch5.4 MPC
