# Extracted: 基礎路徑規劃（A*, RRT, APF）
<!-- session: fe23a257, Q20 -->

## 定義
- Path vs Motion vs Trajectory planning：幾何路線 / 統稱 / 加時間速度約束
- C-space：將機器人抽象為點，碰撞檢測簡化為點在區域內外
- A*：f=g+h，admissible heuristic → 全局最優；高維指數爆炸
- Dijkstra：h=0 退化版 A*，無方向均勻擴展
- RRT：random sample→nearest→steer→collision check；機率完備但非最優
- RRT*：rewiring + 父節點重選 → 漸近最優
- APF：吸引力+排斥力梯度下降；局部最小值（U型障礙物）
- PRM：離線撒點建圖+線上查詢；multi-query 靜態場景

## 閉環
- 規劃前端（戰略導航員）
- 輸入：起點+目標+環境地圖；輸出：waypoints；下游：trajectory optimizer → controller
- 一句話：「路徑規劃是多維迷宮中尋找安全通道的幾何引擎」

## 直覺 + 誤解
- A*=GPS 導航、RRT=探險家隨機探索、APF=磁鐵吸引排斥
- 4 誤解：RRT 非最短(要 RRT*)、APF 有局部最小、A* 高維崩(curse of dim)、path≠trajectory

## 3 情境題
1. 倉庫 AGV A→B 靜態貨架 → A*（低維+最優保證）
2. 7-DoF 雜亂桌面抓取 → RRT*/PRM（高維 → 採樣法）
3. Nav2 global planner 繞路 → 調 h(n) 權重 + 降 inflation_radius + cost scaling

## Talking points / 延伸 / 閱讀
- 3 points：維度決定演算法、啟發函數工程、分層規劃(全局A*+局部APF/DWA)
- 延伸：JPS(A*加速)、D* Lite(增量式)、Informed RRT*/RRT-Connect
- 閱讀：《面試題》Ch1.3+4.2、Ch6.4+8.4
