# Extracted: 動態環境避障與即時重規劃
<!-- session: fe23a257, Q22 -->

## 定義
- DWA：速度空間(v,ω)採樣+代價函數評估，適合差速/全向移動機器人
- TEB：時空軌跡彈性橡皮筋，C²連續，支援非完整約束(阿克曼)
- MPPI：sampling-based MPC，GPU 平行大量軌跡，適合非線性系統
- Local vs Global：Global=低頻(1Hz) A*/Dijkstra；Local=高頻(20-50Hz) DWA/TEB
- Costmap：voxel layer(3D→體素)、inflation layer(膨脹安全梯度)
- 反應式避障：VO/RVO 構造碰撞錐排除危險速度，比規劃式快

## 閉環
- 高頻規劃前端+即時反應層
- 分層：global A* + local DWA → 算力+反應速度平衡
- 一句話：「動態避障是機器人的小腦反射神經」

## 直覺 + 誤解
- A*=GPS導航、RRT=探險家、APF=磁鐵
- 4 誤解：RRT 非最短(RRT*)、APF 有局部最小(U型障礙)、A* 高維崩、path≠trajectory

## 3 情境題
1. DWA 人群中急停繞路 → 缺動態預測 → KF/LSTM 預測行人+寫入時空 Costmap → 切 TEB/VO
2. 機械臂動態抓取 MoveIt 太慢 → 分層：低頻 RRT 參考+高頻 SDF+MPC/QP 即時避讓(10ms)
3. 多機器人交叉路口 → ORCA 分散式互惠避障 + 優先級仲裁(載貨優先/拍賣通行權)

## Talking points / 延伸 / 閱讀
- 3 points：分層規劃、動態預測整合 Costmap、維度決定演算法
- 延伸：ORCA、Informed RRT*、Control Barrier Functions(CBF)
- 閱讀：《面試題》Ch8.3 動態障礙檢測、Ch8.4 動態重規劃
