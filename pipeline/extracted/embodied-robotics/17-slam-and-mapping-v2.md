# Extracted v2: SLAM 與未知環境地圖建構 (5 次專屬查詢)
<!-- session: 8323664b, Q1-Q5 -->

## Q1 — 精確定義 + 閉環定位
- SLAM核心：P(x,m|z,u) 聯合後驗機率；定位依賴地圖、建圖依賴定位
- 前端(里程計：特徵提取+匹配+局部位姿) vs 後端(全域圖最佳化消漂移)
- EKF-SLAM：O(n²)共變異數矩陣、線性化截斷誤差→現代少用
- FastSLAM：粒子濾波+Rao-Blackwellized分解；粒子退化問題
- Graph-based：節點=位姿、邊=觀測約束；min Σ‖eij‖²_Ωij；g2o/GTSAM/Ceres
- ORB-SLAM3：Tracking/Local Mapping/Loop Closing 三執行緒；特徵法 vs 直接法
- LiDAR SLAM：ICP/NDT 配準；LIO-SAM 緊耦合(IMU預積分+因子圖)
- Loop Closure：SLAM 的靈魂；BoW(DBoW3)/Scan Context
- 地圖表示：Occupancy Grid/Point Cloud/OctoMap/Mesh/NeRF
- 閉環：SLAM 發布 map→odom TF(不直接 map→base_link 因為會跳變)
- 下游：Nav2 global(地圖)/local(位姿)/MoveIt(OctoMap碰撞)
- 一句話：「SLAM = 機器人的海馬體」

## Q2 — 核心數學 / API
- EKF 預測-更新：x̂=f(x,u), P=FPF^T+Q → 不確定性必定隨噪聲累積放大
- Graph SLAM：min_X Σ ‖eij‖²_Ωij → 彈簧網路拉到最小位能態
- ICP：min_{R,t} Σ‖Rp+t-q‖² → 初值差會局部最優；NDT 用常態分佈替代
- IMU 預積分：打包為與絕對位姿無關的相對約束→圖最佳化時不需重積分
- 信息矩陣 vs 共變異數：信息矩陣天然稀疏→稀疏代數加速
- 工具鏈：slam_toolbox/Cartographer/ORB-SLAM3/LIO-SAM/rtabmap/OctoMap/g2o/GTSAM/Ceres/PCL/Open3D
- 效能數字：EKF O(n²)；A* vs Dijkstra 60-80% 剪枝；SIFT 128維；VIO 時間誤差±3ms

## Q3 — 直覺 / 視覺 / 誤解
- 類比：前端=邊走邊畫草圖、後端=回家攤桌上對齊；閉環=迷宮走一圈發現回起點拉直地圖；圖最佳化=彈簧網路平衡
- LiDAR vs Visual 選型：Visual=人眼(暗光失效/便宜)、LiDAR=蝙蝠(不怕黑/精準/貴)
- 觀察：無 loop closure → 重影/牆壁裂開；品質看 /map 邊緣清晰度；外參錯2度→牆壁發胖+走過路對不上
- 5 誤解(詳細)：只有一種SLAM/GPS替代/loop closure不重要/SLAM直接當控制位姿(跳變燒馬達)/Visual暗光也行/地圖不更新

## Q4 — 4 情境題(詳細推理鏈)
1. 倉庫AGV選型 → 2D LiDAR + Cartographer/slam_toolbox(走廊紋理缺失排除Visual)
2. 長走廊建圖失敗(扭曲重影) → 退化場景診斷 → LIO-SAM 緊耦合(IMU撐盲區)
3. 大工廠500m漂移1m → 閉環檢測強化(Scan Context) + 因子圖加全局先驗(UWB/人工地標)
4. 動態環境持續更新 → 語意分割剔除動態物 + RANSAC幾何校驗 + OctoMap機率衰減擦鬼影

## Q5 — 5 Talking points / 5 延伸 / 4 閱讀
- 5 points：前後端分離/因子圖稀疏性/閉環生死攸關/緊鬆耦合選型/動態環境Lifelong
- 5 延伸：NeRF-SLAM/Gaussian Splatting/BA vs PGO/Scan Context+DBoW3/Factor Graph+iSAM2/Semantic SLAM
- 4 閱讀：《面試題》Ch10.4+2.4 → 《視覺SLAM十四講》→ LIO-SAM論文 → ORB-SLAM3論文
