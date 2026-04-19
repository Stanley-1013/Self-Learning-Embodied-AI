# Extracted: SLAM 與未知環境地圖建構 (session fe23a257 Q26-A)
- SLAM核心：定位+建圖互依賴；EKF/粒子濾波(早期)/Graph-based(g2o,主流)/Visual(ORB-SLAM3)
- 閉環：感知層全域空間認知器；發布 map→odom TF 消除里程計漂移
- 直覺：黑暗迷宮摸牆+記步；3誤解：只有一種/GPS替代/loop closure不重要
- 情境：走廊特徵少→多模態融合(IMU+輪式)/地圖偏→查TF跳變+外參/大場景漂移→閉環檢測+GPS約束
- Points：緊鬆耦合/動態物剔除/邊緣算力；延伸：NeRF-SLAM/BA/Scan Context；閱讀：Ch10.4+Ch2.4
