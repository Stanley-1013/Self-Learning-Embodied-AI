# Extracted: ROS 2 座標變換 (TF) 與常用工具
<!-- session: fe23a257, Q16 (all-in-one) -->

## 定義
- TF2：管理多座標系隨時間的位姿關係；vs TF1 = Buffer/Listener 解耦
- Frames：map(全域固定) / odom(連續但漂移) / base_link(機器人底座) / camera_link(感測器)
- Static TF：固定關係發一次（感測器外參）；Dynamic TF：持續高頻發布（里程計）
- Buffer+Listener：背景訂閱 /tf /tf_static → Buffer 提供 lookup 查詢
- Broadcaster+StaticBroadcaster：封裝 TransformStamped 發到 /tf 或 /tf_static
- URDF/Xacro：定義 link+joint 幾何 → robot_state_publisher 讀 URDF + joint_states → 自動廣播 TF tree

## 閉環定位
- TF = 全域時空座標統一平台
- 輸入：各感測器/演算法發布的 TransformStamped
- 輸出：任意兩 frame 的平移+旋轉
- 下游：SLAM、Nav2、ros2_control
- 一句話：「TF tree 就像幫每個零件和感測器貼上自帶相對座標與時間戳的 GPS 標籤」

## API 骨架
- lookup_transform(target, source, time) + timeout
- sendTransform(TransformStamped)
- 命名慣例：earth→map→odom→base_link→sensor_link (REP-105)

## 直覺 + 誤解
- 類比：TF tree = 俄羅斯套娃 / 關節接力賽
- 4 誤解：parent↔child 搞反、忘 static TF、time=now() 但 buffer 沒資料（用 TimePointZero）、URDF 閉環（tree 不可有環）

## 3 情境題
1. rviz 模型浮空 → 檢查 Fixed Frame + base_footprint→base_link Z offset + tf2_echo 驗證
2. 外部深度相機點雲轉 base_link → StaticBroadcaster + Buffer+Listener + lookup + doTransform
3. SLAM 地圖偏 5 度 → 檢查 map→odom→base_link 權責 + rqt_tf_tree 找重複發布 + URDF base→laser rpy 外參標定

## Talking points / 延伸 / 閱讀
- 3 points：四元數 vs 歐拉角（setRPY）、Buffer 共用降頻寬、lookup 包 try-catch
- 延伸：REP 105、tf2_geometry_msgs、tf2_ros::MessageFilter
- 閱讀：《ROS2》Ch5 TF 工具、Ch9.5.2 移動機器人座標框架、官方 TF2 Tutorials
