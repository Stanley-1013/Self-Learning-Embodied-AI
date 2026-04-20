---
title: "SLAM 與未知環境地圖建構"
prerequisites: ["05-ros2-tf-and-tools"]
estimated_time: 45
difficulty: 4
tags: ["slam", "mapping", "localization", "lidar", "visual-slam", "loop-closure"]
sidebar_position: 17
---

# SLAM 與未知環境地圖建構

## 你將學到

- 能用兩句話講清楚 SLAM 解的是什麼問題、為什麼定位和建圖必須同時做，面試被問不含糊
- 遇到「機器人在長走廊漂移嚴重」或「地圖出現鬼影」這類情境，知道從 loop closure、退化場景診斷、感測器緊鬆耦合三個方向系統性排查
- 能完整比較 EKF-SLAM / FastSLAM / Graph-based SLAM / ORB-SLAM3 / LIO-SAM 五種主流方案的適用場景與取捨，判斷何時選哪個

## 核心概念

**精確定義**：**SLAM（Simultaneous Localization and Mapping）** 是機器人在未知環境中，同時估計自身位姿（localization）並建構環境地圖（mapping）的問題。數學上就是求聯合後驗機率 $P(\mathbf{x}, m \mid z, u)$，其中 $\mathbf{x}$ 是位姿序列、$m$ 是地圖、$z$ 是觀測、$u$ 是控制輸入。核心難點在於**雞生蛋問題**：精確定位需要地圖，而建構地圖又需要精確位姿，兩者互相依賴、必須聯合求解。

**一句話版本**：「SLAM = 機器人的海馬體 — 邊走邊畫地圖、邊看地圖修正自己位置，兩件事同時做。」

**前端 vs 後端架構**：
- **前端（Front-end / Odometry）**：特徵提取 → 幀間匹配 → 局部位姿估計。負責「邊走邊畫草圖」
- **後端（Back-end / Optimizer）**：全域圖優化，消除累積漂移。負責「回家把草圖攤在桌上對齊」

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：感測器原始資料 — LiDAR 點雲、相機影像、IMU 加速度/角速度、輪式里程計
- **輸出**：(1) 全域一致的環境地圖（occupancy grid / point cloud / OctoMap / mesh / NeRF）；(2) 機器人在地圖中的即時位姿估計
- **下游**：Nav2 路徑規劃（全域規劃器需要地圖）、Nav2 定位（用 SLAM 發布的位姿）、MoveIt 碰撞檢測（用 OctoMap 作為 3D 障礙物表示）
- **閉環節點**：屬於**感知層**的全域空間認知器。SLAM 發布 `map → odom` 的 TF 校正（**不直接發 `map → base_link`**，因為會跳變導致控制器震盪），疊加 `odom → base_link` 的里程計 TF，讓下游拿到漂移修正後的全域位姿

**九大核心概念速覽**：

| 概念 | 精確定義 |
|------|----------|
| **EKF-SLAM** | 用擴展卡爾曼濾波維護 $(3+2n) \times (3+2n)$ 共變異數矩陣，每次更新 $O(n^2)$，線性化截斷誤差大 → 現代少用 |
| **FastSLAM** | 粒子濾波 + Rao-Blackwellized 分解：每個粒子追蹤一條機器人軌跡，各自帶獨立地圖。粒子退化是主要瓶頸 |
| **Graph-based SLAM** | 節點 = 位姿、邊 = 觀測約束；後端求 $\min \sum \|e_{ij}\|^2_{\Omega_{ij}}$。工具：g2o / GTSAM / Ceres |
| **ORB-SLAM3** | Visual(-Inertial) SLAM 集大成：Tracking / Local Mapping / Loop Closing 三執行緒架構，支援單目/雙目/RGBD + IMU |
| **LIO-SAM** | LiDAR-Inertial 緊耦合：IMU 預積分 + 因子圖聯合優化，退化環境（走廊/隧道）魯棒性遠超純 LiDAR |
| **Loop Closure** | SLAM 的靈魂 — 偵測「回到走過的地方」並加入約束，是唯一能消除全域漂移的機制。方法：DBoW3（視覺）/ Scan Context（LiDAR） |
| **Occupancy Grid** | 2D 柵格地圖，每格存佔據機率。Nav2 標準輸入格式 |
| **OctoMap** | 八叉樹 3D 地圖，多解析度壓縮空間，MoveIt 碰撞檢測首選 |
| **NeRF / 3DGS Map** | 神經輻射場 / 3D Gaussian Splatting 地圖表示，能渲染新視角，代表 SLAM + 深度學習融合前沿 |

**最少夠用的數學**：

1. **EKF 預測-更新**（理解不確定性如何傳播）：

$$
\hat{\mathbf{x}}_{t|t-1} = f(\hat{\mathbf{x}}_{t-1}, u_t), \quad P_{t|t-1} = F_t P_{t-1} F_t^T + Q_t
$$

**物理意義**：每走一步，用運動模型預測新位姿 $\hat{\mathbf{x}}$，同時共變異數 $P$ 必定因噪聲 $Q$ 而**放大** — 不確定性只會隨里程計累積增長，永遠不會自動縮小。觀測更新是唯一能**壓回**不確定性的機制。

2. **Graph-based SLAM 最小化目標**（現代主流做法）：

$$
\mathbf{x}^* = \arg\min_{\mathbf{x}} \sum_{(i,j) \in \mathcal{E}} \| e_{ij}(\mathbf{x}_i, \mathbf{x}_j) \|^2_{\Omega_{ij}}
$$

**物理意義**：把每個位姿看成節點、觀測看成彈簧連接的邊，$\Omega_{ij}$ 是彈簧的剛度（信息矩陣 = 共變異數逆）。優化就是找讓**整個彈簧網路勢能最低**的位姿組態。Loop closure 就是加一條「起點到終點」的強力彈簧，把整張圖拉回一致。

3. **ICP（Iterative Closest Point）**（LiDAR scan matching 核心）：

$$
\min_{R, t} \sum_{k} \| R \mathbf{p}_k + t - \mathbf{q}_k \|^2
$$

**物理意義**：找旋轉 $R$ 和平移 $t$ 讓兩幀點雲最大程度重合。上式是 **point-to-point ICP**；實務 LiDAR SLAM（LOAM / LIO-SAM / FAST-LIO2）多用 **point-to-plane** 變體（將殘差投影到局部平面法向），在平面豐富場景收斂更快、更魯棒。**陷阱**：初值差會掉進局部最優 → 需要 IMU 或里程計提供好的初始猜測。NDT（Normal Distributions Transform）用常態分佈替代逐點配對，對初值更魯棒。

4. **IMU 預積分**（緊耦合的數學基石）：

$$
\Delta \tilde{R}_{ij}, \Delta \tilde{v}_{ij}, \Delta \tilde{p}_{ij} = \text{preintegrate}(a_k, \omega_k) \quad k \in [i, j]
$$

**物理意義**：把 $i$ 到 $j$ 之間所有 IMU 讀數打包成一個與**絕對位姿無關**的相對約束。圖優化調整位姿時，不需要對 IMU 量測重新積分 — 直接拿預積分量當邊約束。這是 LIO-SAM 等緊耦合系統的效率關鍵。

5. **信息矩陣 vs 共變異數矩陣**：

$$
\Omega = \Sigma^{-1}
$$

**物理意義**：信息矩陣 $\Omega$ 天然稀疏 — 只有直接觀測關係的節點之間才有非零元素 → 可用稀疏 Cholesky 分解高效求解。這是 Graph-based SLAM 能 scale 到城市級的計算基礎。

<details>
<summary>深入：Graph-based SLAM 完整推導 — Gauss-Newton 求解與 Hessian 稀疏性</summary>

**問題定義**：給定一組位姿節點 $\mathbf{x} = [\mathbf{x}_1, \mathbf{x}_2, \dots, \mathbf{x}_n]$ 和觀測約束集合 $\mathcal{E}$，每條邊 $(i,j)$ 帶有觀測 $z_{ij}$ 和信息矩陣 $\Omega_{ij}$，定義殘差：

$$
e_{ij}(\mathbf{x}) = z_{ij} \ominus h(\mathbf{x}_i, \mathbf{x}_j)
$$

其中 $h(\mathbf{x}_i, \mathbf{x}_j)$ 是從當前位姿估計預測的觀測值，$\ominus$ 是 manifold 上的減法。

**目標函數**：

$$
F(\mathbf{x}) = \sum_{(i,j) \in \mathcal{E}} e_{ij}^T \Omega_{ij} \, e_{ij}
$$

**Gauss-Newton 迭代**：將 $e_{ij}$ 在當前估計 $\breve{\mathbf{x}}$ 處一階泰勒展開：

$$
e_{ij}(\breve{\mathbf{x}} + \Delta \mathbf{x}) \approx e_{ij}(\breve{\mathbf{x}}) + J_{ij} \Delta \mathbf{x}
$$

代入目標函數並對 $\Delta \mathbf{x}$ 求導令其為零，得到法方程：

$$
H \Delta \mathbf{x}^* = -b
$$

其中：

$$
H = \sum_{(i,j)} J_{ij}^T \Omega_{ij} J_{ij}, \quad b = \sum_{(i,j)} J_{ij}^T \Omega_{ij} e_{ij}
$$

**Hessian 稀疏性的物理來源**：每條邊 $(i,j)$ 的 Jacobian $J_{ij}$ 只對節點 $i$ 和 $j$ 的位姿有非零偏導。因此 $J_{ij}^T \Omega_{ij} J_{ij}$ 只影響 $H$ 的 $(i,i)$、$(i,j)$、$(j,i)$、$(j,j)$ 四個分塊。在典型 SLAM 圖中，每個節點只和時序相鄰的節點、以及少量 loop closure 節點有邊 → $H$ 是帶狀稀疏矩陣，可用稀疏 Cholesky（如 CHOLMOD）在 $O(n)$ 或 $O(n^{1.5})$ 時間求解。

**iSAM2 增量式更新**：新約束加入時，用 Bayes Tree 只更新受影響的子圖分支，不需重算整個 $H$ → 支援即時增量式 SLAM。

**Levenberg-Marquardt vs Gauss-Newton**：LM 在 $H$ 對角線加阻尼 $\lambda I$，在遠離最優解時表現如梯度下降（穩定），靠近最優解時退化為 GN（快速收斂）。g2o 和 GTSAM 預設用 LM。

</details>

**常用工具鏈**：

| 層級 | 套件 | 功能 |
|------|------|------|
| 2D LiDAR SLAM | slam_toolbox / Cartographer | ROS 2 即插即用，輸出 occupancy grid |
| 3D LiDAR SLAM | LIO-SAM / FAST-LIO2 | 點雲地圖 + 高頻位姿，支援 IMU 緊耦合 |
| Visual SLAM | ORB-SLAM3 / VINS-Fusion | 單目/雙目/RGBD + IMU，輸出稀疏/稠密地圖 |
| 後端優化 | g2o / GTSAM / Ceres | 圖優化求解器，可獨立使用或整合進 SLAM 系統 |
| 地圖表示 | OctoMap / Open3D | 八叉樹 3D 地圖 / 點雲處理與網格重建 |
| 地圖伺服器 | nav2_map_server | 載入/儲存/提供占用格地圖給 Navigation Stack |
| 點雲處理 | PCL / Open3D | 濾波、降採樣、法向量估計、ICP 配準 |

## 直覺理解

**類比：黑暗迷宮裡摸牆 + 記步數**。想像你被丟進一個完全漆黑的迷宮，手上只有一支粉筆和一個計步器。你邊摸牆邊在地板上畫地圖（mapping），同時數步數推算自己的位置（localization）。問題是：步數會累積誤差（里程計漂移），畫的地圖也就跟著歪。但如果你走著走著**發現回到了剛才畫粉筆記號的地方**（loop closure），你就能一次修正所有累積的偏差 — 這就是 loop closure 為什麼是 SLAM 最關鍵的環節。

**前端 = 邊走邊畫草圖**、**後端 = 回家攤桌上對齊**、**Loop closure = 在迷宮裡走一圈發現回到起點，把地圖拉直**、**圖優化 = 用彈簧連接所有草圖節點，讓彈簧網路達到最低勢能態**。

**LiDAR vs Visual 選型直覺**：
- **Visual SLAM = 人的眼睛**：便宜、能辨識語義，但暗光/強光會瞎、白牆沒紋理會失效
- **LiDAR SLAM = 蝙蝠的超聲波**：不怕黑、精準到厘米，但貴、沒有顏色/語義資訊

**模擬器觀察**：在 Gazebo 裡用 TurtleBot 3 跑 Cartographer，讓機器人繞辦公室一圈。觀察三個關鍵時刻：(1) 開始直走時地圖很準；(2) 轉幾個彎後出現微小偏移；(3) 回到起點時 loop closure 觸發，整張地圖「咔」一聲對齊 — 牆壁不再重疊、邊緣變銳利。在 rviz2 同時開 `/map` topic 和 TF tree，觀察 `map → odom` 的 TF 在 loop closure 前後的**突然跳變** — 這就是為什麼 SLAM 不直接發 `map → base_link`，那個跳變會讓控制器抖起來。

**地圖品質的快速判斷**：看 `/map` 的牆壁邊緣 — 如果銳利且不重影，SLAM 品質好；如果牆壁「發胖」（雙線）或出現鬼影，代表外參校準或 loop closure 有問題。

## 實作連結

**三個典型工程場景**：

1. **倉庫 AMR 首次部署**：新倉庫沒有現成地圖。操作員用搖桿遙控 AMR 跑一圈（teleoperation），後台跑 slam_toolbox 建圖，存成 `.pgm` + `.yaml` 給 Nav2 用。之後切成 AMCL 純定位模式。關鍵：遙控時走慢（< 0.5 m/s）、確保每個區域都覆蓋、盡量走閉環路線觸發 loop closure。

2. **自駕車城市級建圖**：用 LIO-SAM（LiDAR + IMU 緊耦合）在城市道路跑數小時。GPS 作為全域約束加入因子圖後端，消除長距離漂移。動態物體（行人、車輛）用語義分割（RangeNet++）在前端剔除，避免「鬼影」建進地圖。

3. **動態環境持續運營**：工廠或商場環境每天都在變。需要 lifelong SLAM — 語義分割剔除動態物體 + RANSAC 幾何校驗 + OctoMap 機率衰減（舊觀測權重隨時間下降）自動「擦鬼影」。

**Code 骨架**（ROS 2 + slam_toolbox launch）：

```python
# launch file 骨架：啟動 slam_toolbox 線上建圖模式
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': True,
                'solver_plugin': 'solver_plugins::CeresSolver',  # 後端優化器
                'max_laser_range': 12.0,       # LiDAR 最大有效距離
                'minimum_travel_distance': 0.3, # 移動多少才新增節點
                'minimum_travel_heading': 0.3,  # 轉多少才新增節點
                'loop_search_maximum_distance': 3.0,  # loop closure 搜尋半徑
            }],
            remappings=[('/scan', '/lidar/scan')],
        ),
    ])
```

<details>
<summary>深入：完整 ROS 2 SLAM 系統 launch + 參數配置範例</summary>

**完整 launch file（slam_toolbox + Nav2 整合）**：

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # SLAM Toolbox — 線上非同步建圖模式
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,

            # === 後端優化 ===
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',  # 稀疏求解
            'ceres_preconditioner': 'SCHUR_JACOBI',

            # === 前端 scan matching ===
            'max_laser_range': 12.0,          # 超過此距離的點丟棄
            'minimum_travel_distance': 0.3,   # 移動多少公尺才新增節點
            'minimum_travel_heading': 0.3,    # 轉多少弧度才新增節點
            'scan_buffer_size': 10,           # scan matching 的候選 buffer
            'scan_buffer_maximum_scan_distance': 10.0,
            'use_scan_matching': True,
            'use_scan_barycenter': True,

            # === Loop closure ===
            'do_loop_closing': True,
            'loop_search_maximum_distance': 3.0,  # loop closure 搜尋半徑 (m)
            'loop_match_minimum_chain_size': 10,   # 至少匹配多少 scan 才算 loop
            'loop_match_maximum_variance_coarse': 3.0,

            # === 地圖設定 ===
            'resolution': 0.05,                # 5cm 柵格
            'map_update_interval': 5.0,        # 每 5 秒更新一次地圖
            'mode': 'mapping',                 # 'mapping' 或 'localization'

            # === TF ===
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'transform_publish_period': 0.02,  # 50 Hz TF 廣播
        }],
        remappings=[
            ('/scan', '/lidar/scan'),
        ],
    )

    # RViz2 可視化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('my_slam_pkg'), 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        slam_toolbox_node,
        rviz_node,
    ])
```

**LIO-SAM 的 params.yaml 關鍵參數**：

```yaml
lio_sam:
  ros__parameters:
    # === IMU ===
    imuAccNoise: 3.9939570888238808e-03     # 加速度計雜訊密度
    imuGyrNoise: 1.5636343949698187e-03     # 陀螺儀雜訊密度
    imuAccBiasN: 6.4356659353532566e-05     # 加速度計 bias random walk
    imuGyrBiasN: 3.5640318696367613e-05     # 陀螺儀 bias random walk
    imuGravity: 9.80511                      # 當地重力加速度

    # === LiDAR ===
    sensor: velodyne                         # velodyne / ouster / livox
    N_SCAN: 16                               # 光束數
    Horizon_SCAN: 1800                       # 水平掃描點數
    downsampleRate: 1

    # === 外參：LiDAR → IMU ===
    extrinsicTrans: [0.0, 0.0, 0.0]
    extrinsicRot: [1, 0, 0, 0, 1, 0, 0, 0, 1]

    # === Loop closure ===
    loopClosureEnableFlag: true
    loopClosureFrequency: 1.0                # Hz
    surroundingKeyframeSize: 50
    historyKeyframeSearchRadius: 15.0        # 搜尋半徑 (m)
    historyKeyframeSearchNum: 25
    historyKeyframeFitnessScore: 0.3         # ICP fitness 閾值（越小越嚴格）

    # === GPS（可選全域約束）===
    useGpsElevation: false
    gpsCovThreshold: 2.0                     # GPS 共變異數閾值
```

**存圖與載入**：

```bash
# 建圖完成後儲存
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: '/home/robot/maps/warehouse_v1'}}"

# 之後用 nav2_map_server 載入
ros2 run nav2_map_server map_server --ros-args \
  -p yaml_filename:=/home/robot/maps/warehouse_v1.yaml \
  -p use_sim_time:=false
```

</details>

## 常見誤解

1. **「SLAM 只有一種，用就對了」** — SLAM 是一整個問題框架，不是單一演算法。EKF-SLAM、FastSLAM、Graph-based、Visual、LiDAR-Inertial 的適用場景完全不同。**避開**：先搞清楚環境特性（室內/外、動態/靜態、光照條件、幾何複雜度）和硬體限制（感測器類型、算力預算），再選對應的 SLAM 系統。

2. **「有 GPS 就不需要 SLAM」** — GPS 在室內完全無訊號、在城市峽谷遮蔽嚴重、更新頻率低（1–10 Hz）且精度只有米級。SLAM 提供厘米級、100+ Hz 的即時位姿。**正確做法**：把 GPS 作為一元約束（unary factor）注入 SLAM 的因子圖後端，用 SLAM 提供局部高頻位姿、GPS 防止長距離漂移。

3. **「Loop closure 只是錦上添花」** — 沒有 loop closure 的 SLAM 本質上就是**純里程計**，累積漂移隨時間線性成長，跑久了地圖一定歪。Loop closure 是唯一能把全域漂移歸零的機制。**觀察**：在 rviz 看地圖 — 無 loop closure 時牆壁會出現重影、邊緣裂開。

4. **「SLAM 的位姿直接拿來當控制迴路的 setpoint」** — SLAM 發布的 `map → odom` TF 在 loop closure 觸發時會**瞬間跳變**。如果控制器直接用這個位姿做 feedback，機器人會突然猛轉或急煞，燒馬達、撞東西。**正確做法**：控制器用平滑的 `odom → base_link`；SLAM 的 `map → odom` 只用來做全域路徑規劃和地圖對齊。

5. **「Visual SLAM 在任何環境都能用」** — Visual SLAM 依賴紋理梯度提取特徵。暗光、強光直射、白牆、重複紋理（倉庫貨架）這些場景，視覺特徵會大量丟失，追蹤直接失敗。**避開**：暗光或紋理缺失環境必須加 IMU 緊耦合（VINS-Fusion），或直接換 LiDAR。

## 練習題

<details>
<summary>Q1（中）：老闆要你替倉庫 AGV 選 SLAM 方案，倉庫有長走廊、環境光照穩定但紋理單調（白牆 + 金屬貨架），預算有限。你怎麼選？</summary>

**完整推理鏈**：

1. **排除 Visual SLAM**：白牆 + 金屬貨架紋理單調，ORB/FAST 特徵點提取困難，追蹤會頻繁丟失。即使加 IMU 緊耦合（VINS-Fusion），白牆段的視覺約束太弱，純靠 IMU dead reckoning 撐不了多長
2. **選 2D LiDAR SLAM**：倉庫是平面環境，2D LiDAR（如 RPLIDAR A3，約 500 美元）成本低、不需要 GPU、能提供厘米級精度。LiDAR 用幾何特徵（牆壁/貨架邊緣），不依賴光照或紋理
3. **具體方案**：slam_toolbox（ROS 2 原生支援、參數調整方便）或 Cartographer（Google 維護、支援子地圖架構）
4. **長走廊退化場景對策**：走廊兩面平行牆沿軸向無約束 → 加輪式里程計作為額外約束；或放反光板/AprilTag 作為人工路標；建圖時走閉環路線讓 loop closure 消漂移
5. **部署流程**：先 teleoperation 建圖 → 儲存 `.pgm` + `.yaml` → 切 AMCL 純定位模式日常運營

**面試官想聽到**：從環境特性（紋理、光照、幾何）系統性排除不適合的方案，而非上來就推薦最新最炫的系統；知道長走廊退化的具體對策。

</details>

<details>
<summary>Q2（中-難）：長走廊建圖失敗 — 地圖嚴重扭曲、走廊被壓縮或拉長，甚至出現重影。你怎麼診斷和修？</summary>

**完整推理鏈**：

1. **診斷：幾何退化（geometric degeneracy）**：長走廊兩面平行牆 + 天花板 = 沿走廊軸向幾乎零約束。LiDAR scan matching 在走廊方向的位移無法靠 scan 形狀區分，殘差函數在軸向是平坦的 → 優化器沿軸向自由漂移
2. **驗證方法**：
   - 開 rviz2 看 `/map`，走廊段牆壁是否出現雙線或扭曲
   - 查看 scan matching 的 fitness score，走廊段應該明顯偏低
   - 用 `tf2_echo map odom` 觀察 TF 是否在走廊段持續漂移
3. **解法（短期）**：
   - **加 IMU 緊耦合**：上 LIO-SAM 或 FAST-LIO2，IMU 預積分在**短時段內**提供軸向位移先驗 + 陀螺儀防偏航漂移；但 IMU 本身（加速度計二次積分有 bias 漂移）**不能獨立解除長期軸向退化**，仍需 loop closure / UWB / 人工路標做絕對約束
   - **加輪式里程計**：作為額外的位移約束注入因子圖
   - **降速**：機器人速度從 1 m/s 降到 0.3 m/s，讓每幀 scan 重疊率更高
4. **解法（長期）**：
   - 走廊兩端或中間放反光板/AprilTag 作為人工路標
   - 使用 3D LiDAR 利用天花板/地板的幾何約束（2D LiDAR 看不到）
   - 建圖時走閉環路線（走到底回頭再走一次）
5. **本質理解**：SLAM 的精度取決於環境提供的**幾何約束豐富度**。退化不是 SLAM 的 bug，是環境幾何的物理限制 → 解法是**多模態融合**（用不同感測器補不同方向的約束）

**面試官想聽到**：「幾何退化」這個詞要能自然說出來，並理解它的物理含義；解法是分層的（短期急救 vs 長期工程化），不是只說「換更好的 LiDAR」。

</details>

<details>
<summary>Q3（難）：大工廠 500 m 路線跑完，累積漂移 1 m 以上，loop closure 也沒觸發。怎麼辦？</summary>

**完整推理鏈**：

1. **診斷 loop closure 為何失敗**：
   - 可能路線是開環的（沒有回到走過的地方）
   - 即使回到了，場景描述子（DBoW3 / Scan Context）沒能匹配成功 — 可能是動態物體改變了場景外觀、或描述子參數（搜尋半徑、閾值）太嚴格
2. **解法一：強化 loop closure 偵測**：
   - LiDAR 系統改用 Scan Context（旋轉不變的場景描述子），對大場景的 recall 遠高於純 ICP fitness
   - 放寬 `loop_search_maximum_distance` 和 `historyKeyframeSearchRadius`
   - 加幾何驗證：ICP fitness score < 0.3 才接受，防誤匹配
3. **解法二：注入全域先驗約束**：
   - 加 UWB 定位信標（±30 cm 精度），作為一元約束注入因子圖
   - 在關鍵位置放 AprilTag / 反光板作為人工路標
   - 有 GPS 訊號的區域把 GPS 也注入
4. **解法三：分層架構**：
   - 每 50–100 m 切子地圖（submap），子地圖內部精細優化
   - 子地圖之間用 loop closure + 全域先驗做稀疏圖優化
   - 後端用增量式 iSAM2，新約束來了只更新受影響的子圖
5. **部署驗收**：跑完全程後，把起終點的位姿和 GPS/已知座標比對，漂移 < 地圖解析度（如 5 cm）才算通過

**面試官想聽到**：不只說「加 loop closure」，而是系統性地分析 loop closure 為何失敗 + 提出多層次解法（偵測強化、全域先驗、分層架構）。

</details>

<details>
<summary>Q4（難）：動態環境（工廠有人走動、叉車移動）跑 SLAM，地圖會出現「鬼影」（不存在的牆或障礙物），怎麼系統性解決？</summary>

**完整推理鏈**：

1. **根因**：SLAM 把動態物體（行人、叉車）的觀測建進地圖，物體移走後地圖還保留舊觀測 → 出現鬼影
2. **前端剔除（根治）**：
   - **語義分割**：用 RangeNet++（LiDAR）或 Mask R-CNN（Camera）在前端就把人和車的點雲/像素標記為動態物，不送進 SLAM 後端
   - **幾何校驗**：用 RANSAC 檢查 scan matching 的 inlier 比例，動態物體的點是 outlier → 自動剔除
3. **後端衰減（持續維護）**：
   - **OctoMap 機率衰減**：每個 voxel 的佔據機率隨時間衰減（miss 觀測降低機率），動態物移走後 voxel 自動歸零 → 「擦鬼影」
   - 設定 `sensor_model.miss` 參數讓空觀測的降權速度適當（太快會把靜態物也擦掉）
4. **地圖分層**：把靜態地圖（牆壁、固定設備）和動態層（costmap 的 obstacle layer）分開管理，SLAM 只維護靜態層
5. **驗收**：讓人在建圖區域走動 10 分鐘，然後人離開，檢查地圖是否在 30 秒內自動清除鬼影

**面試官想聽到**：從前端剔除（根治）和後端衰減（持續維護）兩路並進；知道 OctoMap 的機率衰減機制；理解靜態/動態分層的架構思維。

</details>

## 面試角度

1. **前後端分離架構** — 區分「只會用 SLAM 套件」和「理解系統架構」的分水嶺。**帶出**：「SLAM 系統一定要看成前端 + 後端。前端負責幀間匹配給出局部位姿，可以換（ICP / ORB / 直接法），後端負責全域優化消漂移，也可以換（g2o / GTSAM / Ceres）。這個解耦讓你能獨立替換和調優每個模組。」

2. **因子圖的稀疏性是 SLAM 能 scale 的關鍵** — 展示對計算底層的理解，不只是用 API。**帶出**：「Graph-based SLAM 的 Hessian 矩陣天然稀疏 — 只有直接觀測關係的節點之間有非零元素。這讓稀疏 Cholesky 分解把 $O(n^3)$ 的稠密求解壓到接近 $O(n)$，這是 SLAM 能跑到城市級的計算基礎。」

3. **Loop closure 生死攸關** — 證明你懂 SLAM 的核心困難，不是調參數就能解決。**帶出**：「沒有 loop closure 的 SLAM 就是純里程計，漂移只會越來越大。但錯誤的 loop closure 比沒有更可怕 — 會把整張圖拉歪。所以我一定會加幾何驗證（ICP fitness 閾值）和時序一致性檢查，不能光靠特徵相似就接受。」

4. **緊耦合 vs 鬆耦合選型** — 展示多模態融合的深入理解。**帶出**：「鬆耦合是 LiDAR 和 IMU 各自算完再融合，容易實作但退化環境會崩；緊耦合是把 IMU 預積分直接嵌入因子圖聯合優化，像 LIO-SAM，在走廊/隧道等退化場景穩定性完全不同級別。代價是實作複雜度和 IMU 校準要求都高很多。」

5. **從一次性建圖到 lifelong SLAM** — 展示前瞻視野。**帶出**：「靜態地圖只是起點。真實部署需要 lifelong SLAM — 機器人每天跑同一條路線，地圖要能增量更新、舊資訊要用機率衰減自動淡出。這和一次性建圖的思維完全不同，是工業落地的核心挑戰。」

## 延伸閱讀

- **《具身智能算法工程師 面試題》Ch10.4 SLAM 與建圖、Ch2.4 狀態估計** — SLAM 面試高頻考點全覆蓋，含 EKF / Graph-based / Visual SLAM 比較，直接對應面試場景
- **高翔《視覺 SLAM 十四講》** — 中文 SLAM 聖經，從李群/李代數到因子圖完整走一遍，適合建立數學框架後回來翻閱具體推導
- **Shan & Englot, LIO-SAM 論文（IROS 2020）** — LiDAR-Inertial 緊耦合的標竿實作，讀完理解 IMU 預積分 + 因子圖怎麼結合，以及為什麼退化環境需要緊耦合
- **Campos et al., ORB-SLAM3 論文（IEEE TRO 2021）** — Visual-Inertial SLAM 集大成之作，三執行緒架構 + 多地圖系統 + Atlas，理解視覺 SLAM 的工程複雜度
