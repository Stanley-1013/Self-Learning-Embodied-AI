---
title: "ROS 2 座標變換 (TF) 與常用工具"
prerequisites: ["04-ros2-basic-communication"]
estimated_time: 45
difficulty: 3
tags: ["ros2", "tf2", "coordinate-transform", "urdf"]
sidebar_position: 5
---

# ROS 2 座標變換 (TF) 與常用工具

## 你將學到

- 能精確講出 TF2 的核心架構（Buffer/Listener + Broadcaster）以及 static vs dynamic TF 的差異與使用時機
- 遇到「rviz 裡模型浮空」或「點雲座標對不上」的情境，知道先用 `tf2_echo` / `rqt_tf_tree` 檢查 TF tree，再定位是 parent-child 搞反還是缺少 static TF
- 能在面試中用兩分鐘講清楚 REP-105 的 frame 層級（earth → map → odom → base_link → sensor_link）以及為什麼 map 和 odom 要分開

## 核心概念

### TF2 系統架構

**TF2（Transform Library 2）** 是 ROS 2 的座標變換基礎設施，負責管理多個座標系（frame）之間隨時間變化的位姿關係。相比 TF1，TF2 把 Buffer 與 Listener 解耦，支援跨語言共用、且可以自訂插值策略。

**一句話**：TF tree 就像幫機器人的每個零件和感測器貼上自帶相對座標與時間戳的 GPS 標籤 — 任何時刻都能查任意兩個零件之間的相對位姿。

### Frame 層級（REP-105）

| Frame | 意義 | 特性 |
|-------|------|------|
| `earth` | 地球固定座標（ECEF） | 多機器人系統使用，用來關聯多個 `map` frame |
| `map` | 全域固定座標 | 由 SLAM/定位演算法維護，**不連續但長期精確** |
| `odom` | 里程計座標 | **連續平滑但會漂移**，由 wheel odometry / VIO 維護 |
| `base_link` | 機器人底座 | 剛性固定在機器人上 |
| `camera_link` / `lidar_link` | 感測器座標 | 通常透過 static TF 固定到 `base_link` |

命名慣例：`earth → map → odom → base_link → sensor_link`。**TF 廣播方向**是 parent 發布 child 的相對姿態（`header.frame_id=parent, child_frame_id=child`，數學上記作 $T^{parent}_{child}$）；而**座標轉換方向**則相反 — 把 child frame 的點乘上 $T^{parent}_{child}$ 才會得到它在 parent frame 下的座標。

### Static TF vs Dynamic TF

| 類型 | Topic | 發布頻率 | 典型用途 |
|------|-------|---------|---------|
| Static TF | `/tf_static` | 發一次，latched | 感測器外參（camera → base_link） |
| Dynamic TF | `/tf` | 高頻持續 | 里程計（odom → base_link） |

### Buffer + Listener / Broadcaster

- **Buffer + Listener**：背景訂閱 `/tf` 和 `/tf_static`，把所有收到的 transform 存進 Buffer（預設保留 10 秒）。查詢時呼叫 `lookup_transform(target, source, time)` — Buffer 自動沿著 tree 連乘中間的變換。
- **Broadcaster / StaticBroadcaster**：把 `TransformStamped` 發到 `/tf` 或 `/tf_static`。

### URDF / Xacro + robot_state_publisher

**URDF**（Unified Robot Description Format）用 link + joint 定義機器人的幾何與關節結構。**Xacro** 是 URDF 的巨集語言，支援參數化與模組化。`robot_state_publisher` 讀取 URDF + 訂閱 `/joint_states` → 自動廣播整棵 TF tree — **你不需要手動為每個 link 寫 Broadcaster**。

**在感知 → 規劃 → 控制閉環的位置**：
- **定位**：TF 是全域時空座標統一平台，橫跨閉環所有階段
- **輸入**：各感測器/演算法發布的 `TransformStamped`（里程計、SLAM 修正、感測器外參）
- **輸出**：任意兩 frame 之間的平移 + 旋轉（`geometry_msgs/msg/TransformStamped`）
- **下游**：SLAM（感測器資料統一到 map frame）、Nav2（costmap 需要 base_link → map）、ros2_control（關節 TF 回饋）、MoveIt 2（末端執行器位姿）

### 最少夠用的 API

**查詢變換**：

```python
transform = buffer.lookup_transform(
    target_frame='base_link',
    source_frame='camera_link',
    time=rclpy.time.Time(),  # Time(0) = 最新可用
    timeout=rclpy.duration.Duration(seconds=1.0)
)
```

$T^{target}_{source}$ — 表示「source frame 在 target frame 下的位姿」。物理意義：把 source frame 裡的一個點乘上這個變換，就得到它在 target frame 裡的座標。

**連乘法則**：

$$
T^{A}_{C} = T^{A}_{B} \cdot T^{B}_{C}
$$

物理意義：從 C 到 A 的變換 = 先從 C 到 B、再從 B 到 A。TF2 的 Buffer 在 `lookup_transform` 時自動沿 tree 做這個連乘 — 你只需指定起終點。

**發布變換**：

```python
t = TransformStamped()
t.header.stamp = self.get_clock().now().to_msg()
t.header.frame_id = 'odom'       # parent
t.child_frame_id = 'base_link'   # child
t.transform.translation.x = x
t.transform.rotation = quaternion_from_euler(roll, pitch, yaw)
broadcaster.sendTransform(t)
```

<details>
<summary>深入：TF2 內部 tree 搜尋與時間插值機制</summary>

### Tree 搜尋

TF2 內部維護一棵有向樹（directed tree），每個 edge 存一段時間序列的 `TransformStamped`。`lookup_transform(A, B, t)` 的流程：

1. 從 A 往 root 走，記錄路徑 `[A → ... → root]`
2. 從 B 往 root 走，記錄路徑 `[B → ... → root]`
3. 找最低共同祖先（LCA）
4. A → LCA 路徑上的 transform 取逆、LCA → B 路徑上的 transform 正向連乘

時間複雜度 O(depth)，典型機器人 depth < 20，查詢耗時微秒級。

### 時間插值

Buffer 中每個 edge 儲存 `(timestamp, transform)` 的時間序列。查詢 `time=t` 時：
- 找到 `t` 兩側最近的兩筆資料 `(t1, T1)` 和 `(t2, T2)`
- 平移做線性插值：$p = p_1 + \frac{t - t_1}{t_2 - t_1}(p_2 - p_1)$
- 旋轉做球面線性插值（SLERP）：$q = \text{slerp}(q_1, q_2, \frac{t - t_1}{t_2 - t_1})$

這確保即使查詢時間不精確對齊感測器時間戳，也能得到合理的插值結果。

### `time=Time(0)` 的意義

`Time(0)` 不是「現在」，而是「Buffer 中最新可用的 transform」。這避免了「查現在的時間但 Buffer 還沒收到」的 race condition — 是新手最常踩的坑之一。

</details>

<details>
<summary>深入：URDF link-joint 結構與 Xacro 巨集展開</summary>

### URDF 最小結構

```xml
<robot name="my_robot">
  <link name="base_link">
    <visual>
      <geometry><box size="0.5 0.3 0.1"/></geometry>
    </visual>
    <collision>
      <geometry><box size="0.5 0.3 0.1"/></geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>

  <link name="camera_link">
    <visual><geometry><box size="0.05 0.05 0.05"/></geometry></visual>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="base_link"/>
    <child link="camera_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0.3 0"/>  <!-- 外參！ -->
  </joint>
</robot>
```

**關鍵**：`<joint>` 的 `<origin>` 就是 static TF 的來源。`robot_state_publisher` 讀到 `type="fixed"` 的 joint 會自動發 static TF，`type="revolute"` / `"prismatic"` 的 joint 則根據 `/joint_states` 發 dynamic TF。

### Xacro 巨集化

```xml
<xacro:macro name="wheel" params="prefix x_offset">
  <link name="${prefix}_wheel_link">...</link>
  <joint name="${prefix}_wheel_joint" type="continuous">
    <parent link="base_link"/>
    <child link="${prefix}_wheel_link"/>
    <origin xyz="${x_offset} 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</xacro:macro>

<xacro:wheel prefix="left" x_offset="-0.15"/>
<xacro:wheel prefix="right" x_offset="0.15"/>
```

展開指令：`xacro model.urdf.xacro > model.urdf`

</details>

## 直覺理解

| 概念 | 類比 |
|------|------|
| TF tree | 俄羅斯套娃 — 每一層都記錄相對於外層的偏移與旋轉，要算最外到最內的關係就逐層連乘 |
| Static TF | 照片上的相對位置 — 拍一次就固定了（感測器鎖螺絲後不會動） |
| Dynamic TF | 舞者的即時姿態 — 每一幀都不同，持續更新 |
| Buffer | 回放錄影帶的緩衝區 — 存了過去 10 秒的所有姿態，可以倒回去查任意時刻 |
| `map` vs `odom` | GPS vs 計步器 — GPS 會跳（不連續但全域準確），計步器很平滑但累積誤差（漂移） |
| REP-105 層級 | 地址系統：國家（earth）→ 城市（map）→ 街區（odom）→ 大樓（base_link）→ 房間（sensor） |

**模擬器 / 工具觀察**：
- **Gazebo / Isaac Sim**：載入 URDF 後在 rviz2 開 TF 顯示，能看到每個 frame 的彩色座標軸。轉動關節 → 對應的子 frame 即時跟著旋轉
- **`ros2 run tf2_tools view_frames`**：生成 `frames.pdf`，一張圖看到整棵 TF tree 的結構、更新頻率、Buffer 長度
- **`ros2 run tf2_ros tf2_echo base_link camera_link`**：即時印出兩個 frame 之間的平移和旋轉，驗證感測器外參是否正確
- **`rqt_tf_tree`**：圖形化 TF tree，能看到哪些 edge 是 static、哪些是 dynamic、更新頻率多少

## 實作連結

**三個典型工程場景**：

1. **感測器外參標定後發布 Static TF**：用標定工具（如 `camera_calibration`）得到相機相對於 base_link 的外參矩陣，用 `StaticTransformBroadcaster` 發布一次。所有下游 node 透過 `lookup_transform` 就能把相機座標系的資料轉到 base_link。

2. **里程計發布 Dynamic TF**：Wheel odometry 或 VIO 演算法持續計算 odom → base_link 的變換，用 `TransformBroadcaster` 以 50–100 Hz 發布。Nav2 的 costmap 靠這個 TF 把 laser scan 正確投影到地圖上。

3. **點雲座標轉換**：外部深度相機拍到的點雲在 `camera_link` 座標系下，需要轉到 `base_link` 做障礙物偵測。用 `tf2_ros::Buffer` + `tf2::doTransform` 把每個點從 camera_link 轉到 base_link。

```python
# 場景 1：Static TF 發布（感測器外參）
class CameraStaticTFPublisher(Node):
    def __init__(self):
        super().__init__('camera_static_tf')
        self.broadcaster = StaticTransformBroadcaster(self)

        t = TransformStamped()
        t.header.frame_id = 'base_link'   # parent
        t.child_frame_id = 'camera_link'  # child
        t.transform.translation.x = 0.2   # 相機在 base 前方 20cm
        t.transform.translation.z = 0.15  # 高 15cm
        # 相機朝下傾斜 15 度
        q = quaternion_from_euler(0, 0.26, 0)
        t.transform.rotation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.broadcaster.sendTransform(t)
```

```python
# 場景 3：查詢 TF 並轉換點雲
class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('pc_transformer')
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)

    def transform_cloud(self, cloud_msg):
        try:
            transform = self.buffer.lookup_transform(
                'base_link', cloud_msg.header.frame_id,
                rclpy.time.Time(),  # 最新可用
                timeout=Duration(seconds=0.5)
            )
            cloud_base = do_transform_cloud(cloud_msg, transform)
            return cloud_base
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None
```

<details>
<summary>深入：完整 TF Listener + Broadcaster 實作（Python，可直接執行）</summary>

```python
#!/usr/bin/env python3
"""完整範例：一個 node 同時做 Static TF 發布、Dynamic TF 發布、TF 查詢"""

import rclpy
from rclpy.node import Node
from tf2_ros import (
    TransformBroadcaster,
    StaticTransformBroadcaster,
    Buffer,
    TransformListener,
)
from tf2_ros import LookupException, ExtrapolationException
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler
import math


class TFDemoNode(Node):
    def __init__(self):
        super().__init__('tf_demo')

        # --- Static TF：camera_link 固定在 base_link 上 ---
        self.static_br = StaticTransformBroadcaster(self)
        static_t = TransformStamped()
        static_t.header.stamp = self.get_clock().now().to_msg()
        static_t.header.frame_id = 'base_link'
        static_t.child_frame_id = 'camera_link'
        static_t.transform.translation.x = 0.2
        static_t.transform.translation.z = 0.15
        q = quaternion_from_euler(0.0, 0.26, 0.0)
        static_t.transform.rotation.x = q[0]
        static_t.transform.rotation.y = q[1]
        static_t.transform.rotation.z = q[2]
        static_t.transform.rotation.w = q[3]
        self.static_br.sendTransform(static_t)
        self.get_logger().info('Published static TF: base_link -> camera_link')

        # --- Dynamic TF：模擬 odom -> base_link ---
        self.dynamic_br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.publish_odom_tf)  # 50 Hz
        self.t = 0.0

        # --- TF Listener：查詢 odom -> camera_link ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.query_timer = self.create_timer(1.0, self.query_tf)  # 1 Hz 查詢

    def publish_odom_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        # 模擬圓弧運動
        t.transform.translation.x = math.cos(self.t * 0.5)
        t.transform.translation.y = math.sin(self.t * 0.5)
        q = quaternion_from_euler(0, 0, self.t * 0.5 + math.pi / 2)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.dynamic_br.sendTransform(t)
        self.t += 0.02

    def query_tf(self):
        try:
            # 查 odom -> camera_link（跨越 dynamic + static TF）
            trans = self.tf_buffer.lookup_transform(
                'odom', 'camera_link',
                rclpy.time.Time(),  # 最新可用
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            p = trans.transform.translation
            self.get_logger().info(
                f'odom -> camera_link: x={p.x:.3f}, y={p.y:.3f}, z={p.z:.3f}'
            )
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup failed: {e}')


def main():
    rclpy.init()
    node = TFDemoNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### C++ 版本骨架（tf2_ros）

```cpp
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class TFDemoNode : public rclcpp::Node {
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_br_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_br_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

public:
  TFDemoNode() : Node("tf_demo") {
    dynamic_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_br_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // lookup 要用 try-catch
    try {
      auto t = tf_buffer_->lookupTransform(
        "base_link", "camera_link", tf2::TimePointZero,
        tf2::durationFromSec(1.0));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "TF: %s", ex.what());
    }
  }
};
```

</details>

## 常見誤解

1. **parent ↔ child 搞反** — `header.frame_id` 是 **parent**（child 姿態是表達在誰的座標下），`child_frame_id` 是 child。搞反的症狀：rviz 裡模型跑到地圖的鏡像位置或翻轉。**記法**：REP-105 的階層是 `map → odom → base_link → sensor_link`，發布時永遠把左邊那個當 parent、右邊那個當 child；讀 $T^{parent}_{child}$ 時順序是「右到左」— 把 source（child）frame 的點映射到 target（parent）frame。

2. **查詢用 `now()` 但 Buffer 還沒收到** — `lookup_transform(..., self.get_clock().now())` 查的是「精確這個時刻」的 TF，但因為網路延遲和發布頻率，Buffer 裡可能還沒有這個時刻的資料 → `ExtrapolationException`。**正確做法**：用 `rclpy.time.Time()` 即 `Time(0)`，意思是「Buffer 中最新可用的 transform」。需要精確時間同步時才用具體時間戳 + 足夠的 timeout。

3. **忘記發 Static TF** — 感測器裝上去後沒有發布 camera_link → base_link 的 static TF。症狀：`lookup_transform` 永遠 timeout，rviz 裡點雲顯示在原點或亂飄。**避開**：感測器外參必須在 URDF 裡定義（`type="fixed"` joint）或用 `static_transform_publisher` launch。

4. **URDF 裡出現閉環（cycle）** — TF 是 **tree**（樹），不可有環。平行四連桿或閉鏈機構不能直接用 URDF joint 表示 — 會導致 `robot_state_publisher` 啟動失敗或 TF tree 異常。**解法**：用虛擬 joint + constraint solver（如 KDL 閉鏈求解）。

## 練習題

<details>
<summary>Q1（簡單）：rviz 裡機器人模型浮在空中，離地面大約 10 cm，怎麼排查？</summary>

**分析推理**：
1. **先確認 Fixed Frame 設定**：rviz 的 Fixed Frame 應設為 `map` 或 `odom`，如果設成 `base_link` 則所有東西都相對 base_link 顯示，看不出浮空問題
2. **用 `tf2_echo` 檢查 Z offset**：`ros2 run tf2_ros tf2_echo odom base_link` — 觀察 `translation.z` 是否異常。如果是 0.1 → 某個環節多了 10 cm 的 Z 偏移
3. **排查來源**：
   - URDF 裡 `base_footprint → base_link` 的 joint origin Z 值是否正確（有些 URDF 會把 base_link 放在底盤幾何中心而非地面）
   - 里程計 node 發布的 odom → base_link 的 Z 是否包含了不該有的高度分量
4. **驗證修復**：修改 URDF origin 或里程計 Z 後，用 `tf2_echo` 確認 Z 回到預期值，rviz 重新載入模型

**結論**：浮空 10 cm 通常是 `base_footprint → base_link` 的 Z offset 設定問題，或里程計的 Z 分量不為零。

</details>

<details>
<summary>Q2（中）：外部 RealSense 深度相機的點雲要轉到 base_link 座標下做障礙物偵測，完整流程怎麼做？</summary>

**分析推理**：
1. **標定外參**：用 `easy_handeye` 或手動量測得到 camera_link 相對 base_link 的平移 (x, y, z) 和旋轉 (roll, pitch, yaw)
2. **發布 Static TF**：在 launch file 用 `static_transform_publisher` 或在 URDF 加 `type="fixed"` joint，把 camera_link → base_link 的外參發出去
3. **Node 內建立 Buffer + Listener**：
   ```python
   self.buffer = tf2_ros.Buffer()
   self.listener = tf2_ros.TransformListener(self.buffer, self)
   ```
4. **收到點雲 callback 時查 TF 並轉換**：
   ```python
   transform = self.buffer.lookup_transform('base_link', msg.header.frame_id, rclpy.time.Time())
   cloud_base = do_transform_cloud(msg, transform)
   ```
5. **陷阱**：
   - 忘記 `Time(0)` 改用 `now()` → 因為相機和主機時鐘不完全同步導致 ExtrapolationException
   - 外參旋轉用歐拉角但順序搞錯（RealSense 預設光軸是 Z，ROS 是 X 向前）→ 點雲旋轉 90 度

**結論**：Static TF（外參）+ Buffer/Listener + `lookup_transform` + `do_transform_cloud` 是標準四步驟。

</details>

<details>
<summary>Q3（難）：SLAM 建出的地圖整體偏轉了約 5 度，怎麼定位問題？</summary>

**分析推理**：
1. **理解 frame 權責**：
   - `map → odom`：由 SLAM 演算法（如 Nav2 AMCL、SLAM Toolbox）發布，修正里程計累積漂移
   - `odom → base_link`：由里程計（wheel odom / VIO）發布
   - SLAM 看到的「偏轉」可能來自任一層
2. **用 `rqt_tf_tree` 排查**：
   - 檢查是否有重複的 publisher 在爭搶同一個 TF edge（例如兩個 node 都在發 `odom → base_link`）→ TF tree 會出現跳動
   - 確認每個 edge 只有一個 publisher
3. **檢查感測器外參**：
   - `ros2 run tf2_ros tf2_echo base_link laser_link` → 確認 rpy 是否正確
   - 如果 laser_link 的 yaw 偏了 5 度 → SLAM 用錯誤的外參把 scan 投影到 map → 地圖整體旋轉
4. **重新標定**：修正 URDF 中 `base_link → laser_link` 的 rpy，重新跑 SLAM 建圖
5. **驗證**：對比修正前後的地圖，用已知直牆檢查是否平行於座標軸

**結論**：地圖偏轉最常見的原因是感測器（通常是 LiDAR）的 yaw 外參標定誤差。用 `tf2_echo` 確認外參 → 修 URDF → 重建圖。

</details>

<details>
<summary>Q4（中-難）：多個 node 都需要查 TF，該各自建 Buffer 還是共用一個？效能考量？</summary>

**分析推理**：
1. **各自建 Buffer 的問題**：每個 Buffer 獨立訂閱 `/tf` 和 `/tf_static` → N 個 node 就有 N 份訂閱、N 份記憶體、N 份反序列化開銷
2. **Component 容器 + 共用 Buffer**：把多個 node 載入同一個 `rclcpp_components` 容器（process），共用一個 `tf2_ros::Buffer` 實例
   - 只需一份訂閱和一份記憶體
   - 啟用 `use_intra_process_comms(true)` 時，TF 消息在 process 內走 shared_ptr，無序列化開銷
3. **取捨**：
   - 少量 node（< 5）：各自建 Buffer 影響不大，簡化程式架構
   - 大量 node 或頻寬受限：共用 Buffer 降低 `/tf` topic 的訂閱數和記憶體佔用
4. **陷阱**：共用 Buffer 時注意 thread safety — `tf2_ros::Buffer` 內部有鎖，高頻 lookup 可能競爭

**結論**：中小系統各自建 Buffer 即可；大系統用 Component 容器 + 共用 Buffer 省頻寬和記憶體。

</details>

## 面試角度

1. **map vs odom 的設計哲學** — 這是 ROS 導航最核心的架構問題。面試時帶出：「map frame 由 SLAM 維護，長期全域精確但會跳動；odom frame 由里程計維護，短期平滑但會漂移。把兩者分開讓控制層可以依賴平滑的 odom 做反應式避障，而規劃層用精確的 map 做全域路徑規劃 — 各取所長。」

2. **四元數 vs 歐拉角** — 面試常考為什麼 TF 用四元數。帶出：「歐拉角有 Gimbal Lock、插值不均勻；四元數沒有 Gimbal Lock、SLERP 插值平滑、只用 4 個參數（比旋轉矩陣 9 個省一半）、重新正規化成本低。實務上用 `setRPY()` 做人類可讀的設定，底層全部用四元數表示和運算。」

3. **lookup 一定要包 try-catch** — 測的是工程嚴謹度。帶出：「TF lookup 可能因為 Buffer 尚未收到資料、時間超出 Buffer 範圍、或 frame 不存在而拋異常。production code 絕對要用 try-catch 或 `canTransform` 先檢查，否則一個遺漏就讓整個 node crash。」

4. **Static TF 是系統穩定性的根基** — 測的是系統整合經驗。帶出：「忘記發 static TF 是新手最常見的問題。一個缺失的 camera → base_link 會讓整個感知 pipeline 斷掉。我的習慣是所有感測器外參都定義在 URDF 裡，啟動時由 robot_state_publisher 自動發布 — 不手動管理。」

5. **REP-105 層級架構** — 測的是系統級理解。帶出：「我能畫出 earth → map → odom → base_link → sensor_link 的完整鏈路，並解釋每一層由誰發布、更新頻率、為什麼要分層。」

## 延伸閱讀

- **REP-105: Coordinate Frames for Mobile Platforms** — ROS 座標系設計的根本性文件，定義了 map / odom / base_link 的語意與責任分工，所有 ROS 2 導航系統都遵守
- **ROS 2 官方 TF2 Tutorials** — 從 Broadcaster 到 Listener 的完整 hands-on 教學，含 Python 和 C++ 版本
- **《ROS2》教科書 Ch5: TF 工具** — 系統性介紹 `tf2_echo`、`view_frames`、`rqt_tf_tree` 等除錯工具的使用場景
- **《ROS2》教科書 Ch9.5.2: 移動機器人座標框架** — 詳細講解 map / odom / base_link 在實際導航系統中的互動
- **`tf2_geometry_msgs`** — 把 `geometry_msgs` 的各種型別（`PointStamped`、`PoseStamped`、`Vector3Stamped`）直接餵給 `tf2::doTransform` 的轉換庫，生產 code 必用
- **`tf2_ros::MessageFilter`** — 把 TF lookup 和 message subscription 結合：只有當 TF 可用時才觸發 callback，避免「收到 msg 但 TF 還沒 ready」的 race condition
