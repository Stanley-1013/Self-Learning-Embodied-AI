---
title: "動態環境避障與即時重規劃"
prerequisites: ["10-basic-path-planning", "11-trajectory-optimization"]
estimated_time: 45
difficulty: 4
tags: ["obstacle-avoidance", "dwa", "teb", "mppi", "reactive"]
sidebar_position: 12
---

# 動態環境避障與即時重規劃

## 你將學到

- 能用兩句話講清楚「分層規劃架構」：Global planner（低頻、全局最優）+ Local planner（高頻、即時閃避）如何協作，面試時不含糊
- 遇到「機器人在人群中急停繞路卻還是撞上」這類情境，知道問題出在缺乏動態預測，會想到 Kalman Filter / LSTM 預測行人軌跡並寫入時空 Costmap
- 能判斷何時用 DWA（差速底盤快速原型）、何時用 TEB（阿克曼非完整約束）、何時用 MPPI（非線性高維系統 GPU 並行）

## 核心概念

### 分層規劃架構

**精確定義**：動態避障系統通常分成兩層 — **Global planner** 以低頻（~1 Hz）在全域地圖上求解最短 / 最安全路徑（A\*、Dijkstra），產出粗略路徑點；**Local planner** 以高頻（20–50 Hz）在局部視窗內考慮動態障礙物、運動學約束與舒適性，即時生成可執行的速度指令或軌跡片段。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：感知層的即時障礙物位置與速度估計（LiDAR scan、深度相機、tracking module 輸出）、全域地圖（static map + costmap layers）、當前機器人狀態 $(x, y, \theta, v, \omega)$
- **輸出**：可直接下發給底層控制器的速度指令 $(v, \omega)$ 或短期軌跡 waypoints
- **下游**：底層運動控制器（PID / 純追蹤）、安全監控層（emergency stop）、多機協調模組
- **閉環節點**：橫跨 **規劃** 與 **控制** — Local planner 每個 tick 讀取最新感知、重新規劃，形成「感知 → 局部規劃 → 速度指令 → 執行 → 感知」的高頻閉環

**一句話版本**：「動態避障是機器人的小腦反射神經 — 全局路徑是 GPS 導航，局部規劃是方向盤上的即時操作。」

### 核心演算法比較

| 演算法 | 核心思路 | 適用場景 | 頻率 | 運動學約束 |
|--------|---------|---------|------|-----------|
| **DWA** | 速度空間 $(v, \omega)$ 採樣 + 代價函數評估 | 差速 / 全向底盤 | 20–50 Hz | 速度與加速度邊界 |
| **TEB** | 時空彈性帶（Elastic Band），C² 連續軌跡優化 | 阿克曼（車型）、非完整約束 | 10–30 Hz | 最小轉彎半徑、加速度限制 |
| **MPPI** | Sampling-based MPC，GPU 並行數千條軌跡 | 非線性系統、高維 | 30–100 Hz | 任意非線性動力學模型 |

### 最少夠用的數學

1. **DWA 代價函數**（速度空間中的多目標加權）：

$$
G(v, \omega) = \sigma \big[\alpha \cdot \text{heading}(v,\omega) + \beta \cdot \text{dist}(v,\omega) + \gamma \cdot \text{velocity}(v,\omega)\big]
$$

**物理意義**：$\text{heading}$ 衡量朝目標對齊度、$\text{dist}$ 衡量離最近障礙物的安全距離、$\text{velocity}$ 鼓勵高速前進。$\alpha, \beta, \gamma$ 是可調權重。在可達速度窗口 $(v, \omega) \in V_s \cap V_d \cap V_a$（靜態限制 $\cap$ 動態限制 $\cap$ 可行加速度）內，選代價最高的一組速度下發。

2. **Velocity Obstacle (VO)**（碰撞錐排除危險速度）：

$$
VO_{A|B} = \left\{ \mathbf{v}_A \mid \exists\, t > 0 : \mathbf{v}_A \cdot t \in D(B \oplus (-A)) \right\}
$$

**物理意義**：把障礙物 $B$ 用 Minkowski 和膨脹成包含機器人形狀的區域，從原點射出的速度錐就是「會撞上」的速度集合。安全速度就是在錐外面選。RVO（Reciprocal VO）讓雙方各分擔一半避讓責任，ORCA 把它線性化到毫秒級求解。

3. **MPPI 軌跡權重**（Information-theoretic MPC）：

$$
w^{(k)} = \exp\!\left(-\frac{1}{\lambda} S(\tau^{(k)})\right), \quad u^* = \frac{\sum_k w^{(k)} u^{(k)}}{\sum_k w^{(k)}}
$$

**物理意義**：對 $K$ 條隨機擾動軌跡 $\tau^{(k)}$ 各自計算總代價 $S$，用 Boltzmann 權重 softmax 聚合。$\lambda$ 是溫度 — 越小越貪心（只信最好的幾條），越大越保守（平均所有軌跡）。GPU 上 $K = 2048\sim 8192$ 並行，每條軌跡獨立 rollout 無資料依賴。

<details>
<summary>深入：DWA 完整演算法流程與速度空間窗口推導</summary>

**DWA 每個 tick 的完整步驟**：

1. **建立速度搜尋空間**：
   - 靜態窗口 $V_s = \{(v, \omega) \mid v \in [v_{\min}, v_{\max}],\; \omega \in [\omega_{\min}, \omega_{\max}]\}$
   - 動態窗口 $V_d = \{(v, \omega) \mid v \in [v_c - \dot{v}_b \Delta t,\; v_c + \dot{v}_a \Delta t],\; \omega \in [\omega_c - \dot{\omega}_b \Delta t,\; \omega_c + \dot{\omega}_a \Delta t]\}$
   - 可達窗口 $V_a = \{(v, \omega) \mid v \leq \sqrt{2 \cdot \text{dist}(v,\omega) \cdot \dot{v}_b}\}$（確保碰前能煞停）
   - 最終搜尋空間 $V = V_s \cap V_d \cap V_a$

2. **前向模擬**：對 $V$ 中離散化的每對 $(v_i, \omega_j)$，模擬 $T_{\text{sim}}$（通常 1–3 秒）的軌跡弧線

3. **代價評估**：
   - $\text{heading}$：軌跡終點朝向與目標方向的夾角（越小越好）
   - $\text{dist}$：軌跡上離最近障礙物的最短距離（越大越好）
   - $\text{velocity}$：前進速度大小（鼓勵快速移動）

4. **選最優**：$G = \alpha \cdot \text{heading} + \beta \cdot \text{dist} + \gamma \cdot \text{velocity}$ 取最大值的 $(v^*, \omega^*)$ 下發

**速度空間離散化**：典型 DWA 用 $30 \times 30$ 到 $50 \times 50$ 的網格。更精細的網格提高解品質但增加計算量。Nav2 的 DWA 實作用迭代式前向模擬，每條軌跡 20–30 個時間步。

**調參實戰**：
- 窄走廊：提高 $\beta$（安全距離權重），降低 $\gamma$（容許減速）
- 開闊空間：提高 $\gamma$（追求速度），降低 $\beta$
- 人群環境：加入預測層，把行人未來位置膨脹寫入 Costmap

</details>

<details>
<summary>深入：MPPI 的 GPU 實作架構與溫度參數調校</summary>

**MPPI GPU 並行架構**：

```
┌─────────────────────────────────────────┐
│  Host (CPU)                             │
│  ├── 讀取當前狀態 x₀                     │
│  ├── 產生控制擾動 δu ~ N(0, Σ)           │
│  └── 上傳 x₀ + δu 到 GPU               │
├─────────────────────────────────────────┤
│  Device (GPU) — K 個 thread blocks       │
│  ├── 每個 thread: rollout 一條軌跡       │
│  │   ├── x_{t+1} = f(x_t, u_nom + δu_t) │
│  │   └── 累加 stage cost S_k            │
│  ├── Parallel reduction: 求 min(S)       │
│  └── 加權平均: u* = Σ w_k · u_k / Σ w_k │
├─────────────────────────────────────────┤
│  Host: u* 下發給控制器                   │
└─────────────────────────────────────────┘
```

**溫度 $\lambda$ 的調校**：
- $\lambda \to 0$：接近 arg min，只取最低代價軌跡 → 激進但 noisy
- $\lambda \to \infty$：所有軌跡等權 → 過度保守
- 實務：$\lambda \in [0.01, 10]$，通常從 $\lambda = 1$ 開始，在模擬器裡二分搜尋到「不碰撞且不過度繞路」的甜蜜點

**代價函數設計（自駕常見）**：

$$
S(\tau) = \sum_{t=0}^{T} \left[ q_{\text{goal}} \|x_t - x_{\text{goal}}\|^2 + q_{\text{obs}} \cdot c_{\text{obs}}(x_t) + q_u \|u_t\|^2 + q_{\text{jerk}} \|\Delta u_t\|^2 \right]
$$

- $c_{\text{obs}}$：用 SDF（Signed Distance Field）查表，距離障礙物越近代價指數增長
- $q_{\text{jerk}}$：抑制控制量突變，提升乘坐舒適性

**常見實作**：NVIDIA Isaac Lab 內建 MPPI controller、[MPPI-Generic](https://github.com/ACDSLab/MPPI-Generic) C++/CUDA 開源庫。

</details>

### Costmap 架構

Costmap 是連接感知與規劃的關鍵資料結構：

| 層級 | 功能 | 更新頻率 |
|------|------|---------|
| **Static Layer** | 從 SLAM 地圖載入的靜態佔據格 | 一次性 |
| **Obstacle Layer** | LiDAR / 深度相機即時偵測的障礙物 | 10–30 Hz |
| **Inflation Layer** | 從障礙物邊緣向外膨脹安全梯度 | 跟隨 Obstacle Layer |
| **Voxel Layer** | 3D 體素層（處理懸空障礙、桌下空間） | 10–30 Hz |

**物理意義**：Inflation Layer 的代價值從障礙物邊緣呈指數衰減 — 機器人中心碰到高代價區域就表示外殼快擦到了。`inflation_radius` 設定膨脹半徑，`cost_scaling_factor` 控制衰減陡度。

## 直覺理解

**類比：開車的三層反應**
- **GPS 導航 = Global planner**：告訴你「走高速再轉國道」，頻率很低（重算一次要幾秒）
- **方向盤操作 = Local planner (DWA/TEB)**：在車道內即時微調方向，避開前方突然變道的車，20–50 Hz
- **ABS 急煞 = 反應式安全層 (VO/ORCA/CBF)**：不經過「規劃」直接在速度空間排除危險指令，<5 ms 反應

**DWA 的直覺**：想像你站在房間中央，面前扇形展開 900 條可能的走法（不同速度和轉彎率），每條走幾步看看會不會撞到東西、能不能朝門口、速度快不快。挑最好的那條走一步，下一瞬間重新展開 900 條。

**TEB 的直覺**：想像你用橡皮筋穿過釘子（障礙物）連到目標，橡皮筋會自然繃成最短且平滑的路徑。TEB 就是把這條橡皮筋放到「時空」中 — 不只避開空間中的障礙物，還避開「某個時刻會出現在某個位置」的動態障礙。

**模擬器觀察**：在 Gazebo + Nav2 裡，同一張地圖分別切換 DWA 和 TEB controller，觀察：
- DWA 在寬走廊流暢但窄道容易原地打轉
- TEB 在阿克曼底盤上能做出更自然的曲線轉彎
- 在 rviz2 開 `local_costmap` 視覺化，看 inflation 梯度如何影響路徑選擇

## 實作連結

**三個典型工程場景**：

1. **ROS 2 Nav2 分層規劃**：`bt_navigator` 用 Behavior Tree 協調 global planner（NavFn / Smac）與 local controller（DWA / TEB / MPPI）。Global path 以 `nav_msgs/Path` 發布，local controller 每個 tick 讀取 `local_costmap` + global path，輸出 `geometry_msgs/Twist` 給底盤。

2. **自駕局部規劃（Autoware / Apollo）**：MPPI 或 Lattice planner 以 30–100 Hz 在 Frenet frame 下規劃，代價函數同時考慮車道中心偏移、障礙物 SDF、舒適性（jerk）、交通規則。輸出軌跡給 PID / Stanley / MPC 追蹤控制器。

3. **多機器人倉庫物流（ORCA）**：每台 AGV 各自跑 ORCA 分散式互惠避障，不需中央協調。計算複雜度 $O(n)$（每台只看鄰近 $n$ 台），適合 100+ 台同時運作。優先級仲裁：載貨車 > 空車；交叉路口用拍賣制分配通行權。

**Code 骨架**（Python，Nav2 DWA 風格）：

```python
class DWAPlanner:
    def __init__(self, config: DWAConfig):
        self.config = config  # 包含 v_max, w_max, acc_lim, dt, sim_time, weights

    def compute_velocity(
        self,
        state: RobotState,        # (x, y, theta, v, omega)
        goal: Pose2D,             # 目標位置
        costmap: Costmap2D,       # 局部代價地圖
        global_path: list[Pose2D] # 全域路徑參考點
    ) -> tuple[float, float]:     # 回傳 (v, omega)
        """DWA 核心：採樣 → 模擬 → 評估 → 選最優"""
        best_score = -float('inf')
        best_v, best_w = 0.0, 0.0

        for v, w in self._sample_velocities(state):
            traj = self._simulate(state, v, w)
            if self._check_collision(traj, costmap):
                continue
            score = self._evaluate(traj, goal, costmap, global_path)
            if score > best_score:
                best_score, best_v, best_w = score, v, w

        return best_v, best_w
```

<details>
<summary>深入：Nav2 MPPI Controller 的完整 ROS 2 配置與調參指南</summary>

**Nav2 MPPI controller 參數配置**（`nav2_params.yaml`）：

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 30.0
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      time_steps: 56              # 前向模擬步數
      model_dt: 0.05              # 每步時間間隔 → 預測窗口 2.8s
      batch_size: 2000            # GPU 並行軌跡數
      vx_std: 0.2                 # 線速度擾動標準差
      vy_std: 0.0                 # 差速底盤 vy = 0
      wz_std: 0.4                 # 角速度擾動標準差
      vx_max: 0.5
      vx_min: -0.35
      wz_max: 1.9
      temperature: 0.3            # lambda，越小越激進
      gamma: 0.015                # 控制平滑項權重
      iteration_count: 1          # 每次規劃的 MPPI 迭代數
      critics:
        - "GoalCritic"
        - "GoalAngleCritic"
        - "ObstaclesCritic"
        - "PathFollowCritic"
        - "PathAngleCritic"
        - "PreferForwardCritic"
      ObstaclesCritic:
        cost_power: 1
        repulsion_weight: 1.5     # 障礙物斥力權重
        critical_weight: 20.0     # 碰撞代價（極高）
        collision_cost: 10000.0
        near_goal_distance: 0.5
```

**調參心法**：
1. 先調 `batch_size` 確保 GPU 不 OOM（RTX 3060 大約能跑 4096 條）
2. `temperature` 從 1.0 開始降：觀察是否開始碰撞（太低）或繞路（太高）
3. `ObstaclesCritic.repulsion_weight` 和 `critical_weight` 控制避障激進度
4. `vx_std / wz_std` 決定探索範圍：太小軌跡集中看不到好路徑，太大計算浪費

**CPU vs GPU 後端**：Nav2 MPPI 有純 CPU 版（batch_size 約 300 以下還行）和 CUDA 版。倉庫 AGV 用 Jetson Orin 跑 CUDA 版 batch_size=2048 可以穩定 50 Hz。

</details>

## 常見誤解

1. **「Local planner 能取代 Global planner」** — Local planner 只看局部窗口（通常 3–5 m），在 U 型走廊或迷宮中會陷入局部最小值。Global planner 提供「大方向」引導 Local planner 跳出死胡同。**避開**：永遠保持分層架構；如果 local planner 原地打轉超過閾值，trigger global replan。

2. **「DWA 適用所有底盤」** — DWA 假設圓弧軌跡（恆定 $v, \omega$），對阿克曼底盤（有最小轉彎半徑）的約束建模不佳。強硬套用會產生不可執行的指令。**避開**：阿克曼底盤用 TEB 或 MPPI，它們原生支援非完整約束。

3. **「把障礙物膨脹到最大就最安全」** — 過度膨脹會讓可通行空間消失，機器人在窄門前停下不走。**避開**：`inflation_radius` 設為機器人半徑 + 安全餘量（通常 0.1–0.3 m），且用 `cost_scaling_factor` 讓代價衰減有梯度 — 靠近障礙但不碰撞是允許的。

4. **「Costmap 只需要 2D 就夠了」** — 純 2D costmap 無法處理桌子下面可穿越、懸空招牌需閃避的場景。**避開**：加入 Voxel Layer 做 3D → 2D 投影；或直接用 3D SDF（ESDF / Voxblox）做碰撞檢測。

5. **「反應式避障（VO/RVO）不需要預測」** — VO 假設障礙物以恆定速度直線運動，對突然轉向的行人會失效。**避開**：在 VO 前端接行人軌跡預測模組（Kalman Filter、Social Force Model 或 LSTM），把預測軌跡轉成時空碰撞錐。

## 練習題

<details>
<summary>Q1（中）：機器人用 DWA 在人群密集的展覽場地導航，頻繁急停 → 繞路 → 幾乎停滯不前。怎麼分析？用什麼工具？要避開什麼？</summary>

**完整推理鏈**：

1. **診斷根因**：DWA 的障礙物層只反映「此刻位置」，不預測行人未來走向。人一動，DWA 瞬間發現原本安全的速度變危險，急煞 → 重新採樣 → 又急煞 → 震盪
2. **加入動態預測**：
   - 接入行人追蹤模組（如 `spencer_people_tracking` 或自訓 YOLO + DeepSORT）
   - 對每個行人用 Kalman Filter 或 Constant Velocity Model 預測未來 2–3 秒軌跡
   - 把預測位置以漸弱的代價值寫入 **時空 Costmap**（未來越遠、代價越低 — 因為不確定性越大）
3. **升級規劃器**：
   - 方案 A：保留 DWA 但把時空 Costmap 當 obstacle layer — 低成本改動
   - 方案 B：換成 TEB（支援動態障礙物 native 輸入）或 MPPI（代價函數直接加預測軌跡碰撞項）
4. **避開的陷阱**：
   - 不要把行人預測窗口拉太長（>3 s），預測誤差累積會讓 costmap 到處都是高代價區，反而動彈不得
   - 不要忘記行人可能突然停下 — 預測模型要有「速度衰減」fallback

**面試官想聽到**：清楚區分「反應式」vs「預測式」避障的差異；能給出 Costmap 整合預測的具體方案；知道預測窗口過長的副作用。

</details>

<details>
<summary>Q2（中-難）：MoveIt 規劃的機械臂在動態抓取場景（傳送帶上的物體）太慢，從規劃到執行 >500 ms，物體早已移走。怎麼解決？</summary>

**完整推理鏈**：

1. **理解瓶頸**：MoveIt 的 RRT/PRM planner 是全域搜尋 C-space，每次規劃需 100–500 ms，且不考慮障礙物的運動
2. **分層解法**：
   - **低頻層（~2 Hz）**：MoveIt 預先規劃一條到「預測抓取點」的參考軌跡（考慮物體速度 $\times$ 到達時間的超前量）
   - **高頻層（100–1000 Hz）**：在關節空間用 QP（Quadratic Programming）或 MPC 做即時修正
     - 讀取視覺追蹤的物體即時位姿
     - 用 SDF（Signed Distance Field）做環境碰撞約束
     - 求解 $\min \|q - q_{\text{ref}}\|^2$ s.t. 碰撞約束 + 關節限位 + 速度限制
3. **SDF 加速**：離線用 `voxblox` 或 GPU 體素化建 ESDF，線上查表 $O(1)$，10 ms 內完成碰撞檢測
4. **工具鏈**：
   - `moveit_servo`：讀 Twist 指令做即時笛卡爾空間追蹤
   - `curobo`（NVIDIA）：GPU 加速的 motion planning，規劃時間壓到 <10 ms

**面試官想聽到**：分層規劃思維（離線粗規劃 + 線上精修正）；SDF 碰撞檢測的效能優勢；知道 `moveit_servo` / `curobo` 這些即時避障工具鏈。

</details>

<details>
<summary>Q3（難）：100 台 AGV 在倉庫中交叉路口頻繁死鎖。目前用中央調度但延遲高、單點故障。怎麼設計分散式避障方案？</summary>

**完整推理鏈**：

1. **選演算法**：**ORCA**（Optimal Reciprocal Collision Avoidance）
   - 每台 AGV 各自計算安全速度集合（ORCA 半平面交集），不需中央協調
   - 計算複雜度 $O(n)$（$n$ 為鄰近車輛數），100 台各自只看半徑 5 m 內的鄰居
   - 數學保證：只要所有 agent 都遵守 ORCA，不會碰撞（reciprocal 假設）
2. **優先級仲裁**：
   - 靜態優先級：載貨 AGV > 空車 AGV > 充電中 AGV
   - 動態優先級：距目標越近 → 優先級越高（避免快到終點卻被擠走）
   - 交叉路口：優先級高的 AGV 的 ORCA 半平面對低優先級 AGV 施加更強約束
3. **死鎖偵測與恢復**：
   - 監控：如果某台 AGV 速度 < 閾值超過 $T$ 秒，標記為「疑似死鎖」
   - 恢復：隨機選一台讓它後退 0.5 m 並重新規劃，打破對稱
   - 備援：如果區域性死鎖（>3 台互鎖），升級為區域中央調度（混合架構）
4. **避開的陷阱**：
   - ORCA 假設所有 agent 都遵守協議 — 混入人類叉車時必須加安全層
   - ORCA 不考慮非完整約束 — 阿克曼 AGV 需用 NH-ORCA 變體

**面試官想聽到**：分散式 vs 集中式的 trade-off；ORCA 的 reciprocal 保證；實務中的混合架構（分散式為主 + 區域性中央調度 fallback）。

</details>

<details>
<summary>Q4（難）：你的自駕車用 MPPI 做局部規劃，在高速公路上表現良好，但進入城市窄巷後經常擦牆。不換演算法的前提下怎麼修？</summary>

**完整推理鏈**：

1. **診斷根因**：高速公路的 `vx_std` 和 `wz_std` 設定偏大（探索範圍廣），進入窄巷後隨機軌跡大部分都碰撞被淘汰，剩餘有效軌跡太少，加權平均的 $u^*$ 品質差
2. **動態調參**：
   - 偵測到進入窄巷（global costmap 的可通行寬度 < 閾值），自動切換參數集：
     - 降低 `vx_std / wz_std`（收窄採樣範圍，集中在可行區域）
     - 增加 `batch_size`（在窄範圍內多採幾條，確保覆蓋度）
     - 降低 `temperature`（更信任少數好軌跡，不被爛軌跡拖累）
3. **代價函數調整**：
   - 加入 `CostmapCritic` 的 `near_wall_penalty`：距離牆壁 < 0.5 m 時代價指數增長
   - 提高 `PathFollowCritic` 權重：窄巷中嚴格跟隨 global path 中心線
4. **備選優化**：
   - 用 importance sampling（非均勻採樣）：在上一輪最優軌跡附近密集採樣
   - 加入 warm-start：用上一 tick 的最優控制序列作為均值，減少浪費

**面試官想聽到**：理解 MPPI 的採樣效率問題（窄空間中有效軌跡比例低）；能根據場景動態調整超參數而非一組參數走天下；知道 importance sampling 和 warm-start 的加速技巧。

</details>

## 面試角度

1. **分層規劃架構是核心設計原則** — 證明你理解工程系統的「算力 vs 反應速度」trade-off。**帶出**：「我會把規劃系統分成 Global（1 Hz A\*，保證全局可達性）和 Local（30+ Hz DWA/MPPI，保證即時避障）。低頻算大方向，高頻做精細閃避 — 這不是教科書分法，是實際算力預算下的唯一可行解。」

2. **動態預測整合 Costmap 是關鍵差異化能力** — 區分「只會跑 Nav2 demo」和「真能處理動態環境」的分水嶺。**帶出**：「純 reactive 避障對靜態環境夠用，但遇到人群就崩。我的做法是把行人追蹤 + 軌跡預測的結果寫成時空 Costmap layer，預測窗口 2–3 秒，代價隨預測不確定性衰減。」

3. **演算法選擇取決於底盤運動學和算力** — 展現你不是死背演算法，而是根據約束做工程決策。**帶出**：「差速底盤首選 DWA — 簡單高效；阿克曼用 TEB 處理最小轉彎半徑；高維非線性系統上 MPPI — 但前提是有 GPU。選錯演算法比調錯參數更致命。」

4. **反應式安全層是最後防線** — 說明你理解 defense in depth 的系統安全設計。**帶出**：「即使 Local planner 夠快，我還是會在最底層放一個 VO/CBF 安全層，延遲 <5 ms，純粹排除會碰撞的速度指令。規劃層出 bug 時，這層能保命。」

5. **多機器人場景要從集中式思維切換到分散式** — 展現系統級視野。**帶出**：「100 台 AGV 不可能靠一台中央伺服器每 50 ms 幫所有人規劃。ORCA 讓每台自己算，只需鄰居資訊，$O(n)$ 複雜度。但要在上面疊優先級仲裁和死鎖偵測，純分散式在交叉路口還是會死鎖。」

## 延伸閱讀

- **Fox et al., *The Dynamic Window Approach to Collision Avoidance* (1997)** — DWA 原始論文，速度空間採樣的開山之作，理解 DWA 的設計動機和數學推導
- **Rösmann et al., *Timed-Elastic-Band Local Planner* (2013–2017 系列)** — TEB 從基礎概念到多拓撲路徑選擇的完整演進，理解時空彈性帶優化
- **Williams et al., *Information Theoretic MPC (MPPI)* (2017)** — MPPI 的資訊理論基礎，理解為什麼 Boltzmann 加權比簡單取 arg min 更穩健
- **van den Berg et al., *Reciprocal n-Body Collision Avoidance (ORCA)* (2011)** — 多機器人分散式避障的數學保證，理解 reciprocal 半平面的幾何意義
- **ROS 2 Nav2 官方文件 — Controller Plugins** — DWA / TEB / MPPI 三種 controller 的完整配置參考與調參指南
- **NVIDIA Isaac Lab / curobo** — GPU 加速的 motion planning 與避障，適合高維機械臂場景的即時規劃
- **《具身智能算法工程師 面試題》Ch8.3 動態障礙檢測、Ch8.4 動態重規劃** — 面試高頻考點：分層架構、預測整合、演算法選擇的 talking points
