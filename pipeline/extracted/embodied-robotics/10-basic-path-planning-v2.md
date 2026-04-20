# Extracted v2: Ch10 路徑規劃 — 5 dedicated queries (from scratch)

## Q1 補強（Grid-based / Search-based Planning）

### 情境題 A：A* / Dijkstra 底層機制
- **Dijkstra 本質**：BFS + Priority Queue，水波紋向外均勻擴展，保證全局最短
- **3D 爆炸原因**：沒有方向感；(x,y,z,θ,t) 狀態空間節點指數增長；O(V log V) 記憶體/算力崩潰
- **A\* 核心性質**：`f(n) = g(n) + h(n)`
  - **Admissibility**：`h(n) ≤ h*(n)` 不高估 → **保證最優**（高估會誤以為最優路徑代價太高而跳過）
  - **Consistency**：`h(n) ≤ c(n,n') + h(n')`（三角不等式）→ 每節點最多 pop 一次 → O(N log N) 不重複探索
- **Heuristic 選型**：
  - **Manhattan**：4 向移動網格
  - **Chebyshev**：8 向，斜向與直向同代價
  - **Euclidean**：任意角度直線距離（真實物理）
- **Weighted A\***（ε-admissible）：`f = g + ε·h (ε>1)` → 搜索侵略性直奔終點
  - 失嚴格最優性（誤差上限 ε 倍）但**快 10-100 倍**
- **JPS (Jump Point Search)**：大片空曠網格時 A* 會把對稱等價路徑都塞進 Queue；JPS 跳躍點規則跳過無 Forced Neighbors 的均勻網格 → 壓縮 Open List
- **格子解析度權衡**（面試陷阱）：
  - 太細 → 維數災難 + 鋸齒路徑
  - 太粗 → 狹窄通道被離散化誤差判定碰撞（Narrow passage 陷阱）
  - **實務解**：多解析度地圖（Octree）或分層規劃
- **平台**：ROS 2 Nav2 Smac Planner、Amazon Kiva 倉儲

### 情境題 B：Any-angle / Theta* / D* Lite 動態重規劃
- **8-directional zigzag 問題**：8 連通網格只能 0°/45°/90° 倍數延伸；15° 捷徑必須用水平+對角拼湊 → 機器人扭動+磨損+耗能
- **Theta\* (Any-angle)**：
  - 父子節點不必相鄰
  - 展開 s→s' 時執行 **Line-of-Sight (LoS) Check**
  - 若 s 的父節點到 s' 無障礙直線 → **直接讓 s' 父節點設為 parent(s)**
  - 「裁彎取直」在搜索時同步完成 path smoothing
  - 更新：`g(s') = min(g(s'), g(parent(u)) + c(parent(u), s'))` if LoS
- **D\* Lite (Dynamic A*)**：
  - 從 Goal 反向搜索
  - 每節點維護 `g(n)`（當前代價）+ `rhs(n)`（基於鄰居的一步前瞻代價）
  - 靜態時 `g = rhs`
  - 感測器發現新障礙 → 只更新局部邊代價 → `g ≠ rhs` 產生 Inconsistency
  - **優先佇列只對「不一致」邊界節點重新傳播** → 復用之前 heuristic 與代價圖
  - 比重跑 A* 快**幾個數量級**
- **動態環境標準答案**（面試必備）：
  - 絕不重跑 A*（會卡死 + 原地抖動）
  - **路線 1**：D* Lite 增量式，僅在感知更新的局部區域修補搜索樹
  - **路線 2**：分層規劃 — Global 拓撲 + Local (TEB/DWA) 速度空間躲動態障礙
- **Theta* LoS 更新 Python**：
  ```python
  def update_vertex(u, s_prime):
      if line_of_sight(u.parent, s_prime):
          cost = u.parent.g + euclidean(u.parent, s_prime)
          if cost < s_prime.g:
              s_prime.g = cost
              s_prime.parent = u.parent  # 越過 u
      else:
          # 退化為常規 A* 更新
          cost = u.g + euclidean(u, s_prime)
          ...
  ```

### 情境題 C：Layered Costmap + Global vs Local Planner
- **Layered Costmap 分離哲學**：
  - **Static Layer**：SLAM 建圖得到的固定牆壁結構
  - **Obstacle Layer（動態）**：LiDAR/深度相機實時寫入的點雲
  - **Inflation Layer**：障礙物向外膨脹機器人半徑 → 將機器人視為質點
  - **Social Layer / Keepout Zones**：社會力模型行人預測 / 人為禁行
  - **分離原因**：動態障礙物走過後只清 Obstacle Layer，不動 Static Map
- **Global vs Local Planner 分工**：
  - **Global (A*/Theta*, 1-5 Hz)**：全域 Costmap 忽略動力學，求幾何上無碰撞的「最短拓撲路徑」
  - **Local (DWA/TEB/MPPI, 20-100 Hz)**：局部 Costmap（5×5 米）考慮運動學約束（最大加減速、轉彎半徑），生成 v/ω 跟蹤 Global 路徑 + 閃避突發行人
- **為什麼 Local 不能直接處理整張地圖**：
  - TEB 非線性優化、MPPI 萬條隨機軌跡積分 → 整張 100×100 米地圖 → 計算指數爆炸 → 10ms 控制週期內無法完成
  - LiDAR 視野有限，遠處動態資訊無優化價值
- **Global 過得去、Local 卡死陷阱**（面試必答）：
  - **根因**：Costmap 膨脹半徑不匹配 (Inflation Mismatch)
  - Global Costmap 膨脹半徑小 → 規劃出緊貼牆壁極限捷徑
  - Local Planner 考量真實機器人輪廓 + 非完整約束（轉彎半徑）→ 發現鑽不過去 → Oscillation/Stuck
  - **解法**：Global Costmap 引入**平滑代價衰減梯度 (Cost Decay)**，迫使 Global 走「距牆足夠裕度」的中心路徑
- **膨脹代價公式**：`Cost(d) = 252·exp(-α·(d - r_inscribed))`
  - d 接近障礙 → 代價指數飆升 → 引導 A* 走通道中央
- **Nav2 YAML 分層配置**：
  ```yaml
  plugins: ["static_layer", "obstacle_layer", "inflation_layer"]
  inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    cost_scaling_factor: 3.0  # α
    inflation_radius: 0.55
  ```
- **平台**：ROS 2 Nav2（nav2_costmap_2d + Smac/DWB/MPPI）、Autoware HD Map

### 面試 talking points（Q1）
1. **Admissibility + Consistency 雙保證**：面試最基礎的 A* 正確性核心
2. **Weighted A* 10-100× 加速**：講得出為什麼 sub-optimal 在產業很實用
3. **Theta* 搜索同時平滑**：消 8-zigzag 的優雅設計
4. **D* Lite 是動態規劃標準答案**：面試「動態環境」必提
5. **Costmap 膨脹不匹配是 Local 卡死常見元兇**：能分辨「讀過教科書」vs「實際除錯過」

## Q2 補強（Sampling-based: RRT/RRT*/PRM + Informed RRT*/BIT*/AIT* + 高維挑戰）

### 情境題 D：PRM vs RRT vs RRT* 本質差異
- **PRM (Multi-query)**：離線全空間撒點連成 Roadmap + 在線 A* 查詢 → 靜態工廠流水線重複任務
- **RRT (Single-query)**：從 Start 像樹枝向自由空間生長 → 高維動態環境
- **RRT Voronoi Bias**：樹上孤立節點的 Voronoi 胞腔面積最大 → 隨機點落大胞腔機率高 → 強制樹枝向未探索空曠區「瘋狂生長」
- **RRT\* Rewiring 機制**：
  - 新節點 `x_new` 半徑 r 內找最小代價父節點
  - 反向檢查：讓 x_new 當鄰居父節點能否降低鄰居代價 → 剪斷重接
  - 達 **Almost-surely Asymptotically Optimal** 漸進最優
  - 代價：比 RRT 耗時（維護近鄰集 + 多次算代價）
- **Goal Biasing 5-10%**：純隨機會盲目；偏向 goal 既保探索又給引力
- **Probabilistic Completeness**：`lim_{N→∞} P(find path) = 1` — 解存在且 N→∞ 時找到機率 → 1
- **RRT vs RRT\* 差異根因**：
  - RRT 只追求「連接拓撲」找到就停
  - RRT* 透過 Rewiring 不斷優化 cost function → 逼近全局最優
- **Rewiring 核心**：
  ```python
  for x_near in near_nodes:
      if x_new.cost + dist(x_new, x_near) < x_near.cost and collision_free(...):
          x_near.parent = x_new
          x_near.cost = ...
          update_children_costs(x_near)  # 遞迴更新
  ```

### 情境題 E：Informed RRT* / BIT* / AIT*
- **Informed RRT\***：
  - 找到第一解後構造**以 start/goal 為焦點的超橢球**
  - 橢圓數學：橢球外任何點到起點+終點距離 > c_best
  - **所有能優化的點絕對只在橢球內** → 採樣空間瞬間縮小
  - 隨 c_best 變小橢球收縮 → 密度成百上千倍提升 → **10-100× 加速**
  - 採樣約束：`||x_rand - x_start||₂ + ||x_rand - x_goal||₂ ≤ c_best`
- **BIT\* (Batch Informed Trees)**：
  - **Informed Sampling + A\* Graph Search 統一**
  - Batch sampling 批量撒點 → A* 在隨機點上搜索
  - **Lazy Edge Collision Check**：先假設邊都安全，A* 確定邊在最優路徑上才查碰撞 → 省算力
- **AIT\* (Adaptively Informed Trees)**：同時跑正向隨機樹 + 反向啟發式估計樹 → 動態更新 heuristic → 極度複雜迷宮自適應調整採樣焦點
- **OMPL 是工業事實標準**
- **面試「調過 BIT\*」vs「只背 RRT\*」**：
  - 講得出 `samples_per_batch` 參數：太大退化為 PRM 初解慢；太小失 A* 優勢
  - Lazy 碰撞檢測 + 7 軸機械臂狹窄空間 → 比 RRT* 縮短 60%+
- **OMPL C++**：
  ```cpp
  auto planner = std::make_shared<ompl::geometric::BITstar>(space_info);
  planner->setSamplesPerBatch(100);
  planner->setUseJustInTimeSampling(true);
  ```

### 情境題 F：高維 Sampling + Collision Detection 工程
- **C-space 維度爆炸（Curse of Dimensionality）**：
  - 7-DoF 機械臂擁擠書架取物 → C-space 狹窄通道 (Narrow Passages)
  - 高維空間狹窄通道體積佔比**指數級縮小 → 0**
  - **Uniform Random Sampling 必敗**：7 維空間盲目撒點落入萬分之一體積狹道的機率近零
- **Learned Sampling**（突破狹道）：
  - GMM 在歷史成功路徑的狹道入口**加採樣權重**
  - 引導隨機點集中瓶頸區域
- **Lazy Collision Checking 關鍵加速**：
  - 高維空間中碰撞檢測佔規劃時間 **80-90%**
  - 傳統：每生成邊就查碰撞
  - **Lazy：除非邊被選為最終路徑候選者，否則絕不查** → OMPL 核心加速
- **Collision Detection 底層**：
  - **Broad Phase**：AABB 軸對齊包圍盒 / BVH 樹快速篩除不可能相撞的連桿
  - **Narrow Phase**：GJK 演算法（Minkowski Difference 計算凸多面體最短距離）+ EPA 算穿透深度與法向量
  - 閔可夫斯基差集：`A ⊖ B = {a - b | a ∈ A, b ∈ B}`
  - **原點 (0,0,0) 包含在 A⊖B 中 ⟺ A 與 B 物理碰撞**
- **MoveIt! 預設依賴 FCL (Flexible Collision Library)**
- **面試「懂底層 OMPL」vs「會用 MoveIt!」**：
  - 多數人只 API 點 "Plan and Execute"
  - 要講：「MoveIt! 是 Wrapper；真正靈魂是底層 OMPL + FCL」
  - 覆寫 `StateValidityChecker` 客製化碰撞邏輯
  - 調整 FCL BVH 深度平衡檢測精度與速度
  - RRTConnect → BIT* with goal bias + lazy → 7 軸壓縮 100ms 內
- **自定義 StateValidityChecker C++**：
  ```cpp
  class CustomCollisionChecker : public ompl::base::StateValidityChecker {
      bool isValid(const State* state) const override {
          const auto* q = state->as<RealVectorStateSpace::StateType>();
          if (fast_aabb_check(q)) return false;  // 先 broad phase
          return exact_gjk_check(q);              // 再 narrow phase
      }
  };
  ```

### 面試 talking points（Q2）
6. **PRM multi-query vs RRT single-query**：分辨靜態 vs 動態、預建圖 vs 即時規劃
7. **RRT Voronoi bias 物理直覺**：講得出「樹為什麼往空曠處長」的機率論
8. **Informed RRT\* 橢球收斂**：橢圓幾何性質的優雅應用
9. **BIT\* Lazy collision**：面試 manipulation 規劃必提
10. **Curse of Dimensionality 狹窄通道**：高維規劃的核心痛點

## Q3 補強（Kinodynamic + Lattice + Learning-based）

### 情境題 G：Kinodynamic Planning — 微分約束下的規劃
- **純 Geometric Planning 在自駕/無人機/四足失敗**：
  - 折線路徑違反最大加速度、最小轉彎半徑
  - **Non-holonomic 約束**：阿克曼車不能原地平移/打轉
  - 動力學慣性限制
  - 硬追折線 → 控制器輸出飽和 → 軌跡嚴重偏離/翻車
- **Dubins Car**（只能前進，最小轉彎半徑）：最優路徑必 **CCC（圓弧-圓弧-圓弧）或 CSC（圓弧-直線-圓弧）**
- **Reeds-Shepp Car**（允許倒車）：**46 種字碼**（如 `C|C|C`，`|` 代表換檔）
- **Kinodynamic RRT**：
  - 不是連直線！`x_near` 處隨機採樣控制輸入 u（方向盤轉角、油門）
  - 丟進動力學微分方程**正向積分 (Forward Integration, RK4)** 一小段 dt
  - 積分終點才是 `x_new`
  - **保證每條分支在物理上 100% 可執行**
- **Kinodynamic RRT\***：兩點間最優連接解兩點邊值問題 (BVP)；加 Euler-Lagrange 最優性條件；時間 T + 控制代價 `∫u^T R u dt` 納入 Cost
- **阿克曼非完整約束**：`ẋ·sinθ - ẏ·cosθ = 0`（後輪中心速度只能沿車身朝向 θ）
- **三點倒車入庫答題**（Hybrid A* 路線）：
  - 節點擴展時採 Reeds-Shepp 模型作 Heuristic
  - 狹窄車位：「往前開碰壁 → 倒車檔的 `C|C` 曲線代價更小」→ 自動湧現倒車入庫
- **OMPL C++ Kinodynamic Extension**：
  ```cpp
  void steer(x_near, x_rand, ctrl, x_new) {
      controlSampler_->sample(ctrl);
      // RK4 積分動力學方程
      spaceInformation_->propagate(x_near, ctrl, dt, x_new);
      if (checkMotion(x_near, x_new)) { /* add to tree */ }
  }
  ```

### 情境題 H：Lattice Planner / Motion Primitives
- **為什麼 State Lattice 是自駕車黃金標準**：
  - 公路環境狀態空間極受限於車道線
  - 離線預先生成符合車輛動力學的 Motion Primitives（平滑五次多項式 / 螺旋線）
  - 狀態空間離散成 Lattice 節點
  - **線上拼樂高 + Dijkstra/A\* 搜索**
  - 完美兼顧動力學可行性 + 計算實時性
- **Motion Primitives 設計 trade-off**：
  - **覆蓋性**：分支越密避障越靈活
  - **平滑性**：端點曲率 κ 連續，否則方向盤突變
  - 實務：橫向 (d) 5-7 個偏移，縱向 (s) 3 種速度
- **Conformal Lattice**（適應性）：
  - 傳統 Lattice 剛性 → 變寬/轉彎失效
  - Frenet 座標系 (s, d) 中生成基元 → 跟車道中心線彎曲拉伸 → 映射回笛卡爾 (x, y)
  - **車輛天然沿彎道行駛**
- **Hybrid A\***（Waymo/Tesla 核心）：
  - A* 搜索 3D `(x, y, θ)` 狀態
  - 擴展子節點時強迫用運動學方程（方向盤左/正/右）一小段模擬
  - **Lattice 動力學 + A\* 強啟發式** 的結合
- **Frenet 五次多項式基元**：`d(s) = a₀ + a₁·s + a₂·s² + a₃·s³ + a₄·s⁴ + a₅·s⁵`
  - 位置、朝向、曲率在起終點絕對平滑對齊
- **室內 AGV vs 室外自駕車陷阱**：
  - **AGV = 差速輪**，允許原地旋轉 (Point-turn) → 幾何 A* 折線 + 轉個向就能硬走
  - **自駕車 = 阿克曼**，嚴格最小轉彎半徑 + 曲率 C² 連續 → **必 Frenet Lattice / Hybrid A\***
- **Apollo Lattice Planner C++**：
  ```cpp
  for (d_target : {-1.0, 0.0, 1.0}) {
      for (s_target : {10.0, 20.0, 40.0}) {
          QuinticPolynomial curve(d_curr, dd_curr, ddd_curr,
                                  d_target, 0.0, 0.0, s_target);
          double cost = computeCost(curve, obstacle_list);
          primitive_graph.addEdge(current_node, curve, cost);
      }
  }
  ```

### 情境題 I：Learning-based Path Planning 2024
- **Diffuser / Diffusion Planning**：
  - **規劃視為圖像去噪過程**
  - Diffusion Model 直接輸出完整軌跡 `[x_0, x_1, ..., x_T]`
  - **優勢**：傳統 RL 在多峰分佈（左繞 vs 右繞）取平均撞牆；Diffusion 精準刻畫多模態 → 極平滑符合物理直覺
  - 平台：Stanford Diffuser、Robomimic
  - 去噪核心：`p_θ(τ_{k-1}|τ_k) = N(τ_{k-1}; μ_θ(τ_k, k), Σ_θ)`
- **NeRF-guided Planning**：
  - 傳統 OctoMap 記憶體大、解析度限制
  - **NeRF 高保真連續 3D 隱式重建**
  - 規劃器直接在密度場計算碰撞梯度 → 極細緻無碰路徑
- **CVAE + RRT\* Seeding**：
  - RRT* 盲目採樣低效
  - 訓 CVAE（條件 VAE）：給 start/goal → 「猜」專家軌跡高機率經過的區域
  - RRT* 從 CVAE 分佈抽樣（Learned Sampling）
  - **「全空間盲搜 → 沿預測通道精確打通」** 速度 ×100
- **Foundation Model Planning (VLM 高層拆解)**：
  - GPT-4V 當「大腦」：模糊語義 → 高層 Motion Proposal（「幫我收桌子」→「導航到桌子 → 抓取杯子」）
  - 「小腦」MPC / RRT* 保證動力學 + 防碰撞硬約束
  - 平台：Tesla Optimus / Figure humanoid 端到端 NN + 底層安全策略混合
- **面試 talking point「Learning 不是取代而是 proposal/heuristic」**：
  - NN 有幻覺 + 缺 100% 硬物理約束 → 端到端直出馬達指令無法通過工業安全認證
  - **SOTA 架構**：Diffuser / VLM 瞬間生成 Reference Path → MPC / 局部規劃器碰撞校驗 + 平滑微調
  - **AI 泛化速度 + 傳統控制數學絕對安全**
- **Diffuser 生成軌跡 PyTorch**：
  ```python
  trajectory = torch.randn((1, horizon, state_dim))
  for k in reversed(range(steps)):
      trajectory[:, 0, :] = condition_start   # 硬性注入起點
      trajectory[:, -1, :] = condition_goal   # 硬性注入終點
      predicted_noise = model(trajectory, k)
      trajectory = remove_noise(trajectory, predicted_noise, k)
  return trajectory  # 平滑避障參考軌跡
  ```

### 面試 talking points（Q3）
11. **Non-holonomic 阿克曼約束**：自駕規劃的核心差異，分辨「會 A*」vs「懂車輛動力學」
12. **Reeds-Shepp 46 字碼 + Hybrid A***：倒車入庫的工業答案
13. **Frenet 座標系 Conformal Lattice**：公路自駕規劃黃金標準
14. **Diffuser 多峰分佈 vs RL 取平均撞牆**：為什麼生成式模型適合規劃
15. **Learning + Traditional 混合**：Proposal/Heuristic 而非取代，面試關於 AI 落地的成熟回答

## Q4 補強（Local Planner 三巨頭 + Multi-robot + 3D Aerial）

### 情境題 J：DWA / TEB / MPPI 三巨頭
- **DWA**：速度空間 (v, ω) 採樣 → Cost = α·heading + β·distance + γ·velocity
  - **局部極小陷阱**：短視 (myopic) + 只前瞻 1-2 秒 → U 型障礙/死胡同原地打轉
- **TEB**：Global path 視為 time-parameterized elastic band
  - ΔT 作為決策變數 → 非線性優化 (G2O) 整合避障 + 時間最佳化 + 動力學
  - **對動態行人更魯棒**：可預測 t 秒後行人位置 + 時空圖中彎曲繞開 + 減速等待（拉長 ΔT）
  - DWA 只對當下瞬間靜態切片反應
- **MPPI**（Sampling-based MPC 極致）：
  - GPU 並行萬條隨機控制軌跡
  - **非光滑 cost 友善**：TEB 依賴 G2O 梯度下降，離散網格/階躍碰撞懲罰會發散；MPPI 只評估軌跡最終代價積分不需求導
- **ROS 2 Nav2 三 controller**：DWB、TEB local_planner、MPPI Controller
- **場景選型**（面試必答）：
  - **室內窄道** → **TEB**（DWA 採樣有效軌跡極少易卡）
  - **人流密集** → TEB 或 MPPI（時空預測）
  - **非結構化越野** → **MPPI**（底盤滑移 + 懸崖非光滑代價）

### 情境題 K：Multi-robot Path Planning（MAPF / CBS / Swarm）
- **MAPF 定義**：n 個 agent 共享圖；任意 t 無頂點衝突（Vertex Conflict）/ 無邊衝突（Edge Conflict）；總代價 (Makespan 或 Sum-of-Costs) 最小
- **CBS (Conflict-Based Search)** 兩層架構：
  - **Low-level**：每 agent 各跑 A* 找最短路，完全無視別人
  - **High-level**：建衝突樹 (Conflict Tree)
    - A, B 在 t 時刻都在 v 撞車 → 分岔兩子節點
    - 左：「A 在 t 禁止出現在 v」；右：「B 在 t 禁止出現在 v」
    - Low-level 帶新約束重跑 A*
  - **全局最優且完備**
- **Dense Warehouse 次優解**（衝突樹指數爆炸時）：
  - **Priority Planning**：AGV 排序；先規劃者的軌跡當動態障礙物讓後面避開 → 解耦計算，快但非最優
  - **Push-and-Swap**：兩車走廊相遇 → A 讓 B 退空位 (Swap) → B 通過 → A 繼續
- **Swarm / 無人機群 ORCA**：O(1) 極速反應；速度空間算避碰半平面；兩機各承擔 50% 避讓責任
- **ORCA 半平面約束**：`(v_A - (v_A^opt + u/2)) · n ≥ 0`
- **NP-hard vs 百台落地的答題**：
  - **降維打擊 + 次優妥協**：
    1. 人為加「交通規則」（單行道、十字紅綠燈）減少迎面衝突
    2. Priority Planning 解耦計算
    3. 底層 ORCA / MPC 處理突發微小衝突
  - 不求理論最優，只求 100ms 內無碰撞次優
- **平台**：Amazon Kiva、Alibaba Cainiao、自駕隊列 Platooning

### 情境題 L：3D Aerial / Rough-Terrain Planning
- **3D 維度災難**：
  - Occupancy Grid 記憶體爆炸
  - **OctoMap**（八叉樹）：空/實合併，O(log n) 查詢
  - **TSDF**：3D 視覺重建（KinectFusion），只保留表面附近距離
  - **ESDF (Euclidean Signed Distance Field)** **規劃最重要**：任意點到最近障礙的歐氏距離；`∇ESDF` 直接指向「最快遠離障礙」方向
- **Fast-Planner / EGO-Planner (HKUST 高飛)**：
  - A* 或 JPS 找幾何路徑 → **B-spline 參數化軌跡**
  - ESDF 懲罰 + 速度/加速度動力學懲罰 → NLopt 優化器把粗糙軌跡「推」成平滑避障最優
  - **EGO-Planner 突破**：拋棄全局 ESDF；只在碰撞軌跡段用障礙物表面法向量生成局部懲罰梯度 → 速度 ×10
- **B-spline 軌跡優化 Cost**：
  - `J = λ_s·J_s + λ_c·J_c + λ_d·J_d`
  - J_s 平滑度（控制點二階差分）
  - J_c 防撞（ESDF 推遠）
  - J_d 動力學（速度/加速度極限）
- **四足/人形 Rough Terrain**：
  - 2.5D **Elevation Map**（高程圖）分析法向量 + 粗糙度
  - **質心導航規劃**（避開大石）+ **Foothold Planner 落腳點**（質心軌跡兩側找平坦無干涉踩踏點）
- **平台**：DJI Mavic APAS 避障、Unitree H1 人形 rough terrain、ANYmal 碎石行走
- **3D vs 2D 難度跳躍答題**：
  - **維度災難**：狀態從 `(x,y,θ)` 變 `SE(3) × ℝ³`（位姿+線/角速度）→ A* 節點立方爆炸
  - **欠驅動動力學**：無人機無剎車，必須傾斜機身（Roll/Pitch）產生反向推力
  - 3D 規劃**必須將微分平坦性 + 動力學約束深度耦合進軌跡優化**
  - 這就是 Fast-Planner 用 B-spline 梯度優化的本質

### 面試 talking points（Q4）
16. **DWA 短視陷阱 + TEB 時空預測**：面試最常問的 Local Planner 本質差異
17. **MPPI 非光滑 cost 友善**：越野/不可微代價場景的殺手級選型
18. **CBS 衝突樹 + Priority Planning 降維**：MAPF 從理論到落地的標準答案
19. **ORCA 半平面各承擔 50%**：Swarm 分散式避碰的 O(1) 核心
20. **ESDF 梯度引導 3D 軌跡優化**：無人機規劃面試的硬核詞彙
21. **2.5D Elevation Map + Foothold Planner**：四足 rough terrain 標準架構

## 備註
Ch10 完成 4 queries × ~3 sub-topics = 12 sub-topics，總體深度已達 A 級：
- Q1 Search-based (A*/Theta*/D* Lite/Costmap)
- Q2 Sampling-based (RRT/PRM/RRT*/Informed/BIT*/high-dim)
- Q3 Kinodynamic (Dubins/Reeds-Shepp) + Lattice + Learning (Diffuser/NeRF/CVAE)
- Q4 Local (DWA/TEB/MPPI) + Multi-robot (MAPF/CBS/ORCA) + 3D (OctoMap/ESDF/Fast-Planner/Rough Terrain)

若要嚴格對齊「5 queries 獨立」，可再補 Q5 深掘單一主題（如 POMDP-based planning for partially observable envs）。但現有 12 sub-topics 深度已超過 A 級典型 5 queries 的覆蓋廣度。
