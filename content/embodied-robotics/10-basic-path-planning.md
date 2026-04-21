---
title: "基礎路徑規劃（A*, RRT, APF）"
prerequisites: ["07-forward-kinematics-dh"]
estimated_time: 90
difficulty: 4
tags: ["path-planning", "a-star", "rrt", "apf", "c-space", "theta-star", "d-star-lite", "informed-rrt-star", "bit-star", "hybrid-a-star", "mppi", "teb", "cbs", "orca", "esdf", "diffuser"]
sidebar_position: 10
---

# 基礎路徑規劃（A*, RRT, APF）

## 你將學到

- 能精確區分 path planning、motion planning、trajectory planning 三個詞，面試不混用
- 遇到「從 A 到 B 要怎麼走」的問題，能根據**維度**、**動力學約束**、**動態性**、**多機器人**立刻判斷該選 A*、Theta*、D* Lite、RRT、RRT\*、Informed RRT\*、BIT\*、Hybrid A\*、Lattice、DWA/TEB/MPPI 還是 Diffuser，講出**完整選型理由鏈**
- 理解 Configuration Space（C-space）為什麼是路徑規劃的核心抽象，以及它如何把碰撞檢測簡化成「點在區域內外」的問題
- 看到 Local Planner 卡死時，能分辨是 **costmap 膨脹不匹配**、**非完整約束被忽略**、**短視 (myopic) 陷阱** 還是 **高維狹窄通道**，並給出具體除錯方向
- 能講清楚 Learning-based planning（Diffuser / NeRF / CVAE / VLM）在產業落地時為什麼是「proposal / heuristic」而非「端到端取代」

## 核心概念

**精確定義**：**Path planning** 是在已知或部分已知的環境中，找出一條從起點到目標的**無碰撞幾何路線**（純空間，不含時間）。它是機器人自主導航的「戰略導航員」— 負責回答「走哪條路」，而非「怎麼走」或「走多快」。

**三個常混淆的詞**：
- **Path planning**：純幾何路線（waypoints 序列），不含速度、加速度
- **Motion planning**：統稱，泛指從起點到目標的任何運動求解（包含幾何 + 動力學）
- **Trajectory planning**：路徑 + 時間參數化（每個 waypoint 何時到達、速度多少）

**Configuration Space（C-space）**：把機器人的所有自由度（關節角、位置、朝向）壓成一個點，障礙物膨脹成 C-space obstacle。這樣碰撞檢測就變成「這個點有沒有落在障礙區域裡」— 幾何問題簡化為集合判斷。2D 移動機器人的 C-space 是 $\mathbb{R}^2$；7-DoF 機械臂的 C-space 是 $\mathbb{R}^7$（或 $T^7$，考慮關節角度環繞）。

**在感知 → 規劃 → 控制閉環的位置**：
- **節點**：**規劃前端**（global planner）+ **規劃後端**（local planner / trajectory optimizer）
- **輸入**：起點配置 $q_{\text{start}}$、目標配置 $q_{\text{goal}}$、環境地圖（occupancy grid / octomap / ESDF / point cloud）、動力學約束（最大加速度、最小轉彎半徑）
- **輸出**：waypoints 序列 $\{q_0, q_1, \dots, q_N\}$（C-space 中的無碰撞路徑）+ 參考速度 / 控制指令
- **下游**：trajectory optimizer（加時間/速度約束）→ controller（追蹤軌跡）→ actuator

**一句話版本**：「路徑規劃是多維迷宮中尋找安全通道的幾何引擎 — 維度、動力學、動態性三個軸共同決定演算法選擇。」

---

### A* 演算法

**核心公式**：

$$
f(n) = g(n) + h(n)
$$

**物理意義**：$g(n)$ 是從起點到節點 $n$ 的已知代價（走了多遠），$h(n)$ 是從 $n$ 到目標的啟發式估計（還要走多遠）。$f(n)$ 就是「經過這個節點的總估計成本」— A* 每次展開 $f$ 最小的節點。

**兩個關鍵性質**（面試雙保證）：
- **Admissibility（可容許性）**：$h(n) \le h^*(n)$ — 啟發式絕不高估真實最短代價。**保證最優性**。高估會讓 A\* 誤以為最優路徑代價太高而跳過
- **Consistency（一致性 / 單調性）**：$h(n) \le c(n, n') + h(n')$（三角不等式）— 從 $n$ 走到鄰居 $n'$ 的估計差不會超過實際邊代價。**保證每節點最多 pop 一次**，整體 $O(N \log N)$ 不重複探索

當 $h(n) = 0$ 時退化為 Dijkstra（無方向均勻擴展，如水波紋向外擴散 — 慢但穩，且保證全局最短）。

**適用場景**：低維離散空間（2D/3D grid map），如倉庫 AGV、室內導航。

**致命弱點**：C-space 維度增加時，grid 節點數指數爆炸（curse of dimensionality）。7-DoF 機械臂的 C-space 若每軸離散 100 格，就有 $100^7 = 10^{14}$ 個節點 — 完全不可行。

---

### Theta\* / D\* Lite（動態與任意角度）

**Theta\* 核心思想**：傳統 A\* 在 8 連通網格上只能 0°/45°/90° 倍數延伸，要走 15° 捷徑必須用水平+對角拼湊，形成**鋸齒 zigzag 路徑** → 機器人扭動 + 磨損 + 耗能。Theta\* 讓父子節點**不必相鄰**，展開時做 **Line-of-Sight (LoS) Check**：若祖父節點直接看得到 $s'$（中間無障礙），就讓 $s'$ 的父節點直接設為祖父，跳過中間節點。**搜索時同步完成 path smoothing**，路徑更接近真實最短歐氏距離。

**D\* Lite 核心思想**（動態環境標準答案）：
- 從 **Goal 反向搜索**
- 每節點維護 $g(n)$（當前代價）+ $\text{rhs}(n)$（基於鄰居的一步前瞻代價）
- 靜態時 $g = \text{rhs}$；感測器發現新障礙 → 只更新局部邊代價 → $g \neq \text{rhs}$ 產生 **Inconsistency**
- 優先佇列只對「不一致」邊界節點重新傳播 → 復用之前的 heuristic 與代價圖
- 比重跑 A\* 快**幾個數量級**

**面試要點**：動態環境絕不重跑 A\*（會卡死 + 原地抖動）。標準答案路線：
1. **D\* Lite 增量式修補** — 僅在感知更新的局部區域重規劃
2. **分層規劃** — Global 拓撲規劃 + Local (TEB/DWA/MPPI) 在速度空間躲動態障礙

---

### RRT（Rapidly-exploring Random Tree）

**核心流程**（每次迭代）：

1. **Random sample**：在 C-space 隨機撒一個點 $q_{\text{rand}}$
2. **Nearest**：找樹上離 $q_{\text{rand}}$ 最近的節點 $q_{\text{near}}$
3. **Steer**：從 $q_{\text{near}}$ 朝 $q_{\text{rand}}$ 方向延伸固定步長，得 $q_{\text{new}}$
4. **Collision check**：$q_{\text{near}} \to q_{\text{new}}$ 這段路有沒有撞到障礙物
5. 沒撞 → 加入樹；撞了 → 丟掉，重抽

**RRT 的 Voronoi Bias（面試物理直覺）**：樹上孤立節點的 Voronoi 胞腔面積最大 → 隨機點落大胞腔的機率高 → 強制樹枝往「空曠未探索區」瘋狂生長。這就是 RRT 為什麼叫「**Rapidly-exploring**」— 幾何上天然偏向覆蓋未探索空間。

**Probabilistic Completeness（機率完備性）**：$\lim_{N \to \infty} P(\text{find path}) = 1$ — 只要解存在且樣本數 $\to \infty$，找到機率 $\to 1$。但**不保證最優**（路徑通常彎彎曲曲）。

**Goal Biasing（5-10%）**：純隨機採樣會盲目；以 5-10% 機率直接採樣 goal 既保探索又給引力，顯著加速收斂。

**RRT\***：在 RRT 基礎上加 **rewiring** — 每次加入新節點 $x_{\text{new}}$ 後：
1. 在半徑 $r$ 內找所有近鄰 $Q_{\text{near}}$
2. **選最佳父節點**：對每個 $q \in Q_{\text{near}}$，若從 $q$ 連到 $x_{\text{new}}$ 代價更低且無碰撞，則改父
3. **反向重接線**：對每個 $q \in Q_{\text{near}}$，若讓 $x_{\text{new}}$ 當 $q$ 的父能降低 $q$ 的代價，則剪斷重接

這讓路徑**漸近最優（almost-surely asymptotically optimal）**，代價是計算量增加（需維護近鄰集 + 多次代價計算）。

$$
r = \gamma \left( \frac{\log n}{n} \right)^{1/d}
$$

**物理意義**：搜索半徑隨樣本數 $n$ 增加而縮小，$d$ 是 C-space 維度。隨 $n \to \infty$ 路徑收斂到全局最優。

**適用場景**：高維 C-space（6-DoF / 7-DoF 機械臂）、複雜障礙物環境。

---

### Informed RRT\* / BIT\* / AIT\*（現代 Sampling-based）

**Informed RRT\***：找到第一個解之後，構造**以 start/goal 為焦點的超橢球**作為採樣區域：

$$
\lVert x_{\text{rand}} - x_{\text{start}} \rVert_2 + \lVert x_{\text{rand}} - x_{\text{goal}} \rVert_2 \le c_{\text{best}}
$$

**物理意義**：橢球外任何點到起點+終點的總距離必定超過當前最優代價 $c_{\text{best}}$，所以**絕不可能改善解** — 只在橢球內採樣，範圍瞬間縮小。隨 $c_{\text{best}}$ 變小橢球收縮，採樣密度成百上千倍提升 → **10-100× 加速**。

**BIT\* (Batch Informed Trees)**：
- **Informed Sampling + A\* Graph Search 統一**
- Batch sampling 批量撒點 → A\* 在隨機點圖上搜索
- **Lazy Edge Collision Check**：先假設邊都安全，只在 A\* 確定邊在最優路徑上時才查碰撞 → 省大量算力（高維空間中碰撞檢測佔規劃時間 80-90%）

**AIT\* (Adaptively Informed Trees)**：同時跑正向隨機樹 + 反向啟發式估計樹，動態更新 heuristic，在極度複雜迷宮中自適應調整採樣焦點。

**OMPL 是工業事實標準**：MoveIt、Nav2 都是 OMPL 的 wrapper。

---

### APF（Artificial Potential Field）

**核心公式**：

$$
F_{\text{total}} = F_{\text{att}} + F_{\text{rep}} = -\nabla U_{\text{att}}(q) - \nabla U_{\text{rep}}(q)
$$

**物理意義**：目標產生「吸引力場」把機器人拉過去，障礙物產生「排斥力場」把機器人推開。機器人沿合力的梯度下降方向走。

- $U_{\text{att}}(q) = \frac{1}{2} k_{\text{att}} \lVert q - q_{\text{goal}} \rVert^2$ — 吸引勢能，越遠越大（彈簧拉力）
- $U_{\text{rep}}(q)$：當距離障礙物小於安全閾值時才有值，越近越大（推力）

**致命弱點**：**局部最小值**（local minima）— 在 U 型障礙物或對稱配置下，吸引力和排斥力抵消，機器人卡住不動。

**適用場景**：常作為**局部規劃器**（local planner）或即時避障層，搭配 A* 等全局規劃器使用。

---

### PRM（Probabilistic Roadmap）

**思路**：離線階段在 C-space 隨機撒大量點，做碰撞檢測後連成路網（roadmap）；線上階段把起點和目標接入路網，用 Dijkstra / A* 查詢最短路。

**適用場景**：**靜態環境 + 多次查詢**（multi-query）。和 RRT 互補 — RRT 是 single-query（每次從頭建樹），PRM 是 multi-query（建一次圖，反覆查）。**工廠流水線重複任務**的標準選擇。

---

### Kinodynamic / Lattice / Hybrid A\*（帶動力學的規劃）

**為什麼純幾何規劃在自駕 / 無人機 / 四足會失敗**：
- 折線路徑違反最大加速度、最小轉彎半徑
- **Non-holonomic 約束**：阿克曼車不能原地平移/打轉，約束方程 $\dot{x} \sin\theta - \dot{y} \cos\theta = 0$（後輪中心速度只能沿車身朝向）
- 動力學慣性限制
- 硬追折線 → 控制器輸出飽和 → 軌跡嚴重偏離甚至翻車

**Dubins Car**（只能前進，最小轉彎半徑）：最優路徑必落在 6 種字碼之一 $\{LSL, RSR, LSR, RSL, LRL, RLR\}$ — 前 4 條是 **CSC** 型（圓弧-直線-圓弧），後 2 條是 **CCC** 型（圓弧-圓弧-圓弧，僅在起終點距離短於特定門檻時會勝出）。

**Reeds-Shepp Car**（允許倒車）：原 paper（Reeds & Shepp 1990）列 48 candidate words，其中 46 種 distinct pattern（學界兩種引法都常見）；`|` 代表換檔方向切換（例如 `C|C|C`）。

**Kinodynamic RRT**（核心物理直覺）：不是連直線！在 $x_{\text{near}}$ 處隨機採樣**控制輸入** $u$（方向盤轉角、油門），丟進動力學微分方程做 **Forward Integration (RK4)** 一小段 $dt$，積分終點才是 $x_{\text{new}}$。**保證每條分支在物理上 100% 可執行**。

**Hybrid A\***（Waymo / Tesla 倒車入庫核心）：
- A* 搜索 3D 狀態 $(x, y, \theta)$
- 擴展子節點時用運動學方程（方向盤左/正/右）積分一小段
- 節點擴展時以 Reeds-Shepp 模型作 Heuristic
- 狹窄車位：「往前開撞牆 → 倒車檔 `C|C` 曲線代價更小」→ **自動湧現三點倒車入庫**

**Frenet Conformal Lattice**（公路自駕黃金標準）：
- 傳統 Lattice 剛性，在彎道 / 變寬車道失效
- 在 Frenet 座標系 $(s, d)$（沿車道縱向 / 橫向）中生成 Motion Primitives
- 映射回笛卡爾座標 → 車輛天然沿彎道行駛
- 五次多項式基元 $d(s) = a_0 + a_1 s + \ldots + a_5 s^5$ 保證位置、朝向、曲率在起終點平滑對齊

**面試陷阱**：
- **室內 AGV = 差速輪**，允許原地旋轉 → 幾何 A* 折線 + 轉向就能硬走
- **自駕車 = 阿克曼**，嚴格最小轉彎半徑 + 曲率 $C^2$ 連續 → **必 Frenet Lattice / Hybrid A\***

---

### Local Planner 三巨頭（DWA / TEB / MPPI）

**DWA（Dynamic Window Approach）**：在速度空間 $(v, \omega)$ 採樣可行控制輸入，對每組控制前瞻模擬 1-2 秒，以 cost 選最佳：

$$
\text{Cost} = \alpha \cdot \text{heading} + \beta \cdot \text{distance} + \gamma \cdot \text{velocity}
$$

**致命弱點**：**短視 (myopic)** — 只前瞻 1-2 秒，遇 U 型障礙、死胡同會原地打轉。

**TEB（Timed Elastic Band）**：把 global path 當成 **時間參數化的彈性帶** (time-parameterized elastic band)，$\Delta T$ 作為決策變數，用非線性優化 (G2O) 同時處理避障 + 時間最佳 + 動力學約束。**對動態行人更魯棒**：可預測 $t$ 秒後行人位置，在時空圖中彎曲繞開 + 減速等待（拉長 $\Delta T$）。

**MPPI（Model Predictive Path Integral）**：Sampling-based MPC 極致形式。GPU 並行萬條隨機控制軌跡，根據最終代價做加權平均輸出。

**MPPI 關鍵優勢：非光滑 cost 友善**。TEB 依賴 G2O 梯度下降，對離散網格 / 階躍碰撞懲罰會發散；MPPI 只評估軌跡最終代價積分，**不需求導** → 越野、崎嶇地形、階躍懲罰函數下的殺手級選型。

**場景選型**（面試必答）：
- **室內窄道** → **TEB**（DWA 在窄道採樣的有效軌跡極少，易卡）
- **人流密集** → **TEB** 或 **MPPI**（時空預測）
- **非結構化越野** → **MPPI**（底盤滑移 + 懸崖非光滑代價）

---

### Multi-robot Path Planning（MAPF / CBS / ORCA）

**MAPF 定義**：$n$ 個 agent 共享圖，任意時刻 $t$ 無 **Vertex Conflict**（同一節點）或 **Edge Conflict**（交換邊）；總代價 (Makespan 或 Sum-of-Costs) 最小。**NP-hard**。

**CBS (Conflict-Based Search)** 兩層架構：
- **Low-level**：每 agent 各跑 A* 找最短路，完全無視別人
- **High-level**：建**衝突樹 (Conflict Tree)**
  - A、B 在 $t$ 時刻都在 $v$ 撞車 → 分岔兩子節點
  - 左：「A 在 $t$ 禁止出現在 $v$」；右：「B 在 $t$ 禁止出現在 $v$」
  - Low-level 帶新約束重跑 A\*
- **全局最優且完備**，但衝突樹可能指數爆炸

**Dense Warehouse 次優解**（百台 AGV 落地答題）：
1. **人為加交通規則** — 單行道、十字紅綠燈，減少迎面衝突
2. **Priority Planning** — AGV 排序，先規劃者的軌跡當動態障礙讓後者避開 → **解耦計算**，快但非最優
3. **底層 ORCA / MPC** — 處理突發微小衝突

**ORCA (Optimal Reciprocal Collision Avoidance)**：無人機群 / swarm 的 $O(1)$ 極速分散式避碰。速度空間算避碰半平面，兩機**各承擔 50%** 避讓責任：

$$
(v_A - (v_A^{\text{opt}} + u/2)) \cdot \mathbf{n} \ge 0
$$

**物理意義**：A 的新速度只能落在「推離對方」的半平面內，每方各退讓一半達成相互避碰。

**Freezing Robot Problem**：人流密集時每個人都當動態障礙 → 可行空間瞬間消失 → 機器人凍結不動。解法：Social Layer（預測行人意圖 + 社會力模型）+ MPPI 時空預測。

---

### 3D Aerial / Rough-Terrain Planning

**3D 維度災難**：Occupancy Grid 記憶體爆炸。解決方案：
- **OctoMap（八叉樹）**：空/實合併，$O(\log n)$ 查詢
- **TSDF (Truncated Signed Distance Field)**：3D 視覺重建（KinectFusion），只保留表面附近距離
- **ESDF (Euclidean Signed Distance Field)**（規劃最重要）：任意點到最近障礙的歐氏距離。**$\nabla \text{ESDF}$ 直接指向最快遠離障礙方向** → 軌跡優化器用梯度直接推遠軌跡

**Fast-Planner / EGO-Planner (HKUST 高飛)**：
1. A* 或 JPS 找初始幾何路徑
2. **B-spline 參數化軌跡**
3. NLopt 優化 cost：

$$
J = \lambda_s J_s + \lambda_c J_c + \lambda_d J_d
$$

**物理意義**：$J_s$ 平滑度（控制點二階差分），$J_c$ 防撞（ESDF 推遠），$J_d$ 動力學（速度 / 加速度極限）。

**EGO-Planner 突破**：拋棄全局 ESDF；只在碰撞軌跡段用障礙物表面法向量生成**局部懲罰梯度** → 速度提升 10×。

**FASTER 雙軌跡**：無人機高速飛行時同時維護「激進軌跡」（全自由空間）+「保守備援軌跡」（已知安全區內 stop-in-place），感知突變時瞬間切換到安全備援。

**四足 / 人形 Rough Terrain**：
- 2.5D **Elevation Map**（高程圖）分析法向量 + 粗糙度
- **質心導航規劃**（避開大石 / 不穩表面）
- **Foothold Planner**（落腳點規劃）：在質心軌跡兩側找平坦無干涉的踩踏點
- 平台：Unitree H1、Boston Dynamics Spot、ANYmal

---

### Learning-based Planning（2024 SOTA）

**核心原則**（面試成熟回答）：**Learning 不是取代，而是 proposal / heuristic**。NN 有幻覺 + 缺 100% 硬物理約束 → 端到端直出馬達指令無法通過工業安全認證。**SOTA 架構**是 Diffuser / VLM 瞬間生成 Reference Path → MPC / 局部規劃器碰撞校驗 + 平滑微調。

**Diffuser / Diffusion Planning**：規劃視為**圖像去噪過程**。Diffusion Model 直接輸出完整軌跡 $[x_0, x_1, \ldots, x_T]$。關鍵優勢：**精準刻畫多峰分佈**（左繞 vs 右繞），傳統 RL 取平均會撞牆。

**NeRF-guided Planning**：NeRF 高保真連續 3D 隱式重建。規劃器直接在密度場計算碰撞梯度，比 OctoMap 更細緻。

**CVAE + RRT\* Seeding**：訓 CVAE（條件 VAE）：給 start/goal → 猜專家軌跡高機率經過的區域。RRT\* 從 CVAE 分佈抽樣 → **全空間盲搜 → 沿預測通道精確打通**，速度提升 100×。

**Foundation Model Planning (VLM 高層拆解)**：GPT-4V 當「大腦」：模糊語義（「幫我收桌子」）→ 高層 Motion Proposal（「導航到桌子 → 抓取杯子」）；「小腦」MPC / RRT\* 保證動力學 + 防碰撞硬約束。平台：Tesla Optimus、Figure humanoid。

---

<details>
<summary>深入：A* 的完整演算法步驟與複雜度分析</summary>

**完整 A* 偽代碼**：

```
OPEN ← priority queue，放入 start（f = h(start)）
CLOSED ← empty set
g[start] = 0

while OPEN 非空:
    n ← OPEN 中 f 值最小的節點
    if n == goal: return reconstruct_path(n)

    OPEN.remove(n)
    CLOSED.add(n)

    for each neighbor m of n:
        if m in CLOSED: continue
        tentative_g = g[n] + cost(n, m)
        if tentative_g < g[m]:     // 找到更好的路
            g[m] = tentative_g
            f[m] = g[m] + h(m)
            parent[m] = n
            if m not in OPEN: OPEN.add(m)
```

**複雜度**：
- 時間：$O(b^d)$，$b$ 是分支因子，$d$ 是最優解深度。好的 $h$ 能大幅減少展開節點數
- 空間：$O(b^d)$（OPEN + CLOSED 都要存），這是 A* 在大地圖的主要瓶頸

**常見啟發函數**（2D grid）：
- **Manhattan distance**：4-connected grid 的 admissible $h$，$h = |x_1 - x_2| + |y_1 - y_2|$
- **Chebyshev distance**：8-connected grid，**僅在斜向代價 = 直向代價 = 1 時** admissible，$h = \max(|dx|, |dy|)$；若採物理真實的 $\sqrt{2}$ 斜向代價會**低估**真實距離，必須改用 Octile（下條）
- **Euclidean distance**：永遠 admissible 但可能不夠 tight，$h = \sqrt{dx^2 + dy^2}$
- **Octile / Diagonal distance**：8-connected grid 搭配 $\sqrt{2}$ 斜向代價時的正確選擇，通式 $h = D \cdot \max(|dx|,|dy|) + (D' - D) \cdot \min(|dx|,|dy|)$（取 $D=1$、$D'=\sqrt{2}$）：$h = \max(|dx|, |dy|) + (\sqrt{2} - 1) \min(|dx|, |dy|)$

**選擇規則**：斜向代價 = 1 → Chebyshev；斜向代價 = $\sqrt{2}$（Nav2 預設）→ Octile；需要與連續空間一致的下界 → Euclidean。

**Weighted A***：$f = g + w \cdot h$，$w > 1$ 時犧牲最優性換速度，解品質在最優解的 $w$ 倍以內（$w$-admissible）。**實務上加速 10-100×**，Nav2 的 `NavFn` planner 用的就是 weighted A*。

**JPS (Jump Point Search)**：在 uniform-cost grid 上加速 A*。核心觀察：大片空曠網格中 A* 會把對稱等價路徑都塞進 Queue，造成大量重複展開。JPS 的**跳躍點規則**跳過無 Forced Neighbors 的均勻網格，只在「必要轉折處」展開節點 → 壓縮 Open List，速度可提升 10×+。限制：只適用於 uniform grid。

**格子解析度權衡**（面試陷阱）：
- 太細 → 維數災難 + 鋸齒路徑
- 太粗 → 狹窄通道被離散化誤差判定碰撞（Narrow passage 陷阱）
- **實務解**：多解析度地圖（Octree / Quadtree）或分層規劃

</details>

<details>
<summary>深入：Theta\* Line-of-Sight 更新與 D\* Lite 增量機制</summary>

**Theta\* 的 LoS 更新**：

```python
def update_vertex(u, s_prime):
    """Theta* 節點展開 — 若祖父能直視 s_prime，跳過 u 直接連祖父"""
    if line_of_sight(u.parent, s_prime):
        # 路徑 1：s_prime 的父節點直接設為 u 的父（裁彎取直）
        new_cost = u.parent.g + euclidean(u.parent, s_prime)
        if new_cost < s_prime.g:
            s_prime.g = new_cost
            s_prime.parent = u.parent
    else:
        # 路徑 2：退化為常規 A* 更新
        new_cost = u.g + euclidean(u, s_prime)
        if new_cost < s_prime.g:
            s_prime.g = new_cost
            s_prime.parent = u
```

**LoS Check 實作**：Bresenham 線段演算法掃過網格，若任一格是障礙則 LoS = False。

**D\* Lite 核心狀態**：

每節點維護兩個值：
- $g(n)$：當前已知從 $n$ 到 goal 的最短代價估計
- $\text{rhs}(n) = \min_{n' \in \text{succ}(n)} (c(n, n') + g(n'))$：基於鄰居的一步前瞻代價

**一致性判斷**：
- $g(n) = \text{rhs}(n)$ → **locally consistent**（一致）
- $g(n) > \text{rhs}(n)$ → **overconsistent**（有更好的路，要更新）
- $g(n) < \text{rhs}(n)$ → **underconsistent**（代價上升，感知到新障礙）

**增量更新流程**：
1. 感測器發現新障礙 → 邊 $(u, v)$ 代價從 $c_{\text{old}}$ 變 $c_{\text{new}}$
2. 更新 $\text{rhs}(u)$
3. 若 $\text{rhs}(u) \neq g(u)$，把 $u$ 放回優先佇列
4. 只傳播受影響區域，未受影響節點保留原 $g$ 值
5. 直到所有鄰接 start 的節點 consistent

**為什麼比重跑 A\* 快幾個數量級**：重跑 A\* 每次從頭展開所有節點 $O(V \log V)$；D\* Lite 只更新受新障礙影響的局部節點，**復用 99% 的搜索樹**。

</details>

<details>
<summary>深入：RRT\* rewiring 完整實作與漸近最優性</summary>

**RRT\* 的 rewiring 步驟**：

```python
def rrt_star_extend(tree, q_rand, r):
    q_near = nearest(tree, q_rand)
    q_new = steer(q_near, q_rand)
    if not collision_free(q_near, q_new):
        return

    Q_near = near_nodes(tree, q_new, radius=r)

    # Step 1: 選最佳父節點
    best_parent = q_near
    best_cost = cost(q_near) + dist(q_near, q_new)
    for q in Q_near:
        c = cost(q) + dist(q, q_new)
        if c < best_cost and collision_free(q, q_new):
            best_parent = q
            best_cost = c

    tree.add(q_new, parent=best_parent, cost=best_cost)

    # Step 2: 反向 rewiring — 新節點可能讓鄰居變短
    for q in Q_near:
        new_cost = best_cost + dist(q_new, q)
        if new_cost < cost(q) and collision_free(q_new, q):
            q.parent = q_new
            q.cost = new_cost
            propagate_cost_to_children(q)  # 遞迴更新子樹
```

**搜索半徑理論公式**：

$$
r = \gamma \left( \frac{\log n}{n} \right)^{1/d}, \quad \gamma > \left(2 \cdot \frac{1 + 1/d}{\xi_d} \right)^{1/d} \cdot \mu(X_{\text{free}})^{1/d}
$$

$\xi_d$ 是單位球體積，$\mu(X_{\text{free}})$ 是自由空間體積。常數 $\gamma$ 確保漸近最優。

**實務變體**：
- **RRT-Connect**：雙向生長（起點和目標各一棵樹），**找到路徑的速度快很多**，但不保證最優。MoveIt 預設就是 RRTConnect，因為機械臂抓取任務通常 good-enough 就好
- **Informed RRT\***：找到初始解後用橢球採樣縮小範圍（見上節）
- **BIT\***：結合 RRT* 和 graph-based 搜索的優點
- **RRT\*-Smart**：在初始解周圍做 biased sampling + 路徑 shortcutting

</details>

<details>
<summary>深入：高維碰撞檢測底層（GJK / EPA / FCL / BVH）</summary>

**為什麼碰撞檢測是高維規劃的瓶頸**：高維空間中碰撞檢測佔規劃時間 **80-90%**。每次 steer 都要檢查新邊是否無碰撞，7-DoF 機械臂在雜亂桌面規劃時，一次 RRT run 可能做 $10^5$+ 次碰撞檢測。

**Broad Phase + Narrow Phase 兩階段**：

1. **Broad Phase**：快速剔除不可能相撞的物體對
   - **AABB (Axis-Aligned Bounding Box)**：軸對齊包圍盒，重疊檢測 $O(1)$
   - **BVH (Bounding Volume Hierarchy)**：樹狀結構，$O(\log n)$ 查詢
   - 對機械臂 — 每個連桿一個包圍盒，連桿自身不會撞（排除）

2. **Narrow Phase**：精確檢測凸物體對
   - **GJK (Gilbert-Johnson-Keerthi)**：用 Minkowski 差集 $A \ominus B = \{a - b \mid a \in A, b \in B\}$ 的性質
   - **核心定理**：$A \cap B \neq \emptyset \iff \mathbf{0} \in A \ominus B$（兩物體相交 ⟺ 原點在 Minkowski 差集內）
   - GJK 疊代建構單純形 (simplex) 包含原點來判斷
   - **EPA (Expanding Polytope Algorithm)**：相交時算穿透深度與法向量

**FCL (Flexible Collision Library)**：MoveIt! 預設碰撞庫，支援：
- AABB / OBB / RSS 包圍盒
- BVH / Octree 空間分割
- GJK / EPA narrow phase
- Mesh-to-mesh / mesh-to-primitive

**Lazy Collision Checking**（BIT\* 核心加速）：傳統 RRT\* 每生成邊就查碰撞；Lazy 做法先假設所有邊都無碰撞，做 A* 搜索找出候選最優路徑，**只對候選路徑上的邊查碰撞**。若無碰撞就確認，否則刪邊重搜。**省下 90%+ 的碰撞檢測**。

**自定義 OMPL StateValidityChecker**：

```cpp
class CustomCollisionChecker : public ompl::base::StateValidityChecker {
public:
    bool isValid(const State* state) const override {
        const auto* q = state->as<RealVectorStateSpace::StateType>();
        // 1) 先做 broad phase（AABB 快速剔除）
        if (fast_aabb_check(q) == COLLISION) return false;
        // 2) 再做 narrow phase（GJK 精確檢測）
        return exact_gjk_check(q);
    }
};
```

**Learned Sampling 突破狹窄通道**：7-DoF 機械臂在擁擠書架取物時，C-space 狹窄通道體積佔比指數級縮小。Uniform Random Sampling 落入狹道的機率近零 → 規劃失敗。解法：訓練 GMM / CVAE 在歷史成功路徑的狹道入口加採樣權重，引導隨機點集中瓶頸區域。

</details>

<details>
<summary>深入：Hybrid A\* 完整展開流程 + 三點倒車入庫</summary>

**Hybrid A\* 的狀態定義**：$(x, y, \theta)$ 連續，但內部用 3D grid 離散化（典型 0.5m × 0.5m × 5°）做 visited check。**關鍵**：不限制節點落在 grid 中心，允許同一 grid cell 內有不同 $(x, y, \theta)$。

**節點擴展**：對每個節點，模擬方向盤左/正/右三種控制各積分 $dt$ 時間：

```python
def expand_node(node, max_steer=0.6, dt=0.3, v=1.0):
    children = []
    for steer in [-max_steer, 0.0, +max_steer]:
        # 阿克曼運動學
        dx = v * cos(node.theta) * dt
        dy = v * sin(node.theta) * dt
        dtheta = v / L * tan(steer) * dt  # L = wheelbase
        new_state = (node.x + dx, node.y + dy, node.theta + dtheta)

        # 同時產生倒車選項
        new_state_reverse = (node.x - dx, node.y - dy, node.theta - dtheta)

        children.extend([new_state, new_state_reverse])
    return children
```

**兩個 Heuristic 結合**：
1. **Non-holonomic without obstacles**：Reeds-Shepp 距離（考慮轉彎半徑但忽略障礙），保證動力學可行性下的最短路徑估計
2. **Holonomic with obstacles**：2D 歐氏 A\* 距離（考慮障礙但忽略動力學），保證避障最短

**實際 $h = \max(h_1, h_2)$** — 取兩者較大者，仍 admissible。

**三點倒車入庫湧現**：狹窄車位場景下：
- 純往前開 → 撞到前方障礙 → 代價極高
- 倒車檔的 `C|C` 曲線（換檔 + 轉彎）→ 雖有倒車代價，但能通過狹窄入口 → 總代價更低
- A* 自動選擇倒車方案 → **「三點倒車入庫」自然湧現**

**Analytic Expansion**：在搜索過程中週期性嘗試從當前節點直接用 Reeds-Shepp 解析解連到 goal（若該段無碰撞就直接結束搜索），加速收斂 10-20×。

</details>

<details>
<summary>深入：MPPI 完整控制律與非光滑代價處理</summary>

**MPPI 的核心思想**：將最優控制問題轉化為機率推斷 — 對每條隨機擾動軌跡做重要性加權。

**演算法完整流程**（每控制週期 10-50 Hz）：

```python
def mppi_control(x_curr, nominal_u, dynamics, cost_fn,
                 K=1000, H=20, sigma=0.5, lambda_=1.0):
    """
    K: 採樣軌跡數（GPU 並行）
    H: 前瞻步數
    sigma: 控制擾動標準差
    lambda_: 溫度（越小越 greedy）
    """
    # 1) 採樣 K 條擾動序列 δu ~ N(0, Σ)
    delta_u = np.random.randn(K, H, u_dim) * sigma

    # 2) Rollout K 條軌跡
    costs = np.zeros(K)
    for k in range(K):  # GPU 並行
        x = x_curr.copy()
        for t in range(H):
            u = nominal_u[t] + delta_u[k, t]
            x = dynamics(x, u)
            costs[k] += cost_fn(x, u)  # 非光滑也 OK！

    # 3) Importance weighting
    beta = costs.min()
    weights = np.exp(-(costs - beta) / lambda_)
    weights /= weights.sum()

    # 4) 加權平均更新 nominal control
    for t in range(H):
        nominal_u[t] += np.sum(weights[:, None] * delta_u[:, t], axis=0)

    return nominal_u[0]  # 只執行第一步，receding horizon
```

**為什麼 MPPI 對非光滑 cost 友善**：
- TEB / G2O 用 Gauss-Newton / Levenberg-Marquardt → **需要 cost 函數可微**
- 碰撞 = 1，無碰撞 = 0 的階躍懲罰 → 梯度為 0（或無窮大），優化器崩潰
- MPPI 只需**評估** cost（前向模擬 + 積分），不需求導 → 任何離散、階躍、不連續代價都能處理

**越野四足 / 自駕車應用**：崎嶇地形的滑移代價、懸崖邊的階躍懲罰、離散交通規則（紅燈禁行）— 全部可以直接寫進 cost 函數。

**Nvidia Isaac 平台的 MPPI**：Robot4Hz、Neural MPPI 等研究用 NN 作為 dynamics，直接從點雲學習越野 rollout。

</details>

<details>
<summary>深入：完整 Python 實作（A* + RRT + APF 三合一，可直接跑）</summary>

```python
import numpy as np
import heapq
from typing import Optional

# ============ A* on 2D Grid ============

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def octile(a: tuple, b: tuple) -> float:
    """Octile distance — admissible for 8-connected grid with √2 diagonal cost."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return max(dx, dy) + (1.4142 - 1.0) * min(dx, dy)

def a_star_grid(grid: np.ndarray, start: tuple, goal: tuple) -> Optional[list]:
    """A* on 2D occupancy grid. grid[r][c]=1 means obstacle.
    8-connected with √2 diagonal cost → uses Octile heuristic (Manhattan would over-estimate)."""
    rows, cols = grid.shape
    open_set = [(octile(start, goal), 0, start)]
    came_from = {}
    g = {start: 0}
    closed = set()
    directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]

    while open_set:
        f_val, g_val, cur = heapq.heappop(open_set)
        if cur in closed:
            continue
        closed.add(cur)
        if cur == goal:
            path = []
            while cur in came_from:
                path.append(cur)
                cur = came_from[cur]
            path.append(start)
            return path[::-1]
        for dr, dc in directions:
            nr, nc = cur[0]+dr, cur[1]+dc
            if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] == 0:
                nb = (nr, nc)
                if nb in closed:
                    continue
                step_cost = 1.414 if abs(dr)+abs(dc)==2 else 1.0
                ng = g_val + step_cost
                if ng < g.get(nb, float('inf')):
                    g[nb] = ng
                    came_from[nb] = cur
                    heapq.heappush(open_set, (ng + octile(nb, goal), ng, nb))
    return None

# ============ RRT* with rewiring ============

class Node:
    def __init__(self, pos, parent=None, cost=0.0):
        self.pos = np.array(pos, dtype=float)
        self.parent = parent
        self.cost = cost

def rrt_star(start, goal, obstacles, bounds, step=0.3, max_iter=3000,
             radius=1.0, goal_bias=0.05):
    def collision_free(p1, p2, n=10):
        for t in np.linspace(0, 1, n):
            pt = p1 + t * (p2 - p1)
            for ox, oy, r in obstacles:
                if np.hypot(pt[0]-ox, pt[1]-oy) < r: return False
        return True

    root = Node(start, cost=0.0)
    nodes = [root]

    for _ in range(max_iter):
        sample = np.array(goal) if np.random.rand() < goal_bias else \
                 np.array([np.random.uniform(bounds[0], bounds[1]),
                           np.random.uniform(bounds[2], bounds[3])])
        nearest = min(nodes, key=lambda n: np.linalg.norm(n.pos - sample))
        direction = sample - nearest.pos
        dist = np.linalg.norm(direction)
        if dist < 1e-6: continue
        new_pos = nearest.pos + direction / dist * min(step, dist)
        if not collision_free(nearest.pos, new_pos): continue

        # Step 1: Find best parent in radius
        near_nodes = [n for n in nodes if np.linalg.norm(n.pos - new_pos) < radius]
        best_parent = nearest
        best_cost = nearest.cost + np.linalg.norm(new_pos - nearest.pos)
        for n in near_nodes:
            c = n.cost + np.linalg.norm(new_pos - n.pos)
            if c < best_cost and collision_free(n.pos, new_pos):
                best_parent, best_cost = n, c

        new_node = Node(new_pos, parent=best_parent, cost=best_cost)
        nodes.append(new_node)

        # Step 2: Rewire neighbors through new_node
        for n in near_nodes:
            c = new_node.cost + np.linalg.norm(n.pos - new_pos)
            if c < n.cost and collision_free(new_pos, n.pos):
                n.parent = new_node
                n.cost = c

        if np.linalg.norm(new_pos - np.array(goal)) < step:
            # Extract path
            path = []
            n = new_node
            while n is not None:
                path.append(n.pos.tolist())
                n = n.parent
            return path[::-1]
    return None

# ============ APF in 2D ============

def apf_2d(start, goal, obstacles, k_att=1.0, k_rep=100.0, d0=1.0,
           step=0.05, max_iter=1000, tol=0.1):
    pos = np.array(start, dtype=float)
    path = [pos.copy()]

    for _ in range(max_iter):
        if np.linalg.norm(pos - np.array(goal)) < tol:
            return [p.tolist() for p in path]

        f_att = -k_att * (pos - np.array(goal))
        f_rep = np.zeros(2)
        for ox, oy, r in obstacles:
            obs = np.array([ox, oy])
            diff = pos - obs
            dist = np.linalg.norm(diff) - r
            dist = max(dist, 0.01)
            if dist < d0:
                f_rep += k_rep * (1.0/dist - 1.0/d0) * (1.0/dist**2) * (diff / np.linalg.norm(diff))

        f_total = f_att + f_rep
        pos = pos + step * f_total / (np.linalg.norm(f_total) + 1e-6)
        path.append(pos.copy())

    return [p.tolist() for p in path]  # may not have reached goal (local min!)

# ============ Demo ============
if __name__ == "__main__":
    grid = np.zeros((20, 20), dtype=int)
    grid[5:15, 10] = 1
    path = a_star_grid(grid, (2, 2), (18, 18))
    print(f"A* path length: {len(path) if path else 'No solution'}")

    obs = [(5, 5, 1.0), (3, 8, 0.8)]
    p_rrt = rrt_star([0, 0], [10, 10], obs, bounds=(0, 12, 0, 12))
    print(f"RRT* path length: {len(p_rrt) if p_rrt else 'No solution'}")

    p_apf = apf_2d([0, 0], [10, 10], obs)
    print(f"APF path length: {len(p_apf)}")
```

</details>

<details>
<summary>深入：Diffuser 軌跡生成 PyTorch 骨架</summary>

```python
import torch
import torch.nn as nn

class Diffuser(nn.Module):
    """將規劃視為圖像去噪，直接輸出平滑軌跡"""
    def __init__(self, state_dim, horizon, n_diffusion_steps=100):
        super().__init__()
        self.horizon = horizon
        self.state_dim = state_dim
        self.n_steps = n_diffusion_steps
        # 1D U-Net backbone（實際實作用 temporal convolutions）
        self.denoiser = TemporalUNet(state_dim)

    def sample(self, cond_start, cond_goal):
        """從純噪聲開始，逐步去噪得到軌跡"""
        tau = torch.randn((1, self.horizon, self.state_dim))

        for k in reversed(range(self.n_steps)):
            # 硬性注入條件（start / goal）— 每步都重設兩端
            tau[:, 0, :] = cond_start
            tau[:, -1, :] = cond_goal

            # 預測噪聲
            noise_pred = self.denoiser(tau, k)

            # 去一步噪
            tau = remove_noise_step(tau, noise_pred, k)

        return tau  # 平滑避障參考軌跡

def plan_with_diffuser(diffuser, start, goal, mpc_refiner):
    """SOTA 混合架構：Diffuser 生成 proposal → MPC 硬約束校驗"""
    ref_traj = diffuser.sample(start, goal)             # NN proposal
    safe_traj = mpc_refiner.optimize(ref_traj, obstacles)  # 物理約束
    return safe_traj
```

**為什麼多峰分佈很重要**：傳統 RL 用 MSE loss 會把「左繞」和「右繞」兩個策略取平均 → 直接撞牆中央。Diffuser 的去噪過程天然保留多模態分佈 → 每次採樣都給一條合理路徑。

</details>

## 直覺理解

**三個基礎類比**：

1. **A\* = GPS 導航**：你開車用 Google Maps，它會計算所有可能路線的「預估總時間」（= $f$），然後推薦最快的。$h$ 就是 Maps 對「剩餘時間」的估計 — 估得越準，搜索越快；但絕不能高估（不然會漏掉真正最快的路）

2. **RRT = 探險家隨機探索**：你在一片未知森林裡找出口。每一步你隨機朝一個方向丟一塊石頭，然後朝那個方向走一段看看有沒有路。走得通就標記，走不通就丟另一塊石頭。最終你會覆蓋整片森林 — 但走出的路線不會是最短的

3. **APF = 磁鐵吸引 + 同極排斥**：目標是一塊大磁鐵在吸你，障礙物是同極磁鐵在推你。你沿著合力方向走。但如果你被夾在兩塊排斥磁鐵中間、剛好對著目標的吸力被抵消 — 你就卡住了（local minimum）

**進階類比**：

4. **Informed RRT\* = 考場限時**：第一次解題用了 60 分鐘，之後就只在「60 分鐘內可能寫完」的範圍內重抽題目 — 不可能更短的選項全部跳過
5. **D\* Lite = 修復路由表**：網際網路路由器發現某條線路斷了，不會整張全球路由表重算，只更新受影響的局部。路徑規劃的「incremental search」就是這個思想
6. **Hybrid A\* = 不能原地轉身的駕駛員**：普通 A* 可以瞬移，但車子不行 — 每次展開子節點只能「方向盤左打 / 正前 / 右打」開一段距離
7. **MPPI = 十萬隻猴子丟骰子**：GPU 並行十萬條隨機控制軌跡，最後按代價加權平均 → 智慧從群體統計湧現
8. **Diffuser = 雕刻家從雜石中鑿出佛像**：初始軌跡是純噪聲，每一步去噪都讓軌跡更像「合理路徑」，最終湧現避障平滑解

**模擬器觀察**：

- **Gazebo + Nav2**：啟動 `nav2_bringup`，在 Rviz2 用 `2D Goal Pose` 設定目標。打開 global costmap 的 inflation layer visualization。觀察 A*（NavFn）展開的節點（`publish_potential: true` 看勢能場），以及 DWA / TEB / MPPI local planner 如何在 waypoints 間做即時避障
- **MoveIt + OMPL**：在 7-DoF 機械臂場景中跑 RRTConnect / RRT* / BIT*，觀察 C-space 採樣點如何逐漸填滿可行空間。把障礙物加密觀察規劃時間如何隨 clutter 上升
- **CARLA 自駕模擬**：實作 Frenet Lattice Planner — 在 Frenet 座標系生成多條橫向偏移的五次多項式，根據碰撞、偏移、速度代價選最佳。觀察過彎時 Lattice 如何自然貼合車道曲線
- **Isaac Sim + 無人機**：跑 Fast-Planner（B-spline + ESDF），觀察軌跡優化器如何從粗糙 A* 折線「推」成平滑避障軌跡
- **2D Python 動畫**：matplotlib 畫 A* 節點展開 + RRT* 樹生長 + rewiring 過程，是建立直覺最快的方式

## 實作連結

**六個典型工程場景**：

1. **ROS 2 Nav2 室內導航**：移動機器人在 2D occupancy grid 上用 A*（NavFn / Smac Planner）算全局路徑，再交給 DWB / TEB / MPPI Controller 做動態避障。這是分層架構的經典範例 — 全局最優 + 局部即時

2. **MoveIt 機械臂抓取**：7-DoF 機械臂在雜亂桌面抓取物體。MoveIt 底層用 OMPL，預設 RRTConnect。需要更優路徑時切 BIT\* + Lazy collision

3. **倉儲多機器人調度（Amazon Kiva）**：上百台 AGV 共享地圖，用 Priority Planning + CBS 解衝突，底層 ORCA 處理突發微小衝突

4. **自駕車 Hybrid A\* 泊車 + Frenet Lattice 公路**：Waymo / Tesla 在車位狹窄時用 Hybrid A* 自動湧現三點倒車入庫；公路上用 Frenet Lattice 五次多項式基元

5. **四足 / 人形 rough terrain（Unitree H1）**：Elevation Map + 質心規劃 + Foothold Planner 兩層分工，避開大石 + 規劃落腳點

6. **無人機 Fast-Planner 高速飛行（DJI APAS）**：A\* → B-spline 軌跡優化 → NLopt 用 ESDF 梯度推遠障礙 + 動力學約束

**Code 骨架**（ROS 2 Nav2 Layered Costmap 設定）：

```yaml
# nav2_params.yaml — Global Costmap 分層配置
global_costmap:
  ros__parameters:
    resolution: 0.05
    robot_radius: 0.22
    plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

    static_layer:
      plugin: "nav2_costmap_2d::StaticLayer"
    obstacle_layer:
      plugin: "nav2_costmap_2d::ObstacleLayer"
      observation_sources: "scan"
      scan:
        topic: /scan
        raytrace_max_range: 3.0
        obstacle_max_range: 2.5
    inflation_layer:
      plugin: "nav2_costmap_2d::InflationLayer"
      cost_scaling_factor: 3.0      # α — 膨脹代價衰減速度
      inflation_radius: 0.55        # 機器人半徑 + 安全餘量
```

**Code 骨架**（OMPL BIT\* for 7-DoF manipulation）：

```cpp
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

auto space = std::make_shared<ob::RealVectorStateSpace>(7);  // 7-DoF
space->setBounds(-M_PI, M_PI);

auto ss = std::make_shared<og::SimpleSetup>(space);
ss->setStateValidityChecker([&](const ob::State* s) {
    // 自定義碰撞檢測 — broad + narrow phase
    return custom_collision_check(s);
});

auto planner = std::make_shared<og::BITstar>(ss->getSpaceInformation());
planner->setSamplesPerBatch(100);
planner->setUseJustInTimeSampling(true);
planner->setPruneThresholdFraction(0.05);

ss->setPlanner(planner);
ss->solve(5.0);  // 5 秒規劃預算
```

**Code 骨架**（Hybrid A\* 節點擴展）：

```python
def hybrid_a_star_expand(node, dt=0.3, v=1.0, L=2.5):
    """阿克曼運動學積分 — 自駕泊車核心"""
    children = []
    for steer in [-0.6, 0.0, 0.6]:
        for direction in [+1, -1]:  # 前進 + 倒車
            dx = direction * v * math.cos(node.theta) * dt
            dy = direction * v * math.sin(node.theta) * dt
            dtheta = direction * v / L * math.tan(steer) * dt
            children.append(State(
                node.x + dx, node.y + dy, node.theta + dtheta,
                parent=node,
                cost=node.cost + 1.0 + (0.5 if direction < 0 else 0)
            ))
    return children
```

## 常見誤解

1. **「RRT 找的就是最短路徑」** — 錯。RRT 只保證**機率完備**（能找到一條路），但路徑通常彎彎曲曲、遠非最優。要漸近最優得用 **RRT\***（加 rewiring）。面試時被問「RRT 和 RRT\* 差在哪」，答案就是 rewiring 帶來的漸近最優性。

2. **「APF 可以當全局規劃器用」** — 危險。APF 有**局部最小值**問題，在 U 型障礙物、死角、對稱配置下會卡住。業界做法是 APF 只當**局部避障層**，全局路徑由 A* 或 RRT 負責。Nav2 的分層架構（global planner + local planner）就是這個思路。

3. **「A* 萬能，什麼場景都能用」** — 在高維 C-space 完全不可行。7-DoF 機械臂的 C-space 是 7 維，grid 離散化後節點數是指數級。這就是為什麼 MoveIt 用 sampling-based 方法（RRT / PRM），而不是 A*。**維度決定演算法選擇**，這是路徑規劃最重要的一條判斷原則。

4. **「Path planning 和 trajectory planning 是一回事」** — 不是。Path 只有幾何（走哪裡），trajectory 加了時間（什麼時候到、速度多少）。規劃 pipeline 是 path → trajectory → control，三層分開。

5. **「Global planner 出的路徑 local planner 一定能跟」** — 錯。**Costmap 膨脹不匹配 (Inflation Mismatch)** 是 Local 卡死最常見元兇：Global costmap 膨脹半徑小 → 規劃出緊貼牆壁的捷徑；Local planner 考量真實機器人輪廓 + 非完整約束（轉彎半徑）→ 發現鑽不過去 → Oscillation / Stuck。解法：Global costmap 引入**平滑代價衰減梯度** (Cost Decay)，迫使 Global 走「距牆足夠裕度」的中心路徑。

6. **「A* 在動態環境下只要感測器一更新就重跑一次就好」** — 災難。每秒重跑 A* 會卡死 + 原地抖動。正確答案：**D\* Lite 增量式修補** 只更新受新障礙影響的局部節點，復用 99% 搜索樹；或 **分層規劃** — Global 慢更新（1-5 Hz）+ Local 快反應（20-100 Hz）。

7. **「Learning-based planning 會完全取代傳統方法」** — 產業不這樣看。NN 有幻覺 + 缺硬物理約束，端到端直出馬達指令無法通過安全認證。**SOTA 架構是 Diffuser / VLM 生成 proposal → MPC / 局部規劃器碰撞校驗硬微調**。AI 提供泛化速度 + 傳統控制提供絕對安全。

## 練習題

<details>
<summary>Q1：倉庫裡一台 AGV 要從貨架 A 走到貨架 B，環境是靜態的 2D 地圖，你會用什麼演算法？為什麼？</summary>

**完整推理鏈**：

1. **判斷維度**：AGV 在 2D 平面移動，C-space 是 $(x, y)$ 或 $(x, y, \theta)$，最多 3 維 → grid-based 方法可行
2. **判斷場景**：靜態環境、單次查詢、需要最短路 → **A\*** 是首選
3. **啟發函數選擇**：如果是 4-connected grid 用 Manhattan distance；8-connected 用 diagonal distance；都是 admissible 的
4. **膨脹半徑**：要把障礙物按 AGV 的物理尺寸做 inflation（膨脹），這樣 AGV 就可以當成一個點來規劃。設定 `inflation_radius = robot_radius + safety_margin`
5. **要避開的陷阱**：如果地圖很大（如 1000x1000），vanilla A* 會展開太多節點。可以用 **JPS**（Jump Point Search）加速（10×+），或 **Weighted A\***（$w \in [1.2, 2.0]$）犧牲一點最優性換速度
6. **若需要平滑路徑**：用 **Theta\*** 取代 A\*，搜索時同步做 Line-of-Sight 裁彎取直，消掉 8-zigzag

**結論**：A\*（低維 + 最優保證 + 靜態環境），配合 inflation layer 和適當的啟發函數。大地圖用 JPS 加速；需要平滑路徑用 Theta\*。

</details>

<details>
<summary>Q2：7-DoF 機械臂要在雜亂桌面上抓取一個杯子，桌上有很多障礙物，你會用什麼規劃方法？</summary>

**完整推理鏈**：

1. **判斷維度**：7-DoF → C-space 是 7 維 → grid-based 方法指數爆炸（$100^7 = 10^{14}$），**必須用 sampling-based**
2. **單次 vs 多次查詢**：抓取任務通常是 single-query（每次目標不同）→ **RRT 系列**優於 PRM
3. **最優性需求**：機械臂通常不需要「全局最短路徑」，而是「夠好的無碰撞路徑」→ **RRTConnect**（雙向 RRT，速度快）是 MoveIt 的預設選擇
4. **若場景狹窄** (Narrow Passage)：C-space 狹道體積佔比指數級小 → uniform sampling 失效。切換到 **BIT\* + Learned Sampling**（CVAE seeding）在歷史成功路徑的狹道入口加權重
5. **碰撞檢測瓶頸**：高維規劃 80-90% 時間花在碰撞檢測。MoveIt 用 **FCL** (Flexible Collision Library)，broad phase（AABB）+ narrow phase（GJK/EPA）。**Lazy Collision Checking**（BIT\* 核心）先假設邊安全，只在候選最優路徑上查 → 省 90%+ 碰撞檢測
6. **路徑後處理**：規劃出的路徑通常需要 **shortcutting + smoothing**，否則軌跡會有不必要的彎繞

**結論**：RRTConnect（MoveIt/OMPL 預設，速度快），複雜狹窄場景切 BIT\* + Lazy collision + CVAE seeding。配合 FCL 碰撞檢測和路徑 shortcutting。若場景固定且需多次查詢，考慮 PRM。

</details>

<details>
<summary>Q3：你用 Nav2 做全局規劃，Global A\* 順利出路徑，但 Local Planner（TEB/DWA）在窄走廊裡 oscillation / 卡死，怎麼診斷？</summary>

**完整推理鏈（面試必答題）**：

1. **根因判斷**：90% 是 **Costmap 膨脹不匹配 (Inflation Mismatch)**
   - Global costmap 用小膨脹半徑 → 規劃出緊貼牆壁的極限捷徑
   - Local Planner 考量真實機器人輪廓 + 非完整約束（轉彎半徑）→ 發現鑽不過去 → 震盪

2. **除錯步驟**：
   - 在 Rviz 打開 global / local costmap 的 inflation layer visualization
   - 比對兩者的 `inflation_radius` — 若 Global 小 Local 大就是元兇
   - 檢查 `cost_scaling_factor` — 太小會讓遠離障礙物的地方也高 cost

3. **解法**：**Global Costmap 引入平滑代價衰減梯度 (Cost Decay)**
   - 公式（Nav2 `InflationLayer`）：$\text{Cost}(d) = (\text{INSCRIBED\_INFLATED\_OBSTACLE} - 1) \cdot \exp(-\alpha \cdot (d - r_{\text{inscribed}})) = 252 \cdot \exp(-\alpha \cdot (d - r_{\text{inscribed}}))$，僅在 $r_{\text{inscribed}} < d \leq r_{\text{inflation}}$ 區間有效；$0 < d \leq r_{\text{inscribed}}$ 設為 253（`INSCRIBED_INFLATED_OBSTACLE`）；$d = 0$（障礙格本身）設為 254（`LETHAL_OBSTACLE`）
   - $d$ 接近障礙 → 代價指數飆升 → 引導 A* 走通道中央
   - 典型值：`cost_scaling_factor: 3.0`, `inflation_radius: 0.55`（機器人半徑 0.22m 時）

4. **若 Local Planner 是 DWA**：DWA 短視，窄道採樣到有效軌跡的機率低 → **換 TEB 或 MPPI**。TEB 用非線性優化整合避障+時間最佳化，在窄道鑽縫能力遠勝 DWA；MPPI 萬條軌跡並行採樣在非光滑 cost 場景更魯棒

5. **檢查 recovery 行為**：Nav2 的 `recovery_server` 提供 `ClearCostmap / Spin / BackUp`。若 recovery 被禁用，小擾動後會直接卡死

**結論**：Global 加 Cost Decay 迫使走中央 + 選對 Local Planner（窄道用 TEB，非光滑用 MPPI）+ 確保 recovery 行為啟用。能分辨「讀過教科書」vs「實際除錯過」的分水嶺題。

</details>

<details>
<summary>Q4：你要設計一個 100 台 AGV 的倉儲調度系統，怎麼做 multi-robot path planning？</summary>

**完整推理鏈**：

1. **問題本質**：MAPF (Multi-Agent Path Finding) 是 **NP-hard**。$n$ agent 共享圖、無 Vertex/Edge Conflict、最小化 Makespan 或 Sum-of-Costs

2. **理論最優方案**：**CBS (Conflict-Based Search)**
   - Low-level：每 agent 各跑 A\*
   - High-level：建衝突樹，遇衝突分岔兩子節點加約束重搜
   - 保證全局最優且完備，但衝突樹在 100 agent 時**指數爆炸** → 10 分鐘都解不出來

3. **產業落地：降維 + 次優妥協**
   - **人為加交通規則**：單行道、十字紅綠燈 → 減少迎面衝突 80%+
   - **Priority Planning**：給每台 AGV 優先級，先規劃者的軌跡當動態障礙讓後者避開 → **解耦計算**，快但非最優
   - **底層 ORCA / MPC**：處理突發微小衝突，$O(1)$ 極速分散式避碰

4. **ORCA 核心物理直覺**：兩機在速度空間各承擔 50% 避讓責任 — $(v_A - (v_A^{\text{opt}} + u/2)) \cdot \mathbf{n} \ge 0$，天然對稱分散式

5. **Freezing Robot 陷阱**：人流密集時每個人都當動態障礙 → 可行空間瞬間消失 → 機器人凍結。解法：Social Layer（社會力模型 + 行人意圖預測）+ MPPI 時空預測

6. **平台參考**：Amazon Kiva（Priority Planning）、Alibaba Cainiao、自駕隊列 Platooning

**結論**：不追求理論最優，只求 100ms 內無碰撞次優。**層次架構**：Traffic rules（預防）→ Priority Planning（離線解耦）→ ORCA（線上微調）。

</details>

<details>
<summary>Q5：你要讓 Waymo 級別的自駕車完成「狹窄車位三點倒車入庫」，怎麼規劃？</summary>

**完整推理鏈**：

1. **為什麼不能用普通 A\***：普通 A* 允許節點任意相鄰，但車輛有**非完整約束 (Non-holonomic)** — 阿克曼運動學 $\dot{x} \sin\theta - \dot{y} \cos\theta = 0$，不能原地平移/打轉

2. **核心方法：Hybrid A\***
   - 狀態：$(x, y, \theta)$ 連續，內部 3D grid 離散化做 visited check
   - 節點擴展：模擬方向盤左/正/右三種控制各積分 $dt$ 時間 → 保證每條分支物理可執行
   - **關鍵**：加入「倒車」選項（$v < 0$），但給 cost penalty（鼓勵前進優先）

3. **Heuristic 兩選一取最大**：
   - $h_1$ = Reeds-Shepp 距離（考慮轉彎半徑，忽略障礙）
   - $h_2$ = 2D 歐氏 A* 距離（考慮障礙，忽略動力學）
   - $h = \max(h_1, h_2)$ 仍 admissible

4. **倒車入庫湧現機制**：
   - 狹窄車位中，純往前 → 撞前方障礙 → cost 極高
   - 倒車檔 $C|C$ 曲線（換檔 + 轉彎）→ 雖有倒車 penalty，但通過狹窄入口 → 總 cost 更低
   - A* 自動選擇倒車方案 → **三點倒車入庫自然湧現**，不需寫死規則

5. **Analytic Expansion 加速**：週期性嘗試從當前節點直接用 Reeds-Shepp 解析解連到 goal（若該段無碰撞就結束），加速 10-20×

6. **Reeds-Shepp 46 種字碼**：如 `C|C|C`、`CSC`，`|` 代表換檔方向切換。這就是 Waymo / Tesla 工業界倒車入庫規劃的標準答案

**結論**：Hybrid A* + Reeds-Shepp heuristic + Analytic Expansion。倒車入庫是**從 cost function 湧現**的，不是寫死的規則。這題能分辨「會 A*」vs「懂車輛動力學」。

</details>

<details>
<summary>Q6：無人機在未知 3D 環境高速飛行，感測突變時還要能瞬間避障，怎麼設計規劃架構？</summary>

**完整推理鏈**：

1. **3D 維度災難**：Occupancy Grid 記憶體爆炸 → 用 **OctoMap**（空/實合併，$O(\log n)$）或 **ESDF**（任意點到障礙歐氏距離，$\nabla \text{ESDF}$ 直接給避障梯度）

2. **兩層規劃架構**：
   - **前端**：A* 或 JPS 在 OctoMap 找粗糙幾何路徑
   - **後端**：**B-spline 軌跡優化**（Fast-Planner / EGO-Planner）
     - Cost = $\lambda_s J_s + \lambda_c J_c + \lambda_d J_d$
     - $J_s$ 平滑度、$J_c$ 防撞（ESDF 推遠）、$J_d$ 動力學
     - NLopt 優化器把粗糙軌跡「推」成平滑避障最優

3. **EGO-Planner 突破**：拋棄全局 ESDF；只在碰撞軌跡段用障礙物表面法向量生成**局部懲罰梯度** → 速度 ×10

4. **高速飛行突變處理：FASTER 雙軌跡**
   - 激進軌跡：全自由空間，飛快但風險高
   - 保守備援軌跡：已知安全區內 stop-in-place，可緊急停
   - 感知突變時瞬間切換到安全備援 → **保證可行性 (Recursive Feasibility)**

5. **欠驅動動力學**：無人機無剎車，必須傾斜機身（Roll/Pitch）產生反向推力 → 3D 規劃必須將**微分平坦性 + 動力學約束深度耦合進軌跡優化**。B-spline 的控制點天然滿足微分平坦性質

6. **Learning-based 加速**：用 **CVAE** 學專家飛行軌跡，RRT* 從 CVAE 分佈 seeding，速度 ×100。或 **Diffuser** 直接生成參考軌跡

**結論**：OctoMap/ESDF 表達環境 + A* 前端 + B-spline 後端優化 + FASTER 雙軌跡保證安全 + 可選 Learning-based proposal 加速。平台：DJI Mavic APAS、HKUST Fast-Planner。

</details>

## 面試角度

1. **維度決定演算法** — 這是路徑規劃最核心的判斷原則。面試時帶出：「我選擇規劃演算法的第一個問題永遠是 C-space 幾維。2D/3D 用 A* / Theta\*，6-DoF 以上必須用 sampling-based（RRT / BIT\*），因為 grid 離散化在高維會指數爆炸。」**為什麼這是重點**：這展現你理解演算法選擇背後的**複雜度根因**，不是隨便套用。

2. **Admissibility + Consistency 雙保證** — 面試最基礎的 A* 正確性核心。帶出：「Admissibility $h \le h^*$ 保證最優性（不會錯過最佳路徑），Consistency 三角不等式保證每節點最多 pop 一次（不重複展開）。」**為什麼這是重點**：區分「會調 API」和「真正理解演算法」，也是 Weighted A* 失最優性但加速 10-100× 的理論基礎。

3. **Weighted A* sub-optimal 在產業實用** — 帶出：「$f = g + w \cdot h$ 當 $w > 1$ 時犧牲最優換速度，解品質在最優解 $w$ 倍內。Nav2 的 NavFn 就是 weighted A\*。大地圖規劃時間從 500ms 壓到 50ms，產業永遠買單。」**為什麼這是重點**：展現**工程 trade-off 思維**，不是死守理論最優。

4. **Theta\* 搜索同時平滑** — 帶出：「傳統 A* 在 8 連通 grid 走不出 15° 捷徑，要拼水平+對角成 zigzag。Theta* 的 Line-of-Sight 檢查讓父子節點不必相鄰，搜索時同步完成 path smoothing。」**為什麼這是重點**：消除 8-zigzag 的優雅設計，面試考官會追問 LoS 具體怎麼做 (Bresenham)。

5. **D\* Lite 是動態環境標準答案** — 帶出：「動態環境絕不重跑 A\*（會卡死+抖動）。D* Lite 反向搜索 + rhs/g 一致性檢查，只更新受影響邊，復用 99% 搜索樹，快幾個數量級。」**為什麼這是重點**：面試「動態環境」必提的標準答案，展現 incremental search 的理解深度。

6. **PRM multi-query vs RRT single-query** — 帶出：「PRM 離線建 roadmap 適合靜態工廠重複任務；RRT 從 start 向自由空間生長適合高維動態。RRT 的 Voronoi bias 天然往空曠處長是機率論的優雅性質。」**為什麼這是重點**：展現 sampling-based 的**設計哲學理解**，不只是背公式。

7. **Informed RRT\* 橢球收斂 + BIT\* Lazy Collision** — 帶出：「找到第一解後只在 $\lVert x - start \rVert + \lVert x - goal \rVert \le c_{\text{best}}$ 橢球內採樣，範圍瞬間收縮。BIT\* 結合 A* graph search + lazy collision，7 軸機械臂狹窄空間比 RRT\* 縮短 60%+。」**為什麼這是重點**：面試 manipulation 規劃必提，能講出「調過 BIT\*」的 `samples_per_batch` 參數就贏了「只背 RRT\*」。

8. **Costmap 膨脹不匹配是 Local 卡死元兇** — 帶出：「Global Costmap 膨脹半徑小 → 規劃緊貼牆壁捷徑；Local 考量真實輪廓 + 非完整約束 → 鑽不過去 → oscillation。解法：Global 加 Cost Decay 梯度迫使走中央。」**為什麼這是重點**：分辨「讀過教科書」vs「實際除錯過」的決定性題目。

9. **Non-holonomic 阿克曼約束 + Hybrid A\*** — 帶出：「阿克曼車 $\dot{x}\sin\theta - \dot{y}\cos\theta = 0$，不能原地平移。Hybrid A* 節點擴展用運動學積分 + Reeds-Shepp heuristic，**三點倒車入庫從 cost 湧現**而不是寫死規則。」**為什麼這是重點**：自駕規劃核心差異，分辨「會 A\*」vs「懂車輛動力學」。

10. **DWA 短視陷阱 + TEB 時空預測 + MPPI 非光滑友善** — 帶出：「DWA 只前瞻 1-2 秒，U 型卡死；TEB 用 G2O 非線性優化，時間 $\Delta T$ 當決策變數，對動態行人魯棒；MPPI GPU 並行萬條軌跡，不需求導，越野階躍 cost 殺手級。」**為什麼這是重點**：Local Planner 本質差異是 Nav2 面試最常問題，場景選型要說得出理由。

11. **CBS 衝突樹 + Priority Planning 降維 + ORCA 分散式** — 帶出：「CBS 理論最優但 100 agent 指數爆炸；產業用 Priority Planning 解耦計算 + ORCA 底層各承擔 50% 避讓 + Traffic rules 減少衝突。Amazon Kiva 就是這套。」**為什麼這是重點**：NP-hard 問題從理論到落地的標準答案，展現**產業務實思維**。

12. **ESDF 梯度引導 3D 軌跡優化 + Learning as proposal** — 帶出：「Fast-Planner 用 A* 找粗糙路徑 → B-spline 參數化 → ESDF 梯度推遠障礙 + 動力學懲罰。EGO-Planner 拋棄全局 ESDF 只用局部法向量速度 ×10。**Learning 是 proposal 不是取代** — Diffuser 生成 → MPC 硬約束校驗是 SOTA 架構。」**為什麼這是重點**：無人機規劃硬核詞彙 + 對 AI 落地的成熟回答，同時展現技術深度和產業思考。

## 延伸閱讀

- **LaValle《Planning Algorithms》Ch5-Ch14**（sampling-based + kinodynamic + decision-theoretic）— RRT / PRM 原作者寫的聖經，免費線上版：<http://lavalle.pl/planning/>
- **Dolgov et al. "Path Planning for Autonomous Vehicles in Unknown Semi-structured Environments" (2010)** — Hybrid A* 原始論文，Stanley DARPA Grand Challenge 倒車入庫的基礎
- **Karaman & Frazzoli "Sampling-based algorithms for optimal motion planning" (2011)** — RRT\* 原始論文，證明漸近最優性，理解 rewiring 理論基礎
- **Gammell et al. "Informed RRT\*" (2014) + "BIT\*" (2015)** — Informed sampling 橢球 + batch graph search，現代 sampling-based 必讀
- **Koenig & Likhachev "D\* Lite" (2002)** — 動態環境增量式搜索的標準答案
- **Zhou et al. "EGO-Planner" (2021) + "Fast-Planner" (2019)** — HKUST 高飛團隊，無人機 B-spline + ESDF 軌跡優化的 SOTA
- **Janson et al. "Diffuser" (Stanford, 2022)** — Diffusion Model 做規劃的開創工作，PyTorch 程式碼開源
- **Sharon et al. "CBS" (2015)** — Multi-agent path finding 理論最優演算法
- **Van den Berg et al. "ORCA" (2011)** — 分散式多機器人避碰，O(1) 極速反應
- **OMPL 官方教程**：<https://ompl.kavrakilab.org/> — MoveIt 底層規劃庫，支援 RRT / RRT\* / PRM / BIT\* / AIT\*
- **Nav2 官方文檔**：<https://navigation.ros.org/> — Layered Costmap + Global/Local Planner + Recovery 行為配置
- **Apollo Lattice Planner 原始碼**：百度自駕公開，Frenet Lattice 五次多項式基元工業實作
- **MoveIt + OMPL 實戰**：<https://moveit.ros.org/documentation/> — 7-DoF 機械臂 sampling-based planning 實作
- **《具身智能算法工程師面試題》Ch1.3 + Ch4.2 + Ch6.4 + Ch8.4** — 中文面試題對應本章：Path Search / Motion Planning / Nav2 / MoveIt 四大主題
