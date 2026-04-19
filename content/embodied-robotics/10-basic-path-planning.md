---
title: "基礎路徑規劃（A*, RRT, APF）"
prerequisites: ["07-forward-kinematics-dh"]
estimated_time: 45
difficulty: 3
tags: ["path-planning", "a-star", "rrt", "apf", "c-space"]
sidebar_position: 10
---

# 基礎路徑規劃（A*, RRT, APF）

## 你將學到

- 能精確區分 path planning、motion planning、trajectory planning 三個詞，面試不混用
- 遇到「從 A 到 B 要怎麼走」的問題，能根據**維度**和**場景**立刻判斷該選 A*、RRT 還是 APF，講出理由
- 理解 Configuration Space（C-space）為什麼是路徑規劃的核心抽象，以及它如何把碰撞檢測簡化成「點在區域內外」的問題

## 核心概念

**精確定義**：**Path planning** 是在已知或部分已知的環境中，找出一條從起點到目標的**無碰撞幾何路線**（純空間，不含時間）。它是機器人自主導航的「戰略導航員」— 負責回答「走哪條路」，而非「怎麼走」或「走多快」。

**三個常混淆的詞**：
- **Path planning**：純幾何路線（waypoints 序列），不含速度、加速度
- **Motion planning**：統稱，泛指從起點到目標的任何運動求解
- **Trajectory planning**：路徑 + 時間參數化（每個 waypoint 何時到達、速度多少）

**Configuration Space（C-space）**：把機器人的所有自由度（關節角、位置、朝向）壓成一個點，障礙物膨脹成 C-space obstacle。這樣碰撞檢測就變成「這個點有沒有落在障礙區域裡」— 幾何問題簡化為集合判斷。2D 移動機器人的 C-space 是 $\mathbb{R}^2$；7-DoF 機械臂的 C-space 是 $\mathbb{R}^7$（或 $T^7$，考慮關節角度環繞）。

**在感知 → 規劃 → 控制閉環的位置**：
- **節點**：**規劃前端**（global planner）
- **輸入**：起點配置 $q_{\text{start}}$、目標配置 $q_{\text{goal}}$、環境地圖（occupancy grid / octomap / point cloud）
- **輸出**：waypoints 序列 $\{q_0, q_1, \dots, q_N\}$（C-space 中的無碰撞路徑）
- **下游**：trajectory optimizer（加時間/速度約束）→ controller（追蹤軌跡）→ actuator

**一句話版本**：「路徑規劃是多維迷宮中尋找安全通道的幾何引擎。」

---

### A* 演算法

**核心公式**：

$$
f(n) = g(n) + h(n)
$$

**物理意義**：$g(n)$ 是從起點到節點 $n$ 的已知代價（走了多遠），$h(n)$ 是從 $n$ 到目標的啟發式估計（還要走多遠）。$f(n)$ 就是「經過這個節點的總估計成本」— A* 每次展開 $f$ 最小的節點。

**關鍵性質**：當 $h(n)$ 是 **admissible**（永不高估真實距離）時，A* 保證找到**全局最優路徑**。當 $h(n) = 0$ 時退化為 Dijkstra（無方向均勻擴展，慢但穩）。

**適用場景**：低維離散空間（2D/3D grid map），如倉庫 AGV、室內導航。

**致命弱點**：C-space 維度增加時，grid 節點數指數爆炸（curse of dimensionality）。7-DoF 機械臂的 C-space 若每軸離散 100 格，就有 $100^7 = 10^{14}$ 個節點 — 完全不可行。

---

### RRT（Rapidly-exploring Random Tree）

**核心流程**（每次迭代）：

1. **Random sample**：在 C-space 隨機撒一個點 $q_{\text{rand}}$
2. **Nearest**：找樹上離 $q_{\text{rand}}$ 最近的節點 $q_{\text{near}}$
3. **Steer**：從 $q_{\text{near}}$ 朝 $q_{\text{rand}}$ 方向延伸固定步長，得 $q_{\text{new}}$
4. **Collision check**：$q_{\text{near}} \to q_{\text{new}}$ 這段路有沒有撞到障礙物
5. 沒撞 → 加入樹；撞了 → 丟掉，重抽

**性質**：**機率完備**（probabilistically complete）— 只要解存在，跑夠久一定找得到；但**不保證最優**（路徑通常彎彎曲曲）。

**RRT\***：在 RRT 基礎上加 **rewiring** — 每次加入新節點後，檢查附近節點能不能透過新節點走更短的路，如果可以就重新接線。這讓路徑**漸近最優**（asymptotically optimal），代價是計算量增加。

**適用場景**：高維 C-space（6-DoF / 7-DoF 機械臂）、複雜障礙物環境。

---

### APF（Artificial Potential Field）

**核心公式**：

$$
F_{\text{total}} = F_{\text{att}} + F_{\text{rep}} = -\nabla U_{\text{att}}(q) - \nabla U_{\text{rep}}(q)
$$

**物理意義**：目標產生「吸引力場」把機器人拉過去，障礙物產生「排斥力場」把機器人推開。機器人沿合力的梯度下降方向走。

- $U_{\text{att}}(q) = \frac{1}{2} k_{\text{att}} \| q - q_{\text{goal}} \|^2$ — 吸引勢能，越遠越大（彈簧拉力）
- $U_{\text{rep}}(q)$：當距離障礙物 < 安全閾值時才有值，越近越大（推力）

**致命弱點**：**局部最小值**（local minima）— 在 U 型障礙物或對稱配置下，吸引力和排斥力抵消，機器人卡住不動。

**適用場景**：常作為**局部規劃器**（local planner）或即時避障層，搭配 A* 等全局規劃器使用。

---

### PRM（Probabilistic Roadmap）

**思路**：離線階段在 C-space 隨機撒大量點，做碰撞檢測後連成路網（roadmap）；線上階段把起點和目標接入路網，用 Dijkstra / A* 查詢最短路。

**適用場景**：**靜態環境 + 多次查詢**（multi-query）。和 RRT 互補 — RRT 是 single-query（每次從頭建樹），PRM 是 multi-query（建一次圖，反覆查）。

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
- **Euclidean distance**：永遠 admissible 但可能不夠 tight
- **Diagonal distance**：8-connected grid，$h = \max(|dx|, |dy|) + (\sqrt{2} - 1) \min(|dx|, |dy|)$

**Weighted A***：$f = g + w \cdot h$，$w > 1$ 時犧牲最優性換速度，解品質在最優解的 $w$ 倍以內（$w$-admissible）。Nav2 的 `NavFn` planner 用的就是 weighted A*。

**JPS（Jump Point Search）**：在 uniform-cost grid 上加速 A*，跳過大量對稱路徑，速度可提升 10x+。只適用於 uniform grid，不適用於 weighted grid。

</details>

<details>
<summary>深入：RRT vs RRT* 的 rewiring 機制與漸近最優性</summary>

**RRT 的問題**：因為隨機採樣的順序影響樹的形狀，RRT 找到的路徑通常遠非最優 — 到處是不必要的彎繞。

**RRT\* 的 rewiring 步驟**：

1. 加入 $q_{\text{new}}$ 後，找半徑 $r$ 內所有已有節點 $Q_{\text{near}}$
2. **選最佳父節點**：對每個 $q \in Q_{\text{near}}$，如果 $g(q) + \text{cost}(q, q_{\text{new}}) < g(q_{\text{new}})$ 且無碰撞，就把 $q$ 設為 $q_{\text{new}}$ 的父節點
3. **重接線**：對每個 $q \in Q_{\text{near}}$，如果 $g(q_{\text{new}}) + \text{cost}(q_{\text{new}}, q) < g(q)$ 且無碰撞，就改讓 $q$ 的父節點變成 $q_{\text{new}}$

搜索半徑的理論值：

$$
r = \gamma \left( \frac{\log n}{n} \right)^{1/d}
$$

$n$ 是目前節點數，$d$ 是 C-space 維度，$\gamma$ 是常數。隨著 $n \to \infty$，路徑收斂到全局最優。

**實務變體**：
- **RRT-Connect**：雙向生長（起點和目標各一棵樹），找到路徑的速度快很多，但不保證最優
- **Informed RRT\***：找到初始解後，用橢球採樣縮小採樣範圍，加速收斂到最優
- **BIT\***（Batch Informed Trees）：結合 RRT* 和 graph-based 搜索的優點

</details>

## 直覺理解

**三個類比**：

1. **A* = GPS 導航**：你開車用 Google Maps，它會計算所有可能路線的「預估總時間」（= $f$），然後推薦最快的。$h$ 就是 Maps 對「剩餘時間」的估計 — 估得越準，搜索越快；但絕不能高估（不然會漏掉真正最快的路）

2. **RRT = 探險家隨機探索**：你在一片未知森林裡找出口。每一步你隨機朝一個方向丟一塊石頭，然後朝那個方向走一段看看有沒有路。走得通就標記，走不通就丟另一塊石頭。最終你會覆蓋整片森林 — 但走出的路線不會是最短的

3. **APF = 磁鐵吸引 + 同極排斥**：目標是一塊大磁鐵在吸你，障礙物是同極磁鐵在推你。你沿著合力方向走。但如果你被夾在兩塊排斥磁鐵中間、剛好對著目標的吸力被抵消 — 你就卡住了（local minimum）

**模擬器觀察**：

- **Gazebo + Nav2**：啟動 `nav2_bringup`，在 Rviz2 用 `2D Goal Pose` 設定目標。觀察 global costmap 上 A*（或 NavFn）展開的節點（`publish_potential: true` 可以看到勢能場），以及 local planner（DWA / TEB）如何在 waypoints 間做即時避障
- **MuJoCo / Isaac Sim**：在 7-DoF 機械臂場景中跑 MoveIt 的 OMPL planner（預設 RRTConnect），觀察 C-space 中的採樣點如何逐漸填滿可行空間。把障礙物加密，看規劃時間如何隨 clutter 增加而上升
- **2D Python 動畫**：用 matplotlib 畫 A* 的節點展開過程和 RRT 的樹生長過程，是建立直覺最快的方式

## 實作連結

**三個典型工程場景**：

1. **ROS 2 Nav2 全局規劃**：移動機器人在 2D occupancy grid 上用 A*（NavFn plugin）算全局路徑，再交給 DWA / TEB local planner 做動態避障。這是分層架構的經典範例 — 全局最優 + 局部即時。

2. **MoveIt 機械臂運動規劃**：7-DoF 機械臂在雜亂桌面抓取物體。MoveIt 底層用 OMPL（Open Motion Planning Library），預設 RRTConnect。C-space 是 7 維，grid-based 方法完全不可行，必須用 sampling-based。

3. **多機器人倉庫調度**：多台 AGV 共享地圖，每台用 A* 算各自路徑，再用 CBS（Conflict-Based Search）或 priority-based 方法解決路徑衝突。這是 multi-agent path finding（MAPF）的入門場景。

**Code 骨架**（Python，A* on 2D grid）：

```python
import heapq

def a_star(grid, start, goal, heuristic):
    """
    grid: 2D numpy array, 0=free, 1=obstacle
    start, goal: (row, col) tuples
    heuristic: callable(node, goal) -> float, must be admissible
    returns: list of (row, col) waypoints, or None
    """
    open_set = [(heuristic(start, goal), 0, start)]  # (f, g, node)
    came_from = {}
    g_score = {start: 0}

    while open_set:
        f, g, current = heapq.heappop(open_set)
        if current == goal:
            return reconstruct_path(came_from, current)
        for neighbor in get_neighbors(grid, current):  # 4-/8-connected
            tentative_g = g + cost(current, neighbor)
            if tentative_g < g_score.get(neighbor, float('inf')):
                g_score[neighbor] = tentative_g
                f_new = tentative_g + heuristic(neighbor, goal)
                came_from[neighbor] = current
                heapq.heappush(open_set, (f_new, tentative_g, neighbor))
    return None  # 無解
```

**Code 骨架**（Python，RRT 基本架構）：

```python
def rrt(start, goal, sample_fn, nearest_fn, steer_fn, collision_free_fn, max_iter=5000):
    """
    sample_fn: () -> q_rand（C-space 隨機取點）
    nearest_fn: (tree, q) -> q_near（找最近節點）
    steer_fn: (q_from, q_to, step) -> q_new（朝目標延伸）
    collision_free_fn: (q1, q2) -> bool（路段是否無碰撞）
    """
    tree = {start: None}  # node -> parent
    for _ in range(max_iter):
        q_rand = sample_fn()
        q_near = nearest_fn(tree, q_rand)
        q_new = steer_fn(q_near, q_rand, step_size=0.1)
        if collision_free_fn(q_near, q_new):
            tree[q_new] = q_near
            if distance(q_new, goal) < threshold:
                tree[goal] = q_new
                return extract_path(tree, goal)
    return None  # 在 max_iter 內沒找到
```

<details>
<summary>深入：完整 Python 實作（A* + RRT + APF 三合一，可直接跑）</summary>

```python
import numpy as np
import heapq
from typing import Optional

# ============ A* on 2D Grid ============

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_grid(grid: np.ndarray, start: tuple, goal: tuple) -> Optional[list]:
    """A* on 2D occupancy grid. grid[r][c]=1 means obstacle."""
    rows, cols = grid.shape
    open_set = [(manhattan(start, goal), 0, start)]
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
                    heapq.heappush(open_set, (ng + manhattan(nb, goal), ng, nb))
    return None

# ============ RRT in 2D continuous space ============

class RRTNode:
    def __init__(self, pos, parent=None):
        self.pos = np.array(pos, dtype=float)
        self.parent = parent

def rrt_2d(start, goal, obstacles, bounds, step=0.3, max_iter=3000, goal_bias=0.1):
    """
    obstacles: list of (cx, cy, radius) circles
    bounds: (xmin, xmax, ymin, ymax)
    """
    def collision_free(p1, p2, n_checks=10):
        for t in np.linspace(0, 1, n_checks):
            pt = p1 + t * (p2 - p1)
            for ox, oy, r in obstacles:
                if np.hypot(pt[0]-ox, pt[1]-oy) < r:
                    return False
        return True

    root = RRTNode(start)
    nodes = [root]

    for _ in range(max_iter):
        if np.random.rand() < goal_bias:
            sample = np.array(goal)
        else:
            sample = np.array([
                np.random.uniform(bounds[0], bounds[1]),
                np.random.uniform(bounds[2], bounds[3]),
            ])
        nearest = min(nodes, key=lambda n: np.linalg.norm(n.pos - sample))
        direction = sample - nearest.pos
        dist = np.linalg.norm(direction)
        if dist < 1e-6:
            continue
        new_pos = nearest.pos + direction / dist * min(step, dist)
        if collision_free(nearest.pos, new_pos):
            new_node = RRTNode(new_pos, parent=nearest)
            nodes.append(new_node)
            if np.linalg.norm(new_pos - np.array(goal)) < step:
                goal_node = RRTNode(goal, parent=new_node)
                path = []
                n = goal_node
                while n is not None:
                    path.append(n.pos.tolist())
                    n = n.parent
                return path[::-1]
    return None

# ============ APF in 2D ============

def apf_2d(start, goal, obstacles, k_att=1.0, k_rep=100.0, d0=1.0,
           step=0.05, max_iter=1000, tol=0.1):
    """
    Simple APF with parabolic attractive + repulsive potential.
    obstacles: list of (cx, cy, radius)
    """
    pos = np.array(start, dtype=float)
    path = [pos.copy()]

    for _ in range(max_iter):
        if np.linalg.norm(pos - np.array(goal)) < tol:
            return [p.tolist() for p in path]

        # Attractive force
        f_att = -k_att * (pos - np.array(goal))

        # Repulsive force
        f_rep = np.zeros(2)
        for ox, oy, r in obstacles:
            obs = np.array([ox, oy])
            diff = pos - obs
            dist = np.linalg.norm(diff) - r
            dist = max(dist, 0.01)  # avoid division by zero
            if dist < d0:
                f_rep += k_rep * (1.0/dist - 1.0/d0) * (1.0/dist**2) * (diff / np.linalg.norm(diff))

        f_total = f_att + f_rep
        pos = pos + step * f_total / (np.linalg.norm(f_total) + 1e-6)
        path.append(pos.copy())

    return [p.tolist() for p in path]  # may not have reached goal (local min!)

# ============ Demo ============
if __name__ == "__main__":
    grid = np.zeros((20, 20), dtype=int)
    grid[5:15, 10] = 1  # vertical wall
    path_astar = a_star_grid(grid, (2, 2), (18, 18))
    print(f"A* path length: {len(path_astar) if path_astar else 'No solution'}")

    obstacles = [(5, 5, 1.0), (3, 8, 0.8)]
    path_rrt = rrt_2d([0, 0], [10, 10], obstacles, bounds=(0, 12, 0, 12))
    print(f"RRT path length: {len(path_rrt) if path_rrt else 'No solution'}")

    path_apf = apf_2d([0, 0], [10, 10], obstacles)
    print(f"APF path length: {len(path_apf)}")
```

</details>

## 常見誤解

1. **「RRT 找的就是最短路徑」** — 錯。RRT 只保證**機率完備**（能找到一條路），但路徑通常彎彎曲曲、遠非最優。要漸近最優得用 **RRT\***（加 rewiring）。面試時被問「RRT 和 RRT\* 差在哪」，答案就是 rewiring 帶來的漸近最優性。

2. **「APF 可以當全局規劃器用」** — 危險。APF 有**局部最小值**問題，在 U 型障礙物、死角、對稱配置下會卡住。業界做法是 APF 只當**局部避障層**，全局路徑由 A* 或 RRT 負責。Nav2 的分層架構（global planner + local planner）就是這個思路。

3. **「A* 萬能，什麼場景都能用」** — 在高維 C-space 完全不可行。7-DoF 機械臂的 C-space 是 7 維，grid 離散化後節點數是指數級。這就是為什麼 MoveIt 用 sampling-based 方法（RRT / PRM），而不是 A*。**維度決定演算法選擇**，這是路徑規劃最重要的一條判斷原則。

4. **「Path planning 和 trajectory planning 是一回事」** — 不是。Path 只有幾何（走哪裡），trajectory 加了時間（什麼時候到、速度多少）。規劃 pipeline 是 path → trajectory → control，三層分開。

## 練習題

<details>
<summary>Q1：倉庫裡一台 AGV 要從貨架 A 走到貨架 B，環境是靜態的 2D 地圖，你會用什麼演算法？為什麼？</summary>

**完整推理鏈**：

1. **判斷維度**：AGV 在 2D 平面移動，C-space 是 $(x, y)$ 或 $(x, y, \theta)$，最多 3 維 → grid-based 方法可行
2. **判斷場景**：靜態環境、單次查詢、需要最短路 → **A\*** 是首選
3. **啟發函數選擇**：如果是 4-connected grid 用 Manhattan distance；8-connected 用 diagonal distance；都是 admissible 的
4. **膨脹半徑**：要把障礙物按 AGV 的物理尺寸做 inflation（膨脹），這樣 AGV 就可以當成一個點來規劃
5. **要避開的陷阱**：如果地圖很大（如 1000x1000），vanilla A* 會展開太多節點。可以用 **JPS**（Jump Point Search）加速，或 **weighted A\*** 犧牲一點最優性換速度

**結論**：A*（低維 + 最優保證 + 靜態環境），配合 inflation layer 和適當的啟發函數。如果地圖特別大，考慮 JPS 或 hierarchical A*。

</details>

<details>
<summary>Q2：7-DoF 機械臂要在雜亂桌面上抓取一個杯子，桌上有很多障礙物，你會用什麼規劃方法？</summary>

**完整推理鏈**：

1. **判斷維度**：7-DoF → C-space 是 7 維 → grid-based 方法指數爆炸（$100^7 = 10^{14}$），**必須用 sampling-based**
2. **單次 vs 多次查詢**：抓取任務通常是 single-query（每次目標不同）→ **RRT 系列**優於 PRM
3. **最優性需求**：機械臂通常不需要「全局最短路徑」，而是「夠好的無碰撞路徑」→ **RRTConnect**（雙向 RRT，速度快）是 MoveIt 的預設選擇
4. **如果需要更優路徑**：用 RRT\* 或 Informed RRT\*，但要允許更長的規劃時間
5. **碰撞檢測**：這是瓶頸 — 每次 steer 都要做碰撞檢測。MoveIt 用 FCL（Flexible Collision Library），可以用 OctoMap 表示環境
6. **要避開的陷阱**：規劃出的路徑通常需要 post-processing（shortcutting + smoothing），否則軌跡會有不必要的彎繞

**結論**：RRTConnect（MoveIt/OMPL 預設），配合 FCL 碰撞檢測和路徑 shortcutting。如果場景固定且需要多次查詢，考慮 PRM。

</details>

<details>
<summary>Q3：你用 Nav2 做全局規劃，發現 A* 算出的路徑繞了很大一圈避開障礙物，明明有更近的路可以走，怎麼診斷？</summary>

**完整推理鏈**：

1. **先看 costmap**：在 Rviz 打開 global costmap 的 inflation layer visualization。很可能是 `inflation_radius` 設太大，把本來可以走的窄通道堵死了
2. **調 inflation_radius**：降低到略大於機器人物理半徑（加安全餘量），確保窄通道不被過度膨脹
3. **檢查 cost_scaling_factor**：這個參數控制 inflation 衰減的速度。太小會讓遠離障礙物的地方也有很高的 cost，A* 自然會繞路
4. **檢查啟發函數權重**：如果用 weighted A*（NavFn），$w$ 太大會讓搜索太「貪心」，可能漏掉更好的路徑；$w$ 太小又搜太慢。典型值 $w \in [1.0, 2.0]$
5. **要避開的陷阱**：不要只調一個參數 — costmap 和 planner 參數要一起看。另外確認 costmap 的 obstacle layer 有正確訂閱感測器 topic，不然可能有幽靈障礙物

**結論**：先視覺化 costmap 的 inflation layer，降 `inflation_radius` + 調 `cost_scaling_factor`；再檢查 planner 的啟發函數權重。90% 的「繞路問題」都是 costmap 膨脹太保守。

</details>

<details>
<summary>Q4：你要設計一個分層規劃系統（全局 + 局部），怎麼架構？各層用什麼演算法？</summary>

**完整推理鏈**：

1. **全局規劃層（Global Planner）**：
   - 輸入：靜態/半靜態地圖 + 起點 + 目標
   - 演算法：A*（2D 移動機器人）或 RRT*（高維機械臂）
   - 輸出：粗略 waypoints 序列
   - 觸發頻率：目標改變時、地圖大幅更新時（不需要每個 loop 跑）

2. **局部規劃層（Local Planner）**：
   - 輸入：當前位姿 + 全局 waypoints + 即時感測器（LiDAR / depth camera）
   - 演算法：DWA（Dynamic Window Approach）、TEB（Timed Elastic Band）、或 APF
   - 輸出：即時速度指令 $(v, \omega)$
   - 頻率：10-20 Hz，必須即時

3. **Recovery 層**：
   - 當局部規劃器失敗（卡住 / 震盪）時觸發
   - 策略：原地旋轉（clear costmap rotation）、後退、重新呼叫全局規劃

4. **要避開的陷阱**：全局和局部的 costmap 不同 — 全局用靜態地圖，局部用即時感測器。如果局部 costmap 沒有正確更新，機器人會撞到全局地圖沒有的動態障礙物

**結論**：Nav2 的架構就是這個標準分層模式 — `planner_server`（全局 A*）+ `controller_server`（局部 DWA/TEB）+ `recovery_server`。理解這三層的職責分工是面試必考。

</details>

## 面試角度

1. **維度決定演算法** — 這是路徑規劃最核心的判斷原則。面試時帶出：「我選擇規劃演算法的第一個問題永遠是 C-space 幾維。2D/3D 用 A*，6-DoF 以上必須用 sampling-based（RRT / PRM），因為 grid 離散化在高維會指數爆炸。」

2. **啟發函數是 A* 的靈魂** — 區分「只會調 API」和「真正理解演算法」的分水嶺。面試時帶出：「A* 的效能取決於 $h(n)$ 的品質 — admissible 保證最優，consistent 保證不重複展開。工程上常用 weighted A* 犧牲最優性換速度，$w$ 的選擇是 trade-off。」

3. **分層規劃是業界標準架構** — 展現你不只會單一演算法，而是理解系統架構。面試時帶出：「實務上不會只用一種規劃器。Nav2 用 A* 做全局 + DWA/TEB 做局部，全局保最優、局部保即時。機械臂用 RRT 做 motion planning + trajectory optimization 做平滑。」

4. **RRT 機率完備 vs RRT\* 漸近最優** — 被問「RRT 和 RRT\* 差在哪」時的標準答法。面試時帶出：「RRT 只保證找得到路，RRT\* 加了 rewiring 機制讓路徑漸近最優。但 rewiring 增加計算量，所以 MoveIt 預設用 RRTConnect（速度快）而非 RRT\*，因為機械臂通常 good-enough 就好。」

5. **C-space 是規劃的核心抽象** — 展現你理解規劃問題的數學本質。面試時帶出：「把機器人縮成 C-space 中的一個點，碰撞檢測變成點是否在 C-obstacle 內。這個抽象讓所有規劃演算法可以統一處理 — 無論是 2D 移動機器人還是 7-DoF 機械臂。」

## 延伸閱讀

- **LaValle,《Planning Algorithms》Ch5（Sampling-based）** — RRT / PRM 的原作者寫的教材，免費線上版，sampling-based planning 的聖經
- **《具身智能算法工程師 面試題》Ch1.3 路徑搜索 + Ch4.2 運動規劃** — 中文面試題直接對應本章內容
- **《具身智能算法工程師 面試題》Ch6.4 Nav2 + Ch8.4 MoveIt** — 工程實作層面的面試考點
- **OMPL（Open Motion Planning Library）官方教程** — MoveIt 底層的規劃庫，支援 RRT / RRT\* / PRM / BIT\* 等，是學 sampling-based planning 最好的實作參考
- **Nav2 官方文檔的 planner / controller 章節** — 理解分層規劃架構的實務配置（inflation_radius、cost_scaling_factor、planner plugin 選擇）
- **論文 "Sampling-based algorithms for optimal motion planning" (Karaman & Frazzoli, 2011)** — RRT\* 的原始論文，證明漸近最優性，理解 rewiring 的理論基礎
- **D\* Lite 演算法** — 增量式搜索，適合地圖動態更新的場景（機器人邊走邊發現新障礙物），Nav2 的 `SmacPlannerHybrid` 就用了類似思路
