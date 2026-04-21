---
title: "動態環境避障與即時重規劃"
prerequisites: ["10-basic-path-planning", "11-trajectory-optimization"]
estimated_time: 60
difficulty: 4
tags: ["obstacle-avoidance", "dwa", "teb", "mppi", "cbf", "orca", "social-nav", "reactive"]
sidebar_position: 12
---

# 動態環境避障與即時重規劃

## 你將學到

- 能用兩句話講清楚「分層規劃架構」：Global planner（低頻、全局最優）+ Local planner（高頻、即時閃避）+ 反應式安全層（VO / CBF，毫秒級守門員）如何協作，面試時不含糊
- 遇到「機器人在人群中急停繞路卻還是撞上」這類情境，知道問題出在缺乏動態預測，會想到 Kalman / IMM / Social-LSTM 預測行人軌跡並寫入時空 Costmap，再用 **Chance-Constrained MPC 以 3σ 擴張** 硬約束涵蓋不確定性
- 能判斷何時用 DWA（差速底盤原型）、TEB（阿克曼非完整約束）、MPPI（非線性高維系統 GPU 並行）、ORCA（多智能體分散式）、CBF-QP（RL 脊髓反射安全濾網）、FASTER / Flight Corridor（3D 無人機）、J^T·F_rep + 零空間（協作機械臂 HRI 認證）
- 看到「服務型機器人被人群卡死（Freezing Robot Problem）」會立刻想到 **Social-Aware 方法**：SFM / CADRL / SARL Self-Attention / Group-Aware；看到「RL 端到端避障」會立刻想到 **Privileged Teacher-Student 蒸餾 + Domain Randomization + CBF 兜底**
- 學完能以這套情境推理鏈在面試或與 AI 協作時即時選型：**「看到 X 情況 → 想到 Y 工具 → 因為原則 Z → 要避開陷阱 W」**

## 核心概念

### 分層規劃架構（三層防禦）

**精確定義**：動態避障系統通常分成三層 — **Global planner**（低頻 ~1 Hz，全域 A\* / Dijkstra 保可達）+ **Local planner**（高頻 20–50 Hz，即時閃避加動力學約束）+ **反應式安全層**（毫秒級 VO / CBF，規劃層出 bug 時兜底）。三層各司其職，是算力預算下唯一可行的工業解。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：感知層（LiDAR / depth camera / pedestrian tracker）即時障礙物位置 + 速度估計、全域地圖（static map + costmap layers）、動態障礙物軌跡預測（Kalman / IMM / Social-LSTM 輸出的時空管）、當前機器人狀態 $(x, y, \theta, v, \omega)$
- **輸出**：可直接下發給底層控制器的速度指令 $(v, \omega)$ 或短期軌跡 waypoints
- **下游**：底層運動控制器（PID / 純追蹤 / MPC tracking）、安全監控層（emergency stop / SSM ISO 15066）、多機協調模組（ORCA / 優先級仲裁）
- **閉環節點**：橫跨 **規劃** 與 **控制** — Local planner 每個 tick 讀最新感知、重新規劃，形成「感知 → 局部規劃 → 速度指令 → 執行 → 感知」高頻閉環；反應式安全層則是閉環尾端的「投影到安全空間」硬兜底

**一句話版本**：「動態避障是機器人的小腦反射神經 — 全局路徑是 GPS 導航，局部規劃是方向盤上的即時操作，反應式安全層是 ABS 急煞。」

### 九大家族地圖（A 級核心）

A 級避障知識地圖由 9 個子家族組成，涵蓋傳統 / 多智能體 / 預測式 / 安全關鍵 / 社會感知 / 3D 空中 / 純視覺 / 端到端 RL / 機械臂 HRI 全景：

| 家族 | 核心武器 | 場景判準 |
|------|---------|---------|
| F1. 傳統局部規劃 | DWA / APF / VFH | 室內 AGV 平坦 2D + 低速靜態 |
| F2. 多智能體分散式 | VO / RVO / ORCA | 100+ 台倉庫 AGV / 多無人機編隊 |
| F3. 動態預測 + MPC | IMM / Social-LSTM / Chance-Constrained MPC | L4 自駕 / 人群中高速行駛 |
| F4. 安全關鍵控制 | CBF / HOCBF / CBF-QP | RL 脊髓反射層 / AEB |
| F5. 社會感知 | SFM / CADRL / SARL / Group-Aware | 醫院商場服務型機器人 |
| F6. 3D 空中避障 | FASTER / Flight Corridor / Event Camera | 無人機高速穿梭 / 災難搜救 |
| F7. 純視覺避障 | MiDaS / Optical Flow / TTC / SNN | Tesla FSD / Skydio 純視覺 |
| F8. RL 端到端 | Privileged Teacher-Student / Diffusion Policy | 四足機器人野外探索 |
| F9. 機械臂 HRI | SSM / J^T·F_rep / 零空間 | Cobot ISO 10218 / 15066 PLd 認證 |

### 最少夠用的數學（保留公式、附物理意義）

**① DWA 代價函數**（速度空間多目標加權，是 F1 家族的基石）：

$$
G(v, \omega) = \sigma \big[\alpha \cdot \text{heading}(v,\omega) + \beta \cdot \text{dist}(v,\omega) + \gamma \cdot \text{velocity}(v,\omega)\big]
$$

**物理意義**：$\text{heading}$ 衡量朝目標對齊度、$\text{dist}$ 衡量離最近障礙物的安全距離、$\text{velocity}$ 鼓勵高速前進。$\alpha, \beta, \gamma$ 可調權重。關鍵是**動態窗口** $V_d = \{(v,\omega) \mid v \in [v_c - a_{\max}\Delta t,\; v_c + a_{\max}\Delta t]\}$ 天然**只在物理絕對可達範圍採樣** → 自動處理加速度極限，不會產生不可執行的指令。

**② APF 吸引斥力勢場**（F1 家族另一經典，但有 local minima 致命傷）：

$$
U_{\text{att}} = \tfrac{1}{2} k \|q - q_{\text{goal}}\|^2, \qquad
U_{\text{rep}} = \tfrac{1}{2} \eta \left(\tfrac{1}{\rho} - \tfrac{1}{\rho_0}\right)^2 \;\; (\rho < \rho_0)
$$

**物理意義**：吸引勢場拉向目標（拋物線，遠處力愈大）；排斥勢場只在與障礙距離 $\rho < \rho_0$ 時生效，靠近時指數飆升。**災難場景**：任何 $-\nabla U_{\text{att}} + \sum -\nabla U_{\text{rep},i} = 0$（或局部 potential 極小）的點都會卡死 — 兩對稱障礙中央只是特例，**U 型走廊／窄通道／目標藏在凹形牆後**同樣觸發。面試經典陷阱。

**③ Velocity Obstacle / ORCA 半平面**（F2 家族的黃金標準）：

$$
VO_{A|B} = \{\mathbf{v}_A \mid \exists\, t > 0 : (\mathbf{v}_A - \mathbf{v}_B) \cdot t \in B \oplus (-A)\}
$$

**物理意義**：把障礙物 $B$ 用 Minkowski 和膨脹成含機器人形狀的區域，從原點射出的相對速度錐就是「會撞」的速度集合。ORCA 把碰撞錐線性化為**半平面約束** $(\mathbf{v}_A - (\mathbf{v}_A^{\text{opt}} + \mathbf{u}/2)) \cdot \mathbf{n} \geq 0$，其中 $\mathbf{u}$ 是把相對速度推出 VO 邊界的最小向量、$\mathbf{n}$ 取作 $\mathbf{u}$ 的單位向量（**指向 VO 外側**，半平面因此指向「遠離碰撞」一側）。所有約束線性 → **LP 求解 $O(n)$**，毫秒級算出上千台 AGV 無碰速度。

**④ Chance-Constrained MPC 3σ 時空管**（F3 家族的現代答案）：

$$
\|\mathbf{p}_{\text{ego}}(k) - \boldsymbol{\mu}_{\text{obs}}(k)\|^2 \geq (d_{\text{safe}} + 3\sigma_{\text{obs}}(k))^2, \quad \forall k \in [1, H]
$$

**物理意義**：Social-LSTM 預測每個未來時間步的行人 2D 高斯分佈 $\mathcal{N}(\boldsymbol{\mu}_k, \Sigma_k)$；MPC 把避障約束**向外擴張 3σ 涵蓋 99.7% 機率**。面對行為莫測的行人自動拉開更大橫向距離。**「預測不確定性完美融入控制硬約束」** 是 L4 自駕工業答題的關鍵句。

**⑤ CBF Forward Invariant Set**（F4 家族 2020 後統一安全控制的骨架）：

$$
\dot{h}(x, u) + \alpha(h(x)) \geq 0, \qquad h(x) \geq 0 \iff x \in \mathcal{C}
$$

**物理意義**：$h(x) \geq 0$ 定義安全集合 $\mathcal{C}$（離障礙物距離）。$-\dot{h}$ 是**實際**靠近危險邊界的速率；不等式 $\dot h + \alpha(h) \geq 0$ 等價於 $-\dot h \leq \alpha(h)$，也就是把「實際靠近速率」上限卡在 $\alpha(h(x))$（**允許的最大靠近速率**，隨 $h$ 線性減小：越靠近邊界、這個上限越小）。**保證**：當 $h \to 0$，允許的最大靠近速率 $\alpha(h) \to 0$，向危險方向速度**幾何級數衰減至零** → 永不越界。**CBF 是安全濾網不主動產力**，沒有 APF 的 local minima 問題。

**⑥ CBF-QP 最小投影**（F4 家族的實作形式，給 RL 加硬安全保證的標準架構）：

$$
\min_{u} \; \tfrac{1}{2} \|u - u_{\text{nom}}\|^2 \quad \text{s.t.}\quad L_f h(x) + L_g h(x) \cdot u + \alpha h(x) \geq 0, \quad u_{\min} \leq u \leq u_{\max}
$$

**物理意義**：RL 大腦輸出 $u_{\text{nom}}$；CBF-QP 是**脊髓反射層**，只在 $u_{\text{nom}}$ 想越界時以最小代價投影到安全空間。凸優化微秒求解 → RL 泛化 + 數學物理安全底線。

**⑦ MPPI 軌跡權重**（F1 / F3 混合，GPU 並行的資訊理論 MPC）：

$$
w^{(k)} = \exp\!\left(-\tfrac{1}{\lambda} S(\tau^{(k)})\right), \qquad u^* = \frac{\sum_k w^{(k)} u^{(k)}}{\sum_k w^{(k)}}
$$

**物理意義**：對 $K$ 條隨機擾動軌跡各自計算總代價 $S$，用 Boltzmann 權重 softmax 聚合。$\lambda$ 是溫度 — 越小越貪心（只信最好的幾條），越大越保守（平均所有）。GPU 上 $K = 2048\sim 8192$ 並行，每條獨立 rollout 無資料依賴。

**⑧ 機械臂 Cartesian 勢場 → 雅可比轉置映射**（F9 家族 Cobot 認證核心）：

$$
\boldsymbol{\tau}_{\text{avoid}} = \sum_i \mathbf{J}_i(q)^\top \cdot \mathbf{F}_{\text{rep},i}, \qquad
\boldsymbol{\tau}_{\text{null}} = (\mathbf{I} - \mathbf{J}^+ \mathbf{J}) \cdot \boldsymbol{\tau}_{\text{avoid}}
$$

**物理意義**：各連桿 Capsule 逼近障礙時算 3D 虛擬斥力 $\mathbf{F}_{\text{rep}}$；雅可比轉置把笛卡爾空間力映射為各關節避讓力矩。**7-DoF 冗餘機械臂核心**：投影到 $\mathbf{J}$ 零空間 → 手肘柔順退讓、末端夾爪拿的水杯紋絲不動 → 主任務與避障解耦。這就是 ISO 10218 / 15066 PLd 認證的數學骨架。

**⑨ Optical Flow TTC 昆蟲式避障**（F7 家族，純視覺的優雅）：

$$
\tau = \frac{Z}{V_z} = \frac{d}{\dot{d}}
$$

**物理意義**：無需絕對距離 $Z$ 和真實速度 $V_z$，**像素面積除以擴張速度即可算到撞時間**。逼近障礙時像素面積幾何級數放大，昆蟲 / 無人機靠這個本能反射避障，頻率可到 event camera 的亞毫秒級。

### Costmap 架構（F1–F3 家族的共同輸入層）

Costmap 是連接感知與規劃的關鍵資料結構：

| 層級 | 功能 | 更新頻率 |
|------|------|---------|
| **Static Layer** | 從 SLAM 地圖載入的靜態佔據格 | 一次性 |
| **Obstacle Layer** | LiDAR / 深度相機即時偵測的障礙物 | 10–30 Hz |
| **Inflation Layer** | 從障礙物邊緣向外膨脹安全梯度 | 跟隨 Obstacle Layer |
| **Voxel Layer** | 3D 體素層（處理懸空障礙、桌下空間） | 10–30 Hz |
| **Spatiotemporal Layer**（A 級） | 行人軌跡預測（未來 $k$ 步的 3σ 擴張） | 跟隨 prediction module |

**物理意義**：Inflation Layer 的代價值從障礙物邊緣呈指數衰減 — 機器人中心碰到高代價區域就表示外殼快擦到了。`inflation_radius` 設定膨脹半徑，`cost_scaling_factor` 控制衰減陡度。**Spatiotemporal Layer** 是 A 級升級關鍵：把未來 $k$ 步的行人預測位置 + 3σ 不確定性寫入不同時間切片，MPC 直接讀多時間片做時空規劃。


<details>
<summary>深入：DWA 完整演算法流程與速度空間窗口推導（F1 家族骨架）</summary>

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

**速度空間離散化**：典型 DWA 用 $30 \times 30$ 到 $50 \times 50$ 的網格。Nav2 的 DWA 實作用迭代式前向模擬，每條軌跡 20–30 個時間步。

**DWA 致命缺陷 — U 型障礙死胡同**：
- DWA 極短視（Myopic），只前瞻 1–2 秒
- U 型走廊 / 死胡同：**無論怎麼走離目標都變遠** → 原地打轉卡死
- 必須靠 Global planner 給出繞行大方向 → 分層架構的起因

**調參實戰**：
- 窄走廊：提高 $\beta$（安全距離權重），降低 $\gamma$（容許減速）
- 開闊空間：提高 $\gamma$（追求速度），降低 $\beta$
- 人群環境：加入預測層，把行人未來位置膨脹寫入 Costmap（升級到 F3 家族）

**Nav2 DWB C++ 核心評分**：

```cpp
double score = alpha * headingDiff(traj.back(), goal)
             + beta * (1.0 / costmap->minDist(traj))
             + gamma * (max_v - traj.velocity);
```

</details>

<details>
<summary>深入：APF Local Minima 的物理推理與 VFH 改良（F1 家族陷阱圖）</summary>

**APF Local Minima 災難場景**：

情境：機器人在走廊中央，左右各一根柱子，目標在正前方。
- **吸引力 $\mathbf{F}_{\text{att}} = -k(\mathbf{q} - \mathbf{q}_{\text{goal}})$**：指向前方目標
- **左柱排斥力 $\mathbf{F}_{\text{rep},L}$**：指向右
- **右柱排斥力 $\mathbf{F}_{\text{rep},R}$**：指向左
- 對稱情況下 $\mathbf{F}_{\text{rep},L} + \mathbf{F}_{\text{rep},R} = 0$，**但引力與合力斥力正好反向抵消** → 合力為零 → 機器人死鎖

**經典變體場景**：
- 目標在 U 型牆後面：引力指向牆、斥力指向牆外、兩者正好相反抵消
- 兩個相同大小障礙物中央：對稱抵消

**VFH (Vector Field Histogram) 改良**：
- 感測器點雲 → **極座標直方圖**：每個角度扇區一個密度值
- 設閾值 → 找出「自由山谷 (Free Valleys)」
- 直接朝**最接近目標方向的自由山谷**前進
- **關鍵**：VFH 不算吸引力與斥力，只找幾何自由空間 → 完全繞開 local minima 問題

**面試答題 — 何時用 APF / DWA / VFH**：
- APF：教學用，實際工業少；有 local minima
- DWA：室內 AGV 平坦 2D + 低速（< 2 m/s）+ 靜態貨架 → 極快 + 成本效益 → 主流
- VFH：早期移動機器人標配（Pioneer、MobileRobots）
- **高動態（60 km/h 汽車 / 亂跑行人）**：DWA / APF 都失效（短視 / 非完整約束 / 高速側滑無法建模）→ **必轉向 F3 家族「基於行為預測的 MPC」**

</details>


<details>
<summary>深入：ORCA 半平面線性規劃與 Freezing Robot Problem（F2 家族黃金標準）</summary>

**VO → RVO → ORCA 的演進**：

1. **VO 幾何本質**：A 與 B 假設保持當前速度，A 的「碰撞錐」在相對速度空間畫出一個三角錐。只要 A 的相對速度 $\mathbf{v}_A - \mathbf{v}_B$ 不在錐內，未來永不相撞。
2. **RVO 震盪消除**：純 VO 走廊相遇會「同時往左閃、再同時往右閃」→ **Reciprocal Dance 震盪**。RVO 要求**雙方各承擔 50% 避讓責任**：A 只調整一半，期望 B 也做一半。
3. **ORCA 線性化**：幾何錐 → **嚴格半平面約束**
   $$
   (\mathbf{v}_A - (\mathbf{v}_A^{\text{opt}} + \mathbf{u}/2)) \cdot \mathbf{n} \geq 0
   $$
   其中 $\mathbf{u}$ 是把 $\mathbf{v}_A - \mathbf{v}_B$ 推出碰撞錐外的最小擾動向量，$\mathbf{n}$ 取作 **$\mathbf{u}$ 的單位向量（指向 VO 外側）**，半平面方向才會一致地指向「遠離碰撞」一側；方向取反則不等式符號整個反掉。**所有約束線性 → LP 求解 $O(n)$**，毫秒級算出上千台 AGV / 無人機無碰速度。

**Freezing Robot Problem 災難（面試必答）**：
- 被夾在兩個相反方向行人中間：左行人給「向右」半平面、右行人給「向左」半平面
- 人群密集時**所有半平面交集為空** → LP 無解 → 瞬間煞停卡死
- **修正方案**：
  1. **硬約束轉軟約束**：引入鬆弛變數 $\epsilon_i \geq 0$，允許微小侵入半平面，但在目標函數加巨大懲罰 $\min \sum \lambda_i \epsilon_i$
  2. **結合拓撲圖搜索**：發現這條路 LP 無解 → Global replan 繞到人群後方
  3. **Social-Aware 升級**（F5 家族）：別把行人當移動圓柱，而是建模「人會讓人」的協商行為

**RVO2 Library C++**：

```cpp
Line orca;
orca.point = velocity_ + 0.5f * u;              // 各承擔一半責任
orca.direction = Vector2(-n.y(), n.x());        // 垂直法向量
orcaLines_.push_back(orca);
linearProgram2(orcaLines_, prefVelocity_, newVelocity_);  // LP 求解
```

**NH-ORCA（Non-Holonomic ORCA）**：阿克曼 AGV / 車型機器人不能側滑 → 把非完整約束轉化為額外的速度空間限制，再套 ORCA。Freiburg 的 Siegwart 組 2011 經典。

**ORCA 工業場景**：Amazon Kiva 倉庫（現 Amazon Robotics）、Cainiao 物流、DJI Swarm 無人機編隊

</details>

<details>
<summary>深入：Chance-Constrained MPC + Social-LSTM 時空管（F3 家族 L4 自駕標配）</summary>

**為什麼 MPC 是移動避障最強武器**：
- APF / DWA 只看當下瞬間快照 → 短視
- **MPC 前瞻**：在 $t$ 時刻優化未來 $H$ 步軌跡；把動態障礙**未來 $k$ 步預測位置**寫進未來 $k$ 步約束
- 自動學會 **減速讓行、加速搶道、提前繞行** 的動態博弈

**動態障礙運動模型三代演進**：

| 代次 | 方法 | 特點 |
|------|------|------|
| 第一代 | CV (Constant Velocity) | 勻速直線，最簡但行人轉彎失效 |
| 第二代 | CA + IMM (Interacting Multiple Models) | 同時跑多個模型（直行 / 左轉 / 右轉），Bayes 加權切換 |
| 第三代 | **Social-LSTM / Trajectron++** | RNN / Transformer 把周圍行人歷史 + Social Force 結合 → 多模態概率分佈軌跡 |

**Social-LSTM 為什麼厲害**：
- 行人軌跡高度耦合（人避人）
- 傳統 Kalman 孤立看每個行人 → 預測誤差大
- Social-LSTM 的 Social Pooling 層讓鄰近行人的 hidden state 互動
- 預測未來 3–5 秒**多模態概率分佈軌跡**（有機率走左 / 機率走右）

**Chance-Constrained MPC 把不確定性變硬約束**：
- NN 預測 → 每個未來時間步一個 2D 高斯分佈 $\mathcal{N}(\boldsymbol{\mu}_k, \Sigma_k)$
- **3σ 擴張**：MPC 避障約束向外擴張 $3\sigma$，涵蓋 99.7% 機率
  $$
  \|\mathbf{p}_{\text{ego}}(k) - \boldsymbol{\mu}_{\text{obs}}(k)\|^2 \geq (d_{\text{safe}} + 3\sigma_{\text{obs}}(k))^2
  $$
- 面對行為莫測的行人自動拉開更大橫向距離
- **預測不確定性完美融入控制硬約束** ← 記住這句工業答題

**CasADi Chance-Constrained MPC Python**：

```python
for k in range(horizon):
    opti.subject_to(X[:, k+1] == rk4_step(model, X[:, k], U[:, k], dt))
    dist_sq = (X[0, k+1] - obs_mu_x[k])**2 + (X[1, k+1] - obs_mu_y[k])**2
    safe_margin = robot_radius + obs_radius + 3.0 * obs_sigma[k]
    opti.subject_to(dist_sq >= safe_margin**2)  # 3σ 硬約束
```

**面試「避障工業標配」答題**：
- L4 自駕 / 高端具身避障 = **時空優化問題**，不是幾何問題
- 傳統把移動車當「正在移動的牆」→ 極保守煞停
- **現代「Prediction + MPC」**：
  - 前端 Social-LSTM 預測行人**時空管（Spatio-temporal Tube）**
  - 後端 MPC 保證自車時空軌跡不與行人時空管干涉
- **平台**：Waymo Driver、Tesla FSD、Apollo、Tier IV Autoware

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
      wz_std: 0.4                 # 角速度擾動標準差
      temperature: 0.3            # lambda，越小越激進
      gamma: 0.015                # 控制平滑項權重
      critics:
        - "GoalCritic"
        - "ObstaclesCritic"
        - "PathFollowCritic"
      ObstaclesCritic:
        repulsion_weight: 1.5
        critical_weight: 20.0
        collision_cost: 10000.0
```

**常見實作**：NVIDIA Isaac Lab 內建 MPPI controller、[MPPI-Generic](https://github.com/ACDSLab/MPPI-Generic) C++/CUDA 開源庫、Nav2 MPPI Plugin（C++ / CUDA 雙後端）。

</details>


<details>
<summary>深入：CBF / HOCBF 的 Forward Invariant Set 理論與 CBF-QP RL 脊髓反射層（F4 家族統一安全控制）</summary>

**2020 後 CBF 成統一安全理論的理由**：
- 傳統：目標追蹤用 **CLF（控制李雅普諾夫）** + 避障用 APF / 幾何硬約束，兩套框架
- **Ames 等人的 CBF 把「安全」嚴格轉化為 Forward Invariant Set 數學問題**
- 安全約束與控制指令在同一能量 / 梯度語境下統一 → 凸優化可解

**Forward Invariant Set 定義**：
- 安全集合 $\mathcal{C} = \{x \in \mathbb{R}^n \mid h(x) \geq 0\}$
- 若控制 $u$ 能滿足 $\dot{h}(x, u) + \alpha(h(x)) \geq 0$，則 $h$ 為 CBF
- **物理意義**：$\dot{h}(x, u) = \nabla h \cdot f(x) + \nabla h \cdot g(x) u$ 是你靠近危險邊界的速度；$\alpha(h(x))$ 是允許的最大靠近速度（正比於離邊界距離，通常取 $\alpha(h) = \gamma h$）
- **保證**：「越靠近死線 $h(x) = 0$，向危險方向移動的速度必須幾何級數衰減至零」→ **永不越界**

**HOCBF（Higher-Order CBF）解 Relative Degree > 1**：
- 機器人是二階系統（輸入是力矩 / 加速度，約束在位置 $x$）
- 直接對位置求一階導只得速度，控制輸入不在 $\dot{h}$ 中 → Relative Degree > 1
- HOCBF 多階泰勒展開 + 指數衰減：
  $$
  \psi_1(x) = \dot{h}(x) + \alpha_1(h(x)) \geq 0
  $$
  $$
  \psi_2(x, u) = \dot{\psi}_1(x, u) + \alpha_2(\psi_1(x)) \geq 0
  $$
- 解決**剎車距離不足問題**：你需要在距離牆還有剎車距離時就開始減速，而不是撞上才發現來不及

**CBF vs APF**（沒有 local minima 的優雅性）：
- APF 主動施加斥力 → 引力與斥力抵消時 **local minima**
- **CBF 是安全濾網，不主動產力**：
  - 只監聽 Nominal Controller（PID 或 RL 輸出 $u_{\text{nom}}$）
  - $u_{\text{nom}}$ 安全時完全不干預
  - 只有 $u_{\text{nom}}$ 想越界時 CBF 以最小代價投影到安全空間
- **不改變原能量場拓撲** → 天然無 local minima

**CBF-QP 數學**：

$$
\min_u \; \tfrac{1}{2} \|u - u_{\text{nom}}\|^2
$$

s.t.
$$
L_f h(x) + L_g h(x) \cdot u + \alpha h(x) \geq 0 \quad \text{(safety)}
$$
$$
u_{\min} \leq u \leq u_{\max} \quad \text{(actuator limits)}
$$

其中 $L_f h = \nabla h \cdot f$、$L_g h = \nabla h \cdot g$ 是李導數。

**「給 RL 加硬安全保證」的工業答題**：
- RL 黑箱 + 探索性 → 物理世界極易出現危險動作
- 不能指望 RL 自己學會 100% 安全（樣本複雜度爆炸）
- 標準解：**「RL 大腦給 $u_{\text{nom}}$，CBF-QP 脊髓反射層攔截」**
- CBF-QP 凸優化微秒求解 → 最小二乘攔截修正 → RL 泛化 + 數學物理安全底線
- 這就是「端到端 RL demo 酷炫但落地少」的工程妥協答案

**CBF-QP Python 實作**：

```python
import cvxpy as cp
u = cp.Variable(u_nom.shape)
objective = cp.Minimize(0.5 * cp.sum_squares(u - u_nom))
Lf_h = grad_h @ f_x
Lg_h = grad_h @ g_x
cbf_constraint = [Lf_h + Lg_h @ u + alpha * h_val >= 0]
torque_limit = [u >= -MAX_TORQUE, u <= MAX_TORQUE]
cp.Problem(objective, cbf_constraint + torque_limit).solve(solver=cp.OSQP)
```

**平台**：自駕 AEB（自動緊急煞車 Mobileye EyeQ、Bosch）、協作機械臂 Speed and Separation Monitoring、Boston Dynamics Atlas safety layer

</details>

<details>
<summary>深入：Social-Aware 導航 — SFM / CADRL / SARL Self-Attention / Group-Aware（F5 家族醫院商場服務型）</summary>

**Freezing Robot Problem 災難**：
- 把行人當移動圓柱障礙 → 密集人群所有未來 Feasible Velocity Set 被行人時空軌跡佔滿 → 規劃器無解 → 煞停卡死
- **但實際上人會讓人** → 機器人主動擠一下，人群自然分開
- 必須引入「社會感知（Social-Aware）」

**Social Force Model（SFM，Helbing 1995）**：
- 人群為相互作用的粒子系統
- 行人受目標吸引 + 其他行人 / 牆壁排斥
  $$
  \mathbf{F}_i = \mathbf{F}_{\text{goal}} + \sum_j \mathbf{F}_{\text{rep},ij} + \mathbf{F}_{\text{wall}}
  $$
- **缺點**：被動反應式，無法建模「主動博弈協商」（例如禮貌退讓、搶先加速）

**CADRL（Collision Avoidance with Deep RL）**：
- 端到端 RL + **Self-play**（機器人自己跟自己對練）
- 學到「從別人身後繞、主動讓路」等隱式社會規則
- 解決 2–4 個 Agent 導航

**SARL（Socially Attentive RL）— 密集人群核心架構**：
- 密集人群 $N$ 變動 → 傳統 RL 難處理可變維度狀態
- **Self-Attention 機制**：
  - 計算自身與每個行人的注意力權重 $\alpha_i = \text{softmax}(e_i)$，其中 $e_i = \text{MLP}(s_{\text{robot}}, s_{\text{human},i})$
  - 只聚焦「可能碰撞」的關鍵行人（高注意力權重）
  - 變長人群狀態壓縮為定長 Context Vector
- 輸入可變 $N$ 個行人、輸出定長 policy → 完美處理動態人群

**Group-Aware Navigation**：
- 兩三人並排走聊天 → 從中間穿過不禮貌（甚至破壞實驗變量：行人可能被突然嚇到）
- 軌跡相似 + 距離近的行人聚類為 **Group** → 視為不可分割整體 → 從外側繞行
- 方法：DBSCAN 聚類行人軌跡 + F-formation 幾何偵測

**「服務型機器人商業化最後一哩」答題**：
- 醫院商場挑戰不是「找路」，而是「被卡死」或「引發人類反感」
- **SARL + SFM 混合 = 非語言社會協商能力**
- 決定機器人是笨拙還是優雅融入人類社會
- **平台**：UBTECH Walker、Figure 01 醫院導航、Amazon Astro、Toyota HSR

**SARL Attention PyTorch 片段**：

```python
class CrowdAttention(nn.Module):
    def forward(self, robot_state, human_states):
        joint = torch.cat([robot_state.expand(-1, N, -1), human_states], dim=2)
        features = F.relu(self.mlp(joint))          # MLP 編碼
        scores = self.attention_net(features)       # 注意力打分
        weights = F.softmax(scores, dim=1)          # 變長 → 定長
        context = torch.sum(weights * features, dim=1)
        return context  # 固定維度環境 context
```

**進一步**：GP-SARL（結合 Gaussian Process uncertainty）、DS-RNN（Decentralized Structural RNN）、Intention-aware navigation（預測行人意圖再行動）

</details>

<details>
<summary>深入：Aerial 3D 避障 — FASTER / Flight Corridor / Event Camera（F6 家族無人機高速穿梭）</summary>

**無人機 3D 避障三大挑戰**：
1. **SE(3) 飛行 + 嚴重欠驅動**：無剎車，減速必須機身 Pitch 仰轉（4 個旋翼只能產生向上推力 + 3 個力矩，6 DoF 狀態但只有 4 個控制輸入）
2. **深度相機 FOV 有限**：高速轉彎盲區無地圖 → 撞機
3. **高速 10 m/s 穿梭動力學**：感知 → 建圖 → 規劃串行延遲不能超過 50 ms

**FASTER Planner（MIT）— 雙軌跡並行**：
- 核心哲學：**安全備援（Safety Fallback）**
- 每週期同時解兩條：
  - **Primary Trajectory**：激進高速，探索 Free space 邊界
  - **Backup Trajectory**：保證能在安全區域完全煞停 / 懸停
- 主軌跡規劃失敗 → 無縫切到 Backup 煞停 → 絕對安全
- **與傳統單軌跡 MPC 差別**：不需要等下一個 tick 才發現失敗

**Bubble Planner / Flight Corridors**：
- 3D 點雲避障是**非凸** → 直接 NLP 很慢
- **Corridor Planner** 把 Free Space 膨脹成**重疊的凸多面體或球體**（Polyhedral Safe Corridor）
- 非凸避障 → 待在凸多面體內的線性不等式 $\mathbf{A}_i \mathbf{x} \leq \mathbf{b}_i$
- QP 極速求解 B 樣條軌跡
- **B-Spline 凸包性保證所有控制點在多面體內 → 整條連續曲線絕不穿出** ← 這是關鍵數學性質

**Reactive Vision-based Avoidance（事件相機）**：
- 傳統相機高速運動**動態模糊** + 30 Hz 幀率在 20 m/s 致命盲區
- **Event Camera** 只對光強變化反應，**異步事件流 < 1 ms 延遲 + 超高動態範圍（140 dB）**
- 亞毫秒內算撲面障礙的 **Time-to-Contact (TTC)** → 純反應式躲避（Reactive Evasion）

**Perception-aware Planning**：
- 規劃時把**相機 FOV 作約束**
- 橫移 / 側飛時**機頭（相機）始終對準即將前往的未知區域** → 避免盲飛
- 數學：在 MPC 目標函數加 FOV 項 $\|\mathbf{v}_{\text{desired}} - \mathbf{d}_{\text{camera}}\|^2$

**「Event Camera 下一代 UAV 方向」答題**：
- 傳統（FASTER / Bubble）依賴「建圖 → 規劃 → 執行」串行管線
- 極高動態下建圖延遲足以墜毀
- **Event Camera 打破幀率限制**，異步脈衝直接反映環境動態邊緣
- 結合 SNN（脈衝神經網路）或低延遲 Reactive → **像昆蟲一樣光流膨脹本能反射**
- **從「幾何導航」→「仿生本能反應」跨代升級**

**平台**：Skydio X10（純視覺 AI 避障）、DJI Mavic 3（ActiveTrack 5.0）、Zipline 醫療無人機、Shield AI Nova

**Flight Corridor QP C++ 片段**：

```cpp
for (int i = 0; i < num_control_points; ++i) {
    A = getPolyhedronA(i); b = getPolyhedronB(i);
    for (int j = 0; j < A.rows(); ++j) {
        // A_j · P_i ≤ b_j：B 樣條凸包性 → 所有控制點在內 → 整條曲線在內
        solver.addConstraint(A.row(j), control_points[i], b(j));
    }
}
solver.addObjective(smoothness_cost + tracking_cost);
solver.solve();  // QP 幾 ms 求解
```

**論文推薦**：FASTER (Tordesillas 2019)、EGO-Planner (Zhou 2020)、Fast-Planner / Ego-Swarm (Gao 2018-2022)

</details>


<details>
<summary>深入：純視覺避障 — MiDaS / Optical Flow TTC / SNN 仿生（F7 家族）</summary>

**單目深度估計 2024 大爆發**：
- 過去缺尺度資訊 → 單目無法給絕對距離
- **MiDaS / ZoeDepth / Depth Anything** 用 Transformer + 海量混合數據集 → 魯棒相對 / 絕對深度
- 避障常不需精確 3.14 米，只需「比背景近且快速放大」就觸發

**Optical Flow 光流擴張（昆蟲式避障）**：
- 逼近障礙時像素面積**幾何級數急劇放大**
- **TTC（Time-to-Collision）**：$\tau = Z / V_z = d / \dot{d}$
- **物理意義**：無需絕對距離 $Z$ 和真實速度 $V_z$，像素距離除以擴張速度即可算到撞時間
- 蜜蜂、蒼蠅、蜻蜓都靠這個：視覺場擴張率 > 閾值 → 本能轉向

**Learned Depth + MPC（現代 Vision MPC）**：
- NN 實時深度預測 → 3D 占用體素 / SDF → 作 MPC 不等式約束
- 保留視覺豐富語義 + MPC 動力學平滑
- Tesla FSD 的 Occupancy Network 就是這類

**Event Camera + SNN 仿生避障**：
- 傳統 30 Hz 高速動態模糊
- Event Camera 只記錄光強變化的異步事件流，微秒級延遲
- **SNN 脈衝神經網路事件驅動** → 亞毫秒內撲面物體檢測 + 避障
- 低功耗 + 低延遲 → 超小型無人機的唯一選擇

**「單目 + AI 取代 LiDAR」答題**：
- LiDAR 幾何精確但**語義稀疏**（分不出實體石 vs 水霧 / 高草）→ 幽靈煞車
- 單目 + AI 低成本（Tesla FSD、Skydio 純視覺）
- **VLM 理解「海報上的車不是真車」**
- **視覺資訊熵上限遠高於 LiDAR**（RGB 8bit × 3 channel × 1M pixel vs LiDAR 16 bit × 32 line × 1800 point）
- **反方論點**：LiDAR 在黑夜 / 雨霧 / 玻璃還是有優勢；L4 目前仍多感測融合

**Vision MPC 片段（ZoeDepth + CasADi）**：

```python
depth_map = zoedepth_model(rgb_image)              # 推論深度
sdf_tensor = depth_to_sdf(depth_map)                # 轉 SDF
d_safe_k = get_obstacle_clearance(sdf_tensor)       # 查表
opti.subject_to(ca.norm_2(p_k - obs_pred_k) >= d_safe_k + margin)  # MPC 硬約束
```

**平台**：Tesla FSD Pure Vision、Skydio X10、comma.ai openpilot、DJI Avata

</details>

<details>
<summary>深入：RL 端到端避障 — Privileged Teacher-Student / Domain Randomization / Diffusion Policy（F8 家族）</summary>

**Pixel → Action 直接學**：
- 傳統「感知 → 建圖 → 規劃 → 控制」串列，誤差逐級放大
- **端到端 RL**：RGB-D 像素 → CNN / Transformer → Policy → 關節力矩
- 網路自學「看到坑直接抬腿」、「看到人閃邊」

**Sim-to-Real 核心：Domain Randomization**：
- 純視覺 RL 極易過擬合模擬器渲染
- Isaac Sim 訓練時**瘋狂隨機化光照、紋理、相機噪聲、FOV、摩擦、質量、動作延遲**
- 迫使網路放棄特定視覺紋理 → **學真正不變的幾何避障深度特徵**
- Anymal / Spot 四足戶外穩健避障都靠這招

**Visual Policy Distillation（Asymmetric Actor-Critic）**：
- **Teacher (Privileged)**：模擬器上帝視角 + 精確 3D 座標、速度、摩擦 → 快速學完美避障
- **Student (Vision)**：真實只有相機 → Behavior Cloning 擬合 Teacher 動作
- **蒸餾 Loss**：
  $$
  \mathcal{L} = \mathbb{E}\left[\|\pi_{\text{student}}(a \mid o_{\text{vision}}) - \pi_{\text{teacher}}(a \mid s_{\text{priv}})\|^2\right]
  $$
- ETH Zurich 的 Anymal team 經典範式，Hwangbo 2019 開山之作

**Legged Gym / Isaac Lab 大規模並行訓練**：
- 萬環境同時 GPU rollout
- 單 GPU 4096 個機器人並行訓練，幾小時搞定原本幾週的訓練

**Diffusion Policy 2024 前沿**：
- PPO / SAC 高斯分佈取平均 → 「左繞 vs 右繞」多模態取平均**撞障礙物**（災難場景）
- Diffusion 將避障動作視為去噪生成 → **完美保留多模態解**
- Columbia 的 Cheng Chi 等人 2023 論文

**「Demo 酷炫但產業落地少」陷阱（面試必答）**：
- **可解釋性 + 功能安全認證**：End-to-End NN 黑箱 → 撞人後無法 debug 哪層算錯
- **ISO 13849 / 15066 SIL / PLd 認證**要求確定性極限 + 可追溯決策
- **工程妥協**：**RL 給 Proposal → 底層墊確定性 CBF-QP 或 MPC 安全濾網兜底**（F4 CBF 家族的應用）
- 這是「端到端 vs 模組化」辯論的現代工業答案

**平台**：ANYbotics ANYmal（Privileged Teacher-Student）、Boston Dynamics Atlas（部分 RL 模組）、Figure 01、Tesla Optimus、Columbia Diffusion Policy

</details>

<details>
<summary>深入：機械臂 HRI 動態避障 — SSM / J^T·F_rep / 零空間冗餘（F9 家族 Cobot 認證核心）</summary>

**機械臂 vs 移動基座避障本質差異**：
- 移動基座：2D 平面上「質點 / 包圍盒」
- **機械臂是高維鉸接鏈**：
  - **自碰撞（Self-collision）**：手肘不能打到基座
  - **整機身連桿（Whole-body links）** 都不能撞環境
  - **7-DoF C-space 複雜計算**

**SSM（Speed and Separation Monitoring）— ISO/TS 15066 核心**：
- 3D 相機檢測人體靠近（Intel RealSense、Microsoft Kinect Azure、Pilz SafetyEYE）
- **根據人機相對距離實時計算機械臂最大速度限制**
- 越過紅線（例如 < 0.5 m）→ **安全級急停**或**降級為零力矩牽引模式**
- 這是**Category 3 PLd** 安全等級的強制要求

**SSM 速度公式（ISO/TS 15066 簡化）**：

$$
v_{\max} = \max\left(0, \frac{S_p - (S_B + S_R + S_C)}{T}\right)
$$

其中 $S_p$ 是相對距離，$S_B, S_R, S_C$ 是機器人煞車距離、反應距離、人類侵入距離，$T$ 為機器人**反應 + 煞車總時間**。**注意**：這是簡化反解；實務 ISO/TS 15066 以 $S_p(t_0) = S_H + S_R + S_S + C + Z_d + Z_r$ 是否超過門檻為主判據，而非直接算 $v_{\max}$。

**Cartesian 空間勢場避障**：
- 機械臂各連桿包 **Capsules**（圓柱 + 兩端半球，比球包圍盒緊、比凸包簡單）
- 點雲檢測動態障礙逼近連桿 $i$
- 算 3D 虛擬排斥力 $\mathbf{F}_{\text{repulsive}}$
- **雅可比轉置 $\mathbf{J}_i^\top \cdot \mathbf{F}_{\text{rep}}$ 映射為各關節避讓力矩**
- 疊加公式：
  $$
  \boldsymbol{\tau}_{\text{avoid}} = \sum_i \mathbf{J}_i(q)^\top \cdot \mathbf{F}_{\text{rep},i}
  $$

**零空間優化（7-DoF 冗餘機械臂核心）**：
- **扭矩空間投影**：$(\mathbf{I} - \mathbf{J}^\top \mathbf{J}^{+\top}) \cdot \boldsymbol{\tau}_{\text{avoid}}$ 投影到末端不感知的零空間（注意：**速度零空間投影** $(\mathbf{I} - \mathbf{J}^+\mathbf{J})$ 用於 $\dot q$、**扭矩零空間投影** $(\mathbf{I} - \mathbf{J}^\top \mathbf{J}^{+\top})$ 才是 $\tau$ 用的，兩者互為轉置）
- 工人推機械臂手肘 → **手肘柔順退讓，但末端夾爪拿的水杯紋絲不動**
- 主任務（末端位姿）與避障（關節層）完全解耦
- 數學：$\mathbf{J}^{+\top} \mathbf{J}^\top \cdot \tau_{null} = 0$ 意味零空間 torque 不引起末端 wrench → 末端不動

**「HRI 認證核心」答題**：
- Cobots（UR / KUKA iiwa / Franka Panda）客戶買單的是**安全**
- $\mathbf{J}^\top \cdot \mathbf{F}_{\text{rep}}$ 勢場避障 + SSM 速度監控 = **ISO 10218 / ISO/TS 15066 PLd 認證底層數學**
- 把不可預測人類行為轉化為嚴格物理力學約束
- **HRI 場景下不發生夾擠或致命撞擊**的保障

**Franka C++ 片段**：

```cpp
for (int i = 0; i < critical_links.size(); ++i) {
    double dist = compute_min_distance(critical_links[i], obstacles);
    Eigen::Vector3d normal = compute_repulsive_direction(...);
    if (dist < safe_margin) {
        double mag = eta * (1.0/dist - 1.0/safe_margin) * (1.0/(dist*dist));
        Eigen::Vector3d F_rep = mag * normal;
        Eigen::MatrixXd J_i = compute_jacobian_for_link(state.q, i);
        tau_avoid += J_i.transpose().topRows(3) * F_rep;
    }
}
// 零空間投影（torque-space，非 velocity-space）
Eigen::MatrixXd N = I - J.transpose() * J_pinv.transpose();  // torque null-space projector
tau_null = N * tau_avoid;
robot.setJointTorques(tau_task + tau_null);  // 主任務 + 零空間避障疊加
```

**平台**：Franka Panda HRI 模式、KUKA iiwa Cobot、Universal Robots UR series、TM Robot AI Cobot

**進階**：Cartesian Impedance Control + CBF（F4 家族結合）、MoveIt 2 Servo + 即時 SDF、NVIDIA cuRobo GPU 加速

</details>

## 直覺理解

**類比：開車的三層反應**
- **GPS 導航 = Global planner**：告訴你「走高速再轉國道」，頻率很低（重算一次要幾秒）
- **方向盤操作 = Local planner（DWA / TEB / MPPI）**：在車道內即時微調方向，避開前方突然變道的車，20–50 Hz
- **ABS 急煞 = 反應式安全層（VO / ORCA / CBF）**：不經過「規劃」直接在速度空間排除危險指令，< 5 ms 反應

**DWA 的直覺**：想像你站在房間中央，面前扇形展開 900 條可能的走法（不同速度和轉彎率），每條走幾步看看會不會撞到東西、能不能朝門口、速度快不快。挑最好的那條走一步，下一瞬間重新展開 900 條。

**APF Local Minima 的直覺**：你是被磁鐵（目標）吸的鐵球，但路上有兩根反向磁柱。當兩柱子推力剛好對稱，磁鐵吸力又正好與合力反向 → **你懸在空中不動**。

**ORCA 的直覺**：走廊對面來的人。純 VO 等於你跟他同時往左閃 → 又同時往右 → 震盪。ORCA 等於你說「我往右半步，你往左半步，各付一半」→ 平順錯身。但如果三個人夾在你周圍各自丟一個半平面約束，**交集為空集** → 你就只能原地煞停，這就是 Freezing Robot。

**CBF 的直覺**：CBF 不是「強迫你煞車」，而是「規定你越靠近懸崖，煞車越急」。$\alpha(h(x)) = \gamma h$ 的意思是：離懸崖 10 m 時你可以 5 m/s 接近；離 1 m 時只能 0.5 m/s；離 0 時完全不能接近。**幾何級數衰減 → 永遠不掉下去**。

**MPC + Prediction 時空管的直覺**：行人像條蠕動的時空蟲（未來每個 $k$ 秒的機率分佈），你的車也畫出自己的時空管。**兩條時空管不干涉 = 不撞**。3σ 擴張就是給蠕動蟲留緩衝。

**Flight Corridor 的直覺**：無人機在 3D 點雲中飛。直接算「避開每個點」會是超慢的非凸問題。**把 Free Space 切成一串首尾相連的透明泡泡**（凸多面體），無人機只需要「在第 $i$ 個泡泡中找段平滑曲線，出口連到第 $i+1$ 個泡泡入口」→ 整條軌跡由 QP 秒解。

**7-DoF 零空間的直覺**：你拿著一杯滿水（末端 6-DoF 位姿鎖定），手肘有 1 個剩餘自由度可以亂轉。工人推你手肘 → 你手肘讓開、但**水杯位置姿態完全不變一滴不灑**。這 1-DoF 冗餘就是「零空間避障」的全部魔法。

**模擬器觀察**：
- **Gazebo + Nav2**：同一張地圖分別切換 DWA / TEB / MPPI controller，觀察寬走廊 vs 窄道 vs 人群下三者差異
- **Isaac Sim + ORCA**：100 台 AGV 在倉庫交叉路口，觀察 Freezing Robot 發生瞬間（全部同時卡死）
- **MuJoCo + CBF-QP**：7-DoF Franka 末端追蹤圓形軌跡時，隨機在工作空間生成障礙球 → 手肘自然退讓但末端軌跡不變
- **AirSim / Flightmare + FASTER**：無人機穿過 10 m/s 視窗，觀察主軌跡失敗瞬間切到 Backup 懸停

## 實作連結

**六個典型工程場景**（對應 9 家族的選型）：

1. **ROS 2 Nav2 分層規劃（F1）**：`bt_navigator` 用 Behavior Tree 協調 global planner（NavFn / Smac）與 local controller（DWA / TEB / MPPI）。Global path 以 `nav_msgs/Path` 發布，local controller 每個 tick 讀取 `local_costmap` + global path，輸出 `geometry_msgs/Twist` 給底盤。

2. **自駕局部規劃（F3，Autoware / Apollo）**：MPPI 或 Lattice planner 以 30–100 Hz 在 Frenet frame 下規劃，代價函數同時考慮車道中心偏移、障礙物 SDF、Social-LSTM 預測軌跡的 3σ 時空管、舒適性（jerk）、交通規則。輸出軌跡給 PID / Stanley / MPC 追蹤控制器。

3. **多機器人倉庫物流（F2，100+ AGV）**：每台 AGV 各自跑 ORCA 分散式互惠避障，不需中央協調。計算複雜度 $O(n)$（每台只看鄰近 $n$ 台），適合 100+ 台同時運作。優先級仲裁：載貨車 > 空車；交叉路口用拍賣制分配通行權；遇到 Freezing 自動升級到區域中央調度。

4. **RL + CBF 脊髓反射（F4 + F8，四足機器人）**：Policy network 輸出 $u_{\text{nom}}$ → CBF-QP 投影到安全空間 → 關節力矩。典型頻率：Policy 50 Hz、CBF-QP 500–1000 Hz。Boston Dynamics Atlas、ANYbotics ANYmal 都採類似架構。

5. **無人機高速穿梭（F6，FASTER）**：每 50 Hz 同時解 Primary + Backup B-spline。Primary 求解成功則執行，失敗則切 Backup 煞停。Flight Corridor QP 在 5–10 ms 內給出凸多面體軌跡。

6. **協作機械臂 HRI（F9，Cobot 生產線）**：`moveit_servo` 讀 Twist 指令做即時笛卡爾追蹤，`moveit_core` 檢測自碰撞 + 外部障礙，配合 SSM 速度限制。機械臂主任務用 Jacobian 末端控制，避障用 $\mathbf{J}^\top \mathbf{F}_{\text{rep}}$ 加在零空間。

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

**分層避障骨架**（A 級工業模板）：

```python
class HierarchicalAvoidance:
    """三層防禦：Global + Local + Reactive Safety"""

    def __init__(self):
        self.global_planner = AStarPlanner()          # F1 保可達
        self.local_planner = MPPIController()         # F1/F3 高頻閃避
        self.pedestrian_predictor = SocialLSTM()      # F3 動態預測
        self.safety_filter = CBFQP()                  # F4 脊髓反射兜底

    def step(self, obs):
        # L1: Global (1 Hz)
        if self.global_replan_needed(obs):
            self.global_path = self.global_planner.plan(obs.map, obs.goal)

        # L2: Local (30 Hz) + 時空 Costmap
        pred_trajectories = self.pedestrian_predictor.predict(obs.pedestrians, horizon=3.0)
        spatiotemporal_costmap = build_stcmap(obs.costmap, pred_trajectories, sigma=3.0)
        u_nom = self.local_planner.solve(obs.state, self.global_path, spatiotemporal_costmap)

        # L3: Reactive Safety (1 kHz)
        u_safe = self.safety_filter.project(obs.state, u_nom)
        return u_safe
```


## 常見誤解

1. **「Local planner 能取代 Global planner」** — Local planner 只看局部窗口（通常 3–5 m），在 U 型走廊或迷宮中會陷入局部最小值。Global planner 提供「大方向」引導 Local planner 跳出死胡同。DWA / APF 在 U 型障礙中都會原地打轉卡死。**避開**：永遠保持分層架構；如果 local planner 原地打轉超過閾值，trigger global replan；真正的 A 級系統是三層（Global + Local + Reactive Safety）。

2. **「DWA 適用所有底盤」** — DWA 假設圓弧軌跡（恆定 $v, \omega$），對阿克曼底盤（有最小轉彎半徑）的約束建模不佳。強硬套用會產生不可執行的指令。**避開**：阿克曼底盤用 TEB 或 MPPI；多機器人場景用 NH-ORCA（Non-Holonomic ORCA）；自駕 / 人形高動態環境必須升級到 F3 家族的 MPC + Prediction。

3. **「把障礙物膨脹到最大就最安全」** — 過度膨脹會讓可通行空間消失，機器人在窄門前停下不走（CBF 也會觸發同樣問題：$\alpha$ 太保守就凍結）。**避開**：`inflation_radius` 設為機器人半徑 + 安全餘量（通常 0.1–0.3 m），用 `cost_scaling_factor` 讓代價衰減有梯度；CBF 的 $\alpha$ 函數也要平衡激進 vs 保守。

4. **「反應式避障（VO / RVO）不需要預測」** — VO 假設障礙物以恆定速度直線運動，對突然轉向的行人會失效。**避開**：在 VO / ORCA 前端接行人軌跡預測模組（Kalman / IMM / Social-LSTM），把預測軌跡轉成時空碰撞錐；單獨用 ORCA 在密集人群會觸發 Freezing Robot Problem。

5. **「APF 比 CBF 簡單，直接用 APF 就好」** — APF 有致命的 **local minima 問題**（兩對稱障礙中央 / U 型牆後目標 → 引力與斥力抵消）。CBF 不主動產力、只做安全投影 → 天然無 local minima。**避開**：工業場景（AEB、Cobot 安全層、RL 脊髓反射）優先用 CBF-QP；APF 只適合教學或簡單原型。

6. **「把行人當移動圓柱就夠了」** — 這會觸發 Freezing Robot Problem：密集人群所有半平面交集為空 → 煞停卡死。**避開**：引入 **Social-Aware**（SFM / SARL Self-Attention / Group-Aware），建模「人會讓人」的協商行為；物理機器人必須主動「擠一下」才能在人群中移動。

7. **「端到端 RL 避障 demo 漂亮就能落地」** — RL 黑箱無法通過 ISO 13849 / 15066 SIL / PLd 功能安全認證；撞人後無法 debug 哪層算錯。**避開**：工程妥協是 **「RL 給 Proposal + CBF-QP / MPC 硬安全濾網兜底」**，提供確定性的數學安全保證 + RL 的泛化能力。

## 練習題

<details>
<summary>Q1（中）：機器人用 DWA 在人群密集的展覽場地導航，頻繁急停 → 繞路 → 幾乎停滯不前。怎麼分析？用什麼工具？要避開什麼？</summary>

**完整推理鏈**：

1. **診斷根因**：DWA 的障礙物層只反映「此刻位置」，不預測行人未來走向。人一動，DWA 瞬間發現原本安全的速度變危險，急煞 → 重新採樣 → 又急煞 → 震盪。這是典型 F1 家族（反應式）在動態環境的失效。
2. **加入動態預測（升級到 F3 家族）**：
   - 接入行人追蹤模組（如 `spencer_people_tracking` 或自訓 YOLO + DeepSORT）
   - 對每個行人用 Kalman Filter / IMM 或 Social-LSTM 預測未來 2–3 秒軌跡
   - 把預測位置以漸弱的代價值寫入 **時空 Costmap**（未來越遠、代價越低 — 因為不確定性越大）
3. **升級規劃器**：
   - 方案 A：保留 DWA 但把時空 Costmap 當 obstacle layer — 低成本改動
   - 方案 B：換成 TEB（支援動態障礙物 native 輸入）或 MPPI（代價函數直接加預測軌跡碰撞項）
   - 方案 C（A 級）：Chance-Constrained MPC 把 Social-LSTM 的高斯分佈 $\mathcal{N}(\boldsymbol{\mu}_k, \Sigma_k)$ 以 3σ 擴張直接進硬約束
4. **避開的陷阱**：
   - 不要把行人預測窗口拉太長（> 3 s），預測誤差累積會讓 costmap 到處都是高代價區 → 反而升級成 Freezing Robot Problem
   - 不要忘記行人可能突然停下 — 預測模型要有「速度衰減」fallback
   - 展覽場地密集人群：單純預測不夠，還要加 **Social-Aware** 建模「機器人會讓人、人也會讓機器人」

**面試官想聽到**：清楚區分「反應式」vs「預測式」避障的差異；能給出 Costmap 整合預測的具體方案；知道預測窗口過長的副作用；能提到 Social-Aware 作為更進階的解。

</details>

<details>
<summary>Q2（中-難）：MoveIt 規劃的機械臂在動態抓取場景（傳送帶上的物體）太慢，從規劃到執行 > 500 ms，物體早已移走。怎麼解決？</summary>

**完整推理鏈**：

1. **理解瓶頸**：MoveIt 的 RRT / PRM planner 是全域搜尋 C-space，每次規劃需 100–500 ms，且不考慮障礙物的運動
2. **分層解法**：
   - **低頻層（~2 Hz）**：MoveIt 預先規劃一條到「預測抓取點」的參考軌跡（考慮物體速度 $\times$ 到達時間的超前量）
   - **高頻層（100–1000 Hz）**：在關節空間用 QP（Quadratic Programming）或 MPC 做即時修正
     - 讀取視覺追蹤的物體即時位姿
     - 用 SDF（Signed Distance Field）做環境碰撞約束
     - 求解 $\min \|q - q_{\text{ref}}\|^2$ s.t. 碰撞約束 + 關節限位 + 速度限制
3. **SDF 加速**：離線用 `voxblox` 或 GPU 體素化建 ESDF，線上查表 $O(1)$，10 ms 內完成碰撞檢測
4. **加入 HRI 安全層（F9 家族）**：
   - 如果傳送帶旁有工人 → 必須加 SSM（Speed and Separation Monitoring）
   - $\mathbf{J}^\top \mathbf{F}_{\text{rep}}$ 勢場避障 + 零空間投影：手肘可以柔順退讓而末端追蹤不受影響
5. **工具鏈**：
   - `moveit_servo`：讀 Twist 指令做即時笛卡爾空間追蹤
   - `curobo`（NVIDIA）：GPU 加速的 motion planning，規劃時間壓到 < 10 ms
   - `Franka Control Interface`：1 kHz 關節力矩控制

**面試官想聽到**：分層規劃思維（離線粗規劃 + 線上精修正）；SDF 碰撞檢測的效能優勢；知道 `moveit_servo` / `curobo` 這些即時避障工具鏈；能提到 SSM / 零空間作為 HRI 場景的升級。

</details>

<details>
<summary>Q3（難）：100 台 AGV 在倉庫中交叉路口頻繁死鎖。目前用中央調度但延遲高、單點故障。怎麼設計分散式避障方案？</summary>

**完整推理鏈**：

1. **選演算法（F2 家族）**：**ORCA**（Optimal Reciprocal Collision Avoidance）
   - 每台 AGV 各自計算安全速度集合（ORCA 半平面交集），不需中央協調
   - 計算複雜度 $O(n)$（$n$ 為鄰近車輛數），100 台各自只看半徑 5 m 內的鄰居
   - 數學保證：只要所有 agent 都遵守 ORCA，不會碰撞（reciprocal 假設）
2. **優先級仲裁**：
   - 靜態優先級：載貨 AGV > 空車 AGV > 充電中 AGV
   - 動態優先級：距目標越近 → 優先級越高（避免快到終點卻被擠走）
   - 交叉路口：優先級高的 AGV 的 ORCA 半平面對低優先級 AGV 施加更強約束
3. **Freezing Robot Problem 處理**：
   - **硬約束轉軟約束**：引入鬆弛變數 $\epsilon \geq 0$，允許微小侵入半平面加巨大懲罰
   - **混合架構**：如果區域性死鎖（> 3 台互鎖），升級為區域中央調度（fallback）
   - **拓撲圖搜索**：發現這條路不通 → 全局路徑繞到人群後方
4. **死鎖偵測與恢復**：
   - 監控：如果某台 AGV 速度 < 閾值超過 $T$ 秒，標記為「疑似死鎖」
   - 恢復：隨機選一台讓它後退 0.5 m 並重新規劃，打破對稱
5. **避開的陷阱**：
   - ORCA 假設所有 agent 都遵守協議 — 混入人類叉車時必須加 F4 安全層（CBF 硬底線）
   - ORCA 不考慮非完整約束 — 阿克曼 AGV 需用 NH-ORCA 變體
   - 極密集環境（> 10 鄰居）單純 ORCA 失效，必須升級 Social-Aware 或 MPC + Prediction

**面試官想聽到**：分散式 vs 集中式的 trade-off；ORCA 的 reciprocal 保證；實務中的混合架構（分散式為主 + 區域性中央調度 fallback）；知道 Freezing Robot Problem 和解法。

</details>

<details>
<summary>Q4（難）：你的自駕車用 MPPI 做局部規劃，在高速公路上表現良好，但進入城市窄巷後經常擦牆，且遇到行人會緊急煞車而非繞行。不換演算法的前提下怎麼修？</summary>

**完整推理鏈**：

1. **診斷根因（兩個問題疊加）**：
   - **窄巷擦牆**：高速公路的 `vx_std` 和 `wz_std` 設定偏大（探索範圍廣），進入窄巷後隨機軌跡大部分都碰撞被淘汰，剩餘有效軌跡太少
   - **行人急煞**：當前代價函數把行人當靜態障礙，沒用 Social-LSTM 預測 → 每次行人稍微動就觸發高代價
2. **動態調參**：
   - 偵測到進入窄巷（global costmap 的可通行寬度 < 閾值），自動切換參數集：
     - 降低 `vx_std / wz_std`（收窄採樣範圍，集中在可行區域）
     - 增加 `batch_size`（在窄範圍內多採幾條，確保覆蓋度）
     - 降低 `temperature`（更信任少數好軌跡，不被爛軌跡拖累）
3. **代價函數升級到 F3**：
   - 加入 Chance-Constrained 約束：$\|\mathbf{p}_{\text{ego}}(k) - \boldsymbol{\mu}_{\text{ped}}(k)\|^2 \geq (d_{\text{safe}} + 3\sigma_{\text{ped}}(k))^2$
   - 輸入 Social-LSTM 或 Trajectron++ 的行人預測軌跡
   - MPPI 每條軌跡的 $S(\tau)$ 計算時，行人代價隨未來時間衰減（不確定性增大）
4. **備選優化**：
   - Importance sampling（非均勻採樣）：在上一輪最優軌跡附近密集採樣
   - Warm-start：用上一 tick 的最優控制序列作為均值，減少浪費
   - 加入 `PathFollowCritic` 在窄巷嚴格跟隨 global path 中心線
5. **雙重防禦**：最底層再墊一層 F4 CBF-QP，MPPI 輸出後再投影到安全空間

**面試官想聽到**：理解 MPPI 的採樣效率問題（窄空間中有效軌跡比例低）；能根據場景動態調整超參數；知道 Chance-Constrained 升級；能提到 CBF 作為最後安全網。

</details>

<details>
<summary>Q5（難）：服務型機器人在醫院大廳導航，被一群圍著聊天的訪客卡死 10 分鐘。純 ORCA 和純 DWA 都失效。怎麼設計？</summary>

**完整推理鏈**：

1. **診斷根因（F5 家族核心情境）**：
   - ORCA 失效：Freezing Robot Problem（所有半平面交集為空）
   - DWA 失效：短視 + 不建模行人意圖 → 永遠找不到安全速度
   - 根本問題：**把行人當移動圓柱**，忽略「人會讓人、也會讓禮貌的機器人」的社會動力
2. **分三層解**：
   - **L1 Group-Aware 偵測**：DBSCAN + F-formation 檢測「三人面對面站成圓」→ 聚類為不可穿越 Group → 從外側繞行
   - **L2 SARL Self-Attention Policy**：端到端 RL 學會「從人群邊緣繞、對領頭者禮貌讓路」的社會規則；Self-Attention 處理可變人數
   - **L3 SFM 兜底**：如果 SARL 輸出太激進，SFM 的反應式斥力場兜底保證基本安全
3. **通訊加值**：
   - 機器人發出「借過提示音」 → 行人通常會主動讓路
   - VLM 語音模組：「不好意思可以讓我過去嗎」的人類語言請求
4. **Freezing 恢復**：
   - 如果實在無解（超過 30 秒無進展）：機器人主動說「請讓一下」並開始**緩慢接近**（< 0.2 m/s），人群通常會自然讓開
   - **主動擠一下** 是社會感知導航的精髓：不擠就永遠過不去
5. **避開的陷阱**：
   - 不要把行人預測窗口拉太長（人在聊天時幾乎不動，未來 30 秒都在同一位置 → costmap 會被長時間佔滿）
   - 不要在人群中用 RL 探索 — 功能安全不允許；部署前必須用 CBF 兜底

**面試官想聽到**：Freezing Robot Problem 的明確表述；Group-Aware / SARL / SFM 三層分工；「主動擠一下」的社會動力學洞察；知道醫院 / 商場等服務型機器人的商業化價值。

</details>

<details>
<summary>Q6（難）：你用 Isaac Lab 訓練四足機器人端到端 RL 避障，在模擬器漂亮跑過山谷，但真實部署在戶外時頻繁撞樹。怎麼診斷與修復？</summary>

**完整推理鏈**：

1. **診斷根因（F8 家族典型失效）**：
   - **Sim-to-Real Gap**：模擬器紋理 / 光照 / 深度噪聲跟真實差異大 → 網路在模擬學到的「障礙物視覺特徵」在真實失效
   - 可能的具體現象：樹葉遮擋深度相機、陽光直射飽和、樹幹與背景對比度低
2. **修復 1：Domain Randomization**：
   - 訓練時**瘋狂隨機化**：光照（HDR 範圍 8 stops）、紋理（隨機貼圖 / 程序生成 noise）、相機 intrinsics / noise、FOV、動態延遲、摩擦係數、質量分佈
   - 迫使網路放棄特定視覺特徵 → 學真正不變的幾何避障深度
3. **修復 2：Privileged Teacher-Student 蒸餾**：
   - **Teacher**（模擬器上帝視角）：精確 3D 座標 + 速度 + 摩擦 → 快速學完美避障
   - **Student**（真實視覺）：RGB-D 像素 → Behavior Cloning 擬合 Teacher 動作
   - 蒸餾 Loss：$\mathcal{L} = \mathbb{E}[\|\pi_{\text{student}}(a|o_{\text{vision}}) - \pi_{\text{teacher}}(a|s_{\text{priv}})\|^2]$
4. **修復 3：加 F4 CBF 兜底**：
   - RL Policy 輸出 $u_{\text{nom}}$ → CBF-QP 投影到安全空間 → 關節力矩
   - 即使 RL 在樹前決策錯誤，CBF 硬約束保證不撞
5. **修復 4：Diffusion Policy 避免多模態塌縮**：
   - PPO / SAC 可能學到「左繞 / 右繞」的平均 → 撞樹中央
   - Diffusion Policy 保留多模態 → 真實場景隨機採樣左 or 右
6. **驗證順序**：先在 Isaac Sim 加 Domain Randomization 重訓 → 再做 Teacher-Student → 最後疊 CBF → 真實部署小範圍測試 → 擴大

**面試官想聽到**：Sim-to-Real Gap 是 RL 部署的頭號殺手；Domain Randomization + Teacher-Student 是標準答案；CBF 兜底是工程妥協；能提到 Diffusion Policy 解決多模態塌縮。

</details>

## 面試角度

1. **分層規劃架構是核心設計原則** — 證明你理解工程系統的「算力 vs 反應速度」trade-off。**為什麼這是重點**：面試官想看你能否用系統觀思考而非單一演算法。**帶出（兩分鐘版本）**：「我會把規劃系統分成 Global（1 Hz A\*，保證全局可達性）+ Local（30+ Hz DWA/MPPI，保證即時避障）+ Reactive Safety（1 kHz CBF / VO，兜底）。低頻算大方向、高頻做精細閃避、反應層守最後一道門 — 這不是教科書分法，是實際算力預算下的唯一可行解。」

2. **動態預測整合 Costmap 是關鍵差異化能力** — 區分「只會跑 Nav2 demo」和「真能處理動態環境」的分水嶺。**為什麼這是重點**：純 reactive 避障在工業被淘汰超過五年，但新人常還停留在 DWA。**帶出**：「純 reactive 避障對靜態環境夠用，但遇到人群就崩。我會把行人追蹤 + Social-LSTM 預測寫成時空 Costmap layer，預測窗口 2–3 秒，再用 Chance-Constrained MPC 以 3σ 擴張硬約束涵蓋不確定性。這是 L4 自駕的工業標配。」

3. **演算法選擇取決於底盤運動學和算力** — 展現你不是死背演算法，而是根據約束做工程決策。**為什麼這是重點**：選錯演算法比調錯參數更致命，面試官想看工程判斷力。**帶出**：「差速底盤首選 DWA — 簡單高效；阿克曼用 TEB 處理最小轉彎半徑；高維非線性系統上 MPPI — 但前提是有 GPU。選錯演算法比調錯參數更致命。」

4. **ORCA Reciprocal 半平面 + Freezing Robot Problem** — 多機器人避障的深度考點。**為什麼這是重點**：面試官用這題篩掉只會講 RVO 名字但不懂 LP 失敗模式的候選人。**帶出**：「ORCA 把幾何碰撞錐線性化為半平面約束，LP 求解 $O(n)$ 毫秒級。但密集人群所有半平面交集為空時會 Freezing → LP 無解 → 煞停卡死。標準解是硬約束轉軟約束加鬆弛變數，或混合架構升級為區域中央調度。」

5. **Chance-Constrained MPC 以 3σ 把預測不確定性融入控制硬約束** — 這是 L4 自駕工業標配的靈魂句。**為什麼這是重點**：說出這句等於證明你讀過 Waymo / Apollo 的論文；沒說出來等於只會教科書 MPC。**帶出**：「Social-LSTM 預測每個未來時間步行人的 2D 高斯分佈，MPC 把避障約束向外擴張 3σ 涵蓋 99.7% 機率。行為莫測的行人自動拉開更大橫向距離。預測不確定性完美融入控制硬約束 — 這是 L4 自駕的現代答案。」

6. **CBF Forward Invariant Set 是 2020 後安全控制的統一語言** — 表明你跟得上學術前沿。**為什麼這是重點**：CBF 從論文到工業（AEB / Cobot / Atlas）才五年，答得出來證明你在讀最新論文。**帶出**：「$\dot{h} + \alpha(h) \geq 0$ 的意思是越靠近危險邊界速度必須幾何級數衰減至零。CBF 是安全濾網不主動產力，天然無 APF 的 local minima 問題。HOCBF 解 Relative Degree > 1 的位置約束，處理二階系統剎車距離。」

7. **CBF-QP 是 RL 的脊髓反射層**（RL + 硬安全保證的標準工業答題）— 端到端 RL 落地的關鍵妥協。**為什麼這是重點**：這是「RL demo 酷炫但不能過安全認證」的標準解，幾乎所有具身 / 四足 / 機械臂公司都用。**帶出**：「RL 大腦輸出 $u_{\text{nom}}$，CBF-QP 以最小二乘投影到安全空間。微秒級凸優化 → RL 泛化 + 數學物理安全底線。這是 ISO 13849 PLd 功能安全認證下 RL 落地的標準架構。」

8. **SARL Self-Attention 處理變長人群 + Group-Aware 禮貌繞行** — Social-Aware 現代架構核心。**為什麼這是重點**：服務型機器人商業化最後一哩（醫院、商場、機場），卡死 = 商業失敗。**帶出**：「密集人群 N 變動 → 傳統 RL 難處理。SARL 用 Self-Attention 只聚焦可能碰撞的關鍵行人，變長狀態壓縮為定長 Context Vector。Group-Aware 用 F-formation 檢測三人聊天圈 → 繞外側走。這決定服務機器人是笨拙還是優雅融入人類社會。」

9. **FASTER 雙軌跡並行 + Flight Corridor B-Spline 凸包性** — 高速無人機避障的數學優雅。**為什麼這是重點**：10 m/s 穿梭無人機建圖延遲足以墜毀，必須有安全備援。**帶出**：「FASTER 每週期同時解 Primary（激進）+ Backup（保證能煞停）兩條軌跡。失敗時無縫切 Backup → 絕對安全。Flight Corridor 把 3D Free Space 膨脹成重疊凸多面體，B-Spline 凸包性保證整條曲線不穿出 → 非凸避障變凸 QP 幾 ms 求解。」

10. **機械臂 $\mathbf{J}^\top \mathbf{F}_{\text{rep}}$ + 零空間「手肘退讓水杯紋絲不動」是 ISO 10218 / 15066 PLd 認證的數學骨架** — Cobot 商業化的簽名賣點。**為什麼這是重點**：Cobot 客戶（UR / KUKA / Franka 使用者）買單的是**安全**，能講出認證底層數學證明你懂工業邏輯而非 toy example。**帶出**：「各連桿 Capsule 包裹，雅可比轉置把笛卡爾斥力映射為關節避讓力矩。7-DoF 冗餘投影到零空間 → 工人推手肘時手肘柔順退讓，但末端拿的水杯紋絲不動。這就是 ISO 15066 PLd 認證的底層數學。」

11. **Privileged Teacher-Student 蒸餾是 Sim-to-Real 的標準範式** — 端到端 RL 避障的工業答案。**為什麼這是重點**：ETH ANYmal、Boston Dynamics Atlas 都用這架構，不會答等於落伍。**帶出**：「Teacher 在模擬器吃上帝視角（精確 3D + 摩擦）學完美避障 → Student 真實只吃相機 → Behavior Cloning 擬合 Teacher 動作。再加 Domain Randomization（瘋狂隨機化光照、紋理、噪聲）讓 Student 學幾何不變特徵，避免過擬合模擬器渲染。」

12. **Diffusion Policy 避免「PPO 取平均撞牆」的多模態塌縮** — 2024 前沿必答。**為什麼這是重點**：面試官用這題篩「只讀到 2023 年論文」vs「跟得上 2024 前沿」的候選人。**帶出**：「PPO / SAC 高斯分佈取平均 → 左繞和右繞平均起來撞障礙物中央。Diffusion Policy 把避障動作視為去噪生成，完美保留多模態解。這就是 Columbia 2023 論文為什麼在機器人界爆紅。」

## 延伸閱讀

- **Fox et al., *The Dynamic Window Approach to Collision Avoidance* (1997)** — F1 家族 DWA 原始論文，速度空間採樣的開山之作，理解 DWA 的設計動機和動態窗口數學
- **Rösmann et al., *Timed-Elastic-Band Local Planner* (2013–2017 系列)** — TEB 從基礎概念到多拓撲路徑選擇的完整演進，理解時空彈性帶優化
- **Williams et al., *Information Theoretic MPC (MPPI)* (2017)** — MPPI 的資訊理論基礎，理解為什麼 Boltzmann 加權比簡單取 arg min 更穩健
- **van den Berg et al., *Reciprocal n-Body Collision Avoidance (ORCA)* (2011)** — F2 家族分散式避障的數學保證，理解 reciprocal 半平面的幾何意義
- **Alonso-Mora et al., *Collision Avoidance for Quadrotor Swarms under Uncertainty* (2019)** — ORCA + Chance Constraints，F2 / F3 結合的工業案例
- **Alahi et al., *Social LSTM: Human Trajectory Prediction in Crowded Spaces* (2016)** — F3 家族行人軌跡預測的經典，Social Pooling 層理解
- **Salzmann et al., *Trajectron++* (2020)** — 比 Social-LSTM 更強的多模態軌跡預測，自駕工業主流
- **Ames et al., *Control Barrier Functions: Theory and Applications* (2019)** — F4 家族 CBF / HOCBF 權威綜述，從理論到 RL 安全控制的應用
- **Chen et al., *Socially Aware Motion Planning with Deep Reinforcement Learning (CADRL/SARL)* (2017-2019)** — F5 家族 Social-Aware RL 的開山之作
- **Helbing & Molnár, *Social Force Model for Pedestrian Dynamics* (1995)** — F5 家族 SFM 的原始論文，理解反應式社會力學基礎
- **Tordesillas et al., *FASTER: Fast and Safe Trajectory Planner* (IROS 2019)** — F6 家族雙軌跡並行的經典，MIT CSAIL 代表作
- **Zhou et al., *EGO-Planner* (RA-L 2021)** — F6 家族 Flight Corridor / B-Spline 的工業實作
- **Ranftl et al., *MiDaS / ZoeDepth* (2020-2023)** — F7 家族單目深度估計，Tesla FSD / Skydio 純視覺的關鍵技術
- **Chi et al., *Diffusion Policy* (RSS 2023)** — F8 家族 Diffusion 保留多模態動作分佈，2024 機器人界爆款
- **Hwangbo et al., *Learning Agile and Dynamic Motor Skills for Legged Robots* (Science Robotics 2019)** — F8 家族 Privileged Teacher-Student 蒸餾的經典，ANYmal 四足戶外避障
- **ISO/TS 15066 — Collaborative Robots Speed and Separation Monitoring** — F9 家族 Cobot 安全認證標準，SSM / Power and Force Limiting 的工程規範
- **Haddadin et al., *Robot Collisions: A Survey* (RAL 2017)** — F9 家族機械臂 HRI 避障全景綜述
- **ROS 2 Nav2 官方文件 — Controller Plugins** — DWA / TEB / MPPI 三種 controller 的完整配置參考與調參指南
- **NVIDIA Isaac Lab / cuRobo** — GPU 加速的 motion planning 與避障，適合高維機械臂場景的即時規劃
- **《具身智能算法工程師 面試題》Ch8.3–8.5 動態避障 / 社會感知 / RL 避障** — 面試高頻考點：分層架構、預測整合、CBF 安全層的 talking points
