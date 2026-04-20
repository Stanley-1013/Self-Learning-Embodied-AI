# Audit Report: Ch06-10

**Summary**: 14 flags total — HIGH: 3, MED: 7, LOW: 4

---

## Ch06 ROS 2 Advanced Features

### Findings

- **[LOW]** 06:31 — `History` 表格行把 `KEEP_LAST(N)` 寫成 Policy，`Depth` 另列一行，但實務上 Depth 是 `KEEP_LAST` 的參數不是獨立 QoS policy（DDS 規範下 History 只有 KEEP_LAST / KEEP_ALL 兩個策略 + 一個 depth 整數參數）。Fix: 把 Depth 併入 History 行或標註「參數而非獨立策略」。Confidence: high.

- **[LOW]** 06:51 — `ParameterEventsQoS` preset 表格寫 "Keep last 1000"。ROS 2 rclcpp 的 `rmw_qos_profile_parameter_events` 實際 depth = 1000 且 Durability 是 `VOLATILE`（不是 Transient Local）— 本行 Durability 欄寫 "Transient local" 不符 rmw defaults。Fix: 改成 `Volatile`（若確認是 rclcpp 預設）。Confidence: medium（各 release 略有差異，但主線 rmw default 是 volatile）。

- **[MED]** 06:105 — 「觸發 `IncompatibleQosEvent`（但預設沒人監聽）」— 正確的事件名稱是 `RequestedIncompatibleQoSEvent`（Subscriber 端）與 `OfferedIncompatibleQoSEvent`（Publisher 端），沒有單一的 `IncompatibleQosEvent` symbol。Fix: 改為正式事件名。Confidence: high.

---

## Ch07 Forward Kinematics & DH

### Findings

- **[MED]** 07:79 — 「Standard DH：frame $i$ 附著在連桿 $i$ **末端**」。標準 Denavit-Hartenberg 約定是：frame $i$ 的 Z 軸與關節 $i+1$ 軸對齊（即 frame i 「擺在連桿 i 末端的下一個關節」），說法「末端」成立但接下來 $a_i, \alpha_i$「用第 i 軸到第 i+1 軸的關係」這句與 frame 附著位置的描述實際上對應 Modified DH 的「frame i 在連桿 i 起端 / 關節 i 處」的鏡像描述。讀者容易搞混。Fix: 直接寫 Standard DH 中 Z_i 對齊 joint i+1 的軸，Modified DH 中 Z_i 對齊 joint i 的軸，避免「起端/末端」這種含糊用語。Confidence: medium（說法不完全錯，但易誤導）。

- **[LOW]** 07:168-175 — UR5 DH 表「近似值」OK，但 $d_4 = 0.109$、$d_5 = 0.095$、$d_6 = 0.082$ 實機 Universal Robots 原廠值為 $d_4 = 0.10915$、$d_5 = 0.09465$、$d_6 = 0.0823$。四捨五入差異會讓末端位置算出幾 mm 誤差，練習題 Q1 拿來跟 TF GT 比會對不上 1e-6。Fix: 建議加一行註記「以下為四捨五入值，精確值請查 UR 官方手冊」。Confidence: high.

---

## Ch08 Inverse Kinematics & Singularity

### Findings

- **[MED]** 08:44 — 偽逆公式 $J^+ = J^T(JJ^T)^{-1}$ 寫成通式，但這是 **right pseudo-inverse**，只適用 $J$ 行滿秩（$n \geq 6$，冗餘或方陣）。對於 over-determined 情境（task 維度 > DoF，如以 6DoF 手臂追蹤 7DoF 任務，罕見但存在）應改用 left pseudo-inverse $(J^T J)^{-1} J^T$。標示條件或直接寫 SVD 形式更嚴謹。Fix: 加上「適用 $n \geq 6$ 且 $J$ 行滿秩」的條件說明。Confidence: high.

- **[MED]** 08:135 — 數值解 pseudocode 寫 `dq = J_dls(q)^(-1) * e`，把 DLS 包裝成「某個 J_dls 矩陣的逆」語法有誤導性 — DLS 解的正確寫法是 `dq = J^T * (J*J^T + λ²I)^{-1} * e`，沒有獨立的 `J_dls` 矩陣可以求逆。Fix: 改為直接寫出阻尼解形式或註記 `J_dls^{-1}` 是比喻性符號。Confidence: high.

- **[LOW]** 08:104 — 「腕部奇異 (wrist singularity)：第 4、6 軸共線」對 UR5/PUMA 這類 spherical wrist 來說是**第 4 與第 6 軸**共線（中間第 5 軸旋轉到特定角度時）。文字 OK，但實務描述常寫「$\theta_5 = 0$ 時 axes 4 與 6 共線」，加上這個角度條件會更明確。Fix: 補充「當 θ_5 = 0 時」條件。Confidence: medium.

---

## Ch09 Robot Dynamics Modeling

### Findings

- **[HIGH]** 09:469 — 物理一致性 LMI 條件寫 "$\text{tr}(I_i) > \max\text{eigval}$" 作為三角不等式。正確的慣性張量三角不等式是 **principal moments 的 pairwise sum ≥ 第三個**：$I_{xx} + I_{yy} \geq I_{zz}$（以及 xy/yx 的所有排列）。文字的「trace > max eigenvalue」是此條件的**必要但非充分**弱化 — 會漏掉一些物理上不可實現的 inertia。SysID LMI 實作會直接錯。Fix: 改為 $I_{jj} + I_{kk} \geq I_{ii}$ 對所有 $i \neq j \neq k$ 成立（principal axes 下），或寫出 Souloumiac pseudo-inertia matrix ≥ 0 的 LMI 形式。Confidence: high.

- **[MED]** 09:505 — 「$\dot{V} = -s^T K s \leq 0 \to e \to 0$」跳過 Barbalat's Lemma 步驟。$\dot V \leq 0$ 只能得 $V$ 單調非增且有界（$s \in L^\infty$），要得 $s \to 0$（進而 $e \to 0$）必須搭配 Barbalat（$\dot V$ uniformly continuous + V 有下界）。Fix: 加一句「由 Barbalat's Lemma 可推 $s \to 0$，進而 $e \to 0$」。Confidence: high.

- **[MED]** 09:117-125 — RNEA Forward pass 寫 $\omega_i = R_i^T(\omega_{i-1} + \dot q_i \hat z)$，此處 $\hat z$ 究竟是 frame i-1 還是 frame i 的 z 軸無標註；第三式 $\ddot p_i$ 缺少連桿原點平移向量 $r_i$ 的 cross product 項中 $r_i$ 是在哪個 frame 表達也無標註。對想自己實作的讀者容易搞錯。Fix: 明確標註向量所處 frame（典型做法：所有向量用 frame i 表達，$\hat z = [0,0,1]^T$ 代表 joint i+1 軸在 frame i 下）。Confidence: medium（非錯誤但 under-specified）。

- **[LOW]** 09:131 — Backward pass $f_i = m_i \ddot p_{c_i} + R_{i+1} f_{i+1}$ 少了重力項。雖然重力通常透過 base pseudo-acceleration trick 注入（forward pass 中 $\ddot p_0 = -g$），但在公式旁未註明此 trick，讀者可能誤以為該公式本身已包含重力。Fix: 明確註記 "假設 base 加速度初值為 $-g$，重力效應透過 forward pass 傳遞"。Confidence: medium.

- **[MED]** 09:227-231 — CMM 定義 $A(q)\dot q = [l; k]$，此處 $l$ 是 linear momentum、$k$ 是 angular momentum。Pinocchio 的 `data.hg`（Force spatial vector）排列是 **[linear; angular]**（與文字符合），但文字同時稱 $A_G$ 是 "centroidal momentum matrix"，慣例不同 paper 排序不同（Orin & Goswami 2008 原文採用 [angular; linear]，即 $h_G = [k; l]$）。建議明標採用哪個 convention 避免抄公式時 index 出錯。Fix: 加一句「本文採 Pinocchio convention：上 3 維線動量，下 3 維角動量」。Confidence: medium.

---

## Ch10 Basic Path Planning

### Findings

- **[HIGH]** 10:322 — 「Chebyshev distance：8-connected grid，斜向與直向同代價」— Chebyshev 只有在斜向代價 = 直向代價 = 1 時才 admissible。若 8-connected grid 採用物理真實的 $\sqrt{2}$ 斜向代價（常見預設），Chebyshev **會低估**真實最短距離，但這時用 octile distance（文中 "Diagonal distance" 那行）才對。原表述會讓讀者在錯誤代價設定下誤用。Fix: 補充「僅在斜向代價=1 時 admissible；若斜向代價=$\sqrt{2}$，請改用 octile/diagonal distance」。Confidence: high.

- **[HIGH]** 10:940 — Cost decay 公式寫 `Cost(d) = 252 · exp(-α · (d - r_inscribed))`。Nav2 InflationLayer 實際公式是 `cost = (LETHAL_OBSTACLE - 1) * exp(-factor * (d - inscribed_radius))`，LETHAL_OBSTACLE = 254，故係數應為 **253**（或 `INSCRIBED_INFLATED_OBSTACLE` 相關值），不是 252。此外在 $d \leq r_{\text{inscribed}}$ 的區域 cost 直接設為 254（LETHAL），公式只在 inflation 區間有效。Fix: 改為 253 並標註 cost 在 inscribed 內是 254，inscribed ~ inflation_radius 之間才用 exp 衰減。Confidence: high（對照 Nav2 source: `nav2_costmap_2d/plugins/inflation_layer.cpp`）。

- **[MED]** 10:173 — 「Dubins Car 最優路徑必為 **CCC（圓弧-圓弧-圓弧）或 CSC（圓弧-直線-圓弧）**，共 6 種組合」。Dubins 1957 結果是 6 條候選路徑 {LSL, RSR, LSR, RSL, LRL, RLR}，其中 CCC 型（LRL, RLR）只在特定距離條件下是最優，其他情況是 CSC 型。敘述「必為」對於某些短距離情況正確，但口語「6 種組合」應標註這是完整 optimal word 字碼集合。Fix: 微調為「Dubins 最優解必落在 6 種字碼 {LSL, RSR, LSR, RSL, LRL, RLR} 之一」。Confidence: medium.

- **[LOW]** 10:173 — 「Reeds-Shepp Car 46 種字碼」— 1990 Reeds & Shepp 原文給出 48 words 的候選集合（其中 46 條在 distinct base word 上再考慮 backward shuffles）。學界引用時兩個數字都見得到（48 base / 46 distinct patterns）。非致命，但嚴謹寫法是「48 candidate words（其中 46 種 distinct pattern）」。Fix: 備註「48 words per Reeds-Shepp 1990；亦常見 46 distinct patterns」。Confidence: medium.

---
