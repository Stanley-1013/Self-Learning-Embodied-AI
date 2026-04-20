# Audit Report: Ch11-14

**Summary**: 10 flags total — HIGH: 1, MED: 6, LOW: 3

---

## Ch11 Trajectory Optimization

### Findings

- **[LOW]** 11:44 — 表格寫「三次 Cubic $C^1$（速度連續）」的致命缺陷是「加速度端點跳變 → jerk 無窮」— 嚴格來說，單段三次多項式 acceleration 本身是「連續且線性」的（$\ddot q = 2a_2 + 6a_3 t$），問題是**段與段之間的交接點**上 acceleration 不連續，且單段起點/終點的 acceleration 沒被邊界條件固定（預設不為 0）→ 交接時 jerk 發散。目前敘述容易讓讀者誤解為「單段內部加速度會跳變」。Fix: 改為「多段 cubic 拼接時 acceleration 在交接點跳變」或「單段起/終點 acceleration 無法指定 → 多段銜接時 jerk 發散」。Confidence: high.

- **[MED]** 11:227 — Differential Flatness 段落寫「四旋翼是 12 狀態 `[x,y,z,ẋ,ẏ,ż,φ,θ,ψ,p,q,r]` 只有 4 馬達輸入」但接著又說「所有 12 狀態和 4 控制輸入都能從 σ 及其有限階導數導出」。Mellinger-Kumar 2011 的原始敘述是「flat outputs 決定 12 state + 4 input」沒錯，但「12 states」這個計數在常見 roll-pitch-yaw 表述下只有 12 個（3 pos + 3 vel + 3 euler + 3 body-rate），若用 quaternion 則 13。文中 φ,θ,ψ 與 p,q,r 並列本身沒問題，但後文討論 flat outputs 時 σ = [x, y, z, ψ] 的 ψ 是**yaw（偏航）**，而前面狀態變數列表裡 ψ 已經是 yaw（歐拉 RPY 第三個） — 於是「12 狀態裡有 ψ」「flat outputs 也有 ψ」雙用同一個符號沒說清楚是同一個量。Fix: 明確標註「σ 的第 4 個元素就是狀態變數中的 yaw ψ（不含 roll / pitch，因它們可由 position trajectory 二階導數反推）」。Confidence: medium.

- **[MED]** 11:629-643 — iLQR Backward Pass 完整推導的 Value function 更新式：`V_x = Q_x + K^T Q_uu k + K^T Q_u + Q_ux^T k`。代入最優控制 $k = -Q_{uu}^{-1} Q_u$, $K = -Q_{uu}^{-1} Q_{ux}$ 化簡後標準結果是 $V_x = Q_x - Q_{ux}^T Q_{uu}^{-1} Q_u$（等價於 $Q_x + K^T Q_u$，因其他項消掉）。文中寫法保留四項不化簡，代數不是錯，但項之間符號關係有陷阱（必須搭配 $k, K$ 的負號定義才正確）；若讀者只看這一步，容易算錯。Fix: 補上「代入 $k, K$ 定義後化簡為 $V_x = Q_x + K^T Q_u$」或直接寫簡化形。Confidence: medium（推導非錯誤，但容易誤用）。

---

## Ch12 Dynamic Obstacle Avoidance

### Findings

- **[MED]** 12:73-75 — ORCA 半平面寫成 `(v_A - (v_A^opt + u/2)) · n ≥ 0`。原始 van den Berg et al. 2011 定義中，u 是把 A 的相對速度推出 VO 邊界的最小向量（垂直於 VO 邊界、指向 VO 外），n 取作 **u 的單位方向向量**（不是 VO 邊界法線，是推離方向）— 兩者在幾何上等同但表達有別。文字寫「n 是垂直法向量」沒有指定方向（朝內還是朝外），容易讓讀者誤判半平面朝向（整個不等式符號就會反）。Fix: 改為「n 為 u 的單位向量，指向 VO 外側 → 半平面約束保證 A 的新速度落在遠離碰撞的一側」。Confidence: high.

- **[LOW]** 12:664 — ISO/TS 15066 SSM 速度公式寫成 `v_max = max(0, (S_p - (S_B + S_R + S_C)) / T)`。標準 ISO/TS 15066 的「保護分離距離 $S_p$」公式是 **$S_p(t_0) = S_H + S_R + S_S + C + Z_d + Z_r$**（距離加總），「最大允許速度」的反解公式要顯式指定時間區間 T 是什麼（煞車時間？反應時間？），文字未標註。實務上工業 SSM 是求 $S_p$ 是否滿足、而非直接算 $v_{\max}$。Fix: 加一句「此為簡化反解，實際 ISO/TS 15066 以 $S_p$ 比較門檻為主，T 為機器人反應+煞車時間總和」。Confidence: medium.

- **[LOW]** 12:67 — APF local minima 段落寫「吸引梯度 = 斥力梯度方向相反 → 合力為零死鎖」。嚴格講「合力為零」不必等於「方向相反且大小相等」才發生 — 在任何 $-\nabla U_{\text{att}} + \sum -\nabla U_{\text{rep},i} = 0$ 的點都會卡住。「兩柱子完全對稱」只是其中一種。通道型 local minima（U 型牆後目標）更常見，不需斥力與引力等大反向。Fix: 改為「吸引與斥力合力為零或局部 potential 極小」，或點出「U 型走廊 / 對稱障礙 / 窄通道」都可觸發」。Confidence: medium.

- **[MED]** 12:87-91 — CBF 條件寫 `ḣ(x, u) + α(h(x)) ≥ 0, h(x) ≥ 0 ⟺ x ∈ C`。第二個等式用了「⟺」— 實際上安全集合定義是 `C = {x | h(x) ≥ 0}`，因此 `h(x) ≥ 0 ⟺ x ∈ C` 成立。但緊接「$\alpha(h)$ 是允許的最大靠近速度」敘述有點繞：$\dot h + \alpha(h) \geq 0$ 意味「h 下降速率 ($-\dot h$) ≤ $\alpha(h)$」，也就是靠近邊界速度被 $\alpha(h)$ **上限**限制。文字「正比於距邊界距離」若讀成「可以靠近的速率與離邊界成正比」OK，但易誤讀為「α 是實際靠近速度」（其實是上限）。Fix: 明確寫「$-\dot h$ ≤ $\alpha(h)$，越靠近邊界 $h \to 0$，允許的最大靠近速率 $\alpha(h) \to 0$」。Confidence: medium.

---

## Ch13 PID Control Tuning

### Findings

- **[MED]** 13:492 — Gain Scheduling 公式 $K_p(q) = K_{p_0} \cdot \det(M(q))/\det(M_0)$。實務上 gain scheduling 依**關節處的有效慣量**（典型用 $M_{ii}(q)$ 或操作空間 $\Lambda_{ii}(q)$，即對角元素／用此方向慣量）縮放 $K_p$，**不會用整個 mass matrix 的 determinant**。Determinant 隨姿態可能劇烈變化（奇異附近趨近 0 或突增），且無直接對應 SISO PID 下「這個關節感受多少慣量」的物理意義。$\det(M)$ 變化不單調於單軸有效慣量 → 照此公式會讓 $K_p$ 在某些姿態不合理地大或小。Fix: 改為 $K_p(q) = K_{p_0} \cdot M_{ii}(q)/M_{ii,0}$ 或基於操作空間慣量 $\Lambda_{ii}(q)$ 的縮放。Confidence: high.

- **[LOW]** 13:432 — Tustin/Bilinear 離散化公式 $s = \frac{2}{T}\cdot\frac{1-z^{-1}}{1+z^{-1}}$ 正確，但敘述「左半 s 平面完整映射到 z 單位圓內」不嚴謹 — 嚴謹說法是「左半 s 平面雙線性映射到 z 單位圓**內部**（穩定極點保持穩定）」且「虛軸 $j\omega$ 被扭曲到單位圓上，造成 **frequency warping**」（這是 Tustin 的著名特性，實務常搭配 pre-warping 抵消）。目前敘述漏了 frequency warping 這個重要警告。Fix: 補充「虛軸頻率有 warping 現象，實作時常以 pre-warping 校正關鍵頻點」。Confidence: medium。

---

## Ch14 Force & Impedance Control

### Findings

- **[MED]** 14:267 — Adjoint Transform 寫 `Ad_T^T = [R, p̂R; 0, R]`。若 `Ad_T` 按標準 Lynch & Park 的 twist 鄰接 $\text{Ad}_T = \begin{bmatrix} R & 0 \\ \hat p R & R \end{bmatrix}$ 直接轉置，應得 $\text{Ad}_T^T = \begin{bmatrix} R^T & (\hat p R)^T \\ 0 & R^T \end{bmatrix} = \begin{bmatrix} R^T & -R^T \hat p \\ 0 & R^T \end{bmatrix}$，與文字的 `[R, p̂R; 0, R]`（沒轉置 R 且符號正）不符。文字形式若要成立，隱含的是 `Ad_{T^{-1}}^T` 或是 wrench 以 $[m;f]$ 排序、frame 方向與推導不同的另一種 convention。直接套用此公式到不同 convention 會出現符號錯誤。Fix: 明確標註所用 convention（wrench 排序是 $[m;f]$ 還是 $[f;m]$、$T$ 方向是 $A\to B$ 還是 $B\to A$），或引用 Lynch & Park §3.4 的標準形式。Confidence: medium。

- **[HIGH]** 14:291-294 — Adjoint C++ 片段與同段文字公式**內部不一致**。文字公式寫 `Ad_T^T = [R, p̂R; 0, R]`（p̂R 在**右上**），但程式碼卻把 `skew_symmetric(p) * R` 塞在 `bottomLeftCorner`（**左下**），且 `topRightCorner = Zero`。也就是說，程式碼構造的是 $\text{Ad}_T$（不是 $\text{Ad}_T^T$），然後又用 `Ad_T.transpose() * K * Ad_T` 做共軛 — 從程式碼角度看，若 `Ad_T` 是標準 twist adjoint，則 `Ad_T.transpose()` 就是 wrench adjoint，這 OK；但此時上面那行「$W_A = \text{Ad}_T^T \cdot W_B$，$\text{Ad}_T^T = [R, \hat p R; 0, R]$」就與程式不對應了。讀者若照公式手寫會得到與程式不同的矩陣，實作結果對不上。Fix: 統一為「先定義 `Ad_T = [R, 0; p̂R, R]`（twist），然後 `W_A = Ad_T^T W_B`、`K_tcp = Ad_T^T K_flange Ad_T`」，讓公式與程式碼結構一致。Confidence: high.

