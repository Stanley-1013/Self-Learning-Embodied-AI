# Extracted: 逆向運動學求解與奇異點分析
<!-- session: fe23a257, Q18 (all-in-one) -->

## 定義
- IK：由末端位姿反解關節角度；非線性、多解/無解
- 解析解 vs 數值解：解析=O(1)但需 Pieper 準則(腕部球形)；數值=牛頓法通用但可能局部最優
- Jacobian：∂x/∂q，關節速度→末端速度的線性映射
- J⁻¹ vs J⁺：方陣非奇異用逆；冗餘/最佳化用偽逆(最小關節運動)
- Singularity：J 秩虧損→自由度丟失→速度爆掉/力矩發散
- DLS：J^T(JJ^T+λ²I)⁻¹ẋ，犧牲微量精度換穩定
- 6-DoF 普遍有 8 組解

## 閉環
- IK = 翻譯官：任務空間目標 → 關節空間指令
- 輸入：末端期望位姿/速度；輸出：關節角度/速度
- 上游：FK+規劃；下游：PID/力矩控制
- 一句話：「IK 把人類 3D 意圖拆解為馬達旋轉指令；奇異點分析是確保引擎不除以零的安全閥」

## 核心數學
- ẋ = J(q)q̇ — 關節微動→末端微動
- q̇ = J⁺ẋ — 偽逆最小關節運動
- q̇ = J^T(JJ^T+λ²I)⁻¹ẋ — DLS 阻尼穿越
- 7-DoF：q̇ = J⁺ẋ + (I-J⁺J)q̇₀ — null-space 次要任務
- SVD condition number 監控奇異距離

## 直覺 + 誤解
- 類比：手摸杯子但手肘可不同位置
- 4 誤解：IK 只有一解、解析永遠好、忽略奇異直線走、忘 joint limits

## 3 情境題
1. 某些位置解不出 → 檢查可達性/joint limits/Jacobian 行列式
2. 走直線馬達電流暴增 → SVD condition number 確診內部奇異 → DLS 或 B-spline 繞道
3. 7-DoF 無窮解選最好 → J⁺主任務 + null-space(I-J⁺J)q̇₀ 次任務(避障/避限位/省電)

## Talking points / 延伸 / 閱讀
- 3 points：解析+數值混合求解、SVD condition number 自適應 DLS、null-space 利用
- 延伸：Null-space projection、Manipulability Ellipsoid、Levenberg-Marquardt
- 閱讀：《面試題》Ch2 IK → Ch3 奇異點
