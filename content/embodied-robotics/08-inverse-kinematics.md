---
title: "逆向運動學求解與奇異點分析"
prerequisites: ["07-forward-kinematics-dh"]
estimated_time: 45
difficulty: 4
tags: ["inverse-kinematics", "jacobian", "singularity", "dls"]
sidebar_position: 8
---

# 逆向運動學求解與奇異點分析

## 你將學到

- 能用兩句話講清楚 inverse kinematics (IK) 與 forward kinematics 的本質差異：FK 是唯一映射，IK 是多解 / 無解的非線性反問題
- 遇到「機械臂走直線時馬達電流突然暴增」或「某些目標位置解不出來」，知道先從 Jacobian singular value 和 workspace boundary 切入分析
- 能在面試中說清楚解析解 vs 數值解的取捨、DLS (Damped Least Squares) 的作用、以及 7-DoF 冗餘臂的 null-space 利用策略

## 核心概念

**精確定義**：**Inverse kinematics (IK)** 是從末端執行器的目標位姿（position + orientation）反解關節角度 $q$ 的過程。數學上是求 $f(q) = x_{\text{desired}}$ 的反函數，其中 $f$ 是 FK 映射。與 FK 不同，IK 本質上是**非線性方程的求解**，通常存在**多組解、無解、或無窮解**。

**在感知 → 規劃 → 控制閉環的位置**：
- **輸入**：末端期望位姿 $x_d \in SE(3)$（來自規劃器或視覺伺服）、當前關節角 $q_{\text{current}}$（來自編碼器）
- **輸出**：目標關節角度 $q_d$（位置級 IK）或目標關節速度 $\dot{q}_d$（速度級 IK）
- **下游**：PID / 力矩控制器接收 $q_d$ 或 $\dot{q}_d$ 執行關節空間追蹤；軌跡規劃器在每個 waypoint 調用 IK 做 Cartesian 插值
- **閉環節點**：IK 是**規劃 → 控制的翻譯官**，把人類直覺的 3D 空間指令（「手移到杯子上方 5 cm」）拆解為每顆馬達的旋轉指令

**一句話版本**：「IK 把人類的 3D 意圖拆解為馬達旋轉指令；奇異點分析是確保這個翻譯過程不會除以零的安全閥。」

**最少夠用的數學**：

1. **Jacobian 與微分運動學**（關節微動 → 末端微動的線性映射）：

$$
\dot{x} = J(q)\,\dot{q}
$$

$J \in \mathbb{R}^{6 \times n}$ 是 Jacobian 矩陣，每列代表一個關節速度對末端 6D 速度（3 平移 + 3 旋轉）的貢獻。**物理意義**：在當前構型 $q$ 附近，末端怎麼動取決於每個關節怎麼動乘上 Jacobian 這個「放大器矩陣」。

2. **偽逆解**（最小關節運動的速度級 IK）：

$$
\dot{q} = J^+(q)\,\dot{x}, \quad J^+ = J^T(JJ^T)^{-1}
$$

**物理意義**：在所有能達成末端速度 $\dot{x}$ 的關節速度中，偽逆挑出 $\|\dot{q}\|$ 最小的那組——用最少的關節運動達成任務。但 $JJ^T$ 接近奇異時，$(JJ^T)^{-1}$ 爆掉。

3. **Damped Least Squares (DLS)**（穿越奇異點的穩定解）：

$$
\dot{q} = J^T(JJ^T + \lambda^2 I)^{-1}\,\dot{x}
$$

**物理意義**：加一個阻尼項 $\lambda^2 I$ 當「安全氣囊」——奇異點附近 $JJ^T$ 趨近零時，$\lambda^2 I$ 保證矩陣可逆。代價是末端追蹤精度微降，但關節速度不會爆掉。$\lambda$ 越大越安全但越不準。

4. **Null-space 投影**（7-DoF 冗餘臂的次任務利用）：

$$
\dot{q} = J^+\dot{x} + (I - J^+J)\,\dot{q}_0
$$

**物理意義**：$J^+\dot{x}$ 負責主任務（追蹤末端軌跡），$(I - J^+J)\dot{q}_0$ 在不影響末端的前提下，利用冗餘自由度做次任務——避障、遠離關節限位、最大化 manipulability。

<details>
<summary>深入：Jacobian 奇異值分解 (SVD) 與奇異點的數學本質</summary>

對 Jacobian 做 SVD 分解：

$$
J = U \Sigma V^T
$$

- $U \in \mathbb{R}^{6 \times 6}$：末端空間的正交基（每列是一個末端運動方向）
- $\Sigma = \text{diag}(\sigma_1, \sigma_2, \dots, \sigma_r, 0, \dots)$：奇異值，按大到小排列
- $V \in \mathbb{R}^{n \times n}$：關節空間的正交基

**奇異點的判定**：當 $\sigma_{\min} \to 0$ 時，Jacobian 秩虧損（rank deficient），代表某個末端運動方向在當前構型下「無法被任何關節組合實現」——自由度丟失。

**Condition number** $\kappa = \sigma_{\max} / \sigma_{\min}$：衡量離奇異點多遠。$\kappa > 100$ 時建議啟動 DLS；$\kappa \to \infty$ 就是奇異。

**自適應 DLS**：根據 condition number 動態調整 $\lambda$：

$$
\lambda^2 =
\begin{cases}
0 & \text{if } \sigma_{\min} \geq \epsilon \\
\left(1 - \left(\frac{\sigma_{\min}}{\epsilon}\right)^2\right) \lambda_{\max}^2 & \text{otherwise}
\end{cases}
$$

遠離奇異時 $\lambda = 0$（退化為偽逆，精度最高）；接近奇異時 $\lambda$ 漸增（犧牲精度換穩定）。

**偽逆的 SVD 形式**：

$$
J^+ = V \Sigma^+ U^T, \quad \Sigma^+ = \text{diag}(1/\sigma_1, \dots, 1/\sigma_r, 0, \dots)
$$

從這裡可以清楚看到：$\sigma_i \to 0$ 時 $1/\sigma_i \to \infty$，關節速度會爆掉——這就是奇異點的數學根源。

**6-DoF 手臂的典型奇異構型**：
- **腕部奇異 (wrist singularity)**：第 4、6 軸共線 → 繞腕部的旋轉自由度退化
- **肩部奇異 (shoulder singularity)**：末端位於肩部球面邊界 → 機械臂完全伸直，無法再往外推
- **肘部奇異 (elbow singularity)**：第 2、3 軸使手臂完全摺疊或伸直 → 平面內運動自由度丟失

每種奇異構型都對應 $J$ 的特定行/列組合的線性相依。

</details>

<details>
<summary>深入：解析解 vs 數值解的完整比較與 Pieper 準則</summary>

**解析解（Closed-form Solution）**：

適用條件——**Pieper 準則**：6-DoF 手臂滿足以下之一即可得封閉解析解：
- 三個相鄰軸交於一點（最常見：腕部球形關節，如 UR5、KUKA iiwa 的最後三軸）
- 三個相鄰軸平行

**求解思路**（以球形腕為例）：
1. **位置解耦**：腕心位置只由前 3 軸決定 → 解 $\theta_1, \theta_2, \theta_3$（幾何法 + 三角恆等式）
2. **姿態解耦**：腕部 3 軸只影響姿態 → $^3R_6 = (^0R_3)^{-1} \cdot ^0R_6$，用 Euler angle 反解 $\theta_4, \theta_5, \theta_6$
3. 每個 $\theta_i$ 通常有 ±2 組解 → 6-DoF 最多 $2^3 = 8$ 組解

**優勢**：$O(1)$ 計算、無迭代收斂問題、可得**所有**解再選最優
**劣勢**：僅適用符合 Pieper 準則的構型；推導繁瑣且易出錯

**數值解（Iterative Solution）**：

```
while ||x_current - x_desired|| > tol:
    e = x_desired - FK(q)
    dq = J_dls(q)^(-1) * e      // DLS 或牛頓法
    q = q + alpha * dq           // alpha: step size
    enforce_joint_limits(q)
```

**優勢**：通用、不需特殊構型、可加 joint limits / obstacle 約束
**劣勢**：可能陷入局部最優、需好的初值、收斂速度不保證

**業界實務**：
- **工業臂（UR、KUKA、Fanuc）**：解析解為主（速度快、全解可選），MoveIt 的 IKFast 插件自動從 URDF 生成 C++ 解析解
- **人形/蛇形/非標機械臂**：數值解為主（KDL、TRAC-IK、Pinocchio）
- **最佳實踐**：解析解做初值 → 數值法精修 → DLS 穿越奇異

</details>

**常用 API**（業界工具鏈）：

| 層級 | 套件 | 介面示例 |
|------|------|----------|
| ROS 2 通用 | MoveIt / KDL | `KDL::ChainIkSolverPos_NR(chain, fk_solver, ik_vel_solver)` |
| 高效數值 | TRAC-IK | 比 KDL 穩定，自動在 KDL + SQP 間切換 |
| 解析解生成 | IKFast (OpenRAVE) | 從 URDF 自動生成 C++ 封閉解，MoveIt 直接載入 |
| 極速求解 | Pinocchio | `pinocchio::computeJointJacobians(model, data, q)` |
| 模擬器內建 | MuJoCo / PyBullet | `p.calculateInverseKinematics(robot, link, target_pos)` |

## 直覺理解

**類比：伸手拿杯子**。你腦中想的是「手到杯子那個位置」（task space），但你的大腦必須計算每個關節（肩、肘、腕）要轉幾度。而且同一個杯子位置，你的手肘可以朝上或朝下——這就是多解。手完全伸直時你會發現「沒辦法再往外推了」——這就是 workspace boundary 奇異。

**視覺比喻：Manipulability Ellipsoid**。在 MuJoCo 或 Isaac Sim 裡，把 $JJ^T$ 的特徵值畫成橢球：橢球越圓、末端在各方向都靈活；橢球退化成薄餅，代表某方向的可操控性趨近零——奇異。模擬器中拖動末端目標，觀察關節速度計量表：遠離奇異點時速度平穩，接近奇異時某個關節速度突然飆高。

**模擬器觀察建議**：
1. 在 MuJoCo 中載入 UR5，用 `mj_jac` 取 Jacobian，即時計算 condition number 並用 HUD 顯示
2. 把末端目標沿直線慢慢移向手臂完全伸直的構型，觀察 condition number 從 ~10 飆到 >1000
3. 比較純偽逆和 DLS 兩種模式下的關節速度曲線——偽逆在奇異附近爆掉，DLS 穩但末端軌跡微偏

## 實作連結

**三個典型工程場景**：

1. **ROS 2 + MoveIt 軌跡規劃**：MoveIt 的 `computeCartesianPath()` 在每個 waypoint 調用 IK。若路徑經過奇異構型，MoveIt 會回報 fraction < 1.0（未完成率）。實務上要檢查回傳的 fraction，必要時拆分路徑或加入中繼點繞開奇異。

2. **視覺伺服閉環**：相機偵測到目標位姿 → IK 解出 $q_d$ → PID 追蹤。若視覺更新率 30 Hz、控制迴圈 1 kHz，IK 必須在 1 ms 內出解。工業臂用 IKFast 解析解 (~10 μs)，非標臂用 TRAC-IK (~0.5 ms)。

3. **7-DoF 冗餘臂避障**：Franka Emika Panda 7 軸——第 7 個自由度不影響末端位姿，但可以用 null-space 把手肘推離桌面障礙物。在 Pinocchio 中用 `(I - J_pinv * J) * q0_dot` 做 null-space 投影。

**Code 骨架**（Python，速度級 IK + DLS）：

```python
import numpy as np

def ik_dls(J, x_err, lam=0.05):
    """Damped Least Squares 速度級 IK
    J:     (6, n) Jacobian
    x_err: (6,) 末端位姿誤差 [pos_err; ori_err]
    lam:   阻尼係數
    Returns: (n,) 關節速度增量
    """
    JJT = J @ J.T
    dq = J.T @ np.linalg.solve(JJT + lam**2 * np.eye(6), x_err)
    return dq

def ik_nullspace(J, x_err, q0_dot, lam=0.05):
    """含 null-space 次任務的冗餘 IK"""
    J_pinv = np.linalg.pinv(J)
    dq_main = J_pinv @ x_err
    dq_null = (np.eye(J.shape[1]) - J_pinv @ J) @ q0_dot
    return dq_main + dq_null
```

<details>
<summary>深入：完整 Python 實作（位置級 IK 迭代求解 + 自適應 DLS + joint limits）</summary>

```python
import numpy as np
import pinocchio as pin

def ik_iterative(model, data, frame_id, oMdes,
                 q_init, max_iter=200, tol=1e-4, lam_max=0.1, eps=0.04):
    """
    位置級 IK：從初始關節角 q_init 迭代求解目標位姿 oMdes。
    自適應 DLS：根據 Jacobian 最小奇異值動態調整阻尼。

    Parameters
    ----------
    model, data : Pinocchio model / data
    frame_id    : 末端 frame ID
    oMdes       : pin.SE3, 目標位姿
    q_init      : (n,) 初始關節角
    max_iter    : 最大迭代次數
    tol         : 收斂閾值 (position + orientation error norm)
    lam_max     : 最大阻尼係數
    eps         : 奇異值閾值，低於此值開始加阻尼
    """
    q = q_init.copy()
    for i in range(max_iter):
        # FK 更新
        pin.forwardKinematics(model, data, q)
        pin.updateFramePlacement(model, data, frame_id)
        oMcur = data.oMf[frame_id]

        # 6D 誤差（SE3 log map → twist）
        err_se3 = pin.log6(oMcur.actInv(oMdes))
        err = err_se3.vector  # (6,)

        if np.linalg.norm(err) < tol:
            print(f"IK 收斂：{i+1} 次迭代, 誤差 {np.linalg.norm(err):.2e}")
            return q, True

        # Jacobian（body frame）
        J = pin.computeFrameJacobian(model, data, q, frame_id,
                                      pin.LOCAL_WORLD_ALIGNED)

        # 自適應 DLS
        U, S, Vt = np.linalg.svd(J)
        sigma_min = S[-1] if len(S) > 0 else 0.0
        if sigma_min < eps:
            lam2 = (1 - (sigma_min / eps)**2) * lam_max**2
        else:
            lam2 = 0.0

        # 求解
        JJT = J @ J.T + lam2 * np.eye(6)
        dq = J.T @ np.linalg.solve(JJT, err)

        # 步長限制（防止大跳）
        dq_norm = np.linalg.norm(dq)
        if dq_norm > 0.5:
            dq *= 0.5 / dq_norm

        q += dq

        # Joint limits clamp（必須！否則可能解到不合法角度）
        q = np.clip(q, model.lowerPositionLimit, model.upperPositionLimit)

    print(f"IK 未收斂：{max_iter} 次迭代, 殘差 {np.linalg.norm(err):.2e}")
    return q, False


# ── 使用範例 ──
if __name__ == "__main__":
    model = pin.buildModelFromUrdf("panda.urdf")
    data = model.createData()
    frame_id = model.getFrameId("panda_hand")

    # 目標位姿
    oMdes = pin.SE3(
        pin.utils.rpyToMatrix(0, np.pi, 0),  # 朝下
        np.array([0.4, 0.0, 0.4])             # 前方 40cm, 高度 40cm
    )

    q_init = pin.neutral(model)
    q_sol, success = ik_iterative(model, data, frame_id, oMdes, q_init)

    if success:
        pin.forwardKinematics(model, data, q_sol)
        pin.updateFramePlacement(model, data, frame_id)
        print("解出的末端位姿:\n", data.oMf[frame_id])
```

**關鍵實作細節**：
- 用 `pin.log6` 計算 SE3 上的對數映射誤差，比分開算位置誤差和歐拉角誤差更穩定
- `np.clip` 做 joint limits clamp 是最低要求；進階做法是在 null-space 加 joint limit avoidance gradient
- 步長限制 `dq_norm > 0.5` 防止離目標遠時跳太大導致發散
- 自適應 DLS 需要做 SVD，計算量略大但 7-DoF 矩陣很小（6x7），不是瓶頸

</details>

## 常見誤解

1. **「IK 只有一個解」** — 6-DoF 手臂對同一目標位姿通常有**最多 8 組解**（解析解），數值法只找到離初值最近的那組。**避開**：需要全局最優時用解析解列出所有候選，再用 cost function（最小關節運動 / 遠離 joint limit / 遠離奇異）選最佳。

2. **「解析解永遠比數值解好」** — 解析解快且能列出全部解，但只適用滿足 Pieper 準則的構型（球形腕或三軸平行）。非標機械臂（蛇形、人形、並聯）沒有解析解可用。**避開**：先確認機構是否符合 Pieper 準則；不符就老實用 TRAC-IK 或 Pinocchio 數值求解。

3. **「走直線就對了，不需要管奇異點」** — Cartesian 直線插值是最常見的軌跡，但如果直線經過（或接近）奇異構型，Jacobian 會秩虧損，偽逆解的關節速度爆掉 → 馬達電流暴增 → 觸發過流保護或機械衝擊。**避開**：規劃前先沿路徑採樣檢查 Jacobian condition number；高於閾值（如 $\kappa > 100$）時改用 DLS 或 B-spline 繞道。

4. **「忘記 joint limits」** — 數值 IK 求出的解可能超出關節機械限位。直接發送會觸發驅動器保護或撞機械硬停。**避開**：每步迭代後都 clamp 到 `[q_min, q_max]`；或在 cost function 裡加 joint limit penalty。

## 練習題

<details>
<summary>Q1（中）：機械臂走 Cartesian 直線時，某些位置 MoveIt 回報 fraction < 1.0 無法完成規劃，你怎麼診斷和處理？</summary>

**完整推理鏈**：

1. **確認是 IK 失敗還是碰撞**：查 MoveIt 日誌，區分 "IK failed" 和 "collision detected"。若是 IK 失敗 → 進入步驟 2
2. **檢查可達性 (reachability)**：目標是否在 workspace 邊界之外？用 FK 從 joint limits 算 workspace 範圍，看目標 XYZ 是否合理
3. **檢查奇異距離**：沿規劃路徑每隔 1 cm 採樣，計算 Jacobian 的 $\det(JJ^T)$ 或 $\sigma_{\min}$。若某段 $\sigma_{\min} < 0.01$ → 路徑穿越奇異構型
4. **處理方案**：
   - 若是 workspace 邊界 → 調整目標位置或更換安裝位姿
   - 若是奇異穿越 → 三種選擇：(a) 加中繼 waypoint 繞開奇異、(b) 改用 DLS 容忍微量軌跡偏差、(c) 改為 joint-space 插值（放棄 Cartesian 直線）
5. **驗證**：重跑 `computeCartesianPath()`，確認 fraction = 1.0；肉眼在 Rviz 看軌跡合理

**面試官想聽到**：不是遇到 fraction < 1.0 就放棄，而是有系統地區分可達性問題 vs 奇異問題，並知道三種繞開策略的取捨。

</details>

<details>
<summary>Q2（中-難）：UR5 走直線焊接時馬達電流突然暴增、機械臂抖動，你如何即時診斷是奇異點問題並線上處理？</summary>

**完整推理鏈**：

1. **即時監控 SVD condition number**：在控制迴圈中，每個 tick 計算 $\kappa = \sigma_{\max} / \sigma_{\min}$，發現 $\kappa$ 從正常的 ~20 飆到 >500 → 確診經過內部奇異構型（UR5 常見：腕部第 4、6 軸接近共線）
2. **切換 DLS**：自適應 DLS — 當 $\sigma_{\min} < \epsilon$ 時動態增大 $\lambda$，犧牲末端追蹤精度（容許 ~1mm 偏差）換關節速度穩定
3. **離線根因修正**：焊接路徑是固定的 → 離線重新規劃，用 B-spline 在奇異附近微調軌跡（偏移幾 mm），讓整條路徑的 $\kappa < 100$
4. **防禦性設計**：在軌跡規劃階段加入 **singularity proximity check**——規劃完成後掃描每個 waypoint 的 condition number，超標就拒絕執行並提示重規劃

**面試官想聽到**：知道 UR5 的典型奇異構型（腕部共線）、懂用 SVD condition number 量化而不是靠直覺猜、能給出線上（DLS）和離線（軌跡修正）兩層解法。

</details>

<details>
<summary>Q3（難）：Franka Panda 7-DoF 在桌面環境做抓取，無窮組 IK 解，你怎麼選「最好的」那組？</summary>

**完整推理鏈**：

1. **理解冗餘**：7-DoF 比 6-DoF 多一個自由度 → 滿足 6D 末端位姿約束後，還剩 1 維 null-space 可以自由運動（手肘的旋轉角度）
2. **定義 cost function $H(q)$**（次任務目標），典型選擇：
   - **避障**：$H_1 = -\min(\text{dist}(q, \text{obstacles}))$，梯度把手肘推離桌面
   - **遠離 joint limits**：$H_2 = \sum_i \left(\frac{q_i - q_{i,\text{mid}}}{q_{i,\text{max}} - q_{i,\text{min}}}\right)^2$
   - **最大化 manipulability**：$H_3 = -\sqrt{\det(JJ^T)}$，讓操控橢球盡量圓
3. **Null-space 投影**：$\dot{q} = J^+\dot{x} + (I - J^+J)\,\nabla H$
   - 主任務：末端追蹤抓取軌跡
   - 次任務：gradient descent on $H$ 在 null-space 投影內優化手肘姿態
4. **權重調配**：多個次任務用加權組合 $\dot{q}_0 = w_1 \nabla H_1 + w_2 \nabla H_2 + w_3 \nabla H_3$，根據場景調權重（桌面擁擠 → 避障優先；精密操作 → manipulability 優先）

**面試官想聯繫到**：null-space 是冗餘臂的核心價值——不改變末端位姿的前提下優化「姿態品質」；具體實作用 $(I - J^+J)$ 投影，確保次任務不干擾主任務。

</details>

<details>
<summary>Q4（情境延伸）：IK 解出來的 $q$ 超出 joint limits，但目標位姿確實在 workspace 內，怎麼處理？</summary>

**完整推理鏈**：

1. **確認多解性**：6-DoF 通常有 8 組解（解析解），可能當前這組超限但其他組合法 → 先列出所有解析解，篩掉超限的
2. **若所有解都超限**：目標雖在幾何 workspace 內，但考慮 joint limits 後的**可達 workspace** 比幾何 workspace 小 → 調整目標位姿（例如改變 approach direction 的 roll 角度）
3. **數值法加約束**：用帶 joint limit constraint 的 SQP（Sequential Quadratic Programming）替代無約束偽逆，TRAC-IK 內建這個功能
4. **冗餘臂優勢**：7-DoF 臂可以用 null-space 把關節角度拉回限位中間區域，同樣末端位姿但關節角度完全不同

**面試官想聽到**：幾何 workspace ≠ 考慮 joint limits 後的可達 workspace；處理策略是先窮舉多解、再調整目標姿態、最後用帶約束的數值法。

</details>

## 面試角度

1. **解析 + 數值混合求解策略** — 這是區分「背過 IK 公式」和「真的在產線上解過 IK」的分水嶺。面試時帶出：「工業臂我會先用 IKFast 生成解析解取得所有候選，再用 cost function 篩最優；非標臂才改用 TRAC-IK 數值求解，初值給上一拍的 $q$ 保證連續性。」

2. **SVD condition number 自適應 DLS** — 展現你不只知道 DLS 公式，還能監控奇異距離、動態調 $\lambda$。面試時帶出：「我在控制迴圈裡每拍算 Jacobian 的 SVD，$\sigma_{\min}$ 低於閾值就漸增阻尼——這樣遠離奇異時精度不打折，穿越奇異時也不會爆。」

3. **Null-space 利用是冗餘臂的核心價值** — 面試 7-DoF 相關職位必問。帶出：「$(I - J^+J)$ 投影讓我在不干擾末端軌跡的前提下做避障、遠離 joint limit、最大化 manipulability。這就是為什麼 Franka 比 UR5 多一軸——不只是多一個自由度，而是多了一整個次任務優化空間。」

4. **IK 失敗的系統化除錯** — 業界 90% 的 IK 問題不是算法錯，而是 workspace 邊界、joint limits、TCP 偏移、或 URDF 定義不一致。面試時帶出：「遇到 IK 解不出，我的 checklist 是：(1) 目標在 workspace 內嗎？(2) joint limits 有考慮嗎？(3) TCP 對嗎？(4) 是不是卡在奇異？——按順序排除，通常第一輪就能定位問題。」

## 延伸閱讀

- **Lynch & Park,《Modern Robotics》Ch6 (IK) + Ch5 (Velocity Kinematics)** — 用 screw theory 統一處理 FK/IK/Jacobian，比 DH 方法更優雅；MIT 開放教材，有完整 Python 程式庫
- **《具身智能算法工程師 面試題》Ch2 IK 求解、Ch3 奇異點分析** — 直接對照面試考點，練情境題的最佳來源
- **TRAC-IK 論文 (Beeson & Ames, 2015)** — 比 KDL 穩定 10 倍的 IK solver，同時跑 KDL Newton-Raphson 和 SQP 兩條線程取先收斂者；ROS 2 直接可用
- **IKFast (OpenRAVE)** — 從 URDF 自動生成 C++ 解析解的工具；了解它怎麼把 Pieper 準則自動化的背後原理
- **Pinocchio 官方 IK 範例** — 看 C++ 高效 Jacobian 計算和 null-space 投影的實作；MPC 場景首選
- **Manipulability Ellipsoid 視覺化** — 在 MuJoCo 或 Rviz 中畫 $JJ^T$ 的特徵橢球，直覺理解構型品質；Robotics Toolbox for Python 有現成 API
