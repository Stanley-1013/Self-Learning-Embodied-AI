# Audit Report: Ch15-18

**Summary**: Ch15 MPC is technically sound overall; spotted two notation/scope issues on offset-free model and one minor CBF HOCBF omission. Ch16 VS contains a real sign error in the PBVS control law translation term and a scope gap on HOCBF-style 1/Z tightening. Ch17 SLAM is solid; minor concerns in ICP formulation wording and Q2 claim about 2D LiDAR axial degeneracy fix. Ch18 RL basics contains one notable concern around the `not terminated` mask usage (conflates terminated vs truncated for bootstrapping) and a minor phrasing issue on Policy Improvement Theorem.

## Ch15 MPC

### Findings

- **[LOW]** `content/embodied-robotics/15-model-predictive-control.md:499` — Offset-free output equation `y_k = C x_k + d_k` models output disturbance, but the state equation is written `x_{k+1} = A x_k + B u_k + B_d d_k` (input/state disturbance form). Standard offset-free literature uses either the pure state-disturbance model (no `d_k` in y) or the output-disturbance model (no `B_d d_k` in x); mixing both over-parameterizes and is usually unobservable. — Fix: Use one canonical form, e.g. `x_{k+1} = Ax_k + Bu_k + B_d d_k`, `y_k = Cx_k` (state-disturbance) and note the alternative exists. — Confidence: high.

- **[LOW]** `content/embodied-robotics/15-model-predictive-control.md:86` — Claim "NMPC 每個 SQP 迭代需算 Jacobian（$O(n^4)$）". Dense Newton/KKT factorization is $O(n^3)$ per iteration; Jacobian assembly is at worst $O(n^2)$ for articulated bodies via RNEA/CRBA ($O(n)$ or $O(n^2)$). The $O(n^4)$ figure is not standard. — Fix: Replace with "KKT factorization $O(n^3)$ per iteration; Hessian/Jacobian assembly dominated by articulated-body dynamics $O(n^2)$-$O(n^3)$". — Confidence: high.

- **[LOW]** `content/embodied-robotics/15-model-predictive-control.md:240` — CBF condition stated as `$\dot{h}(x) \ge -\alpha \cdot h(x)$` with `$\alpha > 0$` is correct only for Relative Degree 1. For position-level `h(x)` on second-order systems (e.g. acceleration-controlled manipulator, quadrotor), the first-order CBF constraint does NOT contain `u` and is vacuous — HOCBF (Xiao & Belta 2019) is required. The text does not flag this. — Fix: Add one-liner "若 `h` 的相對階 > 1（例：位置約束 + 加速度控制輸入），需用 HOCBF 對更高階導數遞迴套 $\alpha$ 函數". — Confidence: high.

## Ch16 Visual Servoing

### Findings

- **[HIGH]** `content/embodied-robotics/16-visual-servoing.md:80` — PBVS control law written as `v_c = -λ [ ^c t - ^{c*} t ; θu ]`. The standard Chaumette PBVS formulation uses the translation error expressed in the **current camera frame relative to the target** (i.e. `^{c*}t_c` or equivalently `-R^T t` after SE(3) inverse), not raw `^c t - ^{c*} t` between two separate translation vectors. Subtracting two world/base translation vectors yields a sign/frame inconsistency with the rotational axis-angle term that follows. — Fix: Write `v_c = -λ [ t ; θu ]` where `[R, t] = ^{c*}T_c = (^cT_{c*})^{-1}` (i.e. current pose w.r.t. target), matching Chaumette & Hutchinson 2006 Eq. (22). — Confidence: high.

- **[MED]** `content/embodied-robotics/16-visual-servoing.md:162` — Interaction matrix row 2, column 4 shows `1+y^2` for `ω_x` coefficient, row 1 column 5 shows `-(1+x^2)`. These are correct individually, but the text earlier states "旋轉列（後三列）與 $Z$ 無關". That is true for the rotational block, but readers may conflate this with "rotation is decoupled from translation" — actually, rotational motion still induces image-point motion that depends on `(x, y)` pixel position, which itself has depth implications when reprojected. Not a formula error but the statement "與 Z 無關" as written could mislead on 2.5D decoupling discussion below. — Fix: Rephrase "旋轉列的**係數**不顯含 Z（純像素位置多項式）" to avoid implying full physical Z-independence. — Confidence: medium.

- **[LOW]** `content/embodied-robotics/16-visual-servoing.md:163` — Claim "至少需要 4 個非共面點讓 $L_s$ 滿秩（6）" mixes two different conditions: (a) 4 non-collinear coplanar points are sufficient for `L_s` to have rank 6 (the classic Chaumette result), and (b) P3P has 4-fold ambiguity resolved by a 4th point. The text says "非共面" (non-coplanar) but 4 coplanar non-collinear points are actually the minimum and most common IBVS setup. — Fix: Change "非共面" to "非共線" (non-collinear) to align with the standard result; non-coplanar is not required. — Confidence: high.

## Ch17 SLAM

### Findings

- **[LOW]** `content/embodied-robotics/17-slam-and-mapping.md:71` — ICP formulation `min_{R,t} Σ ||R p_k + t − q_k||^2` is labeled "LiDAR scan matching 核心". This is point-to-point ICP; modern LiDAR SLAM (LOAM/LIO-SAM/FAST-LIO2) uses point-to-plane or point-to-line metrics which converge faster and are more robust on structured environments. The chapter elsewhere mentions NDT but not point-to-plane. — Fix: Add one sentence "實務 LiDAR SLAM 多用 point-to-plane 變體（殘差投影到局部平面法向），對平面豐富場景收斂更快". — Confidence: medium.

- **[LOW]** `content/embodied-robotics/17-slam-and-mapping.md:362-363` — Q2 claims "加 IMU 緊耦合 (LIO-SAM/FAST-LIO2), IMU 加速度計能補償軸向位移估計, 在 LiDAR 退化方向提供約束". IMU accelerometer after double-integration has bias drift and cannot independently constrain long-corridor axial position (it's why IMU alone drifts in seconds). LIO-SAM helps corridor-degeneracy mainly by (i) preintegration giving short-term axial motion prior between scans and (ii) providing attitude (yaw) from gyro to prevent rotational drift — NOT because accelerometer gives absolute axial position. — Fix: Reword to "IMU 預積分在短時段內提供軸向位移先驗 + 陀螺儀防偏航漂移; 但 IMU 本身不能獨立解除長期軸向退化, 仍需 loop closure/UWB/人工路標做絕對約束". — Confidence: high.

## Ch18 RL Basics

### Findings

- **[HIGH]** `content/embodied-robotics/18-rl-mdp-basics.md:221, 263, 305` — Q-learning/SARSA update code uses `td_target = reward + gamma * np.max(Q[next_state]) * (not terminated)`. This conflates `terminated` (true episode end, bootstrap should be 0) with `truncated` (time-limit cutoff, bootstrap should still happen because the underlying MDP continues). Using `(not terminated)` here is actually correct for termination, but the code also uses `done = terminated or truncated` to exit the loop — consistent enough, yet many readers will copy this and mis-handle `truncated` in environments like CartPole where both flags matter for bootstrapping correctness. — Fix: Add a one-line comment: "注意：bootstrap mask 只用 `terminated`, 不用 `truncated` — time-limit 截斷時 MDP 仍在繼續, 要用 V(s') 繼續 bootstrap". — Confidence: high.

- **[MED]** `content/embodied-robotics/18-rl-mdp-basics.md:94` — Policy Improvement Theorem stated as `Q^π(s, π'(s)) ≥ V^π(s), ∀s ⟹ V^{π'}(s) ≥ V^π(s), ∀s`. This is correct for deterministic `π'`, but the original Sutton & Barto theorem is stated for policies that may be stochastic with the condition `Σ_a π'(a|s) Q^π(s,a) ≥ V^π(s)`. The deterministic form is a corollary. Minor — not wrong, but the quantifier wording "∀s" on the implication RHS is exactly right. — Fix: None strictly needed; optionally mention the stochastic generalization in a note. — Confidence: low (not an error, scope note).

- **[LOW]** `content/embodied-robotics/18-rl-mdp-basics.md:444` — "γ = 0.99 的有效視野約 100 步 (0.99^100 ≈ 0.37)". The "effective horizon" definition via `1/(1-γ)` gives `1/0.01 = 100` steps; the `0.99^100 ≈ 0.37` justification is a weaker heuristic (that's e^-1, the 1/e decay point, not the full effective horizon). Both are defensible but mixing them can confuse. — Fix: Either cite `1/(1-γ)` as the standard definition, or explicitly say "at step 100, discount drops to ~37% (1/e), often used as effective horizon". — Confidence: medium.
