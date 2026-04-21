# Round 4: Code-vs-Prose Consistency

## Summary

- Chapters scanned: 22
- Code blocks scanned: ~56 substantive blocks (excluding YAML config, shell commands, pure declarations)
- Mismatches found: 3 (HIGH: 0, MED: 2, LOW: 1)

Overall signal: Chapters 1-9, 11, 14-22 pass cleanly. Ch10 and Ch12 have meaningful demo-code/prose inconsistencies that a careful reader would catch. Ch13 has a subtle dimensional issue in the Back-calculation formula.

## Ch01 - C++ Memory Optimization
### Block 1 (line 129-134): ROS 2 zero-copy publish skeleton
- Language: cpp
- Prose claim: "unique_ptr + std::move 轉移所有權，零拷貝"
- Code does: `make_unique<PointCloud2>()` + `publisher_->publish(std::move(msg))`
- Match: YES

### Block 2 (lines 74-91, details): shared_ptr / FixedPool concept
- Language: cpp
- Prose claim: Free-list O(1) pop/push
- Code does: `free_list_ = block->next` (pop), `block->next = free_list_; free_list_ = block` (push)
- Match: YES

### Block 3 (lines 139-189, details): Full FixedPool implementation
- Language: cpp
- Prose claim: "預先配置 + free list pop/push = O(1)、無鎖"
- Code does: ctor builds free-list over arena, allocate/deallocate are single-pointer ops
- Match: YES

## Ch02 - C++ Concurrency & Sync
### Block 1 (lines 66-69): cv.wait predicate example
- Prose claim: "wait 的第二個參數是 predicate lambda，內部展開為 while (!pred()) { cv.wait(lk); }"
- Code does: `cv.wait(lk, [&]{ return data_ready; })`
- Match: YES

### Block 2 (lines 117-144): SPSC queue skeleton
- Prose claim: "單生產者單消費者 SPSC ... tail store(release), head load(acquire)"
- Code does: correct acquire/release ordering, tail for producer, head for consumer
- Match: YES

### Block 3 (lines 153-176): ThreadSafeQueue with mutex+CV
- Prose claim: "合併 front+pop 為單步"
- Code does: `wait_and_pop` holds lock across `front()` + `pop()`, returns shared_ptr
- Match: YES

### Block 4 (lines 241-262): MultiExclusive CallbackGroup skeleton
- Prose claim: "同 group 不平行"
- Code does: both sub + timer use same group (`group` passed to both)
- Match: YES

### Block 5 (lines 267-335): DoubleBuffer full impl
- Prose claim: "感知寫 back、規劃讀 front, atomic pointer swap"
- Code does: `front_idx_.store(1 - old_front, release)` after writing back buffer; reader uses `acquire` load
- Match: YES

## Ch03 - C++ Memory Model & Atomics
### Block 1 (lines 63-65): CAS signatures
- Prose: weak vs strong, spurious failure
- Code: standard `compare_exchange_weak/strong` signatures
- Match: YES

### Block 2 (lines 94-103): x86 memory ordering comments
- Prose: "seq_cst store = MOV + MFENCE; acquire/release loads free"
- Code comment: matches exactly
- Match: YES

### Block 3 (lines 111-120): ARMv8 memory ordering comments
- Prose: "seq_cst load/store on ARMv8 = same LDAR/STLR as acquire/release"
- Code comment: "seq_cst store = STLR"
- Match: YES

### Block 4 (lines 128-134, 137-142): atomic_thread_fence example
- Prose: "relaxed writes + release fence, relaxed flag"
- Code: matches exactly
- Match: YES

### Block 5 (lines 160-170): Acquire-release pattern
- Prose: "release-store synchronizes-with acquire-load"
- Code: data=42; flag.store(true, release); consumer spins on acquire then assert(data==42)
- Match: YES

### Block 6 (lines 204-238): LockFreeStack
- Prose: "CAS weak with release on success, relaxed on failure"
- Code: `compare_exchange_weak(..., release, relaxed)` in push; `acq_rel / acquire` in pop
- Match: YES

### Block 7 (lines 244-275): TaggedLockFreeStack
- Prose: "每次 CAS 遞增 tag"
- Code: `new_head = {node, old_head.tag + 1}`
- Match: YES

### Block 8 (lines 326-353): SPSC Ring Buffer
- Prose: "write_idx store with release, read_idx load with acquire"
- Code: matches; own-index uses relaxed, peer-index uses acquire, commit uses release
- Match: YES

## Ch04 - ROS 2 Basic Communication
All 8 code blocks (pub/sub/client skeletons, component node, launch) consistent with prose. Component launch correctly sets `use_intra_process_comms: True` — matches prose claim.

## Ch05 - ROS 2 TF & Tools
All 8 code blocks consistent. Important convention check:
- Prose (line 36, 73): `header.frame_id=parent, child_frame_id=child`, $T^{target}_{source}$ maps source point to target frame
- Code (line 65-71, 87-93): `lookup_transform(target_frame='base_link', source_frame='camera_link', ...)` with publish `header.frame_id='odom', child_frame_id='base_link'`
- Match: YES — convention strictly applied.

## Ch06 - ROS 2 Advanced Features
All 4 code blocks (MultiThreaded + CallbackGroup C++, QoSDemo C++, Python QoS, MultiGroup Python) consistent with the QoS table and Executor description.

## Ch07 - Forward Kinematics & DH
### Block 1 (lines 119-132): KDL FK skeleton
- Prose: "讀 URDF → 建 KDL::Tree → 選 chain → 算 FK"
- Code: exactly those four steps
- Match: YES

### Block 2 (lines 137-181): Full Python FK + Standard DH
- Prose formula: $\text{Rot}_Z(\theta) \cdot \text{Trans}_Z(d) \cdot \text{Trans}_X(a) \cdot \text{Rot}_X(\alpha)$
- Code `dh_transform_standard`: produces the 4x4 matrix matching the "full展開式" given in the detail block exactly (row-by-row)
- Match: YES

### Block 3 (lines 186-191): Pinocchio usage
- Prose: `pinocchio::forwardKinematics(model, data, q)`
- Code: same
- Match: YES

## Ch08 - Inverse Kinematics
### Block 1 (lines 184-204): DLS + nullspace Python
- Prose formula: $\dot{q} = J^T(JJ^T + \lambda^2 I)^{-1} \dot{x}$
- Code: `dq = J.T @ np.linalg.solve(JJT + lam**2 * np.eye(6), x_err)` where `JJT = J @ J.T`
- Match: YES (exact)
- Prose formula (nullspace): $\dot{q} = J^+\dot{x} + (I - J^+J)\dot{q}_0$
- Code `ik_nullspace`: `J_pinv @ x_err + (I - J_pinv @ J) @ q0_dot`
- Match: YES

### Block 2 (lines 210-294): Full iterative IK w/ adaptive DLS
- Prose formula for adaptive DLS: $\lambda^2 = (1 - (\sigma_{\min}/\epsilon)^2)\lambda_{\max}^2$ when $\sigma_{\min} < \epsilon$, 0 otherwise
- Code: `lam2 = (1 - (sigma_min / eps)**2) * lam_max**2` under the same condition
- Match: YES

## Ch09 - Robot Dynamics
### Block 1 (lines 318-344): Pinocchio C++ skeleton
- Prose: "RNEA O(n), ABA O(n), CRBA O(n²), computeRNEADerivatives for NMPC"
- Code: `rnea`, `aba`, `crba`, `computeRNEADerivatives`, `ccrba` all called correctly
- Match: YES

### Block 2 (lines 349-423): Full Python dynamics + CTC + Passivity
- Prose formula for CTC: `a_cmd = a_des + Kp*e + Kd*de; tau = rnea(q, v, a_cmd)`
- Code: matches exactly
- Prose: Skew-symmetry verification `Ṁ - 2C`
- Code `verify_skew_symmetry`: `S = M_dot - 2*C; asym_error = ||S + S.T||`
- Match: YES

## Ch10 - Basic Path Planning
### Block 1 (lines 294-313): A* pseudocode
- Prose + code match
- Match: YES

### Block 2 (lines 344-358): Theta* update_vertex
- Prose: "若祖父能直視 s_prime，跳過 u 直接連祖父"
- Code: `if line_of_sight(u.parent, s_prime): ... new_cost = u.parent.g + euclidean(...)`
- Match: YES

### Block 3 (lines 390-416): RRT* rewiring
- Code matches prose description
- Match: YES

### Block 4 (lines 463-473): OMPL StateValidityChecker (broad + narrow phase)
- Match: YES

### Block 5 (lines 487-501): Hybrid A* expand
- Prose: "方向盤左/正/右三種控制... 同時產生倒車選項"
- Code: `for steer in [-max_steer, 0, +max_steer]` generating forward + reverse (`new_state_reverse = ...`)
- Match: YES (Note: later block at lines 860-874 structures this slightly differently using `for direction in [+1, -1]` — mathematically equivalent; both forms acceptable)

### Block 6 (lines 526-556): MPPI full flow
- Prose formulas for importance weighting: `weights ∝ exp(-(S - S_min) / lambda)`, then weighted average
- Code: matches exactly
- Match: YES

### Block 7 (lines 573-718): **A\* + RRT\* + APF three-in-one demo**
- Language: python
- Prose claim (lines 321-326): "Manhattan distance: 4-connected grid 的 admissible h"; "8-connected grid 搭配 √2 斜向代價時的正確選擇" is **Octile**. The prose explicitly warns: `Chebyshev distance: 8-connected grid，僅在斜向代價 = 直向代價 = 1 時 admissible`.
- Code does: 8-connected movement (`directions` includes all 8 including diagonals), diagonal cost `1.414` (i.e., √2), yet uses `manhattan(nb, goal)` as the heuristic (line 615).
- Match: **NO (PARTIAL)**
- Severity: **MED**
- Issue: With 8-connected grid and √2 diagonal cost, Manhattan heuristic **overestimates** the true cost (Manhattan counts a diagonal step as 2, but true cost is √2 ≈ 1.414). This makes the heuristic **non-admissible** → A* is no longer guaranteed to find an optimal path. The demo code directly contradicts the "選擇規則" decision table the chapter itself provides.
- Fix: Change line 615 to use Octile distance: `h = max(|dx|,|dy|) + (sqrt(2)-1)*min(|dx|,|dy|)`. Alternatively, restrict `directions` to the 4 cardinal moves to match the Manhattan heuristic. Easiest fix in code:
  ```python
  def octile(a, b):
      dx, dy = abs(a[0]-b[0]), abs(a[1]-b[1])
      return max(dx, dy) + (1.414 - 1.0) * min(dx, dy)
  # ... use octile(nb, goal) in the heapq.heappush
  ```

### Block 8 (lines 725-761): Diffuser PyTorch skeleton
- Match: YES (schematic)

### Block 9 (lines 811-832): Nav2 YAML costmap — passes (config, not algorithm)

### Block 10 (lines 836-856): OMPL BIT* — consistent

### Block 11 (lines 860-874): Hybrid A* expand (ackermann)
- Match: YES

## Ch11 - Trajectory Optimization
### Blocks at lines 143, 186, 270, 360, 397, 468, 623, 802, 834, 882, 930
All 11 code blocks consistent with prose:
- Quintic polynomial solve matches 6x6 A·x=b boundary-condition formulation
- Min-snap Q matrix builder matches $\int p^{(4)2} dt$ factorial formula (i,j from 4)
- TOPP-RA Drake API matches prose description
- CasADi Multiple Shooting + RK4: `X[:,k+1] == rk4(X[:,k], U[:,k], dt)` gap-closing
- iLQR Riccati backward/forward pass matches prose equations verbatim (K_ff/K notation consistent with `k = -Q_uu^{-1} Q_u, K = -Q_uu^{-1} Q_ux`)
- Match: YES across all

## Ch12 - Dynamic Obstacle Avoidance
### Block 1 (lines 175-179): Nav2 DWB C++ scoring
- Language: cpp
- Prose claim (lines 155-159): "G = α·heading + β·dist + γ·velocity 取最大值 ... dist: 越大越好; velocity: 鼓勵快速移動"
- Code does: `score = alpha * headingDiff + beta * (1.0/minDist) + gamma * (max_v - velocity)`
- Match: **PARTIAL**
- Severity: **MED**
- Issue: The C++ snippet builds a **cost** (to be minimized): `headingDiff` is positive when misaligned, `1/minDist` grows when close to obstacle (= danger), `max_v - v` grows when slow. This contradicts the immediately preceding prose formula which explicitly says "取最大值" with `dist: 越大越好` and `velocity: 鼓勵快速移動`. The downstream Python skeleton at lines 763-788 (`DWAPlanner`) uses `best_score > -inf` and picks max — matching prose but contradicting this C++ block.
- Fix: Either (a) rename the C++ variable to `cost` and clarify the snippet shows an alternative min-cost formulation, or (b) flip the sign convention: `score = -alpha*headingDiff + beta*minDist + gamma*velocity` to match the prose "maximize" formulation. Option (a) is more honest because Nav2 DWB internally does use cost-minimization (`nav2_dwb_controller` scores are penalties), so the prose should instead read "取最小值" and describe the terms as penalties. Recommended fix: update prose in lines 155-159 to say `score = α·headingPenalty + β·collisionPenalty + γ·slownessPenalty 取最小值`, which aligns with actual Nav2 implementation.

### Block 2 (lines 236-242): ORCA half-plane construction
- Prose: "v_A - (v_A^opt + u/2) · n ≥ 0 ... n 取作 u 的單位向量"
- Code: `orca.point = velocity_ + 0.5f * u; orca.direction = Vector2(-n.y(), n.x())`
- Match: YES — the half-plane's boundary point is `velocity + u/2`, and `direction` is the perpendicular to `n` (tangent direction for the line representation). Consistent with prose.

### Block 3 (lines 283-289): Chance-Constrained MPC CasADi
- Prose: `||p_ego - μ_obs||² ≥ (d_safe + 3σ)²`
- Code: `dist_sq >= safe_margin**2` where `safe_margin = robot_radius + obs_radius + 3.0 * obs_sigma`
- Match: YES

### Block 4 (lines 306-322): MPPI GPU architecture (diagram, not executable) — consistent

### Block 5 (lines 340-361): Nav2 MPPI YAML — matches prose

### Block 6 (lines 427-436): CBF-QP cvxpy
- Prose: `min ||u - u_nom||²  s.t. L_f h + L_g h · u + α·h ≥ 0`
- Code: `objective = Minimize(0.5 * sum_squares(u - u_nom)); cbf_constraint = [Lf_h + Lg_h @ u + alpha * h_val >= 0]`
- Match: YES

### Block 7 (lines 484-493): SARL Attention
- Prose: Self-Attention scoring + softmax pooling
- Code: matches
- Match: YES

### Block 8 (lines 543-553): Flight Corridor QP C++
- Prose: "A_j · P_i ≤ b_j"
- Code: `solver.addConstraint(A.row(j), control_points[i], b(j))`
- Match: YES

### Block 9 (lines 594-599): Vision MPC (ZoeDepth) — schematic, consistent

### Block 10 (lines 695-710): Joint-torque nullspace avoidance Franka
- Prose (line 682): "扭矩空間投影 $(I - J^T J^{+T})$ 用於 τ" (with explicit warning about transpose vs velocity nullspace)
- Code: `N = I - J.transpose() * J_pinv.transpose(); tau_null = N * tau_avoid`
- Match: YES — prose explicitly distinguishes velocity vs torque nullspace, code implements the correct torque form.

### Block 11 (lines 763-788): DWAPlanner Python — consistent with prose (maximize score)
### Block 12 (lines 792-815): HierarchicalAvoidance three-layer — consistent

## Ch13 - PID Control & Tuning
### Block 1 (lines 71-76): Conditional integration pseudocode — consistent
### Block 2 (lines 569-601): C++ PID with conditional anti-windup, D-on-measurement, LPF
- Prose claim: D-on-measurement eliminates setpoint kick; LPF suppresses noise
- Code: `d_raw = -kd * (measurement - prev_measurement) / dt` (note the minus sign since derivative of error = -derivative of measurement when setpoint constant); then LPF
- Match: YES

### Block 3 (lines 606-714): Python IncrementalPID with Back-calculation
- Language: python
- Prose formula (line 85-87): $\dot{I}(t) = K_i e(t) + \frac{1}{T_t}(u_{sat} - u_{raw})$
- Code (lines 641, 656): `i_term = self.ki * self.integral` and `self.integral += (error + (u_sat - u_raw) / self.tracking_tc) * dt`
- Match: **PARTIAL**
- Severity: **LOW**
- Issue: The code stores `integral = ∫ e dt` (no Ki scaling) and computes `i_term = Ki * integral` separately. The back-calc update adds `(u_sat - u_raw) / T_t * dt` directly to `integral` — this means the **effective** correction to the I contribution `Ki·integral` is `Ki · (u_sat - u_raw) / T_t`, i.e., the effective tracking time constant is `T_t / Ki`, not the `T_t` stated in the prose formula. To exactly match the textbook formula $\dot{I} = K_i e + (u_{sat} - u_{raw})/T_t$ (where $I$ is the I-contribution itself, not the error integral), the correction term should divide by `Ki * tracking_tc`, or the code should track the I-contribution directly instead of the error integral.
- Fix: Either change the storage convention so `integral` holds the I-contribution (then `i_term = integral` and the update uses `integral += (Ki*error + (u_sat-u_raw)/T_t) * dt`), or keep current storage and change update to `self.integral += (error + (u_sat - u_raw) / (self.ki * self.tracking_tc)) * dt`. In practice this only shifts the effective T_t by a factor of Ki so the loop still works; but the code does not literally implement the formula in the prose. Minor issue — readers copying the code should be aware.

## Ch14 - Force & Impedance Control
### Block 1 (lines 296-304): Adjoint construction + K_tcp
- Prose (lines 272, 275): wrench transforms as $W_B = \mathrm{Ad}_{T_{AB}}^T \cdot W_A$; stiffness congruence $K_{tcp} = \mathrm{Ad}_T^T K_{flange} \mathrm{Ad}_T$ is robust across convention choices (bilinear)
- Code: `Ad_T = [R, 0; p̂R, R]` (body-frame twist adjoint), `K_tcp = Ad_T^T * K_flange * Ad_T`
- Match: YES — already verified in Round 3 as consistent. Minor stylistic note: comment at line 303 says `W = Ad_T^T · W` but variable is `K`; this is a comment referencing the general wrench-transformation law, not a contradiction with the K-congruence actually being computed.

### Block 2 (lines 348-384): Impedance controller update loop — matches prose pipeline (pose error → K·e + D·ė → J^T·F → gravity comp → null-space → singular fallback)
### Block 3 (lines 389-491): Admittance + F/T calibration + gravity compensation Python — matches prose
### Block 4 (lines 498-530): Decision tree text block (not code, ascii diagram)
### Block 5 (lines 582-593): VSA SEA dynamics snippet — consistent

## Ch15 - Model Predictive Control
### Block 1 (lines 217-221): acados soft constraint C pseudocode — matches prose (slack penalty)
### Block 2 (lines 364-370): GP chance constraint — matches 3σ tightening formula
### Block 3 (lines 413-421): Drake MPCC relaxation — `phi * lam <= epsilon` matches prose
### Block 4 (lines 486-490): manif SE3 log map — matches prose formula `e_R = log(R_target^T R_current)^∨`
### Block 5 (lines 535-543): Event-triggered MPC — `error > threshold` matches prose formula
### Block 6 (lines 587-641): ROS 2 MPC node with 3-layer fallback + CBF filter — matches prose
### Block 7 (lines 652-743): Full CasADi Multiple Shooting NMPC with slack + CBF + chance constraint
- Prose: "h_dot + α h ≥ 0" CBF constraint
- Code: `opti.subject_to(h_dot + alpha * h_k >= 0)` where `alpha = 5.0`
- Match: YES
### Block 8 (lines 755-779): acados C++ wrapper — consistent

## Ch16 - Visual Servoing
### Block 1 (lines 219-225): OpenCV calibrateHandEye — matches prose `A_i X = X B_i`
### Block 2 (lines 339-369): IBVSController with interaction matrix
- Prose formula (lines 55-62): $L_s = \begin{bmatrix} -1/Z & 0 & x/Z & xy & -(1+x^2) & y \\ 0 & -1/Z & y/Z & 1+y^2 & -xy & -x \end{bmatrix}$
- Code line 353-354: `[[-1/Z, 0, x/Z, x*y, -(1+x*x), y], [0, -1/Z, y/Z, (1+y*y), -x*y, -x]]`
- Match: YES (exact row-by-row match)
- Prose control law: `v_c = -λ L_s^+ (s - s*)`
- Code: `v_c = -self.lambda_gain * Ls_pinv @ error` where `error = s_cur_n - s_tgt_n`
- Match: YES
### Block 3 (lines 383-441): DynamicIBVS with KF + PTP — feedforward formula `v_ff = Ls_pinv @ s_dot_target` matches prose `v_c = -λ L_s^+(s-s*) + L_s^+ \hat{\dot{s}}_target`
### Block 4 (lines 490-511): Fallback C++ — matches prose fallback layer description
### Block 5 (lines 589-606): VLA hybrid — schematic, consistent
### Block 6 (lines 661-684): Virtual camera C++ `H = K R_tilt K^{-1}` matches prose formula
### Block 7 (lines 737-752): Multi-rate control Python — consistent

## Ch17 - SLAM & Mapping
Code blocks are launch files and YAML config — consistent with prose.

## Ch18 - RL MDP Basics
### Block 1 (lines 201-228): Tabular Q-Learning
- Prose claim (line 221-223 warning comment inside code): "bootstrap mask 只用 terminated，不可用 truncated"
- Code: `td_target = reward + gamma * np.max(Q[next_state]) * (not terminated)` — uses `terminated`, not `done`
- Match: YES — explicit and correct

### Block 2 (lines 235-318): Full Q-Learning + SARSA — uses `terminated` mask consistently
### Block 3 (lines 322-433): DQN PyTorch — replay buffer stores `float(terminated)` (line 404), target Q masks with `(1 - t)` where `t` is terminated mask; this matches prose warning
- Match: YES

## Ch19 - DRL PPO/SAC/DDPG
### Block 1 (lines 105-134): Evolution tree diagram (ASCII) — consistent with prose
### Block 2 (lines 210-234): SB3 SAC training skeleton
- Prose: "α 自動調節 ← SAC 核心特色"
- Code: `ent_coef="auto"`
- Match: YES
### Block 3 (lines 239-351): Isaac Gym PPO — consistent

## Ch20 - Imitation Learning & DAgger
### Block 1 (lines 242-270): BC skeleton — consistent
### Block 2 (lines 275-359): Full DAgger trainer
- Prose claim (line 362 note inside code): "dagger_iteration 中用學生動作推進環境，但記錄專家動作"
- Code (line 327-331): `student_action = self.policy.get_action(obs); expert_action = self.expert.get_action(obs); self.dataset['act'].append(expert_action); obs, ... = self.env.step(student_action)`
- Match: YES — the canonical DAgger invariant (step with student, label with expert) is correctly implemented.

## Ch21 - Sim-to-Real Transfer
### Block 1 (lines 133-257): Teacher-Student pipeline
- Prose: "Teacher 能存取 sim 特權資訊; Student 只用 proprioception + LSTM 推斷物理參數"
- Code: `TeacherPolicy.forward(obs, privileged)` concatenates both; `StudentPolicy` uses LSTM on `obs_history` only
- Match: YES
### Block 2 (lines 291-312, 317-401): ADR outer loop + domain randomization — schematic, consistent

## Ch22 - Multimodal LLM Robotics
### Block 1 (lines 188-250): Octo fine-tuning pseudocode — schematic
### Block 2 (lines 290-331): VLMPlanner / LowLevelController / HierarchicalAgent — pseudocode with `pass`, consistent with "VLM 1 Hz 大腦 + MPC 100+ Hz 小腦" framing in prose

## Overall Assessment

22 chapters, ~56 non-trivial code blocks audited. Code and prose are tightly aligned in the overwhelming majority of cases. The three issues found are:

1. **Ch10 (MED)** — A* demo with 8-connected diagonal movement using Manhattan heuristic violates admissibility; the chapter's own decision table warns against exactly this. Fix: switch heuristic to Octile, or restrict directions to 4-connected.

2. **Ch12 (MED)** — Nav2 DWB C++ scoring snippet computes a **cost** (minimize) but the surrounding prose formula explicitly says "取最大值" (maximize). Two self-inconsistent formulations coexist in the same section. Fix: align the prose with Nav2's actual minimize-cost convention (the C++ snippet is correct for Nav2, the prose text is the bug).

3. **Ch13 (LOW)** — Back-calculation update in Python code stores `integral = ∫ e dt` and applies `(u_sat - u_raw)/T_t · dt` directly to it, which effectively scales the tracking time constant by `1/Ki` relative to the textbook formula $\dot{I} = K_i e + (u_{sat}-u_{raw})/T_t$. Functionally equivalent after retuning `T_t`, but the code does not literally implement the formula printed in the prose.

All three are in demo/illustrative blocks, not in the conceptual explanations. No mismatches in the formula/API tables or in the main C++/Python production-style snippets (e.g., Pinocchio, CasADi NMPC, Adjoint, Interaction Matrix, DAgger, Teacher-Student).
