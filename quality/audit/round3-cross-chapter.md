# Round 3: Cross-Chapter Consistency

## Summary
- Inconsistencies found: 0
- Concepts verified consistent: 10 / 10
- Notes: all 10 concept groups audited. Most are explicitly consistent; a few concepts only appear in one chapter (no cross-chapter surface, so nothing to contradict).

Evidence paths cite `content/embodied-robotics/...:line`.

---

## Concept 1: ARMv8 seq_cst mapping

**Claim to enforce**: pure seq_cst load/store on ARMv8 = LDAR/STLR (same instructions as acquire-release). DMB ISH only appears on seq_cst RMW (fetch_add/exchange/CAS), or on legacy ARMv7 where LDAR/STLR do not exist.

- `03-cpp-memory-model-atomics.md:49` — table row for `seq_cst`: "純 load/store 就是 LDAR/STLR... seq_cst RMW 才多一道 DMB ISH"
- `03-cpp-memory-model-atomics.md:55` — narrative: "ARMv8 則使用 LDAR/STLR 一條指令承擔 seq_cst load/store 語意，**額外的 DMB ISH 只出現在 seq_cst 的 RMW 操作**"
- `03-cpp-memory-model-atomics.md:112-119` — "ARMv8 的 seq_cst store = STLR... 差異在 RMW"
- `03-cpp-memory-model-atomics.md:427-438` — "Q: 為什麼 acquire-release 而不是 seq_cst?" with matching explanation
- `03-cpp-memory-model-atomics.md:497` — Q3 answer: "單純 seq_cst load/store 與 acquire/release 產生相同指令 (LDAR/STLR)，沒有額外 barrier 成本；差別主要出現在 seq_cst 的 RMW"
- `03-cpp-memory-model-atomics.md:513` — Q4 answer: "release/seq_cst 純 store 都是 STLR（沒差別）；效能差異主要來自 seq_cst 的 RMW 操作"
- `02-cpp-concurrency-sync.md:196` — table row for `seq_cst`: "全序一致（預設）/ 最高"
- `02-cpp-concurrency-sync.md:208` — "為什麼不全用 seq_cst" section header only; no concrete ARMv8 claim here
- `01-cpp-memory-optimization.md` — does not discuss seq_cst vs ARMv8 specifically; only mentions memory_order_acquire/release in the context of triple buffering (line 254)

**Status**: CONSISTENT. Ch03 is the authoritative location with correct ARMv8 mapping throughout. Ch02 and Ch01 do not contradict it (they only reference acquire/release and the general "seq_cst is the strongest/default" framing, which is true).

---

## Concept 2: Terminated vs truncated in RL

**Claim to enforce**: bootstrap mask uses `terminated` only (true absorbing state → V(s')=0). `truncated` (time-limit) should NOT mask bootstrap because MDP is still ongoing.

- `18-rl-mdp-basics.md:221-224` — Q-learning comment: "bootstrap mask 只用 terminated... 要用 V(s') 繼續 bootstrap"; code: `* (not terminated)`
- `18-rl-mdp-basics.md:262-267` — Q-learning main loop: same correct treatment
- `18-rl-mdp-basics.md:304-310` — SARSA: same correct treatment
- `18-rl-mdp-basics.md:399-416` — DQN section: "replay buffer 因此存 terminated 而非 done"; `buffer.push(..., float(terminated))`; `target_q = r + gamma * max_next_q * (1 - t)` with comment "只遮 terminated，不遮 truncated"
- `20-imitation-learning-dagger.md:319-332` — DAgger code uses `done = terminated or truncated` strictly as an **episode loop control** (not bootstrap mask). DAgger is supervised on expert actions and does not bootstrap value, so using `done` for loop control only is correct and does not conflict.
- `19-drl-ppo-sac-ddpg.md` — does NOT mention `terminated` or `truncated` anywhere; the chapter stays at the algorithmic level (PPO objective, SAC entropy, OU noise) and does not show bootstrap-mask code. No contradiction, but also no reinforcement.

**Status**: CONSISTENT. Ch18 carries the correct rule in four separate code blocks + comments (tabular Q-learning, SARSA, DQN, Gymnasium API section). Ch20's use of `done` is scoped to episode-loop control (correct usage). Ch19 has no code-level bootstrap discussion, so nothing to contradict — not a bug, but worth noting for future Ch19 expansion.

---

## Concept 3: DAgger β-mixing schedule

**Claim to enforce**: `π_i = β_i π_E + (1 - β_i) π̂_i` with `β_i` decaying toward 0; most implementations including the chapter's code take `β = 0` ("pure DAgger").

- `20-imitation-learning-dagger.md:68` — explicit statement: "原論文的 β-mixing schedule: π_i = β_i π_E + (1 - β_i) π̂_i，β_i 隨迭代遞減到 0... 實務上大多數實作（含本章 code 骨架）直接取 β = 0 純用學生動作，稱為 'pure DAgger'；論文的 regret 保證對此變體仍然成立"
- `20-imitation-learning-dagger.md:331` — DAgger iteration code uses student action only (β=0 in practice): `obs, _, terminated, truncated, _ = self.env.step(student_action)`
- `20-imitation-learning-dagger.md:462` — reading list cites original paper for "O(T) 線性上界與 β-mixing schedule"
- `21-sim-to-real-transfer.md:263,488,494` — Sim-to-Real references DAgger as a concept ("讓 Student 自己 rollout，Teacher 標註 Student 的觀測") but does not restate the β formula. Conceptually consistent with the pure-DAgger (β=0) view in Ch20.
- `22-multimodal-llm-robotics.md:130` — mentions DAgger as a data-collection technique; no β formula.
- `11-trajectory-optimization.md:669,698,1134` — mentions DAgger/Distillation for TrajOpt distillation; no β formula.

**Status**: CONSISTENT. Only Ch20 defines β. Ch21/Ch22/Ch11 reference DAgger conceptually and are compatible with Ch20's definition.

---

## Concept 4: Open X-Embodiment institution / embodiment count

**Claim to enforce**: "21 institutions across 22 robot embodiments".

- `22-multimodal-llm-robotics.md:115` — table: "22 個機器人、多機構聯合"
- `22-multimodal-llm-robotics.md:123` — "Open X-Embodiment 靠 21 個機構、涵蓋 22 種 robot embodiments 聯合"
- `22-multimodal-llm-robotics.md:436` — reading list: "21 機構、22 種 robot embodiments"
- `21-sim-to-real-transfer.md` — no OXE-number mention (grep for "Open X-Embodiment|21 機構|22 種" returned no hits in Ch21).

**Status**: CONSISTENT. All three Ch22 mentions agree ("21 institutions, 22 embodiments"). Ch21 does not restate the number — no contradiction to find.

---

## Concept 5: Jacobian / null-space conventions

**Claim to enforce**: velocity-space null-space projector = `(I - J^+ J)`; torque-space null-space projector = `(I - J^T J^{+T})`. These must not be conflated.

- `08-inverse-kinematics.md:61,64,180,198-199,298,352,357,359,362,374,386` — all velocity-space contexts; all use `(I - J^+ J)`. Correct.
- `07-forward-kinematics-dh.md:267` — velocity-space: `ΔΘ = J^+ v + (I - J^+ J) ∇H`. Correct.
- `14-force-impedance-control.md:114,117,171,174,631` — all torque-space contexts (command is `τ`); all use `(I - J^T J^{+T})`. Correct.
- `12-dynamic-obstacle-avoidance.md:682-685,706-707` — explicitly distinguishes the two: "**速度零空間投影** (I - J^+ J) 用於 q̇、**扭矩零空間投影** (I - J^T J^{+T}) 才是 τ 用的，兩者互為轉置". Code comment: `// torque null-space projector`. Correct.
- `09-robot-dynamics-modeling.md` — does not define a null-space projector formula (only references "零空間" in the WBC task-priority list at line 108). No contradiction.
- `15-model-predictive-control.md:108` — mentions "零空間" in WBC priority list only, no formula. No contradiction.

**Status**: CONSISTENT. The distinction is made correctly everywhere; Ch12 even calls out the distinction explicitly in-text.

---

## Concept 6: Lyapunov convention (V̇ ≤ 0) and CBF condition (ḣ + α·h ≥ 0)

**Claim to enforce**: Lyapunov stability uses `V̇ ≤ 0` (or `V̇ ≤ -α‖x‖²`); CBF forward-invariance condition is `ḣ + α(h) ≥ 0`.

- `09-robot-dynamics-modeling.md:55,61,507` — Slotine-Li adaptive: "V̇ = -s^T K s ≤ 0" ✓
- `14-force-impedance-control.md:121-127` — impedance passivity: "V̇ = -ė^T D_d ė ≤ 0（前提：D_d 正定）" ✓
- `15-model-predictive-control.md:245` — CBF embedded in MPC: "ḣ + α h ≥ 0" ✓
- `15-model-predictive-control.md:525` — Event-triggered Lyapunov: "V̇ ≤ -α‖x‖²" ✓
- `15-model-predictive-control.md:815,858,860` — CBF condition restated: "ḣ(x) + α h(x) ≥ 0" ✓
- `12-dynamic-obstacle-avoidance.md:378` — "若控制 u 能滿足 ḣ(x, u) + α(h(x)) ≥ 0，則 h 為 CBF" ✓
- `12-dynamic-obstacle-avoidance.md:387,390` — HOCBF: `ψ_1(x) = ḣ(x) + α_1(h(x)) ≥ 0`, `ψ_2 = ψ̇_1 + α_2(ψ_1) ≥ 0` ✓

**Status**: CONSISTENT. V̇ ≤ 0 direction (stability) and ḣ + α·h ≥ 0 direction (safety/forward-invariance) match across Ch09, Ch12, Ch14, Ch15. Note Ch14 adds passivity Lyapunov which uses the same V̇ ≤ 0 direction — compatible.

---

## Concept 7: Adjoint transform for wrench

**Claim to enforce**: twists transform via `Ad`, wrenches transform via `Ad^T` (opposite direction, derived from power invariance `F^T V`).

- `14-force-impedance-control.md:260-272` — explicit statement: "wrench 是 twist 的對偶... 由功率不變性 F^T V 可得 **wrench 以 Ad^T 向『相反方向』變換**". Convention: wrench ordering `[m, f]^T`, twist ordering `[ω, v]^T`. Matches Lynch & Park §3.4.
- `14-force-impedance-control.md:297` — code comment: `// 建構 Ad_T = [R, 0; p̂R, R]（twist 伴隨矩陣；wrench 轉換用 Ad_T^T）`
- `09-robot-dynamics-modeling.md` — does not discuss Adjoint for wrenches explicitly (CMM / spatial dynamics uses Featherstone spatial algebra, which is equivalent but not labeled "Ad^T" here).
- `08-inverse-kinematics.md:237` — uses `SE3 log map → twist` for IK error; no Adjoint for wrench.

**Status**: CONSISTENT. Only Ch14 defines Ad^T for wrench. Ch09 and Ch08 do not contradict — they simply don't use this framing. No cross-chapter conflict.

---

## Concept 8: Rotation / DH convention

**Claim to enforce**: Standard DH (Denavit) vs Modified DH (Craig) distinction must be consistent; right-multiplication order stated correctly.

- `07-forward-kinematics-dh.md:42-82,141,167,199,260,275` — comprehensive treatment: "Standard DH: Z_i 對齊第 i+1 軸... 乘法順序 Rot_Z → Trans_Z → Trans_X → Rot_X"; "Modified DH: Z_i 對齊第 i 軸... Rot_X → Trans_X → Rot_Z → Trans_Z". Right-multiply (`必須右乘`) stated at line 56.
- `08-inverse-kinematics.md:392` — reading list recommends Lynch & Park screw theory over DH; no DH convention restated.
- `09-robot-dynamics-modeling.md` — uses Featherstone spatial-algebra / RNEA frames, does not use DH explicitly. No DH convention claims to contradict.

**Status**: CONSISTENT. Only Ch07 defines the DH convention; Ch08 and Ch09 do not restate it and do not contradict.

---

## Concept 9: Differential Flatness for drones

**Claim to enforce**: flat outputs `[x, y, z, ψ]` (position + yaw).

- `11-trajectory-optimization.md:234,235,772,790,1031,1151,1174` — explicit: "flat outputs [x, y, z, ψ]... 4 個 flat outputs 能代數反推所有狀態" (line 1151); "畫 3D 平滑曲線 (x(t), y(t), z(t), ψ(t))" (line 234). Cites Mellinger-Kumar 2011.
- `10-basic-path-planning.md:1032` — mentions "微分平坦性" / "B-spline 的控制點天然滿足微分平坦性質"; no flat-output list, no contradiction.
- `15-model-predictive-control.md:247` — mentions 四旋翼 only in context of CBF relative degree, not flatness. No contradiction.
- `16-visual-servoing.md:331,647,655` — "IBVS + Differential Flatness" for drones; does not restate flat-output list. Compatible.

**Status**: CONSISTENT. Ch11 is the authoritative statement; Ch10, Ch15, Ch16 reference flatness as a concept without restating (or contradicting) the flat-output list.

---

## Concept 10: PBVS formula

**Claim to enforce**: `v_c = -λ (R^T · ^{c*}t_c, θu)` — with R^T rotating translation from target frame c* into current camera frame c.

- `16-visual-servoing.md:80` — `v_c = -λ (R^T · ^{c*}t_c, θu)^T` ✓
- `16-visual-servoing.md:83,85` — explicit caveat: "**R^T 至關重要** — 它把平移向量從目標座標系 c* 轉回相機自身座標系 c"; "座標系陷阱：絕不能把 ^c t 和 ^{c*} t 兩個絕對平移向量相減"
- All other chapters: no PBVS formula appears. `grep "v_c = -"` outside Ch16 returns only IBVS usages (which is a different control law: `v_c = -λ L_s^+ (s - s*)`).

**Status**: CONSISTENT (trivially — only Ch16 states the PBVS formula). The round-2 fix holds; there is no stale parallel version elsewhere to drift out of sync.

---

## Additional observation (not a request but worth flagging)

Ch19 (DRL: PPO/SAC/DDPG) never uses `terminated`/`truncated`/`done` in any code or text (verified by grep). Ch18 carries all the bootstrap-mask discipline. If Ch19 ever gets code-level PPO/SAC implementations added, they will need to be synced with the Ch18 rule (mask bootstrap on `terminated` only). Not a current bug.
