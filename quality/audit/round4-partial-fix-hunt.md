# Round 4: Partial-Fix Hunt

## Summary
- Inconsistencies found: 1 (borderline/minor) + 1 (minor imprecision in diagnostic prose)
- All 12 concepts verified fully synced: MOSTLY YES (no CRITICAL mismatches; two LOW-severity imprecisions noted)

The big-ticket items from Rounds 1-3 (ARMv8 seq_cst mapping in Ch03, Nav2 constants in Ch10, null-space torque/velocity distinction in Ch07/08/12/14, Adjoint wrench direction in Ch14, PBVS `R^T` in Ch16, terminated/truncated in Ch18/20, OXE 21/22 in Ch22, Barbalat uniform continuity in Ch09, LMI attribution in Ch09, Lillicrap 2015/2016 in Ch19, Ross & Bagnell dates in Ch20, cubic polynomial jerk in Ch11) are all correctly synced in the primary definition passages.

Two residual looseness sites are flagged below as [LOW]. Neither rises to the level of mathematical error; both are diagnostic/table shorthand that is technically defensible but less precise than the main-body treatment.

---

## Concept 1: ARMv8 seq_cst mapping

All occurrences (`seq_cst|STLR|DMB|LDAR` in Ch01-03):
- `03-cpp-memory-model-atomics.md:49` (table): "純 load/store 就是 LDAR/STLR (和 acquire/release 同指令); seq_cst RMW (如 fetch_add) 才多一道 DMB ISH"
- `03-cpp-memory-model-atomics.md:55`: "ARMv8 則使用 LDAR/STLR 一條指令承擔 seq_cst load/store 語意, 額外的 DMB ISH 只出現在 seq_cst 的 RMW 操作..."
- `03-cpp-memory-model-atomics.md:112-119` (Assembly cheat sheet): Explicitly says `ARMv8 的 seq_cst store = STLR`, `load = LDAR`, and differentiates RMW path needing `LDAXR/STLXR` + possibly DMB.
- `03-cpp-memory-model-atomics.md:432-438`: "seq_cst load = LDAR, seq_cst store = STLR - 和 acquire/release 相同指令"; RMW explicitly singled out for DMB.
- `03-cpp-memory-model-atomics.md:497`: Correct (single-sensor case explicit about RMW vs pure load/store).
- `03-cpp-memory-model-atomics.md:508`: "用 `perf stat` 觀察 `dmb` 指令數量 — seq_cst 會大量插入 DMB" — **[LOW]** imprecise as a blanket diagnostic. Strictly, DMB appears only on seq_cst RMW on ARMv8. The follow-up step 3 (line 513) does qualify this correctly, so the full Q4 answer remains correct, but step 1 alone reads as if seq_cst broadly inserts DMB. Could be tightened to "seq_cst 的 RMW 會插入 DMB".
- `03-cpp-memory-model-atomics.md:513`: "ARMv8 上 relaxed store 是普通 STR, release/seq_cst 純 store 都是 STLR (沒差別); 效能差異主要來自 seq_cst 的 RMW 操作..." — CLEAN.
- `03-cpp-memory-model-atomics.md:523`: "純 seq_cst load/store 與 acquire/release 其實產生相同的 LDAR/STLR 指令 / 真正差異在 RMW" — CLEAN.
- `02-cpp-concurrency-sync.md:196`: table entry "seq_cst / 全序一致 (預設) / 最高" — generic, no ARM specifics so not misleading.
- `02-cpp-concurrency-sync.md:208-210` (為什麼不全用 seq_cst): "在 ARMv8 上, 純 load/store 與 acquire/release 使用相同的 LDAR/STLR 指令、沒有額外 barrier 成本; 差異主要在 seq_cst 的 RMW 會多一道 DMB ISH..." — CLEAN, cross-ref to Ch03.

Status: **CLEAN** for the mathematical/architectural statement; **[LOW]** wording imprecision at Ch03:508 (step 1 of the Q4 diagnostic). Suggested tweak: `"— seq_cst 的 RMW 會插入 DMB"` rather than `"— seq_cst 會大量插入 DMB"`. Not a partial-fix in the strict sense — the main claim about ARMv8 mapping is correctly synced everywhere.

---

## Concept 2: Nav2 InflationLayer constant

All occurrences (Ch10):
- `10-basic-path-planning.md:942`: `Cost(d) = (INSCRIBED_INFLATED_OBSTACLE - 1) · exp(-α · (d - r_inscribed)) = 252 · exp(...)`, with d ∈ (r_inscribed, r_inflation]; d ∈ (0, r_inscribed] → 253 (INSCRIBED_INFLATED_OBSTACLE); d = 0 → 254 (LETHAL_OBSTACLE).

Status: **CLEAN**. Single canonical mention, uses the corrected `(INSCRIBED_INFLATED_OBSTACLE - 1) = 252` decay factor and correctly tiers 252/253/254 with correct predicates.

---

## Concept 3: Null-space projector (torque vs velocity space)

Velocity/displacement space `(I - J^+ J)`:
- `07-forward-kinematics-dh.md:267`: `ΔΘ = J^+ v + (I - J^+ J) ∇H` — displacement space, CORRECT.
- `08-inverse-kinematics.md:61`: `q̇ = J^+ ẋ + (I - J^+ J) q̇_0` — velocity space, CORRECT.
- `08-inverse-kinematics.md:180`: `(I - J_pinv * J) * q0_dot` — CORRECT.
- `08-inverse-kinematics.md:357`: `q̇ = J^+ ẋ + (I - J^+ J) ∇H` — CORRECT.
- `08-inverse-kinematics.md:362, 386`: `(I - J^+ J)` talking points — CORRECT.

Torque space `(I - J^T J^{+T})`:
- `12-dynamic-obstacle-avoidance.md:682`: explicit side-by-side distinction: "扭矩空間投影 (I - J^T J^{+T}) ... 速度零空間投影 (I - J^+ J)" — the ONE place the two are juxtaposed, and the distinction is correctly stated.
- `12-dynamic-obstacle-avoidance.md:707`: `Eigen N = I - J.transpose() * J_pinv.transpose(); // torque null-space projector` — CORRECT.
- `14-force-impedance-control.md:114, 117, 171, 174, 631`: all torque equations use `(I - J^T J^{+T})` — CORRECT.

Status: **CLEAN** — no chapter mixes the two forms. The side-by-side clarification at Ch12:682 is the explicit guard, and every other use is consistent with its domain (Δθ/q̇ vs τ).

---

## Concept 4: Adjoint wrench direction

All occurrences (Ch09/14):
- Ch09: No `Ad_T` / `Adjoint` / `伴隨` matches. (OK — Adjoint is hosted in Ch14.)
- `14-force-impedance-control.md:269`: `Ad_{T_AB} = [R,0; p̂R, R]` with `V_A = Ad_{T_AB} V_B` (twist).
- `14-force-impedance-control.md:272`: "wrench 以 Ad^T 向『相反方向』變換" — CLEAN.
- `14-force-impedance-control.md:275`: `W_B = Ad_{T_AB}^T · W_A` — CORRECT (opposite direction from twist).
- `14-force-impedance-control.md:278`: direction verification with `p = 0` case + explicit rebuttal that `W_A = Ad^T W_B` is wrong — CLEAN and pedagogically strong.
- `14-force-impedance-control.md:297` (code comment): "Ad_T = [R,0; p̂R, R] (twist 伴隨矩陣; wrench 轉換用 Ad_T^T)" — CLEAN.
- `14-force-impedance-control.md:303` (code comment): `// 剛度共軛: W = Ad_T^T · W` — **[LOW]** shorthand comment with same `W` on both sides. The intent (stiffness sandwich `K_tcp = Ad^T · K · Ad`) is correct and visible on the same line; the comment just abbreviates. Could be written `W_tcp = Ad^T · W_flange` for clarity.

Status: **CLEAN** on the mathematical body; **[LOW]** shorthand in one code comment at line 303. Since the math body (line 275) and the direction verification (line 278) are unambiguous and the actual line of code is correct, this is cosmetic.

---

## Concept 5: PBVS formula with R^T

All occurrences (Ch16):
- `16-visual-servoing.md:80`: `v_c = -λ (R^T · ^{c*}t_c, θu)^T` — CORRECT.
- `16-visual-servoing.md:83`: "`R^T` 至關重要 — 它把平移向量從目標座標系 c* 轉回相機自身座標系 c" — CLEAN explanation with Chaumette & Hutchinson 2006 Eq. 23-27 citation.
- `16-visual-servoing.md:85`: "v_c 是在 current camera frame c 下的 6-DoF 速度指令, 因此 translation 必須以 R^T 轉回 c frame" + explicit trap rebuttal. CLEAN.
- `16-visual-servoing.md:813`: Q2 answer mentions "PBVS (從 Homography 提取的 θu 旋轉誤差)" — consistent.
- `16-visual-servoing.md:816`: "先用 PBVS 做粗定位" — consistent.
- No `^{c*}t_c` occurrence appears bare without `R^T` in a control-law context.

Status: **CLEAN**.

---

## Concept 6: Terminated vs truncated

All occurrences (Ch18-20):
- `18-rl-mdp-basics.md:159`: API description `env.step(action) → obs, reward, terminated, truncated, info` — correct Gymnasium API.
- `18-rl-mdp-basics.md:218-224` (Q-learning): bootstrap mask uses `(not terminated)`; `done = terminated or truncated` only for loop control. Comment explicitly warns time-limit case. CLEAN.
- `18-rl-mdp-basics.md:262-267` (Sarsa): same pattern. CLEAN.
- `18-rl-mdp-basics.md:301-310` (Expected Sarsa / DDQN): same pattern. CLEAN.
- `18-rl-mdp-basics.md:399-416` (DQN): bootstrap uses `terminated`, replay buffer stores `float(terminated)`, target uses `(1 - t)` where `t = terminated mask, NOT done`. CLEAN with explicit commentary.
- `19-drl-ppo-sac-ddpg.md`: no `terminated`/`truncated` matches — Ch19 does not show env-step pseudocode, so no chance to be wrong here. Not a gap.
- `20-imitation-learning-dagger.md:319-332`: `done = terminated or truncated` used for episode-loop control only; DAgger rollouts don't bootstrap so `terminated` alone isn't needed. CORRECT usage pattern.

Status: **CLEAN**.

---

## Concept 7: OXE institution count

All occurrences:
- `22-multimodal-llm-robotics.md:115`: "21 機構、22 種 robot embodiments 聯合" — CLEAN.
- `22-multimodal-llm-robotics.md:123`: "21 個機構、涵蓋 22 種 robot embodiments 聯合" — CLEAN.
- `22-multimodal-llm-robotics.md:129`: "Open X-Embodiment 證明不同機器人的數據可以互相幫助" — no numbers, consistent.
- `22-multimodal-llm-robotics.md:436`: "**21 機構、22 種 robot embodiments** 聯合數據集" — CLEAN.
- Ch11/14/15/16/20/21: no OXE mentions (checked explicitly).

Status: **CLEAN** — all three numeric mentions pair `21 institutions` with `22 embodiments`.

---

## Concept 8: Barbalat uniform continuity

All occurrences (Ch09):
- `09-robot-dynamics-modeling.md:61`: passivity property and Lyapunov framing, no Barbalat here.
- `09-robot-dynamics-modeling.md:507`: "再由 **Barbalat's Lemma**（$\dot V$ 均勻連續）推得 $s \to 0$" — CLEAN, explicitly states V̇ uniform continuity requirement.
- `09-robot-dynamics-modeling.md:486, 488, 519, 524`: Slotine-Li references, don't restate Barbalat.
- No other file references Barbalat.

Status: **CLEAN** — the single Barbalat mention is the corrected formulation.

---

## Concept 9: Triangle inequality LMI attribution

All occurrences (Ch09):
- `09-robot-dynamics-modeling.md:471`: "等價於 pseudo-inertia matrix ⪰ 0 的 LMI 形式, 見 Traversaro, Prete & Nori 2016、Wensing, Kim & Slotine 2017; 註: tr(I_i) > λ_max 只是此條件的必要弱化..." — CLEAN.
- `09-robot-dynamics-modeling.md:468, 481, 657, 663, 785, 804`: other LMI mentions don't re-cite; consistent with the single canonical attribution at line 471.
- Global `Souloumiac` grep: no matches anywhere.

Status: **CLEAN** — correct Traversaro/Wensing attribution, no Souloumiac anywhere.

---

## Concept 10: DDPG citation

All occurrences (Ch19):
- `19-drl-ppo-sac-ddpg.md:115`: "DDPG (Lillicrap et al., arXiv 2015 / ICLR 2016, DeepMind)" — CLEAN.
- Other DDPG mentions in Ch19 (lines 14, 24, 26, 28, 32, 46, 101, 139, 379, 389, 396, 398, 490) are conceptual, not citations.

Status: **CLEAN**.

---

## Concept 11: Ross & Bagnell BC bound dates

All occurrences (Ch20):
- `20-imitation-learning-dagger.md:52`: "Ross & Bagnell, AISTATS 2010" for BC O(T²) bound — CLEAN.
- `20-imitation-learning-dagger.md:68`: "Ross et al. 2011" for DAgger β-mixing — CLEAN.
- `20-imitation-learning-dagger.md:101`: "Ross & Bagnell (AISTATS 2010, 'Efficient Reductions for Imitation Learning')" — CLEAN.
- `20-imitation-learning-dagger.md:116`: "Ross, Gordon & Bagnell (AISTATS 2011)" for DAgger linear upper bound — CLEAN.
- `20-imitation-learning-dagger.md:461-462`: bibliography entries correctly date 2010 vs 2011.

Status: **CLEAN** — both dates consistently cited throughout.

---

## Concept 12: Cubic polynomial jerk

All occurrences (Ch11):
- `11-trajectory-optimization.md:44` (comparison table inside 核心概念): "單段加速度不能同時指定起／終點值 → 多段拼接時交接點加速度不連續 → jerk 發散 → 微小震顫" — CLEAN, explicitly qualifies "at segment junctions".
- `11-trajectory-optimization.md:60`: "單段內加速度 $\ddot{q} = 2a_2 + 6a_3 t$ 雖是連續線性函數, 起／終點加速度值無法指定, **多段 cubic 拼接時交接點加速度不連續 → jerk 出現階躍**" — CLEAN, explicit within-segment vs junction distinction.
- `11-trajectory-optimization.md:68`: quintic jerk continuity motivation — CLEAN.
- `11-trajectory-optimization.md:157`: section heading 三次多項式 — context.
- `11-trajectory-optimization.md:716-717` (second comparison table in 常見參數化方法比較): `| 三次多項式 | $C^1$ | 否 | 簡單、解析解 | 加速度不連續 |` — **[LOW]** the "缺點: 加速度不連續" cell is an unqualified shorthand. Strictly this is a consequence of $C^1$ (piecewise cubic has discontinuous $\ddot q$ at junctions), and the header column `連續性: C^1` already encodes "at most C^1", but the bare string "加速度不連續" is less precise than line 44/60's within-vs-between distinction. Could be tightened to "交接點加速度不連續".
- `11-trajectory-optimization.md:1147`: talking point repeats "Cubic 加速度不連續 → jerk 階躍" — same shorthand, same context (summary).

Status: **[LOW]** — the primary definitional passages (lines 44, 60) correctly distinguish within-segment (linear acceleration, finite jerk except at boundaries) from segment junctions (jerk step). The two table/talking-point shorthands (717, 1147) omit the qualifier. Not a correctness error given the `C^1` column encodes the same fact, but less precise than the main body.

---

## Final verdict

**No hard partial-fix caught in Round 4.** The pattern that plagued Rounds 1-3 (concept corrected in the primary passage, stale copy left in a secondary mention) is not present this time. The two [LOW] flags are:

1. **Ch03:508** — step 1 of Q4 answer says "seq_cst 會大量插入 DMB" without the "RMW only" qualifier. Step 3 on line 513 of the same answer block does clarify correctly, so the overall Q4 reasoning is sound, but step 1 in isolation is loose.

2. **Ch11:717, :1147** — table/talking-point shorthand "加速度不連續" omits the "at segment junctions" qualifier that lines 44 and 60 carefully include. Technically consistent with the `C^1` continuity column but less precise than the main-body treatment.

Neither issue constitutes a contradictory claim. Round 4 audit is effectively a clean pass — the 12 targeted concepts all have their primary corrected forms in place, and the secondary mentions are consistent (modulo the two minor shorthands above).

### Suggested LOW-severity polish (optional)

- `content/embodied-robotics/03-cpp-memory-model-atomics.md:508`: change "seq_cst 會大量插入 DMB" → "seq_cst 的 RMW 會插入 DMB" for consistency with step 3.
- `content/embodied-robotics/11-trajectory-optimization.md:717`: change "加速度不連續" → "交接點加速度不連續" for consistency with lines 44/60.
- `content/embodied-robotics/14-force-impedance-control.md:303`: change code-comment `// 剛度共軛：W = Ad_T^T · W` → `// K_tcp = Ad_T^T · K_flange · Ad_T (sandwich form; wrench: W_tcp = Ad_T^T · W_flange)` for unambiguous labels.
