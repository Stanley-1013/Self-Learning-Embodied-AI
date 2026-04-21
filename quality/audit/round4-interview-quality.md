# Round 4: Interview Angle Quality

## Summary
- Chapters scanned: 22
- Issues found: 7 (HIGH:1, MED:4, LOW:2)
- Overall: interview angles are largely consistent with core content and cite real papers/years correctly. Main issues are unverifiable industry attributions ("Tesla 用 X 100%") and uncited percentages.

## Ch01 (C++ Memory Optimization)
- No issues found. Claims about `alignas(64)` cache-line, `unique_ptr + move` zero-copy are technically accurate and consistent with core.

## Ch02 (C++ Concurrency)
- No issues found. `scoped_lock` deadlock avoidance, SPSC queue with acquire/release, CallbackGroup alternative — all correct.

## Ch03 (C++ Memory Model / Atomics)
- No issues found. ARMv8 `LDAR`/`STLR` equivalence between seq_cst and acquire/release loads/stores is **factually correct** (a common misconception to know). LL/SC spurious fail reasoning for `compare_exchange_weak` is accurate. Well-written, strong technical grounding.

## Ch04 (ROS 2 Basic Communication)
- Interview point #3: "把 36 MB/s 的拷貝開銷壓到零"
- Consistent with core example (1.2 MB × 30 fps = 36 MB/s). No issue.

## Ch05 (ROS 2 TF)
- No issues found. REP-105 hierarchy, quaternion rationale, `canTransform` try-catch guidance all standard and correct.

## Ch06 (ROS 2 Advanced)
- No issues found. SensorDataQoS best-effort rationale, Executor + CallbackGroup architecture are ROS 2 community-accepted best practices.

## Ch07 (Forward Kinematics / DH)
- No issues found. Standard vs Modified DH, Jacobian condition number monitoring, SIMD unrolling — all real techniques.

## Ch08 (Inverse Kinematics)
- Interview point #4: "業界 90% 的 IK 問題不是算法錯，而是 workspace 邊界..."
- Issue: uncited percentage (**overclaim**). The 90% figure is a rhetorical approximation with no source.
- Severity: LOW. The underlying checklist advice is correct and useful; the percentage is just shorthand. Consider rewording to "大多數" instead of "90%".

## Ch09 (Robot Dynamics)
- Interview point #7: "Tesla Optimus / Figure 的底層 **100%** 是 CWC-QP"
- Issue: **overclaim / industry attribution**. Neither Tesla nor Figure has publicly confirmed CWC-QP as their core stack. Core content line 260 says "底層 100% 依賴 CWC-QP" (same overclaim), and line 590 correctly labels Neural ABA for Tesla/Figure as "傳言" (rumor).
- Severity: **HIGH**. "100%" is an absolute claim that can't be verified; if an interviewer challenges it, the candidate has no citation. Suggest: "多數 state-of-the-art humanoid WBC 都以 CWC-QP 為骨架（ANYmal / MIT Humanoid 公開論文）；Tesla Optimus / Figure 雖未公開細節，但其多接觸能力高度一致於這類架構。"

- Interview point #9: "這是 Tesla / Figure 的路線" (大腦 RL + 小腦 WBC)
- Issue: same class of overclaim, less severe because it's described as "架構" not "100%".
- Severity: MED. Suggest softening to "對外展示的架構高度符合這類分層設計".

- Interview point #2: "7-DoF 在 ARM 上 50 μs 搞定"
- No citation but plausible (RNEA on modern ARM Cortex-A is indeed sub-100 μs for 7-DoF). Accept as reasonable order-of-magnitude.

## Ch10 (Basic Path Planning)
- Interview point #5: "D* Lite ... 復用 **99%** 搜索樹"
- Consistent with core (line 381, line 889). Same number reused. 99% is a marketing-style round number; real figure depends on obstacle density. LOW severity — same claim throughout chapter, not internally contradictory.

- Interview point #7: "BIT\* ... 比 RRT\* 縮短 **60%+**"
- Issue: specific benchmark number without citation.
- Severity: LOW. The qualitative claim (BIT* faster than RRT* in narrow passages) is well-established in literature (Gammell et al. 2015). The specific "60%+" is hard to verify as a universal claim. Consider softening to "實測常比 RRT\* 快一個等級".

## Ch11 (Trajectory Optimization)
- Interview point #8: "Bang-Bang 原則 ... 每瞬間至少一個關節扭矩打滿"
- Technically correct per Pontryagin's minimum principle for time-optimal problems with box constraints. Consistent with core line 297.
- No issue found.

- Interview point #14: "ACT ... 時間低通濾波，穿針打雞蛋等精細操作**唯一解**"
- Issue: "唯一解" (only solution) is overclaim. Diffusion Policy + action chunking is an alternative.
- Severity: LOW. Consider softening to "主流解".

## Ch12 (Dynamic Obstacle Avoidance)
- Interview point #11: "ETH ANYmal、Boston Dynamics Atlas **都**用這架構" (Privileged Teacher-Student)
- Issue: ANYmal (Hwangbo 2019, Lee 2020, Miki 2022) explicitly uses Teacher-Student. Boston Dynamics **has not publicly confirmed** Atlas uses Teacher-Student specifically. Core content line 643 more cautiously says "部分 RL 模組", interview says "都用這架構".
- Severity: **MED**. Contradicts chapter's own cautious phrasing. Fix: drop Atlas or say "Atlas 採類似分層 RL 架構（具體模組未公開）".

- Interview point #2: "L4 自駕的工業標配" (Chance-Constrained MPC)
- Issue: Waymo / Cruise / Apollo details are proprietary; the claim is plausible but unverifiable.
- Severity: LOW. Standard academic framing, acceptable.

## Ch13 (PID Control Tuning)
- Interview point #11: "工業 **90%** 用 Gain Scheduling 依 M(q) 動態調增益"
- Issue: uncited percentage. Many industrial robots use fixed gains tuned per payload preset, not true Gain Scheduling with M(q) lookup. True M(q)-based Gain Scheduling is more common on high-end (KUKA iiwa, Franka) than mid-tier industrial arms.
- Severity: **MED**. Overclaim on "90%". Suggest: "工業高端伺服 / 協作臂常用 Gain Scheduling...".

- Interview point #5: "電流 10 kHz、速度 1 kHz、位置 100 Hz，頻寬比 5-10 倍"
- Consistent with core table (lines 192-193 show sampling rates matching). "頻寬比 5-10 倍" refers to control bandwidth ratios, which in core is 1 kHz / 100 Hz / 10 Hz — ratio of 10x. Consistent.
- No issue.

## Ch14 (Force / Impedance Control)
- Interview point #5: "MIT Cheetah、Tesla Optimus 選 QDD 不選 Harmonic Drive"
- MIT Cheetah: confirmed — Sangbae Kim's lab pioneered QDD (Seok et al. 2012+).
- Tesla Optimus: publicly shown to use a mix of actuators including **planetary gearboxes and strain-wave (harmonic) drives** at some joints. The unqualified claim "Tesla Optimus 選 QDD 不選 Harmonic Drive" is **inaccurate**.
- Severity: **MED**. Fix: "MIT Cheetah / Unitree H1 / 許多 humanoid 腿關節選 QDD；Tesla Optimus 為混合架構（部分關節 QDD、部分諧波減速器）".

- Interview point #1: Mason 1981 Task Frame Formalism — correct citation.
- Interview point #4: Khatib 1987 Operational Space — correct citation.
- Interview point #10: Momentum Observer for ISO/TS 15066 PFL — correct (De Luca et al. formulation, real standard).

## Ch15 (Model Predictive Control)
- Interview point #12: "Tesla Optimus、Figure、1X Neo **都走這路線**" (VLA + MPC + WBC/CBF)
- Tesla Optimus: AI Day presentations emphasize **end-to-end neural net + imitation learning**, not VLA + MPC explicitly. The lower-level architecture details are undisclosed.
- Figure 02: confirmed uses OpenAI VLM — defensible.
- 1X Neo: uses learned policies; specific VLA + MPC layering unconfirmed.
- Severity: **MED**. The general trend (high-level NN + low-level MPC) is plausible, but claiming "都走這路線" with VLA specifically is overclaim. Fix: "許多前沿 humanoid 都採類似分層（VLM/VLA 上層 + MPC/WBC 下層），Figure 02 明確採 OpenAI VLM + 自研 MPC。Tesla Optimus 架構細節未公開但公開 demo 顯示類似分層特徵。"

- Interview point #2: "機械臂不用 MPC" — reasonable simplification for industrial arms; modern contact-rich manipulation DOES use MPC, but the point is pedagogical. Acceptable.

- Interview point #8: "Contact-Implicit MPC 2024–2025 最熱" — correct. Drake + CI-TrajOpt references are real.

## Ch16 (Visual Servoing)
- Interview point #3: "Chaumette 1998" — correct citation (Chaumette, "Potential problems of stability and convergence in image-based and position-based visual servoing", 1998).
- Interview point #5: "旋轉偏 1° 在 30 cm 工作距離會導致 ~5 mm 末端誤差"
- Math check: 30 cm × sin(1°) ≈ 300 mm × 0.01745 ≈ 5.2 mm. **Correct arithmetic**.
- No issue.

## Ch17 (SLAM)
- Interview point #2: "稀疏 Cholesky 分解把 $O(n^3)$ 的稠密求解壓到接近 $O(n)$"
- Core content line 125 more carefully says "$O(n)$ 或 $O(n^{1.5})$". Interview says "接近 $O(n)$".
- Severity: LOW. For band-structured SLAM graphs $O(n)$ is indeed achievable; for general sparse graphs with loop closures $O(n^{1.5})$ is more typical. Small gloss, not wrong.

## Ch18 (RL / MDP Basics)
- No issues found. Q-Learning off-policy vs SARSA on-policy safety distinction is textbook-correct (Cliff Walking example, Sutton & Barto). ε-greedy annealing advice is sound.

## Ch19 (DRL: PPO/SAC/DDPG)
- Interview point #1: "DDPG 已經被 TD3/SAC 取代，我不會用"
- Consistent with core claim that SAC is preferred in robotics. Strong but defensible — DDPG is indeed rarely used in modern practice.
- No issue.

- Interview point #4: "RL policy 20 Hz 輸出目標、PD/MPC 1 kHz 追蹤" — matches standard humanoid/quadruped architecture. OK.

## Ch20 (Imitation Learning / DAgger)
- Interview point #1: "$O(T^2)$ 降到 $O(T)$"
- Correct: Ross & Bagnell 2010 / 2011 bounds are exactly these orders.
- Interview point #3: Ho & Ermon GAIL — correct citation (Ho & Ermon, NeurIPS 2016).
- No issues found.

## Ch21 (Sim-to-Real)
- Interview point #5: "OpenAI 用它解決了 Rubik's Cube"
- Correct: OpenAI 2019 "Solving Rubik's Cube with a Robot Hand" used ADR.
- No issues found.

## Ch22 (Multimodal LLM / Robotics)
- Interview point #2: "SayCan 就是這個範式的代表"
- Correct: Ahn et al. 2022 (Google Robotics) SayCan is exactly affordance-based filtering.
- Interview point #3: "RT-2 用 256 bins 每維"
- Correct: RT-2 paper (Brohan et al. 2023) uses 256 discrete bins per action dimension.
- Interview point #4: DreamerV3 — correct (Hafner et al. 2023).
- No issues found.

---

## Aggregate Assessment

**Strengths**:
- Citation accuracy (years, authors, paper titles) is consistently correct: Chaumette 1998, Mason 1981, Khatib 1987, Ross & Bagnell 2010/2011, Ho & Ermon 2016, Hwangbo 2019, OpenAI Rubik's 2019, RT-2 2023, DreamerV3 2023, SayCan 2022.
- Technical reasoning chains are sound (Bellman recursion, BC compounding error, bang-bang from Pontryagin, retreat-then-advance from $L_s$ structure).
- Math arithmetic where checked (5 mm error from 1° at 30 cm) is correct.
- Internal consistency between interview angles and core concepts is strong — no direct contradictions within a chapter.

**Recurring weakness: industry attribution to closed-source companies**.
The one pattern that shows up repeatedly is confident attribution of specific algorithms to Tesla Optimus / Figure / 1X Neo / Boston Dynamics Atlas. These companies have **not published detailed architecture papers**, so claims like "100% 是 CWC-QP", "都走 VLA + MPC 路線", "選 QDD 不選 Harmonic Drive" are speculation. Interviewers from these companies (or their competitors) will push back.

**Recommended fixes** (by priority):
1. **Ch09 #7** (HIGH): Drop "100%" on Tesla/Figure CWC-QP claim; cite published platforms (ANYmal, MIT Humanoid) instead.
2. **Ch14 #5** (MED): Correct Tesla Optimus actuator claim — Optimus uses mixed actuators, not pure QDD.
3. **Ch12 #11** (MED): Remove Atlas from Teacher-Student list or soften; keep ANYmal (published).
4. **Ch15 #12** (MED): Soften "都走這路線" to "許多前沿 humanoid 採類似分層".
5. **Ch13 #11** (MED): Replace "90%" with "多數工業高端伺服".
6. **Ch08 #4, Ch10 #7, Ch11 #14, Ch17 #2** (LOW): minor rewording of percentages / superlatives.

**Zero findings**: Ch01, Ch02, Ch03, Ch04, Ch05, Ch06, Ch07, Ch18, Ch19, Ch20, Ch21, Ch22 (12 of 22 chapters clean).
