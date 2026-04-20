# Audit Report: Ch19-22

**Summary**: Chapters are largely accurate on core algorithmic and citation facts (DDPG/TD3/SAC/PPO years + affiliations, DAgger AISTATS 2011, Diffusion Policy Chi et al. 2023, ACT Zhao et al. 2023, RT-2 2023, PaLM-E 562B, Rubik's Cube ADR 2019 all check out). Most findings are MEDIUM severity — a fabricated-looking scaling formula in Ch22, one shaky intuition gloss on PPO clip, an incomplete TD3 description, and a couple of claims presented with more confidence than the literature supports. No CRITICAL errors found.

---

## Ch19 DRL

### Findings

- **[MED]** 19-drl-ppo-sac-ddpg.md:185 — PPO clip intuition says "每步最多偏離 20%" (off by 20%). The 0.2 clips the **probability ratio** $r_t = \pi_\theta/\pi_{\text{old}}$, not "action deviation" or "policy deviation" in a metric sense; readers often mistake this for a 20% KL or a 20% action-space bound. — Fix: rephrase as "把新舊策略的機率比 $r_t$ 限制在 [0.8, 1.2] 區間內，超出就停止貢獻梯度" and drop the "偏離 20%" framing. — confidence: high

- **[MED]** 19-drl-ppo-sac-ddpg.md:126 / table at :46 — TD3 description lists "Target smoothing" as the **exploration mechanism** in the comparison table. Target policy smoothing is a **critic-side regularizer** (noise added to target action when computing TD target), not an exploration strategy; TD3's actual exploration is Gaussian noise on the behavior policy (like DDPG, just Gaussian instead of OU). — Fix: change table row "探索機制" for TD3 to "Gaussian 行為噪聲 (target smoothing 是 critic 正則化)". — confidence: high

- **[LOW]** 19-drl-ppo-sac-ddpg.md:115 — "DDPG (2015, DeepMind)" is borderline. Lillicrap et al. arXiv'd DDPG in Sept 2015 and it appeared at **ICLR 2016**. Not strictly wrong (first arXiv was 2015), but if cited as a paper year alongside "TD3 (2018)" (an ICML year) it's inconsistent. — Fix: use "2015/ICLR 2016" or standardize all to venue year. — confidence: medium

---

## Ch20 Imitation Learning

### Findings

- **[MED]** 20-imitation-learning-dagger.md:54-58 — The BC error bound $J(\pi_E) - J(\pi_\theta) \le T^2 \epsilon + O(T)$ is attributed loosely to "Ross et al. 2011". The $O(T^2)$ result is from Ross & Bagnell's earlier **AISTATS 2010 paper "Efficient Reductions for Imitation Learning"**, not the 2011 DAgger paper (which proved the $O(T)$ upper bound for DAgger). Mixing them up is a common citation slip. — Fix: cite Ross & Bagnell 2010 for the BC $T^2$ lower bound, Ross, Gordon & Bagnell 2011 (AISTATS) for DAgger's $O(T)$. — confidence: high

- **[MED]** 20-imitation-learning-dagger.md:60 — The chapter outer-loop description never mentions DAgger's **$\beta$-mixing schedule** ($\pi_i = \beta_i \pi^* + (1-\beta_i)\hat\pi$ with $\beta_i \to 0$), which is part of the original algorithm and a common interview probe. The code skeleton at :329 rolls out using pure student action, which is actually DAgger in the $\beta=0$ limit (sometimes called "pure DAgger") but the chapter never flags the distinction. — Fix: add a sentence noting the $\beta_i$ mixing schedule and that the code uses the common $\beta=0$ variant. — confidence: high

- **[LOW]** 20-imitation-learning-dagger.md:184 — "ACT (Action Chunking with Transformers)" is described as using "CVAE + Transformer". Correct in spirit, but the chapter earlier at :33 and :463 credits ACT for "action chunking + Temporal Ensemble" without mentioning that **temporal ensemble** is the inference-time trick in ACT (exponentially-weighted average across overlapping chunks); the chapter at :181 calls temporal ensemble a general property of action chunking, which is misleading — action chunking alone does not require temporal ensembling. — Fix: clarify temporal ensemble is an inference-time add-on, not intrinsic to chunking. — confidence: medium

---

## Ch21 Sim-to-Real

### Findings

- **[MED]** 21-sim-to-real-transfer.md:515 — "ETH Zurich,《Learning to Walk in Minutes Using Massively Parallel Deep RL》(2022)" — paper is by Rudin, Hoeller, Reist & Hutter; venue is **CoRL 2021** (proceedings appeared in PMLR late 2021/early 2022). More importantly, attributing solely to "ETH Zurich" omits the NVIDIA collaboration implicit in Isaac Gym tooling, though authors are ETH. The bigger issue: this paper is PPO in Isaac Gym, **not** a Teacher-Student paper. It's listed in Ch21 under Teacher-Student context, which misleads. The canonical Teacher-Student ANYmal paper is **Lee et al. 2020 "Learning quadrupedal locomotion over challenging terrain" (Science Robotics)** or **Miki et al. 2022 (Science Robotics)**. — Fix: replace with Lee et al. 2020 or Miki et al. 2022 for Teacher-Student; keep Rudin 2021 separately as the parallel PPO/DR reference. — confidence: high

- **[MED]** 21-sim-to-real-transfer.md:34 — Teacher-Student definition says Student uses "歷史觀測" via LSTM to implicitly infer physics. True for Miki/Lee ANYmal papers, but the chapter presents this as the **definition** of Teacher-Student. In general Teacher-Student / privileged learning (Chen et al. 2020 "Learning by Cheating"), the Student can be a one-step observation network; the LSTM/history trick is an orthogonal design choice specific to locomotion. — Fix: soften to "Student 常以歷史觀測 + RNN 隱式推斷特權資訊，但非必需 — 關鍵是 Student 只用可部署的觀測模態模仿 Teacher". — confidence: medium

- **[LOW]** 21-sim-to-real-transfer.md:429 — "MuJoCo 預設 0ms 致動器延遲, 真機 UR5 有 ~8ms 控制延遲" — UR5 real-time control latency is dominated by the **8ms (125 Hz) RTDE servoJ cycle**, plus variable ROS/network delay (often 20-40ms end-to-end). "8ms" is a lower-bound estimate that ignores typical pipeline delay; readers using this number for DR ranges may under-randomize. — Fix: change to "8-40ms 端到端延遲（取決於控制介面與網路）". — confidence: medium

---

## Ch22 Multimodal LLM Robotics

### Findings

- **[HIGH]** 22-multimodal-llm-robotics.md:99 — The "VLA Scaling 的經驗法則" formula $\text{Success Rate} \propto \log(\text{model size}) \cdot \sqrt{\text{data size}}$ is **presented as an established empirical law but is not from any published paper I can verify**. RT-2's scaling figures don't support this specific functional form — the paper shows discrete 5B vs 55B comparisons, not a $\log \cdot \sqrt{}$ fit. This is the exact kind of plausible-sounding hallucinated formula to flag. — Fix: remove the formula or mark it explicitly as "示意（非正式）" and back it with RT-2's actual reported numbers only. — confidence: high

- **[MED]** 22-multimodal-llm-robotics.md:119 — "模型 5B → 55B 參數: zero-shot 新物體泛化提升 ~3x" — RT-2 paper (Table/Fig comparing RT-2-PaLI-X 55B vs RT-2-PaLM-E 12B and smaller) reports improvements on emergent tasks, but the specific "~3x zero-shot on new objects" number doesn't cleanly match any headline figure I can verify. Generalization gains are typically reported as absolute percentage-point deltas (e.g. +20-30pp), not multiplicative 3x. — Fix: either remove the "3x" or cite the exact RT-2 Table/Fig and metric being referenced. — confidence: medium

- **[MED]** 22-multimodal-llm-robotics.md:432 — "OpenVLA (Stanford, 2024)" — OpenVLA is a **Stanford + UC Berkeley + TRI + MIT + Google DeepMind** collaboration led by Kim et al. Attributing solely to Stanford understates; the project and model are co-led and the base model is Llama 2 7B (Meta), which the description at :432 correctly notes. — Fix: "OpenVLA (Stanford + UC Berkeley + TRI, 2024)". — confidence: high

- **[LOW]** 22-multimodal-llm-robotics.md:32 — "RT-2 用 256 bins 離散化每個維度" — RT-2 does use 256 bins per action dimension, but it **reuses the least-used 256 tokens from the pretrained vocabulary** (not adds new tokens), which is the key trick allowing Transformer reuse. Chapter correctly states the bin count but misses the vocabulary-reuse detail that's a common follow-up interview question. — Fix: add "並重用 VLM vocabulary 中使用頻率最低的 256 個 token 作為 action token" as a parenthetical. — confidence: high
