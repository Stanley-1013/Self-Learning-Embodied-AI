# Extracted v2: 深度強化學習 PPO SAC DDPG (5 次專屬查詢)
<!-- session: 17f29f2a, Q1-Q5 -->

## Q1 — 精確定義 + 閉環定位
- DQN：NN 近似 Q-table + Experience Replay(打破時序相關) + Target Network(穩定目標)
- DDPG：連續動作空間 Actor-Critic + OU 噪聲探索；確定性策略
- TD3：解 DDPG Q 值過估計；雙 Critic / 延遲更新 Actor / 目標策略平滑化
- SAC：最大熵框架(鼓勵多樣化探索) + 隨機策略 + 溫度 α 自動調節；比 DDPG 穩定
- PPO：Clipped Surrogate Objective(信任域裁剪替代 TRPO KL 約束) + GAE(bias-variance trade-off)
- On vs Off-policy：能否重用歷史資料；Off=樣本效率高但不穩、On=穩定但樣本低效
- Actor-Critic：Actor=策略網路輸出動作、Critic=價值網路打分；比純 PG 方差低
- 閉環：端到端策略 π(a|s) 取代手寫狀態機；訓練 Sim → 部署 Real(Sim-to-Real)
- 和傳統控制：DRL 做高階決策 → 底層 MPC/PID 做硬即時追蹤+安全
- 一句話：「DRL 讓機器人在 sim 裡百萬次試錯淬鍊出感測器→馬達的肌肉記憶」

## Q2 — 核心數學 / API
- Policy Gradient：∇J = E[∇log π · Q] — 按表現打分調整機率
- PPO Clip：L = E[min(r·A, clip(r,1-ε,1+ε)·A)] — 限制更新步長在信任區間
- SAC：J(π) = E[Σ r + αH(π)] — 熵項鼓勵探索
- TD error：δ = r + γV(s') - V(s) — 時序差分自舉
- GAE：Â = Σ(γλ)^l δ_{t+l} — λ 控制 bias-variance
- 工具鏈：SB3/Isaac Gym/MuJoCo/PyBullet/CleanRL/wandb/TensorBoard/Gymnasium
- PPO 超參：clip=0.2, lr_actor=1e-3, lr_critic=1e-2, epochs=5, γ=0.98, λ=0.95；SAC α 自動調節

## Q3 — 直覺 / 誤解
- 4 類比：Actor-Critic=球員+教練、PPO clip=安全繩、SAC 最大熵=探索家、Experience Replay=翻舊筆記
- 觀察：clip ratio≈1→lr太小/探索不夠；SAC entropy 先高後降維持微小；reward 震盪→reward bug/lr太大；value loss>>policy loss→Critic 評估困難(POMDP/稀疏獎勵)
- 6 誤解(詳細)：DRL不需調參/直接真機/稀疏reward就行/SAC永遠好/off-policy省時/DRL一定比PID好

## Q4 — 4 情境題(詳細推理鏈)
1. 四足不平地形 → SAC(最大熵+高樣本效率)；排除 DDPG(確定性策略探索差)
2. PPO 5000 epoch 不收斂 → 查 reward shaping bug → 查超參(lr/clip) → 查 value loss(觀測不足)
3. Peg-in-hole reward → 距離稠密獎勵+姿態對齊+接觸力L2懲罰+成功稀疏大獎；防 reward hacking(抖動騙分)
4. Sim 高速甩手部署危險 → 軟約束(動作L2/jerk懲罰) + 硬約束(CBF投影回安全邊界) + 分層控制(RL規劃+MPC底層)

## Q5 — 5 Talking points / 5 延伸 / 4 閱讀
- 5 points：PPO vs SAC 選型/Reward shaping+hacking防禦/Actor-Critic方差降低/Sim-to-Real分層部署/CBF硬約束 vs reward軟約束
- 5 延伸：HER/Curriculum Learning/World Model(DreamerV3)/MARL(CTDE)/Offline RL
- 4 閱讀：《面試題》Ch4.3+12.3 → Ch9 Sim2Real → PPO+SAC 原論文 → 《多模態大模型》Ch6 RLHF
