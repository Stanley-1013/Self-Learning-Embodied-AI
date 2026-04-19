# Extracted v2: RL 基礎 MDP Q-Learning (3 queries, session aa29e664)

## Q1 — 8 精確定義 + 閉環定位
- MDP五元組(S,A,P,R,γ) 含機器人抓取具體例子
- Bellman方程：最優性原理遞迴 V*(s)=max_a[R+γΣP·V*]
- V(s) vs Q(s,a)：V=所有動作的加權期望、Q=特定動作的價值；V=max Q
- Policy π(a|s)：確定性 vs 隨機；機器人用隨機（探索+魯棒）
- On vs Off-policy：SARSA(on,保守安全) vs Q-Learning(off,激進最優)
- Q-Learning更新：Q←Q+α[r+γmax Q'-Q]，每項物理意義
- ε-greedy：不能純貪心(局部最優)、退火衰減、vs Boltzmann
- TD vs MC：bootstrapping(自舉) vs 完整回合；方差-偏差 trade-off
- 閉環：替代手寫規則→學出策略；和 PID/MPC 互補（RL高階+MPC底層）
- Q-table→DQN 維度災難引出 Ch19
- 一句話：「RL 是讓機器人在沙盒中試錯淬鍊出感測器→馬達的決策直覺」

## Q2 — 6 核心公式 + API + 超參數
- Bellman V*、Q*、Q-Learning update、SARSA update、Return G_t、Policy Improvement Theorem
- 每條物理意義（max=最優性/α=學習步長/γ=近視遠視/bootstrapping=自舉）
- Gymnasium API 流程：reset→step→render；Discrete vs Box
- Q-table 骨架(dict/numpy) + DQN 骨架(Replay+TargetNet+MainNet)
- 超參：α=1e-4~1e-3(DQN)/0.01~0.1(表格)；γ=0.95~0.99；ε=1.0→0.01 指數衰減

## Q3+Q4+Q5 — 直覺/誤解/情境/面試/延伸
- 4類比：MDP=棋盤、Q-table=答案對照表、ε-greedy=老餐廳vs新店、TD vs MC=每道菜評分vs吃完打總分
- 5誤解(詳)：直接真機(損壞)/Q-table連續空間(維度災難)/稀疏reward(不收斂)/Q-Learning永遠好(SARSA更安全)/γ=1(發散)
- 4情境題(詳)：抓取reward設計(分層稠密+安全+成功)/grid→6DoF崩(維度災難→DQN)/reward hacking(撞牆走捷徑→CBF)/SARSA vs Q-Learning懸崖(on-policy保守更安全)
- 5 points：Bellman遞迴/On vs Off安全/探索利用退火/Q-table→DQN動機/Reward shaping藝術
- 5延伸：HER/Curriculum Learning/Multi-step TD/Double Q-Learning/PER
- 4閱讀：《面試題》Ch4.1+4.2/《多模態大模型》Ch2.4.2/Sutton&Barto RL教科書
