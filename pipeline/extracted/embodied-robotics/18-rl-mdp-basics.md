# Extracted: RL 基礎 MDP Q-Learning (session fe23a257 Q26-B)
- MDP五元組(S,A,P,R,γ)；Bellman方程；V vs Q函數；Policy π；On vs Off-policy
- Q-Learning：Q(s,a)←Q(s,a)+α[r+γmax Q(s',a')-Q(s,a)]；ε-greedy
- 閉環：決策控制層，替代手寫If-Else狀態機
- 直覺：訓練小狗給零食；3誤解：直接真機(損壞)/Q-table scale(維度災難→DQN)/reward不重要(稀疏不收斂)
- 情境：抓取reward設計(分層稠密)/大狀態空間(DQN+經驗回放+固定目標網路)/reward hacking(能量懲罰+IRL)
- Points：退火探索/HER(稀疏獎勵)/策略梯度選型(SAC/PPO)；延伸：PER/Double DQN/HRL；閱讀：Ch4.1+4.2
