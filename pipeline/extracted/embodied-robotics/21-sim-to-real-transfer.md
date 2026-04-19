# Extracted: Sim-to-Real (session fe23a257 Q27-E)
- Reality Gap：仿真vs真實物理+感測器差異；DR：隨機化參數強健；DA：GAN對齊分佈；SysID：反算物理參數
- 閉環：RL訓練→真機部署的過濾器
- 直覺：極端環境練功回現實覺得輕鬆；3誤解：高保真消除gap/DR越大越好(過保守)/只解視覺(物理更致命)
- 情境：真機力矩錯→SysID/未知地形→DR+MAML/視覺失效→CycleGAN DA
- Points：SysID+DR混合；延伸：Hybrid Reality Training；閱讀：Survey
