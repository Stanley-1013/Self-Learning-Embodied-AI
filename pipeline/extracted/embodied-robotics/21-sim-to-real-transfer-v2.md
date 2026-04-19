# Extracted v2: Sim-to-Real 遷移 (session e5b2a3a5, 3 queries)
## Q1: 7 精確定義 + 閉環定位
- Reality Gap 三層面：物理動力學/視覺渲染/感測器噪聲
- DR：E_ξ[R(π,ξ)] 在隨機分佈上最佳化；物理DR vs 視覺DR
- ADR：自動擴展隨機化邊界（基於智能體表現）= 天然課程學習
- SysID：激勵軌跡+最小平方法反推參數；和DR互補（基準+擾動）
- DA：GAN/對抗特徵對齊（部署時適配 vs DR 訓練時泛化）
- Residual Learning：u = u_model + u_residual_NN（90%解析+10%NN殘差）
- Teacher-Student：sim特權教師→真機受限學生（知識蒸餾+資訊瓶頸）
- 閉環：訓練Sim→遷移技術→真機部署的過濾器
## Q2+Q3: 方法論+直覺+5誤解
- DR數學框架+物理意義（讓真實成為訓練分佈子集）
- SysID持續激勵+掃頻vs隨機
- Residual架構優勢（90%物理+10%NN）
- Teacher-Student具體流程（特權→受限）
- DR/SysID/DA三策略對比表
- 3類比：DR=極端天氣練車、SysID=調校模擬器、DA=美圖秀秀
- 5誤解(詳)：高保真≠消除gap/DR越大越好(保守崩)/只解視覺(動力學更致命)/SysID一次就夠(漂移)/Student≠Teacher(瓶頸)
## Q4+Q5: 4情境題+5面試+5延伸+4閱讀
- 情境1：MuJoCo 95%→UR5 30% → 查延遲→查摩擦→查噪聲
- 情境2：DR[0.1,2.0]過度保守 → SysID鎖基準+ADR動態擴展
- 情境3：Unity→工業相機視覺差異 → 視覺DR+CycleGAN DA+語意Mask降維
- 情境4：5種物體泛化抓取 → URDF+DR+Teacher-Student(LSTM隱式推斷物理參數)
- 5 points：DR+SysID混合/Residual工程優勢/Teacher-Student瓶頸/動力學>視覺優先級/ADR課程學習
- 5延伸：ADR/Sim2Sim/Digital Twin/Mixed Reality/Neural Sim
- 4閱讀：《面試題》Ch9/Survey/OpenAI Rubik's Cube/ETH Walking Minutes
