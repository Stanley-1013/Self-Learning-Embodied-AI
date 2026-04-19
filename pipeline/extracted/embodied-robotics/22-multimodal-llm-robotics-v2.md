# Extracted v2: 多模態大模型與具身感知決策 (session 31e15319 主題B)
## 6 精確定義
- VLA(Vision-Language-Action)：統一架構(RT-2/Octo/OpenVLA)；vs純RL=汲取互聯網預訓練
- World Model：預測環境動力學(DreamerV3)；隱空間推演；model-based RL
- Foundation Model：預訓練→微調；零樣本泛化；像ImageNet預訓練
- End-to-End：省pipeline直接sensor→action；簡化但黑箱+數據重度依賴
- Grounding Problem：LLM語言理解 vs 物理世界落差（穿牆/無限臂長幻覺）
- Action Tokenization：連續動作離散化給Transformer自迴歸預測
## 閉環 + 架構
- 高階認知中樞(感知+決策統一)
- 三架構：LLM as Planner / VLA E2E / VLM+MPC分層
- 推理延遲：LLM 1-2Hz vs 控制100+Hz → 分層必然
## 直覺+5誤解+4情境+5面試+5延伸+4閱讀
- VLA=能看能聽有常識大腦、World Model=做白日夢預演
- 5誤解：LLM直接控制(太慢)/無幻覺(物理推理錯)/不需防護(CBF必要)/大模型萬能(PID更好)/端到端不需工程(大量infra)
- 4情境：模糊指令(LLM拆子任務)/推理慢(VLM+MPC分層)/未見物體(零樣本+Few-shot)/邊緣部署(量化+蒸餾+雲邊協同)
- 5 points：LLM-控制器接口/Grounding解法/Action Tokenization/World Model Sim-to-Real/安全兜底
- 延伸：Prompt Engineering/PaLM-E RT-X/Neural-Symbolic/RLHF for Robotics/BEV Fusion
- 閱讀：《面試題》Ch3.6+10.6/《多模態大模型》/RT系列+Octo論文
