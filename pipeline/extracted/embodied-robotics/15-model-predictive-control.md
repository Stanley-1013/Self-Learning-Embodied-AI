# Extracted: MPC 與軌跡追蹤 (session fe23a257 Q25-A)
- MPC 三要素：預測模型+代價函數+約束；滾動時域(只執行第一步)
- vs PID：MPC=看未來+處理約束；PID=看過去現在
- 線性 MPC(QP,OSQP) vs NMPC(SQP/IPOPT,更精確但慢)
- 閉環：替代/包裹 PID 高階追蹤；vs Ch11 離線軌跡=在線重規劃
- 直覺：開車看前方N步；3誤解：能跑任意模型/只能線性/不需調參
- 情境：四足→LIPM+摩擦錐QP / 跑不進1kHz→warm start+稀疏化+顯式MPC+Learning MPC / Sim偏離→延遲環節+DOB
- Points：時域vs算力權衡、凸鬆弛、硬約束優勢；延伸：Neural ODE/DDP/CBF；閱讀：Ch5.4+Ch10.2
