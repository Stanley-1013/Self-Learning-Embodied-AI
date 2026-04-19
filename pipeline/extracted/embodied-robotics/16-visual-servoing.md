# Extracted: 視覺伺服控制與特徵點提取 (session fe23a257 Q25-B)
- IBVS：像素空間特徵誤差→控制量；PBVS：3D重建→笛卡爾誤差
- Interaction Matrix(Image Jacobian)：2D特徵速度↔6-DoF相機速度映射
- Eye-in-Hand(末端精密) vs Eye-to-Hand(全局視野)
- 特徵：點(SIFT/ORB)、線(Canny)、矩(Hu moments)
- 閉環：感知→控制最短反射弧（跳過完整建圖+規劃）
- 直覺：IBVS=穿針線（看相對位移直接調）
- 3誤解：PBVS 永遠好(依賴標定)、IBVS 不需標定(畸變大仍崩)、特徵越多越好(拖慢+奇異)
- 情境：PCB插孔→IBVS(2D平面特徵豐富) / 180°旋轉後退→2.5D混合伺服 / 15fps vs 100Hz→EKF異步插值
- Points：交互矩陣奇異規避、手眼標定誤差傳遞、異步延遲補償；延伸：2.5D混合/事件相機/端到端NN；閱讀：Ch7.1+Ch7.4
