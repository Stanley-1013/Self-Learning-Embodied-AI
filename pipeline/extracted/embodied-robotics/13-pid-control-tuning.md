# Extracted: PID 控制原理與抗飽和實務
<!-- session: fe23a257, Q23 -->

## 定義
- P=現在誤差、I=過去累積、D=未來趨勢
- 連續：u = Kp*e + Ki*∫e dt + Kd*ė；啟動P主導、穩態I主導、快變D主導
- 離散：位置式(全值,易飽和) vs 增量式(Δu,天然無windup,適合步進/延遲)
- Integral windup：馬達飽和→I爆累→釋放時超調振盪
- Anti-windup：Clamping(限幅停積) / 積分分離(大誤差關I) / Back-calculation(反算衰減)
- Ziegler-Nichols：Ku+Tu→初值；只是起點(線性模型不含摩擦/背隙)
- Cascade PID：外環(位置)→中環(速度)→內環(電流)；內快外慢，由內而外調

## 閉環
- 控制最末端「最後一哩路」「脊髓反射」
- 輸入：誤差 e=期望-實際；輸出：控制量 u(力矩/電壓)；上游：IK/trajectory
- 一句話：「PID 把數學規劃變為精準的物理現實」

## 直覺 + 誤解
- P=盯車距踩油門、I=逆風慢加油、D=看煞車燈提前減速
- 4 誤解：D 越大越穩(雜訊放大)、無腦加 I(相位滯後)、PID 搞定一切(需動力學前饋)、不寫 anti-windup(卡死後燒馬達/砸飛)

## 3 情境題
1. 穩態誤差 → Ki+限幅；更好=重力/摩擦前饋+PD
2. 飽和後釋放超調 → integral windup → clamping anti-windup
3. 三環串級設計 → 電流(10kHz PI)→速度(1kHz PI)→位置(100Hz P)；由內而外調

## Talking points / 延伸 / 閱讀
- 3 points：anti-windup 防禦性編程、前饋+回饋分離、時滯系統微調(降D/增量式)
- 延伸：Fuzzy PID(變負載)、Smith Predictor(延遲補償)、Impedance Control(力位混合)
- 閱讀：《面試題》Ch1.1 PID 基礎、Ch6.2 經典控制
