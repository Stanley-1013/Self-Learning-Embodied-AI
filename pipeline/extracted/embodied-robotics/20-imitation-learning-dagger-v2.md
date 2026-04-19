# Extracted v2: 模仿學習與行為克隆 (session 31e15319 主題A)
## 6 精確定義
- BC：監督學習框架 min‖π(s)-a‖²；簡單但脆弱(i.i.d.假設)
- Distribution Shift：BC必然遭遇；compounding error 二次方累積
- DAgger：學生執行→專家糾偏→聚合訓練集→迭代；解distribution shift
- IRL：學reward而非action；歧義性(MaxEnt約束)
- GAIL：GAN思想 Generator=策略 Discriminator=區分專家vs機器
- Diffusion Policy：擴散模型生成動作序列；適合多模態動作分佈(避免MSE平均化)
## 閉環 + 公式
- 跳過reward design直接從demo學
- BC loss: min E[‖π(s)-a‖²]
- DAgger distribution mixing: D_i = D_{i-1} ∪ {(s, π*(s))}
- IL做warm start → RL做fine-tune
## 直覺+5誤解+4情境+5面試+5延伸+4閱讀
- BC=照食譜(偏離就崩)、DAgger=師傅旁邊糾正
- 5誤解：BC能泛化/IRL唯一解/demo越多越好(品質>量)/離線就夠(需在線)/Diffusion太慢
- 4情境：BC走偏→DAgger/少量demo→增強+sim/多模態動作→Diffusion/長horizon→分層IL
- 5 points：BC compounding error/DAgger核心機制/IRL歧義性/GAIL與GAN對偶/RL+IL混合
- 延伸：BCO/Offline IL/Constraint IL/Reward Hacking/Action Chunking
- 閱讀：《面試題》Ch8.5/DAgger論文/GAIL論文/Diffusion Policy論文
