# Extracted: C++ 記憶體模型與原子操作
<!-- 提取時間: 2026-04-19 -->
<!-- 來源 Notebook: 0502daf4-1648-4456-bfd4-f1dee3875753 -->
<!-- NotebookLM session: fe23a257 (Q11-Q13) -->

## Q1 — 精確定義 + 閉環定位
- Memory model：modification order = 所有 thread 對同一物件修改的全局一致順序
- Happens-before：A 先行於 B → A 的影響對 B 可見；靠 synchronizes-with 跨 thread
- 6 ordering levels: relaxed(原子性only) / consume(data-dep only, deprecated) / acquire(後面不往前排) / release(前面不往後排) / acq_rel(雙向) / seq_cst(全局一致)
- atomic<T>：硬體指令實現原子性，免鎖；lock-free 支援 scalar types + 小 UDT
- CAS：weak 允許 spurious failure（必須迴圈）；strong 只在值不同時 false
- atomic_flag：唯一保證 lock-free（只有 set/clear，無 copy/assign）
- 閉環：acquire-release 在 lock-free 控制↔規劃通訊保證資料可見性；ARM 弱排序 seq_cst 代價極大

## Q2+Q3 — 機制 + 直覺 + 誤解
- Acquire-release 例：producer data=42 + flag.store(release) / consumer flag.load(acquire) + read data → happens-before 保證
- Relaxed 適合：獨立計數器、統計、進度（原子性夠、不需跨變數順序）
- seq_cst 代價：ARM 需大量 dmb barrier；x86 store 天生 release 但 store-load 仍需 mfence
- Fence 是全局屏障（vs per-variable）：適合多個 relaxed 讀寫後統一 release
- ABA 問題：CAS 只比位址，無法察覺值曾變回；解法 tagged pointer / hazard pointer / split ref count
- 類比：release=封信寄出、acquire=拆信讀；seq_cst=統一交卷順序改、relaxed=各科老師獨立改
- 5 誤解：seq_cst ARM 上不免費；relaxed≠不安全；volatile≠atomic；x86 不完全免 barrier；weak CAS 必須迴圈

## Q4+Q5 — 3 情境題 + 面試 + 延伸
- 情境1 SPSC ring buffer：acquire-release on index, relaxed on payload internal writes → happens-before via index
- 情境2 lock-free stack segfault：確診 ABA（同位址重用）→ hazard pointer / split ref count / tagged pointer
- 情境3 ARM 4核 3-sensor fusion：per-sensor double buffer + pointer swap (release/acquire) → wait-free snapshot
- 3 talking points：拒絕無腦 seq_cst、ABA + memory reclamation、CAS weak spurious failure
- 延伸：Hazard Pointer/RCU、Sequence Lock、MESI Cache Coherence
- 閱讀：《併發》Ch5 memory model → Ch7 lock-free DS
