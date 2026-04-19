# Extracted: C++ 記憶體管理與效能優化
<!-- 提取時間: 2026-04-19 -->
<!-- 來源 Notebook: 0502daf4-1648-4456-bfd4-f1dee3875753 -->
<!-- NotebookLM session: fe23a257 (5 queries) -->

## Q1 — 精確定義 + 閉環定位
- Stack：編譯器自動配置/釋放，零執行期開銷；1MB/thread，4096 threads 耗盡 32-bit 4GB
- Heap：new/malloc 動態請求；引發鎖競爭、系統呼叫、碎片合併；pool 4ms vs malloc 64ms (15×)
- RAII：資源獲取即初始化，建構取得/解構釋放，scope 綁定
- Smart pointers：unique_ptr 零成本獨佔；shared_ptr atomic 引用計數（昂貴）；weak_ptr 觀察不擁有
- Memory pool：固定大小區塊 free list，O(1) 分配/釋放，無碎片
- Cache line 32/64 bytes；false sharing = 不同 thread 改同 line 不同變數 → cache ping-pong
- 閉環定位：所有節點的底層基礎設施；控制 1kHz 禁 heap；感知需 zero-copy；ROS 2 intra-process 用 unique_ptr+move

## Q2 — 核心機制 / API / 物理意義
- unique_ptr assembly 等同 raw pointer（-O2+）
- shared_ptr make_shared 一次配置 vs new 兩次（物件+控制塊）
- weak_ptr.lock() atomic 嘗試加強引用計數
- Fixed-block pool：free list 用區塊前幾 bytes 存 next 指標（零額外空間）
- ROS 2 real-time pattern：初始化 reserve + LoanedMessage + unique_ptr move
- alignas(64) 防 false sharing；SIMD 要 16/32/64 對齊
- SoA vs AoS：感知大資料用 SoA（連續屬性、Cache 友好、SIMD 化）
- 非對齊存取 = 2× 時間
- 工具：Valgrind/Memcheck、perf/PMU、ASan、Cachegrind/Callgrind

## Q3 — 直覺 / 視覺 / 誤解
- 類比：Stack=便條紙疊、Heap=倉儲、RAII=旅館插卡取電、unique=車鑰匙、shared=借閱卡、pool=專屬停車場
- 控制迴路隱藏 new → jitter spike；感知無 zero-copy → Cache miss + page thrashing
- ROS 2 topic hz 不穩 → 懷疑 callback heap alloc 或 shared_ptr atomic 競爭
- 4 誤解：shared_ptr 非免費（atomic barrier）；vector push_back 觸發 realloc；false sharing 多核機器人一樣中招；std::string 拼接可慢 170×

## Q4 — 4 情境題 + 推理鏈
1. ROS 2 傳 PointCloud2：1對1 用 unique_ptr+move 零拷貝；1對N 用 const shared_ptr
2. 1kHz 控制偶發 2-5ms spike：perf/eBPF 找 malloc → reserve/array/pool 取代
3. 4-node 感知 pipeline 記憶體設計：Component 容器化 + intra-process + Object Pool + SoA + 對齊
4. RL 30Hz GPU → 1kHz 控制：Triple Buffer/atomic pointer swap + alignas(64) 防 false sharing + CUDA Pinned Memory

## Q5 — 面試 talking points / 延伸 / 閱讀
- 4 talking points：Heap 致命傷、Cache Locality、Memory Pool O(1)、Zero-copy unique_ptr
- 延伸：jemalloc/tcmalloc、std::pmr、perf+FlameGraph、Lock-free (boost::lockfree)、CUDA Pinned Memory
- 閱讀：《性能優化》Ch6+13 → 《併發編程》Ch5 → ROS 2 Real-time docs
