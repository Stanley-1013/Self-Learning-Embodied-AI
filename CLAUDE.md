# 學習系統 — Claude Code 工程指引

個人學習應用，結構化閱讀機器人學、具身智能、底層演算法與數學。
技術棧：Docusaurus + React + TypeScript + KaTeX + Remotion。

## 使用者背景與內容哲學（所有章節必守）

### 使用者背景
- 臺大生機系大四，紮實軟硬韌體整合 + 硬體背景
- 即將進 NUS Robotics 碩士，專攻 Embodied AI
- 知識缺口：機器人學基礎數學建模 / 進階控制理論 / 演算法基礎
- **終極目標**：能投入具身智能業界。整份資源為「產業接軌」服務

### 核心心法（寫內容時的硬性原則）

**概念地圖優先、面試場景導向、物理直覺保留、數學看得懂不用手算**：

1. **能清楚講出來** — 每個概念要能用兩三句白話精確定義，不是含糊「大概是 …」；面試或與 AI 協作時能立刻用
2. **情境推理鏈** — 教材核心是「遇到 X 情況 → 會想到 Y 工具 → 因為原則 Z → 要避開陷阱 W」的完整鏈條，不是公式套用練習
3. **物理直覺 + 閉環定位** — 每個概念都要能視覺化，並明確標出它在**感知 → 規劃 → 控制**閉環的哪個節點（輸入/輸出/下游）
4. **數學看得懂 / 不手推** — 保留最少夠用的公式與 API，每條帶**一句物理意義**；不做完整推導、不要求讀者手算
5. **面試講點是核心** — 保留「面試角度」段作為產業接軌入口，寫真實會被問的 key talking points，目標是讓讀者能在 2 分鐘內把概念講清楚

### 為什麼這樣定位
具身智能是當前最新、AI 尚未漂亮搞定的領域。使用者需要的是**能跟 AI 協作時下得出指令、看得懂輸出**的概念框架；不是論文級推導訓練，也不是工具教學手冊。

### 每章必做
1. **精確定義**：兩三句講清楚這個概念是什麼（不含糊）
2. **閉環定位**：在感知 → 規劃 → 控制哪個節點，輸入是什麼、輸出是什麼、下游誰會用
3. **物理直覺**：類比 / 視覺比喻 / 模擬器（Gazebo / Isaac Sim / MuJoCo）可觀察到的現象
4. **最少夠用的數學/API**：保留核心公式，每條後面跟一句物理意義；不做推導但讀者要能看懂
5. **情境題**（≥ 3 題）：面試 / 實戰會問的「遇到 X 你怎麼分析？」問題，附完整推理鏈答案
6. **面試角度**：3–5 個真 talking points，寫「面試時要強調什麼、為什麼這點最關鍵」
7. **常見誤解**：2–3 個真實會犯的陷阱 + 澄清
8. **延伸關鍵字 / 閱讀**：套件、論文、模擬器 demo — 之後想深挖時 Google 什麼

### 寫作語氣
- 像資深工程師對學弟：簡明扼要，但不含糊
- 專有名詞保留英文（Jacobian、forward kinematics、VLA…），中文為主

### 漸進式揭露（重要：「由淺入深」的呈現方式）

主文保持廣度優先、產業接軌的精簡敘述；**深入細節** — 完整數學推導、完整程式實作、底層實作細節、邊界情況分析 — **用 `<details>` 摺疊**起來，預設收合，需要深入時才展開。

這讓同一份章節同時服務「我想快速建立概念地圖」與「我想深挖這題」兩種閱讀模式。

每章至少安排 **2 個進階摺疊塊**（不含情境題的 `<details>`），建議位置：
- `核心概念` 的公式之後：`<details><summary>深入：完整推導</summary> …</details>`
- `實作連結` 的 code 骨架之後：`<details><summary>深入：完整實作（C++ / Python）</summary> …</details>`
- 其他可加的深入塊：演算法複雜度分析、和其他方法的比較、特殊情況處理、底層實作細節

摺疊塊的 `summary` 務必以「**深入：**」為前綴，方便讀者辨識。

## 核心架構

```
content/{book-slug}/{nn-chapter}.md   ← 所有學習內容（Markdown + KaTeX）
pipeline/
  templates/  sources/  scripts/  extracted/   ← 內容生成管線
src/
  components/  pages/  theme/  css/  lib/
tasks/
  backlog.md  current.md  done.md
quality/
  checklist.md  validate.sh
```

## 任務系統

### 任務格式
```markdown
## TASK-{ID}: {簡短描述}
- **類型**: content | component | pipeline | fix
- **輸入**: 需要什麼資料/檔案
- **輸出**: 預期產出的檔案路徑
- **驗收**: 怎樣算完成
- **預估**: S / M / L
```

### Session 工作流程（自動循環）

完成一個任務後立即開始下一個，不需要等待人類指令。

```
loop:
  1. 讀取 tasks/current.md
  2. 若 current.md 無待辦：
     a. 從 backlog.md 取最高優先級任務移入 current.md
     b. 若 backlog 空 → 停止，通知「所有任務已完成」
  3. 執行 current.md 中的任務
     - 每完成一步立即在 current.md 標記 ✅
     - 涉及 NotebookLM 先讀 pipeline/notebooklm_workflow.md 和對應 yaml 檢查狀態
  4. 任務完成後：
     a. 執行 quality/validate.sh
     b. 將任務移至 tasks/done.md（附完成時間與產出/備註）
     c. 從 backlog.md 移除該任務
     d. 回到 step 1
```

### 停止條件
- backlog.md 清空
- 任務「輸入」需使用者決策
- validate.sh 報錯且無法自行修復
- 任務標記 `needs_human: true`

### 上下文恢復
每個 session 都是全新的。必要脈絡只靠：
- 本檔（規則）
- tasks/current.md（當前工作）
- 檔案本身（已完成的工作）
- pipeline/sources/*.yaml（NotebookLM 狀態）

絕不依賴上次對話記得什麼。

## 內容生成管線

1. **來源登記**：`pipeline/sources/{book-slug}.yaml`，格式見 `pipeline/sources/_example.yaml`。
2. **知識提取**（NotebookLM MCP）：所有操作冪等，依 yaml `status` 欄位推進；詳細流程見 `pipeline/notebooklm_workflow.md`。
3. **內容加工**：依 `pipeline/templates/` 中的 prompt 模板將 `pipeline/extracted/` 轉為 `content/{book}/{chapter}.md`。
4. **品質驗證**：`bash quality/validate.sh`；檢查項見 `quality/checklist.md`。

## 章節模板

完整 prompt 與範例見 `pipeline/templates/chapter_prompt.md`。每個章節必須包含：

- **Frontmatter**：`title`、`prerequisites`、`estimated_time`、`difficulty`、`tags`、`sidebar_position`
- **必要段落**（順序固定、名稱不變；內容遵循上面「內容哲學」）：
  - `你將學到` — 2–3 句：學完能**清楚講出**什麼、遇到什麼問題會想到這個工具
  - `核心概念` — 精確定義（兩三句能講清楚）+ **閉環定位**（輸入 / 輸出 / 下游誰會用）+ 最少夠用的數學或 API，**每條公式必配一句物理意義**（留公式，但不做推導）
  - `直覺理解` — 類比 / 視覺比喻 / 模擬器可觀察的現象（Gazebo / Isaac Sim / MuJoCo 場景建議）
  - `實作連結` — 真實工程情境（ROS 2 node、即時控制迴路、感知 pipeline 之類）+ code 骨架層級描述（function signature + 關鍵註解，不要求完整實作）
  - `常見誤解` — 2–3 個真實陷阱 + 澄清
  - `練習題` — **情境題 ≥ 3 題**：面試或實戰會問的「遇到 X 情況你會怎麼分析？用什麼工具？為什麼？要避開什麼？」，`<details>` 裡附**完整推理鏈答案**（不是含糊討論方向、是清晰的分析過程）
  - `面試角度` — **真 talking points**：3–5 個面試被問到這主題時要強調的關鍵點，每點寫清「為什麼這是重點、怎麼在兩分鐘內把這概念講清楚」
  - `延伸閱讀` — 論文、教科書章節、套件 / 模擬器 demo，每條附一句「為什麼值得看」
- **數學**用 KaTeX：行內 `$...$`、區塊 `$$...$$`；公式保留但**一律配物理意義註解**

`validate.sh` 檢查段落存在與 `<details>` 數量（≥ 3）。

## 元件與樣式

- **進度追蹤**：localStorage key `robotic-learning.progress.v1`，結構 `{ [pagePath]: { completed, completedAt, notes } }`；store 在 `src/lib/progress.ts`。
- **讀 localStorage 的元件**必須用 `<BrowserOnly>` 包（SSR 無 `window`）。
- **swizzle** 以 wrap 為主（`@theme-original/...`），集中放 `src/theme/`。
- **設計 token** 定義在 `src/css/custom.css`（`--ink` / `--paper` / `--accent`…），元件 CSS module 請沿用，不要硬編色值。
- **Remotion 動畫**放 `src/components/animations/`，用 `@remotion/player` 內嵌。

## 響應式設計（所有 UI 任務硬性要求）

- **斷點**：手機 `≤ 576px`、平板 `577–966px`、桌機 `≥ 967px`（對齊 Docusaurus `--ifm-mobile-*` / `--ifm-tablet-*`）。
- **驗收視口**：320 / 375 / 768 / 1024 / 1440。在 320px 不可出現橫向捲軸。
- **流體排版**：字級用 `clamp()`（例：`clamp(1rem, 0.9rem + 0.5vw, 1.35rem)`）；不硬編 px 字級。
- **網格**：卡片/目錄用 `display: grid; grid-template-columns: repeat(auto-fill, minmax(N, 1fr))`；固定欄數只用在有明確設計理由的地方，並附 `@media` fallback。
- **可點擊區域**：手機上 tap target 至少 44 × 44 px。
- **KaTeX 公式**：`.katex-display` 必須 `overflow-x: auto`，避免長公式撐破版面。
- **圖片**：長寬比用 CSS `aspect-ratio` 或明確 `width`/`height`，避免 CLS。
- **媒體查詢**：優先 `prefers-reduced-motion`（尊重使用者設定）、`prefers-color-scheme`（Docusaurus 已內建處理）。
- **驗證**：設計 pass 後至少手動檢 320 / 768 / 1280 三個視口；未驗證過不算完成。

## 命名慣例
- 書籍資料夾 `{nn}-{slug}`
- 章節檔案 `{nn}-{slug}.md`
- 元件 `PascalCase.tsx`（含對應 `.module.css`）
- 任務 ID `TASK-{YYYYMMDD}-{nn}`

## 常用指令
```bash
npm run dev          # 啟動開發伺服器（alias: npm start）
npm run build        # 建置（CI quality gate）
bash quality/validate.sh   # 驗證內容
```
