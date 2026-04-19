# 當前任務

<!-- Claude Code 每次 session 開始時讀取這個檔案 -->
<!-- 如果為空，從 backlog.md 取最高優先級任務 -->

## TASK-20260418-05: 批量生成章節內容（產業接軌版 / 2026-04-18 方針定版）

### 方針（硬性，定版）
- 使用者 2026-04-18 校準定版：**廣度優先 + 精確定義 + 情境推理鏈 + 面試講點 + 閉環定位 + 物理直覺**
- **留**：數學公式（配物理意義）、面試角度（真 talking points）、情境題（完整推理鏈答案）
- **不做**：完整數學推導、手刻底層程式碼、含糊的「討論方向」
- 詳見 `CLAUDE.md` 的「使用者背景與內容哲學」與 `pipeline/templates/chapter_prompt.md`

### 每章 NotebookLM 查詢流程（5 次 / 章）

1. **精確定義 + 閉環定位**：兩三句白話定義、輸入/輸出/下游、在感知→規劃→控制的節點
2. **核心數學 / API + 物理意義**：最少夠用的公式或介面，每條配一句物理意義（不推導，但要留公式）
3. **直覺 + 視覺場景 + 常見誤解**：類比、模擬器可觀察現象、2–3 個真實工程師會犯的陷阱
4. **情境題 + 推理鏈**：面試 / 實戰會問的情境題（≥ 3 題）+ 完整推理鏈答案（遇到 X → 先看 … → 用 Y → 因為 Z → 避開 W）
5. **面試 talking points + 延伸關鍵字**：3–5 個面試講點、延伸閱讀、套件 / 模擬器 demo

素材存 `pipeline/extracted/embodied-robotics/{slug}.md`，合成用 `pipeline/templates/chapter_prompt.md` 生成 `content/embodied-robotics/{nn-slug}.md`，跑 `validate.sh` → yaml status → generated。

### 執行步驟（按章節推進）

- [x] 舊方針素材存檔：Chapter 01 Q1 深推導版存在 `pipeline/extracted/embodied-robotics/01-cpp-memory-optimization.md`（標註為素材庫）
- [x] ✅ CLAUDE.md v1：加入內容哲學（廣度優先）
- [x] ✅ CLAUDE.md v2：校準為**產業接軌版**（保留面試講點 / 數學 / 情境題）
- [x] ✅ 改寫 `pipeline/templates/chapter_prompt.md` 為產業接軌版（5 次查詢流程、情境題附完整推理鏈、面試真 talking points）
- [x] ✅ 改寫首頁 PILLARS：I 講得清楚才算懂 / II 情境推理鏈 / III 閉環中定位
- [x] ✅ 使用者 2026-04-18 確認方向；加「漸進式揭露」— 深入細節用 `<details><summary>深入：…</summary>` 摺疊
- [x] ✅ CLAUDE.md 與 `chapter_prompt.md` 加入漸進式揭露規範（至少 2 個進階摺疊塊）
- [x] ✅ Chapter 07 試產完成：
  - 5 次 NotebookLM 查詢（精確定義 / 核心數學 / 直覺誤解 / 4 情境題 / 面試講點）全部成功
  - 原始 extraction 存 `pipeline/extracted/embodied-robotics/07-forward-kinematics-dh.md`
  - 最終章節 `content/embodied-robotics/07-forward-kinematics-dh.md`（2 進階摺疊 + 4 情境題摺疊 + 60 個 KaTeX math blocks）
  - `quality/validate.sh` 全過、`npm run build` 綠
  - yaml `07-forward-kinematics-dh.status` → `generated`
- [x] ✅ 使用者 review Ch07 通過；新增需求：i18n 中英雙語 + 浮動切換按鈕 + scroll 位置保持
- [x] ✅ i18n infra 完成：`locales: ['zh-Hant', 'en']`、22 EN scaffold、Ch07 完整英譯
- [x] ✅ 浮動 `LocaleToggle` FAB（fixed bottom-right）取代 navbar dropdown；375/768/1440 全 visible
- [x] ✅ scroll 位置保持：head blocking script + sessionStorage ratio，round-trip 誤差 < 2%
- [x] ✅ 元件國際化：`src/lib/strings.ts`（ProgressToggle / ProgressDashboard 全部 locale-aware）
- [x] ✅ build 綠 + E2E 6 項全過
- [ ] 繼續按序生成 Chapter 01 → 22（Ch07 已完成，按 01, 02, 03, 04, 05, 06, 08, 09… 順序）

### 停止條件
- NotebookLM 429 / quota
- 使用者要求暫停
