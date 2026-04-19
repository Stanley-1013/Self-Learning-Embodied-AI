# 任務待辦清單（Backlog）

優先級：P0 = 必須先做 / P1 = 核心功能 / P2 = 增強體驗

---

## TASK-20260418-05: 批量生成章節內容
- **優先級**: P1
- **類型**: content
- **輸入**: 
  - `pipeline/sources/{book-slug}.yaml` 中 status 為 pending 的章節
  - NotebookLM notebook 已包含所有來源
- **輸出**: 完整章節 Markdown，存入 `content/{book-slug}/`
- **驗收**: KaTeX 渲染正確，每章有 ≥3 練習題，所有必要段落完整
- **預估**: L（自動循環處理所有章節）
- **執行方式**:
  1. 讀取 yaml，找到第一個 status 為 pending 的章節
  2. 按 `pipeline/notebooklm_workflow.md` 提取該章節內容
  3. 按 `pipeline/templates/chapter_prompt.md` 生成最終 Markdown
  4. 執行 `quality/validate.sh` 驗證
  5. 更新 yaml 中該章節 status
  6. 繼續下一個 pending 章節，直到全部完成

## TASK-20260418-06: 測驗元件
- **優先級**: P1
- **類型**: component
- **輸入**: Docusaurus 專案已初始化
- **輸出**: `src/components/Quiz.tsx`
- **驗收**: 支援選擇題，答案可摺疊，記錄正確率
- **預估**: S

## TASK-20260418-07: 第一個互動式視覺化
- **優先級**: P2
- **類型**: component
- **輸入**: 需要視覺化的具體概念（讀完前 3 章後決定）
- **輸出**: React + Three.js/D3 互動元件（可搭配 `@remotion/player` 嵌入動畫）
- **驗收**: 可拖拉操作，幫助理解特定概念
- **預估**: L
