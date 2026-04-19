# NotebookLM 內容提取工作流程

## 概述
此文件定義了從 NotebookLM 提取結構化知識的標準流程。
Claude Code 透過 NotebookLM MCP 工具執行這些操作。

## ⚠️ 冪等性規則（最重要）

**每一步執行前必須先檢查狀態，已完成的步驟絕不重複執行。**

狀態記錄在兩個地方：
1. `pipeline/sources/{book-slug}.yaml` 的 `notebook_id` 和各來源/章節的 `status` 欄位
2. `pipeline/extracted/{book-slug}/` 目錄下是否已存在對應的提取檔案

Claude Code 在執行任何 NotebookLM 操作前的標準檢查流程：
```
1. 讀取 pipeline/sources/{book-slug}.yaml
2. 檢查 notebook_id 是否已存在 → 有就跳過建立
3. 檢查每個 source 的 status → ingested/extracted 就跳過
4. 檢查 pipeline/extracted/{book-slug}/{chapter}.md 是否存在 → 有就跳過提取
5. 只執行 status 為 pending 的項目
6. 完成後立即更新 yaml 中的 status 欄位
```

## Step 1: 建立主題 Notebook
`[Study] {book-slug} - {book-title}`

建立後**立即**將 notebook_id 寫回 yaml：
```yaml
notebook_id: "abc123"  # ← 建立後馬上回填
```

## Step 2: 匯入來源

**前置檢查**：逐一檢查每個 source 的 `status` 欄位。
- `pending` → 執行匯入，完成後更新為 `ingested`
- `ingested` 或 `extracted` → **跳過**

根據 `pipeline/sources/{book-slug}.yaml` 中登記的來源匯入。

## Step 3: 結構化提取

**前置檢查**：檢查 `pipeline/extracted/{book-slug}/{chapter-slug}.md` 是否存在。
- 檔案已存在 → **跳過**
- 檔案不存在且對應來源已 `ingested` → 執行提取
- 提取完成後更新章節 status 為 `extracted`

對每個**待提取**章節，向 NotebookLM 發送以下提取 prompt：

### 提取 Prompt 模板

```
針對「{topic}」這個主題，請從已上傳的來源中提取以下資訊：

1. **核心定義與公式**: 列出所有關鍵定義、定理、和公式，保留完整數學符號
2. **推導關鍵步驟**: 最重要的推導過程，標記每一步的物理/幾何意義
3. **不同來源的觀點差異**: 不同教科書/論文對這個概念的解釋角度有何不同？
4. **前置知識清單**: 理解這個主題需要先知道什麼？
5. **應用場景**: 這個概念在哪些實際系統/演算法中被使用？
6. **常見考點**: 根據來源中的習題或討論，這個主題最常被怎麼考？

請引用具體來源，標記出自哪本書/論文的哪個章節。
```

## Step 4: 儲存提取結果

輸出存入 `pipeline/extracted/{book-slug}/{chapter-slug}.md`

格式：
```markdown
# Extracted: {topic}
<!-- 提取時間: {timestamp} -->
<!-- 來源 Notebook: {notebook_id} -->

## 核心定義與公式
...

## 推導關鍵步驟
...

## 觀點差異
...

## 前置知識
...

## 應用場景
...

## 考點整理
...
```

## Step 5: 交給 Chapter 生成

提取結果作為 `chapter_prompt.md` 模板中 `{extracted_content_from_notebooklm}` 的輸入，
由 Claude API 或 Claude Code 直接生成最終章節。

## 注意事項

- NotebookLM 的強項是跨文件檢索和摘要，善用它做「綜合多個來源」的提取
- 數學公式在 NotebookLM 中可能格式不完美，需要在 Chapter 生成階段修正
- 每次提取後檢查是否有遺漏的重要概念，必要時追問
- 保留提取的原始輸出，方便日後追溯和更新
