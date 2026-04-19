# src/components

本目錄存放可複用的 React 元件。後續任務會填入以下檔案：

| 檔案 | 由哪個任務產生 | 用途 |
|------|---------------|------|
| `ProgressToggle.tsx` | TASK-20260418-02 | 每章底部的「標記完成」按鈕，寫入 localStorage |
| `ProgressDashboard.tsx` | TASK-20260418-02 | 進度總覽頁 (`/progress`) 顯示每本書的完成百分比 |
| `Quiz.tsx` | TASK-20260418-06 | 互動式測驗元件（選擇題、答案摺疊、記錄正確率） |
| `animations/*.tsx` | TASK-20260418-07+ | Remotion / Three.js 動畫與視覺化 |

## 約定

- 元件檔名 `PascalCase.tsx`
- Props 用具名 `interface`，不使用 `React.FC`
- 只在 `useEffect` 內讀取 `localStorage` / `window`（避開 Docusaurus SSR 階段）
- 不放業務邏輯，保持元件聚焦在 UI 與互動
