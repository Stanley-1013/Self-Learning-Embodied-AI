---
title: "範例章節 — KaTeX 渲染驗證"
sidebar_position: 1
prerequisites: []
estimated_time: 5
difficulty: 1
tags: ["meta", "verification"]
---

# 範例章節 — KaTeX 渲染驗證

## 你將學到
- 確認 Docusaurus 專案能正確啟動
- 確認 KaTeX 行內與區塊公式都能渲染
- 確認章節模板結構可用於後續內容

## 核心概念

這個章節只用來驗證渲染管線。實際內容會由 TASK-20260418-05 自動生成。

### 行內數學

歐拉恆等式：$e^{i\pi} + 1 = 0$，這是數學中最美的公式之一。

線性變換的形式為 $y = Ax + b$，其中 $A \in \mathbb{R}^{m \times n}$。

### 區塊數學

高斯分佈的機率密度函數：

$$
p(x; \mu, \sigma^2) = \frac{1}{\sqrt{2\pi\sigma^2}} \exp\left(-\frac{(x - \mu)^2}{2\sigma^2}\right)
$$

矩陣乘法展開：

$$
(AB)_{ij} = \sum_{k=1}^{n} A_{ik} B_{kj}
$$

剛體變換的齊次矩陣：

$$
T = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} \in SE(3)
$$

## 直覺理解

把 KaTeX 想像成一個「把 LaTeX 字串即時編譯成漂亮數學符號」的瀏覽器引擎。Docusaurus 在建置時把 Markdown 的 `$...$` 和 `$$...$$` 交給 `remark-math` 分析，再由 `rehype-katex` 轉成 HTML + MathML。

## 實作連結

當 TASK-20260418-05 批量生成章節時，就是依賴這套流程：
- Markdown frontmatter 提供 metadata（title、prerequisites、tags…）
- 章節主體用 KaTeX 寫數學推導
- `<details>` 摺疊練習題答案

## 常見誤解

- **誤以為需要手動載入 KaTeX JS**：本專案只載入 CSS，公式渲染在建置期完成，執行時不需 JS。
- **認為 `$` 符號會到處觸發公式模式**：`remark-math` 要求 `$` 緊貼公式內容（左側後面、右側前面不能有空白），純文字中的美元符號不會被誤判。
- **混用 `$` 與 MDX 表達式**：MDX 解析器對某些字元敏感，寫公式時若出現大括號要用 `$$` 區塊避開解析衝突。

## 練習題

<details>
<summary>Q1：驗證行內公式 $\sum_{i=1}^{n} i = \frac{n(n+1)}{2}$ 能否正確渲染</summary>

如果你看到漂亮的求和符號與分數，而不是原始的 LaTeX 字串，就代表 KaTeX 正常運作。
</details>

<details>
<summary>Q2：為什麼選 KaTeX 而不是 MathJax？</summary>

- KaTeX 渲染速度顯著更快（同步、無 JS 重排）
- bundle 較小
- 對大多數學術論文的語法支援已經足夠
- MathJax 的優勢在於完整 LaTeX 語法相容性，但機器人學 / ML 教材很少需要冷僻命令
</details>

<details>
<summary>Q3：如果有公式沒渲染出來，第一個排查步驟是什麼？</summary>

1. 檢查 `docusaurus.config.ts` 的 `docs.remarkPlugins` 和 `rehypePlugins` 都有掛 `remark-math` / `rehype-katex`
2. 開瀏覽器 DevTools 看 `katex.min.css` 有沒有從 CDN 載入（Network tab）
3. 確認 `$` 符號成對出現，且內容沒有跨行（除非用 `$$`）
</details>

## 面試角度

這只是範例章節，沒有面試相關內容。

## 延伸閱讀

- [KaTeX 官方支援語法](https://katex.org/docs/supported.html)
- [Docusaurus Markdown - Math Equations](https://docusaurus.io/docs/markdown-features/math-equations)
- [remark-math GitHub](https://github.com/remarkjs/remark-math)
