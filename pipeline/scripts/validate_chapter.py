#!/usr/bin/env python3
"""章節品質驗證 — Python 版本。

與 `quality/validate.sh` 等價，但可被其他腳本 import。回傳結構化結果
（errors / warnings），exit code 1 代表任一錯誤。

CLI 用法：
    python pipeline/scripts/validate_chapter.py [chapter.md ...]
    # 不帶參數時掃整個 content/ 目錄

程式用法：
    from pipeline.scripts.validate_chapter import validate_file, validate_many
    result = validate_file("content/example/01-intro.md")
    # result.errors / result.warnings / result.ok
"""
from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Iterable

REQUIRED_FRONTMATTER = (
    "title",
    "prerequisites",
    "estimated_time",
    "difficulty",
    "tags",
)

REQUIRED_SECTIONS = (
    "核心概念",
    "直覺理解",
    "實作連結",
    "常見誤解",
    "練習題",
    "面試角度",
)

MIN_QUIZ_COUNT = 3


@dataclass
class ValidationResult:
    path: Path
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors


def _extract_frontmatter(text: str) -> str | None:
    """回傳 frontmatter 區塊內容（不含 `---` 分隔符）；找不到回 None。"""
    match = re.match(r"^---\r?\n(.*?)\r?\n---\r?\n", text, re.DOTALL)
    return match.group(1) if match else None


def _section_headings(text: str) -> set[str]:
    return {m.group(1).strip() for m in re.finditer(r"^##\s+(.+)$", text, re.MULTILINE)}


def validate_file(path: str | Path) -> ValidationResult:
    p = Path(path)
    result = ValidationResult(path=p)
    try:
        text = p.read_text(encoding="utf-8")
    except FileNotFoundError:
        result.errors.append(f"檔案不存在：{p}")
        return result

    # 1. Frontmatter 結構
    frontmatter = _extract_frontmatter(text)
    if frontmatter is None:
        result.errors.append("缺少 frontmatter（檔案開頭需用 `---` 包起來）")
        return result

    for field_name in REQUIRED_FRONTMATTER:
        if not re.search(rf"^{field_name}\s*:", frontmatter, re.MULTILINE):
            result.errors.append(f"缺少 frontmatter 欄位：{field_name}")

    # 2. 必要段落
    sections = _section_headings(text)
    for section in REQUIRED_SECTIONS:
        if section not in sections:
            result.errors.append(f"缺少段落：## {section}")

    # 3. 練習題數量（以 <details> 計）
    quiz_count = len(re.findall(r"<details\b", text))
    if quiz_count < MIN_QUIZ_COUNT:
        result.errors.append(
            f"練習題不足：{quiz_count} 題（需要 ≥ {MIN_QUIZ_COUNT}）"
        )

    # 4. KaTeX `$` 配對（啟發式：總數應為偶數）
    dollar_count = text.count("$")
    if dollar_count % 2 != 0:
        result.warnings.append(
            f"KaTeX `$` 符號數量為奇數（{dollar_count}），可能未配對"
        )

    # 5. 軟性段落（只警告）
    if "## 你將學到" not in text:
        result.warnings.append("缺少「你將學到」段落")
    if "## 延伸閱讀" not in text:
        result.warnings.append("缺少「延伸閱讀」段落")

    return result


def _discover_chapters(root: Path) -> list[Path]:
    """模仿 validate.sh 的 find 條件：content/ 下 .md，但跳過 `_*` 和 `index.md`。"""
    out: list[Path] = []
    for p in sorted(root.rglob("*.md")):
        if p.name.startswith("_") or p.name == "index.md":
            continue
        out.append(p)
    return out


def validate_many(paths: Iterable[str | Path]) -> list[ValidationResult]:
    return [validate_file(p) for p in paths]


def _format(result: ValidationResult) -> str:
    lines = [f"\n━━━ {result.path} ━━━"]
    if result.ok and not result.warnings:
        lines.append("  ✓ 全部通過")
        return "\n".join(lines)
    for err in result.errors:
        lines.append(f"  ✗ {err}")
    for warn in result.warnings:
        lines.append(f"  ⚠ {warn}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="驗證章節 Markdown 結構")
    parser.add_argument(
        "files",
        nargs="*",
        help="要檢查的 Markdown 檔案；省略時掃 content/ 整個目錄",
    )
    parser.add_argument(
        "--content-root",
        default="content",
        help="批次模式下掃描的根目錄（預設 content/）",
    )
    args = parser.parse_args(argv)

    if args.files:
        targets = [Path(f) for f in args.files]
    else:
        root = Path(args.content_root)
        if not root.exists():
            print(f"[錯誤] 找不到內容根目錄：{root}", file=sys.stderr)
            return 1
        targets = _discover_chapters(root)

    if not targets:
        print("沒有找到任何章節檔。")
        return 0

    results = validate_many(targets)
    total_errors = 0
    total_warnings = 0
    for r in results:
        print(_format(r))
        total_errors += len(r.errors)
        total_warnings += len(r.warnings)

    print("\n" + "━" * 40)
    if total_errors == 0:
        print(f"全部通過！（警告：{total_warnings}）")
        return 0
    print(f"失敗：{total_errors} 個錯誤，{total_warnings} 個警告")
    return 1


if __name__ == "__main__":
    sys.exit(main())
