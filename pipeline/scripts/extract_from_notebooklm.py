#!/usr/bin/env python3
"""NotebookLM 內容提取規劃腳本。

NotebookLM 實際的 MCP 呼叫發生在 Claude Code session 內（透過 MCP tools）。
這支腳本的工作是**圍繞** MCP 呼叫的冪等性 I/O：

1. 讀 `pipeline/sources/{book}.yaml`，了解 notebook id / 來源狀態 / 章節狀態
2. 產生「需要做的事情」清單（建 notebook / 匯入來源 / 提取章節）
3. 提供 update 函式讓 Claude 在完成每一步後回寫 yaml
4. 幫 Claude 檢查 `pipeline/extracted/` 是否已有輸出檔（避免重複提取）

CLI 用法：
    python pipeline/scripts/extract_from_notebooklm.py status pipeline/sources/foo.yaml
    python pipeline/scripts/extract_from_notebooklm.py plan   pipeline/sources/foo.yaml

程式用法（在 Claude Code 中呼叫 MCP 後回寫）：
    from pipeline.scripts.extract_from_notebooklm import (
        load_book, pending_sources, pending_chapters,
        set_notebook_id, mark_source_ingested, mark_chapter_extracted,
    )
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml


SOURCE_PENDING = "pending"
SOURCE_INGESTED = "ingested"

CHAPTER_PENDING = "pending"
CHAPTER_EXTRACTED = "extracted"
CHAPTER_GENERATED = "generated"
CHAPTER_REVIEWED = "reviewed"


@dataclass(frozen=True)
class Book:
    path: Path
    data: dict[str, Any]

    @property
    def slug(self) -> str:
        return str(self.data.get("book", ""))

    @property
    def notebook_id(self) -> str | None:
        nid = self.data.get("notebook_id")
        return nid if isinstance(nid, str) and nid else None

    @property
    def sources(self) -> list[dict[str, Any]]:
        return list(self.data.get("sources", []))

    @property
    def chapters(self) -> list[dict[str, Any]]:
        return list(self.data.get("chapters", []))


def load_book(yaml_path: str | Path) -> Book:
    p = Path(yaml_path)
    with p.open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    if not isinstance(data, dict):
        raise ValueError(f"{p}: 根節點必須是 mapping，收到 {type(data).__name__}")
    return Book(path=p, data=data)


def save_book(book: Book) -> None:
    with book.path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(
            book.data,
            f,
            allow_unicode=True,
            sort_keys=False,
            default_flow_style=False,
        )


def pending_sources(book: Book) -> list[dict[str, Any]]:
    return [s for s in book.sources if s.get("status", SOURCE_PENDING) == SOURCE_PENDING]


def pending_chapters(book: Book, extracted_root: Path | None = None) -> list[dict[str, Any]]:
    """回傳狀態仍為 pending、且 extracted 檔案尚不存在的章節。"""
    out: list[dict[str, Any]] = []
    root = extracted_root or Path("pipeline/extracted") / book.slug
    for ch in book.chapters:
        status = ch.get("status", CHAPTER_PENDING)
        if status != CHAPTER_PENDING:
            continue
        slug = ch.get("slug")
        if slug and (root / f"{slug}.md").exists():
            # 磁碟上已有輸出 → 這是狀態漂移，Claude 該把 yaml 改成 extracted
            continue
        out.append(ch)
    return out


def set_notebook_id(book: Book, notebook_id: str) -> Book:
    new_data = {**book.data, "notebook_id": notebook_id}
    new_book = Book(path=book.path, data=new_data)
    save_book(new_book)
    return new_book


def _update_source(book: Book, name: str, new_status: str) -> Book:
    new_sources = []
    changed = False
    for s in book.sources:
        if s.get("name") == name and not changed:
            new_sources.append({**s, "status": new_status})
            changed = True
        else:
            new_sources.append(s)
    if not changed:
        raise KeyError(f"找不到 source：{name}")
    new_data = {**book.data, "sources": new_sources}
    new_book = Book(path=book.path, data=new_data)
    save_book(new_book)
    return new_book


def _update_chapter(book: Book, slug: str, new_status: str) -> Book:
    new_chapters = []
    changed = False
    for ch in book.chapters:
        if ch.get("slug") == slug and not changed:
            new_chapters.append({**ch, "status": new_status})
            changed = True
        else:
            new_chapters.append(ch)
    if not changed:
        raise KeyError(f"找不到 chapter：{slug}")
    new_data = {**book.data, "chapters": new_chapters}
    new_book = Book(path=book.path, data=new_data)
    save_book(new_book)
    return new_book


def mark_source_ingested(book: Book, source_name: str) -> Book:
    return _update_source(book, source_name, SOURCE_INGESTED)


def mark_chapter_extracted(book: Book, chapter_slug: str) -> Book:
    return _update_chapter(book, chapter_slug, CHAPTER_EXTRACTED)


def mark_chapter_generated(book: Book, chapter_slug: str) -> Book:
    return _update_chapter(book, chapter_slug, CHAPTER_GENERATED)


def mark_chapter_reviewed(book: Book, chapter_slug: str) -> Book:
    return _update_chapter(book, chapter_slug, CHAPTER_REVIEWED)


def _status_summary(book: Book) -> dict[str, Any]:
    src_counts: dict[str, int] = {}
    for s in book.sources:
        st = s.get("status", SOURCE_PENDING)
        src_counts[st] = src_counts.get(st, 0) + 1
    ch_counts: dict[str, int] = {}
    for ch in book.chapters:
        st = ch.get("status", CHAPTER_PENDING)
        ch_counts[st] = ch_counts.get(st, 0) + 1
    return {
        "book": book.slug,
        "notebook_id": book.notebook_id,
        "sources": src_counts,
        "chapters": ch_counts,
    }


def _plan(book: Book) -> dict[str, Any]:
    steps: list[dict[str, Any]] = []
    if book.notebook_id is None:
        steps.append({
            "action": "create_notebook",
            "reason": "yaml.notebook_id 為空",
        })
    for src in pending_sources(book):
        steps.append({
            "action": "ingest_source",
            "source_name": src.get("name"),
            "source_type": src.get("type"),
        })
    for ch in pending_chapters(book):
        steps.append({
            "action": "extract_chapter",
            "chapter_slug": ch.get("slug"),
            "sources_focus": ch.get("sources_focus", []),
        })
    return {
        "book": book.slug,
        "notebook_id": book.notebook_id,
        "steps": steps,
        "next_action": steps[0]["action"] if steps else "done",
    }


def _cmd_status(args: argparse.Namespace) -> int:
    book = load_book(args.yaml)
    print(json.dumps(_status_summary(book), ensure_ascii=False, indent=2))
    return 0


def _cmd_plan(args: argparse.Namespace) -> int:
    book = load_book(args.yaml)
    print(json.dumps(_plan(book), ensure_ascii=False, indent=2))
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="NotebookLM 提取規劃（I/O 層）")
    sub = parser.add_subparsers(dest="cmd", required=True)

    for name, helper, fn in (
        ("status", "顯示 yaml 狀態摘要", _cmd_status),
        ("plan", "列出還需要執行的 MCP 動作", _cmd_plan),
    ):
        sp = sub.add_parser(name, help=helper)
        sp.add_argument("yaml", help="pipeline/sources/{book}.yaml 路徑")
        sp.set_defaults(func=fn)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    sys.exit(main())
