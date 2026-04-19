# Self-Learning Embodied AI

A structured, bilingual (zh-Hant / English) learning system for robotics, embodied AI, and the underlying algorithms and math. Built for someone transitioning from mechatronics hardware into embodied AI research and industry.

## What This Covers

22 chapters organized as a coherent curriculum:

**C++ Systems** &rarr; **ROS 2** &rarr; **Kinematics & Dynamics** &rarr; **Motion Planning** &rarr; **Control Theory** &rarr; **SLAM** &rarr; **Reinforcement Learning** &rarr; **Imitation Learning** &rarr; **Sim-to-Real Transfer** &rarr; **Multimodal LLMs (VLA / VLN)**

Each chapter follows a fixed template: precise definitions, closed-loop positioning (perception &rarr; planning &rarr; control), physical intuition, minimal-but-sufficient math, scenario-based exercises with full reasoning chains, and interview talking points.

## Features

- **Bilingual toggle** &mdash; switch between zh-Hant and English with scroll position preserved
- **Progress tracking** &mdash; per-page completion state stored in localStorage
- **KaTeX math rendering** &mdash; inline and block equations, each annotated with physical meaning
- **Progressive disclosure** &mdash; collapsible `<details>` blocks for full derivations, complete implementations, and edge-case analysis
- **Editorial design** &mdash; custom design tokens, responsive layout (320px&ndash;1440px), fluid typography

## Quick Start

```bash
npm install
npm run dev
```

Open [http://localhost:3000](http://localhost:3000).

## Build & Serve

```bash
npm run build
npm run serve
```

## Content Pipeline

1. **Source registration** &mdash; `pipeline/sources/{book-slug}.yaml`
2. **Knowledge extraction** &mdash; NotebookLM MCP, idempotent, driven by YAML status fields
3. **Chapter synthesis** &mdash; prompt templates in `pipeline/templates/` transform extracted notes into structured chapters
4. **Quality validation** &mdash; `bash quality/validate.sh` checks required sections, `<details>` count, and KaTeX syntax

## Project Structure

```
content/          # All learning content (Markdown + KaTeX)
pipeline/         # Content generation pipeline (templates, sources, scripts)
src/              # React components, pages, theme customizations, CSS
  components/     # Custom React components
  theme/          # Docusaurus theme swizzles (wrap strategy)
  css/            # Design tokens and global styles
  lib/            # Utilities (progress tracking, etc.)
i18n/             # Internationalization (en locale)
quality/          # Validation scripts and checklists
tasks/            # Task tracking (backlog, current, done)
```

## Tech Stack

- [Docusaurus 3](https://docusaurus.io/) &mdash; static site generator
- React 19 + TypeScript 6
- [KaTeX](https://katex.org/) via remark-math + rehype-katex
- Playwright &mdash; visual regression and E2E testing

## License

MIT
