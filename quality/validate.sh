#!/bin/bash
# 章節品質自動驗證腳本（v2 — 含內容深度檢查）
# 用法: bash quality/validate.sh [content/book-slug/chapter.md ...]
# 不帶參數時驗證所有章節

# 不用 set -e：腳本透過 ERRORS 計數器自行管理錯誤狀態
set +e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m'

ERRORS=0
WARNINGS=0

check_pass() { echo -e "  ${GREEN}✓${NC} $1"; }
check_fail() { echo -e "  ${RED}✗${NC} $1"; ((ERRORS++)); }
check_warn() { echo -e "  ${YELLOW}⚠${NC} $1"; ((WARNINGS++)); }

# ──────────────────────────────────
# 品質閾值（依 CLAUDE.md 硬性要求）
# ──────────────────────────────────
MIN_DETAILS=3          # <details> 摺疊塊最少數量（情境題 ≥ 3）
MIN_DEEP_DIVE=2        # 深入摺疊塊最少（summary 含「深入：」前綴）
MIN_WORD_COUNT=2000    # 章節最低字數（含 KaTeX；scaffold 占位不計）
MIN_KATEX=4            # KaTeX $ 符號最少出現次數（偶數計；2 代表 1 個行內公式）
MIN_MISCONCEPTIONS=2   # 常見誤解條目最少數量
MIN_INTERVIEW_POINTS=3 # 面試角度 talking points 最少數量

validate_chapter() {
    local file="$1"
    echo ""
    echo -e "━━━ ${CYAN}驗證: $file${NC} ━━━"

    # 跳過 scaffold（內容為占位符的章節）
    if grep -q "由 TASK-05 填入" "$file" 2>/dev/null; then
        echo -e "  ${YELLOW}⊘${NC} scaffold（占位檔），跳過品質檢查"
        return
    fi

    # ═══════════════════════════════
    # 1. Frontmatter 結構
    # ═══════════════════════════════
    if head -5 "$file" | grep -q "^---"; then
        check_pass "Frontmatter 存在"
    else
        check_fail "缺少 frontmatter"
        return
    fi

    for field in title prerequisites estimated_time difficulty tags; do
        if grep -q "^${field}:" "$file"; then
            check_pass "Frontmatter 欄位: $field"
        else
            check_fail "缺少 frontmatter 欄位: $field"
        fi
    done

    # ═══════════════════════════════
    # 2. 必要段落（CLAUDE.md 定義的 8 段）
    # ═══════════════════════════════
    for section in "核心概念" "直覺理解" "實作連結" "常見誤解" "練習題" "面試角度"; do
        if grep -q "## ${section}" "$file"; then
            check_pass "段落存在: $section"
        else
            check_fail "缺少段落: $section"
        fi
    done

    # 軟性段落（警告而非錯誤）
    if grep -q "## 你將學到" "$file"; then
        check_pass "學習目標段落存在"
    else
        check_warn "缺少「你將學到」段落"
    fi

    if grep -q "## 延伸閱讀" "$file"; then
        check_pass "延伸閱讀段落存在"
    else
        check_warn "缺少延伸閱讀段落"
    fi

    # ═══════════════════════════════
    # 3. <details> 摺疊塊數量
    # ═══════════════════════════════
    local details_count
    details_count=$(grep -c "<details>" "$file" 2>/dev/null || echo 0)
    if [ "$details_count" -ge "$MIN_DETAILS" ]; then
        check_pass "<details> 數量: $details_count (≥$MIN_DETAILS)"
    else
        check_fail "<details> 不足: $details_count (需要 ≥$MIN_DETAILS)"
    fi

    # ═══════════════════════════════
    # 4. 深入摺疊塊（summary 含「深入：」）
    # ═══════════════════════════════
    local deep_dive_count
    deep_dive_count=$(grep -c "深入：" "$file" 2>/dev/null || echo 0)
    if [ "$deep_dive_count" -ge "$MIN_DEEP_DIVE" ]; then
        check_pass "深入摺疊塊: $deep_dive_count (≥$MIN_DEEP_DIVE)"
    else
        check_warn "深入摺疊塊不足: $deep_dive_count (建議 ≥$MIN_DEEP_DIVE)"
    fi

    # ═══════════════════════════════
    # 5. KaTeX 語法檢查
    # ═══════════════════════════════
    local dollar_count
    dollar_count=$(grep -o '\$' "$file" | wc -l)
    if [ $((dollar_count % 2)) -eq 0 ]; then
        check_pass "KaTeX $ 配對正確 ($dollar_count 個)"
    else
        check_warn "KaTeX $ 可能未配對 ($dollar_count 個，奇數)"
    fi

    if [ "$dollar_count" -ge "$MIN_KATEX" ]; then
        check_pass "KaTeX 公式充足 ($dollar_count ≥ $MIN_KATEX)"
    else
        check_warn "KaTeX 公式偏少 ($dollar_count < $MIN_KATEX)，確認是否需要數學"
    fi

    # ═══════════════════════════════
    # 6. 章節字數（含 KaTeX / code）
    # ═══════════════════════════════
    local word_count
    word_count=$(wc -c < "$file")
    if [ "$word_count" -ge "$MIN_WORD_COUNT" ]; then
        check_pass "內容量: $(numfmt --to=si $word_count) bytes (≥$(numfmt --to=si $MIN_WORD_COUNT))"
    else
        check_fail "內容量不足: $(numfmt --to=si $word_count) bytes (需要 ≥$(numfmt --to=si $MIN_WORD_COUNT))，疑似素材不足"
    fi

    # ═══════════════════════════════
    # 7. 常見誤解條目數量
    # ═══════════════════════════════
    local misconception_count
    misconception_count=$(awk '/^## 常見誤解/,/^## [^常]/' "$file" | grep -cE '^[0-9]+\.' 2>/dev/null || true)
    misconception_count=${misconception_count:-0}
    if [ "$misconception_count" -ge "$MIN_MISCONCEPTIONS" ]; then
        check_pass "常見誤解: $misconception_count 條 (≥$MIN_MISCONCEPTIONS)"
    else
        check_warn "常見誤解偏少: $misconception_count 條 (建議 ≥$MIN_MISCONCEPTIONS)"
    fi

    # ═══════════════════════════════
    # 8. 面試角度 talking points 數量
    # ═══════════════════════════════
    local interview_count
    interview_count=$(awk '/^## 面試角度/,/^## [^面]/' "$file" | grep -cE '^[0-9]+\.' 2>/dev/null || true)
    interview_count=${interview_count:-0}
    if [ "$interview_count" -ge "$MIN_INTERVIEW_POINTS" ]; then
        check_pass "面試角度: $interview_count 點 (≥$MIN_INTERVIEW_POINTS)"
    else
        check_warn "面試角度偏少: $interview_count 點 (建議 ≥$MIN_INTERVIEW_POINTS)"
    fi

    # ═══════════════════════════════
    # 9. 閉環定位關鍵字（感知→規劃→控制）
    # ═══════════════════════════════
    local loop_keywords=0
    for keyword in "感知" "規劃" "控制" "閉環"; do
        if grep -q "$keyword" "$file" 2>/dev/null; then
            loop_keywords=$((loop_keywords + 1))
        fi
    done
    if [ "$loop_keywords" -ge 3 ]; then
        check_pass "閉環定位關鍵字: $loop_keywords/4"
    else
        check_warn "閉環定位關鍵字偏少: $loop_keywords/4（確認有標出在閉環哪個節點）"
    fi

    # ═══════════════════════════════
    # 10. 物理意義註解（公式旁應有解釋）
    # ═══════════════════════════════
    local meaning_count
    meaning_count=$(grep -ciE '物理意義|physical meaning|意義是|意思是|代表' "$file" 2>/dev/null || echo 0)
    if [ "$meaning_count" -ge 2 ]; then
        check_pass "物理意義註解: $meaning_count 處"
    else
        check_warn "物理意義註解偏少: $meaning_count 處（公式應配物理意義說明）"
    fi
}

# ──────────────────────────────────
# 主程式
# ──────────────────────────────────
echo "╔═══════════════════════════════════════╗"
echo "║  學習系統 — 內容品質驗證 (v2)         ║"
echo "╠═══════════════════════════════════════╣"
echo "║  硬性: frontmatter / 段落 / details   ║"
echo "║  品質: 字數 / 深入塊 / 誤解 / 面試   ║"
echo "║  哲學: 閉環定位 / 物理意義 / KaTeX    ║"
echo "╚═══════════════════════════════════════╝"

if [ -n "$1" ]; then
    for f in "$@"; do
        validate_chapter "$f"
    done
else
    find content/ -name "*.md" -not -name "_*" -not -name "index.md" | sort | while read -r file; do
        validate_chapter "$file"
    done
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}全部通過！${NC} (警告: $WARNINGS)"
else
    echo -e "${RED}失敗: $ERRORS 個錯誤${NC}, $WARNINGS 個警告"
    exit 1
fi
