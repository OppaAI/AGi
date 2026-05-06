#!/usr/bin/env bash
# session_diff.sh
# Generates one diff file per "coding day" defined as 06:00 to 03:59 next day.
# Usage: ./session_diff.sh /path/to/git/repo /path/to/output/dir
# Output: one .diff file per day that had commits, named YYYY-MM-DD.diff
# Covers Mar 16, 2026 through May 5, 2026 (before 04:00).

set -euo pipefail

REPO="${1:-$(pwd)}"
OUTDIR="${2:-./session_diffs}"

mkdir -p "$OUTDIR"

cd "$REPO"

# Verify it's a git repo
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    echo "ERROR: $REPO is not a git repository." >&2
    exit 1
fi

# Session boundary: a "day" runs from 06:00 on DATE to 03:59:59 on DATE+1
# We iterate from Mar 16 to May 5, 2026

# Generate list of session day start dates (YYYY-MM-DD)
START_DATE="2026-03-16"
END_DATE="2026-05-05"

current="$START_DATE"

while python3 -c "import sys; from datetime import date; sys.exit(0 if date.fromisoformat('$current') <= date.fromisoformat('$END_DATE') else 1)"; do
    # Session window: 06:00 on $current to 03:59:59 on $current + 1 day
    session_start="${current} 06:00:00"
    next_day=$(date -d "$current + 1 day" +%Y-%m-%d 2>/dev/null || \
               python3 -c "from datetime import date, timedelta; d=date.fromisoformat('$current'); print(d+timedelta(days=1))")
    session_end="${next_day} 03:59:59"

    # Get all commits in this session window (chronological order)
    commits=$(git log --all \
        --after="$session_start" \
        --before="$session_end" \
        --reverse \
        --pretty=format:"%H" 2>/dev/null || true)

    if [[ -z "$commits" ]]; then
        # No commits this session — skip, no file created
        current="$next_day"
        continue
    fi

    # First and last commit in this session
    first_commit=$(echo "$commits" | head -1)
    last_commit=$(echo "$commits" | tail -1)

    # Get the parent of the first commit as the diff base
    # If first commit has no parent (initial commit), diff against empty tree
    if git rev-parse --verify "${first_commit}^" > /dev/null 2>&1; then
        diff_base="${first_commit}^"
    else
        diff_base="$(git hash-object -t tree /dev/null)"
    fi

    outfile="${OUTDIR}/${current}.diff"

    {
        echo "# Session Diff: ${current}"
        echo "# Window: ${session_start} → ${session_end}"
        echo "# Commits: $(echo "$commits" | wc -l | tr -d ' ')"
        echo "# Range: ${diff_base:0:7} → ${last_commit:0:7}"
        echo ""
        echo "## Commit Log"
        echo ""
        git log --all \
            --after="$session_start" \
            --before="$session_end" \
            --reverse \
            --pretty=format:"%h %ad %s" \
            --date=format:"%Y-%m-%d %H:%M"
        echo ""
        echo ""
        echo "## Stat Summary"
        echo ""
        git diff --stat "$diff_base" "$last_commit"
        echo ""
        echo "## Full Diff"
        echo ""
        git diff "$diff_base" "$last_commit"
    } > "$outfile"

    line_count=$(wc -l < "$outfile")
    echo "✅ ${current} — $(echo "$commits" | wc -l | tr -d ' ') commit(s) → ${outfile} (${line_count} lines)"

    current="$next_day"
done

echo ""
echo "Done. Files written to: $OUTDIR"
