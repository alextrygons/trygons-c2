#!/bin/sh
# Runs the test gate (tools/check.sh) on a clean checkout of one commit, so
# work can go on in the working folder meanwhile:
#     sh tests/acceptance/gate_commit.sh [COMMIT]      (default: HEAD)
# -> test_results/gate_<short hash>.txt, last line "GATE: OK" / "GATE: FAILED"
cd "$(dirname "$0")/../.." || exit 1
REV=$(git rev-parse --short "${1:-HEAD}") || exit 1
WT="test_results/gate_wt"
OUT="test_results/gate_$REV.txt"
mkdir -p test_results
git worktree remove --force "$WT" 2>/dev/null
rm -rf "$WT"
git worktree prune
git worktree add --detach "$WT" "$REV" > /dev/null 2>&1 || { echo "worktree failed" > "$OUT"; exit 1; }
# downloaded tools / model are not in git: use the ones of this folder
ROOT=$(pwd)
[ -e tools/bin/mediamtx/mediamtx.exe ] && export MEDIAMTX_BIN="$ROOT/tools/bin/mediamtx/mediamtx.exe"
[ -e tools/bin/vision-python/python.exe ] && export VISION_PYTHON="$ROOT/tools/bin/vision-python/python.exe"
if [ -d gateway/vision/models ]; then
    mkdir -p "$WT/gateway/vision/models"
    for f in gateway/vision/models/*; do ln "$f" "$WT/$f" 2>/dev/null || cp "$f" "$WT/$f"; done
fi
sh "$WT/tools/check.sh" > "$OUT" 2>&1
rc=$?
git worktree remove --force "$WT" 2>/dev/null
echo "commit $REV, exit $rc" >> "$OUT"
tail -2 "$OUT"
exit $rc
