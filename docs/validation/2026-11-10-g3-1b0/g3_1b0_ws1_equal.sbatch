#!/bin/bash
# Real-time plan 5.26 1b: the switch-on equality on ws1's interpreter (NumPy 1.x), beside the merge's pytest.
# Every grid_snapshot dump is replayed with FORKLIFT_GRID_CLEAR_KERNEL on, in both kernel modes; the analyzer exits 1
# unless every replay equals the dump's switch-off result byte for byte (state and free_stamp).
#
#   sbatch --export=ALL,SRC=<source copy>,DUMPS=<dump root>,OUT=<json> g3_1b0_ws1_equal.sbatch
#SBATCH --job-name=g3b0_equal
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=3G
#SBATCH --time=00:40:00
set -uo pipefail
: "${SRC:?}" "${DUMPS:?}" "${OUT:?}"
cd "$SRC"
export PYTHONPATH="$SRC/src" PYTHONDONTWRITEBYTECODE=1 NUMBA_NUM_THREADS=1
/home/projects/forklift/.venv/bin/python docs/validation/2026-11-10-g3-1b0/g3_1b0_analyze.py \
  "$DUMPS"/g3b0_F "$DUMPS"/g3b0_H --json "$OUT" --modes reference numba --clear-kernel on \
  --source-commit "$(cat "$SRC/.snapshot_commit" 2>/dev/null || echo unknown)"
rc=$?
echo "EQUAL_EXIT $rc"
exit $rc
