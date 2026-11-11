#!/usr/bin/env bash
# The latency test B (docs/plans/2026-11-11-latency-tolerance.md 3.2): fifteen single-session live jobs, each waiting for
# the one before (afterany), in the registered order (0 25 50 100 500, reversed, again); each run by the 1e stage-1 job
# wrapper (the overlap sampler beside the bundle), --slots 1 --retries 0, CPU 6 (two session cores and a bridge core).
#   bash lat_b_submit.sh SNAP_DIR OUT_ROOT [FIRST_DEPENDENCY] [-- RERUN_NAME ...]
# With names after --, only those jobs again under lat_r_<name> (the registration's one rerun; the first stays).
set -euo pipefail
SNAP=$1 OUT=$2 DEP=${3:-}
shift $(( $# >= 3 ? 3 : $# ))
[ "${1:-}" = "--" ] && shift
PY=${G2M_PY:-/home/projects/forklift/.venv/bin/python}
cd "$SNAP"
if [ $# -gt 0 ]; then
  "$PY" docs/validation/2026-11-11-latency/lat_lists.py rerun "$SNAP" "$OUT" "$@" >/dev/null
  NAMES=$(for n in "$@"; do echo "lat_r_$n"; done)
else
  NAMES=$("$PY" docs/validation/2026-11-11-latency/lat_lists.py b "$SNAP" "$OUT" | cut -d' ' -f1)
fi
prev=$DEP
for NAME in $NAMES; do
  LIST="$OUT/$NAME.jsonl"
  python3 tools/isaac_bundle.py check --list "$LIST" --runner "$SNAP/sim/isaac/run_transport.py" --kind live --jobs 1 >/dev/null
  mkdir -p "$OUT/$NAME"
  DEPARG=()
  [ -n "$prev" ] && DEPARG=(--dependency=afterany:"$prev")
  prev=$(BUNDLE_ARGS="--slots 1 --retries 0 --run-est-min 8" sbatch --parsable "${DEPARG[@]}" --job-name="$NAME" \
    --output="$OUT/$NAME-%j.log" --gres=shard:4 --cpus-per-task=6 --mem=8G --time=00:40:00 \
    docs/validation/2026-11-10-g3-1e-core/g3_1e_core_job.sh "$SNAP" "$OUT/${NAME}_ps.jsonl" \
    "$OUT/${NAME}_threads.jsonl" "$LIST" "$OUT/$NAME/bundle" live)
  echo "$NAME job $prev"
done
