#!/usr/bin/env bash
# The teleop smoothness confirmation (docs/plans/2026-11-12-teleop-smooth-confirmation.md 2): six single-session live
# jobs, each waiting for the one before (afterany), in the registered order; each run by the 1e stage-1 job wrapper
# (the other users' CPU sampler beside the bundle), --slots 1 --retries 0, CPU 6, shard 4 (as the latency batch B).
#   bash ts_submit.sh SNAP_DIR OUT_ROOT [FIRST_DEPENDENCY] [-- NAME ...]
# With names after --, only their one rerun each (<name>_re; plan 3: a session over the overlap rule).
set -euo pipefail
SNAP=$1 OUT=$2
shift 2
DEP=""
if [ $# -gt 0 ] && [ "$1" != "--" ]; then DEP=$1; shift; fi
if [ $# -gt 0 ] && [ "$1" = "--" ]; then shift; fi
PY=${G2M_PY:-/home/projects/forklift/.venv/bin/python}
cd "$SNAP"
NAMES=$("$PY" docs/validation/2026-11-12-teleop-smooth/ts_lists.py "$SNAP" "$OUT" "$@" | cut -d' ' -f1)
prev=$DEP
for NAME in $NAMES; do
  LIST="$OUT/$NAME.jsonl"
  python3 tools/isaac_bundle.py check --list "$LIST" --runner "$SNAP/sim/isaac/run_transport.py" --kind live --jobs 1 >/dev/null
  mkdir -p "$OUT/$NAME"
  DEPARG=()
  [ -n "$prev" ] && DEPARG=(--dependency=afterany:"$prev")
  prev=$(BUNDLE_ARGS="--slots 1 --retries 0 --run-est-min 12" sbatch --parsable "${DEPARG[@]}" --job-name="$NAME" \
    --output="$OUT/$NAME-%j.log" --gres=shard:4 --cpus-per-task=6 --mem=8G --time=00:40:00 \
    docs/validation/2026-11-10-g3-1e-core/g3_1e_core_job.sh "$SNAP" "$OUT/${NAME}_ps.jsonl" \
    "$OUT/${NAME}_threads.jsonl" "$LIST" "$OUT/$NAME/bundle" live)
  echo "$NAME job $prev"
done
