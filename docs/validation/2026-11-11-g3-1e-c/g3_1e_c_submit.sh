#!/usr/bin/env bash
# 1e (c) core-only direct comparison (docs/plans/2026-11-11-g3-1e-core-only.md): the four jobs A B B A, each waiting for
# the one before (afterany), each a single session (--slots 1 --retries 0) run by the 1e stage-1 job wrapper (overlap
# sampler and thread sampler beside the bundle). A asks for CPU 4, B for CPU 6; shard 4 and 8G both.
#   bash g3_1e_c_submit.sh SNAP_DIR OUT_ROOT [FIRST_DEPENDENCY]
set -euo pipefail
SNAP=$1 OUT=$2 DEP=${3:-}
PY=${G2M_PY:-/home/projects/forklift/.venv/bin/python}
cd "$SNAP"
"$PY" docs/validation/2026-11-11-g3-1e-c/g3_1e_c_lists.py "$SNAP" "$OUT" > "$OUT.jobs"
prev=$DEP
while read -r NAME ARM PAIR CPUS LIST; do
  python3 tools/isaac_bundle.py check --list "$LIST" --runner "$SNAP/sim/isaac/run_transport.py" --kind live --jobs 1 >/dev/null
  mkdir -p "$OUT/$NAME"
  DEPARG=()
  [ -n "$prev" ] && DEPARG=(--dependency=afterany:"$prev")
  prev=$(BUNDLE_ARGS="--slots 1 --retries 0 --run-est-min 8" sbatch --parsable "${DEPARG[@]}" --job-name="$NAME" \
    --output="$OUT/$NAME-%j.log" --gres=shard:4 --cpus-per-task="$CPUS" --mem=8G --time=00:40:00 \
    docs/validation/2026-11-10-g3-1e-core/g3_1e_core_job.sh "$SNAP" "$OUT/${NAME}_ps.jsonl" \
    "$OUT/${NAME}_threads.jsonl" "$LIST" "$OUT/$NAME/bundle" live)
  echo "$NAME $ARM pair $PAIR cpus $CPUS job $prev"
done < "$OUT.jobs"
