#!/usr/bin/env bash
# Real-time plan 5.22 correction (2026-11-09): the set-F supplement's job. The bundle runs unchanged from the frozen
# snapshot; this wrapper only adds the lead's CPU sampler (every 10 s, the whole job) beside it.
#   sbatch [resources] g2mf_job.sh SNAP_DIR SAMPLER.py PS_OUT.jsonl LIST.jsonl BUNDLE_OUT live
set -euo pipefail
SNAP=$1 SAMPLER=$2 PS_OUT=$3
shift 3
PY=${G2M_PY:-/home/projects/forklift/.venv/bin/python}
"$PY" "$SAMPLER" "$PS_OUT" --every 10 &
SAMPLER_PID=$!
echo "sampler pid $SAMPLER_PID -> $PS_OUT $(date -Is)"
rc=0
cd "$SNAP"
bash deploy/slurm/isaac_bundle.sbatch "$@" || rc=$?
kill -TERM "$SAMPLER_PID" 2>/dev/null || true
wait "$SAMPLER_PID" 2>/dev/null || true
echo "g2mf job exit $rc $(date -Is)"
exit "$rc"
