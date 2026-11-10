#!/usr/bin/env bash
# 1e stage 1 (docs/plans/2026-11-10-g3-1e-process-worker.md): the measurement job. The bundle runs unchanged from the
# snapshot; this wrapper adds the overlap sampler (g2m_cpu_sampler, every 10 s, the whole node by user) and the thread
# sampler (every 10 s, this job's own threads) beside it.
#   sbatch --cpus-per-task=6 [resources] g3_1e_core_job.sh SNAP_DIR PS_OUT.jsonl THREADS_OUT.jsonl LIST.jsonl BUNDLE_OUT live
set -euo pipefail
# single slot, no automatic retry (plan 3: a retried session invalidates its pair; Codex review P2-3/P2-4)
case " ${BUNDLE_ARGS:-} " in *" --slots 1 "*) ;; *) echo "BUNDLE_ARGS must hold --slots 1" >&2; exit 2;; esac
case " ${BUNDLE_ARGS:-} " in *" --retries 0 "*) ;; *) echo "BUNDLE_ARGS must hold --retries 0" >&2; exit 2;; esac
SNAP=$1 PS_OUT=$2 TH_OUT=$3
shift 3
PY=${G2M_PY:-/home/projects/forklift/.venv/bin/python}
"$PY" "$SNAP/docs/validation/2026-11-09-g2-measure/g2m_cpu_sampler.py" "$PS_OUT" --every 10 &
PS_PID=$!
"$PY" "$SNAP/docs/validation/2026-11-10-g3-1e-core/g3_1e_thread_sampler.py" "$TH_OUT" --every 10 &
TH_PID=$!
echo "samplers $PS_PID -> $PS_OUT, $TH_PID -> $TH_OUT $(date -Is)"
rc=0
cd "$SNAP"
bash deploy/slurm/isaac_bundle.sbatch "$@" || rc=$?
kill -TERM "$PS_PID" "$TH_PID" 2>/dev/null || true
wait "$PS_PID" "$TH_PID" 2>/dev/null || true
echo "g3_1e_core job exit $rc $(date -Is)"
exit "$rc"
