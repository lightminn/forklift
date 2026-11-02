#!/bin/bash
#SBATCH --job-name=replay_gate
#SBATCH --partition=compute
#SBATCH --cpus-per-task=1
#SBATCH --mem=4G
#SBATCH --time=00:40:00
# The boundary replay gate: one array task per record of LIST, replayed on CPU (tools/boundary_replay.py) with the code
# of the snapshot SNAP. Each task replays from its own copy of SNAP (OUT/work_ID, removed at the end), so SNAP itself
# is never written.
#   mkdir -p OUT; sbatch --array=1-$(wc -l < LIST) --output=OUT/r_%a.log deploy/slurm/replay_gate.sbatch SNAP LIST OUT
#   LIST: one record per line, relative to RUNS_ROOT (environment; default the fork-spacing artifacts) or absolute.
#   SETUP_INPUTS (environment): strict (default) replays on SNAP's own setup files, so a record whose setup read a
#     file SNAP has changed is refused as before; recorded writes the record's own non-code setup inputs (config,
#     models: tools/gate_setup_overlay.py) from its recording snapshot onto the copy first and prints every file it
#     wrote with both hashes. Code files are never written: a differing one refuses the task.
# Writes OUT/r_ID.json (the replay), OUT/o_ID.json (the overlay, recorded only) and the log lines
#   SETUP_WRITTEN <path> <recorded sha256> <SNAP's sha256>   (recorded only, one per file written)
#   REPLAY_EXIT <code> <record>                              (0 = the record replayed to the end unchanged)
# Count the result with: grep -h REPLAY_EXIT OUT/r_*.log | sort | uniq -c
set -u
SNAP=$(realpath "$1"); LIST=$(realpath "$2"); OUT=$(realpath -m "$3")
PY=${VENV_PY:-/home/projects/forklift/.venv/bin/python}
RUNS_ROOT=${RUNS_ROOT:-/home/projects/forklift/artifacts/20261013_fork_spacing}
SETUP_INPUTS=${SETUP_INPUTS:-strict}
ID=${SLURM_ARRAY_TASK_ID:?an array task}
case "$SETUP_INPUTS" in strict|recorded) ;; *) echo "SETUP_INPUTS is strict or recorded, not $SETUP_INPUTS"; exit 2;; esac
REL=$(sed -n "${ID}p" "$LIST")
[ -n "$REL" ] || { echo "no line $ID in $LIST"; exit 2; }
case "$REL" in /*) RUN=$REL;; *) RUN=$RUNS_ROOT/$REL;; esac
mkdir -p "$OUT"
W=$OUT/work_$ID
[ -e "$W" ] && { echo "refusing: $W exists"; exit 2; }
trap 'rm -rf "$W"' EXIT
mkdir -p "$W" && cp -a "$SNAP/." "$W/"
echo "gate $SNAP setup=$SETUP_INPUTS record $RUN $(date -Is)"
if [ "$SETUP_INPUTS" = recorded ]; then
  touch "$W/.gate_setup_copy"
  "$PY" "$W/tools/gate_setup_overlay.py" "$RUN" "$W" --json "$OUT/o_$ID.json" > /dev/null
  rc=$?
  "$PY" -c 'import json,sys; o=json.load(open(sys.argv[1]))
print("SETUP_REFUSED", o["refused"]) if "refused" in o else [print("SETUP_WRITTEN", w["path"], w["recorded_sha256"], w["gate_sha256"]) for w in o["written"]]' "$OUT/o_$ID.json"
  [ $rc -eq 0 ] || { echo "REPLAY_EXIT 3 $REL"; exit 0; }
fi
cd "$W"
"$PY" tools/boundary_replay.py "$RUN" --json "$OUT/r_$ID.json" > /dev/null
echo "REPLAY_EXIT $? $REL"
