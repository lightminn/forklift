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
#   REPLAY_EXIT <code> <record> setup=<strict|recorded>     (0 = the record replayed to the end unchanged)
# and ends with that code: 2 a setup error (no copy, no such line, an output already there), 3 the overlay refused,
# otherwise the replay's own. A pass under setup=recorded says the code replays the record on the inputs it was
# recorded with; it says nothing of the changed setup files themselves (their physics, their initialisation).
# One OUT per array: a task claims OUT/work_ID by an atomic mkdir and refuses a result that is already there.
# Count the result with: grep -h REPLAY_EXIT OUT/r_*.log | cut -d' ' -f2,4 | sort | uniq -c
set -u
fail() { echo "REPLAY_EXIT $1 ${REL:-?} setup=${SETUP_INPUTS:-?} ($2)"; exit "$1"; }
SETUP_INPUTS=${SETUP_INPUTS:-strict}
case "$SETUP_INPUTS" in strict|recorded) ;; *) fail 2 "SETUP_INPUTS is strict or recorded, not $SETUP_INPUTS";; esac
[ $# -eq 3 ] || fail 2 "usage: SNAP LIST OUT"
SNAP=$(realpath -e "$1") || fail 2 "no snapshot $1"
LIST=$(realpath -e "$2") || fail 2 "no list $2"
OUT=$(realpath -m "$3")
[ -f "$SNAP/tools/boundary_replay.py" ] || fail 2 "$SNAP is not a snapshot"
PY=${VENV_PY:-/home/projects/forklift/.venv/bin/python}
RUNS_ROOT=${RUNS_ROOT:-/home/projects/forklift/artifacts/20261013_fork_spacing}
ID=${SLURM_ARRAY_TASK_ID:-}
[ -n "$ID" ] || fail 2 "not an array task"
REL=$(sed -n "${ID}p" "$LIST")
[ -n "$REL" ] || fail 2 "no line $ID in $LIST"
case "$REL" in /*) RUN=$REL;; *) RUN=$RUNS_ROOT/$REL;; esac
[ -f "$RUN/boundary/setup_snapshot.json" ] || fail 2 "$RUN holds no boundary record"
mkdir -p "$OUT" || fail 2 "cannot make $OUT"
[ -e "$OUT/r_$ID.json" ] || [ -e "$OUT/o_$ID.json" ] && fail 2 "$OUT already holds task $ID's result"
W=$OUT/work_$ID
mkdir "$W" 2>/dev/null || fail 2 "$W is taken"
trap 'rm -rf "$W"' EXIT
cp -a "$SNAP/." "$W/" || fail 2 "copy of $SNAP failed"
echo "gate $SNAP setup=$SETUP_INPUTS record $RUN $(date -Is)"
if [ "$SETUP_INPUTS" = recorded ]; then
  touch "$W/.gate_setup_copy" || fail 2 "no marker"
  "$PY" "$W/tools/gate_setup_overlay.py" "$RUN" "$W" --json "$OUT/o_$ID.json" > /dev/null
  rc=$?
  "$PY" -c 'import json,sys
o = json.load(open(sys.argv[1]))
for w in o.get("written", []):
    print("SETUP_WRITTEN", w["path"], w["recorded_sha256"], w["gate_sha256"])
if "refused" in o:
    print("SETUP_REFUSED", o["refused"])' "$OUT/o_$ID.json" || fail 3 "no overlay report"
  [ $rc -eq 0 ] || fail 3 "overlay refused"
fi
cd "$W" || fail 2 "cannot enter $W"
"$PY" tools/boundary_replay.py "$RUN" --json "$OUT/r_$ID.json" > /dev/null
rc=$?
echo "REPLAY_EXIT $rc $REL setup=$SETUP_INPUTS"
exit $rc
