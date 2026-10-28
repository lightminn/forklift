#!/bin/bash
#SBATCH --job-name=nfr_export
#SBATCH --partition=compute
#SBATCH --cpus-per-task=4
#SBATCH --mem=16G
#SBATCH --time=03:00:00
# docs/plans/2026-10-28-near-front-range.md 5.1: the riib line_1_m bounds export with the front's range, D8c (a)-(g),
# the roof-suppressed D8c replay and S3 drive terms, on the held runs (nrv_73ba6940: S2 15 + S3 6) and their D8b
# calibration and observations (analysis2), with the tools of the merged main snapshot TOOLS. No new Isaac run.
set -u
P=/home/projects/forklift
RUNSET=nrv_73ba6940
SNAP=$P/snapshots/${TOOLS:?the tools snapshot}
R=test_runs/riibotics/$RUNSET
A=$R/analysis2
O=$R/${OUT:-front_range}
PY=$P/.venv/bin/python
export PYTHONPATH=$SNAP/src:$SNAP PYTHONDONTWRITEBYTECODE=1
cd $P || exit 1
mkdir -p $O
step() { echo "== $* $(date -Is)"; "$@"; local rc=$?; echo "== exit $rc $(date -Is)"; return $rc; }
step $PY $SNAP/tools/export_near_field_bounds.py --calibration $A/calibration/d8b_calibration.json \
  --observations $A/observations/d8b_observations.json --output $O/near_field_bounds_riib_line_1_m.json || exit 1
for MODE in normal roof_off; do
  EXTRA=(); [ $MODE = roof_off ] && EXTRA=(--roof-off)
  step $PY $SNAP/tools/d8c_replay.py --bounds $O/near_field_bounds_riib_line_1_m.json \
    --calibration $A/calibration/d8b_calibration.json --output $O/d8c_$MODE "${EXTRA[@]}"
  step $PY $SNAP/tools/d8c_replay.py --bounds $O/near_field_bounds_riib_line_1_m.json \
    --calibration $A/calibration/d8b_calibration.json --output $O/d8c_$MODE --collect "${EXTRA[@]}"
done
echo "{\"snapshot\": \"$RUNSET\", \"allowed_sources\": {}}" > $O/s4_manifest.json
step $PY $SNAP/tools/s4_drive_terms.py --bounds $O/near_field_bounds_riib_line_1_m.json --manifest $O/s4_manifest.json \
  --data-root $P --runs $R/s3_s1_v055/run $R/s3_s1_v080/run $R/s3_s3_v055/run $R/s3_s3_v080/run $R/s3_s5_v055/run \
  $R/s3_s5_v080/run --output $O/near_field_drive_terms_riib_line_1_m.json
echo "all steps done $(date -Is)"
