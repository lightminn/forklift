#!/bin/bash
# Usage: p0b_run.sh OUTDIR LATENCY DECEL  (run from the tree root on ws1)
set -u
A=/home/projects/forklift/artifacts
O=$1; LAT=$2; DEC=$3; ENV=${4:-0.05}; GAP=${5:-0.0}; INJ=${6:-0.0}; WID=${7:-0.0}; SHB=${8:-0.0}; TAP=${9:-0.0}
mkdir -p "$O"
export A O LAT DEC ENV GAP INJ WID SHB TAP
python - <<'PY' > jobs.txt
import yaml
c = yaml.safe_load(open(__import__("os").environ.get("CANDS", "config/p0b_candidates.yaml")))["candidates"]
for s in (1, 3, 5):
    for n in c:
        print(s, n)
        yaml.safe_dump({"candidates": {n: c[n]}}, open(f"cand_{n}.yaml", "w"))
PY
one() {
  S=$1; N=$2
  python tools/p0b_sensor_study.py evaluate --run $A/20261004_slam_s2/v38f_slam/seed_$S/run \
    --sections $A/20261005_p5_p0b/sections_s$S.npz --candidates cand_$N.yaml \
    --odometry-age $A/20261005_p5_p0a/odometry_age_v2.json --stop-latency-s $LAT --stop-decel-mps2 $DEC --envelope-m $ENV --close-gap-m $GAP --inject-every-m $INJ --free-min-width-m $WID --shadow-band-m $SHB --taper-m $TAP \
    --output $O/s${S}_$N.json 2>&1 | head -1 | sed "s/^/s$S /"
}
export -f one
xargs -P 4 -L 1 bash -c 'one "$0" "$1"' < jobs.txt
