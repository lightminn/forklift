#!/bin/bash
#SBATCH --job-name=pfb_aggL
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=6G
#SBATCH --time=00:30:00
# post-fix baseline RUNBOOK step 5 (live) and the live table; code from the branch copy in $CODE.
set -u
P=/home/projects/forklift; PY=$P/.venv/bin/python; CODE=${CODE:?}; OUT=${OUT:?}
T=$CODE/docs/validation/2026-11-15-post-fix-baseline; R=$P/test_runs/pfb
mkdir -p $OUT; cd $OUT; export PYTHONPATH=$CODE/src
$PY $CODE/tools/ct_survey.py $R/L_pfb_2bea23e5 --json L_survey.json --csv L_missions.csv --runs $R/lists/RUNS_pfb_L_pfb_2bea23e5.jsonl --near-tally L_near.json > ct_survey.log 2>&1; echo "ct_survey exit $?"
$PY $T/live_compare.py --new L_missions.csv --prefix $CODE/docs/validation/2026-11-06-ct-status-survey/missions.csv --arm-map 'LD=D,LO=O6' --out L_vs_cts.json; echo "live_compare exit $?"
$PY $T/report_tables.py --live L_vs_cts.json --out tables_live.md; echo "report exit $?"
$PY - <<PYEOF > L_paths_pace.txt
import json,glob,statistics
R="$R"
bad=0;n=0
for f in sorted(glob.glob(R+"/L_pfb_2bea23e5/*/live/epoch_*/run/result.json")):
    a=json.load(open(f)).get("arguments") or {}; n+=1
    p=[]
    if "scenes_ct1_a770031" not in str(a.get("scene_file")): p.append(("scene_file",a.get("scene_file")))
    if "slam_bundles" not in str(a.get("slam_map_bundle")): p.append(("slam_map_bundle",a.get("slam_map_bundle")))
    for k,v in (("forklift_urdf","/home/projects/forklift/artifacts/20261013_fork_spacing/dd_cfg/old/dls08_measured/forklift.urdf"),("base_scene","/home/projects/forklift/artifacts/20261013_fork_spacing/base_scene/scene.usda")):
        if a.get(k) not in (None,v): p.append((k,a.get(k)))
    if p: bad+=1; print("MISMATCH",f,p)
print("epoch records",n,"mismatched",bad)
for k in (1,2):
    rows=[json.loads(l) for l in open(R+f"/bundle_L{k}_pfb_2bea23e5/results.jsonl")]
    print("bundle",k,"row keys",sorted(rows[0])[:40])
    for r in rows:
        pc={x:r.get(x) for x in r if "pace" in x or "ratio" in x or "gap" in x}
        print(" ",r.get("name"),r.get("status"),r.get("rc"),pc)
PYEOF
ls $R/bundle_L1_pfb_2bea23e5 >> L_paths_pace.txt
echo "AGG_L_DONE $(date -Is)"
