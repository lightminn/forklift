#!/bin/bash
#SBATCH --job-name=pfb_agg
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=6G
#SBATCH --time=00:30:00
# post-fix baseline RUNBOOK steps 2-4 and the R/S tables (no live). Code: the branch copy in $CODE (not the snapshot).
set -u
P=/home/projects/forklift; PY=$P/.venv/bin/python; CODE=${CODE:?}; OUT=${OUT:?}
T=$CODE/docs/validation/2026-11-15-post-fix-baseline; PRE=$P/test_runs/tmp_restore/pfb_prefix/test_runs; R=$P/test_runs/pfb
mkdir -p $OUT; cd $OUT
export PYTHONPATH=$CODE/src
echo "== step 2 input re-hash $(date -Is)" | tee inputs_after.txt
( cd $P && sha256sum artifacts/20261013_fork_spacing/base_scene/scene.usda artifacts/20261013_fork_spacing/dd_cfg/old/dls08_measured/forklift.urdf test_runs/s3/yolo_runs/yolo11s-seg_p15/weights/best.onnx test_runs/live/ct1_queue.sbatch test_runs/riibotics/scenes_ct1_a770031/line_m_session.json test_runs/riibotics/scenes_ct1_a770031/random_m_session.json
  echo "scene_dir_p1c $(find test_runs/riibotics/scenes_p1c_81a007b1 -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -c1-16) files $(find test_runs/riibotics/scenes_p1c_81a007b1 -type f | wc -l)"
  for b in artifacts/slam_bundles/line_m_team_spec_quantized_0ff9e3bdff13 artifacts/slam_bundles/random_m_team_spec_quantized_0facac3eea66; do echo "bundle $b $(find $b -type f -print0 | sort -z | xargs -0 sha256sum | sha256sum | cut -c1-16) files $(find $b -type f | wc -l)"; done ) >> inputs_after.txt
$PY - <<PYEOF > paths_check.txt
import json,glob
want={"scene_file_dir":"$P/test_runs/riibotics/scenes_p1c_81a007b1","forklift_urdf":"$P/artifacts/20261013_fork_spacing/dd_cfg/old/dls08_measured/forklift.urdf","base_scene":"$P/artifacts/20261013_fork_spacing/base_scene/scene.usda","recognition_onnx":"$P/test_runs/s3/yolo_runs/yolo11s-seg_p15/weights/best.onnx"}
bad=0;n=0
for f in sorted(glob.glob("$R/R_pfb_2bea23e5/*/run/result.json")+glob.glob("$R/Rrep_pfb_2bea23e5/*/run/result.json")+glob.glob("$R/S_pfb_2bea23e5/*/run/result.json")):
    a=json.load(open(f)).get("arguments") or {}; n+=1
    sf=str(a.get("scene_file") or ""); probs=[]
    if not sf.startswith(want["scene_file_dir"]): probs.append(("scene_file",sf))
    for k in ("forklift_urdf","base_scene","recognition_onnx"):
        if k in a and a[k] is not None and str(a[k])!=want[k]: probs.append((k,a[k]))
    if probs: bad+=1; print("MISMATCH",f,probs)
print("checked",n,"mismatched",bad)
PYEOF
echo "== step 3" 
$PY $T/baseline_summary.py $R/R_pfb_2bea23e5 --mode overlay --compare $PRE/lcr/cmp_a2b91d2d --compare-mode prefix --out R_vs_lcrB0.json --fields-out R_fields.json.gz
$PY $T/baseline_summary.py $R/R_pfb_2bea23e5 --mode overlay --compare $PRE/k1/cmp_c7701296 --compare-mode prefix --match-compare --out R_vs_k1cK1.json
$PY $T/baseline_summary.py $R/R_pfb_2bea23e5 --mode overlay --compare $PRE/k1r/k1r_7744f285 --compare-mode overlay-noheader --match-compare --out R_vs_k1rK1.json
$PY $T/baseline_summary.py $R/R_pfb_2bea23e5 --mode overlay --compare $R/S_pfb_2bea23e5 --compare-mode stage --match-compare --out R_vs_S.json
$PY $T/baseline_summary.py $R/S_pfb_2bea23e5 --mode stage --out S_summary.json --fields-out S_fields.json.gz
echo "== step 4"
for x in random_1_s0 random_1_s1 line_2_s0 aisle_rack_s0; do
  $PY $CODE/tools/deterministic_compare.py $R/R_pfb_2bea23e5/pfb_R_$x/run $R/Rrep_pfb_2bea23e5/pfb_Rrep_$x/run --json rep_$x.json > rep_$x.txt 2>&1; echo "rep $x exit $?"
done
$PY $T/report_tables.py --riib 'R vs 곡률 B0=R_vs_lcrB0.json' --riib 'R vs K1 비교 K1(40)=R_vs_k1cK1.json' --riib 'R vs K1 재확인 K1(40)=R_vs_k1rK1.json' --riib 'R vs S(40)=R_vs_S.json' --determinism rep_random_1_s0.json rep_random_1_s1.json rep_line_2_s0.json rep_aisle_rack_s0.json --out tables_riib.md
echo "AGG_DONE $(date -Is)"
