#!/bin/bash
#SBATCH -J uc_cost
#SBATCH -p compute
#SBATCH -c 4
#SBATCH --mem=12G
#SBATCH -t 03:00:00
#SBATCH -o /home/projects/forklift/test_runs/live/%x/cost-%j.log
# The unconfirmed position plan's cost measurement (plan 1.3) on seeds 4 and 5, before any implementation: the start
# check over the unattended-confirmed starts and over F1's wrong confirmations, B-P1b's frozen matcher config. Four
# runs at once, then the summary. The job name is the output folder under test_runs/live (it holds the inputs from
# make_believed.py); SNAP the snapshot: uc_cost on uc_7a185af9 (proxy pools), uc_cost2 on the review fix.
set -u
P=/home/projects/forklift
D=$P/test_runs/live/$SLURM_JOB_NAME
cd $P/snapshots/${SNAP:?}
PY=$P/.venv/bin/python
CFG="$(cat docs/validation/assets/b-p1b/match_config.json)"
B4=$P/test_runs/live/bp1b_survey/hall_s4_G0/bundle C4=$P/test_runs/live/f1_dev/cap_n103/capture.npz
B5=$P/test_runs/live/bp2d_survey/hall_s5b_G0/bundle C5=$P/test_runs/live/f1_s5/cap_n104/capture.npz
echo "start $(date -Is)"
$PY tools/unconfirmed_cost.py run --bundle $B4 --capture $C4 --believed $D/s4_starts.jsonl --config "$CFG" --out $D/s4_starts_rows.jsonl &
$PY tools/unconfirmed_cost.py run --bundle $B5 --capture $C5 --believed $D/s5_starts.jsonl --config "$CFG" --out $D/s5_starts_rows.jsonl &
$PY tools/unconfirmed_cost.py run --bundle $B4 --capture $C4 --believed $D/s4_wrong.jsonl --config "$CFG" --out $D/s4_wrong_rows.jsonl &
$PY tools/unconfirmed_cost.py run --bundle $B5 --capture $C5 --believed $D/s5_wrong.jsonl --config "$CFG" --out $D/s5_wrong_rows.jsonl &
wait
echo "runs done $(date -Is)"
$PY tools/unconfirmed_cost.py summary --rows $D/s4_starts_rows.jsonl --rows $D/s5_starts_rows.jsonl \
  --rows $D/s4_wrong_rows.jsonl --rows $D/s5_wrong_rows.jsonl --out $D/summary.json
echo "end $(date -Is)"
