#!/bin/bash
# Submit the near-capture frequency batch, two bundles, from the snapshot (docs/plans/2026-11-07-near-capture-frequency.md).
set -eu
SNAP=/home/projects/forklift/snapshots/nf_46d8929d
B=/home/projects/forklift/test_runs/near_freq/nf_46d8929d
cd "$SNAP"
export BUNDLE_ARGS="--pace-floor 0.335 --pace-gap-p99-s 0.12 --retries 0 --run-timeout-s 4500"
A=$(sbatch --parsable -J nf_A -o "$B/slurm-%j.out" $(python3 tools/isaac_bundle.py sizes live --runs 9 --jobs 1 | grep -- --gres) \
  deploy/slurm/isaac_bundle.sbatch "$B/RUNS_A.jsonl" "$B/bA" live)
C=$(sbatch --parsable -J nf_B -o "$B/slurm-%j.out" $(python3 tools/isaac_bundle.py sizes live --runs 11 --jobs 1 | grep -- --gres) \
  deploy/slurm/isaac_bundle.sbatch "$B/RUNS_B.jsonl" "$B/bB" live)
echo "$A $C" | tee "$B/JOBS.txt"
squeue -h -o "%i %j %T %C %m %b %l %r"
