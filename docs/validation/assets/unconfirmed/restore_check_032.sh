#!/usr/bin/env bash
# The archive's restore check (revision plan 9.23 item 5 (나)), run before any ws1 original of the unconfirmed position
# batches is deleted: restore 032 full's captures and the URDF from their gdrive archives into a fresh folder, recompute
# the 2x2 with tools/start_check_cross.py, and compare with docs/validation/assets/unconfirmed/diag_cross_032f.json.
#
#   RCLONE=~/bin/rclone DIAG_ARCHIVE=gdrive:<folder>/<unit>.tar.zst URDF_ARCHIVE=gdrive:<folder>/<unit>.tar.zst \
#     bash docs/validation/assets/unconfirmed/restore_check_032.sh <repo checkout at or after the tool PR>
#
# Paths inside the archives are relative to /home/projects/forklift (archive_map.md). Exit 0 only when the recomputed
# cross equals the committed one. md5 is the upload check only; this is the restore check.
set -euo pipefail
REPO=${1:?repo checkout}
RCLONE=${RCLONE:-rclone}
PY=${PY:-python3}
W=$(mktemp -d)
trap 'rm -rf "$W"' EXIT
RUN=test_runs/live/uc_diag_s4/s/d_val3_032_full/epoch_1/run
URDF=artifacts/20261013_fork_spacing/dd_cfg/old/dls08_measured/forklift.urdf
"$RCLONE" cat "${DIAG_ARCHIVE:?}" | zstd -dc | tar -xf - -C "$W" "$RUN/pose_diag_premove.npz" "$RUN/prepare_input_1.pkl" "$RUN/prepare_input_1.npz"
"$RCLONE" cat "${URDF_ARCHIVE:?}" | zstd -dc | tar -xf - -C "$W" "$URDF"
# the restored files must be the ws1 originals (archive_map.md 3)
(cd "$W" && sha256sum -c - <<SUMS
00acbf927aa38087f0676ae40083921b725d678ceb4eb059e79ebae0f3d2ed6d  $RUN/pose_diag_premove.npz
787865e0e43712d1d7ea9b0b6da620b0f715d15217b9d06b8beadd9832797d47  $RUN/prepare_input_1.pkl
4123fc0f41234634bfc585ab765cd9196defcbcd60b0fa74170c94ee69cc58bf  $RUN/prepare_input_1.npz
517e8260d0d5257a4381092ce96c6e91d1e58672f7deb44d5f3e86cf3489b760  $URDF
SUMS
)
PYTHONPATH="$REPO/src" "$PY" "$REPO/tools/start_check_cross.py" --premove "$W/$RUN/pose_diag_premove.npz" \
  --prepare "$W/$RUN/prepare_input_1.pkl" --forklift-urdf "$W/$URDF" --rear-axle-offset-m 0.34 --out "$W/cross.json"
"$PY" - "$W/cross.json" "$REPO/docs/validation/assets/unconfirmed/diag_cross_032f.json" <<'PY'
import json, sys
a, b = (json.load(open(p)) for p in sys.argv[1:3])
print("restore check:", "MATCH" if a == b else "DIFFER")
sys.exit(0 if a == b else 1)
PY
