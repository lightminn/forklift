#!/bin/bash
#SBATCH --job-name=scan_image_build
#SBATCH --partition=compute
#SBATCH --cpus-per-task=4
#SBATCH --mem=4G
#SBATCH --time=00:40:00
# Build the A2M12 driver image (forklift patches 0001 + 0002, PR 7a-2 / 7a-3) for amd64: deploy/ros2/Dockerfile.scan
# over forklift/ros2-dev:jazzy, tagged config/real/lidar.yaml scan_image.amd64 -- the job computes the tag from the
# inputs (tools/scan_image.py) and stops before building when lidar.yaml names another. The image build runs the
# SDK's arrival test (ctest) and fails with it. Records in OUT: build.log, image.txt (ID), arch.txt, check.txt (the v2
# contract in the binary and the sources, then the ctest output), built.txt. Docker builds run outside the job's
# cgroup.
#   sbatch deploy/slurm/scan_image_build.sbatch OUT        (from the repository root -- a snapshot)
set -euo pipefail
OUT=$(realpath -m "$1")
ROOT=$(pwd)
mkdir -p "$OUT"
TAG=$(python3 "$ROOT/tools/scan_image.py" check --arch amd64)  # exits non-zero when lidar.yaml differs
docker build -f "$ROOT/deploy/ros2/Dockerfile.scan" -t "$TAG" "$ROOT/deploy/ros2" > "$OUT/build.log" 2>&1
docker image inspect -f '{{.Id}}' "$TAG" > "$OUT/image.txt"
docker image inspect -f '{{.Architecture}}' "$TAG" > "$OUT/arch.txt"
# each check its own exit status: a missing binary, topic, parameter or patched call fails the job (Codex 7a-2 P3)
docker run --rm --entrypoint bash "$TAG" -c '
  set -euo pipefail
  # every check its own command (set -e does not stop on the left of &&): any one failing ends the job
  ws=/opt/sllidar_ws
  bin=$(find $ws/install -name sllidar_node -type f | head -1)
  test -n "$bin"
  test -x "$bin"
  echo "binary $bin"
  grep -q scan_sdk_info "$bin"
  echo "binary has scan_sdk_info"
  grep -q forklift_run_id "$bin"
  echo "binary has forklift_run_id"
  grep -q publish_delay_ms "$bin"
  echo "binary has publish_delay_ms"
  grep -q FORKLIFT_SDK_RX_LOG "$bin"
  echo "binary has the receive record"
  grep -q "@sdk_mono_v2#" $ws/src/sllidar_ros2/src/sllidar_node.cpp
  echo "node names the v2 format"
  if grep -q "@sdk_mono_v1" $ws/src/sllidar_ros2/src/sllidar_node.cpp; then echo "node still names v1"; exit 1; fi
  echo "node no longer names v1"
  grep -q grabScanDataHqWithArrival $ws/src/sllidar_ros2/src/sllidar_node.cpp
  echo "node calls the arrival grab"
  grep -q onDecodeDataAt $ws/src/sllidar_ros2/sdk/src/sl_async_transceiver.cpp
  echo "the decoder hands the arrival on"
  grep -q "all checks passed" $ws/forklift_ctest.txt
  grep -q "100% tests passed" $ws/forklift_ctest.txt
  echo "sdk arrival test passed in the build"
  echo "--- ctest"
  cat $ws/forklift_ctest.txt
' > "$OUT/check.txt" 2>&1
echo "$TAG $(cat "$OUT/image.txt") $(cat "$OUT/arch.txt") $(date -Is)" | tee "$OUT/built.txt"
cat "$OUT/check.txt"
