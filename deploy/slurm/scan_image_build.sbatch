#!/bin/bash
#SBATCH --job-name=scan_image_build
#SBATCH --partition=compute
#SBATCH --cpus-per-task=4
#SBATCH --mem=4G
#SBATCH --time=00:40:00
# Build the A2M12 driver image with forklift patch 0001 (PR 7a-2, docs/design/2026-11-25-real-sources-pr7.md 4):
# deploy/ros2/Dockerfile.scan over forklift/ros2-dev:jazzy, tagged forklift/ros2-scan:jazzy-p0001-<series> (<series>
# the first 8 hex of the patch files' sha256), and check the build: the node binary carries the scan_sdk topic and the
# SDK's first-sample call. Records in OUT: build.log, image.txt, check.txt. Docker builds run outside the job's cgroup.
#   sbatch deploy/slurm/scan_image_build.sbatch OUT        (from the repository root -- a snapshot)
set -euo pipefail
OUT=$(realpath -m "$1")
ROOT=$(pwd)
mkdir -p "$OUT"
SERIES=$(cat "$ROOT"/deploy/ros2/sllidar_patches/*.patch | sha256sum | cut -c1-8)
TAG="forklift/ros2-scan:jazzy-p0001-$SERIES"
docker build -f "$ROOT/deploy/ros2/Dockerfile.scan" -t "$TAG" "$ROOT/deploy/ros2" > "$OUT/build.log" 2>&1
docker image inspect -f '{{.Id}}' "$TAG" > "$OUT/image.txt"
# each check its own exit status: a missing binary, topic or patched call fails the job (Codex 7a-2 P3)
docker run --rm --entrypoint bash "$TAG" -c '
  set -euo pipefail
  # every check its own command (set -e does not stop on the left of &&): any one failing ends the job
  bin=$(find /opt/sllidar_ws/install -name sllidar_node -type f | head -1)
  test -n "$bin"
  test -x "$bin"
  echo "binary $bin"
  grep -q scan_sdk "$bin"
  echo "binary has scan_sdk"
  grep -q publish_delay_ms "$bin"
  echo "binary has publish_delay_ms"
  grep -q "@sdk_mono_v1" /opt/sllidar_ws/src/sllidar_ros2/src/sllidar_node.cpp
  echo "node names the format"
  grep -q "grabScanDataHqWithTimeStamps" /opt/sllidar_ws/src/sllidar_ros2/src/sllidar_node.cpp
  echo "node calls the patched grab"
  grep -q "_scan_last_timestamp_uS" /opt/sllidar_ws/src/sllidar_ros2/sdk/src/sl_lidar_driver.cpp
  echo "sdk keeps the last sample"
' > "$OUT/check.txt" 2>&1
echo "$TAG $(cat "$OUT/image.txt") $(date -Is)" | tee "$OUT/built.txt"
cat "$OUT/check.txt"
