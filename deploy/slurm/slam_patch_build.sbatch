#!/bin/bash
#SBATCH --job-name=slam_patch_build
#SBATCH --partition=compute
#SBATCH --cpus-per-task=8
#SBATCH --mem=8G
#SBATCH --time=02:00:00
# Build the source slam_toolbox images of the online-SLAM comparison (docs/design/2026-11-13-slam-patch-build.md):
# U0 (PATCHES=none) and P1 (PATCHES=all) from deploy/ros2/Dockerfile.slam_patched over forklift/ros2-dev:jazzy.
# Tags: forklift/ros2-slam:jazzy-2.8.5-u0 and forklift/ros2-slam:jazzy-2.8.5-p1-<series>, <series> the first 8 hex of
# the patch files' sha256 (the Dockerfile computes the same). Records in OUT: the build logs, the image ids, the series,
# the toolchain and compile_commands.json copied out of each image. Docker builds run outside the job's cgroup.
#   sbatch deploy/slurm/slam_patch_build.sbatch OUT [u0|p1|both] [SANITIZE]
# Run from the repository root (a snapshot). SLAM_SOURCE=bundle with OUT/slam_toolbox-2.8.5.bundle when GitHub is out
# of reach from ws1.
set -euo pipefail
OUT=$(realpath -m "$1"); WHICH=${2:-both}; SANITIZE=${3:-}
ROOT=$(pwd)
mkdir -p "$OUT"
CTX=$(mktemp -d "$OUT/context.XXXX")
mkdir -p "$CTX/deploy/ros2" "$CTX/tools" "$CTX/slam_source"
cp -r "$ROOT/deploy/ros2/slam_patches" "$CTX/deploy/ros2/"
cp "$ROOT/tools/slam_graph_dump.cpp" "$CTX/tools/"
[ "${SLAM_SOURCE:-github}" = bundle ] && cp "$OUT/slam_toolbox-2.8.5.bundle" "$CTX/slam_source/"
SERIES=$(cat "$ROOT"/deploy/ros2/slam_patches/*.patch 2>/dev/null | sha256sum | cut -c1-8)
build() {  # kind patches tag
  docker build -f "$ROOT/deploy/ros2/Dockerfile.slam_patched" --build-arg PATCHES="$2" \
    --build-arg SLAM_SOURCE="${SLAM_SOURCE:-github}" --build-arg SANITIZE="$SANITIZE" \
    --build-arg JOBS="${SLURM_CPUS_PER_TASK:-8}" -t "$3" "$CTX" \
    > "$OUT/build_$1.log" 2>&1
  docker image inspect -f '{{.Id}}' "$3" > "$OUT/image_$1.txt"
  docker run --rm "$3" bash -c 'cat /opt/slam_patched/series /opt/slam_patched/toolchain.txt /opt/slam_patched/source_log.txt' \
    > "$OUT/meta_$1.txt"
  docker run --rm "$3" cat /opt/slam_patched/compile_commands.json > "$OUT/compile_commands_$1.json"
  echo "$1 $3 $(cat "$OUT/image_$1.txt") $(date -Is)" | tee -a "$OUT/built.txt"
}
SUFFIX=${SANITIZE:+-$SANITIZE}
[ "$WHICH" = u0 ] || [ "$WHICH" = both ] && build u0 none "forklift/ros2-slam:jazzy-2.8.5-u0$SUFFIX"
[ "$WHICH" = p1 ] || [ "$WHICH" = both ] && build p1 all "forklift/ros2-slam:jazzy-2.8.5-p1-$SERIES$SUFFIX"
rm -rf "$CTX"
echo "series $SERIES" | tee "$OUT/series.txt"
