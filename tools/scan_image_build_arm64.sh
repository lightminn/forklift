#!/usr/bin/env bash
# The A2M12 driver image for the Jetson (arm64), cross-built on the laptop under qemu (as on 2026-11-16, 202 s), then
# saved for the LAN copy the hwday scripts load (7a-3 plan 5):
#   bash tools/scan_image_build_arm64.sh [OUT folder, default data/hwday/scan_image_<date>]
# The tag is config/real/lidar.yaml scan_image.arm64; it is computed from the inputs (tools/scan_image.py) and the
# build stops before it starts when lidar.yaml names another. The image build runs the SDK's arrival test under qemu
# and fails with it. Records in OUT: build.log, image.txt (ID), arch.txt, ctest.txt, built.txt; the tarball goes to
# data/hwday/<tag>.tar.zst. The base is the arm64 one deploy/ros2/scan_image_base.txt records (part of the tag's
# inputs); the build stops unless its reference has that ID here.
# Minutes of laptop load: tell the lead before running it.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
cd "$repo_root"
tag="$(python3 tools/scan_image.py check --arch arm64)"
tarball="data/hwday/$(python3 tools/scan_image.py tarball --arch arm64)"
read -r base base_id < <(python3 tools/scan_image.py base --arch arm64)
have="$(docker image inspect -f '{{.Id}}' "$base")"
[[ "$have" == "$base_id" ]] || { echo "base $base is $have here, not $base_id" >&2; exit 1; }
out="${1:-data/hwday/scan_image_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$out" data/hwday
echo "building $tag over $base (records in $out)" >&2
docker buildx build --platform linux/arm64 --build-arg "BASE_IMAGE=${base}" -f deploy/ros2/Dockerfile.scan \
    -t "$tag" --load deploy/ros2 > "$out/build.log" 2>&1
docker image inspect -f '{{.Id}}' "$tag" > "$out/image.txt"
docker image inspect -f '{{.Architecture}}' "$tag" > "$out/arch.txt"
[[ "$(cat "$out/arch.txt")" == arm64 ]] || { echo "built for $(cat "$out/arch.txt"), not arm64" >&2; exit 1; }
docker run --rm --platform linux/arm64 --entrypoint cat "$tag" /opt/sllidar_ws/forklift_ctest.txt > "$out/ctest.txt"
grep -q "all checks passed" "$out/ctest.txt"
docker save "$tag" | zstd -T0 -q -o "$tarball"
echo "$tag $(cat "$out/image.txt") arm64 $tarball $(date -Is)" | tee "$out/built.txt"
