#!/usr/bin/env bash
# librealsense v2.59.2 from source on the Jetson, RSUSB backend (IMU over libusb, no kernel patch), Python
# bindings for ~/forklift/.venv. No sudo here: udev rules are installed separately. Log: ~/src/rs_build.log
#
# Why (hardware day 2026-11-17, docs/validation/2026-11-17-hw-day.md): the PyPI pyrealsense2 2.59.2 wheel saw the
# D435i but not its Motion Module on JetPack R39.2.1 (kernel 6.8.12-tegra), so the IMU record failed; this build did.
# Before it, once, as the user (sudo): the apt packages
#   sudo apt-get install -y git build-essential libssl-dev libusb-1.0-0-dev libudev-dev pkg-config python3-dev
# and after it the udev rule from the source tree, so the camera opens without root:
#   sudo cp ~/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload \
#     && sudo udevadm trigger
# Run it on the Jetson with bash (it is not meant to be sourced); it takes about 20-30 minutes with -j4.
set -euo pipefail
TAG=v2.59.2
SRC=~/src/librealsense
PY=~/forklift/.venv/bin/python
mkdir -p ~/src
if [ ! -d "$SRC/.git" ]; then git clone -q --depth 1 -b "$TAG" https://github.com/IntelRealSense/librealsense.git "$SRC"; fi
# an existing checkout must be that tag and unmodified (the build dir is untracked and not counted)
have=$(git -C "$SRC" describe --tags --exact-match 2>/dev/null || echo "not-a-tag")
[ "$have" = "$TAG" ] || { echo "STOP: $SRC is at $have, not $TAG"; exit 2; }
[ -z "$(git -C "$SRC" status --porcelain --untracked-files=no)" ] || { echo "STOP: $SRC has local changes"; exit 2; }
cmake -S "$SRC" -B "$SRC/build" -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE="$PY" -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
  -DBUILD_TOOLS=OFF -DBUILD_UNIT_TESTS=OFF -DBUILD_WITH_CUDA=OFF -DCHECK_FOR_UPDATES=OFF
cmake --build "$SRC/build" -j4
# point the venv at the built module instead of the PyPI wheel (which has no IMU on this kernel)
"$PY" -m pip uninstall -y -q pyrealsense2  # not installed: pip warns and exits 0; a real failure stops here
SITE=$("$PY" -c 'import site; print(site.getsitepackages()[0])')
MOD=$(find "$SRC/build" -name 'pyrealsense2*.so' -printf '%h\n' -quit)  # -quit, not | head: pipefail and SIGPIPE
echo "$MOD" > "$SITE/pyrealsense2_src.pth"
# the module the venv imports must be the one just built, not a wheel left behind
"$PY" - "$SRC/build" <<'PYEOF'
import os, sys
import pyrealsense2 as rs
path = os.path.realpath(rs.__file__ or "")
build = os.path.realpath(sys.argv[1])
print("imported", path)
sys.exit(0 if path.startswith(build + os.sep) else "STOP: pyrealsense2 is not the source build")
PYEOF
echo RS_BUILD_DONE
