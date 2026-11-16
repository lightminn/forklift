#!/usr/bin/env bash
# Stage the H743 board check (docs/plans/2026-11-17-h743-board-check.md) into the main checkout's data/h743_check/
# (git-ignored, so it survives worktrees and branch switches): the image with its hash and identity, a copy of the
# Python code at this commit, and the three step scripts.
# usage: tools/h743_check/stage.sh <build dir of the FW_BUILD=hil image>
set -euo pipefail
B=$(cd "$1" && pwd)
REPO=$(git rev-parse --show-toplevel)
MAIN=$(cd "$(git rev-parse --path-format=absolute --git-common-dir)/.." && pwd)
H="$MAIN/data/h743_check"
ELF="$B/boards/weact_h743/fw.elf"
INFO="$B/boards/weact_h743/generated/build_info.h"
grep -q 'FW_BUILD_KIND "hil"' "$INFO" || { echo "not a hil build: $INFO"; exit 1; }
mkdir -p "$H/fw"
cp "$ELF" "$H/fw/fw_hil.elf"
GIT=$(sed -n 's/#define FW_GIT_DESCRIBE "\(.*\)"/\1/p' "$INFO")
{
  echo "forklift-mcu hil locked $GIT"
  echo "built from commit $GIT, $(sed -n 's/#define FW_TOOLCHAIN "\(.*\)"/\1/p' "$INFO")"
  echo "staged from code commit $(git -C "$REPO" rev-parse --short=12 HEAD) on $(date -u +%FT%TZ)"
} > "$H/fw/BUILD.txt"
(cd "$H/fw" && sha256sum fw_hil.elf > SHA256SUMS)
rm -rf "$H/code"
mkdir -p "$H/code"
git -C "$REPO" archive HEAD src tools | tar -x -C "$H/code"
cp "$REPO"/tools/h743_check/[123]_*.sh "$H/"
chmod +x "$H"/[123]_*.sh
echo "staged in $H"
cat "$H/fw/BUILD.txt" "$H/fw/SHA256SUMS"
