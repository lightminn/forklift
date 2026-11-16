#!/usr/bin/env bash
# 11-17: bring THIS checkout (the hardware-day worktree) to the latest origin/main -- run it first in the morning so
# the scripts and the firmware session's link check are current. Refuses if the checkout has local edits.
set -euo pipefail
repo="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd -P)"
git -C "$repo" diff --quiet && git -C "$repo" diff --cached --quiet || { echo "local edits in $repo: not updating" >&2; exit 2; }
git -C "$repo" fetch -q origin
git -C "$repo" checkout -q --detach origin/main
git -C "$repo" log --oneline -1
