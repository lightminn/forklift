#!/usr/bin/env python3
"""H's switch in a replay (online-SLAM plan 1-H, docs/plans/2026-11-13-slam-online-bounded.md): the session's unit 0
mapped, its graph serialized into a replay bundle, then the rest of the spliced stream localized on that bundle
(deploy/slurm/slam_h_switch.sbatch runs the phases).

    python tools/slam_h_switch.py pose MAP_DRIVE_JSONL         # x,y,yaw: the mapping's own last pose estimate
    python tools/slam_h_switch.py summary MAP_DRIVE LOC_DRIVE [--steps STEPS_JSON]

``pose`` is what a live switch would know at the start pose -- the mapping's last ``map_from_odom`` composed with that
scan's odometry -- never the truth. ``summary`` gives the switch wait (the last mapping reply to the first localization
reply, the driver's monotonic clock, one host) against 60 s, the localization's keyframe times against 1 s, and, from
the sbatch's step clocks, where the wait went. Both phases' clocks are ``time.monotonic`` on the job's node.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slam_remap_drive import compose  # noqa: E402

SWITCH_WAIT_MAX_S = 60.0  # plan 1-H, before results
AFTER_KEYFRAME_MAX_S = 1.0  # plan 1-H: every keyframe after the switch under 1 s


def _rows(path: Path) -> list:
    rows = [json.loads(x) for x in Path(path).read_text().splitlines() if x.strip()]
    if not rows:
        raise SystemExit(f"{path}: no processed replies")
    return rows


def start_pose(map_drive: Path) -> str:
    last = _rows(map_drive)[-1]
    x, y, yaw = compose(tuple(last["map_from_odom"]), tuple(last["odom_base"]))
    return f"{x:.6f},{y:.6f},{yaw:.6f}"


def summarize(map_drive: Path, loc_drive: Path, *, steps: dict | None = None) -> dict:
    m, loc = _rows(map_drive), _rows(loc_drive)
    m_units = sorted({r["unit"] for r in m})
    l_units = sorted({r["unit"] for r in loc})
    if m_units != [0]:
        raise SystemExit(f"{map_drive}: the mapping phase sent units {m_units}, not [0]")
    if min(l_units) < 1:
        raise SystemExit(f"{loc_drive}: the localization phase sent unit 0")
    last_map = max(r["recv_mono"] for r in m)
    first_loc = min(r["recv_mono"] for r in loc)
    if first_loc <= last_map:
        raise SystemExit("a localization reply came before the mapping phase ended")
    walls = [float(r["slam_wall_s"]) for r in loc]
    wait = first_loc - last_map
    over = sum(w >= AFTER_KEYFRAME_MAX_S for w in walls)
    return {"switch_wait_s": wait, "switch_wait_max_s": SWITCH_WAIT_MAX_S, "switch_pass": wait <= SWITCH_WAIT_MAX_S,
            "after_keyframes": len(walls), "after_keyframe_max_s": max(walls),
            "after_keyframes_at_or_over_1s": over, "after_pass": over == 0,
            "mapping_units": m_units, "localization_units": l_units,
            "steps_from_last_mapping_reply_s": {k: v - last_map for k, v in (steps or {}).items()}}


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["pose"] and len(argv) == 2:
        print(start_pose(Path(argv[1])))
        return 0
    if argv[:1] == ["summary"] and len(argv) in (3, 5):
        steps = json.loads(Path(argv[4]).read_text()) if len(argv) == 5 and argv[3] == "--steps" else None
        print(json.dumps(summarize(Path(argv[1]), Path(argv[2]), steps=steps), indent=1))
        return 0
    raise SystemExit(__doc__)


if __name__ == "__main__":
    raise SystemExit(main())
