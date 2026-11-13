#!/usr/bin/env python3
"""H's switch in a replay (online-SLAM plan 1-H, docs/plans/2026-11-13-slam-online-bounded.md): the session's unit 0
mapped, its graph serialized into a replay bundle, then the rest of the spliced stream localized on that bundle
(deploy/slurm/slam_h_switch.sbatch runs the phases).

    python tools/slam_h_switch.py pose MAP_DRIVE_JSONL MAP_DRIVE_LOG  # JSON: the start pose used and the estimate
    python tools/slam_h_switch.py summary MAP_DRIVE MAP_LOG LOC_DRIVE LOC_LOG [--steps STEPS_JSON]

``pose`` gives the localization's start pose: plan 1-H switches only while the truck is parked at its start pose
(line_m units end back there), so the pose is the session's own start in the mapping's map frame -- the first mapped
scan's ``map_from_odom`` composed with its odometry -- never the truth. That is a prior the operating rule gives H
(a known start pose), not something the switch finds: the bridge's L4 start check only tests the initial match under
that premise (bootstrap scans on the map's walls, the estimate near this pose) and cannot by itself tell a wrong
place in a repeating or symmetric layout (Codex H tools round 2 P3). Beside it, the record keeps the mapping's
latest estimate at the first localization scan (the correction of its last reply of any status, the driver's
``replies`` line, composed with that scan's odometry, its ``next`` line) and its distance from the start: the last keyframe alone is not where the truck stops (ws1 smoke
18785, scan 1513 against 1604: about 0.10 m and 0.04 rad, over the L4 check), and odometry between the last keyframe
and the stop overshot by about 0.09 m there, so the estimate is recorded, not used.
``summary`` gives the switch wait (the last mapping reply to the first localization
reply, of any status -- the drivers' ``replies`` lines, not the processed lines alone; monotonic clock, one host) against 60 s, the localization's keyframe times against 1 s, and, from
the sbatch's step clocks, where the wait went. Both phases' clocks are ``time.monotonic`` on the job's node.
"""

from __future__ import annotations

import json
import math
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


def _log_line(drive_log: Path, tag: str):
    lines = [x[len(tag) + 1:] for x in Path(drive_log).read_text().splitlines() if x.startswith(tag + " ")]
    if len(lines) != 1 or json.loads(lines[0]) is None:
        raise SystemExit(f"{drive_log}: no single '{tag}' line")
    return json.loads(lines[0])


def next_scan(drive_log: Path) -> dict:
    return _log_line(drive_log, "next")  # only a phase run with --end-unit has one


def replies(drive_log: Path) -> dict:
    out = _log_line(drive_log, "replies")
    if out.get("first") is None or out.get("last") is None:
        raise SystemExit(f"{drive_log}: no reply at all")
    return out


def start_pose(map_drive: Path, drive_log: Path) -> dict:
    rows, nxt, last = _rows(map_drive), next_scan(drive_log), replies(drive_log)["last"]
    first = rows[0]
    if nxt["k"] <= last["k"]:
        raise SystemExit(f"the next scan {nxt['k']} is not after the last answered {last['k']}")
    used = compose(tuple(first["map_from_odom"]), tuple(first["odom_base"]))
    estimate = compose(tuple(last["map_from_odom"]), tuple(nxt["odom"]))
    dyaw = math.atan2(math.sin(estimate[2] - used[2]), math.cos(estimate[2] - used[2]))
    return {"used": list(used), "pose": ",".join(f"{v:.6f}" for v in used), "first_mapped_scan": first["scan_id"],
            "estimate_at_next_scan": list(estimate), "next_scan": nxt["k"], "last_answered_scan": last["k"],
            "last_answered_status": last["status"], "last_processed_scan": rows[-1]["scan_id"],
            "estimate_from_start_m": math.hypot(estimate[0] - used[0], estimate[1] - used[1]),
            "estimate_from_start_rad": abs(dyaw)}


def summarize(map_drive: Path, map_log: Path, loc_drive: Path, loc_log: Path, *, steps: dict | None = None) -> dict:
    m, loc = _rows(map_drive), _rows(loc_drive)
    m_rep, l_rep = replies(map_log), replies(loc_log)
    m_units = sorted({r["unit"] for r in m})
    l_units = sorted({r["unit"] for r in loc})
    if m_units != [0]:
        raise SystemExit(f"{map_drive}: the mapping phase sent units {m_units}, not [0]")
    if min(l_units) < 1:
        raise SystemExit(f"{loc_drive}: the localization phase sent unit 0")
    if m_rep["last"]["unit"] != 0 or l_rep["first"]["unit"] < 1:
        raise SystemExit("the phases' first and last replies are not unit 0 then a later unit")
    last_map = m_rep["last"]["recv_mono"]
    first_loc = l_rep["first"]["recv_mono"]
    if first_loc <= last_map:
        raise SystemExit("a localization reply came before the mapping phase ended")
    walls = [float(r["slam_wall_s"]) for r in loc]
    wait = first_loc - last_map
    over = sum(w >= AFTER_KEYFRAME_MAX_S for w in walls)
    return {"switch_wait_s": wait, "switch_wait_max_s": SWITCH_WAIT_MAX_S, "switch_pass": wait <= SWITCH_WAIT_MAX_S,
            "after_keyframes": len(walls), "after_keyframe_max_s": max(walls),
            "after_keyframes_at_or_over_1s": over, "after_pass": over == 0,
            "mapping_units": m_units, "localization_units": l_units,
            "mapping_replies": m_rep["by_status"], "localization_replies": l_rep["by_status"],
            "last_mapping_reply": m_rep["last"]["k"], "first_localization_reply": l_rep["first"]["k"],
            "steps_from_last_mapping_reply_s": {k: v - last_map for k, v in (steps or {}).items()}}


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if argv[:1] == ["pose"] and len(argv) == 3:
        print(json.dumps(start_pose(Path(argv[1]), Path(argv[2]))))
        return 0
    if argv[:1] == ["summary"] and len(argv) in (5, 7):
        steps = json.loads(Path(argv[6]).read_text()) if len(argv) == 7 and argv[5] == "--steps" else None
        print(json.dumps(summarize(*(Path(x) for x in argv[1:5]), steps=steps), indent=1))
        return 0
    raise SystemExit(__doc__)


if __name__ == "__main__":
    raise SystemExit(main())
