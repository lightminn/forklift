"""The C2 injection plan's roll-out (docs/plans/2026-11-03-near-end-stand-c2-injection.md 1.3): every run's near
re-observation leg under the registered detour, closed loop on the D2' plant, from the path's first pose.

The near leg is ``run/paths.json`` ``observe`` where its end is ``near_capture.waypoint``; otherwise (a run that
timed out on the near leg: paths.json is not rewritten when the near leg is installed) ``tracking_failure.path``.
The tracker is the repository's, armed by ``near_end_detour.decide`` and offset by ``set_input_offset`` -- the
runner's own decision and the tracker's own clear. The command chain is the runner's (ackermann_command, creep,
slew_cap_stop with accel 0.8 / decel 1.5, steering command rate 1.0 rad/s); the plant is D2' (#261: wheel steering
lag 0.17 s, curvature from the wheel angles x 0.94 forward / 1.12 reverse, speed x 0.94). Geometry only: no
permission, obstacles, SLAM hold or cusp watch -- an exposure expectation, not a result.

    PYTHONPATH=src python tools/near_end_detour_sim.py RUN_ROOT [OUT.json]
"""

import json
import math
import sys
from dataclasses import fields, replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "sim/isaac"))
import near_end_detour as nd  # noqa: E402
from forklift_core.control.path_tracking import (  # noqa: E402
    AckermannGeometry,
    RearAxlePathTracker,
    TrackerConfig,
    ackermann_command,
)
from forklift_core.control.speed_chain import creep, slew_cap_stop  # noqa: E402

TAU, L, GF, GR, SG, DT = 0.17, 0.6645, 0.94, 1.12, 0.94, 1 / 120
NAMES = {f.name for f in fields(TrackerConfig)}


def near_leg(run: Path) -> tuple[dict, str]:
    result = json.loads((run / "result.json").read_text())
    observe = json.loads((run / "paths.json").read_text())["observe"]
    waypoint = (result.get("near_capture") or {}).get("waypoint")
    if waypoint is not None and np.allclose(observe["poses"][-1], waypoint, atol=1e-9):
        leg, source = observe, "paths.json"
    else:
        leg, source = result["tracking_failure"]["path"], "tracking_failure"
    config = result["tracker_configs"]["observe"]
    return {"poses": leg["poses"], "directions": leg["directions"], "curvatures_inv_m": leg["curvatures_inv_m"],
            "config": {k: v for k, v in config.items() if k in NAMES},
            "drive_geometry": result["arguments"]["drive_geometry"]}, source


def roll(leg: dict) -> dict:
    poses = np.array(leg["poses"])
    cfg = replace(TrackerConfig(**leg["config"]), cusp_heading_pending_rad=None)
    geom = AckermannGeometry(**leg["drive_geometry"])
    tracker = RearAxlePathTracker(poses, np.array(leg["directions"]), np.array(leg["curvatures_inv_m"]), cfg)
    truth, act, sc = poses[0].copy(), np.zeros(2), np.zeros(2)
    v = prev = 0.0
    a = math.exp(-DT / TAU)
    status, stood, decision, max_distance = "timeout", 0, None, 0.0
    for _ in range(int(90 / DT)):
        if (decision is None and tracker.leg_index == len(tracker._leg_ends) - 1
                and tracker.remaining_to_goal_m() <= nd.R1_M):
            decision = nd.decide(tracker)
            if decision["event"] == "armed":
                tracker.set_input_offset(decision["dx_left_m"], decision["dpsi_rad"], nd.R2_M)
        if tracker.input_offset_active():
            max_distance = max(max_distance, nd.path_distance_m(tracker, truth[:2]))
        cmd = tracker.update(truth.copy(), v, DT)
        if cmd.status in ("arrived", "failed"):
            status = cmd.status + ("" if cmd.failure is None else ":" + cmd.failure)
            break
        if cmd.at_cusp and cmd.speed_mps == 0.0 and abs(v) < cfg.stop_speed_mps and tracker.release_cusp(True):
            cmd = tracker.update(truth.copy(), v, DT)
        final = tracker.leg_index == len(tracker._leg_ends) - 1
        target = np.asarray(ackermann_command(cmd.speed_mps, cmd.curvature_inv_m, geom).steering_rad)
        ws = creep(cmd.speed_mps, float(np.max(np.abs(target - act))))
        ws = slew_cap_stop(prev, ws, decel_mps2=1.5, accel_mps2=0.8, dt=DT)
        prev = ws
        sc += np.clip(target - sc, -DT, DT)
        act = a * act + (1 - a) * sc
        v = ackermann_command(ws, cmd.curvature_inv_m, geom).speed_mps * SG
        kappa = (math.tan(act[0]) + math.tan(act[1])) / (2 * L) * (GF if v >= 0 else GR)
        change = v * kappa * DT
        heading = truth[2] + change / 2
        truth += [v * DT * math.cos(heading), v * DT * math.sin(heading), change]
        if final and abs(v) < 1e-4 and cmd.speed_mps == 0.0 and tracker.remaining_to_goal_m() <= 1e-6:
            stood += 1
            if stood > 360:
                status = "end_stand"
                break
        else:
            stood = 0
    goal = poses[-1]
    along_axis = np.array([math.cos(goal[2]), math.sin(goal[2])])
    off = truth[:2] - goal[:2]
    yaw = math.atan2(math.sin(truth[2] - goal[2]), math.cos(truth[2] - goal[2]))
    out = {"status": status, "decision": decision, "along_m": float(off @ along_axis),
           "across_m": float(off @ [-along_axis[1], along_axis[0]]), "yaw_rad": yaw,
           "max_path_distance_m": max_distance, "events": tracker.input_offset_events}
    out["outside"] = status == "end_stand" and (math.hypot(*off) > 0.10 or abs(yaw) > 0.10)
    return out


def main() -> None:
    root = Path(sys.argv[1])
    out = {}
    for run in sorted(p.parent for p in root.glob("*/run/result.json")):
        leg, source = near_leg(run)
        out[run.parent.name] = {"source": source, **roll(leg)}
        r = out[run.parent.name]
        print(run.parent.name, source, (r["decision"] or {}).get("event"), (r["decision"] or {}).get("reason", ""),
              r["status"], f"across {1000 * r['across_m']:+.1f} mm yaw {r['yaw_rad']:+.4f}", r["outside"], flush=True)
    armed = [r for r in out.values() if (r["decision"] or {}).get("event") == "armed"]
    outside = [r for r in armed if r["outside"]]
    print(f"armed {len(armed)}/{len(out)}, outside {len(outside)}/{len(armed)}")
    if outside:
        for key in ("across_m", "yaw_rad", "along_m", "max_path_distance_m"):
            values = np.abs([r[key] for r in outside])
            print(f"|{key}| {values.min():.4f}-{values.max():.4f}")
    if len(sys.argv) > 2:
        Path(sys.argv[2]).write_text(json.dumps(out, indent=1))


if __name__ == "__main__":
    main()
