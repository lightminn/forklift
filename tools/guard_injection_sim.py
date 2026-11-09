"""The guard (2) injection plan's D search (docs/plans/2026-11-09-live-riib-guard-injection.md 1): every intact empty
leg of the v2 main batch's arm B records under the registered heading / ramp grid, closed loop on the D2' plant with the
low-clearance guard's cap in the command chain at its default settings. Geometry only (no permission, obstacles, SLAM
or cusp watch): an exposure expectation, not a result.

    PYTHONPATH=src python tools/guard_injection_sim.py SESSIONS_DIR OUT.json

SESSIONS_DIR holds the main batch's session folders (``ab_<scene>_s<seed>_B/live/epoch_1/run``)."""

from __future__ import annotations

import json
import math
import sys
from dataclasses import fields, replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim/isaac"))
from insertion_geometry import read_carriage_limit_m, read_chassis_reference_m, read_fork_blades_m  # noqa: E402

from forklift_core.control.drive_permission import StoppingModel  # noqa: E402
from forklift_core.control.low_clearance import LegClearance, LowClearanceConfig, guard_cap, shape_clearance  # noqa: E402
from forklift_core.control.path_tracking import (  # noqa: E402
    AckermannGeometry,
    RearAxlePathTracker,
    TrackerConfig,
    ackermann_command,
)
from forklift_core.control.speed_chain import creep, slew_cap_stop  # noqa: E402
from forklift_core.planning.geometry import Footprint, Rectangle  # noqa: E402
from forklift_core.planning.pallet_mission import SEARCH_TAG, SLOT_TAG  # noqa: E402

TAU, L, GF, GR, SG, DT = 0.17, 0.6645, 0.94, 1.12, 0.94, 1 / 120  # D2' (#261), as tools/near_end_detour_sim.py
NAMES = {f.name for f in fields(TrackerConfig)}
URDF = ROOT / "sim/models/dls08_measured/forklift.urdf"
STOPPING = StoppingModel(0.15, 1.5, 0.05)  # config/obstacle_layer_single_team_spec.yaml permission
PLANE_M = 1.05
PALLET_DEPTH_M, PALLET_WIDTH_M = 0.6, 0.8  # EPAL 6: the real outline at the inflated one's centre and heading
FORK_CLEAR_M = 0.36  # withdraw: past the 0.346 m fork length
HOLD_M, REVERSE_M, CLEAR_WITHIN_M = 3.0, 0.30, 0.10
GRID = ([("withdraw", "heading", FORK_CLEAR_M, psi, None) for psi in (-0.30, -0.20, -0.10, 0.10, 0.20, 0.30)]
        + [("withdraw", "ramp", FORK_CLEAR_M, d, 0.10) for d in (0.05, 0.10, 0.15)]
        + [("return_home", "ramp", s_a, d, r) for s_a in (0.0, 0.5, 1.0) for d in (0.1, 0.2, 0.3) for r in (0.5, 1.0)])
RETURN_MIN_M = 10.0


def unloaded_shape():
    """The truck unloaded as run_transport builds it: the body to the carriage and the two fork blades."""
    tip, rear_x = read_chassis_reference_m(URDF)
    carriage = read_carriage_limit_m(URDF)
    return [(Footprint(tip - carriage, 0.17, 0.36), 0.0, 0.0)] + [
        (Footprint((x1 - x0) / 2, (x1 - x0) / 2, (y1 - y0) / 2), (y0 + y1) / 2, (x0 + x1) / 2 + abs(rear_x))
        for x0, x1, y0, y1 in read_fork_blades_m(URDF)]


def stage_props(props: list, stage: str) -> list:
    """low_props' rule on recorded props: no taller than the LiDAR plane; "search" leaves out the slot copies, "slot"
    the search ones."""
    drop = SLOT_TAG if stage == "search" else SEARCH_TAG
    return [Rectangle(**p["rectangle"]) for p in props
            if p["asset"]["height_m"] <= PLANE_M and not p["asset"]["uri"].endswith(drop)]


def real_outline(rect: Rectangle) -> Rectangle:
    """The pallet itself inside an inflated outline: same centre and heading, EPAL 6's size."""
    return Rectangle(rect.x_m, rect.y_m, PALLET_DEPTH_M, PALLET_WIDTH_M, rect.yaw_rad)


def neighbour_side(start, goal_yaw: float, rects: list, shape) -> float:
    """+1 when the low prop nearest the start lies to the goal heading's left, -1 to its right."""
    near = min(rects, key=lambda r: shape_clearance(start, shape, [r]))
    left = np.array([-math.sin(goal_yaw), math.cos(goal_yaw)])
    return 1.0 if float((np.array([near.x_m, near.y_m]) - np.asarray(start[:2])) @ left) > 0.0 else -1.0


def ramp_dx(s: float, s_a: float, d: float, r: float, side: float) -> float:
    if s < s_a:
        return 0.0
    return side * d * min(1.0, (s - s_a) / r)


def reverse_clear(pose, shape, rects: list, length_m: float = REVERSE_M, step_m: float = 0.01) -> float:
    """The least clearance to ``rects`` along a straight reverse of ``length_m`` from ``pose``."""
    x, y, yaw = (float(v) for v in pose)
    best = math.inf
    for k in range(int(round(length_m / step_m)) + 1):
        d = k * step_m
        best = min(best, shape_clearance((x - d * math.cos(yaw), y - d * math.sin(yaw), yaw), shape, rects))
    return best


def roll(leg: dict, record_props: list, tracker_config: dict, drive_geometry: dict, phase: str, kind: str, s_a: float,
         size: float, r: float | None, shape) -> dict:
    poses = np.asarray(leg["poses"], dtype=float)
    search, slot = stage_props(record_props, "search"), stage_props(record_props, "slot")
    backoff = leg.get("start_backoff") or {}
    slot_to = float(backoff["length_m"]) if backoff.get("status") == "prefix" else None
    if phase == "withdraw":
        lc = LegClearance(poses, shape, search_props=search, slot_props=slot, slot_from_m=0.0)
        near_rects = slot
    else:
        lc = LegClearance(poses, shape, search_props=search, slot_props=slot, slot_to_m=slot_to)
        near_rects = search + (slot if slot_to else [])
    real = [real_outline(rc) for rc in search + slot]
    cfg = replace(TrackerConfig(**{k: v for k, v in tracker_config.items() if k in NAMES}), cusp_heading_pending_rad=None)
    geom = AckermannGeometry(**drive_geometry)
    tr = RearAxlePathTracker(poses, np.asarray(leg["directions"]), np.asarray(leg["curvatures_inv_m"]), cfg)
    side = neighbour_side(poses[0], float(poses[-1][2]), near_rects, shape)
    lcfg = LowClearanceConfig()
    end_allow = cfg.overshoot_tolerance_m if cfg.overshoot_tolerance_m is not None else cfg.position_tolerance_m
    truth, act, sc = poses[0].copy(), np.zeros(2), np.zeros(2)
    v = prev = 0.0
    a = math.exp(-DT / TAU)
    out = {"side": side, "status": "timeout", "armed_s": None, "zero": None, "capped_ticks": 0, "min_real_m": math.inf}
    heading_set = False
    for k in range(int(90 / DT)):
        s = tr.progress_m()
        if out["zero"] is None and (out["armed_s"] is None or s - out["armed_s"] <= HOLD_M):
            if kind == "heading" and not heading_set and s >= s_a:
                tr.set_input_offset(0.0, size, CLEAR_WITHIN_M)
                heading_set, out["armed_s"] = True, s
            elif kind == "ramp" and s >= s_a:
                out["armed_s"] = s if out["armed_s"] is None else out["armed_s"]
                tr.set_input_offset(ramp_dx(s, s_a, size, r, side), 0.0, CLEAR_WITHIN_M)
        cmd = tr.update(truth.copy(), v, DT)
        if cmd.status in ("arrived", "failed"):
            out["status"] = cmd.status + ("" if cmd.failure is None else ":" + cmd.failure)
            break
        if cmd.at_cusp and cmd.speed_mps == 0.0 and abs(v) < cfg.stop_speed_mps and tr.release_cusp(True):
            cmd = tr.update(truth.copy(), v, DT)
        speed_cap = max(abs(cmd.speed_mps), abs(prev), abs(v))
        if abs(v) > 0.01:
            direction = 1 if v > 0 else -1
        elif abs(prev) > 1e-6:
            direction = 1 if prev > 0 else -1
        elif cmd.speed_mps != 0.0:
            direction = 1 if cmd.speed_mps > 0 else -1
        else:
            direction = int(tr.leg_ahead()[1]) or 1
        g = guard_cap(tuple(float(x) for x in truth), float(cmd.curvature_inv_m), direction, speed_cap, shape, lc,
                      progress_m=tr.progress_m(), leg_remaining_m=tr.leg_remaining_m(),
                      end_allowance_m=float(end_allow), stopping=STOPPING, config=lcfg)
        target = np.asarray(ackermann_command(cmd.speed_mps, cmd.curvature_inv_m, geom).steering_rad)
        ws = creep(cmd.speed_mps, float(np.max(np.abs(target - act))))
        ws = slew_cap_stop(prev, ws, decel_mps2=STOPPING.decel_mps2, accel_mps2=0.8, dt=DT)
        if math.isfinite(g.allowed_mps):
            if abs(ws) > g.allowed_mps:
                ws = math.copysign(g.allowed_mps, ws)
            if g.allowed_mps < speed_cap:
                out["capped_ticks"] += 1
        if g.allowed_mps == 0.0:
            ws = 0.0
            if out["zero"] is None:
                out["zero"] = {"t_s": k * DT, "progress_m": tr.progress_m(), "s_from_m": g.s_star_m,
                               "lower_m": g.lower_bound_m, "threshold_m": g.threshold_m, "commanded_mps": cmd.speed_mps}
        prev = ws
        sc += np.clip(target - sc, -DT, DT)
        act = a * act + (1 - a) * sc
        v = ackermann_command(ws, cmd.curvature_inv_m, geom).speed_mps * SG
        kappa = (math.tan(act[0]) + math.tan(act[1])) / (2 * L) * (GF if v >= 0 else GR)
        change = v * kappa * DT
        heading = truth[2] + change / 2
        truth += [v * DT * math.cos(heading), v * DT * math.sin(heading), change]
        out["min_real_m"] = min(out["min_real_m"], shape_clearance(truth, shape, real))
        if out["zero"] is not None and abs(v) < 1e-4:
            out["status"] = "guard_stop"
            break
        if out["zero"] is None and out["armed_s"] is not None and tr.progress_m() - out["armed_s"] > HOLD_M:
            out["status"] = "hold_window_passed"
            break
    out["stop_pose"] = [float(x) for x in truth]
    if out["status"] == "guard_stop":
        out["reverse_real_m"] = reverse_clear(truth, shape, real)
        out["reverse_end_inflated_m"] = shape_clearance(
            (truth[0] - REVERSE_M * math.cos(truth[2]), truth[1] - REVERSE_M * math.sin(truth[2]), truth[2]),
            shape, search + slot)
    out["established"] = established(out)
    return out


def established(out: dict) -> bool:
    """The plan's 1.3: a zero cap (first violation within the stopping margin), stopped before contact with any real
    outline, a straight reverse of 0.30 m clear of them, no tracking failure and no arrival first."""
    z = out.get("zero")
    return (out.get("status") == "guard_stop" and z is not None and z["s_from_m"] <= STOPPING.margin_m
            and out["min_real_m"] > 0.0 and out.get("reverse_real_m", -1.0) > 0.0)


def legs(sessions: Path):
    for rec in sorted(sessions.glob("ab_*_B/live/epoch_1/run/missions/m*/records.json")):
        run = rec.parents[2]
        result = json.loads((run / "result.json").read_text())
        state = json.loads(rec.read_text())["state"]
        for phase in ("withdraw", "return_home"):
            leg = (state.get("paths") or {}).get(phase)
            if not leg:
                continue
            if phase == "return_home" and float(leg.get("length_m", 0.0)) < RETURN_MIN_M:
                continue  # overwritten by a later replan (the plan's 0)
            yield (f"{run.parents[2].name}/{rec.parent.name}", phase, leg, state["mission_scenario"]["props"],
                   result["tracker_configs"][phase], result["arguments"]["drive_geometry"])


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    sessions, out_path = Path(argv[0]), Path(argv[1])
    shape = unloaded_shape()
    rows = []
    for name, phase, leg, props, tcfg, dgeo in legs(sessions):
        for g_phase, kind, s_a, size, r in GRID:
            if g_phase != phase:
                continue
            res = roll(leg, props, tcfg, dgeo, phase, kind, s_a, size, r, shape)
            rows.append({"leg": name, "phase": phase, "kind": kind, "s_a": s_a, "size": size, "r": r, **res})
            print(name, phase, kind, s_a, size, r, res["status"], res["established"], flush=True)
    points = {}
    for row in rows:
        key = (row["phase"], row["kind"], row["s_a"], row["size"], row["r"])
        points.setdefault(key, []).append(row["established"])
    summary = sorted(([*k, sum(v), len(v)] for k, v in points.items()), key=lambda x: -x[5])
    out_path.write_text(json.dumps({"grid": [list(g) for g in GRID], "rows": rows, "summary": summary}, indent=1,
                                   default=str))
    print("established points:", [s for s in summary if s[5] > 0] or "none")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
