"""Leg-failures plan V2 (docs/plans/2026-10-25-leg-failures.md): the recorded D7 re-approaches driven again on CPU
by the real transport tracker through a steering-dynamics plant -- the software steering rate limit, a joint lag and
the measured curvature response -- to check whether the S-curve's curvature reversal leaves the heading error the
Isaac runs stalled with, and, if it does, to fix B1/B2's values before Isaac.

The plant, each 1/120 s tick, as the runner's command chain (core.assemble_command):
- the tracker's command (speed, curvature) -> the Ackermann target steering;
- the wheel speed creeps (a quarter) while the joint is more than 0.05 rad from the target, then slews (pulling away
  at COMMAND_ACCEL_MPS2, braking at the permission's stopping deceleration);
- the steering command moves toward the target at steering_command_rate_rad_s (1.00 rad/s, a software limit);
- the joint follows the command DELAY_S late (0.12 s, an estimate biased low with a 0.1 s resolution);
- the realised curvature is the joint's Ackermann curvature times the gear's response (forward 0.877, reverse 1.0:
  the Isaac plant's steady-state ratio, which already holds the joint's tracking loss).
A stall is what raises trigger d in the runner: standing with the tracker at a cusp (or the goal) still tracking for
120 ticks, or the tracker failing at a cusp.

    python tools/d7_reapproach_steer.py extract RUN_DIR... --out cases.json
    python tools/d7_reapproach_steer.py check cases.json --out v2.json
"""

from __future__ import annotations

import argparse
import dataclasses
import glob
import json
import math
import os
import sys
from collections import deque
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
for p_ in (ROOT / "src", ROOT / "tools", ROOT / "sim/isaac", ROOT):
    if str(p_) not in sys.path:
        sys.path.insert(0, str(p_))

from forklift_core.control.path_tracking import RearAxlePathTracker, ackermann_command  # noqa: E402
from forklift_core.control.rollout import CurvatureResponse  # noqa: E402
from forklift_core.control.speed_chain import creep, slew_cap_stop  # noqa: E402

DT_S = 1.0 / 120.0
DELAY_S = 0.12
RESPONSE = CurvatureResponse(forward=0.877, reverse=1.0)
COMMAND_ACCEL_MPS2 = 0.8  # sim/isaac/run_transport.py
STAND_TICKS = 120  # the runner's stall count
GRID_KAPPA = (0.29, 0.26, 0.24, 0.22)
GRID_CRUISE = (0.30, 0.20, 0.15)
FIRST_CANDIDATE = (0.24, 0.15)
MODEL_HEADING_RAD = 0.03  # the reproduction tolerance, kept as the selection's margin
PROGRESS_FRACTION = 0.10
# The two failures the plan names (section 0 B): every counted re-approach of these missions must be a stall case,
# complete (path, stop pose, config, scene) -- none may be missing for V2 to stand (Codex plan round 6 P2).
REQUIRED_STALL_MISSIONS = (("ct2_41a2a7e8_fixed_random_m_s2", "m1"), ("ct2_41a2a7e8_auto_random_m_s2", "m2"))
REQUIRED_STALLS = 4


def plant_constants(settings_path: Path, layer_path: Path) -> dict:
    settings = yaml.safe_load(settings_path.read_text())
    layer = yaml.safe_load(layer_path.read_text())
    return {"rate_rad_s": float(settings["steering_command_rate_rad_s"]),
            "max_wheel_rate_rad_s": float(settings["max_wheel_rate_rad_s"]),
            "decel_mps2": float(layer["permission"]["decel_mps2"])}


def joint_curvature(steering, geometry) -> float:
    """The curvature the two steered wheels' angles stand for (the inverse of ackermann_command), averaged."""
    half = geometry.track_m / 2
    tl, tr = math.tan(float(steering[0])), math.tan(float(steering[1]))
    return 0.5 * (tl / (geometry.wheelbase_m + half * tl) + tr / (geometry.wheelbase_m - half * tr))


def leg_bounds(tracker, k: int) -> tuple[int, int]:
    ends = list(tracker._leg_ends)
    return (0 if k == 0 else int(ends[k - 1])), int(ends[k])


def leg_offsets(poses, start_i: int, end_i: int, pose) -> tuple[float, float]:
    """(progress fraction along the leg of the nearest point, heading minus the leg end's yaw)."""
    seg = np.asarray(poses[start_i:end_i + 1], float)
    d = np.hypot(seg[:, 0] - pose[0], seg[:, 1] - pose[1])
    i = int(np.argmin(d))
    arc = np.concatenate(([0.0], np.cumsum(np.hypot(np.diff(seg[:, 0]), np.diff(seg[:, 1])))))
    frac = float(arc[i] / arc[-1]) if arc[-1] > 0 else 1.0
    end_yaw = float(seg[-1, 2])
    return frac, math.atan2(math.sin(pose[2] - end_yaw), math.cos(pose[2] - end_yaw))


def simulate(poses, directions, curvatures, config, start, geometry, constants, *, steering0=(0.0, 0.0),
             command0=None, delay_s=DELAY_S, response=RESPONSE, max_time_s=90.0) -> dict:
    poses = np.asarray(poses, float)
    tracker = RearAxlePathTracker(poses, np.asarray(directions, np.int8), np.asarray(curvatures, float), config)
    pose = np.asarray(start, float).copy()
    speed = 0.0
    command_steer = np.asarray(command0 if command0 is not None else steering0, float).copy()
    lag = max(1, int(round(delay_s / DT_S)))
    history = deque([np.asarray(steering0, float).copy()] * lag, maxlen=lag)
    rate = constants["rate_rad_s"] * DT_S
    stand = 0
    saturated = moving = 0
    leg_end_headings = []  # the heading against each leg end's yaw, as the tracker leaves it (and at arrival)
    out = {"status": "timeout"}
    for k in range(int(max_time_s / DT_S)):
        joint = history[0]
        leg_before = tracker.leg_index
        command = tracker.update(pose, speed, DT_S)
        if tracker.leg_index != leg_before:
            end_yaw = float(poses[leg_bounds(tracker, leg_before)[1], 2])
            leg_end_headings.append(math.atan2(math.sin(pose[2] - end_yaw), math.cos(pose[2] - end_yaw)))
        if command.status == "arrived":
            leg_end_headings.append(float(command.yaw_error_rad))
            out = {"status": "arrived"}
            break
        if command.status == "failed":
            out = {"status": "stall", "why": f"failed:{command.failure}"}
            break
        target = np.asarray(ackermann_command(command.speed_mps, command.curvature_inv_m, geometry).steering_rad)
        wheel = creep(command.speed_mps, float(np.max(np.abs(target - joint))))
        wheel = slew_cap_stop(speed, wheel, decel_mps2=constants["decel_mps2"], accel_mps2=COMMAND_ACCEL_MPS2,
                              dt=DT_S)
        wheel = ackermann_command(wheel, command.curvature_inv_m, geometry).speed_mps
        command_steer = command_steer + np.clip(target - command_steer, -rate, rate)
        history.append(command_steer.copy())
        kappa = joint_curvature(joint, geometry) * (1.0 if response is None else response.factor(wheel))
        change = wheel * kappa * DT_S
        heading = pose[2] + change / 2
        pose[:2] += wheel * DT_S * np.array([math.cos(heading), math.sin(heading)])
        pose[2] += change
        speed = wheel
        if abs(wheel) > 1e-6:
            moving += 1
            saturated += abs(command.curvature_inv_m) >= config.max_curvature_inv_m - 1e-9
        # the runner's trigger d: standing, still tracking, at a cusp or with nothing left (core stall rule)
        standing = (wheel == 0.0 and command.status == "tracking"
                    and (command.at_cusp or tracker.remaining_to_goal_m() <= 1e-6))
        stand = stand + 1 if standing else 0
        if stand >= STAND_TICKS:
            out = {"status": "stall", "why": "standing_at_cusp" if command.at_cusp else "standing_at_goal"}
            break
    leg = int(tracker.leg_index)
    a, b = leg_bounds(tracker, leg)
    frac, heading_err = leg_offsets(poses, a, b, pose)
    out.update(leg=leg, legs=len(tracker._leg_ends), progress_fraction=frac, heading_error_rad=heading_err,
               pose=[float(v) for v in pose], time_s=k * DT_S, saturated_fraction=saturated / max(moving, 1),
               leg_end_heading_rad=leg_end_headings)
    return out


# --- the recorded cases ---------------------------------------------------------------------------------------------

def _json(path):
    with open(path) as handle:
        return json.load(handle)


def _jsonl(pattern):
    rows = []
    for f in sorted(glob.glob(pattern)):
        with open(f) as handle:
            rows.extend(json.loads(line) for line in handle if line.strip())
    return rows


def extract(run_dirs) -> list:
    """Every counted docking retry's re-approach in the runs: its path, start, the steering at its start, and what the
    Isaac run did with it. Stalled: the next counted retry is d (or the mission ended on
    transport_recovery_after_docking) with no docking round in between, stood at the control pose just before. Arrived:
    a docking round (the retry match, made only once the re-approach stands at the line start) inside the attempt's
    window. Anything else -- a mission that stopped on the way, a lost link -- is excluded, not a control (Codex plan
    round 6 P2)."""
    cases = []
    for run in run_dirs:
        name = run.rstrip("/").split("/")[-4] if run.rstrip("/").endswith("epoch_1/run") else os.path.basename(run)
        try:
            configs = _json(f"{run}/result.json").get("tracker_configs") or {}
        except (OSError, ValueError):
            configs = {}
        control = samples = None
        for md in sorted(glob.glob(f"{run}/missions/m*")):
            try:
                st = _json(f"{md}/records.json")["state"]
                res = _json(f"{md}/result.json").get("result") or {}
            except (OSError, ValueError, KeyError):
                continue
            retries = st.get("docking_retries") or []
            rounds = [float(r_["time_s"]) for r_ in (st.get("delivery_docking") or {}).get("rounds") or []
                      if r_.get("time_s") is not None]
            replaced = sorted((x for x in st.get("paths_replaced") or [] if x.get("leg") == "transport"),
                              key=lambda x: x["time_s"])
            for k, ev in enumerate(retries):
                if not ev.get("counted") or not ev.get("plan") or ev.get("start") is None:
                    continue
                if (ev.get("plan") or {}).get("length_m") is None or (ev.get("plan") or {}).get("refused"):
                    continue
                later = [e for e in retries[k + 1:]]
                nxt = next((e for e in later if e.get("counted")), None)
                between = [e for e in later if not e.get("counted") and (nxt is None or e["time_s"] < nxt["time_s"])]
                after = [x for x in replaced if x["time_s"] > ev["time_s"] + 1e-9]
                path = after[0]["path"] if after else (st.get("paths") or {}).get("transport")
                by = after[0]["by"] if after else None
                row = {"run": name, "mission": os.path.basename(md), "retry": k, "trigger": ev.get("trigger"),
                       "time_s": ev["time_s"], "start": ev["start"], "line_start": ev.get("line_start"),
                       "plan_length_m": ev["plan"]["length_m"], "next": None if nxt is None else nxt.get("trigger"),
                       "mission_outcome": res.get("outcome"), "mission_reason": res.get("reason"),
                       "config": configs.get("transport"), "scenario": st.get("mission_scenario")}
                window_end = nxt["time_s"] if nxt is not None else res.get("ended_s")
                matched = [t_ for t_ in rounds if t_ > ev["time_s"] + 1e-9
                           and (window_end is None or t_ <= float(window_end) + 1e-9)]
                row["rounds_in_window"] = len(matched)
                if between:
                    row["excluded"] = "replanned_on_the_way"
                elif path is None or abs(float(path["length_m"]) - float(ev["plan"]["length_m"])) > 1e-6:
                    row["excluded"] = f"path_not_found:{by}"
                elif matched:
                    row["isaac"] = "arrived"
                elif nxt is not None and nxt.get("trigger") == "d":
                    row["isaac"], row["end_s"] = "stall", nxt["time_s"]
                elif nxt is None and res.get("reason") == "transport_recovery_after_docking":
                    row["isaac"], row["end_s"] = "stall", res.get("ended_s")
                else:
                    nxt_ = None if nxt is None else nxt.get("trigger")
                    row["excluded"] = f"no_arrival_evidence:{nxt_}:{res.get('reason')}"
                if "excluded" not in row:
                    row["path"] = {k_: path[k_] for k_ in ("poses", "directions", "curvatures_inv_m")}
                    if control is None:
                        control = _jsonl(f"{run}/spool/slam.control/*.jsonl")
                        samples = _jsonl(f"{run}/spool/state.samples/*.jsonl")
                    # the last sample at or before the retry (never a later one); which one, and any zero fallback
                    before = [q for q in samples or [] if q["time_s"] <= ev["time_s"] + 1e-9]
                    s0 = before[-1] if before else {}
                    row["steering0_sample_s"] = s0.get("time_s")
                    row["steering0_lag_s"] = None if not s0 else float(ev["time_s"]) - float(s0["time_s"])
                    row["steering0_fallback"] = not s0.get("steering_actual_rad")
                    row["steering0"] = s0.get("steering_actual_rad") or [0.0, 0.0]
                    row["command0"] = s0.get("steering_command_rad") or row["steering0"]
                    if row.get("end_s") is not None and control:
                        stand = [c for c in control if c[0] < float(row["end_s"]) - 1e-6]
                        row["stop_pose"] = stand[-1][1:4] if stand else None
                cases.append(row)
    return cases


def isaac_offsets(case) -> dict | None:
    """The Isaac stop's leg (the nearest leg end), progress fraction and heading error at it."""
    if case.get("stop_pose") is None:
        return None
    poses = np.asarray(case["path"]["poses"], float)
    d = np.asarray(case["path"]["directions"], int)
    ends = [i for i in range(len(d) - 1) if d[i + 1] != d[i]] + [len(d) - 1]
    pose = np.asarray(case["stop_pose"], float)
    leg = int(np.argmin([math.hypot(poses[i, 0] - pose[0], poses[i, 1] - pose[1]) for i in ends]))
    a = 0 if leg == 0 else ends[leg - 1]
    frac, heading = leg_offsets(poses, a, ends[leg], pose)
    return {"leg": leg, "legs": len(ends), "progress_fraction": frac, "heading_error_rad": heading}


def check(cases, geometry, constants, *, planner=None) -> dict:
    """The pre-registered reproduction and, if it holds, the grid's pick (plan V2)."""
    from curvature_audit import tracker_config  # noqa: E402

    rows = []
    incomplete = []
    for c in cases:
        if "excluded" in c:
            continue
        if c.get("config") is None or not c.get("scenario"):
            incomplete.append([c["run"], c["mission"], c["retry"]])
            continue
        cfg = dataclasses.replace(tracker_config(c["config"]), position_tolerance_m=0.03, yaw_tolerance_rad=0.05)
        sim = simulate(c["path"]["poses"], c["path"]["directions"], c["path"]["curvatures_inv_m"], cfg, c["start"],
                       geometry, constants, steering0=c["steering0"], command0=c["command0"])
        row = {k: c[k] for k in ("run", "mission", "retry", "trigger", "isaac")}
        row["sim"] = sim
        if c["isaac"] == "stall":
            iso = isaac_offsets(c)
            row["isaac_stop"] = iso
            tol = cfg.cusp_yaw_tolerance_rad if iso and iso["leg"] < iso["legs"] - 1 else cfg.yaw_tolerance_rad
            row["criteria"] = None if iso is None else {
                "stalls": sim["status"] == "stall",
                "beyond_tolerance": abs(sim["heading_error_rad"]) > tol,
                "sign_and_size": (math.copysign(1, sim["heading_error_rad"]) == math.copysign(1, iso["heading_error_rad"])
                                  and abs(sim["heading_error_rad"] - iso["heading_error_rad"]) <= MODEL_HEADING_RAD),
                "same_leg": sim["leg"] == iso["leg"],
                "progress": abs(sim["progress_fraction"] - iso["progress_fraction"]) <= PROGRESS_FRACTION,
            }
        else:
            row["criteria"] = {"arrives": sim["status"] == "arrived"}
        rows.append(row)
    stalls = [r for r in rows if r["isaac"] == "stall"]
    controls = [r for r in rows if r["isaac"] == "arrived"]
    required = [r for r in stalls if (r["run"], r["mission"]) in REQUIRED_STALL_MISSIONS]
    # every counted re-approach of the named missions, in or out of the rows (an excluded one is a missing one)
    named = [c for c in cases if (c["run"], c["mission"]) in REQUIRED_STALL_MISSIONS]
    complete = (len(named) == REQUIRED_STALLS and len(required) == REQUIRED_STALLS
                and all(r["criteria"] is not None for r in required) and not incomplete)
    # the pre-registered criteria are the named four's; the other stalls (a b re-approach that stood, then a d one
    # that arrived) are reported, not judged
    others = [r for r in stalls if (r["run"], r["mission"]) not in REQUIRED_STALL_MISSIONS]
    reproduced = (complete and bool(controls)
                  and all(all(r["criteria"].values()) for r in required)
                  and all(r["criteria"]["arrives"] for r in controls))
    out = {"rows": rows, "stall_cases": len(stalls), "control_cases": len(controls), "required_stalls": len(required),
           "named_attempts": len(named), "incomplete": incomplete, "complete": complete, "reproduced": reproduced,
           "other_stalls": len(others),
           "other_stalls_reproduced": sum(1 for r in others if r["criteria"] and all(r["criteria"].values())),
           "controls_arrived": sum(1 for r in controls if r["criteria"]["arrives"])}
    if reproduced and planner is not None:
        out["grid"] = grid(cases, geometry, constants, planner)
        # the grid runs on exactly the checked cases, every one planned or counted as no plan (never an empty set)
        expected = sorted([r["run"], r["mission"], r["retry"]] for r in rows)
        for g in out["grid"]:
            g["same_cases"] = bool(expected) and sorted(x["case"] for x in g["results"]) == expected
        ok = [g for g in out["grid"] if g["all_arrive_with_margin"] and g["same_cases"]]
        best = max(ok, key=lambda g: (g["kappa"], g["cruise"])) if ok else None
        out["pick"] = (best["kappa"], best["cruise"]) if best else FIRST_CANDIDATE
        out["pick_rule"] = "largest kappa, then highest cruise, with every case arriving inside tolerance - 0.03 rad"
    else:
        out["pick"] = FIRST_CANDIDATE
        out["pick_rule"] = "not reproduced: the first candidate"
    return out


def grid(cases, geometry, constants, planner) -> list:
    """Every grid point: each usable case re-planned from its recorded start at kappa and driven at the cruise."""
    from curvature_audit import tracker_config  # noqa: E402

    usable = [c for c in cases if "excluded" not in c and c.get("config") is not None and c.get("scenario")]
    points = []
    for kappa in GRID_KAPPA:
        plans = [planner(c, kappa) for c in usable]
        for cruise in GRID_CRUISE:
            results = []
            for c, path in zip(usable, plans):
                if path is None:
                    results.append({"case": [c["run"], c["mission"], c["retry"]], "status": "no_plan"})
                    continue
                cfg = dataclasses.replace(tracker_config(c["config"]), position_tolerance_m=0.03,
                                          yaw_tolerance_rad=0.05)
                cfg = dataclasses.replace(cfg, cruise_speed_mps=min(cfg.cruise_speed_mps, cruise))
                sim = simulate(path.poses, path.directions, path.curvatures_inv_m, cfg, c["start"], geometry,
                               constants, steering0=c["steering0"], command0=c["command0"])
                results.append({"case": [c["run"], c["mission"], c["retry"]], "status": sim["status"],
                                "leg_end_heading_rad": sim["leg_end_heading_rad"], "time_s": sim["time_s"],
                                "saturated_fraction": sim["saturated_fraction"]})
            # every leg end (cusps 0.05, the stop 0.05 rad) inside its tolerance less the model's own accuracy
            margin = 0.05 - MODEL_HEADING_RAD
            points.append({"kappa": kappa, "cruise": cruise, "cases": len(results),
                           "arrived": sum(r["status"] == "arrived" for r in results),
                           "all_arrive_with_margin": all(r["status"] == "arrived" for r in results)
                           and all(abs(h) <= margin for r in results for h in r["leg_end_heading_rad"]),
                           "results": results})
    return points


def make_planner():
    from bay_planning_sweep import mission_geometry  # noqa: E402
    from riibotics_scenario_paths import PALLETS  # noqa: E402
    from scene_file import scenario_from_record  # noqa: E402
    from session_plan_check import RESERVE_M, STRAIGHTS, URDF  # noqa: E402

    from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
    from forklift_core.planning import Pose2D  # noqa: E402
    from forklift_core.planning.pallet_mission import (  # noqa: E402
        D7_PLANNER_OPTIONS, SearchBudget, make_transport_planner_config, plan_docking_reapproach)

    mission = mission_geometry(URDF, load_pallet_geometry(PALLETS["epal6"]), RESERVE_M, **STRAIGHTS)

    def plan(case, kappa):
        config = make_transport_planner_config(curvature_limit_inv_m=kappa, clearance_m=0.10, max_expansions=30000,
                                               **D7_PLANNER_OPTIONS)
        path, _ = plan_docking_reapproach(scenario_from_record(case["scenario"]), Pose2D(*case["start"]),
                                          Pose2D(*case["line_start"]), config, geometry=mission,
                                          keep_m=mission.delivery_straight_m, deadline=SearchBudget(20.0))
        return path

    return plan, URDF


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    ex = sub.add_parser("extract")
    ex.add_argument("runs", nargs="+")
    ex.add_argument("--out", type=Path, required=True)
    ck = sub.add_parser("check")
    ck.add_argument("cases", type=Path)
    ck.add_argument("--out", type=Path, required=True)
    ck.add_argument("--settings", type=Path, default=ROOT / "config/isaac_transport_measured.yaml")
    ck.add_argument("--layer", type=Path, default=ROOT / "config/obstacle_layer_single.yaml")
    args = ap.parse_args(argv)
    if args.cmd == "extract":
        cases = extract(args.runs)
        args.out.write_text(json.dumps(cases, default=float))
        print(json.dumps({"cases": len(cases), "stall": sum(c.get("isaac") == "stall" for c in cases),
                          "arrived": sum(c.get("isaac") == "arrived" for c in cases),
                          "excluded": sum("excluded" in c for c in cases)}))
        return 0
    from insertion_geometry import read_drive_geometry_m  # noqa: E402

    constants = plant_constants(args.settings, args.layer)
    planner, urdf = make_planner()
    geometry = read_drive_geometry_m(urdf, constants["max_wheel_rate_rad_s"])
    result = check(json.loads(args.cases.read_text()), geometry, constants, planner=planner)
    result["constants"] = {**constants, "delay_s": DELAY_S, "response": dataclasses.asdict(RESPONSE)}
    args.out.write_text(json.dumps(result, indent=1, default=float))
    print(json.dumps({k: result[k] for k in ("stall_cases", "control_cases", "reproduced", "pick", "pick_rule")}))
    for r in result["rows"]:
        print(r["run"], r["mission"], r["retry"], r["trigger"], r["isaac"], r["sim"]["status"],
              round(r["sim"]["heading_error_rad"], 3), r["sim"]["leg"], round(r["sim"]["progress_fraction"], 2),
              r.get("isaac_stop"), r["criteria"])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
