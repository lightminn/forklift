"""Plan D8 S4a-1 ⓪: the drive terms of the near-field travel budget (no Isaac).

docs/plans/2026-10-04-lidar-obstacle-map.md, "D8 S4 실행기 연결 설계". From runs recorded
with --record-pocket-frames --measure-safety-stops-m (truth and the applied command of
every physics tick, the control rear axle of every loop tick), over each run's
[near-capture anchor, insertion end]:

  delta_len(w)  the largest (truth path length - control path length) over any window
                of length <= w (the odometry's endpoint error e does not bound path
                lengths -- Codex S4 2nd review);
  eps_c(w)      the largest (control path length - integral of |applied command|) over
                any window of length <= w: the control path the budget's d adds up runs
                ahead of the command (odometry reading high, the truck faster than its
                command) -- the S4a-1 smoke stopped on the budget at 0.065 m/s without it;
  Delta_s(w)    the largest integral of (truth path speed - |applied command|) over any
                window of length <= w, cruising, braking and restarting alike;
  delta_v       the largest truth path speed above the highest |applied command| whose
                interval ends within the preceding 0.3 s (36 intervals, the current one
                included; never negative);

and the near-field cruise: the largest 0.005 m/s step v, up to the fastest measured
straight speed (the terms merge every run, the faster ones included), whose no-loss
steady state keeps d + travel <= 0.03 m at every tick of a render cycle. u after a
result arrives, its pixels are age + u old (age = L_hi + A) and the next result is
period - u away, so

  d(u)      = v (age + u) + eps_c(age + u) + delta_len(age + u)
  travel(u) = vbar h + Delta_s(h) + vbar^2 / (2 decel),  h = period - u + latency,
              vbar = v + delta_v.

Each control interval [t_i, t_i+1] is paired with the truth record stamped t_i+1: the
runner sets the command at loop time t_i and the recorder writes it after the physics
step (Codex S4a-1 ⓪ review P2). A cruise is reported as confirmed only from the plan's
measurement matrix: seeds 1, 3, 5 x 0.055 and 0.08 m/s, D5 off, the insertion finished,
the four scheduled stops made at their gaps (command zero from the trigger to the
release, standing 0.5 s, a restart after), the bounds file's camera, mount and assets,
the arguments and settings of the D8b calibration run of the same seed and speed, and no
source differing from the bounds measurement except the files the measurement manifest
names with the exact hashes reviewed (Codex S4a-1 ⓪ 2nd review); anything else is a
diagnostic.

    python tools/s4_drive_terms.py --bounds config/near_field_bounds_measured.json \\
        --manifest <dir>/manifest.json --runs <run dir> ... --output <dir>/s4_drive_terms.json

The manifest is {"snapshot": name, "allowed_sources": {path: sha256}}.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

TICK_S = 1.0 / 120.0
RECENT_S = 0.3  # the command maximum the speed bound looks back over
STOP_LATENCY_S = 0.15 + TICK_S  # the drive's reaction plus the tick that notices
DECEL_MPS2 = 1.5
BUDGET_M = 0.03
STEP_MPS = 0.005

# The measurement matrix (plan D8 S4a-1 ⓪).
REQUIRED_SEEDS = (1, 3, 5)
REQUIRED_SPEEDS = (0.055, 0.08)
REQUIRED_GAPS_M = (2.0, 1.5, 1.0, 0.7)
# Fields that must equal the bounds file's condition (the D8b camera, mount with its
# 1 mm quantization, cadence and assets).
CONDITION_KEYS = (
    "video",
    "fps",
    "mount",
    "camera",
    "pallet_geometry",
    "forklift_urdf",
    "rear_axle_offset_m",
)
ASSET_HASH_KEYS = (
    "pallet_geometry_sha256",
    "forklift_urdf_sha256",
    "pallet_urdf_sha256",
)
# Fields equal to the D8b calibration run of the same seed and speed (the odometry
# noise, drive, followers, planners and detector), and the arguments allowed to differ.
REFERENCE_FIELDS = (
    "settings_synthetic",
    "detector_params",
    "tracker_configs",
    "planner_config",
    "travel_planner_config",
    "lidar_synthetic",
    "scene_sha256",
    "chassis_model",
    "geometry",
    "scenario",
)
ARGUMENTS_BY_DESIGN = (
    "output",
    "slam_feedback",
    "slam_map_dir",
    "measure_safety_stops_m",
    "record_dwell_gaps",
)
HOLD_S = 0.5


def running_window_max(values: np.ndarray, max_k: int) -> np.ndarray:
    """out[k] = the largest sum of ``values`` over any window of k consecutive entries,
    taken as a running maximum over k (windows up to k). values[i] is the increment of
    step i (between samples i and i + 1)."""
    csum = np.concatenate(([0.0], np.cumsum(values)))
    out = np.zeros(max_k + 1)
    for k in range(1, max_k + 1):
        if k > len(values):
            out[k] = out[k - 1]
            continue
        out[k] = max(out[k - 1], float(np.max(csum[k:] - csum[:-k])))
    return out


def path_steps(xy: np.ndarray) -> np.ndarray:
    return np.hypot(*np.diff(np.asarray(xy, dtype=float), axis=0).T)


def drive_terms(
    control: np.ndarray,
    truth_t: np.ndarray,
    applied: np.ndarray,
    start_s: float,
    end_s: float,
    max_window_s: float = 1.0,
) -> dict:
    """The three terms of one run. control: rows (t, control x y yaw, truth x y yaw) of the
    rear axle per loop tick; truth_t / applied: the applied speed command per physics
    tick, stamped at the end of the step it drove."""
    rows = control[(control[:, 0] >= start_s - 1e-9) & (control[:, 0] <= end_s + 1e-9)]
    if len(rows) < 3:
        raise ValueError("fewer than three control rows in the span")
    dt = np.diff(rows[:, 0])
    if np.any(np.abs(dt - TICK_S) > 0.05 * TICK_S):
        raise ValueError("the control record is not one row per tick across the span")
    k_max = int(round(max_window_s / TICK_S))
    true_steps = path_steps(rows[:, 4:6])
    ctrl_steps = path_steps(rows[:, 1:3])
    delta_len = running_window_max(true_steps - ctrl_steps, k_max)
    # Path speed, unsigned, against the magnitude of the command that drove the interval
    # (Codex S4a-1 ⓪ review P2: a signed projection let delta_v go negative).
    v_true = true_steps / dt
    order = np.argsort(truth_t)
    t_sorted, a_sorted = truth_t[order], applied[order]
    ends = rows[1:, 0]
    idx = np.clip(np.searchsorted(t_sorted, ends - 0.5 * TICK_S), 0, len(t_sorted) - 1)
    if np.any(np.abs(t_sorted[idx] - ends) > 0.25 * TICK_S) or not np.all(
        np.isfinite(a_sorted[idx])
    ):
        raise ValueError(
            "a control interval has no applied command in the truth record"
        )
    v_cmd = np.abs(a_sorted[idx])
    delta_s = running_window_max((v_true - v_cmd) * dt, k_max)
    control_excess = running_window_max(ctrl_steps - v_cmd * dt, k_max)
    recent = int(round(RECENT_S / TICK_S))
    # Interval i ends at ends[i]: the commands of the intervals ending within the
    # preceding 0.3 s are i - 35 .. i (Codex S4a-1 ⓪ 2nd review P2).
    highest = np.array(
        [v_cmd[max(0, i - recent + 1) : i + 1].max() for i in range(len(v_cmd))]
    )
    over = v_true - highest
    worst = int(np.argmax(over))
    return {
        "ticks": len(rows),
        "delta_len_m": delta_len.tolist(),
        "delta_s_m": delta_s.tolist(),
        "control_excess_m": control_excess.tolist(),
        "delta_v_mps": max(0.0, float(over[worst])),
        "delta_v_at_s": float(ends[worst]),
        "speed_true_max_mps": float(v_true.max()),
        "command_max_mps": float(v_cmd.max()),
    }


def term_at(table: list, window_s: float) -> float:
    """The running maximum at the first stored window at or above window_s."""
    k = int(math.ceil(window_s / TICK_S - 1e-9))
    if k >= len(table):
        raise ValueError(
            f"window {window_s:.3f} s is past the measured {(len(table) - 1) * TICK_S:.3f} s"
        )
    return float(table[max(k, 0)])


def budget_cycle(
    v: float,
    delta_len: list,
    control_excess: list,
    delta_s: list,
    delta_v: float,
    align_age_s: float,
    period_s: float,
) -> tuple[float, float]:
    """(the largest d + travel over one render cycle, the u it occurs at) -- see the
    module text. u runs over every tick from a result's arrival to the next one."""
    vbar = v + delta_v
    worst, worst_u = -math.inf, 0.0
    for k in range(int(round(period_s / TICK_S)) + 1):
        u = k * TICK_S
        horizon = max(period_s - u, 0.0) + STOP_LATENCY_S
        total = (
            v * (align_age_s + u)
            + term_at(control_excess, align_age_s + u)
            + term_at(delta_len, align_age_s + u)
            + vbar * horizon
            + term_at(delta_s, horizon)
            + vbar * vbar / (2 * DECEL_MPS2)
        )
        if total > worst:
            worst, worst_u = total, u
    return worst, worst_u


def cruise(
    delta_len: list,
    control_excess: list,
    delta_s: list,
    delta_v: float,
    align_age_s: float,
    period_s: float,
    v_max: float,
) -> float:
    """The largest STEP_MPS multiple up to v_max whose cycle keeps the budget (0 if none)."""
    best = 0.0
    v = STEP_MPS
    while v <= v_max + 1e-12:
        if (
            budget_cycle(
                v, delta_len, control_excess, delta_s, delta_v, align_age_s, period_s
            )[0]
            <= BUDGET_M + 1e-12
        ):
            best = round(v, 6)
        v += STEP_MPS
    return best


def merge(tables: list[list]) -> list:
    n = max(len(t) for t in tables)
    out = np.zeros(n)
    for t in tables:
        out[: len(t)] = np.maximum(out[: len(t)], t)
    return np.maximum.accumulate(out).tolist()


from sim.isaac.near_tracking import (  # noqa: E402,F401 -- shared with the runner's contract
    condition_problems,
    config_paths,
    file_sha256,
    input_files_skipping,
    run_condition,
    snapshot_relative,
    snapshot_root,
)


def input_files(arguments: dict, root: Path | None) -> dict[str, str | None]:
    """The input files of a run against its D8b reference (by-design arguments skipped)."""
    return input_files_skipping(arguments, root, ARGUMENTS_BY_DESIGN)


def reference_problems(key: str, result: dict, reference: dict) -> list[str]:
    """Differences from the D8b calibration run of the same seed and speed: the recorded
    settings, and every file an argument names by content (a YAML changed under the same
    path -- Codex S4a-1 ⓪ 3rd review P2)."""
    problems = []
    for name in REFERENCE_FIELDS:
        if result.get(name) != reference.get(name):
            problems.append(f"{key}: {name} differs from the D8b run")
    factory, factory_ref = (
        {k: v for k, v in (r.get("factory") or {}).items() if k != "layout"}
        for r in (result, reference)
    )
    if factory != factory_ref:
        problems.append(f"{key}: factory differs from the D8b run")
    feedback, feedback_ref = (
        {k: v for k, v in (r.get("slam_feedback") or {}).items() if k != "socket"}
        for r in (result, reference)
    )
    if not feedback or feedback != feedback_ref:
        problems.append(f"{key}: slam_feedback differs from the D8b run")
    args, args_ref = result.get("arguments") or {}, reference.get("arguments") or {}
    for name in sorted(set(args) | set(args_ref)):
        if name in ARGUMENTS_BY_DESIGN:
            continue
        if snapshot_relative(args.get(name)) != snapshot_relative(args_ref.get(name)):
            problems.append(f"{key}: argument {name} differs from the D8b run")
    files = input_files(args, snapshot_root(args))
    files_ref = input_files(args_ref, snapshot_root(args_ref))
    for label in sorted(set(files) | set(files_ref)):
        sha, sha_ref = files.get(label), files_ref.get(label)
        if sha is None or sha_ref is None:
            problems.append(f"{key}: {label} cannot be read on both sides")
        elif sha != sha_ref:
            problems.append(f"{key}: {label} differs from the D8b run")
    return problems


def stop_problems(
    key: str, stops: list, stamps: np.ndarray, applied: np.ndarray, span
) -> list[str]:
    """The four scheduled stops as made: at their gaps in order, finite times, command zero
    from the trigger's interval to the release, standing HOLD_S, a restart after."""
    problems = []
    if [s.get("gap_m") for s in stops] != list(REQUIRED_GAPS_M):
        problems.append(
            f"{key}: safety stops at {[s.get('gap_m') for s in stops]}, need {list(REQUIRED_GAPS_M)}"
        )
        return problems
    previous_end = span[0]
    triggers = [s.get("trigger_s") for s in stops[1:]] + [span[1]]
    for stop, next_trigger in zip(stops, triggers, strict=True):
        times = [stop.get(n) for n in ("trigger_s", "still_since_s", "end_s")]
        d_est = stop.get("d_est_m")
        if not all(
            isinstance(v, (int, float)) and math.isfinite(v) for v in [*times, d_est]
        ):
            problems.append(f"{key}: the {stop['gap_m']} m stop is incomplete")
            continue
        trigger, still, end = times
        if not (previous_end <= trigger <= still <= end <= span[1] + 1e-9):
            problems.append(
                f"{key}: the {stop['gap_m']} m stop's times are out of order"
            )
            continue
        if end - still < HOLD_S - 1e-6 or d_est > stop["gap_m"] + 1e-9:
            problems.append(
                f"{key}: the {stop['gap_m']} m stop did not stand {HOLD_S} s at its gap"
            )
        held = (stamps >= trigger + TICK_S - 0.25 * TICK_S) & (
            stamps <= end + 0.25 * TICK_S
        )
        if not held.any() or np.any(np.abs(applied[held]) > 1e-12):
            problems.append(
                f"{key}: the command was not zero through the {stop['gap_m']} m stop"
            )
        # The restart before the next stop (or the span's end), not a later one (Codex
        # S4a-1 ⓪ 3rd review P3).
        limit = (
            next_trigger
            if isinstance(next_trigger, (int, float)) and math.isfinite(next_trigger)
            else span[1]
        )
        after = (stamps > end + 0.25 * TICK_S) & (stamps <= limit + 1e-9)
        if not np.any(np.abs(applied[after]) > 0):
            problems.append(f"{key}: no restart after the {stop['gap_m']} m stop")
        previous_end = end
    return problems


def matrix_problems(
    runs: dict, bounds: dict, references: dict, manifest: dict, commands: dict
) -> list[str]:
    """Why these runs are not the plan's measurement matrix (empty: they are).

    references: (seed, speed) -> the D8b calibration run's result.json; manifest: the
    sources allowed to differ, with their hashes; commands: key -> (stamps, applied)."""
    from tools import d8b_calibration as CAL

    problems = []
    condition = bounds["condition"]
    expected_sources = condition["source_sha256"]["source_sha256"]
    allowed = manifest.get("allowed_sources") or {}
    try:
        CAL.check_contract(list(runs.values()))
    except ValueError as exc:
        problems.append(str(exc))
    found = {}
    for key, run in runs.items():
        seed = run.result.get("seed")
        if (run.result.get("arguments") or {}).get("slam_noise_seed") != seed:
            problems.append(
                f"{key}: slam noise seed differs from the scene seed {seed}"
            )
        for name in CONDITION_KEYS:
            if run.meta.get(name) != condition.get(name):
                problems.append(f"{key}: meta {name} differs from the bounds condition")
        for name in ASSET_HASH_KEYS:
            if run.result.get(name) != condition["source_sha256"].get(name):
                problems.append(f"{key}: {name} differs from the bounds condition")
        sources = run.result.get("source_sha256") or {}
        for name in sorted(set(sources) | set(expected_sources)):
            if name in allowed:
                if sources.get(name) != allowed[name]:
                    problems.append(
                        f"{key}: source {name} is not the reviewed revision"
                    )
            elif sources.get(name) != expected_sources.get(name):
                problems.append(
                    f"{key}: source {name} differs from the bounds condition"
                )
        if run.meta.get("pocket_check"):
            problems.append(f"{key}: D5 is on")
        if run.dwells:
            problems.append(f"{key}: the run has dwells")
        transitions = run.result.get("transitions") or []
        if not any(
            tr.get("from") == "insert" and tr.get("to") == "lift" for tr in transitions
        ):
            problems.append(f"{key}: the insertion did not finish")
        gaps = run.meta.get("safety_stop_gaps_m")
        if gaps is None or tuple(gaps) != REQUIRED_GAPS_M:
            problems.append(
                f"{key}: safety stop gaps {gaps}, need {list(REQUIRED_GAPS_M)}"
            )
        span = run.phase_span()
        if span is None:
            problems.append(f"{key}: no approach/insert span")
        else:
            problems += stop_problems(
                key, run.meta.get("safety_stops") or [], *commands[key], span
            )
        speed = run.meta.get("approach_straight_speed_mps")
        match = [
            s for s in REQUIRED_SPEEDS if speed is not None and abs(speed - s) < 1e-9
        ]
        if seed not in REQUIRED_SEEDS or not match:
            problems.append(f"{key}: seed {seed} at {speed} m/s is not in the matrix")
            continue
        found.setdefault((seed, match[0]), []).append(key)
        reference = references.get((seed, match[0]))
        if reference is None:
            problems.append(
                f"{key}: no D8b calibration run at seed {seed}, {match[0]} m/s"
            )
        else:
            problems += reference_problems(key, run.result, reference)
    for seed in REQUIRED_SEEDS:
        for speed in REQUIRED_SPEEDS:
            n = len(found.get((seed, speed), []))
            if n != 1:
                problems.append(f"seed {seed} at {speed} m/s: {n} runs, need 1")
    return problems


def load_references(bounds: dict, data_root: Path = ROOT) -> dict:
    """(seed, speed) -> result.json of the D8b calibration runs the bounds were measured
    from (D5 off), each checked against the bounds file's provenance hash."""
    import hashlib

    references = {}
    for directory, hashes in (bounds["provenance"]["runs"]).items():
        path = (
            data_root / directory / "result.json"
        )  # relative keys: from the data root
        if not path.exists():
            continue
        data = path.read_bytes()
        if hashlib.sha256(data).hexdigest() != hashes.get("result.json"):
            raise SystemExit(f"{path}: not the result the bounds were measured from")
        result = json.loads(data)
        if (result.get("arguments") or {}).get("pocket_check"):
            continue
        speed = (result.get("arguments") or {}).get("approach_straight_speed_mps")
        if speed is not None:
            references[(result.get("seed"), round(float(speed), 6))] = result
    return references


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--bounds", type=Path, default=ROOT / "config/near_field_bounds_measured.json"
    )
    parser.add_argument("--manifest", type=Path, required=True)
    parser.add_argument(
        "--data-root",
        type=Path,
        default=ROOT,
        help="where the bounds file's provenance run paths are relative to",
    )
    parser.add_argument("--runs", nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    from tools import d8b_calibration as CAL

    bounds = json.loads(args.bounds.read_text())
    lat = bounds["latency"]
    align_age = (
        lat["max_s"] + lat["read_delay_s"]
    )  # a result's pixel age at its arrival, at most
    period = lat["render_period_s"]
    manifest = json.loads(args.manifest.read_text())
    references = load_references(bounds, args.data_root)
    runs, per_run, commands = {}, {}, {}
    for key in args.runs:
        run = CAL.Run.load(Path(key))
        truth = np.load(Path(key) / "pocket_frames/truth.npz")
        if "applied_speed_mps" not in truth.files:
            raise SystemExit(
                f"{key}: the truth record has no applied command (record with S4a-1 ⓪)"
            )
        span = run.phase_span()
        if span is None or run.control is None:
            raise SystemExit(f"{key}: no approach/insert span or no control record")
        terms = drive_terms(
            run.control, truth["stamp_s"], truth["applied_speed_mps"], *span
        )
        terms["safety_stops"] = len(run.meta.get("safety_stops") or [])
        terms["speed_mps"] = run.meta.get("approach_straight_speed_mps")
        runs[key], per_run[key] = run, terms
        commands[key] = (
            np.asarray(truth["stamp_s"], dtype=float),
            np.asarray(truth["applied_speed_mps"], dtype=float),
        )
    problems = matrix_problems(runs, bounds, references, manifest, commands)
    # The condition the runner checks before it uses these terms: identical in all six.
    conditions = {
        key: run_condition(run.result, snapshot_root(run.result.get("arguments") or {}))
        for key, run in runs.items()
    }
    condition = next(iter(conditions.values()))
    for key, other in conditions.items():
        problems += [f"{key}: {p}" for p in condition_problems(condition, other)]
    delta_len = merge([t["delta_len_m"] for t in per_run.values()])
    delta_s = merge([t["delta_s_m"] for t in per_run.values()])
    control_excess = merge([t["control_excess_m"] for t in per_run.values()])
    delta_v = max(t["delta_v_mps"] for t in per_run.values())
    # Never above the fastest measured straight speed: past it the terms are extrapolated.
    v_max = max(t["speed_mps"] for t in per_run.values() if t["speed_mps"] is not None)
    value = cruise(
        delta_len, control_excess, delta_s, delta_v, align_age, period, v_max
    )
    worst, worst_u = (
        budget_cycle(
            value, delta_len, control_excess, delta_s, delta_v, align_age, period
        )
        if value
        else (None, None)
    )
    out = {
        "status": "confirmed" if not problems else "diagnostic",
        "problems": problems,
        "manifest": manifest,
        "condition": condition,
        # Compared by name only: the same URL does not prove the same remote content
        # (Codex S4a-1 ⓪ 5th review).
        "unverified_inputs": [
            "asset_root: the remote Isaac warehouse USD, compared by URL only"
        ],
        "provenance": {
            "bounds": {"path": str(args.bounds), "sha256": file_sha256(args.bounds)},
            "manifest_sha256": file_sha256(args.manifest),
            "runs": {
                key: {
                    name: file_sha256(Path(key) / name)
                    for name in (
                        "result.json",
                        "slam_control.npy",
                        "pocket_frames/truth.npz",
                        "pocket_frames/index.json",
                    )
                }
                for key in runs
            },
            "tool_sha256": file_sha256(Path(__file__)),
        },
        "runs": {
            k: {kk: vv for kk, vv in v.items() if not kk.endswith("_m")}
            for k, v in per_run.items()
        },
        "delta_v_mps": delta_v,
        "align_age_s": align_age,
        "render_period_s": period,
        "stop_latency_s": STOP_LATENCY_S,
        "cruise_cap_mps": v_max,
        ("cruise_mps" if not problems else "diagnostic_cruise_mps"): value,
        "budget_at_cruise_m": worst,
        "budget_worst_u_s": worst_u,
        "delta_len_m": delta_len,
        "control_excess_m": control_excess,
        "delta_s_m": delta_s,
        "tick_s": TICK_S,
    }
    args.output.write_text(json.dumps(out, indent=1) + "\n")
    print(
        json.dumps(
            {k: v for k, v in out.items() if k not in ("delta_len_m", "delta_s_m")},
            indent=1,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
