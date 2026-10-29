"""B-P2d's judgement of unattended start relocalization sessions (docs/plans/2026-10-29-start-relocalization.md 4.0-4.3,
4.4b, 7).

One session per start (deploy/console/reloc_session.sh: one epoch, the probe's hold), each read from its own records:
``supervisor.jsonl``, ``live_port.json``, ``epoch_1/run/result.json`` (``start_reloc``, ``reloc_console``,
``source_sha256``, ``arguments``), the runner's spooled ``slam.records`` and ``probe.json``.

- **Classes (4.0).** Invalid: the physical input failed before any matching -- ``reloc_invalid_input`` in the record
  or as the session's end reason, whether the record was made or not -- out of the denominator, never replaced. Valid:
  everything else, every later failure included: a start with no record at all (``no_record``), a set-up error
  (``config_error``). Started: a valid session that reached running -- the supervisor's ``localization_ready`` or the
  port file's ``localized_wall``, or the record's running-tick verdict; a start whose verdict is lost leaves the safety
  unjudged (it cannot count as "no wrong start").
- **Pose (4.0).** The SLAM pose at the running tick against the same tick's truth (``start_reloc.verdict``): success
  <= 0.10 m and <= 0.05 rad, wrong > 0.30 m or > 0.10 rad, imprecise between (bounds inclusive). The matcher's
  estimate is recorded, not judged.
- **Success (4.2).** Confirmed by the matcher without an operator, started, its pose a success, and the stationary
  hold: the runner's complete spool of SLAM replies from the running tick for HOLD_S simulated seconds, each processed
  or skipped in tracking mode, from the tick on and to the hold's end with no gap over MAX_REPLY_GAP_S, and the session
  ended by the probe after it (not before).
- **Floors.** Safety (4.1): valid >= 60, wrong 0, imprecise 0, every start judged (the one-sided 95 % upper bounds on
  the valid and the started denominators recorded, exact Clopper-Pearson with the events counted). Success (4.2):
  observed >= 70 % of valid (the CP lower bound recorded). Time (4.3): deadline hits (a matcher timeout or a late
  confirmation refused) <= 10 % of valid; no matcher stage over 32 s from the generation's first scan; every start's
  bridge stage (reloc_pose -> localization ready) known and within the bound frozen before the sessions.
- **Records (plan 7).** F1: for each operator request, whether the truth (success range) is among the shown and among
  all the matcher's candidates. F2: the console's gaps.
- **Fingerprint (4.4b).** The sha256 of every file on the verdict path and the bridge image, and each session held to
  it by the runner's own source hashes; one argument set over the sessions (the start, its output and the epoch folder
  aside). Pooled only when all match.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SUCCESS_M, SUCCESS_RAD = 0.10, 0.05
WRONG_M, WRONG_RAD = 0.30, 0.10
EPS = 1e-9  # the registered bounds are inclusive; a pose on one is not pushed over it by rounding
HOLD_S = 10.0
DEADLINE_S = 30.0  # the session deadline from the generation's first scan (RelocConfig, frozen with B-P1b)
MAX_REPLY_GAP_S = 0.5  # the 10 Hz replies' grid: a longer silence is no shown hold
MIN_VALID = 60
SUCCESS_FLOOR = 0.70
DEADLINE_SHARE = 0.10
WORKER_RETURN_S = 32.0
OK_REPLIES = ("processed", "skipped")
# the arguments that differ by session by design: its start, its output, its epoch folder, its ports
# the world (plan 4.4a pools worlds under one measurement condition): kept per row, out of the argument set
WORLD_ARGUMENTS = ("seed", "layout", "scene_file", "slam_map_bundle", "expected_map_id", "slam_choice")
# the whole core and runner trees besides the files listed: a transitive dependency changes the verdict path too
FINGERPRINT_TREES = ("src/forklift_core/**/*.py", "sim/isaac/*.py")
PER_SESSION_ARGUMENTS = ("start_pose", "output", "reloc_dir", "live_port", "live_port_file", "slam_feedback",
                         "slam_map_dir", "live_epoch")
FINGERPRINT_FILES = (
    # matching and refinement, the map, the tracker and the link (plan 4.4b)
    "src/forklift_core/localization/global_match.py", "src/forklift_core/localization/scan_match.py",
    "src/forklift_core/localization/slam_map.py", "src/forklift_core/localization/slam_pose.py",
    "src/forklift_core/localization/slam_link.py", "src/forklift_core/localization/start_reloc.py",
    "src/forklift_core/localization/relocalization_samples.py",
    "docs/validation/assets/b-p1b/match_config.json", "docs/validation/assets/b-p1b/dev_choice.json",
    # the start relocalization in the runner, its console, the supervisor and the mission core the runner drives
    "sim/isaac/run_transport.py", "sim/isaac/scan_capture.py", "sim/isaac/reloc_phase.py",
    "sim/isaac/reloc_console.py", "sim/isaac/live_runtime.py", "tools/live_supervisor.py",
    "src/forklift_core/mission/core.py",
    # the scans, their noise and the odometry the bridge is fed (Codex B-P2d tooling review P2)
    "sim/isaac/obstacle_layer.py", "sim/isaac/planar_lidar.py", "src/forklift_core/sensors/lidar.py",
    "src/forklift_core/localization/wheel_odometry.py", "src/forklift_core/localization/encoders.py",
    "src/forklift_core/localization/gyro.py",
    "config/isaac_slam_lidar.yaml", "config/obstacle_layer_single_team_spec.yaml",
    "config/obstacle_layer_single_team_spec_gyro.yaml", "config/gyro_model_spec_typical.yaml",
    "config/isaac_transport_measured.yaml",
    # the SLAM bridge
    "ros2/src/forklift_ros/forklift_ros/isaac_slam_bridge.py", "tools/slam_bridge_container.sh",
    "tools/slam_localization_params.py", "ros2/src/forklift_bringup/config/slam_toolbox_isaac_online.yaml",
    # the launch, the probe and this judgement (its definitions above)
    "deploy/console/isaac_live.sbatch", "deploy/console/reloc_session.sh", "tools/live_console_probe.py",
    "tools/reloc_judge.py",
)


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _gap(a, b) -> tuple[float, float]:
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(_wrap(a[2] - b[2]))


def pose_class(err_m: float, err_rad: float) -> str:
    if err_m <= SUCCESS_M + EPS and err_rad <= SUCCESS_RAD + EPS:
        return "success"
    if err_m > WRONG_M + EPS or err_rad > WRONG_RAD + EPS:
        return "wrong"
    return "imprecise"


def _near(truth, candidates) -> bool:
    return any(pose_class(*_gap(c["base"], truth)) == "success" for c in candidates or ())


def _load(path: Path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def _supervisor(d: Path) -> list:
    try:
        return [json.loads(line) for line in (d / "supervisor.jsonl").read_text().splitlines() if line.strip()]
    except (OSError, ValueError):
        return []


def _records(run: Path) -> list | None:
    """The spooled SLAM replies, complete only (None: incomplete or unreadable -- the hold cannot be shown)."""
    from forklift_core.live.spool import load_list

    try:
        return load_list(run / "spool", "slam.records", allow_incomplete=False)
    except Exception:  # noqa: BLE001
        return None


def _hold_ok(records: list | None, verdict: dict, probe: dict | None) -> bool:
    if records is None:
        return False
    t0 = float(verdict["sim_s"])
    # from the running tick on: the warm-up's replies before it are the normal bootstrap (Codex round 2 P2)
    window = sorted((r for r in records if t0 <= float(r.get("stamp_s", -1.0)) <= t0 + HOLD_S),
                    key=lambda r: float(r["stamp_s"]))
    if not window or any(r.get("status") not in OK_REPLIES or r.get("mode") != "tracking" for r in window):
        return False
    stamps = [t0] + [float(r["stamp_s"]) for r in window] + [t0 + HOLD_S]
    if any(b - a > MAX_REPLY_GAP_S for a, b in zip(stamps, stamps[1:], strict=False)):
        return False  # a silence inside the hold, or the replies stop before its end
    hold = (probe or {}).get("hold") or {}
    return hold.get("closed_during") is None and ((probe or {}).get("session_end") or {}).get("t") == "ack"


def _args_signature(arguments) -> str | None:
    if not isinstance(arguments, dict):
        return None
    kept = {k: v for k, v in arguments.items() if k not in PER_SESSION_ARGUMENTS + WORLD_ARGUMENTS}
    return hashlib.sha256(json.dumps(kept, sort_keys=True, default=str).encode()).hexdigest()


def _source_mismatches(sources, fp: dict | None) -> list:
    """The fingerprint's runner-tree files (the core package and sim/isaac, which the runner hashes in full) whose hash
    the session's runner recorded differently -- or did not record: missing evidence is no match (Codex round 2 P2).
    Its keys: forklift_core/... for the package, repository paths otherwise."""
    if fp is None:
        return []
    if not isinstance(sources, dict):
        return ["source_sha256 (not recorded)"]
    out = []
    for rel, digest in fp["files"].items():
        if not (rel.startswith("src/forklift_core/") or rel.startswith("sim/isaac/")):
            continue  # outside the runner's own hashing (the bridge, the launch, this judge): the fingerprint holds them
        key = rel[len("src/"):] if rel.startswith("src/") else rel
        if key not in sources:
            out.append(f"{rel} (not recorded)")
        elif sources[key] != digest:
            out.append(rel)
    return out


def _world(arguments) -> dict | None:
    if not isinstance(arguments, dict):
        return None
    return {k: arguments.get(k) for k in WORLD_ARGUMENTS}


def _reason(row: dict, result) -> str:
    return str((row["exit"] or {}).get("reason") or (result or {}).get("failure_reason") or "")


def judge_session(d: Path, start: dict, fp: dict | None = None) -> dict:
    row = {"id": start["id"], "stratum": start.get("stratum"), "start": start.get("base_world"), "class": "valid",
           "started": False, "outcome": None, "pose_class": None, "error_m": None, "error_rad": None,
           "hold_ok": None, "bridge_stage_s": None, "matcher_wall_max_s": None, "matcher_stage_max_s": None,
           "deadline_hit": False, "attempts": None, "truth_in_shown": None, "truth_in_candidates": None, "gaps": [],
           "exit": None, "source_mismatches": [], "args_signature": None, "world": None}
    result = _load(d / "epoch_1" / "run" / "result.json")
    events = _supervisor(d)
    exits = [e for e in events if e.get("event") == "exit"]
    row["exit"] = exits[-1] if exits else None
    walls = {e.get("event"): e.get("wall") for e in events}
    port = _load(d / "live_port.json") or {}
    localized = walls.get("localization_ready") is not None or port.get("localized_wall") is not None
    rec = (result or {}).get("start_reloc")
    if result is not None:
        row["source_mismatches"] = _source_mismatches(result.get("source_sha256"), fp)
        row["args_signature"] = _args_signature(result.get("arguments"))
        row["world"] = _world(result.get("arguments"))
    if (rec or {}).get("status") == "invalid" or _reason(row, result).startswith("reloc_invalid_input"):
        row["class"], row["outcome"] = "invalid", "invalid"  # by the reason, record or not (Codex P2)
        return row
    if rec is None:
        if localized:  # reached running, then its record was lost: started, its pose unknown (Codex P2)
            row["started"], row["outcome"] = True, "started_unrecorded"
        elif _reason(row, result).startswith("reloc_config_error"):
            row["outcome"] = "config_error"
        else:
            row["outcome"] = "no_record"  # a valid failure: never dropped
        return row
    row["gaps"] = (result.get("reloc_console") or {}).get("gaps") or []
    attempts = rec.get("attempts") or []
    row["attempts"] = len(attempts)
    wall = [a["wall_s"] for a in attempts if a.get("wall_s") is not None]
    stage = [a["stage_s"] for a in attempts if a.get("stage_s") is not None]
    row["matcher_wall_max_s"] = max(wall) if wall else None
    row["matcher_stage_max_s"] = max(stage) if stage else None
    row["deadline_hit"] = any(a.get("status") == "timeout" or a.get("reason") in ("timeout", "deadline")
                              or (a.get("stage_s") is not None and a["stage_s"] > DEADLINE_S)
                              for a in attempts)  # any result past the shared deadline (Codex round 2 P2)
    truth = rec.get("truth")
    status = rec.get("status")
    if status == "needs_operator":
        row["outcome"] = "needs_operator"
        if truth is not None:
            row["truth_in_shown"] = _near(truth, rec.get("shown"))
            row["truth_in_candidates"] = _near(truth, [c for a in attempts for c in a.get("candidates") or ()])
        return row
    if status != "chosen":
        row["outcome"] = status or "unknown"
        return row
    if walls.get("reloc_pose") is not None and walls.get("localization_ready") is not None:
        row["bridge_stage_s"] = walls["localization_ready"] - walls["reloc_pose"]
    verdict = rec.get("verdict")
    if verdict is None:
        if localized:
            row["started"], row["outcome"] = True, "started_unrecorded"
        else:
            row["outcome"] = "bridge_refused" if _reason(row, result).startswith("slam_not_localized") else "not_started"
        return row
    row["started"] = True
    err_m, err_rad = _gap(verdict["slam_base"], verdict["truth_base"])
    row["error_m"], row["error_rad"] = err_m, err_rad
    row["pose_class"] = pose_class(err_m, err_rad)
    row["source"] = rec.get("source")
    row["hold_ok"] = _hold_ok(_records(d / "epoch_1" / "run"), verdict, _load(d / "probe.json"))
    if row["pose_class"] != "success":
        row["outcome"] = row["pose_class"]
    elif rec.get("source") != "matcher":
        row["outcome"] = "operator"  # no operator in an unattended batch: never a success
    elif not row["hold_ok"]:
        row["outcome"] = "hold_failed"
    else:
        row["outcome"] = "success"
    return row


def judge_sessions(root: Path, starts: list, fp: dict | None = None) -> list:
    return [judge_session(Path(root) / s["id"], s, fp) for s in starts]


def cp_upper(k: int, n: int) -> float | None:
    """One-sided 95 % Clopper-Pearson upper bound of k events in n (0 events: 1 - 0.05^(1/n))."""
    if n <= 0:
        return None
    if k == 0:
        return 1.0 - 0.05 ** (1.0 / n)
    if k >= n:
        return 1.0
    from scipy.stats import beta

    return float(beta.ppf(0.95, k + 1, n - k))


def cp_upper_zero(n: int) -> float | None:
    return cp_upper(0, n)


def cp_lower(k: int, n: int) -> float | None:
    """One-sided 95 % Clopper-Pearson lower bound of k successes in n."""
    if n <= 0:
        return None
    if k == 0:
        return 0.0
    from scipy.stats import beta

    return float(beta.ppf(0.05, k, n - k + 1))


def summarize(rows: list, bridge_bound_s: float | None = None) -> dict:
    valid = [r for r in rows if r["class"] == "valid"]
    started = [r for r in valid if r["started"]]
    n_valid, n_started = len(valid), len(started)
    wrong = sum(r.get("pose_class") == "wrong" for r in started)
    imprecise = sum(r.get("pose_class") == "imprecise" for r in started)
    unjudged = sum(r.get("pose_class") is None for r in started)
    success = sum(r["outcome"] == "success" for r in valid)
    deadline = sum(bool(r.get("deadline_hit")) for r in valid)
    slow = sum((r.get("matcher_stage_max_s") or 0.0) > WORKER_RETURN_S for r in valid)
    bridge = [r.get("bridge_stage_s") for r in started]
    missing = sum(b is None for b in bridge)
    judged = n_valid >= MIN_VALID
    requests = [r for r in valid if r["outcome"] == "needs_operator"]
    outcomes: dict = {}
    for r in rows:
        outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1
    signatures = {r.get("args_signature") for r in rows if r.get("args_signature") is not None}
    # a valid session whose matcher time is not known (it ended with no record) is no kept time (Codex round 2 P2)
    unmeasured = sum(r.get("matcher_stage_max_s") is None and r["outcome"] != "config_error" for r in valid)
    off = sum(bool(r.get("source_mismatches")) for r in rows)
    events = wrong + imprecise
    return {
        "valid": n_valid, "invalid": sum(r["class"] == "invalid" for r in rows), "started": n_started,
        "wrong": wrong, "imprecise": imprecise, "unjudged_starts": unjudged, "success": success,
        "outcomes": outcomes,
        # an unjudged start may have been wrong: no bound then, and a conservative one counting it (Codex round 2 P2)
        "safety_upper_valid": None if unjudged else cp_upper(events, n_valid),
        "safety_upper_started": None if unjudged else cp_upper(events, n_started),
        "safety_upper_valid_conservative": cp_upper(events + unjudged, n_valid),
        "safety_upper_started_conservative": cp_upper(events + unjudged, n_started),
        "success_rate": success / n_valid if n_valid else None, "success_cp_lower": cp_lower(success, n_valid),
        "deadline_hits": deadline, "worker_over_32s": slow, "matcher_time_unmeasured": unmeasured,
        "bridge_stage_max_s": max((b for b in bridge if b is not None), default=None),
        "bridge_stage_missing": missing, "bridge_bound_s": bridge_bound_s,
        "sessions_off_fingerprint": off, "argument_sets": len(signatures),
        "arguments_missing": sum(r.get("args_signature") is None for r in rows if r["outcome"] != "no_record"),
        "judged": judged,
        "floors": {
            "safety": None if not judged or (unjudged and not events) else events == 0,
            "success": (success / n_valid >= SUCCESS_FLOOR) if judged else None,
            "matcher_time": (None if not judged or (unmeasured and deadline <= DEADLINE_SHARE * n_valid
                                                         and slow == 0)
                             else deadline <= DEADLINE_SHARE * n_valid and slow == 0),
            "bridge_time": (None if bridge_bound_s is None or not judged or missing
                            else all(b <= bridge_bound_s for b in bridge)),
        },
        "f1": {"requests": len(requests), "truth_in_shown": sum(bool(r.get("truth_in_shown")) for r in requests),
               "truth_in_candidates": sum(bool(r.get("truth_in_candidates")) for r in requests)},
        "f2": {"sessions_with_gaps": sum(bool(r.get("gaps")) for r in rows),
               "gap_max_s": max((g["gap_s"] for r in rows for g in r.get("gaps") or ()), default=None)},
    }


def fingerprint(root: Path, image: str | None) -> dict:
    root = Path(root)
    names = set(FINGERPRINT_FILES)
    for pattern in FINGERPRINT_TREES:
        names |= {p.relative_to(root).as_posix() for p in root.glob(pattern) if "__pycache__" not in p.parts}
    files = {}
    for rel in sorted(names):
        p = root / rel
        files[rel] = hashlib.sha256(p.read_bytes()).hexdigest() if p.is_file() else None
    return {"files": files, "image": image}


def fingerprint_mismatches(frozen: dict, now: dict) -> list:
    keys = sorted(set(frozen["files"]) | set(now["files"]))
    out = [k for k in keys if frozen["files"].get(k) != now["files"].get(k)]
    if frozen.get("image") != now.get("image"):
        out.append("image")
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--sessions", type=Path, required=True, help="the folder of one session folder per start id")
    ap.add_argument("--starts", type=Path, required=True, help="the start list (scan_capture_poses/1)")
    ap.add_argument("--code-root", type=Path, required=True, help="the snapshot the sessions ran")
    ap.add_argument("--image", required=True, help="the SLAM bridge image id the sessions ran")
    ap.add_argument("--frozen", type=Path, default=None, help="the first freeze's fingerprint and bridge bound")
    ap.add_argument("--out", type=Path, required=True)
    a = ap.parse_args(argv)
    fp = fingerprint(a.code_root, a.image)
    frozen = _load(a.frozen) if a.frozen else None
    if a.frozen and (frozen is None or "fingerprint" not in frozen):
        print(f"reloc_judge: the frozen record {a.frozen} is not there or has no fingerprint", file=sys.stderr)
        return 2
    mismatches = fingerprint_mismatches(frozen["fingerprint"], fp) if frozen else None
    starts = json.loads(a.starts.read_text())["poses"]
    rows = judge_sessions(a.sessions, starts, fp)
    summary = summarize(rows, (frozen or {}).get("bridge_bound_s"))
    out = {"fingerprint": fp, "fingerprint_mismatches": mismatches, "summary": summary, "rows": rows}
    a.out.write_text(json.dumps(out, indent=1, default=str) + "\n")
    print(json.dumps({"summary": summary, "fingerprint_mismatches": mismatches}, indent=1, default=str))
    poolable = (not mismatches and summary["sessions_off_fingerprint"] == 0 and summary["argument_sets"] == 1
                and summary["arguments_missing"] == 0)
    return 0 if poolable else 2


if __name__ == "__main__":
    sys.exit(main())
