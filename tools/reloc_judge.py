"""B-P2d's judgement of unattended start relocalization sessions (docs/plans/2026-10-29-start-relocalization.md 4.0-4.3,
4.4b, 7).

One session per start (deploy/console/reloc_session.sh: one epoch, the probe's hold), each read from its own records:
``supervisor.jsonl``, ``epoch_1/run/result.json`` (``start_reloc``, ``reloc_console``), the runner's spooled
``slam.records`` and ``probe.json``.

- **Classes (4.0).** Invalid: the physical input failed before any matching (``reloc_invalid_input``), out of the
  denominator, never replaced. Valid: everything else, every later failure included; a start with no record at all is a
  valid failure (``no_record``). Started: a valid session that reached running.
- **Pose (4.0).** The SLAM pose at the running tick against the same tick's truth (``start_reloc.verdict``): success
  <= 0.10 m and <= 0.05 rad, wrong > 0.30 m or > 0.10 rad, imprecise between. The matcher's estimate is recorded, not
  judged.
- **Success (4.2).** Confirmed by the matcher without an operator, started, its pose a success, and the stationary
  hold: the runner's SLAM replies from the running tick for HOLD_S simulated seconds all processed or skipped (no
  link failure), covering the whole hold, and the session ended by the probe after it (not before).
- **Floors.** Safety (4.1): valid >= 60, wrong 0, imprecise 0 (the one-sided 95 % upper bound on the valid and on the
  started denominator both recorded). Success (4.2): observed >= 70 % of valid (Clopper-Pearson lower bound recorded).
  Time (4.3): deadline hits <= 10 % of valid, no matcher return over 32 s; the bridge stage (reloc_pose -> running)
  within the bound frozen before the sessions.
- **Records (plan 7).** F1: for each operator request, whether the truth (success range) is among the shown and among
  all the matcher's candidates. F2: the console's gaps.
- **Fingerprint (4.4b).** The sha256 of every file on the verdict path and the bridge image: pooled only when equal.
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
HOLD_S = 10.0
HOLD_SLACK_S = 0.5  # the replies' 10 Hz grid against the hold's end
MIN_VALID = 60
SUCCESS_FLOOR = 0.70
DEADLINE_SHARE = 0.10
WORKER_RETURN_S = 32.0
OK_REPLIES = ("processed", "skipped")
FINGERPRINT_FILES = (
    # matching and refinement, the map, the tracker and the link (plan 4.4b)
    "src/forklift_core/localization/global_match.py", "src/forklift_core/localization/scan_match.py",
    "src/forklift_core/localization/slam_map.py", "src/forklift_core/localization/slam_pose.py",
    "src/forklift_core/localization/slam_link.py", "src/forklift_core/localization/start_reloc.py",
    "src/forklift_core/localization/relocalization_samples.py",
    "docs/validation/assets/b-p1b/match_config.json", "docs/validation/assets/b-p1b/dev_choice.json",
    # the start relocalization in the runner, its console and the supervisor
    "sim/isaac/run_transport.py", "sim/isaac/scan_capture.py", "sim/isaac/reloc_phase.py",
    "sim/isaac/reloc_console.py", "sim/isaac/live_runtime.py", "tools/live_supervisor.py",
    # the scans and their noise
    "config/isaac_slam_lidar.yaml", "config/obstacle_layer_single_team_spec_gyro.yaml",
    "config/gyro_model_spec_typical.yaml",
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


EPS = 1e-9  # the registered bounds are inclusive; a pose on one is not pushed over it by rounding


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


def _records(run: Path) -> list:
    from forklift_core.live.spool import load_list

    try:
        return load_list(run / "spool", "slam.records", allow_incomplete=True)
    except Exception:  # noqa: BLE001 -- no spool: no replies (the hold cannot be shown)
        return []


def _hold_ok(records: list, verdict: dict, probe: dict | None) -> bool:
    t0 = float(verdict["sim_s"])
    window = [r for r in records if t0 <= float(r.get("stamp_s", -1.0)) <= t0 + HOLD_S]
    if not window or any(r.get("status") not in OK_REPLIES for r in window):
        return False
    if max(float(r["stamp_s"]) for r in window) < t0 + HOLD_S - HOLD_SLACK_S:
        return False  # the replies stop before the hold's end
    hold = (probe or {}).get("hold") or {}
    return hold.get("closed_during") is None and ((probe or {}).get("session_end") or {}).get("t") == "ack"


def judge_session(d: Path, start: dict) -> dict:
    row = {"id": start["id"], "stratum": start.get("stratum"), "start": start.get("base_world"), "class": "valid",
           "started": False, "outcome": None, "pose_class": None, "error_m": None, "error_rad": None,
           "hold_ok": None, "bridge_stage_s": None, "matcher_wall_max_s": None, "deadline_hit": False,
           "attempts": None, "truth_in_shown": None, "truth_in_candidates": None, "gaps": [], "exit": None}
    result = _load(d / "epoch_1" / "run" / "result.json")
    events = _supervisor(d)
    exits = [e for e in events if e.get("event") == "exit"]
    row["exit"] = exits[-1] if exits else None
    if result is None or "start_reloc" not in result:
        row["outcome"] = "no_record"  # a valid failure: never dropped
        return row
    rec = result["start_reloc"]
    row["gaps"] = (result.get("reloc_console") or {}).get("gaps") or []
    attempts = rec.get("attempts") or []
    row["attempts"] = len(attempts)
    walls = [a.get("wall_s") for a in attempts if a.get("wall_s") is not None]
    row["matcher_wall_max_s"] = max(walls) if walls else None
    row["deadline_hit"] = any(a.get("status") == "timeout" or a.get("reason") == "timeout" for a in attempts)
    truth = rec.get("truth")
    status = rec.get("status")
    if status == "invalid":
        row["class"], row["outcome"] = "invalid", "invalid"
        return row
    if status == "needs_operator":
        row["outcome"] = "needs_operator"
        if truth is not None:
            row["truth_in_shown"] = _near(truth, rec.get("shown"))
            row["truth_in_candidates"] = _near(truth, [c for a in attempts for c in a.get("candidates") or ()])
        return row
    if status != "chosen":
        row["outcome"] = status or "unknown"
        return row
    walls_ = {e.get("event"): e.get("wall") for e in events}
    verdict = rec.get("verdict")
    if verdict is None:
        reason = str((row["exit"] or {}).get("reason") or result.get("failure_reason") or "")
        row["outcome"] = "bridge_refused" if reason.startswith("slam_not_localized") else "not_started"
        return row
    row["started"] = True
    if walls_.get("reloc_pose") is not None and walls_.get("localization_ready") is not None:
        row["bridge_stage_s"] = walls_["localization_ready"] - walls_["reloc_pose"]
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


def judge_sessions(root: Path, starts: list) -> list:
    return [judge_session(Path(root) / s["id"], s) for s in starts]


def cp_upper_zero(n: int) -> float | None:
    """One-sided 95 % upper bound with no event in n (exact)."""
    return None if n <= 0 else 1.0 - 0.05 ** (1.0 / n)


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
    wrong = sum(r["pose_class"] == "wrong" for r in started)
    imprecise = sum(r["pose_class"] == "imprecise" for r in started)
    success = sum(r["outcome"] == "success" for r in valid)
    deadline = sum(bool(r.get("deadline_hit")) for r in valid)
    slow = sum((r.get("matcher_wall_max_s") or 0.0) > WORKER_RETURN_S for r in valid)
    bridge = [r["bridge_stage_s"] for r in started if r.get("bridge_stage_s") is not None]
    judged = n_valid >= MIN_VALID
    requests = [r for r in valid if r["outcome"] == "needs_operator"]
    outcomes: dict = {}
    for r in rows:
        outcomes[r["outcome"]] = outcomes.get(r["outcome"], 0) + 1
    return {
        "valid": n_valid, "invalid": sum(r["class"] == "invalid" for r in rows), "started": n_started,
        "wrong": wrong, "imprecise": imprecise, "success": success, "outcomes": outcomes,
        "safety_upper_valid": cp_upper_zero(n_valid), "safety_upper_started": cp_upper_zero(n_started),
        "success_rate": success / n_valid if n_valid else None, "success_cp_lower": cp_lower(success, n_valid),
        "deadline_hits": deadline, "worker_over_32s": slow,
        "bridge_stage_max_s": max(bridge) if bridge else None, "bridge_bound_s": bridge_bound_s,
        "judged": judged,
        "floors": {
            "safety": (wrong == 0 and imprecise == 0) if judged else None,
            "success": (success / n_valid >= SUCCESS_FLOOR) if judged else None,
            "matcher_time": (deadline <= DEADLINE_SHARE * n_valid and slow == 0) if judged else None,
            "bridge_time": (None if bridge_bound_s is None or not judged
                            else all(b <= bridge_bound_s for b in bridge)),
        },
        "f1": {"requests": len(requests), "truth_in_shown": sum(bool(r.get("truth_in_shown")) for r in requests),
               "truth_in_candidates": sum(bool(r.get("truth_in_candidates")) for r in requests)},
        "f2": {"sessions_with_gaps": sum(bool(r.get("gaps")) for r in rows),
               "gap_max_s": max((g["gap_s"] for r in rows for g in r.get("gaps") or ()), default=None)},
    }


def fingerprint(root: Path, image: str | None) -> dict:
    files = {}
    for rel in FINGERPRINT_FILES:
        p = Path(root) / rel
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
    mismatches = fingerprint_mismatches(frozen["fingerprint"], fp) if frozen else None
    starts = json.loads(a.starts.read_text())["poses"]
    rows = judge_sessions(a.sessions, starts)
    summary = summarize(rows, (frozen or {}).get("bridge_bound_s"))
    out = {"fingerprint": fp, "fingerprint_mismatches": mismatches, "summary": summary, "rows": rows}
    a.out.write_text(json.dumps(out, indent=1, default=str) + "\n")
    print(json.dumps({"summary": summary, "fingerprint_mismatches": mismatches}, indent=1, default=str))
    return 0 if not mismatches else 2


if __name__ == "__main__":
    sys.exit(main())
