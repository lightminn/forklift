"""Post-fix Isaac baseline: per-run fields and descriptive aggregates of riib runs (plan
docs/plans/2026-11-15-post-fix-baseline.md 3.3 and 4). No verdict, no threshold.

Each run folder holds ``run/result.json`` and ``riib_defaults.txt`` (and, after the path display fix,
``run/path_display.jsonl`` and ``run/path_display.json``). The display mode is checked in two places (plan 3.3):

- ``overlay`` (R): ``arguments.path_display == "overlay"`` and the event log header's ``mode`` is ``overlay``.
- ``stage`` (S): no recorded ``arguments.path_display`` (a stage run records none, as every run before the fix) and the
  header's ``mode`` is ``stage``.
- ``prefix`` (a pre-fix comparator): neither the recorded value nor the header file.
- ``overlay-noheader`` (a post-fix comparator from before #513, e.g. the K1 re-check at 7744f285): the recorded value
  ``overlay`` and no header file -- the event log came with #513. Never for this baseline's own runs.

A run that fails its mode's check is refused and kept out of the aggregate. A field a record lacks (or holds in another
shape) is ``None`` and named in ``missing`` -- never a default; a metric no run of a group can read is
``not_comparable``. The failure classes and stages are ``tools/ct_survey.py``'s, so the live baseline reads the same.

  python baseline_summary.py RUNS_ROOT --mode overlay --out summary.json [--fields-out fields.json.gz]
  python baseline_summary.py RUNS_ROOT --mode overlay --compare PREFIX_ROOT --compare-mode prefix --out ...
"""
from __future__ import annotations

import argparse
import gzip
import importlib.util
import json
import re
import sys
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
_spec = importlib.util.spec_from_file_location("ct_survey", ROOT / "tools" / "ct_survey.py")
ct_survey = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(ct_survey)

NAME = re.compile(r"^[A-Za-z0-9]+_([A-Za-z0-9]+)_(.+)_s(\d+)$")
APPLIED = ("near_end_stand", "near_observe_arrival", "layer_latency_budget", "return_cusp_replans",
           "return_start_backoff", "exit_prefix_exempt", "low_clearance_guard", "restart_confirm_scans",
           "d7_reapproach_hold", "docking_degenerate_continue", "lidar_keepout_m", "encoder_model", "deterministic",
           "path_display")
MODES = ("overlay", "stage", "prefix", "overlay-noheader")


def parse_name(name: str) -> tuple[str, str, int]:
    """``<prefix>_<arm>_<scene>_s<seed>`` -> (arm, scene, seed)."""
    m = NAME.match(name)
    if not m:
        raise ValueError(f"not a run name: {name}")
    return m.group(1), m.group(2), int(m.group(3))


def _header_mode(run: Path):
    path = run / "run" / "path_display.jsonl"
    if not path.is_file():
        return None
    try:
        first = json.loads(path.read_text().splitlines()[0])
    except (IndexError, ValueError):
        return "unreadable"
    return first.get("mode") if first.get("t") == "header" else "unreadable"


def mode_refusal(mode: str, recorded, header) -> str | None:
    """Why a run is not of ``mode`` (plan 3.3), else None."""
    if mode == "overlay":
        if recorded != "overlay":
            return f"path_display recorded {recorded!r}, not 'overlay'"
        if header != "overlay":
            return f"path_display.jsonl header mode {header!r}, not 'overlay'"
        return None
    if mode == "stage":
        if recorded is not None:
            return f"path_display recorded {recorded!r}; a stage run records none"
        if header != "stage":
            return f"path_display.jsonl header mode {header!r}, not 'stage'"
        return None
    if mode == "overlay-noheader":
        if recorded != "overlay":
            return f"path_display recorded {recorded!r}, not 'overlay'"
        if header is not None:
            return f"a run from before #513 has no path_display.jsonl (header {header!r})"
        return None
    if mode == "prefix":
        if recorded is not None or header is not None:
            return f"a pre-fix run has no display record (recorded {recorded!r}, header {header!r})"
        return None
    raise ValueError(f"mode {mode!r}")


def _window(transitions: list, phase: str) -> tuple[float, float] | None:
    """[entered, left) of ``phase`` from the transitions (left = inf while the run ended in it), None if never entered."""
    start = end = None
    for t in transitions:
        if t.get("to") == phase and start is None:
            start = float(t["time_s"])
        elif t.get("from") == phase and start is not None and end is None:
            end = float(t["time_s"])
    if start is None:
        return None
    return start, (end if end is not None else float("inf"))


def _count_in(events: list, window) -> int:
    if window is None:
        return 0
    lo, hi = window
    return sum(1 for e in events if isinstance(e, dict) and e.get("time_s") is not None and lo <= float(e["time_s"]) < hi)


def run_fields(run: Path, mode: str) -> dict:
    """The fields the plan's metrics read, from one run folder."""
    run = Path(run)
    arm, scene, seed = parse_name(run.name)
    out: dict = {"name": run.name, "arm": arm, "scene": scene, "seed": seed, "refused": None, "missing": []}
    try:
        r = json.loads((run / "run" / "result.json").read_text())
    except (OSError, ValueError) as exc:
        out["refused"] = f"result.json unreadable: {exc}"
        return out
    args = r.get("arguments") if isinstance(r.get("arguments"), dict) else {}
    header = _header_mode(run)
    out["refused"] = mode_refusal(mode, args.get("path_display"), header)
    out["display_header"] = header
    missing = out["missing"]

    def get(key, kind):
        v = r.get(key)
        if not isinstance(v, kind):
            missing.append(key)
            return None
        return v

    out["success"] = r.get("success") if isinstance(r.get("success"), bool) else None
    out["phase"] = r.get("phase")
    reason = r.get("failure_reason")
    out["failure_reason"] = reason
    failed = out["success"] is False
    out["failure_class"] = ct_survey.classify(reason) if failed else None
    out["failure_stage"] = ct_survey.stage_of(reason, out["phase"]) if failed else None

    attempts = get("observation_attempts", list)
    if attempts:
        a0 = attempts[0] if isinstance(attempts[0], dict) else {}
        rec = a0.get("recognition") if isinstance(a0.get("recognition"), dict) else {}
        pocket = a0.get("pocket_observation") if isinstance(a0.get("pocket_observation"), dict) else {}
        out["far_attempts"] = len(attempts)
        out["far_recognition_status"] = rec.get("status")
        out["far_selected"] = rec.get("status") == "selected"
        out["far_pocket_valid"] = pocket.get("status") == "valid"
        out["far_capture_failure"] = a0.get("capture_failure")
    else:
        out.update(far_attempts=None if attempts is None else 0, far_recognition_status=None, far_selected=None,
                   far_pocket_valid=None, far_capture_failure=None)
    replans = r.get("observe_replans")
    out["observe_replans"] = len(replans) if isinstance(replans, list) else (replans if isinstance(replans, int) else None)

    near = get("near_capture", dict)
    out["near_status"] = None if near is None else (near.get("status") or "absent")

    transitions = get("transitions", list)
    cusps = get("cusp_replans", list)
    if transitions is None:
        out.update(transport_reached=None, transport_cusp_replans=None, return_cusp_replans=None)
    else:
        tw, rw = _window(transitions, "transport"), _window(transitions, "return_home")
        out["transport_reached"] = tw is not None
        out["transport_cusp_replans"] = None if cusps is None else _count_in(cusps, tw)
        out["return_cusp_replans"] = None if cusps is None else _count_in(cusps, rw)
    tf = r.get("tracking_failure")
    out["transport_tracking_failure"] = None
    if isinstance(tf, dict) and tf.get("phase") == "transport":
        tracking = tf.get("tracking") if isinstance(tf.get("tracking"), dict) else {}
        out["transport_tracking_failure"] = tracking.get("failure") or "unknown"

    de = r.get("delivery_error_m")
    out["delivery_error_m"] = float(de) if isinstance(de, (int, float)) else None
    st = r.get("simulated_time_s")
    out["simulated_time_s"] = float(st) if isinstance(st, (int, float)) and out["success"] else None
    contacts = r.get("forbidden_pocket_contacts")
    out["forbidden_contacts"] = len(contacts) if isinstance(contacts, list) else contacts
    det = r.get("deterministic") if isinstance(r.get("deterministic"), dict) else {}
    out["deterministic_enabled"] = det.get("enabled")
    out["deterministic_broken"] = det.get("broken")
    out["applied"] = {k: args.get(k) for k in APPLIED}
    try:
        out["riib_defaults"] = (run / "riib_defaults.txt").read_text()
    except OSError:
        out["riib_defaults"] = None
    try:
        stats = json.loads((run / "run" / "path_display.json").read_text())
        out["path_display_faults"] = {k: v for k, v in stats.items() if isinstance(v, int) and not isinstance(v, bool)
                                      and k != "event_rows_pending"}
    except (OSError, ValueError):
        out["path_display_faults"] = None
    return out


def _stats(values: list) -> dict:
    v = [float(x) for x in values if x is not None]
    if not v:
        return {"n": 0, "median": None, "p90": None, "max": None}
    return {"n": len(v), "median": float(np.median(v)), "p90": float(np.percentile(v, 90)), "max": max(v)}


def _ratio(rows, key) -> list | str:
    known = [r[key] for r in rows if r.get(key) is not None]
    return [sum(1 for x in known if x), len(known)] if known else "not_comparable"


def _group(rows: list) -> dict:
    ok = [r for r in rows if r["success"]]
    near = [r["near_status"] for r in rows if r["near_status"] is not None]
    tcr = [r["transport_cusp_replans"] for r in rows if r["transport_cusp_replans"] is not None]
    rcr = [r["return_cusp_replans"] for r in rows if r["return_cusp_replans"] is not None]
    faults: Counter = Counter()
    for r in rows:
        faults.update(r.get("path_display_faults") or {})
    return {
        "n": len(rows),
        "success": [len(ok), len(rows)],
        "far_selected": _ratio(rows, "far_selected"),
        "far_pocket_valid": _ratio(rows, "far_pocket_valid"),
        "far_capture_failures": sum(1 for r in rows if r.get("far_capture_failure")),
        "observe_replans": sum(r["observe_replans"] or 0 for r in rows),
        "near_status": dict(Counter(near)) if near else "not_comparable",
        "transport_reached": _ratio(rows, "transport_reached"),
        "transport_tracking_failures": dict(Counter(r["transport_tracking_failure"] for r in rows
                                                    if r["transport_tracking_failure"])),
        "transport_cusp_replans": ({"total": sum(tcr), "runs_with": sum(1 for x in tcr if x)} if tcr else "not_comparable"),
        "return_cusp_replans": ({"total": sum(rcr), "runs_with": sum(1 for x in rcr if x)} if rcr else "not_comparable"),
        "delivery_error_m": _stats([r["delivery_error_m"] for r in ok]),
        "delivery_error_failed_runs": [[r["name"], r["delivery_error_m"]] for r in rows
                                       if not r["success"] and r["delivery_error_m"] is not None],
        "simulated_time_s": _stats([r["simulated_time_s"] for r in ok]),
        "failure_classes": dict(Counter(r["failure_class"] for r in rows if r["failure_class"])),
        "failure_stages": dict(Counter(r["failure_stage"] for r in rows if r["failure_stage"])),
        "forbidden_contacts": sum(r["forbidden_contacts"] or 0 for r in rows if isinstance(r["forbidden_contacts"], int)),
        "deterministic_broken_runs": sum(1 for r in rows if r.get("deterministic_broken")),
        "path_display_faults": dict(faults),
        "missing_fields": dict(Counter(k for r in rows for k in r["missing"])),
    }


def aggregate(rows: list) -> dict:
    kept = [r for r in rows if r["refused"] is None]
    scenes = sorted({r["scene"] for r in kept})
    return {"scenes": {s: _group([r for r in kept if r["scene"] == s]) for s in scenes},
            "all": _group(kept),
            "refused": [[r["name"], r["refused"]] for r in rows if r["refused"] is not None]}


PAIR_KEYS = ("success", "phase", "failure_class", "failure_stage", "failure_reason", "near_status",
             "transport_tracking_failure", "delivery_error_m", "simulated_time_s")


def pair_differences(a_rows: list, b_rows: list) -> dict:
    """Same scene and seed in two sets: the pairs whose outcome (success, end phase, failure class) differs."""
    a = {(r["scene"], r["seed"]): r for r in a_rows if r["refused"] is None}
    b = {(r["scene"], r["seed"]): r for r in b_rows if r["refused"] is None}
    common = sorted(set(a) & set(b))
    changed = []
    for k in common:
        x, y = a[k], b[k]
        if (x["success"], x["phase"], x["failure_class"]) != (y["success"], y["phase"], y["failure_class"]):
            changed.append({"scene": k[0], "seed": k[1], "a": {f: x[f] for f in PAIR_KEYS},
                            "b": {f: y[f] for f in PAIR_KEYS}})
    return {"common": len(common), "changed": changed,
            "only_a": [list(k) for k in sorted(set(a) - set(b))], "only_b": [list(k) for k in sorted(set(b) - set(a))]}


def collect(root: Path, mode: str) -> list:
    rows = []
    for d in sorted(p for p in Path(root).iterdir() if p.is_dir()):
        try:
            parse_name(d.name)
        except ValueError:
            continue
        rows.append(run_fields(d, mode))
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("root", type=Path, help="folder of run folders")
    ap.add_argument("--mode", choices=MODES, required=True)
    ap.add_argument("--compare", type=Path, default=None, help="a second folder of run folders (same scene and seed)")
    ap.add_argument("--compare-mode", choices=MODES, default="prefix")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--fields-out", type=Path, default=None, help="per-run fields, gzip JSON")
    a = ap.parse_args(argv)
    rows = collect(a.root, a.mode)
    out = {"schema": "post_fix_baseline_summary/1", "root": str(a.root), "mode": a.mode, "aggregate": aggregate(rows)}
    if a.compare is not None:
        crows = collect(a.compare, a.compare_mode)
        out["compare"] = {"root": str(a.compare), "mode": a.compare_mode, "aggregate": aggregate(crows),
                          "pairs": pair_differences(rows, crows)}
    a.out.write_text(json.dumps(out, indent=1, sort_keys=True) + "\n")
    if a.fields_out is not None:
        with gzip.open(a.fields_out, "wt") as f:
            json.dump({"mode": a.mode, "rows": rows}, f, sort_keys=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
