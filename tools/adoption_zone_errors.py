"""Test 0 of docs/plans/2026-10-28-adoption-zone-check.md: the pallet-centre estimate error per adoption stage, per
world axis, from recorded Isaac runs -- the margin the adoption check needs.

Each valid observation attempt's estimate is placed again the way observe_succeeded places it (the front midpoint half
a pallet depth in, the capture's control base = its control rear + |rear_axle_offset_m| ahead) and compared with the
pallet's truth: the mission's own pallet at its last accepted capture (``handoff.pallet_position_m``; the pallet stands
still before pickup), else ``initial_pallet_m``. A batch run is one record (its result.json); a live run's missions are
records of their own (``missions/*/records.json``, with the run's constants). A record is undetermined, not used, when
its recorded estimates (``perception_pickup_estimate_m``, the near capture's ``far_estimate_m``, an adopted second
capture's ``target``) are not reproduced, or a valid attempt has no capture pose.

    python -m tools.adoption_zone_errors <dir> [<dir> ...] --out errors.json [--manifest-out manifest.json]
    python -m tools.adoption_zone_errors --manifest-in manifest.json --out errors.json   # the same sample again
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

STEP_M = 0.05  # the margin is the largest per-axis error rounded up to this step
_TOL_M = 1e-6


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def site_of(attempt: dict, *, offset_m: float, depth_m: float) -> tuple[float, float]:
    obs = attempt["pocket_observation"]
    left, right, yaw = obs["left"]["center_m"], obs["right"]["center_m"], float(obs["insertion_yaw_rad"])
    bx = (left[0] + right[0]) / 2 + depth_m / 2 * math.cos(yaw)
    by = (left[1] + right[1]) / 2 + depth_m / 2 * math.sin(yaw)
    rx, ry, ryaw = (float(v) for v in attempt["capture_control"]["control_rear"])
    c, s = math.cos(ryaw), math.sin(ryaw)
    base = (rx + abs(offset_m) * c, ry + abs(offset_m) * s)
    return base[0] + c * bx - s * by, base[1] + s * bx + c * by


def _close(a, b) -> bool:
    return abs(a[0] - b[0]) <= _TOL_M and abs(a[1] - b[1]) <= _TOL_M


def _constants(run: dict):
    offset = run.get("rear_axle_offset_m")
    if offset is None:
        offset = (run.get("arguments") or {}).get("rear_axle_offset_m")
    return offset, (run.get("geometry") or {}).get("pallet_depth_m"), run.get("initial_pallet_m")


def record_errors(rec: dict, run: dict) -> tuple[list, str | None]:
    """[(stage, dx, dy)] for one record's valid attempts (``rec``: a batch result or a live mission's state; ``run``:
    the run's result for its constants), or ([], why) when it cannot be verified."""
    offset, depth, initial = _constants(run)
    truth = (rec.get("handoff") or {}).get("pallet_position_m") or initial
    if offset is None or depth is None or truth is None:
        return [], "no rear_axle_offset_m, pallet depth or pallet truth"
    near = rec.get("near_capture") or {}
    second = near.get("second") or {}
    sites = {}
    for i, a in enumerate(rec.get("observation_attempts") or [], start=1):
        if (a.get("pocket_observation") or {}).get("status") != "valid":
            continue
        if (a.get("capture_control") or {}).get("control_rear") is None:
            return [], f"valid attempt {i} has no capture_control.control_rear"
        sites[i] = site_of(a, offset_m=float(offset), depth_m=float(depth))
    checks = []
    est = (rec.get("perception") or {}).get("perception_pickup_estimate_m")
    if est is not None:
        checks.append(("the last estimate", (est["x_m"], est["y_m"]), None))
    far_est = near.get("far_estimate_m")
    if far_est is not None and near.get("far_attempt") is not None:
        checks.append(("the far estimate", (far_est["x_m"], far_est["y_m"]), int(near["far_attempt"])))
    if second.get("status") == "adopted" and second.get("target") is not None:
        checks.append(("the second capture's target", tuple(second["target"][:2]), second.get("capture_attempt")))
    if not checks:
        return [], "no recorded estimate to verify the reconstruction"
    for what, xy, number in checks:
        candidates = list(sites.values()) if number is None else [sites[number]] if number in sites else []
        if not any(_close(xy, c) for c in candidates):
            return [], f"no attempt reproduces {what} {xy}"
    rows = []
    for i, (x, y) in sites.items():
        stage = ("second_near" if i == second.get("capture_attempt") else "near" if i == near.get("near_attempt")
                 else "far" if i == near.get("far_attempt", 1) else "other")
        rows.append((stage, x - float(truth[0]), y - float(truth[1])))
    return rows, None


def _quantile(values, p):
    v = sorted(values)
    return v[min(len(v) - 1, math.ceil(p * len(v)) - 1)] if v else None


def summarize(rows: list) -> dict:
    stages: dict = {}
    for stage, dx, dy in rows:
        stages.setdefault(stage, []).append(max(abs(dx), abs(dy)))
    out = {"stages": {s: {"n": len(v), "max_abs_m": max(v), "p99_abs_m": _quantile(v, 0.99)}
                      for s, v in sorted(stages.items())}}
    overall = max((max(v) for v in stages.values()), default=None)
    out["overall_max_abs_m"] = overall
    out["margin_m"] = None if overall is None else math.ceil(round(overall / STEP_M, 9)) * STEP_M
    return out


def _records_of(result_path: Path):
    """(path, record, run) for a run: a live run's missions (missions/*/records.json), and its result.json unless that
    repeats one of them (it holds the last mission's state, whose own records may not have been written), else the
    batch run's result.json."""
    run = json.loads(result_path.read_text())
    if not isinstance(run, dict):
        return
    seen = []
    for m in sorted((result_path.parent / "missions").glob("*/records.json")):
        state = (json.loads(m.read_text()) or {}).get("state") or {}
        if "observation_attempts" in state:
            seen.append(state["observation_attempts"])
            yield m, state, run
    if "observation_attempts" in run and run["observation_attempts"] not in seen:
        yield result_path, run, run


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("dirs", nargs="*", type=Path)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--manifest-out", type=Path)
    ap.add_argument("--manifest-in", type=Path, help="the exact records and hashes of a fixed sample")
    args = ap.parse_args(argv)
    listed = None
    if args.manifest_in is not None:
        listed = json.loads(args.manifest_in.read_text())
        for e in listed:
            p = Path(e["path"])
            if not p.exists() or sha256(p) != e["sha256"]:
                print(f"{p}: missing or changed since the manifest -- not the fixed sample", file=sys.stderr)
                return 2
        results = sorted({Path(e["run"]) for e in listed})
    else:
        results = sorted(p for d in args.dirs for p in d.glob("**/result.json") if p.parent.name == "run")
    rows, undetermined, manifest = [], [], []
    wanted = None if listed is None else {e["path"] for e in listed}
    for result_path in results:
        try:
            records = list(_records_of(result_path))
        except ValueError:
            continue
        for path, rec, run in records:
            if wanted is not None and str(path) not in wanted:
                continue
            manifest.append({"path": str(path), "run": str(result_path), "sha256": sha256(path)})
            got, problem = record_errors(rec, run)
            if problem is not None:
                undetermined.append({"path": str(path), "problem": problem})
            rows += got
    out = {"records": len(manifest), "undetermined": len(undetermined), "undetermined_records": undetermined,
           **summarize(rows)}
    args.out.write_text(json.dumps(out, indent=1))
    if args.manifest_out is not None:
        args.manifest_out.write_text(json.dumps(manifest, indent=1))
    print(json.dumps({k: out[k] for k in ("records", "undetermined", "overall_max_abs_m", "margin_m")}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
