"""Does the Isaac truck's true body stay inside the region the teleoperated arc bundle checks?

Teleop bundle margin plan V2/V3 (docs/plans/2026-10-17-teleop-bundle-margin.md). The bundle
(``forklift_core.safety.teleop_drive``) installs fixed-curvature arcs from the pose it is asked at and checks the
footprint along them grown by the permission's envelope; ``BundlePermit`` keeps an install until its arcs change, a
new scan comes in, the direction changes or the truck has driven ``refresh_m``. Every cycle it allows only a speed
that stops inside what the install verified (the middle arcs by their path left, the two end arcs with their
emergency-stop arcs asked again from the current pose). So the obligation of a cycle at time t is: **a stop begun at
t stays inside the region of the install in force at t** -- the install's arcs from its own pose, plus the end
arcs' stop arcs from the pose at t -- over the stopping distance at the speed the check covers.

For recorded runs (``slam_log.npz`` -- true base pose and steering joints at 120 Hz, scan stamps -- and
``result.json``), this tool replays the install lifetimes and, every ``--every`` joint samples while moving,

1. takes the install in force (``--rule new``: ``bundle(measured, output, margin)`` on the grid, reinstalled on a
   new grid range; ``--rule old``: the M-c bundle, exact ends, reinstalled when the ends' rounded grid indices or
   the arc count change, the old ``BundlePermit`` key);
2. anchors it at the **true** rear-axle pose of its install (localisation error is the obstacle layer's
   odometry-age business, not the bundle's);
3. follows the true trajectory from t over the stopping distance at the highest speed of the next ``--speed-ahead-s``
   (the speed the check must cover; a recorded drive goes on, a stop would brake along the same steering), cut
   only at a reversal or where the output curvature leaves the install's arcs (a new input, which design 4.4 leaves
   to the next check);
4. measures how far the true body (hull, or body and blades when reversing empty -- the permission's
   ``hull_forward`` shapes) lies outside the union of the region's footprints at the arcs' 0.02 m samples.

Sampling: a coarse pass (every ``--coarse-every`` true pose, outline every 0.05 m, corners included) measures each
check; one that comes within ``--refine-within-m`` of the envelope is measured again at every true pose and a 5 mm
outline, and the outcome is the fine value's. The largest fine-minus-coarse difference is reported as the sampling
error seen -- an empirical figure, not a bound: the result is "no violation at the points and poses measured",
not a continuous guarantee. The figures restricted to speeds the teleop chain can reach (``--teleop-*-mps``) are
reported beside the whole.

Output curvature: the mission's command (``samples[].curvature_inv_m``, 10 Hz, held) in AUTO; while teleoperated,
the runner's ``teleop_k`` records (``--live-test-permit-compare``, teleop bundle margin plan V3) -- an older live
log has no output curvature and its teleop windows are left out. Ground truth, Isaac only.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))  # a run snapshot's own source, as the runner's tools use it
sys.path.insert(0, str(ROOT))

from tools.curvature_residual import joint_curvature, loaded_at, plant_key, yaw_of  # noqa: E402

from forklift_core.control.drive_permission import StoppingModel, arc_poses  # noqa: E402
from forklift_core.live.spool import load_list, load_slam_log, read_live_log  # noqa: E402
from forklift_core.safety.teleop_drive import KAPPA_STEP, bundle  # noqa: E402

Part = tuple  # (front_m, rear_m, half_width_m, lateral_m, longitudinal_m) in the rear-axle frame


def old_bundle(kappa_from: float, kappa_to: float) -> list[float]:
    """The M-c bundle (teleop_drive before the margin): both ends exact, at most KAPPA_STEP apart."""
    if kappa_from == kappa_to:
        return [float(kappa_from)]
    n = max(1, math.ceil(abs(kappa_to - kappa_from) / KAPPA_STEP))
    return [float(k) for k in np.linspace(kappa_from, kappa_to, n + 1)]


def outline(parts: list[Part], spacing: float = 0.1) -> np.ndarray:
    """Points along every part's edges, corners included (rear-axle frame, x forward)."""
    out = []
    for front, rear, half, lat, lon in parts:
        xs = np.linspace(-rear, front, max(2, math.ceil((front + rear) / spacing) + 1)) + lon
        ys = np.linspace(-half, half, max(2, math.ceil(2 * half / spacing) + 1)) + lat
        out += [np.column_stack((xs, np.full_like(xs, lat + half))), np.column_stack((xs, np.full_like(xs, lat - half))),
                np.column_stack((np.full_like(ys, lon + front), ys)), np.column_stack((np.full_like(ys, lon - rear), ys))]
    return np.unique(np.vstack(out), axis=0)


def placed(poses: np.ndarray, points: np.ndarray) -> np.ndarray:
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    x = poses[:, None, 0] + c[:, None] * points[None, :, 0] - s[:, None] * points[None, :, 1]
    y = poses[:, None, 1] + s[:, None] * points[None, :, 0] + c[:, None] * points[None, :, 1]
    return np.column_stack((x.ravel(), y.ravel()))


def outside_m(points: np.ndarray, poses: np.ndarray, parts: list[Part], chunk: int = 4096) -> np.ndarray:
    """Each point's distance outside the union of every part at every pose (0 inside one)."""
    c, s = np.cos(poses[:, 2]), np.sin(poses[:, 2])
    out = np.empty(len(points))
    for a in range(0, len(points), chunk):
        p = points[a:a + chunk]
        dx = p[:, None, 0] - poses[None, :, 0]
        dy = p[:, None, 1] - poses[None, :, 1]
        u = dx * c[None] + dy * s[None]
        v = -dx * s[None] + dy * c[None]
        best = np.full(len(p), np.inf)
        for front, rear, half, lat, lon in parts:
            w = u - lon
            ou = np.maximum(np.maximum(w - front, -rear - w), 0.0)
            ov = np.maximum(np.abs(v - lat) - half, 0.0)
            best = np.minimum(best, np.hypot(ou, ov).min(axis=1))
        out[a:a + chunk] = best
    return out


def shapes_of(result: dict, offset: float, urdf_root: Path | None = None) -> dict:
    """The permission's checked shapes (obstacle layer ``hull_forward``): loaded -> the loaded outline; empty
    forward -> the hull; empty reversing -> the body and the two blades from the chassis URDF. A relative URDF path
    is the run snapshot's; it is looked for under ``urdf_root`` and used only if its hash is the run's recorded
    ``forklift_urdf_sha256`` (else the hull, and the output says so)."""
    g = result.get("geometry", {})
    u, ld = g["unloaded_footprint"], g["loaded_footprint"]
    hull = [(u["front_m"], u["rear_m"], u["half_width_m"], 0.0, 0.0)]
    loaded = [(ld["front_m"], ld["rear_m"], ld["half_width_m"], 0.0, 0.0)]
    parts, source = hull, "hull (no URDF)"
    urdf = result.get("arguments", {}).get("forklift_urdf")
    try:
        sys.path.insert(0, str(ROOT / "sim" / "isaac"))
        from insertion_geometry import read_fork_blades_m

        urdf = Path(urdf)
        if not urdf.is_absolute() and urdf_root is not None:
            urdf = urdf_root / urdf
        recorded = result.get("forklift_urdf_sha256")
        if not recorded or hashlib.sha256(urdf.read_bytes()).hexdigest() != recorded:
            why = "differs" if recorded else "not recorded"
            return {"hull": hull, "parts": hull, "loaded": loaded, "parts_source": f"hull (URDF hash {why})"}
        body_front = float(g["axle_to_fork_tip_m"]) - float(g["carriage_limit_m"])
        parts = [(body_front, u["rear_m"], u["half_width_m"], 0.0, 0.0)] + [
            ((x1 - x0) / 2, (x1 - x0) / 2, (y1 - y0) / 2, (y0 + y1) / 2, (x0 + x1) / 2 + offset)
            for x0, x1, y0, y1 in read_fork_blades_m(Path(urdf))]
        source = "parts"
    except (ImportError, OSError, KeyError, TypeError, ValueError):
        pass
    return {"hull": hull, "parts": parts, "loaded": loaded, "parts_source": source}


def run_coverage(run: Path, args) -> dict:
    result = json.loads((run / "result.json").read_text())
    log = load_slam_log(run)  # a live run's log is in its record spool (persistent console plan D8)
    a = result.get("arguments", {})
    geom = a.get("drive_geometry") or {}
    wheelbase, track = float(geom.get("wheelbase_m", 0.66)), float(geom.get("track_m", 0.53))
    offset = abs(float(a.get("rear_axle_offset_m", 0.34)))
    shapes = shapes_of(result, offset, args.urdf_root)
    stamps = np.asarray(log["joint_stamps_s"], float)
    n = len(stamps)
    base = np.asarray(log["base_pose_world"], float)
    yaw = np.unwrap(yaw_of(base[:, 3:7]))
    rear = np.column_stack((base[:, 0] - offset * np.cos(yaw), base[:, 1] - offset * np.sin(yaw), yaw))
    kj = joint_curvature(np.asarray(log["steering_rad"], float), wheelbase, track)
    loaded = loaded_at(result, stamps)
    dt = float(np.median(np.diff(stamps)))
    step = np.hypot(np.diff(rear[:, 0]), np.diff(rear[:, 1]))
    travel = np.concatenate(([0.0], np.cumsum(step)))
    dx, dy = np.diff(rear[:, 0]), np.diff(rear[:, 1])
    vlon = np.concatenate(((dx * np.cos(rear[:-1, 2]) + dy * np.sin(rear[:-1, 2])) / np.maximum(np.diff(stamps), 1e-9),
                           [0.0]))
    speed = np.abs(vlon)
    direction = np.where(speed >= args.min_speed_mps, np.sign(vlon), 0).astype(int)
    # A new grid snapshot per scan the obstacle layer took in; a silenced scan is recorded but never reaches it.
    scans = np.asarray(log["scan_stamps_s"], float) if "scan_stamps_s" in log else np.zeros(0)
    if "silenced_scan_stamps_s" in log and len(log["silenced_scan_stamps_s"]):
        scans = scans[~np.isin(scans, np.asarray(log["silenced_scan_stamps_s"], float))]
    silence = a.get("live_test_scan_silence")  # "a:b" sim s: recorded raw, kept from the layer (not in the list)
    if silence:
        lo, hi = (float(v) for v in str(silence).split(":"))
        scans = scans[(scans < lo) | (scans >= hi)]
    scan_index = np.searchsorted(scans, stamps, side="right")

    # The output curvature at every joint sample, and whether that sample was teleoperated.
    # A live run's samples are in its record spool (plan D8); an empty list here would fall back to the joints.
    spooled = (result.get("spool") or {}).get("root") is not None and "state.samples" in result.get("spool_registered", ())
    samples = load_list(run / result["spool"]["root"], "state.samples") if spooled else (result.get("samples") or [])
    ts = np.array([s["time_s"] for s in samples], float)
    kc = np.array([s.get("curvature_inv_m", 0.0) for s in samples], float)
    k_out = kc[np.clip(np.searchsorted(ts, stamps, side="right") - 1, 0, len(ts) - 1)] if len(ts) else kj.copy()
    mode = np.zeros(n, dtype="<U9")
    mode[:] = "auto"
    if (run / "live_log.jsonl").exists():
        entries = read_live_log(run)  # every rolled part, oldest first; refused if one rolled off (plan D8)
        tele = [e for e in entries if "teleop_k" in e]
        cmp_ = [e for e in entries if "permit_cmp" in e]
        if tele:
            tt = np.array([r["teleop_k"] for r in tele], float)
            ko = np.array([r["output_kappa"] for r in tele], float)
            at = np.clip(np.searchsorted(tt, stamps, side="right") - 1, 0, len(tt) - 1)
            near = (stamps >= tt[0] - 1e-9) & (np.abs(stamps - tt[at]) <= 2.5 * dt)
            k_out = np.where(near, ko[at], k_out)
            mode[near] = "teleop"
            # A forbidden drive asks no arcs (teleop_drive): no install, no check -- its stop is the chain's brake
            # to the 0 cap, with nothing checked to stay in. Older records lack the field: they always asked.
            asked = np.array([r.get("permit_asked", True) for r in tele], bool)
            mode[near & ~asked[at]] = "forbidden"
        elif cmp_:  # a teleoperated run recorded before teleop_k: its output curvature is not known
            tl = np.array([c["permit_cmp"] for c in cmp_], float)
            at = np.searchsorted(tl, stamps)
            gap = np.minimum(np.abs(tl[np.clip(at, 0, len(tl) - 1)] - stamps), np.abs(tl[np.clip(at - 1, 0, len(tl) - 1)] - stamps))
            mode[gap <= 0.15] = "unknown"

    def arcs_at(i: int) -> list[float]:
        if args.rule == "old":
            return old_bundle(float(kj[i]), float(k_out[i]))
        return bundle(float(kj[i]), float(k_out[i]), args.margin_inv_m)

    def key_of(arcs: list[float]) -> tuple:
        return round(arcs[0] / KAPPA_STEP), round(arcs[-1] / KAPPA_STEP), len(arcs)

    # Install lifetimes as BundlePermit keeps them: one cache per direction, both advanced by every distance
    # driven; the moving direction's is installed again on a new key, a new snapshot or past the refresh distance.
    # (Asked while standing it is too, in the live runner; a standing truck has no stop to keep, so those asks
    # are not replayed -- the next moving sample installs if anything changed.)
    install_of = np.full(n, -1)
    installs: list[dict] = []
    cache: dict = {}
    for i in range(n):
        if direction[i] == 0 or mode[i] == "forbidden":
            continue
        arcs = arcs_at(i)
        cur = cache.get(int(direction[i]))
        if (cur is None or cur["key"] != key_of(arcs) or cur["scan"] != scan_index[i]
                or travel[i] - travel[cur["i0"]] > args.refresh_m):
            cur = {"i0": i, "arcs": arcs, "key": key_of(arcs), "dir": int(direction[i]), "scan": int(scan_index[i]),
                   "index": len(installs)}
            installs.append(cur)
            cache[int(direction[i])] = cur
        install_of[i] = cur["index"]

    stopping = StoppingModel(args.latency_s, args.decel_mps2, args.stop_margin_m)
    ahead = max(1, int(round(args.speed_ahead_s / dt)))
    refined = {"count": 0, "max_increase_m": 0.0}
    stats = {m: {"checks": 0, "ok": 0, "violation": 0, "worst_m": 0.0, "worst": None, "teleop_range_checks": 0,
                 "teleop_range_violation": 0, "teleop_range_worst_m": 0.0}
             for m in ("auto", "teleop", "unknown")}
    cuts = {"input": 0, "reversal": 0, "record_end": 0}
    forbidden = 0  # cycles skipped because the drive was forbidden (no arcs asked)
    arcs_total = 0
    for t in range(0, n - 1, args.every):
        if mode[t] == "forbidden":
            forbidden += 1
            continue
        if direction[t] == 0 or install_of[t] < 0:
            continue
        inst = installs[install_of[t]]
        arcs, d, i0 = inst["arcs"], inst["dir"], inst["i0"]
        stop = stopping.distance_m(float(speed[t:t + ahead].max()))
        end = int(np.searchsorted(travel, travel[t] + stop))
        if end >= n:
            end = n - 1
            cuts["record_end"] += 1
        stop_at = end
        for m in range(t + 1, end + 1):
            if direction[m] == -d:
                stop_at, why = m - 1, "reversal"
                break
            if not (min(arcs) - 1e-9 <= k_out[m] <= max(arcs) + 1e-9):  # the old bundle may run downwards
                stop_at, why = m - 1, "input"
                break
        else:
            why = None
        if why:
            cuts[why] += 1
        if stop_at <= t:
            continue
        shape = shapes["loaded"] if loaded[t] else (shapes["parts"] if d < 0 else shapes["hull"])
        reach = travel[stop_at] - travel[i0] + 0.2  # the arcs' length past where the truth can be
        region = [arc_poses(tuple(rear[i0]), k, d, reach, args.arc_step_m)[0] for k in arcs]
        now_arcs = arcs_at(t)  # BundlePermit asks the end stop arcs at the curvatures passed now (Codex r3 P1)
        region += [arc_poses(tuple(rear[t]), k, d, stop + args.arc_step_m, args.arc_step_m)[0]
                   for k in (now_arcs[0], now_arcs[-1])]
        region = np.vstack(region)
        arcs_total += len(arcs)

        def measure(every: int, spacing: float) -> float:
            idx = np.unique(np.r_[np.arange(t, stop_at + 1, every), stop_at])
            return float(outside_m(placed(rear[idx], outline(shape, spacing)), region, shape).max())

        prot = measure(args.coarse_every, 0.05)
        if prot > args.envelope_m - args.refine_within_m:
            fine = measure(1, 0.005)
            refined["count"] += 1
            refined["max_increase_m"] = max(refined["max_increase_m"], fine - prot)
            prot = fine
        outcome = "violation" if prot > args.envelope_m else "ok"
        s = stats[mode[t]]
        s["checks"] += 1
        s[outcome] += 1
        cap = (args.teleop_loaded_mps if loaded[t] else
               (args.teleop_forward_mps if d > 0 else args.teleop_reverse_mps))
        if speed[t:t + ahead].max() <= cap + 0.02:  # a speed the teleop chain can reach
            s["teleop_range_checks"] += 1
            s["teleop_range_violation"] += outcome == "violation"
            s["teleop_range_worst_m"] = max(s["teleop_range_worst_m"], prot)
        if prot > s["worst_m"]:
            s["worst_m"] = prot
            s["worst"] = {"t": float(stamps[t]), "direction": d, "loaded": bool(loaded[t]), "speed": float(speed[t]),
                          "k_joint": float(kj[t]), "k_out": float(k_out[t]), "arcs": arcs,
                          "install_age_m": float(travel[t] - travel[i0]), "stop_m": stop}
    checks = sum(s["checks"] for s in stats.values())
    return {"run": str(run), "plant": plant_key(result), "parts_source": shapes["parts_source"], "modes": stats,
            "cuts": cuts, "forbidden_cycles": forbidden, "installs": len(installs), "arcs_mean": arcs_total / max(1, checks), "refined": refined}


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("runs", nargs="+", type=Path)
    p.add_argument("--rule", choices=("old", "new"), default="new")
    p.add_argument("--margin-inv-m", type=float, default=0.04, help="teleop_drive's TeleopLimits default")
    p.add_argument("--refresh-m", type=float, default=0.25, help="BundlePermit's reinstall distance")
    p.add_argument("--envelope-m", type=float, default=0.01, help="the permission's envelope offset")
    p.add_argument("--latency-s", type=float, default=0.15)
    p.add_argument("--decel-mps2", type=float, default=1.5)
    p.add_argument("--stop-margin-m", type=float, default=0.05)
    p.add_argument("--arc-step-m", type=float, default=0.02, help="the permission's sample spacing")
    p.add_argument("--every", type=int, default=12, help="joint samples between checked cycles (120 Hz)")
    p.add_argument("--coarse-every", type=int, default=2, help="true poses between the coarse pass's samples")
    p.add_argument("--refine-within-m", type=float, default=0.004, help="measure finely when this near the envelope")
    p.add_argument("--speed-ahead-s", type=float, default=0.3, help="the check covers the top speed this far ahead")
    p.add_argument("--min-speed-mps", type=float, default=0.02)
    p.add_argument("--teleop-forward-mps", type=float, default=0.5, help="TeleopLimits, for the in-range figures")
    p.add_argument("--teleop-reverse-mps", type=float, default=0.25)
    p.add_argument("--teleop-loaded-mps", type=float, default=0.30)
    p.add_argument("--urdf-root", type=Path, default=ROOT, help="where a run's relative chassis URDF path is looked for")
    args = p.parse_args()
    runs, total = [], {}
    for run in args.runs:
        try:
            r = run_coverage(run, args)
        except (OSError, KeyError, ValueError) as error:
            runs.append({"run": str(run), "skipped": repr(error)})
            continue
        runs.append(r)
        for m, s in r["modes"].items():
            t = total.setdefault(m, {"checks": 0, "ok": 0, "violation": 0, "worst_m": 0.0, "worst_run": None,
                                     "teleop_range_checks": 0, "teleop_range_violation": 0,
                                     "teleop_range_worst_m": 0.0})
            t["teleop_range_worst_m"] = max(t["teleop_range_worst_m"], s["teleop_range_worst_m"])
            for k in ("checks", "ok", "violation", "teleop_range_checks", "teleop_range_violation"):
                t[k] += s[k]
            if s["worst_m"] > t["worst_m"]:
                t["worst_m"], t["worst_run"] = s["worst_m"], {"run": r["run"], **(s["worst"] or {})}
    print(json.dumps({"settings": {k: v for k, v in vars(args).items() if k != "runs"}, "total": total,
                      "runs": runs}, indent=1, default=str))


if __name__ == "__main__":
    main()
