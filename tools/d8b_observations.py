"""Plan D8b ③ observation errors and ⑤ CPU-Isaac correspondence (no Isaac).

docs/plans/2026-10-04-lidar-obstacle-map.md, "D8b 구현 설계" (7판) ③ and ⑤, on the runs
tools/d8b_calibration.py has calibrated (its L is an input here).

③ Every timed read of the approach and the insertion, in time order. A read whose depth
did not come back is a loss row. The rest, in the D8a mount study's input contract
(recorded float32 depth rounded to 1 mm, no noise; that study's near-field detector
settings), are observed twice: the front pocket detector (no pose prior) and the roof
tracker with the measurement chain's prior -- the last accepted observation carried to
this frame by the control relative transform between the two aligned times (s - L),
starting from the near-capture estimate. Accepted = valid, (roof) reported position
sigma <= 0.020 m, and within 0.05 m / 0.05 rad of the carried prior: a fixed coarse gate,
wider than the 0.030 association tolerance it must not presuppose (plan D8b 2nd P2).
Front mode feeds the chain until three frames in a row have an accepted front and an
accepted roof that agree within RoofHandoff's tolerances; roof mode after (any other
row resets the count). The handoff gate g is not applied (the CPU replay fixes it).
Every observation is kept with its error against the 6-DoF truth at s - L; the
reported sigma only filters. A truth-prior roof run (③b) gives the detector's own
error, never a bound. Provenance: the front width is the observed gap; the roof's
width, height and spacing come from the pallet model.

⑤ The same reads rendered on the CPU from the recorded truth poses (pallet, floor and the
truck's carriage boxes; 1 mm rounding; optical depth < 0.28 m invalid, as the clipping
plane): depth differences over the pallet region less a 2 px silhouette band, valid /
invalid disagreement over the whole pallet region, and the whole chain replayed on the
CPU frames, compared read by read with the recorded one against the plan's tolerances.

    python tools/d8b_observations.py --calibration <dir>/d8b_calibration.json --output <dir>
    python tools/d8b_observations.py ... --runs <run dir> ...   # one job per run
    python tools/d8b_observations.py ... --collect             # the summary of all runs
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import d8b_calibration as CAL  # noqa: E402
from tools import scene_rig  # noqa: E402

SIGMA_MAX_M = 0.020
STEP_M, STEP_RAD = 0.05, 0.05
HANDOFF = dict(position=0.025, yaw=0.03, size=0.02, frames=3)
CLIP_M = 0.28
BIN_M = 0.1
SILHOUETTE_PX = 2
STANDING_MPS = 0.005
NOISE_SIGMA_A, NOISE_K = 0.0036, 3.0  # the D5 synthetic model, sigma(z) = a z^2 cut at k sigma
# ⑤ tolerances, fixed before the runs (plan D8b 7판 ⑤).
TOL = dict(depth_p95_m=0.005, mismatch=0.02, agreement=0.95, disagree_run=1, geometry_m=0.005, yaw_rad=0.01)
LEVER_M = 0.6  # front reference point to the pocket end: the yaw difference's lever


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def run_name(key: str) -> str:
    """A file name unique per run directory (Codex D8b impl P1: parents can repeat)."""
    return f"{Path(key).parent.name}_{hashlib.sha1(key.encode()).hexdigest()[:8]}"


# Frames ----------------------------------------------------------------------------------
def scene_input(run: CAL.Run, depth_m: np.ndarray, lift_m: float, rng=None):
    """The depth as the detector reads it: the D8a contract (1 mm, no noise); with ``rng``
    the reference variant -- sigma(z) = 0.0036 z^2 noise cut at 3 sigma, then 1 mm."""
    from forklift_core.perception.scene_dataset import SceneInput

    k = run.intrinsics
    spec = dataclasses.replace(scene_rig.intrinsics(), fx=k.fx, fy=k.fy, cx=k.cx, cy=k.cy)
    transform = scene_rig.RigidTransform(
        scene_rig.OPTICAL_FRAME_ID, scene_rig.BASE_FRAME_ID, run.truth.mount.rotation,
        run.truth.mount.translation_m + np.array([0.0, 0.0, lift_m]),
    )
    depth = np.asarray(depth_m, dtype=float).copy()
    finite = np.isfinite(depth)
    if rng is not None:
        sigma = NOISE_SIGMA_A * depth[finite] ** 2
        depth[finite] += np.clip(rng.normal(0.0, 1.0, int(finite.sum())), -NOISE_K, NOISE_K) * sigma
    depth = np.where(finite, np.round(depth * 1000.0) / 1000.0, np.nan)
    return SceneInput(rgb=np.zeros((k.height, k.width, 3), dtype=np.uint8), depth_m=depth, intrinsics=spec,
                      base_from_optical=transform, stamp_ns=0, clock_domain="ros_sim",
                      source_provenance="synthetic")


def truth_in_base(run: CAL.Run, t: float, geometry) -> dict:
    """The pockets in base_link at time t from the 6-DoF truth: front-plane centres and
    the four side walls as points (left = the +y pocket of base), the insertion yaw
    (the pallet axis into the pallet) and the opening size."""
    r_wb, p_wb = CAL.pose_at(run.truth.t, run.truth.base, t)
    r_wp, p_wp = run.truth.pallet_pose(t)
    r_bp, p_bp = r_wb.T @ r_wp, r_wb.T @ (p_wp - p_wb)
    face_sign = -1.0 if (r_bp.T @ (-p_bp))[0] < 0 else 1.0  # the face toward the base origin
    half, z = geometry.overall_depth_m / 2, geometry.opening_centre_height_m
    offset, width = geometry.opening_centre_offset_m, geometry.opening_width_m

    def point(y):
        return p_bp + r_bp @ np.array([face_sign * half, y, z])

    pockets = []
    for y in (offset, -offset):
        pockets.append({"centre": point(y), "walls": (point(y + width / 2), point(y - width / 2))})
    pockets.sort(key=lambda p: -p["centre"][1])
    out = {}
    for side, pocket in zip(("left", "right"), pockets):
        a, b = pocket["walls"]
        outer, inner = (a, b) if (a[1] > b[1]) == (side == "left") else (b, a)
        out[side] = pocket["centre"]
        out[f"{side}_walls"] = {"outer": outer, "inner": inner}
    axis = r_bp @ np.array([-face_sign, 0.0, 0.0])
    out.update(yaw=math.atan2(axis[1], axis[0]), width=width, height=geometry.block_height_m)
    return out


def pocket_errors(observation, truth: dict) -> dict | None:
    """Per pocket: centre (3-D; lateral, along and vertical in the truth's axes), width,
    height and the two walls the blades face, as lateral offsets of the observed wall
    point (observed yaw) from the true wall point (full rigid pose); and the yaw."""
    if observation is None or observation.status != "valid":
        return None
    yaw = truth["yaw"]
    along = np.array([math.cos(yaw), math.sin(yaw), 0.0])
    lateral = np.array([-math.sin(yaw), math.cos(yaw), 0.0])
    o_yaw = observation.insertion_yaw_rad
    o_lateral = np.array([-math.sin(o_yaw), math.cos(o_yaw), 0.0])
    out = {"yaw_rad": wrap(o_yaw - yaw)}
    for side in ("left", "right"):
        p = getattr(observation, side)
        centre = np.asarray(p.center_m)
        c = centre - truth[side]
        sign = 1.0 if side == "left" else -1.0  # the outer wall's side along +lateral
        walls = {}
        for name, s in (("outer", sign), ("inner", -sign)):
            observed = centre + s * p.width_m / 2 * o_lateral
            walls[name] = float(np.dot(observed - truth[f"{side}_walls"][name], lateral))
        out[side] = {"centre_m": float(np.linalg.norm(c)), "lateral_m": float(np.dot(c, lateral)),
                     "along_m": float(np.dot(c, along)), "vertical_m": float(c[2]),
                     "width_m": p.width_m - truth["width"], "height_m": p.height_m - truth["height"],
                     "wall_m": walls}
    return out


def midpoint(observation) -> tuple[np.ndarray, float]:
    return (np.asarray(observation.left.center_m) + observation.right.center_m) / 2, observation.insertion_yaw_rad


def control_base(run: CAL.Run, t: float) -> np.ndarray:
    """The control (SLAM) base_link pose (x, y, yaw) at t, from the recorded rear axle."""
    c = run.control
    x, y = np.interp(t, c[:, 0], c[:, 1]), np.interp(t, c[:, 0], c[:, 2])
    yaw = float(np.interp(t, c[:, 0], np.unwrap(c[:, 3])))
    offset = abs(run.meta["rear_axle_offset_m"])
    return np.array([x + offset * math.cos(yaw), y + offset * math.sin(yaw), yaw])


def control_covers(run: CAL.Run, t: float) -> bool:
    """t is a control row or lies between two rows at most 1.5 ticks apart (no
    interpolation across a gap -- Codex D8b impl P2)."""
    times = run.control[:, 0]
    k = int(np.searchsorted(times, t))
    if k < len(times) and abs(times[k] - t) <= 1e-9:
        return True
    return 0 < k < len(times) and times[k] - times[k - 1] <= 1.5 * CAL.TICK_S


def carry(point: np.ndarray, yaw: float, base_then: np.ndarray, base_now: np.ndarray) -> tuple[np.ndarray, float]:
    """A base-frame point and yaw at one time, moved by the control relative transform."""
    c, s = math.cos(base_then[2]), math.sin(base_then[2])
    world = np.array([base_then[0] + c * point[0] - s * point[1], base_then[1] + s * point[0] + c * point[1]])
    c, s = math.cos(base_now[2]), math.sin(base_now[2])
    d = world - base_now[:2]
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], point[2]]), wrap(yaw + base_then[2] - base_now[2])


def near_capture_prior(run: CAL.Run, at: float, geometry) -> tuple[np.ndarray, float] | None:
    """The chain's start: the near-capture estimate (control-frame pallet centre and
    insertion yaw) as a base-frame front midpoint at time ``at``."""
    estimate = (run.meta.get("near_capture") or {}).get("near_estimate_m")
    if estimate is None:
        return None
    yaw = estimate["yaw_rad"]
    half = geometry.overall_depth_m / 2
    face = np.array([estimate["x_m"] - half * math.cos(yaw), estimate["y_m"] - half * math.sin(yaw)])
    base = control_base(run, at)
    c, s = math.cos(base[2]), math.sin(base[2])
    d = face - base[:2]
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], geometry.opening_centre_height_m]), wrap(yaw - base[2])


def agree(front, roof) -> bool:
    pairs = ((front.left, roof.left), (front.right, roof.right))
    position = max(math.dist(a.center_m, b.center_m) for a, b in pairs)
    size = max(max(abs(a.width_m - b.width_m), abs(a.height_m - b.height_m)) for a, b in pairs)
    return (position <= HANDOFF["position"] and abs(wrap(front.insertion_yaw_rad - roof.insertion_yaw_rad))
            <= HANDOFF["yaw"] and size <= HANDOFF["size"])


class Travel:
    """Truth distance travelled by base_link and its speed toward the face, by time."""

    def __init__(self, run: CAL.Run, model: CAL.PalletModel):
        self.t = run.truth.t
        self.cum = np.r_[0.0, np.cumsum(np.hypot(*np.diff(run.truth.base[:, :2], axis=0).T))]
        self.run, self.model = run, model

    def at(self, t: float) -> float:
        return float(np.interp(t, self.t, self.cum))

    def speed(self, t: float) -> float:
        k = int(np.clip(np.searchsorted(self.t, t), 1, len(self.t) - 1))
        t0, t1 = self.t[k - 1], self.t[k]
        d0 = CAL.face_distance(self.run.truth, t0, self.model)
        d1 = CAL.face_distance(self.run.truth, t1, self.model)
        return (d0 - d1) / max(t1 - t0, 1e-12)


# ③ ---------------------------------------------------------------------------------------
class Handoff:
    """Front mode until HANDOFF["frames"] reads in a row each have an accepted front and an
    accepted roof that agree; roof mode after, for good. Any read that is not such a pair
    -- refused, lost, without a valid time or in a control gap -- resets the run."""

    def __init__(self):
        self.mode, self.agreeing = "front", 0

    def reset(self) -> None:
        self.agreeing = 0

    def observe(self, both_accepted_and_agree: bool) -> str:
        self.agreeing = self.agreeing + 1 if both_accepted_and_agree else 0
        if self.mode == "front" and self.agreeing >= HANDOFF["frames"]:
            self.mode = "roof"
        return self.mode


def observe_run(run: CAL.Run, l_s: float, geometry, prior, front_params, model: CAL.PalletModel,
                depth_of=None, noise_seed: int | None = None) -> list[dict]:
    """③ for one run, every timed approach / insertion read in time order. ``depth_of``
    replaces the recorded depth (⑤'s CPU replay); ``noise_seed`` gives the noisy variant."""
    from forklift_core.perception.pocket_detector import detect_pockets
    from forklift_core.perception.roof_tracking import track_roof

    span = run.phase_span()
    travel = Travel(run, model)
    rng = None if noise_seed is None else np.random.default_rng(noise_seed)
    rows = []
    chain = None  # (base-frame midpoint, yaw, aligned time) of the last accepted observation
    handoff = Handoff()
    for read in run.reads:
        if read["phase"] not in ("approach", "insert"):
            continue
        if read.get("time_label", read["result"]) != "ok" or read["stamp_s"] is None:
            # No usable time: it cannot be placed, but it breaks a run of agreeing
            # frames all the same (Codex D8b impl re-review P1).
            handoff.reset()
            rows.append({"index": read["index"], "skipped": "no_valid_time", "phase": read["phase"]})
            continue
        s = read["stamp_s"]
        phase = run.truth.phase_at(s)
        if phase not in ("approach", "insert"):
            continue
        at = s - l_s
        row = {"index": read["index"], "stamp_s": s, "aligned_s": at, "phase": phase}
        if span is None or at < span[0] - 1e-9:
            rows.append({**row, "skipped": "before_anchor"})  # shows the capture itself
            continue
        if not control_covers(run, at) or at < run.truth.t[0]:
            handoff.reset()
            rows.append({**row, "skipped": "control_gap"})
            continue
        speed = travel.speed(at)
        row.update(mode=handoff.mode, distance_m=CAL.face_distance(run.truth, at, model), travelled_m=travel.at(at),
                   speed_mps=speed, standing=abs(speed) < STANDING_MPS)
        if not CAL.usable(read):
            # Lost in the recording, lost in the CPU replay too: both chains see the same
            # schedule (re-review P1).
            handoff.reset()  # a lost read breaks the handoff run (Codex D8b impl P1)
            rows.append({**row, "loss": read["result"], "front": None, "roof": None, "roof_truth_prior": None,
                         "handoff_agree": False, "chain_updated": False})
            continue
        depth = run.depth(read) if depth_of is None else depth_of(read)
        scene = scene_input(run, depth, read["lift_m"], rng)
        truth = truth_in_base(run, at, geometry)
        base_now = control_base(run, at)
        if chain is None:
            start = near_capture_prior(run, at, geometry)
            if start is not None:
                chain = (start[0], start[1], at)
        carried = None if chain is None else carry(chain[0], chain[1], control_base(run, chain[2]), base_now)
        front = detect_pockets(scene, prior, front_params).observation
        roof = None if carried is None else track_roof(scene, geometry, carried[0], carried[1]).observation
        truth_mid = (truth["left"] + truth["right"]) / 2
        oracle = track_roof(scene, geometry, truth_mid, truth["yaw"]).observation

        def accept(observation, is_roof: bool) -> str | None:
            """None when accepted, else what refused it."""
            if carried is None:
                return "no_prior"
            if observation is None or observation.status != "valid":
                return "invalid"
            if is_roof and (observation.position_sigma_m is None or observation.position_sigma_m > SIGMA_MAX_M):
                return "sigma"
            mid, yaw = midpoint(observation)
            if float(np.linalg.norm(mid[:2] - carried[0][:2])) > STEP_M or abs(wrap(yaw - carried[1])) > STEP_RAD:
                return "step"
            return None

        front_refused, roof_refused = accept(front, False), accept(roof, True)
        both = (front_refused is None and roof_refused is None and agree(front, roof))
        mode = handoff.observe(both)
        used, refused = (roof, roof_refused) if mode == "roof" else (front, front_refused)
        if refused is None:
            mid, yaw = midpoint(used)
            chain = (mid, yaw, at)

        def record(observation, refused_by):
            if observation is None:
                return None
            return {"status": observation.status, "reason": observation.reason, "refused": refused_by,
                    "sigma_m": observation.position_sigma_m, "errors": pocket_errors(observation, truth)}

        rows.append({**row, "mode": mode, "front": record(front, front_refused), "roof": record(roof, roof_refused),
                     "roof_truth_prior": record(oracle, None), "handoff_agree": both,
                     "chain_updated": refused is None})
    return rows


def loss_intervals(rows: list[dict], bounds: tuple | None = None) -> list[tuple]:
    """Stretches with no chain update, each (t1, t2, travelled1, travelled2): from the start
    to the first update, between updates, and from the last update to the end. ``bounds``
    = (anchor t, end t, travelled at the anchor, travelled at the end): the near-capture
    anchor and the insertion's end, not the first and last rows (re-review P1). Without
    it the first and last rows stand in (tests only)."""
    kept = [r for r in rows if "skipped" not in r]
    if bounds is None:
        if not kept:
            return []
        bounds = (kept[0]["aligned_s"], kept[-1]["aligned_s"], kept[0]["travelled_m"], kept[-1]["travelled_m"])
    t_last, d_last = bounds[0], bounds[2]
    out = []
    for r in kept:
        if r["chain_updated"]:
            if r["aligned_s"] > t_last:
                out.append((t_last, r["aligned_s"], d_last, r["travelled_m"]))
            t_last, d_last = r["aligned_s"], r["travelled_m"]
    if bounds[1] > t_last:
        out.append((t_last, bounds[1], d_last, bounds[3]))
    return out


def losses(rows: list[dict], bounds: tuple | None = None) -> dict:
    intervals = loss_intervals(rows, bounds)
    kept = [r for r in rows if "skipped" not in r]
    skipped = {k: sum(1 for r in rows if r.get("skipped") == k) for k in ("before_anchor", "control_gap", "no_valid_time")}
    if not intervals:
        return {"rows": len(kept), "longest_s": None, "longest_m": None, "terminal_s": None, "terminal_m": None,
                "skipped": skipped}
    end_t = bounds[1] if bounds is not None else kept[-1]["aligned_s"]
    updates = [r["aligned_s"] for r in kept if r["chain_updated"]]
    terminal = intervals[-1] if intervals[-1][1] >= end_t - 1e-12 and (not updates or updates[-1] < end_t) else None
    return {"rows": len(kept), "longest_s": max(b - a for a, b, _, _ in intervals),
            "longest_m": max(d - c for _, _, c, d in intervals),
            "terminal_s": None if terminal is None else terminal[1] - terminal[0],
            "terminal_m": None if terminal is None else terminal[3] - terminal[2],
            "handoff_at_m": next((r["distance_m"] for r in kept if r["mode"] == "roof"), None),
            "skipped": skipped}


def run_bounds(run: CAL.Run, model: CAL.PalletModel) -> tuple | None:
    """(anchor t, insertion end t, travelled at both, camera-face distance at both)."""
    span = run.phase_span()
    if span is None:
        return None
    travel = Travel(run, model)
    return (span[0], span[1], travel.at(span[0]), travel.at(span[1]),
            CAL.face_distance(run.truth, span[0], model), CAL.face_distance(run.truth, span[1], model))


def stats(values) -> dict:
    a = np.abs(np.asarray(values, dtype=float))
    return {"n": int(a.size), "max": float(a.max()), "p95": float(np.percentile(a, 95))}


def bin_summary(rows: list[dict], bounds: tuple | None = None) -> list[dict]:
    """Per 0.1 m camera-face bin: frames by motion and speed, per source the counts,
    invalid reasons and refusals, per pocket and quantity the max / p95 (moving and
    standing apart for the walls), the longest loss touching the bin; a bin with no
    valid observation of a source is marked unbounded (상한 미확보)."""
    kept = [r for r in rows if "skipped" not in r]
    intervals = loss_intervals(rows, None if bounds is None else bounds[:4])
    groups: dict[int, list] = {}
    for r in kept:
        groups.setdefault(math.floor(r["distance_m"] / BIN_M), []).append(r)
    # Every bin the path crossed, an empty one marked unbounded (re-review P2).
    keys = set(groups)
    if bounds is not None:
        keys |= set(range(math.floor(min(bounds[4], bounds[5]) / BIN_M), math.floor(max(bounds[4], bounds[5]) / BIN_M) + 1))
    out = []
    for k in sorted(keys, reverse=True):
        group = groups.get(k, [])
        if not group:
            out.append({"bin_m": [round((k + 1) * BIN_M, 3), round(k * BIN_M, 3)], "frames": 0, "unbounded": True})
            continue
        cell = {"bin_m": [round((k + 1) * BIN_M, 3), round(k * BIN_M, 3)], "frames": len(group),
                "standing": sum(r["standing"] for r in group), "lost_reads": sum(1 for r in group if "loss" in r),
                # The calibration's classes: slow 0.05 <= v < 0.10, fast >= 0.15 m/s.
                "speed": {"slow": sum(CAL.SLOW_MPS[0] <= r["speed_mps"] < CAL.SLOW_MPS[1] for r in group),
                          "fast": sum(r["speed_mps"] >= CAL.FAST_MPS for r in group)}}
        times = [r["aligned_s"] for r in group]
        touching = [iv for iv in intervals if iv[0] <= max(times) and iv[1] >= min(times)]
        cell["longest_loss_s"] = max((b - a for a, b, _, _ in touching), default=0.0)
        cell["longest_loss_m"] = max((d - c for _, _, c, d in touching), default=0.0)
        for source in ("front", "roof", "roof_truth_prior"):
            obs = [(r, r[source]) for r in group if r.get(source) is not None]
            valid = [(r, o) for r, o in obs if o["errors"] is not None]
            entry = {"observed": len(obs), "valid": len(valid),
                     "accepted": sum(1 for _, o in obs if o["refused"] is None and source != "roof_truth_prior"),
                     "invalid_reasons": {}, "refused": {}}
            for _, o in obs:
                if o["errors"] is None:
                    entry["invalid_reasons"][str(o["reason"])] = entry["invalid_reasons"].get(str(o["reason"]), 0) + 1
                if o["refused"] is not None and source != "roof_truth_prior":
                    entry["refused"][o["refused"]] = entry["refused"].get(o["refused"], 0) + 1
            if not valid:
                entry["unbounded"] = True
            else:
                entry["yaw_rad"] = stats([o["errors"]["yaw_rad"] for _, o in valid])
                for side in ("left", "right"):
                    e = [o["errors"][side] for _, o in valid]
                    entry[side] = {q: stats([x[q] for x in e]) for q in
                                   ("centre_m", "lateral_m", "along_m", "vertical_m", "width_m", "height_m")}
                    for motion, pick in (("moving", False), ("standing", True)):
                        walls = [max(abs(v) for v in o["errors"][side]["wall_m"].values())
                                 for r, o in valid if r["standing"] == pick]
                        entry[side][f"wall_m_{motion}"] = stats(walls) if walls else None
            cell[source] = entry
        cell["handoff_agree"] = sum(1 for r in group if r["handoff_agree"])
        cell["modes"] = sorted({r["mode"] for r in group})
        out.append(cell)
    return out


# ⑤ ---------------------------------------------------------------------------------------
def render_world(run: CAL.Run, t: float, geometry, truck) -> tuple[np.ndarray, np.ndarray]:
    """CPU depth of the pallet, the floor and the truck's carriage boxes at the truth of
    time t, in the D8a contract (1 mm rounding, optical depth < CLIP_M invalid), and the
    pixels where the pallet is the nearest surface."""
    from forklift_core.perception.pallet_geometry import pallet_boxes

    k = run.intrinsics
    r_wc, p_wc = run.truth.camera(t)
    r_wb, p_wb = CAL.pose_at(run.truth.t, run.truth.base, t)
    lift = float(np.interp(t, run.truth.t, run.truth.lift))
    r_wp, p_wp = run.truth.pallet_pose(t)
    v, u = np.indices((k.height, k.width))
    rays = np.stack(((u - k.cx) / k.fx, (v - k.cy) / k.fy, np.ones(u.shape)), axis=-1) @ r_wc.T
    depth = np.full(u.shape, np.inf)
    on_pallet = np.zeros(u.shape, dtype=bool)
    for box in pallet_boxes(geometry):
        hit = scene_rig._box_depth(p_wc, rays, p_wp + r_wp @ np.asarray(box.centre_m), box.size_m, r_wp)
        on_pallet |= hit < depth
        depth = np.fmin(depth, hit)
    for box in truck:
        centre = p_wb + r_wb @ (np.asarray(box.centre_m) + [0.0, 0.0, lift])
        hit = scene_rig._box_depth(p_wc, rays, centre, box.size_m, r_wb @ scene_rig._yaw_matrix(box.yaw_rad))
        on_pallet &= ~(hit < depth)
        depth = np.fmin(depth, hit)
    with np.errstate(divide="ignore", invalid="ignore"):
        floor = np.where((rays[..., 2] < 0), -p_wc[2] / rays[..., 2], np.inf)
    on_pallet &= ~(floor < depth)
    depth = np.fmin(depth, floor)
    depth = np.where(np.isfinite(depth), np.round(depth * 1000.0) / 1000.0, np.nan)
    with np.errstate(invalid="ignore"):
        depth = np.where(depth < CLIP_M, np.nan, depth)
    return depth, on_pallet


def depth_agreement(run: CAL.Run, read: dict, l_s: float, geometry, truck, rendered=None) -> dict:
    """⑤ depth: p95 / max of |Isaac - CPU| over the pallet region less the silhouette
    band; valid/invalid disagreement over the whole pallet region (impl P1).
    ``rendered``: the (depth, region) render_world already made for this read."""
    cpu, region = rendered if rendered is not None else render_world(run, read["stamp_s"] - l_s, geometry, truck)
    isaac = scene_input(run, run.depth(read), read["lift_m"]).depth_m
    core = CAL.erode(region, SILHOUETTE_PX)
    both = core & np.isfinite(cpu) & np.isfinite(isaac)
    diff = np.abs(isaac[both] - cpu[both])
    mismatch = region & (np.isfinite(cpu) != np.isfinite(isaac))
    return {"index": read["index"], "region": int(region.sum()), "common": int(both.sum()),
            "depth_p95_m": float(np.percentile(diff, 95)) if diff.size else None,
            "depth_max_m": float(diff.max()) if diff.size else None,
            "validity_mismatch": float(mismatch.sum() / region.sum()) if region.sum() else None}


def state(row: dict) -> tuple:
    def valid(o):
        return o is not None and o["errors"] is not None

    return (valid(row.get("front")), valid(row.get("roof")), row.get("mode"), row.get("chain_updated"),
            row.get("handoff_agree"))


def geometry_difference(a: dict, b: dict) -> dict | None:
    """Spatial and yaw differences of two observations' errors against the same truth."""
    if a is None or b is None or a["errors"] is None or b["errors"] is None:
        return None
    ea, eb = a["errors"], b["errors"]
    spatial = 0.0
    for side in ("left", "right"):
        # The centre as a 3-D distance, not per axis (re-review P1: 4 mm on each axis is 6.9 mm).
        spatial = max(spatial, math.dist([ea[side][q] for q in ("lateral_m", "along_m", "vertical_m")],
                                         [eb[side][q] for q in ("lateral_m", "along_m", "vertical_m")]))
        spatial = max(spatial, abs(ea[side]["height_m"] - eb[side]["height_m"]))
        for wall in ("outer", "inner"):
            spatial = max(spatial, abs(ea[side]["wall_m"][wall] - eb[side]["wall_m"][wall]))
    return {"spatial_m": spatial, "yaw_rad": abs(wrap(ea["yaw_rad"] - eb["yaw_rad"]))}


def correspondence(isaac_rows: list[dict], cpu_rows: list[dict], depth_rows: list[dict]) -> dict:
    """⑤ verdict: the chain replayed on CPU frames against the recorded one, read by read,
    and the depth agreement, against the fixed tolerances; plus the margin the CPU replay
    must give up (max spatial difference + max yaw difference x LEVER_M)."""
    # The whole read order, lost and skipped rows included: a run of disagreements is
    # counted across them, and a read the CPU replay lacks fails it (re-review P2).
    order = [r["index"] for r in isaac_rows]
    complete = order == [r["index"] for r in cpu_rows]
    cpu = {r["index"]: r for r in cpu_rows}
    sequence = [(r, cpu.get(r["index"])) for r in isaac_rows]

    def same_row(a, b):
        if b is None:
            return False
        if "skipped" in a or "skipped" in b:
            return a.get("skipped") == b.get("skipped")
        if "loss" in a or "loss" in b:
            return a.get("loss") == b.get("loss")
        return state(a) == state(b)

    same = [same_row(a, b) for a, b in sequence]
    pairs = [(a, b) for a, b in sequence if b is not None and "skipped" not in a and "loss" not in a
             and "skipped" not in b and "loss" not in b]
    run_ = longest = 0
    for ok in same:
        run_ = 0 if ok else run_ + 1
        longest = max(longest, run_)
    diffs = [d for a, b in pairs for src in ("front", "roof") if (d := geometry_difference(a[src], b[src]))]
    p95 = [d["depth_p95_m"] for d in depth_rows if d["depth_p95_m"] is not None]
    mism = [d["validity_mismatch"] for d in depth_rows if d["validity_mismatch"] is not None]
    spatial = max((d["spatial_m"] for d in diffs), default=None)
    yaw = max((d["yaw_rad"] for d in diffs), default=None)
    checks = {
        "complete": complete,
        "depth_p95": None if not p95 else max(p95) <= TOL["depth_p95_m"],
        "mismatch": None if not mism else max(mism) <= TOL["mismatch"],
        "agreement": None if not same else sum(same) / len(same) >= TOL["agreement"],
        # Both chains run over every read, so the run of disagreements is certified.
        "disagree_run": None if not same else longest <= TOL["disagree_run"],
        "geometry": None if spatial is None else spatial <= TOL["geometry_m"],
        "yaw": None if yaw is None else yaw <= TOL["yaw_rad"],
    }
    return {
        "reads": len(sequence), "frames": len(pairs), "depth_frames": len(depth_rows),
        "depth_p95_m_max": max(p95, default=None), "validity_mismatch_max": max(mism, default=None),
        "frames_without_depth_comparison": sum(1 for d in depth_rows if d["depth_p95_m"] is None),
        "agreement": None if not same else sum(same) / len(same), "longest_disagreement": longest,
        "spatial_m_max": spatial, "yaw_rad_max": yaw,
        "margin_m": None if spatial is None else spatial + (yaw or 0.0) * LEVER_M,
        "checks": checks, "pass": all(v is True for v in checks.values()),
    }


# CLI -------------------------------------------------------------------------------------
def check_assets(run: CAL.Run, geometry_path: Path, urdf_path: Path) -> None:
    """The geometry and URDF read here are the ones the run used, by content (impl P2)."""
    for key, path in (("pallet_geometry_sha256", geometry_path), ("forklift_urdf_sha256", urdf_path)):
        actual = hashlib.sha256(Path(path).read_bytes()).hexdigest()
        if run.result.get(key) != actual:
            raise ValueError(f"{run.key}: {path} is not the {key} the run used")


def check_calibration(run: CAL.Run, key: str, calibration: dict) -> None:
    """The run is the one the calibration measured, under its contract and hashes, so its
    L applies here (re-review P2)."""
    contract = calibration["contract"]
    for k in CAL.CONTRACT_KEYS:
        if run.meta.get(k) != contract.get(k):
            raise ValueError(f"{key}: meta {k} differs from the calibration's contract")
    for k in CAL.HASH_KEYS:
        if run.result.get(k) != contract.get(k):
            raise ValueError(f"{key}: {k} differs from the calibration's contract")
    recorded = calibration["inputs"][key]
    CAL.check_seed(run, recorded.get("seed"))  # scene and odometry noise seed (re-review P2)
    now = {"approach_straight_speed_mps": run.meta.get("approach_straight_speed_mps"),
           "pocket_check": run.meta.get("pocket_check"), "dwells": [d["kind"] for d in run.dwells],
           "reads": len(run.reads)}
    for k, v in now.items():
        if recorded.get(k) != v:
            raise ValueError(f"{key}: {k} is {v}, the calibration recorded {recorded.get(k)}")


def analyse_run(key: str, calibration: dict, args, geometry, prior, front_params, model) -> dict:
    import time

    l_s = calibration["L_s"]
    run = CAL.Run.load(Path(key))
    check_calibration(run, key, calibration)
    urdf = ROOT / run.meta["forklift_urdf"]
    check_assets(run, args.pallet_geometry, urdf)
    truck = scene_rig.truck_boxes(urdf, lift_m=0.0)
    bounds = run_bounds(run, model)
    started = time.monotonic()
    rows = observe_run(run, l_s, geometry, prior, front_params, model)
    out = {"key": key, "bounds": bounds, "bins": bin_summary(rows, bounds), "losses": losses(rows, None if bounds is None else bounds[:4]),
           "rows": rows, "timing_s": {"observe": time.monotonic() - started}}
    if args.noise_seed is not None:
        noisy = observe_run(run, l_s, geometry, prior, front_params, model, noise_seed=args.noise_seed)
        out["noisy_reference"] = {"seed": args.noise_seed, "bins": bin_summary(noisy, bounds),
                                  "losses": losses(noisy, None if bounds is None else bounds[:4])}
    if not args.skip_correspondence:
        anchor = bounds[0] if bounds is not None else math.inf
        chosen = {r["index"] for r in run.reads if CAL.usable(r) and run.truth.phase_at(r["stamp_s"]) in ("approach", "insert")
                  and r["stamp_s"] - l_s >= anchor}
        chosen = set(sorted(chosen)[::args.correspondence_every])
        depth_rows = []

        def cpu_depth(read):
            # One render per read: the chain's depth and, on the chosen reads, the depth
            # comparison (re-review P3).
            rendered = render_world(run, read["stamp_s"] - l_s, geometry, truck)
            if read["index"] in chosen:
                depth_rows.append(depth_agreement(run, read, l_s, geometry, truck, rendered))
            return rendered[0]

        started = time.monotonic()
        cpu_rows = observe_run(run, l_s, geometry, prior, front_params, model, depth_of=cpu_depth)
        out["timing_s"]["correspondence"] = time.monotonic() - started
        out["correspondence"] = correspondence(rows, cpu_rows, depth_rows)
        out["correspondence"]["depth_every"] = args.correspondence_every
        out["correspondence_rows"] = {"depth": depth_rows, "cpu": cpu_rows}
    out["frames"] = {"reads": len(run.reads), "rows": len(rows)}
    return out


def calibration_id(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def collect(calibration: dict, calibration_sha: str, output: Path) -> dict:
    """The summary of every calibrated run from the per-run files, each checked to be that
    run under this calibration; complete only when all are there and the calibration's own
    matrix was complete (re-review P2)."""
    runs, missing = {}, []
    for key in calibration["inputs"]:
        path = output / f"{run_name(key)}.json"
        if not path.exists():
            missing.append(key)
            continue
        data = json.loads(path.read_text())
        if data.get("key") != key or data.get("calibration_sha256") != calibration_sha or data.get("L_s") != calibration["L_s"]:
            raise ValueError(f"{path} is not {key} under this calibration")
        runs[key] = {k: data[k] for k in ("bins", "losses", "correspondence", "noisy_reference", "timing_s", "frames")
                     if k in data}
        runs[key]["file"] = str(path)
    corr = [r["correspondence"] for r in runs.values() if "correspondence" in r]
    matrix_complete = bool(calibration["verdict"].get("complete"))
    return {"L_s": calibration["L_s"], "calibration_sha256": calibration_sha,
            "complete": not missing and matrix_complete, "missing": missing,
            "calibration_matrix_complete": matrix_complete, "calibration_pass": bool(calibration["verdict"].get("pass")),
            "correspondence_pass": bool(corr) and len(corr) == len(runs) and all(c["pass"] for c in corr),
            "margin_m": max((c["margin_m"] for c in corr if c["margin_m"] is not None), default=None),
            "provenance": "front width = observed gap; roof width, height and spacing = pallet model",
            "runs": runs}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--calibration", type=Path, required=True, help="d8b_calibration.json (gives L and the runs)")
    parser.add_argument("--runs", nargs="*", default=None, help="only these calibrated run directories")
    parser.add_argument("--collect", action="store_true", help="only build the summary from the per-run files")
    parser.add_argument("--correspondence-every", type=int, default=1, help="⑤ depth comparison on every n-th frame")
    parser.add_argument("--skip-correspondence", action="store_true")
    parser.add_argument("--noise-seed", type=int, default=None, help="also the noisy reference variant")
    parser.add_argument("--pallet-geometry", type=Path, default=ROOT / "config/pallet_geometry_epal6.yaml")
    parser.add_argument("--pallet-prior", type=Path, default=ROOT / "config/pallet_prior_epal6.yaml")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    from forklift_core.perception.pallet_geometry import load_pallet_geometry
    from forklift_core.perception.pallet_prior import load_pallet_prior
    from forklift_core.perception.pocket_detector import DetectorParams

    calibration = json.loads(args.calibration.read_text())
    calibration_sha = calibration_id(args.calibration)
    if calibration["L_s"] is None:
        raise SystemExit("the calibration has no L")
    args.output.mkdir(parents=True, exist_ok=True)
    if not args.collect:
        geometry = load_pallet_geometry(args.pallet_geometry)
        prior = load_pallet_prior(args.pallet_prior)
        model = CAL.PalletModel.from_geometry(geometry)
        # The D8a mount study's near-field front settings (tools/nearfield_mount_study.py).
        front_params = DetectorParams.derived_for(prior, range_min_m=0.1)
        keys = list(calibration["inputs"]) if not args.runs else [str(Path(k).resolve()) for k in args.runs]
        unknown = [k for k in keys if k not in calibration["inputs"]]
        if unknown:
            raise SystemExit(f"not in the calibration: {unknown}")
        for key in keys:
            result = analyse_run(key, calibration, args, geometry, prior, front_params, model)
            result.update(front_params=dataclasses.asdict(front_params), calibration_sha256=calibration_sha,
                          L_s=calibration["L_s"], noise_seed=args.noise_seed,
                          correspondence_every=None if args.skip_correspondence else args.correspondence_every)
            (args.output / f"{run_name(key)}.json").write_text(json.dumps(result) + "\n")
    summary = collect(calibration, calibration_sha, args.output)
    (args.output / "d8b_observations.json").write_text(json.dumps(summary, indent=1) + "\n")
    print(json.dumps({k: summary[k] for k in ("complete", "missing", "correspondence_pass", "margin_m")}, indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
