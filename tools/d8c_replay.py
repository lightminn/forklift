"""Plan D8c 3판 ⑥: the version-3 tracker replayed open loop on the recorded D8b frames.

docs/plans/2026-10-04-lidar-obstacle-map.md, "D8c 3판 구현 설계" -- the D8 entry gate
that replaces the CPU replay (D8b ⑤ failed, so CPU conclusions do not transfer). The
tracker the runner will use (forklift_core.perception.near_field_tracking) reads every
recorded frame of the third matrix at its recorded time: depth in the D8b ③ contract
(1 mm, no noise), ③'s near-field detector settings, the control pose at stamp - L, the
start contract (the runner's near-capture observation at the pose the runner placed it
with, recovered from its map estimate, aligned at t_before_capture), results delivered at their read time before status() on
the same control tick, the section armed when the tracker's own estimate puts the fork
tip within 1.5 m of the face. The record is open loop: the motion never reacts to the
tracker, so this cannot show closed-loop correction, braking or D5.

Pass (fixed before the replay, all of them):
  (a) no mismatch in any run;
  (b) every run (all reach the gate; the D5 run stops at camera-face 0.171 m) hands off
      with the front midpoint x <= g, at least three frames before its last usable front;
  (c) in the <= 0.08 m/s runs no loss (age > 0.3 s) after arming and not lost at arming;
  (d) from the first tick to the insertion end, every wall point's true lateral error of
      the carried estimate (at the face and at the insertion target depth) is within the
      erosion b_t D8d would use at that tick;
  (e) the start pose -- recovered exactly from the runner's near-capture map estimate --
      lies within 1 mm / 1 mrad of the control record's first approach row.
Before a run is replayed its four records and every depth frame must match the bounds
file's provenance (sha256, frame manifest).

    python tools/d8c_replay.py --bounds config/near_field_bounds_measured.json \\
        --calibration <dir>/d8b_calibration.json --output <dir> [--runs <run dir> ...]
    python tools/d8c_replay.py ... --collect
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools import d8b_calibration as CAL  # noqa: E402
from tools import d8b_observations as OBS  # noqa: E402
from tools import export_near_field_bounds as EX  # noqa: E402

SECTION_M = 1.5  # fork tip to face: the near-field section (plan D8e)
GATE_BACK_M = 0.03  # the gate sits this far inside the last correction point
LAST_CORRECTION_M = 0.6  # fork tip to face (plan D8d)
OPERATING_MPS = 0.08  # (c) applies to runs at or below this approach speed
HANDOFF_MARGIN_FRAMES = 3
SOURCES = (
    "tools/d8c_replay.py",
    "tools/export_near_field_bounds.py",
    "tools/d8b_observations.py",
    "tools/d8b_calibration.py",
    "src/forklift_core/perception/near_field_tracking.py",
    "src/forklift_core/perception/near_field_bounds.py",
    "src/forklift_core/perception/pocket_detector.py",
    "src/forklift_core/perception/roof_tracking.py",
    "src/forklift_core/perception/pocket_observation.py",
    "src/forklift_core/perception/pallet_geometry.py",
    "src/forklift_core/perception/pallet_prior.py",
    "sim/isaac/perception_adapter.py",
    "sim/isaac/insertion_geometry.py",
    "tools/scene_rig.py",
)
VERSION = "d8b-third-hold"  # the recorded runs never release the near-capture hold
START_POSITION_M, START_YAW_RAD = 0.001, 0.001


# Start ----------------------------------------------------------------------------------
def observation_from_dict(o: dict):
    from forklift_core.perception.pocket_observation import Pocket, PocketObservation

    pockets = [
        Pocket(tuple(o[s]["center_m"]), o[s]["width_m"], o[s]["height_m"])
        for s in ("left", "right")
    ]
    return PocketObservation(
        o["stamp_ns"],
        o["clock_domain"],
        o["frame_id"],
        o["source_provenance"],
        o["status"],
        *pockets,
        o["insertion_yaw_rad"],
        o["position_sigma_m"],
        o["yaw_sigma_rad"],
        o["reason"],
    )


def capture_pose(observation, pallet_depth_m: float, recorded: dict) -> tuple:
    """The base_link pose the runner placed the near capture with, recovered exactly from
    its recorded map estimate (perception_adapter: site = pose (+) (centre, insertion yaw)
    inverted). The runner took it from SLAM at the end of the capture, a tick the control
    record does not hold (the capture runs inside SensorCapture)."""
    from sim.isaac import perception_adapter as PA

    cx, cy = PA.estimate_pallet_center_m(observation, pallet_depth_m)
    yaw = OBS.wrap(recorded["yaw_rad"] - observation.insertion_yaw_rad)
    c, s = math.cos(yaw), math.sin(yaw)
    pose = (
        recorded["x_m"] - (c * cx - s * cy),
        recorded["y_m"] - (s * cx + c * cy),
        yaw,
    )
    site = PA.estimate_world_pallet_site(
        (cx, cy), observation.insertion_yaw_rad, pose[:2], pose[2]
    )
    if (
        math.hypot(site.x_m - recorded["x_m"], site.y_m - recorded["y_m"]) > 1e-9
        or abs(OBS.wrap(site.yaw_rad - recorded["yaw_rad"])) > 1e-9
    ):
        raise ValueError(
            "the recovered capture pose does not rebuild the recorded estimate"
        )
    return pose


def start_check(pose, control_pose) -> dict:
    """(e): the recovered capture pose against the control record's first approach row --
    the same SLAM estimate a few ticks apart while the truck stands (the control estimate
    moved about 0.8 mm across the capture gap in the D8b smoke)."""
    d = math.hypot(pose[0] - control_pose[0], pose[1] - control_pose[1])
    dyaw = abs(OBS.wrap(pose[2] - control_pose[2]))
    return {
        "position_m": d,
        "yaw_rad": dyaw,
        "ok": d <= START_POSITION_M and dyaw <= START_YAW_RAD,
    }


# Walls ----------------------------------------------------------------------------------
def to_base(point_xy, base) -> np.ndarray:
    c, s = math.cos(base[2]), math.sin(base[2])
    d = np.asarray(point_xy[:2], dtype=float) - base[:2]
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])


def estimate_walls(latest, base_now) -> dict:
    """The accepted observation carried to now (held frame -> base_link at base_now):
    per pocket the outer and inner face wall points and the insertion axis."""
    yaw = OBS.wrap(latest.held["yaw"] - base_now[2])
    axis = np.array([math.cos(yaw), math.sin(yaw)])
    lateral = np.array([-axis[1], axis[0]])
    out = {"axis": axis}
    for side, sign in (("left", 1.0), ("right", -1.0)):
        centre = to_base(latest.held[side], base_now)
        half = getattr(latest.observation, side).width_m / 2
        out[side] = {
            "outer": centre + sign * half * lateral,
            "inner": centre - sign * half * lateral,
        }
    return out


def wall_check(
    latest,
    base_now,
    truth: dict,
    depths: tuple,
    bounds,
    now_s: float,
    rear_x_m: float,
    pallet_term_m: float,
) -> dict:
    """(d) at one tick: the worst ratio of a wall point's true lateral error to b_t, with
    rho the distance from the rear axle to the farthest wall point checked (plan D8d)."""
    from forklift_core.perception.near_field_tracking import wall_erosion_m

    age = now_s - (latest.stamp_s - bounds.max_latency_s)
    odometry = bounds.odometry(age)
    if odometry is None:
        return {"age_s": age, "unbounded": True}
    est = estimate_walls(latest, base_now)
    t_axis = np.array([math.cos(truth["yaw"]), math.sin(truth["yaw"])])
    t_lateral = np.array([-t_axis[1], t_axis[0]])
    points = []
    for side in ("left", "right"):
        for name in ("outer", "inner"):
            for depth in depths:
                e_point = est[side][name] + depth * est["axis"]
                t_point = np.asarray(truth[f"{side}_walls"][name][:2]) + depth * t_axis
                points.append(
                    (
                        side,
                        name,
                        depth,
                        e_point,
                        abs(float(np.dot(e_point - t_point, t_lateral))),
                    )
                )
    rho = max(math.hypot(p[3][0] - rear_x_m, p[3][1]) for p in points)
    worst = {"ratio": 0.0}
    for side, name, depth, _, error in points:
        allowed = wall_erosion_m(latest.bound, odometry, depth, rho, pallet_term_m)
        if error / allowed > worst["ratio"]:
            worst = {
                "ratio": error / allowed,
                "error_m": error,
                "allowed_m": allowed,
                "side": side,
                "wall": name,
                "depth_m": depth,
            }
    worst.update(age_s=age, rho_m=rho)
    return worst


def estimate_errors(latest, base_now, truth: dict) -> dict:
    """The carried estimate against the truth now: per pocket the centre's lateral and
    along error in the truth axes (the largest of the two pockets) and the yaw error."""
    yaw = OBS.wrap(latest.held["yaw"] - base_now[2])
    t_axis = np.array([math.cos(truth["yaw"]), math.sin(truth["yaw"])])
    t_lateral = np.array([-t_axis[1], t_axis[0]])
    lateral = along = 0.0
    for side in ("left", "right"):
        d = to_base(latest.held[side], base_now) - np.asarray(truth[side][:2])
        lateral = max(lateral, abs(float(np.dot(d, t_lateral))))
        along = max(along, abs(float(np.dot(d, t_axis))))
    return {
        "lateral_m": lateral,
        "along_m": along,
        "yaw_rad": abs(OBS.wrap(yaw - truth["yaw"])),
        "source": latest.source,
    }


def observation_excess(observation, bound: dict, truth: dict, source: str) -> dict:
    """How far a valid, bounded observation's true error exceeds its bound, per component
    (lateral, along, yaw, wall and -- front -- width)."""
    e = OBS.pocket_errors(observation, truth)
    sides = ("left", "right")
    measured = {
        "lateral_m": max(abs(e[s]["lateral_m"]) for s in sides),
        "along_m": max(abs(e[s]["along_m"]) for s in sides),
        "yaw_rad": abs(e["yaw_rad"]),
        "wall_m": max(abs(v) for s in sides for v in e[s]["wall_m"].values()),
    }
    if source == "front":
        measured["width_m"] = max(abs(e[s]["width_m"]) for s in sides)
    return {k: max(0.0, v - bound[k]) for k, v in measured.items()}


def budget_stops(
    accepted_arrivals: list, armed_since: float | None, end_s: float, period_s: float
) -> int:
    """Missed results inside the armed section that would have stopped the truck at the
    near-field cruise: a result is due one render period after the last accepted one, so
    a gap stops the truck when it stays open past that while armed -- some time in
    (max(last + period, arming), next arrival) (the next result is processed before the
    tick's check), or up to the end for the terminal gap (Codex S3 3rd review P2)."""
    if armed_since is None:
        return 0
    limit = period_s + 1e-4
    count = sum(
        1
        for a, b in zip(accepted_arrivals, accepted_arrivals[1:], strict=False)
        if b - a > limit and max(a + limit, armed_since) < b
    )
    if accepted_arrivals:
        last = accepted_arrivals[-1]
        if end_s - last > limit and max(last + limit, armed_since) <= end_s:
            count += 1
    return count


def repo_sources() -> dict:
    """Every Python file the replay depends on: the whole forklift_core package and the
    named tools/sim files, one sha256 each (Codex S3 3rd review P1: a transitive
    dependency changed the detector input without changing the provenance)."""
    files = sorted((ROOT / "src/forklift_core").rglob("*.py")) + [
        ROOT / n for n in SOURCES
    ]
    return {str(f.relative_to(ROOT)): EX.sha256(f) for f in files}


def uncovered_modules(covered: dict) -> list:
    """Repository modules loaded in this process that the provenance does not cover."""
    out = []
    for module in list(sys.modules.values()):
        path = getattr(module, "__file__", None)
        if not path:
            continue
        path = Path(path).resolve()
        if ROOT in path.parents and "site-packages" not in path.parts:
            rel = str(path.relative_to(ROOT))
            if rel.endswith(".py") and rel not in covered:
                out.append(rel)
    return sorted(out)


# One run --------------------------------------------------------------------------------
def check_inputs(
    run, bounds_data: dict, calibration: dict, geometry_path: Path
) -> None:
    """The run is the one the bounds were measured on, byte for byte."""
    OBS.check_calibration(run, run.key, calibration)
    OBS.check_assets(run, geometry_path, ROOT / run.meta["forklift_urdf"])
    recorded = bounds_data["provenance"]["runs"].get(EX.artifact_path(run.key))
    if recorded is None:
        raise ValueError(f"{run.key}: not in the bounds file's provenance")
    for name in EX.RAW_FILES:
        if EX.sha256(run.directory / name) != recorded[name]:
            raise ValueError(
                f"{run.key}: {name} differs from the bounds file's provenance"
            )
    frames = EX.frame_manifest(run)
    if (
        frames["count"] != recorded["frames"]["count"]
        or frames["sha256"] != recorded["frames"]["sha256"]
    ):
        raise ValueError(
            f"{run.key}: the depth frames differ from the bounds file's provenance"
        )


def replay_run(key: str, calibration: dict, bounds_data: dict, args) -> dict:
    from forklift_core.perception.near_field_bounds import NearFieldBounds
    from forklift_core.perception.near_field_tracking import (
        NearFieldConfig,
        NearFieldTracker,
    )
    from forklift_core.perception.pallet_geometry import load_pallet_geometry
    from forklift_core.perception.pallet_prior import load_pallet_prior
    from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
    from forklift_core.perception.roof_tracking import track_roof
    from sim.isaac.insertion_geometry import tracking_frame

    run = CAL.Run.load(Path(key))
    check_inputs(run, bounds_data, calibration, args.pallet_geometry)
    bounds = NearFieldBounds.from_dict(bounds_data)
    geometry = load_pallet_geometry(args.pallet_geometry)
    prior = load_pallet_prior(args.pallet_prior)
    front_params = DetectorParams.derived_for(
        prior, range_min_m=0.1
    )  # ③'s near-field settings
    model = CAL.PalletModel.from_geometry(geometry)
    frame_geo = tracking_frame(ROOT / run.meta["forklift_urdf"], geometry)
    gate = frame_geo.fork_tip_x_m + LAST_CORRECTION_M - GATE_BACK_M
    rear_x = -abs(run.meta["rear_axle_offset_m"])
    config = NearFieldConfig(
        handoff_start_front_x_m=gate,
        camera_xy_m=tuple(run.truth.mount.translation_m[:2]),
        rear_axle_x_m=rear_x,
        opening_width_m=geometry.opening_width_m,
        pocket_spacing_m=2 * geometry.opening_centre_offset_m,
    )
    span = run.phase_span()
    perception = run.result["perception"]
    initial = observation_from_dict(perception["pocket_observation"])
    start_pose = capture_pose(
        initial, geometry.overall_depth_m, perception["perception_pickup_estimate_m"]
    )
    start = start_check(start_pose, OBS.control_base(run, span[0]))

    cache: dict = {}

    def front_fn(scene):
        if scene.stamp_ns not in cache:
            cache[scene.stamp_ns] = detect_pockets(
                scene, prior, front_params
            ).observation
        return cache[scene.stamp_ns]

    roof_seen: dict = {}

    def roof_fn(scene, xy, yaw):
        roof_seen[scene.stamp_ns] = track_roof(
            scene, geometry, (xy[0], xy[1], geometry.opening_centre_height_m), yaw
        ).observation
        return roof_seen[scene.stamp_ns]

    tracker = NearFieldTracker(
        config,
        bounds,
        initial,
        start_pose,
        float(perception["t_before_capture_s"]),
        front_fn=front_fn,
        roof_fn=roof_fn,
        frame_version=VERSION,
    )
    reads = sorted(
        (
            r
            for r in run.reads
            if r["phase"] in ("approach", "insert")
            and CAL.usable(r)
            and r.get("time_label", r["result"]) == "ok"
            and r["stamp_s"] is not None
        ),
        key=lambda r: r["read_s"],
    )
    reads = [
        r for r in reads if r["stamp_s"] - bounds.align_latency_s >= span[0] - 1e-9
    ]  # not the capture
    ticks = run.control[
        (run.control[:, 0] >= span[0] - 1e-9) & (run.control[:, 0] <= span[1] + 1e-9), 0
    ]
    depths = (0.0, frame_geo.insertion_target_m)
    frames, statuses, walls, unbounded_ticks = [], [], [], 0
    front_usable = []  # read index of every frame whose front is valid and bounded
    checkpoints = {}  # camera-face 1.331 / 0.931 m (the correction points) and the end
    marks = [
        ("correction_1.0", frame_geo.fork_tip_x_m - config.camera_xy_m[0] + 1.0),
        (
            "correction_0.6",
            frame_geo.fork_tip_x_m - config.camera_xy_m[0] + LAST_CORRECTION_M,
        ),
    ]
    handoff = None
    armed_at = None
    next_read = 0
    for t in ticks:
        while next_read < len(reads) and reads[next_read]["read_s"] <= t + 1e-9:
            read = reads[next_read]
            next_read += 1
            stamp = read["stamp_s"]
            aligned = stamp - bounds.align_latency_s
            if not OBS.control_covers(run, aligned):
                frames.append({"index": read["index"], "skipped": "control_gap"})
                continue
            scene = OBS.scene_input(run, run.depth(read), read["lift_m"])
            scene = dataclasses.replace(scene, stamp_ns=int(round(stamp * 1e9)))
            front = front_fn(scene)
            if (
                front.status == "valid"
                and bounds.observation("front", tracker.distance_m(front)) is not None
            ):
                front_usable.append(read["index"])
            pose = OBS.control_base(run, aligned)
            result = tracker.on_frame(scene, stamp, pose, float(t), VERSION)
            truth = OBS.truth_in_base(run, aligned, geometry)
            row = {
                "index": read["index"],
                "stamp_s": stamp,
                "now_s": float(t),
                "distance_m": CAL.face_distance(run.truth, aligned, model),
                **dataclasses.asdict(result),
            }
            if result.accepted:
                row["source"] = tracker.latest.source
            row["excess"], row["bins"] = {}, {}
            for source, observation in (
                ("front", front),
                ("roof", roof_seen.get(scene.stamp_ns)),
            ):
                if observation is None or observation.status != "valid":
                    continue
                distance = tracker.distance_m(observation)
                row["bins"][source] = math.floor(distance / 0.1) / 10
                bound = bounds.observation(source, distance)
                if bound is not None:
                    row["excess"][source] = observation_excess(
                        observation, bound, truth, source
                    )
            if handoff is None and result.mode == "roof":
                mid_x = (
                    (front.left.center_m[0] + front.right.center_m[0]) / 2
                    if front.status == "valid"
                    else None
                )
                handoff = {
                    "index": read["index"],
                    "distance_m": row["distance_m"],
                    "front_mid_x_m": mid_x,
                }
            frames.append(row)
        base_now = OBS.control_base(run, float(t))
        if armed_at is None:
            est = estimate_walls(tracker.latest, base_now)
            face_x = min(
                est[s][w][0] for s in ("left", "right") for w in ("outer", "inner")
            )
            if face_x - frame_geo.fork_tip_x_m <= SECTION_M:
                tracker.arm(float(t))
                armed_at = {
                    "now_s": float(t),
                    "distance_m": CAL.face_distance(run.truth, float(t), model),
                    "lost": tracker.status(float(t)).lost,
                }
        status = tracker.status(float(t))
        statuses.append(
            {
                "now_s": float(t),
                "age_s": status.age_s,
                "lost": status.lost,
                "failed": status.failed,
                "armed": tracker.armed,
            }
        )
        truth_now = OBS.truth_in_base(run, float(t), geometry)
        distance_now = CAL.face_distance(run.truth, float(t), model)
        for name, at in marks:
            if name not in checkpoints and distance_now <= at:
                checkpoints[name] = {
                    "distance_m": distance_now,
                    **estimate_errors(tracker.latest, base_now, truth_now),
                }
        w = wall_check(
            tracker.latest,
            base_now,
            truth_now,
            depths,
            bounds,
            float(t),
            rear_x,
            config.pallet_term_m,
        )
        if w.get("unbounded"):
            unbounded_ticks += 1
        walls.append(w)
    checkpoints["stop" if run.meta.get("pocket_check") else "end"] = {
        "distance_m": distance_now,
        **estimate_errors(tracker.latest, base_now, truth_now),
    }
    out = summarise(
        run,
        frames,
        statuses,
        walls,
        unbounded_ticks,
        front_usable,
        handoff,
        armed_at,
        start,
        gate,
        bounds,
        float(ticks[-1]),
        tracker.anchor.stamp_s + bounds.read_delay_s,
    )
    out["checkpoints"] = checkpoints
    out["provenance"] = provenance(key, calibration, bounds_data, args)
    missing = uncovered_modules(out["provenance"]["sources_sha256"])
    if missing:
        raise ValueError(
            f"repository modules loaded but not in the provenance: {missing}"
        )
    return out


def recorded_urdf_sha(key: str) -> str:
    meta = json.loads((Path(key) / "pocket_frames/index.json").read_text())["meta"]
    return EX.sha256(ROOT / meta["forklift_urdf"])


def provenance(key: str, calibration: dict, bounds_data: dict, args) -> dict:
    """What a run's result was computed from; the collector refuses a mismatch."""
    recorded = calibration["inputs"][key]
    return {
        "run": EX.artifact_path(key),
        "calibration_sha256": EX.sha256(args.calibration),
        "bounds_sha256": EX.sha256(args.bounds),
        "tool_sha256": EX.sha256(Path(__file__)),
        "sources_sha256": repo_sources(),
        "pallet_geometry_sha256": EX.sha256(args.pallet_geometry),
        "pallet_prior_sha256": EX.sha256(args.pallet_prior),
        "forklift_urdf_sha256": recorded_urdf_sha(key),
        "speed_mps": recorded.get("approach_straight_speed_mps"),
        "pocket_check": recorded.get("pocket_check"),
    }


def summarise(
    run,
    frames,
    statuses,
    walls,
    unbounded_ticks,
    front_usable,
    handoff,
    armed_at,
    start,
    gate,
    bounds,
    end_s: float,
    initial_arrival_s: float,
) -> dict:
    rows = [f for f in frames if "skipped" not in f]
    rejects: dict = {}
    for f in rows:
        for source in ("front", "roof"):
            if f[f"{source}_eval"] == "rejected":
                key = f"{source}:{f['bins'].get(source)}:{f[f'{source}_reason']}"
                rejects[key] = rejects.get(key, 0) + 1
    # The near capture is the first accepted result: a gap before the first replayed
    # acceptance counts too (Codex S3 2nd review P2).
    accepted_arrivals = [initial_arrival_s] + [
        f["now_s"] for f in rows if f["accepted"]
    ]
    armed_since = None if armed_at is None else armed_at["now_s"]
    lost_after_arm = sum(1 for s in statuses if s["armed"] and s["lost"])
    worst_wall = max(
        (w for w in walls if not w.get("unbounded")),
        key=lambda w: w["ratio"],
        default=None,
    )
    handoff_ok = None
    if handoff is not None:
        later = [i for i in front_usable if i > handoff["index"]]
        handoff_ok = (
            handoff["front_mid_x_m"] is not None
            and handoff["front_mid_x_m"] <= gate + 1e-9
            and len(later) >= HANDOFF_MARGIN_FRAMES
        )
    excess = {}
    for source in ("front", "roof"):
        entries = [f["excess"][source] for f in rows if source in f["excess"]]
        keys = sorted({k for e in entries for k in e})
        excess[source] = {
            "checked": len(entries),
            "exceeding": sum(1 for e in entries if any(v > 0 for v in e.values())),
            "max": {k: max(e.get(k, 0.0) for e in entries) for k in keys},
        }
    return {
        "key": run.key,
        "frames": len(frames),
        "delivered": len(rows),
        "skipped": len(frames) - len(rows),
        "accepted": sum(f["accepted"] for f in rows),
        "rejected": sum(f["rejected"] for f in rows),
        "invalid": sum(not f["accepted"] and not f["rejected"] for f in rows),
        "reject_reasons": rejects,
        "mismatches": sum(f["mismatch"] for f in rows),
        "unbounded_events": sum(f["unbounded"] for f in rows),
        "handoff": handoff,
        "handoff_ok": handoff_ok,
        "gate_m": gate,
        "front_usable_after_handoff": None
        if handoff is None
        else sum(1 for i in front_usable if i > handoff["index"]),
        "armed": armed_at,
        "lost_ticks_after_arm": lost_after_arm,
        "budget_stops_at_cruise": budget_stops(
            accepted_arrivals, armed_since, end_s, bounds.render_period_s
        ),
        "longest_age_after_arm_s": max(
            (s["age_s"] for s in statuses if s["armed"]), default=None
        ),
        "wall_worst": worst_wall,
        "wall_unbounded_ticks": unbounded_ticks,
        "wall_violations": sum(
            1 for w in walls if not w.get("unbounded") and w["ratio"] > 1.0
        ),
        "observation_excess": excess,
        "start": start,
        "rows": frames,
    }


def verdict(results: list[dict]) -> dict:
    checks = {
        "a_no_mismatch": all(r["mismatches"] == 0 for r in results),
        # every recorded run reaches the gate (the D5 run stops at camera-face 0.171 m)
        "b_handoff": all(r["handoff"] is not None and r["handoff_ok"] for r in results),
        "c_no_loss": all(
            r["lost_ticks_after_arm"] == 0
            and r["armed"] is not None
            and not r["armed"]["lost"]
            for r in results
            if (r["provenance"]["speed_mps"] or 0) <= OPERATING_MPS + 1e-9
        ),
        "d_walls": all(
            r["wall_violations"] == 0 and r["wall_unbounded_ticks"] == 0
            for r in results
        ),
        "e_start": all(r["start"]["ok"] for r in results),
    }
    return {"pass": all(checks.values()), "checks": checks, "runs": len(results)}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument(
        "--bounds", type=Path, default=ROOT / "config/near_field_bounds_measured.json"
    )
    parser.add_argument("--calibration", type=Path, required=True)
    parser.add_argument(
        "--pallet-geometry",
        type=Path,
        default=ROOT / "config/pallet_geometry_epal6.yaml",
    )
    parser.add_argument(
        "--pallet-prior", type=Path, default=ROOT / "config/pallet_prior_epal6.yaml"
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--runs", nargs="*")
    parser.add_argument("--collect", action="store_true")
    args = parser.parse_args(argv)
    calibration = json.loads(args.calibration.read_text())
    bounds_data = json.loads(args.bounds.read_text())
    if bounds_data["provenance"]["calibration"]["sha256"] != EX.sha256(
        args.calibration
    ):
        raise SystemExit("the bounds were not measured on this calibration")
    args.output.mkdir(parents=True, exist_ok=True)
    keys = (
        list(calibration["inputs"])
        if not args.runs
        else [str(Path(k).resolve()) for k in args.runs]
    )
    if not args.collect:
        for key in keys:
            result = replay_run(key, calibration, bounds_data, args)
            (args.output / f"{OBS.run_name(key)}.json").write_text(
                json.dumps(result, default=OBS.json_default) + "\n"
            )
            print(
                key,
                {
                    k: result[k]
                    for k in (
                        "accepted",
                        "rejected",
                        "mismatches",
                        "handoff_ok",
                        "lost_ticks_after_arm",
                        "wall_violations",
                    )
                },
            )
        return 0
    results = []
    expected = repo_sources()
    for key in calibration["inputs"]:
        path = args.output / f"{OBS.run_name(key)}.json"
        if not path.exists():
            raise SystemExit(f"missing {path}")
        result = json.loads(path.read_text())
        result.pop("rows", None)
        now = provenance(key, calibration, bounds_data, args)
        if (
            result.get("provenance") != now
            or result["key"] != key
            or expected != now["sources_sha256"]
        ):
            raise SystemExit(
                f"{path} was not computed from this run, calibration, bounds and code"
            )
        results.append(result)
    summary = {
        "verdict": verdict(results),
        "runs": results,
        "bounds_sha256": EX.sha256(args.bounds),
        "tool_sha256": EX.sha256(Path(__file__)),
    }
    (args.output / "d8c_replay.json").write_text(
        json.dumps(summary, indent=1, default=OBS.json_default) + "\n"
    )
    print(json.dumps(summary["verdict"], indent=1))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
