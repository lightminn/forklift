"""Stationary LiDAR scans at listed poses, for the global matcher's calibration (docs/plans/2026-10-27-autonomous-
survey-and-relocalization.md B4a): run_transport.py --scan-capture-poses moves the truck to each base pose (teleport,
at rest), steps the physics twice, reads the pose back as the truth and takes ``SCANS`` scans there the way the live
runner sends the shared LiDAR's scan to SLAM (``shared_sent_ranges``: REP-117 ranges, the noise on the finite ones,
clipped to the sensor's limits), each with its own noise seed (``noise_rng``). A pose the truck did not settle on
(position, heading, height, tilt, or motion during the scans) is recorded invalid, never replaced. Isaac-free here:
the runner hands in the engine calls, so the bookkeeping is tested on CPU.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

FORMAT = "scan_capture/1"
SCANS = 5  # 0.5 s of 10 Hz
NOISE_ROOT = 20261027
SETTLE_STEPS = 2
POSITION_TOL_M, YAW_TOL_RAD = 0.02, 0.01
HEIGHT_TOL_M, TILT_TOL_RAD = 0.01, 0.01
STILL_M, STILL_RAD = 0.002, 0.002  # between the first and the last scan


# the sources that build the static world the LiDAR plane meets (the hall layout, the scene file, the base scene and the
# ray cast): a capture compares them with the survey's (Codex B4a re-review P2); run_transport.py itself differs by the
# capture hook and is reviewed by its diff
WORLD_SOURCES = ("forklift_core/planning/factory_layout.py", "sim/isaac/scene_file.py", "sim/isaac/scene.py",
                 "sim/isaac/build_base_scene.py", "sim/isaac/planar_lidar.py", "forklift_core/sensors/lidar.py")


def world_source_mismatches(survey_sources: dict, run_sources: dict) -> list:
    """The world-building sources whose hash differs from the survey's (one missing on either side differs too)."""
    return [f"{name} differs from the survey's" for name in WORLD_SOURCES
            if (survey_sources or {}).get(name) is None or survey_sources.get(name) != (run_sources or {}).get(name)]


def capture_mismatches(manifest: dict, *, seed, arguments: dict | None, input_sha256: dict,
                       scene_sha256: dict | None, run_sources: dict | None) -> list:
    """What of a capture run differs from the survey's world: ``slam_map.run_mismatches`` with the SLAM noise seed
    taken as the survey's -- the capture draws its own noise per scan (``noise_rng``) and runs without the SLAM link,
    which the runner requires for --slam-noise-seed (Codex B4a round 3 P2) -- plus the world-building sources. The
    factory layout's seed, the noise model and every other argument are compared as they are."""
    from forklift_core.localization import slam_map  # noqa: PLC0415

    survey_arguments = ((manifest.get("run") or {}).get("arguments") or {})
    args = {**(arguments or {}), "slam_noise_seed": survey_arguments.get("slam_noise_seed")}
    return (slam_map.run_mismatches(manifest, seed=seed, input_sha256=input_sha256, scene_sha256=scene_sha256,
                                    arguments=args)
            + world_source_mismatches((manifest.get("run") or {}).get("source_sha256"), run_sources))


def shared_sent_ranges(ranges, noise, range_min_m: float, range_max_m: float) -> np.ndarray:
    """The ranges the live runner sends SLAM from the shared LiDAR: the noise on the finite REP-117 ranges only, then
    clipped to the sensor's limits (run_transport's shared_scan branch, which calls this)."""
    sent = np.asarray(ranges, dtype=float).copy()
    measured = np.isfinite(sent)
    sent[measured] = np.clip(sent[measured] + np.asarray(noise, dtype=float)[measured], range_min_m, range_max_m)
    return sent


def noise_rng(split: int, map_index: int, pose_index: int, scan_index: int) -> np.random.Generator:
    return np.random.default_rng([NOISE_ROOT, int(split), int(map_index), int(pose_index), int(scan_index)])


def load_poses(path: Path) -> dict:
    """The pose list tools/global_match_capture.py writes: {"format", "split", "map_index", "poses": [{"id",
    "stratum", "base": [x, y, yaw] in the map, "base_world": the same in the hall (what the truck is moved to)}]}."""
    record = json.loads(Path(path).read_text())
    if record.get("format") != "scan_capture_poses/1":
        raise ValueError(f"{path}: not a scan_capture_poses/1 file")
    return record


def yaw_roll_pitch(q_wxyz) -> tuple:
    w, x, y, z = (float(v) for v in q_wxyz)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return yaw, roll, pitch


def quat_from_yaw(yaw: float) -> np.ndarray:
    return np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])


def _wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


@dataclass(frozen=True)
class Engine:
    """What the runner hands in: ``teleport(position_xyz, q_wxyz)``, ``step()``, ``pose() -> (position_xyz, q_wxyz)``,
    ``cast(position_xyz, q_wxyz) -> (distances, hits)`` from the LiDAR mount, ``draw_noise(rng, distances) -> noise``
    (the obstacle layer's model with that generator), ``ranges_from_hits(distances, hits)`` and the sensor limits."""
    teleport: object
    step: object
    pose: object
    cast: object
    draw_noise: object
    ranges_from_hits: object
    range_min_m: float
    range_max_m: float
    settle_z_m: float  # the truck's base height after the start settle
    laser_xyz_m: tuple  # the mount on base_link
    steps_per_scan: int = 12  # physics steps between scans: 120 Hz / the LiDAR's 10 Hz, as live


def scan_set(engine: Engine, target, noise_for, settle_steps: int = SETTLE_STEPS) -> dict:
    """At the truck's current place: ``settle_steps`` physics steps, then ``SCANS`` scans a LiDAR period apart, each
    the live shared send path's ranges with the noise of ``noise_for(scan_index)`` (a generator, or None for the
    obstacle layer's own). Validity against ``target`` (x, y, yaw world base) and the settle height: at the target,
    level and still over every scan. The truth (``base_truth``, ``laser_truth``) is the simulator's, for records and
    validity only. Used per pose by ``capture`` and by the start relocalization (B-P2) at its one start."""
    x, y, yaw = (float(v) for v in target)
    for _ in range(settle_steps):
        engine.step()
    pos0, q0 = engine.pose()
    yaw0, roll0, pitch0 = yaw_roll_pitch(q0)
    raws, sents, hashes, steps, poses_ = [], [], [], [], []
    step_count = settle_steps
    for s in range(SCANS):
        if s:
            for _ in range(engine.steps_per_scan):
                engine.step()
            step_count += engine.steps_per_scan
        pos, q = engine.pose()
        poses_.append((np.asarray(pos, float), yaw_roll_pitch(q)))
        steps.append(step_count)
        distances, hits = engine.cast(pos, q)
        ranges = np.asarray(engine.ranges_from_hits(distances, hits), dtype=float)
        noise = engine.draw_noise(noise_for(s), np.asarray(distances, dtype=float))
        sent = shared_sent_ranges(ranges, noise, engine.range_min_m, engine.range_max_m).astype(np.float32)
        raws.append(ranges.astype(np.float32))
        sents.append(sent)
        hashes.append(hashlib.sha256(sent.tobytes()).hexdigest())
    reason = None
    if math.hypot(pos0[0] - x, pos0[1] - y) > POSITION_TOL_M or abs(_wrap(yaw0 - yaw)) > YAW_TOL_RAD:
        reason = "not_at_target"
    for p_, (yaw_, roll_, pitch_) in poses_:  # every scan, not only the first (Codex B4a round 3 P3)
        if reason is not None:
            break
        if abs(p_[2] - engine.settle_z_m) > HEIGHT_TOL_M:
            reason = "height"
        elif max(abs(roll_), abs(pitch_)) > TILT_TOL_RAD:
            reason = "tilt"
        elif math.hypot(p_[0] - pos0[0], p_[1] - pos0[1]) > STILL_M or abs(_wrap(yaw_ - yaw0)) > STILL_RAD:
            reason = "moved"
    lx, ly, _ = engine.laser_xyz_m
    c, s_ = math.cos(yaw0), math.sin(yaw0)
    return {"base_truth": (float(pos0[0]), float(pos0[1]), yaw0),
            "laser_truth": (float(pos0[0]) + c * lx - s_ * ly, float(pos0[1]) + s_ * lx + c * ly, yaw0),
            "valid": reason is None, "reason": reason or "", "z_m": float(pos0[2]), "roll_rad": roll0,
            "pitch_rad": pitch0, "raw": raws, "sent": sents, "sent_sha256": hashes, "scan_step": steps}


def capture(engine: Engine, poses: dict) -> dict:
    """Every listed pose: the truth, validity and the scans, as arrays for one npz."""
    split, map_index = int(poses["split"]), int(poses["map_index"])
    rows = poses["poses"]
    n = len(rows)
    out = {"id": [], "stratum": [], "base_target": [], "base_truth": [], "laser_truth": [], "valid": [], "reason": [],
           "z_m": [], "roll_rad": [], "pitch_rad": [], "raw": [], "sent": [], "sent_sha256": [], "scan_step": []}
    for k, row in enumerate(rows):
        x, y, yaw = (float(v) for v in row["base_world"])
        engine.teleport(np.array([x, y, engine.settle_z_m]), quat_from_yaw(yaw))
        got = scan_set(engine, (x, y, yaw), lambda s, k=k: noise_rng(split, map_index, k, s))
        out["id"].append(str(row["id"]))
        out["stratum"].append(str(row.get("stratum", "")))
        out["base_target"].append((x, y, yaw))
        for key in ("base_truth", "laser_truth", "valid", "reason", "z_m", "roll_rad", "pitch_rad", "sent_sha256",
                    "scan_step"):
            out[key].append(got[key])
        out["raw"].append(np.stack(got["raw"]))
        out["sent"].append(np.stack(got["sent"]))
    arrays = {k: np.asarray(v) for k, v in out.items()}
    if n == 0:
        arrays["raw"] = arrays["sent"] = np.zeros((0, SCANS, 0), np.float32)
    return arrays


__all__ = ["FORMAT", "SCANS", "WORLD_SOURCES", "Engine", "capture", "capture_mismatches", "load_poses", "noise_rng", "quat_from_yaw", "scan_set",
           "shared_sent_ranges", "world_source_mismatches", "yaw_roll_pitch"]
