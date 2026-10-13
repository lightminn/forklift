"""Plan D5 v11 stage 1: replay a recorded D5 (--pocket-check) run without Isaac.

docs/plans/2026-10-13-d5-v11-pallet-referenced-evidence.md, "재생 도구". With
--record-pocket-frames the runner keeps every near-field depth read (pocket_frames/
index.json and frame_*.npz: the normalised float32 depth, its stamp and read time, the
odometry speed and yaw rate, the lift), the physics truth of every tick
(pocket_frames/truth.npz) and the control pose of every tick (slam_control.npy). This
tool rebuilds the run's PocketCheck from those records with the run's own source
(--source: the snapshot the run executed; every module it loads from there is checked
against the run's source_sha256), feeds it the near capture and then every read in the
runner's order -- a read made after the step of tick t (read_s = t + dt) is the check's
from the tick at read_s on, placed at the control pose of its stamp as read_pocket_frame
interpolates it -- and, at every approach and insert tick from the build on,

  * asks the check's own limit with the tick's control pose and lift; the held curvature
    and the tracker's requested speed are not recorded per tick, so the last 10 Hz
    sample's command curvature and tracker speed are held (stated in the output);
  * walks the same stop arc to its end, not only to the first failure, and keeps for every
    voxel a truck box overlaps for the first time the state at that tick ("first
    demand"): when it was last certified and by which frame (that frame's camera-face
    distance and speed), its age, its last code, and the two-time displacement -- the
    world distance between where the certifying frame's evidence truly was and where the
    demanding box truly is, both placed through the check's own control poses
    (|T_c C_c^-1 p - T_d C_d^-1 p|, C control and T truth rear-axle poses; the pallet's
    truth is reported static or not over the replay).

The check's recorded summary is the reproduction target: its first frame record and its
first stop reason, time and last_fail (the runner updates the summary when the reason
first changes). Each field is reported reproduced or not; the analysis is read only on a
reproduced run.

    python tools/d5_replay.py --run <run dir> --source <snapshot root> --output <file.json>
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from tools.s4_judge import quat_matrix, run_files_digest, sha256_bytes  # noqa: E402

TICK_S = 1.0 / 120.0
TIME_TOL_S = 1e-6
PHASES = ("approach", "insert")
# Modules the replay loads from --source, by their key in the run's source_sha256.
SOURCE_MODULES = {
    "pocket_check": "sim/isaac/pocket_check.py",
    "insertion_geometry": "sim/isaac/insertion_geometry.py",
    "forklift_core.perception.insertion_clearance": "forklift_core/perception/insertion_clearance.py",
    "forklift_core.perception.pocket_clearance": "forklift_core/perception/pocket_clearance.py",
    "forklift_core.perception.pallet_geometry": "forklift_core/perception/pallet_geometry.py",
    "forklift_core.control.drive_permission": "forklift_core/control/drive_permission.py",
}
AGE_EDGES_S = (0.0, 1.0, 5.0, 10.0, 20.0, 30.0, 40.0, math.inf)


class NotReproducible(Exception):
    """The records cannot rebuild the check."""


def load_source(source: Path, recorded: dict) -> tuple[dict, dict]:
    """Import the replay's modules from the run's snapshot, never from this checkout.

    Returns (modules, {key: {"sha256", "matches_run"}})."""
    for name in list(sys.modules):
        if name == "forklift_core" or name.startswith("forklift_core.") or name in SOURCE_MODULES:
            del sys.modules[name]
    for path in (source / "sim" / "isaac", source / "src"):
        sys.path.insert(0, str(path))
    modules, hashes = {}, {}
    for name, key in SOURCE_MODULES.items():
        module = importlib.import_module(name)
        file_ = Path(module.__file__).resolve()
        if source.resolve() not in file_.parents:
            raise NotReproducible(f"{name} loaded from {file_}, not from {source}")
        digest = sha256_bytes(file_.read_bytes())
        hashes[key] = {"sha256": digest, "matches_run": recorded.get(key) == digest}
        modules[name] = module
    return modules, hashes


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def rear_pose(base7: np.ndarray, offset_m: float) -> np.ndarray:
    """(x, y, yaw) of the rear axle centre from a base_link pose7 row."""
    r = quat_matrix(base7[3:])
    p = base7[:3] + r @ np.array([-abs(offset_m), 0.0, 0.0])
    return np.array([p[0], p[1], math.atan2(r[1, 0], r[0, 0])])


def compose_inverse(t_pose, c_pose, points: np.ndarray) -> np.ndarray:
    """T C^-1 applied to control-frame xy points (rear-axle poses (x, y, yaw))."""
    cx, cy, cyaw = c_pose
    tx, ty, tyaw = t_pose
    dx, dy = points[:, 0] - cx, points[:, 1] - cy
    c, s = math.cos(cyaw), math.sin(cyaw)
    bx, by = dx * c + dy * s, -dx * s + dy * c
    c, s = math.cos(tyaw), math.sin(tyaw)
    return np.column_stack((tx + bx * c - by * s, ty + bx * s + by * c))


class Run:
    """The records of one run, read once (the digest covers every file read)."""

    def __init__(self, directory: Path):
        self.directory = directory
        self.digest_before = run_files_digest(directory)
        self.result = json.loads((directory / "result.json").read_text())
        self.index = json.loads((directory / "pocket_frames" / "index.json").read_text())
        data = np.load(directory / "pocket_frames" / "truth.npz")
        self.truth = {k: data[k] for k in data.files}
        self.control = np.load(directory / "slam_control.npy")
        for key in ("pocket_check", "perception", "geometry", "arguments", "samples", "near_capture"):
            if key not in self.result:
                raise NotReproducible(f"result.json lacks {key}")
        if not self.result["arguments"].get("pocket_check"):
            raise NotReproducible("not a --pocket-check run")
        self.truth_t = np.asarray(self.truth["stamp_s"], dtype=float)
        self.control_t = self.control[:, 0]

    def truth_index(self, t: float) -> int | None:
        i = int(np.argmin(np.abs(self.truth_t - t)))
        return i if abs(self.truth_t[i] - t) <= TIME_TOL_S else None

    def truth_rear(self, t: float, offset_m: float):
        i = self.truth_index(t)
        return None if i is None else rear_pose(np.asarray(self.truth["base_pose"][i], dtype=float), offset_m)

    def phase_at(self, t: float) -> str | None:
        i = self.truth_index(t)
        return None if i is None else str(self.truth["phases"][int(self.truth["phase"][i])])


def build_check(run: Run, source: Path, modules: dict):
    """The runner's build_pocket_check from the run's records."""
    pc_mod = modules["pocket_check"]
    clearance = modules["forklift_core.perception.pocket_clearance"]
    stopping_cls = modules["forklift_core.control.drive_permission"].StoppingModel
    import yaml

    r = run.result
    args, geom = r["arguments"], r["geometry"]
    built = r["pocket_check"]
    est = r["perception"]["perception_pickup_estimate_m"]
    files = {}
    for key, sha_key in (("forklift_urdf", "forklift_urdf_sha256"), ("pallet_geometry", "pallet_geometry_sha256")):
        data = (source / args[key]).read_bytes()
        files[key] = sha256_bytes(data) == r.get(sha_key)
    layer = yaml.safe_load((source / args["obstacle_layer"]).read_text())["permission"]
    offset = abs(float(args["rear_axle_offset_m"]))
    k = r["pocket_camera"]["intrinsics"]
    built_s = float(built["built_s"])
    i_built = run.truth_index(built_s)
    if i_built is None:
        raise NotReproducible(f"the build time {built_s} is not a truth tick")
    bfo = r["perception"]["base_from_optical"]
    check = pc_mod.PocketCheck(
        estimate=(float(est["x_m"]), float(est["y_m"]), float(est["yaw_rad"])),
        axis_yaw=float(built["axis_yaw_rad"]),
        pallet_geometry=modules["forklift_core.perception.pallet_geometry"].load_pallet_geometry(
            source / args["pallet_geometry"]),
        blades_rear=tuple((x0 + offset, x1 + offset, y0, y1)
                          for x0, x1, y0, y1 in modules["insertion_geometry"].read_fork_blades_m(
                              source / args["forklift_urdf"])),
        body_front_m=float(geom["axle_to_fork_tip_m"]) - float(geom["carriage_limit_m"]),
        body_rear_m=float(geom["unloaded_footprint"]["rear_m"]),
        body_half_width_m=float(geom["unloaded_footprint"]["half_width_m"]),
        insertion_depth_m=float(geom["axle_to_fork_tip_m"]) + float(geom["pallet_depth_m"]) / 2
        - float(geom["inserted_offset_m"]),
        base_z_m=float(run.truth["base_pose"][i_built][2]),
        base_from_optical=(np.asarray(bfo["rotation"], dtype=float), np.asarray(bfo["translation_m"], dtype=float)),
        rear_axle_offset_m=offset,
        camera=clearance.DepthCamera(float(k["fx"]), float(k["fy"]), float(k["cx"]), float(k["cy"]),
                                     int(k["width"]), int(k["height"])),
        stopping=stopping_cls(float(layer["latency_s"]), float(layer["decel_mps2"]), float(layer["margin_m"])),
        noise_seed=int(args["seed"]),
    )
    return check, files


def held_samples(run: Run):
    """(times, command curvature, tracker speed) of the 10 Hz samples, for a zero-order hold."""
    s = run.result["samples"]
    times = np.array([float(x["time_s"]) for x in s])
    kappa = np.array([float(x.get("curvature_inv_m") or 0.0) for x in s])
    speed = np.array([float((x.get("tracking") or {}).get("speed_mps") or 0.0) for x in s])
    return times, kappa, speed


def demanded(check, box) -> np.ndarray:
    """Voxel indices a truck box asks of the check (ClearanceMemory.contained's set)."""
    from forklift_core.perception.insertion_clearance import overlapping_voxels

    memory = check.memory
    if not memory._box_meets_region(box, check.region):
        return np.zeros((0, 3), dtype=int)
    idx, _ = overlapping_voxels(memory.volume, box, check.region)
    x0, x1, y0, y1 = check.region
    v = memory.volume.voxel_m
    vx0 = memory.volume.lo[0] + idx[:, 0] * v
    vy0 = memory.volume.lo[1] + idx[:, 1] * v
    return idx[(vx0 + v > x0) & (vx0 < x1) & (vy0 + v > y0) & (vy0 < y1)]


def camera_face_m(check, rear) -> float:
    """Camera optical centre to the estimated face, along the insertion axis."""
    rot, trans = check.optical_from_insertion(rear)
    centre = -np.asarray(rot).T @ np.asarray(trans)
    return float(check.face_x - centre[0])


def replay(run: Run, source: Path) -> tuple[dict, dict]:
    recorded_sources = run.result.get("source_sha256") or {}
    modules, hashes = load_source(source, recorded_sources)
    check, model_files = build_check(run, source, modules)
    r = run.result
    built = r["pocket_check"]
    built_s = float(built["built_s"])
    offset = abs(float(r["arguments"]["rear_axle_offset_m"]))
    vol = check.memory.volume
    near = r["near_capture"]
    capture = np.load(run.directory / f"perception_capture_{int(near['near_attempt'])}_depth_m.npy")
    first = check.add_frame(built_s, capture, tuple(float(v) for v in built["first_frame"]["rear"]))
    frames = [{"index": 0, "time_s": built_s, "source": "near_capture",
               "rear": [float(v) for v in built["first_frame"]["rear"]], "speed_mps": 0.0,
               "camera_face_m": camera_face_m(check, built["first_frame"]["rear"]), "erosion_voxels": 0}]
    cert_frame = np.full(vol.shape, -1, dtype=np.int32)
    cert_frame[check.memory.certified_s == built_s] = 0
    # First-demand state per voxel.
    fd_t = np.full(vol.shape, np.nan)
    fd_cert = np.full(vol.shape, np.nan)
    fd_frame = np.full(vol.shape, -1, dtype=np.int32)
    fd_code = np.full(vol.shape, -1, dtype=np.int8)
    fd_box = np.full(vol.shape, -1, dtype=np.int8)
    fd_disp = np.full(vol.shape, np.nan)
    times_s, kappa_s, speed_s = held_samples(run)
    reads = [x for x in run.index["reads"] if x["result"] == "ok" and float(x["read_s"]) > built_s + TIME_TOL_S]
    reads.sort(key=lambda x: float(x["read_s"]))
    k_read = 0
    history: list = []
    first_stop = None
    pallet_moves = []
    pallet0 = None
    ticks = 0
    truth_prev = None
    cfg = check.config
    for row in run.control:
        t = float(row[0])
        if t < built_s - TIME_TOL_S:
            continue
        phase = run.phase_at(t)
        if phase not in PHASES:
            if ticks:
                break
            continue
        ticks += 1
        rear = (float(row[1]), float(row[2]), float(row[3]))
        i_truth = run.truth_index(t)
        truth_rear = rear_pose(np.asarray(run.truth["base_pose"][i_truth], dtype=float), offset)
        pallet = np.asarray(run.truth["pallet_pose"][i_truth], dtype=float)
        if pallet0 is None:
            pallet0 = pallet
        pallet_moves.append(float(np.linalg.norm(pallet[:3] - pallet0[:3])))
        # Frames read after an earlier tick's step are the check's now.
        while k_read < len(reads) and float(reads[k_read]["read_s"]) <= t + TIME_TOL_S:
            rd = reads[k_read]
            k_read += 1
            stamp = float(rd["stamp_s"])
            if stamp > float(rd["read_s"]) + TIME_TOL_S or stamp <= check.last_new_s:
                continue
            times = np.array([h[0] for h in history])
            if len(history) > 1:
                k = int(np.clip(np.searchsorted(times, stamp), 1, len(history) - 1))
                (t0, p0), (t1, p1) = history[k - 1], history[k]
                a = float(np.clip((stamp - t0) / (t1 - t0), 0.0, 1.0)) if t1 > t0 else 1.0
                rear_at = tuple(p0[i] + a * (p1[i] - p0[i]) for i in range(3))
            else:
                rear_at = history[-1][1]
            depth = np.load(run.directory / "pocket_frames" / rd["file"])["depth_m"]
            speed_now = abs(float(rd["odom_speed_mps"]))
            yaw_now = abs(float(rd.get("odom_yaw_rate_rps") or 0.0))
            rec = check.add_frame(stamp, depth, rear_at, speed_mps=speed_now, yaw_rate_rps=yaw_now,
                                  lift_m=float(rd["lift_m"]))
            if rec.get("new"):
                n = len(frames)
                cert_frame[check.memory.certified_s == stamp] = n
                frames.append({"index": n, "time_s": stamp, "source": rd["file"],
                               "rear": [float(v) for v in rear_at], "speed_mps": speed_now,
                               "camera_face_m": camera_face_m(check, rear_at),
                               "erosion_voxels": int(rec.get("erosion_voxels", 0)),
                               "free": int(rec.get("free", 0))})
        # The limit, as the runner asks it (tick inputs held from the 10 Hz samples).
        j = int(np.searchsorted(times_s, t + TIME_TOL_S)) - 1
        kappa = float(kappa_s[j]) if j >= 0 else 0.0
        requested = float(speed_s[j]) if j >= 0 else 0.0
        if truth_prev is None:
            truth_speed = 0.0
        else:
            d = truth_rear[:2] - truth_prev[:2]
            truth_speed = float(np.dot(d, [math.cos(truth_rear[2]), math.sin(truth_rear[2])])) / TICK_S
        truth_prev = truth_rear
        lift = float(run.truth["lift_m"][i_truth])
        speed = max(abs(requested), abs(truth_speed))
        direction = 1 if requested > 0 else (1 if requested == 0 and truth_speed >= 0 else -1)
        allowed, why = check.limit(t, rear, kappa, direction, speed, lift)
        if first_stop is None and why != "ok":
            first_stop = {"time_s": t, "reason": why, "allowed_mps": allowed,
                          "last_fail": getattr(check, "last_fail", None), "frames": len(frames),
                          "phase": phase}
        # The whole stop arc's demand.
        from forklift_core.control.drive_permission import arc_poses

        length = check.real_stop.distance_m(abs(speed)) + cfg.sample_m
        samples, _ = arc_poses(rear, kappa, direction, length, cfg.sample_m)
        for pose in samples:
            for b_index, box in enumerate(check.truck_boxes(check.to_insertion(*pose), lift)):
                idx = demanded(check, box)
                if not len(idx):
                    continue
                new = idx[np.isnan(fd_t[idx[:, 0], idx[:, 1], idx[:, 2]])]
                if not len(new):
                    continue
                ii, jj, kk = new[:, 0], new[:, 1], new[:, 2]
                fd_t[ii, jj, kk] = t
                fd_cert[ii, jj, kk] = check.memory.certified_s[ii, jj, kk]
                fd_frame[ii, jj, kk] = cert_frame[ii, jj, kk]
                fd_code[ii, jj, kk] = check.memory.last_code[ii, jj, kk]
                fd_box[ii, jj, kk] = b_index
                # Two-time displacement for the certified ones.
                centres = vol.lo + (new + 0.5) * vol.voxel_m
                c, s = math.cos(check.axis_yaw), math.sin(check.axis_yaw)
                pts = np.column_stack((check.estimate[0] + centres[:, 0] * c - centres[:, 1] * s,
                                       check.estimate[1] + centres[:, 0] * s + centres[:, 1] * c))
                q = compose_inverse(truth_rear, rear, pts)
                for f in np.unique(fd_frame[ii, jj, kk]):
                    if f < 0:
                        continue
                    sel = fd_frame[ii, jj, kk] == f
                    fr = frames[int(f)]
                    t_c = run.truth_rear(fr["time_s"], offset)
                    if t_c is None:
                        continue
                    p_ = compose_inverse(t_c, fr["rear"], pts[sel])
                    fd_disp[ii[sel], jj[sel], kk[sel]] = np.hypot(*(p_ - q[sel]).T)
        history.append((t, rear))
        del history[:-120]
    arrays = {"fd_t": fd_t, "fd_cert": fd_cert, "fd_frame": fd_frame, "fd_code": fd_code, "fd_box": fd_box,
              "fd_disp": fd_disp, "in_v": vol.in_v, "lo": vol.lo, "voxel_m": np.array(vol.voxel_m)}
    out = {
        "run": str(run.directory),
        "source": str(source),
        "source_modules": hashes,
        "source_matches_run": all(h["matches_run"] for h in hashes.values()),
        "model_files_match_run": model_files,
        "approximations": [
            "held curvature: the last 10 Hz sample's command curvature",
            "requested speed: the last 10 Hz sample's tracker speed",
            "truth speed: one-tick difference of the truth rear axle along its heading",
        ],
        "ticks": ticks,
        "frames": len(frames),
        "reads_used": k_read,
        "pallet_truth_max_move_m": max(pallet_moves) if pallet_moves else None,
        "reproduction": reproduction(r["pocket_check"], first, first_stop),
        "first_stop": json_safe(first_stop),
        "frames_detail": frames,
        "demand": demand_summary(check, arrays, frames, first_stop),
    }
    out["digest_unchanged"] = run_files_digest(run.directory) == run.digest_before
    return out, arrays


def reproduction(recorded: dict, first: dict, first_stop: dict | None) -> dict:
    keys = ("voxels", "free", "in_view", "obstacle", "erosion_voxels")
    rec_first = recorded.get("first_frame") or {}
    out = {"first_frame": {k: {"recorded": rec_first.get(k), "replayed": first.get(k),
                               "same": rec_first.get(k) == first.get(k)} for k in keys}}
    rec_t = recorded.get("last_reason_s")
    out["first_stop_time"] = {"recorded": rec_t, "replayed": None if first_stop is None else first_stop["time_s"]}
    out["first_stop_time"]["same"] = (
        first_stop is not None and rec_t is not None and abs(first_stop["time_s"] - float(rec_t)) <= TIME_TOL_S)
    out["first_stop_reason"] = {"recorded": recorded.get("last_reason"),
                                "replayed": None if first_stop is None else first_stop["reason"]}
    out["first_stop_reason"]["same"] = out["first_stop_reason"]["recorded"] == out["first_stop_reason"]["replayed"]
    rec_fail = recorded.get("last_fail") or {}
    rep_fail = (first_stop or {}).get("last_fail") or {}
    fields = ("box", "arc_s", "pose_insertion", "voxels", "outside_v", "never_certified", "expired",
              "examples_insertion", "examples_last_code")
    out["last_fail"] = {k: {"recorded": rec_fail.get(k), "replayed": json_safe(rep_fail.get(k)),
                            "same": _same(rec_fail.get(k), rep_fail.get(k))} for k in fields}
    out["reproduced"] = (all(v["same"] for v in out["first_frame"].values()) and out["first_stop_time"]["same"]
                         and out["first_stop_reason"]["same"] and all(v["same"] for v in out["last_fail"].values()))
    return out


def _same(a, b, tol: float = 1e-6) -> bool:
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_same(x, y, tol) for x, y in zip(a, b))
    if isinstance(a, (int, float)) and isinstance(b, (int, float)) and not isinstance(a, bool):
        return abs(float(a) - float(b)) <= tol
    return a == b


def demand_summary(check, arrays: dict, frames: list, first_stop: dict | None) -> dict:
    fd_t, fd_cert, fd_frame = arrays["fd_t"], arrays["fd_cert"], arrays["fd_frame"]
    asked = np.isfinite(fd_t)
    in_v = arrays["in_v"]
    certified = asked & np.isfinite(fd_cert)
    age = np.where(certified, fd_t - fd_cert, np.nan)
    lifetime = check.config.lifetime_s
    bins = []
    for lo, hi in zip(AGE_EDGES_S[:-1], AGE_EDGES_S[1:]):
        bins.append({"age_s": [lo, hi], "voxels": int(((age >= lo) & (age < hi)).sum())})
    disp = arrays["fd_disp"]
    face = np.array([f["camera_face_m"] for f in frames])
    cam_at_cert = np.where(fd_frame >= 0, face[np.clip(fd_frame, 0, None)], np.nan)
    expired = certified & (age > lifetime)
    out = {
        "asked_voxels": int(asked.sum()),
        "asked_outside_v": int((asked & ~in_v).sum()),
        "asked_never_certified": int((asked & in_v & ~np.isfinite(fd_cert)).sum()),
        "asked_expired": int(expired.sum()),
        "age_bins": bins,
        "age_s_quantiles": _quantiles(age[certified]),
        "displacement_m_quantiles": _quantiles(disp[certified & np.isfinite(disp)]),
        "camera_face_at_certification_m_quantiles": _quantiles(cam_at_cert[certified]),
        "by_age": [],
    }
    for lo, hi in zip(AGE_EDGES_S[:-1], AGE_EDGES_S[1:]):
        sel = certified & (age >= lo) & (age < hi)
        if sel.any():
            out["by_age"].append({"age_s": [lo, hi], "voxels": int(sel.sum()),
                                  "displacement_m": _quantiles(disp[sel & np.isfinite(disp)]),
                                  "camera_face_m": _quantiles(cam_at_cert[sel])})
    if expired.any():
        idx = np.argwhere(expired)
        centres = arrays["lo"] + (idx + 0.5) * float(arrays["voxel_m"])
        order = np.argsort(fd_t[expired])
        out["expired_examples"] = [
            {"insertion_m": np.round(centres[i], 3).tolist(), "first_demand_s": float(fd_t[tuple(idx[i])]),
             "certified_s": float(fd_cert[tuple(idx[i])]), "age_s": float(age[tuple(idx[i])]),
             "camera_face_at_certification_m": float(cam_at_cert[tuple(idx[i])]),
             "last_code": int(arrays["fd_code"][tuple(idx[i])]), "box": int(arrays["fd_box"][tuple(idx[i])]),
             "displacement_m": float(disp[tuple(idx[i])])}
            for i in order[:40]
        ]
    return out


def _quantiles(values) -> dict | None:
    v = np.asarray(values, dtype=float)
    v = v[np.isfinite(v)]
    if not len(v):
        return None
    q = np.quantile(v, [0.0, 0.5, 0.9, 0.99, 1.0])
    return {"n": int(len(v)), "min": float(q[0]), "median": float(q[1]), "p90": float(q[2]),
            "p99": float(q[3]), "max": float(q[4])}


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--source", type=Path, required=True, help="the snapshot root the run executed")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--arrays", type=Path, help="also save the per-voxel first-demand arrays (.npz)")
    a = parser.parse_args(argv)
    run = Run(a.run)
    out, arrays = replay(run, a.source)
    out["tool_sha256"] = hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(json_safe(out), indent=1) + "\n")
    if a.arrays is not None:
        np.savez_compressed(a.arrays, **arrays)
    rep = out["reproduction"]
    print(f"reproduced={rep['reproduced']} source_matches_run={out['source_matches_run']} "
          f"first_stop={out['first_stop'] and out['first_stop']['time_s']} asked={out['demand']['asked_voxels']} "
          f"expired={out['demand']['asked_expired']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
