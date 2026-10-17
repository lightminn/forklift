"""D5 v11 check 1: replay a recorded near-tracking run (D5 off) through the v11 pocket check.

docs/plans/2026-10-13-d5-v11-pallet-referenced-evidence.md, check 1. For each episode of a
--near-tracking --record-pocket-frames run the v11 check (sim/isaac/pocket_check_v11.py) is
built anew; its first frame is the near capture placed by the tracker's anchor observation
(V1), then every recorded near read is fed in the runner's order at its read time -- accepted
frames through their own observation (moved from the held frame to base_link by the frame's
pose), the others as obstacle checks placed by the carried estimate -- and the store is dropped
on the tracker's mismatch / unbounded / lost events (V6); a loss-injection run is checked only up
to its injection point (the tracker's drop latch). At every approach/insert tick the check's
limit is asked with the rear axle placed in the pallet frame of the latest accepted observation
and the true speed is compared: a tick blocks when the v11 limit is below the larger of the true
speed at that tick and the next. This is an offline admissibility check of the recorded
trajectory (the motion v11 would change is not simulated; curvature and speed come from the
truth record's applied command; a true speed below STANDING_MPS counts as standing). Reports per run and episode: ticks checked, accepted frames,
blocks with their first time, reason and failing box.

Usage: python tools/d5_v11_replay.py --runs RUN_DIR [RUN_DIR ...] --output OUT.json
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))

import insertion_geometry as IG  # noqa: E402
import pocket_check_v11 as V11  # noqa: E402

from forklift_core.control.drive_permission import StoppingModel  # noqa: E402
from forklift_core.perception.near_field_bounds import NearFieldBounds  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.perception.pocket_clearance import DepthCamera  # noqa: E402

TICK_S = 1.0 / 120
STANDING_MPS = 0.005  # a true speed below this is standing (physics jitter), not a motion to permit


def wrap(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def base_observation(held: dict, pose) -> SimpleNamespace:
    """A held-frame observation (pockets + yaw) back in base_link at the held pose."""
    c, s = math.cos(pose[2]), math.sin(pose[2])

    def back(p):
        dx, dy = p[0] - pose[0], p[1] - pose[1]
        return (dx * c + dy * s, -dx * s + dy * c, p[2])

    return SimpleNamespace(left=SimpleNamespace(center_m=back(held["left"])),
                           right=SimpleNamespace(center_m=back(held["right"])),
                           insertion_yaw_rad=wrap(held["yaw"] - pose[2]))


def held_from_observation(observation, pose) -> dict:
    """The anchor observation (base_link at the held pose) placed in the held frame."""
    c, s = math.cos(pose[2]), math.sin(pose[2])

    def place(p):
        return (pose[0] + c * p[0] - s * p[1], pose[1] + s * p[0] + c * p[1], p[2])

    return {"left": place(observation["left"]["center_m"]), "right": place(observation["right"]["center_m"]),
            "yaw": wrap(observation["insertion_yaw_rad"] + pose[2])}


def pallet_in_held(held: dict):
    """The pallet frame (origin between the pocket front centres, x along the insertion yaw) in the held frame."""
    return ((held["left"][0] + held["right"][0]) / 2, (held["left"][1] + held["right"][1]) / 2, float(held["yaw"]))


class Run:
    def __init__(self, directory: Path):
        self.directory = directory
        self.result = json.loads((directory / "result.json").read_text())
        self.near = json.loads((directory / "near_tracking.json").read_text())
        self.index = json.loads((directory / "pocket_frames" / "index.json").read_text())
        data = np.load(directory / "pocket_frames" / "truth.npz")
        self.truth = {k: data[k] for k in data.files}
        self.t = np.asarray(self.truth["stamp_s"], dtype=float)
        self.reads = {round(float(r["stamp_s"]), 6): r for r in self.index["reads"]}
        b = np.asarray(self.truth["base_pose"], dtype=float)
        v = np.zeros(len(self.t))
        along = np.zeros(len(self.t))
        if len(self.t) > 1:
            dt = np.maximum(np.diff(self.t), 1e-9)
            v[1:] = np.hypot(np.diff(b[:, 0]), np.diff(b[:, 1])) / dt
            w, qx, qy, qz = b[:-1, 3], b[:-1, 4], b[:-1, 5], b[:-1, 6]  # x y z qw qx qy qz
            yaw = np.arctan2(2 * (w * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
            along[1:] = (np.diff(b[:, 0]) * np.cos(yaw) + np.diff(b[:, 1]) * np.sin(yaw)) / dt
        self.true_speed = v
        self.true_along = along  # signed speed along the heading (+ forward)

    def i(self, t: float) -> int:
        return int(np.clip(np.searchsorted(self.t, t - 1e-9), 0, len(self.t) - 1))

    def phase(self, t: float) -> str:
        return str(self.truth["phases"][int(self.truth["phase"][self.i(t)])])

    def depth(self, stamp_s: float):
        rec = self.reads.get(round(float(stamp_s), 6))
        if rec is None:
            return None
        return np.load(self.directory / "pocket_frames" / rec["file"])["depth_m"].astype(float)

    def command_zero(self, t0: float, t1: float) -> bool:
        """The applied command was exactly zero over [t0, t1]. A record at t_k holds the command of
        the step that ended at t_k (the runner records after the physics step), so the records read
        run from the first at or after t0 to the first at or after t1 (the step containing t1);
        a span starting before the record, or ending after it, is not known to be zero."""
        if not len(self.t) or t0 < self.t[0] - 1e-9 or t1 > self.t[-1] + 1e-9:
            return False
        lo = int(np.searchsorted(self.t, t0 - 1e-9))
        hi = int(np.searchsorted(self.t, t1 - 1e-9)) + 1
        v = np.asarray(self.truth["applied_speed_mps"][lo:hi], dtype=float)
        return bool(len(v)) and bool(np.all(v == 0.0))

    def standing_motion(self, t0: float, t1: float, rear_axle_offset_m: float) -> tuple[float, float]:
        """The truth's largest rear-axle position and heading change between any two instants in
        [t0, t1] (V13's d0 / phi0 definition, checked on this run): the records bracketing both
        ends are included, so an end between two records (interpolated) is covered; a span off
        the record or a non-finite pose is unbounded (inf)."""
        if not len(self.t) or t0 < self.t[0] - 1e-9 or t1 > self.t[-1] + 1e-9:
            return math.inf, math.inf
        lo = max(int(np.searchsorted(self.t, t0 + 1e-9)) - 1, 0)  # the record at or before t0
        hi = int(np.searchsorted(self.t, t1 - 1e-9)) + 1  # through the record at or after t1
        seg = np.asarray(self.truth["base_pose"][lo:hi], dtype=float)
        if not len(seg) or not np.all(np.isfinite(seg)):
            return math.inf, math.inf
        w, x, y, z = seg[:, 3], seg[:, 4], seg[:, 5], seg[:, 6]
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        dyaw = yaw[:, None] - yaw[None, :]
        phi = float(np.max(np.abs(np.arctan2(np.sin(dyaw), np.cos(dyaw)))))
        # base_link positions interpolate linearly between records (the bracketing records bound
        # any instant); the rear axle moves by at most that plus offset * the heading change.
        d_base = float(np.max(np.hypot(seg[:, None, 0] - seg[None, :, 0], seg[:, None, 1] - seg[None, :, 1])))
        return d_base + abs(rear_axle_offset_m) * phi, phi

    def window_motion(self, stamp_s: float, l_lo: float, l_hi: float, wheelbase: float) -> tuple[float, float]:
        """(v̄, ω̄): the applied command's largest |v| and |v·κ| over the pixel-time window (V2)."""
        lo, hi = self.i(stamp_s - l_hi), self.i(stamp_s - l_lo)
        v = np.abs(np.asarray(self.truth["applied_speed_mps"][lo:hi + 1], dtype=float))
        st = np.asarray(self.truth["applied_steering_rad"][lo:hi + 1], dtype=float)
        if not (np.all(np.isfinite(v)) and np.all(np.isfinite(st))):
            return math.nan, math.nan  # the window cannot bound the motion
        kappa = np.abs(np.tan(st.mean(axis=1))) / wheelbase if len(st) else np.zeros(0)
        return (float(v.max()) if len(v) else 0.0, float((v * kappa).max()) if len(v) else 0.0)


def build(run: Run, bounds: NearFieldBounds, band, root: Path, config: dict | None = None):
    r = run.result
    args, geom = r["arguments"], r["geometry"]
    offset = abs(float(args["rear_axle_offset_m"]))
    layer = yaml.safe_load((root / args["obstacle_layer"]).read_text())["permission"]
    k = r["pocket_camera"]["intrinsics"]
    bfo = r["perception"]["base_from_optical"]
    config = dict(config or {})
    body_front = float(geom["axle_to_fork_tip_m"]) - float(geom["carriage_limit_m"])
    if "carriage_depth_m" not in config:
        # V14 (plan 6판): the chassis/carriage split read from the run's URDF (refused when the
        # boxes do not cover its collision shapes).
        corners = IG.read_carriage_front_corners_m(root / args["forklift_urdf"])
        split = V11.truck_split_from_urdf(
            root / args["forklift_urdf"], offset, float(geom["unloaded_footprint"]["rear_m"]),
            float(geom["unloaded_footprint"]["half_width_m"]), body_front, max(abs(c[1]) for c in corners))
        config.update(carriage_depth_m=split["carriage_depth_m"], carriage_half_width_m=split["carriage_half_width_m"])
    return V11.PalletPocketCheck(
        pallet_geometry=load_pallet_geometry(root / args["pallet_geometry"]),
        blades_rear=tuple((x0 + offset, x1 + offset, y0, y1)
                          for x0, x1, y0, y1 in IG.read_fork_blades_m(root / args["forklift_urdf"])),
        body_front_m=body_front,
        body_rear_m=float(geom["unloaded_footprint"]["rear_m"]),
        body_half_width_m=float(geom["unloaded_footprint"]["half_width_m"]),
        insertion_depth_m=float(geom["axle_to_fork_tip_m"]) + float(geom["pallet_depth_m"]) / 2
        - float(geom["inserted_offset_m"]),
        base_z_m=float(run.truth["base_pose"][0][2]),
        base_from_optical=(np.asarray(bfo["rotation"], dtype=float), np.asarray(bfo["translation_m"], dtype=float)),
        rear_axle_offset_m=offset,
        camera=DepthCamera(float(k["fx"]), float(k["fy"]), float(k["cx"]), float(k["cy"]), int(k["width"]),
                           int(k["height"])),
        stopping=StoppingModel(float(layer["latency_s"]), float(layer["decel_mps2"]), float(layer["margin_m"])),
        bounds=bounds, latency_band_s=band, noise_seed=int(args["seed"]), config=V11.V11Config(**config),
    ), offset


def _finite(*values) -> bool:
    try:
        return all(math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError):
        return False


def record_problems(run: Run) -> list:
    """What in the records cannot carry the replay: non-finite or unordered times, non-finite start
    poses and anchor observations. Checked once before anything is fed (check 1: such a run is
    unverified, never a pass)."""
    out = []
    t = run.t
    if not (len(t) and np.all(np.isfinite(t)) and np.all(np.diff(t) > 0)):
        out.append("truth stamps not finite and increasing")
    nt = run.result.get("near_tracking") or {}
    starts = nt.get("starts") or ([nt["start"]] if nt.get("start") else [])
    if not starts:
        out.append("no tracker start")
    for s in starts:
        obs = s.get("observation") or {}
        try:
            vals = [s["built_s"], s["capture_stamp_s"], *s["pose"], *obs["left"]["center_m"],
                    *obs["right"]["center_m"], obs["insertion_yaw_rad"]]
        except (KeyError, TypeError):
            out.append(f"episode {s.get('episode')}: start incomplete")
            continue
        if not _finite(*vals):
            out.append(f"episode {s.get('episode')}: start not finite")
    drop = nt.get("drop_latch")
    if drop is not None and not _finite(drop.get("time_s")):
        out.append("drop latch time not finite")
    def times(name, r):
        if name == "frames":  # a skipped read (dropped, not_new, ...) may carry no stamp
            return (r.get("read_s"),) + ((r.get("stamp_s"),) if "skipped" not in r or "stamp_s" in r else ())
        return (r.get("t"),) if name == "ticks" else (r.get("time_s"),)

    for name, rows, keys in (("frames", run.near["frames"], ("read_s",)),
                             ("ticks", run.near["ticks"], ("t",)), ("events", run.near["events"], ("time_s",))):
        for ep in sorted({r.get("episode") for r in rows}, key=str):
            sel = [r for r in rows if r.get("episode") == ep]
            if not all(_finite(*times(name, r)) for r in sel):
                out.append(f"{name} (episode {ep}): non-finite time")
            elif name != "events":
                first = [float(r[keys[0]]) for r in sel]
                if any(b < a for a, b in zip(first, first[1:], strict=False)):
                    out.append(f"{name} (episode {ep}): {keys[0]} out of order")
    return out


def _sha256(path: Path) -> str | None:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None


def run_files_digest(directory: Path) -> str:
    """tools/s4_judge.py run_files_digest: one hash over the run files the judgement reads."""
    names = ["result.json", "near_tracking.json", "paths.json", "slam_control.npy", "pocket_frames/index.json",
             "pocket_frames/truth.npz"]
    names += sorted(p.relative_to(directory).as_posix() for p in (directory / "pocket_frames").glob("frame_*.npz"))
    digest = hashlib.sha256()
    for name in names:
        path = directory / name
        digest.update(name.encode() + b"\0" + (_sha256(path) if path.exists() else "-").encode())
    return digest.hexdigest()


def depth_sha256(depth) -> str:
    """sim/isaac/g2_records.py depth_sha256: the depth array's exact bytes, dtype and shape."""
    array = np.ascontiguousarray(np.asarray(depth))
    digest = hashlib.sha256()
    digest.update(str(array.dtype).encode())
    digest.update(str(array.shape).encode())
    digest.update(array.tobytes())
    return digest.hexdigest()


def input_problems(run: Run, root: Path, bounds_sha: str | None, manifest: dict | None,
                   require_manifest: bool) -> list:
    """The replay contract (plan 6판): the URDF, pallet geometry, near-field bounds, drive terms and
    obstacle layer the replay reads are the files the run used, every near-capture depth is the
    array the runner hashed, and the run files match the batch manifest when one is given."""
    r, out = run.result, []
    args = r.get("arguments") or {}
    nt = r.get("near_tracking") or {}
    for key, sha_key in (("forklift_urdf", "forklift_urdf_sha256"), ("pallet_geometry", "pallet_geometry_sha256")):
        if not args.get(key) or _sha256(root / args[key]) != r.get(sha_key):
            out.append(f"{key} is not the file the run used")
    if bounds_sha is None or bounds_sha != nt.get("bounds_sha256"):
        out.append("near-field bounds are not the file the run used")
    terms_path = root / str(nt.get("drive_terms") or "config/near_field_drive_terms.json")
    if _sha256(terms_path) != nt.get("drive_terms_sha256"):
        out.append("drive terms are not the file the run used")
    else:
        recorded = json.loads(terms_path.read_text())["condition"]["input_files"].get("argument obstacle_layer")
        if not args.get("obstacle_layer") or _sha256(root / args["obstacle_layer"]) != recorded:
            out.append("obstacle layer is not the file the run used")
    attempts = r.get("observation_attempts") or []
    starts = nt.get("starts") or ([nt["start"]] if nt.get("start") else [])
    for s in starts:
        k = int(s.get("attempt", 1))
        path = run.directory / f"perception_capture_{k}_depth_m.npy"
        want = attempts[k - 1].get("depth_sha256") if 0 < k <= len(attempts) else None
        if not path.is_file() or want is None or depth_sha256(np.load(path)) != want:
            out.append(f"near-capture depth of attempt {k} is not the array the runner hashed")
    if manifest is not None:
        want = (manifest.get("runs") or {}).get(run.directory.parent.name)
        if want is None or run_files_digest(run.directory) != want:
            out.append("run files do not match the batch manifest")
    elif require_manifest:
        out.append("no batch manifest")
    return out


def replay(run: Run, bounds: NearFieldBounds, band, root: Path, config: dict | None = None, *,
           bounds_sha: str | None = None, standing=None, manifest: dict | None = None,
           require_manifest: bool = False) -> dict:
    """standing: (d0, φ0) of the bounds file's `standing` field (V13), None turns the anchor
    placement off."""
    problems = record_problems(run) or input_problems(run, root, bounds_sha, manifest, require_manifest)
    if problems:
        return {"run": str(run.directory), "success": run.result.get("success"),
                "failure_reason": run.result.get("failure_reason"), "episodes": [], "status": "unverified",
                "record_problems": problems, "blocked": False, "ticks": 0}
    nt = run.result.get("near_tracking") or {}
    starts = nt.get("starts") or ([nt["start"]] if nt.get("start") else [])
    wheelbase = float(run.result["chassis_model"]["drive_geometry"]["wheelbase_m"])
    l_lo, l_hi = float(band[0]), float(band[1])
    # Frames the tracker read: accepted / rejected, and the ones it skipped as uncovered (V13 may
    # place those by the anchor pose); the other skips (dropped, not_new, ...) reach no one.
    frames = [f for f in run.near["frames"] if f.get("skipped") in (None, "uncovered")]
    ticks = run.near["ticks"]
    # The runner records the terminal events as their own event or as a stop with that reason.
    terminal = ("mismatch", "unbounded", "lost")
    invalid_times = [(float(e["time_s"]), e.get("reason") or e["event"]) for e in run.near["events"]
                     if e["event"] in terminal or (e["event"] == "stop" and e.get("reason") in terminal)]
    # A loss-injection run is checked only up to the injection point (check 1): the tracker drops
    # every frame from the drop latch on.
    drop = (nt.get("drop_latch") or {}).get("time_s")
    cut_s = float(drop) if drop is not None else math.inf
    episodes_out = []
    for start in starts:
        ep = start["episode"]
        check, offset = build(run, bounds, band, root, config)
        anchor_pose = start["pose"]
        held = held_from_observation(start["observation"], anchor_pose)
        attempt = int(start.get("attempt", 1))
        capture = run.directory / f"perception_capture_{attempt}_depth_m.npy"
        latest = None
        out = {"episode": ep, "ticks": 0, "blocks": [], "first_block": None, "frames_fed": 0,
               "standing_refusals": {}, "standing_coverage": [], "nonfinite_ticks": 0, "nonfinite_frames": 0,
               "missing_depth": 0, "anchor_placed": 0, "anchor_refused": {}}
        last_insert = None
        checked_i: set = set()
        if capture.exists():
            obs = base_observation(held, anchor_pose)
            check.add_frame(float(start["capture_stamp_s"]), np.load(capture), obs, dict(bounds.near_capture), 0.0)
            latest = {"held": held, "bound": dict(bounds.near_capture), "stamp": float(start["capture_stamp_s"])}
        else:
            out["missing"] = "near capture depth"
        f_ep = [f for f in frames if f["episode"] == ep]
        t_ep = [t for t in ticks if t["episode"] == ep]
        inv = sorted(t for t, _ in invalid_times if t_ep and t_ep[0]["t"] - 1e-9 <= t <= t_ep[-1]["t"] + 1e-9)
        fi, ii = 0, 0
        first_tick = None
        for tick in t_ep:
            now = float(tick["t"])
            while fi < len(f_ep) and float(f_ep[fi]["read_s"]) <= now + 1e-9:
                f = f_ep[fi]
                fi += 1
                if f.get("skipped") == "uncovered":
                    # V13 (plan 6판): placed by the anchor pose for the obstacle check and freshness
                    # only, while the truck stood under a zero applied command since the anchor.
                    stamp = float(f["stamp_s"])
                    anchor_t = min(float(start["built_s"]), float(start["capture_stamp_s"]) - l_hi)
                    why = ("no_standing_bound" if standing is None
                           else "after_an_observation" if latest is None
                           or latest["stamp"] != float(start["capture_stamp_s"])
                           else "before_the_anchor" if stamp - l_lo <= anchor_t
                           else "moving" if not run.command_zero(anchor_t, stamp - l_lo) else None)
                    if why is None:
                        # The S4-batch check of the empirical bound (plan 6판): a span whose true
                        # motion exceeds d0 / phi0 does not use the rule.
                        d_true, phi_true = run.standing_motion(anchor_t, stamp - l_lo, offset)
                        if not (d_true <= standing[0] + 1e-12 and phi_true <= standing[1] + 1e-12):
                            why = "standing_bound_exceeded"  # NaN / inf never pass
                    if why is not None:
                        out["anchor_refused"][why] = out["anchor_refused"].get(why, 0) + 1
                        continue
                    depth = run.depth(stamp)
                    if depth is None:
                        out["missing_depth"] += 1
                        continue
                    truck = V11.pose_in_frame(pallet_in_held(held), tuple(anchor_pose))
                    rec = check.add_frame(stamp, depth, None, None, float(run.truth["lift_m"][run.i(stamp)]),
                                          truck_pallet=truck, carried=(dict(bounds.near_capture),
                                                                       float(start["capture_stamp_s"])),
                                          standing=tuple(standing))
                    out["anchor_placed"] += int(bool(rec.get("new")) and rec.get("placed", True))
                    continue
                depth = run.depth(f["stamp_s"])
                if depth is None:
                    out["missing_depth"] += 1  # a recorded read without its depth: not reproduced
                    continue
                lift = float(f.get("lift_m") or 0.0)
                held_vals = ([*f["latest_held"]["left"], *f["latest_held"]["right"], f["latest_held"]["yaw"]]
                             if f.get("accepted") else [])
                if not all(math.isfinite(float(v)) for v in (*f["pose"], lift, *held_vals)):
                    out["nonfinite_frames"] += 1  # cannot be placed as recorded: unverified
                    continue
                out["frames_fed"] += 1
                if f.get("accepted"):
                    h = f["latest_held"]
                    obs = base_observation(h, f["pose"])
                    pallet = pallet_in_held(h)
                    rear_p = V11.pose_in_frame(pallet, (f["pose"][0] - offset * math.cos(f["pose"][2]),
                                                        f["pose"][1] - offset * math.sin(f["pose"][2]), f["pose"][2]))
                    rho = math.hypot(abs(rear_p[0]) + check._x_extent, check._y_extent)
                    vbar, wbar = run.window_motion(float(f["stamp_s"]), l_lo, l_hi, wheelbase)
                    if not (math.isfinite(vbar) and math.isfinite(wbar)):
                        out["nonfinite_frames"] += 1
                        continue
                    check.add_frame(float(f["stamp_s"]), depth, obs, f["latest_bound"], lift, motion=(vbar, wbar, rho))
                    latest = {"held": h, "bound": f["latest_bound"], "stamp": float(f["stamp_s"])}
                elif latest is not None:
                    truck = V11.pose_in_frame(pallet_in_held(latest["held"]), f["pose"])
                    check.add_frame(float(f["stamp_s"]), depth, None, None, lift, truck_pallet=truck,
                                    carried=(latest["bound"], latest["stamp"]))
            while ii < len(inv) and inv[ii] <= now + 1e-9:
                check.invalidate("tracker_event", inv[ii])
                ii += 1
            if now >= cut_s:
                out["cut_at_injection_s"] = cut_s
                break
            if run.phase(now) not in ("approach", "insert") or latest is None:
                continue
            base = tick["base"]
            i = run.i(now)
            j = min(i + 1, len(run.t) - 1)
            speed = float(run.truth["applied_speed_mps"][i])
            steer = np.asarray(run.truth["applied_steering_rad"][i], dtype=float)
            lift = float(run.truth["lift_m"][i])
            raw = (*base, *steer, speed, lift, run.true_speed[i], run.true_speed[j], run.true_along[i], run.true_along[j])
            if not all(math.isfinite(float(v)) for v in raw):
                out["nonfinite_ticks"] += 1  # the records cannot carry this tick: unverified, not standing
                continue
            rear_h = (base[0] - offset * math.cos(base[2]), base[1] - offset * math.sin(base[2]), base[2])
            rear_p = V11.pose_in_frame(pallet_in_held(latest["held"]), rear_h)
            kappa = math.tan(float(steer.mean())) / wheelbase
            if not all(math.isfinite(float(v)) for v in (kappa, *rear_p)):
                out["nonfinite_ticks"] += 1
                continue
            need = float(max(run.true_speed[i], run.true_speed[j]))
            moving = run.true_along[j] if abs(run.true_along[j]) >= abs(run.true_along[i]) else run.true_along[i]
            need = need if need >= STANDING_MPS else 0.0
            checked_i.add(i)
            # The stop arc is asked at the speed the truck truly moves (the applied command may
            # be lower) and in the direction it truly moves when the command is zero.
            direction = (1 if speed >= 0 else -1) if abs(speed) > 1e-9 else (1 if moving >= 0 else -1)
            allowed, why = check.limit(now, rear_p, latest["bound"], latest["stamp"], kappa,
                                       direction, max(abs(speed), need), lift)
            out["ticks"] += 1
            first_tick = now if first_tick is None else first_tick
            phase = run.phase(now)
            if phase == "insert":
                full = why == "ok" and math.isinf(allowed)  # the whole stop arc covered
                last_insert = {"t": now, "i": i, "reason": why, "allowed": float(allowed), "need": need,
                               "full_arc": full, "last_fail": None if full else getattr(check, "last_fail", None)}
            # unbounded and obstacle fail at any tick, and so does a coverage failure while
            # standing (check 1: no coverage failure to the insertion end); a standing freshness
            # wait is counted apart -- the motion it would hold is checked at the tick it moves.
            if why in ("pocket_unbounded", "pocket_obstacle") or (need > 0.0 and allowed + 1e-9 < need):
                block = {"t": now, "reason": why, "allowed": float(allowed), "need": need,
                         "phase": phase, "rear_pallet": [round(v, 4) for v in rear_p],
                         "last_fail": getattr(check, "last_fail", None)}
                out["blocks"].append(block)
                if out["first_block"] is None:
                    out["first_block"] = block
            elif why != "ok" and allowed <= 0.0:
                out["standing_refusals"][why] = out["standing_refusals"].get(why, 0) + 1
                if why == "pocket_unverified":
                    out["standing_coverage"].append({"t": now, "phase": phase, "rear_pallet": [round(v, 4) for v in rear_p],
                                                     "last_fail": getattr(check, "last_fail", None)})
        out["summary"] = check.summary()
        out["blocks_count"] = len(out["blocks"])
        out["intervals"] = intervals(out["blocks"], first_tick)
        out["blocks"] = out["blocks"][:20]
        out["standing_coverage_count"] = len(out["standing_coverage"])
        out["standing_coverage"] = out["standing_coverage"][:20]
        # V10's last check: the episode's last insert tick, when the truth leaves insert for the
        # lift (not for a retry or the run's end); it must cover its arc.
        out["final_check"] = None
        if last_insert is not None and "cut_at_injection_s" not in out:
            k = next((k for k in range(last_insert["i"] + 1, len(run.t))
                      if run.phase(float(run.t[k])) != "insert"), None)
            nxt = run.phase(float(run.t[k])) if k is not None else None
            arrival = nxt == "other" and not any(s["episode"] > ep for s in starts)
            out["final_check"] = {"arrival": arrival, "next_phase": nxt,
                                  "tail_unchecked": (k - last_insert["i"] - 1) if k is not None else None,
                                  **{key: v for key, v in last_insert.items() if key != "i"},
                                  "passed": last_insert["full_arc"]}
        # Every truth tick in approach/insert of the episode -- from its build to the next
        # episode's build, the injection cut or the end of the truth -- must have been checked.
        later = [float(s["built_s"]) for s in starts if s["episode"] > ep]
        end_s = min(later + [cut_s, math.inf])
        k0 = int(np.searchsorted(run.t, float(start["built_s"]) - 1e-9))
        out["unchecked_ticks"] = sum(1 for k in range(k0, len(run.t)) if run.t[k] < end_s - 1e-9
                                     and k not in checked_i and run.phase(float(run.t[k])) in ("approach", "insert"))
        accepted = out["summary"].get("accepted_frames", 0)
        fc = out["final_check"]
        if (out["ticks"] == 0 or accepted == 0 or "missing" in out or out["nonfinite_ticks"] or out["nonfinite_frames"]
                or out["missing_depth"]
                or out["unchecked_ticks"] or (fc and fc["arrival"] and fc["tail_unchecked"])):
            out["status"] = "unverified"  # an empty or partly unchecked span is not a pass (check 1)
        elif out["blocks_count"] or out["standing_coverage"] or (out["final_check"] and out["final_check"]["arrival"]
                                                                 and not out["final_check"]["passed"]):
            out["status"] = "blocked"
        else:
            out["status"] = "pass"
        episodes_out.append(out)
    statuses = [e["status"] for e in episodes_out]
    status = ("unverified" if not statuses or "unverified" in statuses
              else "blocked" if "blocked" in statuses else "pass")
    return {"run": str(run.directory), "success": run.result.get("success"),
            "failure_reason": run.result.get("failure_reason"), "episodes": episodes_out, "status": status,
            "blocked": status == "blocked", "ticks": sum(e["ticks"] for e in episodes_out)}


def intervals(blocks: list, first_tick, gap_s: float = 0.05) -> list:
    """Contiguous blocked ticks of one reason: [t0, t1], tick count, where (episode start: within
    0.5 s of the episode's first checked tick; else the phase), the rear at t0 and, for coverage
    failures, the failing box and voxel examples at t0."""
    out = []
    for b in blocks:
        last = out[-1] if out else None
        if last and b["reason"] == last["reason"] and b["t"] - last["t1"] <= gap_s:
            last["t1"], last["ticks"] = b["t"], last["ticks"] + 1
            continue
        lf = b.get("last_fail") or {}
        where = "episode_start" if first_tick is not None and b["t"] - first_tick <= 0.5 else b["phase"]
        out.append({"reason": b["reason"], "t0": b["t"], "t1": b["t"], "ticks": 1, "where": where,
                    "rear0": b["rear_pallet"], "box": lf.get("box"), "examples": lf.get("examples"),
                    "codes": lf.get("examples_code"), "e_d": lf.get("e_d")})
    return out


def json_safe(value):
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--runs", nargs="+", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--bounds", type=Path, default=ROOT / "config/near_field_bounds_measured.json")
    ap.add_argument("--config", type=json.loads, default=json.loads(os.environ.get("D5V11_CONFIG") or "{}"),
                    help="V11Config overrides as JSON (default: $D5V11_CONFIG)")
    ap.add_argument("--standing", type=json.loads, default=None,
                    help="[d0_m, phi0_rad] instead of the bounds file's standing field (diagnostics only)")
    ap.add_argument("--manifest", type=Path, default=None, help="the batch manifest.json (run files digests)")
    ap.add_argument("--require-manifest", action="store_true", help="a run without a manifest is unverified")
    ns = ap.parse_args(argv)
    data = json.loads(ns.bounds.read_text())
    bounds = NearFieldBounds.from_dict(data)
    band = tuple(data["latency"]["band_s"])
    field = data.get("standing")
    standing = ns.standing if ns.standing is not None else (
        (float(field["d0_m"]), float(field["phi0_rad"])) if field else None)
    manifest = json.loads(ns.manifest.read_text()) if ns.manifest else None
    bounds_sha = hashlib.sha256(ns.bounds.read_bytes()).hexdigest()
    out = []
    for d in ns.runs:
        try:
            out.append(replay(Run(d), bounds, band, ROOT, ns.config, bounds_sha=bounds_sha, standing=standing,
                              manifest=manifest, require_manifest=ns.require_manifest))
        except Exception as exc:  # noqa: BLE001
            out.append({"run": str(d), "error": f"{type(exc).__name__}: {exc}", "status": "unverified"})
    ns.output.write_text(json.dumps(json_safe({"rules": V11.RULES, "config": ns.config, "standing": standing,
                                               "standing_from": "argument" if ns.standing is not None else
                                               ("bounds" if field else None), "runs": out}), indent=1) + "\n")
    return 1 if any("error" in r for r in out) else 0


if __name__ == "__main__":
    raise SystemExit(main())
