"""The far recognition inside run_transport (docs/plans/2026-10-18-isaac-recognition-far-capture.md); no Isaac
imports, so the frame contract, the pose lookup and the stop decision run on CPU tests.

The runner hands it: every physics step of an observe leg (odom<-base, truth base, truth pallet), the driving
frames it renders every ``frame_every`` steps (render-metadata time, pixels, instance ids), and the stop capture.
A driving frame's pixels trail its metadata by ``lag_steps`` physics steps: its pose is the logged step at
metadata time minus the lag. Times are the runner's own (simulation seconds since its initial time).
"""
from __future__ import annotations

import json
import math
import sys
from collections import deque
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from forklift_core.geometry import RigidTransform  # noqa: E402
from forklift_core.perception import recognition_bridge as B  # noqa: E402
from forklift_core.perception.candidate_tracking import TrackBounds  # noqa: E402
from forklift_core.perception.fork_fit import ForkSetup, load_site_defaults  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, spec_from_geometry  # noqa: E402
from forklift_core.perception.scene_dataset import SceneInput  # noqa: E402

SHAPES = ("epal6", "t11_06")


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


def planar(x: float, y: float, yaw: float, *, source="base_link", target="odom") -> RigidTransform:
    c, s = math.cos(yaw), math.sin(yaw)
    return RigidTransform(source, target, ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)), (float(x), float(y), 0.0))


def compose(a, b):
    """(x, y, yaw) of a then b: a's frame <- b's frame composed planar."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], _wrap(a[2] + b[2]))


def relative(base, other):
    """other (x, y, yaw) expressed in base's frame."""
    dx, dy = other[0] - base[0], other[1] - base[1]
    c, s = math.cos(base[2]), math.sin(base[2])
    return (c * dx + s * dy, -s * dx + c * dy, _wrap(other[2] - base[2]))


def known_specs(root: Path = ROOT) -> dict:
    return {k: spec_from_geometry(load_pallet_geometry(root / f"config/pallet_geometry_{k}.yaml"),
                                  entry_axes=KNOWN_ENTRY_AXES[k]) for k in SHAPES}


def fork_spacing_m(blades) -> float:
    """Centre-to-centre distance of the two fork blades (base-frame (x0, x1, y0, y1) boxes)."""
    centres = sorted((y0 + y1) / 2 for _, _, y0, y1 in blades)
    if len(centres) != 2:
        raise ValueError("two fork blades are required")
    return float(centres[1] - centres[0])


class FarRuntime:
    def __init__(self, *, fork_spacing: float, pallet_depth_m: float, pallet_shape: str, zone_centre_xy,
                 zone_radius_m: float, camera_x_m: float, dt: float, frame_every: int = 24, lag_steps: int = 0,
                 drive_frames: bool = True, out_dir: Path | None = None, root: Path = ROOT):
        self.specs = known_specs(root)
        site = load_site_defaults(root / "config/site_load_defaults.yaml", self.specs)
        self.site = {self.specs[k].name: v for k, v in site.items()}
        self.setup = ForkSetup(float(fork_spacing))
        self.bounds = TrackBounds.load(root / "config/recognition_track_bounds.yaml")
        self.far = B.FarRecognition(self.bounds, specs=self.specs, setup=self.setup, load_for=self._load_for)
        self.pallet_depth_m, self.pallet_shape = float(pallet_depth_m), pallet_shape
        self.zone_centre_xy, self.zone_radius_m = tuple(float(v) for v in zone_centre_xy), float(zone_radius_m)
        self.camera_x_m, self.dt = float(camera_x_m), float(dt)
        self.frame_every, self.lag_steps, self.drive_frames = int(frame_every), int(lag_steps), bool(drive_frames)
        self.out_dir = out_dir
        self.paused = False  # set while a stationary capture steps the world
        self.steps: deque = deque(maxlen=max(8 * (self.lag_steps + 1), 256))
        self.leg_steps: list = []   # the observe leg's per-step truth and odom (the pixel-lag fit's input)
        self.leg_frames: list = []  # the leg's driving frames (time, measured pallet centre in base)
        self.frame_records: list = []
        self.attempts: list = []
        self.leg = 0

    def _load_for(self, spec):
        return None if spec is None else self.site.get(spec.name)

    # --- the observe leg's steps and frames ---------------------------------------------------------------
    def due(self, tick: int, *, observing: bool) -> bool:
        return bool(self.drive_frames and observing and not self.paused and tick % self.frame_every == 0)

    def record_step(self, tick: int, stamp_s: float, odom_base, truth_base, truth_pallet) -> None:
        row = (int(tick), float(stamp_s), tuple(map(float, odom_base)), tuple(map(float, truth_base)),
               tuple(map(float, truth_pallet)))
        self.steps.append(row)
        self.leg_steps.append(row)

    def pose_at(self, t_s: float):
        """The logged step nearest t_s (within half a step), or None."""
        best = min(self.steps, key=lambda r: abs(r[1] - t_s), default=None)
        if best is None or abs(best[1] - t_s) > 0.5 * self.dt + 1e-9:
            return None
        return best

    def instruction(self, map_from_base) -> dict:
        return B.instruction_from_zone(self.zone_centre_xy, self.zone_radius_m, map_from_base)

    def drive_frame(self, tick: int, t_meta_s: float, rgb, depth_m, instance_ids, intrinsics, base_from_optical,
                    map_from_odom=(0.0, 0.0, 0.0)) -> dict:
        """One driving frame: pose at the pixels' time, the decision and the tracker. Returns its record."""
        pixel_t = float(t_meta_s) - self.lag_steps * self.dt
        row = self.pose_at(pixel_t)
        record = {"leg": self.leg, "kind": "drive", "tick": int(tick), "t_meta_s": float(t_meta_s), "pixel_t_s": pixel_t}
        if row is None or instance_ids is None:
            record["skipped"] = "no_pose" if row is None else "semantic_not_ready"
            self.frame_records.append(record)
            return record
        scene = SceneInput(np.asarray(rgb, dtype=np.uint8), np.asarray(depth_m, dtype=float), intrinsics,
                           base_from_optical, int(round(row[1] * 1e9)), "synthetic", "synthetic")
        odom = row[2]
        rec = self.far.frame(scene, instance_ids, planar(*odom), self.instruction(compose(map_from_odom, odom)))
        record.update(pose_t_s=row[1], odom=list(odom), **{k: rec[k] for k in ("decision", "taken_by", "visible",
                                                                              "untracked", "held", "new")})
        self.leg_frames.append({"t_s": row[1], "tick": int(tick), "candidates": rec["candidates"]})
        record["candidates"] = len(rec["candidates"])
        record["selected_track"] = self.far.tracker.selected
        self.frame_records.append(record)
        self._save(f"frame_{len(self.frame_records):04d}.npz", kind="drive", leg=self.leg,
                   depth_m=np.asarray(depth_m, dtype=float), instance_ids=np.asarray(instance_ids, dtype=np.int16),
                   stamp_ns=scene.stamp_ns, odom=np.asarray(odom), map_from_odom=np.asarray(map_from_odom, float),
                   **self._intrinsics_arrays(intrinsics, base_from_optical))
        return record

    # --- the stop ---------------------------------------------------------------------------------------
    def truth_face(self, truth_base, truth_pallet, used_yaw: float, *, support_z_m: float = 0.0):
        """The truth face of the run's pallet in the true base frame, the one facing the camera whose insertion
        yaw is nearest the used reading's (the CPU-rig truth definition, tools.recognition_truth.face_poses)."""
        from tools import recognition_truth as rt  # noqa: PLC0415

        x, y, yaw = relative(truth_base, truth_pallet)
        spec = self.specs[self.pallet_shape]
        faces = rt.face_poses(spec, rt.Placement(x, y, yaw, support_z_m), self.setup, self.site.get(spec.name))
        camera = np.array([self.camera_x_m, 0.0])

        def facing(f):
            a = np.array([math.cos(f[2]), math.sin(f[2])])
            return float(-a @ (camera - np.asarray(f[7][:2]))) > 0

        faces = [f for f in faces if facing(f)] or list(faces)
        f = min(faces, key=lambda f: abs(_wrap(f[2] - used_yaw)))
        return {"left_m": f[3], "right_m": f[4], "insertion_yaw_rad": f[2], "axis": f[0], "sign": f[1]}

    def stop(self, scene_input, instance_ids, *, initial_time_s: float, control_base, odom_base, truth_base,
             truth_pallet, detector_observation=None):
        """The stop capture's decision: (PocketObservation for the mission, record). Starts the next leg."""
        rel_ns = int(scene_input.stamp_ns) - int(round(initial_time_s * 1e9))
        scene = replace(scene_input, stamp_ns=rel_ns)
        ids = instance_ids if instance_ids is not None else np.full(np.asarray(scene.depth_m).shape, -1)
        self.frame_records.append({"leg": self.leg, "kind": "stop", "stamp_ns": rel_ns})
        self._save(f"frame_{len(self.frame_records):04d}.npz", kind="stop", leg=self.leg,
                   depth_m=np.asarray(scene.depth_m, dtype=float), instance_ids=np.asarray(ids, dtype=np.int16),
                   stamp_ns=rel_ns, odom=np.asarray(odom_base, float), control_base=np.asarray(control_base, float),
                   **self._intrinsics_arrays(scene.intrinsics, scene.base_from_optical))
        obs, rec = self.far.stop(scene, ids, planar(*odom_base), self.instruction(control_base),
                                 pallet_depth_m=self.pallet_depth_m)
        obs = replace(obs, stamp_ns=int(scene_input.stamp_ns), clock_domain=scene_input.clock_domain)
        rec = {k: v for k, v in rec.items() if k != "stop"}
        rec["leg"], rec["drive_frames"] = self.leg, sum(1 for r in self.frame_records if r["leg"] == self.leg
                                                        and "skipped" not in r and r.get("kind") != "stop")
        used_yaw = rec["used"]["insertion_yaw_rad"] if rec.get("used") else 0.0
        face = self.truth_face(truth_base, truth_pallet, used_yaw)
        rec["truth_face"] = {k: (list(v) if isinstance(v, tuple) else v) for k, v in face.items()}
        for key in ("used", "single"):
            if rec.get(key):
                rec[f"{key}_error"] = B.far_error(_view(rec[key]), face, camera_x_m=self.camera_x_m)
        if detector_observation is not None and detector_observation.status == "valid":
            rec["detector_error"] = B.far_error(_observation_view(detector_observation), face,
                                                camera_x_m=self.camera_x_m)
        rec["lag_fit"] = self.lag_fit(truth_base_at_stop=truth_base)
        self.attempts.append(rec)
        self._save(f"leg_{self.leg:02d}_steps.npz", steps=np.array([(r[0], r[1], *r[2], *r[3], *r[4])
                                                                  for r in self.leg_steps], dtype=float))
        self.next_leg()
        return obs, rec

    def next_leg(self) -> None:
        self.far.start_leg()
        self.leg += 1
        self.leg_steps, self.leg_frames = [], []

    # --- the batch pixel lag ------------------------------------------------------------------------------
    def lag_fit(self, *, truth_base_at_stop=None, max_lag: int = 48) -> dict | None:
        """The pixel lag over this leg's frames: the measured pallet centre's forward distance (base x of the
        oracle instance 0 candidate) against the truth one at each logged step (plan v2, 4)."""
        if len(self.leg_frames) < 6 or not self.leg_steps:
            return None
        ticks = np.array([r[0] for r in self.leg_steps])
        truth = np.array([relative(r[3], r[4])[0] for r in self.leg_steps])
        steps, measured = [], []
        for f in self.leg_frames:
            c = next((c for c in f["candidates"] if c["proposal_id"] == "i0"), None)
            if c is None:
                continue
            k = int(np.searchsorted(ticks, f["tick"]))
            if k >= len(ticks) or ticks[k] != f["tick"]:
                continue
            steps.append(k)
            measured.append(c["pallet_centre_m"][0])
        if len(steps) < 6:
            return None
        return B.estimate_lag(steps, measured, truth, max_lag=max_lag)

    @staticmethod
    def _intrinsics_arrays(intrinsics, base_from_optical) -> dict:
        k = intrinsics
        return {"intrinsics": np.array([k.width, k.height, k.fx, k.fy, k.cx, k.cy], float),
                "intrinsics_frame": k.frame_id, "mount_rotation": np.asarray(base_from_optical.rotation, float),
                "mount_translation": np.asarray(base_from_optical.translation_m, float)}

    def _save(self, name: str, **arrays) -> None:
        if self.out_dir is None:
            return
        self.out_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(self.out_dir / name, **arrays)

    def summary(self) -> dict:
        return {"frames": self.frame_records, "attempts": self.attempts, "lag_steps": self.lag_steps,
                "frame_every": self.frame_every, "drive_frames": self.drive_frames,
                "zone": {"centre_xy": list(self.zone_centre_xy), "radius_m": self.zone_radius_m},
                "camera_x_m": self.camera_x_m, "fork_spacing_m": self.setup.spacing_m,
                "pallet_depth_m": self.pallet_depth_m, "pallet_shape": self.pallet_shape}


class _P:
    def __init__(self, centre):
        self.centre_m = tuple(centre)


def _view(c: dict):
    from types import SimpleNamespace  # noqa: PLC0415

    return SimpleNamespace(left=_P(c["left_m"]), right=_P(c["right_m"]), insertion_yaw_rad=c["insertion_yaw_rad"])


def _observation_view(obs):
    from types import SimpleNamespace  # noqa: PLC0415

    return SimpleNamespace(left=_P(obs.left.center_m), right=_P(obs.right.center_m),
                           insertion_yaw_rad=obs.insertion_yaw_rad)


def dumps(record) -> str:
    return json.dumps(record, default=float)
