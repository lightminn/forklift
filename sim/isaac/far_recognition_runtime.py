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
import threading
import time
from collections import deque
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from forklift_core.geometry import RigidTransform  # noqa: E402
from forklift_core.perception import recognition_bridge as B  # noqa: E402
from forklift_core.perception.candidate_tracking import TrackBounds  # noqa: E402
from forklift_core.perception.fork_fit import ForkSetup, load_site_defaults  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, VARIANT_OF, spec_from_geometry  # noqa: E402
from forklift_core.perception.scene_dataset import SceneInput  # noqa: E402

SHAPES = ("epal6", "t11_06")


class LearnedSource:
    """FarRecognition's proposer on the learned path (stage 3 plan, task 6): the model's proposals -- the instance
    ids are never read -- or the geometric fallback's on any model failure (design §10). ``last`` keeps what the
    latest frame used, so the run can save it and the recompute can replay it exactly."""

    def __init__(self, model, geometric=None):
        if geometric is None:
            from forklift_core.perception import geometric_proposals as G  # noqa: PLC0415

            geometric = G.propose
        self.model, self.geometric = model, geometric
        self.fallbacks: list = []
        self.times_ms: list = []  # the model's wall time per call, failures included (design §6 budget)
        self.last: dict | None = None

    def run(self, scene) -> dict:
        """One frame's proposals without touching this object's records (a live run calls it on its worker
        thread): the proposals, the failures, which source made them, why it fell back and the model's time."""
        t0 = time.perf_counter()
        try:
            try:
                props, failures, source, reason = list(self.model.propose(scene)), (), "learned", None
            finally:
                ms = 1e3 * (time.perf_counter() - t0)
        except Exception as exc:  # noqa: BLE001 -- any model failure falls back
            from forklift_core import determinism  # noqa: PLC0415

            if determinism.enabled():
                # Deterministic-run plan D6: the switch depends on the machine (a missed wall deadline among the
                # failures); a deterministic run ends here instead of taking the geometric path.
                determinism.mark(f"learned_failure:{type(exc).__name__}")
                raise determinism.DeterministicBroken(f"learned_failure: {exc}") from exc
            reason = f"{type(exc).__name__}: {exc}"
            got = self.geometric(scene)
            props, failures, source = list(got.proposals), tuple(got.failures), "geometric_fallback"
        return {"source": source, "reason": reason, "proposals": props, "failures": failures, "ms": ms}

    def absorb(self, got: dict) -> None:
        """A frame's ``run`` taken into the records (on the tick that applies the frame)."""
        self.times_ms.append(got["ms"])
        if got["source"] == "geometric_fallback":
            self.fallbacks.append(got["reason"])
        self.last = {k: got[k] for k in ("source", "reason", "proposals", "failures")}

    def reset_stats(self) -> None:
        """New lists for a new live mission (a list handed to an earlier mission's record is not cleared)."""
        self.fallbacks, self.times_ms, self.last = [], [], None

    def __call__(self, scene, _instance_ids):
        got = self.run(scene)
        self.absorb(got)
        return got["proposals"], got["failures"]

    def timing(self) -> dict:
        t = np.asarray(self.times_ms, float)
        if not len(t):
            return {"n": 0}
        return {"n": int(len(t)), "p50": float(np.median(t)), "p95": float(np.percentile(t, 95)), "max": float(t.max()),
                "first": float(t[0]), "max_after_first": float(t[1:].max()) if len(t) > 1 else None}

    def saved(self) -> dict:
        """The latest frame's proposals as npz arrays."""
        last = self.last
        props = last["proposals"]
        shape = props[0].mask.shape if props else (0, 0)
        return {"proposal_masks": np.stack([p.mask for p in props]) if props else np.zeros((0, *shape), bool),
                "proposal_scores": np.array([p.score for p in props], float),
                "proposal_ids": np.array([p.proposal_id for p in props], dtype=str),
                "proposal_sources": np.array([p.source for p in props], dtype=str),
                "proposer_source": last["source"], "proposer_reason": last["reason"] or "",
                "extra_failures": json.dumps([[f.proposal_id, f.reason, list(f.centre_xy) if f.centre_xy else None,
                                               f.extent_m] for f in last["failures"]])}


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


def known_specs(root: Path = ROOT, run_shape: str | None = None) -> dict:
    """The known shapes the far recognition fits. A run whose pallet is a test-article variant (VARIANT_OF) has
    the variant in its parent's place, under its own key: the face cannot tell them apart, the run's instruction
    can (plan 2026-10-22)."""
    keys = [run_shape if VARIANT_OF.get(run_shape) == k else k for k in SHAPES]
    return {k: spec_from_geometry(load_pallet_geometry(root / f"config/pallet_geometry_{k}.yaml"),
                                  entry_axes=KNOWN_ENTRY_AXES[k]) for k in keys}


def fork_spacing_m(blades) -> float:
    """Centre-to-centre distance of the two fork blades (base-frame (x0, x1, y0, y1) boxes)."""
    centres = sorted((y0 + y1) / 2 for _, _, y0, y1 in blades)
    if len(centres) != 2:
        raise ValueError("two fork blades are required")
    return float(centres[1] - centres[0])


def decode_mission_semantic(payload, shape=(480, 640)):
    """The mission scene's semantic segmentation as instance ids (>= 0 for ``inst_k``, -1 elsewhere).

    Unlike the recognition renders (recognition_render_core.decode_semantic), the warehouse assets carry their
    own class labels (``pillar``, ...), so one id can hold several names: only this run's labels count -- an
    ``inst_k`` wins, ``floor`` marks the background that readiness needs, anything else is ignored. Two
    instance labels on one id are a contract error. Returns (ids or None, ready, status)."""
    from types import SimpleNamespace  # noqa: PLC0415

    def not_ready(status):
        return SimpleNamespace(instance_ids=None, ready=False, status=status)

    if not isinstance(payload, dict) or payload.get("data") is None:
        return not_ready("missing_buffer")
    array = np.asarray(payload["data"])
    if not array.size:
        return not_ready("empty_buffer")
    info = payload.get("info")
    labels = info.get("idToLabels") if isinstance(info, dict) else None
    if not isinstance(labels, dict) or not labels:
        return not_ready("missing_label_map")
    if array.ndim == 3 and array.shape[-1] == 1:
        array = array[..., 0]
    if array.dtype.kind not in "ui":
        raise ValueError("semantic segmentation must be an uncolorized integer grid")
    if array.shape != tuple(shape):
        return not_ready("shape_mismatch")
    ids = np.full(array.shape, -1, dtype=np.int32)
    floor = 0
    for key, value in labels.items():
        classes = value.get("class", []) if isinstance(value, dict) else []
        names = [c for c in classes.split(",") if c] if isinstance(classes, str) else list(classes)
        inst = [n for n in names if n.startswith("inst_") and n[5:].isdigit()]
        if len(inst) > 1:
            raise ValueError(f"semantic id {key} carries several instance labels: {inst}")
        hit = array == int(key)
        if inst:
            ids[hit] = int(inst[0][5:])
        elif "floor" in names:
            floor += int(hit.sum())
    ready = floor > 0
    return SimpleNamespace(instance_ids=ids if ready else None, ready=ready, status="ready" if ready else "floor_missing")


class FarRuntime:
    def __init__(self, *, fork_spacing: float, pallet_depth_m: float, pallet_shape: str, zone_centre_xy,
                 zone_radius_m: float, camera_x_m: float, dt: float, frame_every: int = 24, lag_steps: int = 0,
                 drive_frames: bool = True, out_dir: Path | None = None, root: Path = ROOT, proposer=None,
                 identity_threshold=None, proposer_record=None):
        self.specs = known_specs(root, run_shape=pallet_shape)
        site = load_site_defaults(root / "config/site_load_defaults.yaml", self.specs)
        self.site = {self.specs[k].name: v for k, v in site.items()}
        self.setup = ForkSetup(float(fork_spacing))
        self.bounds = TrackBounds.load(root / "config/recognition_track_bounds.yaml")
        self.far = B.FarRecognition(self.bounds, specs=self.specs, setup=self.setup, load_for=self._load_for,
                                    proposer=proposer, identity_threshold=identity_threshold)
        self.proposer_record = proposer_record
        self.source = proposer if isinstance(proposer, LearnedSource) else None  # saves rgb and proposals
        self.pallet_depth_m, self.pallet_shape = float(pallet_depth_m), pallet_shape
        self.zone_centre_xy, self.zone_radius_m = tuple(float(v) for v in zone_centre_xy), float(zone_radius_m)
        self.zone_initial = {"centre_xy": list(self.zone_centre_xy), "radius_m": self.zone_radius_m}
        self.zone_changes: list = []  # [{"leg", "centre_xy", "radius_m"}]: the zone from that leg on
        self.camera_x_m, self.dt = float(camera_x_m), float(dt)
        self.frame_every, self.lag_steps, self.drive_frames = int(frame_every), int(lag_steps), bool(drive_frames)
        self.out_dir = out_dir
        self.paused = False  # set while a stationary capture steps the world
        self.truth_instance = 0  # the run's pallet among the instance ids (a session's mission pallet, its own)
        self.stop_pending = False  # a live stop decision waits for its work: no driving frame meanwhile
        self.writer = None  # a live run's file writer, writer(path, arrays) (the control thread never writes)
        self.steps: deque = deque(maxlen=max(8 * (self.lag_steps + 1), 256))
        self.leg_steps: list = []   # the observe leg's per-step truth and odom (the pixel-lag fit's input)
        self.leg_frames: list = []  # the leg's driving frames (time, measured pallet centre in base)
        self.frame_records: list = []
        self.attempts: list = []
        self.leg_endings: list = []  # legs ended by a target lost while driving (A')
        self.leg = 0

    def _load_for(self, spec):
        return None if spec is None else self.site.get(spec.name)

    # --- the observe leg's steps and frames ---------------------------------------------------------------
    def due(self, tick: int, *, observing: bool) -> bool:
        return bool(self.drive_frames and observing and not self.paused and not self.stop_pending
                    and tick % self.frame_every == 0)

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

    def reset_for_mission(self, *, zone_centre_xy, zone_radius_m: float, out_dir: Path | None) -> None:
        """A live mission's start (plan 2026-10-22-live-far-recognition task 7): a new first leg (no track, target
        or selection of an earlier mission), the mission's own zone as the initial one, its own record directory
        and records, fresh learned statistics -- new lists, the earlier mission's record keeps its own."""
        self.far.start_leg()
        self.leg = 0
        self.leg_steps, self.leg_frames = [], []
        self.frame_records, self.attempts, self.leg_endings, self.zone_changes = [], [], [], []
        self.zone_centre_xy, self.zone_radius_m = tuple(float(v) for v in zone_centre_xy), float(zone_radius_m)
        self.zone_initial = {"centre_xy": list(self.zone_centre_xy), "radius_m": self.zone_radius_m}
        self.out_dir = out_dir
        self.paused = self.stop_pending = False
        self.steps.clear()
        if self.source is not None:
            self.source.reset_stats()

    def set_zone(self, centre_xy, radius_m: float) -> None:
        """The instruction's zone from the current leg on (the bottom-deck re-observation: the pallet judged, not the
        pickup zone). Recorded by leg, so the recompute replays each stop with the zone it had."""
        self.zone_centre_xy, self.zone_radius_m = tuple(float(v) for v in centre_xy), float(radius_m)
        self.zone_changes.append({"leg": int(self.leg), "centre_xy": list(self.zone_centre_xy),
                                  "radius_m": self.zone_radius_m})

    def instruction(self, map_from_base) -> dict:
        return B.instruction_from_zone(self.zone_centre_xy, self.zone_radius_m, map_from_base)

    def drive_frame(self, tick: int, t_meta_s: float, rgb, depth_m, instance_ids, intrinsics, base_from_optical,
                    map_from_odom=(0.0, 0.0, 0.0)) -> dict:
        """One driving frame: pose at the pixels' time, the decision and the tracker. Returns its record."""
        job = self.drive_submit(tick, t_meta_s, rgb, depth_m, instance_ids, intrinsics, base_from_optical,
                                map_from_odom)
        return self.drive_apply(job, self.compute(job))

    def drive_submit(self, tick: int, t_meta_s: float, rgb, depth_m, instance_ids, intrinsics, base_from_optical,
                     map_from_odom=(0.0, 0.0, 0.0)) -> dict:
        """A driving frame's part before its work, on the tick it was rendered: the pose at the pixels' time (taken
        now: the step log is short), the scene and the instruction from this leg's zone. A frame without a pose or
        ready pixels is a job without a scene (no work, recorded as skipped when applied)."""
        pixel_t = float(t_meta_s) - self.lag_steps * self.dt
        row = self.pose_at(pixel_t)
        record = {"leg": self.leg, "kind": "drive", "tick": int(tick), "t_meta_s": float(t_meta_s), "pixel_t_s": pixel_t}
        job = {"kind": "drive", "leg": self.leg, "record": record, "scene": None}
        if row is None or instance_ids is None or depth_m is None:
            record["skipped"] = ("no_pose" if row is None else "depth_not_ready" if depth_m is None
                                 else "semantic_not_ready")
            return job
        scene = SceneInput(np.asarray(rgb, dtype=np.uint8), np.asarray(depth_m, dtype=float), intrinsics,
                           base_from_optical, int(round(row[1] * 1e9)), "synthetic", "synthetic")
        odom = row[2]
        job.update(scene=scene, ids=instance_ids, row=row, odom=odom, depth_m=depth_m, intrinsics=intrinsics,
                   base_from_optical=base_from_optical, map_from_odom=map_from_odom,
                   instruction=self.instruction(compose(map_from_odom, odom)))
        return job

    def compute(self, job):
        """A job's pure work (any thread): the proposals and the scene fit; None for a job without a scene."""
        return None if job.get("scene") is None else self.far.compute(job["scene"], job["ids"])

    def drive_apply(self, job, work) -> dict:
        """A driving frame's work taken in, on the tick: the tracker, the leg's target, the records and files."""
        record = job["record"]
        if job["scene"] is None:
            self.frame_records.append(record)
            return record
        scene, row, odom, instance_ids, depth_m = job["scene"], job["row"], job["odom"], job["ids"], job["depth_m"]
        intrinsics, base_from_optical, map_from_odom = job["intrinsics"], job["base_from_optical"], job["map_from_odom"]
        tick = record["tick"]
        history = list(self.far.selection_history)
        rec = self.far.frame(scene, instance_ids, planar(*odom), job["instruction"], work=work)
        record.update(pose_t_s=row[1], odom=list(odom), **{k: rec[k] for k in ("decision", "taken_by", "visible",
                                                                              "untracked", "held", "new")})
        # evaluation only (the lag fit): the proposal that best covers truth instance 0, by IoU >= 0.5 -- never the
        # proposal's id or order (Codex)
        truth = np.asarray(instance_ids) == self.truth_instance
        ious = [((p.mask & truth).sum() / max(1, (p.mask | truth).sum()), p.proposal_id) for p in self.far.last_proposals]
        best = max(ious, default=(0.0, None))
        self.leg_frames.append({"t_s": row[1], "tick": int(tick), "candidates": rec["candidates"],
                                "truth_proposal": best[1] if best[0] >= 0.5 else None})
        record["candidates"] = len(rec["candidates"])
        record["selected_track"] = self.far.tracker.selected
        self.frame_records.append(record)
        record["leg_lost"] = rec["leg_lost"]
        self._save(f"frame_{len(self.frame_records):04d}.npz", kind="drive", leg=self.leg,
                   depth_m=np.asarray(depth_m, dtype=float), instance_ids=np.asarray(instance_ids, dtype=np.int16),
                   stamp_ns=scene.stamp_ns, odom=np.asarray(odom), map_from_odom=np.asarray(map_from_odom, float),
                   **self._intrinsics_arrays(intrinsics, base_from_optical), **self._learned_arrays(scene))
        if rec["leg_lost"]:
            # The leg's target was deleted while driving: the leg ends as a loss and a re-observation leg starts
            # with the next frame -- the waypoint re-observation's contract without driving to another waypoint
            # (A', Codex). Its record is kept; the drive's own progress and limits are untouched.
            self.leg_endings.append({"leg": self.leg, "reason": "target_lost_while_driving", "tick": int(tick),
                                     "selection_history": history, "lag_fit": self.lag_fit(),
                                     "drive_frames": sum(1 for r in self.frame_records if r["leg"] == self.leg
                                                         and r.get("kind") == "drive" and "skipped" not in r)})
            self._save(f"leg_{self.leg:02d}_steps.npz", steps=np.array([(r[0], r[1], *r[2], *r[3], *r[4])
                                                                      for r in self.leg_steps], dtype=float))
            self.next_leg()
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
        job = self.stop_submit(scene_input, instance_ids, initial_time_s=initial_time_s, control_base=control_base,
                               odom_base=odom_base, truth_base=truth_base, truth_pallet=truth_pallet,
                               detector_observation=detector_observation)
        return self.stop_apply(job, self.compute(job))

    def stop_submit(self, scene_input, instance_ids, *, initial_time_s: float, control_base, odom_base, truth_base,
                    truth_pallet, detector_observation=None) -> dict:
        """The stop capture's part before its work: everything it rests on, fixed at the capture."""
        rel_ns = int(scene_input.stamp_ns) - int(round(initial_time_s * 1e9))
        scene = replace(scene_input, stamp_ns=rel_ns)
        ids = instance_ids if instance_ids is not None else np.full(np.asarray(scene.depth_m).shape, -1)
        return {"kind": "stop", "leg": self.leg, "scene": scene, "ids": ids, "scene_input": scene_input,
                "rel_ns": rel_ns, "control_base": control_base, "odom_base": odom_base, "truth_base": truth_base,
                "truth_pallet": truth_pallet, "detector_observation": detector_observation,
                "instruction": self.instruction(control_base)}

    def stop_apply(self, job, work):
        """The stop's work taken in, on the tick: the decision, the records and files, and the next leg."""
        scene, ids, rel_ns, scene_input = job["scene"], job["ids"], job["rel_ns"], job["scene_input"]
        odom_base, control_base = job["odom_base"], job["control_base"]
        truth_base, truth_pallet, detector_observation = job["truth_base"], job["truth_pallet"], job["detector_observation"]
        self.frame_records.append({"leg": self.leg, "kind": "stop", "stamp_ns": rel_ns})
        name = f"frame_{len(self.frame_records):04d}.npz"
        obs, rec = self.far.stop(scene, ids, planar(*odom_base), job["instruction"],
                                 pallet_depth_m=self.pallet_depth_m, work=work)
        self._save(name, kind="stop", leg=self.leg,
                   depth_m=np.asarray(scene.depth_m, dtype=float), instance_ids=np.asarray(ids, dtype=np.int16),
                   stamp_ns=rel_ns, odom=np.asarray(odom_base, float), control_base=np.asarray(control_base, float),
                   **self._intrinsics_arrays(scene.intrinsics, scene.base_from_optical), **self._learned_arrays(scene))
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
            c = next((c for c in f["candidates"] if f.get("truth_proposal") is not None
                      and c["proposal_id"] == f["truth_proposal"] and c["geometry"] == "fitted"
                      and c["identity"] == "confirmed"), None)  # counted readings only (Codex)
            if c is None:
                continue
            k = int(np.searchsorted(ticks, f["tick"]))
            if k >= len(ticks) or ticks[k] != f["tick"]:
                continue
            steps.append(k)
            measured.append(c["pallet_centre_m"][0])
        if len(steps) < 6:
            return None
        fit = B.estimate_lag(steps, measured, truth, max_lag=max_lag)
        # J2 (Codex): every segment that identifies tau on its own (the whole leg, each half) must agree with the lag
        # the run used within one step; only when no segment identifies anything is there no judgement (None).
        taus = [t for t, ok in zip([fit["tau_steps"], *fit["halves"]], fit["identified"]) if ok]
        fit["agrees_with_used"] = None if not taus else all(abs(t - self.lag_steps) <= 1 for t in taus)
        return fit

    @staticmethod
    def _intrinsics_arrays(intrinsics, base_from_optical) -> dict:
        k = intrinsics
        return {"intrinsics": np.array([k.width, k.height, k.fx, k.fy, k.cx, k.cy], float),
                "intrinsics_frame": k.frame_id, "mount_rotation": np.asarray(base_from_optical.rotation, float),
                "mount_translation": np.asarray(base_from_optical.translation_m, float)}

    def _learned_arrays(self, scene) -> dict:
        """The learned path's frame extras: the RGB the model read and the proposals the decision used."""
        if self.source is None:
            return {}
        return {"rgb": np.asarray(scene.rgb, dtype=np.uint8), **self.source.saved()}

    def _save(self, name: str, **arrays) -> None:
        if self.out_dir is None:
            return
        if self.writer is not None:
            self.writer(self.out_dir / name, arrays)
            return
        self.out_dir.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(self.out_dir / name, **arrays)

    def summary(self) -> dict:
        return {"frames": self.frame_records, "attempts": self.attempts, "leg_endings": self.leg_endings,
                "lag_steps": self.lag_steps,
                "frame_every": self.frame_every, "drive_frames": self.drive_frames,
                "zone": dict(self.zone_initial), "zone_changes": list(self.zone_changes),
                "camera_x_m": self.camera_x_m, "fork_spacing_m": self.setup.spacing_m,
                "pallet_depth_m": self.pallet_depth_m, "pallet_shape": self.pallet_shape,
                "proposer": {**(self.proposer_record or {"kind": "oracle"}),
                             **({"fallbacks": len(self.source.fallbacks), "fallback_reasons": self.source.fallbacks[:5],
                                 "model_ms": self.source.timing()} if self.source is not None else {})}}



# --- the live run (docs/plans/2026-10-22-live-far-recognition.md, decision T1 (A)) ---------------------------------
STOP_WAIT_S = 30.0  # a stop decision's whole wait, monotonic wall time from its hand-over (frames before it included)
QUEUE_CAP = 4  # driving frames waiting or being worked on: 4 x 24 steps = 0.8 sim s, about 2 wall s at factor 0.4


def live_observing(status: str, observing: bool, stopped_in_tick: bool) -> bool:
    """The live gate on the driving frames and the step log (task 6): a running mission in its observe leg, and no
    STOP at this physics step. An idle, paused, cancelling or ended session records nothing."""
    return status == "running" and bool(observing) and not stopped_in_tick


class FarWorker:
    """One thread doing the far frames' pure work (``compute(job)``) in hand-over order. At most ``cap`` driving
    frames wait or run: a new one beyond that drops the oldest one still waiting (``queue_full``); a stop is never
    dropped for room. ``poll`` hands back ``(job, status, payload)`` -- ``done`` with the work, ``error`` with the
    exception, ``dropped`` with why -- in the order they finished or were dropped."""

    def __init__(self, compute, *, cap: int = QUEUE_CAP, name: str = "far-recognition"):
        self.compute, self.cap = compute, int(cap)
        self._cond = threading.Condition()
        self._waiting: deque = deque()
        self._running = None
        self._done: deque = deque()
        self._closing = False
        self._thread = threading.Thread(target=self._loop, name=name, daemon=True)
        self._thread.start()

    def _drives(self) -> int:  # under the lock
        running = self._running is not None and self._running["kind"] == "drive"
        return sum(1 for j in self._waiting if j["kind"] == "drive") + int(running)

    def submit(self, job: dict) -> None:
        with self._cond:
            if job["kind"] == "drive":
                while self._drives() >= self.cap:
                    old = next((j for j in self._waiting if j["kind"] == "drive"), None)
                    if old is None:
                        break
                    self._waiting.remove(old)
                    self._done.append((old, "dropped", "queue_full"))
            self._waiting.append(job)
            self._cond.notify_all()

    def clear(self) -> list:
        """Every job not started yet is dropped (a STOP, a cancel, a generation or a mission change); returned, so
        the caller notes them at once (Codex implementation round 1 P2)."""
        with self._cond:
            out = list(self._waiting)
            self._waiting.clear()
            return out

    def end(self) -> tuple:
        """At a mission's end, in one hold of the lock (Codex implementation round 2 P2): the jobs not started
        (dropped), the finished ones not polled yet, and the one being worked on (None if none)."""
        with self._cond:
            waiting, done = list(self._waiting), list(self._done)
            self._waiting.clear()
            self._done.clear()
            return waiting, done, self._running

    def poll(self) -> list:
        with self._cond:
            out = list(self._done)
            self._done.clear()
            return out

    def idle(self) -> bool:
        with self._cond:
            return not self._waiting and self._running is None

    def close(self, timeout_s: float = 1.0) -> None:
        """Ends the thread once the job it is on returns (one stuck in a model call is a daemon left behind)."""
        with self._cond:
            self._closing = True
            self._waiting.clear()
            self._cond.notify_all()
        self._thread.join(timeout_s)

    def _loop(self) -> None:
        while True:
            with self._cond:
                while not self._waiting:
                    if self._closing:
                        return
                    self._cond.wait()
                job = self._waiting.popleft()
                self._running = job
            try:
                out = ("done", self.compute(job))
            except Exception as exc:  # noqa: BLE001 -- the tick turns it into the mission's failure
                out = ("error", exc)
            with self._cond:
                self._running = None
                self._done.append((job, *out))
                self._cond.notify_all()


class LiveFar:
    """The tick's side of the live far recognition. Frames and stops are handed to the worker tagged with the
    mission generation, their leg and a sequence number; ``pump`` takes the finished ones in on the tick, in order,
    and drops (records, does not apply) those of another generation, an inactive session or an ended leg. The
    tracker, the target, the legs, the records and the learned statistics change only here, on the control thread.

    A stop is a pending decision (``stop`` -> handle): it waits for the leg's earlier frames, the truck standing;
    ``stop_state`` is ``waiting``, ``done`` (``take`` gives the observation and record), or ``invalid`` (``why``:
    the leg ended before it, a STOP or cancel cleared it); waiting past ``STOP_WAIT_S`` raises
    ``recognition_timeout``. A worker failure, or a driving frame's read that raised inside the physics step
    (``read_frame``), raises ``recognition_error:<type>`` from the next ``pump``."""

    def __init__(self, runtime, *, worker=None, clock=time.monotonic, stop_wait_s: float = STOP_WAIT_S, io=None):
        self.rt = runtime
        self.worker = worker if worker is not None else FarWorker(runtime.compute)
        self.clock, self.stop_wait_s, self.io = clock, float(stop_wait_s), io
        self.seq = 0
        self.handle = None
        self.error = None
        self.mission_id = None
        self.mission_dir: Path | None = None  # the runner's: the mission's directory (its result.json)
        self.setup_zone = None  # the runner's: the scene's zone, for a prior pallet at its first place
        self.log: list = []  # this mission's jobs: applied (with their order) or dropped (with why)
        self.record_errors: list = []
        self.applied = 0
        if io is not None:
            runtime.writer = self._write

    # --- hand-over -----------------------------------------------------------------------------------------------
    def _tag(self, job: dict, gen) -> dict:
        self.seq += 1
        job.update(seq=self.seq, gen=gen, submitted=float(self.clock()), mission=self.mission_id)
        return job

    def drive(self, gen, *args, **kw) -> None:
        """A driving frame (``FarRuntime.drive_submit``'s arguments): its pose and instruction taken now."""
        self.worker.submit(self._tag(self.rt.drive_submit(*args, **kw), gen))

    def read_frame(self, read) -> None:
        """Runs a driving frame's read (decode and hand-over) inside the physics step: an exception is kept for the
        tick, so the step's other sensors and records still run (task 9)."""
        try:
            read()
        except Exception as exc:  # noqa: BLE001 -- raised by the next pump as the mission's failure
            if self.error is None:
                self.error = exc

    def stop(self, gen, **kw):
        """The stop capture's decision handed over (``FarRuntime.stop_submit``'s arguments, fixed at the capture)."""
        job = self._tag(self.rt.stop_submit(**kw), gen)
        if self.handle is not None and self.handle.state == "waiting":
            self._end("superseded")
        scene = kw["scene_input"]
        self.handle = SimpleNamespace(seq=job["seq"], gen=gen, leg=job["leg"], submitted=job["submitted"],
                                      state="waiting", why=None, result=None, decided=None,
                                      stamp_ns=int(scene.stamp_ns), clock_domain=scene.clock_domain)
        self.rt.stop_pending = True
        self.worker.submit(job)
        return self.handle

    # --- the tick ----------------------------------------------------------------------------------------------------
    def _note(self, job: dict, status: str, why=None) -> None:
        entry = {"seq": job["seq"], "kind": job["kind"], "leg": job["leg"], "gen": job["gen"], "status": status,
                 "why": why, "wall_s": float(self.clock()) - job["submitted"]}
        if job["kind"] == "drive":
            entry["tick"] = job["record"]["tick"]
        if status == "applied":
            self.applied += 1
            entry["order"] = self.applied
        self.log.append(entry)

    def _end(self, why: str) -> None:
        h = self.handle
        if h is not None and h.state == "waiting":
            h.state, h.why = "invalid", why
        self.rt.stop_pending = False

    def pump(self, gen, *, active: bool) -> None:
        """Takes in what the worker finished, in order (every tick)."""
        failure = None
        if self.error is not None:
            failure, self.error = self.error, None
        for job, status, payload in self.worker.poll():
            if job.get("mission") != self.mission_id:
                continue  # an ended mission's late result: accounted at its end (in flight), never applied
            mine = self.handle is not None and job["seq"] == self.handle.seq
            if status == "dropped":
                why = payload
            elif job["gen"] != gen:
                why = "generation"
            elif not active:
                why = "inactive"
            elif job["kind"] == "stop" and not mine:
                why = "superseded"
            elif job["leg"] != self.rt.leg:
                why = "leg"
            elif status == "error":
                self._note(job, "error", f"{type(payload).__name__}: {payload}")
                if mine:
                    self._end("error")
                failure = failure or payload
                continue
            else:
                why = None
            if why is not None:
                self._note(job, "dropped", why)
                if mine:
                    self._end(why)
                continue
            if job["kind"] == "drive":
                self.rt.drive_apply(job, payload)
                self._note(job, "applied")
            else:
                obs, rec = self.rt.stop_apply(job, payload)
                self._note(job, "applied")
                self.handle.state, self.handle.result = "done", (obs, rec)
                self.handle.decided = float(self.clock())  # judged against the wait limit (stop_state)
                self.rt.stop_pending = False
        if failure is not None:
            raise RuntimeError(f"recognition_error:{type(failure).__name__}")

    def stop_state(self, handle) -> str:
        """``waiting``, ``done`` or ``invalid``; a decision not made within the wait limit -- still waiting, or
        applied after it -- raises ``recognition_timeout`` (Codex implementation round 1 P2)."""
        if handle is not self.handle:
            return "invalid" if handle.state == "waiting" else handle.state
        if handle.state == "waiting" and handle.leg != self.rt.leg:
            self._end("leg")
        elif handle.state in ("waiting", "done"):
            at = handle.decided if handle.decided is not None else float(self.clock())
            if at - handle.submitted > self.stop_wait_s:
                handle.state, handle.result = "waiting", None
                self._end("timeout")
                raise RuntimeError("recognition_timeout")
        return handle.state

    def take(self, handle):
        """A done stop's (observation, record); the handle is spent."""
        if self.handle is handle:
            self.handle = None
        return handle.result

    def clear(self, why: str) -> None:
        """A STOP, a cancel or a mission end: the waiting work and the pending stop are dropped (noted now)."""
        for job in self.worker.clear():
            self._note(job, "dropped", why)
        self._end(why)

    # --- the mission -----------------------------------------------------------------------------------------------
    def start_mission(self, mission_id, *, zone_centre_xy, zone_radius_m: float, out_dir: Path | None) -> None:
        self.clear("mission")
        for job, status, payload in self.worker.poll():  # an earlier mission's, accounted at its end (in flight)
            pass
        self.handle, self.error, self.mission_id = None, None, mission_id
        self.log, self.record_errors, self.applied = [], [], 0
        self.rt.reset_for_mission(zone_centre_xy=zone_centre_xy, zone_radius_m=zone_radius_m, out_dir=out_dir)

    def _write(self, path: Path, arrays: dict) -> None:
        """A frame file on the session's writer thread; a refusal or a failure is this mission's record error."""
        errors = self.record_errors  # the mission's own list: a late write never lands in the next one's

        def job():
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(path, **arrays)
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{path.name}: {type(exc).__name__}: {exc}")

        nbytes = sum(np.asarray(v).nbytes for v in arrays.values())
        if not self.io.put_line(job, nbytes):
            errors.append(f"{path.name}: refused")

    def end_mission(self, mission_id):
        """A live mission's end, whatever ended it: every job not started is dropped and noted, finished ones not yet
        taken in are noted dropped, the one being worked on is noted in flight (its result is never applied). The
        mission's final summary, or None when this mission was never adopted here (it ended while being prepared --
        the earlier mission's record is not its own; Codex implementation round 1 P2). Its ``record_errors`` is the
        mission's own list, not a copy: the session writer runs its jobs in order, so a record written after this
        mission's frame files carries their errors too."""
        waiting, done, running = self.worker.end()
        for job in waiting:
            self._note(job, "dropped", "mission_end")
        for job, status, payload in done:
            if job.get("mission") == self.mission_id:
                self._note(job, "dropped", "mission_end" if status != "dropped" else payload)
        if running is not None and running.get("mission") == self.mission_id:
            self._note(running, "in_flight", "mission_end")  # an earlier mission's was accounted at its own end
        self._end("mission_end")
        if self.mission_id is None or self.mission_id != mission_id:
            return None
        out = self.summary()
        out["live"]["record_errors"] = self.record_errors
        self.mission_id = None
        return out

    def write_json(self, path: Path, record: dict) -> None:
        """A mission's JSON record on the session's writer (directly without one), made when the writer reaches it
        -- after the frame files handed over before it."""
        errors = self.record_errors

        def job():
            try:
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(dumps(record))
            except Exception as exc:  # noqa: BLE001
                errors.append(f"{path.name}: {type(exc).__name__}: {exc}")

        if self.io is None:
            job()
        elif not self.io.put_line(job, 1 << 16):
            errors.append(f"{path.name}: refused")

    def summary(self) -> dict:
        counts: dict = {}
        for j in self.log:
            key = j["status"] if j["status"] != "dropped" else f"dropped:{j['why']}"
            counts[key] = counts.get(key, 0) + 1
        return {**self.rt.summary(), "live": {"mission_id": self.mission_id, "jobs": list(self.log), "counts": counts,
                                              "record_errors": list(self.record_errors)}}

    def close(self) -> None:
        self.worker.close()


class GuardedSession:
    """A live capture session that keeps the far driving frames off while it is open (task 5): ``paused`` from its
    making until it ends -- a result, a failure, or a close (a STOP, a pause, a cancel, a mission end)."""

    def __init__(self, session, runtime):
        self._session, self._runtime = session, runtime
        runtime.paused = True

    @property
    def result(self):
        return self._session.result

    @property
    def done(self):
        return self._session.done

    def _run(self, fn):
        try:
            ended = fn()
        except BaseException:
            self._runtime.paused = False
            raise
        if ended:
            self._runtime.paused = False
        return ended

    def start(self) -> bool:
        return self._run(self._session.start)

    def stepped(self, error=None) -> bool:
        return self._run(lambda: self._session.stepped(error))

    def close(self) -> None:
        try:
            self._session.close()
        finally:
            self._runtime.paused = False


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
