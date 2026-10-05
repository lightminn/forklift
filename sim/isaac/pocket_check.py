"""Depth pocket check on the docking straight and the insertion (plan D5; no Isaac imports).

The runner builds one PocketCheck at the near (stand-off) capture, feeds it
that capture and the 10 Hz carriage depth frames on the way in, and asks it
each control tick for a speed limit. Inside the exemption region (estimated
pallet rectangle plus its axial band) the grid is waived only while this
check holds: every box of the truck along the steering-held stop -- body
between the floor and h_det, blades at the present lift -- must lie in
voxels a depth frame certified free within the lifetime
(forklift_core.perception.insertion_clearance). The stop length inside the
region is the real stopping distance (latency + braking + envelope); the
grid's 0.05 m margin stays outside it (a D4/P0a model change, recorded with
the user's 2026-10-05 D5 body delta).

Frame survival (Codex L3c P1): a frame is new only if its raw depth hash
differs from the last one; the newest new frame must be at most
frame_max_age_s old, or the limit is 0 (N13). Noise is added after hashing.
"""

from __future__ import annotations

import hashlib
import math
from dataclasses import dataclass, field

import numpy as np

from forklift_core.control.drive_permission import StoppingModel, arc_poses
from forklift_core.perception.insertion_clearance import Box, ClearanceMemory, InsertionVolume
from forklift_core.perception.pallet_geometry import pallet_boxes
from forklift_core.perception.pocket_clearance import DepthCamera

H_DET_M = 0.17  # body band top (D0 detection band)
# D0 scope (user decision 2026-10-05): protrusions below 0.03 m are out of
# scope, so every in-scope object rises above it and shows in the voxels from
# there up. The layer against the floor is not separable from it in depth
# (L3c v13: five floor-layer voxels before the face stopped the insertion).
SCOPE_FLOOR_M = 0.03
BODY_FLOOR_M = SCOPE_FLOOR_M
FORK_Z_M = (SCOPE_FLOOR_M, 0.072)  # lowered blades 0.028-0.052 +0.01, from the scope floor


@dataclass
class PocketConfig:
    # Relative pocket estimate error for the volumes' lateral allowance: about
    # twice the S2 near captures' worst (5.3 mm). At 0.02 m the blade box's
    # inner edge at full depth lay in the centre block's shadow and the stop
    # past the insertion end could never be certified (L3c v33 seed 4).
    estimate_m: float = 0.01
    # Surface tolerance's estimate share (D5 delta, user approval 2026-10-05).
    surface_estimate_m: float = 0.02
    tracking_m: float = 0.006  # path-tracking deviation the columns allow (S2 dry runs: <= 2.2 mm)
    lifetime_s: float = 20.0  # unverified; the run records the drift that would justify it
    band_ahead_m: float = 0.10  # fork columns start this far in front of the face
    # Exemption region (and body band) along the axis past the face. Past the
    # carriage face's place at the approach end (face - gap 0.10 - forks 0.406 =
    # 0.506 m) with room, so the face crosses the region's edge while driving
    # the straight, never while starting the insertion slowly from a stand
    # (L3c v8, v12 seed 3: the straddling face cells' memory had aged out).
    band_along_m: float = 0.60
    band_across_m: float = 0.20
    real_stop_extra_m: float = 0.02  # columns reach insertion depth + the real stop
    frame_max_age_s: float = 0.2
    frame_lag_s: float = 0.07  # pixels may be this much older than their reported time (render latency 0.0667 s)
    halo_m: float = 0.06  # V is evaluated this far around itself, so a moving frame can still certify V's edge
    lever_arm_m: float = 3.0  # rear axle to the farthest voxel of V seen (stand-off 2.2 m + forks + band)
    envelope_m: float = 0.01
    sample_m: float = 0.01
    body_slack_m: float = 0.05  # the body band is this much wider again: nothing to meet in front of the face, and
    # the truck may still be converging onto the axis when it gets there (L3c v4: 2.2 cm off at the stand-off)


@dataclass
class PocketCheck:
    estimate: tuple  # pallet centre (x, y, yaw) in the control frame
    axis_yaw: float  # insertion heading in the control frame
    pallet_geometry: object
    blades_rear: tuple  # rear-axle frame (x0, x1, y0, y1) per blade
    body_front_m: float
    body_rear_m: float
    body_half_width_m: float
    insertion_depth_m: float
    base_z_m: float
    base_from_optical: tuple  # (R, t): optical -> base_link
    rear_axle_offset_m: float  # base_link x of the rear axle is -offset
    camera: DepthCamera
    stopping: StoppingModel  # latency, deceleration (margin unused inside the region)
    config: PocketConfig = field(default_factory=PocketConfig)
    noise_seed: int = 0
    valid: bool = True
    invalid_reason: str | None = None
    last_hash: str | None = None
    last_new_s: float = -math.inf
    records: list = field(default_factory=list)

    def __post_init__(self) -> None:
        g = self.pallet_geometry
        cfg = self.config
        self.face_x = -g.overall_depth_m / 2
        rel = self.estimate[2] - self.axis_yaw
        c, s = math.cos(rel), math.sin(rel)
        self.solids = []
        for b in pallet_boxes(g):
            x, y, z = b.centre_m
            half = tuple(float(v) / 2 for v in b.size_m)
            # EPAL is symmetric under a half turn; boxes rotate with the estimate.
            self.solids.append(Box((x * c - y * s, x * s + y * c, z), half, rel))
        # The columns allow estimate + tracking + envelope; at run time the
        # truck's boxes carry estimate + envelope around the control pose, so
        # the tracking share is what the observed pose may deviate.
        lat = cfg.estimate_m + cfg.tracking_m + cfg.envelope_m
        x_end = self.face_x + self.insertion_depth_m + cfg.real_stop_extra_m
        x_start = self.face_x - cfg.band_ahead_m
        parts = []
        for x0, x1, y0, y1 in self.blades_rear:
            yc, hw = (y0 + y1) / 2, (y1 - y0) / 2
            parts.append(Box(((x_start + x_end) / 2, yc, sum(FORK_Z_M) / 2),
                             ((x_end - x_start) / 2, hw + lat, (FORK_Z_M[1] - FORK_Z_M[0]) / 2)))
        body_x0 = self.face_x - cfg.band_along_m
        self.body_band = Box(((body_x0 + self.face_x) / 2, 0.0, (BODY_FLOOR_M + H_DET_M) / 2),
                             ((self.face_x - body_x0) / 2, self.body_half_width_m + lat + cfg.body_slack_m,
                              (H_DET_M - BODY_FLOOR_M) / 2))
        parts.append(Box(((body_x0 + self.face_x) / 2, 0.0, (BODY_FLOOR_M + H_DET_M) / 2),
                         ((self.face_x - body_x0) / 2, self.body_half_width_m + lat + cfg.body_slack_m,
                          (H_DET_M - BODY_FLOOR_M) / 2)))
        self.memory = ClearanceMemory(InsertionVolume(parts, halo_m=cfg.halo_m), cfg.lifetime_s)
        # Known surfaces a return may lie on: the estimated pallet and the floor.
        self.surfaces = self.solids + [Box((0.0, 0.0, -0.5), (50.0, 50.0, 0.5))]
        self.conflict = self.memory.solid_conflict(self.solids)
        if self.conflict:
            self.invalidate("volume_meets_pallet")
        half_l = g.overall_depth_m / 2 + cfg.band_along_m
        half_w = g.overall_width_m / 2 + cfg.band_across_m
        self.region = (-half_l, half_l, -half_w, half_w)
        self.rng = np.random.default_rng([self.noise_seed, 11])
        self.real_stop = StoppingModel(self.stopping.latency_s, self.stopping.decel_mps2, cfg.envelope_m)

    # Frames -----------------------------------------------------------------
    def to_insertion(self, x, y, yaw):
        """Control-frame pose -> insertion frame."""
        dx, dy = x - self.estimate[0], y - self.estimate[1]
        c, s = math.cos(self.axis_yaw), math.sin(self.axis_yaw)
        return dx * c + dy * s, -dx * s + dy * c, yaw - self.axis_yaw

    def optical_from_insertion(self, rear):
        """(R, t) insertion -> optical for the truck's rear-axle control pose."""
        yaw = rear[2]
        bx = rear[0] + self.rear_axle_offset_m * math.cos(yaw)
        by = rear[1] + self.rear_axle_offset_m * math.sin(yaw)
        # insertion -> control: rotate by axis_yaw about the estimate centre.
        ca, sa = math.cos(self.axis_yaw), math.sin(self.axis_yaw)
        r_ci = np.array([[ca, -sa, 0.0], [sa, ca, 0.0], [0.0, 0.0, 1.0]])
        t_ci = np.array([self.estimate[0], self.estimate[1], 0.0])
        # control -> base_link.
        cb, sb = math.cos(yaw), math.sin(yaw)
        r_bc = np.array([[cb, sb, 0.0], [-sb, cb, 0.0], [0.0, 0.0, 1.0]])
        t_bc = -r_bc @ np.array([bx, by, self.base_z_m])
        # base_link -> optical.
        r_bo, t_bo = (np.asarray(a, dtype=float) for a in self.base_from_optical)
        r_ob = r_bo.T
        t_ob = -r_ob @ t_bo
        rot = r_ob @ r_bc @ r_ci
        trans = r_ob @ (r_bc @ t_ci + t_bc) + t_ob
        return rot, trans

    def add_frame(self, stamp_s: float, depth_m: np.ndarray, rear_at_stamp, speed_mps: float = 0.0,
                  yaw_rate_rps: float = 0.0, lift_m: float = 0.0) -> dict:
        raw = np.ascontiguousarray(np.asarray(depth_m, dtype=np.float32))
        digest = hashlib.sha256(raw.tobytes()).hexdigest()
        if digest == self.last_hash:
            rec = {"time_s": float(stamp_s), "new": False}
            self.records.append(rec)
            return rec
        self.last_hash = digest
        self.last_new_s = float(stamp_s)
        depth = np.asarray(depth_m, dtype=float).copy()
        finite = np.isfinite(depth) & (depth > 0)
        sigma = self.camera.sigma_a * depth[finite] ** 2
        # Synthetic contract, like the LiDAR's: Gaussian sigma(z) cut at the
        # same k sigma the classification tolerates.
        k = self.camera.sigma_k
        depth[finite] = depth[finite] + np.clip(self.rng.normal(0.0, 1.0, int(finite.sum())), -k, k) * sigma
        # The truck's own blades (and the carriage face they hang from) are in
        # view from 0.28 m on: a return from them is the truck, not an obstacle
        # (L3c v32 seed 4: the blade tops latched one). Self-filter by their
        # boxes at the frame's pose.
        surfaces = self.surfaces + self.own_boxes(self.to_insertion(*rear_at_stamp), lift_m)
        rec = self.memory.add_frame(stamp_s, depth, self.camera, self.optical_from_insertion(rear_at_stamp),
                                    surfaces, surface_extra_m=self.config.surface_estimate_m,
                                    pose_uncertainty_m=self.pose_uncertainty_m(speed_mps, yaw_rate_rps))
        rec = {**rec, "new": True, "rear": [float(v) for v in rear_at_stamp]}
        self.records.append(rec)
        return rec

    def pose_uncertainty_m(self, speed_mps: float, yaw_rate_rps: float) -> float:
        """How far a moving frame's pixels may sit from where they are placed:
        the lag times the motion of the farthest point of V from the rear axle
        (translation + yaw rate x lever arm; Codex checkpoint P1)."""
        return self.config.frame_lag_s * (abs(speed_mps) + abs(yaw_rate_rps) * self.config.lever_arm_m)

    def invalidate(self, reason: str) -> None:
        if self.valid:
            self.valid = False
            self.invalid_reason = reason

    # Per tick ----------------------------------------------------------------
    def own_boxes(self, pose_i, lift_m: float) -> list:
        """The blades and the carriage face (0.02 m deep) at a pose, no margins."""
        x, y, yaw = pose_i
        c, s = math.cos(yaw), math.sin(yaw)
        out = []
        for x0, x1, y0, y1 in self.blades_rear:
            u, v = (x0 + x1) / 2, (y0 + y1) / 2
            out.append(Box((x + u * c - v * s, y + u * s + v * c, 0.04 + lift_m), ((x1 - x0) / 2, (y1 - y0) / 2, 0.012), yaw))
        u = self.body_front_m - 0.01
        out.append(Box((x + u * c, y + u * s, 0.25 + lift_m), (0.01, 0.235, 0.21), yaw))
        return out

    def truck_boxes(self, pose_i, lift_m: float):
        """Body (floor..h_det) and blades at the lift, in the insertion frame,
        inflated sideways by the estimate error and the envelope (along the
        axis the envelope is part of the stop length)."""
        x, y, yaw = pose_i
        c, s = math.cos(yaw), math.sin(yaw)
        env = self.config.envelope_m
        lat = self.config.estimate_m + env
        boxes = []
        # Along the axis the envelope is already in the stop length (real_stop).
        cx = (self.body_front_m - self.body_rear_m) / 2
        boxes.append(Box((x + cx * c, y + cx * s, (BODY_FLOOR_M + H_DET_M) / 2),
                         ((self.body_front_m + self.body_rear_m) / 2, self.body_half_width_m + lat,
                          (H_DET_M - BODY_FLOOR_M) / 2), yaw))
        # The same +0.01 m vertical allowance the columns were built with; from
        # the scope floor down nothing in scope can be met.
        z0, z1 = max(0.028 + lift_m - 0.01, SCOPE_FLOOR_M), 0.052 + lift_m + 0.01
        for x0, x1, y0, y1 in self.blades_rear:
            u = (x0 + x1) / 2
            v = (y0 + y1) / 2
            boxes.append(Box((x + u * c - v * s, y + u * s + v * c, (z0 + z1) / 2),
                             ((x1 - x0) / 2, (y1 - y0) / 2 + lat, (z1 - z0) / 2), yaw))
        return boxes

    def limit(self, now_s: float, rear, curvature_inv_m: float, direction: int, speed_mps: float,
              lift_m: float) -> tuple[float, str]:
        if not self.valid:
            return 0.0, f"pocket_invalid:{self.invalid_reason}"
        if self.memory.obstacle:
            return 0.0, "pocket_obstacle"
        # The newest frame's pixels may be frame_lag_s older than its stamp
        # (Codex checkpoint P1): the age limit holds for the pixels.
        if now_s - (self.last_new_s - self.config.frame_lag_s) > self.config.frame_max_age_s:
            return 0.0, "depth_stale"
        length = self.real_stop.distance_m(abs(speed_mps)) + self.config.sample_m
        samples, arc = arc_poses(tuple(rear), curvature_inv_m, 1 if direction >= 0 else -1, length,
                                 self.config.sample_m)
        verified = 0.0
        for pose, s in zip(samples, arc):
            pose_i = self.to_insertion(*pose)
            boxes = self.truck_boxes(pose_i, lift_m)
            failing = [k for k, b in enumerate(boxes) if not self.memory.contained(b, now_s, self.region)]
            if failing:
                self.last_fail = self._explain(boxes[failing[0]], failing[0], now_s, float(s), pose_i)
                break
            verified = float(s)
        else:
            return math.inf, "ok"
        allowed = self.real_stop.speed_for(verified)
        return allowed, "ok" if allowed > 0 else "pocket_unverified"

    def depth_free_cells(self, snapshot, now_s: float) -> dict:
        """Grid cells (snapshot indices) whose whole square lies over the body
        band and whose every body-band voxel column was certified within the
        last frame_max_age_s: {cell: oldest pixel time of the column}. Evidence for the
        shadow-band memory around the face (it never waives a cell itself)."""
        if not self.valid or self.memory.obstacle or now_s - (self.last_new_s - self.config.frame_lag_s) > self.config.frame_max_age_s:
            return {}
        vol = self.memory.volume
        v = vol.voxel_m
        res = snapshot.resolution_m
        body = self.body_band
        x0b, x1b = body.center[0] - body.half[0], body.center[0] + body.half[0]
        y0b, y1b = body.center[1] - body.half[1], body.center[1] + body.half[1]
        k0 = int(np.floor((body.center[2] - body.half[2] - vol.lo[2]) / v + 1e-9))
        k1 = int(np.ceil((body.center[2] + body.half[2] - vol.lo[2]) / v - 1e-9))
        c, s = math.cos(self.axis_yaw), math.sin(self.axis_yaw)
        # Control-frame bounding box of the band, then each cell's corners in I.
        corners = [(self.estimate[0] + x * c - y * s, self.estimate[1] + x * s + y * c)
                   for x in (x0b, x1b) for y in (y0b, y1b)]
        xs, ys = [p[0] for p in corners], [p[1] for p in corners]
        i0 = max(int(np.floor((min(xs) - snapshot.origin_x_m) / res)), 0)
        i1 = min(int(np.ceil((max(xs) - snapshot.origin_x_m) / res)), snapshot.state.shape[0])
        j0 = max(int(np.floor((min(ys) - snapshot.origin_y_m) / res)), 0)
        j1 = min(int(np.ceil((max(ys) - snapshot.origin_y_m) / res)), snapshot.state.shape[1])
        out = {}
        for i in range(i0, i1):
            for j in range(j0, j1):
                pts = [self.to_insertion(snapshot.origin_x_m + (i + a) * res, snapshot.origin_y_m + (j + b) * res, 0.0)[:2]
                       for a in (0, 1) for b in (0, 1)]
                px, py = [p[0] for p in pts], [p[1] for p in pts]
                if min(px) < x0b or max(px) > x1b or min(py) < y0b or max(py) > y1b:
                    continue
                vi0 = int(np.floor((min(px) - vol.lo[0]) / v + 1e-9))
                vi1 = int(np.ceil((max(px) - vol.lo[0]) / v - 1e-9))
                vj0 = int(np.floor((min(py) - vol.lo[1]) / v + 1e-9))
                vj1 = int(np.ceil((max(py) - vol.lo[1]) / v - 1e-9))
                block = self.memory.certified_s[vi0:vi1, vj0:vj1, k0:k1]
                inv = vol.in_v[vi0:vi1, vj0:vj1, k0:k1]
                if block.size and inv.all() and (now_s - (block - self.config.frame_lag_s) <= self.config.frame_max_age_s).all():
                    # Every voxel of the column certified by a recent frame, with
                    # that observation's own time (the pixels' worst case): a live
                    # stream is not a re-observation of a column (Codex
                    # checkpoint P1).
                    out[(i, j)] = float(block.min() - self.config.frame_lag_s)
        return out

    def region_control(self):
        """The exemption region as (centre x, centre y, length, width, yaw) in the control frame."""
        x0, x1, y0, y1 = self.region
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        c, s = math.cos(self.axis_yaw), math.sin(self.axis_yaw)
        return (self.estimate[0] + cx * c - cy * s, self.estimate[1] + cx * s + cy * c, x1 - x0, y1 - y0,
                self.axis_yaw)

    def _explain(self, box, index, now_s, s, pose_i) -> dict:
        """Why a box is not contained: its overlapped voxels outside V, never or long ago certified."""
        from forklift_core.perception.insertion_clearance import overlapping_voxels

        vol = self.memory.volume
        idx, leaves = overlapping_voxels(vol, box)
        x0, x1, y0, y1 = self.region
        v = vol.voxel_m
        vx0 = vol.lo[0] + idx[:, 0] * v
        vy0 = vol.lo[1] + idx[:, 1] * v
        idx = idx[(vx0 + v > x0) & (vx0 < x1) & (vy0 + v > y0) & (vy0 < y1)]
        in_v = vol.in_v[idx[:, 0], idx[:, 1], idx[:, 2]]
        age = now_s - self.memory.certified_s[idx[:, 0], idx[:, 1], idx[:, 2]]
        bad = idx[~in_v | (age > self.config.lifetime_s)]
        centres = vol.lo + (bad + 0.5) * v
        return {
            "box": ["body", "blade_left", "blade_right"][index] if index < 3 else str(index),
            "arc_s": s, "pose_insertion": [round(float(c), 4) for c in pose_i], "leaves": bool(leaves),
            "voxels": int(len(idx)), "outside_v": int((~in_v).sum()),
            "never_certified": int((~np.isfinite(age) | (age > 1e9)).sum()),
            "expired": int((np.isfinite(age) & (age > self.config.lifetime_s) & (age < 1e9)).sum()),
            "examples_insertion": np.round(centres[:12], 3).tolist(),
            "examples_last_code": [int(self.memory.last_code[tuple(b)]) for b in bad[:12]],
        }

    def _to_insertion_points(self, entry) -> list:
        rec = next((r for r in self.records if r.get("time_s") == entry["time_s"] and "rear" in r), None)
        if rec is None:
            return []
        rot, trans = self.optical_from_insertion(rec["rear"])
        pts = (np.asarray(entry["points"], dtype=float) - trans) @ rot
        return np.round(pts, 3).tolist()

    def summary(self) -> dict:
        new = [r for r in self.records if r.get("new")]
        return {
            "valid": self.valid,
            "invalid_reason": self.invalid_reason,
            "volume_meets_pallet_voxels": self.conflict,
            "obstacle": self.memory.obstacle,
            "obstacle_points": self.memory.obstacle_points[:5],
            "obstacle_points_insertion_frame": [
                {"time_s": e["time_s"], "points": self._to_insertion_points(e)} for e in self.memory.obstacle_points[:5]
            ],
            "last_fail": getattr(self, "last_fail", None),
            "frames": len(self.records),
            "new_frames": len(new),
            "first_frame": new[0] if new else None,
            "config": self.config.__dict__,
            "depth": "synthetic (Isaac depth + sigma(z) = 0.0036 z^2 noise)",
        }


__all__ = ["PocketCheck", "PocketConfig"]
