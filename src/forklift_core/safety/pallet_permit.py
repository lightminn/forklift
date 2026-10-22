"""The teleop permit sees the pallets (persistent console plan D10, 4th ed.).

The LiDAR plane (1.05 m) does not see a 0.144 m pallet and the known-pallet layer is off, so the grid permit lets a
teleoperated truck drive its forks into a pocket. Each target pallet is a keep-out **zone**: its outline grown by the
error margin of where it and the truck are known to be, and by the bundle's envelope (the zones are built by the
caller). An arc is allowed up to the last sample before its outline meets a zone -- by area, with separating axes
(the body can hold a small pallet whole) -- and the motion between samples is covered by a pad: in one step no point
of the outline moves more than ``step + |kappa| step R_max``. Inside a zone every arc is 0: where it is safe to leave
cannot be told from the zone (plan D10; the recovery that retraces the way in is stage 6b's).

The cache is BundlePermit's: per direction, installed again when the key changes (arc ends, targets, grid snapshot,
applied correction) or after ``refresh_m`` of driving; the two end arcs are judged from the current pose every call.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.control.drive_permission import arc_poses, parts_of

REFRESH_M = 0.25
# The margins (plan D10 4th ed., 2026-10-19 measurements on two live U1 runs, sample maxima, not bounds): a recognised
# or delivered pallet's outline is grown by its map-frame estimate error (0.026 m) + the control pose's position error
# (0.130 m) + its heading error moving the outline's farthest point (2 x 1.66 m x sin(0.031/2) = 0.0515 m); the prior
# pickup zone already holds the placement, so it is grown by the pose error alone. ENVELOPE_M covers the bundle's
# gaps between arcs, counted apart (docs/plans/2026-10-17-teleop-bundle-margin.md).
RECOGNISED_MARGIN_M = 0.21
PRIOR_POSE_MARGIN_M = 0.182
ENVELOPE_M = 0.01


def zones_from(targets, pickup_zone) -> list:
    """The keep-out zones of a targets book: every pallet on the floor; a prior one is the whole pickup zone (its
    registered place is the scene's, plan D10), the others their outline grown by their uncertainty. ``pickup_zone``
    is one zone rectangle for every prior pallet, or a {pallet_id: zone rectangle} map (a session, plan
    2026-10-22-session-scene-pallet-props 2.5: each prior pallet its own)."""
    out = []
    for pallet in targets.pallets.values():
        if pallet.state == "carried" or pallet.pose is None:
            continue
        if pallet.basis == "prior":
            zone = pickup_zone.get(pallet.pallet_id) if isinstance(pickup_zone, dict) else pickup_zone
            if zone is None:
                continue
            grow = PRIOR_POSE_MARGIN_M + ENVELOPE_M
            out.append(Zone(pallet.pallet_id, zone.x_m, zone.y_m, zone.length_m + 2 * grow,
                            zone.width_m + 2 * grow, zone.yaw_rad))
        elif pallet.depth_m is not None and pallet.width_m is not None:
            grow = (RECOGNISED_MARGIN_M if pallet.uncertainty_m is None else pallet.uncertainty_m) + ENVELOPE_M
            out.append(Zone(pallet.pallet_id, pallet.pose.x_m, pallet.pose.y_m, pallet.depth_m + 2 * grow,
                            pallet.width_m + 2 * grow, pallet.pose.yaw_rad))
    return out


def preparation_obstacle(pallet, pickup_zone, *, pose=None, size=None):
    """The rectangle a preparation's observation plan holds a target pallet as (plan D4b item 1; one function for
    the runner's live_prepare and its start check): a prior pallet is the whole pickup zone, the others their outline
    grown by their uncertainty. ``pose`` replaces the pallet's own (a frame change carried); None when it is on the
    forks or has no pose. ``size``: (depth, width) when the pallet's own is not recorded."""
    from forklift_core.planning.geometry import Rectangle

    pose = pose if pose is not None else pallet.pose
    if pallet.state == "carried" or pose is None:
        return None
    if pallet.basis == "prior" and pickup_zone is not None:
        return pickup_zone
    grow = RECOGNISED_MARGIN_M if pallet.uncertainty_m is None else pallet.uncertainty_m
    if pallet.basis == "prior":  # a prior without a zone: the old 0.30 m rule
        grow = 0.30
    depth, width = ((pallet.depth_m, pallet.width_m) if pallet.depth_m is not None and pallet.width_m is not None
                    else size)
    return Rectangle(float(pose.x_m), float(pose.y_m), float(depth) + 2 * grow, float(width) + 2 * grow,
                     float(pose.yaw_rad))


def session_pallet_props(targets, *, exclude_id, nominal: dict, shape, prior_grow_m: float, moved=None,
                         moved_ids=()) -> tuple:
    """The other pallets of a session as planning props (plan 2026-10-22-session-scene-pallet-props 2.6; one function
    for the live preparation's scene, its start check, the post-release preview and the worker's input). A prior
    pallet is its drawn slot (``nominal``: id -> (x, y, yaw)) grown by ``prior_grow_m`` (decision D1); a recognised or
    delivered one its outline where the targets list has it, grown by its uncertainty (the recognised margin when
    none). The mission's own pallet (``exclude_id``), a carried one and one without a pose are left out.
    ``shape`` = (depth, width, height). ``moved`` (x, y, yaw) -> (x, y, yaw) applies to the ids in ``moved_ids`` only:
    the estimates the hold found, as the release itself judges them -- never by basis."""
    from forklift_core.planning.geometry import Rectangle
    from forklift_core.planning.pallet_mission import AssetSpec, PlacedProp

    depth0, width0, height = (float(v) for v in shape)
    out = []
    for pallet_id in sorted(targets.pallets):
        pallet = targets.pallets[pallet_id]
        if pallet_id == exclude_id or pallet.state == "carried" or pallet.pose is None:
            continue
        depth = float(pallet.depth_m) if pallet.depth_m is not None else depth0
        width = float(pallet.width_m) if pallet.width_m is not None else width0
        if pallet.basis == "prior":  # its drawn slot (a pallet without one: where it is registered)
            pose = nominal.get(pallet_id, (pallet.pose.x_m, pallet.pose.y_m, pallet.pose.yaw_rad))
            x, y, yaw = (float(v) for v in pose)
            grow = float(prior_grow_m)
        else:
            x, y, yaw = float(pallet.pose.x_m), float(pallet.pose.y_m), float(pallet.pose.yaw_rad)
            if moved is not None and pallet_id in moved_ids:
                x, y, yaw = (float(v) for v in moved((x, y, yaw)))
            grow = RECOGNISED_MARGIN_M if pallet.uncertainty_m is None else float(pallet.uncertainty_m)
        out.append(PlacedProp(AssetSpec(f"session:pallet:{pallet_id}", depth + 2 * grow, width + 2 * grow, height),
                              Rectangle(x, y, depth + 2 * grow, width + 2 * grow, yaw)))
    return tuple(out)


@dataclass(frozen=True)
class Zone:
    zone_id: str
    x_m: float
    y_m: float
    length_m: float  # along yaw
    width_m: float
    yaw_rad: float


def obb_overlap(centres, half, yaws, centre2, half2, yaw2) -> np.ndarray:
    """Whether oriented boxes (centres (n, 2), half extents (hx, hy), yaws (n,)) overlap one box -- closed, by the
    four separating axes."""
    centres, yaws = np.asarray(centres, dtype=float), np.asarray(yaws, dtype=float)
    d = centres - np.asarray(centre2, dtype=float)
    ax1 = np.stack((np.cos(yaws), np.sin(yaws)), axis=1)
    ay1 = np.stack((-np.sin(yaws), np.cos(yaws)), axis=1)
    ax2 = np.array([math.cos(yaw2), math.sin(yaw2)])
    ay2 = np.array([-math.sin(yaw2), math.cos(yaw2)])
    hx, hy = half
    gx, gy = half2
    out = np.ones(len(centres), dtype=bool)
    for axis in (ax1, ay1, np.broadcast_to(ax2, ax1.shape), np.broadcast_to(ay2, ax1.shape)):
        r1 = hx * np.abs(np.sum(ax1 * axis, axis=1)) + hy * np.abs(np.sum(ay1 * axis, axis=1))
        r2 = gx * np.abs(axis @ ax2) + gy * np.abs(axis @ ay2)
        out &= np.abs(np.sum(d * axis, axis=1)) <= r1 + r2
    return out


def _part_boxes(poses, shape):
    """Per part: (centres (n, 2), half extents, yaws) of its box at every rear-axle pose."""
    x, y, yaw = poses[:, 0], poses[:, 1], poses[:, 2]
    c, s = np.cos(yaw), np.sin(yaw)
    boxes = []
    for fp, lat, lon in parts_of(shape):
        mid = lon + (fp.front_m - fp.rear_m) / 2
        centres = np.stack((x + mid * c - lat * s, y + mid * s + lat * c), axis=1)
        boxes.append((centres, ((fp.front_m + fp.rear_m) / 2, fp.half_width_m), yaw))
    return boxes


def _reach_m(shape) -> float:
    """The farthest point of the shape from the rear axle."""
    return max(math.hypot(max(abs(lon + fp.front_m), abs(lon - fp.rear_m)), abs(lat) + fp.half_width_m)
               for fp, lat, lon in parts_of(shape))


def _meets(boxes, zone, grow_m: float) -> np.ndarray:
    hit = np.zeros(len(boxes[0][0]), dtype=bool)
    for centres, half, yaws in boxes:
        hit |= obb_overlap(centres, half, yaws, (zone.x_m, zone.y_m),
                           (zone.length_m / 2 + grow_m, zone.width_m / 2 + grow_m), zone.yaw_rad)
    return hit


def inside(pose, shape, zones) -> str | None:
    """The zone the outline is in now (unpadded), or None."""
    boxes = _part_boxes(np.asarray([pose], dtype=float), shape)
    for zone in zones:
        if _meets(boxes, zone, 0.0)[0]:
            return zone.zone_id
    return None


def first_contact(pose, kappa: float, direction: int, length_m: float, step_m: float, shape, zones, *,
                  r_max_m: float | None = None) -> tuple[float, str | None]:
    """(distance along the arc the truck may go, the zone that stops it): the last sample whose outline is clear
    of every zone grown by the pad (so the move to it cannot have met a zone); 0 when the outline is in a zone now;
    inf when the arc meets none."""
    if inside(pose, shape, zones) is not None:
        return 0.0, inside(pose, shape, zones)
    poses, s = arc_poses(tuple(pose), kappa, direction, length_m, step_m)
    pad = step_m + abs(kappa) * step_m * (_reach_m(shape) if r_max_m is None else r_max_m)
    boxes = _part_boxes(poses[1:], shape)
    best, which = math.inf, None
    for zone in zones:
        hit = np.flatnonzero(_meets(boxes, zone, pad))
        if len(hit) and s[hit[0]] < best:  # hit[0] is sample hit[0] + 1: the last clear one is s[hit[0]]
            best, which = float(s[hit[0]]), zone.zone_id
    return best, which


def first_contact_m(*args, **kwargs) -> float:
    return first_contact(*args, **kwargs)[0]


class PalletPermit:
    def __init__(self, stopping, *, step_m: float, refresh_m: float = REFRESH_M, walk_speed_mps: float = 0.3):
        self.stopping, self.step_m, self.refresh_m, self.walk_speed_mps = stopping, step_m, refresh_m, walk_speed_mps
        self._cache: dict = {}  # direction -> {"key", "arcs": [(kappa, s, zone)], "since_m"}

    def advance(self, driven_m: float) -> None:
        for entry in self._cache.values():
            entry["since_m"] += abs(driven_m)

    def _length_m(self, check_speed_mps: float) -> float:
        walk = self.refresh_m + self.stopping.distance_m(max(self.walk_speed_mps, abs(check_speed_mps))) + self.step_m
        return walk + self.stopping.distance_m(check_speed_mps) + self.step_m

    def limit(self, zones, zones_key, pose, curvatures, direction: int, check_speed_mps: float, shape, *,
              key) -> tuple[float, str]:
        """(allowed speed, reason) over the bundle's arcs; ``key`` holds what moves the pose or the grid without
        driving (the grid snapshot key, the applied correction)."""
        if not zones:
            return math.inf, "ok"
        near = inside(pose, shape, zones)
        if near is not None:
            return 0.0, "pallet_near"
        length = self._length_m(check_speed_mps)
        ends = (round(curvatures[0] / 0.05), round(curvatures[-1] / 0.05))
        full_key = (direction, ends, len(curvatures), zones_key, key)
        entry = self._cache.get(direction)
        if entry is None or entry["key"] != full_key or entry["since_m"] > self.refresh_m:
            arcs = [(k, *first_contact(pose, k, direction, length, self.step_m, shape, zones)) for k in curvatures]
            entry = self._cache[direction] = {"key": full_key, "arcs": arcs, "since_m": 0.0}
        best = (math.inf, "ok")
        arcs = entry["arcs"]
        for index, (kappa, s, zone) in enumerate(arcs):
            if index in (0, len(arcs) - 1):  # the ends from the pose now (as BundlePermit)
                s, zone = first_contact(pose, kappa, direction, length, self.step_m, shape, zones)
            else:
                s = s - entry["since_m"]
            if math.isinf(s):
                continue
            speed = self.stopping.speed_for(max(0.0, s))
            if speed < best[0]:
                best = (speed, f"pallet:{zone}")
        return best
