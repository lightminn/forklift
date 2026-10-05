"""The priority-5 obstacle layer between the Isaac runner and forklift_core (no Isaac imports).

docs/plans/2026-10-04-lidar-obstacle-map.md. The runner casts the obstacle
LiDARs at every scan tick and hands the raw rays here; this module turns them
into REP-117 scans with the synthetic noise contract (sigma 0.02 m cut at
+-0.06 m, P0a), keeps the rolling grid, refreshes the drive permission with
a local snapshot around the truck, answers the per-tick speed limit, and
builds the hall-wide occupancy grid the planner replans with. Everything is
in the frame control uses: the applied SLAM correction o wheel odometry.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np
import yaml

from forklift_core.control.drive_permission import DrivePermission, PermissionConfig, StoppingModel
from forklift_core.control.shadow_memory import ShadowMemory
from forklift_core.perception.obstacle_grid import (
    AgeErrorTable,
    GridConfig,
    ObstacleGrid,
    ObstacleScan,
    snapshot_to_occupancy,
)
from forklift_core.planning.geometry import Footprint


@dataclass(frozen=True)
class ObstacleSensor:
    name: str
    xyz_m: tuple[float, float, float]  # base_link
    yaw_rad: float
    may_clear: bool


def load_layer_config(path: Path) -> dict:
    data = yaml.safe_load(Path(path).read_text())
    clear_max = float(data["clear_max_height_m"])
    sensors = [
        ObstacleSensor(
            s["name"], tuple(float(v) for v in s["xyz"]), math.radians(float(s["yaw_deg"])),
            float(s["xyz"][2]) <= clear_max,
        )
        for s in data["sensors"]
    ]
    return {**data, "sensors": sensors}


def beam_limits(origin, directions, *, band_top_m: float, floor_margin_m: float = 0.05) -> np.ndarray:
    """Per beam, the range up to which a tilted planar beam stays between the
    floor and band_top_m (h_det): rising beams leave the band, sinking ones hit
    the floor (Codex L0b P1)."""
    z0 = float(origin[2])
    dz = np.asarray(directions, dtype=float)[:, 2]
    out = np.full(len(dz), np.inf)
    up = dz > 1e-9
    down = dz < -1e-9
    out[up] = (band_top_m - z0) / dz[up]
    out[down] = np.maximum(z0 - floor_margin_m, 0.0) / -dz[down]
    return np.maximum(out, 0.0)


def exempt_region(snapshot, region):
    """Cells wholly inside the rectangle become FREE at the snapshot time: the
    depth pocket check, not the grid, answers for them (it must gate every
    control tick while this is set)."""
    from forklift_core.perception.obstacle_grid import FREE

    cx, cy, length, width, yaw = region
    res = snapshot.resolution_m
    nx, ny = snapshot.state.shape
    c, s = math.cos(yaw), math.sin(yaw)
    xs = snapshot.origin_x_m + np.arange(nx + 1) * res - cx
    ys = snapshot.origin_y_m + np.arange(ny + 1) * res - cy
    u = xs[:, None] * c + ys[None, :] * s
    v = -xs[:, None] * s + ys[None, :] * c
    corner_in = (np.abs(u) <= length / 2 + 1e-9) & (np.abs(v) <= width / 2 + 1e-9)
    inside = corner_in[:-1, :-1] & corner_in[1:, :-1] & corner_in[:-1, 1:] & corner_in[1:, 1:]
    if not inside.any():
        return snapshot
    state = snapshot.state.copy()
    stamp = snapshot.free_stamp.copy()
    state[inside] = FREE
    stamp[inside] = snapshot.stamp_s
    return replace(snapshot, state=state, free_stamp=stamp)


class ObstacleLayer:
    def __init__(self, config: dict, *, hall, error_table: AgeErrorTable, unloaded: Footprint,
                 loaded: Footprint, body_front_m: float, rear_axle_x_in_base_m: float, noise_seed: int,
                 blades_rear_m: tuple = ()):
        self.config = config
        self.sensors: list[ObstacleSensor] = config["sensors"]
        self.beam_angles = np.linspace(-math.pi, math.pi, int(config["beams"]), endpoint=False)
        self.range_min_m = float(config["range_min_m"])
        self.range_max_m = float(config["range_max_m"])
        self.noise_std_m = float(config["noise_std_m"])
        self.noise_cut_m = float(config["noise_cut_m"])
        self.band_top_m = float(config["band_top_m"])  # h_det: a beam above it no longer clears
        g = config["grid"]
        self.grid_config = GridConfig(
            hall.x_min_m, hall.x_max_m, hall.y_min_m, hall.y_max_m, error_table,
            resolution_m=float(g["resolution_m"]), max_mark_m=float(g["max_mark_m"]),
            max_clear_m=float(g["max_clear_m"]), range_min_m=self.range_min_m,
            free_max_age_s=float(g["free_max_age_s"]), occupied_max_age_s=float(g["occupied_max_age_s"]),
            sensor_bound_m=self.noise_cut_m, free_r_cap_m=float(g["free_r_cap_m"]),
            free_rho_m=float(g["free_rho_m"]), close_gap_m=float(g.get("close_gap_m", 0.0)),
            free_min_width_m=float(g.get("free_min_width_m", 0.0)),
            taper_m=float(g.get("taper_m", 0.0)),
        )
        self.grid = ObstacleGrid(self.grid_config)
        p = config["permission"]
        self.permission = DrivePermission(
            PermissionConfig(
                StoppingModel(float(p["latency_s"]), float(p["decel_mps2"]), float(p["margin_m"])),
                envelope_offset_m=float(p["envelope_m"]),
                evidence_max_age_s=float(g["free_max_age_s"]),
                envelope_ramp_m=float(p["envelope_ramp_m"]),
                sensor_timeout_s=float(p["sensor_timeout_s"]),
                lookahead_m=float(p["lookahead_m"]),
                shadow_band_m=float(p.get("shadow_band_m", 0.0)),
            )
        )
        self.window_m = float(g["window_m"])
        # Shadow-band memory (D4 delta approved 2026-10-05); absent = strict rule.
        band = p.get("shadow_band_m")
        self.shadow = None if band is None else ShadowMemory(float(band), error_table, evidence_max_age_s=float(g["free_max_age_s"]))
        self.unloaded, self.loaded = unloaded, loaded
        self.body = Footprint(body_front_m, unloaded.rear_m, unloaded.half_width_m)
        # Unloaded, the permission checks the body and the fork blades (rear-axle
        # frame (x0, x1, y0, y1)), not their hull: an object between the blades
        # is met only by the body's front face, which the body part checks (D4).
        # The planner checks the hull, and driving forward the permission checks
        # it too, so the truck never enters a pose the planner cannot start
        # from (L3b v23: an object beside a blade, inside the hull, made every
        # replan invalid_start). Reversing, the forks leave the hull's front:
        # the body and blades are checked instead (L3c v19: after the withdraw
        # the hull's fork end lay in the delivered pallet's grid swelling and
        # no reverse could start). permission.shape: hull_forward (default),
        # hull or parts.
        mode = str(p.get("shape", "hull_forward"))
        self.parts_shape = None if not blades_rear_m else (
            [(self.body, 0.0, 0.0)] + [
                (Footprint((x1 - x0) / 2, (x1 - x0) / 2, (y1 - y0) / 2), (y0 + y1) / 2, (x0 + x1) / 2)
                for x0, x1, y0, y1 in blades_rear_m
            ]
        )
        self.shape_mode = mode if self.parts_shape is not None else "hull"
        self.unloaded_shape = self.parts_shape if self.shape_mode == "parts" else unloaded
        self.rear_x = rear_axle_x_in_base_m
        self.rng = np.random.default_rng([noise_seed, 7])
        self.last_scan_s: dict = {}
        self.snapshot = None
        # Docking exemption (plan D5): (x, y, length, width, yaw) in the control
        # frame; set only while the depth pocket check gates every tick.
        self.exempt = None
        self.depth_support = None  # callable(snapshot, now) -> {cell: certification time}, set with exempt

    def footprints(self, loaded: bool, direction: int = 0) -> tuple[Footprint, Footprint]:
        """(footprint checked, own outline excluded): a carried pallet is part of the truck."""
        if loaded:
            return self.loaded, self.loaded
        if self.shape_mode == "hull_forward" and direction < 0:
            # Reversing, the present body and blades are the truck: a cell the
            # blade tips cover now is entered only where the reverse sweep
            # reaches its part outside them (L3c v21: the delivered pallet's
            # swelling at the tips held every return start).
            return self.parts_shape, self.parts_shape
        return self.unloaded_shape, self.body

    def ranges(self, distances, hits) -> np.ndarray:
        """REP-117 ranges with the truncated noise contract."""
        out = np.where(hits, np.asarray(distances, dtype=float), np.inf)
        finite = np.isfinite(out)
        noise = np.clip(self.rng.normal(0, self.noise_std_m, int(finite.sum())), -self.noise_cut_m, self.noise_cut_m)
        out[finite] = out[finite] + noise
        out[finite & (out > self.range_max_m)] = np.inf
        out[finite & (out < self.range_min_m)] = -np.inf
        return out

    def add_scans(self, stamp_s: float, raw: dict, *, odom_rear, loaded: bool) -> None:
        """raw: sensor name -> (distances, hits, own) from planar_lidar.cast_scan_flags."""
        _, own_outline = self.footprints(loaded)
        for sensor in self.sensors:
            if sensor.name not in raw:
                continue  # a silent sensor (L4 N9/N13): its last stamp ages
            distances, hits, own, *rest = raw[sensor.name]
            limit = rest[0] if rest else None
            self.grid.add_scan(
                ObstacleScan(
                    float(stamp_s), sensor.name, tuple(float(v) for v in odom_rear),
                    (sensor.xyz_m[0] - self.rear_x, sensor.xyz_m[1], sensor.yaw_rad),
                    self.beam_angles, self.ranges(distances, hits), np.asarray(own, dtype=bool),
                    sensor.may_clear, (own_outline.front_m, own_outline.rear_m, own_outline.half_width_m),
                    None if limit is None else np.asarray(limit, dtype=float),
                )
            )
            self.last_scan_s[sensor.name] = float(stamp_s)

    def refresh(self, now_s: float, correction, version: int, *, current_pose, path_ahead, loaded: bool,
                direction: int = 0):
        """New local snapshot and path check around the truck."""
        x, y = current_pose[0], current_pose[1]
        w, res = self.window_m, self.grid_config.resolution_m
        x0 = math.floor((x - w) / res) * res
        y0 = math.floor((y - w) / res) * res
        self.grid.config = replace(self.grid_config, x_min_m=x0, x_max_m=x0 + 2 * w, y_min_m=y0, y_max_m=y0 + 2 * w)
        try:
            self.snapshot = self.grid.snapshot(now_s, correction, version)
        finally:
            self.grid.config = self.grid_config
        footprint, own = self.footprints(loaded, direction)
        # The shadow-band memory may lean on cells the depth pocket check saw
        # free (observation, with its time), never on the exemption itself
        # (Codex checkpoint P1: exempt cells as fresh FREE spread RETAINED past
        # the region). The face cells straddling the region's edge need that
        # evidence (L3c v8, insertion start).
        if self.shadow is not None:
            support = self.depth_support(self.snapshot, now_s) if self.depth_support is not None else None
            self.snapshot = self.shadow.apply(self.snapshot, current_pose, own if isinstance(own, Footprint) else self.body,
                                              extra_support=support)
        if self.exempt is not None:
            self.snapshot = exempt_region(self.snapshot, self.exempt)
        return self.permission.update(self.snapshot, path_ahead, footprint, own, current_pose=current_pose,
                                      direction=direction)

    def limit(self, now_s: float, *, current_pose, curvature_inv_m: float, direction: int, loaded: bool, cap_mps: float):
        footprint, own = self.footprints(loaded, direction)
        return self.permission.allowed_speed(
            now_s, self.last_scan_s, current_pose=current_pose, curvature_inv_m=curvature_inv_m,
            direction=direction, footprint=footprint, own_footprint=own, speed_cap_mps=cap_mps,
        )

    def planner_grid(self, now_s: float, correction, version: int):
        """Hall-wide OccupancyGrid of the occupied cells for a (re)plan."""
        snap = self.grid.snapshot(now_s, correction, version)
        return snapshot_to_occupancy(snap)


__all__ = ["ObstacleLayer", "ObstacleSensor", "load_layer_config"]
