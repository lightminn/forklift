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


class ObstacleLayer:
    def __init__(self, config: dict, *, hall, error_table: AgeErrorTable, unloaded: Footprint,
                 loaded: Footprint, body_front_m: float, rear_axle_x_in_base_m: float, noise_seed: int):
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
            )
        )
        self.window_m = float(g["window_m"])
        self.unloaded, self.loaded = unloaded, loaded
        self.body = Footprint(body_front_m, unloaded.rear_m, unloaded.half_width_m)
        self.rear_x = rear_axle_x_in_base_m
        self.rng = np.random.default_rng([noise_seed, 7])
        self.last_scan_s: dict = {}
        self.snapshot = None

    def footprints(self, loaded: bool) -> tuple[Footprint, Footprint]:
        """(footprint checked, own outline excluded): a carried pallet is part of the truck."""
        return (self.loaded, self.loaded) if loaded else (self.unloaded, self.body)

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

    def refresh(self, now_s: float, correction, version: int, *, current_pose, path_ahead, loaded: bool):
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
        footprint, own = self.footprints(loaded)
        return self.permission.update(self.snapshot, path_ahead, footprint, own, current_pose=current_pose)

    def limit(self, now_s: float, *, current_pose, curvature_inv_m: float, direction: int, loaded: bool, cap_mps: float):
        footprint, own = self.footprints(loaded)
        return self.permission.allowed_speed(
            now_s, self.last_scan_s, current_pose=current_pose, curvature_inv_m=curvature_inv_m,
            direction=direction, footprint=footprint, own_footprint=own, speed_cap_mps=cap_mps,
        )

    def planner_grid(self, now_s: float, correction, version: int):
        """Hall-wide OccupancyGrid of the occupied cells for a (re)plan."""
        snap = self.grid.snapshot(now_s, correction, version)
        return snapshot_to_occupancy(snap)


__all__ = ["ObstacleLayer", "ObstacleSensor", "load_layer_config"]
