"""The teleoperated drive command (M-b plan B8; design 4.4): ratio to physical, the curvature rate limit, the dead-man
switch, the shared speed chain (creep, slew, final permission cap, preempt) and the arc-bundle permission.

The permission is asked on the arbiter's own ``DrivePermission`` (the mission's installed path is left alone): for
every curvature from the measured steering to the output curvature, at most ``KAPPA_STEP`` apart, the arc is
installed with ``update()`` and ``allowed_speed`` read at that curvature; the smallest limit counts. While the
steering moves monotonically between the two, the swept path is taken to lie in that bundle -- an approximation
whose gaps the permission's envelope offset is assumed to cover (plan section 6: to be verified).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.control.drive_permission import DrivePermission, arc_poses
from forklift_core.control.speed_chain import creep, slew_cap_stop

KAPPA_STEP = 0.05  # 1/m between the arcs of the bundle


@dataclass(frozen=True)
class TeleopLimits:
    """Provisional until H3 (design 5.4). Video starts no lower than direct (design 5.3)."""

    video_forward_mps: float = 0.5
    video_reverse_mps: float = 0.25
    direct_forward_mps: float = 0.5
    direct_reverse_mps: float = 0.25
    loaded_mps: float = 0.30
    curvature_inv_m: float = 0.36
    curvature_rate_inv_m_s: float = 0.5
    accel_mps2: float = 0.8
    stop_speed_mps: float = 0.012  # below this the measured direction is not trusted (the stop detector's gate)

    def __post_init__(self):
        if self.direct_forward_mps > self.video_forward_mps or self.direct_reverse_mps > self.video_reverse_mps:
            raise ValueError("direct limits must not exceed video limits (design 5.3)")


def bundle(kappa_from: float, kappa_to: float) -> list[float]:
    """Curvatures from one to the other, both ends included, at most KAPPA_STEP apart (one arc when they agree)."""
    if kappa_from == kappa_to:
        return [float(kappa_from)]
    n = max(1, math.ceil(abs(kappa_to - kappa_from) / KAPPA_STEP))
    return [float(k) for k in np.linspace(kappa_from, kappa_to, n + 1)]


def arc_permit(permission: DrivePermission, snapshot, now_s: float, sensors_last_s: dict, *, pose, curvatures,
               direction: int, check_speed_mps: float, footprint, own_footprint) -> tuple[float, str]:
    """The smallest (limit, reason) over the bundle. Each arc is longer than the permission's walk so its end is
    never reached -- a teleoperated drive has no goal to slow down for."""
    cfg = permission.config
    length = cfg.lookahead_m + cfg.stopping.distance_m(check_speed_mps) + cfg.step_m
    best = (math.inf, "ok")
    for kappa in curvatures:
        path, _ = arc_poses(tuple(pose), kappa, direction, length, cfg.step_m)
        permission.update(snapshot, path, footprint, own_footprint, current_pose=pose, direction=direction)
        limit, why = permission.allowed_speed(now_s, sensors_last_s, current_pose=pose, curvature_inv_m=kappa,
                                              direction=direction, footprint=footprint, own_footprint=own_footprint,
                                              speed_cap_mps=check_speed_mps)
        if limit < best[0]:
            best = (limit, why)
    return best


class TeleopDrive:
    def __init__(self, limits: TeleopLimits, decel_mps2: float, dt: float) -> None:
        self.limits, self.decel_mps2, self.dt = limits, float(decel_mps2), float(dt)
        self.kappa = 0.0  # the output curvature (rate limited)

    def target(self, teleop: dict | None, *, loaded: bool) -> tuple[float, float]:
        """(speed, curvature) the ratios ask for, before the dead-man switch and the chain."""
        if teleop is None:
            return 0.0, self.kappa
        lim = self.limits
        ratio = teleop["speed_ratio"]
        if teleop["mode"] == "video":
            cap = lim.video_forward_mps if ratio >= 0 else lim.video_reverse_mps
        else:
            cap = lim.direct_forward_mps if ratio >= 0 else lim.direct_reverse_mps
        if loaded:
            cap = min(cap, lim.loaded_mps)
        return ratio * cap, teleop["curvature_ratio"] * lim.curvature_inv_m

    def step(self, teleop: dict | None, *, armed: bool, drive_allowed: bool, loaded: bool, preempt: bool,
             last_applied_mps: float, measured_mps: float, measured_kappa: float, steering_error_rad: float,
             permit) -> tuple[float, float, tuple[float, str]]:
        """(speed, curvature, (limit, reason)) for this cycle. ``permit(curvatures, direction, check_speed)`` asks
        the arc bundle; ``steering_error_rad`` is how far the measured steering is from this cycle's target."""
        lim = self.limits
        speed_target, kappa_target = self.target(teleop, loaded=loaded)
        step_k = lim.curvature_rate_inv_m_s * self.dt
        self.kappa += float(np.clip(kappa_target - self.kappa, -step_k, step_k))
        if teleop is None or not armed or not teleop["deadman"]:
            speed_target = 0.0
        # Every direction the truck may move this cycle (design 4.4): the measured one when it moves, the last
        # applied one (the slew keeps its sign until zero) and the request's. A sign change is checked both ways
        # (Codex M-b review P1: forward checked while the output still ran -0.24 m/s).
        directions = {1 if v > 0 else -1 for v in (measured_mps if abs(measured_mps) > lim.stop_speed_mps else 0.0,
                                                    last_applied_mps, speed_target) if v != 0.0} or {1}
        check = max(abs(measured_mps), abs(last_applied_mps), abs(speed_target))
        limit, why = min((permit(bundle(measured_kappa, self.kappa), d, check) for d in sorted(directions)),
                         key=lambda r: r[0])
        if not drive_allowed:
            limit, why = 0.0, "drive_forbidden"
        speed = slew_cap_stop(last_applied_mps, creep(speed_target, steering_error_rad), decel_mps2=self.decel_mps2,
                              accel_mps2=lim.accel_mps2, dt=self.dt, cap_mps=max(0.0, limit), stop=preempt)
        return speed, self.kappa, (limit, why)


class BundlePermit:
    """The arc bundle's permission at the batch runner's cost (M-c live runs, plan C4a): per direction, the arcs'
    path checks are installed again only when the grid changed, the bundle's ends moved by a step or the truck has
    driven ``refresh_m`` since; between, each installed check is advanced by the distance driven, as the batch's
    obstacle layer does between scans. Every cycle: the two ends are asked in full at the **current** curvatures
    (their emergency-stop arcs, the sensor silence, the evidence age), and every arc between by its advanced path
    distance and its FREE evidence's age (Codex M-c round 10 P1 x2). Asking every arc in full every cycle cost 32 ms
    for two arcs and 150 ms for eight (2026-10-17), more than a live physics step."""

    def __init__(self, config, refresh_m: float = 0.25):
        self.config, self.refresh_m = config, refresh_m
        self._cache: dict = {}  # direction -> {"key", "perms": [(kappa, DrivePermission)], "since_m"}

    def advance(self, driven_m: float) -> None:
        """Distance driven since the last call (sim distance, both directions count), for every installed arc."""
        for entry in self._cache.values():
            entry["since_m"] += abs(driven_m)
            for _, perm in entry["perms"]:
                perm.advance(driven_m)

    def limit(self, snapshot, snapshot_key, now_s: float, sensors_last_s: dict, *, pose, curvatures, direction: int,
              check_speed_mps: float, footprint, own_footprint) -> tuple[float, str]:
        cfg = self.config
        ends = (round(curvatures[0] / KAPPA_STEP), round(curvatures[-1] / KAPPA_STEP))
        key = (snapshot_key, ends, len(curvatures))  # one arc -> two (steering starts) installs again (Codex r14)
        entry = self._cache.get(direction)
        if entry is None or entry["key"] != key or entry["since_m"] > self.refresh_m:
            length = cfg.lookahead_m + cfg.stopping.distance_m(check_speed_mps) + cfg.step_m
            perms = []
            for kappa in curvatures:
                perm = DrivePermission(cfg)
                path, _ = arc_poses(tuple(pose), kappa, direction, length, cfg.step_m)
                perm.update(snapshot, path, footprint, own_footprint, current_pose=pose, direction=direction)
                perms.append((kappa, perm))
            entry = self._cache[direction] = {"key": key, "perms": perms, "since_m": 0.0}
        best = (math.inf, "ok")
        perms = entry["perms"]
        for index, (_, perm) in enumerate(perms):
            if index in (0, len(perms) - 1):
                kappa_now = curvatures[0] if index == 0 else curvatures[-1]
                result = perm.allowed_speed(now_s, sensors_last_s, current_pose=pose, curvature_inv_m=kappa_now,
                                            direction=direction, footprint=footprint, own_footprint=own_footprint,
                                            speed_cap_mps=check_speed_mps)
            elif now_s - perm.path_check.oldest_free_s > cfg.evidence_max_age_s:
                result = (0.0, "evidence_stale")
            else:
                left = perm.path_check.verified_m - perm._driven_since_m
                result = (cfg.stopping.speed_for(left), perm.path_check.blocked or "ok")
            if result[0] < best[0]:
                best = result
        return best
