"""The teleoperated drive command (M-b plan B8; design 4.4): ratio to physical, the curvature rate limit, the dead-man
switch, the shared speed chain (creep, slew, final permission cap, preempt) and the arc-bundle permission.

The permission is asked on the arbiter's own ``DrivePermission`` (the mission's installed path is left alone): the
curvatures span the measured steering curvature and the output curvature, grown by ``curvature_margin_inv_m`` on
each side and rounded outward to the ``KAPPA_STEP`` grid; for each, the arc is installed with ``update()`` and
``allowed_speed`` read at that curvature; the smallest limit counts. The margin is there because the truck's path
does not curve as its steering joints say (Isaac, 2026-10-17: up to about 0.037 1/m off while the steering holds,
more while it moves -- docs/plans/2026-10-17-teleop-bundle-margin.md); the gaps between grid arcs are left to the
permission's envelope offset, which that plan checks over the distance a check is relied on. What that plan shows
(recorded Isaac runs, at the points and poses measured -- not a continuous guarantee): a stop begun at any checked
cycle kept the true body inside the checked region while the output curvature stayed inside the bundle; a new
input's path is the next check's (design 4.4).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.control.drive_permission import DrivePermission, arc_poses
from forklift_core.control.speed_chain import creep, slew_cap_stop

KAPPA_STEP = 0.05  # 1/m between the arcs of the bundle
REFRESH_M = 0.25  # BundlePermit installs again after this much driving
# An arc's walk needs reach only what its speed limit can use: the refresh distance and the stop from this speed,
# above every teleop limit, after which speed_for() returns more than any cap (teleop bundle margin plan D6). The
# 3 m lookahead walked four times as far for nothing and, with the margin's extra arcs, cost the live runner its
# pacing at f 0.08 (job 3889).
WALK_SPEED_MPS = 1.0


def walk_m(config, check_speed_mps: float) -> float:
    """The verified distance a teleop arc walks: the refresh distance and the stop from the larger of the check
    speed and WALK_SPEED_MPS, plus a sample."""
    return REFRESH_M + config.stopping.distance_m(max(WALK_SPEED_MPS, abs(check_speed_mps))) + config.step_m


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
    # The bundle's allowance for the path curving otherwise than the steering joints say. Measured on the Isaac
    # baseline plant only (docs/plans/2026-10-17-teleop-bundle-margin.md: 0.02 still let the body out of the
    # checked region in 5 of about 177,000 checks, 0.04 in none); the real truck's comes from H3.
    curvature_margin_inv_m: float = 0.04
    # Persistent console plan D14 (main session conditions 2, 4, 5): the manual lift's rate (within the autonomous
    # ramp's lift_rate_mps -- the runner checks), the recovery's straight reverse cap (below the autonomous withdraw's
    # 0.12 m/s) and the measured steering it moves within.
    lift_rate_mps: float = 0.02
    recovery_reverse_mps: float = 0.10
    recovery_steer_tol_rad: float = 0.01
    # Main session 2026-10-19 (condition 3 amended): an event stops the lift where it is -- the target is held within
    # this much of the measured lift, so a standing lift's gravity sag (target - measured, the sim's position drive:
    # carriage 1.6 kg x 9.81 / 1800 N/m = 8.7 mm plus the blades; 8.5 and 12.6 mm observed) leaves the target alone.
    # The sim's value; the real truck's waits for its actuator's measurement.
    lift_hold_tolerance_m: float = 0.02

    def __post_init__(self):
        for name in ("lift_rate_mps", "recovery_reverse_mps", "recovery_steer_tol_rad", "lift_hold_tolerance_m"):
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be a positive finite number")
        if self.direct_forward_mps > self.video_forward_mps or self.direct_reverse_mps > self.video_reverse_mps:
            raise ValueError("direct limits must not exceed video limits (design 5.3)")
        if not self.curvature_margin_inv_m >= 0:
            raise ValueError("the bundle's curvature margin must be non-negative")


def bundle(kappa_from: float, kappa_to: float, margin_inv_m: float = 0.0) -> list[float]:
    """The arcs' curvatures: the KAPPA_STEP grid points covering [low - margin, high + margin] of the two, the
    outer ones rounded outward (one arc only when that span is a single grid point). On the grid, the cached and
    the full checks install the same arcs, and an end that drifts inside its grid cell keeps them."""
    low, high = min(kappa_from, kappa_to) - margin_inv_m, max(kappa_from, kappa_to) + margin_inv_m
    first, last = math.floor(low / KAPPA_STEP + 1e-9), math.ceil(high / KAPPA_STEP - 1e-9)
    return [round(i * KAPPA_STEP, 12) for i in range(first, max(first, last) + 1)]


def arc_permit(permission: DrivePermission, snapshot, now_s: float, sensors_last_s: dict, *, pose, curvatures,
               direction: int, check_speed_mps: float, footprint, own_footprint) -> tuple[float, str]:
    """The smallest (limit, reason) over the bundle. Each arc is longer than the permission's walk so its end is
    never reached -- a teleoperated drive has no goal to slow down for."""
    cfg = permission.config
    walk = walk_m(cfg, check_speed_mps)
    length = walk + cfg.stopping.distance_m(check_speed_mps) + cfg.step_m  # past the walk: no end to stop at
    best = (math.inf, "ok")
    for kappa in curvatures:
        path, _ = arc_poses(tuple(pose), kappa, direction, length, cfg.step_m)
        permission.update(snapshot, path, footprint, own_footprint, current_pose=pose, direction=direction,
                          lookahead_m=walk)
        limit, why = permission.allowed_speed(now_s, sensors_last_s, current_pose=pose, curvature_inv_m=kappa,
                                              direction=direction, footprint=footprint, own_footprint=own_footprint,
                                              speed_cap_mps=check_speed_mps)
        if limit < best[0]:
            best = (limit, why)
    return best


class TeleopLift:
    """Plan D14 (5th ed.): the manual lift's target, in the autonomous ramp's form (a position moved at a capped
    rate, clipped to [0, the model's travel]). It starts from the target it takes over (never the measurement: a
    loaded lift sags) and moves while a permitted command's active stretch lasts -- TELEOP, armed, the dead-man
    held, a lift direction the policy allows, a measurement in hand. When the stretch ends, for any reason, the
    target is held within lift_hold_tolerance_m of the measured lift, once (main 2026-10-19: stop where it is -- a ramp
    that ran ahead stops near the lift, a standing lift's sag does not lower the target; the core's stop latch,
    core 3118, has the same concern). A lift never moved by hand keeps its target. ``step`` returns the target to
    apply, or None when the arbiter does not own the lift (the tick after TELEOP was left, which hands the last target
    over)."""

    def __init__(self, limits: TeleopLimits, *, limit_m: float, dt: float) -> None:
        if not (math.isfinite(limit_m) and limit_m > 0):
            raise ValueError("limit_m must be a positive finite number")
        self.limits, self.limit_m, self.dt = limits, float(limit_m), float(dt)
        self.target: float | None = None
        self.active = False

    def sync(self, value: float) -> None:
        """A target set from outside while owned -- the stop gate's latch (Codex D14 impl round 1 P1-1): the old
        target must not come back when the gate lifts."""
        if self.target is not None:
            self.target, self.active = float(value), False

    def step(self, teleop_mode: bool, teleop: dict | None, *, armed: bool, dirs, measured: float | None,
             start_from: float | None) -> float | None:
        measured = None if measured is None or not math.isfinite(measured) else float(measured)
        if not teleop_mode:
            if self.target is None:
                return None
            out = self.hold(self.target, measured) if self.active and measured is not None else self.target  # once
            self.target, self.active = None, False
            return out
        if self.target is None:
            self.target = float(start_from) if start_from is not None else (measured or 0.0)
        direction = int((teleop or {}).get("lift_dir") or 0)
        if (armed and teleop is not None and teleop.get("deadman") and direction != 0 and direction in dirs
                and measured is not None):
            step = direction * self.limits.lift_rate_mps * self.dt
            self.target = float(min(self.limit_m, max(0.0, self.target + step)))
            self.active = True
        elif self.active:
            if measured is not None:
                self.target = self.hold(self.target, measured)
            self.active = False
        return self.target

    def hold(self, target: float, measured: float) -> float:
        """Where an event stops the lift (main 2026-10-19): the target as it was, within the hold tolerance of the
        measurement -- a ramp that ran ahead stops near the lift; a standing lift's sag does not move the target."""
        a = self.limits.lift_hold_tolerance_m
        return float(min(measured + a, max(measured - a, target)))


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
             permit, mode: str = "normal", steering_rad: float | None = 0.0) -> tuple[float, float, tuple[float, str]]:
        """(speed, curvature, (limit, reason)) for this cycle. ``permit(curvatures, direction, check_speed)`` asks
        the arc bundle; ``steering_error_rad`` is how far the measured steering is from this cycle's target.
        ``mode`` "reverse_straight" (plan D14): curvature 0, backward only, at most recovery_reverse_mps, and no
        speed at all this cycle while the measured steering (``steering_rad``) is off zero."""
        lim = self.limits
        speed_target, kappa_target = self.target(teleop, loaded=loaded)
        straight = mode == "reverse_straight"
        if straight:
            speed_target, kappa_target = max(-lim.recovery_reverse_mps, min(0.0, speed_target)), 0.0
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
        if not drive_allowed:
            # The cap is 0 whatever the arcs say: they are not asked (the live runner spent about 1.7 wall s per sim
            # s on them while pacing forbade every drive, console session profile 2026-10-17).
            limit, why = 0.0, "drive_forbidden"
        elif straight and (steering_rad is None or abs(float(steering_rad)) > lim.recovery_steer_tol_rad):
            # Main condition 5: a straight command while the wheels still turn draws an arc -- no speed this cycle
            # (the cap is a step after the slew: the output is 0 now, Codex D14 design round 1 P1-2).
            limit, why = 0.0, "recovery:steering"
        else:
            arcs = bundle(measured_kappa, self.kappa, lim.curvature_margin_inv_m)
            limit, why = min((permit(arcs, d, check) for d in sorted(directions)), key=lambda r: r[0])
        speed = slew_cap_stop(last_applied_mps, creep(speed_target, steering_error_rad), decel_mps2=self.decel_mps2,
                              accel_mps2=lim.accel_mps2, dt=self.dt, cap_mps=max(0.0, limit), stop=preempt)
        return speed, self.kappa, (limit, why)


class BundlePermit:
    """The arc bundle's permission at the batch runner's cost (M-c live runs, plan C4a): per direction, the arcs'
    path checks are installed again only when the grid changed, the bundle's ends moved by a step or the truck has
    driven ``refresh_m`` since; between, each installed check is advanced by the distance driven, as the batch's
    obstacle layer does between scans. Every cycle: the two ends are asked in full (their emergency-stop arcs, the
    sensor silence, the evidence age), and every arc between by its advanced path distance and its FREE evidence's
    age (Codex M-c round 10 P1 x2). Asking every arc in full every cycle cost 32 ms for two arcs and 150 ms for eight
    (2026-10-17), more than a live physics step. The curvatures are on the ``bundle`` grid, so an installed set is
    exactly the set a full check would install, and the ends asked are its own outer arcs."""

    def __init__(self, config, refresh_m: float = REFRESH_M):
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
        ends = (round(curvatures[0] / KAPPA_STEP), round(curvatures[-1] / KAPPA_STEP))  # grid indices (bundle)
        key = (snapshot_key, ends, len(curvatures))
        entry = self._cache.get(direction)
        if entry is None or entry["key"] != key or entry["since_m"] > self.refresh_m:
            walk = walk_m(cfg, check_speed_mps) + max(0.0, self.refresh_m - REFRESH_M)
            length = walk + cfg.stopping.distance_m(check_speed_mps) + cfg.step_m
            perms = []
            for kappa in curvatures:
                perm = DrivePermission(cfg)
                path, _ = arc_poses(tuple(pose), kappa, direction, length, cfg.step_m)
                perm.update(snapshot, path, footprint, own_footprint, current_pose=pose, direction=direction,
                            lookahead_m=walk)
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
