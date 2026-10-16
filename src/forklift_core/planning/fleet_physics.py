"""Load-dependent driving limits of a forklift, and the speed profile they allow.

Plan: docs/plans/2026-10-12-fleet-physical-time-windows.md. A quasi-static
planar model in the chassis frame (x forward, z up, origin at base_link):

- **Mass and centre of gravity.** The chassis is the URDF's links as point
  masses at their centres of mass (the carriage raised by the lift); the load
  is one more point mass on the forks. Lifting a load moves the combined
  centre of gravity forward and up.
- **Tipping.** On flat ground the wheels push up at the axles. An
  acceleration a_x (+ forward) moves weight between the axles:
  N_front = m [g (x - x_rear) - a_x h] / L and N_rear = m [g (x_front - x)
  + a_x h] / L. Tipping starts when one goes to zero, so
  a_x <= g (x - x_rear) / h (forward, rear-tip) and
  -a_x <= g (x_front - x) / h (backward, front-tip: braking with the load
  ahead); sideways a_y <= g (track / 2) / h. Only ``share`` of each is used.
- **Traction and drive.** Every wheel drives: |a| <= mu g, and the wheel
  torques give at most n tau / (r m). The wheel speed limit caps the speed.
- **Control caps.** The tracker's acceleration and each stage's cruise speed
  are settings, not physics; they cap what the physics allows.

``speed_profile`` turns the limits into a speed along a path: the cruise
cap, the curve speed sqrt(a_y / kappa), a stop at each gear change and at the
end, then a forward pass (accelerating) and a backward pass (braking), each
with the limit of the direction the force acts in. Its times are what the
truck takes when nothing holds it up.

Every number from the chassis model beyond its measured dimensions is an
estimate (sim/models/dls08_measured/README.md); so is the load.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from math import sqrt

import numpy as np

G = 9.81


@dataclass(frozen=True)
class Mass:
    kg: float
    x_m: float
    z_m: float


@dataclass(frozen=True)
class Chassis:
    """What the limits need from the truck (chassis frame, metres)."""

    parts: tuple[Mass, ...]  # without the carriage
    carriage: Mass  # at the lowest lift
    front_axle_x_m: float
    rear_axle_x_m: float
    track_m: float
    wheel_radius_m: float
    wheels: int = 4
    wheel_torque_nm: float = 3.0
    wheel_speed_radps: float = 8.0
    friction: float = 0.8
    lift_force_n: float = 180.0
    fork_height_m: float = 0.04  # fork centre above the floor at the lowest lift
    fork_tip_x_m: float = 0.95

    @classmethod
    def from_urdf(cls, path, **settings) -> Chassis:
        """The URDF's links as point masses; carriage = the fork_lift child."""
        root = ET.parse(path).getroot()
        joints = {}
        for joint in root.findall("joint"):
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
            joints[joint.find("child").get("link")] = (joint.get("name"), joint.find("parent").get("link"), xyz)

        def place(link):
            x = z = 0.0
            while link in joints:
                _, parent, xyz = joints[link]
                x, z = x + xyz[0], z + xyz[2]
                link = parent
            return x, z

        parts, carriage, axles = [], None, {}
        for link in root.findall("link"):
            name = link.get("name")
            inertial = link.find("inertial")
            if inertial is None:
                continue
            origin = inertial.find("origin")
            com = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
            x, z = place(name)
            mass = Mass(float(inertial.find("mass").get("value")), x + com[0], z + com[2])
            if joints.get(name, ("",))[0] == "fork_lift":
                carriage = mass
            else:
                parts.append(mass)
            if name.endswith("_wheel"):
                axles.setdefault("front" if name.startswith("front") else "rear", []).append((x, z))
        fronts, rears = axles["front"], axles["rear"]
        lefts = [j[2][1] for n, j in joints.items() if n.endswith("_wheel") or n.endswith("_carrier")]
        tip = joints.get("left_fork_tip")
        return cls(
            parts=tuple(parts),
            carriage=carriage,
            front_axle_x_m=float(np.mean([p[0] for p in fronts])),
            rear_axle_x_m=float(np.mean([p[0] for p in rears])),
            track_m=2 * max(abs(v) for v in lefts),
            wheel_radius_m=float(np.mean([p[1] for p in fronts + rears])),
            fork_tip_x_m=tip[2][0] if tip else 0.95,
            fork_height_m=tip[2][2] if tip else 0.04,
            **settings,
        )

    @property
    def mass_kg(self) -> float:
        return sum(p.kg for p in self.parts) + self.carriage.kg


@dataclass(frozen=True)
class Load:
    """What the forks carry, as one point mass: the pallet's centre of mass
    with the forks ``insert_m`` into a pallet ``depth_m`` deep, resting on
    them (its fork opening's top ``opening_top_m`` above its base, its
    centre of mass ``com_height_m`` above its base)."""

    kg: float
    depth_m: float = 0.6
    insert_m: float = 0.30
    opening_top_m: float = 0.100
    com_height_m: float = 0.072
    fork_thickness_m: float = 0.025

    def mass(self, chassis: Chassis, lift_m: float) -> Mass:
        x = chassis.fork_tip_x_m - self.insert_m + self.depth_m / 2
        fork_top = chassis.fork_height_m + lift_m + self.fork_thickness_m / 2
        return Mass(self.kg, x, fork_top - self.opening_top_m + self.com_height_m)


@dataclass(frozen=True)
class Limits:
    """Accelerations (m/s^2, magnitudes) and the speed cap (m/s) one state allows."""

    mass_kg: float
    cog_x_m: float
    cog_z_m: float
    forward: float  # acceleration toward +x (speeding up forward, braking backward)
    backward: float  # toward -x (braking forward, speeding up backward)
    lateral: float
    speed: float
    tip_forward: float  # the tipping bounds before ``share``
    tip_backward: float
    tip_lateral: float
    traction: float
    drive: float
    static_margin_m: float  # combined centre of gravity to the front axle

    def along(self, direction: int, speeding_up: bool) -> float:
        """The acceleration limit of a move in gear direction (+1/-1)."""
        toward_front = (direction > 0) == speeding_up
        return self.forward if toward_front else self.backward


def limits(chassis: Chassis, load: Load | None = None, lift_m: float = 0.0, share: float = 0.5, cap_mps2: float | None = None) -> Limits:
    """The state's limits; share of each tipping bound is used, cap_mps2 (the
    tracker's acceleration) caps the longitudinal ones."""
    carriage = replace(chassis.carriage, z_m=chassis.carriage.z_m + lift_m)
    parts = [*chassis.parts, carriage] + ([load.mass(chassis, lift_m)] if load is not None and load.kg > 0 else [])
    m = sum(p.kg for p in parts)
    x = sum(p.kg * p.x_m for p in parts) / m
    z = sum(p.kg * p.z_m for p in parts) / m
    tip_fwd = G * (x - chassis.rear_axle_x_m) / z
    tip_back = G * (chassis.front_axle_x_m - x) / z
    tip_lat = G * (chassis.track_m / 2) / z
    traction = chassis.friction * G
    drive = chassis.wheels * chassis.wheel_torque_nm / chassis.wheel_radius_m / m
    cap = np.inf if cap_mps2 is None else cap_mps2
    forward = max(min(share * tip_fwd, traction, drive, cap), 0.0)
    backward = max(min(share * tip_back, traction, drive, cap), 0.0)
    lateral = max(min(share * tip_lat, traction), 0.0)
    return Limits(
        m, x, z, forward, backward, lateral, chassis.wheel_speed_radps * chassis.wheel_radius_m,
        tip_fwd, tip_back, tip_lat, traction, drive, chassis.front_axle_x_m - x,
    )


def curvature(poses: np.ndarray) -> np.ndarray:
    """|d yaw / d s| at each pose of a (N, 3) path (0 where it does not move)."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    if len(poses) < 3:
        return np.zeros(len(poses))
    step = np.hypot(*np.diff(poses[:, :2], axis=0).T)
    turn = np.abs((np.diff(poses[:, 2]) + np.pi) % (2 * np.pi) - np.pi)
    k = np.where(step > 1e-9, turn / np.maximum(step, 1e-9), 0.0)
    return np.maximum(np.r_[k, 0.0], np.r_[0.0, k])


def gears(poses: np.ndarray) -> np.ndarray:
    """+1 forward / -1 reverse for each step of a path (heading vs travel)."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    d = np.diff(poses[:, :2], axis=0)
    ahead = d[:, 0] * np.cos(poses[:-1, 2]) + d[:, 1] * np.sin(poses[:-1, 2]) >= 0
    step = np.hypot(d[:, 0], d[:, 1])
    out = []
    for k in range(len(step)):  # zero-length steps keep the gear before them
        out.append((1 if ahead[k] else -1) if step[k] > 1e-9 or not out else out[-1])
    return np.asarray(out, dtype=int)


def speed_profile(poses: np.ndarray, cruise_mps: float, lim: Limits) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """(arc length, speed, time) at each pose: cruise and curve caps, a stop
    at each gear change and at both ends, accelerating and braking with the
    limit of the direction the force acts in."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(poses[:, :2], axis=0).T))]
    if len(poses) < 2 or s[-1] <= 0:
        return s, np.zeros(len(s)), np.zeros(len(s))
    gear = gears(poses)
    kappa = curvature(poses)
    cap = np.minimum(min(cruise_mps, lim.speed), np.sqrt(lim.lateral / np.maximum(kappa, 1e-9)))
    cap[0] = cap[-1] = 0.0
    turn = np.nonzero(np.diff(gear))[0] + 1  # pose index where the gear changes
    cap[turn] = 0.0
    step_gear = np.r_[gear, gear[-1]]  # the gear of the step leaving each pose
    v = cap.copy()
    ds = np.diff(s)
    for i in range(1, len(s)):  # speeding up, in the gear of the step i-1 -> i
        a = lim.along(int(gear[i - 1]), speeding_up=True)
        v[i] = min(v[i], sqrt(v[i - 1] ** 2 + 2 * a * ds[i - 1]))
    for i in range(len(s) - 2, -1, -1):  # braking toward pose i+1
        a = lim.along(int(step_gear[i]), speeding_up=False)
        v[i] = min(v[i], sqrt(v[i + 1] ** 2 + 2 * a * ds[i]))
    mean = (v[:-1] + v[1:]) / 2
    dt = np.where(mean > 1e-9, ds / np.maximum(mean, 1e-9), 0.0)
    # A step that starts and ends at rest (both ends capped to 0) still takes time.
    rest = (mean <= 1e-9) & (ds > 1e-9)
    if np.any(rest):
        a = np.array([lim.along(int(g), True) for g in gear[rest]])
        dt[rest] = 2 * np.sqrt(ds[rest] / np.maximum(a, 1e-9))
    return s, v, np.r_[0.0, np.cumsum(dt)]


def speed_caps(poses: np.ndarray, cruise_mps: float, lim: Limits) -> tuple[np.ndarray, np.ndarray]:
    """(arc length, speed cap) for a follower that speeds up by itself: the
    cruise and curve caps and the braking toward each gear change and the
    end -- the profile without its start from rest."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    s = np.r_[0.0, np.cumsum(np.hypot(*np.diff(poses[:, :2], axis=0).T))]
    if len(poses) < 2 or s[-1] <= 0:
        return s, np.zeros(len(s))
    gear = gears(poses)
    cap = np.minimum(min(cruise_mps, lim.speed), np.sqrt(lim.lateral / np.maximum(curvature(poses), 1e-9)))
    cap[-1] = 0.0
    cap[np.nonzero(np.diff(gear))[0] + 1] = 0.0
    step_gear = np.r_[gear, gear[-1]]
    ds = np.diff(s)
    for i in range(len(s) - 2, -1, -1):
        cap[i] = min(cap[i], sqrt(cap[i + 1] ** 2 + 2 * lim.along(int(step_gear[i]), speeding_up=False) * ds[i]))
    return s, cap


def stop_cost_s(speed_mps: float, lim: Limits, direction: int = 1) -> float:
    """Time lost by stopping from speed and speeding up again, against
    driving through at that speed: v / (2 a_brake) + v / (2 a_up)."""
    if speed_mps <= 0:
        return 0.0
    brake = lim.along(direction, speeding_up=False)
    up = lim.along(direction, speeding_up=True)
    return speed_mps / (2 * max(brake, 1e-9)) + speed_mps / (2 * max(up, 1e-9))
