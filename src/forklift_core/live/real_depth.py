"""The real D435i D5 pocket check's generator (PR 7 7e-2a, docs/plans/2026-11-26-7e-depth-pocket-check.md 6).

``make_pocket_check`` builds ``sim/isaac/pocket_check.py``'s ``PocketCheck`` -- the very check Isaac runs -- with the
real path's options (plan 6.2): the frames' own noise only, novelty by device identity, the usable-observation
freshness, the three-state expected visibility with the error budget, refutation, the guard, and the chassis model's
own boxes. Its inputs:

* the pallet: the operator's map pose (``estimate``) and the S10-A ``pallet`` entry; the shape is the file
  ``config/real/pallet_geometry_<kind>.yaml`` -- none is ``pallet_geometry_unset``, outer dimensions more than 1 mm
  from the entry's ``depth_m``/``width_m`` are ``pallet_geometry_mismatch``;
* the chassis model URDF: the blades, body and insertion depth as Isaac computes them, ``base_z_m`` (the tyres stand
  on the floor), and every ``fork_carriage`` collision box as the truck's own surface (``own_boxes_rear``);
* the stream's recorded intrinsics: a pinhole camera; a distortion that moves a pixel more than
  ``distortion_max_px`` is ``depth_distortion`` (plan 4.3), an unknown model with non-zero coefficients
  ``depth_distortion_model``;
* the depth config's ``pocket`` section: ``sigma_a``, ``surface_estimate_m``, ``lifetime_s``, the usable-observation
  thresholds and the error budget. A budget term still null is ``pocket_budget_unset:<term>`` -- the real D5 runs
  only under the ``d5_pocket`` bypass until the bench enters it (plan 6.2, 10).

A refusal raises ``PocketRefused`` (its ``reason``); a check whose volume meets the pallet comes back with
``valid`` False and its ``invalid_reason``, as in Isaac (plan 6.3)."""

from __future__ import annotations

import importlib.util
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[3]
GEOMETRY_DIR = REPO / "config" / "real"
DIM_TOL_M = 0.001
BROWN = ("brown_conrady", "modified_brown_conrady", "inverse_brown_conrady")


class PocketRefused(ValueError):
    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = reason


def _sim_module(name: str):
    if name in sys.modules:
        return sys.modules[name]
    spec = importlib.util.spec_from_file_location(name, REPO / "sim" / "isaac" / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def pocket_module():
    """``sim/isaac/pocket_check.py`` (loaded once, under its own name, as the runner and ``d5_replay`` do)."""
    return _sim_module("pocket_check")


# --- the pallet ------------------------------------------------------------------------------------------------------
def load_real_pallet_geometry(pallet: dict, folder=GEOMETRY_DIR):
    """The S10-A pallet entry's shape file, checked against its measured outer dimensions."""
    from forklift_core.perception.pallet_geometry import load_pallet_geometry

    kind = pallet.get("kind")
    depth, width = pallet.get("depth_m"), pallet.get("width_m")
    path = Path(folder) / f"pallet_geometry_{kind}.yaml"
    if not kind or depth is None or width is None or not path.is_file():
        raise PocketRefused("pallet_geometry_unset")
    try:
        geometry = load_pallet_geometry(path)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        raise PocketRefused(f"pallet_geometry_unreadable:{exc}") from exc
    if (abs(geometry.overall_depth_m - float(depth)) > DIM_TOL_M
            or abs(geometry.overall_width_m - float(width)) > DIM_TOL_M):
        raise PocketRefused("pallet_geometry_mismatch")
    return geometry, path


# --- the chassis model -----------------------------------------------------------------------------------------------
def base_z_from_urdf(urdf) -> float:
    """Floor to ``base_link`` origin: the rear tyre's radius minus its axle's base-frame height (plan 5; Isaac's
    ``base_now[2]`` on a flat floor)."""
    ig = _sim_module("insertion_geometry")
    truck = ET.parse(urdf).getroot()
    radius = ig._cylinder_radius_m(truck, "rear_left_wheel")
    axle = ig._base_position_m(truck, "rear_left_spin")
    return float(radius - axle[2])


def own_boxes_rear(urdf, rear_axle_offset_m: float, base_z_m: float) -> tuple:
    """Every ``fork_carriage`` collision box (blades, heels, carriage sides, cross bars, grille) at lift 0, as
    ``(x0, x1, y0, y1, z0, z1)`` in the rear-axle frame with z from the floor (plan 6.2 ``own_boxes_from=urdf``)."""
    ig = _sim_module("insertion_geometry")
    truck = ET.parse(urdf).getroot()
    carriage = ig._base_position_m(truck, "fork_lift")
    link = truck.find("link[@name='fork_carriage']")
    if link is None:
        raise PocketRefused("own_boxes:fork_carriage_missing")
    out = []
    for _name, centre, half in ig._boxes(link):
        c = carriage + centre
        out.append((float(c[0] - half[0] + rear_axle_offset_m), float(c[0] + half[0] + rear_axle_offset_m),
                    float(c[1] - half[1]), float(c[1] + half[1]),
                    float(c[2] - half[2] + base_z_m), float(c[2] + half[2] + base_z_m)))
    return tuple(out)


# --- the camera ------------------------------------------------------------------------------------------------------
def distortion_shift_px(intr: dict, step: int = 8) -> float:
    """The largest pixel shift the recorded distortion causes over the image (plan 4.3): 0 with every coefficient 0;
    a Brown-Conrady family model evaluated forward (k1, k2, p1, p2, k3) on a grid with the borders; any other model
    with a non-zero coefficient refuses."""
    coeffs = [float(c) for c in intr.get("coeffs") or ()]
    if not any(coeffs):
        return 0.0
    model = str(intr.get("model", "")).rsplit(".", 1)[-1].lower()
    if model not in BROWN or len(coeffs) < 5:
        raise PocketRefused("depth_distortion_model")
    k1, k2, p1, p2, k3 = coeffs[:5]
    w, h = int(intr["width"]), int(intr["height"])
    fx, fy, cx, cy = (float(intr[k]) for k in ("fx", "fy", "ppx", "ppy"))
    us = np.unique(np.r_[np.arange(0, w, step), w - 1])
    vs = np.unique(np.r_[np.arange(0, h, step), h - 1])
    u, v = np.meshgrid(us, vs)
    x, y = (u - cx) / fx, (v - cy) / fy
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2 ** 2 + k3 * r2 ** 3
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return float(np.max(np.hypot(fx * (xd - x), fy * (yd - y))))


def camera_from_intrinsics(intr: dict, sigma_a: float, distortion_max_px: float):
    from forklift_core.perception.pocket_clearance import DepthCamera

    shift = distortion_shift_px(intr)
    if shift > distortion_max_px:
        raise PocketRefused("depth_distortion")
    return DepthCamera(float(intr["fx"]), float(intr["fy"]), float(intr["ppx"]), float(intr["ppy"]),
                       int(intr["width"]), int(intr["height"]), sigma_a=float(sigma_a))


# --- the budget ------------------------------------------------------------------------------------------------------
def pocket_budget(pocket: dict) -> dict:
    """The depth config's ``pocket`` section as the check's numbers; a budget term still null refuses."""
    err = dict(pocket.get("error") or {})
    for key in ("operator_m", "slam_m", "own_m", "cam_trans_m", "cam_rot_deg"):
        if err.get(key) is None:
            raise PocketRefused(f"pocket_budget_unset:{key}")
    for key in ("sigma_a", "surface_estimate_m", "lifetime_s", "view_min_px", "view_valid_min"):
        if pocket.get(key) is None:
            raise PocketRefused(f"pocket_budget_unset:{key}")
    return {"pallet_rel_error_m": float(err["operator_m"]) + float(err["slam_m"]),
            "operator_m": float(err["operator_m"]), "slam_m": float(err["slam_m"]),
            "own_error_m": float(err["own_m"]), "cam_trans_error_m": float(err["cam_trans_m"]),
            "cam_rot_error_rad": math.radians(float(err["cam_rot_deg"])),
            "sigma_a": float(pocket["sigma_a"]), "surface_estimate_m": float(pocket["surface_estimate_m"]),
            "lifetime_s": float(pocket["lifetime_s"]), "view_min_px": int(pocket["view_min_px"]),
            "view_valid_min": float(pocket["view_valid_min"])}


def make_pocket_check(*, estimate, axis_yaw: float, pallet: dict, urdf, intrinsics: dict, base_from_optical,
                      stopping, pocket: dict, distortion_max_px: float, guard=None, noise_seed: int = 0,
                      geometry_dir=GEOMETRY_DIR, reserve_m: float = 0.016):
    """The real path's ``PocketCheck`` (module text). ``estimate`` the operator's map pose (x, y, yaw), ``pallet``
    the S10-A entry (``kind``, ``depth_m``, ``width_m``), ``base_from_optical`` (R, t) base_link <- optical at lift 0,
    ``stopping`` the layer's ``StoppingModel``, ``pocket`` the depth config's section, ``guard`` the source's
    ``() -> (generation, correction)``."""
    from forklift_core.live.real_run import chassis_reference, mission_geometry

    pc = pocket_module()
    geometry, _path = load_real_pallet_geometry(pallet, geometry_dir)
    budget = pocket_budget(pocket)
    camera = camera_from_intrinsics(intrinsics, budget["sigma_a"], distortion_max_px)
    urdf = Path(urdf)
    ig = _sim_module("insertion_geometry")
    mission = mission_geometry(urdf, geometry.overall_depth_m, geometry.overall_width_m, reserve_m)
    _tip, offset, _limit = chassis_reference(urdf)
    base_z = base_z_from_urdf(urdf)
    real = pc.RealPocketOptions(
        view_min_px=budget["view_min_px"], view_valid_min=budget["view_valid_min"],
        pallet_rel_error_m=budget["pallet_rel_error_m"], own_error_m=budget["own_error_m"],
        cam_trans_error_m=budget["cam_trans_error_m"], cam_rot_error_rad=budget["cam_rot_error_rad"],
        guard=guard, own_boxes_rear=own_boxes_rear(urdf, offset, base_z))
    rot, trans = (np.asarray(a, dtype=float) for a in base_from_optical)
    return pc.PocketCheck(
        estimate=tuple(float(v) for v in estimate), axis_yaw=float(axis_yaw), pallet_geometry=geometry,
        blades_rear=tuple((x0 + offset, x1 + offset, y0, y1) for x0, x1, y0, y1 in ig.read_fork_blades_m(urdf)),
        body_front_m=mission.axle_to_fork_tip_m - mission.carriage_limit_m,
        body_rear_m=mission.unloaded_footprint.rear_m, body_half_width_m=mission.unloaded_footprint.half_width_m,
        insertion_depth_m=mission.axle_to_fork_tip_m + mission.pallet_depth_m / 2 - mission.inserted_offset_m,
        base_z_m=base_z, base_from_optical=(rot, trans), rear_axle_offset_m=offset, camera=camera,
        stopping=stopping,
        config=pc.PocketConfig(surface_estimate_m=budget["surface_estimate_m"], lifetime_s=budget["lifetime_s"]),
        noise_seed=int(noise_seed), synthetic_noise=False, real=real)


__all__ = ["PocketRefused", "base_z_from_urdf", "camera_from_intrinsics", "distortion_shift_px",
           "load_real_pallet_geometry", "make_pocket_check", "own_boxes_rear", "pocket_budget", "pocket_module"]
