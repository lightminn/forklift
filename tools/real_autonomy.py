#!/usr/bin/env python3
"""The real truck's autonomy executor (docs/design/2026-11-19-real-assembly.md). 4b: ``--dry-assemble`` only -- the
core is assembled from the repository's real settings without opening a device, the assembly record written, and
the exit code says whether the assembly is structurally sound (0) or refused (2). Readiness needs the link, the SLAM
link and the layer, which a dry assembly does not have: it is recorded as not ready, not as an error. The 50 Hz loop
is 4c."""

from __future__ import annotations

import argparse
import importlib.util
import json
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[1]


def _insertion_geometry():
    spec = importlib.util.spec_from_file_location("insertion_geometry", REPO / "sim/isaac/insertion_geometry.py")
    module = importlib.util.module_from_spec(spec)
    sys.modules["insertion_geometry"] = module
    spec.loader.exec_module(module)
    return module


def mission_geometry(urdf: Path, depth_m: float, width_m: float, reserve_m: float = 0.016):
    from forklift_core.planning.geometry import Footprint
    from forklift_core.planning.pallet_mission import SyntheticMissionGeometry

    ig = _insertion_geometry()
    axle_to_tip, _ = ig.read_chassis_reference_m(urdf)
    return SyntheticMissionGeometry(unloaded_footprint=Footprint(axle_to_tip, 0.17, 0.36), pallet_depth_m=depth_m,
                                    pallet_width_m=width_m, axle_to_fork_tip_m=axle_to_tip,
                                    carriage_limit_m=ig.read_carriage_limit_m(urdf), insertion_reserve_m=reserve_m)


def map_occupancy(grid):
    """A surveyed map's grid (rows = y) as the planner's occupancy (cells x by y); a rotated origin is refused."""
    import numpy as np

    from forklift_core.planning.grid_collision import OccupancyGrid

    if abs(float(grid.origin[2])) > 1e-9:
        raise ValueError("map_origin_rotated")
    return OccupancyGrid(float(grid.origin[0]), float(grid.origin[1]), float(grid.resolution_m),
                         np.asarray(grid.occupied()).T)


def _sha(path: Path) -> str:
    import hashlib

    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dry_assemble(s10a_path: Path, settings_path: Path, vehicle_path: Path, map_bundle: Path | None,
                 out: Path | None):
    import yaml

    from forklift_core.live import real_assembly as asm
    from forklift_core.live import real_autonomy as ra
    from forklift_core.plant.kinematics import RealKinematics
    from forklift_core.plant.real_vehicle import chassis_model_path, load_real_vehicle
    from forklift_core.planning import Pose2D
    from forklift_core.planning import pallet_mission as pm

    record = {"s10a": str(s10a_path), "settings": str(settings_path), "vehicle": str(vehicle_path),
              "map_bundle": None if map_bundle is None else str(map_bundle)}
    try:
        record["hashes"] = {name: _sha(path) for name, path in (("s10a", s10a_path), ("settings", settings_path),
                                                                 ("vehicle", vehicle_path))}
        cfg = ra.load_s10a(s10a_path)
        data = yaml.safe_load(Path(settings_path).read_text())
        for key in ("lift_target_m", "lift_rate_mps", "drive_acceleration_mps2", "withdraw_speed_mps", "speeds",
                    "mission_stages"):
            if key not in data:
                raise KeyError(f"setting_missing:{key}")
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as err:
        record.update(assembled=False, refused=[f"settings_unreadable:{err}"])
        _write(record, out)
        return 2
    occupancy, map_id = None, None
    if map_bundle is not None:
        from forklift_core.localization.slam_map import MapBundleError, load_bundle

        try:
            bundle = load_bundle(map_bundle, expected_map_id=cfg.map_id)
            map_id, occupancy = bundle.map_id, map_occupancy(bundle.grid)
        except (MapBundleError, ValueError) as err:
            record.update(assembled=False, refused=[f"map:{err}"])
            _write(record, out)
            return 2
    try:
        vehicle = load_real_vehicle(vehicle_path)
        kin = RealKinematics(vehicle)
    except Exception as err:  # noqa: BLE001 -- any malformed vehicle file is a structural refusal (exit 2)
        record.update(assembled=False, refused=[f"vehicle_unreadable:{type(err).__name__}:{err}"])
        _write(record, out)
        return 2
    low, high = kin.cfg.steering.curvature_range
    cap = min(-float(low), float(high))
    settings = {k: data[k] for k in ("lift_target_m", "lift_rate_mps", "drive_acceleration_mps2",
                                     "withdraw_speed_mps")}
    settings.update(steering_command_rate_rad_s=kin.steering_rate_rad_s, tracker_curvature_inv_m=cap)
    planner = pm.make_transport_planner_config(curvature_limit_inv_m=cap)
    try:
        if cfg.pallet_depth_m is None or cfg.pallet_width_m is None:
            raise asm.AssemblyRefused(ra.s10a_refusal(cfg, map_id) + (["check_pallet_pose_unset"]
                                                                       if cfg.check_pallet_pose is None else []))
        geometry = mission_geometry(chassis_model_path(vehicle_path).parent / "forklift.urdf",
                                    cfg.pallet_depth_m, cfg.pallet_width_m)
        hooks = asm.RealHooks(lift_m=lambda: float("nan"), odom_base=lambda: None, ready=lambda t, i: None,
                              started=lambda *a: None, record=lambda *a: None, geometry=geometry)
        core = asm.assemble(
            cfg=cfg, loaded_map_id=map_id, geometry=geometry, pallet_geometry=None, kinematics=kin,
            settings=settings, planner_config=planner, travel_config=planner, speeds=data["speeds"],
            mission_stages=data["mission_stages"], curvatures={"preview": cap},
            layer=SimpleNamespace(known_enabled=False), applied=(0.0, 0.0, 0.0),
            slam_tracker=SimpleNamespace(mode="none", applied=None), jobs=None, hooks=hooks,
            extra_options={"cusp_heading_hold_rad": 0.05, "cusp_heading_hold_phases": ("return_home",),
                           "tracker_profile": None})
    except asm.AssemblyRefused as err:
        record.update(assembled=False, refused=err.reasons)
        _write(record, out)
        return 2
    from forklift_core.planning.geometry import Rectangle

    check = pm.PalletSite(*cfg.check_pallet_pose)
    plan = pm.plan_transport(replace(asm.scenario_of(cfg), props=()), planner, geometry=geometry,
                             target_pickup=check, start_rear=Pose2D(*cfg.start_rear), return_to=Pose2D(*cfg.wait_pose),
                             occupancy=occupancy, pickup_obstacle=Rectangle(check.x_m, check.y_m, geometry.pallet_depth_m,
                                                                            geometry.pallet_width_m, check.yaw_rad))
    record.update(asm.assembly_record(core, cfg, {"planner": cap, "tracker": cap, "preview": cap}),
                  assembled=True, check_plan=plan.status, ready="not_ready:link:no_state (dry assembly)")
    _write(record, out)
    return 0 if plan.success else 2


def _write(record: dict, out: Path | None) -> None:
    text = json.dumps(record, indent=2, default=str)
    if out is not None:
        out.write_text(text)
    print(text)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dry-assemble", action="store_true", required=True)
    ap.add_argument("--s10a", type=Path, default=REPO / "config/real/s10a.yaml")
    ap.add_argument("--settings", type=Path, default=REPO / "config/real/autonomy.yaml")
    ap.add_argument("--vehicle", type=Path, default=REPO / "config/real/vehicle.yaml")
    ap.add_argument("--map-bundle", type=Path, default=None,
                    help="the surveyed map bundle (verified by its manifest; its map id must be s10a.yaml's)")
    ap.add_argument("--out", type=Path, default=None)
    args = ap.parse_args(argv)
    return dry_assemble(args.s10a, args.settings, args.vehicle, args.map_bundle, args.out)


if __name__ == "__main__":
    sys.exit(main())
