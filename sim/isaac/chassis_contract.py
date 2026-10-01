"""Check that an Isaac run's scene, chassis model and settings describe one truck.

The transport and SLAM runs take the truck physics from the base scene's
`/World/Forklift` but compute drive commands, odometry and insertion targets
from `--forklift-urdf`. Nothing ties the two together unless a run checks it,
so a scene imported from one model with the URDF of another would drive with
one wheel radius and command with another. The comparison is SDK-free; only
`stage_chassis` needs `pxr`, and it reads the layout Isaac 5.1's URDF importer
writes (joints under `<root>/joints` with `physics:body0`/`localPos0`, collision
shapes as scaled unit Cubes and Z-axis Cylinders under `<link>/collisions`).
"""

import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import NamedTuple

from insertion_geometry import _base_position_m, _boxes, _cylinder_radius_m

from forklift_core.control.path_tracking import AckermannGeometry

SCENE_JOINTS = ("left_steer", "right_steer", "rear_left_spin", "rear_right_spin")
WHEELS = ("front_left", "front_right", "rear_left", "rear_right")
# A difference strictly greater than these is refused.
POSITION_TOLERANCE_M = 0.001
LIMIT_TOLERANCE_RAD = 0.001

Vector = tuple[float, float, float]


class SceneJoint(NamedTuple):
    """Joint frame and rotation axis in base_link, and (lower, upper) limit.

    `limit_rad` is None for unlimited spin joints. `axis` is a unit vector and
    is compared with its sign.
    """

    position_m: Vector
    limit_rad: tuple[float, float] | None
    axis: Vector


class ChassisDescription(NamedTuple):
    """What a scene must reproduce: axle frames, tyres and carriage/fork boxes.

    Boxes map collision name to (centre, half extents) in base_link. `sources`
    is diagnostic only (what the stage reader saw) and is never compared.
    """

    joints: dict[str, SceneJoint]
    tyre_radii_m: dict[str, float]
    carriage_boxes: dict[str, tuple[Vector, Vector]]
    sources: dict = {}


def _finite(values, what: str) -> tuple[float, ...]:
    result = tuple(float(v) for v in values)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"Scene {what} is not finite: {result}")
    return result


def urdf_chassis(forklift_urdf: Path) -> ChassisDescription:
    """The chassis as the URDF defines it, with the lift lowered."""
    truck = ET.parse(forklift_urdf).getroot()
    joints = {}
    for name in SCENE_JOINTS:
        position = tuple(float(v) for v in _base_position_m(truck, name))
        joint = truck.find(f"joint[@name='{name}']")
        axis = tuple(float(v) for v in joint.find("axis").get("xyz").split())
        limit = joint.find("limit")
        bounds = None
        if name.endswith("_steer"):
            if limit is None:
                raise ValueError(f"Missing steering limit: {name}")
            bounds = (float(limit.get("lower")), float(limit.get("upper")))
        joints[name] = SceneJoint(position, bounds, axis)
    tyres = {wheel: _cylinder_radius_m(truck, f"{wheel}_wheel") for wheel in WHEELS}
    link = truck.find("link[@name='fork_carriage']")
    if link is None:
        raise ValueError("Missing fork_carriage link")
    lift = _base_position_m(truck, "fork_lift")
    boxes = {
        name: (tuple(float(v) for v in lift + center), tuple(float(v) for v in half))
        for name, center, half in _boxes(link)
    }
    return ChassisDescription(joints, tyres, boxes)


def stage_chassis(stage, root: str) -> ChassisDescription:
    """Read the same description from a USD stage, in base_link coordinates.

    Joint frames and axes are composed through their `physics:body0` link, so a
    body0 other than base_link is handled. base_link must sit in the world as a
    rigid transform -- a scaled truck would otherwise cancel out of every
    base-relative comparison. Collision boxes must be unrotated relative to
    base_link; tyres are Z-axis cylinders with equal radial scale.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    meters = UsdGeom.GetStageMetersPerUnit(stage)
    if not math.isclose(meters, 1.0):
        raise ValueError("Scene must be in metres")
    cache = UsdGeom.XformCache()
    base = stage.GetPrimAtPath(f"{root}/base_link")
    if not base.IsValid():
        raise ValueError(f"Scene has no {root}/base_link")
    base_world = cache.GetLocalToWorldTransform(base)
    rotation = [base_world.GetRow3(i) for i in range(3)]
    if base_world.GetDeterminant3() < 0 or any(
        abs(Gf.Dot(rotation[i], rotation[j]) - (1.0 if i == j else 0.0)) > 1e-6
        for i in range(3)
        for j in range(3)
    ):
        raise ValueError("Scene base_link is not a rigid (unscaled) transform")
    to_base = base_world.GetInverse()

    def in_base(prim):
        return cache.GetLocalToWorldTransform(prim) * to_base

    def scale_of(matrix):
        return tuple(matrix.GetRow3(i).GetLength() for i in range(3))

    def unrotated(matrix):
        scale = scale_of(matrix)
        return all(
            abs(matrix[i][j] - (scale[i] if i == j else 0.0)) <= 1e-6
            for i in range(3)
            for j in range(3)
        )

    unit_axes = {"X": Gf.Vec3d(1, 0, 0), "Y": Gf.Vec3d(0, 1, 0), "Z": Gf.Vec3d(0, 0, 1)}
    joints, sources = {}, {"meters_per_unit": meters, "joints": {}}
    for name in SCENE_JOINTS:
        prim = stage.GetPrimAtPath(f"{root}/joints/{name}")
        if not prim.IsValid():
            continue
        revolute = UsdPhysics.RevoluteJoint(prim)
        targets = revolute.GetBody0Rel().GetTargets()
        if len(targets) != 1:
            raise ValueError(f"Scene joint {name} needs exactly one body0")
        body0 = in_base(stage.GetPrimAtPath(targets[0]))
        local = Gf.Vec3d(
            *_finite(revolute.GetLocalPos0Attr().Get(), f"{name} localPos0")
        )
        point = _finite(body0.Transform(local), f"{name} position")
        token = revolute.GetAxisAttr().Get()
        if token not in unit_axes:
            raise ValueError(f"Scene joint {name} has no rotation axis")
        rot0 = revolute.GetLocalRot0Attr().Get() or Gf.Quatf(1, 0, 0, 0)
        in_body0 = Gf.Rotation(Gf.Quatd(rot0)).TransformDir(unit_axes[token])
        axis = body0.TransformDir(in_body0).GetNormalized()
        limit = None
        if name.endswith("_steer"):
            limit = tuple(
                math.radians(v)
                for v in _finite(
                    (
                        revolute.GetLowerLimitAttr().Get(),
                        revolute.GetUpperLimitAttr().Get(),
                    ),
                    f"{name} limit",
                )
            )
        joints[name] = SceneJoint(point, limit, _finite(axis, f"{name} axis"))
        sources["joints"][name] = {
            "prim": str(prim.GetPath()),
            "body0": str(targets[0]),
            "local_pos0_m": list(local),
            "axis_token": token,
        }

    tyres = {}
    for wheel in WHEELS:
        link = stage.GetPrimAtPath(f"{root}/{wheel}_wheel")
        found = []
        if link.IsValid():
            for prim in Usd.PrimRange(link, Usd.TraverseInstanceProxies()):
                if prim.IsA(UsdGeom.Cylinder) and prim.HasAPI(UsdPhysics.CollisionAPI):
                    cylinder = UsdGeom.Cylinder(prim)
                    matrix = in_base(prim)
                    sx, sy, _ = scale_of(matrix)
                    rolls = matrix.GetRow3(2).GetNormalized()
                    if (
                        cylinder.GetAxisAttr().Get() != "Z"
                        or abs(sx - sy) > 1e-6
                        or abs(abs(rolls[1]) - 1.0) > 1e-6
                    ):
                        raise ValueError(
                            f"Scene {wheel} tyre cylinder is not a y-rolling "
                            f"Z-axis cylinder: {prim.GetPath()}"
                        )
                    radius = cylinder.GetRadiusAttr().Get()
                    found.append(_finite((radius * sx,), f"{wheel} tyre radius")[0])
        if len(found) == 1:
            tyres[wheel] = found[0]

    boxes = {}
    carriage = stage.GetPrimAtPath(f"{root}/fork_carriage")
    if carriage.IsValid():
        for prim in Usd.PrimRange(carriage, Usd.TraverseInstanceProxies()):
            if not (prim.IsA(UsdGeom.Cube) and prim.HasAPI(UsdPhysics.CollisionAPI)):
                continue
            matrix = in_base(prim)
            if not unrotated(matrix):
                raise ValueError(f"Rotated carriage collision box: {prim.GetPath()}")
            size = UsdGeom.Cube(prim).GetSizeAttr().Get()
            name = prim.GetParent().GetName()
            boxes[name] = (
                _finite(matrix.ExtractTranslation(), f"{name} centre"),
                _finite((size * s / 2 for s in scale_of(matrix)), f"{name} size"),
            )
    return ChassisDescription(joints, tyres, boxes, sources)


def carriage_limit_from_boxes_m(chassis: ChassisDescription) -> float | None:
    """Fork box front minus the carriage cross members' front, as the URDF reader.

    None when the boxes it needs are absent (the comparison reports that).
    """
    boxes = chassis.carriage_boxes
    fronts = [
        center[0] + half[0]
        for name, (center, half) in boxes.items()
        if name.startswith("carriage_cross_")
    ]
    if "left_fork_collision" not in boxes or not fronts:
        return None
    fork_center, fork_half = boxes["left_fork_collision"]
    return fork_center[0] + fork_half[0] - max(fronts)


def chassis_record(chassis: ChassisDescription) -> dict:
    """JSON-ready account of what a scene check read and compared."""
    joints = {
        name: {
            **chassis.sources.get("joints", {}).get(name, {}),
            "position_m": list(joint.position_m),
            "axis": list(joint.axis),
            "limit_rad": None if joint.limit_rad is None else list(joint.limit_rad),
        }
        for name, joint in chassis.joints.items()
    }
    return {
        "meters_per_unit": chassis.sources.get("meters_per_unit"),
        "joints": joints,
        "tyre_radii_m": dict(chassis.tyre_radii_m),
        "carriage_box_count": len(chassis.carriage_boxes),
        "carriage_limit_m": carriage_limit_from_boxes_m(chassis),
    }


def _exceeds(value: float, tolerance: float) -> bool:
    """True unless value is a number within tolerance (NaN exceeds)."""
    return not value <= tolerance


def require_scene_matches_model(scene: ChassisDescription, forklift_urdf: Path) -> None:
    """Refuse a scene whose axles, axes, limits, tyres or carriage differ."""
    expected = urdf_chassis(forklift_urdf)
    for name, want in expected.joints.items():
        if name not in scene.joints:
            raise ValueError(f"Scene has no chassis joint {name}")
        got = scene.joints[name]
        offset = math.dist(got.position_m, want.position_m)
        if _exceeds(offset, POSITION_TOLERANCE_M):
            raise ValueError(
                f"Scene joint {name} is {offset * 1000:.1f} mm from {forklift_urdf}"
            )
        # Signed: a reversed steering axis turns the truck the other way.
        alignment = sum(a * b for a, b in zip(got.axis, want.axis, strict=True))
        if _exceeds(1.0 - alignment, 1e-6):
            raise ValueError(f"Scene joint {name} axis {got.axis} is not {want.axis}")
        if want.limit_rad is not None and (
            got.limit_rad is None
            or _exceeds(
                max(
                    abs(a - b)
                    for a, b in zip(got.limit_rad, want.limit_rad, strict=True)
                ),
                LIMIT_TOLERANCE_RAD,
            )
        ):
            raise ValueError(f"Scene joint {name} limit differs from {forklift_urdf}")
    for wheel, radius in expected.tyre_radii_m.items():
        got = scene.tyre_radii_m.get(wheel)
        if got is None or _exceeds(abs(got - radius), POSITION_TOLERANCE_M):
            raise ValueError(f"Scene {wheel} tyre radius {got} differs from {radius}")
    if set(scene.carriage_boxes) != set(expected.carriage_boxes):
        raise ValueError("Scene carriage/fork collision boxes differ by name")
    for name, (center, half) in expected.carriage_boxes.items():
        got_center, got_half = scene.carriage_boxes[name]
        if _exceeds(math.dist(got_center, center), POSITION_TOLERANCE_M) or _exceeds(
            max(abs(a - b) for a, b in zip(got_half, half, strict=True)),
            POSITION_TOLERANCE_M,
        ):
            raise ValueError(f"Scene collision box {name} differs from {forklift_urdf}")


def kinematic_curvature_limit_inv_m(geometry: AckermannGeometry) -> float:
    """Largest rear-axle curvature keeping both front wheels within the limit.

    Same clamp as `ackermann_command`: the inner wheel reaches the limit first.
    """
    tangent = math.tan(geometry.max_steering_rad)
    return tangent / (geometry.wheelbase_m + tangent * geometry.track_m / 2)


def require_curvature_within_model(settings: dict, geometry: AckermannGeometry) -> None:
    """Refuse planner/tracker curvature the model's steering cannot produce."""
    limit = kinematic_curvature_limit_inv_m(geometry)
    for key in ("tracker_curvature_inv_m", "planner_curvature_inv_m"):
        if settings[key] > limit:
            raise ValueError(
                f"{key} {settings[key]} exceeds the model's steering limit "
                f"{limit:.4f} 1/m; use settings made for this chassis model"
            )
