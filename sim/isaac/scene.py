"""Isaac-only scene construction; import after starting SimulationApp."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, UsdShade

from forklift_core.planning.pallet_mission import AssetSpec

FACTORY_PROPS = (
    "SM_BarelPlastic_A_01.usd",
    "SM_CratePlastic_D_01.usd",
    "SM_CardBoxA_02.usd",
)


def bounds_of(prim: Usd.Prim) -> tuple[np.ndarray, np.ndarray]:
    """World bounds include visual geometry, conservatively enclosing props."""
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    low, high = np.array(box.GetMin()), np.array(box.GetMax())
    if not np.isfinite([low, high]).all() or np.any(high <= low):
        raise RuntimeError(f"Asset bounds are missing or invalid: {prim.GetPath()}")
    return low, high


def read_catalogue(stage: Usd.Stage, app, asset_root: str) -> tuple[list, dict]:
    """Read dimensions from official USD assets, retaining origin offsets."""
    catalogue, offsets = [], {}
    for index, filename in enumerate(FACTORY_PROPS):
        uri = asset_root.rstrip("/") + "/" + filename
        prim = stage.DefinePrim(f"/World/Catalogue_{index}", "Xform")
        prim.GetReferences().AddReference(uri)
        for _ in range(10):
            app.update()
        low, high = bounds_of(prim)
        dimensions = high - low
        if np.any(dimensions > 3.0) or np.any(dimensions < 0.05):
            raise RuntimeError(f"Unexpected metre scale for {uri}: {dimensions}")
        catalogue.append(AssetSpec(uri, *map(float, dimensions)))
        offsets[uri] = [
            float((low[0] + high[0]) / 2),
            float((low[1] + high[1]) / 2),
            float(low[2]),
        ]
        prim.SetActive(False)
    return catalogue, offsets


def expand_instances(prim: Usd.Prim) -> None:
    """Make only the execution-layer instance copies editable."""
    for _ in range(8):
        instances = [p for p in Usd.PrimRange(prim) if p.IsInstance()]
        if not instances:
            return
        for item in instances:
            item.SetInstanceable(False)
    raise RuntimeError("Unexpected nested instance depth")


def add_props(stage: Usd.Stage, app, props, offsets: dict) -> list[dict]:
    """Place actual factory assets as static collision objects, without rescaling."""
    records = []
    for index, placed in enumerate(props):
        rect = placed.rectangle
        prim = stage.DefinePrim(f"/World/RandomProps/Prop_{index}", "Xform")
        prim.GetReferences().AddReference(placed.asset.uri)
        center_x, center_y, floor = offsets[placed.asset.uri]
        c, s = math.cos(rect.yaw_rad), math.sin(rect.yaw_rad)
        transform = UsdGeom.Xformable(prim)
        transform.ClearXformOpOrder()
        transform.AddTranslateOp().Set(
            Gf.Vec3d(
                rect.x_m - c * center_x + s * center_y,
                rect.y_m - s * center_x - c * center_y,
                -floor,
            )
        )
        transform.AddOrientOp().Set(
            Gf.Quatf(math.cos(rect.yaw_rad / 2), 0, 0, math.sin(rect.yaw_rad / 2))
        )
        for _ in range(6):
            app.update()
        expand_instances(prim)
        count = 0
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.RigidBodyAPI):
                child.RemoveAPI(UsdPhysics.RigidBodyAPI)
            if child.HasAPI(UsdPhysics.ArticulationRootAPI):
                child.RemoveAPI(UsdPhysics.ArticulationRootAPI)
            if child.IsA(UsdGeom.Mesh):
                UsdPhysics.CollisionAPI.Apply(child).CreateCollisionEnabledAttr(True)
                UsdPhysics.MeshCollisionAPI.Apply(child).CreateApproximationAttr("none")
                count += 1
        if not count:
            raise RuntimeError(
                f"Factory prop has no collision mesh: {placed.asset.uri}"
            )
        low, high = bounds_of(prim)
        records.append(
            {
                "path": str(prim.GetPath()),
                "asset_uri": placed.asset.uri,
                "collision_meshes": count,
                "world_bounds_min_m": low.tolist(),
                "world_bounds_max_m": high.tolist(),
            }
        )
    return records


def add_destination(stage: Usd.Stage, x_m: float, y_m: float) -> str:
    """Visible green floor ring, with no collision or rigid-body API."""
    path = "/World/Destination"
    mesh = UsdGeom.Mesh.Define(stage, path)
    points, faces, indices = [], [], []
    segments = 96
    for i in range(segments):
        angle = 2 * math.pi * i / segments
        for radius in [0.59, 0.67]:
            points.append(
                (x_m + radius * math.cos(angle), y_m + radius * math.sin(angle), 0.006)
            )
        faces.append(4)
        j = (i + 1) % segments
        indices.extend([2 * i, 2 * i + 1, 2 * j + 1, 2 * j])
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(faces)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateDoubleSidedAttr(True)
    material = UsdShade.Material.Define(stage, path + "Material")
    shader = UsdShade.Shader.Define(stage, path + "Material/Shader")
    from pxr import Sdf

    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
        (0.01, 0.9, 0.04)
    )
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(
        (0.0, 0.35, 0.01)
    )
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    return path


def add_path_display(stage: Usd.Stage, path, name: str, colour: tuple) -> None:
    """A raised visual line documents the planned axle path, not a collision object."""
    line = UsdGeom.BasisCurves.Define(stage, "/World/PlannedPaths/" + name)
    line.CreateTypeAttr("linear")
    line.CreateCurveVertexCountsAttr([len(path.poses)])
    line.CreatePointsAttr([(float(p[0]), float(p[1]), 0.012) for p in path.poses])
    line.CreateWidthsAttr([0.018])
    line.CreateDisplayColorAttr([Gf.Vec3f(*colour)])


def create_pallet(
    world: World,
    stage: Usd.Stage,
    app,
    urdf_path: Path,
    output: Path,
    site,
    mass_kg: float,
) -> SingleRigidPrim:
    """Import original pallet boxes with explicitly synthetic uniform-volume inertia."""
    # Assign synthetic mass using the exact 22 source boxes and parallel-axis theorem.
    tree = ET.parse(urdf_path)
    link = tree.getroot().find("link")
    assert link.find("inertial") is None, (
        "Revisit mass handling if the source gains inertial data"
    )
    parts = []
    for collision in link.findall("collision"):
        size = np.fromstring(collision.find("geometry/box").get("size"), sep=" ")
        center = np.fromstring(collision.find("origin").get("xyz"), sep=" ")
        parts.append((size, center, float(np.prod(size))))
    volume = sum(p[2] for p in parts)
    com = sum(p[1] * p[2] for p in parts) / volume
    inertia = np.zeros((3, 3))
    for size, center, part_volume in parts:
        mass = mass_kg * part_volume / volume
        sx, sy, sz = size
        inertia += (
            np.diag([sy * sy + sz * sz, sx * sx + sz * sz, sx * sx + sy * sy])
            * mass
            / 12
        )
        delta = center - com
        inertia += mass * (np.dot(delta, delta) * np.eye(3) - np.outer(delta, delta))
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz=" ".join(map(str, com)), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=str(mass_kg))
    ET.SubElement(
        inertial,
        "inertia",
        **{
            name: str(inertia[i, j])
            for name, i, j in [
                ("ixx", 0, 0),
                ("iyy", 1, 1),
                ("izz", 2, 2),
                ("ixy", 0, 1),
                ("ixz", 0, 2),
                ("iyz", 1, 2),
            ]
        },
    )
    runtime_urdf = output / "pallet_with_synthetic_inertia.urdf"
    tree.write(runtime_urdf, encoding="utf-8", xml_declaration=True)
    ok, config = omni.kit.commands.execute("URDFCreateImportConfig")
    assert ok
    config.fix_base = False
    config.make_default_prim = True
    config.import_inertia_tensor = True
    config.create_physics_scene = False
    config.collision_from_visuals = False
    ok, imported = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(runtime_urdf),
        import_config=config,
        dest_path=str(output / "pallet.usd"),
    )
    assert ok, "Pallet import failed"
    stage.DefinePrim("/World/Pallet", "Xform").GetReferences().AddReference(
        str(output / "pallet.usd")
    )
    for _ in range(10):
        app.update()
    rigid = [
        p
        for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Pallet"))
        if p.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    assert len(rigid) == 1, [str(p.GetPath()) for p in rigid]
    for p in Usd.PrimRange(stage.GetPrimAtPath("/World/Pallet")):
        assert not p.IsA(UsdPhysics.Joint), (
            "Pallet must be free, without a fixed attachment"
        )
        if p.HasAPI(UsdPhysics.ArticulationRootAPI):
            p.RemoveAPI(UsdPhysics.ArticulationRootAPI)
    pallet = world.scene.add(
        SingleRigidPrim(
            prim_path=str(rigid[0].GetPath()),
            name="pallet",
            position=np.array([site.x_m, site.y_m, 0.015]),
            orientation=np.array(
                [math.cos(site.yaw_rad / 2), 0, 0, math.sin(site.yaw_rad / 2)]
            ),
        )
    )
    return pallet


def configure_drives(stage: Usd.Stage, settings: dict) -> dict:
    """Set documented synthetic dynamics on the execution scene only."""
    physics_material = UsdShade.Material.Define(stage, "/World/ContactMaterial")
    material_api = UsdPhysics.MaterialAPI.Apply(physics_material.GetPrim())
    material_api.CreateStaticFrictionAttr(settings["static_friction"])
    material_api.CreateDynamicFrictionAttr(settings["dynamic_friction"])
    material_api.CreateRestitutionAttr(0.0)
    # Importer stores collision groups as instances; edit the stage copy only.
    for root_path in ["/World/Forklift", "/World/Pallet"]:
        instances = [
            p
            for p in Usd.PrimRange(stage.GetPrimAtPath(root_path))
            if p.IsInstance() and "collisions" in str(p.GetPath())
        ]
        for prim in instances:
            prim.SetInstanceable(False)
        print("COLLISION_INSTANCES_EXPANDED", root_path, len(instances), flush=True)
    collision_counts = {}
    for root_path in ["/World/Forklift", "/World/Pallet"]:
        collision_counts[root_path] = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
            if prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_counts[root_path] += 1
                contact = PhysxSchema.PhysxCollisionAPI.Apply(prim)
                contact.CreateContactOffsetAttr(0.002)
                contact.CreateRestOffsetAttr(0.0)
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                    physics_material, UsdShade.Tokens.strongerThanDescendants, "physics"
                )
            if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                physics = PhysxSchema.PhysxArticulationAPI.Apply(prim)
                physics.CreateSolverPositionIterationCountAttr(32)
                physics.CreateSolverVelocityIterationCountAttr(8)
            if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(
                UsdPhysics.PrismaticJoint
            ):
                is_lift = prim.IsA(UsdPhysics.PrismaticJoint)
                wheel = "spin" in prim.GetName()
                drive = UsdPhysics.DriveAPI.Apply(
                    prim, "linear" if is_lift else "angular"
                )
                drive.CreateTypeAttr("force")
                drive.CreateStiffnessAttr(
                    0.0
                    if wheel
                    else (4000.0 if is_lift else settings["steering_stiffness"])
                )
                drive.CreateDampingAttr(
                    400.0
                    if is_lift
                    else (20.0 if wheel else settings["steering_damping"])
                )
                drive.CreateMaxForceAttr(
                    settings["lift_force_n"]
                    if is_lift
                    else (
                        settings["wheel_torque_nm"]
                        if wheel
                        else settings["steering_torque_nm"]
                    )
                )
                drive.CreateTargetPositionAttr(0.0)
                drive.CreateTargetVelocityAttr(0.0)
    assert collision_counts["/World/Pallet"] == 22, collision_counts

    return collision_counts
