"""Contact materials by part (docs/plans/2026-11-01-contact-materials.md, step A).

``scene.configure_drives`` binds one material, ``/World/ContactMaterial``, to every truck and liftable-pallet
collider. Step A splits it by part -- tyre, chassis, fork, pallet -- with the same authored values and combine rule
copied from it, so every contact pair keeps its friction; later steps change one part at a time. Only colliders
bound to ``/World/ContactMaterial`` move: a diagnostic material ``configure_drives`` bound instead (the tyre-only
compliant one, the pallet-only friction one) stays where it is.

Kept out of ``scene.py`` on purpose: a stationary scan capture compares ``scene.py``'s hash with the survey's
(``scan_capture.WORLD_SOURCES``), so a change there would refuse every existing survey map.

Settings: ``contact_material_split`` absent = one material as before; present (a positive number, as every settings
value must be) = split. The pure helpers below need no Isaac; the stage functions import ``pxr`` when called.
"""

from __future__ import annotations

SPLIT_KEY = "contact_material_split"
SOURCE_PATH = "/World/ContactMaterial"
ROOT = "/World/ContactMaterials"
ROLES = ("tyre", "chassis", "fork", "pallet")


def split_enabled(settings: dict) -> bool:
    return settings.get(SPLIT_KEY) is not None


def truck_role(collider_path: str) -> str:
    """The part a truck collider belongs to, from its prim path: the four ``*_tire_collision`` are the tyres, the
    blades and their heels (``left_fork_collision``, ``left_fork_heel_collision``, right alike) the forks, everything
    else (body, carriage, mast, guard) the chassis. The carriage's own boxes carry ``carriage`` in their names, not
    ``_fork_``, so they stay chassis."""
    if "_tire_collision" in collider_path:  # as scene.configure_drives finds the tyres
        return "tyre"
    if "/left_fork" in collider_path or "/right_fork" in collider_path:
        return "fork"
    return "chassis"


def _copy_material(stage, source, path: str):
    """A new physics material at ``path`` with the source's applied schemas and every authored attribute value."""
    from pxr import UsdShade

    material = UsdShade.Material.Define(stage, path)
    prim, src = material.GetPrim(), source.GetPrim()
    for schema in src.GetAppliedSchemas():
        prim.AddAppliedSchema(schema)
    for attr in src.GetAuthoredAttributes():
        value = attr.Get()
        if value is None:
            continue
        prim.CreateAttribute(attr.GetName(), attr.GetTypeName(), custom=attr.IsCustom()).Set(value)
    return material


def _bound_path(prim) -> str | None:
    from pxr import UsdShade

    material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
    return str(material.GetPath()) if material else None


def split_contact_materials(stage, pallet_roots=("/World/Pallet",)) -> dict:
    """Step A: rebind the truck and pallet colliders bound to ``/World/ContactMaterial`` to one copy per part. Returns
    the record: each part's material, its values and how many colliders it took, and the colliders left on another
    material (by that material)."""
    from pxr import Usd, UsdPhysics, UsdShade

    source = UsdShade.Material(stage.GetPrimAtPath(SOURCE_PATH))
    if not source:
        raise ValueError(f"{SPLIT_KEY}: no {SOURCE_PATH} to split (configure_drives has not run)")
    materials = {role: _copy_material(stage, source, f"{ROOT}/{role.capitalize()}") for role in ROLES}
    counts = {role: 0 for role in ROLES}
    kept: dict[str, int] = {}
    for root_path, is_pallet in [("/World/Forklift", False), *((r, True) for r in pallet_roots)]:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
            if not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            bound = _bound_path(prim)
            if bound != SOURCE_PATH:
                kept[str(bound)] = kept.get(str(bound), 0) + 1
                continue
            role = "pallet" if is_pallet else truck_role(str(prim.GetPath()))
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                materials[role], UsdShade.Tokens.strongerThanDescendants, "physics")
            counts[role] += 1
    # every truck part takes its material; the tyres may stay on the tyre-only compliant material instead
    missing = [role for role in ("chassis", "fork") if counts[role] == 0]
    if counts["tyre"] == 0 and not any("Tyre" in path for path in kept):
        missing.append("tyre")
    if missing:
        raise ValueError(f"{SPLIT_KEY}: no truck collider took the {', '.join(missing)} material")
    return {"source": SOURCE_PATH, "colliders": counts, "kept_on_other_material": kept,
            "materials": {role: material_record(materials[role].GetPrim()) for role in ROLES}}


def material_record(prim) -> dict:
    """A physics material's authored values (USD side; PhysX's own default material is not visible here)."""
    return {"path": str(prim.GetPath()),
            **{attr.GetName(): attr.Get() for attr in prim.GetAuthoredAttributes()
               if attr.GetName().startswith(("physics:", "physxMaterial:"))}}


def unbound_colliders(stage, roots=("/World",)) -> dict:
    """Colliders with no physics material bound, counted by their first two path components under each root (PhysX
    gives them its default material). For the record and the T0 inventory."""
    from pxr import Usd, UsdPhysics

    out: dict[str, int] = {}
    for root_path in roots:
        for prim in Usd.PrimRange(stage.GetPrimAtPath(root_path)):
            if prim.HasAPI(UsdPhysics.CollisionAPI) and _bound_path(prim) is None:
                key = "/".join(str(prim.GetPath()).split("/")[:3])
                out[key] = out.get(key, 0) + 1
    return out
