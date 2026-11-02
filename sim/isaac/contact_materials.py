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
# Step B (fork-material sensitivity): the fork copy's own friction, combined by "min" against what it touches, so the
# pallet-fork pair takes the fork's values (docs/plans/2026-11-01-contact-materials.md section 5 B). All three keys
# together, and only with the split; absent = the fork copy keeps the source's values.
FORK_KEYS = ("fork_static_friction", "fork_dynamic_friction", "fork_friction_combine_min")
SOURCE_PATH = "/World/ContactMaterial"
ROOT = "/World/ContactMaterials"
ROLES = ("tyre", "chassis", "fork", "pallet")


def split_enabled(settings: dict) -> bool:
    return settings.get(SPLIT_KEY) is not None


def fork_override(settings: dict) -> dict | None:
    """The fork material's step-B values from the settings, or None; refuses a partial or inconsistent set."""
    given = [k for k in FORK_KEYS if k in settings]
    if not given:
        return None
    if len(given) != len(FORK_KEYS) or not split_enabled(settings):
        raise ValueError(f"{', '.join(FORK_KEYS)} go together and need {SPLIT_KEY} (given: {', '.join(given)})")
    mu_s, mu_d = float(settings["fork_static_friction"]), float(settings["fork_dynamic_friction"])
    if not 0 < mu_d <= mu_s:
        raise ValueError(f"fork friction: dynamic {mu_d} must be positive and at most the static {mu_s}")
    return {"static_friction": mu_s, "dynamic_friction": mu_d, "combine": "min"}


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


def _set_combine(prim, mode: str) -> None:
    """The PhysX friction combine rule on a material prim -- through PhysxSchema in Isaac; the same applied schema and
    attribute by name where only plain USD is installed (the unit tests)."""
    try:
        from pxr import PhysxSchema
    except ImportError:
        from pxr import Sdf

        prim.AddAppliedSchema("PhysxMaterialAPI")
        prim.CreateAttribute("physxMaterial:frictionCombineMode", Sdf.ValueTypeNames.Token).Set(mode)
        return
    PhysxSchema.PhysxMaterialAPI.Apply(prim).CreateFrictionCombineModeAttr(mode)


def _bound_material(prim):
    """The physics material bound to ``prim``, or None. ``ComputeBoundMaterial("physics")`` falls back to an
    all-purpose (render) binding, so a bound material counts only when it carries the physics material API -- the
    warehouse floor tiles are bound to render materials with no friction (T0, 2026-11-02)."""
    from pxr import UsdPhysics, UsdShade

    material, _ = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial("physics")
    return material if material and material.GetPrim().HasAPI(UsdPhysics.MaterialAPI) else None


def _bound_path(prim) -> str | None:
    material = _bound_material(prim)
    return str(material.GetPath()) if material else None


def split_contact_materials(stage, pallet_roots=("/World/Pallet",), fork: dict | None = None) -> dict:
    """Step A: rebind the truck and pallet colliders bound to ``/World/ContactMaterial`` to one copy per part; ``fork``
    (``fork_override``) then gives the fork copy its own friction and the "min" rule (step B). Returns the record:
    each part's material, its values and how many colliders it took, and the colliders left on another material (by
    that material)."""
    from pxr import Usd, UsdPhysics, UsdShade

    source = UsdShade.Material(stage.GetPrimAtPath(SOURCE_PATH))
    if not source:
        raise ValueError(f"{SPLIT_KEY}: no {SOURCE_PATH} to split (configure_drives has not run)")
    materials = {role: _copy_material(stage, source, f"{ROOT}/{role.capitalize()}") for role in ROLES}
    if fork is not None:
        api = UsdPhysics.MaterialAPI(materials["fork"].GetPrim())
        api.GetStaticFrictionAttr().Set(fork["static_friction"])
        api.GetDynamicFrictionAttr().Set(fork["dynamic_friction"])
        _set_combine(materials["fork"].GetPrim(), fork["combine"])
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


def colliders(stage, root: str = "/World"):
    """Every collider under ``root``, instance proxies included (a plain traversal skips what sits inside an
    instanceable reference -- Codex A review P2-1)."""
    from pxr import Usd, UsdPhysics

    for prim in Usd.PrimRange(stage.GetPrimAtPath(root), Usd.TraverseInstanceProxies()):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            yield prim


def material_inventory(stage, root: str = "/World") -> dict:
    """The colliders under ``root`` by the physics material bound to them (``None``: none bound, or only a render
    material -- PhysX's default material applies), each counted by its first two path components, and every bound material's values -- the T0
    inventory (plan section 5)."""
    by_material: dict[str, dict[str, int]] = {}
    values = {}
    for prim in colliders(stage, root):
        material = _bound_material(prim)
        key = str(material.GetPath()) if material else "None"
        group = "/".join(str(prim.GetPath()).split("/")[:3])
        by_material.setdefault(key, {})
        by_material[key][group] = by_material[key].get(group, 0) + 1
        if material and key not in values:
            values[key] = material_record(material.GetPrim())
    return {"colliders": by_material, "materials": values}


def unbound_colliders(stage, root: str = "/World") -> dict:
    """The colliders with no physics material bound, counted by their first two path components."""
    return material_inventory(stage, root)["colliders"].get("None", {})
