"""RTX settings written where Isaac 5.1 reads them: the ``omni:rtx:*`` attributes of each camera's render product.

Kit 107 keeps the RTX render settings on the RenderProduct prims (``omni.usd.schema.render_settings.rtx``); the
``/rtx/...`` carb paths are marked deprecated there, and the 2026-10-24/25 probes found carb writes, the SimulationApp
``anti_aliasing`` argument and ``--/rtx/...`` launch arguments all read back unchanged. Written on the render product,
FXAA and sampled direct lighting off did hold, and halved the render step of a 640x480 camera (0.059 -> 0.031 s, ws1
job 11155); ``aa:op`` none or taa were put back to dlss by the renderer, so a preset is read back after it is written
and again at the end of a run.

``default`` writes nothing (the scene's own values). A preset changes the RGB the learned proposer sees, so it is a
measured choice per run, recorded in ``result.json``.
"""

from __future__ import annotations

PRESETS: dict[str, dict[str, object]] = {
    "default": {},
    # aa:op first: written after another key, none/taa reverted the whole prim to the defaults in job 11155
    "fast": {
        "omni:rtx:post:aa:op": "fxaa",
        "omni:rtx:directLighting:sampledLighting:autoEnable": False,
        "omni:rtx:directLighting:sampledLighting:enabled": False,
    },
}


class RenderPresetError(RuntimeError):
    pass


def render_products(stage) -> list[str]:
    """Paths of every RenderProduct prim on ``stage``, in traversal order."""
    return [str(prim.GetPath()) for prim in stage.Traverse() if prim.GetTypeName() == "RenderProduct"]


def readback(stage, path: str, name: str) -> dict[str, object]:
    """The preset's attributes as ``path`` now holds them."""
    prim = stage.GetPrimAtPath(path)
    return {key: prim.GetAttribute(key).Get() for key in PRESETS[name]}


def apply(stage, path: str, name: str) -> dict[str, object]:
    """Writes preset ``name`` on the render product at ``path``; the values read back, which must equal the preset."""
    if name not in PRESETS:
        raise RenderPresetError(f"unknown render preset {name!r}")
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsValid():
        raise RenderPresetError(f"no render product at {path}")
    for key, value in PRESETS[name].items():
        attr = prim.GetAttribute(key)
        if not attr or not attr.IsValid():
            raise RenderPresetError(f"{path} has no attribute {key}")
        attr.Set(value)
    got = readback(stage, path, name)
    wrong = {key: got[key] for key, value in PRESETS[name].items() if got[key] != value}
    if wrong:
        raise RenderPresetError(f"{path} did not keep {wrong} (preset {name})")
    return got


def apply_all(stage, name: str) -> dict:
    """``apply`` on every render product; the record ``result.json`` keeps."""
    if name not in PRESETS:
        raise RenderPresetError(f"unknown render preset {name!r}")
    paths, skipped = [], []
    for path in render_products(stage) if PRESETS[name] else []:
        prim = stage.GetPrimAtPath(path)
        # a render product without the RTX settings schema (none of the keys) is not one the renderer reads them
        # from -- listed, not written; one with some keys but not all is refused by apply()
        if any(prim.GetAttribute(key) for key in PRESETS[name]):
            paths.append(path)
        else:
            skipped.append(path)
    return {"name": name, "values": dict(PRESETS[name]), "render_products": paths, "skipped": skipped,
            "readback": {path: apply(stage, path, name) for path in paths}}


def drifted(stage, record: dict) -> dict:
    """The render products whose preset values changed since ``apply_all`` (empty when they all held)."""
    out = {}
    for path in record.get("render_products", []):
        prim = stage.GetPrimAtPath(path)
        if not prim or not prim.IsValid():
            out[path] = "gone"  # removed with its annotators before the end (not a changed value)
            continue
        got = readback(stage, path, record["name"])
        wrong = {key: got[key] for key, value in record["values"].items() if got[key] != value}
        if wrong:
            out[path] = wrong
    return out
