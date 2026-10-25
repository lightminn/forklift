"""RTX presets on the render products (Kit 107 reads omni:rtx:* there): written, read back, refused when the renderer
does not keep them, and checked again at the end of a run."""

import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))

import render_preset  # noqa: E402
from render_preset import PRESETS, RenderPresetError  # noqa: E402


class Attr:
    def __init__(self, prim, key):
        self.prim, self.key = prim, key

    def IsValid(self):
        return self.key in self.prim.values

    def __bool__(self):
        return self.IsValid()

    def Get(self):
        return self.prim.values[self.key]

    def Set(self, value):
        self.prim.writes.append((self.key, value))
        self.prim.values[self.key] = self.prim.revert.get((self.key, value), value)
        return True


class Prim:
    def __init__(self, path, kind="RenderProduct", values=None, revert=None):
        self.path, self.kind = path, kind
        self.values = dict(values if values is not None else {
            "omni:rtx:post:aa:op": "dlss",
            "omni:rtx:directLighting:sampledLighting:autoEnable": True,
            "omni:rtx:directLighting:sampledLighting:enabled": True,
        })
        self.revert = revert or {}
        self.writes = []

    def GetPath(self):
        return self.path

    def GetTypeName(self):
        return self.kind

    def IsValid(self):
        return True

    def __bool__(self):
        return True

    def GetAttribute(self, key):
        return Attr(self, key)


class Stage:
    def __init__(self, prims):
        self.prims = {p.path: p for p in prims}

    def Traverse(self):
        return iter(self.prims.values())

    def GetPrimAtPath(self, path):
        return self.prims.get(path)


def test_fast_writes_aa_first_and_reads_back_the_preset():
    prim = Prim("/Render/RP_a")
    got = render_preset.apply(Stage([prim]), "/Render/RP_a", "fast")
    assert got == PRESETS["fast"]
    assert prim.writes[0] == ("omni:rtx:post:aa:op", "fxaa")
    assert [k for k, _ in prim.writes] == list(PRESETS["fast"])


def test_a_value_the_renderer_puts_back_is_refused():
    prim = Prim("/Render/RP_a", revert={("omni:rtx:post:aa:op", "fxaa"): "dlss"})
    with pytest.raises(RenderPresetError, match="did not keep"):
        render_preset.apply(Stage([prim]), "/Render/RP_a", "fast")


def test_a_missing_attribute_or_prim_or_preset_is_refused():
    with pytest.raises(RenderPresetError, match="no attribute"):
        render_preset.apply(Stage([Prim("/Render/RP_a", values={})]), "/Render/RP_a", "fast")
    with pytest.raises(RenderPresetError, match="unknown"):
        render_preset.apply_all(Stage([]), "fastest")


def test_apply_all_covers_every_render_product_and_only_them():
    a, b, other = Prim("/Render/RP_a"), Prim("/Render/RP_b"), Prim("/World/cam", kind="Camera")
    record = render_preset.apply_all(Stage([a, other, b]), "fast")
    assert record["render_products"] == ["/Render/RP_a", "/Render/RP_b"]
    assert set(record["readback"]) == {"/Render/RP_a", "/Render/RP_b"}
    assert other.writes == []


def test_default_writes_nothing():
    a = Prim("/Render/RP_a")
    record = render_preset.apply_all(Stage([a]), "default")
    assert record == {"name": "default", "values": {}, "render_products": [], "skipped": [], "readback": {}}
    assert a.writes == []


def test_drifted_reports_values_changed_after_apply():
    a, b = Prim("/Render/RP_a"), Prim("/Render/RP_b")
    stage = Stage([a, b])
    record = render_preset.apply_all(stage, "fast")
    assert render_preset.drifted(stage, record) == {}
    b.values["omni:rtx:post:aa:op"] = "dlss"
    assert render_preset.drifted(stage, record) == {"/Render/RP_b": {"omni:rtx:post:aa:op": "dlss"}}


def test_drifted_names_a_render_product_removed_before_the_end():
    a = Prim("/Render/RP_a")
    stage = Stage([a])
    record = render_preset.apply_all(stage, "fast")
    del stage.prims["/Render/RP_a"]
    assert render_preset.drifted(stage, record) == {"/Render/RP_a": "gone"}


def test_a_render_product_without_the_settings_schema_is_listed_not_written():
    plain, partial = Prim("/Render/viewport", values={}), Prim("/Render/RP_p", values={"omni:rtx:post:aa:op": "dlss"})
    record = render_preset.apply_all(Stage([plain, Prim("/Render/RP_a")]), "fast")
    assert record["skipped"] == ["/Render/viewport"] and record["render_products"] == ["/Render/RP_a"]
    assert plain.writes == []
    with pytest.raises(RenderPresetError, match="no attribute"):
        render_preset.apply_all(Stage([partial]), "fast")
