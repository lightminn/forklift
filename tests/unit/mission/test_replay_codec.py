"""The M-a1 replay's value codec (docs/plans/2026-10-16-m-a1-replay.md, step R1)."""

import json
from dataclasses import dataclass, field

import numpy as np
import pytest

from forklift_core.control.path_tracking import TrackerConfig
from forklift_core.mission.replay_codec import decode, encode
from forklift_core.planning import PlanResult, Pose2D


@dataclass
class Holder:
    name: str
    items: list
    derived: float = field(init=False, default=0.0)


def roundtrip(value, **kw):
    node, arrays = encode(value, **kw)
    node = json.loads(json.dumps(node, allow_nan=False))  # the JSON the snapshot is written as
    return decode(node, arrays)


def test_scalars_tuples_arrays_and_non_finite_floats_come_back_with_their_types():
    value = {"t": (1, 2.5, "x"), "nested": [(np.float64(1.5), np.int64(3), np.bool_(True))],
             "a": np.arange(6, dtype=np.int16).reshape(2, 3), "nan": float("nan"), "inf": -float("inf")}
    out = roundtrip(value)
    assert out["t"] == (1, 2.5, "x") and isinstance(out["t"], tuple)
    assert out["nested"] == [(1.5, 3, True)] and isinstance(out["nested"][0], tuple)
    assert out["a"].dtype == np.int16 and out["a"].shape == (2, 3) and (out["a"] == np.arange(6).reshape(2, 3)).all()
    assert np.isnan(out["nan"]) and out["inf"] == -float("inf")


def test_dataclasses_are_rebuilt_by_their_qualified_name_with_non_init_fields():
    plan = PlanResult(True, "success", np.zeros((2, 3)), np.ones(2, np.int8), np.zeros(2), 1.0, 3)
    h = Holder("h", [Pose2D(1.0, 2.0, 0.5)])
    h.derived = 7.0
    out = roundtrip({"plan": plan, "cfg": TrackerConfig(cruise_speed_mps=0.3), "h": h})
    assert isinstance(out["plan"], PlanResult) and out["plan"].status == "success"
    assert out["plan"].directions.dtype == np.int8
    assert out["cfg"] == TrackerConfig(cruise_speed_mps=0.3)
    assert isinstance(out["h"], Holder) and out["h"].items == [Pose2D(1.0, 2.0, 0.5)] and out["h"].derived == 7.0


def test_the_alias_graph_is_kept_by_object_identity():
    shared = [1, 2]
    table = {"x": 1}
    value = {"a": shared, "b": {"inner": shared, "t": table}, "c": [table, shared]}
    out = roundtrip(value)
    assert out["a"] is out["b"]["inner"] is out["c"][1]
    assert out["b"]["t"] is out["c"][0]
    out["a"].append(3)
    assert out["c"][1] == [1, 2, 3]
    loop = []
    loop.append(loop)  # a self-reference
    back = roundtrip(loop)
    assert back[0] is back


def test_non_string_keys_survive():
    out = roundtrip({1: "one", (2, 3): "pair", "s": {0.5: [1]}})
    assert out == {1: "one", (2, 3): "pair", "s": {0.5: [1]}}


def test_a_value_without_an_encoding_is_refused_unless_comparison_only():
    class Service:
        pass

    with pytest.raises(TypeError, match="no value encoding"):
        encode({"svc": Service()})
    node, _ = encode({"svc": Service()}, opaque_ok=True)
    assert node["$map"]["svc"] == {"$opaque": f"{Service.__module__}:{Service.__qualname__}"}


def test_an_unknown_class_does_not_decode_silently():
    with pytest.raises(ModuleNotFoundError):
        decode({"$dataclass": "no_such_module:X", "fields": {}}, {})


def test_arrays_and_mutable_dataclasses_keep_their_identity():
    arr = np.zeros(3)
    h = Holder("h", [])
    out = roundtrip({"a": arr, "b": [arr], "h1": h, "h2": h})
    assert out["a"] is out["b"][0] and out["h1"] is out["h2"]
    out["a"][0] = 1.0
    assert out["b"][0][0] == 1.0


def test_a_dataclass_whose_post_init_copies_a_field_keeps_the_restored_alias():
    from forklift_core.geometry import FramePoints

    fp = FramePoints("base_link", np.zeros((2, 3)))
    out = roundtrip({"fp": fp, "raw": fp.xyz_m})
    assert out["fp"].xyz_m is out["raw"] and isinstance(out["fp"], FramePoints)
    assert not out["raw"].flags.writeable  # read-only as recorded


def test_numpy_scalars_keep_their_dtype_and_unencodable_ones_are_refused():
    out = roundtrip({"f": np.float32(1.5), "i": np.int16(-3), "b": np.bool_(True), "n": np.float64("nan")})
    assert out["f"].dtype == np.float32 and out["f"] == np.float32(1.5)
    assert out["i"].dtype == np.int16 and out["b"].dtype == np.bool_ and np.isnan(out["n"])
    more = roundtrip({"c": np.complex64(1 + 2j), "l": np.longdouble(1) / 3, "t": np.timedelta64(1, "s"),
                      "d": np.datetime64("2026-10-16")})
    assert more["c"] == np.complex64(1 + 2j) and more["l"] == np.longdouble(1) / 3
    assert more["t"] == np.timedelta64(1, "s") and more["d"] == np.datetime64("2026-10-16")
    text = roundtrip({"s": np.str_("x")})["s"]
    assert type(text) is np.str_ and text == "x"
    with pytest.raises(TypeError, match="no value encoding"):
        encode({"v": np.void(b"ab")})
