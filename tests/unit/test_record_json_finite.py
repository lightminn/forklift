"""The runner's record_json writes a non-finite float as null instead of failing the write (lead 2026-11-05, after
#309: V3 ddc_on_v3a_line_2_s8 lost its result.json to a timed-out rollout's NaN), and leaves every finite record's
text byte for byte as it was."""

import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "sim/isaac"))
import run_transport  # noqa: E402

record_json = run_transport.record_json


def strict(value, indent=None):
    """The encoding before the change."""

    def native(item):
        if isinstance(item, np.ndarray):
            return item.tolist()
        if isinstance(item, np.generic):
            return item.item()
        raise TypeError(type(item).__name__)

    return json.dumps(value, default=native, indent=indent, allow_nan=False)


FINITE = {
    "success": False, "failure_reason": "docking_retry_no_plan:no_path", "phase": "deliver",
    "error": 0.0012, "big": 1e308, "tiny": -5e-324, "count": 3, "none": None, "flag": True,
    "pose": (1.0, -2.5, 0.25), "array": np.array([[0.1, 0.2], [0.3, 0.4]]), "f32": np.float32(0.1),
    "i64": np.int64(7), "nested": {"rounds": [{"round": 1, "dry_run": {"position_error_m": 0.003}}]},
}


@pytest.mark.parametrize("indent", [None, 2])
def test_a_finite_record_is_written_byte_for_byte_as_before(indent, capsys):
    assert record_json(FINITE, indent=indent) == strict(FINITE, indent=indent)
    assert capsys.readouterr().err == ""


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf, np.float64("nan"), np.float32("inf")])
def test_each_non_finite_float_is_null_and_its_path_is_said(bad, capsys):
    record = {"a": [1.0, {"b": bad}], "c": (2.0, bad), "d": np.array([0.5, float(bad)]), "e": bad}
    text = record_json(record)
    assert json.loads(text) == {"a": [1.0, {"b": None}], "c": [2.0, None], "d": [0.5, None], "e": None}
    err = capsys.readouterr().err.strip()
    assert err == "RECORD_NON_FINITE 4 .a[1].b .c[1] .d[1] .e"


def test_a_bare_non_finite_value_is_null():
    assert record_json(math.nan) == "null"


def test_the_failure_state_is_kept_around_the_null():
    # the accident's shape: a failed run whose round copy and top-level dry run carry a timed-out rollout's NaN
    state = {"success": False, "failure_reason": "docking_retry_no_plan:no_path:invalid_start", "phase": "deliver",
             "transitions": [{"from": "transport", "to": "deliver", "time_s": 412.5}],
             "delivery_docking": {"dry_run": {"position_error_m": math.nan, "yaw_error_rad": math.nan,
                                              "timed_out": True},
                                  "rounds": [{"round": 1, "accepted": False, "chosen": "retry_b",
                                              "dry_run": {"position_error_m": math.inf}}]}}
    back = json.loads(record_json(state, indent=2))
    assert (back["success"], back["failure_reason"], back["phase"], back["transitions"]) == (
        False, state["failure_reason"], "deliver", state["transitions"])
    assert back["delivery_docking"]["dry_run"] == {"position_error_m": None, "yaw_error_rad": None, "timed_out": True}
    assert back["delivery_docking"]["rounds"][0] == {"round": 1, "accepted": False, "chosen": "retry_b",
                                                     "dry_run": {"position_error_m": None}}
    assert math.isnan(state["delivery_docking"]["dry_run"]["position_error_m"])  # the state itself is not changed


def test_the_mission_core_s_round_trips_read_null_where_they_failed_before():
    # core.py reads json.loads(record_json(...)) back for the tracker's held pose, a refused near capture and the
    # tracker configs: a finite value reads back as it was, a non-finite one as None (it raised before)
    held = {"pose": np.array([1.0, 2.0, 0.1]), "age_s": 0.4}
    assert json.loads(record_json(held)) == json.loads(strict(held))
    assert json.loads(record_json({"pose": [1.0, math.nan]})) == {"pose": [1.0, None]}


def test_other_encoding_errors_still_raise():
    loop = []
    loop.append(loop)
    with pytest.raises(ValueError, match="Circular"):
        record_json(loop)
    with pytest.raises(TypeError, match="Unsupported record type"):
        record_json({"path": Path("x")})
    with pytest.raises(TypeError, match="Unsupported record type"):
        record_json({"e": math.nan, "path": Path("x")})  # NaN first: the second pass keeps the type check
