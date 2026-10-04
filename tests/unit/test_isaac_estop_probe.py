"""Emergency-stop probe (priority-5 plan P0a): spec parsing, arc offset, stop record."""

import importlib.util
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("estop_probe", ROOT / "sim/isaac/estop_probe.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["estop_probe"] = MODULE  # dataclasses resolves the module by name
SPEC.loader.exec_module(MODULE)


def test_spec_parses_phase_and_delay_pairs():
    assert MODULE.parse_spec("approach@2, transport@5.5,") == [("approach", 2.0), ("transport", 5.5)]
    with pytest.raises(ValueError):
        MODULE.parse_spec("approach")
    with pytest.raises(ValueError):
        MODULE.parse_spec("approach@-1")


def test_arc_offset_is_zero_on_the_arc_and_lateral_off_it():
    start = (0.0, 0.0, 0.0)
    assert MODULE.arc_offset_m(start, 0.0, (3.0, 0.0)) == 0.0
    assert math.isclose(MODULE.arc_offset_m(start, 0.0, (3.0, 0.2)), 0.2)
    # Radius 2 m left turn: centre (0, 2); a quarter turn ends at (2, 2).
    assert MODULE.arc_offset_m(start, 0.5, (2.0, 2.0)) < 1e-12
    assert math.isclose(MODULE.arc_offset_m(start, 0.5, (2.1, 2.0)), 0.1)


def test_a_probe_triggers_once_records_the_stop_and_releases():
    probe = MODULE.EstopProbe(parse := MODULE.parse_spec("transport@1"))
    assert parse == probe.pending
    dt, x, v = 1 / 120, 0.0, 0.3
    held = []
    for k in range(1200):
        t = k * dt
        phase_elapsed = t
        hold = probe.update(
            t=t, phase="transport", phase_elapsed_s=phase_elapsed, rear=(x, 0.0, 0.0),
            speed_mps=v, loaded=True, curvature_inv_m=0.0,
        )
        held.append(hold)
        if hold:
            v = max(0.0, v - 1.0 * dt)  # 1 m/s^2 after the command
        x += v * dt
    record = probe.records[0]
    assert not probe.pending and not probe.active and len(probe.records) == 1
    assert record["loaded"] and record["direction"] == "forward"
    assert math.isclose(record["trigger_time_s"], 1.0, abs_tol=dt)
    assert math.isclose(record["stop_distance_m"], 0.3**2 / 2, rel_tol=0.1)
    assert math.isclose(record["stop_time_s"], 0.29, abs_tol=0.03)
    assert record["decel_start_s"] <= 2 * dt
    assert held.index(True) == 120 and not held[-1]


def test_a_slow_truck_does_not_trigger():
    probe = MODULE.EstopProbe(MODULE.parse_spec("insert@0"))
    assert not probe.update(
        t=0.0, phase="insert", phase_elapsed_s=0.0, rear=(0, 0, 0), speed_mps=0.02,
        loaded=False, curvature_inv_m=0.0,
    )
    assert probe.pending
