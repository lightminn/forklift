"""Restart confirmation after an occupied stop (docs/plans/2026-10-25-restart-confirmation.md R1): the permission's
own zero for an occupied cell holds the output at zero until every clearing sensor has given N new scans in a row with
a positive raw limit; other zeros keep their reason and only clear the count; a new leg or an observation gap starts
over."""

import pytest

from forklift_core.control.restart_confirm import RestartConfirm


def ticks(gate, key, items):
    """items: (raw allowed, raw why, {sensor: newest scan stamp}) per tick -> outputs."""
    return [gate.step(key, a, w, scans) for a, w, scans in items]


def test_the_halo_pulse_is_held_at_zero():
    gate = RestartConfirm(3)
    out = ticks(gate, "k", [(0.0, "occupied", {"lidar": 1.0}),       # the stop
                            (0.15, "occupied", {"lidar": 1.1}),       # one scan clears the halo
                            (0.15, "occupied", {"lidar": 1.1}),
                            (0.0, "occupied", {"lidar": 1.2})])        # painted again
    assert [o.allowed_mps for o in out] == [0.0, 0.0, 0.0, 0.0]
    assert [o.gate_only for o in out] == [False, True, True, False]
    assert out[1].why == "occupied"  # the stop watches keep reading an occupied stop


def test_n_new_scans_in_a_row_release_and_the_stop_scan_does_not_count():
    gate = RestartConfirm(3)
    out = ticks(gate, "k", [(0.0, "occupied", {"lidar": 1.0}),
                            (0.2, "ok", {"lidar": 1.0}),   # the scan of the stop itself: not a confirmation
                            (0.2, "ok", {"lidar": 1.1}),
                            (0.2, "ok", {"lidar": 1.1}),   # the same scan twice counts once
                            (0.2, "ok", {"lidar": 1.2}),
                            (0.2, "ok", {"lidar": 1.3}),   # the third new scan
                            (0.2, "ok", {"lidar": 1.3})])
    assert [o.allowed_mps for o in out] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.2]
    assert gate.releases == 1


def test_a_zero_between_starts_the_count_over():
    gate = RestartConfirm(2)
    out = ticks(gate, "k", [(0.0, "occupied", {"lidar": 1.0}), (0.2, "ok", {"lidar": 1.1}),
                            (0.0, "unknown", {"lidar": 1.2}),  # any zero clears the count, keeps its own reason
                            (0.2, "ok", {"lidar": 1.3}), (0.2, "ok", {"lidar": 1.4})])
    assert [o.allowed_mps for o in out] == [0.0, 0.0, 0.0, 0.0, 0.2]
    assert out[2].why == "unknown" and not out[2].gate_only


def test_every_clearing_sensor_needs_its_own_new_scans():
    # Codex: one sensor refreshing alone must not release on another's single clearing
    gate = RestartConfirm(2)
    out = ticks(gate, "k", [(0.0, "occupied", {"low": 1.0, "high": 1.0}),
                            (0.2, "ok", {"low": 1.1, "high": 1.0}),
                            (0.2, "ok", {"low": 1.1, "high": 1.025}),
                            (0.2, "ok", {"low": 1.1, "high": 1.05}),   # high has two, low only one
                            (0.2, "ok", {"low": 1.2, "high": 1.05})])
    assert [o.allowed_mps for o in out] == [0.0, 0.0, 0.0, 0.0, 0.2]


def test_a_zero_for_another_reason_does_not_arm_it():
    gate = RestartConfirm(3)
    out = ticks(gate, "k", [(0.0, "sensor_silent:lidar", {"lidar": 1.0}), (0.2, "ok", {"lidar": 1.0})])
    assert [o.allowed_mps for o in out] == [0.0, 0.2] and out[0].why == "sensor_silent:lidar"


def test_a_raw_zero_keeps_its_reason_while_armed():
    gate = RestartConfirm(3)
    out = ticks(gate, "k", [(0.0, "occupied", {"lidar": 1.0}), (0.0, "sensor_silent:lidar", {"lidar": 1.0})])
    assert out[1].why == "sensor_silent:lidar" and not out[1].gate_only


def test_a_new_leg_starts_without_it():
    gate = RestartConfirm(3)
    ticks(gate, "leg-1", [(0.0, "occupied", {"lidar": 1.0})])
    out = gate.step("leg-2", 0.2, "ok", {"lidar": 1.1})
    assert out.allowed_mps == 0.2 and not gate.armed


def test_an_observation_gap_starts_the_count_over():
    # Codex: two confirmations, a plan wait, then one new scan must not release with N = 3
    gate = RestartConfirm(3)
    ticks(gate, "k", [(0.0, "occupied", {"lidar": 1.0}), (0.2, "ok", {"lidar": 1.1}), (0.2, "ok", {"lidar": 1.2})])
    gate.observation_gap({"lidar": 1.5})
    out = ticks(gate, "k", [(0.2, "ok", {"lidar": 1.5}), (0.2, "ok", {"lidar": 1.6}), (0.2, "ok", {"lidar": 1.7}),
                            (0.2, "ok", {"lidar": 1.8})])
    assert [o.allowed_mps for o in out] == [0.0, 0.0, 0.0, 0.2]


def test_n_must_be_positive():
    with pytest.raises(ValueError):
        RestartConfirm(0)
