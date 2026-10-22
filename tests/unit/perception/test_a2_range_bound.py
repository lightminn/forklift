"""P3a (docs/plans/2026-10-23-planner-permission-clearance.md, user decision 2026-10-23): the LiDAR range error from
Slamtec's A2 spec page (1 % of the range up to 3 m, 2 % to 5 m, 2.5 % beyond; read as +-2 sigma -- a design
assumption): the simulated noise sigma = acc/2 cut at max(0.02, 1.5 acc) of the true range, the grid's bound read
from the measured range at the farthest true range it could come from (1.04 d)."""

import math

import numpy as np
import pytest

from forklift_core.perception.obstacle_grid import (
    AgeErrorTable,
    GridConfig,
    a2_spec_accuracy_m,
    a2_spec_cut_m,
)

TABLE = AgeErrorTable((0.1, 0.2, 0.3), (0.0241, 0.0315, 0.0421), (0.0144, 0.0219, 0.0260))


def grid(model="const"):
    return GridConfig(-5.0, 5.0, -5.0, 5.0, TABLE, range_bound_model=model)


@pytest.mark.parametrize("d, acc", [(1.0, 0.01), (3.0, 0.03), (3.01, 0.0602), (5.0, 0.10), (6.0, 0.15)])
def test_the_spec_rows(d, acc):
    assert a2_spec_accuracy_m(np.array(d)) == pytest.approx(acc)


def test_the_cut_has_a_floor_and_is_one_and_a_half_accuracies():
    assert a2_spec_cut_m(np.array(0.5)) == pytest.approx(0.02)  # 1.5 % of 0.5 m is below the floor
    assert a2_spec_cut_m(np.array(2.0)) == pytest.approx(0.03)
    assert a2_spec_cut_m(np.array(4.0)) == pytest.approx(0.12)


def test_the_constant_model_is_the_old_bound_everywhere():
    g = grid()
    assert np.all(g.sensor_bound_at(np.array([0.2, 2.9, 3.1, 5.0])) == 0.06)


def test_the_spec_bound_reads_the_farthest_true_range_a_measurement_can_come_from():
    g = grid("a2_spec")
    # Codex v4 P1: true 3.09 m read 0.0927 m short as 2.9973 m -- the bound must still reach the truth
    d_true, noise = 3.09, -float(a2_spec_cut_m(np.array(3.09)))
    measured = d_true + noise
    assert g.sensor_bound_at(np.array(measured)) >= -noise
    assert g.sensor_bound_at(np.array(2.9973)) == pytest.approx(1.5 * 0.02 * 1.04 * 2.9973)


@pytest.mark.parametrize("d_true", np.r_[np.linspace(0.2, 5.0, 97), np.linspace(2.9, 3.2, 61), np.linspace(4.8, 5.0, 21)])
def test_every_admissible_measurement_is_covered_by_its_bound(d_true):
    g = grid("a2_spec")
    cut = float(a2_spec_cut_m(np.array(d_true)))
    for noise in (-cut, 0.0, cut):
        assert g.sensor_bound_at(np.array(d_true + noise)) >= abs(noise) - 1e-12


def test_an_unknown_model_is_refused():
    with pytest.raises(ValueError):
        grid("lenient")
