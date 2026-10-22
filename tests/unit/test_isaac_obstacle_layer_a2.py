"""P3a in the Isaac obstacle layer (docs/plans/2026-10-23-planner-permission-clearance.md): config/obstacle_layer_single_a2
draws each beam's range noise from Slamtec's A2 spec rows (sigma = acc / 2, cut max(0.02, 1.5 acc) of its true range)
and gives the grid the same model; the constant configuration draws exactly as before."""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.obstacle_grid import AgeErrorTable, a2_spec_cut_m
from forklift_core.planning.geometry import Bounds, Footprint

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("obstacle_layer", ROOT / "sim/isaac/obstacle_layer.py")
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules["obstacle_layer"] = MODULE
SPEC.loader.exec_module(MODULE)
AGE = json.loads((ROOT / "config/obstacle_odometry_age.json").read_text())
TABLE = AgeErrorTable(tuple(AGE["ages_s"]), tuple(AGE["cumulative_position_m"]), tuple(AGE["cumulative_yaw_rad"]))


def layer(name, seed=1):
    cfg = MODULE.load_layer_config(ROOT / f"config/{name}.yaml")
    return MODULE.ObstacleLayer(cfg, hall=Bounds(-5, 10, -5, 5), error_table=TABLE,
                                unloaded=Footprint(1.29, 0.17, 0.36), loaded=Footprint(1.53, 0.17, 0.40),
                                body_front_m=0.944, rear_axle_x_in_base_m=-0.34, noise_seed=seed)


def test_the_constant_layer_draws_as_before():
    lay = layer("obstacle_layer_single")
    ref = layer("obstacle_layer_single")
    n = 200
    a = lay.beam_noise(n)
    b = np.clip(ref.rng.normal(0, 0.02, n), -0.06, 0.06)
    assert np.array_equal(a, b) and lay.grid_config.range_bound_model == "const"
    dist = np.linspace(0.5, 6.0, n)
    assert np.array_equal(layer("obstacle_layer_single").ranges(dist, np.ones(n, bool)),
                          dist + np.clip(layer("obstacle_layer_single").rng.normal(0, 0.02, n), -0.06, 0.06))


def test_the_a2_layer_cuts_each_beam_at_its_range_and_hands_the_grid_the_model():
    lay = layer("obstacle_layer_single_a2", seed=3)
    assert lay.grid_config.range_bound_model == "a2_spec"
    dist = np.repeat([0.5, 2.0, 2.99, 3.01, 4.5], 4000)
    noise = lay.beam_noise(len(dist), dist)
    cut = a2_spec_cut_m(dist)
    assert np.all(np.abs(noise) <= cut + 1e-12)
    for d in (2.0, 4.5):  # sigma = acc / 2 (cut at 3 sigma barely trims it)
        sd = noise[dist == d].std()
        assert sd == pytest.approx(0.5 * (0.01 if d <= 3 else 0.02) * d, rel=0.05)
    assert np.abs(noise[dist == 0.5]).max() <= 0.02 + 1e-12  # the floor


def test_the_a2_layer_needs_the_true_ranges_for_a_shared_scan():
    lay = layer("obstacle_layer_single_a2")
    with pytest.raises(ValueError):
        lay.beam_noise(10)


def test_the_widest_bound_is_what_a_caller_needing_one_bound_reads():
    lay = layer("obstacle_layer_single_a2")
    assert lay.noise_cut_m == pytest.approx(float(a2_spec_cut_m(np.array(1.04 * 5.0))))
    assert layer("obstacle_layer_single").noise_cut_m == 0.06


def test_an_unknown_range_noise_model_is_refused():
    cfg = MODULE.load_layer_config(ROOT / "config/obstacle_layer_single_a2.yaml")
    cfg["range_noise"] = {"model": "lenient"}
    with pytest.raises(ValueError):
        MODULE.ObstacleLayer(cfg, hall=Bounds(-5, 10, -5, 5), error_table=TABLE, unloaded=Footprint(1.29, 0.17, 0.36),
                             loaded=Footprint(1.53, 0.17, 0.40), body_front_m=0.944, rear_axle_x_in_base_m=-0.34,
                             noise_seed=1)


def test_the_a2_config_differs_from_the_single_one_only_in_its_range_noise():
    import yaml

    a = yaml.safe_load((ROOT / "config/obstacle_layer_single.yaml").read_text())
    b = yaml.safe_load((ROOT / "config/obstacle_layer_single_a2.yaml").read_text())
    assert b.pop("range_noise") == {"model": "a2_spec"} and a == b


def test_the_planning_memory_marks_at_least_the_widest_range_bound():
    # Codex P3a review P2: a 0.1 m memory disk left a hit read 0.135 m short at 4.5 m outside the memory
    a2 = layer("obstacle_layer_single_a2")
    assert a2.memory.hit_radius_m == pytest.approx(a2.noise_cut_m) and a2.memory.hit_radius_m > 0.1
    assert layer("obstacle_layer_single").memory.hit_radius_m == pytest.approx(0.1)  # unchanged
