"""The loaded curvature rule in the mission planner (docs/plans/2026-11-14-loaded-curvature-rule.md 3.2):
plan_transport's / plan_loaded_delivery's ``transport_travel_config`` replaces the travel config for the loaded
transport search alone; None leaves every stage as before."""

from dataclasses import asdict, replace

import numpy as np
import pytest

from forklift_core.planning import PlannerConfig
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    SyntheticMissionGeometry,
    make_scenario,
    make_transport_planner_config,
    plan_loaded_delivery,
    plan_transport,
    site_poses,
)

ASSETS = (
    AssetSpec("test://barrel.usd", 0.600, 0.708, 0.901),
    AssetSpec("test://crate.usd", 0.450, 0.662, 0.188),
)
STAGES = ("approach", "insert", "extract", "transport", "withdraw", "return_home")


def same_stage(a, b):
    assert (a is None) == (b is None)
    if a is None:
        return
    for name, value in asdict(a).items():
        if name in ("turnaround", "start_backoff"):
            continue
        np.testing.assert_array_equal(asdict(b)[name], value)


def config():
    return make_transport_planner_config(curvature_limit_inv_m=0.29, clearance_m=0.10)


@pytest.mark.parametrize("seed", [0, 1])
def test_none_is_every_stage_as_before(seed):
    scenario = make_scenario(seed, ASSETS)
    a = plan_transport(scenario, config(), return_to=scenario.start_rear)
    b = plan_transport(scenario, config(), return_to=scenario.start_rear, transport_travel_config=None)
    assert a.success == b.success and a.status == b.status
    for stage in STAGES:
        same_stage(getattr(a, stage), getattr(b, stage))


@pytest.mark.parametrize("seed", [0, 2])
def test_the_rule_changes_the_transport_search_alone(seed):
    scenario = make_scenario(seed, ASSETS)
    base = plan_transport(scenario, config(), return_to=scenario.start_rear)
    if not base.success:
        pytest.skip(f"seed {seed}: no base plan ({base.status})")
    loaded = replace(config(), curvature_limit_inv_m=0.27)
    ruled = plan_transport(scenario, config(), return_to=scenario.start_rear, transport_travel_config=loaded)
    if not ruled.success:
        assert ruled.status.startswith("transport:")  # only the transport stage can fail differently
        return
    for stage in ("approach", "insert", "extract", "withdraw"):
        same_stage(getattr(base, stage), getattr(ruled, stage))
    kappa = np.abs(np.asarray(ruled.transport.curvatures_inv_m, dtype=float))
    assert kappa.max() <= 0.27 + 1e-9  # the transport search's arcs at the rule's curvature
    # the return (unloaded) keeps the run's curvature; it starts where the same withdraw ends
    if base.return_home is not None and ruled.return_home is not None:
        same_stage(base.return_home, ruled.return_home)


def test_the_loaded_delivery_takes_the_same_keyword():
    scenario = make_scenario(0, ASSETS)
    geometry = SyntheticMissionGeometry()
    extracted = site_poses(scenario.pickup, geometry)["extracted"]
    a = plan_loaded_delivery(scenario, extracted, config(), geometry=geometry)
    b = plan_loaded_delivery(scenario, extracted, config(), geometry=geometry, transport_travel_config=None)
    same_stage(a.transport, b.transport)
    loaded = replace(config(), curvature_limit_inv_m=0.27)
    c = plan_loaded_delivery(scenario, extracted, config(), geometry=geometry, transport_travel_config=loaded)
    if c.success:
        assert np.abs(np.asarray(c.transport.curvatures_inv_m, dtype=float)).max() <= 0.27 + 1e-9


def test_plannerconfig_is_a_dataclass_the_rule_can_replace():
    cfg = PlannerConfig()
    assert replace(cfg, curvature_limit_inv_m=0.27).curvature_limit_inv_m == 0.27
