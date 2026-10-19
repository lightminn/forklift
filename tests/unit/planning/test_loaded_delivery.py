"""Persistent console plan D5 6c: the loaded delivery plan -- from where the truck stands carrying the pallet, the
same steps plan_transport takes from the extracted pose on (transport search with its tight retry on a grid, the
delivery straight, the withdraw straight, the return), with plan_transport itself untouched."""

import numpy as np
import pytest

from forklift_core.planning import Bounds, Pose2D
from forklift_core.planning.pallet_mission import (
    AssetSpec,
    PalletSite,
    TransportScenario,
    make_scenario,
    plan_loaded_delivery,
    plan_transport,
    site_poses,
    SyntheticMissionGeometry,
)

ASSETS = (
    AssetSpec("test://barrel.usd", 0.600, 0.708, 0.901),
    AssetSpec("test://crate.usd", 0.450, 0.662, 0.188),
)


def same(a, b):
    assert (a is None) == (b is None)
    if a is None:
        return
    assert a.status == b.status and a.success == b.success
    np.testing.assert_array_equal(np.asarray(a.poses), np.asarray(b.poses))
    np.testing.assert_array_equal(np.asarray(a.directions), np.asarray(b.directions))


@pytest.mark.parametrize("seed", [0, 1, 2])
@pytest.mark.parametrize("home", [False, True])
def test_from_the_extracted_pose_it_is_plan_transports_tail(seed, home):
    scenario = make_scenario(seed, ASSETS)
    geometry = SyntheticMissionGeometry()
    return_to = scenario.start_rear if home else None
    full = plan_transport(scenario, return_to=return_to)
    extracted = site_poses(scenario.pickup, geometry)["extracted"]
    loaded = plan_loaded_delivery(scenario, extracted, return_to=return_to)
    assert loaded.success == full.success and loaded.status == full.status
    if full.success:
        same(loaded.transport, full.transport)
        same(loaded.withdraw, full.withdraw)
        same(loaded.return_home, full.return_home)
    assert loaded.approach is None and loaded.insert is None and loaded.extract is None


def test_a_failed_stage_names_itself():
    scenario = TransportScenario(0, Pose2D(-3, 0, 0), PalletSite(4, 3, 0), PalletSite(40, -3, 0), (),
                                 Bounds(-5, 6, -4, 4))  # the destination outside the bounds
    out = plan_loaded_delivery(scenario, Pose2D(0, 0, 0))
    assert not out.success and out.status.startswith("transport:")


def test_the_trace_names_the_loaded_steps():
    scenario = make_scenario(0, ASSETS)
    trace = []
    extracted = site_poses(scenario.pickup, SyntheticMissionGeometry())["extracted"]
    plan_loaded_delivery(scenario, extracted, return_to=scenario.start_rear, trace=trace)
    assert [t["stage"] for t in trace][:2] == ["transport_search", "transport_straight"]


def test_the_worker_refuses_a_plan_with_no_docking_straight_left():
    """Codex 6c round 2 P2-1: from the predelivery pose the transport is the delivery straight alone -- docking
    cannot be armed (arm_docking's final_straight_prefix is None), so the preparation fails by name."""
    from forklift_core.planning import plan_worker as pw  # noqa: PLC0415

    scenario = make_scenario(0, ASSETS)
    geometry = SyntheticMissionGeometry()
    at_predelivery = site_poses(scenario.destination, geometry)["predelivery"]
    out = pw._run("loaded_delivery", {"args": (scenario, at_predelivery), "kwargs": {},
                                      "keep_m": geometry.delivery_straight_m})
    assert not out["result"].success and out["result"].status == "transport:no_final_straight"
    extracted = site_poses(scenario.pickup, geometry)["extracted"]
    out = pw._run("loaded_delivery", {"args": (scenario, extracted), "kwargs": {"return_to": scenario.start_rear},
                                      "keep_m": geometry.delivery_straight_m})
    assert out["result"].success and "wall_s" in out
