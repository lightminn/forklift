"""The offline cross-check of the observation start check (revision plan 9.22, the lead's item 5), on a synthetic hall."""

import json
import pickle
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import start_check_cross as X  # noqa: E402

from forklift_core.planning import Bounds, Pose2D, Rectangle  # noqa: E402
from forklift_core.planning.pallet_mission import (  # noqa: E402
    PalletSite,
    SyntheticMissionGeometry,
    TransportScenario,
    make_transport_planner_config,
)

RES = 0.05


def _arrays(cells_xy=()):
    occ = np.zeros((int(11 / RES), int(8 / RES)), bool)
    live = occ.copy()
    for x, y in cells_xy:
        live[int((x + 5) / RES), int((y + 4) / RES)] = True
    return {
        "occupied": live.copy(),
        "live": live,
        "memory_observed": np.zeros_like(occ),
        "slam_layer": np.zeros_like(occ),
        "own_left_out": np.zeros_like(occ),
        "origin": np.array([-5.0, -4.0]),
        "resolution_m": np.array(RES),
    }


def test_the_four_cells_tell_the_grid_that_refuses_the_pose_that_reversed_into_it():
    # moved back from (-1, 0) to (-3, 0); after the reverse a wall cell stands 5 cm behind the new rear
    world = TransportScenario(
        0,
        Pose2D(-3.0, 0.0, 0.0),
        PalletSite(4, 3, 0),
        PalletSite(4, -3, 0),
        (),
        Bounds(-5, 6, -4, 4),
    )
    geometry = SyntheticMissionGeometry()
    rear_edge = -3.0 - geometry.unloaded_footprint.rear_m
    pre, post = _arrays(), _arrays([(rear_edge - 0.05, 0.0)])
    payload = {
        "world": world,
        "config": make_transport_planner_config(),
        "geometry": geometry,
        "pickup_bounds": None,
        "grid_kwargs": {
            "occupancy": X._grid(post),
            "pickup_obstacle": Rectangle(4, 3, 1, 1, 0),
        },
    }
    premove = {
        "meta": json.dumps({"own_pose": [-1.0, 0.0, 0.0], "grid_applied": [0, 0, 0]}),
        **pre,
    }
    prepare = {
        "payload": pickle.dumps(payload),
        "arrays": post,
        "meta": {"grid_applied": [0, 0, 0]},
    }
    out = X.cross(premove, prepare)
    cl = f"{payload['config'].clearance_m:g}"
    cells = out["cells"]
    assert set(cells) == {
        "pre_move@pre_move",
        "pre_move@submitted",
        "post_reverse@pre_move",
        "post_reverse@submitted",
    }
    assert (
        cells["post_reverse@submitted"]["by_clearance"][cl]["start_check_free"] is False
    )
    assert cells["post_reverse@submitted"]["by_clearance"][cl]["layers_refusing"] == [
        "live"
    ]
    assert (
        cells["post_reverse@submitted"]["by_clearance"]["0"]["start_check_free"] is True
    )  # only the clearance
    assert (
        cells["post_reverse@pre_move"]["by_clearance"][cl]["start_check_free"] is True
    )
    assert cells["pre_move@submitted"]["by_clearance"][cl]["start_check_free"] is True
    assert cells["post_reverse@submitted"]["by_clearance"][cl]["bounds_free"] is True
    assert (
        out["poses"]["post_reverse"] == [-3.0, 0.0, 0.0]
        and out["submitted_layers"] == "layers_matched"
    )
    # Codex review P2: the payload's grid is the submitted one, whatever the capture's layers say, both ways
    for sent, layers, refused in ((post, pre, True), (pre, post, False)):
        payload["grid_kwargs"]["occupancy"] = X._grid(sent)
        prepare = {"payload": pickle.dumps(payload), "arrays": layers, "meta": {}}
        got = X.cross(premove, prepare)
        assert got["submitted_layers"] == "layers_unmatched"
        row = got["cells"]["post_reverse@submitted"]["by_clearance"][cl]
        assert row["start_check_free"] is (not refused) and row["layers_refusing"] == []


def test_the_permission_parts_are_the_body_and_each_blade():
    geometry = SyntheticMissionGeometry()
    parts = X.permission_parts(
        geometry, [(0.5, 1.2, 0.1, 0.2), (0.5, 1.2, -0.2, -0.1)], 0.34
    )
    body, lat, lon = parts[0]
    assert (lat, lon) == (
        0.0,
        0.0,
    ) and body.rear_m == geometry.unloaded_footprint.rear_m
    assert body.front_m == geometry.axle_to_fork_tip_m - geometry.carriage_limit_m
    blade, lat, lon = parts[1]
    assert (
        abs(lat - 0.15) < 1e-12
        and abs(lon - (0.84 + 1.54) / 2) < 1e-12
        and abs(blade.half_width_m - 0.05) < 1e-12
    )
    table = X.shapes_table(geometry, parts)
    assert table["planner_hull"]["front_m"] == geometry.unloaded_footprint.front_m
    assert len(table["permission_reversing_parts"]) == 3


def test_the_layers_follow_the_planners_own_composition():
    # Codex review: the cells under the truck leave the memory's layers only; the SLAM layer counts only when used
    a = _arrays([(0.0, 0.0)])
    a["own_left_out"] = a["live"].copy()
    a["memory_observed"] = a["live"].copy()
    assert (
        X._grid(a, "live").occupied.any()
        and not X._grid(a, "memory_observed").occupied.any()
    )
    a["use_slam"] = np.array(False)
    assert X._layers(a) == ("live", "memory_observed")
    a["use_slam"] = np.array(True)
    assert X._layers(a) == X.LAYERS
