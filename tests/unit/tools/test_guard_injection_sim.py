"""The guard (2) injection plan's D search helpers (docs/plans/2026-11-09-live-riib-guard-injection.md 1)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import guard_injection_sim as G  # noqa: E402
from forklift_core.planning.geometry import Footprint, Rectangle  # noqa: E402

SHAPE = [(Footprint(1.0, 0.2, 0.3), 0.0, 0.0)]


def test_the_real_outline_keeps_the_centre_and_heading_at_epal6_size():
    r = G.real_outline(Rectangle(1.0, 2.0, 1.02, 1.22, 0.1))
    assert (r.x_m, r.y_m, r.length_m, r.width_m, r.yaw_rad) == (1.0, 2.0, 0.6, 0.8, 0.1)


def test_the_stage_rule_drops_the_other_stage_copies_and_tall_props():
    def p(uri, h):
        return {"asset": {"uri": uri, "height_m": h},
                "rectangle": {"x_m": 0.0, "y_m": 0.0, "length_m": 1.0, "width_m": 1.0, "yaw_rad": 0.0}}
    props = [p("session:pallet:P-1@search", 0.144), p("session:pallet:P-1@slot", 0.144), p("session:pallet:P-2", 0.144),
             p("riibotics:slot_marker", 1.3)]
    assert len(G.stage_props(props, "search")) == 2 and len(G.stage_props(props, "slot")) == 2


def test_the_neighbour_side_is_its_side_of_the_goal_heading():
    left = Rectangle(0.0, 2.0, 1.0, 1.0, 0.0)
    right = Rectangle(0.0, -3.0, 1.0, 1.0, 0.0)
    assert G.neighbour_side((0.0, 0.0, 0.0), 0.0, [left, right], SHAPE) == 1.0  # the nearer one (left) decides
    assert G.neighbour_side((0.0, 0.0, 0.0), 0.0, [Rectangle(0.0, -1.5, 1.0, 1.0, 0.0), left], SHAPE) == -1.0
    assert G.neighbour_side((0.0, 0.0, 0.0), 1.5707963267948966, [Rectangle(-2.0, 0.0, 1.0, 1.0, 0.0)], SHAPE) == 1.0


def test_the_ramp_grows_from_its_start_to_its_size():
    assert G.ramp_dx(0.4, 0.5, 0.2, 1.0, 1.0) == 0.0
    assert G.ramp_dx(1.0, 0.5, 0.2, 1.0, -1.0) == pytest.approx(-0.1)
    assert G.ramp_dx(3.0, 0.5, 0.2, 1.0, 1.0) == pytest.approx(0.2)


def test_the_reverse_clearance_sees_a_prop_behind():
    behind = Rectangle(-0.5, 0.0, 0.2, 1.0, 0.0)  # its front face at -0.4; the shape's rear at -0.2
    assert G.reverse_clear((0.0, 0.0, 0.0), SHAPE, [behind], length_m=0.1) == pytest.approx(0.1, abs=1e-9)
    assert G.reverse_clear((0.0, 0.0, 0.0), SHAPE, [behind], length_m=0.3) <= 0.0


def test_established_needs_every_condition():
    ok = {"status": "guard_stop", "zero": {"s_from_m": 0.03}, "min_real_m": 0.1, "reverse_real_m": 0.05}
    assert G.established(ok)
    assert not G.established({**ok, "zero": {"s_from_m": 0.06}})
    assert not G.established({**ok, "min_real_m": 0.0})
    assert not G.established({**ok, "reverse_real_m": 0.0})
    assert not G.established({**ok, "status": "failed:cross_track"})
