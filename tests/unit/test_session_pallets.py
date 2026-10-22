"""sim/isaac/session_pallets.py (plan 2026-10-22-rigid-session-pallets 2.1-2.4, 2.7)."""
import importlib.util
import math
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("session_pallets", ROOT / "sim/isaac/session_pallets.py")
S = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(S)


def test_a_pallet_id_names_a_prim_without_its_dash():
    assert S.pallet_prim("P-1") == "/World/SessionPallets/P_1"
    assert S.self_filter_prefix("P-1") == "/World/SessionPallets/P_1/"
    assert not S.self_filter_prefix("P-1").startswith(S.self_filter_prefix("P-10")[:-1] + "/")
    with pytest.raises(ValueError):
        S.prim_name("1-P")
    with pytest.raises(ValueError, match="same prim"):
        S.check_prim_names(["P-1", "P_1"])
    S.check_prim_names(["P-1", "P-2", "P-10"])


def test_the_prim_paths_are_valid_usd_paths():
    sdf = pytest.importorskip("pxr.Sdf")
    for pid in ("P-1", "P-5", "P-10"):
        assert sdf.Path.IsValidPathString(S.pallet_prim(pid))
    assert not sdf.Path.IsValidPathString("/World/SessionPallets/P-1")  # why the dash goes


class _H:
    def __init__(self, pose, mass=10.0):
        self.pose, self.mass, self.set = pose, mass, []

    def get_world_pose(self):
        return self.pose

    def set_world_pose(self, *a):
        self.set.append(a)

    def get_mass(self):
        return self.mass

    def get_linear_velocity(self):
        return ("v", self.pose)

    def get_angular_velocity(self):
        return ("w", self.pose)


def test_the_switch_reads_the_mission_pallet_and_changes_only_at_a_tick_head():
    a, b = _H(((1.0, 0.0, 0.0), (1, 0, 0, 0))), _H(((5.0, 3.0, 0.0), (1, 0, 0, 0)), mass=12.0)
    sw = S.PalletSwitch({"P-1": a, "P-2": b}, "P-1")
    assert sw.get_world_pose() is a.pose and sw.get_mass() == 10.0
    sw.request("P-2")  # the adoption, mid-tick
    assert sw.get_world_pose() is a.pose  # the rest of that tick reads the old one
    assert sw.handle("P-2").get_world_pose() is b.pose  # the adoption reads the new one directly
    assert sw.apply_pending(42) == ("P-1", "P-2")
    assert sw.get_world_pose() is b.pose and sw.get_mass() == 12.0
    sw.set_world_pose((0, 0, 0), (1, 0, 0, 0))
    assert b.set and not a.set
    assert sw.switches == [{"tick": 42, "from": "P-1", "to": "P-2"}]
    sw.request("P-2")
    assert sw.apply_pending(43) is None and sw.apply_pending(44) is None  # the same pallet, nothing pending
    with pytest.raises(ValueError):
        sw.request("P-9")


def test_a_switch_away_from_a_carried_pallet_is_refused():
    sw = S.PalletSwitch({"P-1": _H(None), "P-2": _H(None)}, "P-1")
    sw.request("P-2")
    with pytest.raises(RuntimeError, match="session_pallet_switch_while_carrying:P-1->P-2"):
        sw.apply_pending(7, carried_id="P-1")
    sw.request("P-2")
    assert sw.apply_pending(8, carried_id=None) == ("P-1", "P-2")


def _q(yaw):
    return (math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2))


def test_a_resting_pallet_may_move_one_centimetre_and_one_hundredth_of_a_radian():
    w = S.RestWatch()
    w.rest("P-2", (5.0, 3.0, 0.0), _q(0.1))
    assert w.check("P-2", (5.0099, 3.0, 0.0), _q(0.1099))[1]
    (m, r), ok = w.check("P-2", (5.011, 3.0, 0.0), _q(0.1))
    assert not ok and m == pytest.approx(0.011)
    assert not w.check("P-2", (5.0, 3.0, 0.0), _q(0.1 + 0.0101))[1]
    assert w.largest["P-2"][0] == pytest.approx(0.011)
    w.rest("P-2", (8.0, 0.0, 0.0), _q(0.0))  # put down elsewhere: that is its rest now
    assert w.check("P-2", (8.0, 0.0, 0.0), _q(0.0))[1]
