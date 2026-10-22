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
    with pytest.raises(ValueError):
        S.check_prim_names(["P-1", "P-1\n"])  # a trailing newline names no other prim (Codex round 3 P3)
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


def test_a_tipped_or_lifted_pallet_counts_as_moved():
    """Codex round 3 P2: tipped 0.2 rad on an edge it moves 8 mm in plan but 79 mm in height; the bounds see both."""
    w = S.RestWatch()
    w.rest("P-3", (1.0, 2.0, 0.0), (1.0, 0.0, 0.0, 0.0))
    tipped = (math.cos(0.1), math.sin(0.1), 0.0, 0.0)  # 0.2 rad about x
    (m, r), ok = w.check("P-3", (1.0, 2.008, 0.079), tipped)
    assert not ok and r == pytest.approx(0.2) and m > 0.07
    assert not w.check("P-3", (1.0, 2.0, 0.011), (1.0, 0.0, 0.0, 0.0))[1]  # lifted 11 mm



# --- slot anchors (docs/plans/2026-10-22-slot-anchored-neighbours.md 2.2-2.3) ----------------------------------------
def _evidence(**over):
    ev = {"mission_id": "m2", "pallet_id": "P-5", "destination_id": "D-2", "docking_accepted": True, "lowered": True,
          "delivered": True, "trace_valid": True, "end_phase": "home_settle"}
    ev.update(over)
    return ev


def test_a_delivery_with_every_piece_of_evidence_is_anchored():
    assert S.decide_anchor(_evidence(), reason=None, trace_valid_now=True) == (True, "anchored")
    assert S.decide_anchor(_evidence(end_phase="return_home"), reason="Tracking timeout in return_home",
                           trace_valid_now=True) == (True, "anchored")  # a failure after the delivery keeps it


@pytest.mark.parametrize("over, reason, now, why", [
    ({"docking_accepted": False}, None, True, "docking_not_accepted"),
    ({"lowered": False}, None, True, "not_lowered"),
    ({"delivered": False}, None, True, "not_delivered"),
    ({"trace_valid": False}, None, True, "trace_invalid_at_end"),
    ({"end_phase": "recover_withdraw"}, None, True, "end_phase:recover_withdraw"),
    ({}, "Pallet missed the green destination center", True, "delivery_verdict"),
    ({}, "Delivered pallet is not grounded", True, "delivery_verdict"),
    ({}, "Fork still inside delivered pallet", True, "delivery_verdict"),
    ({}, "Actual truck/load footprint overlap in withdraw", True, "scene_damage"),
    ({}, None, False, "trace_invalid_now"),
])
def test_a_missing_piece_of_evidence_or_a_failed_verdict_is_not_anchored(over, reason, now, why):
    assert S.decide_anchor(_evidence(**over), reason=reason, trace_valid_now=now) == (False, why)


def test_the_anchors_are_put_in_the_goals_frame():
    slots = {"D-1": (3.4, -7.182, 0.0), "D-2": (3.4, -8.469, 0.0), "D-3": (3.4, -9.756, 0.0)}
    anchors = {"P-4": "D-1", "P-5": "D-2"}
    same = S.anchor_poses(anchors, slots, "D-3", slots["D-3"])  # a slot not visited: the goal is its registered pose
    assert same == {"P-4": pytest.approx(slots["D-1"]), "P-5": pytest.approx(slots["D-2"])}
    moved = S.anchor_poses(anchors, slots, "D-3", (3.4, -9.556, 0.0))  # a revisit: the goal moved 0.20 m
    assert moved["P-5"] == pytest.approx((3.4, -8.269, 0.0)) and moved["P-4"] == pytest.approx((3.4, -6.982, 0.0))
    turned = S.anchor_poses({"P-5": "D-2"}, slots, "D-3", (3.4, -9.756, 0.1))  # turned about the goal
    x, y, yaw = turned["P-5"]
    assert yaw == pytest.approx(0.1) and math.hypot(x - 3.4, y + 9.756) == pytest.approx(1.287)
    assert S.anchor_poses({}, slots, "D-3", slots["D-3"]) == {}
