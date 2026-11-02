"""B-P2c: the fake vehicle's start relocalization for the screen (an injection, the live runner's contract): no pose and
slam.reloc in the state, the candidates picked twice-confirmed on the page and sent as relocalize_pick with their
generation, a retry, a STOP that interrupts, and what would move the truck refused ``relocalizing``."""

import itertools

import pytest

from console.mock_vehicle.core import Config, MockVehicle


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


@pytest.fixture
def v():
    tokens = (f"tok{i}" for i in itertools.count())
    vehicle = MockVehicle(Clock(), lambda: next(tokens), Config())
    vehicle.connect("a", "c1")
    return vehicle


def _lease(v):
    reply = v.handle("a", {"t": "lease_request", "request_id": "L", "session_epoch": v.session_epoch}, conn_id="c1")
    return reply["lease_id"]


def _req(v, lease, t, rid, **fields):
    return v.handle("a", {"t": t, "request_id": rid, "session_epoch": v.session_epoch, "lease_id": lease,
                          "control_gen": v.control_gen, **fields}, conn_id="c1")


def test_an_injected_relocalization_hides_the_pose_and_shows_its_candidates(v):
    v.inject("relocalize")
    st = v.make_state()
    reloc = st["slam"]["reloc"]
    assert st["pose"] is None and st["slam"]["mode"] == "localization"
    assert reloc["state"] == "awaiting_pick" and reloc["generation"] == 1 and len(reloc["candidates"]) >= 2
    assert [c["index"] for c in reloc["candidates"]] == list(range(len(reloc["candidates"])))
    assert v.connect("b", "c2")["capabilities"]["relocalize"] is True


def test_a_pick_of_the_current_set_places_the_truck_there(v):
    v.inject("relocalize")
    lease = _lease(v)
    target = v.make_state()["slam"]["reloc"]["candidates"][1]["base"]
    assert _req(v, lease, "relocalize_pick", "p", generation=1, candidate=1)["t"] == "ack"
    assert v.make_state()["slam"]["reloc"]["state"] == "localizing"
    v.clock.t += 3.0
    v.step(0.1)
    st = v.make_state()
    assert "reloc" not in (st.get("slam") or {}) and st["pose"]["x_m"] == pytest.approx(target[0])


@pytest.mark.parametrize("fields, reason", [
    ({"generation": 0, "candidate": 0}, "reloc_stale_pick"),
    ({"generation": 1, "candidate": 9}, "reloc_pick_refused"),
    ({"generation": True, "candidate": 0}, "invalid"),
    ({"generation": 1}, "invalid"),
])
def test_a_wrong_pick_is_refused(v, fields, reason):
    v.inject("relocalize")
    lease = _lease(v)
    reply = _req(v, lease, "relocalize_pick", "p", **fields)
    assert reply["t"] == "reject" and reply["reason"] == reason


def test_a_stop_interrupts_and_a_retry_brings_a_new_set(v):
    v.inject("relocalize")
    lease = _lease(v)
    v.handle("a", {"t": "stop", "request_id": "s", "session_epoch": v.session_epoch}, conn_id="c1")
    reloc = v.make_state()["slam"]["reloc"]
    assert reloc["state"] == "stopped" and reloc["candidates"] == [] and reloc["generation"] == 2
    assert _req(v, lease, "relocalize_retry", "r", generation=2)["t"] == "ack"  # allowed while STOPPED
    assert v.make_state()["slam"]["reloc"]["state"] == "relocalizing"
    v.clock.t += 3.0
    v.step(0.1)
    reloc = v.make_state()["slam"]["reloc"]
    assert reloc["state"] == "awaiting_pick" and reloc["generation"] == 3 and reloc["candidates"]


@pytest.mark.parametrize("t, fields", [("mission_submit", {"pallet_id": "P-01", "destination_id": "D-A",
                                                           "targets_revision": 0}),
                                       ("mode_request", {"to": "TELEOP", "source": "direct"}), ("arm", {})])
def test_what_would_move_the_truck_is_refused_while_relocalizing(v, t, fields):
    v.inject("relocalize")
    lease = _lease(v)
    reply = _req(v, lease, t, "x", **fields)
    assert reply["t"] == "reject" and reply["reason"] == "relocalizing"


def test_without_a_relocalization_the_commands_are_refused(v):
    lease = _lease(v)
    assert _req(v, lease, "relocalize_pick", "p", generation=1, candidate=0)["reason"] == "not_relocalizing"
    assert _req(v, lease, "relocalize_retry", "r", generation=1)["reason"] == "not_relocalizing"


# --- F1: the operator's placed pose (docs/plans/2026-10-31-operator-placed-start.md 1.2) ---------------------------------
def test_a_placement_near_the_truck_previews_and_its_confirmation_places_the_truck(v):
    v.inject("relocalize")
    lease = _lease(v)
    true = (v.pose.x_m, v.pose.y_m, v.pose.yaw_rad)
    assert _req(v, lease, "relocalize_place", "pl", generation=1, pose=[true[0] + 0.4, true[1], true[2] + 0.1])["t"] == "ack"
    r = v.make_state()["slam"]["reloc"]
    assert r["state"] == "placing" and r["places"] == 1 and r["max_places"] == 3
    v.clock.t += 2.0
    v.step(0.1)
    r = v.make_state()["slam"]["reloc"]
    assert r["state"] == "place_preview" and r["preview_id"] == 1 and r["preview_points"]
    assert r["preview_twins"] == 1  # the fake's turned-around look-alike (its scores against the fake's R_AMB 0.95)
    assert r["preview"] == pytest.approx(list(true))
    # the test's clock jumped past the lease without the page's echoes: a new one (its own request id; "L" replays)
    lease = v.handle("a", {"t": "lease_request", "request_id": "L2", "session_epoch": v.session_epoch},
                     conn_id="c1")["lease_id"]
    assert _req(v, lease, "relocalize_place_confirm", "c", generation=1, preview_id=1)["t"] == "ack"
    v.clock.t += 3.0
    v.step(0.1)
    assert v.make_state()["pose"]["x_m"] == pytest.approx(true[0])


def test_a_placement_far_from_the_truck_is_refused_back_to_the_candidates(v):
    v.inject("relocalize")
    lease = _lease(v)
    _req(v, lease, "relocalize_place", "pl", generation=1, pose=[v.pose.x_m + 5.0, v.pose.y_m, v.pose.yaw_rad])
    v.clock.t += 2.0
    v.step(0.1)
    r = v.make_state()["slam"]["reloc"]
    assert r["state"] == "awaiting_pick" and r["place_reason"] == "low_fit" and r["places"] == 1


@pytest.mark.parametrize("t, fields", [
    ("relocalize_place", {"generation": 1, "pose": [1.0, 2.0]}),
    ("relocalize_place", {"generation": 1, "pose": [1.0, 2.0, True]}),
    ("relocalize_place_confirm", {"generation": 1}),
])
def test_a_malformed_placement_or_confirmation_is_invalid(v, t, fields):
    v.inject("relocalize")
    lease = _lease(v)
    assert _req(v, lease, t, "x", **fields)["reason"] == "invalid"
