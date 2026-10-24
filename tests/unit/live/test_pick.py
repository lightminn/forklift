"""CT-2 D3/D5: pick_request is acked by the arbiter and answered by a separate pick (docs/plans/2026-10-24-ct2-next-pick.md)."""

import itertools
import pickle

from forklift_core.live.link import Channels, Link
from forklift_core.live.pick import PickService
from forklift_core.mission.commands import Context, MissionCommands
from forklift_core.mission.next_pick import PickSnapshot
from forklift_core.mission.targets import Pose, TargetsBook
from forklift_core.planning import Pose2D
from forklift_core.planning.geometry import Bounds
from forklift_core.planning.pallet_mission import PalletSite, SyntheticMissionGeometry, TransportScenario
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig, Feedback

EPOCH = 5
SAFE = {"load": {"loaded": False}, "fork_in_pocket": {"value": False}, "near": None,
        "frame": {"hold": "none", "slam": True}, "mission": {"phase": "observe"}}
G = SyntheticMissionGeometry(alignment_straight_m=2.1, delivery_straight_m=1.5)


def targets():
    tb = TargetsBook()
    tb.add_shape("epal6", 0.6, 0.8)
    tb.add_pallet("P-1", "epal6", Pose(0.8, 0.0, 0.0), depth_m=0.6, width_m=0.8)
    tb.add_destination("D-1", Pose(3.0, -9.0, 0.0))
    return tb


def snapshot(tb):
    sc = TransportScenario(seed=0, start_rear=Pose2D(-3.0, 0.0, 0.0), pickup=PalletSite(0, 0, 0),
                           destination=PalletSite(0, 0, 0), props=(), bounds=Bounds(-6, 6, -12, 6))
    return PickSnapshot(tb, (-3.0, 0.0, 0.0), (-3.0, 0.0, 0.0), sc, G, 0.10, {}, (0.6, 0.8, 0.144), 0.20)


def rig(service=None):
    counter = itertools.count(1)
    m = MissionCommands()
    m.targets = targets()
    a = Arbiter(EPOCH, ArbiterConfig(), lambda: f"tok{next(counter)}", mission=m)
    ch = Channels(256)
    link = Link(a, ch)
    if service is not None:
        link.pick = service
        a.next_pick_allowed = True
    return m, a, ch, link


def cycle(m, link, t):
    stop_kind = link.begin(t, Feedback(t, 0.0))
    records = m.apply(t, Context(stop_kind=stop_kind))
    return link.end(t, records, SAFE, (0.0, 0.0))


def out_for(ch, conn):
    return [msg for c, msg in ch.take_out() if c == conn]


def ask(ch, rid="K"):
    ch.message("A", "op", {"t": "pick_request", "request_id": rid, "session_epoch": EPOCH})


def test_without_a_service_the_request_is_unsupported():
    m, a, ch, link = rig()
    ch.connect("A", "op")
    ask(ch)
    cycle(m, link, 0.01)
    replies = out_for(ch, "A")
    assert [(r["t"], r.get("reason")) for r in replies if r.get("of") == "pick_request"] == [("reject", "unsupported")]
    assert not [r for r in replies if r["t"] == "pick"]


def test_the_ack_comes_first_then_the_pick_with_the_snapshot_revision():
    m, a, ch, link = rig()
    link.pick, a.next_pick_allowed = PickService(lambda: snapshot(m.targets)), True
    ch.connect("A", "op")
    ask(ch)
    cycle(m, link, 0.01)
    cycle(m, link, 0.02)
    replies = out_for(ch, "A")
    kinds = [r["t"] for r in replies if r["t"] in ("ack", "pick")]
    assert kinds == ["ack", "pick"]
    pick = next(r for r in replies if r["t"] == "pick")
    assert pick["request_id"] == "K" and pick["reason"] == "ok"
    assert pick["pick"] == {"pallet_id": "P-1", "destination_id": "D-1"}
    assert pick["targets_revision"] == m.targets.revision


def test_a_resend_that_gets_the_stored_ack_is_answered_again():
    m, a, ch, link = rig()
    link.pick, a.next_pick_allowed = PickService(lambda: snapshot(m.targets)), True
    ch.connect("A", "op")
    ask(ch)
    cycle(m, link, 0.01)
    ask(ch)  # the same request id: the stored ack, and a pick computed again
    cycle(m, link, 0.02)
    picks = [r for r in out_for(ch, "A") if r["t"] == "pick"]
    assert len(picks) == 2 and all(p["request_id"] == "K" for p in picks)


def test_a_failed_computation_is_still_a_pick():
    def broken():
        raise RuntimeError("no scene")

    m, a, ch, link = rig()
    link.pick, a.next_pick_allowed = PickService(broken), True
    ch.connect("A", "op")
    ask(ch)
    cycle(m, link, 0.01)
    pick = next(r for r in out_for(ch, "A") if r["t"] == "pick")
    assert pick["pick"] is None and pick["reason"] == "error:RuntimeError"


def test_a_mission_on_refuses_the_request():
    m, a, ch, link = rig()
    link.pick, a.next_pick_allowed = PickService(lambda: snapshot(m.targets)), True
    m.status = "running"
    ch.connect("A", "op")
    ask(ch)
    cycle(m, link, 0.01)
    replies = out_for(ch, "A")
    assert [(r["t"], r.get("reason")) for r in replies if r.get("of") == "pick_request"] == [("reject", "mission_active")]


def test_an_answer_for_a_gone_connection_is_dropped():
    m, a, ch, link = rig()
    link.pick, a.next_pick_allowed = PickService(lambda: snapshot(m.targets)), True

    class Slow:  # a worker that answers one cycle later
        def __init__(self):
            self.jobs = {}

        def submit(self, name, payload):
            self.jobs[1] = None
            return 1

        def poll(self, job):
            if self.jobs[job] is None:
                self.jobs[job] = "ready"
                return None
            from forklift_core.mission.next_pick import recommend

            return True, {"result": recommend(snapshot(m.targets))}

    link.pick = PickService(lambda: snapshot(m.targets), worker=Slow())
    ch.connect("A", "op")
    ask(ch)
    cycle(m, link, 0.01)
    ch.connect("B", "op")  # the client reconnects before the answer
    ch.disconnect("A", "op")
    cycle(m, link, 0.02)
    cycle(m, link, 0.03)
    assert not [r for c, r in ch.take_out() if r["t"] == "pick"]


def test_the_snapshot_survives_the_worker_pipe():
    snap = snapshot(targets())
    again = pickle.loads(pickle.dumps(snap, protocol=pickle.HIGHEST_PROTOCOL))
    from forklift_core.mission.next_pick import recommend

    assert recommend(again) == recommend(snap)
