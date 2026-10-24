"""CT-2 D4: the Python queue's auto item (docs/plans/2026-10-24-ct2-next-pick.md; console/web/js/queue.js is the
reference -- test_mission_queue_port.py runs the two on the same events)."""

import itertools
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import mission_queue as mq  # noqa: E402

AUTO = {"item_id": "A", "kind": "auto", "scope": "remaining", "pallet_id": None, "destination_id": None,
        "label": "남은 팔레트 전부", "added_unix": 0.0}
STAND = {"mode": "PAUSED", "stopped_confirmed": True, "feedback_ok": True, "mission": None, "load": None}


def targets(revision=1):
    return {"targets_revision": revision,
            "pallets": [{"pallet_id": "P-1", "state": "available"}, {"pallet_id": "P-2", "state": "available"}],
            "destinations": [{"destination_id": "D-1"}, {"destination_id": "D-2"}]}


def ctx(revision=1, next_pick=True, state=STAND):
    return {"connected": True, "welcomed": True, "has_lease": True, "state": state, "targets": targets(revision),
            "next_pick": next_pick}


def started():
    q = mq.MissionQueue()
    q.on_welcome(7)
    q.add(dict(AUTO))
    q.resume(0)
    q.on_state(STAND, 10)
    return q, (f"r{k}" for k in itertools.count(1))


def pick_msg(rid, revision=1, reason="ok", pick=("P-1", "D-1")):
    return {"t": "pick", "request_id": rid, "targets_revision": revision, "reason": reason,
            "pick": None if pick is None else {"pallet_id": pick[0], "destination_id": pick[1]},
            "plan": [list(pick)] if pick else [], "missions": 1 if pick else 0, "blocked": {}}


def key(q, rid):
    return {"epoch": q.epoch, "item_id": "A", "request_id": rid}


def test_one_auto_item_only():
    q = mq.MissionQueue()
    assert q.add(dict(AUTO)) and not q.add(dict(AUTO, item_id="B"))


def test_ask_hold_then_submit_when_the_conditions_hold():
    q, ids = started()
    out = q.next(ctx(), 20, lambda: next(ids))
    assert out == {"pick_request": {"item_id": "A", "request_id": "r1"}} and q.phase == "picking"
    assert q.next(ctx(), 21, lambda: next(ids)) is None  # nothing more while it waits
    q.on_answer(key(q, "r1"), {"t": "ack"}, 22)
    q.on_pick(key(q, "r1"), pick_msg("r1"), 30, 1)
    assert q.phase == "picked" and q.held["pallet_id"] == "P-1"
    moving = dict(STAND, stopped_confirmed=False)
    assert q.next(ctx(state=moving), 31, lambda: next(ids)) is None and q.run == "running"  # waits, no stop
    q.on_state(STAND, 40)
    out = q.next(ctx(), 41, lambda: next(ids))
    assert out["submit"]["pallet_id"] == "P-1" and out["submit"]["destination_id"] == "D-1"
    assert q.phase == "submitting"


def test_a_done_mission_keeps_the_auto_item_and_none_remaining_ends_it():
    q, ids = started()
    q.next(ctx(), 20, lambda: next(ids))
    q.on_pick(key(q, "r1"), pick_msg("r1"), 30, 1)  # the pick before its ack
    assert q.next(ctx(), 35, lambda: next(ids)) is None  # held, not submitted until the ack
    q.on_answer(key(q, "r1"), {"t": "ack"}, 36)
    sub = q.next(ctx(), 41, lambda: next(ids))["submit"]
    q.on_answer({"epoch": q.epoch, "item_id": "A", "request_id": sub["request_id"]}, {"t": "ack", "mission_id": "M-1"})
    q.on_result({"mission_id": "M-1", "outcome": "done"}, 100)
    assert [it["item_id"] for it in q.items] == ["A"] and q.run == "running"
    assert q.finished[0]["pallet_id"] == "P-1" and q.finished[0]["destination_id"] == "D-1"
    assert q.items[0]["moved"] == 1
    q.on_state(STAND, 1200)
    out = q.next(ctx(revision=2), 1200, lambda: next(ids))
    rid = out["pick_request"]["request_id"]
    q.on_pick(key(q, rid), pick_msg(rid, 2, "none_remaining", None), 1300, 2)
    assert q.items == [] and q.run == "stopped" and q.stop_reason == "완료"


def test_a_changed_list_is_asked_once_more_then_stops():
    q, ids = started()
    q.next(ctx(), 20, lambda: next(ids))
    q.on_pick(key(q, "r1"), pick_msg("r1", 1), 30, 2)  # the page holds revision 2 now
    assert q.phase == "waiting" and q.run == "running"
    out = q.next(ctx(revision=2), 31, lambda: next(ids))
    rid = out["pick_request"]["request_id"]
    q.on_pick(key(q, rid), pick_msg(rid, 2), 40, 3)
    assert q.run == "stopped" and q.stop_reason == "대상 목록이 계속 바뀜"


def test_failures_and_refusals_stop_without_an_uncertain_mark():
    for msg, why in ((pick_msg("r1", reason="all_blocked", pick=None), "자동 선택 실패: all_blocked"),
                     (pick_msg("r1", reason="error:RuntimeError", pick=None), "자동 선택 실패: error:RuntimeError")):
        q, ids = started()
        q.next(ctx(), 20, lambda: next(ids))
        q.on_pick(key(q, "r1"), msg, 30, 1)
        assert q.stop_reason == why and not q.items[0].get("uncertain")
    q, ids = started()
    q.next(ctx(), 20, lambda: next(ids))
    q.on_answer(key(q, "r1"), {"t": "reject", "reason": "mission_active"}, 25)
    assert q.stop_reason == "자동 선택 거부: mission_active" and not q.items[0].get("uncertain")


def test_no_pick_by_the_deadline_stops():
    q, ids = started()
    q.next(ctx(), 20, lambda: next(ids))
    for t in range(1000, 20 + mq.PICK_DEADLINE_MS, 1000):  # the states keep coming: only the pick is late
        q.on_state(STAND, t)
    q.on_state(STAND, 20 + mq.PICK_DEADLINE_MS)
    assert q.tick(20 + mq.PICK_DEADLINE_MS) == {} and q.run == "running"  # equal is not over
    q.on_state(STAND, 20 + mq.PICK_DEADLINE_MS)
    q.tick(21 + mq.PICK_DEADLINE_MS)
    assert q.stop_reason == "자동 선택 응답 없음"


def test_a_stop_while_picking_forgets_the_request_and_a_late_pick_is_ignored():
    q, ids = started()
    q.next(ctx(), 20, lambda: next(ids))
    assert q.on_disconnect() == {"forget": "r1"}
    assert q.phase == "waiting" and not q.items[0].get("uncertain")
    assert q.on_pick(key(q, "r1"), pick_msg("r1"), 30, 1) == {} and q.held is None


def test_a_vehicle_without_the_capability_stops():
    q, ids = started()
    assert q.next(ctx(next_pick=False), 20, lambda: next(ids)) is None
    assert q.stop_reason == "차량이 자동 선택을 지원하지 않습니다"


def test_a_refusal_after_the_pick_still_stops():
    # Codex CT-2 r1: the request is tracked through the pick held until its ack
    q, ids = started()
    q.next(ctx(), 20, lambda: next(ids))
    q.on_pick(key(q, "r1"), pick_msg("r1"), 30, 1)
    q.on_answer(key(q, "r1"), {"t": "reject", "reason": "busy"}, 31)
    assert q.stop_reason == "자동 선택 거부: busy" and q.phase == "waiting" and not q.items[0].get("uncertain")


def test_an_unacked_pick_held_past_the_deadline_stops_and_is_forgotten():
    q, ids = started()
    q.next(ctx(), 20, lambda: next(ids))
    q.on_pick(key(q, "r1"), pick_msg("r1"), 30, 1)
    for t in range(1000, 20 + mq.PICK_DEADLINE_MS + 1, 1000):
        q.on_state(STAND, t)
    q.on_state(STAND, 21 + mq.PICK_DEADLINE_MS)
    q.tick(21 + mq.PICK_DEADLINE_MS)
    assert q.stop_reason == "자동 선택 응답 없음" and q.drain_forget() == ["r1"]


def test_a_pick_after_the_deadline_is_not_taken():
    q, ids = started()
    q.next(ctx(), 20, lambda: next(ids))
    q.on_pick(key(q, "r1"), pick_msg("r1"), 21 + mq.PICK_DEADLINE_MS, 1)
    assert q.stop_reason == "자동 선택 응답 없음" and q.held is None


def test_a_busy_auto_submit_keeps_what_may_have_gone_out():
    q, ids = started()
    q.next(ctx(), 20, lambda: next(ids))
    q.on_answer(key(q, "r1"), {"t": "ack"}, 21)
    q.on_pick(key(q, "r1"), pick_msg("r1"), 30, 1)
    sub = q.next(ctx(), 41, lambda: next(ids))["submit"]
    q.on_answer({"epoch": q.epoch, "item_id": "A", "request_id": sub["request_id"]}, {"t": "reject", "reason": "busy"})
    it = q.items[0]
    assert it["uncertain"] and it["uncertain_pallet_id"] == "P-1" and it["uncertain_destination_id"] == "D-1"
