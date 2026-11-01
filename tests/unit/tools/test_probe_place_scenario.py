"""F1 (docs/plans/2026-10-31-operator-placed-start.md 4.2 b): the probe as the operator -- while the candidates are shown
it places a click drawn from the click model around the truth (at most the session's placements), confirms the first
preview, and once localized stands the hold and ends the session; what it sent and got is in the report."""

import asyncio
import math
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import f1_place_eval as F  # noqa: E402
import live_console_probe as lcp  # noqa: E402

TRUTH = (2.0, 1.0, 0.5)


class Vehicle:
    """The console state the probe reads, changed by its requests as the vehicle would: ``outcomes`` says what each
    placement ends in (``preview`` or a refusal reason)."""

    def __init__(self, outcomes):
        self.outcomes = list(outcomes)
        self.reloc = {"state": "awaiting_pick", "generation": 1, "places": 0, "max_places": 3, "preview_id": None,
                      "candidates": [{"index": 0}]}
        self.pose = None

    def state(self):
        return {"pose": self.pose, "slam": {} if self.pose else {"reloc": dict(self.reloc)}}

    def handle(self, kind, fields):
        r = self.reloc
        if kind == "relocalize_place":
            if r["state"] not in ("awaiting_pick", "reloc_failed", "place_preview") or r["places"] >= 3:
                return {"t": "reject", "reason": "reloc_not_awaiting"}
            r["places"] += 1
            r["placed"] = fields["pose"]
            out = self.outcomes.pop(0)
            if out == "preview":
                r.update(state="place_preview", preview_id=r["places"])
            else:
                r.update(state="awaiting_pick", place_reason=out, preview_id=None)
            return {"t": "ack", "places": r["places"]}
        if kind == "relocalize_place_confirm":
            if r["state"] != "place_preview" or fields["preview_id"] != r["preview_id"]:
                return {"t": "reject", "reason": "reloc_stale_pick"}
            self.pose = {"x_m": 0.0}
            return {"t": "ack"}
        return {"t": "ack", "lease_id": "L"} if kind == "lease_request" else {"t": "ack"}


class StubProbe:
    def __init__(self, vehicle):
        self.v, self.report, self.calls, self.closed_code, self.lease_id = vehicle, {}, [], None, None
        self.epoch = 1

    @property
    def state(self):
        return self.v.state()

    async def open(self):
        self.calls.append(("open", {}))

    async def until(self, pred, timeout, hz=20, label=""):
        assert pred(self.state)
        return self.state

    async def request(self, kind, timeout=5.0, **fields):
        self.calls.append((kind, fields))
        return self.v.handle(kind, fields)

    async def request_until_standing(self, kind, tries=20, **fields):
        self.calls.append((kind, fields))
        self.closed_code = 1000
        return {"t": "ack"}


def _run(p, **kw):
    args = SimpleNamespace(**{"hold_sim_s": 0.1, "live_factor": 1.0, "truth": TRUTH, "click_seed": 5,
                              "click_index": 7, "place_wait_s": 5.0, "click_kind": None, **kw})
    orig = asyncio.sleep

    async def sleep(s):
        await orig(min(s, 0.01))

    lcp.asyncio.sleep = sleep
    try:
        asyncio.run(lcp.place_scenario(p, args))
    finally:
        lcp.asyncio.sleep = orig


def test_the_operator_places_from_the_click_model_and_confirms_the_first_preview():
    p = StubProbe(Vehicle(["ambiguous", "preview"]))
    _run(p)
    sent = [(k, f) for k, f in p.calls if k.startswith("relocalize")]
    assert [k for k, _ in sent] == ["relocalize_place", "relocalize_place", "relocalize_place_confirm"]
    for k, (_, f) in enumerate(sent[:2]):  # the k-th placement's click: the model's draw for this start and k
        assert f["pose"] == list(F.click(5, 7, k, TRUTH)) and f["generation"] == 1
    assert sent[2][1] == {"generation": 1, "preview_id": 2}
    assert [x["reply"] for x in p.report["places"]] == ["ack", "ack"] and p.report["confirm"]["reply"] == "ack"
    assert p.report["hold"]["closed_during"] is None and p.report["session_end"] == {"t": "ack"}


def test_three_refusals_leave_the_session_to_the_operators_time():
    p = StubProbe(Vehicle(["edge", "low_fit", "ambiguous"]))
    _run(p, place_wait_s=0.3)
    assert sum(k == "relocalize_place" for k, _ in p.calls) == 3
    assert "confirm" not in p.report and "hold" not in p.report and p.report["localized"] is False


def test_a_click_is_inside_the_model():
    for k in range(3):
        c = F.click(5, 7, k, TRUTH)
        assert math.hypot(c[0] - TRUTH[0], c[1] - TRUTH[1]) <= F.CLICK_M


def test_the_place_scenario_connects_while_the_start_is_still_unknown(tmp_path):
    import json

    port = tmp_path / "port.json"
    port.write_text(json.dumps({"port": 1, "state": "awaiting_pick"}))
    info, why = lcp.wait_port(port, 0.05, sleep=lambda s: None)
    assert info is None  # the default waits for a running session
    info, why = lcp.wait_port(port, 0.05, sleep=lambda s: None, states=lcp.RELOC_PORT_STATES)
    assert info["state"] == "awaiting_pick" and why is None


def test_a_click_off_the_map_uses_one_of_the_operators_tries_as_offline():
    class OffMap(Vehicle):
        def handle(self, kind, fields):
            if kind == "relocalize_place" and not self.__dict__.get("refused"):
                self.refused = True
                return {"t": "reject", "reason": "reloc_place_invalid"}  # the vehicle does not count it
            return super().handle(kind, fields)

    p = StubProbe(OffMap(["preview"]))
    _run(p)
    sent = [f for k, f in p.calls if k == "relocalize_place"]
    assert [x["pose"] for x in sent] == [list(F.click(5, 7, 0, TRUTH)), list(F.click(5, 7, 1, TRUTH))]
    assert [x["reason"] for x in p.report["places"]] == ["reloc_place_invalid", None]


class HandoverProbe(StubProbe):
    """The real hand-over: the confirmation's estimate goes to the bridge, the early console closes every connection
    with 1012 "session ready" once the live runtime adopts it, and the pose shows only on the new connection."""

    async def request(self, kind, timeout=5.0, **fields):
        reply = await super().request(kind, timeout, **fields)
        if kind == "relocalize_place_confirm" and reply.get("t") == "ack":
            self.v.pose, self.ready = None, True
            self.closed_code = 1012
        return reply

    async def open(self):
        self.calls.append(("open", {}))
        if getattr(self, "ready", False):
            self.v.pose, self.closed_code = {"x_m": 0.0}, None


def test_the_operator_reconnects_when_the_live_runtime_adopts_the_console_and_then_holds():
    # F1 Isaac development (ws1 13450): the probe stopped at the 1012 hand-over, so no session end was sent and the
    # launcher had to be ended -- the page reconnects after any close but a session end, and so does the probe
    p = HandoverProbe(Vehicle(["preview"]))
    _run(p)
    assert [k for k, _ in p.calls].count("open") == 2
    assert p.report["handover"]["code"] == 1012 and p.report["localized"] is True
    assert p.report["hold"]["closed_during"] is None and p.report["session_end"] == {"t": "ack"}


def test_a_close_that_is_no_hand_over_is_not_reconnected():
    p = HandoverProbe(Vehicle(["edge", "low_fit", "ambiguous"]))
    p.closed_code = 1001  # the session went away: the probe does not open again
    _run(p, place_wait_s=0.3)
    assert [k for k, _ in p.calls].count("open") == 1 and "handover" not in p.report


def test_a_named_record_click_is_placed_instead_of_the_click_model():
    p = StubProbe(Vehicle(["edge", "preview"]))
    _run(p, click_kind="bay+y")
    poses = [f["pose"] for k, f in p.calls if k == "relocalize_place"]
    assert poses == [list(F.stress_click(5, 7, "bay+y", TRUTH))] * 2
    assert p.report["click_kind"] == "bay+y"


class EpochProbe(HandoverProbe):
    """The reconnect lands on another session (a new epoch on the same port): not this session's to hold or end."""

    async def open(self):
        await super().open()
        self.epoch = 2 if getattr(self, "ready", False) else 1


def test_a_reconnect_to_another_session_neither_holds_nor_ends_it():
    # Codex F1 development review P3: the hand-over keeps the session; a new epoch is another session
    p = EpochProbe(Vehicle(["preview"]))
    _run(p)
    assert p.report["handover"]["epoch_changed"] == [1, 2]
    assert "hold" not in p.report and "session_end" not in p.report and p.report["localized"] is False
