"""rv4 registration 1.1-1.2 (docs/plans/2026-11-10-live-riib-rv4-registration.md): the probe records every
mission_submit attempt -- a ``sent`` event before the send, an ``attempt_seq`` counting new mission_submit requests per
session epoch only, the target and the path -- and its clock identity, without changing a byte of what it sends."""

import asyncio
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))

import live_console_probe as lcp  # noqa: E402


class FakeWS:
    """Records every raw message sent; answers each request at once with the next scripted reply for its kind."""

    def __init__(self, probe, script):
        self.probe, self.script, self.sent = probe, {k: list(v) for k, v in script.items()}, []

    async def send(self, raw):
        self.sent.append(raw)
        msg = json.loads(raw)
        replies = self.script.get(msg["t"])
        if replies:
            reply = dict(replies.pop(0), request_id=msg["request_id"])
            self.probe.replies[msg["request_id"]] = (lcp.time.monotonic(), reply)


def make(script, epoch=7, scenario="cycle"):
    p = lcp.Probe(0, {})
    p.epoch, p.lease_id, p.control_gen, p.scenario_name = epoch, "L", 3, scenario
    p.ws = FakeWS(p, script)
    return p


def events(p, name):
    return [e for e in p.report.get("events", []) if e["event"] == name]


def test_sent_before_reply_with_target_epoch_seq_and_path():
    p = make({"mission_submit": [{"t": "reject", "reason": "not_stopped"}, {"t": "ack", "mission_id": "m1"}]})

    async def go():
        await p.request("mission_submit", pallet_id="P-1", destination_id="D-1", targets_revision=1)
        await p.request("mission_submit", pallet_id="P-1", destination_id="D-1", targets_revision=1)
    asyncio.run(go())
    order = [(e["event"], e.get("attempt_seq")) for e in p.report["events"]]
    assert order == [("sent", 1), ("reply", 1), ("sent", 2), ("reply", 2)]
    s1, r1 = events(p, "sent")[0], events(p, "reply")[0]
    for e in (s1, r1):
        assert (e["session_epoch"], e["pallet_id"], e["destination_id"], e["path"], e["item_id"]) == \
            (7, "P-1", "D-1", "cycle", None)
    assert r1["reason"] == "not_stopped" and events(p, "reply")[1]["t"] == "ack"


def test_attempt_seq_counts_submits_only_and_restarts_per_epoch():
    p = make({"mission_submit": [{"t": "ack"}] * 3, "lease_request": [{"t": "ack", "lease_id": "L"}],
              "stop": [{"t": "ack"}]})

    async def go():
        await p.request("mission_submit", pallet_id="P-1", destination_id="D-1")
        await p.request("lease_request")
        await p.request("stop", source="key")
        await p.request("mission_submit", pallet_id="P-2", destination_id="D-2")
        p.epoch = 8
        await p.request("mission_submit", pallet_id="P-3", destination_id="D-3")
    asyncio.run(go())
    sent = events(p, "sent")
    assert [(e["session_epoch"], e["attempt_seq"]) for e in sent] == [(7, 1), (7, 2), (8, 1)]
    assert all(e["kind"] == "mission_submit" for e in sent)  # no sent event for the other requests
    assert [e.get("attempt_seq") for e in events(p, "reply") if e["kind"] != "mission_submit"] == [None, None]


def test_timeout_keeps_the_attempt_fields():
    p = make({})

    async def go():
        await p._request("mission_submit", timeout=0.05, pallet_id="P-1", destination_id="D-1")
    asyncio.run(go())
    (t,) = events(p, "reply_timeout")
    assert (t["attempt_seq"], t["pallet_id"], t["session_epoch"]) == (1, "P-1", 7)
    assert events(p, "sent")[0]["attempt_seq"] == 1


def test_wire_bytes_unchanged():
    """The instrumentation adds nothing to the message: the bytes are the pre-rv4 construction's."""
    p = make({"mission_submit": [{"t": "ack"}]})

    async def go():
        await p.request("mission_submit", pallet_id="P-1", destination_id="D-1", targets_revision=1,
                        meta={"item_id": "I-9"})
    asyncio.run(go())
    assert p.ws.sent == ['{"t": "mission_submit", "request_id": "r1", "session_epoch": 7, "pallet_id": "P-1", '
                         '"destination_id": "D-1", "targets_revision": 1, "lease_id": "L", "control_gen": 3}']
    assert events(p, "sent")[0]["item_id"] == "I-9"


def test_resent_queue_submit_is_one_attempt_and_bytes_unchanged():
    p = make({}, scenario="queue")

    async def go():
        task = asyncio.ensure_future(p.request_resent("mission_submit", "q4", retry_s=0.02, pallet_id="P-1",
                                                      destination_id="D-1", targets_revision=1,
                                                      meta={"item_id": "I-1"}))
        await asyncio.sleep(0.07)
        p.replies["q4"] = (lcp.time.monotonic(), {"t": "ack", "request_id": "q4"})
        return await task
    reply = asyncio.run(go())
    assert reply["t"] == "ack"
    assert len(p.ws.sent) >= 2 and len(set(p.ws.sent)) == 1  # resends are the same message
    assert p.ws.sent[0] == ('{"t": "mission_submit", "request_id": "q4", "session_epoch": 7, "pallet_id": "P-1", '
                            '"destination_id": "D-1", "targets_revision": 1, "lease_id": "L", "control_gen": 3}')
    (s,) = events(p, "sent")
    (r,) = events(p, "reply")
    assert (s["attempt_seq"], s["path"], s["item_id"]) == (1, "queue", "I-1")
    assert (r["attempt_seq"], r["path"], r["item_id"]) == (1, "queue", "I-1")


def test_clock_identity_in_report(monkeypatch):
    p = lcp.Probe(0, {})
    c = p.report["clock"]
    assert set(c) == {"boot_id", "time_ns"}
    monkeypatch.setattr(lcp.os, "readlink", lambda path: (_ for _ in ()).throw(OSError("no ns")))
    assert lcp.clock_identity()["time_ns"] is None  # a failed read is recorded as missing, never as a value
