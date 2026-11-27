"""Bench console plan 4 (a'): the operating REAL / HIL loop (its arbiter without bench_requests) given the four bench
requests over its console channels -- with and without a request token -- answers none of them and writes nothing to
the MCU because of them: the frames it writes in the window after them are the ones it writes in an equal window
without them."""

import pytest

from forklift_core.safety.arbiter import BENCH_LEASED

from real_loop_rig import EPOCH, Rig


def frames(r, since):
    return [(k, type(m).__name__) for _, k, _, m in r.transport.sent[since:]]


@pytest.mark.parametrize("with_token", [True, False])
def test_bench_requests_on_the_operating_loop_write_nothing(with_token):
    r = Rig(frames=True, real_requests=True)
    r.start()
    r.tick(10)
    n0 = len(r.transport.sent)
    r.tick(10)
    baseline = frames(r, n0)
    r.console_out.clear()
    n1 = len(r.transport.sent)
    sent_ids = []
    for kind in BENCH_LEASED:
        msg = {"t": kind, "request_id": f"bench-{kind}", "session_epoch": EPOCH, "lease_id": r.lease,
               "control_gen": r.arbiter.control_gen, "axis": "L", "duty": 0.5, "seconds": 1.0, "cause": "x",
               "power_switch_in_hand": True}
        if with_token and r.token is not None:
            msg.update(request_token=r.token, request_seq=900 + len(sent_ids))
        r.channels.message("c1", "op", msg)
        sent_ids.append(msg["request_id"])
    r.tick(10)
    after = frames(r, n1)
    assert after == baseline, (after, baseline)  # no frame caused by them (a DISARM included)
    answered = [m for m in r.console_out if m.get("request_id") in sent_ids]
    assert answered == []  # not reliable requests here: never answered, nothing run
