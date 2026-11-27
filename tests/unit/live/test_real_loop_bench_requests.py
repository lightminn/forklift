"""Bench console plan 4 (a'): the REAL / HIL loop (its arbiter without bench_requests), operating or not, given the
four bench requests over its console channels -- with and without a request token -- answers none of them and writes
nothing to the MCU because of them: the frames it writes in the window after them are, field for field, those it
writes in an equal window without them (the times and seqs a frame carries aside)."""

from dataclasses import asdict, is_dataclass

import pytest

from forklift_core.safety.arbiter import BENCH_LEASED

from real_loop_rig import EPOCH, Rig

_VOLATILE = {"echo_mcu_time_us", "boot_id", "valid_after_echo_ms"}


def frames(r, since):
    out = []
    for _, kind, _, m in r.transport.sent[since:]:
        fields = asdict(m) if is_dataclass(m) else {"repr": repr(m)}
        out.append((kind, type(m).__name__, {k: v for k, v in fields.items() if k not in _VOLATILE}))
    return out


@pytest.mark.parametrize("operating", [False, True])
@pytest.mark.parametrize("with_token", [True, False])
def test_bench_requests_on_the_real_loop_write_nothing(operating, with_token):
    r = Rig(frames=True, real_requests=True, operating=operating)
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
    assert baseline == []  # the idle loop writes nothing in this window: the claim is "nothing at all"
    assert after == baseline, (after, baseline)  # no frame caused by them, a DISARM included
    answered = [m for m in r.console_out if m.get("request_id") in sent_ids]
    assert answered == []  # not reliable requests here: never answered, nothing run
