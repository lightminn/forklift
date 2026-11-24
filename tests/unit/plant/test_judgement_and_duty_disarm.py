"""Judgement eligibility and the duty_disarm validity (docs/design/2026-11-25-bench-commission-limits.md 1.2-1.3,
PR-2b)."""

import pytest

from forklift_core.plant import duty_disarm as dd
from forklift_core.plant import judgement
from forklift_core.plant.write_log import Write

MS = 1_000_000


def hdr(**kw):
    return {"t": "header", "cmd": "commission", **kw}


def test_run_marks():
    assert judgement.run_marks([{"t": "header", "cmd": "record"}]) == {}
    assert judgement.run_marks([hdr()]) == {"conditions_unknown": True}  # a commissioning run from before PR-1
    assert judgement.run_marks([hdr(large=False, bypass=None)]) == {}
    assert judgement.run_marks([hdr(large=True, bypass=None)]) == {"high_limit": True}
    m = judgement.run_marks([hdr(large=True, bypass="enc_stuck,enc_polarity")])
    assert m == {"high_limit": True, "not_for_gate": True, "bypassed": ["enc_stuck", "enc_polarity"]}


def test_record_marks_and_check():
    rows = [hdr(large=False, bypass=None), {"t": "state"}, hdr(large=False, bypass="enc_stuck")]
    m = judgement.record_marks(rows)
    assert m["not_for_gate"] and m["bypassed"] == ["enc_stuck"] and not judgement.eligible(m)
    with pytest.raises(judgement.NotForGate, match="bypassed"):
        judgement.check(m, allow=False)
    assert judgement.check(m, allow=True) == m
    with pytest.raises(judgement.NotForGate, match="unknown"):
        judgement.check(judgement.record_marks([hdr()]), allow=False)
    assert judgement.check(judgement.record_marks([hdr(large=True, bypass=None)]), allow=False) == {"high_limit": True}


def test_empty_pass_fields():
    ext = {"output_response_ms": {"value": 50.0, "limit": 100.0, "pass": True}, "other": 3}
    assert judgement.empty_pass_fields(ext) == {"output_response_ms": {"value": 50.0, "limit": 100.0, "pass": None},
                                                "other": 3}


# --- duty_disarm ---

OPEN = ["COMMISSION_BUILD", "ESTOP_UNWIRED", "DRIVERS_READY"]


def st(t_ms, duty, *, seq, mode="ARMED", gen=1, faults=(), flags=OPEN, boot=7):
    return {"t": "state", "host_ns": t_ms * MS, "conn": 1, "boot_id": boot, "mcu_time_us": t_ms * 1000,
            "last_rx_seq": seq, "arm_gen": gen, "mode": mode, "duty_e4": [duty, 0, 0, 0], "fault_flags": list(faults),
            "flags": list(flags)}


def disarm_write(seq, done_ms):
    return Write(order=99, conn=1, kind="DISARM", seq=seq, start_ns=done_ms * MS - 10, done_ns=done_ms * MS,
                 phase="test_disarm")


def run(target=3000, *, at_target_until=900, accept_ms=905, mode_after="DISARMED", faults_at=None, flags=OPEN):
    """The wheel ramps, reaches the target at 100 ms, holds; the DISARM (seq 50) is written at 902 ms."""
    rows = []
    for t in range(0, 1000, 10):
        duty = min(target, t * 30) if t <= at_target_until else 0
        if t >= accept_ms:
            rows.append(st(t, 0, seq=50, mode=mode_after, flags=flags))
        else:
            rows.append(st(t, duty, seq=40 + (t // 100), faults=("COMMAND_TIMEOUT",) if faults_at == t else (),
                           flags=flags))
    return rows, [disarm_write(50, 902)]


def test_a_valid_duty_disarm():
    rows, writes = run()
    v = dd.validity(rows, writes, axis=0, duty_e4=3000)
    assert v["valid"] and v["physical"] and v["disarm_seq"] == 50 and v["disarm_done_ns"] == 902 * MS


@pytest.mark.parametrize("kw,asked,reason", [
    ({}, 6000, "target_not_reached"),                                  # the wheel never got to the asked duty
    ({"at_target_until": 500}, 3000, "hold_not_seen"),                 # 400 ms at the target, then down
    ({"at_target_until": 800}, 3000, "not_at_target_before_disarm"),   # held, then down before the DISARM
    ({"faults_at": 600}, 3000, "stopped_otherwise:COMMAND_TIMEOUT"),
    ({"accept_ms": 2000}, 3000, "disarm_not_accepted"),
])
def test_invalid_duty_disarms(kw, asked, reason):
    rows, writes = run(**kw)
    v = dd.validity(rows, writes, axis=0, duty_e4=asked)
    assert not v["valid"] and v["reason"] == reason


def test_a_locked_or_model_board_is_not_physical():
    rows, writes = run(flags=OPEN + ["OUTPUTS_LOCKED"])
    v = dd.validity(rows, writes, axis=0, duty_e4=3000)
    assert v["valid"] and not v["physical"]
    rows, writes = run(flags=["DRIVERS_READY"])  # a normal build
    assert not dd.validity(rows, writes, axis=0, duty_e4=3000)["physical"]


def test_min_seconds():
    assert dd.min_seconds(0.15) == pytest.approx(0.075 + 0.02 + 0.5 + 0.02 + 0.02)
    assert dd.min_seconds(-1.0) == pytest.approx(0.5 + 0.06 + 0.5)
