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


def on_write(duty, *, seq=40, done_ms=5, order=1):
    return Write(order=order, conn=1, kind="COMMISSION", seq=seq, start_ns=done_ms * MS - 10, done_ns=done_ms * MS,
                 boot_id=7, echo_mcu_time_us=0, valid_ms=None, arm_gen=1, axis=0, duty_e4=duty, phase="on")


def run(target=3000, *, asked=None, at_target_until=900, accept_ms=905, mode_after="DISARMED", faults_at=None,
        flags=OPEN):
    """The drive (seq 40) is accepted at 10 ms; the wheel ramps, reaches the target at 100 ms and holds; the DISARM
    (seq 50) is written at 902 ms."""
    rows = []
    for t in range(0, 1000, 10):
        duty = min(target, t * 30) if t <= at_target_until else 0
        if t >= accept_ms:
            rows.append(st(t, 0, seq=50, mode=mode_after, flags=flags))
        else:
            rows.append(st(t, duty, seq=39 if t == 0 else 40 + (t // 100),
                           faults=("COMMAND_TIMEOUT",) if faults_at == t else (), flags=flags))
    return rows, [on_write(target if asked is None else asked), disarm_write(50, 902)]


def test_a_valid_duty_disarm():
    rows, writes = run()
    v = dd.validity(rows, writes, axis=0, duty_e4=3000)
    assert v["valid"] and v["physical"] and v["disarm_seq"] == 50 and v["disarm_done_ns"] == 902 * MS
    assert v["reach_index"] == 10


@pytest.mark.parametrize("kw,asked,reason", [
    ({}, 6000, "target_not_reached"),                                  # the wheel never got to the asked duty
    ({"at_target_until": 500}, 3000, "hold_not_seen"),                 # 400 ms at the target, then down
    ({"at_target_until": 800}, 3000, "not_at_target_before_disarm"),   # held, then down before the DISARM
    ({"faults_at": 600}, 3000, "stopped_otherwise:COMMAND_TIMEOUT"),
    ({"accept_ms": 2000}, 3000, "disarm_not_accepted"),
])
def test_invalid_duty_disarms(kw, asked, reason):
    rows, writes = run(asked=asked, **kw)
    v = dd.validity(rows, writes, axis=0, duty_e4=asked)
    assert not v["valid"] and v["reason"] == reason


def test_a_target_that_quantizes_to_nothing_or_a_zero_output_never_passes():
    """Codex PR-2b P1-1: a duty under one e4 step, or a hold counted from zero outputs, is not a stop test."""
    rows, writes = run(target=0)
    assert dd.validity(rows, writes, axis=0, duty_e4=0)["reason"] == "target_too_small"
    rows, writes = run(target=50)
    assert dd.validity(rows, writes, axis=0, duty_e4=50)["reason"] == "target_too_small"


def test_the_hold_counts_only_after_the_drive_was_accepted():
    rows, writes = run()
    writes[0] = on_write(3000, seq=45)  # the drive accepted at 500 ms: 400 ms of hold left before the DISARM
    assert dd.validity(rows, writes, axis=0, duty_e4=3000)["reason"] == "hold_not_seen"
    rows, writes = run()
    assert dd.validity(rows, writes[1:], axis=0, duty_e4=3000)["reason"] == "drive_not_accepted"


def test_continuity_from_the_reach_to_the_acceptance():
    """Codex PR-2b P2-1."""
    rows, writes = run()
    del rows[40:45]  # 60 ms without a STATE inside the hold
    assert dd.validity(rows, writes, axis=0, duty_e4=3000)["reason"] == "state_gap"
    rows, writes = run()
    for r in rows[41:]:
        r["mcu_time_us"] += 60_000  # read on time on the host, but the MCU's own clock skipped 70 ms
    assert dd.validity(rows, writes, axis=0, duty_e4=3000)["reason"] == "state_gap"
    rows, writes = run()
    rows[60]["conn"] = 2
    assert dd.validity(rows, writes, axis=0, duty_e4=3000)["reason"] == "connection_changed"
    rows, writes = run()
    rows[70]["duty_e4"][0] = 2000  # one sample off the target after the hold was seen
    assert dd.validity(rows, writes, axis=0, duty_e4=3000)["reason"] == "left_the_target_before_disarm"
    rows, writes = run()
    other = Write(order=50, conn=1, kind="COMMISSION", seq=48, start_ns=0, done_ns=795 * MS, boot_id=7,
                  echo_mcu_time_us=0, valid_ms=None, arm_gen=1, axis=0, duty_e4=0, phase="on")
    assert dd.validity(rows, writes + [other], axis=0, duty_e4=3000)["reason"] == "another_command_before_disarm"
    rows, writes = run()
    writes[1] = disarm_write(50, 550)  # written before the hold had been seen (its acceptance is still at 905)
    assert dd.validity(rows, writes, axis=0, duty_e4=3000)["reason"] == "disarm_before_the_hold_was_seen"


def test_the_acceptance_state_shows_the_stop():
    rows, writes = run(mode_after="ARMED")  # the DISARM's seq reported, still ARMED
    assert dd.validity(rows, writes, axis=0, duty_e4=3000)["reason"] == "not_stopped_at_acceptance"


def test_a_locked_or_model_board_is_not_physical():
    rows, writes = run(flags=OPEN + ["OUTPUTS_LOCKED"])
    v = dd.validity(rows, writes, axis=0, duty_e4=3000)
    assert v["valid"] and not v["physical"]
    rows, writes = run(flags=["DRIVERS_READY"])  # a normal build
    assert not dd.validity(rows, writes, axis=0, duty_e4=3000)["physical"]
    rows, writes = run()
    assert not dd.validity(rows, writes, axis=0, duty_e4=3000, synthetic=True)["physical"]  # the pty model


def test_min_seconds():
    assert dd.min_seconds(0.15) == pytest.approx(0.075 + 0.02 + 0.5 + 0.02 + 0.02)
    assert dd.min_seconds(-1.0) == pytest.approx(0.5 + 0.06 + 0.5)


def test_absent_parts_are_marked_for_every_command():
    """Partial-parts plan v4: the header's parts_absent is read for every command, before the commissioning-only
    marks; a missing lift KTH alone stays gate-eligible (plan 3.5), steer or wheels do not."""
    assert judgement.run_marks([{"t": "header", "cmd": "record", "parts_absent": []}]) == {}
    assert judgement.run_marks([{"t": "header", "cmd": "record", "parts_absent": ["lift"]}]) == {
        "parts_absent": ["lift"]}
    m = judgement.run_marks([{"t": "header", "cmd": "record", "parts_absent": ["wheels", "lift"]}])
    assert m == {"parts_absent": ["lift", "wheels"], "not_for_gate": True}
    m = judgement.run_marks([hdr(large=False, bypass=None, parts_absent=["steer"])])
    assert m == {"parts_absent": ["steer"], "not_for_gate": True}
    rows = [{"t": "header", "cmd": "record", "parts_absent": ["lift"]},
            {"t": "header", "cmd": "step", "parts_absent": ["steer"]}]
    m = judgement.record_marks(rows)
    assert m["parts_absent"] == ["lift", "steer"] and not judgement.eligible(m)
    with pytest.raises(judgement.NotForGate, match="parts were absent"):
        judgement.check(m, allow=False)
    assert judgement.parts_absent({"parts_absent": "steer"}) == []  # not a list: not read as a declaration
