"""tools/rearm_hold_check.py (board design 9, lead decision 2026-11-12 (A)): the CI half runs the check against the
host model as an outputs-locked commissioning build; the verdict is also held to synthetic holds that were shortened,
lengthened or reset."""

import io
import os
import signal
import subprocess
import sys
import tty
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.mcu_link.messages import Mode, RejectReason

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import bringup  # noqa: E402
import rearm_hold_check as check  # noqa: E402
from test_bringup_tool import PtyLink, mcu_model  # noqa: E402,F401 -- the model fixture

GRACE_MS = 300  # the board's stop_grace_ms (board_config.c)


def _bench(mcu_model, *flags):
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    proc = subprocess.Popen([str(mcu_model), "--link", os.ttyname(slave), *flags])
    b = bringup.Bench(PtyLink(master), io.StringIO(), allow_locked=True, commission=True)
    return b, proc, (master, slave)


def _close(proc, fds):
    proc.send_signal(signal.SIGTERM)
    proc.wait(timeout=5)
    for fd in fds:
        os.close(fd)


def test_the_hold_holds_on_a_locked_commissioning_model(mcu_model):
    b, proc, fds = _bench(mcu_model, "--commission-build", "1", "--outputs-locked", "1",
                          "--commission-time-ms", "1000", "--stop-grace-ms", str(GRACE_MS))
    try:
        res = check.run(b, 0, 0.05, 1.0, GRACE_MS)
    finally:
        _close(proc, fds)
    assert res["result"] == "ok", res
    assert all(res["checks"].values())
    assert res["disarm_taken"] and res["clear_faults_written"] and res["left_disarmed"]
    assert res["arms_written"] >= 5


def test_an_unlocked_board_is_refused_before_anything_is_sent(mcu_model):
    b, proc, fds = _bench(mcu_model, "--commission-build", "1")
    try:
        res = check.run(b, 0, 0.05, 1.0, GRACE_MS)
    finally:
        _close(proc, fds)
    assert res["result"] == "refused" and "locked" in res["why"]
    assert not b.written  # not even an ARM


def _s(t_us, mode, rx=0, reject=RejectReason.NONE, errors=0):
    """A STATE received at host time = MCU time (in ns) + 1 ms."""
    return SimpleNamespace(host_ns=t_us * 1000 + 1_000_000, state=SimpleNamespace(
        mcu_time_us=t_us, mode=mode, last_rx_seq=rx, reject_reason=reject, rx_errors=errors))


def _hold(first_taken_us, *, clear_at_us=1_060_000, errors_late=0, other=None):
    """ARMED until 1.000 s, limit seen at 1.010 s (hold end in (1.300, 1.310] s); DISARM seq 10 taken at 1.030 s;
    ARMs written every 10 ms from 1.035 s; DISARMED with stopping until the first ARM is taken at
    ``first_taken_us``."""
    S, D = Mode.STOPPING, Mode.DISARMED
    samples = [_s(990_000, Mode.ARMED), _s(1_000_000, Mode.ARMED), _s(1_010_000, S, 9), _s(1_020_000, S, 9),
               _s(1_030_000, D, 10)]
    t = 1_040_000
    while t < first_taken_us:
        reject = other if (other is not None and t == 1_100_000) else RejectReason.STOPPING
        samples.append(_s(t, D, 10, reject, errors_late if t > 1_200_000 else 0))
        t += 10_000
    samples.append(_s(first_taken_us, Mode.ARMED, 99))
    samples.append(_s(first_taken_us + 10_000, Mode.ARMED, 99, RejectReason.NOT_DISARMED))  # ARMs in flight
    arm_ns = [t_ * 1000 for t_ in range(1_035_000, first_taken_us, 10_000)]
    clear_ns = None if clear_at_us is None else clear_at_us * 1000
    return check.judge(samples, 2, GRACE_MS, 10, clear_ns, arm_ns)


def test_judge_passes_a_hold_that_ends_in_its_window():
    res = _hold(1_310_000)
    assert res["result"] == "ok", res


def test_judge_fails_a_shortened_hold():
    res = _hold(1_200_000)  # taken before the earliest end: DISARM or CLEAR_FAULTS cut it
    assert res["result"] == "not_met" and not res["checks"]["not_shortened"]


def test_judge_fails_a_lengthened_hold():
    res = _hold(1_310_000 + check.slack_us(GRACE_MS) + 10_000)  # a DISARM or CLEAR_FAULTS restarted it
    assert res["result"] == "not_met" and not res["checks"]["not_lengthened"]


def test_judge_needs_a_refusal_seen_after_the_clear_faults():
    res = _hold(1_310_000, clear_at_us=1_300_000)  # nothing written after it had time to be refused
    assert res["result"] == "not_met" and not res["checks"]["refused_after_clear_faults"]
    assert not _hold(1_310_000, clear_at_us=None)["checks"]["refused_after_clear_faults"]


def test_judge_fails_on_link_errors_and_other_refusals():
    res = _hold(1_310_000, errors_late=1)  # a lost frame could pass for a refusal
    assert res["result"] == "not_met" and not res["checks"]["no_link_errors"]
    res = _hold(1_310_000, other=RejectReason.WRONG_BUILD)
    assert res["result"] == "not_met" and res["other_refusals"] == ["WRONG_BUILD"]


def test_a_hold_longer_than_stated_is_caught_on_the_model(mcu_model):
    """Real data against a wrong expectation: the model holds 300 ms, the check is told 150 ms (its minimum)."""
    b, proc, fds = _bench(mcu_model, "--commission-build", "1", "--outputs-locked", "1",
                          "--commission-time-ms", "1000", "--stop-grace-ms", str(GRACE_MS))
    try:
        res = check.run(b, 0, 0.05, 1.0, 150)
    finally:
        _close(proc, fds)
    assert res["result"] == "not_met" and not res["checks"]["not_lengthened"], res


def test_a_hold_too_short_to_judge_is_refused(tmp_path):
    assert check.main(["--device", "/nonexistent", "--out", str(tmp_path / "r.jsonl"), "--stop-grace-ms", "100"]) == 2
