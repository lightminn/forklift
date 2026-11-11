"""S3 commissioning tool ② against the firmware core built as a commissioning build on the fake board (the P0-7 MCU
model, pty; docs/design/2026-11-12-bringup-commission-tool.md 7). It proves the framing, the build gate, arming with
a zero COMMISSION, the per-axis run, the early stops, the MCU time limit's abort and the cleanup -- not currents,
directions or speeds of a real motor (the model's plant is synthetic)."""

import argparse
import io
import json
import os
import signal
import subprocess
import sys
import time
import tty
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import Mode

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "integration"))
import bringup  # noqa: E402
from test_bringup_tool import PtyLink, mcu_model  # noqa: E402,F401 -- the module-scoped model build


class Model:
    def __init__(self, exe, *extra, commission=True):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        argv = [str(exe), "--link", os.ttyname(self.slave)] + (["--commission-build", "1"] if commission else [])
        self.proc = subprocess.Popen(argv + list(extra))

    def close(self):
        self.proc.send_signal(signal.SIGTERM)
        self.proc.wait(timeout=5)
        os.close(self.master)
        os.close(self.slave)


@pytest.fixture
def model(mcu_model):  # noqa: F811
    made = []

    def make(*extra, commission=True, allow_locked=False):
        m = Model(mcu_model, *extra, commission=commission)
        made.append(m)
        return bringup.Bench(PtyLink(m.master), io.StringIO(), allow_locked=allow_locked, commission=True)
    yield make
    for m in made:
        m.close()


def args(**kw):
    base = dict(cmd="commission", axis="L", duty=0.1, seconds=0.6, window=None, observed=None, yes=True,
                power_switch_in_hand=True)
    return argparse.Namespace(**{**base, **kw})


def kinds_written(b):
    return [k for k, _t, _m in b.written.values()]


def test_a_normal_build_gets_nothing(model):
    b = model(commission=False)
    res = bringup.execute(args(), b)
    assert res["result"] == "refused" and "not a commissioning build" in res["why"]
    assert kinds_written(b) == []


def test_a_wheel_run_arms_with_a_zero_commission_drives_and_disarms(model):
    b = model()
    res = bringup.execute(args(observed="pos"), b)
    assert res["result"] == "ok", res
    m = res["metrics"]
    assert m["valid"] and m["measured_sign"] == "pos" and m["duty_sign_in_build"] == 1
    assert m["rate_mean_rad_s"] > 0 and 400 <= m["driven_ms_min"] <= m["driven_ms_max"] <= 800
    assert res["stop"] == "time" and res["verdict"]["duty_sign_ok"] and res["verdict"]["sensor_sign_ok"]
    assert "COMMISSION" in kinds_written(b) and "DRIVE" not in kinds_written(b)
    assert res["cleanup_confirmed"] and b.state.mode == Mode.DISARMED


def test_a_reversed_encoder_is_proposed_as_a_sensor_sign_flip(model):
    b = model("--encoder-sign-flip", "1")
    res = bringup.execute(args(axis="R", observed="pos"), b)
    assert res["result"] == "ok", res
    assert res["metrics"]["measured_sign"] == "neg"
    assert res["verdict"]["duty_sign_ok"] is True and res["verdict"]["sensor_sign_ok"] is False


@pytest.mark.parametrize("duty", [0.15, -0.15])
def test_a_steering_run_stops_at_the_window_either_way(model, duty):
    b = model()
    b.run_until(lambda: b.state is not None, 2.0)
    s0 = b.state.steering_rad
    # the model's steering moves 0.15 rad/s at duty 0.15: a 0.01 rad window is left after about 70 ms of 200
    res = bringup.execute(args(axis="M", duty=duty, seconds=0.2, window=(s0 - 0.01, s0 + 0.01)), b)
    assert res["result"] == "ok" and res["stop"] == "window", res
    v = res["metrics"]["value"]
    assert res["metrics"]["measured_sign"] == ("pos" if duty > 0 else "neg")
    assert (v["max"] > s0 + 0.01 - 1e-6) if duty > 0 else (v["min"] < s0 - 0.01 + 1e-6)
    assert res["metrics"]["driven_ms_min"] < 150


def test_a_lost_link_after_a_steering_command_stops_within_the_commission_cap(model, monkeypatch):
    """Codex design P1-2: the window stop cannot reach the MCU -- the last COMMISSION's validity (60 ms) ends it."""
    b = model()
    b.run_until(lambda: b.state is not None, 2.0)
    s0 = b.state.steering_rad
    real = bringup.CommissionSession.send_duty
    n = {"nonzero": 0}

    def cut_after_three(self, duty_e4, end_ns=None):
        seq = real(self, duty_e4, end_ns)
        if duty_e4 != 0:
            n["nonzero"] += 1
            if n["nonzero"] == 3:
                self.b.run_until(lambda: seq in self.b.written, 0.2)
                self.b.link.tx_enabled = False
        return seq

    monkeypatch.setattr(bringup.CommissionSession, "send_duty", cut_after_three)
    res = bringup.execute(args(axis="M", duty=0.15, seconds=0.2, window=(s0 - 0.5, s0 + 0.5)), b)
    last = max(t for k, t, m in b.written.values() if k == "COMMISSION" and m is not None and m.duty_e4 != 0)
    zero = next(s for s in b.samples if s.host_ns > last and s.state.duty_e4[2] == 0)
    assert (zero.host_ns - last) / 1e6 <= bringup.COMMISSION_CAP_MS + 30, res
    assert res["cleanup_confirmed"]


def test_steering_needs_a_window_around_the_start(model):
    b = model()
    assert bringup.execute(args(axis="M", seconds=0.2), b)["result"] == "refused"
    b.run_until(lambda: b.state is not None, 2.0)
    s0 = b.state.steering_rad
    res = bringup.execute(args(axis="M", seconds=0.2, window=(s0 + 0.1, s0 + 0.2)), b)
    assert res["result"] == "refused" and "window" in res["why"]
    assert kinds_written(b) == []


def test_a_lift_run(model):
    b = model()
    b.run_until(lambda: b.state is not None, 2.0)
    h0 = b.state.lift_m
    res = bringup.execute(args(axis="H", duty=-0.15, seconds=0.2, window=(h0 - 0.05, h0 + 0.05), observed="neg"), b)
    assert res["result"] == "ok", res
    assert res["metrics"]["measured_sign"] == "neg" and res["verdict"]["duty_sign_ok"]


def test_the_mcu_time_limit_aborts_and_the_cleanup_never_cuts_its_stopping(model):
    """Codex design P1-1: the firmware ends a STOPPING on DISARM (and its four-axis brake); the cleanup waits it
    out, so the MCU reaches DISARMED on its own before any DISARM is written."""
    b = model("--commission-time-ms", "300")
    res = bringup.execute(args(seconds=1.0), b)
    assert res["result"] == "aborted" and res["aborted"] == "authorization_revoked", res
    assert "COMMISSION_TIME_LIMIT" in res["faults_seen"] and res["stopping_waited_ms"] is not None
    stopping = next(s for s in b.samples if s.state.mode == Mode.STOPPING)
    disarmed = next(s for s in b.samples if s.host_ns > stopping.host_ns and s.state.mode == Mode.DISARMED)
    disarms = [t for k, t, _m in b.written.values() if k == "DISARM"]
    assert disarms and min(disarms) >= disarmed.host_ns
    assert res["cleanup_confirmed"]


def test_a_new_polarity_bit_of_the_driven_wheel_stops_its_duty_while_still_armed(model):
    """Codex design P2-6: test thresholds low enough for a commissioning duty to raise the bit."""
    b = model("--encoder-sign-flip", "0", "--encoder-test-thresholds", "1")
    res = bringup.execute(args(seconds=1.0), b)
    assert res["result"] == "ok" and res["stop"] == "encoder_bit", res
    names = [n for n, _t in res["metrics"]["encoder_bits_new"]]
    assert "ENCODER_POLARITY_L" in names
    first_bit = next(i for i, s in enumerate(b.samples) if s.state.fault_flags & bringup.FaultFlag.ENCODER_POLARITY_L)
    assert any(s.state.mode == Mode.ARMED and s.state.duty_e4[0] == 0 for s in b.samples[first_bit:])


@pytest.mark.parametrize("kw,why", [({"duty": 0.2}, "duty"), ({"duty": 0.0}, "duty"), ({"seconds": 1.6}, "seconds"),
                                    ({"seconds": 0.05}, "seconds"), ({"window": (0.0, 1.0)}, "window"),
                                    ({"axis": "M", "seconds": 0.3, "window": (-1.0, 1.0)}, "seconds"),
                                    ({"power_switch_in_hand": False}, "power switch")])
def test_tool_limits_refuse_before_anything_is_sent(model, kw, why):
    b = model()
    res = bringup.execute(args(**kw), b)
    assert res["result"] == "refused" and why in res["why"]
    assert kinds_written(b) == []


def test_locked_outputs_need_allow_locked(model):
    b = model("--outputs-locked", "1")
    assert bringup.execute(args(), b)["result"] == "refused"
    b = model("--outputs-locked", "1", allow_locked=True)
    assert bringup.execute(args(), b)["result"] == "ok"


def test_an_interrupt_while_driving_still_disarms(model, monkeypatch):
    b = model()
    real = bringup.CommissionSession.hold_duty
    calls = {"n": 0}

    def interrupt_on_the_nonzero_hold(self, duty_e4, seconds, stop=None):
        if duty_e4 != 0:
            calls["n"] += 1
            end = time.monotonic() + 0.1
            while time.monotonic() < end:
                self.send_duty(duty_e4, time.monotonic_ns() + 10**8)
                self.b.cycle()
                time.sleep(0.01)
            raise KeyboardInterrupt()
        return real(self, duty_e4, seconds, stop)

    monkeypatch.setattr(bringup.CommissionSession, "hold_duty", interrupt_on_the_nonzero_hold)
    res = bringup.execute(args(), b)
    assert calls["n"] == 1 and res["result"] == "aborted" and res["cleanup_confirmed"] is True


def test_the_record_holds_the_result(model):
    b = model()
    bringup.execute(args(observed="pos"), b)
    rows = [json.loads(x) for x in b.out.getvalue().splitlines()]
    (result,) = [r for r in rows if r["t"] == "result"]
    assert result["cmd"] == "commission" and result["axis"] == "L" and result["metrics"]["valid"]
