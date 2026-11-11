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
    base = dict(cmd="commission", axis="L", duty=0.1, seconds=0.6, window=None, observed=None, yes=True)
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
    assert m["rate_mean_rad_s"] > 0 and 400 <= m["driven_ms"] <= 800
    assert res["stop"] == "time" and res["verdict"]["duty_sign_ok"] and res["verdict"]["sensor_sign_ok"]
    assert "COMMISSION" in kinds_written(b) and "DRIVE" not in kinds_written(b)
    assert res["cleanup_confirmed"] and b.state.mode == Mode.DISARMED


def test_a_reversed_encoder_is_proposed_as_a_sensor_sign_flip(model):
    b = model("--encoder-sign-flip", "1")
    res = bringup.execute(args(axis="R", observed="pos"), b)
    assert res["result"] == "ok", res
    assert res["metrics"]["measured_sign"] == "neg"
    assert res["verdict"]["duty_sign_ok"] is True and res["verdict"]["sensor_sign_ok"] is False


def test_a_steering_run_stops_at_the_window(model):
    b = model()
    b.run_until(lambda: b.state is not None, 2.0)
    s0 = b.state.steering_rad
    # the model's steering moves 0.15 rad/s at duty 0.15: a 0.03 rad window is left within about 0.2 s
    res = bringup.execute(args(axis="M", duty=0.15, seconds=1.2, window=(s0 - 0.03, s0 + 0.03)), b)
    assert res["result"] == "ok" and res["stop"] == "window", res
    assert res["metrics"]["measured_sign"] == "pos" and res["metrics"]["value"]["max"] > s0 + 0.03 - 1e-6
    assert res["metrics"]["driven_ms"] < 900


def test_steering_needs_a_window_around_the_start(model):
    b = model()
    assert bringup.execute(args(axis="M"), b)["result"] == "refused"
    b.run_until(lambda: b.state is not None, 2.0)
    s0 = b.state.steering_rad
    res = bringup.execute(args(axis="M", window=(s0 + 0.1, s0 + 0.2)), b)
    assert res["result"] == "refused" and "window" in res["why"]
    assert kinds_written(b) == []


def test_a_lift_run(model):
    b = model()
    b.run_until(lambda: b.state is not None, 2.0)
    h0 = b.state.lift_m
    res = bringup.execute(args(axis="H", duty=-0.1, seconds=0.4, window=(h0 - 0.05, h0 + 0.05), observed="neg"), b)
    assert res["result"] == "ok", res
    assert res["metrics"]["measured_sign"] == "neg" and res["verdict"]["duty_sign_ok"]


def test_the_mcu_time_limit_aborts_and_cleans_up(model):
    b = model("--commission-time-ms", "300")
    res = bringup.execute(args(seconds=1.0), b)
    assert res["result"] == "aborted" and res["aborted"] == "authorization_revoked", res
    assert "COMMISSION_TIME_LIMIT" in res["faults_seen"]
    assert res["cleanup_confirmed"]


@pytest.mark.parametrize("kw,why", [({"duty": 0.2}, "duty"), ({"duty": 0.0}, "duty"), ({"seconds": 1.6}, "seconds"),
                                    ({"seconds": 0.0}, "seconds"), ({"window": (0.0, 1.0)}, "window")])
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
