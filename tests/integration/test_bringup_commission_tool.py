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

    def make(*extra, commission=True, allow_locked=False, initial_seq=None):
        m = Model(mcu_model, *extra, commission=commission)
        made.append(m)
        return bringup.Bench(PtyLink(m.master), io.StringIO(), allow_locked=allow_locked, commission=True,
                             initial_seq=initial_seq)
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
    # the driven span ends where the MCU left ARMED, not at the record's end (Codex re-review P2-3)
    m = res["metrics"]
    assert m["valid"] and m["driven_ms_max"] is not None and m["driven_ms_max"] <= 300 + 60


def test_a_new_polarity_bit_of_the_driven_wheel_stops_its_duty_while_still_armed(model):
    """Codex design P2-6: test thresholds low enough for a commissioning duty to raise the bit."""
    b = model("--encoder-sign-flip", "0", "--encoder-test-thresholds", "1")
    res = bringup.execute(args(seconds=1.0), b)
    assert res["result"] == "ok" and res["stop"] == "encoder_bit", res
    names = [n for n, _t in res["metrics"]["encoder_bits_new"]]
    assert "ENCODER_POLARITY_L" in names
    first_bit = next(i for i, s in enumerate(b.samples) if s.state.fault_flags & bringup.FaultFlag.ENCODER_POLARITY_L)
    assert any(s.state.mode == Mode.ARMED and s.state.duty_e4[0] == 0 for s in b.samples[first_bit:])


# design 2026-11-25 1.2: past 0.15 or 1.5 s a wheel run is "large" and first needs --wheels-lifted
@pytest.mark.parametrize("kw,why", [({"duty": 0.2}, "wheels lifted"), ({"duty": 0.0}, "duty"),
                                    ({"seconds": 1.6}, "wheels lifted"),
                                    ({"axis": "M", "duty": 0.2, "seconds": 0.1, "window": (-1.0, 1.0)}, "duty"),
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


def test_a_link_failing_through_the_cleanup_still_writes_the_result(model):
    """Codex re-review P2-5: the I/O raises from the nonzero hold on, the cleanup's included."""
    b = model()
    real = bringup.CommissionSession.hold_duty

    def broken_on_nonzero(self, duty_e4, seconds, stop=None):
        if duty_e4 != 0:
            def boom(host, now_ns):
                raise OSError(5, "Input/output error")
            self.b.link.service = boom
        return real(self, duty_e4, seconds, stop)

    import pytest as _pytest
    mp = _pytest.MonkeyPatch()
    mp.setattr(bringup.CommissionSession, "hold_duty", broken_on_nonzero)
    try:
        res = bringup.execute(args(), b)
    finally:
        mp.undo()
    assert res["result"] == "aborted" and res["cleanup_confirmed"] is False
    rows = [json.loads(x) for x in b.out.getvalue().splitlines()]
    assert [r for r in rows if r["t"] == "result"]


# --- design ② 10: DISARM in the commissioning time limit's STOPPING (lead decision 2026-11-12) ---

def sargs(**kw):
    base = dict(cmd="commission", axis="L", duty=0.15, seconds=None, window=None, observed=None, yes=True,
                power_switch_in_hand=True, scenario="limit_disarm", limit_s=2.0, disarm_delay_ms=50.0,
                stop_grace_ms=100.0,
                output_response_ms=None, stop_time_ms=None, stop_distance_mm=None)
    return argparse.Namespace(**{**base, **kw})


def _order(b):
    return [(k, t) for _q, (k, t, _m) in sorted(b.written.items(), key=lambda kv: kv[1][1])]


@pytest.mark.parametrize("kind", ["limit_disarm", "hold_disarm"])
def test_a_disarm_in_the_limit_stopping_is_accepted_at_once(model, kind):
    """The model's limit is 300 ms and its stop_grace 100 ms: the DISARM goes out in the limit's STOPPING (at once,
    or 50 ms in), its seq is accepted from STOPPING and the MCU is DISARMED with the limit bit kept."""
    b = model("--commission-time-ms", "300", "--outputs-locked", "1", allow_locked=True)
    res = bringup.execute(sargs(scenario=kind, stop_time_ms="180", output_response_ms="unavailable"), b)
    assert res["result"] == "ok", res
    assert res["limit_reached"] and res["disarm_accepted"] and res["disarm_during_stopping"] is True
    assert res["mode_before_acceptance"] == "STOPPING" and res["natural_end_bound_mcu_us"] is not None
    assert res["mode_at_acceptance"] == "DISARMED"
    acc = next(s for s in b.samples if s.state.last_rx_seq == res["disarm_seq"])
    assert acc.state.fault_flags & bringup.FaultFlag.COMMISSION_TIME_LIMIT  # the latch survives the DISARM
    assert 250 <= res["limit_after_ms"] <= 400
    if kind == "hold_disarm":
        assert res["software"]["limit_seen_to_disarm_write_ms"] >= 45
    # order: ARM, zero COMMISSIONs, nonzero COMMISSIONs, then the scenario's DISARM, then the cleanup's
    kinds = [k for k, _t in _order(b)]
    assert kinds[0] == "ARM" and kinds.count("DISARM") >= 1 and "DRIVE" not in kinds
    first_disarm = kinds.index("DISARM")
    assert set(kinds[1:first_disarm]) == {"COMMISSION"}
    # external values only, against the general DISARM's thresholds (lead decision: provisional until S5)
    ext = res["external"]
    general = bringup.verdict("disarm", {"stop_time_ms": "180", "output_response_ms": "unavailable"})
    assert ext == general
    assert ext["output_response_ms"] == {"value": "unavailable", "limit": 100.0, "pass": None}
    assert ext["stop_time_ms"]["value"] == 180.0
    assert res["cleanup_confirmed"] and b.state.mode == Mode.DISARMED


def test_a_hold_disarm_after_the_stopping_ended_is_not_met(model):
    """The operator's stop_grace is wrong (the build's is 100 ms): a 150 ms delay lands after the natural end."""
    b = model("--commission-time-ms", "300", "--outputs-locked", "1", allow_locked=True)
    res = bringup.execute(sargs(scenario="hold_disarm", disarm_delay_ms=150.0, stop_grace_ms=400.0), b)
    assert res["result"] == "not_met" and res["hold_ended_before_disarm"], res
    assert res["disarm_during_stopping"] is False and res["cleanup_confirmed"]


def test_the_time_limit_scenarios_are_refused_on_an_open_board(model):
    """Design 2026-11-25 1.3: an open commissioning board's wheel limit is the validator's ceiling, so the scenarios
    that drive to it run on a locked board or the model only -- refused before the ARM."""
    b = model("--commission-time-ms", "300")
    res = bringup.execute(sargs(), b)
    assert res["result"] == "refused" and "locked commissioning board" in res["why"], res
    assert "ARM" not in kinds_written(b)


def test_a_limit_that_never_comes_is_an_abort(model):
    b = model("--outputs-locked", "1", allow_locked=True)  # a locked model's limit is 2000 ms; gives up at 1.9 s
    res = bringup.execute(sargs(limit_s=1.6), b)
    assert res["result"] == "aborted" and res["aborted"] == "limit_not_reached", res
    assert res["cleanup_confirmed"]


@pytest.mark.parametrize("kw,why", [({"axis": "M"}, "wheel only"), ({"limit_s": 1.5}, "limit-s"),
                                    ({"limit_s": 3.1}, "limit-s"), ({"disarm_delay_ms": -1.0}, "delay"),
                                    ({"disarm_delay_ms": 100.0}, "delay"), ({"stop_grace_ms": None}, "stop-grace"),
                                    ({"seconds": 0.5}, "seconds"),
                                    ({"power_switch_in_hand": False}, "power switch"), ({"duty": 0.2}, "duty")])
def test_scenario_limits_refuse_before_anything_is_sent(model, kw, why):
    b = model()
    res = bringup.execute(sargs(**kw), b)
    assert res["result"] == "refused" and why in res["why"], res
    assert kinds_written(b) == []


def test_the_scenario_cli_parses():
    ap_args = ["commission", "--device", "/dev/null", "--out", "/dev/null", "--axis", "R", "--duty", "0.1",
               "--scenario", "hold_disarm", "--disarm-delay-ms", "40", "--power-switch-in-hand", "--stop-time-ms",
               "unavailable", "--stop-grace-ms", "120"]
    import unittest.mock as um
    with um.patch.object(bringup, "ReconnectingLink"), um.patch.object(bringup, "execute",
                                                                       return_value={}) as ex:
        bringup.main(ap_args)
    a = ex.call_args[0][0]
    assert (a.scenario, a.limit_s, a.disarm_delay_ms, a.stop_time_ms, a.seconds, a.stop_grace_ms) == (
        "hold_disarm", 2.0, 40.0, "unavailable", None, 120.0)


def test_analyze_takes_the_commissioning_kinds_against_the_disarm_thresholds(tmp_path, monkeypatch):
    monkeypatch.setitem(bringup.WHEEL_STOP_MS, "disarm", 400.0)  # whatever the DISARM row holds, they follow
    rec = tmp_path / "run.jsonl"
    rows = [{"t": "header", "cmd": "commission", "scenario": "hold_disarm", "large": False, "bypass": None},
            {"t": "marker", "host_ns": 0, "label": "x"}, {"t": "result", "event_host_ns": 5}]
    rec.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    out = bringup.analyze(rec, -1, "hold_disarm", {"stop_time_ms": "450"})
    assert out["event_host_ns"] == 5 and out["marker"] is None  # the tool's own event, not the Enter (P2-3)
    assert out["external"]["stop_time_ms"]["reference"] == "stand"  # the commissioning scenarios run lifted
    assert out["external"]["stop_time_ms"]["limit"] == 400.0 and out["external"]["stop_time_ms"]["over_limit"]
    # a time-limit scenario runs on a locked board or the model: no physical pass (Codex PR-2b P1-3)
    assert out["external"]["stop_time_ms"]["pass"] is None and not out["official"]
    # design 2026-11-25 1.3 (PR-2b): a commissioning run is never read with --floor, nor as another kind
    assert bringup.analyze(rec, -1, "hold_disarm", {"stop_time_ms": "450"}, floor=True)["result"] == "refused"
    assert "not 'disarm'" in bringup.analyze(rec, -1, "disarm", {"stop_time_ms": "180"})["why"]


def test_an_old_record_without_its_kind_gets_no_official_pass(tmp_path):
    """A marker in a record that does not name its run kind (before PR-2b): read, its passes left empty."""
    rec = tmp_path / "run.jsonl"
    rec.write_text(json.dumps({"t": "marker", "host_ns": 0, "label": "x"}) + "\n", encoding="utf-8")
    out = bringup.analyze(rec, -1, "disarm", {"output_response_ms": "50", "stop_time_ms": "180"}, floor=True)
    assert out["result"] == "ok" and out["run_kind"] is None
    assert out["external"]["output_response_ms"]["pass"] is None and out["external"]["stop_time_ms"]["pass"] is None
    # an operator-marked event keeps the existing path
    out = bringup.analyze(rec, -1, "iwdg", {"output_response_ms": "50"})
    assert out["external"]["output_response_ms"]["pass"] is True


def test_the_cli_refuses_a_bypassed_record_unless_allowed(tmp_path, capsys):
    rec = tmp_path / "run.jsonl"
    rows = [{"t": "header", "cmd": "commission", "scenario": None, "large": True, "bypass": "enc_stuck"},
            {"t": "marker", "host_ns": 0, "label": "x"}]
    rec.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    assert bringup.main(["analyze", "--record", str(rec), "--kind", "iwdg", "--output-response-ms", "50"]) == 2
    assert "not for a gate" in json.loads(capsys.readouterr().out)["why"]
    assert bringup.main(["analyze", "--record", str(rec), "--kind", "iwdg", "--output-response-ms", "50",
                         "--allow-bypassed"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["not_for_gate"] and out["external"]["output_response_ms"]["pass"] is None


def test_analyze_judges_the_run_it_reads_not_the_whole_file(tmp_path, capsys):
    """Codex PR-2b P2-4: a bypassed commissioning run, then a plain record with a marker appended to the same file."""
    rec = tmp_path / "run.jsonl"
    rows = [{"t": "header", "cmd": "commission", "scenario": None, "large": True, "bypass": "enc_stuck"},
            {"t": "header", "cmd": "record"}, {"t": "marker", "host_ns": 0, "label": "x"}]
    rec.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    assert bringup.main(["analyze", "--record", str(rec), "--kind", "iwdg", "--output-response-ms", "50"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert "not_for_gate" not in out and out["external"]["output_response_ms"]["pass"] is True


def test_a_plain_scenario_needs_real_outputs_in_its_states():
    """Codex PR-2b re-review P2: a tool event's physical pass reads the run's own STATEs."""
    from forklift_core.mcu_link.messages import (
        FaultFlag,
        McuState,
        RejectReason,
        StateFlag,
    )
    from forklift_core.plant.bringup import Sample

    def rows(flags):
        out = [{"t": "header", "cmd": "scenario", "kind": "disarm", "floor": False}]
        if flags is not None:
            st = McuState(mcu_time_us=1, boot_id=1, arm_gen=0, last_rx_seq=0, mode=Mode.ARMED,
                          reject_reason=RejectReason.NONE, fault_flags=FaultFlag(0), wheel_count_left=0,
                          wheel_count_right=0, wheel_rate_left_rad_s=0.0, wheel_rate_right_rad_s=0.0,
                          steering_rad=0.25, lift_m=0.0, current_ma=(0, 0, 0, 0), duty_e4=(0, 0, 0, 0),
                          driver_status=(0, 0, 0, 0), rx_errors=0, cycle_overruns=0, flags=flags)
            out.append(bringup.state_row(Sample(10, st)))
        return out + [{"t": "result", "result": "ok", "cleanup_confirmed": True, "event_host_ns": 5}]

    ext = {"output_response_ms": "50"}
    for flags in (None, StateFlag.OUTPUTS_LOCKED, StateFlag.HIL_PLANT):
        out = bringup.analyze(rows(flags), -1, "disarm", ext)
        assert out["result"] == "ok" and out["external"]["output_response_ms"]["pass"] is None, flags
    assert bringup.analyze(rows(StateFlag(0)), -1, "disarm", ext)["external"]["output_response_ms"]["pass"] is True
    # tx_cut: its worst case met, or no pass (re-review 3 P2)
    for met in (False, True):
        rs = rows(StateFlag(0))
        rs[0]["kind"] = "tx_cut"
        rs[-1]["worst_case"] = {"met": met}
        assert bringup.analyze(rs, -1, "tx_cut", ext)["official"] is met
    # --floor never reads a commissioning build's STATEs (re-review 3 P3)
    rs = rows(StateFlag.COMMISSION_BUILD)
    rs[0]["floor"] = True
    assert bringup.analyze(rs, -1, "disarm", ext, floor=True)["result"] == "refused"


def test_analyze_takes_the_rehearsal_check_per_run(tmp_path, capsys):
    """Codex PR-2b re-review P2: a pty run, then a real record appended: the real run is read as such."""
    rec = tmp_path / "run.jsonl"
    rows = [{"t": "header", "cmd": "record", "device": "/dev/pts/3"},
            {"t": "header", "cmd": "record", "device": "/dev/ttyACM0"}, {"t": "marker", "host_ns": 0, "label": "x"}]
    rec.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    assert bringup.main(["analyze", "--record", str(rec), "--kind", "iwdg", "--output-response-ms", "50"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert "not_for_judgement" not in out and out["external"]["output_response_ms"]["pass"] is True
    assert bringup.main(["analyze", "--record", str(rec), "--kind", "iwdg", "--marker", "0"]) == 0  # one marker


def test_a_diagnostic_image_is_not_for_a_gate(tmp_path, capsys):
    """Codex PR-2b P2-5: the 2026-11-24 image named in the record, whatever the header says."""
    rec = tmp_path / "run.jsonl"
    rows = [{"t": "header", "cmd": "record"},
            {"t": "connected", "product": "forklift-mcu commission OPEN diag-commission-unlimited-1 rec=1f4207f4"},
            {"t": "marker", "host_ns": 0, "label": "x"}]
    rec.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    assert bringup.main(["analyze", "--record", str(rec), "--kind", "iwdg", "--output-response-ms", "50"]) == 2
    assert "diagnostic image" in json.loads(capsys.readouterr().out)["why"]
    res = {"external": {"output_response_ms": {"pass": True}}}
    bringup._mark_commission_result(res, (), False, "forklift-mcu commission OPEN diag-commission-unlimited-1 rec=1")
    assert res["not_for_gate"] and res["external"]["output_response_ms"]["pass"] is None


@pytest.mark.parametrize("what", ["fault", "boot"])
def test_a_fault_or_reboot_first_seen_in_the_scenario_cleanup_is_an_abort(model, monkeypatch, what):
    """Codex scenario review P2-2: the session-wide check runs after the cleanup."""
    from dataclasses import replace

    from forklift_core.plant.bringup import Sample

    b = model("--commission-time-ms", "300", "--outputs-locked", "1", allow_locked=True)
    real = bringup.CommissionSession.cleanup

    def cleanup_then_bad_state(self):
        ok = real(self)
        last = self.b.samples[-1]
        bad = replace(last.state, mode=Mode.FAULT) if what == "fault" else replace(last.state, boot_id=0x0BAD)
        self.b.samples.append(Sample(last.host_ns + 1, bad))
        return ok

    monkeypatch.setattr(bringup.CommissionSession, "cleanup", cleanup_then_bad_state)
    res = bringup.execute(sargs(), b)
    assert res["result"] == "aborted" and res["aborted"] == ("mode_fault" if what == "fault" else "boot_changed")


def test_a_scenario_with_an_invalid_external_value_sends_nothing(model):
    b = model()
    res = bringup.execute(sargs(output_response_ms="-inf"), b)
    assert res["result"] == "refused" and "output_response_ms" in res["why"]
    assert kinds_written(b) == []


# --- markers while driving (arrival-day procedure, firmware session request 2026-11-12) ---------------------------

def test_markers_mark_events_while_the_commission_drives(model, monkeypatch, tmp_path):
    """A main power cut or an NRST reset happens while the wheel turns: Enter marks it in the same process's record,
    and analyze reads the timeline from it."""
    calls = {"n": 0}

    def reader(bench):
        marked = {"done": False}

        def poll():
            # cheap on every cycle, like the real reader: re-parsing the whole record each poll starved the drive loop
            # under load (COMMAND_TIMEOUT, seen by the firmware session)
            calls["n"] += 1
            if marked["done"]:
                return
            if any(k == "COMMISSION" and m is not None and m.duty_e4 != 0 for k, _t, m in bench.written.values()):
                bench.emit({"t": "marker", "host_ns": bringup.time.monotonic_ns(), "label": "power"})
                marked["done"] = True
        return poll

    monkeypatch.setattr(bringup, "_marker_reader", reader)
    b = model()
    res = bringup.execute(args(markers=True), b)
    assert res["result"] == "ok", res
    marks = [r for r in rows(b) if r.get("t") == "marker"]
    assert len(marks) == 1 and calls["n"] > 0
    first_on = min(t for k, t, m in b.written.values() if k == "COMMISSION" and m.duty_e4 != 0)
    assert marks[0]["host_ns"] >= first_on  # marked while driving
    rec = tmp_path / "run.jsonl"
    rec.write_text(b.out.getvalue(), encoding="utf-8")
    out = bringup.analyze(rec, 0, "power", {})
    assert out["result"] == "ok" and out["marker"] == "power"


def rows(bench):
    import json as _json

    return [_json.loads(x) for x in bench.out.getvalue().splitlines() if x.strip()]


def test_the_marker_reader_starts_only_after_the_confirmation(model, monkeypatch):
    """The reader reads stdin: installed before the 'yes' it would take the operator's confirmation line."""
    seen = {}

    def confirm(a, what):
        seen["poll_at_confirm"] = b.poll

    monkeypatch.setattr(bringup, "_confirm", confirm)
    monkeypatch.setattr(bringup, "_marker_reader", lambda bench: (lambda: None))
    b = model()
    bringup.execute(args(markers=True, yes=False), b)
    assert seen["poll_at_confirm"] is None and b.poll is not None


def test_without_markers_nothing_reads_stdin(model, monkeypatch):
    monkeypatch.setattr(bringup, "_marker_reader", lambda bench: pytest.fail("reader installed"))
    b = model()
    assert bringup.execute(args(), b)["result"] == "ok"
    assert b.poll is None


def test_the_commission_cli_takes_markers_and_analyze_says_its_marker_count_from_zero(capsys):
    ap_args = ["commission", "--device", "/dev/null", "--out", "/dev/null", "--axis", "L", "--duty", "0.1",
               "--seconds", "1.5", "--power-switch-in-hand", "--markers"]
    import unittest.mock as um
    with um.patch.object(bringup, "ReconnectingLink"), um.patch.object(bringup, "execute",
                                                                       return_value={}) as ex:
        bringup.main(ap_args)
    assert ex.call_args[0][0].markers is True
    with pytest.raises(SystemExit):
        bringup.main(["analyze", "--help"])
    help_text = capsys.readouterr().out
    assert "0 = the first" in help_text and "-1 = the last" in help_text


def test_analyze_answers_a_marker_out_of_range(tmp_path):
    rec = tmp_path / "run.jsonl"
    rec.write_text(json.dumps({"t": "marker", "host_ns": 0, "label": "x"}) + "\n", encoding="utf-8")
    out = bringup.analyze(rec, 1, "power", {})
    assert out == {"result": "no_marker", "markers": 1, "asked": 1}


def test_the_marker_reader_never_blocks_on_a_partial_line():
    """Codex P2: on a canonical tty a label flushed with Ctrl-D is readable without a newline; a readline() would
    wait for one and stall the drive loop. The reader takes the bytes there are and marks only whole lines."""
    master, slave = os.openpty()  # the slave in its default canonical mode, like a terminal
    try:
        out = io.StringIO()
        b = bringup.Bench(PtyLink(master), out, commission=True)
        poll = bringup._marker_reader(b, fd=slave)
        os.write(master, b"power\x04")  # a label, Ctrl-D: readable, no newline
        bringup.time.sleep(0.05)
        t0 = bringup.time.monotonic()
        poll()
        assert bringup.time.monotonic() - t0 < 0.02
        assert "marker" not in out.getvalue()
        os.write(master, b"\n")
        bringup.time.sleep(0.05)
        poll()
        os.write(master, b"\nnrst\n")  # an empty line (plain Enter) and a label, in one read
        bringup.time.sleep(0.05)
        poll()
        marks = [json.loads(x) for x in out.getvalue().splitlines() if '"marker"' in x]
        assert [m["label"] for m in marks] == ["power", "", "nrst"]
        t1 = bringup.time.monotonic()
        poll()  # nothing waiting: returns at once
        assert bringup.time.monotonic() - t1 < 0.02
        os.write(master, b"\x04")  # Ctrl-D on an empty line: not the end of the markers
        bringup.time.sleep(0.05)
        poll()
        os.write(master, b"again\n")
        bringup.time.sleep(0.05)
        poll()
        marks = [json.loads(x) for x in out.getvalue().splitlines() if '"marker"' in x]
        assert [m["label"] for m in marks][-1] == "again"
    finally:
        os.close(master)
        os.close(slave)


# --- design 2026-11-25-bench-commission-limits (PR-1): large wheel runs on the open commissioning image ---

LR1 = "forklift-mcu commission OPEN v0-test rec=00000000 lim=lr1"


def largs(**kw):
    base = dict(duty=0.3, seconds=2.0, wheels_lifted=True, synthetic=True, observed="none")
    return args(**{**base, **kw})


def _typed(monkeypatch, answer=None):
    """The operator's terminal line: the run's own phrase unless ``answer`` is given."""
    def line(prompt):
        return answer if answer is not None else prompt.split("Type exactly '")[1].rstrip("': ")
    monkeypatch.setattr(bringup, "_tty_line", line)


def test_acceptance_an_encoderless_wheel_runs_large_and_long_with_the_bypass(model, monkeypatch):
    """Design 4 (the lead's acceptance condition): an encoder-less L wheel driven open-loop at 0.3 for 2 s on the
    official open commissioning image, the tool's arguments only -- the stuck bit really comes on and the run goes on,
    ending with its zero and the cleanup."""
    b = model("--encoder-frozen", "0", "--encoder-test-thresholds", "1")
    b.host.usb_product = LR1
    _typed(monkeypatch)
    res = bringup.execute(largs(bypass="enc_stuck,enc_polarity"), b)
    assert res["result"] == "ok" and res["stop"] == "time", res
    assert "ENCODER_STUCK_L" in res["faults_seen"]  # the bit came on and was left out of the early stop
    assert res["large"] and res["bypass"] == ["enc_stuck", "enc_polarity"] and res["cleanup_confirmed"]
    assert max(abs(s.state.duty_e4[0]) for s in b.samples) == 3000
    assert 1.8 <= res["metrics"]["driven_ms_min"] / 1000 <= 2.1
    rows = [json.loads(x) for x in b.out.getvalue().splitlines()]
    (cond,) = [r for r in rows if r["t"] == "run_conditions"]
    assert cond["large"] and cond["bypass"] == ["enc_stuck", "enc_polarity"]
    # the product checked just before the ARM is the run's (Codex PR-1 P2-2)
    (armed,) = [r for r in rows if r["t"] == "pre_arm_checked"]
    assert [r for r in rows if r["t"] == "armed"]  # and the ARM was taken
    assert armed["product"] == LR1 and armed["lim"] and armed["large"] and armed["duty_max"] == 1.0
    assert armed["ignored_bits"] == ["ENCODER_STUCK_L", "ENCODER_POLARITY_L"] and res["board_product"] == LR1


def test_without_the_bypass_the_stuck_bit_still_stops_it(model, monkeypatch):
    b = model("--encoder-frozen", "0", "--encoder-test-thresholds", "1")
    b.host.usb_product = LR1
    _typed(monkeypatch)
    res = bringup.execute(largs(), b)
    assert res["stop"] == "encoder_bit" and res["cleanup_confirmed"], res


def test_only_the_named_kind_is_left_out(model, monkeypatch):
    """--bypass enc_polarity leaves the stuck stop in place."""
    b = model("--encoder-frozen", "0", "--encoder-test-thresholds", "1")
    b.host.usb_product = LR1
    _typed(monkeypatch)
    res = bringup.execute(largs(bypass="enc_polarity"), b)
    assert res["stop"] == "encoder_bit", res


@pytest.mark.parametrize("product,kw,answer,why", [
    (None, {}, None, "no USB product"),                                         # UART7 or a pty: nothing to read
    ("forklift-mcu commission OPEN v0 rec=00000000", {}, None, "lim=lr1"),      # today's image
    ("forklift-mcu commission OPEN v0 rec=00000000 lim=lr1x", {}, None, "lim=lr1"),  # a cut or other token
    ("FT232R USB UART", {}, None, "lim=lr1"),                                   # a USB-UART adapter's own string
    (LR1, {"wheels_lifted": False}, None, "wheels lifted"),
    (LR1, {"duty": 0.5, "synthetic": False}, None, "vcc"),
    (LR1, {}, "yes", "did not retype"),                                         # --yes-style answer is not enough
    (LR1, {"duty": 1.01}, None, "at most"),
    (LR1, {"seconds": 2147.0}, None, "at most 2146.98"),
])
def test_a_large_run_is_refused_before_any_arm(model, monkeypatch, product, kw, answer, why):
    b = model()
    b.host.usb_product = product
    _typed(monkeypatch, answer)
    res = bringup.execute(largs(**kw), b)
    assert res["result"] == "refused" and why in res["why"], res
    assert "ARM" not in kinds_written(b)
    rows = [json.loads(x) for x in b.out.getvalue().splitlines()]
    assert not [r for r in rows if r["t"] in ("pre_arm_checked", "armed")]  # a refused board has neither row


def test_the_large_confirmation_needs_a_terminal(model, monkeypatch):
    b = model()
    b.host.usb_product = LR1
    monkeypatch.setattr(sys.stdin, "isatty", lambda: False)
    res = bringup.execute(largs(), b)
    assert res["result"] == "refused" and "not a TTY" in res["why"]
    assert kinds_written(b) == []


def test_a_signed_triangle_runs_every_piece_with_its_sign(model, monkeypatch):
    """The diagnostic branch drove a negative request positive; the profile keeps --duty's sign on every piece and
    ends on its absolute deadlines."""
    b = model("--encoder-frozen", "0", "--encoder-test-thresholds", "1")
    b.host.usb_product = LR1
    _typed(monkeypatch)
    t0 = time.monotonic()
    res = bringup.execute(largs(duty=-0.3, seconds=None, profile_tri=[0.05, 0.3, 1.0, 2.0], bypass="enc_stuck"), b)
    took = time.monotonic() - t0
    assert res["result"] == "ok" and res["stop"] == "time", (res.get("aborted"), res.get("faults_seen"), res)
    p = res["profile"]
    assert p["pieces"] == 20 and p["sent_max_e4"] == -2750 and p["applied_max_e4"] < 0
    sent = [d for d in (m.duty_e4 for k, _t, m in b.written.values() if k == "COMMISSION" and m is not None)]
    assert min(sent) == -2750 and max(sent) == 0
    # every piece lasts its whole 0.1 s, the last one included (Codex PR-1 P2-1): from the first nonzero write to
    # the first zero after the last nonzero one is the profile's 2.0 s
    writes = sorted((t, m.duty_e4) for k, t, m in b.written.values() if k == "COMMISSION" and m is not None)
    first_on = next(t for t, d in writes if d != 0)
    last_on = max(t for t, d in writes if d != 0)
    zero_after = next(t for t, d in writes if d == 0 and t > last_on)
    assert 1.97 <= (zero_after - first_on) / 1e9 <= 2.1
    assert res["metrics"]["rate_mean_rad_s"] is None  # a profile is not a step
    assert took < 2.0 + 0.3 + 0.5 + 3.0  # pre-zero, profile, post-zero, cleanup: no stretching per piece


@pytest.mark.parametrize("kw,why", [
    ({"profile_tri": [0.2, 0.8, 1.0, 1.0], "duty": 0.8}, "wheel slew"),   # the first piece 0.26 in 0.1 s
    ({"profile_tri": [0.05, 0.3, 1.05, 1.0]}, "whole number"),
    ({"profile_tri": [0.05, 0.3, 1.0, float("nan")]}, "finite"),
    ({"profile_tri": [0.05, 0.3, 1.0, 1.0], "duty": 0.2}, "equal HI"),
    ({"profile_tri": [0.05, 0.3, 1.0, 1.0], "seconds": 2.0}, "no --seconds"),
    ({"profile_tri": [0.05, 0.3, 1.0, 1.0], "axis": "M", "window": (-1.0, 1.0)}, "wheel"),
    # 1.5 s of pieces + hand-over + validity is past 1.5 s: large, so it needs the large rules (Codex re-review P2-1)
    ({"profile_tri": [0.05, 0.10, 0.5, 3.0], "duty": 0.10, "wheels_lifted": False}, "wheels lifted"),
])
def test_bad_profiles_are_refused_before_any_arm(model, monkeypatch, kw, why):
    b = model()
    b.host.usb_product = LR1
    _typed(monkeypatch)
    res = bringup.execute(largs(**{"seconds": None, **kw}), b)
    assert res["result"] == "refused" and why in res["why"], res
    assert kinds_written(b) == []


def test_the_cli_gives_the_host_each_reads_own_time():
    """Codex S 7th P2: the commissioning CLI builds its link with the read-time clock, so a COMMISSION's validity is
    counted from when its STATE was read, not from the cycle's start."""
    import unittest.mock as um
    with um.patch.object(bringup, "ReconnectingLink") as rl, um.patch.object(bringup, "execute", return_value={}):
        bringup.main(["commission", "--device", "/dev/null", "--out", "/dev/null", "--axis", "L", "--duty", "0.1",
                      "--seconds", "0.5", "--power-switch-in-hand"])
    assert rl.call_args.kwargs["clock"] is time.monotonic_ns


def test_a_small_profile_is_judged_on_its_pieces_not_its_nominal_hi(model, monkeypatch):
    """Codex PR-1 P3: --duty 0.16 --profile-tri 0 0.16 0.4 1 peaks at 0.12 for 0.4 s -- a default run, no large rules."""
    b = model()
    res = bringup.execute(args(duty=0.16, seconds=None, profile_tri=[0.0, 0.16, 0.4, 1.0]), b)
    assert res["result"] == "ok" and not res["large"] and res["profile"]["sent_max_e4"] == 1200, res


def test_the_link_hands_the_host_the_read_time_not_the_cycle_start():
    """Codex PR-1 P3: with the clock the host is given when the bytes were read (a late read shortens a COMMISSION's
    validity instead of stretching its MCU deadline past the approved end)."""
    from real_mcu_link import ReconnectingLink

    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    try:
        seen = []

        class Host:
            def link_identity(self, product):
                return []

            def on_bytes(self, data, t):
                seen.append(t)
                return []

            def tick(self, t):
                return []

        clock_t = iter([5_000_000, 6_000_000, 7_000_000])
        def opener(_path, _baud):
            fd = os.dup(slave)
            os.set_blocking(fd, False)  # as open_raw opens it
            return fd

        link = ReconnectingLink("pty", opener=opener, clock=lambda: next(clock_t),
                                product_reader=lambda fd: None)
        link.tx_enabled = False
        os.write(master, bytes([1, 2, 3]))
        time.sleep(0.05)
        try:
            link.service(Host(), 1_000_000)  # the cycle started at 1 ms; the read returned at the clock's 5 ms
            assert seen == [5_000_000]
        finally:
            link.link.close()  # the dup the opener gave
    finally:
        os.close(master)
        os.close(slave)


# --- design 2026-11-25-bench-commission-limits PR-2a: the record contract and matching across the seq wrap ---

def test_a_run_across_the_seq_wrap_is_matched_by_the_write_log(model):
    """The host's seq starts at 0xFFF0 and wraps during the run: the driven span is still found and measured."""
    b = model(initial_seq=0xFFF0)
    res = bringup.execute(args(observed="pos"), b)
    assert res["result"] == "ok", res
    m = res["metrics"]
    assert m["valid"] and 400 <= m["driven_ms_min"] <= m["driven_ms_max"] <= 800
    seqs = [w.seq for w in b.writes]
    assert max(seqs) >= 0xFFF0 and min(seqs) < 0x0100  # it did wrap
    rows = [json.loads(x) for x in b.out.getvalue().splitlines()]
    written = [r for r in rows if r["t"] == "written"]
    assert [r["order"] for r in written] == list(range(len(written)))
    assert all("done_ns" in r and "conn" in r for r in written)
    com = [r for r in written if r["kind"] == "COMMISSION"]
    assert com and all(r["axis"] == 0 and r["echo_mcu_time_us"] is not None and r["valid_ms"] for r in com)
    assert all("conn" in r for r in rows if r["t"] == "state")
    # the rows alone redo the driven span's first acceptance (the record contract)
    from forklift_core.plant.write_log import accepted, seen_from_rows, writes_from_rows
    redone = accepted(seen_from_rows(rows), writes_from_rows(rows),
                      lambda x: x.kind == "COMMISSION" and x.phase == "on" and x.duty_e4 != 0)
    assert redone.index is not None and redone.ambiguous_at is None
    assert {r["phase"] for r in written} >= {"arm", "pre_zero", "on", "post_zero", "cleanup"}


def test_an_ambiguous_seq_leaves_the_run_unjudgeable(model, monkeypatch):
    """Two writes named by one seq and no STATE between them naming another: no figure is taken."""
    b = model()
    real = bringup.accepted

    def ambiguous(samples, writes, pred, start=0):
        m = real(samples, writes, pred, start)
        from forklift_core.plant.write_log import Match
        return m if m.index is None else Match(None, None, ambiguous_at=m.index)

    monkeypatch.setattr(bringup, "accepted", ambiguous)
    res = bringup.execute(args(observed="pos"), b)
    assert res["result"] == "unjudgeable" and res["metrics"]["reason"] == "seq_ambiguous", res
    assert res["verdict"] is None and not res["metrics"]["valid"]  # no sign verdict from it (Codex PR-2a P2-3)


def test_a_frame_cut_by_a_reconnect_is_not_completed_by_the_new_connection():
    """Codex PR-2a P3: half a STATE on the old fd, a disconnect, then the other half on the new fd -- neither the host
    nor the record takes it; the next whole STATE is taken and carries the new connection."""
    from forklift_core.mcu_link.messages import (
        FaultFlag,
        McuState,
        RejectReason,
        StateFlag,
        encode,
    )

    def st(t_us):
        return encode(McuState(t_us, 0xB007, 0, 0, Mode.DISARMED, RejectReason.NONE, FaultFlag(0), 0, 0, 0.0, 0.0,
                               0.0, 0.0, (0, 0, 0, 0), (0, 0, 0, 0), (0xC0,) * 4, 0, 0, StateFlag.DRIVERS_READY), 1)

    b = bringup.Bench(None, io.StringIO(), commission=True)
    b.host.link_identity(None)
    whole = st(1_000)
    b.host.on_bytes(whole[: len(whole) // 2], 1)
    b.host.disconnected(2)
    b.host.link_identity(None)
    b.host.on_bytes(whole[len(whole) // 2:], 3)
    assert b.samples == [] and b.host.state is None
    b.host.on_bytes(st(2_000), 4)
    assert len(b.samples) == 1 and b.sample_conn == [2] and b.host.state.mcu_time_us == 2_000


def test_the_bench_numbers_each_opening_and_drops_a_half_frame():
    """A USB-only reconnect: the record's decoder drops the old connection's half frame and every row after it carries
    the new connection number."""
    out = io.StringIO()
    b = bringup.Bench(None, out, commission=True)
    assert b.conn == 0
    b.host.link_identity("p")
    assert b.conn == 1
    b.host.disconnected(1)
    b.host.link_identity("p")
    assert b.conn == 2
    rows = [json.loads(x) for x in out.getvalue().splitlines()]
    assert [r["conn"] for r in rows if r["t"] == "connection"] == [1, 2]


# --- design 2026-11-25-bench-commission-limits PR-2b: duty_disarm ---

def dargs(**kw):
    base = dict(cmd="commission", axis="L", duty=0.3, seconds=1.5, window=None, observed=None, yes=True,
                power_switch_in_hand=True, scenario="duty_disarm", wheels_lifted=True, synthetic=True,
                output_response_ms="50", stop_time_ms="200", stop_distance_mm=None, limit_s=2.0, disarm_delay_ms=50.0,
                stop_grace_ms=None)
    return argparse.Namespace(**{**base, **kw})


def _with_header(b, **kw):
    hdr = {"t": "header", "cmd": "commission", "scenario": "duty_disarm", "duty": 0.3, "axis": "L", "large": False,
           "bypass": None, "synthetic": None, **kw}
    return [hdr] + [json.loads(x) for x in b.out.getvalue().splitlines()]


def test_duty_disarm_on_an_open_board_holds_then_disarms_and_judges_the_response(model, monkeypatch):
    """The model reports an open commissioning build; read as a real board (not synthetic) the response is judged,
    and analyze judges the same from the record (its event the test DISARM's write, Codex PR-2b P2-3)."""
    b = model()
    b.host.usb_product = LR1
    _typed(monkeypatch)
    res = bringup.execute(dargs(synthetic=False), b)
    assert res["result"] == "ok", res
    v = res["validity"]
    assert v["valid"] and v["physical"] and res["disarm_seq"] is not None
    assert res["external"]["output_response_ms"]["pass"] is True  # judged: valid, open board
    assert res["external"]["stop_time_ms"]["pass"] is None  # on the stand: reference only
    out = bringup.analyze(_with_header(b), -1, "duty_disarm", {"output_response_ms": "50"})
    assert out["result"] == "ok" and out["validity"]["valid"] and out["event_host_ns"] == v["disarm_done_ns"]
    assert out["external"]["output_response_ms"]["pass"] is True and out["software"]["mode_change_ms"] is not None
    # the same record from a rehearsal: refused unless asked, then never a physical pass (Codex PR-2b P1-3)
    syn = _with_header(b, synthetic={"bench": "x"})
    assert bringup.analyze(syn, -1, "duty_disarm", {"output_response_ms": "50"})["result"] == "refused"
    out = bringup.analyze(syn, -1, "duty_disarm", {"output_response_ms": "50"}, allow_synthetic=True)
    assert not out["validity"]["physical"] and out["external"]["output_response_ms"]["pass"] is None
    # the run's own end decides too: an aborted run or an unconfirmed stop (Codex PR-2b re-review P1)
    for end in ({"result": "aborted"}, {"cleanup_confirmed": False}, None):
        rows = _with_header(b)
        res_row = next(r for r in rows if r["t"] == "result")
        if end is None:
            rows.remove(res_row)
        else:
            res_row.update(end)
        out = bringup.analyze(rows, -1, "duty_disarm", {"output_response_ms": "50"})
        assert out["validity"]["valid"] and not out["official"], end
        assert out["external"]["output_response_ms"]["pass"] is None
    # read as another tool kind: refused, not a missing marker (re-review P2)
    assert bringup.analyze(_with_header(b), -1, "disarm", {})["result"] == "refused"
    # a bypassed run is refused before its event is looked for
    assert bringup.analyze(_with_header(b, bypass="enc_stuck"), -1, "duty_disarm", {})["result"] == "refused"


def test_duty_disarm_on_the_model_judges_nothing_physical(model, monkeypatch):
    b = model()
    b.host.usb_product = LR1
    _typed(monkeypatch)
    res = bringup.execute(dargs(), b)  # synthetic: the pty model
    assert res["result"] == "ok" and res["validity"]["valid"] and not res["physical"], res
    assert res["external"]["output_response_ms"]["pass"] is None
    phases = [w.phase for w in b.writes]
    assert "test_disarm" in phases and "post_zero" not in phases  # no zero hold between the drive and the DISARM
    i = phases.index("test_disarm")
    assert b.writes[i - 1].kind == "COMMISSION" and b.writes[i - 1].duty_e4 == 3000  # the DISARM cuts the duty


def test_duty_disarm_on_a_locked_board_proves_the_sequence_but_judges_nothing_physical(model, monkeypatch):
    b = model("--outputs-locked", "1", allow_locked=True)  # a locked build keeps the wheels at 0.15
    res = bringup.execute(dargs(duty=0.15, seconds=0.8), b)
    assert res["result"] == "ok" and res["validity"]["valid"] and not res["physical"], res
    assert res["external"]["output_response_ms"]["pass"] is None


def test_duty_disarm_stops_on_the_encoder_bit_unless_bypassed(model, monkeypatch):
    """Codex PR-2b P1-2: the stuck/polarity early stop of a plain run applies."""
    b = model("--encoder-frozen", "0", "--encoder-test-thresholds", "1")
    b.host.usb_product = LR1
    _typed(monkeypatch)
    res = bringup.execute(dargs(), b)
    assert res["result"] == "not_met" and res["why"] == "early_stop_encoder_bit" and res["cleanup_confirmed"], res
    assert res["external"]["output_response_ms"]["pass"] is None
    assert not any(w.phase == "test_disarm" for w in b.writes)


@pytest.mark.parametrize("duty", [0.00004, 0.0001, 0.005])
def test_duty_disarm_refuses_a_duty_that_shows_no_output(model, duty):
    """Codex PR-2b P1-1: a target that quantizes to zero (or within the tolerance of it) is refused before the ARM."""
    b = model()
    res = bringup.execute(dargs(duty=duty), b)
    assert res["result"] == "refused" and "at least" in res["why"]
    assert kinds_written(b) == []


def test_duty_disarm_needs_time_for_the_hold(model):
    b = model()
    res = bringup.execute(dargs(duty=0.1, seconds=0.6), b)  # 0.05 + 0.02 + 0.5 + 0.02 + 0.02 = 0.61 s needed
    assert res["result"] == "refused" and "at least" in res["why"]
    assert kinds_written(b) == []


def test_a_bypassed_commissioning_result_is_not_for_a_gate(model, monkeypatch):
    b = model("--encoder-frozen", "0", "--encoder-test-thresholds", "1")
    b.host.usb_product = LR1
    _typed(monkeypatch)
    res = bringup.execute(largs(bypass="enc_stuck,enc_polarity"), b)
    assert res["not_for_gate"] and res["bypassed"] == ["enc_stuck", "enc_polarity"] and res["high_limit"]
    assert res["verdict"]["duty_sign_ok"] is None and res["metrics"]["valid"] is None


def test_kth_and_first_boot_refuse_a_bypassed_record(tmp_path, capsys):
    """Design 2026-11-25 1.2 (PR-2b): the S2 judgement entry points too."""
    sys.path.insert(0, str(REPO / "tools"))
    import h743_first_boot

    rec = tmp_path / "run.jsonl"
    rec.write_text(json.dumps({"t": "header", "cmd": "commission", "large": False, "bypass": "enc_stuck"}) + "\n",
                   encoding="utf-8")
    assert bringup.main(["kth", "--record", str(rec), "--configured-m", "g704_bytes", "--configured-h", "g704_bytes",
                         "--image", "x"]) == 2
    assert "not for a gate" in json.loads(capsys.readouterr().out)["why"]
    assert h743_first_boot.main(["--record", str(rec)]) == 2
    assert "not for a gate" in json.loads(capsys.readouterr().out)["why"]
    old = tmp_path / "old.jsonl"  # a commissioning record from before the bench limits: conditions unknown
    old.write_text(json.dumps({"t": "header", "cmd": "commission"}) + "\n", encoding="utf-8")
    assert h743_first_boot.main(["--record", str(old)]) == 2
