"""The bring-up tool's drive conditions (docs/design/2026-11-12-bringup-host-tools.md 3) without a link."""

import sys
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import bringup  # noqa: E402


def state(flags=StateFlag(0)):
    return McuState(mcu_time_us=1, boot_id=1, arm_gen=0, last_rx_seq=0, mode=Mode.DISARMED,
                    reject_reason=RejectReason.NONE, fault_flags=FaultFlag(0), wheel_count_left=0,
                    wheel_count_right=0, wheel_rate_left_rad_s=0.0, wheel_rate_right_rad_s=0.0, steering_rad=0.25,
                    lift_m=0.0, current_ma=(0, 0, 0, 0), duty_e4=(0, 0, 0, 0), driver_status=(0, 0, 0, 0),
                    rx_errors=0, cycle_overruns=0, flags=flags)


def test_the_commissioning_build_takes_no_drive_and_locked_outputs_stop_a_real_run():
    with pytest.raises(bringup.Refused, match="commission_build"):
        bringup.check_build(state(StateFlag.COMMISSION_BUILD), allow_locked=True)
    with pytest.raises(bringup.Refused, match="outputs_locked"):
        bringup.check_build(state(StateFlag.OUTPUTS_LOCKED), allow_locked=False)
    bringup.check_build(state(StateFlag.OUTPUTS_LOCKED | StateFlag.HIL_PLANT), allow_locked=True)  # model or HIL
    bringup.check_build(state(), allow_locked=False)


def test_the_drive_holds_the_other_axes():
    d = bringup.rear(0.5, 0.0, 0.25)
    assert (d.left_wheel_rad_s, d.right_wheel_rad_s, d.steering_rad) == (0.5, 0.0, 0.25)


def test_the_cap_comes_from_the_vehicle_limit_and_the_unmeasured_radius():
    assert bringup._cap() == pytest.approx(1.0)  # radius unmeasured: 1.0 rad/s, under the 8 rad/s operating limit


def test_an_unconfirmed_operator_refuses(monkeypatch):
    import io

    monkeypatch.setattr(sys, "stdin", io.StringIO("no\n"))
    with pytest.raises(bringup.Refused):
        bringup._confirm(type("A", (), {"yes": False})(), "x")


# --- Codex impl review items without a link ---

class _Link:
    tx_enabled = True

    def __init__(self, on_cycle=None):
        self.on_cycle = on_cycle

    def service(self, host, now_ns):
        if self.on_cycle:
            self.on_cycle()
        return []


def _bench(states, on_cycle=None, **kw):
    import io

    from forklift_core.plant.bringup import Sample

    b = bringup.Bench(_Link(on_cycle), io.StringIO(), **kw)
    for i, st in enumerate(states):
        b.samples.append(Sample(i * 10**7, st))
    return b


def test_a_stale_cached_disarmed_never_confirms_the_cleanup():
    """P2-3: nothing is written and no STATE comes after the DISARM -- the cleanup is not confirmed."""
    b = _bench([state()])
    s = bringup.Session(b, 1.0)
    assert s.cleanup() is False and s.cleanup_confirmed is False


def test_a_fault_after_the_event_is_an_abort():
    """P2-5: a FAULT where STOPPING -> DISARMED was expected."""
    import dataclasses
    import time as _t

    from forklift_core.plant.bringup import Sample

    armed = dataclasses.replace(state(), mode=Mode.ARMED)
    b = _bench([armed])
    b.link.on_cycle = lambda: b.samples.append(Sample(_t.monotonic_ns(), dataclasses.replace(armed, mode=Mode.FAULT)))
    assert bringup._observe_stop(b, "tx_cut") == "unexpected_mode_fault"


def test_a_floor_run_needs_real_outputs_a_wired_estop_and_the_switch():
    for flags, reachable, allow in ((StateFlag.ESTOP_UNWIRED, True, False), (StateFlag(0), False, False),
                                    (StateFlag(0), True, True)):
        b = _bench([state(flags)], allow_locked=allow)
        s = bringup.Session(b, 1.0, floor=True, power_switch_reachable=reachable)
        with pytest.raises(bringup.Refused, match="floor"):
            s.arm(1.0)
        assert not s.arm_requested


def test_analyze_reads_the_timeline_from_a_marker(tmp_path):
    import dataclasses
    import json

    from forklift_core.plant.bringup import Sample

    rec = tmp_path / "run.jsonl"
    rows = []
    for k in range(10):
        st = dataclasses.replace(state(), mode=Mode.ARMED, mcu_time_us=k * 10_000, wheel_count_left=100 * k)
        rows.append(bringup.state_row(Sample(k * 10_000_000, st)))
    rows.append({"t": "marker", "host_ns": 95_000_000, "label": "iwdg"})
    for k in range(10, 60):
        st = dataclasses.replace(state(), boot_id=2, mode=Mode.DISARMED, mcu_time_us=(k - 10) * 10_000,
                                 reset_cause=bringup.ResetCause.IWDG)
        rows.append(bringup.state_row(Sample(k * 10_000_000 + 300_000_000, st)))
    rec.write_text("\n".join(json.dumps(r, default=str) for r in rows))
    out = bringup.analyze(rec, -1, "iwdg", {"output_response_ms": "40"})
    sw = out["software"]
    assert sw["new_boot_ms"] is not None and sw["new_boot_reset_cause"] == "IWDG"
    assert out["external"]["output_response_ms"] == {"value": 40.0, "limit": 100.0, "pass": True}
    # a value without a threshold is still kept
    assert bringup.verdict("iwdg", {"stop_time_ms": "300"}, floor=True)["stop_time_ms"] == {
        "value": 300.0, "limit": None, "pass": None}


def test_a_state_not_naming_the_disarm_never_confirms_the_cleanup():
    """Codex re-review P2-2: a late STATE received after the DISARM write still reports an older seq."""
    import dataclasses
    import time as _t

    from forklift_core.plant.bringup import Sample

    b = _bench([state()])
    s = bringup.Session(b, 1.0)
    written = {}

    def cycle():
        if 5 not in b.written:
            b.written[5] = ("DISARM", _t.monotonic_ns(), None)  # the DISARM went out as seq 5
        b.samples.append(Sample(_t.monotonic_ns(), dataclasses.replace(state(), last_rx_seq=77)))

    b.link.on_cycle = cycle
    assert s.cleanup() is False


def test_a_boot_change_while_waiting_for_the_acceptance_is_an_abort():
    """Codex re-review P2-3: the boot as the event began is the reference, and the samples since are checked."""
    import dataclasses

    from forklift_core.plant.bringup import Sample

    armed = dataclasses.replace(state(), mode=Mode.ARMED)
    b = _bench([armed])
    i_event = len(b.samples)
    b.samples.append(Sample(10**9, dataclasses.replace(armed, boot_id=8, mode=Mode.DISARMED)))
    assert bringup._observe_stop(b, "tx_cut", boot=1, start_index=i_event) == "boot_changed"


def test_a_negative_rate_is_refused():
    b = _bench([state()])
    with pytest.raises(bringup.Refused, match="positive"):
        bringup.run_scenario(b, bringup.Session(b, 1.0), "expiry", -0.8)


def test_a_state_naming_the_disarm_after_its_write_confirms_the_cleanup():
    import dataclasses
    import time as _t

    from forklift_core.plant.bringup import Sample

    b = _bench([state()])
    s = bringup.Session(b, 1.0)

    def cycle():
        if 5 not in b.written:
            b.written[5] = ("DISARM", _t.monotonic_ns(), None)
        else:
            b.samples.append(Sample(_t.monotonic_ns(), dataclasses.replace(state(), last_rx_seq=5)))

    b.link.on_cycle = cycle
    assert s.cleanup() is True


def test_a_gap_in_the_acceptance_wait_is_an_abort():
    """Codex re-review 3: 250 ms without a STATE between the event and the acceptance."""
    import dataclasses

    from forklift_core.plant.bringup import Sample

    armed = dataclasses.replace(state(), mode=Mode.ARMED)
    b = _bench([armed])  # host_ns 0
    i_event = len(b.samples)
    b.samples.append(Sample(250_000_000, dataclasses.replace(armed, mcu_time_us=250_000)))
    assert bringup._observe_stop(b, "tx_cut", boot=1, start_index=i_event) == "state_gap"


# --- ② commissioning (docs/design/2026-11-12-bringup-commission-tool.md 3) ---

def _commission_session(axis, window=None):
    import io

    b = bringup.Bench(None, io.StringIO(), commission=True)
    return bringup.CommissionSession(b, axis, window)


def _with(**kw):
    from dataclasses import replace

    return replace(state(StateFlag.COMMISSION_BUILD), **kw)


def test_only_the_driven_wheels_new_stuck_or_polarity_bit_stops_it():
    s = _commission_session(0)
    none = FaultFlag(0)
    assert s.early_stop(_with(fault_flags=FaultFlag.ENCODER_POLARITY_L), none) == "encoder_bit"
    assert s.early_stop(_with(fault_flags=FaultFlag.ENCODER_STUCK_L), none) == "encoder_bit"
    assert s.early_stop(_with(fault_flags=FaultFlag.ENCODER_POLARITY_R | FaultFlag.ENCODER_STUCK_R), none) is None
    # already on before the run: not new
    assert s.early_stop(_with(fault_flags=FaultFlag.ENCODER_POLARITY_L), FaultFlag.ENCODER_POLARITY_L) is None
    assert _commission_session(1).early_stop(_with(fault_flags=FaultFlag.ENCODER_STUCK_R), none) == "encoder_bit"


def test_the_window_stop_and_an_invalid_value():
    s = _commission_session(2, (0.2, 0.3))
    assert s.early_stop(_with(steering_rad=0.25), FaultFlag(0)) is None
    assert s.early_stop(_with(steering_rad=0.31), FaultFlag(0)) == "window"
    assert s.early_stop(_with(steering_rad=float("nan")), FaultFlag(0)) == "window"
    h = _commission_session(3, (-0.01, 0.01))
    assert h.early_stop(_with(lift_m=0.02), FaultFlag(0)) == "window"


def test_the_commission_session_refuses_a_normal_build_and_any_drive():
    s = _commission_session(0)
    with pytest.raises(bringup.Refused, match="not a commissioning build"):
        s.preflight(state())
    with pytest.raises(bringup.Refused, match="no DRIVE"):
        s.send(0.1, 0.1)


class _WriteOnlyLink:
    """Frames are written, nothing is ever received (a lost RX)."""

    tx_enabled = True

    def service(self, host, now_ns):
        while (f := host.next_frame(now_ns)) is not None:
            host.frame_written(f, now_ns)
        return []


def _sample(mode, *, age_s=0.0, boot=1, t_us=1):
    import time as _t
    from dataclasses import replace

    from forklift_core.plant.bringup import Sample

    return Sample(_t.monotonic_ns() - int(age_s * 1e9),
                  replace(state(StateFlag.COMMISSION_BUILD), mode=mode, boot_id=boot, mcu_time_us=t_us))


@pytest.mark.parametrize("mode,age_s", [(Mode.STOPPING, 0.0), (Mode.ARMED, 0.5), (Mode.ARMED, 0.06)])
def test_an_unseen_stopping_end_still_disarms_and_is_an_abort(mode, age_s, monkeypatch):
    """Codex re-review P2-1: a STOPPING seen, or an old ARMED that may hide one -- no fresh STATE out of it within
    the wait: the DISARM still goes out, and the run is aborted as stopping_end_unobserved."""
    import io

    monkeypatch.setattr(bringup, "STOPPING_WAIT_S", 0.1)
    monkeypatch.setattr(bringup, "CLEANUP_S", 0.1)
    b = bringup.Bench(_WriteOnlyLink(), io.StringIO(), commission=True)
    b.samples.append(_sample(mode, age_s=age_s))
    s = bringup.CommissionSession(b, 0, None)
    assert s.cleanup() is False
    assert s.aborted == "stopping_end_unobserved" and s.stopping_waited_ms >= 90
    assert any(k == "DISARM" for k, _t, _m in b.written.values())


def test_a_fresh_armed_state_disarms_at_once():
    import io

    b = bringup.Bench(_WriteOnlyLink(), io.StringIO(), commission=True)
    b.samples.append(_sample(Mode.ARMED))
    s = bringup.CommissionSession(b, 0, None)
    s.cleanup()
    assert s.stopping_waited_ms is None and s.aborted is None


def test_session_faults_and_new_boots_are_aborts_wherever_they_appear():
    ok = [_sample(Mode.DISARMED), _sample(Mode.ARMED), _sample(Mode.STOPPING), _sample(Mode.DISARMED)]
    assert bringup.session_abort_reason(ok, 1) is None
    assert bringup.session_abort_reason(ok + [_sample(Mode.FAULT)], 1) == "mode_fault"
    assert bringup.session_abort_reason(ok + [_sample(Mode.ESTOP)], 1) == "mode_estop"
    assert bringup.session_abort_reason(ok + [_sample(Mode.DISARMED, boot=2)], 1) == "boot_changed"


def test_a_window_already_left_sends_no_nonzero_duty(monkeypatch):
    """Codex re-review P2-4: the stop is judged before the first nonzero command."""
    s = _commission_session(2, (0.2, 0.3))
    sent = []
    monkeypatch.setattr(s.b, "cycle", lambda: None)
    monkeypatch.setattr(s, "send_duty", lambda d, end_ns=None: sent.append(d))
    monkeypatch.setattr(s, "_abort_reason", lambda: None)
    s.hold_duty(1000, 0.2, lambda: "window")
    assert sent == [] and s.stop_reason == "window"
