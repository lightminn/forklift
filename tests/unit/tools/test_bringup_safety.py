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
    assert bringup.verdict("disarm", {"stop_time_ms": "300"})["stop_time_ms"] == {"value": 300.0, "limit": None,
                                                                                 "pass": None}


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
