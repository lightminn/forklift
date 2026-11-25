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


def test_the_cap_comes_from_the_vehicle_limit_and_the_configured_radius():
    # 0.2 m/s on the configured (nominal, until S8) radius 0.125 m: 1.6 rad/s, under the 8 rad/s operating limit
    # (plan v1 e-stop waiver 3: the 1.0 rad/s of an unmeasured radius refused the 0.132 m/s test, 1.06 rad/s)
    assert bringup._cap() == pytest.approx(1.6)


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


def _wrote_disarm(b, seq):
    """A DISARM written through the bench's own write callback (the write log the cleanup matches with, PR-2a)."""
    import time as _t

    from forklift_core.mcu_link.messages import Disarm, encode

    b._log("written", _t.monotonic_ns(), encode(Disarm(), seq), {"kind": "DISARM", "seq": seq})


_MCU_T = {"t": 1000}


def _feed(b, **changes):
    """A STATE through the host's receive path (the host takes it as the latest valid one and the bench records it),
    its MCU time after every earlier one's."""
    import dataclasses
    import time as _t

    from forklift_core.mcu_link.messages import encode

    _MCU_T["t"] += 10_000
    st = dataclasses.replace(state(), mcu_time_us=_MCU_T["t"], **changes)
    b.host.on_bytes(encode(st, 1), _t.monotonic_ns())


def _bench(states, on_cycle=None, **kw):
    import io

    from forklift_core.plant.bringup import Sample

    b = bringup.Bench(_Link(on_cycle), io.StringIO(), **kw)
    for i, st in enumerate(states):
        b.samples.append(Sample(i * 10**7, st))
        b.sample_conn.append(b.conn)  # read on the bench's first connection
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
    b = _bench([state()])
    s = bringup.Session(b, 1.0)

    def cycle():
        if 5 not in b.written:
            _wrote_disarm(b, 5)  # the DISARM went out as seq 5
        _feed(b, last_rx_seq=77)

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
    b = _bench([state()])
    s = bringup.Session(b, 1.0)

    def cycle():
        if 5 not in b.written:
            _wrote_disarm(b, 5)
        else:
            _feed(b, last_rx_seq=5)

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


def _st(mode, t_us, rx=0):
    from dataclasses import replace

    from forklift_core.plant.bringup import Sample

    return Sample(t_us * 1000, replace(state(StateFlag.COMMISSION_BUILD), mode=mode, mcu_time_us=t_us,
                                       last_rx_seq=rx))


def test_a_disarm_inside_the_stopping_is_proved_by_the_grace_bound():
    """Codex scenario review P2-1: last ARMED at 1000 us, grace 100 ms -> the STOPPING lasts past 101000 us."""
    base = [_st(Mode.ARMED, 1000), _st(Mode.STOPPING, 11000)]
    ok = base + [_st(Mode.STOPPING, 21000), _st(Mode.DISARMED, 31000, rx=9)]
    assert bringup.disarm_acceptance(ok, 1, 9, 100)["disarm_during_stopping"] is True
    # the samples between were lost and the acceptance is reported past the bound: unproven, not assumed
    late = base + [_st(Mode.DISARMED, 150000, rx=9)]
    out = bringup.disarm_acceptance(late, 1, 9, 100)
    assert out["disarm_during_stopping"] is None and out["mode_before_acceptance"] == "STOPPING"
    # a natural DISARMED seen before the acceptance: not inside
    ended = base + [_st(Mode.DISARMED, 102000), _st(Mode.DISARMED, 112000, rx=9)]
    assert bringup.disarm_acceptance(ended, 1, 9, 100)["disarm_during_stopping"] is False
    assert bringup.disarm_acceptance(base, 1, 9, 100)["disarm_accepted"] is False


def test_the_kth_subcommand_reads_a_record(tmp_path, capsys):
    """mcu-link/3: S2's CRC variant check from a run.jsonl the record command wrote (state_row -> sample_from_row)."""
    import dataclasses
    import json

    from forklift_core.plant.bringup import Sample

    rec = tmp_path / "run.jsonl"
    rows = []
    for k in range(201):
        st = dataclasses.replace(state(), mcu_time_us=1_000_000 + k * 10_000, kth_frames=(10 * k, 10 * k),
                                 kth_crc_errors=(0, 0), kth_alt_crc_errors=(9 * k, 0))
        rows.append(bringup.state_row(Sample(k * 10_000_000, st)))
    rec.write_text("\n".join(json.dumps(r, default=str) for r in rows))
    back = bringup.sample_from_row(json.loads(rec.read_text().splitlines()[5]))
    assert back.state.kth_frames == (50, 50) and isinstance(back.state.kth_alt_crc_errors, tuple)
    assert bringup.main(["kth", "--record", str(rec), "--configured-m", "g704_bytes", "--configured-h",
                         "reflected_msb_first", "--image", "fw@1234abcd"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["axes"]["M"]["verdict"] == "configured_ok" and out["axes"]["H"]["verdict"] == "undecided"
    assert out["configured"] == {"M": "g704_bytes", "H": "reflected_msb_first"} and out["image"] == "fw@1234abcd"
    # a fitted axis without its variant: refused (partial-parts plan v4 made the variants per fitted axis)
    assert bringup.main(["kth", "--record", str(rec), "--configured-m", "g704_bytes", "--image", "x"]) == 2
    assert "--configured-h is required" in json.loads(capsys.readouterr().out)["why"]


@pytest.mark.parametrize("drop, value", [("kth_crc_errors", None), ("kth_frames", [1]), ("kth_alt_crc_errors", [0, 70000]),
                                         ("kth_crc_errors", [0.5, 0])])
def test_the_kth_subcommand_refuses_rows_without_well_formed_counters(tmp_path, capsys, drop, value):
    """Codex v3 impl P2-2: a row missing a counter must not default to zeros (a false configured_ok)."""
    import dataclasses
    import json

    from forklift_core.plant.bringup import Sample

    rec = tmp_path / "run.jsonl"
    rows = []
    for k in range(201):
        st = dataclasses.replace(state(), mcu_time_us=1_000_000 + k * 10_000, kth_frames=(10 * k, 10 * k),
                                 kth_crc_errors=(9 * k, 9 * k), kth_alt_crc_errors=(9 * k, 9 * k))
        row = bringup.state_row(Sample(k * 10_000_000, st))
        if value is None:
            del row[drop]
        else:
            row[drop] = value
        rows.append(row)
    rec.write_text("\n".join(json.dumps(r, default=str) for r in rows))
    assert bringup.main(["kth", "--record", str(rec), "--configured-m", "g704_bytes", "--configured-h",
                         "g704_bytes", "--image", "x"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["result"] == "invalid" and drop in out["why"]


def test_a_synthetic_record_makes_the_kth_result_synthetic_too(tmp_path, capsys):
    """Arrival-day rehearsal D2: read with --allow-synthetic, the marks and kth's own verdict agree (synthetic)."""
    import dataclasses
    import json

    from forklift_core.plant.bringup import Sample

    rec = tmp_path / "run.jsonl"
    rows = [{"t": "header", "schema": "bringup_record/1", "synthetic": {"bench": {"pts": "/dev/pts/9"}}}]
    for k in range(201):
        st = dataclasses.replace(state(), mcu_time_us=1_000_000 + k * 10_000, kth_frames=(10 * k, 10 * k),
                                 kth_crc_errors=(0, 0), kth_alt_crc_errors=(9 * k, 0))
        rows.append(bringup.state_row(Sample(k * 10_000_000, st)))
    rec.write_text("\n".join(json.dumps(r, default=str) for r in rows))
    args = ["kth", "--record", str(rec), "--configured-m", "g704_bytes", "--configured-h", "reflected_msb_first",
            "--image", "fw@x"]
    assert bringup.main(args) == 2  # synthetic without the flag: refused
    capsys.readouterr()
    assert bringup.main([*args, "--allow-synthetic"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["synthetic_inputs"] is True and out["synthetic"] is True
    assert out["axes"]["M"]["verdict"] == "synthetic" and out["axes"]["M"]["computed"] == "configured_ok"
    assert out["axes"]["H"]["verdict"] == "synthetic" and out["axes"]["H"]["computed"] == "undecided"



def _cleanup_after_reconnect(mode_after_write):
    """A DISARM written on a fresh connection before any of its STATEs; the first STATE after it names it."""
    import time as _t

    b = _bench([state()])
    s = bringup.Session(b, 1.0)
    steps = {"n": 0}

    def cycle():
        steps["n"] += 1
        if steps["n"] == 1:
            b.host.disconnected(_t.monotonic_ns())
            b.host.link_identity(None)  # connection 1, no STATE yet
            _wrote_disarm(b, 5)
        else:
            _feed(b, last_rx_seq=5, mode=mode_after_write)

    b.link.on_cycle = cycle
    return b, s


def test_a_disarm_before_the_first_state_of_a_new_connection_is_confirmed():
    """Codex PR-2a re-reviews 2-4: a stop confirmation needs no earlier sample -- the STATE received after the write
    names the DISARM and reports a stopped mode."""
    b, s = _cleanup_after_reconnect(Mode.DISARMED)
    assert s.cleanup() is True


def test_a_stopping_naming_the_disarm_is_not_a_confirmed_stop():
    b, s = _cleanup_after_reconnect(Mode.STOPPING)
    assert s.cleanup() is False


def test_the_old_connections_disarm_does_not_confirm_on_the_new_one():
    import time as _t

    b = _bench([state()])
    s = bringup.Session(b, 1.0)
    steps = {"n": 0}

    def cycle():
        steps["n"] += 1
        if steps["n"] == 1:
            _wrote_disarm(b, 5)  # on connection 0
            b.host.disconnected(_t.monotonic_ns())
            b.host.link_identity(None)  # connection 1
        else:
            _feed(b, last_rx_seq=5)

    b.link.on_cycle = cycle
    assert s.cleanup() is False


def test_a_buffered_stop_before_a_newer_armed_state_does_not_confirm():
    """Codex PR-2a re-review 5 P2-2: both STATEs are received after the DISARM's write; the newer is ARMED."""
    b = _bench([state()])
    s = bringup.Session(b, 1.0)
    steps = {"n": 0}

    def cycle():
        steps["n"] += 1
        if steps["n"] == 1:
            _wrote_disarm(b, 1)
        else:
            _feed(b, last_rx_seq=1, mode=Mode.DISARMED)
            _feed(b, last_rx_seq=0, mode=Mode.ARMED)

    b.link.on_cycle = cycle
    assert s.cleanup() is False


def test_an_out_of_order_stop_the_host_refused_does_not_confirm():
    """Codex PR-2a re-review 6 P2-2: an older-MCU-time DISARMED after an ARMED is refused by the host; it is not the
    newest valid STATE."""
    import dataclasses
    import time as _t

    from forklift_core.mcu_link.messages import encode

    b = _bench([state()])
    s = bringup.Session(b, 1.0)
    steps = {"n": 0}

    def cycle():
        steps["n"] += 1
        if steps["n"] == 1:
            _wrote_disarm(b, 1)
        elif steps["n"] == 2:
            _feed(b, last_rx_seq=0, mode=Mode.ARMED)
            t = b.host.state.mcu_time_us - 5_000  # older MCU time than the ARMED one, received after the write
            b.host.on_bytes(encode(dataclasses.replace(state(), mcu_time_us=t, last_rx_seq=1), 1), _t.monotonic_ns())

    b.link.on_cycle = cycle
    assert s.cleanup() is False


def test_an_unconfirmed_stop_is_retried_before_the_operator_is_asked(monkeypatch):
    """Codex PR-2a re-review 5 P2-1: the second cleanup comes before the direction prompt, not after it."""
    import argparse

    order = []
    b = _bench([state()], commission=True)

    def fake_run(bench, session, duty, seconds, profile=None):
        session.arm_requested, session.cleanup_confirmed = True, False
        return {"result": "ok", "metrics": {"duty_e4": 1000, "measured_sign": "pos", "duty_sign_in_build": 1}}

    def fake_cleanup(self):
        order.append("cleanup")
        self.cleanup_confirmed = True
        return True

    def fake_observed(args):
        order.append("observed")
        return "pos"

    monkeypatch.setattr(bringup, "run_commission", fake_run)
    monkeypatch.setattr(bringup.CommissionSession, "cleanup", fake_cleanup)
    monkeypatch.setattr(bringup, "_observed", fake_observed)
    a = argparse.Namespace(cmd="commission", axis="L", duty=0.1, seconds=0.6, window=None, observed=None, yes=True,
                           power_switch_in_hand=True)
    bringup.execute(a, b)
    assert order == ["cleanup", "observed"]


def test_no_direction_prompt_while_the_stop_is_unconfirmed(monkeypatch):
    """Codex PR-2a re-review 6 P2-1: both cleanups failed -- nobody is asked to look; observed is 'unknown'."""
    import argparse

    b = _bench([state()], commission=True)

    def fake_run(bench, session, duty, seconds, profile=None):
        session.arm_requested, session.cleanup_confirmed = True, False
        return {"result": "ok", "metrics": {"duty_e4": 1000, "measured_sign": "pos", "duty_sign_in_build": 1}}

    def failing_cleanup(self):
        self.cleanup_confirmed = False
        return False

    def no_prompt(args):
        raise AssertionError("asked the operator while the stop was unconfirmed")

    monkeypatch.setattr(bringup, "run_commission", fake_run)
    monkeypatch.setattr(bringup.CommissionSession, "cleanup", failing_cleanup)
    monkeypatch.setattr(bringup, "_observed", no_prompt)
    a = argparse.Namespace(cmd="commission", axis="L", duty=0.1, seconds=0.6, window=None, observed=None, yes=True,
                           power_switch_in_hand=True)
    res = bringup.execute(a, b)
    assert res["observed"] == "unknown"


@pytest.mark.parametrize("exc_type", [OSError, KeyboardInterrupt])
def test_a_failing_early_cleanup_keeps_the_measured_result(monkeypatch, exc_type):
    """Codex PR-2a re-reviews 6-7 P3: an I/O error or a Ctrl-C in the early cleanup."""
    import argparse

    b = _bench([state()], commission=True)

    def fake_run(bench, session, duty, seconds, profile=None):
        session.arm_requested, session.cleanup_confirmed = True, False
        return {"result": "ok", "metrics": {"duty_e4": 1000, "measured_sign": "pos", "duty_sign_in_build": 1}}

    calls = {"n": 0}

    def broken_cleanup(self):
        calls["n"] += 1
        if calls["n"] == 1:
            raise exc_type("x")
        self.cleanup_confirmed = False
        return False

    monkeypatch.setattr(bringup, "run_commission", fake_run)
    monkeypatch.setattr(bringup.CommissionSession, "cleanup", broken_cleanup)
    a = argparse.Namespace(cmd="commission", axis="L", duty=0.1, seconds=0.6, window=None, observed=None, yes=True,
                           power_switch_in_hand=True)
    res = bringup.execute(a, b)
    assert res["metrics"]["measured_sign"] == "pos" and "cleanup_error" in res and res["observed"] == "unknown"
    assert res["result"] == "aborted" and calls["n"] == 2  # and the final cleanup still ran


def _kth_record(tmp_path, header):
    import dataclasses
    import json

    from forklift_core.plant.bringup import Sample

    rec = tmp_path / "run.jsonl"
    rows = [header] if header else []
    for k in range(201):  # M moving and fitted; H's counters say nothing (no KTH)
        st = dataclasses.replace(state(), mcu_time_us=1_000_000 + k * 10_000, kth_frames=(10 * k, 0),
                                 kth_crc_errors=(0, 0), kth_alt_crc_errors=(9 * k, 0))
        rows.append(bringup.state_row(Sample(k * 10_000_000, st)))
    rec.write_text("\n".join(json.dumps(r, default=str) for r in rows))
    return rec


def test_kth_with_an_absent_axis(tmp_path, capsys):
    """Partial-parts plan v4: one KTH on the bench. The header's parts name the absent axis (lift = H); the flag must
    agree with a declaring header; an absent axis needs no variant and has a null verdict."""
    import json

    hdr = {"t": "header", "cmd": "record", "device": "/dev/forklift-mcu", "parts_absent": ["lift"]}
    rec = _kth_record(tmp_path, hdr)
    assert bringup.main(["kth", "--record", str(rec), "--configured-m", "g704_bytes", "--image", "fw@x"]) == 0
    out = json.loads(capsys.readouterr().out)
    assert out["axes"]["M"]["verdict"] == "configured_ok" and out["axes"]["H"]["verdict"] is None
    assert out["absent"] == ["H"] and out["parts_absent"] == ["lift"] and not out.get("not_for_gate")
    assert out["configured"] == {"M": "g704_bytes"}
    # the variant for an absent axis, or a flag the header does not declare: refused
    assert bringup.main(["kth", "--record", str(rec), "--configured-m", "g704_bytes", "--configured-h",
                         "g704_bytes", "--image", "x"]) == 2
    assert "absent axis H" in json.loads(capsys.readouterr().out)["why"]
    assert bringup.main(["kth", "--record", str(rec), "--configured-h", "g704_bytes", "--axis-absent", "M",
                         "--image", "x"]) == 2
    assert "disagrees" in json.loads(capsys.readouterr().out)["why"]
    # steering absent: no gate evidence -- refused, then read with --allow-bypassed (fitted verdict moved to computed)
    rec = _kth_record(tmp_path, {**hdr, "parts_absent": ["steer"]})
    assert bringup.main(["kth", "--record", str(rec), "--configured-h", "g704_bytes", "--image", "x"]) == 2
    assert "parts were absent" in json.loads(capsys.readouterr().out)["why"]
    assert bringup.main(["kth", "--record", str(rec), "--configured-h", "g704_bytes", "--image", "x",
                         "--allow-bypassed"]) == 1
    out = json.loads(capsys.readouterr().out)
    assert out["axes"]["M"] == {"absent": True, "verdict": None, "why": "no KTH fitted on this axis"}
    assert out["axes"]["H"]["verdict"] is None and out["axes"]["H"]["computed"] == "undecided"
    # a record from before the field: the flag is taken; both axes absent is refused
    rec = _kth_record(tmp_path, {"t": "header", "cmd": "record", "device": "/dev/forklift-mcu"})
    assert bringup.main(["kth", "--record", str(rec), "--configured-m", "g704_bytes", "--axis-absent", "H",
                         "--image", "x"]) == 0
    assert json.loads(capsys.readouterr().out)["axes"]["H"]["verdict"] is None
    assert bringup.main(["kth", "--record", str(rec), "--axis-absent", "H", "--axis-absent", "M",
                         "--image", "x"]) == 2


def test_a_partial_or_bypassed_commission_result_proposes_no_duty_sign():
    """Partial-parts plan v4: steer or wheels absent, like a bypass, empties the sign verdict and the proposal; lift
    absent alone is marked and still judged."""
    verdict = {"duty_sign_ok": True, "sensor_sign_ok": True, "suggested_duty_sign": -1, "note": None}
    res = {"verdict": dict(verdict), "metrics": {"valid": True}}
    bringup._mark_commission_result(res, (), False, None, ("wheels",))
    assert res["not_for_gate"] and res["parts_absent"] == ["wheels"]
    assert res["verdict"]["suggested_duty_sign"] is None and res["verdict"]["duty_sign_ok"] is None
    assert res["metrics"]["valid"] is None
    res = {"verdict": dict(verdict)}
    bringup._mark_commission_result(res, ("enc_stuck",), False, None)
    assert res["verdict"]["suggested_duty_sign"] is None
    res = {"verdict": dict(verdict)}
    bringup._mark_commission_result(res, (), False, None, ("lift",))
    assert res == {"verdict": verdict, "parts_absent": ["lift"]}


def test_kth_refuses_a_file_whose_runs_declare_different_parts_and_counts_an_old_records_flag(tmp_path, capsys):
    """Codex PR-A P2-3, P2-4: kth reads every STATE of the file, so its runs must agree; a flag on a record from
    before the field still makes a steering-absent reading no gate evidence."""
    import json

    rec = _kth_record(tmp_path, {"t": "header", "cmd": "record", "parts_absent": ["lift"]})
    text = rec.read_text()
    rec.write_text(text + "\n" + json.dumps({"t": "header", "cmd": "record", "parts_absent": []}) + "\n" + text)
    assert bringup.main(["kth", "--record", str(rec), "--configured-m", "g704_bytes", "--image", "x"]) == 2
    assert "different absent parts" in json.loads(capsys.readouterr().out)["why"]
    rec = _kth_record(tmp_path, {"t": "header", "cmd": "record"})
    assert bringup.main(["kth", "--record", str(rec), "--configured-h", "g704_bytes", "--axis-absent", "M",
                         "--image", "x"]) == 2
    assert "parts were absent" in json.loads(capsys.readouterr().out)["why"]


def test_step_scenario_and_cpr_results_with_parts_absent_carry_no_pass():
    """Codex PR-A P2-2: the measured values stay, the official pass fields go."""
    res = {"result": "ok", "external": {"output_response_ms": {"value": 25.0, "limit": 100.0, "pass": True}}}
    bringup._mark_absent_result(res, ("steer",))
    assert res["not_for_gate"] and res["external"]["output_response_ms"]["pass"] is None
    assert res["external"]["output_response_ms"]["value"] == 25.0
    cpr = {"valid": True, "cpr": [4096, 4096]}
    bringup._mark_absent_result(cpr, ("steer",))
    assert cpr["valid"] is None and cpr["computed_valid"] is True and cpr["parts_absent"] == ["steer"]
    lift = {"valid": True}
    bringup._mark_absent_result(lift, ("lift",))
    assert lift == {"valid": True, "parts_absent": ["lift"]}


def test_clear_faults_takes_only_this_runs_echo():
    """Codex PR-A P2-1: a STATE read before the frame's last byte, on another connection, of another boot or not newer
    than the floor is never the frame's acceptance."""
    import dataclasses
    from types import SimpleNamespace

    from forklift_core.plant.bringup import Sample
    from forklift_core.plant.write_log import Write

    def w(kind, seq, conn, done):
        return Write(order=0, conn=conn, kind=kind, seq=seq, start_ns=done - 10, done_ns=done, boot_id=None,
                     echo_mcu_time_us=None, valid_ms=None, arm_gen=None, axis=None, duty_e4=None, phase=None)

    def smp(host_ns, seq, t_us, boot=7):
        return Sample(host_ns, dataclasses.replace(state(), last_rx_seq=seq, mcu_time_us=t_us, boot_id=boot))

    b = SimpleNamespace(writes=[w("CLEAR_FAULTS", 1, 2, 1000)], samples=[], sample_conn=[])
    for host_ns, seq, t_us, conn, boot in [(900, 1, 600, 2, 7),     # read before the write ended
                                            (1100, 1, 700, 1, 7),    # another connection
                                            (1200, 1, 800, 2, 8),    # another boot
                                            (1300, 1, 400, 2, 7),    # not newer than the floor (a buffered STATE)
                                            (1400, 1, 900, 2, 7)]:   # this run's
        b.samples.append(smp(host_ns, seq, t_us, boot))
        b.sample_conn.append(conn)
    assert bringup._echo_index(b, "CLEAR_FAULTS", 1, 2, 7, 500) == 4
    assert bringup._echo_index(b, "CLEAR_FAULTS", 1, 2, 7, 500, after=4) is None
    assert bringup._echo_index(b, "CLEAR_FAULTS", 2, 2, 7, 500) is None  # never written


class _ClearBoard:
    """A scripted board for clear-faults: per tick it takes the frames written, then reports STATEs. ``relatch``:
    after the clear's ready STATE, a FAULT comes in the same read; ``reboot_with_disarm_echo``: the DISARM's echo and
    a new boot's FAULT arrive in one read; ``late``: a STATE of an earlier run carrying ``late`` arrives first."""

    def __init__(self, *, relatch=False, reboot_with_disarm_echo=False, late=None, initial_seq=0x1234, refuse=False):
        import dataclasses

        from forklift_core.mcu_link.messages import Mode
        from forklift_core.plant.mcu_host import Blocked

        self.dc, self.Mode, self.Blocked = dataclasses, Mode, Blocked
        self.relatch, self.reboot, self.refuse = relatch, reboot_with_disarm_echo, refuse
        self.mode, self.rx, self.boot, self.t, self.ready_ = Mode.FAULT, 0, 7, 1000, False
        self.samples, self.sample_conn, self.writes, self.queue, self.conn = [], [], [], [], 1
        self.seq, self.last_queued_seq, self.now = initial_seq, None, 0
        self.pending_late = late
        self.host = self

    # host
    def request_disarm(self, now):
        self.queue = [q for q in self.queue if q[0] == "DISARM"] or [("DISARM", self._next())]

    def queued_disarm_seq(self):
        return next(s for k, s in self.queue if k == "DISARM")

    def request_clear_faults(self, now):
        if self.state.mode not in (self.Mode.FAULT, self.Mode.ESTOP):
            return self.Blocked("not_latched")
        self.queue.append(("CLEAR_FAULTS", self._next()))
        return None

    def _next(self):
        s, self.seq = self.seq, (self.seq + 1) & 0xFFFF
        self.last_queued_seq = s
        return s

    # bench
    @property
    def state(self):
        return self.samples[-1].state if self.samples else None

    def _report(self, **kw):
        from forklift_core.mcu_link.messages import StateFlag
        from forklift_core.plant.bringup import Sample

        self.t += 10_000
        self.now += 10
        flags = StateFlag.DRIVERS_READY if self.ready_ else StateFlag(0)
        st = self.dc.replace(state(), mode=self.mode, last_rx_seq=self.rx, boot_id=self.boot, mcu_time_us=self.t,
                             flags=flags, **kw)
        self.samples.append(Sample(self.now, st))
        self.sample_conn.append(self.conn)

    def run_until(self, pred, timeout_s, **_):
        from forklift_core.plant.write_log import Write

        for _ in range(int(timeout_s * 100) + 1):
            self.now += 1
            for kind, seq in self.queue:
                self.writes.append(Write(order=len(self.writes), conn=self.conn, kind=kind, seq=seq,
                                         start_ns=self.now, done_ns=self.now, boot_id=None, echo_mcu_time_us=None,
                                         valid_ms=None, arm_gen=None, axis=None, duty_e4=None, phase=None))
                if kind == "DISARM":
                    self.rx = seq
                    if self.reboot:
                        self._report()  # the echo, then the new boot in the same read
                        self.boot, self.rx, self.mode = 8, 0, self.Mode.FAULT
                elif kind == "CLEAR_FAULTS" and not self.refuse and self.mode in (self.Mode.FAULT, self.Mode.ESTOP):
                    self.rx, self.mode, self.ready_ = seq, self.Mode.DISARMED, True
                    if self.relatch:
                        self._report()
                        self.mode, self.ready_ = self.Mode.FAULT, False
            self.queue = []
            if self.pending_late is not None:
                rx, self.rx = self.rx, self.pending_late
                self._report()
                self.rx, self.pending_late = rx, None
            self._report()
            if pred():
                return True
        return False


@pytest.mark.parametrize("kw, expected, clears_sent", [
    ({}, "cleared", 1),
    ({"relatch": True}, "not_ready", 1),               # ready then FAULT in one read: the newest STATE decides
    ({"reboot_with_disarm_echo": True}, "rebooted", 0),  # nothing is sent to the new boot
    ({"refuse": True}, "not_accepted", 1),
    ({"initial_seq": 0}, "seq_collision", 0),           # 0 is what any boot shows
    ({"late": 0x1235}, "seq_collision", 0),             # an earlier run's STATE showed the CLEAR's number
])
def test_clear_faults_decides_on_this_runs_frames_and_the_newest_state(monkeypatch, kw, expected, clears_sent):
    """Codex PR-A round 2: STATE repeats the last accepted seq, so an echo counts only for a number no STATE of this
    session has shown; a reboot read with the DISARM's echo sends no CLEAR; a re-latch read with the ready STATE is not
    cleared."""
    monkeypatch.setattr(bringup, "CLEAR_DRAIN_S", 0.02)
    b = _ClearBoard(**kw)
    res = bringup.clear_faults(b)
    assert res["result"] == expected, res
    assert sum(1 for w in b.writes if w.kind == "CLEAR_FAULTS") == clears_sent
    assert all(w.kind in ("DISARM", "CLEAR_FAULTS") for w in b.writes)


def test_a_real_device_needs_the_product_string_whatever_the_baud():
    """Codex PR-B round 2: --baud is set on USB CDC too, so it never excuses an unreadable product on an open
    commissioning board; only the rehearsal bench's model (synthetic) has none by nature."""
    assert bringup._product_required(None) is True
    assert bringup._product_required({"bench": {"pts": "/dev/pts/3"}}) is False
