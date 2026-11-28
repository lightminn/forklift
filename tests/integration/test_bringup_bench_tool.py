"""The commissioning tool on the bench image (bench console plan A1a, mcu-link contract 8): the MCU model with
--bench 1 -- the run's BENCH_SET read back by its seq, BENCH_ARM instead of ARM, one zero ending the run with the MCU's
own stop, the steering run with its range turned off, the run's limits, and the refusals. The model's plant is
synthetic: no currents, speeds or real turns."""

import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tests" / "integration"))
import bringup  # noqa: E402
from test_bringup_commission_tool import LR1, args, kinds_written, model  # noqa: E402,F401
from test_bringup_tool import mcu_model  # noqa: E402,F401

from forklift_core.mcu_link.messages import Mode  # noqa: E402

BENCH5 = "forklift-mcu commission OPEN v0-test rec=00000000 lim=lr1 bench8"


def rows(b):
    return [json.loads(x) for x in b.out.getvalue().splitlines()]


def test_a_steering_run_with_its_range_off_and_its_own_limits(model):
    b = model("--bench", "1")
    res = bringup.execute(args(axis="M", duty=0.2, seconds=1.0, bench_bypass="steer_range", bench_duty_max=0.25,
                               bench_time_max=5.0, observed="none"), b)
    assert res["result"] == "ok" and res["cleanup_confirmed"], res
    taken = res["bench_set"]
    assert taken["bypass"] == ["steer_range"] and taken["overrides"]
    assert taken["duty_max_e4"] == [10000, 10000, 2500, 1500] and taken["time_ms"][2] == 5000
    kinds = kinds_written(b)
    assert "ARM" not in kinds  # the bench image arms with BENCH_ARM only
    assert kinds.index("BENCH_SET") < kinds.index("BENCH_ARM")
    assert max(abs(s.state.duty_e4[2]) for s in b.samples) == 2000  # 0.2 > the image's 0.15
    run = res["bench_run"]
    assert run["confirmed"] and run["stop_reason"] == "end_zero" and run["run_id"] == taken["run_id"]
    assert b.host.bench_state.mcu_bypass == 0  # the run is over: its bypass ended with its arming
    assert res["not_for_gate"] and "bench:steer_range" in res["bypassed"]
    assert res["bench"] == {"bypass": ["steer_range"], "duty_max": 0.25, "time_max_s": 5.0}
    (bs,) = [r for r in rows(b) if r["t"] == "bench_set"]
    assert bs["seq"] == taken["seq"]
    assert taken["seq"] != 0  # a fresh boot's last_rx_seq is 0: the BENCH_SET never takes that number (P2)


def test_a_wheel_run_on_the_defaults_ends_with_one_zero_and_the_boards_own_stop(model):
    """Every run on the bench image sends its BENCH_SET (the defaults here), and after driving one zero ends it: the
    MCU stops it itself, so no zero after that one is refused as stopping."""
    b = model("--bench", "1")
    res = bringup.execute(args(axis="L", duty=0.1, seconds=0.6, observed="none"), b)
    assert res["result"] == "ok" and res["cleanup_confirmed"], res
    assert res["bench_set"]["overrides"] is False and res["bench_set"]["bypass"] == []
    assert res["bench_run"]["stop_reason"] == "end_zero" and res["bench_run"]["blocked"] == 0
    assert res["aborted"] is None and res["not_for_gate"]  # every bench run stays out of gates
    assert b.samples[-1].state.mode == Mode.DISARMED
    post = [w for w in b.writes if w.kind == "COMMISSION" and w.phase == "post_zero"]
    assert len(post) == 1


def test_bench_values_on_another_image_are_refused_before_anything(model):
    b = model()
    b.host.usb_product = LR1
    res = bringup.execute(args(axis="M", duty=0.12, seconds=1.0, bench_bypass="steer_range", observed="none"), b)
    assert res["result"] == "refused" and "not the bench image" in res["why"]
    assert not {"ARM", "BENCH_ARM", "BENCH_SET"} & set(kinds_written(b))


@pytest.mark.parametrize("product", [
    "forklift-mcu commission OPEN v0-test rec=00000000 lim=lr1 bench4",
    "forklift-mcu commission OPEN diag-fw-bench-2 rec=1f4207f4 lim=lr1 bench3",
    "forklift-mcu commission OPEN v0-test rec=00000000 lim=lr1 bench7",  # A1b-2's board under a bench8 host
])
def test_another_bench_generation_is_refused_for_every_run(model, product):
    b = model("--bench", "1")
    b.host.usb_product = product
    res = bringup.execute(args(axis="L", duty=0.1, seconds=0.6, observed="none"), b)
    assert res["result"] == "refused" and "another bench generation" in res["why"]
    assert not {"ARM", "BENCH_ARM", "BENCH_SET"} & set(kinds_written(b))


@pytest.mark.parametrize("kw,why", [
    (dict(bench_bypass="steer_range", window=(-3.2, 3.2)), "give no --window"),
    (dict(bench_bypass="overcurrent"), "unknown name"),
    (dict(bench_bypass="lift_range"), "lift axis (H) only"),
    (dict(bench_duty_max=0.25, duty=0.3), "within +-0.25"),
    (dict(bench_duty_max=1.5), "bench-duty-max"),
    (dict(bench_time_max=1.0, seconds=0.8), "seconds must be in"),
    (dict(bench_bypass="steer_range", single_axis=True, motor_detached=True), "a plain run only"),
    (dict(bench_bypass="steer_range", bench_duty_max=0.5, duty=0.35), "watch VCC"),
])
def test_bench_argument_refusals(model, kw, why):
    b = model("--bench", "1")
    base = dict(axis="M", duty=0.12, seconds=1.0, observed="none")
    res = bringup.execute(args(**{**base, **kw}), b)
    assert res["result"] == "refused" and why in res["why"], res
    assert not {"ARM", "BENCH_ARM", "BENCH_SET"} & set(kinds_written(b))


def test_without_the_range_bypass_the_steering_still_needs_its_window(model):
    b = model("--bench", "1")
    res = bringup.execute(args(axis="M", duty=0.12, seconds=1.0, bench_duty_max=0.25, observed="none"), b)
    assert res["result"] == "refused" and "--window" in res["why"]


def test_a_plain_arm_is_never_sent_to_the_bench_image(model):
    """The host itself refuses a plain ARM on the bench image (the board would answer wrong_build)."""
    b = model("--bench", "1")
    b.run_until(lambda: b.state is not None and b.state.mode == Mode.DISARMED, 3.0)
    import time

    blocked = b.host.request_arm(time.monotonic_ns())
    assert blocked is not None and blocked.reason == "bench_image_arms_with_bench_arm"


def test_steering_above_0_3_runs_with_the_vcc_watch_confirmed(model):
    b = model("--bench", "1")
    res = bringup.execute(args(axis="M", duty=0.35, seconds=1.0, bench_bypass="steer_range", bench_duty_max=0.5,
                               vcc_watched=True, observed="none"), b)
    assert res["result"] == "ok", res


def test_the_next_run_recovers_the_last_runs_record_before_its_arm(model):
    """A run's record stays on the board until the next run is consumed: the next run writes it down first (a CLI
    that ended before recording it, same boot)."""
    b = model("--bench", "1")
    first = bringup.execute(args(axis="L", duty=0.1, seconds=0.6, observed="none"), b)
    assert first["result"] == "ok", first
    second = bringup.execute(args(axis="L", duty=0.1, seconds=0.6, observed="none"), b)
    assert second["result"] == "ok", second
    prev = [r for r in rows(b) if r["t"] == "bench_previous_result"]
    assert prev and prev[-1]["run_id"] == first["bench_set"]["run_id"] and prev[-1]["stop_reason"] == "end_zero"


def test_a_run_the_board_ended_by_expiry_is_not_ok(model, monkeypatch):
    """The end zero never sent: the board ends the run by the command expiry, and the board's first cause decides."""
    b = model("--bench", "1")

    def late_end(self):
        self.b.run_until(lambda: self.b.state.mode != Mode.ARMED, 1.0)  # no zero: the commands expire

    monkeypatch.setattr(bringup.CommissionSession, "end_bench_run", late_end)
    res = bringup.execute(args(axis="L", duty=0.1, seconds=0.6, observed="none"), b)
    assert res["result"] != "ok" and res["aborted"] == "bench_stop_command_timeout", res
    assert res["bench_run"]["stop_reason"] == "command_timeout"


def test_a_cpr_on_the_bench_image_keeps_no_official_pass():
    out = {"valid": True, "counts_per_rev": [4096, 4096]}
    bringup._mark_diagnostic_cpr(out, [BENCH5, None])  # the USB lost at the end: the opening's product still counts
    assert out["valid"] is None and out["computed_valid"] is True and out["not_for_gate"]
    plain = {"valid": True}
    bringup._mark_diagnostic_cpr(plain, [LR1])
    assert plain == {"valid": True}
    absent = {"valid": None, "computed_valid": True}  # parts absent marked it first: its computed value stays
    bringup._mark_diagnostic_cpr(absent, [BENCH5])
    assert absent["computed_valid"] is True and absent["valid"] is None


def test_a_record_that_cannot_be_written_stops_the_next_arm(model, monkeypatch):
    """The next BENCH_ARM clears the board's record of the last run: if that record cannot be written, nothing is
    armed."""
    b = model("--bench", "1")
    first = bringup.execute(args(axis="L", duty=0.1, seconds=0.6, observed="none"), b)
    assert first["result"] == "ok", first
    real_emit = b.emit

    def failing(row):
        if row.get("t") == "bench_previous_result":
            b.record_errors += 1  # what the record's own error handling does on an OSError
            return
        real_emit(row)

    monkeypatch.setattr(b, "emit", failing)
    n = len(b.writes)
    second = bringup.execute(args(axis="L", duty=0.1, seconds=0.6, observed="none"), b)
    assert second["result"] == "refused" and "could not be written" in second["why"]
    assert not {"BENCH_SET", "BENCH_ARM"} & {w.kind for w in b.writes[n:]}


# ---- bench6 distance runs (plan 4.2 A1b-1b) -------------------------------------------------------------------------

def test_a_steering_distance_run_ends_at_its_distance(model):
    """--stop-kth-turns: the board cuts the output at the distance and stops the run itself; the CLI reads the latched
    result (never an authorization revoked), records the distance verdict and the rest position at 100 Hz."""
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(args(axis="M", duty=0.2, seconds=3.0, bench_bypass="steer_range", bench_duty_max=0.25,
                               bench_time_max=5.0, stop_kth_turns=0.02, observed="none"), b)
    taken = res["bench_set"]
    assert taken["source"] == 1 and taken["stop_counts"] == 82 and taken["count_sign"] == 1, taken
    assert res["aborted"] is None and res["cleanup_confirmed"], res
    assert res["stop"] == "board_distance_reached", res
    assert res["distance"]["result"] == "reached" and res["distance"]["moved_at_cut"] >= 82, res["distance"]
    assert res["start"] == "accepted"
    assert res["rest"]["result"] in ("settled_at_100hz", "unconfirmed"), res["rest"]
    post = [w for w in b.writes if w.kind == "COMMISSION" and w.phase == "post_zero"]
    assert post == []  # the board ended it: no zero sent into its stop


def test_a_wrong_count_sign_stops_without_progress(model):
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(args(axis="M", duty=0.2, seconds=3.0, bench_bypass="steer_range", bench_duty_max=0.25,
                               bench_time_max=5.0, stop_kth_turns=0.5, count_sign=-1, observed="none"), b)
    assert res["distance"]["result"] == "stopped:no_progress", res
    assert res["aborted"] == "bench_stop_no_progress"


def test_a_count_stuck_from_the_start_refuses_the_distance_arm(model):
    """The KTH frame 0x0000 from the start: no trusted sample, so the board refuses the distance run's ARM."""
    b = model("--bench", "1", "--kth-zero-at-ms", "M:0")
    res = bringup.execute(args(axis="M", duty=0.2, seconds=1.0, bench_bypass="steer_range", bench_duty_max=0.25,
                               stop_kth_turns=0.02, observed="none"), b)
    assert res["result"] == "refused", res
    assert not [w for w in b.writes if w.kind == "COMMISSION" and w.duty_e4 != 0]


@pytest.mark.parametrize(("axis", "turns", "why"), [
    ("M", 0.0001, "counts"),  # rounds to 0: never a run without its distance stop
    ("M", -1.0, "positive"),
    ("L", 0.5, "M's or H's"),
])
def test_bad_distances_are_refused_before_anything(model, axis, turns, why):
    b = model("--bench", "1")
    res = bringup.execute(args(axis=axis, duty=0.1, seconds=1.0, stop_kth_turns=turns, observed="none"), b)
    assert res["result"] == "refused" and why in res["why"], res
    assert not {"BENCH_SET", "BENCH_ARM"} & set(kinds_written(b))


def nonzero_writes(b):
    return [w for w in b.writes if w.kind == "COMMISSION" and w.duty_e4]


def test_a_high_duty_distance_is_shown_never_refused(model, capsys):
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(args(axis="M", duty=0.45, seconds=2.0, bench_bypass="steer_range", bench_duty_max=0.5,
                               bench_time_max=5.0, stop_kth_turns=0.02, vcc_watched=True, observed="none"), b)
    assert res["result"] != "refused", res
    err = capsys.readouterr().err
    assert err.count("advisory: a distance stop at duty 0.45 is unverified") == 1
    assert [r for r in rows(b) if r["t"] == "bench_advisory"]


def test_a_distance_run_ended_before_its_target_failed(model):
    """The tool's time ran out before the distance (its one zero ended the run): the run failed (plan 4.2 decision 8),
    never 'ok'."""
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(args(axis="M", duty=0.2, seconds=0.5, bench_bypass="steer_range", bench_duty_max=0.25,
                               bench_time_max=5.0, stop_kth_turns=5.0, observed="none"), b)
    assert res["distance"]["result"] == "missed" and res["distance"]["ended_by"] == "end_zero", res
    assert res["aborted"] == "bench_stop_end_zero"


def test_a_one_count_run_has_no_driving_sample(model):
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(args(axis="M", duty=0.2, seconds=2.0, bench_bypass="steer_range", bench_duty_max=0.25,
                               bench_time_max=5.0, stop_kth_turns=1 / 4096, observed="none"), b)
    assert res["distance"]["result"] == "reached", res
    driven = any(s.state.mode == Mode.ARMED and s.state.duty_e4[2] for s in b.samples)
    if driven:
        assert res["result"] == "ok", res
    else:
        assert res["result"] == "board_record_only" and res["metrics"]["reason"] == "no_driving_sample", res


def test_the_window_is_judged_before_a_second_non_zero(model):
    """The first accepted sample already outside the window: no second non-zero goes out (impl round 1 P1-2)."""
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(args(axis="M", duty=0.2, seconds=2.0, window=(-0.5, 0.0001), bench_duty_max=0.25,
                               bench_time_max=5.0, stop_kth_turns=0.5, observed="none"), b)
    assert res["stop"] == "window", res
    nz = nonzero_writes(b)
    out = [smp for smp in b.samples if smp.state.mode == Mode.ARMED and smp.state.steering_rad > 0.0001]
    assert out, "the axis never left the window"
    t_out = out[0].host_ns
    # the window is judged on every sample before the next non-zero: once the first sample outside was read, at most
    # the one already queued in that cycle goes out (impl round 1 P1-2)
    assert len([w for w in nz if w.start_ns > t_out]) <= 1, nz
    first = nz[0].seq & 0xFFFF
    acc = next((smp for smp in b.samples if smp.state.last_rx_seq == first), None)
    if acc is not None and acc.state.steering_rad > 0.0001:
        assert len(nz) == 1, nz  # the first accepted sample was already outside: no second non-zero


def test_a_measuring_kth_run(model):
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(args(axis="M", duty=0.2, seconds=0.8, bench_bypass="steer_range", bench_duty_max=0.25,
                               bench_time_max=5.0, kth_measure=True, observed="none"), b)
    assert res["bench_set"]["source"] == 1 and res["bench_set"]["stop_counts"] == 0, res
    assert res["result"] == "ok" and res["aborted"] is None, res
    assert res["measure"]["status"] == "confirmed" and res["measure"]["moved"] > 0, res["measure"]


def test_a_refused_first_non_zero_is_written_once_and_closed_with_a_written_zero(mcu_model, monkeypatch):  # noqa: F811
    """The ARM taken, then the axis pushed to raw 0 (SIGUSR1) before the first non-zero: the board refuses it
    (sensor_invalid); exactly one non-zero is written, then a written zero and the cleanup (Codex A1b-1b r2)."""
    import io
    import signal

    from test_bringup_commission_tool import Model, PtyLink

    m = Model(mcu_model, "--bench", "1", "--kth-start-raw", "256,256", "--axis-push-steer-rad", "-0.3926990817",
              commission=True)
    try:
        b = bringup.Bench(PtyLink(m.master), io.StringIO(), allow_locked=False, commission=True)
            orig = bringup.CommissionSession.confirm_bench_arm

        def confirm_then_push(self):
            orig(self)
            m.proc.send_signal(signal.SIGUSR1)  # M to raw 0 during the preparation zeros

        monkeypatch.setattr(bringup.CommissionSession, "confirm_bench_arm", confirm_then_push)
        res = bringup.execute(args(axis="M", duty=0.2, seconds=2.0, bench_bypass="steer_range", bench_duty_max=0.25,
                                   bench_time_max=5.0, stop_kth_turns=0.5, observed="none"), b)
        assert res.get("start") == "refused" and res["aborted"] == "bench_start_refused_sensor_invalid", res
        coms = [w for w in b.writes if w.kind == "COMMISSION"]
        nz = [i for i, w in enumerate(coms) if w.duty_e4]
        assert len(nz) == 1, coms
        assert any(w.duty_e4 == 0 for w in coms[nz[0] + 1:])  # the closing zero went out
    finally:
        m.close()


def test_a_short_distance_run_waits_for_the_boards_result(model):
    """--seconds 0.1 with a one-count target: the board may stop before the duty loop's own end; the CLI waits for the
    result pair instead of trying an end zero into the stop (Codex A1b-1b round 3 P2-2)."""
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(args(axis="M", duty=0.2, seconds=0.1, bench_bypass="steer_range", bench_duty_max=0.25,
                               bench_time_max=5.0, stop_kth_turns=1 / 4096, observed="none"), b)
    assert res.get("aborted") != "bench_end_zero_not_queued", res
    assert res["result"] != "refused" and res["distance"]["result"] in ("reached", "missed"), res


# ---- bench7 hold runs (plan 4.3 A1b-2b) -----------------------------------------------------------------------------

HOLD_DEFAULTS = dict(hold_kp=None, hold_ki=None, hold_kd=None, hold_deadband=None, hold_min_duty=None, count_sign=1)


def hargs(**kw):
    base = dict(axis="M", duty=0.15, bench_bypass="steer_range", observed="none", **HOLD_DEFAULTS)
    return args(**{**base, **kw})


def test_a_here_hold_settles_and_ends_by_its_zero(model):
    """A here hold: the readiness zero on M (the board refuses L(0) in a hold run), BENCH_HOLDs for the run, the end
    zero, verdict 2 met, the result named a hold with its own bypass and target (plan 4.3 decisions 9-10)."""
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(hargs(hold="here", seconds=1.6), b)
    assert res["result"] == "ok" and res["cleanup_confirmed"], res
    h = res["hold"]
    assert h["start"] == "accepted" and h["holds_sent"] > 10, h
    assert h["verdict"]["result"] == "met", h["verdict"]
    assert h["settings"]["target"] == "here" and h["settings"]["cap"] == 0.15
    assert "PWM 출력(브리지 미확인)" in h["notes"]
    br = res["bench_result"]
    assert br["kind"] == "hold" and br["stop_reason"] == "end_zero" and br["result_bypass"] == ["steer_range"], br
    assert "distance" not in br  # a distance verdict on distance results only
    zeros = [w for w in b.writes if w.kind == "COMMISSION" and w.duty_e4 == 0]
    assert zeros and all(w.axis == 2 for w in zeros), zeros  # every zero on the run axis
    assert res["bench_set"]["mode"] == 2 and res["bench_set"]["source"] == 1
    assert h["entries"]["label"] is None and h["verdict"]["settle"]["samples"] >= 50
    assert h["verdict1"]["ok"] and h["rest"] is not None
    assert [r for r in rows(b) if r["t"] == "hold_settings"] and [r for r in rows(b) if r["t"] == "bench_pair"]
    assert "BENCH_HOLD" in kinds_written(b)


def test_a_rel_hold_moves_to_its_target(model):
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(hargs(hold="rel:60", seconds=2.5), b)
    assert res["result"] == "ok", res
    v = res["hold"]["verdict"]
    assert v["result"] == "met" and abs(v["end_error"]) <= 12, v
    first = next(bs for bs in [b.host.bench_state] if bs is not None)
    assert first.target - res["bench_result"]["cut_position"] == v["end_error"]
    assert any(s.state.duty_e4[2] > 0 for s in b.samples if s.state.mode == Mode.ARMED)  # it drove toward +60


def test_a_pushed_hold_comes_back(mcu_model):  # noqa: F811
    """here, then the axis pushed away (SIGUSR1) mid-run: the hold drives it back and ends normally."""
    import io
    import signal
    import threading

    from test_bringup_commission_tool import Model, PtyLink

    m = Model(mcu_model, "--bench", "1", "--kth-start-raw", "1000,1000", "--axis-push-steer-rad", "0.08",
              commission=True)
    try:
        b = bringup.Bench(PtyLink(m.master), io.StringIO(), allow_locked=False, commission=True)
            orig = bringup.CommissionSession.hold_run

        def push_mid(self, seconds, stop=None):
            threading.Timer(0.6, lambda: m.proc.send_signal(signal.SIGUSR1)).start()
            return orig(self, seconds, stop)

        bringup.CommissionSession.hold_run = push_mid
        try:
            res = bringup.execute(hargs(hold="here", seconds=3.0), b)
        finally:
            bringup.CommissionSession.hold_run = orig
        assert res["result"] == "ok", res
        assert res["hold"]["verdict"]["result"] == "met", res["hold"]
        moved = [s.state.duty_e4[2] for s in b.samples if s.state.mode == Mode.ARMED and s.state.duty_e4[2]]
        assert moved, "the hold never drove back"
    finally:
        m.close()


def test_a_hold_on_a_stuck_count_stops_and_fails(model):
    """here at raw 16, then the count reads 0x0000 only: the output is withheld outside the deadband and the board
    stops the run as kth_zero_suspect within 0.3 s; verdict 2 failed (plan 4.3 decisions 5, 7)."""
    b = model("--bench", "1", "--kth-start-raw", "16,16", "--kth-zero-at-ms", "M:1500")
    res = bringup.execute(hargs(hold="here", seconds=5.0), b)
    assert res["bench_result"]["stop_reason"] == "kth_zero_suspect", res
    assert res["hold"]["verdict"]["result"] == "failed"
    assert res["result"] == "aborted"


@pytest.mark.parametrize(("kw", "why"), [
    (dict(stop_kth_turns=0.5), "no --stop-kth-turns"),
    (dict(axis="L", bench_bypass=None), "M or H"),
    (dict(hold_kp=0.0), "hold-kp"),
    (dict(hold_deadband=600), "hold-deadband"),
    (dict(duty=0.2), "duty cap"),
    (dict(seconds=1.0), "a hold runs"),
    (dict(hold="abs:5000"), "0..4095"),
    (dict(hold=None, hold_kp=0.6, window=(-1.0, 1.0), bench_bypass=None, duty=0.12, seconds=0.5), "only with --hold"),
])
def test_hold_refusals_before_anything(model, kw, why):
    b = model("--bench", "1")
    res = bringup.execute(hargs(**{**dict(hold="here", seconds=2.0), **kw}), b)
    assert res["result"] == "refused" and why in res["why"], res
    assert not {"BENCH_SET", "BENCH_ARM", "BENCH_HOLD"} & set(kinds_written(b))


def test_the_cli_gate_abandons_a_cancelled_hold_continuation():
    """The bench CLI's link (plan 4.3 decision 10): a BENCH_HOLD half written, then the session cancelled -> its
    continuation is refused, the frame abandoned and the poison written; never the rest of the frame."""
    import os
    import time

    from real_mcu_link import BenchTtyLink

    from forklift_core.mcu_link.messages import BenchHold
    from forklift_core.plant.mcu_host import Outgoing

    r, w = os.pipe()
    os.set_blocking(r, False)
    written = []

    frame = {}

    def writer(fd, data):
        if bytes(data) == frame.get("data")[5:] and not frame.get("again"):
            frame["again"] = True
            raise BlockingIOError  # the continuation waits (EAGAIN): the session is cancelled meanwhile
        n = 5 if bytes(data) == frame.get("data") else len(data)  # the frame's first write stops short
        written.append(bytes(data[:n]))
        return os.write(fd, data[:n])

    holder = {"host": None, "cancelled": False}

    class Host:
        auth_epoch = 3
        events = []

        def next_frame(self, now):
            return self.q.pop(0) if self.q else None

        def frame_written(self, f, now):
            self.events.append(("written", f.seq))

        def frame_abandoned(self, f, n, now):
            self.events.append(("abandoned", f.seq, n))

        def unsent(self, f, now):
            self.events.append(("unsent", f.seq))
            self.q.insert(0, f)

        def frame_refused(self, f, now, why):
            self.events.append(("refused", f.seq))

    host = Host()
    holder["host"] = host
    msg = BenchHold(1, 1, 1000, 100, 9, 2, 1, 2, 0, 4096, 0.5, 0.0, 0.0, 12, 1500, 1200, 1)
    from forklift_core.mcu_link.messages import encode

    data = encode(msg, 7)
    frame["data"] = bytes(data)
    host.q = [Outgoing("BENCH_HOLD", 7, data, time.monotonic_ns() + 10**9, 3, True, None)]
    link = BenchTtyLink(w, gate=lambda f, wr: bringup._cli_gate(holder, f, wr), writer=writer)
    link.pump_writes(host, time.monotonic_ns())
    k = next(i for i, x in enumerate(written) if len(x) == 5)  # after the opening's poison, the frame's 5 bytes
    holder["cancelled"] = True
    link.pump_writes(host, time.monotonic_ns())
    assert ("abandoned", 7, 5) in host.events, host.events
    rest = b"".join(written[k + 1:])
    assert rest and set(rest[:-1]) <= {0x01} and rest.endswith(b"\x00"), rest  # the poison, never the frame's tail
    assert data[5:] not in rest
    os.close(r)
    os.close(w)


def test_the_cli_gate_rules():
    import time
    from types import SimpleNamespace

    holder = {"host": SimpleNamespace(auth_epoch=4), "cancelled": False}
    calls = []
    now = time.monotonic_ns()
    hold = SimpleNamespace(kind="BENCH_HOLD", nonzero=True, generation=4, expires_ns=now + 10**9)
    assert bringup._cli_gate(holder, hold, lambda: calls.append(1)) and calls == [1]
    assert not bringup._cli_gate(holder, SimpleNamespace(**{**hold.__dict__, "generation": 3}), lambda: calls.append(2))
    assert not bringup._cli_gate(holder, SimpleNamespace(**{**hold.__dict__, "expires_ns": now - 1}),
                                 lambda: calls.append(3))
    zero = SimpleNamespace(kind="COMMISSION", nonzero=False, generation=0, expires_ns=now - 1)
    holder["cancelled"] = True
    assert not bringup._cli_gate(holder, hold, lambda: calls.append(4))
    assert bringup._cli_gate(holder, zero, lambda: calls.append(5))  # a zero always passes (a stop)
    disarm = SimpleNamespace(kind="DISARM", nonzero=False, generation=0, expires_ns=None)
    assert bringup._cli_gate(holder, disarm, lambda: calls.append(6))
    assert calls == [1, 5, 6]


def test_a_refused_first_hold_is_written_once_and_closed_with_a_written_zero(mcu_model, monkeypatch):  # noqa: F811
    """The ARM taken, then M pushed to raw 0 before the first HOLD: the board refuses it (sensor_invalid); exactly one
    BENCH_HOLD is written, then a written zero on M and the cleanup (plan 4.3 decision 10)."""
    import io
    import signal

    from test_bringup_commission_tool import Model, PtyLink

    m = Model(mcu_model, "--bench", "1", "--kth-start-raw", "256,256", "--axis-push-steer-rad", "-0.3926990817",
              commission=True)
    try:
        b = bringup.Bench(PtyLink(m.master), io.StringIO(), allow_locked=False, commission=True)
            orig = bringup.CommissionSession.confirm_bench_arm

        def confirm_then_push(self):
            orig(self)
            m.proc.send_signal(signal.SIGUSR1)

        monkeypatch.setattr(bringup.CommissionSession, "confirm_bench_arm", confirm_then_push)
        res = bringup.execute(hargs(hold="here", seconds=2.0), b)
        assert res["hold"]["start"] == "refused" and res["aborted"] == "bench_start_refused_sensor_invalid", res
        holds = [w for w in b.writes if w.kind == "BENCH_HOLD"]
        assert len(holds) == 1, holds
        later = [w for w in b.writes if w.order > holds[0].order and w.kind == "COMMISSION"]
        assert later and later[0].duty_e4 == 0 and later[0].axis == 2, later
    finally:
        m.close()


def test_the_holds_stop_before_the_end_zero(model):
    """The end timeline (plan 4.3 decision 10): the BENCH_HOLDs end before the end zero, which the board takes as
    end_zero; no HOLD is written after it."""
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    res = bringup.execute(hargs(hold="here", seconds=1.4), b)
    assert res["bench_result"]["stop_reason"] == "end_zero", res
    end = next(w for w in b.writes if w.kind == "COMMISSION" and w.phase == "post_zero")
    assert not [w for w in b.writes if w.kind == "BENCH_HOLD" and w.order > end.order]


def test_the_cli_link_follows_the_boards_image():
    """bench7 gets the gated link whatever the run's arguments; another image keeps the plain TtyLink (Codex A1b-2b
    round 1 P1)."""
    import os

    from real_mcu_link import BenchTtyLink, TtyLink

    holder = {"host": None, "cancelled": False, "hold_until_ns": None}
    r, w = os.pipe()
    try:
        assert isinstance(bringup._link_factory(holder, lambda fd: BENCH5)(w), BenchTtyLink)
        other = bringup._link_factory(holder, lambda fd: "forklift-mcu commission OPEN v rec=1 lim=lr1")(w)
        assert type(other) is TtyLink
        assert type(bringup._link_factory(holder, lambda fd: None)(w)) is TtyLink
    finally:
        os.close(r)
        os.close(w)


def test_the_cli_gate_stops_holds_at_the_renewal_end():
    import time
    from types import SimpleNamespace

    now = time.monotonic_ns()
    holder = {"host": SimpleNamespace(auth_epoch=1), "cancelled": False, "hold_until_ns": now - 1}
    hold = SimpleNamespace(kind="BENCH_HOLD", nonzero=True, generation=1, expires_ns=now + 10**9)
    zero = SimpleNamespace(kind="COMMISSION", nonzero=False, generation=1, expires_ns=now + 10**9)
    assert not bringup._cli_gate(holder, hold, lambda: None)  # past T - HANDOVER_S: never completed before the zero
    assert bringup._cli_gate(holder, zero, lambda: None)


def test_one_product_read_picks_the_link_and_tells_the_host():
    """A product read that changes between two reads can never give the host bench7 on a plain link (Codex A1b-2b
    round 2 P2): the factory's read is the value the host gets."""
    import os

    from real_mcu_link import TtyLink

    reads = iter([None, BENCH5])
    holder = {"host": None, "cancelled": False, "hold_until_ns": None}
    reader = lambda fd: next(reads)  # noqa: E731
    r, w = os.pipe()
    try:
        link = bringup._link_factory(holder, reader)(w)
        assert type(link) is TtyLink
        assert bringup._product_of_opening(holder, reader)(w) is None  # the same value, not the second read
    finally:
        os.close(r)
        os.close(w)


def test_a_hold_whose_settings_cannot_be_written_sends_no_hold(model, monkeypatch):
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    orig = bringup.Bench.emit

    def emit(self, row):
        if row.get("t") == "hold_settings":
            self.record_errors += 1
            return
        return orig(self, row)

    monkeypatch.setattr(bringup.Bench, "emit", emit)
    res = bringup.execute(hargs(hold="here", seconds=1.4), b)
    assert res["aborted"] == "hold_settings_not_recorded", res
    assert "BENCH_HOLD" not in kinds_written(b)


def test_a_window_stop_closes_the_hold_gate(model):
    """A hold stopped early by its window: the gate refuses any HOLD from then on, so none completes behind the end
    zero (Codex A1b-2b round 2 P2)."""
    import time

    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    b.gate_holder = {"host": b.host, "cancelled": False, "hold_until_ns": None}
    res = bringup.execute(hargs(hold="rel:200", seconds=3.0, bench_bypass=None, window=(-0.1, 0.05)), b)
    assert res["stop"] == "window", res
    assert b.gate_holder["hold_until_ns"] is not None and b.gate_holder["hold_until_ns"] <= time.monotonic_ns()


def test_a_recovered_hold_result_is_undecidable():
    from forklift_core.mcu_link.messages import BenchRunFlag, BenchState

    bs = BenchState(1, 5, 5, 2, 1, 1, 0, 100, 990, 1000, 0, 995, 1, 0, 0, 0, 5, 5, 0, 0x23, 0,
                    int(BenchRunFlag.RES_HOLD), 2, 1, 3, 1)
    f = bringup.recovered_fields(bs)
    assert f["kind"] == "hold" and f["target"] == 1000 and f["result_bypass"] == ["steer_range"]
    assert f["hold_verdict"]["result"] == "undecidable"
    assert "hold_verdict" not in bringup.result_fields(bs)


def test_the_boards_own_end_confirms_the_cleanup_when_the_disarm_echo_is_lost(model, monkeypatch):
    """This run's ENDED then DISARMED pair of the arming that named it: the cleanup is confirmed without the DISARM's
    echo (plan 3 verdict 1; Codex A1b-2b rounds 2-3)."""
    b = model("--bench", "1", "--kth-start-raw", "1000,1000")
    orig = bringup.CommissionSession.cleanup

    def cleanup_echo_lost(self):
        orig(self)
        self.cleanup_confirmed = False  # as if the DISARM's echo never came

    monkeypatch.setattr(bringup.CommissionSession, "cleanup", cleanup_echo_lost)
    res = bringup.execute(hargs(hold="here", seconds=1.4), b)
    assert res["hold"]["verdict1"]["ok"] and res["result"] == "ok", res
    assert [r for r in rows(b) if r["t"] == "cleanup_confirmed_by_board"]


def test_the_run_pairs_keep_the_confirmed_arming():
    from types import SimpleNamespace as NS

    pairs = [(0, Mode.DISARMED, NS(run_id=0)), (10, Mode.ARMED, NS(run_id=5)), (20, Mode.ARMED, NS(run_id=5))]
    idents = [(1, 7), (1, 8), (2, 8)]  # the last pair: another boot
    kept = bringup._run_pairs(pairs, idents, (1, 8))
    assert [k[0] for k in kept] == [0, 10] and kept[1] is pairs[1]
    assert kept[0][2].run_id == 0  # the earlier arming: its time only
    # a run id reused by an earlier arming of the same boot never mixes its count in (Codex A1b-2b round 4 P3)
    from forklift_core.plant import bench_session as bs_

    pairs = [(0, Mode.ARMED, NS(run_id=5, deadband_entries=24)), (100_000, Mode.DISARMED, NS(run_id=5,
             deadband_entries=24)), (200_000, Mode.ARMED, NS(run_id=5, deadband_entries=8))]
    kept = bringup._run_pairs(pairs, [(1, 7), (1, 7), (1, 8)], (1, 8))
    w = bs_.entry_window(bs_.entry_points(kept, 5), 200_000)
    assert w["lower"] == 8 and w["label"] == "none", w


def test_the_cli_gate_refuses_a_hold_at_its_stop_time_itself(monkeypatch):
    from types import SimpleNamespace

    t = 5_000_000_000
    monkeypatch.setattr(bringup.time, "monotonic_ns", lambda: t)
    holder = {"host": SimpleNamespace(auth_epoch=1), "cancelled": False, "hold_until_ns": t}
    hold = SimpleNamespace(kind="BENCH_HOLD", nonzero=True, generation=1, expires_ns=t + 10**9)
    assert not bringup._cli_gate(holder, hold, lambda: None)  # >=: the same instant is past the stop
