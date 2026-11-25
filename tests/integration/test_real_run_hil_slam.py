"""PR 7 7c-1b-2b (docs/design/2026-11-25-real-sources-pr7.md 6.1, 9 'pty HIL'): the HIL run through its command line
with the real SLAM source on a protocol-following fake bridge (``--hil-slam-bridge``) -- the fake bridge server ->
``RealSlamSource`` -> ``_SlamGuard`` -> ``RealLoop`` path on the MCU model behind the runner's pty. S10-A to the lift
on the session start's instance; a declaration whose seed the bridge holds for seconds, a STOP while it runs (the
attempt ends, the seed runs to its end state and completes nothing, a second declaration is refused until then),
the declared run capped at 0.06 m/s; a bridge fault ends the run as a source fault. The scope is the wiring and the
contract on the wall clock: the fake bridge does no matching."""

from __future__ import annotations

import signal
import time

import pytest

pytest.importorskip("websockets")

from fake_slam_bridge import FakeBridge  # noqa: E402
from test_real_run_hil import Console, Runner, complete, end_of, events, mcu_model  # noqa: E402,F401

SILENT_AT_MS, SILENT_MS = 9000, 400  # a same-boot STATE gap past the odometry's 0.2 s: a relocalization generation


def _bridge(tmp_path, **kw):
    b = FakeBridge(tmp_path / "slam.sock", **kw)
    b.start()
    return b


def _runner(model, tmp_path, bridge, duration_s, model_args=(), config=None):
    extra = [f"--hil-slam-bridge={bridge.path}"] + ([f"--config={config}"] if config is not None else [])
    return Runner(model, tmp_path, duration_s=duration_s, model_args=model_args, extra_args=extra)


FAR_PALLET = [3.6, 0.0, 0.0]


def _far_config(tmp_path):
    """The HIL file with the pallet 1 m further (Codex 7c-1b-2b round 2): the approach then has about 1.9 m of
    straight run before the depth-frame wait at 1.0 m, where the d5_pocket bypass is not needed -- so a mission moves
    with no bypass and the declaration's cap can be told from the bypass's."""
    from test_real_run_hil import HIL_YAML

    text = HIL_YAML.read_text()
    assert "check_pallet_pose: [2.6, 0.0, 0.0]" in text
    out = tmp_path / "s10a_hil_far.yaml"
    out.write_text(text.replace("check_pallet_pose: [2.6, 0.0, 0.0]", f"check_pallet_pose: {FAR_PALLET}"))
    return out


def _operator_start(c, timeout=15.0):
    """The start check: the SLAM evidence of the current epoch may still be on its way (the session start's seven
    scans, a seed's start check) -- asked again while the answer is slam_stale."""
    end = time.monotonic() + timeout
    while True:
        r = c.ask("operator_start")
        if r["t"] == "ack" or r.get("reason") != "slam_stale" or time.monotonic() > end:
            return r
        time.sleep(0.3)


def _mission(c):
    r = c.ask("check_bypass", check="d5_pocket", on=True)
    assert r["t"] == "ack", r
    c.wait(lambda s: "d5_pocket" in s["bypassed"], 5)
    r = c.ask("pallet_register", pallet_id="P-S10", pose=[2.6, 0.0, 0.0])
    assert r["t"] == "ack", r
    time.sleep(0.8)  # stands
    r = _operator_start(c)
    assert r["t"] == "ack", r
    r = c.ask("mission_submit", pallet_id="P-S10", destination_id="D-S10",
              targets_revision=r.get("targets_revision", c.status["targets_revision"]))
    assert r["t"] == "ack", r
    s = c.wait(lambda s: s["operator_request"] is not None or s["permit"]["state"] == "LATCHED", 120)
    op = s["operator_request"]
    assert op is not None, s
    r = c.ask("operator_answer", kind=op["kind"], serial=op["serial"], verdict="confirm")
    assert r["t"] == "ack", r
    c.wait(lambda s: s["permit"]["state"] == "LATCHED", 120)


def _lease(c):
    c.wait(lambda s: s["permit"]["state"] == "OFF" and s.get("standing"), 15)
    lease = c.ask("lease_request")
    assert lease["t"] == "ack", lease
    c.lease = lease["lease_id"]


def test_s10_a_runs_to_the_lift_on_the_real_slam_source(mcu_model, tmp_path):
    bridge = _bridge(tmp_path)
    run = _runner(mcu_model, tmp_path, bridge, 150)
    c = Console(run.port)
    try:
        _lease(c)
        _mission(c)
    finally:
        code = _finish(run, c, bridge)
    rows = run.rows()
    end = complete(rows, code)
    assert [e["reason"] for e in end["results"]] == ["s10_lifted"], end
    assert end["closing"] == "confirmed" and bridge.error is None
    kinds = [g[1] for g in bridge.got]
    assert kinds[0] == "Hello" and kinds.count("RealScan") > 7 and "Seed" not in kinds
    assert ("ready", 1) in [(k, i) for _, k, i in bridge.sent]
    assert "slam_late" not in end["slam_counts"] or end["slam_counts"]["slam_late"] < kinds.count("RealScan")


def _sent(bridge, kind, ident):
    return any(k == kind and i == ident for _, k, i in bridge.sent)


def _until(cond, timeout, what):
    end = time.monotonic() + timeout
    while not cond():
        if time.monotonic() > end:
            raise AssertionError(f"not reached: {what}")
        time.sleep(0.05)


def _written_moving(rows):
    """The moving DRIVEs written whole (their queued speed by seq)."""
    speed = {q["seq"]: q["speed_mps"] for q in events(rows, "frame_queued") if q["purpose"] == "drive"}
    return [speed[w["seq"]] for w in events(rows, "frame_written")
            if w["kind"] == "DRIVE" and speed.get(w["seq"]) not in (None, 0.0)]


def _approach(c, seconds):
    """A mission with no bypass on the far pallet, driven for ``seconds``: the approach's straight run is judged."""
    r = c.ask("pallet_register", pallet_id="P-S10", pose=FAR_PALLET)
    assert r["t"] == "ack", r
    time.sleep(0.8)
    r = _operator_start(c)
    assert r["t"] == "ack", r
    r = c.ask("mission_submit", pallet_id="P-S10", destination_id="D-S10",
              targets_revision=r.get("targets_revision", c.status["targets_revision"]))
    assert r["t"] == "ack", r
    time.sleep(seconds)


def _rearm(c, timeout=5.0):
    """The rearm after the STOP: the console's status may still carry the control generation from before the STOP
    (stale_control) -- asked again with the next status."""
    end = time.monotonic() + timeout
    while True:
        r = c.ask("rearm")
        if r["t"] == "ack" or r.get("reason") != "stale_control" or time.monotonic() > end:
            return r
        time.sleep(0.2)


def _finish(run, c, bridge):
    c.close()
    bridge.closing()
    code = run.finish(signal.SIGTERM)
    bridge.stop()
    return code


def test_an_undeclared_approach_on_the_real_source_drives_past_the_declared_cap(mcu_model, tmp_path):
    """The comparison for the declared run below: no declaration and no bypass -- the approach goes above 0.06."""
    bridge = _bridge(tmp_path)
    run = _runner(mcu_model, tmp_path, bridge, 90, config=_far_config(tmp_path))
    c = Console(run.port)
    try:
        _lease(c)
        _approach(c, 12.0)
    finally:
        code = _finish(run, c, bridge)
    rows = run.rows()
    end = complete(rows, code)
    assert end["reason"] == "signal" and end["closing"] == "confirmed" and bridge.error is None, end.get("reason")
    moving = _written_moving(rows)
    assert moving and max(abs(v) for v in moving) > 0.06 + 1e-9, moving[:5]


def test_declarations_on_the_real_source_through_a_lease_loss_and_a_stop(mcu_model, tmp_path):
    """A generation opened by a 400 ms STATE silence; the bridge holds every reply 0.09 s, so a seed accepted with a
    scan in flight gets the stopped instance's reply after it -- dropped, never evidence (three seeds: at least one).
    Declaration 1: its seed held 3 s, the lease released while the bridge reports seeding -- the attempt ends, the
    seed runs to ready and completes nothing. Declaration 2: a STOP while its seed runs -- refused; after the rearm a
    third declaration is refused seed_in_progress (the bridge holds that seed's end state until it is seen), the
    seed's ready then completes nothing. Declaration 3 completes on its own seed; the approach after it, with no
    bypass, writes every moving DRIVE within 0.06 m/s (above it without the declaration: the test before)."""
    bridge = _bridge(tmp_path, seed_delay_s=3.0, reply_delay_s=0.09, hold_end=(3,))
    run = _runner(mcu_model, tmp_path, bridge, 200, model_args=("--state-silent-at-ms", str(SILENT_AT_MS),
                                                               "--state-silent-ms", str(SILENT_MS)),
                  config=_far_config(tmp_path))
    c = Console(run.port)
    try:
        _lease(c)
        s = c.wait(lambda s: s.get("relocalize_generation") is not None and s.get("standing"), 40)
        gen = s["relocalize_generation"]
        # 1: the lease released while the seed runs
        r = c.ask("relocalize_declare", generation=gen)
        assert r["t"] == "ack", r
        _until(lambda: _sent(bridge, "seeding", 2), 10, "seed 2 seeding")
        r = c.ask("lease_release")
        assert r["t"] == "ack", r
        c.lease = None
        s = c.wait(lambda s: s["relocalize"]["attempt"] == 1 and s["relocalize"]["state"] == "refused", 10)
        assert s["relocalize"]["reason"] == "relocalize_voided:lease", s["relocalize"]
        _until(lambda: _sent(bridge, "ready", 2), 15, "seed 2 ready")
        time.sleep(0.5)
        assert c.status["relocalize_generation"] == gen and c.status["relocalize"]["state"] == "refused"
        _lease(c)
        # 2: a STOP while the seed runs; its end state held until the second declaration has been refused
        r = c.ask("relocalize_declare", generation=gen)
        assert r["t"] == "ack", r
        _until(lambda: _sent(bridge, "seeding", 3), 10, "seed 3 seeding")
        r = c.ask("stop")
        assert r["t"] == "ack", r
        s = c.wait(lambda s: s["relocalize"]["attempt"] == 2 and s["relocalize"]["state"] == "refused", 10)
        assert s["relocalize"]["reason"] in ("relocalize_moved", "relocalize_voided:control"), s["relocalize"]
        c.wait(lambda s: s.get("standing"), 10)
        r = _rearm(c)
        assert r["t"] == "ack", r
        c.wait(lambda s: s.get("mode") == "PAUSED" and s.get("standing"), 10)
        r = c.ask("relocalize_declare", generation=gen)
        assert r["t"] == "reject" and r["reason"] == "seed_in_progress", r  # one seed at a time
        bridge.release(3)
        _until(lambda: _sent(bridge, "ready", 3), 15, "seed 3 ready")
        time.sleep(0.5)
        assert c.status["relocalize_generation"] == gen and c.status["relocalize"]["state"] == "refused"
        # 3: completes on its own seed
        r = c.ask("relocalize_declare", generation=gen)
        assert r["t"] == "ack", r
        s = c.wait(lambda s: s["relocalize"]["attempt"] == r["attempt"] and s["relocalize"]["state"] != "waiting", 30)
        assert s["relocalize"]["state"] == "declared", s["relocalize"]
        assert s["relocalize_generation"] is None and "relocalize" in s["bypassed"] and s["manual_only"] is None
        assert "d5_pocket" not in s["bypassed"] and "slam" not in s["bypassed"]
        _approach(c, 12.0)
    finally:
        code = _finish(run, c, bridge)
    rows = run.rows()
    end = complete(rows, code)
    assert end["reason"] == "signal" and end["closing"] == "confirmed", end.get("reason")
    assert bridge.error is None
    seeds = [g for g in bridge.got if g[1] == "Seed"]
    assert [g[2] for g in seeds] == [2, 3, 4] and not any(g[1] == "Cancel" for g in bridge.got)
    moving = _written_moving(rows)
    assert moving and max(abs(v) for v in moving) <= 0.06 + 1e-9  # the declaration's own cap (no bypass)
    assert end["slam_counts"].get("slam_old_instance_reply", 0) >= 1, end["slam_counts"]  # old replies dropped
    assert end["slam_counts"].get("slam_end_after_cancel", 0) == 0, end["slam_counts"]


def test_a_bridge_fault_ends_the_run_as_a_source_fault(mcu_model, tmp_path):
    bridge = _bridge(tmp_path, fault_after=20)
    run = _runner(mcu_model, tmp_path, bridge, 60)
    c = Console(run.port)
    try:
        _lease(c)
        end_wait = time.monotonic() + 30
        while not any(k == "fault" for _, k, _i in bridge.sent) and time.monotonic() < end_wait:
            time.sleep(0.1)
        time.sleep(1.0)
    finally:
        c.close()
        bridge.closing()
        code = run.finish(signal.SIGTERM, timeout=60)
        bridge.stop()
    rows = run.rows()
    end = end_of(rows)
    assert "slam" in str(end.get("reason")) and "fault" in str(end.get("reason")), end.get("reason")
    assert code != 0 and end["closing"] == "confirmed", (code, end.get("closing"))
