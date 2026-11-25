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


def _runner(model, tmp_path, bridge, duration_s, model_args=()):
    return Runner(model, tmp_path, duration_s=duration_s, model_args=model_args,
                  extra_args=[f"--hil-slam-bridge={bridge.path}"])


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
        c.close()
        code = run.finish(signal.SIGTERM)
        bridge.stop()
    rows = run.rows()
    end = complete(rows, code)
    assert [e["reason"] for e in end["results"]] == ["s10_lifted"], end
    assert end["closing"] == "confirmed" and bridge.error is None
    kinds = [g[1] for g in bridge.got]
    assert kinds[0] == "Hello" and kinds.count("RealScan") > 7 and "Seed" not in kinds
    assert ("ready", 1) in [(k, i) for _, k, i in bridge.sent]
    assert "slam_late" not in end["slam_counts"] or end["slam_counts"]["slam_late"] < kinds.count("RealScan")


def test_a_declaration_on_the_real_source_waits_its_seed_survives_a_stop_and_caps_the_run(mcu_model, tmp_path):
    bridge = _bridge(tmp_path, seed_delay_s=3.0)
    run = _runner(mcu_model, tmp_path, bridge, 200, model_args=("--state-silent-at-ms", str(SILENT_AT_MS),
                                                               "--state-silent-ms", str(SILENT_MS)))
    c = Console(run.port)
    try:
        _lease(c)
        s = c.wait(lambda s: s.get("relocalize_generation") is not None and s.get("standing"), 40)
        gen = s["relocalize_generation"]
        # the first declaration: its seed held 3 s by the bridge; a STOP ends the attempt, the seed runs on
        r = c.ask("relocalize_declare", generation=gen)
        assert r["t"] == "ack", r
        time.sleep(1.0)
        r = c.ask("stop")
        assert r["t"] == "ack", r
        s = c.wait(lambda s: s["relocalize"]["state"] == "refused", 10)
        # the STOP ends the attempt (its control generation, or the stand it broke)
        assert s["relocalize"]["reason"] in ("relocalize_moved", "relocalize_voided:control"), s["relocalize"]
        c.wait(lambda s: s.get("standing"), 10)
        r = c.ask("rearm")
        assert r["t"] == "ack", r
        c.wait(lambda s: s.get("mode") == "PAUSED" and s.get("standing"), 10)
        r = c.ask("relocalize_declare", generation=gen)
        assert r["t"] == "reject" and r["reason"] == "seed_in_progress", r  # one seed at a time
        # the first seed's end state completes nothing: the generation stays open
        end_wait = time.monotonic() + 15
        while not any(k == "ready" and i == 2 for _, k, i in bridge.sent) and time.monotonic() < end_wait:
            time.sleep(0.1)
        time.sleep(0.5)
        assert c.status["relocalize_generation"] == gen and c.status["relocalize"]["state"] == "refused"
        # the second declaration completes on its own seed's ready
        r = c.ask("relocalize_declare", generation=gen)
        assert r["t"] == "ack", r
        s = c.wait(lambda s: s["relocalize"]["state"] != "waiting", 30)
        assert s["relocalize"]["state"] == "declared", s["relocalize"]
        assert s["relocalize_generation"] is None and "relocalize" in s["bypassed"] and s["manual_only"] is None
        _mission(c)
    finally:
        c.close()
        code = run.finish(signal.SIGTERM)
        bridge.stop()
    rows = run.rows()
    end = complete(rows, code)
    assert [e["reason"] for e in end["results"]] == ["s10_lifted"], end
    assert bridge.error is None
    seeds = [g for g in bridge.got if g[1] == "Seed"]
    assert len(seeds) == 2 and not any(g[1] == "Cancel" for g in bridge.got)
    moving = [q["speed_mps"] for q in events(rows, "frame_queued") if q["purpose"] == "drive" and q["speed_mps"]]
    assert moving and max(abs(v) for v in moving) <= 0.06 + 1e-9  # the declared session's cap
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
        code = run.finish(signal.SIGTERM, timeout=60)
        bridge.stop()
    rows = run.rows()
    end = end_of(rows)
    assert "slam" in str(end.get("reason")) and "fault" in str(end.get("reason")), end.get("reason")
    assert code != 0 and end["closing"] == "confirmed", (code, end.get("closing"))
