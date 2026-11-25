"""PR 7 7r-2 (docs/plans/2026-11-26-7r-replay-run.md 4-5): the replay run through its command line -- a synthetic
observe record (tests/fixtures/replay, TEST ONLY) replayed behind the MCU model's HIL run on the real SLAM source and
a protocol-following fake bridge (``--run --hil --hil-slam-bridge --replay``).

The record is from "another day" (its host clock ten days ahead of this one): the scans reach the bridge stamped on
this run's own clock. From a non-zero start the session's start generation is confirmed on the record's start (the
bridge's hello carries it); the run ends by itself once both streams are through (``replay_eof``) and its head, anchor
and end carry the replay's marks and counts. F1: a declaration ended by a STOP while its seed runs on to ready, then a
confirmation on that seed's instance -- the session's last seed is no completed declaration: the F1 start block.

The fake bridge does no matching: this is the wiring on the wall clock, not evidence of slam_toolbox on a replay.
CPU: as test_real_run_hil_slam.py (4 CPUs or more; a failing assertion says why from the run's record)."""

from __future__ import annotations

import signal
import sys
import time
from pathlib import Path

import pytest

pytest.importorskip("websockets")

from fake_slam_bridge import FakeBridge  # noqa: E402
from test_real_run_hil import Console, Runner, complete, end_of, events, explained, mcu_model  # noqa: E402,F401
from test_real_run_hil_slam import _confirm_start, _lease, _rearm, _sent, _until  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tests" / "fixtures" / "replay"))
import run_inputs as ri  # noqa: E402

DAY_NS = 86_400 * 10**9


def _inputs(tmp_path):
    return ri.write_run_inputs(tmp_path / "inputs", t0_ns=time.monotonic_ns() + 10 * DAY_NS)


def _run(model, tmp_path, bridge, inp, duration_s):
    extra = [f"--hil-slam-bridge={bridge.path}", f"--config={inp.s10a}", f"--replay={inp.record}",
             f"--lidar-config={inp.lidar}", f"--imu-config={inp.imu}", "--allow-synthetic"]
    return Runner(model, tmp_path, duration_s=duration_s, extra_args=extra)


def _end(run, c, bridge, wait_s=0.0):
    """Waits up to ``wait_s`` for the run to end by itself, then ends it (SIGTERM) if it has not. The bridge is told
    first: a run ending on replay_eof closes its connection as the end of the run, not as a broken one."""
    bridge.closing()
    end = time.monotonic() + wait_s
    while run.proc.poll() is None and time.monotonic() < end:
        time.sleep(0.2)
    c.close()
    code = run.finish(signal.SIGTERM if run.proc.poll() is None else None)
    bridge.stop()
    return code


def test_a_record_from_another_day_confirms_its_start_and_ends_on_its_own_eof(mcu_model, tmp_path):
    from forklift_core.localization.start_reloc import base_from_rear

    inp = _inputs(tmp_path)
    bridge = FakeBridge(tmp_path / "slam.sock")
    bridge.start()
    run = _run(mcu_model, tmp_path, bridge, inp, 150)
    with explained(run):
        c = Console(run.port)
        try:
            _lease(c)
            conf = _confirm_start(c, timeout=40)
            assert conf is not None and conf["state"] == "confirmed", conf
            asked_scans = len(bridge.stamps)
        finally:
            code = _end(run, c, bridge, wait_s=90)
        rows = run.rows()
        end = complete(rows, code)
        assert end["reason"] == "replay_eof" and end["closing"] == "confirmed", end.get("reason")
        assert bridge.error is None
        # the record's start on the bridge's hello (non-zero x, y, yaw)
        want = base_from_rear(inp.start_rear, inp.rear_offset_m)
        assert bridge.poses[1] == pytest.approx(tuple(want), abs=1e-9) and abs(want[2]) > 0.1
        # every scan stamped on this run's clock (the record's is ten days ahead): received within a second of it
        assert bridge.stamps and all(0 <= recv - stamp < 10**9 for recv, stamp in bridge.stamps), bridge.stamps[:3]
        assert len(bridge.stamps) > asked_scans  # the replay went on after the confirmation
        # the head, the anchor and the end
        [head] = events(rows, "head")
        assert head["replayed_from"]["sha256"] == (inp.record.parent / "observe.jsonl.ok").read_text().strip()
        assert head["synthetic_inputs"] is True and head["not_for_judgement"] is True
        [anchor] = events(rows, "replay_anchor")
        assert anchor["record_anchor_ns"] == inp.info["anchor_ns"]
        assert anchor["delta_ns"] == anchor["hil_anchor_ns"] - inp.info["anchor_ns"] and anchor["delta_ns"] < -DAY_NS
        # the stamp on the socket is the record's first-beam time moved by delta, to the nanosecond (the record's
        # scans start at t0 every 100 ms; Codex 7r-2 round 1 P3)
        period = int(1e9 / 10.0)
        assert all((stamp - anchor["delta_ns"] - inp.info["t0_ns"]) % period == 0 for _r, stamp in bridge.stamps)
        # and never a period early: a scan reaches the socket after its last beam (98 ms after its first)
        assert all(recv - stamp >= 98 * 10**6 for recv, stamp in bridge.stamps)
        rep = end["replay"]
        assert rep["scan_fatal"] is None and rep["scan"]["delivered"] > 100 and rep["eof_ns"] is not None
        assert rep["scan"].get("replay_mismatch", 0) == 0 and rep["imu"] is not None
        confirmed = [e for e in end["reloc_events"] if e["event"] == "confirmed"]
        assert confirmed and end["f1_block"] is None


def test_a_stopped_declaration_whose_seed_runs_to_ready_then_a_confirmation_blocks_f1(mcu_model, tmp_path):
    inp = _inputs(tmp_path)
    bridge = FakeBridge(tmp_path / "slam.sock", seed_delay_s=1.0, hold_end=(2,))
    bridge.start()
    run = _run(mcu_model, tmp_path, bridge, inp, 150)
    with explained(run):
        c = Console(run.port)
        try:
            _lease(c)
            gen = c.status["relocalize_generation"]
            assert gen is not None
            _until(lambda: _sent(bridge, "ready", 1), 40, "the session start's instance ready")
            c.wait(lambda s: s.get("standing"), 10)
            r = c.ask("relocalize_declare", generation=gen)
            assert r["t"] == "ack", r
            a1 = r["attempt"]
            _until(lambda: _sent(bridge, "seeding", 2), 10, "seed 2 seeding")
            r = c.ask("stop")
            assert r["t"] == "ack", r
            s = c.wait(lambda s: s["relocalize"]["attempt"] == a1 and s["relocalize"]["state"] == "refused", 10)
            assert s["relocalize"]["reason"] in ("relocalize_moved", "relocalize_voided:control"), s["relocalize"]
            bridge.release(2)
            _until(lambda: _sent(bridge, "ready", 2), 15, "seed 2 ready")
            c.wait(lambda s: s.get("standing"), 10)
            r = _rearm(c)
            assert r["t"] == "ack", r
            c.wait(lambda s: s.get("mode") == "PAUSED" and s.get("standing"), 10)
            conf = _confirm_start(c, timeout=30)  # on seed 2's instance: the operator's attempt itself was stopped
            assert conf is not None and conf["state"] == "confirmed", conf
            s = c.wait(lambda s: s.get("f1_block") is not None, 5)
            assert s["f1_block"]["source_kind"] is not None
        finally:
            code = _end(run, c, bridge)
        rows = run.rows()
        end = end_of(rows)
        assert end["closing"] == "confirmed" and bridge.error is None, end.get("reason")
        assert [g[2] for g in bridge.got if g[1] == "Seed"] == [2]
        assert not any(row.get("declared") for row in end["seed_log"])
        assert end["f1_block"] is not None
