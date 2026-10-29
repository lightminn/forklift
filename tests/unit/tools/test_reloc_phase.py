"""The runner's start relocalization phase (sim/isaac/reloc_phase.py, docs/plans/2026-10-29-start-relocalization.md
1.2 steps 3-10) on a fake engine: settle and physical validity, stationary scans, the matcher on a worker thread
under one session deadline, rescans, the estimate file and the wait for the supervisor's bridge_go."""

import json
import math
import sys
import threading
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))
sys.path.insert(0, str(ROOT / "tests" / "unit" / "localization"))

import reloc_phase as P  # noqa: E402
import scan_capture as C  # noqa: E402
import test_global_match as TG  # noqa: E402

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import start_reloc as R  # noqa: E402


class FakeTruck:
    """A truck standing at ``base`` in ``grid``; its LiDAR (0.12 m behind base) reads the grid."""

    def __init__(self, grid, base, drift=(0.0, 0.0)):
        self.grid, self.base, self.drift, self.steps = grid, base, drift, 0

    def step(self):
        self.steps += 1

    def pose(self):
        x, y, yaw = self.base
        return np.array([x + self.drift[0], y + self.drift[1], 0.1]), C.quat_from_yaw(yaw)

    def cast(self, pos, q):
        yaw = C.yaw_roll_pitch(q)[0]
        laser = (pos[0] - 0.12 * math.cos(yaw), pos[1] - 0.12 * math.sin(yaw), yaw)
        ranges, _, _ = TG._scan(self.grid, laser, beams=360)
        return np.asarray(ranges, float), np.ones(len(ranges), bool)


def _inputs(tmp_path, grid, truck, target, **kw):
    engine = C.Engine(teleport=None, step=truck.step, pose=truck.pose, cast=truck.cast,
                      draw_noise=lambda rng, d: np.zeros(len(d)), ranges_from_hits=lambda d, h: d,
                      range_min_m=0.2, range_max_m=12.0, settle_z_m=0.1, laser_xyz_m=(-0.12, 0.0, 1.05))
    cfg = G.MatchConfig(yaw_step_rad=math.radians(2.0))
    defaults = dict(engine=engine, target=target, grid=grid, index=G.MapIndex(grid, cfg), match_cfg=cfg,
                    reloc_cfg=R.RelocConfig(e_min=0.8, r_max=0.9, r_struct=0.8), angle_min_rad=0.0,
                    angle_increment_rad=2 * math.pi / 360, epoch_dir=tmp_path, settle_steps=3,
                    bridge_go_timeout_s=5.0, pace_s=0.0)
    defaults.update(kw)
    return P.RelocInputs(**defaults)


def _go_later(tmp_path, delay=0.2, wait_s=10.0):
    def write():
        end = time.monotonic() + wait_s
        while R.read_reloc_pose(tmp_path) is None and time.monotonic() < end:
            time.sleep(0.02)
        time.sleep(delay)
        R.write_bridge_go(tmp_path, {"map_start_pose": R.read_reloc_pose(tmp_path)["base"]})

    threading.Thread(target=write, daemon=True).start()


def test_a_confirmable_start_writes_its_estimate_and_waits_for_the_bridge(tmp_path):
    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    truck = FakeTruck(grid, base)
    _go_later(tmp_path)
    out = P.run(_inputs(tmp_path, grid, truck, base))
    assert out["status"] == "chosen" and out["bridge_go"]["map_start_pose"] == out["estimate"]
    assert math.hypot(out["estimate"][0] - base[0], out["estimate"][1] - base[1]) <= 0.10
    saved = R.read_reloc_pose(tmp_path)
    assert saved["source"] == "matcher" and saved["base"] == out["estimate"]
    assert out["attempts"][0]["reason"] == "confirmed" and out["truth"] == pytest.approx(base, abs=1e-9)
    assert truck.steps >= 3 + 4 * 12  # the settle and the scans a LiDAR period apart


def test_a_truck_that_did_not_settle_at_its_target_is_invalid_before_any_matching(tmp_path):
    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base, drift=(0.05, 0.0)), base))
    assert out["status"] == "invalid" and out["reason"] == "not_at_target" and out["attempts"] == []
    assert R.read_reloc_pose(tmp_path) is None


def test_an_ambiguous_unattended_start_needs_the_operator_after_its_rescans(tmp_path):
    grid = TG._grid(6.0, 4.0, [])  # a symmetric room: the turned twin always competes
    base = (2.0, 1.4, 0.0)
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base), base,
                        match_cfg=G.MatchConfig(yaw_step_rad=math.radians(2.0), max_nodes=5_000_000)))
    assert out["status"] in ("needs_operator", "failed") and len(out["attempts"]) == 3
    assert all(a["reason"] != "confirmed" for a in out["attempts"]) and R.read_reloc_pose(tmp_path) is None


def test_no_bridge_go_in_time_is_a_bridge_timeout(tmp_path):
    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base), base, bridge_go_timeout_s=0.3))
    assert out["status"] == "bridge_timeout" and R.read_reloc_pose(tmp_path) is not None


def test_a_cancel_ends_the_phase_without_an_estimate(tmp_path):
    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    cancel = threading.Event()
    cancel.set()
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base), base, cancel=cancel))
    assert out["status"] == "cancelled" and R.read_reloc_pose(tmp_path) is None


def test_the_epoch_record_names_every_attempt(tmp_path):
    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    _go_later(tmp_path)
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base), base))
    record = R.read_reloc_pose(tmp_path)["record"]
    assert record["attempts"][0]["reason"] == "confirmed" and "wall_s" in record["attempts"][0]
    assert json.dumps(out)  # the result is what the runner records: plain JSON


def test_each_attempt_draws_its_own_noise(tmp_path):
    grid = TG._grid(6.0, 4.0, [])
    base = (2.0, 1.4, 0.0)
    asked = []
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base), base,
                        match_cfg=G.MatchConfig(yaw_step_rad=math.radians(2.0), max_nodes=5_000_000),
                        noise_for=lambda a, s: asked.append((a, s))))
    assert len(out["attempts"]) == 3
    assert asked == [(a, s) for a in range(3) for s in range(C.SCANS)]  # a static rescan: new noise, same place


@pytest.mark.parametrize("out, reason", [
    ({"status": "chosen"}, None),
    ({"status": "invalid", "reason": "moved"}, "reloc_invalid_input: moved"),
    ({"status": "needs_operator", "shown": [1, 2]}, "reloc_needs_operator: 2 candidates, no operator (unattended)"),
    ({"status": "failed"}, "reloc_failed: no candidate an operator could pick"),
    ({"status": "bridge_timeout"}, "reloc_bridge_timeout: no bridge_go within the wait"),
    ({"status": "cancelled"}, "reloc_cancelled"),
    ({"status": "operator_timeout"}, "reloc_operator_timeout: no pick or retry within the operator's time"),
])
def test_every_ending_but_chosen_is_a_session_failure_reason(out, reason):
    assert P.failure_reason(out) == reason


class FakeConsole:
    """The early console's part the phase sees (B-P2c): ``service(stage)`` at every step and wait; a script acts on the
    relocalizer at a stage (the operator's pick or retry, a STOP's interrupt, a session end)."""

    def __init__(self, script=None):
        self.reloc, self.calls, self.stages, self.notes, self.state_override = None, 0, [], {}, None
        self.script = script or (lambda console, stage: None)

    def attach(self, relocalizer):
        self.reloc = relocalizer

    def note(self, **fields):
        self.notes.update(fields)

    def localizing(self):
        self.state_override = "localizing"

    def gate(self, stage):
        pass

    def service(self, stage):
        self.calls += 1
        self.stages.append(stage)
        self.script(self, stage)


SYMMETRIC = (6.0, 4.0, [])  # the turned twin always competes: never confirmed


def _symmetric(tmp_path, console, **kw):
    grid = TG._grid(*SYMMETRIC)
    base = (2.0, 1.4, 0.0)
    return _inputs(tmp_path, grid, FakeTruck(grid, base), base,
                   **{"console": console, "unattended": False,
                      "match_cfg": G.MatchConfig(yaw_step_rad=math.radians(2.0), max_nodes=5_000_000), **kw})


def test_attended_the_phase_waits_for_the_operators_pick(tmp_path):
    def pick_when_shown(console, stage):
        r = console.reloc
        if r.state == "awaiting_pick" and stage == "awaiting_pick":
            assert r.pick(r.generation, 0) == (True, "")

    _go_later(tmp_path)
    out = P.run(_symmetric(tmp_path, FakeConsole(pick_when_shown)))
    assert out["status"] == "chosen" and R.read_reloc_pose(tmp_path)["source"] == "operator"
    assert len(out["attempts"]) == 3 and out["bridge_go"] is not None


def test_a_stop_while_the_matcher_runs_drops_its_result_and_waits_for_a_retry(tmp_path):
    seen = {"stopped": False, "retried": False}

    def script(console, stage):
        r = console.reloc
        if stage == "matching" and not seen["stopped"]:
            seen["stopped"] = r.interrupt()
        elif r.state == "stopped" and stage == "stopped" and not seen["retried"]:
            seen["retried"] = r.retry(r.generation)[0]

    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    _go_later(tmp_path)
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base), base, console=FakeConsole(script), unattended=False))
    assert seen == {"stopped": True, "retried": True} and out["status"] == "chosen"
    assert [a["reason"] for a in out["attempts"]] == ["interrupted", "confirmed"]  # the first result never counted


def test_a_stop_between_the_matchers_return_and_the_estimate_drops_a_confirmed_result(tmp_path):
    # Codex B-P2c design P1/P2: the phase takes the STOPs queued meanwhile before it records the result
    state = {"n": 0}

    def script(console, stage):
        if stage == "matched" and state["n"] == 0:
            state["n"] = 1
            console.reloc.interrupt()

    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base), base, console=FakeConsole(script), unattended=False,
                        operator_deadline_at=time.monotonic() + 3.0))
    assert out["attempts"][0]["reason"] == "interrupted" and R.read_reloc_pose(tmp_path) is None
    assert out["status"] == "operator_timeout"  # nobody retried: the operator budget ended it


def test_a_stop_and_a_quick_retry_never_count_the_old_workers_result(tmp_path):
    # Codex B-P2c impl review P2: the old worker's result belongs to its generation, not to the retry's attempt
    seen = {"n": 0}

    def script(console, stage):
        r = console.reloc
        if stage == "matching" and seen["n"] == 0:
            seen["n"] = 1
            assert r.interrupt()
            assert r.retry(r.generation) == (True, "")  # both before the old worker returned

    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    _go_later(tmp_path)
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base), base, console=FakeConsole(script), unattended=False))
    assert out["status"] == "chosen"
    assert [a["reason"] for a in out["attempts"]] == ["interrupted", "confirmed"]


def test_a_stop_and_a_quick_retry_during_the_scans_drop_those_scans(tmp_path):
    # Codex B-P2c impl review round 2 P2: the scans belong to the generation they were taken for
    seen = {"n": 0}

    def script(console, stage):
        r = console.reloc
        if stage == "scan" and seen["n"] == 0 and r.state == "relocalizing":
            seen["n"] = 1
            assert r.interrupt() and r.retry(r.generation) == (True, "")

    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    _go_later(tmp_path)
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base), base, console=FakeConsole(script), unattended=False))
    assert out["status"] == "chosen" and [a["reason"] for a in out["attempts"]] == ["confirmed"]


def test_the_matcher_cannot_confirm_after_the_operators_time(tmp_path):
    # Codex B-P2c impl review round 2 P2: the whole budget bounds the matcher's own choice too
    clock = {"t": 0.0}

    def script(console, stage):
        if stage == "matching":
            clock["t"] = 25.0  # the matcher returns past the operator's deadline, inside its own 30 s one

    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    out = P.run(_inputs(tmp_path, grid, FakeTruck(grid, base), base, console=FakeConsole(script), unattended=False,
                        clock=lambda: clock["t"], operator_deadline_at=20.0))
    assert out["status"] == "operator_timeout" and R.read_reloc_pose(tmp_path) is None


def test_a_pick_after_the_operators_time_is_refused_and_nothing_is_committed(tmp_path):
    # Codex B-P2c impl review P2: the operator's deadline holds for the pick itself
    clock = {"t": 0.0}
    tried = {}

    def script(console, stage):
        r = console.reloc
        if r.state == "awaiting_pick" and "pick" not in tried:
            clock["t"] = 41.0  # the operator answers a second after the deadline
            tried["pick"] = r.pick(r.generation, 0)

    out = P.run(_symmetric(tmp_path, FakeConsole(script), clock=lambda: clock["t"], operator_deadline_at=40.0,
                           pace_s=0.0))
    assert tried["pick"] == (False, "reloc_operator_timeout")
    assert out["status"] == "operator_timeout" and R.read_reloc_pose(tmp_path) is None


def test_the_stop_gate_runs_before_every_physics_step(tmp_path):
    # Codex B-P2c impl review P2: a STOP queued before a step is taken before that step
    order = []

    class GatedConsole(FakeConsole):
        def gate(self, stage):
            order.append("gate")

        def service(self, stage):
            order.append("service")

    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    truck = FakeTruck(grid, base)
    real_step = truck.step
    truck.step = lambda: (order.append("step"), real_step())
    _go_later(tmp_path)
    P.run(_inputs(tmp_path, grid, truck, base, console=GatedConsole(), unattended=False))
    steps = [i for i, x in enumerate(order) if x == "step"]
    assert steps and all(order[i - 1] == "gate" for i in steps)


def test_the_operator_budget_ends_a_wait_nobody_answers(tmp_path):
    out = P.run(_symmetric(tmp_path, FakeConsole(), operator_deadline_at=time.monotonic() + 0.5))
    assert out["status"] == "operator_timeout" and R.read_reloc_pose(tmp_path) is None
    assert P.failure_reason(out) == "reloc_operator_timeout: no pick or retry within the operator's time"


def test_unattended_with_a_console_still_ends_at_the_request(tmp_path):
    out = P.run(_symmetric(tmp_path, FakeConsole(), unattended=True))
    assert out["status"] == "needs_operator"


def test_the_operators_session_end_leaves_the_phase_and_cancels_the_matcher(tmp_path):
    class End(Exception):
        pass

    def script(console, stage):
        if stage == "matching":
            raise End()

    inputs = _symmetric(tmp_path, FakeConsole(script))
    with pytest.raises(End):
        P.run(inputs)
    assert inputs.cancel.is_set() and R.read_reloc_pose(tmp_path) is None


def test_a_retry_never_reuses_a_noise_key(tmp_path):
    keys = []
    retried = {"n": 0}

    def script(console, stage):
        r = console.reloc
        if r.state == "awaiting_pick" and retried["n"] == 0:
            retried["n"] = 1
            r.retry(r.generation)
        elif r.state == "awaiting_pick" and retried["n"] == 1:
            r.pick(r.generation, 0)

    _go_later(tmp_path, wait_s=120.0)  # six attempts in the symmetric room
    out = P.run(_symmetric(tmp_path, FakeConsole(script), noise_for=lambda k, s: keys.append((k, s))))
    assert out["status"] == "chosen" and len(out["attempts"]) == 6
    assert len(keys) == len(set(keys)) == 6 * C.SCANS


def test_the_console_is_serviced_at_every_step_and_says_localizing_after_the_estimate(tmp_path):
    console = FakeConsole()
    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    truck = FakeTruck(grid, base)
    _go_later(tmp_path)
    out = P.run(_inputs(tmp_path, grid, truck, base, console=console, unattended=False))
    assert out["status"] == "chosen" and console.calls >= truck.steps and console.state_override == "localizing"
    assert console.notes["last_reason"] == "confirmed"


def test_the_frozen_settings_are_b_p1bs():
    cfg, reloc, record = P.frozen_settings(ROOT)
    assert cfg.max_work == 2_700_000_000 and cfg == G.MatchConfig(max_work=2_700_000_000)
    assert (reloc.e_min, reloc.r_max, reloc.r_struct) == (0.8, 0.95, 0.8)
    assert reloc.attempts == 3 and reloc.deadline_s == 30.0
    assert set(record) == {"match_config", "dev_choice"} and all(len(v["sha256"]) == 64 for v in record.values())
