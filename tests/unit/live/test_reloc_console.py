"""B-P2c (start relocalization plan 1.2 steps 4, 7; 1.3; Codex B-P2c design rounds 1-3): the console while the start is
unknown, serviced by the runner's main thread -- the STOP gate at every call, the arbiter's cycle at most once per tick
with a fresh measurement, the state with no pose and the relocalization in ``slam.reloc``, a STOP that interrupts the
attempt, the operator's session end, the survey map as the page's grid, the gaps between calls and the port file."""

import json
import math

import pytest

pytest.importorskip("websockets")

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import start_reloc as R  # noqa: E402
from forklift_core.localization.slam_map import Grid  # noqa: E402
from forklift_core.safety.arbiter import ArbiterConfig  # noqa: E402
from sim.isaac import live_runtime as L  # noqa: E402
from sim.isaac import reloc_console as RC  # noqa: E402

DT = 1 / 120


class Clock:
    def __init__(self):
        self.t = 10.0

    def __call__(self):
        return self.t


def _awaiting():
    r = R.StartRelocalizer(R.RelocConfig(e_min=0.8, r_max=0.95, r_struct=0.8, attempts=1), lambda: 0.0)
    r.start()
    cands = (G.Candidate((3.0, 1.0, 0.5), 0.9, 0.97), G.Candidate((9.0, 4.0, 2.0), 0.88, 0.96))
    r.record(G.GlobalMatch("ok", cands), G.Decision(False, "competitor", 0.98))
    return r


def _grid(origin=(-1.0, -2.0, 0.0)):
    import numpy as np

    data = np.full((4, 6), -1, np.int8)
    data[1, 1:5] = 0
    data[2, 2] = 100
    return Grid(data, origin, 0.5)


class Rig:
    def __init__(self, tmp_path, reloc=None, **kw):
        self.clock = Clock()
        self.measured = {"speed": 0.0}
        self.parts = L.make_console(config=ArbiterConfig(tick_s=DT), session_epoch=5)
        self.console = RC.RelocConsole(self.parts, measure=lambda: (self.measured["speed"], 0.0, 0.05, 0.0),
                                       clock=self.clock, port_file=tmp_path / "port.json", tick_s=DT,
                                       slam_info={"mode": "localization", "map_id": "m" * 64, "map_loaded": True},
                                       grid=_grid(), **kw)
        self.ch = self.parts.channels
        self.token = self.ch.issue_token()
        self.ch.connect("A", "op", True, self.token)
        self.seq = 0
        if reloc is not None:
            self.console.attach(reloc)

    def send(self, kind, **fields):
        self.seq += 1
        msg = {"t": kind, "request_id": f"r{self.seq}", "session_epoch": 5, "request_token": self.token,
               "request_seq": self.seq, **fields}
        if kind != "lease_request" and self.parts.arbiter.lease is not None:
            msg.update(lease_id=self.parts.arbiter.lease.lease_id, control_gen=self.parts.arbiter.control_gen)
        self.ch.message("A", "op", msg)
        return msg["request_id"]

    def service(self, n=1, stage="test"):
        for _ in range(n):
            self.clock.t += DT
            self.console.service(stage)

    def out(self):
        return [msg for _, msg in self.ch.take_out()]

    def lease(self):
        self.send("lease_request")
        self.service()
        assert self.parts.arbiter.lease is not None


def test_the_state_has_no_pose_and_carries_the_relocalization(tmp_path):
    rig = Rig(tmp_path, _awaiting())
    rig.console.note(last_reason="competitor", operator_deadline_at=rig.clock.t + 100.0)
    rig.service()
    [state] = [m for m in rig.out() if m.get("t") == "state"]
    assert state["pose"] is None and state["slam"]["mode"] == "localization"
    reloc = state["slam"]["reloc"]
    assert reloc["state"] == "awaiting_pick" and reloc["generation"] == 1 and reloc["last_reason"] == "competitor"
    assert [c["index"] for c in reloc["candidates"]] == [0, 1]
    assert reloc["candidates"][1]["base"] == pytest.approx(list(R.base_from_laser((9.0, 4.0, 2.0))))
    assert reloc["operator_left_s"] == pytest.approx(100.0 - DT, abs=1e-6)


def test_a_pick_through_the_console_is_the_estimate(tmp_path):
    reloc = _awaiting()
    rig = Rig(tmp_path, reloc)
    rig.lease()
    rid = rig.send("relocalize_pick", generation=1, candidate=0)
    rig.service()
    assert reloc.state == "chosen" and any(m.get("request_id") == rid and m["t"] == "ack" for m in rig.out())


def test_a_stop_interrupts_the_relocalization_and_latches_stopped(tmp_path):
    reloc = _awaiting()
    rig = Rig(tmp_path, reloc)
    rig.ch.message("A", "op", {"t": "stop", "request_id": "s1", "session_epoch": 5})
    rig.service()
    assert rig.parts.arbiter.mode == "STOPPED" and reloc.state == "stopped" and rig.console.interrupts == 1
    rig.service(5)
    assert rig.console.interrupts == 1  # one STOP, one interruption
    assert reloc.retry(reloc.generation) == (True, "")  # a retry in STOPPED: a computation, the truck stands
    rig.service()
    assert reloc.state == "relocalizing"  # the STOP taken before the retry does not interrupt the new attempt


def test_the_gate_runs_at_every_call_and_the_cycle_at_most_once_a_tick(tmp_path):
    rig = Rig(tmp_path, _awaiting())
    begins = []
    real = rig.parts.link.begin
    rig.parts.link.begin = lambda now, fb: begins.append(fb) or real(now, fb)
    for _ in range(4):
        rig.clock.t += DT / 5
        rig.console.service("test")
    assert len(begins) == 1
    rig.ch.message("A", "op", {"t": "stop", "request_id": "s", "session_epoch": 5})
    rig.clock.t += DT / 5
    rig.console.service("test")  # 4/5 of a tick since the cycle: none due, the STOP taken at the gate all the same
    assert rig.parts.arbiter.mode == "STOPPED" and len(begins) == 1


def test_the_feedback_is_measured_now(tmp_path):
    rig = Rig(tmp_path, _awaiting())
    seen = []
    real = rig.parts.link.begin
    rig.parts.link.begin = lambda now, fb: seen.append((now, fb)) or real(now, fb)
    rig.measured["speed"] = 0.02
    rig.service()
    now, fb = seen[-1]
    assert fb.stamp_mono == now and fb.rear_axle_speed_mps == 0.02 and fb.lift_m == 0.05


def test_an_accepted_session_end_leaves_the_phase(tmp_path):
    rig = Rig(tmp_path, _awaiting())
    rig.service(60)  # a confirmed stand
    rig.lease()
    rig.send("session_end")
    with pytest.raises(RC.RelocSessionEnd):
        rig.service(3)
    assert rig.parts.arbiter.session_request == "end"


def test_an_accepted_session_end_holds_even_if_the_port_file_cannot_be_written(tmp_path):
    # Codex B-P2c impl review P2: the end is settled before anything that could fail
    rig = Rig(tmp_path, _awaiting())
    rig.console.open_port_file()
    rig.service(60)
    rig.lease()
    rig.send("session_end")
    rig.console.port_file = tmp_path / "missing" / "port.json"  # a write that fails
    rig.console._port_state = None
    with pytest.raises(RC.RelocSessionEnd):
        rig.service(3)
    assert rig.console.ended_by_operator


def test_the_gate_alone_takes_a_stop_and_interrupts(tmp_path):
    reloc = _awaiting()
    rig = Rig(tmp_path, reloc)
    rig.ch.message("A", "op", {"t": "stop", "request_id": "s1", "session_epoch": 5})
    rig.console.gate("scan")
    assert rig.parts.arbiter.mode == "STOPPED" and reloc.state == "stopped" and rig.console.interrupts == 1


def test_a_choice_not_yet_sent_out_reads_as_localizing(tmp_path):
    reloc = _awaiting()
    rig = Rig(tmp_path, reloc)
    reloc.pick(1, 0)
    assert reloc.state == "chosen" and rig.console.state == "localizing"  # the contract has no "chosen"


def test_a_reset_is_refused_while_the_start_is_unknown(tmp_path):
    rig = Rig(tmp_path, _awaiting())
    rig.parts.arbiter.sim_reset_allowed = True
    rig.service(60)
    rig.lease()
    rid = rig.send("sim_reset")
    rig.service()
    [reply] = [m for m in rig.out() if m.get("request_id") == rid]
    assert reply["t"] == "reject" and reply["reason"] == "relocalizing" and rig.parts.arbiter.session_request is None


def test_gaps_between_calls_are_recorded_with_their_stage(tmp_path):
    rig = Rig(tmp_path, _awaiting())
    rig.service()
    rig.clock.t += 0.75
    rig.console.service("prewarm")
    rig.service()
    assert rig.console.gaps == [{"stage": "prewarm", "gap_s": pytest.approx(0.75)}]


def test_the_port_file_follows_the_relocalization_and_never_says_running(tmp_path):
    reloc = _awaiting()
    rig = Rig(tmp_path, reloc)
    rig.console.open_port_file()
    rig.service()
    assert json.loads((tmp_path / "port.json").read_text())["state"] == "awaiting_pick"
    reloc.interrupt()
    rig.service()
    assert json.loads((tmp_path / "port.json").read_text())["state"] == "stopped"
    rig.console.localizing()
    rig.service()
    info = json.loads((tmp_path / "port.json").read_text())
    assert info["state"] == "localizing" and info["session_epoch"] == 5 and "started" in info


def test_the_survey_map_is_the_pages_grid():
    item = RC.grid_from_bundle(_grid(), session_epoch=5)
    head, body = item["head"], item["body"]
    assert head["origin_m"] == [-1.0, -2.0] and head["width"] == 6 and head["height"] == 4 and head["frame"] == "map"
    assert head["values"] == {"unknown": 0, "free": 1, "occupied": 2}
    cells = list(body)
    assert cells[1 * 6 + 1] == 1 and cells[2 * 6 + 2] == 2 and cells[0] == 0
    with pytest.raises(ValueError, match="rotated"):
        RC.grid_from_bundle(_grid(origin=(0.0, 0.0, math.pi / 4)), session_epoch=5)


def test_the_console_publishes_the_survey_map(tmp_path):
    rig = Rig(tmp_path, _awaiting())
    assert rig.parts.store.get("grid") is not None
