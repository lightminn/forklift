"""riib scene records with the far recognition (2026-10-29): the far runtime is a recorded port from the hand-over on,
so the setup snapshot lists it as a service (not an opaque value) and a replay supplies it through the record."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/isaac"))

import replay_setup  # noqa: E402


class FarRuntime:  # a stand-in with the members the runner uses
    paused = False
    truth_instance = 0
    zone_centre_xy = (0.0, 0.0)
    zone_radius_m = 1.0

    def due(self, tick, *, observing):
        return False

    def summary(self):
        return {}


def test_the_setup_snapshot_lists_the_far_runtime_as_a_service_and_none_as_a_value(tmp_path):
    audit = replay_setup.write_setup_snapshot({"far_runtime": FarRuntime()}, tmp_path / "a", [])
    import json

    snap = json.loads((tmp_path / "a" / "setup_snapshot.json").read_text())
    assert snap["names"]["far_runtime"]["kind"] == "service" and snap["names"]["far_runtime"]["id"] == "FarRecognition"
    assert not audit.get("opaque")
    replay_setup.write_setup_snapshot({"far_runtime": None}, tmp_path / "b", [])
    snap = json.loads((tmp_path / "b" / "setup_snapshot.json").read_text())
    assert snap["names"]["far_runtime"]["kind"] == "value"  # recognition off (every older record): no service


def test_the_runner_wraps_the_far_runtime_as_a_port_with_its_frames_by_hash():
    source = (ROOT / "sim/isaac/run_transport.py").read_text()
    block = source[source.index("    if early_ports:"):source.index('    state["phase"] = "planning"')]
    assert 'far_runtime = RecordingProxy(far_runtime, "FarRecognition", boundary,' in block
    assert 'hash_only_requests={"drive_frame", "stop"}' in block
    assert 'watches={"drive_frame": far_watch_, "stop": far_watch_}' in block
    assert '_FarWatch(boundary.recorder, {"state": state}, keys={"state": ["recognition"]})' in block


class LiveLists:
    """The far runtime's shape that matters here: summary() hands back its own lists, drive_frame and stop append
    to them later (Codex review 1 P2)."""

    def __init__(self):
        self.frame_records, self.attempts = [], []

    def drive_frame(self, tick):
        self.frame_records.append({"kind": "drive", "tick": tick})

    def stop(self, tick):
        self.frame_records.append({"kind": "stop", "tick": tick})
        self.attempts.append({"tick": tick})
        return "seen", {"tick": tick}

    def summary(self):
        return {"frames": self.frame_records, "attempts": self.attempts}


def test_frames_the_runtime_adds_after_its_summary_reach_the_replayed_state(tmp_path):
    from forklift_core.mission.boundary_record import BoundaryRecorder, DictWatch, RecordingGate, RecordingProxy
    from forklift_core.mission.replay import ReplayGate, ReplayProxy, ReplaySession

    def run(far, state):
        far.stop(0)
        state.setdefault("recognition", {}).update(far.summary())
        far.drive_frame(1)  # observing again after the stop; the run ends before the next stop
        far.drive_frame(2)

    rec = BoundaryRecorder(tmp_path, value_format=2)
    gate = RecordingGate(rec)
    state: dict = {}
    watch = DictWatch(rec, {"state": state}, keys={"state": ["recognition"]})
    rec.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    run(RecordingProxy(LiveLists(), "FarRecognition", gate, hash_only_requests={"drive_frame", "stop"},
                       watches={"drive_frame": watch, "stop": watch}), state)
    rec.tick_end(outputs={}, branches=[])
    rec.close()
    assert [r["tick"] for r in state["recognition"]["frames"]] == [0, 1, 2]

    session = ReplaySession(tmp_path)
    rgate = ReplayGate(session)
    rstate: dict = {}
    rwatch = DictWatch(None, {"state": rstate}, keys={"state": ["recognition"]})
    session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    run(ReplayProxy("FarRecognition", rgate, spec=LiveLists, hash_only_requests={"drive_frame", "stop"},
                    watches={"drive_frame": rwatch, "stop": rwatch}), rstate)
    session.tick_end(outputs={}, branches=[])
    session.finish()
    assert rstate["recognition"] == state["recognition"]
