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
