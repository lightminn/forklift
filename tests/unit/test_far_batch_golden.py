"""The batch far recognition keeps ec4bece's results and records on fixed inputs (plan 2026-10-22-live-far-recognition
task 11): the per-frame work split off for the live worker thread must not change what a batch run decides or saves.
Both run here, on this machine's rig renders: this checkout's code and ec4bece's two files (tests/fixtures/far_ec4bece/)."""
import json
from pathlib import Path

from tools import far_batch_golden as G

ROOT = Path(__file__).resolve().parents[2]
THEN = ROOT / "tests/fixtures/far_ec4bece"


def test_the_batch_far_path_matches_the_records_of_ec4bece():
    now = json.loads(json.dumps(G.run_all(G.load_runtime()), sort_keys=True))
    then = json.loads(json.dumps(G.run_all(G.load_runtime(THEN / "far_recognition_runtime.py",
                                                          THEN / "recognition_bridge.py")), sort_keys=True))
    assert sorted(now) == sorted(then)
    for name in then:
        for key in then[name]:
            assert now[name][key] == then[name][key], (name, key)
    assert any(s["observation"]["status"] == "valid" for v in then.values() for s in v["stops"])
