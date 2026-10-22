"""The batch far recognition keeps ec4bece's results and records on fixed inputs (plan 2026-10-22-live-far-recognition
task 11): the per-frame work split off for the live worker thread must not change what a batch run decides or saves."""
import json
from pathlib import Path

from tools import far_batch_golden as G

ROOT = Path(__file__).resolve().parents[2]
GOLDEN = ROOT / "tests/fixtures/far_batch_golden_ec4bece.json"


def test_the_batch_far_path_matches_the_records_of_ec4bece():
    now = json.loads(json.dumps(G.run_all(G.load_runtime()), sort_keys=True))
    then = json.loads(GOLDEN.read_text())
    assert sorted(now) == sorted(then)
    for name in then:
        for key in then[name]:
            assert now[name][key] == then[name][key], (name, key)
