"""Resume and failure paths of tools/bay_planning_sweep.py (the results are not frozen)."""

import json
from pathlib import Path

from tools import bay_planning_sweep as sweep

ROOT = Path(__file__).resolve().parents[3]
BASE = [
    "--seeds",
    "0",
    "--settings",
    str(ROOT / "config/isaac_transport.yaml"),
    "--forklift-urdf",
    str(ROOT / "sim/models/dls08_provisional/forklift.urdf"),
    "--retry-expansions",
    "0",
]


def _record(out: Path) -> dict:
    return json.loads((out / "seed_0.json").read_text())


def test_timeout_keeps_the_scene_and_a_rerun_with_the_same_inputs_is_skipped(tmp_path):
    out = tmp_path / "run"
    assert sweep.main([*BASE, "--timeout-s", "0.01", "--output", str(out)]) == 0
    first = _record(out)
    assert first["outcome"] in ("timeout", "crash")
    stamp = (out / "seed_0.json").stat().st_mtime_ns
    assert sweep.main([*BASE, "--timeout-s", "0.01", "--output", str(out)]) == 0
    assert (out / "seed_0.json").stat().st_mtime_ns == stamp


def test_changing_the_budget_reruns_the_seed(tmp_path):
    out = tmp_path / "run"
    sweep.main([*BASE, "--max-expansions", "10", "--output", str(out)])
    small = _record(out)
    sweep.main([*BASE, "--max-expansions", "30000", "--output", str(out)])
    large = _record(out)
    assert small["input_sha256"] != large["input_sha256"]
    assert large["outcome"] == "success"
    assert small["status"] != large["status"]


def test_a_crash_is_not_read_back_as_an_older_success(tmp_path):
    out = tmp_path / "run"
    out.mkdir()
    (out / "seed_0.json").write_text(
        json.dumps({"seed": 0, "input_sha256": "OLD", "outcome": "success"})
    )
    (out / "seed_0.trace.jsonl").write_text('{"stage": "OLD"}\n')
    broken = tmp_path / "pallet.yaml"
    broken.write_text("not: [a pallet geometry")
    sweep.main([*BASE, "--pallet-geometry", str(broken), "--output", str(out)])
    record = _record(out)
    assert record["outcome"] == "crash" and record["input_sha256"] != "OLD"
    assert not (out / "seed_0.trace.jsonl").exists()
