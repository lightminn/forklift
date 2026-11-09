"""Real-time plan 5.25: the re-run's environment validity -- a foreign GPU process, other users' CPU, a missing
monitor or no CPU sample each make a session invalid; a clean one passes."""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "r1c_validity", ROOT / "docs" / "validation" / "2026-11-08-r1-confirmation" / "r1c_validity.py")
rv = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rv)


def _session(tmp_path, procs):
    out = tmp_path / "s"
    (out / "monitor").mkdir(parents=True)
    (out / "monitor" / "host.jsonl").write_text("\n".join(
        json.dumps({"wall": 1000.0 + 10 * i, "gpu_status": "ok", "gpu_procs": procs}) for i in range(11)) + "\n")
    return out


def test_clean_foreign_gpu_and_busy_cpu(tmp_path):
    quiet = [{"wall": 1000.0 + 10 * i, "other_cores": 0.02} for i in range(11)]
    assert rv.session_validity(_session(tmp_path / "a", [[1, "C", "python", True]]), quiet) == []
    bad = rv.session_validity(_session(tmp_path / "b", [[2, "C", "x", False]]), quiet)
    assert bad and "GPU process" in bad[0]
    busy = [{"wall": 1000.0 + 10 * i, "other_cores": 2.0 if i < 2 else 0.0} for i in range(11)]
    assert any("other users' CPU" in b for b in rv.session_validity(_session(tmp_path / "c", []), busy))
    assert any("no sample" in b for b in rv.session_validity(_session(tmp_path / "d", []), []))
    assert rv.session_validity(tmp_path / "none", quiet) == ["no monitor record"]
