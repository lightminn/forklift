"""Persistent console plan D2 (Codex stage 2 design P2-6): a live run's whole observation travel is planned in the
hall -- the preparation, the observe replans, the next candidates and the observe cusp check -- while a batch run's
keeps the pickup bounds. The approach and the insertion keep the pickup bounds in both."""

import ast
from pathlib import Path
from types import SimpleNamespace

from forklift_core.mission.core import MissionCore

CORE = Path(__file__).resolve().parents[3] / "src/forklift_core/mission/core.py"


def test_the_observation_bounds_are_the_pickup_bounds_in_a_batch_run_and_the_hall_live():
    core = SimpleNamespace(options=SimpleNamespace(pickup_bounds="BAY"))
    assert MissionCore.observe_bounds(core) == "BAY"
    core.options.observe_bounds = None
    assert MissionCore.observe_bounds(core) is None


def test_every_observation_leg_plan_in_the_core_takes_the_observation_bounds():
    tree = ast.parse(CORE.read_text())
    calls = [n for n in ast.walk(tree) if isinstance(n, ast.Call) and getattr(n.func, "id", None) == "plan_observation_leg"]
    assert len(calls) >= 5
    for call in calls:
        (kw,) = [k for k in call.keywords if k.arg == "pickup_bounds"]
        assert ast.unparse(kw.value) in ("observe_bounds_", "self.observe_bounds()"), (call.lineno, ast.unparse(kw.value))


def test_the_observe_cusp_check_takes_the_observation_bounds():
    source = CORE.read_text()
    body = source[source.index("        if ms.phase in (\"transport\", \"return_home\"):\n            return escape_leg"):][:900]
    assert "pickup_bounds=self.observe_bounds()" in body
