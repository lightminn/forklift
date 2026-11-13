"""The loaded curvature rule in the runner (docs/plans/2026-11-14-loaded-curvature-rule.md 2-3): the rule file, its
limits, the configs, the near-tracking contract and the runner's wiring (source checks; the runner needs Isaac)."""

import sys
from dataclasses import replace
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))

import loaded_curvature_rule as lcr  # noqa: E402

from forklift_core.planning.pallet_mission import make_transport_planner_config  # noqa: E402

RUNNER = (ROOT / "sim/isaac/run_transport.py").read_text()


def write(tmp_path, text):
    p = tmp_path / "rule.yaml"
    p.write_text(text)
    return p


def test_the_registered_rule_file_loads_and_gives_0_270():
    rule = lcr.load_rule(ROOT / "config/loaded_curvature_rule.yaml")
    assert rule["k"] == 0.90 and rule["saturated_curvature_inv_m"] == 0.300
    assert rule["source"]["values"]["right_turn_median"] == 0.3003
    assert lcr.planned_curvature(rule, 0.36, 0.3665) == pytest.approx(0.270)


@pytest.mark.parametrize("text", [
    "k: 0.9\nsaturated_curvature_inv_m: 0.3\n",  # no source
    "k: 0\nsaturated_curvature_inv_m: 0.3\nsource: {a: 1}\n",
    "k: 1.2\nsaturated_curvature_inv_m: 0.3\nsource: {a: 1}\n",
    "k: 0.9\nsaturated_curvature_inv_m: -0.3\nsource: {a: 1}\n",
    "k: .nan\nsaturated_curvature_inv_m: 0.3\nsource: {a: 1}\n",
    "k: true\nsaturated_curvature_inv_m: 0.3\nsource: {a: 1}\n",
    "k: 0.9\nsaturated_curvature_inv_m: 0.3\nsource: {}\n",
])
def test_a_malformed_rule_is_refused(tmp_path, text):
    with pytest.raises(ValueError):
        lcr.load_rule(write(tmp_path, text))


def test_a_rule_above_the_tracker_or_the_model_is_refused():
    rule = {"k": 1.0, "saturated_curvature_inv_m": 0.37, "source": {"a": 1}}
    with pytest.raises(ValueError, match="tracker"):
        lcr.planned_curvature(rule, 0.36, 0.3665)
    with pytest.raises(ValueError, match="steering limit"):
        lcr.planned_curvature({**rule, "saturated_curvature_inv_m": 0.355}, 0.36, 0.35)


def test_the_configs_replace_the_curvature_only():
    base = make_transport_planner_config(curvature_limit_inv_m=0.29, clearance_m=0.10)
    travel = replace(base, obstacle_heuristic_resolution_m=0.25)
    cfg, tr = lcr.loaded_configs(base, travel, 0.27)
    assert cfg == replace(base, curvature_limit_inv_m=0.27) and tr == replace(travel, curvature_limit_inv_m=0.27)
    assert lcr.loaded_configs(base, None, 0.27)[1] is None


def test_the_near_tracking_contract_names_the_option():
    import near_tracking
    assert "loaded_curvature_rule" in near_tracking.RUN_SPECIFIC_ARGUMENTS


def test_the_runner_wiring():
    assert '"--loaded-curvature-rule"' in RUNNER
    assert 'loaded_planner_config = _bound.get("loaded_planner_config")' in RUNNER  # an old setup: off
    assert "loaded_planner_config=loaded_planner_config, loaded_travel_config=loaded_travel_config" in RUNNER
    # the first mission plan: the transport stage keyword only when on
    assert '**({} if loaded_planner_config is None else {"transport_travel_config": (' in RUNNER
    # the console's loaded delivery job
    assert "**return_start_backoff_kw(args), **core.loaded_mission_kwargs()}" in RUNNER
    assert 'state["loaded_curvature_rule"] = {' in RUNNER
    assert "args.loaded_curvature_rule" in RUNNER and RUNNER.count('getattr(args, "loaded_curvature_rule", None)') >= 3
