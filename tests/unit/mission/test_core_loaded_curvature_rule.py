"""The loaded curvature rule in the mission core (docs/plans/2026-11-14-loaded-curvature-rule.md 3.2-3.3): the
transport-phase planning calls take the rule's configs; with the option off every call is as before (the run's own
objects, no new keyword)."""

import ast
import inspect
import re
from dataclasses import replace
from types import SimpleNamespace

import pytest

from forklift_core.mission import core as core_module
from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState
from forklift_core.planning.pallet_mission import make_transport_planner_config

BASE = make_transport_planner_config(curvature_limit_inv_m=0.29, clearance_m=0.10)
TRAVEL = replace(BASE, obstacle_heuristic_resolution_m=0.25)


def core_with(loaded: bool, travel=TRAVEL):
    lp = replace(BASE, curvature_limit_inv_m=0.27) if loaded else None
    lt = (replace(travel, curvature_limit_inv_m=0.27) if travel is not None else None) if loaded else None
    options = SimpleNamespace(planner_config=BASE, travel_config=travel, loaded_planner_config=lp,
                              loaded_travel_config=lt, return_to_pose=None)
    c = MissionCore(MissionState(phase="transport", phase_started=0.0), state={}, obstacle=None, slam=None,
                    pocket={"check": None}, paths={}, trackers={}, geometry=None, settings={}, pallet_geometry=None,
                    hooks=SimpleNamespace(), near=None, options=options)
    return c


def test_off_returns_the_runs_own_objects():
    c = core_with(False)
    cfg, travel = c.loaded_planner_configs()
    assert cfg is BASE and travel is TRAVEL
    tight, tight_travel = object(), object()
    assert c.loaded_tight_configs(tight, tight_travel) == (tight, tight_travel)
    assert c.loaded_mission_kwargs() == {} and c.loaded_mission_kwargs(tight=True) == {}


def test_a_record_without_the_option_names_is_off():
    c = core_with(False)
    del c.options.loaded_planner_config, c.options.loaded_travel_config
    assert c.loaded_planner_configs() == (BASE, TRAVEL) and c.loaded_mission_kwargs() == {}


def test_on_gives_the_rules_configs():
    c = core_with(True)
    cfg, travel = c.loaded_planner_configs()
    assert cfg.curvature_limit_inv_m == 0.27 and travel.curvature_limit_inv_m == 0.27
    assert travel.obstacle_heuristic_resolution_m == 0.25
    t_cfg, t_travel = c.loaded_tight_configs(object(), object())
    assert t_cfg.clearance_m == 0.0 and t_cfg.curvature_limit_inv_m == 0.27 and t_travel.clearance_m == 0.0
    kw = c.loaded_mission_kwargs()
    assert kw["transport_travel_config"] is travel
    assert c.loaded_mission_kwargs(tight=True)["transport_travel_config"].clearance_m == 0.0


def test_on_without_a_travel_config_uses_the_planner_config_for_the_transport_search():
    c = core_with(True, travel=None)
    cfg, travel = c.loaded_planner_configs()
    assert travel is None and c.loaded_mission_kwargs()["transport_travel_config"] is cfg


@pytest.mark.parametrize("loaded", [False, True])
def test_the_leg_payload_uses_the_rule_for_transport_only(loaded):
    c = core_with(loaded)
    c.hooks.exit_prefix = None
    c.geometry = None
    tr = c._leg_payload("transport", "world", "start", {}, retry_on=("no_path",))
    rt = c._leg_payload("return", "world", "start", {}, retry_on=("no_path",))
    want = 0.27 if loaded else 0.29
    assert tr["args"][2].curvature_limit_inv_m == want and tr["tight_args"][2].curvature_limit_inv_m == want
    assert tr["kwargs"]["travel_config"].curvature_limit_inv_m == want
    assert tr["tight_kwargs"]["travel_config"].curvature_limit_inv_m == want
    assert tr["tight_args"][2].clearance_m == 0.0
    assert rt["args"][3].curvature_limit_inv_m == 0.29 and rt["kwargs"]["travel_config"].curvature_limit_inv_m == 0.29
    if not loaded:
        assert tr["args"][2] is BASE and tr["kwargs"]["travel_config"] is TRAVEL


# ---- source checks: every loaded transport planning call takes the rule's configs ---------------------------------


SOURCE = inspect.getsource(core_module)


def calls_named(name):
    tree = ast.parse(SOURCE)
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            f = node.func
            n = f.attr if isinstance(f, ast.Attribute) else getattr(f, "id", None)
            if n == name:
                out.append(node)
    return out


def test_every_transport_leg_call_takes_a_loaded_config():
    """plan_transport_leg plans a path driven loaded: its config argument is the rule's (lcfg_ / ltight_ from the
    helpers, cfg_ of the scene replan's submit, transport_replan's own ``config`` that its callers fill from the
    helpers). A new call fails here until it is classified."""
    segments = sorted(ast.get_source_segment(SOURCE, n.args[2]) for n in calls_named("plan_transport_leg"))
    assert segments == sorted(["config", "lcfg_", "lcfg_", "lcfg_", "lcfg_", "ltight_", "cfg_",
                               "replace(lcfg_, clearance_m=0.0)"])


def test_every_mission_plan_carries_the_transport_stage_keyword():
    for node in calls_named("plan_transport"):
        src = ast.get_source_segment(SOURCE, node)
        assert "loaded_mission_kwargs" in src, (node.lineno, src[:120])


def test_transport_replans_are_called_with_the_rules_configs():
    assert "transport_replan(start, *self.loaded_planner_configs())" in SOURCE
    assert "transport_replan(start, *self.loaded_tight_configs(tight, tight_travel))" in SOURCE
    assert not re.search(r"transport_replan\(start, planner_config", SOURCE)


def test_the_scene_replan_and_the_reapproach_take_the_rule_in_transport():
    assert 'core.loaded_planner_configs() if ms.phase == "transport"' in SOURCE
    assert "config if config is not None else self.loaded_planner_configs()[0]" in SOURCE
