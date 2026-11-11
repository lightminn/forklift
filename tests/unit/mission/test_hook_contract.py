"""Sim-to-real 4d-2a: the mission core's hook contract and the real truck's fail-closed hooks
(docs/design/2026-11-11-p05-truth-audit.md 4-2)."""

import re
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission import core as core_module
from forklift_core.mission.hook_contract import HOOKS, HookClass, fail_closed_hooks
from forklift_core.safety.truth_access import TruthAccessViolation

CORE_SOURCE = Path(core_module.__file__).read_text(encoding="utf-8")


def core_hook_names():
    names = set(re.findall(r"hooks\.([A-Za-z_][A-Za-z0-9_]*)", CORE_SOURCE))
    names |= set(re.findall(r"getattr\((?:self\.)?hooks, \"([A-Za-z_][A-Za-z0-9_]*)\"", CORE_SOURCE))
    names |= set(re.findall(r"self\._truth\(\"([A-Za-z_][A-Za-z0-9_]*)\"\)", CORE_SOURCE))
    return names


def test_every_hook_the_core_reads_is_classified():
    missing = sorted(core_hook_names() - set(HOOKS))
    assert not missing, f"classify these hooks in hook_contract.HOOKS: {missing}"


def test_the_contract_lists_no_hook_the_core_does_not_read():
    stale = sorted(set(HOOKS) - core_hook_names())
    assert not stale, f"no longer read by the core: {stale}"


def test_optional_hooks_are_the_ones_read_with_a_default():
    read_with_default = set(re.findall(r"getattr\((?:self\.)?hooks, \"([A-Za-z_][A-Za-z0-9_]*)\", None\)", CORE_SOURCE))
    for name in read_with_default:
        assert HOOKS[name][1], f"{name} is read with a default: register it optional"


class Base:
    def __init__(self):
        self.reads = []

    def __getattr__(self, name):
        if name.startswith("__"):
            raise AttributeError(name)
        self.reads.append(name)
        if name in ("rear", "now_s", "save_npz", "truth_rear", "pallet_z", "brand_new_truth", "exit_prefix_x"):
            return lambda *a, **k: name
        raise AttributeError(name)


def test_audited_names_pass_and_evaluation_answers_none_without_reading():
    base = Base()
    hooks = fail_closed_hooks(base)
    assert hooks.rear() == "rear" and hooks.now_s() == "now_s" and hooks.save_npz() == "save_npz"
    assert hooks.truth_rear() is None and hooks.slam_error() is None
    assert hooks.record_capture_evaluation(1, 2, x=3) is None
    assert base.reads == ["rear", "now_s", "save_npz"]  # never read the evaluation names


@pytest.mark.parametrize("name", ["pallet_z", "truth_velocity", "truth_yaw_rate", "truth_speed", "stage",
                                  "export_scene", "brand_new_truth"])
def test_truth_human_sim_only_and_unregistered_names_raise(name):
    base = Base()
    with pytest.raises(TruthAccessViolation):
        getattr(fail_closed_hooks(base), name)
    with pytest.raises(TruthAccessViolation):
        getattr(fail_closed_hooks(base), name, None)  # a default does not soften it
    assert name not in base.reads


def test_absent_hooks():
    hooks = fail_closed_hooks(Base())
    assert getattr(hooks, "exit_prefix", None) is None  # an optional one may be absent
    assert getattr(hooks, "test_recovery_blocked", None) is None  # a test tool is absent
    with pytest.raises(TruthAccessViolation):
        hooks.grid_world  # a required one missing is a violation, not an AttributeError a default could hide


def test_writes_only_to_passing_names():
    base = SimpleNamespace()
    hooks = fail_closed_hooks(base)
    hooks.loaded_cargo_ok = lambda: True
    assert base.loaded_cargo_ok() is True
    for name in ("truth_rear", "pallet_z", "never_heard_of"):
        with pytest.raises(TruthAccessViolation):
            setattr(hooks, name, lambda: 0)


def test_the_classes_used():
    classes = {cls for cls, _ in HOOKS.values()}
    assert classes <= set(HookClass)
    assert HOOKS["pallet_z"][0] is HookClass.HUMAN and HOOKS["truth_rear"][0] is HookClass.EVAL


def test_simulation_only_hooks_are_read_only_behind_the_real_truck_check():
    """4d-2a: the stage only through MissionCore._stage, the scene export only off the real truck."""
    assert CORE_SOURCE.count("hooks.stage()") == 1
    assert "return None if self._real_vehicle() else self.hooks.stage()" in CORE_SOURCE
    assert CORE_SOURCE.count("hooks.export_scene()") == 1
    assert re.search(r"if not self\._real_vehicle\(\):[^\n]*\n\s+hooks\.export_scene\(\)", CORE_SOURCE)
