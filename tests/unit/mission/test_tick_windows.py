"""Sim-to-real plan 2026-11-18 PR 2: the core's tick-count windows are times; at Isaac's 120 Hz every count and boundary
is the one it had (1 s -> 120 with the 119 preset and the % 120 re-judgement, 0.30 s -> 36), at the real truck's 50 Hz
the same times."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from forklift_core.mission.core import MissionCore, ticks_for

CORE = Path(__file__).resolve().parents[3] / "src" / "forklift_core" / "mission" / "core.py"


@pytest.mark.parametrize("seconds,dt,ticks", [
    (1.0, 1 / 120, 120), (0.30, 1 / 120, 36),  # Isaac: the old literals
    (1.0, 1 / 50, 50), (0.30, 1 / 50, 15),  # the real 50 Hz loop
])
def test_ticks_for(seconds, dt, ticks):
    assert ticks_for(seconds, dt) == ticks


def test_isaac_keeps_every_old_boundary():
    dt = 1 / 120
    assert ticks_for(1.0, dt) - 1 == 119  # the SLAM-stall preset that fires on the same tick
    assert ticks_for(MissionCore.PATH_BLOCKED_S, dt) == 36
    assert ticks_for(MissionCore.GOAL_CREEP_WINDOW_S, dt) == 120


def test_no_tick_literal_window_is_left_in_the_core():
    src = CORE.read_text()
    assert "GOAL_CREEP_WINDOW_TICKS" not in src
    left = re.findall(r"_ticks(?:\"\])?\s*(?:>=|>|%)\s*(?:119|120|36)\b|max\(ms\.slam_stall_ticks,\s*119\)", src)
    assert left == []
