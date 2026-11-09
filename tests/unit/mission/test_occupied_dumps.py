"""Diagnostic dumps for occupied holds and blocked scene-replan candidates (docs/validation/2026-11-09-live-riib-v2-
lifecycle4-confirmation.md 2: A seed 0 left no dump; B seed 1's four dumps fell within 0.03 s of the hold's start)."""

from forklift_core.mission.occupied_dumps import HoldDumps, blocked_candidate


def test_a_hold_is_dumped_at_its_registered_ages_once_each():
    d = HoldDumps()
    fired = [(t, d.tick(t, True)) for t in (0.0, 0.5, 1.0, 1.01, 4.9, 5.0, 19.99, 20.0, 59.0, 60.0, 61.0)]
    assert [(t, a) for t, a in fired if a is not None] == [(1.0, 1.0), (5.0, 5.0), (20.0, 20.0), (60.0, 60.0)]
    assert d.since == 0.0


def test_a_release_starts_a_new_episode():
    d = HoldDumps()
    assert d.tick(0.0, True) is None and d.tick(1.0, True) == 1.0
    assert d.tick(1.5, False) is None and d.since is None
    assert d.tick(2.0, True) is None and d.tick(3.0, True) == 1.0  # the new episode's own 1 s
    assert d.episode == 2


def test_a_long_jump_fires_one_age_a_tick_and_the_run_is_capped():
    d = HoldDumps(max_dumps=3)
    assert d.tick(0.0, True) is None
    assert d.tick(30.0, True) == 1.0 and d.tick(30.01, True) == 5.0 and d.tick(30.02, True) == 20.0
    assert d.tick(60.0, True) is None  # the cap
    assert d.count == 3


def test_a_blocked_candidate_is_a_refusal_for_a_blocked_reason():
    assert blocked_candidate({"ok": False, "why": "blocked:occupied"})
    assert blocked_candidate({"ok": False, "why": "blocked:unknown"})
    assert not blocked_candidate({"ok": False, "why": "unverified"})
    assert not blocked_candidate({"ok": True, "why": "ok"})
    assert not blocked_candidate(None)


def test_a_new_mission_starts_no_episode_but_keeps_the_runs_count():
    d = HoldDumps()
    assert d.tick(10.0, True) is None and d.tick(11.0, True) == 1.0
    d.new_mission()
    assert d.since is None and d.tick(100.0, True) is None  # not the old episode's 5 s at once (Codex P2-4)
    assert d.tick(101.0, True) == 1.0 and d.count == 2


def _fake_core(saved):
    from types import SimpleNamespace

    import numpy as np

    from forklift_core.mission.core import MissionCore

    snap = SimpleNamespace(state=np.zeros((4, 4), np.int8), free_stamp=np.zeros((4, 4)), origin_x_m=-1.0,
                           origin_y_m=-2.0, resolution_m=0.05, stamp_s=10.0)
    reads = []

    def check_candidate(*a, **k):  # the diagnostic must never re-walk with the layer's own permission (Codex P2-1)
        reads.append(k)
        raise AssertionError("check_candidate read")

    fake = SimpleNamespace(
        obstacle={"layer": SimpleNamespace(snapshot=snap, check_candidate=check_candidate), "applied": (0.0, 0.0, 0.0),
                  "version": 7},
        ms=SimpleNamespace(phase="transport"),
        hooks=SimpleNamespace(save_npz=lambda name, **arrays: saved.append((name, arrays)),
                              truth_rear=lambda: (0.0, 0.0, 0.0)),
        options=SimpleNamespace(occupied_dumps=True))
    fake._occupied_dump = lambda *a, **k: MissionCore._occupied_dump(fake, *a, **k)
    return fake, MissionCore, reads


def test_a_blocked_candidate_is_dumped_with_the_permits_own_cells():
    import json

    saved = []
    fake, core, reads = _fake_core(saved)
    permit = {"ok": False, "why": "blocked:occupied", "verified_m": 0.3,
              "cells": {"walk": [[1, 2]], "ahead": [[3, 3]], "zone": [[5, 6], [5, 7]]}}
    core._candidate_dump(fake, "candidate", 162.7, (1.0, -8.0, -0.2), permit, leg=[[1.0, -8.0, -0.2]], direction=1,
                         loaded=True)
    assert len(saved) == 1 and saved[0][0] == "candidate_dump_1.npz"
    arrays = saved[0][1]
    assert arrays["blocked_cells"].tolist() == [[1, 2]] and arrays["path_blocked_cells"].tolist() == [[3, 3]]
    assert arrays["zone_blocked_cells"].tolist() == [[5, 6], [5, 7]]
    assert json.loads(arrays["permit_json"]) == {"ok": False, "why": "blocked:occupied", "verified_m": 0.3}
    assert arrays["phase"] == "transport" and reads == []


def test_an_unblocked_refusal_or_an_ok_permit_is_not_dumped_and_the_run_is_capped():
    from forklift_core.mission.occupied_dumps import MAX_CANDIDATE_DUMPS

    saved = []
    fake, core, reads = _fake_core(saved)
    core._candidate_dump(fake, "candidate", 1.0, (0, 0, 0), {"ok": False, "why": "unverified"})
    core._candidate_dump(fake, "candidate", 1.0, (0, 0, 0), {"ok": True, "why": "ok"})
    assert saved == []
    for _ in range(MAX_CANDIDATE_DUMPS + 3):
        core._candidate_dump(fake, "candidate", 1.0, (0, 0, 0), {"ok": False, "why": "blocked:unknown"})
    assert len(saved) == MAX_CANDIDATE_DUMPS and reads == []


def test_an_escape_with_no_choice_is_dumped_without_a_permit():
    saved = []
    fake, core, reads = _fake_core(saved)
    core._candidate_dump(fake, "escape", 156.1, (1.9, -8.7, -0.44), None, candidates_json='{"candidates": []}')
    assert saved[0][0] == "escape_dump_1.npz" and saved[0][1]["why"] == "None"
    assert saved[0][1]["blocked_cells"].shape == (0, 2) and reads == []
