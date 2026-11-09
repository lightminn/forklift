"""Diagnostic dumps for occupied holds and blocked scene-replan candidates (docs/validation/2026-11-09-live-riib-v2-
lifecycle4-confirmation.md 2: A seed 0 left no dump; B seed 1's four dumps fell within 0.03 s of the hold's start)."""

from types import SimpleNamespace

import numpy as np

from forklift_core.mission.occupied_dumps import DumpBudget, HoldDumps, RefusalPacer, blocked_candidate


def test_a_hold_is_dumped_at_its_start_and_registered_ages_once_each():
    d = HoldDumps()
    fired = [(t, d.tick(t, True)) for t in (0.0, 0.5, 1.0, 1.01, 4.9, 5.0, 19.99, 20.0, 59.0, 60.0, 61.0)]
    assert [(t, a) for t, a in fired if a is not None] == [(0.0, 0.0), (1.0, 1.0), (5.0, 5.0), (20.0, 20.0),
                                                            (60.0, 60.0)]
    assert d.since == 0.0


def test_a_release_ends_the_episode_and_is_due_after_a_second():
    d = HoldDumps(ages_s=(0.0, 1.0, 5.0))
    assert d.tick(10.0, True) == 0.0 and d.tick(10.5, True) is None and d.release_due is False
    assert d.tick(11.2, True) == 1.0
    assert d.tick(11.3, False) is None and d.release_due is True and abs(d.released_after_s - 1.2) < 1e-9
    d.release_taken()
    assert d.release_due is False
    d.tick(20.0, True)
    d.tick(20.4, False)
    assert d.release_due is False and d.episode == 2  # a 0.4 s blip: no release dump


def test_a_long_jump_fires_one_age_a_tick():
    d = HoldDumps(ages_s=(1.0, 5.0, 20.0))
    assert d.tick(0.0, True) is None
    assert d.tick(30.0, True) == 1.0 and d.tick(30.01, True) == 5.0 and d.tick(30.02, True) == 20.0
    assert d.tick(31.0, True) is None


def test_a_new_mission_starts_no_episode():
    d = HoldDumps(ages_s=(1.0, 5.0))
    assert d.tick(10.0, True) is None and d.tick(11.0, True) == 1.0
    d.new_mission()
    assert d.since is None and d.tick(100.0, True) is None  # not the old episode's 5 s at once (Codex P2-4)
    assert d.tick(101.0, True) == 1.0 and d.count == 2 and d.release_due is False


def test_the_budget_is_a_missions_under_the_runs_cap():
    b = DumpBudget(per_mission=2, per_run=3)
    assert b.take() and b.take() and not b.take()
    b.new_mission()
    assert b.take() and not b.take()
    assert b.drops == 2


def test_refusals_are_paced_per_key():
    p = RefusalPacer(every_s=1.0)
    assert p.due("r1", 162.70) and not p.due("r1", 163.20) and p.due("r1", 163.71) and p.due("r1", 165.72)
    assert p.due("r2", 165.80)


def test_a_blocked_candidate_is_a_refusal_for_a_blocked_reason():
    assert blocked_candidate({"ok": False, "why": "blocked:occupied"})
    assert blocked_candidate({"ok": False, "why": "blocked:unknown"})
    assert not blocked_candidate({"ok": False, "why": "unverified"})
    assert not blocked_candidate({"ok": True, "why": "ok"})
    assert not blocked_candidate(None)


class _Fake:
    """Enough of a MissionCore for its dump methods (bound from the class)."""

    def __init__(self, saved, snapshot=True):
        from forklift_core.mission.core import MissionCore

        snap = SimpleNamespace(state=np.zeros((4, 4), np.int8), free_stamp=np.zeros((4, 4)), origin_x_m=-1.0,
                               origin_y_m=-2.0, resolution_m=0.05, stamp_s=10.0)
        self.reads = []

        def check_candidate(*a, **k):  # the diagnostic must never re-walk with the layer's own permission (P2-1)
            self.reads.append(k)
            raise AssertionError("check_candidate read")

        self.obstacle = {"layer": SimpleNamespace(snapshot=snap if snapshot else None, check_candidate=check_candidate),
                         "applied": (0.0, 0.0, 0.0), "version": 7}
        self.ms = SimpleNamespace(phase="transport")
        self.state = {}
        self.commands = SimpleNamespace(mission_id="m2")
        self.hooks = SimpleNamespace(save_npz=lambda name, **arrays: saved.append((name, arrays)),
                                     truth_rear=lambda: (0.0, 0.0, 0.0), wall=lambda: 0.0)
        self.options = SimpleNamespace(occupied_dumps=True)
        for name in ("_occupied_state", "_occupied_dump", "_occupied_write", "_candidate_dump",
                     "_occupied_new_mission"):
            setattr(self, name, getattr(MissionCore, name).__get__(self))


def test_a_blocked_candidate_is_dumped_with_the_permits_own_cells_and_listed():
    import json

    saved = []
    f = _Fake(saved)
    permit = {"ok": False, "why": "blocked:occupied", "verified_m": 0.3,
              "cells": {"walk": [[1, 2]], "ahead": [[3, 3]], "zone": [[5, 6], [5, 7]]}}
    f._candidate_dump("candidate", 162.7, (1.0, -8.0, -0.2), permit, leg=np.zeros((1, 3)), direction=1, loaded=True)
    assert [n for n, _ in saved] == ["candidate_dump_1.npz"]
    a = saved[0][1]
    assert a["blocked_cells"].tolist() == [[1, 2]] and a["path_blocked_cells"].tolist() == [[3, 3]]
    assert a["zone_blocked_cells"].tolist() == [[5, 6], [5, 7]] and str(a["mission_id"]) == "m2"
    assert json.loads(a["permit_json"]) == {"ok": False, "why": "blocked:occupied", "verified_m": 0.3}
    assert f.state["occupied_dumps"]["files"][0]["name"] == "candidate_dump_1.npz" and f.reads == []


def test_refusals_are_paced_unblocked_and_ok_permits_skipped_and_the_budget_listed_as_drops():
    from forklift_core.mission.occupied_dumps import PER_MISSION

    saved = []
    f = _Fake(saved)
    f._candidate_dump("candidate", 1.0, (0, 0, 0), {"ok": False, "why": "unverified"})
    f._candidate_dump("candidate", 1.0, (0, 0, 0), {"ok": True, "why": "ok"})
    assert saved == []
    for k in range(10):
        f._candidate_dump("candidate", 2.0 + 0.25 * k, (0, 0, 0), {"ok": False, "why": "blocked:unknown"})
    assert [n for n, _ in saved] == ["candidate_dump_1.npz", "candidate_dump_2.npz", "candidate_dump_3.npz"]
    for k in range(PER_MISSION + 2):
        f._occupied_dump("hold", 10.0 + k, (0, 0, 0), (0, 0, 0))
    drops = f.state["occupied_dumps"]["drops"]
    assert len(saved) == PER_MISSION and len(drops) == 5 and drops[0]["reason"] == "budget"


def test_an_escape_with_no_choice_is_dumped_and_a_missing_snapshot_is_a_listed_drop():
    saved = []
    f = _Fake(saved)
    f._candidate_dump("escape", 156.1, (1.9, -8.7, -0.44), None, candidates_json='{"candidates": []}')
    assert saved[0][0] == "escape_dump_1.npz" and saved[0][1]["blocked_cells"].shape == (0, 2)
    g = _Fake([], snapshot=False)
    g._occupied_dump("hold", 1.0, (0, 0, 0), (0, 0, 0))
    assert g.state["occupied_dumps"]["drops"][0]["reason"] == "no_snapshot"


def test_a_new_mission_resets_the_episode_and_budget_and_dumps_the_start():
    saved = []
    f = _Fake(saved)
    occ = f._occupied_state()
    occ["holds"].tick(10.0, True)
    occ["budget"].mission = 5
    f.hooks.truth_rear = lambda: (-1.27, 0.85, 0.17)
    f._occupied_new_mission(30.0, SimpleNamespace(poses=np.array([[-1.28, 0.87, 0.17], [-1.0, 0.9, 0.17]])))
    assert occ["holds"].since is None and occ["budget"].mission == 1  # the start dump took one
    name, a = saved[-1]
    assert name == "mission_start_dump_1.npz" and str(a["rear_source"]) == "plan_start"
    assert a["rear"].tolist() == [-1.28, 0.87, 0.17]
    assert a["truth_rear"].tolist() == [-1.27, 0.85, 0.17]  # not forced equal


def test_a_failed_write_is_listed_and_never_ends_the_run():
    def boom(name, **arrays):
        raise OSError("disk full")

    f = _Fake([])
    f.hooks.save_npz = boom
    f._occupied_dump("hold", 1.0, (0, 0, 0), (0, 0, 0))  # no exception
    entry = f.state["occupied_dumps"]["files"][0]
    assert entry["error"] == "OSError: disk full" and entry["write_s"] >= 0.0
    saved = []
    g = _Fake(saved)
    g._occupied_dump("hold", 1.0, (0, 0, 0), (0, 0, 0))
    first = g.state["occupied_dumps"]["files"][0]
    assert "error" not in first and first["write_s"] >= 0.0
