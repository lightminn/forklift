"""Ordered prefix comparison of two boundary records (M-a1 per-slice evidence, plan M-a1 row)."""

import importlib.util
from pathlib import Path

import numpy as np

from forklift_core.mission.boundary_record import BoundaryRecorder

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("boundary_diff", ROOT / "tools/boundary_diff.py")
D = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(D)


def record(path, ticks):
    """ticks: list of (calls, outputs, branches); calls: (port, method, request, response)."""
    rec = BoundaryRecorder(path, chunk_ticks=2)
    for i, (calls, outputs, branches) in enumerate(ticks):
        rec.tick_begin(i, tick_t=i / 120, work_t=i / 120, commands=[], stop_kind="none")
        for port, method, request, response in calls:
            rec.call(port, method, request=request, response=response)
        rec.tick_end(outputs=outputs, branches=branches)
    rec.close()
    return path


def base(capture_attempts=11, plan_goal=1.0, slam_wall=0.01, extra_branch=None):
    t0 = ([("Slam", "mode", {"get": True}, "tracking"),
           ("Planner", "plan_transport", {"args": [plan_goal]}, {"success": True, "slam_wall_s": slam_wall})],
          {"speed_mps": 0.1}, ["state.transitions[from=observe,to=approach]"])
    t1 = ([("Vehicle", "get_world_pose", {"args": []}, [1.0, 2.0]),
           ("Capture", "capture", {"kwargs": {"max_attempts": 200}}, [{"depth": np.zeros(3)}, None, capture_attempts])],
          {"speed_mps": 0.0}, ["slam.holds[event=hold]"] + ([extra_branch] if extra_branch else []))
    t2 = ([("Vehicle", "get_world_pose", {"args": []}, [1.1, 2.0])], {"speed_mps": 0.0}, [])
    return [t0, t1, t2]


def test_identical_records_are_equal_and_every_branch_counts(tmp_path):
    a, b = record(tmp_path / "a", base()), record(tmp_path / "b", base(slam_wall=0.07))  # wall field ignored
    rep = D.compare(a, b)
    assert rep["status"] == "equal" and rep["ticks_compared"] == 3
    assert rep["branches_covered"] == ["slam.holds[event=hold]", "state.transitions[from=observe,to=approach]"]


def test_a_known_external_difference_cuts_the_prefix_and_drops_that_ticks_branches(tmp_path):
    a = record(tmp_path / "a", base(capture_attempts=11))
    b = record(tmp_path / "b", base(capture_attempts=12, extra_branch="near.events[kind=x]"))
    rep = D.compare(a, b)
    assert rep["status"] == "prefix_equal"
    assert rep["cutoff"] == {"tick": 1, "seq": 1, "port": "Capture", "method": "capture", "what": "response"}
    assert rep["ticks_compared"] == 1
    assert rep["branches_covered"] == ["state.transitions[from=observe,to=approach]"]  # the cut tick excluded


def test_a_request_or_order_difference_fails_even_at_a_known_call(tmp_path):
    a = record(tmp_path / "a", base(plan_goal=1.0))
    b = record(tmp_path / "b", base(plan_goal=1.5))
    rep = D.compare(a, b)
    assert rep["status"] == "differ" and rep["first_difference"]["what"] == "request"
    assert (rep["first_difference"]["port"], rep["first_difference"]["method"]) == ("Planner", "plan_transport")
    # an unknown response difference fails too
    a2 = record(tmp_path / "a2", base())
    t = base()
    t[2] = ([("Vehicle", "get_world_pose", {"args": []}, [9.9, 2.0])], {"speed_mps": 0.0}, [])
    rep = D.compare(a2, record(tmp_path / "b2", t))
    assert rep["status"] == "differ" and rep["first_difference"]["what"] == "response"


def test_an_incomplete_record_is_never_equal(tmp_path):
    a = record(tmp_path / "a", base())
    b = record(tmp_path / "b", base()[:2])
    rep = D.compare(a, b)
    assert rep["status"] == "differ" and "length" in rep["first_difference"]["what"]
    (tmp_path / "b" / "boundary_meta.json").unlink()
    rep = D.compare(a, tmp_path / "b")
    assert rep["status"] == "incomplete"
