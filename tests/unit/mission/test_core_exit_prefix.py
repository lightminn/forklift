"""The permission on the transport's exit prefix (docs/plans/2026-10-22-riibotics-trailer-exit.md D1, Codex 2nd P1): a
short transport's delivery-straight window -- after arm_docking splits the straight off -- must not reach back into the
straight reverse the exit prefix put first; the docked straight keeps its own exception."""

from types import SimpleNamespace

import numpy as np

from forklift_core.control import RearAxlePathTracker, TrackerConfig
from forklift_core.planning import Footprint, Rectangle
from forklift_core.planning.pallet_mission import ExitPrefix

from test_core_pause import make  # noqa: E402  (the paused tick's fixtures)

LOADED = Footprint(1.56, 0.17, 0.40)
WALL = Rectangle(1.9, 0.8, 1.0, 0.4)  # x 1.4-2.4, y 0.6-1.0: the loaded front starts beside it
EXIT = ExitPrefix((WALL,), 0.30)


def path(*segments, start=(0.0, 0.0)):
    """Straight segments (length, direction) along x from start, 0.02 m apart."""
    xs, ds = [start[0]], [segments[0][1]]
    for length, direction in segments:
        n = int(round(length / 0.02))
        for _ in range(n):
            xs.append(xs[-1] + direction * 0.02)
            ds.append(direction)
    poses = np.column_stack((xs, np.full(len(xs), start[1]), np.zeros(len(xs))))
    return RearAxlePathTracker(poses, np.asarray(ds, float), np.zeros(len(xs)), TrackerConfig())


def transport_core(tracker, exit_prefix=EXIT):
    core, _, _ = make("transport", tracker=tracker)
    core.geometry = SimpleNamespace(alignment_straight_m=1.0, delivery_straight_m=1.5, loaded_footprint=LOADED)
    core.hooks.exit_prefix = exit_prefix
    return core


def test_the_prefix_length_matches_the_planners():
    from forklift_core.planning import Pose2D
    from forklift_core.planning.pallet_mission import exit_prefix_length_m

    core = transport_core(path((0.40, -1)))
    assert core.exit_prefix_m() == exit_prefix_length_m(Pose2D(0, 0, 0), LOADED, EXIT) > 0


def test_a_short_transports_prefix_keeps_the_permission_after_the_split():
    # the transport after arm_docking: only the prefix left before the docked straight, 0.40 m <= the 1.5 m window
    core = transport_core(path((0.40, -1)))
    acting, docking = core.permission_acting()
    assert acting and not docking
    without = transport_core(path((0.40, -1)), exit_prefix=None)
    assert without.permission_acting() == (False, True)  # what the exception changes


def test_past_the_prefix_the_delivery_straight_exception_is_as_before():
    core = transport_core(path((0.40, -1)))
    core.trackers["transport"]._progress = 0.40
    assert core.permission_acting() == (False, True)


def test_the_docked_straight_has_no_prefix():
    core = transport_core(path((1.4, 1), start=(-0.4, 0.0)))  # inside the 1.5 m window
    assert core.exit_prefix_m() == 0.0 and core.permission_acting() == (False, True)


def test_a_path_that_does_not_start_with_the_prefix_gets_none():
    core = transport_core(path((0.10, -1), (0.30, 1)))  # reverses only 0.10 m
    assert core.exit_prefix_m() == 0.0


def test_a_replan_gets_its_own_prefix():
    core = transport_core(path((0.40, -1)))
    first = core.exit_prefix_m()
    core.trackers["transport"] = path((0.20, -1), start=(-0.4, 0.0))  # replanned from where it stopped, clear already
    assert first > 0 and core.exit_prefix_m() == 0.0
