"""Exit-prefix exemption, the mission core's latch (docs/plans/2026-10-28-exit-prefix-extract-rules.md D1, C1): pending
at the extract -> transport transition, armed at the transport's first permission tick, active on the layer's first
verification, closed for good at the common boundary or by a zero on the exempt snapshot at the permission tick's end
-- each close clears the layer and refreshes it untransformed before anything else reads it."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.mission.core import MissionCore
from forklift_core.mission.state import MissionState

REAR = (1.0, 2.0, 0.0)


class Layer:
    def __init__(self, calls):
        self.calls = calls
        self.answer = (0.3, "ok")
        self.status: dict = {}
        self.permission = SimpleNamespace(path_check=None)
        self.known_enabled, self.known_config = False, {}
        self.snapshot = SimpleNamespace(state=np.zeros((2, 2), np.uint8), free_stamp=np.zeros((2, 2)), origin_x_m=0.0,
                                        origin_y_m=0.0, resolution_m=0.05, stamp_s=1.0)

    def refresh(self, *args, **kwargs):
        self.calls.append(("refresh", args, kwargs))

    def limit(self, *args, **kwargs):
        self.calls.append(("limit", args, kwargs))
        return self.answer

    def exit_exempt_set(self, window_rel, **kwargs):
        self.calls.append(("exit_exempt_set", (np.asarray(window_rel),), kwargs))

    def exit_exempt_status(self):
        self.calls.append(("exit_exempt_status", (), {}))
        return dict(self.status)

    def exit_exempt_clear(self, reason):
        self.calls.append(("exit_exempt_clear", (reason,), {}))
        self.status = {"closed": reason}


class Tracker:
    """A straight reverse from REAR, 2 m long."""

    def __init__(self):
        s = np.linspace(0.0, 2.0, 101)
        self.poses = np.column_stack((REAR[0] - s, np.full(101, REAR[1]), np.zeros(101)))
        self.progress = 0.0

    def progress_m(self):
        return self.progress

    def path_arrays(self):
        return self.poses.copy(), np.full(len(self.poses), -1), np.zeros(len(self.poses))

    def leg_ahead(self):
        return self.poses[:10], -1

    def remaining_to_goal_m(self):
        return 2.0 - self.progress


def make(*, prefix=0.35, option="on"):
    calls = []

    def require(ok, reason):
        if not ok:
            raise RuntimeError(reason)

    hooks = SimpleNamespace(require=require, truth_speed=lambda: -0.1, truth_rear=lambda: REAR,
                            truth_velocity=lambda: np.array([-0.1, 0.0, 0.0]), lift_position=lambda: 0.0,
                            save_npz=lambda name, **arrays: None)
    layer = Layer(calls)
    obstacle = {"layer": layer, "applied": (0.0, 0.0, 0.0), "version": 2, "ticks": {}, "min_allowed": {},
                "slowed": {}, "reasons": {}}
    ms = MissionState(phase="transport", phase_started=0.0)
    options = SimpleNamespace(grid_planning=False, obstacle_act=True, pocket_check=False, mid_still_decel_mps2=0.8,
                              exit_prefix_exempt=option)
    tracker = Tracker()
    core = MissionCore(ms, state={"transitions": []}, obstacle=obstacle, slam=None, pocket={"check": None}, paths={},
                       trackers={"transport": tracker},
                       geometry=SimpleNamespace(alignment_straight_m=1.0, delivery_straight_m=0.8),
                       settings={}, pallet_geometry=None, hooks=hooks, options=options)
    core.exit_prefix_m = lambda: prefix
    core._exit_latch = {"state": "pending", "transition_s": 9.0}  # as transition() leaves it (tested below)
    return core, layer, tracker, calls


def tick(core, t, rear=REAR, kappa=0.0, speed=-0.18):
    return core.permission(t, np.asarray(rear), kappa, speed, True, None)


def names(calls):
    return [c[0] for c in calls if c[0] != "exit_exempt_status"]


def closed(core):
    return core.state.get("exit_prefix_exempt", [])


def test_the_extract_to_transport_transition_leaves_a_pending_latch_with_the_option_on_only():
    for option, expected in (("on", "pending"), ("off", None)):
        core, *_ = make(option=option)
        core._exit_latch = None
        core.ms.phase = "extract"
        core.hooks.echo = lambda *_: None
        core.hooks.record_json = lambda v: str(v)
        core.hooks.snapshot = lambda *_: None
        core.cargo = SimpleNamespace(on_transition=lambda *a: None)
        core.commands = SimpleNamespace(mission=None)
        core.transition("transport", 9.0)
        assert (core._exit_latch or {}).get("state") == expected


def test_the_first_transport_permission_tick_arms_it_sets_the_region_and_caps_the_speed():
    core, layer, tracker, calls = make()
    out = tick(core, 10.0)
    latch = core._exit_latch
    assert latch["state"] == "armed" and latch["length_m"] == pytest.approx(0.35) and latch["tracker"] is tracker
    (window,), kw = next((c[1], c[2]) for c in calls if c[0] == "exit_exempt_set")
    assert window[0] == pytest.approx([0.0, 0.0, 0.0]) and window[-1][0] == pytest.approx(-0.35)  # relative, to L
    assert kw == {"measured_kappa": 0.0, "commanded_kappa": 0.0, "direction": -1}
    assert out.allowed == pytest.approx(0.18)  # the extract's cruise while exempt


def test_the_length_is_capped_at_0_6_m():
    core, *_ = make(prefix=0.9)
    tick(core, 10.0)
    assert core._exit_latch["length_m"] == pytest.approx(0.6)


def test_no_prefix_closes_it_without_touching_the_layer():
    core, layer, _, calls = make(prefix=0.0)
    tick(core, 10.0)
    assert core._exit_latch["state"] == "closed" and closed(core)[0]["reason"] == "no_prefix"
    assert "exit_exempt_set" not in names(calls) and "exit_exempt_clear" not in names(calls)


def test_a_zero_while_awaiting_the_first_verification_does_not_close_it_but_a_second_without_one_does():
    # Codex 3rd review P2-2: the stop the exemption is for comes before the first verification
    core, layer, _, calls = make()
    layer.answer = (0.0, "occupied")
    for k in range(9):
        tick(core, 10.0 + 0.1 * k)
    assert core._exit_latch["state"] == "armed"
    tick(core, 11.05)
    assert core._exit_latch["state"] == "closed" and closed(core)[0]["reason"] == "no_verification"
    i = names(calls).index("exit_exempt_clear")
    assert names(calls)[i + 1] == "refresh" and names(calls)[i + 2] == "limit"  # untransformed before this tick's limit


def test_the_first_verification_makes_it_active_and_a_zero_on_the_exempt_snapshot_closes_it_at_the_tick_end():
    core, layer, _, calls = make()
    tick(core, 10.0)
    layer.status = {"verified_s": 10.05, "verified_version": 2, "transformed": True, "skipped": None, "freed": 7}
    tick(core, 10.1)
    assert core._exit_latch["state"] == "active"
    layer.answer = (0.0, "unknown")
    n = len(calls)
    out = tick(core, 10.2)
    assert out.allowed == 0.0  # the tick decided once, on the exempt snapshot
    assert core._exit_latch["state"] == "closed" and closed(core)[0]["reason"] == "zero:unknown"
    tail = names(calls[n:])
    assert tail[-2:] == ["exit_exempt_clear", "refresh"] and tail.count("limit") == 1


def test_a_restart_confirmation_zero_does_not_close_it(monkeypatch):
    # Codex 4th review P2-1: the raw limit is positive, the confirmation holds the output at zero for its scans
    core, layer, _, _ = make()
    core.options.restart_confirm_scans = 3
    monkeypatch.setattr(core, "_restart_gate", lambda t, allowed, why: (0.0, "occupied", True))
    tick(core, 10.0)
    layer.status = {"verified_s": 10.05, "verified_version": 2, "transformed": True, "skipped": None, "freed": 7}
    for k in range(3):
        out = tick(core, 10.1 + 0.1 * k)
        assert out.allowed == 0.0
    assert core._exit_latch["state"] == "active"


def test_a_ring_closes_it_at_the_head_so_this_tick_judges_on_the_general_snapshot():
    core, layer, _, calls = make()
    tick(core, 10.0)
    layer.status = {"verified_s": 10.05, "verified_version": 2, "transformed": True, "skipped": None,
                    "rang": {"stamp_s": 10.15, "hits": 3, "first": []}}
    n = len(calls)
    out = tick(core, 10.2)
    assert closed(core)[0]["reason"] == "watch" and closed(core)[0]["ring"]["hits"] == 3
    assert names(calls[n:])[:3] == ["exit_exempt_clear", "refresh", "limit"]
    # Codex impl review P1-2: the ringing tick stops the wheels though the general limit beyond the arc is positive
    assert (out.allowed, out.why, out.requested_speed, out.obstacle_hold) == (0.0, "exit_watch", 0.0, True)
    assert core.obstacle["permission_cap"] == (10.2, 0.0)
    assert tick(core, 10.3).allowed == pytest.approx(0.3)  # the next tick: the general limit, no exempt cap


def test_the_exempt_part_is_measured_from_the_paths_start():
    # Codex impl review P2-3: a tracker projected 0.03 m along when armed keeps the end at 0.35 m, not 0.38 m
    core, layer, tracker, calls = make()
    tracker.progress = 0.03
    tick(core, 10.0)
    (window,), _ = next((c[1], c[2]) for c in calls if c[0] == "exit_exempt_set")
    assert window[0][0] == pytest.approx(-0.03) and window[-1][0] == pytest.approx(-0.35)  # 0.03..0.35 along the path
    assert core._exit_latch["budget_m"] == pytest.approx(0.32)
    tracker.progress = 0.35
    tick(core, 10.1)
    assert closed(core)[0]["reason"] == "length"
    core, layer, tracker, calls = make()
    tracker.progress = 0.36  # armed past the end: nothing exempt
    tick(core, 10.0)
    assert closed(core)[0]["reason"] == "length" and "exit_exempt_set" not in names(calls)


@pytest.mark.parametrize("change, reason", [
    (lambda core, tr: setattr(tr, "progress", 0.35), "length"),
    (lambda core, tr: core.trackers.__setitem__("transport", Tracker()), "tracker"),
    (lambda core, tr: core.obstacle.__setitem__("backoff", {"phase": "transport"}), "backoff"),
])
def test_length_tracker_and_backoff_close_it(change, reason):
    core, layer, tracker, _ = make()
    tick(core, 10.0)
    change(core, tracker)
    tick(core, 10.1)
    assert closed(core)[0]["reason"] == reason


def test_the_distance_driven_and_the_time_close_it():
    core, *_ = make()
    tick(core, 10.0)
    tick(core, 10.1, rear=(1.0 - 0.36, 2.0, 0.0))  # driven 0.36 m while the tracker's progress lags
    assert closed(core)[0]["reason"] == "driven"
    core, layer, *_ = make()
    tick(core, 10.0)
    layer.status = {"verified_s": 10.05, "verified_version": 2, "transformed": True, "skipped": None}
    tick(core, 10.1)
    tick(core, 18.05)
    assert closed(core)[0]["reason"] == "time"


def test_a_lapse_of_the_verification_over_half_a_second_closes_an_active_latch():
    core, layer, *_ = make()
    tick(core, 10.0)
    layer.status = {"verified_s": 10.05, "verified_version": 2, "transformed": True, "skipped": None}
    tick(core, 10.1)
    layer.status = {"verified_s": 10.05, "verified_version": 2, "transformed": False, "skipped": "stale"}
    layer.answer = (0.0, "occupied")  # the halo is back: not a close by itself (not the exempt snapshot)
    for k in range(5):
        tick(core, 10.2 + 0.1 * k)
    assert core._exit_latch["state"] == "active"
    tick(core, 10.75)
    assert closed(core)[0]["reason"] == "verification_lapsed"


def test_the_watch_and_paused_paths_close_it_before_they_judge():
    # Codex 5th review P2: the paths that skip the permission tick close at their own head, clear and refresh first
    for path, t, reason in (("watch", 18.1, "time"), ("paused", 10.1, "paused")):
        core, layer, _, calls = make()
        tick(core, 10.0)
        n = len(calls)
        core.exit_exempt_boundary(t, np.asarray(REAR), True, path)
        assert closed(core)[0]["reason"] == reason
        assert names(calls[n:]) == ["exit_exempt_clear", "refresh"]


def test_the_evaluation_books_exempt_ticks_and_the_settling_after_a_close():
    core, layer, *_ = make()
    tick(core, 10.0)
    assert not core.exit_exempt_counts(-0.18)  # armed: the general snapshot
    layer.status = {"verified_s": 10.05, "verified_version": 2, "transformed": True, "skipped": None}
    tick(core, 10.1)
    assert core.exit_exempt_counts(-0.18)
    layer.status["rang"] = {"stamp_s": 10.15, "hits": 1, "first": []}
    tick(core, 10.2)
    assert core.exit_exempt_counts(-0.12) and core.exit_exempt_counts(-0.05)  # still braking
    assert not core.exit_exempt_counts(-0.005) and not core.exit_exempt_counts(-0.2)  # stood once: over


def test_off_leaves_every_tick_as_it_was():
    core, layer, _, calls = make(option="off")
    core._exit_latch = None
    out = tick(core, 10.0)
    assert out.allowed == pytest.approx(0.3)
    assert not any(c[0].startswith("exit_exempt") for c in calls)


def _close_by_ring(core, layer):
    tick(core, 10.0)
    layer.status = {"verified_s": 10.05, "verified_version": 2, "transformed": True, "skipped": None,
                    "rang": {"stamp_s": 10.15, "hits": 3, "first": []}}
    tick(core, 10.2)
    assert closed(core)[-1]["reason"] == "watch"


def test_a_closed_latch_decides_every_later_tick_as_a_core_without_one():
    """Rule 6 of the next riib batch (lead 2026-10-31, option (가)): instead of a continuation replay, the closed latch is
    shown inert here -- after the close, with the same layer answers, the decisions and the layer calls are those of a
    core that never had a latch."""
    on, on_layer, _, on_calls = make()
    _close_by_ring(on, on_layer)
    off, off_layer, _, off_calls = make(option="off")
    off._exit_latch = None
    answers = [(0.3, "ok"), (0.0, "occupied"), (0.12, "limit"), (0.3, "ok")]
    for k, answer in enumerate(answers):
        t = 10.3 + 0.1 * k
        on_layer.answer = off_layer.answer = answer
        n_on, n_off = len(on_calls), len(off_calls)
        a, b = tick(on, t, speed=-0.05 * k), tick(off, t, speed=-0.05 * k)
        assert (a.allowed, a.why, a.requested_speed, a.obstacle_hold) == (b.allowed, b.why, b.requested_speed,
                                                                         b.obstacle_hold)
        assert names(on_calls[n_on:]) == names(off_calls[n_off:])
        assert on.obstacle.get("permission_cap") == off.obstacle.get("permission_cap")
    assert len(closed(on)) == 1  # it never opened again


def test_the_closing_tick_and_the_settling_after_it_belong_to_the_exemption():
    """The hole a continuation replay would have had (Codex design review P2-3: a close and a failure in the same
    tick): the closing tick decides as the exemption (the ring's stop), and every tick until the truck first stands
    is booked to it -- "after the close" begins only there."""
    core, layer, _, _ = make()
    tick(core, 10.0)
    layer.status = {"verified_s": 10.05, "verified_version": 2, "transformed": True, "skipped": None,
                    "rang": {"stamp_s": 10.15, "hits": 3, "first": []}}
    out = tick(core, 10.2)
    assert (out.allowed, out.why) == (0.0, "exit_watch") and closed(core)[-1]["closed_s"] == pytest.approx(10.2)
    assert core.exit_exempt_counts(-0.06) and core.exit_exempt_counts(-0.02)  # still moving: the exemption's
    assert not core.exit_exempt_counts(-0.005)  # first standing: settled
    assert not core.exit_exempt_counts(-0.08)  # moving again later is no longer the exemption's
