"""The scene-mode replan device's state machine (docs/plans/2026-10-23-scene-replan.md 2.1, 2.4) on fake ports."""

from types import SimpleNamespace

import pytest

from forklift_core.mission.scene_replan import SceneReplan, SceneReplanConfig


class Ports:
    """A truck that stands, a planner answering after ``plan_ticks`` polls, a candidate check and the wheels."""

    def __init__(self, *, stopped=True, busy=False, plans=None, candidates=None, previews=None, escape=None,
                 steer=None, extra=None):
        self.stopped_, self.busy_ = stopped, busy
        self.plans = list(plans or [("ok", "path-1")])
        self.candidates = list(candidates or [{"status": "tracking", "ok": True, "why": "ok", "steer": 0.2}])
        self.previews = list(previews or [{"status": "tracking", "ok": True, "why": "ok", "steer": 0.2}])
        self.escape_ = escape
        self.steer = list(steer or [0.0])
        self.extra = extra if extra is not None else (lambda: [])
        self.log = []
        self.pending = None
        self.escape_running = False
        self.snapshots = 0  # a new snapshot on every ask unless a test freezes it
        self.frozen = False

    def snapshot_id(self):
        if not self.frozen:
            self.snapshots += 1
        return self.snapshots

    def stopped(self):
        return self.stopped_

    def busy(self):
        return self.busy_

    def release_hold(self):
        self.log.append("release")

    def submit(self, extra, clearance_m):
        self.log.append(("submit", tuple(extra), clearance_m))
        self.pending = self.plans.pop(0) if self.plans else ("ok", "path-x")

    def poll(self):
        out, self.pending = self.pending, None
        return out

    def candidate(self, result):
        self.log.append(("candidate", result))
        return self.candidates.pop(0) if len(self.candidates) > 1 else self.candidates[0]

    def install(self, result):
        self.log.append(("install", result))

    def escape_plan(self, extra, clearance_m):
        self.log.append(("escape_plan", clearance_m))
        return self.escape_

    def escape_install(self, path):
        self.log.append(("escape_install", path))
        self.escape_running = True

    def escape_state(self):
        return "running" if self.escape_running else "done"

    def escape_clear(self):
        self.log.append("escape_clear")

    def steering_error(self, target):
        return self.steer.pop(0) if len(self.steer) > 1 else self.steer[0]

    @staticmethod
    def steer_gap(a, b):
        return abs(a - b)

    def preview(self):
        return self.previews.pop(0) if len(self.previews) > 1 else self.previews[0]


def _device(**cfg):
    records = []
    return SceneReplan(records, SceneReplanConfig(**cfg)), records


def _ask(dev, t=10.0, trigger="cusp_pose", **kw):
    return dev.request(t, trigger, phase="transport", tracker_serial=1, leg=0, rear=(0, 0, 0), truth_rear=(0, 0, 0),
                       deviation={"position_m": 0.075, "yaw_rad": -0.05}, correction_jump=None, **kw)


def _run(dev, ports, t0=10.0, n=50, dt=0.1):
    out = []
    for k in range(n):
        r = dev.step(t0 + dt * k, ports)
        out.append(r)
        if r.end or not r.owned:
            break
    return out


def test_a_request_waits_for_the_stop_then_plans_checks_installs_presteers_and_hands_back():
    dev, records = _device()
    ports = Ports(stopped=False, steer=[0.3, 0.1, 0.01])
    assert _ask(dev) is None  # accepted, nothing ends
    first = dev.step(10.0, ports)
    assert first.owned and first.output == "hold" and dev.state == "wait_stop"
    ports.stopped_ = True
    steps = _run(dev, ports, 10.1)
    assert steps[-1].owned is False and steps[-1].end is None  # handed back to the follower
    kinds = [x if isinstance(x, str) else x[0] for x in ports.log]
    assert kinds == ["release", "submit", "candidate", "install"]
    assert any(s.output == "presteer" and s.steer == 0.2 for s in steps)
    rec = records[-1]
    assert rec["trigger"] == "cusp_pose" and rec["outcome"] == "installed" and rec["presteer"] == "aligned"
    assert rec["budget_left"] == {"total": 3, "cusp_pose": 2}


def test_other_jobs_and_a_pending_release_go_first():
    dev, _ = _device()
    ports = Ports(busy=True)
    _ask(dev)
    for k in range(3):
        assert dev.step(10 + 0.1 * k, ports).output == "hold"
    assert "submit" not in [x[0] for x in ports.log if isinstance(x, tuple)]
    ports.busy_ = False
    _run(dev, ports, 11.0)
    assert ("submit", (), None) in ports.log


def test_the_extra_obstacles_are_read_after_the_release_and_an_error_ends_the_attempt():
    dev, records = _device()
    seen = []
    ports = Ports(extra=lambda: (seen.append("called"), ["box"])[1])
    _ask(dev, trigger="permit_stop", extra_obstacles_fn=ports.extra, clearance_m=0.2)
    _run(dev, ports)
    assert ports.log.index("release") < ports.log.index(("submit", ("box",), 0.2))
    dev2, records2 = _device()

    def broken():
        raise ValueError("no snapshot")

    _ask(dev2, trigger="permit_stop", extra_obstacles_fn=broken)
    steps = _run(dev2, Ports())
    # P6: a permit_stop request ending before any install hands the tick back standing (the mission goes on)
    assert steps[-1].owned is False and steps[-1].end is None
    assert records2[-1]["outcome"] == "failed" and records2[-1]["mission_end"] is False
    assert records2[-1]["reason"] == "scene_replan_failed:permit_stop:obstacles_error"


# ---- P6 (permission-clearance plan, Isaac p6_24f83a14): a standing-safe trigger's failure before an install ----

def _permit(dev, **kw):
    return _ask(dev, trigger="permit_stop", extra_obstacles_fn=lambda: ["box"], clearance_m=0.2, **kw)


def test_a_permit_stop_plan_failing_before_an_install_hands_back_standing_and_gives_the_total_back():
    dev, records = _device()
    ports = Ports(plans=[("ok_fail", "invalid_start")], escape=None)  # no escape either
    assert _permit(dev) is None
    steps = _run(dev, ports)
    assert [s.output for s in steps[:-1]] == ["hold"] * (len(steps) - 1)  # held while it worked
    assert steps[-1] == type(steps[-1])(owned=False)  # the tick handed straight back, nothing to end
    assert dev.request_ is None and dev.state is None
    rec = records[-1]
    assert rec["outcome"] == "failed" and rec["mission_end"] is False
    assert rec["reason"] == "scene_replan_failed:permit_stop:invalid_start"
    assert dev.used_total == 0 and dev.used_by_trigger["permit_stop"] == 1  # the total back, its own count kept
    assert not any(isinstance(x, tuple) and x[0] in ("install", "escape_install") for x in ports.log)


def test_a_permit_stop_candidate_refused_before_an_install_hands_back():
    dev, records = _device()
    ports = Ports(candidates=[{"status": "tracking", "ok": False, "why": "blocked:occupied", "steer": 0.1}])
    _permit(dev)
    steps = _run(dev, ports, n=80)
    assert steps[-1].owned is False and steps[-1].end is None
    assert records[-1]["outcome"] == "refused" and records[-1]["mission_end"] is False
    assert records[-1]["reason"] == "scene_replan_refused:permit_stop:blocked:occupied"


def test_a_permit_stop_failing_after_its_install_ends_the_mission():
    # installed, then the resume check never passes: dropped and planned again (the follower's tracker is the new
    # one now), and that plan's failure is the device's end as for any trigger
    dev, records = _device(drops_per_request=1)
    ports = Ports(plans=[("ok", "path-1"), ("ok_fail", "no_path")],
                  previews=[{"status": "tracking", "ok": False, "why": "blocked:occupied", "steer": 0.2}],
                  steer=[0.0])
    _permit(dev)
    steps = _run(dev, ports, n=120)
    assert ("install", "path-1") in ports.log
    assert steps[-1].end == "scene_replan_failed:permit_stop:no_path"
    assert records[-1]["mission_end"] is True


def test_a_permit_stop_escape_that_stops_ends_the_mission():
    dev, records = _device()
    ports = Ports(plans=[("ok_fail", "invalid_start")], escape="escape-path")
    ports.escape_state = lambda: "stopped"
    _permit(dev)
    steps = _run(dev, ports)
    assert ("escape_install", "escape-path") in ports.log
    assert steps[-1].end == "scene_replan_failed:permit_stop:escape_stopped" and records[-1]["mission_end"] is True


def test_the_follower_s_triggers_still_end_the_mission_before_an_install():
    dev, records = _device()
    _ask(dev)
    steps = _run(dev, Ports(plans=[("ok_fail", "invalid_start")], escape=None))
    assert steps[-1].end == "scene_replan_failed:cusp_pose:invalid_start" and records[-1]["mission_end"] is True
    assert dev.used_total == 1


@pytest.mark.parametrize("why, back_to", [("moving", "wait_stop"), ("superseded", "wait_jobs"),
                                          ("correction", "wait_jobs")])
def test_a_dropped_plan_is_asked_again_without_spending_budget(why, back_to):
    dev, records = _device()
    ports = Ports(plans=[("dropped", why), ("ok", "path-2")])
    _ask(dev)
    states = []
    for k in range(30):
        r = dev.step(10 + 0.1 * k, ports)
        states.append(dev.state)
        if not r.owned or r.end:
            break
    assert back_to in states and ("install", "path-2") in ports.log
    assert records[-1]["drops"] == 1 and records[-1]["budget_left"]["total"] == 3


def test_three_drops_end_the_request():
    dev, _ = _device()
    ports = Ports(plans=[("dropped", "correction")] * 4)
    _ask(dev)
    steps = _run(dev, ports, n=100)
    assert steps[-1].end == "scene_replan_failed:cusp_pose:dropped"


def test_a_failed_plan_without_invalid_start_ends_and_invalid_start_escapes_once():
    dev, _ = _device()
    _ask(dev)
    steps = _run(dev, Ports(plans=[("ok_fail", "no_path")]))
    assert steps[-1].end == "scene_replan_failed:cusp_pose:no_path"
    dev, records = _device()
    ports = Ports(plans=[("ok_fail", "invalid_start"), ("ok", "path-2")], escape="backup")
    _ask(dev)
    states = []
    for k in range(60):
        r = dev.step(10 + 0.1 * k, ports)
        states.append((dev.state, r.output))
        if dev.state == "escape" and k > 3:
            ports.escape_running = False  # the prefix driven and stopped
        if not r.owned or r.end:
            break
    assert ("escape", "escape") in states
    assert ("escape_install", "backup") in ports.log and "escape_clear" in ports.log
    assert ("install", "path-2") in ports.log and records[-1]["escape"] == "done"
    dev, _ = _device()
    _ask(dev)
    steps = _run(dev, Ports(plans=[("ok_fail", "invalid_start")], escape=None))
    assert steps[-1].end == "scene_replan_failed:cusp_pose:invalid_start"


def test_the_escape_stopped_by_the_permit_ends_it():
    dev, _ = _device()
    ports = Ports(plans=[("ok_fail", "invalid_start")], escape="backup")
    _ask(dev)
    for k in range(10):
        r = dev.step(10 + 0.1 * k, ports)
        if r.end:
            break
        if dev.state == "escape":
            ports.escape_state = lambda: "stopped"
    assert r.end == "scene_replan_failed:cusp_pose:escape_stopped"


@pytest.mark.parametrize("candidate, end", [
    ({"status": "tracking", "ok": False, "why": "blocked:occupied"}, "scene_replan_refused:cusp_pose:blocked:occupied"),
])
def test_a_refused_candidate_ends_the_request(candidate, end):
    dev, records = _device()
    _ask(dev)
    steps = _run(dev, Ports(candidates=[candidate]))
    assert steps[-1].end == end and records[-1]["outcome"] == "refused"


def test_a_failed_candidate_is_planned_again_and_counts_as_a_drop():
    dev, records = _device()
    ports = Ports(plans=[("ok", "p1"), ("ok", "p2")],
                  candidates=[{"status": "failed", "ok": False, "why": "endpoint_heading"},
                              {"status": "tracking", "ok": True, "why": "ok", "steer": 0.1}])
    _ask(dev)
    _run(dev, ports, n=80)
    assert ("install", "p2") in ports.log and ("install", "p1") not in ports.log and records[-1]["drops"] == 1


def test_an_arrived_candidate_is_installed_and_handed_back_at_once():
    dev, records = _device()
    ports = Ports(candidates=[{"status": "arrived", "ok": True, "why": "arrived"}])
    _ask(dev)
    steps = _run(dev, ports)
    assert ("install", "path-1") in ports.log and steps[-1].owned is False
    assert records[-1]["presteer"] is None


def test_the_resume_check_sends_a_moved_start_back_to_presteer_and_a_lost_permit_back_to_plan():
    dev, records = _device()
    ports = Ports(steer=[0.0], previews=[{"status": "tracking", "ok": True, "why": "ok", "steer": 0.35},
                                         {"status": "tracking", "ok": True, "why": "ok", "steer": 0.35}])
    _ask(dev)
    _run(dev, ports)
    rec = records[-1]
    assert rec["represteers"] == 1 and rec["outcome"] == "installed"
    assert [s for s, _ in rec["states"]].count("presteer") == 2  # the first target, then the moved start's
    # 7판: a permit lost at the start is re-checked standing; back within permit_wait_s it goes on with the same leg
    dev, records = _device()
    ports = Ports(plans=[("ok", "p1"), ("ok", "p2")],
                  previews=[{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}] * 5
                  + [{"status": "tracking", "ok": True, "why": "ok", "steer": 0.2}])
    _ask(dev)
    steps = _run(dev, ports, n=80)
    assert steps[-1].owned is False and ("install", "p2") not in ports.log
    assert records[-1]["permit_waits"][0]["where"] == "resume" and records[-1]["drops"] == 0
    # ... and lost for longer it is planned again
    dev, records = _device()
    ports = Ports(plans=[("ok", "p1"), ("ok", "p2")],
                  previews=[{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}] * 40
                  + [{"status": "tracking", "ok": True, "why": "ok", "steer": 0.2}])
    _ask(dev)
    _run(dev, ports, n=120)
    assert ("install", "p2") in ports.log and records[-1]["drops"] == 1


def test_a_candidate_the_permit_does_not_pass_yet_is_rechecked_standing_then_refused():
    """7판 (Isaac sri_b907b91: blocked:unknown refused at once): the same candidate re-asked every tick."""
    dev, records = _device()
    ports = Ports(candidates=[{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}] * 10
                  + [{"status": "tracking", "ok": True, "why": "ok", "steer": 0.2}])
    _ask(dev)
    steps = _run(dev, ports, n=60)
    assert steps[-1].owned is False and records[-1]["outcome"] == "installed"
    assert ports.log.count(("install", "path-1")) == 1 and ports.log.count(("candidate", "path-1")) == 11
    dev, records = _device()
    ports = Ports(candidates=[{"status": "tracking", "ok": False, "why": "blocked:occupied", "steer": 0.2}])
    _ask(dev)
    steps = _run(dev, ports, n=60)
    assert steps[-1].end == "scene_replan_refused:cusp_pose:blocked:occupied"
    wait = records[-1]["permit_waits"][0]
    assert wait["where"] == "candidate" and wait["until_s"] - wait["since_s"] == pytest.approx(2.9, abs=0.11)
    dev, records = _device()  # a pause does not count in that wait
    ports = Ports(candidates=[{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}])
    _ask(dev)
    _run(dev, ports, n=20)
    dev.shift_time(5.0)
    assert dev.step(10.0 + 7.0, ports).end is None


def test_presteer_times_out_and_goes_on():
    dev, records = _device()
    ports = Ports(steer=[0.5])  # never aligns
    _ask(dev)
    steps = _run(dev, ports, n=60)
    assert steps[-1].owned is False and records[-1]["presteer"] == "timeout"
    assert records[-1]["presteer_error_rad"] == pytest.approx(0.5)


def test_the_request_limit_is_sim_time_including_plan_waits_and_shifted_by_pauses():
    dev, _ = _device()
    ports = Ports(stopped=False)
    _ask(dev, t=0.0)
    assert dev.step(30.0, ports).end is None
    dev.shift_time(40.0)  # a 40 s pause
    assert dev.step(65.0, ports).end is None  # 25 s of request time
    assert dev.step(101.0, ports).end == "scene_replan_failed:cusp_pose:timeout"


def test_the_budget_is_four_per_mission_three_per_trigger_repeats_counted_and_reset_only_at_the_end():
    dev, records = _device()
    for k in range(3):
        assert _ask(dev, t=10.0 * k) is None
        _run(dev, Ports(), t0=10.0 * k)
    assert _ask(dev, t=40.0) == "scene_replan_exhausted:cusp_pose"
    assert _ask(dev, t=41.0, trigger="permit_stop") is None
    _run(dev, Ports(), t0=41.0)
    assert _ask(dev, t=50.0, trigger="cusp_handed") == "scene_replan_exhausted:cusp_handed"  # 4 in all
    dev.reset()
    assert _ask(dev, t=60.0) is None


def test_a_second_request_while_one_runs_is_noted_not_taken():
    dev, records = _device()
    _ask(dev)
    assert _ask(dev, t=10.05, trigger="permit_stop") is None
    assert dev.request_["trigger"] == "cusp_pose" and records[-1]["ignored"] == ["permit_stop"]


def test_a_stop_lost_while_waiting_for_the_other_jobs_waits_for_the_stop_again():
    dev, _ = _device()
    ports = Ports(busy=True)
    _ask(dev)
    dev.step(10.0, ports)
    assert dev.state == "wait_jobs"
    ports.busy_, ports.stopped_ = False, False
    assert dev.step(10.1, ports).output == "hold" and dev.state == "wait_stop"
    assert not any(isinstance(x, tuple) and x[0] == "submit" for x in ports.log)


def test_an_abort_closes_the_record():
    dev, records = _device()
    _ask(dev)
    dev.abort("async_plan_failed")
    assert not dev.active() and records[-1]["outcome"] == "failed" and records[-1]["reason"] == "async_plan_failed"


def test_a_standing_recheck_waits_for_a_new_snapshot_keeps_the_stop_and_restarts_after_a_replan():
    """Codex impl round 5: P3 -- no new snapshot, no new check (only the time runs); P2-2 -- the stop lost while the
    candidate waits goes back to the stop, nothing installed or steered; P2-3 -- a replan ends the start check's wait,
    the new leg's wait starts again."""
    dev, records = _device()
    ports = Ports(candidates=[{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}])
    _ask(dev)
    _run(dev, ports, n=3)
    asked = ports.log.count(("candidate", "path-1"))
    ports.frozen = True
    _run(dev, ports, t0=10.3, n=5)
    assert ports.log.count(("candidate", "path-1")) == asked  # the same snapshot is not checked again
    steps = _run(dev, ports, t0=10.8, n=40)
    assert steps[-1].end == "scene_replan_refused:cusp_pose:start:unknown"  # the time still runs out
    dev, records = _device()
    ports = Ports(candidates=[{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}] * 3
                  + [{"status": "tracking", "ok": True, "why": "ok", "steer": 0.2}])
    _ask(dev)
    _run(dev, ports, n=3)  # waiting: the next answer would pass
    ports.stopped_ = False
    step = dev.step(10.3, ports)
    assert step.output == "hold" and dev.state == "wait_stop" and ("install", "path-1") not in ports.log
    dev, records = _device()
    ports = Ports(plans=[("ok", "p1"), ("ok", "p2")],
                  previews=[{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}] * 2
                  + [{"status": "failed", "ok": False, "why": "failed:endpoint_heading"}]
                  + [{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}] * 3
                  + [{"status": "tracking", "ok": True, "why": "ok", "steer": 0.2}])
    _ask(dev)
    steps = _run(dev, ports, n=80)
    assert ("install", "p2") in ports.log and steps[-1].owned is False and records[-1]["drops"] == 1
    waits = records[-1]["permit_waits"]
    assert [w["where"] for w in waits] == ["resume", "resume"]  # two stretches, not one across the replan


def test_a_replan_starts_the_start_check_s_wait_again_and_a_cached_wait_is_recorded_to_its_end():
    """Codex impl round 6 P3: the old timer would let the new leg's first start check -- after a long replan --
    drop at once; and the ticks a frozen snapshot answers keep the wait's record running."""
    dev, records = _device()
    ports = Ports(plans=[("ok", "p1"), ("ok", "p2"), ("ok", "p3")],
                  previews=[{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}] * 2
                  + [{"status": "failed", "ok": False, "why": "failed:endpoint_heading"}]
                  + [{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}] * 3
                  + [{"status": "tracking", "ok": True, "why": "ok", "steer": 0.2}])
    _ask(dev)
    t, k = 10.0, 0
    while records[-1]["drops"] == 0:
        dev.step(t, ports)
        t, k = t + 0.1, k + 1
        assert k < 50
    ports.busy_ = True  # a long replan: the new leg's first start check comes 4.5 s later
    for _ in range(45):
        dev.step(t, ports)
        t += 0.1
    ports.busy_ = False
    for _ in range(60):
        out = dev.step(t, ports)
        t += 0.1
        if not out.owned:
            break
    assert not out.owned and ("install", "p2") in ports.log and ("install", "p3") not in ports.log
    assert records[-1]["drops"] == 1  # the new leg waited its own 3 s, not dropped at once
    dev, records = _device()
    ports = Ports(candidates=[{"status": "tracking", "ok": False, "why": "start:unknown", "steer": 0.2}])
    _ask(dev)
    _run(dev, ports, n=2)
    ports.frozen = True
    steps = _run(dev, ports, t0=10.2, n=40)
    wait = records[-1]["permit_waits"][-1]
    assert steps[-1].end == "scene_replan_refused:cusp_pose:start:unknown"
    assert wait["until_s"] - wait["since_s"] == pytest.approx(3.0, abs=0.11)
