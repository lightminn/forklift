"""The scene replan device's escape presteer (docs/plans/2026-10-27-escape-presteer.md D2): when no escape qualifies at
the held steering, the steering is set to the side whose escapes qualify and the escape is evaluated again at the
measured steering before it is installed."""

import pytest

from forklift_core.mission.scene_replan import SceneReplan, SceneReplanConfig

from test_scene_replan import Ports, _ask, _run


class PresteerPorts(Ports):
    """Ports whose escape at the held steering fails, with a probe per curvature and a correction version."""

    def __init__(self, *, probes=None, escapes=None, version=0, **kw):
        kw.setdefault("plans", [("ok_fail", "invalid_start")])
        super().__init__(**kw)
        self.probes = dict(probes or {})  # kappa -> probe answer (None: nothing qualifies)
        self.escapes = list(escapes if escapes is not None else [None, "escape-path"])
        self.version = version

    def escape_plan(self, extra, clearance_m, presteered=False):
        self.log.append(("escape_plan", clearance_m, presteered))
        return self.escapes.pop(0) if self.escapes else None

    def escape_probe(self, extra, clearance_m, kappa):
        self.log.append(("escape_probe", kappa))
        return self.probes.get(kappa)

    def correction_version(self):
        return self.version


def _device(**cfg):
    records = []
    cfg.setdefault("escape_presteer", True)
    return SceneReplan(records, SceneReplanConfig(**cfg)), records


def _permit(dev, t=10.0):
    return _ask(dev, t=t, trigger="permit_stop")


def _probe(clearance):
    return {"ok": True, "end_clearance_m": clearance, "chosen": "-1x0.6"}


def _calls(ports, name):
    return [x for x in ports.log if isinstance(x, tuple) and x[0] == name]


def test_off_by_default_the_probe_is_never_asked():
    dev, records = SceneReplan([], SceneReplanConfig()), None
    ports = PresteerPorts(probes={0.05: _probe(0.3)})
    _permit(dev)
    steps = _run(dev, ports)
    assert steps[-1].owned is False and not _calls(ports, "escape_probe")
    assert [c[2] for c in _calls(ports, "escape_plan")] == [False]


def test_an_escape_at_the_held_steering_is_installed_without_a_probe():
    dev, records = _device()
    ports = PresteerPorts(escapes=["escape-path"], probes={0.05: _probe(0.3)})
    _permit(dev)
    _run(dev, ports, n=5)
    assert ("escape_install", "escape-path") in ports.log and not _calls(ports, "escape_probe")
    assert records[-1]["escape_presteer"] is None


@pytest.mark.parametrize("probes, kappa", [
    ({0.05: _probe(0.30), -0.05: _probe(0.40)}, -0.05),  # both: the larger end clearance
    ({0.05: _probe(0.30)}, 0.05),
    ({-0.05: _probe(0.30), 0.05: {"ok": False, "end_clearance_m": 0.5}}, -0.05),  # a permit refused does not count
])
def test_the_side_whose_escape_qualifies_is_steered_to_and_the_escape_evaluated_again_at_the_measured_steering(
        probes, kappa):
    dev, records = _device()
    ports = PresteerPorts(probes=probes, steer=[0.3, 0.1, 0.01])
    _permit(dev)
    steps = _run(dev, ports, n=20)
    presteers = [s for s in steps if s.output == "presteer"]
    assert presteers and all(s.steer == kappa and s.owned for s in presteers)
    assert [c[2] for c in _calls(ports, "escape_plan")] == [False, True]  # held, then measured after the presteer
    assert ("escape_install", "escape-path") in ports.log and dev.state == "escape"
    rec = records[-1]["escape_presteer"]
    assert rec["kappa"] == kappa and rec["presteer"] == "aligned" and rec["installed"] is True
    assert {p["kappa"] for p in rec["probes"]} == {0.05, -0.05}


def test_no_side_qualifies_ends_as_invalid_start_and_a_permit_stop_hands_back_standing():
    dev, records = _device()
    ports = PresteerPorts(probes={})
    _permit(dev)
    steps = _run(dev, ports)
    assert steps[-1].owned is False and steps[-1].end is None
    assert records[-1]["reason"] == "scene_replan_failed:permit_stop:invalid_start"
    assert records[-1]["mission_end"] is False and not _calls(ports, "escape_install")
    assert not any(s.output == "presteer" for s in steps)


def test_a_stop_lost_during_the_presteer_goes_back_to_wait_stop_without_steering_on_or_installing():
    dev, records = _device()
    ports = PresteerPorts(probes={0.05: _probe(0.3)}, steer=[0.3])
    _permit(dev)
    _run(dev, ports, n=4)
    assert dev.state == "escape_presteer"
    ports.stopped_ = False
    out = dev.step(20.0, ports)
    assert out.output == "hold" and dev.state == "wait_stop"
    assert dev.request_["escapes"] == 0 and dev.request_["changed"] is False
    assert not _calls(ports, "escape_install")


def test_a_correction_change_during_the_presteer_plans_again_instead_of_evaluating():
    dev, records = _device()
    ports = PresteerPorts(probes={0.05: _probe(0.3)}, steer=[0.3, 0.01],
                          plans=[("ok_fail", "invalid_start"), ("ok", "path-2")])
    _permit(dev)
    for k in range(4):
        dev.step(10.0 + 0.1 * k, ports)
        if dev.state == "escape_presteer":
            break
    assert dev.state == "escape_presteer"
    ports.version = 1  # a SLAM release re-projected the grid
    _run(dev, ports, t0=11.0, n=40)
    assert len(_calls(ports, "submit")) == 2  # planned again with the trigger's rectangles read again
    assert [c[2] for c in _calls(ports, "escape_plan")] == [False]  # never evaluated on the old rectangles
    assert records[-1]["escape_presteer"]["version_changed"] is True
    assert ("install", "path-2") in ports.log


def test_a_presteer_timeout_is_evaluated_at_the_measured_steering_and_a_failure_ends_as_invalid_start():
    dev, records = _device()
    ports = PresteerPorts(probes={0.05: _probe(0.3)}, steer=[0.3], escapes=[None, None])
    _permit(dev)
    steps = _run(dev, ports, n=40)
    assert [c[2] for c in _calls(ports, "escape_plan")] == [False, True]
    assert records[-1]["escape_presteer"]["presteer"] == "timeout"
    assert records[-1]["escape_presteer"]["installed"] is False
    assert records[-1]["reason"] == "scene_replan_failed:permit_stop:invalid_start"
    assert steps[-1].owned is False and records[-1]["mission_end"] is False


def test_a_pause_during_the_presteer_does_not_count_towards_its_time_limit():
    dev, records = _device()
    ports = PresteerPorts(probes={0.05: _probe(0.3)}, steer=[0.3])
    _permit(dev)
    t = 10.0
    while dev.state != "escape_presteer":
        dev.step(t, ports)
        t += 0.1
    dev.shift_time(30.0)  # a 30 s pause
    out = dev.step(t + 30.0, ports)
    assert out.output == "presteer" and dev.state == "escape_presteer"


def test_changed_stays_as_it_was_and_an_earlier_install_keeps_the_mission_end():
    # Codex plan round 3 P3: an earlier candidate installed in the same request already set changed
    dev, records = _device(drops_per_request=2)
    ports = PresteerPorts(probes={}, plans=[("ok", "path-1"), ("ok_fail", "invalid_start")],
                          previews=[{"status": "tracking", "ok": False, "why": "blocked:occupied", "steer": 0.2}],
                          steer=[0.0], escapes=[None])
    _permit(dev)
    steps = _run(dev, ports, n=150)
    assert ("install", "path-1") in ports.log
    assert steps[-1].end == "scene_replan_failed:permit_stop:invalid_start"
    assert records[-1]["mission_end"] is True


def test_a_stop_lost_by_the_probe_s_tick_gives_no_steering():
    # Codex impl round 5 P2-1: the first presteer output is behind the stop check as every later one
    dev, records = _device()
    ports = PresteerPorts(probes={0.05: _probe(0.3)}, steer=[0.3])
    _permit(dev)
    t = 10.0
    while dev.state != "wait_plan":
        dev.step(t, ports)
        t += 0.1
    ports.stopped_ = False
    out = dev.step(t, ports)
    assert out.output == "hold" and dev.state == "wait_stop"


def test_the_presteer_records_where_the_truck_stood_before_and_after():
    from types import SimpleNamespace

    dev, records = _device()
    ports = PresteerPorts(probes={0.05: _probe(0.3)}, steer=[0.3, 0.01])
    ports.box = SimpleNamespace(rear=(1.0, 2.0, 0.1))
    _permit(dev)
    t = 10.0
    while dev.state != "escape_presteer":
        dev.step(t, ports)
        t += 0.1
    ports.box.rear = (1.002, 2.0, 0.1)  # tyre scrub while steering
    _run(dev, ports, t0=t, n=5)
    rec = records[-1]["escape_presteer"]
    assert rec["rear_start"] == [1.0, 2.0, 0.1] and rec["rear_end"] == [1.002, 2.0, 0.1]


def test_after_a_correction_change_the_trigger_s_rectangles_are_read_only_once_the_grid_has_followed():
    # Codex impl round 6 P2: the runner re-projects the grid after the device's tick
    dev, records = _device()
    reads = []
    ports = PresteerPorts(probes={0.05: _probe(0.3)}, steer=[0.3, 0.01],
                          plans=[("ok_fail", "invalid_start"), ("ok", "path-2")])
    current = {"ok": True}
    ports.grid_current = lambda: current["ok"]
    _ask(dev, t=10.0, trigger="permit_stop", extra_obstacles_fn=lambda: reads.append("grid") or [])
    t = 10.0
    while dev.state != "escape_presteer":
        dev.step(t, ports)
        t += 0.1
    ports.version, current["ok"] = 1, False  # a correction applied, the grid not re-projected yet
    for _ in range(3):
        out = dev.step(t, ports)
        t += 0.1
        assert out.output == "hold" and dev.state == "escape_presteer" and len(reads) == 1
    current["ok"] = True
    _run(dev, ports, t0=t, n=40)
    assert len(reads) == 2 and ("install", "path-2") in ports.log
    assert records[-1]["escape_presteer"]["version_changed"] is True


def test_after_a_correction_change_the_plan_waits_for_the_grid_and_the_stop_both():
    # Codex impl round 7 P2
    dev, records = _device()
    reads = []
    ports = PresteerPorts(probes={0.05: _probe(0.3)}, steer=[0.3, 0.01],
                          plans=[("ok_fail", "invalid_start"), ("ok", "path-2")])
    current = {"ok": True}
    ports.grid_current = lambda: current["ok"]
    _ask(dev, t=10.0, trigger="permit_stop", extra_obstacles_fn=lambda: reads.append("grid") or [])
    t = 10.0
    while dev.state != "escape_presteer":
        dev.step(t, ports)
        t += 0.1
    ports.version, current["ok"] = 1, False
    dev.step(t, ports)
    ports.stopped_ = False  # the stop lost while it waits
    current["ok"] = True  # and the grid follows
    for _ in range(3):
        t += 0.1
        out = dev.step(t, ports)
        assert out.output == "hold" and len(reads) == 1 and not _calls(ports, "escape_install")
    ports.stopped_ = True
    _run(dev, ports, t0=t + 0.1, n=40)
    assert len(reads) == 2 and ("install", "path-2") in ports.log
