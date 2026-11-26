"""PR 6a-2b (docs/design/2026-11-20-real-hil-6a.md 10.1-10.3): the one run assembly -- the configuration through the
dry assembly's loaders, the refusals per reason, and S10-A through ``assemble_run`` itself on a fake clock (the rig's
fake MCU behind a transport that says it verified a HIL board; the plan worker run inline, the layer's worker
synchronous -- the only things replaced), plus the record and the scheduler."""

from __future__ import annotations

import dataclasses
import itertools
import pickle
import threading
from types import SimpleNamespace

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.live import real_run as rr
from forklift_core.live.real_assembly import AssemblyRefused
from forklift_core.live.real_channels import RealChannels
from forklift_core.live.real_layer import ManualWorker
from forklift_core.live.real_loop import real_host
from forklift_core.mcu_link.build_policy import Client
from forklift_core.mcu_link.messages import StateFlag
from forklift_core.safety.arbiter import REAL_LEASED

from real_loop_rig import MS, FakeTransport, FreshMcu

HIL_YAML = rr.REPO / "config/real/s10a_hil.yaml"
START = 10_000 * MS
HIL_FLAGS = StateFlag.DRIVERS_READY | StateFlag.HIL_PLANT | StateFlag.OUTPUTS_LOCKED


class InlineJobs:
    """The plan worker's jobs computed at submit (the worker module's own job function, its payload pickled)."""

    alive = True

    def __init__(self):
        self.done, self.n, self.closed = {}, 0, False

    def submit(self, name, payload):
        from forklift_core.planning.plan_worker import _run

        self.n += 1
        name_, payload_ = pickle.loads(pickle.dumps((name, payload)))
        try:
            self.done[self.n] = (True, _run(name_, payload_))
        except Exception as err:  # noqa: BLE001
            self.done[self.n] = (False, str(err))
        return self.n

    def poll(self, job):
        return self.done.pop(job, None)

    def discard(self, job):
        self.done.pop(job, None)

    def close(self):
        self.closed = True


class SyncWorker(ManualWorker):
    def submit(self, job, fn):
        self.done.append((job, fn(job)))


class HilFake(FakeTransport):
    """The rig's fake MCU link, presenting itself as a verified HIL board (the loop TTY transport's own check)."""

    fake_mcu = False

    def __init__(self, mcu):
        super().__init__(mcu)
        self.hil_plant, self.hil_boot, self.lost = True, mcu.boot, None


@pytest.fixture(scope="module")
def base_prep():
    return rr.prepare_run(rr.RunPaths(s10a=HIL_YAML), clock=lambda: START, jobs_factory=InlineJobs, epoch=9)


class Run:
    def __init__(self, base, **kw):
        self.now = START
        self.prep = dataclasses.replace(base, channels=RealChannels(), jobs=InlineJobs())
        self.host = real_host(rr.RUN_HOST_CFG, client=Client.HIL, initial_seq=0xFB80)
        self.mcu = FreshMcu(flags=HIL_FLAGS)
        self.transport = HilFake(self.mcu)
        self.events: list = []
        self.sources = rr.make_hil_sources(self.prep, self.host, self.transport)
        for k, v in kw.items():
            setattr(self.sources, k, v)
        self.run = rr.assemble_run(self.prep, self.sources, events=self.events.append, clock=lambda: self.now,
                                   layer_worker=SyncWorker())
        self.loop = self.run.loop
        self.arbiter = self.loop.link.a
        self.channels = self.prep.channels
        self.channels.connect("c1", "op", control=True, token=900_001)
        self.lease, self.out = None, []
        self._seq, self._rid = itertools.count(1), itertools.count(1)

    def tick(self, n=1):
        rec = None
        for _ in range(n):
            self.now += 20 * MS
            rec = self.loop.cycle(self.now)
            got = self.channels.take_out()
            self.out.extend(m for _, m in got if m.get("t") != "tick")
            ticks = [m for _, m in got if m.get("t") == "tick"]
            if ticks and self.lease is not None:
                last = ticks[-1]
                self.channels.message("c1", "op", {"t": "echo", "session_epoch": 9, "lease_id": self.lease,
                                                   "echo": {"state_seq": last["state_seq"], "token": last["token"]}})
        return rec

    def say(self, kind_, /, **fields):
        msg = {"t": kind_, "request_id": f"q{next(self._rid)}", "session_epoch": 9,
               "control_gen": self.arbiter.control_gen, **fields}
        if self.lease is not None:
            msg.setdefault("lease_id", self.lease)
        if kind_ in REAL_LEASED or kind_ == "recovery":
            msg.setdefault("request_token", 900_001)
            msg.setdefault("request_seq", next(self._seq))
        self.channels.message("c1", "op", msg)
        return msg["request_id"]

    def ask(self, kind_, /, **fields):
        rid = self.say(kind_, **fields)
        self.tick()
        mine = [m for m in self.out if m.get("request_id") == rid]
        assert mine, (kind_, self.out[-5:])
        return mine[0]

    def until(self, cond, limit):
        for _ in range(limit):
            rec = self.tick()
            if cond(rec):
                return rec
        raise AssertionError(f"not reached: {self.loop.records[-1]}")

    def start(self):
        self.tick(2)
        self.run.start_session(self.now)
        self.tick(6)
        assert self.loop.permit.session_started
        self.tick(3)
        self.say("lease_request")
        self.tick()
        self.lease = self.arbiter.lease.lease_id


def test_the_hil_configuration_prepares_with_the_dry_loaders(base_prep):
    p = base_prep
    assert p.check_plan == "success" and p.map_id == "hil-s10a-1" and p.cfg.start_rear == (0.0, 0.0, 0.0)
    assert p.budget_s == 0.10 and p.overdue_end_s == 1.0  # config/real/obstacle_layer.yaml (10.9)
    assert p.layer_cfg["bridge"]["scan_period_s"] == 0.1 and p.layer_cfg["bridge"]["state_delay_s"] == 0.03
    assert p.rear_offset_m > 0 and p.body_front_m > 0 and set(p.hashes) >= {"s10a", "layer", "budget"}
    # the pallet is no obstacle in the planning occupancy; the separate obstacle beside it is (10.6)
    occ = p.occupancy
    assert not occ.occupied[occ.cell_of(2.6, 0.0)] and occ.occupied[occ.cell_of(2.6, 1.4)]


def test_s10_a_runs_through_the_production_assembly(base_prep):
    r = Run(base_prep)
    r.start()
    reply = r.ask("check_bypass", check="d5_pocket", on=True)
    assert reply["t"] == "ack", reply
    r.tick()
    reply = r.ask("pallet_register", pallet_id="P-S10", pose=[2.6, 0.0, 0.0])
    assert reply["t"] == "ack", reply
    rev = reply["targets_revision"]
    r.tick(30)
    assert r.ask("operator_start")["t"] == "ack"
    r.say("mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=rev)
    r.until(lambda rec: r.loop.core.operator_open() is not None or r.loop.permit.state == ra.LATCHED, 12000)
    op = r.loop.core.operator_open()
    assert op is not None, (r.loop.core.commands.end_reason, r.loop.permit.reason)
    assert r.ask("operator_answer", kind=op["kind"], serial=op["serial"], verdict="confirm")["t"] == "ack"
    r.until(lambda rec: r.loop.permit.state == ra.LATCHED and r.loop.session.results, 4000)
    assert [e["reason"] for e in r.loop.session.results] == ["s10_lifted"]
    assert r.mcu.lift > 0.15  # the model's lift followed the DRIVEs
    kinds = {e["t"] for e in r.events}
    assert {"frame_queued", "frame_written"} <= kinds


def test_closing_confirms_a_disarm_within_one_second(base_prep):
    r = Run(base_prep)
    r.start()

    def sleep(s):
        r.now += int(s * 1e9)

    r.run.clock = lambda: r.now
    assert r.run.close(sleep=sleep) == "confirmed"
    assert r.loop.closing and r.loop.closing_disarms and r.loop.closing_confirmed_ns is not None


def test_closing_without_the_disarm_reaching_the_model_is_unconfirmed(base_prep):
    r = Run(base_prep)
    r.start()
    r.transport.mcu.receive = lambda msg, seq: None  # nothing reaches the model; its DISARMED STATEs go on

    def sleep(s):
        r.now += int(s * 1e9)

    assert r.run.close(sleep=sleep) == "disarm_unconfirmed"


@pytest.mark.parametrize("change,reason", [
    (lambda s: setattr(s, "client", Client.REAL), "precondition:unlock_incomplete"),
    (lambda s: setattr(s.transport, "hil_plant", False), "hil_check_not_held"),
    (lambda s: setattr(s.transport, "fake_mcu", True), "transport_fake_mcu"),
    (lambda s: setattr(s.slam, "map_id", "other"), "slam_map_mismatch:other"),
    (lambda s: setattr(s.scans, "sensor", "lidar2"), "scan_sensor_unknown:lidar2"),
    (lambda s: setattr(s, "imu", None), "imu_required"),
    (lambda s: setattr(s, "host", SimpleNamespace(client=Client.REAL)), "host_client_mismatch:Client.REAL"),
])
def test_assembly_refusals_by_reason(base_prep, change, reason):
    prep = dataclasses.replace(base_prep, channels=RealChannels(), jobs=InlineJobs())
    host = real_host(rr.RUN_HOST_CFG, client=Client.HIL)
    tr = HilFake(FreshMcu(flags=HIL_FLAGS))
    sources = rr.make_hil_sources(prep, host, tr)
    change(sources)
    with pytest.raises(AssemblyRefused) as err:
        rr.assemble_run(prep, sources, layer_worker=SyncWorker())
    assert reason in err.value.reasons


def test_a_hil_scene_not_starting_at_the_origin_is_refused(tmp_path):
    import yaml

    d = yaml.safe_load(HIL_YAML.read_text())
    d["start_rear"] = [0.2, 0.0, 0.0]
    p = tmp_path / "s.yaml"
    p.write_text(yaml.safe_dump(d))
    with pytest.raises(AssemblyRefused) as err:
        rr.prepare_run(rr.RunPaths(s10a=p), jobs_factory=InlineJobs)
    assert "hil_start_not_zero" in err.value.reasons


def test_a_file_without_a_hil_scene_is_refused_for_a_hil_run(tmp_path):
    import yaml

    d = yaml.safe_load(HIL_YAML.read_text())
    d.pop("hil")
    p = tmp_path / "s.yaml"
    p.write_text(yaml.safe_dump(d))
    with pytest.raises(AssemblyRefused) as err:
        rr.prepare_run(rr.RunPaths(s10a=p), jobs_factory=InlineJobs)
    assert "hil_scene_missing" in err.value.reasons


def test_the_plan_worker_start_is_cancelled_and_closed_by_the_side_that_comes_second(base_prep):
    gate, made = threading.Event(), []

    class Slow(InlineJobs):
        def __init__(self):
            super().__init__()
            gate.wait(5)
            made.append(self)

    stop = threading.Event()
    th = threading.Thread(target=lambda: (stop.wait(0.2), stop.set()))
    th.start()
    with pytest.raises(rr.RunCancelled):
        rr.prepare_run(rr.RunPaths(s10a=HIL_YAML), jobs_factory=Slow, stop=stop)
    gate.set()
    th.join()
    for _ in range(100):
        if made:
            break
        threading.Event().wait(0.02)
    threading.Event().wait(0.05)
    assert made and made[0].closed  # finished after the cancel: the starting thread closed it


def test_the_record_never_waits_and_says_when_it_lost_rows(tmp_path):
    """A writer held back while the queue (2 rows) overflows: put() returns at once, the loss is ``queue_full`` and the
    record is not complete; the end row still reaches the file."""
    gate = threading.Event()

    class Slow:
        def __init__(self):
            self.f = open(tmp_path / "r.jsonl", "w")

        def write(self, s):
            gate.wait(5)
            self.f.write(s)

        def flush(self):
            self.f.flush()

        def close(self):
            self.f.close()

    rec = rr.RunRecord(tmp_path / "r.jsonl", maxsize=2, opener=lambda p, m: Slow())
    for i in range(50):
        rec.put({"t": "x", "i": i})
    assert rec.incomplete == "queue_full"
    gate.set()
    assert rec.close({"t": "end"}) is False
    lines = (tmp_path / "r.jsonl").read_text().splitlines()
    assert lines and '"t": "end"' in lines[-1] and '"record_incomplete": "queue_full"' in lines[-1]


def test_a_failing_writer_is_told_and_fails_the_record(tmp_path):
    class Bad:
        def write(self, s):
            raise OSError("disk")

        def flush(self):
            pass

        def close(self):
            pass

    rec = rr.RunRecord(tmp_path / "r.jsonl", opener=lambda p, m: Bad())
    rec.put({"t": "x"})
    ok = rec.close({"t": "end"})
    assert not ok and rec.failed and "disk" in rec.failed


class FakeClockLoop:
    def __init__(self, costs_ms):
        self.now, self.starts, self.costs = 0, [], list(costs_ms)
        self.fatal_source, self.records = None, []

    def cycle(self, now):
        self.starts.append(now)
        self.now += int((self.costs.pop(0) if self.costs else 1) * MS)
        return SimpleNamespace(t=0, start_interval_s=None, branch="", reason=None, seqs=[], phase=None, permit=None,
                               elapsed_s=None, notes=[])


def _sched(costs, duration_s=0.2, stop_after=None):
    loop = FakeClockLoop(costs)
    run = rr.RealRun.__new__(rr.RealRun)
    run.loop, run.sources, run.end_reason, run.ticks = loop, SimpleNamespace(transport=SimpleNamespace(lost=None)), \
        None, 0
    run.clock = lambda: loop.now
    stop = threading.Event()

    def sleep(s):
        loop.now += int(round(s * 1e9))
        if stop_after is not None and len(loop.starts) >= stop_after:
            stop.set()

    why = run.run(duration_s, stop=stop, sleep=sleep)
    return loop.starts, why


def test_the_schedule_keeps_the_20_ms_boundaries():
    starts, why = _sched([1] * 20, 0.1)
    assert why == "deadline" and starts[:5] == [0, 20 * MS, 40 * MS, 60 * MS, 80 * MS]


def test_a_tick_ending_exactly_on_a_boundary_skips_it():
    starts, _ = _sched([20, 1, 1], 0.1)
    assert starts[1] == 40 * MS  # the boundary at 20 ms was reached by the tick itself: skipped, not run late


def test_slow_ticks_skip_one_boundary_each():
    starts, _ = _sched([22, 23, 25, 1], 0.2)
    assert [b - a for a, b in zip(starts, starts[1:])][:3] == [40 * MS, 40 * MS, 40 * MS]


def test_a_stop_during_the_sleep_ends_at_the_next_wake():
    starts, why = _sched([1] * 50, 1.0, stop_after=3)
    assert why == "signal" and len(starts) == 3


# --- Codex impl round 1 ---------------------------------------------------------------------------------------------
def test_the_safety_speed_takes_the_newest_states_delay_bound(base_prep):
    """P1-1: the encoder speed's age adds the newest STATE's own round-trip bound; unknown -> no safety speed."""
    from forklift_core.live.real_teleop import encoder_speed

    r = Run(base_prep)
    r.start()
    st = r.loop.samples[-1].state
    bound = r.loop._sample_delay[(int(st.boot_id), int(st.mcu_time_us))][1]  # noqa: SLF001
    assert bound is not None and r.loop._speed_delay_s() == bound  # noqa: SLF001
    r.loop._sample_delay.clear()  # noqa: SLF001
    assert r.loop._speed_delay_s() is None  # noqa: SLF001
    reading = encoder_speed(list(r.loop.samples), r.now, r.loop.params, r.loop.steering_ok, None)
    assert not reading.speed.valid


def test_a_dead_plan_worker_is_worker_dead_and_one_without_liveness_is_refused(base_prep):
    """P1-2: the readiness reads the plan worker's own liveness; a jobs object without it is not assembled."""
    r = Run(base_prep)
    r.start()
    assert r.loop.evidence.worker_ok(r.now)
    r.prep.jobs.alive = False
    assert not r.loop.evidence.worker_ok(r.now)
    prep = dataclasses.replace(base_prep, channels=RealChannels(), jobs=SimpleNamespace(close=lambda: None))
    host = real_host(rr.RUN_HOST_CFG, client=Client.HIL)
    sources = rr.make_hil_sources(prep, host, HilFake(FreshMcu(flags=HIL_FLAGS)))
    with pytest.raises(AssemblyRefused) as err:
        rr.assemble_run(prep, sources, layer_worker=SyncWorker())
    assert "plan_worker_no_liveness" in err.value.reasons


def test_the_plan_workers_liveness_follows_its_permanent_failure():
    from forklift_core.planning.plan_worker import PlanWorker

    w = PlanWorker.__new__(PlanWorker)
    w._lock, w._dead, w._replacing = threading.Lock(), None, False  # noqa: SLF001
    w._thread = SimpleNamespace(is_alive=lambda: True)  # noqa: SLF001
    w._stopping = threading.Event()  # noqa: SLF001
    assert w.alive
    w._dead = "plan worker restart failed"  # noqa: SLF001
    assert not w.alive


def test_the_snapshot_view_compares_the_installed_results_own_correction(base_prep):
    """P2: the installed result's correction, not the applied one, is the view's installed correction."""
    r = Run(base_prep)
    r.start()
    r.tick(10)
    layer = r.run.layer
    assert layer.installed_correction() is not None
    layer._installed["correction"] = (0.5, 0.0, 0.0)  # noqa: SLF001 -- made with another correction
    view = r.loop.evidence.snapshot(r.now)
    assert view.installed_correction == (0.5, 0.0, 0.0) and view.applied_correction == (0.0, 0.0, 0.0)


def test_a_hold_is_owned_by_the_generation_it_began_in(base_prep):
    r = Run(base_prep)
    r.start()
    core = r.loop.core
    core.hold_mission_id = "m1"
    tracker = core.slam["tracker"]
    tracker.mode = "holding"
    core.commands.mission_gen += 3  # a pause within the mission: still its hold
    core.commands.mission_id = "m1"
    ev = r.loop.evidence
    assert ev.slam(r.now).hold_mission_gen == "m1" == ev.mission_gen(r.now)
    core.commands.mission_id = "m2"  # another mission: not its hold
    assert ev.slam(r.now).hold_mission_gen != ev.mission_gen(r.now)
    tracker.mode = "tracking"
    assert ev.slam(r.now).hold_mission_gen is None


def test_a_confirmation_after_the_closing_second_is_unconfirmed(base_prep):
    """P2: a closing tick begun before the end whose confirmation lands after it is late."""
    r = Run(base_prep)
    r.start()

    def sleep(s):
        r.now += int(s * 1e9)

    r.run.clock = lambda: r.now
    real_cycle = r.loop.cycle

    def late_cycle(now):
        rec = real_cycle(now)
        if r.loop.closing_confirmed_ns is not None:
            r.loop.closing_confirmed_ns = r.loop.closing_started_ns + int(1e9) + 1
        return rec

    r.loop.cycle = late_cycle
    assert r.run.close(sleep=sleep) == "disarm_unconfirmed"


def test_a_record_without_a_file_or_without_its_end_row_is_not_complete(tmp_path):
    rec = rr.RunRecord(None)
    rec.put({"t": "x"})
    assert rec.close({"t": "end"}) is False
    rec = rr.RunRecord(tmp_path / "r.jsonl")
    rec._finish.set()  # noqa: SLF001 -- the writer stops on an empty queue before the end row comes
    rec._thread.join(5)  # noqa: SLF001
    assert rec.close({"t": "end"}, timeout_s=0.5) is False and not rec.end_written


def test_a_full_queue_at_the_end_still_lets_the_writer_finish_and_close(tmp_path):
    """Round 3 P3: the end row is the last item; with no slot for it the close fails in time and the writer, once
    released, ends on the finish flag and closes its file."""
    gate, closed = threading.Event(), []

    class Slow:
        def write(self, s):
            gate.wait(5)

        def flush(self):
            pass

        def close(self):
            closed.append(True)

    rec = rr.RunRecord(tmp_path / "r.jsonl", maxsize=1, opener=lambda p, m: Slow())
    rec.put({"t": "a"})
    rec.put({"t": "b"})  # the writer holds a, b fills the one slot
    assert rec.close({"t": "end"}, timeout_s=0.2) is False
    gate.set()
    rec._thread.join(5)  # noqa: SLF001
    assert not rec._thread.is_alive() and closed  # noqa: SLF001


def test_no_imu_source_under_the_imu_none_policy(base_prep):
    from forklift_core.live.real_assembly import Watches

    prep = dataclasses.replace(base_prep, channels=RealChannels(), jobs=InlineJobs(),
                               watches=Watches(base_prep.watches.safety, None, "none"))
    host = real_host(rr.RUN_HOST_CFG, client=Client.HIL)
    sources = rr.make_hil_sources(prep, host, HilFake(FreshMcu(flags=HIL_FLAGS)))
    assert sources.imu is None
    run = rr.assemble_run(prep, sources, layer_worker=SyncWorker())
    assert run.loop.imu_source is None


def test_a_slam_reply_iteration_failure_is_the_runs_fatal_source(base_prep):
    """P3: the guard covers the source's iteration too, not only its call."""
    r = Run(base_prep)
    r.start()

    def broken():
        yield from ()
        raise OSError("socket")

    r.sources.slam.replies = lambda now: broken()
    rec = r.tick()
    assert r.loop.fatal_source is not None and "slam:replies" in r.loop.fatal_source and rec.branch == "emergency"


def test_a_hil_run_casts_the_scene_obstacles_with_a_bundle_too(base_prep, tmp_path, monkeypatch):
    """P2: the HIL scene's fixed obstacles are read whatever map the run plans on."""
    monkeypatch.setattr(rr, "read_map", lambda bundle, cfg: (cfg.map_id, base_prep.occupancy))
    prep = rr.prepare_run(rr.RunPaths(s10a=HIL_YAML, map_bundle=tmp_path), clock=lambda: START,
                          jobs_factory=InlineJobs)
    assert prep.obstacles == [(2.6, 1.4, 0.3, 0.2, 0.0)]


# --- Codex impl round 2 ---------------------------------------------------------------------------------------------
def test_a_plan_worker_replacing_a_dead_process_is_not_alive():
    from forklift_core.planning.plan_worker import PlanWorker

    w = PlanWorker.__new__(PlanWorker)
    w._lock, w._dead, w._replacing = threading.Lock(), None, True  # noqa: SLF001
    w._thread = SimpleNamespace(is_alive=lambda: True)  # noqa: SLF001
    w._stopping = threading.Event()  # noqa: SLF001
    assert not w.alive
    w._replacing = False  # noqa: SLF001
    assert w.alive


def test_an_imu_batch_failing_while_iterated_with_source_fatal_ends_the_run(base_prep):
    r = Run(base_prep)
    r.start()

    def gen(upto, limit):
        yield (int(upto) - 1_000_000, (0.0, 0.0, 9.81))
        raise rr.SourceFatal("imu link gone")

    r.sources.imu.samples = gen
    rec = r.tick()
    assert r.loop.fatal_source is not None and "imu" in r.loop.fatal_source and rec.branch == "emergency"


def test_a_depth_check_is_registered_with_its_region_only_for_a_frame_of_this_moment(base_prep):
    """Round 2 P1-1, round 3 P1 (design 10.5 P1-4): the check is built from the very frame the core judged (handed to
    the builder), registered before the hook sets its region as the layer's exemption; a frame of another mission
    generation, or a build for another frame than the judged one, registers nothing."""
    from forklift_core.live.real_assembly import RealRequireFailed

    class Check:
        valid, invalid_reason = True, None

        def __init__(self, frame):
            self.frame = frame

        def region_control(self):
            return (2.6, 0.0, 1.8, 0.8, 0.0)

    frame = {"f": None}
    depth = SimpleNamespace(frames=lambda now: [], pocket_frame=lambda: frame["f"],
                            pocket_check=lambda *a, frame: Check(frame))
    r = Run(base_prep, depth=depth)
    r.start()
    core = r.loop.core
    hooks = core.hooks

    def make(gen, stamp):
        return SimpleNamespace(mission_gen=gen, phase=core.ms.phase, correction=(0.0, 0.0, 0.0), stamp_s=stamp)

    frame["f"] = make(core.commands.mission_gen + 1, 9.95)
    hooks.known_pocket_frame()
    with pytest.raises(RealRequireFailed):
        hooks.build_pocket_check(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 9.95)
    assert core.pocket["check"] is None and r.run.layer.exempt is None
    judged = make(core.commands.mission_gen, 9.95)
    frame["f"] = judged
    hooks.known_pocket_frame()  # the core judges this frame
    frame["f"] = make(core.commands.mission_gen, 8.00)  # the source now holds another one
    with pytest.raises(RealRequireFailed):
        hooks.build_pocket_check(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 8.00)  # not the judged frame
    assert core.pocket["check"] is None
    hooks.build_pocket_check(0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), 9.95)
    assert core.pocket["check"].frame is judged and r.run.layer.exempt == (2.6, 0.0, 1.8, 0.8, 0.0)


def test_a_writer_stuck_in_a_write_does_not_hold_the_close_past_its_deadline(tmp_path):
    gate = threading.Event()

    class Stuck:
        def write(self, s):
            gate.wait(10)

        def flush(self):
            pass

        def close(self):
            pass

    rec = rr.RunRecord(tmp_path / "r.jsonl", opener=lambda p, m: Stuck())
    rec.put({"t": "x"})
    import time as _t

    t0 = _t.monotonic()
    assert rec.close({"t": "end"}, timeout_s=0.2) is False
    assert _t.monotonic() - t0 < 1.0
    gate.set()


def test_the_hold_records_the_mission_it_began_in(base_prep):
    r = Run(base_prep)
    r.start()
    core = r.loop.core
    core.commands.mission_id = "m7"
    core.hold_slam(r.loop._t(r.now), held_frame_pallet=False)  # noqa: SLF001
    assert core.hold_mission_id == "m7" and core.slam["tracker"].mode == "holding"
    assert r.loop.evidence.slam(r.now).hold_mission_gen == "m7"


def test_a_file_changed_while_preparing_is_refused(monkeypatch):
    """Codex unlock U1 review P1-2: the run and the operating preconditions judge the same bytes -- a settings file that
    changes between the start and the end of the preparation refuses it."""
    calls = {"n": 0}
    real_sha = rr._sha

    def sha(path):
        calls["n"] += 1
        h = real_sha(path)
        return h if calls["n"] <= len(rr._hashed_files(rr.RunPaths(s10a=HIL_YAML))) else "changed" + h[7:]

    monkeypatch.setattr(rr, "_sha", sha)
    with pytest.raises(AssemblyRefused) as err:
        rr.prepare_run(rr.RunPaths(s10a=HIL_YAML), jobs_factory=InlineJobs)
    assert any(r.startswith("files_changed_while_preparing:") for r in err.value.reasons)


def test_an_unparsable_vehicle_file_is_still_a_structured_refusal(tmp_path):
    """Codex unlock U1 review round 4 P2: the chassis hash read at the start never escapes a YAML error."""
    bad = tmp_path / "vehicle.yaml"
    bad.write_text("chassis_model: [unclosed\n")
    with pytest.raises(AssemblyRefused):
        rr.prepare_run(rr.RunPaths(s10a=HIL_YAML, vehicle=bad), jobs_factory=InlineJobs)
