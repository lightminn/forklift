"""A fake truck for the real loop's tests (docs/design/2026-11-19-real-loop.md 6): the real McuLinkHost and port, a
fake MCU integrating its wheel counts, RealOdometry, a SLAM stand-in whose correction is the identity, straight-line
plans, a permissive layer, and the evidence read from the loop's own values. Not a test module."""

from __future__ import annotations

import itertools
import math
import sys
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from forklift_core.live import real_assembly as asm
from forklift_core.live import real_autonomy as ra
from forklift_core.live.real_channels import RealChannels, RealLink
from forklift_core.live.real_loop import RealLoop, RealSession, real_host
from forklift_core.live.real_teleop import EncoderSpeedParams, real_auto_refusal
from forklift_core.mcu_link.messages import LinkDecoder, encode
from forklift_core.mission.commands import Submit
from forklift_core.plant.kinematics import RealKinematics
from forklift_core.plant.mcu_host import HostConfig
from forklift_core.plant.port import RealPlantPort
from forklift_core.plant.real_vehicle import RealOdometry
from forklift_core.planning.pallet_mission import SyntheticMissionGeometry
from forklift_core.safety.arbiter import REAL_LEASED, Arbiter, ArbiterConfig

from test_teleop_bridge_cycle import MS, VEH, FakeMcu

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tests/unit/mission"))
from test_core_async_plan import FakeJobs  # noqa: E402

EPOCH = 9
KIN = RealKinematics(VEH)
CAP = min(-KIN.cfg.steering.curvature_range[0], KIN.cfg.steering.curvature_range[1])
STAGES = ra.S10A_STAGES
PALLET = (3.0, 0.0, 0.0)


class Path_:
    def __init__(self, x0, x1, direction=1, status="success"):
        n = max(int(abs(x1 - x0) / 0.02) + 1, 2)
        self.poses = np.column_stack((np.linspace(x0, x1, n), np.zeros(n), np.zeros(n)))
        self.directions = np.full(n, direction)
        self.curvatures_inv_m = np.zeros(n)
        self.length_m, self.expanded_nodes, self.status, self.turnaround = abs(x1 - x0), 1, status, None

    @property
    def success(self):
        return self.status == "success"


class Plan:
    def __init__(self, approach_to=2.0, insert_to=2.3):
        self.status = "success"
        self.approach = Path_(0.0, approach_to)
        self.insert = Path_(approach_to, insert_to)
        self.extract = Path_(insert_to, approach_to, -1)
        self.transport = Path_(approach_to, approach_to - 0.5, -1)
        self.withdraw = Path_(approach_to - 0.5, approach_to - 0.8, -1)
        self.return_home = Path_(approach_to - 0.8, 0.0, -1)

    @property
    def success(self):
        return True


class Tracker:
    """The SLAM tracker stand-in: identity correction, hold and release by mode only."""

    def __init__(self):
        self.mode, self.applied = "tracking", ((0.0, 0.0, 0.0), 0)

    def hold(self, odom_from_base=None):
        self.mode = "holding"

    def release(self, odom_from_base=None):
        self.mode = "tracking"
        return 0.0, 0.0

    def may_drive(self):
        return True


REAR_OFFSET_M = 0.2  # 5c-2 rigs: a non-zero rear-axle offset (base_link ahead of the rear axle)


class FakeSlam:
    """5c-2 (design 6): the SLAM source's epoch contract. Every ``period_ns`` a processed reply of the current epoch
    whose first beam is ``lag_ns`` old, with ``correction`` (map <- odom); ``request_processed`` answers the first
    scan at or after its time; ``relocalize`` keeps the estimate, ``seed`` restarts it at the given base pose."""

    def __init__(self, rig):
        self.rig, self.epoch, self.correction = rig, 0, (0.0, 0.0, 0.0)
        self.auto, self.period_ns, self.lag_ns = True, 100 * MS, 0
        self.status = "processed"  # the auto reply's status ("skipped": a stream without new poses)
        self.requests, self.calls, self.extra = [], [], []
        self._next, self._scan = None, 0

    def relocalize(self, epoch):
        self.calls.append(("relocalize", epoch))
        self.epoch = epoch

    def seed(self, epoch, base_pose, stamp_s):
        self.calls.append(("seed", epoch, tuple(base_pose), stamp_s))
        self.epoch = epoch
        from forklift_core.localization.slam_pose import compose, invert

        self.correction = tuple(compose(tuple(base_pose), invert(tuple(self.rig.odom_base()))))

    def request_processed(self, epoch, attempt, not_before):
        self.calls.append(("request_processed", epoch, attempt, not_before))
        self.requests.append((epoch, attempt, not_before))

    def reply(self, stamp_s, *, status="processed", epoch=None, attempt=None, correction=None, scan_id=None):
        if scan_id is None:
            self._scan += 1
            scan_id = self._scan
        r = SimpleNamespace(scan_id=scan_id, stamp_s=float(stamp_s), status=status, slam_wall_s=None,
                            map_from_odom=tuple(self.correction if correction is None else correction),
                            epoch=self.epoch if epoch is None else epoch, attempt=attempt)
        return (r, {}, None, None)

    def replies(self, now_ns):
        t = self.rig.loop._t(now_ns)
        out, self.extra = list(self.extra), []
        if self.auto and (self._next is None or now_ns >= self._next):
            self._next = now_ns + self.period_ns
            out.append(self.reply(t - self.lag_ns / 1e9, status=self.status))
        for req in list(self.requests):
            epoch, attempt, not_before = req
            if epoch == self.epoch and t >= not_before:
                self.requests.remove(req)
                out.append(self.reply(t, attempt=attempt))
        return sorted(out, key=lambda it: it[0].stamp_s)


class Layer:
    known_enabled = False

    def __init__(self):
        self.exempt = self.depth_support = None
        self.permission = SimpleNamespace(path_check=None, last_estop=None,
                                          config=SimpleNamespace(stopping=SimpleNamespace(decel_mps2=1.0)))
        self.noise_cut_m, self.memory = 0.06, None

    def refresh(self, *a, **k):
        pass

    def limit(self, *a, **k):
        return math.inf, "ok"

    def watchdog(self, now):
        return None


class FreshMcu(FakeMcu):
    """The shared fake MCU, with the firmware's COMMAND_FRESH (a DRIVE's validity not yet run out while armed) and a
    lift that follows a DRIVE's lift target at 0.05 m/s while armed (it holds where it is otherwise)."""

    lift, lift_target, LIFT_RATE = 0.0, 0.0, 0.05
    reject = None  # a RejectReason every STATE carries (injection)
    fresh_off = False  # COMMAND_FRESH never set (injection)

    def receive(self, msg, seq):
        from forklift_core.mcu_link.messages import Drive, Mode

        super().receive(msg, seq)
        if isinstance(msg, Drive) and self.mode == Mode.ARMED and msg.arm_gen == self.gen and self.zero_seen:
            self.lift_target = float(msg.lift_m)

    def advance(self, dt_us):
        from forklift_core.mcu_link.messages import Mode

        super().advance(dt_us)
        if self.mode == Mode.ARMED:
            step = self.LIFT_RATE * dt_us / 1e6
            self.lift += max(-step, min(step, self.lift_target - self.lift))

    def state(self):
        from dataclasses import replace as _r

        from forklift_core.mcu_link.messages import Mode, StateFlag

        st = _r(super().state(), lift_m=float(self.lift))
        if self.mode == Mode.ARMED and self.deadline_us is not None and self.t_us <= self.deadline_us \
                and self.zero_seen and not self.fresh_off:
            st = _r(st, flags=st.flags | StateFlag.COMMAND_FRESH)
        if self.reject is not None:
            st = _r(st, reject_reason=self.reject)
        return st


class _FixedSkip:
    @staticmethod
    def randint(lo, hi):
        return lo


class FakeTransport:
    fake_mcu = True  # the internal fake MCU: the loop's device block lets ARM and DRIVE through (design 5)

    def __init__(self, mcu):
        self.mcu, self.dec, self.last_ns, self.sent = mcu, LinkDecoder(), None, []
        self.refuse_next = 0
        self.silent = False  # no STATE reaches the host (an RX cut; the MCU keeps running)
        self.judged: dict = {}  # seq -> the judged clock value of its last successful write (6a design 6 ②)
        self.eagain: set = set()  # frame kinds whose first byte meets EAGAIN after the gate passed them

    def receive(self, host, now_ns, limit=64):
        if self.silent:
            steps = 0 if self.last_ns is None else max(0, int((now_ns - self.last_ns) // (10 * MS)))
            for _ in range(steps):
                self.mcu.advance(10_000)
            self.last_ns = now_ns if self.last_ns is None else self.last_ns + steps * 10 * MS
            return
        steps = 0 if self.last_ns is None else max(0, int((now_ns - self.last_ns) // (10 * MS)))
        self.last_ns = now_ns if self.last_ns is None else self.last_ns + steps * 10 * MS
        for _ in range(min(steps, limit)):
            self.mcu.advance(10_000)
            host.on_bytes(encode(self.mcu.state(), 1), now_ns)
        if steps == 0:
            host.on_bytes(encode(self.mcu.state(), 1), now_ns)

    stall = False  # the device takes a frame's first bytes and then nothing (EAGAIN until it is reopened)
    _cur = None  # the frame being written
    partial_since_ns = None  # when the frame being written started (the loop's stuck-write evidence)
    before_continue = None  # a hook run just before a started frame is continued (timing injection)
    resyncs = 0

    def transmit(self, host, now_ns, gate):
        out = []
        if self._cur is not None:
            if self.stall:
                return out  # nothing more goes out, a DISARM included
            f = self._cur

            def finish(f=f, *, judged_ns=None):
                self.judged[int(f.seq) & 0xFFFF] = judged_ns
                self._cur, self.partial_since_ns = None, None
                self._deliver(host, f, now_ns)

            if self.before_continue is not None:
                self.before_continue()
            if gate(f, finish):  # a started frame passes the gate again before it is finished (the loop's contract)
                out.append((f.kind, f.seq, now_ns))
            else:
                self.resync(host, now_ns)  # refused half frame: abandoned, the link reopened
        while (f := host.next_frame(now_ns)) is not None:
            if f.kind in self.eagain:
                # 6a impl rounds 1-2: EAGAIN only after the gate passed it (a refusal keeps the refusal's own path)
                passed = gate(f, lambda *, judged_ns=None: None)
                host.unsent(f, now_ns)
                if not passed and not getattr(gate, "keep", False):
                    host.drop_drives("send_gate")
                break

            def write(f=f, *, judged_ns=None):
                self.judged[int(f.seq) & 0xFFFF] = judged_ns
                if self.stall:
                    self._cur, self.partial_since_ns = f, now_ns  # its first bytes only
                    return
                self._deliver(host, f, now_ns)

            if gate(f, write):
                if self._cur is not None:
                    break
                out.append((f.kind, f.seq, now_ns))
            else:
                host.unsent(f, now_ns)
                if not getattr(gate, "keep", False):  # 6a: a kept ARM / arming zero leaves the queue as it is
                    host.drop_drives("send_gate")
                break
        return out

    @property
    def partial_frame(self):
        return self._cur  # the frame being written (its own expiry bounds the stall)

    def _deliver(self, host, f, now_ns):
        host.frame_written(f, now_ns)
        for r in self.dec.feed(f.data, now_ns):
            self.sent.append((now_ns, f.kind, f.seq, r.message))
            self.mcu.receive(r.message, f.seq)

    def resync(self, host, now_ns):
        """The link closed and reopened: the half-written frame abandoned (counted as possibly delivered), the MCU's
        half frame closed by the opening delimiter (never applied)."""
        if self._cur is not None:
            host.frame_abandoned(self._cur, 1, now_ns)
        self._cur, self.partial_since_ns, self.stall = None, None, False
        self.resyncs += 1


class FakeImu:
    """A 200 Hz accelerometer on the rig's clock (the source contract of safety wiring design 4: the samples taken
    so far, non-blocking, ascending, at most ``limit``). ``accel`` is what it reads now; ``paused`` stops it."""

    def __init__(self, clock, period_ns=5 * MS):
        self.clock, self.period_ns = clock, int(period_ns)
        self.accel = (0.0, 0.0, 9.81)
        self.paused = False
        self.next_ns = None
        self.fail = False

    def samples(self, upto_ns, limit):
        if self.fail:
            raise OSError("imu read")
        if self.next_ns is None:
            self.next_ns = int(upto_ns) - 20 * self.period_ns
        out = []
        while self.next_ns <= int(upto_ns) and len(out) < limit:
            if not self.paused:
                out.append((self.next_ns, self.accel))
            self.next_ns += self.period_ns
        return out


class Rig:
    def __init__(self, *, cfg=None, plan=None, frames=None, check=None, operator=None, direction_evidence=True,
                 initial_seq=None, watch=None, real_layer=False, imu=False, real_requests=False, reloc=False,
                 check_builder=True, operating=False, imu_unqualified=False):
        self.direction_evidence = direction_evidence
        self.now = 10_000 * MS
        # the executor's host; its first seq set so the run after the session check crosses the u16 wrap
        self.host = real_host(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100,
                                         outbox_max=16), initial_seq=0xFB80 if initial_seq is None else initial_seq)
        self.port = RealPlantPort(self.host, KIN, lift_motion_counts=True)
        self.mcu = FreshMcu()
        self.transport = FakeTransport(self.mcu)
        self.odo = RealOdometry(VEH, max_gap_s=0.2)
        self.reloc = reloc
        if reloc:
            from forklift_core.localization.slam_pose import SlamPoseTracker

            # 5c-2: the real tracker, tracking from the start (its first correction: identity)
            self.tracker = SlamPoseTracker(max_age_s=ra.SLAM_MAX_AGE_S, hold_limit_m=ra.HOLD_MAX_M, bootstrap_scan_id=0)
            self.tracker.receive(0, -1.0, "processed", (0.0, 0.0, 0.0))
        else:
            self.tracker = Tracker()
        self.answers = list(operator or [])
        self.frames = frames
        geometry = SyntheticMissionGeometry()
        loop_box = {}

        def speed():
            sp = loop_box["loop"].speed
            if self.direction_evidence and sp.valid and len(sp.directions) > 1:
                # the normal example's direction evidence (design 2.2 4: a fake one; the real encoder model's block is
                # tested apart): the truck moves the way its confirmed command goes, forward when standing
                c = loop_box["loop"].book.confirmed
                d = -1 if (c is not None and c.speed_mps < 0) else 1
                from forklift_core.safety.truth_access import SafetySpeed as _S

                return _S(sp.signed_mps, sp.bound_mps, (d,), valid=True, age_s=sp.age_s)
            return sp

        def steering():
            q = loop_box["loop"]._measured_steering()
            return (0.0 if q is None else q,)

        def slam_rear(t):
            if not reloc:
                return np.asarray(loop_box["loop"].odom_pose, dtype=float)
            from forklift_core.localization.start_reloc import rear_from_base

            # the real hooks' contract (design 6, 2b): the age and the hold limit not read while the outputs are closed
            base = self.tracker.map_from_base(t, self.odom_base(), closed=loop_box["loop"].outputs_closed)
            return np.asarray(rear_from_base(base, REAR_OFFSET_M), dtype=float)

        tick = SimpleNamespace(
            slam_rear=slam_rear, speed=speed, steering=steering,
            kinematics=KIN, loaded=lambda: False, cusp_holds=_NullHolds(), cusp_monitor=_cusp_monitor(),
            standing=lambda: bool(loop_box["loop"].core.slam.get("stop_now")),
            now_s=lambda: (self.now - 10_000 * MS) / 1e9)
        built = {}

        def build_check(axis_yaw, rear, base, stamp_s=None, rear_at=None):
            built["check"] = check or _OkCheck()
            loop_box["loop"].core.pocket["check"] = built["check"]
            return built["check"]

        self.layer = None
        if real_layer:
            from forklift_core.live.real_layer import ManualWorker, RealLayer, real_layer_configs
            from forklift_core.live.real_teleop import DelayBound
            from forklift_core.perception.obstacle_grid import AgeErrorTable
            from forklift_core.planning.geometry import Footprint

            class StepWorker(ManualWorker):  # computes at once; the result installs at the next tick head
                held = False  # tests: hold the computation back (``step()`` releases the newest request)

                def submit(self, job, fn):
                    if self.held:
                        self.pending = (job, fn)
                    else:
                        self.done.append((job, fn(job)))

            layer_cfg = __import__("yaml").safe_load((ROOT / "config/real/obstacle_layer_real.yaml").read_text())
            age_ = __import__("json").loads((ROOT / layer_cfg["odometry_age"]).read_text())  # the real layer's table
            table = AgeErrorTable(tuple(age_["ages_s"]), tuple(age_["cumulative_position_m"]),
                                  tuple(age_["cumulative_yaw_rad"]))
            kappa_ = max(abs(k) for k in VEH.steering.curvatures_inv_m)
            hall = SimpleNamespace(x_min_m=-2.0, x_max_m=7.0, y_min_m=-3.0, y_max_m=3.0)
            grid_cfg, perm_cfg = real_layer_configs(layer_cfg, error_table=table, budget_s=0.10, scan_period_s=0.10,
                                                    state_delay_s=0.05, v_scan_mps=0.225, kappa_max_inv_m=kappa_,
                                                    hall=hall)
            # the hull with the forks is what the permission checks; the scans' own outline is the body alone (the
            # Isaac layer's rule) -- the cells under the forks may be FREE
            hull = geometry.unloaded_footprint
            body = Footprint(0.35, hull.rear_m, hull.half_width_m)
            self.layer = RealLayer(grid_cfg, perm_cfg, sensors={"lidar": ((0.0, 0.0, 0.0), True, {"measured": False})},
                                   unloaded=hull,
                                   loaded=geometry.loaded_footprint, body=body,
                                   placer=lambda a, b: loop_box["loop"].place_scan(a, b),
                                   budget_s=0.10, overdue_end_s=1.0, scan_period_s=0.10, scan_max_age_s=0.35,
                                   worker=StepWorker(), odom_now=lambda: loop_box["loop"].odom_travel)
            self.delay_bound = DelayBound()

        self._loop_box = loop_box
        hooks = asm.RealHooks(lift_m=lambda: self.mcu.state().lift_m,
                              odom_base=self.odom_base if reloc else (lambda: (0.0, 0.0, 0.0)),
                              ready=lambda t, inputs: None, started=lambda *a: None, record=lambda *a: None,
                              geometry=geometry, pocket_frame=self._frame,
                              pocket_check=build_check if check_builder else None,  # 5d: no builder (PR 7)
                              operator=((lambda r: loop_box["loop"].operator_check(r)) if real_requests
                                        else self._operator), tick=tick, layer=self.layer,
                              occupancy=(lambda: loop_box["loop"].planning_occupancy()) if real_layer else None)
        base_cfg = cfg or replace(ra.load_s10a(ROOT / "config/real/s10a.yaml"), map_id="hall-1", pallet_depth_m=0.6,
                                  pallet_width_m=0.4, check_pallet_pose=PALLET)
        settings = {"drive_acceleration_mps2": 0.3, "lift_rate_mps": 0.05, "lift_target_m": 0.08,
                    "steering_command_rate_rad_s": 1.0, "withdraw_speed_mps": 0.05, "tracker_curvature_inv_m": CAP}
        planner = SimpleNamespace(curvature_limit_inv_m=CAP)
        self.jobs = FakeJobs()
        core = asm.assemble(
            cfg=base_cfg, loaded_map_id="hall-1", geometry=geometry, pallet_geometry=None, kinematics=KIN,
            settings=settings, planner_config=planner, travel_config=planner,
            speeds={name: 0.1 for name in STAGES}, mission_stages=STAGES, curvatures={"preview": CAP},
            layer=self.layer if real_layer else Layer(), applied=(0.0, 0.0, 0.0), slam_tracker=self.tracker,
            jobs=self.jobs, hooks=hooks,
            extra_options={"cusp_heading_hold_rad": 0.05, "cusp_heading_hold_phases": ("return_home",),
                           "tracker_profile": None, "command_accel_mps2": 0.3, "mid_still_decel_mps2": 0.3,
                           "max_cusp_replans": 0, "cusp_stop_needed": False, "cusp_stop_yaw_rate_radps": 0.05,
                           "cusp_heading_hold_position_m": 0.03, "observe_replans_per_leg": 0,
                           "drive_geometry": None, "rear_axle_offset_m": 0.0, "initial_time": 0.0,
                           "dwell_d_end_m": 0.0, "observe_arrival_m": 0.03, "observe_arrival_yaw_rad": 0.05})
        core.obstacle.update(ticks={}, min_allowed={}, slowed={}, reasons={})
        self.plan = plan or Plan()
        tokens = itertools.count(1)
        self.real_requests = real_requests
        self.console_out: list = []  # what the console received besides the ticks
        self.arbiter = Arbiter(EPOCH, ArbiterConfig(tick_s=0.02, auto_lease_loss="stop", external_stop=True,
                                                    disarm_ignores_control_gen=True, real_requests=real_requests),
                               lambda: f"t{next(tokens)}", mission=core.commands)
        self.channels = RealChannels()
        self.link = RealLink(self.arbiter, self.channels, tick_period_s=0.05, state_period_s=0.1)
        # the session check's random skip fixed at its least (1024) so the mission's frames cross the u16 wrap
        self.permit = ra.AutoPermit(self.host, self.port, outstanding=self.port.outstanding,
                                    stop_since=self._stop_since, rng=_FixedSkip())
        # the arbiter's clock is seconds, the permit's ns (6a-2b design 10.1)
        self.arbiter.rearm_refusal, self.arbiter.on_rearmed = self.permit.arbiter_callbacks()
        params = EncoderSpeedParams(radius_m=VEH.wheel_radius_m, counts_per_rev=VEH.counts_per_wheel_rev,
                                    track_m=VEH.track_m, kappa_max=CAP, axle_accel_mps2=0.5, max_age_s=0.1,
                                    dt_min_s=0.02, dt_max_s=0.2)
        def t_of(n):
            return (int(n) - 10_000 * MS) / 1e9

        def snapshot_view(n):
            lay = self.layer
            if lay is None:
                return ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, False, 0, 0)
            key, applied_v = lay.installed_key(), int(core.obstacle["version"])
            newest = lay.installed_newest_s()  # the installed result's, not the newest placed scan (5a impl P2-7)
            applied_c = tuple(float(v) for v in core.obstacle["applied"])
            wait_ = lay.watchdog(t_of(n))
            return ra.SnapshotView(None if key is None else applied_c, applied_c,
                                   None if newest is None else int(10_000 * MS + newest * 1e9),
                                   lay.installing(), None if key is None else key.correction_gen, applied_v,
                                   wait_ if wait_ == "layer_overdue" else None)

        evidence = ra.Evidence(
            worker_ok=lambda n: True, baseline_ok=lambda n: True,
            layer_ok=(lambda n: self.layer.ok(t_of(n))) if real_layer else (lambda n: True),
            link_reason=lambda n: None if self.host.health(n).fresh else "stale",
            measurement=lambda n: ra.MeasurementView(True, True, True, True),
            odometry_continuous=lambda n: loop_box["loop"].odom_continuous,
            slam=lambda n: ra.SlamView(self.tracker.mode, n, float(getattr(self.tracker, "hold_travel_m", 0.0)),
                                       core.commands.mission_gen),
            snapshot=snapshot_view,
            operator_start_ok=(lambda n: self.loop.operator_start.ok(self.lease)) if real_requests
            else (lambda n: True), core_refusal=lambda n: real_auto_refusal(core.safety_state(None)),
            mission_gen=lambda n: core.commands.mission_gen,
            imu_unqualified=(lambda n: True) if imu_unqualified else None)  # U3a: as the operating assembly gives it
        self.loop = RealLoop(core=core, session=RealSession(core, self.link), permit=self.permit, port=self.port,
                             host=self.host, link=self.link, channels=self.channels, transport=self.transport,
                             params=params, odometry=self.odo, kinematics=KIN, max_speed_mps=0.15,
                             start_ns=10_000 * MS,
                             slam_source=FakeSlam(self) if reloc else SimpleNamespace(replies=lambda n: []),
                             evidence=evidence, base_scenario=core.options.scenario, wait_pose=None,
                             depth_source=SimpleNamespace(frames=self._depth_frames),
                             clock=lambda: self.now + self.tick_cost_ns,
                             real_layer=self.layer, delay_bound=self.delay_bound if real_layer else None,
                             state_delay_s=0.05, pallet_size=(0.6, 0.4),
                             kappa_max_inv_m=max(abs(k) for k in VEH.steering.curvatures_inv_m),
                             operator_start=asm.OperatorStart(EPOCH) if real_requests else None,
                             **({"start_rear": tuple(base_cfg.start_rear), "rear_offset_m": REAR_OFFSET_M}
                                if reloc or operating else {}),
                             operating=operating, imu_unqualified=imu_unqualified,
                             **self._watch_args(watch, imu))
        self.slam = self.loop.slam_source
        if operating:
            self.loop.check_pallet_pose = tuple(float(v) for v in PALLET)  # U3b-1: the rig's pallet is case A's
        loop_box["loop"] = self.loop
        # 5c-2 (design 6, 2b): the arbiter's stop evidence is the physical stand (no SLAM term)
        self.arbiter.stop_evidence = lambda now: self.loop.physical_stop(now)
        self.core = core
        # 5c-1: the real console's connection carries a request token (its real requests go through the ledger)
        self.token = 900_001 if real_requests else None
        self._seq = itertools.count(1)
        self.channels.connect("c1", "op", control=True, token=self.token)
        self.lease = None

    def _watch_args(self, watch, imu):
        """The watches as the assembly builds them: ``imu`` -- build_watches on the real configs (imu: required)
        with a fake 200 Hz source (5b); a bare SafetyWatch -- with the policy ``none`` (the 4c watch tests)."""
        self.imu = None
        if imu:
            import yaml

            autonomy = yaml.safe_load((ROOT / "config/real/autonomy.yaml").read_text())
            watches = asm.build_watches(autonomy, ROOT / "config/real/safety_watch.yaml", VEH.wheel_radius_m)
            if watch is not None:
                watches = asm.Watches(watch, watches.imu, watches.imu_policy)
            self.imu = FakeImu(lambda: self.now)
            return {"watches": watches, "imu_source": self.imu}
        return {} if watch is None else {"watches": asm.Watches(watch, None, "none")}

    # --- sources --------------------------------------------------------------------------------------------------
    def odom_base(self):
        """5c-2 rigs: odom <- base_link from the raw odometry's rear-axle pose."""
        from forklift_core.localization.start_reloc import base_from_rear

        return base_from_rear(self._loop_box["loop"].odom_pose, REAR_OFFSET_M)

    def _stop_since(self, now_ns):
        return self.loop.rearm_stand(now_ns)  # 5c-2 (design 6, 2b): the MCU sample stand and its boot

    def _frame(self):
        if self.frames is None:
            return None
        loop = self.loop
        t = loop._t(self.now)
        return SimpleNamespace(scene="SCENE", stamp_s=t, rear=loop.odom_pose, base=(0.0, 0.0, 0.0),
                               mission_gen=loop.core.commands.mission_gen, phase=loop.core.ms.phase,
                               correction=(0.0, 0.0, 0.0))

    def _depth_frames(self, now_ns):
        if self.frames is None:
            return []
        f = self._frame()
        return [SimpleNamespace(stamp_s=f.stamp_s, depth=None, mission_gen=f.mission_gen, phase=f.phase,
                                correction=f.correction)]

    def _operator(self, request):
        return self.answers.pop(0) if self.answers else None

    # --- driving --------------------------------------------------------------------------------------------------
    echo = True  # the console answers every tick (it keeps the lease); False: a console gone quiet
    extra_boxes: tuple = ()  # 5d: obstacles besides the pallet the fake LiDAR sees, (x, y, half length, half width)
    scan_on = True  # the fake LiDAR revolves (10 Hz) while the rig has a real layer
    _next_scan_ns = None

    def _scan_maybe(self):
        """A revolution every 100 ms: the registered pallet's outline ray-cast from the truck's odometry pose (the
        rig's truth), first beam 100 ms ago, handed to the layer at its last beam."""
        from forklift_core.live.real_layer import RealScan

        if not self.permit.session_started:
            return  # no measurement interval before the session check (the delay bound's anchors), so no scan yet
        if self._next_scan_ns is None:
            self._next_scan_ns = self.now + 100 * MS
            return
        if self.now < self._next_scan_ns:
            return
        self._next_scan_ns += 100 * MS
        loop = self.loop
        t = loop._t(self.now)
        x, y, yaw = loop.odom_pose
        lx, ly, lyaw = x, y, yaw  # the rig's LiDAR over the rear axle: its 0.2 m blind zone stays inside the body
        angles = np.linspace(-math.pi, math.pi, 360, endpoint=False)
        ranges = np.full(len(angles), np.inf)
        px, py, pyaw = PALLET
        hl, hw = 0.3, 0.2
        c, s_ = math.cos(pyaw), math.sin(pyaw)
        corners = [(px + c * u - s_ * v, py + s_ * u + c * v) for u, v in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]
        edges = list(zip(corners, corners[1:] + corners[:1]))
        for bx0, by0, bhl, bhw in self.extra_boxes:  # 5d tests: axis-aligned obstacles (centre, half sizes)
            bc = [(bx0 + bhl, by0 + bhw), (bx0 + bhl, by0 - bhw), (bx0 - bhl, by0 - bhw), (bx0 - bhl, by0 + bhw)]
            edges += list(zip(bc, bc[1:] + bc[:1]))
        for k, a in enumerate(angles):
            dx, dy = math.cos(lyaw + a), math.sin(lyaw + a)
            best = math.inf
            for (ax, ay), (bx, by) in edges:
                ex, ey = bx - ax, by - ay
                den = dx * ey - dy * ex
                if abs(den) < 1e-12:
                    continue
                tt = ((ax - lx) * ey - (ay - ly) * ex) / den
                uu = ((ax - lx) * dy - (ay - ly) * dx) / den
                if tt > 0.0 and 0.0 <= uu <= 1.0:
                    best = min(best, tt)
            ranges[k] = best
        # a level mount: every beam stays in its band (the limit an S7-measured tilt would give is finite)
        scan = RealScan("lidar", t - 0.1, t, angles, ranges, np.ones(len(angles), bool), np.zeros(len(angles), bool),
                        np.full(len(angles), np.inf))
        self.layer.add_scan(scan, t, initial_correction=tuple(self.core.obstacle["applied"]),
                            initial_version=int(self.core.obstacle["version"]))

    tick_cost_ns = 0  # the tick's own computing time before its first write (the fake clock's)
    step_ns = 20 * MS  # the start interval

    def tick(self, n=1):
        rec = None
        for _ in range(n):
            self.now += self.step_ns
            if self.layer is not None and self.scan_on:
                self._scan_maybe()
            rec = self.loop.cycle(self.now)
            out_ = self.channels.take_out()
            self.console_out.extend(m for _, m in out_ if m.get("t") != "tick")  # replies, real_status (5c-1)
            ticks = [m for _, m in out_ if m.get("t") == "tick"]
            if ticks and self.echo and self.lease is not None:
                last = ticks[-1]
                self.channels.message("c1", "op", {"t": "echo", "session_epoch": EPOCH, "lease_id": self.lease,
                                                   "echo": {"state_seq": last["state_seq"], "token": last["token"]}})
            if self.core.commands.status == "preparing" and self.jobs.submitted and self.core.pending is not None \
                    and self.core.pending.job not in self.jobs.answers:
                self.jobs.answers[self.core.pending.job] = (True, {"result": self.plan, "trace": []})
        return rec

    def say(self, kind_, /, **fields):
        kind = kind_  # positional only: a request may carry its own ``kind`` field (operator_answer)
        msg = {"t": kind, "request_id": f"r{next(_REQ)}", "session_epoch": EPOCH,
               "control_gen": self.arbiter.control_gen, **fields}
        if self.lease is not None:
            msg.setdefault("lease_id", self.lease)
        if self.token is not None and (kind in REAL_LEASED or kind == "recovery"):
            msg.setdefault("request_token", self.token)
            msg.setdefault("request_seq", next(self._seq))
        self.channels.message("c1", "op", msg)
        return msg["request_id"]

    def grant(self):
        self.tick(3)
        self.say("lease_request")
        self.tick(1)
        self.lease = self.arbiter.lease.lease_id

    def start(self):
        """A session started (the two DISARMs answered) and the lease held."""
        self.permit.bind(transmit=lambda: self.loop._transmit(self.now), arbiter_stop=lambda r: None,
                         collect=lambda r: None)
        self.tick(2)
        self.permit.start_session(self.now)
        self.tick(6)
        assert self.permit.session_started
        self.grant()

    def submit(self):
        rev = asm.register_pallet(self.core, self.core.targets.pallets and next(iter(self.core.targets.pallets)),
                                  PALLET)
        self.tick(30)  # stands: the still span and the arbiter's stand
        self.say("mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=rev)

    def loop_drives(self, since=0):
        from forklift_core.mcu_link.messages import Drive

        return [m for _, k, _, m in self.transport.sent[since:] if isinstance(m, Drive)]


_REQ = itertools.count(1)


class _NullHolds:
    """The cusp heading hold records: S10-A's phases never hold (the hold phases are return_home only)."""

    def sync(self, *a, **k):
        pass

    def tick(self, *a, **k):
        pass

    def held(self, *a, **k):
        return {}

    def decide(self, *a, **k):
        pass

    def close_watch(self, *a, **k):
        pass


def _cusp_monitor():
    from forklift_core.control.cusp_stall import CuspStallMonitor

    return CuspStallMonitor([])


class _OkCheck:
    def region_control(self):
        """The pallet's pocket region (x, y, length, width, yaw): the layer's exemption while this check gates."""
        return (PALLET[0], PALLET[1], 0.6 + 2 * 0.6, 0.4 + 2 * 0.2, PALLET[2])  # the pallet plus PocketCheck's bands

    def __init__(self):
        self.last_new_s, self.valid = 0.0, True
        self.config = SimpleNamespace(frame_lag_s=0.0, frame_max_age_s=0.2)

    def limit(self, *a):
        return math.inf, "ok"

    def summary(self):
        return {}

    def add_frame(self, stamp=None, *a, **k):
        if stamp is not None:
            self.last_new_s = float(stamp)  # 5d (7.3): the frame's acquisition, as the real check keeps it
        return {"new": True}


class DirCheck(_OkCheck):
    """5d (design 7.1): a pocket check answering per direction -- ``allow[d]`` (m/s, inf by default) -- recording every
    query (time, direction, speed)."""

    def __init__(self):
        super().__init__()
        self.allow = {1: math.inf, -1: math.inf}
        self.reason = {1: "ok", -1: "ok"}
        self.calls = []

    def limit(self, now_s, rear, kappa, direction, speed, lift):
        self.calls.append((float(now_s), int(direction), float(speed)))
        a = self.allow[1 if direction >= 0 else -1]
        return a, self.reason[1 if direction >= 0 else -1] if a < math.inf else "ok"
