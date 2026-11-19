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
from forklift_core.safety.arbiter import Arbiter, ArbiterConfig

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
                and self.zero_seen:
            st = _r(st, flags=st.flags | StateFlag.COMMAND_FRESH)
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

    def receive(self, host, now_ns, limit=64):
        steps = 0 if self.last_ns is None else max(0, int((now_ns - self.last_ns) // (10 * MS)))
        self.last_ns = now_ns if self.last_ns is None else self.last_ns + steps * 10 * MS
        for _ in range(min(steps, limit)):
            self.mcu.advance(10_000)
            host.on_bytes(encode(self.mcu.state(), 1), now_ns)
        if steps == 0:
            host.on_bytes(encode(self.mcu.state(), 1), now_ns)

    def transmit(self, host, now_ns, gate):
        out = []
        while (f := host.next_frame(now_ns)) is not None:
            def write(f=f):
                host.frame_written(f, now_ns)
                for r in self.dec.feed(f.data, now_ns):
                    self.sent.append((now_ns, f.kind, f.seq, r.message))
                    self.mcu.receive(r.message, f.seq)

            if gate(f, write):
                out.append((f.kind, f.seq, now_ns))
            else:
                host.unsent(f, now_ns)
                host.drop_drives("send_gate")
                break
        return out


class Rig:
    def __init__(self, *, cfg=None, plan=None, frames=None, check=None, operator=None, direction_evidence=True,
                 initial_seq=None):
        self.direction_evidence = direction_evidence
        self.now = 10_000 * MS
        # the executor's host; its first seq set so the run after the session check crosses the u16 wrap
        self.host = real_host(HostConfig(state_max_age_s=0.05, cap_ms=150, arm_timeout_s=0.3, zero_valid_ms=100,
                                         outbox_max=16), initial_seq=0xFB80 if initial_seq is None else initial_seq)
        self.port = RealPlantPort(self.host, KIN, lift_motion_counts=True)
        self.mcu = FreshMcu()
        self.transport = FakeTransport(self.mcu)
        self.odo = RealOdometry(VEH, max_gap_s=0.2)
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

        tick = SimpleNamespace(
            slam_rear=lambda t: np.asarray(loop_box["loop"].odom_pose, dtype=float), speed=speed, steering=steering,
            kinematics=KIN, loaded=lambda: False, cusp_holds=_NullHolds(), cusp_monitor=_cusp_monitor(),
            standing=lambda: bool(loop_box["loop"].core.slam.get("stop_now")),
            now_s=lambda: (self.now - 10_000 * MS) / 1e9)
        built = {}

        def build_check(axis_yaw, rear, base, stamp_s=None, rear_at=None):
            built["check"] = check or _OkCheck()
            loop_box["loop"].core.pocket["check"] = built["check"]

        hooks = asm.RealHooks(lift_m=lambda: self.mcu.state().lift_m, odom_base=lambda: (0.0, 0.0, 0.0),
                              ready=lambda t, inputs: None, started=lambda *a: None, record=lambda *a: None,
                              geometry=geometry, pocket_frame=self._frame, pocket_check=build_check,
                              operator=self._operator, tick=tick)
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
            layer=Layer(), applied=(0.0, 0.0, 0.0), slam_tracker=self.tracker, jobs=self.jobs, hooks=hooks,
            extra_options={"cusp_heading_hold_rad": 0.05, "cusp_heading_hold_phases": ("return_home",),
                           "tracker_profile": None, "command_accel_mps2": 0.3, "mid_still_decel_mps2": 0.3,
                           "max_cusp_replans": 0, "cusp_stop_needed": False, "cusp_stop_yaw_rate_radps": 0.05,
                           "cusp_heading_hold_position_m": 0.03, "observe_replans_per_leg": 0,
                           "drive_geometry": None, "rear_axle_offset_m": 0.0, "initial_time": 0.0,
                           "dwell_d_end_m": 0.0, "observe_arrival_m": 0.03, "observe_arrival_yaw_rad": 0.05})
        core.obstacle.update(ticks={}, min_allowed={}, slowed={}, reasons={})
        self.plan = plan or Plan()
        tokens = itertools.count(1)
        self.arbiter = Arbiter(EPOCH, ArbiterConfig(tick_s=0.02, auto_lease_loss="stop", external_stop=True,
                                                    disarm_ignores_control_gen=True),
                               lambda: f"t{next(tokens)}", mission=core.commands)
        self.channels = RealChannels()
        self.link = RealLink(self.arbiter, self.channels, tick_period_s=0.05, state_period_s=0.1)
        # the session check's random skip fixed at its least (1024) so the mission's frames cross the u16 wrap
        self.permit = ra.AutoPermit(self.host, self.port, outstanding=self.port.outstanding,
                                    stop_since=self._stop_since, rng=_FixedSkip())
        self.arbiter.rearm_refusal = self.permit.rearm_refusal
        self.arbiter.on_rearmed = self.permit.rearmed
        params = EncoderSpeedParams(radius_m=VEH.wheel_radius_m, counts_per_rev=VEH.counts_per_wheel_rev,
                                    track_m=VEH.track_m, kappa_max=CAP, axle_accel_mps2=0.5, max_age_s=0.1,
                                    dt_min_s=0.02, dt_max_s=0.2)
        evidence = ra.Evidence(
            worker_ok=lambda n: True, layer_ok=lambda n: True, baseline_ok=lambda n: True,
            link_reason=lambda n: None if self.host.health(n).fresh else "stale",
            measurement=lambda n: ra.MeasurementView(True, True, True, True),
            odometry_continuous=lambda n: loop_box["loop"].odom_continuous,
            slam=lambda n: ra.SlamView(self.tracker.mode, n, 0.0, core.commands.mission_gen),
            snapshot=lambda n: ra.SnapshotView((0.0, 0.0, 0.0), (0.0, 0.0, 0.0), n, False, 0, 0),
            operator_start_ok=lambda n: True, core_refusal=lambda n: real_auto_refusal(core.safety_state(None)),
            mission_gen=lambda n: core.commands.mission_gen)
        self.loop = RealLoop(core=core, session=RealSession(core, self.link), permit=self.permit, port=self.port,
                             host=self.host, link=self.link, channels=self.channels, transport=self.transport,
                             params=params, odometry=self.odo, kinematics=KIN, max_speed_mps=0.15,
                             start_ns=10_000 * MS, slam_source=SimpleNamespace(replies=lambda n: []),
                             evidence=evidence, base_scenario=core.options.scenario, wait_pose=None,
                             depth_source=SimpleNamespace(frames=self._depth_frames))
        loop_box["loop"] = self.loop
        self.arbiter.stop_evidence = lambda now: (self.host.health(int(now * 1e9)).fresh,
                                                  bool(core.slam.get("stop_now")))
        self.core = core
        self.channels.connect("c1", "op", control=True)
        self.lease = None

    # --- sources --------------------------------------------------------------------------------------------------
    def _stop_since(self, now_ns):
        s = self.loop.still_since_ns
        return None if s is None or self.host.state is None else (s, self.host.state.boot_id)

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

    def tick(self, n=1):
        rec = None
        for _ in range(n):
            self.now += 20 * MS
            rec = self.loop.cycle(self.now)
            ticks = [m for _, m in self.channels.take_out() if m.get("t") == "tick"]
            if ticks and self.echo and self.lease is not None:
                last = ticks[-1]
                self.channels.message("c1", "op", {"t": "echo", "session_epoch": EPOCH, "lease_id": self.lease,
                                                   "echo": {"state_seq": last["state_seq"], "token": last["token"]}})
            if self.core.commands.status == "preparing" and self.jobs.submitted and self.core.pending is not None \
                    and self.core.pending.job not in self.jobs.answers:
                self.jobs.answers[self.core.pending.job] = (True, {"result": self.plan, "trace": []})
        return rec

    def say(self, kind, **fields):
        msg = {"t": kind, "request_id": f"r{next(_REQ)}", "session_epoch": EPOCH,
               "control_gen": self.arbiter.control_gen, **fields}
        if self.lease is not None:
            msg.setdefault("lease_id", self.lease)
        self.channels.message("c1", "op", msg)

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
    def __init__(self):
        self.last_new_s, self.valid = 0.0, True
        self.config = SimpleNamespace(frame_lag_s=0.0, frame_max_age_s=0.2)

    def limit(self, *a):
        return math.inf, "ok"

    def summary(self):
        return {}

    def add_frame(self, *a, **k):
        return {"new": True}
