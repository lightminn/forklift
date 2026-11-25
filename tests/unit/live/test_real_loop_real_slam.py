"""PR 7c-1b-2: the real SLAM source behind the run's guard in the real loop on the fake truck (design 6.1, 2), with
an in-process fake bridge that follows the protocol: the session start to ``ready``, a confirmation whose forced
request a real scan answers, a declaration that completes on its seed's ``ready`` -- or latches manual-only on
``not_localized``. The pty HIL run is 7c-1b-2b."""

import math
import socket
from collections import Counter

import numpy as np

from forklift_core.live.real_run import _SlamGuard
from forklift_core.live.real_slam_source import RealSlamLink, RealSlamSource
from forklift_core.localization import slam_real_link as rl
from forklift_core.localization import slam_real_scan as srs
from forklift_core.localization.slam_pose import compose, invert
from forklift_core.localization.start_reloc import base_from_rear

from real_loop_rig import MS, REAR_OFFSET_M
from test_real_loop_console import ask, run_until
from test_real_loop_relocalize import open_generation, rig, stand, status
from test_real_slam_source import Bridge


class AutoBridge(Bridge):
    """Answers a start or seed with ``seeding`` / ``ready_for_scans``, every scan at once with the correction its
    start pose implies on its first scan's odometry, and the seventh start scan with ``end``."""

    def __init__(self, sock, end="ready"):
        super().__init__(sock)
        self.end, self.cur = end, None

    def pump(self):
        for f in self.frames():
            if isinstance(f, (rl.Hello, rl.Seed)):
                ident = f.start_id if isinstance(f, rl.Hello) else f.seed_id
                self.cur = {"id": ident, "pose": f.start_pose, "n": 0, "corr": None,
                            "end": "ready" if isinstance(f, rl.Hello) else self.end}
                self.state(ident, "seeding")
                self.state(ident, "ready_for_scans")
            elif isinstance(f, rl.RealScan):
                c = self.cur
                if c["corr"] is None:
                    c["corr"] = tuple(compose(c["pose"], invert(f.scan.odom_from_base)))
                c["n"] += 1
                self.reply(f, corr=c["corr"])
                if c["n"] == 7:
                    self.state(c["id"], c["end"], None if c["end"] == "ready" else "NOT_LOCALIZED: test")


def real(r, end="ready"):
    a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
    bridge = AutoBridge(b, end)
    loop = r.loop
    src = RealSlamSource(
        RealSlamLink(a), map_id="m" * 64, start_ns=loop.start_ns,
        start_pose=base_from_rear(tuple(loop.start_rear), REAR_OFFSET_M),
        odom_history=lambda: loop.odom_history, odom_base=r.odom_base, boot=lambda: loop.host.state.boot_id,
        rear_offset_m=REAR_OFFSET_M, clock=lambda: r.now)
    marks = []
    loop.slam_source = _SlamGuard(src, marks.append, Counter())
    plain = r.tick

    def tick(n=1):
        rec = None
        for _ in range(n):
            t = r.now - 40 * MS
            src.offer(srs.SlamScanIn(key=("rig", t), stamp_ns=t, flo_ns=t - MS, fhi_ns=t + MS, measured=True,
                                     angles=np.linspace(-math.pi, math.pi, 16, endpoint=False),
                                     ranges=np.full(16, 3.0), self_hit=np.zeros(16, bool), range_min=0.1,
                                     range_max=10.0))
            rec = plain()
            bridge.pump()
        return rec

    r.tick = tick
    return src, bridge, marks


def test_the_session_start_then_a_confirmation_through_the_real_source():
    r = rig()
    src, bridge, marks = real(r)  # the source from the session's start: it sees every epoch the loop opens
    gen = open_generation(r)
    run_until(r, lambda rec: src.can_confirm(), 200)
    assert any(isinstance(f, rl.Hello) for f in bridge.got)
    stand(r)
    reply, _ = ask(r, "relocalize_confirm", generation=gen)
    assert reply["t"] == "ack", reply
    run_until(r, lambda rec: status(r)["state"] != "waiting", 300)
    assert status(r)["state"] == "confirmed", status(r)
    assert not marks and r.loop.fatal_source is None


def test_a_declaration_completes_on_its_seeds_ready():
    r = rig()
    src, bridge, marks = real(r)  # the source from the session's start: it sees every epoch the loop opens
    gen = open_generation(r)
    run_until(r, lambda rec: src.can_confirm(), 200)
    stand(r)
    reply, _ = ask(r, "relocalize_declare", generation=gen)
    assert reply["t"] == "ack", reply
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    assert status(r)["state"] == "declared", status(r)
    [seed] = [f for f in bridge.got if isinstance(f, rl.Seed)]
    target = base_from_rear(tuple(r.loop.start_rear), REAR_OFFSET_M)
    assert seed.start_pose == tuple(float(v) for v in target)
    assert r.loop.manual_only is None and not marks
    assert any(e["event"] == "seed_end" and e["seed_id"] == seed.seed_id for e in r.loop.reloc_events)


def test_a_not_localized_seed_latches_manual_only():
    r = rig()
    src, bridge, marks = real(r, end="not_localized")  # the source from the session's start: it sees every epoch the loop opens
    gen = open_generation(r)
    run_until(r, lambda rec: src.can_confirm(), 200)
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    assert status(r)["state"] == "declared", status(r)
    assert r.loop.manual_only is not None and r.loop.manual_only["seed_id"] is not None
    reply, _ = ask(r, "mode_request", to="AUTO")
    assert reply["reason"] == "operator_placed_start"
    assert not marks
