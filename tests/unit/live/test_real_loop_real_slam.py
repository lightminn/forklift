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
from test_real_slam_source import _OPEN, Bridge, _close_sockets  # noqa: F401 -- the fixture closes the pair


class AutoBridge(Bridge):
    """Answers a start or seed with ``seeding`` / ``ready_for_scans``, every scan at once with the correction its
    start pose implies on its first scan's odometry, and the seventh start scan with ``end``."""

    def __init__(self, sock, end="ready"):
        super().__init__(sock)
        self.end, self.cur = end, None
        self.hold, self.held = False, []  # hold: the next replies are kept until release()

    def release(self):
        self.hold = False
        for s in self.held:
            self.reply(s, corr=self.cur["corr"])
        self.held = []

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
                if self.hold:
                    self.held.append(f)
                    continue
                self.reply(f, corr=c["corr"])
                if c["n"] == 7 and c["end"] != "never":
                    self.state(c["id"], c["end"], None if c["end"] == "ready" else "NOT_LOCALIZED: test")
            elif isinstance(f, rl.Cancel):
                self.state(f.seed_id, "void", f.reason)


def real(r, end="ready"):
    a, b = socket.socketpair(socket.AF_UNIX, socket.SOCK_STREAM)
    _OPEN.extend((a, b))
    bridge = AutoBridge(b, end)
    loop = r.loop
    src = RealSlamSource(
        RealSlamLink(a), mode="operating", map_id="m" * 64, start_ns=loop.start_ns,
        start_pose=base_from_rear(tuple(loop.start_rear), REAR_OFFSET_M),
        odom_history=lambda: loop.odom_history, odom_base=r.odom_base, odom_gen=lambda: loop.odom_gen,
        boot=lambda: loop.host.state.boot_id, rear_offset_m=REAR_OFFSET_M, clock=lambda: r.now)
    marks = []
    loop.slam_source = _SlamGuard(src, marks.append, Counter())
    loop.odom_listeners.append(src.odom_step)
    plain = r.tick

    def tick(n=1):
        rec = None
        for _ in range(n):
            t = r.now - 40 * MS
            src.offer(srs.SlamScanIn(key=("rig", t), gen=("rig",), stamp_ns=t, flo_ns=t - MS, fhi_ns=t + MS, measured=True,
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



def test_a_break_in_a_tick_that_never_polls_the_source_still_cancels_the_seed():
    """Codex 7c-1b-2a round 2 P1: an encoder gap in a slow tick (no SLAM step that tick) -- the generation, not the
    flag of the next tick, cancels the seed."""
    from forklift_core.mcu_link.messages import FaultFlag

    r = rig()
    src, bridge, marks = real(r)
    gen = open_generation(r)
    run_until(r, lambda rec: src.can_confirm(), 200)
    stand(r)
    bridge.end = "never"  # the seed's start check never ends on its own
    ask(r, "relocalize_declare", generation=gen)
    run_until(r, lambda rec: any(isinstance(f, rl.Seed) for f in bridge.got), 50)
    step = r.step_ns
    r.step_ns = 80 * MS
    r.mcu.faults = FaultFlag.ENCODER_GAP
    r.tick()
    r.mcu.faults = FaultFlag(0)
    r.step_ns = step
    r.tick(5)
    assert any(isinstance(f, rl.Cancel) and f.reason == "odom_break" for f in bridge.got)
    assert not src.can_confirm()



def test_a_reply_of_the_same_instance_from_before_a_new_epoch_is_dropped_by_the_loop():
    """Codex 7c-1b-2b round 2 P2-3: a relocalization keeps the instance; the reply of a scan acquired before the new
    epoch, released after it within the 0.25 s limit, passes the source with its old epoch and the loop's epoch
    filter drops it -- never the applied correction's provenance."""
    r = rig()
    src, bridge, marks = real(r)
    run_until(r, lambda rec: src.can_confirm(), 200)
    r.tick(10)
    bridge.hold = True
    run_until(r, lambda rec: bridge.held, 20)
    held_id = bridge.held[0].scan.scan_id
    passed = []
    plain = r.core.slam_replies
    r.core.slam_replies = lambda t, replies, loaded: passed.extend(it[0].scan_id for it in replies) or plain(
        t, replies, loaded)
    before = r.loop.reloc_dropped["epoch"]
    gen = open_generation(r)
    assert r.loop.reloc_gen == gen
    bridge.release()
    r.tick(2)
    assert r.loop.reloc_dropped["epoch"] > before
    assert held_id not in passed  # never handed to the core (Codex 7c-1b-2b round 3 P3)
    prov = r.loop.provenance
    assert prov is None or prov[0] == r.loop.epoch
    assert not marks


# --- PR 7 7r plan 2-3: the HIL check's round and the anchor -------------------------------------------------------------
def _hil(r, boot=1):
    """The rig's loop as a HIL client on a transport whose check holds for ``boot``."""
    r.loop._hil_client = True
    r.loop.transport.hil_plant, r.loop.transport.hil_boot, r.loop.transport.hil_losses = True, boot, 0
    r.tick()


def _lose(r):
    r.loop.transport.hil_plant, r.loop.transport.hil_boot = False, None
    r.loop.transport.hil_losses += 1


def test_a_confirmation_that_took_its_reply_ends_on_a_loss_before_it_completes():
    """Codex 7r plan round 4 P1: the processed after the request already taken, the completion waiting -- the HIL
    check lost in between: never confirmed, ended hil_check_lost (the check just before the completion)."""
    r = rig()
    src, bridge, marks = real(r)
    _hil(r)
    gen = open_generation(r)
    run_until(r, lambda rec: src.can_confirm(), 200)
    stand(r)
    reply, _ = ask(r, "relocalize_confirm", generation=gen)
    assert reply["t"] == "ack", reply
    assert r.loop.attempt["hil_round"] == r.loop.hil_check_round
    # the reference taken, the completion not yet: closed on the commit only once the layer has installed after it
    lay = r.loop.real_layer
    installed = lay.installed_newest_s
    lay.installed_newest_s = lambda: None  # the install after the request held back
    run_until(r, lambda rec: r.loop.attempt is not None and r.loop.attempt["reference"], 200)
    assert status(r)["state"] == "waiting"
    _lose(r)
    lay.installed_newest_s = installed
    r.tick(3)
    assert status(r)["state"] == "refused" and status(r)["reason"] == "hil_check_lost", status(r)
    assert r.loop.core.relocalize_generation() == gen  # still open: nothing closed it


def test_a_boot_change_during_a_declaration_ends_it_and_marks_no_seed_declared():
    """7r plan 3: a declaration whose seed runs across a boot change completes nothing -- no declared, no F1 mark."""
    r = rig()
    src, bridge, marks = real(r)
    _hil(r)
    gen = open_generation(r)
    run_until(r, lambda rec: src.can_confirm(), 200)
    stand(r)
    bridge.end = "never"
    reply, _ = ask(r, "relocalize_declare", generation=gen)
    assert reply["t"] == "ack", reply
    run_until(r, lambda rec: any(isinstance(f, rl.Seed) for f in bridge.got), 50)
    r.loop.transport.hil_boot = 2  # held, on another boot
    r.tick()
    assert status(r)["state"] == "refused" and status(r)["reason"] == "hil_check_lost", status(r)
    bridge.end = "ready"
    r.tick(20)
    assert status(r)["state"] == "refused"
    assert not any(row.get("declared") for row in r.loop.seed_log)
    assert r.loop.f1_block is None or not r.loop.f1_block.get("declared")


def test_no_attempt_starts_while_the_check_is_lost_and_off_hil_there_is_no_round():
    r = rig()
    src, bridge, marks = real(r)
    gen = open_generation(r)
    run_until(r, lambda rec: src.can_confirm(), 200)
    stand(r)
    assert r.loop._hil_watch() is None  # the rig's host is not a HIL client: nothing changes there
    _hil(r)
    _lose(r)
    stand(r)
    reply, _ = ask(r, "relocalize_confirm", generation=gen)
    assert reply["t"] == "reject" and reply["reason"] == "hil_check_lost", reply
    assert r.loop.hil_check_round == 1


def test_the_anchor_is_the_first_state_with_a_finite_delay_bound_and_is_called_once():
    from real_loop_rig import Rig

    got = []
    r = Rig(frames=True, real_requests=True, reloc=True, real_layer=True)
    r.loop.on_first_bounded_state = got.append
    r.start()
    r.tick(50)
    assert len(got) == 1 and got[0] == r.loop.anchor_rx_ns
    bounded = sorted(rx for rx, d in r.loop._sample_delay.values() if d is not None and math.isfinite(d))
    unbounded = [rx for rx, d in r.loop._sample_delay.values() if d is None]
    assert bounded[0] == got[0] and all(rx <= got[0] for rx in unbounded if rx < bounded[0])


def test_a_loss_hidden_by_a_new_boot_in_the_same_read_still_ends_the_attempt():
    """Codex 7r-2 round 1 P1: only the transport's loss count moved (and the boot): the round moves, the attempt ends."""
    r = rig()
    src, bridge, marks = real(r)
    _hil(r)
    gen = open_generation(r)
    run_until(r, lambda rec: src.can_confirm(), 200)
    stand(r)
    reply, _ = ask(r, "relocalize_confirm", generation=gen)
    assert reply["t"] == "ack", reply
    r.loop.transport.hil_losses += 1  # lost and held again within one read, on the same boot as far as it shows
    r.tick()
    assert status(r)["state"] == "refused" and status(r)["reason"] == "hil_check_lost", status(r)
