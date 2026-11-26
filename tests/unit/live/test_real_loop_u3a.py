"""The operating unlock U3a (docs/plans/2026-11-26-real-operating-assembly.md 2.5, 2.7) on the fake truck: the operating
run's layer_timing not bypassable (HIL unchanged), the scan evidence boundary's latch (the layer's evidence, the
processed freshness and a relocalization attempt in progress thrown away, the boundary judged before any install),
and an IMU that did not qualify refused at submit and arm until the session's imu bypass."""

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.live import real_loop as rl

from real_loop_rig import Rig


def run_until(r, cond, limit=6000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def replies(r):
    out, r.console_out[:] = list(r.console_out), []
    return [m for m in out if m.get("t") in ("ack", "reject")]


def ask(r, kind, /, **fields):
    rid = r.say(kind, **fields)
    r.tick()
    mine = [m for m in replies(r) if m.get("request_id") == rid]
    assert mine, (kind, r.console_out)
    return mine[0]


# --- layer_timing ---------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("operating, want", [(True, "not_bypassable"), (False, None)])
def test_layer_timing_is_not_bypassable_on_the_operating_run_only(operating, want):
    r = Rig(frames=True, real_requests=True, operating=operating)
    r.start()
    r.console_out.clear()
    reply = ask(r, "check_bypass", check="layer_timing", on=True)
    assert reply.get("reason") == want, reply
    if want is None:
        assert reply["t"] == "ack"  # HIL and the rigs keep the session bypass (plan 2.5)


# --- the scan evidence boundary ---------------------------------------------------------------------------------------
def _driving(operating=True):
    r = Rig(frames=True, operator=[(1, "confirm")], real_layer=True, operating=operating)
    r.start()
    r.tick(40)
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and rec.branch == "auto")
    return r


def test_a_boundary_latches_and_throws_the_evidence_away():
    r = _driving()
    lay = r.layer
    assert lay.installed_key() is not None and lay.grid.scans
    serial = lay.exempt_serial
    r.loop._fresh_ns = 123  # noqa: SLF001 -- a processed freshness of this epoch
    r.loop.scan_boundary(r.now, "lidar")
    assert lay.installed_key() is None and not lay.grid.scans and lay.exempt_serial == serial + 1
    assert r.loop._fresh_ns is None  # noqa: SLF001 -- a reply taken before never vouches for the new generation
    rec = r.tick()
    assert (rec.branch, rec.reason) == ("emergency", "scan_generation_changed")
    r.tick(10)
    assert r.permit.state in (ra.DISARMING, ra.LATCHED) and r.loop.scan_boundaries == 1


def test_the_boundary_is_judged_before_this_ticks_install():
    """Codex U3 split round 1 P1-2: a result finished for the old evidence is never installed in the boundary's tick."""
    r = _driving()
    lay = r.layer
    installs = lay.counts["installs"]
    fired = []

    def events(now_ns):
        if not fired:
            fired.append(now_ns)
            r.loop.scan_boundary(now_ns, "lidar")

    r.loop.scan_events = events
    r.tick()
    # the rig's worker answers at once: without the early judgement this tick's head would have installed a result
    assert fired and lay.installed_key() is None and lay.counts["installs"] == installs


def test_the_boundary_is_not_judged_early_off_the_operating_run():
    r = _driving(operating=False)
    called = []
    r.loop.scan_events = lambda n: called.append(n)
    r.tick(3)
    assert called == []  # the HIL and rig loops keep their tick order


def test_a_relocalization_attempt_in_progress_is_refused_even_while_latched():
    """Codex U3 split round 3 P2: already latched, an attempt open, the boundary closes it (no late completion)."""
    r = _driving()
    r.loop.scan_boundary(r.now, "lidar")
    r.tick(10)
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)
    r.loop.attempt = {"attempt": 9, "kind": "confirm"}
    r.loop.scan_boundary(r.now, "lidar")
    assert r.loop.attempt is None
    assert (r.loop.relocalize_status["state"], r.loop.relocalize_status["reason"]) == (
        "refused", "relocalize_voided:scan_boundary")


def test_after_a_boundary_readiness_needs_the_new_generations_own_install():
    """Nothing of the old evidence answers: right after the boundary the layer has no result; only the scans after
    it (the rig keeps sending them) make a new install, and the latch still waits for the operator's rearm."""
    r = _driving()
    r.loop.scan_boundary(r.now, "lidar")
    assert r.loop._ready(r.now, "drive") == "layer:missing"  # noqa: SLF001 -- not ready on the old evidence
    installs = r.layer.counts["installs"]
    r.tick(20)
    assert r.layer.counts["installs"] > installs  # the new generation's own result
    assert r.permit.state == ra.LATCHED  # no resume of its own


def test_the_end_is_one_of_the_layer_ends():
    assert "scan_generation_changed" in rl.LAYER_ENDS and rl.OPERATING_NOT_BYPASSABLE == ("layer_timing",)


# --- IMU unqualified ------------------------------------------------------------------------------------------------
def test_an_unqualified_imu_on_the_operating_run_has_no_watch_or_source():
    r = Rig(frames=True, imu=True, operating=True, imu_unqualified=True)
    assert r.loop.imu_unqualified and r.loop.imu_watch is None and r.loop.imu_source is None


def test_imu_unqualified_needs_the_operating_run():
    r = Rig(frames=True, imu=True, operating=False, imu_unqualified=True)
    assert not r.loop.imu_unqualified and r.loop.imu_watch is not None  # off the operating run nothing changes


def test_an_unqualified_imu_refuses_the_submit_until_the_imu_bypass():
    r = Rig(frames=True, real_requests=True, operating=True, imu_unqualified=True)
    r.start()
    r.console_out.clear()
    assert r.loop._ready(r.now, "submit") == "imu:unqualified"  # noqa: SLF001
    assert r.loop._ready(r.now, "drive") == "imu:unqualified"  # noqa: SLF001
    reply = ask(r, "check_bypass", check="imu", on=True)
    assert reply["t"] == "ack"
    r.tick(2)
    assert r.loop._ready(r.now, "submit") != "imu:unqualified"  # noqa: SLF001
    status = r.loop.real_status(r.now)
    assert status["operating"] is True and status["imu_unqualified"] is True and "imu" in status["bypassed"]


# --- Codex U3a round 1 ----------------------------------------------------------------------------------------------
def test_a_boundary_drops_a_layer_request_not_started_yet():
    """P2: a request waiting in the worker would compute the old grid and delay the new generation's."""
    from forklift_core.live.real_layer import ManualWorker

    r = _driving()
    w = ManualWorker()
    r.layer.worker = w
    w.submit({"key": "old"}, lambda job: {"snapshot": None})
    r.loop.scan_boundary(r.now, "lidar")
    assert w.pending is None
    w.step()
    assert w.take() == []


def test_a_late_reply_of_the_old_generation_never_refreshes_the_evidence():
    """P2: after the boundary only a processed reply whose evidence began after it vouches for SLAM; the new layer's
    install and that reply are each needed (readiness names what is still missing)."""
    r = Rig(frames=True, real_requests=True, reloc=True, real_layer=True, operating=True)
    r.start()
    r.tick(40)
    loop = r.loop
    loop.scan_boundary(r.now, "lidar")
    t_b = loop._t(r.now)  # noqa: SLF001
    assert loop._fresh_ns is None and loop._ready(r.now, "drive") == "layer:missing"  # noqa: SLF001
    # a reply of a scan taken before the boundary arrives late
    r.slam.auto = False
    r.slam.extra.append(r.slam.reply(t_b - 0.05))
    r.tick()
    assert loop._fresh_ns is None and loop.reloc_dropped["before_scan_boundary"] == 1  # noqa: SLF001
    # the new layer is installed from the scans after the boundary; SLAM still has nothing new
    r.tick(20)
    assert r.layer.installed_key() is not None
    assert loop._fresh_ns is None  # noqa: SLF001
    assert (loop._ready(r.now, "drive") or "").startswith("slam:")  # noqa: SLF001 -- the new layer alone is not enough
    # a reply of a scan after the boundary: both new pieces of evidence are there
    r.slam.extra.append(r.slam.reply(loop._t(r.now)))  # noqa: SLF001
    r.tick()
    assert loop._fresh_ns is not None  # noqa: SLF001


def _rearmed_after_a_boundary(*, slam_replies: bool, layer_installs: bool):
    """Codex U3a rounds 1-2 P2: an operating truck latched on a boundary, rearmed by the operator, then given only the
    new evidence named -- the real ARM path decides."""
    from test_real_loop_console import ask

    r = Rig(frames=True, real_requests=True, reloc=True, real_layer=True, operating=True)
    r.start()
    r.console_out.clear()
    r.tick(40)
    r.loop.scan_boundary(r.now, "lidar")
    r.slam.auto = slam_replies
    r.layer.worker.held = not layer_installs
    r.tick(20)
    assert r.permit.state == ra.LATCHED
    reply, _ = ask(r, "rearm")
    r.tick(5)
    assert reply["t"] == "ack", reply
    return r


def _arms_after_submit(r, ticks=400):
    """(armed, the refusals seen): the operator's start check and the submit as the console sends them, whatever their
    answers, then the ticks -- an ARM written is the only thing that counts as armed."""
    from forklift_core.mcu_link.messages import Arm

    from real_loop_rig import PALLET
    from test_real_loop_console import ask

    since = len(r.transport.sent)
    pid = next(iter(r.core.targets.pallets))
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=list(PALLET))
    rev = reply.get("targets_revision")
    r.tick(30)  # stands
    start, _ = ask(r, "operator_start")
    sub, _ = ask(r, "mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=rev)
    refusals = [m.get("reason") for m in (start, sub) if m.get("t") == "reject"]
    for _ in range(ticks):
        r.tick()
        if any(isinstance(m, Arm) for _, k, _, m in r.transport.sent[since:]):
            return True, refusals
    return False, refusals


@pytest.mark.parametrize("slam_replies, layer_installs, arms", [(False, True, False), (True, False, False),
                                                                (True, True, True)])
def test_a_rearm_after_a_boundary_needs_both_new_pieces_of_evidence(slam_replies, layer_installs, arms):
    r = _rearmed_after_a_boundary(slam_replies=slam_replies, layer_installs=layer_installs)
    armed, refusals = _arms_after_submit(r)
    assert armed is arms, (refusals, r.loop.last_refusal, r.loop.session.results[-1:])
    if not arms:
        # readiness names the missing piece itself: SLAM's freshness, or the layer's new result
        ready = r.loop._ready(r.now, "drive") or ""  # noqa: SLF001
        assert ready.startswith("slam:") if not slam_replies else ready.startswith(("layer:", "wait:")), (ready, refusals)
