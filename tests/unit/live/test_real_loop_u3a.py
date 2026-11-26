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
