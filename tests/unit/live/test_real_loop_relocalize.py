"""docs/design/2026-11-19-real-safety-wiring.md 6 (stage 5c-2) on the fake truck: the relocalization generation the loop
opens, the old evidence it drops, the epoch filter, the confirmation and the operator's declaration (one attempt
frame), the start held until the first move, the 0.06 m/s cap, and the physical stand the arbiter reads."""

import math

import pytest

from forklift_core.live import real_autonomy as ra
from forklift_core.live.real_loop import RealLoop
from forklift_core.localization.slam_pose import compose
from forklift_core.localization.start_reloc import base_from_rear, rear_from_base

from real_loop_rig import EPOCH, PALLET, REAR_OFFSET_M, Rig
from test_real_loop_console import ask, replies, run_until, start_and_submit


def rig(**kw):
    r = Rig(frames=True, real_requests=True, reloc=True, **kw)
    r.start()
    r.console_out.clear()
    return r


def open_generation(r, reason="test"):
    gen = r.core.require_relocalization(r.loop._t(r.now), reason)
    r.tick()
    assert r.loop.reloc_gen == gen and r.tracker.mode == "warming_up"
    return gen


def stand(r, n=10):
    r.tick(n)
    assert r.loop.stand_since_ns is not None


def status(r):
    return r.loop.real_status(r.now)["relocalize"]


def test_opening_drops_the_old_evidence_and_refuses_the_start_check():
    r = rig()
    stand(r)
    epoch0 = r.loop.epoch
    gen = open_generation(r)
    assert r.loop.epoch == epoch0 + 1 and ("relocalize", r.loop.epoch) in r.slam.calls
    assert r.loop.provenance is None or r.loop.provenance[0] == r.loop.epoch
    reply, _ = ask(r, "operator_start")
    assert reply["reason"] == "relocalize_required"
    assert r.core.relocalize_generation() == gen


def test_a_confirmation_closes_the_generation():
    r = rig()
    gen = open_generation(r)
    stand(r)
    reply, _ = ask(r, "relocalize_confirm", generation=gen)
    assert reply["t"] == "ack" and reply["attempt"] == 1 and reply["generation"] == gen
    assert ("request_processed", r.loop.epoch, 1) == r.slam.calls[-1][:3]
    run_until(r, lambda rec: status(r)["state"] != "waiting", 200)
    assert status(r)["state"] == "confirmed" and status(r)["start_pose_m"] == pytest.approx(0.0, abs=1e-9)
    assert r.core.relocalize_generation() is None and r.tracker.mode == "tracking"
    assert r.loop.real_status(r.now)["relocalize"]["generation"] == gen  # kept after the close


def test_refusals_of_an_attempt():
    r = rig()
    reply, _ = ask(r, "relocalize_confirm", generation=1)
    assert reply["reason"] == "no_relocalization"
    gen = open_generation(r)
    stand(r)
    reply, _ = ask(r, "relocalize_confirm", generation=gen + 1)
    assert reply["reason"] == "stale_relocalization"
    reply, _ = ask(r, "check_bypass", check="relocalize", on=True)
    assert reply["reason"] == "not_bypassable"


def test_the_epoch_filter():
    r = rig()
    gen = open_generation(r)
    t = r.loop._t(r.now)
    old = r.slam.reply(t, epoch=r.loop.epoch - 1)
    early = r.slam.reply(r.loop.t_epoch - 0.05)
    future = r.slam.reply(t + 1.0)
    r.slam.extra = [old, early, future]
    r.tick()
    d = r.loop.reloc_dropped
    assert d["epoch"] >= 1 and d["before_epoch"] >= 1 and d["future"] >= 1
    assert r.core.relocalize_generation() == gen


def test_off_the_start_refused_then_confirmed_with_the_start_pose_bypass():
    r = rig()
    gen = open_generation(r)
    r.slam.correction = (0.15, 0.0, 0.0)  # SLAM puts the truck 0.15 m off the taped start
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    assert status(r)["state"] == "refused" and status(r)["reason"] == "not_at_start"
    assert status(r)["start_pose_m"] == pytest.approx(0.15, abs=1e-6)
    reply, _ = ask(r, "check_bypass", check="start_pose", on=True)
    assert reply["t"] == "ack"
    r.tick()
    stand(r, 2)
    ask(r, "relocalize_confirm", generation=gen)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 200)
    assert status(r)["state"] == "confirmed"
    assert "start_pose" in r.loop.real_status(r.now)["bypassed"]


def test_a_declaration_seeds_the_start_and_marks_the_session():
    r = rig()
    gen = open_generation(r)
    r.slam.correction = (1.0, 0.5, 0.3)  # a SLAM estimate far from the tape
    stand(r)
    reply, _ = ask(r, "relocalize_declare", generation=gen)
    assert reply["t"] == "ack" and reply["attempt"] == 1
    run_until(r, lambda rec: status(r)["state"] != "waiting", 200)
    assert status(r)["state"] == "declared" and r.core.relocalize_generation() is None
    seed = next(c for c in r.slam.calls if c[0] == "seed")
    target = base_from_rear(tuple(r.loop.start_rear), REAR_OFFSET_M)
    assert seed[2] == pytest.approx(target)
    # the base_link pose the correction gives sits at the start's base_link pose; the rear axle at the start
    rear = rear_from_base(compose(tuple(r.tracker.applied[0]), r.odom_base()), REAR_OFFSET_M)
    assert rear == pytest.approx(tuple(r.loop.start_rear), abs=1e-9)
    st = r.loop.real_status(r.now)
    assert "relocalize" in st["bypassed"] and r.loop._speed_cap() == pytest.approx(0.06)
    assert any(e["event"] == "relocalize_seeded" for e in r.loop.reloc_events)


def test_a_declaration_followed_by_a_lease_release_seeds_nothing():
    r = rig()
    gen = open_generation(r)
    stand(r)
    r.say("relocalize_declare", generation=gen)
    r.say("lease_release")
    r.tick()
    r.tick()
    assert not any(c[0] == "seed" for c in r.slam.calls)
    assert status(r)["state"] == "refused" and status(r)["reason"] == "relocalize_voided:lease"


def test_a_new_attempt_supersedes_the_last():
    r = rig()
    gen = open_generation(r)
    r.slam.auto = False  # no reply comes: the confirmation waits
    r.slam.request_processed = lambda *a, **k: None
    stand(r)
    first, _ = ask(r, "relocalize_confirm", generation=gen)
    second, _ = ask(r, "relocalize_declare", generation=gen)
    assert second["attempt"] == first["attempt"] + 1
    assert any(e.get("reason") == "relocalize_superseded" and e["attempt"] == first["attempt"]
               for e in r.loop.reloc_events)


def test_an_old_epoch_reply_is_no_freshness_after_a_declaration():
    r = rig()
    gen = open_generation(r)
    stand(r)
    r.slam.auto = False  # the source answers nothing after the seed
    ask(r, "relocalize_declare", generation=gen)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 200)
    assert status(r)["state"] == "declared"
    r.slam.extra = [r.slam.reply(r.loop._t(r.now), epoch=r.loop.epoch - 1)]  # fresh, but of the old epoch
    r.tick(3)
    assert r.loop._fresh_ns is None
    reply, _ = ask(r, "operator_start")
    assert reply["reason"] == "slam_stale"


def test_no_emergency_while_closed_without_slam_and_the_arbiter_sees_the_stand():
    r = rig()
    gen = open_generation(r)
    stand(r)
    r.slam.auto = False
    ask(r, "relocalize_declare", generation=gen)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 200)
    r.tick(60)  # 1.2 s with no reply: the tracker's age passed, the outputs closed
    assert all(rec.branch != "emergency" for rec in r.loop.records[-60:])
    assert r.loop.physical_stop(r.loop._t(r.now) + 10.0)[1] is True or r.loop.stand_since_ns is not None
    assert r.permit.state == ra.OFF and r.arbiter.mode != "STOPPED"


def test_a_mission_off_the_start_ends_before_the_arm():
    r = rig()
    r.loop.start_rear = (0.3, 0.0, 0.0)  # the tape 0.3 m away from where the truck stands
    start_and_submit(r)
    run_until(r, lambda rec: r.core.commands.status in ("cancelled", "failed", "done"), 400)
    assert r.core.commands.end_reason == "not_at_start"
    assert r.permit.state == ra.OFF


def test_a_tracker_whose_age_differs_is_refused():
    from forklift_core.localization.slam_pose import SlamPoseTracker

    r = rig()
    r.core.slam["tracker"] = SlamPoseTracker(max_age_s=0.4, hold_limit_m=0.3)
    loop = r.loop
    with pytest.raises(ValueError, match="tracker_max_age"):
        RealLoop(core=r.core, session=loop.session, permit=r.permit, port=r.port, host=r.host, link=r.link,
                 channels=r.channels, transport=r.transport, params=loop.params, odometry=r.odo,
                 kinematics=loop.kin, max_speed_mps=0.15, start_ns=loop.start_ns, slam_source=loop.slam_source,
                 evidence=loop.evidence, base_scenario=loop.base_scenario, wait_pose=None)


def test_with_the_real_layer_the_confirmation_waits_for_a_scan_after_the_attempt():
    r = rig(real_layer=True)
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    t_try = r.loop.attempt["t"] if r.loop.attempt else None
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    assert status(r)["state"] == "confirmed", status(r)
    assert r.layer.installed_newest_s() >= t_try


def test_with_no_scan_after_the_attempt_the_layer_refuses():
    r = rig(real_layer=True)
    gen = open_generation(r)
    stand(r)
    r.scan_on = False  # the LiDAR stops: no scan after the cut
    ask(r, "relocalize_confirm", generation=gen)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    assert status(r)["state"] == "refused" and status(r)["reason"] in ("relocalize_layer", "relocalize_voided:layer")


def test_the_slam_bypass_caps_the_written_drives():
    r = rig()
    reply, _ = ask(r, "check_bypass", check="slam", on=True)
    assert reply["t"] == "ack"
    r.tick()
    asked = []
    orig = r.loop._auto_drive

    def spy(now_ns, command_, steer, rec):
        asked.append(abs(float(command_.drive.speed_mps)))
        return orig(now_ns, command_, steer, rec)

    r.loop._auto_drive = spy
    booked, written = {}, []
    orig_book, orig_written = r.loop._book, r.loop.book.written

    def book_spy(now_ns, seq, speed, *a, **k):
        booked[int(seq) & 0xFFFF] = float(speed)
        return orig_book(now_ns, seq, speed, *a, **k)

    def written_spy(seq, written_ns):
        written.append(booked.get(int(seq) & 0xFFFF))
        return orig_written(seq, written_ns)

    r.loop._book, r.loop.book.written = book_spy, written_spy
    start_and_submit(r)
    run_until(r, lambda rec: max(asked, default=0.0) > 0.06 and any(abs(v or 0.0) > 0 for v in written), 1500)
    r.tick(20)
    moving = [abs(v) for v in written if v]
    assert moving and max(moving) <= 0.06 + 1e-9  # every moving DRIVE written within the cap
    assert max(asked) > 0.06  # while the core asked for more


def test_a_skipped_reply_before_any_processed_is_filtered():
    r = rig()
    r.tracker.received = r.tracker.applied = None  # a tracker that never had a correction
    r.slam.auto = False
    r.slam.extra = [r.slam.reply(r.loop._t(r.now), status="skipped")]
    r.tick()
    assert r.loop.reloc_dropped["skipped_first"] == 1
    assert r.loop.records[-1].branch != "emergency"


def test_a_closed_hold_over_its_limit_opens_a_generation():
    r = rig()
    stand(r)
    x, y, yaw = r.odom_base()
    r.tracker.hold(odom_from_base=(x - ra.HOLD_MAX_M - 0.1, y, yaw))  # the next read counts the travel past the limit
    r.tick()
    assert all(rec.branch != "emergency" for rec in r.loop.records[-1:])
    assert r.core.relocalize_generation() is not None
    assert any(e.get("reason") == "hold_too_long" for e in r.core.state.get("field_recoveries", []))


def test_a_truck_pushed_off_the_start_after_the_arm_is_stopped_at_the_transmit_gate():
    r = rig()
    start_and_submit(r)
    run_until(r, lambda rec: r.permit.state in (ra.ARMING, ra.READY), 600)
    r.loop.start_rear = (0.3, 0.0, 0.0)  # as if the truck were pushed 0.3 m off while the ARM was answered
    rec = run_until(r, lambda rec: rec.branch == "emergency", 200)
    assert "not_at_start" in (rec.reason or "")
    assert r.loop._moved_for != r.core.commands.mission_id  # no moving DRIVE of this mission was written
