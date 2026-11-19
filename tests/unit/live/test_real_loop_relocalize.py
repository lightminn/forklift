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
    kw.setdefault("real_layer", True)  # no generation closes without the layer (impl round 1 P2-3)
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
    r = rig()
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    t_try = r.loop.attempt["t"] if r.loop.attempt else None
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    assert status(r)["state"] == "confirmed", status(r)
    assert r.layer.installed_newest_s() >= t_try


def test_with_no_scan_after_the_attempt_the_layer_refuses():
    r = rig()
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
    mid = r.core.commands.mission_id
    r.loop.start_rear = (0.3, 0.0, 0.0)  # as if the truck were pushed 0.3 m off while the ARM was answered
    rec = run_until(r, lambda rec: rec.branch == "emergency", 200)
    assert "not_at_start" in (rec.reason or "")
    assert mid is not None and r.loop._moved_for != mid  # no moving DRIVE of this mission was written


# --- impl round 1 ---------------------------------------------------------------------------------------------------
def test_a_disconnect_queued_before_the_close_voids_the_attempt():
    r = rig()
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    orig = r.loop._commit_void

    def late(t, closing=True):
        r.channels.disconnect("c1", "op")  # queued after this tick's batch, before the commit
        r.loop._commit_void = orig
        return orig(t, closing)

    r.loop._commit_void = late
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    assert status(r)["state"] == "refused"
    assert status(r)["reason"] in ("relocalize_voided:lease", "relocalize_voided:emergency")
    assert r.core.relocalize_generation() == gen


def test_a_disconnect_queued_before_the_seed_leaves_no_seed():
    r = rig()
    gen = open_generation(r)
    stand(r)
    orig = r.loop._commit_void

    def late(t, closing=True):
        r.channels.disconnect("c1", "op")
        r.loop._commit_void = orig
        return orig(t, closing)

    r.loop._commit_void = late  # the seed is checked in the same tick the declaration is taken
    r.say("relocalize_declare", generation=gen)
    r.tick(2)
    assert not any(c[0] == "seed" for c in r.slam.calls) and not r.loop._declared()
    assert status(r)["state"] == "refused" and status(r)["reason"] == "relocalize_voided:lease"
    assert all("TypeError" not in (rec.reason or "") for rec in r.loop.records[-3:])


def test_the_mark_holds_when_the_synthetic_reply_fails_after_the_seed():
    r = rig()
    gen = open_generation(r)
    stand(r)
    orig = r.core.slam_replies

    def fail_once(t, replies, loaded):
        if replies and replies[0][1].get("source") == "operator_declared":
            r.core.slam_replies = orig
            raise RuntimeError("injected")
        return orig(t, replies, loaded)

    r.core.slam_replies = fail_once
    ask(r, "relocalize_declare", generation=gen)
    r.tick(2)
    assert any(c[0] == "seed" for c in r.slam.calls)
    assert r.loop._declared() and "relocalize" in r.loop.real_status(r.now)["bypassed"]
    assert r.loop._speed_cap() == pytest.approx(0.06)


def test_a_reference_after_the_limit_is_refused():
    r = rig()
    gen = open_generation(r)
    stand(r)
    held = []
    r.slam.request_processed = lambda epoch, attempt, not_before: held.append((epoch, attempt, not_before))
    ask(r, "relocalize_confirm", generation=gen)
    t0 = r.loop.attempt["t"]
    while r.loop._t(r.now) - t0 <= ra.SLAM_MAX_AGE_S * 0 + 3.0 - 0.02:
        r.tick()
    r.slam.requests.extend(held)  # the reference comes only now, past the 3 s limit
    run_until(r, lambda rec: status(r)["state"] != "waiting", 50)
    assert status(r)["state"] == "refused"


def test_processed_then_skipped_in_one_batch():
    r = rig()
    r.slam.auto = False
    t = r.loop._t(r.now)
    r.slam.extra = [r.slam.reply(t + 0.005, correction=(0.01, 0.0, 0.0)), r.slam.reply(t + 0.01, status="skipped")]
    r.tick()
    first = r.loop.provenance
    assert first is not None and first[3] == pytest.approx(t + 0.005)  # the processed, not the later skipped
    r.tracker.received = r.tracker.applied = None
    t = r.loop._t(r.now)
    r.slam.extra = [r.slam.reply(t + 0.005), r.slam.reply(t + 0.01, status="skipped")]
    r.tick()
    assert r.loop.reloc_dropped["skipped_first"] == 0  # the skipped after a processed of the same batch passes


def test_no_layer_no_close():
    r = rig(real_layer=False)
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    assert status(r)["state"] == "refused" and status(r)["reason"] == "relocalize_layer"
    assert r.core.relocalize_generation() == gen


def test_the_open_generation_without_replies_stays_warming_up_without_an_emergency():
    r = rig()
    gen = open_generation(r)
    r.slam.auto = False
    r.slam.request_processed = lambda *a, **k: None  # no answer to the confirmation either
    stand(r)
    reply, _ = ask(r, "relocalize_confirm", generation=gen)
    assert reply["t"] == "ack"
    r.tick(200)  # 4 s of silence, past the attempt's limit
    assert r.tracker.mode == "warming_up" and r.core.relocalize_generation() == gen
    assert status(r)["state"] == "refused" and status(r)["reason"] == "relocalize_no_reply"
    assert all(rec.branch != "emergency" for rec in r.loop.records[-200:])


def test_a_reboot_before_the_first_correction():
    r = rig()
    r.tracker.received = r.tracker.applied = None
    r.tracker.mode = "warming_up"
    r.slam.auto = False
    r.mcu.boot = 0x6262  # the MCU reboots
    r.tick(20)
    gen = r.core.relocalize_generation()
    assert gen is not None
    stand(r, 40)
    reply, _ = ask(r, "relocalize_confirm", generation=gen)  # only the confirmation's own reply comes
    run_until(r, lambda rec: status(r)["state"] != "waiting", 300)
    assert reply["t"] == "ack" and status(r)["state"] == "confirmed"
    assert not any(rec.branch == "emergency" for rec in r.loop.records[-300:])  # never the no-correction exception


def test_a_long_slam_silence_does_not_stop_the_arbiter_or_refuse_teleop():
    r = rig()
    gen = open_generation(r)
    stand(r)
    r.slam.auto = False
    r.tick(int(10.5 / 0.02))  # past the arbiter's not_stopped_s with no SLAM reply
    assert r.arbiter.mode != "STOPPED"
    reply, _ = ask(r, "mode_request", to="TELEOP")
    assert reply.get("reason") != "not_stopped", reply
    r.say("stop")
    r.tick(3)
    assert r.arbiter.mode == "STOPPED"
    stand(r, 5)
    reply, _ = ask(r, "rearm")
    r.tick(3)
    assert reply["t"] == "ack" and r.arbiter.mode == "PAUSED", reply  # the rearm reads the physical stand


def test_a_push_off_the_start_during_the_arm_is_stopped_before_it_reaches_the_gate():
    """A physical push: ARM times out after 0.3 s, so 0.10 m off the start inside it needs > 0.33 m/s -- the overspeed
    watch (0.225 m/s) or the ARM timeout latches first; either way no moving DRIVE of the mission is written."""
    import math as _m

    r = rig()
    start_and_submit(r)
    run_until(r, lambda rec: r.permit.state == ra.ARMING, 800)
    mid = r.core.commands.mission_id
    cpm = 327680.0 / (2 * _m.pi * 0.125)
    for _ in range(20):
        if r.permit.state != ra.ARMING:
            break
        r.mcu.counts[0] += 0.007 * cpm  # 0.35 m/s
        r.mcu.counts[1] += 0.007 * cpm
        r.tick()
    r.tick(10)
    assert mid is not None and r.permit.state == ra.LATCHED and r.loop._moved_for != mid


def test_a_half_written_first_drive_continued_off_the_start_is_link_stuck():
    r = rig()
    orig = r.loop._book

    def book(now_ns, seq, speed, *a, **k):
        if speed != 0.0 and k.get("steer") is not None and not getattr(r, "_stalled", False):
            r._stalled = True
            r.transport.stall = True  # the first moving DRIVE gets only its first bytes out
        return orig(now_ns, seq, speed, *a, **k)

    r.loop._book = book
    start_and_submit(r)
    run_until(r, lambda rec: r.transport.partial_frame is not None and getattr(r, "_stalled", False), 800)
    mid = r.core.commands.mission_id
    r.loop.start_rear = (0.3, 0.0, 0.0)  # off the start before the rest of the frame
    r.transport.stall = False
    rec = run_until(r, lambda rec: rec.branch == "emergency", 50)
    assert "link_stuck" in (rec.reason or "") and r.transport.resyncs == 1, rec  # the half frame abandoned
    assert mid is not None and r.loop._moved_for != mid


def test_the_cap_follows_the_slam_bypass_and_the_declaration():
    r = rig()
    ask(r, "check_bypass", check="slam", on=True)
    r.tick()
    assert r.loop._speed_cap() == pytest.approx(0.06)
    ask(r, "check_bypass", check="slam", on=False)
    r.tick()
    assert r.loop._speed_cap() == pytest.approx(0.15)  # no declaration in this session: lifted
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_declare", generation=gen)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 200)
    ask(r, "check_bypass", check="slam", on=True)
    r.tick()
    ask(r, "check_bypass", check="slam", on=False)
    r.tick()
    assert r.loop._speed_cap() == pytest.approx(0.06)  # the declaration's mark keeps it


def test_skipped_only_with_both_bypasses_never_arms():
    r = rig()
    for check in ("slam", "operator_start"):
        ask(r, "check_bypass", check=check, on=True)
        r.tick()
    r.slam.status = "skipped"  # the source keeps answering, never with a new pose
    r.tick(40)
    since = len(r.transport.sent)
    pid = next(iter(r.core.targets.pallets))
    reply, _ = ask(r, "pallet_register", pallet_id=pid, pose=list(PALLET))
    rid = r.say("mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=reply["targets_revision"])
    modes = []
    for _ in range(300):
        r.tick()
        modes.append(r.arbiter.mode)
    got, _ = replies(r)
    assert any(m.get("request_id") == rid and m["t"] == "ack" for m in got)  # the submit taken
    assert "AUTO" in modes and r.jobs.submitted  # AUTO entered and the plan asked for
    assert [e["reason"] for e in r.loop.session.results] == ["prepare:moving"]  # the adoption's fresh stand refused
    from forklift_core.mcu_link.messages import Arm

    assert not any(isinstance(m, Arm) for _, k, _, m in r.transport.sent[since:])
    assert not any(d.left_wheel_rad_s or d.right_wheel_rad_s for d in r.loop_drives(since))


def test_a_complete_source_loss_while_driving_latches():
    r = rig()
    ask(r, "check_bypass", check="slam", on=True)
    r.tick()
    start_and_submit(r)
    run_until(r, lambda rec: r.permit.state == ra.DRIVING, 800)
    r.slam.auto = False  # nothing at all from the source
    rec = run_until(r, lambda rec: rec.branch == "emergency", 100)
    assert rec.branch == "emergency"


def test_with_the_slam_bypass_skipped_replies_keep_it_driving_capped_until_the_pocket_stop():
    """Design 6, 2b (the start/continue boundary): after the arm, processed replies stop and skipped ones go on -- the
    truck keeps driving within 0.06 m/s on the last correction and odometry, stops before the pocket (that stop needs
    a fresh SLAM stand) and that stage's timeout ends the mission."""
    r = rig()
    ask(r, "check_bypass", check="slam", on=True)
    r.tick()
    booked, written = {}, []
    ob, ow = r.loop._book, r.loop.book.written

    def book(now_ns, seq, speed, *a, **k):
        booked[int(seq) & 0xFFFF] = float(speed)
        return ob(now_ns, seq, speed, *a, **k)

    def wrote(seq, written_ns):
        written.append((r.loop._t(r.now), booked.get(int(seq) & 0xFFFF)))
        return ow(seq, written_ns)

    r.loop._book, r.loop.book.written = book, wrote
    asked = []
    od = r.loop._auto_drive

    def spy(now_ns, command_, steer, rec_):
        asked.append((r.loop._t(now_ns), abs(float(command_.drive.speed_mps))))
        return od(now_ns, command_, steer, rec_)

    r.loop._auto_drive = spy
    start_and_submit(r)
    run_until(r, lambda rec: any(v for _, v in written), 1500)
    t_cut = r.loop._t(r.now)
    r.slam.status = "skipped"
    run_until(r, lambda rec: r.core.commands.status in ("cancelled", "failed", "done"), 2000)
    after = [abs(v) for t, v in written if t > t_cut + ra.SLAM_MAX_AGE_S and v]
    assert after and max(after) <= 0.06 + 1e-9  # driving on after the last processed expired, capped
    assert max(v for t, v in asked if t > t_cut + ra.SLAM_MAX_AGE_S) > 0.06  # while more was asked then
    assert "pocket_stop" in (r.core.commands.end_reason or "")


# --- impl round 2 ---------------------------------------------------------------------------------------------------
def _before_commit(r, action):
    orig = r.loop._commit_void

    def late(t, closing=True):
        action()
        r.loop._commit_void = orig
        return orig(t, closing)

    r.loop._commit_void = late


@pytest.mark.parametrize("kind", ["stop", "disarm"])
def test_a_stop_or_disarm_queued_before_the_close_voids_the_attempt(kind):
    r = rig()
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    _before_commit(r, lambda: r.say(kind))
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    assert status(r)["state"] == "refused"  # held while waiting, voided once taken
    assert status(r)["reason"] in ("relocalize_voided:emergency", "relocalize_voided:control")
    assert r.core.relocalize_generation() == gen


def test_a_disarm_queued_before_the_seed_leaves_no_seed():
    r = rig()
    gen = open_generation(r)
    stand(r)
    _before_commit(r, lambda: r.say("disarm"))
    r.say("relocalize_declare", generation=gen)
    r.tick(2)
    assert not any(c[0] == "seed" for c in r.slam.calls) and not r.loop._declared()


def test_a_disarm_taken_voids_a_waiting_attempt():
    r = rig()
    gen = open_generation(r)
    stand(r)
    r.slam.request_processed = lambda *a, **k: None  # the confirmation keeps waiting
    ask(r, "relocalize_confirm", generation=gen)
    r.say("disarm")
    r.tick(3)
    assert status(r)["state"] == "refused"
    assert status(r)["reason"] in ("relocalize_voided:emergency", "relocalize_voided:control")


def test_freshness_is_judged_on_the_clock_at_the_close():
    r = rig()
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    r.slam.auto = False  # only the reference reply comes

    def later():
        r.tick_cost_ns = 600 * 1_000_000  # the close runs 0.6 s after the tick began: the correction is too old

    _before_commit(r, later)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    r.tick_cost_ns = 0
    assert status(r)["state"] == "refused" and status(r)["reason"] in (
        "relocalize_stale", "relocalize_voided:lease", "relocalize_voided:link")
    assert r.core.relocalize_generation() == gen


def test_a_layer_worker_dying_before_the_close_voids_the_attempt():
    r = rig()
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    _before_commit(r, lambda: setattr(r.layer.worker, "alive", False))
    run_until(r, lambda rec: status(r)["state"] != "waiting" or rec.branch == "emergency", 400)
    assert r.core.relocalize_generation() == gen


def test_a_push_with_the_start_fixed_is_stopped_at_the_transmit_gate():
    """The start target fixed 0.095 m behind the truck; the ARM answered 150 ms late; both encoders advance 1.5 mm
    every 20 ms meanwhile -- the first moving DRIVE meets the gate past 0.10 m."""
    import math as _m

    r = rig()
    r.loop.start_rear = (-0.095, 0.0, 0.0)
    r.mcu.arm_delay = 15
    start_and_submit(r)
    run_until(r, lambda rec: r.permit.state == ra.ARMING, 800)
    mid = r.core.commands.mission_id
    cpm = 327680.0 / (2 * _m.pi * 0.125)
    rec = None
    for _ in range(60):
        r.mcu.counts[0] += 0.0015 * cpm
        r.mcu.counts[1] += 0.0015 * cpm
        rec = r.tick()
        if rec.branch == "emergency":
            break
    assert rec.branch == "emergency" and "not_at_start" in (rec.reason or ""), rec
    assert r.loop._moved_for != mid



# --- impl round 3 ---------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("kind", ["stop", "disarm"])
def test_a_stop_or_disarm_queued_between_the_batch_and_the_acceptance_voids_the_declaration(kind):
    r = rig()
    gen = open_generation(r)
    stand(r)
    orig = r.loop._attempt_request

    def first(kind_, msg, now_ns):
        r.say(kind)  # queued after this tick's emergency batch was taken, before the attempt is accepted
        r.loop._attempt_request = orig
        return orig(kind_, msg, now_ns)

    r.loop._attempt_request = first
    rid = r.say("relocalize_declare", generation=gen)
    r.tick()
    got, _ = replies(r)
    r.tick(3)
    assert not any(c[0] == "seed" for c in r.slam.calls)
    mine = [m for m in got if m.get("request_id") == rid]
    assert mine and mine[0]["t"] == "reject" and mine[0]["reason"] == "emergency_pending"


def test_a_viewers_connection_does_not_void_the_attempt():
    r = rig()
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    r.channels.connect("v1", "viewer", control=False)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    assert status(r)["state"] == "confirmed"


def test_the_layer_watchdog_is_read_at_the_close():
    r = rig()
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    orig = r.layer.watchdog
    seen = []

    def watchdog(now_s):
        seen.append(now_s)
        return "layer_latency_exceeded" if now_s > r.loop._t(r.now) else orig(now_s)

    r.layer.watchdog = watchdog

    def later():
        r.tick_cost_ns = 20 * 1_000_000

    _before_commit(r, later)
    run_until(r, lambda rec: status(r)["state"] != "waiting" or rec.branch == "emergency", 400)
    r.tick_cost_ns = 0
    assert r.core.relocalize_generation() == gen  # the close's own instant saw the fault



# --- impl round 4 ---------------------------------------------------------------------------------------------------
def test_a_disarm_in_the_same_batch_refuses_the_attempt():
    r = rig()
    gen = open_generation(r)
    stand(r)
    rid = r.say("relocalize_declare", generation=gen)
    r.say("disarm")  # the emergency batch takes it first; the request comes after it in the same tick
    r.tick()
    got, _ = replies(r)
    r.tick(3)
    mine = [m for m in got if m.get("request_id") == rid]
    assert mine and mine[0]["t"] == "reject" and mine[0]["reason"] == "emergency_pending"
    assert not any(c[0] == "seed" for c in r.slam.calls)


def test_a_disarm_the_arbiter_refuses_holds_the_close_but_does_not_void_it():
    r = rig()
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)
    held = {}

    def inject():
        held["rid"] = r.say("disarm", lease_id="old-lease")  # the arbiter will refuse it (stale lease)
        held["tick"] = len(r.loop.records)

    _before_commit(r, inject)
    run_until(r, lambda rec: "rid" in held, 400)
    assert status(r)["state"] == "waiting" and r.core.relocalize_generation() == gen  # held while it waits
    got = []
    for _ in range(50):
        r.tick()
        g, _ = replies(r)
        got.extend(g)
        if status(r)["state"] != "waiting":
            break
    mine = [m for m in got if m.get("request_id") == held["rid"]]
    assert mine and mine[0]["t"] == "reject"  # judged (refused) by the arbiter ...
    assert status(r)["state"] == "confirmed" and r.loop._disarms_taken == 0  # ... and only then the close


def test_a_stale_link_at_the_close_voids_the_attempt():
    r = rig()
    gen = open_generation(r)
    stand(r)
    ask(r, "relocalize_confirm", generation=gen)

    def later():
        r.tick_cost_ns = 60 * 1_000_000  # the close runs 60 ms after the tick began: the last STATE past 50 ms

    _before_commit(r, later)
    run_until(r, lambda rec: status(r)["state"] != "waiting", 400)
    r.tick_cost_ns = 0
    assert status(r)["state"] == "refused" and status(r)["reason"] == "relocalize_voided:link"
    assert r.core.relocalize_generation() == gen
