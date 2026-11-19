"""Codex 4c impl round 1 (P1-6, P1-14): the stand and the odometry the loop reports -- no stand from samples the
link stopped delivering, an encoder cut or an MCU reboot breaks the odometry, a reboot asks for a relocalization."""

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import FaultFlag

from real_loop_rig import Rig


def standing_rig():
    r = Rig(frames=True)
    r.start()
    r.tick(30)
    assert r.core.slam["stop_now"]
    return r


def test_a_stand_is_not_kept_from_samples_the_link_stopped_delivering():
    r = standing_rig()
    r.transport.silent = True
    r.tick(15)  # 300 ms without a STATE, SLAM still answering
    assert not r.core.slam["stop_now"] and r.loop.still_since_ns is None


def test_a_stand_needs_samples_measured_after_the_settlement():
    r = standing_rig()
    r.loop.samples.clear()  # only samples older than any settlement would be left otherwise
    r.tick(1)
    assert not r.core.slam["stop_now"]  # one new sample does not cover the window


def test_an_encoder_cut_breaks_the_odometry_for_its_tick():
    r = standing_rig()
    r.mcu.faults = FaultFlag.ENCODER_GAP
    r.tick(1)
    assert not r.loop.odom_continuous and r.loop.pose_history[-1][2] is False
    assert not r.core.slam["stop_now"] and r.loop.still_since_ns is None
    r.mcu.faults = FaultFlag(0)
    r.tick(1)
    assert r.loop.odom_continuous  # the next tick without the bit re-references


def test_an_encoder_cut_while_driving_is_the_emergency_branch():
    r = Rig(frames=True)
    r.start()
    r.submit()
    for _ in range(4000):
        if r.tick().permit == ra.DRIVING:
            break
    r.mcu.faults = FaultFlag.ENCODER_GAP
    rec = r.tick()
    assert (rec.branch, rec.reason) == ("emergency", "odometry_break")
    assert r.permit.state in (ra.DISARMING, ra.LATCHED)


def test_an_mcu_reboot_asks_for_a_relocalization_and_drops_the_old_samples():
    r = standing_rig()
    r.mcu.boot = 0x6262
    r.tick(1)
    assert r.core.cargo.relocalize_required
    assert all(x.state.boot_id == 0x6262 for x in r.loop.samples)
    assert not r.core.slam["stop_now"]
