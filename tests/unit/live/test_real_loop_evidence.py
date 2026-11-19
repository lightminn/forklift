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


def test_a_depth_frame_gets_the_motion_and_lift_of_its_own_acquisition():
    """Codex 4c impl P1-15: a frame taken while turning and arriving after the stop is judged with that turn's yaw
    rate, speed and lift, not with the standing tick's."""
    import math
    from types import SimpleNamespace

    r = standing_rig()
    loop = r.loop
    t = loop._t(r.now)  # noqa: SLF001
    loop.pose_history.clear()
    Z = (0.0, 0.0, 0.0)  # the SLAM correction applied (none)
    loop.pose_history.extend([(t - 0.10, (0.0, 0.0, 0.0), True, 0.12, 0.02, Z),
                              (t - 0.08, (0.002, 0.0, 0.02), True, 0.12, 0.02, Z),
                              (t - 0.06, (0.004, 0.0, 0.04), True, 0.10, 0.03, Z),
                              (t - 0.04, (0.004, 0.0, 0.04), True, 0.0, 0.05, Z)])
    rear, speed, yaw_rate, lift = loop._pose_at(t - 0.09, Z)  # noqa: SLF001
    assert loop._pose_at(t - 0.09, (0.2, 0.0, 0.0)) is None  # noqa: SLF001 -- another correction's poses
    assert math.isclose(rear[2], 0.01, abs_tol=1e-9) and speed == 0.12 and lift == 0.02
    assert math.isclose(yaw_rate, 1.0, rel_tol=1e-6)  # 0.02 rad over 20 ms
    seen = []
    check = SimpleNamespace(last_new_s=-1.0, config=SimpleNamespace(frame_lag_s=0.0, frame_max_age_s=0.5),
                            add_frame=lambda *a, **k: seen.append(k) or {"new": True})
    r.core.pocket["check"] = check
    phase, gen = r.core.ms.phase, r.core.commands.mission_gen
    loop.depth_source = SimpleNamespace(frames=lambda n: [SimpleNamespace(stamp_s=t - 0.09, depth=None,
                                                                         mission_gen=gen, phase=phase,
                                                                         correction=(0.0, 0.0, 0.0))])
    loop._feed_depth(r.now, t)  # noqa: SLF001
    assert seen and seen[0]["lift_m"] == 0.02 and seen[0]["speed_mps"] == 0.12 and seen[0]["yaw_rate_rps"] > 0.9
