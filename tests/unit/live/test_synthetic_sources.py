"""PR 6a-2b (docs/design/2026-11-20-real-hil-6a.md 10.2): the synthetic sources -- refused off a verified HIL board,
silent while the check is false and across a boot change, the SLAM replies stamped with the scan they came from and
keeping a seeded correction, separate delivery stops for the layer and the SLAM side."""

from types import SimpleNamespace

import pytest

from forklift_core.live import synthetic_sources as ss
from forklift_core.live.synthetic import SyntheticRefused
from forklift_core.mcu_link.build_policy import Client

MS = 1_000_000
START = 10_000 * MS


def transport(ok=True, boot=7):
    return SimpleNamespace(hil_plant=ok, hil_boot=boot if ok else None)


def scan_source(tr, pose=(0.0, 0.0, 0.0), boxes=((3.0, 0.0, 0.3, 0.2),)):
    return ss.SyntheticScan(tr, Client.HIL, START, odom_pose=lambda: pose, boxes=boxes)


def test_sources_are_refused_off_a_verified_hil_board():
    with pytest.raises(SyntheticRefused):
        ss.SyntheticScan(transport(False), Client.HIL, START, odom_pose=lambda: (0, 0, 0), boxes=())
    with pytest.raises(SyntheticRefused):
        ss.SyntheticScan(transport(), Client.REAL, START, odom_pose=lambda: (0, 0, 0), boxes=())
    with pytest.raises(SyntheticRefused):
        ss.SyntheticImu(transport(False), Client.HIL)


def test_scans_come_every_100_ms_with_the_box_at_its_range():
    sc = scan_source(transport())
    assert sc.scans(START) == []
    out = sc.scans(START + 100 * MS)
    assert len(out) == 1
    scan = out[0]
    # an instantaneous scan (design 10.7): its first and last beam at the cast's instant
    assert scan.sensor == "lidar" and scan.last_s == pytest.approx(0.1) and scan.first_s == pytest.approx(0.1)
    front = abs(scan.angles_rad).argmin()
    assert scan.ranges_m[front] == pytest.approx(2.7, abs=0.02)  # the box's near face 3.0 - 0.3


def test_nothing_is_delivered_while_the_check_is_false_and_nothing_old_after():
    tr = transport()
    sc = scan_source(tr)
    sc.scans(START)
    tr.hil_plant, tr.hil_boot = False, None
    assert sc.scans(START + 200 * MS) == []
    tr.hil_plant, tr.hil_boot = True, 8  # a new boot
    out = sc.scans(START + 300 * MS)
    assert all(s.first_s >= 0.2 - 1e-9 for s in out)  # nothing acquired before or during the false check


def test_slam_replies_carry_the_scans_first_beam_and_a_seeded_correction():
    tr = transport()
    sc = scan_source(tr)
    base = {"pose": (0.0, 0.0, 0.0)}
    sl = ss.SyntheticSlam(tr, Client.HIL, sc, map_id="hil-1", odom_base=lambda: base["pose"])
    sc.scans(START)
    sc.scans(START + 100 * MS)
    rep = sl.replies(START + 100 * MS)
    assert rep and rep[0][0].stamp_s == pytest.approx(0.1) and rep[0][0].map_from_odom == (0.0, 0.0, 0.0)
    sl.seed(1, (1.0, 0.0, 0.0), 0.1)
    sc.scans(START + 200 * MS)
    rep = sl.replies(START + 200 * MS)
    assert rep[-1][0].epoch == 1 and rep[-1][0].map_from_odom[0] == pytest.approx(1.0)


def test_the_layer_and_the_slam_deliveries_stop_apart():
    tr = transport()
    sc = scan_source(tr)
    sl = ss.SyntheticSlam(tr, Client.HIL, sc, map_id="hil-1", odom_base=lambda: (0.0, 0.0, 0.0))
    sc.scans(START)
    sc.stop_delivery = True  # the layer's feed stops; the acquisitions go on
    assert sc.scans(START + 100 * MS) == []
    assert sl.replies(START + 100 * MS)
    sl.stop_delivery = True
    sc.stop_delivery = False
    assert sc.scans(START + 200 * MS)
    assert sl.replies(START + 200 * MS) == []


def test_the_imu_reads_gravity_every_5_ms_up_to_the_asked_instant():
    tr = transport()
    imu = ss.SyntheticImu(tr, Client.HIL)
    imu.samples(START, 64)
    out = imu.samples(START + 20 * MS, 64)
    assert len(out) == 4 and all(abs(a[2] - 9.81) < 0.2 for _, a in out)
    tr.hil_plant = False
    assert imu.samples(START + 40 * MS, 64) == []


def test_a_late_call_takes_one_scan_at_the_call_with_its_pose_and_mount():
    tr = transport()
    pose = {"p": (0.0, 0.0, 0.0)}
    sc = ss.SyntheticScan(tr, Client.HIL, START, odom_pose=lambda: pose["p"], boxes=((3.0, 0.0, 0.3, 0.2),),
                          mount=(0.5, 0.0, 0.0))
    sc.scans(START)
    pose["p"] = (1.0, 0.0, 0.0)
    out = sc.scans(START + 350 * MS)  # three periods late
    assert len(out) == 1 and out[0].last_s == pytest.approx(0.35)
    front = abs(out[0].angles_rad).argmin()
    assert out[0].ranges_m[front] == pytest.approx(3.0 - 0.3 - 1.0 - 0.5, abs=0.02)  # pose at the call, mount added


def test_a_beam_with_no_return_is_a_valid_inf():
    sc = scan_source(transport(), boxes=())
    sc.scans(START)
    scan = sc.scans(START + 100 * MS)[0]
    assert scan.valid.all() and (scan.ranges_m == float("inf")).all()


def test_a_rotated_box_and_no_acquisition_before_the_source_is_active():
    on = {"v": False}
    sc = ss.SyntheticScan(transport(), Client.HIL, START, odom_pose=lambda: (0.0, 0.0, 0.0),
                          boxes=((3.0, 0.0, 0.3, 0.2, 1.5707963267948966),), active=lambda: on["v"])
    sc.scans(START)
    assert sc.scans(START + 200 * MS) == []  # inactive: nothing acquired, the period not started
    on["v"] = True
    sc.scans(START + 300 * MS)
    out = sc.scans(START + 400 * MS)
    front = abs(out[0].angles_rad).argmin()
    assert out[0].ranges_m[front] == pytest.approx(3.0 - 0.2, abs=0.02)  # turned a quarter: the half width faces it
