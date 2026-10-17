from console.gateway.store import CameraFrame, Store


def test_a_resent_capture_is_not_a_new_frame():
    s = Store()
    assert s.set_camera(CameraFrame(b"\xff\xd8a", 123, 1.0)) is True
    assert s.set_camera(CameraFrame(b"\xff\xd8a", 123, 1.5)) is False  # same stamp
    assert s.latest("camera_rgb")[0] == 1
    assert s.set_camera(CameraFrame(b"\xff\xd8b", 124, 2.0)) is True
    assert s.latest("camera_rgb")[0] == 2


def test_an_unstamped_image_always_counts():
    s = Store()
    assert s.set_camera(CameraFrame(b"\xff\xd8a", 0, 1.0))
    assert s.set_camera(CameraFrame(b"\xff\xd8a", 0, 1.1))  # no stamp: cannot tell


def test_snapshot_reports_notes_and_the_correction_count():
    s = Store()
    s.set_map_to_odom((0.0, 0.0, 0.0), 1.0)
    s.note("grid", "회전")
    snap = s.snapshot(now=3.0)
    assert snap.correction_version == 1 and snap.correction_age_s == 2.0
    assert snap.notes == ["회전"]
    s.note("grid", None)
    assert s.snapshot(now=3.0).notes == []


def test_the_correction_age_comes_from_its_own_time_not_from_when_it_was_seen():
    # a correction first seen at t=100 but already 50 s old must not read as fresh
    s = Store()
    s.set_map_to_odom((0.0, 0.0, 0.0), 100.0, received_s=50.0)
    snap = s.snapshot(now=100.0)
    assert snap.correction_age_s == 50.0
    assert snap.correction_version == 1
