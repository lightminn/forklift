"""Near-field mount study bookkeeping (docs/plans/2026-10-03-near-field-mount-study.md)."""

from tools import nearfield_mount_study as study


def test_gaps_count_the_longest_and_the_terminal_run():
    assert study.gaps([True, False, False, True, True]) == (0.02, 0.0)
    assert study.gaps([True, True, False]) == (0.01, 0.01)
    assert study.gaps([False, False, False]) == (0.03, 0.03)


def test_the_prior_offsets_are_the_same_for_every_mount_and_bounded():
    for seed in range(study.SEEDS):
        dx, dy, dyaw = study.prior_offsets(seed)
        assert study.prior_offsets(seed) == (dx, dy, dyaw)
        assert abs(dx) <= study.PRIOR_XY_M and abs(dy) <= study.PRIOR_XY_M
        assert abs(dyaw) <= study.PRIOR_YAW


def test_the_camera_k_is_isaacs_integer_index_k():
    k = study.isaac_intrinsics()
    assert (k.fx, k.fy, k.cx, k.cy) == (465.741156, 465.741156, 319.5, 239.5)


def test_the_d8a_reruns_carry_their_conditions():
    # Plan D8a: the regression reproduces the 2026-10-03 conditions explicitly; the
    # current rule and the real depth clip are separate runs of the same grid.
    from tools import nearfield_mount_study as study

    regression = study.jobs(("measured",), near_only=True, min_range_m=0.175, reserve_m=0.046)
    assert len(regression) == len(study.HEIGHTS) * len(study.TILTS) == 15
    assert all(j[0] == "near" and j[1] == "measured" and j[4:] == (0.175, 0.046) for j in regression)
    assert ("near", "measured", 0.27, 0.10, 0.28, 0.016) in study.jobs(
        ("measured",), near_only=True, min_range_m=0.28, reserve_m=0.016)
    # The default grid still holds the far regression jobs.
    assert any(j[0] == "far" for j in study.jobs())


def test_the_smoke_run_keeps_the_conditions_asked_for():
    # Codex D8a implementation review P2-1: --smoke used to replace them with provisional defaults.
    import argparse

    from tools import nearfield_mount_study as study

    args = argparse.Namespace(smoke=True, chassis=["measured"], near_only=True, min_range_m=0.28, reserve_m=0.046)
    assert study.planned_jobs(args) == [("near", "measured", 0.27, 0.10, 0.28, 0.046)]
    args.near_only = False
    assert study.planned_jobs(args)[1] == ("far", "measured", study.CURRENT, 0.28)


def test_the_run_record_hashes_every_core_source():
    # Codex D8a P2-2: the detector calls into sensors/ and geometry modules too.
    import argparse

    from tools import nearfield_mount_study as study

    meta = study.run_meta(argparse.Namespace(chassis=["measured"], near_only=True, min_range_m=0.175,
                                             reserve_m=0.046, smoke=False))
    assert "src/forklift_core/sensors/rgbd.py" in meta["sources_sha256"]
    assert "src/forklift_core/perception/pocket_detector.py" in meta["sources_sha256"]
    assert meta["numpy"] and meta["python"] and "blas" in meta
