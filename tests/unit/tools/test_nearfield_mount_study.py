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
