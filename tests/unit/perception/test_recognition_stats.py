# tests/unit/perception/test_recognition_stats.py
import pytest

from forklift_core.perception.recognition_stats import allowed_failures, cp_upper, eval_count, min_scenes


@pytest.mark.parametrize(("bound", "k", "n"), [(0.05, 0, 59), (0.05, 1, 93), (0.05, 2, 124), (0.10, 0, 29),
                                               (0.10, 1, 46), (0.10, 2, 61), (0.15, 0, 19), (0.15, 1, 30),
                                               (0.20, 0, 14), (0.20, 1, 22)])
def test_min_scenes_matches_the_design_table(bound, k, n):
    assert min_scenes(k, bound) == n


def test_known_values():
    assert cp_upper(0, 59) == pytest.approx(0.0495, abs=1e-4)
    assert cp_upper(1, 114) == pytest.approx(0.04094, abs=1e-4)
    assert cp_upper(5, 100) == pytest.approx(0.10225, abs=1e-4)
    assert cp_upper(1, 20) == pytest.approx(0.21611, abs=1e-4)


def test_allowed_failures():
    assert allowed_failures(114, 0.05) == 1 and allowed_failures(114, 0.10) == 5
    assert allowed_failures(0, 0.05) is None and allowed_failures(10, 0.05) is None


def test_empty_population_has_no_bound_and_eval_split_convention():
    assert cp_upper(0, 0) is None
    assert [eval_count(n) for n in (44, 61, 94, 194)] == [14, 19, 29, 59]
