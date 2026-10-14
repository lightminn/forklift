import bisect

import numpy as np
import pytest

from tools import odometry_coverage as OC

TICK = 1.0 / 120.0


class Table:
    """NearFieldBounds.odometry's lookup over a small table."""

    def __init__(self, ages, e, psi):
        self.ages, self.e, self.psi = list(ages), list(e), list(psi)

    def odometry(self, age):
        if age == 0:
            return 0.0, 0.0
        i = bisect.bisect_left(self.ages, age - 1e-9)
        return None if i >= len(self.ages) else (self.e[i], self.psi[i])


def test_pairs_are_read_by_the_runtime_lookup_and_ages_beyond_count_whatever_their_error():
    table = Table([0.5, 0.6, 0.7], [0.001, 0.002, 0.005], [0.001, 0.002, 0.005])
    # 0.600000004 s reads the 0.7 s row at runtime (1 ns tolerance), so 3 mrad is inside.
    r = OC.check_pairs(np.array([0.600000004]), np.array([0.0]), np.array([0.003]), table)
    assert r["psi_excess_rad"] < 0 and r["ages_beyond_table"] == 0
    # A pair past the last age is beyond the table even with zero error.
    r = OC.check_pairs(np.array([0.5, 0.75]), np.zeros(2), np.zeros(2), table)
    assert r["ages_beyond_table"] == 1 and r["pairs"] == 2
    over = OC.check_pairs(np.array([0.55]), np.array([0.0025]), np.array([0.0]), table)
    assert over["e_excess_m"] == pytest.approx(0.0005) and over["e_age_s"] == pytest.approx(0.55)


class _Truth:
    phases = ["other", "observe", "approach", "insert"]

    def __init__(self, phase):
        self.phase = np.array([self.phases.index(p) for p in phase])


class _Run:
    def __init__(self, phase, near):
        self.truth = _Truth(phase)
        self.result = {"near_tracking": near}


def test_a_run_counts_as_started_by_any_record_of_an_episode():
    assert not OC.started(_Run(["observe"] * 5, {}))
    assert OC.started(_Run(["observe", "approach"], {}))  # approach ticks without a start
    assert OC.started(_Run(["observe"] * 3, {"frames_fed": 12}))
    assert OC.started(_Run(["observe"] * 3, {"starts": [{"episode": 2}]}))


def test_the_gate_reads_the_table_with_the_runtime_validation():
    source = OC.Path(OC.__file__).read_text()
    assert "NearFieldBounds.load(args.bounds)" in source
    assert 'raise ValueError("a near-field episode is recorded without its tracker start")' in source


def test_near_field_records_in_a_run_not_marked_as_near_tracking_are_unverifiable():
    source = OC.Path(OC.__file__).read_text()
    marked = source.index('if not (run.result.get("arguments") or {}).get("near_tracking"):')
    assert source.index("if started(run):", marked) < source.index("continue  # a run without", marked)
    assert 'raise ValueError("near-field records in a run not marked --near-tracking")' in source
