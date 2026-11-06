"""docs/plans/2026-11-06-near-capture-plane-refit.md 5 (vii): the near detection's boundary call keeps its frozen
request when the arms are off, so a record from before replays; an armed call replays its own record and is refused
by an old one (the new keyword is compared, never dropped)."""

import pytest

from forklift_core.mission.boundary_record import BoundaryRecorder, RecordingGate, recorded_function
from forklift_core.mission.replay import ReplayGate, ReplayMismatch, ReplaySession


def _detect(depth, prior, params, site, **kw):
    return {"status": "valid", "kw": sorted(kw)}


def _call(gate, rec_or_session, kw):
    detect = recorded_function(_detect, "Perception", "detect_pockets_expected", gate)
    rec_or_session.tick_begin(0, tick_t=0.0, work_t=0.0, commands=[], stop_kind="none")
    out = detect([1.0, 2.0], "epal6", {"ransac_iterations": 200}, (2.0, 0.1, 0.0), **kw)
    rec_or_session.tick_end(outputs=out, branches=[])
    return out


def _record(path, kw):
    rec = BoundaryRecorder(path, chunk_ticks=2, value_format=2)
    _call(RecordingGate(rec), rec, kw)
    rec.close()


def _replay(path, kw):
    session = ReplaySession(path)
    out = _call(ReplayGate(session), session, kw)
    session.finish()
    return out


def test_the_unarmed_call_replays_a_record_from_before(tmp_path):
    _record(tmp_path, {})
    assert _replay(tmp_path, {}) == {"status": "valid", "kw": []}


def test_an_armed_call_replays_its_own_record(tmp_path):
    _record(tmp_path, {"plane_refit_iterations": 5})
    assert _replay(tmp_path, {"plane_refit_iterations": 5})["kw"] == ["plane_refit_iterations"]


def test_an_armed_call_is_refused_by_a_record_from_before(tmp_path):
    _record(tmp_path, {})
    with pytest.raises(ReplayMismatch, match="request differs"):
        _replay(tmp_path, {"plane_refit_iterations": 5})
