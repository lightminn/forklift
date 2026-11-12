"""tools/teleop_realtime_report.py on a synthetic session: per-mode ratio and pacing, a hold whose output waits for
pacing, the lift change, timers per call, and the overlap rule."""

import json
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import teleop_realtime_report as tr  # noqa: E402

DT = 1 / 120


def make_session(root: Path, name: str, *, other_cores: float = 0.0) -> Path:
    """AUTO for 2 s at ratio 1.0, then TELEOP for 2 s at ratio 0.9 with pacing not ok for its first 0.5 s and a
    driving hold from 0.2 s that outputs once pacing is ok, then a lift hold."""
    sd = root / name / "s"
    run = sd / "live" / "epoch_1" / "run"
    run.mkdir(parents=True)
    recs, mono, t = [], 100.0, 0.0
    for _ in range(240):
        recs.append({"tick_t": t, "mode": "AUTO", "pacing_ok": True, "mono": mono, "speed_mps": 0.3,
                     "armed": False, "deadline": None})
        t += DT
        mono += DT
    tele0 = mono
    for k in range(216):
        ok = mono - tele0 >= 0.5
        drive = ok and 0.2 <= mono - tele0 < 1.2
        recs.append({"tick_t": t, "mode": "TELEOP", "pacing_ok": ok, "mono": mono, "speed_mps": 0.1 if drive else 0.0,
                     "armed": True, "deadline": None})
        if k % 108 == 107:
            recs.append({"status": True, "tick_t": t, "mono": mono,
                         "timers": {"teleop.permit": [0.06, 60, 0.01, 0.0]}})
        t += DT
        mono += DT / 0.9
    recs.append({"status": True, "tick_t": recs[200]["tick_t"], "mono": recs[200]["mono"] + 1e-6,
                 "timers": {"core.permission": [0.05, 100, 0.002, 0.0]}})
    with open(run / "live_log.jsonl", "w") as fh:
        for r in sorted(recs, key=lambda r: r["mono"]):
            fh.write(json.dumps(r) + "\n")
    probe = {"states": [{"mono": tele0 + 0.3, "teleop_forbidden": "pacing"}],
             "confirm": {"holds": [
                 {"label": "forward", "mode": "direct", "lift_dir": 0, "start_mono": tele0 + 0.2,
                  "end_mono": tele0 + 1.2, "start_state": {"lift_m": 0.0}, "end_state": {"lift_m": 0.0}},
                 {"label": "lift_up", "mode": "direct", "lift_dir": 1, "start_mono": tele0 + 1.3,
                  "end_mono": tele0 + 2.2, "start_state": {"lift_m": 0.10}, "end_state": {"lift_m": 0.13}}]}}
    (sd / "probe.json").write_text(json.dumps(probe))
    with open(root / f"{name}_ps.jsonl", "w") as fh:
        fh.write(json.dumps({"sampler": "x"}) + "\n")
        for _ in range(10):
            fh.write(json.dumps({"other_cores": other_cores}) + "\n")
    return sd


def test_modes_holds_and_timers(tmp_path):
    s = tr.session(make_session(tmp_path, "lat_x"))
    assert s["modes"]["AUTO"]["ratio"] == pytest.approx(1.0, abs=1e-6)
    assert s["modes"]["TELEOP"]["ratio"] == pytest.approx(0.9, abs=1e-6)
    assert s["modes"]["TELEOP"]["longest_not_ok_s"] == pytest.approx(0.5, abs=0.02)
    assert s["modes"]["TELEOP"]["ok_wall"] == pytest.approx(1.5 / 2.0, abs=0.02)
    fwd = next(h for h in s["holds"] if h["label"] == "forward")
    assert fwd["ok_at_start"] is False and fwd["first_output_ms"] == pytest.approx(fwd["first_ok_ms"])
    assert fwd["first_output_ms"] == pytest.approx(300, abs=10) and fwd["reasons_before_output"] == ["pacing"]
    lift = next(h for h in s["holds"] if h["label"] == "lift_up")
    assert lift["lift_delta_m"] == pytest.approx(0.03) and lift["output_share"] == 0.0
    t = s["timers"]["TELEOP"]["teleop.permit"]
    assert t["mean_ms"] == pytest.approx(1.0) and t["max_ms"] == pytest.approx(10.0)
    assert s["overlap"] == 0.0


def test_the_summary_leaves_out_overlapped_sessions(tmp_path):
    a = tr.session(make_session(tmp_path, "lat_a"))
    b = tr.session(make_session(tmp_path, "lat_b", other_cores=2.0))
    out = tr.summarize([a, b])
    assert out["kept"] == ["lat_a"] and out["left_out"] == {"lat_b": 1.0}
    assert out["holds"]["forward"]["started_not_ok"] == 1 and out["holds"]["forward"]["output_at_first_ok"] == 1
    assert out["modes"]["TELEOP"]["ratio"]["median"] == pytest.approx(0.9, abs=1e-6)


def test_the_cli_writes_json(tmp_path):
    sd = make_session(tmp_path, "lat_c")
    out = tmp_path / "o.json"
    assert tr.main([str(sd), "--json", str(out)]) == 0
    assert json.loads(out.read_text())["summary"]["kept"] == ["lat_c"]
