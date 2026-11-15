"""tools/m_analyze.py (docs/design/2026-11-15-m-analyze-entry.md 2, 5): files in, one m_analyze/1 line out; refusals
exit 2 with only the refusal line."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))

import m_analyze as tool  # noqa: E402
from forklift_core.live.synthetic import OUTPUT_MARKS  # noqa: E402
from forklift_core.plant import m_synth as ms  # noqa: E402
from forklift_core.plant.real_vehicle import load_real_vehicle  # noqa: E402

PROFILE = REPO / "config" / "profiles" / "m1_m3" / "m1_fwd_v010.yaml"
CFG = load_real_vehicle(REPO / "config" / "real" / "vehicle.yaml")
TIGHT = {"pose_yaw_rad": 1e-4, "pose_xy_m": 2e-4, "source": "test"}


def _write(tmp, name, rows):
    p = tmp / name
    p.write_text("".join(json.dumps(r) + "\n" for r in rows))
    return p


def _files(tmp, **kw):
    out = ms.synth_trial(yaml.safe_load(PROFILE.read_text()), CFG, still_s=5.0, true_yaw_per_m=0.003, **kw)
    out["truth"][0]["sigma"] = dict(TIGHT)
    return {k: _write(tmp, f"{k}.jsonl", v) for k, v in out.items()}, out


def _run(capsys, files, *extra):
    code = tool.main(["--record", str(files["record"]), "--imu", str(files["imu"]), "--profile", str(PROFILE),
                      "--truth", str(files["truth"]), "--up", "0,-1,0", *extra])
    lines = [x for x in capsys.readouterr().out.splitlines() if x.strip()]
    return code, [json.loads(x) for x in lines]


def test_a_synthetic_day_is_refused_without_the_flag(tmp_path, capsys):
    files, _ = _files(tmp_path)
    code, out = _run(capsys, files)
    assert code == 2 and len(out) == 1 and out[0]["result"] == "refused" and "synthetic" in out[0]["why"]


def test_with_the_flag_the_result_is_marked_first_and_carries_its_inputs(tmp_path, capsys):
    files, _ = _files(tmp_path)
    code, out = _run(capsys, files, "--allow-synthetic")
    assert code == 0 and len(out) == 1
    res = out[0]
    assert list(res)[: len(OUTPUT_MARKS)] == list(OUTPUT_MARKS) and all(res[k] is True for k in OUTPUT_MARKS)
    assert res["schema"] == "m_analyze/1" and res["measurement"] == "m1"
    assert res["inputs"]["profile"]["sha256"] and res["inputs"]["up"] == {"vector": [0.0, -1.0, 0.0],
                                                                          "source": "argument"}
    assert res["inputs"]["vehicle"]["calibration_id"] == CFG.calibration_id
    assert "config_check" in res["inputs"]  # the synthetic header names no configuration


def test_a_record_that_names_another_vehicle_or_profile_is_refused(tmp_path, capsys):
    files, out = _files(tmp_path)
    for bad in ({"vehicle": {"calibration_id": "0" * 16}, "profile": None},
                {"vehicle": {"calibration_id": CFG.calibration_id}, "profile": {"sha256": "0" * 64}}):
        rec = [dict(out["record"][0], config=bad), *out["record"][1:]]
        files["record"] = _write(tmp_path, "record.jsonl", rec)
        code, res = _run(capsys, files, "--allow-synthetic")
        assert code == 2 and res[0]["result"] == "refused" and "differs" in res[0]["why"]


def test_bad_inputs_are_refusals_not_tracebacks(tmp_path, capsys):
    files, _ = _files(tmp_path)
    code = tool.main(["--record", str(files["record"]), "--imu", str(files["imu"]), "--profile", str(PROFILE),
                      "--no-truth", "--up", "1,2", "--allow-synthetic"])
    out = capsys.readouterr().out
    assert code == 2 and json.loads(out)["result"] == "refused"
    code = tool.main(["--record", str(tmp_path / "missing.jsonl"), "--imu", str(files["imu"]), "--profile",
                      str(PROFILE), "--no-truth", "--up", "0,-1,0"])
    assert code == 2 and json.loads(capsys.readouterr().out)["result"] == "refused"
