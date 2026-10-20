"""SLAM localization plan, V5 diagnosis: the localization replay's score and the localization params."""

import json
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[3]


def _session(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    np.savez_compressed(run / "slam_log.npz", scan_ranges_m=np.full((4, 1600), np.inf, np.float32),
                        joint_stamps_s=np.arange(4) / 10.0)
    bundle = tmp_path / "bundle"
    bundle.mkdir()
    np.savez(bundle / "grid.npz", data=np.zeros((10, 10), np.int8), origin=np.zeros(3), resolution_m=0.05)
    live = tmp_path / "records.jsonl"
    live.write_text("".join(json.dumps({"scan_id": i, "stamp_s": i / 10.0, "sent": "local" if i == 2 else "expect_pose"})
                            + "\n" for i in range(4)))
    return run, bundle, live


def _replies(path, ids, *, correction=(0.0, 0.0, 0.0), skipped=()):
    path.write_text("".join(json.dumps({"scan_id": i, "stamp_s": i / 10.0,
                                        "status": "skipped" if i in skipped else "processed",
                                        "map_from_odom": list(correction) if i == 3 else [0.0, 0.0, 0.0],
                                        "odom": [i * 0.1, 0.0, 0.0], "truth_base": [i * 0.1, 0.0, 0.0]}) + "\n"
                            for i in ids))


def _eval(*args):
    return subprocess.run([sys.executable, str(ROOT / "tools/slam_loc_replay_eval.py"), *map(str, args)],
                          capture_output=True, text=True)


def test_a_complete_replay_scores_and_reports_the_correction_jump(tmp_path):
    run, bundle, live = _session(tmp_path)
    _replies(tmp_path / "r.jsonl", [0, 1, 3], correction=(0.03, 0.0, 0.0))
    out = _eval(run, tmp_path / "r.jsonl", bundle, live)
    assert out.returncode == 0, out.stdout + out.stderr
    data = json.loads(out.stdout)
    assert data["complete"] and abs(data["jump"]["position_max_m"] - 0.03) < 1e-9
    assert data["jump"]["position_over_0_02_m"] == 1


def test_a_replay_that_stopped_early_is_not_complete(tmp_path):
    run, bundle, live = _session(tmp_path)
    _replies(tmp_path / "r.jsonl", [0])  # a prefix: one processed reply
    out = _eval(run, tmp_path / "r.jsonl", bundle, live)
    assert out.returncode == 1 and not json.loads(out.stdout)["complete"]


def test_a_keyframe_that_should_have_a_pose_but_was_skipped_is_not_complete(tmp_path):
    run, bundle, live = _session(tmp_path)
    _replies(tmp_path / "r.jsonl", [0, 1, 3], skipped=(3,))
    out = _eval(run, tmp_path / "r.jsonl", bundle, live)
    assert out.returncode == 1 and not json.loads(out.stdout)["complete"]


def test_bridge_records_missing_their_tail_are_refused(tmp_path):
    sys.path.insert(0, str(ROOT / "tools"))
    import pytest
    import slam_loc_replay_drive as drive

    _, _, live = _session(tmp_path)
    records = [{"scan_id": i, "stamp_s": i / 10.0} for i in range(5)]  # the runner sent five
    with pytest.raises(SystemExit, match="one for one"):
        drive.sent_ids(live, records)
    assert drive.sent_ids(live, records[:4]) == [0, 1, 3]


def test_the_localization_variant_closes_loops_over_short_links():
    sys.path.insert(0, str(ROOT / "tools"))
    import slam_localization_params as p

    params = p.localization_params({"slam_toolbox": {"ros__parameters": {"mode": "mapping"}}}, "/bundle/map", [0, 0, 0])
    r = params["slam_toolbox"]["ros__parameters"]
    assert r["mode"] == "localization" and r["do_loop_closing"] is True and r["link_scan_maximum_distance"] == 3.0
    assert (r["loop_search_space_dimension"], r["loop_search_maximum_distance"], r["scan_buffer_size"]) == (3.0, 2.0, 3)
