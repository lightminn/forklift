"""Plan D7c CPU docking re-alignment report (tools/d7c_reapproach_sweep.py)."""

import json

from tools import d7c_reapproach_sweep as SWEEP


def test_one_case_plans_back_to_the_line_start_with_the_adopted_options(tmp_path):
    args = SWEEP.parse_args(["--output", str(tmp_path), "--seeds", "1", "--laterals", "0.24", "--yaws", "0.08"])
    out = SWEEP.run(args)
    assert out["summary"]["cases"] == 1 and out["summary"]["accepted"] == 1
    assert out["summary"]["planner_options"] == {"goal_connection": "reeds_shepp", "shot_cap_m": 4.0}
    row = out["rows"][0]
    assert row["refused"] is None and row["length_m"] <= 6.0 and row["lateral_max_m"] <= 1.5
    # The runner's transport tracker stops at L within the docking stop's tolerance.
    assert row["dry_run"]["arrived"] and row["dry_run"]["position_error_m"] <= 0.03
    assert json.loads((tmp_path / "reapproach.json").read_text())["summary"]["accepted"] == 1


def test_a_refused_case_keeps_the_helpers_statistics():
    # Codex D7c P3: 2 m lateral leaves the box; its gear changes are still reported.
    args = SWEEP.parse_args(["--seeds", "1", "--laterals", "2.0", "--yaws", "0.0"])
    row = SWEEP.run(args)["rows"][0]
    assert row["refused"] == "leaves_box" and row["dry_run"] is None
    assert row["gear_changes"] is not None and row["expansions"] >= 1
