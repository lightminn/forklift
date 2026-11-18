"""Design 2026-11-19-real-assembly 1: tools/real_autonomy.py --dry-assemble -- 2 for a structural refusal (the
repository's template, whose field values are unset), 0 for a sound assembly whose reference plan succeeds; the
record names the options, the hooks and the caps. No device is opened."""

import json
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))
import real_autonomy as tool  # noqa: E402


def test_the_template_is_refused_with_its_unset_values(tmp_path):
    out = tmp_path / "assembly.json"
    assert tool.main(["--dry-assemble", "--out", str(out)]) == 2
    rec = json.loads(out.read_text())
    assert rec["assembled"] is False
    assert {"map_id_unset", "pallet_depth_m_unset", "check_pallet_pose_unset"} <= set(rec["refused"])


def test_a_filled_configuration_assembles_and_plans_its_reference(tmp_path):
    d = yaml.safe_load((ROOT / "config/real/s10a.yaml").read_text())
    d.update(map_id="hall-1", check_pallet_pose=[3.0, 0.0, 0.0])
    d["pallet"].update(depth_m=0.6, width_m=0.4)
    path = tmp_path / "s10a.yaml"
    path.write_text(yaml.safe_dump(d))
    out = tmp_path / "assembly.json"
    code = tool.main(["--dry-assemble", "--s10a", str(path), "--map-id", "hall-1", "--out", str(out)])
    rec = json.loads(out.read_text())
    assert rec["assembled"] is True, rec
    assert code == 0 and rec["check_plan"] == "success"
    assert rec["options"]["dt"] == 1 / 50 and rec["options"]["s10_stop_at_lift"] is True
    assert rec["ready"].startswith("not_ready") and rec["curvature_caps"]["planner"] == rec["settings"][
        "tracker_curvature_inv_m"]


def test_another_map_is_refused(tmp_path):
    d = yaml.safe_load((ROOT / "config/real/s10a.yaml").read_text())
    d.update(map_id="hall-1", check_pallet_pose=[3.0, 0.0, 0.0])
    d["pallet"].update(depth_m=0.6, width_m=0.4)
    path = tmp_path / "s10a.yaml"
    path.write_text(yaml.safe_dump(d))
    assert tool.main(["--dry-assemble", "--s10a", str(path), "--map-id", "hall-2"]) == 2
