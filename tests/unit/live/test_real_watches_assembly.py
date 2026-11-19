"""docs/design/2026-11-19-real-safety-wiring.md 4 (stage 5b): build_watches -- one assembly of the watches for the
dry and the live assembly; a missing or broken watch file, an unknown IMU policy or an IMU setting the watch cannot
judge with refuses."""

from pathlib import Path

import pytest
import yaml

from forklift_core.live.real_assembly import AssemblyRefused, build_watches

ROOT = Path(__file__).resolve().parents[3]
WATCH = ROOT / "config/real/safety_watch.yaml"
R = (0.06, 0.06)


def test_the_configs_build_a_required_imu_watch():
    autonomy = yaml.safe_load((ROOT / "config/real/autonomy.yaml").read_text())
    w = build_watches(autonomy, WATCH, R)
    assert w.imu_policy == "required" and w.imu is not None and w.safety is not None


def test_none_builds_no_imu_watch():
    assert build_watches({"imu": "none"}, WATCH, R).imu is None


@pytest.mark.parametrize("autonomy", [{}, {"imu": "maybe"}, {"imu": True}])
def test_an_unknown_policy_refuses(autonomy):
    with pytest.raises(AssemblyRefused) as e:
        build_watches(autonomy, WATCH, R)
    assert e.value.reasons[0].startswith("imu_policy")


def test_a_missing_watch_file_refuses(tmp_path):
    with pytest.raises(AssemblyRefused) as e:
        build_watches({"imu": "required"}, tmp_path / "none.yaml", R)
    assert e.value.reasons[0].startswith("watch_config")


@pytest.mark.parametrize("change,reason", [({"limit_deg": 181.0}, "imu_config:limit_deg"),
                                           ({"gap_max_s": 0.5}, "imu_config:gap_max_s"),
                                           ({"provenance": None}, "imu_config:missing")])
def test_an_imu_setting_it_cannot_judge_with_refuses(tmp_path, change, reason):
    raw = yaml.safe_load(WATCH.read_text())
    raw["imu"].update(change)
    if change.get("provenance", 1) is None:
        del raw["imu"]["provenance"]
    path = tmp_path / "watch.yaml"
    path.write_text(yaml.safe_dump(raw))
    with pytest.raises(AssemblyRefused) as e:
        build_watches({"imu": "required"}, path, R)
    assert e.value.reasons == [reason]
    assert build_watches({"imu": "none"}, path, R).imu is None  # the IMU part is not read with none
