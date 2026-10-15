# tests/unit/test_apply_recognition_noise.py
import numpy as np
import pytest
import yaml

from forklift_core.perception.recognition_dataset import load_recognition_sample
from tools import apply_recognition_noise as app
from tools import generate_recognition_scenes as gen


def _levels(k_long):
    return {"k_iid": 0.0, "components": [{"k": 0.0005, "correlation_px_source": [4.0, 0.0]},
                                         {"k": k_long, "correlation_px_source": [8.0, 6.0]}]}


CONFIG = {"source": "test", "license": "CC-BY-4.0", "revision": "r", "files_sha256": "x", "role": "provisional",
          "source_fx": 931.482312, "dropout_fraction": 0.02,
          "levels": {"low": _levels(0.002), "mid": _levels(0.004), "high": _levels(0.006)}}


def _config(tmp_path, **changes):
    cfg = tmp_path / "noise.yaml"
    cfg.write_text(yaml.safe_dump({**CONFIG, **changes}))
    return cfg


def _clean(tmp_path):
    gen.main(["--out", str(tmp_path / "clean"), "--cases", "S1", "--limit", "1", "--noise", "none"])
    return tmp_path / "clean"


def test_noise_is_applied_to_clean_scenes_and_truth_is_kept(tmp_path):
    clean = _clean(tmp_path)
    app.main(["--src", str(clean), "--out", str(tmp_path / "noisy"), "--noise-config", str(_config(tmp_path))])
    for clean_dir in clean.iterdir():
        a, b = load_recognition_sample(clean_dir), load_recognition_sample(tmp_path / "noisy" / clean_dir.name)
        assert a.truth == b.truth and a.instruction == b.instruction and a.origin == b.origin
        noise = b.meta["noise"]
        assert noise["axial_k"] == 0.0 and noise["role"] == "provisional" and noise["level"] == "mid"
        assert noise["components"][1][0] == pytest.approx(0.004)
        assert noise["components"][1][1][0] == pytest.approx(4.0, rel=1e-3)  # 8 * 465.741156 / 931.482312
        diff = b.input.scene.depth_m - a.input.scene.depth_m
        assert diff[np.isfinite(diff)].std() > 0.001


@pytest.mark.parametrize("role, scale, expected", [("provisional", 1.0, "provisional"), ("official", 1.0, "official"),
                                                    ("official", 1.66, "sensitivity")])
def test_the_role_comes_from_the_config_and_the_scale(tmp_path, role, scale, expected):
    clean = _clean(tmp_path)
    app.main(["--src", str(clean), "--out", str(tmp_path / "out"), "--noise-config",
              str(_config(tmp_path, role=role)), "--scale", str(scale), "--level", "high"])
    noise = load_recognition_sample(next((tmp_path / "out").iterdir())).meta["noise"]
    assert noise["role"] == expected and noise["level"] == "high"


def test_noisy_scenes_are_not_noised_twice(tmp_path):
    gen.main(["--out", str(tmp_path / "noisy"), "--cases", "S1", "--limit", "1"])
    with pytest.raises(ValueError, match="already carries noise"):
        app.main(["--src", str(tmp_path / "noisy"), "--out", str(tmp_path / "again"), "--noise-config",
                  str(_config(tmp_path))])
