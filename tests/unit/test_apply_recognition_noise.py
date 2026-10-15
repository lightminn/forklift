# tests/unit/test_apply_recognition_noise.py
import numpy as np
import pytest
import yaml

from forklift_core.perception.recognition_dataset import load_recognition_sample
from tools import apply_recognition_noise as app
from tools import generate_recognition_scenes as gen

CONFIG = {"source": "test", "license": "CC-BY-4.0", "revision": "r", "files_sha256": "x", "k_iid": 0.0006,
          "k_corr": 0.004, "correlation_px_source": [8.0, 6.0], "source_fx": 931.482312, "dropout_fraction": 0.02}


def _config(tmp_path):
    cfg = tmp_path / "noise.yaml"
    cfg.write_text(yaml.safe_dump(CONFIG))
    return cfg


def test_noise_is_applied_to_clean_scenes_and_truth_is_kept(tmp_path):
    gen.main(["--out", str(tmp_path / "clean"), "--cases", "S1", "--limit", "1", "--noise", "none"])
    app.main(["--src", str(tmp_path / "clean"), "--out", str(tmp_path / "noisy"), "--noise-config",
              str(_config(tmp_path))])
    for clean_dir in (tmp_path / "clean").iterdir():
        a, b = load_recognition_sample(clean_dir), load_recognition_sample(tmp_path / "noisy" / clean_dir.name)
        assert a.truth == b.truth and a.instruction == b.instruction and a.origin == b.origin
        noise = b.meta["noise"]
        assert noise["correlated_k"] == pytest.approx(0.004) and noise["role"] == "official" and noise["scale"] == 1.0
        assert noise["correlation_px"][0] == pytest.approx(4.0, rel=1e-3)  # 8 * 465.741156 / 931.482312
        diff = b.input.scene.depth_m - a.input.scene.depth_m
        assert diff[np.isfinite(diff)].std() > 0.001


def test_the_sensitivity_scale_marks_the_role_and_scales_the_width(tmp_path):
    gen.main(["--out", str(tmp_path / "clean"), "--cases", "S1", "--limit", "1", "--noise", "none"])
    app.main(["--src", str(tmp_path / "clean"), "--out", str(tmp_path / "k166"), "--noise-config",
              str(_config(tmp_path)), "--scale", "1.66"])
    noise = load_recognition_sample(next((tmp_path / "k166").iterdir())).meta["noise"]
    assert noise["role"] == "sensitivity" and noise["correlation_px"][0] == pytest.approx(4.0 * 1.66, rel=1e-3)


def test_noisy_scenes_are_not_noised_twice(tmp_path):
    gen.main(["--out", str(tmp_path / "noisy"), "--cases", "S1", "--limit", "1"])
    with pytest.raises(ValueError, match="already carries noise"):
        app.main(["--src", str(tmp_path / "noisy"), "--out", str(tmp_path / "again"), "--noise-config",
                  str(_config(tmp_path))])
