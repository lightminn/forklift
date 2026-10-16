"""M-c5: reading the camera's pixel lag from the marker frames (plan C5), on synthetic frames."""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

PIL = pytest.importorskip("PIL")
from PIL import Image  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("live_marker_lag", ROOT / "tools/live_marker_lag.py")
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def test_a_known_lag_is_read_back(tmp_path, monkeypatch):
    lag = 0.05  # the pixels show the scene 50 ms of sim before the stamp
    frames = []
    for k in range(60):
        stamp = 10.0 + k * 0.1 + 0.013
        phase = (stamp - lag) % 1.0
        img = np.zeros((100, 400, 3), dtype=np.uint8)
        col = int(20 + 360 * phase)
        img[45:55, col:col + 6] = (255, 0, 0)
        Image.fromarray(img).save(tmp_path / f"frame_{k + 1:06d}.jpg", quality=95)
        frames.append({"frame_id": k + 1, "stamp_ns": int(stamp * 1e9)})
    (tmp_path / "probe.json").write_text(json.dumps({"frames": frames}))
    out = tmp_path / "lag.json"
    monkeypatch.setattr(sys, "argv", ["x", "--report", str(tmp_path / "probe.json"), "--frames-dir", str(tmp_path),
                                      "--factor", "0.1", "--out", str(out)])
    M.main()
    result = json.loads(out.read_text())
    assert result["lag_sim_s"]["median"] == pytest.approx(lag, abs=0.02)
    assert result["lag_wall_s"]["median"] == pytest.approx(lag / 0.1, abs=0.2)
