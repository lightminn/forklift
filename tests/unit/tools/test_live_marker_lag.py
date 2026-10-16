"""M-c5: reading the camera's pixel lag from the lift wave (plan C5), on synthetic frames."""

import importlib.util
import json
import sys
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("PIL")
from PIL import Image  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("live_marker_lag", ROOT / "tools/live_marker_lag.py")
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def test_a_known_lag_is_read_back(tmp_path, monkeypatch):
    lag = 0.05  # the pixels show the scene 50 ms of sim before the stamp
    t = np.arange(0.0, 30.0, 1 / 120)
    lift = 0.06 + 0.04 * np.sin(2 * np.pi * t / 2.0)
    (tmp_path / "live.jsonl").write_text("\n".join(json.dumps({"lift_sim_t": float(a), "lift_m": float(b)})
                                                   for a, b in zip(t, lift)))
    frames = []
    for k in range(250):
        stamp = 1.0 + k * 0.1
        h = 0.06 + 0.04 * np.sin(2 * np.pi * (stamp - lag) / 2.0)
        row = 200 - 600 * (h - 0.06)  # the camera rises, the scene goes down the picture
        y = np.arange(480)[:, None] - row
        img = np.repeat(np.clip(140 + 60 * np.tanh(y / 3.0) + 30 * np.sin(y / 9.0), 0, 255), 640, axis=1)
        img = img.astype(np.uint8)
        Image.fromarray(img).save(tmp_path / f"frame_{k + 1:06d}.jpg", quality=95)
        frames.append({"frame_id": k + 1, "stamp_ns": int(stamp * 1e9)})
    (tmp_path / "probe.json").write_text(json.dumps({"frames": frames}))
    out = tmp_path / "lag.json"
    monkeypatch.setattr(sys, "argv", ["x", "--report", str(tmp_path / "probe.json"), "--frames-dir", str(tmp_path),
                                      "--live-log", str(tmp_path / "live.jsonl"), "--factor", "0.1", "--out", str(out)])
    M.main()
    result = json.loads(out.read_text())
    assert result["pixel_lag_sim_s"] == pytest.approx(lag, abs=1 / 120 + 1e-9)
    assert abs(result["fit_correlation"]) > 0.99
