"""M-c5: reading the camera's pixel lag from the lift wave (plan C5), on synthetic frames."""

import importlib.util
import math

import numpy as np
import pytest

pytest.importorskip("PIL")
from PIL import Image  # noqa: E402

from pathlib import Path  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location("live_marker_lag", ROOT / "tools/live_marker_lag.py")
M = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(M)


def wave(t):
    return 0.14 + 0.06 * (np.sin(2 * np.pi * t / 2.0) + np.sin(2 * np.pi * t / 3.1))


def make_run(tmp_path, lag_of, factor=0.1, n=300):
    """A run whose frame k shows the lift at stamp - lag_of(k); the steps start 1/(120 f) s apart (wall)."""
    t = np.arange(0.0, 40.0, 1 / 120)
    log = [{"lift_sim_t": float(a), "lift_m": float(wave(a)), "step_start_mono": 1000.0 + (a - 1 / 120) / factor}
           for a in t]
    frames = []
    for k in range(n):
        stamp = 4.0 + k * 0.1
        row = 200 - 100 * (wave(stamp - lag_of(k)) - 0.14)
        y = np.arange(480)[:, None] - row
        img = np.repeat(np.clip(140 + 60 * np.tanh(y / 3.0) + 30 * np.sin(y / 9.0), 0, 255), 640, axis=1)
        Image.fromarray(img.astype(np.uint8)).save(tmp_path / f"frame_{k + 1:06d}.jpg", quality=95)
        frames.append({"frame_id": k + 1, "stamp_ns": int(round(stamp * 1e9))})
        log.append({"camera": k + 1, "latency_s": 0.05})
    return {"frames": frames}, log


def test_a_known_lag_is_read_back_and_turned_into_wall_time(tmp_path):
    report, log = make_run(tmp_path, lambda k: 0.05)
    r = M.analyse(report, tmp_path, log)
    assert r["pixel_lag_sim_s"] == pytest.approx(0.05, abs=1 / 120 + 1e-9)
    # wall = metadata latency + the lag through the step table (0.05 sim s at f 0.1 = 0.5 s), not a declared factor
    assert r["pixel_latency_wall_s_at_fitted_tau"]["median"] == pytest.approx(0.05 + 0.5, abs=0.1)
    assert r["alias_residual_ratio"] > 5


@pytest.mark.parametrize("lag", [1.05, 2.05])
def test_a_lag_over_a_sine_period_is_not_read_as_a_small_one(tmp_path, lag):
    report, log = make_run(tmp_path, lambda k: lag)
    r = M.analyse(report, tmp_path, log)
    assert r["pixel_lag_sim_s"] == pytest.approx(lag, abs=1 / 120 + 1e-9)


def test_frames_far_late_are_undetermined_not_a_clipped_maximum(tmp_path):
    report, log = make_run(tmp_path, lambda k: 0.55 if k % 10 == 0 else 0.05)
    r = M.analyse(report, tmp_path, log)
    assert r["per_frame_undetermined"] > 0
    assert r["per_frame_lag_sim_s"]["max"] < r["per_frame_window_s"][1]
