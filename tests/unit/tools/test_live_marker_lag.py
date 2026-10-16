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
    assert r["pixel_latency_wall_s_conforming_frames"]["median"] == pytest.approx(0.05 + 0.5, abs=0.1)
    assert r["alias_residual_ratio"] > 5


@pytest.mark.parametrize("lag", [1.05, 2.05])
def test_a_lag_over_a_sine_period_is_not_read_as_a_small_one(tmp_path, lag):
    report, log = make_run(tmp_path, lambda k: lag)
    r = M.analyse(report, tmp_path, log)
    assert r["pixel_lag_sim_s"] == pytest.approx(lag, abs=1 / 120 + 1e-9)


def test_frames_late_by_more_are_listed_as_nonconforming_not_given_a_lag(tmp_path):
    late = {k for k in range(300) if k % 10 == 0}
    report, log = make_run(tmp_path, lambda k: 0.55 if k in late else 0.05)
    r = M.analyse(report, tmp_path, log)
    assert r["pixel_lag_sim_s"] == pytest.approx(0.05, abs=1 / 120 + 1e-9)
    assert "per_frame_lag_sim_s" not in r
    assert r["trim_converged"]
    flagged = set(r["nonconforming_frames"])
    # every late frame whose picture differs from the on-time one by more than 3 rows is caught; no on-time frame is
    for k in range(300):
        stamp = 4.0 + k * 0.1
        visible = 100 * abs(wave(stamp - 0.55) - wave(stamp - 0.05)) > 3
        if k + 1 in {f["frame_id"] for f in report["frames"]} and stamp > 3.5 and stamp < 37.0:
            if k in late and visible:
                assert k + 1 in flagged
            if k not in late:
                assert k + 1 not in flagged


def test_the_trimmed_fit_and_its_statistics_use_one_set(tmp_path):
    rng = np.random.default_rng(3)
    late = set(rng.choice(300, 30, replace=False).tolist())
    report, log = make_run(tmp_path, lambda k: 0.05 + (rng.uniform(0.02, 0.4) if k in late else 0.0))
    r = M.analyse(report, tmp_path, log)
    assert r["trim_converged"] and r["pixel_lag_sim_s"] == pytest.approx(0.05, abs=1 / 120 + 1e-9)
    # refitting the reported conforming set gives the reported lag back
    keep = [f for f in report["frames"] if f["frame_id"] not in set(r["nonconforming_frames"])]
    again = M.analyse({"frames": keep}, tmp_path, log)
    assert again["pixel_lag_sim_s"] == pytest.approx(r["pixel_lag_sim_s"], abs=1 / 120 + 1e-9)
