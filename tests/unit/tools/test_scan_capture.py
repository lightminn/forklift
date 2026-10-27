"""The scan capture's bookkeeping (sim/isaac/scan_capture.py, plan B4a) on a fake engine: the live send path, the
per-scan noise seeds, the truth read back after the settle, and the validity rules."""

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "sim" / "isaac"))
import scan_capture as C  # noqa: E402


def test_the_sent_ranges_follow_the_live_shared_path():
    ranges = np.array([0.21, 5.0, np.inf, -np.inf, 11.99])
    noise = np.array([-0.06, 0.02, 0.05, 0.05, 0.05])
    sent = C.shared_sent_ranges(ranges, noise, 0.2, 12.0)
    # a finite range is clipped to the limits (0.15 -> 0.20, 12.04 -> 12.0); REP-117 infinities stay as they are
    assert sent[0] == 0.2 and sent[1] == 5.02 and sent[2] == np.inf and sent[3] == -np.inf and sent[4] == 12.0


def test_noise_seeds_differ_by_split_map_pose_and_scan():
    draws = {(sp, m, p, s): C.noise_rng(sp, m, p, s).normal(size=3).tobytes()
             for sp in (0, 1) for m in (0, 1) for p in (0, 1) for s in (0, 1)}
    assert len(set(draws.values())) == len(draws)
    assert C.noise_rng(1, 0, 3, 2).normal() == C.noise_rng(1, 0, 3, 2).normal()


class FakeEngine:
    def __init__(self, drift=(0.0, 0.0), tilt=0.0, creep=0.0, z=0.015):
        self.pos, self.q = np.zeros(3), C.quat_from_yaw(0.0)
        self.drift, self.tilt, self.creep, self.z = drift, tilt, creep, z
        self.teleports = []

    def teleport(self, pos, q):
        self.teleports.append((pos.copy(), q.copy()))
        self.pos = pos + [self.drift[0], self.drift[1], self.z - pos[2]]
        yaw = C.yaw_roll_pitch(q)[0]
        h = self.tilt / 2  # a roll about x on top of the yaw
        self.q = np.array([math.cos(yaw / 2) * math.cos(h), math.cos(yaw / 2) * math.sin(h),
                           math.sin(yaw / 2) * math.sin(h), math.sin(yaw / 2) * math.cos(h)])

    def step(self):
        self.pos = self.pos + [self.creep, 0.0, 0.0]

    def pose(self):
        return self.pos.copy(), self.q.copy()

    def cast(self, pos, q):
        return np.array([1.0, 2.0, np.nan]), np.array([True, True, False])


def _engine(fake):
    return C.Engine(teleport=fake.teleport, step=fake.step, pose=fake.pose, cast=fake.cast,
                    draw_noise=lambda rng, d: np.clip(rng.normal(0, 0.02, len(d)), -0.06, 0.06),
                    ranges_from_hits=lambda d, h: np.where(h, d, np.inf), range_min_m=0.2, range_max_m=12.0,
                    settle_z_m=0.015, laser_xyz_m=(-0.12, 0.0, 1.05))


POSES = {"format": "scan_capture_poses/1", "split": 1, "map_index": 0,
         "poses": [{"id": "v0", "stratum": "A", "base": [2.0, 1.0, math.pi / 2],
                    "base_world": [2.0, 1.0, math.pi / 2]}]}


def test_a_settled_pose_records_its_truth_and_five_scans():
    out = C.capture(_engine(FakeEngine()), POSES)
    assert out["valid"].tolist() == [True] and out["raw"].shape == (1, 5, 3) and out["sent"].dtype == np.float32
    assert np.allclose(out["laser_truth"][0], (2.0, 1.0 - 0.12, math.pi / 2), atol=1e-9)
    assert len(set(out["sent_sha256"][0])) == 5  # each scan its own noise
    assert np.isinf(out["sent"][0, :, 2]).all()


def test_poses_the_truck_did_not_settle_on_are_invalid_not_replaced():
    assert C.capture(_engine(FakeEngine(drift=(0.03, 0.0))), POSES)["reason"].tolist() == ["not_at_target"]
    assert C.capture(_engine(FakeEngine(tilt=0.02)), POSES)["reason"].tolist() == ["tilt"]
    assert C.capture(_engine(FakeEngine(creep=0.001)), POSES)["reason"].tolist() == ["moved"]
    assert C.capture(_engine(FakeEngine(z=0.04)), POSES)["reason"].tolist() == ["height"]
