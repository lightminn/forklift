"""The observation's capture pose stays apart from the tick's truth (docs/validation/2026-11-02-observe-capture-base.md).

On the tick an observation completes, the runner used to unpack the capture's truth pose into the tick's own ``base``,
``q`` and ``yaw``: everything later in that tick -- the slip watch's baseline, the trace evidence, the stand lock and its
probe, the 0.1 s samples, the chase camera -- read the pose of the capture instant (30-74 mm off at a near capture's
recovery: S1 st_b3e3ea7f's three failed lock lifetimes, rc_95e61a74's two sample spikes). Only the core's observation
calls take the capture pose."""

import re
from pathlib import Path

SOURCE = (Path(__file__).resolve().parents[3] / "sim/isaac/run_transport.py").read_text()


def test_the_capture_pose_never_overwrites_the_ticks_truth():
    assert not re.search(r"\bbase, q, yaw = captured_", SOURCE)
    assert not re.search(r"(^|[\s(,])(base|q|yaw) *= *captured_\.(base|q|yaw)\b", SOURCE, re.M)
    assert SOURCE.count("cap_base, cap_yaw = ") == 2


def test_only_the_cores_observation_calls_take_the_capture_pose():
    uses = [line.strip() for line in SOURCE.splitlines() if re.search(r"\bcap_(base|yaw)\b", line)]
    calls = [u for u in uses if "cap_base, cap_yaw =" not in u and not u.startswith("captured_.yaw")]
    assert len(calls) == 4
    assert sum("core.observe_succeeded(t, rear, cap_base, cap_yaw," in c for c in calls) == 2
    assert sum("core.near_leg_or_recapture(t, rear, cap_base, cap_yaw," in c for c in calls) == 2


def test_the_ticks_truth_is_read_once_at_its_start():
    drive = SOURCE[SOURCE.index("def drive(S, isaac)"):]
    assert drive.count("                base, q = robot.get_world_pose()") == 1
