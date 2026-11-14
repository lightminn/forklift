"""G6 (docs/design/2026-11-14-profile-drive-and-m-analysis.md 2): the profile file's checks and the runner that puts a
profile's speed and curvature into the operator's teleop messages without opening any gate."""

from __future__ import annotations

import math

import pytest

from forklift_core.live import drive_profile as dp

# a toy steering map: q = 0.65 * kappa (close to the real truck's near 0), limits like the bridge's
LIM = dp.ProfileLimits(cap_mps=0.132, kappa_max=0.36, kappa_rate=0.5, q_rate=1.0, accel_mps2=0.8,
                       q_of_kappa=lambda k: 0.65 * k, kappa_of_q=lambda q: q / 0.65, max_total_s=600.0)
NS = 1_000_000_000
DT = 0.02


def prof(*segments, load="empty"):
    return {"schema": dp.SCHEMA, "name": "t", "load": load, "segments": list(segments)}


ARC = prof({"label": "presteer", "presteer_kappa_inv_m": "max"}, {"label": "still_pre", "hold_s": 1.0},
           {"label": "arc", "speed_mps": 0.10, "kappa_inv_m": "max", "distance_m": 0.3},
           {"label": "still_post", "hold_s": 0.5})


# --- the file ------------------------------------------------------------------------------------------------------

def test_a_valid_profile_reads_with_max_and_q():
    p = dp.parse_profile(ARC, LIM)
    assert [s.kind for s in p.segments] == ["presteer", "hold", "drive", "hold"]
    assert p.segments[2].kappa_at(0.1, 0.0) == pytest.approx(0.36)
    q = dp.parse_profile(prof({"label": "p", "presteer_q_rad": 0.13},
                              {"label": "d", "speed_mps": -0.1, "q_rad": 0.13, "distance_m": 0.2}), LIM)
    assert q.segments[1].kappa_at(0.0, 0.0) == pytest.approx(0.2) and q.segments[1].speed_mps == -0.1


@pytest.mark.parametrize("seg, match", [
    ({"label": "d", "speed_mps": 0.2, "kappa_inv_m": 0.0, "distance_m": 1}, "speed"),
    ({"label": "d", "speed_mps": 0.1, "kappa_inv_m": 0.4, "distance_m": 1}, "curvature"),
    ({"label": "d", "speed_mps": 0.132, "s_merge": {"offset_m": 0.004, "length_m": 0.3}, "distance_m": 0.3}, "rate"),
    ({"label": "d", "speed_mps": 0.1, "s_merge": {"offset_m": 0.04, "length_m": 0.3}, "distance_m": 0.3}, "curvature"),
    ({"label": "d", "speed_mps": 0.0, "kappa_inv_m": 0.0, "distance_m": 1}, "speed"),
])
def test_a_drive_the_gates_would_reshape_is_refused(seg, match):
    with pytest.raises(ValueError, match=match):
        dp.parse_profile(prof({"label": "p", "presteer_kappa_inv_m": 0.0}, seg), LIM)


def test_loaded_profiles_a_missing_presteer_a_curvature_jump_and_overtime_are_refused():
    with pytest.raises(ValueError, match="empty"):
        dp.parse_profile(prof({"label": "p", "presteer_kappa_inv_m": 0.0}, load="loaded"), LIM)
    with pytest.raises(ValueError, match="presteer"):
        dp.parse_profile(prof({"label": "d", "speed_mps": 0.1, "kappa_inv_m": 0.0, "distance_m": 1}), LIM)
    with pytest.raises(ValueError, match="continu"):
        dp.parse_profile(prof({"label": "p", "presteer_kappa_inv_m": 0.0},
                              {"label": "d", "speed_mps": 0.1, "kappa_inv_m": 0.2, "distance_m": 1}), LIM)
    with pytest.raises(ValueError, match="total"):
        dp.parse_profile(prof({"label": "p", "presteer_kappa_inv_m": 0.0}, {"label": "h", "hold_s": 700}), LIM)


def test_the_sine_merge_moves_sideways_by_its_offset_and_ends_straight():
    p = dp.parse_profile(prof({"label": "p", "presteer_kappa_inv_m": 0.0},
                              {"label": "m", "speed_mps": 0.10, "s_merge": {"offset_m": 0.04, "length_m": 0.9},
                               "distance_m": 0.9}), LIM)
    seg = p.segments[1]
    assert seg.peak_kappa == pytest.approx(2 * math.pi * 0.04 / 0.81)
    n = 9000
    ds = 0.9 / n
    th = y = 0.0
    for i in range(n):
        th += seg.kappa_at((i + 0.5) * ds, 0.0) * ds
        y += math.sin(th) * ds
    assert th == pytest.approx(0.0, abs=1e-9) and y == pytest.approx(0.04, rel=1e-3)
    assert seg.kappa_at(0.9, 0.0) == pytest.approx(0.0, abs=1e-12)


def test_a_q_step_is_by_time_after_the_speed_is_reached_and_its_ramp_is_reported():
    p = dp.parse_profile(prof({"label": "p", "presteer_kappa_inv_m": 0.0},
                              {"label": "s", "speed_mps": 0.10, "kappa_inv_m": 0.0, "distance_m": 0.6,
                               "q_step": {"to_rad": 0.13, "after_s": 2.0}}), LIM)
    seg = p.segments[1]
    assert seg.kappa_at(0.1, 1.9) == 0.0 and seg.kappa_at(0.25, 2.0) == pytest.approx(0.2)
    # curvature 0 -> 0.2 at the teleop rate 0.5 /m/s takes 0.4 s, longer than the q slew's 0.13 s
    assert seg.q_step_ramp_s == pytest.approx(max(0.2 / 0.5, 0.13 / 1.0))
    with pytest.raises(ValueError, match="settles"):
        dp.parse_profile(prof({"label": "p", "presteer_kappa_inv_m": 0.0},
                              {"label": "s", "speed_mps": 0.10, "kappa_inv_m": 0.0, "distance_m": 0.25,
                               "q_step": {"to_rad": 0.13, "after_s": 2.0}}), LIM)
    with pytest.raises(ValueError, match="finite"):
        dp.parse_profile(prof({"label": "p", "presteer_kappa_inv_m": 0.0},
                              {"label": "s", "speed_mps": 0.10, "kappa_inv_m": 0.0, "distance_m": 0.6,
                               "q_step": {"to_rad": 0.13, "after_s": float("nan")}}), LIM)
    with pytest.raises(ValueError, match="after_s"):
        dp.parse_profile(prof({"label": "p", "presteer_kappa_inv_m": 0.0},
                              {"label": "s", "speed_mps": 0.10, "kappa_inv_m": 0.0, "distance_m": 0.6,
                               "q_step": {"to_rad": 0.13, "after_s": 0.1}}), LIM)


# --- the runner ----------------------------------------------------------------------------------------------------

class Rig:
    """A runner with a plant that follows the request at once (the gates are elsewhere)."""

    def __init__(self, profile=ARC):
        self.r = dp.ProfileRunner(dp.parse_profile(profile, LIM), LIM)
        self.t = 10 * NS
        self.q = 0.0
        self.speed = 0.0
        self.deadman = True
        self.armed = True
        self.applied = None  # None: the plant applies the request
        self.rows, self.marks = [], []

    def msg(self):
        return {"t": "teleop", "seq": 1, "mode": "direct", "deadman": self.deadman, "speed_ratio": 0.5,
                "curvature_ratio": -0.3}

    def tick(self, n=1):
        for _ in range(n):
            out = self.r.rewrite(self.msg())
            req_v = out["speed_ratio"] * LIM.cap_mps
            req_k = out["curvature_ratio"] * LIM.kappa_max
            self.q = LIM.q_of_kappa(req_k)  # steering follows at once in this rig
            self.speed = req_v if self.applied is None else self.applied
            obs = dp.Observation(dt_s=DT, speed_valid=True, speed_mps=self.speed, meas_q=self.q, armed=self.armed,
                                 mode="TELEOP" if self.armed else "IDLE", target_mps=req_v, applied_mps=self.speed,
                                 applied_q=self.q, permit=(1.0, "ok"), port="queued", direction_cut=None, stop=None,
                                 deadman=self.deadman)
            row = self.r.step(self.t, obs)
            if row is not None:
                self.rows.append(row)
            self.marks += self.r.take_marks()
            self.t += int(DT * NS)

    def start(self, still_s=2.0):
        return self.r.start(self.t, standstill_s=still_s, armed=self.armed)


def test_idle_leaves_the_operators_message_alone():
    rig = Rig()
    m = rig.msg()
    assert rig.r.rewrite(m) == m and rig.r.state == "idle"


def test_a_start_needs_arming_and_a_standstill():
    rig = Rig()
    assert rig.start(still_s=0.5) == "not_still"
    rig.armed = False
    assert rig.start() == "not_armed"
    rig.armed = True
    assert rig.start() is None and rig.r.state == "running"
    assert rig.start() == "busy"


def test_a_profile_runs_presteer_hold_drive_hold_and_ends_with_zeros_until_the_deadman_is_let_go():
    rig = Rig()
    rig.start()
    rig.tick(300)
    labels = [m["label"] for m in rig.marks]
    assert labels == ["profile:1:presteer:start", "profile:1:still_pre:start", "profile:1:arc:start",
                      "profile:1:still_post:start"]
    drive = [r for r in rig.rows if r["label"] == "arc"]
    assert drive and drive[-1]["s_m"] == pytest.approx(0.3, abs=0.003)
    assert max(r["req_mps"] for r in drive) == pytest.approx(0.10)
    assert rig.r.result == "done" and rig.r.state == "zeroing"
    z = rig.r.rewrite(rig.msg())
    assert z["speed_ratio"] == 0.0 and z["curvature_ratio"] == 0.0 and z["deadman"] is True
    rig.deadman = False
    rig.tick()
    assert rig.r.state == "idle"
    assert rig.r.rewrite(rig.msg())["speed_ratio"] == 0.5  # the operator has the joystick back


def test_still_segments_go_on_without_the_deadman_but_a_drive_waits_for_it():
    rig = Rig()
    rig.deadman = False
    rig.start()
    rig.tick(150)
    assert rig.r.segment.label == "arc" and rig.r.result is None
    assert all(r["req_mps"] == 0.0 for r in rig.rows)  # waiting, not driving
    rig.deadman = True
    rig.tick(60)
    assert any(r["label"] == "arc" and r["req_mps"] > 0 for r in rig.rows)


def test_letting_go_while_driving_voids_the_trial_and_it_never_resumes():
    rig = Rig()
    rig.start()
    rig.tick(80)  # into the drive
    assert rig.r.segment.label == "arc" and rig.speed > 0
    rig.deadman = False
    rig.tick()
    assert rig.r.result == "void:deadman"
    rig.deadman = True
    rig.tick(20)
    assert rig.r.state == "zeroing" and all(r["req_mps"] == 0.0 for r in rig.rows[-20:])


def test_a_gate_cutting_the_speed_outside_the_ramp_voids_the_trial():
    rig = Rig()
    rig.start()
    rig.tick(85)
    rig.applied = 0.02  # the floor permit (say) held it down
    rig.tick(3)
    assert rig.r.result.startswith("void:limited")


def test_disarming_or_a_stop_voids_the_trial():
    rig = Rig()
    rig.start()
    rig.tick(30)
    rig.armed = False
    rig.tick()
    assert rig.r.result == "void:disarmed"


def test_a_drive_that_takes_too_long_is_void():
    rig = Rig()
    rig.start()
    rig.tick(80)
    rig.applied = 0.0001  # crawling: never limited enough to be cut? -- overtime catches it
    rig.r.tolerance_mps = 1.0  # let the speed check pass so only the clock judges
    rig.tick(300)
    assert rig.r.result == "void:overtime"


def test_presteer_that_never_settles_is_void():
    rig = Rig()
    rig.start()
    rig.r._presteer_check = lambda q, target: False  # the measured q never comes within tolerance
    rig.tick(int(dp.PRESTEER_TIMEOUT_S / DT) + 5)
    assert rig.r.result == "void:presteer_timeout"


def test_a_max_steering_trial_whose_q_falls_short_is_marked_under_reached():
    rig = Rig()
    rig.start()
    rig.r._presteer_check = lambda q, target: True
    real_step = rig.r.step
    rig.r.step = lambda t, obs: real_step(t, obs._replace(meas_q=LIM.q_of_kappa(0.36) - 0.05))
    rig.tick(300)
    assert rig.r.result == "done" and rig.r.under_reached == ["arc"]


def test_only_the_accepted_deadman_releases_the_zeros_never_a_messages_own():
    rig = Rig()
    rig.start()
    rig.tick(300)
    assert rig.r.state == "zeroing"
    # an observer's message with the deadman off: rewritten to zeros, and the runner is not released by it
    out = rig.r.rewrite({"t": "teleop", "deadman": False, "speed_ratio": 1.0, "curvature_ratio": 1.0})
    assert out["speed_ratio"] == 0.0
    rig.tick()  # the accepted command still holds the deadman
    assert rig.r.state == "zeroing"


def test_the_drop_to_zero_is_flagged_at_the_end_of_a_drive_and_on_a_void():
    rig = Rig()
    rig.start()
    flagged = []
    for _ in range(300):
        rig.tick()
        if rig.r.zero_now:
            flagged.append((rig.r.state, rig.r.segment.label if rig.r.segment else None))
            rig.r.zero_now = False
    assert flagged[0] == ("running", "still_post") and flagged[-1][0] == "zeroing"


@pytest.mark.parametrize("bad", [
    {"segments": ["not a mapping"]},
    {"segments": [{"label": "p", "presteer_kappa_inv_m": 0.0},
                  {"label": "m", "speed_mps": 0.1, "s_merge": {"offset_m": 0.02, "length_m": 0}}]},
    {"segments": [{"label": "p", "presteer_kappa_inv_m": 0.0},
                  {"label": "m", "speed_mps": 0.1, "s_merge": {"offset_m": float("nan"), "length_m": 0.9}}]},
    {"segments": [{"label": "p", "presteer_kappa_inv_m": 0.0},
                  {"label": "d", "speed_mps": "fast", "kappa_inv_m": 0.0, "distance_m": 1}]},
    {"segments": [{"label": "p", "presteer_kappa_inv_m": 0.0}, {"label": "d", "speed_mps": 0.1}]},
])
def test_a_malformed_file_is_one_kind_of_refusal(bad):
    with pytest.raises(ValueError):
        dp.parse_profile({"schema": dp.SCHEMA, "load": "empty", **bad}, LIM)


def test_a_negative_merge_offset_merges_to_the_other_side():
    p = dp.parse_profile(prof({"label": "p", "presteer_kappa_inv_m": 0.0},
                              {"label": "m", "speed_mps": 0.10, "s_merge": {"offset_m": -0.02, "length_m": 0.9}}), LIM)
    assert p.segments[1].peak_kappa < 0

