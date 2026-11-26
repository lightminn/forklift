"""PR 7c-1a (docs/design/2026-11-25-real-sources-pr7.md 6.1, 2): the real-mode bridge's frames and its state machine
without ROS -- one session, an instance per start or seed with the existing L4 start check (a /pose for start scans
1, 5, 6, 7; the map load; scans on the walls near the start), end states only before the deadline, one scan in
flight, cancel to void, messages of an earlier instance ignored, protocol violations ending the session."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from forklift_core.localization import slam_link, slam_map
from forklift_core.localization import slam_real_link as rl
from forklift_core.localization.real_bridge_core import WARMUP_S, BridgeCore

MS = 1_000_000
RES = 0.05


def room() -> slam_map.Grid:
    data = np.zeros((100, 100), np.int8)
    data[0, :] = data[-1, :] = data[:, 0] = data[:, -1] = 100
    data[40:60, 30] = 100  # a feature, so poses are not symmetric
    return slam_map.Grid(data, (0.0, 0.0, 0.0), RES)


GRID = room()
BUNDLE = SimpleNamespace(map_id="m1", grid=GRID)
A = (2.5, 2.5, 0.0)
B = (2.0, 2.2, 0.3)


def ranges_at(pose, beams=360):
    x, y, yaw = pose
    angles = yaw - math.pi + np.arange(beams) * (2 * math.pi / beams)
    steps = np.arange(0.05, 8.0, RES / 2)
    px = x + np.cos(angles)[:, None] * steps[None, :]
    py = y + np.sin(angles)[:, None] * steps[None, :]
    rows, cols, inside = GRID.cells_of(np.column_stack((px.ravel(), py.ravel())))
    occ = np.zeros(rows.shape, bool)
    occ[inside] = GRID.occupied()[rows[inside], cols[inside]]
    occ = occ.reshape(beams, len(steps))
    return np.where(occ.any(axis=1), steps[occ.argmax(axis=1)], np.inf).astype(np.float32)


def scan_at(sid, pose, stamp_ns):
    return slam_link.Scan(sid, stamp_ns / 1e9, (0.0, 0.0, 0.0), ranges_at(pose), -math.pi, 2 * math.pi / 360, 0.1,
                          8.0)


class IO:
    def __init__(self):
        self.frames, self.started, self.stopped, self.published = [], [], [], []
        self.ns = 1_000 * MS
        self.stop_ok = True

    def send(self, frame):
        self.frames.append(rl.decode(frame[4:]))

    def start_instance(self, k, pose):
        self.started.append((k, pose))

    def stop_instance(self, k):
        self.stopped.append(k)
        return self.stop_ok

    def publish(self, k, scan, stamp_ns):
        self.published.append((k, scan.scan_id, stamp_ns))

    def now_ns(self):
        return self.ns

    def states(self):
        return [(f.id, f.kind) for f in self.frames if isinstance(f, rl.State)]

    def replies(self):
        return [(f.scan_id, f.status) for f in self.frames if isinstance(f, slam_link.Reply)]


def hello(deadline_ms=21_000, start=A, map_id="m1"):
    return rl.Hello(7, map_id, 1, start, deadline_ms * MS)


def bootstrap(core, io, k, pose_given, pose_true=None, *, with_map=True, map_grid=GRID, sid0=0):
    """Seven start scans of the scene at ``pose_true``; slam_toolbox answers 1, 5, 6, 7 at ``pose_given``."""
    pose_true = pose_given if pose_true is None else pose_true
    core.on_instance_ready(k)
    for i in range(1, 8):
        io.ns += 50 * MS
        core.on_frame(rl.RealScan(scan_at(sid0 + i, pose_true, io.ns), io.ns))
        if i in (1, 5, 6, 7):
            core.on_pose(k, io.ns, pose_given)
            if i == 1 and with_map:
                core.on_map(k, map_grid)
        else:
            io.ns += int(WARMUP_S * 1e9)
            core.tick()


# --- frames -----------------------------------------------------------------------------------------------------------
def test_frames_round_trip():
    for obj, enc in ((hello(), rl.encode_hello), (rl.Seed(3, 2, B, (0.1, 0.0, 0.0), 40, 5 * MS), rl.encode_seed),
                     (rl.Cancel(3, "boot_changed"), rl.encode_cancel), (rl.State(3, "void", "boot"), rl.encode_state)):
        assert rl.decode(enc(obj)[4:]) == obj
    s = scan_at(4, A, 1_234_567_891)
    back = rl.decode(rl.encode_scan(s, 1_234_567_891)[4:])
    assert back.stamp_ns == 1_234_567_891 and back.scan.scan_id == 4 and np.array_equal(back.scan.ranges_m, s.ranges_m)


@pytest.mark.parametrize("body", [b'[]\n', b'{"type": "hello", "session_epoch": 1, "map_id": "m", "start_id": 1, '
                                  b'"start_pose": null, "deadline_ns": 1}\n',
                                  b'{"type": "hello", "session_epoch": 1.5, "map_id": "m", "start_id": 1, '
                                  b'"start_pose": [0, 0, 0], "deadline_ns": 1}\n',
                                  b'{"type": "seed", "seed_id": 1}\n', b'{"type": "nope"}\n', b'not json\n',
                                  b'{"type": "state", "id": 1, "kind": "maybe"}\n'])
def test_malformed_frames_are_refused(body):
    with pytest.raises(ValueError):
        rl.decode(body)


# --- the start check ---------------------------------------------------------------------------------------------------
def test_a_start_goes_seeding_ready_for_scans_and_ready():
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello())
    assert io.started == [(1, A)] and io.states() == [(1, "seeding")]
    bootstrap(core, io, 1, A)
    assert io.states() == [(1, "seeding"), (1, "ready_for_scans"), (1, "ready")]
    assert [s for _, s in io.replies()] == ["processed", "skipped", "skipped", "skipped", "processed", "processed",
                                            "processed"]
    io.ns += 100 * MS
    core.on_frame(rl.RealScan(scan_at(8, A, io.ns), io.ns))
    core.on_pose(1, io.ns, A)
    last = [f for f in io.frames if isinstance(f, slam_link.Reply)][-1]
    assert last.status == "processed" and last.map_id == "m1"  # the map id rides on replies once ready


def test_scans_off_the_walls_are_not_localized_and_no_fault():
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello())
    bootstrap(core, io, 1, A, pose_true=(1.2, 3.6, 1.0))  # slam answers at A, the scans were taken elsewhere
    assert io.states()[-1] == (1, "not_localized") and not core.closed


def test_a_map_after_the_last_pose_is_waited_for_within_the_deadline():
    """Codex 7c-1a round 1 P2-1: the 7th /pose before the /map -- the check waits for it."""
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello())
    bootstrap(core, io, 1, A, with_map=False)
    assert io.states()[-1] == (1, "ready_for_scans") and not core.closed
    io.ns += 50 * MS
    core.on_map(1, GRID)
    assert io.states()[-1] == (1, "ready")


def test_no_map_by_the_deadline_is_no_end_state():
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello(deadline_ms=3_000))
    bootstrap(core, io, 1, A, with_map=False)
    io.ns = 3_001 * MS
    core.tick()
    core.on_map(1, GRID)
    assert io.states()[-1] == (1, "ready_for_scans") and core.cur["phase"] == "expired"


def test_a_thin_map_is_a_fault():
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello())
    bootstrap(core, io, 1, A, map_grid=slam_map.Grid(np.zeros((100, 100), np.int8), (0.0, 0.0, 0.0), RES))
    assert io.states()[-1][1] == "fault" and "NOT_LOADED" in io.frames[-1].reason.upper()


def test_no_end_state_past_the_deadline():
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello(deadline_ms=1_100))  # the seven scans take longer
    bootstrap(core, io, 1, A)
    assert io.states() == [(1, "seeding"), (1, "ready_for_scans")] and not core.closed


@pytest.mark.parametrize("first, second", [
    (lambda: hello(map_id="other"), None),
    (hello, hello),
    (lambda: rl.Seed(2, 1, B, (0.0, 0.0, 0.0), 1, 9 * 10**12), None),
])
def test_session_rules(first, second):
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(first())
    if second is not None:
        core.on_frame(second())
    assert io.states()[-1][1] == "fault" and core.closed


# --- seeds, cancels, stale instances ------------------------------------------------------------------------------------
def ready_core():
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello())
    bootstrap(core, io, 1, A)
    return io, core


def test_a_seed_stops_the_instance_and_starts_the_next_at_its_pose():
    io, core = ready_core()
    core.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, io.ns + 20_000 * MS))
    assert io.stopped == [1] and io.started[-1] == (2, B) and io.states()[-1] == (5, "seeding")
    core.on_pose(1, io.ns, A)  # the old instance's late message
    core.on_map(1, GRID)
    assert not core.closed and core.counts["stale_instance"] == 1 and core.counts["late_maps"] == 1
    bootstrap(core, io, 2, B, sid0=100)
    assert io.states()[-1] == (5, "ready")


def test_a_seed_during_a_start_or_with_a_scan_in_flight_is_a_protocol_fault():
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello())
    core.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, 9 * 10**12))
    assert core.closed
    io, core = ready_core()
    io.ns += MS
    core.on_frame(rl.RealScan(scan_at(50, A, io.ns), io.ns))
    core.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, 9 * 10**12))
    assert core.closed


def test_a_cancel_with_a_scan_in_flight_voids_the_seed_and_late_messages_change_nothing():
    """Start scan 5 sent, cancelled before its /pose: void, the slot free, the late /pose only counted."""
    io, core = ready_core()
    core.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, io.ns + 20_000 * MS))
    core.on_instance_ready(2)
    for i in range(1, 6):
        io.ns += 50 * MS
        core.on_frame(rl.RealScan(scan_at(100 + i, B, io.ns), io.ns))
        if i == 1:
            core.on_pose(2, io.ns, B)
        elif i < 5:
            io.ns += int(WARMUP_S * 1e9)
            core.tick()
    core.on_frame(rl.Cancel(5, "boot_changed"))
    assert io.stopped == [1, 2] and io.states()[-1] == (5, "void") and core.awaited is None
    core.on_pose(2, io.ns, B)
    assert not core.closed and core.counts["stale_instance"] == 1
    core.on_frame(rl.Seed(6, 3, A, (0.0, 0.0, 0.0), 2, io.ns + 20_000 * MS))  # the next seed proceeds
    assert io.started[-1] == (3, A) and io.states()[-1] == (6, "seeding")


def test_a_cancel_of_a_ready_seed_still_voids_it():
    io, core = ready_core()
    core.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, io.ns + 20_000 * MS))
    bootstrap(core, io, 2, B, sid0=100)
    core.on_frame(rl.Cancel(5, "moved"))
    assert io.states()[-1] == (5, "void") and io.stopped == [1, 2]


@pytest.mark.parametrize("cancel", [rl.Cancel(9, "x"), rl.Cancel(1, "x")])
def test_a_cancel_of_anything_but_the_current_seed_is_a_protocol_fault(cancel):
    io, core = ready_core()
    core.on_frame(cancel)
    assert core.closed


def test_a_stray_pose_of_the_current_instance_is_a_protocol_fault():
    io, core = ready_core()
    io.ns += MS
    core.on_frame(rl.RealScan(scan_at(50, A, io.ns), io.ns))
    core.on_pose(1, io.ns - 1, A)
    assert core.closed and io.states()[-1][1] == "fault"


def test_scan_order_rules():
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello())
    core.on_frame(rl.RealScan(scan_at(1, A, io.ns), io.ns))  # before ready_for_scans
    assert core.closed
    io, core = ready_core()
    core.on_frame(rl.RealScan(scan_at(50, A, io.ns), io.ns))  # not after the last stamp
    assert core.closed


def test_a_stop_that_does_not_complete_and_a_failed_instance_are_faults():
    io, core = ready_core()
    io.stop_ok = False
    core.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, io.ns + 20_000 * MS))
    assert core.closed and "stopped" in io.frames[-1].reason
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello(deadline_ms=2_000))
    core.on_instance_failed(1, "lifecycle configure failed")
    io.ns = 3_000 * MS
    core.tick()  # no end state of its own: the executor's deadline ends it (round 3 P2-1)
    assert not core.closed and core.cur["phase"] == "failed" and "lifecycle" in core.failures[-1]["why"]


# --- Codex 7c-1a round 1 ----------------------------------------------------------------------------------------------
def test_the_deadline_is_read_again_after_the_map_check(monkeypatch):
    """P2-2: the check starts 1 ms before the deadline and ends 1 ms after it -- no ready."""
    import forklift_core.localization.real_bridge_core as core_mod

    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello())
    real = core_mod.slam_map.map_coverage

    def slow(a, b):
        io.ns = core.cur["deadline_ns"] + MS
        return real(a, b)

    monkeypatch.setattr(core_mod.slam_map, "map_coverage", slow)
    bootstrap(core, io, 1, A)
    assert io.states()[-1] == (1, "ready_for_scans") and core.cur["phase"] == "expired"


def test_a_seed_after_an_expired_start_check_is_refused_but_its_cancel_voids_it():
    io, core = ready_core()
    core.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, io.ns + 2_000 * MS))
    core.on_instance_ready(2)
    io.ns += 3_000 * MS
    for i in range(1, 8):
        io.ns += 50 * MS
        core.on_frame(rl.RealScan(scan_at(200 + i, B, io.ns), io.ns))
        if i in (1, 5, 6, 7):
            core.on_pose(2, io.ns, B)
            if i == 1:
                core.on_map(2, GRID)
        else:
            io.ns += int(WARMUP_S * 1e9)
            core.tick()
    assert core.cur["phase"] == "expired"
    core.on_frame(rl.Cancel(5, "deadline"))
    assert io.states()[-1] == (5, "void")
    io2, core2 = ready_core()
    core2.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, io2.ns - 1))  # already past
    core2.on_instance_ready(2)
    bootstrap(core2, io2, 2, B, sid0=300)
    core2.on_frame(rl.Seed(6, 3, A, (0.0, 0.0, 0.0), 1, io2.ns + 20_000 * MS))
    assert core2.closed


def test_wants_map_until_adopted():
    io = IO()
    core = BridgeCore(io, BUNDLE, (0.0, 0.0, 0.0, 0.0))
    core.on_frame(hello())
    assert core.wants_map(1)
    bootstrap(core, io, 1, A)
    assert not core.wants_map(1) and not core.wants_map(2)


# --- Codex 7c-1a round 2 ----------------------------------------------------------------------------------------------
def test_a_bring_up_failure_waits_for_a_cancel_until_the_deadline():
    """P2-2: a lifecycle failure at 20.15 s of a seed whose cancel was decided at 19.9 s: the cancel still voids it."""
    io, core = ready_core()
    core.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, io.ns + 20_000 * MS))
    io.ns += 20_150 * MS
    core.on_instance_failed(2, "lifecycle get timed out")
    assert not core.closed
    core.on_frame(rl.Cancel(5, "boot_changed"))
    assert io.states()[-1] == (5, "void")


def test_a_bring_up_failure_never_ends_the_start_on_its_own():
    """Round 3 P2-1: failure at 20.15 s, a tick at 20.16 s, the cancel decided at 19.9 s lands at 20.4 s -- void."""
    io, core = ready_core()
    core.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, io.ns + 20_000 * MS))
    t0 = io.ns
    io.ns = t0 + 20_150 * MS
    core.on_instance_failed(2, "lifecycle get timed out")
    io.ns = t0 + 20_160 * MS
    core.tick()
    assert not core.closed and core.failures[-1]["why"] == "lifecycle get timed out"
    io.ns = t0 + 20_400 * MS
    core.on_frame(rl.Cancel(5, "boot_changed"))
    assert io.states()[-1] == (5, "void")


def test_a_string_pose_is_no_pose():
    body = (b'{"type": "hello", "session_epoch": 1, "map_id": "m", "start_id": 1, "start_pose": "123", '
            b'"deadline_ns": 1}\n')
    with pytest.raises(ValueError):
        rl.decode(body)


# --- one calibration (plan 2.3, unlock U2b) -----------------------------------------------------------------------------
LASER = (0.0, 0.0, 0.25, 0.0)  # in the plane of the scene's casts (the L4 check reads the laser pose)


def test_a_hello_with_the_bridges_laser_gets_laser_checked_on_every_ready():
    io = IO()
    core = BridgeCore(io, BUNDLE, LASER)
    sent = (-0.0005, 0.0004, 0.2509, math.radians(0.09))  # within 1 mm / 0.1 deg
    core.on_frame(rl.Hello(7, "m1", 1, A, 21_000 * MS, sent))
    bootstrap(core, io, 1, A)
    ready = [f for f in io.frames if isinstance(f, rl.State) and f.kind == "ready"]
    assert ready and ready[-1].laser_checked == rl.laser_digest(sent)
    assert all(f.laser_checked is None for f in io.frames if isinstance(f, rl.State) and f.kind != "ready")
    core.on_frame(rl.Seed(5, 2, B, (0.0, 0.0, 0.0), 1, io.ns + 20_000 * MS))
    bootstrap(core, io, 2, B, sid0=100)
    last = [f for f in io.frames if isinstance(f, rl.State)][-1]
    assert (last.id, last.kind, last.laser_checked) == (5, "ready", rl.laser_digest(sent))


@pytest.mark.parametrize("sent", [(0.0011, 0.0, 0.25, 0.0), (0.0, -0.0011, 0.25, 0.0), (0.0, 0.0, 0.2489, 0.0),
                                  (0.0, 0.0, 0.25, math.radians(0.11)), (0.0, 0.0, 0.25, math.tau + 0.01)])
def test_a_hello_with_another_laser_is_a_fault(sent):
    io = IO()
    core = BridgeCore(io, BUNDLE, LASER)
    core.on_frame(rl.Hello(7, "m1", 1, A, 21_000 * MS, sent))
    assert core.closed and io.started == []
    [st] = [f for f in io.frames if isinstance(f, rl.State)]
    assert st.kind == "fault" and st.reason.startswith("laser_mismatch")


def test_a_full_turn_of_yaw_is_the_same_laser():
    io = IO()
    core = BridgeCore(io, BUNDLE, LASER)
    core.on_frame(rl.Hello(7, "m1", 1, A, 21_000 * MS, (0.0, 0.0, 0.25, math.tau)))
    assert not core.closed


def test_a_hello_without_the_field_asks_nothing_and_gets_no_laser_checked():
    """The HIL and observe runs' hello: no comparison, no laser_checked (their executors require none)."""
    io = IO()
    core = BridgeCore(io, BUNDLE, LASER)
    core.on_frame(hello())
    bootstrap(core, io, 1, A)
    assert io.states()[-1] == (1, "ready")
    assert all(f.laser_checked is None for f in io.frames if isinstance(f, rl.State))


def test_the_laser_fields_round_trip_and_malformed_ones_are_refused():
    h = rl.Hello(7, "m1", 1, A, 5, (-0.34, 0.0, 0.25, 0.1))
    assert rl.decode(rl.encode_hello(h)[4:]) == h
    st = rl.State(1, "ready", None, "a" * 64)
    assert rl.decode(rl.encode_state(st)[4:]) == st
    for body in (b'{"type": "hello", "session_epoch": 1, "map_id": "m", "start_id": 1, "start_pose": [0, 0, 0], '
                 b'"deadline_ns": 1, "base_from_laser": [0, 0, 0]}\n',
                 b'{"type": "hello", "session_epoch": 1, "map_id": "m", "start_id": 1, "start_pose": [0, 0, 0], '
                 b'"deadline_ns": 1, "base_from_laser": [0, 0, 0, "x"]}\n',
                 b'{"type": "state", "id": 1, "kind": "ready", "laser_checked": "short"}\n',
                 b'{"type": "state", "id": 1, "kind": "ready", "laser_checked": 5}\n'):
        with pytest.raises(ValueError):
            rl.decode(body)
