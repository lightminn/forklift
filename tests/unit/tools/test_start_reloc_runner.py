"""B-P2b: the runner's start relocalization (docs/plans/2026-10-29-start-relocalization.md 1.1, 1.2, 1.4) --
``--start-pose`` / ``--start-relocalize`` / ``--reloc-dir`` (off by default, not recorded off, in the near-field and
replay contracts), where the truck is placed and where the odometry, the obstacle layer and the bridge's start check
begin, the phase's failures as session failures, and that the localization path never reads the placed pose."""

import math
import re
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))
sys.path.insert(0, str(ROOT / "tests/unit/localization"))

import scan_capture as C  # noqa: E402
import test_global_match as TG  # noqa: E402
from test_reloc_phase import FakeTruck  # noqa: E402
from test_riibotics_isaac_scenes import _runner_args  # noqa: E402

from forklift_core.localization import global_match as G  # noqa: E402
from forklift_core.localization import start_reloc as R  # noqa: E402
from forklift_core.localization.slam_map import MapBundle  # noqa: E402

SCRIPT = ROOT / "sim/isaac/run_transport.py"
NEW = ("start_pose", "start_relocalize", "reloc_dir")
ANGLES = (0.0, 2 * math.pi / 360)  # the fake LiDAR's


def _live(tmp_path, *extra):
    return ["--live", "--slam-feedback", str(tmp_path / "s.sock"), "--slam-map-bundle", str(tmp_path),
            "--expected-map-id", "a" * 64, *extra]


def _reloc(tmp_path, pose="1.0,2.0,0.5"):
    return ["--start-pose", pose, "--start-relocalize", "--reloc-dir", str(tmp_path)]


def test_the_start_relocalization_is_off_by_default_and_not_recorded_off(tmp_path):
    import run_transport

    args = _runner_args(_live(tmp_path))
    assert (args.start_pose, args.start_relocalize, args.reloc_dir) == (None, False, None)
    assert not set(NEW) & set(run_transport.recorded_arguments(args))  # an off run records what it always did


def test_a_placed_pose_with_a_negative_x_parses_as_one_word(tmp_path):
    args = _runner_args(_live(tmp_path, "--start-pose=-1.5,2.0,0.5", "--start-relocalize", "--reloc-dir",
                              str(tmp_path)))
    assert args.start_pose == (-1.5, 2.0, 0.5)


@pytest.mark.parametrize("xyz, yaw, ok", [
    ((-0.12, 0.0, 1.05), 0.0, True),
    ((-0.12, 0.0, 0.40), 0.0, True),  # the height is the scan plane's, not a frame offset
    ((-0.10, 0.0, 1.05), 0.0, False),
    ((-0.12, 0.02, 1.05), 0.0, False),
    ((-0.12, 0.0, 1.05), 0.3, False),  # a turned LiDAR: base_from_laser would keep its yaw (Codex B-P2b P2)
])
def test_the_lidar_mount_must_be_start_relocs(xyz, yaw, ok):
    import run_transport

    assert (run_transport.reloc_mount_problem(xyz, yaw) is None) == ok


def test_a_relocalizing_live_run_takes_its_placed_pose(tmp_path):
    import run_transport

    args = _runner_args(_live(tmp_path, *_reloc(tmp_path)))
    assert args.start_pose == (1.0, 2.0, 0.5) and args.start_relocalize and args.reloc_dir == tmp_path
    assert set(NEW) <= set(run_transport.recorded_arguments(args))


@pytest.mark.parametrize("extra", [
    ["--start-pose", "1,2,0.5"],  # the placed pose only with the relocalization
    ["--start-relocalize", "--reloc-dir", "d"],  # no placed pose
    ["--start-pose", "1,2,0.5", "--start-relocalize"],  # no epoch folder
    ["--reloc-dir", "d"],
    ["--start-pose", "1,2", "--start-relocalize", "--reloc-dir", "d"],
    ["--start-pose", "1,2,nan", "--start-relocalize", "--reloc-dir", "d"],
    ["--start-pose", "1,inf,0", "--start-relocalize", "--reloc-dir", "d"],
])
def test_the_start_arguments_are_refused_out_of_place(tmp_path, extra):
    with pytest.raises(SystemExit):
        _runner_args(_live(tmp_path, *extra))


def test_a_relocalizing_run_is_live_on_a_survey_map_with_the_shared_layer(tmp_path):
    for argv in (_reloc(tmp_path),  # not live
                 ["--live", "--slam-feedback", str(tmp_path / "s.sock"), *_reloc(tmp_path)],  # online mapping
                 _live(tmp_path, *_reloc(tmp_path), "--no-layer")):  # no obstacle layer (its LiDAR and noise)
        with pytest.raises(SystemExit):
            _runner_args(argv)


def test_the_near_field_and_replay_contracts_know_the_new_arguments():
    import near_tracking

    from tools import boundary_replay

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["start_pose"] is None
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["start_relocalize"] is False
    assert "reloc_dir" in near_tracking.RUN_SPECIFIC_ARGUMENTS  # the epoch folder is the run's own
    assert boundary_replay.SETUP_NAMES_ADDED["start_reloc"] == ("start_relocalize", False, None)
    # B-P2c: the early console and the operator's options
    assert boundary_replay.SETUP_NAMES_ADDED["reloc_console"] == ("start_relocalize", False, None)
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["reloc_unattended"] is False
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["reloc_operator_timeout_s"] is None


def test_the_operator_options_go_with_the_relocalization(tmp_path):
    import run_transport

    args = _runner_args(_live(tmp_path, *_reloc(tmp_path), "--reloc-unattended", "--reloc-operator-timeout-s", "300"))
    assert args.reloc_unattended is True and args.reloc_operator_timeout_s == 300.0
    assert {"reloc_unattended", "reloc_operator_timeout_s"} <= set(run_transport.recorded_arguments(args))
    off = _runner_args(_live(tmp_path))
    assert (off.reloc_unattended, off.reloc_operator_timeout_s) == (False, None)
    assert not {"reloc_unattended", "reloc_operator_timeout_s"} & set(run_transport.recorded_arguments(off))
    for extra in (["--reloc-unattended"], ["--reloc-operator-timeout-s", "300"],  # without the relocalization
                  [*_reloc(tmp_path), "--reloc-operator-timeout-s", "0"],
                  [*_reloc(tmp_path), "--reloc-operator-timeout-s", "nan"]):
        with pytest.raises(SystemExit):
            _runner_args(_live(tmp_path, *extra))


def test_the_operator_budget_runs_from_the_runners_start():
    import run_transport

    args = SimpleNamespace(reloc_operator_timeout_s=None)
    assert run_transport.operator_deadline(args, started=100.0) == 100.0 + run_transport.RELOC_OPERATOR_TIMEOUT_S
    assert run_transport.RELOC_OPERATOR_TIMEOUT_S < 600.0  # inside the supervisor's whole wait (RELOC_WAIT_S)
    assert run_transport.operator_deadline(SimpleNamespace(reloc_operator_timeout_s=30.0), started=5.0) == 35.0


def test_the_early_consoles_arbiter_is_the_live_runtimes():
    import run_transport

    args = SimpleNamespace(live_factor=0.4, live_lease_timeout_s=1.0, live_camera_latency_bound_s=None,
                           live_camera_pixel_lag_s=0.05)
    a = run_transport.live_arbiter_config(args, lift_limit_m=0.28, catchup_steps=2)
    assert a == run_transport.live_arbiter_config(args, lift_limit_m=0.28, catchup_steps=2)
    assert a.lift_limit_m == 0.28 and a.command_max_age_s == pytest.approx((0.30 - 2 / 120) / 0.4)
    drive_ = _source("\ndef drive(S, isaac)", "\ndef recorded_arguments(")
    assert "config=live_arbiter_config(args, lift_limit_m=live_lift_limit_, catchup_steps=live_catchup_)" in drive_


class _SlowLink:
    def __init__(self, result=None, error=None, delay=0.05):
        self.result, self.error, self.delay, self.closed, self.sock = result, error, delay, False, "socket"

    def exchange(self, scan):
        import time

        time.sleep(self.delay)
        if self.error is not None:
            raise self.error
        return (self.result, scan)

    def close(self):
        self.closed = True


def test_the_bootstrap_exchange_waits_with_the_console_serviced():
    # Codex B-P2c design P2: a blocking bridge reply must not stop the console (STOP, lease, state)
    import run_transport

    stages = []
    link = run_transport.ServicedExchange(_SlowLink("reply"), lambda stage: stages.append(stage), tick_s=0.005)
    assert link.exchange("scan-1") == ("reply", "scan-1")
    assert len(stages) >= 3 and set(stages) == {"prewarm_exchange"}
    assert link.sock == "socket"  # what the async link takes over afterwards
    link.close()
    assert link.inner.closed


def test_an_exchange_failure_and_a_session_end_both_leave_the_exchange():
    import run_transport

    failing = run_transport.ServicedExchange(_SlowLink(error=RuntimeError("bridge")), lambda stage: None, tick_s=0.005)
    with pytest.raises(RuntimeError, match="bridge"):
        failing.exchange("scan")

    class End(Exception):
        pass

    def end(stage):
        raise End()

    with pytest.raises(End):
        run_transport.ServicedExchange(_SlowLink("reply", delay=0.2), end, tick_s=0.005).exchange("scan")


def test_an_operator_end_before_the_localization_is_the_sessions_end(monkeypatch):
    import reloc_console
    import run_transport

    console = SimpleNamespace(ended_by_operator=False,
                              parts=SimpleNamespace(arbiter=SimpleNamespace(session_request=None)))
    monkeypatch.setitem(run_transport._EARLY, "console", console)
    assert run_transport.operator_ended({}, reloc_console.RelocSessionEnd("x")) is True
    assert run_transport.operator_ended({"operator_ended": True}, RuntimeError("slam reply timeout")) is True
    assert run_transport.operator_ended({}, RuntimeError("slam reply timeout")) is False
    # Codex B-P2c impl review P2: an accepted end counts even when what followed it failed before the flag was set
    console.parts.arbiter.session_request = "end"
    assert run_transport.operator_ended({}, OSError("port file")) is True


def test_a_run_without_the_early_console_never_imports_it(monkeypatch):
    # Codex B-P2c impl review P2: a batch failure must not meet a missing console dependency
    import run_transport

    monkeypatch.setitem(run_transport._EARLY, "console", None)
    monkeypatch.setitem(sys.modules, "reloc_console", None)  # an import would raise
    assert run_transport.operator_ended({}, RuntimeError("anything")) is False


def test_a_session_end_while_the_bootstrap_waits_closes_the_link_and_ends_the_helper():
    # Codex B-P2c impl review P2: the helper is ended on every way out, not left blocked in its socket
    import threading

    import run_transport

    class Blocking:
        def __init__(self):
            self.released = threading.Event()

        def exchange(self, scan):
            self.released.wait(10.0)
            raise ConnectionError("closed")

        def close(self):
            self.released.set()

    class End(Exception):
        pass

    calls = {"n": 0}

    def service(stage):
        calls["n"] += 1
        if calls["n"] == 3:
            raise End()

    inner = Blocking()
    with pytest.raises(End):
        run_transport.ServicedExchange(inner, service, tick_s=0.005).exchange("scan")
    assert inner.released.is_set()
    assert not [t for t in threading.enumerate() if t.name == "slam-bootstrap-exchange"]


def test_a_real_blocking_read_is_ended_by_shutting_its_socket_down():
    # Codex B-P2c impl review round 2 P2: closing a socket does not wake a recv blocked on it in another thread
    import socket
    import threading
    import time

    import run_transport

    a, b = socket.socketpair()

    class Client:  # as slam_link.SlamLinkClient: a recv on its socket, close() closes it
        def __init__(self, sock):
            self.sock = sock

        def exchange(self, scan):
            data = self.sock.recv(16)
            if not data:
                raise ConnectionError("closed")
            return data

        def close(self):
            self.sock.close()

    class End(Exception):
        pass

    calls = {"n": 0}

    def service(stage):
        calls["n"] += 1
        if calls["n"] == 3:
            raise End()

    t0 = time.monotonic()
    link = run_transport.ServicedExchange(Client(a), service, tick_s=0.005)
    with pytest.raises(End):
        link.exchange("scan")
    assert time.monotonic() - t0 < 1.0
    assert link.cleanup == {"shutdown_error": None, "helper_alive": False}  # recorded either way (Codex P3)
    assert not [t for t in threading.enumerate() if t.name == "slam-bootstrap-exchange"]
    b.close()


def test_an_operator_end_wins_over_a_failing_diagnostic_write():
    # Codex B-P2c impl review round 2 P2: the end is decided before failure.txt is written
    main_ = _source("\ndef main() -> None:", '\nif __name__ == "__main__":')
    except_ = main_[main_.index("    except BaseException as exc:"):main_.index("    finally:")]
    assert except_.index("operator_ended(state, exc)") < except_.index('"failure.txt"')
    assert re.search(r"try:\n\s+\(Path\(str\(args\.output\)\) / \"failure\.txt\"\)\.write_text", except_)


def test_the_prewarm_gates_before_each_step_and_gives_the_raw_link_back_on_every_path():
    drive_ = _source("\ndef drive(S, isaac)", "\ndef recorded_arguments(")
    loop = drive_[drive_.index('slam["link"] = ServicedExchange('):drive_.index('raise SessionFailure("slam_prewarm_timeout")')]
    assert loop.index('reloc_console.gate("prewarm")') < loop.index("step_world(False)")
    assert re.search(r"finally:\n\s+if reloc_console is not None:\n\s+slam\[\"link\"\] = slam_lockstep_", drive_)


def test_the_gaps_are_kept_from_the_consoles_opening():
    # Codex B-P2c impl review P3: a phase that fails still leaves its gaps in the record
    helper = _source("\ndef open_reloc_console(", "\ndef shared_lidar_engine(")
    assert '"gaps": console.gaps' in helper


def test_the_early_console_opens_before_the_phase_and_the_live_runtime_adopts_it():
    run_ = _source("\ndef run(app,", "\ndef drive(S, isaac)")
    drive_ = _source("\ndef drive(S, isaac)", "\ndef recorded_arguments(")
    opened = run_.index("reloc_console = open_reloc_console(")
    assert opened < run_.index("start_reloc = start_relocalization(")
    assert re.search(r"\n(\s+)reloc_console = None[^\n]*\n\1if args\.start_relocalize:\n", run_)
    assert '**({} if reloc_console is None else {"parts": reloc_console.parts})' in drive_
    # the warm-up serviced, the localization recorded in the port file, then the adoption
    assert drive_.index("ServicedExchange(") < drive_.index('raise SessionFailure("slam_prewarm_timeout")')
    assert drive_.index('live.port_extra["localized_wall"]') < drive_.index("live.start()")


def _scenario(rear=(0.5, -1.0, 0.3)):
    return SimpleNamespace(start_rear=SimpleNamespace(x_m=rear[0], y_m=rear[1], yaw_rad=rear[2]))


def test_the_truck_is_placed_at_the_start_pose_or_else_at_the_scenes_start():
    import run_transport

    scenario = _scenario()
    off = SimpleNamespace(start_pose=None, rear_axle_offset_m=-0.34)
    x, y, yaw = run_transport.spawn_base_pose(off, scenario)
    assert (x, y, yaw) == pytest.approx((0.5 + 0.34 * math.cos(0.3), -1.0 + 0.34 * math.sin(0.3), 0.3))
    on = SimpleNamespace(start_pose=(4.0, 3.0, -2.0), rear_axle_offset_m=-0.34)
    assert run_transport.spawn_base_pose(on, scenario) == (4.0, 3.0, -2.0)


def test_the_map_start_is_the_estimate_of_a_relocalization_or_else_the_scenes_start():
    import run_transport

    scenario = _scenario()
    assert run_transport.map_start_rear(scenario, None) == (0.5, -1.0, 0.3)
    assert run_transport.map_start_rear(scenario, {"estimate_rear": [7.0, 8.0, 1.0]}) == (7.0, 8.0, 1.0)


def _source(start: str, end: str) -> str:
    text = SCRIPT.read_text()
    return text[text.index(start):text.index(end)]


def test_the_localization_path_never_reads_the_placed_pose():
    run_ = _source("\ndef run(app,", "\ndef drive(S, isaac)")
    drive_ = _source("\ndef drive(S, isaac)", "\ndef recorded_arguments(")
    assert "start_pose" not in drive_.replace("map_start_pose", "")  # the drive starts from map_start_rear only
    assert len(re.findall(r"args\.start_pose\b", run_)) == 0  # the setup places by spawn_base_pose...
    helper = _source("\ndef start_relocalization(", "\ndef run(app,")
    assert len(re.findall(r"args\.start_pose\b", helper)) == 1  # ...and the phase reads it for validity only
    assert "target=args.start_pose" in helper.replace(" ", "")


def test_the_setup_and_the_drive_start_from_map_start_rear():
    run_ = _source("\ndef run(app,", "\ndef drive(S, isaac)")
    drive_ = _source("\ndef drive(S, isaac)", "\ndef recorded_arguments(")
    assert "start_pose=map_start_rear(scenario,start_reloc)" in run_.replace(" ", "")  # the obstacle layer's prime
    assert "odom_rear=start_pose," in run_.replace(" ", "")
    assert "slam_start=map_start_rear(scenario,start_reloc)" in drive_.replace(" ", "")  # odometry, start check
    assert "base_start=spawn_base_pose(args, scenario)" not in run_  # (it builds the array from it)
    assert "spawn_base_pose(args, scenario)" in run_
    # the verdict is taken when every warm-up is done, before the server opens (Codex B-P2b review P2)
    prewarm = drive_.index('raise SessionFailure("slam_prewarm_timeout")')
    verdict = drive_.index('start_reloc["verdict"] = {')
    assert prewarm < verdict < drive_.index("live.start()")
    # every new call of the setup is behind the option (plan 1.4): the phase runs only when it is on
    assert re.search(r"\n(\s+)if args\.start_relocalize:\n(?:\1    [^\n]*\n|\1        [^\n]*\n)*?\1    start_reloc = "
                     r"start_relocalization\(", run_)


def _bundle(grid, mfw=(0.0, 0.0, 0.0)):
    return MapBundle(folder=Path("b"), map_id="m" * 64, manifest={"map_from_world": list(mfw)}, grid=grid)


def _phase_args(tmp_path, base):
    return SimpleNamespace(start_pose=base, reloc_dir=tmp_path, seed=7, rear_axle_offset_m=-0.34)


def _engine(truck):
    return C.Engine(teleport=None, step=truck.step, pose=truck.pose, cast=truck.cast,
                    draw_noise=lambda rng, d: np.zeros(len(d)), ranges_from_hits=lambda d, h: d,
                    range_min_m=0.2, range_max_m=12.0, settle_z_m=0.1, laser_xyz_m=(-0.12, 0.0, 1.05))


def _frozen():
    return (G.MatchConfig(yaw_step_rad=math.radians(2.0)), R.RelocConfig(e_min=0.8, r_max=0.9, r_struct=0.8),
            {"match_config": {"sha256": "x"}})


def test_a_chosen_estimate_is_the_runs_map_start(tmp_path):
    import run_transport
    from test_reloc_phase import _go_later

    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    _go_later(tmp_path)
    state = {}
    rec = run_transport.start_relocalization(_phase_args(tmp_path, base), state, _engine(FakeTruck(grid, base)),
                                             ANGLES, _bundle(grid), frozen=_frozen(), pace_s=0.0)
    assert rec is state["start_reloc"] and rec["status"] == "chosen" and rec["map_id"] == "m" * 64
    est = rec["estimate"]
    assert math.hypot(est[0] - base[0], est[1] - base[1]) <= 0.10
    assert rec["estimate_rear"] == pytest.approx(list(R.rear_from_base(est, 0.34)))
    assert rec["settings"] == {"match_config": {"sha256": "x"}}


def test_a_start_that_did_not_settle_ends_the_session_before_any_matching(tmp_path):
    import run_transport

    grid = TG._grid(*TG.ROOM)
    base = (2.42, 1.7, 0.6)
    state = {}
    with pytest.raises(run_transport.SessionFailure, match="^reloc_invalid_input: not_at_target"):
        run_transport.start_relocalization(_phase_args(tmp_path, base), state,
                                           _engine(FakeTruck(grid, base, drift=(0.05, 0.0))), ANGLES, _bundle(grid),
                                           frozen=_frozen(), pace_s=0.0)
    assert state["start_reloc"]["status"] == "invalid" and state["start_reloc"]["attempts"] == []


def test_a_bundle_whose_map_is_not_the_world_is_refused(tmp_path):
    import run_transport

    grid = TG._grid(*TG.ROOM)
    with pytest.raises(run_transport.SessionFailure, match="^reloc_config_error: .*map_from_world"):
        run_transport.start_relocalization(_phase_args(tmp_path, (2.42, 1.7, 0.6)), {},
                                           _engine(FakeTruck(grid, (2.42, 1.7, 0.6))), ANGLES,
                                           _bundle(grid, (0.1, 0, 0)),
                                           frozen=_frozen(), pace_s=0.0)


def test_each_attempt_has_its_own_seeded_noise():
    import run_transport

    a = run_transport.reloc_noise(7)(0, 1).standard_normal(3)
    assert np.array_equal(a, run_transport.reloc_noise(7)(0, 1).standard_normal(3))  # reproducible per run seed
    assert not np.array_equal(a, run_transport.reloc_noise(7)(1, 1).standard_normal(3))
    assert not np.array_equal(a, run_transport.reloc_noise(8)(0, 1).standard_normal(3))
