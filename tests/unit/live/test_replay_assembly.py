"""PR 7 7r-2 (docs/plans/2026-11-26-7r-replay-run.md 4): the replay run's preparation -- the configs, the record, the
map, the start and the laser checked before anything starts; the layer's LiDAR from the LiDAR config in the replay
only; the output marks; the runner's command line refusals; the run's EOF, drain and read-failure step."""

from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.live import real_run as rr
from forklift_core.live import replay_assembly as rpa
from forklift_core.live.real_assembly import AssemblyRefused

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tests" / "fixtures" / "replay"))
import make_record as mk  # noqa: E402
import run_inputs as ri  # noqa: E402

from test_real_run import HIL_YAML, START, InlineJobs  # noqa: E402

MS = 1_000_000


def prep(inp, **kw):
    kw.setdefault("lidar_config", inp.lidar)
    kw.setdefault("imu_config", inp.imu)
    kw.setdefault("allow_synthetic", True)
    paths = rr.RunPaths(s10a=kw.pop("s10a", inp.s10a), replay=kw.pop("replay", inp.record), **kw)
    return rr.prepare_run(paths, clock=lambda: START, jobs_factory=InlineJobs, epoch=9)


def reasons(inp, **kw):
    with pytest.raises(AssemblyRefused) as err:
        prep(inp, **kw)
    return err.value.reasons


@pytest.fixture(scope="module")
def inputs(tmp_path_factory):
    return ri.write_run_inputs(tmp_path_factory.mktemp("replay"))


def test_a_replay_takes_the_records_start_its_map_and_its_lidar_for_the_layer(inputs):
    p = prep(inputs)
    assert p.replay is not None and p.map_id == inputs.map_id
    assert p.cfg.start_rear == pytest.approx(inputs.start_rear, abs=1e-9)  # the record's own start
    assert p.obstacles == []  # no hil: scene (the record stands for it)
    lc = mk.lidar_config()
    [sensor] = p.layer_cfg["sensors"]
    assert sensor["xyz"] == pytest.approx(list(lc.mount[:3])) and sensor["yaw_deg"] == pytest.approx(0.0)
    assert p.layer_cfg["bridge"]["scan_period_s"] == lc.scan_period_s
    assert p.hashes["replay_record"] == p.replay.record.sha256 and "lidar_config" in p.hashes
    head = p.replay.head()
    assert head["replayed_from"]["sha256"] == p.replay.record.sha256
    assert head["replayed_from"]["configs"]["lidar"]["shipped"] is False
    assert head["synthetic_inputs"] is True and head["not_for_judgement"] is True


def test_a_synthetic_hil_run_keeps_the_layer_files_lidar():
    """Regression (plan 4): only the replay reads the LiDAR config; the HIL run's layer is obstacle_layer_real.yaml's."""
    import yaml

    p = rr.prepare_run(rr.RunPaths(s10a=HIL_YAML), clock=lambda: START, jobs_factory=InlineJobs, epoch=9)
    layer = yaml.safe_load((REPO / "config/real/obstacle_layer_real.yaml").read_text())
    assert p.replay is None and p.layer_cfg == layer
    assert p.layer_cfg["bridge"]["scan_period_s"] == 0.1 and p.obstacles


def test_the_shipped_configs_are_refused_for_want_of_a_bound(inputs):
    got = reasons(inputs, lidar_config=REPO / "config/real/lidar.yaml", imu_config=REPO / "config/real/imu.yaml")
    assert any(r.startswith("replay_needs_bound:") and r.endswith(":mount") for r in got), got
    assert any(r.startswith("replay_needs_bound:") and r.endswith(":timing") for r in got), got
    got = reasons(inputs, imu_config=REPO / "config/real/imu.yaml")
    assert got == [f"replay_needs_bound:{REPO / 'config/real/imu.yaml'}:delay_bound"], got


def test_a_synthetic_record_needs_allow_synthetic(inputs):
    got = reasons(inputs, allow_synthetic=False)
    assert len(got) == 1 and got[0].startswith("replay:record_synthetic"), got


def test_the_start_map_offset_and_laser_must_be_the_records(tmp_path, inputs):
    # the S10-A file's start 0.2 m away from the record's
    far = ri.write_s10a(tmp_path / "far.yaml", inputs.map_id, (inputs.start_rear[0] + 0.2, *inputs.start_rear[1:]))
    got = reasons(inputs, s10a=far)
    assert len(got) == 1 and got[0].startswith("replay_start_mismatch:"), got
    # within 0.10 m / 5 deg: accepted, the record's start used
    near = ri.write_s10a(tmp_path / "near.yaml", inputs.map_id,
                         (inputs.start_rear[0] + 0.05, inputs.start_rear[1], inputs.start_rear[2] + math.radians(3)))
    assert prep(inputs, s10a=near).cfg.start_rear == pytest.approx(inputs.start_rear, abs=1e-9)
    # a record on another rear-axle offset, another laser, another map id
    for slam, want in (({"rear_offset_m": inputs.rear_offset_m + 0.01}, "replay_rear_offset_mismatch"),
                       ({"base_laser": [0.0, 0.0, 0.3, 0.0]}, "replay_base_laser_mismatch"),
                       ({"map_id": "f" * 64}, "replay_map_mismatch")):
        other = ri.write_run_inputs(tmp_path / want, slam=slam)
        got = reasons(other)
        assert any(r.startswith(want) for r in got), (want, got)


def test_a_bundle_given_by_hand_must_be_the_records_map(tmp_path, inputs):
    other = tmp_path / "other_bundle"
    ri.write_bundle(other)
    (other / "map.data").write_bytes(b"another map")  # its files no longer the manifest's
    got = reasons(inputs, map_bundle=other)
    assert got and got[0].startswith("map:"), got
    mine = tmp_path / "mine"
    mid = ri.write_bundle(mine)
    assert mid == inputs.map_id  # same files, same id: accepted wherever it lies
    assert prep(inputs, map_bundle=mine).map_id == inputs.map_id


def test_a_replay_needs_the_hil():
    got = None
    with pytest.raises(AssemblyRefused) as err:
        rr.prepare_run(rr.RunPaths(s10a=HIL_YAML, hil=False, replay=Path("x")), clock=lambda: START,
                       jobs_factory=InlineJobs)
    got = err.value.reasons
    assert got == ["replay_needs_hil"]


# --- the runner's command line ------------------------------------------------------------------------------------------
def _cli(argv, capsys):
    sys.path.insert(0, str(REPO / "tools"))
    import real_autonomy as tool

    code = tool.main(argv)
    return code, json.loads(capsys.readouterr().out)


def test_the_command_line_refuses_a_replay_without_the_bridge_or_with_scan_injections(capsys, tmp_path):
    base = ["--run", "--hil", "--hil-model", "/bin/true", "--duration-s", "10", "--out-dir", str(tmp_path),
            "--replay", str(tmp_path / "r.jsonl")]
    code, out = _cli(base, capsys)
    assert code == 2 and "replay_needs_slam_bridge" in out["refused"], out
    code, out = _cli(base + ["--hil-slam-bridge", "/tmp/x.sock", "--hil-inject", "scan:100",
                             "--hil-inject", "scan_lost:200", "--hil-inject", "tx:300"], capsys)
    assert code == 2 and out["refused"] == ["replay_inject:scan", "replay_inject:scan_lost"], out
    code, out = _cli(["--dry-assemble", "--replay", str(tmp_path / "r.jsonl")], capsys)
    assert code == 2 and out["refused"] == ["replay_needs_run_hil"], out


# --- the run's step ------------------------------------------------------------------------------------------------------
class _Src:
    def __init__(self, done=False):
        self.reader = SimpleNamespace(error=None)
        self.done = done
        self.counts = {}
        self.fatal = None
        self.reads = 0
        self.drained = []
        self.delta, self.live_from_ns = 5, 7

    def eof(self):
        return self.done

    def drain(self, now):
        self.drained.append(now)

    def set_anchor(self, a, n):
        self.anchor = (a, n)


def _control(scan, imu):
    plan = SimpleNamespace(record=SimpleNamespace(anchor_ns=1, counts={}))
    c = rpa.ReplayControl(plan, scan, imu)
    loop = SimpleNamespace(clock=lambda: 100, fatal_source=None, emitted=[], on_first_bounded_state=None)
    loop._emit = loop.emitted.append
    c.bind(loop)
    return c, loop


def test_the_imu_is_drained_after_a_tick_that_did_not_read_it_and_eof_needs_the_anchor_and_both_streams():
    scan, imu = _Src(done=True), _Src(done=True)
    c, loop = _control(scan, imu)
    assert c.step(10) is None  # no anchor yet: never EOF
    c.after_tick(10)
    assert imu.drained == [10]  # the loop did not read it in that tick: drained after it, up to its instant
    loop.on_first_bounded_state(50)
    assert scan.anchor == (50, 100) and imu.anchor == (50, 100) and loop.emitted[0]["t"] == "replay_anchor"
    imu.done = False
    assert c.step(20) is None
    imu.reads += 1  # the loop read it in this tick: not drained
    c.after_tick(20)
    assert imu.drained == [10]
    imu.done = True
    assert c.step(30) == "replay_eof"


def test_a_drain_failure_is_the_runs_fatal_source():
    scan, imu = _Src(), _Src()
    c, loop = _control(scan, imu)

    def boom(now):
        raise RuntimeError("replay read: gone")

    imu.drain = boom
    c.after_tick(10)
    assert loop.fatal_source.startswith("imu:replay_drain:RuntimeError")


def test_a_read_failure_is_the_runs_fatal_source():
    scan, imu = _Src(), _Src()
    c, loop = _control(scan, imu)
    imu.reader.error = OSError("gone")
    assert c.step(10) is None and loop.fatal_source.startswith("imu:replay_read:OSError")


def test_the_layers_hall_is_the_field_in_the_odometry_frame():
    """The odometry starts at the start's rear axle: a non-zero start carries the field's box by its inverse (the
    zero start keeps the bounds themselves -- the HIL's layer unchanged)."""
    cfg = SimpleNamespace(bounds=(-1.0, 6.0, -2.5, 2.5), start_rear=(0.0, 0.0, 0.0))
    assert rr._odom_bounds(cfg) == (-1.0, 6.0, -2.5, 2.5)
    cfg = SimpleNamespace(bounds=(-1.0, 6.0, -2.5, 2.5), start_rear=(1.0, 0.5, 0.0))
    assert rr._odom_bounds(cfg) == pytest.approx((-2.0, 5.0, -3.0, 2.0))
    cfg = SimpleNamespace(bounds=(-1.0, 6.0, -2.5, 2.5), start_rear=(0.0, 0.0, math.pi / 2))
    assert rr._odom_bounds(cfg) == pytest.approx((-2.5, 2.5, -6.0, 1.0))


def test_any_config_outside_config_real_marks_the_output_and_shipped_ones_do_not():
    """Codex 7r-2 round 1 P2-5: the marks follow every config the run reads (one outside config/real/ alone is
    enough), not only the sensor configs."""
    shipped = {k: {"shipped": True} for k in ("s10a", "lidar", "imu", "settings", "vehicle", "watch", "layer",
                                              "budget")}
    assert rpa.output_marks(allow_synthetic=False, record_synthetic=False, configs=shipped) == {}
    watch = dict(shipped, watch={"shipped": False})
    assert rpa.output_marks(allow_synthetic=False, record_synthetic=False, configs=watch)["not_for_judgement"] is True
    assert rpa.output_marks(allow_synthetic=True, record_synthetic=False, configs=shipped)["synthetic_inputs"] is True
    s10a = dict(shipped, s10a={"shipped": False})  # a field, speed or pallet the record does not check
    assert rpa.output_marks(allow_synthetic=False, record_synthetic=False, configs=s10a)["not_for_judgement"] is True
    assert rpa.output_marks(allow_synthetic=False, record_synthetic=True, configs=shipped)["not_for_judgement"] is True


def test_a_non_finite_start_or_offset_is_refused_before_any_comparison(tmp_path, inputs):
    other = ri.write_run_inputs(tmp_path / "nan", slam={"rear_offset_m": float("nan")})
    assert reasons(other) == ["replay_slam_header:not_finite"]
    nan_yaw = ri.write_s10a(tmp_path / "nan_yaw.yaml", inputs.map_id, inputs.start_rear)
    text = nan_yaw.read_text()
    nan_yaw.write_text(text.replace(f"start_rear: {list(inputs.start_rear)}",
                                    f"start_rear: [{inputs.start_rear[0]}, {inputs.start_rear[1]}, .nan]"))
    assert reasons(inputs, s10a=nan_yaw) == ["s10a_start_not_finite"]  # never overwritten by the record's start


def test_an_imu_stream_is_judged_without_a_watch_and_its_config_needs_the_bound(tmp_path, inputs):
    """Codex 7r-2 round 1 P2-2: with the watches taking no IMU, the record's IMU rows still go through an operating
    source (drained after each tick): its config must carry a measured bound."""
    import yaml

    data = yaml.safe_load((REPO / "config/real/autonomy.yaml").read_text())
    data["imu"] = "none"
    settings = tmp_path / "autonomy_no_imu.yaml"
    settings.write_text(yaml.safe_dump(data))
    p = prep(inputs, settings=settings)
    assert p.watches.imu_policy == "none" and p.replay.imu_cfg is not None
    got = reasons(inputs, settings=settings, imu_config=REPO / "config/real/imu.yaml")
    assert got == [f"replay_needs_bound:{REPO / 'config/real/imu.yaml'}:delay_bound"], got
