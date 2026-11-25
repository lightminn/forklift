"""The replay run's assembly (PR 7 7r, docs/plans/2026-11-26-7r-replay-run.md 4): ``real_autonomy.py --run --hil
--hil-slam-bridge <socket> --replay <record>`` -- an observe record's LiDAR and IMU behind the HIL run's loop, on the
real SLAM source.

``replay_inputs`` (before the model starts): the selected LiDAR config (``--lidar-config``, default the shipped
``config/real/lidar.yaml``) and the IMU source config (``--imu-config``); a LiDAR config whose mount or bound is not
measured, or an IMU config without a measured bound when the watches take an IMU or the record has IMU rows, is
refused ``replay_needs_bound:<config>:<what>`` (the shipped ones today); then the record checked, verified and indexed
(``replay_record.load_replay_record``; ``replay:<reason>`` when refused).
``replay_start``: the record's start (``slam_header.start_pose``, base_link) on the rear axle by the record's own
``rear_offset_m`` -- which must be the chassis model's -- within ``START_TOL_M`` / ``START_TOL_RAD`` of the S10-A
file's ``start_rear`` and then used for the whole run (the loop, the reference plan and the bridge's hello), the
record's ``map_id`` the S10-A file's, and the laser in base_link from the LiDAR config's mount the record's
``base_laser``. ``replay_layer_cfg``: the run's layer takes its LiDAR mount and ``scan_period_s`` from the same LiDAR
config the SLAM path uses (replay only: a synthetic HIL run keeps ``obstacle_layer_real.yaml``'s).

``make_replay_sources``: the real SLAM source on the bridge (with the HIL check), ``ReplayScan`` offering to it
directly (``RunSources.slam_feed`` None) and -- whenever the record has IMU rows, whatever the watches -- ``ReplayImu``
(the loop's IMU source only when the watches take one); ``ReplayControl`` gives the sources the loop's anchor
(``on_first_bounded_state``), after each tick drains the IMU when the loop did not read it that tick, surfaces a read
or drain failure as the run's fatal source and, before a tick, ends the run ``replay_eof`` once both streams are
through.

Output marks: ``--allow-synthetic``, a synthetic record or any config the run reads outside ``config/real/`` (the
S10-A file -- its map id and start are checked against the record, its speed, field and pallet are not --, the LiDAR
and IMU configs, settings, vehicle, watch, layer and budget files) marks the run's head ``synthetic_inputs`` / ``not_for_judgement``; the head carries ``replayed_from`` (path,
sha256, schema, configs). The bridge's own launch (its static TF) is outside the executor: the ROS stack check's item."""

from __future__ import annotations

import copy
import math
from dataclasses import dataclass, field, replace
from pathlib import Path

from forklift_core.live.real_assembly import AssemblyRefused

REPO = Path(__file__).resolve().parents[3]
SHIPPED = REPO / "config" / "real"
BASE_LASER_TOL = 1e-6
REAR_OFFSET_TOL_M = 1e-6


def _shipped(path) -> bool:
    return Path(path).resolve().parent == SHIPPED.resolve()


def _sha(path) -> str:
    import hashlib

    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _config(path) -> dict:
    return {"path": str(path), "sha256": _sha(path), "shipped": _shipped(path)}


def output_marks(*, allow_synthetic: bool, record_synthetic: bool, configs: dict) -> dict:
    """``OUTPUT_MARKS`` when any input makes the run's output no evidence (module text), else none."""
    from forklift_core.live.synthetic import OUTPUT_MARKS

    test_configs = any(c is not None and not c["shipped"] for c in configs.values())
    return dict(OUTPUT_MARKS) if (allow_synthetic or record_synthetic or test_configs) else {}


@dataclass
class ReplayPlan:
    record: object  # replay_record.ReplayRecord
    lidar_cfg: object  # lidar_cfg.LidarConfig
    imu_cfg: object  # imu_source.ImuSourceConfig, or None when the watches take no IMU
    configs: dict  # {"lidar": {path, sha256, shipped}, "imu": {...} | None}
    marks: dict = field(default_factory=dict)  # synthetic.OUTPUT_MARKS when the output is not for judgement

    def head(self) -> dict:
        r = self.record
        return {"replayed_from": {"path": str(r.path), "sha256": r.sha256, "schema": r.header.get("schema"),
                                  "synthetic": bool(r.synthetic), "configs": self.configs}, **self.marks}


def replay_inputs(paths, watches) -> ReplayPlan:
    """The configs and the record of ``paths.replay`` (``RunPaths``), or ``AssemblyRefused``."""
    from forklift_core.live.replay_record import ReplayRefused, load_replay_record
    from forklift_core.sensors.imu_source import load_imu_source_config
    from forklift_core.sensors.lidar_cfg import load_lidar_config

    refused = []
    try:
        lidar = load_lidar_config(paths.lidar_config)
    except Exception as err:  # noqa: BLE001 -- any unreadable config is structural
        raise AssemblyRefused([f"config_unreadable:lidar:{type(err).__name__}:{err}"]) from err
    if not lidar.mount_measured:
        refused.append(f"replay_needs_bound:{paths.lidar_config}:mount")  # the layer and SLAM placed on a guess
    if not lidar.bound_usable:
        refused.append(f"replay_needs_bound:{paths.lidar_config}:timing")  # every scan would be diagnostic
    imu_required = watches.imu_policy == "required"
    imu = None
    try:
        imu = load_imu_source_config(paths.imu_config)
    except Exception as err:  # noqa: BLE001
        raise AssemblyRefused([f"config_unreadable:imu:{type(err).__name__}:{err}"]) from err
    needs_imu = f"replay_needs_bound:{paths.imu_config}:delay_bound"
    if imu_required and not imu.measured:
        refused.append(needs_imu)
    configs = {"lidar": _config(paths.lidar_config), "imu": _config(paths.imu_config),
               **{name: _config(getattr(paths, name))
                  for name in ("s10a", "settings", "vehicle", "watch", "layer", "budget")}}
    if refused:
        raise AssemblyRefused(refused)
    try:
        record = load_replay_record(paths.replay, lidar_cfg=lidar, imu_cfg=imu, imu_required=imu_required,
                                    allow_synthetic=bool(paths.allow_synthetic))
    except ReplayRefused as err:
        raise AssemblyRefused([f"replay:{err}"]) from err
    if record.imu and not imu.measured:
        # the record's IMU rows go through an operating source whatever the watches (plan 2): it needs the bound
        raise AssemblyRefused([needs_imu])
    marks = output_marks(allow_synthetic=bool(paths.allow_synthetic), record_synthetic=bool(record.synthetic),
                         configs=configs)
    return ReplayPlan(record=record, lidar_cfg=lidar, imu_cfg=imu if record.imu else None, configs=configs,
                      marks=marks)


def replay_bundle(plan: ReplayPlan, paths) -> Path:
    """The map: the record's bundle, or ``--map-bundle`` when given (its map id checked against the record's)."""
    return Path(paths.map_bundle) if paths.map_bundle is not None else Path(plan.record.slam_header["bundle"])


def replay_start(plan: ReplayPlan, cfg, map_id: str, rear_offset_m: float):
    """The S10-A config with the record's start (rear axle), or ``AssemblyRefused`` (module text)."""
    from forklift_core.live.real_loop import START_TOL_M, START_TOL_RAD
    from forklift_core.localization.slam_real_scan import base_from_laser
    from forklift_core.localization.start_reloc import rear_from_base

    sh = plan.record.slam_header
    refused = []
    if sh.get("map_id") != map_id:
        refused.append(f"replay_map_mismatch:{str(sh.get('map_id'))[:12]}")
    try:
        rec_offset = float(sh["rear_offset_m"])
        start_base = tuple(float(v) for v in sh["start_pose"])
        rec_laser = tuple(float(v) for v in sh["base_laser"])
        s10a_start = tuple(float(v) for v in cfg.start_rear)
    except (KeyError, TypeError, ValueError) as err:
        raise AssemblyRefused([f"replay_slam_header:{err}"]) from err
    # every number finite before any comparison: a NaN passes every "greater than" (Codex 7r-2 round 1 P2-4)
    if len(start_base) != 3 or len(rec_laser) != 4 or not all(map(math.isfinite, start_base + rec_laser + (rec_offset,))):
        raise AssemblyRefused(["replay_slam_header:not_finite"])
    if len(s10a_start) != 3 or not all(map(math.isfinite, s10a_start)):
        raise AssemblyRefused(["s10a_start_not_finite"])
    if abs(rec_offset - float(rear_offset_m)) > REAR_OFFSET_TOL_M:
        refused.append(f"replay_rear_offset_mismatch:{rec_offset}:{rear_offset_m}")
    start = tuple(float(v) for v in rear_from_base(start_base, rec_offset))
    sx, sy, syaw = s10a_start
    d = math.hypot(start[0] - sx, start[1] - sy)
    dyaw = abs(math.atan2(math.sin(start[2] - syaw), math.cos(start[2] - syaw)))
    if d > START_TOL_M or dyaw > START_TOL_RAD:
        refused.append(f"replay_start_mismatch:{d:.3f}m:{math.degrees(dyaw):.1f}deg")
    laser = tuple(float(v) for v in base_from_laser(plan.lidar_cfg.mount, float(rear_offset_m)))
    if any(abs(a - b) > BASE_LASER_TOL for a, b in zip(laser, rec_laser)):
        refused.append("replay_base_laser_mismatch")
    if refused:
        raise AssemblyRefused(refused)
    return replace(cfg, start_rear=start)


def replay_layer_cfg(layer_cfg: dict, lidar_cfg) -> dict:
    """The layer file with its LiDAR mount and scan period from the LiDAR config (one source for the layer and
    SLAM, plan 4)."""
    lc = copy.deepcopy(layer_cfg)
    x, y, z, yaw = (float(v) for v in lidar_cfg.mount)
    lc["sensors"] = [{"name": "lidar", "xyz": [x, y, z], "yaw_deg": math.degrees(yaw)}]
    lc["bridge"]["scan_period_s"] = float(lidar_cfg.scan_period_s)
    return lc


class ReplayControl:
    """The run's side of the replay sources (module text): ``bind(loop)`` once the loop exists, ``step(now_ns)``
    before each tick (``replay_eof`` or None), ``summary()`` for the end row."""

    def __init__(self, plan: ReplayPlan, scan, imu) -> None:
        self.plan, self.scan, self.imu = plan, scan, imu
        self.clock = None
        self.loop = None
        self.anchor = None
        self._imu_reads = 0
        self.eof_ns = None
        self.drains = 0

    def bind(self, loop) -> None:
        self.loop, self.clock = loop, loop.clock
        loop.on_first_bounded_state = self._anchor

    def _anchor(self, rx_ns: int) -> None:
        now = int(self.clock())
        self.scan.set_anchor(int(rx_ns), now)
        if self.imu is not None:
            self.imu.set_anchor(int(rx_ns), now)
        self.anchor = {"t": "replay_anchor", "ns": now, "hil_anchor_ns": int(rx_ns),
                       "record_anchor_ns": int(self.plan.record.anchor_ns), "delta_ns": int(self.scan.delta),
                       "live_from_ns": int(self.scan.live_from_ns)}
        self.loop._emit(dict(self.anchor))  # noqa: SLF001 -- the run's record sink

    def _mark(self, why: str) -> None:
        if self.loop is not None and self.loop.fatal_source is None:
            self.loop.fatal_source = why

    def _read_failed(self) -> bool:
        for name, src in (("scan", self.scan), ("imu", self.imu)):
            err = None if src is None else src.reader.error
            if err is not None:
                self._mark(f"{name}:replay_read:{type(err).__name__}:{err}")
                return True
        return False

    def step(self, now_ns: int):
        """Before a tick: a read failure is the run's fatal source; ``replay_eof`` once the anchor stands and both
        streams are through."""
        if self._read_failed():
            return None
        imu = self.imu
        if self.anchor is not None and self.scan.eof() and (imu is None or imu.eof()):
            self.eof_ns = int(now_ns)
            return "replay_eof"
        return None

    def after_tick(self, now_ns: int) -> None:
        """After a tick: the IMU the loop did not read in it (no watch, or bypassed in that tick) put in up to the
        tick's instant, judged, output dropped (Codex 7r-2 round 1 P2-1: never before the tick that may lift the
        bypass); its failure is the run's fatal source."""
        imu = self.imu
        if imu is None:
            return
        if imu.reads == self._imu_reads:
            try:
                imu.drain(int(now_ns))
                self.drains += 1
            except Exception as exc:  # noqa: BLE001 -- the run ends on it
                self._mark(f"imu:replay_drain:{type(exc).__name__}:{exc}")
        self._imu_reads = imu.reads

    def summary(self) -> dict:
        return {"anchor": self.anchor, "eof_ns": self.eof_ns, "drains": self.drains,
                "record": dict(self.plan.record.counts),
                "scan": dict(self.scan.counts), "imu": None if self.imu is None else dict(self.imu.counts),
                "scan_fatal": self.scan.fatal}


def make_replay_sources(prep, host, transport, *, slam_socket: str):
    """The replay run's sources (module text) on a transport whose HIL check holds."""
    from forklift_core.live import real_run as rr
    from forklift_core.live.replay_sources import ReplayImu, ReplayScan

    plan = prep.replay
    box: dict = {}
    sources = rr._real_slam_sources(prep, host, transport, None, None, box, slam_socket)  # noqa: SLF001
    try:
        scan = ReplayScan(transport, plan.record, plan.lidar_cfg, prep.start_ns, slam=sources.slam)
        sources.closers.append(scan.close)
        imu = None
        if plan.record.imu:  # its rows judged whatever the watches (plan 2); the loop reads it only with a watch
            imu = ReplayImu(transport, plan.record, plan.imu_cfg)
            sources.closers.append(imu.close)
    except BaseException:
        for fn in sources.closers:
            fn()
        raise
    control = ReplayControl(plan, scan, imu)
    loop_imu = imu if prep.watches.imu_policy == "required" else None
    sources.scans, sources.imu, sources.slam_feed, sources.replay = scan, loop_imu, None, control
    sources.on_assembled.append(lambda loop: box.__setitem__("loop", loop))
    sources.on_assembled.append(control.bind)
    return sources


__all__ = ["ReplayControl", "ReplayPlan", "make_replay_sources", "replay_bundle", "replay_inputs",
           "replay_layer_cfg", "replay_start"]
