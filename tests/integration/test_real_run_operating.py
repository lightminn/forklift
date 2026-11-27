"""The operating REAL run end to end (operating unlock U3e-1, docs/plans/2026-11-26-real-operating-assembly.md 6.1).

The run is ``tools/real_autonomy.run_real`` in a child (``real_operating_child.py``) on the MCU model behind a pty
the test makes, as an operating board: ``--hil-plant 0`` (outputs open, e-stop wired), the operating product string
and serial handed to ``RealTtyTransport.open``. Its sources are the operating ones: the real scan source on a
datagram socket this test feeds with the scan adapter's own v2 frames (an empty corridor: every beam at a far finite
range, pose-independent), the real SLAM source on ``FakeBridge`` in the real bridge's comparison mode (map id and
mount compared, the digest on every ready), no IMU device. The site is a valid temporary one -- its lidar.yaml first,
then the survey report, registry and bundle from that file's hash, and an S10 precondition record bound to it --
with evidence in tests/fixtures/operating/ (the assembly takes evidence inside the repository only).

The lock (``precondition:unlock_incomplete``) is in the repository until U3e-2. A case that needs the run to move
takes only that reason out of the judgement (the child's ``drop_lock``); every other judgement runs as it is.

What this shows is the wiring and the contract on the wall clock -- the empty corridor's run and stop. It is no
evidence of obstacle projection, SLAM matching or any field value. CPU: as the HIL runs, 4 CPUs or more (a partial
forklift-ws1-pytest naming a test_real_run_* file gives 4)."""

from __future__ import annotations

import asyncio
import contextlib
import json
import math
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

pytest.importorskip("websockets")

from websockets.asyncio.client import connect  # noqa: E402

from fake_slam_bridge import FakeBridge  # noqa: E402
from test_real_run_hil import Console, accept_rows, accepted, mcu_model  # noqa: E402,F401

REPO = Path(__file__).resolve().parents[2]
REAL = REPO / "config" / "real"
sys.path.insert(0, str(REPO / "tests" / "fixtures" / "replay"))
sys.path.insert(0, str(REPO / "tests" / "unit" / "live"))
import make_record as mr  # noqa: E402

from forklift_core.live import real_preconditions as rp  # noqa: E402
from forklift_core.live import real_run as rr  # noqa: E402
from forklift_core.live.real_site import REAL_ENCODER_MODEL  # noqa: E402
from forklift_core.localization import slam_map as sm  # noqa: E402
from forklift_core.plant.real_vehicle import chassis_model_path, load_real_vehicle  # noqa: E402
from forklift_core.sensors.lidar_cfg import load_lidar_config  # noqa: E402

SERIAL = "E2E0OPERATING0000000001"
ABBREV = "0123456789ab"
PRODUCT = f"forklift-mcu normal OPEN {ABBREV} rec=1f4207f4"
BOOT = 0x0B00700  # mcu_model's default --boot-id
RUN = mr.RUN
EVIDENCE = "tests/fixtures/operating/evidence.md"
BOUND = "tests/fixtures/operating/bound.md"
PALLET = [2.6, 0.0, 0.0]  # as the HIL S10: about 1.34 m of rear-axle travel to the lift, under the 1.5 m hold
D5_CAP = 0.06  # real_loop.DECLARED_SPEED_MPS: the cap while d5_pocket is bypassed


def _sha(path) -> str:
    import hashlib

    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _measured(node):
    if isinstance(node, dict):
        for k, v in node.items():
            if k == "provenance" and isinstance(v, dict):
                v["measured"] = True
            else:
                _measured(v)
    elif isinstance(node, list):
        for v in node:
            _measured(v)
    return node


def _bundle(root):
    folder = root / "site_bundle"
    folder.mkdir()
    (folder / "map.posegraph").write_bytes(b"graph")
    (folder / "map.data").write_bytes(b"data")
    np.savez(folder / "grid.npz", data=np.zeros((50, 70), np.int8), origin=np.asarray((-1.0, -2.5, 0.0)),
             resolution_m=0.1)
    hashes = {name: sm.file_sha256(folder / name) for name in sm.BUNDLE_FILES}
    map_id = sm.map_id_of(hashes)
    (folder / "manifest.json").write_text(json.dumps({"format": sm.FORMAT, "map_id": map_id, "files": hashes}))
    return folder, map_id


class Site:
    """A valid temporary operating site (plan 6.1): configs measured, lidar.yaml first, the survey and registry bound
    to it, the S10 precondition record bound to the board this test gives."""

    def __init__(self, root: Path, *, record_over=None):
        self.root = root
        cfg = self.cfg = root / "cfg"
        cfg.mkdir(parents=True)
        shutil.copy(REAL / "imu.yaml", cfg / "imu.yaml")
        auto = yaml.safe_load((REAL / "autonomy.yaml").read_text())
        auto.update(mission_time_budget_s=60, insert_time_min_s=20)  # the test's run is short (plan 6.1)
        (cfg / "autonomy.yaml").write_text(yaml.safe_dump(auto, sort_keys=False))
        for name in ("vehicle.yaml", "obstacle_layer.yaml", "safety_watch.yaml", "obstacle_layer_real.yaml"):
            d = _measured(yaml.safe_load((REAL / name).read_text()))
            if name == "vehicle.yaml":
                d["chassis_model"] = str((REAL / d["chassis_model"]).resolve())
            (cfg / name).write_text(yaml.safe_dump(d, sort_keys=False))
        lidar = yaml.safe_load((REAL / "lidar.yaml").read_text())
        lidar["timing"].update(first_beam_err_s=0.002, basis="engineering", decision_id="D-E2E", basis_doc=BOUND,
                               applies_to={k: mr.FACTS[k] for k in ("image_id", "usb", "host", "ans_type",
                                                                    "sample_period_us", "baudrate")})
        lidar["mount"]["measured"] = True
        (cfg / "lidar.yaml").write_text(yaml.safe_dump(lidar, sort_keys=False))
        # the survey: its report hashes this lidar.yaml (made first) and the chassis
        bundles = root / "bundles"
        bundles.mkdir()
        folder, self.map_id = _bundle(bundles)
        s10a = yaml.safe_load((REAL / "s10a.yaml").read_text())
        s10a.update(map_id=self.map_id, check_pallet_pose=PALLET, start_rear=[0.0, 0.0, 0.0])
        s10a["pallet"].update(depth_m=0.6, width_m=0.4)
        (cfg / "s10a.yaml").write_text(yaml.safe_dump(s10a, sort_keys=False))
        self.site_record = root / "site_record.yaml"
        self.site_record.write_text("site: e2e hall\n")
        vehicle = cfg / "vehicle.yaml"
        urdf = chassis_model_path(vehicle).parent / "forklift.urdf"
        self.rear_offset = rr.chassis_reference(urdf)[1]
        start_base = [self.rear_offset, 0.0, 0.0]
        report = {"kind": "site_survey/1", "ok": True, "site": "e2e hall", "site_record_sha256": _sha(self.site_record),
                  "map_id": self.map_id, "encoder_model": REAL_ENCODER_MODEL, "start_pose": start_base,
                  "config_sha256": {"lidar": _sha(cfg / "lidar.yaml"), "chassis": _sha(urdf)},
                  "vehicle_calibration_id": load_real_vehicle(vehicle).calibration_id, "raw": [],
                  "site_check": {"points": 5}}
        rpath = folder / "site_survey_report.json"
        rpath.write_text(json.dumps(report))
        row = {"name": "e2e", "site": "e2e hall", "site_record_sha256": _sha(self.site_record),
               "survey_report": {"path": "site_bundle/site_survey_report.json", "sha256": _sha(rpath)},
               "encoder_model": REAL_ENCODER_MODEL, "folder": folder.name, "map_id": self.map_id,
               "start_pose": start_base, "start_relocalizable": False, "start_reloc_evidence": None}
        self.registry = root / "registry.yaml"
        self.registry.write_text(yaml.safe_dump({"bundles": [row]}))
        self.bundle_root = bundles
        self.record = cfg / "s10_preconditions.yaml"
        self.write_record(self.full_record())
        if record_over is not None:
            rec = self.full_record()
            record_over(rec)
            self.write_record(rec)
        self.laser = rr.bridge_laser_xyz_yaw(load_lidar_config(cfg / "lidar.yaml"), self.rear_offset)

    def paths(self):
        c = self.cfg
        return dict(record_path=self.record, lidar_path=c / "lidar.yaml", imu_path=c / "imu.yaml",
                    settings_path=c / "autonomy.yaml", s10a_path=c / "s10a.yaml",
                    layer_path=c / "obstacle_layer_real.yaml", budget_path=c / "obstacle_layer.yaml",
                    vehicle_path=c / "vehicle.yaml", watch_path=c / "safety_watch.yaml")

    def write_record(self, rec):
        self.record.write_text(yaml.safe_dump(rec, sort_keys=False))

    def full_record(self):
        self.write_record({"schema": rp.SCHEMA, "items": {}, "config_exceptions": []})
        board = {"usb_product": PRODUCT, "usb_serial": SERIAL, "state_seen": True, "drive_refusal": None, "boot_id": BOOT}
        f = rp.gather(board=board, **self.paths())
        stop = dict(f.stop_files)
        common = {"status": "pass", "evidence": EVIDENCE, "date": "2026-12-01"}
        binds = {
            "s5_safety": {"board_serial": SERIAL, "product": PRODUCT, "stop_files": stop},
            "iwdg_evidence": {"commit": ABBREV + "cdef0123", "board_serial": SERIAL},
            "fault_detection": {"board_serial": SERIAL, "product": PRODUCT},
            "port_review": {},
            "scan_load": {"lidar_sha256": f.lidar_sha256, "scan_image": mr.FACTS["image_id"]},
            "scan_coverage": {"lidar_sha256": f.lidar_sha256, "start_rear": list(f.s10a.start_rear),
                              "check_pallet_pose": list(f.s10a.check_pallet_pose), "corridor_half_width_m": 0.10,
                              "map_id": f.s10a.map_id, "inserted_offset_m": f.inserted_offset_m},
            "s10_speed": {"approved_max_speed_mps": 0.15, "tested_speed_mps": 0.15, "approved_cases": ["A"]},
            "start_taped": {"start_rear": list(f.s10a.start_rear)},
            "lift": {"board_serial": SERIAL, "lift_target_m": f.settings["lift_target_m"],
                     "lift_rate_mps": f.settings["lift_rate_mps"], "test_pallet": f.s10a.pallet_id,
                     "lift_mechanism": f"test only, {EVIDENCE}", "pallet_mass_kg": 4.5},
            "firmware_build": {"product": PRODUCT, "firmware_vehicle_calibration_id": f.vehicle_calibration_id,
                               "readback_sha256": "0" * 64, "board_lift_rate_max_mps": 0.10},
            "stop_model": {"stop_files": stop, "lidar_sha256": f.lidar_sha256},
            "jetson_load": {"stop_files": stop},
        }
        items = {name: {**common, "binds": b} for name, b in binds.items()}
        target = float(f.settings["lift_target_m"])
        items["lift"].update(
            origin={"state_lift_m_at_floor": 0.0, "tape_cm_at_target": target * 100, "state_lift_m_at_target": target},
            hold=[{"raised_cm": target * 40, "sag_cm_after_60s": 0.1, "worst": True, "worst_reason": "test only"},
                  {"raised_cm": target * 100, "sag_cm_after_60s": 0.2}],
            sag_limit_cm=1.0, sag_limit_decision_id="D-E2E")
        return {"schema": rp.SCHEMA, "items": items, "config_exceptions": []}

    def argv(self, device, out_dir, duration_s, scan_socket, slam_socket, *extra):
        c = self.cfg
        return ["--run", "--client", "real", "--device", str(device), "--scan-socket", str(scan_socket),
                "--slam-bridge", str(slam_socket), "--site-record", str(self.site_record),
                "--bundle-root", str(self.bundle_root), "--registry", str(self.registry),
                "--s10a", str(c / "s10a.yaml"), "--settings", str(c / "autonomy.yaml"),
                "--vehicle", str(c / "vehicle.yaml"), "--watch", str(c / "safety_watch.yaml"),
                "--layer", str(c / "obstacle_layer_real.yaml"), "--budget", str(c / "obstacle_layer.yaml"),
                "--lidar-config", str(c / "lidar.yaml"), "--imu-config", str(c / "imu.yaml"),
                "--preconditions", str(self.record), "--duration-s", str(duration_s), "--out-dir", str(out_dir),
                *extra]


class ScanProducer(threading.Thread):
    """The scan container's stand-in: the adapter's own v2 frames (``SdkStream`` + ``make_frame``) of an empty
    corridor at 10 Hz on the run's datagram socket, a new scan id and first-sample time each revolution. ``cut()``
    stops sending (the run sees a receive gap)."""

    def __init__(self, path, *, n=800, range_m=8.0, hz=10.0):
        super().__init__(daemon=True)
        ra = mr._adapter()  # noqa: SLF001 -- the record maker's import of the ROS adapter module
        self.ra, self.path, self.n, self.range_m, self.period = ra, str(path), n, float(range_m), 1.0 / hz
        self.stream = ra.SdkStream(RUN, {k: v for k, v in mr.FACTS.items() if k in ("image_id", "usb", "host")})
        self.start_id = "fedcba9876543210"
        self.stream.on_info(json.dumps({"format": "sdk_mono_v2", "run": RUN, "start": self.start_id,
                                        **{k: mr.FACTS[k] for k in ("ans_type", "sample_period_us", "baudrate")}}))
        self._halt, self._cut = threading.Event(), threading.Event()
        self.sent, self.cut_ns = 0, None
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def cut(self):
        self.cut_ns = time.monotonic_ns()
        self._cut.set()

    def stop(self):
        self._halt.set()
        self.join(5)
        self.sock.close()

    def run(self):
        span = 0.098
        j = 0
        while not self._halt.is_set():
            if not self._cut.is_set() and os.path.exists(self.path):
                now = time.monotonic_ns()
                first = now - int(span * 1e9) - 2_000_000
                msg = SimpleNamespace(
                    ranges=list(np.full(self.n, self.range_m)), intensities=[], angle_min=-math.pi,
                    angle_increment=2 * math.pi / self.n, time_increment=float(np.float32(span / (self.n - 1))),
                    scan_time=float(np.float32(span)), range_min=0.1, range_max=12.0,
                    header=SimpleNamespace(stamp=SimpleNamespace(sec=first // 10**9, nanosec=first % 10**9),
                                           frame_id=f"laser@sdk_mono_v2#{RUN}.{self.start_id}"))
                off = time.time_ns() - time.monotonic_ns()
                out, why = self.stream.on_scan(first, msg.scan_time, now, off, frame_id=msg.header.frame_id)
                if why is None:
                    j += 1
                    frame = self.ra.make_frame(msg, scan_id=j, recv_mono_ns=now, sys_minus_mono_ns=off, gen=out["gen"],
                                               stamp_kind=out["stamp_kind"], sdk=out["sdk"])
                    with contextlib.suppress(OSError):
                        self.sock.sendto(frame, self.path)
                        self.sent += 1
            self._halt.wait(self.period)


class OperatingConsole(Console):
    """The HIL test's console client, from the run's own page: the REAL console refuses any other Origin."""

    async def _open(self):
        self.ws = await connect(f"ws://127.0.0.1:{self.port}/api/v1/control", origin=f"http://127.0.0.1:{self.port}")
        await self.ws.send(json.dumps({"t": "hello", "api": "console/1", "client_id": "e2e-test"}))
        welcome = json.loads(await self.ws.recv())
        self.token, self.epoch = welcome.get("request_token"), welcome["session_epoch"]
        self.reader = asyncio.ensure_future(self._read())


class Operating:
    """The model on the test's pty, the scan producer, the fake bridge and the run's child."""

    def __init__(self, model, tmp_path: Path, site: Site, *, duration_s=120, drop_lock=True, model_args=(),
                 product=PRODUCT):
        self.tmp, self.site = tmp_path, site
        self.out = tmp_path / "run"
        self.writes_log = tmp_path / "writes.jsonl"
        self.accept_log = tmp_path / "accept.log"
        master, slave = os.openpty()
        self.slave_name = os.ttyname(slave)
        self.model = subprocess.Popen([str(model), "--link-fd", str(master), "--hil-plant", "0",
                                       "--duration-ms", str(int((duration_s + 60) * 1000)),
                                       "--accept-log", str(self.accept_log), *map(str, model_args)],
                                      pass_fds=(master,), start_new_session=True)
        os.close(master)
        self._slave = slave
        self.bridge = FakeBridge(tmp_path / "slam.sock", map_id=site.map_id, laser=site.laser)
        self.bridge.start()
        self.scan = ScanProducer(tmp_path / "scan.sock")
        self.scan.start()
        cfg = tmp_path / "child.json"
        cfg.write_text(json.dumps({"argv": site.argv(self.slave_name, self.out, duration_s, tmp_path / "scan.sock",
                                                     self.bridge.path), "product": product, "serial": SERIAL,
                                   "writes_log": str(self.writes_log), "drop_lock": bool(drop_lock),
                                   "fd": slave}))
        self.proc = subprocess.Popen([sys.executable, str(REPO / "tests/integration/real_operating_child.py"),
                                      str(cfg)], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                     env={**os.environ, "PYTHONPATH": str(REPO / "src")}, start_new_session=True,
                                     pass_fds=(slave,))
        # the child holds the slave now; the test's own copy closed at once -- one open file description shared by
        # both, so the lock the child takes on it is released only when every copy is closed (the run's own release
        # must be what frees the device)
        os.close(slave)
        self._slave = None
        self.port = None
        self.lines = []

    def wait_console(self, timeout=60):
        """The console line, read with a deadline that holds when the child prints nothing (or hangs)."""
        import select

        end = time.monotonic() + timeout
        fd = self.proc.stdout.fileno()
        os.set_blocking(fd, False)
        buf = b""
        try:
            while time.monotonic() < end:
                ready, _, _ = select.select([fd], [], [], max(0.0, end - time.monotonic()))
                if not ready:
                    break
                chunk = os.read(fd, 65536)  # never a blocking readline: a line without its newline keeps the deadline
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    text = line.decode() + "\n"
                    self.lines.append(text)
                    if text.startswith('{"console"'):
                        self.port = json.loads(text)["port"]
                        return self.port
        finally:
            if buf:
                self.lines.append(buf.decode(errors="replace"))
            os.set_blocking(fd, True)
        return None

    def finish(self, sig=None, timeout=60):
        if sig is not None and self.proc.poll() is None:
            self.proc.send_signal(sig)
        out, err = self.proc.communicate(timeout=timeout)
        self.lines += out.splitlines(keepends=True)
        self.stderr = err
        return self.proc.returncode

    def end(self):
        text = "".join(self.lines)
        i = text.rfind('{\n  "run"')
        return json.loads(text[i:]) if i >= 0 else None

    def rows(self):
        p = self.out / "run.jsonl"
        return [json.loads(x) for x in p.read_text().splitlines() if x.strip()] if p.exists() else []

    def writes(self):
        if not self.writes_log.exists():
            return []
        return [json.loads(x) for x in self.writes_log.read_text().splitlines() if x.strip()]

    def close(self):
        for p in (self.proc, self.model):
            if p.poll() is None:
                p.kill()
            with contextlib.suppress(Exception):
                p.wait(10)
        for pipe in (self.proc.stdout, self.proc.stderr):
            with contextlib.suppress(Exception):
                pipe.close()
        self.scan.stop()
        self.bridge.closing()
        self.bridge.stop()
        if self._slave is not None:
            with contextlib.suppress(OSError):
                os.close(self._slave)


def _motion_bytes(writes, since_ns=None):
    return sum(w["n"] for w in writes if w["kind"] in ("ARM", "DRIVE") and (since_ns is None or w["ns"] >= since_ns))


@contextlib.contextmanager
def operating(model, tmp_path, site, **kw):
    run = Operating(model, tmp_path, site, **kw)
    try:
        yield run
    finally:
        run.close()


def _explain(run):
    return f"stdout:\n{''.join(run.lines)[-3000:]}\nstderr:\n{(getattr(run, 'stderr', '') or '')[-3000:]}"


# --- case 1: the repository's lock on a valid site ---------------------------------------------------------------
def test_on_a_valid_site_the_lock_alone_refuses_and_nothing_reaches_the_board(mcu_model, tmp_path):
    """Plan 6.1 case 1: no judgement taken out -- the assembly is refused with exactly the lock, and not one byte
    (no resync byte either) was written to the board."""
    site = Site(tmp_path / "site")
    with operating(mcu_model, tmp_path, site, drop_lock=False) as run:
        code = run.finish(timeout=90)
        end = run.end()
        assert code == 2, _explain(run)
        assert end is not None and end["end"]["refused"] == [rp.UNLOCK_INCOMPLETE], _explain(run)
        assert sum(w["n"] for w in run.writes()) == 0, run.writes()[:5]
        assert run.port is None  # no console was served


# --- the operator's steps on the REAL console ---------------------------------------------------------------------
def _events(rows, t):
    return [r for r in rows if r.get("t") == t]


def _written_moving(rows):
    """The moving DRIVEs written whole, with their queued speed by seq."""
    speed = {q["seq"]: q["speed_mps"] for q in _events(rows, "frame_queued") if q.get("purpose") == "drive"}
    return [speed[w["seq"]] for w in _events(rows, "frame_written")
            if w["kind"] == "DRIVE" and speed.get(w["seq"]) not in (None, 0.0)]


def _lease(c):
    c.wait(lambda s: s["permit"]["state"] == "OFF" and s.get("standing"), 20)
    r = c.ask("lease_request")
    assert r["t"] == "ack", r
    c.lease = r["lease_id"]


def _confirm_start(c, timeout=30.0):
    """The session starts by relocalizing (PR 7 design 6.1, 6): the start generation confirmed on the bridge's first
    instance -- asked again while the evidence of this epoch is not there yet -- until ``confirmed``. The operating
    session always opens one: none at all is a failure, not a pass."""
    end = time.monotonic() + timeout
    assert isinstance(c.status.get("relocalize_generation"), int), c.status.get("relocalize_generation")
    while True:
        gen = c.status.get("relocalize_generation")
        if gen is None:
            assert (c.status.get("relocalize") or {}).get("state") == "confirmed", c.status.get("relocalize")
            return
        r = c.ask("relocalize_confirm", generation=gen)
        if r["t"] == "ack":
            s = c.wait(lambda s: (s.get("relocalize") or {}).get("state") not in ("waiting", None), 20)
            if s["relocalize"]["state"] == "confirmed":
                return
        assert time.monotonic() < end, f"the start generation was not confirmed: {r} {c.status.get('relocalize')}"
        time.sleep(0.3)


def _bypass(c, check):
    r = c.ask("check_bypass", check=check, on=True)
    assert r["t"] == "ack", r
    c.wait(lambda s: check in s["bypassed"], 5)


def _register(c):
    r = c.ask("pallet_register", pallet_id="P-S10", pose=list(PALLET))
    assert r["t"] == "ack" and isinstance(r.get("register_serial"), int) and r.get("region"), r
    return r


def _confirm_pallet(c, reg, timeout=20.0):
    """The pallet confirmation as the console sends it (plan 2.6, U3c): the registration and the still span."""
    end = time.monotonic() + timeout
    while True:
        s = c.wait(lambda s: isinstance(s.get("still_serial"), int), 20)
        r = c.ask("operator_start", pallet_id="P-S10", targets_revision=reg["targets_revision"],
                  register_serial=reg["register_serial"], still_serial=s["still_serial"])
        if r["t"] == "ack" or time.monotonic() > end or r.get("reason") not in ("slam_stale", "not_still",
                                                                                 "still_changed"):
            return r
        time.sleep(0.3)


def _ready_to_submit(c):
    _lease(c)
    _confirm_start(c)
    _bypass(c, "d5_pocket")
    _bypass(c, "imu")  # no IMU device on this run: C7 is opened by the session's override (plan 2.7, 6.1)
    return _register(c)


# --- case 3: the normal path -------------------------------------------------------------------------------------
def test_the_operating_run_drives_to_the_lift_stops_and_disarms(mcu_model, tmp_path):
    """Plan 6.1 case 3: relocalization confirmed -> pallet registered -> d5_pocket and imu overridden -> pallet
    confirmed -> submitted -> ARM -> moving DRIVEs, all at most 0.06 m/s -> lift -> the lifted check confirmed ->
    s10_lifted, the S10 stop's DISARM confirmed on the board -> the run's own closing DISARM confirmed, exit 0."""
    site = Site(tmp_path / "site")
    with operating(mcu_model, tmp_path, site, duration_s=150) as run:
        assert run.wait_console() is not None, _explain(run)
        c = OperatingConsole(run.port)
        try:
            reg = _ready_to_submit(c)
            r = _confirm_pallet(c, reg)
            assert r["t"] == "ack" and r.get("region"), r
            r = c.ask("mission_submit", pallet_id="P-S10", destination_id="D-S10",
                      targets_revision=reg["targets_revision"])
            assert r["t"] == "ack", r
            s = c.wait(lambda s: s["operator_request"] is not None or s["permit"]["state"] == "LATCHED", 120)
            op = s["operator_request"]
            # the cause in the message, not a truncated dict (a latch names its reason in permit)
            assert op is not None and op["kind"] == "lifted", (s["permit"], s["mission"], s["last_refusal"])
            r = c.ask("operator_answer", kind=op["kind"], serial=op["serial"], verdict="confirm")
            assert r["t"] == "ack", r
            c.wait(lambda s: s["permit"]["state"] == "LATCHED", 60)
        finally:
            with contextlib.suppress(Exception):
                c.close()
            run.bridge.closing()
            code = run.finish(signal.SIGINT, timeout=60)
        rows = run.rows()
        end = _events(rows, "end")[-1]
        assert code == 0, _explain(run)
        assert end["closing"] == "confirmed" and end["record_incomplete"] is None, end
        assert [e["reason"] for e in end["results"]] == ["s10_lifted"], end
        perm = _events(rows, "permit")
        assert any(p["event"] == "control_stop_confirmed" for p in perm), perm[-6:]  # the S10 stop DRIVE taken
        disarmed = [p for p in perm if p["event"] == "disarmed"]
        assert disarmed and disarmed[0]["ns"] < end["closing_started_ns"], perm[-6:]  # the S10 DISARM answered
        moving = _written_moving(rows)
        assert moving and max(abs(v) for v in moving) <= D5_CAP + 1e-9, moving
        grants = [r for r in rows if r.get("t") == "real_operating"]
        assert grants and grants[0]["granted"] is True and grants[0]["boot"] == BOOT, grants
        assert run.bridge.error is None and run.bridge.digest is not None
        assert _motion_bytes(run.writes()) > 0


# --- case 4: refused before departure ----------------------------------------------------------------------------
def test_a_submit_without_the_pallet_confirmation_never_moves_the_board(mcu_model, tmp_path):
    """Plan 6.1 case 4: everything else ready, no pallet confirmation -> the submit's exact refusal, and not one ARM
    or DRIVE byte for the whole run."""
    site = Site(tmp_path / "site")
    with operating(mcu_model, tmp_path, site, duration_s=150) as run:  # run time to spare: only the confirmation lacks
        assert run.wait_console() is not None, _explain(run)
        c = OperatingConsole(run.port)
        try:
            reg = _ready_to_submit(c)
            r = c.ask("mission_submit", pallet_id="P-S10", destination_id="D-S10",
                      targets_revision=reg["targets_revision"])
            assert r["t"] == "reject" and r["reason"].endswith("operator_start:unconfirmed"), r
            time.sleep(2.0)
        finally:
            with contextlib.suppress(Exception):
                c.close()
            run.bridge.closing()
            code = run.finish(signal.SIGINT, timeout=60)
        assert code == 0, _explain(run)
        assert _motion_bytes(run.writes()) == 0, [w for w in run.writes() if w["kind"] in ("ARM", "DRIVE")][:5]


def test_one_reason_left_in_the_judgement_refuses_the_assembly(mcu_model, tmp_path):
    """Plan 6.1 case 4: with fault_detection unset the judgement less the lock is exactly that one reason -- the run is
    not assembled and nothing reaches the board."""
    site = Site(tmp_path / "site", record_over=lambda rec: rec["items"].__setitem__("fault_detection", None))
    with operating(mcu_model, tmp_path, site, drop_lock=True) as run:
        code = run.finish(timeout=90)
        end = run.end()
        assert code == 2, _explain(run)
        assert end["end"]["refused"] == ["precondition:C3:fault_detection:unset"], _explain(run)
        assert sum(w["n"] for w in run.writes()) == 0


# --- case 5: the scan cut while driving --------------------------------------------------------------------------
def test_a_scan_cut_while_driving_latches_and_no_motion_byte_follows(mcu_model, tmp_path):
    """Plan 6.1 case 5 (Codex impl round 1 P2-test-1): a moving DRIVE accepted by the board first; then the scans stop
    -- the host latches on the scan's own evidence (a scan boundary or the scan stale), its DISARM is written and a
    DISARMED STATE of the same boot naming its seq comes back, all before the test's signal; from the latch on, no
    ARM or DRIVE byte."""
    site = Site(tmp_path / "site")
    with operating(mcu_model, tmp_path, site, duration_s=150) as run:
        assert run.wait_console() is not None, _explain(run)
        c = OperatingConsole(run.port)
        try:
            reg = _ready_to_submit(c)
            assert _confirm_pallet(c, reg)["t"] == "ack"
            assert c.ask("mission_submit", pallet_id="P-S10", destination_id="D-S10",
                         targets_revision=reg["targets_revision"])["t"] == "ack"
            end_t = time.monotonic() + 90
            moving_accepted = None
            while moving_accepted is None:
                assert time.monotonic() < end_t, "no moving DRIVE accepted by the board"
                rows = run.rows()
                speed = {q["seq"]: q["speed_mps"] for q in _events(rows, "frame_queued") if q.get("purpose") == "drive"}
                written = [w for w in _events(rows, "frame_written")
                           if w["kind"] == "DRIVE" and speed.get(w["seq"]) not in (None, 0.0)]
                acc = accept_rows(run.accept_log) if run.accept_log.exists() else []
                for w in written:
                    # the model's acceptance of that write: same boot and seq, at or after its last write's entry
                    hit = accepted(acc, "DRIVE", w["seq"], BOOT, w["last_write_ns"] or w["written_ns"])
                    if hit is not None and hit["verdict"] == "accepted":
                        moving_accepted = hit
                        break
                time.sleep(0.2)
            run.scan.cut()
            c.wait(lambda s: s["permit"]["state"] == "LATCHED", 30)
            time.sleep(1.5)  # the latch's DISARM goes out and is answered
            signal_ns = time.monotonic_ns()
        finally:
            with contextlib.suppress(Exception):
                c.close()
            run.bridge.closing()
            code = run.finish(signal.SIGINT, timeout=60)
        rows = run.rows()
        perm = _events(rows, "permit")
        latch = next((p for p in perm if p.get("event") == "latched" and p["ns"] >= run.scan.cut_ns), None)
        # the verification table's transition: the scan source's boundary after the cut, and the latch it causes
        # (impl round 2 P2-4) -- not any scan-named stop
        assert latch is not None and latch.get("reason") == "scan_generation_changed", [
            (p["event"], p.get("reason")) for p in perm if p["ns"] >= run.scan.cut_ns]
        boundary = [b for b in _events(rows, "scan_boundary") if run.scan.cut_ns <= b["ns"] <= latch["ns"]]
        assert boundary, _events(rows, "scan_boundary")[-3:]
        assert latch["ns"] < signal_ns
        from forklift_core.mcu_link.messages import Mode

        dis = [w for w in _events(rows, "frame_written") if w["kind"] == "DISARM" and latch["ns"] <= w["written_ns"]
               < signal_ns]
        assert dis, [(p["event"], p.get("reason")) for p in perm if p["ns"] >= run.scan.cut_ns]
        boot = moving_accepted["boot"]
        conf = [st for st in _events(rows, "state") if st["boot"] == boot and st["last_rx_seq"] == dis[0]["seq"]
                and st["mode"] == int(Mode.DISARMED) and st["rx_ns"] > dis[0]["written_ns"]]
        assert conf and conf[0]["rx_ns"] < signal_ns, (dis[0], conf[:1], signal_ns)
        assert _motion_bytes(run.writes(), since_ns=latch["ns"]) == 0, [
            w for w in run.writes() if w["kind"] in ("ARM", "DRIVE") and w["ns"] >= latch["ns"]][:5]
        assert code == 0, _explain(run)  # the signal's closing confirmed


# --- case 2: the command line on the repository's own settings -------------------------------------------------------
def test_the_command_line_takes_the_real_preparation_and_refuses_an_unregistered_site(tmp_path):
    """Plan 6.1 case 2 (Codex round 2 P3): every required argument given, the repository's own configuration, a site
    record the registry does not hold -> the preparation refuses (precondition:site_bundle:no_row) before the device
    is ever opened. It shows the CLI runs the operating preparation itself."""
    site_record = tmp_path / "site_record.yaml"
    site_record.write_text("site: nowhere\n")
    device = tmp_path / "no-device"
    args = [sys.executable, str(REPO / "tools/real_autonomy.py"), "--run", "--client", "real", "--device", str(device),
            "--scan-socket", str(tmp_path / "scan.sock"), "--slam-bridge", str(tmp_path / "slam.sock"),
            "--site-record", str(site_record), "--bundle-root", str(tmp_path), "--duration-s", "30",
            "--out-dir", str(tmp_path / "run")]
    p = subprocess.run(args, capture_output=True, text=True, timeout=120,
                       env={**os.environ, "PYTHONPATH": str(REPO / "src")})
    assert p.returncode == 2, p.stdout + p.stderr
    text = p.stdout
    doc = json.loads(text[text.rfind('{\n  "run"'):])
    assert doc["end"]["refused"] == ["precondition:site_bundle:no_row"], doc
    assert doc["end"]["opened"] == [] and not device.exists() and not (tmp_path / "scan.sock").exists()


# --- case 6: what the tool itself owns -----------------------------------------------------------------------------
def _try_lock(path) -> bool:
    """A competitor's attempt on the device: open it and take the executor lock (both must succeed)."""
    import fcntl

    try:
        fd = os.open(path, os.O_RDWR | os.O_NONBLOCK | os.O_NOCTTY)
    except OSError:
        return False
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return True
    except OSError:
        return False
    finally:
        os.close(fd)


def _driving(c, run):
    reg = _ready_to_submit(c)
    assert _confirm_pallet(c, reg)["t"] == "ack"
    assert c.ask("mission_submit", pallet_id="P-S10", destination_id="D-S10",
                 targets_revision=reg["targets_revision"])["t"] == "ack"
    end_t = time.monotonic() + 90
    while not _written_moving(run.rows()):
        assert time.monotonic() < end_t, "no moving DRIVE written"
        time.sleep(0.2)


def test_a_signal_while_driving_holds_the_device_until_the_disarm_is_confirmed(mcu_model, tmp_path):
    """Plan 6.1 case 6 (Codex round 2 P2-3): SIGTERM while moving DRIVEs are written -- until the closing DISARM is
    confirmed no other process can take the device's lock; once the run has let go, one can."""
    site = Site(tmp_path / "site")
    with operating(mcu_model, tmp_path, site, duration_s=150) as run:
        assert run.wait_console() is not None, _explain(run)
        c = OperatingConsole(run.port)
        tries = []
        try:
            _driving(c, run)
            run.bridge.closing()
            run.proc.send_signal(signal.SIGTERM)
            give_up = time.monotonic() + 60
            while run.proc.poll() is None:
                ok = _try_lock(run.slave_name)
                tries.append((time.monotonic_ns(), ok, run.proc.poll() is None))  # stamped after the attempt returned
                assert time.monotonic() < give_up, "the run did not end after SIGTERM"
                time.sleep(0.002)
        finally:
            with contextlib.suppress(Exception):
                c.close()
            run.bridge.closing()
            code = run.finish(timeout=60)
        after = _try_lock(run.slave_name)
        rows = run.rows()
        end = _events(rows, "end")[-1]
        assert end["closing"] == "confirmed" and end["reason"] == "signal", end
        assert code == 0, _explain(run)
        confirmed = end["closing_confirmed_ns"]
        early = [ok for ns, ok, _alive in tries if ns < confirmed]
        assert early and not any(early), tries[:5]  # never the competitor's before the DISARM was confirmed
        # the device let go by the run itself after the confirmation, while its cleanup still ran -- not by its exit
        assert any(ok and alive for ns, ok, alive in tries if ns >= confirmed), tries[-5:]
        assert after is True


def test_a_lost_device_while_driving_revokes_and_is_never_reopened(mcu_model, tmp_path):
    """Plan 6.1 case 6: the board's link goes away while moving (the model killed: the pty's other end closes) -- the
    run ends with link_lost, the permission revoked, no reopening, and a non-zero exit."""
    site = Site(tmp_path / "site")
    with operating(mcu_model, tmp_path, site, duration_s=150) as run:
        assert run.wait_console() is not None, _explain(run)
        c = OperatingConsole(run.port)
        try:
            _driving(c, run)
            run.bridge.closing()
            run.model.kill()
            run.model.wait(10)
            code = run.finish(timeout=60)
        finally:
            with contextlib.suppress(Exception):
                c.close()
        rows = run.rows()
        end = _events(rows, "end")[-1]
        assert end["closing"] == "link_lost" and end.get("device_lost"), end
        revoked = [r for r in rows if r.get("t") == "real_operating" and r.get("granted") is False]
        assert revoked and str(revoked[0]["reason"]).startswith("device_lost"), revoked
        assert code != 0, _explain(run)
        assert end["opened"] == [run.slave_name]  # opened once



def test_the_child_takes_only_the_tests_pty(tmp_path):
    """Codex U3e-1 implementation round 1 P1: the child that drops the lock and hands out a test identity runs only on
    the pty fd the test gave it -- a regular file's fd (or a real tty's) is refused before anything else."""
    f = tmp_path / "not-a-pty"
    f.write_bytes(b"")
    fd = os.open(f, os.O_RDWR)
    try:
        cfg = tmp_path / "child.json"
        cfg.write_text(json.dumps({"argv": [], "product": PRODUCT, "serial": SERIAL, "drop_lock": True, "fd": fd,
                                   "writes_log": str(tmp_path / "w.jsonl")}))
        p = subprocess.run([sys.executable, str(REPO / "tests/integration/real_operating_child.py"), str(cfg)],
                           capture_output=True, text=True, timeout=60, pass_fds=(fd,),
                           env={**os.environ, "PYTHONPATH": str(REPO / "src")})
    finally:
        os.close(fd)
    assert p.returncode == 2 and json.loads(p.stdout)["refused"] == ["child_fd_not_a_pty"], p.stdout + p.stderr
    assert not (tmp_path / "w.jsonl").exists()
