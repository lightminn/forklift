#!/usr/bin/env python3
"""The real truck's autonomy executor (docs/design/2026-11-19-real-assembly.md, docs/design/2026-11-20-real-hil-6a.md).

``--dry-assemble`` (4b): the core is assembled from the repository's real settings without opening a device, the
assembly record written, and the exit code says whether the assembly is structurally sound (0) or refused (2).
Readiness needs the link, the SLAM link and the layer, which a dry assembly does not have: it is recorded as not
ready, not as an error.

``--run --hil`` (6a-2b, design 10.3): the wall-clock run on the MCU model behind a pty this process makes, with the
synthetic sources (scan, SLAM, IMU) of ``forklift_core.live.real_run`` -- the one assembly the real-input run (PR 7)
reuses with its sources swapped. ``--run`` with a device (a real MCU) is refused before anything is opened: the real
MCU's AUTO waits for S10's preconditions. The loaders, the geometry and the checks are the dry assembly's
(``real_run``).

``--run --client real --device D`` (the operating unlock U1, docs/plans/2026-11-26-real-operating-assembly.md 2.1,
2.4): the real board opened with the executor lock on its fd, its USB identity read and its first STATE waited for --
nothing is ever written -- then S10's operating preconditions judged and printed. Until the unlock's last PR the list
always holds ``precondition:unlock_incomplete`` and the run exits 2 without assembling."""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace

REPO = Path(__file__).resolve().parents[1]

from forklift_core.live.real_run import map_occupancy, mission_geometry  # noqa: E402,F401 -- re-exported (tests)
from forklift_core.live.real_loop import mission_counts  # noqa: E402 -- the end record's F1/validation counts

EXIT_FAILED = 1


def _sha(path: Path) -> str:
    import hashlib

    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dry_assemble(s10a_path: Path, settings_path: Path, vehicle_path: Path, map_bundle: Path | None,
                 out: Path | None, watch_path: Path | None = None):
    import yaml

    from forklift_core.live import real_assembly as asm
    from forklift_core.live import real_autonomy as ra
    from forklift_core.live import real_run as rr
    from forklift_core.plant.real_vehicle import chassis_model_path

    record = {"s10a": str(s10a_path), "settings": str(settings_path), "vehicle": str(vehicle_path),
              "map_bundle": None if map_bundle is None else str(map_bundle)}
    try:
        record["hashes"] = {name: _sha(path) for name, path in (("s10a", s10a_path), ("settings", settings_path),
                                                                 ("vehicle", vehicle_path))}
        cfg = ra.load_s10a(s10a_path)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as err:
        record.update(assembled=False, refused=[f"settings_unreadable:{err}"])
        _write(record, out)
        return 2
    try:
        data = rr.read_settings(settings_path)
        occupancy, map_id = None, None
        if map_bundle is not None:
            map_id, occupancy = rr.read_map(map_bundle, cfg)
        vehicle, kin = rr.read_vehicle(vehicle_path)
        # the watches as the live assembly builds them (safety wiring design 4: one build_watches for both)
        watch_path = REPO / "config/real/safety_watch.yaml" if watch_path is None else watch_path
        watches = asm.build_watches(data, watch_path, vehicle.wheel_radius_m)
    except asm.AssemblyRefused as err:
        record.update(assembled=False, refused=err.reasons)
        _write(record, out)
        return 2
    record["watch"] = {"path": str(watch_path), "sha256": _sha(watch_path), "imu": watches.imu_policy}
    settings, planner, cap = rr.planning_settings(data, kin)
    try:
        if cfg.pallet_depth_m is None or cfg.pallet_width_m is None:
            raise asm.AssemblyRefused(ra.s10a_refusal(cfg, map_id) + (["check_pallet_pose_unset"]
                                                                       if cfg.check_pallet_pose is None else []))
        urdf = chassis_model_path(vehicle_path).parent / "forklift.urdf"
        geometry = rr.mission_geometry(urdf, cfg.pallet_depth_m, cfg.pallet_width_m)
        _, rear_offset, _ = rr.chassis_reference(urdf)
        hooks = asm.RealHooks(lift_m=lambda: float("nan"), odom_base=lambda: None, ready=lambda t, i: None,
                              started=lambda *a: None, record=lambda *a: None, geometry=geometry)
        core = asm.assemble(
            cfg=cfg, loaded_map_id=map_id, geometry=geometry, pallet_geometry=None, kinematics=kin,
            settings=settings, planner_config=planner, travel_config=planner, speeds=data["speeds"],
            mission_stages=data["mission_stages"], curvatures={"preview": cap},
            layer=SimpleNamespace(known_enabled=False), applied=(0.0, 0.0, 0.0),
            slam_tracker=SimpleNamespace(mode="none", applied=None), jobs=None, hooks=hooks,
            extra_options=rr.core_options(settings, rear_offset))
    except asm.AssemblyRefused as err:
        record.update(assembled=False, refused=err.reasons)
        _write(record, out)
        return 2
    plan = rr.check_plan(cfg, planner, geometry, occupancy)
    record.update(asm.assembly_record(core, cfg, {"planner": cap, "tracker": cap, "preview": cap}),
                  assembled=True, check_plan=plan.status, ready="not_ready:link:no_state (dry assembly)")
    _write(record, out)
    return 0 if plan.success else 2


def run_refused(device: str | None, out: Path | None) -> int:
    """``--run`` without ``--hil``: nothing is opened. A device is refused by name before any transport exists (the
    real MCU's AUTO waits for S10's preconditions); without one the run names the HIL path."""
    reason = "device_refused:s10_preconditions" if device is not None else "run_needs_hil"
    _write({"run": False, "device": device, "refused": [reason], "opened": []}, out)
    return 2


REAL_FIRST_STATE_S = 2.0


def run_real_precheck(args) -> int:
    """U1's real path (plan 2.1, 2.4): open, read the identity and the first STATE, judge, print, close. The transport
    is never granted and nothing is written to the board."""
    sys.path.insert(0, str(REPO / "tools"))
    import real_loop_transport as rlt
    import yaml

    from forklift_core.live import real_preconditions as rp
    from forklift_core.live import real_run as rr
    from forklift_core.live.executor_lock import DeviceBusy
    from forklift_core.live.real_loop import real_host
    from forklift_core.live.synthetic import SyntheticRefused, device_identity  # noqa: F401 -- the pty refusal below
    from forklift_core.mcu_link.build_policy import Client

    record = {"run": False, "client": "real", "device": args.device, "opened": []}
    if args.out is not None and not _plain_output(args.out, args.device):
        # the record never goes to a device (the board itself or an alias of it): nothing is written anywhere
        _write({**record, "refused": ["out_not_a_regular_file"]}, None)
        return 2
    if args.device is None:
        record["refused"] = ["real_needs_device"]
        _write(record, args.out)
        return 2
    if _is_pty(args.device):
        record["refused"] = ["real_device_is_pty"]  # a real MCU never links by pty (the model's bench is HIL's)
        _write(record, args.out)
        return 2
    try:
        tr = rlt.RealTtyTransport.open(args.device)
    except DeviceBusy as busy:
        record["refused"] = [f"device_busy:{busy}"]
        _write(record, args.out)
        return 2
    except OSError as err:
        record["refused"] = [f"device_unopenable:{type(err).__name__}:{err}"]
        _write(record, args.out)
        return 2
    record["opened"] = [args.device]
    try:
        host = real_host(rr.RUN_HOST_CFG, client=Client.REAL)
        record["identity_events"] = [{"kind": e.kind, **dict(e.detail)} for e in host.link_identity(tr.usb_product)]
        end = time.monotonic() + REAL_FIRST_STATE_S
        while host.state is None and time.monotonic() < end:
            try:
                tr.receive(host, time.monotonic_ns())
            except rlt.DeviceLost as lost:
                record["refused"] = [f"device_lost:{lost}"]
                _write(record, args.out)
                return 2
            time.sleep(0.005)
        board = rr.board_facts(host, tr)
        record["board"] = board
        try:
            facts = rp.gather(record_path=args.preconditions, lidar_path=args.lidar_config, imu_path=args.imu_config,
                              settings_path=args.settings, s10a_path=args.s10a or REPO / "config/real/s10a.yaml",
                              layer_path=REPO / "config/real/obstacle_layer_real.yaml",
                              budget_path=REPO / "config/real/obstacle_layer.yaml", vehicle_path=args.vehicle,
                              watch_path=args.watch, board=board)
            record["refused"] = rp.operating_preconditions(facts)
            record["imu_qualified"] = rp.imu_qualified(facts)
        except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as err:
            record["refused"] = [rp.UNLOCK_INCOMPLETE, f"precondition:facts_unreadable:{type(err).__name__}:{err}"]
        _write(record, args.out)
        return 2 if record["refused"] else 0
    finally:
        os.close(tr.fd)


def _plain_output(out: Path, device: str | None) -> bool:
    """``--out`` may be a regular file or a new name -- never a device, a FIFO or the board's own node."""
    import stat as stat_

    try:
        st = os.stat(out)
    except FileNotFoundError:
        return True
    except OSError:
        return False
    if not stat_.S_ISREG(st.st_mode):
        return False
    if device is not None:
        try:
            dev = os.stat(device)
            return not (dev.st_ino == st.st_ino and dev.st_dev == st.st_dev)
        except OSError:
            return True
    return True


def _is_pty(device: str) -> bool:
    import stat as stat_

    from forklift_core.live import model_peer

    try:
        st = os.stat(device)
    except OSError:
        return False
    return stat_.S_ISCHR(st.st_mode) and os.major(st.st_rdev) in model_peer.PTY_SLAVE_MAJORS


def _write(record: dict, out: Path | None) -> None:
    text = json.dumps(record, indent=2, default=str)
    if out is not None:
        out.write_text(text)
    print(text, flush=True)


# --- the HIL run (design 10.3) ---------------------------------------------------------------------------------------
INJECTIONS = ("scan", "scan_lost", "slam", "tx", "tx_keep", "tx_partial", "tx_release", "axis_push")


def parse_injections(items) -> list:
    """``name:ms`` -- ms after the first moving DRIVE this run wrote (every injection starts with a non-zero command
    live, design 6); refused before anything is opened."""
    out = []
    for item in items or ():
        name, _, ms = str(item).partition(":")
        if name not in INJECTIONS or not ms.isdigit():
            raise ValueError(f"hil_inject_invalid:{item}")
        out.append((name, int(ms)))
    return sorted(out, key=lambda x: x[1])


class Injector:
    """The test-only injections (design 6, 10.2, 10.8), each ``ms`` after the first moving normal DRIVE this run wrote,
    recorded with the other sources' newest sample times. ``tx`` cuts every write until the closing, ``tx_keep``
    through the closing too (an undelivered closing DISARM); ``tx_partial`` arms a condition: the first moving normal
    DRIVE written after a DRIVE the model took (the STATE's ``last_rx_seq`` naming a DRIVE this run wrote) gets its
    first 4 bytes out, and the rest of that frame only meets EAGAIN until ``tx_release`` -- the poison and the DISARM
    after its abandonment pass."""

    def __init__(self, items, run, record, model_pid=None) -> None:
        self.todo, self.run, self.record, self.model_pid = list(items), run, record, model_pid
        self.push_armed, self.pushed = False, False  # axis_push: SIGUSR1 to the model at the next holding frame
        self.t0 = None  # the first moving DRIVE's write (host ns)
        self.tx_cut = self.tx_keep = False
        self.partial_armed = False
        self.partial_seq = None
        self.released = False
        self.drives_written: set = set()
        run.sources.transport.inject = self._inject

    def _booked(self, seq):
        return next((f for f in self.run.loop.book.frames if f.seq == (int(seq) & 0xFFFF)), None)

    def _inject(self, fd, data, frame):
        if self.tx_cut or self.tx_keep:
            return 0  # every write meets EAGAIN (the send alone cut; STATEs go on)
        if frame is None or not self.partial_armed or self.released:
            return None
        seq = int(frame.seq) & 0xFFFF
        if self.partial_seq is None:
            if frame.kind != "DRIVE" or len(data) != len(frame.data):
                return None  # only a first write starts it
            host = self.run.sources.host
            booked = self._booked(seq)
            last = None if host.state is None else int(host.state.last_rx_seq) & 0xFFFF
            if booked is None or booked.kind != "drive" or float(booked.speed_mps) == 0.0 \
                    or last not in self.drives_written:
                return None
            n = os.write(fd, data[:4])
            if n > 0:  # a 0-byte or refused first write is no half frame (round 1 P2)
                self.partial_seq = seq
                self.record.put({"t": "inject_partial", "seq": seq, "bytes": int(n), "ns": time.monotonic_ns(),
                                 "after_accepted": last})
            return n
        return 0 if seq == self.partial_seq else None  # the rest of that frame only

    def note_written(self, kind, seq, written_ns) -> None:
        if kind != "DRIVE":
            return
        self.drives_written.add(int(seq) & 0xFFFF)
        booked = self._booked(seq)
        if self.push_armed and not self.pushed and booked is not None and booked.kind in ("hold", "stop") \
                and self.model_pid is not None:
            # design 2 (가): the plant pushed while a position-holding frame holds it (the model's own options say how
            # far); the watch sees it only through the next STATEs
            os.kill(self.model_pid, signal.SIGUSR1)
            self.pushed = True
            self.record.put({"t": "inject_push", "ns": int(written_ns), "seq": int(seq) & 0xFFFF})
        if self.t0 is None and booked is not None and booked.kind == "drive" and float(booked.speed_mps) != 0.0:
            self.t0 = int(written_ns)
            self.record.put({"t": "inject_t0", "ns": self.t0, "seq": int(seq) & 0xFFFF})

    def tick(self, now_ns: int) -> None:
        while self.t0 is not None and self.todo and now_ns >= self.t0 + self.todo[0][1] * 1_000_000:
            name, ms = self.todo.pop(0)
            src = self.run.sources
            if name == "scan":
                src.scans.stop_delivery = True
            elif name == "scan_lost":
                src.scans.events.append(("lost", src.scans.sensor))
            elif name == "slam":
                src.slam.stop_delivery = True
            elif name == "tx":
                self.tx_cut = True
            elif name == "tx_keep":
                self.tx_keep = True
            elif name == "tx_partial":
                self.partial_armed = True
            elif name == "tx_release":
                self.released = True
            elif name == "axis_push":
                self.push_armed = True
            loop = self.run.loop
            self.record.put({"t": "inject", "name": name, "ms": ms, "ns": int(now_ns),
                             "last_scan_s": dict(self.run.layer.last_scan_s),
                             "scan_delivered_s": getattr(src.scans, "last_delivered_s", None),
                             "slam_fresh_ns": getattr(loop, "_fresh_ns", None),
                             "state_age_s": src.host.health(int(now_ns)).state_age_s})


def _first_state(host, transport, pty, stop, rlt) -> str | None:
    """Wait up to HIL_FIRST_STATE_S for a STATE of a verified HIL board (design 1 ①)."""
    end = time.monotonic() + rlt.HIL_FIRST_STATE_S
    while time.monotonic() < end:
        if stop.is_set():
            return "signal"
        if pty.poll() is not None:
            transport.peer_exited()
            return f"device_lost:model_exit:{pty.poll()}"
        try:
            transport.receive(host, time.monotonic_ns())
        except Exception as err:  # noqa: BLE001 -- a lost device
            lost = getattr(transport, "lost", None)
            return lost["reason"] if lost else f"receive:{type(err).__name__}"
        if transport.hil_plant:
            return None
        if host.state is not None:
            return f"hil_refused:{int(host.state.flags)}"
        time.sleep(0.005)
    return "hil_no_state"


def run_hil(args) -> int:
    """10.3's order: ① arguments (before anything is opened) ② preparation (before the model; a signal cancels it)
    ③ the pty and the model ④ the host and the transport, the first STATE ⑤ the sources ⑥ the assembly ⑦ the console
    ⑧ the session ⑨ the run -- then the closing, and the cleanup with the model first."""
    sys.path.insert(0, str(REPO / "tools"))
    import real_loop_transport as rlt

    refused = []
    if args.device is not None:
        refused.append("device_with_hil")
    if args.hil_model is None:
        refused.append("hil_model_missing")
    if args.duration_s is None or not 0 < float(args.duration_s) <= 3600:
        refused.append("duration_s")
    if args.out_dir is None:
        refused.append("out_dir_required")  # a run keeps its record (round 1 P2)
    model_args = list(args.hil_model_arg or ())
    refused += [f"hil_model_arg_reserved:{a}" for a in model_args if str(a) in rlt.RESERVED_MODEL_ARGS]
    try:
        injections = parse_injections(args.hil_inject)
    except ValueError as err:
        refused.append(str(err))
        injections = []
    if args.hil_slam_bridge is not None and any(name == "slam" for name, _ in injections):
        refused.append("hil_inject_slam_with_bridge")  # the real source has no stop switch: stop the bridge instead
    if args.replay is not None:
        # PR 7 7r plan 4: the replay's SLAM is the real source; its scans have no injection hooks
        if args.hil_slam_bridge is None:
            refused.append("replay_needs_slam_bridge")
        refused += [f"replay_inject:{name}" for name, _ in injections if name in ("scan", "scan_lost")]
    if refused:
        _write({"run": False, "refused": refused, "opened": []}, None)
        return 2
    stop = threading.Event()
    old = {s: signal.signal(s, lambda *_: stop.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        return _run_hil(args, rlt, stop, model_args, injections)
    finally:
        for s, h in old.items():
            signal.signal(s, h)


def _run_hil(args, rlt, stop, model_args, injections) -> int:
    from forklift_core.live import real_run as rr
    from forklift_core.live.real_assembly import AssemblyRefused

    paths = rr.RunPaths(s10a=args.s10a, settings=args.settings, vehicle=args.vehicle, watch=args.watch,
                        map_bundle=args.map_bundle, hil=True, replay=args.replay, lidar_config=args.lidar_config,
                        imu_config=args.imu_config, allow_synthetic=bool(args.allow_synthetic))
    try:
        prep = rr.prepare_run(paths, stop=stop)
    except AssemblyRefused as err:
        _write({"run": False, "refused": err.reasons}, None)
        return 2
    except rr.RunCancelled:
        _write({"run": False, "end": "signal_while_preparing"}, None)
        return EXIT_FAILED
    out_dir = Path(args.out_dir)
    try:
        out_dir.mkdir(parents=True, exist_ok=True)
        record = rr.RunRecord(out_dir / "run.jsonl")
    except OSError as err:
        prep.jobs.close()
        _write({"run": False, "end": f"record_unopenable:{err}"}, None)
        return rr.EXIT_RECORD
    held = SimpleNamespace(pty=None, server=None, run=None, sources=None)
    end = {"t": "end", "reason": None}
    code = EXIT_FAILED
    try:
        code = _hil_body(args, rlt, rr, AssemblyRefused, prep, record, held, end, stop, model_args, injections)
    except Exception as err:  # noqa: BLE001 -- recorded as the end; the cleanup below still runs, model first
        import traceback

        traceback.print_exc()
        end["reason"] = end["reason"] or f"exception:{type(err).__name__}:{err}"
        code = EXIT_FAILED
    finally:
        ok = _cleanup(held, prep, record, end)
    if not ok or end.get("model_exit") == 4:
        return rr.EXIT_RECORD  # the host's record or the model's log is not complete (round 1 P2)
    if code == rr.EXIT_OK and end.get("model_exit") != 0:
        return EXIT_FAILED  # the model ended other than by the closing's own stop
    return code


def _hil_body(args, rlt, rr, AssemblyRefused, prep, record, held, end, stop, model_args, injections) -> int:
    from forklift_core.live.real_loop import real_host
    from forklift_core.live.synthetic import SyntheticRefused
    from forklift_core.mcu_link.build_policy import Client

    # ③ the pty and the model (the model's own end: the duration + HIL_MODEL_EXTRA_S from here)
    try:
        held.pty = rlt.HilPty(args.hil_model, float(args.duration_s), model_args)
    except rlt.HilRefused as err:
        end["reason"] = str(err)
        return 2
    model_start = time.monotonic()
    record.put(rr.header_row(prep, synthetic=True, uninspected=held.pty.uninspected, model_args=model_args,
                             extra={"model": str(args.hil_model), "model_sha256": _sha(Path(args.hil_model)),
                                    "model_pid": held.pty.proc.pid,
                                    "duration_s": float(args.duration_s), "injections": injections,
                                    # 7r plan 4: what was replayed and the output marks
                                    **({} if prep.replay is None else prep.replay.head())}))
    # ④ the host and the transport: the first STATE of a verified HIL board
    host = real_host(rr.RUN_HOST_CFG, client=Client.HIL)
    transport = rlt.LoopTtyTransport(held.pty.slave)
    transport.on_event = record.put
    why = _first_state(host, transport, held.pty, stop, rlt)
    if why is not None:
        end["reason"] = why
        return EXIT_FAILED
    # ⑤ the sources ⑥ the assembly (the same host and transport)
    try:
        held.sources = sources = rr.make_run_sources(prep, host, transport, slam_socket=args.hil_slam_bridge)
        held.run = run = rr.assemble_run(prep, sources, events=record.put)
    except (AssemblyRefused, SyntheticRefused) as err:
        end["reason"] = f"refused:{getattr(err, 'reasons', err)}"
        return 2
    # ⑦ the console, within what is left of the startup budget (a late bind closes itself on the cancel flag)
    if str(REPO) not in sys.path:
        sys.path.insert(0, str(REPO))  # console.live, as tools/teleop_bridge.py
    from console.live.server import LiveServer, MediaStore

    cancel = threading.Event()
    held.server = server = LiveServer(prep.channels, MediaStore(), prep.welcome, host=args.host, port=args.port,
                                      web_root=Path(args.web_root) if args.web_root else None, cancel=cancel)
    left = rlt.HIL_STARTUP_MAX_S - (time.monotonic() - model_start)
    try:
        if left <= 0:
            raise RuntimeError("no time left")
        server.start(timeout_s=left)
    except RuntimeError:
        cancel.set()
        server._ready.wait(2.0)  # noqa: SLF001 -- a bind after the flag closes itself
        end["reason"] = "hil_startup_slow"
        return EXIT_FAILED
    if time.monotonic() - model_start > rlt.HIL_STARTUP_MAX_S:
        end["reason"] = "hil_startup_slow"
        return EXIT_FAILED
    print(json.dumps({"console": f"http://{args.host}:{server.port}/", "port": server.port,
                      "session_epoch": prep.epoch}), flush=True)
    # ⑧ the session (its first DISARM at once) ⑨ the run -- from the session's first act on, every exit goes through
    # the closing below (round 2 P3)
    inj = None
    try:
        now = time.monotonic_ns()
        run.start_session(now)
        record.put({"t": "session", "ns": now})
        if injections:
            inj = Injector(injections, run, record, model_pid=held.pty.proc.pid)
            put = run.loop.events

            def events(ev):
                if ev.get("t") == "frame_written":
                    inj.note_written(ev.get("kind"), ev.get("seq"), ev.get("written_ns"))
                put(ev)

            run.loop.events = events
        reason = run.run(float(args.duration_s), stop=stop, model_exit=held.pty.poll, record=record,
                         before_tick=None if inj is None else inj.tick)
    except Exception as err:  # noqa: BLE001 -- the closing below still runs while the link lives (rounds 1-2)
        import traceback

        traceback.print_exc()
        reason = f"exception:{type(err).__name__}:{err}"
    end["reason"] = reason
    if inj is not None:
        inj.tx_cut = False  # the cut send was the injection; the closing's DISARM goes out (tx_keep keeps it cut)
    try:
        closing = run.close(record=record)
    except Exception as err:  # noqa: BLE001
        closing = f"closing_exception:{type(err).__name__}:{err}"
    loop = run.loop
    end.update(closing=closing, closing_started_ns=loop.closing_started_ns, closing_confirmed_ns=loop.closing_confirmed_ns,
               closing_disarms=[list(d) for d in loop.closing_disarms],
               disarm_decisions=[list(d) for d in loop.disarm_decisions],
               axis_watch_times=[list(d) for d in loop.axis_watch_times], results=list(loop.session.results),
               permit=loop.permit.state, permit_reason=loop.permit.reason, counts=dict(sources.counts),
               slam_counts=dict(getattr(sources.slam, "counts", {}) or {}),
               reloc_dropped=dict(loop.reloc_dropped),
               # PR 7 design 6.1, 7 (7c-2b): the seeds, the F1 block and its events, and the missions counted for the
               # F1 and validation figures (f1-bypassed ones left out)
               seed_log=[dict(r) for r in getattr(loop, "seed_log", [])],
               f1_block=None if getattr(loop, "f1_block", None) is None else dict(loop.f1_block),
               reloc_events=[dict(e) for e in getattr(loop, "reloc_events", [])],
               mission_counts=mission_counts(loop.session.results),
               ticks=run.ticks, records_kept=len(loop.records),
               hil_check_round=getattr(loop, "hil_check_round", None))
    if sources.replay is not None:
        end["replay"] = sources.replay.summary()  # 7r plan 4: the replay's counts
        end["replay"]["layer"] = dict(getattr(run.layer, "counts", {}) or {})  # scans taken, dropped, installs
    ok_reasons = ("deadline", "signal", "replay_eof")
    return rr.EXIT_OK if closing == "confirmed" and reason in ok_reasons else EXIT_FAILED


def _cleanup(held, prep, record, end) -> bool:
    """The model first (its exit code completes a device loss's reason), then the server, the workers (the plan
    worker last) and the record; a step that fails does not skip the next. True when the record is complete."""
    model_code = late = None
    try:
        if held.pty is not None:
            model_code = held.pty.close()
    finally:
        try:
            if held.server is not None:
                end["server_stopped"] = held.server.stop()
        finally:
            try:
                if held.run is not None:
                    late = held.run.close_workers()
                else:
                    # no run assembled: what the sources hold (the SLAM bridge's socket) closed here (7c-1b-2b)
                    for fn in (() if held.sources is None else held.sources.closers):
                        try:
                            fn()
                        except Exception:  # noqa: BLE001
                            pass
                    if prep.jobs is not None:
                        prep.jobs.close()
            finally:
                tr = None if held.run is None else held.run.sources.transport
                lost = None if tr is None else tr.lost
                if lost is not None:
                    end["device_lost"] = (f"device_lost:model_exit={model_code}:"
                                          f"stream_tainted={int(lost['stream_tainted'])}:first={lost['first']}")
                end.update(model_exit=model_code, workers_late=late)
                ok = record.close(end)
                _write({"run": True, "end": end, "record_complete": ok}, None)
                if not ok:
                    print(f"record incomplete: {record.incomplete or record.failed}", file=sys.stderr)
    return ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    mode = ap.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-assemble", action="store_true")
    mode.add_argument("--run", action="store_true", help="the wall-clock run: --hil only (a device is refused)")
    ap.add_argument("--hil", action="store_true", help="run on the MCU model behind this process's own pty")
    ap.add_argument("--hil-model", default=None, help="the mcu_model executable")
    ap.add_argument("--hil-model-arg", action="append", default=None,
                    help="one argument for the model (repeat; --hil-model-arg=--accept-log --hil-model-arg=PATH)")
    ap.add_argument("--hil-inject", action="append", default=None,
                    help="tests only: name:ms after the first moving DRIVE written "
                         "(scan|scan_lost|slam|tx|tx_keep|tx_partial|tx_release|axis_push)")
    ap.add_argument("--hil-slam-bridge", default=None,
                    help="--hil: the real SLAM source on a real-mode bridge's Unix socket (PR 7 7c-1b-2b) instead of "
                         "the synthetic SLAM")
    ap.add_argument("--replay", type=Path, default=None,
                    help="--run --hil --hil-slam-bridge: an observe record's LiDAR and IMU replayed behind the HIL run "
                         "(PR 7 7r; refused unless --lidar-config / --imu-config carry measured bounds)")
    ap.add_argument("--lidar-config", type=Path, default=REPO / "config/real/lidar.yaml",
                    help="--replay: the LiDAR config the layer and SLAM both take")
    ap.add_argument("--imu-config", type=Path, default=REPO / "config/real/imu.yaml",
                    help="--replay: the IMU source config")
    ap.add_argument("--allow-synthetic", action="store_true",
                    help="--replay: a synthetic record may be replayed (the output is marked not_for_judgement)")
    ap.add_argument("--duration-s", type=float, default=None, help="--hil: required, 0 < s <= 3600")
    ap.add_argument("--device", default=None, help="--run --client real: the real board's tty (U1: a precheck only)")
    ap.add_argument("--client", choices=("hil", "real"), default=None,
                    help="--run --client real: open the real board read-only and judge S10's operating preconditions")
    ap.add_argument("--preconditions", type=Path, default=REPO / "config/real/s10_preconditions.yaml",
                    help="--client real: S10's human-confirmed precondition record")
    ap.add_argument("--s10a", "--config", dest="s10a", type=Path, default=None,
                    help="the S10-A file (default config/real/s10a.yaml; --run --hil: config/real/s10a_hil.yaml)")
    ap.add_argument("--settings", type=Path, default=REPO / "config/real/autonomy.yaml")
    ap.add_argument("--vehicle", type=Path, default=REPO / "config/real/vehicle.yaml")
    ap.add_argument("--watch", type=Path, default=REPO / "config/real/safety_watch.yaml")
    ap.add_argument("--map-bundle", type=Path, default=None,
                    help="the surveyed map bundle (verified by its manifest; its map id must be s10a.yaml's)")
    ap.add_argument("--out", type=Path, default=None)
    ap.add_argument("--out-dir", type=Path, default=None, help="--run: the run record's folder (run.jsonl), required")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=0)
    ap.add_argument("--web-root", default=str(REPO / "console/web"), help="the console's pages (as the teleop bridge)")
    args = ap.parse_args(argv)
    if args.replay is not None and not (args.run and args.hil):
        _write({"run": False, "refused": ["replay_needs_run_hil"], "opened": []}, args.out)
        return 2
    if args.run:
        if args.client == "real":
            if args.hil:
                _write({"run": False, "refused": ["real_with_hil"], "opened": []}, args.out)
                return 2
            return run_real_precheck(args)
        if not args.hil:
            return run_refused(args.device, args.out)
        if args.s10a is None:
            args.s10a = REPO / "config/real/s10a_hil.yaml"
        return run_hil(args)
    if args.device is not None or args.hil:
        return run_refused(args.device or "hil", args.out)
    return dry_assemble(args.s10a or REPO / "config/real/s10a.yaml", args.settings, args.vehicle, args.map_bundle,
                        args.out, args.watch)


if __name__ == "__main__":
    sys.exit(main())
