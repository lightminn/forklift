"""Replay a format-2 boundary record into the runner's drive() on CPU (docs/plans/2026-10-16-m-a1-replay.md, R4).

The setup snapshot gives drive's inputs: values are decoded, imports re-imported, services become replay
proxies (their recorded method lists tell calls from attribute reads), the planner functions become recorded
functions on the replay gate, and the Isaac adapters become replay stand-ins. drive's own recording
machinery is pointed at the replay: ``boundary`` is the replay gate (its ``recorder`` the session),
``RecordingProxy`` hands back replay proxies, ``recorded_function`` is unchanged (it calls the gate). Every
recorded call must then be made in order with the recorded request; any difference stops the replay there.

    python tools/boundary_replay.py RUN_DIR [--json OUT]     # RUN_DIR holds boundary/ and result.json
"""

from __future__ import annotations

import argparse
import importlib
import importlib.util
import json
import sys
import traceback
import types
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT / "sim/isaac"))

from forklift_core.mission import boundary_record as BR  # noqa: E402
from forklift_core.mission.replay import ReplayGate, ReplayMismatch, ReplayProxy, ReplaySession  # noqa: E402
from forklift_core.mission.replay_codec import decode  # noqa: E402


def _install_isaac_value_types() -> None:
    """The Isaac classes recorded by value, as CPU stand-ins under the same import path (decode resolves them,
    and the replayed code's own requests encode to the same qualified name)."""
    if "isaacsim.core.utils.types" in sys.modules:
        return

    class ArticulationAction:  # noqa: D401 -- the SDK's value object: the public attributes it records
        def __init__(self, joint_positions=None, joint_velocities=None, joint_efforts=None, joint_indices=None):
            self.joint_positions = joint_positions
            self.joint_velocities = joint_velocities
            self.joint_efforts = joint_efforts
            self.joint_indices = joint_indices

    ArticulationAction.__module__ = "isaacsim.core.utils.types"
    ArticulationAction.__qualname__ = "ArticulationAction"
    for name in ("isaacsim", "isaacsim.core", "isaacsim.core.utils"):
        sys.modules.setdefault(name, types.ModuleType(name))
    module = types.ModuleType("isaacsim.core.utils.types")
    module.ArticulationAction = ArticulationAction
    sys.modules["isaacsim.core.utils.types"] = module


class SessionRecorder:
    """The replay session where drive expects ``boundary.recorder``."""

    def __init__(self, session: ReplaySession) -> None:
        self.session = session
        self.value_format = 2
        self.value_types = set(session.value_types)
        self.tick_error = None

    @property
    def in_tick(self) -> bool:
        return self.session.in_tick

    def tick_begin(self, tick, **head):
        self.session.tick_begin(tick, **head)

    def tick_end(self, *, outputs, branches):
        self.session.tick_end(outputs=outputs, branches=branches)

    def tick_time(self, *, tick_t, work_t):
        self.session.tick_time(tick_t=tick_t, work_t=work_t)

    def encode(self, value):
        return self.session.encode(value)

    def close(self, final_state=None):
        self.session.closed = True


class ReplayBoundary(ReplayGate):
    def __init__(self, session: ReplaySession) -> None:
        super().__init__(session)
        self.recorder = SessionRecorder(session)


def _unrunnable(name):
    def run(*args, **kwargs):
        raise ReplayMismatch(f"{name} ran outside a recorded tick (a replay has no planner to run)")
    return run


def build_inputs(run_dir: Path, gate: ReplayBoundary):
    snap = json.loads((run_dir / "boundary/setup_snapshot.json").read_text())
    arrays = {}
    extra = run_dir / "boundary/setup_arrays.npz"
    if extra.exists():
        with np.load(extra, allow_pickle=False) as z:
            arrays = {k: z[k] for k in z.files}
    specs, services = {}, {}
    for name, item in snap["names"].items():
        if item["kind"] == "service":
            specs[item["id"]] = item.get("spec") or {}
            services[item["id"]] = ReplayProxy(item["id"], gate, spec=specs[item["id"]])
    # Services that sat inside containers only (the obstacle layer), with their recorded specs.
    for logical, spec_ in (snap.get("service_specs") or {}).items():
        specs.setdefault(logical, spec_)
    for logical, spec_ in (snap.get("service_specs") or {}).items():
        if logical not in services and logical != "Permission":
            services[logical] = ReplayProxy(logical, gate, spec=spec_,
                                            child_ports={"permission": "Permission"} if logical == "Obstacles" else None,
                                            child_specs={"permission": specs.get("Permission", {})})
    values = {}
    shared_ids: dict = {}  # one alias numbering across the snapshot's values (one Encoder wrote them in name order)
    for name, item in sorted(snap["names"].items()):
        kind = item["kind"]
        if kind == "service":
            values[name] = services[item["id"]]
        elif kind == "import":
            module, _, qual = item["path"].partition(":")
            try:
                obj = importlib.import_module(module)
            except ModuleNotFoundError:
                # A sibling module the runner loaded by file under its own name: load it the same way.
                file_ = item.get("file")
                if not file_:
                    raise
                path_ = Path(file_) if Path(file_).is_absolute() else ROOT / file_
                spec_ = importlib.util.spec_from_file_location(module, path_)
                obj = importlib.util.module_from_spec(spec_)
                sys.modules[module] = obj
                spec_.loader.exec_module(obj)
            for part in qual.split(".") if qual else ():
                obj = getattr(obj, part)
            values[name] = obj
        elif kind == "port":
            values[name] = BR.recorded_function(_unrunnable(name), item["port"], item["method"], gate)
        elif kind == "value":
            values[name] = decode(item["node"], arrays, services=services, by_id=shared_ids)
        elif kind in ("adapters", "excluded"):
            continue
        else:
            raise ValueError(f"unknown snapshot kind {kind} for {name}")
    specs_by_port = dict(specs)

    def proxy_factory(target, port, gate_, **kw):
        if isinstance(target, ReplayProxy):
            spec = object.__getattribute__(target, "_spec")
        elif isinstance(target, types.ModuleType):  # the runner's `time` module as the Clock port
            spec = {"methods": sorted(n for n in dir(target) if callable(getattr(target, n, None)))}
        else:
            spec = specs_by_port.get(port, type(target))
        child_specs = {name: specs_by_port.get(child, {}) for name, child in (kw.get("child_ports") or {}).items()}
        return ReplayProxy(port, gate, spec=spec, child_ports=kw.get("child_ports"), child_specs=child_specs,
                           hash_only_requests=kw.get("hash_only_requests", ()), watches=kw.get("watches"))

    values["RecordingProxy"] = proxy_factory
    values["recorded_function"] = BR.recorded_function
    values["boundary"] = gate
    _install_isaac_value_types()
    action_type = sys.modules["isaacsim.core.utils.types"].ArticulationAction
    adapters = types.SimpleNamespace(
        action=lambda **kw: action_type(**kw),
        add_path_display=lambda *a, **k: None,
        export_scene=lambda: None,
        spawn_box=lambda *a, **k: None,
        remove_prim=lambda *a, **k: None,
        slam_link_client=lambda *a, **k: types.SimpleNamespace(close=lambda: None),
        cast_reference_scan=_unrunnable("cast_reference_scan"),
        place_chase_camera=lambda *a, **k: None,
    )
    return types.SimpleNamespace(**values), adapters, snap


def replay_run(run_dir: Path) -> dict:
    run_dir = Path(run_dir)
    _install_isaac_value_types()
    session = ReplaySession(run_dir / "boundary", allow_opaque={("Planner", "redraw_docking")})
    gate = ReplayBoundary(session)
    # The runner module first: the sibling modules it loads by file (perception adapter, near tracking ...) are
    # what the snapshot's imports name.
    spec = importlib.util.spec_from_file_location("run_transport_replay", ROOT / "sim/isaac/run_transport.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    S, adapters, snap = build_inputs(run_dir, gate)
    module._BOUNDARY_RECORD["gate"] = gate
    outcome = {"run_dir": str(run_dir), "ticks_done": 0}
    try:
        module.drive(S, adapters)
        outcome["drive"] = "returned"
    except ReplayMismatch as exc:
        outcome["drive"] = f"mismatch: {exc}"
        context = exc.__context__
        if context is not None:  # what the replayed run raised first (a require, a crash) before this mismatch
            outcome["context"] = f"{type(context).__name__}: {context}"
            outcome["context_traceback"] = "".join(
                traceback.format_exception(type(context), context, context.__traceback__))[-4000:]
    except Exception as exc:  # noqa: BLE001 -- the replayed run's own end (a require) is part of the record
        outcome["drive"] = f"{type(exc).__name__}: {exc}"
        outcome["traceback"] = traceback.format_exc()[-4000:]
    if session.failure is None and outcome.get("drive") == "returned":
        try:
            session.finish()  # every recorded tick replayed, as many as the meta says
            outcome["finish"] = "ok"
        except ReplayMismatch as exc:
            outcome["finish"] = str(exc)
    outcome["ticks_done"] = session.ticks_done
    outcome["failure"] = session.failure
    outcome["tick_open"] = session.tick["tick"] if session.tick else None
    return outcome


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--json", type=Path)
    args = parser.parse_args(argv)
    out = replay_run(args.run_dir)
    text = json.dumps(out, indent=1)
    if args.json:
        args.json.write_text(text + "\n")
    print(text)
    return 0 if out.get("failure") is None and out.get("finish") == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
