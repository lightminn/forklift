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


# Names today's setup binds on every run, with the value it binds when their feature is off. A snapshot recorded
# before the name existed has none of them: that run had no such feature, so the name is bound as the setup binds
# it off (drive's prologue would otherwise leave it unbound, unlike any setup now). Recorded with the feature on,
# the snapshot holds the name. (PR #53 far recognition: the 29 gate records predate it.)
SETUP_NAMES_ADDED = {
    "far_runtime": ("recognition", "off", None),  # (args flag, its off value, the name's value then)
    "far_intrinsics": ("recognition", "off", None),
    "frr": ("recognition", "off", None),
    # SLAM localization plan L7 (2026-10-21): every earlier record mapped online, with no survey bundle
    "slam_bundle_": ("slam_map_bundle", None, None),
    # Trailer exit plan (2026-10-22): every earlier record planned its transports with no exit prefix
    "transport_exit": ("exit_clearance_m", None, None),
}


def bind_added_names(values: dict, names: dict) -> list[str]:
    """Bind the SETUP_NAMES_ADDED a snapshot lacks; a lacking name whose recorded flag is not off is refused."""
    added = []
    args_ = values.get("args")
    for name, (flag, off, value) in SETUP_NAMES_ADDED.items():
        if name in names:
            continue
        recorded = getattr(args_, flag, off)  # a record from before the flag has none: it ran without it
        if recorded != off:
            raise ReplayMismatch(f"the setup snapshot has no {name} although {flag}={recorded!r}")
        values[name] = value
        added.append(name)
    return added


class _SiblingFinder:
    """The runner loads some sibling modules by file under ``run_transport_<stem>`` only inside its setup (the
    new-obstacle schedule); a record's values name their classes by that module: found here the same way."""

    @staticmethod
    def find_spec(name, path=None, target=None):
        if not name.startswith("run_transport_"):
            return None
        file_ = ROOT / "sim/isaac" / (name[len("run_transport_"):] + ".py")
        return importlib.util.spec_from_file_location(name, file_) if file_.is_file() else None


if not any(isinstance(f, _SiblingFinder) or f is _SiblingFinder for f in sys.meta_path):
    sys.meta_path.append(_SiblingFinder)


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

    def tick_time(self, *, tick_t, work_t, commands=None, stop_kind=None):
        # M-a3d: the runner also passes the mission commands applied and the stop kind (None without commands).
        self.session.tick_time(tick_t=tick_t, work_t=work_t, commands=commands, stop_kind=stop_kind)

    def encode(self, value):
        return self.session.encode(value)

    def close(self, final_state=None, ending=None):
        self.session.closed = True
        self.ending = ending
        self.final_state = final_state


class ReplayBoundary(ReplayGate):
    def __init__(self, session: ReplaySession) -> None:
        super().__init__(session)
        self.recorder = SessionRecorder(session)


def _unrunnable(name, session=None):
    def run(*args, **kwargs):
        message = f"{name} ran outside a recorded tick (a replay has nothing to run)"
        if session is not None:
            raise session.fail(message)  # latched: the replayed code catching it does not hide it
        raise ReplayMismatch(message)
    return run


# Features whose on-records are not replayable yet: the replay refuses them by name rather than replaying a run
# whose new ports it does not supply (plan 2026-10-19 task 4, Codex design round 1 P2 -- the far recognition the
# side evidence runs with is not replayable either).
UNREPLAYABLE_ON = {"bottom_deck_check": "off",
                   # gyro-heading plan G2 (A): the simulated gyro (a setup service holding its RNG) is not replayable yet
                   "odometry_heading": "steering"}


def refuse_unreplayable(values: dict) -> None:
    args_ = values.get("args")
    for flag, off in UNREPLAYABLE_ON.items():
        recorded = getattr(args_, flag, off)  # a record from before the flag ran without it
        if recorded != off:
            raise ReplayMismatch(f"a {flag}={recorded!r} record is not replayable (only {off!r} records are)")


def build_inputs(run_dir: Path, gate: ReplayBoundary, output: Path | None = None):
    snap = json.loads((run_dir / "boundary/setup_snapshot.json").read_text())
    if snap.get("opaque"):  # a value the snapshot could not keep: the replay would hand back a marker
        raise ReplayMismatch(f"the setup snapshot holds opaque values: {snap['opaque']}")
    arrays = {}
    extra = run_dir / "boundary/setup_arrays.npz"
    if extra.exists():
        from forklift_core.mission.replay_codec import Encoder

        with np.load(extra, allow_pickle=False) as z:
            arrays = {k: z[k] for k in z.files}
        bad = [k for k, a in arrays.items() if Encoder.digest(a) != k]
        if bad:
            raise ReplayMismatch(f"setup arrays do not match their hashes: {bad[:3]}")
    # The files the setup read must still be what it read (the replay runs where they are, by the same paths).
    import hashlib

    changed = []
    for item in snap.get("files", []):
        path_ = Path(item["path"])
        try:
            digest = hashlib.sha256(path_.read_bytes()).hexdigest()
        except OSError:
            digest = None
        if digest != item["sha256"]:
            changed.append(item["path"])
    if changed:
        raise ReplayMismatch(f"setup input files differ from the recorded ones: {changed[:5]}")
    specs, services = {}, {}
    for name, item in snap["names"].items():
        if item["kind"] == "service":
            specs[item["id"]] = item.get("spec") or {}
            services[item["id"]] = ReplayProxy(item["id"], gate, spec=specs[item["id"]])
    # Services that sat inside containers only (the obstacle layer), with their recorded specs.
    for logical, spec_ in (snap.get("service_specs") or {}).items():
        specs.setdefault(logical, spec_)
    for logical, spec_ in (snap.get("service_specs") or {}).items():
        if logical not in services and logical not in ("Permission", "Memory"):
            services[logical] = ReplayProxy(
                logical, gate, spec=spec_,
                child_ports={"permission": "Permission", "memory": "Memory"} if logical == "Obstacles" else None,
                child_specs={"permission": specs.get("Permission", {}), "memory": specs.get("Memory", {})})
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
            values[name] = BR.recorded_function(_unrunnable(name, gate.session), item["port"], item["method"], gate)
        elif kind == "value":
            values[name] = decode(item["node"], arrays, services=services, by_id=shared_ids)
        elif kind in ("adapters", "excluded"):
            continue
        else:
            raise ValueError(f"unknown snapshot kind {kind} for {name}")
    specs_by_port = dict(specs)

    def proxy_factory(target, port, gate_, **kw):
        if gate_ is not gate:
            raise gate.session.fail(f"a {port} proxy built on another gate")
        if isinstance(target, ReplayProxy):
            if object.__getattribute__(target, "_port") != port:  # a service wired to another port
                raise gate.session.fail(f"the {object.__getattribute__(target, '_port')} service wrapped as {port}")
            spec = object.__getattribute__(target, "_spec")
        elif isinstance(target, types.ModuleType):  # the runner's `time` module as the Clock port
            spec = {"methods": sorted(n for n in dir(target) if callable(getattr(target, n, None)))}
        else:
            spec = specs_by_port.get(port, type(target))
        child_specs = {name: specs_by_port.get(child, {}) for name, child in (kw.get("child_ports") or {}).items()}
        return ReplayProxy(port, gate, spec=spec, child_ports=kw.get("child_ports"), child_specs=child_specs,
                           hash_only_requests=kw.get("hash_only_requests", ()), watches=kw.get("watches"),
                           returns=kw.get("returns"))

    if output is not None and "args" in values:  # the replay writes its own outputs, never the evidence's
        values["args"].output = Path(output)
        Path(output).mkdir(parents=True, exist_ok=True)
    refuse_unreplayable(values)
    bind_added_names(values, snap["names"])
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
        cast_reference_scan=_unrunnable("cast_reference_scan", gate.session),
        place_chase_camera=lambda *a, **k: None,
    )
    return types.SimpleNamespace(**values), adapters, snap


def final_state_differences(boundary: Path, session: ReplaySession, final_state) -> list[str]:
    """Where the replayed end-state projection differs from the recorded one (map keys, three levels down)."""
    from forklift_core.mission.replay_codec import Encoder

    recorded = json.loads((boundary / "final_state.json").read_text())
    replayed = json.loads(json.dumps(Encoder(opaque_ok=True, value_types=session.value_types, hash_arrays=True,
                                             identity=False).encode(final_state), sort_keys=True, allow_nan=True))
    out: list[str] = []

    def walk(a, b, path, depth):
        if a == b:
            return
        if depth and isinstance(a, dict) and isinstance(b, dict):
            for key in sorted(set(a) | set(b)):
                walk(a.get(key), b.get(key), f"{path}.{key}", depth - 1)
            return
        out.append(path or ".")

    walk(recorded, replayed, "", 6)
    return out[:50]


def _BOUNDARY_OPEN(module) -> bool:  # noqa: N802
    return module._BOUNDARY_RECORD.get("gate") is not None


def check_ending(session: ReplaySession, raised: BaseException | None) -> None:
    """The replayed code ends as the recorded one did -- returned, or the same exception (type and message) --
    and closed its record; otherwise the replay fails (latched)."""
    if session.failure is not None:
        return
    recorded = session.meta.get("ending")
    if recorded is None:
        session.fail("the record does not say how the recorded run ended (meta 'ending')")
    elif BR.boundary_ending(raised) != recorded:
        session.fail(f"the replay ended {BR.boundary_ending(raised)}, the recorded run {recorded}")
    elif not getattr(session, "closed", False):
        session.fail("the replayed run never closed its record")


def replay_run(run_dir: Path, output: Path | None = None) -> dict:
    run_dir = Path(run_dir)
    _install_isaac_value_types()
    # Callbacks by type in a Planner request only behind their Trace.callback_spec (ReplaySession._described).
    session = ReplaySession(run_dir / "boundary")
    gate = ReplayBoundary(session)
    # The runner module first: the sibling modules it loads by file (perception adapter, near tracking ...) are
    # what the snapshot's imports name.
    spec = importlib.util.spec_from_file_location("run_transport_replay", ROOT / "sim/isaac/run_transport.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    own_output = output is None
    if own_output:
        import tempfile

        output = Path(tempfile.mkdtemp(prefix="boundary_replay_"))
    S, adapters, snap = build_inputs(run_dir, gate, output)
    module._BOUNDARY_RECORD["gate"] = gate
    outcome = {"run_dir": str(run_dir), "ticks_done": 0}
    raised = None
    try:
        module.drive(S, adapters)
        outcome["drive"] = "returned"
    except ReplayMismatch as exc:
        raised = exc
        outcome["drive"] = f"mismatch: {exc}"
        context = exc.__context__
        if context is not None:  # what the replayed run raised first (a require, a crash) before this mismatch
            outcome["context"] = f"{type(context).__name__}: {context}"
            outcome["context_traceback"] = "".join(
                traceback.format_exception(type(context), context, context.__traceback__))[-4000:]
    except (Exception, SystemExit) as exc:  # noqa: BLE001 -- the replayed run's own end is part of the record
        raised = exc
        outcome["drive"] = f"{type(exc).__name__}: {exc}"
        outcome["traceback"] = traceback.format_exc()[-4000:]
    if not isinstance(raised, ReplayMismatch) and _BOUNDARY_OPEN(module):
        # What main() does when run() raised before drive's own finally closed the record (the initial plans, the
        # bootstrap): the failure reason in the state, then close with that exception as the ending.
        try:
            if raised is not None:
                S.state["success"] = False
                S.state["failure_reason"] = str(raised)
            module.close_boundary_record(S.state, raised)
        except ReplayMismatch:
            pass  # latched in the session
    check_ending(session, raised)
    recorded_final = run_dir / "boundary/final_state.json"
    if session.failure is None and recorded_final.exists() and '"projection_error"' in recorded_final.read_text():
        # The recorded run could not project its end state (a value without an encoding): nothing to compare.
        session.fail(f"the record's end-state projection failed: {recorded_final.read_text()[:300]}")
    if session.failure is None:  # a recorded failure replayed exactly ends the same way: finish it too
        final_state = getattr(gate.recorder, "final_state", None)
        try:
            session.finish(final_state)  # every recorded tick replayed, as many as the meta says; the end state
            outcome["finish"] = "ok"
        except ReplayMismatch as exc:
            outcome["finish"] = str(exc)
            if "final state" in str(exc):
                outcome["final_state_differences"] = final_state_differences(run_dir / "boundary", session,
                                                                             final_state)
    outcome["ticks_done"] = session.ticks_done
    outcome["failure"] = session.failure
    outcome["tick_open"] = session.tick["tick"] if session.tick else None
    if own_output:
        # The replay's own scratch output (about 38 MB a run): removed when the replay passed -- 925 of them filled
        # ws1's /tmp (2026-10-19) -- and kept, named in the outcome, when it failed (for the diagnosis).
        if session.failure is None and outcome.get("finish") == "ok":
            import shutil

            shutil.rmtree(output, ignore_errors=True)
        else:
            outcome["output_dir"] = str(output)
    return outcome


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--json", type=Path)
    parser.add_argument("--output", type=Path, help="where the replayed run writes (default: a new temporary directory)")
    args = parser.parse_args(argv)
    out = replay_run(args.run_dir, args.output)
    text = json.dumps(out, indent=1)
    if args.json:
        args.json.write_text(text + "\n")
    print(text)
    return 0 if out.get("failure") is None and out.get("finish") == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
