"""The setup snapshot of a format-2 boundary record (docs/plans/2026-10-16-m-a1-replay.md, step R3).

At the hand-over from run()'s setup to drive(), every name drive reads is written by kind, so a replay can
build the same ``S`` on CPU:

- ``service``: a live object a replay supplies through its ports (by logical id, also where it sits inside a
  container);
- ``import``: a module, class or function, by its import path (re-imported);
- ``value``: everything else, through the replay codec (value types registered; an object without a value
  encoding is listed under ``opaque`` -- the snapshot's own audit, refused by the replay unless allowed).

The files the setup opened for reading (an audit hook) are copied beside it with their content hash: the
inputs the settings named indirectly as well as those the arguments named.
"""

from __future__ import annotations

import hashlib
import json
import sys
import types
from pathlib import Path

import numpy as np

FORMAT = "forklift_setup_snapshot/1"

# Logical ids of the live objects the setup hands over (the ports a replay supplies).
SERVICES = {
    "robot": "Vehicle", "world": "Clock", "pallet": "SimTruth", "stage": "Stage", "camera": "OverviewCamera",
    "pocket_camera": "PocketCamera", "perception_camera": "PerceptionCamera", "perception_display": "DisplayCamera",
    "perception_capture": "Capture", "boundary": "Boundary", "recorder": "PocketRecorder",
}
# Value types kept by their attributes (plain objects without a dataclass encoding).
VALUE_TYPES = {"Namespace", "SimpleNamespace"}
# Helper objects the setup built, kept whole (private attributes included: their methods read them).
FULL_TYPES = {"InsertionGeometry", "CommandWindow", "RearHistory", "Schedule"}
# The recorded planner functions (recorded_function wrappers when recording): a replay supplies its own wrappers.
PORT_FUNCTIONS = {name: "Planner" for name in (
    "bicycle_rollout", "final_straight_prefix", "plan_docking_reapproach", "plan_observation_leg", "plan_return_leg",
    "plan_transport", "plan_transport_leg", "redraw_docking", "runtime_viewpoints", "straight_from_pose")}
PORT_FUNCTIONS["detect_pockets"] = "Perception"
PORT_FUNCTIONS["detect_pockets_expected"] = "Perception"  # plan 2026-10-22 task A
# Setup names drive's prologue lists but never reads before rebinding them (checked statically, plan R3 notes).
EXCLUDED = {"q": "a setup Gf.Quatd; drive reads q only in hook lambdas called after the loop has assigned it"}


def drive_inputs(drive) -> list[str]:
    """The setup names drive's prologue binds (``if "name" in _bound``)."""
    import ast
    import inspect
    import textwrap

    tree = ast.parse(textwrap.dedent(inspect.getsource(drive)))
    fn = tree.body[0]
    return [ast.literal_eval(st.test.left) for st in fn.body
            if isinstance(st, ast.If) and ast.unparse(st.test).endswith("in _bound")]


class FileAudit:
    """The files opened for reading from now until :meth:`stop` (an audit hook cannot be removed: it goes quiet)."""

    def __init__(self) -> None:
        self.paths: list[str] = []
        self.active = True
        sys.addaudithook(self._hook)

    def _hook(self, event: str, args) -> None:
        if not self.active or event != "open" or not args:
            return
        path, mode = args[0], (args[1] if len(args) > 1 else "r")
        if isinstance(path, (str, Path)) and (mode is None or "r" in str(mode)) and not any(
                c in str(mode or "") for c in "wax+"):
            text = str(path)
            if text not in self.paths:
                self.paths.append(text)

    def stop(self) -> list[str]:
        self.active = False
        return list(self.paths)


def service_spec(obj) -> dict:
    """What a replay proxy needs to tell a method call from an attribute read on a service whose class it cannot
    import (Isaac objects on CPU): the class's methods and properties."""
    methods, properties, seen = set(), set(), set()
    for klass in type(obj).__mro__:  # the first definition of a name in the MRO is the one that counts
        for key, value in vars(klass).items():
            if key in seen:
                continue
            seen.add(key)
            if isinstance(value, property):
                properties.add(key)
            elif callable(value) or isinstance(value, (staticmethod, classmethod)):
                methods.add(key)
    return {"type": f"{type(obj).__module__}:{type(obj).__qualname__}", "methods": sorted(methods - properties),
            "properties": sorted(properties)}


def _import_path(obj) -> str | None:
    if isinstance(obj, types.ModuleType):
        return obj.__name__
    module, qualname = getattr(obj, "__module__", None), getattr(obj, "__qualname__", None)
    if module and qualname and "<locals>" not in qualname:
        try:
            target = sys.modules.get(module)
            for part in qualname.split("."):
                target = getattr(target, part)
            if target is obj:
                return f"{module}:{qualname}"
        except AttributeError:
            return None
    return None


def write_setup_snapshot(names: dict, directory: Path, files: list[str], *, services: dict | None = None,
                         repo_root: Path | None = None, wanted=None) -> dict:
    """``names``: run()'s locals at the hand-over. ``services``: extra live objects by logical id (objects inside
    containers, e.g. the obstacle layer). Returns the snapshot's audit (opaque paths, files)."""
    from forklift_core.mission.replay_codec import Encoder

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    service_ids = {id(names[k]): v for k, v in SERVICES.items() if k in names and names[k] is not None}
    for logical, obj in (services or {}).items():
        if obj is not None:  # an absent service (no planning memory) stays None, never a service reference
            service_ids[id(obj)] = logical
    encoder = Encoder(opaque_ok=True, value_types=VALUE_TYPES, services=service_ids, full_types=FULL_TYPES)
    out, opaque = {}, []
    for name, value in sorted(names.items()):
        if name.startswith("__") or name in ("S",) or (wanted is not None and name not in wanted):
            continue
        if name in EXCLUDED:
            out[name] = {"kind": "excluded", "why": EXCLUDED[name]}
            continue
        if name in PORT_FUNCTIONS:
            out[name] = {"kind": "port", "port": PORT_FUNCTIONS[name], "method": name}
            continue
        if name == "isaac":
            out[name] = {"kind": "adapters", "names": sorted(vars(value))}
            continue
        if id(value) in service_ids:
            out[name] = {"kind": "service", "id": service_ids[id(value)], "spec": service_spec(value)}
            continue
        path = _import_path(value) if callable(value) or isinstance(value, types.ModuleType) else None
        if path is not None:
            module_ = sys.modules.get(path.partition(":")[0])
            file_ = getattr(module_, "__file__", None)
            if file_ and repo_root is not None:  # a module loaded by file under its own name (run_transport_*)
                try:
                    file_ = str(Path(file_).resolve().relative_to(Path(repo_root).resolve()))
                except ValueError:
                    pass
            out[name] = {"kind": "import", "path": path, "file": file_}
            continue
        node = encoder.encode(value)
        text = json.dumps(node)
        if '"$opaque"' in text:
            opaque.append(f"{name}: {sorted(set(_opaque_types(node)))}")
        out[name] = {"kind": "value", "node": node}
    copied = []
    root = Path(repo_root) if repo_root is not None else None
    file_dir = directory / "setup_files"
    for raw in files:
        p = Path(raw)
        try:
            if not p.is_file():
                continue
            data = p.read_bytes()
        except OSError:
            continue
        digest = hashlib.sha256(data).hexdigest()
        file_dir.mkdir(exist_ok=True)
        (file_dir / digest).write_bytes(data)
        rel = None
        if root is not None:
            try:
                rel = str(p.resolve().relative_to(root.resolve()))
            except ValueError:
                rel = None
        copied.append({"path": str(p), "repo_relative": rel, "sha256": digest, "bytes": len(data)})
    # Every service's method/property spec by logical id (those inside containers included).
    service_specs = {logical: service_spec(obj) for logical, obj in (services or {}).items() if obj is not None}
    (directory / "setup_snapshot.json").write_text(json.dumps(
        {"format": FORMAT, "names": out, "opaque": opaque, "files": copied, "service_specs": service_specs},
        allow_nan=True) + "\n")
    if encoder.arrays:
        np.savez_compressed(directory / "setup_arrays.npz", **encoder.arrays)
    return {"opaque": opaque, "files": len(copied)}


def _opaque_types(node):
    if isinstance(node, dict):
        if "$opaque" in node:
            yield node["$opaque"]
        for v in node.values():
            yield from _opaque_types(v)
    elif isinstance(node, list):
        for v in node:
            yield from _opaque_types(v)
