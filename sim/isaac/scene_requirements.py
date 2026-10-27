"""Scene requirements (docs/plans/2026-10-27-scene-requirements.md): the runner arguments a scene needs, and the
argument sets a purpose ("profile") compares against, declared in config/scene_requirements.yaml apart from the scene
files (the SLAM map registry matches those by their whole content) and checked by the runner before Isaac starts.

A scene entry is found by the scene file's "name"; its ``match`` (dotted paths into the scene record) decides whether
it applies to this copy -- two generations of aisle_rack share the name with posts of another size -- and ``any_of``
lists argument sets of which one must hold. A profile applies only when the run names it (--requirements-profile);
its ``args`` must hold, its ``recommends`` only warn. Keys are the runner's argparse dests; a value is a number, a
string, null (the argument not given) or {gt: number}."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import yaml

FORMAT = "scene_requirements_v1"
DEFAULT_PATH = Path(__file__).resolve().parents[2] / "config" / "scene_requirements.yaml"
TOLERANCE = 1e-9
# recommends compares the runner's final values, not the launcher's environment (a wrapper's default never shows there)
RECOMMEND_KEYS = ("live_factor", "encoder_model", "slam")


def _check_value(where: str, value) -> None:
    if isinstance(value, bool):
        # YAML reads on/off/yes/no as booleans: quote them (the runner's choices are strings)
        raise ValueError(f"{where}: {value!r} is a boolean -- quote choice values such as 'on'")
    if isinstance(value, dict):
        if set(value) != {"gt"} or isinstance(value["gt"], bool) or not isinstance(value["gt"], (int, float)):
            raise ValueError(f"{where}: a condition is {{gt: <number>}}")
        value = value["gt"]
    if value is not None and not isinstance(value, (int, float, str)):
        raise ValueError(f"{where}: {value!r} is not a number, a string, null or {{gt: x}}")
    if isinstance(value, float) and not math.isfinite(value):
        # .nan / .inf would compare oddly and break the result's strict JSON (Codex)
        raise ValueError(f"{where}: {value!r} is not a finite number")


def _check_args(where: str, args) -> None:
    if not isinstance(args, dict) or not args:
        raise ValueError(f"{where}: an argument set is a non-empty mapping of runner dests")
    for k, v in args.items():
        _check_value(f"{where}.{k}", v)


def load(path) -> dict:
    """The requirements file, its shape checked (the dests are checked against the runner's arguments in check())."""
    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or data.get("format") != FORMAT:
        raise ValueError(f"{path}: not a {FORMAT} file")
    unknown = set(data) - {"format", "scenes", "profiles"}
    if unknown:
        raise ValueError(f"{path}: unknown keys {sorted(unknown)}")
    for name, entry in (data.get("scenes") or {}).items():
        if not isinstance(entry, dict) or set(entry) - {"match", "any_of", "why"} or not entry.get("why"):
            raise ValueError(f"{path}: scene {name}: keys are match, any_of and why (why required)")
        for k, v in (entry.get("match") or {}).items():
            _check_value(f"scene {name} match {k}", v)
        if not isinstance(entry.get("any_of"), list) or not entry["any_of"]:
            raise ValueError(f"{path}: scene {name}: any_of is a non-empty list")
        for i, option in enumerate(entry["any_of"]):
            _check_args(f"scene {name} any_of[{i}]", option)
    for name, entry in (data.get("profiles") or {}).items():
        if not isinstance(entry, dict) or set(entry) - {"args", "recommends", "why"} or not entry.get("why"):
            raise ValueError(f"{path}: profile {name}: keys are args, recommends and why (why required)")
        _check_args(f"profile {name} args", entry.get("args"))
        for k, v in (entry.get("recommends") or {}).items():
            if k not in RECOMMEND_KEYS:
                raise ValueError(f"{path}: profile {name} recommends {k}: one of {RECOMMEND_KEYS}")
            _check_value(f"profile {name} recommends {k}", v)
    return data


def holds(actual, want) -> bool:
    """Whether a runner value meets a declared one: null is "not given", numbers within TOLERANCE (never against
    None), {gt: x} strictly above x, anything else as a string."""
    if isinstance(want, dict):
        return isinstance(actual, (int, float)) and not isinstance(actual, bool) and actual > want["gt"]
    if want is None:
        return actual is None
    if isinstance(want, (int, float)):
        return (isinstance(actual, (int, float)) and not isinstance(actual, bool)
                and abs(float(actual) - float(want)) <= TOLERANCE)
    return actual is not None and str(actual) == str(want)


def _dig(record, dotted: str):
    node = record
    for part in dotted.split("."):
        if not isinstance(node, dict) or part not in node:
            return _MISSING
        node = node[part]
    return node


_MISSING = object()


def _plain(v):
    return str(v) if isinstance(v, Path) else v


def check(values: dict, data: dict, scene_record: dict | None = None, profile: str | None = None,
          final: dict | None = None) -> dict:
    """The verdict for a run whose arguments are ``values`` (vars(args)): which scene entry applied, the mismatches
    (each a dict; any one refuses the run unless it is overridden) and the warnings (recommends). ``final`` holds the
    values recommends compares (RECOMMEND_KEYS). Raises ValueError for a dest the runner does not have or a profile
    the file does not declare -- a mistake in the file or the call, not in the run."""
    def dest(where, k):
        if k not in values:
            raise ValueError(f"scene requirements: {where}: {k} is not an argument of this runner")
        return _plain(values[k])

    report = {"scene": None, "matched": None, "profile": profile, "mismatches": [], "warnings": []}
    scenes = data.get("scenes") or {}
    if scene_record is not None:
        name = scene_record.get("name")
        report["scene"] = name
        entry = scenes.get(name)
        if entry is not None:
            for k, want in (entry.get("match") or {}).items():
                got = _dig(scene_record, k)
                if got is _MISSING or not holds(got, want):
                    report["matched"] = False
                    report["not_matched"] = f"{k} = {'(absent)' if got is _MISSING else got!r}, the entry needs {want!r}"
                    break
            else:
                report["matched"] = True
                options = entry["any_of"]
                have = {k: dest(f"scene {name}", k) for option in options for k in option}
                if not any(all(holds(have[k], w) for k, w in option.items()) for option in options):
                    report["mismatches"].append({"scene": name, "any_of": options, "have": have, "why": entry["why"]})
    if profile is not None:
        entry = (data.get("profiles") or {}).get(profile)
        if entry is None:
            raise ValueError(f"scene requirements: no profile {profile!r} (declared: {sorted(data.get('profiles') or {})})")
        for k, want in entry["args"].items():
            got = dest(f"profile {profile}", k)
            if not holds(got, want):
                report["mismatches"].append({"profile": profile, "arg": k, "want": want, "have": got})
        for k, want in (entry.get("recommends") or {}).items():
            got = (final or {}).get(k)
            if not holds(got, want):
                report["warnings"].append({"profile": profile, "recommends": k, "want": want, "have": got})
    return report


def describe(m: dict) -> str:
    """One line for a mismatch, as the runner's refusal and the preflight print it."""
    if "scene" in m:
        return (f"scene {m['scene']} needs one of {json.dumps(m['any_of'])}; this run has {json.dumps(m['have'])} "
                f"({m['why']})")
    return f"profile {m['profile']} needs {m['arg']} = {m['want']!r}; this run has {m['have']!r}"


def file_sha256(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()
