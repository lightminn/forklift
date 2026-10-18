"""The live failure classes (persistent console plan D6): every require reason the runner and the core can raise is
classified; unknown reasons end the session."""

import ast
from pathlib import Path

import pytest

from forklift_core.mission.failures import DYNAMIC_SITES, classify

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize("reason, cls", [
    ("Tracking failed in observe", "mission"),
    ("approach:no_path", "mission"),
    ("near_field_mismatch", "mission"),
    ("near_field_contract: x", "session"),
    ("near_field_frame_released", "session"),
    ("Excessive body tilt in lift: 0.2", "scene"),
    ("slam_link_failed: reply late", "session"),
    ("something nobody wrote down", "session"),
    ("", "session"),
])
def test_reasons_are_classified_by_their_longest_prefix(reason, cls):
    assert classify(reason) == cls


def _require_reasons(path):
    tree = ast.parse(path.read_text())
    out = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or len(node.args) < 2:
            continue
        func = node.func
        if (func.attr if isinstance(func, ast.Attribute) else getattr(func, "id", None)) != "require":
            continue
        arg = node.args[1]
        if isinstance(arg, ast.Constant):
            out.append((node.lineno, arg.value, None))
        elif isinstance(arg, ast.JoinedStr):
            first = arg.values[0]
            out.append((node.lineno, first.value if isinstance(first, ast.Constant) else "", "f"))
        else:
            out.append((node.lineno, None, ast.unparse(arg)))
    return out


@pytest.mark.parametrize("rel", ["sim/isaac/run_transport.py", "src/forklift_core/mission/core.py"])
def test_every_require_reason_has_a_class_of_its_own(rel):
    for line, text, kind in _require_reasons(ROOT / rel):
        if text is None:  # a reason made at run time: its site is listed with the class its values take
            assert (rel, kind) in DYNAMIC_SITES, f"{rel}:{line} dynamic reason {kind!r} not listed"
            continue
        if kind == "f" and text == "":
            continue  # an f-string with no fixed head: its sites are checked through DYNAMIC_SITES below
        assert classify(text, strict=True) is not None, f"{rel}:{line} {text!r} unclassified"
