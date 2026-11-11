"""tools/fw_stack_check.py on synthetic GCC call-graph files."""
from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("fw_stack_check", REPO / "tools" / "fw_stack_check.py")
tool = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tool)


def ci(path: Path, nodes: dict[str, tuple[int, str]], edges: list[tuple[str, str]]) -> None:
    lines = ['graph: { title: "x"']
    for title, (size, kind) in nodes.items():
        lines.append(f'node: {{ title: "{title}" label: "{title.split(":")[-1]}\\nf.c:1:1\\n{size} bytes ({kind})" }}')
    for a, b in edges:
        lines.append(f'edge: {{ sourcename: "{a}" targetname: "{b}" label: "f.c:2:2" }}')
    lines.append("}")
    path.write_text("\n".join(lines))


def test_levels_add_up_with_frames(tmp_path):
    ci(tmp_path / "a.ci", {"main": (100, "static"), "f": (40, "static"), "isr": (30, "static"),
                           "/x/b.c:helper": (8, "static")},
       [("main", "f"), ("f", "helper"), ("isr", "memset")])
    r = tool.analyse(tmp_path, [["main"], ["isr"]], [], 64)
    assert r["levels"][0]["worst"] == 148  # main -> f -> helper (file-static, by short name)
    assert r["levels"][1]["worst"] == 30 + 64  # memset unknown
    assert r["total"] == 148 + 94 + tool.EXCEPTION_FRAME
    assert r["unknown_callees"] == ["memset"]


def test_indirect_rules_pick_targets_per_caller(tmp_path):
    ci(tmp_path / "a.ci", {"tick_isr": (10, "static"), "on_tick": (20, "static"),
                           "app": (30, "static"), "hal_read": (5, "static"), "hal_big": (50, "static")},
       [("tick_isr", "__indirect_call"), ("on_tick", "app"), ("app", "__indirect_call")])
    r = tool.analyse(tmp_path, [["tick_isr"]], ["tick_isr=on_tick", "*=hal_*"], 0)
    assert r["levels"][0]["worst"] == 10 + 20 + 30 + 50


def test_recursion_and_dynamic_frames_fail(tmp_path):
    ci(tmp_path / "a.ci", {"a": (8, "static"), "b": (8, "static")}, [("a", "b"), ("b", "a")])
    with pytest.raises(SystemExit, match="recursion"):
        tool.analyse(tmp_path, [["a"]], [], 0)
    ci(tmp_path / "a.ci", {"a": (8, "dynamic")}, [])
    with pytest.raises(SystemExit, match="dynamic"):
        tool.analyse(tmp_path, [["a"]], [], 0)


def test_indirect_call_without_a_rule_fails(tmp_path):
    ci(tmp_path / "a.ci", {"a": (8, "static")}, [("a", "__indirect_call")])
    with pytest.raises(SystemExit, match="indirect"):
        tool.analyse(tmp_path, [["a"]], [], 0)
