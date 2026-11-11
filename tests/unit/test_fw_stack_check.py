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
       [("main", "f"), ("f", "/x/b.c:helper")])
    r = tool.analyse(tmp_path, [["main"], ["isr"]], [])
    assert r["levels"][0]["worst"] == 148  # main -> f -> the file-static helper (exact title)
    assert r["levels"][1]["worst"] == 30
    assert r["total"] == 148 + 30 + tool.EXCEPTION_FRAME


def test_an_external_call_never_resolves_to_another_files_static(tmp_path):
    """Codex B2b review P3: main calls an external helper; a static helper elsewhere is not it."""
    ci(tmp_path / "a.ci", {"main": (16, "static"), "/other.c:helper": (0, "static")},
       [("main", "helper")])
    with pytest.raises(SystemExit, match="helper"):
        tool.analyse(tmp_path, [["main"]], [])


def test_binary_frames_from_disassembly():
    frame, callees = tool.binary_frame("fmodf", ["push\t{r3, lr}", "vpush\t{d8}",
                                                 "bl\t80047b8 <__ieee754_fmodf>",
                                                 "beq.n\t8004720 <fmodf+0x18>", "bl\t8004be8 <__errno>"])
    assert frame == 8 + 8 and callees == {"__ieee754_fmodf", "__errno"}
    frame, _ = tool.binary_frame("u", ["sub.w\tip, sp, #8", "strd\tip, lr, [sp, #-16]!", "sub\tsp, #24"])
    assert frame == 16 + 24
    frame, _ = tool.binary_frame("m", ["push\t{r4-r7, lr}", "stmdb\tsp!, {r8, r9}"])
    assert frame == 20 + 8
    with pytest.raises(SystemExit, match="indirect"):
        tool.binary_frame("x", ["blx\tr3"])
    with pytest.raises(SystemExit, match="register-sized"):
        tool.binary_frame("y", ["sub\tsp, r2"])


def test_indirect_rules_pick_targets_per_caller(tmp_path):
    ci(tmp_path / "a.ci", {"tick_isr": (10, "static"), "on_tick": (20, "static"),
                           "app": (30, "static"), "hal_read": (5, "static"), "hal_big": (50, "static")},
       [("tick_isr", "__indirect_call"), ("on_tick", "app"), ("app", "__indirect_call")])
    r = tool.analyse(tmp_path, [["tick_isr"]], ["tick_isr=on_tick", "*=hal_*"])
    assert r["levels"][0]["worst"] == 10 + 20 + 30 + 50


def test_recursion_and_dynamic_frames_fail(tmp_path):
    ci(tmp_path / "a.ci", {"a": (8, "static"), "b": (8, "static")}, [("a", "b"), ("b", "a")])
    with pytest.raises(SystemExit, match="recursion"):
        tool.analyse(tmp_path, [["a"]], [])
    ci(tmp_path / "a.ci", {"a": (8, "dynamic")}, [])
    with pytest.raises(SystemExit, match="dynamic"):
        tool.analyse(tmp_path, [["a"]], [])


def test_indirect_call_without_a_rule_fails(tmp_path):
    ci(tmp_path / "a.ci", {"a": (8, "static")}, [("a", "__indirect_call")])
    with pytest.raises(SystemExit, match="indirect"):
        tool.analyse(tmp_path, [["a"]], [])


def test_recursion_in_a_function_without_a_call_graph_fails(tmp_path):
    """Codex B2b review 2: a library function calling itself is recursion, not a 0-byte call."""
    ci(tmp_path / "a.ci", {"main": (16, "static")}, [("main", "lib_recursive")])
    binary = {"lib_recursive": ["push\t{r4, lr}", "bne.n\t8000010 <lib_recursive+0x8>",
                                "bl\t8000000 <lib_recursive>"]}
    with pytest.raises(SystemExit, match="recursion"):
        tool.analyse(tmp_path, [["main"]], [], binary=binary)
    binary = {"lib_leaf": ["push\t{r4, lr}", "bne.n\t8000010 <lib_leaf+0x8>"]}
    ci(tmp_path / "a.ci", {"main": (16, "static")}, [("main", "lib_leaf")])
    assert tool.analyse(tmp_path, [["main"]], [], binary=binary)["total"] == 24
