"""Real-time plan 5.23 G2-1a: the check's list -- 12 sessions, 3 of each scene and mode, alternating, the mode always
passed explicitly through the wrapper's own variable."""

import importlib.util
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "g2_1a_lists", ROOT / "docs" / "validation" / "2026-11-10-g2-1a-check" / "g2_1a_lists.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_twelve_alternating_sessions_with_the_mode_passed_explicitly():
    e = tool.entries("/snap", "/out")
    assert len(e) == 12 and len({x["name"] for x in e}) == 12
    arms = [(x["name"].split("_")[2], x["name"].split("_")[3]) for x in e]
    assert Counter(arms) == {("H", "sync"): 3, ("H", "async"): 3, ("F", "sync"): 3, ("F", "async"): 3}
    for (s1, m1), (s2, m2) in zip(arms[::2], arms[1::2]):
        assert s1 == s2 and {m1, m2} == {"sync", "async"}  # each scene's pair holds both modes
    assert [m for s, m in arms if s == "H"][:2] == ["sync", "async"] and \
        [m for s, m in arms if s == "H"][2:4] == ["async", "sync"]  # round 2 flips the first arm
    for x, (scene, mode) in zip(e, arms):
        env = x["env"]
        assert (env.get("MORE") == f"--live-capture-dump {mode}") if scene == "H" else (env.get("CAPTURE_DUMP") == mode)
        assert env["INSTR"] == "off" and env["SEED"] == "0"
        for script in (Path(ROOT, x["argv"][1].removeprefix("/snap/")),):
            assert script.exists()
