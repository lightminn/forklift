"""The replay container's check that the slam_toolbox it ran is the one asked for (docs/design/2026-11-13-slam-patch-build.md
2, Codex patch review P2-2/P2-3): the image kind against the build record, the libraries the node loaded, and the patch
identification line against the switches passed."""

from __future__ import annotations

import pytest

from tools import slam_patch_check as pc

APT = "/opt/ros/jazzy/lib/libkartoSlamToolbox.so"
OVL = "/opt/slam_patched/install/slam_toolbox/lib/libkartoSlamToolbox.so"
IDENT = ("[sync_slam_toolbox_node-1] forklift_slam_patch series=1a2b3c4d trace=1 cwm_s=600 delete_s=0 s1=0 "
         "s1_buffer=10")


def libs(*paths):
    return "".join(f"123 {p}\n" for p in paths)


def test_the_deployed_image_runs_apt_libraries_and_prints_no_identification():
    assert pc.check(kind="", series=None, libs=libs(APT, "/opt/ros/jazzy/lib/libtoolbox_common.so"), log="", env={}) == []


def test_a_source_image_must_load_its_overlay_and_match_its_kind():
    assert pc.check(kind="u0", series="none", libs=libs(OVL), log="", env={}) == []
    problems = pc.check(kind="u0", series="1a2b3c4d", libs=libs(OVL), log="", env={})
    assert any("series" in p for p in problems)  # a patched image run as U0
    assert any("apt" in p for p in pc.check(kind="patched", series="1a2b3c4d", libs=libs(APT), log=IDENT,
                                            env={"FORKLIFT_SLAM_TRACE": "/out/t", "FORKLIFT_SLAM_CWM_S": "600"}))
    assert any("no libkartoSlamToolbox" in p for p in pc.check(kind="u0", series="none", libs="", log="", env={}))
    mixed = libs(OVL, APT)  # two copies loaded
    assert any("apt" in p for p in pc.check(kind="u0", series="none", libs=mixed, log="", env={}))


def test_the_identification_line_must_say_what_was_asked():
    env = {"FORKLIFT_SLAM_TRACE": "/out/trace.jsonl", "FORKLIFT_SLAM_CWM_S": "600"}
    assert pc.check(kind="patched", series="1a2b3c4d", libs=libs(OVL), log=IDENT, env=env) == []
    # a misspelt switch: the node ran without C-WM
    wrong = {"FORKLIFT_SLAM_TRACE": "/out/trace.jsonl", "FORKLIFT_SLAM_CWM": "600"}
    assert any("cwm_s" in p for p in pc.check(kind="patched", series="1a2b3c4d", libs=libs(OVL), log=IDENT, env=wrong))
    # no line at all from a patched image, or a series other than the image's
    assert any("identification" in p for p in pc.check(kind="patched", series="1a2b3c4d", libs=libs(OVL), log="",
                                                       env=env))
    assert any("series" in p for p in pc.check(kind="patched", series="ffffffff", libs=libs(OVL), log=IDENT, env=env))
    # U0 and the deployed image have no such line
    assert any("identification" in p for p in pc.check(kind="u0", series="none", libs=libs(OVL), log=IDENT, env={}))


def test_the_cli_writes_the_verdict_and_exits_nonzero_on_a_problem(tmp_path):
    (tmp_path / "libs.txt").write_text(libs(APT))
    (tmp_path / "log.txt").write_text("")
    out = tmp_path / "patch_check.json"
    assert pc.main(["--kind", "", "--libs", str(tmp_path / "libs.txt"), "--log", str(tmp_path / "log.txt"),
                    "--out", str(out)]) == 0
    assert '"ok": true' in out.read_text()
    with pytest.raises(SystemExit):
        pc.main(["--kind", "u0", "--libs", str(tmp_path / "libs.txt"), "--log", str(tmp_path / "log.txt"),
                 "--out", str(out)])
