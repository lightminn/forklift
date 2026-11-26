"""The firmware identity's git description is ``git describe --long`` (operating plan 2026-11-26 U3d): a tagged
build names its commit too -- even on the tagged commit, and whatever the tag is named -- so the host's
``image_commit`` reads the real abbreviation (Codex U1 round 4). Needs cmake and git only (no toolchain)."""

import re
import shutil
import subprocess
from pathlib import Path

import pytest

from forklift_core.live.real_preconditions import image_commit

ROOT = Path(__file__).resolve().parents[2]
BOARD = ROOT / "firmware" / "boards" / "weact_h743"

pytestmark = pytest.mark.skipif(shutil.which("cmake") is None or shutil.which("git") is None,
                                reason="needs cmake and git")


def _git(repo, *args):
    return subprocess.run(["git", "-c", "user.email=t@example.invalid", "-c", "user.name=t", *args], cwd=repo,
                          capture_output=True, text=True, check=True).stdout.strip()


def _describe(repo: Path, out: Path) -> str:
    res = subprocess.run(["cmake", f"-DSRC_DIR={repo}", f"-DIN={BOARD / 'build_info.h.in'}", f"-DOUT={out}",
                          "-DFW_BUILD=normal", "-DFW_LINK=usb", "-DFW_OUTPUTS_WORD=OPEN", "-DFW_OUTPUTS_LOCKED=0",
                          "-DFW_ESTOP_WAIVED=0", "-DFW_SINGLE_AXIS=0", "-DFW_TOOLCHAIN_VERSION=x",
                          "-P", str(BOARD / "gen_build_info.cmake")], capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr
    return re.search(r'#define FW_GIT_DESCRIBE "([^"]*)"', out.read_text()).group(1)


@pytest.mark.parametrize("tag", ["v1.0", "release-1-gabcdef012345", "deadbeefdead"])
def test_a_tagged_build_names_its_commit_whatever_the_tag(tmp_path, tag):
    repo = tmp_path / "repo"
    repo.mkdir()
    (repo / "a").write_text("a\n")
    _git(repo, "init", "-q")
    _git(repo, "add", "a")
    _git(repo, "commit", "-q", "-m", "a")
    head = _git(repo, "rev-parse", "--short=12", "HEAD")
    untagged = _describe(repo, tmp_path / "u.h")
    assert untagged == head and image_commit(f"forklift-mcu normal OPEN {untagged}") == head
    _git(repo, "tag", "-a", "-m", "t", tag)  # on the commit itself (an annotated tag, as git describe reads)
    got = _describe(repo, tmp_path / "t.h")
    assert got == f"{tag}-0-g{head}"
    assert image_commit(f"forklift-mcu normal OPEN {got} rec=0123abcd waiver=0123abcd") == head
