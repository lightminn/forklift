"""Tracked files must not use session-temporary paths as raw data (CONTRIBUTING.md section 7).

Session folders (a Claude job's tmp/, the session scratchpad) are cleaned without notice, so a validation record or
script that reads from one stops being reproducible. The files listed in ALLOWED predate the rule: freeze1.json is a
frozen record (its reference result now lives in docs/validation/assets/b-p2d/), and the two c0 records still name the
running job's dumps until that job ends and they are moved.
"""

import re
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SESSION_PATH = re.compile(
    "|".join(
        (
            r"\.claude/jobs/[0-9a-f]{8}/tmp",  # a Claude job's tmp folder
            r"\.cache/claude-tmp/claude-\d+",  # the session scratchpad root
        )
    )
)
ALLOWED = {
    "docs/validation/assets/b-p2d/freeze1.json",
    "docs/validation/2026-11-10-g3-1c0/c0_hall.json",
    "docs/validation/2026-11-10-g3-1c0/c0_line_m.json",
}
# the ws1 pytest copy has no .git and holds tracked files only; these trees never reach it but are skipped when walking
SKIP_DIRS = {
    ".git",
    "artifacts",
    "data",
    "test_runs",
    "ros2/build",
    "ros2/install",
    "ros2/log",
    "presentation",
}
MAX_BYTES = 20 * 2**20  # larger tracked files are data, not records or scripts


def tracked_files():
    """git ls-files when this is a checkout; otherwise the tree itself."""
    if (ROOT / ".git").exists():
        out = subprocess.run(
            ["git", "-C", str(ROOT), "ls-files", "-z"], capture_output=True, check=True
        ).stdout
        return [p for p in out.decode().split("\0") if p]
    files = []
    for path in ROOT.rglob("*"):
        rel = path.relative_to(ROOT).as_posix()
        if path.is_file() and not any(
            rel == d or rel.startswith(d + "/") for d in SKIP_DIRS
        ):
            files.append(rel)
    return files


def text_of(path):
    """The file's text, or None for symlinks, missing, oversized or binary (NUL byte) files."""
    if path.is_symlink() or not path.is_file() or path.stat().st_size > MAX_BYTES:
        return None
    data = path.read_bytes()
    return None if b"\0" in data else data.decode("utf-8", errors="ignore")


def offending_files():
    found = []
    for rel in tracked_files():
        if rel in ALLOWED:
            continue
        text = text_of(ROOT / rel)
        if text is not None and SESSION_PATH.search(text):
            found.append(rel)
    return found


def test_no_tracked_file_reads_raw_data_from_a_session_folder():
    assert offending_files() == []


def test_pattern_catches_the_forms_seen_so_far():
    # built from pieces so this file does not itself contain a forbidden path
    job = "/home/light/" + ".claude/jobs/" + "53111190" + "/tmp/g1raw/g1"
    scratch = (
        "/home/light/" + ".cache/claude-tmp/" + "claude-1000/-home-x/tasks/a.output"
    )
    assert SESSION_PATH.search(job)
    assert SESSION_PATH.search(scratch)
    assert not SESSION_PATH.search(
        "`~/.claude/jobs/*/tmp`, session scratchpads"
    )  # the rule text itself


def test_text_detection_covers_files_without_a_known_suffix(tmp_path):
    forbidden = "x = '/home/light/" + ".claude/jobs/" + "deadbeef" + "/tmp/a'\n"
    for name in ("run.args", "Dockerfile", "patch.diff"):
        (tmp_path / name).write_text(forbidden)
        assert SESSION_PATH.search(text_of(tmp_path / name))
    (tmp_path / "blob.bin").write_bytes(b"\0" + forbidden.encode())
    assert text_of(tmp_path / "blob.bin") is None


def test_allowed_files_exist():
    # an allowance for a file that is gone hides nothing and should be dropped
    assert all((ROOT / rel).is_file() for rel in ALLOWED)
