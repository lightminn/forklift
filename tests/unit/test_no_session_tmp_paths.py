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
TEXT_SUFFIXES = {
    ".py",
    ".md",
    ".json",
    ".jsonl",
    ".yaml",
    ".yml",
    ".toml",
    ".txt",
    ".sh",
    ".sbatch",
    ".mjs",
    ".js",
    ".cfg",
    ".ini",
    ".csv",
    ".tsv",
}
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


def tracked_files():
    """git ls-files when this is a checkout; otherwise the tree itself (the ws1 pytest copy holds tracked files only)."""
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


def offending_files():
    found = []
    for rel in tracked_files():
        path = ROOT / rel
        if (
            path.suffix not in TEXT_SUFFIXES
            or path.is_symlink()
            or not path.is_file()
            or rel in ALLOWED
        ):
            continue
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        if SESSION_PATH.search(text):
            found.append(rel)
    return found


def test_no_tracked_file_reads_raw_data_from_a_session_folder():
    assert offending_files() == []


def test_pattern_catches_the_forms_seen_so_far():
    assert SESSION_PATH.search("/home/light/.claude/jobs/53111190/tmp/g1raw/g1")
    assert SESSION_PATH.search(
        "/home/light/.cache/claude-tmp/claude-1000/-home-x/tasks/a.output"
    )
    assert not SESSION_PATH.search(
        "`~/.claude/jobs/*/tmp`, session scratchpads"
    )  # the rule text itself


def test_allowed_files_exist():
    # an allowance for a file that is gone hides nothing and should be dropped
    assert all((ROOT / rel).is_file() for rel in ALLOWED)
