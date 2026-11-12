"""No undefined or used-before-assignment name in the code (ruff F821/F823). The runner's Isaac-only paths have no
unit test: a name used there without its import surfaces only in an Isaac run -- the layer latency budget's lidar
read (#442) ended every budgeted run of its registered check at start-up (2026-11-12, NameError: yaml)."""

import importlib.util
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.skipif(importlib.util.find_spec("ruff") is None, reason="ruff is not installed in this environment")
def test_no_undefined_names():
    p = subprocess.run([sys.executable, "-m", "ruff", "check", "--select", "F821,F823", "--no-cache", "--isolated",
                        "sim", "src", "tools", "deploy", "docs/validation"],
                       capture_output=True, text=True, cwd=ROOT, timeout=120)
    assert p.returncode == 0, p.stdout[-4000:] + p.stderr[-2000:]
