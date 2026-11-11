"""Vendored MCU dependencies match firmware/third_party/MANIFEST.yaml (lead condition 2026-11-12)."""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[2]
MANIFEST = REPO / "firmware" / "third_party" / "MANIFEST.yaml"


def test_vendored_files_match_manifest_hashes():
    res = subprocess.run([sys.executable, str(REPO / "tools" / "fetch_firmware_deps.py"), "--check"],
                         capture_output=True, text=True)
    assert res.returncode == 0, res.stdout + res.stderr


def test_manifest_pins_commits_and_licenses():
    m = yaml.safe_load(MANIFEST.read_text())
    for pkg in m["packages"]:
        assert len(pkg["commit"]) == 40 and all(c in "0123456789abcdef" for c in pkg["commit"])
        assert pkg["license"] in ("Apache-2.0", "BSD-3-Clause")
        assert any(Path(f).name.startswith("LICENSE") for f in pkg["files"]), pkg["name"]
        for rel, digest in pkg["files"].items():
            assert isinstance(digest, str) and len(digest) == 64, rel


def test_only_listed_files_are_vendored():
    m = yaml.safe_load(MANIFEST.read_text())
    listed = {str(Path(p["dest"]) / rel) for p in m["packages"] for rel in p["files"]}
    root = REPO / "firmware" / "third_party"
    present = {str(p.relative_to(REPO)) for p in root.rglob("*") if p.is_file() and p != MANIFEST}
    assert present == listed
