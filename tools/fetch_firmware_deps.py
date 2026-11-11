#!/usr/bin/env python3
"""Vendored MCU dependencies: fetch the pinned tags and check every file's sha256.

The list, tags, commits and hashes live in firmware/third_party/MANIFEST.yaml (board design
2026-11-12 section 2). The build never downloads anything; this tool is how the vendored files
are (re)created and audited.

  python tools/fetch_firmware_deps.py --check          # hash the committed files
  python tools/fetch_firmware_deps.py --fetch          # download the tags, copy, then check
  python tools/fetch_firmware_deps.py --fetch --write-hashes   # first import: record hashes

Downloaded archives are untrusted data: they are only unpacked into a fresh temporary
directory and the listed files are copied byte for byte; nothing in them is executed.
"""
from __future__ import annotations

import argparse
import hashlib
import io
import sys
import tarfile
import tempfile
import urllib.request
from pathlib import Path

import yaml

REPO = Path(__file__).resolve().parents[1]
MANIFEST = REPO / "firmware" / "third_party" / "MANIFEST.yaml"


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load() -> dict:
    return yaml.safe_load(MANIFEST.read_text())


def archive_url(pkg: dict) -> str:
    return f"https://codeload.github.com/{pkg['repo']}/tar.gz/{pkg['commit']}"


def fetch(pkg: dict) -> None:
    with urllib.request.urlopen(archive_url(pkg), timeout=120) as resp:
        data = resp.read()
    dest = REPO / pkg["dest"]
    with tempfile.TemporaryDirectory() as tmp, tarfile.open(fileobj=io.BytesIO(data)) as tar:
        tar.extractall(tmp, filter="data")
        roots = [p for p in Path(tmp).iterdir() if p.is_dir()]
        if len(roots) != 1:
            raise SystemExit(f"{pkg['name']}: unexpected archive layout")
        for rel in pkg["files"]:
            src = roots[0] / rel
            out = dest / rel
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_bytes(src.read_bytes())


def check(manifest: dict, write: bool) -> int:
    bad = 0
    for pkg in manifest["packages"]:
        for rel, want in pkg["files"].items():
            path = REPO / pkg["dest"] / rel
            if not path.is_file():
                print(f"missing: {path.relative_to(REPO)}")
                bad += 1
                continue
            got = sha256(path)
            if write:
                pkg["files"][rel] = got
            elif got != want:
                print(f"hash mismatch: {path.relative_to(REPO)}")
                bad += 1
    return bad


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--fetch", action="store_true", help="download the pinned commits and copy")
    ap.add_argument("--check", action="store_true", help="check the committed files' hashes")
    ap.add_argument("--write-hashes", action="store_true", help="record hashes (first import)")
    args = ap.parse_args(argv)
    if not (args.fetch or args.check):
        ap.error("give --fetch and/or --check")
    manifest = load()
    if args.fetch:
        for pkg in manifest["packages"]:
            fetch(pkg)
    bad = check(manifest, args.write_hashes)
    if args.write_hashes:
        text = MANIFEST.read_text()
        head = text[: text.index("packages:")]
        MANIFEST.write_text(head + yaml.safe_dump({"packages": manifest["packages"]}, sort_keys=False))
    print("ok" if bad == 0 else f"{bad} problem(s)")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
