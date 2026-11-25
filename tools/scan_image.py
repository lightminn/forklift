#!/usr/bin/env python3
"""The A2M12 scan image's tag, from its inputs (7a-3 plan 5): the forklift patches to sllidar_ros2 and
deploy/ros2/Dockerfile.scan (the pinned SDK commit, the build and the SDK test). ``config/real/lidar.yaml``
``scan_image`` is the one place the tag is written; the build entry points compute it here and stop when the two
differ, and the launchers read it from there.

    python3 tools/scan_image.py tag --arch amd64|arm64     the tag the inputs give
    python3 tools/scan_image.py configured --arch ...      the tag lidar.yaml names
    python3 tools/scan_image.py check --arch ...           exit 1 (and say both) when they differ
    python3 tools/scan_image.py tarball --arch ...         the tarball name a built image is saved under

A timing bound is tied to the image ID that ran (lidar.yaml timing.applies_to.image_id), never to this tag: the tag
names the inputs, not the base image's bytes. Standard library only (ws1 compute nodes, the Jetson's system python).
"""

from __future__ import annotations

import argparse
import hashlib
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
PATCHES = REPO / "deploy" / "ros2" / "sllidar_patches"
DOCKERFILE = REPO / "deploy" / "ros2" / "Dockerfile.scan"
LIDAR_YAML = REPO / "config" / "real" / "lidar.yaml"
REPOSITORY = "forklift/ros2-scan"
ARCHES = ("amd64", "arm64")
_TAG = re.compile(r"^forklift/ros2-scan:jazzy-s[0-9a-f]{8}(-arm64)?$")


def inputs(root: Path = REPO) -> list[Path]:
    """The patch files in their application order (by name), then the Dockerfile."""
    patches = sorted((root / "deploy" / "ros2" / "sllidar_patches").glob("*.patch"))
    return [*patches, root / "deploy" / "ros2" / "Dockerfile.scan"]


def digest(root: Path = REPO) -> str:
    h = hashlib.sha256()
    for p in inputs(root):
        h.update(p.name.encode("utf-8") + b"\0" + p.read_bytes() + b"\0")
    return h.hexdigest()


def tag(arch: str, root: Path = REPO) -> str:
    if arch not in ARCHES:
        raise ValueError(f"arch must be one of {ARCHES}")
    return f"{REPOSITORY}:jazzy-s{digest(root)[:8]}" + ("-arm64" if arch == "arm64" else "")


def configured(arch: str, path: Path = LIDAR_YAML) -> str:
    """lidar.yaml's ``scan_image.<arch>`` -- read without a YAML library: a top-level ``scan_image:`` block of
    ``  <arch>: <tag>`` lines (tests check it reads what yaml.safe_load reads)."""
    if arch not in ARCHES:
        raise ValueError(f"arch must be one of {ARCHES}")
    found, inside = {}, False
    for line in Path(path).read_text(encoding="utf-8").splitlines():
        if re.match(r"^scan_image:\s*(#.*)?$", line):
            inside = True
            continue
        if inside:
            if line.strip() == "" or line.lstrip().startswith("#"):
                continue
            m = re.match(r"^  ([a-z0-9]+):\s*(\S+)\s*(#.*)?$", line)
            if not m:
                break
            found[m.group(1)] = m.group(2)
    value = found.get(arch)
    if value is None or not _TAG.match(value):
        raise ValueError(f"{path}: scan_image.{arch} missing or not a scan image tag")
    return value


def tarball(arch: str, path: Path = LIDAR_YAML) -> str:
    """The configured tag as a file name: forklift/ros2-scan:jazzy-sXXXXXXXX-arm64 -> ros2-scan-jazzy-sXXXXXXXX-arm64."""
    return configured(arch, path).split("/", 1)[1].replace(":", "-") + ".tar.zst"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("what", choices=("tag", "configured", "check", "tarball"))
    ap.add_argument("--arch", required=True, choices=ARCHES)
    a = ap.parse_args(argv)
    try:
        if a.what == "tag":
            print(tag(a.arch))
        elif a.what == "configured":
            print(configured(a.arch))
        elif a.what == "tarball":
            print(tarball(a.arch))
        else:
            want, have = tag(a.arch), configured(a.arch)
            if want != have:
                print(f"lidar.yaml scan_image.{a.arch} is {have}, the inputs give {want}: update lidar.yaml",
                      file=sys.stderr)
                return 1
            print(want)
    except (OSError, ValueError) as exc:
        print(f"scan_image: {exc}", file=sys.stderr)
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
