"""tools/ros2_dev.sh scan-a2m12 (real-scan design 1.5): the driver and the adapter in the ROS container with only
the LiDAR device and the scan socket's folder passed in; the image pins sllidar_ros2 by commit."""

import os
import re
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
PINNED = "34300099fadfc772965962dec837bf436706188f"  # Slamtec/sllidar_ros2 2024-06-17 (design 1.1)


def run_launcher(tmp_path, *args):
    fake = tmp_path / "bin"
    fake.mkdir()
    (fake / "docker").write_text('#!/bin/sh\nprintf "%s\\n" "$@"\n')
    (fake / "docker").chmod(0o755)
    env = {**os.environ, "PATH": f"{fake}:{os.environ['PATH']}"}
    return subprocess.run(["bash", str(REPO / "tools/ros2_dev.sh"), *args], env=env, capture_output=True, text=True)


def test_scan_a2m12_passes_the_device_and_the_socket_folder_only(tmp_path):
    dev = tmp_path / "ttyUSB9"
    dev.write_text("")
    sock = tmp_path / "sock"
    res = run_launcher(tmp_path, "scan-a2m12", str(dev), str(sock))
    assert res.returncode == 0, res.stderr
    argv = res.stdout.splitlines()
    assert f"{dev}:{dev}" in argv and "--device" in argv
    assert "--group-add" in argv and argv[argv.index("--group-add") + 1] == str(dev.stat().st_gid)
    assert f"type=bind,src={sock},dst=/run/forklift-scan" in argv and sock.is_dir()
    cmd = argv[-1]
    assert "sllidar_node" in cmd and "serial_baudrate:=256000" in cmd and f"serial_port:={dev}" in cmd
    assert "scan:=/sensors/lidar/scan_raw" in cmd
    assert "real_scan_adapter.py --socket /run/forklift-scan/scan.sock" in cmd
    assert "--privileged" not in argv and "--network" in argv and argv[argv.index("--network") + 1] == "bridge"


def test_scan_a2m12_refuses_a_missing_device(tmp_path):
    res = run_launcher(tmp_path, "scan-a2m12", str(tmp_path / "nope"))
    assert res.returncode != 0 and "device" in res.stderr


def test_scan_a2m12_runs_the_scan_image(tmp_path):
    dev = tmp_path / "ttyUSB9"
    dev.write_text("")
    argv = run_launcher(tmp_path, "scan-a2m12", str(dev), str(tmp_path / "s")).stdout.splitlines()
    assert argv[-4] == "forklift/ros2-scan:jazzy"  # image, then bash -c <command>


def test_the_image_pins_sllidar_ros2_by_commit():
    text = (REPO / "deploy/ros2/Dockerfile.scan").read_text(encoding="utf-8")
    assert "FROM ${BASE_IMAGE}" in text and "ARG BASE_IMAGE=forklift/ros2-dev:jazzy" in text
    assert f"SLLIDAR_ROS2_COMMIT={PINNED}" in text
    assert re.search(r"git -C \S+ checkout --detach \"?\$\{SLLIDAR_ROS2_COMMIT\}", text)


def test_a_device_given_by_link_passes_the_real_devices_group(tmp_path):
    """Codex 7a P2-6: /dev/serial/by-id/... is a link; its own group is not the port's."""
    dev = tmp_path / "ttyUSB9"
    dev.write_text("")
    link = tmp_path / "by-id-link"
    link.symlink_to(dev)
    argv = run_launcher(tmp_path, "scan-a2m12", str(link), str(tmp_path / "s")).stdout.splitlines()
    assert argv[argv.index("--group-add") + 1] == str(os.stat(link).st_gid)  # followed
    assert "stat -L" in (REPO / "tools/ros2_dev.sh").read_text()
