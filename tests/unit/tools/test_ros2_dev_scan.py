"""tools/ros2_dev.sh scan-a2m12 (real-scan design 1.5): the driver and the adapter in the ROS container with only
the LiDAR device and the scan socket's folder passed in; the image pins sllidar_ros2 by commit."""

import os
import re
import subprocess
from pathlib import Path

REPO = Path(__file__).resolve().parents[3]
PINNED = "34300099fadfc772965962dec837bf436706188f"  # Slamtec/sllidar_ros2 2024-06-17 (design 1.1)


IMAGE_ID = "sha256:" + "c" * 64


def run_launcher(tmp_path, *args, env=None, image=True):
    """The launcher with a fake docker that prints its arguments (and answers ``image inspect`` with an ID when the
    image is there) and a fake udevadm."""
    fake = tmp_path / "bin"
    fake.mkdir(exist_ok=True)
    inspect = f'echo {IMAGE_ID}; exit 0' if image else 'exit 1'
    (fake / "docker").write_text('#!/bin/sh\nif [ "$1" = image ] && [ "$2" = inspect ]; then ' + inspect
                                 + '; fi\nprintf "%s\\n" "$@"\n')
    (fake / "docker").chmod(0o755)
    (fake / "udevadm").write_text('#!/bin/sh\nprintf "ID_VENDOR_ID=10c4\\nID_MODEL_ID=ea60\\nID_SERIAL_SHORT=0001\\n"\n')
    (fake / "udevadm").chmod(0o755)
    full = {**os.environ, "PATH": f"{fake}:{os.environ['PATH']}", **(env or {})}
    return subprocess.run(["bash", str(REPO / "tools/ros2_dev.sh"), *args], env=full, capture_output=True, text=True)


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


# --- 7a-3a: the run id, the source facts and the bench arguments ---

def _dev(tmp_path):
    dev = tmp_path / "ttyUSB9"
    dev.write_text("")
    return dev


def test_the_sdk_source_names_its_run_and_its_source_facts(tmp_path):
    res = run_launcher(tmp_path, "scan-a2m12", str(_dev(tmp_path)), str(tmp_path / "s"),
                       env={"FORKLIFT_SCAN_SOURCE": "sdk"})
    assert res.returncode == 0, res.stderr
    cmd = res.stdout.splitlines()[-1]
    run = re.search(r"forklift_run_id:=([0-9a-f]{16})\b", cmd).group(1)
    assert f"--run-id {run}" in cmd and f"--image-id {IMAGE_ID}" in cmd and "--usb-id 10c4:ea60:0001" in cmd
    assert "--host " in cmd and "scan_sdk_info:=/sensors/lidar/scan_sdk_info" in cmd
    assert f"id {IMAGE_ID}" in res.stderr and run in res.stderr  # the run and the image ID that ran, on record


def test_each_run_gets_its_own_id(tmp_path):
    runs = set()
    for k in range(2):
        d = tmp_path / str(k)
        d.mkdir()
        cmd = run_launcher(d, "scan-a2m12", str(_dev(d)), str(d / "s")).stdout.splitlines()[-1]
        runs.add(re.search(r"forklift_run_id:=([0-9a-f]{16})", cmd).group(1))
    assert len(runs) == 2


def test_the_raw_source_passes_no_source_facts(tmp_path):
    cmd = run_launcher(tmp_path, "scan-a2m12", str(_dev(tmp_path)), str(tmp_path / "s")).stdout.splitlines()[-1]
    assert "--source raw" in cmd and "--run-id" not in cmd and "--image-id" not in cmd


def test_no_image_refuses_the_start(tmp_path):
    res = run_launcher(tmp_path, "scan-a2m12", str(_dev(tmp_path)), str(tmp_path / "s"), image=False)
    assert res.returncode == 2 and "no scan image" in res.stderr


def test_an_image_override_is_said(tmp_path):
    res = run_launcher(tmp_path, "scan-a2m12", str(_dev(tmp_path)), str(tmp_path / "s"),
                       env={"FORKLIFT_ROS2_SCAN_IMAGE": "forklift/ros2-scan:other"})
    assert "overridden by FORKLIFT_ROS2_SCAN_IMAGE" in res.stderr and IMAGE_ID in res.stderr


def test_the_bench_delay_and_receive_record(tmp_path):
    log = tmp_path / "rx"
    res = run_launcher(tmp_path, "scan-a2m12", str(_dev(tmp_path)), str(tmp_path / "s"),
                       env={"FORKLIFT_SDK_PUBLISH_DELAY_MS": "300", "FORKLIFT_SDK_RX_LOG_DIR": str(log)})
    assert res.returncode == 0, res.stderr
    argv = res.stdout.splitlines()
    assert "publish_delay_ms:=300" in argv[-1] and log.is_dir()
    k = argv.index(f"type=bind,src={log},dst=/run/forklift-sdk-log")
    assert argv[k + 1] == "--env"
    assert re.fullmatch(r"FORKLIFT_SDK_RX_LOG=/run/forklift-sdk-log/sdk_rx_[0-9a-f]{16}\.log", argv[k + 2])
    d = tmp_path / "plain"
    d.mkdir()
    plain = run_launcher(d, "scan-a2m12", str(_dev(d)), str(d / "s"))
    assert "publish_delay_ms" not in plain.stdout and "forklift-sdk-log" not in plain.stdout


def test_a_bad_bench_delay_is_refused(tmp_path):
    for bad in ("-1", "abc", "5000"):
        res = run_launcher(tmp_path, "scan-a2m12", str(_dev(tmp_path)), str(tmp_path / "s"),
                           env={"FORKLIFT_SDK_PUBLISH_DELAY_MS": bad})
        assert res.returncode == 2 and "FORKLIFT_SDK_PUBLISH_DELAY_MS" in res.stderr
