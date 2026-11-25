"""7a-3b (plan 2026-11-26 2 and 5): the scan image's one tag and patch 0002's contract with the 7a-3a adapter.

The tag is written in config/real/lidar.yaml only, computed from the patches and Dockerfile.scan by
tools/scan_image.py; the build entry points and the launchers read it from there. Patch 0002's node speaks the
formats the adapter (ros2/.../real_scan_adapter.py, 7a-3a) takes."""

import json
import re
import subprocess
import sys
from pathlib import Path

import pytest
import yaml

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import scan_image as si  # noqa: E402

PATCH2 = REPO / "deploy" / "ros2" / "sllidar_patches" / "0002-sdk-arrival-stamp.patch"


def _adapter():
    sys.path.insert(0, str(REPO / "ros2" / "src" / "forklift_ros" / "forklift_ros"))
    import real_scan_adapter as ra

    return ra


# --- the one tag ---------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("arch", si.ARCHES)
def test_lidar_yaml_names_the_tag_the_inputs_give(arch):
    """Change a patch or Dockerfile.scan and this fails until lidar.yaml scan_image is updated."""
    assert si.configured(arch) == si.tag(arch)


def test_the_reader_reads_what_yaml_reads():
    raw = yaml.safe_load(si.LIDAR_YAML.read_text(encoding="utf-8"))["scan_image"]
    assert raw == {a: si.configured(a) for a in si.ARCHES}


def test_the_tag_is_written_only_in_lidar_yaml():
    """The lead's check for 7a-3b: no tracked file but lidar.yaml carries a scan image tag. git ls-files in a
    checkout, the tree itself in the ws1 pytest copy (no .git -- Codex 7a-3b round 1 P2-4): the same listing as
    test_no_session_tmp_paths."""
    sys.path.insert(0, str(REPO / "tests" / "unit"))
    from test_no_session_tmp_paths import text_of, tracked_files

    pat = re.compile(r"forklift/ros2-scan:jazzy-s[0-9a-f]{8}")
    hits = []
    for f in tracked_files():
        text = text_of(REPO / f)
        if text is not None and pat.search(text):
            hits.append(f)
    assert hits == ["config/real/lidar.yaml"]


def test_every_input_byte_and_the_base_move_the_tag(tmp_path):
    root = tmp_path / "r"
    for p in [*si.inputs(), si.BASES]:
        dst = root / p.relative_to(REPO)
        dst.parent.mkdir(parents=True, exist_ok=True)
        dst.write_bytes(p.read_bytes())
    tags = {a: si.tag(a, root) for a in si.ARCHES}
    assert tags == {a: si.tag(a) for a in si.ARCHES}
    for p in si.inputs(root):
        saved = p.read_bytes()
        p.write_bytes(saved + b" ")
        assert si.tag("amd64", root) != tags["amd64"], p.name
        p.write_bytes(saved)
    # another base (Codex 7a-3b round 1 P2-5): another tag, for that architecture only
    bases = root / "deploy" / "ros2" / "scan_image_base.txt"
    text = bases.read_text(encoding="utf-8")
    amd_id = si.base("amd64")[1]
    bases.write_text(text.replace(amd_id, "sha256:" + "0" * 64), encoding="utf-8")
    assert si.tag("amd64", root) != tags["amd64"] and si.tag("arm64", root) == tags["arm64"]
    assert tags["arm64"].endswith("-arm64") and tags["arm64"][:-6] != tags["amd64"]


@pytest.mark.parametrize("bad", ["amd64 forklift/ros2-dev:jazzy sha256:abc", "amd64 x sha256:" + "a" * 64 + " extra"])
def test_a_malformed_base_line_is_refused(tmp_path, bad):
    d = tmp_path / "deploy" / "ros2"
    d.mkdir(parents=True)
    (d / "scan_image_base.txt").write_text(bad + "\n", encoding="utf-8")
    with pytest.raises(ValueError):
        si.base("amd64", tmp_path)


def test_check_stops_when_lidar_yaml_differs(tmp_path, monkeypatch, capsys):
    other = tmp_path / "lidar.yaml"
    text = si.LIDAR_YAML.read_text(encoding="utf-8")
    other.write_text(re.sub(r"jazzy-s[0-9a-f]{8}", "jazzy-s00000000", text), encoding="utf-8")
    monkeypatch.setattr(si, "LIDAR_YAML", other)
    monkeypatch.setattr(si.configured, "__defaults__", (other,))
    assert si.main(["check", "--arch", "amd64"]) == 1
    assert "update lidar.yaml" in capsys.readouterr().err


def test_the_tarball_is_named_from_the_tag():
    assert si.tarball("arm64") == si.configured("arm64").split("/")[1].replace(":", "-") + ".tar.zst"


def test_the_build_entry_points_and_launchers_read_the_tag_from_lidar_yaml():
    sbatch = (REPO / "deploy/slurm/scan_image_build.sbatch").read_text(encoding="utf-8")
    assert "tools/scan_image.py\" check --arch amd64" in sbatch and "all checks passed" in sbatch
    assert 'scan_image.py" base --arch amd64' in sbatch and '"$HAVE" == "$BASE_ID"' in sbatch
    assert 'docker tag "$BASE_ID" "$PINNED"' in sbatch and '--build-arg "BASE_IMAGE=$PINNED"' in sbatch
    arm = (REPO / "tools/scan_image_build_arm64.sh").read_text(encoding="utf-8")
    assert "tools/scan_image.py check --arch arm64" in arm and "--platform linux/arm64" in arm
    assert "scan_image.py base --arch arm64" in arm and "FORKLIFT_ARM64_BASE" not in arm
    assert 'docker tag "$base_id" "$pinned"' in arm and 'BASE_IMAGE=${pinned}' in arm
    launcher = (REPO / "tools/ros2_dev.sh").read_text(encoding="utf-8")
    assert "tools/scan_image.py\" configured --arch" in launcher
    for f in ("tools/hwday/jetson_prep.sh", "tools/hwday/jetson_a2m12.sh"):
        text = (REPO / f).read_text(encoding="utf-8")
        assert "scan_image.py\" configured --arch arm64" in text and "jazzy-arm64.tar.zst" not in text, f
        assert "/home/light" not in text, f  # no fallback to an old tarball elsewhere (plan 5)


def test_the_dockerfile_applies_both_patches_and_runs_the_sdk_test():
    text = (REPO / "deploy/ros2/Dockerfile.scan").read_text(encoding="utf-8")
    a, b = text.index("0001-sdk-first-sample-stamp.patch"), text.index("0002-sdk-arrival-stamp.patch")
    assert a < b and "-DBUILD_TESTING=ON" in text and "forklift_arrival_test" in text
    assert text.index("ctest") < text.index("rm -rf /opt/sllidar_ws/build")  # before the build tree goes


# --- patch 0002 speaks what the adapter takes ----------------------------------------------------------------------

def _info_format() -> str:
    """The node's scan_sdk_info snprintf format, as C source in the patch."""
    lines = [ln[1:] for ln in PATCH2.read_text(encoding="utf-8").splitlines() if ln.startswith("+")]
    src = "\n".join(lines)
    for m in re.finditer(r'std::snprintf\(buf, sizeof\(buf\),\s*((?:"(?:[^"\\]|\\.)*"\s*)+),', src):
        fmt = "".join(re.findall(r'"((?:[^"\\]|\\.)*)"', m.group(1))).replace('\\"', '"')
        if "sdk_mono_v2" in fmt:
            return fmt
    raise AssertionError("scan_sdk_info format not found")


def test_the_info_the_node_writes_is_what_the_adapter_takes():
    fmt = _info_format()
    # %s run, %s start, %u ans_type, %.9g sample period, %d baud -- filled as the C library does
    text = (fmt.replace("{", "{{").replace("}", "}}").replace("%s", "{}", 2).replace("%u", "{}").replace("%.9g", "{:.9g}").replace("%d", "{}")
            .format("a1b2c3d4e5f60718", "fedcba9876543210", 132, 31.25, 256000))
    assert json.loads(text) == {"format": "sdk_mono_v2", "run": "a1b2c3d4e5f60718", "start": "fedcba9876543210",
                                "ans_type": 132, "sample_period_us": 31.25, "baudrate": 256000}
    ra = _adapter()
    st = ra.SdkStream("a1b2c3d4e5f60718", {})
    assert st.on_info(text) is None and st.info["fedcba9876543210"]["baudrate"] == 256000


def test_the_frame_id_the_node_writes_is_what_the_adapter_takes():
    text = PATCH2.read_text(encoding="utf-8")
    assert 'frame_id + "@sdk_mono_v2#" + run_id + "." + start_id' in text
    assert '"%016llx"' in text  # the start id: 16 lowercase hex
    assert 'std::regex("^[0-9a-f]{8,32}$")' in text  # the run id the launcher gives (a-f first, 7a-3a)
    ra = _adapter()
    assert ra.RUN_ID.match("a1b2c3d4e5f60718") and ra._V2_ID.match("laser@sdk_mono_v2#a1b2c3d4e5f60718.0123456789abcdef")


def test_the_info_topic_is_latched_on_both_sides():
    text = PATCH2.read_text(encoding="utf-8")
    assert '"scan_sdk_info", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local()' in text
    adapter = (REPO / "ros2/src/forklift_ros/forklift_ros/real_scan_adapter.py").read_text(encoding="utf-8")
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in adapter and "ReliabilityPolicy.RELIABLE" in adapter
    launcher = (REPO / "tools/ros2_dev.sh").read_text(encoding="utf-8")
    assert f"scan_sdk_info:={_adapter().INFO_TOPIC}" in launcher


def test_the_node_publishes_scan_sdk_only_with_arrival_times_and_a_live_start():
    text = PATCH2.read_text(encoding="utf-8")
    assert "grabScanDataHqWithArrival(nodes, count, first_sample_us, last_sample_us, arrival_ok)" in text
    assert "if (sdk_ok && (!arrival_ok || start_id.empty() || first_sample_us < start_after_us))" in text
    assert "start_after_us = mono_us() + 150000ULL;" in text  # a restart's first rotation stays off (round 1 P2-2)
    assert "delete drv;" in text and "~SLlidarNode() override" in text  # threads end before exit (round 1 P2-3)
    assert "-                        max_distance, frame_id + \"@sdk_mono_v1\");" in text  # v1 no longer published
