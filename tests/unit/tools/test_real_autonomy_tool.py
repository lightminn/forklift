"""Design 2026-11-19-real-assembly 1, 9: tools/real_autonomy.py --dry-assemble -- 2 for a structural refusal (the
repository's template, a missing or another map bundle, unreadable settings, a reference plan the map blocks), 0 for a
sound assembly whose reference plan succeeds on the surveyed map; the record names the options, the hooks, the caps
and the files' hashes. No device is opened."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml

from forklift_core.localization import slam_map as sm

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))
import real_autonomy as tool  # noqa: E402


def bundle(tmp_path, wall=False):
    """An empty surveyed field over s10a.yaml's bounds (0.1 m cells), optionally a wall across the approach."""
    folder = tmp_path / ("bundle_wall" if wall else "bundle")
    folder.mkdir()
    data = np.zeros((50, 70), np.int8)  # rows = y from -2.5, columns = x from -1.0
    if wall:
        data[:, 30] = 100  # x = 2.0 m, the whole field's width
    (folder / "map.posegraph").write_bytes(b"graph")
    (folder / "map.data").write_bytes(b"data")
    np.savez(folder / "grid.npz", data=data, origin=np.asarray((-1.0, -2.5, 0.0)), resolution_m=0.1)
    hashes = {name: sm.file_sha256(folder / name) for name in sm.BUNDLE_FILES}
    (folder / "manifest.json").write_text(json.dumps({"format": sm.FORMAT, "map_id": sm.map_id_of(hashes),
                                                      "files": hashes}))
    return folder, sm.map_id_of(hashes)


def filled(tmp_path, map_id):
    d = yaml.safe_load((ROOT / "config/real/s10a.yaml").read_text())
    d.update(map_id=map_id, check_pallet_pose=[3.0, 0.0, 0.0])
    d["pallet"].update(depth_m=0.6, width_m=0.4)
    path = tmp_path / f"s10a_{map_id[:6]}.yaml"
    path.write_text(yaml.safe_dump(d))
    return path


def run(*argv, out):
    code = tool.main(["--dry-assemble", "--out", str(out), *argv])
    return code, json.loads(out.read_text())


def test_the_template_is_refused_with_its_unset_values(tmp_path):
    code, rec = run(out=tmp_path / "a.json")
    assert code == 2 and rec["assembled"] is False
    assert {"map_id_unset", "pallet_depth_m_unset", "check_pallet_pose_unset"} <= set(rec["refused"])


def test_a_filled_configuration_assembles_and_plans_its_reference_on_the_map(tmp_path):
    folder, map_id = bundle(tmp_path)
    code, rec = run("--s10a", str(filled(tmp_path, map_id)), "--map-bundle", str(folder), out=tmp_path / "a.json")
    assert code == 0 and rec["assembled"] is True and rec["check_plan"] == "success", rec
    assert rec["options"]["dt"] == 1 / 50 and rec["options"]["s10_stop_at_lift"] is True
    caps = rec["curvature_caps"]
    assert caps["planner"] == caps["travel"] == caps["tracker"] == caps["preview"]
    assert rec["ready"].startswith("not_ready") and set(rec["hashes"]) == {"s10a", "settings", "vehicle"}


def test_no_bundle_or_another_bundle_is_refused(tmp_path):
    folder, map_id = bundle(tmp_path)
    cfg = filled(tmp_path, map_id)
    code, rec = run("--s10a", str(cfg), out=tmp_path / "a.json")
    assert code == 2 and any(r.startswith("map_id_mismatch") for r in rec["refused"])
    other = filled(tmp_path, "0" * 64)
    code, rec = run("--s10a", str(other), "--map-bundle", str(folder), out=tmp_path / "b.json")
    assert code == 2 and rec["refused"][0].startswith("map:")


def test_a_reference_plan_the_map_blocks_is_refused(tmp_path):
    folder, map_id = bundle(tmp_path, wall=True)
    code, rec = run("--s10a", str(filled(tmp_path, map_id)), "--map-bundle", str(folder), out=tmp_path / "a.json")
    assert code == 2 and rec["assembled"] is True and rec["check_plan"] != "success"


def test_unreadable_settings_are_refused(tmp_path):
    bad = tmp_path / "autonomy.yaml"
    bad.write_text("lift_target_m: [unclosed")
    code, rec = run("--settings", str(bad), out=tmp_path / "a.json")
    assert code == 2 and rec["refused"][0].startswith("settings_unreadable")
    partial = tmp_path / "partial.yaml"
    partial.write_text("lift_target_m: 0.2\n")
    code, rec = run("--settings", str(partial), out=tmp_path / "b.json")
    assert code == 2 and "setting_missing" in rec["refused"][0]


def test_an_unreadable_vehicle_file_is_a_structural_refusal(tmp_path):
    folder, map_id = bundle(tmp_path)
    empty = tmp_path / "vehicle.yaml"
    empty.write_text("")
    code, rec = run("--s10a", str(filled(tmp_path, map_id)), "--map-bundle", str(folder), "--vehicle", str(empty),
                    out=tmp_path / "a.json")
    assert code == 2 and rec["refused"][0].startswith("vehicle_unreadable")


def test_run_refuses_any_device_before_opening_anything(tmp_path, monkeypatch, capsys):
    """4c design 5: ``--run --device X`` exits 2 naming the refusal; nothing is opened (an open() of the device path
    would fail the test); without a device the run is refused as not wired."""
    import builtins

    opened = []
    real_open = builtins.open

    def spy(path, *a, **k):
        opened.append(str(path))
        return real_open(path, *a, **k)

    monkeypatch.setattr(builtins, "open", spy)
    assert tool.main(["--run", "--device", "/dev/ttyACM0"]) == 2
    assert not any("ttyACM0" in p for p in opened)
    record = json.loads(capsys.readouterr().out)
    assert record["refused"] == ["device_refused:s10_preconditions"] and record["run"] is False
    assert tool.main(["--dry-assemble", "--device", "/dev/ttyACM0"]) == 2  # a device never rides along a dry assembly
    capsys.readouterr()
    assert tool.main(["--run"]) == 2
    assert json.loads(capsys.readouterr().out)["refused"] == ["run_needs_hil"]


def test_a_hil_run_refuses_a_device_and_reserved_model_arguments_before_opening_anything(capsys):
    """6a-2b (design 10.3 ①): --device with --hil, a missing model or duration, a reserved model argument and a bad
    injection are refused before any pty or model exists."""
    assert tool.main(["--run", "--hil", "--device", "/dev/ttyACM0", "--hil-model", "/bin/false",
                      "--duration-s", "5"]) == 2
    assert "device_with_hil" in json.loads(capsys.readouterr().out)["refused"]
    assert tool.main(["--run", "--hil", "--hil-model", "/bin/false", "--duration-s", "0"]) == 2
    assert "duration_s" in json.loads(capsys.readouterr().out)["refused"]
    assert tool.main(["--run", "--hil", "--hil-model", "/bin/false", "--duration-s", "5",
                      "--hil-model-arg=--link-fd"]) == 2
    assert "hil_model_arg_reserved:--link-fd" in json.loads(capsys.readouterr().out)["refused"]
    assert tool.main(["--run", "--hil", "--hil-model", "/bin/false", "--duration-s", "5", "--hil-inject", "x:1"]) == 2
    assert "hil_inject_invalid:x:1" in json.loads(capsys.readouterr().out)["refused"]
    assert tool.main(["--run", "--hil", "--duration-s", "5"]) == 2
    assert "hil_model_missing" in json.loads(capsys.readouterr().out)["refused"]


def test_the_real_client_refuses_a_missing_device_a_pty_and_hil_before_opening(capsys):
    """The operating unlock U1 (plan 2.1): --run --client real needs a device that is not a pty, never with --hil."""
    import os

    assert tool.main(["--run", "--client", "real", "--precheck"]) == 2
    assert json.loads(capsys.readouterr().out)["refused"] == ["real_needs_device"]
    m, s = os.openpty()
    try:
        assert tool.main(["--run", "--client", "real", "--precheck", "--device", os.ttyname(s)]) == 2
        rec = json.loads(capsys.readouterr().out)
        assert rec["refused"] == ["real_device_is_pty"] and rec["opened"] == []
    finally:
        os.close(m)
        os.close(s)
    assert tool.main(["--run", "--client", "real", "--precheck", "--hil", "--device", "/dev/ttyACM0"]) == 2
    assert json.loads(capsys.readouterr().out)["refused"] == ["real_with_hil"]


def test_the_real_precheck_reads_the_first_state_judges_and_writes_nothing(monkeypatch, capsys):
    """U1 (plan 2.4): the board opened (here a pty standing in), its first STATE read, the preconditions printed with
    the unlock lock first, exit 2 -- and not one byte written to the board, the transport never granted."""
    import os
    import sys
    import tty

    sys.path.insert(0, str(tool.REPO / "tools"))
    import real_loop_transport as rlt

    from forklift_core.mcu_link.messages import McuState, Mode, RejectReason, StateFlag, encode

    m, s = os.openpty()
    tty.setraw(s)
    os.set_blocking(m, False)
    os.set_blocking(s, False)
    st = McuState(1000, 9, 0, 0, Mode.DISARMED, RejectReason.NONE, 0, 0, 0, 0.0, 0.0, 0.0, 0.0, (0,) * 4, (0,) * 4,
                  (0,) * 4, 0, 0, StateFlag(0))
    os.write(m, encode(st, 0))
    made = []

    def fake_open(path, **kw):
        t = rlt.RealTtyTransport(s, usb_product="forklift-mcu normal open v1", usb_serial="ABC")
        made.append(t)
        return t

    monkeypatch.setattr(rlt.RealTtyTransport, "open", staticmethod(fake_open))
    monkeypatch.setattr(tool, "_is_pty", lambda d: False)
    assert tool.main(["--run", "--client", "real", "--precheck", "--device", "/dev/forklift-mcu"]) == 2
    rec = json.loads(capsys.readouterr().out)
    assert rec["refused"] and "precondition:unlock_incomplete" not in rec["refused"]  # U3e-2: the lock is gone
    assert rec["board"]["state_seen"] is True and rec["board"]["usb_serial"] == "ABC"
    assert made and made[0].real_operating is False
    import errno

    try:
        assert os.read(m, 4096) == b""  # nothing written: not even the poison
    except BlockingIOError:
        pass
    except OSError as err:
        assert err.errno == errno.EIO  # the slave closed with nothing pending (written bytes would be read first)
    os.close(m)


def test_the_real_precheck_never_writes_its_record_to_a_device(tmp_path, capsys):
    """Codex unlock U1 review P2: --out naming a device (the board itself or an alias) is refused before anything."""
    import os

    m, s = os.openpty()
    try:
        name = os.ttyname(s)
        assert tool.main(["--run", "--client", "real", "--precheck", "--device", name, "--out", name]) == 2
        assert json.loads(capsys.readouterr().out)["refused"] == ["out_not_a_regular_file"]
        assert tool.main(["--run", "--client", "real", "--precheck", "--device", "/dev/x", "--out", "/dev/null"]) == 2
        assert json.loads(capsys.readouterr().out)["refused"] == ["out_not_a_regular_file"]
    finally:
        os.close(m)
        os.close(s)



def test_no_early_refusal_writes_its_record_to_a_device(capsys):
    """Codex unlock U1 review round 2 P2-1: every branch writes --out only to a regular file checked on its fd."""
    import errno
    import os
    import tty

    m, s = os.openpty()
    tty.setraw(s)
    os.set_blocking(m, False)
    name = os.ttyname(s)
    try:
        assert tool.main(["--run", "--client", "real", "--precheck", "--hil", "--device", "/dev/x", "--out", name]) == 2
        assert tool.main(["--run", "--client", "real", "--hil", "--device", "/dev/x", "--out", name]) == 2
        assert tool.main(["--run", "--replay", "x.jsonl", "--out", name]) == 2
        try:
            assert os.read(m, 65536) == b""
        except BlockingIOError:
            pass
        except OSError as err:
            assert err.errno == errno.EIO
    finally:
        os.close(m)
        os.close(s)
    capsys.readouterr()



def test_a_real_refusal_is_one_printed_document_and_writes_no_out(tmp_path, capsys):
    """Codex unlock U1 review round 3 P2 and U3e-1 implementation round 1 P2-4: every REAL invocation's early refusal
    is a single printed document; it never opens --out (an existing file keeps its bytes, a missing one is not made)."""
    missing = tmp_path / "no" / "such" / "dir" / "out.json"
    assert tool.main(["--run", "--client", "real", "--precheck", "--hil", "--device", "/dev/x", "--out",
                      str(missing)]) == 2
    rec = json.loads(capsys.readouterr().out)  # one document
    assert rec["refused"] == ["real_with_hil"] and "out_error" not in rec and not missing.parent.exists()
    keep = tmp_path / "run.jsonl"
    keep.write_bytes(b"another run\n")
    for args in (["--run", "--client", "real", "--precheck", "--replay", "x.jsonl", "--out", str(keep)],
                 ["--run", "--client", "real", "--precheck", "--device", "/dev/x", "--out", str(keep)]):
        assert tool.main(args) == 2
        rec = json.loads(capsys.readouterr().out)
        assert rec["refused"] in (["real_with_replay"], ["out_exists"]), rec
        assert keep.read_bytes() == b"another run\n"


def test_a_dry_assembly_that_cannot_write_its_record_is_not_a_success(tmp_path, capsys):
    """Codex unlock U1 review round 4 P2: --out failing makes the exit non-zero even when the plan succeeds."""
    missing = tmp_path / "no" / "dir" / "out.json"
    code = tool.main(["--dry-assemble", "--out", str(missing)])
    rec = json.loads(capsys.readouterr().out)
    assert code == 2 and rec.get("out_error", "").startswith("FileNotFoundError")



# --- the operating REAL run's arguments (unlock U3e-1, plan 6.1 step 1) ----------------------------------------------
def _real_args(tmp_path, *extra):
    site = tmp_path / "site.yaml"
    site.write_text("x: 1\n")
    return ["--run", "--client", "real", "--device", "/dev/forklift-mcu-none", "--scan-socket",
            str(tmp_path / "scan.sock"), "--slam-bridge", str(tmp_path / "slam.sock"), "--site-record", str(site),
            "--bundle-root", str(tmp_path), "--duration-s", "30", "--out-dir", str(tmp_path / "run"), *extra]


def test_the_operating_run_refuses_hil_and_replay_arguments_before_opening(tmp_path, capsys):
    for extra, why in ((["--hil"], "real_with_hil"), (["--hil-model", "/bin/false"], "real_with_hil_model"),
                       (["--hil-model-arg=--boot-id"], "real_with_hil_model_arg"),
                       (["--hil-inject", "scan:1"], "real_with_hil_inject"),
                       (["--hil-slam-bridge", "/x"], "real_with_hil_slam_bridge"),
                       (["--replay", "x.jsonl"], "real_with_replay"), (["--allow-synthetic"], "real_with_allow_synthetic"),
                       (["--map-bundle", "/x"], "real_with_map_bundle"), (["--imu-serial", "1"], None)):
        if why is None:  # U3e-1 has no IMU device argument at all
            with pytest.raises(SystemExit):
                tool.main(_real_args(tmp_path, *extra))
            capsys.readouterr()
            continue
        assert tool.main(_real_args(tmp_path, *extra)) == 2
        rec = json.loads(capsys.readouterr().out)
        assert why in rec["refused"] and rec["opened"] == [], (extra, rec)
    assert not (tmp_path / "run").exists()  # nothing made before the refusal


def test_the_operating_run_refuses_missing_arguments_a_pty_and_an_existing_folder(tmp_path, capsys):
    import os

    assert tool.main(["--run", "--client", "real"]) == 2
    rec = json.loads(capsys.readouterr().out)
    for why in ("real_needs_device", "real_needs_scan_socket", "real_needs_slam_bridge", "real_needs_site_record",
                "real_needs_bundle_root", "real_needs_out_dir", "duration_s"):
        assert why in rec["refused"]
    m, s = os.openpty()
    try:
        args = _real_args(tmp_path)
        args[args.index("--device") + 1] = os.ttyname(s)
        assert tool.main(args) == 2
        assert "real_device_is_pty" in json.loads(capsys.readouterr().out)["refused"]
    finally:
        os.close(m)
        os.close(s)
    (tmp_path / "run").mkdir()
    assert tool.main(_real_args(tmp_path)) == 2
    assert "out_dir_exists" in json.loads(capsys.readouterr().out)["refused"]


def test_the_operating_runs_inputs_must_be_regular_files(tmp_path, capsys):
    """Codex U3e-1 round 3 P2-3: a FIFO given as an input is refused before the device's lock could be held on it."""
    import os

    fifo = tmp_path / "pre.fifo"
    os.mkfifo(fifo)
    assert tool.main(_real_args(tmp_path, "--preconditions", str(fifo))) == 2
    rec = json.loads(capsys.readouterr().out)
    assert "input_not_regular:preconditions" in rec["refused"]
    assert tool.main(_real_args(tmp_path, "--settings", str(tmp_path / "none.yaml"))) == 2
    assert "input_missing:settings" in json.loads(capsys.readouterr().out)["refused"]


def test_the_operating_run_never_writes_out_and_keeps_an_existing_files_bytes(tmp_path, capsys):
    """Codex U3e-1 round 3 P2-2: a refusal of the operating run is printed only -- an existing file given as --out
    (another run's record, say) keeps every byte."""
    keep = tmp_path / "run.jsonl"
    keep.write_bytes(b"earlier run\n")
    assert tool.main(_real_args(tmp_path, "--out", str(keep), "--hil")) == 2
    rec = json.loads(capsys.readouterr().out)
    assert "real_run_takes_out_dir_not_out" in rec["refused"] and "out_error" not in rec
    assert keep.read_bytes() == b"earlier run\n"


def test_the_operating_console_stays_on_loopback_unless_lan_is_named(tmp_path, capsys):
    assert tool.main(_real_args(tmp_path, "--host", "0.0.0.0")) == 2
    assert "console_host_needs_console_lan" in json.loads(capsys.readouterr().out)["refused"]
    assert tool.main(_real_args(tmp_path, "--host", "0.0.0.0", "--console-lan")) == 2
    assert "console_lan_needs_origin_host" in json.loads(capsys.readouterr().out)["refused"]
    args = tool.argparse.Namespace(console_lan=True, console_origin_host="192.168.4.1")
    assert tool.real_origin_hosts(args) == ("127.0.0.1", "localhost", "192.168.4.1")


def test_the_precheck_refuses_a_fifo_input_before_the_device(tmp_path, capsys, monkeypatch):
    """Codex U3e-1 implementation round 2 P2-3: a FIFO given to the precheck would block its read with the lock held."""
    import os

    fifo = tmp_path / "layer.fifo"
    os.mkfifo(fifo)
    opened = []
    monkeypatch.setattr(tool, "_is_pty", lambda d: False)
    assert tool.main(["--run", "--client", "real", "--precheck", "--device", "/dev/forklift-mcu-none",
                      "--layer", str(fifo)]) == 2
    rec = json.loads(capsys.readouterr().out)
    assert rec["refused"] == ["input_not_regular:layer"] and rec["opened"] == [] and not opened


def test_the_operating_scan_socket_never_replaces_an_existing_name(tmp_path):
    """Codex U3e-1 implementation round 2 P2-2: a stale socket left at the name is not replaced (bind fails)."""
    import socket

    path = tmp_path / "scan.sock"
    stale = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    stale.bind(str(path))
    stale.close()  # nobody holds it now: scan_feed.bind_socket would replace it
    ino = path.lstat().st_ino
    from types import SimpleNamespace

    held = SimpleNamespace(scan_path=None, scan_ino=None, scan_keep=None)
    with pytest.raises(OSError):
        tool._bind_own_socket(path, held)  # noqa: SLF001
    assert path.lstat().st_ino == ino and held.scan_path is None  # not this run's name: never removed by it
    path.unlink()
    s = tool._bind_own_socket(path, held)  # noqa: SLF001
    try:
        assert oct(path.stat().st_mode & 0o777) == oct(0o660) and s.getblocking() is False
    finally:
        s.close()
        tool._remove_scan_path(held)  # noqa: SLF001


def test_a_bound_scan_name_is_owned_from_the_bind_and_removed_while_held(tmp_path):
    """Codex U3e-1 implementation round 3 P2-1: the name is the run's from the bind on (its inode and a kept copy);
    the cleanup removes it while that copy still holds the socket, then closes the copy."""
    import socket
    from types import SimpleNamespace

    path = tmp_path / "scan.sock"
    held = SimpleNamespace(scan_path=None, scan_ino=None, scan_keep=None)
    s = tool._bind_own_socket(path, held)  # noqa: SLF001
    s.close()  # the sources' copy closed first (as make_real_sources on a failure)
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        probe.connect(str(path))  # still live: no receiver could take the name over as stale
    finally:
        probe.close()
    tool._remove_scan_path(held)  # noqa: SLF001
    assert not path.exists() and held.scan_keep is None


def test_the_precheck_refuses_a_fifo_the_vehicle_file_names(tmp_path, capsys, monkeypatch):
    """Codex U3e-1 implementation round 3 P2-2: the chassis model vehicle.yaml names is read by the judgement too."""
    import os

    fifo = tmp_path / "chassis.fifo"
    os.mkfifo(fifo)
    veh = tmp_path / "vehicle.yaml"
    d = yaml.safe_load((tool.REPO / "config/real/vehicle.yaml").read_text())
    d["chassis_model"] = str(fifo)
    veh.write_text(yaml.safe_dump(d))
    monkeypatch.setattr(tool, "_is_pty", lambda d: False)
    assert tool.main(["--run", "--client", "real", "--precheck", "--device", "/dev/forklift-mcu-none",
                      "--vehicle", str(veh)]) == 2
    rec = json.loads(capsys.readouterr().out)
    assert "input_not_regular:chassis_model" in rec["refused"] and rec["opened"] == []


def test_a_failed_copy_before_the_bind_leaves_no_name(tmp_path, monkeypatch):
    """Codex U3e-1 implementation round 4 P2-1: the kept copy is made before the bind, so a copy that fails (EMFILE)
    leaves no bound name behind and nothing in ``held``."""
    import socket
    from types import SimpleNamespace

    def no_dup(self):
        raise OSError(24, "Too many open files")

    monkeypatch.setattr(socket.socket, "dup", no_dup)
    path = tmp_path / "scan.sock"
    held = SimpleNamespace(scan_path=None, scan_ino=None, scan_keep=None)
    with pytest.raises(OSError):
        tool._bind_own_socket(path, held)  # noqa: SLF001
    assert not path.exists() and held.scan_path is None and held.scan_keep is None


def test_a_broken_vehicle_file_is_a_refusal_not_a_traceback(tmp_path):
    """Codex U3e-1 implementation round 4 P3: a vehicle.yaml that does not parse is refused by name."""
    veh = tmp_path / "vehicle.yaml"
    veh.write_text("chassis_model: [unclosed\n")
    out = tool._vehicle_files_refused(veh)  # noqa: SLF001
    assert len(out) == 1 and out[0].startswith("input_unreadable:vehicle:")


def test_a_transient_inode_read_after_the_bind_is_retried_and_the_name_removed(tmp_path, monkeypatch):
    """Codex U3e-1 implementation round 5 P2: the first lstat after the bind failing (ESTALE) is retried while held."""
    import errno
    import os
    from types import SimpleNamespace

    path = tmp_path / "scan.sock"
    real_lstat, calls = os.lstat, []

    def flaky(p, *a, **k):
        if str(p) == str(path) and not calls:
            calls.append(p)
            raise OSError(errno.ESTALE, "Stale file handle")
        return real_lstat(p, *a, **k)

    monkeypatch.setattr(tool.os, "lstat", flaky)
    held = SimpleNamespace(scan_path=None, scan_ino=None, scan_keep=None)
    s = tool._bind_own_socket(path, held)  # noqa: SLF001
    s.close()
    assert calls and held.scan_ino is not None
    assert tool._remove_scan_path(held) is None and not path.exists()  # noqa: SLF001


def test_a_name_whose_inode_was_never_read_is_reported_not_removed_blind(tmp_path):
    """Codex U3e-1 implementation round 5 P2: without the inode the name is left and named for the end row."""
    import socket
    from types import SimpleNamespace

    path = tmp_path / "scan.sock"
    s = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    s.bind(str(path))
    held = SimpleNamespace(scan_path=str(path), scan_ino=None, scan_keep=s)
    assert tool._remove_scan_path(held) == "inode_unknown"  # noqa: SLF001
    assert path.exists() and held.scan_keep is None


class _Loop:
    closing_started_ns, closing_confirmed_ns, closing_disarms = 1, 2, [(1, 2)]
    session = SimpleNamespace(results=[])
    permit = SimpleNamespace(state="revoked", reason="closing")


class _Run:
    def __init__(self, fail):
        self.fail, self.loop, self.ticks, self.order = fail, _Loop(), 0, []

    def start_session(self, now):
        self.order.append("session")

    def run(self, duration_s, stop, record):
        raise self.fail

    def close(self, record):
        self.order.append("close")
        return "confirmed"


class _Rec:
    def put(self, row):
        pass


class _Interrupt(BaseException):
    pass


def test_a_non_exception_still_closes_and_leaves_the_verdict_in_the_end_row():
    """Codex U3e-1 implementation round 5 P3: the closing's verdict and times reach the end row before it propagates."""
    run, end = _Run(_Interrupt()), {"reason": None}
    with pytest.raises(_Interrupt):
        tool._run_session(run, SimpleNamespace(counts={}), 1.0, None, _Rec(), SimpleNamespace(trace=None), end)  # noqa: SLF001
    assert run.order == ["session", "close"]
    assert end["closing"] == "confirmed" and end["closing_confirmed_ns"] == 2 and end["reason"].startswith("exception")


def test_a_run_exception_writes_nothing_to_stderr_before_the_device_is_released(monkeypatch, tmp_path):
    """Codex U3e-1 implementation rounds 4-5: the trace is kept, the closing runs, and stderr -- even a broken one --
    is written only after the cleanup has released the device."""
    order = []
    run, held, end = _Run(RuntimeError("boom")), SimpleNamespace(trace=None), {"reason": None}

    class BrokenErr:
        def write(self, s):
            order.append("stderr")
            raise BrokenPipeError(32, "Broken pipe")

        def flush(self):
            raise BrokenPipeError(32, "Broken pipe")

    monkeypatch.setattr(tool.sys, "stderr", BrokenErr())
    reason, closing = tool._run_session(run, SimpleNamespace(counts={}), 1.0, None, _Rec(), held, end)  # noqa: SLF001
    assert closing == "confirmed" and reason.startswith("exception:RuntimeError") and "boom" in held.trace
    assert order == []  # nothing written during the session

    def body(args, rlt, rr, refused, record, held_, end_, stop, factory):
        held_.trace = held.trace
        raise RuntimeError("after the closing")

    def cleanup(held_, record, end_):
        order.append("cleanup")
        return True

    monkeypatch.setattr(tool, "_real_body", body)
    monkeypatch.setattr(tool, "_real_cleanup", cleanup)
    args = tool.build_parser().parse_args(["--run", "--client", "real", "--out-dir", str(tmp_path / "rec")])
    assert tool.run_real(args, transport_factory=lambda d: None) == tool.EXIT_FAILED
    assert order == ["cleanup", "stderr"]


def test_a_failed_summary_keeps_the_disarm_evidence_and_names_the_gap(monkeypatch):
    """Codex U3e-1 implementation round 6 P3: the DISARM times and frames are written on their own, and a summary
    that fails is named in the end row (which makes the run's exit non-zero)."""

    def boom(results):
        raise ValueError("bad results")

    class Done(_Run):
        def run(self, duration_s, stop, record):
            return "deadline"

    monkeypatch.setattr(tool, "mission_counts", boom)
    end = {"reason": None}
    reason, closing = tool._run_session(Done(None), SimpleNamespace(counts={}), 1.0, None, _Rec(),  # noqa: SLF001
                                        SimpleNamespace(trace=None), end)
    assert (reason, closing) == ("deadline", "confirmed")
    assert end["closing_confirmed_ns"] == 2 and end["closing_disarms"] == [[1, 2]]
    assert end["end_fields_error"].startswith("summary:ValueError")
