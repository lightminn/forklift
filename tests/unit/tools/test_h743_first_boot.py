"""tools/h743_first_boot.py: reads a bring-up record and prints the S2 first-boot verdict; refuses a synthetic
record unless asked, and a record without the v3 KTH counters is invalid, never a pass."""

import dataclasses
import json
import sys
from pathlib import Path

from forklift_core.mcu_link.messages import FaultFlag, McuState, Mode, RejectReason, StateFlag
from forklift_core.plant.bringup import Sample
from forklift_core.plant.record_rows import state_row

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import h743_first_boot  # noqa: E402

OK = McuState(mcu_time_us=0, boot_id=7, arm_gen=0, last_rx_seq=0, mode=Mode.DISARMED,
              reject_reason=RejectReason.NONE, fault_flags=FaultFlag(0), wheel_count_left=0, wheel_count_right=0,
              wheel_rate_left_rad_s=0.0, wheel_rate_right_rad_s=0.0, steering_rad=0.1, lift_m=0.05,
              current_ma=(0,) * 4, duty_e4=(0,) * 4, driver_status=(0xC0,) * 4, rx_errors=0, cycle_overruns=0,
              flags=StateFlag.DRIVERS_READY | StateFlag.OUTPUTS_LOCKED)


def write(tmp_path, header, n=1100, **change):
    rows = [header] if header else []
    for k in range(n):
        st = dataclasses.replace(OK, mcu_time_us=k * 10_000, kth_frames=(10 * k, 10 * k), **change)
        rows.append(state_row(Sample(k * 10_000_000, st)))
    path = tmp_path / "run.jsonl"
    path.write_text("".join(json.dumps(r) + "\n" for r in rows), encoding="utf-8")
    return path


def run(capsys, *argv):
    code = h743_first_boot.main(list(argv))
    return code, json.loads(capsys.readouterr().out)


def test_pass_and_fail_exit_codes(tmp_path, capsys):
    real = {"t": "header", "device": "/dev/forklift-mcu"}
    code, out = run(capsys, "--record", str(write(tmp_path, real)))
    assert code == 0 and out["result"] == "pass"
    code, out = run(capsys, "--record", str(write(tmp_path, real, current_ma=(0, 0, 400, 0))))
    assert code == 1 and out["result"] == "fail" and not out["checks"]["current"]["ok"]


def test_a_synthetic_record_is_refused_or_never_a_pass(tmp_path, capsys):
    path = write(tmp_path, {"t": "header", "device": "/dev/pts/7", "synthetic": {"bench": {}}})
    code, out = run(capsys, "--record", str(path))
    assert code == 2 and out["result"] == "refused"
    code, out = run(capsys, "--record", str(path), "--allow-synthetic")
    assert code == 1 and out["result"] == "synthetic" and out["not_for_judgement"]


def test_two_appended_runs_are_invalid(tmp_path, capsys):
    path = write(tmp_path, {"t": "header", "device": "/dev/forklift-mcu"})
    text = path.read_text()
    path.write_text(text + text)  # bringup.py appends a second run to the same file (Codex P2-3)
    code, out = run(capsys, "--record", str(path))
    assert code == 1 and out["result"] == "invalid" and "several runs" in out["why"]


def test_malformed_v3_counters_are_invalid(tmp_path, capsys):
    for bad in ([], "12", None, [1], [1, -1], [1, 70000], [True, 1]):
        path = write(tmp_path, {"t": "header", "device": "/dev/forklift-mcu"})
        rows = [json.loads(x) for x in path.read_text().splitlines()]
        rows[5]["kth_crc_errors"] = bad
        path.write_text("".join(json.dumps(r) + "\n" for r in rows))
        code, out = run(capsys, "--record", str(path))
        assert code == 1 and out["result"] == "invalid", bad


def test_a_record_without_v3_counters_is_invalid(tmp_path, capsys):
    path = write(tmp_path, {"t": "header", "device": "/dev/forklift-mcu"})
    rows = [json.loads(x) for x in path.read_text().splitlines()]
    for r in rows:
        r.pop("kth_alt_crc_errors", None)
    path.write_text("".join(json.dumps(r) + "\n" for r in rows))
    code, out = run(capsys, "--record", str(path))
    assert code == 1 and out["result"] == "invalid"


def test_absent_parts_come_from_the_header_and_a_disagreeing_flag_is_refused(tmp_path, capsys):
    """Partial-parts plan v4: the capture's header names the absent parts; steering absent is no gate pass."""
    nan_steer = dict(steering_rad=float("nan"), fault_flags=FaultFlag.STEER_SENSOR_INVALID)
    hdr = {"t": "header", "device": "/dev/forklift-mcu", "cmd": "record", "parts_absent": ["steer"]}
    path = write(tmp_path, hdr, **nan_steer)
    code, out = run(capsys, "--record", str(path))
    assert code == 2 and out["result"] == "refused" and "parts were absent" in out["why"]
    code, out = run(capsys, "--record", str(path), "--allow-bypassed")
    assert out["parts_absent"] == ["steer"] and out["not_for_gate"] and out["steer_absent"] is True
    assert code == 1 and out["result"] == "not_for_gate" and out["computed_result"] == "pass"
    code, out = run(capsys, "--record", str(path), "--allow-bypassed", "--lift-absent")
    assert code == 2 and out["result"] == "refused" and "disagrees" in out["why"]
    # lift alone stays gate-eligible, from the header without the flag
    nan_lift = dict(lift_m=float("nan"), fault_flags=FaultFlag.LIFT_SENSOR_INVALID)
    path = write(tmp_path, {**hdr, "parts_absent": ["lift"]}, **nan_lift)
    code, out = run(capsys, "--record", str(path))
    assert code == 0 and out["result"] == "pass" and out["lift_absent"] is True and not out.get("not_for_gate")
    # a record from before the field: the flag alone is taken
    path = write(tmp_path, {"t": "header", "device": "/dev/forklift-mcu"}, **nan_steer)
    code, out = run(capsys, "--record", str(path), "--steer-absent", "--allow-bypassed")
    assert out["parts_absent"] == ["steer"] and out["not_for_gate"] and out["computed_result"] == "pass"
