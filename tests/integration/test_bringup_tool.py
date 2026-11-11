"""S1–S6 bring-up tool ① against the firmware core on a fake board (the P0-7 MCU model, pty). What this proves is the
design's section 5 scope: framing, arming to ready, the step and the scenarios' software timelines, the cleanup and
the record -- not physical braking, currents, IWDG or stopping distance (the model's wheel is synthetic)."""

import io
import json
import os
import shutil
import signal
import subprocess
import sys
import time
import tty
from pathlib import Path

import pytest

from forklift_core.mcu_link.messages import Mode

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
import bringup  # noqa: E402
from real_mcu_link import TtyLink, service_fd  # noqa: E402


@pytest.fixture(scope="module")
def mcu_model(tmp_path_factory) -> Path:
    if shutil.which("cmake") is None or (shutil.which("cc") is None and shutil.which("gcc") is None):
        pytest.skip("cmake or a C compiler missing")
    build = tmp_path_factory.mktemp("mcu_model_build")
    env = {**os.environ, "PYTHONPATH": str(REPO / "src")}
    for cmd in (["cmake", "-S", str(REPO / "firmware"), "-B", str(build), "-DFW_MCU_MODEL=ON",
                 f"-DFW_PYTHON={sys.executable}"], ["cmake", "--build", str(build), "--target", "mcu_model", "-j2"]):
        r = subprocess.run(cmd, capture_output=True, text=True, env=env)
        assert r.returncode == 0, r.stdout + r.stderr
    return build / "mcu_model"


class PtyLink:
    """The bench's link on a pty master, with the bring-up TX gate."""

    def __init__(self, fd: int) -> None:
        self.tty, self.tx_enabled = TtyLink(fd), True

    def service(self, host, now_ns):
        return service_fd(self.tty, host, now_ns, tx_enabled=self.tx_enabled)


@pytest.fixture
def bench(mcu_model):
    master, slave = os.openpty()
    tty.setraw(slave)
    os.set_blocking(master, False)
    proc = subprocess.Popen([str(mcu_model), "--link", os.ttyname(slave)])
    out = io.StringIO()
    b = bringup.Bench(PtyLink(master), out)
    yield b
    proc.send_signal(signal.SIGTERM)
    proc.wait(timeout=5)
    os.close(master)
    os.close(slave)


def rows(b, kind):
    return [json.loads(x) for x in b.out.getvalue().splitlines() if json.loads(x)["t"] == kind]


def test_record_keeps_every_state_with_its_build_bits_and_reset_cause(bench):
    bench.run_until(lambda: len(bench.samples) >= 20, 3.0)
    states = rows(bench, "state")
    assert len(states) == len(bench.samples) >= 20
    assert states[0]["reset_cause"] == "POWER" and states[0]["mode"] in ("BOOT", "DISARMED")
    ts = [s["mcu_time_us"] for s in states]
    assert ts == sorted(ts)


def test_a_step_arms_to_ready_steps_and_disarms(bench):
    session = bringup.Session(bench, 1.0)
    res = bringup.run_step(bench, session, "both", 0.8)
    assert res["result"] == "ok", res
    assert res["rise_s"] is not None and res["input_delay_host_s"] is not None and res["input_delay_host_s"] >= 0
    assert bench.state.mode == Mode.DISARMED and rows(bench, "cleanup")[-1]["confirmed"]


def test_a_target_over_the_cap_is_refused_before_anything_moves(bench):
    session = bringup.Session(bench, 1.0)
    session.arm(2.0)
    with pytest.raises(bringup.Refused):
        session.send(1.5, 0.0, time.monotonic_ns() + 10**8)
    assert session.cleanup()


@pytest.mark.parametrize("kind", ["tx_cut", "expiry", "disarm"])
def test_a_scenario_records_its_software_timeline_without_aborting_on_the_expected_stop(bench, kind):
    session = bringup.Session(bench, 1.0)
    res = bringup.run_scenario(bench, session, kind, 0.8)
    assert res["result"] == "ok" and res["rolling_confirmed"], res
    sw = res["software"]
    assert sw["mode_change_ms"] is not None and sw["mode_change_ms"] >= 0
    if kind == "tx_cut":
        assert res["worst_case"]["met"] and res["worst_case"]["accepted"]
        assert sw["mode_change_ms"] >= 100  # the last DRIVE's validity (cap 150 ms) runs out first
    assert bench.state.mode == Mode.DISARMED
    assert session.aborted is None


def test_the_verdict_takes_only_external_measurements():
    v = bringup.verdict("tx_cut", {"stop_time_ms": "420", "stop_distance_mm": "unavailable"})
    assert v["stop_time_ms"]["pass"] is True and v["stop_distance_mm"] == {"value": "unavailable", "limit": 100.0,
                                                                           "pass": None}
    assert "output_response_ms" not in v  # the link loss has no 0.1 s output-response bound (plan 3.4)
    assert bringup.verdict("disarm", {"output_response_ms": "150"})["output_response_ms"]["pass"] is False
