"""PR 6a-2b (docs/design/2026-11-20-real-hil-6a.md 6, 10.3-10.8): the HIL run through its command line --
``tools/real_autonomy.py --run --hil`` on the MCU model behind the runner's own pty, the synthetic sources, the
console over its WebSocket. S10-A once to ``s10_lifted`` (the stop DRIVE and the mission's DISARM taken by the model
and confirmed before the closing), the stop injections (send cut, STATE cut, scan, SLAM) judged from the record and
the model's logs, a half DRIVE's general property, the closing DISARM never delivered, the model's lifetime and a
killed model. The scope is S10-A's commands, STATEs, odometry and stop transitions: the synthetic scan and the
identity SLAM come from the same pose, so nothing here is evidence of position accuracy or pocket contact."""

from __future__ import annotations

import asyncio
import json
import math
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import pytest

pytest.importorskip("websockets")

from websockets.asyncio.client import connect  # noqa: E402

REPO = Path(__file__).resolve().parents[2]
HIL_YAML = REPO / "config/real/s10a_hil.yaml"
ARMED, STOPPING, DISARMED = 2, 3, 1
WHEEL_TAU_S, MODEL_GAP_S, STOP_GRACE_S = 0.05, 0.005, 0.100  # plant.h, design 2, mcu_model --stop-grace-ms default
MCU_TIMEOUT_MS = 200  # mcu_model.c make_config: command_timeout_ms (drive.c window_for's cap)
A_NS, W_NS = 1_000_000, 35_000_000  # the write allowance and the write margin (real_loop)


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


class Runner:
    """The runner as a child (its own process group, so a test's SIGINT to the group reaches it and not pytest);
    the console address read from its first line."""

    def __init__(self, model, out: Path, duration_s=60, model_args=(), inject=()):
        self.out = out
        args = [sys.executable, str(REPO / "tools/real_autonomy.py"), "--run", "--hil", "--hil-model", str(model),
                "--duration-s", str(duration_s), "--config", str(HIL_YAML), "--out-dir", str(out),
                "--hil-model-arg=--accept-log", f"--hil-model-arg={out / 'accept.log'}",
                "--hil-model-arg=--axis-log", f"--hil-model-arg={out / 'axis.log'}"]
        args += [f"--hil-model-arg={a}" for a in model_args] + [f"--hil-inject={i}" for i in inject]
        self.proc = subprocess.Popen(args, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                                     env={**os.environ, "PYTHONPATH": str(REPO / "src")}, start_new_session=True)
        self.port = None
        end = time.monotonic() + 60
        for line in self.proc.stdout:
            if line.startswith('{"console"'):
                self.port = json.loads(line)["port"]
                break
            if time.monotonic() > end:
                break
        assert self.port is not None, self.proc.stderr.read() if self.proc.poll() is not None else "no console"

    def finish(self, sig=None, timeout=60):
        if sig is not None and self.proc.poll() is None:
            self.proc.send_signal(sig)
        out, err = self.proc.communicate(timeout=timeout)
        self.stderr = err
        return self.proc.returncode

    def rows(self):
        return [json.loads(line) for line in (self.out / "run.jsonl").read_text().splitlines() if line.strip()]

    def kill(self):
        if self.proc.poll() is None:
            self.proc.kill()
            self.proc.wait(5)


class Console:
    """The operator's console on its own thread: hello, then an echo for every tick while a lease is held; requests
    answered by request id; real_status kept."""

    def __init__(self, port):
        self.port, self.status, self.replies, self.lease, self.token, self.epoch = port, None, {}, None, None, None
        self._seq, self._rid = 0, 0
        self.loop = asyncio.new_event_loop()
        self._ready = threading.Event()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        asyncio.run_coroutine_threadsafe(self._open(), self.loop).result(15)

    async def _open(self):
        self.ws = await connect(f"ws://127.0.0.1:{self.port}/api/v1/control")
        await self.ws.send(json.dumps({"t": "hello", "api": "console/1", "client_id": "hil-test"}))
        welcome = json.loads(await self.ws.recv())
        self.token, self.epoch = welcome.get("request_token"), welcome["session_epoch"]
        self.reader = asyncio.ensure_future(self._read())

    async def _read(self):
        try:
            async for raw in self.ws:
                if isinstance(raw, bytes):
                    continue
                m = json.loads(raw)
                if m.get("t") == "tick" and self.lease is not None:
                    await self.ws.send(json.dumps({"t": "echo", "session_epoch": self.epoch, "lease_id": self.lease,
                                                   "echo": {"state_seq": m.get("state_seq"), "token": m.get("token")}}))
                elif m.get("t") == "real_status":
                    self.status = m
                elif m.get("request_id") is not None:
                    self.replies[m["request_id"]] = m
        except Exception:  # noqa: BLE001 -- the runner closed the connection
            pass

    def ask(self, kind, /, timeout=10.0, **fields):
        from forklift_core.safety.arbiter import REAL_LEASED

        self._rid += 1
        rid = f"h{self._rid}"
        msg = {"t": kind, "request_id": rid, "session_epoch": self.epoch, **fields}
        if self.status is not None:
            msg["control_gen"] = self.status["control_gen"]
        if self.lease is not None:
            msg["lease_id"] = self.lease
        if kind in REAL_LEASED or kind == "recovery":
            self._seq += 1
            msg.update(request_token=self.token, request_seq=self._seq)
        asyncio.run_coroutine_threadsafe(self.ws.send(json.dumps(msg)), self.loop).result(5)
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if rid in self.replies:
                return self.replies[rid]
            time.sleep(0.01)
        raise AssertionError(f"no answer to {kind}")

    def wait(self, cond, timeout):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if self.status is not None and cond(self.status):
                return self.status
            time.sleep(0.02)
        raise AssertionError(f"not reached: {self.status}")

    def close(self):
        async def _close():
            try:
                await self.ws.close()
            except Exception:  # noqa: BLE001
                pass
        asyncio.run_coroutine_threadsafe(_close(), self.loop).result(5)
        self.loop.call_soon_threadsafe(self.loop.stop)
        self.thread.join(5)


def start_s10(c: Console, answer=True):
    """6 (premise review P1-1): the session check, the lease, the D5 bypass applied, the pallet, the stand, the
    operator's start, the submit -- and the lifted check confirmed."""
    c.wait(lambda s: s["permit"]["state"] == "OFF" and s.get("standing"), 10)
    lease = c.ask("lease_request")
    assert lease["t"] == "ack", lease
    c.lease = lease["lease_id"]
    r = c.ask("check_bypass", check="d5_pocket", on=True)
    assert r["t"] == "ack", r
    c.wait(lambda s: "d5_pocket" in s["bypassed"], 5)
    r = c.ask("pallet_register", pallet_id="P-S10", pose=[2.6, 0.0, 0.0])
    assert r["t"] == "ack", r
    time.sleep(0.8)  # stands
    r = c.ask("operator_start")
    assert r["t"] == "ack", r
    r = c.ask("mission_submit", pallet_id="P-S10", destination_id="D-S10", targets_revision=r.get(
        "targets_revision", c.status["targets_revision"]))
    assert r["t"] == "ack", r
    if answer:
        s = c.wait(lambda s: s["operator_request"] is not None or s["permit"]["state"] == "LATCHED", 90)
        op = s["operator_request"]
        assert op is not None, s
        r = c.ask("operator_answer", kind=op["kind"], serial=op["serial"], verdict="confirm")
        assert r["t"] == "ack", r


# --- the records ----------------------------------------------------------------------------------------------------
def accept_rows(path: Path):
    out = []
    for line in path.read_text().splitlines():
        p = line.split()
        if len(p) == 8:
            out.append({"mcu_us": int(p[0]), "seq": int(p[1]), "kind": p[2], "verdict": p[3], "prior_mode": int(p[4]),
                        "prior_expiry_us": int(p[5]), "boot": int(p[6]), "mono_ns": int(p[7])})
    return out


def axis_rows(path: Path):
    return [(int(p[0]), int(p[2]), float(p[5]), float(p[6])) for p in
            (line.split() for line in path.read_text().splitlines()) if len(p) == 7]


def accepted(rows, kind, seq, boot, after_ns):
    """The model's acceptance of the frame written last at ``after_ns`` (its last write's entry): the first row of that
    boot, kind and seq at or after it (design 10.6)."""
    return next((r for r in rows if r["kind"] == kind and r["seq"] == seq and r["boot"] == boot
                 and r["mono_ns"] >= after_ns), None)


def events(rows, t):
    return [r for r in rows if r.get("t") == t]


def end_of(rows):
    ends = events(rows, "end")
    assert ends, "no end row: the record is not complete"
    return ends[-1]


# --- S10-A once (6, 10.4) --------------------------------------------------------------------------------------------
def test_s10_a_runs_to_the_lift_through_the_command_line(mcu_model, tmp_path):
    run = Runner(mcu_model, tmp_path, duration_s=120)
    c = Console(run.port)
    try:
        start_s10(c)
        c.wait(lambda s: s["permit"]["state"] == "LATCHED" and s["mission"]["status"] not in ("accepted", "preparing",
                                                                                              "running"), 60)
    finally:
        c.close()
        code = run.finish(signal.SIGTERM)
    rows = run.rows()
    end = end_of(rows)
    assert code == 0, (end, run.stderr[-2000:])
    assert [e["reason"] for e in end["results"]] == ["s10_lifted"], end
    assert end["closing"] == "confirmed" and end["model_exit"] == 0 and end["record_incomplete"] is None
    # the S10 stop DRIVE and the mission's DISARM after it, both taken by the model; the DISARM confirmed by a fresh
    # DISARMED STATE of the same boot naming it -- before the closing started (host monotonic ns on both sides)
    written = {(e["kind"], e["seq"]): e for e in events(rows, "frame_written")}
    stops = [q for q in events(rows, "frame_queued") if q["purpose"] == "stop" and ("DRIVE", q["seq"]) in written]
    assert stops, "no S10 stop DRIVE written"
    stop_w = written[("DRIVE", stops[-1]["seq"])]
    boot = stops[-1]["boot"]
    acc = accept_rows(tmp_path / "accept.log")
    a_stop = accepted(acc, "DRIVE", stops[-1]["seq"], boot, stop_w["last_write_ns"])
    assert a_stop is not None and a_stop["verdict"] == "accepted", a_stop
    closing = end["closing_started_ns"]
    disarms = [e for e in events(rows, "frame_written") if e["kind"] == "DISARM"
               and stop_w["written_ns"] <= e["written_ns"] < closing]
    assert disarms, "no mission DISARM before the closing"
    d = disarms[0]
    a_dis = accepted(acc, "DISARM", d["seq"], boot, d["last_write_ns"])
    assert a_dis is not None and a_dis["verdict"] == "accepted"
    conf = [s for s in events(rows, "state") if s["boot"] == boot and s["last_rx_seq"] == d["seq"]
            and s["mode"] == DISARMED and s["rx_ns"] > d["written_ns"]]
    assert conf and conf[0]["rx_ns"] < closing, (conf[:1], closing)


# --- the stop injections (6) -----------------------------------------------------------------------------------------
def _injected(model, tmp_path, name, ms=2000, model_args=(), extra=()):
    run = Runner(model, tmp_path, duration_s=90, model_args=model_args, inject=[f"{name}:{ms}", *extra])
    c = Console(run.port)
    try:
        start_s10(c, answer=False)
        c.wait(lambda s: s["permit"]["state"] in ("LATCHED", "OFF") and s["mission"]["status"] not in (
            "accepted", "preparing", "running"), 60)
        time.sleep(0.5)
    finally:
        c.close()
        code = run.finish(signal.SIGTERM)
    return run, run.rows(), code


def _moving_decided(rows):
    return [q for q in events(rows, "frame_queued") if q["purpose"] == "drive" and q["speed_mps"] != 0.0]


def test_a_cut_send_stops_the_model_on_its_own_command_expiry(mcu_model, tmp_path):
    """The send alone cut while moving: the model's last accepted DRIVE expires at E = echo + min(validity, cap); its
    STOPPING by E + g, DISARMED within the grace, both wheels below 0.01 m/s by DISARMED + 3 tau + g and kept 200 ms."""
    run, rows, code = _injected(mcu_model, tmp_path, "tx")
    inj = events(rows, "inject")[0]
    acc = accept_rows(tmp_path / "accept.log")
    drives = [r for r in acc if r["kind"] == "DRIVE" and r["verdict"] == "accepted" and r["mono_ns"] <= inj["ns"]]
    assert drives, "no DRIVE taken before the cut"
    last = drives[-1]
    assert not [r for r in acc if r["kind"] == "DRIVE" and r["mono_ns"] > inj["ns"] + 2_000_000]  # nothing after
    from forklift_core.mcu_link.messages import LinkDecoder

    w = [e for e in events(rows, "frame_written") if e["kind"] == "DRIVE" and e["seq"] == last["seq"]][-1]
    drive = [r.message for r in LinkDecoder().feed(bytes.fromhex(w["data"]), 0)][0]
    # the firmware's own expiry (drive.c window_for): the echo plus min(validity, command_timeout_ms), as written
    e_us = int(drive.echo_mcu_time_us) + min(int(drive.valid_after_echo_ms), MCU_TIMEOUT_MS) * 1000
    axis = axis_rows(tmp_path / "axis.log")
    after = [a for a in axis if a[0] > last["mcu_us"]]
    stopping = next(a for a in after if a[1] != ARMED)
    assert stopping[0] <= e_us + MODEL_GAP_S * 1e6, (stopping, e_us)
    disarmed = next(a for a in after if a[1] == DISARMED)
    assert disarmed[0] <= stopping[0] + (STOP_GRACE_S + MODEL_GAP_S) * 1e6
    from forklift_core.plant.real_vehicle import load_real_vehicle

    r_m = max(load_real_vehicle(REPO / "config/real/vehicle.yaml").wheel_radius_m)  # the effective rolling radius
    slow = next(a for a in after if a[0] >= disarmed[0] - 1 and max(abs(a[2]), abs(a[3])) * r_m < 0.01)
    assert slow[0] <= disarmed[0] + (3 * WHEEL_TAU_S + MODEL_GAP_S) * 1e6
    kept = [a for a in axis if slow[0] <= a[0] <= slow[0] + 200_000]
    assert all(max(abs(a[2]), abs(a[3])) * r_m < 0.01 for a in kept)


def test_a_state_cut_makes_the_runner_write_a_disarm_the_model_takes_before_the_command_expires(mcu_model, tmp_path):
    """STATE alone cut (the model's --state-silent-*, commands still taken) while moving: the loop's stale STATE
    alarm writes a DISARM; the model's accept log shows it taken with the prior mode ARMED, before the active command's
    expiry (or the authority already revoked first)."""
    run = Runner(mcu_model, tmp_path, duration_s=90, model_args=("--state-silent-at-ms", "16000",
                                                                 "--state-silent-ms", "1500"))
    c = Console(run.port)
    try:
        start_s10(c, answer=False)
        time.sleep(18)
    finally:
        c.close()
        run.finish(signal.SIGTERM)
    rows = run.rows()
    acc = accept_rows(tmp_path / "accept.log")
    cut = [r for r in acc if 16_000_000 <= r["mcu_us"] < 17_500_000]
    moving_before = [r for r in acc if r["kind"] == "DRIVE" and r["verdict"] == "accepted" and
                     15_000_000 <= r["mcu_us"] < 16_000_000]
    if not moving_before:
        pytest.fail("inconclusive: no DRIVE taken in the second before the cut (the mission timing moved)")
    dis = [r for r in cut if r["kind"] == "DISARM"]
    assert dis, "no DISARM taken in the cut"
    d = dis[0]
    reason = end_of(rows)["permit_reason"]
    assert d["prior_mode"] == ARMED or "authorization_revoked" in str(reason), (d, reason)
    if d["prior_mode"] == ARMED:
        assert d["prior_expiry_us"] >= d["mcu_us"], d  # taken before the active command ran out


@pytest.mark.parametrize("name", ["scan", "slam"])
def test_a_stopped_source_stops_motion_by_its_expiry(mcu_model, tmp_path, name):
    """The layer's scans or the SLAM replies stop while moving: with F the expiry the gate judges by (scan: the last
    placed scan's + period + budget; SLAM: the last processed reply's first beam + 0.5 s), no moving DRIVE decided
    after F, every moving DRIVE's last judged instant <= F - A, and the run latches with a DISARM written."""
    run, rows, code = _injected(mcu_model, tmp_path, name)
    head = rows[0]
    inj = events(rows, "inject")[0]
    if name == "scan":
        last_s = max(inj["last_scan_s"].values())
        f_ns = head["start_ns"] + int((last_s + 0.1 + 0.1) * 1e9)
    else:
        assert inj["slam_fresh_ns"] is not None
        f_ns = inj["slam_fresh_ns"] + int(0.5 * 1e9)
    written = {(e["kind"], e["seq"]): e for e in events(rows, "frame_written")}
    for q in _moving_decided(rows):
        assert q["decided_ns"] < f_ns, (q, f_ns)
        w = written.get(("DRIVE", q["seq"]))
        if w is not None:
            assert w["judged_ns"] is not None and w["judged_ns"] <= f_ns - A_NS, (w, f_ns)
            assert w["judged_ns"] <= q["decided_ns"] + W_NS - A_NS
    end = end_of(rows)
    assert end["permit"] == "LATCHED" and end["permit_reason"], end
    assert [e for e in events(rows, "frame_written") if e["kind"] == "DISARM" and e["written_ns"] > inj["ns"]]


def test_a_half_drive_is_abandoned_with_the_poison_then_a_disarm(mcu_model, tmp_path):
    """10.8's general property: the first moving DRIVE after one the model took is left half written past its own
    write bound and released -- it is discarded by either path (a refused continuation or the stuck-write cut), the
    model never takes it, and a DISARM follows the abandonment."""
    run, rows, code = _injected(mcu_model, tmp_path, "tx_partial", ms=1000, extra=["tx_release:1400"])
    parts = events(rows, "inject_partial")
    assert parts, "inconclusive: no half DRIVE made"
    seq, at = parts[0]["seq"], parts[0]["ns"]
    boot = events(rows, "frame_queued")[-1]["boot"]
    acc = accept_rows(tmp_path / "accept.log")
    assert accepted(acc, "DRIVE", seq, boot, at) is None  # never completed
    ticks = [t for t in events(rows, "tick") if t["ns"] >= at - 40_000_000]
    discard = next((t for t in ticks if any(n.startswith("continuation_refused:") for n in t["notes"])
                    or "link_stuck" in str(t["reason"])), None)
    assert discard is not None, "the half frame was never discarded"
    dis = [e for e in events(rows, "frame_written") if e["kind"] == "DISARM" and e["written_ns"] >= discard["ns"]]
    assert dis and accepted(acc, "DISARM", dis[0]["seq"], boot, dis[0]["last_write_ns"]) is not None


def test_a_closing_disarm_that_never_reaches_the_model_is_unconfirmed(mcu_model, tmp_path):
    """The send cut through the closing (tx_keep): the model's own expiry still brings DISARMED STATEs, but none names
    a closing DISARM -- disarm_unconfirmed, a failed run."""
    run, rows, code = _injected(mcu_model, tmp_path, "tx_keep", ms=500)
    end = end_of(rows)
    assert end["closing"] == "disarm_unconfirmed" and code != 0
    assert events(rows, "state")[-1]["mode"] == DISARMED


# --- the model's lifetime and a killed model (design 1, 10.3) ------------------------------------------------------
def test_a_killed_runner_leaves_a_model_that_ends_itself(mcu_model, tmp_path):
    run = Runner(mcu_model, tmp_path, duration_s=3)
    pid = None
    for _ in range(100):
        if (tmp_path / "run.jsonl").exists() and (tmp_path / "run.jsonl").read_text():
            pid = json.loads((tmp_path / "run.jsonl").read_text().splitlines()[0])["model_pid"]
            break
        time.sleep(0.05)
    assert pid
    started = time.monotonic()
    run.kill()  # SIGKILL: no cleanup runs
    while time.monotonic() - started < 3 + 11 + 2:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            break
        try:
            if os.waitpid(pid, os.WNOHANG)[0] == pid:
                break
        except ChildProcessError:
            pass
        with open(f"/proc/{pid}/stat") as f:
            if f.read().split()[2] == "Z":
                break
        time.sleep(0.2)
    else:
        pytest.fail("the model outlived its own end")


def test_a_model_killed_mid_run_ends_the_run_as_a_device_loss(mcu_model, tmp_path):
    run = Runner(mcu_model, tmp_path, duration_s=30)
    c = Console(run.port)
    try:
        time.sleep(1.0)
        pid = run.rows()[0]["model_pid"]
        os.kill(pid, signal.SIGKILL)
        code = run.finish(timeout=30)
    finally:
        c.close()
    end = end_of(run.rows())
    assert code != 0
    assert "model_exit" in str(end["reason"]) or "device_lost" in str(end["reason"]), end
    assert end["model_exit"] == -9 and "model_exit=-9" in end.get("device_lost", "model_exit=-9")
