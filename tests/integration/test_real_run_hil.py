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
        self.proc.wait(10)
        for pipe in (self.proc.stdout, self.proc.stderr):  # the model still holds their other ends until it ends
            pipe.close()


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
            self.reader.cancel()
            try:
                await self.reader
            except BaseException:  # noqa: BLE001 -- cancelled
                pass
        try:
            asyncio.run_coroutine_threadsafe(_close(), self.loop).result(5)
        finally:
            self.loop.call_soon_threadsafe(self.loop.stop)
            self.thread.join(5)
            self.loop.close()  # its self-pipe sockets too (-W error: ResourceWarning)


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
    """Every row of a complete accept log (a malformed row makes the judgement impossible, design 10.7)."""
    out = []
    for line in path.read_text().splitlines():
        p = line.split()
        assert len(p) == 8, f"inconclusive: malformed accept-log row {line!r}"
        out.append({"mcu_us": int(p[0]), "seq": int(p[1]), "kind": p[2], "verdict": p[3], "prior_mode": int(p[4]),
                    "prior_expiry_us": int(p[5]), "boot": int(p[6]), "mono_ns": int(p[7])})
    return out


def axis_rows(path: Path):
    """(MCU us, wall gap ns, mode, lift m, steering rad, wheel L, R rad/s) per model tick."""
    out = []
    for line in path.read_text().splitlines():
        p = line.split()
        assert len(p) == 7, f"inconclusive: malformed axis-log row {line!r}"
        out.append((int(p[0]), int(p[1]), int(p[2]), float(p[3]), float(p[4]), float(p[5]), float(p[6])))
    return out


def accepted(rows, kind, seq, boot, after_ns, before_ns=None):
    """The model's acceptance of the frame whose last write entered at ``after_ns``: the one row of that boot, kind
    and seq at or after it (and before ``before_ns``, the next write of that seq if any; design 10.6). Two
    candidates are ambiguous: inconclusive."""
    hits = [r for r in rows if r["kind"] == kind and r["seq"] == seq and r["boot"] == boot
            and r["mono_ns"] >= after_ns and (before_ns is None or r["mono_ns"] < before_ns)]
    assert len(hits) <= 1, f"inconclusive: {len(hits)} acceptances match seq {seq}"
    return hits[0] if hits else None


def model_complete(end):
    """The model's logs are complete only when it ended 0 (it exits 4 on any log write failure, design 10.8)."""
    assert end["model_exit"] == 0, f"inconclusive: model exit {end['model_exit']}"


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
        c.wait(lambda s: s["permit"]["state"] == "LATCHED", 60)
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
    model_complete(end)
    # the stop DRIVE the permit confirmed (control_stop_confirmed with its seq) and the DISARM s10_stopped asked
    perm = events(rows, "permit")
    confirmed = [p for p in perm if p["event"] == "control_stop_confirmed"]
    assert len(confirmed) == 1, perm[-6:]
    stop_seq = confirmed[0]["detail"]["seq"]
    asked = [p for p in perm if p["event"] == "disarm" and p["reason"] == "s10_stopped"]
    assert len(asked) == 1 and asked[0]["ns"] >= confirmed[0]["ns"], perm[-6:]
    written = {(e["kind"], e["seq"]): e for e in events(rows, "frame_written")}
    stop_w = written[("DRIVE", stop_seq)]
    boot = next(q["boot"] for q in events(rows, "frame_queued") if q["seq"] == stop_seq)
    acc = accept_rows(tmp_path / "accept.log")
    a_stop = accepted(acc, "DRIVE", stop_seq, boot, stop_w["last_write_ns"])
    assert a_stop is not None and a_stop["verdict"] == "accepted", a_stop
    closing = end["closing_started_ns"]
    disarms = [e for e in events(rows, "frame_written") if e["kind"] == "DISARM" and e["written_ns"] >= asked[0]["ns"]]
    assert disarms and disarms[0]["written_ns"] < closing, "the mission's DISARM not written before the closing"
    d = disarms[0]
    a_dis = accepted(acc, "DISARM", d["seq"], boot, d["last_write_ns"])
    assert a_dis is not None and a_dis["verdict"] == "accepted"
    conf = [s for s in events(rows, "state") if s["boot"] == boot and s["last_rx_seq"] == d["seq"]
            and s["mode"] == DISARMED and s["rx_ns"] > d["written_ns"]]
    assert conf and conf[0]["rx_ns"] < closing, (conf[:1], closing)
    _holding_spans_stay_within_the_bound(rows, tmp_path, end)


# design 2 (가): the worst displacement while a position-holding span is watched -- threshold + the axis's top speed
# x (sample age bound 70 ms + the longest watch interval recorded + link/write 10 ms + the model tick gap 5 ms) + the
# top speed x stop_grace (the firmware keeps the axis under control until DISARMED). Top speeds: plant.h full duty.
LIFT_TOP_MPS, STEER_TOP_RADPS = 0.1, 1.0  # PLANT_LIFT_M_S_PER_DUTY, PLANT_STEER_RAD_S_PER_DUTY at duty 1
WAIT_LIFT_RISE_M, WAIT_STEER_MOVE_RAD = 0.005, 0.02


def _holding_spans_stay_within_the_bound(rows, out, end):
    """Every wait span of the run (contiguous wait ticks) on the model's own axis samples, through DISARMED when it
    ended in one: the lift's rise and the steering's move stay within (가)'s bound."""
    watch = [w[0] for w in end["axis_watch_times"]]
    gap_s = max((b - a for a, b in zip(watch, watch[1:])), default=20_000_000) / 1e9
    delay = 0.070 + gap_s + 0.010 + 0.005
    lift_bound = WAIT_LIFT_RISE_M + LIFT_TOP_MPS * (delay + 0.1)
    steer_bound = WAIT_STEER_MOVE_RAD + STEER_TOP_RADPS * (delay + 0.1)
    states = events(rows, "state")
    off = sorted(s["rx_ns"] - s["mcu_us"] * 1000 for s in states)
    offset = off[len(off) // 2]  # host ns - MCU ns (one boot); receipt jitter only widens the window
    axis = axis_rows(out / "axis.log")
    ticks = [t for t in events(rows, "tick") if "rear" in t]
    spans, cur = [], None
    for t in ticks:
        if t["branch"] == "wait":
            cur = [t["ns"], t["ns"]] if cur is None else [cur[0], t["ns"]]
        elif cur is not None:
            spans.append(cur)
            cur = None
    assert spans, "no holding span in the run"
    for a, b in spans:
        lo, hi = (a - offset) // 1000, (b - offset) // 1000 + 200_000  # 200 ms past the span: the stop's tail
        seg = [x for x in axis if lo <= x[0] <= hi]
        if not seg:
            continue
        lift0, steer0 = seg[0][3], seg[0][4]
        assert max(x[3] - lift0 for x in seg) <= lift_bound, (a, b, lift_bound)
        assert max(abs(x[4] - steer0) for x in seg) <= steer_bound, (a, b, steer_bound)


# --- the stop injections (6) -----------------------------------------------------------------------------------------
def _injected(model, tmp_path, name, ms=2000, model_args=(), extra=()):
    run = Runner(model, tmp_path, duration_s=90, model_args=model_args, inject=[f"{name}:{ms}", *extra])
    c = Console(run.port)
    try:
        start_s10(c, answer=False)
        c.wait(lambda s: s["permit"]["state"] == "DRIVING", 90)  # moving: the injection counts from here
        c.wait(lambda s: s["permit"]["state"] == "LATCHED", 60)
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
    model_complete(end_of(rows))
    axis = axis_rows(tmp_path / "axis.log")
    after = [a for a in axis if a[0] > last["mcu_us"]]
    first_out = next(a for a in after if a[2] != ARMED)
    assert first_out[2] == STOPPING, first_out  # the expiry's own path: STOPPING first, never DISARMED at once
    assert first_out[0] <= e_us + MODEL_GAP_S * 1e6, (first_out, e_us)
    disarmed = next(a for a in after if a[2] == DISARMED)
    assert disarmed[0] <= first_out[0] + (STOP_GRACE_S + MODEL_GAP_S) * 1e6
    window = [a for a in axis if e_us - 10_000 <= a[0] <= disarmed[0] + 400_000]
    if max(a[1] for a in window) > MODEL_GAP_S * 1e9:
        pytest.fail("observed_only:model_tick_gap (the model ticked late in the judged window: run again)")
    from forklift_core.plant.real_vehicle import load_real_vehicle

    r_m = max(load_real_vehicle(REPO / "config/real/vehicle.yaml").wheel_radius_m)  # the effective rolling radius
    slow = next(a for a in after if a[0] >= disarmed[0] - 1 and max(abs(a[5]), abs(a[6])) * r_m < 0.01)
    assert slow[0] <= disarmed[0] + (3 * WHEEL_TAU_S + MODEL_GAP_S) * 1e6
    assert axis[-1][0] >= slow[0] + 200_000, "inconclusive: the log ends before the 200 ms hold"
    kept = [a for a in axis if slow[0] <= a[0] <= slow[0] + 200_000]
    assert all(max(abs(a[5]), abs(a[6])) * r_m < 0.01 for a in kept)


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
    model_complete(end_of(rows))
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
    """The layer's scans or the SLAM replies stop while moving. With F the expiry the gate judges by (scan: the last
    scan the layer received, + period + budget, the watchdog's own; SLAM: the last processed reply's first beam
    + 0.5 s): no normal DRIVE (wheels, lift or steering alike) decided after F; every one written judged <= F - A and
    <= its decision + W - A; the latch names this source; the DISARM is written in time (SLAM: by F + two ticks; scan:
    by the first overdue evaluation + overdue_end + two ticks); the STATEs stay fresh to the latch; logs complete."""
    run, rows, code = _injected(mcu_model, tmp_path, name)
    end = end_of(rows)
    model_complete(end)
    head = rows[0]
    inj = events(rows, "inject")[0]
    if name == "scan":
        assert inj["scan_delivered_s"] is not None
        f_ns = head["start_ns"] + int((inj["scan_delivered_s"] + 0.1 + 0.1) * 1e9)
    else:
        assert inj["slam_fresh_ns"] is not None
        f_ns = inj["slam_fresh_ns"] + int(0.5 * 1e9)
    written = {(e["kind"], e["seq"]): e for e in events(rows, "frame_written")}
    for q in events(rows, "frame_queued"):
        if q["purpose"] != "drive":
            continue
        assert q["decided_ns"] < f_ns, (q, f_ns)
        w = written.get(("DRIVE", q["seq"]))
        if w is not None:
            assert w["judged_ns"] is not None and w["judged_ns"] <= f_ns - A_NS, (w, f_ns)
            assert w["judged_ns"] <= q["decided_ns"] + W_NS - A_NS
    latch = [p for p in events(rows, "permit") if p["event"] in ("disarm", "latched") and p["ns"] >= inj["ns"]
             and p["reason"] != "signal"]
    assert latch, events(rows, "permit")[-5:]
    reason = str(latch[0]["reason"])
    if name == "scan":
        assert "layer_latency_exceeded" in reason or "scan" in reason, reason
        overdue = [t["ns"] for t in events(rows, "tick") if "layer_overdue" in str(t["reason"]) and t["ns"] >= inj["ns"]]
        overdue += [g["ns"] for g in events(rows, "gate") if g["ns"] and "layer_overdue" in str(g["verdict"])
                    and g["ns"] >= inj["ns"]]
        assert overdue, "no overdue evaluation recorded"
        deadline = min(overdue) + 1_000_000_000 + 2 * 20_000_000 + 20_000_000
    else:
        # the readiness's slam:stale, or the tracker's own LocalizationStale in the control tick -- whichever reads
        # the expired correction first
        assert "slam" in reason or "correction" in reason, reason
        deadline = f_ns + 2 * 20_000_000 + 20_000_000
    dis = [e for e in events(rows, "frame_written") if e["kind"] == "DISARM" and e["written_ns"] >= inj["ns"]]
    assert dis and dis[0]["written_ns"] <= deadline, (dis[:1], deadline)
    before = [s_ for s_ in events(rows, "state") if s_["rx_ns"] <= latch[0]["ns"]]
    assert before and latch[0]["ns"] - before[-1]["rx_ns"] <= 50_000_000  # STATEs fresh up to the latch


def test_a_half_drive_is_abandoned_with_the_poison_then_a_disarm(mcu_model, tmp_path):
    """10.8's general property: the first moving DRIVE after one the model took is left half written (a positive
    first write) past its own write bound and released -- the transport abandons that very frame, finishes the
    poison, then a DISARM is written; the model (its log complete) never takes that DRIVE."""
    run, rows, code = _injected(mcu_model, tmp_path, "tx_partial", ms=1000, extra=["tx_release:1400"])
    model_complete(end_of(rows))
    parts = events(rows, "inject_partial")
    assert parts and parts[0]["bytes"] > 0, "inconclusive: no half DRIVE made"
    seq, at = parts[0]["seq"], parts[0]["ns"]
    tx = [e for e in rows if e.get("t", "").startswith("tx_") and e.get("ns", 0) >= at - 1]
    half = next(e for e in tx if e["t"] == "tx_partial" and e["seq"] == seq)
    gone = next((e for e in tx if e["t"] == "tx_abandoned" and e["seq"] == seq), None)
    assert gone is not None and gone["ns"] >= half["ns"], "that half frame was never abandoned"
    poison = next((e for e in tx if e["t"] == "tx_poison_done" and e["ns"] >= gone["ns"]), None)
    assert poison is not None, "the poison never finished after the abandonment"
    dis = [e for e in events(rows, "frame_written") if e["kind"] == "DISARM" and e["written_ns"] >= poison["ns"]]
    assert dis, "no DISARM after the poison"
    boot = events(rows, "frame_queued")[-1]["boot"]
    acc = accept_rows(tmp_path / "accept.log")
    assert accepted(acc, "DRIVE", seq, boot, at) is None  # never completed
    assert accepted(acc, "DISARM", dis[0]["seq"], boot, dis[0]["last_write_ns"]) is not None


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
