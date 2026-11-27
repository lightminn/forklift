"""A scripted console back end for the bench screen's browser replay (plan 2026-11-27 4.1 (a), (a'')). Validation only.

Modes (the welcome and the stream it sends every 100 ms with a tick):
  bench-mute  welcome session bench + bench_state (ready, the lease holder); bench requests are recorded, never answered
  real        welcome backend real + real_status
  hil         welcome backend real (HIL) + real_status
  isaac       welcome backend isaac + state
Every message the page sends is appended to --log as JSONL. --welcome-delay-s holds the welcome after the hello;
--close-after-s closes the control socket after that long (the page then reconnects); --epoch fixes the session epoch.
"""

import argparse
import asyncio
import json
import time

from websockets.asyncio.server import serve


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", required=True, choices=["bench-mute", "real", "hil", "isaac"])
    ap.add_argument("--port", type=int, required=True)
    ap.add_argument("--log", required=True)
    ap.add_argument("--epoch", type=int, default=4242)
    ap.add_argument("--welcome-delay-s", type=float, default=0.0)
    ap.add_argument("--close-after-s", type=float, default=None)
    ap.add_argument("--duration-s", type=float, default=30.0)
    a = ap.parse_args()
    log = open(a.log, "a", encoding="utf-8")
    state = {"lease_holder": None, "control_gen": 3, "seq": 0}

    def record(conn, msg):
        log.write(json.dumps({"at": time.time(), "mode": a.mode, "epoch": a.epoch, "conn": conn, "msg": msg}) + "\n")
        log.flush()

    def welcome():
        w = {"t": "welcome", "api": "console/1", "session_epoch": a.epoch, "vehicle_id": f"scripted-{a.mode}",
             "request_token": 1000 + a.epoch % 1000, "capabilities": {"control": True, "mission": a.mode == "isaac"}}
        if a.mode == "bench-mute":
            w.update(session="bench", backend="real",
                     product="forklift-mcu commission OPEN scripted rec=0 lim=lr1 bench5")
        else:
            w["backend"] = "isaac" if a.mode == "isaac" else "real"
        return w

    def stream():
        state["seq"] += 1
        holder = state["lease_holder"]
        if a.mode == "bench-mute":
            return {"t": "bench_state", "session_epoch": a.epoch, "mode": "PAUSED", "control_gen": state["control_gen"],
                    "stops": 0, "lease": None if holder is None else {"client_id": holder, "remaining_s": 3.0},
                    "product": "forklift-mcu commission OPEN scripted rec=0 lim=lr1 bench5", "ready": True,
                    "not_ready": None, "reopen": "go", "board": {"mode": "DISARMED", "boot_id": 1, "arm_gen": 0,
                    "duty_e4": [0, 0, 0, 0], "current_ma": [0, 0, 0, 0], "driver_status": ["C0"] * 4,
                    "fault_flags": [], "reject_reason": "NONE", "steering_rad": 0.0, "lift_m": 0.0, "invalid": [],
                    "wheel_count": [0, 0]}, "bench": None, "run": None, "clear": None, "last_result": None}
        if a.mode in ("real", "hil"):
            return {"t": "real_status", "session_epoch": a.epoch, "control_gen": state["control_gen"],
                    "lease": None if holder is None else {"holder": holder}, "bypassed": []}
        return {"t": "state", "session_epoch": a.epoch, "state_seq": state["seq"], "control_gen": state["control_gen"],
                "arm_gen": 0, "armed": False, "mode": "PAUSED",
                "lease": None if holder is None else {"holder": holder}}

    async def control(ws):
        first = json.loads(await ws.recv())
        record("control", first)
        client = first.get("client_id")
        if a.welcome_delay_s:
            await asyncio.sleep(a.welcome_delay_s)
        await ws.send(json.dumps(welcome()))
        t0 = time.monotonic()

        async def pump():
            k = 0
            while True:
                k += 1
                await ws.send(json.dumps({"t": "tick", "session_epoch": a.epoch, "state_seq": k, "token": f"k{k}"}))
                await ws.send(json.dumps(stream()))
                await asyncio.sleep(0.1)
                if a.close_after_s is not None and time.monotonic() - t0 > a.close_after_s:
                    await ws.close()
                    return

        task = asyncio.create_task(pump())
        try:
            async for raw in ws:
                msg = json.loads(raw)
                record("control", msg)
                t = msg.get("t")
                rid = msg.get("request_id")
                if t == "lease_request":
                    state["lease_holder"] = client
                    await ws.send(json.dumps({"t": "ack", "of": t, "request_id": rid, "lease_id": "LS1",
                                              "lease_timeout_s": 3.0, "control_gen": state["control_gen"]}))
                elif t in ("bench_run", "bench_stop", "bench_disarm", "clear_faults") and a.mode == "bench-mute":
                    pass  # recorded, never answered (the ACK lost)
                elif rid:
                    await ws.send(json.dumps({"t": "ack", "of": t, "request_id": rid}))
        finally:
            task.cancel()

    async def media(ws):
        async for raw in ws:
            record("media", json.loads(raw))

    async def handler(ws):
        path = ws.request.path.split("?", 1)[0]
        if path == "/api/v1/control":
            await control(ws)
        elif path == "/api/v1/media":
            await media(ws)

    async def run():
        async with serve(handler, "127.0.0.1", a.port):
            await asyncio.sleep(a.duration_s)

    asyncio.run(run())


if __name__ == "__main__":
    main()
