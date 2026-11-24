"""PR 6a-2b (docs/design/2026-11-20-real-hil-6a.md 10): the loop's run hooks -- the scan feed after the tick's STATE,
a source's fatal error, closing (DISARM only, its own confirmation), the event sink."""

from forklift_core.live import real_autonomy as ra

from real_loop_rig import Rig


def run_until(r, cond, limit=4000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def driving():
    r = Rig(frames=True)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and r.loop.book.confirmed is not None
              and r.loop.book.confirmed.speed_mps != 0.0)
    return r


def test_the_scan_feed_runs_after_the_ticks_state():
    r = driving()
    seen = []
    r.loop.scan_feed = lambda now_ns: seen.append((now_ns, r.host.state.mcu_time_us, r.loop.samples[-1].host_ns))
    before = r.mcu.t_us
    r.tick()
    assert seen and seen[-1][1] > before and seen[-1][2] == seen[-1][0]  # this tick's STATE already taken


def test_a_failing_scan_feed_is_a_fatal_source_and_the_ticks_emergency():
    r = driving()

    def boom(now_ns):
        raise RuntimeError("lidar gone")

    r.loop.scan_feed = boom
    rec = r.tick()
    assert r.loop.fatal_source.startswith("scan:RuntimeError") and rec.branch == "emergency"
    n = len(r.loop.records)
    r.tick()
    assert r.loop.records[n].branch == "emergency"  # stays: no scan feed is called again


def test_closing_writes_only_the_disarm_and_confirms_it_by_its_seq():
    r = driving()
    r.loop.begin_closing(r.now)
    k = len(r.transport.sent)
    for _ in range(20):
        r.tick()
        if r.loop.closing_confirmed_ns is not None:
            break
    kinds = {kind for _, kind, _, _ in r.transport.sent[k:]}
    assert kinds <= {"DISARM"}
    assert r.loop.closing_disarms and r.loop.closing_confirmed_ns is not None


def test_closing_is_not_confirmed_by_disarmed_states_when_its_disarm_never_arrived():
    r = driving()
    r.say("disarm")  # the MCU DISARMED before the run ends
    run_until(r, lambda rec: r.mcu.mode.name == "DISARMED", limit=50)
    r.transport.eagain = {"DISARM"}  # the closing DISARM never gets out
    r.loop.begin_closing(r.now)
    for _ in range(50):
        r.tick()
    assert r.loop.closing_confirmed_ns is None and not r.loop.closing_disarms


def test_the_event_sink_sees_queued_and_written_frames():
    r = driving()
    events = []
    r.loop.events = events.append
    r.tick(3)
    kinds = {e["t"] for e in events}
    assert {"frame_queued", "frame_written"} <= kinds
    q = next(e for e in events if e["t"] == "frame_queued")
    assert {"purpose", "decided_ns", "expires_ns", "echo_mcu_time_us"} <= set(q)
