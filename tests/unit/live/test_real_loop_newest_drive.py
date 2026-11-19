"""A loop DRIVE supersedes every unsent older one (Codex PR 5 design round 6 P1-1, a 4c gap): a frame held before its
first byte for a tick is never written after the next tick decided to stop."""

from forklift_core.live import real_autonomy as ra
from forklift_core.mcu_link.messages import Drive

from real_loop_rig import Rig


def run_until(r, cond, limit=4000):
    for _ in range(limit):
        rec = r.tick()
        if cond(rec):
            return rec
    raise AssertionError(f"not reached: {r.loop.records[-1]}")


def test_a_held_moving_drive_is_not_written_after_the_next_tick_decides_to_stop():
    r = Rig(frames=True)
    r.start()
    r.submit()
    run_until(r, lambda rec: r.permit.state == ra.DRIVING and r.loop.book.confirmed is not None
              and r.loop.book.confirmed.speed_mps != 0.0)
    real = r.transport.transmit
    r.transport.transmit = lambda host, now, gate: []  # this tick's moving DRIVE stays queued, unsent
    r.tick()
    queued = [o.seq for o in r.host._outbox if o.kind == "DRIVE"]  # noqa: SLF001
    assert queued
    r.transport.transmit = real
    real_assemble = r.core.assemble_command

    def zero(*a, **kw):  # the next tick's normal AUTO output is a stop (a D5 settle gate's zero, say)
        out = real_assemble(*a, **kw)
        out.drive = r.loop.kin.command(0.0, float(out.drive.curvature_inv_m))
        return out

    r.core.assemble_command = zero
    k = len(r.transport.sent)
    r.tick()
    written = [(seq, m) for _, kind, seq, m in r.transport.sent[k:] if kind == "DRIVE"]
    assert not [seq for seq, _ in written if seq in queued]  # the older unsent frame never goes out
    assert not [m for _, m in written if m.left_wheel_rad_s or m.right_wheel_rad_s]
    assert not [f for f in r.loop.book.frames if f.seq in queued]  # and the record no longer counts it
