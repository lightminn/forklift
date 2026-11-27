"""The bench console's channels and cycle (docs/plans/2026-11-27-bench-console.md 3.1, B1).

``BenchChannels`` is ``RealChannels`` with the bench's stop-like requests -- ``bench_stop``, ``bench_disarm`` and
``lease_release`` -- raising the emergency serial the moment they arrive, under the channels' lock, as a STOP, a
DISARM and a connection change already do. A write judged before such a request then fails its send boundary, and
the next cycle handles the request before anything else is written. The requests themselves still go through the
ordered channel and the arbiter's ledger (one answer per request).

``BenchLink`` is the cycle: the emergency batch, the arbiter's expiries and its ledger purge, then the requests and
the latest values (``echo`` renews the lease). It never calls the vehicle's ``observe`` or ``mission_context`` -- their
stop detector would stop the session after 10 s without wheel feedback (``not_stopped``) -- nor ``step``. ``tick``
puts the arbiter's tick out every ``tick_period_s``."""

from __future__ import annotations

from forklift_core.live.link import Channels
from forklift_core.live.real_channels import RealChannels, RealLink
from forklift_core.safety.validate import problem

EMERGENCY_REQUESTS = ("bench_stop", "bench_disarm", "lease_release")


class BenchChannels(RealChannels):
    def message(self, conn_id: str, client_id: str, msg: dict) -> dict | None:
        if problem(msg) is None and msg.get("t") in EMERGENCY_REQUESTS:
            with self._lock:  # re-entrant: the ordered put below takes it again
                self.emergency_serial += 1
                return Channels.message(self, conn_id, client_id, msg)
        return super().message(conn_id, client_id, msg)


class BenchLink(RealLink):
    def begin(self, now: float, feedback=None) -> int:  # noqa: ARG002 -- the bench has no vehicle feedback
        """The cycle's requests; returns the emergency serial of the batch handled -- the writes' reference (a stop-like
        request that came after it refuses this cycle's writes)."""
        serial = self.emergency(now)
        self.a._expire(now)
        self.a._cycle_mono = now
        self.a.ledger.purge(now)
        conns, stops, ordered, latest = self.ch.take_cycle()
        for kind, conn_id, client_id, extra in conns:
            self._conn_event(kind, conn_id, client_id, extra, now, "cycle")
        for conn_id, client_id, msg in stops:
            self._handle(conn_id, client_id, msg, now, "cycle")
        for _, conn_id, client_id, msg in ordered:
            self._handle(conn_id, client_id, msg, now, "cycle")
        for conn_id, client_id, msg in latest:
            self._handle(conn_id, client_id, msg, now, "cycle")
        return serial

    def tick(self, now: float) -> bool:
        """The arbiter's tick when due (the lease is kept by its echo); True when one went out."""
        if self._next_tick is None or now >= self._next_tick:
            self.ch.put(None, self.a.tick(now))
            self._next_tick = now + self.tick_period_s
            return True
        return False


__all__ = ["BenchChannels", "BenchLink", "EMERGENCY_REQUESTS"]
