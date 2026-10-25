"""The ``pick`` answers to ``pick_request`` (CT-2 D3/D5, docs/plans/2026-10-24-ct2-next-pick.md).

The arbiter acks the request; ``Link`` then hands it here. ``submit`` takes the snapshot at that tick and queues it
on the planning worker (a ``PlanWorker``, its own process: the control thread never waits on it) or, without one,
computes it at once (the tests, a stand-in back end). ``poll`` gives the finished answers as ``pick`` messages, a
failed computation as ``reason: "error:<type>"`` so a client is never left to its deadline by a crash. Every ack -- a
resend that got the stored one too -- asks again: the answer is the snapshot of that moment.
"""

from __future__ import annotations

from typing import Callable

# The vehicle's bound on request -> pick (welcome capabilities.next_pick_deadline_ms; the console waits this long).
# A setting, to be set again from the live round trips measured in V3.
PICK_DEADLINE_MS = 5000


class PickService:
    def __init__(self, snapshot: Callable[[], object], *, worker=None, pose: Callable[[], dict | None] | None = None):
        self.snapshot, self.worker, self.pose = snapshot, worker, pose
        self._jobs: list = []  # (job id or None, conn_id, client_id, request_id, revision, pose, answer or None)
        self.log: list[dict] = []

    def submit(self, conn_id: str, client_id: str, request_id: str, now: float) -> None:
        pose = self.pose() if self.pose is not None else None
        try:
            snap = self.snapshot()
        except Exception as exc:  # noqa: BLE001 -- a snapshot that cannot be built is the answer's error
            self._jobs.append((None, conn_id, client_id, request_id, None, pose, (False, f"{type(exc).__name__}: {exc}")))
            return
        revision = snap.targets.revision
        if self.worker is None:
            from forklift_core.mission.next_pick import recommend

            try:
                answer = (True, {"result": recommend(snap)})
            except Exception as exc:  # noqa: BLE001
                answer = (False, f"{type(exc).__name__}: {exc}")
            self._jobs.append((None, conn_id, client_id, request_id, revision, pose, answer))
            return
        job = self.worker.submit("next_pick", {"snapshot": snap})
        self._jobs.append((job, conn_id, client_id, request_id, revision, pose, None))
        self.log.append({"mono": now, "pick_submit": request_id, "job": job})

    def poll(self, now: float) -> list:
        """[(conn_id, client_id, message)] for every answer finished since the last poll."""
        out, keep = [], []
        for job, conn_id, client_id, request_id, revision, pose, answer in self._jobs:
            if answer is None:
                answer = self.worker.poll(job)
                if answer is None:
                    keep.append((job, conn_id, client_id, request_id, revision, pose, None))
                    continue
            ok, value = answer
            msg = {"t": "pick", "request_id": request_id, "targets_revision": revision, "pose": pose}
            if ok:
                msg.update(value["result"])
            else:
                kind = str(value).split(":", 1)[0].strip() or "Exception"
                msg.update({"pick": None, "reason": f"error:{kind}", "plan": [], "missions": 0,
                            "estimate_total_m": 0.0, "home_assumed": False, "blocked": {}})
            self.log.append({"mono": now, "pick_answer": request_id, "reason": msg["reason"],
                             "wall_s": value.get("wall_s") if ok and isinstance(value, dict) else None})
            out.append((conn_id, client_id, msg))
        self._jobs = keep
        return out
