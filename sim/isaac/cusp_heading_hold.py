"""Records of the observe cusp heading hold (no Isaac imports).

docs/plans/2026-10-13-epal6-fork-spacing.md, fix (A). The observe tracker holds a cusp
reached inside its window but missed in heading by 0.05-0.08 rad (``cusp_pending``); the
runner judges the next leg once standing and releases the cusp or fails the hold. This
keeps one record per hold and watches a released leg to its end, in the run state so the
records land in the result:

- ``state["cusp_heading_hold"]``: the hold open now (None between holds);
- ``state["cusp_heading_holds"]``: every decided hold, in order, each with its decision and,
  once released, ``next_leg`` (how the released leg ended);
- ``state["cusp_heading_watch"]``: the released leg being watched (None otherwise).

A leg released and then held again at its own end closes its watch as ``cusp_pending``
before the new hold opens, so no record loses its ``next_leg`` (Codex fix A implementation
review 1 P2: a second release overwrote the first watch).
"""

from __future__ import annotations

DECISIONS = ("realign_held", "window", "judged_out", "released", "release_refused")
ENDS = ("accepted", "cusp_pending", "failed", "replaced")


class CuspHeadingHolds:
    def __init__(self, state: dict) -> None:
        self.state = state
        state.setdefault("cusp_heading_hold", None)
        state.setdefault("cusp_heading_holds", [])
        state.setdefault("cusp_heading_watch", None)

    def held(self, now_s: float, key, info: dict, end: dict) -> dict:
        """The hold open for ``key`` ([tracker id, leg index]), opened now with ``info`` (pose,
        errors, motion) when it is new; ``end`` (the tracking result now) closes a watch still open."""
        hold = self.state["cusp_heading_hold"]
        if hold is None or hold["key"] != list(key):
            watch = self.state["cusp_heading_watch"]
            if watch is not None:  # the runner's tick() normally closed it already
                same = watch["tracker"] == list(key)[0]
                self.close_watch(now_s, "cusp_pending" if same else "replaced", end)
            hold = {"key": list(key), "since_s": now_s, **info}
            self.state["cusp_heading_hold"] = hold
        return hold

    def decide(self, now_s: float, decision: str, info: dict, end: dict) -> None:
        """Close the open hold with ``decision``; a release starts watching the next leg (its
        tracker id and leg index are the hold key's id and the released leg's index)."""
        if decision not in DECISIONS:
            raise ValueError(f"unknown decision {decision!r}")
        hold = self.state["cusp_heading_hold"]
        if hold is None:
            raise ValueError("no hold is open")
        hold.update(decided_s=now_s, decision=decision, **info)
        self.state["cusp_heading_holds"].append(hold)
        self.state["cusp_heading_hold"] = None
        if decision == "released":
            self.close_watch(
                now_s, "cusp_pending", end
            )  # none is open: held() closed it
            self.state["cusp_heading_watch"] = {
                "hold": len(self.state["cusp_heading_holds"]) - 1,
                "tracker": hold["key"][0],
                "leg": hold["key"][1] + 1,
                "moving_s": None,
                "max_abs_lateral_m": 0.0,
                "max_abs_heading_rad": 0.0,
            }

    def tick(
        self,
        now_s: float,
        tracker_id,
        leg: int,
        end: dict,
        pending: bool,
        speed_mps: float,
        offset=None,
    ) -> None:
        """One control tick of the observe phase, before the hold step: the offset (``offset`` =
        (lateral_m, heading_rad) from the watched path, or None) counts first, the closing tick's
        included (Codex fix A implementation review 2 P3); then the watch closes when its leg
        ended (``end`` is the tracking result now), else notes when it first moves."""
        watch = self.state["cusp_heading_watch"]
        if watch is None:
            return
        if tracker_id == watch["tracker"] and offset is not None:
            watch["max_abs_lateral_m"] = max(
                watch["max_abs_lateral_m"], abs(float(offset[0]))
            )
            watch["max_abs_heading_rad"] = max(
                watch["max_abs_heading_rad"], abs(float(offset[1]))
            )
        if tracker_id != watch["tracker"]:
            self.close_watch(now_s, "replaced", end)
        elif leg != watch["leg"] or end["status"] == "arrived":
            self.close_watch(now_s, "accepted", end)
        elif end["status"] == "failed":
            self.close_watch(now_s, "failed", end)
        elif pending:
            self.close_watch(now_s, "cusp_pending", end)
        elif watch["moving_s"] is None and speed_mps != 0.0:
            watch["moving_s"] = now_s

    def close_watch(self, now_s: float, how: str, end: dict) -> None:
        """End the watch (if any): how = accepted (the leg's end was accepted -- the next leg or
        arrival), cusp_pending (held again at its end; that hold's record decides), failed, or
        replaced (another observe tracker took over)."""
        if how not in ENDS:
            raise ValueError(f"unknown end {how!r}")
        watch = self.state["cusp_heading_watch"]
        if watch is None:
            return
        self.state["cusp_heading_holds"][watch["hold"]]["next_leg"] = {
            "time_s": now_s,
            "end": how,
            **end,
            "moving_s": watch["moving_s"],
            "max_abs_lateral_m": watch["max_abs_lateral_m"],
            "max_abs_heading_rad": watch["max_abs_heading_rad"],
        }
        self.state["cusp_heading_watch"] = None
