"""The fork relation and the load of a live session (persistent console plan D5a, 4th ed.).

Session state, not a mission's: what the last mission left the forks in stays after it ends. It is not proven from
geometry -- a live run has no near-field tracking and the map-frame estimates drift with time (plan D5a) -- but set
from where the mission put the forks (its transitions) and the lift/lower verdicts, and made unknown where a pallet
check fails. ``basis`` says so in what is published.

relation: ``free`` (forks outside any pallet), ``constrained`` (inside, the pallet supported elsewhere),
``carrying`` (inside, the pallet lifted -- the lift verdict passed), ``unknown``.
load: ``none``, ``acquiring`` (lifting), ``confirmed``, ``releasing`` (lowering), ``unknown``.
"""

from __future__ import annotations

# The transition into a phase, and what it leaves the forks in (plan D5a's table). A phase not listed changes
# nothing: observe, approach, settle and the return keep the forks outside a pallet.
_ON_ENTRY = {
    "insert": ("constrained", "none"),
    "lift": ("constrained", "acquiring"),
    "lower": ("constrained", "releasing"),
}
_ON_STEP = {
    ("lift", "extract"): ("carrying", "confirmed"),  # the lift verdict passed (the pallet is off the floor)
    ("lower", "withdraw"): ("constrained", "none"),  # the lower verdict passed (on the floor, forks still in)
    ("withdraw", "settle"): ("free", "none"),  # the withdrawal straight ran to its end
}


class CargoState:
    def __init__(self) -> None:
        self.relation = "free"  # a session starts with empty forks outside any pallet (the scene's contract)
        self.load = "none"
        self.pallet_id: str | None = None
        self.invalidated: str | None = None  # the pallet check that made it unknown

    def on_transition(self, from_phase: str | None, to_phase: str, pallet_id: str | None) -> None:
        if (from_phase, to_phase) in _ON_STEP:
            self.relation, self.load = _ON_STEP[(from_phase, to_phase)]
        elif to_phase in _ON_ENTRY and from_phase != to_phase:
            self.relation, self.load = _ON_ENTRY[to_phase]
        else:
            return
        if pallet_id is not None:
            self.pallet_id = pallet_id
        self.invalidated = None

    def invalidate(self, reason: str) -> None:
        """A pallet check failed (dropped, slipped, displaced, forbidden contact, a measured pallet overlap, its
        tilt): whatever the forks were in is no longer known -- with or without a mission."""
        self.relation, self.load, self.invalidated = "unknown", "unknown", reason

    def submit_refusal(self) -> str | None:
        if self.relation in ("constrained", "unknown"):
            return "recovery_required"
        if self.relation == "carrying":
            return "cargo_held"  # until the cargo submit (plan D5, stage 6c)
        return None

    def release_allowed(self) -> bool:
        """The end SLAM release (plan D1 step 4): the truck may drive on its own after it."""
        return self.relation in ("free", "carrying")

    def loaded_outline(self) -> bool:
        """The outline the permits use: the load's, unless the forks are known to be empty or merely inside."""
        return self.relation == "carrying" or self.load in ("acquiring", "releasing", "unknown")

    def safety_fields(self) -> dict:
        return {"load": {"loaded": self.loaded_outline(), "basis": "phase+cargo", "pallet_id": self.pallet_id,
                         "state": self.load},
                "fork_in_pocket": {"value": self.relation != "free", "basis": "phase+cargo",
                                   "relation": self.relation}}
