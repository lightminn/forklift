"""The unconfirmed position check's contact record (docs/plans/2026-11-06-unconfirmed-validation.md 1.4).

Every forklift rigid body carries a PhysX contact report (``--pose-check-contacts``); after each physics step the
runner hands in the report's actor pairs, and this keeps each contact of a forklift body with anything but the floor
and the forklift itself as an interval by pair: its first and last time, the check's statuses, the mission phases and
the cargo relations seen. An interval is split where the check's confirmation changes, so a judge counts the ones
before the confirmation (every one a failure) apart from the ones after it (forbidden ones only, by the runner's own
truth checks; a fork carrying its pallet is the normal support). Isaac-free but for ``read_pairs``.
"""

from __future__ import annotations

ROBOT_ROOT = "/World/Forklift"
# the warehouse scene's floor collider (steering_probe_arms.FLOOR_TOKEN) and the decals lying on it
FLOOR_TOKENS = ("/GroundPlane/", "SM_FloorDecal")


def _robot(path: str) -> bool:
    return path == ROBOT_ROOT or path.startswith(ROBOT_ROOT + "/")


def external_pairs(pairs) -> set:
    """(forklift body, other) of each pair with exactly one side on the forklift and the other not the floor."""
    out = set()
    for a, b in pairs:
        a, b = str(a), str(b)
        if _robot(a) == _robot(b):
            continue
        body, other = (a, b) if _robot(a) else (b, a)
        if any(token in other for token in FLOOR_TOKENS):
            continue
        out.add((body, other))
    return out


def read_pairs():
    """This physics step's contact report as (actor0, actor1) prim paths; None when it cannot be read."""
    try:
        from omni.physx import get_physx_simulation_interface
        from pxr import PhysicsSchemaTools

        headers, _ = get_physx_simulation_interface().get_contact_report()
        return [(str(PhysicsSchemaTools.intToSdfPath(h.actor0)), str(PhysicsSchemaTools.intToSdfPath(h.actor1)))
                for h in headers]
    except Exception:  # noqa: BLE001 -- counted as unreadable, never taken as no contact
        return None


class ContactLog:
    def __init__(self):
        self.open: dict = {}
        self.intervals: list = []
        self.from_t = None
        self.unreadable = 0

    def update(self, t: float, pairs, *, status: str, phase: str, relation: str) -> None:
        if self.from_t is None:
            self.from_t = float(t)
        if pairs is None:  # the open contacts stay open: an unreadable step says nothing
            self.unreadable += 1
            return
        confirmed = status == "confirmed"
        now = external_pairs(pairs)
        for key in list(self.open):
            if key not in now or self.open[key]["confirmed"] != confirmed:
                self.intervals.append(self.open.pop(key))
        for body, other in now:
            row = self.open.get((body, other))
            if row is None:
                row = self.open[(body, other)] = {"body": body, "other": other, "t0": float(t), "t1": float(t),
                                                  "confirmed": confirmed, "statuses": set(), "phases": set(),
                                                  "relations": set()}
            row["t1"] = float(t)
            row["statuses"].add(status)
            row["phases"].add(phase)
            row["relations"].add(relation)

    def record(self) -> dict:
        rows = self.intervals + list(self.open.values())
        rows.sort(key=lambda r: (r["t0"], r["body"], r["other"]))
        return {"from_t": self.from_t, "unreadable": self.unreadable,
                "intervals": [{**r, "statuses": sorted(r["statuses"]), "phases": sorted(r["phases"]),
                               "relations": sorted(r["relations"])} for r in rows]}


__all__ = ["ContactLog", "FLOOR_TOKENS", "ROBOT_ROOT", "external_pairs", "read_pairs"]
