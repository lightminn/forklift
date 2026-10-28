"""The mission's targets and destinations with their revision (M-a4; plan docs/plans/2026-10-15-m-a-mission-core.md
D7, contract docs/interfaces/2026-10-13-operator-console-contract-draft.md section 5.1).

Pure. ``targets_revision`` goes up whenever a pallet's pose or state, or the correction version the poses are
expressed in, changes -- so an ID picked from an older list never names another pallet (a submit with an older
revision is ``stale_targets``). A carried pallet has no pose (it moves with the truck every tick; carrying it in
the list would raise the revision without end); put down, it is registered again where it stands, under the same
ID. IDs are never reused within a session.
"""

from __future__ import annotations

from dataclasses import dataclass, field, replace

STATES = ("available", "carried", "delivered")


@dataclass(frozen=True)
class Pose:
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class Pallet:
    pallet_id: str
    kind: str
    pose: Pose | None
    state: str = "available"
    depth_m: float | None = None  # overall size for the console's drawing (C3); None when not known
    width_m: float | None = None
    # What the pose rests on (persistent console plan D10): "prior" as registered (a simulation registers the scene's
    # placement -- the teleop permit then keeps out of the whole pickup zone), "recognised" once the observation
    # placed it, "delivered" where it was put down; and the error margin the permit grows it by (None: prior).
    basis: str = "prior"
    uncertainty_m: float | None = None
    # Persistent console plan D3 (3rd ed.): the destination the pallet is verified to stand at -- the first place
    # (D-0) at the start, a destination only after its delivery verdict passed, None once lifted
    at: str | None = None
    # Plan 2026-10-28-adoption-zone-check: where it was last put down (a delivery or a recovery), re-expressed with the
    # frame -- a re-pick's adoption rectangle sits about it; a later observation's pose does not move it. None: never
    # put down in this session (its first place's zone applies). Not part of the targets message.
    put_down_pose: Pose | None = None


@dataclass
class TargetsBook:
    pallets: dict = field(default_factory=dict)
    destinations: dict = field(default_factory=dict)
    revision: int = 0
    correction_version: int = 0
    _issued: set = field(default_factory=set)
    # Plan D3: the allowed pallet shapes, kind -> (depth, width); a pallet registered without its size must be of one
    # of them, and the largest bounds an unknown size in the occupancy check
    shapes: dict = field(default_factory=dict)

    # --- registration ---------------------------------------------------------------------------------------
    def add_shape(self, kind: str, depth_m: float, width_m: float) -> None:
        """An allowed pallet shape (plan D3); configuration, not a list change (the revision stays)."""
        if not (depth_m > 0 and width_m > 0):
            raise ValueError("a shape needs a positive depth and width")
        self.shapes[kind] = (float(depth_m), float(width_m))

    def add_pallet(self, pallet_id: str, kind: str, pose: Pose, *, depth_m: float | None = None,
                   width_m: float | None = None, at: str | None = None) -> None:
        if pallet_id in self._issued:
            raise ValueError(f"pallet id {pallet_id!r} was already used in this session")
        if (depth_m is None or width_m is None) and self.shapes and kind not in self.shapes:
            raise ValueError(f"pallet {pallet_id!r} of kind {kind!r} has no size and no allowed shape (plan D3)")
        self._issued.add(pallet_id)
        self.pallets[pallet_id] = Pallet(pallet_id, kind, pose, depth_m=depth_m, width_m=width_m, at=at)
        self.revision += 1

    def add_destination(self, destination_id: str, pose: Pose) -> None:
        if destination_id in self._issued:
            raise ValueError(f"destination id {destination_id!r} was already used in this session")
        self._issued.add(destination_id)
        self.destinations[destination_id] = pose
        self.revision += 1

    # --- changes ----------------------------------------------------------------------------------------------
    def _set(self, pallet_id: str, **changes) -> bool:
        old = self.pallets[pallet_id]
        new = replace(old, **changes)
        if new == old:
            return False
        self.pallets[pallet_id] = new
        self.revision += 1
        return True

    def update_pose(self, pallet_id: str, pose: Pose, *, basis: str | None = None,
                    uncertainty_m: float | None = None, carry_put_down: bool = False) -> bool:
        """A new estimate of a pallet on the floor; a carried pallet keeps no pose. ``basis``/``uncertainty_m``:
        what the estimate rests on, when it says (plan D10); else they stay. ``carry_put_down``: the same put-down
        expressed again (its own hold released, its delivering mission's refresh) -- the snapshot moves with it; an
        observation's estimate leaves the snapshot where it was put down."""
        if self.pallets[pallet_id].state == "carried":
            return False
        changes = {"pose": pose}
        if carry_put_down and self.pallets[pallet_id].put_down_pose is not None:
            changes["put_down_pose"] = pose
        if basis is not None:
            changes.update(basis=basis, uncertainty_m=uncertainty_m)
        return self._set(pallet_id, **changes)

    def update_destination(self, destination_id: str, pose: Pose) -> bool:
        """A new estimate of a destination (a docking match, a frame change); the same pose changes nothing."""
        if self.destinations[destination_id] == pose:
            return False
        self.destinations[destination_id] = pose
        self.revision += 1
        return True

    def carried(self, pallet_id: str) -> bool:
        return self._set(pallet_id, pose=None, state="carried", at=None)  # lifted: its verified place is gone

    def verify_at(self, pallet_id: str, destination_id: str | None) -> bool:
        """The pallet's verified place (plan D3): a destination after its delivery verdict passed, None when it is
        moved by other means (a manual recovery's ground verdict)."""
        return self._set(pallet_id, at=destination_id)

    def put_down(self, pallet_id: str, pose: Pose, *, state: str = "delivered",
                 uncertainty_m: float | None = None) -> bool:
        """Registered again where it was put down, under the same ID (basis "delivered", plan D10)."""
        if state not in STATES or state == "carried":
            raise ValueError(state)
        return self._set(pallet_id, pose=pose, state=state, basis="delivered", uncertainty_m=uncertainty_m,
                         put_down_pose=pose)

    def set_correction_version(self, version: int, reexpress=None) -> bool:
        """The poses are re-expressed in a new correction (``reexpress(pose) -> pose``, the change of frame) and
        the revision goes up -- even when no pose moved, the frame they are in did."""
        if version == self.correction_version:
            return False
        self.correction_version = version
        if reexpress is not None:
            for pid, pallet in list(self.pallets.items()):
                if pallet.pose is not None:
                    pallet = self.pallets[pid] = replace(pallet, pose=reexpress(pallet.pose))
                if pallet.put_down_pose is not None:
                    self.pallets[pid] = replace(pallet, put_down_pose=reexpress(pallet.put_down_pose))
            self.destinations = {did: reexpress(pose) for did, pose in self.destinations.items()}
        self.revision += 1
        return True

    # --- the submit check and the message ---------------------------------------------------------------------
    def check_submit(self, pallet_id: str, destination_id: str, targets_revision: int, *,
                     loaded: bool = False) -> str | None:
        """None when a submit may name these; otherwise the refusal reason."""
        if targets_revision != self.revision:
            return "stale_targets"
        pallet = self.pallets.get(pallet_id)
        if pallet is None or destination_id not in self.destinations:
            return "invalid"
        # delivered: put down, available again where it lies; carried: only a loaded submit (plan D5 6c)
        if pallet.state not in (("carried",) if loaded else ("available", "delivered")):
            return "invalid"
        if pallet.at == destination_id:
            return "already_there"  # plan D3: its verified place, not a distance (the list copies the target's pose)
        occupied = self._occupied(pallet, destination_id)
        if occupied is None:
            return "invalid"  # no size and no shape to bound it: no occupancy verdict can be made
        return "destination_occupied" if occupied else None

    def _size(self, pallet: Pallet):
        if pallet.depth_m is not None and pallet.width_m is not None:
            return (float(pallet.depth_m), float(pallet.width_m))
        if not self.shapes or pallet.kind not in self.shapes:
            return None  # a kind outside the allowed set cannot be bounded (Codex D3 impl round 1 P2)
        # an unknown size takes the largest allowed shape (Codex D3 round 1 P2-2: the submitted one's understates)
        return (max(d for d, _ in self.shapes.values()), max(w for _, w in self.shapes.values()))

    def _occupied(self, pallet: Pallet, destination_id: str) -> bool | None:
        """Whether another pallet on the floor overlaps the submitted one placed at the destination: D10's outline
        of the other (prior 0.30 m, else its uncertainty, none 0.21 m -- estimate plus placement error) against the
        submitted pallet's own outline, by the separating axes. None: a size that cannot be bounded."""
        from forklift_core.safety.pallet_permit import obb_overlap, preparation_obstacle

        others = [o for o in self.pallets.values()
                  if o.pallet_id != pallet.pallet_id and o.state != "carried" and o.pose is not None]
        if not others:
            return False  # nothing else on the floor: no size is needed
        size = self._size(pallet)
        if size is None:
            return None
        dest = self.destinations[destination_id]
        for other in others:
            other_size = self._size(other)
            if other_size is None:
                return None
            rect = preparation_obstacle(other, None, size=other_size)
            if obb_overlap([[rect.x_m, rect.y_m]], (rect.length_m / 2, rect.width_m / 2), [rect.yaw_rad],
                           (dest.x_m, dest.y_m), (size[0] / 2, size[1] / 2), dest.yaw_rad)[0]:
                return True
        return False

    def message(self, map_frame: str = "map") -> dict:
        def pose_(p):
            return None if p is None else {"x_m": p.x_m, "y_m": p.y_m, "yaw_rad": p.yaw_rad}

        def pallet_(p):
            item = {"pallet_id": p.pallet_id, "kind": p.kind, "pose": pose_(p.pose), "state": p.state,
                    "basis": p.basis, "uncertainty_m": p.uncertainty_m}  # plan D10 (contract 3)
            if p.depth_m is not None and p.width_m is not None:  # the size, when registered with one
                item.update(depth_m=p.depth_m, width_m=p.width_m)
            return item

        return {"t": "targets", "targets_revision": self.revision, "map_frame": map_frame,
                "correction_version": self.correction_version,
                "pallets": [pallet_(p) for p in self.pallets.values()],
                "destinations": [{"destination_id": d, "pose": pose_(p)} for d, p in self.destinations.items()]}
