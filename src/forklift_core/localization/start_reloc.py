"""Start relocalization (docs/plans/2026-10-29-start-relocalization.md 1.1-1.3).

A session that starts anywhere: while the truck stands still its scans go to the global matcher; a confirmed top, or
else a candidate an operator picks, becomes the estimate the SLAM bridge is started at. ``StartRelocalizer`` is the
pure state machine -- attempts and static rescans under one session deadline from the first scan, the candidates an
operator may pick (those that pass E_MIN and did not walk in refinement, at most four), and candidate generations so
a late pick never selects from a set a retry replaced. The runner owns the threads and the clock; this module owns
the decisions.

Poses: ``base`` is base_link, ``laser`` the LiDAR (``LASER_X_M`` ahead of base), ``rear`` the rear axle
(``REAR_AXLE_OFFSET_M`` behind base); the matcher's candidates are laser poses in the map frame.

The epoch files hand the estimate to the supervisor (``reloc_pose.json``) and its bridge start back (``bridge_go``);
both are written to a temporary file and renamed, and a half-written one reads as absent.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, replace
from pathlib import Path

from forklift_core.localization.global_match import (
    Candidate,
    Decision,
    GlobalMatch,
    suppress,
)
from forklift_core.localization.relocalization_samples import (
    LASER_X_M,
    REAR_AXLE_OFFSET_M,
)

RELOC_POSE = "reloc_pose.json"
BRIDGE_GO = "bridge_go"


def laser_from_base(base) -> tuple:
    x, y, yaw = base
    return (x + LASER_X_M * math.cos(yaw), y + LASER_X_M * math.sin(yaw), yaw)


def base_from_laser(laser) -> tuple:
    x, y, yaw = laser
    return (x - LASER_X_M * math.cos(yaw), y - LASER_X_M * math.sin(yaw), yaw)


def rear_from_base(base, offset_m: float = REAR_AXLE_OFFSET_M) -> tuple:
    x, y, yaw = base
    return (x - offset_m * math.cos(yaw), y - offset_m * math.sin(yaw), yaw)


@dataclass(frozen=True)
class RelocConfig:
    e_min: float
    r_max: float
    r_struct: float
    attempts: int = 3  # the first scan set and up to two static rescans (no in-place turn, no move before S-P1)
    deadline_s: float = 30.0  # one session deadline from the first scan for every attempt
    max_shown: int = 4
    sep_m: float = 0.3  # a candidate refinement moved this far is not pickable (decide's refine_moved)
    sep_rad: float = 0.1
    max_places: int = 3  # F1 (plan 2026-10-31-operator-placed-start 1.2): local re-matches the operator may start


@dataclass(frozen=True)
class Estimate:
    base: tuple  # map frame
    source: str  # matcher | operator
    generation: int
    attempts: int


def _plain_int(v) -> bool:
    return isinstance(v, int) and not isinstance(v, bool)


def _moved(c: Candidate, cfg: RelocConfig) -> bool:
    lp = c.search_pose
    d = math.atan2(math.sin(c.pose[2] - lp[2]), math.cos(c.pose[2] - lp[2]))
    return math.hypot(c.pose[0] - lp[0], c.pose[1] - lp[1]) >= cfg.sep_m or abs(d) >= cfg.sep_rad


class StartRelocalizer:
    """relocalizing -> chosen | awaiting_pick | reloc_failed; awaiting_pick -> chosen (pick); awaiting_pick or
    reloc_failed -> relocalizing (retry); any -> cancelled.

    F1 (plan 2026-10-31-operator-placed-start 1.2): awaiting_pick, reloc_failed or place_preview -> placing (place, at
    most ``max_places`` per session); placing -> place_preview (the local re-match confirmed) or back to the candidate
    stage it came from (refused); place_preview -> chosen (confirm, source ``operator_placed``) or relocalizing
    (retry). A placement keeps the candidate generation: a refused one leaves the shown candidates pickable."""

    def __init__(self, cfg: RelocConfig, clock):
        self.cfg, self.clock = cfg, clock
        self.state, self.generation, self.attempts = "relocalizing", 0, 0
        self.deadline_at: float | None = None
        self.expires_at: float | None = None  # B-P2c: the operator's whole time (clock time); no pick or retry after
        self.estimate: Estimate | None = None
        self.shown: tuple = ()
        self._pool: list = []  # the pickable candidates every attempt of this generation produced, one per place
        self._picked: tuple | None = None
        self.places = 0  # F1: local re-matches started this session (a retry does not reset it)
        self.placed: tuple | None = None  # the pose the operator placed (the one being matched or previewed)
        self.preview: tuple | None = None  # the local re-match's base pose, shown for confirmation
        self.preview_id: int | None = None
        self.place_reason: str | None = None  # the last refusal's reason
        self._stage = "awaiting_pick"  # the candidate stage a placement returns to when it is refused
        self.place_bounds: tuple | None = None  # the map's extent (x0, y0, x1, y1); a placement outside is refused
        self._confirmed: tuple | None = None

    def start(self) -> None:
        """The first scan of this generation is being collected: the session deadline starts now (once)."""
        if self.state == "relocalizing" and self.deadline_at is None:
            self.deadline_at = self.clock() + self.cfg.deadline_s

    def _keep(self, candidates) -> None:
        """Pool the pickable candidates (E_MIN, not moved by refinement) of every attempt (Codex B-P2a review: by
        raw top an attempt's pickable candidate could be lost); duplicates go when the pool is shown."""
        self._pool += [c for c in candidates if c.endpoint_fraction >= self.cfg.e_min and not _moved(c, self.cfg)]

    def _shown(self) -> tuple:
        """The pool by lattice score, one per place (the matcher's suppression over the whole pool, so a better
        candidate between two pooled ones removes both -- Codex B-P2a re-review), at most ``max_shown``."""
        ranked = [replace(c, score=c.search_score) for c in self._pool]  # suppress orders by .score
        kept = suppress(ranked, sep_m=self.cfg.sep_m, sep_rad=self.cfg.sep_rad)[:self.cfg.max_shown]
        by_id = {id(r): c for r, c in zip(ranked, self._pool, strict=True)}
        return tuple(by_id[id(k)] for k in kept)

    def record(self, match: GlobalMatch, decision: Decision) -> str:
        """One attempt's result; returns what follows: rescan, chosen, awaiting_pick or reloc_failed."""
        if self.state != "relocalizing":
            return self.state
        if self.deadline_at is None:
            raise RuntimeError("record() before start(): an attempt has no session deadline")
        self.attempts += 1
        self._keep(match.candidates)
        # past the session deadline, or past the operator's whole time (no choice of the matcher's after it either,
        # Codex B-P2c impl review round 2 P2)
        late = self.clock() > self.deadline_at or self._expired()
        if decision.confirmed and not late:
            self._choose(base_from_laser(match.candidates[0].pose), "matcher")
            return self.state
        if self.attempts < self.cfg.attempts and not late:
            return "rescan"
        self.shown = self._shown()
        self.generation += 1
        self.state = "awaiting_pick" if self.shown else "reloc_failed"
        return self.state

    def _expired(self) -> bool:
        return self.expires_at is not None and self.clock() > self.expires_at

    def pick(self, generation: int, index: int) -> tuple[bool, str]:
        if not (_plain_int(generation) and _plain_int(index)):  # True is no generation 1 (Codex B-P2c design P2)
            return False, "reloc_pick_refused"
        if self._expired():  # the operator's time is over (Codex B-P2c impl review P2)
            return False, "reloc_operator_timeout"
        if self.state == "chosen" and self._picked == (generation, index):
            return True, "duplicate"
        if self.state != "awaiting_pick":
            return False, "reloc_not_awaiting"
        if generation != self.generation:
            return False, "reloc_stale_pick"
        if not 0 <= index < len(self.shown):
            return False, "reloc_pick_refused"
        self._picked = (generation, index)
        self._choose(base_from_laser(self.shown[index].pose), "operator")
        return True, ""

    def place(self, generation: int, pose) -> tuple[bool, str]:
        """F1: the operator's placed base pose (map); its local re-match runs next (``placing``)."""
        if self.state not in ("awaiting_pick", "reloc_failed", "place_preview"):
            return False, "reloc_not_awaiting"
        if self._expired():
            return False, "reloc_operator_timeout"
        if not _plain_int(generation) or generation != self.generation:
            return False, "reloc_stale_pick"
        if not (isinstance(pose, (list, tuple)) and len(pose) == 3 and all(
                isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in pose)):
            return False, "reloc_place_invalid"
        if self.place_bounds is not None:  # off the map (Codex F1 implementation review P2): never matched
            x0, y0, x1, y1 = self.place_bounds
            if not (x0 <= pose[0] < x1 and y0 <= pose[1] < y1):
                return False, "reloc_place_invalid"
        if self.places >= self.cfg.max_places:
            return False, "reloc_place_limit"
        if self.state != "place_preview":
            self._stage = self.state
        self.places += 1
        self.state, self.placed, self.preview, self.preview_id = "placing", tuple(float(v) for v in pose), None, None
        return True, ""

    def placed_result(self, confirmed: bool, base, reason: str | None = None) -> str:
        """The local re-match's result. A result after a STOP, a retry or the end (no longer ``placing``) is ignored."""
        if self.state != "placing":
            return self.state
        if confirmed:
            self.state, self.preview, self.preview_id = "place_preview", tuple(float(v) for v in base), self.places
        else:  # back to the candidate stage, never to a preview the new placement invalidated (Codex F1 plan P3)
            self.state, self.place_reason = self._stage, reason
        return self.state

    def confirm_place(self, generation: int, preview_id: int) -> tuple[bool, str]:
        if not (_plain_int(generation) and _plain_int(preview_id)):
            return False, "reloc_pick_refused"
        if self.state == "chosen" and self._confirmed == (generation, preview_id):
            return True, "duplicate"
        if self._expired():
            return False, "reloc_operator_timeout"
        if self.state != "place_preview":
            return False, "reloc_not_awaiting"
        if generation != self.generation or preview_id != self.preview_id:
            return False, "reloc_stale_pick"
        self._confirmed = (generation, preview_id)
        self._choose(self.preview, "operator_placed")
        return True, ""

    def retry(self, generation: int) -> tuple[bool, str]:
        if self.state not in ("awaiting_pick", "reloc_failed", "stopped", "place_preview"):
            return False, "reloc_not_awaiting"
        if self._expired():
            return False, "reloc_operator_timeout"
        if not _plain_int(generation) or generation != self.generation:
            return False, "reloc_stale_pick"
        self.state, self.attempts, self.shown, self._pool = "relocalizing", 0, (), []
        self.preview = self.preview_id = None  # the count of placements stays (plan 1.2)
        self.generation += 1
        self.deadline_at = None
        return True, ""

    def interrupt(self) -> bool:
        """A STOP before the estimate (B-P2c, Codex design P1): the attempt running or the candidates shown are
        dropped and the generation moves on; a retry may follow. After the choice or the end it changes nothing."""
        if self.state not in ("relocalizing", "awaiting_pick", "reloc_failed", "placing", "place_preview"):
            return False
        self.state, self.shown, self._pool = "stopped", (), []
        self.preview = self.preview_id = None
        self.generation += 1
        return True

    def cancel(self) -> None:
        self.state, self.shown = "cancelled", ()

    def _choose(self, base: tuple, source: str) -> None:  # source: matcher | operator | operator_placed
        """The estimate keeps the generation it was chosen in; the generation then moves on, so any later pick of the
        old set is stale (plan 1.2 step 7)."""
        self.estimate = Estimate(tuple(float(v) for v in base), source, self.generation, self.attempts)
        self.state, self.shown = "chosen", ()
        self.generation += 1


def _write_atomic(path: Path, text: str) -> None:
    tmp = path.with_name(path.name + ".tmp")
    tmp.write_text(text)
    os.replace(tmp, path)


def _read(path: Path):
    try:
        return json.loads(path.read_text())
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def write_reloc_pose(epoch_dir, estimate: Estimate, record: dict) -> None:
    data = {"base": list(estimate.base), "source": estimate.source, "generation": estimate.generation,
            "attempts": estimate.attempts, "record": record}
    _write_atomic(Path(epoch_dir) / RELOC_POSE, json.dumps(data) + "\n")


def read_reloc_pose(epoch_dir):
    return _read(Path(epoch_dir) / RELOC_POSE)


def write_bridge_go(epoch_dir, info: dict) -> None:
    _write_atomic(Path(epoch_dir) / BRIDGE_GO, json.dumps(info) + "\n")


def read_bridge_go(epoch_dir):
    return _read(Path(epoch_dir) / BRIDGE_GO)


def clear_epoch(epoch_dir) -> None:
    for name in (RELOC_POSE, BRIDGE_GO):
        for p in (Path(epoch_dir) / name, Path(epoch_dir) / (name + ".tmp")):
            p.unlink(missing_ok=True)


__all__ = ["BRIDGE_GO", "Estimate", "RELOC_POSE", "RelocConfig", "StartRelocalizer", "base_from_laser",
           "clear_epoch", "laser_from_base", "read_bridge_go", "read_reloc_pose", "rear_from_base", "write_bridge_go",
           "write_reloc_pose"]
