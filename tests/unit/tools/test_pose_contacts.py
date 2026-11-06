"""The unconfirmed position check's contact record (sim/isaac/pose_contacts.py, docs/plans/2026-11-06-unconfirmed-
validation.md 1.4): every contact of a forklift body with anything but the floor and the forklift itself, as intervals
by pair, split where the check's confirmation changes, with the mission phase and the cargo relation seen."""

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

import pose_contacts as PC  # noqa: E402

FLOOR = "/World/Environment/GroundPlane/CollisionPlane"
TYRE = "/World/Forklift/rear_left_wheel"
FORK = "/World/Forklift/fork_carriage"
PALLET = "/World/Pallet_0/deck"


def test_only_external_contacts_other_than_the_floor_count():
    pairs = [(TYRE, FLOOR), (FORK, TYRE), (PALLET, FORK), ("/World/Shelf/post", "/World/Pallet_0/deck")]
    assert PC.external_pairs(pairs) == {(FORK, PALLET)}


def test_a_floor_decal_is_the_floor():
    # D session val3_042: the decal's collider lies on the floor
    assert PC.external_pairs([(TYRE, "/World/Environment/SM_FloorDecal_RecRed_12/mesh")]) == set()


def test_a_contact_is_one_interval_until_it_ends_or_the_confirmation_changes():
    log = PC.ContactLog()
    log.update(1.0, [(FORK, PALLET)], status="unconfirmed", phase="idle", relation="free")
    log.update(1.1, [(FORK, PALLET), (TYRE, FLOOR)], status="checking", phase="idle", relation="free")
    log.update(1.2, [(PALLET, FORK)], status="confirmed", phase="approach", relation="free")  # split here
    log.update(1.3, [(FORK, PALLET)], status="confirmed", phase="insert", relation="constrained")
    log.update(1.4, [], status="confirmed", phase="insert", relation="constrained")
    log.update(2.0, [(FORK, PALLET)], status="confirmed", phase="lift", relation="carrying")
    rec = log.record()
    assert [(r["t0"], r["t1"], r["confirmed"]) for r in rec["intervals"]] == [(1.0, 1.1, False), (1.2, 1.3, True),
                                                                             (2.0, 2.0, True)]
    first, second, third = rec["intervals"]
    assert first["body"] == FORK and first["other"] == PALLET and first["statuses"] == ["checking", "unconfirmed"]
    assert second["phases"] == ["approach", "insert"] and second["relations"] == ["constrained", "free"]
    assert third["relations"] == ["carrying"] and rec["from_t"] == 1.0 and rec["unreadable"] == 0
    json.dumps(rec)


def test_an_unreadable_report_is_counted_not_taken_as_no_contact():
    log = PC.ContactLog()
    log.update(1.0, [(FORK, PALLET)], status="unconfirmed", phase="idle", relation="free")
    log.update(1.1, None, status="unconfirmed", phase="idle", relation="free")
    log.update(1.2, [(FORK, PALLET)], status="unconfirmed", phase="idle", relation="free")
    rec = log.record()
    assert rec["unreadable"] == 1 and [(r["t0"], r["t1"]) for r in rec["intervals"]] == [(1.0, 1.2)]
