"""PR 7 7r-1 (docs/plans/2026-11-26-7r-replay-run.md 1-3, 5): an observe record checked, verified and indexed, and
given back to a HIL run's loop by the replay sources -- on synthetic records made by the observe tool's own code paths
(tests/fixtures/replay/make_record.py, TEST ONLY)."""

import json
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

from forklift_core.live import replay_record as rr
from forklift_core.live import replay_sources as rs

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tests" / "fixtures" / "replay"))
import make_record as mk  # noqa: E402

MS = 1_000_000


def load(path, **kw):
    kw.setdefault("lidar_cfg", mk.lidar_config())
    kw.setdefault("imu_cfg", mk.imu_config())
    kw.setdefault("imu_required", True)
    kw.setdefault("allow_synthetic", True)
    return rr.load_replay_record(path, **kw)


def rewrite(path, edit):
    rows = [json.loads(line) for line in Path(path).read_text().splitlines()]
    rows = edit(rows)
    Path(path).write_text("".join(json.dumps(r) + "\n" for r in rows))
    from forklift_core.live.observe_record import file_sha256

    Path(str(path) + ".ok").write_text(file_sha256(path) + "\n")


@pytest.fixture(scope="module")
def good(tmp_path_factory):
    p = tmp_path_factory.mktemp("rec") / "obs.jsonl"
    info = mk.write_record(p)
    return p, info


# --- the record: qualification and verification --------------------------------------------------------------------

def test_a_good_record_loads_with_its_anchor_and_two_file_order_streams(good):
    p, info = good
    rec = load(p)
    assert rec.anchor_ns == info["anchor_ns"] and rec.synthetic and rec.imu_config_recorded
    assert len(rec.lidar) == info["scans"] and len(rec.imu) > 8000
    assert rec.counts["scans_operating"] == rec.counts["scans_accepted"] == info["scans"] - 9  # 9 establish the clock
    offs = [o for _ns, o in rec.lidar]
    assert offs == sorted(offs)  # file order, never re-sorted by time


def test_a_synthetic_record_needs_its_flag(good):
    with pytest.raises(rr.ReplayRefused) as e:
        load(good[0], allow_synthetic=False)
    assert e.value.reason == "record_synthetic"


@pytest.mark.parametrize("edit,reason", [
    (lambda rows: rows[:-1] + [{**rows[-1], "result": "no_scans"}], "record_result"),
    (lambda rows: [r for r in rows if r.get("t") != "slam_header"], "record_no_slam_header"),
    (lambda rows: [{**r, "delay_bound_s": None} if r.get("t") == "state" else r for r in rows], "record_no_anchor"),
    (lambda rows: [r for r in rows if r.get("t") not in ("imu", "imu_gen")], "record_no_imu"),
    (lambda rows: [{**r, "schema": "x/1"} if r.get("t") == "header" else r for r in rows], "record_schema"),
])
def test_refusals(tmp_path, edit, reason):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, seconds=45)
    rewrite(p, edit)
    with pytest.raises(rr.ReplayRefused) as e:
        load(p)
    assert e.value.reason == reason


def test_an_unsealed_or_changed_record_is_incomplete(tmp_path):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p)
    with open(p, "a") as f:
        f.write("\n")
    with pytest.raises(rr.ReplayRefused) as e:
        load(p)
    assert e.value.reason == "record_incomplete"


def test_a_short_record_is_refused(tmp_path):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, seconds=20)
    with pytest.raises(rr.ReplayRefused) as e:
        load(p)
    assert e.value.reason == "record_too_short"


def test_the_shipped_lidar_config_gives_no_operating_scans(good):
    from forklift_core.sensors.lidar_cfg import load_lidar_config

    with pytest.raises(rr.ReplayRefused) as e:
        load(good[0], lidar_cfg=load_lidar_config())
    assert e.value.reason == "replay_no_operating_scans"
    with pytest.raises(rr.ReplayRefused) as e:
        load(good[0], lidar_cfg=mk.lidar_config(mount_measured=False))  # the layer takes them, SLAM would not
    assert e.value.reason == "replay_no_operating_scans"


def test_the_start_window_needs_seven_scans_of_one_generation(tmp_path):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, scans_until_s=14.0, seconds=45)  # scans end 13.5 s after t0: 12.5 s past the anchor at 1 s
    with pytest.raises(rr.ReplayRefused) as e:
        load(p)
    assert e.value.reason == "replay_no_operating_scans"


def test_a_scan_verdict_that_does_not_come_back_is_a_mismatch(tmp_path):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p)

    def edit(rows):
        k = [i for i, r in enumerate(rows) if r.get("t") == "scan"][20]
        rows[k] = {**rows[k], "verdict": "stale_id"}
        return rows

    rewrite(p, edit)
    with pytest.raises(rr.ReplayRefused) as e:
        load(p)
    assert e.value.reason == "replay_mismatch"


def test_imu_verdicts_are_verified_with_the_recorded_config(tmp_path):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, imu_rewind_at_s=20.0)
    rec = load(p, imu_cfg=mk.imu_config(receive_gap_s=0.5))  # the record names its own: that one judges
    assert rec.imu_config_recorded and not rec.counts["imu_config_unrecorded"]

    def edit(rows):
        k = [i for i, r in enumerate(rows) if r.get("t") == "imu"][500]
        rows[k] = {**rows[k], "verdict": "late"}
        return rows

    rewrite(p, edit)
    with pytest.raises(rr.ReplayRefused) as e:
        load(p)
    assert e.value.reason == "replay_mismatch"


def test_a_record_without_its_imu_config_is_judged_with_the_given_one_and_counted(tmp_path):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, imu_config_in_header=False, imu_rewind_at_s=20.0,
                    imu_cfg=mk.imu_config(restart_back_us=100_000))  # recorded: the 50 ms step back was "late"
    rec = load(p, imu_cfg=mk.imu_config(restart_back_us=20_000))  # now judged a restart
    assert not rec.imu_config_recorded and rec.counts["imu_config_unrecorded"] > 0


# --- the scan source -----------------------------------------------------------------------------------------------

class Slam:
    def __init__(self):
        self.calls = []

    def generation(self, gen):
        self.calls.append(("gen", gen))

    def offer(self, s):
        self.calls.append(("offer", s.key))


def reader(rec, entries):
    """The record read through before the test drives it (a test loop runs far faster than the wall clock; a run's
    reader keeps ahead of it -- and when it does not, that is read_late, tested apart)."""
    r = rr.StreamReader(rec.path, entries, capacity=1 << 20)
    assert r.wait_read()
    return r


def scan_source(rec, *, start_ns=0, slam=None, transport=None):
    tr = transport or SimpleNamespace(hil_plant=True, hil_boot=1)
    return rs.ReplayScan(tr, rec, mk.lidar_config(), start_ns, slam=slam, reader=reader(rec, rec.lidar)), tr


def drive(src, hil_from, hil_to, step_ms=20):
    """The loop's order each tick: events, then scans. Returns [(now, events, scans)]."""
    out = []
    for now in range(hil_from, hil_to, step_ms * MS):
        ev = src.scan_source_events(now)
        sc = src.scans(now)
        out.append((now, ev, sc))
    return out


def test_nothing_before_the_anchor_and_each_scan_after_its_moved_arrival(tmp_path):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, anchor_after_s=3.0)  # scans accepted before the anchor: the prefix
    rec = load(p)
    hil_anchor = 900_000 * MS
    delta = hil_anchor - rec.anchor_ns
    src, _tr = scan_source(rec, start_ns=hil_anchor - 5 * 10**9)
    assert drive(src, hil_anchor - 200 * MS, hil_anchor, 20) and src.counts["delivered"] == 0  # no anchor yet
    src.set_anchor(hil_anchor, hil_anchor)
    ticks = drive(src, hil_anchor, hil_anchor + 10 * 10**9)
    got = [(now, s) for now, _e, sc in ticks for s in sc]
    assert len(got) > 80 and src.counts["before_anchor"] > 0
    for now, s in got:
        assert s.origin == "replayed"
        lo_ns = int(round(s.first_s * 1e9)) + src.start_ns
        assert lo_ns <= now  # acquired before it was given
    # the first scan given: arrived at the anchor call or later, on the record axis moved by delta
    first_now, first = got[0]
    assert int(round(first.first_s * 1e9)) + src.start_ns + 2 * MS - delta >= rec.anchor_ns - 200 * MS


def test_a_scan_arriving_300_ms_late_is_given_300_ms_late(tmp_path):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, late_scans=(40,))
    rec = load(p)
    hil = 2_000_000 * MS
    src, _ = scan_source(rec, start_ns=hil - 10**9)
    src.set_anchor(hil, hil)
    ticks = drive(src, hil, hil + 8 * 10**9, 10)
    # scan 40's first beam: t0 + 4.0 s on the record axis; anchor at t0 + 1 s -> 3.0 s after the HIL anchor
    first40 = hil + 3 * 10**9
    seen = [now for now, _e, sc in ticks for s in sc
            if abs(int(round(s.first_s * 1e9)) + src.start_ns + 2 * MS - first40) < 1 * MS]
    assert seen and seen[0] >= first40 + 98 * MS + 300 * MS


def test_receive_gap_latches_resumed_reconnects_socket_error_is_fatal(tmp_path):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, events=((5.0, "receive_gap"), (6.0, "receive_resumed"), (30.0, "socket_error")))
    rec = load(p)
    hil = 3_000_000 * MS
    src, _ = scan_source(rec, start_ns=hil - 10**9)
    src.set_anchor(hil, hil)
    ticks = drive(src, hil, hil + 40 * 10**9, 50)
    events = [e for _now, ev, _sc in ticks for e in ev]
    assert events == [("lost", "lidar"), ("connected", "lidar")]
    assert src.fatal == "scan_socket_error"


def test_a_generation_change_in_the_same_call_follows_the_scan_offered_before_it(tmp_path):
    """Codex plan round 1 P1: A (operating) then a receive_gap in one call -- A is offered before the new generation
    is told, so A's reply is of the earlier generation."""
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, events=((5.205, "receive_gap"),))  # just after the scan whose first beam is 5.1 s
    rec = load(p)
    hil = 4_000_000 * MS
    slam = Slam()
    src, _ = scan_source(rec, start_ns=hil - 10**9, slam=slam)
    src.set_anchor(hil, hil)
    drive(src, hil, hil + 4 * 10**9, 20)
    slam.calls.clear()
    src.scan_source_events(hil + 4 * 10**9 + 207 * MS)  # one call: the scans before the gap, then the gap
    kinds = [c[0] for c in slam.calls]
    assert kinds.count("offer") >= 1 and kinds[-1] == "gen"  # the new generation told after the scans offered
    last_offer = max(i for i, k in enumerate(kinds) if k == "offer")
    assert slam.calls[-1][1] != slam.calls[last_offer - 1][1] if last_offer > 0 else True


def test_only_the_scans_the_layer_takes_go_to_slam(good):
    rec = load(good[0])
    hil = 5_000_000 * MS
    slam = Slam()
    src, _ = scan_source(rec, start_ns=hil - 10**9, slam=slam)
    src.set_anchor(hil, hil)
    got = src.scans(hil + 3 * 10**9)  # 3 s of scans in one call: only the last SCANS_PER_CALL go
    offers = [c for c in slam.calls if c[0] == "offer"]
    assert len(got) == rs.SCANS_PER_CALL == len(offers) and src.counts["cut"] > 0


def test_a_lost_hil_check_gives_nothing_and_a_boot_change_sets_a_boundary(good):
    rec = load(good[0])
    hil = 6_000_000 * MS
    tr = SimpleNamespace(hil_plant=True, hil_boot=1)
    src, _ = scan_source(rec, start_ns=hil - 10**9, transport=tr)
    src.set_anchor(hil, hil)
    drive(src, hil, hil + 2 * 10**9)
    tr.hil_plant = False
    lost = drive(src, hil + 2 * 10**9, hil + 3 * 10**9)
    assert not any(sc for _n, _e, sc in lost) and src.counts["hil_check_lost"] > 0
    tr.hil_plant, tr.hil_boot = True, 2
    after = drive(src, hil + 3 * 10**9, hil + 5 * 10**9)
    given = [s for _n, _e, sc in after for s in sc]
    assert given and all(int(round(s.first_s * 1e9)) + src.start_ns >= src.gate.boundary_ns for s in given)
    assert src.counts["before_boot_boundary"] >= 0 and src.gate.boundary_ns == hil + 3 * 10**9


def test_the_scan_stream_ends(good):
    rec = load(good[0])
    hil = 7_000_000 * MS
    src, _ = scan_source(rec, start_ns=hil - 10**9)
    src.set_anchor(hil, hil)
    drive(src, hil, hil + 50 * 10**9, 200)
    assert src.eof()


# --- the IMU source ------------------------------------------------------------------------------------------------

def imu_source(rec, transport=None, cfg=None):
    tr = transport or SimpleNamespace(hil_plant=True, hil_boot=1)
    return rs.ReplayImu(tr, rec, cfg or mk.imu_config(), reader=reader(rec, rec.imu)), tr


def loop_reads(src, hil_from, hil_to, step_ms=20, gens=None):
    """The loop's order: samples, then generation only when the read did not stop at its budget."""
    got = []
    for now in range(hil_from, hil_to, step_ms * MS):
        out = src.samples(now, 64)
        got += [(now, o) for o in out]
        if not src.last_read_capped:
            g = src.generation()
            if gens is not None:
                gens.append(g)
    return got


def test_a_long_prefix_goes_in_without_overflow_and_no_false_gap(tmp_path):
    """Codex plan rounds 8-10: 400 samples before the anchor, put in at most 64 a call; no receive gap the record
    did not have; the first sample given after the anchor at its recorded time + delta."""
    p = tmp_path / "obs.jsonl"
    info = mk.write_record(p, anchor_after_s=2.0)
    rec = load(p)
    hil = 8_000_000 * MS
    src, _ = imu_source(rec)
    src.set_anchor(hil, hil)
    gens = []
    got = loop_reads(src, hil, hil + 5 * 10**9, 20, gens)
    assert src.src.counts["overflow"] == 0 and src.counts["before_anchor"] >= 390
    assert set(gens) == {1}  # no generation change at all: no false gap
    first_given = got[0][1]
    delta = hil - info["anchor_ns"]
    assert first_given[2] - delta >= info["anchor_ns"]  # hi: a recorded receipt after the anchor, moved


def test_a_row_arriving_between_samples_and_generation_opens_no_gap(good):
    rec = load(good[0])
    hil = 9_000_000 * MS
    src, _ = imu_source(rec)
    src.set_anchor(hil, hil)
    loop_reads(src, hil, hil + 2 * 10**9, 20)
    src.samples(hil + 2 * 10**9 + 4_900_000, 64)  # just before the next 5 ms sample
    g = src.generation()  # asked later than that (the clock moved on): uses the samples' input instant
    out = src.samples(hil + 2 * 10**9 + 20 * MS, 64)
    assert g == 1 and out and src.src.counts["before_boundary"] == 0


def test_a_stop_before_the_anchor_holds_after_it(tmp_path):
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, anchor_after_s=2.0, imu_rewind_at_s=1.0)
    rec = load(p)
    hil = 10_000_000 * MS
    src, _ = imu_source(rec)
    src.set_anchor(hil, hil)
    got = loop_reads(src, hil, hil + 3 * 10**9)
    assert got == [] and src.src.stopped == "device_restart"


@pytest.mark.parametrize("kind", ["rewind", "lost"])
def test_a_stop_while_the_imu_is_not_read_holds_after(tmp_path, kind):
    """Codex plan rounds 5-6: the IMU bypassed (drain) while the device restarts or is lost -- nothing valid after the
    bypass ends."""
    p = tmp_path / "obs.jsonl"
    mk.write_record(p, **({"imu_rewind_at_s": 5.0} if kind == "rewind" else {"imu_lost_at_s": 5.0}))
    rec = load(p)
    hil = 11_000_000 * MS
    src, _ = imu_source(rec)
    src.set_anchor(hil, hil)
    assert loop_reads(src, hil, hil + 2 * 10**9)
    for now in range(hil + 2 * 10**9, hil + 8 * 10**9, 20 * MS):
        src.drain(now)
    assert loop_reads(src, hil + 8 * 10**9, hil + 10 * 10**9) == []


def test_a_lost_hil_check_drops_the_output_and_a_boot_change_sets_a_boundary(good):
    rec = load(good[0])
    hil = 12_000_000 * MS
    tr = SimpleNamespace(hil_plant=True, hil_boot=1)
    src, _ = imu_source(rec, transport=tr)
    src.set_anchor(hil, hil)
    loop_reads(src, hil, hil + 2 * 10**9)
    tr.hil_plant = False
    assert loop_reads(src, hil + 2 * 10**9, hil + 3 * 10**9) == [] and src.counts["hil_check_lost"] > 0
    tr.hil_plant, tr.hil_boot = True, 2
    after = loop_reads(src, hil + 3 * 10**9, hil + 4 * 10**9)
    assert after and all(o[1] >= src.gate.boundary_ns for _n, o in after)


def test_the_imu_stream_ends_only_when_everything_is_taken(good):
    rec = load(good[0])
    hil = 13_000_000 * MS
    src, _ = imu_source(rec)
    src.set_anchor(hil, hil)
    loop_reads(src, hil, hil + 46 * 10**9, 100)
    for now in range(hil + 46 * 10**9, hil + 47 * 10**9, 20 * MS):
        src.samples(now, 64)
    assert src.eof() and src.src.pending() == 0


def test_a_reader_behind_gives_nothing_more_and_counts_it(good):
    """Plan 4: rows not yet read are never given, and the IMU's clock does not run past them (no false gap)."""
    rec = load(good[0])
    slow = rr.StreamReader(rec.path, rec.imu, start=False)  # never reads
    src = rs.ReplayImu(SimpleNamespace(hil_plant=True, hil_boot=1), rec, mk.imu_config(), reader=slow)
    hil = 14_000_000 * MS
    src.set_anchor(hil, hil)
    assert src.samples(hil + 10**9, 64) == [] and src.last_read_capped and src.counts["read_late"] == 1


def test_the_synthetic_writer_carries_what_the_replay_reads(good):
    """The writer's rows against the keys the replay reads (the observe tool's are checked the same way in
    tests/integration/test_real_observe.py)."""
    seen = set()
    for line in Path(good[0]).read_text().splitlines():
        r = json.loads(line)
        want = rr.READS.get(r.get("t"))
        if want is not None:
            assert want <= set(r), (r["t"], want - set(r))
            seen.add(r["t"])
    assert seen == set(rr.READS) - {"scan_event"}
