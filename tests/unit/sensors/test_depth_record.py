"""PR 7 7e-1 (docs/plans/2026-11-26-7e-depth-pocket-check.md 1-4): the depth config, the calibration identity, the
depth source's judgement (intervals, generations, the cumulative delay check) and the side files."""

from __future__ import annotations

import dataclasses
import json
from pathlib import Path

import numpy as np
import pytest

from forklift_core.sensors import depth_cfg as dc
from forklift_core.sensors.depth_record import DepthRecorder, read_side_file, side_folder
from forklift_core.sensors.depth_source import DepthSource

MS = 1_000_000
INTR = {"serial": "123", "firmware": "5.16", "stream": "depth", "width": 640, "height": 480, "fx": 388.8121,
        "fy": 388.8121, "ppx": 321.5, "ppy": 239.25, "model": "brown_conrady", "coeffs": [0.0, 0.0, 0.0, 0.0, 0.0]}


def cfg(**over):
    return dataclasses.replace(dc.load_depth_config(), **over)


def test_the_shipped_config_is_diagnostic_and_refuses_an_operating_source():
    c = dc.load_depth_config()
    assert (c.width, c.height, c.fps) == (640, 480, 15) and c.delay_bound_s is None and not c.measured
    assert c.interval_bound_s == c.provisional_delay_bound_s and c.mount.get("measured") is False
    with pytest.raises(ValueError, match="delay_unmeasured"):
        DepthSource(c, operating=True)


def test_a_bound_needs_its_basis_and_the_capture_it_applies_to(tmp_path):
    import yaml

    raw = yaml.safe_load(dc.DEPTH_YAML.read_text())
    raw["timing"].update(delay_bound_s=0.03, measured=True)
    p = tmp_path / "d.yaml"
    p.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="go together"):
        dc.load_depth_config(p)
    raw["timing"]["basis"] = "measured"
    p.write_text(yaml.safe_dump(raw))
    with pytest.raises(ValueError, match="applies_to"):
        dc.load_depth_config(p)
    raw["timing"]["applies_to"] = {k: "x" for k in dc.APPLIES_KEYS}
    p.write_text(yaml.safe_dump(raw))
    assert dc.load_depth_config(p).delay_bound_s == 0.03


def test_the_calibration_id_is_canonical_and_moves_with_every_value():
    a = dc.calib_id(INTR)
    assert a == dc.calib_id(dict(reversed(list(INTR.items()))))  # key order does not matter
    assert a == dc.calib_id(json.loads(json.dumps(INTR)))  # a JSON round trip keeps it
    for k, v in (("serial", "124"), ("firmware", "5.17"), ("width", 848), ("fx", 388.8122), ("ppy", 239.0),
                 ("model", "inverse_brown_conrady"), ("coeffs", [0.0, 0.0, 0.0, 0.0, 1e-9])):
        assert dc.calib_id({**INTR, k: v}) != a, k
    with pytest.raises(KeyError):
        dc.calib_id({k: v for k, v in INTR.items() if k != "fx"})


def z16(value=1000, shape=(480, 640)):
    return np.full(shape, value, dtype=np.uint16)


def feed(src, frames):
    """frames: (host_ns, frame_number, ts_ms[, domain]); returns the events of one take after each push."""
    out = []
    for f in frames:
        host, fn, ts = f[:3]
        dom = f[3] if len(f) > 3 else "hardware_clock"
        src.push(host, fn, ts, dom, {}, 0.001, INTR, None)
        out += src.take(host)
    return out


def frames_of(events):
    return [e[1] for e in events if e[0] == "frame"]


def gens_of(events):
    return [e[1] for e in events if e[0] == "gen"]


def test_intervals_follow_the_receipt_and_the_device_time_only_orders():
    src = DepthSource(cfg())
    ev = feed(src, [(1000 * MS + k * 66 * MS, k, 5000.0 + k * 66.6) for k in range(10)])
    fr = frames_of(ev)
    assert [f["verdict"] for f in fr] == ["ok"] * 10 and not gens_of(ev)
    bound = int(round(cfg().provisional_delay_bound_s * 1e9))
    assert all((f["lo"], f["hi"]) == (f["host_ns"] - bound, f["host_ns"]) for f in fr)
    assert fr[0]["calib_id"] == dc.calib_id(INTR) and all(f["gen"] == 1 for f in fr)


def test_a_receive_gap_opens_a_generation_when_the_next_item_comes_and_the_stream_goes_on():
    """Codex 7e-1 round 1 P2-2: decided when the next frame or loss is judged (or at the end), never on a quiet take
    -- so the rows do not depend on how the takes were batched."""
    src = DepthSource(cfg())
    ev = feed(src, [(1000 * MS, 0, 5000.0), (1066 * MS, 1, 5066.0)])
    assert src.take(1500 * MS) == []  # quiet: nothing decided yet
    ev = feed(src, [(1600 * MS, 2, 5600.0)])
    [g] = gens_of(ev)
    assert g == {"t": "depth_gen", "why": "receive_gap", "gen": 2, "host_ns": 1366 * MS}
    [f] = frames_of(ev)
    assert f["verdict"] == "ok" and f["gen"] == 2 and src.stopped is None
    # the trailing gap of a stream that stopped: at the end
    src2 = DepthSource(cfg())
    feed(src2, [(1000 * MS, 0, 5000.0)])
    assert gens_of(src2.finish(2000 * MS)) == [{"t": "depth_gen", "why": "receive_gap", "gen": 2,
                                                "host_ns": 1300 * MS}]
    # a gap then a loss: the gap first, at its own time, then the loss
    src3 = DepthSource(cfg())
    feed(src3, [(1000 * MS, 0, 5000.0)])
    src3.lost("device_removed", at_ns=1800 * MS)
    assert [g["why"] for g in gens_of(src3.take(1900 * MS))] == ["receive_gap", "lost:device_removed"]


def test_a_frame_that_may_have_been_taken_before_a_boundary_is_never_ok():
    """Codex 7e-1 round 1 P2-3: the gap's boundary at 1.366 s, a frame received at 1.380 s with a 50 ms bound -- its
    interval [1.330, 1.380] starts before the boundary: before_boundary, its interval kept as it is."""
    src = DepthSource(cfg())
    feed(src, [(1000 * MS, 0, 5000.0), (1066 * MS, 1, 5066.0)])
    [f] = frames_of(feed(src, [(1380 * MS, 2, 5380.0)]))
    assert f["verdict"] == "before_boundary" and f["lo"] == 1330 * MS and f["gen"] == 2


@pytest.mark.parametrize("third,why", [((1132 * MS, 2, 5000.0), "restart"),
                                        ((1132 * MS, 2, 5132.0, "global_time"), "domain_changed")])
def test_a_device_clock_change_stops_the_source(third, why):
    src = DepthSource(cfg())
    ev = feed(src, [(1000 * MS, 0, 5000.0), (1066 * MS, 1, 5066.0), third, (1200 * MS, 3, 5200.0)])
    assert [g["why"] for g in gens_of(ev)] == [why] and src.stopped == why
    assert [f["verdict"] for f in frames_of(ev)] == ["ok", "ok", "stopped", "stopped"]


def test_the_same_device_time_again_is_a_duplicate():
    src = DepthSource(cfg())
    ev = feed(src, [(1000 * MS, 0, 5000.0), (1066 * MS, 1, 5000.0), (1132 * MS, 2, 5132.0)])
    assert [f["verdict"] for f in frames_of(ev)] == ["ok", "dup", "ok"] and not gens_of(ev)


def test_a_delay_that_creeps_breaks_the_cumulative_check_where_neighbours_pass():
    """Plan 2 (Codex plan round 2 P2-2): delays 0, 12, 24, 36 ms with a 30 ms bound -- each neighbour pair differs by
    12 ms, but the first and the last by 36 ms > 30 + 1 ms (+ 100 ppm of 200 ms = 0.02 ms)."""
    src = DepthSource(cfg(provisional_delay_bound_s=0.030))
    rows = [(1000 * MS + k * 66 * MS + k * 12 * MS, k, 5000.0 + k * 66.0) for k in range(4)]
    ev = feed(src, rows)
    assert [f["verdict"] for f in frames_of(ev)] == ["ok", "ok", "ok", "stopped"]
    assert [g["why"] for g in gens_of(ev)] == ["clock_mismatch"]


def test_a_full_queue_drops_the_oldest_and_stops_the_source():
    src = DepthSource(cfg(queue_max=3))
    for k in range(5):
        src.push(1000 * MS + k * 66 * MS, k, 5000.0 + k * 66.0, "hardware_clock", {}, 0.001, INTR, None)
    ev = src.take(1400 * MS)
    assert src.overflow == 2 and gens_of(ev)[0]["why"] == "overflow" and src.stopped == "overflow"
    assert [f["frame_number"] for f in frames_of(ev)] == [2, 3, 4]


def test_a_loss_is_a_final_boundary_after_the_frames_received_before_it():
    src = DepthSource(cfg())
    src.push(1000 * MS, 0, 5000.0, "hardware_clock", {}, 0.001, INTR, None)
    src.lost("device_removed", at_ns=1010 * MS)
    src.push(1066 * MS, 1, 5066.0, "hardware_clock", {}, 0.001, INTR, None)
    ev = src.take(1100 * MS)
    assert [e[0] for e in ev] == ["frame", "gen", "frame"]
    assert ev[1][1] == {"t": "depth_gen", "why": "lost:device_removed", "gen": 2, "host_ns": 1010 * MS}
    assert ev[2][1]["verdict"] == "stopped"


def test_the_record_rejudged_in_file_order_gives_the_same_rows():
    """The replay's verification: frames and losses fed again in record order reproduce every verdict, generation
    and interval, quiet gaps included (``finish`` at the record's last take)."""
    src = DepthSource(cfg())
    rows = []
    t = 1000 * MS
    for k in range(6):
        src.push(t, k, 5000.0 + (t - 1000 * MS) / 1e6, "hardware_clock", {}, 0.001, INTR, None)
        rows += src.take(t + 5 * MS)
        t += 66 * MS if k != 2 else 500 * MS  # a gap after the third frame
    rows += src.finish(t + 400 * MS)  # the end, with its trailing gap
    last = src.last_take_ns
    again = DepthSource(cfg())
    out = []
    for kind, r in rows:
        if kind == "frame":
            again.push(r["host_ns"], r["frame_number"], r["timestamp"], r["domain"], {}, 0.001, INTR, None)
            out += again.take(r["host_ns"])
    out += again.finish(last)
    strip = [(k, {kk: vv for kk, vv in r.items() if kk != "z16"}) for k, r in rows]
    assert [(k, {kk: vv for kk, vv in r.items() if kk != "z16"}) for k, r in out] == strip


# --- side files ---------------------------------------------------------------------------------------------------------
def test_frames_go_to_exclusive_side_files_in_chunks_and_read_back_byte_for_byte(tmp_path):
    rec = tmp_path / "obs.jsonl"
    folder = side_folder(rec)
    r = DepthRecorder(folder, chunk_frames=3)
    frames = [z16(v) for v in (0, 1, 65535, 7, 8)]
    frames[1][0, 0] = 65535  # boundary values inside one frame
    places = [r.add(f) for f in frames]
    assert [p[0] for p in places] == ["depth_0000.npz"] * 3 + ["depth_0001.npz"] * 2
    first = r.take_rows()
    assert len(first) == 1 and first[0]["name"] == "depth_0000.npz" and first[0]["frames"] == 3
    last = r.close()
    assert len(last) == 1 and last[0]["frames"] == 2 and r.failed is None
    for (name, key, sha), f in zip(places, frames):
        row = next(x for x in first + last if x["name"] == name)
        got = read_side_file(folder / name, row["sha256"])[0][key]
        assert got.dtype == np.uint16 and got.shape == f.shape and got.tobytes() == f.astype("<u2").tobytes()
        assert dc.frame_sha256(got) == sha
    with pytest.raises(FileExistsError):
        DepthRecorder(folder, chunk_frames=3)  # one record, one folder


def test_a_changed_side_file_is_refused_and_a_failed_write_marks_the_recorder(tmp_path, monkeypatch):
    r = DepthRecorder(tmp_path / "a.depth", chunk_frames=2)
    r.add(z16(1))
    r.add(z16(2))
    [row] = r.take_rows()
    p = tmp_path / "a.depth" / row["name"]
    data = bytearray(p.read_bytes())
    data[-10] ^= 1
    p.write_bytes(bytes(data))
    with pytest.raises(ValueError, match="sha256"):
        read_side_file(p, row["sha256"])
    # the last partial chunk's write fails at the end: the recorder says so (the record gets no .ok)
    r2 = DepthRecorder(tmp_path / "b.depth", chunk_frames=5)
    r2.add(z16(3))
    real_open = open

    def failing_open(path, mode="r", *a, **k):
        if str(path).endswith(".npz") and "x" in mode:
            raise OSError("disk full")
        return real_open(path, mode, *a, **k)

    monkeypatch.setattr("builtins.open", failing_open)
    assert r2.close() == [] and "disk full" in r2.failed
    with pytest.raises(ValueError, match="uint16"):
        r2.add(np.zeros((4, 4), dtype=np.int32))


def test_the_rows_do_not_depend_on_how_the_takes_were_batched():
    """Codex 7e-1 round 1 P2-2: four frames, a quiet stretch, a loss -- taken one by one or four at a time, the same
    rows (the observe tool takes at most four a cycle; the replay re-judges one at a time)."""
    def run(batch):
        src = DepthSource(cfg())
        items = [("f", 1000 * MS + k * 66 * MS, k) for k in range(4)] + [("l", 1900 * MS, None)]
        out = []
        for kind, ns, k in items:
            if kind == "f":
                src.push(ns, k, 5000.0 + k * 66.0, "hardware_clock", {}, 0.001, INTR, None)
            else:
                src.lost("device_removed", at_ns=ns)
            if batch == 1:
                out += src.take(ns + 1, 4)
        if batch != 1:
            out += src.take(2000 * MS, 4)
            out += src.take(2000 * MS, 4)
        out += src.finish(2100 * MS)
        return [(k, {x: v for x, v in r.items() if x != "z16"}) for k, r in out]

    assert run(1) == run(4)


def test_a_side_file_replaced_by_the_same_bytes_is_not_the_file_verified(tmp_path):
    r = DepthRecorder(tmp_path / "c.depth", chunk_frames=1)
    r.add(z16(5))
    [row] = r.take_rows()
    p = tmp_path / "c.depth" / row["name"]
    _arrays, pin = read_side_file(p, row["sha256"])
    data = p.read_bytes()
    p.unlink()
    p.write_bytes(data)  # a new inode with the same bytes
    with pytest.raises(ValueError, match="not the file verified"):
        read_side_file(p, row["sha256"], pin=pin)
