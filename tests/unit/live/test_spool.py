"""The live session's record spool (persistent console plan D8): what keeps growing goes to disk in chunks, the last
items stay for the readers that use them; nothing moved is changed afterwards."""

import json

import numpy as np

from forklift_core.live.spool import RecordSpool


def test_old_items_move_to_chunks_and_the_tail_stays(tmp_path):
    samples = []
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool.register("samples", samples, keep=2)
    for i in range(10):
        samples.append({"i": i})
    spool.tick(1.0)
    assert samples == [{"i": 8}, {"i": 9}]
    spool.close()
    chunk = (tmp_path / "samples" / "000001.jsonl").read_text().splitlines()
    assert [json.loads(line)["i"] for line in chunk] == list(range(8))
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["samples"] == {"chunks": 1, "items": 8, "dropped_items": 0}


def test_a_group_moves_together_so_its_lists_stay_aligned(tmp_path):
    log = {"stamps": [], "poses": []}
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool.register_group("slam_log", log, keep=1)
    for i in range(5):
        log["stamps"].append(float(i))
        log["poses"].append(np.full(3, i, dtype=np.float32))
    spool.tick(1.0)
    assert log["stamps"] == [4.0] and len(log["poses"]) == 1
    spool.close()
    with np.load(tmp_path / "slam_log" / "000001.npz") as data:
        assert data["stamps"].tolist() == [0.0, 1.0, 2.0, 3.0] and data["poses"].shape == (4, 3)


def test_nothing_moves_before_the_interval(tmp_path):
    items = []
    spool = RecordSpool(tmp_path, interval_s=10.0)
    spool.register("x", items, keep=1)
    items.extend(range(100))
    spool.tick(5.0)
    assert len(items) == 100
    spool.tick(10.0)
    assert items == [99]
    spool.close()


def test_a_full_queue_drops_the_chunk_and_counts_it_without_waiting(tmp_path):
    items = []
    spool = RecordSpool(tmp_path, interval_s=0.0, queue_items=5)
    spool.pause_writer()  # the writer stalls (a slow disk)
    spool.register("x", items, keep=0)
    items.extend(range(4))
    spool.tick(1.0)
    items.extend(range(4))
    spool.tick(2.0)  # 8 queued would pass 5: dropped, the list still emptied
    assert items == []
    spool.resume_writer()
    spool.close()
    assert json.loads((tmp_path / "manifest.json").read_text())["x"] == {"chunks": 1, "items": 4, "dropped_items": 4}


def test_a_failed_write_is_counted(tmp_path):
    items = []
    (tmp_path / "x").write_text("a file where the folder would be")
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool.register("x", items, keep=0)
    items.extend(range(3))
    spool.tick(1.0)
    spool.close()
    manifest = json.loads((tmp_path / "manifest.json").read_text())
    assert manifest["x"]["dropped_items"] == 3 and manifest["errors"]
