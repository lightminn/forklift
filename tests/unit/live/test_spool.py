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
    assert manifest["samples"] == {"chunks": 2, "items": 10, "dropped_items": 0, "unsettled_items": 0}  # the tail at the close


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


def test_a_full_backlog_drops_the_cut_and_counts_it_without_waiting(tmp_path):
    items = []
    spool = RecordSpool(tmp_path, interval_s=0.0, queue_cuts=1)
    spool.pause_writer()  # the writer stalls (a slow disk)
    spool.register("x", items, keep=0)
    items.extend(range(4))
    spool.tick(1.0)
    items.extend(range(4))
    spool.tick(2.0)  # a second cut unwritten would pass 1: dropped, the list still emptied
    assert items == []
    spool.resume_writer()
    spool.close()
    assert json.loads((tmp_path / "manifest.json").read_text())["x"] == {"chunks": 1, "items": 4, "dropped_items": 4,
                                                                         "unsettled_items": 0}


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


def test_chunks_read_back_in_the_order_written(tmp_path):
    from forklift_core.live.spool import iter_chunks

    items = []
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool.register("x", items, keep=1)
    for start in (0, 10, 20):
        items.extend({"i": i} for i in range(start, start + 10))
        spool.tick(float(start + 1))
    spool.close()
    read = [item["i"] for chunk in iter_chunks(tmp_path, "x") for item in chunk]
    assert read == list(range(30))  # the last one with the close's cut


def test_a_group_loads_back_whole_with_its_tail(tmp_path):
    from forklift_core.live.spool import load_group

    log = {"stamps": [], "poses": []}
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool.register_group("slam_log.joint", log, keep=1)
    for i in range(12):
        log["stamps"].append(float(i))
        log["poses"].append(np.full(3, i, dtype=np.float32))
        if i % 5 == 4:
            spool.tick(float(i))
    spool.close()
    whole = load_group(tmp_path, "slam_log.joint", tail=log)
    assert whole["stamps"].tolist() == [float(i) for i in range(12)]
    assert whole["poses"].shape == (12, 3) and whole["poses"][:, 0].tolist() == list(range(12))


def test_nothing_spooled_reads_as_no_chunks(tmp_path):
    from forklift_core.live.spool import iter_chunks, load_group

    assert list(iter_chunks(tmp_path, "never")) == []
    assert load_group(tmp_path, "never", tail={"a": [1.0, 2.0]})["a"].tolist() == [1.0, 2.0]


def test_close_writes_the_tail_too_so_the_disk_holds_everything(tmp_path):
    from forklift_core.live.spool import iter_chunks, load_group

    items, log = [], {"a": [], "b": []}
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool.register("x", items, keep=1)
    spool.register_group("g", log, keep=1)
    for i in range(12):
        items.append(i)
        log["a"].append(float(i)), log["b"].append(float(-i))
        if i == 8:
            spool.tick(1.0)
    manifest = spool.close()
    assert items == [] and log == {"a": [], "b": []}
    assert [v for chunk in iter_chunks(tmp_path, "x") for v in chunk] == list(range(12))
    assert load_group(tmp_path, "g")["a"].tolist() == [float(i) for i in range(12)]
    assert manifest["complete"] is True and manifest["x"]["items"] == 12


def test_a_group_out_of_step_at_the_end_is_reported(tmp_path):
    log = {"a": [1.0, 2.0, 3.0], "b": [1.0, 2.0]}
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool.register_group("g", log, keep=1)
    manifest = spool.close()
    assert manifest["g"]["misaligned"] == {"a": 3, "b": 2} and manifest["complete"] is False


def test_a_write_still_running_at_the_close_deadline_is_not_counted_as_written(tmp_path):
    import threading

    from forklift_core.live import spool as spool_module

    entered, release = threading.Event(), threading.Event()
    real = spool_module.RecordSpool._write

    def slow(self, name, kind, payload):
        entered.set()
        release.wait(5.0)
        real(self, name, kind, payload)

    items = []
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool._write = slow.__get__(spool)
    spool.register("x", items, keep=0)
    items.extend(range(3))
    spool.tick(1.0)
    assert entered.wait(5.0)
    manifest = spool.close(timeout_s=0.05)
    assert manifest["complete"] is False and manifest["x"]["unsettled_items"] == 3
    release.set()
    spool._thread.join(5.0)
    saved = json.loads((tmp_path / "manifest.json").read_text())
    assert saved["complete"] is False  # a late finish does not make the record whole after the fact


def test_a_chunk_appears_only_when_written_whole(tmp_path):
    from forklift_core.live.spool import iter_chunks

    (tmp_path / "x").mkdir()
    (tmp_path / "x" / "000009.jsonl.tmp").write_text('{"half": ')  # a write cut off by a crash
    items = []
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool.register("x", items, keep=0)
    items.extend(range(2))
    spool.close()
    assert [v for chunk in iter_chunks(tmp_path, "x") for v in chunk] == [0, 1]


def test_errors_are_counted_and_only_the_last_few_kept(tmp_path):
    items = []
    (tmp_path / "x").write_text("a file where the folder would be")
    spool = RecordSpool(tmp_path, interval_s=0.0, queue_cuts=100)  # every cut reaches the failing writer
    spool.register("x", items, keep=0)
    for i in range(30):
        items.append(i)
        spool.tick(float(i + 1))
    manifest = spool.close()
    assert manifest["error_count"] == 30 and len(manifest["errors"]) == 10


def test_the_cut_reports_its_own_cost(tmp_path):
    items = []
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool.register("x", items, keep=1)
    items.extend(range(1000))
    spool.tick(1.0)
    manifest = spool.close()
    assert manifest["cuts"] == 1 and 0.0 <= manifest["cut_wall_max_s"] < 0.5


def test_a_runs_slam_log_loads_from_its_npz_or_from_the_spool_meta_points_at(tmp_path):
    from forklift_core.live.spool import load_slam_log

    batch = tmp_path / "batch"
    batch.mkdir()
    np.savez_compressed(batch / "slam_log.npz", joint_stamps_s=np.arange(3.0), scan_stamps_s=np.arange(2.0))
    assert load_slam_log(batch)["joint_stamps_s"].tolist() == [0.0, 1.0, 2.0]

    live = tmp_path / "live"
    log = {"joint_stamps_s": [], "scan_stamps_s": [], "silenced_scan_stamps_s": []}
    spool = RecordSpool(live / "spool", interval_s=0.0)
    spool.register_group("slam_log.joint", {"joint_stamps_s": log["joint_stamps_s"]}, keep=1)
    spool.register_group("slam_log.scan", {"scan_stamps_s": log["scan_stamps_s"]}, keep=1)
    spool.register("slam_log.silenced", log["silenced_scan_stamps_s"], keep=1)
    for i in range(5):
        log["joint_stamps_s"].append(float(i))
        if i % 2 == 0:
            log["scan_stamps_s"].append(float(i))
        spool.tick(float(i))
    spool.close()
    (live / "meta.json").write_text(json.dumps({"spool": {"joint": "slam_log.joint", "scan": "slam_log.scan",
                                                          "silenced": "slam_log.silenced", "root": "spool"}}))
    loaded = load_slam_log(live)
    assert loaded["joint_stamps_s"].tolist() == [0.0, 1.0, 2.0, 3.0, 4.0]
    assert loaded["scan_stamps_s"].tolist() == [0.0, 2.0, 4.0]
    assert loaded["silenced_scan_stamps_s"].tolist() == []


def test_the_backlog_is_bounded_in_cuts_whatever_the_records_sizes(tmp_path):
    """Codex D8 design round 2 P2-2: grid records vary in size (occupied cells), so the bound is not a size estimate
    but a count of cuts -- each one interval's records -- not yet written."""
    grids, scans = [], {"ranges": []}
    spool = RecordSpool(tmp_path, interval_s=0.0, queue_cuts=2)
    spool.pause_writer()
    spool.register("grids", grids, keep=0)
    spool.register_group("scan", scans, keep=0)
    for i in range(4):
        grids.append((float(i), np.zeros((9_500 if i % 2 else 0, 2), np.int32)))
        scans["ranges"].append(np.zeros(1600, np.float32))
        spool.tick(float(i + 1))
    spool.resume_writer()
    manifest = spool.close()
    assert manifest["grids"]["items"] == 2 and manifest["grids"]["dropped_items"] == 2
    assert manifest["scan"]["items"] == 2 and manifest["scan"]["dropped_items"] == 2  # a cut goes, or not, whole
    assert manifest["complete"] is False


def test_a_second_close_returns_the_first_manifest(tmp_path):
    items = []
    spool = RecordSpool(tmp_path, interval_s=0.0)
    spool.register("x", items, keep=0)
    items.extend(range(3))
    first = spool.close()
    items.extend(range(5))
    assert spool.close() == first
    assert json.loads((tmp_path / "manifest.json").read_text()) == first


def _live_run(tmp_path, *, drop=False):
    run = tmp_path / "live"
    log = {"joint_stamps_s": [], "scan_stamps_s": [], "silenced_scan_stamps_s": []}
    samples = []
    spool = RecordSpool(run / "spool", interval_s=0.0, queue_cuts=1 if drop else 100)
    spool.register_group("slam_log.joint", {"joint_stamps_s": log["joint_stamps_s"]}, keep=1)
    spool.register_group("slam_log.scan", {"scan_stamps_s": log["scan_stamps_s"]}, keep=1)
    spool.register("slam_log.silenced", log["silenced_scan_stamps_s"], keep=1)
    spool.register("state.samples", samples, keep=1)
    if drop:
        spool.pause_writer()
    for i in range(4):
        log["joint_stamps_s"].append(float(i)), log["scan_stamps_s"].append(float(i))
        samples.append({"time_s": float(i)})
        spool.tick(float(i))
    spool.resume_writer()
    spool.close()
    (run / "meta.json").write_text(json.dumps({"spool": {"joint": "slam_log.joint", "scan": "slam_log.scan",
                                                         "silenced": "slam_log.silenced", "root": "spool"}}))
    return run


def test_an_incomplete_spooled_log_is_refused_unless_asked_for(tmp_path):
    import pytest

    from forklift_core.live.spool import load_slam_log

    run = _live_run(tmp_path, drop=True)
    with pytest.raises(ValueError, match="incomplete"):
        load_slam_log(run)
    assert len(load_slam_log(run, allow_incomplete=True)["joint_stamps_s"]) < 4


def test_a_spooled_list_loads_whole(tmp_path):
    from forklift_core.live.spool import load_list

    run = _live_run(tmp_path)
    assert [s["time_s"] for s in load_list(run / "spool", "state.samples")] == [0.0, 1.0, 2.0, 3.0]


def test_a_rolled_live_log_reads_oldest_first_and_a_lost_part_is_refused(tmp_path):
    import pytest

    from forklift_core.live.spool import read_live_log

    for name, rows in (("live_log.jsonl.2", [1]), ("live_log.jsonl.1", [2]), ("live_log.jsonl", [3])):
        (tmp_path / name).write_text("".join(json.dumps({"i": i}) + "\n" for i in rows))
    (tmp_path / "live_log.index.json").write_text(json.dumps({"rotations": 2, "keep": 3}))
    assert [e["i"] for e in read_live_log(tmp_path)] == [1, 2, 3]
    (tmp_path / "live_log.index.json").write_text(json.dumps({"rotations": 5, "keep": 3}))
    with pytest.raises(ValueError, match="rolled"):
        read_live_log(tmp_path)
    plain = tmp_path / "plain"
    plain.mkdir()
    (plain / "live_log.jsonl").write_text('{"i": 1}\n')
    assert [e["i"] for e in read_live_log(plain)] == [1]  # a run that never rolled over has no index


def test_a_copy_missing_chunks_or_a_rolled_part_is_refused(tmp_path):
    """Codex D8 round 3 P3-1: the manifest says what was written; a record copied without some of it is not whole."""
    import pytest

    from forklift_core.live.spool import load_list, load_slam_log, read_live_log

    run = _live_run(tmp_path)
    next(iter(sorted((run / "spool" / "state.samples").iterdir()))).unlink()
    with pytest.raises(ValueError, match="missing"):
        load_list(run / "spool", "state.samples")
    next(iter(sorted((run / "spool" / "slam_log.joint").iterdir()))).unlink()
    with pytest.raises(ValueError, match="missing"):
        load_slam_log(run)
    logs = tmp_path / "logs"
    logs.mkdir()
    (logs / "live_log.jsonl.1").write_text('{"i": 2}\n')
    (logs / "live_log.jsonl").write_text('{"i": 3}\n')
    (logs / "live_log.index.json").write_text(json.dumps({"rotations": 2, "keep": 3}))  # .2 is not there
    with pytest.raises(ValueError, match="missing"):
        read_live_log(logs)
