"""PR 7 7e-1 (docs/plans/2026-11-26-7e-depth-pocket-check.md 4.2-4.4): a record with depth is checked against its
side files -- every frame byte for byte through its sha256, its units, its calibration id recomputed from its own
intrinsics, its verdicts re-judged in file order -- and a side file changed after the check is caught before a reader
uses it. On synthetic records made by the observe tool's own code paths (tests/fixtures/replay/make_record.py)."""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

import pytest

from forklift_core.live import replay_record as rr

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tests" / "fixtures" / "replay"))
import make_record as mk  # noqa: E402

from test_replay_sources import load, rewrite  # noqa: E402


@pytest.fixture(scope="module")
def made(tmp_path_factory):
    p = tmp_path_factory.mktemp("depth") / "obs.jsonl"
    mk.write_record(p, depth=True)
    return p


def copy_record(made, tmp_path):
    tmp_path.mkdir(parents=True, exist_ok=True)
    dst = tmp_path / "obs.jsonl"
    shutil.copy(made, dst)
    shutil.copy(str(made) + ".ok", str(dst) + ".ok")
    shutil.copytree(str(made) + ".depth", str(dst) + ".depth")
    return dst


def refused(path):
    with pytest.raises(rr.ReplayRefused) as err:
        load(path)
    return err.value


def test_a_record_with_depth_is_verified_frame_by_frame_and_indexed(made):
    rec = load(made)
    rows = [json.loads(x) for x in Path(made).read_text().splitlines()]
    depth = [r for r in rows if r["t"] == "depth"]
    assert depth and rec.counts["depth_frames_verified"] == len(depth) == rec.counts["depth_ok"]
    assert len(rec.depth) == len(depth) and set(rec.depth_files) == {r["name"] for r in rows if r["t"] == "depth_file"}
    rr.check_depth_pins(rec)  # nothing changed since
    # the writer carries every key the replay reads, for the depth kinds too
    for r in rows:
        want = rr.READS.get(r["t"])
        if want is not None and r["t"] in rr.DEPTH_KINDS:
            assert want <= set(r), (r["t"], want - set(r))
    assert {r["t"] for r in rows} >= {"depth", "depth_file", "depth_profile"}


def edit_first_depth(key, value):
    def edit(rows):
        r = next(x for x in rows if x["t"] == "depth")
        if key == "intrinsics.fx":
            r["intrinsics"] = {**r["intrinsics"], "fx": r["intrinsics"]["fx"] + 0.01}  # calib_id left as it was
        else:
            r[key] = value
        return rows
    return edit


@pytest.mark.parametrize("key,value,why", [
    ("units", 0.002, "units"),
    ("intrinsics.fx", None, "calib_id"),
    ("verdict", "dup", "re-judged"),
    ("lo", 0, "re-judged"),
])
def test_a_row_that_does_not_match_its_frame_or_its_judgement_is_refused(made, tmp_path, key, value, why):
    p = copy_record(made, tmp_path)
    rewrite(p, edit_first_depth(key, value))
    err = refused(p)
    assert err.reason == "replay_depth_mismatch" and why in str(err), err


def test_a_side_file_changed_or_missing_is_refused(made, tmp_path):
    p = copy_record(made, tmp_path / "a")
    f = sorted((Path(str(p) + ".depth")).iterdir())[0]
    data = bytearray(f.read_bytes())
    data[len(data) // 2] ^= 1
    f.write_bytes(bytes(data))
    assert refused(p).reason == "replay_depth_mismatch"
    p2 = copy_record(made, tmp_path / "b")
    sorted((Path(str(p2) + ".depth")).iterdir())[-1].unlink()
    assert refused(p2).reason == "replay_depth_mismatch"


def test_a_side_file_changed_after_the_check_is_caught_before_it_is_read(made, tmp_path):
    p = copy_record(made, tmp_path)
    rec = load(p)
    f = Path(str(p) + ".depth") / sorted(rec.depth_files)[0]
    f.write_bytes(f.read_bytes())  # the same bytes, a new file: not the file verified
    with pytest.raises(rr.ReplayRefused, match="changed since"):
        rr.check_depth_pins(rec)


def test_depth_rows_without_a_depth_header_are_refused(made, tmp_path):
    p = copy_record(made, tmp_path)

    def edit(rows):
        rows[0]["depth"] = None
        return rows

    rewrite(p, edit)
    assert refused(p).reason == "replay_depth_mismatch"


def _edit_rows(made, tmp_path, edit):
    p = copy_record(made, tmp_path)
    rewrite(p, edit)
    return p


def test_keys_must_be_one_to_one_and_each_row_a_uint16_frame(made, tmp_path):
    """Codex 7e-1 round 1 P2-4: two rows naming the same array (one array left unreferenced), a row's dtype, a profile
    without a finite depth scale."""
    def same_key(rows):
        d = [r for r in rows if r["t"] == "depth"]
        d[1]["key"], d[1]["sha256"] = d[0]["key"], d[0]["sha256"]
        return rows

    assert "one to one" in str(refused(_edit_rows(made, tmp_path / "k", same_key)))

    def dtype(rows):
        next(r for r in rows if r["t"] == "depth")["dtype"] = "float32"
        return rows

    assert refused(_edit_rows(made, tmp_path / "d", dtype)).reason == "replay_depth_mismatch"

    def scale(rows):
        next(r for r in rows if r["t"] == "depth_profile")["depth_scale"] = None
        return rows

    assert "depth_scale" in str(refused(_edit_rows(made, tmp_path / "s", scale)))


@pytest.mark.parametrize("key,value", [("overflow", 1), ("bad_frames", 1), ("callback_errors", 2),
                                       ("record_failed", "depth_0001.npz: OSError")])
def test_an_end_that_says_raw_frames_were_lost_is_refused(made, tmp_path, key, value):
    def edit(rows):
        rows[-1]["depth"][key] = value
        return rows

    assert "lost" in str(refused(_edit_rows(made, tmp_path, edit))) or "not written whole" in str(
        refused(_edit_rows(made, tmp_path / "again", edit)))


def test_depth_rows_do_not_lengthen_the_record():
    """Codex 7e-1 round 1 P2-7: the replay does not consume depth yet, so depth rows past the last scan or IMU row do
    not count toward REPLAY_MIN_S."""
    import inspect

    src = inspect.getsource(rr.load_replay_record)
    block = src[src.index("elif t in DEPTH_KINDS"):src.index("depth_ver.row(r)")]
    assert "last_ns" not in block


def test_the_file_list_and_the_end_are_checked_even_without_frames_and_names_stay_in_the_folder(made, tmp_path):
    """Codex 7e-1 round 2 P2-3: every depth frame and generation row removed (the side files then unreferenced, the
    end's overflow set), a side file declared twice, a name outside the folder's form -- each refused."""
    def no_frames(rows):
        rows = [r for r in rows if r["t"] not in ("depth", "depth_gen")]
        rows[-1]["depth"]["overflow"] = 1
        return rows

    assert refused(_edit_rows(made, tmp_path / "a", no_frames)).reason == "replay_depth_mismatch"

    def twice(rows):
        i = next(i for i, r in enumerate(rows) if r["t"] == "depth_file")
        rows.insert(i + 1, dict(rows[i], sha256="0" * 64))
        return rows

    assert "twice" in str(refused(_edit_rows(made, tmp_path / "b", twice)))

    def outside(rows):
        f = next(r for r in rows if r["t"] == "depth_file")
        for r in rows:
            if r["t"] == "depth" and r["npz"] == f["name"]:
                r["npz"] = "../elsewhere.npz"
        rows[-1]["depth"]["files"] = ["../elsewhere.npz" if n == f["name"] else n for n in rows[-1]["depth"]["files"]]
        f["name"] = "../elsewhere.npz"
        return rows

    assert "folder" in str(refused(_edit_rows(made, tmp_path / "c", outside)))


def test_side_files_declared_with_no_depth_row_are_refused_and_long_records_keep_their_names(made, tmp_path):
    """Codex 7e-1 round 3 P2-3/P2-4: files declared in the rows and the end with no frame row; a file name past four
    digits (the recorder's ``:04d`` grows) is still the recorder's own."""
    def no_rows(rows):
        rows = [r for r in rows if r["t"] not in ("depth", "depth_gen")]
        rows[-1]["depth"]["frames"] = 0
        return rows

    assert "no depth row" in str(refused(_edit_rows(made, tmp_path / "a", no_rows)))
    assert rr._SIDE_NAME.fullmatch("depth_10000.npz") and not rr._SIDE_NAME.fullmatch("depth_10.npz")
