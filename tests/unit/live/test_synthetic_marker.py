"""Synthetic (rehearsal) records are marked where written and refused where measured
(docs/design/2026-11-15-arrival-day-rehearsal.md 2.3)."""

from __future__ import annotations

import os

import pytest

from forklift_core.live import synthetic as syn


def test_a_marked_or_pty_header_is_synthetic():
    assert syn.is_synthetic({"t": "header", "synthetic": {"bench": {}}})
    assert syn.is_synthetic({"t": "header", "device": "/dev/pts/4"})  # an unmarked rehearsal record
    assert not syn.is_synthetic({"t": "header", "device": "/dev/forklift-mcu"})
    assert not syn.is_synthetic({"t": "header", "device": "/dev/ttyACM0", "synthetic": None})
    # a pty named through a link: the header's device identity tells
    assert syn.is_synthetic({"t": "header", "device": "/dev/forklift-mcu", "device_id": {"major": 136, "minor": 4}})
    assert not syn.is_synthetic({"t": "header", "device": "/dev/forklift-mcu", "device_id": {"major": 166, "minor": 0}})


def test_measurement_entry_points_refuse_synthetic_unless_allowed():
    rows = [{"t": "header", "device": "/dev/pts/3"}, {"t": "state"}]
    with pytest.raises(syn.SyntheticRefused):
        syn.refuse_or_mark(rows, allow=False)
    assert syn.refuse_or_mark(rows, allow=True) == {"synthetic_inputs": True, "not_for_judgement": True}
    assert syn.refuse_or_mark([{"t": "header", "device": "/dev/ttyACM0"}, {"t": "state"}], allow=False) == {}
    # several inputs: one synthetic is enough
    assert syn.refuse_or_mark([{"t": "header", "device": "/dev/ttyACM0"}, {"t": "header", "synthetic": True}],
                              allow=True)["synthetic_inputs"] is True


def test_the_writer_marker_needs_a_bench_on_a_pty_and_refuses_one_elsewhere(monkeypatch):
    monkeypatch.setattr(syn, "_is_pty", lambda device: device.startswith("/dev/pts/"))
    with pytest.raises(syn.SyntheticRefused, match="needs --bench"):
        syn.writer_marker("/dev/pts/4", None)
    with pytest.raises(syn.SyntheticRefused, match="not a pty"):
        syn.writer_marker("/dev/ttyACM0", "bench.json")
    assert syn.writer_marker("/dev/ttyACM0", None) is None
    monkeypatch.setattr(syn, "_load_bench", lambda path: {"schema": "rehearsal_bench/1"})
    monkeypatch.setattr(syn.model_peer, "model_peer_refusal", lambda device, bench: "model_gone")
    with pytest.raises(syn.SyntheticRefused, match="model_gone"):
        syn.writer_marker("/dev/pts/4", "bench.json")
    monkeypatch.setattr(syn.model_peer, "model_peer_refusal", lambda device, bench: None)
    assert syn.writer_marker("/dev/pts/4", "bench.json") == {"bench": {"schema": "rehearsal_bench/1"}}


def test_is_pty_reads_the_device_major(tmp_path):
    m, s = os.openpty()
    try:
        assert syn._is_pty(os.ttyname(s))
    finally:
        os.close(m)
        os.close(s)
    f = tmp_path / "f"
    f.write_text("")
    assert not syn._is_pty(str(f)) and not syn._is_pty(str(tmp_path / "missing"))


def test_the_bench_opener_refuses_an_fd_on_another_device(tmp_path):
    m, sl = os.openpty()
    try:
        pts = os.ttyname(sl)
        bench = {"pts_rdev": os.stat(pts).st_rdev}
        plain = lambda path, baud=None: os.open(path, os.O_RDWR | os.O_NOCTTY)  # noqa: E731
        fd = syn.bench_opener(plain, bench)(pts)
        os.close(fd)
        with pytest.raises(OSError, match="not the bench"):
            syn.bench_opener(plain, bench)("/dev/null")
    finally:
        os.close(m)
        os.close(sl)
