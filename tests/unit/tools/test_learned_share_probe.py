"""A3 P0 probe (tools/learned_share_probe.py): the reply check against the serial reference and the pre-registered
verdict, on synthetic rows (the probe itself runs on a GPU)."""
import numpy as np

from forklift_core.perception.instance_proposals import InstanceProposal
from tools import learned_share_probe as P


def prop(i, score, rows=4):
    m = np.zeros((10, 10), bool)
    m[:rows] = True
    return InstanceProposal(i, m, score, "learned")


def test_a_reply_matches_its_reference_within_the_registered_tolerances():
    ref = [prop("l0", 0.8), prop("l1", 0.3)]
    assert P.same([prop("l0", 0.8005), prop("l1", 0.3)], ref) is None
    assert "ids" in P.same([prop("l1", 0.3), prop("l0", 0.8)], ref)
    assert "score" in P.same([prop("l0", 0.81), prop("l1", 0.3)], ref)
    assert "IoU" in P.same([prop("l0", 0.8, rows=3), prop("l1", 0.3)], ref)  # 30/40 = 0.75 < 0.99


def _rows(mode, phases_sm, server_mib=900, cpu_per_s=1.0, extra=()):
    rows, t = [], 0.0
    for label, sm in [*phases_sm.items(), *extra]:
        for k in range(3):
            rows.append({"t": t, "phase": f"{mode}_{label}", "apps": {"42": server_mib},
                         "sm": None if sm is None else {"42": 50.0, "99": sm},
                         "procs": {"server": {"cpu_s": cpu_per_s * t, "hwm_kib": 2 * 2 ** 20, "rss_kib": 1, "pid": 42}}})
            t += 1.0
    return rows


def _phase(label, rtts, expected=None):
    rows = [{"n": i, "rtt_ms": r, "error": None, "mismatch": None} for i, r in enumerate(rtts)]
    return {"label": label, "rows": rows, "expected": len(rtts) if expected is None else expected, "hung": 0}


def _phases(mode="serial", b=90):
    return [(_phase(f"{mode}_A{j}", [50] * 20), _phase(f"{mode}_B{j}", [b] * 20)) for j in range(4)]


SM = {f"{s}{j}": 10.0 for j in range(4) for s in "AB"}


def test_the_verdict_follows_the_registered_rules():
    r = P.judge("serial", _rows("serial", SM), 500.0, 42, _phases())
    assert r["delta_mib"] == 400 and r["kept_pairs"] == 4 and r["verdict"]["latency_ok"] is True
    assert r["verdict"]["delta_ok"] and r["verdict"]["correct_ok"] and r["verdict"]["pass"]
    assert abs(r["server_cores_b"] - 1.0) < 1e-9  # CPU time over the wall between the same samples
    phases = _phases()
    phases[0] = (_phase("serial_A0", [50] * 20), _phase("serial_B0", [120] * 20))  # B over twice A in a kept pair
    assert P.judge("serial", _rows("serial", SM), 500.0, 42, phases)["verdict"]["latency_ok"] is False


def test_pairs_with_off_other_load_are_left_out_and_too_few_is_undecided():
    sm = dict(SM, B0=60.0, A1=60.0)  # the median stays 10: pairs 0 and 1 go
    r = P.judge("serial", _rows("serial", sm), 500.0, 42, _phases())
    assert r["kept_pairs"] == 2 and r["verdict"]["latency_ok"] is None and r["verdict"]["pass"] is False


def test_the_faults_phase_is_not_in_the_median():
    # Codex: a loud faults phase must not move the reference and save a pair
    sm = dict(SM, A0=4.8, B1=15.7)
    base = P.judge("serial", _rows("serial", sm), 500.0, 42, _phases())
    loud = P.judge("serial", _rows("serial", sm, extra=[("faults", 100.0)] * 3), 500.0, 42, _phases())
    assert base["kept_pairs"] == loud["kept_pairs"] and base["other_sm_median"] == loud["other_sm_median"]


def test_missing_measurements_are_undecided_not_a_pass():
    # Codex: a failed nvidia-smi, a server never seen, an own worker never seen
    r = P.judge("serial", _rows("serial", SM), 0.0, 42, _phases())
    assert r["verdict"]["delta_ok"] is None and r["verdict"]["pass"] is False
    r = P.judge("serial", _rows("serial", SM), 500.0, 7, _phases())  # another pid: the server never seen
    assert r["delta_mib"] is None and r["verdict"]["delta_ok"] is None
    sm = dict(SM, A2=None)
    r = P.judge("serial", _rows("serial", sm), 500.0, 42, _phases())
    assert r["pairs"][2]["excluded"] is True  # a phase without SM samples leaves its pair out


def test_a_client_that_never_ran_fails_the_correctness_verdict():
    # Codex: a phase run with three clients instead of four must not pass
    phases = _phases()
    phases[1] = (phases[1][0], _phase("serial_B1", [90] * 15, expected=20))
    r = P.judge("serial", _rows("serial", SM), 500.0, 42, phases)
    assert r["short_calls"] == 5 and r["verdict"]["correct_ok"] is False


def test_too_much_vram_over_the_own_worker_fails():
    r = P.judge("serial", _rows("serial", SM, server_mib=1300), 500.0, 42, _phases(b=60))
    assert r["delta_mib"] == 800 and r["verdict"]["delta_ok"] is False


def test_a_failed_query_or_an_unknown_load_is_a_missing_sample(monkeypatch):
    # Codex: exit 1 with empty output, and "-" for another process, must not read as an idle GPU
    import subprocess
    import types

    outputs = iter([types.SimpleNamespace(returncode=1, stdout=""),
                    types.SimpleNamespace(returncode=0, stdout="7, 900\n"),
                    types.SimpleNamespace(returncode=0, stdout="# gpu pid type sm\n0 7 C 40\n0 99 C -\n")])
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: next(outputs))
    s = P.Sampler("0", period_s=0.0)
    s.stop.set()
    s.stop.is_set = iter([False, False, True]).__next__  # two samples, then stop
    s.run()
    assert s.rows[0]["sm"] is None and "error" in s.rows[0]
    assert s.rows[1]["apps"] == {"7": 900} and s.rows[1]["sm_unknown"] == ["99"]
    rows = _rows("serial", SM)
    for r in rows:
        if r["phase"] == "serial_A1":
            r["sm_unknown"] = ["99"]
    r = P.judge("serial", rows, 500.0, 42, _phases())
    assert r["pairs"][1]["excluded"] is True and r["kept_pairs"] == 3


def test_the_gpu_is_numbered_as_nvidia_smi_inside_the_job_sees_it(monkeypatch):
    # P0 12080: inside the job nvidia-smi lists only the job's GPU, as 0; the node-wide SLURM_JOB_GPUS=1 failed
    import subprocess
    import types

    monkeypatch.setenv("SLURM_JOB_GPUS", "1")
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: types.SimpleNamespace(stdout="GPU 0: RTX (UUID: x)\n"))
    assert P.gpu_index() == "0"
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: types.SimpleNamespace(stdout="GPU 0: a\nGPU 1: b\n"))
    assert P.gpu_index() == "1"


def test_the_expected_calls_follow_the_ticks():
    assert P.expected_calls(2.0, 90.0) == 180 and P.expected_calls(2.0, 0.4) == 1
