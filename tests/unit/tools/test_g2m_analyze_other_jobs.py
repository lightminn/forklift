"""Real-time plan 5.22 correction (2026-11-09): a measurement session is invalid when any other job ran on the node
beside it -- RUNNING in a start/node/end squeue snapshot, or (given sacct) an allocation overlapping the session's
monitor window. Pending jobs and the session's own bundle job do not count; a list's warm-up never enters a set."""

import importlib.util
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "g2m_analyze", ROOT / "docs" / "validation" / "2026-11-09-g2-measure" / "g2m_analyze.py")
g2m = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(g2m)


def test_running_others_counts_other_running_jobs_only():
    text = ("17015 ksw sequoia_all RUNNING N/A 8\n"
            "17010 lightminn rtb_g2m RUNNING gres/shard:4 4\n"
            "17020 lightminn l4r PENDING N/A 4\n"
            "17019_3 lightminn arr RUNNING N/A 2\n"
            "   CfgTRES=cpu=20,mem=58000M\n")
    assert g2m.running_others(text, "17010") == ["17015 ksw sequoia_all RUNNING", "17019_3 lightminn arr RUNNING"]
    assert g2m.running_others("17010 lightminn rtb_g2m RUNNING gres/shard:4 4\n", "17010") == []


def test_sacct_overlap_includes_a_job_that_came_and_went_and_a_running_one():
    text = ("17010|lightminn|2026-11-09T15:12:29|2026-11-09T16:26:30\n"
            "17015|ksw|2026-11-09T15:55:47|Unknown\n"
            "17016|ksw|2026-11-09T16:02:00|2026-11-09T16:05:00\n"
            "17030|lightminn|Unknown|Unknown\n"
            "17007|lightminn|2026-11-09T14:59:00|2026-11-09T15:11:04\n")
    jobs = g2m.read_sacct(text)
    assert [j[0] for j in jobs] == ["17010", "17015", "17016", "17007"]  # a never-started job cannot overlap
    ts = {j[0]: j for j in jobs}
    t0, t1 = ts["17016"][2] - 120, ts["17016"][3] + 120  # a session around 17016, after 17015 started
    assert g2m.overlapping(jobs, "17010", t0, t1) == ["17015 (ksw)", "17016 (ksw)"]
    early0, early1 = ts["17007"][3] + 60, ts["17007"][3] + 300  # after 17007, before 17015
    assert g2m.overlapping(jobs, "17010", early0, early1) == []


def _session_dir(tmp_path, squeue_end, walls):
    out = tmp_path / "g2mf_01_F_on"
    (out / "monitor").mkdir(parents=True)
    (out / "args.txt").write_text("BUNDLE_JOB=17010\nSET=far\n")
    (out / "squeue_end.txt").write_text(squeue_end)
    (out / "monitor" / "host.jsonl").write_text(
        "\n".join(json.dumps({"wall": w, "gpu_status": "ok", "gpu_procs": []}) for w in walls) + "\n")
    return out


def test_the_session_check_reads_snapshots_and_sacct(tmp_path):
    s = g2m.Session.__new__(g2m.Session)  # only the overlap check, without a run's trace and logs
    s.out = _session_dir(tmp_path, "17010 lightminn rtb_g2m RUNNING gres/shard:4 4\n", [1000.0, 1500.0])
    s.cond = g2m.read_conditions(s.out)
    s.invalid = []
    s._check_other_jobs(None)
    assert s.invalid == []
    s._check_other_jobs([("17010", "lightminn", 0.0, float("inf")), ("17016", "ksw", 1200.0, 1300.0)])
    assert s.invalid == ["another job on the node: sacct: 17016 (ksw)"]
    s.invalid = []
    (s.out / "squeue_end.txt").write_text("17015 ksw sequoia_all RUNNING N/A 8\n")
    s._check_other_jobs([("17015", "ksw", 2000.0, float("inf"))])  # started after the session: only the snapshot
    assert s.invalid == ["another job on the node: squeue_end.txt: 17015 ksw sequoia_all RUNNING"]


def test_a_warm_up_never_enters_a_set(tmp_path, monkeypatch):
    entries = [{"name": "g2mf_00_F_warm", "env": {"SET": "far", "INSTR": "off"}},
               {"name": "g2mf_01_F_on", "env": {"SET": "far", "INSTR": "on"}}]
    lst = tmp_path / "l.jsonl"
    lst.write_text("\n".join(json.dumps(e) for e in entries) + "\n")
    seen = {}
    monkeypatch.setattr(g2m, "m7_verdict", lambda rows, names: seen.setdefault("names", names) or {})
    monkeypatch.setattr("sys.argv", ["g2m_analyze.py", str(tmp_path / "none"), "--list", str(lst)])
    g2m.main()
    assert seen["names"] == {"H": [], "F": ["g2mf_01_F_on"]}


def test_ps_rule_counts_samples_over_one_core_inside_the_window():
    samples = [{"wall": 100.0 + 10 * i, "other_cores": c} for i, c in enumerate([0.0, 0.2, 1.5] + [0.1] * 17)]
    r = g2m.ps_rule(samples, 95.0, 300.0)  # 20 samples, one over a core: 5 %, not over
    assert (r["samples"], r["over_1_core"], r["share"], r["invalid"]) == (20, 1, 0.05, False)
    r = g2m.ps_rule(samples, 95.0, 200.0)  # 11 samples, the same one: 9 %
    assert r["invalid"] is True
    assert g2m.ps_rule(samples, 1000.0, 2000.0)["invalid"] is True  # no sample: not shown to be clean


def test_with_the_cpu_rule_other_jobs_are_recorded_not_judged(tmp_path):
    s = g2m.Session.__new__(g2m.Session)
    s.out = _session_dir(tmp_path, "17015 ksw sequoia_all RUNNING N/A 8\n", [1000.0, 1100.0])
    s.cond = g2m.read_conditions(s.out)
    s.invalid = []
    s._check_other_jobs(None, judge=False)
    assert s.invalid == [] and s.other_jobs == ["squeue_end.txt: 17015 ksw sequoia_all RUNNING"]
    s._check_ps([{"wall": 1000.0 + 10 * i, "other_cores": 0.02} for i in range(11)])
    assert s.invalid == [] and s.ps["samples"] == 11
    s._check_ps([{"wall": 1000.0 + 10 * i, "other_cores": 2.0 if i < 2 else 0.0} for i in range(11)])
    assert len(s.invalid) == 1 and s.invalid[0].startswith("other users' CPU over 1 core")


def test_the_sampler_differences_ticks_per_user_and_skips_first_seen():
    spec = importlib.util.spec_from_file_location(
        "g2m_cpu_sampler", ROOT / "docs" / "validation" / "2026-11-09-g2-measure" / "g2m_cpu_sampler.py")
    smp = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(smp)
    own = 1000
    prev = {1: (0, 100), 2: (own, 50), 3: (1001, 10), 4: (65534, 0)}
    now = {1: (0, 300), 2: (own, 450), 3: (1001, 10 + 15 * smp.TCK), 4: (65534, 9 * smp.TCK), 5: (1002, 99999)}
    names = {0: "root", own: "me", 1001: "ksw", 65534: "nobody", 1002: "new"}
    r = smp.sample(prev, now, 10.0, own, names)
    assert r["new"] == 1  # pid 5: its time before this sample is not this interval's
    assert r["other_cores"] == 1.5  # ksw only: root, the job's own user and nobody are not other users
    assert r["cores"]["ksw"] == 1.5 and r["cores"]["nobody"] == 0.9 and "new" not in r["cores"]
    real = smp.table()  # the live process table reads without opening anyone's files
    assert os.getpid() in real and real[os.getpid()][0] == os.getuid()
