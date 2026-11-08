"""Run list of the live riib-defaults validation (registration 3.2): 19 sessions, the A/B pairs first in a mixed
arm order (they run in the bundle's full waves), then C2, the targets and the lifecycle sessions.
    python make_runs.py OUT_ROOT SNAPSHOT FACTOR > RUNS.jsonl"""
import json
import sys

out_root, snap, factor = sys.argv[1], sys.argv[2], sys.argv[3]
P = "/home/projects/forklift"
SCENES = {"line_m": f"{P}/test_runs/riibotics/scenes_ct1_a770031/line_m_session.json",
          "random_m": f"{P}/test_runs/riibotics/scenes_ct1_a770031/random_m_session.json"}
ORDER = {"line_m": "P-1:D-1,P-2:D-2,P-3:D-3", "random_m": "P-4:D-1,P-5:D-2,P-3:D-3"}
WRAP = f"{out_root}/tools/riib_session.sh"
runs = []


def add(name, scene, seed, arm, scenario, cycle=None, fix="none", probe_extra="", **env):
    out = f"{out_root}/s/{name}"
    e = {"ARM": arm, "FIX": fix, "SCENARIO": scenario, "SCENE_FILE": SCENES[scene], "SEED": str(seed),
         "FACTOR": factor, **env}
    if cycle:
        e["CYCLE"] = cycle
    if probe_extra:
        e["PROBE_EXTRA"] = probe_extra
    runs.append({"name": name, "argv": ["bash", WRAP, out], "env": e, "cwd": f"{P}/snapshots/{snap}",
                 "retry_move": [out], "pace_log": f"{out}/live/epoch_*/run/live_log.jsonl",
                 "first_record": f"{out}/live/epoch_1/run/scenario.json", "scan": [f"{out}/live/epoch_1/run.log"]})


# integrated A/B, seeds 0 and 1, the arm order mixed within each scene and seed
for scene, seed, arms in (("line_m", 0, "AB"), ("random_m", 0, "BA"), ("line_m", 1, "BA"), ("random_m", 1, "AB")):
    for arm in arms:
        add(f"ab_{scene}_s{seed}_{arm}", scene, seed, arm, "queue", ORDER[scene])
for scene in ("line_m", "random_m"):  # C2 (2.3): shadow and on, both injected, 2 and 4 off
    for arm in ("C2shadow", "C2on"):
        add(f"c2_{scene}_{arm[2:]}", scene, 0, arm, "queue", ORDER[scene])
for scene in ("line_m", "random_m"):  # guard target (2.1) with the registered stall recovery
    add(f"guard_{scene}", scene, 0, "B", "cycle", ORDER[scene], fix="guard",
        probe_extra="--stall-recover-s 60 --stall-backoff-m 0.30 --manual-recovery")
for scene in ("line_m", "random_m"):  # backoff target (2.2)
    add(f"boxed_{scene}", scene, 0, "B", "queue", ORDER[scene], fix="boxed")
add("life1_line_m_main", "line_m", 0, "B", "main")
add("life2_random_m_held", "random_m", 0, "B", "cycle", "P-4:D-1,P-4:D-1,P-5:D-2",
    probe_extra="--cancel-at transport:5")
add("life3_random_m_reloc", "random_m", 0, "B", "queue", "P-4:D-1", START_RELOC="on", RELOC_UNATTENDED="1")
assert len(runs) == 19 and len({r["name"] for r in runs}) == 19
print("\n".join(json.dumps(r) for r in runs))
