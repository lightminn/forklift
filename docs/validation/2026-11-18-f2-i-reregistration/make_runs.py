"""Run lists and settings copies of the F2/I re-registration (docs/plans/2026-11-18-f2-i-reregistration.md 1, 3, 6, 7).

  python make_runs.py SNAP_NAME OUT_DIR batch|pace

Run from the snapshot's root (the settings copies start from its config/isaac_transport_measured.yaml).
batch: settings_<arm>[_240].yaml, RUNS_f2i_batch_<snap>.jsonl = 9 probes + 39 paired transport runs (deterministic).
pace: RUNS_f2i_pace_<snap>.jsonl = random_1 s1 x O, F2, I x 3 in the operating (non-deterministic) mode.
"""
import importlib.util
import json
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location("pfb_runs", HERE.parent / "2026-11-15-post-fix-baseline" / "make_runs.py")
pfb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pfb)

P = pfb.P
ARMS = {
    "O": {},
    "F2": {"tyre_contact_stiffness_n_per_m": 140000.0, "tyre_contact_damping_n_s_per_m": 108.5},
    "I": {"solver_position_iterations": 128},
    "F2I": {"tyre_contact_stiffness_n_per_m": 140000.0, "tyre_contact_damping_n_s_per_m": 108.5,
            "solver_position_iterations": 128},
}
PROBE_CLEAR = {"probe_clear_area": True}
CASES = [("random_1", s) for s in (0, 5, 7, 8, 9, 19, 23)] + [("aisle_rack", 17), ("aisle_rack", 4)]
REGRESSION = [("line_2", 0), ("aisle_rack", 0), ("random_1", 1), ("random_1", 2)]
PROBES = [("O", 120, ""), ("F2", 120, ""), ("I", 120, ""), ("F2I", 120, ""),
          ("O", 240, ""), ("F2", 240, ""), ("I", 240, ""),
          ("O", 120, "r"), ("F2", 120, "r")]


def settings_text(base: str, arm: str, hz: int, probe: bool) -> str:
    keys = {**ARMS[arm], **(PROBE_CLEAR if probe else {})}
    text = base
    if hz != 120:
        text, n = re.subn(r"(?m)^physics_hz:.*$", f"physics_hz: {hz}", text)
        assert n == 1, "physics_hz must appear once"
    for k in keys:
        assert not re.search(rf"(?m)^{k}:", text), f"{k} already set in the base settings"
    return text.rstrip("\n") + "\n# F2/I re-registration arm " + arm + "\n" + "".join(
        f"{k}: {str(v).lower() if isinstance(v, bool) else v}\n" for k, v in keys.items())


def batch(snap: str, out: Path) -> list:
    base = (Path.cwd() / "config/isaac_transport_measured.yaml").read_text()
    root = f"{P}/test_runs/f2i/batch_{snap}"
    rows = []
    for arm, hz, rep in PROBES:
        tag = f"{arm}{'' if hz == 120 else hz}"
        sp = out / f"settings_probe_{tag}.yaml"
        sp.write_text(settings_text(base, arm, hz, True))
        rows.append(pfb.riib(snap, f"{root}/probe", f"f2ip_{tag}{rep}_random_1_s1", "random_1", 1,
                             {"EXTRA_ARGS": f"--deterministic --steering-probe --steering-probe-dump --settings {sp}"}))
    for arm in ("O", "F2", "I"):
        sp = out / f"settings_{arm}.yaml"
        sp.write_text(settings_text(base, arm, 120, False))
    for scene, seed in CASES + REGRESSION:
        for arm in ("O", "F2", "I"):
            rows.append(pfb.riib(snap, f"{root}/pair", f"f2i_{arm}_{scene}_s{seed}", scene, seed,
                                 {"EXTRA_ARGS": f"--deterministic --settings {out / f'settings_{arm}.yaml'}"}))
    return rows


def pace(snap: str, out: Path) -> list:
    root = f"{P}/test_runs/f2i/pace_{snap}"
    rows = []
    for k in range(3):
        for arm in ("O", "F2", "I"):
            rows.append(pfb.riib(snap, root, f"f2iv_{arm}_r{k}_random_1_s1", "random_1", 1,
                                 {"EXTRA_ARGS": f"--settings {out / f'settings_{arm}.yaml'}"}))
    return rows


def main(snap: str, out_dir: str, kind: str) -> None:
    out = Path(out_dir)
    rows = batch(snap, out) if kind == "batch" else pace(snap, out)
    assert len({x["name"] for x in rows}) == len(rows) == (48 if kind == "batch" else 9)
    path = out / f"RUNS_f2i_{kind}_{snap}.jsonl"
    path.write_text("".join(json.dumps(x) + "\n" for x in rows))
    print(path, len(rows))


if __name__ == "__main__":
    main(*sys.argv[1:4])
