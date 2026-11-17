"""Run lists and settings copies of the F2/I re-registration (docs/plans/2026-11-18-f2-i-reregistration.md 1, 3, 6, 7).

  python make_runs.py SNAP_NAME OUT_DIR batch|pace|deck|recheck

Run from the snapshot's root (the settings copies start from its config/isaac_transport_measured.yaml).
batch: settings_<arm>[_240].yaml, RUNS_f2i_batch_<snap>.jsonl = 9 probes + 39 paired transport runs (deterministic).
pace: RUNS_f2i_pace_<snap>.jsonl = random_1 s1 x O, F2, I x 3 in the operating (non-deterministic) mode.
deck: the week-11 deck's steering-related cases the batch lacks (lead 2026-11-18): aisle_rack s1, s2, s18 and
  random_1 s12 x O, F2 (prefix f2ix: reported beside, never in the registered judgement). Uses batch's settings copies.
recheck: cases already fixed by other changes, O only on the current defaults (prefix f2ir): random_1 s13, s22,
  s28 (K1 near loose arrival) and s24 (return-home stop).
b2: the week-11 deck's B2 (lead 2026-11-18): one live queue session, random_m s0, the operating mode (learned
  proposals) with the four riib defaults the live launcher still lacks (the status survey's O6 had all six; the
  launcher now carries return-start-backoff and near-end-stand itself). Measurement-like: submitted alone.
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


DECK = [("aisle_rack", 1), ("aisle_rack", 2), ("aisle_rack", 18), ("random_1", 12)]
RECHECK = [("random_1", 13), ("random_1", 22), ("random_1", 28), ("random_1", 24)]


def deck(snap: str, out: Path) -> list:
    root = f"{P}/test_runs/f2i/batch_{snap}"
    return [pfb.riib(snap, f"{root}/deck", f"f2ix_{arm}_{scene}_s{seed}", scene, seed,
                     {"EXTRA_ARGS": f"--deterministic --settings {out / f'settings_{arm}.yaml'}"})
            for scene, seed in DECK for arm in ("O", "F2")]


def recheck(snap: str, out: Path) -> list:
    root = f"{P}/test_runs/f2i/batch_{snap}"
    return [pfb.riib(snap, f"{root}/recheck", f"f2ir_O_{scene}_s{seed}", scene, seed) for scene, seed in RECHECK]


O6_MORE = "--exit-prefix-exempt on --low-clearance-guard on --restart-confirm-scans 3 --return-cusp-replans 2"


def b2(snap: str, out: Path) -> list:
    # a short root: the SLAM socket path under OUT/live/epoch_999/sock must stay within AF_UNIX's 107 bytes
    row = pfb.live(snap, f"{P}/test_runs/f2ib", "f2ib_O6_random_m_s0", "random_m", 0, False)
    row["env"]["MORE"] = O6_MORE
    return [row]


def video(snap: str, out: Path) -> list:
    """Deck video (lead 2026-11-18): random_1 s8 O/F2, deterministic, VIDEO=on, the top-down overview only."""
    root = f"{P}/test_runs/f2i/batch_{snap}/video"
    return [pfb.riib(snap, root, f"f2iw_{arm}_random_1_s8", "random_1", 8,
                     {"VIDEO": "on", "VIEWS": "none",
                      "EXTRA_ARGS": f"--deterministic --settings {out / f'settings_{arm}.yaml'}"}) for arm in ("O", "F2")]


def main(snap: str, out_dir: str, kind: str) -> None:
    out = Path(out_dir)
    rows = {"batch": batch, "pace": pace, "deck": deck, "recheck": recheck, "b2": b2, "video": video}[kind](snap, out)
    assert len({x["name"] for x in rows}) == len(rows) == {"batch": 48, "pace": 9, "deck": 8, "recheck": 4, "b2": 1, "video": 2}[kind]
    path = out / f"RUNS_f2i_{kind}_{snap}.jsonl"
    path.write_text("".join(json.dumps(x) + "\n" for x in rows))
    print(path, len(rows))


if __name__ == "__main__":
    main(*sys.argv[1:4])
