# F1: the operator's placed start and a local re-match (2026-10-31)

Validation of `docs/plans/2026-10-31-operator-placed-start.md`. This file holds the development on the seed-4 map (plan section 3), the first freeze (4.6) and, once collected, the judgement on seed 3 and seed 5.

## 1. Development (seed-4 map, not judged)

Development map `hall_s4_G0` (map `9a5cefd5`). The starts are B-P1b's seed-4 validation list (`docs/validation/assets/b-p1b/val3_s4.json`, 93 starts).

### 1.1 Offline sessions and R_AMB (plan 2, 3)

Captures: three stationary scan sets per start, one capture job per set (ws1 13442–13444, snapshot `f1_07e1c349`, each a different noise draw). Evaluated by `tools/f1_place_eval.py run` and `sweep` (ws1 13445):

- 84 valid starts and 9 invalid.
- Policy: at most three placements per session, placement k on scan set k. The click is drawn from the click model with the world's seed and the start's index. The first preview is confirmed.
- Records: `docs/validation/assets/f1/dev_s4_rows.jsonl` and `dev_s4_sweep.json`.

| R_AMB | success | imprecise | wrong | failed |
|---|---|---|---|---|
| 0.80 | 24 | 0 | 0 | 60 |
| 0.90 | 62 | 0 | 0 | 22 |
| 0.94 | 72 | 0 | 0 | 12 |
| 0.96 | 77 | 0 | 0 | 7 |
| **0.98** | **81** | **0** | **0** | **3** |
| 1.00 | 81 | 0 | 2 | 1 |

- **R_AMB = 0.98** by the registered rule: among the values with no wrong and no imprecise session, the one that rescues the most, ties to the stricter.
  - 0.80–0.98 are all clean.
  - At 1.00, two wrong sessions are confirmed along a repeated row: val3_004 1.8 m off and val3_090 0.7 m off. Their placements' ambiguity ratios are 0.996–0.9997.
  - The margin between those ratios and 0.98 is small. Seed 3 and seed 5 test it.
- **Time** (one placement's local re-match, ws1, one thread, 247 placements): p95 1.14 s, maximum 2.13 s. Plan 4.4 asks p95 ≤ 2 s and maximum ≤ 10 s.
- **The starts the unattended matcher left to a person** (B-P1b's seed-4 decision not `confirmed`): 15 of the 84. 12 are rescued offline (80 %).
  - val3_004, val3_047 and val3_090 are not: every placement is refused as ambiguous.
- Frozen in `docs/validation/assets/f1/place_config.json`: window 2.0 m / 0.6 rad, edge bands 0.3 m / 0.1 rad, R_AMB 0.98, T_LOC 10 s.

### 1.2 Record-only placements (plan 4.5)

`tools/f1_place_eval.py stress`, first scan set of every valid start, one placement per kind, judged at R_AMB 0.98 (ws1 13451):

| Click | confirmed | of which wrong | endpoint fractions of the confirmed |
|---|---|---|---|
| one bay (6 m) along +y / −y | 6 / 4 of 84 each | all 10 | 0.805–0.996 |
| one bay along +x / −x | 0 / 0 | — | — |
| 1.5 m off, drawn direction | 77 | 0 | — |
| 0.5 rad off | 40 (the rest refused at the edge) | 0 | — |
| heading turned around | 1 | 1 (val3_038, 1.45 m off) | 0.843 |

A neighbouring-bay placement is confirmed with an overlap as good as the true one's. The overlap does not defend against it.

### 1.3 Isaac end to end (plan 3)

Attended sessions through `deploy/console/reloc_session.sh` `RELOC_PROBE=place`: the probe is the operator.

- **ws1 13450** (snapshot `f1_a13e1ac2`):
  - val3_000 confirmed at its first placement, start 0.037 m from the truth.
  - val3_083 confirmed at its first placement, 0.006 m.
  - val3_004 refused three times as ambiguous (ratios 0.999–1.0), then ended at the operator's time, as offline.
  - val3_071 was confirmed by the unattended matcher itself in Isaac, so no placement.
  - A placement's re-match took 0.27–0.33 s.
  - The probe stopped at the early console's hand-over (1012 "session ready") and sent no session end. Fixed in `6eff5a07`.
- **ws1 13486** (snapshot `f1_84d6375b`):
  - val3_000 again: placed, confirmed, reconnected at the hand-over, held 10 s, ended (exit 0).
  - **The safety net (the lead's question):** on four starts the unattended matcher left to a person, the probe placed one bay away on purpose (`--click-kind`, development only) and confirmed:
    - val3_083 bay+y: start (4.97, 1.28), truth (4.97, −4.78)
    - val3_000 bay+y: start (3.48, 3.87), truth (3.50, −2.16)
    - val3_019 bay−y: start (−4.11, −5.29), truth (−4.14, 1.29)
    - val3_084 bay+y: start (4.88, 7.15), truth (4.89, 1.13), its heading also turned around
  - All four passed the bridge's start check (`localization_ready`), held and ended normally. **The bridge's start check does not stop a neighbouring-bay confirmation. The defence is the operator premise (checking the bay number) and the screen aids alone.**
  - A heading-only flip could not be placed in Isaac: the offline case (val3_038) is a start the unattended matcher confirms itself.

### 1.4 What development changed (before the freeze)

- **`6eff5a07`, `d0d4de32` — probe:** the place scenario reconnects at the 1012 hand-over, then holds and ends only the session it started. A reconnect onto another epoch is recorded and left alone.
- **`84d6375b` — record clicks (development only):** `CLICK_KIND`. The judge refuses a session that ran one (`d0d4de32`), so it never pools.
- **`28cbbed2`, `61e3e561`, `e5bfd2a1` — screen aids (the lead's, operating aids; not in the judgement):**
  - The preview counts the look-alike places among the shown global candidates (`slam.reloc.preview_twins`): those scoring at least R_AMB of the best, more than 0.30 m or 0.10 rad from the preview. The page says "같은 모양 칸 N곳 — 칸 번호를 확인하세요", or "같은 모양 칸 여부를 알 수 없음" when no candidates were shown.
  - With look-alikes or none known, confirming needs the operator's "칸 번호를 확인했습니다". The tick belongs to that session, set and preview.
  - A large forward arrow is drawn on the preview.
  - Limits: the twins are counted only among the shown candidates (at most four), and the tick is a page step the vehicle does not enforce.
- Codex reviewed `cb3b8383..e5bfd2a1` over three rounds (P2 ×2, P3 ×1, all fixed) to "no new P1/P2".

## 2. First freeze (plan 4.6)

Recorded in `docs/validation/assets/f1/freeze1.json` by `tools/reloc_judge.py --make-frozen`, as B-P2d's first freeze:

- **Fingerprint:** the sha256 of 258 verdict-path files at code `678f10fb` (this branch with main merged, #236 in).
  - These are the listed files plus every file under `src/forklift_core/**/*.py`, `sim/isaac/*.py` and `config/**/*.{yaml,json}`.
  - F1 added the placement settings `docs/validation/assets/f1/place_config.json` (sha256 `5adc06ca…`) and the click model `tools/f1_place_eval.py`.
- **Shared arguments and external inputs:** from the reference session (ws1 13493, seed 4, val3_000). It was launched exactly as the collection will be:
  - list generator `f1_list.py`, kept on ws1 beside the reference: B-P2d's `reloc_list.py` plus `RELOC_PROBE=place`, `CLICK_SEED` = the world's seed and `CLICK_INDEX` = the start's index in its full list;
  - `tools/isaac_bundle.py` live, after `check`;
  - wrapper `deploy/console/reloc_session.sh` and the bridge image by digest.
  - The session placed once, confirmed and held. Judged against this freeze it pools as `rescued`, SLAM pose 0.013 m from the truth.
- **Bridge image:** `forklift/ros2-dev@sha256:e7c29c2a…`, B-P2d's digest.
- **Bridge stage bound:** 40 s, B-P2d's.
- **Placement settings** (frozen with the code, `place_config.json`): R_LOC 2.0 m, Θ_LOC 0.6 rad, edge bands 0.3 m / 0.1 rad, R_AMB 0.98, T_LOC 10 s. N_PLACE 3 is in `start_reloc.py`.
- **Click model:** base position uniform over the area of a 1.0 m disc and heading uniform within ±0.35 rad. It is seeded per world and start (`tools/f1_place_eval.py click`, the probe draws the same).
- **Judgement:** `tools/reloc_judge.py` with `--frozen freeze1.json` and the code root the sessions ran.
  - A session that placed a record click is refused.
  - The second freeze adds the hard-population id lists (`--hard`).

Next (plan 4.1, 4.6): the seed-3 survey after this commit, then the starts, the second freeze (map id, keep-out, start list hash, both worlds' hard populations) and the collection.
