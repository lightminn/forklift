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

## 3. Seed-3 world (plan 4.1)

Surveyed after the freeze commit `cb1490d3` (the lead's order), by B-P2d's procedure with layout seed 3:

- a one-second seed-3 record for planning;
- the tour planned from `config/slam_survey_tour.yaml` and refined (refine 8, margin 0.10 m);
- the gyro-heading survey (`deploy/console/slam_survey.sbatch`, SLAM noise seed 0, then 1 and 2 if V2′ fails, limits not relaxed);
- then starts drawn at 3 m (seed 1031).

Registered before the survey (the lead's request):

- **Expected output:** about 0.6 GB. Seed 5's `hall_s5b_G0` was 611 MB.
- **Run:** about 1 h 45 min and 7.3 GB peak RSS (ws1 13006).
- **Disk:** at submission ws1 `/home` had 35 GB free (96 % used).
- **Queue:** two of the lead's riib jobs and one teammate job.

### 3.1 Survey and starts

- **Survey** `hall_s3_G0` (ws1 13495 → 13496 → 13497, snapshot `f1_678f10fb`, SLAM noise seed 0). Map `fb0ad8ca…`.
  - The tour arrived: 562 m, 1,245 graph nodes (`s3_survey_report.json`), and the job ran 2 h 03 min (Slurm).
  - **V2′ passed:** world position maximum 0.064 m (RMSE 0.027 m), yaw maximum 0.0043 rad. Coverage, start, nodes, received, cost and finalize checks also passed.
  - The moving check did not run (no rigid-body list), as on seed 4 and seed 5. The survey tool's overall `ok` is therefore false and Slurm reports the job FAILED.
  - The map is accepted on V2′ as B4a registered, the rule B-P2d applied to seed 5 (the lead confirmed, 2026-11-01).
  - **Output:** 734 MB (`du` on ws1), against the about 0.6 GB registered.
- **Starts** `val_s3.json` (ws1 13618): 98 starts (layer A 49, B 24, C 25).
  - Drawn with seed 1031, 3 m apart, until all three layers were exhausted (95,460 attempts).
  - Validation split, map index 5. Keep-out from the survey's own `result.json`.

## 4. Offline judgement (plan 4.2 a, 4.4, 4.5; frozen code)

Records, per world: `rows_sN.jsonl` (each session's placements), `sweep_sN.json` (the table; R_AMB 0.98 is the judged row, `time` the placement times) and `stress_sN.jsonl` / `stress_sN.json` (4.5). The hard populations' offline rescue is `tools/f1_place_eval.py sweep --rows rows_sN.jsonl --hard hard_sN.json`, read at R_AMB 0.98.

These runs used `tools/f1_place_eval.py` on the frozen code (snapshot `f1_678f10fb`) with R_AMB from `place_config.json`. Each start had three scan sets from capture jobs on that code (map indices 1xx, 2xx and 3xx), and the clicks came from the click model with the world's seed and the start's index.

| World | Valid starts | Success | Imprecise | Wrong | Failed | Placement p95 / max |
|---|---|---|---|---|---|---|
| seed 3 (map `fb0ad8ca`, ws1 13619–13621, 13623) | 91 of 98 | 88 | 0 | 0 | 3 (ambiguous 6, low fit 3 placements) | 1.35 s / 2.94 s |
| seed 5 (map `3edf24b0`, ws1 13498–13500, 13501) | 84 of 93 | 81 | 0 | 0 | 3 (ambiguous 9, off-map 1 placement) | 1.31 s / 2.10 s |
| **both** | **175** | **169** | **0** | **0** | 6 | |

- **Safety (4.2 a) passes:** 169 confirmed sessions with no wrong and no imprecise one. The one-sided 95 % Clopper-Pearson upper bound is 1.8 %. There are 175 valid starts, against at least 120 required.
  - The scope is the click model, the N_PLACE = 3 policy and an operator who confirms the first preview.
- **Time (4.4) passes:** p95 ≤ 2 s and maximum ≤ 10 s in both worlds.
- **Hard populations, offline (4.3, a record):** 16 of seed 3's 18 and 14 of seed 5's 16 are rescued offline (section 5).
- **Record-only placements (4.5)**, first scan set:

| Click | seed 3 confirmed (wrong) | seed 5 confirmed (wrong) |
|---|---|---|
| one bay ±x | 1 (1) | 0 |
| one bay ±y | 9 (9) | 10 (10) |
| 1.5 m off | 86 (0) | 78 (0) |
| 0.5 rad off | 55 (0) | 37 (0) |
| heading turned around | 1 (1) | 0 |

- **Endpoint fractions of the wrong neighbouring-bay confirmations:**
  - seed 3: 0.80–1.00 (10 confirmations: 0.800, 0.849, 0.867, 0.877, 0.877, 0.908, 0.989, 1.0, 1.0, 1.0);
  - seed 5: 0.81–1.00 (10 confirmations: 0.808, 0.831, 0.843, 0.888, 0.938, 0.951, 0.964, 0.984, 1.0, 1.0).
  - The overlap reads as good as the truth's. Overlap is no defence (plan 5).

## 5. Hard populations and the second freeze (plan 4.1, 4.6)

- **Seed 3's hard population:** all 98 seed-3 starts ran through the frozen B-P2 unattended path (ws1 13622, snapshot `bp2d_0fa66b4a`, B-P2d's list generator and bridge digest).
  - Judged against B-P2d's first freeze, the batch pools: no fingerprint mismatch, one argument set (`s3_unattended_judgement.json`).
  - 91 valid: 73 started on the matcher's confirmation (0 wrong, 0 imprecise), 16 asked for a person, 2 had no candidate (`reloc_failed`). 7 invalid.
  - The hard population is those 18 (`hard_s3.json`): val5_011, val5_012, val5_017, val5_024, val5_028, val5_033, val5_035, val5_042, val5_045, val5_047, val5_052, val5_068, val5_074, val5_081, val5_083, val5_088, val5_089, val5_096.
- **Seed 5's hard population:** B-P2d's 16 operator requests (`hard_s5.json`): val4_001, val4_002, val4_012, val4_021, val4_032, val4_033, val4_034, val4_040, val4_043, val4_045, val4_052, val4_069, val4_075, val4_077, val4_081, val4_084.
- **Offline (a record):** 16 of seed 3's 18 and 14 of seed 5's 16 are rescued, 30 of 34.
- **Second freeze** (`docs/validation/assets/f1/freeze2.json`), committed before any F1 collection session ran:
  - seed 3: map id `fb0ad8ca…`, keep-out (the survey's `result.json`, sha256 `be975094…`), start list `val_s3.json` (sha256 `81fdf6e5…`), the survey's V2′ figures, the hard list and the judgement it came from;
  - seed 5: B-P2d's second freeze and the hard list with its judgement;
  - the first freeze's sha256, and the collection's probe, snapshot (`f1_678f10fb`) and list generator.
- **Collection (plan 4.2 b, 4.3):** one attended session per hard start (34), the probe as the operator (`RELOC_PROBE=place`), on snapshot `f1_678f10fb`. Judged with `tools/reloc_judge.py --frozen freeze1.json --hard` per world.

## 6. Isaac judgement (plan 4.2 b, 4.3)

Collection: ws1 13725 on snapshot `f1_678f10fb`, one attended place session per frozen hard start (34). Judged per world by `tools/reloc_judge.py --frozen freeze1.json --hard hard_sN.json` (`judgement_s3.json`, `judgement_s5.json`). Both pool: no fingerprint mismatch, one argument set, every listed start run.

| World | Hard | Started | Wrong / imprecise | Rescued | Held short (`hold_failed`) | Operator's time (3 refusals) |
|---|---|---|---|---|---|---|
| seed 3 | 18 | 16 | 0 / 0 | 7 | 9 | 2 (val5_011 ambiguous, val5_083 low fit) |
| seed 5 | 16 | 14 | 0 / 0 | 13 | 1 | 2 (val4_021, val4_052 ambiguous) |
| **both** | **34** | **30** | **0 / 0** | **20** | **10** | 4 |

- **Safety (4.2 b) passes:** 0 wrong and 0 imprecise of 30 started. The SLAM pose at the running tick was 0.001–0.041 m from the truth. The one-sided 95 % upper bound is 9.5 % (reported, not required, as registered).
- **Rescue (4.3) does not pass as registered:** 20 of 34 = 59 % (one-sided 95 % CP lower bound 43 %) against the 70 % floor. Seed 5 alone: 13 of 16 (81 %); seed 3: 7 of 18.
- **Why the ten were held short** (`hold_check.txt`, per session):
  - The probe holds by wall time: 10 s ÷ the live factor 0.3, plus 1 s, from when it sees the pose after reconnecting.
  - The simulation ran slightly below 0.3×, so every one of the ten ended at t0 + 7.75–9.48 s of simulated time, before the 10 s window closed.
  - Their SLAM replies were tracking without a bad status, at most 0.10 s apart, up to 0.00–0.08 s before the simulation's end. The localization did not stop: the probe ended the session early.
  - The rescued sessions ended at t0 + 9.58–10.05 s and passed only within the window's final 0.5 s gap allowance. The whole collection stood at the edge of the probe's wall-time margin.
- The registered result stands as written. Any rerun is a separate decision (Codex review, below).

## 7. Rerun registration (the lead's decision (B), 2026-11-02), committed before any rerun session

The registered result in section 6 (`0da6d571`) stands and is not replaced. This rerun is a second, separate measurement.

- **What changes:** only the probe's hold. `HOLD_SIM_S=14` is written into each run's environment in the run list. At live factor 0.3 the wall hold becomes 14 ÷ 0.3 + 1 ≈ 47.7 s, which covers 10 s of simulated time down to about 0.21×.
- **What stays:**
  - the frozen code (snapshot `f1_678f10fb`, freeze 1) and the judge and its criterion: 10 s of simulated time from the running tick, gaps ≤ 0.5 s;
  - the start lists `val_s3.json` and `val_s5.json`, the hard lists, the click seeds and the click indices (the start's index in its full list);
  - the bridge image digest, the bundle shape (live, 3 slots, `shard:4`) and the list generator, the output only to a new folder (`f1_val2`).
- **Which starts:** all 34 frozen hard starts, each once.
- **How it is reported:**
  - by itself, against the same floors: safety 0 wrong / imprecise, rescue ≥ 70 %;
  - next to section 6, never pooled with it (not 68 samples), and a worse result is reported as it comes.
- **Also judged on the rerun (the lead's questions):**
  1. Do the four sessions that ended at the operator's time after three refusals (val5_011, val5_083, val4_021, val4_052) end the same way again?
  2. On seed 3 (7 of 18 the first time), is anything other than the probe's hold behind the low count? Every seed-3 session's outcome, placements and refusal reasons will be compared with the first run's.

## 8. Rerun (section 7's registration)

ws1 13936: the first run's list with only `HOLD_SIM_S=14`, the same snapshot and bundle shape. Judged per world by the unchanged judge against freeze 1 with the frozen hard lists (`rerun_judgement_s3.json`, `rerun_judgement_s5.json`). Both pool: no fingerprint mismatch, one argument set, every listed start run.

| World | Hard | Started | Wrong / imprecise | Rescued | Held short | Operator's time |
|---|---|---|---|---|---|---|
| seed 3 | 18 | 16 | 0 / 0 | 16 | 0 | 2 |
| seed 5 | 16 | 14 | 0 / 0 | 14 | 0 | 2 |
| **both** | **34** | **30** | **0 / 0** | **30** | **0** | 4 |

- **Safety:** 0 wrong and 0 imprecise of 30 started (bound 9.5 %). The SLAM pose was 0.001–0.043 m from the truth.
- **Rescue:** 30 of 34 = 88 %, one-sided 95 % CP lower bound 75 %. This clears the 70 % floor in this run, which is reported beside section 6 and not pooled with it.
- **The holds:** every started session ran to t0 + 13.5–14.2 s of simulated time, past the judged 10 s. Each session's clicks, t0, last simulated tick and last SLAM reply, for both runs, are in `sessions_trace.json`.
- **The lead's two questions:**
  1. **The four operator-time endings repeated** with the same refusals: val5_011, val4_021 and val4_052 ambiguous three times, val5_083 low fit three times. They are the hard starts F1's window cannot settle.
  2. **On seed 3, the only difference between the runs is the hold.**
     - Its nine sessions held short the first time are rescued now. They confirmed on the same first click both times: the click model's draw is fixed, and the clicks match in `sessions_trace.json`.
     - Every other seed-3 outcome, placement count and refusal reason is the same in both runs.
     - Its two failures remain: val5_011 was refused three times as ambiguous, val5_083 three times on low fit.
- **Each start's outcome against the first run:**
  - the 20 rescued then are rescued again;
  - the 10 held short (seed 3 nine, seed 5 one) are rescued;
  - the 4 operator-time endings are the same.

- **Seed 5 by the plan's request types** (section 0's typing of B-P2d's 16):

| Type | Registered run | Rerun |
|---|---|---|
| repeated bay (11) | 10 (val4_001 held short) | 11 |
| truth outside the four shown (2: 043, 075) | 2 | 2 |
| deadline (2: 032, 052) | 1 (052 at the operator's time) | 1 |
| aisle slip (1: 021) | 0 (operator's time) | 0 |

## 9. Conclusion, constraints and the follow-up

- **As registered (section 6):**
  - Safety passes: 0 wrong or imprecise of 30 started.
  - The rescue floor does not pass: 20 of 34.
- **In the rerun the lead registered (section 7, reported beside it):**
  - Safety 0 of 30.
  - Rescue 30 of 34 (88 %, CP lower bound 75 %).
- **Offline:** 0 wrong among 169 confirmations, and the time limits pass.
- **What stays until the next freeze:**
  - The probe holds by wall time (plan 5). The sim-clock hold goes into the next, third freeze.
  - A neighbouring-bay or turned-around confirmation passes the bridge's start check. The defence is the operator premise plus the page's look-alike count and bay check:
    - the count is taken only among the shown candidates, at most four;
    - the check is a page step, not enforced by the vehicle.
  - The four hard starts F1 does not rescue need another means: seed 5's val4_021 (aisle slip) and val4_052 (deadline), both refused as ambiguous at every placement, and seed 3's val5_011 (ambiguous) and val5_083 (low fit).
- **Operating residual risk** (the lead's decision of 2026-11-07, unconfirmed position revision plan 9.21; no code change):
  - F1 placement is offered by the console by default and does not require the unconfirmed position check (③).
  - Measured: all four neighbouring-bay misplacements passed the bridge's start check (section 1.3).
  - The defence is the operator premise and the page's aids only, not enforced by the vehicle. ③ is off by default and is not counted as an operating safety layer.
  - Before autonomous operation on the real truck, F1 is turned off (the lead's decision of 2026-11-08, revision plan 9.23 item 4). **Required before the real truck, not done:** implement and verify either a vehicle-side F1 block (AUTO and missions refused while the estimate's source is `operator_placed`) or a manual-only restriction. No "manual driving after a person's check" exception is defined; if one is ever needed, a procedure that independently confirms the real position and heading (not a re-check on the screen) has to be defined and validated separately. No code changes now.
  - This does not resolve the remaining ambiguity of matcher (unattended) starts.
  - **Vehicle-side block implemented** ([plan](../plans/2026-11-08-f1-vehicle-block.md), approved by the lead 2026-11-08): an `operator_placed` start, or one of unknown source, latches a refusal of AUTO and missions (reason `operator_placed_start`) in the arbiter and the mission commands for the whole epoch; teleop stays open. Defaults block in the core, the runner and the Isaac launcher (`F1_AUTONOMY=allow` only for approved development, recorded). Checked in Isaac (plan 5); not verified on the real truck, so the item above stays open until then.
  - **Residual risk to review before the real truck** (the lead's decision of 2026-11-08, outside the block): while blocked, teleop's obstacle-layer permit runs on the placed frame. Objects the live sensors cannot see (registered low props, obstacles only on the static layer) can be missed when the position is wrong, so the risk is not only a needless stop in an empty aisle. Switching the permit to sensor-only evidence is left to the real-truck operation plan; the permit's behaviour is not changed now.
- **Follow-up "확인 전 위치"** (the lead, after F1 closes; plan 5.1):
  - **What:** a placed start with look-alikes (or none known) is held as an unconfirmed position.
    - While unconfirmed, the arbiter refuses pick, lift and lower, unload and mission legs.
    - The lead's provisional defaults allow slow operator teleop and short automatic disambiguating moves with distance and speed caps, in free cells only, the obstacle-layer stop on.
    - The look-alike hypotheses' map-match scores are accumulated from the scans along the way. A hypothesis is promoted when it wins by a margin and the repetition's end is seen.
    - The deferred teleop-while-uninformative case comes under the same rule.
  - **Pieces:**
    - ① a hypothesis tracker (pure core, reusing the global matcher's scores);
    - ② the arbiter's permission and the state field with the console's band. The runner changes, so the replay gate applies;
    - ③ validation: offline on short captured drives from neighbouring-bay confirmations, then Isaac sessions.
    - A plan with three independent reviews comes first. The caps are set there, with reasons. The probe's sim-clock hold joins this third freeze.
  - **Cost:** two to three days including the plan, and about two Isaac batches.
