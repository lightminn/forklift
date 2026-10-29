# B-P2d: unattended start relocalization in Isaac (2026-10-29)

Validation of `docs/plans/2026-10-29-start-relocalization.md` 4.0–4.5. This file holds the development runs (plan section 3), the first freeze (4.5) and, once collected, the seed-5 judgement.

## 1. Development (seed-4 map, not judged)

Development map `hall_s4_G0` (map `9a5cefd5`). Starts are B-P1b's seed-4 validation rows, which are B-P2's development set (plan 3). Every session ran through `deploy/console/reloc_session.sh`: unattended, one epoch, the probe's 10 s hold.

| Batch | ws1 | Snapshot | Shape | Sessions |
|---|---|---|---|---|
| dev 1 | 12908 | `bp2d_f4507714` | one at a time | 6 starts |
| dev 2 | 12980 | `bp2d_466becdb` | `isaac_bundle` live, 3 slots, `shard:4` (the validation shape) | 12 starts |
| offsets | 12942 | `bp2d_f4507714` | one at a time, no relocalization | 5 |

### 1.1 Sessions

- **Valid:** 15 sessions. 10 succeeded: confirmed by the matcher on the first attempt, started, and held. SLAM pose errors at the running tick were 1.6–5.3 cm and ≤ 0.003 rad. No start was wrong or imprecise.
- **The other five valid sessions:**
  - 4 operator requests. In 2 the truth was among the shown candidates. In 2 it was not: val3_047 (after the 30 s deadline) and val3_004.
  - 1 `reloc_failed` (val3_019): no candidate could be shown.
- **Invalid:** val3_031 and val3_050, `reloc_invalid_input: not_at_target`. val3_003 too in dev 1, before the fix below.
- **Matcher stage:** at most 30.0 s (val3_047, a deadline hit). Otherwise at most 14.8 s.

### 1.2 Fixes the runs found (before the freeze)

- **`711c5fc3` — spawn check:** the setup's 5 mm truck spawn check ended a start the truck cannot hold as a generic failure, before step 3 could call it invalid (val3_003). With the relocalization on, it now gives way to step 3. Runs without it are unchanged.
- **`711c5fc3` — unrun starts:** the judge counted a start without a session folder as a valid failure, which lowered the safety bound. It is now `not_run`, in no denominator, and the judgement waits until every start has run.
- **`14ad9bc9` — height reference (Codex round 5):** step 3's height check took the placed start's own settled height as its reference. The reference is now B4a's: the truck settled 1 s at the scene's start, then teleported to the placed start at that height and settled 1 s there, before the console opens.
  - Measured: the settled base height on clear floor is about 0.0 m, at the scene start and at placed starts. The authored height is 0.015 m, so it cannot be the reference.
- **`466becdb` — wrapper DRY_RUN:** the wrapper got a DRY_RUN, so `isaac_bundle.py check` can prove a session list.

### 1.3 Bridge stage (`reloc_pose.json` → localization ready)

- Dev 1: 19.7, 8.2, 8.2 s.
- Dev 2: 19.7 s (the job's first session), then 7.6–8.1 s.
- Both 19.7 s values are a job's first bridge start, the cold start. Every validation session is an epoch's first map load, and only a job's first one ran cold here.
- **Development maximum: 19.67 s.**

### 1.4 Stationary correction of an injected start (plan 3)

The truck stood at the scene start (base −2.0, 0, 0). The bridge was told a start 0, 0.08 or 0.25 m away, along x and along y.

| Offset | Bridge's first scan, pose error to the truth | During the 10 s hold | End |
|---|---|---|---|
| 0 | 3.6 mm | ≤ 5.1 mm | ran, held |
| 0.08 m (x), 0.08 m (y) | 3.6 mm, 3.7 mm | ≤ 5.5 mm, ≤ 7.0 mm | ran, held |
| 0.25 m (x), 0.25 m (y) | (refused before replies were kept) | — | `slam_not_localized`; the bridge's poses were within 6 mm of the truth |

- slam_toolbox corrects an injected start to the map on its first processed scan. So the bridge's start check, which compares the SLAM pose with the estimate within 0.10 m / 0.05 rad (`slam_map.START_POSITION_M`, `START_YAW_RAD`), is not a comparison of a value with itself.
- In effect it passes an estimate within about 0.10 m of the truth and refuses one 0.25 m off. That is the success bound, so an imprecise estimate becomes a refusal rather than a start.
- **Decision: the tolerance stays at 0.10 m / 0.05 rad.** Raising it would let imprecise estimates start. Lowering it would refuse estimates that are inside the success bound.

### 1.5 Control gaps

- The console's non-stepping gap (`prewarm`) was 11.6–12.2 s on a job's first session and 0.6–0.7 s after (F2 in plan 7).
- In every successful session the SLAM replies over the hold had no gap over 0.5 s.

## 2. First freeze (plan 4.5)

Recorded in `docs/validation/assets/b-p2d/freeze1.json` by `tools/reloc_judge.py --make-frozen`:

- **Fingerprint:** the sha256 of 190 verdict-path files. These are the listed files plus every file under `src/forklift_core/**/*.py` and `sim/isaac/*.py`, at code `466becdb`.
- **Bridge image:** `forklift/ros2-dev:jazzy@sha256:e7c29c2a0c34900a12d99ce98d383534b3f100eb4e625872fb2ad749f9f52659`.
- **Bridge stage bound: 40 s.** That is 2 × the development maximum of 19.67 s, rounded up (plan 4.3).

Frozen with it (in the fingerprinted code):

- **Matcher settings and thresholds:** as B-P1b froze them (`match_config.json`, `dev_choice.json`).
- **Attempts and deadline:** 3 attempts, one 30 s deadline from the generation's first scan, and the 32 s worker return.
- **Start settle:** 1 s at the scene start for the reference, then 1 s at the placed start (`RELOC_SETTLE_STEPS` 120).
- **Invalid limits** (`scan_capture`):
  - position 2 cm, yaw 0.01 rad;
  - height 1 cm from the reference, tilt 0.01 rad;
  - still within 2 mm / 0.002 rad over the scans.
- **Stationary hold:** 10 s, with reply gaps ≤ 0.5 s.
- **Bridge start check:** 0.10 m / 0.05 rad (1.4).
- **Judgement:** `tools/reloc_judge.py` (4.0 classes, the bounds, floors and records).
- **Lead decisions of 2026-10-29:**
  - the main judgement uses the valid denominator;
  - the started-denominator bound is the promotion condition (4.4b);
  - the success floor is the observed rate ≥ 70 %, with the CP lower bound recorded;
  - the next-world rule (4.4a).

A later change to any fingerprinted file means a new freeze, and the counts start again from 0 (4.4b).

**Order note:** plan 4.5 puts this freeze before the new survey. The seed-5 survey (ws1 12819, snapshot `bp2d_2db8d916`) ran before it, while the development runs were still going.
- No relocalization or matching ran on the seed-5 map before this freeze.
- The survey tool is not on the verdict path.
- The starts are drawn after this freeze.
