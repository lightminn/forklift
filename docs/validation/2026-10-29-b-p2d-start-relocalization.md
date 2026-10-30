# B-P2d: unattended start relocalization in Isaac (2026-10-29)

Validation of `docs/plans/2026-10-29-start-relocalization.md` 4.0–4.5. This file holds the development runs (plan section 3), the first freeze (4.5) and, once collected, the seed-5 judgement.

## 1. Development (seed-4 map, not judged)

Development map `hall_s4_G0` (map `9a5cefd5`). Starts are B-P1b's seed-4 validation rows, which are B-P2's development set (plan 3). Every session ran through `deploy/console/reloc_session.sh`: unattended, one epoch, the probe's 10 s hold.

| Batch | ws1 | Snapshot | Shape | Sessions |
|---|---|---|---|---|
| dev 1 | 12908 | `bp2d_f4507714` | one at a time | 6 starts |
| dev 2 | 12980 | `bp2d_466becdb` | `isaac_bundle` live, 3 slots, `shard:4` (the validation shape) | 12 starts |
| offsets | 12942 | `bp2d_f4507714` | one at a time, no relocalization | 5 |
| frames | 12983 | `bp2d_16b6f86a` | `isaac_bundle` live, 3 slots; bridge image by digest | 4 |

Plan 3 also named the registry's `line_m` and `random_m` maps for development. No relocalization session ran on them. The seed-4 hall is the validation world's kind (the same hall, factory layout), and the bridge bound is taken there, on the larger map. What `line_m` and `random_m` would have added (a small scene's matcher and bridge times) is not part of this freeze (Codex freeze review P3).

### 1.1 Sessions

- **Valid:** 16 sessions. 10 succeeded: confirmed by the matcher on the first attempt, started, and held. SLAM pose errors at the running tick were 1.6–5.3 cm and ≤ 0.003 rad. No start was wrong or imprecise.
- **The other six valid sessions:**
  - 4 operator requests. In 2 the truth was among the shown candidates. In 2 it was not: val3_047 (after the 30 s deadline) and val3_004.
  - 1 `reloc_failed` (val3_019): no candidate could be shown.
  - 1 setup failure from before the fix below (val3_003, dev 1): `Truck spawn displaced`, ended before step 3 ran. The 5 mm check failing does not show that the registered 2 cm limit was exceeded, so it stays a valid setup failure, not an invalid input (Codex freeze review P3).
- **Invalid:** val3_031 and val3_050, `reloc_invalid_input: not_at_target`.
- **Matcher stage:** at most 30.0 s (val3_047, a deadline hit). Otherwise at most 14.8 s.

### 1.2 Fixes the runs found (before the freeze)

- **`711c5fc3` — spawn check:** the setup's 5 mm truck spawn check ended a start the truck cannot hold as a generic failure, before step 3 could call it invalid (val3_003). With the relocalization on, it now gives way to step 3. Runs without it are unchanged.
- **`711c5fc3` — unrun starts:** the judge counted a start without a session folder as a valid failure, which lowered the safety bound. It is now `not_run`, in no denominator, and the judgement waits until every start has run.
- **`14ad9bc9` — height reference (Codex round 5):** step 3's height check took the placed start's own settled height as its reference. The reference is now B4a's: the truck settled 1 s at the scene's start, then teleported to the placed start at that height and settled 1 s there, before the console opens.
  - Measured: the settled base height on clear floor is about 0.0 m, at the scene start and at placed starts. The authored height is 0.015 m, so it cannot be the reference.
- **`466becdb` — wrapper DRY_RUN:** the wrapper got a DRY_RUN, so `isaac_bundle.py check` can prove a session list.
- **`16b6f86a` — bridge image and frames (Codex freeze review):** the bridge runs `BRIDGE_IMAGE` (the tag by default), and each epoch keeps the image id its container ran (`bridge/image_id`). The judge holds every session whose bridge started to the freeze's digest. The setup records plan 3's frame check (`reloc_reference.frames`).
- **`08994238` — what the freeze binds (Codex freeze review, second thread):**
  - **Invalid class:** only the first scan set's input check marks a session invalid. A later rescan that finds the truck moved fails a valid session (`reloc_unsteady`). Before this, a session matched once and then moved would have left the denominator.
  - **Starts:** each session is held to its listed start (`arguments.start_pose` against the row), and repeated start ids are refused.
  - **Arguments and inputs:** the freeze binds a reference session's shared arguments (repository paths relative to the code root) and its external input hashes (chassis URDF, pallet URDF and geometry, scene). The fingerprint now takes every file under `config/`.

### 1.3 Bridge stage (`reloc_pose.json` → localization ready)

- Dev 1: 19.7, 8.2, 8.2 s.
- Dev 2: 19.7 s (the job's first session), then 7.6–8.1 s.
- Both 19.7 s values are the first bridge start in their job (val3_008 was dev 2's first bridge, not its first session). The rest came after one.
- **Development maximum: 19.67 s.**
- Limit (Codex freeze review P3): dev 2 was set for 3 slots but at most 2 sessions overlapped, and its 19.67 s bridge ran alone. The 40 s bound follows the registered rule (2 × the development maximum, rounded up). It is not a measurement of the bridge under the collection's full overlap. A session over it fails the bridge-time floor, not the safety floor.

### 1.4 Stationary correction of an injected start (plan 3)

The truck stood at the scene start (base −2.0, 0, 0). The bridge was told a start 0, 0.08 or 0.25 m away, along x and along y.

| Offset | Bridge's first scan, pose error to the truth | 10 s from tracking (max) | End |
|---|---|---|---|
| 0 | 3.6 mm | 5.2 mm, 0.0005 rad | ran, held |
| 0.08 m (x), 0.08 m (y) | 3.6 mm, 3.7 mm | 5.6 mm, 7.2 mm; 0.0005 rad | ran, held |
| 0.25 m (x), 0.25 m (y) | (refused before replies were kept) | — | `slam_not_localized`; the bridge's poses were within 6 mm of the truth |

- At this place (an open view) and for these offsets (x or y, no yaw offset), slam_toolbox corrects an injected start to the map on its first processed scan. So the bridge's start check, which compares the SLAM pose with the estimate within 0.10 m / 0.05 rad (`slam_map.START_POSITION_M`, `START_YAW_RAD`), is not a comparison of a value with itself.
- It is an agreement check between the estimate and SLAM, not a bound on the truth (Codex freeze review P2, with `slam_map.localized`):
  - an estimate 0.103 m off passes when SLAM reads 3.6 mm toward it;
  - one 0.099 m off is refused when SLAM reads 3.6 mm the other way;
  - an estimate and a SLAM pose both 0.20 m wrong pass together.
- The judgement does not rest on it. The judged pose is SLAM's at the running tick against the truth (plan 4.0), so a start SLAM corrected is judged where SLAM puts it, and a wrong pose that both agree on is still a wrong start.
- **Decision: the tolerance stays at 0.10 m / 0.05 rad.** The measurement gave no reason to move it. It refused the 0.25 m disagreements and passed the 0.08 m ones. Its role stays what plan 5 says: an added check, not the safety argument.

### 1.5 Control gaps

- The console's non-stepping gap (`prewarm`) was 11.6–12.2 s on a job's first session and 0.6–0.7 s after (F2 in plan 7).
- In every successful session the SLAM replies over the hold had no gap over 0.5 s.

### 1.6 Frame check (plan 3)

Four sessions at the scene start's position (−2.0, 0) at yaw 0, π/2, π and −π/2 (ws1 12983). Each recorded, at the placed start after its settle:
- the simulator's base;
- its rear axle, the midpoint of the rear wheel links read from the physics view;
- its LiDAR origin, from the shared cast (`planar_lidar`), not from start_reloc's formula.

| Yaw | Base from placed pose | Rear axle vs 1.1 | LiDAR origin vs 1.1 |
|---|---|---|---|
| 0 | 0.08 mm | 0.09 µm | 0.27 µm |
| π/2 | 0.06 mm | 0.07 µm | 0.50 µm |
| π | 0.07 mm | 0.04 µm | 0.07 µm |
| −π/2 | 0.06 mm | 0.05 µm | 0.46 µm |

- All are within the plan's 1 mm.
- The rear axle sits 0.34 m behind the base on the right side at every heading. At yaw π/2 it is at (−2.0, −0.340), at −π/2 at (−2.0, +0.340).
- The reference heights were −6.6 µm and −6.5 µm.
- All four bridges ran image `sha256:e7c29c2a…`.

## 2. First freeze (plan 4.5)

Recorded in `docs/validation/assets/b-p2d/freeze1.json` by `tools/reloc_judge.py --make-frozen`:

- **Fingerprint:** the sha256 of 254 verdict-path files at code `d8cc6c87`. These are the listed files plus every file under `src/forklift_core/**/*.py`, `sim/isaac/*.py` and `config/**/*.{yaml,json}`.
- **Shared arguments and external inputs:** from the reference session `frame_0` (ws1 12983). It was launched exactly as the collection will be: the same list generator, launcher, wrapper and image digest, with only the world differing.
  - The arguments are stored with the paths under the session's own run root (its `live_web_root`) as repository paths, the world and per-session arguments left out. A path elsewhere stays absolute, so a changed copy outside the run root does not pass as the frozen file, and a session run from a root other than the judged code root is off the freeze (`d8cc6c87`).
  - The external inputs are stored as their recorded content hashes: `forklift_urdf_sha256`, `pallet_urdf_sha256`, `pallet_geometry_sha256` and `scene_sha256`.
  - The base scene and URDF stay named by their absolute ws1 paths.
- **Bridge image:** `forklift/ros2-dev@sha256:e7c29c2a0c34900a12d99ce98d383534b3f100eb4e625872fb2ad749f9f52659`, by digest. The collection runs it as `BRIDGE_IMAGE`, and each session's recorded image id is held to it.
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

**Discarded survey (lead decision 2026-10-29):** plan 4.5 puts this freeze before the new survey. A seed-5 survey (ws1 12819, snapshot `bp2d_2db8d916`, map `2b5c76bd`) ran before the freeze, while the development runs were still going.
- It was surveyed once before the freeze and discarded.
- What was seen: its V2′ figures (world 0.052 m / 0.0048 rad, coverage 99.98 %) and the map itself. No relocalization or matching ran on it, and no setting changed because of it.
- The lead chose to survey seed 5 again after this freeze, by the same procedure with SLAM noise seed 0. If that survey fails V2′, the registered order follows: noise seed 1, then the next world, seed 3.
