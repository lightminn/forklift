# Unconfirmed position: the D batch (seed 4, shadow), 2026-11-06

Plan `docs/plans/2026-11-06-unconfirmed-validation.md` 3 and 6. This is the development batch. **Nothing here is frozen**: the plan's section 6 puts a design revision between this record and `dev_bounds.json`. Seed 5 was not read.

## Setup

- **Code:** PR #335 and the start list PR #336, snapshot `ucd_94e912f8` on ws1.
- **Sessions:** 30 starts from `docs/validation/assets/unconfirmed/dev_starts.jsonl`, 10 each of full, half and chatter pressing, 15 flagged for the mission after the strokes.
- **Mode:** `--pose-check shadow`, `--pose-check-contacts`, no audit, factor 0.3, the hall's gyro configuration. `val3_*` starts on map `9a5cefd5`.
- **Jobs:** ws1 16260 and 16261, six at once, 1 h 4 min. Session wall time p50 4.5 min, max 22.6 min. Size 9–86 MB, 1.1 GB in all.
- **Inputs of the numbers:** `dev_report.json` is `docs/validation/assets/unconfirmed/d1_report.json`; the table comes from `d_table.py` beside it.

## What ran

| | sessions |
|---|---|
| records complete | 30 of 30 |
| start check unconfirmed | 2 |
| strokes ended by the stuck watch (rear held) | 10 |
| reached about 2.9 m of reverse | 21 |
| mission flagged | 15: 10 done, 2 failed at planning (`invalid_start`, both also held at the rear), 3 refused `scene_damaged` |
| truth check `scene_damaged` | 7 (6 at the start, 0.6 s; 1 at 35 s) |

The stuck watch's first live use worked: no session ran out of time. No session lost its arm to a stalled tick in this batch.

Two sessions touched a scene object and are left out of the table below. `val3_016`'s forks sat on a scene pallet from the start. `val3_055` backed into a low box at 28.7 s. In both the wheels slipped and the odometry moved without the truck, so the applied pose went wrong (up to 0.87 m and 0.44 m) while the re-match's g stayed within 2 cm of the truth. That is the check's intended case, but it is not a straight reverse's odometry. Both are outside the bounds by a rule fixed after the first three sessions were read and before the rest were: a window or a generation overlapping a contact is left out and counted. This is a post-hoc rule, marked as such; `pose_check_dev.py --ignore-contacts` gives the values without it, listed below.

## Delta_int against distance

The 28 sessions without a scene contact. Distance is the odometry path since C_int was last set. Delta_int is `C o O` against `C_int o O`, which is not the truth error.

| path since C_int (m) | sessions | Delta_int p50 / p95 / max (m) | Delta_int p95 / max (rad) | truth error p95 / max (m) | truth error max (rad) |
|---|---|---|---|---|---|
| 0–1 | 27 | 0.041 / 0.059 / 0.073 | 0.009 / 0.033 | 0.057 / 0.065 | 0.016 |
| 1–2 | 11 | 0.044 / 0.053 / 0.060 | 0.007 / 0.014 | 0.056 / 0.060 | 0.005 |
| 2–5 | 10 | 0.038 / 0.067 / 0.074 | 0.016 / 0.027 | 0.051 / 0.053 | 0.007 |
| 5–10 | 10 | 0.050 / 0.152 / 0.196 | 0.031 / 0.055 | 0.051 / 0.053 | 0.008 |
| 10–20 | 10 | 0.082 / 0.345 / 0.511 | 0.071 / 0.071 | 0.044 / 0.049 | 0.011 |
| 20–50 | 10 | 0.371 / 1.325 / 2.008 | 0.137 / 0.178 | 0.051 / 0.075 | 0.050 |
| over 50 | 10 | 0.915 / 2.352 / 3.454 | 0.261 / 0.295 | 0.067 / 0.103 | 0.054 |

- Delta_int per metre of path, from 5 m on: position p50 1.1 %, p95 3.6 %, max 4.4 %; heading p50 0.0012, p95 0.0038, max 0.0060 rad/m.
- Of 43,259 samples, 0 were wrong by the truth (over 0.30 m or 0.10 rad) and 111 imprecise.
- **All 10 sessions that drove a mission crossed the gate of 0.20 m, 0.05 rad, at 9.5–98 m of path (median about 20 m), with a truth error of 2.3 cm or less at the crossing.** None of the other 18 crossed: they stood or reversed under 3 m, renewing C_int at each stand.

So on the same C and O history, today's monitor would revoke every correct session that drives a mission. The correction is absorbing odometry drift, and Delta_int reads that as error. 5.3's persistence of 0.9 cannot be met with the gate as written.

## Standing heading drift

- 96 stands of at least 2 s, all 30 sessions: the odometry's heading rate p50 0.00007 rad/s, max 0.0018 rad/s (simulated time), the truth still. 6 of them would pass STILL_RAD in 1.5 s.
- **No re-match of 139 was discarded for moving.** The first Isaac sessions' 7 of 8 came from one start (`val3_006`, plan appendix A), which is not in the D list; the largest rate in those sessions was 0.0048 rad/s.
- So the drift is real but rare. D gives no reason to change STILL_RAD, and one start of the 32 seen so far shows that it can matter.

## The development values, not frozen

| value | observed largest | 1.5 x | samples |
|---|---|---|---|
| odometry position over a reverse | 0.077 m | 0.115 m (constant) | 22,564; 19 sessions reach 2.75 m |
| odometry heading over a reverse | 0.054 rad | 0.081 rad (constant) | same |
| E_STILL | 0.043 m, 0.050 rad | 0.065 m, **0.076 rad** | 744 stands |
| E_SLAM | 0.073 m, 0.009 rad | 0.109 m, 0.014 rad | 28 starts, 130 re-matches |

- `e_still_over_budget`: 0.076 rad is over the design's 0.05 rad. Every stand over 0.03 rad (7 of 696) was in a mission; outside a mission the largest is 0.0052 rad, 0.006 m. It is flagged and not cut (plan 3.2).
- Without the post-hoc contact rule the odometry bound is 4.38 m, 0.45 rad and E_SLAM 0.51 m, 0.022 rad: the two slipping sessions set them. E_STILL does not change.
- Both odometry bounds come out constant, because the rule takes the least at 3.0 m and the samples stop at 2.93 m.
- Every evidence pose was within the success tolerance.

## Limits

- 30 starts in one hall, on one map, and 10 missions: the distance figures come from those 10. Sessions are not independent.
- Cut presses shorten the long-distance samples: 16 of 172 strokes closed under 0.05 m, 110 under 0.5 m (half and chatter by design).
- 7 scene-damage starts are this list's: starts were drawn for matching, not for driving. V's mission populations will meet the same thing.
