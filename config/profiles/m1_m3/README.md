# M1–M3·G3 drive profiles (G6, `drive_profile/1`)

Registered with `docs/plans/2026-11-15-m1-m3-registration.md` (draft, not yet reviewed). Each file is one trial type for
`tools/teleop_bridge.py --record … --profile <file>`; the trial counts, order, analysis and reading rules are in the
registration, not here. Do not edit a file after registration: a change is a new registration.

| Files | Registration | What |
|---|---|---|
| `g3_arc_{left,right}` | 2, 2.6 | teleop-max arc 0.12 m/s, half turn + 1.5 turns, stills at 0°, 180°, end |
| `m1_{fwd,rev}_{v010,v005}` | 3 | q 0, 3.0 m straight at 0.10 / 0.05 m/s |
| `m2_{q003,q008,q015,max}_{left,right}_{fwd,rev}` | 4 | q held (approached from 0), 1.5 m arc at 0.10 m/s |
| `m3_merge_o{05,20,40}_{l09,l21}_{left,right}` | 5.1 | restart, sine merge (offset, length), 0.42 m straight, stop |
| `m3_qstep_{left,right}` | 5.2 | 0.10 m/s straight, q 0 → ±0.1312 rad 1.5 s in, 0.5 m |

Every file starts with a presteer and has 60 s stills before and after its drive (registration 2.2), `load: empty`
only (lead decision (a)). Re-run the checker in registration section 9 whenever `config/real/vehicle.yaml` or
`config/real/teleop_bridge.yaml` changes.
