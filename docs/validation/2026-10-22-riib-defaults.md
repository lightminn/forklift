# Riibotics scene runs — the agreed defaults (2026-10-22)

User decision (2026-10-22, from the lead session): the Riibotics scene runs default to PR #88's near re-capture options
and, for line / random, to the slot-marker scenes of PR #99; the plain scenes and the options off stay selectable for
regression. The launcher is `deploy/riibotics/riib_scene.sbatch` (`deploy/riibotics/README.md`); ws1's untracked
`test_runs/riibotics/riib.sbatch` and `riib_truth.sbatch` now call it with that machine's paths, and the scripts from
before are kept as `riib_pre_defaults.sbatch` / `riib_truth_pre_defaults.sbatch`.

## Check: line_2 with the new defaults

Isaac on ws1, snapshot `riib_395ec642` (main 11d7015a plus the launcher), scenes `scenes_riib_395ec642` (the scene tool
with `--slot-markers`), `riib.sbatch` defaults (MODE=slam, NEAR=on, MARKERS=on -> `line_2_m.json`), jobs 8650–8652.
The leg-end creep fix (`fix/leg-end-creep`) was **not** merged at this point, so this compares the defaults on the code
before that fix.

| seed | new defaults (NEAR=on, markers) | before (NEAR off, markers; PR #99 runs, snapshot riib_3f68edf) |
|---|---|---|
| 0 | complete, delivery 0.44 cm, docking ok ×3 | near capture failed (`no_opening_pattern`) twice (the first try also lost the learned worker) |
| 1 | complete, 1.15 cm, ok ×2 | complete, 1.91 cm, ok ×2 |
| 2 | complete, 1.21 cm, ok ×3 | transport tracking timeout (leg-end creep, the fix's target) |

The seed 0 near capture that failed without the near options passes with them. Seed 2's creep did not recur here; with
the fix not yet merged that is one run's outcome, not evidence about the creep. After `fix/leg-end-creep` merges,
the same three seeds on the merged code are the comparison to make.
