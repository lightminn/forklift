# Riibotics scene runs

`riib_scene.sbatch` runs one Riibotics demo scene (`tools/riibotics_isaac_scenes.py`) as an Isaac autonomous
mission on a Slurm node. It keeps no host paths: the caller passes the code checkout, the scene files, the base
scene, the URDF and the interpreters as environment variables (see the script's header).

## Defaults (user decision 2026-10-22)

| Switch | Default | What it does | Regression choice |
|---|---|---|---|
| `MODE` | `slam` | far recognition with the learned proposals, SLAM feedback through the slam_toolbox bridge, D7 docking | `truth`: truck and pallet poses as truth, no camera, no SLAM |
| `NEAR` | `on` | PR #88 near re-capture: `--near-capture-expected on --near-capture-lower waive_far_confirmed --near-capture-redraw always --near-capture-redraw-lateral-m 0.10` (only in `slam`: they need the perception target) | `off`: the runner's own defaults |
| `MARKERS` | `on` | `line_*` / `random_*` run their slot-marker copy `<scene>_m.json` (PR #99, `docs/plans/2026-10-22-open-zone-docking.md`) | `off`: the plain scene |
| `RACK` | `on` | `aisle` / `aisle_out` run their rack copy `<scene>_rack.json` — rack uprights, 0.58 m slot gaps, 0.20 m back gap (`docs/plans/2026-10-22-riibotics-aisle.md` D-A, D-C) | `off`: the plain scene (0.15 m back gap, no uprights) |
| `KEEPOUT` | `0.15` for the aisle's rack copy (`aisle_rack`), `0` for every other scene | at `0` the planner plans as before; the rack aisle's default follows the same-window comparison in `docs/validation/2026-10-24-start-band-release.md` (rack aisle, release off, within the same batches: transports passed 0/9 at `0` against 7/14 at `0.15`; trailer no gain) | any value overrides it; `0.15`: `--lidar-keepout-m`, the planner's searches keep the props taller than the LiDAR plane this much wider on each side, the fixed straights keep them at their size, a loaded transport starting inside the band backs out straight first (`docs/plans/2026-10-23-planner-permission-clearance.md` P1c) |
| `NOISE` | `const` | the obstacle layer `config/obstacle_layer_single.yaml` (σ 0.02, cut 0.06 m at every range) | `a2`: `config/obstacle_layer_single_a2.yaml` -- LiDAR range noise σ = acc/2 cut at max(0.02, 1.5·acc) of the true range and the grid's range bound from the measured range, from Slamtec's A2 spec rows (read as ±2σ, a design assumption; P3a). The same scan feeds SLAM and the scan docking, whose acceptance under it is not settled (`docs/validation/2026-10-24-p3a-a2-range-bound.md`) |
| `START_BAND` | `off` | `--start-band-release off` | `on`: an occupied cell the stopped truck already stands within the permission's own start inflation of (0.02 m) does not block moving away from it -- `docs/plans/2026-10-24-start-band-release.md`; off by default: no gain on the trailer and one actual load overlap in the rack aisle with it on (`docs/validation/2026-10-24-start-band-release.md`) |
| `LOW_CLEARANCE` | `off` | `--low-clearance-guard off` | `on`: the props below the LiDAR plane, which the drive permission never sees, are watched along the stop arc of the estimated pose against the leg's planned clearance, and the wheel speed is capped before it is used up -- `docs/plans/2026-10-25-low-clearance-guard.md`; the default follows the Isaac on/off comparison |

trailer and conveyor have no copy and run as they are; aisle and aisle_out run their rack copy unless `RACK=off`.
Write the scene files with the marker and rack copies first:

    python tools/riibotics_isaac_scenes.py --output <scene dir> --slot-markers --rack

The sim-only docking option (`--docking-acceptance precision`) and the trailer exit prefix (`--exit-clearance-m`) stay
off; pass them through `EXTRA_ARGS` when a run needs them. `DRY_RUN=1` prints the runner's arguments and starts nothing
(`tests/unit/tools/test_riib_scene_launcher.py` checks them against the runner's parser).

The job asks for `shard:2`: Isaac with the learned proposal worker peaked at 3.9 GB of a 4 GB shard (PR #98). A
`MODE=truth` run can ask for one shard with `sbatch --gres=shard:1`.
