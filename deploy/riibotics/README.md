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
| `KEEPOUT` | `0` | plans as before. Off by default until P6 (user decision 2026-10-23: no gain shown in Isaac -- `docs/validation/2026-10-23-p1c-search-keepout.md`) | `0.15`: `--lidar-keepout-m`, the planner's searches keep the props taller than the LiDAR plane this much wider on each side, the fixed straights keep them at their size, a loaded transport starting inside the band backs out straight first (`docs/plans/2026-10-23-planner-permission-clearance.md` P1c) |

trailer and conveyor have no copy and run as they are; aisle and aisle_out run their rack copy unless `RACK=off`.
Write the scene files with the marker and rack copies first:

    python tools/riibotics_isaac_scenes.py --output <scene dir> --slot-markers --rack

The sim-only docking option (`--docking-acceptance precision`) and the trailer exit prefix (`--exit-clearance-m`) stay
off; pass them through `EXTRA_ARGS` when a run needs them. `DRY_RUN=1` prints the runner's arguments and starts nothing
(`tests/unit/tools/test_riib_scene_launcher.py` checks them against the runner's parser).

The job asks for `shard:2`: Isaac with the learned proposal worker peaked at 3.9 GB of a 4 GB shard (PR #98). A
`MODE=truth` run can ask for one shard with `sbatch --gres=shard:1`.
