# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Repository conventions

Read `CONTRIBUTING.md` before adding or restructuring code. It is the authoritative project convention for folder layout, naming, module dependencies, sensor-data contracts, tests, configuration and documentation. All robot code (upper control, ROS 2, simulation and future MCU firmware) belongs to this repository; presentations stay in the separate repository.

The source uses the `src/` layout from `CONTRIBUTING.md` §1 since 2026-09-11 (`src/forklift_core/`, `examples/`, `tests/unit|integration/`). Install with `python -m pip install -e '.[dev]'` before running anything; do not rely on the checkout directory being on `sys.path`. Keep this section identical in both instruction files.

## What this directory is

This is the **리보틱스** team's **2026-2** project. The shared course Drive groups it under `2026 2학기 임베설/리보틱스/`. Use `리보틱스/forklift/` as the remote project workspace without a duplicate team directory; see `docs/plans/2026-09-10-local-and-remote-development-environment.md` for the observed Drive layout and team execution directories.

`forklift/` is the workspace for the pallet-handling forklift assignment of 임베디드구동 및 실습 (2-2): 「주변 장애물을 고려한 최적의 팔레트 핸들링 경로 생성 및 제어」. The brief is an industry-posed deck (cover: "서울시립대 과제"; the reference hardware is a Riibotics forklift). It is a **separate project from the Lab1/Lab2 course archive** in the parent directory. The parent `../CLAUDE.md` is loaded here too: its shell notes (`ls` → `eza`, zsh aborts on unmatched globs, quote every Korean/space path) still apply, but its Lab1 STM32, Lab2 Simulink, exemplar-report and `report/` Overleaf sections describe the *labs*, not this project.

As of 2026-09-10 the directory holds the brief and candidate-platform links, `src/forklift_core/` (local sensor geometry), `sim/models/dls08_provisional/` (product-informed URDF/MJCF), model tools and tests, and `presentation/` (a compatibility symlink to the separate `../forklift-presentations/` repository). The Git repository has been initialized on `main`, but **there is no autonomous chassis controller yet**. The presentation is a development proposal, not evidence of completed hardware or robot software. As of 2026-09-11 it also holds the remote Slurm/Gazebo tooling, the static Gazebo scene (`sim/gazebo/`) and the `forklift_ros` validator package; the verified state and its evidence are summarised in `docs/validation/2026-09-11-development-checkpoint.md`.

## Current implementation decisions (2026-09-10, last updated 2026-09-17)

- RealSense **D435i is the confirmed RGB-D camera**. Keep the Gemini 335Le below as the assignment's reference sensor, not the selected hardware.
- **RPLIDAR is confirmed; A2 from the brief is planned** following the user's clarification. The subvariant is **likely A2M12** (2026-09-15, user's expectation, not a purchase), and its communication and scan settings remain unconfirmed. Catalogue figures are in `docs/hardware.md`; do not use them as measured values.
- The current purchase recommendation is the official **Jetson Orin Nano Super Developer Kit 8GB with a 256GB M.2 2280 NVMe SSD**. A 128GB SSD is acceptable when already owned or required by cost. Neither the computer nor storage has been purchased.
- Final chassis identification remains deferred until delivery. The user subsequently requested a provisional model from product images/specifications. `dls08_provisional` uses a visually matching candidate catalogue for its envelope and explicitly labelled estimates for components, steering and dynamics; do not promote these into measured robot specifications.
- The development baseline is **ROS 2 Jazzy in an Ubuntu 24.04 laptop container**. The proposed onboard baseline is JetPack 7.2.1 (Ubuntu 24.04) with Jazzy, subject to real D435i validation before freezing versions. See `docs/hardware.md`, `docs/decisions/0001-development-and-deployment-platforms.md`, and `docs/validation/2026-09-10-sensor-core.md` for decision and evidence boundaries.
- **Both EPAL 6 and T11 x0.6 are required** (user, 2026-09-17). The A-D completion conditions are per shape; one shape passing does not stand in for the other. T11's canonical geometry is **not settled** — ADR 0002 records the opening as both 235 and 350 mm and `config/pallet_geometry_t11_06.yaml` has already fixed the 350 reading. Both readings are drawn in `docs/design/2026-09-17-t11-test-article-drawing.md` and **neither is adopted**; do not bake either into a regression constant before the standard, edition and drawing are identified.
- **New dynamic simulation goes to Isaac Sim alone** ([ADR 0004](docs/decisions/0004-simulation-engine-and-insertion-depth.md), 2026-09-17). Gazebo is **frozen, not deleted**: its static scene generators, the EPAL and v1 captures and the `forklift_ros` sensor validator stay as a read-only baseline, and no new Gazebo features are written. `tools/scene_rig.py` and MuJoCo stay as CPU test fixtures rather than as the designated dynamic-milestone simulator — both do simulate; the distinction is the role, not the nature. The remote `bench03` Isaac assets are to be brought into the repository; until they are, an Isaac result cannot be reviewed or reproduced from a clone.
- **Insertion depth is a rule, not a constant**: `min(pallet depth x 0.6, 406 mm - 46 mm)`, giving 360 mm for both shapes today for different reasons. This is the **operating target**, not a safety pass condition: the 406 mm carriage limit and the 46 mm reserve are provisional, `depth/2` is a longitudinal proxy that assumes a centred load and continuous support, and the real stability contract waits on M6's measured support and load tests. Carry the rule, not the number, and do not read either as clearance that has been verified. The docking planner does not enforce the 46 mm reserve — it rejects only at the 406 mm all-box limit the provisional model computes — not a measured physical limit — so 405 mm passes. `build_t11_drawing.py` does compute the rule, with its own copies of the constants.

## The assignment brief (11 slides)

Re-fetch it without a browser — the deck is link-shared and both exports work anonymously (`<scratchpad>` = the session scratchpad directory):

```bash
ID=$(grep -oE 'presentation/d/[^/]+' docs/references/quest.txt | cut -d/ -f3)
curl -sL "https://docs.google.com/presentation/d/$ID/export?format=txt" -o "<scratchpad>/quest.txt"
curl -sL "https://docs.google.com/presentation/d/$ID/export/pdf"        -o "<scratchpad>/quest.pdf"
pdftoppm -r 60 -png "<scratchpad>/quest.pdf" "<scratchpad>/pg"   # slides 4 and 8–11 are pictures only
```

**Goal.** Detect a pallet, plan the *fastest collision-free* path that puts the forks into its pockets, load it, drive to a destination, unload. During insertion, keep **tracking the pocket positions** so the forks never touch the pallet.

**Sensors named in the brief.**
- Pallet detection: RGB-D camera **Gemini 335Le** (Orbbec; GigE + PoE, IP67 stereo). On the Riibotics reference forklift it sits on the mast, centred above the forks and looking forward along them (slide 4).
- Localization and obstacles: **2D LiDAR, Slamtec RPLIDAR A2**, for ROS 2D SLAM and obstacle perception (slide 3).

**Recommended platform.** A small toy forklift converted into a robot, *or* an existing mobile robot with a fork-shaped bar added so it can pick up a pallet.

**Required pipeline** (the deck's numbered process — keep this order as the module boundary):
1. Pallet detection (RGB-D)
2. Pallet pose in the **robot base frame**
3. Path to the pockets that avoids surrounding obstacles (LiDAR map)
4. Path following → fork insertion with pocket tracking → loading
5. Drive to the destination → unloading

**Four approach scenarios** (slides 8–11; each starts from "인식" = pallet detected). These are the acceptance cases — evaluate any planner/controller against all four:

| Case | Situation | Expected behaviour |
|---|---|---|
| A | Pallet straight ahead, aligned | Drive straight in and insert |
| B | Pallet laterally offset but far enough away | Curved approach, insert without stopping |
| C | Too close to enter | Back up first, then insert |
| D | Too close **and** an obstacle behind | Rotate to relocate and then approach; or back up only as far as the obstacle allows and align with the pallet |

## Approved Gazebo sensor baseline

The user approved a static Gazebo Harmonic scene and generic RGB-D/2D LiDAR ROS 2 observation, 30-second recording and fresh-process replay validation on 2026-09-10. `sim/gazebo/`, `ros2/src/forklift_ros/` and the dedicated `deploy/gazebo/` image are within this scope. Use explicit synthetic sensor poses and the current provisional model frozen at neutral joints. This approval does not identify the delivered chassis, authorize a drive controller or turn simulated sensor parameters into real calibration. See `docs/design/2026-09-10-gazebo-sensor-baseline.md` for the contract and `docs/validation/2026-09-10-gazebo-sensor-baseline.md` for observed results.

## Environment requirements

- Use the repository's Ubuntu 24.04 / ROS 2 Jazzy development container for ROS work. Do not mix ROS packages into an arbitrary host Python environment.
- Keep host-specific installation facts, account names, paths, SSH aliases and available remote machines in the user's environment record, not in tracked project guidance. Access to any machine does not establish that it is this robot's onboard computer.
- `AGENTS.md` next to this file is the Codex mirror; keep the two identical apart from the header (global pairing rule).

## Separate presentation repository

The presentation is managed in the sibling `../forklift-presentations/` repository: https://github.com/lightminn/forklift-presentations . GitHub Actions publishes `main` to https://lightminn.github.io/forklift-presentations/ . The local `presentation/` symlink preserves existing commands.

The repository now manages weekly decks. Its root URL lists published weeks, `/week-02/` contains the current week-2 development proposal, and `/latest/` opens the highest numbered published week. Each `week-NN/` directory independently stores its source, notes, provenance, images, and UOS runtime. Do not overwrite a previous week to prepare a new one.

`presentation/week-02/build_deck.py` is the editable week-2 slide/notes source. It generates that directory’s `index.html`, `SCRIPT.md`, and `slide-metadata.json`; `deck.css` controls the body layout. Root `presentation/build_deck.py` regenerates all weeks and the index. `presentation/tools/new_week.py 3 --title "3주차 개발 진행 보고"` creates a separate draft from the latest published week; this is a usage example, not an instruction to create week 3 now. Each week’s `week.json` controls its title, summary, and draft/published status. Drafts are excluded from Pages deployment. React, fonts, images, and the UOS runtime remain local to each week.

```bash
# From forklift/; Python 3, no extra build packages
python3 presentation/build_deck.py
bash presentation/present.sh
# Open http://127.0.0.1:8765/ for the weekly index; /week-02/ for week 2
# F = fullscreen, arrows = navigation
```

Read `presentation/README.md` and the target week’s `README.md`, `SOURCES.md`, and `VALIDATION.md` before changing that deck. The presentation target is 1920 × 1080 fullscreen; phone checks are not required. Keep proposed targets, example schedules, candidate hardware, and actual measured results distinct. Preserve the official UOS template geometry. Check rendered slides after content changes; successful generation does not prove visual fit.

For subsequent presentations, follow the composition and anti-repetition principles in `presentation/AGENTS.md` and `presentation/CLAUDE.md`. Choose topics, structure, and length according to the presentation purpose and audience; do not mandate particular topics or a fixed slide allocation. Review slides and narration together, merging redundant explanations while retaining detail or brief recap where it helps understanding.

## Local sensor core; chassis-dependent work deferred

Run `python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error` and `python examples/sensor_geometry.py` with Python >= 3.10, NumPy >= 1.23, and pytest >= 7. These exercise sensor mathematics with synthetic input, not camera/LiDAR drivers, perception, SLAM, or physical motion. Keep the package independent of ROS and hardware SDKs; adapters must supply real calibration, acquisition timing and frame metadata before live use.

The approved ROS 2 Jazzy development container lives under `deploy/ros2/` (launcher `tools/ros2_dev.sh`, usage in `docs/development.md`); the remote Slurm/Gazebo path is `tools/submit_model_check.py` with `deploy/slurm/`, `deploy/gazebo/` and `requirements/`. Do not infer or scaffold chassis-specific ROS robot packages before the delivered chassis, actuator interfaces and compute hardware are identified. MuJoCo is used for local model loading, kinematics, unloaded settling and rendering; Gazebo Harmonic provides the approved static sensor observation/replay baseline; chassis dynamics, actuation and control still require separate validation. See `sim/models/dls08_provisional/README.md` and `docs/validation/2026-09-10-product-forklift-model.md`. Generate both XML formats from the YAML/shared assembly; URDF preserves geometry and kinematics, while engine-specific contact and actuation settings require separate configuration. Keep catalogue candidate values, photo estimates, simulated results and physical measurements distinct.
