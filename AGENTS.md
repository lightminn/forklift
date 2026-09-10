# AGENTS.md

Guidance for coding agents (Codex and others) working in this repository. Mirror of `CLAUDE.md` — keep both files in sync.

## Repository conventions

Read `CONTRIBUTING.md` before adding or restructuring code. It is the authoritative project convention for folder layout, naming, module dependencies, sensor-data contracts, tests, configuration and documentation. All robot code (upper control, ROS 2, simulation and future MCU firmware) belongs to this repository; presentations stay in the separate repository.

The existing source has not yet migrated to the target `src/` layout. Use the current README commands until the dedicated structural migration updates paths, packaging and checks. Do not combine that migration with new robot functionality. Keep this section identical in both instruction files.

## What this directory is

`forklift/` is the workspace for the pallet-handling forklift assignment of 임베디드구동 및 실습 (2-2): 「주변 장애물을 고려한 최적의 팔레트 핸들링 경로 생성 및 제어」. The brief is an industry-posed deck (cover: "서울시립대 과제"; the reference hardware is a Riibotics forklift). It is a **separate project from the Lab1/Lab2 course archive** in the parent directory. The parent `../CLAUDE.md` is loaded here too: its shell notes (`ls` → `eza`, zsh aborts on unmatched globs, quote every Korean/space path) still apply, but its Lab1 STM32, Lab2 Simulink, exemplar-report and `report/` Overleaf sections describe the *labs*, not this project.

As of 2026-09-10 the directory holds the brief and candidate-platform links, `forklift_core/` (local sensor geometry), `sim/models/dls08_provisional/` (product-informed URDF/MJCF), model tools and tests, and `presentation/` (a compatibility symlink to the separate `../forklift-presentations/` repository). The Git repository has been initialized on `main`, but **there is no autonomous chassis controller yet**. The presentation is a development proposal, not evidence of completed hardware or robot software.

## Current implementation decisions (2026-09-10)

- RealSense **D435i is the confirmed RGB-D camera**. Keep the Gemini 335Le below as the assignment's reference sensor, not the selected hardware.
- **RPLIDAR is confirmed; A2 from the brief is planned** following the user's clarification. The A2 subvariant and its communication/scan settings remain unconfirmed.
- The current purchase recommendation is the official **Jetson Orin Nano Super Developer Kit 8GB with a 256GB M.2 2280 NVMe SSD**. A 128GB SSD is acceptable when already owned or required by cost. Neither the computer nor storage has been purchased.
- Final chassis identification remains deferred until delivery. The user subsequently requested a provisional model from product images/specifications. `dls08_provisional` uses a visually matching candidate catalogue for its envelope and explicitly labelled estimates for components, steering and dynamics; do not promote these into measured robot specifications.
- The development baseline is **ROS 2 Jazzy in an Ubuntu 24.04 laptop container**. Gazebo Harmonic is planned for remote integration simulation; MuJoCo remains the fast local/product-model check. The proposed onboard baseline is JetPack 7.2.1 (Ubuntu 24.04) with Jazzy, subject to real D435i validation before freezing versions. See `docs/hardware.md`, `docs/decisions/0001-development-and-deployment-platforms.md`, and `docs/LOCAL_VALIDATION.md` for decision and evidence boundaries.

## The assignment brief (11 slides)

Re-fetch it without a browser — the deck is link-shared and both exports work anonymously (`<scratchpad>` = the session scratchpad directory):

```bash
ID=$(grep -oE 'presentation/d/[^/]+' quest.txt | cut -d/ -f3)
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

Run `python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error` and `python -m forklift_core.demo` with Python >= 3.10, NumPy >= 1.23, and pytest >= 7. These exercise sensor mathematics with synthetic input, not camera/LiDAR drivers, perception, SLAM, or physical motion. Keep the package independent of ROS and hardware SDKs; adapters must supply real calibration, acquisition timing and frame metadata before live use.

The approved ROS 2 Jazzy development container may be added under `deploy/ros2/`. Do not infer or scaffold chassis-specific ROS robot packages before the delivered chassis, actuator interfaces and compute hardware are identified. MuJoCo is used for local model loading, kinematics, unloaded settling and rendering; Gazebo Harmonic is the planned remote integration simulator and still requires model, sensor and control validation. See `sim/models/dls08_provisional/README.md` and `docs/validation/2026-09-10-product-forklift-model.md`. Generate both XML formats from the YAML/shared assembly; URDF preserves geometry and kinematics, while engine-specific contact and actuation settings require separate configuration. Keep catalogue candidate values, photo estimates, simulated results and physical measurements distinct.
