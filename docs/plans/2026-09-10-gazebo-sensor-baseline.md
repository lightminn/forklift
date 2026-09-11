# Gazebo Sensor Baseline Implementation Plan

> For agentic workers: use `superpowers:subagent-driven-development` for bounded implementation tasks and task reviews. No commits or pushes are authorized.

**Goal:** Repeatable remote Slurm runs and a real Gazebo RGB-D/LiDAR → ROS 2 → 30-second bag/replay validation.

**Architecture:** Independent source snapshots and per-run outputs. A dedicated Jazzy/Harmonic container runs a static provisional forklift and synthetic sensor scene. A ROS adapter validates real messages; the hardware-independent core is unchanged.

**Tech Stack:** Python >=3.10 host tools, Python 3.12 ROS runtime, ROS 2 Jazzy, Gazebo Harmonic, Docker, Slurm, SSH/rsync, pytest, Ruff.

**Spec:** `docs/design/2026-09-10-gazebo-sensor-baseline.md`.

## Global Constraints

- Preserve the current `forklift_core/` layout and all existing dirty documentation.
- No commit/push, no presentation edits, no existing research jobs/environment/Slurm changes.
- Synthetic sensor/chassis assumptions remain labelled; no physical/autonomy claims.
- Explicit host/root/interpreter/image arguments; no personal paths in tracked code.
- CPU default: 2 CPUs and 4GiB; GPU requires explicit Slurm allocation.
- TDD: run behavior tests and confirm failure before implementation. Record outputs.
- Source snapshots are new, hash identified, read-only; outputs are separate, never reused.

### Task 1: Remote execution and collection

**Files:** `tools/submit_model_check.py`, `tools/remote_model_job.py`, `deploy/slurm/model_check.sbatch`, `requirements/model_py311.txt`, `requirements/README.md`, `tests/integration/test_remote_model_jobs.py`, `docs/development.md` remote section.

**Interfaces:** Host CLI with `submit`, `status`, `collect`; submit supports `--host`, `--remote-root`, `--source`, `--mode model-cpu|model-render|gazebo`, `--python`, `--image`, `--run-id`, `--dry-run`, `--wait`. Gazebo executes `python3 sim/gazebo/run_sensor_smoke.py --output /output --duration 30` in the mounted `/workspace`; image comes from explicit argument. Optional argument `--duration` defaults to 30. Model modes use explicitly supplied interpreter. Every mode writes `job_result.json` including commands, exit codes, source hashes and selected Slurm environment.

- [x] Write behavior tests: excluded/symlink secret files never enter snapshot, dry-run has no filesystem/network side effects, path metacharacters survive argument round-trip, duplicates are rejected, failed child commands propagate nonzero status, failed/incomplete Slurm states never become success.
- [x] Run `python -m pytest tests/integration/test_remote_model_jobs.py -q -p no:cacheprovider` and save the expected missing-feature failure.
- [x] Implement the CLI, remote job runner, named file allowlist, manifest verification and explicit resource-limited Docker argv. Package metadata/lock derives from project extras, not a machine-wide freeze.
- [x] Run the tests until passing, then `python -m ruff check tools tests/integration` and format checks. Exercise `--dry-run` with a Korean/space path.
- [x] Main agent submits and collects a real model CPU job with this CLI; task reviewer checks spec compliance and quality before the next implementation task.

### Task 2: Static Gazebo scene and ROS observation validator

**Files:** `sim/gazebo/scene_config.yaml`, `sim/gazebo/build_sensor_world.py`, `sim/gazebo/run_sensor_smoke.py`, `sim/gazebo/README.md`; `ros2/src/forklift_ros/` package with sensor validation node and meaningful tests. Docker environment is prepared by the main agent in `deploy/gazebo/Dockerfile`.

**Interfaces:** `python3 sim/gazebo/run_sensor_smoke.py --output <new-output> --duration 30` from repository root, with sourced Jazzy. It creates world/bridge/TF from one configuration, launches Gazebo headless and bridge, records and validates live topics, stops those publishers, plays bag into a fresh validator, writes result JSON and representative PNGs, stops only its child processes and returns nonzero on any required validation failure.

- [x] Write behavior tests for generation/config errors and message validation: wrong frame/encoding/time order, missing streams, incorrect known distance, coordinate axis sign, truncated bag duration, and stale replay state must fail. Use literal synthetic expectations independent of generation.
- [x] Run new tests and save RED output before writing implementation.
- [x] Build a no-Fuel SDF world with static current-model visuals, synthetic pallet openings, RGB-D/2D scan, known-distance targets. Generate TF and bridge mappings from the same explicit synthetic configuration. Resolve actual sensor header frames through documented Gazebo settings and verify real output.
- [x] Implement ROS validation/bag orchestration with simulation time, bounded wall timeout, unique run isolation, child cleanup, result JSON and PNG exports. Record observed counts/ranges/distances, not only pass flags.
- [x] Run host behavior tests and container ROS tests; main agent runs Slurm integration and views exported images. Preserve failed run directories for diagnosis and rerun with new IDs.

### Task 3: Integration proof and documentation

**Files:** `deploy/gazebo/Dockerfile`, `README.md`, `docs/validation/2026-09-10-gazebo-sensor-baseline.md`, current environment plan; personal machine facts only in environment record.

- [x] Build the dedicated image; record image digest/ID, installed apt package versions and graphics backend. Run image preparation commands as Slurm CPU jobs with bounded Docker resources.
- [x] Submit the real Gazebo job via Task 1 CLI, collect Slurm completion and original result hashes, inspect 30-second live/bag/replay evidence and RGB/depth/scan PNGs.
- [x] Run `python -m ruff check .`, `python -m ruff format --check .`, existing CPU regression plus new tests; run package ROS tests inside Jazzy image.
- [x] Independent final review, fix concrete findings, recheck affected paths. Copy reviewed changes to the original checkout only after checking initial hashes to preserve concurrent work. Update runbook and actual validation boundaries without commit/push.
