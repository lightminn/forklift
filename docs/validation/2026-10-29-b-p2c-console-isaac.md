# B-P2c console: first Isaac sessions (2026-10-29)

Development check of the start relocalization console (`docs/plans/2026-10-29-start-relocalization.md` 1.2–1.3) in Isaac. It is not the B-P2d validation. Starts come from B-P1b's seed-4 validation rows, which are not B-P2d's registered starts. No floor is judged here.

## Setup

| | |
|---|---|
| Map | `hall_s4_G0` (map `9a5cefd5`). This is the B-P1b seed-4 survey and the B-P2 development map (plan section 3). |
| Code | Snapshots `bp2c_d1eca81d` (session 1) and `bp2c_81727013` (session 2). |
| Launcher | `deploy/console/isaac_live.sbatch` with `START_POSE` set, `RELOC_WAIT_S` 600 (operator time 540 s). Seed 4, factor 0.3. Survey arguments: gyro heading, `obstacle_layer_single_team_spec_gyro.yaml`, SLAM noise seed 0. |
| ws1 | Jobs 12806 and 12810. Records under `test_runs/live/bp2c_capture/`. |

## Session 1: start `val3_000` (3.499, −2.162, 1.156). In B-P1b the matcher refused this start as `competitor`.

### Epoch 1
1. The early console opened about 26 s after the epoch started, showing `relocalizing`.
2. Three attempts ended `competitor`. The phase moved to `awaiting_pick` with 4 candidates (`isaac_awaiting_pick.png`):
   - three at 246°, the true heading turned by π, along the repeated shelf bays;
   - one at 66° at (3.48, −14.18), on the true heading but 12.0 m along the bay row.

   None of them was the true start. B-P1b's candidates for this start did not include it either.
3. The operator (the probe, by heading) picked candidate 4 with the two-click confirm. The ack, `reloc_pose` (source `operator`) and `bridge_go` followed (`isaac_localizing.png`).
4. The bridge's start check refused the pose: `slam_not_localized`, endpoint fractions 0.756–0.759. The runner ended at stage `relocalize`. **A wrong pick did not become a running session.** The supervisor restarted the epoch.

### Epoch 2
- Same candidates. The operator sent `session_end` while the start was still unknown.
- The ack came back and the runner exited with code 0 and no reason. The result says `ended_before_localization: {by: operator}`.
- The early console closed, and the supervisor stopped without a restart.

## Session 2: start `val3_073` (−1.719, −20.290, 1.177). In B-P1b the matcher refused this start too.

1. With this run's noise the matcher confirmed on its first attempt. The phase went to `localizing` without a pick.
2. The estimate (−1.718, −20.324, 1.177) was 0.034 m / 0.0002 rad from the placed truth.
3. `localization_ready`, then the live runtime adopted the console; the port file read `running` with `localized_wall` (`isaac_running.png`).
4. Verdict record at the running tick: SLAM (−1.759, −20.327, 1.174) against truth (−1.719, −20.290, 1.177). That is 0.055 m / 0.003 rad.
5. The operator ended the session: exit 0.

## Findings

- **Behaved as designed in Isaac:**
  - the early console, `awaiting_pick` with candidates, the two-click pick;
  - the bridge start check refusing a wrong pick;
  - the session end before localization (exit 0, no restart);
  - the matcher's own confirmation leading to adoption and `running` with `localized_wall`.
- **The non-stepping setup gap is about 15 s.** It was 15.4 s and 15.2 s, recorded at stage `prewarm`, between the phase and the warm-up: drive's initialisation, the live runtime and the kernels' warm-up. Over that stretch the console shows `localizing` and does not update, and a lease held before it can expire. This is the limit the contract states. A follow-up could run the kernel warm-up while the matcher waits.
- **On this map a periodic start can leave the true pose out of the shown candidates** (`val3_000`). The operator then has nothing right to pick. Only the bridge check and a retry from another place help. B-P2d's operator-path judgement should count this case separately.
- The first session's candidate buttons stacked over the map. Fixed in `81727013`, which session 2 ran.
