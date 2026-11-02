# Unconfirmed position: the cost measured before implementation (2026-11-02)

Plan `docs/plans/2026-11-02-unconfirmed-position.md` 1.3 and the lead's decision 2: before any implementation, measure on seeds 4 and 5 how many unattended-confirmed starts the start check (plan 1.2) would hold unconfirmed. More than half in either world would go back to the lead first.

## Setup

- **Code:** snapshot `uc_7a185af9` (branch `feat/unconfirmed-cost`): `forklift_core.localization.twin_check` and `tools/unconfirmed_cost.py`, B-P1b's frozen matcher config.
- **Job:** ws1 14205, 4 runs at once, 7 min 21 s, 1.1 GB RSS.
- **Starts, seed 4:** B-P1b's validation rows the matcher confirmed on its first scan set: 69. b is the top candidate's base pose. Map `9a5cefd5`.
- **Starts, seed 5:** B-P2d's sessions that started on the matcher's confirmation: 68. b is the session's estimate. Map `3edf24b0`.
- **F1's wrong confirmations:**
  - These are the record-only neighbouring-bay and turnaround clicks that the local re-match confirmed at R_AMB 0.98 and that lie more than 0.30 m or 0.10 rad from the truth.
  - Seed 4 has 11 (bay 10, turnaround 1); seed 5 has 10.
- **Scans:** the first F1 capture of each start (`f1_dev/cap_n103`, `f1_s5/cap_n104`). The sessions kept no scan points, so a different noise draw of the same start pose stands in for the session's last scan set.
- **Inputs:** made by `make_believed.py`; rows and summary are in `docs/validation/assets/unconfirmed/`.

## Results

| | seed 4 starts | seed 5 starts | seed 4 F1 wrong | seed 5 F1 wrong |
|---|---|---|---|---|
| checked | 69 | 68 | 11 | 10 |
| **unconfirmed** | **5 (7.2 %)** | **5 (7.4 %)** | **11 (100 %)** | **10 (100 %)** |
| truth within a hypothesis region | 69 | 68 | 10 | 10 |
| rear corridor blocked by survey obstacles (0.62 m) | 14 | 13 | 4 | 4 |
| hypotheses, median / max | 25 / 32 | 25 / 33 | 30 / 32 | 30 / 33 |
| check time (global search + start check), p50 / p95 / max | 5.9 / 10.8 / 15.8 s | 5.6 / 11.5 / 18.5 s | 8.3 / 12.6 / 12.6 s | 10.3 / 16.5 / 16.5 s |

- **The 50 % gate passes in both worlds:** 7.2 % and 7.4 %. Implementation goes ahead (the lead's decision 2).
- **Completeness:** every correct start's truth lies in its own region (b is the truth there). One F1 wrong confirmation's truth is outside the set (val3_019 bay−y); it is caught anyway.
- **What catches the wrong confirmations:**
  - All 21 are unconfirmed.
  - In at least 19 a competitor comes from another hypothesis's region: the pool or a synthetic twin, 6 m or about 30 m away. The stored list is cut at 20 competitors, so this is a lower bound.
  - The rest are caught by leaves 0.1–0.3 m inside b's own region: a wrong start fits worse (s(b) 0.74–0.99, median 0.84 / 0.94), and its conservatively scored neighbours reach 0.98·s(b).
- **The correct starts held unconfirmed (10):** all by competitors 0.06–0.28 m from b inside b's own region. 1 also has F1's in-window ambiguity, and 4 also have a twin-region competitor. s(b) is 0.904–0.992 over all correct starts.
- **Rear corridor:**
  - Survey obstacles alone block a straight reverse of 0.62 m at 14 of 69 and 13 of 68 starts. Among the starts held unconfirmed it is 2 of 5 in each world.
  - The pallet zones are not in the survey map, so the real figure can only be higher.
  - A blocked unconfirmed start ends the session (plan, the lead's decision 8). On these numbers that would be about 3 % of unattended-confirmed starts plus whatever the pallet zones add.
- **Time:**
  - The check runs after the matcher's own search and adds about 3–8 s on ws1 (one process of four).
  - With the search it took at most 18.5 s here. A Jetson about three times slower would approach the 30 s matcher deadline: record only, as F2's note.
