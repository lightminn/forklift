# M-a2 evidence: source tests to behaviour tests (2026-10-16)

Plan: `docs/plans/2026-10-16-m-a2-source-tests-to-behaviour.md` (2nd edition, section 3 acceptance).

## Revisions

- Frozen check list: `docs/validation/2026-10-16-m-a2-assertions.json` (base `a438c3a`, built by
  `tools/ma2_assertions.py`; 920 checks in 156 tests, 17 files).
- Property map: `tests/ma2_property_map.json`, built from `tools/ma2/batch_*.py` by `tools/ma2/build_map.py --complete`.
- Implementation commits: `293153d` (checker and mutation tool) .. `2032d1e` (batch 7, map complete, nine source
  tests deleted). **No production change**: `git diff 5faf5e9..2032d1e -- src sim` is empty, so the section 2.6
  gate (M-a1 replay of the 29 records) does not apply.

## Acceptance (final, `293c90f`)

| Criterion | Result | Evidence |
|---|---|---|
| Every frozen check accounted for; every property D/N/R/S/X; D/N with mutation evidence | 405 properties, 920/920 checks, 0 problems | `python tools/ma2_property_check.py --complete` |
| Mutation evidence re-run on the final tree | every D/N property (257): baseline passed and the mutation failed on a check | ws1 Slurm job 3174 (`tools/ma2_mutate.py --write` on a clone of `293c90f`), evidence written into the map |
| Checker self-test | passes | `tests/unit/tools/test_ma2_property_map.py` |
| ws1 full suite: the known failures, nothing new | 10 failed, 3572 passed, 19 skipped at `f30cc56` (`-rfE tests`); 10 failed, 3494 passed, 18 skipped at `293c90f` (`--ignore=tests/simulation`, with coverage); the failed set equals `2026-10-16-m-a2-known-failures.txt` both times | `~/bin/forklift-ws1-pytest -rfE tests`; ws1 job 3173 |
| `core.py` coverage does not fall | 2121 statements, 650 branches: missed lines 25 -> 10, partial branches 41 -> 36 (97 % -> 98 %) | ws1 jobs 3121 (`f3d0812`) and 3173 (`293c90f`), `pytest tests --ignore=tests/simulation --cov=forklift_core.mission --cov-branch` |
| Codex implementation review | converged at round 6 (no new P1/P2) | rounds 1-5: P2 5, 5, 3, 4, 1 -- see below |

The coverage runs ignore `tests/simulation`; both sides do, so the comparison is like for like.

## The core sweep

Codex kept finding core-side conditions no behaviour test held, one sample at a time, so `tools/ma2_core_sweep.py`
mutates every core place the frozen source tests read (each first line of an asserted string of 12+ characters,
every occurrence up to ten; at the base revision, deleted tests included): drop an and/or operand, force an if test,
`pass` a simple statement, and with `--operators` move a comparison across its boundary, narrow a membership, swap
dependent neighbouring statements. Survivors got behaviour tests; `--emit` turns them into properties (batches 9-11).
Final state: every survivor is either killed, the unreachable backoff site (X), or one of 138 swaps judged equivalent
under the rule in `2026-10-16-m-a2-sweep-equivalent.json` (each with its category; Codex round 6 re-checked them).

## Classes

| Class | Properties | Meaning |
|---|---|---|
| D | existing behaviour test guards it | mutation evidence on that test |
| N | new behaviour test | mutation evidence on the new test |
| R | runner wiring, order, constants, cross-file order kept as a runner/source test | the test kept as written |
| S | structural ban kept as an AST check | the test kept as written |
| X | not a property (a path used as data, or unreachable code) | reason recorded |

Counts in the final map: D 124, N 133, R 126, S 2, X 20 (`jq '[.properties[].class] | group_by(.) | map({(.[0]): length}) | add' tests/ma2_property_map.json`).

## Notable decisions

- **Unreachable site** (`B7:s2-site-backoff`, X): `realign_install("backoff")` sits under `if ms.phase == "observe"`
  inside `blocked_replan`'s backoff branch, which only runs in `transport` and `return_home`. No behaviour can show
  it; it is dead code carried over from the runner, left for a production change with its own gate.
- **Deleted source tests**: only the nine whose every check maps to D/N (seven in
  `test_isaac_transport_planning_config.py`, one in `test_isaac_truth_audit.py`, one in
  `tools/test_d7c_round2_prefix.py`). Tests mixing runner and core sides stay as R beside the D/N properties of their
  core side; X tests are other tools' tests and stay.
- **Fakes that assert on an unexpected call** (blocked replan, docking stall, cusp/observe planners): a mutation that
  adds a planning call now fails a check instead of an `IndexError`.

## Codex review (implementation)

- Round 1 (`99727c1`): P1 0, P2 5 -- behaviour the R side still held or a mutation did not prove: the core's mission plan
  uses the recorded planner config object; failed and escape replans never count towards the replan limits (a
  deleted source test checked it); the same path is judged before the limits (the old mutation repeated the
  same-path filter); only an observation leg arrives loosely (a deleted source test checked it); the escape budget's
  upper bound per leg. Each got a new behaviour test and a mutation that breaks exactly it (`B2:mission-plan-config`,
  `B3:failed-escape-not-counted`, `B3:same-path-before-limit` re-pointed, `B3:loose-arrival-observe-only`,
  `B4:escape-budget-bound`); the X verdict on the backoff site and the batch-0 X list were confirmed.
- Rounds 2-6: P2 5 (each count and excluded kind, the cusp replan's leg and config, no escape in a re-approach, the
  retry's order), 3 (the return leg's escape, the small jump's bound, the release's order), 4 (orders first judged
  equivalent: the refusal before the wait, a callback's import, the paths file's payload, the release's poses), 1 (a
  refused install leaves the new follower unrun), then none. Each became a behaviour test and a mutation that breaks
  exactly it; from round 2 on the core sweep looked for the same kinds everywhere.
