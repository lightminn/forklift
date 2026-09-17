# Hybrid A* Transport Implementation Plan

> **For agentic workers:** Use the executing-plans/subagent-driven workflow task by task. Planner and simulator work have disjoint ownership; integration is sequential.

**Goal:** Implement and physically test random pallet pickup and transport through random official warehouse props to a green destination marker, with a global 60fps camera.

**Architecture:** Hardware-independent rear-axle Hybrid A* and path tracking; Isaac Sim adapter owns asset geometry, random scenario, lift/contact mission and recording.

**Tech Stack:** Python >=3.10, NumPy >=1.23, pytest >=7, Isaac Sim 5.1.0, USD/PhysX.

**Spec:** [Design](../design/2026-09-17-hybrid-astar-transport.md)

## Global Constraints

- Keep unmeasured provisional chassis and synthetic dynamics distinct from hardware.
- Flat floor; official NVIDIA warehouse props; randomized pallet, obstacles and destination.
- Full-scene camera; 60fps output, 120Hz physics; green ground circle marks pallet delivery center.
- Use simulator pose feedback; no pose teleportation after initialization, no attached pallet.
- No commits/push requested. Keep prior validation and artifacts intact. Host facts stay outside tracked guidance.

## Tasks

- [x] Planner: failing tests; continuous forward/reverse search and exact feasible goal connector; footprint collision tests and fixed-seed regression.
- [x] Tracking: failing tests for forward, reverse and cusps; measured wheel/steer diagnostics; bounded physical command conversion.
- [x] Scene: discover official props; randomized valid map; green circle and full-view camera; reproducible CLI/source snapshot.
- [x] Mission: unloaded and loaded plans; controlled insertion/lift/withdrawal; obstacle transport; grounded delivery and fork clearance verification.
- [ ] Run local tests and multiple deterministic HPC scenarios, retaining every failure. Record 60fps representative missions and verify video.
- [ ] Review code, run final required checks, save validation with results and source hashes, return code/video links.

## Video-first checkpoint

The user requested the first result video only and deferred the main tests.
Keep the multi-seed physical campaign and final acceptance review pending.
The first recording uses seed 0 and four official warehouse props. Preview
completion must not be reported as full validation. The reviewer identified
missing insertion contact/pocket-clearance validation; this remains open.
Measured-pallet obstacle checks and the steered-tire envelope were corrected
before recording. The command-history ramp and curved-endpoint lookahead have
targeted regression evidence; the complete suite has not been rerun on this
final preview revision.

## 검증 재개

사용자가 기록 버그 수정 후 테스트 벤치와 대규모 검증을 요청했다.
JSON 수치형 버그와 삽입 기하 검사 보완 후 로컬 778개 시험이 통과했다.
물리 100개 seed 작업1025와 계획1000개 seed 작업1026을 제출했다.
대규모 최종 집계·분석은 아직 완료되지 않았다. 실행 기록은
`docs/validation/2026-09-17-isaac-hybrid-astar-bench.md`를 따른다.
