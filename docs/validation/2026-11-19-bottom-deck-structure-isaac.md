# 밑판 옆면 `structure` 검사 — riib Isaac 확인 (2026-11-19)

계획: [밑판 옆면 structure 를 riib 운영 기본으로](../plans/2026-11-18-bottom-deck-structure-default.md) §3-4. 코드는 PR #600(main `b50140ec`)이다. 실행 스냅숏은 ws1 `snapshots/bd_struct_c42a498f` 로, main 과 트리가 같다(`bd8f0ba4`). 장면 폴더는 `test_runs/riibotics/scenes_side_c42a498f` 다. 그 원본 18 개는 운영 폴더 `scenes_p1c_81a007b1` 와 바이트가 같고, `_side` 사본 12 개가 더 있다.

실행: 묶음 잡 20060·20061(각 shard 4·CPU 8·20G, `--retries 2`), 18 실행. 두 묶음 기록(`bundle_b1_results.jsonl`·`bundle_b2_results.jsonl`)에서 18 실행 모두 첫 시도이고, `infra` 는 비었으며, 재시도와 멈춤은 없다. S− 셋의 rc 1 은 러너 자신의 실패 종료다.

- 운영 스택으로 돌렸다: learned 원거리 인식, SLAM, 결정 모드 아님. `VIDEO=off`·`VIEWS=none` 이다.
- 실행 목록은 [`RUNS.jsonl`](assets/2026-11-19-bottom-deck-structure-isaac/RUNS.jsonl) 이다.
- 묶음 기록은 [`bundle_b1_results.jsonl`](assets/2026-11-19-bottom-deck-structure-isaac/bundle_b1_results.jsonl)·[`bundle_b2_results.jsonl`](assets/2026-11-19-bottom-deck-structure-isaac/bundle_b2_results.jsonl) 이다.
- 판정 요약은 [`summary.json`](assets/2026-11-19-bottom-deck-structure-isaac/summary.json) 이고, 스크립트 [`summarize.py`](2026-11-19-bottom-deck-structure-isaac/summarize.py)가 만든다.
- 실행 폴더(ws1 `test_runs/riibotics/bds1_c42a498f`, 3.3 GB)는 정리 대상으로 넘겼다.

## 결론

| 묶음 | 조건 | 결과 | 계획의 합격 조건 |
|---|---|---|---|
| S+ `line_2_m_side` | 진짜 EPAL 6, `BOTTOM_DECK=auto`(structure) 3 회와 `BOTTOM_DECK=off SIDE=on` 3 회, seed 0–2 | structure 3/3 두 면 `present`(board_in_gaps) → 허가 → 재관측 대조 통과, 완주 3/3. off 완주 3/3 | 합격 |
| S+ `random_1_m_side` | 같음 | 같음 | 합격 |
| S+g `random_1_m_side` | grounded EPAL 6(이웃도 grounded — 전체 grounded 배치), structure 3 회 | 3/3 두 면 `absent`(open_gaps) → 기록 `absent` → 삽입 허가, 완주 3/3 | 합격 |
| S− `random_2_m_side` | `shelf_closed`(이웃도 선반 — 전체 선반 배치), structure 3 회 | 3/3 옆면 단계 전에 끝남. 관측 후보를 모두 쓰고 `observe:all_candidates_exhausted` 로 끝났다. 마지막 원거리 거부 사유는 `perception_no_pallet:support_unverified` 다(종료 문자열의 `success` 여섯은 관측 지점까지의 경로 계획 상태다. 지점마다의 인식 판정은 이 요약에 없다). 삽입 전환은 없음 | 옆면 판정 표본 0 — 판정하지 못함(아래 한계) |

총괄 결정(2026-11-19)으로 S−는 seed 3–5 를 더 돌리지 않는다. 그리고 riib 런처 기본을 `BOTTOM_DECK=auto` 로 바꾼다.

## 시간 비용 (시뮬 시간, 두 팔이 모두 완주한 seed 짝 6)

| 장면 | 첫 접근 시작 structure / off | 차이 | 완주 structure / off | 차이 |
|---|---|---:|---|---:|
| line_2_m_side | 92.3–103.3 s / 13.2–14.3 s | +78.8–90.1 s | 236.3–245.0 s / 154.1–166.3 s | +73.2–82.9 s |
| random_1_m_side | 119.0–121.1 s / 34.6–37.0 s | +81.9–86.0 s | 247.1–253.0 s / 164.6–166.6 s | +81.6–86.5 s |

- 늘어난 시간은 거의 모두 첫 접근 전이다. 옆면 두 곳으로 가서 촬영하고, 첫 원거리 관측 지점으로 돌아와 다시 관측하는 시간이다.
- 삽입 뒤 구간의 차이는 짝마다 ±10 s 안이다.
- 완주 사이클은 약 +50 % 다. 총괄은 자율 우선·속도는 나중 방침에 따라 이 비용을 받아들였다.
- 2026-10-21 기록의 +110–130 s(공장 홀, `on` 모드)보다 작다.
- 줄이는 안은 계획 §5 에 있다(결정 아님).
- `structure` 를 켜면 인식 카메라의 근거리 절단도 1.0 → 0.28 m 로 바뀐다. 이 차이가 첫 원거리 채택 전에 미친 영향은 이 시각들로 가르지 않았다.

## 한계

- **S− 는 옆면 단계까지 오지 않았다.** Isaac 의 `shelf_closed` 는 원거리에서 대상으로 채택되지 않았다. 마지막 거부 사유는 지지대 미확인(`support_unverified`)이다. 그래서 옆면 판정 표본이 0 이다.
  - 옆면 단계가 선반을 거부하는지는 PR ③ CPU 결과(닫힌 선반 넷 72/72 `closed_face` 거부, [2026-10-27 기록](2026-10-27-bottomless-pr3-cpu.md))로 갈음한다.
  - 이 선반이 원거리에서 늘 걸러진다는 뜻은 아니다. 표본은 seed 3 개, 한 장면, 한 선반 종류다.
  - 다리 달린 선반(`shelf_legs`)은 옆면 검사를 통과하는 받아들인 위험이다(총괄 2026-10-28). 이번에 시험하지 않았다.
- 표본은 장면 둘(+grounded 한 장면)과 seed 3 개다. 완주율을 정할 표본이 아니다.
- `aisle*`·`trailer`·`conveyor` 는 옆면 촬영 위치가 막혀 기본에서 꺼진다. 면별 면제(D-F′)와 콘솔 기본은 보류다(계획 §6).
- 영상은 껐다. 운영 기본(영상 켬)과 렌더 부하가 다르다.

## 재현

```bash
# ws1: 장면(원본 + 사본), 목록 검사, 묶음 2 잡
python tools/riibotics_isaac_scenes.py --output <scene dir> --slot-markers --rack --side-room
python3 tools/isaac_bundle.py check --list RUNS.jsonl --runner sim/isaac/run_transport.py --kind riib --jobs 2
BUNDLE_ARGS="--retries 2 --run-est-min 75 --claim-dir OUT/claims" sbatch $(python3 tools/isaac_bundle.py sizes riib --runs 18 --jobs 2 --run-min 75) \
  --output=OUT/b1/bundle-%j.log deploy/slurm/isaac_bundle.sbatch RUNS.jsonl OUT/b1 riib   # b2 도 같은 목록·claim
python docs/validation/2026-11-19-bottom-deck-structure-isaac/summarize.py OUT --json summary.json
```
