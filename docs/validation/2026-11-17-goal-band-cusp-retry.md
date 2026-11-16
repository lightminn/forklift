# 목표 사각 띠 복구·cusp 재계획 한 번 다시 — 결정 모드 짝 결과 (2026-11-17)

- 계획: [docs/plans/2026-11-17-goal-band-and-cusp-retry.md](../plans/2026-11-17-goal-band-and-cusp-retry.md). 코드: PR #569(머지 7ae025ef, 두 옵션 기본 off).
- 실행: ws1 잡 19566, 스냅숏 `gbc_aa27990b`(코드 = aa27990b), riib random_1, 한 묶음(3 슬롯·shard 4). 실행 목록은 [make_runs.py](2026-11-17-goal-band-cusp-retry/make_runs.py)로 만들었다. 출력 ws1 `test_runs/gbc/gbc_aa27990b/`. 판정에 쓴 줄(8 실행의 인자·실패 사유·정체·재계획 기록)은 [results_excerpt.jsonl](2026-11-17-goal-band-cusp-retry/results_excerpt.jsonl)에 뽑아 두었다.
- 끔 팔은 두 옵션 모두 끔, A 켬 팔은 `--goal-creep-band on` 만, B 켬 팔은 `--cusp-replan-retry on` 만 켰다. 결정 모드(`--deterministic`) 6 실행이다.

## 1. 결과

| seed | 끔 | 켬 | 옵션 발동 |
|---|---|---|---|
| s24 (A) | Tracking timeout in return_home (기준선 재현) | **완주** | 206.74 s 띠 진입 고정(`band: true`, 남은 78 µm·명령 0.80 mm·오도메트리 0, 821 틱) → 207.93 s 정체 재계획 성공(0.89 m, 기어 변경 1) |
| s11 (B) | scene_replan_refused:cusp_pose:blocked:occupied (재현) | scene_replan_refused:cusp_pose:**retry_straight_collision** | 133.16 s 첫 후보 blocked:occupied → 막힌 칸 사각형 5 개로 다시 계획 → 하역 직선 충돌 |
| s49 (B) | 같음 (재현) | 같음(retry_straight_collision) | 130.73 s, 사각형 7 개, 같은 실패 |

- **끔 재현:** 세 seed 모두 새 스냅숏에서 기준선과 같은 사유로 끝났다. 켬 결과를 효과 증거로 읽을 수 있다.
- **A:** 정체 재계획 시점(207.93 s)의 위치 오차 8.103 mm·방향 오차 −0.0042 rad 는 #567 진단(8.100 mm)과 같은 정지다(띠 고정 시점의 오차는 따로 기록되지 않는다). 옵션은 이 정지를 R2 로 고정했고, 기존 정체 재계획이 받아 완주했다. 새 실패 사유는 없다(정체 재계획 1 회를 썼다). 끔·켬 모두 134.76 s 의 cusp_pose 재계획 설치까지 같다.
- **B:** 두 seed 모두 옵션이 발동했다(첫 사유 blocked:occupied, 재시도 1 회, 예산 그대로). 재시도 계획은 탐색 자체는 끝났지만, 고정 하역 직선(`predelivery → delivery`, 적재 외곽·계획 여유)이 추가한 사각형과 겹쳐 `straight_collision` 으로 실패했다. 계획대로 escape 없이 `retry_straight_collision` 으로 거부됐다.

## 2. B 의 '대체 경로 없음'이 뜻하는 것

- `straight_collision` 은 탐색 시간·확장 한도에 걸린 실패가 아니다. 탐색은 하역 앞 자세까지 성공했고, 그 뒤 `plan_transport_leg` 가 붙이는 고정 하역 직선(`predelivery → delivery`)이 추가한 사각형과 겹쳤다(탐색 앞 출구 직선의 `unmet:straight_collision` 과는 다른 분기다).
- 그래서 확인된 것은 **같은 점유 사각형·적재 외곽·계획 여유와 고정 하역 직선을 유지하는 이 재계획 후보군 안에서는 경로가 없다**는 것까지다. 다른 접근이나 다른 하역 자세로 물리적으로 도달할 수 없다는 증명은 아니다. 여유는 계획기의 `clearance_m` 이고, 사각형 구성(칸을 넓히는 `grow`)도 결과에 관계한다.
- 막힌 칸의 격자 인덱스는 후보 기록(`candidate.permit.cells`)에 남아 있다. 그러나 점유 스냅숏과 실제로 넣은 사각형의 좌표·크기가 기록되지 않아, 정확한 장애물 입력과 칸의 정체(칸 표지·벽의 팽창인지, SLAM 위치 오차 10 cm 안팎에서 생긴 칸인지)는 복원하지 못한다.
- 정리하면, s11·s49 에서 층은 하역 직선 근처에 점유를 보고 있다. 그 칸이 실제 장애물인지, 사각형·여유·하역 직선 정책 가운데 무엇을 바꾸면 풀리는지는 이번 결과로 가릴 수 없다.

## 3. 켬 경로 재생

- 결정 모드는 경계 기록과 함께 쓸 수 없어서, 비결정 2 실행(A 켬 s24, B 켬 s11)을 `--record-boundary --record-boundary-format 2` 로 따로 기록했다. 둘 다 완주했다.
- 비결정이라 경로가 갈라졌다. A 켬 s24 는 띠 정지가 없었고(goal_creep_stalls 없음), B 켬 s11 은 173.37 s 의 cusp_pose 후보가 처음부터 통과해 재시도가 없었다. 그래서 이 재생은 **켬 인자·기록 인코딩**을 확인할 뿐, 발동한 분기의 재생까지 확인하지는 않는다.
- 재생 결과(ws1 잡 19567, 같은 스냅숏, `SETUP_INPUTS=recorded`): 두 기록 모두 `REPLAY_EXIT 0`.

## 4. 기본 켬 권고

- **A `--goal-creep-band`: 기본 켬을 권고한다(장면 제한 없이 riib 런처부터).** 대상 정지를 기준선 그대로 재현했고, 켬에서 완주했다. 폐루프 시험의 기존 반례와 20 % 수렴 접근도 가로채지 않는다. 남는 위험은 계획에 적은 대로, 띠 안에서 명령의 1/20 보다 느리게 수렴하는 정상 접근이다. 이번 증거는 seed 하나다. 런처 기본으로 올리면 #442 규칙대로 실제 Isaac smoke 뒤에 머지한다.
- **B `--cusp-replan-retry`: 기본 켬을 권고하지 않는다.** 발동은 했지만 두 seed 모두 완주로 바꾸지 못했다. 실패 사유만 `blocked:occupied` 에서 `retry_straight_collision` 으로 바뀌었다. 옵션은 끔으로 남겨 둔다. 이 사례를 더 보려면 먼저 하역 직선 근처 점유 칸의 정체를 확인해야 한다(`--occupied-dumps` 로 스냅숏과 칸을 남기는 실행 하나).

### 총괄 결정 (2026-11-17)

- **A:** riib 런처 기본 켬(`GOAL_CREEP_BAND=on`, `MODE=slam` 의 모든 장면; `MODE=truth` 는 끔). 러너 기본과 라이브 런처는 끔으로 둔다. 런처는 적용한 값을 `riib_defaults.txt` 에 남긴다. **근거는 seed 1 개(random_1 s24)**이고, 띠 안에서 명령의 1/20 보다 느리게 수렴하는 접근을 가로챌 위험은 남는다. 이전 riib 배치는 끔이었으니 비교할 때는 두 팔의 값을 적고 섞지 않는다. near-tracking 계약은 끔에서 측정했으므로 `--near-tracking` 은 `GOAL_CREEP_BAND=off` 를 요구한다.
- **B:** 끔으로 종결한다. `--occupied-dumps` 추가 실행은 하지 않는다(70 중 2 건이고, 칸의 정체가 SLAM 오차라면 실측 뒤 일이다).
- Isaac smoke: SMOKE_PLACEHOLDER

## 5. 범위와 한계

- seed 셋, 결정 모드 한 번씩이다. 드문 사례(70 중 1–2)라 큰 비교 배치는 돌리지 않았다.
- A 의 완주는 띠 정지 하나를 푼 결과다. 같은 띠에 걸리는 다른 실행이 얼마나 되는지는 세지 않았다.

## 6. 검토

- **Codex 결과 검토(새 스레드).** P1 없음. tail 직선 해석(탐색 성공 뒤 고정 하역 직선, 출구 직선 분기와 구분)과 A·B 권고를 지지했다. 반영한 것: P2 '대체 경로 없음'을 같은 점유·외곽·여유·고정 하역 직선의 재계획 후보군으로 한정하고 '재계획 규칙으로 안 풀림'을 철회, P3 칸 인덱스는 기록에 있고 없는 것은 스냅숏·사각형 좌표, A 오차는 정체 재계획 시점 값, 판정 발췌를 저장소에 보관.
