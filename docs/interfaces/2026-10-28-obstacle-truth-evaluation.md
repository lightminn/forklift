# 장애물층 truth 평가 지표 — 결과 계약 (2026-10-28)

`run/result.json` 의 `obstacle_layer` 아래 truth 평가 키의 뜻이다. 평가만 하는 지표이고, 주행 판단에는 쓰이지 않는다. 코드는 `sim/isaac/run_transport.py`(운행 중 평가)와 `sim/isaac/obstacle_layer.py` 의 `stop_arc_hits` 에 있다.

## 무엇을 세나

허가가 작용하는 단계에서, truth 속도가 0.05 m/s 를 넘는 틱마다 센다.
- truth 후축 자세에서 그 틱의 조향(`kappa`)과 이동 방향으로 정지 원호를 그린다. 원호 길이는 허가의 정지 거리이고, 0.025 m 간격으로 샘플한다.
- 원호 위의 모양(+ envelope)이 truth 장애물 사각형과 만나면 그 물체를 센다. 장애물은 장면 소품 전부다. 낮은 소품도 포함하고, 복귀 때는 측정한 하역 팔레트를 더한다. 접근·삽입의 픽업 팔레트는 빼는 경우가 있다.
- 그 틱에 허가가 truth 속도 이상을 허용했으면 `unpermitted` 에도 넣는다.

이 지표는 **접촉 기록이 아니다.** "그 순간 멈춰야 했다면 정지 경로가 닿았을 수 있다"는 정지 부피 검사다. 실제 접촉은 별도의 현재 자세 검사(차체 + 날)와 물리 접촉 기록으로 본다.

## 키

| 키 | 뜻 | 모양 |
|---|---|---|
| `truth_events` | 위 검사에 걸린 (틱, 물체) 수 | hull(`footprints(loaded)` — 방향 없음). 빈 차이면 포크 둘레 빈 공간까지 덮는 외곽 |
| `event_objects` | 한 번이라도 걸린 물체 수 | hull |
| `unpermitted_entries` | 그중 허가가 그 속도를 허용한 (틱, 물체) 수, **총수** | hull |
| `unpermitted` | 그 항목 목록, **처음 50 건만** | hull |
| `unpermitted_truncated` | `unpermitted_entries` > 50 이라 목록이 잘렸는지 | — |
| `truth_events_dir` | 같은 검사, 허가가 그 이동 방향에서 쓰는 모양 | `footprints(loaded, 이동 방향)`. 빈 차 후진이면 차체 + 날(hull_forward), 그 밖에는 hull 과 같다 |
| `event_objects_dir` | 같은 모양으로 걸린 물체 수 | 이동 방향 모양 |
| `unpermitted_dir_entries` | 같은 모양의 허가 항목 총수 | 이동 방향 모양 |
| `unpermitted_dir` | 그 목록, 처음 50 건 | 이동 방향 모양 |
| `unpermitted_dir_truncated` | 그 목록이 잘렸는지 | — |

- 기존 키(`truth_events`·`event_objects`·`unpermitted_entries`·`unpermitted`)는 뜻과 값이 바뀌지 않는다. hull 은 이동 방향 모양을 포함하므로 `_dir` 수는 hull 수보다 크지 않다.
- 위 표의 "hull" 은 장애물층 기본 모양 설정(`permission.shape: hull_forward`, riib·CT·콘솔 설정 모두)일 때다. `shape: parts` 로 돌린 실행에서는 `footprints(loaded)` 자체가 차체 + 날이라 기존 키도 차체 + 날을 세고, `_dir` 키와 같아진다.
- `_dir` ⊆ hull 은 (틱, 물체) 수와 총수에 대한 관계다. 각각 처음 50 건으로 자른 두 목록(`unpermitted`·`unpermitted_dir`)은 서로 부분집합이라는 보장이 없다.
- 기존 키를 읽는 도구(2026-10-28 grep): `tools/s4_judge.py`(`unpermitted`·`unpermitted_entries`), `tools/slam_async_ab.py`(`unpermitted` 목록 길이), `tools/branch_coverage.py`(경계 기록의 `obstacle.unpermitted` 증가 — `obstacle.unpermitted_dir` 도 같은 정규식에 걸리지만, `_dir` 항목이 생긴 틱에는 hull 항목도 생기므로 hull 목록이 그 틱에(50 건이 찼으면 그 전에) 이미 늘어, 항목 판정은 같다), `tools/summarise_p0b.py`·`tools/p0b_sensor_study.py`(자기 집계의 같은 이름 키, 이 결과 키가 아니다). 모두 기존 키만 읽으므로 영향이 없다.
- 2026-10-28 이전 기록에는 `_dir`·`_truncated` 키가 없다. 그런 기록에서는 `unpermitted_entries > len(unpermitted)` 이면 잘린 것이다.

## 왜 둘인가

빈 차가 후진할 때 허가와 저상 감시는 차체 + 날로 판정한다(`obstacle_layer.footprints`, `low_clearance`). hull 은 날 바깥과 옆면 사이 빈 띠까지 채운다. 그래서 hull 지표는 허가가 원래 보지 않는 띠에 걸린 것까지 센다.

2026-10-28 리보틱스 aisle_rack 복귀(0.6 m/s 후진)의 "물체 9"(높이 0.144 m 블록)가 그 경우였다. hull 은 0 mm 까지 갔지만 차체 + 날은 truth 로 65 mm 이상 떨어져 있었고, 접촉은 없었다(`docs/validation/2026-10-28-escape-presteer.md` 와 Codex 재계산). 평가기 주석은 원래 "허가가 검사하는 모양"이라고 적혀 있었는데, 코드는 방향을 넘기지 않아 hull 을 썼다.

## 판정 규칙에서

이미 사전 등록된 판정은 등록된 키 그대로 둔다(예: C4″ 의 "실제 겹침" 은 hull 키). 다음 계획부터 어느 키로 판정할지는 계획마다 정한다(총괄 2026-10-28).
