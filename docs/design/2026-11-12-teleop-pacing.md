# 텔레옵 pacing — 허가 계산 비용 줄이기 (설계 2판, 2026-11-12)

배경: [기존 기록 확인](../validation/2026-11-12-teleop-realtime.md). Isaac TELEOP 배율은 0.926 으로 AUTO(0.978)보다 낮다. 페이서 ok 문턱 0.9 를 자주 밑돌고, 중재기가 'pacing' 으로 조작을 거부해 직접 조작 hold 출력이 40–57 % 였다. 회전 hold 의 첫 출력 지연(0.6–1.15 s)도 모두 pacing 이었다. 총괄 결정: 진단 → 수정((가) 비용 줄이기 우선) → 등록 확인 실행.

## 1. 진단 — 무엇이 두 번 도나

벽 1 s 당 비용이다(지연 B 12 세션, `tools/teleop_realtime_report.py` 타이머).

| | AUTO | TELEOP |
|---|---|---|
| 코어의 허가 | `core.permission` 103/s × 0.53 ms = 0.055 s | `core.permission_watch`(`paused_tick` 안) 110/s × 0.41 ms = 0.045 s, 그중 `layer.limit<permission_watch` 0.043 s |
| 중재기의 텔레옵 허가 | — | `teleop.permit` 58/s × 1.11 ms = 0.065 s(그 안 lidar 묶음 0.045 s) |
| `permission.allowed_speed` 호출 | 107/s | 225/s(= 감시 110 + 텔레옵 58 × 양 끝 2) |

- TELEOP 동안 미션은 멈춰(held) 있다. 실행기는 매 틱 `core.paused_tick` → `permission_watch` 를 부르고, 그 안에서 `layer.limit` 가 현재 곡률로 계산된다(`core.py` 5574).
- 같은 틱에 중재기는 텔레옵 허가(`TeleopDrive.step` → `BundlePermit.limit`)를 따로 계산한다.
- 출력은 중재기의 텔레옵 값만 쓴다(`arbiter.step` TELEOP 가지는 코어 후보를 쓰지 않는다). 그래서 TELEOP 동안의 감시 `limit` 결과가 바퀴에 닿는 길이 있는지가 핵심이다(2절).

## 2. TELEOP 동안 감시 결과를 읽는 곳 (코드 확인)

`permission_watch` 가 내는 것과 그 소비자는 다음과 같다.

- `obstacle_hold`(반환): `paused_tick` → 실행기(`run_transport.py` 10714)의 `obstacle_hold`. 멈춘 동안 요청 속도는 이미 0 이고, TELEOP 출력은 중재기가 정한다. ← 확인 필요 1
- `self.last_permission`: 안전 상태의 `blocked.reason`(`core.py` 5995).
  - 콘솔은 `out.permit` 이 없을 때만 그것을 보인다(`state_message`). TELEOP 에서는 `TeleopDrive.step` 이 늘 `permit` 을 돌려준다.
  - `scene_escape`(9150)는 AUTO 장면 탈출에서만 읽는다.
- `obstacle["permission_cap"]`: 명령 조립의 상한이다. TELEOP 에서는 조립 결과를 쓰지 않는다. ← 확인 필요 2
- 부작용:
  - 보정 바뀜 때의 격자 재투영(`refresh`).
  - 출구 경계(`exit_exempt_boundary`).
  - `_note_release_standing`.
  - 재출발 확인 게이트의 `observation_gap`.
  - 층 지연 예산의 `layer_latency_exceeded` → `hooks.require(False)`(실행 실패).
  - 이 부작용들은 `limit` 와 상관없거나(재투영·출구·정지 기록), `limit` 의 `why` 에서 나온다(예산).

## 3. 1판 검토 결과 — (가1)을 앞세우지 않는 이유 (Codex 1판 적대 검토, 총괄 2026-11-12)

- **[P1] `layer.limit` 에는 상태가 있다.**
  - `allowed_speed → check_arc → _start_release` 가 출발 띠 해제 앵커(`release_anchor["best"]`)를 갱신하고, 지우기도 한다(`drive_permission.py` 778). `last_estop` 도 바꾼다.
  - 그래서 감시의 `limit` 를 건너뛰면 다음 판정이 달라질 수 있다. 앵커가 살아 있을 때 간격이 15 → 18 → 16 mm 로 바뀌는 경우, 건너뛰지 않으면 다음 정지 호의 허용 길이가 0 이고 건너뛰면 제한이 없다(Codex 재현).
  - 1판의 "그 틱의 입력만 읽는다" 는 틀렸다.
- **[P2]** 코어 후보는 TELEOP 에서도 조립되고, 그 값이 `safety_state.command` 에 들어간다(`run_transport.py` 10920). 최종 바퀴 출력은 같지만 중간 안전 상태는 달라진다.
- **[P2]** 지금의 텔레옵 허가 58 회/s 는 이미 억제된 값이다. pacing 이 거부하는 동안은 묻지 않기 때문이다(`teleop_drive.py` 219). ok 0.9 로 돌아오면 텔레옵 허가 비용이 약 0.034 s/s 늘어, (가1)의 절감 0.043 s/s 를 대부분 다시 쓴다.
- 확인 필요 1–4 의 답:
  - `obstacle_hold`·`permission_cap` 은 TELEOP 에서도 명령 조립이 읽는다.
  - 재출발 게이트는 TELEOP 에서도 적용된다.
  - `limit` 는 상태를 바꾼다.
- 총괄 요구(2026-11-12):
  - (1) (가1)을 하려면 텔레옵 허가가 같은 명령에 대해 코어 `layer.limit` 보다 느슨하지 않음을 코드 대조와 반례 시험으로 보여야 한다. 느슨한 경우가 하나라도 있으면 그 경우는 건너뛰지 않는다.
  - (2) TELEOP → AUTO 재개 때 묵은 감시 값을 쓰면 안 된다.
  - (3) 확인 실행 기준을 덧붙인다(6절).

## 4. 벤치 — 호출당 비용이 어디에 드나

`tools/permit_bench.py` 를 썼다. live 층 설정(team_spec), 합성 2D 장면, 10 Hz 스캔 1 s, 실시간 실행과 같은 커널(`kernels.prefer_fast()` → numba)이다. 노트북(Intel Core Ultra 5 125H) 기준, 1000 회다.

| 호출 | 평균 | p99 | 최대 |
|---|---|---|---|
| 코어 감시(`layer.limit`, 두 방향) | 0.57 ms | 1.28 ms | 1.63 ms |
| 텔레옵 허가(양 끝 전체, 설치된 호) | 0.55 ms | 1.38 ms | 5.07 ms |
| 텔레옵 허가, 새 snapshot 의 호 재설치(10 Hz) | 2.08 ms | 3.83 ms | 6.13 ms |

- 프로파일상 둘 다 `allowed_speed → check_arc → _walk`(numba 묶음 + 파이썬 감쌈)가 대부분이다.
  - `footprint_cells`(파이썬, 호출당 약 0.07 ms), `_walk_numba` 의 감쌈·배열 준비, `arc_poses`·`meshgrid` 다.
  - `allowed_speed` 한 번에 약 0.25 ms 다.
  - TELEOP 는 한 틱에 `allowed_speed` 를 AUTO 의 약 두 배 부른다(감시 + 텔레옵 양 끝).
- ws1 live 단가(감시 0.41 ms, 텔레옵 1.11 ms — 짐 구역·자세 검사 포함)와 같은 차수다.

## 5. 제안 (2판)

### (가0) `allowed_speed` 한 번의 비용을 줄인다 — 1순위

- 판정 경로를 바꾸지 않고 같은 계산을 싸게 한다. 후보는 다음과 같다.
  - `footprint_cells`·`shape_cells` 의 numba 판(지금 파이썬).
  - 같은 (곡률, 방향, 길이, step) 의 `arc_poses`·격자 좌표를 snapshot 동안 재사용한다.
  - `_walk_numba` 감쌈의 배열 준비를 줄인다.
- 동등성:
  - `tools/perf_bench.py` 의 덤프 재실행으로 바이트 동등(결과·수신자·인자 전후 상태)을 본다.
  - 새 커널은 `reference` 와 같아야 한다(실시간 계획 D2 의 기존 규칙).
  - 시험에 활성 해제 앵커·보정 변경·재정지 경계를 넣는다.
- 효과는 AUTO·TELEOP 모두에 난다. 상태가 있는 판정(해제 이력·`last_estop`·재출발 게이트)을 건드리지 않으므로 Codex P1·총괄 (1)(2)의 문제가 생기지 않는다.
- 러너·코어 파일을 건드리면 재생 게이트 29/29 를 거친다.
- 목표: TELEOP 허가 몫(ok 0.9 기준 약 0.144 s/s = 감시 0.045 + 텔레옵 0.099)을 절반 가까이 줄여, 텔레옵 배율을 AUTO − 0.02 안으로 넣는다. 달성 여부는 확인 실행으로 본다.

### (가1) TELEOP 동안 감시의 `limit` 를 건너뛴다 — 보류

- (가0) 뒤에도 모자랄 때만 다시 설계한다. 조건은 다음과 같다.
  - 출발 띠 해제가 꺼져 있고(런처 기본, #128) 활성 앵커가 없다. `last_estop`·재출발 게이트 등 `limit` 의 모든 상태 변경을 보존한다.
  - 텔레옵 허가가 코어 `layer.limit` 보다 느슨하지 않음을 코드 대조와 반례 시험으로 보인다(총괄 1).
  - 재개 뒤 첫 AUTO 출력 전에 감시를 새로 계산한다(총괄 2).
  - 중간 안전 상태의 허용 차이를 명시한다(Codex P2).

### (가2) 서서 요청이 없을 때 텔레옵 허가를 묻지 않는다 — 보류

- 출력 판정은 동등하다. 다만 보이는 permit 의 시각과 stale 표시를 따로 시험해야 한다. 대기 시간에만 효과가 있다.

### (나) pacing 판정 — 건드리지 않음

## 6. 확인 실행 (결과 전 고정, 별도 등록 문서로)

- 지연 B 와 같은 세션 조건과 프로브 hold 열로, 단독 실행한다. 같은 세션에 AUTO 대조를 둔다.
- 기준:
  - 텔레옵 배율 ≥ 같은 세션 AUTO 배율 − 0.02. 세션별 짝 차이의 중앙으로 본다.
  - 텔레옵 ok(벽 가중) ≥ 0.9.
  - 직접 조작 네 종류(forward·reverse·left·right)의 hold 출력 비율이 종류별 중앙으로 각각 ≥ 0.9 다. loaded_forward·영상 모드는 보고만 한다.
  - 회전 첫 출력: 시작 때 ok 인 hold 의 중앙 ≤ 100 ms. 시작 때 not-ok 인 hold 는 `ok_hold_s` 1 s 때문이라 기준에서 빼고 건수만 적는다.
  - 안전 비열화 0: 허가 거부·정지 기록·층 예산 실패 건수가 같은 세션 AUTO 대조와 기존 lat/b 대비 늘지 않는다.
- 집계·결측 규칙(Codex 1판):
  - 무출력 hold 와 미완주 hold 는 따로 실패로 센다. 중앙값에서 빠뜨리지 않는다(지금 `_med` 는 None 을 뺀다 — 등록 판정 도구에서 고친다).
  - 포함 범위와 세션 제외 규칙(다른 사용자 CPU)은 지연 B 와 같다.
- 등록 문서와 제출은 총괄 신호 뒤에 한다(lat_r_ 와 순서 조정).

## 7. 실차 다리(Jetson, 50 Hz = 20 ms) 추정 (총괄 5번)

- 실차 다리의 바닥 허가는 ⑦b 에서 같은 `BundlePermit` 계산을 쓸 예정이다.
- 노트북 단가: 설치된 호 0.55 ms(p99 1.38), 재설치 2.08 ms(p99 3.83, 최대 6.13)다.
- Jetson Orin Nano Super(A78AE 6코어, 최대 약 1.7 GHz)의 단일 스레드는 이 노트북 P 코어보다 대략 2–2.5 배 느리다고 본다. 공개 단일 스레드 점수 기준의 추정이고, 실측이 아니다.
- 그러면 재설치 틱은 평균 약 4–5 ms, p99 약 8–10 ms, 최대 약 12–15 ms 다. 20 ms 주기 안에는 들지만, 같은 주기의 다른 일(엔코더 속도·조향 등)과 합치면 여유가 크지 않다.
- (가0) 는 Jetson 에도 그대로 효과가 있다. numba 가 aarch64 에서 돌아야 하는데, conda·pip 에 판이 있다. 접속되면 이 벤치를 Jetson 에서 그대로 돌려 실측한다.

## 6. 자체 검토

| 검토 | 지적 | 처리 |
|---|---|---|
| ① 원자료 재계산(Claude) | 타이머 호출 수·단가·벽 1 s 당 비용을 B 12 세션에서 다시 셈 | 1절 표 |
| ② Codex 1판 적대 검토 | P1 `limit` 의 상태(해제 앵커·`last_estop`), P2 중간 안전 상태, P2 pacing 회복 뒤 호출률 증가; 확인 필요 1–4 답; 집계·결측 규칙 | 3절, (가0) 우선·(가1) 보류, 6절 집계 규칙 |
| 총괄 2026-11-12 | (가1) 안전 근거·재개 때 묵은 값 금지·확인 기준 추가 | 3·5·6 절 |
