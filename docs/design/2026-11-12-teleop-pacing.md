# 텔레옵 pacing — 허가 계산 중복 줄이기 (설계 1판, 2026-11-12)

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

## 3. 제안

### (가1) TELEOP 동안 감시의 `limit` 만 건너뛴다

- 조건: 중재기 모드가 TELEOP 이고 그 틱이 `paused_tick` 일 때다. 실행기가 `paused_tick(..., teleop=True)` 로 넘긴다.
- 건너뛰는 것: `_direction_limits(... layer.limit ...)` 계산만이다.
- 그대로 두는 것:
  - 재투영, 출구 경계, 정지 기록.
  - 예산 감시: `layer.watchdog(t)` 를 직접 불러, `layer_latency_exceeded` 면 지금과 같이 `hooks.require(False)` 를 부른다. `layer_overdue` 의 바퀴 정지는 텔레옵 쪽 `live_permit` 이 이미 `watchdog` 으로 0 을 준다.
  - 재출발 게이트(`gate_` 는 TELEOP 에서 적용되지 않는지 확인, 확인 필요 3).
- 값:
  - `last_permission` 은 `(0.0, "teleop")` 로 둔다. TELEOP 동안 코어의 허가는 판단에 쓰이지 않는다는 표시다.
  - `permission_cap` 은 None 이다.
  - `obstacle_hold` 는 False 다(출력은 중재기가 정한다).
  - TELEOP 를 떠나는 다음 틱부터 감시가 원래대로 계산한다.
- 기대 효과: TELEOP 의 벽 1 s 당 약 0.043 s 를 줄인다. 그러면 허가 몫이 AUTO 수준(0.055 대 0.065)이 된다.
- 동등성:
  - (ⅰ) TELEOP 가 아닌 모든 틱은 바이트 동등이다. 재생 게이트 29/29 로 보인다(배치 기록에는 중재기가 없다).
  - (ⅱ) TELEOP 틱은 판정 동등이다. 중재기 출력(속도·곡률·승강 목표·permit), 정지·래치, 안전 상태 중 `blocked` 외의 필드가 같아야 하고, 이것을 시험으로 고정한다.
  - (ⅲ) TELEOP 를 떠난 뒤의 첫 감시 틱은 건너뛰지 않았을 때와 같은 `limit` 를 낸다(`limit` 는 그 틱의 입력만 읽는다 — 확인 필요 4).

### (가2) 서서 아무 요청도 없을 때 텔레옵 허가를 묻지 않는다

- 조건: `TeleopDrive.step` 에서 측정·직전 적용·요청 속도가 모두 0 이다.
- 출력은 `slew_cap_stop(0, 0, …)` = 0 이라 허가 값과 상관없다(판정 동등).
- 보이는 permit 은 직전 계산값을 그대로 둔다. 표시 사유가 낡을 수 있어서 `stale` 꼬리표를 단다.
- 효과는 TELEOP 대기 시간에만 난다(B 자료로 TELEOP 시뮬 시간의 약 1/3).
- 2순위다. (가1)로 모자랄 때 더한다.

### 하지 않는 것

- (나) pacing 판정(문턱 0.9·창 1 s·ok 1 s 유지·0.25 s 멈춤)은 안전 게이트다. (가)로 모자랄 때만 따로 설계하고, 독립 검토 3 회와 총괄 결정을 거친다.

## 4. 시험

- 코어 단위:
  - `paused_tick(teleop=True)` 는 `layer.limit` 를 부르지 않는다(호출 계수).
  - 재투영·출구·정지 기록은 `teleop=False` 와 같다.
  - `layer_latency_exceeded` 면 `hooks.require(False)` 를 부른다.
  - `last_permission == (0.0, "teleop")`, `obstacle_hold` 는 False 다.
- 다음 틱 `teleop=False` 의 `limit` 결과는 TELEOP 틱을 건너뛴 경우와 안 건너뛴 경우가 같다.
- 중재기·코어 묶음(가짜 층):
  - 같은 텔레옵 명령열에서 출력열(속도·곡률·승강·permit)이 (가1) 전후로 같다.
  - 안전 상태는 `blocked` 만 다르다.
- 재생 게이트 29/29.
- 성능(벤치):
  - 같은 묶음 시험에서 TELEOP 틱당 `layer.limit` 호출이 0 이다.
  - 텔레옵 허가 단가를 노트북·ws1 에서 오프라인 벤치로 잰다. 그 수치로 Jetson 20 ms 주기 안에 드는지를 추정한다(총괄 5번).

## 5. 확인 실행 (결과 전 고정, 별도 등록)

- 지연 B 와 같은 세션 조건과 프로브 hold 열로, 단독 실행한다. 같은 세션에 AUTO 대조를 둔다.
- 판정 기준(안):
  - 텔레옵 배율 ≥ AUTO 배율 − 0.02.
  - 텔레옵 ok(벽 가중) ≥ 0.9.
  - 직접 조작 hold 출력 비율(종류별 중앙) ≥ 0.9.
  - 회전 hold 첫 출력 중앙 ≤ 100 ms.
- 등록 문서와 제출은 총괄 신호 뒤에 한다(lat_r_ 와 순서 조정).

## 6. 자체 검토

| 검토 | 지적 | 처리 |
|---|---|---|
| (작성 중) | | |
