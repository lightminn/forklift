# U3e-1b: 실차 포켓 구간에서 경로 끝 감속이 서 있는 차의 속도 상한보다 낮아져 생기는 정체

- 상태: 계획(2026-11-27). 총괄 지시: 진단을 Codex 원자료 교차검증으로 확정 → 수정 → 운영 파일 정상 경로 4 CPU 20 회 통과가 머지 조건. 이 PR 머지 전에는 U3e-2(잠금 제거)를 시작하지 않는다.
- 관련: `docs/plans/2026-11-26-real-operating-assembly.md` §6.1(U3e-1, #655), `docs/design/2026-11-19-real-safety-wiring.md` §7.1(두 방향 질의) 4 항 "면제 밖(층)도 같은 비교".

## 1. 무엇이 일어났나

`tests/integration/test_real_run_operating.py::test_the_operating_run_drives_to_the_lift_stops_and_disarms`(운영 REAL 실행, pty 위 `mcu_model --hil-plant 0`, D5 상한 0.06 m/s, `d5_pocket` 우회)가 140708bc 에서 23 회 중 2 회 실패했다. 원인을 본 것은 ws1 잡 21634 의 1 회다. 앞선 1 회(21629)는 기록이 지워져 원인을 모르지만, 같은 시험·같은 단계(승강 전 `lifted` 확인을 기다리는 중)에서 멈췄다.

- 콘솔 상태: permit `LATCHED`, 사유 `exception:RealRequireFailed:pocket_stop in approach: residual_over_allowed`.
- 기록(ws1 `test_runs/u655x10/test_the_operating_run_drives_0/run/run.jsonl`):
  - 접근(`approach`) 끝점은 rear 0.910 m, 도착 허용오차는 8 mm다.
  - 차는 rear 0.89874 m(끝점까지 11.26 mm)에서 섰다.
  - 그 뒤 매 틱 `hold:residual_over_allowed` 로 0 DRIVE 를 쓰고, 보드는 매번 그것을 확인했다.
  - 4.98 s 동안 hold 가 풀리지 않아 `core.py` 의 `hooks.require(work_elapsed(zero_since) < 5.0, "pocket_stop in …")` 가 실행을 끝냈다.
- 통과한 실행에서도 같은 구간에서 0.02–0.1 s 짜리 같은 hold 가 수십 번 생겼다가 풀린다(같은 잡의 1 회차는 29 구간, 최장 5 틱). 그 실행은 0.90221 m 에서 `insert` 로 넘어갔다.

## 2. 원인 (Codex 교차검증, 원자료 한 쌍 재계산으로 확정)

판정을 낸 곳은 실차 포켓 질의 `MissionCore._pocket_query` 의 **면제 밖 재질의**(설계 §7.1 4 항)다: `layer_fn(req, motion[req])` 가 `motion[req]` 보다 작은 양의 허용치를 `ok` 로 돌려주면 hold, 사유 `residual_over_allowed`.

- **`motion[req]` 쪽은 출력 잔여가 아니다.** 마지막 비영 DRIVE(+0.006 m/s, 명령 가속 0.30 m/s² × 0.020 s 의 한 틱)는 100 ms 뒤 만료된다. 그 뒤 hold 프레임은 모두 0 속도로 확인됐고, `_commanded_now()` 도 0 이다.
- **엔코더 안전 속도의 상한이 남는다**(서 있어도 0 이 아니다, `real_teleop.EncoderSpeedParams.error`).
  - 상한 = `axle_accel × (1 + κmax·track/2) × (dt/2 + age) + 한 카운트`, 실 런처 값으로 `axle_accel = 0.5`, `dt ≈ 0.020 s`.
  - `age` 는 STATE 수신 나이에 `DelayBound` 의 지연 상한을 더한 값이다.
  - 실패 틱은 지연 상한 27.8 ms → 상한 ≥ 0.02086 m/s.
  - 통과 실행의 같은 자리(0.89876 m) 틱은 지연 상한 18.3 ms → 0.01565 m/s.
- **층의 허용치 쪽은 경로 끝 감속이다**(`DrivePermission.allowed_speed`).
  - 경로가 끝에 닿으면(`reached_end`) 허용치는 `max(speed_for(path_left + margin), end_creep_mps if path_left > 0)` 이고, `end_creep_mps = 0.02` 다.
  - 끝점 가까이에서는 이 값이 약 0.02 m/s 로 내려간다(11.26 mm 에서 약 0.0208, 기록이 아니라 직선 잔여거리로 복원한 값).
- 그래서 지연 상한이 약 27 ms 를 넘는 틱이 이어지면, **서 있는 차의 속도 상한이 경로 끝 감속 허용치보다 커서** hold 가 풀리지 않는다. 차는 도착 허용오차(8 mm) 밖에서 서고, 5 s 규칙이 실행을 끝낸다. 지연 상한이 낮은 틱이 끼면 그 틱에 `auto` 로 풀려 조금 전진하므로 통과 실행은 지나간다.
- **U3e-1 의 코드 변경 탓이 아니다.** #655 병합 직전 main(b1746d11)과 비교해 미션 코어·`OutputBook`·엔코더 속도·장애물 층·틱 스케줄러는 같다. 운영 경로를 처음 끝까지 돌린 시험이 이 지연 민감성을 드러냈다. 시험의 기동 절차가 지연 분포에 영향을 줄 수는 있다(배제 못 함).
- **지연 상한이 실행 내내 높았던 이유는 위상이다.** 실패 실행의 STATE 지연 상한 중앙값은 28.1 ms, 통과 실행은 20.0 ms 다. 치명 hold 5 s 동안 한 틱에 읽는 STATE 두 개가 38.3 / 27.8 ms 로 매 틱 똑같이 반복됐다. 호스트 20 ms 틱과 MCU STATE 주기가 한 위상으로 맞물려 그대로 유지된 것이다. 위상은 기동마다 정해지므로 같은 지연·거리 조건이면 실차에서도 재현될 수 있다. 28 ms 는 정상 범위 안이다: 엔코더 안전 속도는 수신 나이 + 지연을 100 ms(`max_age_s`)까지 받는다(스캔 배치의 STATE 지연 한도 50 ms 와는 다른 한도).
- **첫 hold 가 무엇이었는지는 기록만으로 가릴 수 없다**(Codex 2 회차 P1). 정지 카운트에서 엔코더는 같은 상한을 양 방향 `(1, -1)` 에 둔다. 전진 요청이면 `_direction_limits` 가 후진 가능 운동을 먼저 층에 묻는다. 층은 방향별로 정지 호를 다시 계산하지만 경로 끝 감속은 같은 설치된 `path_check` 에서 오므로 후진 질의도 같은 경로 끝 허용치를 받아 `direction_unsure` 가 된다. 이어지는 포켓 재질의가 `motion_hold` 를 `residual_over_allowed` 로 덮어쓴다. Codex 가 이 worktree 의 `encoder_speed`·`DrivePermission`·`MissionCore.permission` 으로 한 틱을 재현했다(빈 격자, D5 우회, 끝까지 11.26 mm, 카운트 변화 0, 간격 20 ms, 지연 27.8 ms): 엔코더 상한 0.0208557, 경로 끝 상한 0.0208202 m/s, 현재 코드는 `residual_over_allowed`·0. 전진 재질의만 바꾸면 `direction_unsure`·0, 두 방향의 가능 운동을 모두 정지 호로 보면 `ok`·0.0208202.

## 3. 수정

**원칙: 경로 끝 감속은 정지 공간 증거가 아니다.** `allowed_speed` 의 주석 그대로("Stopping at the goal needs no margin past it; the arc check still covers the actual stopping volume beyond the end"), 경로가 끝까지 검증됐고 막히지 않았을 때의 `path_limit` 은 끝점에 맞춰 서기 위한 목표 감속이다. 끝점 너머의 정지 부피는 지금 곡률·방향의 정지 호 검사(`check_arc`, `arc_limit`)가 맡는다. 설계 §7.1 의 가능 운동 비교가 지키려는 것은 "지금 있을 수 있는 운동의 정지 호가 덮였는가"다.

**그러나 모든 가능 운동에 정지 호만 쓰면 안 된다**(Codex 2 회차 P1-2·agy 2): 가능 운동에는 서 있는 읽기의 오차폭뿐 아니라 **주행 중 측정 속도의 상한**, **지금 효력 있는 명령**, **미확인 출력**도 들어간다. D5 우회 중 삽입 끝에서는 영역 안 팔레트 셀이 비고 우회 검사기가 `inf` 를 내므로, 움직이는 차의 가능 운동을 정지 호만으로 보면 삽입 목표 너머로 넘어갈 수 있다(Codex 반례: 끝 11.26 mm 앞, `signed 0.0392`, `bound 0.060`, 출력 0.018 → 지금은 hold, 정지 호만이면 0.0208 전진 허용; 캐리지 여유 16 mm). 그래서 **서 있을 수 있는 읽기의 오차폭만** 정지 호로 판정하고, 나머지는 지금처럼 경로 끝 감속까지 포함한 층 답과 비교한다.

1. **층 API**: `DrivePermission.allowed_speed(…, path_end_target=True)`, `RealLayer.limit(…, path_end_target=True)` 키워드(기본 = 지금과 같음).
   - False 이면 `path.reached_end and path.blocked is None and path.first_unknown_m is None`(끝까지 검증, 공유 API 의 `unknown_passes` 대비 — Codex 2 회차)일 때 경로 끝 감속을 빼고 `arc_limit` 과 그 사유를 돌려준다. 그 밖(막힘·미검증·끝에 닿지 않음)은 지금과 같다.
   - 스냅숏 없음·센서 침묵·`evidence_stale`(경로·호 양쪽 증거 나이, `t + W`), `RealLayer` 의 설치 중·`scan_stale`·watchdog 은 지금처럼 그 앞에서 판정한다.
   - 분기는 `allowed_speed` 의 결과 선택에만 둔다. `@dumped`·`@dual` 이 붙은 `check_path`·`check_arc` 는 바꾸지 않는다.
2. **가능 운동을 둘로 나눈다**(실차만, `_possible_motion` 옆 새 함수):
   - **측정된 정지의 오차폭** `stand[d]`: 루프의 **측정된 정지**(`RealLoop._still` 의 `stand_since_ns` — 이전 출력이 모두 정리되고(`outstanding` settled) 0.2 s 창 동안 바퀴 카운트가 2 카운트 안, 오도메트리 연속)를 코어에 `state["measured_stand"]` 로 넘기고, 그것이 참이고 안전 속도가 유효한 두 방향 읽기일 때 각 방향에 `bound_mps`. 이때 남은 상한은 모형의 오차폭(가속도 × 나이)이지 운동이 아니다.
   - 두 방향 읽기만으로는 정지가 아니다(Codex 계획 2 회차 P1): `signed ± err` 가 0 을 걸치면 두 방향이 되지만, 카운트가 움직이면 상한에 측정 속도가 들어 있다(반례: 30 ms 에 292 카운트, `signed 0.0233`, `bound 0.0469`, 삽입 끝 8.1 mm 전, 명령·출력 0.018 — 정지 호만 보면 0.020 전진을 허용하고, 상한 속도의 정지 이동량 26.6 mm 가 남은 8.1 + 캐리지 여유 16 = 24.1 mm 를 넘음).
   - **그 밖의 가능 운동** `rest[d]`: 지금 명령(`_commanded_now`)의 방향에 그 크기, 미확인 출력(`outstanding_motion`)의 방향마다 그 값, 그리고 측정된 정지가 아닌 모든 읽기의 `bound_mps`. 각 방향의 최대.
   - `motion[d] = max(stand[d], rest[d])` 는 지금과 같다(기록·요청 속도 상한 계산은 그대로).
3. **가능 운동의 층 비교**(실차 D5 단계의 두 곳: `_direction_limits` 의 다른 방향 hold — 요청 0 의 양방향 포함 — 와 `_pocket_query` 의 면제 밖 재질의): 방향 `d` 에 대해
   - `rest[d] > 0` 이면 지금처럼 `layer_fn(d, rest[d])`(경로 끝 감속 포함)와 비교, 부족하면 hold.
   - `stand[d] > rest[d]` 이면 추가로 `layer_fn(d, stand[d], path_end_target=False)`(정지 호만)와 비교, 부족하면 hold.
   - 그래서 서 있는 차의 오차폭은 정지 호가 덮이면 hold 를 만들지 않고, 실제 명령·출력·움직이는 측정 속도는 지금처럼 경로 끝 감속에 묶인다.
   - D5 검사기(`check.limit`) 쪽 비교는 바꾸지 않는다(경로 끝 항이 없음).
4. **새 명령의 상한은 그대로**: 요청 속도로 묻는 층 답(`_direction_limits` 의 driving 질의)과 D5 답 중 작은 값. 경로 끝 감속이 상한이므로 차는 끝점 앞에서 감속해 허용오차 안에 선다. `path_left ≤ 0` 이면 상한 0 — 정지 호가 열려도 앞으로 가지 않는다.
5. **적용 범위**: 실차의 **D5 단계만** — `approach`·`insert` 이고 포켓 검사기(또는 우회 검사기)가 있을 때(`_direction_limits(…, stand_on_arc=True)`, `_pocket_query`). 다른 단계(운반·하역·cusp, `permission_watch`)는 지금 그대로다(Codex 계획 2 회차 P2: `transport` 하역 직선은 `acting=False` 라 층의 0 이 출력에 닿지 않을 수 있는데, 다른 방향 hold 가 풀리면 요청이 그대로 남는 반례). 그 단계의 같은 정체는 근거가 생기면 따로 다룬다.
   - 정체가 풀리는 모양은 **멈췄다 가기**다: 측정된 정지(0.2 s) → 정지 호 판정으로 hold 없음 → 경로 끝 상한 이하의 기어가기 명령 → 카운트가 움직이면 측정된 정지가 아니므로 오차폭이 다시 경로 끝 감속에 묶여 hold → 다시 정지. 통과 실행에서도 이 구간은 짧은 hold 와 진행이 번갈아 나왔다. 허용오차까지 남은 몇 mm 를 이렇게 간다.
6. **Isaac·시뮬레이션 불변**: `_direction_limits` 는 비실차면 지금처럼 한 번 부르고 끝난다. `_pocket_query` 는 실차만. 코어의 `layer_fn_` 클로저는 `path_end_target` 을 **받았을 때만** 넘긴다 — 시뮬레이션 호출의 인자·횟수·순서·기록된 kwargs 는 그대로다(Codex 2 회차: 무조건 넘기면 Isaac `limit` 시그니처·기록 kwargs 가 달라짐). 실차 조립을 쓰는 HIL·센서 재생은 `real_vehicle=True` 라 바뀐다(의도).
7. **바꾸지 않는 것**: 출력 잔여 hold(`residual.get(direction) > allowed`), D5 검사기, `end_creep_mps`·`axle_accel`·지연 상한 값.

**남는 위험(적어 둠)**: 측정된 정지에서도 모형상 오차폭(약 0.02 m/s)만큼의 운동은 정지 호로만 덮인다. 측정된 정지는 이전 출력이 모두 정리되고 0.2 s 동안 카운트가 2 카운트 안이어야 하므로 실제 운동은 그보다 훨씬 작다(2 카운트 / 0.2 s ≈ 0.024 mm/s 수준). 이번 수정이 바꾸는 것은 그때 보내는 명령이 0 인가, 경로 끝 상한(≤ 약 0.02 m/s, 끝을 지나면 0) 이하인가다. 그 명령으로 움직이기 시작하면 다음 틱부터는 지금 규칙이다. 삽입 끝 캐리지 여유(16 mm)와의 관계는 실차 S5 정지 측정 전까지 모형 수준의 판단이다.

**고르지 않은 안**:
- (나) `end_creep_mps` 를 서 있는 차의 최악 상한(`EncoderSpeedParams.worst_error()`) 이상으로 올리는 안. 경로 끝에서 실제 명령 속도까지 올라가고, 상한이 지연에 달려 있어 값이 맞을 보장이 없다.
- (다) "끝점 X mm 안에서 경로 끝 감속 때문에만 hold 면 도착으로 본다". 도착 판정과 안전 판정을 섞고, X 가 또 하나의 임시값이 된다.
- (라) 1 판의 "포켓 재질의 한 곳만 정지 호" 안. 후진 쪽 `direction_unsure` 가 남아 여전히 0 이고(Codex 재현), 움직이는 차의 hold 까지 풀린다(Codex 반례).

## 4. 시험

- **결정적 회귀**(Codex 2 회차 P2: 기본 리그는 가짜 층의 무한 허용치·`direction_evidence`·리그 스캔 주기 0.10 s 때문에 실패 조건을 가림): 한 틱을 실제 부품으로 잇는다 — 같은 카운트의 STATE 두 개(간격 20 ms) → 실제 `encoder_speed`(지연 27.8 ms) → 짧은 빈 경로·운영 정지 모형(지연 0.52 s, 감속 0.5, 여유 0.05)의 실제 `DrivePermission`/`RealLayer` → 실제 `MissionCore.permission`(D5 우회 검사기). 새 파일 `tests/unit/live/test_real_path_end_hold.py`.
  1. 끝까지 11.26 mm, 양쪽 호 빔 → `0 < requested_speed ≤ path_limit`, hold 없음. **수정 전 코드에서 0·hold 임을 먼저 확인**한다.
  2. 후진 호만 막힘(뒤에 장애물) → hold.
  3. 전진 호만 막힘 → hold.
  4. 요청 0 → 양방향 모두 정지 호로 물음(`path_end_target=False` 전달 단언), 출력 0, hold 없음. 한쪽 호만 막히면 `motion_hold`.
  5. 움직이는 읽기(Codex 반례: `signed 0.0392`, `bound 0.060`, 출력 0.018, 끝 11.26 mm) → 지금처럼 hold.
  5a. 움직이는 두 방향 읽기(Codex 2 회차 반례: 30 ms 에 292 카운트, 명령·출력 0.018, 끝 8.1 mm) → hold.
  5b. 측정된 정지가 아님(`measured_stand` 거짓)·같은 정지 읽기 → hold.
  5c. D5 단계 밖(`transport`) → hold(지금 그대로).
  6. 명령이 효력 중(`command_speed` 0.03 > 경로 끝 상한, 읽기는 서 있음) → hold(`rest` 가 경로 끝 감속에 묶임).
  7. `path_left ≤ 0`(`advance()` 로 끝 넘김), 호 빔 → 새 명령 0.
- **전이 시험**: `bypass_rig(real_layer=True, direction_evidence=False)` 에 운영 정지 모형을 맞추고 `_speed_delay_s()` 를 27.8 ms 로 고정해 접근 끝 11 mm 앞에서 세운다 → 수정 전 `pocket_stop`, 수정 후 허용오차 안 도착·`insert` 전이. 리그로 만들기 어려우면 그 이유를 적고 결정적 회귀가 대신한다(E2E 20 회로 대신하지 않는다).
- **층 단위**(`DrivePermission`): `reached_end`·막힘 없음·미검증 없음 → False 가 `arc_limit`; `blocked`·`first_unknown_m`·끝에 닿지 않음 → 같은 값; 기본 인자는 지금과 같다.
- **`RealLayer` 전달**(Codex 2 회차 P2): `RealLayer.limit` 이 키워드를 받았을 때만 `allowed_speed` 에 넘기고, `scan_stale`(증거 시각 `t + W`) 같은 층 자신의 거부가 그 앞에 그대로 있다.
- **가짜 층**: `test_real_loop_d5.Low.limit`, `test_core_truth_split._Layer.limit` 이 키워드를 받아 기록하게 고친다. 기존 "낮은 양수 응답은 hold" 시험이 그대로 hold 인지 확인(Codex 2 회차 P2, agy 6). 실차 경로에서 `layer.limit` 을 부를 수 있는 구현(`RealLayer`, 시험 대역)은 모두 같은 키워드를 받는다 — 받지 않는 구현으로 조용히 되돌아가는 분기는 두지 않는다.
- **머지 조건**(총괄): 결정적 회귀 1–7 통과(1 은 수정 전 실패 확인), 운영 파일 정상 경로 4 CPU 20 회 통과(`--keep-duplicates`, basetemp 는 짧은 `test_runs/u655b…`, 실패 기록 보존).
- **재생 게이트**: 미션 코어를 건드리므로 runs29 29/29(시뮬레이션 경로는 이 분기를 타지 않으므로 불변이 기대값).
- **회귀**: `test_real_run_hil*` 4 CPU, 전체 pytest 를 최종 트리에서 한 번.
- **원자료**: 노트북 사본(`u655diag/`)은 이 판정 뒤 삭제한다. ws1 `test_runs/u655x10` 은 머지 때까지 보존한다.

## 5. 자체 검토

| 검토 | 지적 | 처리 |
|---|---|---|
| Codex 진단(원자료 run0·run1 재계산) | hold 는 출력 잔여·현재 명령이 아니라 엔코더 상한 대 층의 경로 끝 허용치. `+0.006` 은 브레이크 최저속도가 아니라 한 틱 가속. U3e-1 diff 는 이 경로를 바꾸지 않았으나 타이밍 영향은 배제 못 함 | 2 절 |
| agy(적히지 않은 전제, Neon High) | ① 끝점 너머 구조물이 정지 호에 잡힌다는 전제 ② 정지한 차만 해당한다는 전제(움직이는 차·출력 잔여·끝 넘김) ③ `arc_limit` 이 늘 상한보다 크다는 전제 ④ 묶인 항이 경로 끝이라는 확정 ⑤ 지연 고착의 원인 ⑥ 가짜 층 시그니처 ⑦ 20 회 E2E 의 검출력(실패율 2/23 이 그대로여도 약 16 % 로 통과) ⑧ §7.1 불변식과의 관계 | ② → 3 절 2–3 항(서 있는 오차폭만), 4 절 5–7. ④ → 2 절(Codex 한 틱 재현). ⑤ → 2 절(위상, 기록의 38.3/27.8 ms 반복). ⑥ → 4 절 가짜 층. ⑦ → 결정적 회귀를 머지 조건으로. ⑧ → 3 절 원칙·6 항, 설계 §7.1 에 예외 문장 추가(구현 PR). ① → 3 절 남는 위험. ③ → 4 절 2–3(막히면 hold 가 정상) |
| Codex 계획 2 회차 | P1 두 방향 읽기가 움직이는 차도 포함(반례 292 카운트) → 측정된 정지가 필요. P2 모든 단계 적용은 `transport`(`acting=False`)에서 층의 0 이 출력에 닿지 않는 경로를 엶 → D5 단계로 좁힘. P2 요청 0 의 새 판정·`RealLayer` 전달을 구별하는 단언. P3 50 ms 와 100 ms 한도 구분. Isaac 불변성은 코드상 성립 확인 | 3 절 2·5 항, 남는 위험, 4 절 4·5a–5c·`RealLayer` 전달, 2 절 P3 |
| Codex 계획 1 회차(적대적) | P1 후진 검사가 같은 경로 끝 상한으로 이미 hold(`direction_unsure`) — 전진 재질의만으로는 0. P1 D5 우회 삽입 끝에서 움직이는 차의 hold 도 풀림(반례). P2 기본 리그가 실패 조건을 가림 — 실제 부품 한 틱 회귀. P2 가짜 층 시그니처. 그 밖: `first_unknown_m`, `path_left ≤ 0`, Isaac kwargs 불변, P3 가속 0.30·28 ms 출처 | 3 절 전면 수정(가능 운동 분리, 세 곳), 4 절 1–7·가짜 층, 2 절 P3 정정 |
