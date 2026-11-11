# S3 시운전 호스트 도구 ② — 축별 개루프 `COMMISSION` (설계 2판, 2026-11-12)

bring-up 묶음의 PR ② 다(총괄 순서 2026-11-11: 계약 PR #452 → 도구 ① #453 → 이 PR). 대상은 계획 `docs/plans/2026-11-11-sim-to-real-bringup.md` 5절 **S3 축별 개루프**다. 계약은 mcu-link/2 의 3.4 `COMMISSION`(`docs/interfaces/2026-10-22-mcu-link-contract.md`)이다. 도구 ①(`docs/design/2026-11-12-bringup-host-tools.md`)의 원칙(실행기가 아님, 소프트웨어 근거, 모든 `STATE` 보존, 시각 분리, 정리 절차)을 그대로 따른다. 2판은 Codex 설계 상의(P1 2·P2 5·P3 1)를 반영했다(9절).

플래시는 USB-C DFU 로만 한다(사용자 결정, SWD 디버거 없음). 이 도구와 절차는 SWD 를 전제하지 않는다. 시운전 빌드와 정상 빌드를 바꿔 올리는 일은 펌웨어 쪽 절차(DFU)이고, 도구는 올라간 빌드를 `STATE.flags` 로만 판단한다.

## 1. S3 가 요구하는 것과 도구가 내는 것

| S3 합격 조건(계획 5절) | 도구가 내는 근거 | 외부에서 확인할 것 |
|---|---|---|
| 방향·극성(`duty_sign`) 확정 | 양의 명령에 대해 MCU 가 적용한 듀티의 부호(`STATE.duty_e4` 는 `duty_sign` 을 곱한 값), 측정 회전의 부호(바퀴: 카운트 차분, 조향·승강: KTH 값 차분), 운용자가 본 물리 방향 | 운용자의 눈(바퀴가 차체 앞으로 굴렀나, 조향이 왼쪽으로 꺾였나, 포크가 올라갔나) |
| 전류 측정 정상 | 고른 축 채널의 `current_ma` 최대·평균, 고르지 않은 채널의 최대 | 벤치 전원 전류계와 대조 |
| 조향·승강 범위 끝을 기구 한계 전에 측정 | 고른 축 값의 시작·끝·최소·최대, 운용자 창(2절)에서 멈춘 기록 | 기구 한계까지의 여유(눈·자) |
| 무부하 최대 바퀴 속도 | 이 듀티에서의 정상 회전 속도만. **최대 속도는 내지 않는다**: 시운전 듀티 상한(제안 15 %) 안에서는 전 듀티 속도를 잴 수 없고, 외삽은 측정이 아니다 | 남는 항목으로 기록(4절) |
| 전원 강하·회생 과전압 | 없음(호스트가 볼 수 없다). `record --markers` 로 기동·역전 시각을 표시해 오실로스코프 기록과 맞출 수 있다 | 오실로스코프 |

## 2. 명령

```
bringup.py commission --device D --out run.jsonl --axis L|R|M|H --duty X --seconds S
                      [--window LO HI] [--observed pos|neg|none] --power-switch-in-hand [--allow-locked] [--yes]
```

- `--duty`: −0.15…0.15, 0 은 거부. 도구 상한 `COMMISSION_DUTY_MAX = 0.15` 는 계획의 제안 빌드 상한과 같다. 빌드 상수가 더 낮으면 MCU 가 `bad_value` 로 거부하고, 도구는 그 거부를 결과에 남기고 정리한다(자르지 않는다, 계약 3.4).
- `--seconds`: 바퀴는 0.1–1.5 s(`COMMISSION_HOLD_MIN_S`·`COMMISSION_HOLD_MAX_S`). 상한은 빌드의 연속 듀티 시간 상한(제안 2 s)보다 짧게 두어, 도구가 정상일 때 MCU 상한에 닿지 않게 한다. MCU 상한은 도구와 무관한 최후 보호다. 하한은 비영 구간이 `HANDOVER_S`(50 ms)보다 길어 비영 명령이 실제로 나가게 하는 값이다.
- **조향·승강은 한 번에 0.1–0.2 s**(`AXIS_HOLD_MAX_S`). 비영 명령은 그 구간 끝을 넘어 유효하지 않으므로(도구 ① 3절), 창을 못 보거나 링크가 끊겨도 한 번의 구동 시간은 이 값을 넘지 않는다. 범위 끝은 창 안에서 시작하는 짧은 구동을 되풀이해 찾는다.
- `--window LO HI`: 조향(rad)·승강(m)은 **필수**, 바퀴는 받지 않는다. 시작 값이 창 안이어야 시작한다. 구동 중 측정값이 창을 벗어나면 즉시 0 듀티(같은 축)로 바꾸고 `window` 로 기록한다.
  - 이 정지는 호스트 쪽이라 늦다. 최악은 창 이탈 뒤 `STATE` 한 주기 + 호스트 루프(10 ms) 동안 감지가 늦고, 0 명령이 가지 못하면 마지막 비영 명령의 유효시간까지 구동이 이어지는 경우다. 그래서 시운전 호스트는 명령 유효시간 상한을 **60 ms**(`COMMISSION_CAP_MS`, 도구 ① 은 150 ms)로 줄인다.
  - 창과 기구 한계 사이 여유 ≥ (축 속도 × (`STATE` 주기 + 0.01 s + 0.06 s)) + 정지 뒤 관성 이동 + 측정 오차. 축 속도는 앞선 짧은 구동의 결과(4절 `value`)로 잰다. 그 근거가 없는 첫 구동은 범위 한가운데에서 가장 낮은 듀티·0.1 s 로 시작한다.
  - MCU 의 범위 밖 결함(`steer_out_of_range`·`lift_out_of_range`)은 S3 에서 센서 변환이 아직 검증되지 않았으므로 독립된 물리 보호로 치지 않는다. 물리 보호는 짧은 구동·창 여유·주 전원을 쥔 사람이다.
- `--observed`: 운용자가 **실제로 본** 방향(바퀴: 차체 앞으로 굴렀으면 `pos`, 조향: 왼쪽 `pos`, 승강: 위 `pos`, 안 움직였으면 `none`). 명령 부호와 무관하게 본 방향이다. 주지 않으면 실행 뒤 물어본다(`--yes` 면 묻지 않고 `unknown`).
- `--power-switch-in-hand`: 필수. 시운전 빌드는 비상정지 핀이 없어도 `ARM` 을 받는다. 그 조건이 '바퀴를 띄운 받침대, 사람이 주 전원 차단을 손에 쥔 상태'다(펌웨어 설계 `docs/design/2026-11-11-mcu-link-v2-firmware-core.md` 2.1, 총괄 결정 Q3). `--yes` 는 이 확인을 대신하지 않는다. 결과에 기록한다.
- 결과는 설정에 바로 쓰지 않는다. `duty_sign`·엔코더 부호 제안과 근거만 출력한다(도구 ① 의 `cpr` 과 같은 방식).

## 3. 안전 조건

- **빌드**: `commission_build`(bit4)가 아니면 아무것도 보내지 않는다(계약 3.3 마지막 줄, `build_policy.commission_allowed`). `outputs_locked`(bit6)면 실기에서는 출력이 없으므로 중단한다. 모형 시험만 `--allow-locked`. 계획 S3 선행 조건(사용자 결정: IPROPI 보호 확인 뒤 `FW_MOTOR_OUTPUT` 을 켠 빌드)은 운용자 확인 문구에 넣는다. `estop_unwired` 는 시운전 빌드에서 `ARM` 을 막지 않으므로(계약 3.3 거부 이유 15) 결과에 표시만 한다.
- **받침대 전용**: 바닥 시험 경로(`--floor`)는 없다. 운용자 확인 문구는 "바퀴를 띄웠는가, 조향·승강은 창이 기구 한계 안쪽인가"다.
- **무장**: 도구 ① 과 같다(운용자 요청 → `ready` 까지, 자동 재시도 없음). 다만 무장 직후의 0 명령은 `DRIVE` 가 아니라 **0 듀티 `COMMISSION`** 이다(시운전 빌드는 `DRIVE` 를 `wrong_build` 로 거부한다). **이 무장용 0 명령만** L(axis 0)로 보낸다. 조향·승강 측정이 무효여도 무장까지는 가게 하고, 그 축의 명령이 `sensor_invalid` 로 거부되는 것을 기록하기 위해서다.
- **순서**: 고른 축 0 듀티 0.3 s → 비영 `--seconds`(창·엔코더 비트·중단이면 일찍 0 으로) → 고른 축 0 듀티 0.5 s → 정리. 무장 뒤의 0 명령은 모두 **고른 축**으로 보낸다(Codex P2-3). 다른 축으로 0 을 보내면 고른 축이 그 채널의 정지 상태(`safe_brake=false` 면 코스트)로 바뀌고, 조향·승강은 센서 결함 검사 대상에서도 빠진다. 그래서 뒤쪽 0.5 s 는 고른 축이 0 듀티로 전기 제동되는 동안의 관찰이다. 비영 구간의 만료 상한·`HANDOVER_S`·고정 종료 시각 규칙은 도구 ① 3절과 같다.
- **일찍 0 으로 바꾸는 경우**(모두 기록):
  - 비영 구동 직전에 무장 뒤 받은 표본 전체를 먼저 본다. 그 뒤로도 매 주기 새 표본 전체를 본다(한 번에 받은 표본 묶음 안의 창 이탈·복귀를 놓치지 않는다, 재검토 P2-4).
- 고른 축의 엔코더 비트 18–21(`encoder_stuck_*`·`encoder_polarity_*`)이 새로 켜짐. 시운전 빌드에서는 이 비트가 래치만 되고 `ARMED` 가 이어진다(계약 3.4). 도구는 비트를 표시하고, 그 축의 구동을 거기서 멈춘다 — 극성 판단은 이미 끝났고 더 돌릴 이유가 없다.
  - 창 이탈(조향·승강).
- **세션 전체 검사**: 무장부터 정리까지의 표본 전체에서 `FAULT`·`ESTOP` 모드나 부팅 변경이 하나라도 있으면 중단이다. 정리 중 처음 보인 결함도 포함한다. 동역학 계산용으로 `DISARM` 확인 표본에서 자르는 것과는 따로다(재검토 P2-2).
- **중단**(도구 ① 과 같은 분류): 권한 철회(모드가 `ARMED` 를 벗어남 — 과전류·nFAULT·`commission_time_limit`·비상정지), 부팅 변경, `STATE` 공백(0.2 s), 비영 명령 거부(`reject_reason`, 예: `bad_value`·`sensor_invalid`).
- **정리**: 정상·예외·Ctrl-C 모두 도구 ① 의 `execute` 경로(무장 요청이 나갔으면 마지막 `STATE` 와 무관하게 `DISARM` 확인). 단 **`STOPPING` 중에는 `DISARM` 을 미룬다**(Codex P1-1). 시운전 빌드의 `STOPPING` 은 네 축 전기 제동이다. 펌웨어 코어(`drive.c` `drive_handle` 의 `LINK_DISARM`)는 `STOPPING` 에서도 `DISARM` 을 받으면 바로 `DISARMED` 로 가서 제동을 끝낸다(`safe_brake=false` 채널은 코스트). 이것은 정지 명령에 예외를 두지 않는다는 코어의 선택이고, 계약 3.4 문구를 그에 맞춰 고쳤다(9절). 줄일 수 없는 것은 재무장 금지다. 도구는 제동 상태를 끝까지 보기 위해 대기 명령을 버리고 `STOPPING` 이 스스로 끝나기를 기다린 뒤(최대 1 s, `STOPPING_WAIT_S`) `DISARM` 확인을 한다.
  - 마지막 `STATE` 가 오래됐으면(호스트 신선도 기준 50 ms 를 넘김) 보이지 않는 `STOPPING` 일 수 있으므로 같은 방식으로 기다린다. 기다림은 '대기 시작 뒤 새로 받은 `STATE` 가 `STOPPING` 이 아님'으로 끝난다.
  - 1 s 안에 그것을 보지 못하거나 대기 중 I/O 가 실패해도 `DISARM` 은 보낸다(정지 명령은 미루기만 하고 거두지 않는다). 대신 그 실행은 `stopping_end_unobserved` 로 중단 처리한다(Codex 재검토 P2-1). 정리 자체가 예외로 끝나도 결과 행은 `cleanup_confirmed=false` 와 `cleanup_error` 로 남는다(재검토 P2-5).

## 4. 결과(소프트웨어 근거)

공통:
- `accepted`: 첫 비영 `COMMISSION` 을 MCU 가 받았는가(`last_rx_seq`), 거부면 `reject_reason`.
- 구동 구간의 끝은 뒤따른 0 명령을 받아들인 첫 표본, 또는 MCU 가 스스로 `ARMED` 를 벗어난 첫 표본(시간 상한·만료·결함)이다. 끝을 찾지 못하면 결과는 `end_unknown` 으로 무효다. 기록 끝까지를 구동으로 치지 않는다(재검토 P2-3).
- `driven_ms_min`·`driven_ms_max`: 구동 시간의 표본 경계(MCU 시간). `last_rx_seq` 는 마지막으로 받아들인 명령 하나만 알리므로(Codex P2-7), 첫 비영 명령의 수락 순간은 표본 사이에 있다. 수락은 도구가 보낸 비영 seq 전체의 집합으로 찾는다. min 은 비영 수락 첫 표본부터 0 수락 직전 표본까지, max 는 각각의 직전 표본부터다.
- `applied`: 고른 축 `duty_e4` 의 부호·최대 크기. 명령 부호와 비교해 **빌드의 `duty_sign`** 을 추정한다(`+1`·`-1`, 출력이 0 이면 `null`).
- `current`: 고른 채널 `current_ma` 의 최대·비영 구간 평균, 다른 세 채널의 최대.
- `measured_sign`: 측정 움직임의 부호(바퀴: 같은 부팅 안 모듈러 카운트 차분, 조향·승강: 값 차분). 움직임이 잡음 문턱(바퀴 4 카운트, 조향 0.005 rad, 승강 0.5 mm)보다 작으면 `none`.
- `encoder_bits`: 18–21 가운데 실행 중 켜진 비트의 이름과 처음 본 MCU 시간. 시작 때 이미 켜진 비트는 따로(`at_start`).
- `stop`: `time`·`window`·`encoder_bit`·중단 이유.
- `observed`: 운용자 입력.

판정 제안(운용자 관측이 있고 움직임이 있을 때만, Codex P2-4):
- `duty_sign_ok` = 명령 부호와 본 방향이 같은가. 아니면 "이 채널의 `duty_sign` 을 뒤집는다"(`suggested_duty_sign` = 빌드 값 × −1).
- `sensor_sign_ok` = 측정 부호와 본 방향이 같은가. 아니면 "이 축 센서의 부호(`wheel[i].sign` 또는 `steer/lift.sign`)를 뒤집는다".
- 본 방향과 측정이 모두 `none` 이면 `insufficient_motion`(정지 마찰 이하·걸린 모터일 수 있다), 한쪽만 `none` 이면 `inconsistent`, 관측이 없으면 `no_observation`. 모두 제안하지 않는다.

바퀴만:
- `rate_rad_s`: 비영 구간 뒤쪽 절반의 평균·최대(정상 상태 근사). 이 듀티의 무부하 속도이지 최대 속도가 아니다.

조향·승강만:
- `value`: 시작·끝·최소·최대, 창, 창 이탈 시각.

## 5. 호스트 코어 변경 (`McuLinkHost`)

- 생성자에 `commission: bool = False` 를 더한다. `client` 가 `BRINGUP` 일 때만 허용(그 밖은 `ValueError`).
- `commission=True` 의 `build_refusal`: `commission_build` 가 아니면 `not_commission_build`, `outputs_locked` 면 `allow_locked` 일 때만 허용. 기존 `BRINGUP` 의 "시운전 빌드에 `DRIVE` 금지" 는 `commission=False` 에서 그대로다. 그래서 같은 호스트가 두 빌드에 두 종류 명령을 섞어 보낼 수 없다(양방향).
- 무장 뒤 0 명령: `commission=True` 면 0 듀티 `COMMISSION`(axis 0, 유효시간 `zero_valid_ms`).
- 새 `commission(axis, duty_e4, expires_ns, now_ns) -> Blocked | None`: `drive()` 와 같은 검사(신선도·`ready`·빌드·만료·유효시간 상한·outbox)를 거쳐 `Commission` 을 넣는다. 유효시간 상한은 호스트 설정의 `cap_ms` 이고, 시운전 도구는 60 ms 설정을 쓴다. `commission=False` 면 `Blocked("not_commission_host")`. 거꾸로 `commission=True` 의 `drive()` 는 `Blocked("commission_host")`.
- outbox 규칙은 `COMMISSION` 을 `DRIVE` 와 같이 다룬다: 만료·권한 세대가 지난 것은 쓰지 않고 버림, `drop_drives` 가 함께 버림, `DISARM` 요청이 버림, 쓴 기록(`_written`)에 남김.
- 실행기 경로(`commission=False`, `client` REAL·HIL)는 바뀌지 않는다. 기존 시험이 그대로 통과해야 한다.

## 6. MCU 모형 (`firmware/tools/mcu_model.c`)

시험용 선택지를 더한다. 모두 호스트 시험용 합성값이고 하드웨어 값이 아니다.
- `--commission-build`: `commission_build = true`, `commission_duty_max = 0.15`, `commission_time_ms = 2000`(계획 제안값).
- `--commission-time-ms N`: 시간 상한을 줄여 MCU 상한 경로(`commission_time_limit` → `STOPPING`)를 시험한다.
- `--outputs-locked`: `outputs_locked = true`(모형은 출력을 그대로 plant 에 넣으므로 `--allow-locked` 경로 시험용 표시일 뿐이다).
- `--encoder-sign-flip 0|1`: 그 바퀴의 카운트 방향을 뒤집어 넣는다(센서 부호 제안 시험).
- `--encoder-test-thresholds 1`: stuck·polarity 판정을 듀티 0.05·100–300 ms 로 낮춘다. 모형의 기본 문턱(듀티 0.95, 지속 2 s)으로는 시운전 듀티(≤ 0.15)·구동 시간(≤ 1.5 s)에서 비트가 켜지지 않아 조기 정지 경로를 시험할 수 없기 때문이다(Codex P2-6).

## 7. 시험

- 단위(`tests/unit/plant/test_mcu_host_commission.py`): `commission=True` 의 빌드 거부 양방향, 0 `COMMISSION` 무장 흐름, `drive()`·`commission()` 상호 거부, 만료·세대·`drop_drives`·`DISARM` 의 outbox 규칙, `client` 제한.
- 계산(`tests/unit/plant/test_bringup_commission_calc.py`): 결과 계산(부호·잡음 문턱·뒤쪽 절반 평균·`inconsistent`·판정 제안)을 합성 `STATE` 로.
- 도구 단위(`tests/unit/tools/test_bringup_safety.py`): 조기 정지 규칙(구동 바퀴의 새 비트만, 다른 바퀴의 비트·시작 때 켜진 비트는 아님, 창 이탈·무효 값), 시운전 세션의 빌드 거부와 `DRIVE` 거부.
- 종단(`tests/integration/test_bringup_commission_tool.py`, MCU 모형): 정상 빌드 거부(아무것도 안 보냄), 바퀴 L 정상·부호, 엔코더 부호 뒤집기 → 센서 부호 제안, 조향 창 정지(양방향), 조향 구동 중 TX 단절 → 60 ms 상한 안 0 듀티, 승강 정상, 시간 상한 단축 → 중단·`STOPPING` 이 스스로 끝난 뒤에만 `DISARM`, 낮춘 문턱의 polarity 비트 → `ARMED` 유지 중 0 듀티, 도구 상한(듀티·시간·축별 시간·창·주 전원 확인) 거부, 출력 잠금, 무장 중 예외 → `DISARM` 확인.
- 호스트가 볼 수 없는 것: `STOPPING` 중 네 채널 DRVOFF 가 실제로 low 로 유지되는지(`STATE` 에 없다). 펌웨어 코어 시험의 몫이다.
- 정상 빌드 쪽 도구 ① 의 종단 10 건은 그대로 통과해야 한다.

## 8. 범위 밖

- 최대 바퀴 속도 측정(시운전 듀티 상한 밖 — 빌드 상수를 바꾸는 결정이 먼저다).
- 전원 강하·회생 과전압(오실로스코프).
- 설정 파일 자동 수정.
- 승강 위치 측정 방식(계획 3.5, 미정). 승강 시운전은 빈 포크·한 바퀴 미만 창에서만 한다.

## 10. 시간 상한 `STOPPING` 중 `DISARM` 시나리오 (총괄 결정 2026-11-12)

펌웨어 쪽이 S5 에 넣은 항목('최대 시운전 출력에서 상한 직후 `DISARM`·보류 중 `DISARM` 의 정지 시간·거리')을 도구로 연다(총괄 결정). 계약 3.2·3.4(2026-11-12 정정)와 펌웨어 회귀 시험 v2r6(`origin/feat/fw-weact-h743` 의 `firmware/core/test/test_drive.c` `limit_then_disarm`)가 정한 동작을 실기에서 기록하는 시나리오다.

```
bringup.py commission --device D --out run.jsonl --axis L|R --duty X --scenario limit_disarm|hold_disarm
                      [--limit-s 2.0] [--disarm-delay-ms 50] --power-switch-in-hand
                      [--output-response-ms X|unavailable] [--stop-time-ms X|unavailable] [--stop-distance-mm X|unavailable]
```

- **바퀴만**(L·R). 조향·승강은 상한까지 돌리면 기구 한계에 닿는다.
- 이 시나리오만 2절의 1.5 s 상한을 넘겨 **빌드의 시간 상한까지** 비영 듀티를 이어 보낸다. `--limit-s` 는 빌드 상한(제안 2 s)을 운용자가 적는 값이고 (1.5, 3.0] s 만 받는다. `--limit-s` + 0.3 s 안에 MCU 의 상한(`STOPPING` + `commission_time_limit`)이 보이지 않으면 `limit_not_reached` 로 중단한다. 비영 명령의 유효시간은 그대로 60 ms 이하라, 도구가 멈추면 MCU 만료가 구동을 끝낸다.
- 순서: 무장 → 고른 바퀴 0 듀티 0.3 s → 비영 듀티(MCU 상한까지, 매 주기 새 표본 전체에서 상한을 찾음) → 사건:
  - `limit_disarm`: 상한 표본을 본 즉시 `DISARM`.
  - `hold_disarm`: 상한 뒤 `--disarm-delay-ms`(기본 50 ms, `stop_grace_ms` 보다 짧아야 함) 기다렸다 `DISARM`. 그 사이 `STOPPING` 이 끝났으면 시나리오의 뜻이 없으므로 `not_met`(`hold_ended_before_disarm`).
  - 그 뒤 0.6 s 카운트 관찰 → 정리(도구 ① 과 같은 `DISARM` 확인).
- 기록(소프트웨어 근거): 명령 순서(쓰기 완료 시각), 상한까지의 MCU 시간(`limit_after_ms`), 사건 `DISARM` 의 seq·쓰기 시각, 그 seq 를 담은 첫 `STATE`(수락)와 그 직전 표본의 모드(`disarm_during_stopping`), 수락 표본의 모드, 상한 표본에서 `DISARM` 쓰기까지(`limit_seen_to_disarm_write_ms`), `DISARMED` 까지·카운트 정착까지의 시각. 수락 표본에서 비트 24 가 남아 있는지는 시험이 본다.
- 결과: `DISARM` 이 수락되고(`hold_disarm` 은 `STOPPING` 에서 수락돼야) 세션 전체 검사(3절)를 통과하면 `ok`, 아니면 `not_met`·`aborted`.
- **합격 판정은 외부 측정값으로만** 한다(다른 S5 항목과 같다, 도구 ① 4절). 문턱은 일반 `DISARM` 의 것을 잠정 적용한다(총괄 결정 2026-11-12, 계획 3.4): 도구의 문턱 표에서 두 시나리오는 `disarm` 행을 따른다(`THRESHOLD_KIND`). 그 행은 출력 반응 0.1 s, 바퀴 정지 0.5 s·0.10 m(잠정, 총괄 결정 2026-11-12)다. 시운전 시나리오는 받침대(바퀴 띄움)에서만 하므로 정지 시간·거리는 문턱 대비 **참고(받침대)**로만 보인다(`reference: stand`, `over_limit`, `pass: null`). 합격은 바닥 S5 외부 측정의 몫이고, 받침대에서 넘으면 문턱을 늦추지 않고 원인(코스트·`safe_brake`)부터 보고한다. 출력 반응은 받침대에서도 판정한다. `analyze --kind limit_disarm|hold_disarm` 도 같다(`--floor` 를 줄 때만 정지 시간·거리를 판정한다). 이 받침대 규칙은 도구 ① 의 `scenario`(`--floor` 없이)에도 똑같이 적용된다.
- 재무장 보류의 확인(보류 중 `ARM` 거부)은 실기 도구에 넣지 않는다(총괄 결정 2026-11-12). 펌웨어 회귀 시험 v2r6 가 고정하고 있고, 실기 확인은 펌웨어 HIL 빌드 시험(펌웨어 PR ⑤)으로 넘긴다.
- 시험(MCU 모형, 상한 300 ms·`stop_grace_ms` 100 ms): 두 시나리오 모두 `STOPPING` 에서 수락·`DISARMED`·비트 24 유지, 명령 순서(`ARM` → `COMMISSION`… → `DISARM`, `DRIVE` 없음), 외부 값 기록. 지연이 `stop_grace_ms` 보다 길면 `not_met`. 상한이 오지 않으면 `limit_not_reached`. 인자 거부(축·`--limit-s`·지연·주 전원·듀티). CLI 파싱.

## 9. 자체 검토

Codex 설계 상의 1회(P1 2·P2 5·P3 1). 모두 반영했다.

| 지적 | 처리 |
|---|---|
| P1-1 정리용 `DISARM` 이 시간 상한 `STOPPING` 의 네 축 제동을 끊음(`drive.c` `LINK_DISARM`, 계약 3.4 와 불일치) | 근거 확인(`drive_handle` 이 `driving(d)` 면 바로 `DISARMED`). 도구는 `STOPPING` 이 끝나기 전에 `DISARM` 을 보내지 않는다(3절). 펌웨어-H743 세션의 판단(정지 명령에 예외를 두면 그쪽이 더 위험, 상한의 목적인 우회는 재무장 금지가 이미 막음)에 따라 코어는 그대로 두고 계약 3.4 문구를 고쳤다 |
| P1-2 창 여유 0.1 s 가 명령 유효시간 150 ms 보다 짧고, S3 의 MCU 범위 검사는 독립 보호가 아님 | 시운전 유효시간 상한 60 ms, 조향·승강 한 번 0.2 s 이하, 여유 식을 명시, 첫 구동 조건, MCU 범위 검사를 물리 보호에서 뺌(2절). TX 단절·양방향 창 시험 추가 |
| P2-3 axis 0 의 0 명령은 전체 축의 중립이 아님 | 무장용 0 명령만 axis 0, 그 뒤 0 은 모두 고른 축(3절) |
| P2-4 부호 판정의 음의 명령·무동작 반례 | 관측을 '본 방향'으로 정의, 무동작 둘 다 `insufficient_motion`, 한쪽 `inconsistent`(4절) |
| P2-5 비상정지 미배선 허용의 운용 전제 | `--power-switch-in-hand` 필수, `--yes` 로 대신 못 함, 결과에 기록(2절) |
| P2-6 모형 문턱으로는 엔코더 비트 조기 정지를 시험 못 함 | 모형 `--encoder-test-thresholds`, 종단·단위 시험(6·7절) |
| P2-7 `last_rx_seq` 로 첫 수락·구동 시간 확정 불가 | 비영 seq 집합으로 수락 찾기, `driven_ms_min`·`max` 로 경계 제시(4절) |
| P3-8 짧은 `--seconds` 는 비영 송신 없음 | 하한 0.1 s(2절) |

Codex 재검토(설계 2판 + 구현, 새 P1 없음, P2 5·P3 1). 모두 반영했다.

| 지적 | 처리 |
|---|---|
| P2-1 `STOPPING` 대기가 수신 단절·시간 초과에서 보장되지 않음 | 오래된 `STATE` 도 대기 대상, 대기 뒤 새 표본으로만 끝, 못 보면 `DISARM` 은 보내되 `stopping_end_unobserved` 중단(3절) |
| P2-2 정리 중 처음 본 `FAULT` 가 성공 | 세션 전체 검사(`session_abort_reason`)를 동역학 절단과 분리(3절) |
| P2-3 MCU 자체 정지에서 끝 표본을 못 찾음 | 끝 = 0 수락 또는 `ARMED` 이탈, 못 찾으면 `end_unknown` 무효(4절) |
| P2-4 비영 직전 이미 창 밖이어도 첫 비영이 나감, 표본 묶음의 이탈·복귀 누락 | 비영 송신 전 검사, 새 표본 전체 검사(3절) |
| P2-5 대기 중 예외가 정리·결과 기록을 탈출 | 대기 예외를 잡아 `DISARM` 계속, `execute` 의 마지막 정리 예외도 잡아 결과 행 기록 |
| P3-6 설계에 이전 계약 문장 | 고침 |

Codex 재검토 3회차(P1 없음, P2 1·P3 1). 반영했다.

| 지적 | 처리 |
|---|---|
| P2 정리의 신선도 기준(0.2 s)이 호스트(50 ms)와 달라 50–200 ms 공백에서 보이지 않는 `STOPPING` 을 기다리지 않음 | 호스트 설정의 `state_max_age_s` 로 맞춤, 60 ms 공백 시험 |
| P3 끝 표본이 `DISARM` 확인 표본이면 절단으로 경계가 사라짐 | 끝 표본은 절단해도 남김, 계산은 끝이 입력 밖이면 `end_unknown` |
