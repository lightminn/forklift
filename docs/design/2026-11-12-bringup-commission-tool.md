# S3 시운전 호스트 도구 ② — 축별 개루프 `COMMISSION` (설계 1판, 2026-11-12)

bring-up 묶음의 PR ② 다(총괄 순서 2026-11-11: 계약 PR #452 → 도구 ① #453 → 이 PR). 대상은 계획 `docs/plans/2026-11-11-sim-to-real-bringup.md` 5절 **S3 축별 개루프**다. 계약은 mcu-link/2 의 3.4 `COMMISSION`(`docs/interfaces/2026-10-22-mcu-link-contract.md`)이다. 도구 ①(`docs/design/2026-11-12-bringup-host-tools.md`)의 원칙(실행기가 아님, 소프트웨어 근거, 모든 `STATE` 보존, 시각 분리, 정리 절차)을 그대로 따른다.

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
                      [--window LO HI] [--observed pos|neg|none] [--allow-locked] [--yes]
```

- `--duty`: −0.15…0.15, 0 은 거부. 도구 상한 `COMMISSION_DUTY_MAX = 0.15` 는 계획의 제안 빌드 상한과 같다. 빌드 상수가 더 낮으면 MCU 가 `bad_value` 로 거부하고, 도구는 그 거부를 결과에 남기고 정리한다(자르지 않는다, 계약 3.4).
- `--seconds`: 0 초과 1.5 s 이하(`COMMISSION_HOLD_MAX_S`). 빌드의 연속 듀티 시간 상한(제안 2 s)보다 짧게 두어, 도구가 정상일 때 MCU 상한에 닿지 않게 한다. MCU 상한은 도구와 무관한 최후 보호다.
- `--window LO HI`: 조향(rad)·승강(m)은 **필수**, 바퀴는 받지 않는다. 시작 값이 창 안이어야 시작한다. 구동 중 측정값이 창을 벗어나면 즉시 0 듀티로 바꾸고 `window_stop` 으로 기록한다. 이 정지는 호스트 쪽이라 늦다(`STATE` 주기 + 링크 + 0 명령 왕복). 그래서 창은 기구 한계보다 (예상 속도 × 0.1 s) 이상 안쪽으로 잡고, 첫 시험은 짧게(0.2 s) 한다. MCU 의 범위 밖 결함(`steer_out_of_range`·`lift_out_of_range`, 빌드의 `min_joint`·`max_joint`)은 그 뒤의 보호다.
- `--observed`: 양의 명령일 때 운용자가 본 물리 방향(바퀴: 차체 앞, 조향: 왼쪽, 승강: 위 = `pos`). 주지 않으면 실행 뒤 물어본다(`--yes` 면 묻지 않고 `unknown`).
- 결과는 설정에 바로 쓰지 않는다. `duty_sign`·엔코더 부호 제안과 근거만 출력한다(도구 ① 의 `cpr` 과 같은 방식).

## 3. 안전 조건

- **빌드**: `commission_build`(bit4)가 아니면 아무것도 보내지 않는다(계약 3.3 마지막 줄, `build_policy.commission_allowed`). `outputs_locked`(bit6)면 실기에서는 출력이 없으므로 중단한다. 모형 시험만 `--allow-locked`. 계획 S3 선행 조건(사용자 결정: IPROPI 보호 확인 뒤 `FW_MOTOR_OUTPUT` 을 켠 빌드)은 운용자 확인 문구에 넣는다. `estop_unwired` 는 시운전 빌드에서 `ARM` 을 막지 않으므로(계약 3.3 거부 이유 15) 결과에 표시만 한다.
- **받침대 전용**: 바닥 시험 경로(`--floor`)는 없다. 운용자 확인 문구는 "바퀴를 띄웠는가, 조향·승강은 창이 기구 한계 안쪽인가"다.
- **무장**: 도구 ① 과 같다(운용자 요청 → `ready` 까지, 자동 재시도 없음). 다만 무장 직후의 0 명령은 `DRIVE` 가 아니라 **0 듀티 `COMMISSION`** 이다(시운전 빌드는 `DRIVE` 를 `wrong_build` 로 거부한다). 축은 고른 축이 아닌 L(0)로 보낸다. 조향·승강 측정이 무효여도 무장까지는 가게 하고, 그 축의 비영 명령이 `sensor_invalid` 로 거부되는 것을 기록하기 위해서다.
- **순서**: 0 듀티 0.3 s → 비영 `--seconds`(창·엔코더 비트·중단이면 일찍 0 으로) → 0 듀티 0.5 s(관성 관찰) → 정리(`DISARM`, 도구 ① 과 같이 그 `DISARM` 의 seq 를 담은 `STATE` 로 확인). 비영 구간의 만료 상한·`HANDOVER_S`·고정 종료 시각 규칙은 도구 ① 3절과 같다.
- **일찍 0 으로 바꾸는 경우**(모두 기록):
  - 고른 축의 엔코더 비트 18–21(`encoder_stuck_*`·`encoder_polarity_*`)이 새로 켜짐. 시운전 빌드에서는 이 비트가 래치만 되고 `ARMED` 가 이어진다(계약 3.4). 도구는 비트를 표시하고, 그 축의 구동을 거기서 멈춘다 — 극성 판단은 이미 끝났고 더 돌릴 이유가 없다.
  - 창 이탈(조향·승강).
- **중단**(도구 ① 과 같은 분류): 권한 철회(모드가 `ARMED` 를 벗어남 — 과전류·nFAULT·`commission_time_limit`·비상정지), 부팅 변경, `STATE` 공백(0.2 s), 비영 명령 거부(`reject_reason`, 예: `bad_value`·`sensor_invalid`).
- **정리**: 정상·예외·Ctrl-C 모두 도구 ① 의 `execute` 경로(무장 요청이 나갔으면 마지막 `STATE` 와 무관하게 `DISARM` 확인).

## 4. 결과(소프트웨어 근거)

공통:
- `accepted`: 첫 비영 `COMMISSION` 을 MCU 가 받았는가(`last_rx_seq`), 거부면 `reject_reason`.
- `driven_ms`: 첫 비영 수락 표본부터 0 수락 표본까지의 MCU 시간.
- `applied`: 고른 축 `duty_e4` 의 부호·최대 크기. 명령 부호와 비교해 **빌드의 `duty_sign`** 을 추정한다(`+1`·`-1`, 출력이 0 이면 `null`).
- `current`: 고른 채널 `current_ma` 의 최대·비영 구간 평균, 다른 세 채널의 최대.
- `measured_sign`: 측정 움직임의 부호(바퀴: 같은 부팅 안 모듈러 카운트 차분, 조향·승강: 값 차분). 움직임이 잡음 문턱(바퀴 4 카운트, 조향 0.005 rad, 승강 0.5 mm)보다 작으면 `none`.
- `encoder_bits`: 18–21 가운데 실행 중 켜진 비트의 이름과 처음 본 MCU 시간. 시작 때 이미 켜진 비트는 따로(`at_start`).
- `stop`: `time`·`window`·`encoder_bit`·중단 이유.
- `observed`: 운용자 입력.

판정 제안(운용자 관측이 있을 때만):
- `duty_sign_ok` = 양의 명령에 물리 방향이 `pos` 인가. 아니면 "이 채널의 `duty_sign` 을 뒤집는다".
- `sensor_sign_ok` = 측정 부호가 물리 방향과 같은가. 아니면 "이 축 센서의 부호(`wheel[i].sign` 또는 `steer/lift.sign`)를 뒤집는다".
- 관측이 `none`(안 움직임)인데 측정이 움직였거나 그 반대면 `inconsistent` 로 표시하고 제안하지 않는다.

바퀴만:
- `rate_rad_s`: 비영 구간 뒤쪽 절반의 평균·최대(정상 상태 근사). 이 듀티의 무부하 속도이지 최대 속도가 아니다.

조향·승강만:
- `value`: 시작·끝·최소·최대, 창, 창 이탈 시각.

## 5. 호스트 코어 변경 (`McuLinkHost`)

- 생성자에 `commission: bool = False` 를 더한다. `client` 가 `BRINGUP` 일 때만 허용(그 밖은 `ValueError`).
- `commission=True` 의 `build_refusal`: `commission_build` 가 아니면 `not_commission_build`, `outputs_locked` 면 `allow_locked` 일 때만 허용. 기존 `BRINGUP` 의 "시운전 빌드에 `DRIVE` 금지" 는 `commission=False` 에서 그대로다. 그래서 같은 호스트가 두 빌드에 두 종류 명령을 섞어 보낼 수 없다(양방향).
- 무장 뒤 0 명령: `commission=True` 면 0 듀티 `COMMISSION`(axis 0, 유효시간 `zero_valid_ms`).
- 새 `commission(axis, duty_e4, expires_ns, now_ns) -> Blocked | None`: `drive()` 와 같은 검사(신선도·`ready`·빌드·만료·유효시간 상한·outbox)를 거쳐 `Commission` 을 넣는다. `commission=False` 면 `Blocked("not_commission_host")`. 거꾸로 `commission=True` 의 `drive()` 는 `Blocked("commission_host")`.
- outbox 규칙은 `COMMISSION` 을 `DRIVE` 와 같이 다룬다: 만료·권한 세대가 지난 것은 쓰지 않고 버림, `drop_drives` 가 함께 버림, `DISARM` 요청이 버림, 쓴 기록(`_written`)에 남김.
- 실행기 경로(`commission=False`, `client` REAL·HIL)는 바뀌지 않는다. 기존 시험이 그대로 통과해야 한다.

## 6. MCU 모형 (`firmware/tools/mcu_model.c`)

시험용 선택지를 더한다. 모두 호스트 시험용 합성값이고 하드웨어 값이 아니다.
- `--commission-build`: `commission_build = true`, `commission_duty_max = 0.15`, `commission_time_ms = 2000`(계획 제안값).
- `--commission-time-ms N`: 시간 상한을 줄여 MCU 상한 경로(`commission_time_limit` → `STOPPING`)를 시험한다.
- `--outputs-locked`: `outputs_locked = true`(모형은 출력을 그대로 plant 에 넣으므로 `--allow-locked` 경로 시험용 표시일 뿐이다).
- `--encoder-sign-flip 0|1`: 그 바퀴의 카운트 방향을 뒤집어 넣는다(센서 부호 제안·`encoder_polarity` 비트 시험).

## 7. 시험

- 단위(`tests/unit/plant/test_mcu_host_commission.py`): `commission=True` 의 빌드 거부 양방향, 0 `COMMISSION` 무장 흐름, `drive()`·`commission()` 상호 거부, 만료·세대·`drop_drives`·`DISARM` 의 outbox 규칙, `client` 제한.
- 계산(`tests/unit/plant/test_bringup_commission_calc.py`): 결과 계산(부호·잡음 문턱·뒤쪽 절반 평균·`inconsistent`·판정 제안)을 합성 `STATE` 로.
- 종단(`tests/integration/test_bringup_commission_tool.py`, MCU 모형): 정상 빌드 거부(아무것도 안 보냄), 바퀴 L 정상·부호, 엔코더 부호 뒤집기 → 센서 부호 제안, 조향 창 정지, 승강 정상, 시간 상한 단축 → 중단·정리, 듀티 초과는 도구가 거부, 무장 중 예외 → `DISARM` 확인.
- 정상 빌드 쪽 도구 ① 의 종단 10 건은 그대로 통과해야 한다.

## 8. 범위 밖

- 최대 바퀴 속도 측정(시운전 듀티 상한 밖 — 빌드 상수를 바꾸는 결정이 먼저다).
- 전원 강하·회생 과전압(오실로스코프).
- 설정 파일 자동 수정.
- 승강 위치 측정 방식(계획 3.5, 미정). 승강 시운전은 빈 포크·한 바퀴 미만 창에서만 한다.
