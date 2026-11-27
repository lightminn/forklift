# Jetson ↔ MCU 링크 계약 (mcu-link/3)

작성일: 2026-10-22 · 개정: 2026-11-11(version 2), 2026-11-12(version 3, 7절) · 상태: **초안, 실기기 미검증** · 구현: `firmware/core/`(C), `src/forklift_core/mcu_link/`(Python)

상위 제어(Jetson)와 하위 제어 MCU 사이의 바이트 프레임과 메시지 의미를 정한다. 전송로(USB CDC 또는 UART)는 아직 정하지 않았으므로([핀맵 검토](../design/2026-10-22-h743-pinmap-draft-review.md) 4절) 이 계약은 **순서가 보장되는 바이트 스트림**만 가정한다. 펌웨어 구조는 [MCU 펌웨어 코어 설계](../design/2026-10-22-mcu-firmware-core.md)를 따른다.

## 1. 프레임

```
wire  = COBS(body) 0x00
body  = version:u8  type:u8  seq:u16  payload:N  crc:u16
```

- 모든 다바이트 정수와 실수는 **little-endian**. 실수는 IEEE 754 binary32.
- `version` = 3. 다른 값은 버린다(version 1·2 프레임 포함).
- `crc` = CRC-16/CCITT-FALSE(다항식 0x1021, 초기값 0xFFFF, 반사 없음, 최종 XOR 0) 를 `version` 부터 `payload` 끝까지 계산. 검사값: ASCII `123456789` → `0x29B1`.
- COBS(Consistent Overhead Byte Stuffing)로 0x00 을 없앤 뒤 구분자 0x00 하나를 붙인다. 받는 쪽은 0x00 까지 모아 디코드하고, 깨진 프레임은 버리고 다음 0x00 부터 다시 맞춘다. 빈 프레임(연속 0x00)은 오류가 아니다.
- `body` 최대 **90 바이트**(v3, `STATE`). 넘으면 버린다. COBS 를 거친 프레임은 최대 91 바이트, 구분자 포함 92 바이트다.
- `seq` 는 보낸 쪽이 메시지마다 1 씩 올린다(0xFFFF 다음 0). 양방향이 각자 센다.
- 버린 프레임(COBS·CRC·버전·형식·길이·크기 오류)은 받는 쪽의 `rx_errors` 에 센다.
- 보내는 쪽은 **프레임을 통째로만** 송신 버퍼에 넣는다. 자리가 모자라면 그 프레임 전체를 버린다(앞부분만 보내면 뒤 프레임까지 깨진다).

## 2. 신선도: 부팅 ID · 무장 세대 · 시각 되돌려받기

MCU 는 받은 시각만으로 명령의 나이를 알 수 없다. 전송로에 오래 묶여 있던 명령, MCU 재부팅 전에 보낸 명령 묶음이 늦게 도착해도 "방금 받은 명령"으로 보이기 때문이다. 그래서 움직임을 여는 메시지(`ARM`, `DRIVE`)는 MCU 가 발급한 값을 되돌려 보내고, **유효 시간도 MCU 가 발급한 시각을 기준으로** 적는다.

| 값 | MCU 가 정하는 법 | 막는 것 |
|---|---|---|
| `boot_id` (u32) | 켤 때마다 하드웨어 난수로 새로 뽑는다(0 은 쓰지 않음) | MCU 재부팅 전에 보낸 명령 |
| `arm_gen` (u16) | `ARM` 을 받아들일 때마다 1 오른다(0xFFFF 다음 1, 0 은 "무장한 적 없음") | 이전 무장 구간의 명령 |
| `echo_mcu_time_us` (u64) | 호스트가 받은 **가장 최근 `STATE.mcu_time_us`** | 전송 지연·재생 |

- `mcu_time_us` 는 **64비트** 단조 시계다. 32비트였다면 약 71.6 분마다 같은 값이 돌아와, 같은 부팅·같은 무장 구간에서도 그만큼 지난 명령이 echo 나이 검사를 통과할 수 있다(64비트는 사실상 넘어가지 않는다).
- 명령의 마감은 **`deadline = echo_mcu_time_us + min(valid_after_echo_ms, command_timeout_ms)`**(MCU 시계). 호스트는 `valid_after_echo_ms` 를 "자기 쪽 만료 시각 − 그 `STATE` 를 받은 호스트 시각"의 **밀리초 내림**으로 채우고, 1 ms 미만이면 보내지 않는다. `STATE` 가 MCU 를 떠나 호스트에 닿기까지 걸린 시간만큼 MCU 쪽 마감이 이르게 잡히므로 **언제나 보수적**이다(늦어지는 일이 없다).
- MCU 는 `echo ≤ now ≤ deadline` 일 때만 `DRIVE` 를 받아들인다. `ARM` 은 `echo ≤ now ≤ echo + command_timeout_ms`.
- 남는 위험(밝혀 둠): MCU 가 재부팅된 뒤 새 `boot_id` 가 우연히 이전 값과 같을 때(2⁻³²)만 재부팅 전 명령이 통과할 수 있다. 그 경우에도 echo 가 새 부팅의 시각 범위 안이어야 한다.

## 3. 메시지

| type | 이름 | 방향 | payload |
|---|---|---|---|
| 0x01 | `DRIVE` | 호스트 → MCU | 33 바이트 `<ffffHBHIQ` |
| 0x02 | `ARM` | 호스트 → MCU | 12 바이트 `<IQ`: `boot_id`, `echo_mcu_time_us` |
| 0x03 | `DISARM` | 호스트 → MCU | 없음(언제나 받아들인다 — 멈추는 쪽이라 신선도 검사를 하지 않는다) |
| 0x04 | `CLEAR_FAULTS` | 호스트 → MCU | 4 바이트 `<I`: `boot_id` |
| 0x05 | `COMMISSION` | 호스트 → MCU | 19 바이트 `<BhHHIQ` (3.4, `FW_COMMISSION` 빌드만) |
| 0x81 | `STATE` | MCU → 호스트 | 84 바이트 `<QIHHBBIiiffff4h4h4BHHBB6H` |

길이가 표와 다른 메시지와 모르는 `type` 은 버린다(`rx_errors`).

### 3.1 `DRIVE`

| 필드 | 형 | 단위·의미 |
|---|---|---|
| `left_wheel_rad_s` | f32 | 왼쪽 뒤 바퀴 목표 각속도, 차량 전진 방향 구름이 + |
| `right_wheel_rad_s` | f32 | 오른쪽 뒤 바퀴 목표 각속도 |
| `steering_rad` | f32 | 조향 목표각(KTH7801 축을 보정해 얻은 조향 기구 각, + = 왼쪽) |
| `lift_m` | f32 | 승강 목표 위치. `flags.lift_hold` 이면 무시 |
| `valid_after_echo_ms` | u16 | 2절. 1…65535 |
| `flags` | u8 | bit0 `lift_hold`(승강은 현재 목표 유지). 나머지 비트는 0 |
| `arm_gen` | u16 | 지금 무장 세대 |
| `boot_id` | u32 | 지금 부팅 ID |
| `echo_mcu_time_us` | u64 | 2절 |

받아들이는 조건(이 순서로 검사, 처음 걸린 이유를 `reject_reason` 에 남김): 형식(유한한 실수, `valid_after_echo_ms ≥ 1`, 정의된 플래그만; `bad_value`) → `boot_id` 일치(`boot_mismatch`) → 모드 `ARMED`(`STOPPING` 이면 `stopping`, 그 밖은 `not_armed`) → `arm_gen` 일치(`arm_gen_mismatch`) → 마감 전(2절; `stale_echo`) → `seq` 가 마지막으로 받아들인 `DRIVE` 보다 새것(`(int16)(seq − last) > 0`; 무장 직후 첫 `DRIVE` 는 검사 안 함; `stale_seq`) → 무장 직후 두 바퀴 0 조건(`zero_first`, 3.2).

받아들이면 그 명령의 마감은 2절 식. 조향·승강 목표는 MCU 설정 한계로 자르고 설정된 최대 변화율로 따라간다. 자르면 `STATE.flags.limited`.

### 3.2 `ARM`, `DISARM`, `CLEAR_FAULTS`

`ARM` 거부 이유는 이 순서로 정한다: `boot_id` 다름 → `boot_mismatch`, 신선도 실패 → `stale_echo`, 모드가 `BOOT` → `driver_not_ready`, `STOPPING` → `stopping`, `ARMED` → `not_disarmed`, `FAULT`·`ESTOP` → `latched`, 비상정지 입력 → `estop_active`, 비상정지 핀이 없는 출력 잠금 아닌 정상 빌드 → `estop_unwired`(v2), 조향·승강 측정이 무효이거나 범위 밖이거나 전류 측정이 무효 → `sensor_invalid`(시운전 빌드는 3.4 의 예외).

- `ARM`: 위 검사를 모두 통과하면 `arm_gen` 을 올리고, 바퀴 목표 0, **조향·승강 목표 = 지금 측정값**, 적분 초기화, 마감 = `echo + command_timeout_ms`(뒤따르는 `DRIVE` 가 없으면 `STOPPING` 을 거쳐 `DISARMED`)로 `ARMED` 가 된다. 그 뒤 **두 바퀴 목표가 모두 0 인 `DRIVE` 를 먼저 받아야** 0 이 아닌 바퀴 목표를 받아들인다(`zero_first`).
- `DISARM`: `ARMED`·`STOPPING` 이면 바로 `DISARMED`(출력은 채널별 정지 상태). 다른 모드에서는 아무 것도 바꾸지 않는다(`FAULT`·`ESTOP` 래치는 그대로 남는다).
- `CLEAR_FAULTS`: `FAULT`·`ESTOP` 에서, `boot_id` 가 같고 비상정지 입력이 풀려 있고 조향·승강·전류 측정이 유효하면(v2: 시운전 빌드는 `ARM` 과 같이 **전류 측정 유효만** 요구한다) 래치를 지우고 `BOOT` 로 가 드라이버를 다시 초기화한다. **과전류와 드라이버 결함은 해제 조건에 넣지 않는다** — 드라이버 결함은 `CLR_FLT` 전까지 nFAULT 가 low 로 남고, 고압측 과전류 뒤에는 출력이 꺼져도 IPROPI 가 클램프 전압에 붙어 있어 "지금 전류"가 높게 읽히기 때문이다(SLVSG24C 8.3.4.1). 다시 초기화는 모든 DRVOFF 를 세운 다음에 `CLR_FLT` 를 보낸다(`CLR_FLT` 가 출력을 다시 켜기 때문). 초기화가 끝나고 nFAULT 가 정상이면 `DISARMED`, 아니면 `FAULT`(`driver_not_ready`). 주행 재개에는 그 뒤 `ARM` 이 또 필요하다.

### 3.4 `COMMISSION` (version 2)

S3 축별 개루프 시운전 전용이다(계획 `docs/plans/2026-11-11-sim-to-real-bringup.md` 3.4). `FW_COMMISSION` 빌드만 받고, 정상 빌드는 `wrong_build` 로 거부한다. 거꾸로 `FW_COMMISSION` 빌드는 `DRIVE` 를 `wrong_build` 로 거부한다. 단위가 다른 명령을 섞어 해석하지 않기 위해서다.

| 필드 | 형 | 의미 |
|---|---|---|
| `axis` | u8 | 0 L, 1 R, 2 M(조향), 3 H(승강). 그 밖은 `bad_value` |
| `duty_e4` | i16 | 그 축의 듀티 ×10000(−10000…10000). 그 축의 빌드 시운전 한계(아래 표)를 넘으면 `bad_value`(자르지 않는다, 정수 비교) |
| `valid_after_echo_ms` | u16 | 2절. 1…65535 |
| `arm_gen` | u16 | 지금 무장 세대 |
| `boot_id` | u32 | 지금 부팅 ID |
| `echo_mcu_time_us` | u64 | 2절 |

- 받아들이는 순서: 프레임 검사(version·길이·CRC) 뒤 **맨 앞에 `wrong_build`**(정상 빌드의 `COMMISSION`, 시운전 빌드의 `DRIVE` 는 내용과 무관하게 14). 그다음은 `DRIVE` 와 같다(형식 → `boot_id` → 모드 → `arm_gen` → 마감 → `seq` → 무장 직후 첫 명령은 듀티 0 `zero_first`). M·H 를 고른 명령은 그 축 측정이 무효면 13 `sensor_invalid` 로 거부한다.
- 한 번에 한 축만 움직이고, 나머지 축은 정지 상태다.
- **축별 시운전 한계**(2026-11-25, `docs/design/2026-11-25-bench-commission-limits.md` 1.1, 필드·version 무변경). 빌드 상수이고 메시지로 바뀌지 않는다.

  | | 잠긴 시운전 빌드 | 열린 시운전 빌드(`FW_MOTOR_OUTPUT=1`) |
  |---|---|---|
  | L·R 듀티 상한 | 0.15 | 1.0 |
  | L·R 연속 출력 시간 상한 | 2 s | 2 147 483 ms(검증기 ms 상한, 약 35.8 분) |
  | M·H 듀티·시간 상한 | 0.15·2 s | 0.15·2 s |

  - 열린 시운전 이미지는 정체 문자열 끝에 ` lim=lr1` 을 붙인다. 실제로 얼마나 세게·오래 돌릴지는 bring-up 도구의 인자가 정한다(도구 쪽 확인 규칙은 설계 1.2).
  - **출력 구간 하나에는 축 하나**: 시간 상한은 출력 구간의 첫 비영 출력 때 축으로 정한다. 그래서 출력 구간(`cm_on`)이 열린 동안 다른 축을 고른 `COMMISSION` 은 11 `stopping` 으로 거부한다. 이 거부는 지금 명령·마감·마지막 수락 seq 를 바꾸지 않는다. 축을 바꾸려면 지금 축에 0 을 보내고, 네 듀티가 모두 0 인 `STATE` 를 본 다음에 보낸다.
  - 거부 순서는 형식(축·듀티 한계·`valid_after_echo_ms`) → `boot_id` → 모드 → `arm_gen` → 마감 → `seq` → `zero_first` → 축 충돌 11 → 고른 M·H 축의 측정 13 이다.
- 시간 상한으로 들어간 `STOPPING` 에서도 `DISARM` 은 언제나 받아들여 바로 `DISARMED`(채널별 정지 상태)로 간다(3.2 와 같다. 정지 명령에는 예외를 두지 않는다). `FAULT`·`ESTOP` 래치는 그대로 남는다. 이때 네 축 전기 제동이 끝나고, `safe_brake=false` 채널은 코스트가 된다.
  - 줄일 수 없는 것은 **재무장 금지**다. 상한이 걸린 때부터 `stop_grace_ms` 가 끝날 때까지 `ARM` 은 `stopping` 으로 거부된다. `DISARM`·`CLEAR_FAULTS`·결함은 이 보류의 만료 시각을 앞당기거나 지우지 못한다.
  - 보류 시간은 **구동 재개 금지만** 보장한다. 전기 제동 유지나 실제 바퀴 정지를 뜻하지 않는다. 상한과 `DISARM` 이 같은 주기에 오면 전기 제동이 한 주기도 걸리지 않을 수 있다(`safe_brake=false` 면 바로 코스트).
  - 제동 상태를 끝까지 보려는 도구는 `STOPPING` 이 끝날 때까지 `DISARM` 을 보내지 않는다(시운전 도구가 그렇게 한다).
- `CLEAR_FAULTS` 는 `commission_time_limit`(24)을 지우지 않는다. 다음 `ARM` 이 지운다. 상한 뒤 `stop_grace_ms` 동안의 `ARM` 거부는 그 사이에 결함·`CLEAR_FAULTS` 가 끼어도 유지된다.
- **시운전 빌드의 `STOPPING`** 은 고르지 않은 축까지 **네 축 모두** 듀티 0·DRVOFF low(전기 제동)다. 단 그 채널에 드라이버 결함이 있거나 드라이버가 준비되지 않았으면 끊는다.
- **시간 상한은 실제 적용 출력으로 잰다**(`ARM`·`DISARM` 으로 지워지지 않는다. 한 주기에 `DISARM`→`ARM`→`COMMISSION`(0)→`COMMISSION`(비영)을 받아도 실제 출력이 0 인 주기가 없으면 시계가 이어진다): 네 축의 적용 듀티 가운데 하나라도 0 이 아닌 주기가 이어지는 동안 시계가 돌고, 모두 0 인 주기가 한 번 지나야 지워진다(같은 주기에 `COMMISSION`(0)→`COMMISSION`(비영)을 보내도 지워지지 않는다). 시계가 그 구간 축의 상한(위 표)을 넘으면 `STOPPING` → `DISARMED` 로 가고 `fault_flags.commission_time_limit` 을 켠다. 이 비트는 다음 `ARM` 까지 남는다.
- **시운전 빌드의 판정 예외**(S3 는 극성·범위를 처음 찾는 단계다):
  - `encoder_stuck_*`·`encoder_polarity_*`(18–21)는 비트만 래치하고 `FAULT` 로 가지 않는다. 다음 `ARM` 에서 지운다. 그래서 이 비트가 켜진 채 `ARMED` 일 수 있다(시운전 도구가 표시한다).
  - 고르지 않은 조향·승강 축의 측정 무효·범위 밖은 표시만 하고 `ARM` 을 막지 않는다(승강 KTH 가 없어도 바퀴 시운전이 된다). 고른 축이 출력 중 무효·범위 밖이 되면 결함이다.
  - 전류·nFAULT·과전류·비상정지·`encoder_gap`·`encoder_jump_*`·주기 지연은 정상 빌드와 같다.
- **단일 축 시운전 이미지 `fw-single`**(2026-11-25, `docs/plans/2026-11-25-bench-partial-parts.md` PR-B, 필드·코드·version·시험 벡터 무변경). 열린 시운전 빌드에 빌드 옵션을 더한 것이고, 정체 문자열 끝이 정확히 ` lim=lr1 sa1` 이다. 다른 이미지는 이 동작을 컴파일하지 않는다. 이 이미지에서만 위 규칙이 이렇게 바뀐다.
  - **무장의 축 고정**: 무장 뒤 처음 받아들인 `COMMISSION`(`zero_first` 때문에 듀티 0)이 그 무장의 축을 **그 무장이 끝날 때까지**(`DISARM`, 또는 만료·상한의 `STOPPING` 이 끝나 `DISARMED` 로 갈 때) 정한다. 다른 축의 `COMMISSION` 은 0 이든 아니든 11 `stopping` 으로 거부한다. 거부 순서는 위와 같은 자리(`zero_first` 뒤, 축 충돌 11)이고 지금 명령·마감·마지막 수락 seq 를 바꾸지 않는다.
  - **다른 축으로 바꾸기**: 축이 정해지기 전이라도, 마지막으로 적용된 주기에 DRVOFF 가 low 였던 채널과 다른 축은 네 DRVOFF 가 모두 high 로 적용된 주기가 한 번 지나기 전까지 11 `stopping` 이다(HAL 이 L→R→M→H 순서로 쓰므로, H 제동 중 한 주기에 `DISARM`→`ARM`→M(0) 이 오면 H 를 올리기 전에 M 을 내리게 된다). 무장 직후 축이 정해지기 전 주기는 네 채널 모두 high 다.
  - **시험하지 않는 채널은 잠금**: 고정 축이 아닌 세 채널의 DRVOFF 는 `STOPPING` 을 포함한 모든 모드에서 high 다. `STOPPING` 의 전기 제동은 고정 축만 받는다. `DISARM` 뒤에는 마지막으로 고정됐던 축만 자기 정지 상태(`safe_brake`)를 지키고 나머지는 high 다.
  - **M·H 의 센서**: M·H 를 고른 명령에 13 `sensor_invalid` 거부가 없고, 출력 중 조향·승강의 무효·범위 밖도 결함이 아니라 `STATE` 표시만이다. M·H 듀티·시간 상한(0.15·2 s), 과전류, 명령 만료, `DISARM`, DRVOFF 는 그대로다. 링크가 물리적으로 끊기면 호스트는 권한과 송신 큐를 버리고, MCU 는 마지막 명령의 만료로 `STOPPING` 에 들어간다. `DISARM` 은 연결이 살아 있거나 돌아온 뒤에 간다.
  - 호스트의 무장 0 명령은 이 이미지를 정체 문자열로 확인한 경로에서만 그 실행의 축으로 보낸다. 다른 경로는 지금처럼 L(0) 이다.

### 3.3 `STATE` (MCU 가 `state_period_ms` 마다 보냄)

| 필드 | 형 | 의미 |
|---|---|---|
| `mcu_time_us` | u64 | MCU 단조 시계(마이크로초, 켤 때 0 부터). 호스트 시계와 섞지 않는다 |
| `boot_id` | u32 | 2절 |
| `arm_gen` | u16 | 2절(무장한 적 없으면 0) |
| `last_rx_seq` | u16 | 마지막으로 **받아들인** 호스트 메시지의 `seq` |
| `mode` | u8 | 0 `BOOT`, 1 `DISARMED`, 2 `ARMED`, 3 `STOPPING`, 4 `FAULT`, 5 `ESTOP` |
| `reject_reason` | u8 | 마지막 거부 이유: 0 없음, 1 `bad_value`, 2 `not_armed`, 3 `latched`, 4 `estop_active`, 5 `driver_not_ready`, 6 `stale_seq`, 7 `zero_first`, 8 `boot_mismatch`, 9 `arm_gen_mismatch`, 10 `stale_echo`, 11 `stopping`, 12 `not_disarmed`, 13 `sensor_invalid`, 14 `wrong_build`(v2, 3.4), 15 `estop_unwired`(v2: 비상정지 핀이 없고 **출력이 잠기지 않은** 정상 빌드가 `ARM` 을 거부. `outputs_locked` 빌드와 `FW_COMMISSION` 빌드는 허용) |
| `fault_flags` | u32 | 결함 비트(아래) |
| `wheel_count_left` | i32 | 왼쪽 바퀴 누적 엔코더 카운트(4체배). **32비트 모듈러** — 차이는 2³² 를 법으로 구한다 |
| `wheel_count_right` | i32 | 오른쪽 |
| `wheel_rate_left_rad_s` | f32 | 추정 각속도 |
| `wheel_rate_right_rad_s` | f32 | |
| `steering_rad` | f32 | 측정 조향각. 무효면 NaN |
| `lift_m` | f32 | 측정 승강 위치. 무효면 NaN |
| `current_ma` | 4 × i16 | L, R, M, H 모터 전류, 단위 mA(IPROPI 환산, ±32,767 로 포화) |
| `duty_e4` | 4 × i16 | 적용한 듀티 ×10000(−10000…10000), L, R, M, H. v2: `flags.outputs_locked` 이면 코어가 **계산한** 듀티이고 물리 출력은 0 이다(HIL 모형이 이 값으로 돈다) |
| `driver_status` | 4 × u8 | 각 DRV8244 의 최근 SPI 응답 상태 바이트: 상위 2비트는 항상 `11`(아니면 SPI 배선·통신 이상), 하위 6비트는 `FAULT VMOV VMUV OCP TSD SPI_ERR`(SLVSG24C 표 8-21). 응답을 아직 못 받았으면 0 |
| `rx_errors` | u16 | 받기 오류 누적(포화) |
| `cycle_overruns` | u16 | 제어 주기 지연 누적(포화) |
| `flags` | u8 | bit0 `limited`, bit1 `command_fresh`(마감 전 명령이 있음), bit2 `zero_seen`, bit3 `drivers_ready`, bit4 `commission_build`(v2), bit5 `hil_plant`(v2), bit6 `outputs_locked`(v2), bit7 `estop_unwired`(v2) |
| `reset_cause` | u8 | v2: 마지막 시작 원인. 0 전원·BOR, 1 리셋 핀, 2 IWDG, 3 소프트웨어, 4 그 밖. 한 부팅 동안 모든 `STATE` 에 같은 값 |
| `kth_frames` | 2 × u16 | v3: 축별(M 조향, H 승강) 받은 **신선한** KTH7801 프레임 수. 16비트 모듈러 |
| `kth_crc_errors` | 2 × u16 | v3: 그중 **설정된 CRC 변형**으로 CRC 가 틀린 프레임 수. 16비트 모듈러 |
| `kth_alt_crc_errors` | 2 × u16 | v3: 그중 **다른 CRC 변형**이었다면 CRC 가 틀렸을 프레임 수. 진단 전용 — 위치·유효성 판단에 쓰지 않는다. 16비트 모듈러 |

`fault_flags` 비트: 0 `estop_input`, 1 `command_timeout`, 2 `driver_fault_l`, 3 `driver_fault_r`, 4 `driver_fault_m`, 5 `driver_fault_h`, 6 `steer_sensor_invalid`, 7 `lift_sensor_invalid`, 8 `overcurrent_l`, 9 `overcurrent_r`, 10 `overcurrent_m`, 11 `overcurrent_h`, 12 `driver_not_ready`, 13 `steer_out_of_range`, 14 `lift_out_of_range`, 15 `cycle_overrun`, 16 `current_sense_invalid`, 17 `encoder_gap`(v2, 두 바퀴 공통: 같은 주기에 읽는다), 18 `encoder_stuck_l`, 19 `encoder_stuck_r`, 20 `encoder_polarity_l`, 21 `encoder_polarity_r`, 22 `encoder_jump_l`, 23 `encoder_jump_r`, 24 `commission_time_limit`(v2, 3.4). 엔코더 비트 18–23 은 뒤 바퀴 L·R 에만 적용하고, 조향·승강(KTH 절대각)은 6·7·13·14 를 쓴다. 나머지(25–31)는 0 이어야 하며, 켜진 미정의 비트는 받는 쪽이 오류로 처리한다.

**KTH CRC 계수(v3)** — S2 의 CRC 변형 확정용(계획 S2, 도착일 절차서). 두 변형(`KTH_CRC_G704_BYTES`, `KTH_CRC_REFLECTED_MSB_FIRST`)은 4096 개 각도 중 256 개(0x000·데이터시트 예 0x0FF 포함)에서 CRC 가 같다. 그래서 설정 변형의 오류 0 만으로는 증거가 되지 않고, 다른 변형이 틀렸을 프레임(`kth_alt_crc_errors`)을 실제로 지났다는 것이 함께 있어야 한다.
- 세 계수는 같은 신선 프레임 집합에서 같은 주기에 갱신한다. 신선한 프레임 하나에 `kth_frames` 가 1 오르고, 그 프레임이 설정 변형으로 틀리면 `kth_crc_errors`, 다른 변형으로 틀리면 `kth_alt_crc_errors` 가 1 오른다. 신선하지 않은(받지 못한) 프레임은 어느 계수도 올리지 않는다.
- 축마다 제어 주기당 많아야 1 오르고, 제어 주기는 1 ms 이상이다 — 축당 초당 1000 이하.
- **부팅 때만** 0 으로 초기화한다. `ARM`·`DISARM`·`CLEAR_FAULTS`·재연결에는 그대로 둔다.
- 호스트는 같은 부팅 안의 **인접** `STATE` 사이 차분 `(new − old) mod 2¹⁶`(부호 없는 차분, `wheel_count_delta` 의 반범위 부호 차분이 아님)을 정수로 더해 읽는다. 첫 `STATE` 와 마지막 `STATE` 만 빼면 랩을 놓친다. 인접 `STATE` 간격(`mcu_time_us`)이 30 s 를 넘거나(1000/s × 30 s < 2¹⁶), MCU 시각이 되돌아가거나, `boot_id` 가 바뀐 구간은 복원할 수 없어 그 판정을 무효로 한다. 짧은 `STATE` 누락은 누적 계수라서 복원된다. 합하기 **전에 구간마다** `Δerrors ≤ Δframes`, `Δalt ≤ Δframes` 이고 `Δframes ≤ ⌈Δt × 1000⌉ + 1` 이어야 한다. 아니면(같은 부팅 안의 초기화, 간격이 숨긴 랩, 구간 안의 모순) 무효다. 기록 행에 세 계수 중 하나라도 없거나 형식이 틀리면 0 으로 채우지 않고 무효다.
- `flags.hil_plant` 인 빌드의 계수는 모형이 만든 프레임의 것이다 — 합성 진단으로만 보고 실물 S2 확정에 쓰지 않는다.
- 판정 도구: `tools/bringup.py kth --record run.jsonl --configured-m V --configured-h V --image ID`(`src/forklift_core/plant/kth_check.py`). 축마다 `configured_ok`(frames ≥ 1000, errors = 0, alt ≥ 100, 마지막 `STATE` 에서 축이 유효하고 범위 안), `other_variant`(frames ≥ 1000, alt = 0, errors ≥ 100), 그 밖은 `undecided`(이유와 함께). 문턱은 작업용 값이고 검증된 신뢰수준이 아니다. alt 증거는 두 변형을 가르는 프레임을 지났다는 뜻이지 범위 전체를 지났다는 뜻이 아니다 — 범위는 S2 가 따로 본다. 판정 결과에 설정 변형과 이미지 식별자가 함께 남는다.

**호스트의 결함 해석(v2)**:
- 비트 17·22·23 이 켜져 있는 동안은 그 바퀴(17 은 두 바퀴) 카운트의 연속성이 끊긴 것으로 본다. 펌웨어는 그 표본의 차분만 버리고 누적 카운트는 이어 간다. 끊긴 횟수는 알 수 없다.
- 초기화 뒤 첫 주기가 gap 상한보다 늦으면 결함으로 보지 않지만 그 차분은 버린다. 그래서 부팅 뒤 첫 `STATE` 의 카운트가 0 이 아닐 수 있다. 호스트는 누적값이 아니라 같은 부팅 안의 차분만 쓴다.
- 출력이 잠긴 HIL 아닌 빌드는 stuck·polarity 를 판정하지 않는다(바퀴가 안 도는 것이 정상).
- 세션 중 `boot_id` 가 바뀌면 `reset_cause` 가 0(전원·BOR)이어도 모터 기동 브라운아웃일 수 있으니 경고로 다룬다.

**빌드 비트를 읽는 호스트 규칙(v2)**:
- 실행기(텔레옵·자율)는 `commission_build` 면 연결하지 않는다.
- **양방향 차단(총괄 조건 2026-11-11)**: 실차 실행기는 `hil_plant` 보드에 연결하지 않는다. HIL 실행기는 `hil_plant` 가 아닌 보드에 연결하지 않는다.
- HIL 실행기는 `hil_plant` 보드에서 `outputs_locked`·`estop_unwired` 가 함께 켜져 있어도 텔레옵·자율을 연다. HIL 빌드는 PWM 을 초기화하지 않고 DRVOFF 를 high 로 고정해, 바퀴가 물리적으로 돌 수 없다. 코어 보드만으로 돌아 비상정지 핀도 없다.
- 실차 실행기는 `outputs_locked`·`estop_unwired` 가 하나라도 켜져 있으면 텔레옵·자율을 거부한다.
- 구현과 시험: `src/forklift_core/mcu_link/build_policy.py`(`connect_refusal`, `drive_refusal`, `commission_allowed`), `tests/unit/mcu_link/test_build_policy.py`.
- 송신 경로: `McuLinkHost(client=...)` 가 `ARM` 요청과 `DRIVE` 송신 전에 빌드 비트를 검사한다(`build_refusal`). 무장 중 금지 빌드의 `STATE` 를 받으면 권한을 철회하고 `DISARM` 을 보낸다. 실행기는 `drive_refusal` 을 따른다. bring-up 도구(`Client.BRINGUP`)는 시운전 빌드에 `DRIVE` 를 보내지 않고, 출력 잠금 보드에는 모형·HIL 용 허용(`allow_locked`)이 있을 때만 보낸다.
- bring-up 도구(STATE 기록·시운전·안전 시나리오)는 실행기가 아니다. 이 비트들을 기록·표시하되 연결은 막지 않는다. 시운전 도구는 `commission_build` 가 아니면 `COMMISSION` 을 보내지 않는다.

## 4. 시간과 만료

- `ARMED` 에서 마감이 지나면 `STOPPING`: 바퀴 목표 0·조향과 승강은 그 순간 측정값 유지로 `stop_grace_ms` 동안 **출력을 켠 채** 제어해 세운 뒤 `DISARMED`(출력은 채널별 정지 상태). `command_timeout` 비트는 다음 `ARM` 까지 남는다. `STOPPING` 중 `DRIVE`·`ARM` 은 `stopping` 으로 거부한다 — 늦게 온 명령이 정지를 취소하지 못한다. `STOPPING` 중 비상정지·결함이 생기면 그쪽이 즉시 이긴다.
- MCU 는 주기마다 **메시지를 처리하기 전에** 마감을 먼저 판정한다. 마감 시각과 같은 시각은 아직 유효하다.
- v2: **시운전 빌드의 `STOPPING`**(만료·시운전 시간 상한)은 정상 빌드의 속도 0 제어가 아니다. 그 주기에 모든 축 듀티를 0 으로 두는 전기 제동이고, 출력은 켠 채 `stop_grace_ms` 동안 유지한다. 바퀴 듀티 변화율 제한은 크기가 커지는 쪽에만 걸리고, 줄어드는 쪽은 즉시다(정지 예산 보호).
- 호스트는 `STATE` 를 받은 **호스트 단조 시각**으로 신선도를 판단한다(`mcu_time_us` 와 빼지 않는다). 다만 `valid_after_echo_ms` 계산에는 그 `STATE` 를 받은 호스트 시각을 쓴다.

## 5. 전송량

`STATE` 는 body 90 바이트, COBS·구분자 포함 최대 92 바이트다(v3). 10 ms 마다 보내면 9.2 kB/s(UART 8N1 기준 92 kbit/s)라 115,200 bps 에서는 링크의 약 80 % 를 쓴다. UART 라면 460,800 bps 이상이나 USB CDC 를 쓰고, 그러지 못하면 `state_period_ms` 를 늘린다. MCU 는 송신을 막고 기다리지 않는다 — 보낼 자리가 없으면 그 `STATE` 를 통째로 버린다. `STATE` 를 못 받으면 호스트의 echo 가 오래돼 명령이 거부되는데, 그것은 정상적인 정지 동작이다.

## 6. 고정 시험 벡터

C·Python 시험은 아래 바이트와 같아야 한다(형식 시험용 값이라 상태 의미는 따지지 않는다). v3: `version` 이 모든 메시지의 CRC 에 들어가므로 모든 벡터를 version 3 으로 다시 만들었다. 3절 형식 문자열과 Python 표준 라이브러리(`struct`, `binascii.crc_hqx(body, 0xFFFF)`)로 한 번 만들어 고정했다. 실수 0.3 은 binary32 로 반올림된 값이다.

- `arm_1234` — `ARM`, seq 0x1234, boot_id 0xDEADBEEF, echo 0x0000000501020304:
  `0e03023412efbeadde04030201050101031f0500`
- `drive_0007` — `DRIVE`, seq 7, left 1.5, right −2.25, steer 0.125, lift 0.3, valid_after_echo_ms 150, flags 1, arm_gen 0x0203, boot_id 0xDEADBEEF, echo 0x0000000100ABCDEF:
  `04030107010103c03f010310c00101073e9a99993e960b010302efbeaddeefcdab02010101033f4600`
- `clear_0008` — `CLEAR_FAULTS`, seq 8, boot_id 0xDEADBEEF: `0403040807efbeaddea4c100`
- `disarm_0009` — `DISARM`, seq 9: `0403030903d4fc00`
- `commission_000a` — `COMMISSION`, seq 0x000A, axis 2, duty_e4 −1500, valid_after_echo_ms 150, arm_gen 0x0203, boot_id 0xDEADBEEF, echo 0x0000000100ABCDEF:
  `0403050a050224fa960a0302efbeaddeefcdab02010101037f4500`
- `state_0102` — `STATE`, seq 0x0102, 필드 순서대로 `0x0000000211223344, 0xDEADBEEF, 0x0203, 0x0102, 2, 7, 0x00018001, −123456, 654321, 1.25, −0.5, 0.0625, 0.75, (1500, −200, 30, 4000), (5000, −10000, 2500, 0), (0xC0, 0xC4, 0xE0, 0x00), 17, 2, 0x0B, 0, kth_frames (0x1234, 0xFFFF), kth_crc_errors (3, 0), kth_alt_crc_errors (0x0100, 7)`:
  `0a03810201443322110201010eefbeadde03020201020701800108c01dfefff1fb09010103a03f010102bf0103803d0108403fdc0538ff1e09a00f8813f0d8c4090104c0c4e002110202020b063412ffff0301010103010703c0b000`
- `state_0103` — `STATE`, seq 0x0103, `state_0102` 과 같되 `fault_flags` = bit 17·19·22·24(`0x014A0000`), `flags` = bit 0·3·4·6·7(`0xD9`), `reset_cause` = 2, kth_frames (0x8000, 1), kth_crc_errors (0xFFFF, 2), kth_alt_crc_errors (5, 0x7FFF):
  `0a03810301443322110201010befbeadde030202010207010a4a01c01dfefff1fb09010103a03f010102bf0103803d0108403fdc0538ff1e09a00f8813f0d8c4090104c0c4e00211020203d90203800104ffff02020505ff7fc42100`

## 7. 버전 관리

- **벤치 확장(8 절, 2026-11-27, bench console plan A1a)**: 운영 `version` 3 은 그대로이고, 벤치 이미지 전용 메시지 셋(0x06·0x08·0x82)을 더했다. 그 세대는 제품 토큰 `bench5` 로 정한다. 다른 이미지는 이 type 을 모른다.

필드를 바꾸면 `version` 을 올린다. 한 버전 안에서 필드 의미를 바꾸지 않는다.

- 2026-11-25 정정(필드·version 무변경, 총괄 결정 — 설계 `2026-11-25-bench-commission-limits`): 3.4 의 시운전 한계가 축별·빌드별 상수가 됐다. 열린 시운전 빌드의 L·R 은 1.0 과 검증기 ms 상한이다. 출력 구간 안의 축 변경은 11 이고, 듀티 한계는 정수로 비교한다.
- **version 3 (2026-11-12, 콘솔·펌웨어-H743 합의, 계약 주인 콘솔, 총괄 배정)**:
  - `STATE` 끝에 축별 KTH 계수 셋(`kth_frames`, `kth_crc_errors`, `kth_alt_crc_errors`, 각 2 × u16 모듈러)을 붙였다(3.3). 도착일 S2 의 CRC 변형 확정이 목적이다.
  - `STATE` payload 72 → 84 바이트, body 상한 80 → 90 바이트.
  - 모든 고정 벡터를 version 3 으로 다시 만들었다.
  - 받는 쪽은 version 3 만 받는다(version 2 장비 배포 없음).
- **version 2 (2026-11-11, 콘솔·펌웨어-H743 합의, 계약 주인 콘솔)**:
  - 엔코더 결함 비트 17–23 을 더했다.
  - `COMMISSION`(0x05)과 `commission_time_limit`(24)을 더했다.
  - 거부 이유 14 `wrong_build`, 15 `estop_unwired` 를 더했다.
  - `STATE.flags` 의 bit4–7 로 빌드 종류를 알린다.
  - `STATE` 끝에 `reset_cause` 를 붙였다.
  - 승강 원점 비트는 승강 방식 결정(계획 3.5) 뒤 version 3 으로 미룬다.
  - 2026-11-12 정정(필드·코어 동작 무변경, 펌웨어-H743 권고): 3.2·3.4 의 시간 상한 `STOPPING` 과 `DISARM` 문구를 코어 동작에 맞췄다(래치 유지, 보류 만료 시각 고정, 보류는 구동 재개 금지일 뿐 제동·정지가 아님 — Codex 동의). 이전 문구('`DISARM` 으로 줄일 수 없다')는 재무장 금지만 뜻하는 것으로 읽는다.

## 8. 벤치 확장 (wire 세대 `bench5`)

벤치 이미지 전용 메시지다(계획 `docs/plans/2026-11-27-bench-console.md` 1 절). 운영 `version` 3 의 메시지·필드·고정 벡터(3–6 절)는 바꾸지 않는다. 프레임(1 절)과 `version` 3 은 같다. 벤치 확장의 세대는 USB 제품 문자열 끝 토큰 `bench5` 로 정한다.

- **벤치 이미지**: `FW_BUILD=commission`, `FW_MOTOR_OUTPUT=1`, `FW_LINK=usb`, `FW_BENCH=1` 이고 fw-single 이 아니다. 제품 문자열은 `… lim=lr1 bench5` 로 끝난다. 다른 조합은 configure 가 거부한다.
- **다른 이미지**: 0x06·0x08·0x82 를 모르는 `type` 으로 버린다(`rx_errors`). 코어를 `FW_CORE_BENCH` 없이 컴파일하기 때문이다. 받아 들이지도, 보내지도 않는다.
- **벤치 이미지는 운영 `ARM` 을 `wrong_build` 로 거부한다.** 무장은 `BENCH_ARM` 으로만 한다.

| type | 이름 | 방향 | payload |
|---|---|---|---|
| 0x06 | `BENCH_SET` | 호스트 → MCU | 52 바이트 `<IQIBBBbI4H4Ii` |
| 0x08 | `BENCH_ARM` | 호스트 → MCU | 16 바이트 `<IQI` |
| 0x82 | `BENCH_STATE` | MCU → 호스트 | 48 바이트 `<HIIBBBBHiiiiBBHHHHBBBBBB` |

### 8.1 `BENCH_SET`

한 실행의 설정이다. `DISARMED` 이고 시운전 출력 구간이 닫혀 있을 때만 받는다. 같은 `run_id` 의 `BENCH_ARM` 이 **한 번** 소비한다.

| 필드 | 형 | 뜻 |
|---|---|---|
| `boot_id` | u32 | 지금 부팅 ID |
| `echo_mcu_time_us` | u64 | 받은 `STATE` 의 시각. `command_timeout_ms` 안이고, 마지막 연결 변경(8.4) 이후여야 한다 |
| `run_id` | u32 | 호스트가 실행마다 새로 뽑는 값, 0 아님 |
| `mode` | u8 | 1 `duty`(시간 상한 듀티 구동). 2 `hold` 는 다음 세대(A1b) |
| `axis` | u8 | 0 L, 1 R, 2 M, 3 H — 이 실행에서 0 이 아닌 출력을 낼 수 있는 유일한 축 |
| `source` | u8 | 0 없음. 1 `kth`·2 `mt` 는 다음 세대 |
| `count_sign` | i8 | ±1, 듀티 부호와 원천 카운트 증가 방향의 관계(측정 원천이 생기는 세대에서 씀) |
| `bypass` | u32 | bit0 `steer_range`(M 범위 검사 끔, KTH 유효 검사는 유지), bit1 `lift_range`(H). 나머지 비트는 0. 멈추는 쪽(과전류·전류센서·드라이버 결함·STOP/DISARM·명령 만료·IWDG·사이클 초과)은 비트가 없다 |
| `duty_max_e4[4]` | u16 ×4 | 축별 듀티 상한, 1…10000 |
| `time_ms[4]` | u32 ×4 | 축별 연속 출력 시간 상한, 1…2147483 |
| `stop_counts` | i32 | 거리 정지, 0 = 없음(다음 세대) |

거부 이유(처음 걸린 것): 벤치 이미지 아님 → `wrong_build`, `DISARMED` 아님·출력 구간 열림 → `not_disarmed`, `boot_id` → `boot_mismatch`, echo → `stale_echo`, 값 → `bad_value`(이 세대가 받지 않는 `mode`·`source`·`stop_counts` 포함). 받아들이면 그 설정을 대기시키고 `last_rx_seq` 를 올린다. 대기 설정은 1 s 뒤 만료되고, `DISARM`·결함·연결 변경에서 버려진다.

### 8.2 `BENCH_ARM`

`ARM` 과 같은 검사(3.2: `boot_id`, 신선도, 모드, 시간 상한 보류, 전류 측정)에 더해, echo 가 마지막 연결 변경 이후여야 하고(`stale_echo`), 대기 중인 `BENCH_SET` 이 같은 연결에서 왔고 만료되지 않았으며 `run_id` 가 같아야 한다(`bad_value`). 받아들이면 `ARM` 처럼 무장하고 그 설정을 소비한다. 우회 비트와 축별 상한은 **이 무장 동안만** 쓰이고, 모드가 `ARMED`·`STOPPING` 을 벗어나면 빌드 기본값으로 돌아간다. 이전 실행의 결과(8.3)는 이때 비워진다.

### 8.3 실행과 결과

- **준비**: 무장 뒤 첫 `COMMISSION` 은 0 이어야 한다(3.4 `zero_first`). 실행 축이 아닌 축의 0(무장 직후 L(0) 등)은 준비로 받는다. 실행 축이 아닌 축의 0 이 아닌 듀티는 `bad_value`.
- **구동 시작**: 실행 축의 0 이 아닌 `COMMISSION` 을 처음 받아들인 때.
- **끝 전이**(처음 일어난 것이 결과의 정지 사유): 구동 시작 뒤 받아들인 0 `COMMISSION`(1 `end_zero`), 명령 만료(2), 시간 상한(3), `DISARM`(4), 결함(5), 비상정지(6), 연결 변경(7). `end_zero` 와 연결 변경에서는 MCU 가 스스로 `STOPPING` 의 전기제동을 거쳐 `DISARMED` 로 가며, 이때 `command_timeout` 비트는 서지 않는다. 결함·비상정지는 결과만 남기고 출력과 래치는 3.2 그대로다. 끝난 뒤 `stopping`·`not_armed` 로 거부된 구동 요청만 결과의 `blocked` 에 센다(다른 축·다른 부팅 등 다른 이유의 거부는 세지 않음).
- **`STOPPING` 중 `DISARM`(벤치 이미지만, 3.2 의 예외)**: 받아들이되(`last_rx_seq`) 제동을 줄이지 않는다. `stop_grace_ms` 가 끝나야 `DISARMED` 가 된다. 연결 변경 뒤 늦게 도착한 이전 호스트의 `DISARM` 이 제동을 끊지 못하게 하기 위해서다. `ARMED` 에서의 `DISARM` 은 3.2 그대로 바로 `DISARMED`(정지 사유 4).
- **출력 구간의 시간 상한**은 구간이 열릴 때의 상한(실행 중이면 그 실행의 상한)을 구간이 닫힐 때까지 쓴다. 실행이 끝난 직후에도 빌드 기본값으로 다시 재지 않는다.
- **결과**: 구동한 실행만 결과를 남긴다(`result_run_id`, 정지 사유, 결과의 축·원천, 차단한 요청 수). 최초 원인은 덮어쓰지 않으며, 다음 `BENCH_ARM` 이 새 설정을 소비할 때까지 남는다. 구동 없이 끝난 실행은 결과가 없다(`result_run_id` 가 그 실행이 아님).

### 8.4 연결 변경 (USB)

- **변화의 시작**: 버스 리셋은 레지스터 상태로 본다 — 컨트롤러가 `USBRST` 를 알렸거나(OTG 인터럽트 전) 장치 주소가 0 인 동안(TinyUSB 가 리셋 처리에서 0 으로 만들고 새 열거의 `SET_ADDRESS` 가 다시 정함) 연결은 바뀌는 중이고, 제어 주기가 처음 본 시각이 시작이다(해상도 1 주기: 같은 주기 안에서는 명령 만료를 먼저로 본다). USB 클록을 켜기 전에도 바뀌는 중이다. 분리·suspend 는 OTG 인터럽트(TinyUSB 사건 훅)가 사건을 낼 때 세고, 그 밖의 변화(mount·umount·resume·DTR 값의 바뀜 — RTS 만 바뀌거나 같은 DTR 을 다시 쓴 요청은 아님)는 주 루프의 생명주기 콜백이 센다. 시작 시각(HAL 시계)은 앞선 변화를 제어 주기가 본 뒤의 첫 시작만 남긴다. 변화가 끝나지 않은 동안 연결은 '바뀌는 중'이다.
- **확정**: 주 루프가 OTG 인터럽트를 잠시 끈 채 TinyUSB 수신 FIFO 를 비우고 세대를 확정한다. 송신 FIFO 는 비우지 않는다(보드 설계 5.1).
- **제어 주기**는 바뀌는 중에는 한 바이트도 읽지 않는다. 그 주기의 `drive_begin` 이 그 주기의 시각으로 변화를 적용한다(전기제동의 `stop_grace_ms` 는 이 시각부터 온전히). `ARMED` 면 무장을 끝내는데, 명령 마감이나 출력 구간의 시간 상한이 변화 시작보다 먼저 지났으면 그것이 최초 원인이다(실제 순서). 확정을 본 주기는 수신 링에 남은 이전 연결의 바이트와 반쪽 프레임을 버리고, 다음을 한다.

- 대기 중인 `BENCH_SET` 을 버린다. (무장은 위에서 이미 끝났다 — 무장은 연결을 넘지 못한다.)
- **남는 틈(받아들임, Codex A1a 3·4 회차)**: 이전 호스트의 `CLEAR_FAULTS`(echo 없음)가 FIFO 비우기 뒤 늦게 들어오면 같은 부팅의 래치를 지울 수 있다. 해제는 출력이 꺼진 채 드라이버를 다시 초기화할 뿐이고, 구동은 새 연결의 STATE 를 echo 한 `BENCH_ARM` 이 있어야 다시 시작된다. 그 해제의 사유는 이전 세션의 호스트 기록(A1a-2)에만 있다 — 같은 부팅에서 그 뒤 새로 래치된 결함도 함께 지워졌을 수 있으므로, 새 호스트는 연결 직후의 결함 비트를 기록한다.
- 벤치 명령(`BENCH_SET`·`BENCH_ARM`)은 **확정을 적용한 주기 이후에 보낸** `STATE` 를 echo 해야 받는다(경계 시각 = 그 주기의 시각, 이전에 보낸 모든 `STATE` 는 그보다 이르다). 그 주기 끝에 보낸 `STATE` 부터 쓸 수 있다. `BENCH_ARM` 은 3.2 의 `ARM` 검사(모드·보류 등)에서 먼저 걸리면 그 이유를 낸다. 경계 echo 검사는 그 다음이다.

### 8.5 `BENCH_STATE`

벤치 이미지는 `STATE` 마다 바로 뒤에 `BENCH_STATE` 를 붙여 **한 번에** 송신 버퍼에 넣는다. 자리가 모자라면 둘 다 버리고 `dropped_pairs` 를 올린다. `BENCH_STATE` 의 헤더 seq 는 공통 송신 seq 를 이어 받는다(짝 `STATE` + 1). 송신 seq 는 넣기에 성공할 때만 오르므로, 넣지 못한 짝은 seq 공백 없이 `mcu_time_us` 만 건너뛴다(`dropped_pairs` 는 이 넣기 실패만 센다). 이미 넣은 짝이 연결 변경 때 송신 링에서 버려지거나 송신 FIFO 꼬리가 잘리면 seq 공백이나 한쪽만 도착이 생길 수 있다 — 호스트는 짝 seq 로 맞지 않는 표본을 버린다.

| 필드 | 형 | 뜻 |
|---|---|---|
| `pair_seq` | u16 | 짝 `STATE` 의 헤더 seq |
| `run_id` | u32 | 마지막으로 소비한 실행(없으면 0) |
| `result_run_id` | u32 | 결과가 있는 실행(없으면 0) |
| `axis`, `source` | u8, u8 | 마지막 실행의 축·원천 |
| `count_state` | u8 | 0 `none_yet`, 1 `valid`, 2 `lost` (이 세대는 0) |
| `kth_lost` | u8 | bit0 M, bit1 H (이 세대는 0) |
| `raw_angle` | u16 | 측정 축의 같은 주기 원시각 (이 세대는 0) |
| `cumulative`, `target`, `distance_ref`, `cut_position` | i32 ×4 | 원천 카운트 값들 (이 세대는 0) |
| `stop_reason` | u8 | 8.3 의 정지 사유(0 없음; 8–12 는 다음 세대) |
| `distance_confirm` | u8 | 0 없음, 1 도달, 2 보류 (이 세대는 0) |
| `blocked` | u16 | 끝난 뒤 차단한 구동 요청 수(포화) |
| `dropped_pairs` | u16 | 버린 짝 수(포화) |
| `evidence_age_ms`, `end_evidence_age_ms` | u16, u16 | 증거 나이(포화, 0xFFFF = 아직 없음; 이 세대는 0xFFFF) |
| `mcu_bypass` | u8 | 지금 무장에서 쓰이는 우회 비트(8.1) |
| `drive` | u8 | bit0–1 실행 상태(0 없음, 1 준비, 2 구동, 3 끝), bit4–5 모드 |
| `conn_gen` | u8 | 연결 변경 수(mod 256) |
| `reserved` | u8 | 0 |
| `result_axis`, `result_source` | u8, u8 | 결과의 축·원천 |

운영 `STATE.fault_flags` 의 bit 25–31 은 벤치 이미지에서도 0 이다. 우회 표식은 `mcu_bypass` 에만 있다.

### 8.6 고정 시험 벡터

`tests/unit/mcu_link/test_bench_messages.py` 와 `firmware/core/test/test_bench_link.c` 가 함께 쓴다.

- `bench_set_000b` — seq 0x000B, boot 0xDEADBEEF, echo 0x0000000100ABCDEF, run 0x12345678, mode 1, axis 2, source 0, count_sign 1, bypass 1, duty (10000, 10000, 2500, 1500), time (2147483, 2147483, 30000, 2000), stop 0:
  `0403060b08efbeaddeefcdab020101010778563412010203010101010c10271027c409dc059bc420049bc4200330750103d007010101010103e94200`
- `bench_arm_000c` — seq 0x000C, boot 0xDEADBEEF, echo 0x0000000100ABCDEF, run 0x12345678:
  `0403080c08efbeaddeefcdab0201010107785634124c7300`
- `bench_state_0103` — seq 0x0103, 필드 순서대로 `0x0102, 0x12345678, 0x11223344, 2, 0, 0, 0, 0x0ABC, −123456, 654321, −7, 8, 1, 0, 3, 4, 0xFFFF, 250, 1, 0x13, 5, 0, 2, 0`:
  `1003820301020178563412443322110201010abc0ac01dfefff1fb0906f9ffffff08010102010203020404fffffa04011305020203659700`
