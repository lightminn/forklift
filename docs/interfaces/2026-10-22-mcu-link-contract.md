# Jetson ↔ MCU 링크 계약 (mcu-link/1)

작성일: 2026-10-22 · 상태: **초안, 실기기 미검증** · 구현: `firmware/core/`(C), `src/forklift_core/mcu_link/`(Python)

상위 제어(Jetson)와 하위 제어 MCU 사이의 바이트 프레임과 메시지 의미를 정한다. 전송로(USB CDC 또는 UART)는 아직 정하지 않았으므로([핀맵 검토](../design/2026-10-22-h743-pinmap-draft-review.md) 4절) 이 계약은 **순서가 보장되는 바이트 스트림**만 가정한다. 펌웨어 구조는 [MCU 펌웨어 코어 설계](../design/2026-10-22-mcu-firmware-core.md)를 따른다.

## 1. 프레임

```
wire  = COBS(body) 0x00
body  = version:u8  type:u8  seq:u16  payload:N  crc:u16
```

- 모든 다바이트 정수와 실수는 **little-endian**. 실수는 IEEE 754 binary32.
- `version` = 1. 다른 값은 버린다.
- `crc` = CRC-16/CCITT-FALSE(다항식 0x1021, 초기값 0xFFFF, 반사 없음, 최종 XOR 0) 를 `version` 부터 `payload` 끝까지 계산. 검사값: ASCII `123456789` → `0x29B1`.
- COBS(Consistent Overhead Byte Stuffing)로 0x00 을 없앤 뒤 구분자 0x00 하나를 붙인다. 받는 쪽은 0x00 까지 모아 디코드하고, 깨진 프레임은 버리고 다음 0x00 부터 다시 맞춘다. 빈 프레임(연속 0x00)은 오류가 아니다.
- `body` 최대 **80 바이트**. 넘으면 버린다.
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
| 0x81 | `STATE` | MCU → 호스트 | 71 바이트 `<QIHHBBIiiffff4h4h4BHHB` |

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

`ARM` 거부 이유는 이 순서로 정한다: `boot_id` 다름 → `boot_mismatch`, 신선도 실패 → `stale_echo`, 모드가 `BOOT` → `driver_not_ready`, `STOPPING` → `stopping`, `ARMED` → `not_disarmed`, `FAULT`·`ESTOP` → `latched`, 비상정지 입력 → `estop_active`, 조향·승강 측정이 무효이거나 범위 밖이거나 전류 측정이 무효 → `sensor_invalid`.

- `ARM`: 위 검사를 모두 통과하면 `arm_gen` 을 올리고, 바퀴 목표 0, **조향·승강 목표 = 지금 측정값**, 적분 초기화, 마감 = `echo + command_timeout_ms`(뒤따르는 `DRIVE` 가 없으면 `STOPPING` 을 거쳐 `DISARMED`)로 `ARMED` 가 된다. 그 뒤 **두 바퀴 목표가 모두 0 인 `DRIVE` 를 먼저 받아야** 0 이 아닌 바퀴 목표를 받아들인다(`zero_first`).
- `DISARM`: `ARMED`·`STOPPING` 이면 바로 `DISARMED`(출력은 채널별 정지 상태). 다른 모드에서는 아무 것도 바꾸지 않는다.
- `CLEAR_FAULTS`: `FAULT`·`ESTOP` 에서, `boot_id` 가 같고 비상정지 입력이 풀려 있고 조향·승강·전류 측정이 유효하면 래치를 지우고 `BOOT` 로 가 드라이버를 다시 초기화한다. **과전류와 드라이버 결함은 해제 조건에 넣지 않는다** — 드라이버 결함은 `CLR_FLT` 전까지 nFAULT 가 low 로 남고, 고압측 과전류 뒤에는 출력이 꺼져도 IPROPI 가 클램프 전압에 붙어 있어 "지금 전류"가 높게 읽히기 때문이다(SLVSG24C 8.3.4.1). 다시 초기화는 모든 DRVOFF 를 세운 다음에 `CLR_FLT` 를 보낸다(`CLR_FLT` 가 출력을 다시 켜기 때문). 초기화가 끝나고 nFAULT 가 정상이면 `DISARMED`, 아니면 `FAULT`(`driver_not_ready`). 주행 재개에는 그 뒤 `ARM` 이 또 필요하다.

### 3.3 `STATE` (MCU 가 `state_period_ms` 마다 보냄)

| 필드 | 형 | 의미 |
|---|---|---|
| `mcu_time_us` | u64 | MCU 단조 시계(마이크로초, 켤 때 0 부터). 호스트 시계와 섞지 않는다 |
| `boot_id` | u32 | 2절 |
| `arm_gen` | u16 | 2절(무장한 적 없으면 0) |
| `last_rx_seq` | u16 | 마지막으로 **받아들인** 호스트 메시지의 `seq` |
| `mode` | u8 | 0 `BOOT`, 1 `DISARMED`, 2 `ARMED`, 3 `STOPPING`, 4 `FAULT`, 5 `ESTOP` |
| `reject_reason` | u8 | 마지막 거부 이유: 0 없음, 1 `bad_value`, 2 `not_armed`, 3 `latched`, 4 `estop_active`, 5 `driver_not_ready`, 6 `stale_seq`, 7 `zero_first`, 8 `boot_mismatch`, 9 `arm_gen_mismatch`, 10 `stale_echo`, 11 `stopping`, 12 `not_disarmed`, 13 `sensor_invalid` |
| `fault_flags` | u32 | 결함 비트(아래) |
| `wheel_count_left` | i32 | 왼쪽 바퀴 누적 엔코더 카운트(4체배). **32비트 모듈러** — 차이는 2³² 를 법으로 구한다 |
| `wheel_count_right` | i32 | 오른쪽 |
| `wheel_rate_left_rad_s` | f32 | 추정 각속도 |
| `wheel_rate_right_rad_s` | f32 | |
| `steering_rad` | f32 | 측정 조향각. 무효면 NaN |
| `lift_m` | f32 | 측정 승강 위치. 무효면 NaN |
| `current_ma` | 4 × i16 | L, R, M, H 모터 전류, 단위 mA(IPROPI 환산, ±32,767 로 포화) |
| `duty_e4` | 4 × i16 | 적용한 듀티 ×10000(−10000…10000), L, R, M, H |
| `driver_status` | 4 × u8 | 각 DRV8244 의 최근 SPI 응답 상태 바이트: 상위 2비트는 항상 `11`(아니면 SPI 배선·통신 이상), 하위 6비트는 `FAULT VMOV VMUV OCP TSD SPI_ERR`(SLVSG24C 표 8-21). 응답을 아직 못 받았으면 0 |
| `rx_errors` | u16 | 받기 오류 누적(포화) |
| `cycle_overruns` | u16 | 제어 주기 지연 누적(포화) |
| `flags` | u8 | bit0 `limited`, bit1 `command_fresh`(마감 전 명령이 있음), bit2 `zero_seen`, bit3 `drivers_ready` |

`fault_flags` 비트: 0 `estop_input`, 1 `command_timeout`, 2 `driver_fault_l`, 3 `driver_fault_r`, 4 `driver_fault_m`, 5 `driver_fault_h`, 6 `steer_sensor_invalid`, 7 `lift_sensor_invalid`, 8 `overcurrent_l`, 9 `overcurrent_r`, 10 `overcurrent_m`, 11 `overcurrent_h`, 12 `driver_not_ready`, 13 `steer_out_of_range`, 14 `lift_out_of_range`, 15 `cycle_overrun`, 16 `current_sense_invalid`. 나머지는 0 이어야 하며, 켜진 미정의 비트는 받는 쪽이 오류로 처리한다.

## 4. 시간과 만료

- `ARMED` 에서 마감이 지나면 `STOPPING`: 바퀴 목표 0·조향과 승강은 그 순간 측정값 유지로 `stop_grace_ms` 동안 **출력을 켠 채** 제어해 세운 뒤 `DISARMED`(출력은 채널별 정지 상태). `command_timeout` 비트는 다음 `ARM` 까지 남는다. `STOPPING` 중 `DRIVE`·`ARM` 은 `stopping` 으로 거부한다 — 늦게 온 명령이 정지를 취소하지 못한다. `STOPPING` 중 비상정지·결함이 생기면 그쪽이 즉시 이긴다.
- MCU 는 주기마다 **메시지를 처리하기 전에** 마감을 먼저 판정한다. 마감 시각과 같은 시각은 아직 유효하다.
- 호스트는 `STATE` 를 받은 **호스트 단조 시각**으로 신선도를 판단한다(`mcu_time_us` 와 빼지 않는다). 다만 `valid_after_echo_ms` 계산에는 그 `STATE` 를 받은 호스트 시각을 쓴다.

## 5. 전송량

`STATE` 는 body 77 바이트, COBS·구분자 포함 최대 79 바이트다. 10 ms 마다 보내면 7.9 kB/s(UART 8N1 기준 79 kbit/s)라 115,200 bps 에서는 링크의 약 69 % 를 쓴다. UART 라면 460,800 bps 이상이나 USB CDC 를 쓰고, 그러지 못하면 `state_period_ms` 를 늘린다. MCU 는 송신을 막고 기다리지 않는다 — 보낼 자리가 없으면 그 `STATE` 를 통째로 버린다. `STATE` 를 못 받으면 호스트의 echo 가 오래돼 명령이 거부되는데, 그것은 정상적인 정지 동작이다.

## 6. 고정 시험 벡터

C·Python 시험은 아래 바이트와 같아야 한다(형식 시험용 값이라 상태 의미는 따지지 않는다). 3절 형식 문자열과 Python 표준 라이브러리(`struct`, `binascii.crc_hqx(body, 0xFFFF)`)로 한 번 만들어 고정했다. 실수 0.3 은 binary32 로 반올림된 값이다.

- `arm_1234` — `ARM`, seq 0x1234, boot_id 0xDEADBEEF, echo 0x0000000501020304:
  `0e01023412efbeadde0403020105010103764500`
- `drive_0007` — `DRIVE`, seq 7, left 1.5, right −2.25, steer 0.125, lift 0.3, valid_after_echo_ms 150, flags 1, arm_gen 0x0203, boot_id 0xDEADBEEF, echo 0x0000000100ABCDEF:
  `04010107010103c03f010310c00101073e9a99993e960b010302efbeaddeefcdab020101010388be00`
- `clear_0008` — `CLEAR_FAULTS`, seq 8, boot_id 0xDEADBEEF: `0401040807efbeadde024e00`
- `disarm_0009` — `DISARM`, seq 9: `0401030903bc1100`
- `state_0102` — `STATE`, seq 0x0102, 필드 순서대로 `0x0000000211223344, 0xDEADBEEF, 0x0203, 0x0102, 2, 7, 0x00018001, −123456, 654321, 1.25, −0.5, 0.0625, 0.75, (1500, −200, 30, 4000), (5000, −10000, 2500, 0), (0xC0, 0xC4, 0xE0, 0x00), 17, 2, 0x0B`:
  `0a01810201443322110201010eefbeadde03020201020701800108c01dfefff1fb09010103a03f010102bf0103803d0108403fdc0538ff1e09a00f8813f0d8c4090104c0c4e002110202040b2cbb00`

## 7. 버전 관리

필드를 바꾸면 `version` 을 올린다. 한 버전 안에서 필드 의미를 바꾸지 않는다.
