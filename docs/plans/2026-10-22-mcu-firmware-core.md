# MCU 펌웨어 코어 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 실기기 없이 시험할 수 있는 하위 제어 MCU 펌웨어 코어(C)와 Jetson 쪽 링크 대응(Python)을 만든다.

**Architecture:** `firmware/core/` 는 C11 순수 모듈(입력·시각을 인자로 받고 호출자 소유 구조체만 바꿈)과 HAL 함수 표를 받는 `fw_app` 로 이뤄진다. 호스트에서 CMake 로 빌드해 자체 단언 헤더 시험을 돌리고, pytest 가 그 빌드·실행과 C↔Python 링크 교차 시험을 맡는다. Python 쪽은 `forklift_core.mcu_link` 로 프레임·메시지만 다루며 시리얼·ROS 를 import 하지 않는다.

**Tech Stack:** C11(gcc/clang, 호스트), CMake ≥ 3.16 + CTest, Python ≥ 3.10(표준 라이브러리 `struct`·`binascii`·`dataclasses`·`enum`), pytest.

**Spec:** [MCU 펌웨어 코어 설계](../design/2026-10-22-mcu-firmware-core.md), [링크 계약 mcu-link/1](../interfaces/2026-10-22-mcu-link-contract.md), 하드웨어 근거는 [핀맵 초안 검토](../design/2026-10-22-h743-pinmap-draft-review.md).

## Global Constraints

- 코어 C 는 C11, `-Wall -Wextra -Werror -Wconversion -Wshadow`, 동적 할당·전역 가변 상태 없음, 수학은 `float`(`<math.h>` 의 `f` 함수).
- C 파일·함수는 `snake_case`, 외부 함수는 모듈 접두사(`crc_`, `cobs_`, `link_`, `wheel_enc_`, `abs_axis_`, `kth7801_`, `pi_`, `slew_`, `drv8244_`, `drive_`, `fw_app_`) (CONTRIBUTING 3절). 자체 C 는 `firmware/.clang-format` 으로 맞춘다(4절).
- 하드웨어 미확정 값(극성·게인·영점·비율·R_IPROPI·한계·`safe_brake`)은 설정 구조체 필드로만 받고 기본값을 넣지 않는다. `drive_config_valid` 가 설계 7절의 조건을 검사한다. 시험 설정에는 `/* synthetic */` 주석(CONTRIBUTING 6절).
- 시각: HAL 원시 시계는 `uint32_t` µs(넘어감, 차분만 씀), 코어가 넓힌 MCU 시각·명령 마감·echo 는 `uint64_t` µs. 설정의 시간 값은 2³¹ µs 미만.
- 계약 문서가 정본이다. 메시지 형식 문자열: `DRIVE <ffffHBHIQ`(33), `ARM <IQ`(12), `CLEAR_FAULTS <I`(4), `DISARM` 0, `STATE <QIHHBBIiiffff4h4h4BHHB`(71), body 최대 80. MCU 시각은 64비트(`uint64_t`). 고정 벡터는 계약 6절의 다섯 개.
- Python 은 Ruff(88자) 규칙, 공개 함수 type hint, SI 단위 접미사. `forklift_core.mcu_link` 는 `forklift_core` 의 다른 모듈을 import 하지 않는다.
- C 빌드가 필요한 시험은 `tests/integration/test_firmware_host.py` 에 모으고 `cmake`·C 컴파일러가 없으면 사유를 달고 skip(통과로 세지 않음).
- 커밋은 파일 단위 stage, 메시지는 `type(scope): summary` 영어로 무엇이 왜 바뀌었는지까지.

## Review Focus

1. 쓰레기 바이트·잘린 프레임·연속 0x00 이 섞인 스트림 → 다음 정상 프레임부터 다시 맞고 오류 계수만 오른다. (Task 2, 3)
2. 전송로에 묶였다 늦게 온 `DRIVE`, MCU 재부팅 전에 보낸 `ARM → 0 → 주행` 묶음 → 신선도·부팅 ID 로 거부. (Task 7)
3. MCU 시각 `uint32_t` µs 넘어감 직전·직후의 명령 만료와 echo 나이. (Task 6, 7)
4. 하드웨어 카운터 0x0000 ↔ 0xFFFF 양방향 넘어감, 주기 하나가 늦은 표본 → 누적·속도가 끊기거나 배로 뛰지 않음. (Task 5)
5. 조향 영점이 0/4095 경계 근처일 때 각도 펼치기, 절단점을 지나는 한계 설정 거부. (Task 5, 7)

---

## 파일 구조

```
firmware/
  README.md  .clang-format  CMakeLists.txt
  core/include/fw/  crc.h cobs.h link.h wheel_enc.h abs_axis.h pi.h drv8244.h drive.h app.h
  core/src/         crc.c cobs.c link.c wheel_enc.c abs_axis.c pi.c drv8244.c drive.c app.c
  core/test/        fw_test.h test_crc.c test_cobs.c test_link.c test_wheel_enc.c test_abs_axis.c
                    test_pi.c test_drv8244.c test_drive.c test_app.c
  tools/link_tool.c 호스트 전용 인코드/디코드(교차 시험용)
src/forklift_core/mcu_link/  __init__.py framing.py messages.py
tests/unit/mcu_link/         test_framing.py test_messages.py
tests/integration/test_firmware_host.py
```

---

### Task 1: 빌드 골격, CRC, COBS (C)

**Files:** Create `firmware/CMakeLists.txt`, `firmware/README.md`, `firmware/core/include/fw/crc.h`, `cobs.h`, `firmware/core/src/crc.c`, `cobs.c`, `firmware/core/test/fw_test.h`, `test_crc.c`, `test_cobs.c`, `tests/integration/test_firmware_host.py` (`firmware/.clang-format` 은 이미 있음)

**Interfaces:**
- Produces:
  - `uint16_t crc16_ccitt_false(const uint8_t *data, size_t len);`
  - `uint8_t crc4_g704_bytes(const uint8_t *data, size_t len);` (CRC-4/G-704: 다항식 0x3 반사형, 초기 0, 최종 XOR 0)
  - `typedef enum { KTH_CRC_G704_BYTES = 0, KTH_CRC_REFLECTED_MSB_FIRST = 1 } kth_crc_variant;` · `uint8_t crc4_kth7801(uint16_t angle12, kth_crc_variant variant);`
  - `size_t cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);` — 구분자 없음, 0 = 공간 부족.
  - `size_t cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);` — 0 = 잘못된 입력 또는 공간 부족.
  - pytest 세션 fixture `firmware_build` → 빌드 디렉터리 `Path`(도구 없으면 skip).
- [ ] **Step 1: 실패하는 시험** — `test_crc.c`: `crc16_ccitt_false("123456789") == 0x29B1`, 빈 입력 `0xFFFF`; `crc4_g704_bytes("123456789") == 0x7`(CRC-4/G-704 표준 검사값); 두 KTH 방식 모두 `crc4_kth7801(0x0FF, v) == 0x2`(데이터시트 예); `crc4_kth7801(0x123, 0) != crc4_kth7801(0x123, 1)`. `test_cobs.c`: `{00}` → `01 01`, `{00 00}` → `01 01 01`, `{11 22 00 33}` → `03 11 22 02 33`, `{11 22 33 44}` → `05 11 22 33 44`, 0x01…0xFE 254 바이트 → `FF` + 254 바이트(255 바이트, 끝에 0x01 을 덧붙이지 않음); 왕복; 0x00 이 든 입력·남은 길이를 넘는 코드는 decode 0; 공간 부족은 0. `test_firmware_host.py::test_c_core_unit_tests_pass`: 빌드 후 `ctest --output-on-failure` 종료 코드 0.
- [ ] **Step 2: 실패 확인** — `python -m pytest tests/integration/test_firmware_host.py -q`.
- [ ] **Step 3: 구현** — CMake: C11, 위 경고, `fw_core` 정적 라이브러리, 시험마다 실행 파일 + `add_test`, `m` 링크. fixture 는 `cmake -S firmware -B <tmp>` → `cmake --build <tmp> -j2`. `fw_test.h`: `CHECK`, `CHECK_EQ_U`, `CHECK_EQ_I`, `CHECK_NEAR`, 실패 수를 종료 코드로.
- [ ] **Step 4: 통과 확인.**
- [ ] **Step 5: 커밋** — `feat(firmware): host-built C core skeleton with link CRC-16, KTH7801 CRC-4 variants and COBS`

### Task 2: 링크 메시지 (C)

**Files:** Create `link.h`, `link.c`, `test_link.c`

**Interfaces:**
- Produces:
  - 상수 `LINK_VERSION 1`, `LINK_BODY_MAX 80`, `LINK_WIRE_MAX 83`, 타입 `LINK_DRIVE 0x01`, `LINK_ARM 0x02`, `LINK_DISARM 0x03`, `LINK_CLEAR_FAULTS 0x04`, `LINK_STATE 0x81`.
  - `typedef struct { float left_wheel_rad_s, right_wheel_rad_s, steering_rad, lift_m; uint16_t valid_after_echo_ms; uint8_t flags; uint16_t arm_gen; uint32_t boot_id; uint64_t echo_mcu_time_us; } link_drive;`
  - `typedef struct { uint32_t boot_id; uint64_t echo_mcu_time_us; } link_arm;` · `typedef struct { uint32_t boot_id; } link_clear;`
  - `typedef struct { uint64_t mcu_time_us; uint32_t boot_id; uint16_t arm_gen, last_rx_seq; uint8_t mode, reject_reason; uint32_t fault_flags; int32_t wheel_count_left, wheel_count_right; float wheel_rate_left_rad_s, wheel_rate_right_rad_s, steering_rad, lift_m; int16_t current_ma[4], duty_e4[4]; uint8_t driver_status[4]; uint16_t rx_errors, cycle_overruns; uint8_t flags; } link_state;`
  - `typedef struct { uint8_t type; uint16_t seq; union { link_drive drive; link_arm arm; link_clear clear; link_state state; } u; } link_msg;`
  - `size_t link_encode(const link_msg *msg, uint8_t *wire, size_t cap);` (구분자 포함, 0 = 실패)
  - `typedef enum { LINK_OK, LINK_BAD_COBS, LINK_BAD_CRC, LINK_BAD_VERSION, LINK_BAD_TYPE, LINK_BAD_LENGTH, LINK_OVERSIZE } link_status;` · `link_status link_decode_body(const uint8_t *body, size_t len, link_msg *out);`
  - `typedef struct { uint8_t buf[LINK_WIRE_MAX]; size_t len; bool overflow; uint16_t errors; } link_rx;` · `void link_rx_init(link_rx*);` · `bool link_rx_feed(link_rx*, uint8_t byte, link_msg *out);` (오류는 `errors` 포화 증가)
  - `bool link_seq_newer(uint16_t candidate, uint16_t last);`
- [ ] **Step 1: 실패하는 시험** — 계약 6절 고정 벡터 다섯 개를 `static const uint8_t` 배열로 넣고: 인코드 결과가 바이트 단위로 같다, 디코드하면 모든 필드가 같다(`state_0102` 는 28 값 전부). 스트림: `[55 66][00][arm_1234][00 00][CRC 비트 하나 뒤집은 drive_0007][disarm_0009]` 를 한 바이트씩 → 메시지 2 개, `errors == 2`. body 81 바이트 → `LINK_OVERSIZE`, 버전 2 → `LINK_BAD_VERSION`, 타입 0x7F → `LINK_BAD_TYPE`, payload 5 바이트 `ARM` → `LINK_BAD_LENGTH`. (`STATE` 의 미정의 비트 검사는 받는 쪽인 Python 몫이다.) `link_seq_newer(0, 0xFFFF)` true, `(0xFFFF, 0)` false, `(5, 5)` false.
- [ ] **Step 2–4:** 실패 확인 → 구현(바이트 단위 little-endian 직렬화, float 은 `memcpy` 로 `uint32_t` 변환; 구조체 통째 `memcpy` 금지) → 통과.
- [ ] **Step 5: 커밋** — `feat(firmware): mcu-link/1 messages with boot id, arm generation and MCU-time echo; resynchronising receiver; contract golden vectors`

### Task 3: Python 링크 대응

**Files:** Create `src/forklift_core/mcu_link/__init__.py`, `framing.py`, `messages.py`, `tests/unit/mcu_link/test_framing.py`, `test_messages.py`

**Interfaces:**
- Produces:
  - `framing`: `crc16_ccitt_false(data: bytes) -> int`, `cobs_encode(data: bytes) -> bytes`, `cobs_decode(data: bytes) -> bytes`(오류 `FrameError(ValueError)`), `class FrameDecoder` — `feed(data: bytes) -> list[bytes]`(검증된 body), 속성 `errors: int`.
  - `messages`: `Mode(IntEnum)` BOOT 0 … ESTOP 5, `RejectReason(IntEnum)` 계약 순서, `FaultFlag(IntFlag)` 비트 0–16, `StateFlag(IntFlag)` 비트 0–3; frozen dataclass `Drive(left_wheel_rad_s, right_wheel_rad_s, steering_rad, lift_m, valid_after_echo_ms, lift_hold, arm_gen, boot_id, echo_mcu_time_us)`, `Arm(boot_id, echo_mcu_time_us)`, `Disarm()`, `ClearFaults(boot_id)`, `McuState(...)` — 계약 필드 그대로에 더해 SI 속성 `current_a: tuple[float, ...]`(mA/1000), `duty: tuple[float, ...]`(/10000); `encode(message, seq: int) -> bytes`(wire), `decode(body: bytes) -> tuple[int, Message]`(오류 `LinkError(ValueError)`), `class LinkDecoder: feed(data: bytes, received_ns: int) -> list[Received]`, frozen `Received(seq: int, message: Message, received_ns: int)` — `received_ns` 는 호스트 `time.monotonic_ns()` 영역(docstring).
  - `Drive`/`Arm`/`ClearFaults` 생성 검사: 유한한 실수, `valid_after_echo_ms` 1…65535, `arm_gen` 1…65535, `boot_id` 1…2³²−1, `echo_mcu_time_us` 0…2⁶⁴−1 아니면 `ValueError`.
  - `def valid_after_echo_ms(expires_ns: int, state_received_ns: int) -> int` — 호스트 만료 시각과 echo 한 `STATE` 를 받은 호스트 시각(둘 다 `monotonic_ns`)의 차이를 밀리초 내림, 65535 로 자름; 내림 결과가 1 ms 미만이면 `ValueError`(올림 보정은 마감을 늘리므로 하지 않는다).
  - `McuState` 디코드: 미정의 `fault_flags`(비트 17 이상)·`flags` 비트, 범위 밖 `mode`·`reject_reason` 은 `LinkError`.
  - `def wheel_count_delta(newer: int, older: int) -> int` — 32비트 모듈러 차이(−2³¹…2³¹−1).
- [ ] **Step 1: 실패하는 시험** — 계약 6절 벡터 다섯 개 인코드·디코드 일치; `valid_after_echo_ms(expires_ns=1_150_000_000, state_received_ns=1_000_000_000) == 150`, 지난 만료는 `ValueError`; CRC·COBS 표준 예(Task 1 과 같음); `FrameDecoder` 스트림 재동기(Task 2 와 같은 구성) + 프레임이 두 `feed` 에 걸쳐 잘려도 복원; NaN `Drive` → `ValueError`; 미정의 fault 비트 `STATE` → `LinkError`; `Received.received_ns` 는 넘긴 값 그대로; `wheel_count_delta(-2**31, 2**31 - 1) == 1`.
- [ ] **Step 2: 실패 확인** — `python -m pytest tests/unit/mcu_link -q` → ImportError.
- [ ] **Step 3: 구현** — `struct` 형식은 Global Constraints 의 문자열.
- [ ] **Step 4: 통과 확인**, `ruff check`·`ruff format --check`.
- [ ] **Step 5: 커밋** — `feat(mcu_link): host side of mcu-link/1 (framing, messages, stream decoder), serial- and ROS-free`

### Task 4: link_tool 과 C↔Python 교차 시험

**Files:** Create `firmware/tools/link_tool.c`; Modify `firmware/CMakeLists.txt`, `tests/integration/test_firmware_host.py`

**Interfaces:**
- Produces: `link_tool encode drive <seq> <left> <right> <steer> <lift> <valid_after_echo_ms> <flags> <arm_gen> <boot_id> <echo>` / `encode arm <seq> <boot_id> <echo>` / `encode disarm <seq>` / `encode clear <seq> <boot_id>` / `encode state <seq> <28 값: 계약 3.3 순서, 배열은 펼침>` → 표준출력 wire 16진(소문자). `link_tool decode` → 표준입력 16진 wire 한 줄마다 `<type> <seq> <값…>`(실수 `%.9g`), 오류면 `error <status>`.
- [ ] **Step 1: 실패하는 시험** — seed `20261022` 의 `random.Random`, float32 로 표현되는 값만(`struct.unpack("<f", struct.pack("<f", x))[0]`):
  - `test_python_frames_decode_in_c`: `Drive` 200·`Arm` 20·`ClearFaults` 20·`McuState` 50 → Python `encode` → `link_tool decode` 필드 일치.
  - `test_c_frames_decode_in_python`: 같은 값을 `link_tool encode` → Python `decode` 일치.
  - `test_c_and_python_wire_bytes_match`: 같은 메시지의 wire 바이트 동일.
  실패 메시지에 seed 와 인덱스.
- [ ] **Step 2–4.**
- [ ] **Step 5: 커밋** — `test(firmware): C and Python agree on mcu-link/1 wire bytes for seeded random messages`

### Task 5: 바퀴 엔코더·절대각 축·PI·변화율 제한

**Files:** Create `wheel_enc.h/.c`, `abs_axis.h/.c`, `pi.h/.c`(`slew_step` 포함)와 시험 셋.

**Interfaces:**
- Produces:
  - `typedef struct { uint32_t counts_per_rev; uint16_t window; int8_t sign; } wheel_enc_config;` (`WHEEL_ENC_WINDOW_MAX 64`) · `typedef struct { uint32_t count; float rate_rad_s; /* 링버퍼: count, t_us */ } wheel_enc;` · `bool wheel_enc_init(wheel_enc*, const wheel_enc_config*, uint16_t hw, uint32_t now_us);` · `void wheel_enc_update(wheel_enc*, const wheel_enc_config*, uint16_t hw, uint32_t now_us);` — `count` 은 32비트 모듈러, 속도는 창 처음·끝 표본의 실제 시각 차이로.
  - `bool kth7801_decode(uint16_t frame, kth_crc_variant v, uint16_t *angle12);`
  - `typedef struct { kth_crc_variant crc; float zero_offset_rad; int8_t sign; float sensor_per_joint; float min_joint, max_joint; uint16_t invalid_limit; } abs_axis_config;` · `bool abs_axis_config_valid(const abs_axis_config*);` (`max(|min|,|max|) × sensor_per_joint < π`, `min < max`, `invalid_limit ≥ 1`, `sensor_per_joint > 0`) · `typedef struct { float position; bool valid, out_of_range; uint16_t bad_streak; uint32_t crc_errors; } abs_axis;` · `void abs_axis_init(abs_axis*);` (`valid = false`) · `void abs_axis_update(abs_axis*, const abs_axis_config*, uint16_t frame, bool fresh);`
  - `typedef struct { float kp, ki, kff, out_max, deadband; } pi_config;` · `typedef struct { float integral; } pi_state;` · `float pi_step(const pi_config*, pi_state*, float target, float measured, float dt_s);` · `void pi_reset(pi_state*);` · `float slew_step(float current, float target, float max_rate, float dt_s);`
- [ ] **Step 1: 실패하는 시험** —
  - `test_wheel_enc.c`: hw 0xFFF0 에서 +0x20 씩 3 번 → `count == 96`; 0x0010 에서 −0x20 씩 3 번 → `(int32_t)count == −96`; `sign = −1` 이면 반대; `count` 가 `0xFFFFFFF0` 근처에서 넘어가도 모듈러로 이어짐; 4096 카운트/회전·창 20·1000 µs 간격 +4 카운트 → 창이 찬 뒤 `rate ≈ 6.1359`(= 4 × 2π/4096/0.001, ±1e−3); 한 표본만 2000 µs 늦고 그 사이 카운트가 +8 이면 속도는 그대로(±1 %); `now_us` 가 0xFFFFFF00 에서 넘어가도 같음; 창 0·65, `counts_per_rev` 0, `sign ∉ {−1,1}` 은 init false.
  - `test_abs_axis.c`: 프레임 `0x0FF2` 는 각도 0x0FF 로 받아들임(표의 상수, 구현으로 CRC 를 다시 계산하지 않음), `0x0FF3` 은 거부·`crc_errors` +1·위치 유지; `fresh = false` 도 실패 표본; `invalid_limit = 3` → 연속 3 번 실패 뒤 `valid == false`, 정상 하나로 회복; `zero_offset` = 4000/4096 × 2π 일 때 raw 4090 과 raw 10 의 환산 차이가 16 카운트 상당(연속); 한계 밖 측정 → `out_of_range == true`, `valid` 는 그대로; `min=-3.0, max=3.0, sensor_per_joint=1.1` 은 config 거부.
  - `test_pi.c`: `kp = 2` → `pi_step(1, 0.25)` 1.5 → `out_max = 1` 로 1.0; 포화 상태 같은 방향 오차 10 번에 `integral` 불변, 반대 방향이면 줄어듦; `deadband = 0.1`, `|e| < 0.1` → 출력 0·적분 유지; `kff = 0.1, target = 5` → +0.5. `slew_step(0, 1, 2, 0.1) == 0.2`, `slew_step(0.9, 1, 2, 0.1) == 1.0`.
- [ ] **Step 2–4.**
- [ ] **Step 5: 커밋** — `feat(firmware): wheel encoder modular unwrap with time-stamped windowed rate, KTH7801 axis with freshness/CRC streak and range checks, conditional-integration PI, slew limiter`

### Task 6: DRV8244 프레임과 초기화 순서기

**Files:** Create `drv8244.h/.c`, `test_drv8244.c`

**Interfaces:**
- Produces: 레지스터 상수(`FAULT_SUMMARY 0x01`, `STATUS1 0x02`, `COMMAND 0x08`, `CONFIG3 0x0C`, `CONFIG4 0x0D`), 상태 비트 `DRV8244_ST_FAULT 0x20 … DRV8244_ST_SPI_ERR 0x01`, `uint16_t drv8244_write_frame(uint8_t addr, uint8_t data);` · `uint16_t drv8244_read_frame(uint8_t addr);` · `bool drv8244_response_valid(uint16_t sdo);` · `uint8_t drv8244_status_byte(uint16_t sdo);` · `bool drv8244_config3_valid(uint8_t config3);`(`S_MODE == 00b`) · 순서기 `typedef struct { uint8_t step; uint32_t wait_start_us, wait_us; bool done, failed; uint8_t fault_summary; } drv8244_init;` · `void drv8244_init_start(drv8244_init*, uint32_t now_us, uint32_t wake_wait_us);` · `bool drv8244_init_next(drv8244_init*, uint32_t now_us, uint16_t last_sdo, uint8_t config3, uint8_t config4, uint16_t *frame_out);`(보낼 프레임이 있으면 true; `last_sdo` 는 직전 프레임의 응답)
- [ ] **Step 1: 실패하는 시험** — 쓰기 `(0x0C, 0x40)` → `0x0C40`, 읽기 `0x01` → `0x4100`; 순서기 프레임 열(SLVSG24C 8.6.1.5 근거 주석): 대기 `wake_wait_us = 1000` 동안 없음 → `0x0809`(잠금 풀기: `SPI_IN_LOCK 01b`·`REG_LOCK 01b`) → `0x0C`+config3 → `0x0D`+config4 → `0x4C00`(CONFIG3 읽기, 다음 응답 둘째 바이트가 config3 이어야 함) → `0x4D00` → `0x0888`(`CLR_FLT`) → `0x080A`(잠금: `REG_LOCK 10b`) → `0x4100`(FAULT_SUMMARY 읽기) → `done`. 읽어 온 값이 다르면 `failed`; 어떤 응답이든 상위 2비트가 `11` 이 아니면 `failed`; `now = 0xFFFFFF00` 에서 시작해도 1000 µs 뒤 진행; `drv8244_config3_valid(0x40)` true, `(0x41)` false.
- [ ] **Step 2–4.**
- [ ] **Step 5: 커밋** — `feat(firmware): DRV8244S SPI frames and an unlock/config/read-back/clear/lock sequencer that survives an MCU-only reset`

### Task 7: 구동 코어(상태기 + 제어)

**Files:** Create `drive.h/.c`, `test_drive.c`

**Interfaces:**
- Consumes: Task 2 `link_msg`/`link_state`, Task 5·6 모듈.
- Produces:
  - `enum { DRIVE_CH_L, DRIVE_CH_R, DRIVE_CH_M, DRIVE_CH_H, DRIVE_CH_COUNT };`, 모드·거부·결함 상수는 계약 값 그대로(`DRIVE_MODE_BOOT` …, `DRIVE_REJECT_*`, `DRIVE_FAULT_*`).
  - `typedef struct { uint32_t period_us, command_timeout_ms, stop_grace_ms, driver_retry_ms, driver_wake_us, spi_timeout_us /* >= 2 * period_us */, status_poll_ms; uint16_t overrun_limit, adc_invalid_limit; wheel_enc_config wheel[2]; pi_config wheel_pi, steer_pi, lift_pi; abs_axis_config steer, lift; float steer_rate_max, lift_rate_max, max_wheel_rad_s; int8_t duty_sign[4]; bool safe_brake[4]; uint8_t drv_config3, drv_config4; float ipropi_ohm, ipropi_gain, adc_vref_v; uint32_t adc_full_scale; float overcurrent_a[4]; uint32_t overcurrent_hold_ms; } drive_config;` · `bool drive_config_valid(const drive_config*);`
  - `typedef struct { uint32_t now_us; uint16_t wheel_hw[2]; uint16_t abs_frame[2]; bool abs_fresh[2]; uint32_t adc_raw[4]; bool adc_fresh; bool nfault_low[4]; bool estop_active; } drive_inputs;`
  - `typedef struct { float duty[4]; bool drvoff[4]; } drive_outputs;`
  - SPI 는 한 번에 하나: `bool drive_spi_next(drive*, uint8_t *ch, uint16_t *frame, uint8_t *tag);`(보낼 것이 있으면 true; app 이 시작하지 못하면 같은 프레임을 다음 주기에 다시 준다) · `void drive_spi_started(drive*);`(시작됨, 응답 대기 시작) · `void drive_spi_result(drive*, uint8_t ch, uint16_t sdo, uint8_t tag);`(태그가 지금 거래와 다르면 버림) — 응답이 `spi_timeout_us` 안에 없으면 초기화 실패(BOOT 재시도) 또는 그 채널 드라이버 결함(준비 뒤 상태 읽기).
  - `typedef struct drive { /* 공개: */ uint8_t mode, reject_reason; uint16_t arm_gen, last_rx_seq; uint32_t boot_id, fault_flags; uint8_t driver_status[4]; /* 그 밖 내부 */ } drive;`
  - `bool drive_init(drive*, const drive_config*, const drive_inputs *first, uint32_t boot_id);` (`boot_id` 0 이면 false) · `void drive_begin(drive*, const drive_inputs*);`(① 센서 해석 ② 마감 판정 ③ 비상정지·결함 판정) · `void drive_handle(drive*, const link_msg*);`(④, 시각은 `drive_begin` 의 것) · `void drive_finish(drive*, drive_outputs*);`(⑤ 제어 ⑥ 출력) · `void drive_fill_state(const drive*, link_state*);`
- [ ] **Step 1: 실패하는 시험**(합성 설정 `period 1000 µs, timeout 200 ms, grace 100 ms, spi timeout 3000 µs`; 가짜 드라이버는 시험 안 작은 함수로 프레임에 맞게 응답) —
  - 부팅: 초기화 중 `BOOT`·모든 `drvoff`; `ARM` → `driver_not_ready`; 네 채널 초기화 성공 → `DISARMED`; 응답 상위 비트 `00`(VM 없음 흉내)이면 `BOOT` 유지, `driver_retry_ms` 뒤 재시도해 성공; 응답이 오지 않으면 `spi_timeout_us` 뒤 실패·재시도.
  - 무장: `ARM(boot_id, echo = 최근 STATE 시각)` → `ARMED`, `arm_gen` 1; 조향 측정 0.1 rad 에서 `ARM` 만 받고 1 s 지나도 조향 목표 = 0.1, 바퀴 듀티 0; 첫 `DRIVE` 바퀴 1 rad/s → `zero_first`; 0 명령 뒤 1 rad/s 받아들임. 조향 센서가 무효이거나 범위 밖이거나 `adc_fresh` 가 `adc_invalid_limit` 번 연속 거짓이면 `ARM` → `sensor_invalid`. 거부 우선순위(계약 3.2)는 표로 시험: 각 모드에서 `boot_id` 를 틀리게 주면 언제나 `boot_mismatch`, 맞게 주면 `BOOT`→`driver_not_ready`, `STOPPING`→`stopping`, `ARMED`→`not_disarmed`, `FAULT`→`latched`.
  - 신선도와 마감: `boot_id`·`arm_gen` 다름 → 각 거부; echo 가 미래 → `stale_echo`; echo 가 250 ms 전·`valid_after_echo_ms = 300` → 마감 = echo + 200 ms(timeout) 이 이미 지나 `stale_echo`; echo 가 30 ms 전·`valid_after_echo_ms = 50` → 받아들이고 마감 = echo + 50 ms(받은 시각 + 20 ms); 재부팅 흉내(`drive_init` 새 boot_id) 뒤 예전 `ARM → 0 → 주행` 묶음 → 셋 다 거부.
  - 만료: 마감 시각과 같은 주기는 유효, 다음 주기 `STOPPING`; 마감이 지난 주기에 새 `DRIVE` 가 같이 도착해도 `STOPPING`(`stopping` 거부); `STOPPING` 동안 **출력이 켜져 있음**(`drvoff` 모두 false, 바퀴 측정이 0 이 아니면 듀티가 감속 방향), 100 ms 뒤 `DISARMED`·정지 출력; `STOPPING` 중 비상정지 → 같은 주기 `ESTOP`; `now` 가 `0xFFFFFFFF − 50000` 근처에서 시작해도 같은 결과.
  - `seq`: 무장 뒤 첫 `DRIVE(seq 100)` 받아들임, 다시 `seq 100` → `stale_seq`, `seq 101` 받아들임, `0xFFFF` 다음 `0` 받아들임.
  - `DISARM` 은 `ARMED`·`STOPPING` 에서 즉시 `DISARMED`, 다른 모드에선 변화 없음.
  - 비상정지: 같은 주기에 `ESTOP`, 모든 듀티 0, `safe_brake[H] = true` 인 H 만 `drvoff == false`(전기 제동), 나머지는 `drvoff == true`; 풀려도 `ESTOP`; `ARM` → `latched`; 입력이 남은 채 `CLEAR_FAULTS` → 변화 없음(`estop_active`); 풀린 뒤 `CLEAR_FAULTS` → `BOOT`(모든 `drvoff` true, 재초기화) → `DISARMED`.
  - 드라이버 결함: 준비 뒤 `nfault_low[M]` → `FAULT`, `driver_fault_m`, M 은 `safe_brake` 와 무관하게 `drvoff`, 다른 채널은 각자 정지 출력; 상태 읽기 응답 상위 비트 `00` → 그 채널 드라이버 결함. nFAULT 가 low 인 채 `CLEAR_FAULTS` 를 받아들여 `BOOT` 로 가고, 가짜 드라이버가 `CLR_FLT` 뒤 nFAULT 를 high 로 돌리면 `DISARMED`, 계속 low 면 `FAULT`(`driver_not_ready`).
  - 과전류: `ARMED` 에서 `overcurrent_a + 0.1 A` 상당 ADC 값이 `overcurrent_hold_ms` 동안 → `FAULT`; 1 ms 짧으면 아님; **IPROPI 가 클램프 전압에 붙은 채(가짜 고압측 OCP)** `CLEAR_FAULTS` 를 받아들여 `BOOT` 로 가고, `DISARMED` 에서는 그 값으로 과전류를 판정하지 않음.
  - 전류 측정: `ARMED` 에서 `adc_fresh` 거짓이 `adc_invalid_limit` 번 연속 → `FAULT`(`current_sense_invalid`).
  - 조향 센서: `abs_fresh = false` 를 `invalid_limit` 번 → `ARMED` 면 `FAULT`(`steer_sensor_invalid`), `DISARMED` 면 비트만; 범위 밖 측정 → `steer_out_of_range` 로 `FAULT`.
  - 동시 원인: 같은 주기에 마감 경과·비상정지·`nfault_low[L]` → 모드 `ESTOP`, 세 비트 모두 기록, L 은 `drvoff`.
  - 시계 넘어감 재생(②3회차 반례): HAL 32비트 시계가 넘어간 뒤 2³² µs 전의 echo 를 담은 `DRIVE` → `stale_echo`.
  - SPI: 시작이 계속 거절되면 `spi_timeout_us` 뒤 초기화 실패·재시도; 응답이 없어도 같음; `spi_timeout_us < 2 × period_us` 설정은 거부.
  - 무장만 하고 `DRIVE` 가 없으면 `echo + command_timeout_ms` 뒤 `STOPPING`.
  - 설정 검증: `drive_config_valid` 가 `drv_config3 = 0x41`, `duty_sign = 0`, 절단점 위반 한계, `command_timeout_ms = 65536`, `adc_invalid_limit = 0` 을 거부.
  - 제어 방향: 바퀴 목표 > 측정 → 듀티 > 0, `duty_sign[L] = −1` 이면 부호 반대; 조향 목표가 한계 밖이면 잘리고 `limited`; 조향 목표가 0 → 1 rad 로 바뀌면 내부 목표는 `steer_rate_max × dt` 씩만 움직임.
  - `drive_fill_state`: 계약 필드(반올림·포화한 `current_ma`, `duty_e4`, 무효 위치 NaN, `flags`).
- [ ] **Step 2–4.**
- [ ] **Step 5: 커밋** — `feat(firmware): drive core -- fresh-only arming and driving (boot id, arm generation, echo-based deadline), expiry before messages, STOPPING with outputs on, latched faults cleared through driver re-init, per-channel brake-or-cut`

### Task 8: app 과 가짜 HAL 루프

**Files:** Create `app.h/.c`, `test_app.c`

**Interfaces:**
- Produces: `typedef struct { void *ctx; uint32_t (*now_us)(void*); void (*read_inputs)(void*, drive_inputs*); void (*write_outputs)(void*, const drive_outputs*); size_t (*link_read)(void*, uint8_t*, size_t); bool (*link_write_frame)(void*, const uint8_t*, size_t); bool (*drv_spi_start)(void*, uint8_t ch, uint16_t tx, uint8_t tag); bool (*drv_spi_take)(void*, uint8_t *ch, uint16_t *rx, uint8_t *tag); void (*watchdog_kick)(void*); } fw_hal;` — 모두 비차단, `link_write_frame` 은 전부 넣거나 거절(설계 3절). · `typedef struct { ... } fw_app;` · `bool fw_app_init(fw_app*, const fw_hal*, const drive_config*, uint32_t state_period_ms, uint32_t boot_id);` · `void fw_app_tick(fw_app*);` — 순서: SPI 응답 수거(`drv_spi_take` → `drive_spi_result`) → 입력 → `drive_begin` → 받은 메시지 → `drive_finish` → `write_outputs` → 다음 SPI 프레임 시작 → `STATE` → `watchdog_kick`.
- [ ] **Step 1: 실패하는 시험** — 가짜 HAL(바퀴 = 1차 지연 `ω' = (k·duty − ω)/τ`, 합성 값; 가짜 DRV8244 는 프레임에 맞는 응답): ① 초기화 SPI 열을 거쳐 `DISARMED`, 가짜가 기록한 순서에서 **`CLR_FLT` 프레임 전에 그 채널 `drvoff == true` 가 쓰여 있음**; ② `drv_spi_start` 가 두 번 거절해도 같은 프레임이 세 번째에 나감, 거절된 프레임이 사라지거나 두 번 나가지 않음; ③ 호스트 흉내가 `STATE` 를 읽어 echo 를 채운 `ARM`·0 명령·2 rad/s 명령을 50 ms 마다 새로 보내면 1 s 뒤 바퀴 속도가 2 rad/s ±10 %; ④ 명령을 끊으면 `200 + 100` ms 안에 `DISARMED`·모든 채널 `drvoff`(`safe_brake` 거짓 설정); ⑤ `state_period_ms = 10` → 1 s 에 `STATE` 100 개, `link_write_frame` 이 하나 걸러 거절하면 받는 쪽 `link_rx` 가 오류 0 으로 50 개를 받음; ⑥ 쓰레기 바이트 → `rx_errors` 증가; ⑦ `now_us` 가 한 번 3 ms 건너뛰면 `cycle_overruns` +1, `ARMED` 중 `overrun_limit` 번 연속이면 `FAULT`; ⑧ `watchdog_kick` 은 주기마다 정확히 한 번.
- [ ] **Step 2–4.**
- [ ] **Step 5: 커밋** — `feat(firmware): fw_app tick over a non-blocking HAL table; fake-HAL closed-loop tests (arming with echo, expiry stop, whole-frame STATE drops, SPI retry, overruns, watchdog)`

### Task 9: 문서 정리와 마무리 검증

**Files:** Modify `CONTRIBUTING.md`(1절 트리의 `firmware/` 설명: "하드웨어 독립 코어는 보드 확정 전부터, 보드 HAL 은 보드 확정 뒤"), `docs/hardware.md`(MCU 행: STM32H743VIT6 유력·미구매, 핀맵 검토·펌웨어 설계 링크), `README.md`(펌웨어 시험 명령 한 줄), `firmware/README.md` 완성.
- [ ] **Step 1:** 문서 수정.
- [ ] **Step 2:** `ruff check`, `ruff format --check`, `clang-format --dry-run --Werror` (자체 C 파일 전부).
- [ ] **Step 3:** ws1 `~/bin/forklift-ws1-pytest`(새 파일 `git add` 뒤) — 알려진 실패 외 새 실패 없음.
- [ ] **Step 4:** 커밋 `docs: firmware core status in CONTRIBUTING, hardware and README`.

## 자체 검토

계획 단계 독립 검토 3회(서로의 결론을 보지 않음)와 처리.

**① 원자료 재계산(Claude, 2026-10-22).** 표준 라이브러리로 다시 계산: CRC-16/CCITT-FALSE 검사값 0x29B1, CRC-4/G-704 검사값 0x7, KTH 예 0x0FF → 2 를 두 비트 순서 해석이 모두 만족(그래서 설정), COBS 표준 예, 형식 문자열 크기(최종 33·12·4·71), 계약 6절 고정 벡터, `uint32_t` µs 넘어감 71.6 분, 속도 분해능 0.0767 rad/s(20 ms 창), IPROPI 3.6 kΩ 포화 4.35 A. 반영: COBS 254 바이트 블록 끝에 0x01 을 덧붙이지 않음을 명시(단순 구현은 256 바이트), DRV8244 `COMMAND` 프레임 값(`CLR_FLT` 0x0888, 잠금 0x080A — `SPI_IN_LOCK` 리셋값 01b 유지), SPI 응답 상태 바이트 배치가 `FAULT_SUMMARY` 레지스터와 다름을 명시.

**② Codex 적대 검토(새 스레드).** P1 3·P2 6·P3 1. 모두 근거 확인 후 반영:
- P1 받은 시각 기준 만료는 늦게 온 명령·재부팅 전 명령 묶음을 살린다 → 부팅 ID·무장 세대·MCU 시각 되돌려받기(계약 2절), 만료 = `min(rx + valid, echo + timeout)`.
- P1 메시지 처리가 만료 판정보다 먼저면 늦은 `DRIVE` 가 만료를 덮는다 → 주기 순서에서 만료 판정을 먼저, `STOPPING` 모드에서 `DRIVE`·`ARM` 거부.
- P1 전 축 DRVOFF 는 승강 하중 유지가 아니다 → 채널별 `safe_brake`(전기 제동은 정지 하중 유지 아님을 명시), 승강 자잠김·브레이크를 실물 확인 전 빈 포크 운용 전제로 설계 4절에 명시.
- P2 `ARM` 계약·시험 불일치, 무장 직후 조향·승강 목표 미정 → `DISARMED` 에서만 무장, 목표 = 측정값, 변화율 제한, "ARM 만 받은 뒤" 시험.
- P2 래치형 드라이버 결함 해제의 순환 의존 → `CLEAR_FAULTS` 는 드라이버 외 원인만 보고 `BOOT` 재초기화 결과로 판정, `CLR_FLT` 는 DRVOFF high 에서.
- P2 MCU 만 재부팅하면 드라이버 설정이 잠겨 있다 → 잠금 풀기·읽어 확인 단계, `S_MODE = 00b` 검증.
- P2 절대각 절단점·범위 밖 사유 → 설정 검증으로 절단점 강제, `*_out_of_range` 결함 비트.
- P2 HAL 의 신선도·비차단·실행시간·워치독 계약 없음 → 설계 3절(표본별 fresh, 비차단, 주기 지연 계수, 독립 워치독).
- P2 수치 불변조건 → 속도 주기를 `period_us` 하나로, 표본 시각 기반 속도, 32비트 모듈러 누적, 시간 값 2³¹ µs 미만.
- P3 `link_tool` 인자 수(28 값)와 Task 8 명령 갱신 주기 → 고침. 같은 배치 오류를 C·Python 이 함께 통과하는 문제 → 계약 6절 독립 고정 벡터(모든 필드가 다른 값).

**③ agy 독립 검토(Gemini 3.8 Flash — `agy-quota-ok --pick` 결과, Argon 은 한도 보류; ①·② 결론 미공개, "적히지 않은 전제" 질문).** 반영: 승강 자중 낙하 전제(②와 같은 결론, 설계 4절), 전송 큐 지연 전제(신선도 설계로 해결), 1 ms 안 블로킹 I/O 전제(HAL 비차단 계약, 계약 5절 전송량 — 115,200 bps 면 `STATE` 가 링크의 약 62 %), VM 이 MCU 보다 늦게 들어오는 경우(`BOOT` 재시도), 실행 중 드라이버 재초기화(`CLEAR_FAULTS → BOOT`), 호스트 시험의 이상적 표본 전제(설계 8절 한계). 반영 안 함: 링크로 설정을 넣는 `SET_CONFIG` — 잘못된 게인이 링크로 들어오는 경로를 만들지 않으려고 빌드에 넣는 쪽을 택하고 그 이유를 설계 7절에 남김.

**② Codex 2회차(같은 스레드).** P1 1·P2 7·P3 1. 근거 확인 후 반영:
- P1 받은 시각 + `valid_for_ms` 는 여전히 전송 지연만큼 늘어난다(예: 남은 50 ms 명령이 60 ms 늦게 오면 60 ms 초과) → 필드를 `valid_after_echo_ms` 로 바꾸고 마감 = `echo + min(valid_after_echo_ms, command_timeout_ms)`(언제나 보수적), 받을 때 마감이 지났으면 거부, `max_echo_age_ms` 설정 삭제.
- P2 `STOPPING` 이 "ARMED 아닌 상태"의 정지 출력에 걸림 → `STOPPING` 은 출력을 켠 채 제어, 시험이 실제 듀티·DRVOFF 를 검사, 동시 원인 시험.
- P2 부분 송신 뒤 `STATE` 폐기는 프레임을 깨뜨림 → HAL `link_write_frame` 전부/거절.
- P2 세대 값 재사용 → `boot_id` u32, `arm_gen` u16, 남는 위험(세 조건이 모두 겹쳐야 함)을 계약 2절에 명시.
- P2 무효 센서로 무장 가능 → `ARM` 에 센서 유효·범위 조건, `sensor_invalid`.
- P2 ADC 신선도 실패 처리 없음 → `adc_invalid_limit`, `current_sense_invalid` 결함·무장 거부.
- P2 비차단 SPI 시작 거절·응답 유실 → 한 번에 한 거래, `drive_spi_next/started/result`, `spi_timeout_us`, 재시도 전 응답 비우기, DRVOFF 쓰기가 SPI 시작보다 먼저(시험 ①).
- P2 고압측 OCP 뒤 IPROPI 클램프가 `CLEAR_FAULTS` 를 막음 → 과전류는 출력이 켜진 `ARMED`·`STOPPING` 에서만 판정, 해제 조건에서 과전류·드라이버 결함 제외, 가짜 OCP 시험.
- P3 `ARM` 거부 순서 → 계약 3.2 에 표로 고정, 시험도 표로.

**② Codex 3회차.** P1 0·P2 3·P3 1. 반영:
- P2 32비트 µs 시계는 71.6 분마다 돌아와, 같은 부팅·무장 안에서도 그만큼 지난 `DRIVE` 가 echo 나이와 `seq` 검사를 함께 통과할 수 있다(반례: echo 100000·seq 100 을 2³² + 10 ms 뒤 재생, 현재 seq 40100) → `mcu_time_us`·echo 를 **u64** 로 넓힘(코어가 HAL 의 32비트 시계를 64비트로 넓힘), 페이로드 33·12·71, 고정 벡터 재생성, 반례를 Task 7 시험에 넣음.
- P2 SPI 제한 시간과 응답 수거 순서가 맞물리지 않고, 시작 거절이 계속되면 기한이 없다 → 제한 시간은 유실 감지용으로 정의, app 이 주기 머리에서 응답을 먼저 수거, `spi_timeout_us ≥ 2 × period_us` 설정 검증, 시작 대기에도 같은 기한(Task 7·8 시험).
- P2 `valid_after_echo_ms` 의 1 ms 올림 보정이 마감을 늘림 → 내림 결과가 1 ms 미만이면 `ValueError`(구현은 처음부터 이렇게 했고 계획 문구만 고침), 900 µs 경계 시험.
- P3 `FaultFlag` 범위 0–16.
- 함께 고친 것: `ARM` 도 `echo + command_timeout_ms` 마감을 건다(무장 뒤 `DRIVE` 가 없으면 `STOPPING` → `DISARMED`).

**② Codex 4회차 — 수렴(새 P1/P2 없음).** P3 문구 세 곳(시각 형 구분, echo 상한 2⁶⁴−1, 시험 설정 SPI 제한 3000 µs) 반영. 판정 범위는 문서 설계이며 구현·실기 검증은 별도다.
