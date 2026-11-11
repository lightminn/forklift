# firmware

하위 제어 MCU 펌웨어. 하드웨어와 무관한 C 코어(`core/`)와 호스트 시험, 그리고 WeAct H743 보드 빌드(`boards/weact_h743/`, B1 골격)가 있다. 보드 HAL·링크는 B2–B4 에서 더한다.

- 설계: [`docs/design/2026-10-22-mcu-firmware-core.md`](../docs/design/2026-10-22-mcu-firmware-core.md)
- Jetson ↔ MCU 링크 계약: [`docs/interfaces/2026-10-22-mcu-link-contract.md`](../docs/interfaces/2026-10-22-mcu-link-contract.md)
- 핀맵 초안 검토: [`docs/design/2026-10-22-h743-pinmap-draft-review.md`](../docs/design/2026-10-22-h743-pinmap-draft-review.md)

## 구성

| 파일(`core/`) | 역할 |
|---|---|
| `crc`, `cobs`, `link` | mcu-link/1 프레임(COBS + CRC-16)과 메시지, 재동기 수신기. KTH7801 CRC-4 두 비트 순서 |
| `wheel_enc` | 16비트 하드웨어 카운터 → 32비트 모듈러 누적, 표본 시각 기반 각속도 |
| `abs_axis` | KTH7801 프레임 → 관절 위치, CRC·신선도 연속 실패로 무효, 범위 이탈 |
| `pi` | 조건부 적분 PI, 변화율 제한 |
| `drv8244` | DRV8244S SPI 프레임과 잠금 해제·설정·읽어 확인·CLR_FLT·잠금 순서기 |
| `drive` | 안전 상태기(BOOT·DISARMED·ARMED·STOPPING·FAULT·ESTOP), 신선도·만료, 결함 래치, 채널별 정지 출력, 제어 |
| `app` | 한 주기 연결: HAL 함수 표(`fw_hal`)를 받아 입력·메시지·출력·SPI·STATE·워치독 순서로 부름 |
| `tools/link_tool.c` | 호스트 전용 인코드/디코드(C↔Python 교차 시험용) |

차량 값 가운데 바퀴 한 바퀴당 카운트(모터 축 엔코더 × 감속비)·바퀴 속도 상한·조향 기구각 범위는 손으로 적지 않는다. 보드 빌드가 `tools/real_vehicle_header.py --config config/real/vehicle.yaml --out <build>/vehicle_config.h` 로 헤더를 만들어 쓴다. 헤더의 `FW_VEHICLE_CALIBRATION_ID` 는 호스트 `forklift_core.plant.real_vehicle` 설정의 `calibration_id` 와 같아야 한다(실차 이식 계획 P0-6).

보드 HAL 이 지켜야 할 것(비차단, 프레임 단위 송신, 표본별 신선도, LPTIM 두 번 읽기, 독립 워치독)은 설계 문서 3절이다. 실제 하드웨어 값(극성·게인·영점·R_IPROPI·`safe_brake` 등)은 모두 `drive_config` 로 받고 기본값이 없다. 시험의 설정은 합성 값이다.

## WeAct H743 보드 빌드 (B1 골격)

설계: [`docs/design/2026-11-12-weact-h743-board-firmware.md`](../docs/design/2026-11-12-weact-h743-board-firmware.md). Arm GNU Toolchain **13.3.rel1**(공식 tarball)이 필요하다. CMake 가 버전을 검사한다. 외부 헤더(CMSIS Core·CMSIS 장치 H7)는 `third_party/` 에 고정 태그로 들어 있고, `python tools/fetch_firmware_deps.py --check` 가 `third_party/MANIFEST.yaml` 의 해시와 대조한다.

```bash
cmake -S firmware -B build/mcu -DFW_BOARD=weact_h743 \
      -DARM_GNU_TOOLCHAIN_DIR=<툴체인 폴더> \
      -DCMAKE_TOOLCHAIN_FILE=$PWD/firmware/boards/weact_h743/arm-gnu-13.3.cmake   # 절대 경로
cmake --build build/mcu -j
# build/mcu/boards/weact_h743/fw.elf · fw.bin · fw.map · fw.size.txt
```

- 출력은 잠겨 있다(`FW_MOTOR_OUTPUT=0` 만 허용). TIM1 은 모든 보드 소스에서 poison 된다. 출력을 여는 것은 IPROPI 보호 확인 뒤 사용자 결정이고 별도 PR 이다.
- B1 은 부팅·클록·시간 기준·IWDG·안전 핀·리셋 원인·RNG `boot_id`·LED 까지다. 주기(TIM6 1 kHz)는 IWDG 갱신과 LED 심박만 한다. 코어는 ARM 으로 같이 컴파일만 한다(B2 에서 연결).
- 플래시는 USB-C ROM DFU 만 쓴다(사용자 결정 2026-11-12, WeAct 를 메인보드에서 뺀 상태): `STM32_Programmer_CLI -c port=usb1 -w fw.elf -v`.
- pytest `tests/integration/test_firmware_cross.py` 가 크로스 빌드·크기·빌드 표시·TIM1 잠금(음성 대조 포함)·스택 상한을 본다(툴체인이 없으면 skip).

## 호스트 빌드·시험

CMake ≥ 3.16 과 C11 컴파일러가 필요하다.

```bash
cmake -S firmware -B build/firmware
cmake --build build/firmware -j2
ctest --test-dir build/firmware --output-on-failure
```

pytest 의 `tests/integration/test_firmware_host.py` 가 같은 빌드를 임시 폴더에서 하고 C 시험을 돌린다(도구가 없으면 skip).

자체 C 코드는 `firmware/.clang-format` 으로 맞춘다: `clang-format --dry-run --Werror` 로 확인한다.
