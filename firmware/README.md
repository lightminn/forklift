# firmware

하위 제어 MCU 펌웨어. 지금은 **하드웨어와 무관한 C 코어와 호스트 시험만** 있다. 보드 HAL·CubeMX 프로젝트·MCU 빌드는 보드와 핀맵이 확정된 뒤 `boards/<board>/` 에 추가한다.

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

보드 HAL 이 지켜야 할 것(비차단, 프레임 단위 송신, 표본별 신선도, LPTIM 두 번 읽기, 독립 워치독)은 설계 문서 3절이다. 실제 하드웨어 값(극성·게인·영점·R_IPROPI·`safe_brake` 등)은 모두 `drive_config` 로 받고 기본값이 없다. 시험의 설정은 합성 값이다.

## 호스트 빌드·시험

CMake ≥ 3.16 과 C11 컴파일러가 필요하다.

```bash
cmake -S firmware -B build/firmware
cmake --build build/firmware -j2
ctest --test-dir build/firmware --output-on-failure
```

pytest 의 `tests/integration/test_firmware_host.py` 가 같은 빌드를 임시 폴더에서 하고 C 시험을 돌린다(도구가 없으면 skip).

자체 C 코드는 `firmware/.clang-format` 으로 맞춘다: `clang-format --dry-run --Werror` 로 확인한다.
