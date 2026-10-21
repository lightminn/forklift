# firmware

하위 제어 MCU 펌웨어. 지금은 **하드웨어와 무관한 C 코어와 호스트 시험만** 있다. 보드 HAL·CubeMX 프로젝트·MCU 빌드는 보드와 핀맵이 확정된 뒤 `boards/<board>/` 에 추가한다.

- 설계: [`docs/design/2026-10-22-mcu-firmware-core.md`](../docs/design/2026-10-22-mcu-firmware-core.md)
- Jetson ↔ MCU 링크 계약: [`docs/interfaces/2026-10-22-mcu-link-contract.md`](../docs/interfaces/2026-10-22-mcu-link-contract.md)
- 핀맵 초안 검토: [`docs/design/2026-10-22-h743-pinmap-draft-review.md`](../docs/design/2026-10-22-h743-pinmap-draft-review.md)

## 호스트 빌드·시험

CMake ≥ 3.16 과 C11 컴파일러가 필요하다.

```bash
cmake -S firmware -B build/firmware
cmake --build build/firmware -j2
ctest --test-dir build/firmware --output-on-failure
```

pytest 의 `tests/integration/test_firmware_host.py` 가 같은 빌드를 임시 폴더에서 하고 C 시험을 돌린다(도구가 없으면 skip).

자체 C 코드는 `firmware/.clang-format` 으로 맞춘다: `clang-format --dry-run --Werror` 로 확인한다.
