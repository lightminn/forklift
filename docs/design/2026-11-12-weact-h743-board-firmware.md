# WeAct H743 보드 펌웨어 설계 (P0-8)

작성일: 2026-11-12 · 상태: **설계 초안, 실기기 없음** · 관련: [실차 이식 계획](../plans/2026-11-11-sim-to-real-bringup.md) 3절·4절 P0-8·5절 S1–S5·7절, [코어 설계](2026-10-22-mcu-firmware-core.md) 3절(HAL 계약), [mcu-link/2 코어 설계](2026-11-11-mcu-link-v2-firmware-core.md), [링크 계약](../interfaces/2026-10-22-mcu-link-contract.md), [핀맵 검토](2026-10-22-h743-pinmap-draft-review.md)

## 1. 목적과 범위

WeAct MiniSTM32H7xx(STM32H743VIT6, 이하 WeAct) 위에서 `firmware/core/` 를 돌리는 보드 계층을 만든다. 부품(메인보드 PCB, 11-20 발송)이 오면 계획 5절 S2–S5 를 바로 할 수 있게 하는 것이 목표다. 보드 계층이 코어를 부른다: 보드가 `fw_hal` 함수 표와 `drive_config` 를 채우고, 1 kHz 주기마다 `fw_app_tick` 을 부른다.

PR 나누기(총괄 승인 2026-11-11, 계약 PR 은 먼저 끝냄):

| PR | 내용 | 하드웨어 없이 확인하는 것 |
|---|---|---|
| B1 빌드 골격 | 툴체인 파일, 외부 코드 고정 들여오기, 시작 코드·링커 스크립트, 클록, 시간 기준, IWDG, 안전 핀 초기값, 리셋 원인·`boot_id`, LED | 크로스 빌드, 맵·크기·스택 보고, 출력 잠금 링크 검사 |
| B2 주변장치 HAL | SPI1(DRV8244)·SPI2(KTH7801)·LPTIM1/2·ADC·nFAULT·DRVOFF·nSLEEP 을 `fw_hal` 로 | 위 + 순수 논리 호스트 시험 |
| B3 링크 | USB CDC(TinyUSB), UART7 대안 빌드, 프레임 단위 송신 링 | 위 |
| B4 시운전·HIL 빌드 | `FW_COMMISSION`, `FW_HIL_PLANT`(MCU 안 모형) | 위 + 모형 호스트 시험 |
| (별도) 출력 열기 | `FW_MOTOR_OUTPUT=1` 과 TIM1 PWM | 사용자 결정(IPROPI 보호 확인) 뒤에만 |

## 2. 빌드 방식 — K4 개정 (총괄 결정 2026-11-11 Q1)

계획 3.1(K4)은 CubeMX 생성 코드와 `.ioc` 커밋이었다. 무인 환경에서 재현 가능한 빌드를 위해 다음으로 바꾼다.

- **CubeMX 를 쓰지 않는다.** 초기화는 CMSIS 레지스터 정의와 LL 로 손으로 쓴다. `.ioc` 는 필요할 때 핀 대조용 참고로만 둔다(이 PR 들에는 넣지 않는다).
- **외부 코드는 고정 태그의 필요한 파일만 `firmware/third_party/` 에 복사**하고, 출처·태그·커밋·라이선스·파일별 sha256 을 `firmware/third_party/MANIFEST.yaml` 에 적는다. `tools/fetch_firmware_deps.py` 가 태그에서 다시 받아 해시를 대조한다(빌드는 네트워크를 쓰지 않는다).

| 이름 | 저장소·태그(커밋) | 라이선스 | 쓰는 것 |
|---|---|---|---|
| CMSIS Core | ARM-software/CMSIS_5 `5.9.0` (`2b7495b8`) | Apache-2.0 | `CMSIS/Core/Include/*` |
| CMSIS 장치 H7 | STMicroelectronics/cmsis-device-h7 `v1.10.7` (`8f922cdc`) | Apache-2.0 | `stm32h7xx.h`, `stm32h743xx.h`, `system_stm32h7xx.h`(시작 코드는 직접 씀) |
| STM32H7 LL | STMicroelectronics/stm32h7xx-hal-driver `v1.11.6` (`a1996eed`) | BSD-3-Clause | **들여오지 않았다**(B1): 레지스터를 CMSIS 이름으로 직접 쓴다. 원문 확인용으로만 참조 |
| TinyUSB | hathach/tinyusb `0.21.0` (`dae3f9a3`, 2026-06-30 공개) | MIT | 장치 스택·CDC 클래스·Synopsys DWC2 포트 26개 파일 + LICENSE. USB 빌드의 컴파일러 의존 목록과 MANIFEST 가 같은지 크로스 시험이 확인한다(B3) |

- 태그는 2026-11-11 `git ls-remote` 로 확인했다. STM32CubeH7 V1.13.0 패키지의 HAL 드라이버 V1.11.6·CMSIS 장치 V1.10.7 과 같은 판이다(로컬 `~/tools/stm32cube-repo` 의 릴리스 노트).

### 2.1 툴체인

- Arm GNU Toolchain **13.3.rel1** 공식 tarball(총괄 결정 Q5). 설치 위치는 환경 기록(wiki)에 두고, CMake 는 `-DARM_GNU_TOOLCHAIN_DIR=...` 로 받는다(경로를 저장소에 적지 않음).
- `firmware/boards/weact_h743/arm-gnu-13.3.cmake`: `arm-none-eabi-gcc --version` 첫 줄에 `13.3.Rel1` 이 없으면 구성 단계에서 멈춘다. 버전 문자열은 빌드 정보 헤더와 맵 옆 `build-info.json` 에 남긴다.
- 플래그: `-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard -ffunction-sections -fdata-sections -fstack-usage -Wdouble-promotion` + 코어와 같은 경고(`-Wall -Wextra -Werror -Wconversion -Wshadow`), 링크 `--gc-sections -Map --specs=nano.specs`. 코어 소스는 호스트 시험과 같은 파일을 그대로 링크한다(코어에 double 승격 없음, 2026-11-12 확인).
- 최상위 `firmware/CMakeLists.txt`: `FW_BOARD` 가 정해지면 보드 하위 디렉터리를 더하고 호스트 시험은 만들지 않는다. 호스트 빌드는 지금과 같다.

### 2.2 빌드 선택(모두 컴파일 시점, 런타임에 못 바꿈)

| 옵션 | 값 | 기본 | 뜻 |
|---|---|---|---|
| `FW_BUILD` | `normal`·`commission`·`hil` | `normal` | 코어 설정 `commission_build`·`hil_plant` |
| `FW_MOTOR_OUTPUT` | `0`·`1` | **`0`** | 0 이면 출력 잠금(3.4). 1 은 `FW_IPROPI_PROTECTION_RECORD=<검증 문서 경로>` 없이는 구성 단계에서 멈춤. `hil` 은 늘 0 |
| `FW_LINK` | `usb`·`uart7` | `usb` | 링크 전송로(5절) |
| `FW_ESTOP_PIN` | `none`·핀 이름 | `none` | 코어 설정 `estop_wired`. 회로도에 넷이 없어 지금은 `none`(7절) |

## 3. 보드 사실과 초기화

### 3.1 보드 사실(출처)

- HSE 25 MHz 크리스털(WeAct 회로도 X1 "25Mhz"), LSE 있음(X2), 파란 LED PE3(트랜지스터 경유, 메인보드 `BOARD_LED` 넷과 같은 핀), 키 PC13, BOOT0 버튼, VREF+ = VDDA(R9 0 Ω, VREFBUF 는 외부 모드 그대로 — 핀맵 B9).
- USB-C VBUS 는 쇼트키 D10(B5819WS)을 거쳐 보드 5 V 로 간다(WeAct 회로도 V12). 그 5 V 는 헤더(U2·U4 의 5V 핀)로 메인보드 5 V 와 이어진다 → **USB 만 꽂아도 메인보드 5 V 레일에 전원이 들어갈 수 있다**(계획 R2a, 질문 15. S1 에서 측정).
- 두 플래시: QSPI W25Q64(nCS PB6, 100 kΩ 풀업)와 SPI W25Q64(nCS PD6 via SB3, 100 kΩ 풀업, MISO = PB4). 펌웨어는 PB6·PD6 를 출력 high 로 고정하고 두 플래시를 쓰지 않는다. PB4 공유의 SDO 전압 문제는 사용자 결정(2026-11-11, S2 4단계 실측, 3.6 V 넘으면 플래시 제거)을 따른다.
- 메인보드 회로도 V1.0 2쪽(STM32 핀 배정)과 4.1 표의 핀을 대조했다. PA13·PA14·PA15·PC14·PC15·PD6·PE10·PE12 는 메인보드에서 미연결(X)이다.

### 3.2 리셋 직후 순서

1. **전원 공급 확정이 RAM 보다 먼저**(Codex P1): 전원 투입 직후 Run* 상태에서는 `ACTVOSRDY` 전에 RAM 을 쓰면 안 된다. 그래서 시작 코드(`startup_stm32h743.s`, 어셈블리)가 스택·RAM 을 쓰지 않고 `PWR_CR3` 를 LDO 공급으로 쓰고 `PWR_CSR1.ACTVOSRDY` 를 레지스터 계수기로 제한해 기다린다. 그다음 `.data` 복사·`.bss` 0, FPU 켬, VTOR, C 진입이다. CubeH7 V1.13.0 참조 startup 도 데이터 복사 전에 `ExitRun0Mode` 를 부른다(LDO 경로는 `system_stm32h7xx.c` 의 그 함수와 같은 순서). 제한 시간을 넘기면 멈춤 루프로 간다(핀은 떠 있고 DRV8244 내부 풀업·풀다운이 출력을 끈다).
2. **안전 핀**(클록 전, 기본 HSI 64 MHz): GPIO 클록만 켜고 DRVOFF PD9·PD10·PD13·PD14 = 출력 high, nSLEEP PB0 = 출력 low, PWM 핀 PE9·PE11·PE13·PE14 = 출력 low, IN2 PE2·PE4·PE5·PE6 = 출력 low, DRV nCS PA0–PA3 = 출력 high, 플래시 nCS PB6·PD6 = 출력 high. 리셋 동안과 이 단계 전에는 핀이 떠 있고 DRV8244 의 DRVOFF 내부 풀업·nSLEEP 내부 풀다운이 출력을 끈다(SLVSG24C 8.3.2).
3. IWDG 시작(3.5).
4. 리셋 원인 읽기(`RCC_RSR`): IWDG1 → 2, 소프트웨어 → 3, BOR·POR → 0, 핀 → 1, 그 밖 → 4 순서로 판정하고 `RMVF` 로 지운다(핀 플래그는 다른 리셋에도 함께 켜지므로 마지막에 본다).
5. 전압: VOS1 요청 → `VOSRDY` 대기(제한 시간 있음).
6. 클록(3.3), 시간 기준(3.4), 주변장치, `boot_id`(3.7), 링크. 각 단계는 제한 시간을 두고 기다리며, IWDG 갱신 간격이 **40 ms 를 넘지 않게** 나눈다(3.5). **링크는 7 의 주기 시작 뒤 주 문맥에서 켠다**(B3): TinyUSB 는 코어 리셋·엔드포인트 끄기에서 제한 없이 기다리므로, 그 기다림이 주기·만료·IWDG 를 막지 않게 한다. USB 열거는 부팅을 막지 않는다.
7. `fw_app_init(..., reset_cause)` 직후 곧바로 1 kHz 주기를 시작한다(첫 주기 지연은 코어가 차분만 버린다 — mcu-link/2 코어 설계 5절).

### 3.3 클록

| 항목 | 값 | 근거 |
|---|---|---|
| PLL1(HSE) | HSE 25 MHz / M 5 = 5 MHz(입력 범위 4–8 MHz), × N 160 = VCO 800 MHz(넓은 VCO), P /2 = **SYSCLK 400 MHz**, Q /8 = 100 MHz | VOS1 의 최대 400 MHz(rev Y·rev V 공통 — 480 MHz 는 rev V·VOS0 전용이라 쓰지 않음) |
| PLL1(HSI 대체) | HSI 64 MHz / M 10 = 6.4 MHz(입력 범위 4–8 MHz — /8 은 8 MHz 경계라 HSI 오차에서 범위를 벗어날 수 있어 피함), × N 125 = VCO 800 MHz, P /2 = 400 MHz, Q /8 = 100 MHz. HSI 경로의 주파수·시간 정확도는 HSI 공칭값 기준이다 | 두 경로가 같은 SYSCLK·PLL1Q·버스 주파수를 내야 한다 — 그래야 TIM2 의 1 MHz 와 SPI 분주가 그대로 맞는다. 두 경로의 값을 `_Static_assert` 로 같이 검사한다(Codex P2) |
| 버스 | HPRE /2 → AXI·AHB 200 MHz, APB1–4 /2 → 100 MHz, APB 타이머 200 MHz | |
| 플래시 | `LATENCY` 2, `WRHIGHFREQ` 2 | RM0433 플래시 대기 상태 표(VOS1, AXI 200 MHz) — B1 구현 때 표로 재확인 |
| SPI1·SPI2 커널 | PLL1Q 100 MHz → SPI1 /16 = 6.25 MHz(DRV8244 최대 10 MHz), SPI2 /32 = 3.125 MHz(KTH7801 최대 10 MHz, LVDS·케이블 경유라 낮게 시작, S2 에서 올림) | |
| ADC 커널 | per_ck = HSI 64 MHz, ADC 분주 /2 = 32 MHz | **판(revision)마다 다르다**(AN5312 1.7): rev Y 는 아날로그 ADC 클록 32 MHz → `BOOST` 켬, rev V 는 안에서 /2 가 더 붙어 16 MHz → `BOOST=10`(12.5–25 MHz 대역). 부팅 때 `DBGMCU_IDCODE.REV_ID`(0x1003 = Y, 0x2003 = V)로 판을 읽어 고른다. 모르는 판이면 ADC 를 켜지 않는다(전류 무효 → ARM 거부) |
| LPTIM1·LPTIM2 커널 | 기본 rcc_pclk1·rcc_pclk4 = 100 MHz | 엔코더 입력 ≤ 커널/4 = 25 MHz ≫ 약 104 kHz(계획 3.3) |
| USB | **(B3 정정) PLL3 Q = 48 MHz**: PLL1 과 같은 기준(`PLLSRC` 는 세 PLL 공통)·같은 M 에서 HSE 경로 5 MHz × 96, HSI 경로 6.4 MHz × 75 = VCO 480 MHz, Q /10. `USBSEL=10`. 링크를 켤 때(주 문맥) PLL3 를 켜고, 잠기지 않으면 링크 없이 돈다(ARM 이 오지 않음). HSI 대체 경로의 USB 클록은 HSI 오차를 그대로 가져 USB FS 허용 오차(±0.25 %)를 지키지 못한다 — **그때 USB 링크는 실패하는 것으로 본다**(agy). 링크가 없으면 ARM 이 오지 않아 안전 쪽이고, UART7 빌드는 영향이 없다 | **HSI48 + CRS 를 버린 이유**: USB FS 는 ±0.25 % 가 필요한데 HSI48 은 보정 없이 30 °C 에서 47.5–48.5 MHz, 온도 전 범위 −4.5/+3.5 %(DS12110 Rev 7 표 46). CRS 의 USB SOF 동기는 **rev Y 에서 동작하지 않는다**(ES0396 Rev 6 2.2.2, 해결책은 LSE 동기). 또 설계 초안의 판별 `SYNCSRC`(rev Y 00·rev V 11, AN5312 인용)는 RM0433 Rev 7 의 CRS_CFGR 표(00 = USB2 SOF, 10 = OTG HS1 SOF, 11 = 예약)와 맞지 않았다. 수정 결정으로 이 불일치를 쓰지 않게 됐다. 컨트롤러는 `USB2_OTG_FS`, 내장 FS PHY |
| RNG | HSI48 (`RNGSEL`) | HSI48 은 RNG 에만 쓴다(링크 종류와 무관하게 켬) |
| 실패 | HSE 가 제한 시간 안에 안 서면 위 HSI 대체 경로로 같은 주파수를 만든다(LED 로 표시. STATE 에는 이 사실을 실을 필드가 없다 — 한계로 적는다). 실행 중 HSE 를 잃으면 CSS → NMI → 출력을 안전 핀 상태로 되돌리고 멈춤 → IWDG 리셋 → 다시 시작해 HSI 경로 | 주기가 6 배 느려진 채 제어가 이어지는 일을 막는다 |

### 3.4 시간 기준과 주기

- `now_us`: TIM2(32비트) 를 PSC 199 로 1 MHz 자유 계수. 넘어감은 코어가 64비트로 넓힌다.
- 제어 주기: TIM6 갱신 인터럽트 1 kHz 안에서 `fw_app_tick` 을 부른다(우선순위 아래 표). 제어 주기를 주 루프에 두지 않는 이유: USB 스택(주 루프)이 늦어도 제어·만료·워치독은 돈다.

| 인터럽트 | 우선순위(작을수록 높음) | 하는 일 |
|---|---|---|
| NMI(CSS) | 고정 | 안전 핀 상태로 되돌리고 멈춤 |
| SPI1·SPI2 EOT | 1 | 거래 끝: 수신값·태그를 한 칸 우편함에 넣고 nCS 를 올림, KTH 는 다음 센서 거래를 이어 시작 |
| TIM6 | 2 | `fw_app_tick` |
| USB OTG | 3 | TinyUSB 인터럽트(실제 처리는 주 루프 `tud_task`) |

- **TIM6 주기 안의 순서**(Codex P2): ① 끝난 KTH·ADC 결과를 세대 번호로 수거 → ② `fw_app_tick`(그 안의 `read_inputs` 가 새 세대만 `fresh` 로 소비) → ③ 다음 주기용 KTH 묶음(조향 M → 승강 H 뒤 멈춤)과 ADC 주입 변환을 시작. KTH 와 ADC 의 시작·종료 주체는 이 주기 래퍼 하나다.
- **ISR ↔ 주기 ↔ 주 루프 공유 규칙**:
  - 공유 인덱스·`ready`·세대 번호는 정렬된 32비트 `volatile` 변수로 한 번에 읽고 쓴다(Cortex-M7 에서 원자적). 세대 비교는 넘김을 고려해 `(int32_t)(done - consumed) > 0` 으로 한다.
  - 링 버퍼는 32비트 head·tail 인덱스의 단일 생산자·단일 소비자다. 생산자는 데이터를 다 쓴 뒤 `__DMB()` 를 하고 head 를 한 번에 올린다(release). 소비자는 head 를 읽고 `__DMB()` 뒤 데이터를 읽은 다음, 다시 `__DMB()` 를 하고 tail 을 올린다(acquire). 프레임은 다 복사한 뒤에만 head 를 공개한다.
  - SPI 우편함: EOT 인터럽트가 채널·응답·태그를 다 쓴 뒤 `ready` 를 세운다(release). `drv_spi_take` 가 `ready` 를 보고(acquire) 복사한 뒤 지운다. `drv_spi_start` 는 버스가 바쁘거나 **소비되지 않은 결과가 있으면** 거절한다.
  - 호스트 단위 시험: 순차 시험에 더해, 공개·소비 중간에 다른 쪽 호출을 끼워 넣는 인터리빙 시험을 둔다.
- 주 루프: `tud_task()` 와 링크 링 버퍼 옮기기만 한다. 주 루프가 멈춰도 TIM6 주기는 돌고, 명령이 끊기면 코어가 만료로 세운다. 주기가 멈추면 IWDG 가 리셋한다.

### 3.5 IWDG

- LSI 29.76–33.60 kHz(데이터시트 DS12110 표 49, −40–105 °C). PR /4, RLR 399 → 400 × 4 / f = **47.6–53.8 ms**(공칭 50 ms, 계획 3.4 제안). 코어가 주기를 끝까지 돌았을 때만 `watchdog_kick`.
- 초기화 동안에는 단계마다 직접 갱신하고, **갱신 간격은 40 ms 이하**로 둔다(최소 만료 47.6 ms 에 여유). HSE·PLL·VOS·ADC 보정·RNG 대기는 각각 제한 시간을 둔다. 무한 대기 안에서 계속 갱신하는 방식은 쓰지 않는다.
- 디버거 정지 중에도 멈추지 않는다(`DBGMCU` 의 IWDG 정지 비트를 켜지 않음). 디버그 빌드도 같다 — SWD 로 정지시키면 약 50 ms 뒤 리셋된다.
- 하드웨어 IWDG 옵션 바이트(`IWDG1_SW`)는 바꾸지 않는다(옵션 바이트는 되돌리기 어려운 변경이라 펌웨어가 쓰지 않음). BOR 수준 옵션 바이트도 같다: 펌웨어는 읽어서 빌드 정보와 함께 부팅 로그에만 남기고, 바꾸는 절차는 S1 문서에 CubeProgrammer 명령으로 적는다(사용자 실행).

### 3.6 출력 잠금(총괄 결정 Q3 개정 조건 1, Q6)

- `FW_MOTOR_OUTPUT=0`(기본)·`hil` 빌드는 `motor_out_locked.c` 를, `=1` 은 `motor_out_tim1.c` 를 컴파일한다. 잠금 쪽 `write_outputs` 는 듀티를 버리고 DRVOFF 를 늘 high, PWM·IN2 핀을 GPIO low 로 둔다. **TIM1 을 켜는 코드가 링크되지 않는다.**
- 보장은 구조로 하고, 기계어 검색은 보조로만 쓴다(Codex 2라운드: `MOVW`/`MOVT` 로 주소를 나눠 만들면 즉시값·리터럴 검색이 놓친다):
  1. **오브젝트 선택**: 잠금 빌드는 `motor_out_tim1.c` 를 컴파일하지 않는다. 링크 맵에 그 오브젝트가 없음을 검사한다.
  2. **식별자 금지**: 잠금 빌드의 모든 보드 소스는 CMSIS 헤더 뒤에 강제 포함되는 헤더에서 필요한 CMSIS·LL 헤더를 먼저 포함한 뒤 `#undef TIM1`·`#undef TIM1_BASE` 를 하고 `#pragma GCC poison TIM1 TIM1_BASE` 를 건다(정의된 매크로를 그대로 poison 하면 정상 파일도 경고로 실패한다 — Codex 3라운드). 소스에 TIM1 을 쓰면 컴파일이 실패한다. 음성 대조로, TIM1 을 쓰는 시험 파일이 잠금 빌드에서 컴파일 실패하는지 본다.
  3. **출력 경로 한 곳**: 모터 핀(DRVOFF·EN·IN2·nSLEEP)과 TIM1 에 쓰는 코드는 `motor_out_*.c` 하나에만 둔다(소스 검토 규칙, 리뷰에서 확인).
  4. **동작 시험**: 잠금 출력 함수를 레지스터 모형(호스트)에 걸어, ARM·COMMISSION·CLEAR_FAULTS·결함 해제 어느 경로에서도 DRVOFF 가 high, EN·IN2 가 low 로 남는지 본다.
  5. **보조**: `objdump -d` 에서 TIM1 영역(0x4001_0000–0x4001_03FF) 주소 리터럴과 `MOVW`/`MOVT` 쌍을 찾아 보고한다. 이것만으로 부재를 증명했다고 하지 않는다.
- 코어 설정의 `outputs_locked` 는 같은 매크로에서 나와 `STATE.flags` bit6 으로 드러난다.
- 잠금 빌드도 nSLEEP 은 올린다(DRVOFF high 라 출력은 Hi-Z): S2 의 VM 투입 절차(사용자 결정 2026-11-11)에서 DRV SPI 를 확인해야 하기 때문이다. `hil` 빌드는 nSLEEP 을 내린 채 둔다.

### 3.7 `boot_id`, 빌드 표시, 설정

- `boot_id`: RNG(HSI48 커널)에서 `DRDY` 를 제한 시간 안에 기다려 0 이 아닌 32비트를 얻는다. **건강한 RNG 값을 얻지 못하면(클록·시드 오류 `CECS`·`SECS`, 시간 초과) 부팅 실패로 본다**: 안전 핀 상태를 유지하고 제어 주기·링크를 시작하지 않으며, LED 로 표시하고 IWDG 를 갱신하지 않아 리셋으로 다시 시도한다. UID·타이머를 섞은 결정적 값은 쓰지 않는다 — 같은 값이 다음 부팅에 다시 나오면 이전 부팅의 `ARM`·`DRIVE` 묶음이 신선도 검사를 통과할 수 있다(계약 2절 하드웨어 난수 전제, Codex P1).
- **어떤 빌드가 올라가 있는지**(agy): `git describe --always --dirty`, 빌드 종류(`FW_BUILD`·`FW_MOTOR_OUTPUT`·`FW_LINK`), 툴체인 버전을 빌드 정보 헤더로 만들어 ELF 의 한 구역과 USB 제품 문자열 끝(`forklift-mcu normal/locked <git>`)에 싣는다. Jetson 은 sysfs 로 읽는다. UART 빌드는 ELF·`build-info.json` 으로만 확인한다.
- `drive_config`: 차량 값(바퀴 카운트·속도 상한·조향 범위)은 기존 `tools/real_vehicle_header.py` 가 `config/real/vehicle.yaml` 에서 만드는 헤더를 빌드 의존성으로 쓴다(P0-6, 손으로 두 번 적지 않음). 나머지(게인·문턱·KTH CRC 변형·영점·`safe_brake`)는 보드 설정 파일 한 곳에 두고, 값마다 출처 꼬리표(회로도·카탈로그·'실측 대기')를 단다. S2·S3 실측 뒤 그 파일을 고친다. 호스트 MCU 모형의 합성 설정은 가져오지 않는다.
- **KTH 원시 0 의 위치**(agy): KTH7801 프레임 `0x0000`(각도 0) 은 CRC 를 통과한다(두 CRC 변형 모두, 2026-11-12 계산). MISO 가 low 에 붙으면 각도 0 으로 읽힌다. 실제 각도 0 도 같은 값이라 그 값만 거부할 수는 없다. 그래서 계획 S2 의 "넘김 지점이 범위 밖" 조건을 **WeAct 보드 설정의 검증**으로 강제한다: 원시 각도 0 이 바꾼 관절값이 `[min_joint, max_joint]` 밖이어야 한다(조향·승강 둘 다). 그러면 붙은 MISO 는 `out_of_range` 로 걸린다. 코어의 일반 `abs_axis_config_valid` 에는 넣지 않는다 — 코어는 센서 영점이 범위 안에 있는 구성도 허용하고(기존 시험), 호스트 MCU 모형도 그렇게 쓴다(Codex P2). B2 에 보드 설정 검사 함수와 호스트 시험으로 넣는다. 케이블을 뺐을 때의 수신기 출력은 S2 에서 실측한다.

## 4. 주변장치 → `fw_hal`

### 4.1 핀(메인보드 회로도 V1.0 2쪽, AF 는 RM0433·DS12110)

| 기능 | 핀 | 설정 |
|---|---|---|
| DRV SPI | PA5 SCK·PA7 MOSI·PB4 MISO(AF5 SPI1), nCS PA0 L·PA1 R·PA2 M·PA3 H(GPIO) | 모드 1, 16비트, 소프트웨어 NSS |
| DRVOFF | PD9 L·PD10 R·PD13 M·PD14 H | GPIO 출력, 기본 high |
| nSLEEP | PB0 `MTR_SLEEP`(공통) | GPIO 출력 |
| nFAULT | PB11 L·PB12 R·PB14 M·PB15 H | 입력 풀업 |
| PWM(EN/IN1) | PE9·PE11·PE13·PE14 (TIM1 CH1–4, AF1) | 잠금 빌드는 GPIO low |
| 방향(PH/IN2) | PE2 L·PE6 R·PE4 M·PE5 H | GPIO 출력 |
| IPROPI | PA6 INP3·PC4 INP4·PB1 INP5·PC0 INP10 (ADC1) | 4.4 |
| 바퀴 엔코더 L | PD12 IN1·PE1 IN2 (LPTIM1, AF1), Z PC6 | 4.3 |
| 바퀴 엔코더 R | PB10 IN1·PD11 IN2 (LPTIM2, AF3), Z PC7 | 4.3 |
| KTH7801 | PB13 SCK·PC1 MOSI·PC2_C MISO (SPI2, AF5), CS1 PD0 조향·CS2 PD1 승강 | 모드 3, `SYSCFG_PMCR.PC2SO` 를 지워 아날로그 스위치를 닫음(핀맵 B2) |
| 링크 | PA11/PA12 USB OTG FS 또는 PE7 RX·PE8 TX (UART7, AF7) | 5절 |
| LED | PE3 | 상태 표시 |
| 비상정지 | 없음 | 7절 |

엔코더 Z(PC6·PC7)는 쓰지 않는다(입력으로만 둠). 핀맵과 회로도 사이의 모순은 이번 대조에서 찾지 못했다.

### 4.2 DRV8244 SPI(`drv_spi_start`·`drv_spi_take`)

- 시작: SPI 가 바쁘면 거절(false). 아니면 해당 nCS low → `TXDR` 쓰기 → `CSTART`. 거래 하나는 6.25 MHz 에서 약 2.6 µs.
- 거래는 `TSIZE=1`. EOT 인터럽트: `RXDR` 읽기 → `EOTC`·`TXTFC` 로 플래그 지움 → nCS high → (채널, 응답, 태그)를 한 칸 우편함에 넣음(3.4 공유 규칙). 오류 플래그(`OVR`·`MODF`·`UDR`)면 SPI 를 멈추고 다시 켜며, 그 거래는 응답 없이 끝낸다(코어의 `spi_timeout_us` 가 판정). `drv_spi_take` 는 주기 머리에서 그것을 꺼낸다. DMA 는 쓰지 않는다(16비트 하나라 인터럽트가 더 단순하고, H7 의 DMA·D-캐시·DTCM 접근 제약을 피한다).

### 4.3 엔코더

- LPTIM 엔코더 모드: `CKSEL=0`(내부 클록), `PRESC[2:0]=000`(분주 /1, `LL_LPTIM_PRESCALER_DIV1`), `ENC=1`, `CKPOL=10`(양 에지, 4체배), `ARR=0xFFFF`, 디지털 필터 `CKFLT` 8 클록(100 MHz 에서 80 ns). `CNT` 는 두 번 연달아 읽어 같은 값만 쓴다(RM0433 43.4). 초기화는 켜기(`ENABLE`) → `ARR` 쓰기 → `ARROK` 확인 → 연속 계수 시작 순서다.
- 읽기 시각과 `now_us` 는 같은 주기 머리에서 연달아 읽는다(코어의 gap·jump 판정이 그 시각 차를 쓴다).

### 4.4 IPROPI ADC

- ADC1 주입(injected) 순서 4개(INP3·INP4·INP5·INP10), 16비트, 부팅 때 보정(`ADCAL`, 선형 포함), 채널별 표본 시간은 IPROPI 원(1 kΩ·10 nF)에 맞춰 B2 에서 정한다. 주기 끝에 소프트웨어로 시작한다(`JADSTART` 가 이미 서 있으면 다시 시작하지 않음). 다음 주기 머리에서 `JEOS` 가 서 있으면 4개를 읽고, **`JEOS` 를 1 을 써서 지우고**, 그 주기만 `adc_fresh = true` 다. `JEOS` 가 없으면 반드시 false — 변환이 멈췄는데 옛 값을 신선하다고 넘기지 않는다(Codex P2, `JEOS` 는 JDR 을 읽어도 지워지지 않는다). DMA 없음.
- PWM 중앙 동기 표본(TIM1 TRGO2)은 출력을 여는 별도 PR 에서 한다(잠금 빌드에는 TIM1 이 없다).
- `adc_full_scale` = 65535, `ipropi_ohm` = 1000(회로도 R5·R6·R15·R12), `ipropi_gain` = 4750(SLVSG24C 대표값, VQFN-HR) — 모두 '회로도·카탈로그' 출처 꼬리표.
- **과전압 위험(계획 R1a, 질문 12)**: 회로도 V1.0 에서 IPROPI 는 1 kΩ·10 nF 를 거쳐 ADC 핀에 바로 이어진다. DRV8244 IPROPI 는 고장 때 4.5–5.5 V 까지 오르고 PC4·PB1 은 절대최대 4.0 V(TT 핀)다. **그래서 출력 잠금이 기본이고, 출력을 여는 것은 보호 확인 뒤의 사용자 결정이다.**

### 4.5 `read_inputs` 스냅숏

LPTIM 두 개(두 번 읽기), KTH 두 프레임과 그 신선도(축별 완료 세대가 소비 세대보다 크면 true, 소비하면 같게 맞춤 — 같은 결과가 두 번 신선하지 않게), ADC 4개와 신선도, nFAULT 4개, 비상정지(`none` 이면 false). 인터럽트가 쓰는 값은 단일 생산자·단일 소비자 칸으로 넘기고, 주기 함수는 읽기만 한다.

### 4.6 KTH7801 거래 이어 붙이기

- SPI2 도 `TSIZE=1`. 조향 거래의 EOT 인터럽트에서 결과를 넣고 CS1 을 올린 뒤, **DWT 계수기로 1 µs 이상 기다렸다가** CS2 를 내리고 승강 거래를 시작한다. 승강 EOT 뒤에는 멈춘다.
  - 이유: ES0392 2.22.2 — EOT 뒤 SCK 한 주기 안에 `CSTART` 를 다시 세우면 거래가 멈출 수 있다. 3.125 MHz 에서 한 주기는 320 ns 다.
  - KTH7801 의 CS high 최소 시간은 데이터시트로 확인해 그보다 길게 둔다(실물·데이터시트 확인 필요).
  - **(B2a 정정)** 거래마다 RM0433 50.4.12 의 표준 종료 절차(EOT → RX 읽기 → 플래그 지움 → `SPE=0`)를 따르고, 다음 거래는 `SPE=1` 로 다시 켠다. 이것이 오류 정오표의 첫째 해결책이다(ES0396 2.12.2 = ES0392 의 같은 항목: "Disable then enable SPI after each EOT event"). SPI 를 끈 동안에는 EOT 인터럽트를 꺼 둔다(ES0396 2.12.4). 1라운드 검토가 든 "2.22.6 SPE 토글 출력 끊김" 은 원문(ES0396 Rev 6, 같은 실리콘)에서 찾지 못해 근거로 쓰지 않는다. 1 µs 간격은 CS high 시간을 위해 그대로 둔다.
- 두 번째 거래가 주기 안에 끝나지 않으면(오류·멈춤) 그 축은 그 주기에 신선하지 않다. 코어의 `invalid_limit` 이 연속 실패를 센다.

## 5. 링크

- **USB CDC(기본)**: TinyUSB, OTG FS(PA11/PA12), VBUS 감지 끔, 제품 문자열 `forklift-mcu`, 일련번호 = MCU UID 96비트(16진). VID/PID 는 ST 의 VCP 쌍(0x0483/0x5740)을 쓰고, Jetson 쪽은 VID/PID 가 아니라 **일련번호와 제품 문자열**로 장치를 고른다(udev 규칙, P0-7 어댑터) — ROM DFU 장치(0483:df11)를 링크로 오인하지 않게.
- **UART7 대안**(`FW_LINK=uart7`): PE7 RX·PE8 TX, 460,800 bps 8N1(계약 5절: 115,200 은 STATE 만으로 69 %). 메인보드에서 미연결 핀이라 Jetson 과의 배선은 따로 한다.
- 재연결: 호스트가 포트를 다시 열면 남은 옛 `STATE` 가 한꺼번에 나가지 않게 한다(구현은 5.1 — 닫힌 동안 링을 버리고, 다시 열 때 구분자를 먼저 보냄). 호스트는 echo 신선도로 옛 프레임을 걸러 내지만, 쌓인 프레임이 한꺼번에 나가지 않게 한다. Jetson udev 규칙에 `ID_MM_DEVICE_IGNORE=1`(ModemManager 가 AT 명령을 보내지 않게)을 넣는 것은 호스트 쪽(P0-7 어댑터) 몫으로 넘긴다(agy).
- 링 버퍼: 수신은 주 루프(생산)→주기(소비), 송신은 주기(생산)→주 루프(소비). `link_write_frame` 은 프레임 전체가 들어갈 자리가 있을 때만 넣는다(코어 설계 3절). 크기는 수신 512 B·송신 1024 B(STATE 80 B × 10 주기 여유).

### 5.1 B3 구현

- **문맥**: 주기는 링만 만진다. USB 빌드는 OTG 인터럽트(우선순위 3)가 사건만 쌓고 `tud_task`·수신 복사·송신 펌프가 주 루프에서 돈다. 주 루프는 `link_poll` 뒤 `WFI` 로 자며, 놓친 일은 길어야 한 주기(TIM6 1 ms 깨움) 늦는다. UART7 빌드는 두 방향을 UART7 인터럽트(우선순위 1, 주기 위)에서 옮긴다: 16바이트 수신 FIFO 가 460,800 bps 에서 350 µs 만 버티므로 주기 아래에 둘 수 없다.
- **링 초기값**: 링은 정적 초기화로 리셋 때부터 유효하다. 링크가 켜지기 전 주기가 `STATE` 를 넣으면 USB 는 링에 쌓였다가 연결 전이라 버려지고, UART7 은 준비 플래그 전이라 거부된다(코어가 누락을 센다).
- **옛 프레임**(5절 재연결의 구현): 호스트가 포트를 열지 않은 동안(`tud_cdc_connected()` 거짓 — DTR 낮음·suspend·미구성) 송신 펌프는 프레임을 오는 대로 버리고 "재동기 필요"를 켠다. mount·umount·suspend·resume·line-state 콜백도 링을 버리고 같은 표시를 켠다(DTR 을 쥔 채 resume 하면 line-state 콜백이 없고, 두 poll 사이 리셋·재열거는 poll 이 닫힘을 못 볼 수 있다 — Codex B3 2차). 다시 열린 것을 보면 **구분자 0x00 을 먼저 쓰고**, 그것이 들어가기 전에는 어떤 프레임도 보내지 않는다. **TinyUSB FIFO 는 비우지 않는다**: DWC2 slave 포트(`CFG_TUD_EDPT_DEDICATED_HWFIFO=1`)는 IN 전송을 소프트웨어 FIFO 전체 길이로 예약하고 인터럽트가 그 자리에서 읽으므로, 전송 중에 비우면 그 전송이 깨진다(Codex B3 2차, `tusb.c` `tu_edpt_stream_write_xfer`). 그래서 다시 연 호스트가 받을 수 있는 옛 바이트는 많아야 TinyUSB FIFO(`CFG_TUD_CDC_TX_BUFSIZE` 256 B)와 컨트롤러 TX FIFO 의 한 패킷(64 B, 이중 버퍼 끔)을 합한 320 B(`STATE` 약 4 개)다. 닫힘을 poll 이 못 봐도 생명주기 콜백이 링을 버리므로 링 잔량은 더해지지 않는다(Codex B3 3차 P3). 그리고 구분자가 그 꼬리를 깨진 프레임 하나로 닫아 첫 새 프레임을 지킨다(빈 프레임은 펌웨어·호스트 디코더 모두 오류가 아님). 옛 `STATE` 는 echo 신선도로 걸러진다. 연결된 채 호스트가 읽지 않으면 FIFO 와 링이 차고, 그 뒤 프레임은 `link_write_frame` 거부로 코어가 센다.
- **호스트 쪽 전제**(agy 독립 검토): 호스트는 DTR 을 올린 채 둬야 한다. Linux `cdc_acm` 은 포트를 열 때 DTR 을 올리므로(`cat`·pyserial 기본도 올라감) P0-7 어댑터가 일부러 DTR 을 내리지만 않으면 된다 — 내리면 모든 `STATE` 가 버려진다. 같은 이유로 호스트도 포트를 연 뒤 첫 명령 앞에 구분자 0x00 을 하나 보내는 것이 좋다(MCU 수신 버퍼에 지난 연결의 반쪽 프레임이 남아 있을 수 있음, P0-7 어댑터). ModemManager 의 AT 탐침은 udev `ID_MM_DEVICE_IGNORE=1` 로 막고(P0-7), 들어오더라도 수신 디코더가 CRC 로 버린다. UART7 대안 배선은 3.3 V 레벨(Jetson Orin Nano 40핀 헤더 UART)과 공통 GND 가 필요하고, 흐름 제어가 없으므로 호스트 수신 오버런 계수를 S 단계에서 본다.
- **수신**: USB 는 링이 차면 TinyUSB FIFO 에 남겨 엔드포인트가 NAK 한다(잃지 않음). UART7 은 링이 차면 바이트를 버리고 센다(COBS 구분자에서 다시 맞춤). 오버런·프레이밍·노이즈 오류는 지우고 센다. RX(PE7)는 풀업 — 선이 없을 때 잡음 바이트를 막는다.
- **UART7 송신 인터럽트 켬**: 주기가 프레임을 넣고 `TXFNFIE` 를 켜고, 인터럽트가 링이 비면 끈다. 주기의 CR1 읽기-수정-쓰기 사이에 인터럽트가 끄면 주기가 켬을 다시 써서 빈 인터럽트가 한 번 더 온다 — 해가 없다.
- **USB 전원**: `USB33DEN` 만 켜고 `USB33RDY` 는 기다리지 않는다(진단용으로 기록). LQFP100 에는 VDD33USB 핀이 없고 VDD 에 붙어 있으며(DS12110 Rev 7 개정 이력, 그림 5), ST 는 그 공급 구성에서 `USB33RDY` 가 서지 않는다고 답했다 — 기다리면 정상 전원에서도 USB 가 영영 시작되지 않는다(Codex B3 P1). USB 레귤레이터(`USBREGEN`)는 리셋값(꺼짐) 그대로.
- **USB 장치 정보**: 제품 문자열 = 빌드 정체(`forklift-mcu <종류> locked <git>`, 126자에서 자름), 일련번호 = UID 세 단어(0 번 먼저, 16진 대문자 24자), `bcdDevice` 0x0200(mcu-link/2), 인터페이스 문자열 `mcu-link`. `SystemCoreClock` 은 TinyUSB STM32 포트가 읽으므로(턴어라운드 시간) 400 MHz 고정값으로 정의한다.
- **멈춘 USB**: TinyUSB 의 제한 없는 기다림이 걸리면 링크가 조용해지고 명령 만료로 차량이 멈춘다. IWDG 는 주기가 계속 갱신하므로 리셋되지 않는다 — 전원을 다시 넣어야 링크가 돌아온다(링크 생존 감시는 이 PR 범위 밖).

## 6. 메모리·캐시

- 플래시 0x0800_0000 부터(뱅크 1). RAM: `.data`·`.bss`·스택(힙 없음) → **DTCM**(0x2000_0000, 128 KB, 대기 없음). DTCM 은 DMA1·DMA2·BDMA 가 접근하지 못하고 MDMA 와 CPU 만 닿는다 — 이 설계는 DMA 를 쓰지 않으므로 문제없다. TinyUSB 버퍼도 DTCM(OTG FS 는 FIFO 를 CPU 가 채우므로 DMA 접근 없음).
- DMA 를 쓰지 않으므로 D-캐시 일관성 문제가 없다. I-캐시만 켠다(D-캐시는 DTCM 에 해당 없음).
- 스택: 주 스택 8 KB. `-fstack-usage` 와 `-fcallgraph-info=su` 로 호출 경로별 최악값을 구하고(간접 호출은 표에 적은 대상으로), 인터럽트 중첩(우선순위 1·2·3 + 주 루프)마다 예외 프레임(FPU 지연 저장 포함 104 B, 정렬 패딩 4 B)을 더해 빌드 검사로 보고한다.

## 7. 비상정지·리밋 입력

- 회로도 V1.0 에 넷이 없다(계획 7절 1·4·16). `FW_ESTOP_PIN=none` 이 기본이다: `estop_wired=false` 이고 `STATE.flags` bit7 이 켜진다.
- 총괄 결정 Q3(개정): 출력 잠금 빌드와 `FW_COMMISSION` 은 ARM 을 받고, 출력이 열린 정상 빌드는 `estop_unwired` 로 거부한다. `FW_COMMISSION` 을 핀 없이 쓰는 조건은 계획 5절 S2–S4 대로 **바퀴를 띄운 받침대, 사람이 주 전원 차단을 손에 쥔 상태**다.
- 핀이 정해지면 후보는 PE15(헤더에 있고 비어 있음)다. 접점은 NC 권고(선이 끊기면 정지로 읽힘, 질문 16). 입력은 매 주기 읽고 디바운스 없이 코어에 넘긴다(코어가 래치).

## 8. 플래시·디버그·BOOT0

- **기본 플래시 경로는 ROM DFU**(총괄 승인): BOOT0 를 올린 채 하드웨어 리셋 → 0483:df11 열거 확인 → 열거된 번호로 `STM32_Programmer_CLI -c port=usbN -w fw.elf -v` → BOOT0 를 내리고 리셋. 하드웨어 리셋으로 ROM 에 들어가므로 사용자 펌웨어가 멈추거나 IWDG 로 계속 리셋돼도 복구된다(AN2606 의 H74x 절: ROM 부트로더는 하드웨어 IWDG 가 켜져 있으면 갱신한다. 단 BOOT_ADD·보호 수준·USB 조건이 맞아야 함). 펌웨어는 SWD 에 기대지 않는다.

### 8.1 SWD 디버그 — 보유 NUCLEO 의 ST-LINK 재사용 (2026-11-12, 총괄 배정, Codex 교차검증 2라운드 수렴)

보유 보드는 NUCLEO-G474RE(MB1367)·NUCLEO-H533RE(MB1814)뿐이고 별도 ST-LINK 는 없다(사용자 2026-11-12).

> **사용자 결정(2026-11-12): 지금은 USB-C ROM DFU 만 쓴다.** G474RE 의 SB40/SB41 개조도, STLINK-V3MINIE 구매도 하지 않는다. 보드가 도착한 뒤 디버거가 꼭 필요해지면 그때 다시 정한다. 아래는 그때를 위해 남겨 두는 **보류된 선택지**의 조사 결과다.

- **기본 배선 그대로는 쓰지 않는다.** 두 매뉴얼은 CN4(MIPI10/STDC14)를 '외부 디버거로 보드 위 MCU 를 디버그하는' 커넥터로 설명한다(UM2505 Rev 4 6.3.3·표 5, UM3121 Rev 1 7.1.3·표 5). 기본 상태에서는 T_SWDIO·T_SWCLK·T_NRST 가 보드 위 MCU 에 이어져 있어, WeAct 를 병렬로 이으면 SWD 대상이 둘이 된다(보드 위 MCU 의 디버그 포트는 리셋 중에도 응답한다).
- **개조하면 쓸 수 있다**(사용자 결정, 0 Ω 를 다시 달면 되돌릴 수 있음). SB40(T_SWDIO↔PA13)·SB41(T_SWCLK↔PA14)을 열면 보드 위 MCU 가 SWD 에서 빠진다(UM2505 Rev 4 표 12. OFF 설명의 "T_SWO" 는 오타. UM3121 Rev 1 27쪽 각주 5 도 SB40/SB41. 다만 같은 문서의 CN4 표는 SB30/SB29 라고 적어 서로 어긋나고, SB29·SB30 은 발진기 브리지이므로 따르지 않는다). JP1 은 열어 ST-LINK 를 실행 상태로 두고, Nucleo 는 자기 ST-LINK USB 로 전원을 받는다.
- **권고 보드: NUCLEO-G474RE.** 매뉴얼의 SB40/SB41 행이 분명하다. 주의할 점: G474RE 는 수업 Lab1·Lab2 보드이고 H533RE 는 다른 프로젝트 보드다. 개조해 두는 동안에는 그 보드 자신의 MCU 를 온보드 ST-LINK 로 디버그·플래시할 수 없다.
- **배선**(CN4 MIPI10 핀 번호, UM2505 표 5 → WeAct V12 회로도):

| Nucleo CN4 (MIPI10) | WeAct | 비고 |
|---|---|---|
| 2 T_SWDIO | P3.2 (PA13, 22 Ω) | |
| 4 T_SWCLK | P3.3 (PA14, 22 Ω) | |
| 3 또는 5 GND | P3.4 GND | |
| 1 3V3 | **잇지 않음** | Nucleo 자신의 3.3 V 망이다. WeAct 는 따로 전원을 받는다 |
| 10 T_NRST | **잇지 않음** | 보드 위 G474 NRST 와 묶여 있다. 리셋 상태에서 연결해야 하면 JP3 shunt 를 빼고, 도통으로 확인한 **ST-LINK 쪽 단자만** WeAct NRST(2×22 헤더)에 잇는다 |
| 6 T_SWO | 처음에는 잇지 않음 | 쓰려면 SB15 도 열어 보드 위 PB3 를 떼고 분리를 확인한 뒤 WeAct PB3 |

  CN4 는 1.27 mm 10핀이라 1.27 mm 케이블이나 변환 보드가 필요하다. 우리 펌웨어는 PA13·PA14 를 다른 용도로 쓰지 않으므로, NRST 없이 일반·hot-plug 연결로 시작한다.
- **도구**(노트북에 있음):
  - STM32CubeProgrammer v2.21.0 CLI: NRST 를 잇지 않았으면 `mode=NORMAL` 이나 `HOTPLUG` 를 쓴다. `UR` 은 NRST 가 필요하다.
  - ST OpenOCD 0.12.0+dev(CubeIDE 번들): `-f interface/stlink-dap.cfg -c "transport select dapdirect_swd" -f target/stm32h7x.cfg`. `STOP_WATCHDOG` 기본값은 0 이다.
- **IWDG 와 디버그**: IWDG 가 시작된 뒤 정지 비트 없이 halt 하면, 남은 워치독 시간 안에 리셋된다(설계 47.6–53.8 ms, 실측 아님). 디버거가 정지 비트를 켤 수 있다(OpenOCD `STOP_WATCHDOG=1`). **IWDG 를 멈추는 디버그 세션은 출력 잠금 빌드에서만** 한다.
- **개조 없는 대안**: 별도 STLINK-V3MINIE(대상 1.65–3.6 V, 대상에 전원을 주지 않음)를 산다 — 사용자 구매 결정.
- **DFU 대비 SWD 의 이점**: 정지·단계 실행, 결함 레지스터(HardFault 원인) 읽기, 변수 실시간 보기, 리셋 상태 연결. 플래시와 복구만이라면 DFU 로 충분하다.
- **실물 확인 필요**:
  - 보유 보드의 정확한 판(MB1367 C0x, MB1814 C0x).
  - SB40·SB41(SWO 를 쓰면 SB15) 위치와, 연 뒤 도통이 끊겼는지.
  - CN4 실장 여부와 방향.
  - NRST 가 필요할 때 JP3 의 ST-LINK 쪽 단자.
  - WeAct 별도 전원과 양쪽 3.3 V 실측. 전원이 없는 대상에 SWD 신호를 넣지 않는다.
  - 연결 뒤 H743 장치 ID(0x450)와 플래시 용량을 읽어 확인한다.
- **운용 중 BOOT0 가 우연히 걸리는 경우**: BOOT0 은 리셋 순간에만 읽힌다. Jetson 은 BOOT0 에 닿는 선이 없으므로, 버튼을 누른 채 리셋(또는 전원 투입)될 때만 일어난다. 대책:
  - BOOT0 버튼은 차체 조립 뒤 손이 닿지 않는 쪽으로 두거나 덮는다(하드웨어, 문서로 권고).
  - 펌웨어는 소프트웨어로 부트로더에 들어가는 경로를 두지 않는다(링크 명령으로 DFU 진입 없음).
  - ROM 부트로더에 들어가면 펌웨어가 돌지 않아 STATE 가 끊기고, 호스트는 신선도를 잃어 정지한다. 핀은 부트로더 설정으로 남는다(아래 위험).
- **ROM 부트로더의 핀 사용(위험, AN2606 현행판으로 확인 필요)**: 커뮤니티 자료 기준 H743 부트로더는 USART1(PA9/PA10, PB14/PB15)·USART2(PA2/PA3)·USART3(PB10/PB11)·SPI4(PE11–PE14) 등을 켠다. 메인보드에서 PB10 은 AM26LV32 출력(R_ENC_A), PB14·PB15 는 nFAULT(오픈 드레인), PA2·PA3 은 DRV nCS, PE11·PE13·PE14 는 IN1 이다. TX 핀이 출력으로 잡히면 PB10 에서 수신기 출력과 맞부딪칠 수 있다. 모터 출력은 DRVOFF 풀업으로 꺼진 채다. PA6(IPROPI L)도 부트로더의 SPI1 MISO 출력으로 쓰인다(Codex, AN2606 표 135 — B1 전에 직접 확인). 대책: **DFU 플래시는 WeAct 를 메인보드에서 물리적으로 뺀 상태에서만 한다.** "메인보드 전원만 끈 상태"는 대안이 아니다 — USB VBUS 가 D10 을 거쳐 메인보드 5 V 로 가므로 수신기가 살아날 수 있다(3.1, Codex P1). 조립 상태에서 플래시하는 방법은 전원·신호 격리를 따로 입증한 뒤에만 더한다.
- **S0 첫 준비**(agy): 새 WeAct 는 공장 펌웨어가 들어 있고, 옵션 바이트가 바뀌어 있을 수 있다. 메인보드에 꽂기 전에 단독으로 ROM DFU 에 들어가 옵션 바이트(`BOOT_ADD0`, `RDP`, 쓰기 보호)를 읽어 기록하고, 전체 소거 뒤 출력 잠금 빌드를 먼저 굽는다. 옵션 바이트를 바꿔야 하면 사용자 확인 뒤 CubeProgrammer 로 한다(되돌리기 어려운 변경).
- USB-C 하나를 DFU 와 CDC 가 같이 쓰고, VBUS 가 D10 을 거쳐 메인보드 5 V 로 갈 수 있다(3.1). S1 의 역전원 측정 전에는 VBUS 를 끊은 케이블 또는 UART7 을 권고한다(계획 R2a).

## 9. HIL 빌드(B4 요약)

`FW_BUILD=hil`: 출력 잠금, nSLEEP low, `read_inputs` 를 MCU 안의 모형이 채운다 — 바퀴(1차 지연 + 계산 듀티), KTH 프레임(조향·승강 위치 적분), ADC(듀티 비례), nFAULT high, DRV SPI 는 모형이 응답(`0xC0` 상태). 호스트 빌드 `mcu_model.c` 의 모형을 옮긴다. 고장 주입(엔코더 정지·부호 반전·튐, 링크 끊김)은 빌드 상수로 켜는 시험용 빌드에서만.

- **시운전 상한과 DISARM**(Codex 2라운드, 계약 3.4 정정 2026-11-12): 상한 `STOPPING` 에서도 `DISARM` 은 바로 `DISARMED`(채널별 정지 상태)로 간다. 재무장 금지는 줄지 않는다. 그래서 상한과 `DISARM` 이 같은 주기에 오면 전기 제동이 한 주기도 걸리지 않을 수 있다(`safe_brake=false` 면 코스트). S5 에 **최대 허용 시운전 출력에서 상한 직후 DISARM·보류 중 DISARM** 의 바퀴 정지 시간·거리 측정을 넣고, 코스트가 문턱을 넘으면 그 축의 `safe_brake` 나 시운전 한계를 고친다.
- **S5 의 IWDG 강제 시험용 빌드**(Codex P3): 주 루프를 멈추는 것으로는 TIM6 주기가 계속 갱신하므로 시험이 되지 않는다. 컴파일 시점 옵션 `FW_TEST_IWDG_STALL` 이 켜진 빌드는 정해진 링크 신호 없이도, 부팅 N 초 뒤 TIM6 주기와 갱신을 멈춘다. 이 빌드는 시험 전용이며 STATE flags 로는 구별되지 않으므로 ELF 이름과 USB 제품 문자열로 표시한다.

- **재무장 보류의 HIL 확인**(총괄 배정 2026-11-12): 회귀 시험 v2r6 이 고정한 동작 — 시운전 상한 뒤 보류 중 `ARM` 거부(`reject_reason` 11 `stopping`)와 보류 만료 시각이 `DISARM`·`CLEAR_FAULTS` 로 바뀌지 않음 — 을 B4 의 HIL 빌드에서 호스트가 받은 `STATE`·거부 코드로 확인한다(실기 도구에는 넣지 않는다).

## 10. 하드웨어 없이 하는 확인

- 크로스 빌드(조합: normal·commission·hil × usb·uart7, 출력 잠금).
- 맵·크기: 플래시·DTCM 사용량을 `build-info.json` 에 남기고, 상한(플래시 256 KB, DTCM 96 KB)을 넘으면 실패.
- 스택: B1 은 모든 함수 프레임의 합(재귀가 없으므로 어느 호출 경로보다 크다) + 중첩 예외 프레임 3 개가 주 스택의 50 % 를 넘으면 실패. 코드가 커지는 B2 부터 호출 그래프(`-fcallgraph-info=su`)로 바꾼다.
- 출력 잠금 링크 검사(3.6).
- 호스트 시험: 리셋 원인 판정, 링 버퍼(프레임 단위·32비트 넘김·발행 전 바이트·전부 버림), 송신 펌프(연결 전 버림·순서·막힌 스택), UART BRR(217, +0.006 %), USB 문자열(UID 16진·126자 자름), 클록 설정 상수의 `_Static_assert`(VCO 범위, SPI 분주 → 최대 클록 이하, PLL3 48 MHz).
- B3 크로스 시험: usb·uart7 두 빌드 모두 크기·정체 문자열(`link=`)·벡터·스택. 스택 단계는 주 루프 → OTG(3) → TIM6(2) → SPI·UART7(1) → 예상 밖 IRQ → HardFault → NMI. TinyUSB 간접 호출은 호출자별 대상(드라이버 표·완료 콜백·OSAL 인터럽트 스위치)으로 묶는다. 들여온 TinyUSB 파일 = 빌드가 읽은 파일.
- 선택: Renode 의 STM32H743 플랫폼으로 UART7 빌드 부팅·링크 왕복(USB 는 에뮬 안 됨). B3 뒤 여유가 있을 때.
- 실기로만 되는 것: HSE·PLL 실제 동작, USB 열거, IWDG 리셋 시 DRVOFF high 측정, LPTIM 실제 입력, SPI 파형, ADC 값.

## 11. 묻거나 확인할 것

- (B1 전 확인) AN2606 H74x 부트로더 핀 표(PB10·PB14·PA2·PA6·PE11–PE14), AN5312(rev Y·V 의 ADC 차이 — CRS 는 B3 에서 쓰지 않기로 해 해당 없음), ES0392 SPI 항목.
- (S1) VBUS → D10 → 메인보드 5 V 역전원. 벤치 전원·노트북·Jetson 사이 접지 경로(agy).
- (S2) KTH7801 CS high 최소 시간, 케이블을 뺐을 때 LVDS 수신기 출력, 보드 판(rev Y·V).
- (S1·S5, agy B3) VBUS 감지를 끈 상태에서 Jetson 전원이 꺼졌을 때 D+ 풀업(1.5 kΩ)이 호스트 쪽으로 역전류를 흘리는지, Jetson 재부팅 뒤 `/dev/ttyACM*` 이 케이블을 다시 꽂지 않고 돌아오는지. 모터 구동 중 USB 끊김·리셋(`dmesg`)과 두 보드 접지 전위차 — 절연 필요 여부.
- (알려진 한계) USB 스택이 멈추면 주기가 IWDG 를 계속 갱신하므로 자동 복구가 없다(차량은 명령 만료로 멈춤). 링크 생존 감시 리셋은 별도 판단.

## 12. 자체 검토

독립 검토 세 번(서로 결과를 보지 않음).

**① 원자료 재계산(Claude)**
- 데이터시트 DS12110 의 AF 표에서 PB10 `LPTIM2_IN1`·PD11 `LPTIM2_IN2`(AF3), PD12 `LPTIM1_IN1`·PE1 `LPTIM1_IN2`(AF1), PE7 `UART7_RX`(AF7), PE9·PE11·PE13·PE14 `TIM1_CH1–4`(AF1), PB13 `SPI2_SCK`(AF5)를 직접 확인했다. SPI1 핀·PC1·PC2_C·ADC 채널은 텍스트 추출이 깨져 핀맵 검토(#83)의 대조에 기댄다. B2 구현 때 다시 확인한다.
- IWDG: 400 × 4 / 33.60 kHz = 47.6 ms, / 29.76 kHz = 53.8 ms. PLL: 25 / 5 × 160 / 2 = 400 MHz, 64 / 8 × 100 / 2 = 400 MHz.
- KTH 프레임 `0x0000` 이 두 CRC 변형 모두에서 통과함을 코어 함수로 계산했다(3.7).
- CubeH7 V1.13.0 참조 startup 의 `ExitRun0Mode` 순서와 LL v1.11.6 의 `LL_CRS_SYNC_SOURCE_USB = SYNCSRC_1` 을 원문에서 확인했다.

**② Codex 적대적 검토 1라운드(P1 2·P2 10·P3 5)** — 모두 반영.
- P1: `ACTVOSRDY` 전 RAM 사용 → 어셈블리 시작 코드에서 공급 확정(3.2). "메인보드 전원만 끄고 DFU" 는 VBUS 역전원 때문에 안전하지 않음 → 물리적으로 뺀 상태만(8절).
- P2:
  - HSI 대체 경로 분주(3.3), LPTIM `PRESC=000`(4.3).
  - 판별 ADC·BOOST(3.3), 판별 CRS 동기원(3.3).
  - SPI EOT 재시작 erratum(4.6), JEOS 소비·지우기(4.4).
  - KTH 시작 주체와 세대(3.4·4.5), SPSC 발행 규칙·우편함(3.4).
  - IWDG 초기화 간격 40 ms(3.5), 출력 잠금 검사 세 층(3.6).
- P3: RNG 오류 처리, 설정 생성 경로(3.7), IWDG 강제 시험 빌드(9절), 스택 호출 경로·예외 프레임(6절), DTCM 접근 표현(6절), 11절 정리.
- 반박 없이 맞다고 본 것: HSE 경로 400 MHz·VOS1·플래시 2 WS, PC2SO 방향, IWDG 계산, 리셋 원인 순서, 잠금 빌드의 nSLEEP, CSS→NMI 경로.

**② Codex 2라운드(P1 1·P2 2 새로·남음, P3 4)** — 모두 반영: RNG 실패는 부팅 실패(3.7), KTH 원시 0 검사는 보드 설정으로 한정(3.7), TIM1 잠금은 구조로 보장하고 기계어 검색은 보조(3.6), HSI /10×125·세대 넘김 비교·정렬 패딩·계획 3.2·P0-8 줄. 시운전 상한 STOPPING 의 DISARM 은 계약을 고치는 쪽에 Codex 도 동의(재무장 보류만 줄일 수 없음) — 회귀 시험 v2r6, S5 측정 항목(9절).

**② Codex 3라운드: 새 P1/P2 없음 — 수렴.** P3 poison 순서(3.6) 반영.

**③ agy(Gemini 4 Argon, ①② 미공개) '적히지 않은 전제' 7개** — 모두 반영하거나 넘겼다.
- 공장 펌웨어·옵션 바이트 → S0 절차(8절).
- 조립 상태 DFU 의 부트로더 오분기 → 물리적으로 뺀 상태만(②와 같음).
- 링 버퍼 장벽 → 3.4.
- KTH CS 간격 → 4.6(1 µs, CS high 최소 시간은 실물 확인).
- KTH `0x0000` → 3.7 범위 검증(B2 코어 변경).
- ModemManager·재연결 → 5절(udev 는 호스트 몫).
- 접지 경로·빌드 식별 → 11절·3.7.
