# 메인보드 도착일 결과 (V1.0, 2026-11-24)

[순서표](../plans/2026-11-20-arrival-day-sequence.md)의 순서로 한 줄씩 적는다. 정본 절차는 [도착일 절차서](../plans/2026-11-12-arrival-day-procedure.md)다. 순서표는 11-20 을 전제했지만 납땜이 늦어(현장 보고) 실제로는 11-24 에 했다. 그래서 파일 이름을 실제 날짜로 붙였다.

- 원자료: [assets/arrival-day-2026-11-24/](assets/arrival-day-2026-11-24/)(작은 기록). 노트북 원본은 메인 체크아웃 `artifacts/<UTC>_<단계>_<구분>/`(git 무시)에 있다.
- 원자료가 없는 서술(사용자가 손·눈·멀티미터로 한 확인)은 **(현장 보고)** 로 표시했다. 총괄이 사용자와 함께 한 작업의 보고다. 이 세션이 명령으로 직접 봤지만 저장소 원자료에 남기지 않은 값은 **(세션 관측)** 으로 표시했다.
- 코드: main `de49e746db98`(작업 트리 깨끗). 노트북에서 빌드하고 굽고 확인했다. 노트북은 Arm GNU Toolchain 13.3.rel1, STM32CubeProgrammer 2.23.0 이다.

## 전원 레일 U12 경위 (현장 보고)

메인보드 V1.0 납땜을 마친 뒤 5 V 벅 U12(LMR604403S)에서 이상이 보였다. 단계는 다음과 같다.

1. 처음 무부하 출력이 **5.52 V**(FB 1.084 V)였다.
2. 재리플로우 1 회 뒤 출력이 **8.5–9 V** 로 출렁였다.
3. 그사이 **LCD 판 WeAct H743 한 장이 5 V 과전압으로 파손**됐다. 5V–GND 가 0.5 Ω 로 붙었고, D5 를 떼어도 살아나지 않아 폐기했다.
4. 재리플로우 2 회 뒤 무부하 **5.05 V** 가 됐다(사용자 멀티미터). 절차 기준(공칭 ±5 %, 4.75–5.25 V) 안이다. 총괄이 잡은 더 좁은 목표 4.9–5.1 V 에도 든다.

S0 은 그 뒤 **새 LCD 판 WeAct** 로 했다.

## 0 준비

오실로스코프는 이날 쓰지 않았다(사용자 결정, 6.1 의 PB4 피크 측정 생략과 함께, 현장 보고). 그래서 오실로스코프가 필요한 항목(6.1 PB4, S3 VCC 강하·회생, S5 ②)은 미완료로 남는다.

## 1 빌드 (절차서 2 절)

| 이미지 | 정체 문자열(`strings … \| grep '^forklift-mcu'`) | ELF sha256 |
|---|---|---|
| `fw-normal` | `forklift-mcu normal locked de49e746db98 link=usb` | `bd2515e6…7803cc` |
| `fw-commission` | `forklift-mcu commission locked de49e746db98 link=usb` | `f8f27f8e…eaca` |
| `fw-stall` | `forklift-mcu normal+iwdg-stall-5s locked de49e746db98 link=usb` | `a624e195…1024` |

- de49e746 의 세 이미지는 정체 문자열이 빌드 종류와 맞는다(전체 해시는 `s0/SHA256SUMS`).
- 순서표 1 번은 `fw-open` 까지 네 ELF 를 요구한다. 네 종류가 다 갖춰진 것은 #606 머지 뒤 **main 24a0475f 빌드**다(`build_main_24a0475f/`). normal·commission·stall·open·waived 다섯 개 모두 정체에 `24a0475fe8e9` 가 찍혔고, open 은 `rec=1f4207f4`, waived 는 `waiver=17b73d71` 이다. **빌드 합격은 24a0475f 빌드 기준**이다.

## 2 S0 — 첫 굽기와 옵션 바이트 (절차서 3 절)

WeAct 를 메인보드에서 뺀 채(현장 보고) 노트북 USB 로 했다. 원자료는 `s0/`, 노트북 원본은 `artifacts/20261124T073239Z_s0_ob/`·`…_s0_flash/`·`20261124T073435Z_s0_link/` 다.

| 항목 | 결과 |
|---|---|
| DFU | `0483:df11` 한 대, `STM32_Programmer_CLI -l usb` → `USB1`, 장치 ID 0x450, DFU 펌웨어 0x011a, DFU 시리얼 `200364500000`. 이 시리얼은 H7 ROM 이 보고하는 값이라 11-22 보드와도 같다. 그래서 보드 구별에는 아래 앱 UID 를 쓴다. |
| 옵션 바이트(읽기만, `ob.txt`) | RDP 0xAA(Level 0), `IWDG1_SW` 0x1(소프트웨어), `BOR_LEV` 0x0(VBOR0), `BOOT_ADD0` 0x0800(0x08000000), `SWAP_BANK` 0, `SECURITY` 0. 바꿀 것 없음. |
| 부트로더 버전 | 0x1FF1E7FE = **0x92**(v9.2, `bootloader_id.txt`). |
| 공장 펌웨어 백업 | 2,097,152 B, sha256 `c2b999ab884e7ebf0d18affe2cfb5ac5c99a0927d7ea8ad0d96c7afa58bc0f40`. 내용은 이전 데모 펌웨어("bad_apple"·FATFS 시험 문자열)다. 제3자 펌웨어라 공개 저장소에는 넣지 않았고, 노트북 `artifacts/20261124T073239Z_s0_ob/factory.bin` 에만 있다. |
| 전체 소거 | 백업 크기 확인 뒤 `-e all` → `Mass erase command correctly executed`(exit 0). |
| 굽기 | `-w fw-normal.elf -v` → `Download verified successfully`(exit 0). |
| RESET 뒤 | 07:34:18Z 에 `0483:5740` 열거. 제품 `forklift-mcu normal locked de49e746db98` = 빌드 정체. UID(시리얼) `004800413233511430313337` 은 11-22 보드 `0051003C3233510639383332` 와 다른 판이다. |
| 파란 LED(PE3) | 1 초마다 규칙적으로 깜빡임, 실패 코드 아님(현장 보고). |
| 덤: 링크 30 s | `PASS v3 30.0s states=3000 seq_gaps=0 frame_errors=0 mcu_period=10.0/10.0ms host_gap_max=11.676ms rtt_p50/max=8.67/9.14ms disarm_lost=0/30 resets=0`(`link_30s_verdict.txt`) |
| **판정** | **합격**: LED 1 Hz, `0483:5740`, 제품 문자열이 정체와 같다. |

- 첫 부팅 판정 도구 `tools/h743_first_boot.py` 는 메인보드에 꽂은 상태의 bring-up 기록을 보는 S2(6.1) 도구다. 그래서 보드 단독인 S0 은 절차서 3 절 5·6 대로 판정했다.
- **LCD 판을 메인보드에 꽂을 때**(기록만): 이 판은 PE10–PE14 가 보드 위 LCD 에 이어져 있다. 메인보드에서 PE10·PE12 는 미연결이고, PE11·PE13·PE14 는 R·M·H 의 IN1(TIM1 CH2–4)이다([보드 설계](../design/2026-11-12-weact-h743-board-firmware.md) 4.1).
  - 출력 잠금 빌드: 세 핀이 push-pull low 라 LCD 입력과 드라이버 IN1 이 함께 low 다.
  - 출력이 열린 빌드: TIM1 PWM 이 LCD 쪽에도 그대로 걸린다. LCD 는 받기만 하므로 전기적으로 다투지 않는다.
  - MCU 리셋·부팅 중(핀이 하이 임피던스일 때): LCD 쪽에 풀업이 있으면 IN1 이 high 로 뜰 수 있다. 보드 설계가 보이는 범위는 DRVOFF(드라이버 내부 풀업)와 nSLEEP(내부 풀다운)으로 드라이버가 **최종적으로** 출력을 끈 SLEEP 에 수렴한다는 것까지다([모터 출력 설계](../design/2026-11-12-weact-h743-motor-output.md) 3). 리셋 직전 nSLEEP 이 high 였다면 방전·필터 지연 동안의 출력 펄스는 배제하지 못한다(S5 ② 파형은 이날 미측정). LCD 판의 풀업 유무는 확인하지 않았다(WeAct LCD 판 회로도 미확인).

## 3 udev (절차서 4 절)

| 확인 | 결과 |
|---|---|
| 설치된 `99-forklift-mcu.rules` | 저장소(de49e746) 판과 같음(`cmp`, `udev_checks.txt`). 노트북에 이미 설치돼 있었고 이날 새로 깐 것은 없다(세션 관측). |
| `ls -l /dev/forklift-mcu` | `-> ttyACM0` |
| `ID_MM_DEVICE_IGNORE` | `1` |
| `power/control` | `on` |
| DFU 규칙 | `50-stm32-dfu.rules` 있음 |
| **판정** | **합격**(`udev_checks.txt`) |

## 4–6 S1 (절차서 5 절)

조건: 메인보드 주 입력 12 V 벤치 전원(CC 최대). V1.0 은 DRV8244 의 VM 이 VCC 넷이고 따로 끊는 점퍼·커넥터가 없다. 그래서 "모터 전원 경로를 뺀 채" 는 **모터를 떼어 둔 상태**로 했다(모터·엔코더·KTH 미연결, 현장 보고).

| # | 항목 | 값 | 기준(공칭 ±5 %) | 판정 |
|---|---|---|---|---|
| 4 | VCC | 12.0 V(벤치 12 V) (현장 보고) | 입력 그대로 | 기록 |
| 4 | 5 V 무부하 | 5.05 V (현장 보고, U12 재리플로우 2 회 뒤) | 4.75–5.25 V | 합격 |
| 4 | 5 V 부하 | 측정 안 함 | 4.75–5.25 V | 미완료 |
| 4 | 3.3 V(WeAct 꽂음) | 3.31 V (현장 보고, 무부하·부하 구분 없음) | 3.135–3.465 V | 값은 범위 안 |
| 4 | 19 V 무부하 | 19.02 V (현장 보고, Jetson 끈 뒤) | 18.05–19.95 V | 합격 |
| 5 | 19 V 부하(Jetson stall 10 분 동안) | 18.9–19.1 V, U12·19 V 승압부 발열 없음 (현장 보고) | 18.05–19.95 V | 합격 |
| 5 | Jetson 다리 정체(`jetson_stall.sh 300`) | 정지 300 s ALL PASS(14,948 주기, 간격 p99 20.2·최대 20.8 ms, 40 ms 초과 0), 전 코어 부하 300 s ALL PASS(14,666 주기, p99 23.8·최대 39.5 ms, 80 ms 이상 0), 기록 누락 0. Jetson 이 꺼지거나 재부팅되지 않음 | 순서표 5 | 합격 |
| 6 | 역전원 | (측정 예정) | 5 V 에 전압 없음 | 미완료 |

- Jetson 은 forklift-jetson(Orin Nano 개발자 키트 Super, R39.2.1)이고, 부팅 유닛 `forklift-jetson-clocks.service`(2026-11-22 설치) 로 클럭을 고정한다. 이날 `host.txt` 에는 MAXN_SUPER 만 남았다. CPU 최소 주파수 1728 MHz 와 유닛 active 는 시작 때 따로 확인했다(세션 관측). 기록: 메인 체크아웃 `data/jetson_check/2026-11-24/stall/`, 판정 사본 `s1_stall/`. 이 Jetson 은 순서표가 전제한 Orin Nano Super(`scienceowl`)가 아니라 2026-11-22 에 실차용으로 재설치한 장치다.
- S1-2 의 GPU 부하는 순서표대로 CPU 부하로 합격 처리했다(총괄 2026-11-18).

## Jetson ↔ STM 링크 (메인보드 장착, 순서표 밖 — 사용자 요청)

WeAct USB-C ↔ forklift-jetson USB-A, 이미지 `normal locked de49e746db98`. 열거·제품·UID 는 노트북과 같다(세션 관측, udevadm). 링크 60 s **PASS**: STATE 5,997, seq 틈 0, 프레임 오류 0, MCU 주기 10.0/10.0 ms, 호스트 간격 최대 11.5 ms, 왕복 p50/최대 5.65/10.27 ms, DISARM 잃음 0/60, 리셋 0(`jetson_link/`).

## 7–10 S2 (절차서 6 절) — SPI 두 버스 무응답의 원인 두 가지

### 무엇이 보였나

모터·엔코더·KTH 를 하나도 잇지 않은 채(현장 보고) 첫 부팅 판정(`bringup.py record 15 s` → `tools/h743_first_boot.py --lift-absent`)을 돌렸다. 네 드라이버의 `driver_status` 가 모두 0x00 이었고 `DRIVER_NOT_READY` 가 떴다(`s2_noVM/`). KTH 두 축의 `kth_frames` 도 0 이었다.

- V1.0 은 DRV8244 의 VM 이 VCC 넷이라 VM 은 이미 들어와 있었다. 그래서 0x00 은 "VM 없음"이 아니다.
- 펌웨어 코어(`drive.c`)는 초기화를 L 부터 한 채널씩 하고, 응답이 무효면 100 ms 뒤 L 부터 다시 한다. 무효 응답은 0 으로 적는다.
  - 그래서 0x00 은 실제 SDO 값이 아니다.
  - R·M·H 는 한 번도 불리지 않았다.
- KTH 쪽 SPI2 는 센서가 없어도 매 주기 스스로 클록을 내고, 끝난 전송을 CRC 와 무관하게 센다. 그런데 프레임 수가 0 이었다. 즉 **SPI2 는 전송을 한 번도 끝내지 못했다**(Codex 지적, 코드로 확인). SPI1 은 `driver_status = 0` 이 완료된 무효 응답에도 적히므로 이 기록만으로는 완료 여부를 말할 수 없다. 두 버스가 같은 `spi_setup()` 을 쓰므로 같은 원인을 의심했다.

### 원인 ① — 펌웨어 `spi_setup` 의 쓰기 순서 (#606 으로 수정·머지)

`spi_setup()` 은 `CR1 = 0` → `CFG1` → `CFG2 = MASTER | SSM | …` 순서로 쓰고, 그 뒤에야 `CR1 = SSI` 를 썼다.

- SSM = 1 이면 내부 NSS 가 SSI 를 따른다. RM0433(SPI mode fault)에 따르면 NSS 가 low 인 마스터는 MODF 가 서고 MASTER 가 지워질 수 있다. MODF 자체는 측정하지 않았고, 관측과 맞는 RM0433 동작이다.
- 실제 SPI 경로는 이날 처음 하드웨어에서 돌았다. 11-17·11-22 의 HIL 빌드는 `board_hal_hil.c` 라 SPI 를 쓰지 않는다.

진단 이미지 세 가지로 원인을 좁혔다. 모두 출력 잠금이고, main 에 넣지 않았다. 셋 다 굽기 전에 Codex 검토를 받았고 P1 은 없었다.

| 이미지(로컬 태그) | 변경 | 결과 |
|---|---|---|
| `diag-spi-ssi-1`(72eee346) | SSI 를 MASTER 보다 먼저 | WeAct 단독: `kth_frames` 15 s 에 +14,980(초당 약 1,000) → **SPI2 회복**(`diag_spi/`) |
| `diag-spi-ssi-2` | 위 + `driver_status` 에 SPI1 원시 수신·SR·IRQ/시작 횟수 | 메인보드: SR = RXP·EOT·TXTF·TXC, MODF 0, MASTER 1 → **SPI1 도 MCU 쪽은 정상 완료**. 수신 워드는 매번 0xFFFF 였다(재시도당 4 전송, CONFIG3 되읽기에서 실패)(`diag_spi2/`) |
| `diag-spi-ssi-3` | SSI 수정 + 초기화가 실패해도 다음 채널로, 채널별 원시 응답 바이트 | 네 채널 모두 FF → 칩 하나가 아니라 **공유 경로** 문제(`diag_spi3/`) |

- 정식 수정은 [#606](https://github.com/lightminn/forklift/pull/606)(main 24a0475f)이다.
  - 쓰기 순서를 순수 함수 `spi_setup_writes()` 로 뺐다.
  - 호스트 시험 `spi_setup_order` 는 mode-fault 규칙 모형으로 순서를 고정한다.
  - Codex 리뷰에서 새 P1/P2 없이 수렴했다.

### 원인 ② — 헤더 소켓 납땜 (현장 보고)

SDO 쪽은 다음 순서로 좁혀졌다.

1. U14(L) 리플로우 뒤(현장 보고) diag-2 로 다시 보니 L 수신이 0x0000 으로 바뀌었다(`diag_spi2_after_reflow/`). 부팅마다 0xFFFF 와 0x0000 이 오가는 것은 PB4 가 어느 칩에도 구동되지 않고 떠 있을 때와 맞는다. PB4 전압을 직접 잰 것은 아니다.
2. 메인보드 U4-9(PB4)↔GND 는 개방이었다(현장 보고). SDO 단락은 아니다.
3. 회로도 1·2쪽(MISO·CLK·MOSI 넷에 부품·테스트 패드 없음)과 EasyEDA 배선(넷마다 MCU + DRV 4 개 연결, Ratlines 0, 총괄 확인)에서 설계·배선 문제는 없었다.
4. 남은 공유 지점인 **헤더 소켓 U2·U4 의 88 핀을 사용자가 모두 재납땜했다**. 어느 핀이 원인이었는지는 특정할 수 없다.
5. 재납땜 뒤 diag-3: **L/R/M/H = C0/C0/C0/C0, DRIVERS_READY, mode DISARMED**(기록 1,500 STATE 가 처음부터 DISARMED 라 BOOT → DISARMED 전이 자체는 기록에 없다), 마지막 STATE 전류 판독 0/19/38/5 mA(`diag_spi3_after_socket/`).

### S2 첫 부팅 판정 (6.1 끝)

이미지는 `diag-spi-ssi-1` 이다. 수정 PR 머지 전 이미지이고, 코드는 72eee346 = main de49e746 + SSI 순서다. 메인보드·12 V, `h743_first_boot.py --lift-absent`(`s2_first_boot/`).

| 항목 | 결과 | 판정 |
|---|---|---|
| `drivers` | 네 채널 bad_status 없음, not_ready 0, DRIVERS_READY | 합격 |
| `link`·`wheels_still` | 오류·주기 초과 증가 0, 바퀴 이동 0 | 합격 |
| `kth`·`faults` | 프레임은 10 s 10,000 씩이나 각도 NaN, STEER/LIFT_SENSOR_INVALID(`--lift-absent` 에서도 STEER 는 unexpected) | 실패 — **KTH 미연결로 예상된 실패** |
| `current` | 아래 표 | 실패(기존 기준 유지) |
| **종합** | `result: fail`(current·faults·kth) | **실패** — 드라이버·링크는 합격, 나머지는 센서 미연결·전류 기준 |

IPROPI 전류 판독(10 s 창, 1,001 행):

| 채널 | 중앙값 | p99 | 최대 | 50 mA 초과 |
|---|---:|---:|---:|---:|
| L | 5 mA | 131 mA | 138 mA | 154 |
| R | 0 mA | 64 mA | 381 mA | 15 |
| M | 1 mA | 325 mA | 374 mA | 192 |
| H | 0 mA | 41 mA | 273 mA | 4 |

- 영점(중앙값)은 네 채널 모두 50 mA 이하다. 짧은 튐이 기존 피크 기준 50 mA 를 넘긴다. 이 10 s 창에서 튐은 대부분 한 표본이고, 10 ms STATE 해상도에서 주기성이 보이지 않으며, L·M 동시 초과가 독립일 때 기대값에 가깝다(관찰 수준). `CURRENT_SENSE_INVALID` 는 없다.
- 모터가 떨어져 있고 출력이 STANDBY(Hi-Z)라 실제 전류일 가능성은 매우 낮다(Codex: DRV8244 표 8-4).
- 원인은 확정하지 않았다. 1순위 후보는 매 1 ms SPI2 전송 직후 ADC 변환이 시작돼 두 동작이 겹치는 것이다.
- TI 사양의 IPROPI 영점 오프셋 최대 15 µA 는 표시값으로 약 71 mA 다. 그래서 기존 50 mA 피크 기준은 사양 오프셋보다도 작다.
- 기준 변경안('중앙값 ≤ 50 mA' 추가 + p99·최대 보고)과 SPI2↔ADC 위상 A/B 실험은 별도 PR 로 둔다(총괄 결정).
- 드라이버 초기화가 실패하던 부팅(`DRIVER_NOT_READY`, 잠금 펌웨어라 nSLEEP 은 high)에서 H 에 남던 잔여(중앙값 303 mA)는 드라이버가 준비된 뒤 중앙값 0 으로 사라졌다.

### 링크 10 분 (메인보드 장착, 노트북)

`diag-spi-ssi-1`, 18:25:24–18:35:24, **PASS**(`s2_link_10min/`).

| 항목 | 결과 |
|---|---|
| STATE | 60,000 |
| seq 틈·프레임 오류 | 0 · 0 |
| MCU 주기 | 10.0/10.0 ms |
| 호스트 간격 최대 | 12.4 ms |
| 왕복 p50/최대 | 5.25/6.02 ms |
| DISARM 잃음·리셋 | 0/600 · 0 |

L 모터는 이 10 분 도중(18:3x 대) 연결했다(현장 보고). 출력 잠금이라 영향은 없다.

### 6.4 재무장 보류 (fw-commission, main 24a0475f)

**합격**(`s2_rearm_hold/`).

- `rearm_hold_check.py --axis L --stop-grace-ms 300 --limit-s 2.0` → `result ok`, checks 7/7 참.
- 재무장 확인 구간에서 ARM 을 17 회 보냈다(최초 ARM 포함 전체 18 회). 보류 끝 8.830–8.840 s 안에 첫 재무장 ARM 이 받아들여졌다(8.840013 s).
- 시험 조건은 센서(엔코더·KTH) 미연결이다.
- 엔코더·KTH 가 없어도 돈다. 시운전 빌드의 ARM 은 `sensors_valid` = `adc_valid` 만 보고, L commission 은 KTH 를 요구하지 않는다.

### 남은 S2

- KTH·감속비(6.2·6.3)는 미완료다. 엔코더·KTH 가 미연결이다.
- **6.1 VM 투입 순서는 절차대로 하지 않았다 — 절차 예외로 기록한다.**
  - V1.0 은 VM 이 VCC 넷이다. 주 입력을 넣는 순간 VM 도 들어간다.
  - 0.3–0.5 A 전류 제한: 하지 않았다. 벤치는 CC 노브 최대(값 미상)였다(현장 보고).
  - 공통 접지 도통 확인: 기록 없음(미확인).
  - MCU 장착 전 입력 핀(IPROPI·nFAULT·KTH·엔코더) 전압 확인: 하지 않았다(미실시).
  - **PB4 SDO high 피크(6.1-6): 측정하지 않았다 — 사용자 결정으로 오실로스코프 확인 생략**(현장 보고). 사용자가 든 근거는 두 가지다.
    - DRV SDO high 는 nSLEEP 3.3 V 를 따른다.
    - WeAct 플래시는 펌웨어가 쓰지 않는다.
  - 절차서는 SDO 가 무부하에서 3.8 V 까지 갈 수 있어, 스코프가 없으면 플래시를 단 채 부팅하지 않도록 한다. 펌웨어가 플래시를 쓰지 않아도 PB4 의 전기적 노출은 생긴다.
  - 이날 WeAct SPI 플래시의 실장 여부는 확인하지 않았다(미확인).
  - 드라이버가 응답하기 시작한 부팅들(소켓 재납땜 뒤)에서 PB4 high 전압은 **미판정**이다.

## 11 S3 — L 축만 (모터 L 하나만 연결, 엔코더·KTH 없음)

조건(현장 보고):
- 바퀴는 띄웠고, 사람이 주 전원 차단을 손에 쥐었다.
- 벤치 전원은 **CC 노브 최대, 값 미상(아날로그 전원)**이다.
- 이미지는 main 24a0475f 다섯 개(`build_main_24a0475f/`)다. 정체는 모두 `24a0475fe8e9`(fw-open `commission OPEN 24a0475fe8e9 rec=1f4207f4`)다.

| 실행 | 이미지 | 결과 |
|---|---|---|
| 0.05·0.1 s | fw-open | **취소** — 도구의 '바퀴 띄움' 확인에서 멈춘 것을 늦게 봤다. 사용자 지시가 바뀌어(현장 보고) `no` 로 취소했다(ARM 전, 모터 안 돎, `result refused`). 그 사이 사용자가 답한 'none'(현장 보고)은 없던 구동에 대한 답이라 쓰지 않는다. **첫 실제 구동은 다음 줄의 0.15·1.5 s 명령이다.** |
| 0.15·1.5 s | fw-open | 듀티 0→0.15 를 80 ms(변화율 제한), 0.15 를 180 ms 유지했다. **0.26 s 에 ENCODER_STUCK_L 이 처음 기록됐고 다음 STATE 에서 듀티 0**(엔코더 미연결, 보호 정상 동작, 도구 driven 0.26–0.28 s). 도구 current_peak/mean 298/159 mA. 방향은 너무 짧아 **관측 못 함**(도구 observed unknown, note no_observation)(`s3_L_015_1p5s/`). |
| 0.05·20 s | **진단 이미지 `diag-commission-unlimited-1`**(절차 밖 진단) | 도구 driven 19.94–19.96 s, 결함은 KTH 미연결 둘뿐. L IPROPI 판독(듀티 0.05 인 1,992 행) 중앙값 144.5 mA, 평균 146, p90 234, 최대 393 mA. 모터 없는 R/M/H 도 최대 192/363/259 mA 까지 튀어 최대값엔 잡음이 섞였다. **모터 축 시계방향 관측**(현장 보고). 감속기 때문에 차체 기준 앞·뒤는 미정이라 도구엔 unknown 으로 넣었다(`s3_L_diag_005_20s/`). |

- **진단 이미지 `diag-commission-unlimited-1`**(브랜치 `diag/commission-long`, main 24a0475f 위, main 에 넣지 않음): 사용자 요청으로 시운전 상한을 풀었다.
  - 듀티 상한 1.0, 시간 상한 2,147,483 ms(검증 천장).
  - 엔코더 정체 보호는 문턱 1.0 으로 사실상 껐다. 도구 상한도 같이 풀었다.
  - 그대로 둔 것: 과전류 5 A·20 ms, STOP/DISARM, 링크 끊김 정지(COMMISSION 유효 60 ms), DRVOFF, 변화율 제한.
  - **Codex 검토는 사용자 결정으로 생략했다**(사용자 2026-11-24).
  - 그 전 단계 `diag-commission-long-1`(0.5·11 s)은 빌드만 하고 굽지 않았다.
- 벤치 전류계(현장 보고, 아날로그, 12 V 입력 전체, Jetson 없음): 모터 정지 0.07 A, 0.05 구동 중 0.19 A. 입력 증가분은 약 **0.12 A** 다.
- IPROPI 판독과 벤치 입력 전류는 **서로 다른 양이라 따로 둔다**. PH/EN 모드에서 EN = 0(PWM 꺼진 구간)이면 OUT1·OUT2 가 모두 H 이고 IPROPI 는 그 재순환 전류도 보고한다(DRV8244 표 8-4, Codex 지적). 그래서 "하이사이드 평균 = 입력 평균" 식의 환산은 쓰지 않는다. 둘의 관계(일치 여부)는 미확인이다.
- 2 행은 fw-open 으로 한 S3 시도다. 다만 절차서 7 절의 첫 구동(0.1 s 로 어느 축이 움직이는지 확인)은 1 행이 ARM 전에 취소돼 **하지 않았다**. 3 행(진단 이미지)부터는 절차 밖 진단이다.
- **미완료**(S3 합격으로 치지 않음):
  - `duty_sign`(차체 기준 방향 미정) — **L 축은 2026-11-26 사용자 관찰로 채움: 양의 듀티 = 차체 전진**([2026-11-26 벤치](2026-11-26-bench-l-real-motor.md), 진단 이미지·진단 도구 4fc69237·같은 삼각파 명령). R·M·H 는 미완료
  - `sensor_sign`·속도(엔코더 미연결)
  - 무부하 최대 바퀴 속도
  - VCC 강하·회생(오실로스코프 생략)
  - 전류의 벤치 전류계 대조(IPROPI 와 입력 전류의 관계 미확인)
  - R·M·H 축(모터 미연결)

### Jetson ↔ H743 (메인보드 장착, Jetson 은 메인보드 19 V 로 켬)

- forklift-jetson 의 `~/forklift` 를 main 24a0475f 추적 파일로 맞췄다. 진단 도구는 `~/forklift-diag-commission-long` 에 따로 두었다(main 체크아웃에는 진단 코드가 없다)(세션 관측).
- 열거·정체: `commission OPEN diag-commission-unlimited-1 rec=1f4207f4`, UID 004800413233511430313337. CPU 최소 주파수 1728 MHz 로 클럭 고정을 확인했다(세션 관측, sysfs).
- 링크 60 s **PASS**(`jetson_link_60s_unlimited.jsonl.xz`).

  | 항목 | 결과 |
  |---|---|
  | STATE | 6,001 |
  | seq 틈·프레임 오류 | 0 · 0 |
  | MCU 주기 | 10.0/10.0 ms |
  | 호스트 간격 최대 | 10.5 ms |
  | 왕복 p50/최대 | 9.68/10.38 ms |
  | DISARM 잃음·리셋 | 0/60 · 0 |

- **L 삼각파 구동**(절차 밖 진단, 사용자 지시·허락(현장 보고), 18:56:41–18:57:15, `s3_L_diag_tri_005_030_x3/`)
  - 듀티 0.05 → 0.30 → 0.05 를 설정 주기 10 s 로 3 회 돌렸다. 0.1 s 조각, 기울기 0.05/s 다. 조각 루프의 부가시간 때문에 STATE 에서 본 꼭대기 간격은 10.49 s·10.48 s 였다.
  - 진단 브랜치 도구 `bringup.py commission --profile-tri 0.05 0.30 10 3`(커밋 4fc69237)을 썼다. 노트북 호스트 MCU 모형으로 먼저 2 주기 시험했다. 그때 조각 길이가 절반으로 줄던 버그(hold_duty 의 50 ms 인계)를 찾아 고쳤다.
  - 도구 변경은 사용자 지시대로 Codex 검토 없이 진행했다.
  - **결과**
    - 세 번의 가감속을 모두 마쳤다. 0 이 아닌 듀티 구간은 31.44–31.46 s, 듀티 최대 0.2975 였다.
    - 리셋 없음(boot_id 하나 — 리셋 원인 PIN 은 실행 전 부팅 값), 과전류 결함 비트 미검출.
    - 끝 상태는 DISARMED·출력 0(`cleanup_confirmed`)이다.
  - 도구 결과 라벨은 `aborted: authorization_revoked` 다.
    - 도구는 ARM 시간 예산을 0.3 + 30 + 0.5 + 1 = 31.8 s 로 잡는다.
    - 조각 300 개 루프의 부가시간 때문에 이 예산이 끝나는 시점이 **듀티 0 유지 중**(31.81 s)으로 밀렸다. 그때 명령 유효기간이 지나 COMMAND_TIMEOUT → STOPPING 으로 펌웨어가 정리했다.
    - 구동 구간에는 영향이 없다. 다음 실행부터는 예산을 실제 조각 시간으로 잡도록 고친다.
  - L 전류 판독(IPROPI) 듀티 구간별 중앙값/p90:

    | 듀티 | 중앙값 | p90 |
    |---|---:|---:|
    | 0.05 | 153 mA | 250 mA |
    | 0.10 | 167 mA | 271 mA |
    | 0.15 | 182 mA | 279 mA |
    | 0.20 | 189 mA | 298 mA |
    | 0.25 | 199 mA | 300 mA |
    | 0.30 | 199 mA | 306 mA |

    - 듀티는 6 배인데 중앙값은 약 1.3 배만 올랐다. 무부하 바퀴라 마찰이 지배하는 것으로 보이지만 확정하지 않았다.
    - 이 표는 **시험용 모터**로 잰 것이라 L 실 모터의 비교 기준이 아니다(사용자 2026-11-26). 실 모터 기준선은 [2026-11-26 벤치](2026-11-26-bench-l-real-motor.md)의 표다.
    - 모터 없는 채널도 구동 중 최대 955 mA 까지 튀어, 판정에는 중앙값을 쓴다.
  - 사용자 관찰(현장 보고): 방향 바뀜 없음, 소리는 듀티를 따라 정상, 발열 없음.
  - 벤치 전류(현장 보고, 아날로그, 12 V 입력, Jetson 이 메인보드 19 V 에 연결된 상태): 대기 약 0.85 A, 최대 약 1.07 A. 구동 증분은 약 **0.22 A**(듀티 0.3 부근)다.
    - IPROPI 판독과 입력 전류의 관계는 위 S3 와 같은 이유로 미확인이다.

- ⚠️ **보드에는 지금 진단 이미지 `diag-commission-unlimited-1` 이 남아 있다**(시운전 듀티·시간 상한 해제, 엔코더 정체 보호 꺼짐). **다음 사용 전에 반드시 main 이미지(fw-normal 또는 fw-open)로 다시 굽는다.**

## 오늘 판정 요약

| 단계 | 판정 |
|---|---|
| 0 준비 | 오실로스코프는 사용자 결정으로 쓰지 않음 |
| 1 빌드 | 합격(main 24a0475f 의 네 ELF 기준, de49e746 은 셋) |
| 2 S0 | 합격 |
| 3 udev | 합격 |
| 4 S1 레일 | 부분: 5 V 무부하·19 V 무부하/부하 합격, 3.3 V 는 값만(무부하·부하 구분 없음), 5 V 부하 미측정 |
| 5 S1 Jetson 부하 | 합격 |
| 6 S1 역전원 | 미완료 |
| 7 S2 VM·PB4 | **절차 예외**: VM = VCC 라 투입 순서·0.3–0.5 A 제한·장착 전 핀 전압을 하지 않음, PB4 피크 미판정(사용자 결정으로 생략) |
| 7 S2 첫 부팅 | **종합 실패** — drivers·link·wheels 합격, current 실패(기존 기준), kth·faults 는 센서 미연결 |
| 8–9 S2 KTH·감속비 | 미완료(센서 미연결) |
| 10 S2 재무장 보류 | 합격 |
| 11 S3 | L 만 부분: fw-open 으로 한 S3 시도 0.15·1.5 s(정체 보호로 0.26 s 정지), **최초 0.1 s 확인은 미실시**. 이후 20 s·삼각파는 진단 이미지로 한 절차 밖 진단. 방향(차체 기준)·센서·전류 대조·오실로스코프는 미완료 |
| — Jetson ↔ H743 | 링크 60 s 합격(메인보드 장착 두 번). L 삼각파: 세 번의 가감속을 마쳤고 듀티 0 이 된 뒤 도구 예산이 끝나 COMMAND_TIMEOUT → STOPPING 으로 정리됐다. 도구 최종 결과는 `aborted: authorization_revoked`, 끝 상태는 DISARMED·듀티 0 |
| 12–13 S5 | 미실시 |

⚠️ 벤치 세션 종료 시 보드 상태: WeAct 에 **진단 이미지 `diag-commission-unlimited-1`** 이 남아 있다. 다음 사용 전에 main 이미지로 다시 굽는다. Jetson `~/forklift` 는 main 24a0475f 이고, 진단 도구는 `~/forklift-diag-commission-long` 에만 있다(세션 관측).
