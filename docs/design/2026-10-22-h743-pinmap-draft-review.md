# STM32H743VIT6 핀맵 초안 검토

작성일: 2026-10-22 · 대상: 팀이 공유한 CubeMX 핀 배치 화면(STM32H743VITx LQFP100, 2026-10-21 카카오톡 공유본) · 상태: **초안 검토, 팀 하드웨어 파일은 고치지 않음**

MCU 는 WeAct Studio MiniSTM32H7xx 보드의 STM32H743VIT6 가 유력하지만 아직 사지 않았다. 핀맵도 확정본이 아니다. 이 문서는 초안의 각 신호를 ST 핀 데이터·참조 매뉴얼·부품 데이터시트·WeAct 보드 회로도와 대조해 **실기기 없이 확인할 수 있는 문제**만 적는다. 고칠 방법은 후보로만 적고, 결정은 하드웨어 담당 팀원이 한다.

## 1. 대조 근거

| 근거 | 무엇을 확인했나 |
|---|---|
| ST `STM32_open_pin_data` — `mcu/STM32H743VITx.xml`, `mcu/IP/GPIO-STM32H747_gpio_v1_0_Modes.xml` (CubeMX 와 같은 핀 데이터, 2026-10-22 받음) | 핀마다 가능한 신호와 AF 번호 |
| RM0433 Rev 7, 표 348 「LPTIM features」, 43.4.15 Encoder mode, 12.3.1 SYSCFG_PMCR | LPTIM 엔코더 지원 범위, PC2_C 아날로그 스위치 |
| ST DS12110 Rev 7 (STM32H742/743 데이터시트, WeAct 저장소 `Datasheet/MCU/STM32H743VI_Datasheet.pdf`) 표 8·9·21 | 핀 I/O 구조(FT/TT), 입력 전압 절대최대 |
| TI DRV8244-Q1 SLVSG24C (2022-11) | SPI 변형 핀, nSLEEP·DRVOFF 기본 풀, S_MODE 기본값, SPI 모드, AIPROPI |
| KTH7801 Series 데이터시트(2024-09-27) 5장 | SPI 모드 3, 최대 클럭, CRC 프레임 |
| WeAct `MiniSTM32H7xx` 저장소 — `Hardware/STM32H7xx SchDoc V12.pdf`, `SDK/HAL/STM32H743/*/*.ioc` | 보드가 이미 쓰는 핀, 헤더에 나온 핀, 풀업 |
| 발표 7주차 SCRIPT.md 4–6쪽(팀 EasyEDA 자료 2026-10-14) | 채널 의미(L·R 바퀴, M 조향, H 승강), 메인보드 회로의 드라이버 신호 |

채널 의미는 7주차 자료가 정본이다: **L·R = 뒤 구동 바퀴, M = 조향, H = 승강** (사용자 확인 2026-10-14). 바퀴는 MT6701 증분 엔코더(ABZ, 1,024 펄스/회전), 조향·승강은 KTH7801 절대각(SPI, 12비트 + CRC-4)이다.

## 2. 초안 핀 배치(그림에서 옮겨 적음)

| 기능 | 신호 → 핀 |
|---|---|
| 모터 방향 | MTR_L_DIR PE2 · MTR_R_DIR PE3 · MTR_M_DIR PE4 · MTR_H_DIR PE5 |
| 모터 PWM | MTR_L_PWM PE9 · MTR_R_PWM PE11 · MTR_M_PWM PE13 · MTR_H_PWM PE14 · **MTR_H_PWM PB10(중복)** |
| 모터 SPI | MTR_SCK PA5 · MTR_MOSI PA7 · MTR_MISO PB4 · nCS L/R/M/H = PA0/PA1/PA2/PA3 |
| 모터 DRVOFF | L PD9 · R PD10 · M PD13 · H PD14 |
| 모터 nFAULT | **MTR_nFAULT PB0(채널 없음)** · MTR_L_nFAULT PB11 · **MTR_L_nFAULT PB15(중복)** · MTR_M_nFAULT PB14 · MTR_R_nFAULT PB12 |
| 모터 IPROPI | L PA6 · R PC4 · M PB1 · H PC0 |
| 바퀴 엔코더 | L_ENC_IN1 PD12 · L_ENC_IN2 PE1 · L_ENC_IDX PC6 · **R_ENC_IN1 없음** · R_ENC_IN2 PD11 · R_ENC_IDX PC7 |
| 절대각 엔코더 SPI | KTH_SCK PB13 · KTH_MOSI PC1 · KTH_MISO PC2_C · KTH_M_nCS PD0 · KTH_H_nCS PD1 |
| IMU | MU_nCS PD3 · MU_INT PD4 · **버스 신호 없음** |
| Wi-Fi | WIFI_TX PA12 · WIFI_RX PA11 · WIFI_RST PA10 |
| SD | SDMMC1 D0 PC8 · D1 PC9 · D2 PC10 · D3 PC11 · CK PC12 · CMD PD2 |

## 3. 확인 결과

심각도는 **A = 이대로면 기능이 안 된다**, **B = 동작은 하지만 고치거나 확인해야 한다**, **C = 알아 두면 되는 것**이다.

### A. 그대로면 안 되는 것

**A1. 오른쪽 바퀴 엔코더 IN1 이 없다.** R_ENC_IN2(PD11)와 R_ENC_IDX(PC7)만 있다. PD11 에는 범용 타이머 채널이 없고 `LPTIM2_IN2`(AF3)만 있다(아래 A2). LPTIM2 의 짝인 `LPTIM2_IN1` 이 나오는 핀은 이 패키지에서 **PD12(이미 L_ENC_IN1)와 PB10 둘뿐**이다. PB10 에는 지금 `MTR_H_PWM` 이 중복으로 붙어 있으므로, **PB10 이 원래 R_ENC_IN1 자리이고 이름표만 잘못 붙은 것으로 보인다**(추정 — 설계자 확인 필요). H 채널 PWM 은 PE14(TIM1_CH4)에 이미 있다.

**A2. 바퀴 엔코더는 범용 타이머가 아니라 LPTIM 엔코더 모드로만 셀 수 있다.** 핀 쌍을 대조하면:

| 쌍 | 가능한 하드웨어 계수기 |
|---|---|
| L: PD12 / PE1 | PD12 = `LPTIM1_IN1`(AF1)·`TIM4_CH1`(AF2), PE1 = `LPTIM1_IN2`(AF1) 뿐 → **LPTIM1 엔코더만 가능** (TIM4_CH2 는 PD13 인데 MTR_M_DRVOFF 로 쓰임) |
| R: (PB10) / PD11 | PB10 = `LPTIM2_IN1`(AF3), PD11 = `LPTIM2_IN2`(AF3) → **LPTIM2 엔코더만 가능** |

RM0433 표 348 기준 H743 에서 엔코더 모드가 있는 LPTIM 은 **LPTIM1·LPTIM2 뿐**이라 둘 다 쓸 수 있다. 다만 TIM 엔코더와 다른 점이 있어 펌웨어가 맞춰야 한다.
- 카운터 16비트, 입력 캡처 없음 → 속도는 고정 주기 위치 차분으로 구한다. `ARR = 0xFFFF` 면 카운터는 4체배 4,096 카운트/회전 기준 16 바퀴마다 돌아 넘어간다. 그래서 펌웨어는 읽을 때마다 **모듈러 16비트 차분**을 누적해야 하고, 두 읽기 사이 이동량이 32,768 카운트(8 바퀴) 미만이어야 한다 — 1 ms 주기면 충분히 여유가 있지만 **실제 최대 읽기 간격**(인터럽트가 밀릴 때 포함)으로 확인한다.
- 엔코더 모드는 LPTIM 이 **내부 클럭**(`CKSEL = 0`)이고 분주가 1(`PRESC = 000`)일 때만 되며, 입력 주파수는 LPTIM 클럭의 1/4 이하여야 한다(43.4.15).
- 4체배는 `CKPOL = 10`(양 에지)로 설정한다. 디지털 필터(`CKFLT`)는 2·4·8 클럭뿐이다.
- `LPTIM_CNT` 는 두 번 연달아 읽어 같은 값일 때만 믿을 수 있다(RM0433 43.4 카운터 읽기 주의).

**의도가 LPTIM 이었는지 확인이 필요하다.** TIM3·TIM4·TIM5·TIM8 처럼 32·16비트 범용 타이머 엔코더를 원하면 쌍을 옮겨야 한다. 헤더에 나와 있고 보드 부품과 겹치지 않는 후보는 TIM3/TIM8 = PC6·PC7(지금 IDX 둘), TIM5/TIM2 = PA0·PA1(지금 L·R nCS) 등이다. 어느 쪽이 나은지는 다음 판 배선과 함께 정할 일이라 여기서 고르지 않는다.

**A3. nFAULT 이름이 맞지 않는다.** `MTR_L_nFAULT` 가 PB11 과 PB15 두 핀에 있고, PB0 은 채널 없이 `MTR_nFAULT`, **H 채널 nFAULT 는 없다**. 넷 중 하나는 H 일 텐데 어느 것인지 그림만으로는 알 수 없다. EXTI 줄 번호는 PB0=0, PB11=11, PB12=12, PB14=14, PB15=15 로 다른 인터럽트 입력(MU_INT PD4=4, IDX PC6=6·PC7=7)과 겹치지 않으므로 어느 쪽으로 정리해도 EXTI 충돌은 없다. 다섯 핀 중 하나는 남는다.

**A4. IMU 버스가 없다.** MU_nCS·MU_INT 만 있고 SCK·MOSI·MISO 가 없다. nCS 가 있으니 SPI 장치인데, 모터 SPI(SPI1)·KTH SPI(SPI2) 중 어디에 붙는지, 아니면 별도 버스인지 정해야 한다. 모드가 서로 다르다는 점에 주의한다 — DRV8244 는 SCLK 평상시 low·상승 에지 출력·하강 에지 샘플(모드 1), KTH7801 은 모드 3 이고 **최대 10 MHz**, 그리고 KTH 버스는 STDS 보드의 LVDS 로 나간다. IMU 품번이 아직 어디에도 없어 모드·속도는 품번이 정해진 뒤 맞춘다.

**A5. DRV8244 의 nSLEEP 이 MCU 핀에 없다.** 7주차 메인보드 회로에는 채널마다 `x_SLEEP` 이 있는데 핀맵에는 없다. DRV8244 는 **nSLEEP 에 내부 풀다운**이 있어 떠 있으면 SLEEP 상태로 남아 모터가 돌지 않는다(SLVSG24C 8.3.2). 메인보드에서 3.3 V 로 묶었다면 문제없지만, 그러면 MCU 가 드라이버를 재시작(SLEEP→깨우기)할 길이 없다. 어느 쪽인지 회로도 확인이 필요하다. 깨어난 뒤에는 SPI 로 `CLR_FLT` 를 보내야 STANDBY 로 넘어가므로(8.4.3), 펌웨어 초기화 순서에 넣는다.

**A6. IPROPI 를 ADC 핀에 바로 이으면 고장 때 MCU 정격을 넘을 수 있다.** 네 핀 모두 ADC 채널은 맞다(C 절). 그러나 DRV8244 의 IPROPI 는 내부에서 **4.5–5.5 V(`VIPROPI_LIM`)로 클램프**되고, 출력이 접지로 단락되면(고압측 과전류) FET 가 꺼진 뒤에도 IPROPI 를 이 전압까지 끌어올린 채 둔다(SLVSG24C 7.5.8, 8.3.4). STM32H743 쪽은 PC4 가 `TT_a`, PB1 이 `TT_u` 로 **3.3 V 내성·절대최대 4.0 V** 다(DS12110 표 9·21). PA6·PC0 는 `FT_a` 지만 ADC 측정 범위는 VREF+ 까지이고, 아날로그 입력으로 쓸 때 5 V 를 견딘다는 보장을 이 데이터시트 표에서 찾지 못했다. 정상 동작에서도 R_IPROPI = 3.6 kΩ 이면 대표 이득(4,750)으로 약 4.4 A 에서 3.3 V 에 닿는다. 저항을 1.5 kΩ 로 바꾸는 것은 측정 범위 문제만 풀고 **고장 전압 문제는 풀지 않는다**. ADC 앞에 클램프(예: 3.3 V 쪽 쇼트키 + 직렬저항) 또는 분압이 있는지 메인보드 회로에서 확인해야 한다.

### B. 동작은 하지만 고치거나 확인할 것

**B1. Wi-Fi UART 가 보드의 USB-C 단자와 같은 핀(PA11·PA12)이다.** AF 로는 `UART4_RX/TX`(AF6)라 맞지만, WeAct 보드에서 PA11·PA12 는 USB-C 의 D−·D+ 에 바로 연결돼 있다. Wi-Fi 모듈을 달면 USB 를 쓸 수 없고(USB DFU 부트로더 포함), USB 케이블과 모듈 출력이 같은 선에서 맞부딪칠 수 있다. 헤더에 나와 있고 보드 부품과 안 겹치는 UART 후보는 **UART7 = PE7(RX, AF7)·PE8(TX, AF7)**, USART1 = PA9(TX)·PA10(RX) 등이다(후보일 뿐).

**B2. KTH_MISO 의 PC2_C 는 아날로그 스위치를 닫아야 디지털로 쓸 수 있다.** LQFP100 의 PC2_C 는 PC2 와 PC2_C 가 내부 아날로그 스위치로 이어진 패드이고(RM0433 그림 76), 스위치 기본 상태는 `SYSCFG_PMCR.PC2SO` 리셋값에 따른다. CubeMX 가 `SPI2_MISO`(AF5)를 허용하긴 하지만, 펌웨어에서 `HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE)` 로 닫는 것을 확인하는 항목으로 둔다. SPI2_MISO 를 낼 수 있는 다른 핀은 PB14(지금 MTR_M_nFAULT)뿐이다.

**B3. 보드의 QSPI 플래시와 핀 네 개를 같이 쓴다.** WeAct 보드에는 QSPI 플래시(W25Q64)가 33 Ω 직렬저항을 거쳐 PE2(IO2)·PD11(IO0)·PD12(IO1)·PD13(IO3)에 달려 있다. 초안은 이 넷을 MTR_L_DIR·R_ENC_IN2·L_ENC_IN1·MTR_M_DRVOFF 로 쓴다. 플래시 nCS(PB6)에 **100 kΩ 풀업(R36)** 이 있어 PB6 을 건드리지 않으면 플래시 출력은 Hi-Z 로 남으므로 동작은 한다. 펌웨어는 **PB6 을 출력 high 로 고정**(또는 입력으로 두고 손대지 않기)하고 QSPI 를 켜지 않는다. 엔코더 선에 플래시 입력 용량이 붙는 것은 1,024 펄스 규모에서는 문제가 아닐 것으로 본다(미측정).

**B4. 모터 SPI MISO(PB4)를 보드의 SPI 플래시와 같이 쓴다.** 보드의 두 번째 W25Q64 는 SPI1(PB3 SCK·PB4 MISO·PD7 MOSI)에 있고 nCS 는 솔더 브리지 SB3 를 거쳐 PD6 에 가며 **100 kΩ 풀업(R40)** 이 있다. 이 플래시가 선택되지 않는 한 PB4 를 몰지 않으므로 **출력 충돌은 없다**. 다만 **전압 조건**이 있다: DRV8244 SPI 변형의 SDO 출력 레벨은 nSLEEP 핀 전압을 따른다(SLVSG24C 표 6-3, nSLEEP "Also VIO logic level for SDO"). nSLEEP 을 5 V 로 묶으면 SDO 가 5 V 로 PB4 를 몰고, 같은 선에 달린 3.3 V W25Q64 의 입력 절대최대(VCC + 0.4 V = 3.7 V)를 넘는다. **nSLEEP 이 3.3 V 여도 충분조건이 아니다** — SPI 변형의 무부하 SDO 출력 high 는 nSLEEP = 3.3 V 에서 **최대 3.8 V** 로 규정돼 있다(SLVSG24C 전기 특성 `VOH_SDO_NL`). 실제로 넘는다는 뜻은 아니지만 보장되지 않으므로, 레벨 변환을 넣거나 플래시를 떼어 낸 뒤에 공유 가능으로 본다(직렬저항만으로는 고임피던스 입력에서 전압이 거의 떨어지지 않아 해결이 안 된다). PD6·PB3·PD7 은 다른 용도로 쓰지 않는다.

**B5. 보드 LED 가 MTR_R_DIR(PE3)에 달려 있다.** PE3 은 디지털 트랜지스터(PDTC114ET, 입력 10 kΩ)를 거쳐 파란 LED 를 켠다. 부하는 작아 방향 신호에는 지장이 없지만 LED 가 오른쪽 바퀴 방향을 따라 켜지고 꺼져 **상태 LED 로 쓸 수 없다**. 상태 표시가 필요하면 다른 핀을 잡아야 한다.

**B6. PWM 네 개가 TIM1 채널 1–4 에 한 벌로 맞는다** — PE9 `TIM1_CH1`, PE11 `TIM1_CH2`, PE13 `TIM1_CH3`, PE14 `TIM1_CH4`(모두 AF1). 같은 타이머라 주기가 같고 ADC 트리거를 TIM1 에서 한 번에 낼 수 있어 좋은 구성이다. 다만 WeAct 의 0.96″ LCD 를 꽂으면 PE11(LCD_CS)·PE13(LCD_WR_RS)·PE14(LCD_SDA)·PE10(LCD_LED)·PE12(LCD_SCL)와 겹치므로 **LCD 는 달지 않는다**. DRV8244 SPI 변형의 기본 모드(`CONFIG3.S_MODE = 00b`)가 PH/EN(EN/IN1 = PWM, PH/IN2 = 방향)이라 DIR+PWM 배선과 맞는다. 7주차 회로의 신호 이름이 IN1·IN2 이므로 IN1 = PWM, IN2 = DIR 로 이어졌는지 메인보드에서 확인한다.

**B7. IMU 인터럽트 PD4 는 SD 카드 감지(SB2)와 이어질 수 있다.** 보드의 MicroSD 카드 감지 스위치가 솔더 브리지 SB2 를 거쳐 PD4 에 간다. SB2 가 열려 있으면(회로도 기호상 기본 개방으로 보임) 무관하고, 닫혀 있으면 MU_INT 와 충돌한다. 실물에서 SB2 를 확인한다.

**B8. DRVOFF 와 무선 비상정지.** 7주차 자료는 DRVOFF 를 무선 비상정지와 이을 계획이다. 초안은 DRVOFF 를 MCU GPIO 가 직접 몬다. 비상정지 회로도 같은 선을 몰려면 MCU 출력과 맞부딪치지 않게 OR 구조(예: 비상정지가 high 로 끌어올리면 MCU 와 관계없이 off)가 필요하다. DRV8244 의 DRVOFF 는 **내부 풀업**이고 high = 출력 끔이라, MCU 가 리셋돼 핀이 떠도 출력이 꺼지는 쪽이다(SLVSG24C 8.3.2). 보조 경로 후보로 TIM1 브레이크 입력(`TIM1_BKIN` = PE15, AF1, 헤더에 있고 비어 있음)이 있지만 **그것만으로 출력 차단이 되지는 않는다**. 브레이크 뒤 PWM 핀 상태는 `OSSI`·`OISx` 설정에 따르고, `AOE = 1` 이면 입력이 풀릴 때 PWM 이 저절로 다시 나간다(RM0433 TIM1 break 기능). 또 PH/EN 모드에서 EN = 0 은 Hi-Z 가 아니라 **OUT1 = OUT2 = H 인 ACTIVE 제동 상태**다(SLVSG24C 표 8-4, DRVOFF = 0 일 때) — 바퀴를 묶는 제동이지 드라이버 출력 차단이 아니다. BKIN 을 쓴다면 EN 의 정지 레벨을 OIS 로 정하고 `AOE = 0`(펌웨어가 명시적으로 재무장할 때만 `MOE` 를 다시 켬)로 두며, 출력 차단 자체는 DRVOFF 로 한다. PE15 를 BKIN 으로 쓰면 EXTI15 는 쓰지 않으므로 PB15(nFAULT 후보)의 EXTI15 와 충돌하지 않는다.

**B9. VREF+ 는 외부 기준 모드여야 한다.** WeAct V1.2 는 R9(0 Ω)로 VREF+ 를 VDDA 에 바로 잇는다. 이 상태에서 내부 VREFBUF 출력을 켜면 안 되므로, CubeMX 에서 VREFBUF 를 외부 기준(`ExternalMode`) 그대로 두는지 확인한다(WeAct 의 ADC 예제 `.ioc` 도 `ExternalMode`). 그림의 VREF+ 핀이 초록색인 것만으로는 어느 모드인지 알 수 없다.

### C. 확인만 하면 되는 것

- **IPROPI 네 개는 모두 ADC 입력이다**(전압 보호는 A6): PA6 `ADC12_INP3`, PC4 `ADC12_INP4`, PB1 `ADC12_INP5`, PC0 `ADC123_INP10`. ADC1·ADC2 로 나눠 TIM1 트리거(TRGO/TRGO2)에 맞춰 샘플할 수 있다. 메인보드 부품 DRV8244SQRYJRQ1 은 VQFN-HR 이라 **AIPROPI = 4,750 A/A(대표)** 다(SLVSG24C 7.5.8; HVSSOP 는 4,570). R_IPROPI 는 아직 3.6 kΩ 이고 1.5 kΩ 재선정은 예정일 뿐이므로 펌웨어는 저항값을 설정으로 받는다.
- **모터 SPI 핀은 SPI1 한 벌로 맞다**: PA5 `SPI1_SCK`, PA7 `SPI1_MOSI`, PB4 `SPI1_MISO`(모두 AF5). **KTH SPI 는 SPI2 한 벌로 맞다**: PB13 `SPI2_SCK`, PC1 `SPI2_MOSI`, PC2_C `SPI2_MISO`(AF5, B2 주의).
- **SDMMC1 여섯 핀은 보드의 MicroSD 슬롯 배선과 같다.** 데이터·CMD 에 47 kΩ 풀업이 이미 있다.
- **SWD(PA13·PA14), BOOT0, NRST, HSE 25 MHz(PH0·PH1), LSE(PC14·PC15), 키 K1(PC13)은 초안과 겹치지 않는다.** PB3(SWO)도 비어 있다.
- **초안의 모든 신호 핀이 WeAct 보드의 2×22 헤더 두 줄(P1·P2)에 나와 있다.** 헤더에 없는 것은 PA13·PA14(별도 SW 헤더)·크리스털 핀뿐이다.
- 카메라(DCMI) 커넥터에 가는 핀(PA4·PA6·PB7·PC6·PC7·PD3·PE0·PE1·PE4·PE5·PE6, PA7 은 SB1 경유)은 카메라를 꽂지 않으면 그냥 배선이다.
- **초안에 없는 신호**(필요하면 잡을 자리): 배터리 전압 감지(PC5 `ADC12_INP8` 가 비어 있음), 비상정지 입력(PE15 `TIM1_BKIN` 후보), 상태 LED(B5), Jetson 연결 UART(아래 4절).

## 4. Jetson 연결

초안에는 Jetson 과 이을 신호가 없다(Wi-Fi UART 만 있음). 상위 제어와의 명령·피드백 경로는 다음 중 하나가 필요하다: USB-C 의 USB FS 를 CDC 로 쓰기(이 경우 B1 대로 Wi-Fi 를 옮겨야 한다), 또는 별도 UART(예: USART1 PA9·PA10, UART7 PE7·PE8). 펌웨어 설계는 전송 경로와 무관한 바이트 프레임으로 잡는다(펌웨어 설계 계획서 참조).

## 5. 팀에 물을 것

1. PB10 은 R_ENC_IN1(LPTIM2_IN1) 자리가 맞나? 바퀴 엔코더를 LPTIM1·LPTIM2 로 세는 것이 의도인가?
2. nFAULT 넷(PB0·PB11·PB12·PB14·PB15 중)의 채널 배정은? H 는 어디인가?
3. IMU 품번과 버스(SPI1·SPI2·별도)는?
4. DRV8244 nSLEEP 은 메인보드에서 어디에(몇 V 에) 묶였나? IN1 = PWM, IN2 = DIR 이 맞나?
5. IPROPI 와 ADC 사이에 클램프·분압이 있나(A6)?
6. Wi-Fi 를 USB 핀에서 옮길 수 있나? Jetson 은 USB CDC 와 UART 중 무엇으로 붙이나?
7. 무선 비상정지가 DRVOFF 를 직접 끄는 구조인가, MCU 입력으로 들어오는가?

## 6. 이 검토가 하지 않은 것

전기적 타이밍(LVDS 경유 KTH SPI 최대 클럭, 엔코더 차동 수신 후 상승시간), 전원·발열, PCB 배선은 보지 않았다. AF 와 보드 연결은 데이터 파일과 회로도로만 확인했고 실물 보드에서 재지 않았다.
