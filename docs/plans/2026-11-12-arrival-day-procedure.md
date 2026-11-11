# 도착일 절차서 — S0 첫 굽기부터 S5 안전 동작까지

작성일: 2026-11-12 · 상태: **절차 초안, 실기 전**(부품 도착 11-20) · 관련: [실차 이식 계획](2026-11-11-sim-to-real-bringup.md) 5절(단계 표 정본), [보드 설계](../design/2026-11-12-weact-h743-board-firmware.md), [모터 출력 설계](../design/2026-11-12-weact-h743-motor-output.md), bring-up 도구 `tools/bringup.py`(주인: 콘솔 세션), udev `deploy/udev/README.md`

이 문서는 계획 5절의 S0–S5 를 **칠 명령** 단위로 풀어 쓴다. 단계의 합격·멈춤 조건의 정본은 계획 5절이다. 여기 적은 판정 필드는 그 조건을 도구 출력에서 읽는 방법이다. 명령·옵션은 콘솔 세션과 맞췄다(2026-11-12).

## 0. 공통 규칙

- **멈춤**: 단계의 멈춤 조건이 하나라도 생기면 그 단계에서 멈추고, 원인과 기록 경로를 남긴 뒤 총괄에 올린다. 다음 단계로 넘어가지 않는다.
- **결정 경계** — 아래는 이 절차가 대신 넘지 않는다.
  - 모터 전원(VM)은 IPROPI 보호 확인 전에는 S2 의 VM 투입 순서로만 넣는다(사용자 결정 2026-11-11).
  - 출력이 열린 빌드는 **사용자가 쓰고 커밋한 보호 확인 기록**이 있어야 구성된다(모터 출력 설계 2절). 그 기록을 쓰는 것이 사용자 결정(Q6)이고, 에이전트는 만들지 않는다.
  - 지금 출력을 열 수 있는 빌드는 **시운전(`commission`) 하나뿐**이다. 지금 판(회로도 V1.0)에는 비상정지 입력이 없고, **다음 PCB 판에 커넥터를 더한다**(사용자 결정 2026-11-12). 그래서 출력이 열린 정상 빌드, **S4, S5 의 정상 빌드 시나리오(`tx_cut`·`disarm`·`expiry`), S6 텔레옵은 다음 판을 기다린다**(8절, 9.2, 11절).
  - MCU 와 무관한 하드웨어 차단은 **사람이 손에 든 주 전원 차단 스위치**로 대신한다(사용자 결정 2026-11-12, 다음 PCB 때 재검토). 출력이 열린 실행에서는 늘 그 스위치를 손에 쥔다.
- **기록**: 실행마다 `artifacts/<UTC 시각>_<단계>_<구분>/` 를 새로 만든다. 예: `artifacts/20261120T093000Z_s3_commission_L_01/`. 덮어쓰지 않는다.
  - 구동 명령은 결과를 `run.jsonl` 의 `{"t":"result"}` 줄과 stdout JSON 두 곳에 남긴다.
  - `cpr`·`analyze` 는 stdout 에만 남기므로 `| tee result.json` 을 붙인다.
- **장치 경로**: udev 규칙 설치(3절) 뒤에는 `--device /dev/forklift-mcu`(USB CDC, `--baud` 없음). UART7 대안은 USB-UART 어댑터 경로에 `--baud 460800`.
- **`--allow-locked` 는 실기에서 쓰지 않는다**(모형·HIL 전용).
- 공통 판정: 구동 명령은 `result == "ok"`, `aborted == null`, `cleanup_confirmed == true` 셋이 모두 서야 한다. 예외는 일부러 리셋·전원 차단을 거는 S5 실행뿐이다(9.1 — 중단이 정상, 외부 측정으로 판정).

## 1. 준비물

**모든 단계의 몸·차체 안전**(agy 검토):
- 차체는 받침대에 올린 뒤 스트랩·클램프로 묶는다. 바퀴와 주변 사이 틈을 눈으로 확인한다.
- 승강 축을 다루는 동안 포크 아래에 낙하 방지 블록을 둔다. 출력이 꺼지면(차단·SLEEP) 승강 축이 하중을 붙잡지 못할 수 있다 — 코어 설계 4절. 구동 중에는 포크 가동 범위에 손을 넣지 않는다.
- WeAct 를 꽂고 빼는 것은 **모든 전원(벤치·배터리·USB)을 끈 뒤**에만 한다.

| 도구 | 쓰는 단계 | 없으면 |
|---|---|---|
| 노트북: 저장소, Arm GNU Toolchain 13.3.rel1, cmake, STM32CubeProgrammer CLI(`STM32_Programmer_CLI`, 2.21 이상) | 전부 | 진행 불가 |
| USB-C 케이블(데이터). S1 에서 역전원이 보이면 VBUS 를 끊은 케이블 | S0–S5 | — |
| USB-UART 어댑터(3.3 V) — **TX·RX·GND 세 선만**(어댑터의 DTR·RTS 핀은 포트를 열 때 실제로 움직인다) | UART7 대안 | USB 만 |
| 멀티미터 | S1·S2 | 진행 불가 |
| 전류 제한 벤치 전원(0.3–0.5 A 한계) | S2 VM 투입 | 진행 불가 |
| 오실로스코프 2채널 이상(대역 25 MHz 이상, 10× 프로브), 전류 프로브 또는 션트 — 사용자 보유(2026-11-12), 종류·채널 수는 S0 에서 확인 | S2 PB4, S3 전원 강하, S5 IWDG ② | 예비 분기: S2 는 6.1 의 6번, 나머지는 그 측정만 미룬다(S5 는 ①만) |
| 받침대(바퀴 띄움)와 차체를 받침대에 묶는 래칫 스트랩·클램프 | S3–S5 | 진행 불가 |
| 손에 쥐는 주 전원 차단 스위치 — **DC 전압·전류 정격**이 배터리 전압·모터 전류 이상인 것. MCU 와 무관한 유일한 차단 경로다(0절) | S3–S5 | 진행 불가 |
| 승강 낙하 방지 블록(포크 아래 받침) | S2–S5 | 승강 축 시험을 하지 않는다 |
| 외부 리셋 버튼: WeAct 헤더 U2 9번(NRST)과 GND 사이에 선으로 뺀 누름 버튼 | S5 ② | 보드 위 버튼을 구동 중에 손으로 누르지 않는다 — ②를 미룬다 |
| 줄자·마킹 테이프, 스톱워치 또는 영상 | S2 손 회전, S5 정지 거리 | 영상으로 대신 |

## 2. 펌웨어 빌드

저장소 루트에서. `<TC>` 는 Arm GNU Toolchain 13.3.rel1 설치 디렉터리다(툴체인 파일은 절대 경로로 준다).

```bash
build_fw() {  # build_fw <이름> [cmake 옵션…]
  cmake -S firmware -B build/fw-$1 -DFW_BOARD=weact_h743 \
    -DCMAKE_TOOLCHAIN_FILE=$PWD/firmware/boards/weact_h743/arm-gnu-13.3.cmake \
    -DARM_GNU_TOOLCHAIN_DIR=<TC> -DFW_PYTHON=$(command -v python) "${@:2}" &&
  cmake --build build/fw-$1 -j
}
build_fw normal                                  # S0·S2: 출력 잠금 정상 빌드
build_fw commission -DFW_BUILD=commission        # S2: 출력 잠금 시운전 빌드(재무장 보류 확인)
build_fw stall -DFW_TEST_IWDG_STALL=5            # S5 ①: 부팅 5 s 뒤 주기 정지, 출력 잠금 강제
# S3·S5 ②: 출력 열린 시운전 빌드 — 사용자가 보호 확인 기록을 커밋한 뒤에만 구성된다
build_fw open -DFW_BUILD=commission -DFW_MOTOR_OUTPUT=1 \
  -DFW_IPROPI_PROTECTION_RECORD=<커밋된 기록의 저장소 상대 경로>
```

- ELF: `build/fw-<이름>/boards/weact_h743/fw.elf`(stall 은 `fw_iwdg_stall.elf`).
- 굽기 전 정체 확인: `strings build/fw-<이름>/boards/weact_h743/fw*.elf | grep '^forklift-mcu'`. 빌드 종류, `locked` 또는 `OPEN … rec=<기록 해시>`, git 커밋, `link=usb` 가 맞는지 본다.
- 크로스 시험(로컬 전용): `python -m pytest tests/integration/test_firmware_cross.py -q`.

## 3. S0 — 첫 굽기와 옵션 바이트 (메인보드에서 뺀 WeAct 만)

**WeAct 를 메인보드에서 물리적으로 뺀 상태에서만** 한다(보드 설계 8절: ROM 부트로더가 메인보드 신호 핀을 몰 수 있고, USB VBUS 가 메인보드 5 V 로 갈 수 있다).

0. **준비물 점검**: 오실로스코프의 채널 수와 대역을 확인해 기록한다(1절 표의 2채널 이상·25 MHz 이상). 모자라면 S2 PB4 와 S5 ②는 1절의 예비 분기를 따른다.
1. BOOT0 버튼을 누른 채 RESET → BOOT0 를 놓는다. DFU 장치를 확인한다: `lsusb | grep 0483:df11`, 그리고 `STM32_Programmer_CLI -l usb` 로 이 보드의 `usbN` 번호를 확인한다(DFU 장치가 여럿이면 다른 보드를 지울 수 있다). 아래 `usb1` 은 그 번호로 바꾼다. DFU 의 udev 권한이 없으면 사용자가 sudo 로 규칙을 넣는다.
2. 옵션 바이트 읽기(쓰지 않는다):
   ```bash
   D=artifacts/20261120T090000Z_s0_ob; mkdir -p $D
   STM32_Programmer_CLI -c port=usb1 -ob displ > $D/ob.txt 2>&1; echo "exit=$?"   # 0 이어야 한다
   ```
   RDP Level 0, `IWDG1_SW`, BOR 수준, BOOT_ADD0, 부트로더 버전을 기록한다. 바꿔야 하면 멈추고 올린다(옵션 바이트는 되돌리기 어려워 펌웨어가 쓰지 않는다 — 보드 설계 3.5).
3. 공장 펌웨어 백업 → **백업이 성공했을 때만** 전체 소거:
   ```bash
   STM32_Programmer_CLI -c port=usb1 -u 0x08000000 0x200000 $D/factory.bin && \
     [ "$(stat -c %s $D/factory.bin)" = 2097152 ] && \
     STM32_Programmer_CLI -c port=usb1 -e all; echo "exit=$?"   # 0 이어야 한다
   ```
   내부 플래시를 지워도 ROM DFU 는 남는다. 백업·소거가 실패하면 멈춘다(H74x 부트로더 일부 판은 DFU 전체 소거에 제한이 있다 — AN2606. 부트로더 버전을 기록에 남긴다).
4. 굽기: `STM32_Programmer_CLI -c port=usb1 -w build/fw-normal/boards/weact_h743/fw.elf -v` → BOOT0 를 누르지 않고 RESET.
5. **합격**:
   - 파란 LED(PE3)가 1 Hz 로 깜빡인다.
   - `lsusb | grep 0483:5740` 이 보인다.
   - 제품 문자열이 2절의 정체와 같다: `udevadm info -a -n /dev/ttyACM0 | grep -m1 'ATTRS{product}'`.
6. **멈춤**:
   - DFU 가 열거되지 않음, 백업·소거 실패, `-v` 검증 실패.
   - LED 가 짧게 n 번 깜빡이고 쉼을 되풀이: 부팅 실패 코드.
     - 2–6: 전압·클록 단계
     - 7: RNG
     - 8: 보드 설정
     - 9: 코어 초기화
     - 10: 엔코더 타이머
     - 11: `DBG_IWDG1` 이 지워지지 않음
   - 어느 경우든 IWDG 가 약 50 ms 마다 다시 시작시킨다.

### 3.1 이미지 바꾸기 (S2 이후, 굽는 이미지가 바뀔 때마다)

S2 부터는 단계마다 다른 이미지를 쓴다(`fw-normal`·`fw-commission`·`fw-open`·`fw-stall`). 바꿀 때마다:

1. 모든 전원(배터리·벤치·USB)을 끈다.
2. WeAct 를 메인보드에서 뺀다(3절의 이유와 같음).
3. DFU 로 들어간다(3절 1번, `usbN` 확인).
4. 굽는다: `STM32_Programmer_CLI -c port=usbN -w build/fw-<이름>/boards/weact_h743/<fw 또는 fw_iwdg_stall>.elf -v`
5. 전원을 끈 채 WeAct 를 다시 꽂는다.
6. 전원을 넣고 실제 제품 문자열이 그 이미지의 정체와 같은지 확인한다: `udevadm info -a -n /dev/forklift-mcu | grep -m1 'ATTRS{product}'`. 다르면 멈춘다.

## 4. udev 규칙 (사용자가 sudo 로, 한 번)

```bash
sudo install -m 644 deploy/udev/99-forklift-mcu.rules /etc/udev/rules.d/ && \
  sudo udevadm control --reload && sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb
ls -l /dev/forklift-mcu
udevadm info -q property /dev/forklift-mcu | grep ID_MM_DEVICE_IGNORE   # =1 이어야 한다
```

규칙은 0483:5740 과 제품 문자열 `forklift-mcu *` 로 고른다. 보드가 여럿이면 `/dev/forklift-mcu-<UID>` 를 쓴다. 호스트 도구는 DTR 을 내리지 않고, 열 때마다 첫 프레임 앞에 0x00 을 보낸다(#462).

## 5. S1 — 전원·배선 (모터 전원 경로를 뺀 채)

계획 5절 S1 행이 정본이다. 이 단계에는 펌웨어 명령이 없다. 측정 순서는 이렇다.

1. 모터 전원 경로를 뺀다. 배터리 → F1 → LM74801 → VCC·5 V·**3.3 V**·19 V 레일을 무부하·부하로 잰다. 각 레일 공칭 ±5 % 안이어야 한다.
2. Jetson 을 19 V 로 켜고 MAXN SUPER 로 둔 채 **최대 부하를 실제로 건다**(CPU·GPU 를 함께 쓰는 부하 프로그램 — 모드만 바꿔 놓고 쉬는 것은 최대 부하가 아니다). 10 분 동안 19 V 와 Jetson 이 안정한지 본다.
3. **역전원**: WeAct 를 메인보드에 꽂고 USB-C 를 연결한 채 주 전원을 끈다. 메인보드 5 V 레일에 전압이 생기면 역전원이다(보드 설계 3.1). 그러면 VBUS 를 끊은 케이블이나 UART7 을 쓴다.
4. **멈춤**: 어느 레일이든 정격 밖, 발열, 역전원.

## 6. S2 — 링크·센서, 모터 분리 (출력 잠금 빌드)

### 6.1 VM 투입 순서 (사용자 결정 2026-11-11, 보호 확인 전에도 이 순서로만)

1. 출력 잠금 정상 빌드(`fw-normal`)가 올라가 있는지 확인한다(S0 에서 구운 것, 아니면 3.1 로 바꾼다).
2. 모터를 뗀다.
3. 벤치 전원 전류 한계를 0.3–0.5 A 로 둔다. 벤치 전원의 GND(−)를 메인보드 GND 에 직접 잇고, 멀티미터 도통으로 확인한다(공통 접지).
4. MCU 를 꽂기 **전에** VM 을 넣고 MCU 핀 전압을 멀티미터로 잰다. PB4 를 뺀 아래 핀 중 3.3 V 를 넘는 것이 있으면 멈춘다.
   - IPROPI(PA6·PC4·PB1·PC0)
   - nFAULT(PB11·PB12·PB14·PB15)
   - KTH·엔코더 입력
   - PB4 는 이 단계에서 판정하지 않는다: MCU 가 없으면 nSLEEP 이 내려가 있고 nSCS 가 high 라 SDO 는 Hi-Z 이므로, 멀티미터 값은 SDO high 전압이 아니다(Codex).
   - 이 무부하 측정은 단락·레벨 확인일 뿐이다. 모터가 없으면 IPROPI 는 0 이라, 부하 전류에서 보호 회로가 맞는지는 보여 주지 않는다(agy). 그 확인은 7절 선행 조건의 보호 확인 기록이 맡는다.
5. **모든 전원을 끄고**(벤치 전원·USB), 레일이 내려간 것을 확인한 뒤 MCU 를 꽂는다. 6번의 오실로스코프를 먼저 걸어 두고, 그다음 전원(VM 포함)을 넣고 부팅한다. 오실로스코프가 없으면 6번의 분기를 먼저 따른다.
6. **PB4 — SDO high 전압**(사용자 결정 2026-11-12: 플래시를 둔 채 잰다). DRV8244S 의 SDO high 는 nSLEEP 전압을 따라 nSLEEP = 3.3 V 에서 부하 없이 최대 3.8 V 다(SLVSG24C 7.5.2 `VOH_SDO_NL`). 출력 잠금 빌드도 부팅하면 드라이버를 깨우고(DRVOFF high 라 출력은 꺼진 채) 20 ms 마다 상태를 SPI 로 읽는다. 그래서 **이 부팅이 곧 WeAct SPI 플래시의 첫 노출**이다. 사용자는 이 위험을 알고 감수했다.
   - **측정**: PB4 를 WeAct 헤더에서 오실로스코프로 본다(대역 25 MHz 이상, 10× 프로브, 짧은 접지). SCK 는 6.25 MHz 이고, 거래 하나는 약 2.6 µs 다.
     - 정상 폴링은 L→R→M→H 를 5 ms 간격으로 돈다(`firmware/core/src/drive.c`). 그래서 한 드라이버는 20 ms 마다 한 번 읽힌다. 한 nCS 로만 트리거하면 그 채널만 보이고 나머지 셋의 과전압을 놓친다(Codex).
     - 첫 단발: 트리거를 PA0(L 채널 nCS) 하강에 걸고 부팅해, **드라이버를 깨운 뒤 L 의 첫 SPI 거래**를 잡는다.
     - 이어서 트리거를 PA0·PA1·PA2·PA3 하강으로 차례로 바꾸고, 채널마다 피크 검출(또는 잔상) 모드로 1–2 s 를 쌓아 그 드라이버의 high 최고값을 읽는다. 네 값 가운데 가장 큰 것이 판정값이다.
     - 멀티미터 평균값은 쓰지 않는다.
   - **판정**: 최고값이 3.6 V 이하이면 플래시를 두고 계속한다. 3.6 V 를 넘으면 **바로 모든 전원을 끄고** 플래시를 뗀 뒤 5번부터 다시 한다. 3.6 V 이하라도 플래시가 안전하다는 보장은 아니다.
   - **기록(노출 사실)**: `artifacts/<시각>_s2_pb4_01/` 에 남기고 총괄에 보고한다.
     - 날짜·시각
     - 전원을 넣은 때부터 판정까지 걸린 시간
     - 채널별 high 최고값과 오실로스코프 화면
     - 플래시 실장 여부와 판정 결과
   - **오실로스코프가 없으면**: 플래시를 단 채로는 부팅하지 않는다. 멀티미터로는 SDO high 를 판정할 수 없으므로, 그렇게 부팅하면 노출만 있고 판정은 없다. 오실로스코프를 구할 때까지 5번을 미룬다. 미룰 수 없으면, 플래시를 먼저 떼고 진행할지를 총괄에 올린다(사용자 결정).

STATE 에서 `flags` 에 `outputs_locked`, 그리고 `drivers_ready` 를 확인한다.

### 6.2 기록과 손 회전

```bash
D=artifacts/20261120T100000Z_s2_record_01; mkdir -p $D
python tools/bringup.py record --device /dev/forklift-mcu --out $D/run.jsonl --seconds 120 --markers
```

- 바퀴·조향·승강을 손으로 돌리고 Enter 로 사건을 표시한다. 모터는 6.1 대로 떼어 둔 상태다. 모터가 드라이버에 이어진 채 돌리면 역기전력이 VM 으로 올라올 수 있으니 천천히 돌리고 VM 을 지켜본다.
- 판정은 사람이 `run.jsonl` 의 `{"t":"state"}` 줄로 한다.
  - `mode`·`fault_flags`·`reset_cause`
  - `wheel_count_left/right` 가 회전과 비례하는지
  - `steering_rad`·`lift_m` 이 손 회전을 따라가는지
- 조향 끝에서 끝까지 KTH 각이 한 바퀴 안에 드는지, **넘김 지점이 사용 범위 밖에 있는지** 보고, 직진 영점을 기록한다(계획 S2).
- **KTH CRC 변형 확정**(계획 S2, mcu-link/3 계약 3.3): 위 기록의 STATE 에 축별 KTH 계수 셋(받은 프레임, 설정 변형 CRC 오류, 다른 변형이었다면 났을 오류)이 실린다. 펌웨어의 설정 변형은 두 축 모두 `g704_bytes` 다(`board_config.c`). 기록 동안 **조향·승강을 범위 안에서 천천히 끝까지 움직인다** — 두 변형은 4096 개 각도 중 256 개에서 CRC 가 같으므로, 축이 멈춰 있으면 판정이 나지 않는다.
  ```bash
  python tools/bringup.py kth --record $D/run.jsonl --configured-m g704_bytes --configured-h g704_bytes \
    --image "<3.1 의 udevadm 으로 읽은 제품 문자열>" | tee $D/kth.json
  ```
  - **합격**: 두 축 모두 `configured_ok`(신선 프레임 1000 이상, 설정 변형 오류 0, 다른 변형 오류 100 이상, 마지막 STATE 에서 축이 유효하고 범위 안).
  - `other_variant` 이면 멈추고, `firmware/boards/weact_h743/src/board_config.c` 에서 그 축의 `.crc` 를 다른 변형(`KTH_CRC_REFLECTED_MSB_FIRST`)으로 바꾼다. 빌드 옵션이 아니라 코드 변경이므로 커밋한다. 그 이미지를 3.1 로 구워 다시 하고, `--configured-*` 도 바꿔 준다.
  - `undecided` 는 이유를 읽는다. 축을 덜 움직였으면 다시 기록하고, 두 변형 모두 오류가 나면 배선·잡음을 보고 멈춘다.
  - 승강 KTH 가 없거나 무효면 그 축은 판정하지 않고 기록한다(승강 방식 결정 대기, 계획 3.5).
  - `hil_plant` 빌드의 기록은 합성이라 이 확정에 쓰지 않는다.

### 6.3 감속비 G

```bash
D=artifacts/20261120T103000Z_s2_cpr_01; mkdir -p $D
python tools/bringup.py cpr --device /dev/forklift-mcu --out $D/run.jsonl --seconds 120 \
  --turns-left 10 --turns-right 10 --dir-left 1 --dir-right 1 | tee $D/result.json
```

- 바퀴에 표시 테이프를 붙이고 **정확히 10 바퀴**를 센다. 다른 수를 돌렸으면 그 수를 `--turns-*` 에 넣는다 — 도구는 받은 수로 나누므로 틀린 수는 틀린 G 를 그대로 합격시킨다(Codex). `--dir` 의 1 은 차체 앞으로 구르는 방향이다.
- `--motor-counts-confirmed` 는 MT6701 의 ABZ 해상도(4,096)를 따로 확인한 뒤에만 붙인다. 확인 전에는 빼고 돌려 `cpr` 만 얻는다.
- **합격**: `valid == true`, `cpr` [좌, 우]. `gear` [좌, 우] 는 `--motor-counts-confirmed` 를 붙인 실행에서만 나온다.
  - 값은 `config/real/vehicle.yaml` 한 곳에 적는다.
- 무효면 `reason` 이 붙는다.

### 6.4 재무장 보류 확인 (출력 잠금 시운전 빌드, 센서 연결)

`fw-commission` 으로 바꾸고(3.1):

```bash
D=artifacts/20261120T110000Z_s2_rearm_hold_01; mkdir -p $D
python tools/rearm_hold_check.py --device /dev/forklift-mcu --out $D/run.jsonl --stop-grace-ms 300 --limit-s 2.0 \
  | tee $D/result.json
```

- **합격**: `result == "ok"`, 그리고 `checks` 의 모든 항목이 참이다.
  - `refused_after_disarm`·`refused_after_clear_faults`
  - `no_other_refusal`·`no_link_errors`·`no_state_gap`
  - `not_shortened`·`not_lengthened`
- 출력이 잠기지 않은 보드면 아무것도 보내지 않고 거부한다.

### 6.5 링크가 멈췄을 때

자동 리셋은 없다(총괄 결정). 복구 순서는 이렇다.

1. 주 전원을 다시 넣는다.
2. 링크를 다시 연다.
3. `STATE.reset_cause` 를 확인한다(전원이면 `POWER`).

USB 멈춤을 본 날짜·조건은 기록한다. 조건부 감시 리셋을 다시 검토하는 계기다.

**S2 멈춤**: SPI 응답 상위 비트가 11 이 아님, 센서 무효, 카운트가 회전과 비례하지 않음, 3.3 V 넘는 핀.

## 7. S3 — 축별 개루프 (출력 열린 시운전 빌드)

**선행 조건**(모두):
- IPROPI 보호 회로가 실장됐거나 최종 회로를 확인했다(계획 5절 선행 조건).
- 사용자가 보호 확인 기록을 쓰고 커밋했다.
  - 표지 줄: `IPROPI protection record: confirmed-by=…; date=YYYY-MM-DD; evidence=…`
- `fw-open` 이 구성·빌드되고, 3.1 로 구운 보드의 제품 문자열이 `forklift-mcu commission OPEN <git> rec=<해시>` 이다.
- 보호 확인 기록의 근거에는 저항 부하 등으로 작은 전류를 흘려 IPROPI 전압을 미리 잰 값을 넣기를 권한다(agy).
- **전원 바꿈**: 6.1 의 0.3–0.5 A 한계로는 모터 기동 전류에서 전압이 주저앉아 가짜 리셋·UVLO 가 난다(agy). 계획 S1 에서 확인한 배터리 경로로 바꾸거나, 벤치 전원을 계속 쓰면 올린 한계값과 이유를 기록한다.
- **배선 대조**: 각 축(L·R·M·H)의 모터·엔코더·KTH 커넥터가 그 축에 맞는지 도통으로 확인한다. 처음 구동은 도구가 허용하는 가장 짧은 시간(`--seconds 0.1`)으로 어느 축이 움직이는지부터 본다(agy).

바퀴를 띄우고, 사람이 주 전원 차단을 손에 쥔다. 실차 실행기는 이 보드를 시운전 빌드로 거부한다(정상).

```bash
D=artifacts/20261120T130000Z_s3_commission_L_01; mkdir -p $D
python tools/bringup.py commission --device /dev/forklift-mcu --out $D/run.jsonl \
  --axis L --duty 0.05 --seconds 0.5 --power-switch-in-hand
# R 도 같게. 조향(M)·승강(H)은 창을 반드시 주고 0.2 s 이하, 처음엔 범위 가운데에서:
python tools/bringup.py commission --device /dev/forklift-mcu --out $D/run_M.jsonl \
  --axis M --duty 0.05 --seconds 0.1 --window <LO> <HI> --power-switch-in-hand
```

- **명령 제약**
  - 듀티는 0 이 아니고 ±0.15 이하다. 0.15 를 넘으면 펌웨어가 거부하고 이전 명령을 유지한다.
  - 시간은 0.1–1.5 s.
  - `--observed` 는 주지 않는다: 실행 뒤 도구가 물으면 **실제로 본 방향**을 답한다(pos = 차체 앞·왼쪽·위). 미리 적어 두면 모터와 센서 극성이 함께 뒤집힌 경우를 놓친다(Codex).
- **합격**
  - 판정: `verdict.duty_sign_ok`·`verdict.sensor_sign_ok` 가 참이고, `metrics.valid` 가 참이다.
  - 극성: 틀리면 `verdict.suggested_duty_sign` 을 보드 설정의 `duty_sign` 에 넣는다.
  - 전류: `metrics.current_peak_ma`·`current_mean_ma` 가 벤치 전류계와 맞는다.
  - 범위 끝: 조향·승강은 기구 한계 전에 잰다.
- **무부하 최대 바퀴 속도**: 바퀴마다 허용 최대 듀티(0.15)에서 `metrics.rate_mean_rad_s` 를 기록한다(시운전 듀티 상한 안의 값 — 그 이상은 정상 빌드 결정 뒤).
- **전원 강하·회생 과전압(오실로스코프)**: 기동·역전·멈춤 때 VCC 가 LM74801 UVLO(10.41 V)·19 V 쪽 UVLO 에 걸리지 않아야 하고, 급감속·역전 때 VCC 상승이 레일 부품 정격 안이어야 한다. 오실로스코프가 없으면 이 항목은 **미완료**로 남는다 — S3 합격으로 치지 않는다.
- **멈춤**: 의도와 다른 방향, 과전류, 기구 한계 접촉, Jetson·MCU 리셋, `faults_seen` 에 예상 밖 결함.

## 8. S4 — 축별 폐루프

**다음 PCB 판을 기다린다.** `bringup.py step` 은 정상 빌드에 `DRIVE` 를 보낸다. 그런데 출력이 열린 정상 빌드는 비상정지 입력이 있는 다음 판에서만 연다(0절, 11절, 모터 출력 설계 2절). 그때 명령은 이렇다.

```bash
python tools/bringup.py step --device /dev/forklift-mcu --out $D/run.jsonl --wheel left --target 0.8
```

- 바퀴 반경 실측 전 상한은 1.0 rad/s, 계단은 최대 3 s 다.
- 기록할 값: `rise_s`·`overshoot_pct`·`settle_s`·`unsettled`·`gaps`.
- 계획 S4 의 조향 추종 응답과 엔코더 고장 주입(커넥터 뽑기·A/B 바꾸기)도 이때 한다 — 그 전까지 **미완료**.

## 9. S5 — 안전 동작

### 9.1 지금 할 수 있는 것

**시운전 상한 중 DISARM**(받침대, `fw-open`):

```bash
python tools/bringup.py commission --device /dev/forklift-mcu --out $D/run.jsonl --axis L --duty 0.15 \
  --scenario limit_disarm --limit-s 2.0 --stop-grace-ms 300 --power-switch-in-hand \
  --output-response-ms <X|unavailable> --stop-time-ms <X|unavailable> --stop-distance-mm <X|unavailable>
# hold_disarm 은 --scenario hold_disarm [--disarm-delay-ms 50]
```

- `result == "ok"` 는 DISARM 이 상한의 STOPPING 안에서 받아들여졌음이 입증됐다는 뜻이다. 그것만으로 합격이 아니다.
- **출력 반응은 받침대에서도 판정한다**: `external.output_response_ms.pass == true`(DISARM 문턱 100 ms). 정지 시간·거리만 받침대에서 참고값(`reference: "stand"`, `over_limit`)이다(Codex).
- `--stop-grace-ms` 는 빌드 값(300)을 정확히 넣는다.

**IWDG 리셋**(총괄 결정 (A), 모터 출력 설계 4.1):
- **① 주기 정지 → 리셋**(`fw-stall` 로 바꾸고 3.1, 출력 잠금):
  ```bash
  python tools/bringup.py record --device /dev/forklift-mcu --out $D/run.jsonl --seconds 30 --markers
  # 보드 전원을 넣은 직후, STATE 가 들어오기 시작하면 Enter 한 번(사건 표시 0번)
  python tools/bringup.py analyze --record $D/run.jsonl --marker 0 --kind iwdg | tee $D/result.json
  ```
  - 사건 번호는 0 부터 센다(첫 표시 0, 마지막 -1).
  - 표시 뒤 약 5 s 에 STATE 가 끊기고(`software.last_state_before_gap_ms`), 다음 부팅이 `new_boot_reset_cause == "IWDG"` 여야 한다.
  - `new_boot_ms` 는 USB 재열거·재연결 시간을 포함하므로 리셋 시각이 아니다. 주기 정지 → 리셋 시간은 IWDG 설정의 계산값 47.6–53.8 ms(LSI 범위, 보드 설계 3.5)로 잡는다.
- **② 리셋 → 차단 과도**(`fw-open`, 받침대, 1절의 외부 리셋 버튼 — 보드 위 버튼을 손으로 누르지 않는다):
  - 한 프로세스가 링크를 쥐고 구동하고, 같은 프로세스가 사건을 표시한다(#466):
    ```bash
    python tools/bringup.py commission --device /dev/forklift-mcu --out $D/run.jsonl --axis L --duty 0.05 \
      --seconds 1.5 --power-switch-in-hand --markers
    # 'yes' 확인 뒤, 바퀴가 도는 동안 외부 리셋 버튼을 누르는 순간 Enter(앞에 nrst 라고 적으면 그 이름) — 사건 0번
    python tools/bringup.py analyze --record $D/run.jsonl --kind iwdg --marker 0 | tee $D/result.json
    ```
  - 두 프로세스가 같은 tty 를 열면 수신을 나눠 읽으므로 `record` 를 함께 돌리지 않는다(Codex).
  - `analyze` 는 소프트웨어 시간축만 준다(`last_state_ms`·`last_state_before_gap_ms`). 판정은 아래 외부 측정으로 한다.
  - 리셋으로 USB 가 끊겨 표시 뒤 STATE 가 하나도 없으면, 두 시각이 모두 `null` 인데도 `analyze.result` 는 `"ok"` 다. 그때는 `run.jsonl` 원본에서 표시(`{"t":"marker"}`) 직전의 마지막 `{"t":"state"}` 줄을 직접 읽어 기록한다(Codex).
  - 오실로스코프로 잴 것: MTR_SLEEP·DRVOFF·EN·PH 핀, 모터 단자 전압, 모터 전류. 트리거는 NRST 하강.
  - 합격: 리셋 뒤 MTR_SLEEP 이 low 로 가서 전 채널이 SLEEP(출력 Hi-Z)이 되고, 모터 전류가 0 으로 간다. 리셋 순간의 반대 방향 펄스 유무와 그 길이를 기록한다.
  - 이 실행의 도구 결과는 **중단이 정상**이다. USB 가 끊기면 도구가 먼저 권한을 회수하므로 `aborted == "authorization_revoked"`, `result == "invalid_measurement"` 가 나온다. `metrics.reason` 은 재부팅을 봤는지에 따라 `end_unknown` 또는 `boot_changed` 다. 특정 조합을 기대하지 않는다(Codex). 0절의 공통 합격 조건은 여기 적용하지 않고, 합격은 위 외부 측정으로만 한다. 재부팅 확인은 `new_boot_reset_cause == "PIN"` 으로 한다. USB 재열거로 링크가 끊겨 새 부팅이 기록에 없으면, 리셋 뒤 따로 `record --seconds 10` 을 돌려 `reset_cause == "PIN"` 을 본다.
- **IWDG 판정**(계획 3.4: 주기 정지부터 출력 정지까지 100 ms): ①의 계산값 최대 53.8 ms + 주기 1 ms + ②에서 잰 리셋→차단 시간이 100 ms 안이어야 한다. 셋을 기록하고, 합이 100 ms 안이라는 근거가 없으면 판정을 보류한다. 한계: NRST 버튼과 IWDG 의 리셋 펄스 길이가 다르다(총괄 결정 (A)). 오실로스코프가 없으면 ②와 이 판정은 미룬다.
- NRST 를 GND 로 붙잡은 상태에서도 위 핀 전압을 잰다. 계산과 다르면 멈추고 올린다(모터 출력 설계 4.1).

**주 전원 차단**(`fw-open`, 받침대):
- ②와 같이 `commission --seconds 1.5 --markers` 로 한 바퀴를 돌린다. 차단 스위치를 내리는 순간 Enter(앞에 `power` 라고 적으면 그 이름)를 누른다.
- 이 실행도 **중단이 정상**이다(공통 합격 조건 적용 안 함, 도구 결과 조합은 ②처럼 고정되지 않는다). `analyze --record $D/run.jsonl --kind power --marker 0 | tee $D/result.json` 으로 표시 뒤 마지막 STATE 시각을 읽는다. 시각이 `null` 이면 ②처럼 원본에서 읽는다.
- **MCU 전원이 어디서 오는지에 따라 관측이 다르다.** 어느 구성이었는지 기록한다.
  - VBUS 를 끊은 케이블이나 UART7: MCU 도 꺼진다. 전원을 다시 넣은 뒤 따로 `record --seconds 10` 을 돌려 `reset_cause == "POWER"` 를 본다.
  - 보통 USB-C 케이블: VBUS 가 D10 을 거쳐 WeAct 5 V 에 들어가므로(보드 설계 3.1) 주 전원을 내려도 MCU 가 살아 있고 모터 전원(VM)만 빠질 수 있다. 이것은 시험 구성으로 고르는 분기가 아니라 관측 설명이다 — S1 에서 메인보드 5 V 역전원이 보였으면 5절 3 대로 이미 케이블·링크를 바꿨어야 한다(Codex). 이 경우 재부팅 없이 링크가 이어지며, 그때 켜진 `fault_flags`·`driver_status` 를 그대로 기록한다. 특정 결함 비트를 기대하지 않는다(이미 초기화된 드라이버는 `drivers_ready` 가 저절로 내려가지 않는다).
- 바퀴가 서는 시간·거리는 영상이나 줄자로 잰다.

### 9.2 다음 PCB 판 뒤에 할 것

링크 끊김·DISARM·명령 만료(`bringup.py scenario --kind tx_cut|disarm|expiry --rate 0.5`)는 정상 빌드의 `DRIVE` 가 필요해 8절과 같은 이유로 기다린다. 바닥 시험은 `--floor --power-switch-reachable` 을 붙이고, `outputs_locked`·`estop_unwired` 가 해제된 빌드에서만 한다.

합격은 `external.<항목>.pass == true` 로만 한다.
- `output_response_ms` 문턱(100 ms)은 `disarm`·`iwdg` 에만 있다: 받침대에서도 판정한다. `tx_cut`·`expiry` 에는 출력 반응 문턱이 없다.
- `stop_time_ms` 500 ms·`stop_distance_mm` 100 mm(잠정, `tx_cut`·`expiry`·`disarm`): 바닥에서만 판정한다.
- `tx_cut` 은 `worst_case.met == true` 여야 한다.
- 계획 S5 의 비상정지 입력 경로와 Jetson 강제 재부팅(브라운아웃 → MCU 만료 정지)도 여기서 한다 — 그 전까지 **미완료**.

**S5 멈춤**: 어느 경우든 문턱 안에 서지 않으면 자율 단계로 가지 않는다.

## 10. 끝나면

- 단계마다 `artifacts/` 경로와 합격·멈춤을 총괄에 보고한다.
- 실측값(G, 영점, `duty_sign`, 정지 시간·거리)은 계획 5절 표가 가리키는 정본 파일에만 적는다.
- 끝난 산출물의 보관은 저장소 규칙(CONTRIBUTING 7절)을 따른다.

## 11. 다음 PCB 판 요구 사항 (사용자 결정 2026-11-12)

**비상정지 커넥터와 입력**
- **배선**: 접점은 상시 닫힘(NC)으로, 한쪽은 MCU 입력 핀에, 다른 쪽은 GND 에 잇는다. 핀은 외부 저항 4.7–10 kΩ 으로 3.3 V 에 풀업한다. 버튼을 누르거나 선이 끊기면 접점이 열려 핀이 high 가 된다. **high = 정지**다.
- **핀 후보**:
  - PE15: `TIM1_BKIN`(AF1)이고, 헤더에 있으며 지금 비어 있다(핀맵 검토 B8).
  - PD8·PD15: 지금 펌웨어가 쓰지 않는 핀이다. 다음 판 회로도에서 비어 있는지, 헤더에 나와 있는지 확인한다.
- **PE15 를 고를 때**: TIM1 브레이크로 PWM 을 하드웨어에서 끊는 보조 경로를 함께 쓸 수 있다. 그래도 출력 차단 자체는 DRVOFF 로 한다. 브레이크는 `AOE = 0` 으로 두고 정지 레벨을 OIS 로 정한다(핀맵 검토 B8). 이 경로도 MCU 안에 있으므로 MCU 와 무관한 차단은 아니다.
- **펌웨어**(다음 판 때 별도 PR): `FW_ESTOP_PIN` 에 그 핀을 넣고 high 를 정지로 읽는다(보드 설계 7절). 그 뒤에 출력이 열린 정상 빌드, S4, S5 의 정상 빌드 시나리오, S6 을 연다.

**MCU 와 무관한 하드웨어 차단**
- 지금은 사람이 손에 든 주 전원 차단 스위치로 대신한다.
- 다음 PCB 때 다시 검토한다. 후보는 계획 R1 의 권고다: 비상정지가 MCU 와 관계없이 DRVOFF 를 high 로 올리는 OR 구조, 또는 모터 전원을 끊는 외부 릴레이.
