# 도착일 절차서 — S0 첫 굽기부터 S5 안전 동작까지

작성일: 2026-11-12 · 상태: **절차 초안, 실기 전**(부품 도착 11-20) · 관련: [실차 이식 계획](2026-11-11-sim-to-real-bringup.md) 5절(단계 표 정본), [보드 설계](../design/2026-11-12-weact-h743-board-firmware.md), [모터 출력 설계](../design/2026-11-12-weact-h743-motor-output.md), bring-up 도구 `tools/bringup.py`(주인: 콘솔 세션), udev `deploy/udev/README.md`

이 문서는 계획 5절의 S0–S5 를 **칠 명령** 단위로 풀어 쓴다. 단계의 합격·멈춤 조건의 정본은 계획 5절이다. 여기 적은 판정 필드는 그 조건을 도구 출력에서 읽는 방법이다. 명령·옵션은 콘솔 세션과 맞췄다(2026-11-12).

## 0. 공통 규칙

- **멈춤**: 단계의 멈춤 조건이 하나라도 생기면 그 단계에서 멈추고, 원인과 기록 경로를 남긴 뒤 총괄에 올린다. 다음 단계로 넘어가지 않는다.
- **결정 경계** — 아래는 이 절차가 대신 넘지 않는다.
  - 모터 전원(VM)은 IPROPI 보호 확인 전에는 S2 의 VM 투입 순서로만 넣는다(사용자 결정 2026-11-11).
  - 출력이 열린 빌드는 **커밋된 보호 확인 기록**이 있어야 구성된다(모터 출력 설계 2절). 출력을 열지는 사용자 결정(Q6)이다. 사용자가 채팅에서 결정을 말하면 에이전트가 기록을 써도 된다(사용자 2026-11-18, evidence 에 결정 원문).
  - 회로도 V1.0 이 최종판이고 비상정지 입력이 없다(사용자 2026-11-18). 출력이 열린 정상 빌드는 **사용자의 비상정지 면제 기록**이 있어야 구성된다([면제 계획](2026-11-18-v1-estop-waiver.md), #580·#584, 기록 #582). 그래서 **S4, S5 의 정상 빌드 시나리오(`tx_cut`·`disarm`·`expiry`), S6 텔레옵은 면제 이미지(2절 `fw-waived`)로 면제 계획 4절의 순서대로** 한다: S4 축별 폐루프(받침대) → 받침대 정지 확인 다섯 가지 → 바닥 0.05 m/s → 0.132 m/s.
  - MCU 와 무관한 하드웨어 차단은 **사람이 손에 든 주 전원 차단 스위치**(또는 배터리 커넥터 뽑기)로 대신한다(사용자 결정 2026-11-12, V1.0 최종판이라 그대로 — 면제 계획 0절). 출력이 열린 실행에서는 늘 손 닿는 곳에 둔다.
- **기록**: 실행마다 `artifacts/<UTC 시각>_<단계>_<구분>/` 를 새로 만든다. 예: `artifacts/20261120T093000Z_s3_commission_L_01/`. 덮어쓰지 않는다.
  - 구동 명령은 결과를 `run.jsonl` 의 `{"t":"result"}` 줄과 stdout JSON 두 곳에 남긴다.
  - `cpr`·`analyze`·`kth`·`clock`·`rearm_hold_check.py` 는 stdout 에만 남기므로 `| tee result.json` 을 붙인다.
- **장치 경로**: udev 규칙 설치(4절) 뒤에는 `--device /dev/forklift-mcu`(USB CDC, `--baud` 없음). UART7 대안은 USB-UART 어댑터 경로에 `--baud 460800`.
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
# S3·S5(9.1 의 ②·시운전 상한 DISARM·주 전원 차단 모두): 출력 열린 시운전 빌드 — 사용자가 보호 확인 기록을 커밋한 뒤에만 구성된다
build_fw open -DFW_BUILD=commission -DFW_MOTOR_OUTPUT=1 \
  -DFW_IPROPI_PROTECTION_RECORD=docs/validation/2026-11-16-ipropi-protection-record.md   # 사용자 기록 #552
# S4·S5 정상 빌드 시나리오·S6: 출력 열린 정상 빌드(면제 이미지) — 면제 계획 4절 순서(S4 → 받침대 정지 확인 다섯 가지 → 바닥 0.05 m/s → 0.132 m/s)
build_fw waived -DFW_BUILD=normal -DFW_MOTOR_OUTPUT=1 \
  -DFW_IPROPI_PROTECTION_RECORD=docs/validation/2026-11-16-ipropi-protection-record.md \
  -DFW_ESTOP_WAIVER_RECORD=docs/validation/2026-11-18-estop-waiver-record.md        # 사용자 기록 #582
```

- ELF: `build/fw-<이름>/boards/weact_h743/fw.elf`(stall 은 `fw_iwdg_stall.elf`).
- 굽기 전 정체 확인: `strings build/fw-<이름>/boards/weact_h743/fw*.elf | grep '^forklift-mcu'`. 빌드 종류, `locked` 또는 `OPEN … rec=<기록 해시>`(면제 이미지는 끝에 `waiver=<면제 기록 해시>` 까지), git 커밋, `link=usb` 가 맞는지 본다.
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
udevadm info -q property /dev/forklift-mcu | grep ID_MM_DEVICE_IGNORE   # =1 이어야 한다(ModemManager 무시)
cat "/sys$(udevadm info -q path /dev/forklift-mcu | sed 's,/[^/]*:[^/]*/tty/.*,,')/power/control"   # on (USB autosuspend 끔)
```

DFU 로 펌웨어를 올리는 노트북에는 STM32 DFU 장치(0483:df11)를 일반 사용자가 열 수 있게 하는 규칙 한 줄도 둔다(STM32CubeProgrammer 의 규칙과 같은 내용). 사용자 권한 임시 파일에 쓴 뒤 설치한다:

```bash
printf '%s\n' 'SUBSYSTEMS=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", GROUP="users", MODE="0666"' > /tmp/50-stm32-dfu.rules && \
  sudo install -m 644 /tmp/50-stm32-dfu.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger --subsystem-match=usb
```

ModemManager 무시와 USB autosuspend 끄기는 둘 다 이 규칙이 한다(D5 설계 4.10, 총괄 2026-11-15). 펌웨어는 DTR·suspend 때 송신 링을 통째로 버리고, 다리는 그것을 MCU 침묵으로 읽는다. 노트북에서 재연습할 때도 같은 규칙을 깐다. **되돌리기**는 `deploy/udev/README.md` 의 Undo 절을 따른다. 규칙 파일을 지우고 reload·trigger 한다. 꽂혀 있는 포트는 `power/control` 에 `auto` 를 쓰거나 다시 꽂으면 커널 기본으로 돌아간다.

규칙은 0483:5740 과 제품 문자열 `forklift-mcu *` 로 고른다. 보드가 여럿이면 `/dev/forklift-mcu-<UID>` 를 쓴다. 호스트 도구는 DTR 을 내리지 않고, 열 때마다 첫 프레임 앞에 0x00 을 보낸다(#462).

## 5. S1 — 전원·배선 (모터 전원 경로를 뺀 채)

계획 5절 S1 행이 정본이다. 이 단계에는 펌웨어 명령이 없다. 측정 순서는 이렇다.

1. 모터 전원 경로를 뺀다. 배터리 → F1 → LM74801 → VCC·5 V·**3.3 V**·19 V 레일을 무부하·부하로 잰다. 각 레일 공칭 ±5 % 안이어야 한다.
2. Jetson 을 19 V 로 켜고 MAXN SUPER 로 둔 채 **최대 부하를 실제로 건다**(CPU·GPU 를 함께 쓰는 부하 프로그램 — 모드만 바꿔 놓고 쉬는 것은 최대 부하가 아니다). 10 분 동안 19 V 와 Jetson 이 안정한지 본다.
   - 장치는 Orin Nano Super(`scienceowl@jetson.local`, zetin Wi-Fi, 2026-11-17 기록 [hw-day](../validation/2026-11-17-hw-day.md)). 부하는 `HW_DAY=<날짜> bash tools/hwday/jetson_stall.sh 300`(정지 5 분 + 모든 코어 부하 5 분, 다리 정체 판정 포함)이다. 이 부하는 CPU 만이고, **S1-2 는 이것으로 합격 처리**한다(총괄 2026-11-18: 지금 실차 스택은 GPU 를 무겁게 쓰지 않음. GPU 부하는 인식을 Jetson GPU 로 돌릴 때 다시).
   - 재부팅 뒤 `jetson_clocks` 가 풀리는지는 전날 `tools/hwday/jetson_prep.sh` 가 기록한다. 풀렸으면 부하 전에 `bash tools/hwday/jetson_power.sh` 를 한 번 돌린다.
   - 당일 순서·판정 한 장: [11-20 순서표](2026-11-20-arrival-day-sequence.md).
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

**첫 부팅 판정**: 부팅하고 링크가 열리면, 아무것도 만지지 않은 채 15 s 를 기록해 도구로 판정한다. 손작업은 없다(기다리기만 한다).

```bash
D=artifacts/20261120T095000Z_s2_first_boot_01; mkdir -p $D
python tools/bringup.py record --device /dev/forklift-mcu --out $D/run.jsonl --seconds 15
python tools/h743_first_boot.py --record $D/run.jsonl --lift-absent | tee $D/result.json   # 승강 KTH 를 달았으면 --lift-absent 를 뺀다
```

합격은 `result == "pass"` 이고, 처음 10 s 를 다음 여섯 항목으로 본다. 기록이 10 s 에 못 미치거나, 그 안에 재부팅·STATE 끊김(0.1 s 넘게)·MCU 시각 역행·계약보다 빠른 KTH 계수 증가가 있으면 `invalid` 다 — 새 파일로 다시 기록한다(한 파일에 여러 번 이어 쓴 기록도 `invalid`). 기준값은 첫 확인용 작업값이라 **실측 뒤 재검토**한다.

| 항목 | 합격 | 근거 |
|---|---|---|
| `drivers` | 모든 STATE 에서 `drivers_ready`, DRV8244 네 개의 `driver_status` 가 상위 2비트 11·하위 결함 6비트 0 | 응답의 상위 비트는 언제나 11(SLVSG24C 표 8-21). VM 이 없으면 0x0000 이 온다 |
| `current` | `current_sense_invalid` 없음, 네 채널 모두 \|`current_ma`\| ≤ 50 mA | 모터를 떼었으니 0 이어야 한다. 1 ADC 카운트 ≈ 0.24 mA(R_IPROPI 1 kΩ, A_IPROPI 4,750). 50 mA 는 약 200 카운트로, 영점 오프셋·잡음을 넉넉히 덮는 작업값이다 |
| `faults` | 예상 밖 결함 비트 없음(`--lift-absent` 면 승강 축 두 비트는 허용) | 손대지 않은 부팅에는 결함이 없어야 한다 |
| `kth` | 조향(승강을 달았으면 승강도)이 NaN 아님, 축마다 받은 프레임 수(`kth_frames`)가 오름 | mcu-link/3 3.3 |
| `wheels_still` | 바퀴 카운트가 2 카운트 넘게 움직이지 않음 | 엔코더 떨림 허용량(작업값). 아무도 만지지 않는 동안이다 |
| `link` | `rx_errors`·`cycle_overruns` 가 오르지 않고 `cycle_overrun` 비트 없음 | 링크·주기가 깨끗해야 한다 |

함께 나오는 `report` 에서 `outputs_locked`(출력 잠금 빌드)와 `reset_cause`(첫 전원이면 `POWER`)를 확인한다.

**실패하면 볼 것**:
- `drivers` 가 실패하면: VM 이 들어왔는지, `bad_status` 값, nFAULT 핀.
- `kth` 가 실패하면: RJ50 케이블과 축 배정(CS1 조향, CS2 승강). 프레임이 안 오르면 배선 문제다. 프레임은 오르는데 NaN 이거나 범위 밖이면, 아직 실측 전인 설정(CRC 변형·영점) 때문일 수 있다 — 고장으로 단정하지 말고 6.2 의 `kth` 판정과 영점 기록으로 가린다(Codex).
- `current`·`faults` 가 실패하면: 6.1 4번에서 잰 핀 전압과 대조한다.

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
- **KTH CRC 변형 확정**(계획 S2, mcu-link/3 계약 3.3): 위 기록의 STATE 에 축별 KTH 계수 셋(받은 프레임, 설정 변형 CRC 오류, 다른 변형이었다면 났을 오류)이 실린다. 펌웨어의 설정 변형은 두 축 모두 `g704_bytes` 다(`board_config.c`). 기록 동안 **조향·승강을 범위 안에서 천천히 끝까지 움직인다** — 두 변형은 4096 개 각도 중 256 개에서 CRC 가 같다. 축이 그런 각도에 멈춰 있으면 판정이 나지 않는다. 다른 각도에 멈춰 있어도 `configured_ok` 는 나올 수 있지만, 범위 전체를 지나는지는 S2 가 따로 본다(Codex).
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
- `fw-open` 이 구성·빌드되고, 3.1 로 구운 보드의 제품 문자열이 `forklift-mcu commission OPEN <git> rec=<해시> lim=lr1` 이다(2026-11-25 부터 열린 시운전 이미지는 `lim=lr1` 로 끝난다).
- 한 보드에는 한 실행기만 붙는다. 보드를 여는 모든 도구는 장치를 배타 잠금한다(실 안전 배선 설계 8.1). 다른 도구가 잡고 있으면 아무것도 쓰지 않고 `refused`·종료 2 다. `record --markers` 와 `commission` 을 따로 띄우지 않는다(11 절). 보드 점검 사본(`data/h743_check/`)은 잠금 머지 뒤 main 에서 다시 스테이징한다.
- **Jetson 이 실제로 실행할 도구의 커밋**을 확인했다. 열린 시운전 이미지는 L·R 을 1.0·검증기 천장까지 받으므로, 큰 값의 확인(TTY 재입력·`--vcc-watched`)은 도구에만 있다. 11-24 처럼 진단 도구가 남아 있으면 그 확인이 빠진다.
  - 확인하고 실행하는 곳이 같아야 한다. 확인한 체크아웃으로 **먼저 이동한 뒤**, 그 자리에서 확인하고 그 자리의 `tools/bringup.py` 로 실행한다. `git -C` 는 현재 디렉터리를 바꾸지 않아서, 진단 체크아웃에 머문 채 main 을 확인하는 일이 생긴다.
    ```bash
    cd ~/forklift && git log -1 --format=%h && git status --short | head
    python -c "import forklift_core.plant.bench_limits as b; print(b.__file__)"   # ~/forklift/src/... 이어야 함
    ```
  - 그 커밋이 이 PR 이후의 main 이어야 하고, 불러오는 `forklift_core` 가 같은 체크아웃이어야 한다. 아래 명령은 모두 이 `~/forklift` 에서 실행한다.
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
  - 기본은 듀티 0 이 아닌 ±0.15 이하, 시간 0.1–1.5 s 다. 도구가 그 밖의 값은 보내지 않고 거부한다(`refused`). M·H 는 펌웨어도 0.15 를 넘으면 거부하고 이전 명령을 유지한다.
  - **L·R 의 큰 값**(0.15 초과 또는 1.5 s 초과, 설계 `docs/design/2026-11-25-bench-commission-limits.md` 1.2): `lim=lr1` 이 붙은 열린 시운전 이미지여야 한다. `--wheels-lifted`(바퀴를 띄우고 차체를 고정, 회전체 주변 배선 정리)가 필요하다. 터미널에서 문구를 다시 입력해야 한다(`--yes` 로 건너뛸 수 없음). 0.3 초과는 0.3 → 0.5 → 0.7 → 1.0 으로 단계를 올리고 `--vcc-watched` 를 준다(VCC 감시). 한 실행은 보드 시간 상한에서 0.5 s 를 뺀 값(약 2147 s)까지다. 펌웨어는 L·R 을 1.0·검증기 천장까지 받으므로 크기는 도구 인자가 정한다.
  - 엔코더가 없는 바퀴는 `--bypass enc_stuck,enc_polarity` 로 도구의 엔코더 조기 정지를 끈다. 비트는 기록에 남는다.
  - 삼각파는 `--profile-tri LO HI PERIOD CYCLES` 이고 \|`--duty`\| = HI 다. 부호는 모든 조각에 붙는다.
  - `--observed` 는 주지 않는다: 실행 뒤 도구가 물으면 **실제로 본 방향**을 답한다(pos = 차체 앞·왼쪽·위). 미리 적어 두면 모터와 센서 극성이 함께 뒤집힌 경우를 놓친다(Codex).
- **합격**
  - 판정: `verdict.duty_sign_ok`·`verdict.sensor_sign_ok` 가 참이고, `metrics.valid` 가 참이다.
  - 극성: 틀리면 `verdict.suggested_duty_sign` 을 보드 설정의 `duty_sign` 에 넣는다.
  - 전류: `metrics.current_peak_ma`·`current_mean_ma` 가 벤치 전류계와 맞는다.
  - 범위 끝: 조향·승강은 기구 한계 전에 잰다.
- **무부하 최대 바퀴 속도**: 엔코더가 있는 바퀴마다, 큰 값 규칙(위 명령 제약)으로 듀티를 단계적으로 올리며(0.3 → 0.5 → 0.7 → 1.0, `--vcc-watched`) `metrics.rate_mean_rad_s` 를 기록한다. 기본 0.15 값도 함께 남긴다.
- **전원 강하·회생 과전압(오실로스코프)**: 기동·역전·멈춤 때 VCC 가 LM74801 UVLO(10.41 V)·19 V 쪽 UVLO 에 걸리지 않아야 하고, 급감속·역전 때 VCC 상승이 레일 부품 정격 안이어야 한다. 오실로스코프가 없으면 이 항목은 **미완료**로 남는다 — S3 합격으로 치지 않는다.
- **멈춤**: 의도와 다른 방향, 과전류, 기구 한계 접촉, Jetson·MCU 리셋, `faults_seen` 에 예상 밖 결함.

## 8. S4 — 축별 폐루프

**면제 이미지(2절 `fw-waived`)로 받침대에서 한다.** `bringup.py step` 은 정상 빌드에 `DRIVE` 를 보내므로, 출력이 열린 정상 빌드인 면제 이미지가 필요하다(0절, [면제 계획](2026-11-18-v1-estop-waiver.md) 4절 3번, 모터 출력 설계 2절). 이미지를 바꾸는 방법은 3.1 이다.

```bash
python tools/bringup.py step --device /dev/forklift-mcu --out $D/run.jsonl --wheel left --target 0.8
```

- 바퀴 목표 상한은 0.2 m/s 를 설정된 바퀴 반경(지금 공칭 0.125 m, S8 실측 전)으로 바꾼 1.6 rad/s 다(V1.0 e-stop 면제 계획 3절: 1.0 rad/s 면 0.132 m/s 시험이 막혔다). 계단은 최대 3 s 다.
- 기록할 값: `rise_s`·`overshoot_pct`·`settle_s`·`unsettled`·`gaps`.
- 계획 S4 의 조향 추종 응답과 엔코더 고장 주입(커넥터 뽑기·A/B 바꾸기)도 이때 한다 — 그 전까지 **미완료**.

## 9. S5 — 안전 동작

### 9.1 지금 할 수 있는 것

**고른 듀티에서 DISARM**(받침대, `fw-open`, 2026-11-25 부터 — 설계 `docs/design/2026-11-25-bench-commission-limits.md` 1.3):
- 열린 시운전 이미지의 L·R 시간 상한은 검증기 천장(약 35.8 분)이다. 그래서 '상한에서 멈춘 뒤 정지 거리' 대신 `duty_disarm` 을 쓴다. 고른 듀티에서 0.5 s 유지가 STATE 에 보이면 도구가 DISARM 하고 그 뒤의 정지를 본다.
- 큰 값 규칙(7 절)을 그대로 따른다.

```bash
python tools/bringup.py commission --device /dev/forklift-mcu --out $D/run.jsonl --axis L --duty 0.3 --seconds 1.5 \
  --scenario duty_disarm --wheels-lifted --power-switch-in-hand \
  --output-response-ms <X|unavailable> --stop-time-ms <X|unavailable> --stop-distance-mm <X|unavailable>
```

- `result == "ok"` 이고 `validity.valid`·`validity.physical` 이 참이어야 시험이 성립한다. 성립하면 목표 도달, 0.5 s 유지, 시험용 DISARM 의 수락이 같은 무장 세대에서 입증된 것이다. 그것만으로 합격이 아니다.
- `limit_disarm`·`hold_disarm`(MCU 시간 상한에 닿은 뒤 DISARM)은 이제 **잠긴 시운전 보드나 모형**에서만 돈다(열린 보드에서는 도구가 ARM 전에 거부). 상한 경로 자체의 확인이고, 물리 합격 칸은 비어 있다(`pass: null`). 이때만 `--stop-grace-ms` 에 빌드 값(300)을 정확히 넣는다.
- **출력 반응은 받침대에서도 판정한다**: `external.output_response_ms.pass == true`(DISARM 문턱 100 ms). 정지 시간·거리만 받침대에서 참고값(`reference: "stand"`, `over_limit`)이다(Codex).
- 나중에 기록으로 다시 볼 때: `python tools/bringup.py analyze --record $D/run.jsonl --kind duty_disarm`. 사건 시각은 기록의 시험용 DISARM 쓰기 완료이고 Enter 표시는 쓰지 않는다(`--marker` 는 그 종류의 실행 순번).

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

### 9.2 면제 이미지로 할 것 (면제 계획 4절)

링크 끊김·DISARM·명령 만료(`bringup.py scenario --kind tx_cut|disarm|expiry`)는 정상 빌드의 `DRIVE` 가 필요해 8절처럼 면제 이미지(2절 `fw-waived`)로 한다. 순서는 [면제 계획](2026-11-18-v1-estop-waiver.md) 4절이다.
- 받침대: S4 의 최종 게인·정지 설정으로 만료·통신 끊김·`DISARM`·콘솔 STOP·게임패드 놓음을 각 1 회 한다(4번). 하나라도 멈추지 않으면 바닥으로 가지 않는다.
- 바닥 0.05 m/s: `--floor --power-switch-reachable --rate 0.4` 로 통신 끊김 한 번, `DISARM` 한 번을 한다(5번).
- 0.132 m/s: `--rate 1.06` 으로 같은 두 가지를 한 번씩 더 한다(6번).
- 면제 이미지도 STATE bit7(`estop_unwired`)은 켜진 채다. 호스트는 열린 tty 의 USB 제품 문자열에 `waiver=` 토큰이 있을 때만 이것을 경고로 본다(콘솔 몫). bit6(`outputs_locked`)은 꺼져 있어야 한다.

합격은 `external.<항목>.pass == true` 로만 한다.
- `output_response_ms` 문턱(100 ms)은 `disarm`·`iwdg` 에만 있다(`duty_disarm`·`limit_disarm`·`hold_disarm` 은 `disarm` 의 문턱을 따른다 — 9.1): 받침대에서도 판정한다. `analyze` 는 기록의 실행 종류가 `--kind` 와 다르면 거부하고, 시운전 기록에 `--floor` 를 주면 거부한다. `tx_cut`·`expiry` 에는 출력 반응 문턱이 없다.
- `stop_time_ms` 500 ms·`stop_distance_mm` 100 mm(잠정, `tx_cut`·`expiry`·`disarm`): 바닥에서만 판정한다.
- `tx_cut` 은 `worst_case.met == true` 여야 한다.
- Jetson 강제 재부팅(브라운아웃 → MCU 만료 정지)도 여기서 한다 — 그 전까지 **미완료**. 비상정지 입력 경로는 V1.0 에 없으므로 시험하지 않는다(11절).

**S5 멈춤**: 어느 경우든 문턱 안에 서지 않으면 자율 단계로 가지 않는다.

## 10. 끝나면

- 단계마다 `artifacts/` 경로와 합격·멈춤을 총괄에 보고한다.
- 실측값(G, 영점, `duty_sign`, 정지 시간·거리)은 계획 5절 표가 가리키는 정본 파일에만 적는다.
- 끝난 산출물의 보관은 저장소 규칙(CONTRIBUTING 7절)을 따른다.
- 메모(총괄 2026-11-14): Isaac 런처(`riib_scene`·`isaac_live`)는 K1 승격(#499)으로 near-end-stand 가 켜져 있으면 `--near-observe-arrival 0.05,0.05` 가 기본이다(러너 기본은 끔). 실차 런치 설정에 같은 값을 쓸지는 첫 주행 뒤에 정한다.

## 11. 비상정지 — V1.0 최종판 (사용자 2026-11-18)

- 회로도 V1.0 이 끝까지 쓰는 PCB 다. 비상정지 입력도, 다음 판도 없다.
- 출력이 열린 정상 빌드는 사용자의 면제 기록으로 연다([면제 계획](2026-11-18-v1-estop-waiver.md) #580, 펌웨어 #584, 기록 #582). 바닥 주행 전 확인 순서는 면제 계획 4절이다(8절·9.2).
- 정지 수단은 명령 만료·통신 끊김·`DISARM`·콘솔 STOP·게임패드 놓음이다. 마지막 수단은 손에 든 주 전원 스위치나 배터리 커넥터 뽑기이고, 출력이 열린 모든 실행에서 손 닿는 곳에 둔다(0절). MCU 와 무관한 하드웨어 차단은 없다.
- 다음 판을 전제로 모았던 요구 사항(비상정지 커넥터 배선·핀 후보 등)은 [메인보드 다음 판 요구 사항](../design/2026-11-12-next-board-requirements.md)에 참고로만 남는다.

## 11a. S6·G6 전 — 스캔 허가(7b)의 장착 확인과 운용 조건 (총괄 2026-11-16)

텔레옵 다리의 바닥 허가는 A2M12 스캔으로 만든 장애물 층이 연다(#559, [설계 메모](../design/2026-11-16-bridge-scan-permit.md)). 다리는 `--scan-socket <소켓> --lidar-mount x,y,z,yaw_deg` 를 둘 다 받아야 스캔 허가를 쓰고(장착값 기본값 없음), 없으면 지금처럼 `no_scan`(속도 0)이다.

**S7 첫 항목 — 장착 위치 확인.** 자기 가림 구간이 차체 둘레 띠(정지호·외곽이 비어 있어야 하는 곳)에 걸리면 그 방향의 허가는 늘 0 이다. 그래서 장착을 정하면 먼저:

1. A2M12 를 **차체 둘레가 가리지 않는 높이·위치**에 단다(마스트 기둥 같은 좁은 가림은 괜찮다).
2. 뒤축 중심 기준으로 x·y·z·yaw 를 줄자로 재어 `--lidar-mount` 에 넣고, 가리는 각도 구간은 `--lidar-mask a:b,…`(센서 기준 도)로 준다.
3. 차를 세운 채 다리를 켜고 전진·후진 허가가 둘 다 열리는지 본다. 한쪽이 늘 0 이면 가림이 차체 둘레에 걸린 것이니 장착을 옮긴다.

**운용 조건 — S6·G6 에서 비영 허가를 쓰기 전에 모두 확인한다**(총괄 승인 2026-11-16):

- [ ] 빈 시험장·빈차이고, 운용자가 주 전원 차단 스위치를 손에 쥐고 있다.
- [ ] 시험장에 LiDAR 띠(장착 높이 ± 0.1 m)보다 낮은 물체, 거울·유리·투명 랩·검은 무광 물체가 없다.
- [ ] 첫 주행 전 정적 확인: 차를 세운 채 사람이 상자를 들고 진행 방향 정지 영역에 들어가면 허가가 0 이 되고, 나가면 돌아온다(`scan_check.py --record` 로 함께 기록).
- [ ] 남는 위험(틀린 유효 거리, 평면 밖 물체)은 저속(다리 상한 0.2 m/s)과 운용자 차단으로 받는다는 것을 운용자가 안다.

층 설정(`config/real/obstacle_layer_real.yaml`)의 값은 모두 잠정이다. 감속 0.5 m/s²·지연 0.30 s 는 S5 정지 실측 뒤, 나이 오차표는 S6 첫 기록 뒤 다시 정한다.

## 12. 실차 측정 M1–M3 — 직진 yaw 드리프트·조향 대 곡률·재출발 합류 (총괄 2026-11-14)

출처: [C3 기록 진단](../validation/2026-11-13-c3-record-diagnosis.md) 6 절(시나리오·낼 값·참값 요건·읽는 법 제안). 시뮬에서 본 합류 뒤 불일치가 시뮬 plant 의 성질인지 실차로 먼저 확인한다. 항목·수치의 정본은 그 6 절이고, 이 절은 도착일에 칠 순서와 지금 도구로 기록되는 범위를 적는다.

### 12.1 언제

- 바닥 주행이 열린 뒤다: 면제 이미지로 면제 계획 4절의 받침대·바닥 정지 확인을 마친 뒤([면제 계획](2026-11-18-v1-estop-waiver.md) 4절 7번), S6 텔레옵 단계, 자율 단계 전(0 절 결정 경계, 계획 5 절).
- 속도는 지금의 S6 범위 안이다(텔레옵 명령 상한 0.132 m/s, 안전 상한 0.2 m/s). 속도 이력(가감속 포함)을 시행마다 같게 정하고 기록한다.
- **측정 전에 고정할 것(시뮬-물성 세션과 함께):** 시행 수, 속도 이력, 참값 방법(12.4), 그리고 출처 6.5 의 읽는 법 전체. 고정한 값은 `docs/plans/` 의 별도 등록 문서로 남기고 그 뒤에 잰다. 등록 문서는 6.5 를 빠짐없이 이어받는다(제안값):
  - M3 의 **0.9 m 합류에서 0.42 m 뒤**의 |heading 오차|·|횡 오차| 를 시행마다 구해 그 절댓값들의 중앙값으로 읽고, 오프셋·방향별로도 낸다.
  - heading 중앙값 ≤ 0.003 rad 이고 횡 중앙값 ≤ 5 mm → "이 시험 조건에서 큰 불일치를 보지 못함"(시뮬 고유 원인이라고 확정하지 않는다).
  - heading 중앙값 ≥ 0.006 rad 이거나 횡 중앙값 ≥ 10 mm → "실차 대책 필요".
  - 그 사이이거나, heading·횡 각각의 반복 중앙값에 대한 95 % 구간이 문턱을 가로지르면 → "판정 불가": 조건마다 5 회를 더하고(추가는 한 번), 그래도 가르지 못하면 그대로 보고한다.

### 12.2 무엇을 (출처 6.4)

| 항목 | 시나리오 | 낼 값 |
|---|---|---|
| M1 직진 yaw 드리프트 | q 영점, 3 m 직진, 전진·후진 각 5 회, S6 범위 안 두 속도 | 오도메트리 heading − 참값(rad/m), 끝 횡 오차 |
| M2 조향 대 실제 곡률 | q 를 몇 단계(작은 각 포함)로 유지, 1–2 m 원호, 전진·후진·좌·우 각 3 회. 추가 조건(아래): 최대 조향 좌·우, 적재·빈차 | 참 yaw rate ÷ (v × κ(q)), 방향·좌우별; 명령 q 대비 측정 q |
| M3 재출발·합류 응답 | 정지에서 재출발, 횡 5·20·40 mm 에 해당하는 S 자 합류(0.9 m·2.1 m), 조건마다 5 회; 따로 일정 속도에서 q 계단 | 합류 끝과 0.42 m 뒤의 상대 위치·heading 오차, q 계단에서 yaw rate 응답까지의 지연 |

M2 추가 조건(총괄 2026-11-14, 근거 [왼쪽 조향 부족 기록](../validation/2026-11-14-left-steer-shortfall.md) 4.3 — 시뮬 적재 운반의 포화 S 자에서 왼쪽만 약 34 mrad 덜 꺾임). 위 시험을 대체하지 않고 더한다:

- **최대 조향 좌·우 각각:** 포화 명령에서 좌회전·우회전을 따로 잰다. 시뮬에서 여유가 가장 작은 곳이 포화다.
- **명령 q 대비 측정 q(KTH7801):** 시행마다 명령한 조향 기구각과 STATE 의 측정 q 의 차(정상 오차·방향별)를 남긴다(G1 원행).
- **적재·빈차 각각:** 같은 조건을 빈차와 적재로 잰다(시뮬은 적재가 필요조건이 아니었으나 추가 기여를 배제하지 못했다). **적재 = 자율 단계(경로 추종) 측정, 텔레옵·G6 은 빈차만**(총괄 2026-11-14 (가)): 실차 다리는 적재 텔레옵을 거부하고(`real_teleop.enforce_real`) 그 규칙은 바꾸지 않는다. 적재 조건은 경로 추종이 적재 운반을 할 때 같은 정의로 잰다.
- **최대 조향의 한계:** 텔레옵·G6 의 최대 조향은 텔레옵 곡률 한계 0.36 /m(q 0.2333 rad)다. 기구 최대 0.3665 /m(q 0.2374 rad)까지의 4 mrad 는 이 측정이 닿지 않는 범위로 결과 문서에 적는다(텔레옵 한계는 바꾸지 않는다, 총괄 2026-11-14).
- 실차는 조향각 센서가 하나(조향 기구각 q)라 '왼쪽 바퀴만' 같은 바퀴별 비대칭은 직접 못 본다. 그래서 **방향별로 측정 q 대비 실제 곡률**(참 yaw rate ÷ v 를 κ(측정 q) 와 비교)을 낸다 — 좌·우 비율이 다르면 그것이 실차에서 보이는 비대칭이다. 명령 q 대비 곡률은 조향 추종과 기구 관계가 섞인 값이라 따로 적는다.

### 12.3 기록과 참값 준비 (출처 6.2·6.3) — 지금 도구로 되는 것과 빠진 것

한 시계로 남긴다(MCU 시각·호스트 시각과 그 변환·정렬 오차). 2026-11-14 main 기준으로 확인했다.

| 필요한 기록 | 지금 | 빠진 것 |
|---|---|---|
| MT6701 L·R 모터 축 원 카운트와 바퀴 증분(G·실효 반경 명시) | `bringup.py record` 와 `teleop_bridge.py --record`(G1, #497) 가 STATE 를 원행으로(수신 `host_ns`, 다리는 읽기 시각 `read_ns` 도·`mcu_time_us`·카운트·q·결함 비트) 남긴다. 텔레옵 주행 중에는 `teleop_bridge.py --record` 만 써야 한다(두 도구가 같은 tty 의 수신 스트림을 나눠 읽게 된다). 그 기록은 주기마다의 명령·상태 요약뿐이다 | 구현됨(G1, #497) — 실차 기록에서 행 형식·시각을 확인 |
| KTH7801 조향 기구각 q 와 q → κ 변환(영점·기구 관계) | STATE 의 `steering_rad`(q), 변환은 `RealKinematics` | 영점·기구 관계의 실측값은 S2–S3 결과로 채운다(계획 5 절 정본) |
| 구동·조향 명령 | 텔레옵 기록의 `applied`(제출한 속도·q·`curvature_inv_m`)와 목표 곡률 `target_kappa`. 측정 곡률(`measured_kappa`)은 내부 판단에만 쓰이고 파일에는 남지 않는다 | (측정 곡률은 G1 의 원행과 G8 로 다시 낸다) |
| D435i 세 축 자이로·가속도, 장착 회전으로 차체 yaw rate | `tools/imu_record.py`(G2, [설계](../design/2026-11-14-imu-record-and-mount.md)): 표본마다 32 bit 장치 시각 `dev_us`·시간 영역·호스트 단조 `host_ns`. `forklift_core.sensors.imu_record` 가 되감김을 풀고 연속 구간마다 G7 fit, `imu_mount` 가 차체 위쪽 축과 yaw rate(G3). 책상 정지 측정: `docs/validation/2026-10-27-gyro-model-d435i-static.md`, 형식 회귀: `docs/validation/2026-11-14-imu-record-g2/` | 차체 위 G3 절차(아래)와 그 등록 항목. 텔레옵 다리와 IMU 기록기는 같은 Jetson 에서 돌려 `host_boot_id` 가 같아야 한다 |
| A2M12 스캔(참값용 벽 정합) | 7a 의 `scan_check --record`·`ScanFeed` 가 원본 프레임·`host_recv_ns`·판정을 파일에 남긴다. 텔레옵 다리에는 아직 스캔 원천이 없다(7b) | **G4** 그 기록 기능을 시행 중에 연결·운용하고 같은 시계로 정렬하기, 그리고 벽 정합으로 heading 을 내는 방법 |
| 시행 경계 | `bringup.py record --markers` 와 `teleop_bridge.py --markers`(G5, #497)가 Enter 로 표시한다. `--profile` 과는 함께 쓰지 않는다(일정표가 구간 표지를 자동으로 남긴다) | 구현됨(G5, #497) — 실차에서 확인 |
| 위치·자세 참값(시작·끝, M3 는 합류 끝과 0.42 m 뒤 두 곳) | 없음 | **G9** 고른 참값 방법에 맞춘 기준점(바닥 표시·차체 두 기준점), 측정 도구, 좌표 추출, 위치·heading 불확도 계산. 영상을 쓰면 영상 프레임 시각 ↔ 호스트 시각 동기 방법과 오차 |
| 같은 속도 이력, q 유지·계단, S 자 합류 | `teleop_bridge.py --profile`(G6, #508, [설계](../design/2026-11-14-profile-drive-and-m-analysis.md) 2 절): 콘솔 teleop 메시지의 두 비율만 일정표 값으로 바꾸고 관문은 그대로, 다리 터미널 Enter 로 시작, 관문이 깎거나 deadman 을 놓으면 그 시행은 void. 실차 실행은 G4 뒤 | **G6** S6 범위 안의 개루프 프로필 구동(속도 이력 + q 일정표: 유지·계단·합류 모양). 받침대 전용 `commission` 이 아니라 **정상 빌드에서, S6 의 속도 감시·스캔 허가·정지 조건과 운용자 확인 게이트를 그대로 유지**하고, 손에 든 차단 스위치를 쥔다. M3 의 합류를 경로 추종으로 하려면 자율 단계 뒤가 된다 — 개루프 q 일정표를 M3 로 인정할지는 측정 전에 정한다 |
| 시각 변환 | STATE 행마다 `mcu_time_us` 와 수신 `host_ns` 가 있고, `bringup.py clock --record`(G7, #497)가 MCU 시각 ↔ 호스트 시각을 맞추며, IMU 는 `forklift_core.sensors.imu_record.imu_clock_fits`(G2, #500)가 맞춘다 | 구현됨(G7·G2) — 실차 기록으로 정렬·지연의 한계(G7 설계 5 절)를 확인 |
| M1–M3 의 값 | `forklift_core.plant.m_analysis`(G8, 설계 3 절): 시행 구분, 편향 구간, IMU heading 과 그 불확도 하한, 자이로 배율, M1 drift, M2 이득 둘, 엔코더 차동·IMU 계단 응답 | 위치 참값이 필요한 값(M1 끝 횡 오차, M3 합류 끝·0.42 m 뒤 상대 위치)은 G9 의 방법이 정해진 뒤 그 입력으로 계산한다. 95 % 구간과 판정은 등록 문서(C3 6.5) |

**G3 차체 위 절차**(S6 텔레옵이 열린 뒤, 포크를 가장 낮게 둔 채 — IMU 가 캐리지에 붙어 승강과 함께 움직인다. 승강하면 다시 잰다). 이 차는 제자리 회전을 못 하므로(뒤축 기준 유한 곡률) 원호로 잰다:

1. `imu_record.py --out artifacts/<UTC>_g3/imu.jsonl` 과 `teleop_bridge.py --record … --markers` 를 같은 Jetson 에서 띄운다. Jetson 의 `pyrealsense2` 는 PyPI 휠이 아니라 `~/src/librealsense`(v2.59.2) 소스 빌드다 — 휠로는 Motion Module 이 보이지 않았다(2026-11-17, 설치 `tools/hwday/jetson_librealsense_build.sh`).
2. 데움 → 정지 60 s → 왼쪽 최대 곡률 원호 2 바퀴(S6 속도, 지름 2/κ_max 의 빈 바닥) → 정지 60 s → 오른쪽 최대 곡률 2 바퀴 → 정지 60 s. 원호의 정상 구간 시작·끝을 G5 표지로 남긴다.
3. 결과: 정지 중력으로 잰 위쪽 `û_g`, 앞뒤 정지 편향을 뺀 두 방향 원호 축 `û_ω`, 세 벡터 사이 각, 정지 구간 사이 편향 변화, 중력 기준 회전 부호(왼쪽 양·오른쪽 음)와 바퀴 오도메트리 부호의 일치, 원호 중 위쪽에 수직인 회전 성분(평면 가정 진단값, yaw 오차 상한 아님).
4. 측정 전에 시뮬-물성과 등록할 것: 최소 데움 시간과 정지 구간 사이 편향 변화 허용치(넘으면 다시 기다림), `û_g`·`û_ω` 사이 각의 문턱(넘으면 평면 가정이나 장착을 의심하고 멈춤). 책상 실측의 450–960 s 는 데움 시정수이지 완료 시간이 아니다.

### 12.3a 무장 전 MCU 시계 확인 (D5, 총괄 2026-11-15)

`teleop_bridge.py` 는 MCU 시계 속도가 1 ± 0.001 안에 든다고 확인한 뒤에만 무장한다. 판정은 무장 전 DISARM 핑의 왕복으로 낸다. 연결마다, 그리고 부팅이 바뀔 때마다 처음부터 다시 잰다.
- 콘솔 상태 줄에 `MCU 시계 확인 중 n/m s — 확인 뒤 무장할 수 있습니다` 가 보이는 동안은 무장 요청이 대기한다(`arm_wait: clock_rate_unverified`). 노트북 리허설 기준 20–40 s 가 걸린다. 시간을 약속하지 않고 확인이 끝나기를 기다린다.
- `MCU 시계 속도가 범위 밖입니다` 가 뜨면 그 부팅에서는 무장할 수 없다(`clock_rate`). HSE 가 안 잡혀 HSI 로 부팅한 보드일 수 있다. 전원을 다시 켜고, 되풀이되면 크리스털·배선을 본다. 무장 중에 범위 밖이 확인되면 다리가 STOP·DISARM 한다.
- 링크 끊김 판정은 MCU 시각 간격으로 한다(D5). 다리가 30–80 ms 멈춰도 끊김이 아니다. 150 ms 를 넘거나, MCU 가 50 ms 넘게 침묵하거나, 가장 새 STATE 의 나이(지연 상한 포함)가 50 ms 를 넘으면 끊김이다. 85–150 ms 멈춤에서는 MCU 의 DRIVE 만료(100 ms)가 먼저 멈춘다. 콘솔에 `mcu_stopping` 사건이 뜬다. 1 s 안에 40 ms 넘는 주기가 세 번이면 `loop_slow` STOP 이다.

### 12.3b 단말 정지·다리 정체로 난 void (총괄 2026-11-16)

- **운용자 단말은 유선이나 AP 근거리로 둔다.** 단말이 멈추면 DRIVE 의 유효 기간이 다하고 MCU 가 스스로 멈춘다.
  - 일정표 주행 구간에서는 텔레옵 명령 기한(0.30 s)이, 정지 구간에서는 lease(1.0 s)가 DRIVE 만료에 들어간다. 단말이 그만큼 멈추면 MCU 가 멈추고 그 시행은 `void:disarmed:mcu_stopping` 이다.
- **다리 정체:** 다리 루프가 약 100 ms 넘게 멈추면 DRIVE 가 끊기고 같은 결과가 난다(85–150 ms 구간은 12.3a).
- **단말 정지나 다리 정체로 난 `void:disarmed:mcu_stopping` 도 다른 void 와 같이 그날 블록 끝에 같은 일정표로 다시 한다**(등록 1.1). 현장 점검표에는 원인(단말·다리)을 결과 계산 전에 적는다.
- 확인한 사실(노트북 리허설, 2026-11-16): 운용자 대역(websocket 클라이언트)이 약 1 s 멈춘 정지 중 시행이 `COMMAND_TIMEOUT` 으로 void 됐다. CPU 절반을 점유한 부하 창에서 다리 루프가 한 번 124 ms 멈춰 `mcu_stopping` 이 났다(80 ms 한 번은 견딤). Jetson(Orin Nano Super, MAXN_SUPER + `jetson_clocks`)에서는 2026-11-17 에 정지·모든 코어 부하 5 분씩 40 ms 넘는 주기 0, 최대 29.6 ms 였다([hw-day](../validation/2026-11-17-hw-day.md)) — 실차 다리의 기준선은 이 값이다.

### 12.4 참값 (출처 6.3)

- 판정에 쓰려면 heading 불확도 1 mrad(1σ) 이하. 아니면 그 항목은 "판정 불가".
- 자이로 적분: 데움 시간, 시행 앞뒤 완전 정지 구간(편향 추정), 장착 상태 감도 확인. 책상 백색잡음만으로 30 s 주행 + 30 s 정지 편향 평균이 약 1.6 mrad(1σ)라 정지 구간을 늘리거나 시행을 짧게 한다.
- 바닥 표시: 차체 두 기준점 간격과 측정 오차로 heading 불확도를 계산해 적는다.
- LiDAR 벽 정합: 오도메트리와 독립으로, 정지 반복으로 반복성을 먼저 잰다(G4 필요).

### 12.5 순서

1. G1–G9 를 도구·절차로 채우고(콘솔 세션, 시험 먼저), 받침대·모형에서 기록 형식을 확인한다. 순서(총괄 2026-11-14): G1·G5·G7 → G2·G3 → G6·G8 → G4(A2M12 연결 뒤). G9 는 참값 방법을 정할 때 함께. **G6 을 실차에서 돌리는 것은 G4(A2M12 스캔을 텔레옵 다리에 연결) 뒤다** — 다리의 바닥 허가는 스캔이 있어야 연다(지금은 `no_scan`, 속도 0). A2M12 연결은 사용자 대기다.
2. 12.1 의 고정 항목을 시뮬-물성 세션과 등록한다.
3. 바닥 주행이 열린 날: 데움 → 정지 구간 → M1 → M2 → M3, 시행마다 `artifacts/<UTC>_m<번호>_<조건>_<회차>/`.
4. 결과는 등록 문서의 읽는 법대로 판정한다. 후속은 총괄이 검토해 제안하고, 운용에 들어가는 채택(자이로 heading 오도메트리, 재관측, 근접 추적 인증 S4 를 다시 올리는 일)은 사용자가 결정한다. 이 측정만으로 S4 보류(사용자 10-15 결정)가 풀리지 않는다. 시뮬 plant 를 실측에 맞추는 일은 시뮬-물성 세션에 올린다.

### 12.6 이 절의 검토

- Codex 검토(새 스레드): P1 없음. P2 3 — 후속 채택 권한(사용자 결정, S4 보류 유지), 위치 참값 취득 누락(G9 추가), IMU 시각 영역·평면 가정(G2·G3·G7 확장). P3 3 — 읽는 법 전체 승계, 표의 과장 두 곳(측정 곡률은 파일에 없음, 스캔 파일 기록은 이미 있음), G6 의 안전 조건(정상 빌드·S6 조건). 모두 반영했다.
