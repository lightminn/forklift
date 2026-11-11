# 도착일 절차서 — S0 첫 굽기부터 S5 안전 동작까지

작성일: 2026-11-12 · 상태: **절차 초안, 실기 전**(부품 도착 11-20) · 관련: [실차 이식 계획](2026-11-11-sim-to-real-bringup.md) 5절(단계 표 정본), [보드 설계](../design/2026-11-12-weact-h743-board-firmware.md), [모터 출력 설계](../design/2026-11-12-weact-h743-motor-output.md), bring-up 도구 `tools/bringup.py`(주인: 콘솔 세션), udev `deploy/udev/README.md`

이 문서는 계획 5절의 S0–S5 를 **칠 명령** 단위로 풀어 쓴다. 단계의 합격·멈춤 조건의 정본은 계획 5절이다. 여기 적은 판정 필드는 그 조건을 도구 출력에서 읽는 방법이다. 명령·옵션은 콘솔 세션과 맞췄다(2026-11-12).

## 0. 공통 규칙

- **멈춤**: 단계의 멈춤 조건이 하나라도 생기면 그 단계에서 멈추고, 원인과 기록 경로를 남긴 뒤 총괄에 올린다. 다음 단계로 넘어가지 않는다.
- **결정 경계** — 아래는 이 절차가 대신 넘지 않는다.
  - 모터 전원(VM)은 IPROPI 보호 확인 전에는 S2 의 VM 투입 순서로만 넣는다(사용자 결정 2026-11-11).
  - 출력이 열린 빌드는 **사용자가 쓰고 커밋한 보호 확인 기록**이 있어야 구성된다(모터 출력 설계 2절). 그 기록을 쓰는 것이 사용자 결정(Q6)이고, 에이전트는 만들지 않는다.
  - 지금 출력을 열 수 있는 빌드는 **시운전(`commission`) 하나뿐**이다. 정상 빌드의 출력은 비상정지 입력 핀이 정해진 뒤 따로 결정한다. 그래서 **S4 와 S5 의 정상 빌드 시나리오(`tx_cut`·`disarm`·`expiry`)는 그 결정 전에는 할 수 없다**(7·8절).
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
| 오실로스코프 2채널 이상, 전류 프로브 또는 션트 | S3 전원 강하, S5 IWDG ② | 그 측정만 미룬다(S5 는 ①만) |
| 받침대(바퀴 띄움)와 차체를 받침대에 묶는 래칫 스트랩·클램프 | S3–S5 | 진행 불가 |
| 손에 쥐는 주 전원 차단 스위치 — **DC 전압·전류 정격**이 배터리 전압·모터 전류 이상인 것 | S3–S5 | 진행 불가 |
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
5. (6번의 PB4 결정이 난 뒤) **모든 전원을 끄고**(벤치 전원·USB), 레일이 내려간 것을 확인한 뒤 MCU 를 꽂는다. 그다음 전원을 넣고 부팅한다.
6. **PB4 — SDO high 전압: 결정 대기**(Codex 2라운드). 사용자 결정(2026-11-11)은 "실측해서 3.6 V 이하면 플래시를 둔다"이다. 그런데 DRV8244S 의 SDO high 는 nSLEEP 전압을 따라 nSLEEP = 3.3 V 에서 부하 없이 최대 3.8 V 다(SLVSG24C 7.5.2 `VOH_SDO_NL`). 측정하려면 드라이버를 깨우고 SPI 를 돌려야 하므로 **측정이 곧 플래시의 첫 노출**이다. 총괄에 재결정을 요청했다(권고: 첫 전원 전에 쓰지 않는 SPI 플래시를 뺀다). **결정이 오기 전에는 5번(MCU 를 꽂고 부팅)으로 가지 않는다.** 결정 뒤 측정은 오실로스코프로 SPI 가 도는 동안 PB4 의 high 최고값을 읽는다(멀티미터 평균값은 쓰지 않는다).

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
- **KTH CRC 변형 확정**(계획 S2): **지금 도구로는 할 수 없어 미완료**다(Codex 2라운드). STATE 의 `rx_errors` 는 링크 프레임 오류이고, KTH 의 CRC 오류 계수는 STATE 에 실리지 않는다. 또 CRC 실패가 연속 3 회 미만이면 코어가 이전 유효 위치를 유지하므로, 값이 NaN 이 아니라는 것만으로 변형이 맞다고 할 수 없다. 원시 KTH 프레임이나 CRC 오류 계수를 내보내는 경로가 생기면(계약 변경, 후속) 그때 확정한다. 지금 볼 수 있는 것은 값이 계속 무효(NaN)이면 변형이 틀렸다는 반대 방향 증거뿐이다 — 그러면 멈추고 다른 변형으로 다시 굽는다.

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

**지금은 할 수 없다.** `bringup.py step` 은 정상 빌드에 `DRIVE` 를 보내는데, 출력이 열린 정상 빌드는 비상정지 입력 핀이 정해진 뒤 따로 결정한다(모터 출력 설계 2절). 결정 뒤 명령은 이렇다.

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
  - 한 프로세스가 링크를 쥐고 구동한다: `python tools/bringup.py commission --device /dev/forklift-mcu --out $D/run.jsonl --axis L --duty 0.05 --seconds 1.5 --power-switch-in-hand`. 바퀴가 도는 동안 외부 리셋 버튼을 누른다. 두 프로세스가 같은 tty 를 열면 수신을 나눠 읽으므로 `record` 를 함께 돌리지 않는다(Codex).
  - 오실로스코프로 잴 것: MTR_SLEEP·DRVOFF·EN·PH 핀, 모터 단자 전압, 모터 전류. 트리거는 NRST 하강.
  - 합격: 리셋 뒤 MTR_SLEEP 이 low 로 가서 전 채널이 SLEEP(출력 Hi-Z)이 되고, 모터 전류가 0 으로 간다. 리셋 순간의 반대 방향 펄스 유무와 그 길이를 기록한다.
  - 이 실행의 도구 결과는 **중단이 정상**이다: 의도한 리셋으로 부팅이 바뀌므로 `aborted == "boot_changed"`, `result == "aborted"` 가 나온다(Codex 2라운드). 0절의 공통 합격 조건은 여기 적용하지 않고, 합격은 위 외부 측정으로만 한다. 재부팅 확인(`reset_cause == "PIN"`)은 리셋 뒤 따로 `record --seconds 10` 을 돌려 그 기록에서 본다.
- **IWDG 판정**(계획 3.4: 주기 정지부터 출력 정지까지 100 ms): ①의 계산값 최대 53.8 ms + 주기 1 ms + ②에서 잰 리셋→차단 시간이 100 ms 안이어야 한다. 셋을 기록하고, 합이 100 ms 안이라는 근거가 없으면 판정을 보류한다. 한계: NRST 버튼과 IWDG 의 리셋 펄스 길이가 다르다(총괄 결정 (A)). 오실로스코프가 없으면 ②와 이 판정은 미룬다.
- NRST 를 GND 로 붙잡은 상태에서도 위 핀 전압을 잰다. 계산과 다르면 멈추고 올린다(모터 출력 설계 4.1).

**주 전원 차단**(`fw-open`, 받침대): ②와 같이 `commission --seconds 1.5` 로 한 바퀴를 돌리는 동안 차단 스위치를 내린다. 이 실행도 **중단이 정상**이고(공통 합격 조건 적용 안 함), 그 `run.jsonl` 에서는 마지막 STATE 시각만 읽는다. 전원을 다시 넣은 뒤 따로 `record --seconds 10` 을 돌려 `reset_cause == "POWER"` 를 본다. 바퀴가 서는 시간·거리는 영상이나 줄자로 잰다. 구동 중 사건 표시를 하나의 프로세스로 남기는 도구 경로는 아직 없다 — 지금은 이 기록과 외부 측정으로 판정하고, 도구 보강은 콘솔 세션에 넘긴다.

### 9.2 결정 뒤에 할 것

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
