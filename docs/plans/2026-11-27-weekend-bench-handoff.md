# 주말 벤치 인수인계 (2026-11-27) — WeAct H743 보드·M 축 시험

이 문서 하나로 주말 벤치를 이어간다. 팀원이 자기 노트북의 Claude Code 로 이 저장소를 열고 이 문서를 맥락으로 쓰는 것을 전제로 썼다.

## 0. Claude 용 지침 (먼저 읽을 것)

- **이 문서가 정본이다.** 다른 문서·기억과 다르면 이 문서를 따르고, 다른 점을 사람에게 알린다.
- **모터를 돌리는 명령은 사람 확인 뒤에 실행한다.** `commission` 명령(모터 구동)마다 "차단 스위치를 손에 쥐었는가"를 사람에게 묻고 답을 받은 뒤 실행한다. 상태 읽기(`record`)와 빌드는 묻지 않아도 된다.
- **멈추는 쪽의 보호는 끄지 않는다.** 과전류·드라이버 결함·STOP·DISARM·명령 만료·워치독은 우회할 방법이 없고, 만들지도 않는다. 이 문서에 없는 `--bench-bypass` 값을 쓰지 않는다.
- **비밀번호는 사람이 직접 입력한다.** `sudo`·`ssh` 비밀번호를 명령에 넣거나 파일에 쓰지 않는다. 계정 정보는 팀 공유 계정 정보에 있다.
- **이미지와 도구는 짝을 맞춘다**(3 절). 보드 제품 문자열의 끝 토큰(`bench3`/`bench4`)과 도구 체크아웃의 태그가 다르면 구동하지 않는다.
- 판정은 아래 숫자 기준으로 한다. 기준을 벗어나면 다음 단계로 가지 말고 사람에게 결과를 보인다.

## 1. 하드웨어 구성·배선

회로도·배선 그림은 **팀 원본 회로도**(팀 드라이브)를 본다. 이 저장소에는 넣지 않는다. 아래는 확인용 핀 표다.

| 연결 | 내용 |
|---|---|
| 메인보드 ↔ WeAct H743 | 메인보드에 꽂은 WeAct 보드(펌웨어가 도는 MCU) |
| VM 12 V | 모터 전원. **차단 스위치를 손에 쥔 채** 켠다 |
| Jetson ↔ WeAct USB-C | mcu-link(USB CDC). 보드는 `/dev/forklift-mcu` 로 보인다 |
| RJ1 | 조향 KTH(KTH7801 절대각 센서). 지금 벤치에서는 이 센서를 **M 모터 축에 시험 장착**해 M 의 회전을 센다 |
| CN3 | L 모터 |
| CN5 | M 모터 |

**KTH ↔ STDS 점퍼 6 가닥** — 같은 번호끼리 잇는다.

| CN1 핀 | 신호 |
|---|---|
| 1 | VCC |
| 2 | CS |
| 3 | 연결 안 함 |
| 4 | CLK |
| 5 | 연결 안 함 |
| 6 | MOSI |
| 7 | GND |
| 8 | MISO |

- **STDS 의 U8 은 반드시 LVDS32** 여야 한다(근거: [2026-11-27 KTH LVDS 기록](../validation/2026-11-27-bench-kth-lvds.md)).
- 메인보드 U8 의 MISO 종단 저항은 선택이다(있어도 없어도 됨).
- **핀헤더 접촉 불량이면 R/M/H 드라이버 초기화가 실패한다**(아래 상태 읽기에서 `driver_status` 가 `192,0,0,0`, 모드 `BOOT`). 그때는 모터 등을 **전부 빼고** 전원을 다시 켠 뒤, 하나씩 꽂으며 매번 전원을 다시 켠다. 11-27 에는 핀헤더를 다시 납땜해 회복했다.
- **KTH 점퍼 납땜이 약해 자꾸 떨어졌다**(11-27). 상태 읽기에서 KTH CRC 오류가 늘면 점퍼부터 본다.
- **차단 스위치는 항상 손에 쥔다.** 이상한 소리·냄새·움직임이면 바로 VM 을 끊는다.

## 2. 접속 경로 — 둘 중 하나

### 2.1 Jetson 을 거쳐 (11-27 까지 쓴 경로)

- Jetson 호스트 이름 `forklift-jetson.local`, 계정·비밀번호는 팀 공유 계정 정보. Jetson 과 팀원 노트북이 **같은 Wi-Fi** 에 있어야 한다. USB-C 로 Jetson 을 노트북에 꽂으면 `192.168.55.1` 로도 붙는다.
- 이 경로에서는 모든 명령을 Jetson 에서 실행한다(`ssh <계정>@forklift-jetson.local` 뒤).

```bash
ssh <계정>@forklift-jetson.local    # 비밀번호는 사람이 입력
```

### 2.2 보드를 팀원 노트북에 직접 USB 로

- WeAct USB-C 를 노트북에 꽂는다. Linux 노트북이어야 한다(udev 규칙·dfu-util).
- 아래 4 절의 설치를 노트북에서 하고, 모든 명령을 노트북에서 실행한다.
- udev 규칙(한 번, 사람이 sudo 입력): `deploy/udev/README.md` 를 따른다. 핵심은 다음이다.

```bash
sudo install -m 0644 deploy/udev/99-forklift-mcu.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb
ls -l /dev/forklift-mcu    # 보드를 꽂은 뒤 있어야 한다
```

- 사용자가 직렬 포트 그룹에 있어야 한다(Ubuntu `dialout`, Arch `uucp`). 없으면 `sudo usermod -aG dialout $USER` 뒤 다시 로그인한다.

## 3. 이미지와 도구 — 어느 것을 쓰나

- **지금 보드에 구워진 것:** `diag-fw-bench-2`. 제품 문자열 끝은 `lim=lr1 bench3` 이다.
- **기본 경로 (이것을 쓴다):** 보드를 **`diag-fw-bench-3`**(감쇠 항 `--hold-kd` 추가, 끝 `bench4`)으로 굽고, 도구도 **같은 태그 `diag-fw-bench-3`** 체크아웃에서 실행한다.
- **굽지 않을 때:** `diag-fw-bench-2` 그대로 쓰려면 도구도 **`diag-fw-bench-2`** 체크아웃에서 실행한다(`--hold-kd` 없음).
- 도구는 다른 세대의 벤치 이미지를 거부한다. 그래도 실행 전에 아래 짝 확인을 한다.

| 태그 | 제품 문자열 | 내용 |
|---|---|---|
| `diag-fw-bench-2` | `forklift-mcu commission OPEN diag-fw-bench-2 rec=1f4207f4 lim=lr1 bench3` | BENCH_SET·KTH 이동량 카운터·BENCH_HOLD(M 위치 유지, PI, 최소 듀티). **지금 보드** |
| `diag-fw-bench-3` | `forklift-mcu commission OPEN diag-fw-bench-3 rec=1f4207f4 lim=lr1 bench4` | 위 + hold 감쇠 항 `--hold-kd`. **빌드만, 미시험** |

## 4. 설치 — 새 체크아웃 `~/forklift-bench`

**기존 폴더(`~/forklift`, `~/forklift-diag-commission-long`, `~/forklift-fw-bench`, `~/forklift-fw-bench2`)는 쓰지 않는다.** 아래로 새로 만든다(Jetson 이든 노트북이든 같음).

```bash
TAG=diag-fw-bench-3          # 굽지 않고 bench-2 를 쓰면 diag-fw-bench-2
git clone https://github.com/lightminn/forklift.git ~/forklift-bench
cd ~/forklift-bench
git fetch --tags origin
git checkout --detach "$TAG"
git describe --tags --exact-match    # → 위 TAG 가 나와야 한다
python3 -m venv .venv
.venv/bin/pip install -U pip
.venv/bin/pip install -e '.[real,dev]'
.venv/bin/python tools/bringup.py commission --help | grep -c -- '--hold-kd'   # bench-3: 1, bench-2: 0
```

- 굽기에 필요한 것: `sudo apt install dfu-util`(Jetson 에는 0.11 이 이미 있음).

## 5. 펌웨어 — 빌드(주 경로) 또는 Release bin(대안)

### 5.1 직접 빌드 (주 경로, Jetson 또는 노트북)

툴체인은 **Arm GNU Toolchain 13.3.rel1** 이어야 한다. 빌드가 버전을 확인해 다른 판(예: apt `gcc-arm-none-eabi`)은 거부한다. 먼저 이미 있는지 본다.

```bash
ls ~/opt/arm-gnu-toolchain-13.3.rel1-*/bin/arm-none-eabi-gcc 2>/dev/null && ~/opt/arm-gnu-toolchain-13.3.rel1-*/bin/arm-none-eabi-gcc --version | head -1
# "arm-none-eabi-gcc (Arm GNU Toolchain 13.3.Rel1 ...)" 이면 설치 건너뜀
```

없으면 설치한다. 아래는 Jetson(aarch64)이고, x86_64 노트북은 파일 이름의 `aarch64` 를 `x86_64` 로 바꾼다.

```bash
mkdir -p ~/opt && cd ~/opt
F=arm-gnu-toolchain-13.3.rel1-aarch64-arm-none-eabi.tar.xz
curl -LO https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel/$F
echo "c8824bffd057afce2259f7618254e840715f33523a3d4e4294f471208f976764  $F" | sha256sum -c -   # aarch64 판의 값
tar xf $F
~/opt/arm-gnu-toolchain-13.3.rel1-aarch64-arm-none-eabi/bin/arm-none-eabi-gcc --version | head -1
```

빌드(벤치 이미지, `~/forklift-bench` 에서):

```bash
cd ~/forklift-bench
TC=$(ls -d ~/opt/arm-gnu-toolchain-13.3.rel1-*-arm-none-eabi | head -1)
sudo apt install -y cmake    # 없을 때
cmake -S firmware -B build/bench -DFW_BOARD=weact_h743 \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/firmware/boards/weact_h743/arm-gnu-13.3.cmake -DARM_GNU_TOOLCHAIN_DIR=$TC \
  -DFW_PYTHON=$PWD/.venv/bin/python \
  -DFW_BUILD=commission -DFW_MOTOR_OUTPUT=1 -DFW_BENCH=1 \
  -DFW_IPROPI_PROTECTION_RECORD=docs/validation/2026-11-16-ipropi-protection-record.md
cmake --build build/bench -j4
$TC/bin/arm-none-eabi-objcopy -O binary build/bench/boards/weact_h743/fw.elf fw-bench.bin
strings -n 12 build/bench/boards/weact_h743/fw.elf | grep -m1 '^forklift-mcu '   # 3 절의 제품 문자열과 같아야 함
sha256sum fw-bench.bin
```

- 태그 그대로 빌드하면 제품 문자열은 3 절 표와 같다. 펌웨어를 고치면 끝 토큰은 같고 가운데가 `diag-fw-bench-3-<n>-g<해시>`(커밋했을 때) 또는 `-dirty` 로 바뀐다.
- 노트북(x86_64)에서 11-27 에 빌드한 태그 이미지의 크기·sha256 은 5.2 표와 같다. 같은 툴체인 판이면 Jetson 빌드도 크기가 같아야 한다(sha256 은 다를 수 있음).

### 5.2 Release bin (빌드가 안 될 때)

GitHub Release 에 올려 두었다. 받은 뒤 sha256 을 반드시 대조한다.

| 파일 | Release(태그) | 제품 문자열 | 크기(B) | sha256 |
|---|---|---|---|---|
| `fw-bench-3.bin` | `diag-fw-bench-3` | `… diag-fw-bench-3 rec=1f4207f4 lim=lr1 bench4` | 36,396 | `b61d4bb267a87277953dc1bb073cd49590c9c021507cd3153a5f904a4957f47b` |
| `fw-bench-2.bin` | `diag-fw-bench-2` | `… diag-fw-bench-2 rec=1f4207f4 lim=lr1 bench3` | 36,116 | `d18d33325c79438a281bddff35f3c62154db553ec523ff4280993b2baaa11a4a` |
| `fw-open.bin` | `diag-fw-bench-3` (되돌리기용) | `forklift-mcu commission OPEN add5683ed6e7 rec=1f4207f4 lim=lr1` | 35,144 | `1de51e6153a3b88dfa3a9d36ba6a32f7a0535a62784de9474ed769c447c59741` |
| `fw-single.bin` | `diag-fw-bench-3` (되돌리기용) | `forklift-mcu commission OPEN add5683ed6e7 rec=1f4207f4 lim=lr1 sa1` | 35,296 | `dddd70eeee55cf533aa883f81a97e3ca29e197616123503a1074809c9a295833` |

- `fw-open`·`fw-single` 은 main(add5683e)의 정상 시운전 이미지다. 벤치 시험이 끝나면 이것으로 되돌린다. `fw-single` 은 한 채널·KTH 하나 구성용이다(`lim=lr1 sa1`).

```bash
cd ~/forklift-bench
R=https://github.com/lightminn/forklift/releases/download
curl -LO $R/diag-fw-bench-3/fw-bench-3.bin
echo "b61d4bb267a87277953dc1bb073cd49590c9c021507cd3153a5f904a4957f47b  fw-bench-3.bin" | sha256sum -c -
```

### 5.3 굽기 (dfu-util, sudo 는 사람이 입력)

1. **DFU 모드:** WeAct 의 BOOT0 를 누른 채 RESET 을 눌렀다 떼고, BOOT0 를 뗀다.
2. 장치 확인 — 정확히 한 줄이어야 한다(다른 STM32 보드는 빼 둔다).

```bash
lsusb | grep -c 0483:df11    # → 1
```

3. 굽기·검증·빠져나가기(`BIN` 은 굽는 파일):

```bash
BIN=fw-bench.bin             # Release 를 쓰면 fw-bench-3.bin
sudo dfu-util -a 0 -s 0x08000000 -D "$BIN"
rm -f readback.bin
sudo dfu-util -a 0 -s 0x08000000:$(stat -c %s "$BIN") -U readback.bin
cmp "$BIN" readback.bin && echo VERIFY_OK
rm -f leave.bin
sudo dfu-util -a 0 -s 0x08000000:4:leave -U leave.bin; echo "exit $?"   # 74 는 정상(장치가 응답 전에 떠남)
```

- 판정: `VERIFY_OK` 가 나와야 한다. 몇 초 뒤 `ls -l /dev/forklift-mcu` 가 있어야 한다.
- 실패하면 1 부터 다시 한다. 옵션 바이트는 건드리지 않는다(위 명령은 쓰지 않음).

## 6. 시험 명령

모두 `~/forklift-bench` 에서 실행한다. 실행마다 새 기록 폴더를 만든다.

```bash
cd ~/forklift-bench
mkdir -p ~/bench_runs
D=~/bench_runs/<이름>_$(date +%m%d_%H%M%S); mkdir -p $D
```

### 6.1 상태 읽기 + 짝 확인 (매번 구동 전)

```bash
D=~/bench_runs/status_$(date +%m%d_%H%M%S); mkdir -p $D
.venv/bin/python tools/bringup.py record --device /dev/forklift-mcu --out $D/run.jsonl --seconds 5 > $D/record.log 2>&1
.venv/bin/python - $D/run.jsonl <<'EOF'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
prod = [r["product"] for r in rows if r.get("kind") == "connected"]
st = [r for r in rows if r.get("t") == "state"]
a, b = st[0], st[-1]
print("product:", prod[-1] if prod else None)
print("mode:", b["mode"], "| driver_status:", b["driver_status"], "| flags:", b["flags"])
print("faults:", b["fault_flags"])
print("steer KTH frames +%d, CRC errors +%d" % (b["kth_frames"][0] - a["kth_frames"][0],
                                               b["kth_crc_errors"][0] - a["kth_crc_errors"][0]))
print("steering_rad first/last: %.3f / %.3f" % (a["steering_rad"], b["steering_rad"]))
EOF
git describe --tags --exact-match
```

판정:
- **짝**: `product` 끝이 `bench4` 이면 `git describe` 가 `diag-fw-bench-3` 이어야 한다. `bench3` 이면 `diag-fw-bench-2` 여야 한다. 다르면 멈춘다.
- **드라이버 4 개**: `driver_status` 가 `[192, 192, 192, 192]`(0xC0) 이고 `flags` 에 `DRIVERS_READY`, `mode` 가 `DISARMED` 여야 한다. `[192, 0, 0, 0]`·`BOOT` 이면 1 절 접촉 불량 절차를 따른다.
- **KTH CRC**: 5 s 동안 프레임이 약 500 늘고 CRC 오류 증가가 **0** 이어야 한다. 늘면 점퍼를 본다.
- **자석**: 손으로 M 축을 돌리면서 한 번 더 읽어, `steering_rad` 가 변하고 CRC 오류 증가가 0 이어야 한다(`STEER_OUT_OF_RANGE` 는 KTH 가 조향이 아니라 M 축에 달려 있어서 뜰 수 있다 — 벤치에서는 `--bench-bypass steer_range` 로 끈다).
- `LIFT_SENSOR_INVALID`(승강 센서 없음), `ESTOP_UNWIRED`, `BENCH_KTH_LOST`(첫 ARM 전) 는 벤치에서 정상이다.

### 6.2 M 구동 (commission)

**사람 확인: 차단 스위치를 손에 쥐었는가.** M 은 코깅이 강해 0.12 에서는 걸려 멈추고(`kth_no_motion`), 0.20 에서 매끈하게 돈다(약 20 바퀴/s).

```bash
D=~/bench_runs/m_020_$(date +%m%d_%H%M%S); mkdir -p $D
.venv/bin/python tools/bringup.py commission --device /dev/forklift-mcu --out $D/run.jsonl \
  --axis M --duty 0.20 --seconds 20 --stop-kth-turns 3 \
  --bench-bypass steer_range --bench-duty-max 0.25 --bench-time-max 30 \
  --power-switch-in-hand --yes --observed none > $D/stdout.json 2> $D/stderr.txt; echo "exit $?"
```

- 판정(11-27 결과 기준): `stdout.json` 의 정지 사유가 `kth_turns`, 약 +3 바퀴에서 섬. 관성으로 0.6 바퀴쯤 더 돈다. I_M 평균 약 1.1 A·최고 약 3.6 A(기동). 끝 모드 DISARMED, 과전류·`DRIVER_FAULT_M` 없음.
- 처음이면 `--duty 0.12 --seconds 1` 짧은 펄스부터 해도 된다(코깅에 걸릴 수 있음).

### 6.3 L 구동

**사람 확인: 차단 스위치.** L 은 **0.05 부터** 올린다(바퀴를 띄운 상태).

```bash
D=~/bench_runs/l_005_$(date +%m%d_%H%M%S); mkdir -p $D
.venv/bin/python tools/bringup.py commission --device /dev/forklift-mcu --out $D/run.jsonl \
  --axis L --duty 0.05 --seconds 2 --power-switch-in-hand --wheels-lifted --yes > $D/stdout.json 2> $D/stderr.txt; echo "exit $?"
```

- 판정: 바퀴가 듀티 부호대로 돌고, 결함 없이 DISARMED 로 끝난다. 문제가 없으면 0.10 → 0.15 순으로 올린다.

### 6.4 M 위치 유지 (hold)

**사람 확인: 차단 스위치.** 손으로 축을 비틀 때는 장갑을 끼고, 구속 전류(11-27: 4.8 A 에서 `DRIVER_FAULT_M`)에 주의한다.

```bash
D=~/bench_runs/hold_$(date +%m%d_%H%M%S); mkdir -p $D
.venv/bin/python tools/bringup.py commission --device /dev/forklift-mcu --out $D/run.jsonl \
  --axis M --duty 0.15 --seconds 10 --hold-target here \
  --hold-kp 0.5 --hold-kd 0.02 --hold-deadband 12 --hold-sign 1 \
  --bench-bypass steer_range --bench-duty-max 0.25 --bench-time-max 30 \
  --power-switch-in-hand --yes --observed none > $D/stdout.json 2> $D/stderr.txt; echo "exit $?"
```

- `--hold-target`: `here`(지금 자리), `zero`(가장 가까운 센서 원시 0), 또는 `±카운트`(4096 = 한 바퀴).
- **권장 시작값:** 상한 `--duty 0.15`, `--hold-kp 0.5`, `--hold-kd 0.02`(bench-3 만), 데드밴드 12. 10 s.
  - **감쇠 항은 실물 미시험이다.** 호스트 시험은 kp 2.0·kd 0.05 에서 넘침을 줄였다. 진동이 남으면 kd 를 0.05 → 0.1 로 올려 본다. 축이 떨거나 반대로 밀면 바로 멈추고 kd 를 0 으로 되돌린다.
  - bench-2 이면 `--hold-kd` 를 빼야 한다.
- **판정(`stdout.json` 의 hold 요약):**
  - 목표 대비 최종 오차 ≤ 데드밴드(12 카운트, 약 1°), 데드밴드 안 비율이 높을수록 좋다.
  - **진동:** 오차 부호 전환이 5 s 에 5 번 이상이면 진동이다(11-27 kp 0.5·kd 0: 5–18 번).
  - 결함 없음(`DRIVER_FAULT_M` 이면 6.5 로 해제).
- `zero` 목표는 11-27 에 마지막 약 2°(24 카운트)에서 멈췄다. `--hold-min-duty 0.05` 나 `--hold-ki` 로 밀어 볼 수 있다(미시험).

### 6.5 정지·고장 해제

- **멈추기:** 실행 중 `Ctrl-C`(도구가 DISARM 을 보내고 끝냄). 급하면 **차단 스위치**.
- **고장 해제**(`FAULT`·`ESTOP` 일 때만):

```bash
D=~/bench_runs/clear_$(date +%m%d_%H%M%S); mkdir -p $D
.venv/bin/python tools/bringup.py clear-faults --device /dev/forklift-mcu --out $D/run.jsonl --cause "<무엇을 했는지>"
```

- 해제 뒤 6.1 상태 읽기로 `DISARMED`·드라이버 0xC0 를 확인한다.

## 7. 지금까지의 결과와 알려진 문제

근거 기록: [11-27 fw-bench](../validation/2026-11-27-bench-fw-bench-m-hold.md), [11-27 KTH LVDS](../validation/2026-11-27-bench-kth-lvds.md), [11-26 L 실모터](../validation/2026-11-26-bench-l-real-motor.md).

- M 3바퀴: 0.20 정상(`kth_turns` 정지), 0.12 코깅에 걸림.
- **hold 진동:** P 만(kp 0.5)이면 ±1,000–2,000 카운트 왕복. 감쇠 항(bench-3)은 이를 줄이려고 넣었고 미시험이다.
- **zero 2° 앞 정지:** 남은 오차에서 P 출력이 코깅을 못 넘는다(추정).
- **MISO low 고착 위험:** bench-2/3 은 직전 각이 원시 0 근처(±512)이면 0x0000 프레임도 센다. 이때 MISO 가 low 로 붙으면 카운터가 가짜 이동(예: −256)을 읽고, hold 가 듀티 상한까지 밀 수 있다. 3 ms 손실 정지도 걸리지 않는다. **원시 0 근처에서 hold 할 때는 차단 스위치를 특히 손에 쥔다.**
- **KTH 점퍼 납땜 이탈:** 사용자가 납땜한 점퍼가 약해 떨어졌다. CRC 오류가 늘면 여기부터 본다.
- **R/M/H 초기화 실패:** 핀헤더 접촉 불량. 1 절 절차로 회복.

## 8. 기록과 결과 돌려주기

- 기록은 실행마다 `~/bench_runs/<이름>_<시각>/` 의 `run.jsonl`·`stdout.json`·`stderr.txt` 다.
- 주말이 끝나면 폴더를 묶어 팀 드라이브에 올리고, 한 줄 요약(무엇을 했고 판정이 어땠는지)을 남긴다.

```bash
tar cJf ~/bench_runs_$(date +%m%d).tar.xz -C ~ bench_runs
sha256sum ~/bench_runs_$(date +%m%d).tar.xz
```

- 펌웨어나 도구를 고쳤다면 새 브랜치로 커밋해 push 하고(main 에 직접 push 하지 않음), 어느 커밋을 구웠는지(제품 문자열)를 요약에 적는다.
- **끝나면 정상 이미지로 되돌린다**: 5.2 의 `fw-open.bin`(두 채널) 또는 `fw-single.bin`(한 채널)을 5.3 대로 굽고, 6.1 로 제품 문자열이 `lim=lr1`(또는 `lim=lr1 sa1`)으로 끝나는지 확인한다. 이때 도구는 main 체크아웃을 쓴다(벤치 태그 도구는 정상 이미지에 BENCH_SET 을 보내지 않지만, 짝을 맞춘다).
