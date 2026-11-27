# 주말 벤치 인수인계 (2026-11-27) — WeAct H743 보드·M 축 시험

이 문서 하나로 주말 벤치를 이어간다. 팀원이 자기 노트북의 Claude Code 로 이 저장소를 열고 이 문서를 맥락으로 쓰는 것을 전제로 썼다.

## 0. Claude 용 지침 (먼저 읽을 것)

- **이 문서가 정본이다.** 다른 문서·기억과 다르면 이 문서를 따르고, 다른 점을 사람에게 알린다.
- **도구의 종료 코드로 판정하지 않는다.** 거부·중단에도 0 일 수 있다. 결과 파일로 판정한다(6.2 의 `chk`).
- **비밀번호는 사람이 직접 입력한다.** `sudo`·`ssh` 비밀번호를 명령에 넣거나 파일에 쓰지 않는다. 계정 정보는 팀 공유 계정 정보에 있다.
- **이미지와 도구는 짝을 맞춘다**(3·4 절). 보드 제품 문자열의 끝 토큰(`bench3`/`bench4`)과 도구 폴더의 표식이 다르면 구동하지 않는다. 도구 폴더를 고치지 않는다(4.2 로 확인).
- 판정은 아래 숫자 기준으로 한다. 기준을 벗어나면 다음 단계로 가지 말고 사람에게 결과를 보인다.

## 1. 하드웨어 구성·배선

회로도·배선 그림은 **팀 원본 회로도**(팀 드라이브)를 본다. 이 저장소에는 넣지 않는다. 아래는 확인용 핀 표다.

| 연결 | 내용 |
|---|---|
| 메인보드 ↔ WeAct H743 | 메인보드에 꽂은 WeAct 보드(펌웨어가 도는 MCU) |
| VM 12 V | 모터 전원(차단 스위치) |
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
- `--bench-bypass steer_range` 는 조향 범위 정지를 끈다(11-27 M 시험은 모터 단독). 3바퀴 시험은 관성으로 +3.67 바퀴까지 돌았다.

## 2. 접속 경로 — 둘 중 하나

### 2.1 Jetson 을 거쳐 (11-27 까지 쓴 경로)

- Jetson 호스트 이름 `forklift-jetson.local`, 계정·비밀번호는 팀 공유 계정 정보. Jetson 과 팀원 노트북이 **같은 Wi-Fi** 에 있어야 한다. USB-C 로 Jetson 을 노트북에 꽂으면 `192.168.55.1` 로도 붙는다.
- 이 경로에서는 모든 명령을 Jetson 에서 실행한다(`ssh <계정>@forklift-jetson.local` 뒤).

```bash
ssh <계정>@forklift-jetson.local    # 비밀번호는 사람이 입력
```

### 2.2 보드를 팀원 노트북에 직접 USB 로

- WeAct USB-C 를 노트북에 꽂는다. Linux 노트북이어야 한다(udev 규칙·dfu-util).
- 4.3 의 설치(저장소 클론·udev 규칙)를 노트북에서 하고, 모든 명령을 노트북에서 실행한다.

## 3. 이미지와 도구 — 짝을 맞춘다

- **지금 보드에 구워진 것:** `diag-fw-bench-2`. 제품 문자열 끝은 `lim=lr1 bench3` 이다.
- **기본 경로:** 보드를 **`diag-fw-bench-3`**(감쇠 항 `--hold-kd` 추가, 끝 `bench4`)으로 굽고 도구는 **`~/forklift-fw-bench3`** 에서 실행한다.
- **굽지 않을 때:** `diag-fw-bench-2` 그대로 쓰면 도구는 **`~/forklift-fw-bench2`** 에서 실행한다(`--hold-kd` 없음).
- 도구는 다른 세대의 벤치 이미지를 거부한다. 그래도 실행 전에 아래 짝 확인(4.2)을 한다.

| 보드 이미지(태그) | 제품 문자열 끝 | 도구 폴더(Jetson) | 내용 |
|---|---|---|---|
| `diag-fw-bench-2` | `… diag-fw-bench-2 rec=1f4207f4 lim=lr1 bench3` | `~/forklift-fw-bench2` | BENCH_SET·KTH 이동량 카운터·BENCH_HOLD(M 위치 유지, PI, 최소 듀티). **지금 보드** |
| `diag-fw-bench-3` | `… diag-fw-bench-3 rec=1f4207f4 lim=lr1 bench4` | `~/forklift-fw-bench3` | 위 + hold 감쇠 항 `--hold-kd`. **빌드만, 미시험** |

## 4. Jetson 의 폴더와 실행 방법

### 4.1 폴더

| 폴더 | 무엇 | 이번에 |
|---|---|---|
| `~/forklift-fw-bench2` | 태그 `diag-fw-bench-2` 의 `git archive` 를 푼 것(git 없음) | bench-2 보드의 도구 |
| `~/forklift-fw-bench3` | 태그 `diag-fw-bench-3` 의 `git archive`(git 없음) | bench-3 보드의 도구 |
| `~/forklift-fw-bench` | 태그 `diag-fw-bench-1`(옛 세대, 표식 `bench2`) | 쓰지 않음 |
| `~/forklift` | 커밋 24a0475fe8e9 를 옮겨 둔 폴더(**git 없음**, 폴더의 `SYNCED_FROM` 파일에 커밋). 그 `.venv`(Python 3.12.3, `forklift-core` 편집 설치·NumPy·PyYAML)가 모든 벤치 도구를 실행하는 Python 이다 | **venv 만 쓴다. 폴더 내용은 바꾸지 않는다** |
| `~/forklift-diag-commission-long` | 11-26 L 긴 시운전용 도구. 태그가 아니라 브랜치 `diag/commission-long`(11-24)에서 옮긴 것이라 태그 `diag-commission-long-1` 과 세 파일이 다르다(`SYNCED_FROM`) | 쓰지 않음 |
| `~/fwbench_flash/` | 예전에 옮겨 둔 이미지·검증 사본·로그. `diag-fw-bench-2.bin`·`diag-fw-bench-3.bin` 의 sha256 은 5.2 Release 값과 같다(11-27 확인) | 굽기에 그대로 써도 됨 |
| `~/arrival_1124/` | 11-24 이후 벤치 기록 폴더(`run.jsonl` 등, 11-27 에 65 MB) | 새 기록은 `~/bench_runs/` 에 둔다 |

**Jetson 환경(11-27 확인)**:
- 계정 `zetin` 은 `dialout` 그룹이라 `/dev/forklift-mcu` 를 sudo 없이 연다(udev 규칙 `99-forklift-mcu.rules` 설치됨).
- `sudo` 는 비밀번호를 묻는다(굽기 `dfu-util` 만 sudo).
- `dfu-util` 0.11, `cmake` 3.28.3. 디스크 여유 약 211 GB.
- 시계는 NTP 로 맞는다. 예전 기록 폴더 이름에 `0101` 이 붙은 것은 부팅 직후 시계가 맞기 전이라서다. `/dev/forklift-mcu` 링크 시각이 1970 으로 보이는 것도 같은 이유이고 문제없다.
- Arm 툴체인: 11-27 에 `~/opt/arm-gnu-toolchain-13.3.rel1-aarch64-arm-none-eabi` 로 설치했고, 빌드 전용 클론 `~/forklift-fw-src`(태그 `diag-fw-bench-3`)도 있다 — 5.1.

도구 폴더는 `git archive` 라 git 이 없다. 그래서 venv 는 `~/forklift/.venv` 를 쓰고, `PYTHONPATH` 로 그 폴더의 `src` 를 먼저 읽게 한다. 아래 함수를 셸에 한 번 정의해 두고 쓴다(bash·zsh 둘 다 됨).

```bash
B=~/forklift-fw-bench3          # bench-2 보드면 ~/forklift-fw-bench2
PY=~/forklift/.venv/bin/python
br() { PYTHONPATH="$B/src" "$PY" "$B/tools/bringup.py" "$@"; }
cd "$B"
```

### 4.2 시작 때 확인 두 가지

1. **폴더의 태그 표식** — 3 절 표의 토큰과 맞아야 한다.

```bash
grep -o 'bench[0-9]' "$B/firmware/boards/weact_h743/gen_build_info.cmake" | sort -u   # fw-bench2: bench3, fw-bench3: bench4
```

   보드 제품 문자열 끝 토큰은 6.1 상태 읽기의 `product` 줄에서 본다. 둘이 같아야 구동한다.

2. **폴더에 손댄 변경이 없는지** — 같은 태그의 GitHub 압축본과 비교한다(`__pycache__`·빌드 폴더만 예외).

```bash
TAG=diag-fw-bench-3             # ~/forklift-fw-bench2 면 diag-fw-bench-2
rm -rf /tmp/tagcheck && mkdir /tmp/tagcheck
curl -sL https://github.com/lightminn/forklift/archive/refs/tags/$TAG.tar.gz | tar xz -C /tmp/tagcheck
diff -rq --no-dereference -x __pycache__ -x '*.pyc' -x build "/tmp/tagcheck/forklift-$TAG" "$B" && echo UNCHANGED
```

   판정: 마지막 줄이 `UNCHANGED` 여야 한다. 다른 줄이 나오면 그 파일이 바뀐 것이므로 사람에게 보인다(펌웨어·도구를 일부러 고친 경우가 아니면 쓰지 않는다). 이 GitHub 압축본은 `git archive <태그>` 와 같은 내용이다(2026-11-27 노트북에서 대조). 11-27 Jetson 에서 실제로 돌려 `~/forklift-fw-bench2`·`~/forklift-fw-bench3`(·`~/forklift-fw-bench`) 모두 차이 0 줄이었다.

### 4.3 보드를 팀원 노트북에 직접 꽂는 경우 (2.2)

노트북에는 위 폴더가 없으니 새로 만든다.

```bash
TAG=diag-fw-bench-3             # 보드 이미지와 같은 태그
git clone https://github.com/lightminn/forklift.git ~/forklift-bench
cd ~/forklift-bench && git fetch --tags origin && git checkout --detach "$TAG"
python3 -m venv .venv && .venv/bin/pip install -U pip && .venv/bin/pip install -e '.[real,dev]'
B=~/forklift-bench; PY=~/forklift-bench/.venv/bin/python
br() { PYTHONPATH="$B/src" "$PY" "$B/tools/bringup.py" "$@"; }
# udev 규칙(한 번, 사람이 sudo 입력; deploy/udev/README.md 와 같음)
sudo install -m 644 deploy/udev/99-forklift-mcu.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb
ls -l /dev/forklift-mcu    # 보드를 꽂은 뒤 있어야 한다
```

- 사용자가 직렬 포트 그룹에 있어야 한다(Ubuntu `dialout`, Arch `uucp`). 없으면 `sudo usermod -aG <그룹> "$USER"` 뒤 다시 로그인한다.
- 짝 확인은 `git describe --tags --exact-match`(→ `$TAG`)와 4.2 의 1 번으로 한다.
- 굽기에 필요한 것: `sudo apt install dfu-util`(Jetson 에는 0.11 이 이미 있음).

## 5. 펌웨어 — 빌드(주 경로) 또는 Release bin(대안)

### 5.1 직접 빌드 (주 경로, Jetson 또는 노트북)

툴체인은 **Arm GNU Toolchain 13.3.rel1** 이어야 한다. 빌드가 버전을 확인해 다른 판(예: apt `gcc-arm-none-eabi`)은 거부한다. 먼저 이미 있는지 본다.

```bash
ls ~/opt/arm-gnu-toolchain-13.3.rel1-*/bin/arm-none-eabi-gcc 2>/dev/null && ~/opt/arm-gnu-toolchain-13.3.rel1-*/bin/arm-none-eabi-gcc --version | head -1
# "arm-none-eabi-gcc (Arm GNU Toolchain 13.3.Rel1 ...)" 이면 설치 건너뜀
```

없으면 설치한다. 호스트 아키텍처(Jetson `aarch64`, 보통 노트북 `x86_64`)를 `uname -m` 으로 고르고, Arm 이 함께 내는 `.sha256asc` 로 검사가 통과해야만 푼다.

```bash
mkdir -p ~/opt && cd ~/opt
A=$(uname -m)                                   # aarch64 또는 x86_64
F=arm-gnu-toolchain-13.3.rel1-$A-arm-none-eabi.tar.xz
U=https://developer.arm.com/-/media/Files/downloads/gnu/13.3.rel1/binrel
curl -LO $U/$F && curl -LO $U/$F.sha256asc && sha256sum -c $F.sha256asc && tar xf $F && \
  ~/opt/arm-gnu-toolchain-13.3.rel1-$A-arm-none-eabi/bin/arm-none-eabi-gcc --version | head -1
# aarch64 판의 sha256 은 c8824bffd057afce2259f7618254e840715f33523a3d4e4294f471208f976764 (2026-11-27 확인)
```

빌드는 **git 체크아웃**에서 한다 — 제품 문자열의 이름(`git describe`)을 git 에서 읽기 때문에, git 이 없는 `~/forklift-fw-bench*` 폴더에서는 같은 문자열이 나오지 않는다. 빌드 전용 클론을 하나 만든다(도구는 계속 4 절 폴더에서 실행). 보드를 노트북에 직접 꽂는 경우(4.3)는 `~/forklift-bench` 가 이미 git 체크아웃이므로 거기서 빌드하고 `FW_PYTHON` 을 그 venv 로 준다.

```bash
TAG=diag-fw-bench-3
git clone https://github.com/lightminn/forklift.git ~/forklift-fw-src     # 처음 한 번
cd ~/forklift-fw-src && git fetch --tags origin && git checkout --detach "$TAG"
TC=$(ls -d ~/opt/arm-gnu-toolchain-13.3.rel1-*-arm-none-eabi | head -1)
sudo apt install -y cmake    # 없을 때
cmake -S firmware -B build/bench -DFW_BOARD=weact_h743 \
  -DCMAKE_TOOLCHAIN_FILE=$PWD/firmware/boards/weact_h743/arm-gnu-13.3.cmake -DARM_GNU_TOOLCHAIN_DIR=$TC \
  -DFW_PYTHON=$HOME/forklift/.venv/bin/python \
  -DFW_BUILD=commission -DFW_MOTOR_OUTPUT=1 -DFW_BENCH=1 \
  -DFW_IPROPI_PROTECTION_RECORD=docs/validation/2026-11-16-ipropi-protection-record.md
cmake --build build/bench -j4
$TC/bin/arm-none-eabi-objcopy -O binary build/bench/boards/weact_h743/fw.elf fw-bench.bin
strings -n 12 build/bench/boards/weact_h743/fw.elf | grep -m1 '^forklift-mcu '   # 3 절의 제품 문자열과 같아야 함
sha256sum fw-bench.bin
```

- 태그 그대로 빌드하면 제품 문자열은 3 절 표와 같다. 펌웨어를 고치면 끝 토큰은 같고 가운데가 `diag-fw-bench-3-<n>-g<해시>`(커밋했을 때) 또는 `-dirty` 로 바뀐다.
- **11-27 Jetson 에서 실제로 했다:**
  - 툴체인 받기·검사·풀기는 약 18 분 걸렸다(받기가 대부분).
  - 태그 `diag-fw-bench-3` 빌드 결과는 제품 문자열이 3 절 표와 같고 크기 36,396 B 로 Release 와 같았다.
  - sha256 은 `1dc4f0792ef9bce55fc95eef79485dfd049be5cc9ce22bc56b0f27e5f3c636bf` 로, 노트북 빌드(Release, `b61d4bb2…`)와 **다르다**(호스트가 달라서). Jetson 빌드는 sha256 대신 **제품 문자열과 크기**로 확인한다.
  - 툴체인과 `~/forklift-fw-src` 는 이미 있으므로, 다음부터는 위 빌드 블록의 `git clone` 줄만 건너뛰면 된다.

### 5.2 Release bin (빌드가 안 될 때)

GitHub Release 에 올려 두었다. 받은 뒤 sha256 을 반드시 대조한다.

| 파일 | Release(태그) | 제품 문자열 | 크기(B) | sha256 |
|---|---|---|---|---|
| `fw-bench-3.bin` | `diag-fw-bench-3` | `… diag-fw-bench-3 rec=1f4207f4 lim=lr1 bench4` | 36,396 | `b61d4bb267a87277953dc1bb073cd49590c9c021507cd3153a5f904a4957f47b` |
| `fw-bench-2.bin` | `diag-fw-bench-2` | `… diag-fw-bench-2 rec=1f4207f4 lim=lr1 bench3` | 36,116 | `d18d33325c79438a281bddff35f3c62154db553ec523ff4280993b2baaa11a4a` |
| `fw-open.bin` | `diag-fw-bench-3` (되돌리기용) | `forklift-mcu commission OPEN add5683ed6e7 rec=1f4207f4 lim=lr1` | 35,144 | `1de51e6153a3b88dfa3a9d36ba6a32f7a0535a62784de9474ed769c447c59741` |
| `fw-single.bin` | `diag-fw-bench-3` (되돌리기용) | `forklift-mcu commission OPEN add5683ed6e7 rec=1f4207f4 lim=lr1 sa1` | 35,296 | `dddd70eeee55cf533aa883f81a97e3ca29e197616123503a1074809c9a295833` |

- `fw-open`·`fw-single` 은 main(add5683e)의 정상 시운전 이미지다. 벤치 시험이 끝나면 이것으로 되돌린다. `fw-open` 은 실행마다 L/R/M/H 중 축을 고르고, `fw-single`(`lim=lr1 sa1`)은 한 번 ARM 하는 동안 고른 축 하나로 고정한다(한 채널·KTH 하나 구성용).

```bash
mkdir -p ~/fwbench_flash && cd ~/fwbench_flash
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
BIN=~/forklift-fw-src/fw-bench.bin   # Release 를 쓰면 ~/fwbench_flash/fw-bench-3.bin
mkdir -p ~/fwbench_flash && cd ~/fwbench_flash && rm -f readback.bin leave.bin
sudo dfu-util -a 0 -s 0x08000000 -D "$BIN" && \
sudo dfu-util -a 0 -s 0x08000000:$(stat -c %s "$BIN") -U readback.bin && \
cmp "$BIN" readback.bin && echo VERIFY_OK && \
{ sudo dfu-util -a 0 -s 0x08000000:4:leave -U leave.bin; echo "leave exit $?"; }   # 74 는 정상(장치가 응답 전에 떠남)
```

- 판정: `VERIFY_OK` 가 나온 뒤에만 빠져나가기(leave)가 실행된다. 나오지 않으면 보드는 DFU 에 남아 있다 — 1 부터 다시 한다.
- 빠져나간 뒤 몇 초 안에 `ls -l /dev/forklift-mcu` 가 있고, 6.1 의 `product` 가 구운 이미지의 제품 문자열이어야 한다(종료 코드 74 는 이 확인과 함께만 정상).
- 옵션 바이트는 건드리지 않는다(위 명령은 쓰지 않음).

## 6. 시험 명령

4.1 의 `B`·`PY`·`br` 를 정의한 셸에서 실행한다. 실행마다 새 기록 폴더를 만든다.

```bash
mkdir -p ~/bench_runs
```

실행 결과는 **종료 코드가 아니라 결과 파일로** 판정한다(거부·중단에도 종료 코드가 0 일 수 있음). 아래 `chk` 를 셸에 한 번 정의해 둔다.

```bash
chk() { "$PY" - "$1" <<'EOF'
import json, os, sys
d = sys.argv[1]
rows = [json.loads(l) for l in open(os.path.join(d, "run.jsonl"))]
out = None
if os.path.exists(os.path.join(d, "stdout.json")) and os.path.getsize(os.path.join(d, "stdout.json")):
    out = json.load(open(os.path.join(d, "stdout.json")))
res = [r for r in rows if r.get("t") == "result"]
out = out or (res[-1] if res else {})
st = [r for r in rows if r.get("t") == "state"]
last = st[-1] if st else {}
ok_faults = {"LIFT_SENSOR_INVALID", "STEER_OUT_OF_RANGE", "BENCH_OVERRIDES", "BENCH_KTH_LOST"}
other = sorted(set(out.get("faults_seen") or []) - ok_faults)
print("result:", out.get("result"), "| stop:", out.get("stop"), "| cleanup_confirmed:", out.get("cleanup_confirmed"),
      "| aborted:", out.get("aborted"))
print("faults other than the bench's usual:", other)
print("last STATE mode:", last.get("mode"), "| duty_e4:", last.get("duty_e4"))
m = out.get("metrics") or {}
print("current peak/mean mA:", m.get("current_peak_ma"), m.get("current_mean_ma"), "| kth:", out.get("kth"))
if out.get("hold"):
    print("hold:", json.dumps(out["hold"]))
good = (out.get("result") == "ok" and out.get("cleanup_confirmed") is True and out.get("aborted") is None
        and not other and last.get("mode") == "DISARMED" and not any(last.get("duty_e4") or []))
print("PASS" if good else "CHECK: not a clean run -- show this to the person")
EOF
}
```

### 6.1 상태 읽기 + 짝 확인 (매번 구동 전)

```bash
D=~/bench_runs/status_$(date +%m%d_%H%M%S); mkdir -p $D
br record --device /dev/forklift-mcu --out $D/run.jsonl --seconds 5 > $D/record.log 2>&1
"$PY" - $D/run.jsonl <<'EOF'
import json, sys
rows = [json.loads(l) for l in open(sys.argv[1])]
prod = [r["product"] for r in rows if r.get("kind") == "connected"]
st = [r for r in rows if r.get("t") == "state"]
a, b = st[0], st[-1]
print("product:", prod[-1] if prod else None)
print("same boot:", len({r["boot_id"] for r in st}) == 1, "| STATEs:", len(st),
      "| MCU s: %.2f" % ((b["mcu_time_us"] - a["mcu_time_us"]) / 1e6))
print("mode:", b["mode"], "| driver_status:", b["driver_status"], "| flags:", b["flags"])
print("faults:", b["fault_flags"])
frames = (b["kth_frames"][0] - a["kth_frames"][0]) % 65536          # 16-bit counters
crc = (b["kth_crc_errors"][0] - a["kth_crc_errors"][0]) % 65536
print("steer KTH frames +%d, CRC errors +%d" % (frames, crc))
EOF
grep -o 'bench[0-9]' "$B/firmware/boards/weact_h743/gen_build_info.cmake" | sort -u
```

판정:
- **짝**: `product` 의 끝 토큰과 마지막 줄의 토큰이 같아야 한다(둘 다 `bench4` 또는 둘 다 `bench3`). 다르면 멈춘다.
- **드라이버 4 개**: `driver_status` 가 `[192, 192, 192, 192]`(0xC0) 이고 `flags` 에 `DRIVERS_READY`, `mode` 가 `DISARMED` 여야 한다. `[192, 0, 0, 0]`·`BOOT` 이면 1 절 접촉 불량 절차를 따른다.
- **KTH CRC**: `same boot: True`, STATE 약 500 개(5 s), 프레임 증가가 MCU 시간 × 1,000 (약 5,000, 11-27 정상 기록 4,990)이고 CRC 오류 증가가 **0** 이어야 한다. 늘면 점퍼를 본다.
- `LIFT_SENSOR_INVALID`(승강 센서 없음), `ESTOP_UNWIRED`, `STEER_OUT_OF_RANGE`(KTH 가 조향이 아니라 M 축에 달림), `BENCH_KTH_LOST`(첫 ARM 전) 는 벤치에서 정상이다.

**자석 확인**(KTH 를 다시 붙였거나 점퍼를 고친 뒤). VM 을 끈 채로 해도 된다(센서는 보드 전원으로 돈다). 15 s 기록 동안 **0–4 s 가만히 → 4–10 s 한 방향(예: 축 끝에서 보아 시계 방향)으로 천천히 약 1 바퀴 → 10–15 s 가만히** 둔다.

```bash
D=~/bench_runs/magnet_$(date +%m%d_%H%M%S); mkdir -p $D
br record --device /dev/forklift-mcu --out $D/run.jsonl --seconds 15 > $D/record.log 2>&1
"$PY" - $D/run.jsonl <<'EOF'
import json, math, sys, statistics as S
st = [json.loads(l) for l in open(sys.argv[1])]
st = [r for r in st if r.get("t") == "state"]
t0 = st[0]["mcu_time_us"]
ts = [(r["mcu_time_us"] - t0) / 1e6 for r in st]
un, prev, off = [], None, 0.0                     # unwrap steering_rad
for r in st:
    q = r["steering_rad"]
    if prev is not None and abs(q - prev) > math.pi:
        off -= math.copysign(2 * math.pi, q - prev)
    un.append(q + off); prev = q
seg = lambda lo, hi: [u for t, u in zip(ts, un) if lo <= t < hi]
a, b = seg(0.5, 3.5), seg(11, 14.5)
print("still spread first/last (rad): %.4f / %.4f" % (max(a) - min(a), max(b) - min(b)))
print("turn (rad, + = sensor's positive): %.2f" % (S.mean(b) - S.mean(a)))
print("CRC errors +%d" % ((st[-1]["kth_crc_errors"][0] - st[0]["kth_crc_errors"][0]) % 65536))
EOF
```

- 판정: 두 정지 구간의 흔들림이 각각 0.01 rad 이하, 돌린 양이 손으로 돌린 만큼(약 ±6.3 rad = 1 바퀴, 부호는 방향을 기록해 둔다), CRC 오류 증가 0. 자석이 없거나 멀면 정지 구간이 흔들리거나 돌린 양이 맞지 않는다.

### 6.2 M 구동 (commission)

M 은 코깅이 강해 0.12 에서는 걸려 멈추고(`kth_no_motion`), 0.20 에서 매끈하게 돈다(약 20 바퀴/s).

```bash
D=~/bench_runs/m_020_$(date +%m%d_%H%M%S); mkdir -p $D
br commission --device /dev/forklift-mcu --out $D/run.jsonl \
  --axis M --duty 0.20 --seconds 20 --stop-kth-turns 3 \
  --bench-bypass steer_range --bench-duty-max 0.25 --bench-time-max 30 \
  --power-switch-in-hand --yes > $D/stdout.json 2> $D/stderr.txt
chk $D
```

- 판정: `chk` 가 `PASS` 이고 `stop` 이 `kth_turns`, `kth` 의 돈 양이 약 +3 바퀴(관성으로 끝 위치는 +3.6 바퀴쯤). 11-27 의 같은 실행은 I_M 평균 약 1.1 A·최고 약 3.6 A(기동)였다. `--observed` 는 주지 않는다(방향은 기록되지 않음, `unknown`).
- 처음이면 `--duty 0.12 --seconds 1` 짧은 펄스부터 해도 된다(코깅에 걸려 `kth_no_motion` 으로 끝날 수 있음 — 그때 `chk` 의 `result` 가 `ok` 가 아니어도 결함이 없으면 정상).

### 6.3 L 구동

L 은 **0.05 부터** 올린다(바퀴를 띄운 상태). 2 s 실행은 도구가 '큰 실행'으로 보고 터미널에서 확인 문구 입력을 요구한다 — 비대화형 셸(Claude 의 명령 실행)에서는 거부되므로, 이 명령은 사람이 전경 터미널에서 출력을 화면에 둔 채 실행하고 도구가 요구하는 문구를 입력한다.

```bash
D=~/bench_runs/l_005_$(date +%m%d_%H%M%S); mkdir -p $D
br commission --device /dev/forklift-mcu --out $D/run.jsonl \
  --axis L --duty 0.05 --seconds 2 --power-switch-in-hand --wheels-lifted --yes
```

- 끝난 뒤 `chk $D` 로 판정한다(`run.jsonl` 의 결과 행을 읽는다). `PASS` 이고 바퀴가 듀티 부호대로 돌았으면(본 방향을 적는다) 0.10 → 0.15 순으로 올린다.

### 6.4 M 위치 유지 (hold)

반 바퀴 계단(`+2048`)으로 목표에 가서 머무는지 본다. 선택 시험으로 `here` 에서 손으로 비틀어 복원을 볼 수 있다(아래).

```bash
D=~/bench_runs/hold_$(date +%m%d_%H%M%S); mkdir -p $D
br commission --device /dev/forklift-mcu --out $D/run.jsonl \
  --axis M --duty 0.15 --seconds 10 --hold-target +2048 \
  --hold-kp 0.5 --hold-kd 0.02 --hold-deadband 12 --hold-sign 1 \
  --bench-bypass steer_range --bench-duty-max 0.25 --bench-time-max 30 \
  --power-switch-in-hand --yes > $D/stdout.json 2> $D/stderr.txt
chk $D
```

- `--hold-target`: `here`(지금 자리), `zero`(가장 가까운 센서 원시 0), 또는 `±카운트`(지금 자리에서 상대, 4096 = 한 바퀴).
- **선택 시험 — 흔들어 복원 보기:** `--hold-target zero --seconds 30` 으로 두고, 실행 중 축을 손으로 **짧게** 흔들었다 바로 놓는다. 흔든 시각을 적어 둔다(로그에는 표시가 없다). **몇 초씩 붙잡고 버티면 드라이버 과열 차단(TSD)으로 `DRIVER_FAULT_M` 이 난다**(아래 11-27 결과).
- **권장 시작값(11-27 bench-2 결과 기준):** 상한 `--duty 0.15`, `--hold-kp 0.5`, `--hold-deadband 12`, **`--hold-min-duty 0.12`**, 목표 `zero`, 30 s.
  - bench-3 이면 감쇠 `--hold-kd 0.02` 부터 더해 본다. **실물 미시험**이다(호스트 시험은 kp 2.0·kd 0.05 에서 넘침을 줄임). 흔들기 뒤 지나침이 남으면 0.05 → 0.1 로 올린다. 축이 떨거나 엉뚱하게 밀면 멈추고(Ctrl-C) kd 를 0 으로 되돌린다.
  - bench-2 이면 `--hold-kd` 를 빼야 한다.
- **11-27 결과(bench-2, 위 권장값, 기록 Jetson `~/arrival_1124/hold30_zero_md012_*`):**
  - **손대지 않은 zero:** 시작 원시각 2,533 → +1,563 카운트 이동. 한 번 146 카운트 지나쳤다가 0.12 s 만에 정착했다. 최종 오차 +8 카운트(약 0.7°, 데드밴드 안)로 30 s 내내 유지, 진동 없음. 정지 유지 전류 25–30 mA, 이동 중 최고 1.95 A.
  - **흔들기(짧게 흔들고 놓기):** 데드밴드 밖 구간 11 개, 최대 오차 ±1,600–2,700 카운트(140–240°). 모두 복귀했고 대부분 0.15–0.7 s 걸렸다. 다만 5 번은 반대쪽으로 1,341–2,281 카운트(120–200°) 크게 지나쳤다가 돌아왔다(복귀 1.2–5.8 s). 결함 없음, 드라이버 0xC0, 최고 3.5 A. **감쇠 항(D, bench-3)이 필요한 근거**다.
  - **붙잡고 버티기:** 사용자가 축을 붙잡고 흔든 실행은 15 s 에 `DRIVER_FAULT_M` 이 났다. FAULT 중 상태 바이트 M = **0xE2(FAULT+TSD)**, 리셋 없음. 11-27 앞선 `DRIVER_FAULT_M`(4.8 A)도 0xE2 였다. 6.5 참고.
- **판정(`chk` 의 `hold` 줄):**
  - `abs(final_error_counts) < 12`(데드밴드, 약 1°)이고 `in_deadband_share_after_settle` 가 0.8 이상이면 잘 머문 것이다. `settled_after_s` 가 `null` 이면 정착하지 못한 것이다.
  - 진동 여부는 `max_abs_error_after_settle_counts` 가 데드밴드보다 훨씬 크면(예: 100 이상) 진동으로 본다. 11-27 kp 0.5·kd 0 은 ±1,000–2,000 카운트를 왕복했다. 부호 전환 횟수 같은 자세한 분석은 기록을 돌려받아 한다(8 절).
  - 결함 판정: `chk` 의 "faults other than the bench's usual" 이 비어 있어야 한다(`DRIVER_FAULT_M` 이 있으면 6.5).
- `zero` 목표는 최소 듀티 없이는 11-27 오전에 목표를 24 카운트(약 2°) **지나서** 멈췄다. 최소 듀티 0.12 로 데드밴드 안(+8)에 섰다(위).

### 6.5 정지·고장 해제

- **멈추기:** 실행 중 `Ctrl-C`(도구가 DISARM 을 보내고 끝냄). 급하면 **차단 스위치**.
- **고장 해제**(`FAULT`·`ESTOP` 일 때만). 펌웨어는 원인이 없어졌는지 스스로 확인하지 않고 드라이버를 다시 초기화한다. 그래서 순서를 지킨다.
  - **원인 비트 읽기:** `DRIVER_FAULT_M` 이면 해제 전에 6.1 상태 읽기(3 s)로 `driver_status` 를 본다. 드라이버 SDO 상태 바이트(데이터시트 SLVSG24C 표 8-21)이며 비트는 0x20 FAULT·0x10 VMOV·0x08 VMUV·0x04 OCP·0x02 TSD·0x01 SPI_ERR(0xC0 이 정상)이다. FAULT 로 바뀐 바로 그 STATE 에는 아직 0xC0 으로 보일 수 있다(20 ms 마다 읽음). FAULT 중 다시 읽으면 보인다.
  - **TSD(과열 차단)는 래치다:** 펌웨어가 CONFIG1 을 쓰지 않아 `TSD_RETRY=0`(데이터시트 8.6.1.7)이고, 저절로 풀리지 않는다. `clear-faults` 로만 풀린다. TTSD 155/170/185 °C, 히스테리시스 30 °C(7.5.10).
  1. **VM 차단.**
  2. 원인 확인과 조치: 축이 걸렸는지(구속), 배선·커넥터·점퍼, 전류가 컸던 이유.
  3. 원인과 조치를 `--cause` 에 적어 해제한다.

```bash
D=~/bench_runs/clear_$(date +%m%d_%H%M%S); mkdir -p $D
br clear-faults --device /dev/forklift-mcu --out $D/run.jsonl --cause "M 구속 전류: 축 걸림 제거, 배선 확인" > $D/stdout.json 2> $D/stderr.txt
cat $D/stdout.json
```

  4. 판정: `"result": "cleared"`. 그 뒤 6.1 상태 읽기로 `DISARMED`·드라이버 0xC0 를 확인하고 나서 VM 을 다시 켠다.

## 7. 지금까지의 결과와 알려진 문제

근거 기록: [11-27 fw-bench](../validation/2026-11-27-bench-fw-bench-m-hold.md), [11-27 KTH LVDS](../validation/2026-11-27-bench-kth-lvds.md), [11-26 L 실모터](../validation/2026-11-26-bench-l-real-motor.md).

- M 3바퀴: 0.20 정상(`kth_turns` 정지), 0.12 코깅에 걸림.
- **hold:** 11-27 오전 P 만(kp 0.5, 최소 듀티 없음, here)은 ±1,000–2,000 카운트를 왕복했다. 오후 최소 듀티 0.12·zero 는 손대지 않으면 +8 카운트에 섰다. 흔들면 매번 돌아오지만 5/11 번 크게 지나쳤다 — 감쇠 항(bench-3) 미시험.
- **붙잡고 버티면 TSD:** 몇 초 붙잡으면 `DRIVER_FAULT_M`, 상태 바이트 0xE2(FAULT+TSD), 래치(`clear-faults`).
- **MISO low 고착 위험:** bench-2/3 은 직전 각이 원시 0 근처(±512)이면 0x0000 프레임도 센다. 이때 MISO 가 low 로 붙으면 카운터가 가짜 이동(예: −256)을 읽고, hold 가 듀티 상한까지 밀 수 있다. 3 ms 손실 정지도 걸리지 않는다.
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
- **끝나면 정상 이미지로 되돌린다**: 5.2 의 `fw-open.bin`(시운전 이미지 — 실행마다 L/R/M/H 중 축을 고름) 또는 `fw-single.bin`(한 번 ARM 하는 동안 고른 축 하나로 고정, 한 채널·KTH 하나 구성용)을 5.3 대로 굽고, 상태 읽기로 제품 문자열이 `lim=lr1`(또는 `lim=lr1 sa1`)으로 끝나는지 확인한다. 정상 이미지의 도구는 main 의 것이다(벤치 폴더 도구를 쓰지 않는다).
