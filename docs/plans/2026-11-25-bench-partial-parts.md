# 부품이 덜 온 벤치 — 모터 1개·KTH 1개로 시운전하기

작성일: 2026-11-25 · 상태: **계획 v4 승인(총괄 2026-11-25), PR-A 구현** · 관련: [벤치 시운전 한계 설계](../design/2026-11-25-bench-commission-limits.md), [도착일 절차서](2026-11-12-arrival-day-procedure.md), `tools/bringup.py`, `tools/h743_first_boot.py`

## 0. 요청과 원칙

사용자 요청(총괄 전달, 2026-11-25): 다음 벤치는 부품이 다 오지 않아 모터 4개를 동시에 못 돌리고 **한 채널에 모터 하나씩** 꽂아 보고, 엔코더도 **KTH7801 하나**로 시험한다. 모터나 엔코더가 인식되지 않아도 그 검사를 끄고 시험할 수 있게 해 둔다.

총괄이 정한 원칙:
- mcu-link 는 바꾸지 않는다. 열린 시운전 빌드와 도구 인자로 처리하고, 인자는 그 실행에만 걸리며 기록·화면에 남는다.
- 우회는 축별·검사별이다. 시험하지 않는 채널은 잠긴 채로 둔다.
- 우회할 수 없는 것: 과전류, STOP/DISARM, 링크 끊김, DRVOFF, 무장 절차.
- 우회한 실행은 `not_for_gate` 다. 링크를 바꿔야 하는 일은 총괄에 먼저 올린다.
- **실제 모터 구동은 매번 직전에 사용자 허락**을 받는다.

## 1. 조사 요약 (main 325c5b08 기준, 전수 표는 부록)

- L·R 바퀴는 지금 이미지·도구로 돌릴 수 있다. 엔코더 정체 정지는 `--bypass enc_stuck` 으로 끄거나 |duty| < 0.1 로 피한다.
- 조향(M)·승강(H)은 그 축의 KTH 가 유효하고 범위 안이어야 한다. 펌웨어(`drive.c` 799-802)와 도구(`--window`) 양쪽에 걸려 있다 → PR-B.
- 래치되는 결함은 **보드 전체 FAULT** 다(드라이버 nFAULT 하나로도). bring-up 도구에 CLEAR_FAULTS 가 없어 MCU 리셋이 필요했다 → PR-A `clear-faults`.
- ARM 은 보드 공통이고, 열린 시운전 이미지의 STOPPING 은 네 채널을 모두 제동한다.

## 2. 나눔

### PR-A — 도구·판정만 (펌웨어·링크 변경 없음)
- `bringup.py record|cpr|step|scenario|commission|clear-faults --absent steer|lift|wheels`(반복 가능): 그 실행 헤더에 `parts_absent` 를 쓴다(없으면 `[]`, 옛 기록과 구별).
- 판정(`plant/judgement.py`): `parts_absent` 를 모든 명령의 실행 표식으로 읽는다. **steer·wheels 가 빠지면 `not_for_gate`**, lift 만 빠진 것은 지금처럼 판정 대상(승강 방식 미정, 계획 3.5 와 `--lift-absent` 선례).
- `h743_first_boot.py`: 헤더의 `parts_absent` 를 읽고, `--lift-absent`·`--steer-absent` 는 헤더와 맞아야 한다(헤더가 부품을 적었는데 플래그가 다른 부품을 말하면 거부). 빠진 축의 센서 비트는 허용, 그 축 kth 항목은 생략.
- `bringup.py kth --axis-absent M|H`: 빠진 축은 `--configured-*` 없이, 판정 `null`. 헤더의 steer = M, lift = H 와 맞아야 한다.
- `bringup.py clear-faults --cause "<원인과 조치>"`: FAULT·ESTOP 일 때만. DISARM 을 먼저 보내 그 seq 가 되돌아온 것을 본 뒤 CLEAR_FAULTS 하나를 보내고, 그 seq 수락과 뒤이은 DISARMED + DRIVERS_READY 를 확인한다. 다시 무장하지 않는다. 결과 `cleared`·`not_accepted`(거절 사유 표시)·`not_ready`·`refused`(`not_latched`).
- 우회·부품 부재 실행의 시운전 결과는 `suggested_duty_sign` 도 비운다(진단값이 설정으로 옮겨지지 않게).

### PR-B — 단일 축 시운전 이미지 `fw-single` (링크 변경 없음)
열린 시운전 빌드 + `drive_config` 옵션(기본 끔), product 끝 표식 `lim=lr1 sa1`. 첫 0 COMMISSION 의 축을 DISARM 까지 고정하고 다른 축은 11 `stopping` 으로 거부, 나머지 세 채널 DRVOFF 는 모든 모드에서 high, M·H 는 KTH 없이(센서 비트는 표시만). 도구는 M·H 에 `--motor-detached` 와 `--window` 또는 `--sensorless` 를 요구하고 둘 다 `not_for_gate`. 상세는 PR-B 에서 이 절을 채운다.

### PR-C — 보류
채널별 nFAULT 분리. DRV8244 리셋값이 EN_OLA=0 이고 초기화는 CONFIG3/4 만 쓰므로 지금 근거가 없다. 벤치에서 결함이 나면 F2(초기화 실패)·F1(nFAULT)·F3(SPI 응답·시간 초과)부터 가른다.

## 3. 벤치 절차 — 1채널·KTH 1개

공통 규칙(멈춤·기록 폴더·장치 경로·`--allow-locked` 금지)은 [도착일 절차서](2026-11-12-arrival-day-procedure.md) 0절을 따른다. 아래 `<부재>` 는 그날 빠진 부품마다 `--absent steer`·`--absent lift`·`--absent wheels` 를 붙인 것이다(예: KTH 를 승강에만 달고 MT6701 이 없으면 `--absent steer --absent wheels`).

**제한(총괄 잠정 승인 2026-11-25): 조향·승강 모터를 기구에 물린 채(결합)로는 KTH 없이 돌리지 않는다.** 센서 없는 M·H 는 모터를 기구에서 떼어 낸 경우에만 한다 — 결합 상태에는 끝단 보호가 없다(KTH 창이 유일한 끝단 정지이고, 센서를 끄면 끝단을 넘어도 멈추지 않는다). 결합된 M·H 는 열린 이미지에서 KTH 와 `--window` 로, 지금처럼 0.2 s 로 한다.

1. **전원 넣기 전**
   - 구운 이미지(product 문자열)와 `bringup.py`·`forklift_core` 커밋을 적는다.
   - 모터 하나·KTH 하나 배선은 전원을 끈 채로. 받침대 고정, 포크는 비우고 받친다. 전원 공급기 전류 제한값을 적는다. 주 전원 스위치를 사람이 손에 든다.
2. **모터 구동 전 확인** — 각각 새 파일에 기록한다.
   ```bash
   python tools/bringup.py record --device /dev/forklift-mcu --out $RUN/first_boot.jsonl --seconds 15 <부재>
   python tools/h743_first_boot.py --record $RUN/first_boot.jsonl --allow-bypassed | tee $RUN/first_boot.json
   python tools/bringup.py record --device /dev/forklift-mcu --out $RUN/kth.jsonl --seconds 30 <부재>   # KTH 를 손으로 돌림
   python tools/bringup.py kth --record $RUN/kth.jsonl --configured-h g704_bytes --image "<product>" --allow-bypassed | tee $RUN/kth.json
   ```
   - `--allow-bypassed` 는 steer·wheels 가 빠졌을 때만 필요하다. 이때 공식 판정은 비고 계산값(`computed_result`·`computed`)만 진단으로 읽는다.
   - 확인할 것: 빠진 축·달린 축, CRC 계수, raw 0 이 보일 값, DRIVERS_READY, 전류 유효, 바퀴 계수 정지.
   - `current`(무부하 50 mA 튐, T12)는 아직 실패한다. 그래도 진행하려면 총괄 결정을 기록한다.
3. **매 실행 직전**
   - 축·방향·duty·시간·꺼진 검사를 보여 주고 **사용자의 명시적 허락**을 받는다.
   - 가장 낮은 duty 와 0.1 s 방향 확인부터. 바퀴는 엔코더가 없으면 |duty| < 0.1 이거나 `--bypass enc_stuck`(그 실행은 `not_for_gate`).
   - 정상 종료·만료 뒤 STATE 가 출력 0 을 보이고 모터가 멈춘 것을 본 다음에만 배선을 바꾼다.
4. **래치가 나면**
   - 원인을 적고 고친 뒤 `python tools/bringup.py clear-faults --device /dev/forklift-mcu --out $RUN/clear.jsonl --cause "<원인과 조치>" <부재>`.
   - `cleared` 가 아니면 멈추고 올린다(`not_accepted` 의 `reject_reason` 이 이유: `SENSOR_INVALID` 는 시운전 빌드의 ADC, `ESTOP_ACTIVE` 등).
   - 다시 무장하기 전에 새로 허락을 받는다.
5. 엔코더·센서 없는 관측은 진단으로만 기록한다(방향·위치 판정 아님).

## 부록 — 부품이 빠진 구성에서 걸리는 검사 전수 조사

**결론:**
- L·R 바퀴는 오늘 이미지와 오늘 도구로 돌릴 수 있습니다. 모터 없는 채널, KTH 하나, MT6701 없음 어느 것도 ARM 을 막지 않습니다. 엔코더 정체 정지는 `--bypass enc_stuck` 으로 끄거나 |듀티| < 0.1 로 피합니다.
- 조향(M)·승강(H)은 그 축의 KTH 가 유효하고 범위 안이어야 합니다. 이 조건이 펌웨어와 도구 양쪽에 걸려 있어 끌 방법이 없습니다.
- 래치되는 결함은 채널별이 아니라 **보드 전체 FAULT** 입니다. bring-up 도구에는 CLEAR_FAULTS 경로가 없어서, 래치되면 MCU 를 리셋해야 합니다.
- 링크 메시지·필드 변경이 필요한 항목은 없습니다(빌드 옵션이나 도구 인자로 처리할 때).

### 1. 모터 없는 채널의 open-load·nFAULT — 채널별이 아니라 보드 전체입니다
- `drive.c:625-635`: `if (in->nfault_low[ch]) bits |= DRIVE_FAULT_DRIVER(ch); … enter_fault(d, bits);`
- `enter_fault`(112-118)는 `d->mode = DRIVE_MODE_FAULT` 로 모드 하나를 바꿉니다. 그래서 어느 채널이든 nFAULT 가 서면 ARM 은 `DRIVE_REJECT_LATCHED`(670-672)로 거부되고, 구동 중이면 고른 축도 멈춥니다.
- 채널별로 따로 갈리는 것은 출력 차단 하나뿐입니다(`channel_safe_drvoff` 930-934, `own_fault`).
- **실제로는 걸리지 않을 것으로 봅니다.**
  - 펌웨어는 FAULT_SUMMARY 상태 바이트를 기록만 하고 판정하지 않습니다(`drive.c:267` `d->driver_status[ch] = valid ? … : 0`). 그래서 open-load 가 상태 레지스터에만 뜨면 영향이 없습니다.
  - 11-24 기록(`docs/validation/2026-11-24-arrival-day.md` 131·211)에서 모터 없는 R·M·H 는 `C0/C0/C0/C0, DRIVERS_READY` 였습니다. L 을 20 s·31 s 구동하는 동안 드라이버 결함 비트도 없었습니다.

### 2. KTH 하나가 없거나 raw 0 일 때
- **보드 규칙**(`board_rules.c:3-9`, `board_config.c:45-62`): `zero_offset_rad = PI_F` 로 두어 0x0000(CRC 를 통과하는 MISO stuck-low 값)이 **valid 이지만 범위 밖**으로 읽히게 합니다. 이 규칙이 깨지면 부팅이 `led_fail_forever(8)` 로 멈춥니다(`main.c:71-73`).
- **CRC 오류 계수에는 펌웨어 문턱이 없습니다.** `abs_axis.c:58-68` 은 세기만 하고, 무효 판정은 연속 실패 `invalid_limit = 3` 개입니다(`abs_axis.c:69-76`). fresh 가 아닌 주기도 실패로 셉니다.
- 11-24 관측은 raw 0 이 아니었습니다. 미연결 KTH 는 프레임이 초당 1000 개 들어오면서 CRC 실패, 즉 NaN·`SENSOR_INVALID` 로 나타났습니다(기록 141).
- **시운전 빌드에서 안 쓰는 축의 KTH 가 없어도 ARM 과 구동을 막지 않습니다.**
  - ARM 과 CLEAR 는 `sensors_valid` 만 봅니다: `if (d->cfg->commission_build) return d->adc_valid;`(333-339, 866).
  - 구동 중 결함 판정도 고른 축만 봅니다(`driving_sensor_bits` 426-439).
  - 다만 `refresh_flags`(542-551)가 그 축의 비트를 STATE 의 `fault_flags` 에 계속 띄웁니다. 그래서 판정 도구가 걸립니다(4절 표 T10).

### 3. M·H 를 KTH 없이(0.15·2 s) 돌릴 수 있나 — 지금은 안 됩니다
위치 루프는 없습니다(시운전은 개루프). 막는 것은 아래 다섯 곳입니다.
- **펌웨어, 명령 수락**(`drive.c:799-802`): `if ((c->axis == DRIVE_CH_M && !axis_ok(&d->steer)) || …) return DRIVE_REJECT_SENSOR_INVALID;`
  - `axis_ok = valid && !out_of_range`(326-329)입니다. KTH 가 달려 있어도 영점이 실측 전 상수 π 라 범위 밖일 가능성이 큽니다. 이 경우에도 거부됩니다.
- **펌웨어, 구동 중**(433-437, 536-537): 고른 축의 INVALID 또는 OUT_OF_RANGE 가 뜨면 보드 전체 FAULT 로 갑니다.
- **도구, 무장 전 창 검사**(`bringup.py:603-607`): `if not (v == v and lo < v < hi): raise Refused`. NaN 이면 거부합니다.
- **도구, 구동 중 창 정지**(697-699): 창을 벗어나면 멈춥니다.
- **도구, 시간 상한**: M·H 는 0.2 s 입니다(`commission.py:19` `AXIS_HOLD_MAX_S`, `bringup.py:1585-1587`). 큰 값 규칙은 바퀴 전용이라(`bench_limits.py:111` `ask.axis < 2`) 펌웨어의 2 s 를 쓸 수 없습니다.

### 4. 걸리는 검사 전체

| # | 위치 (file:line) | 검사 | 이 구성에서의 효과 | 지금 끌 수 있나 | 필요한 변경 | 링크 변경 |
|---|---|---|---|---|---|---|
| F1 | drive.c:625-635, 112-118 | 구동 중 어느 채널이든 nFAULT low → `enter_fault` | 보드 전체 FAULT, ARM 거부(3), 구동 정지, 래치. 11-24 증거로는 무부하 채널에서 발생 안 함 | 아니오 | 펌웨어(열린 시운전 빌드에서 `cm_axis` 가 아닌 채널의 nFAULT 는 그 채널 차단만) | 아니오 |
| F2 | drive.c:189-201; drv8244.c:65-66, 80-88 | 초기화: 네 채널 모두 SPI 응답 유효·CONFIG 되읽기 일치, CLR_FLT 뒤 nFAULT high | 하나라도 실패하면 `drivers_ready` 거짓, BOOT 유지, ARM 거부 5(664-665). 드라이버가 모두 있으면 통과(11-24) | 아니오 | 펌웨어(채널 마스크 빌드 옵션) | 아니오 |
| F3 | drive.c:260-272, 224-237 | 폴링 응답 무효(상위 비트≠11) 또는 SPI 시간 초과 → `DRIVER(ch)` | 보드 전체 FAULT. 상태 바이트 내용(OL 등)은 판정하지 않음(267) | 아니오 | F1 과 같음 | 아니오 |
| F4 | drive.c:519-531; board_config.c:69,77 | 어느 채널이든 5 A·20 ms 과전류 | 보드 전체 FAULT. 무부하 채널 튐 최대 955 mA 라 걸리지 않을 것으로 봄 | 아니오 | 필요 없음 | 아니오 |
| K1 | drive.c:333-339, 866 | ARM·CLEAR_FAULTS 의 센서 조건 | 시운전 빌드는 ADC 만 봄. **KTH 없어도 ARM 가능** | — | — | — |
| K2 | drive.c:799-802 | M·H `COMMISSION` 은 그 축 `axis_ok` 필요 | KTH 없음·범위 밖이면 거부 13. 도구는 `rejected_sensor_invalid` 로 중단(bringup.py:705-706) | 아니오 | 펌웨어(열린 빌드 옵션 "센서 없는 M·H") | 아니오(계약 3.4 문구만) |
| K3 | drive.c:433-437, 536-538; abs_axis.c:69-76 | 고른 M·H 축이 구동 중 무효(연속 3 프레임) 또는 범위 밖 | FAULT 래치, 정지 | 아니오 | K2 와 같은 옵션 | 아니오 |
| K4 | board_rules.c:3-17; board_config.c:48-62 | raw 0 은 범위 밖. 영점 π·범위는 컴파일 상수 | 영점 미보정 KTH 는 대개 범위 밖이라 K2·K3 에 걸림 | 아니오(값은 빌드 상수) | 펌웨어(실측 영점으로 다시 빌드) | 아니오 |
| K5 | abs_axis.c:58-68 | CRC·대체 CRC 오류 계수 | 진단용, 펌웨어 문턱 없음 | — | — | — |
| E1 | drive.c:451-460, 509-515; bringup.py:691-696 | 엔코더 정체: \|u\| ≥ 0.1 이 200 ms 동안 50 카운트 미만 | 시운전 빌드는 래치만 하고 FAULT 로 가지 않음. 도구 early_stop 이 멈춤(11-24 에 0.26 s) | **예**: `--bypass enc_stuck`(bench_limits.py:27-52), 또는 \|duty\| < 0.1 | — | 아니오 |
| E2 | drive.c:465-490 | 엔코더 극성 | `u*w<0` 이 필요한데 엔코더가 없으면 w=0 이라 발생 안 함 | 예(`enc_polarity`) | — | 아니오 |
| E3 | drive.c:355-362, 371-374, 381-383 | ENCODER_GAP(주기 간격 > 20 ms)·JUMP | 모든 모드에서 보드 전체 FAULT. 엔코더 부재와는 무관(11-24: 이동 0, 비트 없음) | 아니오 | 필요 없음(필요하면 펌웨어 빌드 옵션) | 아니오 |
| E4 | (없음) | 바퀴 계수 정체 감시 | 펌웨어에 없음. GAP 은 주기 시간 기준 | — | — | — |
| E5 | commission.py:111-115, 144-145 | S3 바퀴 판정: 연속성·측정 부호 | 엔코더 없으면 `measured_sign none`, 판정은 `inconsistent` 이나 `no_observation` 이라 `duty_sign_ok` 가 null | 아니오 | 판정 표식(무엔코더 실행은 판정 불가로 명시) | 아니오 |
| T1 | bringup.py:603-607, 1588-1590 | M·H 는 `--window` 필수, 값이 NaN 아니고 창 안 | ARM 전 거부 | 아니오 | 도구 인자(무센서 M·H, K2 와 짝) | 아니오 |
| T2 | commission.py:14,19; bringup.py:1583-1587; bench_limits.py:110-112 | M·H 0.15·**0.2 s**, 큰 값은 바퀴만 | 펌웨어의 2 s 를 못 씀 | 아니오 | 도구 인자 | 아니오 |
| T3 | bringup.py:697-699 | M·H 창 이탈·NaN 이면 정지 | 구동 중 정지 | 아니오 | 도구 인자 | 아니오 |
| T4 | bringup.py:1492-1493 | `--bypass` 는 L·R 만 | M·H 에 쓰면 거부 | — | — | — |
| T5 | bringup.py:701-707, 710-722 | 거부 사유가 하나라도 있거나 FAULT·ESTOP 이면 중단 | F·K·E3 이 뜨면 aborted | 아니오 | — | 아니오 |
| T6 | bringup.py 전체; mcu_host.py:558-567 | CLEAR_FAULTS 를 보내는 CLI 없음 | 래치 뒤 다음 ARM 이 `latched` 로 막힘(mcu_host.py:509-510), MCU 리셋 필요 | 아니오(리셋으로만) | 도구 명령 추가(선택) | 아니오 |
| T7 | bringup.py:1596-1615; judgement.py:47-49 | `--bypass` 를 쓴 실행은 `not_for_gate` | S3 부호 판정·`metrics.valid`·외부 pass 가 null. `duty_disarm`(\|duty\| ≥ 0.1)은 엔코더 없으면 공식 합격 불가 | 판정만 `--allow-bypassed`(pass 는 계속 null) | 의도된 동작 | 아니오 |
| T8 | bringup.py:1196-1200, 1222 | `analyze` 가 우회 실행 거부 | `--allow-bypassed` 없으면 refused | 예(판정 아님) | — | 아니오 |
| T9 | bringup.py:1377-1382; judgement.py:55-64; kth_check.py:84-94 | `kth`: 파일의 어느 실행이든 우회면 거부. 축별 판정 | 없는 축은 `undecided`. 범위 밖 축도 `undecided`(87-88). 종료 코드 0. 절차서는 두 축 `configured_ok` 를 요구 | 부분(`--configured-h` 는 값만 주면 됨) | 판정 표식(`--axis-absent`) | 아니오 |
| T10 | first_boot.py:86-92 | `faults`: `--lift-absent` 일 때 승강 비트만 허용 | 조향 KTH 없음 또는 범위 밖이면 fail | 승강만(`--lift-absent`) | 도구 인자(`--steer-absent`) | 아니오 |
| T11 | first_boot.py:94-100 | `kth`: M 은 항상 판정(NaN 0, 프레임 증가) | KTH 를 승강에 달면 fail | 아니오 | 도구 인자(`--steer-absent`) | 아니오 |
| T12 | first_boot.py:81-84 | `current`: 네 채널 \|mA\| ≤ 50 | 11-24 에 튐 때문에 fail. 무부하 채널도 튐 | 아니오 | 판정 기준 변경(중앙값, 별도 PR 로 결정됨) | 아니오 |
| T13 | first_boot.py:75-79 | `drivers`: 상태 0xC0 계열, 결함 6 비트 0 | 무부하 채널도 C0 이라 통과(11-24) | — | — | — |
| T14 | first_boot.py:102-104 | `wheels_still` ≤ 2 카운트 | MT6701 없어도 통과(11-24 이동 0) | — | — | — |
| T15 | h743_first_boot.py:54-61, 74-75 | 우회 기록 거부, 헤더 2 개 이상이면 invalid | 우회 실행과 같은 파일이면 refused | `--allow-bypassed`(결과는 `not_for_gate`) | 새 파일에 기록 | 아니오 |
| T16 | rearm_hold_check.py:131-132, 54 | 출력 잠금 보드만, 축은 L·R 만 | 열린 `lim=lr1` 보드면 refused. 잠긴 fw-commission 은 센서 없이 합격(11-24) | 이미지를 바꿔서만 | 의도된 동작 | 아니오 |
| T17 | plant/bringup.py:76-77 | `cpr`: 돌린 바퀴의 계수 합 ≠ 0 | 엔코더 없으면 `direction_mismatch` | 아니오 | 해당 없음(엔코더가 필요한 측정) | 아니오 |
| N1 | drive.c:338, 514; mcu_host.py:436-439 | 정상·면제 빌드: KTH 두 축 모두 필요, 엔코더 결함은 FAULT, 호스트는 조향 NaN 이면 무장 취소 | 정상 빌드로는 ARM 자체 불가 | 아니오 | 펌웨어 축 끄기(`steer_off`·`lift_off`, 보류된 별도 설계) | 아니오(런타임 선택이면 예) |

### 5. 질문별 짧은 답
1. **보드 전체입니다.** 모터 없는 채널의 nFAULT·SPI 무효·과전류는 다른 채널의 ARM 과 구동도 막습니다(F1–F4). 다만 상태 레지스터의 OL 비트는 판정하지 않고, 11-24 실측에서는 걸리지 않았습니다.
2. **안 쓰는 축의 KTH 부재는 ARM 을 막지 않고 FAULT 도 내지 않습니다**(K1). raw 0 은 valid·범위 밖이고, CRC 오류는 세기만 하며 무효 판정은 연속 3 프레임입니다(K4·K5).
3. **M·H 는 KTH 없이 안 됩니다.** 펌웨어 K2·K3, 도구 T1–T3 가 막습니다. KTH 가 있어도 영점이 상수 π 라 범위 밖이면 같은 결과입니다(K4).
4. **바퀴 개루프는 됩니다.**
   - 정체 정지(E1)만 막고, `--bypass enc_stuck` 으로 끕니다(극성은 원래 발생 안 함). 우회하면 그 실행은 `not_for_gate` 가 됩니다.
   - |duty| < 0.1 이면 정체 판정 자체가 걸리지 않아 우회 없이 공식 판정 대상으로 남습니다.
   - GAP·JUMP 는 끌 수 없지만 엔코더 부재로는 생기지 않습니다. 바퀴 계수 정체 감시는 없습니다.
5. **판정 도구는 표의 T9–T17 에서 걸립니다.** h743_first_boot 에는 `--lift-absent` 만 있고 `--steer-absent` 가 없습니다. `current` 항목은 부품과 무관하게 실패합니다.
6. **링크 메시지·필드 변경이 필요한 항목은 없습니다.** 빌드 옵션(열린 시운전 빌드)과 도구 인자로 처리하는 경우입니다. 런타임에 실행마다 펌웨어 보호를 고르게 하면 ARM·COMMISSION 에 필드가 필요해 링크가 바뀝니다. 이것은 기각된 4판 방식입니다. K2 를 바꾸면 계약 3.4 의 "고른 축 센서 13" 문구만 고치면 되고 version 은 3 그대로입니다.

### 확인 필요
- **DRV8244 open-load**: CONFIG1/2 는 쓰지 않으므로(드라이버 초기화는 CONFIG3 0x40·CONFIG4 0x04 만 씀) 기본값으로 OLA·OLP 가 켜지는지, 켜지면 nFAULT 를 당기는지 데이터시트로 확인하지 않았습니다. 무부하 채널을 직접 구동하는 경우도 시험된 적이 없습니다.
- **빈 KTH 커넥터의 MISO 레벨**: 11-24 는 CRC 실패(NaN)였습니다. 케이블은 꽂혀 있고 센서 전원만 없는 경우 0x0000 이 오면 valid·범위 밖이 됩니다. LVDS 수신기의 failsafe 레벨은 확인하지 않았습니다.
- **조향 범위**: 조향 범위는 차대 모형(`dls08_measured`)에 있고 열어 보지 않았습니다. 영점 π 에서 실제 장착 각이 범위 안에 들 확률은 추정입니다.
- **`tools/h743_check/`**: 이 디렉터리를 찾지 못했습니다. 지금 쓸 수 있는 도구로는 디렉터리 목록을 볼 수 없습니다. 대신 `tools/h743_link_check.py` 를 읽었는데, DISARM 만 보내므로 부품 유무와 무관합니다.
- **엔코더 입력 풀업**: LPTIM 입력의 풀 설정(`gpio_af`)은 읽지 않았습니다. 엔코더 부재 시 계수가 고정된다는 것은 11-24 의 관측 근거뿐입니다.
- **`lim=lr1` 표식**: `gen_build_info.cmake` 가 product 문자열에 이 표식을 실제로 넣는지는 확인하지 않았습니다.
- **CLEAR_FAULTS 경로(T6)**: bring-up CLI 에 CLEAR_FAULTS 경로가 없다는 것은 `bringup.py` 와 `rearm_hold_check.py` 를 읽은 범위의 결론입니다. 다른 도구는 확인하지 않았습니다.

