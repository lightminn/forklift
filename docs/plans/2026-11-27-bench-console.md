# 벤치 콘솔 — fw-bench 의 main 반영과 콘솔 '벤치' 화면 (2026-11-27)

- 상태: 계획.
- 요청: 사용자 "모터 테스트 GUI 를 콘솔에 통합해서, 안전장치 우회도 가능하게". 총괄 전달 2026-11-27.
- 범위와 기본안 세 가지는 총괄 동의를 받았다(아래 0 절).
- 원칙: 실차 검사는 세션 한정으로 우회할 수 있다. 우회 불가는 멈추는 쪽만이다(메모 real-checks-bypassable).
- 수준: 학교 수준으로 단순하게. 화면 하나에 기존 콘솔 프런트를 재사용하고, Isaac·실차 콘솔 동작은 바꾸지 않는다.

## 0. 정한 것 (총괄 동의 2026-11-27)

1. **백엔드는 별도 도구** `tools/bench_console.py`(Jetson)다. 실차 운영 콘솔(`tools/real_autonomy.py --run --client real`)과 동시에 띄우지 않는다(같은 보드 잠금). 운영 판정·잠금 코드와 섞지 않는다.
2. **'원시 0 근처 MISO low 고착' 위험 수정은 A1 에 포함**한다.
3. **그래프는 canvas 에 직접 그린다**(외부 라이브러리 없음, 기존 `map.js` 방식).

4. **측정 원천을 축별로 고른다**(사용자 추가 요구 2026-11-27): 나중에 MT(MT6701 증분 엔코더, L·R 바퀴 모터축, ABZ 4체배 4,096 카운트/회전)도 쓴다. 그래서 위치·회전 측정을 원천과 무관한 틀로 만든다(아래 0.1).

### 0.1 측정 원천

| 원천 | 축 | 값 | 기준 | 카운트/회전 |
|---|---|---|---|---|
| `kth` | M(조향 KTH), H(승강 KTH) | 절대각 12비트 → 펌웨어 이동량 카운터(이어 붙인 누적) | 센서 원시 0(절대) | 4,096 |
| `mt` | L·R(바퀴 모터축) | 증분 ABZ 카운트(이미 있는 LPTIM 바퀴 카운트) | 실행 시작 위치(상대) | 원천 설정값, 기본 4,096(4체배) |
| `none` | 어느 축이든 | 측정 없음 — 듀티 구동만, 시간으로만 멈춤 | — | — |

- 축의 기본 원천: M→`kth`(조향), H→`kth`(승강), L·R→`mt`. 화면과 도구에서 바꿀 수 있다. 측정이 없는 조합은 hold 와 거리 정지를 고를 수 없다.
- 원천 설정(카운트/회전, 부호)은 `config/real/` 의 한 곳에 둔다(KTH 4,096 고정, MT 는 `vehicle.yaml` 의 바퀴 카운트와 같은 값 — 이중 정의 금지).
- **정지 조건·hold 를 원천 무관하게:** `N 바퀴` / `N 카운트` / 목표 위치.
  - KTH 목표: 영점(원시 0)·절대값·지금·`±N`.
  - MT 목표: 지금 위치·`±N 카운트`(영점 없음).
- **실물 MT 는 미장착**이다. 검증은 `mcu_model` 의 엔코더 모형으로 한다.

순서는 A1 → A2 → B → C 다. 각 PR 은 Codex 수렴까지 돌리고, 최종 트리에서 ws1 전체 pytest 를 한 번 돌린다. 펌웨어 교차 빌드는 노트북에서 한다. 실물 확인은 사용자가 벤치 날에 한다.

## 1. A1 — fw-bench 의 main 반영

진단 브랜치 `diag/fw-bench`(태그 `diag-fw-bench-3`, 14 커밋, 약 2,200 줄)를 main 의 정식 빌드 종류로 옮긴다.

- **벤치 이미지 = 별도 빌드 종류.** 지금처럼 `FW_BUILD=commission FW_MOTOR_OUTPUT=1 FW_BENCH=1` 이고, 제품 문자열 끝은 `bench<세대>` 다.
  - **완료 조건 1 — 운영 빌드에 못 들어감:** `FW_BUILD=normal`(운영)·`hil`·`fw-single` 과 `FW_BENCH=1` 의 조합은 configure 에서 `FATAL_ERROR` 로 멈춘다.
  - 벤치 코드는 `FW_CORE_BENCH` 로만 컴파일되고, 운영 이미지 바이너리에는 벤치 메시지 처리기가 없다. 이를 교차 빌드 시험으로 고정한다: 운영·HIL·fw-single 이미지의 `nm` 에 벤치 심볼 0 개, 벤치 조합 configure 실패.
  - 호스트(`mcu_host`)는 제품 문자열에 `bench` 토큰이 없는 보드에 BENCH_SET·BENCH_HOLD 를 보내지 않는다(지금 진단 도구와 같음).
- **옮기는 것:**
  - 펌웨어: BENCH_SET(0x06, DISARMED 에서만, 우회 비트 `steer_range`·`lift_range`, 축별 듀티·연속 시간 상한), KTH 이동량 카운터(STATE `kth_alt_crc_errors` 칸), BENCH_HOLD(PI + 감쇠 D + 최소 듀티 + 데드밴드).
  - **원천 일반화(0.1):** BENCH_HOLD 와 펌웨어 쪽 정지 조건이 원천을 받는다.
    - `kth` 는 이동량 카운터를 쓴다.
    - `mt` 는 그 축의 LPTIM 바퀴 카운트(이미 STATE `wheel_count_left/right` 로 보냄)를 시작 시점 기준 상대값으로 쓴다.
    - BENCH_HOLD 메시지에 `source` 와 축을 넣는다. 골든 벡터 갱신이 필요하니 메시지 세대를 올린다.
    - 펌웨어는 축과 원천의 짝을 검사한다(M·H↔kth, L·R↔mt). 맞지 않는 짝은 거부한다.
    - 원천 손실은 KTH 는 `BENCH_KTH_LOST`, MT 는 기존 엔코더 결함(`enc_stuck` 등)이 맡는다. 손실이면 그 축을 세운다.
  - 호스트·도구: `mcu_link/messages` 의 메시지·인코딩, `plant/bench_limits`, `mcu_host` 의 큐, `tools/bringup.py commission` 의 `--bench-*`·`--stop-kth-turns`·`--hold-*`, `mcu_model` 의 벤치 동작.
  - 시험: 기존 진단 브랜치 시험(골든 벡터, 펌웨어 호스트 시험, 모형 시험, `test_bringup_bench_tool`).
- **MISO 원시 0 근처 위험 수정**(진단 기록 2026-11-27 '남은 위험'):
  - 지금 규칙: 직전에 센 각이 원시 0 에서 ±512 안이면 0x0000 프레임도 센다. MISO 가 low 로 붙으면 0x0000 이 CRC 를 통과하므로, 가짜 이동(예: −256)을 세고 hold 가 듀티 상한까지 밀 수 있다. 3 ms 손실 정지도 걸리지 않는다.
  - 수정 ① **0x0000 을 세는 범위를 좁힌다:** 직전 각이 원시 0 에서 ±32 카운트(약 3°) 안일 때만 센다. 이 밖에서 시작된 0x0000 연속은 지금처럼 '셀 수 없음'으로 쌓여 `BENCH_KTH_LOST` 로 M 을 세운다. 가짜 이동은 32 카운트 이하로 묶인다.
  - 수정 ② **구동 중 정지 감시(hold):** hold 가 데드밴드 밖에서 최소 듀티 이상을 0.3 s 내는 동안 이동량 카운터가 한 카운트도 바뀌지 않으면 `BENCH_KTH_LOST`(또는 새 비트)로 세운다. 붙은 MISO 는 축이 움직여도 카운터를 고정시키므로 여기에 걸린다. 서 있는 영점 유지(듀티 0)는 걸리지 않는다.
  - 판단 근거: 11-27 hold 기록에서 ±32 안 0x0000 과 정상 정지 영점(+8 카운트) 유지를 펌웨어 호스트 시험으로 재현한다.
  - ±32 와 0.3 s 는 벤치 날 실측 전 잠정값이다.
- **우회할 수 없는 것**(그대로): 과전류, 드라이버 결함(`DRIVER_FAULT_*`, TSD 래치 포함), STOP, DISARM, 명령 만료(링크 끊김), IWDG, 사이클 초과, `BENCH_KTH_LOST`.
- **검증:** 호스트·모형 시험, 노트북 교차 빌드(이미지 크기·제품 문자열), 완료 조건 1 의 빌드 거부 시험, ws1 전체 pytest.

## 2. A2 — 2차 우회 비트

BENCH_SET 의 우회 비트를 늘린다. 틀은 A1 그대로다.

| 비트 | 뜻 | 기본 |
|---|---|---|
| `kth_invalid` | KTH 무효 프레임에도 조향·승강 듀티 구동을 막지 않음(카운터를 쓰는 hold 는 여전히 `BENCH_KTH_LOST` 로 멈춤) | 끔 |
| `enc_stuck` | MT 축(L·R) 바퀴 엔코더 정지 검출로 멈추지 않음 — 켜면 그 축의 `mt` hold·거리 정지는 쓸 수 없음(측정 신뢰 없음) | 끔 |
| `enc_polarity` | MT 축 바퀴 엔코더 극성 검출로 멈추지 않음 — 켜면 `mt` hold 는 거부(부호를 믿을 수 없음) | 끔 |
| `axis_lock` | 한 번 ARM 하는 동안 한 축만 허용하는 잠금을 풂 | 끔 |
| `--bench-absent L,R,M,H,lift` (도구) | 부재 부품의 무효 표식을 판정에서 뺌(기록에 남음) | 없음 |

- 멈추는 쪽은 비트가 없다(1 절).
- 우회가 걸려 있으면 STATE 고장 비트 31 `BENCH_OVERRIDES` 가 선다(지금과 같음).
- **검증:** 비트마다 켬/끔 호스트 시험, 멈추는 쪽은 비트로 꺼지지 않음을 고정, 골든 벡터 갱신.

## 3. B — 벤치 세션 백엔드 `tools/bench_console.py`

- **실행:** Jetson 에서 `python tools/bench_console.py --device /dev/forklift-mcu --host 127.0.0.1 [--console-lan --console-origin-host H]` 로 띄운다.
  - 보드를 실행기 잠금으로 연다. 제품 문자열에 `bench` 토큰이 없으면 시작을 거부한다.
  - 콘솔 서빙은 실차 콘솔과 같은 `LiveServer` 다(Origin 검사·루프백 기본 그대로).
- **명령**(WebSocket 제어 채널, lease 하나):
  - `bench_run {axis, source: kth|mt|none, mode: duty|hold, duty, seconds, stop: {turns?|counts?}, hold: {target: zero|here|abs:N|rel:±N, kp, ki, kd, deadband, min_duty, sign}, bypass: [...], bench_duty_max, bench_time_max, absent: [...]}` → 기존 `bringup.py commission` 의 실행 함수를 같은 프로세스에서 부른다.
    - 원천이 `mt` 면 `zero`·`abs` 를 거부한다.
    - 카운트/회전은 원천 설정에서 읽는다.
    - 도구 인자도 원천 무관하게 맞춘다: `--stop-turns`/`--stop-counts` + `--source`. 기존 `--stop-kth-turns` 는 `--source kth --stop-turns` 의 별칭으로 남긴다.
  - `bench_stop`(즉시 0 + DISARM), `bench_disarm`, `clear_faults {cause}`(FAULT·ESTOP 일 때만, 6.5 절 순서는 화면 안내).
  - 실행 중 다른 `bench_run` 은 거부한다.
- **스트림:** STATE 를 약 20 Hz 로 보낸다. 필드: 모드, 축별 듀티·전류, 드라이버 상태 바이트, 원천별 측정(KTH 이동량 카운터와 각, MT 바퀴 카운트), hold 목표·오차, 고장 비트, 우회 표식. 단위는 원천 설정으로 카운트·바퀴·각을 함께 보낸다.
- **기록:** 실행마다 `~/bench_runs/<모드>_<축>_<시각>/` 에 `run.jsonl`·`result.json` 를 남긴다. 결과 요약은 인수인계 문서 `chk` 와 같은 판정이다: `result ok`·`cleanup_confirmed`·`aborted` 없음·벤치 평상 고장 외 없음·마지막 DISARMED·듀티 0, hold 요약. 우회 목록과 `--cause` 도 기록한다.
- **검증:** pty 위 `mcu_model`(벤치 이미지 모형, MT 는 모형의 엔코더 카운트)로 HIL 시험. KTH·MT 원천 각각 hold·거리 정지를 시험한다. 듀티 구동·hold·STOP·고장 해제·우회 표식을 돌리고, 링크 끊김이면 보드가 명령 만료로 서는지 본다. 4 CPU 로 돌린다.

## 4. C — 콘솔 '벤치' 화면

- `console/web` 에 화면 하나(`data-view-panel="bench"`)를 더한다. 서버 welcome 이 벤치 세션일 때만 왼쪽 메뉴에 '벤치' 가 보인다.
- 구성(화면 하나):
  - 머리줄: 제품 문자열, 드라이버 C0×4, 모드, '우회 중: …' 칩.
  - 입력: 축(L/R/M/H)과 원천(축을 고르면 기본 원천이 따라오고 바꿀 수 있음: kth / mt / 없음), 모드(듀티 구동 / 위치 유지), 듀티 구동 칸(듀티·시간·정지: N 바퀴 / N 카운트), 위치 유지 칸(목표: KTH 는 영점·절대값·지금·±N, MT 는 지금·±N 카운트; 상한·kp·kd·데드밴드·최소 듀티·부호·시간). 원천에 맞지 않는 칸은 비활성.
  - 우회: 체크박스(이 세션만).
  - 버튼: 시작, 큰 STOP, DISARM, 고장 해제(원인 입력 필수).
  - 그래프: 최근 10 s, canvas. 측정값(원천 단위: 카운트·바퀴·각 전환)과 목표, 오차, 듀티, 전류 L R M H.
  - 결과: PASS/CHECK, 정지 사유, 최종 오차(원천 단위), 데드밴드 안 비율, 전류 최고, 드라이버 상태, 기록 폴더.
- 새 문구는 `profile.js` 에 둔다.
- **완료 조건 2 — Isaac·실차 콘솔 무변경을 시험으로 고정:**
  - (a) node 시험: 벤치가 아닌 welcome(Isaac·실차 REAL·HIL)에서는 '벤치' 메뉴와 화면 DOM 이 만들어지지 않고 벤치 메시지를 보내지 않는다.
  - (b) 기존 `console/web/test/*.test.mjs` 와 `tests/unit/console` 전부 통과.
  - (c) 목업 차량으로 기존 화면을 렌더해 PR 전후 스크린샷 비교(지도·오버레이·카메라·측면 패널).
- **검증:** node 시험, B 의 HIL 백엔드와 붙인 렌더 확인(목업).

## 5. 미검증으로 남는 것

- 실물 MT: 미장착이라 모형으로만 검증한다. MT 장착 날 카운트/회전·부호·정지 검출을 실물로 확인한다.
- 실물 동작: 벤치 날에 사용자가 확인한다. A1 의 MISO 수정 값(±32, 0.3 s)과 hold 기본값의 실물 적합성도 그때 본다.
- Jetson 에서의 그래프 스트림 부하(20 Hz). 벤치 날 기록으로 본다.

## 6. 자체 검토

| 검토 | 지적 | 처리 |
|---|---|---|
| (Codex 계획 검토 예정) | | |
