# 도착일(11-20) 준비 점검표 — 2026-11-16(등록 개정 4 반영)

절차서([도착일 절차](2026-11-12-arrival-day-procedure.md))와 M1–M3 등록([등록](2026-11-15-m1-m3-registration.md))을 기준으로, 항목마다 상태를 적는다. 상태는 **준비됨**(도구·문서·시험이 main 에 있음), **사용자 대기**(사용자가 할 일이 남음), **미착수**(아직 만들지 않음) 셋이다. 리허설 2 차 결과는 [기록](../validation/2026-11-16-arrival-day-rehearsal-2.md).

## 사용자 대기 — 사용자가 할 일

| 항목 | 사용자가 할 일 |
|---|---|
| Jetson 켜기(11-19) | Jetson 을 켜고 zetin Wi-Fi 에 붙인다(1 분). 콘솔 세션이 `tools/hwday/jetson_prep.sh` 로 main 반영·스캔 이미지 적재·부팅 뒤 전원 상태를 기록한다([11-20 순서표](2026-11-20-arrival-day-sequence.md)) |
| A2M12 연결(G4) | A2M12 를 노트북에 연결해 `scan_check.py` 실측을 할 수 있게 한다. 이것이 7b(다리 스캔 원천)와 G6 실차 주행의 선행 조건이다. |
| 현장 준비물 | bring-up 안전 물건(절차서 1 절: 받침대·스트랩, 손에 쥐는 주 전원 차단 스위치, 승강 낙하 방지 블록, 멀티미터, 전류 제한 벤치 전원, 오실로스코프, 외부 리셋 버튼)과 M1–M3 용 **줄자·마스킹 테이프**만 갖춘다(등록 개정 4: 바닥 십자·지시봉·천장 카메라는 필요 없음). |
| A2M12 장착 | A2M12 는 **차체 둘레가 가리지 않는 높이·위치**에 단다(7b: 가림이 차체 둘레에 걸리면 그 방향 허가가 늘 0). 장착 뒤 뒤축 기준 x·y·z·yaw 를 줄자로 재어 절차서 11a 대로 다리에 넣는다. |

## 준비됨

| 항목 | 근거 |
|---|---|
| Jetson(Orin Nano Super, `scienceowl@jetson.local`) | 2026-11-17([hw-day](../validation/2026-11-17-hw-day.md)): 저장소·venv `.[real,console]`, udev, `dialout`·`docker` 그룹, MAXN_SUPER + `jetson_clocks`, 다리 정체 정지·부하 PASS(최대 29.6 ms), H743 HIL 링크 PASS, D435i IMU 400/200 Hz(librealsense 소스 빌드). `tools/hwday` 기본값은 이 장치(#이 PR) |
| Arm GNU Toolchain 13.3.rel1 | 노트북 `~/opt/arm-gnu-toolchain-13.3.rel1-x86_64-arm-none-eabi`(절차서 2 절의 `<TC>`) |
| STM32CubeProgrammer CLI | 노트북에 2.23.0 설치(2026-11-16, `STM32_Programmer_CLI --version`, `-l usb` 동작 — 장치 없음) |
| 모터 출력 보호 확인 기록 | 사용자가 작성·커밋(#552, `docs/validation/2026-11-16-ipropi-protection-record.md`): 보호 회로 없이 출력을 연다, 예비 보드 있음. 게이트 검사 `rec=1f4207f4` 통과(총괄 확인). 절차서 2 절의 open 빌드 명령에 경로를 채움 |
| D435i 실기 확인 | 2026-11-16 노트북 USB 3(5 Gbps), 정지 120 s, `tools/imu_record.py`(main 64e3b6da): 자이로 48,107 표본 399.1 Hz·가속도 24,094 표본 199.9 Hz, dropped 0·write errors 0, 장치 시각 역행 0, 가속도 공백은 시작 직후 24.9 ms 한 번, 자이로 정지 편향 −0.235/−0.209/+0.091 °/s, \|g\| 9.817 m/s². 기록 `data/imu_check/2026-11-16.jsonl`(로컬, 커밋 안 함) |
| udev 규칙(노트북) | 2026-11-16 설치: `/etc/udev/rules.d/99-forklift-mcu.rules`(main `deploy/udev` 와 같음)와 DFU 규칙 `50-stm32-dfu.rules`(0483:df11, 절차서 4 절), reload·trigger 완료. 보드를 꽂은 확인(절차서 4 절의 `ls`·`udevadm` 세 줄)은 아직 |
| 콘솔 단말 | 사용자 결정(2026-11-16): 노트북 Chrome, 앞쪽 탭·화면 켬. 콘솔이 송신 간격(10 s p99·최대)을 보여 주고 0.2 s 를 넘거나 화면이 가려지면 경고한다(#550). 리허설 모형에서 무장·TELEOP 앞쪽 탭 송신 간격 최대 49.9 ms(리허설 2 차 기록 R2-3 후속) |
| bring-up 도구 S2–S5(기록·kth·clock·cpr·재무장 보류·commission·step·시나리오·analyze) | 리허설 1·2 차 모두 통과(모형 MCU) |
| 텔레옵 다리(D5 링크 판정·시계 확인 무장·loop_slow·기록 머리 줄) | #533·#530·#534·#540, 절차서 12.3a |
| 단말 정지·다리 정체 void 의 운용 문구 | 절차서 12.3b(#538) |
| G1·G5·G7(기록·표지·시계 맞춤), G2(IMU 기록), G6(일정표 구동 — 개정 4 는 있는 일정표 가운데 9 개만 씀) | #497·#500·#508, 리허설 2 차. G3 원호는 개정 4 에서 하지 않음 |
| G8 분석 입구 M1–M3 | #532·#537·#544·#545, 개정 4 의 줄자 입력·cm·도 판정 #554: `m_analyze.py --truth <줄자 파일>` 이 시행마다 `rev4` 값과 L/H 띠를 내고, `tools/m_summary.py` 가 하루 표(중앙값·H 넘은 시행·M2 좌우 이득 비)를 만든다. 예산·k 한계·편향 변화율은 판정에서 뺌 |
| 7b 스캔 허가 | #559: 다리 `--scan-socket`·`--lidar-mount`, 기존 장애물 층·`BundlePermit`, 합성 스캔 시험·모형 MCU 통합 시험. A2M12 연결 날엔 `scan_check` 실측 한 번과 절차서 11a 의 장착 확인·정적 확인만 한다 |
| 지연 한계 진단 기록 | #546: 주기마다 `delay_terms`(앵커·오프셋 차·드리프트 몫), 판단은 바꾸지 않음 |
| 리허설 2 차 결함 R2-1·R2-2·R2-3(b) | #548(표지 꺼짐 경고와 기록 머리 줄, `rehearsal_truth/1` 을 heading 참값으로), #550(콘솔 송신 간격 표시·가림 경고, 다리 기록 `teleop_age_ms`·`teleop_client`) |
| M1–M3 등록 | 개정 4(2026-11-16, 단순화): 차 센서 + 줄자, cm·도 판정, 하루 11 시행, 줄자 손작업 약 10–12 분(등록 11 절) |

## 미착수 / 남은 일

| 항목 | 상태 |
|---|---|
| 재부팅 뒤 `jetson_clocks` | 11-19 `jetson_prep.sh` 기록으로 확인. 풀리면 S1 부하 전에 `jetson_power.sh` |
| S1 의 GPU 부하 도구 | 없음. 11-20 S1 은 CPU 모든 코어 부하로 하고 그렇게 기록 |
| Wi-Fi + ssh 터널 텔레옵 송신 간격 | 바닥 주행 전(콘솔 표시로 5 분, 모형 MCU) |
