# 표시선 수정 뒤 Isaac 새 기준선 — 결과 (2026-11-16)

- 계획: `docs/plans/2026-11-15-post-fix-baseline.md`(#524, 4 판 개정 #525). 기술 기록이며 판정·문턱·승격 권고는 없다.
- 집계 순서와 명령: `2026-11-15-post-fix-baseline/RUNBOOK.md`. 도구: `baseline_summary.py`(#528), `live_compare.py`, `report_tables.py`(같은 폴더).
- 스냅숏 `pfb_2bea23e5`(= main 2bea23e5). 실행 목록 `RUNS_pfb_{R,L,S}_pfb_2bea23e5.jsonl`(74/32/40). 적용값 대조표 `applied_table_pfb_2bea23e5.md`, 입력 해시 `inputs_pfb_2bea23e5.txt`.
- **수정 전과의 차이는 "스냅숏 차이(표시선 수정 포함)"다. 표시선 몫으로 돌리지 않는다**(계획 3.2). 지금 코드 위의 누출 몫은 R↔S 짝(3.3)만 말한다.

## 요약

개수만 적는다. 판정은 없다(계획 1 절).

- **R(riib 결정 모드, 70)**: 완주 65/70 이다. 장면별로 random_1 55/60, line_2 5/5, aisle_rack 5/5 다. 원거리 첫 시도 채택은 70/70, 근접 최종 상태는 done 69·pending 1 이다. 운반 추종 실패는 cross_track 1 이고, 완주 실행의 하역 오차 중앙값은 8.6 mm(p90 21.2, 최대 53.6)다. 금지 접촉 0, 결정성 깨짐 실행 0 이다.
- **수정 전 곡률 비교 B0 와(같은 장면·seed 70)**: B0 는 완주 64/70 이다. 결과가 다른 짝은 7 이다. aisle_rack s2·random_1 s0·s23·s40 은 R 만 완주했고, random_1 s11·s24·s49 는 B0 만 완주했다. 이 차이는 스냅숏 차이(표시선 수정과 그 사이 main 머지 45 개)이며 표시선 몫으로 나누지 않는다(계획 3.2).
- **수정 뒤 K1 재확인 K1 팔과(같은 40)**: 결과가 다른 짝 0 이다. 표의 모든 칸이 같다. 그 사이 main 머지는 34 개다.
- **수정 전 K1 비교 K1 팔과(같은 40)**: 결과가 다른 짝 5 다(aisle_rack s2, random_1 s0·s11·s23·s24).
- **S(같은 스냅숏, 표시 prim 이 인식 카메라에 보임, 40)와**: 결과가 다른 짝 5 다. seed 는 수정 전 K1 비교와 같은 다섯이다. S 의 aisle_rack 다섯 실행은 집계 칸이 수정 전 곡률 B0·K1 비교 K1 팔과 값까지 같다(s2 근접 후보 없음 포함). S random_1 s24 는 렌더 동기 실패로 끝나 별도로 표시했다(1 절).
- **결정성 반복 4 짝**: 모두 같다(차이 0, broken 0).
- **라이브 L**: 결정 모드 L-D 16 세션 48 임무, 운영 모드 L-O 16 세션 48 임무가 모두 완주했다. 수정 전 현황 조사 D 도 48/48 이었고, L-D 와 D 의 같은 seed·순번 임무에서 결과가 다른 것은 없다. 수정 전 O6 는 4 세션 가운데 시작 확인 11·완주 10·미제출 1 이었다(그 조사의 학습 근접 캡처 실패 1 건). 하역 오차·임무 시간 분포는 5 절 표에 있다.

## 1. 실행과 유효성

| 묶음 | 잡 | 시작–끝 | 실행 | 끝남 / 인프라 실패 / 미완 / 멈춤 | 비고 |
|---|---|---|---|---|---|
| R(70 + 반복 4) | 19264 | 11-15 16:23:54–19:45:15 | 74 | 69 / 0 / 0 / 0, 비 0 종료 5 는 모두 임무 실패 | GPU0 한 잡(VM 이 GPU1), `--retries 2`. sacct FAILED 는 비 0 종료 실행이 있으면 묶음이 exit 1 을 내기 때문 |
| S(40) | 19268(array) | 11-16 01:53:41–03:32:40 | 40 | 34 / 0 / 0 / 0, 비 0 종료 6(임무 실패 5, random_1 s24 렌더 동기 실패 1) | isaac_array, PREFLIGHT=off 예외(아래). 두 GPU |
| L(32) | 19313·19314 | 11-16 04:09:03–06:25:07 | 32 | 32 / 0 / 0 / 0, 임무 96 제출·96 완료 | 단독 창, 두 잡(3 슬롯씩, claim-dir 공유), 배속 물러남 0 |

- 입력 대조(계획 5.2): R·S 뒤 재측정한 입력 해시가 제출 전(`inputs_pfb_2bea23e5.txt`)과 모두 같다. R·Rrep·S 114 실행의 기록 경로(장면 파일 폴더·URDF·기준 장면·ONNX) 불일치 0(집계 잡 19310). L 32 세션의 에폭 기록 경로(장면 파일·지도 묶음·URDF·기준 장면) 불일치 0(집계 잡 19315).
- 표시 모드 거부: R·S·비교 대상 모두 0(R overlay 두 곳 확인, S stage, 곡률 B0·K1 비교 prefix, K1 재확인 overlay-noheader).
- **S random_1 s24 는 별도 표시한다.** 관측 단계에서 `frame_sync: no frame at 1.016667 s after 20 renders` 로 끝났다. 임무 판단이 아니라 시뮬레이터의 렌더 동기 실패이고, 결정 모드 기록의 `broken` 이 이 실행에서 비어 있지 않다(표의 '결정성 깨짐 실행 1'). 진단 팔이라 다시 내지 않았다(총괄 승인).
- **VM 과 표본 공백.** R 은 ws1 VM 이 GPU1 을 쥔 채 시작했다(사용자가 18:30 무렵 끔). vm_watch 표본(30 s)의 2 분 넘는 공백은 17:30:04–17:32:34, 18:14:04–18:16:04, 18:23:04–18:26:34, 18:30:34–18:43:34 다(zetin 점프 서버 장애, 그 뒤 연결 빈도 문제로 표본기를 멈춤). R·S 는 결정 모드 배치다. 결정성 반복 4 짝의 비교 결과를 4 절에 적고, VM·배속과의 관계는 판정하지 않는다.
- **S 의 예외(총괄 2026-11-15, 이 배치 한정).** 묶음 점검이 진단 변수를 설계상 거부하므로 S 는 `isaac_array.sbatch` 에 `PREFLIGHT=off`(PREFLIGHT_WHY: "path display diagnostic arm S (plan 3.3): check refuses the diagnostic variable by design; DRY_RUN of two S scenes passed")로 냈다. stage 는 런처가 `EXTRA_ARGS` 의 `--path-display stage` 를 막으므로 환경변수 `FORKLIFT_PATH_DISPLAY_DIAGNOSTIC=stage` 만으로 켰다(러너 `path_overlay.effective_mode`). S 두 장면 DRY_RUN 은 통과했다.

## 2. 적용값과 비교 조건

- riib: R 과 곡률 B0·K1 비교 K1 팔·K1 재확인 K1 팔의 기록 항목(K1 0.05,0.05·층 예산·C2 on·복귀 cusp 2·복귀 출발 boxed·exit/low/restart·D7 hold·aisle_rack KEEPOUT 0.15·line_2_m degenerate on·team_spec)이 같다. 비교 대상끼리의 기록 인자 차이는 출력 경로와 `path_display`(None→overlay)뿐이다.
- 라이브(총괄 결정 (a), 2026-11-15): 런처 기본 그대로 돌렸다. 지금 라이브 런처에는 riib 운영 기본 여섯 가운데 넷(exit-prefix-exempt·low-clearance-guard·restart-confirm 3·return-cusp-replans 2)이 없고, 현황 조사 D·O6 에는 켜져 있었다. 반대로 지금은 K1 과 층 예산이 켜져 있고 현황 조사에는 없었다. 넷의 라이브 승격은 별도 과제로 기록만 한다.
- 출발: 라이브 묶음 실행은 알려진 출발(`BUNDLE_TOKEN` 이면 START_RELOC off), 현황 조사와 같다. 운용자 세션의 기본(random_m 시작 재위치)과 다르다.
- 스냅숏 사이 main 머지 수: a2b91d2d→2bea23e5 45, c7701296→2bea23e5 60, 7744f285→2bea23e5 34, dfa9aaf2→2bea23e5 382.

## 3. riib 결정 모드(R)

### 3.1 R 과 곡률 비교 B0(수정 전, 같은 장면·seed 70 실행)

#### riib: R vs 곡률 B0

- a: `/home/projects/forklift/test_runs/pfb/R_pfb_2bea23e5`(mode overlay) · b: `/home/projects/forklift/test_runs/tmp_restore/pfb_prefix/test_runs/lcr/cmp_a2b91d2d`(mode prefix) · 칸은 a / b (a = 이번 기준선)
- 거부 실행: a 0 · b 0

| 장면 | n | 완주 k/n | 원거리 첫 시도 채택 k/n | 근접 최종 상태 | 근접 시도 valid/n | 운반 이탈(추종 실패) | 운반 cusp 재계획 total(실행 수) | 하역 오차 중앙값·p90·최대 mm | 시뮬 시간 중앙값·p90 s | 실패 분류 | 금지 접촉 / 결정성 깨짐 실행 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| random_1 | 60 / 60 | 55/60 / 55/60 | 60/60 / 60/60 | done 59·pending 1 / done 59·pending 1 | 59/59 / 59/59 | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 8(7) / 9(9) | 8.6·20.1·29.2 / 9.5·19.5·29.2 | 205.0·225.8 / 205.0·221.7 | planning 2·tracking 3 / planning 1·tracking 4 | 0 / 0 / 0 / 0 |
| line_2 | 5 / 5 | 5/5 / 5/5 | 5/5 / 5/5 | done 5 / done 5 | 5/5 / 5/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 0(0) / 2(1) | 17.8·22.6·24.9 / 10.0·24.0·26.0 | 171.3·180.0 / 178.0·212.8 | 없음 / 없음 | 0 / 0 / 0 / 0 |
| aisle_rack | 5 / 5 | 5/5 / 4/5 | 5/5 / 5/5 | done 5 / done 4·failed 1 | 5/5 / 4/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 7(4) / 2(2) | 10.3·53.5·53.6 / 21.9·53.4·64.7 | 297.1·307.6 / 276.0·284.2 | 없음 / perception 1 | 0 / 0 / 0 / 0 |
| 합계 | 70 / 70 | 65/70 / 64/70 | 70/70 / 70/70 | done 69·pending 1 / done 68·failed 1·pending 1 | 69/69 / 68/69 | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 15(11) / 13(12) | 8.6·21.2·53.6 / 9.9·20.9·64.7 | 205.0·234.3 / 205.0·228.4 | planning 2·tracking 3 / perception 1·planning 1·tracking 4 | 0 / 0 / 0 / 0 |

#### 짝 차이(완주·끝 단계·실패 분류가 다른 짝)

- 공통 70 · 바뀐 짝 7 · only_a 0 · only_b 0

| 장면 | seed | a 완주 | a 끝 단계 | a 사유(앞 60 자) | b 완주 | b 끝 단계 | b 사유(앞 60 자) |
|---|---|---|---|---|---|---|---|
| aisle_rack | 2 | 완주 | complete | — | 실패 | observe | near_capture_failed:perception_no_pallet:no_expected_candida |
| random_1 | 0 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.9974,yaw=1.0147 |
| random_1 | 11 | 실패 | transport | scene_replan_refused:cusp_pose:blocked:occupied | 완주 | complete | — |
| random_1 | 23 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.2105,yaw=0.6871 |
| random_1 | 24 | 실패 | return_home | Tracking timeout in return_home | 완주 | complete | — |
| random_1 | 40 | 완주 | complete | — | 실패 | transport | scene_replan_refused:cusp_pose:blocked:occupied |
| random_1 | 49 | 실패 | transport | scene_replan_refused:cusp_pose:blocked:occupied | 완주 | complete | — |


### 3.2 R 과 K1 비교·K1 재확인의 K1 팔(같은 장면·seed 40 실행)

#### riib: R vs K1 비교 K1(40)

- a: `/home/projects/forklift/test_runs/pfb/R_pfb_2bea23e5`(mode overlay) · b: `/home/projects/forklift/test_runs/tmp_restore/pfb_prefix/test_runs/k1/cmp_c7701296`(mode prefix) · 칸은 a / b (a = 이번 기준선) · 맞댄 짝 40(두 쪽 모두 거부되지 않은 공통 장면·seed; 공통 이름 40, a 에만 30, b 에만 0)
- 거부 실행: a 0 · b 0

| 장면 | n | 완주 k/n | 원거리 첫 시도 채택 k/n | 근접 최종 상태 | 근접 시도 valid/n | 운반 이탈(추종 실패) | 운반 cusp 재계획 total(실행 수) | 하역 오차 중앙값·p90·최대 mm | 시뮬 시간 중앙값·p90 s | 실패 분류 | 금지 접촉 / 결정성 깨짐 실행 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| random_1 | 30 / 30 | 26/30 / 26/30 | 30/30 / 30/30 | done 29·pending 1 / done 29·pending 1 | 29/29 / 29/29 | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 3(3) / 5(5) | 6.5·18.7·25.3 / 9.9·17.2·19.6 | 198.2·225.8 / 205.7·224.0 | planning 1·tracking 3 / tracking 4 | 0 / 0 / 0 / 0 |
| line_2 | 5 / 5 | 5/5 / 5/5 | 5/5 / 5/5 | done 5 / done 5 | 5/5 / 5/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 0(0) / 2(1) | 17.8·22.6·24.9 / 10.0·24.0·26.0 | 171.3·180.0 / 178.0·212.8 | 없음 / 없음 | 0 / 0 / 0 / 0 |
| aisle_rack | 5 / 5 | 5/5 / 4/5 | 5/5 / 5/5 | done 5 / done 4·failed 1 | 5/5 / 4/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 7(4) / 2(2) | 10.3·53.5·53.6 / 21.9·53.4·64.7 | 297.1·307.6 / 276.0·284.2 | 없음 / perception 1 | 0 / 0 / 0 / 0 |
| 합계 | 40 / 40 | 36/40 / 35/40 | 40/40 / 40/40 | done 39·pending 1 / done 38·failed 1·pending 1 | 39/39 / 38/39 | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 10(7) / 9(8) | 8.2·22.2·53.6 / 11.7·20.4·64.7 | 198.2·267.0 / 206.4·255.2 | planning 1·tracking 3 / perception 1·tracking 4 | 0 / 0 / 0 / 0 |

#### 짝 차이(완주·끝 단계·실패 분류가 다른 짝)

- 공통 40 · 바뀐 짝 5 · only_a 30 · only_b 0

| 장면 | seed | a 완주 | a 끝 단계 | a 사유(앞 60 자) | b 완주 | b 끝 단계 | b 사유(앞 60 자) |
|---|---|---|---|---|---|---|---|
| aisle_rack | 2 | 완주 | complete | — | 실패 | observe | near_capture_failed:perception_no_pallet:no_expected_candida |
| random_1 | 0 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.9974,yaw=1.0147 |
| random_1 | 11 | 실패 | transport | scene_replan_refused:cusp_pose:blocked:occupied | 완주 | complete | — |
| random_1 | 23 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.2105,yaw=0.6871 |
| random_1 | 24 | 실패 | return_home | Tracking timeout in return_home | 완주 | complete | — |

#### riib: R vs K1 재확인 K1(40)

- a: `/home/projects/forklift/test_runs/pfb/R_pfb_2bea23e5`(mode overlay) · b: `/home/projects/forklift/test_runs/tmp_restore/pfb_prefix/test_runs/k1r/k1r_7744f285`(mode overlay-noheader) · 칸은 a / b (a = 이번 기준선) · 맞댄 짝 40(두 쪽 모두 거부되지 않은 공통 장면·seed; 공통 이름 40, a 에만 30, b 에만 0)
- 거부 실행: a 0 · b 0

| 장면 | n | 완주 k/n | 원거리 첫 시도 채택 k/n | 근접 최종 상태 | 근접 시도 valid/n | 운반 이탈(추종 실패) | 운반 cusp 재계획 total(실행 수) | 하역 오차 중앙값·p90·최대 mm | 시뮬 시간 중앙값·p90 s | 실패 분류 | 금지 접촉 / 결정성 깨짐 실행 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| random_1 | 30 / 30 | 26/30 / 26/30 | 30/30 / 30/30 | done 29·pending 1 / done 29·pending 1 | 29/29 / 29/29 | cross_track 1·endpoint_heading 0 / cross_track 1·endpoint_heading 0 | 3(3) / 3(3) | 6.5·18.7·25.3 / 6.5·18.7·25.3 | 198.2·225.8 / 198.2·225.8 | planning 1·tracking 3 / planning 1·tracking 3 | 0 / 0 / 0 / 0 |
| line_2 | 5 / 5 | 5/5 / 5/5 | 5/5 / 5/5 | done 5 / done 5 | 5/5 / 5/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 0(0) / 0(0) | 17.8·22.6·24.9 / 17.8·22.6·24.9 | 171.3·180.0 / 171.3·180.0 | 없음 / 없음 | 0 / 0 / 0 / 0 |
| aisle_rack | 5 / 5 | 5/5 / 5/5 | 5/5 / 5/5 | done 5 / done 5 | 5/5 / 5/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 7(4) / 7(4) | 10.3·53.5·53.6 / 10.3·53.5·53.6 | 297.1·307.6 / 297.1·307.6 | 없음 / 없음 | 0 / 0 / 0 / 0 |
| 합계 | 40 / 40 | 36/40 / 36/40 | 40/40 / 40/40 | done 39·pending 1 / done 39·pending 1 | 39/39 / 39/39 | cross_track 1·endpoint_heading 0 / cross_track 1·endpoint_heading 0 | 10(7) / 10(7) | 8.2·22.2·53.6 / 8.2·22.2·53.6 | 198.2·267.0 / 198.2·267.0 | planning 1·tracking 3 / planning 1·tracking 3 | 0 / 0 / 0 / 0 |

#### 짝 차이(완주·끝 단계·실패 분류가 다른 짝)

- 공통 40 · 바뀐 짝 0 · only_a 30 · only_b 0

(바뀐 짝 없음)


### 3.3 R 과 S(지금 코드 위의 누출 몫, 같은 장면·seed 40 실행)

읽는 법은 계획 3.3: 같은 코드에서 표시 prim 이 인식 카메라에 보일 때와 안 보일 때의 차이이며, #512 전후 차이의 표시선 몫이 아니다.

#### riib: R vs S(40)

- a: `/home/projects/forklift/test_runs/pfb/R_pfb_2bea23e5`(mode overlay) · b: `/home/projects/forklift/test_runs/pfb/S_pfb_2bea23e5`(mode stage) · 칸은 a / b (a = 이번 기준선) · 맞댄 짝 40(두 쪽 모두 거부되지 않은 공통 장면·seed; 공통 이름 40, a 에만 30, b 에만 0)
- 거부 실행: a 0 · b 0

| 장면 | n | 완주 k/n | 원거리 첫 시도 채택 k/n | 근접 최종 상태 | 근접 시도 valid/n | 운반 이탈(추종 실패) | 운반 cusp 재계획 total(실행 수) | 하역 오차 중앙값·p90·최대 mm | 시뮬 시간 중앙값·p90 s | 실패 분류 | 금지 접촉 / 결정성 깨짐 실행 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| random_1 | 30 / 30 | 26/30 / 25/30 | 30/30 / 29/29 | done 29·pending 1 / done 28·pending 1 (결측 near_capture 1) | 29/29 / 28/28 (결측 near_capture 1) | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 3(3) / 5(5) | 6.5·18.7·25.3 / 9.5·17.5·19.6 | 198.2·225.8 / 206.4·224.4 | planning 1·tracking 3 / other 1·tracking 4 | 0 / 0 / 0 / 1 |
| line_2 | 5 / 5 | 5/5 / 5/5 | 5/5 / 5/5 | done 5 / done 5 | 5/5 / 5/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 0(0) / 2(1) | 17.8·22.6·24.9 / 10.0·24.0·26.0 | 171.3·180.0 / 178.0·212.8 | 없음 / 없음 | 0 / 0 / 0 / 0 |
| aisle_rack | 5 / 5 | 5/5 / 4/5 | 5/5 / 5/5 | done 5 / done 4·failed 1 | 5/5 / 4/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 7(4) / 2(2) | 10.3·53.5·53.6 / 21.9·53.4·64.7 | 297.1·307.6 / 276.0·284.2 | 없음 / perception 1 | 0 / 0 / 0 / 0 |
| 합계 | 40 / 40 | 36/40 / 34/40 | 40/40 / 39/39 | done 39·pending 1 / done 37·failed 1·pending 1 (결측 near_capture 1) | 39/39 / 37/38 (결측 near_capture 1) | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 10(7) / 9(8) | 8.2·22.2·53.6 / 11.0·20.5·64.7 | 198.2·267.0 / 206.4·258.4 | planning 1·tracking 3 / other 1·perception 1·tracking 4 | 0 / 0 / 0 / 1 |

#### 짝 차이(완주·끝 단계·실패 분류가 다른 짝)

- 공통 40 · 바뀐 짝 5 · only_a 30 · only_b 0

| 장면 | seed | a 완주 | a 끝 단계 | a 사유(앞 60 자) | b 완주 | b 끝 단계 | b 사유(앞 60 자) |
|---|---|---|---|---|---|---|---|
| aisle_rack | 2 | 완주 | complete | — | 실패 | observe | near_capture_failed:perception_no_pallet:no_expected_candida |
| random_1 | 0 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.9974,yaw=1.0147 |
| random_1 | 11 | 실패 | transport | scene_replan_refused:cusp_pose:blocked:occupied | 완주 | complete | — |
| random_1 | 23 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.2105,yaw=0.6871 |
| random_1 | 24 | 실패 | return_home | Tracking timeout in return_home | 실패 | observe | frame_sync: no frame at 1.016667 s after 20 renders |

## 4. 결정성 반복(4 짝)

비교 범위는 `deterministic_compare.py` 가 모으는 실행 폴더 파일이고 RGB 배열은 뺀다. 반복은 R 과 같은 스냅숏·같은 묶음 잡에서 돌았다.


| 짝 | 결과 | 첫 차이(또는 기록 불완전 사유) | 차이 수 | broken 수 | 기록 불완전 수 |
|---|---|---|---|---|---|
| rep_random_1_s0 | 같음 | — | 0 | 0 | 0 |
| rep_random_1_s1 | 같음 | — | 0 | 0 | 0 |
| rep_line_2_s0 | 같음 | — | 0 | 0 | 0 |
| rep_aisle_rack_s0 | 같음 | — | 0 | 0 | 0 |

## 5. 라이브 연속 이송(L)

- 집계: `ct_survey.py`(스냅숏 판과 같은 코드) → `live_compare.py` → `report_tables.py`(집계 잡 19315). 수정 전 비교 대상은 현황 조사의 `missions.csv` 열만 쓴다.
- L-O 는 seed 마다 갈라지므로 짝 비교하지 않는다(계획 1 절). 짝 표는 L-D 만이고, 결과·사유·단계까지 본다. 시간·하역 오차는 분포로만 본다.
- 런처 기본 차이(2 절): 지금은 K1·층 예산이 켜지고 riib 기본 넷(exit-prefix-exempt·low-clearance-guard·restart-confirm 3·return-cusp-replans 2)이 꺼졌다. 현황 조사는 반대였다. 수정 전 O6 는 seed 0·1 의 4 세션뿐이다.
- 배속: 세션마다 묶음이 기록한 최저 배속(`pace_min`)은 L-D 0.353–0.374(중앙값 0.359), L-O 0.353–0.375(중앙값 0.357)로, 32 세션 모두 물러남 기준 0.335 위였고 물러남은 없었다(`agg/L_paths_pace.txt`). L-O 의 비율·분포는 이 배속 조건의 표본이다.


- 새: `L_missions.csv` · 수정 전: `docs/validation/2026-11-06-ct-status-survey/missions.csv` · 칸은 새 / 수정 전 · 세션당 예정 임무 3

| 팔 | 장면 | 세션 | 세 임무 완주 세션 | 예정·시작 확인·done·실패·미제출 | 그 밖의 시작 상태 | 실패 class | 하역 오차 중앙값·p90·최대 mm | 임무 시간 중앙값·p90 s |
|---|---|---|---|---|---|---|---|---|
| LD(수정 전 D) | line_m | 8 / 8 | 8/8 / 8/8 | 24·24·24·0·0 / 24·24·24·0·0 | 없음 / 없음 | 없음 / 없음 | 14.3·28.7·34.3 / 13.2·26.0·40.4 | 154.7·161.6 / 155.2·164.5 |
| LD(수정 전 D) | random_m | 8 / 8 | 8/8 / 8/8 | 24·24·24·0·0 / 24·24·24·0·0 | 없음 / 없음 | 없음 / 없음 | 13.9·43.2·48.1 / 15.7·32.3·42.7 | 158.3·167.6 / 157.1·166.5 |
| LO(수정 전 O6) | line_m | 8 / 2 | 8/8 / 2/2 | 24·24·24·0·0 / 6·6·6·0·0 | 없음 / 없음 | 없음 / 없음 | 10.5·31.9·38.5 / 11.0·26.2·28.0 | 156.5·164.2 / 154.9·160.4 |
| LO(수정 전 O6) | random_m | 8 / 2 | 8/8 / 1/2 | 24·24·24·0·0 / 6·5·4·1·1 | 없음 / 없음 | 없음 / perception 1 | 15.2·37.2·45.4 / 14.0·26.7·30.6 | 158.3·169.9 / 156.8·166.2 |

#### seed·order 짝에서 outcome 이 다른 임무(LD)

- LD line_m: 공통 seed 8 · 새에만 [] · 수정 전에만 [] · outcome 다른 임무 0
- LD random_m: 공통 seed 8 · 새에만 [] · 수정 전에만 [] · outcome 다른 임무 0

(outcome 다른 임무 없음)

## 6. 원자료와 재현

- 축약본 `assets/post-fix-baseline/`:
  - riib: `R_fields.json.gz`·`S_fields.json.gz`(실행별 필드), `R_vs_lcrB0.json`·`R_vs_k1cK1.json`·`R_vs_k1rK1.json`·`R_vs_S.json`·`S_summary.json`(집계).
  - 라이브: `L_missions.csv`, `L_survey.json.gz`, `L_near.json.gz`, `L_vs_cts.json`.
- 집계 출력과 잡 스크립트: `2026-11-15-post-fix-baseline/agg/`(표·결정성 비교·입력 재측정·경로 대조·배속), `agg.sbatch`(잡 19310), `agg_L.sbatch`(잡 19315). 두 잡은 이 브랜치 코드 사본(ws1 `test_runs/tmp_restore/pfb_code_cd57c65a`)으로 돌았다.
- 표 재현: `RUNBOOK.md` 3–6 단계. 비교 대상 집계(`R_vs_*.json` 의 compare)는 ws1 복원본(`test_runs/tmp_restore/pfb_prefix`, hold)에서 낸 것이다.
- 원본: ws1 `test_runs/pfb/{R,Rrep,S,L}_pfb_2bea23e5`, 묶음·array 기록 `test_runs/pfb/{bundle_R,array_S,bundle_L1,bundle_L2}_pfb_2bea23e5`, 실행 목록 `test_runs/pfb/lists/`, 스냅숏 `snapshots/pfb_2bea23e5`. 이 문서가 머지되면 디스크 세션 정리 목록으로 넘긴다(원본과 스냅숏은 짝으로 보관). 이 기준선은 다른 스냅숏의 B0 로 그대로 쓰지 않는다(계획 1 절).

## 7. 검토
(결과 뒤)
