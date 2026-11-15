# 표시선 수정 뒤 Isaac 새 기준선 — 결과 (뼈대, 결과 전 고정)

- 계획: `docs/plans/2026-11-15-post-fix-baseline.md`(#524, 4 판 개정 #525). 기술 기록이며 판정·문턱·승격 권고는 없다.
- 집계 순서와 명령: `2026-11-15-post-fix-baseline/RUNBOOK.md`. 도구: `baseline_summary.py`(#528), `live_compare.py`, `report_tables.py`(같은 폴더).
- 스냅숏 `pfb_2bea23e5`(= main 2bea23e5). 실행 목록 `RUNS_pfb_{R,L,S}_pfb_2bea23e5.jsonl`(74/32/40). 적용값 대조표 `applied_table_pfb_2bea23e5.md`, 입력 해시 `inputs_pfb_2bea23e5.txt`.
- **수정 전과의 차이는 "스냅숏 차이(표시선 수정 포함)"다. 표시선 몫으로 돌리지 않는다**(계획 3.2). 지금 코드 위의 누출 몫은 R↔S 짝(3.3)만 말한다.

## 요약

(결과 뒤 작성. 장면별 수치, 바뀐 짝 수, R↔S 짝 차이 수, 라이브 두 팔 임무 수를 개수로.)

## 1. 실행과 유효성

| 묶음 | 잡 | 시작–끝 | 실행 | 끝남 / 인프라 실패 / 미완 / 멈춤 | 비고 |
|---|---|---|---|---|---|
| R(70 + 반복 4) | 19264 | (sacct) | 74 | | GPU0 한 잡(VM 이 GPU1), `--retries 2` |
| S(40) | (array) | | 40 | | isaac_array, PREFLIGHT=off 예외(아래) |
| L(32) | | | 32 | | 단독 창, 두 잡 |

- 입력 대조(계획 5.2): (결과 뒤)
- 표시 모드 거부: (baseline_summary 의 refused 목록)
- **VM 과 표본 공백.** R 은 ws1 VM 이 GPU1 을 쥔 채 시작했다(사용자가 18:30 무렵 끔). vm_watch 표본(30 s)의 2 분 넘는 공백은 17:30:04–17:32:34, 18:14:04–18:16:04, 18:23:04–18:26:34, 18:30:34–18:43:34 다(zetin 점프 서버 장애, 그 뒤 연결 빈도 문제로 표본기를 멈춤). R·S 는 결정 모드라 결과값이 배속·VM 과 무관하다는 것을 결정성 반복 4 짝으로 확인한다(4 절).
- **S 의 예외(총괄 2026-11-15, 이 배치 한정).** 묶음 점검이 진단 변수를 설계상 거부하므로 S 는 `isaac_array.sbatch` 에 `PREFLIGHT=off`(PREFLIGHT_WHY: "path display diagnostic arm S (plan 3.3): check refuses the diagnostic variable by design; DRY_RUN of two S scenes passed")로 냈다. stage 는 런처가 `EXTRA_ARGS` 의 `--path-display stage` 를 막으므로 환경변수 `FORKLIFT_PATH_DISPLAY_DIAGNOSTIC=stage` 만으로 켰다(러너 `path_overlay.effective_mode`). S 두 장면 DRY_RUN 은 통과했다.

## 2. 적용값과 비교 조건

- riib: R 과 곡률 B0·K1 비교 K1 팔·K1 재확인 K1 팔의 기록 항목(K1 0.05,0.05·층 예산·C2 on·복귀 cusp 2·복귀 출발 boxed·exit/low/restart·D7 hold·aisle_rack KEEPOUT 0.15·line_2_m degenerate on·team_spec)이 같다. 비교 대상끼리의 기록 인자 차이는 출력 경로와 `path_display`(None→overlay)뿐이다.
- 라이브(총괄 결정 (a), 2026-11-15): 런처 기본 그대로 돌렸다. 지금 라이브 런처에는 riib 운영 기본 여섯 가운데 넷(exit-prefix-exempt·low-clearance-guard·restart-confirm 3·return-cusp-replans 2)이 없고, 현황 조사 D·O6 에는 켜져 있었다. 반대로 지금은 K1 과 층 예산이 켜져 있고 현황 조사에는 없었다. 넷의 라이브 승격은 별도 과제로 기록만 한다.
- 출발: 라이브 묶음 실행은 알려진 출발(`BUNDLE_TOKEN` 이면 START_RELOC off), 현황 조사와 같다. 운용자 세션의 기본(random_m 시작 재위치)과 다르다.
- 스냅숏 사이 main 머지 수: a2b91d2d→2bea23e5 45, c7701296→2bea23e5 60, 7744f285→2bea23e5 34, dfa9aaf2→2bea23e5 382.

## 3. riib 결정 모드(R)

### 3.1 R 과 곡률 비교 B0(수정 전, 70 짝)
(report_tables (a)·(b))

### 3.2 R 과 K1 비교·K1 재확인의 K1 팔(40 짝)
(report_tables (a)·(b), matched)

### 3.3 R 과 S(지금 코드 위의 누출 몫, 40 짝)
(report_tables (a)·(b), matched. 읽는 법은 계획 3.3: 같은 코드에서 표시 prim 이 인식 카메라에 보일 때와 안 보일 때의 차이이며 #512 전후 차이의 표시선 몫이 아니다)

## 4. 결정성 반복(4 짝)
(report_tables (d). 비교 범위는 `deterministic_compare.py` 가 모으는 파일이고 RGB 는 뺀다)

## 5. 라이브 연속 이송(L)
(report_tables (c). 세션별 배속·슬롯 변화 요약, L-O 는 짝 비교하지 않음. L-D 짝 비교는 결과·사유·단계까지)

## 6. 원자료와 재현
- 축약본: `assets/post-fix-baseline/`(결과 뒤).
- 원본: ws1 `test_runs/pfb/{R,Rrep,S,L}_pfb_2bea23e5`, 스냅숏 `snapshots/pfb_2bea23e5`, 수정 전 복원본 `test_runs/tmp_restore/pfb_prefix`(hold). 결과 머지 뒤 디스크 세션 정리 목록으로(원본과 스냅숏은 짝으로 보관).

## 7. 검토
(결과 뒤)
