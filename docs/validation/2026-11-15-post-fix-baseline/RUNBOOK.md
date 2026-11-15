표시선 수정 뒤 기준선 — 배치 뒤 집계 순서(결과 보기 전 고정, 계획 `docs/plans/2026-11-15-post-fix-baseline.md` 4 판)

모든 명령은 ws1 비측정 칸에서 스냅숏 `snapshots/pfb_2bea23e5` 를 cwd 로, `PY=/home/projects/forklift/.venv/bin/python`, `T=docs/validation/2026-11-15-post-fix-baseline` 로 둔다. 원자료 경로:
`P=/home/projects/forklift/test_runs`, 복원본 `PRE=$P/tmp_restore/pfb_prefix/test_runs`.

1. 실행 수 확인
   - R: `ls $P/pfb/R_pfb_2bea23e5 | wc -l` = 70, `ls $P/pfb/Rrep_pfb_2bea23e5 | wc -l` = 4, 묶음 요약 `$P/pfb/bundle_R_pfb_2bea23e5/summary.json`(ended·ended_rc_nonzero·infra_failed_attempts·stalled), `sacct -X -j 19264 -o JobID,State,Start,End,Elapsed`.
   - S: `PYTHONPATH=src $PY tools/isaac_bundle.py collect $P/pfb/array_S_pfb_2bea23e5 --expect 40`.
   - L: 묶음 요약 두 개, 세션 32.
2. 입력 대조(계획 5.2): `inputs_pfb_2bea23e5.txt` 의 해시를 같은 명령으로 다시 재서 비교, 각 실행 `result.json` 의 장면 파일·URDF·기준 장면·지도 묶음·ONNX 경로가 목록 값과 같은지.
3. riib 집계(`baseline_summary.py`, 모드 확인 포함):
   - `$PY $T/baseline_summary.py $P/pfb/R_pfb_2bea23e5 --mode overlay --compare $PRE/lcr/cmp_a2b91d2d --compare-mode prefix --out R_vs_lcrB0.json --fields-out R_fields.json.gz`
   - `... --compare $PRE/k1/cmp_c7701296 --compare-mode prefix --match-compare --out R_vs_k1cK1.json`
   - `... --compare $PRE/k1r/k1r_7744f285 --compare-mode overlay-noheader --match-compare --out R_vs_k1rK1.json`
   - `$PY $T/baseline_summary.py $P/pfb/R_pfb_2bea23e5 --mode overlay --compare $P/pfb/S_pfb_2bea23e5 --compare-mode stage --match-compare --out R_vs_S.json`
   - S 의 실행별 값(`--fields-out` 은 기준 쪽만 씀): `$PY $T/baseline_summary.py $P/pfb/S_pfb_2bea23e5 --mode stage --out S_summary.json --fields-out S_fields.json.gz`
   - 거부(refused)가 있으면 실행 이름과 사유를 결과 문서에 적고 집계에서 뺀다.
4. 결정성 반복: 네 짝마다 `$PY tools/deterministic_compare.py $P/pfb/R_pfb_2bea23e5/pfb_R_<scene>_s<n>/run $P/pfb/Rrep_pfb_2bea23e5/pfb_Rrep_<scene>_s<n>/run --json rep_<scene>_s<n>.json` (RGB 는 도구가 뺀다).
5. 라이브: `$PY tools/ct_survey.py $P/pfb/L_pfb_2bea23e5 --json L_survey.json --csv L_missions.csv --runs $P/pfb/lists/RUNS_pfb_L_pfb_2bea23e5.jsonl --near-tally L_near.json`, 그다음 `$PY $T/live_compare.py --new L_missions.csv --prefix docs/validation/2026-11-06-ct-status-survey/missions.csv --arm-map 'LD=D,LO=O6' --out L_vs_cts.json`. 세션별 배속(`pace.jsonl`)·슬롯 변화는 묶음 결과에서 따로 요약.
6. 표: `$PY $T/report_tables.py --riib 'R vs 곡률 B0=R_vs_lcrB0.json' --riib 'R vs K1 비교 K1(40)=R_vs_k1cK1.json' --riib 'R vs K1 재확인 K1(40)=R_vs_k1rK1.json' --riib 'R vs S(40)=R_vs_S.json' --live L_vs_cts.json --determinism rep_*.json --out tables.md`.
7. 축약본: R·S fields gz, L_missions.csv·L_survey.json, 실행 목록 세 개를 `docs/validation/assets/post-fix-baseline/` 로(노트북 사본은 결과 머지 뒤 지움).
8. 결과 문서 `docs/validation/2026-11-15-post-fix-baseline.md` 의 뼈대 자리표에 표를 넣고, 아래를 함께 적는다: vm_watch 공백(17:30:04–17:32:34, 18:14:04–18:16:04, 18:23:04–18:26:34, 18:30:34–18:43:34)과 R 실행 중 VM 켜짐 구간(18:30 이전) — 결정 모드라 결과값과 무관(결정성 반복 4 개로 확인); S 의 PREFLIGHT=off 예외와 DRY_RUN 근거; L 의 런처 기본에 riib 기본 넷이 없는 점(현황 조사와의 차이); 스냅숏 사이 main 머지 수(a2b91d2d→2bea23e5 45, dfa9aaf2→2bea23e5 382, 7744f285→2bea23e5 34).
