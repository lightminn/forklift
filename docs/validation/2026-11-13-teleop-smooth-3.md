# 매끄러운 텔레옵 확인 실행 3 — K09 재실행 결과 (2026-11-13)

등록: [`docs/plans/2026-11-13-teleop-smooth-3.md`](../plans/2026-11-13-teleop-smooth-3.md)(결과 전 고정, Codex 4라운드 수렴).

판정 원자료(모두 `2026-11-13-teleop-smooth-3/`):
- 최종 판정: `verdict_k09.json`.
  - `tools/teleop_realtime_report.py --verdict --factor 0.9 --budget-on` 의 출력 그대로다(스냅숏 `ts3_3f2c2a65` 의 도구).
  - 출력의 `verdict.judged` 가 세 자리를 모두 `_re` 로 바꿔 읽었다(`u01_k09_r1 → u01_k09_r1_re` 등). `verdict.left_out` 은 원래 세 세션이다.
  - 명령: 1회차·재실행 여섯 세션을 넣고 `--factor 0.9 --verdict --slots u01_k09_r1,u02_k09_r2,u03_k09_r3 --budget-on --overlapped u01_k09_r1,u02_k09_r2,u03_k09_r3`.
  - `sessions[]` 에는 판정에서 뺀 원래 세 세션도 들어 있다. `summary` 는 `_re` 셋만 모았다(`summary.kept`). 원래 세션의 수치는 판정과 이 문서의 서술에 쓰지 않았다(이 초안을 쓴 API 크레딧 실행이 파일 구조를 읽다 일부를 보았지만 쓰지 않았다).
- 1회차 판정: `verdict_k09_round1.json`. 세 자리가 모두 겹침으로 빠져 `verdict.unevaluated` 가 "every run of the slot over the overlap rule" 셋과 "no slot judged" 다. `failures` 와 `judged` 는 비어 있다.
- sacct 원출력: `sacct_14-25_14-55.txt`(1회차), `sacct_14-40_15-05.txt`(재실행).
- VM 표본기 원출력: `vm_watch_round1.jsonl`(1회차), `vm_watch_re.jsonl`(재실행). 등록 2 절은 한 파일(`vm_watch_k09.jsonl`)로 적었지만, 재실행이 생겨 회차별 두 파일로 나눠 보관했다(내용은 표본기 출력 그대로).
- 스냅숏 ws1 `snapshots/ts3_3f2c2a65`(브랜치 커밋 3f2c2a65, main 1f60d4e2 병합 뒤), 출력 `test_runs/ts3/ts3_3f2c2a65/`(결과 문서 머지까지 보존).

## 실행

- **스냅숏.** 라이브 경로의 원본 파일은 `ts2_65a862dd` 와 md5 가 같다. 차이는 둘뿐이다.
  - 재생 전용 SLAM 도구(등록 1 절의 여섯 파일).
  - 옛 스냅숏의 numba·pyc 캐시.
- **조건.** 2차와 같다.
  - `r1c_session.sh` line_m far confirm, seed 0, INSTR off, G3, 지연 주입 없음.
  - 층 지연 예산 on: `LAYER_BUDGET` 미설정 → 런처 기본 `config/layer_latency_budget.yaml`.
  - 정확 키 캐시 포함.
  - 팔은 K09 하나다(`ARM=K`·catch-up 6, `FACTOR=0.9`).
- **1회차.**
  - 잡: 18798 `u01_k09_r1` 14:33:37–14:37:48, 18799 `u02_k09_r2` –14:41:48, 18800 `u03_k09_r3` –14:45:49. 셋 모두 COMPLETED.
  - 같은 계정 잡 18801 `lbg_an2`(CPU 2)가 14:34:52–14:47:29 에 돌아 세 창과 모두 겹쳤다. 종료 시각은 `sacct_14-40_15-05.txt` 에서 읽었다. `sacct_14-25_14-55.txt` 에서는 아직 RUNNING 이었다.
  - 등록 2 절(같은 계정 겹침 규칙)대로 세 자리를 모두 판정에서 빼고, 자리마다 `_re` 를 한 번씩 돌렸다. 겹친 세션의 수치는 판정에 쓰지 않았다.
  - 18797 `lbg_r` 는 14:32:25 에 끝나 1회차 창 앞에 있다.
- **1회차 VM 표본**(판정에는 쓰지 않음, 기록).
  - `u01`·`u02` 는 clear 였다. `u03` 는 미평가였다.
  - `u03` 미평가의 원인: 14:43:21·14:43:51(`t_local` 기준)의 두 표본이 Wi-Fi DNS 실패로 끊겼다(`ssh_exit` 255, "Could not resolve hostname zetin.uos.ac.kr"). 앞뒤 완전 표본 사이 간격이 90 s 다(`t_ws1` 14:42:54 → 14:44:24).
  - 14:37:51(`t_local`)에도 같은 실패가 한 번 있었다(간격 60 s, `u01`·`u02` 경계). `judge` 의 허용 간격은 연속 완전 표본 사이 2 주기(60 s) 이하라 이 범위는 clear 다(등록 2 절).
  - 이 뒤 표본기를 보강했다(커밋 4b0d8447): 실패한 표본을 주기 안에서 5 s 뒤 최대 두 번 다시 시도한다. 주기(30 s)와 판정 규칙은 그대로다.
- **재실행.** 스냅숏·OUT 은 1회차와 같다.
  - 잡: 18802 `u01_k09_r1_re` 14:48:36–14:52:37, 18803 `u02_k09_r2_re` –14:56:37, 18804 `u03_k09_r3_re` –15:00:38. 셋 모두 COMPLETED(241·240·241 s).
  - 제출 직전 상태: squeue 0, 두 GPU persistence Enabled(2/2), qemu 0, `isaac_bundle check` 3/3, 표본기 켬(`systemd-inhibit` 아래).
- **재실행의 겹침.**
  - 같은 계정: 겹친 잡은 0 이다. 18806 `pytest` 는 `afterany:18804` 로 15:00:39 에 시작해 뒤에 있다.
  - 다른 사용자 CPU: 세 세션 모두 0 이다(판정 출력 `sessions[].overlap` = 0.0).
  - sacct 원출력은 `sacct -a`(모든 사용자)로 받았다.
- **재실행 VM 판정**(`vm_watch.py judge`, 등록 2 절의 VM 규칙): 세 세션 모두 clear 다.
  - 표본 28 개, `ssh_exit` 0 이 28 개(실패 0), `attempts` 1 이 28 개(재시도 0).
  - 28 개 모두 `qemu` `[]` 이고, GPU 2 개가 모두 `Enabled` 였다.
  - 표본 범위는 `t_ws1` 14:48:22–15:01:52 KST 다.
  - clear 는 "등록한 표본 조건에서 VM 을 보지 못함"이다. VM 이 없었다는 증명은 아니다.

## 결론

| 자리 | 판정 | TELEOP 배율 | TELEOP ok | 창 최솟값(문턱 0.81) / 문턱 아래 | pacing 실패 | 실패 hold |
|---|---|---|---|---|---|---|
| `u01_k09_r1_re` | 실패 | 0.862 | 0.961 | 0.8095 / 1 | 1 | 왼쪽: 출력 0.610, not-ok 출발 875 ms |
| `u02_k09_r2_re` | 실패 없음 | 0.866 | 1.000 | 0.8113 / 0 | 0 | — |
| `u03_k09_r3_re` | 실패 | 0.865 | 0.960 | 0.8098 / 1 | 1 | 후진: 출력 0.782, 끊김 722 ms. 왼쪽: 출력 0.870, not-ok 출발 289 ms |

- **K09(f 0.9·catch-up 6)는 불통과다** (`verdict.pass` false, 3 자리 가운데 2 자리 실패, 미평가 0).
- **K 의 등록 후보 가운데 통과한 가장 높은 목표는 f 0.8(K08)로 남는다**(등록 3 절).
  - 실제 한계를 찾은 것은 아니다.
  - 단조성과 신뢰 수준도 주장하지 않는다.
- **TELEOP 배율이 목표에 못 미쳤다.** 배율은 0.862–0.866 으로 목표 0.9 보다 낮았다. 창 최솟값은 문턱 0.81 에 붙어 있었고, 두 자리에서 한 스텝씩 문턱 아래로 내려갔다.
- **AUTO 는 배율 0.890, ok 0.952–0.954 였다.**
- **실패 규칙.**
  - `u01_re`: `pacing_drops` 1, 왼쪽 hold 의 `hold_output`·`pacing_gap`.
    - hold 별 `pacing_drops` 는 모두 0 이고 왼쪽 hold 가 not-ok 로 시작했다(`ok_at_start` false). 그래서 drop 은 hold 사이(후진 hold 가 끝난 뒤 왼쪽 hold 전)에서 났다고 읽는다. 시점을 따로 재지는 않았다.
    - 그 앞 후진 hold 는 ok 1.0 이었다.
  - `u03_re`: `pacing_drops` 1(후진 hold), 후진 hold 의 `hold_output`·`output_stutter`·`pacing_gap`, 이어진 왼쪽 hold 의 `hold_output`·`pacing_gap`.
- **승강 hold** 는 세 자리 모두 ok 1.0 이었다. 변화는 ±23–25 mm 다.
- **영상 hold**(판정 밖): `video_forward` 출력 0.019–0.103, `video_reverse` 0.

### 1차 K09 와의 관측 비교 (보고만)

| | 1차 K09(예산 off·캐시 없음, 2026-11-12) | 이번 K09(예산 on·캐시, 재실행) |
|---|---|---|
| 판정 | 불통과(3/3 자리 실패) | 불통과(2/3 자리 실패, u02 실패 없음) |
| TELEOP 배율 | 0.854–0.863 | 0.862–0.866 |
| TELEOP ok | 0.916–0.960 | 0.960·1.000·0.961 |
| 창 최솟값 / 문턱 아래 스텝 | 0.808–0.810 / 3·1·1 | 0.8095·0.8113·0.8098 / 1·0·1 |
| pacing 실패 | 2·1·1(4 번: 후진 hold 셋, ts1 승강 하나) | 1·0·1(u03 은 후진 hold, u01 은 hold 별 값 0) |
| 실패 hold | 후진, 후진 뒤 not-ok 를 이어받은 왼쪽(ts4·ts5), ts1 lift_up | u01 왼쪽(not-ok 출발), u03 후진 + 이어진 왼쪽 |
| AUTO 배율 / ok | 0.890–0.891 / 0.951–0.956 | 0.890 / 0.952–0.954 |

- 두 실행 사이에는 예산 on·캐시·날짜가 함께 바뀌었다. 그래서 차이의 몫을 가르지 않는다(등록 3 절).
- 이번에는 승강 hold 실패가 없었다. 후진에서 무너지고 이어진 왼쪽 hold 가 not-ok 로 시작하는 모양은 1차와 같다.
- 원인은 진단하지 않았다.

## 안전

세 `_re` 세션 모두 같았다(`sessions[].safety`).
- `result` true.
- `scene_damaged` 없음.
- 포켓 금지 접촉 0, AUTO 의 허가 없는 진입 0, TELEOP 참값 hull 겹침 0.
- 안전 `unevaluated` `[]`.
- 층 예산: `budget_s` 0.1(출처 `config/layer_latency_budget.yaml`), 불변조건 위반 0, 감시 사건 0, 지연 기록 있음.
- TELEOP 최소 여유는 약 7.0 m 다. 넓은 공간에서 한 hold 라 장애물 가까이의 안전을 시험한 것은 아니다(1차와 같음).

## 결정 (총괄, 2026-11-13, K09 결과 뒤)

- **런처 기본은 f 0.4·catch-up 0(L04)으로 유지한다.**
- **K08(f 0.8·catch-up 6)은 '더 빠른 텔레옵 선택지'로 문서화한다.**
  - 켜는 방법: 콘솔 런처 환경에 `FACTOR=0.8` 과 `FORKLIFT_LIVE_CATCHUP_STEPS=6` 을 준다.
  - 적용 범위를 함께 적는다: line_m 한 장면, seed 0, 짧은 hold 열, 층 지연 예산 on, 정확 키 캐시 포함.
  - 문서화: `console/README.md` 의 알려진 운용 한계 아래에 한 줄로 넣었다(이 PR).
- **콘솔 pacing 거부 문구** 끝의 조건을 "(line_m 장면, 층 지연 예산 off 조건)"에서 "(line_m 장면)"으로 줄였다(총괄 결정, 이 PR). L04 가 예산 on·캐시에서도 통과했기 때문이다(확인 실행 2).
- **f 0.9 는 이 조건에서 쓸 수 없다.** 위 범위에서 등록 기준을 채우지 못했다.

## 범위

- 한 장면(line_m)·seed 0·짧은 hold 열의 확인이다(등록 4 절).
- 판정은 직접 조작 다섯 종류와 승강의 pacing 에 한정한다. 승강 높이 변화는 판정하지 않는다.
- 예산 on·캐시 포함 main(1f60d4e2 병합 뒤, 라이브 경로는 `ts2_65a862dd` 와 같음)에 한정한다.
- 판정한 세 세션은 재실행 하나씩이다. 실패 없는 한 자리(u02_re)는 그 자리가 다시 돌아도 통과한다는 뜻이 아니다.

## 자체 검토

- 초안은 API 크레딧 실행이 판정 출력에서 직접 읽어 썼고, 이 세션이 배율·ok·창 최솟값·실패·안전을 같은 키로 다시 읽어 대조했다.
- Codex 결과 검토(등록 스레드): P1 없음. 수치·판정·자리 규칙·VM 판정이 원자료와 맞고, 등록을 넘는 주장이 없다는 확인을 받았다.
  - P2(표본기 재시도가 앞선 시도의 양성을 덮어씀): 시도마다 출력을 남기고, 어느 시도든 양성이면 양성으로 고쳤다. 이번 재실행 표본 28 개는 모두 한 번에 성공해(`attempts` 1) 결과에 영향이 없다.
  - P3(README 의 R1 한계 줄과 새 줄이 모순되게 읽힘, K09 '두 번'의 조건 구분): 관측 시점·조건을 나눠 고쳤다.
- Codex 결과 검토 2라운드: P1 없음. 남은 P2 두 건 — 마지막 시도의 시각이 없으면 앞선 양성이 범위 판단에서 빠짐, 뒤 시도의 시간 초과가 앞 시도 기록을 버림. 시도마다 시각·출력을 남기고(시간 초과도 시도로 기록), 양성은 그 시도의 ws1 시각(없으면 노트북 시각)으로 범위를 판단하도록 고쳤다. README 지적은 해소됐다.
  - u01 drop 위치의 엄밀한 범위는 '후진 hold 끝부터 왼쪽 hold 시작까지(경계 포함)'다.
- 시간: 등록 추정은 3 잡 약 14 분이었다. 재실행 실측은 12 분 2 초(14:48:36–15:00:38)였다.

## 수치 출처

모두 `verdict_k09.json` 에서 직접 읽었다. 세션별 값은 `sessions[]` 에서 `session` 이 `*_re` 인 항목만 썼다.

- 판정·자리: `verdict.pass`(false), `verdict.factor`(0.9), `verdict.judged`, `verdict.left_out`, `verdict.unevaluated`(`[]`).
- 실패: `verdict.failures[]` 의 `rule`·`session`·`hold`·`value`.
  - `u01_k09_r1_re`: `pacing_drops` 1; `hold_output` left 0.6095; `pacing_gap` left 874.8 ms.
  - `u03_k09_r3_re`: `pacing_drops` 1; `hold_output` reverse 0.7820; `output_stutter` reverse 721.8 ms; `pacing_gap` reverse 721.8 ms; `hold_output` left 0.8705; `pacing_gap` left 289.0 ms.
  - `u02_k09_r2_re` 항목은 없다.
- TELEOP 배율: `summary.modes.TELEOP.ratio` min 0.8619, median 0.8653, max 0.8662.
  - 세션별 `sessions[].modes.TELEOP.ratio`: u01_re 0.8619, u02_re 0.8662, u03_re 0.8653.
- TELEOP ok: `summary.modes.TELEOP.ok_wall` min 0.9598, median 0.9605, max 1.0.
  - 세션별 `sessions[].modes.TELEOP.ok_wall`: u01_re 0.9605, u02_re 1.0, u03_re 0.9598.
- 세션 pacing drop: `sessions[].modes.TELEOP.pacing_drops`: 1·0·1.
- AUTO: `summary.modes.AUTO.ratio` 0.8896–0.8900, `summary.modes.AUTO.ok_wall` 0.9521–0.9542.
- 창: `verdict.windows.<세션>.teleop_min` / `.threshold` / `.teleop_below`.
  - u01_re 0.80949 / 0.81 / 1.
  - u02_re 0.81125 / 0.81 / 0.
  - u03_re 0.80976 / 0.81 / 1.
- hold: `sessions[].holds[]` 의 `label`·`ok_at_start`·`ok_share`·`output_share`·`first_output_ms`·`longest_not_ok_ms`·`pacing_drops`.
  - u01_re left: false / 0.6095 / 0.6095 / 874.8 / 874.8 / 0.
  - u01_re reverse: ok_share 1.0, pacing_drops 0.
  - u03_re reverse: true / 0.7850 / 0.7820 / 10.1 / 721.8 / 1.
  - u03_re left: false / 0.8705 / 0.8705 / 289.0 / 289.0 / 0.
  - 승강: `summary.holds.lift_up|lift_down.ok_share` 1.0, `.lift_delta_m` ±0.0234–0.0247 m.
  - 영상: `summary.holds.video_forward.output_share` 0.019–0.103, `video_reverse` 0.
- 겹침: `sessions[].overlap` 0.0(세 `_re`).
- 안전: `sessions[].safety` 의 `result`, `scene_damaged`, `forbidden_pocket_contacts`, `unpermitted_auto`, `teleop_overlap_ticks`, `unevaluated`, `teleop_min_clearance_m`(7.045·7.001·7.017).
  - 층 예산: `safety.layer_budget` 의 `budget_s` 0.1, `source`, `invariant_violations` 0, `watchdog.events` 0, `latency_record` true.
- 1회차 판정: `verdict_k09_round1.json` 의 `verdict.unevaluated`·`verdict.left_out`·`summary.sessions`(0).
- VM 표본:
  - `vm_watch_re.jsonl`: 28 줄. `ssh_exit` 0, `attempts` 1, `qemu` `[]`, `gpus` 2 개 `Enabled`. 첫 `t_ws1` 1794548902.81(14:48:22 KST), 끝 1794549712.88(15:01:52 KST).
  - `vm_watch_round1.jsonl`: 28 줄, `attempts` 필드 없음(보강 전 판).
    - 10번째 줄 `t_local` 1794548271.82(14:37:51)에서 `ssh_exit` 255.
    - 21·22번째 줄 `t_local` 1794548601.82·1794548631.82(14:43:21·14:43:51)에서 `ssh_exit` 255, `err` 는 DNS 실패.
    - 20·23번째 줄 `t_ws1` 1794548574.35·1794548664.45 로 간격 90.1 s.
  - KST 환산은 2차 기록의 대응(1794539390.47 = 12:09:50 KST)과 같은 기준이다.
- 잡 시각·상태: `sacct_14-25_14-55.txt`, `sacct_14-40_15-05.txt`.
- 1차 비교값: `docs/validation/2026-11-12-teleop-smooth.md` 의 팔별 결과 표와 결론 절.
