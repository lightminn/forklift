# 매끄러운 텔레옵 확인 실행 2 — 결과 (2026-11-13, 최종판)

등록: [`docs/plans/2026-11-13-teleop-smooth-2.md`](../plans/2026-11-13-teleop-smooth-2.md)(결과 전 고정, Codex 수렴).

판정 원자료:
- L04·K08: `2026-11-13-teleop-smooth-2/verdict_{l04,k08}.json`. `tools/teleop_realtime_report.py --verdict --budget-on --overlapped t07_k07_r2,t08_k06_r2,t09_l05_r2` 의 출력 그대로다(스냅숏 `ts2_65a862dd` 의 도구). `_re` 와 무관하므로 중간판의 것이 그대로 최종이다.
- K06·K07·L05: `2026-11-13-teleop-smooth-2/verdict_{k06,k07,l05}_final.json`. 같은 도구의 출력 그대로다. 팔마다 원래 세 세션과 그 자리의 `_re` 세션을 넣고 `--factor F --verdict --slots <세 자리> --budget-on --overlapped <겹친 자리>` 로 돌렸다(예: K07 은 `--slots t04_k07_r1,t07_k07_r2,t14_k07_r3 --overlapped t07_k07_r2`). 출력의 `verdict.judged` 가 원래 자리를 `_re` 로 바꿔 읽었고(`t07_k07_r2 → t07_k07_r2_re` 등), `verdict.left_out` 은 원래 세션이다. 중간판의 `verdict_{k06,k07,l05}.json` 은 `_re` 전의 출력으로 남겨 둔다.
- sacct 원출력: `2026-11-13-teleop-smooth-2/sacct_09-50_11-30.txt`(처음 15 잡), `sacct_11-45_12-20.txt`(재실행).
- VM 표본기 원출력: `2026-11-13-teleop-smooth-2/vm_watch_re.jsonl`.
- ws1 `test_runs/ts2/ts2_65a862dd/`(스냅숏 `snapshots/ts2_65a862dd`, 결과 문서 머지까지 보존).

## 실행

- 잡 18766–18780, 09:57–11:29 KST, 직렬. 15 잡 모두 COMPLETED 였다. 두 GPU 의 persistence 는 제출 전·끝 모두 Enabled 였다.
- 조건: 층 지연 예산 on(목록에 `LAYER_BUDGET` 없음 → 런처 기본 `config/layer_latency_budget.yaml`), 정확 키 캐시 포함(main 0092bcf7 위).
- **겹침.** 다른 사용자 CPU 겹침은 15 세션 모두 0 이었다. 같은 계정의 다른 잡 `wscheck`(18781–18784, CPU 10·10·2·10)이 세 세션 창에 들었다:
  - 18781·18782 가 t07_k07_r2 와 겹쳤다.
  - 18782·18783 이 t08_k06_r2 와 겹쳤다.
  - 18784 가 t09_l05_r2 와 겹쳤다.
  - 등록 2 절(1차 등록 3 절의 같은 계정 겹침 규칙)대로 세 세션을 판정에서 빼고(`--overlapped`), 그 자리를 `_re` 로 한 번씩 다시 돌았다.
- **재실행.** 스냅숏·OUT 은 원래와 같다(`ts2_65a862dd`).
  - 18786 `t07_k07_r2_re` 11:54:06–11:59:07, 18787 `t08_k06_r2_re` 11:59:07–12:04:48, 18788 `t09_l05_r2_re` 12:04:48–12:12:09. 셋 모두 COMPLETED 였다.
  - 제출 직전: 두 GPU persistence Enabled, qemu 0, squeue 0. 사용자가 11:52–11:53 에 VM 을 껐다.
- **재실행의 겹침.** 재실행 창 안에 같은 계정의 다른 잡은 없었다. 다른 사용자 CPU 겹침도 세 세션 모두 0 이었다(판정 출력의 `overlap`).
  - 18785 `slam_h_smoke` 는 11:52:17–11:52:41 로 앞에서 끝났다.
  - 18789·18790(`lbg2_1`·`lbg2_2`)은 12:16:35·12:16:36 에 시작해 뒤에 있다.
  - 다른 사용자 잡은 없었다(`sacct -a`).
- **VM 판정**(`vm_watch.py judge`, 판 4383bb2c 의 표본기, 등록 2 절의 VM 규칙).
  - `t07_k07_r2_re`: clear.
  - `t08_k06_r2_re`: clear.
  - `t09_l05_r2_re`: 미평가. 노트북 덮개가 닫혀 12:09:52 에 노트북이 절전에 들어갔다(journalctl: Lid closed, System Suspend). 표본이 12:09:50 → 12:15:58 사이 끊겨, END + 45 s 범위의 연속 완전 표본 조건을 채우지 못했다. 등록대로 기록 결측은 겹침으로 보지 않아 재실행하지 않는다.
  - 표본 35 개 전체에서 qemu 관측 0, GPU 2 개 미만 0, persistence Disabled 0 이었다. 끊기기 전 표본도 같았다.
  - clear 는 "등록한 표본 조건에서 VM 을 보지 못함"이고, VM 이 없었다는 증명은 아니다(등록 2 절).
  - 처음 15 세션은 VM 왕복 시험(11:30:12–11:32:38)과 그 뒤 VM 시작(11:34:02)보다 앞에 끝났다(등록 2 절).

## 결론

| 팔 | 판정 | TELEOP 배율 | TELEOP ok | 창 최솟값(문턱) | pacing 실패 |
|---|---|---|---|---|---|
| **L04** f 0.4·k0(운용 기본) | **통과**(3/3 자리) | 0.377–0.378 | 1.000 | 0.365–0.369(0.36), 문턱 아래 0 | 0 |
| **K08** f 0.8·k6 | **통과**(3/3 자리) | 0.798–0.799 | 1.000 | 0.768–0.769(0.72), 문턱 아래 0 | 0 |
| **K07** f 0.7·k6 | **통과**(3/3 자리: t04·t07_re·t14) | 0.700 | 1.000 | 0.663–0.677(0.63), 문턱 아래 0 | 0 |
| **K06** f 0.6·k6 | **통과**(3/3 자리: t03·t08_re·t13) | 0.600 | 1.000 | 0.576–0.579(0.54), 문턱 아래 0 | 0 |
| **L05** f 0.5·k0 | **불통과** — t02·t12 실패. t09_re 도 판정 출력에서 규칙 위반(VM 미평가) | 0.460 | 0.577·0.667·0.764 | 0.446–0.448(0.45), 문턱 아래 22·18·14 | 18·15·11 |

(L05 의 세 값은 t02·t12·t09_re 순서다.)

- **운용 기본(L04)은 예산 on·캐시에서도 끊기지 않았다.** 1차(예산 off·캐시 없음)의 창 최솟값 0.367–0.369 와 같은 수준이다. 예산 on 과 캐시가 함께 바뀌었으므로 둘의 몫은 가르지 않는다(등록 2 절).
  - 도움말 문구("콘솔 기본(0.4, catch-up 0)에서 직접 조작이 끊기지 않음")가 운용 기본(예산 on·캐시)에서도 확인됐다(등록 3 절). 문구 끝의 "층 지연 예산 off 조건"은 이 결과로 지울 수 있다(총괄 결정).
- **등록 후보 가운데 통과한 가장 높은 목표는 K 의 f 0.8(K08)이다.** K06·K07·K08 이 모두 통과했다. 더 높은 후보는 이 실행에 없다.
  - L 은 L04(f 0.4)가 통과하고 L05 가 불통과다.
  - 실제 한계를 찾은 것은 아니고, 단조성도 주장하지 않는다(등록 2 절).
- **L05(catch-up 0·f 0.5)는 이번 조건에서 등록 기준을 채우지 못했다.** 실제 상한이나 원인을 확정한 것은 아니다.
  - TELEOP 배율 0.460 이 문턱 0.45 바로 위에 붙어 있어 창이 자주 문턱 아래로 내려갔다.
  - 1차 등록의 재생 예측(catch-up 0·f 0.5 → 배율 0.460, ok 실패)과 같다.
  - 구동·승강 hold 의 가장 긴 not-ok 구간은 t02·t12 에서 0.38–1.24 s 였다. t09_re 의 값(1.02–1.24 s)도 그 범위 안이다.
  - t09_re 의 규칙 위반: `pacing_drops` 11. 짐 실은 전진·전진·후진 hold 에서 `hold_output`·`output_stutter`·`pacing_gap`. 승강 올림 hold 에서 `pacing_gap`.
  - t09_re 는 VM 미평가지만, t02·t12 두 자리가 이미 실패라 결론이 바뀌지 않는다.
- 후진 진단(K 가 무너질 때)은 K 가 무너지지 않아 하지 않았다(등록 3 절).
- **안전.** 15 세션과 재실행 3 세션 모두 같았다.
  - `scene_damaged` 없음, 포켓 금지 접촉 0, AUTO 의 허가 없는 진입 0, TELEOP 참값 hull 겹침 0.
  - 층 예산 불변조건 위반 0, 감시 사건 0.

## 다음

- K09 재실행을 등록한다(K08 통과, 등록 3 절·총괄 3). 등록 문서는 `docs/plans/2026-11-13-teleop-smooth-3.md` 로 별도 PR 에 들어간다.
- 기본값(FACTOR·catch-up) 승격 권고는 K09 결과 뒤의 총괄 결정이다. 이 문서는 권고하지 않는다.

## 범위

- 한 장면(line_m)·seed 0·짧은 hold 열의 확인이다(등록 4 절).
- 판정은 직접 조작 다섯 종류와 승강의 pacing 에 한정한다. 승강 높이 변화는 판정하지 않는다.
- 예산 on·캐시 포함 main(0092bcf7 위)에 한정한다.

## 자체 검토

- Codex 결과 검토(새 스레드): P1/P2 없음, 판정·수치·규칙 적용 동의. P3 세 건(L05 서술을 등록 기준 미충족으로 좁힘, 아직 없는 다음 등록으로의 링크를 경로 언급으로, 18790 시작 시각)을 반영했다.

## 수치 출처

K06·K07·L05 는 각 `verdict_*_final.json` 에서 직접 읽었다(초안은 API 크레딧 실행이 쓰고, 이 세션이 같은 키로 다시 읽어 대조했다). L04·K08 행은 중간판의 값 그대로다(`_re` 와 무관).

- 판정·자리: `verdict.pass`, `verdict.failures`, `verdict.judged`, `verdict.left_out`, `verdict.unevaluated`(셋 모두 `[]`).
- TELEOP 배율: `summary.modes.TELEOP.ratio.min`–`.max`.
  - K07 0.6996–0.7001, K06 0.5997–0.6001, L05 0.4603–0.4604.
- TELEOP ok: `summary.modes.TELEOP.ok_wall`(중간판과 같은 열).
  - K06·K07: min = max = 1.0.
  - L05: min 0.5769, median 0.6674, max 0.7638. 중간판의 t02 0.577·t12 0.667 과 맞춰 보아 max 가 t09_re 다.
- 창 최솟값·문턱·문턱 아래 스텝: `verdict.windows.<세션>.teleop_min` / `.threshold` / `.teleop_below`.
  - K07: t04 0.6631, t14 0.6769, t07_re 0.6746, 문턱 0.63, 아래 0·0·0.
  - K06: t03 0.5792, t13 0.5787, t08_re 0.5756, 문턱 0.54, 아래 0·0·0.
  - L05: t02 0.4474, t12 0.4455, t09_re 0.4475, 문턱 0.45, 아래 22·18·14.
- pacing 실패: `verdict.failures[]` 의 `rule == "pacing_drops"` 항목의 `value`.
  - L05: t02 18, t12 15, t09_re 11.
  - K06·K07: `failures` 가 비어 0.
- L05 t09_re 의 hold 위반과 not-ok 길이: `verdict.failures[]` 에서 `session == "t09_l05_r2_re"` 인 `hold_output`·`output_stutter`·`pacing_gap` 항목.
  - `pacing_gap` 값 1229.0–1237.5 ms(구동), 1016.3 ms(`lift_up`).
- 재실행 세션 안전: 각 파일 `sessions[]` 의 `_re` 항목.
  - `safety.result` true, `scene_damaged` null, `forbidden_pocket_contacts` 0, `unpermitted_auto` 0, `teleop_overlap_ticks` 0.
  - `layer_budget.budget_s` 0.1, `invariant_violations` 0, `watchdog.events` 0.
- VM 표본: `vm_watch_re.jsonl` 35 줄 모두 `ssh_exit` 0, `qemu` `[]`, `gpus` 2 개 `Enabled`.
  - 33번째 `t_ws1` 1794539390.47(12:09:50 KST), 34번째 1794539758.26(12:15:58 KST).
- 잡 시각·상태: `sacct_11-45_12-20.txt`.
