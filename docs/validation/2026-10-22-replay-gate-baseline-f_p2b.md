# M-a1 리플레이 게이트 — f_p2 기준 기록을 PR #89 뒤 동작으로 다시 만듦 (2026-10-22)

사용자 결정(2026-10-22, 총괄 세션 전달): PR #89(cusp 래치, 22f87d9) 뒤로 게이트 기록 `f_p2/seed_2_v0.055_plain_0` 은 의도한 동작 차이 때문에 tick 1441 에서
멈추고, 그 뒤 구간은 어떤 PR 에서도 검증되지 않는다. 이 기록을 #89 이후 동작으로 다시 만들어 게이트를 29/29 로 되돌린다.

## 한 일

- **다시 기록.** 같은 구성(일반 seed 2, factory 배치, 인식·SLAM 피드백·D7 계획·D7 도킹·장애물 층, `--record-boundary-format 2`)을 main `ec4bece`(PR #91 머지,
  #89 포함) 스냅숏 `ma1_rp_ec4bece` 로 Isaac 에서 한 번 돌렸다(ws1 잡 8252, 8 분 11 초). 명령줄은 옛 기록의 `result.json` `arguments` 에서 다시 만들었다
  (오늘 기본값과 다른 옵션만 내보내고, 기록 코드 스냅숏 경로와 출력 경로만 바꿈 — `--no-return-turnaround` 처럼 그 뒤 기본값이 바뀐 옵션은 옛 값으로 명시된다).
  새 기록: `artifacts/20261013_fork_spacing/f_p2b/seed_2_v0.055_plain_0/run`(명령줄 `command.txt`).
- **옛 기록은 그대로 둔다.** `artifacts/20261013_fork_spacing/f_p2/seed_2_v0.055_plain_0`(387 MB)은 옮기거나 지우지 않았다. 게이트 목록 `test_runs/runs29.txt` 의
  둘째 줄만 `f_p2b/...` 로 바꾸고, 바꾸기 전 목록을 `test_runs/runs29_2026-10-16_f_p2.txt` 로 남겼다(옛 기록으로 되돌릴 때 이 목록을 쓴다).
- 스크립트(ws1): `test_runs/replay_gate/rebuild_cli.py`(명령줄 재구성), `test_runs/replay_gate/rebase/rec_f_p2b.sbatch`(기록),
  `test_runs/replay_gate/rebase/compare_records.py`·`after_diff.py`(두 기록 비교).

## 새 기준이 #89 차이 말고는 옛 기록과 같다는 근거

- **임무 결과가 같다.** 두 기록 모두 `complete`, 26 576 tick, 단계 전환 시각이 모두 같다(approach 41.60 · insert 47.48 · lift 56.34 · extract 60.84 ·
  transport 66.53 · lower 162.57 · withdraw 167.07 · settle 173.04 · return_home 174.04 · home_settle 220.72 · complete 221.72 s), 하역 오차 0.0336 m 로 같다.
- **첫 불일치 이전이 모두 같다.** 두 경계 기록을 tick 마다 비교했다(포트 호출의 요청·응답·변경, tick 출력·분기·명령). 벽시계 값은 실행마다 달라 뺐다 — `Clock` 포트 응답,
  계획 `deadline`, 이름이 `wall_s` 로 끝나는 진단 값(스캔 `cast_wall_s`·`total_wall_s`, SLAM `slam_wall_s` 등). 그 밖의 모든 값이 tick −1 부터 1440 까지 같고,
  첫 차이는 **tick 1441 의 출력** 이다.
- **차이 지점의 원인은 #89 다.** tick 1441 출력의 차이는 `holds.cusp`(옛 False → 새 True)와 명령 곡률(옛 −0.36 → 새 −0.3514 m⁻¹, 유지된 조향)뿐이다.
  옛 기록의 관측 다리 cusp 래치 기록은 `closed: status:braking`(옛) 과 `key_changed` 둘이고, 새 기록은 `key_changed` 하나 — #89 가 막으려던 "반대 방향 제동으로 닫힘"
  이 사라진 것이다(#89 결과 절: 이 기록이 tick 1441 = 12.008 s 에 그렇게 닫혔다).
- **그 뒤 차이는 짧다.** tick 1441 이후 출력이 다른 tick 8 개, 분기가 다른 tick 1 개, 포트 호출(`Clock` 제외)이 다른 tick 11 개이고, 그 뒤로는 끝까지 같다 — 유지된
  조향 몇 tick 뒤 같은 상태로 돌아왔다.

## 게이트

- 바꾼 목록으로 M-a1 리플레이 게이트 29 건을 `ma1_rp_ec4bece` 로 돌렸다(ws1 잡 8253): **29/29 `REPLAY_EXIT 0`**. `f_p2b` 는 26 576 tick 끝까지 재생
  (`drive: returned`, `finish: ok`).
- 바꾸기 전 같은 코드의 게이트(PR #91 의 ecbf9b2)는 28/29 였고 남은 1 건이 옛 `f_p2` 였다.

## 주의

- 이 기준은 **#89 뒤 동작** 의 기록이다. 다음에 추종기·cusp·도킹처럼 기록된 동작을 바꾸는 PR 이 오면(예정: 추종기 다리 끝 크리프·교착 수정) 그 PR 은 같은 방식으로
  갈리는 기록을 가려 내고, 의도한 차이면 이 절차(같은 인자로 다시 기록 → 첫 불일치 이전 일치·차이 원인 → 목록 교체·옛 목록 보존)로 기준을 바꾼다.
- 기록 하나를 다시 만들 때 Isaac 렌더 비결정성으로 앞 구간이 갈릴 수 있다(M-a1 계획 R3: 근접 seed 1 의 끔/끔 두 번이 첫 캡처 프레임에서 갈림). 이번 기록은 그 일 없이
  1440 tick 까지 같았다 — 그렇지 않으면 위 근거가 성립하지 않으니 비교부터 한다.
- 2026-10-16 매니페스트(`docs/validation/2026-10-16-m-a1-replay-manifest.json`)는 그날의 기록이라 고치지 않았다. 오늘 목록의 f_p2 줄은 이 문서가 대체한다.
