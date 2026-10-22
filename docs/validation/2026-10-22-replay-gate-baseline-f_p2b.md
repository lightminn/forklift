# M-a1 리플레이 게이트 — f_p2 기준 기록을 PR #89 뒤 동작으로 다시 만듦 (2026-10-22)

사용자 결정(2026-10-22, 총괄 세션 전달): PR #89(cusp 래치, 22f87d9) 뒤로 게이트 기록 `f_p2/seed_2_v0.055_plain_0` 은 의도한 동작 차이 때문에 tick 1441 에서
멈추고, 그 뒤 구간은 어떤 PR 에서도 검증되지 않는다. 이 기록을 #89 이후 동작으로 다시 만들어 게이트를 29/29 로 되돌린다.

## 한 일

- **다시 기록.** 같은 구성(일반 seed 2, factory 배치, 인식·SLAM 피드백·D7 계획·D7 도킹·장애물 층, `--record-boundary-format 2`)을 main `ec4bece`(PR #91 머지,
  #89 포함) 스냅숏 `ma1_rp_ec4bece` 로 Isaac 에서 한 번 돌렸다(ws1 잡 8252, 8 분 11 초). 명령줄은 옛 기록의 `result.json` `arguments` 에서 다시 만들었다
  (오늘 기본값과 다른 옵션만 내보내고, 기록 코드 스냅숏 경로와 출력 경로만 바꿈 — `--no-return-turnaround` 처럼 그 뒤 기본값이 바뀐 옵션은 옛 값으로 명시된다).
  새 기록: `artifacts/20261013_fork_spacing/f_p2b/seed_2_v0.055_plain_0/run`(명령줄 `command.txt`).
- **옛 기록은 그대로 둔다.** `artifacts/20261013_fork_spacing/f_p2/seed_2_v0.055_plain_0`(387 MB)은 옮기거나 지우지 않았다. 보관본도 있다 — 2026-10-19 의
  fork_spacing 보관에서 게이트 기록 폴더들은 `gdrive:forklift-archive/ws1-fork-spacing-20261019/_keep.tar.zst` 로 올리고 ws1 에 남겼다(드라이브 MD5
  `7ff6b99422fd1d886571c257bbe4d36b`). 그 목록 `_keep.files.tsv` 의 이 기록 `run/boundary` 일반 파일 178 개가 ws1 원본의 178 개와 상대 경로·크기까지 모두 같다
  (`find -type f -printf '%p\t%s'` 대조). 파일 내용의 해시는 대조하지 않았다(목록에 해시가 없고, 묶음을 받아 풀지 않았다). 이번에 다시 보관하지는 않았다.
- 스크립트(ws1): `test_runs/replay_gate/rebuild_cli.py`(명령줄 재구성), `test_runs/replay_gate/rebase/rec_f_p2b.sbatch`(기록),
  `tools/boundary_diff.py`(저장소, 첫 차이) 와 `test_runs/replay_gate/rebase/after_diff2.py`(같은 규칙으로 그 뒤 차이 전부).

## 새 기준과 옛 기록의 차이 — 첫 차이는 #89, 그 뒤 출력 차이는 8 tick

- **임무 결과가 같다.** 두 기록 모두 `complete`, 26 576 tick, 단계 전환 시각이 모두 같다(approach 41.60 · insert 47.48 · lift 56.34 · extract 60.84 ·
  transport 66.53 · lower 162.57 · withdraw 167.07 · settle 173.04 · return_home 174.04 · home_settle 220.72 · complete 221.72 s), 하역 오차 0.0336 m 로 같다.
  cusp 래치 기록은 다르다(아래).
- **비교 규칙.** 저장소의 `tools/boundary_diff.py` 규칙을 그대로 썼다 — 이름이 `_wall_s` 로 끝나는 키·`wall_time_s`·`elapsed_s`·`wall_s_*` 와 벽시계 읽기
  (`Clock.monotonic`·`time`·`perf_counter`)의 응답 값만 빼고(호출 순서·요청은 비교), 시뮬레이션 시각·계획 `deadline` 을 포함한 나머지는 모두 비교한다.
  `boundary_diff.py` 결과: `differ`, 1 443 tick 비교, **첫 차이 tick 1441 의 outputs** — tick −1 부터 1440 까지 같다(ws1 `test_runs/replay_gate/rebase/boundary_diff_f_p2_f_p2b.json`).
- **첫 차이는 #89 다.** tick 1441 출력의 차이는 `holds.cusp`(옛 False → 새 True)와 명령 곡률(옛 −0.36 → 새 −0.3514 m⁻¹, 유지된 조향)뿐이다.
  옛 기록의 관측 다리 cusp 래치 기록은 `closed: status:braking` 과 `key_changed` 둘이고, 새 기록은 `key_changed` 하나 — #89 가 막으려던 "반대 방향 제동으로 닫힘"
  이 사라진 것이다(#89 결과 절: 이 기록이 tick 1441 = 12.008 s 에 그렇게 닫혔다).
- **그 뒤 차이를 모두 나열했다**(같은 규칙, tick 1441 이후 전부 — ws1 `test_runs/replay_gate/rebase/after_diff2_f_p2_f_p2b.txt`, 마지막 차이 tick 26 573).
  - 출력: tick 1441–1448 의 8 tick, 위의 두 값만(유지된 조향). 그 뒤 출력·분기·명령은 끝까지 같다.
  - tick 1449: 호출 순서가 다르다(옛: 장애물 층 스냅숏·후보 경로 검사, 새: SLAM 추적기 모드 읽기) — 래치가 닫히지 않고 유지 끝에 이어진 결과로, 같은 원인의 다음 tick 이다.
  - tick 2696·4165·4970: 카메라 캡처의 렌더 결과(이미지 해시)·`Capture.state` 진단·그 이미지를 받은 검출 요청이 다르다 — 같은 요청에 렌더러가 다른 프레임을 낸
    `boundary_diff` 의 "알려진 외부 차이"(M-a1 R3 의 렌더 비결정성)다. `Capture.state` 의 `instance_ids` 는 기록 코드(cd23f87) 뒤에 더해진 필드라 새 기록에만 있다.
    검출 결과 뒤의 출력은 같았다.
  - tick 1579·7950·18327·18948·20733·20852: 격자 준비(`Planner.grid_kwargs`)가 읽은 SLAM 지도 순번(`index`·`after_scan_id`)이 다르다 — 비동기로 도착하는 지도의
    알려진 외부 차이다. 7950·20852 에서는 그 응답과 다음 계획 요청의 `deadline`(프로세스 단조 시계 기준 절대값, 676 447 vs 1 187 701)도 다르다 — 시작 시각이
    다른 두 프로세스의 절대 시각이다. 예산은 같다 — 두 기록 모두 그 `deadline` 에서 같은 tick 의 바로 앞 벽시계 읽기(각 tick 의 call 12·10)를 빼면
    20.0 s 다(core 의 `clock + 20.0`).
  - tick 26 573: 끝의 결과 파일 쓰기(`Sink.write_text`) 내용이 다르다 — 위의 차이(래치 기록·벽시계 진단)를 담은 파일이다.
- 그러니 확인한 것은 "첫 차이가 #89 의 의도한 변경과 부합하고, 그 뒤 출력 차이는 같은 유지의 8 tick 이며, 그 밖의 차이는 기록 필드 추가(`Capture.state.instance_ids`)와, `boundary_diff` 가 알려진 외부 차이로 분류하는 렌더·지도 도착, 그리고 절대 시각" 까지다 —
  그 분류는 비교기의 것이고 개별 차이의 원인을 따로 확정한 검사는 아니다.
  경계 기록이 다시 같아졌다는 것이 내부 상태 전체가 같다는 뜻은 아니다(래치 기록은 실제로 다르다).

## 게이트

- 바꾼 목록으로 M-a1 리플레이 게이트 29 건을 `ma1_rp_ec4bece` 로 돌렸다(ws1 잡 8253): **29/29 `REPLAY_EXIT 0`**. `f_p2b` 는 26 576 tick 끝까지 재생
  (`drive: returned`, `finish: ok`).
- 바꾸기 전, 재생 관련 코드(`forklift_core`·Isaac 실행기·재생 도구)가 같은 리비전의 게이트(PR #91 의 ecbf9b2)는 28/29 였고 남은 1 건이 옛 `f_p2` 였다.

## 주의

- 이 기준은 **#89 뒤 동작** 의 기록이다. 다음에 추종기·cusp·도킹처럼 기록된 동작을 바꾸는 PR 이 오면(예정: 추종기 다리 끝 크리프·교착 수정) 그 PR 은 같은 방식으로
  갈리는 기록을 가려 내고, 의도한 차이면 이 절차(같은 인자로 다시 기록 → 첫 불일치 이전 일치·차이 원인 → 목록 교체·옛 목록 보존)로 기준을 바꾼다.
- 기록 하나를 다시 만들 때 Isaac 렌더 비결정성으로 앞 구간이 갈릴 수 있다(M-a1 계획 R3: 근접 seed 1 의 끔/끔 두 번이 첫 캡처 프레임에서 갈림). 이번 기록은 그 일 없이
  1440 tick 까지 같았다 — 그렇지 않으면 위 근거가 성립하지 않으니 비교부터 한다.
- 2026-10-16 매니페스트(`docs/validation/2026-10-16-m-a1-replay-manifest.json`)는 그날의 기록이라 고치지 않았다. 오늘 목록의 f_p2 줄은 이 문서가 대체한다.
