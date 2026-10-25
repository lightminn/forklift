# occupied 정지 뒤 재출발 확인 — 검증 결과 (2026-10-26)

계획: `docs/plans/2026-10-25-restart-confirmation.md`(3 판, 계획·구현 Codex 수렴). 지시: 총괄(2026-10-25) — 인계 사례(CT-1 line_m P-3 return_home 정체)의 (c). 정책(사용자 2026-10-24): 안전 판정 규칙·런처 기본값은 사용자 결정 없이 정하고 문서·보고로 남긴다.
구현: branch `fix/zero-permit-creep`. 검증한 코드는 `fdf0e789`(Isaac 두 배치의 스냅숏 `rc_fdf0e789`) 이다. 그 뒤 origin/main 병합과 런처 기본값만 바뀌었다.

## 결론

- **리보틱스 런처 `RESTART_CONFIRM` 기본값을 3 으로 바꾼다.** 러너·근접 계약 기본은 0(끔)이다.
  - rack 통로에서 켬 팔 네 실행에 실제로 걸렸다(아래 표).
  - 켬에서 새 실패 유형이 없고, 장면별 완주가 끔보다 줄지 않았다.
  - 설계상 이 규칙은 정지 중 칠한 칸이 한 스캔 사라질 때마다 조금씩 나아가는 것을 막는다. 인계 사례의 기록 재현에서 N = 3 은 펄스 11 번을 모두 막았다.
- **CT-1·콘솔 런처(`deploy/console/isaac_live.sbatch`) 기본은 끔으로 둔다.** sw-연속이송-재관측과 합의했다(2026-10-26).
  - CT-1 비교에서 켬 팔 10 실행 모두 한 번도 걸리지 않았다(노출 0). 효과를 판정할 근거가 없다.
  - 이 런처는 콘솔 운용에도 쓰인다. 켜려면 `MORE=--restart-confirm-scans 3` 으로 켠다.
- Codex 교차검증: 계획·구현에서 "새 P1/P2 없음"까지 반복했다(계획 5 장). 이 문서의 검토는 아래 자체 검토 절이다.

## 무엇을 넣었나

계획서 2 장 그대로다.
- `forklift_core.control.restart_confirm.RestartConfirm`: 허가 자체의 상한이 이유 `occupied` 로 0 이 되면 확인을 건다. 그 뒤 칸을 지울 수 있는 센서(`may_clear`)마다 마지막 0 틱 이후 새 스캔 N 개가 연속으로 원래 상한 양수를 보일 때까지 출력은 0 이다. 원래 상한을 올리지 않는다(출력은 둘 중 작은 값).
- 적용 범위: 허가가 작동하는 단계, 포켓 단계가 아닐 때, 허가 자체의 상한에만 건다. 단계나 추종기가 바뀌면 버린다. 계획 대기의 관측 틱은 모은 것을 비운다.
- P6 정지 감시와 격자 `blocked_ticks`: 확인 대기 틱은 감시를 일시정지할 뿐 처음부터 다시 시작하지 않는다. 그 작업 시간은 3 s 에 더하지 않는다.
- 스위치 `--restart-confirm-scans N`(0 = 끔), 근접 계약 등록, 리보틱스 런처 `RESTART_CONFIRM`.
- 기록 `obstacle_layer.restart_confirm = {scans, arms, releases, gate_ticks}`.

## CPU 확인

- 단위 시험: `tests/unit/control/test_restart_confirm.py` 9 건, `test_core_permission.py`·`test_permit_stop.py`·`test_riib_scene_launcher.py` 의 관련 시험.
- 인계 사례 기록 선별(계획 1 장): N = 2 는 첫 펄스 12 틱을 남기고, N = 3 은 11 펄스를 모두 보류한다.
- 재생 게이트 runs29 29/29(`replay_gate/rc_fdf0e789`, 끔).
- ws1 전체 pytest(fdf0e789): 6842 passed, 알려진 ws1 전용 실패 10 건만.

## Isaac 비교

두 배치 모두 스냅숏 `rc_fdf0e789`, 묶음 런처, 같은 seed 를 두 팔이 번갈아 돌렸다. 팔 차이는 `--restart-confirm-scans 3` 하나다. Isaac 실행은 같은 코드·seed 에서도 갈라지므로 배치 안에서만 비교한다.

### 리보틱스 장면(rcf, 묶음 2 잡, 장면 `scenes_p1c_81a007b1`, 런처 기본)

| 장면 · seed | 끔 완주 | 켬 완주 |
|---|---|---|
| aisle_rack 0–7 | 2/8 | 2/8 |
| line_2_m 0–2 | 3/3 | 3/3 |
| trailer 0–4 | 3/5 | 4/5 |

- 켬에서 확인이 걸린 실행(무장·해제·출력을 막은 틱): aisle s0 1·0·60, s2 1·1·24, s4 3·2·264, s6 3·3·108, trailer s0 1·0·0. 나머지 켬 실행은 0 이다.
- 켬 실패:
  - aisle s1 은 삽입 중 오른쪽 포크 뒤끝 접촉이다. 전에 해제 비교에서도 나온 삽입 깊이 유형이고, 이 실행에서 확인은 걸리지 않았다.
  - aisle s3·s7 은 운반 전 근접 캡처 실패(`opening_width_mismatch`)다.
  - 나머지는 끔에도 있는 운반 시간 초과·복귀 추종 실패다.
- 확인이 걸린 aisle s4 는 운반 시간 초과로 끝났다. 끔 s4 도 같은 사유다. 확인이 정체를 만들었다는 근거는 없다. 운반 occupied 틱 수(켬 28 721, 끔 25 529)는 같은 규모다.

### CT-1 세션(rcq, 묶음 2 잡 — 한 잡이 20 실행을 모두 받음)

조건(sw-연속이송-재관측 합의): `ct1_queue.sbatch`, SCENARIO=queue, 고정 순서(line_m P-1:D-1,P-2:D-2,P-3:D-3 / random_m P-4:D-1,P-5:D-2,P-3:D-3), QUEUE_EXTRA 없음, reported 오도메트리, seed 0–4.

| 장면 | 끔 미션 완료 | 켬 미션 완료 |
|---|---|---|
| line_m | 11/13 | 10/12 |
| random_m | 10/13 | 14/14 |

- **노출 0**: 켬 10 실행 모두 `arms 0` 이다. 두 팔 모두 P6 정지가 없었고, 인계 사례의 return_home 정체는 다시 나지 않았다.
- 실패 미션:
  - 끔: line_m s2 미끄러짐, s4 `transport_recovery_after_docking`, random_m s0 `docking_retry_match_refused`, s2 하역 위치, s4 `session_anchor_unaligned`.
  - 켬: line_m s2 m2 복귀 추종 실패(확인 무장 0), s3 미끄러짐.
  - 두 팔 모두 확인과 무관한 유형이다.
- 켬 두 세션(line_m s0, random_m s2)은 m1·m2 를 마친 뒤 `slam_link_failed`(SLAM 링크 응답 없음, 콘솔 담당)로 세션이 끝났다. 표의 분모에서는 그 세션이 마친 미션만 셌다.
- 묶음 기록: 기반 실패·멈춤·`time_left` 0 건, 실행 길이 570–1798 s.

## 한계와 남은 일

- CT-1 에서 노출이 없었다. 인계 사례는 드문 정체이고, 이번 seed 들에서는 생기지 않았다. CT-1 의 기본값은 그 사례가 다시 모일 때 정한다.
- 확인은 칠한 칸이 다시 칠해질 때까지의 진동을 막을 뿐, 그 정체 자체(계획 여유와 표시 반경의 차, P1c)를 풀지 않는다. CT-1 쪽 정지 원인은 KEEPOUT 비교(`feat/ct1-keepout`)가 다룬다.
- 확인 대기는 재출발을 스캔 세 번만큼(약 0.3 s) 늦춘다. 이번 비교에서 그것이 완주를 줄이지는 않았다.

## 실행 위치

- ws1 `test_runs/riibotics/rcf_*`, `rcf_bundle`; `test_runs/live/rcq_fdf0e789_*`, `rcq_bundle`(요약 `rcq_bundle/rcq_sum.py`); 스냅숏 `snapshots/rc_fdf0e789`; 재생 게이트 `test_runs/replay_gate/rc_fdf0e789`.

## 자체 검토

- (결과 뒤 적는다)
