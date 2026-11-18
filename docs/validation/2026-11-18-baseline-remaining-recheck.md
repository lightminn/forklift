# 기준선 남은 실패 재확인 — random_1 seed 11·49·40 (2026-11-18)

- 배정(총괄, 사용자 승인 2026-11-18): 기준선 R(스냅숏 2bea23e5)에서 아직 못 고친 random_1 seed 11·49 의 전환점 재계획 거부([#568](2026-11-17-cusp-pose-replan-refused.md), `scene_replan_refused:cusp_pose:blocked:occupied`)가 순응 타이어 기본([#576](2026-11-18-f2-i-reregistration.md)) 뒤에도 남는지 본다. 새 고침은 하지 않는다.

## 1. 무엇을 돌렸나

| 항목 | 값 |
|---|---|
| 잡 | ws1 19921, `isaac_bundle` riib 2 슬롯, `BUNDLE_ARGS="--run-est-min 30 --retries 2"` |
| 스냅숏 | `main_89f9cee1`(main 그대로) |
| 실행 | riib random_1 seed 11·49, 결정 모드(`--deterministic` 만), 런처 기본 그대로, VIDEO=off |
| 런처 기본(`riib_defaults.txt`) | `TYRE_CONTACT=f2`, `GOAL_CREEP_BAND=on`, `SHORT_CUSP_ARC=off`(random_1 은 목록 밖), `RETURN_CUSP_REPLANS=2`, `NEAR_END_STAND=on`, `NEAR_OBSERVE_ARRIVAL=0.05,0.05` 등 |

- 두 실행 모두 첫 시도에 끝났다(기반 장애 재시도 0, 각 336–337 s).

## 2. 결과

| seed | 기준선 R(#568) | main_89f9cee1 |
|---|---|---|
| 11 | 운반 마지막 cusp 에서 방향 오차 −0.06495 rad 로 정지 → `cusp_pose` 재계획 거부 | **완주**(`complete`). cusp 정지·cusp 재계획·장면 재계획 0. 운반 cusp 도착 방향 오차 −0.0059·+0.0116 rad |
| 49 | 같은 꼴, −0.06628 rad | **완주**(`complete`). 위와 같이 0. 운반 cusp 도착 방향 오차 +0.0121·−0.0003 rad |
| 40(B0, 추가) | 수정 전 곡률 B0(`cmp_a2b91d2d`) 같은 꼴, −0.06635 rad | **완주**(`complete`, 스냅숏 `main_0b167934` — 코드는 89f9cee1 과 같음). 위와 같이 0. 운반 cusp 도착 방향 오차 −0.0016·+0.0073 rad |

- 두 seed 모두 운반 cusp 도착 방향 오차가 허용치 0.05 rad 안이라, #568 의 정지와 재계획 거부가 생기지 않았다.
- **`89f9cee1` 의 riib 기본값과 `--deterministic` 으로 random_1 s11·s49 를 각 1 회 돌렸고, 두 실행 모두 완주했으며 #568 의 실패는 관측되지 않았다.** 두 실행 모두 `deterministic.broken` 은 비어 있고, 결정 모드가 바꾼 값은 `recognition_proposals`(oracle)·`recognition_pixel_lag_steps` 둘이다.

- **추가(같은 날, 총괄 배정 "덱 04 장의 미해결 1 건"):** #568 의 세 번째 사례 B0 s40(random_1 seed 40)을 같은 방식으로 돌렸다. ws1 19922, 스냅숏 `main_0b167934`(89f9cee1 뒤로는 문서만 바뀐 main), 런처 기본(`TYRE_CONTACT=f2`·`GOAL_CREEP_BAND=on`·`SHORT_CUSP_ARC=off` 기록 확인), 결정 모드 1 회. 완주했고 `deterministic.broken` 은 비어 있다. **`89f9cee1`(s11·s49)과 `0b167934`(s40)의 riib 기본값에 `--deterministic` 을 적용해 각 1 회 실행했고, 세 실행 모두 완주했으며 `scene_replan_refused:cusp_pose:blocked:occupied` 는 관측되지 않았다.** B0 와 이번 s40 재실행에서 같은 것은 적재 곡률 규칙(`--loaded-curvature-rule`)이 둘 다 꺼져 있었다는 점까지다. 스냅숏(a2b91d2d → 0b167934)과 런처 기본(`TYRE_CONTACT=f2`·`GOAL_CREEP_BAND=on`)은 달랐다.

## 3. 해석과 한계

- 기준선 R 이후 riib 런처 기본으로 켜진 것이 둘이다. 순응 타이어 `TYRE_CONTACT=f2`(#576)와 `GOAL_CREEP_BAND=on`(#571)이다. `SHORT_CUSP_ARC`(#566)는 aisle_rack 만이라 random_1 에서는 끔이다. 코드도 2bea23e5 → 89f9cee1 사이에 329 커밋이 들어왔다(그중 `run_transport.py` 를 바꾼 것 11). 장면 파일·URDF·기준 USD 의 해시와 기준선 R 의 기록은 대조하지 않았다(R 원본은 Drive 보관). 장면 이름·seed·결정 모드 플래그만 같다. 그래서 "타이어 때문에 사라졌다"고 원인을 한 변경에 돌리지 않는다. 이 문서가 말하는 것은 '지금 기본으로 세 재실행(s11·s49·s40)이 완주한다'까지다.
- 결정 모드 한 번씩이다. 반복 실행으로 같은 궤적을 다시 확인하지는 않았다. 결정 모드(oracle 인식, 픽셀 지연 0) 결과를 운영 모드로 넓히지 않고, 다른 seed·다른 장면에서 같은 꼴의 실패가 없다는 증거도 아니다.
- #568 이 적은 소프트웨어 후보 (a)(cusp_pose 재계획에 permit_stop 과 같은 장애물·여유 넘기기)는, 이번 세 재실행(s11·s49·s40)에서 정지·재계획 분기가 발동하지 않아 검증을 보류한다. 고치지 않고, 같은 꼴의 거부가 다시 보일 때 다시 연다.

## 4. 원자료

- ws1 `test_runs/acs/r11_49_89f9cee1/`(두 실행, `bundle/`, `RUNS.jsonl`), 스냅숏 `snapshots/main_89f9cee1`(이미 정리됨); s40 은 `test_runs/acs/r40_0b167934/`, 스냅숏 `snapshots/main_0b167934`. 이 문서가 머지되면 정리 목록으로 넘긴다.
- 읽은 값: `run/result.json` 의 `success`·`transitions`·`cusp_stalls`·`cusp_replans`·`scene_replans`·`leg_switches[].yaw_error_rad`(운반 `phase == "transport"`), `riib_defaults.txt`.

## 5. 검토

- **Codex 1 회차.** P2 세 건: 결론을 실행 조건(스냅숏·기본값·결정 모드·각 1 회)에 묶고 운영 모드로 넓히지 않음, 커밋 수를 시험한 스냅숏까지(329, `run_transport.py` 11)로 고치고 입력 해시를 대조하지 않았음을 적음, 후보 (a)는 분기가 발동하지 않아 검증 보류로 좁힘.
- **Codex(s40 추가).** P2 둘: B0 와 같은 조건은 적재 곡률 규칙이 둘 다 꺼진 것까지로 좁힘(스냅숏·타이어·크리프 띠는 다름), 결론을 세 실행(89f9cee1 s11·s49, 0b167934 s40)·각 1 회로 묶고 후보 (a) 보류 줄도 세 재실행으로 맞춤.
