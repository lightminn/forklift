# 기준선 남은 실패 재확인 — random_1 seed 11·49 (2026-11-18)

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

- 두 seed 모두 운반 cusp 도착 방향 오차가 허용치 0.05 rad 안이라, #568 의 정지와 재계획 거부가 생기지 않았다.
- **기준선 남은 실패(이 두 seed)는 지금 main 에서 재현되지 않는다.**

## 3. 해석과 한계

- 기준선 R 이후 riib 런처 기본으로 켜진 것이 둘이다. 순응 타이어 `TYRE_CONTACT=f2`(#576)와 `GOAL_CREEP_BAND=on`(#571)이다. `SHORT_CUSP_ARC`(#566)는 aisle_rack 만이라 random_1 에서는 끔이다. 러너 코드도 2bea23e5 → 89f9cee1 사이에 336 커밋이 바뀌었다. 그래서 "타이어 때문에 사라졌다"고 원인을 한 변경에 돌리지 않는다. 이 문서가 말하는 것은 '지금 기본으로 두 seed 가 완주한다'까지다.
- 결정 모드 한 번씩이다. 같은 스냅숏·인자로 다시 돌리면 같은 궤적이 나오지만, 다른 seed·다른 장면에서 같은 꼴의 실패가 없다는 증거는 아니다.
- #568 이 적은 소프트웨어 후보 (a)(cusp_pose 재계획에 permit_stop 과 같은 장애물·여유 넘기기)는 이번 결과로 필요성이 생기지 않았다. 고치지 않고, 같은 꼴의 거부가 다시 보일 때 다시 연다.

## 4. 원자료

- ws1 `test_runs/acs/r11_49_89f9cee1/`(두 실행, `bundle/`, `RUNS.jsonl`), 스냅숏 `snapshots/main_89f9cee1`. 이 문서가 머지되면 정리 목록으로 넘긴다.
- 읽은 값: `run/result.json` 의 `success`·`transitions`·`cusp_stalls`·`cusp_replans`·`scene_replans`·`leg_switches[].yaw_error_rad`(운반 `phase == "transport"`), `riib_defaults.txt`.
