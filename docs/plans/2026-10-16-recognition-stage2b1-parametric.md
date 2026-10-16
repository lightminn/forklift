# 팔레트 인식 일반화 단계 2B-1 — 비규격 매개변수 가설과 원거리 집기 허가 관문 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **실행 방식(사용자 결정 2026-10-15, 단계 1A·1B·2A 와 같음):** 이 세션에서 직접 구현(executing-plans), 검토 지점마다 Codex 검토를 수렴까지 받고 진행한다.

**Goal:** 알려진 형상으로 설명되지 않는 진입면을 **비규격 매개변수형**(부재·개구 배치, 차선 천장, 양 끝 물리 확인 — 설계 §7.1, §7.4-5c)으로 읽어 `PocketCandidate` 를 내고, **원거리 경로가 집기를 허가하지 않도록**(근거리 재관측 §9.1 전까지 `pickable` 은 `False` 또는 `None`, 미확인 항목은 `pending`) 공통 관문을 둔다. 평가기에는 보류된 후보의 자세 위험을 보이는 진단 `pose_dangerous` 를 더한다.

**Architecture:** `instance_fit.py` 에 매개변수 판독(`read_parametric_face`)과 후보 조립(`_parametric_candidate`, `_parametric_fit`)을 더하고, 알려진 형상 판독이 `shape_mismatch`·`weak_fit` 일 때만 시도한다. 알려진 형상·비규격 모두 원거리 후보는 면·지시가 정하는 사유가 있으면 `False`, 없으면 `None` + `FAR_PENDING`. `pocket_candidate.py` 에 `pending` 필드, `evaluation_multi.py` 에 `pose_dangerous` 진단, `target_selection.py` 는 비규격 후보의 중심을 깊이 범위 전체로 본다. 실행은 ws1 Slurm CPU.

**Tech Stack:** numpy, PyYAML(기존), pytest. 실행 ws1 Slurm(`compute`), 단위 시험 CPU 리그(`tools/instance_rig.py`).

**Spec:** `docs/design/2026-10-15-pallet-recognition-generality.md` §3.4, §7.1, §7.4, §7.5, §8, §9.1, §9.3, §12 항목 2. 단계 2A: `docs/plans/2026-10-16-recognition-stage2a-instance-fit.md`, `docs/validation/2026-10-16-recognition-stage2a-oracle-fit.md`.

## 계획 전 실측 (2026-10-16, 이 계획의 근거)

작업 1–3 의 코드와 시험(이 계획의 코드블록과 같은 파일)을 ws1 Slurm 에서 돌렸다 — 단위·통합 시험 159 개 통과(잡 3211). dev 집계는 1B 렌더 `20261015T150838Z_recognition_isaac` 의 dev 분할 전체를 정답 인스턴스 ID 영상을 제안으로 써서 맞춘 것이다(oracle-fit). **비교는 같은 평가기**(작업 1 의 `evaluation_multi.py`)로 한다: 2A 머지 커밋 `93f1e47` 에 최종 평가기(`a24ec8d` 의 `evaluation_multi.py`)만 얹은 실행(잡 3255)과 2B 커밋 `a24ec8d`(잡 3260, 1G·2 CPU·약 4 분). 둘 다 커밋된 소스에서 돌렸다(`source_manifest.json` 에 소스·평가기 리비전). 오버레이 단계의 잡 3184·3218 과 2B 쪽 산출물은 같다. 2A 쪽 첫 비교(잡 3177)는 정답 끼움 불가 면을 자세 판정에서 빼기 전의 평가기였어서 S7 자세 위험이 낮음·중간에서 1 씩 많았다 — 아래는 최종 평가기 값이다.

셀은 장면 수로 `놓침 / 위험 / 자세 위험 / 대상 오류 / 보류(애매하지 않은 지시)`, 왼쪽이 2A·오른쪽이 2B 다. 음성(N)은 오검출 수(oracle-fit 은 정답 인스턴스만 제안하므로 구조적으로 0).

| 사례 · 형상 | 장면 | 잡음 없음 | 잠정 낮음 | 잠정 중간 | 잠정 높음 |
|---|---|---|---|---|---|
| N 음성 | 65 | 0 ‖ 0 | 0 ‖ 0 | 0 ‖ 0 | 0 ‖ 0 |
| S1 EPAL 6 | 42 | 1/0/0/0/1 ‖ 1/0/0/0/1 | 7/2/2/0/7 ‖ 7/0/2/0/7 | 5/6/6/0/5 ‖ 5/0/6/0/5 | 8/13/13/0/7 ‖ 8/0/13/0/7 |
| S1 T11 | 42 | 1/0/0/0/0 ‖ 1/0/0/0/0 | 3/5/5/0/3 ‖ 3/0/5/0/3 | 4/5/5/0/4 ‖ 4/0/5/0/4 | 5/12/12/0/3 ‖ 4/0/12/1/2 |
| S2 EPAL 6 | 42 | 11/0/0/1/7 ‖ 11/0/0/1/7 | 18/3/3/1/9 ‖ 18/0/3/1/9 | 20/5/5/1/9 ‖ 20/0/5/1/9 | 25/13/13/1/10 ‖ 24/0/13/1/10 |
| S2 T11 | 42 | 7/0/0/0/2 ‖ 7/0/0/0/2 | 14/13/13/0/4 ‖ 14/0/13/0/4 | 16/17/17/0/6 ‖ 16/0/17/0/6 | 22/18/18/0/9 ‖ 22/0/18/0/9 |
| S3 EPAL 6 | 42 | 8/0/0/0/16 ‖ 8/0/0/0/16 | 16/0/0/0/16 ‖ 16/0/0/0/16 | 14/5/5/0/16 ‖ 14/0/5/0/16 | 12/9/9/0/16 ‖ 12/0/9/0/16 |
| S3 T11 | 42 | 35/0/0/0/15 ‖ 35/0/0/0/15 | 36/5/5/0/16 ‖ 36/0/5/0/16 | 35/6/6/0/16 ‖ 35/0/6/0/16 | 37/8/9/0/16 ‖ 36/0/9/0/16 |
| S4 EPAL 6 | 42 | 3/0/0/0/4 ‖ 3/0/0/0/4 | 10/2/2/0/8 ‖ 10/0/2/0/8 | 8/5/5/0/8 ‖ 8/0/5/0/8 | 11/6/6/0/12 ‖ 11/0/6/0/12 |
| S4 T11 | 42 | 10/0/0/0/7 ‖ 10/0/0/0/7 | 14/2/2/0/9 ‖ 14/0/2/0/9 | 17/5/5/0/9 ‖ 16/0/5/0/9 | 20/6/6/0/11 ‖ 19/0/6/0/11 |
| S5 EPAL 6 | 42 | 9/0/0/0/8 ‖ 9/0/0/0/8 | 13/3/3/0/9 ‖ 13/0/3/0/9 | 12/7/7/0/8 ‖ 12/0/7/0/8 | 15/9/9/0/11 ‖ 14/0/9/0/11 |
| S5 T11 | 42 | 10/0/0/0/5 ‖ 10/0/0/0/5 | 12/3/3/1/5 ‖ 12/0/3/1/5 | 12/11/11/1/6 ‖ 12/0/11/1/6 | 20/13/13/0/9 ‖ 18/0/13/0/9 |
| S6 EPAL 6 | 42 | 4/0/0/0/3 ‖ 4/0/0/0/3 | 8/4/4/0/8 ‖ 8/0/4/0/8 | 7/6/6/0/7 ‖ 7/0/6/0/7 | 8/6/6/0/7 ‖ 8/0/6/0/7 |
| S6 T11 | 42 | 5/0/0/0/5 ‖ 5/0/0/0/5 | 9/5/5/0/8 ‖ 9/0/5/0/8 | 13/7/7/0/11 ‖ 13/0/7/0/11 | 17/10/10/0/15 ‖ 16/0/10/1/14 |
| S7 비규격 | 135 | 112/6/0/1/112 ‖ 50/0/0/2/47 | 115/8/2/1/115 ‖ 65/0/2/2/63 | 117/8/3/1/117 ‖ 72/0/3/2/69 | 118/7/5/1/118 ‖ 78/0/7/2/76 |

- **S1–S6.** 잡음 없음·낮음은 모든 셀에서 놓침·대상 오류·보류가 2A 와 같다. 잠정 중간은 S4 T11 놓침 하나(17→16, `s4_t11_06_1000460`), 높음은 장면 1–2 개씩 다르다 — 높은 잡음에서 형상 불일치로 떨어진 알려진 형상 일부가 매개변수로 읽혀 놓침이 줄고, 그중 S1·S6 T11 각 1 개는 지시 하중이 없어 `support_unverified` 로 멈춰 대상 오류가 됐다(하중 선언 요구를 지킨 동작이지만 알려진 형상을 오독한 파이프라인 실패다 — 대상 오류에 그대로 둔다). 위험은 관문 때문에 0 이다. **자세 위험은 두 실행에서 같은 정의로 잰 값이 네 잡음 수준 모두 장면 단위로 같다.** 정답 면이 끼움 불가라 자세 판정에서 뺀 후보는 형상마다 1–3 개다.
- **S7 놓침**: 112→50(잡음 없음), 115→65(낮음), 117→72(중간), 118→78(높음). 위험 6–8→0. 대상 오류 1→2(네 수준 같음 — 1000791 은 2A 에도 있던 것으로, 3.3 m 에서 EPAL 틀이 82 mm 어긋나 맞고 틀의 간격 판정으로 멈춘다. 1000873 은 Review Focus 5 의 옆면 판독). 자세 위험(실패/검사 장면): 2B 0/39·2/33·3/32·7/29, 2A 0/10·2/7·3/8·5/8. 자세 판정에서 뺀 후보(정답상 끼움 불가인데 원거리가 멈추지 않은 것 — 들기·평판 상판·깊이처럼 근거리만 가르는 사유): 2B 21·16·15·11, 2A 6·6·5·2. 비규격 끼움 판정 완료: 2B 25·21·16·17 / 135, 2A 22·20·18·17.
- **남은 S7 놓침의 원인**(첫 후보의 상태, 장면 수). 잡음 없음 50: 알려진 형상 EPAL 판독의 `lateral_unconstrained` 18(매개변수 트리거 밖 — Ruling), 매개변수 `weak_fit` 14, 매개변수 `end_unobserved` 6(양 끝 중 하나가 가려지거나 화면에서 잘림), 알려진 형상 `shape_mismatch` 인데 매개변수 판독도 실패 4, 매개변수 `fitted` 인데 정답에 대응되지 않음 3(1000873 옆면 등), 그 밖 5. 잠정 중간 72: 매개변수 `weak_fit` 25, 알려진 형상 `weak_fit` 15, `shape_mismatch` 9, `lateral_unconstrained` 9, `end_unobserved` 7, 그 밖 7.
- 매개변수 판독으로 대응된 후보의 자세 p95(가로·삽입축·yaw·높이): 잡음 없음 14.0 mm·0.1 mm·0.02°·12.8 mm(62 개), 잠정 중간 22.4 mm·32.2 mm·3.7°·13.5 mm(45 개). 잡음 없음의 가로 14 mm 는 알려진 형상(5–7 mm)보다 크다 — 부재 경계를 5 mm 격자로 읽기 때문이다. 높이는 차선 바닥을 읽지 않아 하판/2 만큼 낮게 나온다(설계 해석 절).
- **같은 날 팀 D435i 촬영에서 관찰한 것(공식 잡음 맞춤 전, 성능 대응은 미확인).** 벽 ROI(메타데이터)의 프레임마다 화소 좌표 (u, v) 에 깊이 평면을 최소제곱으로 맞추고 잔차의 MAD×1.4826 을 낸 뒤 30 프레임 중앙값을 보면 1 m 5.5(640)·6.0(848) · 2 m 26.4·28.4 · 3 m 57.6·60.7 mm 로, 잠정 모형의 σ_z ≈ k z² 를 그 거리에 넣은 값(중간 5.2/20.7/43.9, 높음 6.9/27.5/61.6)과 규모가 비슷하다. 다만 방법에 따라 값이 달라지고(Codex: 같은 `wall_2p00m_848` 첫 프레임을 3차원 평면에 맞춘 잔차 표준편차 31.4 mm), 잔차 규모가 비슷하다는 것만으로 공간 상관·결측 구조·내부 파라미터까지 같다고 할 수 없으므로 **dev 의 어느 잡음 열이 현실 성능에 대응하는지는 확인하지 않았다** — 공식 잡음 맞춤(`tools/calibrate_depth_noise.py` 절차)과 그 수준의 dev 재집계로 정한다. 장난감 팔레트(0.5–2 m)에서는 차선 높이 행의 무효 화소가 0–2 %(무효는 다리 왼쪽 1–2 px 스테레오 그늘)였다. **유효 깊이는 `BEHIND` 와 다르다** — 개구 관문은 깊이 > 면 + max(30 mm, 0.015 t²) 인 광선 3 개 이상을 요구하므로, 이 관문이 실측에서 성립하는지는 재생 검증 전까지 미확인이다.

## 설계 해석 — 원거리 집기 허가 (Codex 상의 2026-10-16, 5회차 수렴)

**문제.** 2A 코드에 매개변수 판독만 더한 첫 dev(잡 3161)에서 S7 위험 6건(잡음 없음)이 **모두 알려진 형상으로 `fitted`·`pickable True` 인 비규격**이었다. 4건은 T11 과 정면 배치가 같고 차선이 10→55–57 mm(T11 15→60 mm, 필요 천장 = 날 윗면 52.5 + 5 = 57.5 mm 에 0.8–2 mm 못 미침, 정답 `lift_alignment_needed`), 2건은 EPAL 과 정면이 같고 상판이 평판(정답 `near_tracking_unavailable`)이다. 틀 개구 안 면 광선 비율(알려진 형상 p50 0.065·p99 0.52 — 비스듬한 차선 벽·바닥이 면 허용오차 안)과 관측 폭 − 틀 폭(알려진 형상 |dW| p95 51 mm, 15 mm 문턱이면 184/570 거부)으로는 가를 수 없었다.

**결정.** 원거리에서는 어떤 후보에도 집기를 허가하지 않는다. 알려진 틀이 맞았다는 것이 측정하지 않은 상판·깊이·세로 경계까지 확인했다는 뜻이 되면 안 되기 때문이다(Codex P1). §9.1 은 이미 선택 대상의 상판 배치·앞뒤끝을 근거리(≤ 1.5 m)에서 확인해 `ObservedPalletShape` 를 확정한다고 적고 있다. 그래서:

- 원거리 후보는 면·지시가 정하는 사유(알려진 형상은 `fork_fit` 사유, 비규격은 `fork_spacing_adjust`·`support_unverified`·`payload_limit`)가 있으면 `pickable False`, 없으면 `None`. 미확인 항목은 `pending = FAR_PENDING = ("depth", "lane_bounds", "top_layout")` 로 남긴다. `True` 는 `pending` 이 비어야만 낼 수 있다(타입 검증).
- `target_selection._decide` 는 `None` 도 `selected` 로 내므로(75행) 이 관문이 대상 선택·콘솔 요청률을 바꾸지 않는다. 선택과 삽입 허가는 단계 4 에서 실행기에서도 분리한다(`PocketObservation` 에는 `pickable/pending` 이 없으므로 변환 때 보류 상태를 잃지 않게 한다 — 단계 4 계획의 조건).
- `lane_bounds` 는 차선의 **가로·세로 경계** 둘 다다. 근거리는 두 경계를 다시 재서 M_REQ 66 mm 와 날 세로 여유로 다시 판정하고, **포켓 추정값(`PocketEstimate`)도 정답 계약(포켓 중심 = (하판 + 천장)/2)에 맞게 다시 만든다** — `pending` 만 지우고 끝내면 안 된다(Codex 계획 검토). 지금 매개변수 후보는 차선 바닥을 읽지 않아 포켓 중심을 받침면 + 천장/2 에 두므로 정답보다 하판/2 만큼 낮다(대응 S7 매개변수 후보의 높이 오차 p95: 잡음 없음 12.8 mm·62 개, 잠정 중간 13.5 mm·45 개, 설계 S7 높이 문턱 5 mm).
- 세로(차선 천장·바닥)는 2B-1 에서 정하지 않는다. **현재 추정기는** 그 정밀도를 보장하지 못한다(매개변수 판독 격자 5 mm, 세로 화소 간격 2·3·4 m 에서 4.3·6.4·8.6 mm, 필요한 판별 0.8–2 mm). 매개변수 판독의 관측 천장은 `layout` 에 진단으로 남는다. 근거리 세로 측정은 단계 4.
- 비규격 지지는 **지시 하중만** 쓴다(`fork_fit` 130행과 같음). 하중이 없거나 지시가 아니면 `support_unverified → False`(근거리 관측으로 질량·질량중심을 채울 수 없다). 깊이를 모르므로 종방향은 깊이 상한 1.2 m 에서의 최대 삽입 목표(330 mm)로 판정하고, 개구 쌍은 가로 여유와 지지를 **함께** 보고 고른다(Codex 반례: 개구 [−325,−135]·[−50,170]·[240,460] mm, 폭 1040, 간격 290 이면 최대 여유 쌍 (1,2) 는 중심 205 mm 로 횡지지 실패, 쌍 (0,1) 은 통과).
- 가로 여유도 같은 원칙이다. 매개변수 판독의 가로 여유는 잡음에서 정답보다 크게 낮게 나온다(측정 − 정답, mm: 잡음 없음 p5/p50/p95 −5.2/1.0/8.3, 잠정 낮음 −12.3/−3.1/10.8, 중간 −12.5/−2.9/12.0, 높음 −22.9/−4.7/6.3, 최저 −42.5). 조리법의 집을 수 있는 S7 은 여유가 대부분 70–72 mm(M_REQ 66 mm 위 4–6 mm)라, 원거리에서 66 mm 로 판정하면 첫 dev 재집계에서 낮음 7·중간 6·높음 7 장면을 잘못 멈췄다. 그래서 원거리는 측정 여유 < M_REQ − δ 일 때만 `fork_spacing_adjust` 로 멈추고, 그 사이는 `None`(근거리가 66 mm 로 판정). **δ = 30 mm 는 잠정 잡음 dev 에서 고른 원거리 조기 거부의 경험적 여유값이고 오차 상한으로 검증된 값이 아니다**(Codex: 정답 67 mm 가 −32 mm 로 읽히면 여전히 잘못 멈춘다). 공식 잡음(팀 D435i) 뒤 다시 정하고, 확인은 eval 이 아니라 추가 dev 시드와 M_REQ 근처 형상으로 한다. 개구 쌍은 여전히 가로·지지를 함께 보고 고른다.
- 설계 지표는 바꾸지 않는다(Codex P2: 상판은 0.27 m 카메라에서 보인다 — CPU 리그로 `flat→boards` 만 바꾸면 1000774 에서 526 화소, 1000918 에서 151 화소의 깊이가 1 mm 넘게 달라진다. "원리상 관측 불가"는 성립하지 않는다). 정지 위험 지표의 원래 정의는 그대로이고, 원거리에서 `True` 가 없으므로 **집기 허용 후보에 따른 위험은 구조적으로 0**이다. 계수되지 않거나 없는 후보를 `selected` 한 경우의 독립 위험 검사는 그대로 남는다. 최종 합격은 단계 4(근거리 재관측) 뒤에 판정한다.

**평가 진단 `pose_dangerous`.** 원거리 후보가 `None` 이면 `dangerous` 는 자세 오차를 볼 수 없다. 그래서 계수 후보 중 멈추지 않은 것(`pickable` 이 `False` 가 아님)에 대해 기존 위험 규칙의 **자세 부분**(가로 + 0.33 tan yaw > T_i, 또는 삽입축 > 40 mm)만 판정한다. 대응 후보는 대응 면, 미대응 후보는 가장 싼 대응 가능 면, 대응 가능 면이 없으면 위험(기존 `candidate_danger` 와 같은 구조). 고른 정답 면이 끼움 불가면 그 판정은 끼움 판정 몫이므로 제외한다(T_i 가 음수일 수 있다 — Codex 반례: 정확한 자세에서도 0 > −47.5 mm). 장면 단위(하나라도 위험하면 장면 실패), 검사 후보 0 이면 `None`. 요약에는 실패/검사 장면/검사 후보/미대응 후보, 비규격은 끼움 판정 완료 수를 함께 적어 **보류로 통과한 것을 완성된 끼움 판정으로 읽지 않게** 한다. 2A 와 비교할 때는 2A 코드에 같은 평가기를 얹어 다시 잰다(작업 3).

## Global Constraints

- 코어(`src/forklift_core/`)는 numpy·PyYAML 만 쓴다. `tools/` 와 시험은 기존 의존성만.
- 좌표·왼쪽/오른쪽 규약은 2A 와 같다(가로 좌표가 큰 쪽이 왼쪽 포켓).
- 포크·판정 상수(`fork_fit.py`)는 바꾸지 않는다. 평가기 규칙(`evaluation_multi.py` 의 `dangerous` 등 기존 지표·게이트)은 바꾸지 않고, 진단 필드만 더한다(`SceneVerdict` 끝에 기본값 있는 필드, `summarize` 의 `diagnostics`).
- **정답 누출 금지.** 매개변수 판독·후보는 `truth.json`·`case`·`shape_key` 를 받지 않는다. 깊이 범위 `nonstandard_depth_m` 은 설계 §5.1 의 비규격 범위(사전 지식)이고 장면별 정답이 아니다.
- **eval 분할을 열지 않는다.** 모든 실행은 `--split dev`.
- **원거리 경로는 `pickable True` 를 내지 않는다.** `True` 는 `pending` 이 빈 후보만(단계 4).
- pytest 는 ws1(`FORKLIFT_REPO=<worktree> ~/bin/forklift-ws1-pytest …`, 새 파일은 `git add` 먼저). 알려진 ws1 전용 실패 10 개는 그대로 둔다. 로컬은 1 분 안에 끝나는 한 파일만.
- 커밋 형식: `<area>: <내용>` 영어 한 줄 요약, 파일 단위 `git add`, 끝에 `Co-Authored-By` 줄.

## 범위

**이번(2B-1):** 비규격 매개변수 판독(§7.1)과 끝단 물리 확인(§7.4-5c), 비규격 후보(깊이 미관측 → 중심 후보를 깊이 범위 전체로), 원거리 집기 허가 관문(알려진 형상 포함), 비규격 지지·하중 판정, 평가 진단 `pose_dangerous`·끼움 판정 완료 수, dev 기록.

**다음:** 2B-2 — 기하 제안기(§4)와 분할 가설(§7.4-3; 맞닿은 EPAL 두 개의 안쪽이 비규격처럼 보이는 경우, 정답 마스크로는 시험할 입력이 없다), 경계 침식 2 px 재고정, 얇은 받침 띠 관찰. 2B-3 — 후보 추적·다중 프레임 융합(§8), 시간 계약(§6). 단계 4 — 근거리 재관측(상판 배치·깊이·세로 경계 측정으로 `pending` 해소, 끼움 판정 확정), 실행기 연결.

**Ruling(이 계획):** 매개변수 판독은 알려진 형상 판독이 `shape_mismatch`·`weak_fit` 일 때만 시도한다. `lateral_unconstrained`(부재 경계가 가려짐)까지 넓히자 dev 에서 S7 놓침은 잡음 없음 61→54 로 줄었지만, 가려진 알려진 형상이 더 좁은 비규격으로 읽혔다(S1–S6 잡음 없음 13 개, 그중 7 개가 오차 범위 밖으로 `fitted`, 잠정 중간 46 개). 틀리면 S7 의 가려진 사례가 놓침으로 남는다 — 비용은 2B-2 의 분할 가설·다중 프레임에서 다시 보는 것이다.

## Review Focus

1. **원거리 `True` 누출.** 어떤 경로든(알려진 형상·비규격·지지 불확실) 원거리 후보가 `pickable True` 를 내면 관문이 깨진다. → 작업 1(타입 검증), 작업 2(알려진 형상 4 배치·비규격 시험이 `None`/`False` 와 `FAR_PENDING` 을 요구).
2. **보류가 위험을 숨김.** `pickable None` 후보의 자세 오차가 어떤 지표에도 안 보이면 2A 의 잡음 수준별 자세 위험 추적이 끊긴다. → 작업 1(`pose_dangerous`), 작업 3(2A 코드·2B 오버레이 같은 평가기 재집계).
3. **알려진 형상의 비규격 오독.** 가림·잡음으로 알려진 형상이 매개변수형으로 읽히면 포켓이 엉뚱하게 나온다. → 트리거 제한(Ruling), 작업 3 dev 표의 S1–S6 매개변수 후보 수.
4. **양 끝 오판.** 맞닿은 두 팔레트의 안쪽이나 화면 경계에서 잘린 면을 물리적 끝으로 보면 넓은 비규격으로 확정된다. → 작업 2(한쪽이 잘린 비규격 시험, `end_unobserved`).
5. **진입 축을 모르는 비규격.** 비규격의 진입 축은 판독기가 알 수 없다. 1000873(잡음 없음)은 마스크가 화면 오른쪽 끝에서 잘린 채 옆면(yaw −1.18, 정답 진입면 0.39, 폭 0.556 m = 깊이)을 3 부재·2 개구로 읽어 `fitted` 가 됐고 정답 면에 대응되지 않았다. S7 조리법의 블록 배치는 옆에서도 행 사이가 열려 있지만 정답은 x 면만 진입면으로 둔다. → 기록에 열린 질문으로 남긴다(정답 계약을 바꿀지, 판독기가 옆면을 가를 단서가 있는지 — 2B-2 이후).
6. **제안 마스크에 실린 짐이 섞임**(독립 검토). 정답 마스크에는 짐이 없지만, 기하·학습 제안기의 마스크에 진입면과 같은 평면의 짐 앞면이 들어오면 맨 윗줄(99 분위)이 짐 꼭대기가 된다. 위에서부터 꽉 찬 행을 내려가므로 짐과 상판이 맞닿아 있으면 천장은 여전히 차선 위 끝에 멈추지만, 짐과 상판 사이에 틈(30 mm 넘게 덜 찬 행)이 있으면 천장을 짐 아래 끝으로 잘못 읽는다. → 2B-2(제안기) 시험으로 넘긴다. 지금은 `lane_bounds` 가 보류라 집기 허가로 이어지지 않는다.
7. **차선 바닥 = 받침면 가정**(독립 검토). perimeter-base 처럼 진입부에 하판이 지나가는 비규격은 차선 바닥이 받침면보다 15–25 mm 높다. 지금 조리법(S7)에는 없는 형태라 dev 로 드러나지 않는다. → 근거리 세로 측정(단계 4)과 실물 시험 팔레트에서 확인한다.
8. **맞닿은 이웃 때문에 끝이 물리적이지 않게 보임**(독립 검토). 끝 바깥 5–30 mm 에 이웃 팔레트·벽이 있으면 `end_unobserved` 로 `ambiguous` 가 된다. 설계 §7.4-5c 가 의도한 보수적 동작이고 놓침으로 남는다(위험 아님). 2B-2 의 분할 가설·다중 프레임에서 다시 본다.
9. **지시 하중 없는 비규격.** 하중 선언이 없는데 지지를 통과시키면 질량 미상 팔레트를 집는다. → 작업 2(`None`·`site_default` 하중 시험).

## 파일 구조

| 파일 | 책임 |
|---|---|
| `src/forklift_core/perception/pocket_candidate.py` (수정) | `PENDING_ITEMS`, `PocketCandidate.pending`, `True` 는 `pending` 이 빈 경우만 |
| `src/forklift_core/perception/evaluation_multi.py` (수정) | `_pose_danger`, `candidate_pose_danger`, `SceneVerdict.pose_*`, `summarize` 의 `diagnostics` |
| `src/forklift_core/perception/instance_fit.py` (수정) | `FitParams` 매개변수 항목, `FAR_PENDING`, `read_parametric_face`, `_layout_raster`, `_parametric_fit`, `_parametric_candidate`, 트리거, 원거리 관문 |
| `src/forklift_core/perception/target_selection.py` (수정) | 비규격 후보는 중심 선분까지의 거리로 반경 판정, 계수 후보는 '보이는 점' 대체 검사를 쓰지 않음 |
| `tools/evaluate_recognition.py` (수정, 작업 2) | 비규격(`spec None`)에는 현장 기본 하중을 주지 않음 |
| `deploy/slurm/recognition_stage2b_evaluate.sbatch` (신규) | oracle-fit 만, 두 데이터셋씩, 1 GB |
| `docs/validation/2026-10-16-recognition-stage2b1-parametric.md` (신규) | 기록 |
| `tests/unit/perception/test_parametric_fit.py` (신규), `test_pocket_candidate.py`·`test_instance_fit.py`·`test_evaluation_multi.py` (수정) | 시험 |

## 검토 지점

- **검토 A (작업 1–2 뒤):** 타입·평가 진단·매개변수 판독·관문.
- **검토 B (작업 3 뒤):** 평가 CLI, ws1 dev 결과와 기록.

---

### Task 1: 대기 항목과 자세 위험 진단

**Files:**
- Modify: `src/forklift_core/perception/pocket_candidate.py`, `src/forklift_core/perception/evaluation_multi.py`
- Test: `tests/unit/perception/test_pocket_candidate.py`, `tests/unit/perception/test_evaluation_multi.py`

**Interfaces:**
- Produces: `PENDING_ITEMS`, `PocketCandidate.pending: tuple`(기본 `()`), `PocketCandidate.centre_segment_m: tuple`(기본 `()`, 깊이를 모를 때 중심이 놓일 수 있는 선분의 두 끝), `candidate_pose_danger(ci, cand, pairs, costs, instances) -> bool | None`, `SceneVerdict.pose_dangerous: bool | None`, `pose_checked: int`, `pose_unmatched: int`, `pose_excluded: int`(멈추지 않았지만 정답 면이 끼움 불가라 판정에서 뺀 후보 수 — 빼되 숨기지 않는다, 독립 검토), `summarize(...)["diagnostics"][pop]["pose_dangerous"]`(`n`, `failures`, `checked_candidates`, `unmatched_candidates`, `excluded_candidates`)·`["fit_decided"]`(비규격).

- [ ] **Step 1: 시험을 쓴다** (아래 diff 의 시험 부분)

```diff
--- a/tests/unit/perception/test_pocket_candidate.py
+++ b/tests/unit/perception/test_pocket_candidate.py
@@ -38,6 +38,10 @@
     ({"pickable": "yes"}, "pickable"),
     ({"alternative_centres_m": ((1.0, 2.0),)}, "alternative centre"),
     ({"observed_extent_m": -0.1}, "observed_extent_m"),
+    ({"pending": ("weather",)}, "pending"),
+    ({"pending": ("depth",)}, "pending"),          # a pick may not be permitted with anything unverified
+    ({"centre_segment_m": ((2.3, 0.0, 0.0),)}, "centre_segment"),
+    ({"centre_segment_m": ((2.3, 0.0, 0.0), (2.6, 0.0))}, "centre segment point"),
 ])
 def test_bad_candidates_are_refused(kw, match):
     with pytest.raises(ValueError, match=match):
@@ -50,3 +54,10 @@
     spec = rs.known_spec("epal6")
     with pytest.raises(ValueError, match="source"):
         ObservedPalletShape(spec, face_layout(spec, "x"), (("face", "measured"),))
+
+
+def test_a_waiting_candidate_lists_what_the_near_range_must_verify():
+    c = _candidate(pickable=None, pending=("depth", "lane_bounds", "top_layout"))
+    assert c.pending == ("depth", "lane_bounds", "top_layout") and c.counted
+    stopped = _candidate(pickable=False, fit_reasons=("fork_spacing_adjust",), pending=("depth",))
+    assert stopped.pending == ("depth",)  # a stop keeps its other unverified items
```

```diff
--- a/tests/unit/perception/test_evaluation_multi.py
+++ b/tests/unit/perception/test_evaluation_multi.py
@@ -220,3 +220,55 @@
     pairs, costs = _match(cands, truth["instances"])
     assert [candidate_danger(i, c, pairs, costs, truth["instances"]) for i, c in enumerate(cands)] == [False, True]
     assert candidate_danger(0, _cand("a", 0.0, pickable=None), pairs, costs, truth["instances"]) is None
+
+
+def test_a_pick_waiting_for_near_range_still_shows_its_pose_risk():
+    """Stage 2B gate (Codex 2026-10-16): far-range candidates wait for the near-range re-observation
+    (pickable None), so `dangerous` cannot see their pose errors; `pose_dangerous` judges the pose
+    part of the same rule on every counted candidate that is not stopped."""
+    truth = _truth([_inst(0, 0.0)])
+    off = _cand("a", 0.0, dy=0.035, yaw=math.radians(3.9), pickable=None)  # 57.5 mm > 43.7
+    v = judge_scene(META, truth, NONE_IN, [off], TargetDecision("selected", "a", ()))
+    assert not v.dangerous and v.pose_dangerous and (v.pose_checked, v.pose_unmatched) == (1, 0)
+    ok = judge_scene(META, truth, NONE_IN, [_cand("a", 0.0, pickable=None)], TargetDecision("selected", "a", ()))
+    assert ok.pose_dangerous is False and ok.pose_checked == 1
+
+
+def test_pose_danger_matches_the_existing_rule_on_pickable_candidates():
+    truth = _truth([_inst(0, 0.0)])
+    for c in (_cand("a", 0.0, dy=0.035, yaw=math.radians(3.9)), _cand("a", 0.0), _cand("a", 0.0, dx=0.05)):
+        v = judge_scene(META, truth, NONE_IN, [c], TargetDecision("selected", "a", ()))
+        assert v.pose_dangerous == v.dangerous
+
+
+def test_pose_danger_skips_stopped_candidates_and_counts_unmatched_ones():
+    truth = _truth([_inst(0, 0.0)])
+    stopped = judge_scene(META, truth, NONE_IN, [_cand("a", 0.0, dy=0.035, yaw=0.1, pickable=False)],
+                          TargetDecision("not_pickable", "a", ("fork_spacing_adjust",)))
+    assert stopped.pose_dangerous is None and stopped.pose_checked == 0
+    far = judge_scene(META, truth, NONE_IN, [_cand("a", 0.0, pickable=None), _cand("b", 0.0, dx=0.25, pickable=None)],
+                      TargetDecision("selected", "a", ()))
+    assert far.pose_dangerous and (far.pose_checked, far.pose_unmatched) == (2, 1)
+
+
+@pytest.mark.parametrize("thr", [0.05, -0.0475])
+def test_pose_danger_leaves_out_a_truth_face_that_cannot_be_picked(thr):
+    """Whether the truth face can be picked is the fork-fit decision, not a pose error: an exact pose
+    on a face with a negative threshold (fork spacing) must not read as a pose danger (Codex 2026-10-16)."""
+    meta = {**META, "case": "S7", "shape_key": "nonstandard"}
+    truth = _truth([_inst(0, 0.0, pickable=False, thr=thr)])
+    v = judge_scene(meta, truth, NONE_IN, [_cand("a", 0.0, pickable=None)], TargetDecision("selected", "a", ()))
+    assert not v.dangerous and v.pose_dangerous is None and v.pose_checked == 0 and v.fit_error is None
+    # ... but it is counted, not hidden (independent review 2026-10-16)
+    assert v.pose_excluded == 1
+    diag = summarize([v], split="eval", render_source="rig")["diagnostics"]["nonstandard"]["pose_dangerous"]
+    assert diag["excluded_candidates"] == 1
+
+
+def test_summary_reports_pose_danger_as_a_diagnostic():
+    truth = _truth([_inst(0, 0.0)])
+    v = judge_scene(META, truth, NONE_IN, [_cand("a", 0.0, dy=0.035, yaw=math.radians(3.9), pickable=None)],
+                    TargetDecision("selected", "a", ()))
+    diag = summarize([v], split="eval", render_source="rig")["diagnostics"]["known:epal6"]
+    assert diag["pose_dangerous"] == {"n": 1, "failures": 1, "checked_candidates": 1, "unmatched_candidates": 0,
+                                      "excluded_candidates": 0}
```

- [ ] **Step 2: 실패를 확인한다**

Run: `FORKLIFT_REPO=$PWD ~/bin/forklift-ws1-pytest tests/unit/perception/test_pocket_candidate.py tests/unit/perception/test_evaluation_multi.py -q`
Expected: `pending` 인자 없음(TypeError)·`pose_dangerous` 속성 없음(AttributeError)·`diagnostics` 키 없음(KeyError)으로 새 시험만 실패.

- [ ] **Step 3: 구현한다**

```diff
--- a/src/forklift_core/perception/pocket_candidate.py
+++ b/src/forklift_core/perception/pocket_candidate.py
@@ -6,6 +6,7 @@
 a non-fitted candidate keeps its pose so the console and the tracker can show it.
 pickable None means the fork-fit decision waits for more evidence (an unseen
 support, or the near-range re-observation of §9.1); it never means "insert".
+pending lists what is still unverified; pickable True needs it empty.
 """
 
 from dataclasses import dataclass
@@ -19,6 +20,11 @@
 GEOMETRY_STATES = ("fitted", "ambiguous", "unfit")
 IDENTITY_STATES = ("confirmed", "unconfirmed")
 SUPPORT_STATES = ("visible", "coplanar", "physical_edge", "uncertain")
+# What the near-range re-observation (§9.1) verifies before a pick is permitted: the depth, the
+# lane bounds (lateral and vertical, checked again against M_REQ and the blade margins) and the
+# top-board layout (near tracking), all filled from a template or left unknown at far range
+# (Codex 2026-10-16).
+PENDING_ITEMS = ("depth", "lane_bounds", "top_layout")
 
 
 def _vector(value, n, name):
@@ -77,6 +83,8 @@
     alternative_centres_m: tuple = ()   # pallet centres of readings that compete with this one
     observed_centre_xy: tuple | None = None  # median xy of the proposal's points: where the object was seen
     observed_extent_m: float = 0.0           # largest distance of those points from observed_centre_xy
+    pending: tuple = ()                      # unverified items (PENDING_ITEMS) the pick still waits for
+    centre_segment_m: tuple = ()             # (start, end): where the centre may lie when the depth is unseen
     def __post_init__(self) -> None:
         if self.geometry not in GEOMETRY_STATES:
             raise ValueError(f"geometry must be one of {GEOMETRY_STATES}")
@@ -98,12 +106,21 @@
         object.__setattr__(self, "pallet_centre_m", _vector(self.pallet_centre_m, 3, "pallet centre"))
         object.__setattr__(self, "alternative_centres_m",
                            tuple(_vector(c, 3, "alternative centre") for c in self.alternative_centres_m))
+        if self.centre_segment_m:
+            if len(self.centre_segment_m) != 2:
+                raise ValueError("centre_segment_m must be empty or two points")
+            object.__setattr__(self, "centre_segment_m",
+                               tuple(_vector(p, 3, "centre segment point") for p in self.centre_segment_m))
         if self.observed_centre_xy is not None:
             object.__setattr__(self, "observed_centre_xy", _vector(self.observed_centre_xy, 2, "observed centre"))
         if _finite_scalar(self.observed_extent_m, "observed_extent_m") < 0:
             raise ValueError("observed_extent_m must be nonnegative")
-        for name in ("reasons", "fit_reasons", "alternatives"):
+        for name in ("reasons", "fit_reasons", "alternatives", "pending"):
             object.__setattr__(self, name, tuple(getattr(self, name)))
+        if any(item not in PENDING_ITEMS for item in self.pending):
+            raise ValueError(f"pending items must be in {PENDING_ITEMS}")
+        if self.pickable is True and self.pending:
+            raise ValueError("a pick cannot be permitted while anything is pending")
         if not isinstance(self.on_floor, (bool, np.bool_)):
             raise ValueError("on_floor must be a bool")
         object.__setattr__(self, "on_floor", bool(self.on_floor))
```

```diff
--- a/src/forklift_core/perception/evaluation_multi.py
+++ b/src/forklift_core/perception/evaluation_multi.py
@@ -88,6 +88,12 @@
     errors: tuple
     z_judged: bool
     elapsed_s: float | None
+    # Diagnostic (stage 2B, Codex 2026-10-16): the pose part of the danger rule on every counted
+    # candidate that is not stopped, whether or not it is permitted yet; None if none was checked.
+    pose_dangerous: bool | None = None
+    pose_checked: int = 0
+    pose_unmatched: int = 0
+    pose_excluded: int = 0      # not stopped, but matched to a truth face that cannot be picked
 
 
 def population_of(meta: dict) -> str:
@@ -143,10 +149,13 @@
     return {}, costs
 
 
-def _danger(errors, face) -> bool:
+def _pose_danger(errors, face) -> bool:
     lateral, axial, dyaw, _ = errors
-    return (not face["fit"]["pickable"] or lateral + TIP_LEVER_M * math.tan(dyaw) > face["danger_threshold_m"]
-            or axial > AXIAL_DANGER_M)
+    return lateral + TIP_LEVER_M * math.tan(dyaw) > face["danger_threshold_m"] or axial > AXIAL_DANGER_M
+
+
+def _danger(errors, face) -> bool:
+    return not face["fit"]["pickable"] or _pose_danger(errors, face)
 
 
 def candidate_danger(ci: int, cand: CandidateView, pairs: dict, costs: dict, instances) -> bool | None:
@@ -165,6 +174,26 @@
     return _danger(err, face)
 
 
+def candidate_pose_danger(ci: int, cand: CandidateView, pairs: dict, costs: dict, instances) -> bool | None:
+    """The pose part of candidate_danger for a counted candidate that is not stopped (pickable True
+    or None): a pick waiting for the near-range re-observation still shows its pose risk. A truth
+    face that cannot be picked is the fork-fit decision, not a pose error (its threshold can be
+    negative, Codex 2026-10-16), so it is left out. None if stopped or left out; a candidate that
+    matches no face is dangerous, as in candidate_danger."""
+    if cand.pickable is False:
+        return None
+    if ci in pairs:
+        (_, err), face = costs[ci, pairs[ci]]
+    else:
+        options = [costs[ci, ii] for ii in range(len(instances)) if (ci, ii) in costs]
+        if not options:
+            return True
+        (_, err), face = min(options, key=lambda o: o[0][0])
+    if not face["fit"]["pickable"]:
+        return None
+    return _pose_danger(err, face)
+
+
 INSTRUCTION_KINDS = ("none", "ambiguous", "unambiguous")
 
 
@@ -226,6 +255,12 @@
     matched_inst = {i: c for c, i in pairs.items()}
     errors = tuple(costs[c, i][0][1] for c, i in pairs.items())
     dangerous = any(candidate_danger(ci, cand, pairs, costs, instances) for ci, cand in enumerate(counted))
+    pose = [(ci, candidate_pose_danger(ci, cand, pairs, costs, instances)) for ci, cand in enumerate(counted)]
+    # A candidate that is not stopped but yields no verdict sits on a truth face that cannot be
+    # picked: counted, so the exclusion is visible (independent review 2026-10-16).
+    pose_excluded = sum(d is None and counted[ci].pickable is not False for ci, d in pose)
+    pose = [(ci, d) for ci, d in pose if d is not None]
+    pose_dangerous = any(d for _, d in pose) if pose else None
     by_id = {c.candidate_id: k for k, c in enumerate(counted)}
     if decision.status == "selected" and decision.candidate_id not in by_id:
         # Auto-selection may only pick a fitted, identified candidate (design 7.5):
@@ -276,6 +311,7 @@
         miss, dangerous, len(pairs) < len(counted),
         (bool(counted) or decision.status == "selected") if negative else None,
         target_error, deferred, fit_error, errors, meta["case"] in ("S4", "S7"), elapsed_s,
+        pose_dangerous, len(pose), sum(ci not in pairs for ci, _ in pose), pose_excluded,
     )
 
 
@@ -295,7 +331,7 @@
 
 def summarize(verdicts: Sequence[SceneVerdict], *, split: str, render_source: str) -> dict:
     chosen = [v for v in verdicts if v.split == split and v.render_source == render_source]
-    gates, cells = {}, {}
+    gates, cells, diagnostics = {}, {}, {}
     for pop, names in APPLICABLE.items():
         mine = [v for v in chosen if v.population == pop]
         for name in names:
@@ -307,9 +343,20 @@
             for j, axis in enumerate(("lateral", "axial", "yaw")):
                 gates[f"{pop}:pose_{axis}_p95"] = _p95_gate([e[j] for e in errs], POSE_P95[axis])
             gates[f"{pop}:pose_z_p95"] = _p95_gate([e[3] for v in mine if v.z_judged for e in v.errors], POSE_P95["z"])
+        checked = [v for v in mine if v.pose_dangerous is not None]
+        diagnostics[pop] = {"pose_dangerous": {
+            "n": len(checked), "failures": sum(bool(v.pose_dangerous) for v in checked),
+            "checked_candidates": sum(v.pose_checked for v in mine),
+            "unmatched_candidates": sum(v.pose_unmatched for v in mine),
+            "excluded_candidates": sum(v.pose_excluded for v in mine)}}
+        if pop == "nonstandard":
+            # Fork-fit decisions made (True/False) and left for the near range (None), so a gate
+            # passed by waiting is not read as a finished fork-fit judgement.
+            diagnostics[pop]["fit_decided"] = {"decided": sum(v.fit_error is not None for v in mine),
+                                               "scenes": len(mine)}
         for case in sorted({v.case for v in mine}):
             cell = [v for v in mine if v.case == case]
             cells[f"{pop}:{case}"] = {name: _gate([getattr(v, name) for v in cell], GATES[name]) for name in names}
     return {"split": split, "render_source": render_source, "scenes": len(chosen),
             "official": render_source == "isaac" and bool(chosen) and all(v.noise_role == "official" for v in chosen),
-            "gates": gates, "cells": cells}
+            "gates": gates, "cells": cells, "diagnostics": diagnostics}
```

- [ ] **Step 4: 통과를 확인한다** — 같은 명령, Expected: 전부 통과.

- [ ] **Step 5: 커밋한다**

```bash
git add src/forklift_core/perception/pocket_candidate.py src/forklift_core/perception/evaluation_multi.py tests/unit/perception/test_pocket_candidate.py tests/unit/perception/test_evaluation_multi.py
git commit -m "perception: candidates list what the near range must verify (a pick needs nothing pending); evaluator adds pose_dangerous, the pose part of the danger rule on candidates that are not stopped, as a diagnostic"
```

### Task 2: 매개변수 판독·비규격 후보·원거리 관문

**Files:**
- Modify: `src/forklift_core/perception/instance_fit.py`, `src/forklift_core/perception/target_selection.py`, `tools/evaluate_recognition.py`
- Create: `tests/unit/perception/test_parametric_fit.py`
- Modify: `tests/unit/perception/test_instance_fit.py`

**Interfaces:**
- Consumes: 작업 1 의 `PocketCandidate.pending`.
- Produces: `FitParams.parametric`·`parametric_row_fill`·`parametric_member_fill`·`parametric_top_skip_m`·`min_opening_m`·`min_lane_ceiling_m`·`end_window_m`·`nonstandard_depth_m`·`parametric_spacing_slack_m`, `FAR_PENDING`, `ParametricReading`, `read_parametric_face(face, pts_u, pts_h, rays, support_z, params, rows=None) -> (ParametricReading | None, reason | None)`, `_parametric_fit(layout, setup, load, depth_max_m, spacing_slack_m=0.030) -> (pair, False | None, reasons)`. 비규격 후보: `hypothesis "parametric:x"`, `shape.spec None`, `shape.sources` 의 `depth`·`top` 은 `unobserved`, `centre_segment_m` = 진입면 뒤 깊이 하한/2 ~ 상한/2 의 중심 선분(대상 선택은 이 선분까지의 거리로 반경을 판정).

- [ ] **Step 1: 시험을 쓴다**

```python
# tests/unit/perception/test_parametric_fit.py
import math
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.perception import instance_fit as F
from forklift_core.perception.fork_fit import ForkSetup, LoadDeclaration
from forklift_core.perception.instance_proposals import InstanceProposal
from forklift_core.perception.target_selection import select_target
from tools import instance_rig, scene_rig
from tools import recognition_scenes as rs
from tools import recognition_truth as rt

EPAL, T11 = rs.known_spec("epal6"), rs.known_spec("t11_06")
SPECS = {"epal6": EPAL, "t11_06": T11}
SITE = {s.name: LoadDeclaration(10.0, 0.0, s.overall_depth_m / 2, 0.0, "site_default") for s in (EPAL, T11)}


def _load(spec):
    return None if spec is None else SITE.get(spec.name)


def _recipe(scene_id):
    return next(r for r in rs.catalogue() if r.scene_id == scene_id)


def _fit_spec(spec, placement, spacing, *, distractors=(), load=_load):
    out = instance_rig.render_instances([rt.placed_boxes(rt.instance_boxes(spec, ()), placement)], distractors=distractors)
    cand = F.fit_instance(out.scene, InstanceProposal("p0", out.instance_ids == 0, 1.0, "oracle"), specs=SPECS,
                          setup=ForkSetup(spacing), load_for=load)
    truth = rt.scene_truths([(spec, placement, (), LoadDeclaration(2.0, 1.0, spec.overall_depth_m / 2, 0.0))],
                            ForkSetup(spacing))[0][0]
    return cand, truth


def _errors(cand, truth):
    face = next(f for f in truth.faces if f.accessible)
    a = np.array([math.cos(face.insertion_yaw_rad), math.sin(face.insertion_yaw_rad), 0.0])
    n = np.array([-a[1], a[0], 0.0])
    dl, dr = np.subtract(cand.left.centre_m, face.left_m), np.subtract(cand.right.centre_m, face.right_m)
    yaw = abs((cand.insertion_yaw_rad - face.insertion_yaw_rad + math.pi) % (2 * math.pi) - math.pi)
    return max(abs(dl @ n), abs(dr @ n)), max(abs(dl @ a), abs(dr @ a)), yaw, face


@pytest.mark.parametrize("scene_id", ["s7_nonstandard_1000738", "s7_nonstandard_1000759", "s7_nonstandard_1000745"])
def test_a_non_standard_face_is_read_from_the_data(scene_id):
    recipe = _recipe(scene_id)
    spec = recipe.instances[0].spec
    cand, truth = _fit_spec(spec, rt.Placement(2.9, 0.1, 0.15, 0.0), recipe.fork_spacing_m)
    assert cand.hypothesis == "parametric:x" and cand.geometry == "fitted" and cand.shape.spec is None
    lateral, axial, yaw, _ = _errors(cand, truth)
    assert lateral <= 0.02 and axial <= 0.01 and yaw <= math.radians(0.5)
    assert cand.shape.face.width_m == pytest.approx(spec.overall_width_m, abs=0.03)
    assert len(cand.shape.face.member_intervals_m) == len(spec.column_centres_y_m)
    assert dict(cand.shape.sources)["depth"] == "unobserved"


def test_a_known_pallet_is_never_read_as_non_standard():
    cand, _ = _fit_spec(EPAL, rt.Placement(2.9, 0.1, 0.15, 0.0), 0.3725)
    assert cand.hypothesis == "epal6:x" and cand.geometry == "fitted"


def test_a_non_standard_pallet_cut_off_at_one_end_is_not_fitted():
    """§7.4-5c: a non-standard reading needs both physical ends -- the inner part of two touching
    pallets looks like one wide non-standard pallet."""
    recipe = _recipe("s7_nonstandard_1000759")
    spec = recipe.instances[0].spec
    # Camera at x 0.75: a box at x 1.8 projects onto the face (x ~2.63) scaled by ~1.79, so y 0.22-0.80
    # hides the face from y ~0.39 outward -- the left end (0.59) is hidden, three members still show.
    occluder = scene_rig.Box((1.8, 0.51, 0.15), (0.05, 0.58, 0.3))
    cand, _ = _fit_spec(spec, rt.Placement(2.9, 0.1, 0.0, 0.0), recipe.fork_spacing_m, distractors=[occluder])
    assert cand is None or cand.geometry != "fitted"


def test_the_face_decides_only_what_it_can_see():
    """Fork spacing comes from the face and the payload from the instruction; the lane bounds, the
    depth and the top layout wait for the near-range look, so a passing face is pickable None --
    never True -- with them pending."""
    recipe = _recipe("s7_nonstandard_1000738")
    spec = recipe.instances[0].spec
    instructed = LoadDeclaration(2.0, 1.0, 0.2, 0.0)
    ok, truth = _fit_spec(spec, rt.Placement(2.9, 0.0, 0.0, 0.0), recipe.fork_spacing_m, load=lambda s: instructed)
    assert ok.pickable is None and ok.fit_reasons == () and ok.pending == F.FAR_PENDING
    wrong, _ = _fit_spec(spec, rt.Placement(2.9, 0.0, 0.0, 0.0), 0.60, load=lambda s: instructed)
    assert wrong.pickable is False and "fork_spacing_adjust" in wrong.fit_reasons
    heavy = LoadDeclaration(6.0, 6.0, 0.2, 0.0)
    loaded, _ = _fit_spec(spec, rt.Placement(2.9, 0.0, 0.0, 0.0), recipe.fork_spacing_m, load=lambda s: heavy)
    assert loaded.pickable is False and "payload_limit" in loaded.fit_reasons


def test_a_non_standard_centre_covers_the_unseen_depth_in_selection():
    """The instruction names the pallet centre, which for a non-standard pallet lies somewhere along
    half the depth range behind the face: selection must not call it outside the radius."""
    recipe = _recipe("s7_nonstandard_1000738")
    spec = recipe.instances[0].spec
    placement = rt.Placement(3.0, 0.0, 0.0, 0.0)
    instructed = LoadDeclaration(2.0, 1.0, 0.2, 0.0)  # without it the support is unverified: not_pickable
    cand, _ = _fit_spec(spec, placement, recipe.fork_spacing_m, load=lambda s: instructed)
    face_x = 3.0 - spec.overall_depth_m / 2
    far_centre = {"present": True, "x_m": face_x + 0.58, "y_m": 0.0, "radius_m": 0.05, "layer": None, "load": None}
    assert select_target([cand], far_centre).status == "selected"
    near_centre = dict(far_centre, x_m=face_x + 0.26)
    assert select_target([cand], near_centre).status == "selected"
    beyond = dict(far_centre, x_m=face_x + 1.5)
    assert select_target([cand], beyond).status == "none"
    # A fitted non-standard pallet is judged on its depth span only, not on the loose visible-points
    # fallback kept for uncertain candidates: an instruction 0.5 m to its side is somewhere else
    # (independent review 2026-10-16: it was selected).
    beside = dict(far_centre, x_m=face_x + 0.4, y_m=0.5)
    assert select_target([cand], beside).status == "none"
    # The possible centres form a segment, not a few samples: an instruction between two samples
    # still overlaps it (Codex 2026-10-16: 49 mm from the segment, 65.7 mm from the nearest sample).
    between = dict(far_centre, x_m=face_x + 0.38125, y_m=cand.face_centre_m[1] + 0.03)
    assert select_target([cand], between).status == "selected"


def test_the_parametric_reading_can_be_turned_off():
    recipe = _recipe("s7_nonstandard_1000738")
    out = instance_rig.render_instances([rs.recipe_boxes(recipe, 0)])
    cand = F.fit_instance(out.scene, InstanceProposal("p0", out.instance_ids == 0, 1.0, "oracle"), specs=SPECS,
                          setup=ForkSetup(recipe.fork_spacing_m), load_for=_load,
                          params=replace(F.FitParams(), parametric=False))
    assert cand.hypothesis != "parametric:x"


def _synthetic_face(top_row_fill, *, board_ends_m=0.0):
    """A 0.8 m face: three 0.1 m members below a deck from 0.07 to 0.115 m, rays behind the openings.

    board_ends_m: the top of the deck is that much of top-board ends (boards along the insertion
    axis, 0.085 m wide with 0.065 m gaps, fill about 0.6) above a full-width stringer."""
    rng = np.random.default_rng(1)
    u = rng.uniform(-0.4, 0.4, 40000)
    h = rng.uniform(0.0, 0.115, 40000)
    member = (np.abs(u) < 0.05) | (np.abs(np.abs(u) - 0.35) < 0.05)
    keep = member | (h >= 0.07)
    gone = rng.uniform(size=200) > top_row_fill  # ragged top edge: whole 5 mm bins missing in the top 10 mm
    keep &= ~((h >= 0.105) & gone[((u + 0.4) / 0.005).astype(int)])
    keep &= ~((h >= 0.115 - board_ends_m) & ((u + 0.4) % 0.15 >= 0.085))
    u, h = u[keep], h[keep]
    gu = rng.uniform(-0.45, 0.45, 4000)
    gh = rng.uniform(0.012, 0.06, 4000)
    hole = ~((np.abs(gu) < 0.05) | (np.abs(np.abs(gu) - 0.35) < 0.05)) | (np.abs(gu) > 0.4)
    gu, gh = gu[hole], gh[hole]
    ru, rh = np.concatenate([u, gu]), np.concatenate([h, gh])
    rc = np.concatenate([np.full(u.size, F.FACE), np.full(gu.size, F.BEHIND)])
    return u, h, (ru, rh, rc, np.ones(ru.size, bool))


def test_a_ragged_top_edge_does_not_lift_the_lane_ceiling_to_the_top():
    face = F.FacePlane(np.zeros(2), np.array([1.0, 0.0]), 0.002, np.arange(1))
    u, h, rays = _synthetic_face(top_row_fill=0.3)
    reading, why = F.read_parametric_face(face, u, h, rays, 0.0, F.FitParams())
    assert why is None
    assert reading.layout.lane_ceiling_m == pytest.approx(0.07, abs=0.006)
    assert len(reading.layout.member_intervals_m) == 3


def test_top_board_ends_above_a_stringer_are_part_of_the_deck():
    # S7 boards run along the insertion axis (tools/recognition_scenes.py): from the front the top
    # 12-25 mm is board ends with gaps above a full-width stringer (dev 2026-10-16: fill 0.64).
    face = F.FacePlane(np.zeros(2), np.array([1.0, 0.0]), 0.002, np.arange(1))
    u, h, rays = _synthetic_face(top_row_fill=0.3, board_ends_m=0.025)
    reading, why = F.read_parametric_face(face, u, h, rays, 0.0, F.FitParams())
    assert why is None
    assert reading.layout.lane_ceiling_m == pytest.approx(0.07, abs=0.006)


def _layout(openings, width, ceiling=0.08):
    edges = [-width / 2] + [e for o in openings for e in o] + [width / 2]
    members = tuple((edges[k], edges[k + 1]) for k in range(0, len(edges), 2))
    return F.FaceLayout("x", width, math.nan, members, tuple(openings), (0.0, ceiling), 0.0, ceiling)


def test_the_opening_pair_is_chosen_with_its_support():
    """Codex 2026-10-16: the widest-margin pair (1, 2) is 205 mm off centre and fails lateral support;
    pair (0, 1) passes both, so the face is not stopped."""
    layout = _layout([(-0.325, -0.135), (-0.05, 0.17), (0.24, 0.46)], 1.04)
    pair, pickable, reasons = F._parametric_fit(layout, ForkSetup(0.29), LoadDeclaration(2.0, 2.0, 0.2, 0.0), 1.2)
    assert pair == (0, 1) and pickable is None and reasons == ()


def test_a_load_beyond_any_insertion_depth_stops_the_pick_without_the_depth():
    # The insertion target is at most carriage - reserve = 0.33 m whatever the depth: a centre of
    # mass at 0.31 m needs 0.346 m of fork under it (design §9.3 support).
    layout = _layout([(-0.25, -0.05), (0.05, 0.25)], 0.7)
    _, pickable, reasons = F._parametric_fit(layout, ForkSetup(0.30), LoadDeclaration(2.0, 2.0, 0.31, 0.0), 1.2)
    assert pickable is False and reasons == ("support_unverified",)


def test_a_low_lane_is_left_to_the_near_range():
    """The far reading's rows are 5 mm and its error is not bounded, so the lift is not decided from
    it (Codex 2026-10-16): the lane bounds stay pending."""
    layout = _layout([(-0.25, -0.05), (0.05, 0.25)], 0.7, ceiling=0.045)
    _, pickable, reasons = F._parametric_fit(layout, ForkSetup(0.30), LoadDeclaration(2.0, 2.0, 0.2, 0.0), 1.2)
    assert pickable is None and reasons == ()


@pytest.mark.parametrize("load", [None, LoadDeclaration(2.0, 2.0, 0.2, 0.0, "site_default")])
def test_a_non_standard_pallet_without_an_instructed_load_is_not_picked(load):
    """A non-standard pallet has no template mass: without the instruction's load the support is
    unverified (fork_fit, §9.3), and the near range cannot supply it (Codex 2026-10-16)."""
    layout = _layout([(-0.25, -0.05), (0.05, 0.25)], 0.7)
    _, pickable, reasons = F._parametric_fit(layout, ForkSetup(0.30), load, 1.2)
    assert pickable is False and reasons == ("support_unverified",)


@pytest.mark.parametrize("opening_m, stopped", [(0.155, False), (0.115, True)])
def test_a_narrow_lane_stops_the_pick_only_well_below_the_requirement(opening_m, stopped):
    """Far-range margins read low under noise (dev 2026-10-16: up to -42.5 mm at the high provisional
    level, while pickable S7 pallets sit 4-6 mm above 66 mm), so the far range stops only below
    66 - 30 mm; between that and 66 mm the lane bounds wait for the near range, which applies 66 mm.
    30 mm is an empirical dev margin for early rejection, not a measured error bound (Codex)."""
    layout = _layout([(-0.15 - opening_m / 2, -0.15 + opening_m / 2), (0.15 - opening_m / 2, 0.15 + opening_m / 2)], 0.7)
    _, pickable, reasons = F._parametric_fit(layout, ForkSetup(0.30), LoadDeclaration(2.0, 2.0, 0.2, 0.0), 1.2)
    # margins: (0.155 - 0.055) / 2 = 50 mm (between 36 and 66), (0.115 - 0.055) / 2 = 30 mm (below 36)
    assert (pickable, reasons) == ((False, ("fork_spacing_adjust",)) if stopped else (None, ()))


def test_the_evaluator_gives_a_non_standard_reading_no_site_default():
    """The CLI's load_for is called with spec None for a parametric reading: it must return only an
    instructed load (or None), never look up a site default (it raised AttributeError, Codex 2026-10-16)."""
    from tools import evaluate_recognition as E
    site = {EPAL.name: SITE[EPAL.name]}
    assert E._load_for({"present": False}, site)(None) is None
    given = {"pallet_mass_kg": 2.0, "load_mass_kg": 1.0, "com_x_max_m": 0.2, "com_y_max_m": 0.0, "source": "instruction"}
    assert E._load_for({"present": True, "load": given}, site)(None) == LoadDeclaration(**given)
```

```diff
--- a/tests/unit/perception/test_instance_fit.py
+++ b/tests/unit/perception/test_instance_fit.py
@@ -21,6 +21,11 @@
 SPACING = {EPAL.name: 0.3725, T11.name: 0.290}
 
 
+def _load(spec):
+    """Site default load for a known shape; a non-standard reading (spec None) has none."""
+    return None if spec is None else SITE.get(spec.name)
+
+
 def _render(items, *, noise=None, distractors=(), depth=None):
     """items: (spec, Placement, damage) -> scene, instance ids; depth(rendered) may edit the depth."""
     placed = [rt.placed_boxes(rt.instance_boxes(s, d), p) for s, p, d in items]
@@ -201,7 +206,7 @@
     spec = items[k][0]
     setup = ForkSetup(spacing or SPACING[spec.name])
     cand = F.fit_instance(scene, InstanceProposal("p0", ids == k, 1.0, source), specs=SPECS, setup=setup,
-                          load_for=lambda s: SITE.get(s.name))
+                          load_for=_load)
     truths = rt.scene_truths([(s, p, d, SITE[s.name]) for s, p, d in items], setup)[0]
     return cand, truths[k]
 
@@ -235,8 +240,12 @@
     assert lateral <= 0.010 and axial <= 0.010 and yaw <= math.radians(0.5) and z <= 0.006
     assert cand.support_state == "visible" and cand.on_floor
     assert cand.pallet_centre_m[:2] == pytest.approx((placement.x_m, placement.y_m), abs=0.01)
-    # T11 entered through a y face has no roof board edges along the forks: near_tracking_unavailable
-    assert cand.pickable == any(f.fit["pickable"] for f in truth.faces if f.accessible and f.axis == cand.axis)
+    # T11 entered through a y face has no roof board edges along the forks: near_tracking_unavailable.
+    # A face that passes still waits for the near-range re-observation (§9.1): pickable None, never
+    # True, with the unverified items listed (Codex 2026-10-16: a known template is not a measurement).
+    truth_ok = any(f.fit["pickable"] for f in truth.faces if f.accessible and f.axis == cand.axis)
+    assert cand.pickable is (None if truth_ok else False)
+    assert cand.pending == F.FAR_PENDING
 
 
 def test_left_is_the_larger_lateral_pocket():
@@ -280,7 +289,7 @@
     scene, ids = _render([(T11, rt.Placement(4.0, 0.0, math.pi / 2, 0.0), damage)])
     strict = replace(F.FitParams(), min_score=0.6)
     cand = F.fit_instance(scene, InstanceProposal("p0", ids == 0, 1.0, "oracle"), specs=SPECS,
-                          setup=ForkSetup(0.290), load_for=lambda s: SITE.get(s.name), params=strict)
+                          setup=ForkSetup(0.290), load_for=_load, params=strict)
     assert (cand.geometry == "unfit") == (cand.score < 0.6)
 
 
@@ -324,7 +333,7 @@
     scene, ids = _render([(T11, rt.Placement(2.0, 0.0, 0.0, 0.0), ()), (T11, rt.Placement(3.2, 0.0, 0.0, 0.0), ())],
                          distractors=occluders)
     props = [InstanceProposal(f"p{k}", ids == k, 1.0, "oracle") for k in (0, 1) if (ids == k).any()]
-    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.290), load_for=lambda s: SITE.get(s.name))
+    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.290), load_for=_load)
     instr = {"present": True, "x_m": 2.6, "y_m": 0.0, "radius_m": 0.62, "layer": None, "load": None}
     assert select_target(out.candidates, instr, unresolved=out.failures).status != "selected"
 
@@ -337,7 +346,7 @@
     s7 = next(r for r in rs.catalogue() if r.scene_id == "s7_nonstandard_1000769").instances[0].spec
     scene, ids = _render([(s7, rt.Placement(2.0, -0.4, 0.0, 0.0), ()), (T11, rt.Placement(3.2, 0.4, 0.0, 0.0), ())])
     props = [InstanceProposal(f"p{k}", ids == k, 1.0, "oracle") for k in (0, 1) if (ids == k).any()]
-    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.290), load_for=lambda s: SITE.get(s.name))
+    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.290), load_for=_load)
     instr = {"present": True, "x_m": 2.6, "y_m": 0.0, "radius_m": 0.73, "layer": None, "load": None}
     assert select_target(out.candidates, instr, unresolved=out.failures).status != "selected"
 
@@ -409,16 +418,22 @@
 @pytest.mark.parametrize("scene_id", ["s7_nonstandard_1000738", "s7_nonstandard_1000759"])
 def test_a_non_standard_pallet_is_not_fitted_as_a_known_shape(scene_id):
     """Rig renders of S7 dev recipes (0.77 m wide with four blocks; 0.99 m wide) that the EPAL raster
-    fits when the shape check is off (ws1 dev 2026-10-16: 76 of 135 S7 pallets came out fitted)."""
+    fits when the shape check is off (ws1 dev 2026-10-16: 76 of 135 S7 pallets came out fitted).
+    With the parametric reading off the shape check leaves them ambiguous; with it on (stage 2B)
+    they are read as non-standard, never as a known shape."""
     from dataclasses import replace
     recipe = next(r for r in rs.catalogue() if r.scene_id == scene_id)
     out = instance_rig.render_instances([rs.recipe_boxes(recipe, 0)])
     proposal = InstanceProposal("p0", out.instance_ids == 0, 1.0, "oracle")
-    kw = dict(specs=SPECS, setup=ForkSetup(recipe.fork_spacing_m), load_for=lambda s: SITE.get(s.name))
-    off = replace(F.FitParams(), shape_outside_max=9.0, shape_seen_through_max=9.0)
+    kw = dict(specs=SPECS, setup=ForkSetup(recipe.fork_spacing_m), load_for=_load)
+    known_only = replace(F.FitParams(), parametric=False)
+    off = replace(known_only, shape_outside_max=9.0, shape_seen_through_max=9.0)
     assert F.fit_instance(out.scene, proposal, params=off, **kw).geometry == "fitted"
-    cand = F.fit_instance(out.scene, proposal, **kw)
+    cand = F.fit_instance(out.scene, proposal, params=known_only, **kw)
     assert cand.geometry == "ambiguous" and "shape_mismatch" in cand.reasons
+    full = F.fit_instance(out.scene, proposal, **kw)
+    # 1000759 sits at 1.07 m with its right end out of the image: unreadable, so never a confident fit
+    assert full.hypothesis == "parametric:x" or full.geometry != "fitted"
 
 
 def test_a_geometric_proposal_stays_unconfirmed():
@@ -451,7 +466,7 @@
     placement = rt.Placement(2.9, 0.2, 0.2, 0.0)
     scene, ids = _render([(EPAL, placement, ())])
     cand = F.fit_instance(scene, InstanceProposal("p0", dilate(ids == 0), 1.0, "oracle"), specs=SPECS,
-                          setup=ForkSetup(0.3725), load_for=lambda s: SITE.get(s.name))
+                          setup=ForkSetup(0.3725), load_for=_load)
     truth = rt.scene_truths([(EPAL, placement, (), SITE[EPAL.name])], ForkSetup(0.3725))[0][0]
     lateral, axial, yaw, z = _errors(cand, truth)
     assert cand.geometry == "fitted" and lateral <= 0.010 and axial <= 0.010 and z <= 0.006
@@ -463,7 +478,7 @@
     tiny[0, 0] = True
     props = [InstanceProposal("a", ids == 0, 1.0, "oracle"), InstanceProposal("b", tiny, 1.0, "oracle"),
              InstanceProposal("c", ids == 1, 1.0, "oracle")]
-    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=lambda s: SITE.get(s.name))
+    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=_load)
     assert [(c.candidate_id, c.proposal_id) for c in out.candidates] == [("c0", "a"), ("c2", "c")]
     assert [(f.proposal_id, f.reason) for f in out.failures] == [("b", "no_face")]
     real = F.fit_instance
@@ -474,7 +489,7 @@
         return real(scene, proposal, **kw)
 
     monkeypatch.setattr(F, "fit_instance", boom)
-    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=lambda s: SITE.get(s.name))
+    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=_load)
     assert [c.proposal_id for c in out.candidates] == ["c"]
     failed = {f.proposal_id: f for f in out.failures}
     assert failed["a"].reason == "exception:FloatingPointError"
@@ -487,7 +502,7 @@
     and stopped the scene, so the next (valid) proposal was never fitted."""
     scene, ids = _render([(EPAL, rt.Placement(2.6, 0.0, 0.0, 0.0), ())])
     props = [InstanceProposal("w", np.ones((2, 2), bool), 1.0, "oracle"), InstanceProposal("a", ids == 0, 1.0, "oracle")]
-    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=lambda s: SITE.get(s.name))
+    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=_load)
     assert [c.proposal_id for c in out.candidates] == ["a"]
     (failure,) = out.failures
     assert failure.proposal_id == "w" and failure.reason.startswith("bad_mask") and failure.centre_xy is None
```

- [ ] **Step 2: 실패를 확인한다**

Run: `FORKLIFT_REPO=$PWD ~/bin/forklift-ws1-pytest tests/unit/perception/test_parametric_fit.py tests/unit/perception/test_instance_fit.py -q`
Expected: `read_parametric_face`·`_parametric_fit`·`FAR_PENDING` 이 없어 새 시험이 실패한다. 알려진 형상 4 배치 시험은 앞의 셋이 `pickable True is None` 으로, T11 y 면 배치는 기존 `False` 검사를 통과한 뒤 `F.FAR_PENDING` 이 없어 실패한다. CLI 시험(`test_the_evaluator_gives_a_non_standard_reading_no_site_default`)은 `site.get(None.name)` 의 AttributeError 로 실패한다.

- [ ] **Step 3: 구현한다**

```diff
--- a/src/forklift_core/perception/instance_fit.py
+++ b/src/forklift_core/perception/instance_fit.py
@@ -18,14 +18,21 @@
 
 import numpy as np
 
-from forklift_core.perception.fork_fit import fork_fit
+from forklift_core.perception.fork_fit import (
+    M_REQ_PROVISIONAL_M, PAYLOAD_LIMIT_KG, REASON_CODES, SUPPORT_LATERAL_MARGIN_M, SUPPORT_LONGITUDINAL_MARGIN_M, fork_fit,
+)
 from forklift_core.perception.instance_proposals import InstanceProposal
-from forklift_core.perception.pallet_model import ObservedPalletShape, PalletSpec, face_layout, spec_boxes
+from forklift_core.perception.pallet_geometry import target_insertion_depth_m
+from forklift_core.perception.pallet_model import FaceLayout, ObservedPalletShape, PalletSpec, face_layout, pair_margin, spec_boxes
 from forklift_core.perception.pocket_candidate import PocketCandidate, PocketEstimate
 from forklift_core.perception.scene_dataset import SceneInput
 
 BEHIND, FACE, FRONT, UNOBSERVED = 0, 1, 2, 3
 HEIGHT_REASONS = ("lift_alignment_needed",)  # fork-fit reasons that read the support height
+# The far-range path never permits a pick: the near-range re-observation (§9.1) verifies these first.
+# A known template fills them, but a non-standard pallet can match a template's face and differ in
+# its lane height or top (S7 dev 2026-10-16: T11-like lanes 3-5 mm lower, EPAL-like flat tops).
+FAR_PENDING = ("depth", "lane_bounds", "top_layout")
 
 
 @dataclass(frozen=True)
@@ -60,6 +67,19 @@
     peak_window_m: float = 0.02         # a local maximum beats every placement within this distance
     equivalence_m: float = 0.020
     equivalence_yaw_rad: float = math.radians(2.0)
+    parametric: bool = True              # read non-standard faces when no known shape explains them (§7.1)
+    parametric_row_fill: float = 0.65    # rows of the top band are at least this full (boards leave gaps)
+    parametric_member_fill: float = 0.5  # a member column is at least this full below the lane ceiling
+    parametric_top_skip_m: float = 0.03  # board ends (S7 top boards 12-25 mm, along x) and a ragged edge above the deck run
+    min_opening_m: float = 0.05          # narrower gaps are member splits, not fork lanes (a split block is 5-15 mm)
+    min_lane_ceiling_m: float = 0.03
+    end_window_m: tuple = (0.005, 0.03)  # rays this far beyond an end decide whether the end is physical
+    nonstandard_depth_m: tuple = (0.50, 1.20)  # depth range of non-standard pallets (design §5.1): unobserved here
+    # The far range stops a non-standard pick for fork spacing only below M_REQ minus this; in between
+    # the lane bounds wait for the near range, which applies M_REQ itself. An empirical dev margin
+    # (provisional noise, 2026-10-16: far margins read up to 42.5 mm low at the high level, pickable
+    # S7 pallets sit 4-6 mm above M_REQ), not a measured error bound -- revisit with the D435i noise.
+    parametric_spacing_slack_m: float = 0.030
     seed: int = 0
 
 
@@ -539,6 +559,211 @@
     return "unconfirmed"
 
 
+def _runs(flags):
+    out, start = [], None
+    for i, f in enumerate(list(flags) + [False]):
+        if f and start is None:
+            start = i
+        if not f and start is not None:
+            out.append((start, i))
+            start = None
+    return out
+
+
+@dataclass(frozen=True)
+class ParametricReading:
+    """A non-standard entry face read off the data (§7.1): member and opening layout across the
+    face, the lane ceiling above the support, and whether each end is the object's physical end.
+    The depth is not read, and the top layout is left to the near range: from the front it is a few
+    pixels this reader does not use (§3.2), not something that cannot be seen."""
+    layout: FaceLayout
+    u_centre: float
+    support_z: float
+    top_m: float
+    ends_physical: tuple
+
+
+def read_parametric_face(face: FacePlane, pts_u, pts_h, rays, support_z: float, params: FitParams, rows=None):
+    """Members and openings from the face's own points; ends from the raw rays it owns.
+
+    The top band is the contiguous run of rows from the top that are at least parametric_row_fill
+    full, counted on rows=(u, h) -- the proposal's un-eroded pixels on the face plane: erosion
+    removes most of a thin deck band at range (mixed-depth edge pixels fall off the plane anyway).
+    Below it, a column is a member if it is at least parametric_member_fill full, and a gap is
+    an opening only if it is min_opening_m wide and rays pass behind the face through it. Fork
+    lanes run down to the support (bottom boards only under the members), so the lane floor is
+    the support and the bottom-board thickness is not observable from the front. Returns
+    (reading, None) or (None, reason).
+    """
+    b = params.bin_m
+    ru, rh, rc, owned = rays
+    top = float(np.percentile(pts_h, 99))
+    on = owned & (rc == FACE) & (rh > support_z + 0.005) & (rh < top)
+    if on.sum() < 20:
+        return None, "few_face_rays"
+    umin, umax = float(np.percentile(ru[on], 0.5)), float(np.percentile(ru[on], 99.5))
+    nu, nh = int(math.ceil((umax - umin) / b)), int(math.ceil((top - support_z) / b))
+    if nu < 10 or nh < 4:
+        return None, "too_small"
+    occ = np.zeros((nh, nu), bool)
+    occ[np.clip(((pts_h - support_z) / b).astype(int), 0, nh - 1), np.clip(((pts_u - umin) / b).astype(int), 0, nu - 1)] = True
+    row_u, row_h = (pts_u, pts_h) if rows is None else rows
+    keep = (row_u >= umin) & (row_u <= umax) & (row_h >= support_z)
+    rocc = np.zeros((nh, nu), bool)
+    rocc[np.clip(((row_h[keep] - support_z) / b).astype(int), 0, nh - 1),
+         np.clip(((row_u[keep] - umin) / b).astype(int), 0, nu - 1)] = True
+    row_fill = rocc.mean(axis=1)
+    # Above the full deck run there can be rows that are not full: a ragged top edge (mixed-depth
+    # pixels, the 99th-percentile top cuts through a row) and, where top boards run along the
+    # insertion axis, their ends with gaps between them (S7 fill 0.64). Skip up to
+    # parametric_top_skip_m of them before the deck run starts, else the run is empty and the
+    # ceiling lands on the top, reading the whole deck as openings (dev 2026-10-16).
+    k = nh - 1
+    while k > 0 and row_fill[k] < params.parametric_row_fill and (nh - 1 - k) * b < params.parametric_top_skip_m:
+        k -= 1
+    if row_fill[k] < params.parametric_row_fill:
+        k = nh - 1
+    while k > 0 and row_fill[k] >= params.parametric_row_fill:
+        k -= 1
+    ceiling = (k + 1) * b
+    if ceiling < params.min_lane_ceiling_m:
+        return None, "no_lane_band"
+    col_fill = occ[1:k + 1].mean(axis=0) if k >= 1 else occ[:1].mean(axis=0)
+    band = (rh > support_z + 0.01) & (rh < support_z + ceiling - 0.005)
+    sel = band & (rc == BEHIND) & (ru >= umin) & (ru < umax)
+    behind = np.zeros(nu)
+    np.add.at(behind, np.clip(((ru[sel] - umin) / b).astype(int), 0, nu - 1), 1)
+    member = col_fill >= params.parametric_member_fill
+    openings = []
+    for lo, hi in _runs(~member):
+        if (hi - lo) * b >= params.min_opening_m and behind[lo:hi].sum() >= params.edge_min_hits:
+            openings.append((lo, hi))
+        else:
+            member[lo:hi] = True
+    members = _runs(member)
+    if len(members) < 3 or len(openings) < 2:
+        return None, f"layout:{len(members)}m{len(openings)}o"
+    c = (umin + umax) / 2
+    ends = []
+    for edge, side in ((umin, -1.0), (umax, 1.0)):
+        off = side * (ru - edge)
+        beyond = band & (off > params.end_window_m[0]) & (off < params.end_window_m[1])
+        ends.append(bool(((beyond & (rc == BEHIND)).sum() >= params.edge_min_hits)
+                         and ((beyond & (rc == FRONT)).sum() < params.edge_min_hits)))
+    layout = FaceLayout("x", umax - umin, math.nan,
+                        tuple((lo * b + umin - c, hi * b + umin - c) for lo, hi in members),
+                        tuple((lo * b + umin - c, hi * b + umin - c) for lo, hi in openings),
+                        (0.0, ceiling), 0.0, ceiling)
+    return ParametricReading(layout, c, support_z, top, (ends[0], ends[1])), None
+
+
+def _layout_raster(layout: FaceLayout, height: float, bin_m: float):
+    """Occupancy of a parametric face: members up to the lane ceiling, the full width above."""
+    nu, nh = int(round(layout.width_m / bin_m)), max(1, int(round(height / bin_m)))
+    uc = -layout.width_m / 2 + (np.arange(nu) + 0.5) * bin_m
+    hc = (np.arange(nh) + 0.5) * bin_m
+    occ = np.zeros((nh, nu), bool)
+    occ[hc >= layout.lane_ceiling_m, :] = True
+    for lo, hi in layout.member_intervals_m:
+        occ[:, (uc >= lo) & (uc <= hi)] = True
+    return occ
+
+
+def _parametric_fit(layout: FaceLayout, setup, load, depth_max_m: float,
+                    spacing_slack_m: float = FitParams.parametric_spacing_slack_m):
+    """What the face and the instruction decide (§9.3): the fork spacing, the support and the payload.
+
+    Each opening pair is judged on its lateral margin and its support together, like fork_fit: the
+    widest-margin pair may fail lateral support while another passes (Codex 2026-10-16). Only the
+    instruction's load counts: without it the support is unverified, as in fork_fit -- a non-standard
+    pallet has no template mass and the near range cannot supply one. The depth is not seen, so the
+    longitudinal support uses the largest insertion target any depth up to depth_max_m allows -- a
+    load centre beyond it fails at every depth. The lift is not decided: the
+    far reading's 5 mm rows do not bound the lane tightly enough for a 2.5 mm margin, so the lane
+    bounds wait for the near range (FAR_PENDING). The lateral margin stops the pick only below
+    M_REQ - spacing_slack_m: the far reading's margins can be well under the true ones, and between
+    the two the near range decides on M_REQ. Returns (pair, pickable False | None, reasons)."""
+    target_max = target_insertion_depth_m(depth_max_m, setup.carriage_limit_m, setup.reserve_m)
+    options = []
+    n = len(layout.opening_intervals_m)
+    for i in range(n):
+        for j in range(i + 1, n):
+            margin, centre = pair_margin(layout, (i, j), setup.spacing_m, setup.blade_width_m)
+            reasons = set()
+            if margin < M_REQ_PROVISIONAL_M - spacing_slack_m:
+                reasons.add("fork_spacing_adjust")
+            if load is None or load.source != "instruction" or (
+                    target_max < load.com_x_max_m + SUPPORT_LONGITUDINAL_MARGIN_M
+                    or abs(centre) + load.com_y_max_m > setup.spacing_m / 2 - SUPPORT_LATERAL_MARGIN_M):
+                reasons.add("support_unverified")
+            options.append(((-round(margin, 9), round(abs(centre), 9)), (i, j), reasons))
+    options.sort(key=lambda o: o[0])
+    _, pair, reasons = next((o for o in options if not o[2]), options[0])
+    if load is not None and load.pallet_mass_kg + load.load_mass_kg > PAYLOAD_LIMIT_KG:
+        reasons = reasons | {"payload_limit"}
+    ordered = tuple(code for code in REASON_CODES if code in reasons)
+    return pair, (False if ordered else None), ordered
+
+
+def _parametric_candidate(scene, proposal, cam, face, fi, pts_u, pts_h, rays_f, reading, setup, load_for, params,
+                          candidate_id, identity_threshold, competing, state, seen, extent):
+    layout = reading.layout
+    occ = _layout_raster(layout, reading.top_m - reading.support_z, params.bin_m)
+    # The top band was read as one block, but boards leave gaps the rays pass through: like the
+    # damageable band of a known shape, it is not checked for seen-through rays.
+    top_band = (np.arange(occ.shape[0]) + 0.5) * params.bin_m >= layout.lane_ceiling_m
+    placement = best_placement(occ, pts_u, pts_h, rays_f, params, reading.support_z, 1, top_band)
+    hyp = Hypothesis("parametric:x", None, "x", False, layout.member_intervals_m, layout.pocket_band_m, placement, fi)
+    reach = _reach(cam, face.centre_xy + face.left_xy * placement.u_centre)
+    geometry, reasons = "fitted", []
+    if placement.score < params.min_score:
+        geometry, reasons = "unfit", ["weak_fit"]
+    elif placement.score - placement.runner_up_score < params.ambiguity_margin:
+        geometry, reasons = "ambiguous", ["competing:parametric:x@shifted"]
+    elif pts_u.max() - pts_u.min() < params.min_face_span_m:
+        geometry, reasons = "ambiguous", ["yaw_unconstrained"]
+    elif observed_members(hyp, rays_f, params, max(params.edge_window_m, 4 * reach / scene.intrinsics.fx)) < 2:
+        geometry, reasons = "ambiguous", ["lateral_unconstrained"]
+    elif not all(reading.ends_physical):
+        # A non-standard reading must see both physical ends: the inner part of two touching
+        # pallets looks like one wide non-standard pallet (§7.4-5c).
+        geometry, reasons = "ambiguous", ["end_unobserved"]
+    pair, pickable, fit_reasons = _parametric_fit(layout, setup, load_for(None), params.nonstandard_depth_m[1],
+                                                  params.parametric_spacing_slack_m)
+    centre = face.centre_xy + face.left_xy * placement.u_centre
+    lats = sorted(((layout.opening_intervals_m[i][0] + layout.opening_intervals_m[i][1]) / 2,
+                   layout.opening_intervals_m[i][1] - layout.opening_intervals_m[i][0]) for i in pair)
+    # The lane floor is not read, so the pocket centre sits at the support + half the ceiling: below
+    # the truth convention (deck_bottom + ceiling) / 2 by deck_bottom / 2 (S7 dev height p95 12.8 mm
+    # against 5 mm, Codex 2026-10-16). The near range re-reads both lane bounds and regenerates the
+    # pockets (stage 4); until then lane_bounds is pending.
+    z = reading.support_z + layout.lane_ceiling_m / 2
+    right = PocketEstimate((*(centre + face.left_xy * lats[0][0]), z), lats[0][1], layout.lane_ceiling_m,
+                           "observed", "observed")
+    left = PocketEstimate((*(centre + face.left_xy * lats[1][0]), z), lats[1][1], layout.lane_ceiling_m,
+                          "observed", "observed")
+    d_lo, d_hi = params.nonstandard_depth_m
+    # The depth is not seen: the centre may lie anywhere on the segment from half the shallowest to
+    # half the deepest depth behind the face; selection tests the whole segment (Codex 2026-10-16:
+    # five samples missed an instruction between two of them).
+    pallet_centre = centre + face.insertion_xy * (d_lo + d_hi) / 4
+    segment = tuple((float(p[0]), float(p[1]), float(reading.support_z))
+                    for p in (centre + face.insertion_xy * d_lo / 2, centre + face.insertion_xy * d_hi / 2))
+    alternatives = tuple(competing) + segment
+    if state == "uncertain":
+        reasons = list(reasons) + ["support_uncertain"]
+    shape = ObservedPalletShape(None, layout, (("face", "observed"), ("depth", "unobserved"), ("top", "unobserved"),
+                                               ("support", "unobserved" if state == "uncertain" else "observed")))
+    yaw = math.atan2(face.insertion_xy[1], face.insertion_xy[0])
+    return PocketCandidate(
+        candidate_id, proposal.proposal_id, proposal.source, scene.stamp_ns, "parametric:x", "x", (), yaw,
+        (float(centre[0]), float(centre[1]), float(reading.support_z)),
+        (float(pallet_centre[0]), float(pallet_centre[1]), float(reading.support_z)), left, right,
+        reading.support_z, state, state == "visible" and abs(reading.support_z) <= params.support_window_m, reach,
+        placement.score, geometry, _identity(proposal, identity_threshold), tuple(reasons), pickable, fit_reasons,
+        shape, alternatives, (float(seen[0]), float(seen[1])), extent, FAR_PENDING, segment)
+
+
 def _decide(hyps, faces, evidence, pts, scene, setup, load_for, params):
     """Rank the hypotheses and apply every gate: weak fit, rivals, face span, member edges, shape."""
     cam = camera_xyz(scene)
@@ -635,12 +860,13 @@
         fixed[hyp.face] = measured
     hyp, fit, centre, left, right, yaw = best
     reach = _reach(cam, centre)
-    pickable, fit_reasons = fit.pickable, tuple(fit.reasons)
+    fit_reasons = tuple(fit.reasons)
     if state == "uncertain":
         # An unseen support may not decide anything that reads the height (§7.3).
         fit_reasons = tuple(r for r in fit_reasons if r not in HEIGHT_REASONS)
-        pickable = False if fit_reasons else None
         reasons = list(reasons) + ["support_uncertain"]
+    # Far range: a reason the face decides stops the pick; otherwise it waits for the near range.
+    pickable = False if fit_reasons else None
     layout = face_layout(hyp.spec, hyp.axis)
     pallet_centre = centre + face.insertion_xy * layout.depth_m / 2
     # Centres of the readings close enough to make this one ambiguous: the shifted placement of the
@@ -663,13 +889,33 @@
     zs = hyp.placement.support_z
     seen = np.median(pts[:, :2], axis=0)
     extent = float(np.max(np.hypot(*(pts[:, :2] - seen).T)))
+    if params.parametric and geometry != "fitted" and reasons and reasons[0].split(":")[0] in ("shape_mismatch", "weak_fit"):
+        # No known shape explains this object (shape mismatch or a weak fit): read the face as a
+        # non-standard pallet (§7.1), on the faces in order of evidence. A known reading that is only
+        # under-constrained (occluded member edges) or ambiguous between known shapes is left as it
+        # is -- trying the parametric reading there read partly hidden known pallets as narrower
+        # non-standard ones (dev 2026-10-16). The known reading's centre stays among the competing ones.
+        competing_all = tuple(competing) + ((float(pallet_centre[0]), float(pallet_centre[1]), float(zs)),)
+        raw, _ = proposal_points(scene, rays, proposal.mask, 0)
+        for fi in sorted(range(len(faces)), key=lambda k: -len(faces[k].index)):
+            f = faces[fi]
+            u_f, rays_f = evidence[fi]
+            h_f = pts[f.index, 2]
+            low = float(np.percentile(h_f, 2))
+            st, measured = support_state(scene, rays, proposal.mask, f, low, params)
+            on_plane = np.abs((raw[:, :2] - f.centre_xy) @ f.normal_xy) < max(params.ransac_tol_m, 2.5 * f.scale_m)
+            rows = face_coords(f, raw[on_plane])
+            reading, _ = read_parametric_face(f, u_f, h_f, rays_f, measured if measured is not None else low, params, rows)
+            if reading is not None:
+                return _parametric_candidate(scene, proposal, cam, f, fi, u_f, h_f, rays_f, reading, setup, load_for,
+                                             params, candidate_id, identity_threshold, competing_all, st, seen, extent)
     return PocketCandidate(
         candidate_id, proposal.proposal_id, proposal.source, scene.stamp_ns, hyp.label, hyp.axis,
         tuple((s[0].label, round(s[0].placement.score, 4)) for s in rivals[:3]), yaw,
         (float(centre[0]), float(centre[1]), float(zs)), (float(pallet_centre[0]), float(pallet_centre[1]), float(zs)),
         left, right, zs, state, state == "visible" and abs(zs) <= params.support_window_m, reach,
         hyp.placement.score, geometry, _identity(proposal, identity_threshold), tuple(reasons), pickable,
-        fit_reasons, shape, tuple(competing), (float(seen[0]), float(seen[1])), extent)
+        fit_reasons, shape, tuple(competing), (float(seen[0]), float(seen[1])), extent, FAR_PENDING)
 
 
 @dataclass(frozen=True)
```

```diff
--- a/src/forklift_core/perception/target_selection.py
+++ b/src/forklift_core/perception/target_selection.py
@@ -82,14 +82,28 @@
     return min(cands, key=lambda c: (round(c.range_m, 3), c.pallet_centre_m[1]))
 
 
+def _segment_distance(p, a, b) -> float:
+    """Horizontal distance from p to the segment a-b."""
+    a, b = np.asarray(a[:2], float), np.asarray(b[:2], float)
+    d = b - a
+    t = 0.0 if not d.any() else float(np.clip((p - a) @ d / (d @ d), 0.0, 1.0))
+    return float(np.linalg.norm(p - (a + t * d)))
+
+
 def _could_be_inside(c, target, radius) -> bool:
     """A fitted, confirmed candidate is where its fit says. Any other candidate is uncertain: it is
     in the radius if its centre, a competing reading or -- conservatively, like an unfitted object --
     its visible points within the largest half-diagonal could be (Codex review A round 4)."""
-    centres = [c.pallet_centre_m] + ([] if c.counted else list(c.alternative_centres_m))
+    # A non-standard fit has no known depth: its centre may lie anywhere along the depth range.
+    exact = c.counted and c.shape.spec is not None
+    centres = [c.pallet_centre_m] + ([] if exact else list(c.alternative_centres_m))
     if any(np.linalg.norm(np.asarray(p[:2]) - target) <= radius for p in centres):
         return True
+    if c.centre_segment_m and _segment_distance(target, *c.centre_segment_m) <= radius:
+        return True
     if c.counted:
+        # A fit -- known or non-standard -- is where its centres say; the visible-points fallback
+        # below is for uncertain candidates only (independent review 2026-10-16).
         return False
     if c.observed_centre_xy is None:
         return True
```

```diff
--- a/tools/evaluate_recognition.py
+++ b/tools/evaluate_recognition.py
@@ -92,7 +92,8 @@
 def _load_for(instruction, site):
     given = instruction.get("load") if instruction.get("present") else None
     declared = None if given is None else LoadDeclaration(**given)
-    return lambda spec: declared if declared is not None else site.get(spec.name)
+    # A non-standard reading (spec None) has no site default: only an instructed load applies.
+    return lambda spec: declared if declared is not None else (None if spec is None else site.get(spec.name))
 
 
 def _oracle_fit(sample):
```

(CLI 수정이 이 작업에 있어야 한다: 매개변수 판독이 기본으로 켜져 `load_for(None)` 을 부르므로, 없으면 이 작업의 커밋에서 평가기가 모든 매개변수 판독을 `exception:AttributeError` 실패로 바꾼다 — Codex 계획 검토.)

- [ ] **Step 4: 통과를 확인한다** — 같은 명령에 `test_target_selection.py test_instance_proposals.py test_pocket_candidate.py test_evaluation_multi.py` 를 더해, Expected: 전부 통과.

- [ ] **Step 5: 커밋한다**

```bash
git add src/forklift_core/perception/instance_fit.py src/forklift_core/perception/target_selection.py tools/evaluate_recognition.py tests/unit/perception/test_parametric_fit.py tests/unit/perception/test_instance_fit.py
git commit -m "perception: read faces no known shape explains as non-standard pallets (members, openings, lane ceiling, physical ends); the far range never permits a pick -- face and instruction reasons stop it, everything else waits for the near range"
```

### Task 3: 평가 CLI, ws1 dev 재집계와 기록

**Files:**
- Create: `deploy/slurm/recognition_stage2b_evaluate.sbatch`, `docs/validation/2026-10-16-recognition-stage2b1-parametric.md`

- [ ] **Step 1: sbatch 를 더한다**

```bash
#!/bin/bash
#SBATCH --job-name=recog-2b-eval
#SBATCH --partition=compute
#SBATCH --cpus-per-task=2
#SBATCH --mem=1G
#SBATCH --time=00:45:00
#SBATCH --output=%x-%j.log
# Stage 2B dev evaluation: oracle-fit only (the baseline detector is unchanged since 2A), two
# datasets at a time. 1B/2A evaluation processes peaked near 230 MB each (job 2876: 8 processes,
# MaxRSS 1.8 GB), so two fit in 1G, which ws1 could still schedule on 2026-10-16.
# Usage: sbatch --chdir=<run dir> deploy/slurm/recognition_stage2b_evaluate.sbatch <source dir> <isaac run dir> <manifest>
# Every process is waited for by PID and must leave a metrics.json: a bare `wait` hides failures.
set -uo pipefail
src=$1; run=$2; manifest=$3
PY=/home/projects/forklift/.venv/bin/python
export PYTHONPATH="$src/src:$src" PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
rc=0
for pair in "clean low" "mid high"; do
  pids=(); names=()
  for ds in $pair; do
    $PY "$src/tools/evaluate_recognition.py" --dataset "$run/isaac_$ds" --split dev --detector oracle-fit \
      --output "out/${ds}_oracle-fit" --diagnostics --source-manifest "$manifest" > "out_${ds}_oracle-fit.log" 2>&1 &
    pids+=($!); names+=("${ds}_oracle-fit")
  done
  for i in "${!pids[@]}"; do
    if ! wait "${pids[$i]}"; then echo "FAILED ${names[$i]}"; rc=1; fi
    [ -s "out/${names[$i]}/metrics.json" ] || { echo "MISSING out/${names[$i]}/metrics.json"; rc=1; }
  done
done
echo "ALL_DONE rc=$rc"
exit $rc
```

- [ ] **Step 2: 같은 평가기로 2A 코드와 2B 를 잰다** — ws1 에서 (a) 2A 머지 커밋 `93f1e47` 에 작업 1 의 `evaluation_multi.py` 만 얹은 트리, (b) 이 브랜치 HEAD 를 각각 `sbatch --chdir=<run> deploy/slurm/recognition_stage2b_evaluate.sbatch <source> /home/projects/forklift/artifacts/20261015T150838Z_recognition_isaac <manifest>` 로 돌린다. Expected: 두 잡 모두 `ALL_DONE rc=0`, 데이터셋 넷의 `metrics.json` 에 `diagnostics`.

- [ ] **Step 3: 기록을 쓰고 커밋한다** — 사례·형상·잡음 수준별 놓침/위험/자세 위험/대상 오류/보류(2A ‖ 2B), 자세 위험 진단(검사 장면·후보·미대응), 비규격 끼움 판정 완료 수, S1–S6 의 매개변수 후보 수, 남은 S7 놓침 원인.

```bash
git add deploy/slurm/recognition_stage2b_evaluate.sbatch docs/validation/2026-10-16-recognition-stage2b1-parametric.md
git commit -m "docs: stage 2B-1 record -- non-standard reading and the far-range pick gate on the Isaac dev scenes, measured against the 2A code with the same evaluator"
```

## 자체 검토

### ① 원자료 재계산 (Claude, 2026-10-16)

- 필요 천장·바닥: `ForkSetup(0.29)` 의 `blade_top_m` 0.0525, `blade_bottom_m` 0.0275 와 `VERTICAL_MARGIN_M` 0.005 → 천장 ≥ 57.5 mm, 바닥 ≤ 22.5 mm. T11×0.6 x 면 차선 15→60 mm 는 위쪽 여유 2.5 mm.
- S7 lift 위험 4건의 차선 천장(정답 `deck_bottom_m + pocket_height_m`): 1000779 56.69, 1000794 56.01, 1000869 55.55, 1000911 55.68 mm → 필요 천장에 0.81–1.95 mm 못 미친다.
- 최대 삽입 목표: `target_insertion_depth_m(1.2, 0.346, 0.016)` = min(0.72, 0.330) = 0.330 m. 지시 질량중심 0.31 m + 0.036 = 0.346 > 0.330 → 깊이와 무관하게 `support_unverified`(시험 `test_a_load_beyond_any_insertion_depth_stops_the_pick_without_the_depth`).
- 세로 화소 간격: Isaac dev `camera_info.json` 의 fy 465.74 → 2·3·4 m 에서 4.29·6.44·8.59 mm.
- 개구 쌍 반례(Codex): 개구 [−325,−135]·[−50,170]·[240,460] mm, 간격 290·날 55 mm. 쌍 (0,1) 중심 −85 mm, 날 [−257.5,−202.5]·[32.5,87.5] → 여유 67.5·67.5·82.5·82.5, 최소 67.5 ≥ 66 mm(`M_REQ_PROVISIONAL_M`), |−85| ≤ 145 − 20 = 125 → 통과. 쌍 (1,2) 중심 205 mm > 125 → 횡지지 실패. 시험 `test_the_opening_pair_is_chosen_with_its_support` 가 (0,1) 을 요구한다.
- 윗판 끝 채움: S7 조리법 `top_board_count = max(2, min(9, int(width / (board × 1.4))))` 이므로 판 끝 띠의 채움은 count·board/width ≤ 1/1.4 ≈ 0.71 이고 내림 때문에 더 낮다(dev 1000732 에서 0.64). 판 두께 `top_thickness_m` 0.012–0.025 m → 건너뛰기 30 mm 는 판 끝 25 mm + 들쭉날쭉한 맨 윗줄 한 칸.

### ② Codex 적대적 검토

**설계 상의(구현 전, 같은 스레드 5 회 + 수렴 뒤 1 회).**
- 1회차: 원거리 정면 통과를 `pickable True` 로 내면 미측정 상판·깊이까지 확인한 뜻이 된다(P1) — 알려진 형상에도 미확인 항목이 남으면 `None`. "원리상 관측 불가 → 지표에서 제외" 는 반례가 있다(P2, CPU 리그에서 `flat→boards` 만 바꾸면 1000774 526 화소·1000918 151 화소 깊이가 1 mm 넘게 달라짐) → 지표 축소 제안 철회. 세로는 실제 필요 천장 57.5 mm(여유 2.5 mm)로 판단(P2). 보류가 콘솔 요청을 늘리지 않음(`_decide` 75행)·평가기 분모에서 빠짐을 구분(P2). 깊이 없이도 지시 질량중심으로 지지 실패를 알 수 있음(P3). → 공통 관문·`pending`·`pose_dangerous`·지시 기반 지지로 반영.
- 2회차: 관측 천장 + 5 mm 는 위반 상한이 아님(P2) → 세로는 보류. `pose_dangerous` 는 2A 와 같은 정의로 재집계해야 비교 가능(P2) → 같은 평가기로 2A 코드 재실행. 개구 쌍을 최대 여유로 고른 뒤 지지를 보면 오거부(P2, 반례 개구 [−325,−135]·[−50,170]·[240,460]) → 쌍마다 함께 판정.
- 3회차: 하중 없음·현장 기본값이면 지지 미확인이어야 함(P2) → `fork_fit` 과 같게. 정답 끼움 불가 면의 음수 T_i 가 정확한 자세를 위험으로 셈(P2) → 그 면은 자세 판정에서 뺌.
- 4회차: 깊이 범위 선택 시험에 지시 하중이 빠짐(P2) → 하중 추가. 5회차: **새 P1/P2 없음(수렴)**.
- 수렴 뒤(dev 재집계): 가로 여유 오판으로 S7 대상 오류 1→9·7·7. δ 를 dev 에서 정하는 것은 누출이 아니지만 **오차 상한이 아니라 경험적 조기 거부 여유**로 적어야 함(P2), 1000873 은 여유 오판이 아니라 미대응 옆면 판독(P2), S1–S6 '전 셀 동일' 서술 오류(P3) → 이 계획의 서술대로 고침.

**계획 검토(이 문서, 3회차) — 새 P1/P2/P3 없음, 수렴.** 중심 선분 반례가 `selected` 로 바뀌고(선분 전달을 빼면 새 시험이 실패함을 확인), 8 개 diff 가 `93f1e47` 에 적용되며, dev 잡 3218 산출물이 잡 3184 와 같고 표 56 셀·D435i 여섯 중앙값이 재계산과 맞음을 확인받았다.

**계획 검토(2회차 — P1 0 · P2 2).** 8 개 diff 적용·표 56 셀·제외 후보·완료 수·자세 p95 가 산출물과 맞음을 확인받았다.
- P2: 깊이 범위를 표본 5 점으로 보면 그 사이의 지시를 놓친다(재현: 선분까지 49 mm, 가장 가까운 표본까지 65.7 mm, 반경 50 mm → `none`) → 후보에 중심 선분(`centre_segment_m`)을 두고 선택은 선분까지의 거리로 판정, 시험(`between` → `selected`) 추가.
- P2: D435i 실측에서 끌어낸 결론이 증거보다 넓다(잔차 규모 유사 ≠ 성능 대응, 유효 깊이 ≠ `BEHIND`, σ 산법·해상도 미기재) → 관찰로 한정하고 방법을 적음, 성능 대응·관문 성립은 미확인으로 둠.

**계획 검토(1회차 — P1 0 · P2 2 · P3 3).** 모든 diff 가 `93f1e47` 에 `git apply --check` 로 깨끗이 적용되고, 표·원인 분류·p95 가 산출물과 맞으며, 작업 1 은 작업 2 없이 RED→GREEN 이 됨을 확인받았다.
- P2: CLI 의 `_load_for(None)` 수정이 작업 3 에 있어 작업 2 커밋에서 매개변수 판독이 전부 `exception:AttributeError` 실패가 된다(재현) → CLI 수정과 회귀 시험을 작업 2 로 옮김.
- P2: 매개변수 포켓 중심이 정답 계약보다 하판/2 낮다(높이 p95 12.8·13.5 mm vs 문턱 5 mm) → 기록하고, 단계 4 가 `pending` 해제와 함께 포켓 추정값을 다시 만드는 조건을 명시.
- P3: 잠정 중간 S4 T11 놓침 17→16 을 '전 셀 동일' 로 잘못 적음, `lane_bounds` 가 가로도 포함함을 타입 주석·범위에 맞춤, 작업 2 RED 기대치(T11 y 면은 `FAR_PENDING` 부재로 실패) → 고침.

### ③ 독립 검토 (Gemini 3.8 Flash High via `agy`, 계획 본문만 — 자체 검토 절과 상의 기록은 빼고 "적히지 않은 전제" 를 물음)

| 지적 | 처리 |
|---|---|
| P1 비규격 후보가 지시 반경 밖인데 선택됨 | **사실이었다.** `exact=False` 인 계수 비규격 후보가 깊이 범위 중심이 모두 반경 밖이어도 '보이는 점' 대체 검사(반경 + 0.85 m)로 넘어가 유일한 후보면 `selected`. 시험(`beside` 지시 → `none`)으로 재현·고정, 계수 후보는 중심만 보도록 고침. dev 표는 바뀌지 않았다(해당 장면 없음) |
| P1 짐이 마스크에 섞이면 천장이 짐까지 올라감 | 정답 마스크에는 없다. 위에서부터 꽉 찬 행을 내려가므로 짐·상판이 붙어 있으면 천장은 맞고, 틈이 있으면 틀린다 → Review Focus 6, 2B-2 시험. `lane_bounds` 보류라 집기 허가로 이어지지 않음 |
| P1 실행기가 `None` 을 못 다룸 | 새 선택 경로는 아직 실행기에 연결되지 않았고, 연결 조건(보류 상태를 잃지 않기)은 단계 4 계획 조건으로 이미 적혀 있다 — 변경 없음 |
| P2 차선 안 광선이 결측이면 개구가 부재로 합쳐짐 | 같은 날 팀 D435i 실측(장난감 팔레트 0.5–2 m)에서 차선 높이 행의 무효 화소는 0–2 %(다리 왼쪽 스테레오 그늘)였다. 결측이 드물다는 관찰일 뿐 `BEHIND` 관문의 성립은 아니다(Codex 계획 검토 2회차) — 실측 재생 검증 전까지 미확인. 실패하면 놓침(위험 아님) |
| P2 하판이 진입부를 지나는 비규격(perimeter base)의 차선 바닥 | 조리법에 없는 형태 → Review Focus 7, 단계 4·실물 시험 |
| P2 지시 하중 필수 | 설계 §9.3(질량 미상은 콘솔 확인)대로이며 Codex 도 같은 판단 — 변경 없음 |
| P2 끼움 불가 면을 자세 판정에서 빼면 잘못된 자세가 숨음 | 빼되 세도록 `pose_excluded`·`excluded_candidates` 를 더함(시험 고정) |
| P2 맞닿은 이웃 때문에 끝이 물리적이지 않게 보임 | 설계 §7.4-5c 의 의도된 보수 동작(놓침) → Review Focus 8 |
| P3 행 채움 0.65·건너뛰기 30 mm 가 S7 조리법에 맞춰짐 | 맞다 — dev 조정값으로 기록하고 실물 시험 팔레트·공식 잡음에서 다시 본다 |
| P3 δ 30 mm 가 최저 −42.5 mm 를 못 덮음 | 계획이 이미 '오차 상한 아님' 으로 적었다 — 변경 없음 |
