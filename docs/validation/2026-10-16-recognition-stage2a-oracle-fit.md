# 인식 단계 2A — 정답 마스크 기하 맞춤(oracle-fit) dev 기록 (2026-10-16)

계획: `docs/plans/2026-10-16-recognition-stage2a-instance-fit.md`. 설계: `docs/design/2026-10-15-pallet-recognition-generality.md` §3.4, §7, §8.

**이 기록은 dev 분할의 결과이며 합격 판정이 아니다.** eval 분할은 열지 않았다. 잡음은 1B 의 잠정 설정(`config/depth_noise_d435i.yaml`, role provisional)이고, 공식 잡음은 팀 D435i 측정 뒤에 정한다.

## 1. 실행

| 항목 | 값 |
|---|---|
| 잡 | ws1 Slurm 3128 (`artifacts/20261016T071718Z_recognition_stage2a`), CPU 8, MaxRSS 0.61 GB, 3 분 9 초 |
| 소스 | `d8c9256`(이 브랜치, 리뷰 A 1회차 수정 포함) — `source_manifest.json` |
| 데이터 | 1B Isaac 렌더 `artifacts/20261015T150838Z_recognition_isaac` 의 `isaac_clean`·`isaac_low`·`isaac_mid`·`isaac_high`, dev 704 장면 |
| 검출기 | `oracle-fit`(정답 인스턴스 ID 영상을 제안으로 → 단계 2 기하 맞춤 → 대상 선택), `baseline`(지금 검출기) |
| 매개변수 | `FitParams()` 기본값(`metrics.json` 의 `fit_params`) |

## 2. 사례별 결과

칸은 **놓침 / 위험 오위치 / 대상 선택 오류 / 애매하지 않은 지시의 재관측·확인 요청** 장면 수, 왼쪽이 `oracle-fit`, ‖ 오른쪽이 `baseline`. 음성(N)은 오검출 장면 수.

| 사례 · 형상 | 장면 | 잡음 없음 | 잠정 낮음 | 잠정 중간 | 잠정 높음 |
|---|---|---|---|---|---|
| S1 EPAL 6 | 42 | 1/0/1/0 ‖ 2/0/2/0 | 7/2/7/0 ‖ 30/0/22/0 | 5/6/5/0 ‖ 32/0/23/0 | 8/13/6/1 ‖ 35/2/28/0 |
| S1 T11 | 42 | 1/0/0/0 ‖ 4/2/2/0 | 3/5/3/0 ‖ 34/1/26/0 | 4/5/4/0 ‖ 37/0/28/0 | 5/12/3/0 ‖ 41/0/31/0 |
| S2 EPAL 6 | 42 | 11/0/3/3 ‖ 41/0/29/0 | 18/3/5/3 ‖ 41/1/20/0 | 20/5/4/3 ‖ 41/1/22/0 | 25/13/3/4 ‖ 41/0/21/0 |
| S2 T11 | 42 | 7/0/0/0 ‖ 42/0/22/0 | 14/13/2/1 ‖ 42/1/19/0 | 16/17/3/1 ‖ 42/0/15/0 | 22/18/4/3 ‖ 42/0/17/0 |
| S3 EPAL 6 | 42 | 8/0/1/9 ‖ 14/0/25/0 | 16/0/4/9 ‖ 32/0/14/0 | 14/5/5/9 ‖ 35/0/14/0 | 12/9/3/9 ‖ 36/0/14/0 |
| S3 T11 | 42 | 35/0/2/9 ‖ 39/0/25/0 | 36/5/3/10 ‖ 42/0/15/0 | 35/6/2/10 ‖ 42/0/13/0 | 37/8/4/10 ‖ 42/0/12/0 |
| S4 EPAL 6 | 42 | 3/0/0/3 ‖ 42/0/30/0 | 10/2/2/5 ‖ 42/0/21/0 | 8/5/1/6 ‖ 42/1/19/0 | 11/6/1/10 ‖ 42/0/17/0 |
| S4 T11 | 42 | 10/0/1/3 ‖ 42/1/33/0 | 14/2/1/6 ‖ 42/3/20/0 | 17/5/1/5 ‖ 42/2/18/0 | 20/6/2/7 ‖ 42/2/17/0 |
| S5 EPAL 6 | 42 | 9/0/3/2 ‖ 28/0/19/0 | 13/3/7/3 ‖ 35/3/21/0 | 12/7/4/3 ‖ 40/0/21/0 | 15/9/5/3 ‖ 36/1/21/0 |
| S5 T11 | 42 | 10/0/0/3 ‖ 28/2/19/0 | 12/3/1/3 ‖ 38/1/17/0 | 12/11/1/3 ‖ 38/0/15/0 | 20/13/2/4 ‖ 40/1/17/0 |
| S6 EPAL 6 | 42 | 4/0/3/0 ‖ 13/3/11/0 | 8/4/8/0 ‖ 30/0/25/0 | 7/6/7/0 ‖ 32/2/26/0 | 8/6/7/0 ‖ 33/0/27/0 |
| S6 T11 | 42 | 5/0/4/1 ‖ 4/9/3/0 | 9/5/7/1 ‖ 35/3/27/0 | 13/7/9/2 ‖ 34/2/26/0 | 17/10/11/4 ‖ 39/1/29/0 |
| S7 비규격 | 135 | 112/6/113/0 ‖ 117/11/117/0 | 115/8/116/0 ‖ 124/6/124/0 | 117/8/118/0 ‖ 128/3/128/0 | 118/7/118/1 ‖ 128/4/128/0 |
| N 음성 | 65 | 0 ‖ 30 | 0 ‖ 8 | 0 ‖ 7 | 0 ‖ 3 |

- **잡음 없음**에서 `oracle-fit` 은 알려진 형상(S1–S6)의 위험 오위치가 0 이고, 자세 p95 는 EPAL 6 가로 7.0 mm·삽입축 0.2 mm·yaw 0.0°·높이 4.2 mm, T11 가로 5.2·0.2 mm·0.0°·3.7 mm 로 자세 합격선을 넘지 않는다. 놓침은 기준선보다 크게 줄었다(예: S2 EPAL 6 41 → 11, S4 EPAL 6 42 → 3, S4 T11 42 → 10). 그래도 형상별 놓침·대상 선택·재관측 관문은 아직 통과하지 못한다.
- **N 의 0 은 구성상 값**이다 — `oracle-fit` 은 정답 인스턴스(ID ≥ 0)만 제안하므로 방해물·음성 구조물은 후보가 되지 않는다. 배경 억제 성능은 기하 제안기(2B)와 학습 제안기(단계 3)에서 잰다.
- **S7 비규격**은 2A 범위 밖이다(알려진 형상 가설만). 형상 불일치 관문 덕분에 알려진 형상으로 잘못 확정되는 경우가 적지만(잡음 없음 위험 6 장면), 맞추지는 못해 대부분 놓침이다.

## 3. 거리별 자세 오차와 위험 (알려진 형상, 대응된 counted 후보)

칸은 표본 수 · 가로/삽입축(mm)/yaw/높이(mm) p95 · 위험 후보 수. 거리는 정답 진입면(두 포켓 중점)과 카메라의 수평 거리.

| 정답 거리(m) | 잡음 없음 oracle-fit | 잠정 낮음 oracle-fit | 잠정 중간 oracle-fit | 잠정 높음 oracle-fit |
|---|---|---|---|---|
| 0–1.5 | n 113 · 5/0/0.0°/3 · 위험 0 | n 110 · 9/5/0.7°/3 · 위험 0 | n 110 · 9/6/0.9°/3 · 위험 0 | n 105 · 14/9/1.3°/3 · 위험 0 |
| 1.5–2.0 | n 127 · 5/0/0.0°/2 · 위험 0 | n 127 · 12/10/2.0°/3 · 위험 0 | n 126 · 15/13/2.8°/3 · 위험 0 | n 125 · 18/20/3.8°/3 · 위험 1 |
| 2.0–2.5 | n 133 · 6/0/0.0°/2 · 위험 0 | n 123 · 22/18/2.9°/4 · 위험 1 | n 123 · 23/29/3.9°/5 · 위험 3 | n 113 · 27/37/5.3°/6 · 위험 9 |
| 2.5–3.0 | n 152 · 6/0/0.0°/2 · 위험 0 | n 127 · 24/31/4.5°/5 · 위험 5 | n 129 · 30/41/6.1°/5 · 위험 21 | n 120 · 47/62/8.6°/6 · 위험 43 |
| 3.0– | n 222 · 7/0/0.0°/2 · 위험 0 | n 182 · 37/56/8.8°/6 · 위험 48 | n 172 · 42/73/11.6°/6 · 위험 69 | n 149 · 44/97/14.2°/6 · 위험 82 |

**위험 후보는 거의 모두 2 m 밖에서 나온다.** 잠정 중간 잡음에서 위험 후보 94 개 중 90 개가 2.5 m 밖이고 1.5 m 안은 0 개다. 단일 프레임의 원거리 yaw 오차가 날 끝 변위로 번지는 것이고, 계획 전 실측과 Codex 의 원거리 지적(통합 p95 합격선, 설계 개정 필요, 공식 잡음 대기)과 같은 그림이다. 합격선은 바꾸지 않았다.

## 4. 상태·선택·처리 시간

- 잡음 없음: 선택 {'none': 204, 'selected': 311, 'not_pickable': 29, 'ask_operator': 81, 'reobserve': 79}, 사유 {'no_candidate': 68, 'no_candidate_in_radius': 136, 'near_tracking_unavailable': 6, 'multiple_in_radius': 81, 'uncertain_in_radius': 16, 'unresolved_in_radius': 57, 'lift_alignment_needed': 18, 'layer_relative': 6, 'payload_limit': 4, 'fork_spacing_adjust': 2}, 미대응 위험 후보 0, 제안 실패 {'no_face': 89}, 받침면 {'visible': 711, 'uncertain': 19, 'physical_edge': 7, 'coplanar': 123}, 처리 p50/p95 0.14/0.30 s, 자세 p95 EPAL {'pose_lateral_p95': 7.0, 'pose_axial_p95': 0.2, 'pose_yaw_p95': 0.0, 'pose_z_p95': 4.2} T11 {'pose_lateral_p95': 5.2, 'pose_axial_p95': 0.2, 'pose_yaw_p95': 0.0, 'pose_z_p95': 3.7}
  기하 상태(알려진 형상): {('fitted', '-'): 740, ('ambiguous', 'lateral_unconstrained'): 38, ('ambiguous', 'competing'): 37, ('unfit', 'weak_fit'): 33, ('fitted', 'support_uncertain'): 7, ('ambiguous', 'yaw_unconstrained'): 4, ('ambiguous', 'shape_mismatch'): 1}
- 잠정 낮음: 선택 {'none': 244, 'selected': 274, 'not_pickable': 25, 'reobserve': 95, 'ask_operator': 66}, 사유 {'no_candidate': 71, 'no_candidate_in_radius': 173, 'near_tracking_unavailable': 6, 'uncertain_in_radius': 23, 'multiple_in_radius': 66, 'unresolved_in_radius': 62, 'lift_alignment_needed': 13, 'layer_relative': 10, 'payload_limit': 4, 'fork_spacing_adjust': 2}, 미대응 위험 후보 0, 제안 실패 {'no_face': 89}, 받침면 {'visible': 712, 'uncertain': 20, 'physical_edge': 9, 'coplanar': 119}, 처리 p50/p95 0.15/0.32 s, 자세 p95 EPAL {'pose_lateral_p95': 23.6, 'pose_axial_p95': 37.2, 'pose_yaw_p95': 5.0, 'pose_z_p95': 6.3} T11 {'pose_lateral_p95': 25.6, 'pose_axial_p95': 42.0, 'pose_yaw_p95': 6.7, 'pose_z_p95': 3.9}
  기하 상태(알려진 형상): {('fitted', '-'): 662, ('ambiguous', 'lateral_unconstrained'): 101, ('unfit', 'weak_fit'): 53, ('ambiguous', 'competing'): 31, ('fitted', 'support_uncertain'): 7, ('ambiguous', 'shape_mismatch'): 4, ('ambiguous', 'yaw_unconstrained'): 2}
- 잠정 중간: 선택 {'none': 243, 'selected': 274, 'not_pickable': 23, 'reobserve': 99, 'ask_operator': 65}, 사유 {'no_candidate': 74, 'no_candidate_in_radius': 169, 'near_tracking_unavailable': 6, 'uncertain_in_radius': 27, 'multiple_in_radius': 65, 'unresolved_in_radius': 62, 'lift_alignment_needed': 12, 'layer_relative': 10, 'payload_limit': 3, 'fork_spacing_adjust': 2}, 미대응 위험 후보 1, 제안 실패 {'no_face': 92}, 받침면 {'visible': 708, 'uncertain': 21, 'coplanar': 121, 'physical_edge': 7}, 처리 p50/p95 0.15/0.31 s, 자세 p95 EPAL {'pose_lateral_p95': 30.4, 'pose_axial_p95': 50.3, 'pose_yaw_p95': 6.8, 'pose_z_p95': 6.2} T11 {'pose_lateral_p95': 30.0, 'pose_axial_p95': 56.4, 'pose_yaw_p95': 7.5, 'pose_z_p95': 4.4}
  기하 상태(알려진 형상): {('fitted', '-'): 654, ('ambiguous', 'lateral_unconstrained'): 89, ('unfit', 'weak_fit'): 65, ('ambiguous', 'competing'): 31, ('ambiguous', 'shape_mismatch'): 7, ('fitted', 'support_uncertain'): 7, ('ambiguous', 'yaw_unconstrained'): 4}
- 잠정 높음: 선택 {'none': 258, 'selected': 247, 'reobserve': 121, 'not_pickable': 22, 'ask_operator': 56}, 사유 {'no_candidate': 79, 'no_candidate_in_radius': 179, 'unresolved_in_radius': 73, 'near_tracking_unavailable': 6, 'uncertain_in_radius': 33, 'multiple_in_radius': 56, 'layer_relative': 15, 'lift_alignment_needed': 9, 'payload_limit': 4, 'fork_spacing_adjust': 3}, 미대응 위험 후보 0, 제안 실패 {'no_face': 105}, 받침면 {'visible': 695, 'uncertain': 27, 'physical_edge': 6, 'coplanar': 117}, 처리 p50/p95 0.15/0.33 s, 자세 p95 EPAL {'pose_lateral_p95': 39.2, 'pose_axial_p95': 71.2, 'pose_yaw_p95': 8.8, 'pose_z_p95': 6.2} T11 {'pose_lateral_p95': 32.2, 'pose_axial_p95': 69.6, 'pose_yaw_p95': 10.8, 'pose_z_p95': 4.6}
  기하 상태(알려진 형상): {('fitted', '-'): 604, ('unfit', 'weak_fit'): 107, ('ambiguous', 'lateral_unconstrained'): 58, ('ambiguous', 'competing'): 33, ('ambiguous', 'shape_mismatch'): 31, ('fitted', 'support_uncertain'): 8, ('ambiguous', 'yaw_unconstrained'): 4}

- `unresolved_in_radius`(맞춤 실패 물체가 지시 반경 안일 수 있어 재관측)가 잡음 없음에서 57 장면이다 — 대부분 S3 의 가려진 뒤 팔레트(아래 5). Codex 가 짚은 대로 최근접 점 하한으로 판정하므로 보수적이다.
- 받침면은 잡음 없음 기준 `visible` 711, `coplanar` 123(대부분 S4 위층), `uncertain` 19.

## 5. 남은 문제와 결정

- **S3 T11 놓침 35/42.** 앞 팔레트에 가려진 뒤 T11 이 위쪽 몇 cm 만 보여, 마스크를 3 px 깎으면 맞춤 점이 0–30 개만 남는다(31 건 `no_face` 중 점 부족 16·몸통 점 부족 14). 정답 노출은 0.5 이상이라 놓침에 들어간다. 깎기를 2 px 로 줄이면 dev 에서 `fitted` 가 잡음 없음 +14·잠정 중간 +22(늘어난 것은 모두 허용 오차 안) 늘었지만, 고정해 둔 회귀 시험 5 개(관문별 반례 3, 잡음 재판정 1, 배치 탐색 1)의 입력 조건이 바뀌어 **2A 에서는 3 px 를 유지했다**(Ruling — 비용: S3 뒤 팔레트 놓침이 2B 까지 남음). 2B 에서 반례를 다시 고정하며 2 px 를 다룬다. 최소 점 수 60 → 30 은 추가 효과가 없었다.
- **같은 형상의 다른 배치(리뷰 A).** 블록 한 칸 밀린 배치가 0.7173 대 0.7154 로 비슷한데 형상마다 최고 배치 하나만 남겨 286 mm 틀린 위치를 확정하던 것을 고쳤다 — 따로 떨어진 극댓값을 경쟁 가설로 넣어 점수가 가까우면 `ambiguous(competing:<label>@shifted)`. dev 영향은 잡음 없음 S6 T11 놓침 4 → 5 하나였다.
- **원거리 정확도**는 2B 의 다중 프레임 융합과 근거리 재관측(단계 4), 그리고 공식 잡음으로 다시 본다.

## 6. 다음

2B: 비규격 매개변수 가설(S7)·분할 가설·기하 제안기(N·S6 배경 억제가 처음 측정됨)·후보 추적과 다중 프레임 융합, 깎기 2 px 재고정, 얇은 받침 띠 관찰(S4 `uncertain`).
