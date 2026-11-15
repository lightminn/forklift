### riib: R vs 곡률 B0

- a: `/home/projects/forklift/test_runs/pfb/R_pfb_2bea23e5`(mode overlay) · b: `/home/projects/forklift/test_runs/tmp_restore/pfb_prefix/test_runs/lcr/cmp_a2b91d2d`(mode prefix) · 칸은 a / b (a = 이번 기준선)
- 거부 실행: a 0 · b 0

| 장면 | n | 완주 k/n | 원거리 첫 시도 채택 k/n | 근접 최종 상태 | 근접 시도 valid/n | 운반 이탈(추종 실패) | 운반 cusp 재계획 total(실행 수) | 하역 오차 중앙값·p90·최대 mm | 시뮬 시간 중앙값·p90 s | 실패 분류 | 금지 접촉 / 결정성 깨짐 실행 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| random_1 | 60 / 60 | 55/60 / 55/60 | 60/60 / 60/60 | done 59·pending 1 / done 59·pending 1 | 59/59 / 59/59 | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 8(7) / 9(9) | 8.6·20.1·29.2 / 9.5·19.5·29.2 | 205.0·225.8 / 205.0·221.7 | planning 2·tracking 3 / planning 1·tracking 4 | 0 / 0 / 0 / 0 |
| line_2 | 5 / 5 | 5/5 / 5/5 | 5/5 / 5/5 | done 5 / done 5 | 5/5 / 5/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 0(0) / 2(1) | 17.8·22.6·24.9 / 10.0·24.0·26.0 | 171.3·180.0 / 178.0·212.8 | 없음 / 없음 | 0 / 0 / 0 / 0 |
| aisle_rack | 5 / 5 | 5/5 / 4/5 | 5/5 / 5/5 | done 5 / done 4·failed 1 | 5/5 / 4/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 7(4) / 2(2) | 10.3·53.5·53.6 / 21.9·53.4·64.7 | 297.1·307.6 / 276.0·284.2 | 없음 / perception 1 | 0 / 0 / 0 / 0 |
| 합계 | 70 / 70 | 65/70 / 64/70 | 70/70 / 70/70 | done 69·pending 1 / done 68·failed 1·pending 1 | 69/69 / 68/69 | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 15(11) / 13(12) | 8.6·21.2·53.6 / 9.9·20.9·64.7 | 205.0·234.3 / 205.0·228.4 | planning 2·tracking 3 / perception 1·planning 1·tracking 4 | 0 / 0 / 0 / 0 |

#### 짝 차이(완주·끝 단계·실패 분류가 다른 짝)

- 공통 70 · 바뀐 짝 7 · only_a 0 · only_b 0

| 장면 | seed | a 완주 | a 끝 단계 | a 사유(앞 60 자) | b 완주 | b 끝 단계 | b 사유(앞 60 자) |
|---|---|---|---|---|---|---|---|
| aisle_rack | 2 | 완주 | complete | — | 실패 | observe | near_capture_failed:perception_no_pallet:no_expected_candida |
| random_1 | 0 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.9974,yaw=1.0147 |
| random_1 | 11 | 실패 | transport | scene_replan_refused:cusp_pose:blocked:occupied | 완주 | complete | — |
| random_1 | 23 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.2105,yaw=0.6871 |
| random_1 | 24 | 실패 | return_home | Tracking timeout in return_home | 완주 | complete | — |
| random_1 | 40 | 완주 | complete | — | 실패 | transport | scene_replan_refused:cusp_pose:blocked:occupied |
| random_1 | 49 | 실패 | transport | scene_replan_refused:cusp_pose:blocked:occupied | 완주 | complete | — |

### riib: R vs K1 비교 K1(40)

- a: `/home/projects/forklift/test_runs/pfb/R_pfb_2bea23e5`(mode overlay) · b: `/home/projects/forklift/test_runs/tmp_restore/pfb_prefix/test_runs/k1/cmp_c7701296`(mode prefix) · 칸은 a / b (a = 이번 기준선) · 맞댄 짝 40(두 쪽 모두 거부되지 않은 공통 장면·seed; 공통 이름 40, a 에만 30, b 에만 0)
- 거부 실행: a 0 · b 0

| 장면 | n | 완주 k/n | 원거리 첫 시도 채택 k/n | 근접 최종 상태 | 근접 시도 valid/n | 운반 이탈(추종 실패) | 운반 cusp 재계획 total(실행 수) | 하역 오차 중앙값·p90·최대 mm | 시뮬 시간 중앙값·p90 s | 실패 분류 | 금지 접촉 / 결정성 깨짐 실행 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| random_1 | 30 / 30 | 26/30 / 26/30 | 30/30 / 30/30 | done 29·pending 1 / done 29·pending 1 | 29/29 / 29/29 | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 3(3) / 5(5) | 6.5·18.7·25.3 / 9.9·17.2·19.6 | 198.2·225.8 / 205.7·224.0 | planning 1·tracking 3 / tracking 4 | 0 / 0 / 0 / 0 |
| line_2 | 5 / 5 | 5/5 / 5/5 | 5/5 / 5/5 | done 5 / done 5 | 5/5 / 5/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 0(0) / 2(1) | 17.8·22.6·24.9 / 10.0·24.0·26.0 | 171.3·180.0 / 178.0·212.8 | 없음 / 없음 | 0 / 0 / 0 / 0 |
| aisle_rack | 5 / 5 | 5/5 / 4/5 | 5/5 / 5/5 | done 5 / done 4·failed 1 | 5/5 / 4/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 7(4) / 2(2) | 10.3·53.5·53.6 / 21.9·53.4·64.7 | 297.1·307.6 / 276.0·284.2 | 없음 / perception 1 | 0 / 0 / 0 / 0 |
| 합계 | 40 / 40 | 36/40 / 35/40 | 40/40 / 40/40 | done 39·pending 1 / done 38·failed 1·pending 1 | 39/39 / 38/39 | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 10(7) / 9(8) | 8.2·22.2·53.6 / 11.7·20.4·64.7 | 198.2·267.0 / 206.4·255.2 | planning 1·tracking 3 / perception 1·tracking 4 | 0 / 0 / 0 / 0 |

#### 짝 차이(완주·끝 단계·실패 분류가 다른 짝)

- 공통 40 · 바뀐 짝 5 · only_a 30 · only_b 0

| 장면 | seed | a 완주 | a 끝 단계 | a 사유(앞 60 자) | b 완주 | b 끝 단계 | b 사유(앞 60 자) |
|---|---|---|---|---|---|---|---|
| aisle_rack | 2 | 완주 | complete | — | 실패 | observe | near_capture_failed:perception_no_pallet:no_expected_candida |
| random_1 | 0 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.9974,yaw=1.0147 |
| random_1 | 11 | 실패 | transport | scene_replan_refused:cusp_pose:blocked:occupied | 완주 | complete | — |
| random_1 | 23 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.2105,yaw=0.6871 |
| random_1 | 24 | 실패 | return_home | Tracking timeout in return_home | 완주 | complete | — |

### riib: R vs K1 재확인 K1(40)

- a: `/home/projects/forklift/test_runs/pfb/R_pfb_2bea23e5`(mode overlay) · b: `/home/projects/forklift/test_runs/tmp_restore/pfb_prefix/test_runs/k1r/k1r_7744f285`(mode overlay-noheader) · 칸은 a / b (a = 이번 기준선) · 맞댄 짝 40(두 쪽 모두 거부되지 않은 공통 장면·seed; 공통 이름 40, a 에만 30, b 에만 0)
- 거부 실행: a 0 · b 0

| 장면 | n | 완주 k/n | 원거리 첫 시도 채택 k/n | 근접 최종 상태 | 근접 시도 valid/n | 운반 이탈(추종 실패) | 운반 cusp 재계획 total(실행 수) | 하역 오차 중앙값·p90·최대 mm | 시뮬 시간 중앙값·p90 s | 실패 분류 | 금지 접촉 / 결정성 깨짐 실행 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| random_1 | 30 / 30 | 26/30 / 26/30 | 30/30 / 30/30 | done 29·pending 1 / done 29·pending 1 | 29/29 / 29/29 | cross_track 1·endpoint_heading 0 / cross_track 1·endpoint_heading 0 | 3(3) / 3(3) | 6.5·18.7·25.3 / 6.5·18.7·25.3 | 198.2·225.8 / 198.2·225.8 | planning 1·tracking 3 / planning 1·tracking 3 | 0 / 0 / 0 / 0 |
| line_2 | 5 / 5 | 5/5 / 5/5 | 5/5 / 5/5 | done 5 / done 5 | 5/5 / 5/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 0(0) / 0(0) | 17.8·22.6·24.9 / 17.8·22.6·24.9 | 171.3·180.0 / 171.3·180.0 | 없음 / 없음 | 0 / 0 / 0 / 0 |
| aisle_rack | 5 / 5 | 5/5 / 5/5 | 5/5 / 5/5 | done 5 / done 5 | 5/5 / 5/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 7(4) / 7(4) | 10.3·53.5·53.6 / 10.3·53.5·53.6 | 297.1·307.6 / 297.1·307.6 | 없음 / 없음 | 0 / 0 / 0 / 0 |
| 합계 | 40 / 40 | 36/40 / 36/40 | 40/40 / 40/40 | done 39·pending 1 / done 39·pending 1 | 39/39 / 39/39 | cross_track 1·endpoint_heading 0 / cross_track 1·endpoint_heading 0 | 10(7) / 10(7) | 8.2·22.2·53.6 / 8.2·22.2·53.6 | 198.2·267.0 / 198.2·267.0 | planning 1·tracking 3 / planning 1·tracking 3 | 0 / 0 / 0 / 0 |

#### 짝 차이(완주·끝 단계·실패 분류가 다른 짝)

- 공통 40 · 바뀐 짝 0 · only_a 30 · only_b 0

(바뀐 짝 없음)

### riib: R vs S(40)

- a: `/home/projects/forklift/test_runs/pfb/R_pfb_2bea23e5`(mode overlay) · b: `/home/projects/forklift/test_runs/pfb/S_pfb_2bea23e5`(mode stage) · 칸은 a / b (a = 이번 기준선) · 맞댄 짝 40(두 쪽 모두 거부되지 않은 공통 장면·seed; 공통 이름 40, a 에만 30, b 에만 0)
- 거부 실행: a 0 · b 0

| 장면 | n | 완주 k/n | 원거리 첫 시도 채택 k/n | 근접 최종 상태 | 근접 시도 valid/n | 운반 이탈(추종 실패) | 운반 cusp 재계획 total(실행 수) | 하역 오차 중앙값·p90·최대 mm | 시뮬 시간 중앙값·p90 s | 실패 분류 | 금지 접촉 / 결정성 깨짐 실행 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| random_1 | 30 / 30 | 26/30 / 25/30 | 30/30 / 29/29 | done 29·pending 1 / done 28·pending 1 (결측 near_capture 1) | 29/29 / 28/28 (결측 near_capture 1) | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 3(3) / 5(5) | 6.5·18.7·25.3 / 9.5·17.5·19.6 | 198.2·225.8 / 206.4·224.4 | planning 1·tracking 3 / other 1·tracking 4 | 0 / 0 / 0 / 1 |
| line_2 | 5 / 5 | 5/5 / 5/5 | 5/5 / 5/5 | done 5 / done 5 | 5/5 / 5/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 0(0) / 2(1) | 17.8·22.6·24.9 / 10.0·24.0·26.0 | 171.3·180.0 / 178.0·212.8 | 없음 / 없음 | 0 / 0 / 0 / 0 |
| aisle_rack | 5 / 5 | 5/5 / 4/5 | 5/5 / 5/5 | done 5 / done 4·failed 1 | 5/5 / 4/5 | cross_track 0·endpoint_heading 0 / cross_track 0·endpoint_heading 0 | 7(4) / 2(2) | 10.3·53.5·53.6 / 21.9·53.4·64.7 | 297.1·307.6 / 276.0·284.2 | 없음 / perception 1 | 0 / 0 / 0 / 0 |
| 합계 | 40 / 40 | 36/40 / 34/40 | 40/40 / 39/39 | done 39·pending 1 / done 37·failed 1·pending 1 (결측 near_capture 1) | 39/39 / 37/38 (결측 near_capture 1) | cross_track 1·endpoint_heading 0 / cross_track 3·endpoint_heading 0 | 10(7) / 9(8) | 8.2·22.2·53.6 / 11.0·20.5·64.7 | 198.2·267.0 / 206.4·258.4 | planning 1·tracking 3 / other 1·perception 1·tracking 4 | 0 / 0 / 0 / 1 |

#### 짝 차이(완주·끝 단계·실패 분류가 다른 짝)

- 공통 40 · 바뀐 짝 5 · only_a 30 · only_b 0

| 장면 | seed | a 완주 | a 끝 단계 | a 사유(앞 60 자) | b 완주 | b 끝 단계 | b 사유(앞 60 자) |
|---|---|---|---|---|---|---|---|
| aisle_rack | 2 | 완주 | complete | — | 실패 | observe | near_capture_failed:perception_no_pallet:no_expected_candida |
| random_1 | 0 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.9974,yaw=1.0147 |
| random_1 | 11 | 실패 | transport | scene_replan_refused:cusp_pose:blocked:occupied | 완주 | complete | — |
| random_1 | 23 | 완주 | complete | — | 실패 | transport | Tracking failed in transport: pos=2.2105,yaw=0.6871 |
| random_1 | 24 | 실패 | return_home | Tracking timeout in return_home | 실패 | observe | frame_sync: no frame at 1.016667 s after 20 renders |

### 결정성 반복

| 짝 | 결과 | 첫 차이(또는 기록 불완전 사유) | 차이 수 | broken 수 | 기록 불완전 수 |
|---|---|---|---|---|---|
| rep_random_1_s0 | 같음 | — | 0 | 0 | 0 |
| rep_random_1_s1 | 같음 | — | 0 | 0 | 0 |
| rep_line_2_s0 | 같음 | — | 0 | 0 | 0 |
| rep_aisle_rack_s0 | 같음 | — | 0 | 0 | 0 |
