# ① 적재 운반 계획 곡률 상한 규칙 — 140 실행 비교 결과 (2026-11-14)

- 계획: `docs/plans/2026-11-14-loaded-curvature-rule.md`(#505). 판정 규칙은 그 계획의 5 절이고, 결과를 보기 전에 고정했다.
- 구현: #506(a2b91d2d). `--loaded-curvature-rule config/loaded_curvature_rule.yaml` 은 k 0.90 × κ_sat 0.300 = 0.270 /m 이다. 기본은 끔이다.
- 판정 도구: `2026-11-14-loaded-curvature-comparison/lcr_judge.py`(시험 `tests/unit/tools/test_lcr_judge.py`). 결과 전에 Codex 검토를 받았다(새 P1/P2 없음, P3 반영). 출력은 `2026-11-14-loaded-curvature-comparison/lcr_judge.json` 이다.
- **판정: "더 나쁨".** 완주(3)와 비회귀(6)가 통과하지 못했고, 운반 계획 실패(4)도 통과하지 못했다.
- **총괄 결정(2026-11-14):** 등록 판정을 그대로 두고 규칙을 승격하지 않는다. 옵션 기본 끔을 유지한다. 곡률만 낮추면 경로가 더 꺾이는 맞바꿈이 생긴다는 결과로 기록한다.

## 1. 실행

- 스냅숏 `snapshots/lcrcmp_a2b91d2d`. 두 팔이 같은 스냅숏을 썼다.
- ws1 잡 19036·19037: riib 묶음, 잡마다 3 슬롯(shard 4 · CPU 8 · 20G), claim-dir 공유, `--retries 2`. 팀원 잡 19032 뒤에 의존으로 걸었고, 2026-11-14 06:51–10:01 KST 에 돌았다.
- 공통 인자: riib 런처 기본(K1 `--near-observe-arrival 0.05,0.05` 포함) + `--deterministic`, VIDEO=off.
- 팔: B0 = 공통 인자, R = B0 + `--loaded-curvature-rule`.
- 장면·seed: random_1 0–59, line_2 0–4, aisle_rack 0–4. 70 짝, 140 실행.
- 유효성:
  - 140/140 끝났고, 인프라 실패 0, `deterministic.broken` 0.
  - 팔 신원: R 만 규칙 기록이 있고, R 의 운반 경로 최대 |κ| 는 0.27 이다.

## 2. 판정(계획 5 절)

| 항목 | 기준 | 결과 | 판정 |
|---|---|---|---|
| 1 안전 | R 에서 새로 생긴 금지 접촉·SCENE 실패 0 | 두 팔 0 | 통과 |
| 2 운반 cross_track(random_1) | R ≤ ⌊B0/2⌋, B0 < 4 면 노출 부족 | B0 3 · R 0 | 노출 부족(B0 3) |
| 3 완주 | 전체 R ≥ B0 − 1, random_1 R ≥ B0 − 1 | 전체 B0 64 · R 62, random_1 55 · 56 | **불통과**(전체) |
| 4 운반 계획 실패 | 전체 R ≤ B0 + 1, line_2·aisle_rack 새 실패 0 | B0 0 · R 3(모두 random_1), 좁은 장면 새 실패 0 | **불통과** |
| 5 시간 | 두 팔 완주 짝의 운반 시간 비 ≤ 1.15, 사이클 비 ≤ 1.10 | 58 짝, 0.962 · 0.998 | 통과 |
| 6 비회귀 | line_2·aisle_rack 상태·실패 분류가 같음 | 다른 짝 3(line_2 s3, aisle_rack s0·s1) | **불통과** |

- 판정 순서(1·3·6 → 노출 부족 → 2·4·5)대로 "더 나쁨" 이다.

### 2.1 두 팔이 갈라진 짝

| 짝 | B0 | R | 갈라진 곳 |
|---|---|---|---|
| random_1 s0·s8·s23 | 운반 cross_track | 완주 | 운반 |
| random_1 s40 | 장면 재계획 거부(cusp_pose occupied) | 완주 | 운반 |
| random_1 s10·s14·s19 | 완주 | `scene_replan_failed:permit_stop:straight_collision` | 운반 |
| line_2 s3 | 완주 | `Tracking timeout in transport` | 운반 |
| aisle_rack s0·s1 | 완주 | `near_capture_failed:perception_invalid:opening_width_mismatch` | **운반 전(근접 캡처)**, 3 절 |

- R 의 운반 실패 네 건(random_1 s10·s14·s19, line_2 s3)은 모두 같은 꼴이다.
  - 장애물 층이 앞 경로를 점유로 보고 멈췄다(`permit_stop`, why `occupied`, 3 s 유지).
  - 장면 재계획이 실패하거나(straight_collision) 운반 시간 한도를 넘었다.
  - 운반 경로의 앞뒤 전환은 3·2·3·3 번이었다.

## 3. 두 팔 간 근접 캡처 깊이 차이: 계획 경로 표시선 영향 확인

- 두 짝은 운반 전, 근접 캡처에서 갈라졌다.
  - 근접 waypoint, 캡처 시각(53.092 s 등), 캡처 때 참값 뒷축 자세(s0: 0.35285, −4.09012, 0.05262)는 두 팔이 같다.
  - 근접 캡처의 depth 가 다르다(s0: `6a6b54af…` 대 `cf46307b…`). 첫 캡처(원거리)의 depth 는 같다.
- **재실행(총괄 승인, 잡 19041, 같은 스냅숏, 4 실행)은 팔마다 원래 결과를 그대로 재현했다.**
  - 결과(B0 완주, R 근접 캡처 실패)와 근접 캡처 depth 해시가 팔마다 원래와 같다.
  - 렌더 횟수(`frame_sync.renders`)와 센서 프레임 번호는 실행마다 달랐다(예: B0 s0 499 → 628, 프레임 830 → 957). 그래도 depth 는 같았다.
  - 그래서 결정 모드의 렌더 비결정성이 아니다. 같은 입력이면 같은 depth 다.
- **두 팔의 장면 차이는 계획 경로 표시선이다.**
  - `sim/isaac/scene.py` 의 `add_path_display` 는 계획 경로를 렌더되는 `UsdGeom.BasisCurves` 로 그린다(`/World/PlannedPaths/…`, 바닥 위 0.012 m, 폭 0.018 m).
  - 러너는 미션 계획 직후 접근·운반(·복귀) 경로를 그린다(`run_transport.py` 6557–6560). 미션 코어는 `show_path` 포트로 재계획 경로를 그린다.
  - 인식 카메라에서 이것을 빼는 설정·코드는 없다.
  - R 은 미션 계획의 운반 경로가 0.27 로 달라서 다르게 그려진다.
- **원래 실행의 영상 차분(근접 캡처):**
  - 차이가 행 261–317 의 가로 띠, 전체 폭에 몰렸다(s0 4,137 px, s1 8,339 px). s0 에서는 그 픽셀의 91 % 에서 R 이 더 가깝다.
  - RGB 영상에서 R 에는 운반 경로의 주황 표시선(색 1.0, 0.65, 0.04)이 팔레트 앞 바닥을 가로지른다. B0 에는 오른쪽 끝에 짧은 선이 보인다.
  - 그래서 바닥 쪽 광선 수가 바뀌었다(s0 왼쪽 opening_rays behind 529 → 1274, opening_width_raw 0.23 → 0.65 m). 그 결과 opening_width_mismatch 가 났다.
- **표시를 끈 확인(총괄 승인, 잡 19043):**
  - 진단 스냅숏 `diag/path-display-off`(27daf0b0, `FORKLIFT_PATH_DISPLAY=off` 이면 `add_path_display` 를 건너뜀, 머지하지 않음)로 aisle_rack s0·s1 × B0/R 4 실행을 돌렸다.
  - 두 팔의 근접 캡처 depth 가 **완전히 같아졌다(차이 0 px)**. 두 팔 모두 캡처 valid 다.
  - 그때 R 은 s1 완주, s0 운반 실패(cusp 에서 endpoint_heading, yaw 0.0714 rad)였다. 표시가 없으면 이 두 짝의 R 결과는 운반 단계에서 갈린다.
  - B0 의 근접 캡처 depth 도 표시를 끄면 달라졌다(s0 `6a6b54af` → `9bb0db71`). 표시선은 비교의 두 팔 차이뿐 아니라 모든 실행의 인식 입력에 들어가 있다.
- 뜻:
  - 두 팔의 근접 캡처 depth 차이는 표시선으로 설명된다(표시를 끄면 차이 0 px). 렌더 비결정성은 아니다.
  - 원래 실행의 근접 캡처 실패가 표시선만의 탓인지, 표시를 끄면 R 결과가 나아지는지는 이 확인으로 정해지지 않는다. 표시를 끈 R 은 s1 완주·s0 운반 실패였다.
  - 등록 판정은 바꾸지 않는다(총괄 결정).
  - 이 결함은 별도로 다룬다. 인식 렌더에서 표시선을 빼는 수정 계획, 그리고 최근 결론들의 영향 조사다(총괄 배정).
  - LiDAR 는 PhysX 레이캐스트라 충돌체 없는 표시선을 보지 못한다(`planar_lidar.py`). SLAM·장애물 층은 영향이 없다.

## 4. 기전과 기록(판정 아님)

- **포화는 줄었다.** random_1 두 팔 완주 52 짝에서 운반 단계 명령 곡률 포화 비율의 중앙값은 B0 0.539 → R 0.399 다. R 이 더 낮은 짝은 50/52 다.
  - 포화 비율은 운반 단계 samples 가운데 속도 > 0.05 m/s 인 표본에서 |κ_cmd| ≥ 0.355 인 표본의 비율이다.
  - ws1 원본 samples 로 셌다. 축약본에는 samples 가 없어 이 축약본으로는 다시 계산할 수 없다.
  - 계획 2 절의 반대 가능성(pure pursuit 가 먼저 포화해 효과가 없을 수 있음)은 이 지표로는 나타나지 않았다.
  - B0 의 cross_track 3 건(s0·s8·s23)은 R 에서 모두 완주했다.
- **경로가 더 꺾였다.** 처음 계획한 운반 경로(배송 직선으로 바뀌기 전, `paths_replaced` 의 첫 transport 항목. 없으면 `paths.transport`; 끝난 실행의 `paths.transport` 는 0.66 m 배송 직선이다)에서 다음을 셌다.
  - 앞뒤 전환 합: B0 233 → R 339(+45 %).
  - 길이 중앙값: 15.0 → 15.5 m.
  - 회전 반경이 커지자 계획기가 전환을 더해 같은 장면을 지나간다.
  - R 의 새 운반 실패 네 건은 모두 장애물 층의 점유 정지 뒤였다. 전환이 늘고 경로 모양이 바뀐 것이 점유 판정과 장면 재계획 실패로 이어졌을 것으로 추론한다. 경로별 대조는 하지 않았다.
- cusp 재계획: B0 13 · R 11. 운반 endpoint_heading: 0 · 0.
- 시간: 두 팔 완주 짝에서 운반 시간은 R 이 약간 짧았다(비 0.962). 포화가 줄어 운반 속도가 덜 떨어진 것으로 보인다(추론).

## 5. 한계

- "더 나쁨" 은 등록한 판정 규칙의 결과다. 곡률 규칙의 일반적인 효과나 원인에 대한 통계적 결론이 아니다.
- 결정 모드, riib 기본, 이 장면들에 한정한다. random_1 B0 의 cross_track 이 3 건이라 등록 기준으로는 효과가 노출 부족이다.
- 모든 실행에 경로 표시선이 인식 입력으로 들어가 있었다(3 절). 그 몫은 aisle_rack 두 짝에서만 확인했다.
- k 는 0.90 하나만 등록해 시험했다. 다른 k(예: 0.95)나 방향별 상한은 시험하지 않았다.

## 6. 원자료와 재현

- `assets/loaded-curvature-comparison/`:

| 파일 | 내용 | md5 |
|---|---|---|
| `lcr_fields.json.gz` | 140 실행의 result.json 가운데 판정이 읽는 필드 | 0f3fa3acf7c661e70c93cbab4fe8d315 |
| `RUNS_cmp_a2b91d2d.jsonl` | 실행 목록(스냅숏·런처·환경·EXTRA_ARGS) | b5a8f3f8d50fb693d012bcb13c0e1fb7 |

- 판정 재현: `python docs/validation/2026-11-14-loaded-curvature-comparison/lcr_judge.py --raw-fields docs/validation/assets/loaded-curvature-comparison/lcr_fields.json.gz --out LCR.json`(forklift_core 설치 필요). 원본 폴더에 돌린 출력과 판정·기록이 같다.
- 원본: ws1 `test_runs/lcr/cmp_a2b91d2d/`(140), `test_runs/lcr/rep_a2b91d2d/`(재실행 4), `test_runs/lcr/pdoff_27daf0b0/`(표시 끔 4), 스냅숏 `snapshots/lcrcmp_a2b91d2d`·`snapshots/pdoff_27daf0b0`. 이 문서가 머지되면 정리 목록으로 넘긴다.
- 포화 비율(4 절)은 ws1 원본 samples 에서 셌다. 축약본에는 samples 가 없다.

## 7. 검토

- Codex 새 스레드, 축약본에서 독립 재계산, 1 회차. 2 절 표와 2.1 은 일치했다. 처리:
  - P2 표시선 원인 귀속이 증거보다 강함: 그 뒤 표시를 끈 확인(잡 19043, 근접 depth 차이 0 px)과 영상 차분을 했다. 3 절을 확인 결과로 다시 썼다.
  - P2 경로 길이 정의: 처음 계획한 운반 경로(`paths_replaced` 의 첫 transport)라고 적었다. 끝난 실행의 `paths.transport` 는 배송 직선이다.
  - P3 포화 비율 재현 불가: 정의와 "축약본으로 재현 불가" 를 4 절에 적었다.
  - P3 판정의 뜻: 5 절에 "등록 판정이지 통계적 결론이 아님" 을 적었다.
- 2 회차. 처리:
  - P2 3 절의 결론이 표시 끔 확인보다 강함: "두 팔 깊이 차이는 표시선으로 설명되며, 근접 캡처 실패가 표시선만의 탓인지는 정해지지 않음" 으로 좁혔다.
  - P3 3 절 제목: "두 팔 간 근접 캡처 깊이 차이: 계획 경로 표시선 영향 확인" 으로 고쳤다.
