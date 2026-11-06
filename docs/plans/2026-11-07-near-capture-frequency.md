# 라이브 근접 캡처 종결 빈도 측정 (2026-11-07, 5판)

총괄 결정 D(2026-11-07)다. 라이브 런처 비교(O0 대 O6)는 하지 않고, 운영 설정 O0 한 팔로 근접 캡처 종결 빈도만 잰다. 이 숫자는 A2 재개 여부(#337·#338)라는 실제 결정을 바꾼다.

## 1. 결정과 사실

- **라이브 런처 승격은 보류한다. 런처는 그대로 둔다.** 사실로 기록한다.
  - 여섯 인자(`--exit-prefix-exempt on --low-clearance-guard on --restart-confirm-scans 3 --return-start-backoff boxed --near-end-stand on --return-cusp-replans 2`)는 현황 조사(`cts_dfa9aaf2`)의 여섯 켠 24 세션 71 시작 임무에서 **제어 개입이 하나도 관측되지 않았고**(아래 표), 그래서 이 두 장면에서 동작 효과는 검증되지 않았다. 24 세션 가운데 20 은 오라클 인식이고 학습 인식은 4 뿐이다. `no_prefix` 는 장면이 정하지만 나머지는 실행 상태에 따라 발동하므로, 앞으로의 seed 에서도 작동하지 않는다는 뜻은 아니다. 비교 배치를 돌려도 작동 수가 적으면 효과를 가를 수 없다는 판단이다.
  - 승격하면 `SCENE_FILE` 조건이 필요하다. `--exit-prefix-exempt on` 은 `--layout scene`·`--obstacle-layer` 없이, 또는 `--near-tracking` 과 함께면 러너가 기동을 거부한다.
  - **재개 계기:** 라이브에서 인자가 작동하는 장면(`aisle_rack`·`trailer` 등)을 돌릴 필요가 생길 때. 그때 별도 계획으로 세션 장면·조사 지도를 만든다.

| 인자 | 작동(24 세션) | 기록 |
|---|---|---|
| `--exit-prefix-exempt` | 0 | `state.exit_prefix_exempt` 70 항목 모두 `no_prefix` |
| `--low-clearance-guard` | 0 | `low_clearance` 제한 틱 0·정지 틱 0, 다리 858 평가, 최소 여유 0.188 m |
| `--restart-confirm-scans` | 0 | `obstacle_layer.restart_confirm` arms 0·gate_ticks 0 |
| `--return-start-backoff` | 0 | `state.return_start_backoffs` |
| `--near-end-stand` | 0 | `state.observe_end_stands`(near)·`observe_replans` |
| `--return-cusp-replans` | 0 | `state.return_cusp_replans`·`return_cusp_waits` |

## 2. 배치

- **팔:** O0 한 팔. 현재 운영 설정 그대로이고 `MORE` 없음. 학습 인식, 라이브 대기열, 등록 지도 위치 추정(`MAP_BUNDLE=registry`), 알려진 출발, `MAX_EPOCHS=1`, 근접 재관측 `waive_far_confirmed`. **O0 의 합격 판정은 두지 않는다.** 임무 결과와 집계, 재검토 계기만 낸다.
- **세션:** 20. `scenes_ct1_a770031` 의 `line_m_session`(P-1:D-1, P-2:D-2, P-3:D-3)과 `random_m_session`(P-4:D-1, P-5:D-2, P-3:D-3) 각 10, seed 100–109(조사가 쓴 값과 겹치지 않는다). 예정 임무 60.
- **크기:** `tools/isaac_bundle.py` live 묶음(3 슬롯, shard 4, CPU 8) 두 잡, 디스크 약 3.5 GB(조사 실측 28 세션 5.0 GB). `/home` 은 2026-11-07 기준 38 GB 남았다.
- **시간:** 총괄 추정 1.3 시간은 두 잡이 동시에 도는 값이다. 지금 ws1 은 VM 이 켜져 노드가 CPU 8·30 GB·GPU 1·shard 4 라 묶음이 하나씩 돌고, 약 2.8 시간(7 차례 × 약 24 분)이다. 잡을 둘로 나눠 두 잡 사이에 대기 중인 시험이 끼어들 틈을 둔다. 잡이 CPU 8 을 다 쓰므로 그동안 다른 잡은 대기한다. VM 이 꺼지면 두 잡이 함께 돈다.
- **형상:** EPAL 6 뿐이다. 라이브 런처가 EPAL 6 을 고정한다. T11 은 `missions_started: 0`, 표에는 '미실행·미검증'으로 적는다.
- **조사 세션과 합치지 않는다.** 조사 O0 4 세션(12/12)·O6(10/11)은 스냅숏이 다르고(`cts_dfa9aaf2` 뒤 #335·#337 이 러너·탐침·검출기를 고쳤다, 기본 동작은 켜지 않은 옵션이지만 바이트가 다르다), O6 는 설정도 다르다. 재검토 계기의 N 은 이 배치만으로 낸다. 뒤에 같은 `condition_sha256` 의 새 자료가 생기면 누적한다.
- **결과 전 동결:** 실행 목록(sha256), 스냅숏 SHA, 래퍼 `ct1_queue.sbatch`·`isaac_live.sbatch` 해시, 학습 모델·임계값, 장면 해시, 지도 ID. `isaac_bundle.py check` 를 통과해야 제출한다. 집계 함수는 데이터를 읽기 전에 Codex 수렴 상태로 고정한다.

**기반 실패:** 시도마다 셋으로 나눈다. 빼는 것은 '제출 전 종료 확인'뿐이고, 흔적이 없다는 것만으로 접수 전으로 보지 않는다.

- 접수 확인(접수 ack, `mission_results` 항목, `missions/` 폴더, 임무 전이 중 하나): 유효.
- 제출 전 종료 확인(묶음 시작 실패 사유 `startup_silent`·`startup_viewport`, 슬롯 폴더 없음, 탐침의 제출 전 연결 실패이면서 위 흔적 없음): 제외하고 그 슬롯만 재시도한다. 합계 4 회까지.
- 접수 불명(그 밖 전부): 유효로 두고, 도중이던 임무는 `start_unknown` 이다(세션당 최대 1). 시작이 확인됐는데 결과만 없는 `result_missing` 과 구분한다.

**결과 비열람:** 집계 도구 PR 머지 전에는 임무 결과를 보지 않는다. 묶음 `results.jsonl` 의 상태(`early_mission_failure` 등)와 `missions` 칸(완료 수, 첫 실패 사유)도 임무 결과다.

- 허용 감시는 `squeue`·`sacct` 의 잡 상태, `du`·`df`, 그리고 아래 필터 출력뿐이다.
- 필터 `nf_monitor.py`(배치 폴더, 집계 도구 PR 에 함께 커밋, sha256 `7e7e5fc82ec4b3131c390fbed57d06329ac8237f94d702997e5a97542daba17a`)는 `results.jsonl` 의 플래그 `startup_silent`·`startup_viewport`·`startup_crash` 와 상태 `start_failed` 로 제출 전 종료를 가리고, 그 개수·그 밖의 개수·재시도할 실행 이름만 찍는다. 그 밖 안의 상태·사유·완료 수는 찍지 않는다.
- 잡이 비정상 종료하면 스케줄러 로그부터 본다. 임무 기록을 먼저 봐야 진단되는 경우는 그 사실을 결과 문서에 적고, 집계 규칙은 바꾸지 않는다.

## 3. 집계 — `near_capture_tally/1`

콘솔 ③ 과 공유한다(`tools/ct_survey.py` 의 같은 함수). 이 값은 **이 배치의 조건부 관측률**이고, 같은 세션 임무는 독립이 아니다.

**세 비율**(총괄 결정):

| 지표 | 분자 | 분모 |
|---|---|---|
| 근접 종결률 | 근접 캡처로 끝난 고유 임무 k | 실제 시작한 임무 N |
| 근접 도달 조건부 종결률 | k | 근접 캡처에 도달한 고유 임무 R |
| 세션 멈춤률 | 근접 캡처로 멈춘 세션 | 임무를 시작한 세션 |

뒤 미제출 임무 수는 따로 적는다. 비율에는 양측 95 % Wilson 구간을 붙이되 참고값이다(0/60 이면 상한 6.0 %, 1/60 이면 8.9 %).

- **임무 종결:** `reason` 이 `near_capture_` 로 시작(`failures.PREFIXES` 의 한 분류). 검출 실패 `near_capture_failed:`, 수용 거절 `near_capture_misaligned:`, 직선 없음 `near_capture_no_final_straight` 등 근접 캡처에서 끝난 임무 전체다. 평면이 틀어져 `valid` 뒤 거절된 임무가 빠지지 않게 한다. `ended_by_reason` 은 `status:reason` 수준으로 나누고 `no_expected_candidate` 는 그 안의 보조 수다. 재시도·면제 재검출은 분자를 늘리지 않는다.
- **근접 도달:** 근접 예상 자리 시도(`expected`)가 있거나, `near_capture` 상태가 시작됐거나, 임무 종결이 `near_capture_` 인 임무. 그래서 k 는 R 의 부분집합이다. `ended_without_capture` 는 사유 문자열이 아니라 그 임무의 근접 시도 기록이 없는 경우로 판별한다(`near_capture_no_final_straight` 는 캡처 전에도 후에도 난다). 분자에는 그대로 들어간다. 재검출 진단 대상은 근접 시도가 있고 저장 파일의 깊이 sha256 이 시도 기록과 맞는 임무다.
- **단계별 수:** 캡처 시도, 첫 호출 비valid(면제가 성공하면 `lower_waiver.first` 에서 읽음), 면제 재검출 실행·구제, 최종 비valid(`status:reason` 별). 후보 평면별 거절 사유는 실패 시도 수가 아니므로 따로 둔다.
- **회복(분자에 넣지 않음):** 같은 캡처의 면제 구제와 새 캡처로 넘어간 회복을 나눈다.

**두 수준.** 칸은 장면을 나누어 보여 주는 단위이고, 비율과 재검토 계기는 운영 설정(`operating_config_sha256`) 단위로 합산한다. 두 장면은 다른 조건이 아니라 같은 운영 설정의 사전 지정 모집단이기 때문이다(#338 의 (a) 는 '운영 팔'의 임무 수다). 합산 값 옆에는 항상 장면별 값을 적고, 분자가 한 장면에 몰렸는지 밝힌다. 모집단이 두 장면이므로 값의 일반화 범위도 두 장면이다.

**칸:** 형상(`arguments.pallet_prior_loaded.catalogue_version`) × 팔 × 위치 검사 모드(`arguments.pose_check`, 없으면 off) × 장면 × seed × 임무 순서 × 주입(`reloc_inject`, 없으면 none). 칸마다 `condition_sha256`, `operating_config_sha256`, `run_config_sha256s`, 진입면(참 팔레트 +x/−x). 진입면은 `spool/state.samples` 의 표본에서 읽는다(`result.json` 상위 표본은 비워진다). 표본이 없으면 `unknown` 이다. 그래서 디스크 세션은 집계가 끝날 때까지 이 배치의 `spool` 을 지우지 않는다.

- `condition_sha256`: 기록된 러너 인자(`result.json` `arguments`)에서 실행별 값 `output`·`seed`·`slam_noise_seed`·`live_port`·`live_port_file`·`slam_feedback`·`slam_map_dir` 을 뺀 해시다. 제외 목록은 도구에 고정하고 새 인자는 포함한다. `observation_waypoints` 는 임무마다 덮어 쓰이는 장면 값이라(대기열이 어디까지 갔는지에 따라 기록이 다르다), 기록값이 `scene_file` 의 어느 팔레트 값과 같을 때만 뺀다. 다르거나 장면 파일을 못 읽으면 명시한 설정으로 보고 해시에 남긴다. 칸은 이 값이 다르면 합치지 않는다.
- `operating_config_sha256`: 거기서 장면에 묶인 인자(`scene_file`, `slam_map_bundle`, `expected_map_id`, `slam_choice`)를 더 뺀 해시다. 임무 순서 목록은 러너 인자가 아니다. 합산은 이 값이 같고 형상·팔·위치 검사·주입이 같을 때만 한다. 조사 28 세션의 `arguments` 에서 달라지는 키는 위 목록, 이 네 키, 팔·인식 설정(여섯 인자, `deterministic`, `recognition_proposals`, `recognition_pixel_lag_steps`)뿐이었다. 제출 전 확인 결과는 7 장에 적는다. 도구 시험은 이 기록의 값을 고정 입력으로 쓴다.

```json
{"schema": "near_capture_tally/1", "condition_sha256": "…", "operating_config_sha256": "…", "run_config_sha256s": [], "shape": "epal6", "entry_face": "+x",
 "arm": "O0", "pose_check": "off", "scene": "random_m", "seed": 103, "mission_seq": 2, "reloc_inject": "none",
 "missions_started": 0, "start_unknown": 0, "missions_reached_near": 0, "missions_ended_near": 0, "ended_without_capture": 0,
 "ended_by_reason": {"no_expected_candidate": 0}, "ended_session_ids": [],
 "capture_attempts": 0, "first_call_invalid_by_reason": {}, "final_invalid_by_reason": {},
 "waiver_second_calls": 0, "waiver_rescues": 0, "missions_recovered_same_capture": 0,
 "missions_recovered_new_capture": 0, "reached_unknown": 0, "result_missing": 0}
```

세션 집계는 `mission_seq` 없는 키의 별도 객체다(`near_capture_session_tally/1`: `started_session_ids`, `stopped_near_session_ids`, `unsubmitted_after_stop`). 합칠 때는 ID 합집합을 쓴다.

**기록 읽기:**

- 시작 임무: 라이브 대기열은 `session_rows` 의 시작 확인, ③ 은 `probe.json` 의 `reverse.mission`. submit 거부는 시작 아니다. 시작 여부를 모르면 N 에 넣지 않고 `start_unknown` 으로 둔다.
- 종결: `mission_results`·`live_failures`, 없는 마지막 임무는 상위 `failure_reason` 으로 보충. 실행 기록이 탐침보다 우선이고, 어긋나면 적는다.
- 시도: 끝난 임무는 `missions/mN/records.json`(`result.json` 상위는 비워진다), 도중에 끝난 임무는 `result.json` 상위. 같은 실행 안에서 시도 식별자(depth sha, 시각, loop_step)로 합친다.
- 임무 연결: 기록이 없는 시도는 탐침의 마지막 접수 순번과 완료된 `missions/m*` 번호로 잇고, 표본 `pallet_id` 는 교차 확인이다. 못 이으면 `mission_seq: "unknown"` 칸에 남긴다.
- **결측은 범위로 낸다.** N 은 확인 시작, S 는 `start_unknown` 이다. R 은 확인 도달, U 는 `reached_unknown`, m_R·m_U 는 그 안의 결과 불명이다. S 는 U 에 넣지 않는다. 근접 종결률은 k/(N + S) 부터 (k + m_R + m_U + S)/(N + S), 조건부는 k/(R + U + S) 부터 (k + m_R + m_U + S)/(R + m_U + S). 분모 0 은 산출 불가다.

## 4. 재검토 계기 판정

#338 의 값 그대로다. 우선순위 판단값이고 허용 실패율이 아니다. 미충족을 낮은 실패율의 입증으로 읽지 않는다.

- (a) 실제 시작 임무 N ≥ 50, k/N ≥ 2 %, 서로 다른 세션 2 건 이상.
- (b) 새 세션 둘 이상에서 재발하고, 종결 실패 원인 가운데 최다(공동 최다 포함).

두 계기 모두 운영 설정 합산 단위로 판단한다. 도구는 (a)·(b) 를 각각 `met`/`not_met`/`denominator_short`/`undecidable` 로 낸다.

- 결과 불명 임무는 근접 종결이나 다른 원인에, `start_unknown` 임무는 미시작이나 그 두 결과에 배정하는 모든 경우를 평가한다. 결론이 모두 같으면 그 결론이고, 갈리면 `undecidable` 이다.
- 어떤 배정에서도 N < 50 이면 (a) 는 `denominator_short` 다. 충족이 불가능한 상태와 미달을 구분한다.
- 부족해도 이 배치 안에서 세션을 늘리지 않는다. 같은 운영 설정의 추가 자료는 총괄이 정한다. 종결 원인은 `ct_survey.CLASSES` 와 `failures.PREFIXES` 의 가장 긴 접두어 두 수준으로 세고, (b) 는 세부 수준으로 판단한다. `near_capture_` 는 한 묶음이고 다른 원인은 더 잘게 쪼개지므로 (b) 는 충족 쪽으로 기운다. 계기는 재검토 우선순위일 뿐이라 그대로 두고, `status:reason` 표를 같이 낸다.

**계기 충족은 A2 재개가 아니다.** 충족하면 먼저 임무별로 원인을 나눈다.

- 분자의 모든 원인이 A2 로 고칠 수 있는 것은 아니다. 원거리·SLAM 오차로 예상 자리가 어긋나 참 팔레트가 `outside_expected` 로 걸러져도 같은 `no_expected_candidate` 다.
- 재검출 대상 임무마다 예상 자리와 참 팔레트의 차이, 평면 후보 사유, 보존된 프레임의 A0·A2(`--near-capture-ransac-iterations 400`) 오프라인 재검출을 표로 낸다. A2 구제 가능 수는 계기를 바꾸지 않는 보조 수다.
- 대기열은 실패에서 멈추므로 실패 임무의 프레임은 다음 임무에 덮이지 않는다. 디스크 세션에 이 프레임 보존을 등록한다. 성공 임무의 프레임은 임무 번호가 다시 시작해 덮이므로 '안 터진 평면 분리'는 이 배치로 재지 않는다.
- 이 진단에 쓴 프레임은 A2 개발 자료가 되므로 후속 확인은 새 자료로 한다(#338 자료 분리).

**이 배치로 무엇을 결정할 수 있나:** 대기열은 실패에서 멈추므로 세션당 근접 종결은 최대 1 건이고, 2 건이면 자동으로 서로 다른 세션이다. 합산 N ≈ 60 이면 2 건 이상이 (a) 를 채운다. 참 근접 종결률이 2 %일 때 60 임무에서 2 건 이상 나올 확률은 약 34 %, 5 % 면 약 81 %, 1 % 면 약 12 % 다(독립·세션 멈춤 무시 가정의 어림). 즉 2 % 언저리는 이 크기로 가르지 못한다. 0–1 건이 나와도 '드물다'는 결론이 아니라 '계기 미충족'이다. 멈춤 하나가 뒤 임무를 최대 2 개 없애므로 N 이 50 아래로 가려면 첫째 임무 멈춤이 여섯쯤 있어야 한다.

## 5. 구현과 제출

1. 이 계획 PR 을 머지하고 main 스냅숏으로 `isaac_bundle.py check` 뒤 제출한다(총괄 승인, 2026-11-07: Codex 수렴 뒤 바로). 디스크 세션에 배치를 등록한다. 집계에 새 기록은 필요 없으므로 제출은 집계 도구를 기다리지 않는다.
2. 배치가 도는 동안 `tools/ct_survey.py` 에 `near_capture_tally`, 세션 집계, 재검토 계기를 더한다. 시험을 먼저 쓴다. **이 도구의 PR 을 머지하기 전에는 배치 결과를 읽지 않는다**(2 장 결과 비열람).
   - 면제 구제가 첫 호출 비valid 에 들어가고 임무 분자에는 한 번만 들어감, 회복 구분
   - 마지막 임무 시도가 `result.json` 상위에만 있을 때 연결, 못 이으면 unknown, 겹치면 한 번만 셈
   - 범위 식(k 2·R 20·U 80 전부 결과 불명이면 82 %), 분모 0
   - (a)·(b) 경계(N 49·50, k/N 2 %, 세션 1·2), 모든 배정에서 갈리면 `undecidable`, `start_unknown` 이 N 을 50 위로 올릴 수 있을 때
   - 두 장면 합산: 다른 장면의 운영 설정 해시가 같고, 팔·형상·주입이 다르면 합치지 않음
   - `condition_sha256` 이 seed·경로에 불변, 다른 인자에 민감
   - ③ 형태(`probe.json` `reverse.mission`, submit 거부, 표본 없을 때 `entry_face: unknown`)
3. 결과 문서: 세 비율, 칸 표, 임무 결과, 재검토 계기 판정, 한계. 사후에 A2 재개를 권고할지는 총괄 결정이다.

## 6. 보지 못하는 것

- T11, 두 번째 캡처·재관측(세션 장면에서 드물다), 근접 추적(이 운영 설정에서 꺼져 있다), 시작 재위치·자동 임무 선택, 실제 D435i.
- 두 고정 장면의 픽업 여섯(P-1·2·3, P-4·5·3)의 값이다. 다른 장면으로 일반화하지 않는다.
- 평면 적합의 시드 민감성을 표집하지 않는다. RANSAC 시드는 상수이고(`DetectorParams.seed`) 러너 `--seed` 와 무관하므로, seed 100–109 는 주행·SLAM·인식 변동이 만든 프레임 분포만 바꾼다. 같은 프레임이면 결과도 같다. 이 값이 A2 의 필요량을 직접 재는 것은 아니다.
- 앞 임무가 멈추면 뒤 임무는 노출되지 않는다. 특히 조사의 유일한 근접 실패는 `random_m` 둘째 임무(P-5)였다. 결과에 임무 순서·팔레트별 도달 수 R 을 적고, 노출이 적으면 그 사실을 적는다.
- 위치 검사 켬은 시작 임무 수가 달라지므로 다른 칸이다.

## 7. 자체 검토

이 계획은 O0 대 O6 등록 비교 계획(7 판까지, 커밋 안 함)을 줄인 것이다. 그 판에서 검토받은 집계·기록 읽기·기반 실패 정의를 물려받았고, 비교 전용 규칙(비열등 상한, 장면 하한, 안전 비교, hold, 승격 근거 표)은 뺐다.

### ① 원자료

- 개입 0 은 `cts_dfa9aaf2` 기록에서 직접 셌다(스크립트 2 개). `result.json` 상위 시도가 끝난 임무에 0 건임을 28 세션에서 확인했다. 확률과 Wilson 구간은 scipy·식으로 다시 계산했다.
- 조사 28 세션의 `arguments` 를 비교해 해시 제외 목록을 정했다. 이 규칙을 그 기록에 돌리면 운영 설정 해시가 D·D′(20)·O0(4)·O6(4) 세 값이고, 각각 장면별 `condition_sha256` 두 개로 나뉜다. 두 장면이 한 운영 설정으로 합쳐진다는 뜻이다.
- 코드 확인: `mission_queue.py:524` 실패에서 대기열 정지, `pocket_detector.py:430` 시드는 `params.seed`, `run_transport.py:1557` 관측점은 장면 값.

### ② Codex

이전 판 3 회(P1 2·P2 9 → P2 7 → P2 2)는 비교 계획에 대한 것이고 이 판에 남는 부분은 물려받았다. 이 계획은 3 회 지적 뒤 4 회차(5판)에 **새 P1/P2 없음**으로 수렴했다.

| 회차 | 지적 | 처리 |
|---|---|---|
| 이 계획 1 | P2 조건별 N 최대 30 이라 (a) 도달 불가 | 칸과 합산을 나눔, `operating_config_sha256` |
| | P2 접수 불명과 결과 불명 혼동 | `start_unknown`, N 범위까지 배정 평가 |
| | P2 감시로 결과가 샌다 | 허용 감시와 필터 고정 |
| | P2 작동 0 을 장면 일반화 | 1 장 문구, 오라클 20/24 명시 |
| 이 계획 2 | P2 `observation_waypoints` 가 해시를 가름 | 장면 값과 같을 때만 제외, 조사 기록으로 확인 |
| | P2 k 가 R 의 부분집합이 아님 | 도달 정의 확장, `ended_without_capture` |
| 이 계획 3 | P2 `near_capture_no_final_straight` 는 캡처 뒤에도 남 | 사유가 아니라 시도 기록으로 판별, 재검출 대상은 깊이 sha256 대조 |

### ③ 독립 검토

- agy(Gemini 4 Argon, 이 판의 앞 판을 자체 검토 없이 전달)는 배치가 결정을 바꿀 수 있다고 답했고 P1 2·P2 3·P3 1 을 냈다.
- 이전 판 agy 검토(P1 4·P2 5·P3 2)의 노출 0 지적이 총괄 결정 D 를 낳았다.

| 지적 | 처리 |
|---|---|
| P1 시드 고정이라 시드 민감성을 표집 안 함 | 6 장에 명시, 오프라인 재검출은 계기 충족 때 실패 프레임에만 |
| P1 모든 근접 종결이 A2 대상이 아님 | 계기 충족 ≠ 재개, 원인 분리 절차, A2 구제 가능 수는 보조 |
| P2 `near_capture_` 분류 해상도 | k 를 분류와 맞춤, 기우는 방향과 `status:reason` 표 명시 |
| P2 N 여유 10·P-5 노출 편향 | 세션 추가 없음 규칙, 순서·팔레트별 R 보고 |
| P2 `near_capture_misaligned` 누락 | 분자에 포함 |
| P3 진입면 출처 | `spool/state.samples`, 집계 전 삭제 금지 |

**불채택:** agy 의 '성공 임무 근접 프레임 전부 재검출'은 프레임이 덮여 할 수 없다. 분자 확대는 #338 '근접 캡처로 끝난 임무'의 해석이므로 총괄 보고에 명시한다.
