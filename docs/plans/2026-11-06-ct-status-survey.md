# 연속 이송 현황 조사 — 계획 (1 판)

- 작성: 2026-11-06, v-g2 세션. 기준 origin/main `a5ec6cd1`.
- 지시: 총괄(2026-11-06) — 8 주차 다음 목표(일자·랜덤 배치 연속 이송을 인식·SLAM 포함 완주, `docs/references/week8_feedback.md`·week-08 발표)에 맞춰, 그동안 운영 기본에 들어간 변경이 쌓인 지금 main 으로 연속 이송을 돌려 **무엇이 막는지** 실패 유형·빈도·단계를 집계한다.
- **현황 조사다.** 승격·판정 게이트가 아니므로 합격 문턱을 두지 않는다. 운영 변경은 없다. 결과는 다음 개선 후보 목록(근거·예상 효과·비용)으로 보고하고, 그 목록 안에서 선택지 ①(재시도 b 계획 실패 시 대체 경로, `docs/validation/2026-11-05-line2-robustness-v3.md`)의 순위를 매긴다.
- 미리 정하는 것은 장면·seed·사이클 수·실행 수 상한, 실패 분류 규칙, 다음 후보를 고르는 기준이다.

## 1. 실행 경로

- **연속 이송은 콘솔 대기열로 돈다.** 배치 러너(`riib_scene.sbatch`)는 장면 하나에 임무 하나다. 여러 팔레트를 잇는 세션은 라이브 러너(`deploy/console/isaac_live.sbatch`)와 콘솔 대기열 탐침(`tools/live_console_probe.py --scenario queue`, `tools/mission_queue.py` = queue.js)으로 돈다. ws1 래퍼 `test_runs/live/ct1_queue.sbatch`(`SCENARIO=queue`, `CYCLE=`)를 그대로 쓴다.
- **장면:** 등록부(`config/slam_map_bundles.yaml`)에 있는 세션 장면 두 개 — `scenes_ct1_a770031/line_m_session.json`(sha256 83e27b4c…)·`random_m_session.json`(8b8fe554…). ws1 의 지도 묶음 `line_m_team_spec_quantized_0ff9e3bdff13`·`random_m_team_spec_quantized_0facac3eea66` 이 남아 있다. 라이브 기본대로 등록 지도에서 위치를 잡는다. 묶음 실행(`BUNDLE_TOKEN`)이라 알려진 출발(재위치 없음)이다.
- **사이클:** 세션마다 고정 순서 세 임무 — line_m `P-1:D-1,P-2:D-2,P-3:D-3`, random_m `P-4:D-1,P-5:D-2,P-3:D-3`. CT-1 PR ④(#119)·KEEPOUT 비교(2026-10-27)와 같은 순서라 과거 결과와 나란히 놓을 수 있다(합산하지는 않는다 — 코드와 기본값이 다르다). 대기열 정책은 실패하면 멈추므로, 앞 임무가 실패한 세션의 뒤 임무는 돌지 않는다. 그래서 노출은 단계별로 센다(4 장).
- **운영 기본(riib 런처 기본):** 라이브 런처는 riib 런처가 쌓아 온 기본값 가운데 일부만 넣는다(`ENCODER_MODEL`·`D7_HOLD`·근접 재관측). 빠진 것을 `MORE` 로 같은 값으로 넣는다.
  - `--exit-prefix-exempt`(EXIT_EXEMPT=on), `--low-clearance-guard`(LOW_CLEARANCE=on), `--restart-confirm-scans 3`(RESTART_CONFIRM=3), `--return-start-backoff boxed`(RETURN_START=boxed, #263), `--near-end-stand on`(#273), `--return-cusp-replans 2`(#282).
  - `--docking-degenerate-continue` 는 넣지 않는다(#310 의 허용 목록은 line_2_m 하나다. 세션 장면은 목록 밖이므로 riib 기본도 off 다).
  - KEEPOUT 0·NOISE const·START_BAND off·ESCAPE off 는 riib 기본과 같다(라이브 런처도 넣지 않는다). 근접 재관측의 `lower` 는 라이브 래퍼가 `waive_far_confirmed`, riib 가 `waive` 다 — 라이브 래퍼 값을 그대로 둔다(라이브 운영 경로의 값).
  - **이 조합(라이브 + 위 riib 기본)은 처음이다.** 각 기본은 riib 배치에서 검증됐고 라이브 세션에서는 처음 함께 돈다. 결과는 이 조합에 조건부로 쓴다.
- **사전 점검:** 팔마다 `DRY_RUN=1` 로 래퍼·런처·러너 `--check-args` 를 통과시키고, 실제 러너 인자 목록(`RUNNER_ARGS`)을 결과 폴더에 남긴다. 위 여섯 인자가 다 들어갔는지, `--deterministic` 이 결정 팔에만 있는지 확인한다.

## 2. 모집단

| 팔 | 설정 | 장면 × seed | 세션 |
|---|---|---|---|
| D (주) | 결정 모드 `--deterministic`, 영상 없음(라이브 런처는 영상을 켜지 않는다) | line_m·random_m × seed 0–7 | 16 |
| D′ (결정성 쌍) | D 와 같은 설정, 같은 스냅숏·seed 로 한 번 더 | line_m·random_m × seed 0·1 | 4 |
| O (운영 보조) | 결정 모드 끔(학습 제안기, 실행 중 SLAM 정적 층이 계획에 들어감) | line_m·random_m × seed 0–3 | 8 |

- **합계 28 세션, 상한 32.** 상한 안의 재실행은 기반 실패(Isaac 기동 실패·`startup_silent`·`startup_viewport`, 래퍼가 탐침 전에 끝남)만 한다. 임무 실패·SLAM 링크 실패는 재실행하지 않고 결과로 센다.
- **결정 모드의 한계(D·D′):** 학습 제안기 대신 oracle 인스턴스 마스크를 쓰고(학습 인식 실패는 나오지 않는다), 실행 중 SLAM 정적 층을 계획에 쓰지 않으며, 작업자 기한이 상한이다(`docs/plans/2026-10-25-deterministic-sim-mode.md` D1b·D4·D6). 그래서 D 는 운영과 다른 설정이다. O 는 그 차이에서 나오는 실패(학습 인식, SLAM 정적 층이 막는 계획)를 보려는 작은 보조 집단이다. **D 와 O 는 합치지 않는다.** O 는 seed 마다 갈라지므로 빈도로 읽지 않고 유형이 D 에 없는지만 본다.
- **결정성 쌍:** D′ 를 D 의 같은 seed 와 `tools/deterministic_compare.py` 로 비교한다. 같으면 D 의 실패를 다시 돌려 볼 수 있는 사례로 적고, 다르면 어느 기록부터 갈라졌는지 적는다(이 조사의 결론을 바꾸지 않는다).
- **크기:** 라이브 세션은 GPU 당 3(묶음 3 슬롯 shard 4, `tools/isaac_bundle.py sizes` 값). 두 GPU 면 동시 6 이다. 세 임무 세션이 과거 최대 30.5 min 이었고, 결정 모드는 작업자 기한이 상한이라 더 걸릴 수 있다 — 세션 한도 75 min(래퍼 기본)을 쓰고, 5 회전 안팎 약 4–6 h 로 본다. 디스크는 세션당 수백 MB 이하로 보고 10 GB 안이다(지금 /home 여유 39 G). 넘을 것 같으면 디스크 세션에 롤링 등록한다.
- 스냅숏은 main `a5ec6cd1`(또는 제출 시점 main) 하나다. 세 팔 모두 같다.

## 3. 실패 분류

- **단위:** 임무. 분모는 세션마다 예정된 세 임무 가운데 대기열이 받아들여 시작한 임무다(탐침 `missions_run` 의 접수와 `leg_failures_pairs._session` 의 규칙 — 기록 없이 잘린 임무도 센다). 시작하지 못한 뒤 임무는 '미노출'로 따로 센다.
- **결과:** `done` / 실패(사유 문자열) / 세션 중단(`session_failed`: 그 epoch 의 실행 기록에서 읽은 사유) / 기록 없음.
- **단계:** 사유 문자열이 단계를 말하면 그것(`Tracking failed in transport` → transport), 아니면 그 임무 기록의 마지막 전이 단계.
- **유형(문자열 규칙, 처음 표 — 새 문자열은 결과 문서에 추가 규칙으로 적는다):**

| 유형 | 문자열·조건 | 기전 확인에 볼 기록 |
|---|---|---|
| 인식 | `observe:*`, 근접 캡처 거절·`near_*`, 후보 없음 | 원거리 후보 수·확신, 근접 캡처 판정, oracle/학습 |
| 추종 | `Tracking failed in <단계>`, `Tracking timeout in <단계>` | 실패 순간 횡오차·요 오차·조향 포화(`slam_control`·제어 기록), 보정 나이와 마지막 보정의 크기, 경로 교체 이력 |
| 도킹·하역 | `transport_recovery_after_docking`, `Pallet missed`, 하역 맞춤 거부 뒤 실패 | 맞춤 라운드·상태(`delivery_docking`), D7 재접근 기록 |
| 계획·허가 | `scene_replan_*`, `permit_stop:*`, `invalid_start`, cusp | 거절 사유, 정지 위치와 가까운 점유, 재계획 횟수 |
| 적재 안전 | 차체 기울기·들기 기울기, 미끄럼, 금지 접촉 | 삽입 깊이·포크 위치, 미끄럼 판정 값, 접촉 기록 |
| SLAM 링크 | `slam_link_failed`, `slam_not_localized` | 브리지 기록·마지막 응답 스캔 ID |
| 기반 | 기동·래퍼·탐침 오류 | 래퍼·탐침 로그 |

- **문자열과 기전을 따로 적는다.** 같은 오류 문자열이 같은 기전이라는 뜻이 아니다. 각 실패에 위 표의 기록을 확인해 기전 꼬리표를 단다. 확인이 안 되면 '기전 미확인'으로 둔다. 개선 후보의 근거는 기전 꼬리표가 붙은 실패만 쓴다.
- **적재 운반 중 추종 실패(`Tracking failed in transport`·`Tracking timeout in transport`)는 따로 표시한다.** 질량 탐침 재검토 계기 (i)(#326) 판단에 쓰도록, 세션·임무·적재 상태·실패 순간의 회전 방향(좌·우, 전진·후진)·보정 나이를 함께 적는다. 이 조사로 계기 충족을 판정하지는 않는다 — 총괄의 판단 자료다.

## 4. 보고

- 팔 × 장면별: 세 임무 완주 세션 수, 임무 완료/시작 임무, 미노출 임무, 유형·단계별 실패 수, 단계별 노출(그 단계에 들어간 임무 수)과 그 단계의 실패 수.
- 실패마다 한 줄: 세션·seed·임무·단계·문자열·유형·기전 꼬리표(또는 미확인)·근거 기록 경로.
- 결정성 쌍 결과, O 에서만 나온 유형, 적재 운반 추종 실패 표.
- 금지 접촉·적재 안전 사건은 완주 여부와 별개로 모두 적는다.

## 5. 다음 후보를 고르는 기준

- 후보마다 근거(어느 유형·기전, 몇 건, 어느 단계, D·O 어디), 예상 효과의 상한(그 기전으로 실패한 임무 수 — 고치면 최대 그만큼 회복), 비용(코드·검증·Isaac 비교 규모), 기존 검증과 겹치는지(이미 기본인 변경의 남은 실패인지)를 적는다.
- 순위: 기전이 확인된 실패 수가 많고(D 기준), 단계 노출 대비 실패율이 높고, 비용이 작은 순. O 에서만 나온 유형은 따로 적는다(빈도로 비교하지 않는다).
- **선택지 ①의 순위:** 같은 기준으로 매긴다. 이 분기는 `--docking-degenerate-continue` 가 켜진 line_2_m 하역에만 있다. 세션 장면은 그 목록 밖이라 이 조사에서 노출이 0 일 것으로 보이며, 그러면 근거는 V3b 의 관측된 열화(off 가 성공한 3 seed)뿐이다. 노출이 0 이면 그 사실과 함께 순위를 적는다.

## 6. 가를 수 없는 것

- 결정 모드의 설정 차이 때문에 D 의 빈도가 운영 빈도라고 말하지 않는다. O 는 작아서 운영 빈도를 추정하지 못한다.
- 대기열이 첫 실패에서 멈추므로 뒤 임무의 실패율은 앞 임무를 통과한 세션에 조건부다.
- 두 장면(일자·랜덤 세션)만 본다. 통로·트레일러·컨베이어 장면은 세션 장면이 없어 범위 밖이다.

## 7. 산출물·순서

- 집계 도구 `tools/ct_survey.py`(세션 폴더 → 임무 표·유형·단계·노출·적재 운반 추종 실패 표, `leg_failures_pairs._session` 재사용)와 합성 시험. 코드가 들어가므로 머지 전 ws1 전체 pytest.
- 순서: 계획 검토(Codex) → 사전 점검(DRY_RUN, 인자 목록) → 묶음 제출 → 집계 → 결과 문서 `docs/validation/<날짜>-ct-status-survey.md`·Codex → 전체 pytest → PR → 총괄 보고(후보 목록).

## 8. 자체 검토

(검토 뒤 적는다.)
