# 팔레트 인식 일반화 단계 2A — 알려진 형상의 인스턴스 기하 맞춤·대상 선택 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **실행 방식(사용자 결정 2026-10-15, 단계 1A·1B 와 같음):** 이 세션에서 직접 구현(executing-plans), 검토 지점마다 Codex 검토를 수렴까지 받고 진행한다.

**Goal:** 인스턴스 마스크 하나에서 진입면 평면·받침면·알려진 형상 가설을 맞춰 `PocketCandidate` 를 내고(설계 §7), 후보 목록에서 작업 지시에 따라 대상을 고르는(§8) 기하 코어를 만든다. **정답 마스크(oracle)로 먼저 평가**해 맞춤 품질을 분할 품질과 떼어 보고(§12 항목 2), 같은 Isaac dev 장면에서 지금 검출기 기준선과 비교한다.

**Architecture:** 코어(numpy 만)에 네 모듈을 더한다 — `instance_proposals.py`(제안 타입), `pocket_candidate.py`(후보 타입), `instance_fit.py`(진입면·광선 분류·형상 틀 탐색·가설 경쟁·후보 조립), `target_selection.py`(층 번호·대상 선택). 형상 틀은 `spec_boxes` 를 진입축으로 투영한 점유 격자이고, 마스크 점과 원본 깊이 광선을 같은 면 좌표(가로 u, 높이 h) 격자에 모아 (가로 위치, 받침면 높이) 전 범위를 상관으로 훑는다. 평가기 CLI 에 `oracle-fit` 검출기(정답 인스턴스 ID 영상 → 제안 → 맞춤 → 선택)와 후보별 진단 파일을 더한다. 실행은 데이터가 있는 ws1 의 Slurm CPU 잡.

**Tech Stack:** numpy, PyYAML(기존), pytest. 실행 ws1 Slurm(`compute`, CPU 만), 단위 시험 CPU 리그(`tools/instance_rig.py`).

**Spec:** `docs/design/2026-10-15-pallet-recognition-generality.md` §3.3–3.4, §7, §8, §10, §11, §12 항목 2. 단계 1: `docs/plans/2026-10-15-recognition-stage1a-data-and-evaluator.md`, `docs/plans/2026-10-15-recognition-stage1b-noise-and-isaac.md`, `docs/validation/2026-10-16-recognition-stage1b-isaac-baseline.md`.

## 계획 전 실측 (2026-10-16, 이 계획의 근거)

작업 1–5 의 코드와 시험(이 계획의 코드블록과 같은 파일)을 ws1 Slurm 에서 돌렸다 — 시험 79 개 통과(잡 3114, `artifacts/20261016T064803Z_s2a_proto_round5`). dev 집계는 1B 렌더 `20261015T150838Z_recognition_isaac` 의 dev 분할 전체를 정답 인스턴스 ID 영상을 제안으로 써서 맞춘 것이다. 단위는 대상 인스턴스(진입면 노출 ≥ 50 %·접근 가능), 오차는 평가기 `_face_cost` 그대로. 아래 수치는 **`fitted` 후보만 본 조건부 값**이다(놓침·대상 선택은 작업 6 의 평가기로 잰다).

| 사례 · 형상 | 인스턴스 | 잡음 없음: `fitted` / 그중 20 mm·20 mm·2° 안 | 잠정 중간: `fitted` / 그중 안 |
|---|---|---|---|
| S1 EPAL 6 / T11 | 42 / 42 | 41/41 · 41/41 | 37/20 · 38/16 |
| S2 EPAL 6 / T11 | 96 / 101 | 85/85 · 94/94 | 73/40 · 85/44 |
| S3 EPAL 6 / T11 | 55 / 91 | 47/47 · 48/48 | 37/22 · 43/17 |
| S4 EPAL 6 / T11 | 109 / 104 | 101/101 · 93/93 | 90/49 · 83/50 |
| S5 EPAL 6 / T11 | 66 / 80 | 56/56 · 67/67 | 52/30 · 61/30 |
| S6 EPAL 6 / T11 | 42 / 41 | 38/38 · 37/37 | 35/22 · 28/11 |
| S7 비규격 | 135 | 22/16 | 18/7 |

`fitted` 가 아닌 것은 대부분 `ambiguous`(가로 구속 부족 — 가려져 경계를 본 부재가 둘 미만, T11 x/y 경합, 형상 불일치)와 `unfit`(점수 미달)이고, 후보 없음은 S3 T11 의 23 개(아래 미해결 관찰)를 빼면 장면마다 0–7 개다. 잡음 없음의 `fitted` 오차 p95 는 가로 5–10 mm·삽입축 0 mm·yaw 0.0°·높이 2–4 mm, 형상 식별은 전부 맞았다. 처리 시간은 인스턴스당 p50 0.05 s·p95 0.10 s(ws1).

`fitted` 후보의 오차 p95 를 **예측 거리**(맞춘 진입면 중심과 카메라의 수평 거리, 대응된 알려진 형상 후보만 — Codex: 정답 거리 집계는 작업 6 진단에서)로 나누면:

| 예측 거리(m) | 0–1.5 | 1.5–2.0 | 2.0–2.5 | 2.5–3.0 | ≥ 3.0 |
|---|---|---|---|---|---|
| 잡음 없음 가로 / 삽입축 / yaw | 5 / 0 / 0.0° | 5 / 0 / 0.0° | 6 / 0 / 0.0° | 6 / 0 / 0.0° | 7 / 0 / 0.0° |
| 잠정 낮음 | 9 / 5 / 0.7° | 13 / 10 / 2.0° | 22 / 18 / 2.6° | 27 / 32 / 4.8° | 39 / 57 / 9.9° |
| 잠정 중간 | 9 / 6 / 0.9° | 15 / 13 / 2.8° | 25 / 26 / 3.8° | 30 / 41 / 6.1° | 41 / 72 / 11.6° |
| 잠정 높음 | 14 / 9 / 1.3° | 18 / 20 / 3.9° | 31 / 36 / 5.3° | 37 / 62 / 9.0° | 41 / 96 / 14.2° |

시제품을 고치며 확인한 것(각각 시험으로 고정했다):

1. **광선이 점수를 더하면 틀이 배경 위로 떠다니며 점수를 번다.** 첫 판은 개구 자리의 '뒤' 광선에 가산점을 줘 T11 면에 EPAL 틀이 이겼다(T11 S1 `fitted` 5/42). 광선은 감점만 한다 — 개구 자리의 '면' 광선, 재료 자리를 뚫고 지나가는 광선(약하게 — 부러진 부재도 뚫려 보인다).
2. **진입면 평면이 블록 안쪽 측면·상판 윗면에 끌린다.** ① 내려다볼 때 윗면 점이 가장 큰 선을 만든다 → 평면은 윗면을 뺀 점으로 맞추고 면 점에는 다시 넣는다(T11 상판 15 mm 가 빠지면 가로가 13 mm 로 나빠졌다) ② RANSAC 최다 내점 선이 면보다 1.5 cm 안쪽(면 + 차선 벽)에 놓인다 → 가장 조밀한 얇은 층으로 옮기고 방향을 다시 맞춘다 ③ 대칭 Huber 가 면 뒤 차선 벽에 끌려 척도가 2–3 cm 로 굳는다 → 면 뒤 2.5 척도 밖은 가중치 0, 척도는 |r| 하위 25 % 분위수 ④ 안쪽 평행 평면·느슨한 점 묶음·엉뚱한 둘째 면 → 바깥면 검사, 척도 상한, 첫 면과 직각(±10°)만. 면 사이 비교는 정규화 점수가 아니라 근거 총량으로 한다(점 적은 면이 정규화 점수로 이기던 것).
3. **가로 구속은 원본 광선으로, 소유를 따져 본다.** 경계마다 안쪽에 제안 마스크 소유의 '면' 광선, 바깥쪽에 '뒤' 광선이 각각 3 개 이상(창 max(15 mm, 4 px 환산))이면 그 경계를 본 것으로 센다.
4. **받침면 상태.** 차선 아래로는 바닥이 뚫려 보이므로 맨 아래 점이 진입면 위·받침면 높이에 있는 열만 본다. 이 수정 전에는 바닥 위 S1 팔레트의 절반이 `uncertain` 이었다.
5. **형상 불일치.** 알려진 형상이 최고 점수여도 틀 밖 점 비율 > 0.08 이거나 재료 자리를 뚫는 광선 비율 > 0.12 면 `shape_mismatch`. 이 검사 전에는 S7 135 개 중 76 개가 알려진 형상으로 `fitted`(가로 p95 66 mm)였고, 검사 뒤 22 개다. 뚫림은 **허용 파손이 비울 수 있는 상판 띠(§5.2 — 상판 1–2 장 결손·깨짐)를 빼고** 센다(Codex 2·3회차: 빠진 가장자리 판 한두 장이 다른 형상으로 읽혔다. 처음 시도한 '면 뒤 0.15 m 안의 자기 화소 제외'는 인접 두 장 결손(다음 판 192.5 mm 뒤)을 못 덮었고, 실제 개구에서도 가까운 차선 벽을 얕게 맞혀 구분이 성립하지 않았다). 블록 깎임·갈라짐은 면적이 작아(진입면 재료의 2 % 안팎) 그대로 센다. 같은 문턱에서 알려진 형상(S1–S6)의 오거부는 잡음 없음 0–1, 잠정 중간 1–3 장면 수준이다.
6. **보이는 받침면 높이를 쓴다.** 받침면이 `visible` 이면 그 측정 높이(아래 띠 수평 점의 중앙값)로 틀을 다시 놓는다(§7.3). 상판이 빠지면 높이 탐색이 12 mm 떠서 거짓 `lift_alignment_needed` 가 붙던 것이 없어지고, 높이 오차 p95 가 3–5 → 2–4 mm(잡음 없음)로 줄었다. 받침면이 보이는지는 거리 띠가 아니라 **아래 두 화소 줄의 높이 차**(수평면이면 그대로, 수직면이면 화소 간격 × 거리만큼 낮아짐)로 가른다 — 1.3 m 에서는 바로 아래 바닥이 진입면보다 22 mm 앞일 뿐이라 거리 띠로는 면으로 읽혔다(Codex 4회차). 이 수정으로 S1 단독 팔레트의 받침면이 전부 `visible` 이 됐다. 받침면을 고정하면 그 면의 모든 가설·순위·관문을 다시 계산한다(돌려주는 배치와 판정이 늘 짝을 이룬다).

**원거리 yaw 위험(Codex 교차검증 2026-10-16).** 잡음 없는 꽉 찬 EPAL 6 앞면(0.8 × 0.144 m)에 잠정 잡음만 입혀 면 화소 전부로 평면을 맞춰도 yaw p95 는 mid 에서 2.0 m 1.8–2.0°, 2.5 m 3.5–4.0°, 3 m 5.4–6.2° 였다(주성분·역깊이 최소제곱 모두). Codex 는 이것이 **정보 하한이 아니라 특정 추정기의 성능**임을 반례로 보였다 — 잡음 공분산을 아는 역깊이 GLS 는 2.0 m 1.58°, 3 m 약 4.6°. 또 §3.4 는 거리별이 아니라 통합 p95 이고, 근거리로 판정을 한정하는 것은 별도 승인·새 모집단이 필요한 설계 개정이며, 잠정 잡음 모형으로 실제 D435i 의 한계 거리를 선언할 수 없다고 했다. 그래서 **이 계획은 합격선을 바꾸지 않는다.** 결과를 거리별로 함께 기록하고(작업 6), 판단은 공식 잡음(팀 D435i 측정) 뒤로 미룬다.

## Global Constraints

- 코어(`src/forklift_core/`)는 numpy·PyYAML 만 쓴다. `tools/` 와 시험은 기존 의존성만.
- 좌표: base 프레임(x 전방, y 좌, z 위), 카메라는 `scene.base_from_optical`. 진입면 좌표는 가로 u = 삽입 방향을 +90° 돌린 왼쪽 축, 높이 h = base z. 포켓 왼쪽/오른쪽은 정답(`tools/recognition_truth.py:126–129`)과 같이 **가로 좌표가 큰 쪽이 왼쪽**이다.
- 포크·판정 상수(`fork_fit.py`)와 평가기 규칙(`evaluation_multi.py`)은 바꾸지 않는다. 평가기 CLI 의 기존 검출기(`baseline`, `oracle`) 출력은 그대로다.
- **정답 누출 금지.** `instance_fit`·`target_selection` 은 `truth.json` 이나 장면 메타의 `case`·`shape_key` 를 받지 않는다. 입력은 `SceneInput`, 제안, 알려진 형상 사전, `ForkSetup`(포크 간격은 로봇 설정 — 장면 메타의 `fork_spacing_m`), 작업 지시(`instruction.json`), 현장 기본 적재(`config/site_load_defaults.yaml`). 정답 인스턴스 ID 영상은 `oracle-fit` 검출기의 **제안 출처**로만 쓰고 그 후보의 출처는 `oracle` 로 기록한다.
- **eval 분할을 열지 않는다.** 모든 실행은 `--split dev`. 매개변수는 dev 에서만 정하고, 정하면 이 계획의 `FitParams` 기본값과 기록에 남긴다.
- 식별 상태: 제안 출처가 `oracle` 이면 `confirmed`, `geometric` 이면 `unconfirmed`, `learned` 면 점수 ≥ τ 일 때 `confirmed`(τ 는 단계 3, 그 전에는 `None` → `unconfirmed`).
- pytest 는 ws1 (`FORKLIFT_REPO=<worktree> ~/bin/forklift-ws1-pytest …`, 새 파일은 `git add` 먼저). 알려진 ws1 전용 실패 10 개는 그대로 둔다. 로컬은 1 분 안에 끝나는 한 파일만.
- 커밋 형식: `<area>: <내용>` 영어 한 줄 요약(최근 커밋을 따른다), 파일 단위 `git add`, 끝에 `Co-Authored-By` 줄.

## 범위

**이번(2A):** 알려진 형상(EPAL 6, T11 × 0.6)의 진입면 가설, 진입면 평면(바깥면 검사), 광선 분류, 받침면 높이·상태, 가설 경쟁(동치 묶기·모호), 자세 구속 검사, 포크 끼움(기존 `fork_fit`), `PocketCandidate`, 층 번호, 대상 선택, `oracle-fit` 평가와 dev 기록(S1–S7·N, 잡음 없음·잠정 세 수준).

**다음(2B, 별도 계획):** 비규격 매개변수형 가설(§7.1, S7)과 끝단 물리 확인(§7.4-5c), 분할 가설(§7.4-3), 기하 제안기(§4 — 수직면 분할 + 연결 성분, 마스크가 합쳐지는 경우는 여기서 처음 생긴다), 후보 추적·다중 프레임 융합(§8), 시간 계약(§6). 2A 에서 S7 비규격은 알려진 형상 가설로만 맞춰지므로 대부분 `unfit`/`ambiguous` 이거나 형상이 틀린 후보가 된다 — 기록만 하고 판정하지 않는다.

**S7 은 판정하지 않지만 막는다.** 형상 불일치 검사가 없으면 S7 비규격 135 개 중 76 개가 알려진 형상으로 `fitted` 됐고 가로 p95 66 mm 였다(ws1 dev, 잡음 없음). 2A 는 비규격을 맞추지 못해도 되지만 **알려진 형상으로 잘못 확정해서는 안 되므로** 형상 불일치 검사(작업 4)를 넣는다 — 같은 조건에서 `fitted` 가 22 개로 줄었다. 남은 22 개는 알려진 형상과 치수가 비슷한 비규격이고 2B 의 매개변수 가설과 경쟁시켜 가린다.

**Ruling(이 계획):** 정답 마스크는 인스턴스를 합치지 않으므로 분할 가설을 시험할 입력이 2A 에 없다. 분할 가설을 기하 제안기와 함께 2B 로 미룬다 — 틀리면 2A 의 S2 맞닿음 결과가 기하 제안기에서 재현되지 않을 수 있고, 그 비용은 2B 에서 다시 재는 것이다.

## Review Focus

1. **틀의 거울·면 방향.** 반대쪽 면(sign −1)이나 비대칭 형상에서 왼쪽/오른쪽 포켓이 뒤바뀌면 가로 오차가 포켓 간격만큼 난다. 대칭 격자는 한 번만, 비대칭이면 거울 가설을 따로 만든다. → 작업 3·4 시험.
2. **잡음 꼬리에 의한 면 거부.** 바깥면 검사·광선 분류의 여유가 거리 제곱으로 커지지 않으면 원거리 면이 버려진다(시제품 첫 판: 잠정 중간 S5 T11 `none` 10). → 작업 2 시험(잡음 입힌 리그 장면).
3. **부분 가림·파손에서 거짓 확신.** 부재 경계가 하나만 보이는데 `fitted` 로 내면 가로가 틀 폭만큼 미끄러질 수 있다. 경계 두 개(서로 다른 부재) 미만이면 `ambiguous`. → 작업 4 시험(절반 가림).
4. **층 번호의 거짓 절대값.** 맨 아래 받침이 안 보이는데 절대 층을 내면 지시 층과 틀린 대상을 고른다. → 작업 5 시험.
5. **지시 반경 안의 미식별·애매 후보.** 식별된 적합 후보가 하나여도 반경 안에 다른 후보가 있으면 자동 선택하지 않는다(§8-1). → 작업 5 시험.

## 착수 시 확인할 미해결 관찰

- **얇은 받침 띠**(Codex 5회차): 위층이 아래층보다 뒤로 물러나 아래층 상판 앞쪽이 조금 보이는 적층(T11 아래 x 2.40, 위 x 2.43)에서 +2·+6 화소 줄이 얇은 상판 띠를 벗어나 `uncertain` 이 된다. 거짓 절대 층은 아니고 재관측·절대 층 보류가 는다. 작업 6 dev 의 S4 에서 `uncertain` 비율을 보고 줄 간격을 거리에 맞출지 정한다.
- **S3 T11 의 후보 없음 23/91**(ws1 dev, 잡음 없음 — 앞뒤 배치의 뒤 팔레트로 보인다). 작업 4 를 마친 뒤 ws1 에서 해당 인스턴스의 마스크 크기·면 추출 단계별 결과를 뽑아 원인을 확인하고, 결함이면 시험으로 고정해 고친다. 노출 50 % 이상이라 놓침 지표에 들어간다.

## 파일 구조

| 파일 | 책임 |
|---|---|
| `src/forklift_core/perception/instance_proposals.py` (신규) | `InstanceProposal`(마스크·점수·출처) 검증 |
| `src/forklift_core/perception/pocket_candidate.py` (신규) | `PocketEstimate`, `PocketCandidate`(기하 상태 × 식별 상태, 값 출처, 끼움 결과) |
| `src/forklift_core/perception/pallet_model.py` (수정) | `ObservedPalletShape`(형상과 값 출처) |
| `src/forklift_core/perception/instance_fit.py` (신규) | 진입면·광선 분류·형상 틀·가설·받침면·후보 조립, `fit_instance`, `fit_scene` |
| `src/forklift_core/perception/target_selection.py` (신규) | `assign_layers`, `select_target` → `Selection` |
| `tools/evaluate_recognition.py` (수정) | `--detector oracle-fit`, `--diagnostics` |
| `deploy/slurm/recognition_stage2a_evaluate.sbatch` (신규) | ws1 dev 평가 잡 |
| `docs/validation/<실행일>-recognition-stage2a-oracle-fit.md` (신규) | 기록 |
| `tests/unit/perception/test_instance_proposals.py`, `test_pocket_candidate.py`, `test_instance_fit.py`, `test_target_selection.py` (신규), `tests/integration/test_recognition_cli.py`(추가) | 시험 |

## 검토 지점

- **검토 A (작업 1–4 뒤):** 타입, 진입면·광선·틀·가설·후보.
- **검토 B (작업 5–6 뒤):** 대상 선택, 평가기 연결, ws1 dev 결과와 기록.

---

### Task 1: 제안·후보·관측 형상 타입

**Files:**
- Create: `src/forklift_core/perception/instance_proposals.py`, `src/forklift_core/perception/pocket_candidate.py`
- Modify: `src/forklift_core/perception/pallet_model.py`
- Test: `tests/unit/perception/test_instance_proposals.py`, `tests/unit/perception/test_pocket_candidate.py`

**Interfaces:**
- Produces:
  - `PROPOSAL_SOURCES = ("learned", "geometric", "oracle")`; `InstanceProposal(proposal_id: str, mask: NDArray[bool] (H, W), score: float ∈ [0, 1], source: str)`
  - `ObservedPalletShape(spec: PalletSpec | None, face: FaceLayout, sources: tuple[tuple[str, str], ...])` — `sources` 는 (항목, 출처) 쌍, 출처 ∈ `VALUE_SOURCES`
  - `GEOMETRY_STATES = ("fitted", "ambiguous", "unfit")`, `IDENTITY_STATES = ("confirmed", "unconfirmed")`, `VALUE_SOURCES = ("observed", "known_shape", "instruction", "unobserved")`, `SUPPORT_STATES = ("visible", "coplanar", "physical_edge", "uncertain")`
  - `PocketEstimate(centre_m: tuple[3], width_m: float, height_m: float, width_source: str, height_source: str)`
  - `PocketCandidate(candidate_id, proposal_id, proposal_source, stamp_ns, hypothesis: str, axis: str, alternatives: tuple, insertion_yaw_rad, face_centre_m: tuple[3], pallet_centre_m: tuple[3], left: PocketEstimate, right: PocketEstimate, support_z_m, support_state, on_floor: bool, range_m, score, geometry, identity, reasons: tuple, pickable: bool | None, fit_reasons: tuple, shape: ObservedPalletShape)` — 프로퍼티 `counted`(= `fitted` 이고 `confirmed`). `pallet_centre_m` 은 진입면 중심 + 삽입 방향 × 형상 깊이/2 — 작업 지시가 팔레트 중심을 주기 때문이다(`tools/recognition_scenes.py` `_instruction`). `pickable is None` 은 "판정 보류"(받침면 미확인, 또는 근거리 재관측 대기)이고 삽입 허용이 아니다

- [ ] **Step 1: 실패하는 시험 작성**

`tests/unit/perception/test_instance_proposals.py`:

```python
# tests/unit/perception/test_instance_proposals.py
import numpy as np
import pytest

from forklift_core.perception.instance_proposals import InstanceProposal


def _mask():
    m = np.zeros((4, 5), bool)
    m[1:3, 1:4] = True
    return m


def test_a_proposal_keeps_its_mask_score_and_source():
    p = InstanceProposal("p0", _mask(), 0.9, "learned")
    assert p.mask.sum() == 6 and p.score == 0.9 and p.source == "learned"


@pytest.mark.parametrize("kw, match", [
    ({"mask": np.zeros((4, 5), bool)}, "nonempty"),
    ({"mask": np.ones((4, 5), np.uint8)}, "bool"),
    ({"mask": np.ones(5, bool)}, "two-dimensional"),
    ({"score": 1.5}, r"\[0, 1\]"),
    ({"score": True}, "finite"),
    ({"source": "truth"}, "source"),
    ({"proposal_id": ""}, "proposal_id"),
])
def test_bad_proposals_are_refused(kw, match):
    args = {"proposal_id": "p0", "mask": _mask(), "score": 0.5, "source": "geometric", **kw}
    with pytest.raises(ValueError, match=match):
        InstanceProposal(**args)
```

`tests/unit/perception/test_pocket_candidate.py`:

```python
# tests/unit/perception/test_pocket_candidate.py
import math
from dataclasses import replace

import pytest

from forklift_core.perception.pallet_model import ObservedPalletShape, face_layout
from forklift_core.perception.pocket_candidate import PocketCandidate, PocketEstimate
from tools import recognition_scenes as rs


def _candidate(**kw):
    spec = rs.known_spec("epal6")
    shape = ObservedPalletShape(spec, face_layout(spec, "x"), (("face", "known_shape"), ("depth", "known_shape")))
    pocket = PocketEstimate((2.0, 0.2, 0.061), 0.2275, 0.078, "known_shape", "known_shape")
    base = dict(candidate_id="c0", proposal_id="p0", proposal_source="oracle", stamp_ns=1, hypothesis="epal6:x",
                axis="x", alternatives=(), insertion_yaw_rad=0.1, face_centre_m=(2.0, 0.0, 0.0),
                pallet_centre_m=(2.3, 0.03, 0.0), left=pocket, right=replace(pocket, centre_m=(2.0, -0.2, 0.061)),
                support_z_m=0.0, support_state="visible", on_floor=True, range_m=1.25, score=0.9,
                geometry="fitted", identity="confirmed", reasons=(), pickable=True, fit_reasons=(), shape=shape)
    return PocketCandidate(**{**base, **kw})


def test_only_fitted_and_confirmed_candidates_are_counted():
    assert _candidate().counted
    assert not _candidate(geometry="ambiguous", reasons=("lateral_unconstrained",)).counted
    assert not _candidate(identity="unconfirmed").counted


@pytest.mark.parametrize("kw, match", [
    ({"geometry": "valid"}, "geometry"),
    ({"identity": "maybe"}, "identity"),
    ({"support_state": "floor"}, "support_state"),
    ({"insertion_yaw_rad": math.nan}, "finite"),
    ({"pallet_centre_m": (1.0, 2.0)}, "pallet centre"),
    ({"geometry": "ambiguous"}, "reason"),
    ({"pickable": "yes"}, "pickable"),
])
def test_bad_candidates_are_refused(kw, match):
    with pytest.raises(ValueError, match=match):
        _candidate(**kw)


def test_value_sources_are_checked():
    with pytest.raises(ValueError, match="source"):
        PocketEstimate((0.0, 0.0, 0.0), 0.2, 0.08, "guess", "known_shape")
    spec = rs.known_spec("epal6")
    with pytest.raises(ValueError, match="source"):
        ObservedPalletShape(spec, face_layout(spec, "x"), (("face", "measured"),))
```

- [ ] **Step 2: 실패 확인** — Run: `PYTHONPATH=src:. python -m pytest tests/unit/perception/test_instance_proposals.py tests/unit/perception/test_pocket_candidate.py -q -p no:cacheprovider` · Expected: `ModuleNotFoundError: No module named 'forklift_core.perception.instance_proposals'`

- [ ] **Step 3: 구현**

`src/forklift_core/perception/instance_proposals.py`:

```python
# src/forklift_core/perception/instance_proposals.py
"""Instance proposals: one mask per pallet-like object and where it came from (design §4, §6).

A proposal says which pixels belong to one object. Whether it is a pallet and
where its pockets are is decided by the geometric fit (instance_fit), not here.
"""

from dataclasses import dataclass

import numpy as np

from forklift_core._validation import _finite_scalar

PROPOSAL_SOURCES = ("learned", "geometric", "oracle")


@dataclass(frozen=True)
class InstanceProposal:
    proposal_id: str
    mask: np.ndarray
    score: float
    source: str

    def __post_init__(self) -> None:
        if not isinstance(self.proposal_id, str) or not self.proposal_id:
            raise ValueError("proposal_id must be nonempty text")
        mask = np.asarray(self.mask)
        if mask.ndim != 2:
            raise ValueError("mask must be a two-dimensional bool array")
        if mask.dtype != bool:
            raise ValueError("mask must be a bool array")
        if not mask.any():
            raise ValueError("mask must be nonempty")
        score = _finite_scalar(self.score, "score")
        if not 0.0 <= score <= 1.0:
            raise ValueError("score must lie in [0, 1]")
        if self.source not in PROPOSAL_SOURCES:
            raise ValueError(f"source must be one of {PROPOSAL_SOURCES}")
        object.__setattr__(self, "mask", mask)
        object.__setattr__(self, "score", score)
```

`src/forklift_core/perception/pallet_model.py` 끝에 추가:

```python
VALUE_SOURCES = ("observed", "known_shape", "instruction", "unobserved")


@dataclass(frozen=True)
class ObservedPalletShape:
    """The 3D shape downstream code uses, with where each value came from (design §7.1).

    spec is the known shape when the hypothesis is one, else None; face is the
    entry-face layout actually used for the pockets.
    """

    spec: PalletSpec | None
    face: FaceLayout
    sources: tuple

    def __post_init__(self) -> None:
        sources = tuple(tuple(item) for item in self.sources)
        for item in sources:
            if len(item) != 2 or not isinstance(item[0], str) or item[1] not in VALUE_SOURCES:
                raise ValueError(f"each source must be (name, one of {VALUE_SOURCES})")
        object.__setattr__(self, "sources", sources)
```

`src/forklift_core/perception/pocket_candidate.py`:

```python
# src/forklift_core/perception/pocket_candidate.py
"""PocketCandidate: one fitted pallet instance (ADR 0003 1-a, design §8).

Geometry state and identity state are separate (§7.5): only a fitted and
confirmed candidate may be selected automatically. Unlike PocketObservation,
a non-fitted candidate keeps its pose so the console and the tracker can show it.
pickable None means the fork-fit decision waits for more evidence (an unseen
support, or the near-range re-observation of §9.1); it never means "insert".
"""

from dataclasses import dataclass

import numpy as np

from forklift_core._validation import _finite_scalar
from forklift_core.perception.instance_proposals import PROPOSAL_SOURCES
from forklift_core.perception.pallet_model import VALUE_SOURCES, ObservedPalletShape

GEOMETRY_STATES = ("fitted", "ambiguous", "unfit")
IDENTITY_STATES = ("confirmed", "unconfirmed")
SUPPORT_STATES = ("visible", "coplanar", "physical_edge", "uncertain")


def _vector(value, n, name):
    try:
        out = tuple(_finite_scalar(v, name) for v in value)
    except TypeError:
        raise ValueError(f"{name} must be a finite {n}-vector") from None
    if len(out) != n:
        raise ValueError(f"{name} must be a finite {n}-vector")
    return out


@dataclass(frozen=True)
class PocketEstimate:
    centre_m: tuple
    width_m: float
    height_m: float
    width_source: str
    height_source: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "centre_m", _vector(self.centre_m, 3, "pocket centre"))
        for name in ("width_m", "height_m"):
            if _finite_scalar(getattr(self, name), name) <= 0:
                raise ValueError(f"{name} must be positive")
        for name in ("width_source", "height_source"):
            if getattr(self, name) not in VALUE_SOURCES:
                raise ValueError(f"{name} must be one of {VALUE_SOURCES}")


@dataclass(frozen=True)
class PocketCandidate:
    candidate_id: str
    proposal_id: str
    proposal_source: str
    stamp_ns: int
    hypothesis: str
    axis: str
    alternatives: tuple
    insertion_yaw_rad: float
    face_centre_m: tuple
    pallet_centre_m: tuple
    left: PocketEstimate
    right: PocketEstimate
    support_z_m: float
    support_state: str
    on_floor: bool
    range_m: float
    score: float
    geometry: str
    identity: str
    reasons: tuple
    pickable: bool | None
    fit_reasons: tuple
    shape: ObservedPalletShape

    def __post_init__(self) -> None:
        if self.geometry not in GEOMETRY_STATES:
            raise ValueError(f"geometry must be one of {GEOMETRY_STATES}")
        if self.identity not in IDENTITY_STATES:
            raise ValueError(f"identity must be one of {IDENTITY_STATES}")
        if self.support_state not in SUPPORT_STATES:
            raise ValueError(f"support_state must be one of {SUPPORT_STATES}")
        if self.proposal_source not in PROPOSAL_SOURCES:
            raise ValueError(f"proposal_source must be one of {PROPOSAL_SOURCES}")
        if self.axis not in ("x", "y"):
            raise ValueError("axis must be x or y")
        if self.geometry != "fitted" and not self.reasons:
            raise ValueError("a candidate that is not fitted needs a reason")
        if self.pickable is not None and not isinstance(self.pickable, bool):
            raise ValueError("pickable must be a bool or None")
        for name in ("insertion_yaw_rad", "support_z_m", "range_m", "score"):
            _finite_scalar(getattr(self, name), name)
        object.__setattr__(self, "face_centre_m", _vector(self.face_centre_m, 3, "face centre"))
        object.__setattr__(self, "pallet_centre_m", _vector(self.pallet_centre_m, 3, "pallet centre"))
        for name in ("reasons", "fit_reasons", "alternatives"):
            object.__setattr__(self, name, tuple(getattr(self, name)))
        if not isinstance(self.on_floor, (bool, np.bool_)):
            raise ValueError("on_floor must be a bool")
        object.__setattr__(self, "on_floor", bool(self.on_floor))

    @property
    def counted(self) -> bool:
        return self.geometry == "fitted" and self.identity == "confirmed"
```

- [ ] **Step 4: 통과 확인** — Run: Step 2 와 같음 · Expected: 17 개 통과.

- [ ] **Step 5: 커밋**

```bash
git add src/forklift_core/perception/instance_proposals.py src/forklift_core/perception/pocket_candidate.py src/forklift_core/perception/pallet_model.py tests/unit/perception/test_instance_proposals.py tests/unit/perception/test_pocket_candidate.py
git commit -m "perception: instance proposal, pocket candidate and observed shape types -- geometry and identity states kept apart, every pocket value carries its source"
```

---

### Task 2: 진입면 평면과 광선 분류

**Files:**
- Create: `src/forklift_core/perception/instance_fit.py` (이 작업은 앞부분)
- Test: `tests/unit/perception/test_instance_fit.py`

**Interfaces:**
- Consumes: `SceneInput`(`scene_dataset.py`), `instance_rig.render_instances`·`recognition_truth.placed_boxes/instance_boxes/Placement`(시험만)
- Produces:
  - `FitParams`(아래 기본값 전부 — 이후 작업이 같은 객체를 쓴다), `HEIGHT_REASONS`
  - `BEHIND, FACE, FRONT, UNOBSERVED = 0, 1, 2, 3`
  - `FacePlane(centre_xy, normal_xy, scale_m, index)`; 프로퍼티 `insertion_xy`(= −normal), `left_xy`
  - `erode(mask, radius) -> NDArray[bool]`, `pixel_rays(scene) -> (H, W, 3)`, `camera_xyz(scene) -> (3,)`
  - `proposal_points(scene, rays, mask, erode_px) -> (points (N, 3), pixels (N, 2))`
  - `fit_faces(points, camera, params, rng) -> list[FacePlane]` — RANSAC(거리에 따라 넓어지는 띠) → 가장 조밀한 얇은 층으로 옮기고 방향 재조준(`_snap_to_layer`) → 면 뒤쪽을 자르는 한쪽 Huber 재적합(`_irls_line`) → 척도 상한·바깥면·직각 검사
  - `face_coords(face, points) -> (u, h)`
  - `ray_samples(scene, rays, face, u_range, h_range, params, mask=None) -> (u, h, cls, owned)` — `owned` 는 제안 마스크 화소 여부(§7.2 증거 소유권)

- [ ] **Step 1: 실패하는 시험 작성** — `tests/unit/perception/test_instance_fit.py`:

```python
# tests/unit/perception/test_instance_fit.py
import math
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception import instance_fit as F
from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.fork_fit import ForkSetup, LoadDeclaration
from forklift_core.perception.instance_proposals import InstanceProposal
from forklift_core.perception.pallet_damage import DamageOp
from tools import instance_rig, scene_rig
from tools import recognition_scenes as rs
from tools import recognition_truth as rt

ROOT = Path(__file__).resolve().parents[3]
EPAL, T11 = rs.known_spec("epal6"), rs.known_spec("t11_06")
SPECS = {"epal6": EPAL, "t11_06": T11}
SITE = {s.name: LoadDeclaration(10.0, 0.0, s.overall_depth_m / 2, 0.0, "site_default") for s in (EPAL, T11)}
SPACING = {EPAL.name: 0.3725, T11.name: 0.290}


def _render(items, *, noise=None, distractors=(), depth=None):
    """items: (spec, Placement, damage) -> scene, instance ids; depth(rendered) may edit the depth."""
    placed = [rt.placed_boxes(rt.instance_boxes(s, d), p) for s, p, d in items]
    out = instance_rig.render_instances(placed, distractors=distractors)
    scene, ids = out.scene, out.instance_ids
    if depth is not None:
        from dataclasses import replace
        scene = replace(scene, depth_m=depth(out))
    if noise is not None:
        scene, ids = apply_noise(scene, ids, noise)
    return scene, ids


def _mid_noise(seed):
    return NoiseModel.from_yaml(ROOT / "config/depth_noise_d435i.yaml", seed=seed, target_fx=465.74, level="mid")


def _faces_of(scene, mask, params=F.FitParams()):
    rays = F.pixel_rays(scene)
    pts, _ = F.proposal_points(scene, rays, mask, params.erode_px)
    return F.fit_faces(pts, F.camera_xyz(scene), params, np.random.default_rng(0)), pts, rays


def _yaw(face):
    return math.atan2(face.insertion_xy[1], face.insertion_xy[0])


# ---- Task 2: faces and ray classes -------------------------------------------------------------

def test_the_front_face_plane_and_its_insertion_yaw_are_recovered():
    scene, ids = _render([(EPAL, rt.Placement(3.0, 0.3, 0.25, 0.0), ())])
    faces, _, _ = _faces_of(scene, ids == 0)
    assert faces and _yaw(faces[0]) == pytest.approx(0.25, abs=math.radians(0.3))
    front = np.array([3.0, 0.3]) - 0.3 * np.array([math.cos(0.25), math.sin(0.25)])   # EPAL 6 depth 0.6 m
    assert abs((front - faces[0].centre_xy) @ faces[0].normal_xy) < 0.005
    assert (F.camera_xyz(scene)[:2] - faces[0].centre_xy) @ faces[0].normal_xy > 0


@pytest.mark.parametrize("x_m", [2.4, 2.8])
def test_a_t11_seen_at_45_degrees_gives_only_its_exterior_faces(x_m):
    """Seen across a corner, every accepted plane is one of the two outer faces (+-45 deg)."""
    scene, ids = _render([(T11, rt.Placement(x_m, 0.0, math.pi / 4, 0.0), ())])
    faces, _, _ = _faces_of(scene, ids == 0)
    assert faces and all(abs(abs(math.degrees(_yaw(f))) - 45) < 1 for f in faces)


@pytest.mark.parametrize("placement", [
    rt.Placement(2.1500067490892496, -0.8781266206323024, 0.3974022305354298, 0.0),  # s1_t11_06_1000119 (Isaac dev)
    rt.Placement(1.9, 0.35, 0.55, 0.0),
])
def test_no_two_accepted_faces_are_parallel(placement):
    """A convex pallet shows at most one face per direction. In the first case the block sides
    seen through a lane form a plane parallel to the side face and 75 mm inside it (accepted
    without the exterior check); in the second the leftover points form a loose 'plane' with a
    48 mm scale (accepted without the scale limit)."""
    scene, ids = _render([(T11, placement, ())])
    faces, _, _ = _faces_of(scene, ids == 0)
    for i in range(len(faces)):
        for j in range(i + 1, len(faces)):
            d = abs((_yaw(faces[i]) - _yaw(faces[j]) + math.pi) % (2 * math.pi) - math.pi)
            assert d > math.radians(30)


def _wrong_faces(faces, pallet_yaw):
    """Faces that are not one of the box's four face directions (> 3 deg off) or form a parallel pair."""
    ys = [math.degrees(_yaw(f)) for f in faces]
    off = [y for y in ys if min(abs((y - math.degrees(pallet_yaw) - 90 * k + 180) % 360 - 180) for k in range(4)) > 3]
    parallel = [(a, b) for i, a in enumerate(ys) for b in ys[i + 1:] if abs((a - b + 180) % 360 - 180) < 30]
    return off + parallel


@pytest.mark.parametrize("scene_id, k, gate", [
    ("s1_epal6_1000009", 0, {"corner_tol_rad": math.pi / 2}),        # without it: a 5.2 deg face beside 5.6
    ("s2_t11_06_1000227", 1, {"exterior_max_fraction": 1.0}),        # without it: an interior 112.9 deg plane
    ("s2_t11_06_1000218", 0, {"max_face_scale_m": 1e9}),             # without it: a loose -12.2 deg 'plane'
])
def test_each_face_gate_removes_a_wrong_face_on_its_own(scene_id, k, gate):
    """Rig renders of dev recipes where turning off this one gate lets a wrong face through
    (ws1 search 2026-10-16; Codex: each gate needs a regression of its own)."""
    from dataclasses import replace
    recipe = next(r for r in rs.catalogue() if r.scene_id == scene_id)
    out = instance_rig.render_instances([rs.recipe_boxes(recipe, j) for j in range(len(recipe.instances))])
    yaw = recipe.instances[k].placement.yaw_rad
    faces, pts, _ = _faces_of(out.scene, out.instance_ids == k)
    assert not _wrong_faces(faces, yaw)
    loose = F.fit_faces(pts, F.camera_xyz(out.scene), replace(F.FitParams(), **gate), np.random.default_rng(0))
    assert _wrong_faces(loose, yaw)


def test_the_face_survives_far_range_noise():
    scene, ids = _render([(EPAL, rt.Placement(3.9, 0.0, 0.1, 0.0), ())], noise=_mid_noise(3))
    faces, _, _ = _faces_of(scene, ids == 0)
    assert faces and abs(_yaw(faces[0]) - 0.1) < math.radians(15)


def test_rays_through_an_opening_pass_behind_the_face_and_rays_on_a_block_stop_on_it():
    scene, ids = _render([(EPAL, rt.Placement(2.6, 0.0, 0.0, 0.0), ())])
    faces, _, rays = _faces_of(scene, ids == 0)
    u, h, cls, owned = F.ray_samples(scene, rays, faces[0], (-0.5, 0.5), (0.0, 0.2), F.FitParams(), mask=ids == 0)
    lane = (np.abs(u - 0.18625) < 0.05) & (h > 0.03) & (h < 0.09)   # left opening centre (EPAL 6 y = 0.18625)
    block = (np.abs(u) < 0.04) & (h > 0.03) & (h < 0.09)            # middle block
    assert (cls[lane] == F.BEHIND).mean() > 0.95 and (cls[block] == F.FACE).mean() > 0.95
    assert owned[block].mean() > 0.9   # (rays through a lane may end on the lane walls: owned, but behind the face)


def test_erode_shrinks_a_mask_by_its_radius():
    m = np.zeros((9, 9), bool)
    m[2:7, 2:7] = True
    assert F.erode(m, 1).sum() == 9 and F.erode(m, 2).sum() == 1 and not F.erode(m, 3).any()
```

주: EPAL 6 왼쪽 개구 중심 0.18625 m 는 `face_layout(EPAL, "x").opening_intervals_m` 의 (0.0725, 0.3) 중심이다(2026-10-16 재계산).

- [ ] **Step 2: 실패 확인** — Run: `PYTHONPATH=src:. python -m pytest tests/unit/perception/test_instance_fit.py -q -p no:cacheprovider` · Expected: `ImportError: cannot import name 'instance_fit'`

- [ ] **Step 3: 구현** — `src/forklift_core/perception/instance_fit.py` 앞부분(모듈 머리의 import 는 작업 3–4 가 쓸 것까지 한 번에 넣는다 — 작업 1 이 먼저 끝나 있으므로 모두 존재한다):

```python
# src/forklift_core/perception/instance_fit.py
"""Instance geometric fit (design §7): one proposal mask -> one PocketCandidate.

Evidence ownership (§7.2): members are fitted from the proposal's own pixels
(eroded), and a member edge counts only when the face-side rays belong to the
proposal; openings are judged from every raw-depth ray that meets the face
plane inside the hypothesised opening. A member that is not seen costs nothing
(damage, occlusion); material where a hypothesis has an opening does. Rays
never add score, so a hypothesis cannot earn score over empty background.

Faces are vertical planes in the base frame. Face coordinates: u along the
left axis (insertion direction turned +90 deg), h = base z. FitParams values
were chosen on dev scenes (docs/plans/2026-10-16-recognition-stage2a-instance-fit.md).
"""

import math
from dataclasses import dataclass

import numpy as np

from forklift_core.perception.fork_fit import fork_fit
from forklift_core.perception.instance_proposals import InstanceProposal
from forklift_core.perception.pallet_model import ObservedPalletShape, PalletSpec, face_layout, spec_boxes
from forklift_core.perception.pocket_candidate import PocketCandidate, PocketEstimate
from forklift_core.perception.scene_dataset import SceneInput

BEHIND, FACE, FRONT, UNOBSERVED = 0, 1, 2, 3
HEIGHT_REASONS = ("lift_alignment_needed",)  # fork-fit reasons that read the support height


@dataclass(frozen=True)
class FitParams:
    erode_px: int = 3                 # mask edge pixels carry flying depth and a 2 px RGB shift
    min_points: int = 60
    ransac_iters: int = 300
    ransac_tol_m: float = 0.03
    max_faces: int = 2
    min_face_span_m: float = 0.15
    top_exclude_m: float = 0.015      # planes are fitted without the top surface
    irls_iters: int = 15
    exterior_margin_m: float = 0.03
    exterior_max_fraction: float = 0.10
    corner_tol_rad: float = math.radians(10.0)  # a second face must meet the first at 90 deg +- this
    max_face_scale_m: float = 0.01    # a face's robust scale may not exceed max(this, ray_margin_k * reach^2 / 2)
    bin_m: float = 0.005
    support_window_m: float = 0.03
    ray_margin_m: float = 0.03        # ray class margin: max(ray_margin_m, ray_margin_k * t^2)
    ray_margin_k: float = 0.015
    hole_point_weight: float = 1.0
    outside_weight: float = 0.5
    face_ray_weight: float = 0.5
    seen_through_weight: float = 0.25
    edge_window_m: float = 0.015
    edge_min_hits: int = 3
    min_score: float = 0.3
    shape_outside_max: float = 0.08    # face points outside the shape's rectangle (dev 2026-10-16)
    shape_seen_through_max: float = 0.12  # rays passing through the shape's material (dev 2026-10-16)
    ambiguity_margin: float = 0.05
    equivalence_m: float = 0.020
    equivalence_yaw_rad: float = math.radians(2.0)
    seed: int = 0


@dataclass(frozen=True)
class FacePlane:
    centre_xy: np.ndarray
    normal_xy: np.ndarray  # unit, pointing toward the camera
    scale_m: float
    index: np.ndarray      # indices of the proposal points on this face

    @property
    def insertion_xy(self):
        return -self.normal_xy

    @property
    def left_xy(self):
        a = self.insertion_xy
        return np.array([-a[1], a[0]])


def erode(mask, radius: int):
    out = np.asarray(mask, bool).copy()
    for _ in range(radius):
        m = out.copy()
        m[1:, :] &= out[:-1, :]
        m[:-1, :] &= out[1:, :]
        m[:, 1:] &= out[:, :-1]
        m[:, :-1] &= out[:, 1:]
        m[0, :] = m[-1, :] = False
        m[:, 0] = m[:, -1] = False
        out = m
    return out


def pixel_rays(scene: SceneInput):
    """Base-frame ray per pixel with unit optical z: point = camera + axial_depth * ray."""
    k = scene.intrinsics
    v, u = np.indices((k.height, k.width))
    optical = np.stack(((u - k.cx) / k.fx, (v - k.cy) / k.fy, np.ones(u.shape)), axis=-1)
    return optical @ np.asarray(scene.base_from_optical.rotation, float).T


def camera_xyz(scene: SceneInput):
    return np.asarray(scene.base_from_optical.translation_m, float)


def proposal_points(scene: SceneInput, rays, mask, erode_px: int):
    kept = erode(mask, erode_px) & np.isfinite(scene.depth_m)
    v, u = np.nonzero(kept)
    return camera_xyz(scene) + scene.depth_m[v, u, None] * rays[v, u], np.column_stack((v, u))


def _irls_line(points_xy, centre, normal, camera_xy, iters):
    """Huber-weighted total least squares line, started at the RANSAC line (centre, normal).

    The face is the object's outer layer: lane walls and a neighbouring face's corner lie
    only behind it. Points more than 2.5 scales behind the line (away from the camera) get
    no weight, so they cannot pull the line inward; the scale is a low quantile of |r|
    (|r| < 0.3186 sigma for 25 % of a Gaussian), which stays on the face layer.
    """
    if (camera_xy - centre) @ normal < 0:
        normal = -normal
    scale = 0.002
    for _ in range(iters):
        r = (points_xy - centre) @ normal          # > 0: toward the camera
        scale = max(float(np.percentile(np.abs(r), 25)) / 0.3186, 0.002)
        a = np.abs(r) / (1.345 * scale)
        w = np.where(a <= 1.0, 1.0, 1.0 / np.maximum(a, 1e-12))
        w[(np.abs(r) > 4.0 * scale) | (r < -2.5 * scale)] = 0.0
        if w.sum() <= 0:
            break
        centre = (points_xy * w[:, None]).sum(0) / w.sum()
        d = points_xy - centre
        n = np.linalg.eigh((d * w[:, None]).T @ d / w.sum())[1][:, 0]
        normal = n if n @ normal >= 0 else -n
    r = (points_xy - centre) @ normal
    kept = np.abs(r[(r >= -2.5 * scale) & (r <= 4.0 * scale)])
    return centre, normal, max(float(np.percentile(kept, 25)) / 0.3186 if len(kept) else scale, 0.002)


def _snap_to_layer(points_xy, centre, normal, tol):
    """Move a line onto the densest thin layer of points near it and re-aim it along that layer.

    Twice: histogram the offsets within +-tol, shift to the densest bin, then refit the
    direction from the points within a thin band of it (a few bins wide).
    """
    step = max(0.002, tol / 15)
    for _ in range(2):
        r = (points_xy - centre) @ normal
        r = r[np.abs(r) < tol]
        if len(r) < 2:
            break
        counts, edges = np.histogram(r, bins=np.arange(-tol, tol + step, step))
        k = int(np.argmax(np.convolve(counts, np.ones(3), mode="same")))
        centre = centre + normal * float((edges[k] + edges[k + 1]) / 2)
        band = points_xy[np.abs((points_xy - centre) @ normal) < 1.5 * step]
        if len(band) < 2:
            break
        centre = band.mean(0)
        d = band - centre
        n = np.linalg.eigh(d.T @ d)[1][:, 0]
        normal = n if n @ normal >= 0 else -n
    return centre, normal


def _reach(camera, centre_xy) -> float:
    return float(np.hypot(*(camera[:2] - centre_xy)))


def _margin(params, reach) -> float:
    return max(params.ray_margin_m, params.ray_margin_k * reach * reach)


def fit_faces(points, camera, params: FitParams, rng) -> list[FacePlane]:
    """Up to max_faces exterior vertical planes, largest first.

    RANSAC on two-point vertical lines and a Huber refit, both on the points below the
    top surface (seen from above, the top surface would otherwise give the largest
    line). Each face then takes every remaining point within its band, top included.
    A plane whose robust scale exceeds the noise allowance is a loose bundle of points;
    a plane with more than exterior_max_fraction of the object's points on the camera
    side of it is an interior plane (block sides seen through a lane); a second plane that
    does not meet the first at a right angle is neither of a box's two visible faces. All
    three are dropped.
    """
    faces = []
    if len(points) < params.min_points:
        return faces
    zmax = float(points[:, 2].max())
    body = points[:, 2] < zmax - params.top_exclude_m
    # The inlier band widens with the noise at the object's range (sigma grows with range^2).
    reach0 = float(np.median(np.hypot(*(points[:, :2] - camera[:2]).T)))
    tol = max(params.ransac_tol_m, 0.5 * params.ray_margin_k * reach0 * reach0)
    remaining = np.arange(len(points))
    for _ in range(params.max_faces + 2):
        if len(faces) == params.max_faces:
            break
        fit_from = remaining[body[remaining]]
        if len(fit_from) < params.min_points:
            break
        sub = points[fit_from, :2]
        best = None
        for _ in range(params.ransac_iters):
            i, j = rng.choice(len(sub), 2, replace=False)
            d = sub[j] - sub[i]
            length = float(np.hypot(*d))
            if length < 0.05:
                continue
            n = np.array([-d[1], d[0]]) / length
            inliers = np.abs((sub - sub[i]) @ n) < tol
            count = int(inliers.sum())
            if best is None or count > best[0]:
                best = (count, inliers, sub[i], n)
        if best is None or best[0] < params.min_points:
            break
        _, inliers, c0, n0 = best
        # The line with the most points in the band can sit inside the face, slightly tilted,
        # covering the face and the first centimetres of the lane walls behind it.
        c0, n0 = _snap_to_layer(sub, c0, n0, tol)
        near = np.abs((sub - c0) @ n0) < tol
        centre, normal, scale = _irls_line(sub[near], c0, n0, camera[:2], params.irls_iters)
        on = np.abs((points[remaining, :2] - centre) @ normal) < max(tol, 2.5 * scale)
        index = remaining[on]
        if len(index) < params.min_points:
            break
        remaining = remaining[~on]
        if (camera[:2] - centre) @ normal < 0:
            normal = -normal
        reach = _reach(camera, centre)
        if scale > max(params.max_face_scale_m, 0.5 * params.ray_margin_k * reach * reach):
            continue
        margin = max(params.exterior_margin_m, 3 * scale, params.ray_margin_k * reach * reach)
        if ((points[:, :2] - centre) @ normal > margin).mean() > params.exterior_max_fraction:
            continue
        # A pallet is a box: a second visible face meets the first at a right angle.
        if faces and abs(abs(float(normal @ faces[0].normal_xy)) - 0.0) > math.sin(params.corner_tol_rad):
            continue
        faces.append(FacePlane(centre, normal, scale, index))
    return faces


def face_coords(face: FacePlane, points):
    d = points[:, :2] - face.centre_xy
    return d @ face.left_xy, points[:, 2]


def ray_samples(scene: SceneInput, rays, face: FacePlane, u_range, h_range, params: FitParams, mask=None):
    """(u, h, class, owned) of every pixel whose ray meets the face plane inside the window.

    The class compares the measured axial depth d with the plane crossing t:
    BEHIND if d > t + m, FRONT if d < t - m, FACE otherwise, UNOBSERVED if d is NaN,
    with m = max(ray_margin_m, ray_margin_k * t^2) (the noise grows with t^2).
    owned marks the pixels of the proposal mask (all False without a mask).
    """
    cam = camera_xyz(scene)
    n = face.normal_xy
    den = rays[..., :2] @ n
    with np.errstate(divide="ignore", invalid="ignore"):
        t = ((face.centre_xy - cam[:2]) @ n) / den
    hit = np.isfinite(t) & (t > 0)
    p = cam + np.where(hit, t, 0.0)[..., None] * rays
    u = (p[..., :2] - face.centre_xy) @ face.left_xy
    h = p[..., 2]
    inside = hit & (u >= u_range[0]) & (u <= u_range[1]) & (h >= h_range[0]) & (h <= h_range[1])
    d, tt = scene.depth_m[inside], t[inside]
    margin = np.maximum(params.ray_margin_m, params.ray_margin_k * tt * tt)
    cls = np.full(len(tt), FACE, np.int8)
    cls[d > tt + margin] = BEHIND
    cls[d < tt - margin] = FRONT
    cls[~np.isfinite(d)] = UNOBSERVED
    owned = np.zeros(len(tt), bool) if mask is None else np.asarray(mask, bool)[inside]
    return u[inside], h[inside], cls, owned
```

- [ ] **Step 4: 통과 확인** — Run: Step 2 와 같음 · Expected: 11 개 통과. 관문별 시험(`test_each_face_gate_removes_a_wrong_face_on_its_own`)은 각 관문을 끈 결과도 시험 안에서 확인하므로, 관문을 지우면 그 시험이 실패한다.

- [ ] **Step 5: 커밋**

```bash
git add src/forklift_core/perception/instance_fit.py tests/unit/perception/test_instance_fit.py
git commit -m "perception: instance fit faces -- exterior vertical planes from a proposal's eroded pixels (RANSAC, Huber refit without the top surface, interior block-side planes dropped) and raw-ray classes against a face (behind, face, front, unobserved; margin grows with range squared)"
```

---

### Task 3: 형상 틀 격자와 (가로, 받침면) 탐색

**Files:**
- Modify: `src/forklift_core/perception/instance_fit.py`
- Test: `tests/unit/perception/test_instance_fit.py`

**Interfaces:**
- Consumes: `PalletSpec`, `face_layout`, `spec_boxes`(`pallet_model.py`); 작업 2 의 `FitParams`, `ray_samples`, `face_coords`
- Produces:
  - `face_raster(spec, axis, bin_m, *, mirror=False) -> NDArray[bool] (nh, nu)` — 행 h 는 받침면 0 부터, 열 u 는 −W/2(오른쪽)부터 +W/2(왼쪽)
  - `Placement2D(score, u_centre, support_z)`
  - `best_placement(occ, pts_u, pts_h, rays, params, h_lo, n_shift) -> Placement2D`

- [ ] **Step 1: 실패하는 시험 추가**

```python
# ---- Task 3: rasters and the placement search ---------------------------------------------------

def test_an_x_face_raster_has_open_lanes_down_to_the_support():
    occ = F.face_raster(EPAL, "x", 0.005)
    lane = occ[:int(0.10 / 0.005), int((0.4 + 0.18625) / 0.005)]       # left opening, below the ceiling
    assert occ.shape == (round(EPAL.overall_height_m / 0.005), round(0.8 / 0.005))
    assert not lane.any() and occ[int(0.11 / 0.005)].all()              # lane open to the floor, stringer row full
    assert not occ[-1].all()                                            # the top deck shows the gaps between boards
    assert occ[:, int(0.4 / 0.005)].all()                               # the middle column is solid


def test_a_y_face_lane_reaches_up_to_the_top_deck():
    occ = F.face_raster(T11, "y", 0.005)
    col = occ[:, occ.shape[1] // 2 + int(0.145 / 0.005)]               # a y-face opening column (centre 0.145 m)
    rows = np.nonzero(~col)[0] * 0.005
    assert rows.min() == pytest.approx(T11.deck_bottom_m, abs=0.005)
    assert rows.max() == pytest.approx(T11.deck_bottom_m + T11.opening_height_m + T11.stringer_m, abs=0.006)


def _evidence(scene, ids, k=0):
    p = F.FitParams()
    rays = F.pixel_rays(scene)
    pts, _ = F.proposal_points(scene, rays, ids == k, p.erode_px)
    face = F.fit_faces(pts, F.camera_xyz(scene), p, np.random.default_rng(0))[0]
    u, h = F.face_coords(face, pts[face.index])
    h_lo = float(np.percentile(h, 2)) - p.support_window_m
    rs_ = F.ray_samples(scene, rays, face, (u.min() - 0.9, u.max() + 0.9), (h_lo, h_lo + 0.4), p, mask=ids == k)
    return face, u, h, rs_, h_lo


def test_the_placement_search_finds_the_pallet_centre_and_support_height():
    scene, ids = _render([(EPAL, rt.Placement(2.8, -0.2, 0.15, 0.0), ())])
    face, u, h, rays, h_lo = _evidence(scene, ids)
    best = F.best_placement(F.face_raster(EPAL, "x", 0.005), u, h, rays, F.FitParams(), h_lo, 13)
    centre = np.array([2.8, -0.2]) - 0.3 * np.array([math.cos(0.15), math.sin(0.15)])
    assert (face.centre_xy + face.left_xy * best.u_centre - centre) @ face.left_xy == pytest.approx(0.0, abs=0.005)
    assert best.support_z == pytest.approx(0.0, abs=0.005) and best.score > 0.8


def test_the_right_shape_scores_higher_than_the_wrong_one():
    for spec, other in ((EPAL, T11), (T11, EPAL)):
        scene, ids = _render([(spec, rt.Placement(2.6, 0.0, -0.1, 0.0), ())])
        _, u, h, rays, h_lo = _evidence(scene, ids)
        right = F.best_placement(F.face_raster(spec, "x", 0.005), u, h, rays, F.FitParams(), h_lo, 13)
        wrong = F.best_placement(F.face_raster(other, "x", 0.005), u, h, rays, F.FitParams(), h_lo, 13)
        assert right.score > wrong.score + 0.2


@pytest.mark.parametrize("spec", [EPAL, T11])
def test_rays_never_add_score(spec):
    """Removing rays of either class can only raise the best score, so no placement can earn
    score over empty background (Codex 2026-10-16: the first prototype rewarded BEHIND rays)."""
    scene, ids = _render([(T11, rt.Placement(2.6, 0.0, 0.0, 0.0), ())])
    _, u, h, rays, h_lo = _evidence(scene, ids)
    occ = F.face_raster(spec, "x", 0.005)
    full = F.best_placement(occ, u, h, rays, F.FitParams(), h_lo, 13).score
    for cls in (F.BEHIND, F.FACE):
        keep = rays[2] != cls
        fewer = tuple(a[keep] for a in rays)
        assert F.best_placement(occ, u, h, fewer, F.FitParams(), h_lo, 13).score >= full - 1e-12
```

주: T11 y 면 개구는 (−0.25, −0.04)·(0.04, 0.25), 중심 ±0.145 m, 차선 0.015–0.075 m(하판 위 ~ 스트링거 위)이고 x 면 개구 띠는 0.015–0.06 m 다(2026-10-16 `face_layout` 재계산). 두 면의 부재 배치가 같아 x·y 가설은 받침면 위 15 mm 스트링거 띠로만 갈리고, 포켓 높이 차가 7.5 mm 라 동치(20 mm)로 묶인다.

- [ ] **Step 2: 실패 확인** — Run: `PYTHONPATH=src:. python -m pytest tests/unit/perception/test_instance_fit.py -q -p no:cacheprovider` · Expected: 새 시험 6 개가 `AttributeError: module ... has no attribute 'face_raster'` 로 실패.

- [ ] **Step 3: 구현** — `instance_fit.py` 에 추가:

```python
def face_raster(spec: PalletSpec, axis: str, bin_m: float, *, mirror: bool = False):
    """Occupancy of an entry face seen along its axis (every box projected through the depth).

    Rows are h from 0 at the support surface; columns are u from -W/2 (right) to +W/2
    (left). Entering an x face, left is +y; entering a y face, left is -x. Lanes are
    through-holes, so the projection is what the face shows.
    """
    layout = face_layout(spec, axis)
    width = layout.width_m
    nu, nh = int(round(width / bin_m)), int(round(spec.overall_height_m / bin_m))
    uc = -width / 2 + (np.arange(nu) + 0.5) * bin_m
    hc = (np.arange(nh) + 0.5) * bin_m
    occ = np.zeros((nh, nu), bool)
    for box in spec_boxes(spec):
        (cx, cy, cz), (sx, sy, sz) = box.centre_m, box.size_m
        lo, hi = (cy - sy / 2, cy + sy / 2) if axis == "x" else (-cx - sx / 2, -cx + sx / 2)
        rows = (hc >= cz - sz / 2) & (hc <= cz + sz / 2)
        cols = (uc >= lo) & (uc <= hi)
        occ[np.ix_(rows, cols)] = True
    return occ[:, ::-1] if mirror else occ


def damageable_rows(spec: PalletSpec, bin_m: float):
    """Raster rows that allowed damage may empty: the top-deck band (design §5.2 removes 1-2 top
    boards, edge boards included, or breaks one off). Seen-through rays there are not evidence of
    a different shape; the blocks, stringers and bottom boards stay (chips and splits are small)."""
    nh = int(round(spec.overall_height_m / bin_m))
    hc = (np.arange(nh) + 0.5) * bin_m
    return hc >= spec.overall_height_m - spec.top_thickness_m


def _hist(u, h, u0, h0, nu, nh, bin_m):
    iu = np.floor((u - u0) / bin_m).astype(int)
    ih = np.floor((h - h0) / bin_m).astype(int)
    ok = (iu >= 0) & (iu < nu) & (ih >= 0) & (ih < nh)
    out = np.zeros((nh, nu))
    np.add.at(out, (ih[ok], iu[ok]), 1.0)
    return out


def _correlate_rows(hist, kernel, row_shift):
    total = None
    for i in range(kernel.shape[0]):
        row = np.correlate(hist[row_shift + i], kernel[i], mode="valid")
        total = row if total is None else total + row
    return total


@dataclass(frozen=True)
class Placement2D:
    score: float
    u_centre: float
    support_z: float


def best_placement(occ, pts_u, pts_h, rays, params: FitParams, h_lo: float, n_shift: int,
                   damageable=None) -> Placement2D:
    """Exhaustive search of a face raster over u and support height.

    Per mask point: +1 on material, -hole_point_weight in a hole, -outside_weight outside
    the raster. Per raw ray: -face_ray_weight if it stops on the face inside a hole,
    -seen_through_weight if it passes behind the face where the raster has material outside the
    damageable rows (a missing or broken top board is allowed damage, §5.2; §7.4 does not
    penalise it).
    Rays never add. The sum is divided by the number of face points. The support height
    runs over h_lo + s * bin_m for s < n_shift.
    """
    b = params.bin_m
    nh_t, nu_t = occ.shape
    u0 = float(pts_u.min()) - nu_t * b
    nu = int(math.ceil((float(pts_u.max()) - float(pts_u.min())) / b)) + 2 * nu_t + 1
    nh = nh_t + n_shift + 8
    ru, rh, rc = rays[0], rays[1], rays[2]
    pm = _hist(pts_u, pts_h, u0, h_lo, nu, nh, b)
    rb = _hist(ru[rc == BEHIND], rh[rc == BEHIND], u0, h_lo, nu, nh, b)
    rf = _hist(ru[rc == FACE], rh[rc == FACE], u0, h_lo, nu, nh, b)
    k_pts = np.where(occ, 1.0 + params.outside_weight, params.outside_weight - params.hole_point_weight)
    solid = occ if damageable is None else occ & ~np.asarray(damageable)[:, None]
    k_b = np.where(solid, -params.seen_through_weight, 0.0)
    k_f = np.where(occ, 0.0, -params.face_ray_weight)
    n_pts = len(pts_u)
    best = None
    for s in range(n_shift):
        score = (_correlate_rows(pm, k_pts, s) + _correlate_rows(rb, k_b, s) + _correlate_rows(rf, k_f, s)
                 - params.outside_weight * n_pts) / n_pts
        j = int(np.argmax(score))
        if best is None or score[j] > best.score:
            best = Placement2D(float(score[j]), u0 + (j + nu_t / 2) * b, h_lo + s * b)
    return best
```

(점수 항등식: 각 점의 기본값 −outside 에 재료 칸이면 1 + outside, 구멍 칸이면 outside − hole 을 더하므로 재료 +1, 구멍 −hole, 격자 밖 −outside 가 된다. 탐색 창의 높이는 틀 높이 + n_shift + 8 칸이라 틀 위로 벗어난 점은 격자 밖으로 센다.)

- [ ] **Step 4: 통과 확인** — Run: Step 2 와 같음 · Expected: 작업 2–3 시험 17 개 통과.

- [ ] **Step 5: 커밋**

```bash
git add src/forklift_core/perception/instance_fit.py tests/unit/perception/test_instance_fit.py
git commit -m "perception: instance fit rasters -- entry-face occupancy from the shape's boxes and an exhaustive (lateral, support height) search scored by mask points and subtractive raw-ray evidence"
```

---

### Task 4: 가설 경쟁·받침면·자세 구속·후보 조립

**Files:**
- Modify: `src/forklift_core/perception/instance_fit.py`
- Test: `tests/unit/perception/test_instance_fit.py`

**Interfaces:**
- Consumes: 작업 1–3; `fork_fit`, `ForkSetup`, `LoadDeclaration`(`fork_fit.py`); `InstanceProposal`; `PocketCandidate`, `PocketEstimate`; `ObservedPalletShape`
- Produces:
  - `Hypothesis(label, spec, axis, mirror, members, band, placement, face)`
  - `known_hypotheses(specs, face_index, pts_u, pts_h, rays, params, h_lo, n_shift) -> list[Hypothesis]`
  - `observed_members(hyp, rays, params, window_m) -> int`
  - `damageable_rows(spec, bin_m)` — 허용 파손이 비울 수 있는 상판 띠의 격자 행(§5.2); 점수의 '재료 자리 뚫림' 감점과 형상 증거가 이 행을 뺀다
  - `shape_evidence(hyp, pts_u, pts_h, rays, params) -> {"outside", "top_gap_m", "seen_through"}` — 최고 가설이 `fitted` 여도 `outside > shape_outside_max(0.08)` 또는 `seen_through > shape_seen_through_max(0.12)` 면 `ambiguous(shape_mismatch)`. 알려진 형상이 이 면을 설명하지 못한다는 뜻이고, 2B 의 매개변수형(비규격) 가설로 넘어갈 후보다
  - `support_state(scene, rays, mask, face, support_z, params) -> (state, measured | None)` — `visible` / `coplanar` / `physical_edge` / `uncertain`; `visible` 이면 측정 높이를 함께 내고 `fit_instance` 가 그 높이로 틀을 다시 놓는다(§7.3). `uncertain` 이면 높이를 읽는 끼움 사유(`HEIGHT_REASONS`)를 빼고 `pickable` 은 남은 사유가 있으면 False, 없으면 None, 사유에 `support_uncertain`
  - `fit_instance(scene, proposal, *, specs: dict[str, PalletSpec], setup: ForkSetup, load_for: Callable[[PalletSpec], LoadDeclaration | None], params=FitParams(), rng=None, candidate_id="c0", rays=None, identity_threshold=None) -> PocketCandidate | None`
  - `FitFailure(proposal_id, reason, centre_xy | None, extent_m)`(관측했지만 맞추지 못한 물체 — `centre_xy` 는 제안 깊이 점의 xy 중앙값, `extent_m` 은 그 점들의 최대 거리), `SceneFit(candidates: tuple[PocketCandidate], failures: tuple[FitFailure])`
  - `fit_scene(scene, proposals, *, specs, setup, load_for, params=FitParams(), identity_threshold=None) -> SceneFit` — 후보 ID `c<k>`(k = 제안 순번), 난수 `default_rng(params.seed)` 하나를 순서대로 쓴다. 제안마다 예외를 격리해 `exception:<type>` 으로, 면이 없으면 `no_face` 로 `failures` 에 남긴다(§10)

- [ ] **Step 1: 실패하는 시험 추가**

```python
# ---- Task 4: hypotheses, support, candidates ----------------------------------------------------

def _fit(items, k=0, *, source="oracle", noise=None, distractors=(), depth=None, spacing=None):
    scene, ids = _render(items, noise=noise, distractors=distractors, depth=depth)
    spec = items[k][0]
    setup = ForkSetup(spacing or SPACING[spec.name])
    cand = F.fit_instance(scene, InstanceProposal("p0", ids == k, 1.0, source), specs=SPECS, setup=setup,
                          load_for=lambda s: SITE.get(s.name))
    truths = rt.scene_truths([(s, p, d, SITE[s.name]) for s, p, d in items], setup)[0]
    return cand, truths[k]


def _errors(cand, truth):
    best = None
    for face in truth.faces:
        if not face.accessible:
            continue
        a = np.array([math.cos(face.insertion_yaw_rad), math.sin(face.insertion_yaw_rad), 0.0])
        n = np.array([-a[1], a[0], 0.0])
        dl = np.subtract(cand.left.centre_m, face.left_m)
        dr = np.subtract(cand.right.centre_m, face.right_m)
        yaw = abs((cand.insertion_yaw_rad - face.insertion_yaw_rad + math.pi) % (2 * math.pi) - math.pi)
        e = (max(abs(dl @ n), abs(dr @ n)), max(abs(dl @ a), abs(dr @ a)), yaw, max(abs(dl[2]), abs(dr[2])))
        if best is None or sum(e[:2]) < sum(best[:2]):
            best = e
    return best


@pytest.mark.parametrize("spec, placement", [
    (EPAL, rt.Placement(3.0, 0.4, 0.3, 0.0)),
    (EPAL, rt.Placement(2.2, -0.5, -0.45, 0.0)),
    (T11, rt.Placement(2.6, 0.0, 0.2, 0.0)),
    (T11, rt.Placement(2.4, 0.3, math.pi / 2 + 0.2, 0.0)),   # the y face toward the camera
])
def test_a_clean_known_pallet_is_fitted_within_the_pocket_tolerances(spec, placement):
    cand, truth = _fit([(spec, placement, ())])
    assert cand.geometry == "fitted" and cand.counted and cand.hypothesis.startswith(spec.name)
    lateral, axial, yaw, z = _errors(cand, truth)
    assert lateral <= 0.010 and axial <= 0.010 and yaw <= math.radians(0.5) and z <= 0.006
    assert cand.support_state == "visible" and cand.on_floor
    assert cand.pallet_centre_m[:2] == pytest.approx((placement.x_m, placement.y_m), abs=0.01)
    # T11 entered through a y face has no roof board edges along the forks: near_tracking_unavailable
    assert cand.pickable == any(f.fit["pickable"] for f in truth.faces if f.accessible and f.axis == cand.axis)


def test_left_is_the_larger_lateral_pocket():
    cand, _ = _fit([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0), ())])
    assert cand.left.centre_m[1] > cand.right.centre_m[1]


def test_a_damaged_pallet_is_still_fitted():
    damage = (DamageOp("top_board_missing", "top_board_0", ()), DamageOp("top_board_missing", "top_board_3", ()))
    cand, truth = _fit([(EPAL, rt.Placement(2.7, 0.1, 0.2, 0.0), damage)])
    assert cand.geometry == "fitted"
    lateral, axial, yaw, _ = _errors(cand, truth)
    assert lateral <= 0.010 and axial <= 0.010 and yaw <= math.radians(0.5)


@pytest.mark.parametrize("x_m, boards", [(2.6, ("top_board_6",)), (2.6, ("top_board_5", "top_board_6")),
                                         (2.04, ("top_board_5", "top_board_6"))])
def test_missing_edge_boards_are_damage_not_a_different_shape(x_m, boards):
    """Codex 2026-10-16: a T11 seen through its y face with its edge board missing read as
    shape_mismatch (seen-through 0.278), with two adjacent boards missing as well (0.238), and
    the vertical search drifted 12 mm, adding a false lift_alignment_needed; at 2.04 m the floor
    just below the edge is only 22 mm in front of the face and was taken for the face. Allowed damage
    (design §5.2: 1-2 top boards, edge boards included) must fit like an intact pallet."""
    damage = tuple(DamageOp("top_board_missing", b, ()) for b in boards)
    cand, truth = _fit([(T11, rt.Placement(x_m, 0.0, math.pi / 2, 0.0), damage)])
    assert cand.geometry == "fitted" and cand.axis == "y" and cand.support_state == "visible"
    lateral, axial, yaw, z = _errors(cand, truth)
    assert lateral <= 0.010 and axial <= 0.010 and yaw <= math.radians(0.5) and z <= 0.006
    assert cand.support_z_m == pytest.approx(0.0, abs=0.006) and cand.on_floor
    face = next(f for f in truth.faces if f.accessible and f.axis == "y")
    assert cand.fit_reasons == tuple(face.fit["reasons"])


def test_the_verdict_belongs_to_the_returned_placement():
    """Codex 2026-10-16: the support correction moved the placement after the gates had run, so a
    shape_mismatch (or a pass of min_score) of the old placement came back with the new one."""
    from dataclasses import replace
    damage = (DamageOp("top_board_missing", "top_board_5", ()), DamageOp("top_board_missing", "top_board_6", ()))
    noisy, _ = _fit([(T11, rt.Placement(4.0, 0.2, math.pi / 2, 0.0), damage)], noise=_mid_noise(3))
    assert noisy.geometry != "ambiguous" or "shape_mismatch" not in noisy.reasons
    scene, ids = _render([(T11, rt.Placement(4.0, 0.0, math.pi / 2, 0.0), damage)])
    strict = replace(F.FitParams(), min_score=0.6)
    cand = F.fit_instance(scene, InstanceProposal("p0", ids == 0, 1.0, "oracle"), specs=SPECS,
                          setup=ForkSetup(0.290), load_for=lambda s: SITE.get(s.name), params=strict)
    assert (cand.geometry == "unfit") == (cand.score < 0.6)


def test_a_switch_to_the_other_face_gets_that_faces_support_too():
    """Codex 2026-10-16: the first pass chose the y face (support 5.7 mm), fixing it moved the choice
    to the x face (3.2 mm uncorrected) and the second pass stopped there -- the x face came back
    pickable at 0.80 rad although every face fixed gives the y face, not pickable, at -0.77 rad."""
    damage = (DamageOp("top_board_missing", "top_board_5", ()), DamageOp("top_board_missing", "top_board_6", ()))
    cand, truth = _fit([(T11, rt.Placement(2.6, -0.2, 0.8, 0.0), damage)])
    # The expected end state is stated outright: matching the truth of whatever axis came back
    # would accept the wrong face (Codex 2026-10-16, round 6).
    assert cand.geometry == "fitted" and cand.axis == "y" and cand.support_state == "visible"
    assert cand.insertion_yaw_rad == pytest.approx(-0.7708, abs=0.02)
    assert cand.support_z_m == pytest.approx(0.0, abs=0.002)
    assert cand.fit_reasons == ("near_tracking_unavailable",) and cand.pickable is False


def test_a_pallet_hidden_but_for_one_block_is_not_fitted():
    """An occluder, not a cut mask: raw rays would still see members hidden only from the mask."""
    occluder = scene_rig.Box((1.6, -0.15, 0.15), (0.1, 0.6, 0.3))   # silhouette reaches y = 0.272 at the face
    cand, _ = _fit([(EPAL, rt.Placement(2.5, 0.0, 0.0, 0.0), ())], distractors=[occluder])
    assert cand is None or cand.geometry != "fitted"


@pytest.mark.parametrize("seen, geometry", [(1, "ambiguous"), (2, "fitted")])
def test_fewer_than_two_members_with_a_seen_edge_leave_the_lateral_position_unconstrained(monkeypatch, seen, geometry):
    """The gate itself (Codex 2026-10-16: the occluder test passes on other gates even when
    observed_members always says 99)."""
    monkeypatch.setattr(F, "observed_members", lambda *a, **k: seen)
    cand, _ = _fit([(EPAL, rt.Placement(2.6, 0.0, 0.0, 0.0), ())])
    assert cand.geometry == geometry and (geometry == "fitted" or "lateral_unconstrained" in cand.reasons)


def test_a_member_edge_counts_only_with_the_proposals_own_face_rays():
    hyp = F.Hypothesis("t:x", EPAL, "x", False, ((-0.1, 0.0), (0.2, 0.3)), (0.0, 0.1),
                       F.Placement2D(1.0, 0.0, 0.0), 0)
    u = np.array([-0.095, -0.094, -0.093, -0.105, -0.106, -0.107, 0.205, 0.206, 0.207, 0.195, 0.194, 0.193])
    h = np.full(u.shape, 0.05)
    cls = np.array([F.FACE] * 3 + [F.BEHIND] * 3 + [F.FACE] * 3 + [F.BEHIND] * 3, np.int8)
    for owned, expected in ((np.zeros(u.shape, bool), 0), (np.ones(u.shape, bool), 2)):
        assert F.observed_members(hyp, (u, h, cls, owned), F.FitParams(), 0.015) == expected
    far = u + np.where(u < 0.05, -0.05, 0.05) * (cls == F.BEHIND)   # behind rays 50 mm outside the window
    assert F.observed_members(hyp, (far, h, cls, np.ones(u.shape, bool)), F.FitParams(), 0.015) == 0


def test_a_hidden_lower_edge_leaves_the_support_uncertain_and_the_fit_undecided():
    """Codex 2026-10-16: a low front occluder lifted the support search to 10.8 mm and the
    candidate still came out pickable."""
    occluder = scene_rig.Box((1.8, 0.0, 0.06), (0.05, 1.5, 0.12))
    cand, _ = _fit([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0), ())], distractors=[occluder])
    assert cand.support_state == "uncertain" and "support_uncertain" in cand.reasons
    assert cand.pickable is None and not cand.on_floor


def test_a_stack_reports_support_heights_and_how_each_edge_was_seen():
    H = EPAL.overall_height_m
    items = [(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0), ()), (EPAL, rt.Placement(2.8, 0.0, 0.0, H), ()),
             (EPAL, rt.Placement(2.8, 0.0, 0.0, 2 * H), ())]
    states = []
    for k in range(3):
        cand, _ = _fit(items, k)
        assert cand.support_z_m == pytest.approx(k * H, abs=0.006)
        states.append(cand.support_state)
    # The top support (0.288 m) is above the camera (0.27 m): its surface cannot be seen (truth: support_visible False)
    assert states == ["visible", "coplanar", "coplanar"]


def test_x_and_y_readings_of_a_t11_face_compete_instead_of_merging():
    """Codex 2026-10-16: with depth only in 20-55 mm the x and y readings scored 0.940 / 0.943 and
    their pockets were 17.5 mm apart; merging them hid a choice between different fork fits."""
    def band_only(out):
        d = out.scene.depth_m.copy()
        z = out.points_base[..., 2]
        d[(out.instance_ids == 0) & ((z < 0.020) | (z > 0.055))] = np.nan
        return d
    cand, _ = _fit([(T11, rt.Placement(2.6, 0.0, math.pi / 2, 0.0), ())], depth=band_only)
    assert cand.geometry == "ambiguous" and any(r.startswith("competing:") for r in cand.reasons)
    clean, _ = _fit([(T11, rt.Placement(2.6, 0.0, math.pi / 2, 0.0), ())])
    assert clean.geometry == "fitted" and clean.axis == "y"


@pytest.mark.parametrize("scene_id", ["s7_nonstandard_1000738", "s7_nonstandard_1000759"])
def test_a_non_standard_pallet_is_not_fitted_as_a_known_shape(scene_id):
    """Rig renders of S7 dev recipes (0.77 m wide with four blocks; 0.99 m wide) that the EPAL raster
    fits when the shape check is off (ws1 dev 2026-10-16: 76 of 135 S7 pallets came out fitted)."""
    from dataclasses import replace
    recipe = next(r for r in rs.catalogue() if r.scene_id == scene_id)
    out = instance_rig.render_instances([rs.recipe_boxes(recipe, 0)])
    proposal = InstanceProposal("p0", out.instance_ids == 0, 1.0, "oracle")
    kw = dict(specs=SPECS, setup=ForkSetup(recipe.fork_spacing_m), load_for=lambda s: SITE.get(s.name))
    off = replace(F.FitParams(), shape_outside_max=9.0, shape_seen_through_max=9.0)
    assert F.fit_instance(out.scene, proposal, params=off, **kw).geometry == "fitted"
    cand = F.fit_instance(out.scene, proposal, **kw)
    assert cand.geometry == "ambiguous" and "shape_mismatch" in cand.reasons


def test_a_geometric_proposal_stays_unconfirmed():
    cand, _ = _fit([(EPAL, rt.Placement(2.8, 0.0, 0.0, 0.0), ())], source="geometric")
    assert cand.geometry == "fitted" and cand.identity == "unconfirmed" and not cand.counted


def test_pickability_follows_the_fork_fit_of_the_fitted_shape():
    cand, _ = _fit([(T11, rt.Placement(2.6, 0.0, 0.0, 0.0), ())], spacing=0.3725)
    assert cand.pickable is False and "fork_spacing_adjust" in cand.fit_reasons


def test_noise_far_away_still_gives_a_candidate_with_the_right_shape():
    cand, _ = _fit([(T11, rt.Placement(3.6, 0.2, 0.1, 0.0), ())], noise=_mid_noise(5))
    assert cand is not None and cand.hypothesis.startswith(T11.name)


def test_an_over_segmented_mask_is_still_fitted():
    """A learned mask spills a few pixels onto the floor and background (Gemini review 2026-10-16).
    Dev check: masks dilated by 4 px gave the same fitted counts and errors on Isaac S1/S2/S5."""
    def dilate(mask, r=4):
        for _ in range(r):
            m = mask.copy()
            m[1:] |= mask[:-1]
            m[:-1] |= mask[1:]
            m[:, 1:] |= mask[:, :-1]
            m[:, :-1] |= mask[:, 1:]
            mask = m
        return mask
    placement = rt.Placement(2.9, 0.2, 0.2, 0.0)
    scene, ids = _render([(EPAL, placement, ())])
    cand = F.fit_instance(scene, InstanceProposal("p0", dilate(ids == 0), 1.0, "oracle"), specs=SPECS,
                          setup=ForkSetup(0.3725), load_for=lambda s: SITE.get(s.name))
    truth = rt.scene_truths([(EPAL, placement, (), SITE[EPAL.name])], ForkSetup(0.3725))[0][0]
    lateral, axial, yaw, z = _errors(cand, truth)
    assert cand.geometry == "fitted" and lateral <= 0.010 and axial <= 0.010 and z <= 0.006


def test_fit_scene_keeps_proposal_order_and_records_failures(monkeypatch):
    scene, ids = _render([(EPAL, rt.Placement(2.6, -0.6, 0.0, 0.0), ()), (EPAL, rt.Placement(2.6, 0.6, 0.0, 0.0), ())])
    tiny = np.zeros(ids.shape, bool)
    tiny[0, 0] = True
    props = [InstanceProposal("a", ids == 0, 1.0, "oracle"), InstanceProposal("b", tiny, 1.0, "oracle"),
             InstanceProposal("c", ids == 1, 1.0, "oracle")]
    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=lambda s: SITE.get(s.name))
    assert [(c.candidate_id, c.proposal_id) for c in out.candidates] == [("c0", "a"), ("c2", "c")]
    assert [(f.proposal_id, f.reason) for f in out.failures] == [("b", "no_face")]
    real = F.fit_instance

    def boom(scene, proposal, **kw):
        if proposal.proposal_id == "a":
            raise FloatingPointError("injected")
        return real(scene, proposal, **kw)

    monkeypatch.setattr(F, "fit_instance", boom)
    out = F.fit_scene(scene, props, specs=SPECS, setup=ForkSetup(0.3725), load_for=lambda s: SITE.get(s.name))
    assert [c.proposal_id for c in out.candidates] == ["c"]
    failed = {f.proposal_id: f for f in out.failures}
    assert failed["a"].reason == "exception:FloatingPointError"
    # The failed pallet keeps an approximate position (its visible surface) for target selection
    assert failed["a"].centre_xy == pytest.approx((2.3, -0.6), abs=0.15)
```

- [ ] **Step 2: 실패 확인** — Run: Step 2 (작업 3) 와 같음 · Expected: 새 시험들이 `AttributeError: ... 'fit_instance'` 로 실패.

- [ ] **Step 3: 구현** — `instance_fit.py` 에 추가:

```python
@dataclass(frozen=True)
class Hypothesis:
    label: str             # "<shape>:<axis>" with ":m" for a mirrored raster
    spec: PalletSpec
    axis: str
    mirror: bool
    members: tuple         # (lo, hi) in u relative to the face centre, left = +u
    band: tuple            # pocket band (z0, z1) above the support
    placement: Placement2D
    face: int


def _mirrored(intervals, mirror):
    return tuple(sorted((-hi, -lo) for lo, hi in intervals)) if mirror else tuple(intervals)


def known_hypotheses(specs: dict, face_index: int, pts_u, pts_h, rays, params, h_lo, n_shift):
    out = []
    for name in sorted(specs):
        spec = specs[name]
        for axis in spec.entry_axes:
            layout = face_layout(spec, axis)
            base = face_raster(spec, axis, params.bin_m)
            for mirror in ((False,) if np.array_equal(base, base[:, ::-1]) else (False, True)):
                occ = base[:, ::-1] if mirror else base
                out.append(Hypothesis(
                    f"{spec.name}:{axis}" + (":m" if mirror else ""), spec, axis, mirror,
                    _mirrored(layout.member_intervals_m, mirror), layout.pocket_band_m,
                    best_placement(occ, pts_u, pts_h, rays, params, h_lo, n_shift,
                                   damageable_rows(spec, params.bin_m)), face_index))
    return out


def observed_members(hyp: Hypothesis, rays, params: FitParams, window_m: float) -> int:
    """Members with a side edge seen in the pocket band: rays owned by the proposal stop on
    the face inside the member, raw rays pass behind it just outside (§7.2: a neighbour's
    face pixels are not this instance's evidence)."""
    zs = hyp.placement.support_z
    ru, rh, rc, owned = rays[:4]
    band = (rh >= zs + hyp.band[0]) & (rh <= zs + hyp.band[1])
    on_face, behind = band & (rc == FACE) & owned, band & (rc == BEHIND)
    seen = 0
    for lo, hi in hyp.members:
        edges = 0
        for edge, inward in ((hyp.placement.u_centre + lo, 1.0), (hyp.placement.u_centre + hi, -1.0)):
            off = inward * (ru - edge)
            inside = on_face & (off >= 0) & (off <= window_m)
            outside = behind & (off < 0) & (off >= -window_m)
            edges += int(inside.sum() >= params.edge_min_hits and outside.sum() >= params.edge_min_hits)
        seen += edges > 0
    return seen


def support_state(scene: SceneInput, rays, mask, face: FacePlane, support_z: float, params: FitParams):
    """How the surface under the object's lower edge was seen (design §7.3).

    Only columns whose lowest object pixel lies on the face at the support height are used
    (under an open lane the lowest pixel is a lane wall behind the face). Two pixels below that
    edge and five pixels below it are compared: on a horizontal surface the height stays the
    same, on a vertical one (a stacked pallet's face) it drops by the pixel step times the range.
    visible -- a level surface at the support height, seen from above it (its measured height
      is returned, §7.3: a visible support gives z_s);
    coplanar -- the face continues down below the edge: the edge may be a stack joint;
    physical_edge -- the rays pass behind the face (nothing under the edge in view);
    uncertain -- too few rays, or something nearer hides the edge.
    Returns (state, measured height or None). Codex 2026-10-16: at 1.3 m the floor just below the
    edge is only 22 mm in front of the face, so a distance band alone took it for the face.
    """
    cam = camera_xyz(scene)
    margin = _margin(params, _reach(cam, face.centre_xy))
    near_rows, far_rows = [], []
    for c in np.nonzero(mask.any(axis=0))[0]:
        bottom = int(np.nonzero(mask[:, c])[0].max())
        d0 = scene.depth_m[bottom, c]
        if not np.isfinite(d0) or bottom + 6 >= mask.shape[0] or mask[bottom + 2:bottom + 7, c].any():
            continue
        p0 = cam + d0 * rays[bottom, c]
        if abs(p0[2] - support_z) > params.support_window_m or abs((p0[:2] - face.centre_xy) @ face.normal_xy) > margin:
            continue
        near_rows.append((bottom + 2, c))
        far_rows.append((bottom + 6, c))
    if len(near_rows) < 5:
        return "uncertain", None
    (v1, c1), (v2, c2) = np.array(near_rows).T, np.array(far_rows).T
    d1, d2 = scene.depth_m[v1, c1], scene.depth_m[v2, c2]
    ok = np.isfinite(d1) & np.isfinite(d2)
    if ok.mean() < 0.5:
        return "uncertain", None
    p1 = cam + d1[ok, None] * rays[v1[ok], c1[ok]]
    p2 = cam + d2[ok, None] * rays[v2[ok], c2[ok]]
    ahead = (p1[:, :2] - face.centre_xy) @ face.normal_xy
    drop = p1[:, 2] - p2[:, 2]                                   # height lost over four pixel rows
    vertical_drop = 4.0 * _reach(cam, face.centre_xy) / scene.intrinsics.fy
    level = np.abs(p1[:, 2] - support_z) <= params.support_window_m
    horizontal = np.median(np.abs(drop)) < 0.4 * vertical_drop
    if horizontal and level.mean() > 0.5 and np.median(ahead) > -margin and cam[2] > support_z + params.support_window_m:
        return "visible", float(np.median(p1[level, 2]))
    if np.median(np.abs(ahead)) <= margin:
        return "coplanar", None
    if np.median(ahead) < -margin:
        return "physical_edge", None
    return "uncertain", None


def shape_evidence(hyp: Hypothesis, pts_u, pts_h, rays, params: FitParams) -> dict:
    """How well a known shape explains the face beyond its score (design §7.4-4, §7.5).

    outside: share of face points outside the shape's face rectangle;
    top_gap_m: observed top of the face (98th percentile of h) minus the shape's top (recorded);
    seen_through: share of the classified rays on the shape's material that pass behind it, the
      damageable top-deck band left out (missing or broken top boards are allowed damage, §5.2;
      Codex 2026-10-16: a missing edge board, or two, must not read as another shape).
    On the stage 1B Isaac dev scenes (fitted candidates), known shapes stayed at outside p99
    <= 0.06 and seen_through p99 <= 0.09 up to the mid noise level (damaged S5 included),
    while 70 % of the non-standard S7 pallets fitted as a known shape exceeded the limits.
    """
    layout = face_layout(hyp.spec, hyp.axis)
    zs, uc = hyp.placement.support_z, hyp.placement.u_centre
    du = pts_u - uc
    w, top = layout.width_m, zs + hyp.spec.overall_height_m
    outside = float(((np.abs(du) > w / 2 + 2 * params.bin_m) | (pts_h > top + 0.02)).mean())
    occ = face_raster(hyp.spec, hyp.axis, params.bin_m, mirror=hyp.mirror)
    ru, rh, rc = rays[0], rays[1], rays[2]
    occ = occ & ~damageable_rows(hyp.spec, params.bin_m)[:, None]
    iu = np.floor((ru - (uc - w / 2)) / params.bin_m).astype(int)
    ih = np.floor((rh - zs) / params.bin_m).astype(int)
    ok = (iu >= 0) & (iu < occ.shape[1]) & (ih >= 0) & (ih < occ.shape[0]) & ((rc == FACE) | (rc == BEHIND))
    mat = np.zeros(len(ru), bool)
    mat[ok] = occ[ih[ok], iu[ok]]
    seen = int(mat.sum())
    return {"outside": outside, "top_gap_m": float(np.percentile(pts_h, 98)) - top,
            "seen_through": float((mat & (rc == BEHIND)).sum() / seen) if seen else 1.0}


def _pockets(hyp: Hypothesis, face: FacePlane, setup, load):
    fit = fork_fit(hyp.spec, hyp.axis, setup, load, support_z_m=max(hyp.placement.support_z, 0.0))
    layout = face_layout(hyp.spec, hyp.axis)
    pockets = []
    for i in fit.pair:
        lo, hi = layout.opening_intervals_m[i]
        pockets.append(((-(lo + hi) if hyp.mirror else (lo + hi)) / 2, hi - lo))
    (r_lat, r_w), (l_lat, l_w) = sorted(pockets)
    centre = face.centre_xy + face.left_xy * hyp.placement.u_centre
    z = hyp.placement.support_z + (hyp.band[0] + hyp.band[1]) / 2
    height = hyp.band[1] - hyp.band[0]
    left = PocketEstimate((*(centre + face.left_xy * l_lat), z), l_w, height, "known_shape", "known_shape")
    right = PocketEstimate((*(centre + face.left_xy * r_lat), z), r_w, height, "known_shape", "known_shape")
    return fit, centre, left, right


def _identity(proposal: InstanceProposal, threshold) -> str:
    if proposal.source == "oracle":
        return "confirmed"
    if proposal.source == "learned" and threshold is not None and proposal.score >= threshold:
        return "confirmed"
    return "unconfirmed"


def _decide(hyps, faces, evidence, pts, scene, setup, load_for, params):
    """Rank the hypotheses and apply every gate: weak fit, rivals, face span, member edges, shape."""
    cam = camera_xyz(scene)
    scored = []
    for hyp in hyps:
        face = faces[hyp.face]
        fit, centre, left, right = _pockets(hyp, face, setup, load_for(hyp.spec))
        scored.append((hyp, fit, centre, left, right, math.atan2(face.insertion_xy[1], face.insertion_xy[0])))
    # Faces compete by total evidence (a face with few points must not win on a normalised
    # score); readings of the same face compete by normalised score.
    sizes = [len(f.index) for f in faces]
    scored.sort(key=lambda s: -s[0].placement.score * sizes[s[0].face])
    best = scored[0]

    def equivalent(a, b):
        # Only the same reading of the same face merges: an x and a y reading of a square T11
        # face act differently (lane height, near tracking) and must compete (design §7.4-7).
        d = max(np.linalg.norm(np.subtract(a[3].centre_m, b[3].centre_m)),
                np.linalg.norm(np.subtract(a[4].centre_m, b[4].centre_m)))
        dyaw = abs((a[5] - b[5] + math.pi) % (2 * math.pi) - math.pi)
        return (a[0].axis == b[0].axis and a[1].reasons == b[1].reasons
                and d <= params.equivalence_m and dyaw <= params.equivalence_yaw_rad)

    rivals = sorted((s for s in scored[1:] if s[0].face == best[0].face and not equivalent(best, s)),
                    key=lambda s: -s[0].placement.score)
    hyp = best[0]
    u, rays_f = evidence[hyp.face]
    reach = _reach(cam, best[2])
    geometry, reasons = "fitted", []
    if hyp.placement.score < params.min_score:
        geometry, reasons = "unfit", ["weak_fit"]
    elif rivals and hyp.placement.score - rivals[0][0].placement.score < params.ambiguity_margin:
        geometry, reasons = "ambiguous", [f"competing:{rivals[0][0].label}"]
    elif u.max() - u.min() < params.min_face_span_m:
        geometry, reasons = "ambiguous", ["yaw_unconstrained"]
    elif observed_members(hyp, rays_f, params, max(params.edge_window_m, 4 * reach / scene.intrinsics.fx)) < 2:
        geometry, reasons = "ambiguous", ["lateral_unconstrained"]
    else:
        ev = shape_evidence(hyp, u, pts[faces[hyp.face].index, 2], rays_f, params)
        if ev["outside"] > params.shape_outside_max or ev["seen_through"] > params.shape_seen_through_max:
            # The best known shape does not explain this face: a candidate for the parametric
            # (non-standard) reading of stage 2B, never a confident known-shape fit.
            geometry, reasons = "ambiguous", ["shape_mismatch"]
    return best, rivals, geometry, reasons


def fit_instance(scene: SceneInput, proposal: InstanceProposal, *, specs: dict, setup, load_for,
                 params: FitParams = FitParams(), rng=None, candidate_id: str = "c0", rays=None,
                 identity_threshold=None) -> PocketCandidate | None:
    """Best known-shape hypothesis for one proposal, or None when no face can be fitted.

    When the support under the chosen face is seen, its measured height fixes z_s (§7.3) and the
    whole decision -- every hypothesis on that face, the ranking and all gates -- is made again
    on it, so the returned placement and its verdict always belong together (Codex 2026-10-16);
    if the new decision moves to another face, that face's support is fixed in turn.
    """
    rng = np.random.default_rng(params.seed) if rng is None else rng
    rays = pixel_rays(scene) if rays is None else rays
    cam = camera_xyz(scene)
    pts, _ = proposal_points(scene, rays, proposal.mask, params.erode_px)
    faces = fit_faces(pts, cam, params, rng)
    if not faces:
        return None
    wmax = max(face_layout(s, a).width_m for s in specs.values() for a in s.entry_axes)
    hmax = max(s.overall_height_m for s in specs.values())
    n_shift = int(round(2 * params.support_window_m / params.bin_m)) + 1
    evidence, lows = [], []
    for face in faces:
        u, h = face_coords(face, pts[face.index])
        h_lo = float(np.percentile(h, 2)) - params.support_window_m
        rays_f = ray_samples(scene, rays, face, (u.min() - wmax - 0.05, u.max() + wmax + 0.05),
                             (h_lo, h_lo + hmax + 2 * params.support_window_m + 0.1), params, mask=proposal.mask)
        evidence.append((u, rays_f))
        lows.append(h_lo)
    fixed = {}
    # Each pass fixes the support of the face it chose; it ends when the chosen face is already
    # fixed or has no measured support. A face is fixed at most once, so len(faces) + 1 passes
    # always end on a decision made with every fix applied (Codex 2026-10-16: with two passes a
    # switch to the other face returned that face uncorrected, 90 deg off and the wrong fork fit).
    for _ in range(len(faces) + 1):
        hyps = []
        for fi, face in enumerate(faces):
            u, rays_f = evidence[fi]
            h_lo, shifts = (fixed[fi], 1) if fi in fixed else (lows[fi], n_shift)
            hyps += known_hypotheses(specs, fi, u, pts[face.index, 2], rays_f, params, h_lo, shifts)
        best, rivals, geometry, reasons = _decide(hyps, faces, evidence, pts, scene, setup, load_for, params)
        hyp = best[0]
        face = faces[hyp.face]
        state, measured = support_state(scene, rays, proposal.mask, face, hyp.placement.support_z, params)
        if hyp.face in fixed or measured is None or abs(measured - hyp.placement.support_z) <= params.bin_m / 2:
            break
        fixed[hyp.face] = measured
    hyp, fit, centre, left, right, yaw = best
    reach = _reach(cam, centre)
    pickable, fit_reasons = fit.pickable, tuple(fit.reasons)
    if state == "uncertain":
        # An unseen support may not decide anything that reads the height (§7.3).
        fit_reasons = tuple(r for r in fit_reasons if r not in HEIGHT_REASONS)
        pickable = False if fit_reasons else None
        reasons = list(reasons) + ["support_uncertain"]
    layout = face_layout(hyp.spec, hyp.axis)
    pallet_centre = centre + face.insertion_xy * layout.depth_m / 2
    shape = ObservedPalletShape(hyp.spec, layout, (("face", "known_shape"), ("depth", "known_shape"),
                                                    ("support", "unobserved" if state == "uncertain" else "observed")))
    zs = hyp.placement.support_z
    return PocketCandidate(
        candidate_id, proposal.proposal_id, proposal.source, scene.stamp_ns, hyp.label, hyp.axis,
        tuple((s[0].label, round(s[0].placement.score, 4)) for s in rivals[:3]), yaw,
        (float(centre[0]), float(centre[1]), float(zs)), (float(pallet_centre[0]), float(pallet_centre[1]), float(zs)),
        left, right, zs, state, state == "visible" and abs(zs) <= params.support_window_m, reach,
        hyp.placement.score, geometry, _identity(proposal, identity_threshold), tuple(reasons), pickable,
        fit_reasons, shape)


@dataclass(frozen=True)
class FitFailure:
    proposal_id: str
    reason: str                  # no_face, or exception:<type> (§10)
    centre_xy: tuple | None      # median base-frame xy of the proposal's depth points, if any
    extent_m: float = 0.0        # largest distance of those points from centre_xy


@dataclass(frozen=True)
class SceneFit:
    candidates: tuple
    failures: tuple              # FitFailure: an object that was seen but not fitted


def fit_scene(scene: SceneInput, proposals, *, specs: dict, setup, load_for, params: FitParams = FitParams(),
              identity_threshold=None) -> SceneFit:
    """Fit every proposal; one proposal's failure never stops the others."""
    rng = np.random.default_rng(params.seed)
    rays = pixel_rays(scene)
    candidates, failures = [], []
    for k, proposal in enumerate(proposals):
        try:
            cand = fit_instance(scene, proposal, specs=specs, setup=setup, load_for=load_for, params=params,
                                rng=rng, candidate_id=f"c{k}", rays=rays, identity_threshold=identity_threshold)
            reason = None if cand is not None else "no_face"
        except Exception as exc:  # noqa: BLE001 -- isolated per proposal by design (§10)
            cand, reason = None, f"exception:{type(exc).__name__}"
        if cand is not None:
            candidates.append(cand)
            continue
        pts, _ = proposal_points(scene, rays, proposal.mask, 0)
        if len(pts):
            c = np.median(pts[:, :2], axis=0)
            failures.append(FitFailure(proposal.proposal_id, reason, (float(c[0]), float(c[1])),
                                       float(np.max(np.hypot(*(pts[:, :2] - c).T)))))
        else:
            failures.append(FitFailure(proposal.proposal_id, reason, None))
    return SceneFit(tuple(candidates), tuple(failures))
```

- [ ] **Step 4: 통과 확인** — Run: Step 2 와 같음 · Expected: 작업 2–4 시험 42 개 통과. 실패하면 시험 기대값을 고치지 말고 원인을 찾는다(시제품 수치: 잡음 없는 리그 장면에서 가로·삽입축 오차는 bin 5 mm 의 절반 정도).

- [ ] **Step 5: 커밋**

```bash
git add src/forklift_core/perception/instance_fit.py tests/unit/perception/test_instance_fit.py
git commit -m "perception: instance fit candidates -- known-shape hypotheses per face (mirrored when asymmetric), faces compared by total evidence, equivalent hypotheses merged and close rivals ambiguous, pose constraints (face span, two members with a seen edge), support height and state, fork fit of the fitted shape, identity from the proposal source"
```

- [ ] **Step 6: 검토 A** — 작업 1–4 의 변경(`git diff origin/feat/p5-p0a...HEAD`)을 Codex `adversarial-review --wait --base origin/feat/p5-p0a --scope branch` 로 검토받는다. 초점: 정답 누출, 거울·좌우, 바깥면 검사의 잡음 의존성, 점수 항등식, 받침면 상태 판정, 자세 구속 판정. 새 P1/P2 가 없을 때까지 반복한다.

---

### Task 5: 층 번호와 대상 선택

**Files:**
- Create: `src/forklift_core/perception/target_selection.py`
- Test: `tests/unit/perception/test_target_selection.py`

**Interfaces:**
- Consumes: `PocketCandidate`(작업 1), 지시 사전(`check_instruction` 결과: `present`, `x_m`, `y_m`, `radius_m`, `layer`, `load`)
- Produces:
  - `SELECTION_STATUSES = ("selected", "ask_operator", "reobserve", "none", "not_pickable")`(평가기 `DECISIONS` 와 같은 값)
  - `Selection(status, candidate_id, reasons)`
  - `assign_layers(candidates, *, stack_xy_m=0.15, stack_yaw_rad=radians(10), height_tol_m=0.02) -> dict[str, (int, int | None)]` — 후보 ID → (상대 층, 절대 층 또는 None)
  - `select_target(candidates, instruction, *, reachable=None, unresolved=()) -> Selection` — `unresolved` 는 `SceneFit.failures`. 지시가 있고 실패한 물체의 가장 가까운 보이는 점(중앙값 거리 − `extent_m` 으로 아래에서 잡음)이 반경 + 최대 반대각선 0.85 m(S7 최대 1.2 × 1.2 m) 안이거나 위치를 모르면 `reobserve(unresolved_in_radius)` — 그 물체의 중심이 반경 안일 수 있기 때문이다(Codex 3회차: 0.4 m 여유는 맞닿은 EPAL 반례 0.81 m 를 놓쳤다)

규칙(설계 §8, §10):
- **층.** 팔레트 중심 xy 거리 ≤ 0.15 m 이고 yaw 차 ≤ 10° 인 후보들을 한 더미로 묶고 받침면 높이 순으로 상대 층 0, 1, … 을 매긴다. 맨 아래 후보가 `on_floor`(받침면 `visible` 이고 높이 ≈ 0) 이고 위층마다 받침면 상태가 `visible`·`coplanar`(아래 경계를 봤음 — `uncertain` 은 안 됨)이며 받침면 높이가 아래 후보의 받침면 + 그 형상 높이와 `height_tol_m` 안에서 맞으면 절대 층 = 상대 층, 아니면 그 더미 전체가 `None`(Codex 2026-10-16: 받침면 미확인 위층에 절대 층 1 이 붙던 경로).
- **지시 있음.** 반경 안 후보 = **팔레트 중심** xy 가 (x, y) 에서 `radius_m` 안인 모든 후보(상태 무관). 지시는 팔레트 중심을 준다 — 포켓 중점(진입면)과 비교하면 T11 은 0.33 m 떨어져 반경 0.3 m 밖이 된다(Codex 2026-10-16). 지시에 층이 있으면: 반경 안 후보 중 절대 층이 `None` 인 것이 있으면 `reobserve(layer_relative)`, 없으면 그 층의 후보만 남긴다. 남은 것 중 `counted` 가 둘 이상이면 `ask_operator(multiple_in_radius)`, 하나이고 다른 후보가 없으면 그것을 고르되 `pickable is False` 면 `not_pickable(<fit_reasons>)`, 그 밖이면 `selected`; 하나인데 다른 후보(애매·미식별·부적합)가 같이 있으면 `reobserve(uncertain_in_radius)`; 없으면 `none(no_candidate_in_radius)`.
- **지시 없음.** `counted` 이고 `pickable is not False` 이고 `reachable(c)` 인 후보 중 거리(`range_m`)가 가장 작은 것, 동률(1 mm)이면 팔레트 중심 y 가 작은 것. 그런 후보가 없지만 `counted`·도달 가능한 후보가 있으면(모두 끼움 불가) 가장 가까운 것을 `not_pickable(<사유>)` 로 낸다 — §10 은 끼움 실패를 `none` 과 구별한다. 아무것도 없으면 `none(no_candidate)`. 동률이어도 콘솔에 묻지 않는다. **2A 의 접근 비용은 카메라 거리, 도달 가능성은 "모두 가능"으로 대신한다** — §8-2 의 경로 계획기 비용·도달 가능성은 실행기가 이 함수를 부를 때(단계 4) 넘긴다.
- `pickable is None`(판정 보류) 후보도 선택될 수 있다 — 끼움 판정은 근거리 재관측 뒤(§9.1, 단계 4)이므로 그때까지 삽입 허용으로 바꾸지 않는다(Codex 2026-10-16: 선택과 삽입 허용을 구분).

- [ ] **Step 1: 실패하는 시험 작성** — `tests/unit/perception/test_target_selection.py`:

```python
# tests/unit/perception/test_target_selection.py
from dataclasses import replace

import pytest

from forklift_core.perception.pallet_model import ObservedPalletShape, face_layout
from forklift_core.perception.pocket_candidate import PocketCandidate, PocketEstimate
from forklift_core.perception.target_selection import assign_layers, select_target
from tools import recognition_scenes as rs

EPAL = rs.known_spec("epal6")
H = EPAL.overall_height_m


def _c(cid, x, y, *, z=0.0, state=None, geometry="fitted", identity="confirmed", pickable=True):
    """A candidate whose pallet centre is (x, y); its face is 0.3 m nearer the camera (EPAL 6)."""
    state = state or ("visible" if z == 0.0 else "coplanar")
    shape = ObservedPalletShape(EPAL, face_layout(EPAL, "x"), (("face", "known_shape"),))
    p = PocketEstimate((x - 0.3, y + 0.18625, z + 0.061), 0.2275, 0.078, "known_shape", "known_shape")
    return PocketCandidate(cid, "p" + cid, "oracle", 1, "epal6:x", "x", (), 0.0, (x - 0.3, y, z), (x, y, z), p,
                           replace(p, centre_m=(x - 0.3, y - 0.18625, z + 0.061)), z, state,
                           state == "visible" and z == 0.0, x - 0.3 - 0.75, 0.9, geometry, identity,
                           () if geometry == "fitted" else ("lateral_unconstrained",), pickable,
                           () if pickable is not False else ("fork_spacing_adjust",), shape)


def _instr(x, y, r=0.3, layer=None):
    return {"present": True, "x_m": x, "y_m": y, "radius_m": r, "layer": layer, "load": None}


NONE = {"present": False}


def test_a_stack_on_the_floor_gets_absolute_layers():
    cands = [_c("a", 3.0, 0.0), _c("b", 3.0, 0.01, z=H), _c("c", 3.01, 0.0, z=2 * H)]
    assert assign_layers(cands) == {"a": (0, 0), "b": (1, 1), "c": (2, 2)}


def test_a_stack_whose_bottom_is_not_on_the_floor_has_only_relative_layers():
    cands = [_c("b", 3.0, 0.0, z=H), _c("c", 3.0, 0.0, z=2 * H)]
    assert assign_layers(cands) == {"b": (0, None), "c": (1, None)}


def test_a_gap_in_the_stack_drops_the_absolute_numbers():
    cands = [_c("a", 3.0, 0.0), _c("c", 3.0, 0.0, z=2 * H)]   # the middle pallet was not seen
    assert assign_layers(cands)["c"] == (1, None)


def test_an_upper_layer_with_an_unseen_support_drops_the_absolute_numbers():
    """Codex 2026-10-16: a floor candidate and one at height H with an uncertain support got layer 1."""
    cands = [_c("a", 3.0, 0.0), _c("b", 3.0, 0.0, z=H, state="uncertain")]
    assert assign_layers(cands)["b"] == (1, None)


def test_one_identified_fit_alone_in_the_radius_is_selected():
    s = select_target([_c("a", 3.0, 0.0), _c("b", 3.0, 1.5)], _instr(3.0, 0.0))
    assert (s.status, s.candidate_id) == ("selected", "a")


def test_the_instruction_names_the_pallet_centre_not_the_face():
    """Codex 2026-10-16: instructions carry the pallet centre (tools/recognition_scenes.py _instruction);
    the face of a T11 is 0.33 m nearer, outside a 0.3 m radius."""
    s = select_target([_c("a", 3.0, 0.0)], _instr(3.0, 0.0, r=0.1))
    assert (s.status, s.candidate_id) == ("selected", "a")
    assert select_target([_c("a", 3.0, 0.0)], _instr(2.7, 0.0, r=0.1)).status == "none"


def test_two_identified_fits_in_the_radius_ask_the_operator():
    s = select_target([_c("a", 3.0, 0.0), _c("b", 3.0, 0.5)], _instr(3.0, 0.25, r=0.5))
    assert s.status == "ask_operator" and s.candidate_id is None


def test_an_unconfirmed_neighbour_in_the_radius_asks_for_another_look():
    s = select_target([_c("a", 3.0, 0.0), _c("b", 3.0, 0.4, identity="unconfirmed")], _instr(3.0, 0.2, r=0.5))
    assert s.status == "reobserve" and "uncertain_in_radius" in s.reasons


def test_nothing_in_the_radius_is_none():
    assert select_target([_c("a", 3.0, 1.5)], _instr(3.0, 0.0)).status == "none"


def test_an_unpickable_target_is_reported_with_its_reason():
    s = select_target([_c("a", 3.0, 0.0, pickable=False)], _instr(3.0, 0.0))
    assert (s.status, s.candidate_id, s.reasons) == ("not_pickable", "a", ("fork_spacing_adjust",))


def test_the_instructed_layer_is_picked_from_an_absolute_stack():
    s = select_target([_c("a", 3.0, 0.0), _c("b", 3.0, 0.0, z=H)], _instr(3.0, 0.0, layer=1))
    assert (s.status, s.candidate_id) == ("selected", "b")


def test_a_layer_instruction_on_a_relative_stack_asks_for_another_look():
    s = select_target([_c("b", 3.0, 0.0, z=H), _c("c", 3.0, 0.0, z=2 * H)], _instr(3.0, 0.0, layer=1))
    assert s.status == "reobserve" and "layer_relative" in s.reasons


def test_without_an_instruction_the_nearest_pickable_identified_fit_is_chosen():
    cands = [_c("far", 4.0, 0.0), _c("near", 2.5, 0.8), _c("nope", 2.0, 0.0, pickable=False),
             _c("amb", 1.9, 0.0, geometry="ambiguous")]
    assert select_target(cands, NONE).candidate_id == "near"


def test_without_an_instruction_an_unpickable_pallet_is_reported_not_hidden():
    """§10: a fork-fit failure is not_pickable, not none (Codex 2026-10-16)."""
    s = select_target([_c("a", 3.0, 0.0, pickable=False)], NONE)
    assert (s.status, s.candidate_id) == ("not_pickable", "a")


def test_a_pending_fork_fit_may_be_selected_but_stays_pending():
    s = select_target([_c("a", 3.0, 0.0, pickable=None)], NONE)
    assert (s.status, s.candidate_id) == ("selected", "a")


def test_ties_without_an_instruction_are_broken_by_lateral_position_not_by_asking():
    s = select_target([_c("l", 3.0, 0.5), _c("r", 3.0, -0.5)], NONE)
    assert (s.status, s.candidate_id) == ("selected", "r")


def test_unreachable_candidates_are_skipped():
    s = select_target([_c("a", 2.5, 0.0), _c("b", 3.0, 0.0)], NONE, reachable=lambda c: c.candidate_id != "a")
    assert s.candidate_id == "b"


def test_a_pallet_seen_but_not_fitted_inside_the_radius_asks_for_another_look():
    """Codex 2026-10-16: touching pallets at (3, -0.4) and (3, 0.4), radius 0.401; the failed one is
    seen around (2.7, 0.75), 0.808 m away -- outside a 0.4 m margin, inside the 0.85 m half-diagonal."""
    from forklift_core.perception.instance_fit import FitFailure
    cands = [_c("a", 3.0, -0.4)]
    instr = _instr(3.0, 0.0, r=0.401)
    assert select_target(cands, instr).candidate_id == "a"
    seen = FitFailure("p1", "exception:FloatingPointError", (2.7, 0.75), 0.0)
    s = select_target(cands, instr, unresolved=(seen,))
    assert s.status == "reobserve" and "unresolved_in_radius" in s.reasons
    assert select_target(cands, instr, unresolved=(FitFailure("p1", "no_face", None),)).status == "reobserve"


def test_a_failed_fit_is_judged_by_its_nearest_visible_point():
    """The median of the visible points is 1.5 m out, but the object spreads 0.3 m around it: its
    nearest point is 1.2 m away, within 0.401 + 0.85, so its centre may be in the radius."""
    from forklift_core.perception.instance_fit import FitFailure
    cands = [_c("a", 3.0, -0.4)]
    instr = _instr(3.0, 0.0, r=0.401)
    spread = FitFailure("p1", "no_face", (3.0, 1.5), 0.3)
    assert select_target(cands, instr, unresolved=(spread,)).status == "reobserve"
    point = FitFailure("p1", "no_face", (3.0, 1.5), 0.0)
    assert select_target(cands, instr, unresolved=(point,)).candidate_id == "a"


def test_no_candidates_is_none():
    assert select_target([], NONE).status == "none"
```

- [ ] **Step 2: 실패 확인** — Run: `PYTHONPATH=src:. python -m pytest tests/unit/perception/test_target_selection.py -q -p no:cacheprovider` · Expected: `ModuleNotFoundError: ... target_selection`

- [ ] **Step 3: 구현** — `src/forklift_core/perception/target_selection.py`:

```python
# src/forklift_core/perception/target_selection.py
"""Layers and target selection over PocketCandidates (design §7.3, §8, §10).

Only a fitted and confirmed candidate is selected automatically. With an
instruction (a map position of the pallet centre, a radius and optionally an
absolute layer), a unique counted candidate is not enough: any other candidate
in the radius sends the robot to look again, and two counted ones go to the
operator. Without one, any pallet will do, so ties are broken deterministically.
Selecting a candidate is not permission to insert: pickable None stays None
until the near-range re-observation (§9.1).

Stage 2A has no path planner, so the approach cost without an instruction is the
camera range of the candidate and reachability defaults to "all reachable"; the
planner's cost and reachability replace both when the executor calls this (§8-2).
"""

import math
from dataclasses import dataclass

import numpy as np

SELECTION_STATUSES = ("selected", "ask_operator", "reobserve", "none", "not_pickable")
ABSOLUTE_SUPPORT = ("visible", "coplanar")  # an upper layer's support edge must be seen, not guessed
# Any visible point of a pallet lies within its half-diagonal of the centre; the largest pallet
# the scenes allow is 1.2 x 1.2 m (S7, design §5.1), so 0.85 m covers every shape.
MAX_HALF_DIAGONAL_M = 0.85


@dataclass(frozen=True)
class Selection:
    status: str
    candidate_id: str | None
    reasons: tuple

    def __post_init__(self) -> None:
        if self.status not in SELECTION_STATUSES:
            raise ValueError(f"status must be one of {SELECTION_STATUSES}")
        object.__setattr__(self, "reasons", tuple(self.reasons))


def _height(c) -> float:
    spec = c.shape.spec
    return spec.overall_height_m if spec is not None else math.nan


def assign_layers(candidates, *, stack_xy_m=0.15, stack_yaw_rad=math.radians(10), height_tol_m=0.02) -> dict:
    """Candidate id -> (relative layer, absolute layer or None).

    Candidates whose pallet centres are within stack_xy_m and whose yaws agree form a
    stack, numbered by support height. A stack is absolute only if its bottom stands on a
    seen floor and every layer above rests on the one below (support height = lower support
    + lower pallet height) with its lower edge seen (visible or coplanar, never uncertain).
    """
    stacks = []
    for c in sorted(candidates, key=lambda c: c.support_z_m):
        for stack in stacks:
            ref = stack[0]
            dyaw = abs((c.insertion_yaw_rad - ref.insertion_yaw_rad + math.pi) % (2 * math.pi) - math.pi)
            if math.dist(c.pallet_centre_m[:2], ref.pallet_centre_m[:2]) <= stack_xy_m and dyaw <= stack_yaw_rad:
                stack.append(c)
                break
        else:
            stacks.append([c])
    out = {}
    for stack in stacks:
        absolute = stack[0].on_floor and all(
            upper.support_state in ABSOLUTE_SUPPORT
            and abs(upper.support_z_m - (lower.support_z_m + _height(lower))) <= height_tol_m
            for lower, upper in zip(stack, stack[1:]))
        for k, c in enumerate(stack):
            out[c.candidate_id] = (k, k if absolute else None)
    return out


def _decide(c) -> Selection:
    if c.pickable is False:
        return Selection("not_pickable", c.candidate_id, c.fit_reasons)
    return Selection("selected", c.candidate_id, ())


def _nearest(cands):
    return min(cands, key=lambda c: (round(c.range_m, 3), c.pallet_centre_m[1]))


def select_target(candidates, instruction, *, reachable=None, unresolved=()) -> Selection:
    """unresolved: objects seen but not fitted (instance_fit.FitFailure). If such an object's
    centre could lie inside the instruction's radius -- its nearest visible point within radius +
    the largest half-diagonal, or its position unknown -- the instructed target may not be unique,
    so the robot looks again (Codex 2026-10-16)."""
    reachable = (lambda c: True) if reachable is None else reachable
    if instruction.get("present"):
        target = np.array([instruction["x_m"], instruction["y_m"]])
        reach = instruction["radius_m"] + MAX_HALF_DIAGONAL_M
        if any(f.centre_xy is None or np.linalg.norm(np.asarray(f.centre_xy) - target) - f.extent_m <= reach
               for f in unresolved):
            return Selection("reobserve", None, ("unresolved_in_radius",))
        near = [c for c in candidates
                if np.linalg.norm(np.asarray(c.pallet_centre_m[:2]) - target) <= instruction["radius_m"]]
        layer = instruction.get("layer")
        if layer is not None and near:
            layers = assign_layers(candidates)
            if any(layers[c.candidate_id][1] is None for c in near):
                return Selection("reobserve", None, ("layer_relative",))
            near = [c for c in near if layers[c.candidate_id][1] == layer]
        counted = [c for c in near if c.counted]
        if len(counted) >= 2:
            return Selection("ask_operator", None, ("multiple_in_radius",))
        if len(counted) == 1:
            if len(near) == 1:
                return _decide(counted[0])
            return Selection("reobserve", None, ("uncertain_in_radius",))
        return Selection("none", None, ("no_candidate_in_radius",))
    usable = [c for c in candidates if c.counted and reachable(c)]
    options = [c for c in usable if c.pickable is not False]
    if options:
        return _decide(_nearest(options))
    if usable:
        # Every identified pallet in reach fails the fork fit: report it, not "nothing there" (§10).
        return _decide(_nearest(usable))
    return Selection("none", None, ("no_candidate",))
```

주: 반경 안에 `counted` 가 하나도 없지만 다른 후보가 있으면 위 규칙은 `none` 이다 — 설계 §8-1 의 "없으면 대상 없음 — 재관측"을 `none` 으로 내고, 애매·미식별 후보만 있는 경우를 `reobserve` 로 바꿀지는 작업 6 의 dev 결과(대상 선택 오류 대 재관측 요청률)를 보고 정하지 않는다 — 합격선을 dev 로 고치지 않는 원칙과 같이, 규칙은 설계대로 둔다. 다만 이 경우의 지표를 기록한다.

- [ ] **Step 4: 통과 확인** — Run: Step 2 와 같음 · Expected: 20 개 통과.

- [ ] **Step 5: 커밋**

```bash
git add src/forklift_core/perception/target_selection.py tests/unit/perception/test_target_selection.py
git commit -m "perception: target selection -- stacks get relative layers and absolute ones only from a floor-supported, gap-free stack; an instruction picks a lone fitted and confirmed candidate, asks the operator for two, looks again when an uncertain one shares the radius or the layer is relative; without one the nearest pickable candidate wins"
```

---

### Task 6: 평가기 `oracle-fit`, 진단 파일, ws1 dev 실행과 기록

**Files:**
- Modify: `tools/evaluate_recognition.py`, `src/forklift_core/perception/evaluation_multi.py`(판정 동작은 그대로, 후보별 위험 판정을 함수로 꺼낸다)
- Create: `deploy/slurm/recognition_stage2a_evaluate.sbatch`, `docs/validation/<실행일>-recognition-stage2a-oracle-fit.md`
- Test: `tests/integration/test_recognition_cli.py`(기존 파일에 추가 — 같은 생성기 도우미 `_run` 을 쓴다), `tests/unit/perception/test_evaluation_multi.py`(추가)

**Interfaces:**
- Consumes: 작업 4 의 `fit_scene -> SceneFit`, 작업 5 의 `select_target`·`assign_layers`; `CandidateView`, `TargetDecision`, `judge_scene`, `summarize`, `_match`(평가기, 변경 없음); `rs.known_spec`; `load_site_defaults`
- Produces:
  - `--detector oracle-fit` — `_oracle_fit(sample) -> (views, decision, fitted: SceneFit)`; 기존 두 검출기는 `(views, decision)` 그대로이고 `main` 이 세 번째 값이 없으면 진단을 비운다
  - `evaluation_multi.candidate_danger(ci, cand, pairs, costs, instances) -> bool | None` — `judge_scene` 의 후보별 위험 분기를 그대로 꺼낸 함수(집기 허용이 아니면 None; 대응된 면, 대응은 안 됐지만 대응 가능한 면 중 최소 비용 면, 대응 가능한 면이 없으면 위험). `judge_scene` 이 이 함수를 쓰고 판정 결과는 바뀌지 않는다(기존 시험 그대로 통과). Codex 2026-10-16: 정답과 같은 후보 둘이면 하나는 미대응이지만 `extra=True`, `dangerous=False` — 진단도 이 분기를 따라야 지표와 맞는다
  - `--diagnostics` — `candidates.jsonl`. 줄마다 종류 `row` 가 있다(`candidate` / `decision` / `failure` — Codex 2026-10-16: 선택 줄에도 `candidate_id` 가 있어 키로는 못 가른다). 후보 줄: `scene_id`, `case`, `shape_key`, `candidate_id`, `proposal_id`, `hypothesis`, `geometry`, `identity`, `reasons`, `pickable`, `fit_reasons`, `insertion_yaw_rad`, `left_m`, `right_m`, `range_m`(예측), `support_z_m`, `support_state`, `layer_abs`, `score`, `counted`, `matched_instance`(대응 없으면 null), `errors`(가로·삽입축·yaw·높이 또는 null), `truth_range_m`(대응된 정답 면의 두 포켓 중점과 카메라의 수평 거리, 대응 없으면 null), `dangerous`(`candidate_danger`). 장면마다 선택 줄 `{"row": "decision", "scene_id", "decision", "candidate_id", "decision_reasons"}`, 제안 실패마다 `{"row": "failure", "scene_id", "proposal_id", "failure", "centre_xy", "extent_m"}`
  - `metrics.json` 에 `detector: "oracle-fit"`, `fit_params`(FitParams 전체), `proposal_failures`(사유별 개수)

```python
SPECS = {k: rs.known_spec(k) for k in ("epal6", "t11_06")}
SITE = load_site_defaults(ROOT / "config/site_load_defaults.yaml", SPECS)
SITE_BY_NAME = {SPECS[k].name: v for k, v in SITE.items()}
MIN_PROPOSAL_PX = 50   # smaller fragments cannot reach the 60 eroded points a face needs


def _load_for(instruction):
    given = instruction.get("load") if instruction.get("present") else None
    declared = None if given is None else LoadDeclaration(**given)
    return lambda spec: declared if declared is not None else SITE_BY_NAME.get(spec.name)


def _oracle_fit(sample):
    """Truth instance masks as proposals (source oracle), then the stage 2 fit and selection.

    Reads the instance id image, the instruction and the scene's fork spacing (a robot
    setting) -- never truth.json, the case or the shape key. Distractor and background
    pixels (ids < 0) are never proposed, so negative scenes cannot produce a candidate here:
    their false-positive rate under oracle-fit is zero by construction, not a result.
    """
    ids = sample.instance_ids
    proposals = [InstanceProposal(f"i{k}", ids == k, 1.0, "oracle")
                 for k in sorted(int(v) for v in np.unique(ids) if v >= 0) if (ids == k).sum() >= MIN_PROPOSAL_PX]
    fitted = fit_scene(sample.input.scene, proposals, specs=SPECS, setup=ForkSetup(sample.meta["fork_spacing_m"]),
                       load_for=_load_for(sample.input.instruction))
    layers = assign_layers(fitted.candidates)
    # A pallet seen but not fitted inside the instruction's radius keeps the target ambiguous (Codex 2026-10-16)
    sel = select_target(fitted.candidates, sample.input.instruction, unresolved=fitted.failures)
    views = [CandidateView(c.candidate_id, c.insertion_yaw_rad, c.left.centre_m, c.right.centre_m,
                           c.geometry == "fitted", c.identity == "confirmed", c.pickable, c.fit_reasons,
                           layers[c.candidate_id][1]) for c in fitted.candidates]
    return views, TargetDecision(sel.status, sel.candidate_id, sel.reasons), fitted
```

진단의 `matched_instance`·`errors` 는 `judge_scene` 과 같은 `_match(counted, instances)` 결과에서, `dangerous` 는 `candidate_danger` 로 채운다. 판정 규칙은 바꾸지 않는다.

- [ ] **Step 1: 실패하는 시험 작성** — `tests/integration/test_recognition_cli.py` 끝에:

```python
def _fit_fields(path):
    """Everything the recogniser decided -- candidates with their pockets and yaw, the selection, the
    failures -- and nothing the judge computed from truth (Codex 2026-10-16: compare pockets and the
    decision too, or a fit that copied truth pockets or a selection that read the target would pass)."""
    judged = {"case", "shape_key", "matched_instance", "errors", "truth_range_m", "dangerous"}
    rows = [json.loads(line) for line in (path / "candidates.jsonl").read_text().splitlines()]
    return [{k: v for k, v in r.items() if k not in judged} for r in rows]


def test_oracle_fit_judges_a_rig_dataset_and_writes_diagnostics(tmp_path):
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1,S2", "--limit", "2", "--noise", "none")
    out = tmp_path / "fit"
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "oracle-fit",
         "--output", str(out), "--diagnostics")
    metrics = json.loads((out / "metrics.json").read_text())
    assert metrics["detector"] == "oracle-fit" and metrics["fit_params"]["bin_m"] == 0.005
    rows = [json.loads(line) for line in (out / "candidates.jsonl").read_text().splitlines()]
    cands = [r for r in rows if r["row"] == "candidate"]
    assert cands and all(r["geometry"] in ("fitted", "ambiguous", "unfit") for r in cands)
    matched = [r for r in cands if r["errors"] is not None]
    assert matched and all(r["truth_range_m"] > 0 for r in matched)
    dev = sum(json.loads(p.read_text())["split"] == "dev" for p in data.glob("*/scene.json"))
    assert sum(r["row"] == "decision" for r in rows) == dev   # one decision per judged scene
    assert max(r["errors"][0] for r in matched) <= 0.02   # clean rig scenes: lateral within 20 mm


def test_oracle_fit_never_reads_truth_or_scene_labels(tmp_path):
    """Corrupt truth.json and the scene's case/shape labels: the fit and selection must not move
    (Codex 2026-10-16: a check on keyword names would pass a fit narrowed by shape_key)."""
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1,S2", "--limit", "2", "--noise", "none")
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "oracle-fit",
         "--output", str(tmp_path / "a"), "--diagnostics")
    for scene_json in data.glob("*/scene.json"):
        meta = json.loads(scene_json.read_text())
        meta["shape_key"] = "t11_06" if meta["shape_key"] == "epal6" else "epal6"
        meta["case"] = "S1" if meta["case"] == "S2" else "S2"
        scene_json.write_text(json.dumps(meta))
        truth_json = scene_json.parent / "truth.json"
        truth = json.loads(truth_json.read_text())
        itruth = truth["instruction_truth"]
        if itruth["kind"] != "none" and len(truth["instances"]) > 1:
            # Point the instruction truth at another valid instance and flip its kind: a selection
            # that consults the target or the ambiguity label moves (Codex 2026-10-16).
            itruth["target"] = (itruth["target"] + 1) % len(truth["instances"])
            itruth["kind"] = "ambiguous" if itruth["kind"] == "unambiguous" else "unambiguous"
        for inst in truth["instances"]:
            inst["spec"]["name"] = "x"
            for face in inst["faces"]:
                face["left_m"], face["right_m"] = [9.0, 9.0, 0.0], [9.0, 8.0, 0.0]
        truth_json.write_text(json.dumps(truth))
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "oracle-fit",
         "--output", str(tmp_path / "b"), "--diagnostics")
    a, b = _fit_fields(tmp_path / "a"), _fit_fields(tmp_path / "b")
    assert a == b and any(r["row"] == "candidate" for r in a) and any(r["row"] == "decision" for r in a)
```

`tests/unit/perception/test_evaluation_multi.py` 에(import 에 `_match`, `candidate_danger` 추가; `META`·`NONE_IN`·`_truth`·`_inst`·`_cand` 는 이 파일의 기존 도우미):

```python
def test_a_duplicate_of_a_matched_candidate_is_extra_but_not_dangerous():
    """Codex 2026-10-16: an unmatched pick-permitted candidate that can match a face is judged on that
    face, not flagged dangerous; candidate_danger gives exactly the answer judge_scene uses."""
    truth = _truth([_inst(0, 0.0)])
    cands = [_cand("a", 0.0), _cand("b", 0.0)]
    v = judge_scene(META, truth, NONE_IN, cands, TargetDecision("selected", "a", ()))
    assert v.extra and not v.dangerous
    pairs, costs = _match(cands, truth["instances"])
    assert [candidate_danger(i, c, pairs, costs, truth["instances"]) for i, c in enumerate(cands)] == [False, False]


def test_candidate_danger_agrees_with_the_unmatched_far_duplicate():
    truth = _truth([_inst(0, 0.0)])
    cands = [_cand("a", 0.0), _cand("b", 0.0, dx=0.25)]
    pairs, costs = _match(cands, truth["instances"])
    assert [candidate_danger(i, c, pairs, costs, truth["instances"]) for i, c in enumerate(cands)] == [False, True]
    assert candidate_danger(0, _cand("a", 0.0, pickable=None), pairs, costs, truth["instances"]) is None
```

(두 번째 실행에서 판정 결과는 정답을 망쳤으므로 달라진다 — 비교는 맞춤·선택 필드만 한다. 기존 기준선 시험이 같은 방식으로 `shape_key` 를 망친다.)

- [ ] **Step 2: 실패 확인** — Run: `PYTHONPATH=src:. python -m pytest tests/integration/test_recognition_cli.py -q -p no:cacheprovider -k oracle_fit` · Expected: 두 시험 모두 `CalledProcessError`(argparse 가 `oracle-fit` 을 `invalid choice` 로 거부).

- [ ] **Step 3: 구현** — `tools/evaluate_recognition.py`: 위 코드와 import(`numpy`, `ForkSetup`, `LoadDeclaration`, `load_site_defaults`, `InstanceProposal`, `FitParams`, `fit_scene`, `assign_layers`, `select_target`, `tools.recognition_scenes`, 평가기의 `_match`·`_danger`); `--detector` 에 `oracle-fit`, `--diagnostics` 플래그; `run(sample)` 의 결과가 셋이면 세 번째를 진단에 쓴다; `oracle-fit` 일 때 `metrics["fit_params"] = asdict(FitParams())`, `metrics["proposal_failures"]`.

- [ ] **Step 4: 통과 확인** — Run: Step 2 와 같음 · Expected: 통과. 이어서 ws1 전체 시험: `git add` 로 새 파일을 올린 뒤 `FORKLIFT_REPO=$PWD ~/bin/forklift-ws1-pytest tests --ignore=tests/simulation -q -p no:cacheprovider` · Expected: 알려진 ws1 전용 실패 10 개 외 통과.

- [ ] **Step 5: 커밋**

```bash
git add tools/evaluate_recognition.py tests/integration/test_recognition_cli.py src/forklift_core/perception/evaluation_multi.py tests/unit/perception/test_evaluation_multi.py
git commit -m "tools: evaluate_recognition oracle-fit detector -- truth instance masks as oracle proposals into the stage 2 fit and target selection; per-candidate diagnostics with the matched truth range and the danger verdict, proposal failures, fit parameters in metrics"
```

- [ ] **Step 6: ws1 dev 실행** — 데이터(1B 의 `isaac_clean`·`isaac_low`·`isaac_mid`·`isaac_high`)가 ws1 `/home/projects/forklift/artifacts/20261015T150838Z_recognition_isaac/` 에 있으므로 ws1 Slurm 에서 돌린다(전역 지침의 "ws1 이 확실히 나은 경우"). `deploy/slurm/recognition_stage2a_evaluate.sbatch`:

```bash
#!/bin/bash
#SBATCH --job-name=recog-2a-eval
#SBATCH --partition=compute
#SBATCH --cpus-per-task=8
#SBATCH --mem=8G
#SBATCH --time=00:30:00
#SBATCH --output=%x-%j.log
# Stage 2A dev evaluation: oracle-fit and baseline on the stage 1B Isaac datasets.
# Usage: sbatch --chdir=<run dir> deploy/slurm/recognition_stage2a_evaluate.sbatch <source dir> <isaac run dir> <manifest>
# (1B evaluation job 2876: 8 processes, MaxRSS 1.8 GB, 6 min -> 8G and 30 min with headroom.)
set -uo pipefail
src=$1; run=$2; manifest=$3
PY=/home/projects/forklift/.venv/bin/python
export PYTHONPATH="$src/src:$src" PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
pids=(); names=()
for ds in clean low mid high; do
  for det in oracle-fit baseline; do
    $PY "$src/tools/evaluate_recognition.py" --dataset "$run/isaac_$ds" --split dev --detector "$det" \
      --output "out/${ds}_${det}" --diagnostics --source-manifest "$manifest" > "out_${ds}_${det}.log" 2>&1 &
    pids+=($!); names+=("${ds}_${det}")
  done
done
rc=0
for i in "${!pids[@]}"; do
  if ! wait "${pids[$i]}"; then echo "FAILED ${names[$i]}"; rc=1; fi
  [ -s "out/${names[$i]}/metrics.json" ] || { echo "MISSING out/${names[$i]}/metrics.json"; rc=1; }
done
echo "ALL_DONE rc=$rc"
exit $rc
```

(Codex 2026-10-16: 인자 없는 `wait` 는 자식 실패를 삼킨다 — PID 마다 종료 상태와 `metrics.json` 을 확인한다.) 소스는 `git archive` 스냅숏을 run 폴더의 `source/` 에 풀고 1B 와 같은 방식으로 `source_manifest.json` 을 만든다. 제출 전에 `scontrol show node ws1 | grep -E "CfgTRES|AllocTRES"` 로 자원을 확인하고, 대기는 `run_in_background` 로 건다.

- [ ] **Step 7: 기록** — `docs/validation/<실행일>-recognition-stage2a-oracle-fit.md`: 실행(잡 ID, 소스 리비전, 데이터셋 해시, `FitParams`), 사례·형상·잡음 수준별 놓침 / 위험 / 추가 후보 / 대상 선택 오류 / 재관측·확인 요청을 `oracle-fit` 과 `baseline` 나란히, 형상별 자세 p95(가로·삽입축·yaw·높이), **정답 진입면 거리(`truth_range_m`) 구간별** 자세 p95 와 위험 수(대응되지 않은 위험 후보 따로), 기하 상태·사유·받침면 상태 분포, 제안 실패, 처리 시간. 합격선과 견주되 **dev 결과이며 판정이 아님**을 적는다. 다음을 분명히 적는다: S7 은 2A 범위 밖(알려진 형상 가설만); N·S6 방해물은 `oracle-fit` 에서 제안되지 않으므로 음성 오검출 0 은 구성상 값이고 배경 억제 성능이 아니다(Codex); 원거리 yaw 위험과 Codex 의 지적(통합 p95·설계 개정 필요·공식 잡음 대기).

- [ ] **Step 8: 커밋** — `git add deploy/slurm/recognition_stage2a_evaluate.sbatch docs/validation/<실행일>-recognition-stage2a-oracle-fit.md` 후 `docs: stage 2A oracle-fit dev record -- ...`.

- [ ] **Step 9: 검토 B** — 작업 5–6 의 변경과 기록을 Codex `adversarial-review` 로 검토받아 새 P1/P2 가 없을 때까지 반복한다. 초점: 선택 규칙과 평가기 판정의 일치, 정답 누출, 진단의 표본 선택(정답 거리), 기록의 수치가 `metrics.json`·`candidates.jsonl` 과 맞는가.

---

## 자체 검토

### ① 원자료 재계산 (Claude, 2026-10-16)

`face_layout`·`spec_boxes`·`scene_rig` 를 다시 돌려 계획의 수치를 확인했다. 틀린 것 넷을 고쳤다.

| 항목 | 처음 | 원자료 | 처리 |
|---|---|---|---|
| EPAL 6 진입면과 중심 거리 | 0.4 m | 깊이 0.6 m 의 절반 0.3 m | 시험 두 곳 수정 |
| EPAL 6 격자 맨 윗줄 | 꽉 참 | 상판 판 사이 틈이 보인다. 꽉 찬 줄은 스트링거(0.100–0.122 m) | 시험 수정 |
| 절반 가림 시험 | 마스크만 자름 | 원본 광선은 숨긴 부재를 계속 본다 — 마스크 절단은 가림이 아니다 | 가림막 상자(실루엣이 진입면에서 y = 0.272 m 까지)로 바꿈. 시제품으로 `ambiguous` 확인 |
| T11 y 면 끼움 | `pickable is True` | y 진입은 판 경계가 포크 방향이 아니라 `near_tracking_unavailable` (`fork_fit._roof_trackable`) | 정답 면의 끼움과 같음을 확인하도록 수정 |

확인만 한 값: EPAL 6 개구 (±0.0725, ±0.3), 중심 ±0.18625 m, 띠 0.022–0.100 m; T11 x·y 면 개구 (±0.04, ±0.25), 중심 ±0.145 m, x 띠 0.015–0.060 m, y 띠 0.015–0.075 m; 리그 초점 465.741 px(잡음 설정의 `target_fx` 465.74 와 같음).

시험 배치를 시제품으로 돌리다 하나를 더 찾았다. 비스듬한 T11 에서 앞면 다음으로 남은 점(상판 윗면 등)이 강건 척도 48 mm 의 '평면'으로 잡혔고, 바깥면 검사 여유가 3 × 척도라 통과했다. 면 척도 상한 `max(10 mm, ray_margin_k · 거리² / 2)` 를 더했다(3 m 에서 67 mm — 잠정 중간 잡음의 상관 성분 σ = 0.0049 m⁻¹ × 9 m² = 44 mm 보다 크다). 상한을 넣고 dev 를 다시 돌린 결과 `fitted`·오차는 그대로였다.

### ③ 독립 검토 (Gemini 3.8 Flash High, 계획과 핵심 코드만 — ①·② 의 결론과 원거리 상의 내용은 빼고 보냄)

"적히지 않은 전제"를 물었다. 지적 7 개와 처리:

| # | 지적 | 처리 |
|---|---|---|
| 1 | 진입면이 base z 에 수직이고 차체·바닥이 수평이라는 전제 — 차체 롤 1.5° 면 0.8 m 폭 양 끝 높이가 21 mm 어긋나 틀과 안 맞는다 | 맞다. Isaac 장면은 수평이라 2A 에서 드러나지 않는다. **2B 로 넘김**: 롤·피치가 있는 리그 장면 시험과 면 기울기를 함께 맞추는 경로. 실차 IMU 자세를 base 에 반영하는지는 실행기 쪽 확인 대상 |
| 2 | 원거리 오차를 그대로 쓰면 접근 궤적이 틀어진다 | 선택과 삽입 허용을 구분(작업 5 주), 거리별 기록(작업 6). 접근 경로가 원거리 오차를 견디는지는 임무 단계 증거가 필요하다(Codex 와 같은 지적) |
| 3 | Jetson 지연 | 설계 §6 의 시간 계약·지연 예산은 단계 3·2B. 2A 는 ws1 처리 시간을 기록만 |
| 4 | D435i 경계 번짐(전경이 배경 쪽으로 10–20 mm 부풂) | 잡음 모형은 경계 혼합 화소만 있고 번짐이 없다. 개구 양쪽이 같이 줄면 포켓 중심은 그대로지만 경계 판정 창(4 px)이 번짐보다 작으면 `lateral_unconstrained` 가 는다. **팀 D435i 촬영 뒤** 번짐 폭을 재서 창을 다시 정한다(기록에 남김) |
| 5 | 상판 처짐·늘어진 랩이 개구를 가림 | 랩이 진입면 앞이면 '앞' 광선이라 감점이 없고, 면 위에 걸리면 감점된다. S6 의 랩 감은 짐으로 dev 에서 본다. 처짐은 모형에 없다 — 기록 |
| 6 | 바깥면 검사가 튀어나온 짐·바닥 턱을 내부 평면으로 오인 | 마스크는 팔레트만이고 짐은 다른 클래스다. 마스크가 바닥으로 넘치는 경우는 아래 7 의 시험이 덮는다 |
| 7 | 학습 마스크가 바닥으로 3–5 px 넘침 | **시험 추가**(`test_an_over_segmented_mask_is_still_fitted`). dev 에서 정답 마스크를 4 px 부풀려도 `fitted` 수와 오차가 같았다(잡음 없음 S1 EPAL 41/41, 가로 p95 6 mm) |

### ② Codex 적대적 검토

**1회차 (P1 6 · P2 5, 미수렴).** Codex 는 계획 코드블록을 메모리에 올려 직접 돌렸다(기하 시험 23 개 중 21 통과). 지적과 처리:

| # | 지적 | 처리 |
|---|---|---|
| P1-1 | 지시는 팔레트 **중심**을 주는데(`recognition_scenes._instruction`) 선택은 포켓 중점(진입면)과 비교 — T11 은 0.33 m 떨어져 반경 0.3 m 밖, 깨끗한 후보도 `none` | `PocketCandidate.pallet_centre_m`(진입면 중심 + 삽입 방향 × 깊이/2)을 더하고 선택·층 묶음이 그것을 쓴다. 시험 `test_the_instruction_names_the_pallet_centre_not_the_face` |
| P1-2 | 하단이 가려지면 받침면 탐색이 10.8 mm 로 올라가도 `fitted`·`pickable=True`; 받침면 상태가 수평면을 확인하지 않아 카메라보다 높은 받침면(0.288 m)도 `visible`; 층 부여가 위층의 받침면 상태를 안 봄 | `support_state` 를 다시 짰다 — 맨 아래 점이 진입면 위·받침면 높이에 있는 열만 쓰고(차선 열 제외), 그 아래 광선이 면보다 앞에서 받침면 높이의 면을 카메라 위에서 볼 때만 `visible`, 면이 이어지면 `coplanar`, 뒤로 빠지면 `physical_edge`, 가려지면 `uncertain`. `uncertain` 은 높이를 읽는 사유를 빼고 `pickable` 을 None(또는 남은 사유로 False)로, 사유 `support_uncertain`. `assign_layers` 는 위층 받침면이 `visible`·`coplanar` 일 때만 절대 층. 시험 세 개(하단 가림, 3 단 적층 `visible/coplanar/coplanar`, 받침면 미확인 위층) |
| P1-3 | T11 x/y 가설을 포켓 거리로 동치 처리 — 끼움(근접 추적)이 다른 가설을 묶어 버림(점수 0.940/0.943 재현) | 동치는 같은 축·같은 끼움 사유일 때만. Codex 재현 입력(20–55 mm 깊이만)으로 `ambiguous(competing)` 확인하는 시험 |
| P1-4 | 시제품의 면 척도 상한이 계획 코드에 빠짐(48.4 mm 평면이 바깥면 검사를 무력화) | 계획 코드를 시험을 통과한 파일에서 그대로 옮기는 방식으로 바꿨다(아래). 면 추출을 다시 고치며 원인 셋을 더 찾았다 — ① RANSAC 최다 내점 선이 면보다 1.5 cm 안쪽(면 + 차선 벽)에 놓임 → 가장 조밀한 얇은 층으로 옮기고 방향 재조준, ② 대칭 Huber 가 면 뒤 벽 점에 끌림 → 면 뒤 2.5 척도 밖 가중치 0(한쪽 절단)·척도는 |r| 하위 25 % 분위수, ③ 남은 점의 엉뚱한 둘째 면 → 첫 면과 90° ± 10° 만. 실패 사례 s1_t11_06_1000080·1000091·1000095·1000119, s2_t11_06_1000190·1000192·1000218 을 리그에서 재현해 확인했다 |
| P1-5 | 부재 경계 증거가 원본 전체 광선에서 '면' 광선을 세 이웃 팔레트의 면도 증거가 됨(§7.2 위반); 시험이 다른 관문 때문에 통과 | `ray_samples` 가 `owned`(제안 마스크 화소)를 돌려주고 경계 안쪽 '면' 광선은 소유 화소만 센다. 직접 시험(소유 여부·창 밖 광선)과 관문 배선 시험(`observed_members` 를 1/2 로 바꿔 상태가 바뀌는지) |
| P1-6 | 지시 없는 선택이 §8 의 접근 비용 대신 카메라 거리; 끼움 불가 후보만 있으면 `none` 이라 §10 과 다름 | 끼움 불가만 남으면 가장 가까운 것을 `not_pickable` 로. 카메라 거리·"모두 도달 가능"은 2A 의 대용이고 경로 계획기 값은 실행기가 넘긴다고 모듈 주석·작업 5 에 적었다 |
| P2-7 | 배경 위 틀 시험이 성립하지 않음(최적은 −320 mm·점수 −0.499) | 성질 시험으로 바꿈 — 어느 부류의 광선을 빼도 최고 점수가 내려가지 않는다(`test_rays_never_add_score`) |
| P2-8 | 제안 하나의 예외가 장면 평가 전체를 멈춤(§10) | `fit_scene` 이 제안마다 예외를 격리해 `SceneFit.failures` 에 `exception:<type>`·`no_face` 로 남긴다. 예외 주입 시험 |
| P2-9 | 누출 시험이 키워드 이름만 봄; N·S6 방해물은 제안되지 않아 음성 오검출 0 이 구성상 값 | 정답·장면 라벨을 망쳐도 맞춤·선택 필드가 같은지 보는 통합 시험으로 바꿈. N·S6 의 0 은 기록에 "구성상 값"으로 적는다(작업 6 Step 7) |
| P2-10 | 계획 전 거리별 표는 예측 거리·대응 성공 조건부 | 표에 그렇게 표시했다. 작업 6 진단은 정답 거리(`truth_range_m`)와 대응 안 된 위험 후보(`dangerous`)를 따로 남긴다 |
| P2-11 | 인자 없는 `wait` 가 자식 실패를 삼킴(`false & true & wait` → 0) | sbatch 가 PID 마다 종료 상태와 `metrics.json` 존재를 확인하고 실패면 0 이 아닌 값으로 끝난다 |

**2회차 (새 P1 2 · P2 3, 미수렴).** 1회차 처리는 P1-1·2·3·5·6, P2-7·10·11 이 닫힘, P1-4·P2-8·P2-9 는 아래 새 지적으로 남음.

| # | 지적 | 처리 |
|---|---|---|
| P1 | `shape_mismatch` 가 허용된 파손을 거부(T11 y 면, `top_board_6` 결손, 뚫림 0.278, 관문 없으면 오차 0.3 mm) | 뚫림·점수의 '재료 자리 뚫림'에서 **면 뒤 recess_m(0.15 m) 안에서 제안 자신의 화소에 닿은 광선**(`ray_samples` 의 `recessed`)을 뺀다. 그 입력의 회귀 시험. 모든 자기 화소를 빼는 안은 S7 오적합을 55 개로 늘려 버렸다 |
| P1 | 맞춤에 실패한 제안을 빼면 애매한 지시가 자동 선택으로 바뀜(두 번째 팔레트에 예외 주입 → `selected(c0)`) | `FitFailure` 가 대략 위치를 갖고 `select_target(..., unresolved=...)` 이 반경 안(여유 0.4 m)이나 위치 모름이면 `reobserve(unresolved_in_radius)`. `_oracle_fit` 이 `fitted.failures` 를 넘긴다. 단위 시험 |
| P2 | 누출 시험이 포켓·yaw·선택을 비교하지 않음 | 진단에 `insertion_yaw_rad`·`left_m`·`right_m` 과 장면별 선택 줄·실패 줄을 넣고, 판정 필드만 뺀 전체를 비교 |
| P2 | 면 관문별 회귀 시험이 관문 삭제를 못 잡음 | ws1 탐색으로 관문 하나만 결과를 바꾸는 dev 레시피를 찾아(직각 s1_epal6_1000009, 바깥면 s2_t11_06_1000227, 척도 s2_t11_06_1000218) 시험이 "기본값이면 잘못된 면 없음, 그 관문만 끄면 생김"을 함께 확인 |
| P2 | 진단의 미대응 위험 규칙이 평가기와 다름 | `evaluation_multi.candidate_danger` 로 `judge_scene` 의 분기를 꺼내 판정과 진단이 같은 함수를 쓰게 함. 중복 후보 시험 |

Codex 는 0.08·0.12 를 dev 에서 정하는 것은 검출기 매개변수라 허용된다고 봤다(합격선 변경 아님). 변이 확인: 형상 관문을 끈 판에서 S7 시험 두 개가 실패한다(ws1).

**3회차 (P1 3 · P2 2, 미수렴).** 2회차의 면 관문별 회귀·위험 함수 공유는 닫힘.

| # | 지적 | 처리 |
|---|---|---|
| P1 | recess 0.15 m 가 인접 두 장 결손을 못 덮고(뚫림 0.238), 실제 개구에서도 차선 벽을 얕게 맞힘(1,404 중 257) — 깊이로 파손과 형상을 가를 수 없음 | recess 규칙을 버리고 §5.2 의 허용 파손 목록을 그대로 반영 — 비울 수 있는 상판 띠를 뚫림 근거에서 뺀다(`damageable_rows`). 한 장·두 장 결손 회귀 시험 |
| P1 | 한 장 결손에서 높이 12 mm 오차·거짓 `lift_alignment_needed`(시험이 높이를 버림) | 받침면이 보이면 측정 높이로 틀을 다시 놓는다. 시험이 높이·받침면·끼움 사유를 정답과 비교 |
| P1 | 실패 제안 여유 0.4 m 가 위치 편차(반대각선 0.5 m, S7 은 더 큼)를 못 덮음 | 가장 가까운 보이는 점 기준 + 최대 반대각선 0.85 m. Codex 반례 수치로 단위 시험 |
| P2 | 선택 진단 줄에도 `candidate_id` 가 있어 후보 줄 판별이 `KeyError` | 모든 줄에 `row` 종류 |
| P2 | 누출 시험이 지시 정답(대상·애매함)을 그대로 둬 선택 정답을 쓰는 변이를 못 잡음 | 다중 인스턴스 장면에서 대상을 다른 유효 ID 로, 애매함을 뒤집어 훼손 |

**4회차 (P1 2 · P2 1, 미수렴).** 3회차의 x = 2.6 m 결손 반례·unresolved 범위·`row`·지시 정답 훼손은 닫힘. 상판 띠 제외에서는 새 위반을 찾지 못했다고 했다.

| # | 지적 | 처리 |
|---|---|---|
| P1 | 근거리(2.04 m)에서 보이는 바닥을 `coplanar` 로 오인(바닥 점 1,605 개 모두 z = 0, 면 앞 21.6 mm) → 받침 8.2 mm·거짓 `lift_alignment_needed` | 받침면 판정을 아래 두 줄(+2·+6 화소)의 높이 차로 수평/수직을 가르게 바꿈. 그 입력을 결손 시험의 매개변수로 추가 |
| P1 | 받침면 보정 뒤 이전 배치의 판정(형상 불일치·점수 관문)이 그대로 붙음 | `_decide` 로 순위·관문을 묶고, 받침면이 보이면 그 면의 가설을 받침 고정으로 다시 만들어 `_decide` 를 다시 돈다. 판정·배치 일치 시험(잡음 mid seed 3, `min_score=0.6`) |
| P2 | unresolved 시험이 0.85 m 와 `extent_m` 을 따로 못 잡음(0.4 로 되돌려도·extent 0 이어도 통과) | Codex 반례를 extent 0 으로(0.4 면 실패), extent 가 있어야만 재관측되는 시험을 따로 |

Codex: 0.85 m 는 최대 반대각선 0.8485 m 를 덮어 기하 근거가 타당하나 `거리 − extent` 는 최근접 점의 하한이라 과잉 재관측이 있을 수 있다 — 작업 6 에서 `unresolved_in_radius` 발생 수를 따로 센다(진단의 선택 줄 사유로 집계).

**5회차 (P1 1 · P2 1, 미수렴).** 4회차의 근거리 결손·판정 일치·unresolved 변이·S7 전제는 모두 닫힘.

| # | 지적 | 처리 |
|---|---|---|
| P1 | 두 번째 판정이 다른 면을 고르면 그 면의 받침 보정 없이 끝남(T11 (2.6, −0.2) yaw 0.8, 5·6 결손: x 면·`pickable=True`·0.80 rad 반환, 모든 면 고정이면 y 면·불가·−0.77 rad) | 고른 면의 받침이 이미 고정됐거나 측정이 없을 때까지 반복(면마다 한 번, 최대 면 수 + 1 회). 그 입력의 회귀 시험 |
| P2 | 작업 6 커밋에 `evaluation_multi.py` 와 그 단위 시험이 빠져 `git archive` dev 실행에 새 함수가 없음 | 커밋 목록에 추가 |

관찰(Codex): 위층이 뒤로 물러난 얇은 받침 띠는 +2·+6 화소 줄이 띠를 벗어나 `uncertain` — 거짓 절대 층은 아니다. 착수 시 확인 항목에 올렸다.

**6회차 (P1 0 · P2 1, 미수렴).** 5회차의 반복 상한·커밋 목록·관찰 기록은 닫힘.

| # | 지적 | 처리 |
|---|---|---|
| P2 | 면 전환 시험이 출력된 축의 정답과 비교해, 반복 2 회 변이(x 면·3.2 mm·`pickable=True`)도 통과 | 기대 최종 상태를 명시(y 면·yaw −0.771 rad·`fitted`·받침 0 ± 2 mm·`near_tracking_unavailable`). ws1 잡 3115: 현재 판 79 개 통과, 반복 2 회 변이판에서 이 시험 실패 |

**7회차 — 수렴(새 P1/P2 없음).** Codex 가 면 전환 시험을 현재 판(통과)과 `range(2)` 변이(실패)로 직접 돌려 6회차 P2 가 닫혔음을 확인했고, 계획 코드·시험과 원본 파일의 AST 일치를 확인했다.

③ 의 Gemini 독립 검토는 1판(계획 코드 이전 판)에 대해 했다. 이후 바뀐 것은 Codex 지적에 따른 알고리즘 세부(면 추출·받침면·형상 관문·선택 규칙)이고 범위·전제는 그대로라 독립 검토를 다시 돌리지 않았다.

**계획 코드의 출처를 바꿨다.** 1회차에서 계획의 코드가 시제품과 어긋난 것(P1-4)이 드러나, 작업 1–5 의 코드블록은 세션 임시 폴더에서 시험 67 개를 통과한 파일을 그대로 잘라 넣는다(작업 2 = `instance_fit.py` 의 머리–`ray_samples`, 작업 3 = 격자·탐색, 작업 4 = 나머지; 시험도 같은 경계). 그 시험은 ws1 Slurm 에서 돌렸다(잡 아래 실측 절).
