# 팔레트 인식 일반화 단계 1B — 깊이 잡음 실측 맞춤과 Isaac 렌더 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **실행 방식(사용자 결정 2026-10-15, 단계 1A 와 같음):** 이 세션에서 직접 구현(executing-plans), 검토 지점마다 Codex 검토를 수렴까지 받고 진행한다.

**Goal:** 단계 1A 기준선은 판정이 잡음 모델에 좌우됨을 보였다. 이 계획은 (1) MR6D 의 실물 D435i 깊이로 잡음의 **크기와 공간 상관**을 재서 잡음 설정 파일로 고정하고, (2) 1A 장면 조리법을 Isaac 5.1 로 잡음 없이 렌더해 판 2 장면(`isaac_render_pose`)을 만들고 그 위에 실측 잡음을 입히며, (3) 그 장면에서 지금 검출기의 dev 기준선을 다시 낸다.

**Architecture:** 잡음은 코어에 둔다 — `depth_noise.py` 에 공간 상관 성분을 더하고(기본값 0 이면 1A 와 같은 출력), 측정은 `depth_noise_fit.py`(순수 numpy)가 한다. MR6D 내려받기·집계는 `tools/calibrate_depth_noise.py`. Isaac 은 **잡음 없는 렌더만** 맡고 잡음은 CPU 도구(`tools/apply_recognition_noise.py`)가 나중에 입힌다 — 잡음 설정이 바뀌어도 다시 렌더하지 않는다. Isaac 렌더는 Isaac 없이 시험 가능한 순수 부분(`sim/isaac/recognition_render_core.py`)과 얇은 Isaac 스크립트(`sim/isaac/render_recognition_scenes.py`)로 나눈다. 정답은 1A 와 같은 CPU 리그 계산(`scene_truths`)을 **Isaac 장면의 K 로** 돌려 만들고, 생성기와 같은 함수(`recipe_record`)를 쓴다. 비교 도구가 Isaac 렌더를 리그와 인스턴스·배경별로 대조하고, 전체 렌더 모두 이 대조를 통과해야 잡음을 입힌다.

**Tech Stack:** numpy, Pillow, PyYAML; Isaac Sim 5.1(ws1 `/home/projects/forklift/.venv`, `/opt/isaacsim-env` 위), Replicator 어노테이터(`sim/isaac/camera_calibration.py` 의 `attach_annotators`·`capture_static`), Slurm(ws1 `--gres=shard:N`, UBAI `tools/hpc_sweep.sh`).

**Spec:** `docs/design/2026-10-15-pallet-recognition-generality.md` §5.1·§5.3, §3.3–3.4. 단계 1A: `docs/plans/2026-10-15-recognition-stage1a-data-and-evaluator.md`, `docs/validation/2026-10-15-recognition-stage1a-cpu-baseline.md`.

## 계획 전 실측 (2026-10-15, 이 계획의 근거)

MR6D(Hugging Face `anas-gouda/mr6d`, 리비전 `3c4ed8453312655499c3d3a3cf38b4929fae77ee`, 2026-06-05) val 장면 `000000` 프레임 0(1280×720 uint16, `depth_scale` 1.0 → mm, fx 914.55, 최대 깊이 3.59 m, `mask_visib` 4 개 중 obj_id 1 = "wooden Euro pallet" 2 개)을 직접 읽었다. 시제품 코드는 세션 임시 폴더에만 있다.

| 항목 | 값 |
|---|---|
| 역깊이 2 차 차분의 강건 표준편차 /√6, 배경(모든 물체 마스크와 무효 화소에서 8 px 떨어진 화소, 깊이 p5–p95 2.43–3.05 m, 대부분 칠한 벽) | 간격 1 px: 0.00057 · 4 px: 0.00066 · 8 px: 0.0024 · 16 px: 0.0054 · 32 px: 0.0088 (가로) |
| 같은 프레임 팔레트 표면(팔레트 마스크를 4 px 깎은 화소, 0.97–1.60 m) | 간격 1 px: 0.00056 · 4 px: 0.00103 · 8 px: 0.00257 (가로). 작은 간격에서 벽과 같다. 큰 간격은 판 사이 틈이 섞여 부푼 상한이다 |
| 배경 48 px 칸마다 역깊이 평면을 맞춘 잔차(독립 검토가 같은 프레임에서 잼) | 중앙값 σ_w = 0.0063 — 칠한 벽의 저주파 변동이 장면 형상이 아닌 센서 공간 오차라는 교차 근거 |
| 작업 2 의 모형을 맞춘 결과(한 프레임) | 화소 독립 k₁ = 0.00054, 상관 k₂ = 0.0091, 상관 폭 g = (9.5, 6.1) px(1280 격자). 2.86 m 에서 화소 독립 σ 4 mm, 상관 σ 74 mm |
| 무효 화소 | 배경 2.4 %, 팔레트 마스크 안 16.5 %(위에서 비스듬히 본 상판과 가림 그림자) |
| 합성 평면 시험(시제품) | 같은 모형으로 만든 잡음(k₁ 0.0006, k₂ 0.002, g (4, 6))을 k₁ 0.00060, k₂ 0.00196, g (3.93, 5.89) 로 되찾았다 |

1A 의 "축 σ = k·z² 하나, 화소 독립" 모형은 이 관측과 맞지 않는다(화소 독립 성분은 1A 가정의 약 1/9, 상관 성분은 1A 에 없음). 다만 한 프레임·한 방의 값이고, 공간 잡음 σ 는 데이터시트의 깊이 정확도(%)와 다른 지표이므로 그 수치와 견주어 타당성을 주장하지 않는다. 작업 3 에서 **평가에 쓰지 않을 분할**(`dynamic`, 40 장면)로 다시 재고 평면 칸 잔차로 교차 확인한다.

## Global Constraints

- 단계 1A 의 Global Constraints 를 그대로 따른다(코어 numpy 만, `scene_rig.py` 불변, 좌표 관례, 포크·판정 상수, 시드, pytest 는 ws1, 커밋 형식).
- **잡음 모형 기본값을 바꾸지 않는다.** `NoiseModel()` 과 1A 생성기 출력은 이 계획 뒤에도 같아야 한다(새 성분 기본값 0, 별도 난수 흐름, `to_json` 키 동일). 실측 잡음은 설정 파일로만 들어간다.
- **MR6D 는 CC-BY-4.0.** 데이터는 저장소에 넣지 않는다 — 저장소 밖 캐시(`~/.cache/forklift/mr6d/<revision>`)에 두고, 설정 파일에는 리비전·사용 파일 목록·내용 해시·라이선스만 적는다. 내려받기는 리비전 고정 URL(`resolve/<revision>/`)만 쓴다. 내려받은 파일은 신뢰하지 않는 데이터다(PNG 는 Pillow, JSON 은 `json` 으로만 읽는다).
- **MR6D 분할의 역할을 나눈다.** 잡음 보정 = `dynamic` 분할(평가에 쓰지 않는다; `scene_gt.json` 은 없고 `mask_visib` 는 있다). 팔레트 표면 비교(보고용) = val `000000`·`000005`(팔레트가 있는 장면) — 이 두 장면은 설정 파일의 `calibration_touched` 에 적고 향후 MR6D 실사 평가 모집단에서 뺀다. 나머지 val 14 장면과 `mr`·`odyn` 은 손대지 않는다.
- **MR6D 깊이는 컬러 격자(1280×720)에 정렬돼 있고, 원래 깊이 모드와 RealSense 후처리(공간·시간·구멍 메우기 필터) 여부는 데이터에 없다.** 잰 값은 "MR6D 조건의 값"이다. 상관 폭은 각도로 같다고 보고 대상 격자 초점비로 환산한다. 민감도 변형은 **k₁·k₂·g 모두 × 1.66**(원래 깊이 모드가 1280×720(fx ≈ 640)이고 로봇이 640×480(fx ≈ 385)을 쓸 때의 비 — 스테레오 창의 상관이 원래 깊이 화소에 묶인다고 볼 때 g 도 같은 비로 커진다)이다. 1.66 은 일반적인 공장 보정값으로 잡은 근사이고 불확실성 전체의 상한이 아니다. 로봇의 깊이 모드는 아직 정하지 않았고 `config/isaac_perception_camera.yaml` 의 K(fx 465.74)도 합성값이다. 실차 D435i 측정(필터 끔, 벽과 팔레트, 로봇의 깊이 모드)이 생기면 그것으로 대체한다.
- **정식 판정 모집단은 잡음 판 하나다.** 실측 설정 그대로(배율 1)의 판이 정식(`role = "official"`), 배율 1.66 판은 민감도(`role = "sensitivity"`)다. 평가기는 한 집계 안에 잡음 설정(해시·배율)이 둘 이상이면 거부하고, `official` 은 Isaac 이면서 모든 장면이 `role = "official"` 일 때만 참이다. S6·N 은 설계 §5.1 의 창고 자산·다른 출처 구조 대신 상자만 쓰므로 이 단계에서는 **잠정**으로 기록한다(자산은 단계 3).
- Isaac 은 ws1 Slurm 에서만: `--partition=compute --gres=shard:1 --qos=admin`, Isaac 프로세스 하나당 shard 1·RAM 6G(실측 뒤 조정)·시간은 실측에 맞춘다. 로그인 셸에서 Isaac 을 띄우지 않는다. 산출물은 `/home/projects/forklift/artifacts/<UTC>_recognition_isaac/`. CPU 만 쓰는 측정(작업 3)은 UBAI.
- Isaac 장면 메타: `origin = "isaac_render_pose"`, `render_source = "isaac"`, `source_provenance = "synthetic"`, `clock_domain = "synthetic"`, `meta.truth_render = "rig"`.
- **eval 분할 규칙:** 검출기는 dev 에서만 돌린다. 렌더 검증(작업 8 의 리그 대조)은 검출기를 돌리지 않는 기하 대조라 eval 장면도 포함하되, eval 장면의 영상은 열어 보지 않는다. 연기 시험(작업 7)은 dev 장면만 렌더한다.

## Review Focus

1. **잡음 측정의 식별** — 2 차 차분은 평면 하나만 지운다. 라벨 없는 가구·표면 경계·여러 프레임 혼합이 큰 간격 분산을 부풀릴 수 있다. 프레임마다 따로 맞추고 분포를 보고하며, 평면 칸 잔차가 모형 예측과 맞을 때만 채택한다. → 작업 2·3.
2. **1A 재현성** — 새 잡음 성분과 생성기 정리가 기본 출력(깊이·ID·RGB·K·TF·메타·정답)을 바꾸면 1A 와 비교할 수 없다. → 작업 1·4.
3. **Isaac 과 리그의 기하·관례 불일치** — 상자·바닥·벽·카메라 자세·K(반 화소 관례)와, 정답을 같은 K 로 계산하는지. 인스턴스마다 IoU·깊이·유효 깊이 비율을, 배경은 바닥 높이·벽 위치를 따로 본다. → 작업 4·6·7·8.
4. **이전 장면의 라벨이 남은 프레임, 준비 안 된 어노테이터** — 장면 토큰 라벨과, 빈 버퍼를 "아직"으로 다루는 준비 판정. → 작업 5·6.
5. **출처·잡음 설정이 섞인 집계** — 렌더 출처·잡음 설정·역할이 섞이면 거부, 이미 잡음이 있는 장면에 다시 입히면 거부. → 작업 4·8.

## 파일 구조

| 파일 | 책임 |
|---|---|
| `src/forklift_core/perception/depth_noise.py` (수정) | 공간 상관 성분, `gaussian_kernel`, `correlated_field`, `NoiseModel.from_yaml` |
| `src/forklift_core/perception/depth_noise_fit.py` (신규) | 역깊이 2 차 차분, 강건 분산, 잡음 구조 맞춤, 평면 칸 잔차와 모형 예측, 배경 마스크, 마스크 깎기, 무효 화소 비율 |
| `tools/calibrate_depth_noise.py` (신규) | MR6D 목록 고정·내려받기·프레임별 측정·채택·`config/depth_noise_d435i.yaml` 작성 |
| `config/depth_noise_d435i.yaml` (신규, 생성물) | 잡음 설정 |
| `deploy/hpc/recognition_stage1b_calibrate.tsv` (신규) | UBAI 측정 잡 |
| `tools/recognition_truth.py` (수정) | `scene_truths(..., intrinsics=None)` |
| `tools/generate_recognition_scenes.py` (수정) | `recipe_record`·`noise_seed`·`load_site` 공유 |
| `tools/apply_recognition_noise.py` (신규) | 잡음 없는 장면에 설정 파일 잡음(배율·역할)을 입힌다 |
| `src/forklift_core/perception/evaluation_multi.py`, `tools/evaluate_recognition.py` (수정) | 잡음 역할·설정 기록, 섞임 거부, `official` 조건 |
| `sim/isaac/recognition_render_core.py` (신규, Isaac 없음) | 상자 명세, 토큰 라벨, 의미 분할 해석·준비 판정, 깊이 정리 |
| `sim/isaac/render_recognition_scenes.py` (신규, Isaac) | 장면별 prim·라벨·캡처 → 판 2 (잡음 없음) |
| `tools/compare_isaac_rig.py` (신규) | Isaac 장면과 같은 K 의 리그 렌더 대조, K 규약 검사 |
| `deploy/slurm/recognition_isaac_render.sbatch` (신규) | 렌더 잡 |
| `docs/validation/<실행일>-recognition-stage1b-isaac-baseline.md` (신규) | 기록 |

## 검토 지점

- **검토 A (작업 1–4 뒤):** 잡음 모형·측정·MR6D 결과·설정 파일·잡음 입히기·생성기 정리·평가기 변경.
- **검토 B (작업 5–7 뒤):** 렌더 코어·Isaac 스크립트·비교 도구·연기 결과.
- **검토 C (작업 8 뒤):** 전체 렌더·대조·dev 기준선·기록.

---

### Task 1: 잡음 모형에 공간 상관 성분 추가

**Files:**
- Modify: `src/forklift_core/perception/depth_noise.py`
- Test: `tests/unit/perception/test_depth_noise.py`(추가)

**Interfaces:**
- Produces:
  - `NoiseModel` 새 필드 `correlated_k: float = 0.0`, `correlation_px: tuple = (0.0, 0.0)`(가로 u, 세로 v 의 가우스 폭, **출력 격자 화소**). `to_json()` 은 `correlated_k > 0` 일 때만 두 키를 넣는다.
  - `gaussian_kernel(sigma_px) -> NDArray`(합 1, 반경 ⌈4σ⌉, σ ≤ 0 이면 [1])
  - `correlated_field(shape, sigma_px, rng) -> NDArray` — 커널 길이만큼 넓게 뽑은 백색 잡음을 분리형 가우스로 거른 **분산 1** 장(경계에서도 정상성 유지)
  - `apply_noise` — `correlated_k > 0` 이면 화소 독립 잡음 다음에 `field · correlated_k · z²` 를 더한다. 난수는 `default_rng([seed, 1])` 로 따로 뽑는다.

- [ ] **Step 1: 실패하는 시험 작성** — 파일 위쪽에 `import pytest`, `from dataclasses import replace`, `from forklift_core.perception.depth_noise import correlated_field` 를 더하고(기존 `QUIET` 를 그대로 쓴다) 끝에 추가:

```python
def test_correlated_field_has_unit_variance_and_gaussian_correlation():
    field = correlated_field((400, 600), (3.0, 5.0), np.random.default_rng(1))
    assert field.std() == pytest.approx(1.0, abs=0.05)
    for lag, g, axis in ((3, 3.0, 1), (5, 5.0, 0)):
        a = np.moveaxis(field, axis, 0)
        assert np.mean(a[lag:] * a[:-lag]) == pytest.approx(np.exp(-lag * lag / (4 * g * g)), abs=0.05)


def test_zero_correlated_k_leaves_the_stage1a_noise_untouched():
    out = instance_rig.render_instances([[scene_rig.Box((2.0, 0.0, 0.15), (0.3, 0.6, 0.3))]])
    base = NoiseModel(seed=11)
    a, ids_a = apply_noise(out.scene, out.instance_ids, base)
    b, ids_b = apply_noise(out.scene, out.instance_ids, replace(base, correlation_px=(5.0, 5.0)))
    assert np.array_equal(a.depth_m, b.depth_m, equal_nan=True) and np.array_equal(ids_a, ids_b)
    assert np.array_equal(a.rgb, b.rgb)
    assert set(base.to_json()) == {"axial_k", "edge_jump_m", "flying_fraction", "registration_shift_px",
                                   "dropout_fraction", "quantize_step_m", "seed"}


def test_correlated_noise_grows_with_depth_squared():
    out = instance_rig.render_instances([], floor=False)  # back wall only: z = 5.25 m everywhere
    model = NoiseModel(axial_k=0.0, correlated_k=0.002, correlation_px=(3.0, 3.0), seed=2, **QUIET)
    noisy, _ = apply_noise(out.scene, out.instance_ids, model)
    residual = noisy.depth_m - out.scene.depth_m
    assert residual.std() == pytest.approx(0.002 * 5.25 ** 2, rel=0.1)
    assert model.to_json()["correlation_px"] == [3.0, 3.0]
```

- [ ] **Step 2: 실패 확인** — Run: `PYTHONPATH=src:. python -m pytest tests/unit/perception/test_depth_noise.py -q -p no:cacheprovider` · Expected: `ImportError: cannot import name 'correlated_field'`
- [ ] **Step 3: 구현** — `depth_noise.py`:

```python
@dataclass(frozen=True)
class NoiseModel:
    axial_k: float = 0.005
    edge_jump_m: float = 0.05
    flying_fraction: float = 0.5
    registration_shift_px: tuple = (2, 0)
    dropout_fraction: float = 0.01
    quantize_step_m: float = 0.001
    seed: int = 0
    correlated_k: float = 0.0
    correlation_px: tuple = (0.0, 0.0)

    def to_json(self) -> dict:
        obj = asdict(self)
        obj["registration_shift_px"] = list(self.registration_shift_px)
        if self.correlated_k > 0:
            obj["correlation_px"] = [float(v) for v in self.correlation_px]
        else:
            del obj["correlated_k"], obj["correlation_px"]
        return obj


def gaussian_kernel(sigma_px: float):
    if sigma_px <= 0:
        return np.ones(1)
    radius = int(np.ceil(4 * sigma_px))
    x = np.arange(-radius, radius + 1)
    kernel = np.exp(-0.5 * np.square(x / sigma_px))
    return kernel / kernel.sum()


def correlated_field(shape, sigma_px, rng):
    """Unit-variance Gaussian-filtered white noise; sigma_px = (along u, along v) in output pixels."""
    ku, kv = gaussian_kernel(sigma_px[0]), gaussian_kernel(sigma_px[1])
    h, w = shape
    white = rng.normal(0.0, 1.0, (h + len(kv) - 1, w + len(ku) - 1))
    rows = sum(c * white[:, i:i + w] for i, c in enumerate(ku))
    field = sum(c * rows[j:j + h, :] for j, c in enumerate(kv))
    return field / np.sqrt(np.sum(np.square(ku)) * np.sum(np.square(kv)))
```

`apply_noise` 의 화소 독립 잡음 줄 바로 다음:

```python
    if model.correlated_k > 0:
        field = correlated_field(clean.shape, model.correlation_px, np.random.default_rng([model.seed, 1]))
        depth = np.where(finite, depth + field * model.correlated_k * np.square(np.where(finite, clean, 0.0)), depth)
```

모듈 docstring 에 "optional Gaussian-correlated k_corr z^2 component (stage 1B, measured)" 한 줄을 더한다. 장마다 새 상관 장을 뽑으므로 시간상 고정된 패턴(같은 시점에서 반복되는 오차)은 표현하지 않는다 — 다중 프레임 융합의 이득을 과대평가할 수 있다는 점을 작업 3 의 한계 목록에 적는다.

- [ ] **Step 4: 통과 확인** — Expected: 파일 전체 통과(기존 시험 포함).
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/depth_noise.py tests/unit/perception/test_depth_noise.py` · `git commit -m "perception: depth noise gains a spatially correlated k*z^2 component (Gaussian-filtered unit field, own rng stream); defaults and stage-1A output unchanged"`

### Task 2: 잡음 구조 측정 코어 `depth_noise_fit`

**Files:**
- Create: `src/forklift_core/perception/depth_noise_fit.py`
- Test: `tests/unit/perception/test_depth_noise_fit.py`

**Interfaces:**
- Consumes: 작업 1 의 `NoiseModel`, `apply_noise`, `correlated_field`
- Produces:
  - `STEPS = (1, 2, 3, 4, 6, 8, 12, 16, 24, 32)`, `GRID_PX = np.geomspace(0.5, 40.0, 120)`
  - `second_differences(depth_m, *, axis, step, z_range=(1.5, 4.0), valid=None) -> NDArray` — 역깊이 `w[i−s] − 2w[i] + w[i+s]`(세 화소 모두 유효·`valid`, 가운데 깊이가 `z_range` 안). `axis=1` 은 가로(u), `axis=0` 은 세로(v).
  - `robust_variance(samples) -> float` — (1.4826·MAD)², 1,000 개 미만이면 `ValueError`
  - `NoiseStructure` (frozen): `k_iid`, `k_corr`, `correlation_px: tuple`(u, v), `rel_residual_rms`, `at_grid_edge: bool`
  - `fit_noise_structure(var_u, var_v) -> NoiseStructure` — V(s) = 6A + B(6 − 8ρ(s) + 2ρ(2s)), ρ(s) = exp(−s²/(4g²)); g_u·g_v 를 `GRID_PX` 에서 찾고 A·B 는 비음수 가중 최소제곱(가중치 1/V). `at_grid_edge` 는 k_corr ≥ 0.1·k_iid 인데 g 가 격자 양 끝 점에 걸렸을 때 참.
  - `patch_plane_rms(depth_m, valid, *, patch=48, z_range=(1.5, 4.0)) -> NDArray` — 겹치지 않는 patch×patch 칸 중 모두 유효하고 평균 깊이가 범위 안인 칸마다 역깊이 평면(화소 좌표 1 차식) 잔차 RMS(자유도 보정)
  - `model_patch_rms(structure, *, patch=48, size=(720, 1280), seed=0) -> float` — 같은 구조의 역깊이 잡음 장(화소 독립 + 상관)에서 `patch_plane_rms` 와 같은 칸 통계의 중앙값
  - `background_mask(depth_m, object_masks, *, margin_px=8) -> NDArray[bool]`, `erode(mask, radius) -> NDArray[bool]`(적분 영상 정사각 팽창·깎기)
  - `dropout_fraction(depth_m, region) -> float`

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/perception/test_depth_noise_fit.py
import numpy as np
import pytest

from forklift_core.perception import depth_noise_fit as fit
from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from tools import instance_rig

QUIET = dict(edge_jump_m=1e9, dropout_fraction=0.0, quantize_step_m=0.0, registration_shift_px=(0, 0))
WALL = (5.0, 6.0)  # the rig's back wall sits at z = 5.25 m
MIXED = dict(axial_k=0.0006, correlated_k=0.004, correlation_px=(4.0, 3.0))


def _wall():
    return instance_rig.render_instances([], floor=False)


def _variances(depth, valid=None):
    return [{s: fit.robust_variance(fit.second_differences(depth, axis=axis, step=s, z_range=WALL, valid=valid))
             for s in fit.STEPS} for axis in (1, 0)]


def test_a_plane_has_zero_second_differences():
    d = fit.second_differences(_wall().scene.depth_m, axis=1, step=4, z_range=WALL)
    assert d.size > 100_000 and np.max(np.abs(d)) < 1e-12


def test_iid_noise_is_recovered_without_a_correlated_part():
    out = _wall()
    noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(axial_k=0.0006, seed=3, **QUIET))
    s = fit.fit_noise_structure(*_variances(noisy.depth_m))
    assert s.k_iid == pytest.approx(0.0006, rel=0.10) and s.k_corr < 0.2 * 0.0006


def test_iid_and_correlated_noise_are_separated():
    out = _wall()
    noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=4, **MIXED, **QUIET))
    s = fit.fit_noise_structure(*_variances(noisy.depth_m))
    assert s.k_iid == pytest.approx(0.0006, rel=0.15) and s.k_corr == pytest.approx(0.004, rel=0.15)
    assert s.correlation_px[0] == pytest.approx(4.0, rel=0.2) and s.correlation_px[1] == pytest.approx(3.0, rel=0.2)
    assert not s.at_grid_edge


def test_patch_plane_rms_matches_the_model_prediction():
    out = _wall()
    noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=5, **MIXED, **QUIET))
    observed = np.median(fit.patch_plane_rms(noisy.depth_m, np.ones((480, 640), bool), patch=48, z_range=WALL))
    s = fit.NoiseStructure(0.0006, 0.004, (4.0, 3.0), 0.0, False)
    assert observed == pytest.approx(fit.model_patch_rms(s, patch=48, size=(480, 640)), rel=0.10)
    assert fit.patch_plane_rms(out.scene.depth_m, np.ones((480, 640), bool), z_range=WALL).max() < 1e-12


def test_robust_variance_ignores_sparse_outliers():
    x = np.random.default_rng(0).normal(0.0, 1.0, 20_000)
    x[::20] = 1e3
    assert fit.robust_variance(x) == pytest.approx(1.0, rel=0.15)
    with pytest.raises(ValueError):
        fit.robust_variance(x[:999])


def test_background_mask_and_erosion_keep_a_margin():
    depth = np.full((40, 60), 2.0)
    depth[5, 50] = np.nan
    obj = np.zeros(depth.shape, bool)
    obj[20:25, 10:15] = True
    bg = fit.background_mask(depth, [obj], margin_px=3)
    assert not bg[17, 12] and not bg[20, 12] and bg[16, 12]
    assert not bg[5, 47] and bg[5, 46] and bg[0, 0]
    inner = fit.erode(obj, 1)
    assert inner.sum() == 9 and inner[22, 12] and not inner[20, 12]


def test_dropout_fraction_counts_zero_and_nan():
    depth = np.array([[1.0, 0.0], [np.nan, 2.0]])
    assert fit.dropout_fraction(depth, np.ones_like(depth, bool)) == pytest.approx(0.5)
```

- [ ] **Step 2: 실패 확인** — Run: `PYTHONPATH=src:. python -m pytest tests/unit/perception/test_depth_noise_fit.py -q -p no:cacheprovider` · Expected: `ModuleNotFoundError`
- [ ] **Step 3: 구현**

```python
# src/forklift_core/perception/depth_noise_fit.py
"""Measure depth-noise structure from real depth images (design §5.3, stage 1B).

Stereo depth noise is roughly constant in inverse depth w = 1/z, i.e. sigma_z = k z^2.
A plane is exactly linear in w over pixel coordinates, so the second difference
w[i-s] - 2 w[i] + w[i+s] removes a plane and keeps the noise. Its variance against
the step s separates a per-pixel part (A) from a spatially correlated part (B,
Gaussian-filtered white noise of width g pixels):

    V(s) = 6 A + B (6 - 8 rho(s) + 2 rho(2 s)),   rho(s) = exp(-s^2 / (4 g^2)).

A second difference cannot tell sensor error from real surface shape, so callers
fit each frame separately on masked background and cross-check the fitted model
against per-patch plane residuals (patch_plane_rms vs model_patch_rms).
"""

from dataclasses import dataclass

import numpy as np

from forklift_core.perception.depth_noise import correlated_field

STEPS = (1, 2, 3, 4, 6, 8, 12, 16, 24, 32)
GRID_PX = np.geomspace(0.5, 40.0, 120)


@dataclass(frozen=True)
class NoiseStructure:
    k_iid: float
    k_corr: float
    correlation_px: tuple
    rel_residual_rms: float
    at_grid_edge: bool


def _inverse(depth_m, valid):
    depth = np.asarray(depth_m, dtype=float)
    ok = np.isfinite(depth) & (depth > 0)
    if valid is not None:
        ok &= np.asarray(valid, dtype=bool)
    return np.where(ok, 1.0 / np.where(ok, depth, 1.0), np.nan), np.where(ok, depth, np.nan)


def second_differences(depth_m, *, axis: int, step: int, z_range=(1.5, 4.0), valid=None):
    w, z = _inverse(depth_m, valid)
    a = np.moveaxis(w, axis, 0)
    centre = np.moveaxis(z, axis, 0)[step:-step]
    d = a[:-2 * step] - 2.0 * a[step:-step] + a[2 * step:]
    keep = np.isfinite(d) & (centre >= z_range[0]) & (centre < z_range[1])
    return d[keep]


def robust_variance(samples) -> float:
    x = np.asarray(samples, dtype=float)
    if x.size < 1000:
        raise ValueError("robust variance needs at least 1000 samples")
    return float(np.square(1.4826 * np.median(np.abs(x - np.median(x)))))


def _rho(s, g):
    return np.exp(-np.square(s) / (4.0 * g * g))


def _shape(steps, g):
    s = np.asarray(steps, dtype=float)
    return 6.0 - 8.0 * _rho(s, g) + 2.0 * _rho(2.0 * s, g)


def _nonnegative_lstsq(x, y):
    coef = np.linalg.lstsq(x, y, rcond=None)[0]
    if np.all(coef >= 0):
        return coef
    best = None
    for j in range(x.shape[1]):
        c = np.zeros(x.shape[1])
        col = x[:, j]
        c[j] = max(0.0, float(col @ y / (col @ col)))
        r = float(np.sum(np.square(x @ c - y)))
        if best is None or r < best[0]:
            best = (r, c)
    return best[1]


def fit_noise_structure(var_u: dict, var_v: dict) -> NoiseStructure:
    su, sv = sorted(var_u), sorted(var_v)
    y = np.array([var_u[s] for s in su] + [var_v[s] for s in sv], dtype=float)
    if not np.all(np.isfinite(y)) or np.any(y <= 0):
        raise ValueError("variances must be finite and positive")
    weight = 1.0 / y
    best = None
    for gu in GRID_PX:
        cu = _shape(su, gu)
        for gv in GRID_PX:
            x = np.column_stack((np.full(y.size, 6.0), np.concatenate((cu, _shape(sv, gv)))))
            coef = _nonnegative_lstsq(x * weight[:, None], y * weight)
            r = float(np.sum(np.square((x @ coef - y) * weight)))
            if best is None or r < best[0]:
                best = (r, coef, gu, gv)
    r, (a, b), gu, gv = best
    k_iid, k_corr = float(np.sqrt(a)), float(np.sqrt(b))
    edge = k_corr >= 0.1 * k_iid and any(g in (GRID_PX[0], GRID_PX[-1]) for g in (gu, gv))
    return NoiseStructure(k_iid, k_corr, (float(gu), float(gv)), float(np.sqrt(r / y.size)), bool(edge))


def _tiles(array, patch):
    h, w = array.shape
    rows, cols = h // patch, w // patch
    t = array[: rows * patch, : cols * patch].reshape(rows, patch, cols, patch).transpose(0, 2, 1, 3)
    return t.reshape(rows * cols, patch * patch)


def _plane_rms(w_tiles, patch):
    v, u = np.mgrid[0:patch, 0:patch]
    design = np.column_stack((u.ravel(), v.ravel(), np.ones(patch * patch)))
    residual = w_tiles - w_tiles @ (design @ np.linalg.pinv(design)).T
    n = patch * patch
    return np.sqrt(np.mean(np.square(residual), axis=1) * n / (n - 3))


def patch_plane_rms(depth_m, valid, *, patch: int = 48, z_range=(1.5, 4.0)):
    w, z = _inverse(depth_m, valid)
    wt, zt = _tiles(w, patch), _tiles(z, patch)
    keep = np.all(np.isfinite(wt), axis=1)
    mean_z = np.where(keep, np.nanmean(np.where(np.isfinite(zt), zt, 0.0), axis=1), np.nan)
    keep &= (mean_z >= z_range[0]) & (mean_z < z_range[1])
    return _plane_rms(wt[keep], patch)


def model_patch_rms(structure: NoiseStructure, *, patch: int = 48, size=(720, 1280), seed: int = 0) -> float:
    rng = np.random.default_rng(seed)
    field = structure.k_iid * rng.normal(0.0, 1.0, size)
    if structure.k_corr > 0:
        field = field + structure.k_corr * correlated_field(size, structure.correlation_px, rng)
    return float(np.median(_plane_rms(_tiles(field, patch), patch)))


def _box_any(mask, radius: int):
    m = np.pad(np.asarray(mask, dtype=np.int64), ((radius + 1, radius), (radius + 1, radius)))
    c = m.cumsum(0).cumsum(1)
    h, w = mask.shape
    e = 2 * radius + 1
    return (c[e:e + h, e:e + w] - c[:h, e:e + w] - c[e:e + h, :w] + c[:h, :w]) > 0


def erode(mask, radius: int):
    return ~_box_any(~np.asarray(mask, dtype=bool), radius)


def background_mask(depth_m, object_masks, *, margin_px: int = 8):
    depth = np.asarray(depth_m, dtype=float)
    bad = ~(np.isfinite(depth) & (depth > 0))
    for mask in object_masks:
        bad |= np.asarray(mask, dtype=bool)
    return ~_box_any(bad, margin_px)


def dropout_fraction(depth_m, region) -> float:
    values = np.asarray(depth_m, dtype=float)[np.asarray(region, dtype=bool)]
    if not values.size:
        raise ValueError("empty region")
    return float(np.mean(~np.isfinite(values) | (values <= 0)))
```

(`erode` 는 `~mask` 를 팽창하므로 영상 밖은 "마스크 밖"으로 보지 않는다 — 영상 테두리에 닿은 마스크는 테두리 쪽으로 깎이지 않는다. 시험의 물체는 영상 안쪽에 있다.)

- [ ] **Step 4: 통과 확인** — Expected: 7 passed. 회복·예측 시험이 빗나가면 구현을 고친다 — 시험 문턱은 바꾸지 않는다.
- [ ] **Step 5: 커밋** — `git add src/forklift_core/perception/depth_noise_fit.py tests/unit/perception/test_depth_noise_fit.py` · `git commit -m "perception: depth-noise structure from inverse-depth second differences -- robust variogram, iid plus Gaussian-correlated fit, patch-plane cross-check, background mask, dropout fraction"`

### Task 3: MR6D 측정 도구와 잡음 설정 파일

**Files:**
- Create: `tools/calibrate_depth_noise.py`, `tests/unit/test_calibrate_depth_noise.py`, `deploy/hpc/recognition_stage1b_calibrate.tsv`, `config/depth_noise_d435i.yaml`(UBAI 실행 결과)
- Modify: `src/forklift_core/perception/depth_noise.py`(`NoiseModel.from_yaml`)
- Test: `tests/unit/perception/test_depth_noise.py`(설정 검사)

**Interfaces:**
- `NoiseModel.from_yaml(path, *, seed: int, target_fx: float, scale: float = 1.0) -> NoiseModel` — `axial_k = k_iid·scale`, `correlated_k = k_corr·scale`, `correlation_px = correlation_px_source · (target_fx / source_fx) · scale`, `dropout_fraction` 는 파일 값, 나머지는 기본값. 검사: `source`·`license`·`revision`·`files_sha256` 이 비어 있지 않은 문자열, `k_iid`·`k_corr` ≥ 0 유한, `correlation_px_source` 길이 2 의 양수, `source_fx` 양수, `dropout_fraction` ∈ [0, 1), `target_fx`·`scale` 양의 유한수. 어긋나면 `ValueError("noise config ...")`.
- `tools/calibrate_depth_noise.py`:
  - 상수 `REVISION = "3c4ed8453312655499c3d3a3cf38b4929fae77ee"`, `CALIBRATION_SCENES = dynamic/000000 … dynamic/000039`, `PALLET_SCENES = ("val/000000", "val/000005")`.
  - `snapshot_listing(cache) -> list[str]` — 리비전 API(`…/api/datasets/anas-gouda/mr6d/revision/<revision>`)의 `siblings` 를 `cache/listing.json` 에 저장·재사용한다(API 가 돌려준 `sha` 가 리비전과 다르면 거부). 프레임 선택은 **이 목록에서만** 정한다(캐시가 더 조밀해도 같은 입력).
  - `select_frames(listing, scenes, stride) -> list[dict]` — 장면마다 정렬한 `depth/*.png` 의 `[::stride]` 와 그 프레임의 `mask_visib/<frame>_*.png`.
  - `download(cache, frames, listing)` — `resolve/<REVISION>/` 로 `scene_camera.json`, 있으면 `scene_gt.json`, `models_names.json`, 선택된 깊이·마스크를 받는다(있으면 재사용).
  - `frame_structure(depth, masks, *, z_range, margin_px) -> dict` — 배경 마스크 → 간격·축별 2 차 차분 강건 분산 → `fit_noise_structure`, 평면 칸 관측 중앙값, 밴드별 맞춤((1.5, 2.5), (2.5, 4.0), (4.0, 6.0); 표본이 모자라면 없음), 배경 무효 화소 비율, 배경 깊이 분위.
  - `calibrate(cache, frames, *, z_range=(1.5, 4.0), margin_px=8, accept_fraction=0.75) -> dict` — 프레임마다 `frame_structure`. 채택 조건: 모든 간격·축 표본 ≥ 1,000, 완전히 유효한 48 px 평면 칸 ≥ `MIN_PATCHES`(20) — 교차 확인을 할 수 없는 프레임은 채택하지 않는다, `at_grid_edge` 거짓, `rel_residual_rms` ≤ 0.15. 채택 프레임 비율이 `accept_fraction` 미만이면 `ValueError`. 채택값은 채택 프레임의 k₁·k₂·g 각각의 중앙값, 분포(p10·p50·p90·개수)와 버린 사유별 개수를 함께 기록. 교차 확인: 프레임마다 (관측 평면 칸 중앙값) / `model_patch_rms(채택 구조)` 의 중앙값이 [0.67, 1.5] 밖이면 `ValueError`.
  - `pallet_surface_report(cache, frames, *, z_range=(0.5, 2.5)) -> dict` — `PALLET_SCENES` 프레임에서 obj 이름에 "pallet" 이 든 마스크를 4 px 깎은 영역과 같은 프레임 배경의 간격 1–8 분산, 비(팔레트/배경)의 분포, 팔레트 무효 화소 비율. **보고만** 한다.
  - 출력 YAML 키: `source`, `license`, `revision`, `retrieved`, `method`, `calibration_split`, `calibration_touched`, `files`, `files_sha256`, `source_grid`, `source_fx`, `z_range_m`, `margin_px`, `steps`, `k_iid`, `k_corr`, `correlation_px_source`, `per_frame`(분포), `rejected`(사유별), `band_fits`, `patch_check`, `dropout_fraction`(배경 중앙값), `background_depth_m`, `pallet_surface`, `frames`, `limitations`.

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/test_calibrate_depth_noise.py
import json

import numpy as np
import pytest
from PIL import Image

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from tools import calibrate_depth_noise as cal
from tools import instance_rig

QUIET = dict(edge_jump_m=1e9, dropout_fraction=0.0, quantize_step_m=0.0, registration_shift_px=(0, 0))
TRUE = dict(axial_k=0.0006, correlated_k=0.004, correlation_px=(4.0, 3.0))


def _cache(root, scene="dynamic/000000", frames=4, with_gt=False):
    """A tiny BOP-layout cache: a noisy wall at z = 5.25 m with one masked object per frame."""
    out = instance_rig.render_instances([], floor=False)
    i = out.scene.intrinsics
    base = root / scene
    (base / "depth").mkdir(parents=True)
    (base / "mask_visib").mkdir()
    (root / "models_names.json").write_text(json.dumps({"1": "wooden Euro pallet", "2": "small grey KLT"}))
    names, cams, gts = ["models_names.json", f"{scene}/scene_camera.json"], {}, {}
    for f in range(frames):
        noisy, _ = apply_noise(out.scene, out.instance_ids, NoiseModel(seed=f, **TRUE, **QUIET))
        raw = np.rint(noisy.depth_m * 1000).astype(np.uint16)
        raw[200:240, 300:400] = 0
        Image.fromarray(raw).save(base / "depth" / f"{f:06d}.png")
        mask = np.zeros(raw.shape, np.uint8)
        mask[180:260, 280:420] = 255
        Image.fromarray(mask).save(base / "mask_visib" / f"{f:06d}_000000.png")
        names += [f"{scene}/depth/{f:06d}.png", f"{scene}/mask_visib/{f:06d}_000000.png"]
        cams[str(f)] = {"cam_K": [i.fx, 0, i.cx, 0, i.fy, i.cy, 0, 0, 1], "depth_scale": 1.0}
        gts[str(f)] = [{"obj_id": 1}]
    (base / "scene_camera.json").write_text(json.dumps(cams))
    if with_gt:
        (base / "scene_gt.json").write_text(json.dumps(gts))
        names.append(f"{scene}/scene_gt.json")
    (root / "listing.json").write_text(json.dumps(names))
    return i.fx


def _frames(root, scene, stride=1):
    return cal.select_frames(json.loads((root / "listing.json").read_text()), [scene], stride)


def test_frames_come_from_the_listing_not_from_the_cache(tmp_path):
    _cache(tmp_path)
    frames = _frames(tmp_path, "dynamic/000000", 2)
    assert [f["frame"] for f in frames] == [0, 2]
    assert frames[0]["masks"] == ["dynamic/000000/mask_visib/000000_000000.png"]


def test_calibration_recovers_known_noise_and_passes_the_patch_check(tmp_path):
    fx = _cache(tmp_path)
    result = cal.calibrate(tmp_path, _frames(tmp_path, "dynamic/000000"), z_range=(5.0, 6.0))
    assert result["k_iid"] == pytest.approx(0.0006, rel=0.15) and result["k_corr"] == pytest.approx(0.004, rel=0.15)
    assert result["correlation_px_source"][0] == pytest.approx(4.0, rel=0.2)
    assert result["source_fx"] == pytest.approx(fx) and result["frames"] == 4 and result["per_frame"]["accepted"] == 4
    assert 0.9 < result["patch_check"]["ratio_median"] < 1.1
    assert result["dropout_fraction"] < 0.01 and result["license"] == "CC-BY-4.0"


def test_pallet_surface_report_uses_the_pallet_masks(tmp_path):
    _cache(tmp_path, scene="val/000000", with_gt=True)
    report = cal.pallet_surface_report(tmp_path, _frames(tmp_path, "val/000000"), z_range=(5.0, 6.0))
    assert report["scenes"] == ["val/000000"]
    assert report["pallet_dropout_fraction"] == pytest.approx(4000 / 11200, rel=0.01)
    assert set(report["ratio_u"]) == {1, 2, 3, 4, 6, 8}


def test_frames_without_full_plane_patches_are_not_accepted(tmp_path):
    _cache(tmp_path)
    from PIL import Image as _Image

    for f in range(3):  # holes on a 48 px lattice: enough second differences, no fully valid patch
        path = tmp_path / "dynamic/000000/depth" / f"{f:06d}.png"
        raw = np.asarray(_Image.open(path)).copy()
        raw[24::48, 24::48] = 0
        _Image.fromarray(raw).save(path)
    with pytest.raises(ValueError, match="frames accepted"):
        cal.calibrate(tmp_path, _frames(tmp_path, "dynamic/000000"), z_range=(5.0, 6.0))


def test_config_round_trip_scales_k_and_correlation(tmp_path):
    fx = _cache(tmp_path)
    out = tmp_path / "noise.yaml"
    cal.main(["--cache", str(tmp_path), "--no-download", "--calibration-scenes", "dynamic/000000",
              "--pallet-scenes", "", "--stride", "1", "--z-range", "5.0,6.0", "--out", str(out)])
    model = NoiseModel.from_yaml(out, seed=3, target_fx=fx / 2, scale=1.5)
    assert model.correlation_px[0] == pytest.approx(4.0 / 2 * 1.5, rel=0.2) and model.seed == 3
    assert model.axial_k == pytest.approx(0.0009, rel=0.15) and model.correlated_k == pytest.approx(0.006, rel=0.15)
```

`tests/unit/perception/test_depth_noise.py` 끝에:

```python
GOOD = {"source": "s", "license": "CC-BY-4.0", "revision": "r", "files_sha256": "h", "k_iid": 0.001, "k_corr": 0.002,
        "correlation_px_source": [8.0, 6.0], "source_fx": 900.0, "dropout_fraction": 0.02}


@pytest.mark.parametrize("change", [{"source": ""}, {"k_corr": -1.0}, {"correlation_px_source": [8.0]},
                                    {"dropout_fraction": 1.0}, {"source_fx": 0.0}, {"revision": None}])
def test_noise_config_is_validated(tmp_path, change):
    import yaml

    path = tmp_path / "n.yaml"
    path.write_text(yaml.safe_dump({**GOOD, **change}))
    with pytest.raises(ValueError, match="noise config"):
        NoiseModel.from_yaml(path, seed=0, target_fx=450.0)
```

- [ ] **Step 2: 실패 확인** — `ModuleNotFoundError: tools.calibrate_depth_noise`, `AttributeError: ... from_yaml`
- [ ] **Step 3: 구현** — `depth_noise.py`:

```python
    @classmethod
    def from_yaml(cls, path, *, seed: int, target_fx: float, scale: float = 1.0) -> "NoiseModel":
        import math

        import yaml

        with open(path, encoding="utf-8") as f:
            obj = yaml.safe_load(f)

        def number(value, name, *, low=0.0, inclusive=True, high=math.inf):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
                    or value < low or (value == low and not inclusive) or value >= high:
                raise ValueError(f"noise config {name} out of range: {value!r}")
            return float(value)

        if not isinstance(obj, dict):
            raise ValueError("noise config must be a mapping")
        for name in ("source", "license", "revision", "files_sha256"):
            if not isinstance(obj.get(name), str) or not obj[name]:
                raise ValueError(f"noise config needs a non-empty {name}")
        px = obj.get("correlation_px_source")
        if not isinstance(px, list) or len(px) != 2:
            raise ValueError("noise config correlation_px_source must hold two widths")
        ratio = number(target_fx, "target_fx", inclusive=False) / number(obj.get("source_fx"), "source_fx",
                                                                         inclusive=False)
        scale = number(scale, "scale", inclusive=False)
        return cls(axial_k=number(obj.get("k_iid"), "k_iid") * scale,
                   correlated_k=number(obj.get("k_corr"), "k_corr") * scale,
                   correlation_px=tuple(number(p, "correlation_px_source", inclusive=False) * ratio * scale
                                        for p in px),
                   dropout_fraction=number(obj.get("dropout_fraction"), "dropout_fraction", high=1.0),
                   seed=int(seed))
```

`tools/calibrate_depth_noise.py`:

```python
#!/usr/bin/env python3
"""Measure D435i depth-noise structure on MR6D (CC-BY-4.0) and write the noise config.

The calibration uses MR6D's `dynamic` split, which the project does not evaluate on;
two val scenes with pallets give a reported (not adopted) pallet-surface comparison
and are listed as touched. Frames are chosen from a pinned listing, downloaded from
the pinned revision into a cache outside the repository, and treated as untrusted
data (Pillow for PNG, json for JSON).
"""

import argparse
import datetime
import hashlib
import json
import sys
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import yaml
from PIL import Image

from forklift_core.perception import depth_noise_fit as fit

DATASET = "anas-gouda/mr6d"
REVISION = "3c4ed8453312655499c3d3a3cf38b4929fae77ee"
BASE = f"https://huggingface.co/datasets/{DATASET}/resolve/{REVISION}/"
API = f"https://huggingface.co/api/datasets/{DATASET}/revision/{REVISION}"
CALIBRATION_SCENES = tuple(f"dynamic/{i:06d}" for i in range(40))
PALLET_SCENES = ("val/000000", "val/000005")
BANDS = ((1.5, 2.5), (2.5, 4.0), (4.0, 6.0))
PALLET_STEPS = (1, 2, 3, 4, 6, 8)
MIN_PATCHES = 20
METHOD = ("per-frame robust variogram of inverse-depth second differences on background pixels (margin_px from "
          "every visible object mask and invalid pixel); V(s) = 6A + B(6 - 8 rho(s) + 2 rho(2s)), "
          "rho = exp(-s^2/(4 g^2)); adopted values are medians over accepted frames; cross-checked against "
          "48 px patch plane residuals")
LIMITATIONS = [
    "MR6D depth is registered to the 1280x720 colour grid; its native depth mode and RealSense post-processing "
    "(spatial, temporal, hole-filling filters) are not recorded, and a spatial filter alone produces correlated error",
    "the robot's depth mode is not chosen and the target K (fx 465.74) is synthetic; the x1.66 sensitivity variant "
    "scales k and g for a 640x480 native mode and is an approximation, not a bound",
    "fitted on indoor background surfaces; the patch check ties the model to planar patches, and pallet surfaces "
    "are compared in pallet_surface (reported only)",
    "beyond the measured depth bands the k z^2 law is assumed",
    "a fresh correlated field is drawn per scene; any fixed-pattern part of the error is not reproduced, so gains "
    "from multi-frame fusion would be overstated",
    "dropout is modelled as independent pixels at the background rate; structured holes on pallets (grazing top "
    "boards, occlusion shadows) are reported in pallet_surface, not modelled",
    "edge mixing (flying pixels), quantisation and RGB registration keep the model's assumed defaults",
]


def _get(url: str) -> bytes:
    with urllib.request.urlopen(url, timeout=120) as response:
        return response.read()


def snapshot_listing(cache: Path) -> list:
    path = cache / "listing.json"
    if not path.exists():
        info = json.loads(_get(API))
        if info.get("sha") != REVISION:
            raise ValueError(f"dataset API returned revision {info.get('sha')}, expected {REVISION}")
        cache.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(sorted(s["rfilename"] for s in info["siblings"])))
    return json.loads(path.read_text())


def select_frames(listing, scenes, stride: int) -> list:
    frames = []
    for scene in scenes:
        depths = sorted(n for n in listing if n.startswith(f"{scene}/depth/") and n.endswith(".png"))
        for rel in depths[::stride]:
            stem = Path(rel).stem
            frames.append({"scene": scene, "frame": int(stem), "depth": rel,
                           "masks": sorted(n for n in listing if n.startswith(f"{scene}/mask_visib/{stem}_"))})
    return frames


def download(cache: Path, frames, listing) -> None:
    wanted = {"models_names.json"}
    for f in frames:
        wanted |= {f"{f['scene']}/scene_camera.json", f["depth"], *f["masks"]}
        if f"{f['scene']}/scene_gt.json" in listing:
            wanted.add(f"{f['scene']}/scene_gt.json")
    for rel in sorted(wanted):
        target = cache / rel
        if not target.exists():
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(_get(BASE + rel))


def _load(cache: Path, f):
    cams = json.loads((cache / f["scene"] / "scene_camera.json").read_text())
    cam = cams[str(f["frame"])]
    raw = np.asarray(Image.open(cache / f["depth"]))
    if raw.dtype != np.uint16 or raw.ndim != 2:
        raise ValueError(f"{f['depth']} is not a single-channel uint16 depth image")
    depth = np.where(raw > 0, raw * float(cam.get("depth_scale", 1.0)) / 1000.0, np.nan)
    masks = [np.asarray(Image.open(cache / m)) > 0 for m in f["masks"]]
    return depth, masks, np.asarray(cam["cam_K"], dtype=float).reshape(3, 3)


def _variances(depth, valid, z_range, steps=fit.STEPS):
    return {axis: {s: fit.robust_variance(fit.second_differences(depth, axis=axis, step=s, z_range=z_range,
                                                                 valid=valid)) for s in steps}
            for axis in (1, 0)}


def frame_structure(depth, masks, *, z_range, margin_px) -> dict:
    background = fit.background_mask(depth, masks, margin_px=margin_px)
    objects = np.any(masks, axis=0) if masks else np.zeros(depth.shape, bool)
    record = {"dropout": fit.dropout_fraction(depth, ~objects),
              "depth_m": [float(v) for v in np.nanpercentile(np.where(background, depth, np.nan), [5, 50, 95])]}
    try:
        var = _variances(depth, background, z_range)
    except ValueError as exc:
        return {**record, "reject": "samples", "detail": str(exc)}
    s = fit.fit_noise_structure(var[1], var[0])
    patches = fit.patch_plane_rms(depth, background, z_range=z_range)
    record.update(k_iid=s.k_iid, k_corr=s.k_corr, g=list(s.correlation_px), rel_residual_rms=s.rel_residual_rms,
                  patch_median=float(np.median(patches)) if patches.size else None, patches=int(patches.size))
    bands = {}
    for lo, hi in BANDS:
        try:
            b = _variances(depth, background, (lo, hi))
        except ValueError:
            continue
        bs = fit.fit_noise_structure(b[1], b[0])
        bands[f"{lo}-{hi}"] = {"k_iid": bs.k_iid, "k_corr": bs.k_corr, "g": list(bs.correlation_px)}
    record["bands"] = bands
    if patches.size < MIN_PATCHES:
        record["reject"] = "patches"
    elif s.at_grid_edge:
        record["reject"] = "grid_edge"
    elif s.rel_residual_rms > 0.15:
        record["reject"] = "residual"
    return record


def _dist(values):
    v = np.asarray(values, dtype=float)
    if not v.size:
        return {"n": 0}
    return {"n": int(v.size), "p10": float(np.percentile(v, 10)), "p50": float(np.median(v)),
            "p90": float(np.percentile(v, 90))}


def calibrate(cache: Path, frames, *, z_range=(1.5, 4.0), margin_px: int = 8, accept_fraction: float = 0.75) -> dict:
    records, files, grid = [], [], None
    for f in frames:
        depth, masks, k = _load(cache, f)
        records.append(frame_structure(depth, masks, z_range=z_range, margin_px=margin_px))
        files += [f["depth"], *f["masks"]]
        grid = {"width": int(depth.shape[1]), "height": int(depth.shape[0]), "fx": float(k[0, 0]),
                "fy": float(k[1, 1])}
    accepted = [r for r in records if "reject" not in r]
    if len(accepted) < accept_fraction * len(records):
        raise ValueError(f"only {len(accepted)}/{len(records)} frames accepted")
    k_iid = float(np.median([r["k_iid"] for r in accepted]))
    k_corr = float(np.median([r["k_corr"] for r in accepted]))
    g = [float(np.median([r["g"][j] for r in accepted])) for j in (0, 1)]
    model = fit.model_patch_rms(fit.NoiseStructure(k_iid, k_corr, tuple(g), 0.0, False),
                                size=(grid["height"], grid["width"]))
    ratios = [r["patch_median"] / model for r in accepted if r["patch_median"] is not None]
    if not ratios or not 0.67 <= float(np.median(ratios)) <= 1.5:
        raise ValueError(f"patch check failed: observed/model ratios {_dist(ratios)}")
    bands = {}
    for lo, hi in BANDS:
        key = f"{lo}-{hi}"
        rows = [r["bands"][key] for r in accepted if key in r["bands"]]
        bands[key] = {"k_iid": _dist([b["k_iid"] for b in rows]), "k_corr": _dist([b["k_corr"] for b in rows]),
                      "g_u": _dist([b["g"][0] for b in rows])}
    scenes = sorted({f["scene"] for f in frames})
    files = ["models_names.json", *[f"{s}/scene_camera.json" for s in scenes], *files]
    digest = hashlib.sha256()
    for rel in files:
        digest.update(rel.encode())
        digest.update(hashlib.sha256((cache / rel).read_bytes()).digest())
    return {
        "source": f"Hugging Face {DATASET} (MR6D, Intel RealSense D435i)", "license": "CC-BY-4.0",
        "revision": REVISION, "retrieved": datetime.date.today().isoformat(), "method": METHOD,
        "calibration_split": scenes, "files": files, "files_sha256": digest.hexdigest(),
        "source_grid": grid, "source_fx": grid["fx"], "z_range_m": [float(v) for v in z_range],
        "margin_px": margin_px, "steps": list(fit.STEPS),
        "k_iid": k_iid, "k_corr": k_corr, "correlation_px_source": g,
        "per_frame": {"accepted": len(accepted), "total": len(records),
                      "k_iid": _dist([r["k_iid"] for r in accepted]), "k_corr": _dist([r["k_corr"] for r in accepted]),
                      "g_u": _dist([r["g"][0] for r in accepted]), "g_v": _dist([r["g"][1] for r in accepted])},
        "rejected": {reason: sum(r.get("reject") == reason for r in records)
                     for reason in ("samples", "patches", "grid_edge", "residual")},
        "band_fits": bands,
        "patch_check": {"model_median": model, "ratio": _dist(ratios), "ratio_median": float(np.median(ratios))},
        "dropout_fraction": float(np.median([r["dropout"] for r in records])),
        "background_depth_m": _dist([r["depth_m"][1] for r in records]),
        "frames": len(records), "limitations": LIMITATIONS,
    }


def _pallet_ids(cache: Path) -> set:
    names = json.loads((cache / "models_names.json").read_text())
    return {int(k) for k, v in names.items() if "pallet" in str(v).lower()}


def pallet_surface_report(cache: Path, frames, *, z_range=(0.5, 2.5), margin_px: int = 8) -> dict:
    ids = _pallet_ids(cache)
    ratios_u, ratios_v = {s: [] for s in PALLET_STEPS}, {s: [] for s in PALLET_STEPS}
    drops, touched = [], set()
    for f in frames:
        gts = json.loads((cache / f["scene"] / "scene_gt.json").read_text())[str(f["frame"])]
        depth, masks, _ = _load(cache, f)
        pallets = [m for m, gt in zip(masks, gts) if int(gt["obj_id"]) in ids]
        if not pallets:
            continue
        touched.add(f["scene"])
        region = np.any(pallets, axis=0)
        drops.append(fit.dropout_fraction(depth, region))
        try:
            pal = _variances(depth, fit.erode(region, 4), z_range, PALLET_STEPS)
            bg = _variances(depth, fit.background_mask(depth, masks, margin_px=margin_px), z_range, PALLET_STEPS)
        except ValueError:
            continue
        for s in PALLET_STEPS:
            ratios_u[s].append(float(np.sqrt(pal[1][s] / bg[1][s])))
            ratios_v[s].append(float(np.sqrt(pal[0][s] / bg[0][s])))
    return {"scenes": sorted(touched), "pallet_dropout_fraction": float(np.median(drops)) if drops else None,
            "ratio_u": {s: _dist(v) for s, v in ratios_u.items()},
            "ratio_v": {s: _dist(v) for s, v in ratios_v.items()},
            "note": "pallet / background noise std at small steps; larger steps include gaps between boards"}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--cache", type=Path, default=Path.home() / ".cache/forklift/mr6d" / REVISION)
    p.add_argument("--calibration-scenes", default=",".join(CALIBRATION_SCENES))
    p.add_argument("--pallet-scenes", default=",".join(PALLET_SCENES))
    p.add_argument("--stride", type=int, default=50)
    p.add_argument("--pallet-stride", type=int, default=8)
    p.add_argument("--z-range", default="1.5,4.0")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--no-download", action="store_true")
    a = p.parse_args(argv)
    listing = json.loads((a.cache / "listing.json").read_text()) if a.no_download else snapshot_listing(a.cache)
    frames = select_frames(listing, [s for s in a.calibration_scenes.split(",") if s], a.stride)
    pallet_frames = select_frames(listing, [s for s in a.pallet_scenes.split(",") if s], a.pallet_stride)
    if not a.no_download:
        download(a.cache, frames + pallet_frames, listing)
    result = calibrate(a.cache, frames, z_range=tuple(float(v) for v in a.z_range.split(",")))
    pallet = pallet_surface_report(a.cache, pallet_frames) if pallet_frames else {"scenes": []}
    result["pallet_surface"] = pallet
    result["calibration_touched"] = pallet["scenes"]
    a.out.write_text(yaml.safe_dump(result, sort_keys=False, allow_unicode=True))
    print(json.dumps({k: result[k] for k in ("k_iid", "k_corr", "correlation_px_source", "dropout_fraction",
                                             "frames")} | {"accepted": result["per_frame"]["accepted"]}))


if __name__ == "__main__":
    main()
```

`deploy/hpc/recognition_stage1b_calibrate.tsv`(UBAI, `tools/hpc_sweep.sh` 형식):

```
calibrate	mkdir -p hpc_data && python tools/calibrate_depth_noise.py --out hpc_data/depth_noise_d435i.yaml && cat hpc_data/depth_noise_d435i.yaml
```

- [ ] **Step 4: 통과 확인(합성)** — Expected: `test_calibrate_depth_noise.py` 5 passed, `test_depth_noise.py` 의 설정 검사 6 passed. 시험은 로컬에서 한 파일씩(1 분 이내).
- [ ] **Step 5: 실측(UBAI)** — 로컬에서 `snapshot_listing` 만 먼저 돌려 `dynamic` 40 장면의 깊이 프레임 수를 확인한다(약 10,949 → `--stride 50` 이면 약 230 프레임, 깊이 PNG 약 1.5 MB → 약 350 MB). UBAI 에서 `tools/hpc_sweep.sh deploy/hpc/recognition_stage1b_calibrate.tsv cpu2 4` — 원격이 직접 내려받는다(캐시는 원격 `~/.cache/forklift/mr6d/<revision>`). 결과 YAML 을 `config/depth_noise_d435i.yaml` 로 가져온다. 읽을 것: 채택 프레임 비율, k₁·k₂·g 분포, 밴드별 분포(4–6 m 가 z² 법칙과 맞는지), `patch_check`, `pallet_surface` 의 비(작은 간격에서 1 에 가까운지). 채택이 안 되면(`ValueError`) 사유를 보고 방법을 고친다 — 문턱을 낮추지 않는다. 한 프레임 시제품 값(k₁ ≈ 0.0005·k₂ ≈ 0.009)과 크게 다르면 원인을 찾는다.
- [ ] **Step 6: 커밋** — `git add src/forklift_core/perception/depth_noise.py tools/calibrate_depth_noise.py deploy/hpc/recognition_stage1b_calibrate.tsv config/depth_noise_d435i.yaml tests/unit/test_calibrate_depth_noise.py tests/unit/perception/test_depth_noise.py` · `git commit -m "perception: D435i depth-noise config measured on MR6D's dynamic split (CC-BY-4.0, pinned revision) -- per-frame iid plus correlated fits, patch-plane cross-check, pallet-surface comparison, provenance and limits"`

### Task 4: 정답 K 전달, 생성기 정리, 잡음 입히기 도구, 평가기의 잡음 판 검사

**Files:**
- Modify: `tools/recognition_truth.py`(`scene_truths(..., intrinsics=None)`), `tools/generate_recognition_scenes.py`, `src/forklift_core/perception/evaluation_multi.py`, `tools/evaluate_recognition.py`
- Create: `tools/apply_recognition_noise.py`, `tests/unit/test_apply_recognition_noise.py`
- Test: `tests/integration/test_recognition_cli.py`(생성기 해시·섞임 거부), `tests/unit/perception/test_evaluation_multi.py`(official 조건), 1A 정답 시험 파일(K 전달)

**Interfaces:**
- `scene_truths(instances, setup, *, distractors=(), layers=None, intrinsics=None)` — `intrinsics` 를 두 `render_instances` 호출(장면·단독)에 모두 넘긴다. 기본값이면 지금과 같다.
- `tools/generate_recognition_scenes.py`: `noise_seed(recipe_seed) -> int`(지금 식), `load_site() -> dict`, `RecipeRecord`(frozen: `truths`, `rendered`, `meta`, `truth`, `instruction`), `recipe_record(recipe, site, intrinsics=None) -> RecipeRecord` — 지금 `main` 의 조립을 그대로 옮기고 `intrinsics` 를 `scene_truths` 에 넘긴다.
- `tools/apply_recognition_noise.py --src DIR --out DIR --noise-config FILE [--scale X]` — 장면마다 `NoiseModel.from_yaml(cfg, seed=noise_seed(meta.seed), target_fx=K.fx, scale=X)` → `apply_noise` → 같은 origin·truth·instruction 으로 기록. `meta.noise = {**model.to_json(), "config": 파일 이름, "config_sha256": 해시, "scale": X, "role": "official" if X == 1.0 else "sensitivity"}`. 이미 `meta.noise` 가 있으면 `ValueError("... already carries noise")`. 판 2 깊이는 mm 반올림이라 잡음은 반올림된 깊이 위에 얹힌다(리그 생성기 경로와 0.5 mm 이내 차이 — 기록에 적는다).
- `SceneVerdict` 에 `noise_role: str | None`, `noise_signature: str | None`(`f"{config_sha256}:{scale}"`; 잡음 없음이면 None; 1A 처럼 해시가 없는 잡음은 `"legacy"`)을 더한다. `summarize` 의 `official` = `render_source == "isaac"` 이고 모든 장면 `noise_role == "official"`. `evaluate_recognition.py` 는 한 집계에 `noise_signature` 가 둘 이상이면 `SystemExit("dataset must hold exactly one noise configuration ...")`.

- [ ] **Step 1: 생성기 기준 해시 기록** — 정리 **전** 코드로 아래 `_content_digest` 를 S2·S7·N 첫 장면(잡음 기본·없음 두 벌, 6 장면)에 대해 재고 값을 `GOLDEN` 에 적는다. 이 시험은 정리 전 코드에서도 통과해야 한다.

```python
# tests/integration/test_recognition_cli.py (위쪽 import 에 hashlib, numpy as np, pytest 를 더한다)
from forklift_core.perception.recognition_dataset import load_recognition_sample
from tools import generate_recognition_scenes as gen

GOLDEN = {}  # Step 1: {(noise, scene_id): digest}, measured with the pre-refactor generator


def _content_digest(scene_dir):
    s = load_recognition_sample(scene_dir)
    scene, h = s.input.scene, hashlib.sha256()
    for array in (np.nan_to_num(scene.depth_m, nan=-1.0), scene.rgb, s.instance_ids,
                  np.asarray(scene.base_from_optical.rotation), np.asarray(scene.base_from_optical.translation_m)):
        h.update(np.ascontiguousarray(array).tobytes())
    i = scene.intrinsics
    for obj in ([i.width, i.height, i.fx, i.fy, i.cx, i.cy, i.frame_id], s.origin, s.meta, s.truth, s.instruction):
        h.update(json.dumps(obj, sort_keys=True).encode())
    return h.hexdigest()


@pytest.mark.parametrize("noise", ["default", "none"])
def test_generator_output_is_unchanged_by_the_record_refactor(tmp_path, noise):
    gen.main(["--out", str(tmp_path), "--cases", "S2,S7,N", "--limit", "1", "--noise", noise])
    got = {(noise, d.name): _content_digest(d) for d in sorted(tmp_path.iterdir())}
    assert got == {k: v for k, v in GOLDEN.items() if k[0] == noise}
```

- [ ] **Step 2: 실패하는 시험 작성**

정답 K 전달(1A 정답 시험 파일 끝에):

```python
def test_truth_can_be_computed_at_another_camera_k():
    from dataclasses import replace

    from tools import generate_recognition_scenes as gen
    from tools import recognition_scenes as rs

    recipe = next(r for r in rs.catalogue() if r.case == "S3" and r.split == "dev")
    base = gen.recipe_record(recipe, gen.load_site())
    k = base.rendered.scene.intrinsics
    shifted = gen.recipe_record(recipe, gen.load_site(), intrinsics=replace(k, cx=k.cx - 0.5, cy=k.cy - 0.5))
    assert shifted.rendered.scene.intrinsics.cx == k.cx - 0.5
    assert [f.exposure for t in base.truths for f in t.faces] != [f.exposure for t in shifted.truths for f in t.faces]
```

잡음 입히기(`tests/unit/test_apply_recognition_noise.py`):

```python
import numpy as np
import pytest
import yaml

from forklift_core.perception.recognition_dataset import load_recognition_sample
from tools import apply_recognition_noise as app
from tools import generate_recognition_scenes as gen

CONFIG = {"source": "test", "license": "CC-BY-4.0", "revision": "r", "files_sha256": "x", "k_iid": 0.0006,
          "k_corr": 0.004, "correlation_px_source": [8.0, 6.0], "source_fx": 931.482312, "dropout_fraction": 0.02}


def _config(tmp_path):
    cfg = tmp_path / "noise.yaml"
    cfg.write_text(yaml.safe_dump(CONFIG))
    return cfg


def test_noise_is_applied_to_clean_scenes_and_truth_is_kept(tmp_path):
    gen.main(["--out", str(tmp_path / "clean"), "--cases", "S1", "--limit", "1", "--noise", "none"])
    app.main(["--src", str(tmp_path / "clean"), "--out", str(tmp_path / "noisy"), "--noise-config",
              str(_config(tmp_path))])
    for clean_dir in (tmp_path / "clean").iterdir():
        a, b = load_recognition_sample(clean_dir), load_recognition_sample(tmp_path / "noisy" / clean_dir.name)
        assert a.truth == b.truth and a.instruction == b.instruction and a.origin == b.origin
        noise = b.meta["noise"]
        assert noise["correlated_k"] == pytest.approx(0.004) and noise["role"] == "official" and noise["scale"] == 1.0
        assert noise["correlation_px"][0] == pytest.approx(4.0, rel=1e-3)  # 8 * 465.741156 / 931.482312
        diff = b.input.scene.depth_m - a.input.scene.depth_m
        assert diff[np.isfinite(diff)].std() > 0.001


def test_the_sensitivity_scale_marks_the_role_and_scales_the_width(tmp_path):
    gen.main(["--out", str(tmp_path / "clean"), "--cases", "S1", "--limit", "1", "--noise", "none"])
    app.main(["--src", str(tmp_path / "clean"), "--out", str(tmp_path / "k166"), "--noise-config",
              str(_config(tmp_path)), "--scale", "1.66"])
    noise = load_recognition_sample(next((tmp_path / "k166").iterdir())).meta["noise"]
    assert noise["role"] == "sensitivity" and noise["correlation_px"][0] == pytest.approx(4.0 * 1.66, rel=1e-3)


def test_noisy_scenes_are_not_noised_twice(tmp_path):
    gen.main(["--out", str(tmp_path / "noisy"), "--cases", "S1", "--limit", "1"])
    with pytest.raises(ValueError, match="already carries noise"):
        app.main(["--src", str(tmp_path / "noisy"), "--out", str(tmp_path / "again"), "--noise-config",
                  str(_config(tmp_path))])
```

평가기(`tests/unit/perception/test_evaluation_multi.py` 끝에):

```python
def _verdict(noise, render_source="isaac"):
    meta = {**META, "split": "dev", "render_source": render_source, "noise": noise}
    return judge_scene(meta, _truth([_inst(0, 0.0)]), NONE_IN, [_cand("a", 0.0)], TargetDecision("selected", "a", ()))


def test_official_needs_isaac_and_the_official_noise_role():
    official = {"config_sha256": "h", "scale": 1.0, "role": "official"}
    sensitivity = {"config_sha256": "h", "scale": 1.66, "role": "sensitivity"}
    assert summarize([_verdict(official)], split="dev", render_source="isaac")["official"]
    assert not summarize([_verdict(sensitivity)], split="dev", render_source="isaac")["official"]
    assert not summarize([_verdict({"k": 1})], split="dev", render_source="isaac")["official"]
    assert not summarize([_verdict(None)], split="dev", render_source="isaac")["official"]
    assert _verdict(official).noise_signature == "h:1.0" and _verdict({"k": 1}).noise_signature == "legacy"
```

평가 CLI 섞임 거부(`tests/integration/test_recognition_cli.py` 끝에):

```python
def test_mixed_noise_configurations_are_refused(tmp_path):
    from tools import apply_recognition_noise as app

    cfg = tmp_path / "noise.yaml"
    cfg.write_text(json.dumps({"source": "t", "license": "CC-BY-4.0", "revision": "r", "files_sha256": "x",
                               "k_iid": 0.0006, "k_corr": 0.0, "correlation_px_source": [1.0, 1.0],
                               "source_fx": 465.741156, "dropout_fraction": 0.0}))
    gen.main(["--out", str(tmp_path / "clean"), "--cases", "S1", "--limit", "2", "--noise", "none"])
    dirs = sorted((tmp_path / "clean").iterdir())
    app.apply_to_scene(dirs[0], tmp_path / "mixed", cfg, 1.0)
    app.apply_to_scene(dirs[1], tmp_path / "mixed", cfg, 1.66)
    for d in (tmp_path / "mixed").iterdir():  # both into dev so the split filter keeps them
        meta = json.loads((d / "scene.json").read_text())
        meta["split"] = "dev"
        (d / "scene.json").write_text(json.dumps(meta))
    proc = subprocess.run([sys.executable, "tools/evaluate_recognition.py", "--dataset", str(tmp_path / "mixed"),
                           "--split", "dev", "--detector", "oracle", "--output", str(tmp_path / "out")],
                          cwd=ROOT, capture_output=True, text=True)
    assert proc.returncode != 0 and "exactly one noise configuration" in proc.stderr
```

(JSON 은 YAML 의 부분집합이라 `from_yaml` 이 읽는다.)

- [ ] **Step 3: 실패 확인** — `TypeError`(`recipe_record`·`intrinsics` 없음), `ModuleNotFoundError: tools.apply_recognition_noise`, `AttributeError: 'SceneVerdict' object has no attribute 'noise_signature'`.
- [ ] **Step 4: 구현**

`tools/recognition_truth.py` 의 `scene_truths` 서명에 `intrinsics=None` 을 더하고 두 호출을 `instance_rig.render_instances(placed, distractors=distractors, intrinsics=intrinsics)`, `instance_rig.render_instances([placed[k]], floor=False, back_wall=False, intrinsics=intrinsics)` 로 바꾼다.

`tools/generate_recognition_scenes.py`(`from dataclasses import dataclass` 추가):

```python
@dataclass(frozen=True)
class RecipeRecord:
    truths: list
    rendered: object
    meta: dict
    truth: dict
    instruction: dict


def noise_seed(recipe_seed: int) -> int:
    return int(np.random.SeedSequence(recipe_seed).spawn(3)[2].generate_state(1)[0])


def load_site() -> dict:
    return load_site_defaults(Path(__file__).resolve().parents[1] / "config/site_load_defaults.yaml",
                              {k: rs.known_spec(k) for k in ("epal6", "t11_06")})


def recipe_record(recipe, site, intrinsics=None) -> RecipeRecord:
    instr = recipe.instruction
    holder = instr.target if instr.target is not None else 0

    def load_for(k, inst):
        if k == holder and instr.load is not None:
            return instr.load
        return site.get(recipe.shape_key) if inst.spec.known else None

    truths, rendered = scene_truths(
        [(i.spec, i.placement, i.damage, load_for(k, i)) for k, i in enumerate(recipe.instances)],
        ForkSetup(recipe.fork_spacing_m), distractors=recipe.distractors,
        layers=[i.layer for i in recipe.instances], intrinsics=intrinsics)
    meta = {"scene_id": recipe.scene_id, "case": recipe.case, "shape_key": recipe.shape_key, "split": recipe.split,
            "seed": recipe.seed, "rank": recipe.rank, "fork_spacing_m": recipe.fork_spacing_m,
            "touching": recipe.touching}
    return RecipeRecord(truths, rendered, meta,
                        {"instances": [t.to_json() for t in truths],
                         "instruction_truth": {"kind": instr.kind, "target": instr.target}},
                        {"present": instr.kind != "none", "x_m": instr.x_m, "y_m": instr.y_m,
                         "radius_m": instr.radius_m, "layer": instr.layer,
                         "load": None if instr.load is None else instr.load.to_json()})
```

`main` 의 반복은 `record = recipe_record(recipe, site)` 다음, 잡음이면 `NoiseModel(seed=noise_seed(recipe.seed))`, 기록은 `meta={**record.meta, "render_source": "rig", "noise": noise}`, `truth=record.truth`, `instruction=record.instruction`(`write_recognition_scene` 이 키를 정렬해 쓰므로 순서는 출력에 영향이 없다).

`tools/apply_recognition_noise.py`:

```python
#!/usr/bin/env python3
"""Apply a fitted depth-noise config to noise-free recognition scenes (stage 1B).

scale 1 is the official noise population; any other scale is a sensitivity variant.
"""

import argparse
import hashlib
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.recognition_dataset import load_recognition_sample, write_recognition_scene
from tools.generate_recognition_scenes import noise_seed


def apply_to_scene(src_dir: Path, out_root: Path, config: Path, scale: float) -> None:
    sample = load_recognition_sample(src_dir)
    if sample.meta.get("noise") is not None:
        raise ValueError(f"{src_dir.name} already carries noise")
    scene = sample.input.scene
    model = NoiseModel.from_yaml(config, seed=noise_seed(sample.meta["seed"]), target_fx=scene.intrinsics.fx,
                                 scale=scale)
    noisy, ids = apply_noise(scene, sample.instance_ids, model)
    meta = dict(sample.meta)
    meta["noise"] = {**model.to_json(), "config": config.name, "scale": scale,
                     "config_sha256": hashlib.sha256(config.read_bytes()).hexdigest(),
                     "role": "official" if scale == 1.0 else "sensitivity"}
    write_recognition_scene(out_root / src_dir.name, noisy, ids, origin=sample.origin, meta=meta,
                            truth=sample.truth, instruction=sample.instruction)


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--src", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--noise-config", type=Path, required=True)
    p.add_argument("--scale", type=float, default=1.0)
    a = p.parse_args(argv)
    for scene_json in sorted(a.src.glob("*/scene.json")):
        apply_to_scene(scene_json.parent, a.out, a.noise_config, a.scale)


if __name__ == "__main__":
    main()
```

`evaluation_multi.py` — `SceneVerdict` 의 `noisy` 다음에 `noise_role: str | None`, `noise_signature: str | None` 를 더하고, `judge_scene` 에서

```python
    noise = meta.get("noise")
    role = noise.get("role") if isinstance(noise, dict) else None
    signature = None if noise is None else (
        f"{noise['config_sha256']}:{noise['scale']}" if isinstance(noise, dict) and "config_sha256" in noise
        else "legacy")
```

를 계산해 반환의 `noisy` 다음에 `role, signature` 를 넘긴다. `summarize` 의 `official` 을 `render_source == "isaac" and bool(chosen) and all(v.noise_role == "official" for v in chosen)` 로 바꾼다. `evaluate_recognition.py` 는 장면마다 `signatures.add(verdict.noise_signature)` 를 모으고 출처 검사 다음에 `if len(signatures) != 1: raise SystemExit(f"dataset must hold exactly one noise configuration, found {sorted(map(str, signatures))}")`.

- [ ] **Step 5: 통과 확인** — Expected: 새 시험 모두 통과, 생성기 해시 시험 2 passed(정리 뒤에도 같은 값), 기존 평가기·정답·CLI 시험 통과(ws1 전체 묶음).
- [ ] **Step 6: 리그 참고(짧은 CPU, UBAI)** — `generate --cases S1 --noise none` → `apply_recognition_noise --noise-config config/depth_noise_d435i.yaml` → `evaluate_recognition --split dev --detector baseline`. 리그 결과는 판정 모집단이 아니다 — 기록의 참고 표로만 쓴다.
- [ ] **Step 7: 커밋** — `git add tools/recognition_truth.py tools/generate_recognition_scenes.py tools/apply_recognition_noise.py src/forklift_core/perception/evaluation_multi.py tools/evaluate_recognition.py tests/unit/test_apply_recognition_noise.py tests/integration/test_recognition_cli.py tests/unit/perception/test_evaluation_multi.py <1A 정답 시험 파일>` · `git commit -m "perception: truth at the rendered scene's K, shared recipe record, noise applied to clean scene dirs with an official/sensitivity role, and the evaluator refuses mixed noise configurations and gates official on the official role"`

**검토 A:** 작업 1–4 뒤 Codex — 초점: "상관 성분의 정상성·정규화·난수 분리, 변동도 모형·비음수 맞춤·격자 끝 판정, 프레임별 채택과 평면 칸 교차 확인, 배경·팔레트 마스크, MR6D 분할 역할과 리비전 고정, 설정 검사, 정답 K 전달, 생성기 무변경, 잡음 역할·섞임 거부". 수렴까지 반복.

### Task 5: Isaac 렌더 순수 코어 `recognition_render_core`

**Files:**
- Create: `sim/isaac/recognition_render_core.py`
- Test: `tests/unit/test_recognition_render_core.py`

**Interfaces:**
- Consumes: `tools.recognition_scenes.recipe_boxes`, `scene_rig.BACK_WALL_X_M`
- Produces:
  - `PrimSpec` (frozen): `path`, `centre_m`, `size_m`, `yaw_rad`, `base_label` (`floor`/`wall`/`inst_k`/`distractor`), `colour`
  - `prim_specs(recipe) -> list[PrimSpec]` — 바닥(윗면 z = 0: 중심 (0, 0, −0.05), 크기 (40, 40, 0.1)), 벽(앞면 x = `BACK_WALL_X_M`: 중심 (6.05, 0, 5), 크기 (0.1, 40, 10)), 인스턴스 상자(`/World/Scene/inst_k/box_i`), 방해물(`/World/Scene/distractor/box_i`). 카메라(0.75, 0, 0.27, 시야 약 69°×55°) 앞 광선은 모두 이 벽 안쪽에 닿는다(가로 약 ±3.6 m, 높이 약 −2.4…+3.0 m).
  - `scene_label(base, token) -> str` = `f"{base}_t{token}"`, `parse_label(label) -> tuple[str, int] | None`
  - `SemanticFrame` (frozen): `instance_ids: NDArray[int32] | None`, `ready: bool`, `status: str`, `stale_pixels: int`
  - `decode_semantic(payload, token, shape=(480, 640)) -> SemanticFrame` — 페이로드가 사전이 아니거나 `data`·`info.idToLabels` 가 없거나 비어 있거나 모양이 다르면 `ready=False`(`status` 에 사유) — 아직 준비 안 된 것으로 보고 다시 캡처한다. 비어 있지 않은데 정수형이 아니면 `ValueError`. 현재 토큰 라벨만 해석(`inst_k` → k, `distractor` → −2, 그 밖 → −1), 다른 토큰 화소 수를 `stale_pixels` 로 세고, `ready` 는 현재 토큰의 바닥 화소가 있고 `stale_pixels == 0` 일 때만 참. 한 의미 ID 에 라벨이 여럿이면 `ValueError`. 클래스 값은 문자열(쉼표 구분 가능)·목록 모두 받는다.
  - `clean_depth(z) -> NDArray[float64]` — 비유한·0 이하 → NaN, `(H, W, 1)` 이면 마지막 축 제거

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/test_recognition_render_core.py
import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest

from tools import recognition_scenes as rs
from tools import scene_rig

ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module")
def rc():
    name = "recognition_render_core"
    if name not in sys.modules:  # dataclasses need the module registered before execution
        spec = importlib.util.spec_from_file_location(name, ROOT / "sim/isaac/recognition_render_core.py")
        module = importlib.util.module_from_spec(spec)
        sys.modules[name] = module
        spec.loader.exec_module(module)
    return sys.modules[name]


def _payload(data, labels):
    return {"data": np.asarray(data, dtype=np.uint32), "info": {"idToLabels": labels}}


def test_prim_specs_reproduce_the_recipe_boxes_and_the_rig_floor_and_wall(rc):
    recipe = next(r for r in rs.catalogue() if r.case == "S6")
    specs = rc.prim_specs(recipe)
    boxes = [b for k in range(len(recipe.instances)) for b in rs.recipe_boxes(recipe, k)]
    inst = [s for s in specs if s.base_label.startswith("inst_")]
    assert len(inst) == len(boxes)
    for spec, box in zip(inst, boxes):
        assert spec.centre_m == pytest.approx(tuple(box.centre_m)) and spec.size_m == pytest.approx(tuple(box.size_m))
        assert spec.yaw_rad == pytest.approx(box.yaw_rad)
    assert len([s for s in specs if s.base_label == "distractor"]) == len(recipe.distractors)
    floor = next(s for s in specs if s.base_label == "floor")
    wall = next(s for s in specs if s.base_label == "wall")
    assert floor.centre_m[2] + floor.size_m[2] / 2 == pytest.approx(0.0)
    assert wall.centre_m[0] - wall.size_m[0] / 2 == pytest.approx(scene_rig.BACK_WALL_X_M)


def test_labels_carry_the_scene_token(rc):
    assert rc.scene_label("inst_3", 12) == "inst_3_t12"
    assert rc.parse_label("inst_3_t12") == ("inst_3", 12)
    assert rc.parse_label("BACKGROUND") is None and rc.parse_label("UNLABELLED") is None


def test_semantics_decode_instances_and_readiness(rc):
    labels = {"0": {"class": "BACKGROUND"}, "1": {"class": "inst_0_t5"}, "2": {"class": ["inst_3_t5"]},
              "3": {"class": "distractor_t5"}, "4": {"class": "floor_t5"}}
    frame = rc.decode_semantic(_payload([[0, 1, 2], [3, 4, 4]], labels), 5, shape=(2, 3))
    assert frame.instance_ids.tolist() == [[-1, 0, 3], [-2, -1, -1]] and frame.ready


def test_a_frame_from_the_previous_scene_is_not_ready(rc):
    frame = rc.decode_semantic(_payload([[1, 4]], {"1": {"class": "inst_0_t4"}, "4": {"class": "floor_t5"}}), 5,
                               shape=(1, 2))
    assert not frame.ready and frame.stale_pixels == 1
    assert not rc.decode_semantic(_payload([[1]], {"1": {"class": "inst_0_t5"}}), 5, shape=(1, 1)).ready


@pytest.mark.parametrize("payload", [None, {}, {"data": None, "info": {}}, {"data": np.zeros((0,), np.uint32)},
                                     {"data": np.zeros((2, 2), np.uint32), "info": {"idToLabels": {}}}])
def test_an_unready_annotator_is_retried_not_failed(rc, payload):
    frame = rc.decode_semantic(payload, 1, shape=(1, 1))
    assert not frame.ready and frame.status != "ready"


def test_bad_nonempty_semantics_are_refused(rc):
    with pytest.raises(ValueError, match="label"):
        rc.decode_semantic(_payload([[1]], {"1": {"class": "inst_0_t1,inst_1_t1"}}), 1, shape=(1, 1))
    with pytest.raises(ValueError, match="integer"):
        rc.decode_semantic({"data": np.zeros((1, 1), np.float32), "info": {"idToLabels": {"0": {"class": "x"}}}}, 1,
                           shape=(1, 1))


def test_clean_depth(rc):
    out = rc.clean_depth(np.array([np.inf, 0.0, -1.0, 2.0]).reshape(2, 2, 1))
    assert out.shape == (2, 2) and np.isnan(out.ravel()[:3]).all() and out[1, 1] == 2.0
```

- [ ] **Step 2: 실패 확인** — Expected: `FileNotFoundError`(모듈 파일 없음)
- [ ] **Step 3: 구현**

```python
# sim/isaac/recognition_render_core.py
"""Pure helpers for rendering recognition recipes in Isaac (no Isaac imports).

The prims reproduce tools.instance_rig's world exactly -- the recipe's boxes, a
floor whose top is z = 0 and a back wall whose front face is x = BACK_WALL_X_M --
so the CPU-rig truth (stage 1A) applies to the Isaac image. Every label carries a
per-scene token: a frame that still shows the previous scene's labels is stale,
and an annotator that has not produced a buffer yet is "not ready", not a failure.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from tools import recognition_scenes as rs  # noqa: E402
from tools import scene_rig  # noqa: E402

WOOD, GREY, CONCRETE, PAINT = (0.62, 0.45, 0.28), (0.5, 0.5, 0.52), (0.55, 0.55, 0.55), (0.85, 0.85, 0.8)


@dataclass(frozen=True)
class PrimSpec:
    path: str
    centre_m: tuple
    size_m: tuple
    yaw_rad: float
    base_label: str
    colour: tuple


@dataclass(frozen=True)
class SemanticFrame:
    instance_ids: np.ndarray | None
    ready: bool
    status: str
    stale_pixels: int


def prim_specs(recipe) -> list[PrimSpec]:
    specs = [
        PrimSpec("/World/Scene/floor", (0.0, 0.0, -0.05), (40.0, 40.0, 0.1), 0.0, "floor", CONCRETE),
        PrimSpec("/World/Scene/wall", (scene_rig.BACK_WALL_X_M + 0.05, 0.0, 5.0), (0.1, 40.0, 10.0), 0.0,
                 "wall", PAINT),
    ]
    for k in range(len(recipe.instances)):
        for i, box in enumerate(rs.recipe_boxes(recipe, k)):
            specs.append(PrimSpec(f"/World/Scene/inst_{k}/box_{i}", tuple(box.centre_m), tuple(box.size_m),
                                  float(box.yaw_rad), f"inst_{k}", WOOD))
    for i, box in enumerate(recipe.distractors):
        specs.append(PrimSpec(f"/World/Scene/distractor/box_{i}", tuple(box.centre_m), tuple(box.size_m),
                              float(box.yaw_rad), "distractor", GREY))
    return specs


def scene_label(base: str, token: int) -> str:
    return f"{base}_t{token}"


def parse_label(label: str):
    base, sep, token = str(label).rpartition("_t")
    if not sep or not base or not token.isdigit():
        return None
    return base, int(token)


def _class_names(value) -> list[str]:
    classes = value.get("class", []) if isinstance(value, dict) else []
    if isinstance(classes, str):
        classes = [c for c in classes.split(",") if c]
    return list(classes)


def _not_ready(status: str) -> SemanticFrame:
    return SemanticFrame(None, False, status, 0)


def decode_semantic(payload, token: int, shape=(480, 640)) -> SemanticFrame:
    if not isinstance(payload, dict) or payload.get("data") is None:
        return _not_ready("missing_buffer")
    array = np.asarray(payload["data"])
    if not array.size:
        return _not_ready("empty_buffer")
    info = payload.get("info")
    labels = info.get("idToLabels") if isinstance(info, dict) else None
    if not isinstance(labels, dict) or not labels:
        return _not_ready("missing_label_map")
    if array.ndim == 3 and array.shape[-1] == 1:
        array = array[..., 0]
    if array.dtype.kind not in "ui":
        raise ValueError("semantic segmentation must be an uncolorized integer grid")
    if array.shape != tuple(shape):
        return _not_ready("shape_mismatch")
    ids = np.full(array.shape, -1, dtype=np.int32)
    stale, floor = 0, 0
    for key, value in labels.items():
        names = _class_names(value)
        if len(names) > 1:
            raise ValueError(f"semantic id {key} carries several labels: {names}")
        parsed = parse_label(names[0]) if names else None
        if parsed is None:
            continue
        base, tok = parsed
        hit = array == int(key)
        if tok != token:
            stale += int(hit.sum())
        elif base == "distractor":
            ids[hit] = -2
        elif base.startswith("inst_"):
            ids[hit] = int(base.split("_", 1)[1])
        elif base == "floor":
            floor += int(hit.sum())
    ready = floor > 0 and stale == 0
    return SemanticFrame(ids, ready, "ready" if ready else ("stale" if stale else "floor_missing"), stale)


def clean_depth(z) -> np.ndarray:
    depth = np.asarray(z, dtype=float)
    if depth.ndim == 3 and depth.shape[-1] == 1:
        depth = depth[..., 0]
    depth = depth.copy()
    depth[~np.isfinite(depth) | (depth <= 0)] = np.nan
    return depth
```

- [ ] **Step 4: 통과 확인** — Expected: 12 passed(매개변수화 포함)
- [ ] **Step 5: 커밋** — `git add sim/isaac/recognition_render_core.py tests/unit/test_recognition_render_core.py` · `git commit -m "sim: pure helpers for Isaac recognition renders -- recipe to labelled box prims reproducing the rig's floor and wall, per-scene label tokens, unready annotators retried rather than failed"`

### Task 6: Isaac 렌더 스크립트, 리그 비교 도구, 잡 스크립트

**Files:**
- Create: `sim/isaac/render_recognition_scenes.py`, `tools/compare_isaac_rig.py`, `deploy/slurm/recognition_isaac_render.sbatch`
- Test: `tests/unit/test_compare_isaac_rig.py`

**Interfaces:**
- `render_recognition_scenes.py --out DIR [--settings config/isaac_perception_camera.yaml] [--cases ...] [--split dev|eval|all] [--limit N] [--shard i/n] [--render-mode RaytracedLighting] [--max-captures 8]` — 잡음 없는 판 2 장면만 쓴다. 시작할 때 SDK K 를 설정과 대조한다(fx·fy = `focal_px` ± 0.01, 원 SDK 주점 = (320, 240) ± 0.01; 어긋나면 치명 오류). 장면마다: 토큰 = 이 실행의 순번, `/World/Scene` 를 지우고 `prim_specs` 로 `UsdGeom.Cube`(기본 크기 2 → 반크기 scale, z 축 yaw, `displayColor`)에 `add_labels(prim, labels=[scene_label(base_label, token)], instance_name="class")`; 최대 `--max-captures` 번 `capture_static` 하며 `decode_semantic(frame["seg"], token).ready` 일 때 받는다. K 는 `IsaacIntrinsics(...).integer_index`(주점 −0.5 px), `base_from_optical` 은 카메라 world 자세(ros 축). 정답·메타·지시는 `recipe_record(recipe, site, intrinsics=K)` — **정답을 Isaac 장면의 K 로 계산**한다. 메타 `render_source="isaac"`, `truth_render="rig"`, `noise=None`, `render_token`. `status.json` 을 `app.close()` 전에 쓰고 실패가 있으면 종료 코드 1.
- `tools/compare_isaac_rig.py --isaac DIR --out FILE [--settings config/isaac_perception_camera.yaml]` — 장면마다 같은 `scene_id` 의 조리법으로 `recipe_record(recipe, site, intrinsics=장면 K)` 의 리그 렌더와 대조한다. 함수 `compare_scene(sample, recipe, settings) -> dict`, `load_settings(path)`, `DEFAULT_SETTINGS`:
  - **K 규약:** 장면 K 가 설정에서 기대하는 정수 첨자 K(fx = fy = `focal_px`, cx = 319.5, cy = 239.5)와 0.01 px 안.
  - **자세:** `tf.json` 과 리그 카메라 변환의 차 ≤ 1 mm·0.01°.
  - **대상마다**(인스턴스 k, 방해물 묶음 −2; 리그 화소 ≥ 200): IoU ≥ 0.97, 리그 대상 화소 중 Isaac 깊이가 유효한 비율 ≥ 0.99, 같은 ID 화소의 깊이 차 중앙값 ≤ 1 mm·p95 ≤ 3 mm. 200 화소 미만 대상은 `small` 로 보고만.
  - **배경:** 리그가 배경(−1)으로 칠한 유효 화소 중 Isaac 도 배경·유효인 비율 ≥ 0.99, 그 화소를 base 좌표로 되투영해 리그 점이 바닥(z ≈ 0)이면 |z|, 벽이면 |x − 6| 의 p95 ≤ 2 mm(깊이 기울기가 큰 바닥에서도 면까지의 거리는 작게 남는다).
  - 전체 `pass` 는 모든 장면 합격, 종료 코드 0/1.

- [ ] **Step 1: 실패하는 시험 작성**

```python
# tests/unit/test_compare_isaac_rig.py
from dataclasses import replace

import numpy as np
import pytest

from forklift_core.perception.recognition_dataset import load_recognition_sample, write_recognition_scene
from tools import compare_isaac_rig as cmp
from tools import generate_recognition_scenes as gen
from tools import recognition_scenes as rs

SETTINGS = cmp.load_settings(cmp.DEFAULT_SETTINGS)


def _recipe():
    return next(r for r in rs.catalogue() if r.case == "S2" and r.split == "dev")


def _write(tmp_path, record, depth, ids, name):
    write_recognition_scene(tmp_path / name, replace(record.rendered.scene, depth_m=depth), ids,
                            origin="isaac_render_pose",
                            meta={**record.meta, "render_source": "isaac", "truth_render": "rig", "noise": None},
                            truth=record.truth, instruction=record.instruction)
    return load_recognition_sample(tmp_path / name)


def _as_isaac(tmp_path, *, edit=None):
    recipe = _recipe()
    rig_k = gen.recipe_record(recipe, gen.load_site()).rendered.scene.intrinsics
    record = gen.recipe_record(recipe, gen.load_site(), intrinsics=replace(rig_k, cx=319.5, cy=239.5))
    depth, ids = record.rendered.scene.depth_m.copy(), record.rendered.instance_ids.copy()
    if edit:
        depth, ids = edit(depth, ids)
    return cmp.compare_scene(_write(tmp_path, record, depth, ids, recipe.scene_id), recipe, SETTINGS)


def test_a_rig_render_at_the_isaac_k_passes(tmp_path):
    report = _as_isaac(tmp_path)
    assert report["pass"] and min(t["iou"] for t in report["targets"].values()) == 1.0


@pytest.mark.parametrize("edit", [
    lambda d, i: (np.where(i == 0, d + 0.1, d), i),        # whole pallet 100 mm too far
    lambda d, i: (np.where(i == 0, np.nan, d), i),         # whole pallet without depth
    lambda d, i: (d, np.roll(i, 3, axis=1)),               # mask shifted 3 px
    lambda d, i: (np.where(i == -1, d + 0.005, d), i),     # floor and wall 5 mm off
])
def test_wrong_renders_fail(tmp_path, edit):
    assert not _as_isaac(tmp_path, edit=edit)["pass"]


def test_the_rig_k_fails_the_k_contract(tmp_path):
    recipe = _recipe()
    record = gen.recipe_record(recipe, gen.load_site())
    sample = _write(tmp_path, record, record.rendered.scene.depth_m, record.rendered.instance_ids, "x")
    report = cmp.compare_scene(sample, recipe, SETTINGS)
    assert not report["k_contract"] and not report["pass"]
```

- [ ] **Step 2: 실패 확인** — `ModuleNotFoundError: tools.compare_isaac_rig`
- [ ] **Step 3: 구현** — `tools/compare_isaac_rig.py`:

```python
#!/usr/bin/env python3
"""Check noise-free Isaac renders against the CPU rig rendered at the scene's own K.

Per target (instance or the distractor group): IoU, valid-depth fraction and depth
error. Background: distance of Isaac's back-projected points to the floor (z = 0)
or the back wall (x = BACK_WALL_X_M). Plus the K convention and the camera pose.
"""

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import yaml

from forklift_core.perception.recognition_dataset import load_recognition_sample
from tools import instance_rig, scene_rig
from tools import recognition_scenes as rs
from tools.generate_recognition_scenes import load_site, recipe_record

ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SETTINGS = ROOT / "config/isaac_perception_camera.yaml"
IOU_MIN, VALID_MIN, DEPTH_MEDIAN_MAX_M, DEPTH_P95_MAX_M = 0.97, 0.99, 0.001, 0.003
BACKGROUND_P95_MAX_M, POSE_MM_MAX, POSE_DEG_MAX, K_TOL_PX, MIN_PIXELS = 0.002, 1.0, 0.01, 0.01, 200


def load_settings(path) -> dict:
    return yaml.safe_load(Path(path).read_text(encoding="utf-8"))


def _k_contract(k, settings) -> bool:
    width, height = settings["resolution"]
    f = float(settings["focal_px"])
    if (k.width, k.height) != (width, height):
        return False
    want = (f, f, width / 2 - 0.5, height / 2 - 0.5)
    return all(abs(a - b) <= K_TOL_PX for a, b in zip((k.fx, k.fy, k.cx, k.cy), want))


def _target(depth_a, depth_b, a, b) -> dict:
    union = np.logical_or(a, b).sum()
    valid = float(np.isfinite(depth_a[b]).mean()) if b.any() else 1.0
    both = a & b & np.isfinite(depth_a) & np.isfinite(depth_b)
    diff = np.abs(depth_a[both] - depth_b[both])
    report = {"pixels": int(b.sum()), "iou": float(np.logical_and(a, b).sum() / union) if union else 1.0,
              "valid": valid, "depth_median_m": float(np.median(diff)) if diff.size else None,
              "depth_p95_m": float(np.percentile(diff, 95)) if diff.size else None}
    report["pass"] = bool(report["iou"] >= IOU_MIN and valid >= VALID_MIN and diff.size
                          and report["depth_median_m"] <= DEPTH_MEDIAN_MAX_M
                          and report["depth_p95_m"] <= DEPTH_P95_MAX_M)
    return report


def compare_scene(sample, recipe, settings) -> dict:
    scene = sample.input.scene
    rig = recipe_record(recipe, load_site(), intrinsics=scene.intrinsics).rendered
    ids_a, ids_b = sample.instance_ids, rig.instance_ids
    da, db = scene.depth_m, rig.scene.depth_m
    targets, small = {}, {}
    for k in sorted(set(np.unique(ids_b).tolist()) - {-1}):
        report = _target(da, db, ids_a == k, ids_b == k)
        (targets if report["pixels"] >= MIN_PIXELS else small)[str(k)] = report
    bg = (ids_b == -1) & np.isfinite(db)
    use = bg & (ids_a == -1) & np.isfinite(da)
    bg_valid = float(use.sum() / bg.sum()) if bg.any() else 1.0
    v, u = np.nonzero(use)
    k = scene.intrinsics
    z = da[use]
    optical = np.stack(((u - k.cx) / k.fx * z, (v - k.cy) / k.fy * z, z), axis=1)
    tf = scene.base_from_optical
    points = optical @ np.asarray(tf.rotation).T + np.asarray(tf.translation_m)
    on_floor = rig.points_base[use][:, 2] < 1e-6
    dist = np.where(on_floor, np.abs(points[:, 2]), np.abs(points[:, 0] - scene_rig.BACK_WALL_X_M))
    background = {"pixels": int(use.sum()), "valid": bg_valid,
                  "surface_p95_m": float(np.percentile(dist, 95)) if dist.size else None}
    background["pass"] = bool(dist.size and bg_valid >= VALID_MIN
                              and background["surface_p95_m"] <= BACKGROUND_P95_MAX_M)
    want = instance_rig.PERCEPTION_CAMERA.base_from_optical()
    dt = float(np.linalg.norm(np.asarray(tf.translation_m) - np.asarray(want.translation_m)) * 1000)
    cos = (np.trace(np.asarray(tf.rotation).T @ np.asarray(want.rotation)) - 1) / 2
    dr = float(np.degrees(np.arccos(np.clip(cos, -1.0, 1.0))))
    report = {"targets": targets, "small": small, "background": background, "k_contract": _k_contract(k, settings),
              "pose_translation_mm": dt, "pose_rotation_deg": dr}
    report["pass"] = bool(all(t["pass"] for t in targets.values()) and background["pass"] and report["k_contract"]
                          and dt <= POSE_MM_MAX and dr <= POSE_DEG_MAX)
    return report


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--isaac", type=Path, required=True)
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--settings", type=Path, default=DEFAULT_SETTINGS)
    a = p.parse_args(argv)
    settings = load_settings(a.settings)
    recipes = {r.scene_id: r for r in rs.catalogue()}
    reports = {d.parent.name: compare_scene(load_recognition_sample(d.parent), recipes[d.parent.name], settings)
               for d in sorted(a.isaac.glob("*/scene.json"))}
    targets = [t for r in reports.values() for t in r["targets"].values()]
    summary = {
        "scenes": len(reports), "targets": len(targets),
        "iou_min": min((t["iou"] for t in targets), default=None),
        "valid_min": min((t["valid"] for t in targets), default=None),
        "depth_p95_max_m": max((t["depth_p95_m"] or 0.0 for t in targets), default=None),
        "background_p95_max_m": max((r["background"]["surface_p95_m"] or 0.0 for r in reports.values()),
                                    default=None),
        "failed": sorted(name for name, r in reports.items() if not r["pass"]),
    }
    summary["pass"] = bool(reports) and not summary["failed"]
    a.out.write_text(json.dumps({"summary": summary, "scenes": reports}, indent=1))
    print(json.dumps(summary))
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
```

`sim/isaac/render_recognition_scenes.py`(Isaac — ws1 에서만 실행):

```python
#!/usr/bin/env python3
"""Render recognition recipes in Isaac Sim 5.1 into noise-free schema-v2 scenes.

origin = isaac_render_pose; truth comes from the CPU rig at this render's K through
the generator's recipe_record. Noise is applied afterwards by
tools/apply_recognition_noise.py. Run inside a ws1 Slurm GPU job only.
"""

import argparse
import json
import math
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "sim/isaac"))

import numpy as np


def arguments(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--settings", type=Path, default=ROOT / "config/isaac_perception_camera.yaml")
    p.add_argument("--cases", default="S1,S2,S3,S4,S5,S6,S7,N")
    p.add_argument("--split", choices=("dev", "eval", "all"), default="all")
    p.add_argument("--limit", type=int, default=None, help="first N selected recipes per case and shape")
    p.add_argument("--shard", default="0/1")
    p.add_argument("--render-mode", default="RaytracedLighting")
    p.add_argument("--max-captures", type=int, default=8)
    return p.parse_args(argv)


def selected(args):
    from tools import recognition_scenes as rs

    index, count = (int(v) for v in args.shard.split("/"))
    cases, seen, out = set(args.cases.split(",")), {}, []
    for recipe in rs.catalogue():
        if recipe.case not in cases or (args.split != "all" and recipe.split != args.split):
            continue
        key = (recipe.case, recipe.shape_key)
        seen[key] = seen.get(key, 0) + 1
        if args.limit is None or seen[key] <= args.limit:
            out.append(recipe)
    return out[index::count]


def build(stage, specs, token, rc):
    from isaacsim.core.utils.semantics import add_labels
    from pxr import Gf, UsdGeom

    if stage.GetPrimAtPath("/World/Scene"):
        stage.RemovePrim("/World/Scene")
    UsdGeom.Xform.Define(stage, "/World/Scene")
    for spec in specs:
        parent = spec.path.rsplit("/", 1)[0]
        if not stage.GetPrimAtPath(parent):
            UsdGeom.Xform.Define(stage, parent)
        cube = UsdGeom.Cube.Define(stage, spec.path)
        xf = UsdGeom.Xformable(cube)
        xf.ClearXformOpOrder()
        xf.AddTranslateOp().Set(Gf.Vec3d(*spec.centre_m))
        xf.AddRotateZOp().Set(math.degrees(spec.yaw_rad))
        xf.AddScaleOp().Set(Gf.Vec3d(*(s / 2 for s in spec.size_m)))
        cube.CreateDisplayColorAttr([Gf.Vec3f(*spec.colour)])
        add_labels(cube.GetPrim(), labels=[rc.scene_label(spec.base_label, token)], instance_name="class")


def main(argv=None):
    args = arguments(argv)
    args.out.mkdir(parents=True, exist_ok=False)
    status = {"rendered": [], "failed": [], "args": {k: str(v) for k, v in vars(args).items()}}
    app = None
    try:
        from isaacsim import SimulationApp

        app = SimulationApp({"headless": True, "renderer": args.render_mode, "multi_gpu": False})
        import omni.replicator.core as rep
        import omni.usd
        from isaacsim.core.api import World
        from pxr import UsdGeom, UsdLux

        import camera_calibration as cc
        import perception_camera as pc
        import recognition_render_core as rc
        from forklift_core.geometry import RigidTransform, rotation_matrix_from_quaternion_xyzw
        from forklift_core.perception.recognition_dataset import write_recognition_scene
        from forklift_core.perception.scene_dataset import SceneInput
        from forklift_core.sensors.rgbd import PinholeIntrinsics
        from tools.generate_recognition_scenes import load_site, recipe_record

        settings = pc.load_camera_settings(args.settings)
        omni.usd.get_context().new_stage()
        app.update()
        stage = omni.usd.get_context().get_stage()
        world = World(stage_units_in_meters=1.0)
        UsdGeom.Xform.Define(stage, "/World/Base")
        UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1000.0)
        camera = pc.create_perception_camera(args.settings, prim_path="/World/Base/PerceptionCamera")
        world.reset()
        camera.initialize()
        sdk = np.asarray(camera.get_intrinsics_matrix(), dtype=float)
        f = float(settings["focal_px"])
        if max(abs(sdk[0, 0] - f), abs(sdk[1, 1] - f), abs(sdk[0, 2] - 320.0), abs(sdk[1, 2] - 240.0)) > 0.01:
            raise RuntimeError(f"SDK K {sdk.tolist()} does not match the camera settings")
        intrinsics = pc._isaac_intrinsics_type()(
            PinholeIntrinsics(640, 480, sdk[0, 0], sdk[1, 1], sdk[0, 2], sdk[1, 2], "camera_optical_frame")
        ).integer_index
        status["sdk_k"] = sdk.tolist()
        annotators = cc.attach_annotators(rep, camera)
        for _ in range(10):
            world.step(render=True)
        world.pause()
        site = load_site()
        for token, recipe in enumerate(selected(args)):
            try:
                build(stage, rc.prim_specs(recipe), token, rc)
                frame, semantic, attempt = None, None, 0
                for attempt in range(1, args.max_captures + 1):
                    frame = cc.capture_static(rep, annotators)
                    semantic = rc.decode_semantic(frame.get("seg"), token)
                    if semantic.ready:
                        break
                if not semantic.ready:
                    raise RuntimeError(f"semantics not ready after {args.max_captures} captures: {semantic.status} "
                                       f"(stale pixels {semantic.stale_pixels})")
                pos, q_wxyz = camera.get_world_pose(camera_axes="ros")
                scene = SceneInput(
                    rgb=np.asarray(frame["rgb"])[:, :, :3].astype(np.uint8), depth_m=rc.clean_depth(frame["z"]),
                    intrinsics=intrinsics,
                    base_from_optical=RigidTransform(
                        "camera_optical_frame", "base_link",
                        rotation_matrix_from_quaternion_xyzw(np.roll(np.asarray(q_wxyz, dtype=float), -1)),
                        np.asarray(pos, dtype=float)),
                    stamp_ns=2_000_000_000, clock_domain="synthetic", source_provenance="synthetic")
                record = recipe_record(recipe, site, intrinsics=intrinsics)
                write_recognition_scene(
                    args.out / recipe.scene_id, scene, semantic.instance_ids, origin="isaac_render_pose",
                    meta={**record.meta, "render_source": "isaac", "truth_render": "rig", "noise": None,
                          "render_token": token},
                    truth=record.truth, instruction=record.instruction)
                status["rendered"].append({"scene_id": recipe.scene_id, "captures": attempt})
            except Exception as exc:  # record the scene and keep going
                status["failed"].append({"scene_id": recipe.scene_id, "error": repr(exc),
                                         "trace": traceback.format_exc()})
        cc.detach_annotators(annotators)
    except Exception as exc:
        status["fatal"] = {"error": repr(exc), "trace": traceback.format_exc()}
    finally:
        # Persist before close: the observed Isaac close path may exit Python.
        (args.out / "status.json").write_text(json.dumps(status, indent=1))
        if app is not None:
            app.close()
    return 1 if status["failed"] or "fatal" in status else 0


if __name__ == "__main__":
    sys.exit(main())
```

`deploy/slurm/recognition_isaac_render.sbatch`:

```bash
#!/bin/bash
#SBATCH --job-name=recog_isaac
#SBATCH --partition=compute
#SBATCH --gres=shard:1
#SBATCH --qos=admin
#SBATCH --cpus-per-task=4
#SBATCH --mem=6G
#SBATCH --time=01:00:00
#SBATCH --output=%x_%j.log
# Usage: sbatch --chdir=<run dir> --export=ALL,SRC=<source snapshot>,OUT=<out dir>,ARGS="<render args>" \
#        recognition_isaac_render.sbatch   (--mem and --time are overridden on the command line from measured runs)
set -euo pipefail
cd "${SRC:?}"
export OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH="$SRC/src:$SRC" PYTHONDONTWRITEBYTECODE=1
nvidia-smi --query-gpu=name,driver_version,memory.used --format=csv > "${OUT:?}.gpu_start.csv"
/home/projects/forklift/.venv/bin/python sim/isaac/render_recognition_scenes.py --out "$OUT" ${ARGS:-}
```

- [ ] **Step 4: 통과 확인(CPU)** — `tests/unit/test_compare_isaac_rig.py` 6 passed. Isaac 스크립트는 작업 7 에서 확인한다.
- [ ] **Step 5: 커밋** — `git add sim/isaac/render_recognition_scenes.py tools/compare_isaac_rig.py deploy/slurm/recognition_isaac_render.sbatch tests/unit/test_compare_isaac_rig.py` · `git commit -m "sim: Isaac renderer for recognition recipes (token-labelled box prims, K checked against the settings, truth at the render's K, noise-free schema v2) and a per-target / background comparison with the rig"`

### Task 7: ws1 연기 시험과 리그 비교

- [ ] **Step 1: 소스 스냅숏** — `git ls-files -z | tar --null -T - -cf -` 를 `ws1:/home/projects/forklift/artifacts/<UTC>_recognition_isaac/source` 로 풀고 `source_manifest.json`(리비전·dirty)을 같이 둔다.
- [ ] **Step 2: 카탈로그 해시(Isaac 환경)** — ws1 짧은 CPU 잡: `/home/projects/forklift/.venv/bin/python tools/generate_recognition_scenes.py --out /dev/null --digest-only` → `catalogue_sha256` 이 1A 의 `37a5d25de304b558ee4d24216ba2dc7086656fbf85dbb550bc3675bb4c593be7` 과 같은지 본다. 다르면 원인(numpy 판)을 기록하고, 1A 리그 결과와 장면 단위 비교 표는 만들지 않는다(Isaac 결과 자체는 유효 — 정답은 같은 환경의 조리법으로 계산된다).
- [ ] **Step 3: 연기 렌더(dev 만)** — 사례·형상마다 dev 2 장면: `sbatch --chdir=<run dir> --export=ALL,SRC=…/source,OUT=…/smoke_isaac,ARGS="--split dev --limit 2" deploy/slurm/recognition_isaac_render.sbatch`(대기는 `run_in_background`). Expected: 종료 코드 0, `status.json` 의 `failed` 비어 있음, `fatal` 없음.
- [ ] **Step 4: 비교** — ws1 짧은 CPU 잡: `python tools/compare_isaac_rig.py --isaac …/smoke_isaac --out …/smoke_compare.json`. Expected: 종료 코드 0, `pass: true`. 실패하면 원인(yaw 부호, 반크기, 카메라 자세, 바닥 높이, 라벨 누락, 반 화소)을 systematic-debugging 으로 찾아 고친 뒤 다시 — 문턱은 바꾸지 않는다(문턱이 렌더러의 정상 거동과 맞지 않는다는 근거가 나오면 그 근거와 함께 검토 B 에 올려 정한다). dev 장면 몇 개의 RGB·깊이·인스턴스 영상을 내려받아 직접 본다.
- [ ] **Step 5: 시간·자원 실측** — `sacct -j <id> -o JobID,Elapsed,MaxRSS` 와 `status.json` 의 장면 수로 장면당 시간·RAM 을 재서 작업 8 의 샤드 수·`--mem`(실측 최대 + 25 %)·`--time`(실측 × 1.5)을 정한다.
- [ ] **Step 6: 커밋** — 코드 수정이 있었을 때만(해당 파일만 `git add`).

**검토 B:** 작업 5–7 뒤 Codex — 초점: "prim 명세가 리그 세계와 같은가, 토큰 라벨·준비 판정, K 대조와 정답 K, 깊이 정리, 비교 도구의 집계 단위·유효 비율·배경 면 거리, 연기 결과". 수렴까지 반복.

### Task 8: 전체 Isaac 렌더·리그 대조·잡음 판·dev 기준선

- [ ] **Step 1: 전체 렌더** — 잡음 없는 1,020 장면(dev·eval 모두 — eval 은 나중 한 번의 판정에 쓸 같은 렌더다)을 작업 7 실측대로 샤드(예: 4 샤드 동시, 각 shard 1)로 렌더해 샤드마다 `…/isaac_clean_<i>/` 에 쓰고 `…/isaac_clean/` 로 모은다(디렉터리 이름이 겹치지 않음을 확인). 모든 잡의 `status.json` `failed` 비어 있음, 모은 장면 수 1,020.
- [ ] **Step 2: 전체 리그 대조** — ws1 CPU 잡: `compare_isaac_rig --isaac isaac_clean` 이 1,020 장면 모두 합격해야 다음 단계로 간다. 검출기를 돌리지 않는 기하 대조라 eval 을 연 것으로 보지 않는다(Global Constraints). 불합격 장면은 원인을 찾아 다시 렌더한다.
- [ ] **Step 3: 잡음 판** — ws1 CPU 잡: `apply_recognition_noise --src isaac_clean --out isaac_official --noise-config config/depth_noise_d435i.yaml` 과 `--scale 1.66 --out isaac_sensitivity`.
- [ ] **Step 4: dev 평가** — ws1 CPU 잡(데이터가 ws1 에 있음)에서 세 판(`isaac_clean`, `isaac_official`, `isaac_sensitivity`) 각각 oracle·baseline: `python tools/evaluate_recognition.py --dataset …/isaac_official --split dev --detector oracle --output …/isaac_official_oracle_dev --source-manifest source_manifest.json` 등. oracle 은 평가기·정답 자기 점검일 뿐이다(관측을 읽지 않는다) — 세 판 모두 관문 실패 0. `official` 은 `isaac_official` 에서만 참.
- [ ] **Step 5: 섞임 거부 확인** — 리그 장면 하나를 Isaac 판 복사본에 넣으면 `exactly one render source`, 두 잡음 판의 장면을 섞으면 `exactly one noise configuration`, 잡음 판에 `apply_recognition_noise` 를 다시 돌리면 `already carries noise` 로 끝나는지 한 번씩 확인한다.
- [ ] **Step 6: 기록** — `docs/validation/<실행일>-recognition-stage1b-isaac-baseline.md`: 잡음 측정(MR6D 리비전·분할·프레임 수·해시, 채택값과 프레임 분포, 밴드별 분포, 평면 칸 교차 확인, 팔레트 표면 비교, 무효 화소, 한계, `calibration_touched`), 연기·전체 대조, 렌더 조건(리비전·카탈로그 해시·잡 ID·GPU·시간·RAM), oracle 자기 점검, 세 판의 baseline 사례별 표(dev; S6·N 은 상자만 쓴 **잠정**으로 표시), 작업 4 Step 6 의 리그 참고 표와 1A 리그 결과와의 비교, 해석 경계(상자 렌더 — 재질·조명 단순, 사실적 외형 메쉬와 창고 자산은 단계 3; 잡음은 MR6D 조건의 배경 측정; 무효 화소는 독립 화소 모형; 시간상 고정 패턴 없음).
- [ ] **Step 7: 커밋·푸시** — `git add docs/validation/<파일>`(그리고 바뀐 sbatch) · `git commit -m "validation: recognition stage 1B -- MR6D-measured depth noise (iid + correlated), Isaac renders matching the rig, dev baseline on the official (Isaac + measured noise) population"` · `git push -u origin feat/recognition-stage1b`

**검토 C:** 작업 8 뒤 Codex — 초점: "기록 수치와 원시 출력의 일치, 해석의 근거, 정식 판정 모집단의 조건 충족". 수렴까지 반복.

## 자체 검토

1. **설계 대응:** §5.3 렌더(Isaac, 인스턴스 마스크, 상자 prim 묶기) → 작업 5–6; 잡음 판 → 작업 1–4·8; TF 출처 → 작업 6(`isaac_render_pose`); 정답 → 작업 4·6(`recipe_record`, Isaac K, `truth_render = rig`); 분할 → 검출기는 dev 만. §5.1 의 S6 창고 자산과 N 의 다른 출처 구조는 쓰지 않는다 — 리그와 같은 기하를 지켜 정답을 그대로 쓰기 위해서이고, 그래서 S6·N 결과는 잠정이다. 자산은 사실적 외형과 함께 단계 3. 설계가 실사 평가로 남긴 MR6D 는 `dynamic` 을 보정에, val 2 장면을 보고용 비교에만 쓰고 그 사실을 설정 파일에 남긴다.
2. **자리표시:** 작업 4 Step 1 의 `GOLDEN` 값은 실행 시 정리 전 코드로 잰다(잴 방법과 시점은 정했다). 작업 4 의 정답 K 시험은 1A 정답 시험 파일(이름은 실행 때 확인)에 더한다.
3. **타입 일관성:** `NoiseModel.from_yaml(path, *, seed, target_fx, scale)` 을 작업 3·4 가 같게 쓴다. `recipe_record(recipe, site, intrinsics=None)`·`load_site`·`noise_seed` 를 작업 4·6 이 같게 쓴다. `decode_semantic(payload, token, shape)` 을 작업 5·6 이 같게 쓴다.
4. **검토 기록:** 아래.

## 검토 기록

### 1판 → 2판 (2026-10-15)

세 검토를 서로 결론을 보지 않고 돌렸다.

- **① 원자료 재계산(작성자):** MR6D val 000000 프레임 0 을 읽어 변동도·모형 맞춤·무효 화소를 쟀다(위 "계획 전 실측"). 1판의 "평면 칸 맞춤 + 거리 띠" 방법은 상관 잡음을 놓치고(간격 1→32 에서 15 배), 모서리 거르기가 비스듬한 바닥을 모두 버린다는 것을 확인해 2 차 차분 변동도로 바꿨다. 리그 카메라·K(주점 320 vs Isaac 정수 첨자 319.5), N 조리법의 방해물, 평가기의 `noisy` 쓰임새, 판 2 깊이 mm 반올림을 코드에서 확인했다. 시제품으로 합성 회복을 확인했다. `dynamic` 분할을 열어 마스크·K(fx ≈ 909)·깊이 범위(20 m 까지)를 확인했다.
- **② Codex 적대적 검토(1회차, 미수렴 — P1 2·P2 8):** 반영 — 깊이 비교를 대상별·유효 비율·배경 면 거리로 나눔(P1: 팔레트 전체 +100 mm 나 NaN 이 합격하던 집계), 동적 로더의 `sys.modules` 등록과 빠진 import(P1), 정답을 Isaac K 로 계산하고 K 규약을 설정과 대조(P2; 초점 차 검사가 자기 비교였음), 배경 변동의 식별을 프레임별 맞춤·평면 칸 교차 확인으로 보강(P2), 프레임 혼합 MAD 대신 프레임별 맞춤과 분포·채택 조건(P2), 잡음 설정 섞임 거부와 역할 기반 `official`(P2), 전체 렌더를 oracle 대신 리그 대조로 검증(P2), 연기 시험 dev 한정(P2), MR6D 보정 분할 분리(P2), 준비 안 된 어노테이터 재시도(P2), 설정 범위 검사·고정 목록·생성기 해시 범위(P3). 반박 없음으로 확인된 것: 분리형 가우스 정규화와 V(s) 식, 합성 회복 문턱, 별도 난수 흐름, Isaac 초기화·`capture_static`·ROS 축·깊이 단위·반 화소 방향, `add_labels` 형식, 잡음 없는 렌더 + CPU 잡음 구성.
- **③ 독립 검토(결론 비공개, 적히지 않은 전제):** 반영 — 칠한 벽 측정이 나무 팔레트에 옮겨지는가(P1 제기): 같은 프레임 팔레트 표면을 재 보니 작은 간격에서 벽과 같아(위 표) 채택값은 배경 측정을 유지하되 `pallet_surface` 비교를 설정에 남긴다; 후처리 미상과 g 도 민감도 배율에 포함(P2); z² 법칙 외삽은 `dynamic` 의 4–6 m 밴드로 확인(P2); 정식 모집단을 배율 1 판 하나로 정하고 S6·N 잠정 표시(P2); 시간 고정 패턴·합성 K·로봇 깊이 모드 미정을 한계에 적음(P3); 바닥의 깊이 기울기는 배경 면 거리 비교로 해결(P3); 리비전 고정·프레임 수 정정(P3). 이 검토가 같은 프레임 48 px 칸 평면 잔차(σ_w 0.0063)로 상관 성분이 장면 형상이 아님을 교차 확인했다.
- 받아들이지 않은 것: 독립 검토의 "4 m 에서 잡음 깊이를 자르는 변형" — 팔레트 거리는 카메라에서 1.1–4.0 m 라 판정 대상 밖이고, `dynamic` 밴드 맞춤으로 외삽 자체를 확인하므로 판을 하나 더 두지 않는다(밴드가 z² 와 어긋나면 그때 다시 정한다).

### 2판 Codex 재검토 (2회차, 미수렴 — 새 P1 없음, P2 2)

- 평가기 `official` 시험의 기존 `META` 가 `split="eval"` 이라 `summarize(split="dev")` 에서 장면이 0 개가 됨 → 시험 메타에 `"split": "dev"` 를 명시.
- 완전히 유효한 48 px 칸이 없는 프레임이 채택돼 교차 확인을 우회(합성 반례: 4/4 채택, 교차 확인 1 프레임) → 칸 수 < 20 이면 `patches` 사유로 버리고, 같은 반례를 시험으로 추가(채택 3/4 미만 → 실패).
- 반박 없음으로 확인: 대상별 깊이 비교의 반례 4 종 불합격, `SceneVerdict` 위치 인자 순서·CSV, 정답 K 전달, 잡음 메타 보존, 생성기 정리 뒤 8 개 기록의 해시 동일, 준비 판정, 연기 선택 28 장 모두 dev, 평면 칸 구현.

### 3회차 (수렴)

Codex: 새 P1/P2 없음. `official` 시험이 `split="dev"` 로 통과하고, 평면 칸 반례는 앞 3 프레임 `reject="patches"`(칸 0 개)·마지막 채택(118 칸)으로 `only 1/4 frames accepted` 가 되어 우회가 막혔음을 계획 코드를 CPU 로 재현해 확인했다. 전체 pytest·MR6D 실측·Isaac 실행은 구현 단계에서 확인한다.

## 개정 1 (2026-10-16, 사용자 결정 — 잠정 범위로 진행 + 실차 측정)

### 무엇이 계획대로 되지 않았나

- **MR6D 보정(작업 3 Step 5):** `dynamic` 깊이는 1 mm 정수이고 가로로 매끈해 가로 간격 1 의 2 차 차분이 절반 넘게 정확히 0 이다(MAD = 0). UBAI 1012969 는 그런 프레임에서 멈췄고, 0 분산 프레임을 거부한 20261015T144823Z 는 240 프레임 중 0 개를 채택했다. ±0.5 mm 디더를 넣으면 단일 상관 모형의 잔차가 모든 프레임 0.50–0.55 다. 두 성분(짧은 성분 k ≈ 0.0005, 폭 가로 1.6–2.2·세로 0–0.9 px / 긴 성분 k ≈ 0.0025–0.0047, 폭 가로 12–17·세로 7–9 px; 1280 격자)은 6 프레임에서 일관되지만 잔차 ≈ 0.25 다.
- **Codex 상의(같은 날):** 다중 성분 방향은 타당. 그러나 간격 4–32 만 보는 관문은 세로 간격 1 을 91 % 작게 예측하는 모형을 통과시키고(P1), 같은 영상의 평면 칸 잔차는 센서 오차와 표면 굴곡을 구별하지 못하며(P1), 디더 + MAD 는 분산 덧셈으로 해석할 수 없다(P2). MR6D 만으로 "센서 잡음을 쟀다"고 할 근거는 측정 과정 모사 맞춤·확인된 평면·별도 검증 자료 없이는 만들 수 없다.
- **Isaac 라벨 맵:** Isaac 이 같은 34/1,020 장면에서 이전 장면의 ID→라벨 맵을 유지했다(깊이는 새 장면). 장면 사이에 라벨 prim 이 없는 프레임을 한 번 캡처하면 해결됐다(`0ae6d1b`). 전체 1,020 장면 렌더 성공.

### 사용자 결정

정식(official) 잡음 판은 **팀 D435i 실측 뒤**에 정한다. 그 전까지 Isaac 기준선은 MR6D 에서 잰 두 성분 모형의 **잠정 범위(낮음·중간·높음)** 로 내고 `role = "provisional"` 로 표시한다 — 정식 판정이 아니다.

### 바뀐 작업

- **작업 1′ — 잡음 모형 다중 성분:** `NoiseModel.correlated_k`·`correlation_px` 를 `components: tuple = ()`(각 `(k, (g_u, g_v))`)로 바꾼다. 빈 튜플이면 1A 와 같은 출력. 성분마다 `default_rng([seed, 1 + i])`. `to_json` 은 성분이 있을 때만 `components` 를 넣는다.
- **작업 2′ — 두 성분 측정:** `fit_noise_components(var_u, var_v, n=2)` — 짧은 폭의 응답은 연속 근사 대신 **생성기 이산 커널의 정규화 자기상관**으로 계산하고, 진폭은 모든 열 부분집합을 보는 비음수 최소제곱으로 푼다(Codex P2). 단일 성분 `fit_noise_structure` 는 남긴다(시험·진단용).
- **작업 3′ — MR6D 조사(채택 관문 없음):** `calibrate_depth_noise.py` 가 프레임마다 디더(±0.5 mm, 시드 고정) 뒤 두 성분을 맞춰 **분포만 기록**한다(성분은 g_u + g_v 순으로 짧은·긴으로 구분, 간격 1–32 의 모형/측정 비와 잔차, 간격 48·64 진단 분산, 평면 칸 비, 거리 구간별 맞춤). 분위수 모집단은 **사용 가능 프레임** = 표본 부족·0 분산·평면 칸 부족으로 빠지지 않은 모든 프레임이고, 빠진 개수는 사유별로 남긴다. 설정 파일은 `role: provisional` 과 세 수준 `levels: {low, mid, high}` — 짧은 성분은 모든 수준에서 사용 가능 프레임의 중앙값, 긴 성분 k 는 p10·p50·p90, 폭은 중앙값, **`k_iid` 는 0**(두 성분이 화소 독립까지 포함한 전체 모형; 폭 0 은 화소 독립) — 을 담는다. 세 수준은 **인공 민감도 시나리오**이며 80 % 포괄 범위·신뢰 구간·최악 조건이 아니고(폭·진폭의 공동 변동, 짧은 성분의 변동, 디더/MAD 편향을 포괄하지 않음), 검출 성능이 수준 순서대로 변한다는 보장도 없다고 설정 파일(`levels_note`)과 기록에 적는다. 값은 "센서 잡음 실측"이 아니라 "MR6D 배경 역깊이 변동의 모형"이다. 1 차 시도의 실패 이력(잡 ID·0/240·반례)도 기록한다.
- **작업 4′ — 잡음 입히기:** `apply_recognition_noise --level low|mid|high` 가 그 수준을 쓰고 `meta.noise.level` 과 `role`(설정이 provisional 이면 provisional, official 이면 배율 1 에서 official·그 밖은 sensitivity)을 기록한다. 평가기 서명은 `config_sha256:scale:role:level` 이라 수준이나 역할이 섞인 집계를 거부하고, `official` 은 `role == "official"` 일 때만 참이다(잠정 판은 모두 거짓). 수준 혼합 거부와 provisional 의 `official = false` 를 시험으로 둔다.
- **작업 8′ — 기준선:** `isaac_clean` 과 세 잠정 수준 판에서 dev oracle·baseline. 기록은 "잠정 기준선".
- **작업 9(신규) — 실차 측정 준비:** `tools/capture_d435i_noise.py`(pyrealsense2; 깊이 모드·노출·레이저 설정 기록, RealSense 후처리 필터 끔, 정지 장면 N 프레임을 uint16 mm PNG + 내부 행렬 + 메타로 저장)와 촬영 절차 문서 `docs/procedures/d435i-noise-capture.md`(벽 평면 1·2·3·4 m 정면, 팔레트 정면 같은 거리, 각 30 프레임; 로봇에서 쓸 깊이 모드). 하드웨어 없이 시험할 수 있는 부분(메타·저장 형식)만 단위 시험한다. **캡처 계약:** 원 Z16 값과 장치의 `depth_scale_m` 을 그대로 저장(mm 영상은 파생본), 실제 스트림 해상도·FPS·깊이 격자 K·왜곡계수·정렬 여부, 프레임 번호·장치 시각·clock domain, 장치·펌웨어·SDK 판, 노출/자동노출·gain·emitter·preset 의 실제 적용값, 워밍업 시간, 정지 확인, 평면·팔레트 ROI 와 줄자(또는 레이저 거리계)로 잰 거리. 30 프레임은 첫 촬영 분량이고 충분한지는 시간 상관과 반복 세션을 본 뒤 정한다. **해석 한계:** 같은 위치의 반복 촬영은 **시간 변동**만 추정한다 — 시간 평균에는 실제 형상과 고정 센서 편향이 함께 남는다. 고정 공간 오차와 절대 편향은 평평함이 보장된 기준 평면(예: 유리·평판)과 독립 거리 기준이 있어야 가를 수 있고, 가르지 못한 성분은 `unverified` 로 남긴다. 실측 자료로 정식 판을 정하는 방법은 측정 뒤 별도 계획으로 한다.

### 검토

이 개정은 Codex 에 보내 수렴까지 검토한다(아래 기록).

### 개정 1 검토 (Codex)

- 1 회차(미수렴, P1 1·P2 4): 반복 촬영의 시간 평균이 형상을 가른다는 주장 철회 — 시간 변동만 추정하고 고정 편향은 기준 평면·거리 기준 필요(P1); 캡처는 원 Z16·`depth_scale_m` 과 전체 메타 보존(P2); 잠정 수준의 모집단·정렬 기준·"인공 시나리오" 명시(P2); 잠정 설정의 `k_iid = 0` 명시(P2); 평가기 서명에 역할·수준을 넣어 수준 혼합 거부(P2). 모두 반영.
