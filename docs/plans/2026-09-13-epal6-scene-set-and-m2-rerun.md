# EPAL 6 장면 세트 재생성과 M2 재평가 계획

> **실행자:** Task 1–2(파이프라인 전환)는 Codex에 위임한다. 실패 시험 → 실패 확인 → 구현 → 통과 확인 순서를 지키고 **커밋·stage 는 하지 않는다.** Task 3–6(카탈로그 생성·원격 캡처·튜닝·eval·기록)은 Claude가 한다.

**목표:** 합성 장면 100장을 실물 EPAL 6 형상으로 다시 만들고, 그 위에서 M2 포켓 인식기를 다시 튜닝해 eval 30장면을 **한 번** 돌려 최종 지표를 낸다.

**왜:** 기존 100장은 높이 0.30 m·개구 0.20 m의 가공 형상이다. M2의 18/18은 그 형상에 대한 결과이고 실물 팔레트로 옮겨지지 않는다. 형상 정본과 도킹 검증이 끝났으므로(`docs/validation/2026-09-13-epal6-pallet-and-docking.md`) 이제 장면을 그 형상으로 맞춘다.

**Spec:** `config/pallet_geometry_epal6.yaml`(정본), `docs/design/2026-09-13-pocket-detector-baseline.md`(인식 설계 v2), `docs/design/2026-09-11-pocket-observation-and-scene-set.md`(장면 세트 설계 v1).

## 전역 제약

- 기준: `main` `5157f7b`. 트리 깨끗.
- **형상 말고는 아무것도 바꾸지 않는다.** 카메라(640×480, HFOV 1.204, 5 Hz, base (0.75, 0, 0.5)), 자세 범위(x 2.0–4.0, y ±1.0, yaw ±0.52), 범주 구성(positive 60 / occluded 20 / negative_no_pallet 10 / negative_lookalike 10), dev 70 / eval 30 분할, 조명·바닥·distractor 목록, 난수 seed 규칙을 그대로 둔다. **그래야 v1 대비 차이가 형상 때문임을 말할 수 있다.**
- **v1 은 건드리지 않는다.** `sim/gazebo/scenes/catalogue_v1.yaml`, `config/pallet_prior_v1.yaml`, `data/synthetic_scenes/catalogue_v1/`, 그리고 그 위의 검증 기록은 그대로 남는다. 새 것은 `catalogue_epal6` 이름을 쓴다.
- **개구 폭은 이제 장면마다 달라지지 않는다.** 실물 EPAL 6 은 한 종류이므로 개구 폭은 280 mm 고정이다. 인식기 prior 의 폭 범위는 이제 모집단이 아니라 **허용오차**다. 이 변화를 검증 기록에 적는다.
- **⚠ 난수 흐름을 반드시 보존한다.** v1 은 장면마다 `x_m`, `y_m`, `yaw_rad`, `opening_width_m` 네 값을 그 순서로 뽑는다. 폭 추출을 그냥 지우면 이후 모든 난수가 밀려 **장면이 전부 달라지고 v1 과 비교할 수 없게 된다.** 따라서 **폭은 계속 뽑되 값을 쓰지 않는다.** 코드에 그 이유를 주석으로 남기고, 생성한 카탈로그의 자세가 v1 과 장면 단위로 **완전히 일치하는지 시험으로 고정한다.** 이것이 "형상만 바뀌었다"를 증명하는 유일한 방법이다.
- **v1 dev/eval 분할 규칙을 그대로 쓴다.** 같은 seed·같은 분할 규칙이면 같은 장면 번호가 같은 split 에 간다. 그래야 v1 과 epal6 을 장면 단위로 비교할 수 있다.
- dev 에서만 튜닝하고 **eval 은 파라미터 확정 후 한 번**만 돌린다. eval 을 보고 되돌리면 그 실행은 최종 보고가 아니며 기록에 적는다.
- 기존 시험 724 개의 행동을 보존한다. `ruff check .`·`ruff format --check .` 통과.

## 형상이 인식에 주는 영향 (착수 전 계산)

개구 높이가 200 → 78 mm 다. 인식기는 그 높이 대역 안의 점만 쓰므로 대역이 얇아진다.

| 거리 | v1 대역(margin 10 mm) | EPAL 6 대역(margin 10 mm) | EPAL 6 대역(margin 5 mm) |
|---|---|---|---|
| 2.0 m | 67.1 px | 21.6 px | 25.3 px |
| 3.0 m | 37.3 px | 12.0 px | 14.1 px |
| 4.0 m | 25.8 px | 8.3 px | 9.7 px |

**따라서 `band_margin_m 0.01` 과 `min_band_points 100` 은 거의 확실히 다시 정해야 한다.** margin 10 mm 는 78 mm 대역의 26 %를 잘라낸다(v1 은 10 %). `floor_z_m 0.02` 도 위험하다. EPAL 6 의 아래 덱은 0–22 mm 라서 바닥 필터가 덱 윗면과 2 mm 밖에 안 떨어진다. 이것들은 **Task 5 에서 dev 로만** 조정한다. 지금 미리 바꾸지 않는다.

## 파일 구조

| 파일 | 책임 |
|---|---|
| `tools/generate_scene_catalogue.py` | 형상 YAML 에서 팔레트 블록을 읽고 개구 폭 표본추출을 뺀다 |
| `sim/gazebo/build_scene_world.py` | 형상 YAML 에서 SDF 팔레트 11개 box 를 만든다 |
| `sim/gazebo/scenes/catalogue_epal6.yaml` | 생성물(Task 3) |
| `config/detector_params_epal6.yaml` | dev 튜닝 결과(Task 5) |
| `docs/validation/2026-09-1x-pocket-detector-epal6.md` | 재평가 기록(Task 6) |

---

### Task 0: 기준 확인 (Claude)

- [ ] `git status --short` 가 비어 있고 HEAD 가 `5157f7b` 임을 확인한다.
- [ ] 기준 회귀 724 passed, 1 deselected.

### Task 1: 카탈로그 생성기를 형상 정본에 연결

**Files:** Modify `tools/generate_scene_catalogue.py`, `tests/simulation/test_scene_catalogue.py`

**Interfaces — Produces:** `sample_catalogue(seed, count, geometry)` 가 `PalletGeometry` 를 받는다. 모듈 상수 `PALLET` 을 지우고 카탈로그의 `pallet:` 블록을 형상에서 만든다:

```python
{
    "depth_m": geometry.overall_depth_m,
    "width_m": geometry.overall_width_m,
    "height_m": geometry.overall_height_m,
    "deck_bottom_m": geometry.deck_bottom_m,
    "deck_top_m": geometry.deck_top_m,
    "block_width_m": geometry.block_width_m,
    "block_depth_m": geometry.block_depth_m,
    "opening_height_m": geometry.block_height_m,
    "opening_width_m": geometry.opening_width_m,
    "center_spacer_m": geometry.block_width_m,
}
```

`RANGES` 의 `opening_width_m` 항목은 **남겨 두고 계속 뽑되 결과를 버린다**(전역 제약의 난수 흐름 보존). `pallet_ground_truth(x_m, y_m, yaw_rad, geometry)` 로 바꾸고 포켓 중심 오프셋은 `geometry.opening_centre_offset_m`, 높이는 `geometry.opening_centre_height_m` 을 쓴다. 장면 항목에는 `opening_width_m` 을 더 이상 넣지 않는다. `catalogue_version` 은 `epal6`.

- [ ] **Step 1: 시험 작성** — `tests/simulation/test_scene_catalogue.py` 에 추가

```python
def test_the_catalogue_pallet_block_comes_from_the_geometry_file():
    geometry = load_pallet_geometry(EPAL6)
    catalogue = generate_scene_catalogue.sample_catalogue(20260913, 8, geometry)
    assert catalogue["catalogue_version"] == "epal6"
    assert catalogue["pallet"]["opening_width_m"] == pytest.approx(0.280)
    assert catalogue["pallet"]["opening_height_m"] == pytest.approx(0.078)
    assert catalogue["pallet"]["height_m"] == pytest.approx(0.144)
    assert catalogue["pallet"]["deck_bottom_m"] == pytest.approx(0.022)
    assert catalogue["pallet"]["deck_top_m"] == pytest.approx(0.044)


def test_no_scene_carries_its_own_opening_width_any_more():
    geometry = load_pallet_geometry(EPAL6)
    catalogue = generate_scene_catalogue.sample_catalogue(20260913, 8, geometry)
    for scene in catalogue["scenes"]:
        if scene["pallet"] is not None:
            assert "opening_width_m" not in scene["pallet"]


def test_the_scene_poses_are_identical_to_the_v1_catalogue():
    """The width draw is kept and discarded so the random stream does not shift.

    Without this the whole set would be different scenes and no statement of the
    form "only the pallet shape changed" would be supportable.
    """
    geometry = load_pallet_geometry(EPAL6)
    fresh = generate_scene_catalogue.sample_catalogue(20260911, 100, geometry)
    v1 = yaml.safe_load((REPO_ROOT / "sim/gazebo/scenes/catalogue_v1.yaml").read_text())
    assert len(fresh["scenes"]) == len(v1["scenes"])
    for new_scene, old_scene in zip(fresh["scenes"], v1["scenes"], strict=True):
        assert new_scene["scene_id"] == old_scene["scene_id"]
        assert new_scene["category"] == old_scene["category"]
        assert new_scene["split"] == old_scene["split"]
        if old_scene["pallet"] is None:
            assert new_scene["pallet"] is None
            continue
        for key in ("x_m", "y_m", "yaw_rad"):
            assert new_scene["pallet"][key] == pytest.approx(old_scene["pallet"][key])


def test_the_ground_truth_pockets_sit_at_the_geometry_offset_and_height():
    geometry = load_pallet_geometry(EPAL6)
    truth = generate_scene_catalogue.pallet_ground_truth(3.0, 0.0, 0.0, geometry)
    assert truth.left.center_m[1] == pytest.approx(0.180)
    assert truth.right.center_m[1] == pytest.approx(-0.180)
    assert truth.left.center_m[2] == pytest.approx(0.061)
    assert truth.left.width_m == pytest.approx(0.280)
    assert truth.left.height_m == pytest.approx(0.078)


def test_a_geometry_whose_openings_vanish_is_refused(tmp_path):
    # three blocks that fill the width leave no opening to sample
    data = yaml.safe_load(EPAL6.read_text())
    data["block_width_m"] = 0.267
    bad = tmp_path / "bad.yaml"
    bad.write_text(yaml.safe_dump(data))
    with pytest.raises(ValueError):
        generate_scene_catalogue.sample_catalogue(
            20260913, 4, load_pallet_geometry(bad)
        )
```

- [ ] **Step 2: 실패 확인.** **Step 3: 구현.** **Step 4: 통과 확인** + Ruff.

### Task 2: 월드 생성기를 형상 정본에 연결

**Files:** Modify `sim/gazebo/build_scene_world.py`, `tests/simulation/test_build_scene_world.py`

팔레트를 **11개 box** 로 만든다: 아래 덱(`deck_bottom`) 1개, 블록 9개(`block_x{0,1,2}_y{0,1,2}`), 위 덱(`deck_top`) 1개. 이름과 배치는 `tools/build_pallet_model.py` 가 만드는 MJCF 와 **같은 규약**을 쓴다. 카탈로그의 `pallet:` 블록이 형상 파일에서 유도한 값과 다르면 `ValueError` 로 거부한다(하드코딩된 숫자 비교를 지운다). `catalogue_version` 은 `epal6` 만 받는다.

lookalike 는 지금처럼 꽉 찬 직육면체 하나를 유지하되 **EPAL 6 외형**(0.6 × 0.8 × 0.144)으로 만든다. 크기가 팔레트와 같아야 음성으로서 의미가 있다.

- [ ] **Step 1: 시험 작성** — `tests/simulation/test_build_scene_world.py` 에 추가

```python
def test_the_world_pallet_is_the_eleven_box_epal6_shape():
    world = build_scene_world.build_world(CATALOGUE, "s001", GEOMETRY)
    boxes = _pallet_boxes(world)  # helper in this file
    assert len(boxes) == 11
    assert {name for name, _, _ in boxes} == {"deck_bottom", "deck_top"} | {
        f"block_x{i}_y{j}" for i in range(3) for j in range(3)
    }
    sizes = {name: size for name, size, _ in boxes}
    assert sizes["deck_bottom"] == pytest.approx([0.600, 0.800, 0.022])
    assert sizes["block_x1_y1"] == pytest.approx([0.073, 0.080, 0.078])
    assert sizes["deck_top"] == pytest.approx([0.600, 0.800, 0.044])


def test_the_openings_of_the_world_pallet_are_where_the_geometry_says():
    world = build_scene_world.build_world(CATALOGUE, "s001", GEOMETRY)
    ys = sorted(pose[1] for name, _, pose in _pallet_boxes(world) if "block" in name)
    assert sorted(set(round(v, 4) for v in ys)) == pytest.approx([-0.360, 0.0, 0.360])


def test_a_catalogue_that_disagrees_with_the_geometry_is_refused(tmp_path):
    data = copy.deepcopy(CATALOGUE)
    data["pallet"]["opening_height_m"] = 0.200
    with pytest.raises(ValueError):
        build_scene_world.build_world(data, "s001", GEOMETRY)


def test_the_lookalike_keeps_the_pallet_envelope():
    world = build_scene_world.build_world(CATALOGUE, LOOKALIKE_SCENE, GEOMETRY)
    name, size, _ = _lookalike_box(world)
    assert size == pytest.approx([0.600, 0.800, 0.144])
```

- [ ] **Step 2: 실패 확인.** **Step 3: 구현.** **Step 4: 통과 확인** + Ruff. 기존 v1 장면 시험이 깨지면 그 시험을 epal6 형상으로 옮긴다. **v1 카탈로그 파일 자체는 지우지 않는다.**

### Task 3: 카탈로그 생성과 검토 (Claude)

- [ ] `tools/generate_scene_catalogue.py` 로 `sim/gazebo/scenes/catalogue_epal6.yaml` 을 만든다. seed 는 v1 과 같은 `20260911` 을 쓴다.
- [ ] 범주별 개수(60/20/10/10)와 split(dev 70 / eval 30), 그리고 **자세(x, y, yaw)가 v1 과 장면 번호 단위로 완전히 같은지** 확인한다. 하나라도 다르면 난수 흐름이 어긋난 것이므로 멈추고 원인을 고친다. 같은 장면·다른 형상이어야 비교가 성립한다.
- [ ] 장면 몇 개의 월드 SDF 를 만들어 팔레트가 11개 box 인지, 개구부가 비어 있는지 확인한다.

### Task 4: 원격 캡처와 병합 (Claude)

- [ ] `tools/submit_model_check.py` 의 `scenes` mode 로 4배치(s001–s025, s026–s050, s051–s075, s076–s100) 제출한다. 새 run ID 를 쓴다.
- [ ] `tools/merge_scene_batches.py` 로 `data/synthetic_scenes/catalogue_epal6/` 를 만든다. set manifest SHA-256 을 기록한다.
- [ ] 로더로 몇 장면을 읽어 정답 포켓 폭 0.280·높이 0.078·중심 높이 0.061 을 확인한다.
- [ ] 원격 파일 그룹·권한(`kang:ed26_2_riibotics`)을 확인한다.

### Task 5: dev 튜닝 (Claude)

- [ ] 먼저 **현재 파라미터 그대로** dev 70장면을 1회 돌려 기준선을 남긴다. 얼마나 나빠지는지가 형상 변화의 비용이다.
- [ ] 실패 원인별로 파라미터 후보를 정하고 dev 에서만 재실행한다. 우선 후보는 `band_margin_m`, `min_band_points`, `floor_z_m`, `max_plane_residual_m` 이다. **한 번에 하나씩** 바꾸고 각 실행의 지표를 표로 남긴다.
- [ ] 시험 허용오차와 `evaluation.py` 의 목표 상수는 바꾸지 않는다.
- [ ] 확정 파라미터를 `config/detector_params_epal6.yaml` 로 저장한다.

### Task 5.5: 동결 게이트 (Claude)

- [ ] 최종 파라미터 YAML 을 CLI 로 다시 읽어 dev 1회 실행하고 그 실행을 선택 근거로 기록한다.
- [ ] 구현·시험·prior·params 를 커밋해 고정 revision 을 만든다. 전체 회귀와 Ruff 통과를 확인하고 `git_dirty` 가 `false` 인 상태에서 eval 을 돌린다.
- [ ] eval split 이 30장면 전체·양성 18개이고 부분 선택이 없음을 확인한다.

### Task 6: eval 1회와 기록 (Claude)

- [ ] `--split eval --video` 를 **한 번** 실행한다.
- [ ] 목표 도달 여부를 **있는 그대로** 기록한다. 미달이어도 파라미터를 되돌리지 않는다.
- [ ] overlay PNG 와 MP4 를 **직접 보고** 확인한다.
- [ ] `docs/validation/2026-09-1x-pocket-detector-epal6.md`: 구현 범위, v1 대비 비교표(같은 조건에서 형상만 바뀐 결과), dev 실행 표, eval 1회 결과, 실패 장면 분석, 고정 revision, 경계.
- [ ] 체크포인트·로드맵 갱신, 커밋 → push → 원격 동기화.

## 자체 검토

- **Spec 대조:** 형상 정본 사용 → Task 1·2; 장면 재생성 → Task 3·4; dev 전용 튜닝과 eval 1회 → Task 5·5.5·6. 설계의 카메라·범주·분할은 전역 제약에서 고정했다.
- **자리표시자 없음.** 시험은 실제 기대값을 갖는다(형상에서 손으로 유도).
- **이름 일관성:** `load_pallet_geometry`·`PalletGeometry`·`sample_catalogue`·`pallet_ground_truth`·`build_world` 가 Interfaces·시험·파일 표에서 동일하다. box 이름은 `tools/build_pallet_model.py` 규약과 같다.
- **미포함(의도):** v1 세트 수정, 카메라 변경, 학습 검출기, 실물 데이터, 추적(M3), σ 추정, Case B–D.
