# src 레이아웃 구조 전환 구현 계획 (M1-a)

> **실행자:** 이 저장소에서는 구현을 Codex에 위임한다(개인 전역 규칙). 각 Task는 실패 시험 → 실패 확인 → 구현 → 통과 확인 → 커밋 순서로 진행하며, 체크박스(`- [ ]`)로 추적한다. Claude는 계획·검증·문서화를 맡는다.

**목표:** `CONTRIBUTING.md` §10의 전환 표를 **구조 정리 작업 하나**로 적용해 `src/forklift_core/` 레이아웃, `examples/`, `tests/unit|integration/` 배치, 배포명 `forklift-core`로 옮기고, 원격 실행·개발 컨테이너·문서가 새 배치에서 동일하게 동작함을 증명한다.

**아키텍처:** 코어는 `src/forklift_core/`(공유 검증 `_validation.py`, `geometry.py`, `sensors/{rgbd,lidar}.py`)로 이동하고 예제는 `examples/sensor_geometry.py`로 분리한다. 개발 컨테이너는 이미지 빌드 시 `/workspace`를 editable 설치해 bind mount된 checkout의 `src/`를 cwd와 무관하게 import한다. 원격 model job은 읽기 전용 snapshot에서 wheel을 만들어 실행별 venv에 설치한 뒤 시험을 실행하고 import 위치·wheel 해시를 기록한다. 기능 추가는 없다.

**기술 스택:** Python ≥ 3.10, setuptools ≥ 61(PEP 660 editable은 ≥ 64), pytest ≥ 7, Ruff, Docker(legacy builder), Slurm/SSH/rsync, ROS 2 Jazzy 컨테이너.

**근거 문서:** `CONTRIBUTING.md` §1·§4·§6·§7·§9·§10, `docs/plans/2026-09-11-development-roadmap.md` M1, `docs/validation/2026-09-11-development-checkpoint.md`. 이 계획은 2026-09-11 Codex 교차검증에서 지적된 숨은 의존 4곳(원격 model-cpu 무설치 import, snapshot allowlist, 개발 컨테이너 cwd 우선 import, 예제 시험 cwd)을 범위에 포함한다. 같은 날 계획 검토(Codex)에서 나온 수정 2건(원격 복사본 권한, 루트 `__init__.py` 이동)과 보완 사항을 반영했다.

**사전 확인(2026-09-11):** Jazzy 이미지에서 src 레이아웃 editable 설치는 `/workspace/src` 한 줄의 정적 `.pth`를 만들고 `/tmp`에서도 mount된 소스를 import했다. 호스트 `pip wheel`은 빌드 디렉터리에 `build/`와 `src/*.egg-info`를 쓴다. 원격 conda py311 환경에서 `python -m venv --system-site-packages`가 동작하며 새 venv는 pip 24.0·setuptools 79.0.1(base의 26.2.1·84.0.0과 다름)을 갖고 mujoco 3.10.0·numpy 2.4.6이 보인다. base에는 옛 배포명 `forklift-sensor-core 0.1.0`이 일반 설치돼 있다.

## 전역 제약

- 기준 revision: `main` `1beea12`. 작업 브랜치 `refactor/src-layout`에서 진행하고 완료 후 fast-forward로 `main`에 올린다.
- 기능 변경 금지. 코어 함수의 시그니처·동작·오류 메시지를 바꾸지 않는다. 현재 호스트 회귀 145개 행동(rendering 1개 deselected)은 보존하되 개수를 고정 목표로 삼지 않는다.
- `ros2/`, `sim/gazebo/`, `sim/models/`, `tests/simulation/`은 이동하지 않는다. 해당 Python은 `forklift_core`를 import하지 않는다(2026-09-11 grep 확인).
- 개인 경로·SSH 별칭을 tracked 파일에 넣지 않는다. 원격 실행은 새 snapshot·새 실행 ID로만 한다.
- 커밋은 `type(scope): summary` 영어. 파일 단위 stage. commit/push는 사용자 승인 범위(2026-09-11 "ok 진행")에서 계획 커밋 1개 + 문서 이동 1개 + 전환 1개 + 검증 기록 1개로 나눈다.
- 과거 날짜 검증 기록의 명령·수치는 바꾸지 않는다. 링크만 새 경로로 고친다.
- 동적 import(`import_module`)는 모듈 상단 import로 바꾼다(§4).
- `sys.path.insert`, 전역 `PYTHONPATH`, 전역 pip 설치를 표준 절차로 쓰지 않는다(§6).

## 파일 구조 (전환 후)

| 책임 | 경로 |
|---|---|
| 공유 입력 검증 | `src/forklift_core/_validation.py` (`_frame_id`, `_real_array`, `_finite_scalar`) |
| 좌표계·강체 변환 | `src/forklift_core/geometry.py` (`FramePoints`, `RigidTransform`) |
| 깊이 역투영 | `src/forklift_core/sensors/rgbd.py` (`PinholeIntrinsics`, `deproject_depth_pixels`) |
| LiDAR 점 변환 | `src/forklift_core/sensors/lidar.py` (`scan_to_points`) |
| 합성 예제 | `examples/sensor_geometry.py` (구 `forklift_core/demo.py`, 진입점 `main()`) |
| 단위시험 | `tests/unit/test_geometry.py`, `tests/unit/sensors/test_rgbd.py`, `tests/unit/sensors/test_lidar.py` |
| 예제 통합시험 | `tests/integration/test_sensor_geometry_example.py` (구 `tests/test_demo.py`) |
| 원격 도구 시험 | `tests/integration/test_remote_model_jobs.py` (fixture 경로 갱신 + 코어 환경 준비 시험 추가) |
| 패키징 | `pyproject.toml` (`name = "forklift-core"`, `where = ["src"]`, `[tool.ruff] src = [".", "src"]`) |
| 개발 컨테이너 | `deploy/ros2/Dockerfile`, `.dockerignore`, `tools/ros2_dev.sh`(변경 없음 예상), `docs/development.md` |
| 원격 도구 | `tools/submit_model_check.py`(allowlist), `tools/remote_model_job.py`(실행별 코어 환경) |
| 이동 문서 | `docs/validation/2026-09-10-sensor-core.md`, `docs/design/2026-09-10-local-sensor-core-design.md`, `docs/plans/2026-09-10-local-sensor-core.md`, `docs/references/quest.txt`, `docs/references/forklift_store_link.txt` |

---

### Task 0: 기준 상태와 브랜치

**Files:** 없음(확인만).

- [ ] **Step 1:** `git rev-parse HEAD`가 `1beea12…`이고 `git status --short`의 유일한 항목이 이 계획 파일(untracked)임을 확인한다.
- [ ] **Step 2:** `git switch -c refactor/src-layout`.
- [ ] **Step 3:** 계획 문서 커밋: `git add docs/plans/2026-09-11-src-layout-migration.md && git commit -m "docs(plans): plan src layout migration"`. 이후 `git status --short`는 비어 있어야 한다.
- [ ] **Step 4:** 기준 회귀를 기록한다. 실행: `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` → 기대 `145 passed, 1 deselected`. `python -m ruff check .` / `python -m ruff format --check .` → 통과.

### Task 1: 문서·참고자료 이동과 링크 갱신 (순수 이동)

**Files:**
- Move: `docs/LOCAL_VALIDATION.md` → `docs/validation/2026-09-10-sensor-core.md`
- Move: `docs/superpowers/specs/2026-09-10-local-sensor-core-design.md` → `docs/design/2026-09-10-local-sensor-core-design.md`
- Move: `docs/superpowers/plans/2026-09-10-local-sensor-core.md` → `docs/plans/2026-09-10-local-sensor-core.md`
- Move: `quest.txt` → `docs/references/quest.txt`, `forklift_store_link.txt` → `docs/references/forklift_store_link.txt`
- Modify: `README.md`(41행 검증 기록 링크, 75–76행 두 링크), `CLAUDE.md`·`AGENTS.md`(25행 `docs/LOCAL_VALIDATION.md`, 32행 `quest.txt` 명령), `docs/plans/2026-09-10-local-sensor-core.md`(9행 Spec 경로), `docs/references/dls08/README.md`(quest 참조가 있으면)

- [ ] **Step 1:** `git mv`로 5개 파일을 이동하고 빈 `docs/superpowers/`를 제거한다.
- [ ] **Step 2:** 위 링크·명령을 새 경로로 바꾼다. `CLAUDE.md`/`AGENTS.md`의 명령은 `ID=$(grep -oE 'presentation/d/[^/]+' docs/references/quest.txt | cut -d/ -f3)`.
- [ ] **Step 3:** 링크 검사 스크립트로 깨진 상대 링크 0개를 확인한다(아래 Task 7 Step 1의 스크립트를 사용).
- [ ] **Step 4:** `diff <(tail -n +4 AGENTS.md) <(tail -n +4 CLAUDE.md)`가 비어 있음을 확인한다.
- [ ] **Step 5:** 커밋: `git commit -m "docs(references): move brief links and sensor-core records to convention paths"`.

### Task 2: 코어 패키지 src 레이아웃과 예제

**Files:**
- Create: `src/forklift_core/_validation.py`, `src/forklift_core/sensors/__init__.py`(`"""Sensor-specific geometry: depth and 2D scan conversions."""`)
- Move: `forklift_core/__init__.py` → `src/forklift_core/__init__.py`(내용 유지; 루트에 `forklift_core/`가 남으면 `python -m pytest`가 새 패키지를 가리므로 반드시 `git mv`), `forklift_core/geometry.py` → `src/forklift_core/geometry.py`, `forklift_core/rgbd.py` → `src/forklift_core/sensors/rgbd.py`, `forklift_core/lidar.py` → `src/forklift_core/sensors/lidar.py`, `forklift_core/demo.py` → `examples/sensor_geometry.py`
- Move: `tests/test_geometry.py` → `tests/unit/test_geometry.py`, `tests/test_rgbd.py` → `tests/unit/sensors/test_rgbd.py`, `tests/test_lidar.py` → `tests/unit/sensors/test_lidar.py`, `tests/test_demo.py` → `tests/integration/test_sensor_geometry_example.py`
- Modify: `pyproject.toml`

**Interfaces:**
- Produces: `forklift_core._validation._frame_id/_real_array/_finite_scalar`(내용 동일), `forklift_core.geometry.FramePoints/RigidTransform`, `forklift_core.sensors.rgbd.PinholeIntrinsics/deproject_depth_pixels`, `forklift_core.sensors.lidar.scan_to_points`. `examples/sensor_geometry.py`는 `python examples/sensor_geometry.py`로 실행되며 stdout JSON 계약(`input_source`, `scope`, `depth_in_base`, `lidar_in_sensor`)은 그대로다.

- [ ] **Step 1 (RED):** 시험을 먼저 옮기고 import를 바꾼다. `tests/unit/test_geometry.py` 상단을 `from forklift_core import geometry as g`로 두고 본문의 `g = import_module("forklift_core.geometry")` 9줄을 삭제한다(`g.` 참조는 그대로 유지). `tests/unit/sensors/test_rgbd.py`는 `from forklift_core.sensors import rgbd`로 두고 `setup_camera`의 `rgbd = import_module(...)` 줄을 삭제한다(반환값 `(rgbd, intrinsics)` 유지). `tests/unit/sensors/test_lidar.py`도 같은 방식(`from forklift_core.sensors import lidar`). 예제 시험은 다음으로 교체한다.

```python
import json
import subprocess
import sys
from pathlib import Path

EXAMPLE = Path(__file__).resolve().parents[2] / "examples" / "sensor_geometry.py"


def test_documented_example_executes_real_converters_and_preserves_unknown_samples(
    tmp_path: Path,
) -> None:
    result = subprocess.run(
        [sys.executable, str(EXAMPLE)],
        cwd=tmp_path,  # the example must not depend on the checkout as cwd
        capture_output=True,
        text=True,
        timeout=15,
    )
    assert result.returncode == 0, result.stderr
    output = json.loads(
        result.stdout,
        parse_constant=lambda value: (_ for _ in ()).throw(ValueError(value)),
    )
    assert output["input_source"] == "synthetic"
    # keep the remaining assertions of the former tests/test_demo.py unchanged
```

- [ ] **Step 2:** 실행 `python -m pytest tests/unit tests/integration/test_sensor_geometry_example.py -q -p no:cacheprovider -W error` → 기대: `ModuleNotFoundError: forklift_core.sensors` 등으로 실패.
- [ ] **Step 3:** 소스를 이동한다. `_validation.py`에 `geometry.py` 10–31행의 세 함수와 그 함수들만 쓰는 `from numbers import Real` import를 그대로 옮긴다. `geometry.py`는 분리 후 `_finite_scalar`와 `Real`을 쓰지 않으므로 `from forklift_core._validation import _frame_id, _real_array`만 import한다(남기면 F401). `sensors/rgbd.py`·`sensors/lidar.py`의 `from .geometry import ...`를 `from forklift_core._validation import _finite_scalar, _frame_id, _real_array`와 `from forklift_core.geometry import FramePoints`로 바꾼다. `examples/sensor_geometry.py`는 docstring을 `"""Run a synthetic sensor-math example: python examples/sensor_geometry.py."""`로, import를 `from forklift_core.geometry import FramePoints, RigidTransform`, `from forklift_core.sensors.lidar import scan_to_points`, `from forklift_core.sensors.rgbd import PinholeIntrinsics, deproject_depth_pixels`로 바꾼다. 나머지 코드는 바이트 동일.
- [ ] **Step 4:** `pyproject.toml`을 수정한다.

```toml
[project]
name = "forklift-core"

[tool.setuptools.packages.find]
where = ["src"]
include = ["forklift_core*"]

[tool.ruff]
src = [".", "src"]
```

(다른 항목은 유지. `testpaths = ["tests"]` 유지. `pythonpath`는 추가하지 않는다. `src = ["src"]`만 두면 루트 `tools`가 first-party에서 빠져 `tests/integration/test_remote_model_jobs.py`에서 I001이 난다.)

- [ ] **Step 5:** 호스트 editable 재설치: `python -m pip uninstall -y forklift-sensor-core` 후 `python -m pip install -e '.[dev]'`. 루트의 잔여 `forklift_sensor_core.egg-info/`를 삭제한다(Git 제외 산출물). 확인: `cd /tmp && python -c 'import forklift_core.sensors.rgbd as m; print(m.__file__)'` → `.../forklift/src/forklift_core/sensors/rgbd.py`.
- [ ] **Step 6 (GREEN):** Step 2 명령 재실행 → 통과. `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error` → 기존과 같은 통과 개수. `python -m ruff check .` / `format --check .` 통과(isort가 `forklift_core`를 first-party로 분류하는지 확인).

### Task 3: 원격 snapshot allowlist와 fixture

**Files:**
- Modify: `tools/submit_model_check.py` 32–35행 `_SOURCE_RULES`
- Modify: `tests/integration/test_remote_model_jobs.py` 16–29행 `_write_source`, 49행 symlink 대상, 284행 변조 대상

- [ ] **Step 1 (RED):** `_write_source`의 `"forklift_core/__init__.py"`를 `"src/forklift_core/__init__.py": "VALUE = 1\n"`로 바꾸고 `"examples/sensor_geometry.py": "print('example')\n"`을 추가한다. `test_snapshot_includes_named_untracked_source_but_excludes_links_and_secrets`에 `assert "src/forklift_core/__init__.py" in paths`와 `assert "examples/sensor_geometry.py" in paths`를 추가한다. symlink 대상과 변조 대상 경로도 `src/forklift_core/__init__.py`로 바꾼다.
- [ ] **Step 2:** 실행 `python -m pytest tests/integration/test_remote_model_jobs.py -q -p no:cacheprovider -W error` → 기대: 새 assert 2개 실패(allowlist가 `src`·`examples`를 모른다).
- [ ] **Step 3:** `_SOURCE_RULES`에서 `"forklift_core": {".py"}`를 `"src": {".py"}`로 바꾸고 `"examples": {".py"}`를 추가한다.
- [ ] **Step 4 (GREEN):** Step 2 명령 통과. `python tools/submit_model_check.py submit --host X --remote-root /tmp/x --source . --mode model-cpu --python /usr/bin/python3 --dry-run`의 파일 목록에 `src/forklift_core/...` 6개(`__init__`, `_validation`, `geometry`, `sensors/__init__`, `sensors/rgbd`, `sensors/lidar`)와 `examples/sensor_geometry.py`, `tests/unit/...`, `tests/integration/...`가 있고 `forklift_core/`가 없음을 확인한다.

### Task 4: 원격 model job의 실행별 코어 환경

**Files:**
- Modify: `tools/remote_model_job.py` (`build_command`, `execute_job`, 새 `prepare_core_environment`)
- Modify: `tests/integration/test_remote_model_jobs.py` (기존 `execute_job` 호출 6곳에 stub 주입, 실제 환경 준비 시험 1개 추가)

**Interfaces:**
- Produces:

```python
@dataclass(frozen=True)
class CoreEnvironment:
    python: str  # 실행별 venv의 interpreter 절대 경로
    wheel_path: str  # output/wheels/forklift_core-<ver>-py3-none-any.whl
    wheel_sha256: str
    import_path: str  # venv 안에서 확인한 forklift_core.__file__


def prepare_core_environment(
    *,
    python: str,
    source: Path,
    output: Path,
    runtime: Path,
    environment: Mapping[str, str],
    command_runner: CommandRunner,
    commands: list[dict[str, Any]],
) -> CoreEnvironment: ...
```

`execute_job`는 새 키워드 인자 `core_environment_builder: Callable[..., CoreEnvironment] = prepare_core_environment`를 받고, mode가 `model-cpu`·`model-render`이면 **manifest 검증 이후, 기존 try/finally 안에서** 이를 호출해 반환된 `python`으로 `build_command`를 만든다. `job_result.json`에 `"core_environment": {"wheel_path", "wheel_sha256", "import_path", "python", "venv_pip_version", "venv_setuptools_version"}`를 기록한다. gazebo mode는 호출하지 않는다. 준비 단계가 실패해도 기존 실패 경로가 `job_result.json`을 남긴다.

- [ ] **Step 1 (RED):** 실제 준비 시험을 추가한다. fixture는 빌드 가능한 최소 패키지여야 한다.

```python
def _write_buildable_source(source: Path) -> None:
    _write_source(source)
    (source / "pyproject.toml").write_text(
        "[build-system]\nrequires=['setuptools>=61']\n"
        "build-backend='setuptools.build_meta'\n"
        "[project]\nname='forklift-core'\nversion='0.0.1'\n"
        "[tool.setuptools.packages.find]\nwhere=['src']\n"
    )


def test_prepare_core_environment_installs_snapshot_wheel_into_run_venv(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source"
    _write_buildable_source(source)
    for path in [source, *source.rglob("*")]:  # mimic the submitter's chmod -R a-w
        path.chmod(path.stat().st_mode & ~0o222)
    before = _tree_listing(source)
    output = tmp_path / "output"
    runtime = output / ".runtime"
    runtime.mkdir(parents=True)
    commands: list[dict] = []

    env = remote_model_job.prepare_core_environment(
        python=sys.executable,
        source=source,
        output=output,
        runtime=runtime,
        environment=dict(os.environ),
        command_runner=subprocess.run,
        commands=commands,
    )

    assert Path(env.python).is_file() and str(runtime) in env.python
    assert Path(env.wheel_path).is_file() and env.wheel_path.startswith(str(output))
    assert env.import_path.startswith(
        str(runtime)
    )  # not the snapshot, not site-packages of the base interpreter
    assert not (source / "build").exists()  # read-only snapshot untouched
    assert not list(
        source.rglob("*.egg-info")
    )  # no in-tree build artefacts in the snapshot
    assert (
        _tree_listing(source) == before
    )  # contents and modes unchanged (see helper below)
    shutil.rmtree(
        runtime
    )  # the run-time copy must be removable despite the read-only source
    probe = subprocess.run(
        [env.python, "-c", "import forklift_core; print(forklift_core.VALUE)"],
        capture_output=True,
        text=True,
        check=True,
        cwd=tmp_path,
    )
    assert probe.stdout.strip() == "1"
    assert [c["label"] for c in commands] == [
        "core-wheel",
        "core-venv",
        "core-install",
        "core-import-check",
    ]
```

헬퍼 `_tree_listing(root)`는 `sorted((str(p.relative_to(root)), p.stat().st_mode, p.read_bytes() if p.is_file() else None) for p in root.rglob("*"))`를 반환한다. 시험 종료 시 tmp_path 정리를 위해 `finally`에서 source의 쓰기 권한을 되돌린다.

기존 `execute_job` 시험 6곳에는 `core_environment_builder=lambda **kw: remote_model_job.CoreEnvironment(python=sys.executable, wheel_path="", wheel_sha256="", import_path="")`를 넘겨 기존 fake `command_runner` 동작을 유지한다. `job_result.json`에 `core_environment` 키가 남는지 한 곳(`test_manifest_failure_still_writes_a_failed_job_result` 외 성공 경로 시험)에서 확인한다.

- [ ] **Step 2:** 실행 → 기대: `AttributeError: prepare_core_environment` / `TypeError: unexpected keyword core_environment_builder`.
- [ ] **Step 3:** 구현. 순서와 argv는 다음과 같고 모두 `_run_and_record`로 기록한다(label은 위 시험의 4개).
  1. `build_src = runtime / "core-build"`: `source/pyproject.toml`은 `shutil.copyfile`, `source/src/`는 `shutil.copytree(..., copy_function=shutil.copyfile)`로 복사한 뒤 **복사본 전체에** 디렉터리 `0o700` 이상(owner rwx)·파일 owner rw를 `os.chmod`로 보장한다. 제출기가 snapshot에 `chmod -R a-w`를 적용하므로 `copytree` 기본 동작은 디렉터리 쓰기 금지까지 복사해 setuptools의 `src/*.egg-info` 생성과 종료 시 `rmtree`가 실패한다. 빌드는 snapshot이 아닌 이 복사본에서만 한다.
  2. `core-wheel`: `[python, "-m", "pip", "wheel", "--no-deps", "--no-build-isolation", "--no-index", "--wheel-dir", str(output / "wheels"), str(build_src)]`. 결과 wheel은 정확히 1개여야 하며 SHA-256을 계산한다.
  3. `core-venv`: `[python, "-m", "venv", "--system-site-packages", str(runtime / "venv")]`. venv python은 `runtime / "venv" / "bin" / "python"`.
  4. `core-install`: `[venv_python, "-m", "pip", "install", "--no-deps", "--no-index", "--force-reinstall", wheel_path]`. `--force-reinstall`은 base 환경에 같은 배포명·버전이 있을 때 pip가 설치를 생략하는 경우를 막는다(옛 배포명 `forklift-sensor-core`는 이 조건에 해당하지 않지만 검사는 유지). 설치 후 `[venv_python, "-c", "import pip, setuptools, json; print(json.dumps([pip.__version__, setuptools.__version__]))"]`로 venv의 pip·setuptools 버전을 기록한다(원격 venv는 base와 다른 버전을 갖는 것이 확인됨).
  5. `core-import-check`: `[venv_python, "-c", "import forklift_core, json; print(json.dumps(forklift_core.__file__))"]`, cwd는 `runtime`(snapshot이 아니어야 한다). stdout을 파싱해 `import_path`로 저장하고 `str(runtime / "venv")`로 시작하지 않으면 `RuntimeError`.
  어느 단계든 returncode≠0이면 `RuntimeError`를 올리고 `execute_job`의 기존 실패 경로가 `job_result.json`을 남긴다. `wheels/`는 output에 남겨 회수 대상이 되게 하고 `runtime/`은 기존대로 종료 시 삭제한다.
- [ ] **Step 4 (GREEN):** `python -m pytest tests/integration/test_remote_model_jobs.py -q -p no:cacheprovider -W error` 통과. 로컬 end-to-end: `python tools/remote_model_job.py --mode model-cpu --source <manifest 있는 snapshot 사본> --output <새 디렉터리> --python "$(command -v python)"`를 실제 저장소 snapshot(dry-run 목록과 같은 파일을 임시 디렉터리에 복사하고 `.remote-source-manifest.json`을 생성)으로 실행해 `job_result.json`의 `pytest.tests > 0`, `core_environment.import_path`가 venv 경로임을 확인한다. snapshot 생성은 `submit_model_check.discover_snapshot(source).manifest()`를 사용한다.
- [ ] **Step 5:** `docs/development.md` 원격 절에 "model 모드는 snapshot에서 wheel을 만들어 실행별 venv에 설치하며 `job_result.json`의 `core_environment`에 wheel 해시와 import 경로를 남긴다"를 추가한다(문서는 Task 6에서 함께 커밋).

### Task 5: 개발 컨테이너의 editable 설치

**Files:**
- Modify: `deploy/ros2/Dockerfile` 47–56행, `.dockerignore` 2–4행
- Modify: `docs/development.md` 컨테이너 절(Task 6에서 커밋)
- `tools/ros2_dev.sh`는 변경하지 않는다(mount 경로 `/workspace` 유지).

- [ ] **Step 1:** `.dockerignore`의 `!forklift_core/`, `!forklift_core/**`를 `!src/`, `!src/**`로 바꾸고 마지막에 `**/*.egg-info/`를 추가해 호스트 editable 설치 산출물이 `!src/**`로 재포함되지 않게 한다.
- [ ] **Step 2:** Dockerfile의 snapshot 설치 블록을 다음으로 바꾼다.

```dockerfile
# Editable install that points at /workspace/src. The launcher bind-mounts the
# checkout at /workspace, so imports resolve to the live source from any cwd.
# Without the mount, the copy baked below is used (stale snapshot semantics).
COPY pyproject.toml /workspace/pyproject.toml
COPY src /workspace/src
RUN python3 -m venv --system-site-packages /opt/forklift/venv \
    && /opt/forklift/venv/bin/python -m pip install \
        --no-build-isolation --no-deps -e /workspace \
    && chown -R "${DEV_UID}:${DEV_GID}" /workspace /opt/forklift/venv
```

`install -d ... /workspace` 줄은 유지하되 COPY 이후 chown이 소유권을 맞춘다. `WORKDIR /workspace`, `USER forklift-dev`, ENTRYPOINT는 유지. 이 방식은 **기본 editable 모드와 현재의 단순 src 구조**에서 정적 `.pth`가 `/workspace/src`를 가리키는 것에 의존한다(2026-09-11 이미지에서 확인, setuptools 68.1.2). strict 모드는 쓰지 않는다. 설치 metadata(버전·의존성)는 이미지 빌드 시점에 고정되므로 `pyproject.toml`을 바꾸면 이미지를 다시 빌드한다는 문장을 `docs/development.md`에 넣는다.

- [ ] **Step 3:** 이미지 재빌드: `docker build -f deploy/ros2/Dockerfile --build-arg DEV_UID="$(id -u)" --build-arg DEV_GID="$(id -g)" -t forklift/ros2-dev:jazzy .` (기존 태그 덮어씀; 이전 이미지 ID `4fe07bbd…`는 검증 기록에 남아 있다).
- [ ] **Step 4:** 검증 명령 세 개를 모두 통과해야 한다.
  - `bash tools/ros2_dev.sh python -c 'import forklift_core.sensors.rgbd as m; print(m.__file__)'` → `/workspace/src/forklift_core/sensors/rgbd.py`
  - `bash tools/ros2_dev.sh bash -c 'cd /tmp && python -c "import forklift_core; print(forklift_core.__file__)"'` → `/workspace/src/forklift_core/__init__.py` (cwd 무관)
  - `bash tools/ros2_dev.sh python -m pytest tests --ignore=tests/simulation -q -p no:cacheprovider -W error` → 통과, 개수 기록. `bash tools/ros2_dev.sh python examples/sensor_geometry.py` → JSON 출력.
- [ ] **Step 5:** 편집 반영 확인: `git diff > <scratch>/before_probe.diff`로 현재 미커밋 diff를 저장한 뒤, 호스트에서 `src/forklift_core/__init__.py`에 임시로 `PROBE = 1`을 넣고 컨테이너에서 `python -c 'import forklift_core; print(forklift_core.PROBE)'`가 `1`을 출력하면 되돌린다(재빌드 없이 반영됨을 증명). 되돌린 뒤 `git diff`가 저장한 diff와 동일해야 한다.

### Task 6: 문서·지침 갱신과 전환 커밋

**Files:**
- Modify: `README.md`(29·36–38·41행), `CLAUDE.md`·`AGENTS.md`(9행 "not yet migrated" 문단, 17행 목록, 94·96행 명령), `CONTRIBUTING.md`(7행 적용 상태, 71행 옛 배포명 문장, 199행 미래형 문서 이동 문장, §10 머리말에 "2026-09-1x 적용 완료, 표는 이력" 한 줄), `docs/development.md`(3행 root 배치 문장, 21행 Ruff 경로 문장, 호스트 절 명령·컨테이너 절 설명·원격 절 Task 4 문장·새 "새 checkout에서 시작하기" 절), `docs/design/2026-09-10-repository-conventions.md`(5행 "구조 전환 미수행" 문장은 보존하고 전환 완료 기록 링크를 한 줄 추가)

- [ ] **Step 1:** 실행 명령을 바꾼다: `python -m forklift_core.demo` → `python examples/sensor_geometry.py`; 파일 링크 `forklift_core/rgbd.py` → `src/forklift_core/sensors/rgbd.py` 등. README 41행의 "코어 합성 시험 64개" 문장은 유지하되 경로만 갱신.
- [ ] **Step 2:** `CLAUDE.md`/`AGENTS.md` 9행을 "The source uses the `src/` layout from `CONTRIBUTING.md` §1 since 2026-09-1x (`src/forklift_core/`, `examples/`, `tests/unit|integration/`). Install with `python -m pip install -e '.[dev]'` before running anything; do not rely on the checkout directory being on `sys.path`. Keep this section identical in both instruction files."로 바꾼다. 17행 목록의 `forklift_core/`를 `src/forklift_core/`로, 94행 명령의 demo를 예제 경로로 바꾼다.
- [ ] **Step 3:** `docs/development.md`에 "새 checkout에서 시작하기" 절을 추가한다: clone → Python ≥ 3.10 환경에서 `python -m pip install -e '.[dev]'` → 코어 명령 두 개 → `python tools/submit_model_check.py submit --host <SSH_HOST> --remote-root '<REMOTE_PROJECT_ROOT>' --source . --mode model-cpu --python '<REMOTE_PYTHON>' --dry-run`(SSH 없이 계획만 출력) → 원격 사용 권한은 팀 계정 배정 후 별도 확인이라는 경계. 컨테이너 절의 "checkout이 snapshot보다 먼저 import" 설명을 Task 5의 editable 방식으로 바꾼다.
- [ ] **Step 4:** `diff <(tail -n +4 AGENTS.md) <(tail -n +4 CLAUDE.md)` 빈 출력, 링크 검사 0개. Markdown 링크 검사는 backtick 경로를 못 잡으므로 `git grep -n -E 'forklift_core/(rgbd|lidar|demo|geometry)\.py|forklift_core\.demo|forklift-sensor-core|docs/LOCAL_VALIDATION|docs/superpowers' -- . ':!docs/validation' ':!docs/plans/2026-09-10*' ':!docs/design/2026-09-10*'`가 비어 있어야 한다(날짜 기록은 제외).
- [ ] **Step 5:** 전환 커밋(Task 2–6 전체, 파일 단위 stage): `git commit -m "refactor(core): migrate to src layout with examples and per-run remote install"`. 본문에 이동 표와 숨은 의존 4곳의 처리 방법을 적는다.

### Task 7: 동등성 검증과 기록

**Files:**
- Create: `docs/validation/2026-09-1x-src-layout-migration.md`
- Modify: `docs/validation/2026-09-11-development-checkpoint.md`(저장소 구조 행), `docs/plans/2026-09-11-development-roadmap.md`(§8 M1 구조 전환 체크박스)

- [ ] **Step 1 (호스트):** 전체 회귀 `python -m pytest tests ros2/src/forklift_ros/test -m 'not rendering' -q -p no:cacheprovider -W error`, `python -m ruff check .`, `python -m ruff format --check .`. 링크 검사:

```python
import re, pathlib

root = pathlib.Path(".")
bad = []
for md in sorted(
    [
        *root.glob("*.md"),
        *root.glob("docs/**/*.md"),
        *root.glob("sim/**/*.md"),
        *root.glob("requirements/*.md"),
    ]
):
    if "presentation" in md.parts or "artifacts" in md.parts:
        continue
    for m in re.finditer(r"\[[^\]]*\]\(([^)]+)\)", md.read_text(encoding="utf-8")):
        t = m.group(1).split("#")[0].strip()
        if (
            t
            and not t.startswith(("http://", "https://", "mailto:"))
            and not (md.parent / t).resolve().exists()
        ):
            bad.append((str(md), t))
print(len(bad), bad)
```

- [ ] **Step 2 (wheel):** `python -m build --wheel --outdir <scratch>/wheels` → `forklift_core-0.1.0-py3-none-any.whl`. 깨끗한 Python 3.11 venv와 `python:3.10-slim`(digest `sha256:fd76ade0…`) 컨테이너 venv 각각에서 `pip install <wheel> pytest numpy`, source 밖 cwd에서 `python -c 'import forklift_core.sensors.lidar, forklift_core.geometry; print(forklift_core.__file__)'`, `pip check`, `python <checkout>/examples/sensor_geometry.py`, `python -m pytest <checkout>/tests/unit --import-mode=importlib -q -p no:cacheprovider -W error` 통과. wheel SHA-256 기록.
- [ ] **Step 3 (컨테이너):** Task 5 Step 4의 세 명령과 `bash tools/ros2_dev.sh python -m pytest ros2/src/forklift_ros/test -q -p no:cacheprovider -W error`(28개) 결과, 새 이미지 ID를 기록.
- [ ] **Step 4 (원격 model-cpu):** `python tools/submit_model_check.py submit --host <SSH_HOST> --remote-root '<REMOTE_PROJECT_ROOT>' --source . --mode model-cpu --python '<REMOTE_PYTHON>' --wait --output artifacts/<run-id>`. 확인: Slurm `COMPLETED 0:0`, `job_result.json`의 `exit_code 0`, `pytest.tests`가 호스트의 `-m 'not rendering'` 코어+도구+시뮬레이션 개수와 일치, `core_environment.import_path`가 원격 `.runtime/venv` 경로, `wheels/*.whl` 회수와 해시 일치.
- [ ] **Step 5 (원격 gazebo):** `... --mode gazebo --image forklift/gazebo:jazzy-harmonic --duration 30 --wait`. 확인: `COMPLETED 0:0`, job 결과 0, colcon 시험 28 passed·skip 0, live/저장 bag/새 프로세스 replay 통과, 회수 파일 해시 일치. 메시지 수는 계약 기준(각 스트림 ≥ 150개·≥ 30초)으로 판정하며 과거 161/162에 고정하지 않는다.
- [ ] **Step 6:** 검증 기록을 쓴다: 실행 ID·Slurm job·명령·실제 개수·이미지 ID·wheel 해시·snapshot SHA-256·원격 venv pip/setuptools 버전, 그리고 "구조 전환은 기능 변경이 아니며 실물·인식·주행 검증이 아니다"라는 경계. 체크포인트의 "저장소 구조" 행을 갱신하고, 로드맵 §8의 M1 항목은 **구조 전환 부분만** 완료로 표시한다(합성 장면 100개 고정은 M1-b로 남는다).
- [ ] **Step 7:** 커밋 `docs(validation): record src layout migration checks`, `git switch main && git merge --ff-only refactor/src-layout`, 사용자 승인 범위에서 `git push origin main`과 원격 `repo/` 갱신(bundle → `git pull --ff-only`).

## 자체 검토

- **범위 대조:** §10 표 12행 전부 Task 1·2·6에 배정. Codex 지적 4곳: 원격 무설치 import → Task 4, allowlist → Task 3, 컨테이너 cwd import → Task 5, 예제 cwd → Task 2 Step 1. 계획 검토 수정 2건: 읽기 전용 snapshot 복사본 권한 → Task 4 Step 1·3, 루트 `__init__.py` 이동 → Task 2 Files. 로드맵 M1 완료 조건(동작 보존·wheel source 밖 import·새 snapshot 원격 실행) → Task 7.
- **자리표시자:** 코드 블록은 실제 내용이며 "나중에", "적절히"류 표현 없음. `_validation.py` 내용은 기존 함수 3개의 바이트 이동이므로 코드를 반복 게재하지 않는다.
- **이름 일관성:** `prepare_core_environment`·`CoreEnvironment`·`core_environment_builder`·`core_environment` 키·label 4개가 Task 4 시험과 구현에서 동일하다. 모듈 경로 `forklift_core.sensors.rgbd/lidar`, `forklift_core._validation`, `examples/sensor_geometry.py`는 Task 2·5·6·7에서 동일하다.
- **미포함(의도):** `forklift_core.rgbd`/`lidar` 호환 shim(사용처 없음), `pytest pythonpath` 설정(editable 검증 목적과 충돌), `tests/unit/__init__.py`(basename 충돌 없음), `requirements/model_py311.txt` 재생성(프로젝트 wheel과 무관), `physics_probe.py` 승격(별도 소작업).
