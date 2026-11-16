"""Path-loaded synthetic scenes shared by unit and standalone integration runs."""

import importlib.util
from pathlib import Path

import pytest

_SPEC = importlib.util.spec_from_file_location(
    "synthetic_scene", Path(__file__).parent / "fixtures" / "synthetic_scene.py"
)
_SCENE = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(_SCENE)


@pytest.fixture
def pallet_scene():
    return _SCENE.make_pallet_scene


@pytest.fixture
def opening_ray_fractions():
    return _SCENE.opening_ray_fractions


_PALLET_SPEC = importlib.util.spec_from_file_location(
    "pallet_urdf_fixture", Path(__file__).parent / "unit" / "pallet_urdf_fixture.py"
)
_PALLET = importlib.util.module_from_spec(_PALLET_SPEC)
_PALLET_SPEC.loader.exec_module(_PALLET)


@pytest.fixture
def synthetic_pallet_urdf(tmp_path):
    def write(**kwargs):
        return _PALLET.write_pallet_urdf(tmp_path / "pallet.urdf", **kwargs)

    return write


@pytest.fixture
def full_t11_pallet_urdf(tmp_path):
    def write(**kwargs):
        return _PALLET.write_full_t11_pallet_urdf(tmp_path / "full_t11.urdf", **kwargs)

    return write


# --- the teleop end-to-end harness's evidence (tests/integration/test_teleop_bridge.py Run.to_ready) ----------------
# Here, in the root conftest, so the xdist controller loads it whatever the run's arguments are (with "tests" a conftest
# under tests/integration is only loaded by the workers, and the summary never printed: ws1 2026-11-16). Workers append
# one JSON line per to_ready to the file named in the environment; the summary prints them into the run's log.
_LOOP_ENV = "FORKLIFT_LOOP_EVIDENCE"


# Real time (lead/user 2026-11-16): a test whose fixtures start the firmware core's MCU model (the ``mcu_model``
# fixture, directly or through a bench, a run or the bridge harness) drives it as a live peer over a pty on the wall
# clock -- STATE ages, link deadlines, the bridge's 20 ms loop -- and fails when xdist workers starve the process. Those
# are marked ``realtime`` here, by fixture and not by file, so the synthetic and fake-clock tests beside them stay in the
# parallel stage; the ws1 wrapper runs ``-m realtime`` alone after it.
REALTIME_FIXTURES = frozenset({"mcu_model"})


def pytest_collection_modifyitems(config, items):
    for item in items:
        if REALTIME_FIXTURES.intersection(getattr(item, "fixturenames", ())):
            item.add_marker(pytest.mark.realtime)


def pytest_configure(config):
    import os
    import tempfile

    if _LOOP_ENV not in os.environ and not hasattr(config, "workerinput"):  # the controller; workers inherit it
        fd, path = tempfile.mkstemp(prefix="forklift_loop_evidence_", suffix=".jsonl")
        os.close(fd)
        os.environ[_LOOP_ENV] = path


def pytest_terminal_summary(terminalreporter, exitstatus, config):
    import os

    path = os.environ.get(_LOOP_ENV)
    if not path or hasattr(config, "workerinput") or not Path(path).exists():
        return
    lines = [x for x in Path(path).read_text(encoding="utf-8").splitlines() if x.strip()]
    if lines:
        terminalreporter.section("teleop harness evidence (to_ready)")
        for line in lines:
            terminalreporter.write_line("LOOP_EVIDENCE " + line)
