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
