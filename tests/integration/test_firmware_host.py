"""Host build of the hardware-independent firmware core (firmware/) and its C tests.

Builds with CMake into a temporary directory; skipped (not passed) when CMake or a
C compiler is missing.
"""

import shutil
import subprocess
from pathlib import Path

import pytest

FIRMWARE = Path(__file__).resolve().parents[2] / "firmware"


@pytest.fixture(scope="session")
def firmware_build(tmp_path_factory) -> Path:
    if shutil.which("cmake") is None:
        pytest.skip("cmake not installed")
    if shutil.which("cc") is None and shutil.which("gcc") is None:
        pytest.skip("no C compiler")
    build = tmp_path_factory.mktemp("firmware_build")
    result = subprocess.run(
        ["cmake", "-S", str(FIRMWARE), "-B", str(build), "-DCMAKE_BUILD_TYPE=Debug"],
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    result = subprocess.run(
        ["cmake", "--build", str(build), "-j2"], capture_output=True, text=True
    )
    assert result.returncode == 0, result.stdout + result.stderr
    return build


def test_c_core_unit_tests_pass(firmware_build):
    result = subprocess.run(
        ["ctest", "--output-on-failure"],
        cwd=firmware_build,
        capture_output=True,
        text=True,
    )
    assert result.returncode == 0, result.stdout + result.stderr
