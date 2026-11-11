"""The firmware vehicle header is generated from config/real/vehicle.yaml (plan P0-6)."""

import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from forklift_core.plant.real_vehicle import (
    load_real_vehicle,
    real_vehicle_from_mapping,
)

REPO = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(REPO / "tools"))
import real_vehicle_header as gen  # noqa: E402
from test_real_vehicle import CHASSIS, PROV, mapping  # noqa: E402

VEHICLE_YAML = REPO / "config" / "real" / "vehicle.yaml"


def define(text, name):
    m = re.search(rf"^#define {name} (.+)$", text, re.M)
    assert m, name
    return m.group(1)


def test_header_carries_the_rounded_counts_limits_and_calibration_id():
    cfg = load_real_vehicle(VEHICLE_YAML)
    text = gen.render_header(cfg)
    assert define(text, "FW_WHEEL_COUNTS_PER_REV_L") == "327680u"
    assert define(text, "FW_WHEEL_COUNTS_PER_REV_R") == "327680u"
    assert define(text, "FW_MAX_WHEEL_RAD_S") == "8.0f"
    assert float(define(text, "FW_STEER_MAX_RAD").rstrip("f")) == pytest.approx(cfg.steering.angle_range[1], abs=1e-9)
    assert float(define(text, "FW_STEER_MIN_RAD").rstrip("f")) == pytest.approx(cfg.steering.angle_range[0], abs=1e-9)
    assert define(text, "FW_VEHICLE_CALIBRATION_ID") == f'"{cfg.calibration_id}"'
    assert define(text, "FW_VEHICLE_CALIBRATION_ID_U32") == f"0x{cfg.calibration_id[:8]}u"
    assert "user_estimate_2026_11_11" in text and "measured: false" in text
    assert gen.render_header(cfg) == text  # deterministic


def test_fractional_counts_are_rounded_with_the_error_stated():
    counts = {side: {"counts_per_wheel_rev": 326041.6, "provenance": PROV} for side in ("left", "right")}
    text = gen.render_header(real_vehicle_from_mapping(mapping(wheel_counts=counts), CHASSIS))
    assert define(text, "FW_WHEEL_COUNTS_PER_REV_L") == "326042u"
    assert "1.23 ppm" in text


@pytest.mark.parametrize("value", [0.4, 2.0**32, 10000.4])  # the last: in range, 4e-5 rounding
def test_counts_outside_uint32_or_too_coarse_are_refused(value):
    counts = {side: {"counts_per_wheel_rev": value, "provenance": PROV} for side in ("left", "right")}
    with pytest.raises(ValueError):
        gen.render_header(real_vehicle_from_mapping(mapping(wheel_counts=counts), CHASSIS))


def test_cli_writes_the_header(tmp_path):
    out = tmp_path / "vehicle_config.h"
    assert gen.main(["--config", str(VEHICLE_YAML), "--out", str(out)]) == 0
    assert out.read_text() == gen.render_header(load_real_vehicle(VEHICLE_YAML))


def test_generated_header_drives_the_core_wheel_encoder(tmp_path):
    """One wheel revolution in 1 s through wheel_enc with the generated counts reads 2*pi rad/s."""
    cc = shutil.which("cc") or shutil.which("gcc")
    if cc is None:
        pytest.skip("no C compiler")
    (tmp_path / "vehicle_config.h").write_text(gen.render_header(load_real_vehicle(VEHICLE_YAML)))
    (tmp_path / "main.c").write_text(r"""
#include <math.h>
#include <stdio.h>
#include "fw/wheel_enc.h"
#include "vehicle_config.h"
int main(void) {
    wheel_enc_config c = {FW_WHEEL_COUNTS_PER_REV_L, 10, 1};
    wheel_enc e;
    if (!wheel_enc_init(&e, &c, 0, 0)) return 2;
    uint32_t total = 0; uint16_t hw = 0;
    for (int k = 1; k <= 1000; k++) {           /* 1 kHz for 1 s */
        uint32_t step = (uint32_t)((uint64_t)FW_WHEEL_COUNTS_PER_REV_L * (uint64_t)k / 1000u) - total;
        total += step; hw = (uint16_t)(hw + step);
        wheel_enc_update(&e, &c, hw, (uint32_t)k * 1000u);
    }
    printf("%.6f %u\n", (double)e.rate_rad_s, (unsigned)e.count);
    return fabs((double)e.rate_rad_s - 6.283185307) < 1e-3 && e.count == FW_WHEEL_COUNTS_PER_REV_L ? 0 : 1;
}
""")
    fw = REPO / "firmware" / "core"
    exe = tmp_path / "t"
    build = subprocess.run([cc, "-std=c11", "-Wall", "-Werror", "-I", str(fw / "include"), "-I", str(tmp_path),
                            str(tmp_path / "main.c"), str(fw / "src" / "wheel_enc.c"), "-lm", "-o", str(exe)],
                           capture_output=True, text=True)
    assert build.returncode == 0, build.stderr
    run = subprocess.run([str(exe)], capture_output=True, text=True)
    assert run.returncode == 0, run.stdout + run.stderr


def test_print_chassis_resolves_relative_quoted_and_absolute_paths(tmp_path, capsys):
    chassis = REPO / "sim" / "models" / "dls08_measured" / "parameters.yaml"
    text = VEHICLE_YAML.read_text()
    for value in ('"../../sim/models/dls08_measured/parameters.yaml"', str(chassis)):
        cfg = tmp_path / "config" / "real" / "vehicle.yaml"
        cfg.parent.mkdir(parents=True, exist_ok=True)
        cfg.write_text(text.replace("chassis_model: ../../sim/models/dls08_measured/parameters.yaml",
                                    f"chassis_model: {value}"))
        if value.startswith('"'):
            (tmp_path / "sim" / "models" / "dls08_measured").mkdir(parents=True, exist_ok=True)
            (tmp_path / "sim" / "models" / "dls08_measured" / "parameters.yaml").write_text(chassis.read_text())
        assert gen.main(["--config", str(cfg), "--print-chassis"]) == 0
        printed = Path(capsys.readouterr().out.strip())
        assert printed.is_absolute() and printed.exists() and printed.name == "parameters.yaml"
