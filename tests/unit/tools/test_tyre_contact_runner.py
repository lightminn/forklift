"""--tyre-contact in the runner (user decision 2026-11-18: the F2 compliant tyre contact on by default in the launchers,
the runner's default off; docs/validation/2026-11-18-f2-i-reregistration.md)."""

import argparse
import sys
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim" / "isaac"))

import near_tracking  # noqa: E402
import run_transport  # noqa: E402

RUNNER = (ROOT / "sim/isaac/run_transport.py").read_text()
MEASURED = yaml.safe_load((ROOT / "config/isaac_transport_measured.yaml").read_text())


def test_f2_puts_the_registered_values_onto_the_settings():
    out = run_transport.apply_tyre_contact(MEASURED, "f2")
    assert out["tyre_contact_stiffness_n_per_m"] == 140000.0 and out["tyre_contact_damping_n_s_per_m"] == 108.5
    assert {k: v for k, v in out.items() if not k.startswith("tyre_contact_")} == MEASURED
    assert run_transport.apply_tyre_contact(MEASURED, "off") is MEASURED  # off: the settings as they were


def test_f2_is_refused_over_a_contact_the_settings_set_and_outside_120_hz():
    for key in run_transport.TYRE_CONTACT_KEYS:
        with pytest.raises(ValueError, match="already set"):
            run_transport.apply_tyre_contact({**MEASURED, key: 1.0}, "f2")
    with pytest.raises(ValueError, match="120 Hz"):
        run_transport.apply_tyre_contact({**MEASURED, "physics_hz": 240}, "f2")
    with pytest.raises(ValueError):
        run_transport.apply_tyre_contact(MEASURED, "soft")


def test_the_measured_settings_set_no_tyre_contact_and_run_at_120_hz():
    assert not any(k in MEASURED for k in run_transport.TYRE_CONTACT_KEYS) and MEASURED["physics_hz"] == 120


def test_off_is_not_recorded_f2_is():
    assert "tyre_contact" not in run_transport.recorded_arguments(argparse.Namespace(tyre_contact="off", seed=1))
    assert run_transport.recorded_arguments(argparse.Namespace(tyre_contact="f2", seed=1))["tyre_contact"] == "f2"


def test_the_near_contract_was_measured_rigid():
    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["tyre_contact"] == "off"


def test_the_runner_wiring():
    assert '"--tyre-contact"' in RUNNER and 'default="off"' in RUNNER
    assert 'settings = apply_tyre_contact(settings, getattr(args, "tyre_contact", "off"))' in RUNNER
