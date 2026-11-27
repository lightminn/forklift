"""U3e-2 (Codex round 2 P2-3): an engineering timing bound needs a decision id that is not blank."""

from pathlib import Path

import pytest
import yaml

from forklift_core.sensors.lidar_cfg import APPLIES_KEYS, lidar_config_from

REPO = Path(__file__).resolve().parents[3]


def engineering(decision):
    d = yaml.safe_load((REPO / "config/real/lidar.yaml").read_text())
    d["timing"].update(first_beam_err_s=0.002, basis="engineering", decision_id=decision, basis_doc="docs/bound.md",
                       applies_to={k: "x" for k in APPLIES_KEYS})
    return d


@pytest.mark.parametrize("blank", ["", "  ", "\t"])
def test_a_blank_decision_is_none(blank):
    with pytest.raises(ValueError, match="decision_id"):
        lidar_config_from(engineering(blank))
