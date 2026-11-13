"""The loaded transport curvature cap rule (docs/plans/2026-11-14-loaded-curvature-rule.md 2-3).

The loaded transport phase plans with curvature k x kappa_sat, where kappa_sat is the saturated-curvature reference:
the smaller of the per-direction medians of the observed curvature in forward loaded saturated samples (the current
plant's value from docs/validation/2026-11-14-left-steer-shortfall/left_steer_table.json; a real truck's from the
arrival-day M2). The rule file holds k, kappa_sat and where kappa_sat came from; the runner refuses a rule whose
curvature the tracker or the model cannot command.
"""

from __future__ import annotations

import math
from dataclasses import replace
from pathlib import Path

REQUIRED = ("k", "saturated_curvature_inv_m", "source")


def load_rule(path) -> dict:
    """The rule file (YAML): ``k`` in (0, 1], ``saturated_curvature_inv_m`` > 0, a ``source`` mapping. ValueError
    otherwise."""
    import yaml

    data = yaml.safe_load(Path(path).read_text())
    if not isinstance(data, dict) or any(key not in data for key in REQUIRED):
        raise ValueError(f"--loaded-curvature-rule {path}: needs {', '.join(REQUIRED)}")
    k, kappa = data["k"], data["saturated_curvature_inv_m"]
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in (k, kappa)):
        raise ValueError(f"--loaded-curvature-rule {path}: k and saturated_curvature_inv_m must be finite numbers")
    if not 0.0 < float(k) <= 1.0:
        raise ValueError(f"--loaded-curvature-rule {path}: k must be in (0, 1], got {k}")
    if not float(kappa) > 0.0:
        raise ValueError(f"--loaded-curvature-rule {path}: saturated_curvature_inv_m must be positive, got {kappa}")
    if not isinstance(data["source"], dict) or not data["source"]:
        raise ValueError(f"--loaded-curvature-rule {path}: source must say where kappa_sat came from")
    return {"k": float(k), "saturated_curvature_inv_m": float(kappa), "source": dict(data["source"])}


def planned_curvature(rule: dict, tracker_limit_inv_m: float, kinematic_limit_inv_m: float) -> float:
    """kappa_plan = k x kappa_sat, refused (ValueError) above the tracker's command limit or the model's kinematic
    limit."""
    kappa = rule["k"] * rule["saturated_curvature_inv_m"]
    if kappa > tracker_limit_inv_m:
        raise ValueError(f"loaded curvature {kappa:.4f} 1/m exceeds the tracker's limit {tracker_limit_inv_m}")
    if kappa > kinematic_limit_inv_m:
        raise ValueError(f"loaded curvature {kappa:.4f} 1/m exceeds the model's steering limit {kinematic_limit_inv_m:.4f}")
    return kappa


def loaded_configs(planner_config, travel_config, kappa_inv_m: float):
    """The (planner, travel) configs for paths driven loaded in transport: the run's own with only the curvature limit
    replaced (travel None stays None)."""
    return (replace(planner_config, curvature_limit_inv_m=float(kappa_inv_m)),
            None if travel_config is None else replace(travel_config, curvature_limit_inv_m=float(kappa_inv_m)))


__all__ = ["load_rule", "planned_curvature", "loaded_configs"]
