# src/forklift_core/perception/fork_fit.py
"""Fork-fit truth and decision with reason codes (design §3.1, §9.2, §9.3).

Margins are measured from the real blade edges at the configured spacing, with
the fork centre on the chosen opening pair's midpoint. The support check is a
reach/footprint precheck against an instructed centre of mass, not a support
polygon or contact verification (ADR 0004 D3).
"""

from dataclasses import asdict, dataclass
from pathlib import Path

from forklift_core._validation import _finite_scalar
from forklift_core.perception.pallet_geometry import target_insertion_depth_m
from forklift_core.perception.pallet_model import PalletSpec, face_layout, pair_margin

M_REQ_PROVISIONAL_M = 0.066
TRACKING_ALLOWANCE_M = 0.0338
TIP_LEVER_M = 0.33
AXIAL_DANGER_M = 0.040
SUPPORT_LONGITUDINAL_MARGIN_M = 0.036
SUPPORT_LATERAL_MARGIN_M = 0.020
VERTICAL_MARGIN_M = 0.005
PAYLOAD_LIMIT_KG = 10.0
REASON_CODES = ("fork_spacing_adjust", "lift_alignment_needed", "support_unverified",
                "payload_limit", "near_tracking_unavailable")
TEST_FORK_SPACING_M = {"epal6": 0.3725, "t11_06": 0.290}


@dataclass(frozen=True)
class ForkSetup:
    spacing_m: float
    blade_width_m: float = 0.055
    blade_thickness_m: float = 0.025
    lowest_centre_m: float = 0.040
    carriage_limit_m: float = 0.346
    reserve_m: float = 0.016
    lift_travel_m: float = 0.28

    @property
    def blade_top_m(self) -> float:
        return self.lowest_centre_m + self.blade_thickness_m / 2

    @property
    def blade_bottom_m(self) -> float:
        return self.lowest_centre_m - self.blade_thickness_m / 2


LOAD_SOURCES = ("instruction", "site_default")


@dataclass(frozen=True)
class LoadDeclaration:
    pallet_mass_kg: float
    load_mass_kg: float
    com_x_max_m: float
    com_y_max_m: float
    source: str = "instruction"

    def __post_init__(self) -> None:
        for name in ("pallet_mass_kg", "load_mass_kg", "com_x_max_m", "com_y_max_m"):
            if _finite_scalar(getattr(self, name), name) < 0:
                raise ValueError(f"{name} must be finite and nonnegative")
        if self.pallet_mass_kg <= 0:
            raise ValueError("pallet_mass_kg must be positive")
        if self.source not in LOAD_SOURCES:
            raise ValueError(f"source must be one of {LOAD_SOURCES}")

    def to_json(self) -> dict:
        return asdict(self)


@dataclass(frozen=True)
class FitResult:
    pickable: bool
    reasons: tuple
    axis: str
    pair: tuple
    min_lateral_margin_m: float
    fork_centre_offset_m: float
    insertion_target_m: float
    load_source: str | None

    def to_json(self) -> dict:
        obj = asdict(self)
        obj["reasons"], obj["pair"] = list(self.reasons), list(self.pair)
        return obj


def load_site_defaults(path, specs: dict) -> dict:
    """Site default load declarations for known shapes (a site policy, not a measurement)."""
    import yaml

    obj = yaml.safe_load(Path(path).read_text())
    out = {}
    for name, entry in obj["shapes"].items():
        if name not in specs:
            continue
        if entry.get("com") != "centred":
            raise ValueError("site defaults support only a centred centre of mass")
        out[name] = LoadDeclaration(entry["pallet_mass_kg"], entry["load_mass_kg"],
                                    specs[name].overall_depth_m / 2, 0.0, "site_default")
    return out


def _lane(spec, axis, support_z_m):
    layout = face_layout(spec, axis)
    return layout, support_z_m + layout.lane_floor_m, support_z_m + layout.lane_ceiling_m


def lane_reachable(spec: PalletSpec, axis: str, setup: ForkSetup, *, support_z_m: float) -> bool:
    """Some lift in [0, travel] puts both blades inside the lane with margins."""
    _, floor, ceiling = _lane(spec, axis, support_z_m)
    lift = max(0.0, floor + VERTICAL_MARGIN_M - setup.blade_bottom_m)
    return lift <= setup.lift_travel_m + 1e-12 and setup.blade_top_m + lift + VERTICAL_MARGIN_M <= ceiling + 1e-12


def _pair_reasons(spec, setup, load, margin, centre, target, m_req_m) -> set:
    reasons = set()
    if margin < m_req_m:
        reasons.add("fork_spacing_adjust")
    if not spec.known and (load is None or target < load.com_x_max_m + SUPPORT_LONGITUDINAL_MARGIN_M
                           or abs(centre) + load.com_y_max_m > setup.spacing_m / 2 - SUPPORT_LATERAL_MARGIN_M):
        reasons.add("support_unverified")
    return reasons


def fork_fit(spec, axis, setup, load, *, support_z_m: float = 0.0, m_req_m: float = M_REQ_PROVISIONAL_M) -> FitResult:
    layout, floor, ceiling = _lane(spec, axis, support_z_m)
    target = target_insertion_depth_m(layout.depth_m, setup.carriage_limit_m, setup.reserve_m)
    n = len(layout.opening_intervals_m)
    if n < 2:
        raise ValueError("A face needs two openings")
    options = []
    for i in range(n):
        for j in range(i + 1, n):
            margin, centre = pair_margin(layout, (i, j), setup.spacing_m, setup.blade_width_m)
            options.append((-round(margin, 9), round(abs(centre), 9), (i, j), margin, centre))
    options.sort()
    chosen = next((o for o in options if not _pair_reasons(spec, setup, load, o[3], o[4], target, m_req_m)), options[0])
    _, _, pair, margin, centre = chosen
    reasons = _pair_reasons(spec, setup, load, margin, centre, target, m_req_m)
    if (floor > setup.blade_bottom_m - VERTICAL_MARGIN_M + 1e-12
            or ceiling < setup.blade_top_m + VERTICAL_MARGIN_M - 1e-12):
        reasons.add("lift_alignment_needed")
    load_source = None if load is None else load.source
    if load is None:
        reasons.add("support_unverified")  # mass unknown: confirm on the console
    elif load.pallet_mass_kg + load.load_mass_kg > PAYLOAD_LIMIT_KG:
        reasons.add("payload_limit")
    if not spec.known and spec.top_style != "boards":
        reasons.add("near_tracking_unavailable")
    ordered = tuple(code for code in REASON_CODES if code in reasons)
    return FitResult(not ordered, ordered, axis, pair, margin, centre, target, load_source)


def danger_threshold_m(fit: FitResult) -> float:
    """T_i (design §3.4): the blade-tip lateral displacement that reaches contact."""
    return fit.min_lateral_margin_m - TRACKING_ALLOWANCE_M
