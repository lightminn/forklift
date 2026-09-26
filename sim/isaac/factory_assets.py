"""Which Simple_Warehouse props fill the factory hall, by role; no Isaac imports.

The Isaac adapter measures every asset's bounds at run time (`read_catalogue`)
and uses those. MEASURED_DIMENSIONS_M is the same measurement taken offline
with usd-core on 2026-09-26 from the Isaac 5.1 S3 copies, for CPU tools that
cannot open the stage. A run records its own measured values; if the two ever
disagree, the run's values are the ones that were used.
"""

from forklift_core.planning.factory_layout import FactoryAssets
from forklift_core.planning.pallet_mission import AssetSpec

PALLET = "SM_PaletteA_01.usd"
# The smallest box (SM_CardBoxD_05, 0.15 m) would need up to ten layers per
# pallet at the configured load heights; two box sizes keep the prop count sane.
LOADS = ("SM_CardBoxA_02.usd", "SM_CardBoxC_01.usd")
CLUTTER = (
    "SM_BarelPlastic_A_01.usd",
    "SM_BarelPlastic_C_01.usd",
    "SM_CratePlastic_D_01.usd",
    "SM_PushcartA_02.usd",
    "S_TrafficCone.usd",
    "SM_RackPile_04.usd",
)
ALL = (PALLET, *LOADS, *CLUTTER)

# (length x, width y, height z) of the world-aligned bounds, metres, at full
# precision: for the three bay props these equal the values Isaac recorded in
# artifacts/20260923_return_home_v2 bit for bit. Rounding them is not harmless
# -- at 4 decimals seed 1's bay approach flips from success to expansion_limit.
MEASURED_DIMENSIONS_M = {
    "SM_BarelPlastic_A_01.usd": (
        0.5999998721480395,
        0.7080822595637528,
        0.9013595379585126,
    ),
    "SM_CratePlastic_D_01.usd": (
        0.449549798453976,
        0.6620987553425408,
        0.1880701023026532,
    ),
    "SM_CardBoxA_02.usd": (0.7970254338452492, 0.6365090800111943, 0.5032872468988465),
    "SM_PaletteA_01.usd": (1.213234950604253, 1.0028731312705617, 0.21111953263462624),
    "SM_CardBoxC_01.usd": (0.4999999888241291, 0.4999999888241291, 0.24999999441206455),
    "SM_BarelPlastic_C_01.usd": (
        0.42125208866777797,
        0.42125205052080616,
        0.5118258171120473,
    ),
    "SM_PushcartA_02.usd": (0.9357002812094919, 1.6481130613063328, 0.3774050056024265),
    "S_TrafficCone.usd": (0.33569705212793366, 0.3339443704349314, 0.46281219210895624),
    "SM_RackPile_04.usd": (0.936802309077926, 0.9537060333705085, 0.2999993805840632),
}


def factory_assets(specs_by_filename: dict[str, AssetSpec]) -> FactoryAssets:
    """Group measured specs by role; a missing file raises KeyError."""
    return FactoryAssets(
        specs_by_filename[PALLET],
        tuple(specs_by_filename[name] for name in LOADS),
        tuple(specs_by_filename[name] for name in CLUTTER),
    )


def offline_specs(asset_root: str) -> dict[str, AssetSpec]:
    """Specs from the offline measurement, with URIs under asset_root."""
    return {
        name: AssetSpec(asset_root.rstrip("/") + "/" + name, *dimensions)
        for name, dimensions in MEASURED_DIMENSIONS_M.items()
    }
