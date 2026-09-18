"""Synthetic envelope only; not a T11 internal-layout or contact fixture."""

from pathlib import Path


def write_pallet_urdf(
    path: Path,
    *,
    x_m: float = 0,
    z_m: float = 0.045,
    depth_m: float = 0.66,
    width_m: float = 0.66,
) -> Path:
    path.write_text(f'''<robot name="synthetic"><link name="pallet">
<collision name="envelope"><origin xyz="{x_m} 0 {z_m}"/>
<geometry><box size="{depth_m} {width_m} 0.09"/></geometry>
</collision></link></robot>''')
    return path
