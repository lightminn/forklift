import importlib.util, math, sys
import numpy as np
spec = importlib.util.spec_from_file_location("old_permit", sys.argv[1]); old = importlib.util.module_from_spec(spec); sys.modules["old_permit"] = old; spec.loader.exec_module(old)
from forklift_core.safety import pallet_permit as new
from forklift_core.planning.geometry import Footprint
rng = np.random.default_rng(7)
hull = Footprint(1.29, 0.17, 0.36)
parts = [(Footprint(0.944, 0.17, 0.36), 0.0, 0.0), (Footprint(0.18, 0.18, 0.05), 0.2, 0.9), (Footprint(0.18, 0.18, 0.05), -0.2, 0.9)]
n = bad = 0
yaws = [0.0, math.pi / 2, math.pi, -math.pi / 2, math.pi / 4, 3 * math.pi / 4, math.pi / 6]
for k in range(30000):
    shape = hull if k % 2 else parts
    yaw = float(rng.choice(yaws)); x, y = float(rng.integers(-40, 40)) / 8, float(rng.integers(-40, 40)) / 8
    zones = []
    for j in range(int(rng.integers(1, 6))):
        zy = float(rng.choice(yaws))
        # touching: a box whose edge lies exactly on the outline's, or a random one
        L, W = float(rng.integers(1, 24)) / 8, float(rng.integers(1, 24)) / 8
        if k % 3 == 0:
            c, s = math.cos(yaw), math.sin(yaw)
            gap = 0.36 + W / 2
            zones.append(new.Zone(f"z{j}", x - s * gap * (1 if j % 2 else -1), y + c * gap * (1 if j % 2 else -1), L, W, yaw))
        else:
            zones.append(new.Zone(f"z{j}", x + float(rng.integers(-24, 24)) / 8, y + float(rng.integers(-24, 24)) / 8, L, W, zy))
    zo = [old.Zone(z.zone_id, z.x_m, z.y_m, z.length_m, z.width_m, z.yaw_rad) for z in zones]
    pose = (x, y, yaw)
    a, b = old.inside(pose, shape, zo), new.inside(pose, shape, zones)
    n += 1
    if a != b:
        bad += 1
        if bad < 6: print("inside", pose, a, b)
    kappa = float(rng.choice((0.0, 0.1, -0.1, 0.25)))
    d = int(rng.choice((1, -1)))
    a, b = old.first_contact(pose, kappa, d, 1.0, 0.02, shape, zo), new.first_contact(pose, kappa, d, 1.0, 0.02, shape, zones)
    n += 1
    if a != b:
        bad += 1
        if bad < 6: print("contact", pose, kappa, d, a, b)
print("compared", n, "different", bad)
