"""Exact equality of the new inside/first_contact and the old module on the recorded zones and arcs: python equiv.py OLD.py TRACE..."""
import importlib.util, math, pickle, sys, time
spec = importlib.util.spec_from_file_location("old_permit", sys.argv[1]); old = importlib.util.module_from_spec(spec); sys.modules["old_permit"] = old; spec.loader.exec_module(old)
from forklift_core.safety import pallet_permit as new
n = bad = 0; t_old = t_new = 0.0
for f in sys.argv[2:]:
    rows = []
    with open(f, "rb") as h:
        try:
            while True: rows.append(pickle.load(h))
        except EOFError: pass
    for r in rows:
        zones = r["zones"]; zo = [old.Zone(z.zone_id, z.x_m, z.y_m, z.length_m, z.width_m, z.yaw_rad) for z in zones]
        t = time.perf_counter(); a = old.inside(r["pose"], r["fp"], zo); t_old += time.perf_counter() - t
        t = time.perf_counter(); b = new.inside(r["pose"], r["fp"], zones); t_new += time.perf_counter() - t
        n += 1; bad += a != b
        for k in r["curvatures"]:
            for d in (r["direction"], -r["direction"]):
                for L in (0.3, 0.6, 1.2):
                    t = time.perf_counter(); a = old.first_contact(r["pose"], k, d, L, r["step_m"], r["fp"], zo); t_old += time.perf_counter() - t
                    t = time.perf_counter(); b = new.first_contact(r["pose"], k, d, L, r["step_m"], r["fp"], zones); t_new += time.perf_counter() - t
                    n += 1
                    if a != b:
                        bad += 1; print("DIFF", f, k, d, L, a, b)
print("compared", n, "different", bad, "old s", round(t_old, 2), "new s", round(t_new, 2))
