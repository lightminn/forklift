"""Achieved curvature and the distance from the path from slam.control's truth columns (t, est x y yaw, truth x y yaw)."""
import glob
import json
import math

B = "/home/projects/forklift/test_runs/near_freq/nf_46d8929d"
run = f"{B}/O0_random_m_s100/live/epoch_1/run"
rows = []
for name in sorted(glob.glob(run + "/spool/slam.control/*.jsonl")):
    rows += [json.loads(line) for line in open(name)]
rows.sort(key=lambda r: r[0])
print("rows", len(rows), rows[-1][0])
tf = json.load(open(run + "/missions/m2/records.json"))["state"]["tracking_failure"]
poses = tf["path"]["poses"]


def unwrap(values):
    out = [values[0]]
    for v in values[1:]:
        out.append(out[-1] + ((v - out[-1] + math.pi) % (2 * math.pi) - math.pi))
    return out


for t0, t1 in ((232.0, 246.0), (250.0, 256.0)):
    w = [r for r in rows if t0 <= r[0] <= t1]
    for label, c in (("truth", 4), ("estimate", 1)):
        s = sum(math.dist(p[c:c + 2], q[c:c + 2]) for p, q in zip(w, w[1:]))
        yaw = unwrap([r[c + 2] for r in w])
        print((t0, t1), label, "curvature", round((yaw[-1] - yaw[0]) / s, 4), "arc", round(s, 3))
for t in (232, 240, 246, 248, 250, 252, 254, 256, 256.5):
    r = min(rows, key=lambda x: abs(x[0] - t))
    out = []
    for c in (1, 4):
        d, j = min((math.hypot(p[0] - r[c], p[1] - r[c + 1]), i) for i, p in enumerate(poses))
        out.append((round(d, 3), j, round(((r[c + 2] - poses[j][2] + math.pi) % (2 * math.pi)) - math.pi, 3)))
    print(t, "est (dist, index, yaw err)", out[0], "truth", out[1], "est-truth", round(math.hypot(r[1] - r[4], r[2] - r[5]), 3))
