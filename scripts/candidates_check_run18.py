import math
from pathlib import Path
import sys
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
import verify_phase4_goal as probe

p = Path("/home/toto/linobot2_sim/log/g4-nav-20261007-073925-03yJlJ")
with open(p / "fortress_map_nav.yaml") as f:
    meta = yaml.safe_load(f)
ox, oy = meta["origin"][:2]
res = meta["resolution"]

with open(p / "fortress_map_nav.pgm", "rb") as f:
    header = []
    while len(header) < 4:
        line = f.readline().strip()
        if not line or line.startswith(b"#"):
            continue
        header.extend(line.split())
    w, h = int(header[1]), int(header[2])
    data = list(f.read())

map_meta = {
    "width": w,
    "height": h,
    "resolution": res,
    "origin": [ox, oy, 0.0],
    "frame_id": "map",
    "known_cells": sum(v != 127 for v in data),
    "occupied_cells": sum(v == 0 for v in data),
}

grid_data = []
for r in range(h):
    for c in range(w):
        pgm_val = data[(h - 1 - r) * w + c]
        if pgm_val == 0:
            grid_data.append(100)
        elif pgm_val == 254:
            grid_data.append(0)
        else:
            grid_data.append(-1)

start = (0.49, 0.0)
test_candidates = tuple(
    (x, y)
    for x in (2.85, 2.80, 2.75, 2.70)
    for y in (-1.10, -1.05, -1.00, -0.95, -0.90, -0.84, -0.79, -0.74, -0.70)
)
print("Testing candidates...")
valid = []
for x, y in test_candidates:
    occ = probe.occupancy_at(map_meta, grid_data, x, y)
    clr = probe.obstacle_clearance(x, y)
    safe_path = probe.has_known_safe_grid_path(map_meta, grid_data, start, (x, y))
    if occ == 0 and safe_path and clr >= 0.15:
        valid.append((x, y, clr))
        print(f"VALID: x={x:.2f}, y={y:.2f} -> occ={occ}, clr={clr:.3f}m, path={safe_path}")

print(f"Total valid candidates: {len(valid)}")
