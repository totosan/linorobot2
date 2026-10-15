import sys
import yaml

with open("/home/toto/linobot2_sim/log/g4-nav-20261006-205629-ylgV2V/fortress_map_nav.yaml") as f:
    meta = yaml.safe_load(f)
ox, oy = meta["origin"][:2]
res = meta["resolution"]

with open("/home/toto/linobot2_sim/log/g4-nav-20261006-205629-ylgV2V/fortress_map_nav.pgm", "rb") as f:
    header = []
    while len(header) < 4:
        line = f.readline().strip()
        if not line or line.startswith(b"#"):
            continue
        header.extend(line.split())
    width = int(header[1])
    height = int(header[2])
    data = list(f.read())

occ_x = []
occ_y = []
for r in range(height):
    for c in range(width):
        val = data[r * width + c]
        if val == 0:
            map_r = height - 1 - r
            x = ox + (c + 0.5) * res
            y = oy + (map_r + 0.5) * res
            occ_x.append(x)
            occ_y.append(y)

print("OCCUPIED COUNT:", len(occ_x))
print("MIN X:", min(occ_x), "MAX X:", max(occ_x))
print("MIN Y:", min(occ_y), "MAX Y:", max(occ_y))

cells = list(zip(occ_x, occ_y))
west = [c for c in cells if abs(c[0] - 1.50) < 0.06]
south = [c for c in cells if abs(c[1] - (-0.50)) < 0.06]
east = [c for c in cells if abs(c[0] - 2.50) < 0.06]
north = [c for c in cells if abs(c[1] - 0.50) < 0.06]

print("West face cells (x ~ 1.50):", len(west), [(round(c[0], 3), round(c[1], 3)) for c in west[:3]])
print("South face cells (y ~ -0.50):", len(south), [(round(c[0], 3), round(c[1], 3)) for c in south[:3]])
print("East face cells (x ~ 2.50):", len(east), [(round(c[0], 3), round(c[1], 3)) for c in east[:3]])
if east:
    east_y = [c[1] for c in east]
    print("East face min Y:", min(east_y), "max Y:", max(east_y))
print("North face cells (y ~ 0.50):", len(north), [(round(c[0], 3), round(c[1], 3)) for c in north[:3]])
