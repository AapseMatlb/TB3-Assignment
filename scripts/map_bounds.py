# scripts/map_bounds.py
import os, sys, yaml
def main(p, sx=None, sy=None, gx=None, gy=None):
    with open(p, 'r', encoding='utf-8') as f:
        info = yaml.safe_load(f)
    res = float(info.get('resolution', 0.05))
    ox, oy, _ = info.get('origin', [0.0, 0.0, 0.0])
    image = info['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(p), image)
    # get width/height from PGM header
    with open(image, 'rb') as f:
        magic = f.readline().strip()
        def non_comment():
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            return line
        w, h = [int(x) for x in non_comment().split()]
    x_min, x_max = ox, ox + w*res
    y_min, y_max = oy, oy + h*res
    print(f"Map: {p}")
    print(f"resolution: {res}, size: {w}x{h} px")
    print(f"origin: ({ox}, {oy}), world X∈[{x_min}, {x_max}], Y∈[{y_min}, {y_max}]")
    if sx is not None:
        okS = (x_min <= sx <= x_max) and (y_min <= sy <= y_max)
        okG = (x_min <= gx <= x_max) and (y_min <= gy <= y_max)
        print(f"start=({sx},{sy}) in-bounds? {okS}")
        print(f"goal =({gx},{gy}) in-bounds? {okG}")

if __name__ == "__main__":
    if len(sys.argv) not in (2,6):
        print("Usage:\n  python scripts\\map_bounds.py <map.yaml>\n  python scripts\\map_bounds.py <map.yaml> sx sy gx gy")
        sys.exit(2)
    if len(sys.argv)==2:
        main(sys.argv[1])
    else:
        _, p, sx, sy, gx, gy = sys.argv
        main(p, float(sx), float(sy), float(gx), float(gy))
