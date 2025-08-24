#!/usr/bin/env python3
import argparse, os, json, math, random
import numpy as np
import yaml
import matplotlib.pyplot as plt

def load_map(yaml_path):
    with open(yaml_path, 'r') as f:
        info = yaml.safe_load(f)
    image_path = os.path.join(os.path.dirname(yaml_path), info['image'])
    # load P2 PGM
    with open(image_path, 'r') as img:
        assert img.readline().strip() == 'P2'
        line = img.readline().strip()
        while line.startswith('#'):
            line = img.readline().strip()
        w,h = map(int, line.split())
        maxval = int(img.readline().strip())
        vals = []
        for line in img:
            vals.extend([int(v) for v in line.strip().split() if v])
        arr = np.array(vals, dtype=np.uint8).reshape(h, w)
    meta = {
        'resolution': float(info['resolution']),
        'origin': info['origin'],
    }
    occ = arr < 128
    return occ, meta

def inflate(occ, r=2):
    if r<=0: return occ.copy()
    import numpy as np
    h,w = occ.shape
    out = occ.copy()
    for y in range(h):
        for x in range(w):
            if occ[y,x]:
                y0 = max(0, y-r); y1 = min(h, y+r+1)
                x0 = max(0, x-r); x1 = min(w, x+r+1)
                out[y0:y1, x0:x1] = True
    return out

def world_to_px(meta, x, y):
    ox, oy, _ = meta['origin']
    res = meta['resolution']
    return int((x-ox)/res), int((y-oy)/res)

def rrt(occ, start_px, goal_px, step=5, max_iters=5000, goal_bias=0.05):
    def collision_free(a,b):
        (x0,y0),(x1,y1) = a,b
        n = max(abs(x1-x0), abs(y1-y0))+1
        for t in range(n):
            x = int(round(x0 + (x1-x0)*t/(n-1)))
            y = int(round(y0 + (y1-y0)*t/(n-1)))
            if x<0 or y<0 or y>=occ.shape[0] or x>=occ.shape[1] or occ[y,x]:
                return False
        return True

    nodes = [start_px]
    parents = {start_px: None}
    for i in range(max_iters):
        sample = goal_px if (random.random()<goal_bias) else (random.randrange(occ.shape[1]), random.randrange(occ.shape[0]))
        nx, ny = min(nodes, key=lambda n: (n[0]-sample[0])**2+(n[1]-sample[1])**2)
        dx, dy = sample[0]-nx, sample[1]-ny
        d = math.hypot(dx,dy)
        if d==0: continue
        ux, uy = dx/d, dy/d
        new = (int(round(nx+ux*step)), int(round(ny+uy*step)))
        if new[0]<0 or new[1]<0 or new[1]>=occ.shape[0] or new[0]>=occ.shape[1]: continue
        if not collision_free((nx,ny), new): continue
        nodes.append(new); parents[new]=(nx,ny)
        if collision_free(new, goal_px) and math.hypot(goal_px[0]-new[0], goal_px[1]-new[1])<=step:
            parents[goal_px]=new
            path=[]; cur=goal_px
            while cur is not None:
                path.append(cur); cur=parents.get(cur)
            path.reverse()
            return path, nodes
    return None, nodes

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--map", default="maps_demo/office.yaml")
    ap.add_argument("--start", nargs=2, type=float, default=[0.5,0.5])
    ap.add_argument("--goal", nargs=2, type=float, default=[8.0, 8.0])
    ap.add_argument("--inflate", type=int, default=2)
    args = ap.parse_args()

    occ, meta = load_map(args.map)
    occ_inf = inflate(occ, args.inflate)
    spx = world_to_px(meta, args.start[0], args.start[1])
    gpx = world_to_px(meta, args.goal[0], args.goal[1])
    path, nodes = rrt(occ_inf, spx, gpx, step=5, max_iters=5000, goal_bias=0.05)

    plt.figure(figsize=(6,6))
    plt.imshow(~occ_inf, cmap="gray", origin="lower")  # free=white
    xs=[n[0] for n in nodes]; ys=[n[1] for n in nodes]
    plt.scatter(xs, ys, s=1)
    if path:
        px=[p[0] for p in path]; py=[p[1] for p in path]
        plt.plot(px, py, linewidth=2)
    plt.title("RRT Path on Demo Map")
    plt.axis("equal"); plt.axis("off")
    os.makedirs("artifacts", exist_ok=True)
    out = os.path.join("artifacts","rrt_path_example.png")
    plt.savefig(out, bbox_inches="tight")
    print("Saved", out)

if __name__ == "__main__":
    main()
