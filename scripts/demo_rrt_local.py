#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
demo_rrt_local.py — Offline RRT path demo (no ROS required)

Windows PowerShell:
  python scripts\demo_rrt_local.py --map maps_demo\office.yaml --start 0.5 0.5 --goal 2.0 -1.5 --inflate 2

Linux/macOS:
  python scripts/demo_rrt_local.py --map maps_demo/office.yaml --start 0.5 0.5 --goal 2.0 -1.5 --inflate 2

Requires:
  pip install numpy pyyaml matplotlib
"""

import os
import math
import random
import argparse
from collections import deque

import numpy as np
import matplotlib.pyplot as plt
from yaml import safe_load


# ---------------------------- Map loading ----------------------------

def _read_pgm(path):
    """Return (array[h,w] uint8, maxval, magic). Supports P2 (ASCII) and P5 (binary)."""
    with open(path, 'rb') as f:
        magic = f.readline().strip()

        def _non_comment():
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            return line

        # width/height
        wh = _non_comment().strip()
        while not wh:
            wh = _non_comment().strip()
        w, h = [int(x) for x in wh.split()]

        # maxval
        maxval = int(_non_comment().strip())

        if magic == b'P5':
            data = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)
            return data, maxval, 'P5'
        elif magic == b'P2':
            rest = f.read().decode('ascii').split()
            vals = np.array([int(v) for v in rest[:w * h]], dtype=np.uint16).reshape(h, w)
            if maxval != 255:
                vals = (vals.astype(np.float32) * (255.0 / maxval)).astype(np.uint8)
            else:
                vals = vals.astype(np.uint8)
            return vals, 255, 'P2'
        else:
            raise ValueError(f"Unsupported PGM magic: {magic!r} in {path}")


def load_map_from_yaml(yaml_path):
    """Load map info and return (occ_bool[h,w], meta_dict)."""
    with open(yaml_path, 'r', encoding='utf-8') as f:
        info = safe_load(f)

    image_path = info['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)

    gray, maxval, magic = _read_pgm(image_path)

    res = float(info.get('resolution', 0.05))
    origin = list(info.get('origin', [0.0, 0.0, 0.0]))
    negate = int(info.get('negate', 0))
    occ_th = float(info.get('occupied_thresh', 0.65))
    free_th = float(info.get('free_thresh', 0.196))  # not used directly, kept for completeness

    # Normalize to [0,1]
    g = gray.astype(np.float32) / 255.0
    # Map server convention:
    # negate==0  => black=occupied => occ = 1 - g
    # negate==1  => white=occupied => occ = g
    occ_prob = (1.0 - g) if negate == 0 else g
    occ = occ_prob > occ_th  # boolean occupied grid

    meta = {
        'resolution': res,
        'origin': origin,           # [ox, oy, yaw]
        'occupied_thresh': occ_th,
        'free_thresh': free_th,
        'negate': negate,
        'image_path': image_path,
        'size': (gray.shape[1], gray.shape[0])  # (w, h)
    }
    return occ, meta


# ---------------------------- Utilities ----------------------------

def inflate_obstacles(occ, radius_px):
    """Square inflation in pixel space (cheap but effective)."""
    if radius_px <= 0:
        return occ.copy()
    h, w = occ.shape
    out = occ.copy()
    r = int(radius_px)
    ys, xs = np.where(occ)
    for y, x in zip(ys, xs):
        y0, y1 = max(0, y - r), min(h, y + r + 1)
        x0, x1 = max(0, x - r), min(w, x + r + 1)
        out[y0:y1, x0:x1] = True
    return out


def world_to_px(meta, x, y):
    ox, oy, _ = meta['origin']
    res = meta['resolution']
    return int((x - ox) / res), int((y - oy) / res)


def px_to_world(meta, px, py):
    ox, oy, _ = meta['origin']
    res = meta['resolution']
    return (ox + px * res), (oy + py * res)


def segment_collision_free(occ, a, b):
    """Bresenham-like check along segment a(px,py)->b(px,py)."""
    x0, y0 = a
    x1, y1 = b
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    h, w = occ.shape
    while True:
        if x < 0 or y < 0 or x >= w or y >= h or occ[y, x]:
            return False
        if x == x1 and y == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy
    return True


def nearest_free(occ, px):
    """Return nearest free (x,y) pixel to px=(x,y), BFS over 4-neighbors."""
    w = occ.shape[1]
    h = occ.shape[0]
    q = deque([px])
    seen = {px}
    while q:
        x, y = q.popleft()
        if 0 <= x < w and 0 <= y < h and not occ[y, x]:
            return (x, y)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and (nx, ny) not in seen:
                seen.add((nx, ny))
                q.append((nx, ny))
    return None


# ---------------------------- RRT ----------------------------

def rrt_plan(occ, start_px, goal_px, step=5, max_iters=5000, goal_bias=0.05, rng=None):
    """Basic RRT in pixel space. Returns list of pixel coords from start to goal (inclusive) or None."""
    if rng is None:
        rng = random.Random()

    H, W = occ.shape
    nodes = [start_px]
    parent = {start_px: None}

    for _ in range(max_iters):
        # occasional goal-bias sampling
        sample = goal_px if (rng.random() < goal_bias) else (rng.randrange(0, W), rng.randrange(0, H))

        # nearest
        sx, sy = sample
        nx, ny = min(nodes, key=lambda n: (n[0] - sx) ** 2 + (n[1] - sy) ** 2)

        # steer
        dx, dy = sx - nx, sy - ny
        d = math.hypot(dx, dy)
        if d == 0:
            continue
        ux, uy = dx / d, dy / d
        new = (int(round(nx + ux * step)), int(round(ny + uy * step)))

        # bounds & collision
        if not (0 <= new[0] < W and 0 <= new[1] < H):
            continue
        if not segment_collision_free(occ, (nx, ny), new):
            continue

        nodes.append(new)
        parent[new] = (nx, ny)

        # try connecting to goal directly
        if segment_collision_free(occ, new, goal_px) and math.hypot(goal_px[0] - new[0], goal_px[1] - new[1]) <= step:
            parent[goal_px] = new
            # reconstruct
            path = []
            cur = goal_px
            while cur is not None:
                path.append(cur)
                cur = parent.get(cur)
            path.reverse()
            return path

    return None


# ---------------------------- Main ----------------------------

def main():
    ap = argparse.ArgumentParser(description="Offline RRT on a ROS map (YAML+PGM). Saves artifacts/rrt_path_example.png")
    ap.add_argument('--map', required=True, help='Path to map YAML (e.g., maps_demo/office.yaml)')
    ap.add_argument('--start', nargs=2, type=float, default=[0.5, 0.5], metavar=('SX', 'SY'),
                    help='Start (meters) in map frame, default 0.5 0.5')
    ap.add_argument('--goal', nargs=2, type=float, default=[2.0, -1.5], metavar=('GX', 'GY'),
                    help='Goal (meters) in map frame, default 2.0 -1.5')
    ap.add_argument('--inflate', type=int, default=2, help='Obstacle inflation radius in pixels (default 2)')
    ap.add_argument('--step', type=int, default=5, help='RRT step size in pixels (default 5)')
    ap.add_argument('--max-iters', type=int, default=5000, help='RRT max iterations (default 5000)')
    ap.add_argument('--goal-bias', type=float, default=0.05, help='Goal sampling probability (default 0.05)')
    ap.add_argument('--seed', type=int, default=42, help='RNG seed for reproducibility (default 42)')
    args = ap.parse_args()

    # Load map
    occ, meta = load_map_from_yaml(args.map)
    occ_inf = inflate_obstacles(occ, args.inflate)

    # Convert start/goal to pixels
    start_px = world_to_px(meta, args.start[0], args.start[1])
    goal_px = world_to_px(meta, args.goal[0], args.goal[1])

    # Validate & snap to nearest free if blocked
    H, W = occ_inf.shape

    def is_free(px):
        x, y = px
        return 0 <= x < W and 0 <= y < H and not occ_inf[y, x]

    snapped = False
    if not is_free(start_px):
        s2 = nearest_free(occ_inf, start_px)
        if s2:
            print(f"Start {start_px} blocked -> snapping to {s2}")
            start_px = s2
            snapped = True
    if not is_free(goal_px):
        g2 = nearest_free(occ_inf, goal_px)
        if g2:
            print(f"Goal  {goal_px} blocked -> snapping to {g2}")
            goal_px = g2
            snapped = True

    # Plan
    rng = random.Random(args.seed)
    path_px = None
    if is_free(start_px) and is_free(goal_px):
        path_px = rrt_plan(
            occ_inf, start_px, goal_px,
            step=args.step, max_iters=args.max_iters, goal_bias=args.goal_bias, rng=rng
        )
    else:
        print("Start or goal is invalid (in obstacle or out of bounds). Try different coordinates or lower --inflate.")

    # Prepare plot (in world coordinates)
    os.makedirs('artifacts', exist_ok=True)
    fig = plt.figure(figsize=(5, 5))
    ax = plt.gca()

    # Draw free=white, occupied=black using world extents
    ox, oy, _ = meta['origin']
    res = meta['resolution']
    Wpx, Hpx = meta['size']
    extent = (ox, ox + Wpx * res, oy + Hpx * res, oy)  # left, right, bottom, top (origin='upper')
    img = 1 - occ_inf.astype(np.uint8)  # show occupied as black
    ax.imshow(img, cmap='gray', origin='upper', extent=extent, vmin=0, vmax=1)
    ax.set_title('RRT Path on Demo Map')
    ax.set_aspect('equal', adjustable='box')
    ax.axis('off')

    # Draw path in world coords
    if path_px:
        path_world = [px_to_world(meta, px, py) for (px, py) in path_px]
        xs = [p[0] for p in path_world]
        ys = [p[1] for p in path_world]
        ax.plot(xs, ys, linewidth=3, zorder=3)
    else:
        ax.text(0.5, 0.05, 'NO PATH', transform=ax.transAxes, ha='center', va='center')

    # Mark start & goal (original requested positions in world coords)
    ax.scatter([args.start[0]], [args.start[1]], s=60, marker='o', zorder=4)
    ax.scatter([args.goal[0]], [args.goal[1]], s=80, marker='*', zorder=4)

    out = os.path.join('artifacts', 'rrt_path_example.png')
    plt.savefig(out, bbox_inches='tight', dpi=150)
    print(f"Saved {out}")


if __name__ == '__main__':
    main()
